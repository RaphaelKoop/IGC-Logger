// src/main.cpp
// ESP32-C3 IGC Logger (NO WiFi) + On-board LED status (ACTIVE-LOW)
// - BN-180 GPS (UART)
// - BMP180 baro (I2C)
// - microSD (SPI)
// - Start/stop recording with BOOT button
// - Start/stop recording with RC receiver PWM channel (optional)
//
// LOG RATE: 5 Hz (B-record every 200 ms)
//
// IGC FIX (B) RECORDS — IMPORTANT (per IGC spec behavior):
// - Pressure altitude field = ABSOLUTE pressure altitude referenced to ISA/QNE 1013.25 hPa (NOT AGL).
//   -> Can be negative (and must be written with '-' in the 5-char field).
// - GNSS altitude field = GPS altitude above ellipsoid.
//   -> If GNSS altitude is not valid, write 00000.
// - Fix validity flag:
//   -> 'A' only when we have a proper 3D fix (lat/lon valid and fresh, sats>=4, plus altitude valid).
//   -> Otherwise 'V'.
//
// AGL:
// - We still compute AGL (relative to baro baseline at start) and write it as an extra L record: "LAGLxxxxx".
// - AGL is clamped to >= 0 (so you will not get negative AGL anymore).
//
// SD:
// - Writes at 5 Hz, flush once per second.
//
// LED patterns (single LED):
// - SD missing/error:   ON 2s, OFF 2s, repeat
// - GPS time NOT OK:    fast blink (~5 Hz)
// - GPS time OK no fix: slow blink (1 Hz)
// - GPS fix OK:         solid ON (ready)
// - RECORDING active:   double blink per second

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>
#include <WiFi.h>   // for MAC read (no WiFi used)

static const int PIN_GPS_RX = 20;   // ESP RX  <- GPS TX
static const int PIN_GPS_TX = 21;   // ESP TX  -> GPS RX

static const int PIN_SD_CS   = 10;
static const int PIN_SD_MOSI = 7;
static const int PIN_SD_MISO = 2;
static const int PIN_SD_SCK  = 6;

static const int PIN_I2C_SDA = 4;
static const int PIN_I2C_SCL = 5;

static const int PIN_BOOT_BTN = 9;

// RC PWM input pin (optional)
static const int PIN_PWM_IN = 3;

// Built-in LED pin
static const int LED_PIN = 8;
static const bool LED_ACTIVE_LOW = true;

// PWM thresholds
static const int PWM_START_US = 1600;
static const int PWM_STOP_US  = 1400;
static const uint32_t PWM_TIMEOUT_MS = 500;

// Logging
static const uint32_t LOG_INTERVAL_MS = 200; // 5 Hz
static const uint32_t SD_FLUSH_MS     = 1000;

// Baro baseline averaging (for AGL only)
static const int BARO_BASE_SAMPLES = 20;
static const int BARO_SAMPLE_DELAY_MS = 20;

// If GPS has no location yet, we still log B records using last known coords.
// If we have never had a coord, use 0/0 (still marked V).
static const double FALLBACK_LAT = 0.0;
static const double FALLBACK_LON = 0.0;

// Device ID
static const char* IGC_A_RECORD       = "AESP32C3";          // A record line (your own)
static const char* LOGGER_NAME        = "ESP32-C3-RC-IGC";
static const char* LOGGER_VERSION     = "5HZ-PWM-QNE-ABSALT";

// Globals
HardwareSerial GPS(1);
TinyGPSPlus gps;
Adafruit_BMP085 bmp180;

bool baroOK = false;
bool sdOK = false;

bool recording = false;
File igcFile;

uint32_t lastLogMs = 0;
uint32_t lastFlushMs = 0;

// LED blink state
uint32_t lastLedToggle = 0;
bool ledState = false;

// PWM state
bool pwmWantsRec = false;
bool pwmSeenEver = false;
uint32_t lastPwmOkMs = 0;

// Owner
enum RecOwner : uint8_t { OWNER_NONE = 0, OWNER_BOOT = 1, OWNER_PWM = 2 };
RecOwner recOwner = OWNER_NONE;

// Baro baseline for AGL (computed from ABS QNE pressure altitude, but only used for LAGL)
bool baroBaseSet = false;
int  baroBaseAbsQneAltM = 0;

// Last known GPS position (for repeating when GPS drops, per spec guidance)
bool   haveLastPos = false;
double lastLat = FALLBACK_LAT;
double lastLon = FALLBACK_LON;

// ---- Helpers ----
static String two(int v) { return (v < 10) ? "0" + String(v) : String(v); }

static bool gpsTimeOK() { return gps.time.isValid() && gps.date.isValid(); }

// “Proper” fix for our purposes: location valid + fresh + sats>=4
static bool gpsFixOK() {
  if (!gps.location.isValid()) return false;
  if (gps.location.age() > 3000) return false;
  if (gps.satellites.isValid() && gps.satellites.value() < 4) return false;
  return true;
}

// Pressure altitude in meters, referenced to ISA sea level 1013.25 hPa (QNE)
static int pressureAltMetersFromPa(float pressurePa) {
  if (!isfinite(pressurePa) || pressurePa <= 0) return 0;
  float p_hPa = pressurePa / 100.0f;
  float alt = 44330.0f * (1.0f - powf(p_hPa / 1013.25f, 0.1903f));
  return (int)lroundf(alt);
}

// IGC coord formatting: DDMMmmmN / DDDMMmmmE
static String igcLat(double lat) {
  char hemi = (lat >= 0) ? 'N' : 'S';
  lat = fabs(lat);
  int deg = (int)lat;
  double minutes = (lat - deg) * 60.0;
  int min_int = (int)minutes;
  int min_thou = (int)lround((minutes - min_int) * 1000.0);

  if (min_thou >= 1000) { min_thou = 0; min_int++; }
  if (min_int >= 60) { min_int = 0; deg++; }

  char buf[16];
  snprintf(buf, sizeof(buf), "%02d%02d%03d%c", deg, min_int, min_thou, hemi);
  return String(buf);
}

static String igcLon(double lon) {
  char hemi = (lon >= 0) ? 'E' : 'W';
  lon = fabs(lon);
  int deg = (int)lon;
  double minutes = (lon - deg) * 60.0;
  int min_int = (int)minutes;
  int min_thou = (int)lround((minutes - min_int) * 1000.0);

  if (min_thou >= 1000) { min_thou = 0; min_int++; }
  if (min_int >= 60) { min_int = 0; deg++; }

  char buf[16];
  snprintf(buf, sizeof(buf), "%03d%02d%03d%c", deg, min_int, min_thou, hemi);
  return String(buf);
}

// IGC altitude field (5 chars)
// Positive: "00123"
// Negative: "-0123"   (minus sign replaces the leading zero; total length stays 5)
static String igcAlt5Signed(int meters) {
  if (meters > 99999) meters = 99999;
  if (meters < -9999) meters = -9999; // because "-9999" is 5 chars including '-'

  char buf[8];
  if (meters >= 0) {
    snprintf(buf, sizeof(buf), "%05d", meters);
  } else {
    snprintf(buf, sizeof(buf), "-%04d", abs(meters));
  }
  return String(buf);
}

// AGL as a 5-digit clamped non-negative (for L record only)
static String igcAlt5Unsigned(int meters) {
  if (meters < 0) meters = 0;
  if (meters > 99999) meters = 99999;
  char buf[8];
  snprintf(buf, sizeof(buf), "%05d", meters);
  return String(buf);
}

static bool ensureDir(const char *path) {
  if (SD.exists(path)) return true;
  return SD.mkdir(path);
}

static String makeIgcFilenameUTC() {
  char buf[48];
  snprintf(buf, sizeof(buf), "/IGC/%04d%02d%02d_%02d%02d%02d.IGC",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second());
  return String(buf);
}

static String macNoColonsLower() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char buf[13];
  snprintf(buf, sizeof(buf), "%02x%02x%02x%02x%02x%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

static void writeIgcHeaders() {
  int dd = gps.date.day();
  int mm = gps.date.month();
  int yy = gps.date.year() % 100;

  // A record
  igcFile.print(IGC_A_RECORD);
  igcFile.print("\r\n");

  // Required-ish header lines (kept simple but correct format)
  igcFile.printf("HFDTE%02d%02d%02d\r\n", dd, mm, yy);
  igcFile.print("HFDTM100GPSDATUM:WGS-1984\r\n");

  igcFile.print("HFRFWFIRMWAREVERSION:");
  igcFile.print(LOGGER_NAME);
  igcFile.print("-");
  igcFile.print(LOGGER_VERSION);
  igcFile.print("\r\n");

  igcFile.print("HFRHWHARDWAREVERSION:ESP32-C3+BN180+SD+BMP180\r\n");

  igcFile.print("HFFTYFRTYPE:DIY,");
  igcFile.print(LOGGER_NAME);
  igcFile.print("\r\n");

  igcFile.print("HFGIDGLIDERID:N/A\r\n");
  igcFile.print("HFPLTPILOTINCHARGE:RC\r\n");
  igcFile.print("HFGTYGLIDERTYPE:RCGLIDER\r\n");
  igcFile.print("HFCIDCOMPETITIONID:RC\r\n");
  igcFile.print("HFCCLCOMPETITIONCLASS:RC\r\n");

  igcFile.print("HFGPS:BN-180 (u-blox)\r\n");
  igcFile.print("HFPRSPRESSALTSENSOR:BMP180\r\n");

  igcFile.print("HFGIDLOGGERID:");
  igcFile.print(macNoColonsLower());
  igcFile.print("\r\n");

  igcFile.flush();
}

// Establish baseline for AGL only (average ABS QNE pressure altitude)
static void setBaroBaselineIfPossible() {
  baroBaseSet = false;
  if (!baroOK) return;

  long sum = 0;
  int n = 0;

  for (int i = 0; i < BARO_BASE_SAMPLES; i++) {
    float p = bmp180.readPressure();
    if (isfinite(p) && p > 0) {
      sum += pressureAltMetersFromPa(p);
      n++;
    }
    delay(BARO_SAMPLE_DELAY_MS);
  }

  if (n <= 0) return;

  baroBaseAbsQneAltM = (int)lround((double)sum / (double)n);
  baroBaseSet = true;

  Serial.print("Baro baseline set (avg ");
  Serial.print(n);
  Serial.print(" samples): ");
  Serial.print(baroBaseAbsQneAltM);
  Serial.println(" m (QNE)");
}

static bool startRecording(RecOwner owner) {
  if (recording) return true;
  if (!sdOK) { Serial.println("START FAIL: SD not OK"); return false; }
  if (!gpsTimeOK()) { Serial.println("START FAIL: GPS time/date not valid yet"); return false; }

  ensureDir("/IGC");
  String fn = makeIgcFilenameUTC();
  igcFile = SD.open(fn.c_str(), FILE_WRITE);
  if (!igcFile) { Serial.println("START FAIL: cannot open file"); return false; }

  // Reset “last known” values for this flight
  haveLastPos = false;
  lastLat = FALLBACK_LAT;
  lastLon = FALLBACK_LON;

  writeIgcHeaders();
  setBaroBaselineIfPossible(); // only affects LAGL output

  recording = true;
  recOwner = owner;
  lastFlushMs = millis();

  Serial.print("RECORDING STARTED (owner=");
  Serial.print(owner == OWNER_PWM ? "PWM" : "BOOT");
  Serial.print("): ");
  Serial.println(fn);
  return true;
}

static void stopRecording() {
  if (!recording) return;
  igcFile.flush();
  igcFile.close();
  recording = false;
  recOwner = OWNER_NONE;
  baroBaseSet = false;
  Serial.println("RECORDING STOPPED");
}

// BOOT debounce/toggle (active LOW)
static bool bootPressed() { return digitalRead(PIN_BOOT_BTN) == LOW; }

static void handleBootToggle() {
  static bool last = false;
  static uint32_t lastChange = 0;

  bool cur = bootPressed();
  uint32_t now = millis();

  if (cur != last && (now - lastChange) > 40) {
    lastChange = now;
    last = cur;
    if (cur) {
      if (recording) stopRecording();
      else startRecording(OWNER_BOOT);
    }
  }
}

// PWM
static bool readPwmUs(uint16_t &outUs) {
  uint32_t us = pulseIn(PIN_PWM_IN, HIGH, 25000);
  if (us < 900 || us > 2200) return false;
  outUs = (uint16_t)us;
  return true;
}

static void handlePwmControl() {
  uint16_t pwmUs;
  bool got = readPwmUs(pwmUs);

  if (got) {
    pwmSeenEver = true;
    lastPwmOkMs = millis();

    if (!pwmWantsRec && pwmUs >= PWM_START_US) pwmWantsRec = true;
    if (pwmWantsRec && pwmUs <= PWM_STOP_US)  pwmWantsRec = false;
  }

  if (pwmSeenEver && (millis() - lastPwmOkMs > PWM_TIMEOUT_MS)) {
    pwmWantsRec = false;
  }

  if (pwmWantsRec && !recording) startRecording(OWNER_PWM);
  if (!pwmWantsRec && recording && recOwner == OWNER_PWM) stopRecording();
}

// LED
static void setLED(bool on) {
  if (LED_ACTIVE_LOW) digitalWrite(LED_PIN, on ? LOW : HIGH);
  else               digitalWrite(LED_PIN, on ? HIGH : LOW);
}

static void updateLED() {
  uint32_t now = millis();

  if (!sdOK) {
    uint32_t phase = now % 4000;
    setLED(phase < 2000);
    return;
  }

  if (recording) {
    uint32_t phase = now % 1000;
    bool on = (phase < 100) || (phase > 200 && phase < 300);
    setLED(on);
    return;
  }

  if (gpsFixOK()) { setLED(true); return; }

  if (gpsTimeOK()) {
    if (now - lastLedToggle > 500) {
      lastLedToggle = now;
      ledState = !ledState;
      setLED(ledState);
    }
    return;
  }

  if (now - lastLedToggle > 100) {
    lastLedToggle = now;
    ledState = !ledState;
    setLED(ledState);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(LED_PIN, OUTPUT);
  setLED(false);

  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  pinMode(PIN_PWM_IN, INPUT);
  lastPwmOkMs = millis();

  Serial.println("BOOT LOGGER STARTED (5Hz, ABS QNE press-alt in B, GNSS alt 00000 if invalid, AGL in L record)");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  baroOK = bmp180.begin();
  Serial.println(baroOK ? "Baro: BMP180 OK" : "Baro: BMP180 NOT FOUND");

  GPS.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("GPS UART started @ 9600");

  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  sdOK = SD.begin(PIN_SD_CS);
  Serial.println(sdOK ? "SD: OK" : "SD: FAILED");

  Serial.println("Press BOOT to start/stop recording.");
  Serial.println("PWM start>=1600us, stop<=1400us, timeout=500ms (PWM stops only PWM-started recordings)");
}

void loop() {
  while (GPS.available()) gps.encode(GPS.read());

  // Update last known position if we have one (fresh)
  if (gps.location.isValid() && gps.location.age() <= 3000) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
    haveLastPos = true;
  }

  handleBootToggle();
  handlePwmControl();
  updateLED();

  // Status every 2s
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 2000) {
    lastStatus = millis();

    Serial.print("GPS time=");
    Serial.print(gpsTimeOK() ? "OK" : "NO");
    Serial.print(" fix=");
    Serial.print(gpsFixOK() ? "OK" : "NO");
    Serial.print(" sats=");
    Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : -1);

    Serial.print(" baro=");
    Serial.print(baroOK ? "OK" : "NO");
    Serial.print(" sd=");
    Serial.print(sdOK ? "OK" : "NO");
    Serial.print(" recording=");
    Serial.print(recording ? "YES" : "NO");

    Serial.print(" owner=");
    Serial.print(recOwner == OWNER_PWM ? "PWM" : (recOwner == OWNER_BOOT ? "BOOT" : "NONE"));

    Serial.print(" pwmRec=");
    Serial.println(pwmWantsRec ? "YES" : "NO");

    // Debug altitude readouts
    float baroQne = NAN;
    float agl = NAN;
    if (baroOK) {
      float p = bmp180.readPressure();
      int absQne = pressureAltMetersFromPa(p);
      baroQne = (float)absQne;
      if (baroBaseSet) {
        int rel = absQne - baroBaseAbsQneAltM;
        if (rel < 0) rel = 0;
        agl = (float)rel;
      }
    }

    float gpsAlt = (gpsFixOK() && gps.altitude.isValid()) ? gps.altitude.meters() : NAN;

    Serial.print("ALT baroQNE=");
    if (isfinite(baroQne)) Serial.print(baroQne, 1); else Serial.print("N/A");
    Serial.print(" m  baroAGL=");
    if (isfinite(agl)) Serial.print(agl, 1); else Serial.print("N/A");
    Serial.print(" m  gps=");
    if (isfinite(gpsAlt)) Serial.print(gpsAlt, 1); else Serial.print("N/A");
    Serial.println(" m");
  }

  // IGC logging at 5 Hz
  if (recording && millis() - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = millis();

    // Need GPS time to build B-record time field
    if (!gpsTimeOK()) return;

    int hh = gps.time.hour();
    int mi = gps.time.minute();
    int ss = gps.time.second();

    // Repeat last known position if current invalid/missing (spec-style behavior)
    double useLat = haveLastPos ? lastLat : FALLBACK_LAT;
    double useLon = haveLastPos ? lastLon : FALLBACK_LON;

    String lat = igcLat(useLat);
    String lon = igcLon(useLon);

    // Determine GNSS altitude validity
    bool gnssAltValid = (gpsFixOK() && gps.altitude.isValid());

    // Fix flag: only 'A' if we consider it a proper 3D fix AND GNSS altitude valid
    char fix = (gpsFixOK() && gnssAltValid) ? 'A' : 'V';

    // Pressure altitude (ABS QNE) — must be absolute ISA/QNE 1013.25, can be negative, no clamping
    int pressAltQne = 0;
    bool pressAltValid = false;
    if (baroOK) {
      float p = bmp180.readPressure();
      if (isfinite(p) && p > 0) {
        pressAltQne = pressureAltMetersFromPa(p);
        pressAltValid = true;
      }
    }
    if (!pressAltValid) pressAltQne = 0; // if no sensor, write 00000

    // GNSS altitude: if not valid, write 00000 (per spec guidance)
    int gnssAlt = 0;
    if (gnssAltValid) {
      gnssAlt = (int)lround(gps.altitude.meters());
      if (gnssAlt < 0) gnssAlt = 0;      // you *can* allow negative, but 0 makes invalid obvious near sea level
      if (gnssAlt > 99999) gnssAlt = 99999;
    } else {
      gnssAlt = 0;
    }

    // Build B record:
    // BHHMMSS + LAT(8) + LON(9) + Fix(1) + PressAlt(5) + GnssAlt(5)
    String line = "B" + two(hh) + two(mi) + two(ss) + lat + lon + String(fix)
                + igcAlt5Signed(pressAltQne)
                + igcAlt5Unsigned(gnssAlt)
                + "\r\n";
    igcFile.print(line);

    // Also write AGL as custom L-record (always >=0)
    if (pressAltValid && baroBaseSet) {
      int aglM = pressAltQne - baroBaseAbsQneAltM;
      if (aglM < 0) aglM = 0;
      igcFile.print("LAGL");
      igcFile.print(igcAlt5Unsigned(aglM));
      igcFile.print("\r\n");
    }

    uint32_t now = millis();
    if (now - lastFlushMs >= SD_FLUSH_MS) {
      lastFlushMs = now;
      igcFile.flush();
    }
  }
}
