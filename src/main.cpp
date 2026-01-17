// src/main.cpp
// ESP32-C3 IGC Logger (NO WiFi) + On-board LED status (ACTIVE-LOW fixed)
// - BN-180 GPS (UART)
// - BMP180 baro (I2C)
// - microSD (SPI)
// - Start/stop recording with BOOT button
// - Start/stop recording with RC receiver PWM channel (optional)
//
// LOG RATE: 5 Hz (B-record every 200 ms)
//
// Warm-start / dropouts:
// - Recording start requires GPS time+date valid.
// - If GPS fix drops, we KEEP logging B-records with fix flag 'V'.
// - If position is missing, we reuse last known lat/lon (still 'V').
//
// BARO altitude / OLC compatibility (IMPORTANT):
// - IGC "pressure altitude" field now logs ABSOLUTE pressure altitude (QNE, 1013.25 hPa).
//   This value may be negative near sea level on high-pressure days, and that's OK.
// - We write pressure altitude with SIGN (e.g. -0045) to avoid clamping to 00000.
// - We optionally add an L-record "AGL" altitude relative to the start baseline (clamped >=0).
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
#include <WiFi.h>   // for MAC (no WiFi used)

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

// Baro baseline averaging (for AGL L-record only)
static const int BARO_BASE_SAMPLES = 10;
static const int BARO_SAMPLE_DELAY_MS = 20;

// If GPS has no location yet, we still log B records using last known coords.
// If we have never had a coord, use 0/0 (still marked V).
static const double FALLBACK_LAT = 0.0;
static const double FALLBACK_LON = 0.0;

// Device ID / description
static const char* IGC_MANUFACTURER_ID = "AESP32C3";
static const char* LOGGER_NAME         = "ESP32-C3-RC-IGC";
static const char* LOGGER_VERSION      = "5HZ-PWM-QNE-SIGNED-VREC";

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

// Baro baseline (for AGL reference only)
bool baroBaseSet = false;
int  baroBaseAlt = 0;  // absolute pressure altitude at start (QNE)

// Last known GPS data to survive dropouts
bool   haveLastPos = false;
double lastLat = FALLBACK_LAT;
double lastLon = FALLBACK_LON;
int    lastGpsAltM = 0;

// ---- Helpers ----
static String two(int v) { return (v < 10) ? "0" + String(v) : String(v); }

static bool gpsTimeOK() { return gps.time.isValid() && gps.date.isValid(); }

static bool gpsFixOK() {
  if (!gps.location.isValid()) return false;
  if (gps.location.age() > 3000) return false;
  if (gps.satellites.isValid() && gps.satellites.value() < 4) return false;
  return true;
}

// Pressure altitude from pressure in Pa (ISA/QNE, 1013.25 hPa)
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

// Signed 5-char altitude for PRESSURE altitude (QNE). Examples: 00123, -0045
static String igcAlt5Signed(int meters) {
  if (meters > 99999) meters = 99999;
  if (meters < -9999) meters = -9999; // "-9999" is 5 chars

  char buf[8];
  if (meters < 0) snprintf(buf, sizeof(buf), "-%04d", -meters);
  else           snprintf(buf, sizeof(buf), "%05d", meters);
  return String(buf);
}

// Unsigned 5-char altitude for GPS altitude. Negative/invalid -> 00000
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

// Unique filename even if multiple starts in same second
static String makeIgcFilenameUTC() {
  char base[48];
  snprintf(base, sizeof(base), "/IGC/%04d%02d%02d_%02d%02d%02d",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second());

  String fn = String(base) + ".IGC";
  if (!SD.exists(fn.c_str())) return fn;

  for (int i = 1; i <= 99; i++) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s_%02d.IGC", base, i);
    if (!SD.exists(buf)) return String(buf);
  }

  return String(base) + "_XX.IGC";
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

  igcFile.print(IGC_MANUFACTURER_ID);
  igcFile.print("\r\n");

  igcFile.printf("HFDTE%02d%02d%02d\r\n", dd, mm, yy);
  igcFile.print("HFDTM100GPSDATUM:WGS-1984\r\n");

  igcFile.print("HFRFWFIRMWARE:");
  igcFile.print(LOGGER_NAME);
  igcFile.print("-");
  igcFile.print(LOGGER_VERSION);
  igcFile.print("\r\n");

  igcFile.print("HFRHWHARDWARE:ESP32-C3+BN180+SD+BMP180\r\n");

  igcFile.print("HFGIDLOGGERID:");
  igcFile.print(macNoColonsLower());
  igcFile.print("\r\n");

  igcFile.print("HFGPS:BN-180 (u-blox)\r\n");
  igcFile.print("HFPRS:BMP180\r\n");

  igcFile.print("HFPLTPILOTINCHARGE:RC\r\n");
  igcFile.print("HFGTYGLIDERTYPE:RCGLIDER\r\n");
  igcFile.print("HFGIDGLIDERID:N/A\r\n");

  igcFile.flush();
}

static void setBaroBaselineIfPossible() {
  if (!baroOK) { baroBaseSet = false; return; }

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

  if (n <= 0) { baroBaseSet = false; return; }

  baroBaseAlt = (int)lround((double)sum / (double)n);
  baroBaseSet = true;

  Serial.print("Baro baseline set (avg ");
  Serial.print(n);
  Serial.print(" samples): ");
  Serial.print(baroBaseAlt);
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

  haveLastPos = false;
  lastLat = FALLBACK_LAT;
  lastLon = FALLBACK_LON;
  lastGpsAltM = 0;

  writeIgcHeaders();
  setBaroBaselineIfPossible();

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

  Serial.println("BOOT LOGGER STARTED (5Hz, QNE signed pressure alt, V-record dropouts)");

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

  // Update last known position if we have one
  if (gps.location.isValid() && gps.location.age() <= 3000) {
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();
    haveLastPos = true;
  }
  if (gps.altitude.isValid()) {
    lastGpsAltM = (int)lround(gps.altitude.meters());
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

    // Debug altitudes
    float gpsAlt = gps.altitude.isValid() ? gps.altitude.meters() : NAN;

    float baroAbsAlt = NAN;
    float baroRelAlt = NAN;
    if (baroOK) {
      float p = bmp180.readPressure();
      baroAbsAlt = (float)pressureAltMetersFromPa(p);
      if (baroBaseSet) {
        baroRelAlt = baroAbsAlt - (float)baroBaseAlt;
        if (baroRelAlt < 0) baroRelAlt = 0; // keep AGL non-negative for display
      }
    }

    Serial.print("ALT baroQNE=");
    if (isfinite(baroAbsAlt)) Serial.print(baroAbsAlt, 1); else Serial.print("N/A");
    Serial.print(" m  baroAGL=");
    if (isfinite(baroRelAlt)) Serial.print(baroRelAlt, 1); else Serial.print("N/A");
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

    // Use last known position if current invalid
    double useLat = haveLastPos ? lastLat : FALLBACK_LAT;
    double useLon = haveLastPos ? lastLon : FALLBACK_LON;

    String lat = igcLat(useLat);
    String lon = igcLon(useLon);

    // If we have a good fix, 'A', otherwise 'V'
    char fix = gpsFixOK() ? 'A' : 'V';

    int gpsAltM = gps.altitude.isValid() ? (int)lround(gps.altitude.meters()) : lastGpsAltM;

    // ABSOLUTE pressure altitude (QNE) for IGC pressure altitude field
    int pAltQNE = 0;
    if (baroOK) {
      float p = bmp180.readPressure();
      pAltQNE = pressureAltMetersFromPa(p); // may be negative
    } else {
      // If no baro, fall back to GPS altitude (still valid, but less ideal)
      pAltQNE = gpsAltM;
    }

    // Standard B-record
    String line = "B" + two(hh) + two(mi) + two(ss) + lat + lon + String(fix)
                + igcAlt5Signed(pAltQNE)         // pressure alt (signed)
                + igcAlt5Unsigned(gpsAltM)       // gps alt (unsigned)
                + "\r\n";
    igcFile.print(line);

    // Optional L-record with AGL (relative to baseline), clamped >=0
    if (baroOK && baroBaseSet) {
      int agl = pAltQNE - baroBaseAlt;
      if (agl < 0) agl = 0;
      // Example: LAGL00012  (meters)
      char lbuf[20];
      snprintf(lbuf, sizeof(lbuf), "LAGL%05d\r\n", agl);
      igcFile.print(lbuf);
    }

    uint32_t now = millis();
    if (now - lastFlushMs >= SD_FLUSH_MS) {
      lastFlushMs = now;
      igcFile.flush();
    }
  }
}
