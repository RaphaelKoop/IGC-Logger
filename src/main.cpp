// src/main.cpp
// ESP32-C3 IGC Logger (BN-180 GPS + BMP180 + microSD)
// - BOOT button start/stop
// - RC PWM start/stop (optional)
// - IGC formatting aligned to spec:
//   * A record: A + 3-char manufacturer + 3-char unique ID
//   * H records incl. HFFXA
//   * I record defines FXA + SIU extensions
//   * B record: BHHMMSS LAT LON AV PPPPP GGGGG + FXA(3) + SIU(2)
// - NO "LAGLxxxxx" records (removed)
// - Pressure altitude = ISA/QNE 1013.25 hPa from BMP180 (signed 5 chars)
// - GNSS altitude = GNSS height (meters), 00000 if invalid/2D/unstable
//
// LOG RATE: 5 Hz (200 ms)
// SD: write at 5 Hz, flush once per second
//
// LED patterns (single LED, ACTIVE-LOW):
// - SD missing/error:   ON 2s, OFF 2s, repeat
// - GPS time NOT OK:    fast blink (~5 Hz)
// - GPS time OK no fix: slow blink (1 Hz)
// - GPS fix OK:         solid ON (ready)  <-- now requires >=6 sats + stable GNSS altitude
// - RECORDING active:   double blink per second
//
// Serial status: every 2s prints time/fix/sats/baro/sd/recording/owner/pwm + altitude debug
//
// NEW: "Altitude stability gate"
// - We only claim a proper fix (and light solid LED / write 'A') when:
//   * location is valid+fresh
//   * sats >= 6
//   * altitude is valid+fresh
//   * after a short warmup period
//   * and altitude stays within a small range during a window (no big jumps)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>
#include <esp_system.h>
#include <esp_mac.h>

// ---------------- Pins ----------------
static const int PIN_GPS_RX = 20;   // ESP RX  <- GPS TX
static const int PIN_GPS_TX = 21;   // ESP TX  -> GPS RX

static const int PIN_SD_CS   = 10;
static const int PIN_SD_MOSI = 7;
static const int PIN_SD_MISO = 2;
static const int PIN_SD_SCK  = 6;

static const int PIN_I2C_SDA = 4;
static const int PIN_I2C_SCL = 5;

static const int PIN_BOOT_BTN = 9;

// Optional PWM input (set to -1 to disable)
static const int PIN_PWM_IN = 3;       // set to -1 to disable
static const int PWM_START_US = 1600;
static const int PWM_STOP_US  = 1400;
static const uint32_t PWM_TIMEOUT_MS = 500;

// Built-in LED
static const int LED_PIN = 8;
static const bool LED_ACTIVE_LOW = true;

// ---------------- Logging ----------------
static const uint32_t LOG_INTERVAL_MS = 200; // 5 Hz
static const uint32_t SD_FLUSH_MS     = 1000;

// ---------------- GNSS "solid fix" gate ----------------
// Tune these if needed:
static const int      GNSS_SATS_MIN_FOR_SOLID = 6;     // >=6 sats
static const uint32_t GNSS_WARMUP_MS          = 8000;  // wait after first plausible fix
static const uint32_t GNSS_STABLE_WIN_MS      = 4000;  // stability window
static const float    GNSS_MAX_SPREAD_M       = 8.0f;  // max (max-min) altitude spread in window

// ---------------- Globals ----------------
HardwareSerial GPS(1);
TinyGPSPlus gps;

Adafruit_BMP085 bmp180;
bool baroOK = false;

bool sdOK = false;
bool recording = false;
File igcFile;

uint32_t lastLogMs = 0;
uint32_t lastFlushMs = 0;

// Owner
enum Owner : uint8_t { OWNER_NONE = 0, OWNER_BOOT = 1, OWNER_PWM = 2 };
Owner recOwner = OWNER_NONE;

// PWM state
bool pwmWantsRec = false;
bool pwmSeenEver = false;
uint32_t lastPwmOkMs = 0;

// LED blink state (for non-recording blinking)
uint32_t lastLedToggle = 0;
bool ledState = false;

// last known position strings (for brief invalid periods)
String lastLatStr = "0000000N";
String lastLonStr = "00000000E";

// GNSS stability tracking
static bool     gnssSeenPlausibleFix = false;
static uint32_t gnssFirstFixMs = 0;
static uint32_t gnssWinStartMs = 0;
static float    gnssAltMin =  1e9f;
static float    gnssAltMax = -1e9f;

// ---------------- Helpers ----------------
static String two(int v) { return (v < 10) ? "0" + String(v) : String(v); }

static void setLED(bool on) {
  if (LED_ACTIVE_LOW) digitalWrite(LED_PIN, on ? LOW : HIGH);
  else digitalWrite(LED_PIN, on ? HIGH : LOW);
}

static bool gpsTimeOK() {
  return gps.time.isValid() && gps.date.isValid();
}

// Basic "location OK" (fresh lat/lon + some sats)
static bool gpsLocOK_basic() {
  if (!gps.location.isValid()) return false;
  if (gps.location.age() > 3000) return false;
  int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  if (sats < 4) return false;
  return true;
}

// GNSS altitude stability gate
static bool gnssAltStableOK() {
  // need fresh location
  if (!gps.location.isValid() || gps.location.age() > 3000) return false;

  // need altitude and it must be fresh
  if (!gps.altitude.isValid() || gps.altitude.age() > 3000) return false;

  int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  if (sats < GNSS_SATS_MIN_FOR_SOLID) return false;

  uint32_t now = millis();

  // start warmup timer on first plausible fix
  if (!gnssSeenPlausibleFix) {
    gnssSeenPlausibleFix = true;
    gnssFirstFixMs = now;
    gnssWinStartMs = now;
    gnssAltMin =  1e9f;
    gnssAltMax = -1e9f;
  }

  // warmup period
  if (now - gnssFirstFixMs < GNSS_WARMUP_MS) return false;

  // update min/max within sliding window
  if (now - gnssWinStartMs > GNSS_STABLE_WIN_MS) {
    gnssWinStartMs = now;
    gnssAltMin =  1e9f;
    gnssAltMax = -1e9f;
  }

  float a = (float)gps.altitude.meters();
  if (a < gnssAltMin) gnssAltMin = a;
  if (a > gnssAltMax) gnssAltMax = a;

  // stable if spread small enough
  return (gnssAltMax - gnssAltMin) <= GNSS_MAX_SPREAD_M;
}

// “Proper” fix for LED + 'A' in IGC:
// - location valid + fresh
// - sats >= 6
// - altitude valid + fresh
// - altitude stable over time
static bool gpsFixOK() {
  if (!gpsLocOK_basic()) return false;
  int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  if (sats < GNSS_SATS_MIN_FOR_SOLID) return false;
  return gnssAltStableOK();
}

// Reset GNSS stability tracker (call when starting a new recording)
static void resetGnssStability() {
  gnssSeenPlausibleFix = false;
  gnssFirstFixMs = 0;
  gnssWinStartMs = 0;
  gnssAltMin =  1e9f;
  gnssAltMax = -1e9f;
}

// ISA pressure altitude from pressure in Pa (QNE 1013.25)
static int pressureAltMetersFromPa(float pressurePa) {
  if (!isfinite(pressurePa) || pressurePa <= 0) return 0;
  float p_hPa = pressurePa / 100.0f;
  float alt = 44330.0f * (1.0f - powf(p_hPa / 1013.25f, 0.1903f));
  return (int)lroundf(alt);
}

// IGC coord formatting: DDMMmmmN / DDDMMmmmE (thousandths of minutes)
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

// GNSS altitude field: 5 digits, digits only. If invalid -> 00000.
static String igcAlt5Unsigned(int meters) {
  if (meters < 0) meters = 0;
  if (meters > 99999) meters = 99999;
  char buf[8];
  snprintf(buf, sizeof(buf), "%05d", meters);
  return String(buf);
}

// Pressure altitude field: 5 chars, may be signed: "-0123" or "00123"
static String igcAlt5Signed(int meters) {
  if (meters > 99999) meters = 99999;
  if (meters < -9999) meters = -9999; // "-9999" is 5 chars
  char buf[8];
  if (meters < 0) snprintf(buf, sizeof(buf), "-%04d", abs(meters));
  else           snprintf(buf, sizeof(buf), "%05d", meters);
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

// 3-char unique ID from MAC last 24 bits (base36)
static String make3CharId() {
  uint64_t mac = ESP.getEfuseMac();
  uint32_t x = (uint32_t)(mac & 0xFFFFFF);
  const char *abc = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  char out[4];
  out[0] = abc[(x / (36 * 36)) % 36];
  out[1] = abc[(x / 36) % 36];
  out[2] = abc[x % 36];
  out[3] = 0;
  return String(out);
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

  // A record: A + MMM + NNN (use XXX if no manufacturer code)
  String id3 = make3CharId();
  igcFile.print("AXXX");
  igcFile.print(id3);
  igcFile.print("FLIGHT:1\r\n");

  // Required-ish headers
  igcFile.printf("HFDTE%02d%02d%02d\r\n", dd, mm, yy);
  igcFile.print("HFFXA050\r\n"); // typical fix accuracy category (meters)
  igcFile.print("HFDTM100GPSDATUM:WGS-1984\r\n");

  igcFile.print("HFRFWFIRMWAREVERSION:ESP32-C3-RC-IGC-5HZ-PWM-FXA-SIU-ALTSTABLE\r\n");
  igcFile.print("HFRHWHARDWAREVERSION:ESP32-C3+BN180+SD+BMP180\r\n");
  igcFile.print("HFFTYFRTYPE:XXX,ESP32C3-LOGGER\r\n");
  igcFile.print("HFGPS:u-blox,BN-180\r\n");
  igcFile.print("HFPRSPRESSALTSENSOR:BMP180\r\n");

  igcFile.print("HFPLTPILOTINCHARGE:RC\r\n");
  igcFile.print("HFGTYGLIDERTYPE:RCGLIDER\r\n");
  igcFile.print("HFGIDGLIDERID:N/A\r\n");

  igcFile.print("HFGIDLOGGERID:");
  igcFile.print(macNoColonsLower());
  igcFile.print("\r\n");

  // I record: FXA (36-38), SIU (39-40)
  igcFile.print("I023638FXA3940SIU\r\n");

  igcFile.flush();
}

static bool startRecording(Owner owner) {
  if (recording) return true;
  if (!sdOK) { Serial.println("START FAIL: SD not OK"); return false; }
  if (!gpsTimeOK()) { Serial.println("START FAIL: GPS time not valid yet"); return false; }

  ensureDir("/IGC");
  String fn = makeIgcFilenameUTC();
  igcFile = SD.open(fn.c_str(), FILE_WRITE);
  if (!igcFile) { Serial.println("START FAIL: cannot open file"); return false; }

  // reset last-known position placeholders for this file
  lastLatStr = "0000000N";
  lastLonStr = "00000000E";

  // reset GNSS stability tracker so solid-fix starts "fresh"
  resetGnssStability();

  writeIgcHeaders();

  recording = true;
  recOwner = owner;
  lastFlushMs = millis();

  Serial.print("RECORDING STARTED (owner=");
  Serial.print(owner == OWNER_BOOT ? "BOOT" : (owner == OWNER_PWM ? "PWM" : "NONE"));
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
  Serial.println("RECORDING STOPPED");
}

// BOOT button debounce/toggle (active LOW)
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

// PWM input (optional)
static bool readPwmUs(uint16_t &outUs) {
  if (PIN_PWM_IN < 0) return false;
  uint32_t us = pulseIn(PIN_PWM_IN, HIGH, 25000); // 25ms
  if (us < 900 || us > 2200) return false;
  outUs = (uint16_t)us;
  return true;
}

static void handlePwmControl() {
  if (PIN_PWM_IN < 0) return;

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

// LED logic (now uses "solid fix" gate)
static void updateLED() {
  uint32_t now = millis();

  // SD missing/error: ON 2s OFF 2s
  if (!sdOK) {
    uint32_t phase = now % 4000;
    setLED(phase < 2000);
    return;
  }

  // RECORDING active: double blink per second
  if (recording) {
    uint32_t phase = now % 1000;
    bool on = (phase < 100) || (phase > 200 && phase < 300);
    setLED(on);
    return;
  }

  // GPS fix OK (ready): solid on (>=6 sats + stable GNSS altitude)
  if (gpsFixOK()) {
    setLED(true);
    return;
  }

  // GPS time OK, but no solid fix: slow blink 1 Hz (toggle every 500ms)
  if (gpsTimeOK()) {
    if (now - lastLedToggle > 500) {
      lastLedToggle = now;
      ledState = !ledState;
      setLED(ledState);
    }
    return;
  }

  // GPS time NOT OK: fast blink (~5 Hz) (toggle every 100ms)
  if (now - lastLedToggle > 100) {
    lastLedToggle = now;
    ledState = !ledState;
    setLED(ledState);
  }
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(LED_PIN, OUTPUT);
  setLED(false);

  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  if (PIN_PWM_IN >= 0) pinMode(PIN_PWM_IN, INPUT);
  lastPwmOkMs = millis();

  Serial.println("ESP32-C3 IGC LOGGER STARTED (5Hz, FXA+SIU, NO LAGL, QNE press-alt, GNSS alt 00000 if invalid/unstable)");
  Serial.println("LED: SD err 2s/2s, time NO fast, time OK no solid-fix slow, solid-fix OK solid, recording double blink");

  // I2C + BMP180
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  baroOK = bmp180.begin();
  Serial.println(baroOK ? "Baro: BMP180 OK" : "Baro: BMP180 NOT FOUND");

  // GPS UART
  GPS.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("GPS UART started @ 9600");

  // SD
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  sdOK = SD.begin(PIN_SD_CS);
  Serial.println(sdOK ? "SD: OK" : "SD: FAILED");

  Serial.println("Press BOOT to start/stop recording.");
  if (PIN_PWM_IN >= 0) {
    Serial.println("PWM start>=1600us, stop<=1400us, timeout=500ms (PWM stops only PWM-started recordings)");
  }
}

void loop() {
  while (GPS.available()) gps.encode(GPS.read());

  handleBootToggle();
  handlePwmControl();
  updateLED();

  // Serial status every 2s
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 2000) {
    lastStatus = millis();

    int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : -1;

    Serial.print("GPS time=");
    Serial.print(gpsTimeOK() ? "OK" : "NO");

    Serial.print(" solidFix=");
    Serial.print(gpsFixOK() ? "OK" : "NO");

    Serial.print(" sats=");
    Serial.print(sats);

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
    if (baroOK) {
      float p = bmp180.readPressure();
      int absQne = pressureAltMetersFromPa(p);
      baroQne = (float)absQne;
    }

    float gpsAlt = (gps.altitude.isValid() && gps.altitude.age() <= 3000) ? gps.altitude.meters() : NAN;

    Serial.print("ALT baroQNE=");
    if (isfinite(baroQne)) Serial.print(baroQne, 1); else Serial.print("N/A");

    Serial.print(" m  gpsRaw=");
    if (isfinite(gpsAlt)) Serial.print(gpsAlt, 1); else Serial.print("N/A");

    Serial.print(" m  altStable=");
    Serial.println(gnssAltStableOK() ? "YES" : "NO");
  }

  // Write IGC B-record at 5 Hz while recording
  if (recording && (millis() - lastLogMs >= LOG_INTERVAL_MS)) {
    lastLogMs = millis();

    // Need GPS time to timestamp records
    if (!gpsTimeOK()) return;

    int hh = gps.time.hour();
    int mi = gps.time.minute();
    int ss = gps.time.second();

    // "Solid" fix validity: A only when stable gate passes, otherwise V
    bool solidFix = gpsFixOK();
    char fix = solidFix ? 'A' : 'V';

    // Position: if current invalid, repeat last known (spec guidance)
    String latStr = lastLatStr;
    String lonStr = lastLonStr;
    if (gps.location.isValid() && gps.location.age() <= 3000) {
      latStr = igcLat(gps.location.lat());
      lonStr = igcLon(gps.location.lng());
      lastLatStr = latStr;
      lastLonStr = lonStr;
    }

    // GNSS altitude: 00000 unless solidFix (stable/3D)
    int gnssAltM = 0;
    if (solidFix) {
      gnssAltM = (int)lround(gps.altitude.meters());
      if (gnssAltM < 0) gnssAltM = 0;
      if (gnssAltM > 99999) gnssAltM = 99999;
    } else {
      gnssAltM = 0;
    }

    // Pressure altitude (QNE/ISA) from baro (can be negative); if baro missing -> 00000
    int pAltM = 0;
    if (baroOK) {
      float p = bmp180.readPressure(); // Pa
      pAltM = pressureAltMetersFromPa(p);
    }

    // FXA (Estimated Position Error): rough mapping from HDOP
    int fxa = 50;
    if (gps.hdop.isValid()) {
      double hdop = gps.hdop.hdop();
      int epe = (int)lround(hdop * 5.0);
      if (epe < 5) epe = 5;
      if (epe > 999) epe = 999;
      fxa = epe;
    }
    char fxaBuf[4];
    snprintf(fxaBuf, sizeof(fxaBuf), "%03d", fxa);

    // SIU (satellites in use)
    int siu = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
    if (siu < 0) siu = 0;
    if (siu > 99) siu = 99;
    char siuBuf[3];
    snprintf(siuBuf, sizeof(siuBuf), "%02d", siu);

    // B record (no spaces):
    // BHHMMSS + LAT(8) + LON(9) + AV(1) + PPPPP(5 signed) + GGGGG(5) + FXA(3) + SIU(2)
    String line = "B" + two(hh) + two(mi) + two(ss)
                + latStr + lonStr + String(fix)
                + igcAlt5Signed(pAltM)
                + igcAlt5Unsigned(gnssAltM)
                + String(fxaBuf)
                + String(siuBuf)
                + "\r\n";

    igcFile.print(line);

    // Flush once per second
    uint32_t now = millis();
    if (now - lastFlushMs >= SD_FLUSH_MS) {
      lastFlushMs = now;
      igcFile.flush();
    }
  }
}
