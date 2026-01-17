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
// LED patterns (single LED):
// - SD missing/error:   ON 2s, OFF 2s, repeat
// - GPS time NOT OK:    fast blink (~5 Hz)
// - GPS time OK no fix: slow blink (1 Hz)
// - GPS fix OK:         solid ON (ready)
// - RECORDING active:   double blink per second
//
// PWM control behavior (important):
// - PWM can START a recording (if configured / connected)
// - PWM can STOP a recording ONLY if PWM started it
//   (so BOOT-start won't auto-stop when no PWM is connected)
//
// BARO ALTITUDE FIX:
// - Logs BARO altitude as RELATIVE altitude (AGL-style) referenced to baro altitude at recording start.
// - GPS altitude is still logged as the GPS altitude field in IGC.

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP085.h>

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

// RC PWM input pin (connect receiver PWM signal here)
static const int PIN_PWM_IN = 3;   // PWM signal to GPIO3

// Built-in LED pin
static const int LED_PIN = 8;
static const bool LED_ACTIVE_LOW = true;

// ---------------- PWM thresholds ----------------
static const int PWM_START_US = 1600;       // start requested above this
static const int PWM_STOP_US  = 1400;       // stop requested below this
static const uint32_t PWM_TIMEOUT_MS = 500; // if signal lost -> disarm PWM

// ---------------- Logging ----------------
static const uint32_t LOG_INTERVAL_MS = 200; // 5 Hz

// ---------------- Globals ----------------
HardwareSerial GPS(1);
TinyGPSPlus gps;

Adafruit_BMP085 bmp180;
bool baroOK = false;

bool sdOK = false;
bool recording = false;
File igcFile;
uint32_t lastLogMs = 0;

// LED blink state
uint32_t lastLedToggle = 0;
bool ledState = false;

// PWM state
bool pwmWantsRec = false;
bool pwmSeenEver = false;          // becomes true once we read a valid PWM pulse
uint32_t lastPwmOkMs = 0;

// Who started the current recording?
enum RecOwner : uint8_t { OWNER_NONE = 0, OWNER_BOOT = 1, OWNER_PWM = 2 };
RecOwner recOwner = OWNER_NONE;

// Baro baseline (relative altitude)
bool baroBaseSet = false;
int  baroBaseAlt = 0;  // pressure altitude at start (meters, QNE/ISA)

// ---------------- Helpers ----------------
static String two(int v) { return (v < 10) ? "0" + String(v) : String(v); }

static bool gpsTimeOK() { return gps.time.isValid() && gps.date.isValid(); }

static bool gpsFixOK() {
  if (!gps.location.isValid()) return false;
  if (gps.location.age() > 3000) return false;
  if (gps.satellites.isValid() && gps.satellites.value() < 4) return false;
  return true;
}

// Pressure altitude from pressure in Pa (ISA, 1013.25 hPa)
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

static String igcAlt5(int meters) {
  if (meters < 0) meters = 0;         // clamp for IGC
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

static void writeIgcHeaders() {
  int dd = gps.date.day();
  int mm = gps.date.month();
  int yy = gps.date.year() % 100;

  igcFile.print("AESP32C3\r\n");
  igcFile.printf("HFDTE%02d%02d%02d\r\n", dd, mm, yy);
  igcFile.print("HFDTM100GPSDATUM:WGS-1984\r\n");
  igcFile.print("HFRFWFIRMWARE:ESP32-C3-RC-IGC-LED-5HZ-PWM-RELALT-OWNER\r\n");
  igcFile.print("HFRHWHARDWARE:ESP32-C3+BN180+SD+BMP180\r\n");
  igcFile.print("HFPLTPILOTINCHARGE:RC\r\n");
  igcFile.print("HFGTYGLIDERTYPE:RCGLIDER\r\n");
  igcFile.print("HFGIDGLIDERID:N/A\r\n");
  igcFile.flush();
}

static void setBaroBaselineIfPossible() {
  if (!baroOK) { baroBaseSet = false; return; }
  float p = bmp180.readPressure(); // Pa
  baroBaseAlt = pressureAltMetersFromPa(p);
  baroBaseSet = true;
  Serial.print("Baro baseline set (pressure altitude): ");
  Serial.print(baroBaseAlt);
  Serial.println(" m");
}

static bool startRecording(RecOwner owner) {
  if (recording) return true;
  if (!sdOK) { Serial.println("START FAIL: SD not OK"); return false; }
  if (!gpsTimeOK()) { Serial.println("START FAIL: GPS time not valid yet"); return false; }

  ensureDir("/IGC");
  String fn = makeIgcFilenameUTC();
  igcFile = SD.open(fn.c_str(), FILE_WRITE);
  if (!igcFile) { Serial.println("START FAIL: cannot open file"); return false; }

  writeIgcHeaders();
  setBaroBaselineIfPossible();

  recording = true;
  recOwner = owner;

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

// ---------------- PWM control ----------------
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

    // Hysteresis
    if (!pwmWantsRec && pwmUs >= PWM_START_US) pwmWantsRec = true;
    if (pwmWantsRec && pwmUs <= PWM_STOP_US)  pwmWantsRec = false;
  }

  // If PWM signal lost -> disarm PWM request (but DON'T kill BOOT recordings)
  if (pwmSeenEver && (millis() - lastPwmOkMs > PWM_TIMEOUT_MS)) {
    pwmWantsRec = false;
  }

  // Apply desired state:
  // PWM can start when requested
  if (pwmWantsRec && !recording) startRecording(OWNER_PWM);

  // PWM can stop ONLY if PWM started this recording
  if (!pwmWantsRec && recording && recOwner == OWNER_PWM) stopRecording();
}

// ---------------- LED ----------------
static void setLED(bool on) {
  if (LED_ACTIVE_LOW) digitalWrite(LED_PIN, on ? LOW : HIGH);
  else               digitalWrite(LED_PIN, on ? HIGH : LOW);
}

static void updateLED() {
  uint32_t now = millis();

  // SD missing/error -> ON 2s, OFF 2s, repeat
  if (!sdOK) {
    uint32_t phase = now % 4000;
    setLED(phase < 2000);
    return;
  }

  // Recording -> double blink per second
  if (recording) {
    uint32_t phase = now % 1000;
    bool on = (phase < 100) || (phase > 200 && phase < 300);
    setLED(on);
    return;
  }

  // GPS fix OK -> solid ON
  if (gpsFixOK()) { setLED(true); return; }

  // GPS time OK but no fix -> slow blink (1 Hz)
  if (gpsTimeOK()) {
    if (now - lastLedToggle > 500) {
      lastLedToggle = now;
      ledState = !ledState;
      setLED(ledState);
    }
    return;
  }

  // No GPS time -> fast blink (~5 Hz)
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

  pinMode(PIN_PWM_IN, INPUT);
  lastPwmOkMs = millis();

  Serial.println("BOOT LOGGER STARTED (5Hz, PWM optional, BOOT always works)");

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

    float gpsAlt = gps.altitude.isValid() ? gps.altitude.meters() : NAN;

    float baroAbsAlt = NAN;
    float baroRelAlt = NAN;
    if (baroOK) {
      float p = bmp180.readPressure();
      baroAbsAlt = (float)pressureAltMetersFromPa(p);
      if (baroBaseSet) baroRelAlt = baroAbsAlt - (float)baroBaseAlt;
    }

    Serial.print("ALT baroAbs=");
    if (isfinite(baroAbsAlt)) Serial.print(baroAbsAlt, 1); else Serial.print("N/A");
    Serial.print(" m  baroRel=");
    if (isfinite(baroRelAlt)) Serial.print(baroRelAlt, 1); else Serial.print("N/A");
    Serial.print(" m  gps=");
    if (isfinite(gpsAlt)) Serial.print(gpsAlt, 1); else Serial.print("N/A");
    Serial.println(" m");
  }

  // IGC logging at 5 Hz
  if (recording && millis() - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = millis();
    if (!gpsTimeOK() || !gps.location.isValid()) return;

    int hh = gps.time.hour();
    int mi = gps.time.minute();
    int ss = gps.time.second();

    String lat = igcLat(gps.location.lat());
    String lon = igcLon(gps.location.lng());
    char fix = gpsFixOK() ? 'A' : 'V';

    int gpsAlt = gps.altitude.isValid() ? (int)lround(gps.altitude.meters()) : 0;

    int pAlt = gpsAlt;
    if (baroOK) {
      float p = bmp180.readPressure();
      int absAlt = pressureAltMetersFromPa(p);
      if (baroBaseSet) pAlt = absAlt - baroBaseAlt; else pAlt = absAlt;
    }
    if (pAlt < 0) pAlt = 0;

    String line = "B" + two(hh) + two(mi) + two(ss) + lat + lon + String(fix)
                + igcAlt5(pAlt) + igcAlt5(gpsAlt) + "\r\n";

    igcFile.print(line);
    igcFile.flush();
  }
}
