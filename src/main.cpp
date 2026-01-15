// src/main.cpp
// ESP32-C3 IGC Logger (NO WiFi) + On-board LED status (ACTIVE-LOW fixed)
// - BN-180 GPS (UART)
// - BMP180 baro (I2C)
// - microSD (SPI)
// - Start/stop recording with BOOT button
//
// LOG RATE: 5 Hz (B-record every 200 ms)
//
// LED patterns (single LED):
// - SD missing/error:   ON 1.0s, OFF 0.5s, repeat
// - GPS time NOT OK:    fast blink (~5 Hz)
// - GPS time OK no fix: slow blink (1 Hz)
// - GPS fix OK:         solid ON (ready)
// - RECORDING active:   double blink per second
//
// NOTE: Many ESP32-C3 "Super Mini" boards have the onboard LED active-LOW (LOW = ON).
// This code supports that via LED_ACTIVE_LOW=true.

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

// Built-in LED pin (try 8 first; if LED doesn't respond at all try 2)
static const int LED_PIN = 8;

// On many ESP32-C3 Super Mini boards LED is active-LOW: LOW=ON, HIGH=OFF
static const bool LED_ACTIVE_LOW = true;

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

static void writeIgcHeaders() {
  int dd = gps.date.day();
  int mm = gps.date.month();
  int yy = gps.date.year() % 100;

  igcFile.print("AESP32C3\r\n");
  igcFile.printf("HFDTE%02d%02d%02d\r\n", dd, mm, yy);
  igcFile.print("HFDTM100GPSDATUM:WGS-1984\r\n");
  igcFile.print("HFRFWFIRMWARE:ESP32-C3-RC-IGC-LED-5HZ\r\n");
  igcFile.print("HFRHWHARDWARE:ESP32-C3+BN180+SD+BMP180\r\n");
  igcFile.print("HFPLTPILOTINCHARGE:RC\r\n");
  igcFile.print("HFGTYGLIDERTYPE:RCGLIDER\r\n");
  igcFile.print("HFGIDGLIDERID:N/A\r\n");
  igcFile.flush();
}

static bool startRecording() {
  if (recording) return true;
  if (!sdOK) { Serial.println("START FAIL: SD not OK"); return false; }
  if (!gpsTimeOK()) { Serial.println("START FAIL: GPS time not valid yet"); return false; }

  ensureDir("/IGC");
  String fn = makeIgcFilenameUTC();
  igcFile = SD.open(fn.c_str(), FILE_WRITE);
  if (!igcFile) { Serial.println("START FAIL: cannot open file"); return false; }

  writeIgcHeaders();
  recording = true;
  Serial.print("RECORDING STARTED: ");
  Serial.println(fn);
  return true;
}

static void stopRecording() {
  if (!recording) return;
  igcFile.flush();
  igcFile.close();
  recording = false;
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
      else startRecording();
    }
  }
}

// ---------------- LED ----------------
static void setLED(bool on) {
  if (LED_ACTIVE_LOW) {
    digitalWrite(LED_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(LED_PIN, on ? HIGH : LOW);
  }
}

static void updateLED() {
  uint32_t now = millis();

  // SD missing/error -> ON 1.0s, OFF 0.5s, repeat
  if (!sdOK) {
    uint32_t phase = now % 1500;
    bool on = (phase < 1000);
    setLED(on);
    return;
  }

  // Recording -> double blink per second (two short flashes)
  if (recording) {
    uint32_t phase = now % 1000;
    bool on = (phase < 100) || (phase > 200 && phase < 300);
    setLED(on);
    return;
  }

  // GPS fix OK -> solid ON
  if (gpsFixOK()) {
    setLED(true);
    return;
  }

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

  Serial.println("BOOT LOGGER STARTED (LED active-low fix, 5Hz)");

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
}

void loop() {
  // Parse GPS stream
  while (GPS.available()) gps.encode(GPS.read());

  // Handle BOOT toggle
  handleBootToggle();

  // Update LED continuously
  updateLED();

  // Optional status print every 2s
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 2000) {
    lastStatus = millis();
    Serial.print("GPS time=");
    Serial.print(gpsTimeOK() ? "OK" : "NO");
    Serial.print(" fix=");
    Serial.print(gpsFixOK() ? "OK" : "NO");
    if (gps.satellites.isValid()) {
      Serial.print(" sats=");
      Serial.print(gps.satellites.value());
    }
    Serial.print(" baro=");
    Serial.print(baroOK ? "OK" : "NO");
    Serial.print(" sd=");
    Serial.print(sdOK ? "OK" : "NO");
    Serial.print(" recording=");
    Serial.println(recording ? "YES" : "NO");
  }

  // Write IGC B-record at 5 Hz while recording
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
      float p = bmp180.readPressure(); // Pa
      pAlt = pressureAltMetersFromPa(p);
    }

    String line = "B" + two(hh) + two(mi) + two(ss) + lat + lon + String(fix)
                + igcAlt5(pAlt) + igcAlt5(gpsAlt) + "\r\n";

    igcFile.print(line);

    // NOTE: flushing at 5 Hz increases SD write load, but is safest.
    // If you ever get SD glitches, tell me and we’ll flush only once per second.
    igcFile.flush();
  }
}
