ESP32-C3 IGC Flight Logger (BN-180 + BMP180)

DIY IGC-compatible flight logger based on an ESP32-C3, designed for RC gliders / sailplanes.
Logs valid IGC B-records with absolute pressure altitude (QNE) and GNSS altitude, compatible with tools like SeeYou, XCSoar, WeGlide, and OLC (unsigned).

✈️ Features

✅ IGC-compliant B records

✅ Absolute pressure altitude (QNE / ISA 1013.25 hPa) from BMP180

✅ GNSS altitude from BN-180 (0 if invalid)

✅ Correct fix flag handling (A = valid 3D fix, V = invalid/2D)

✅ FXA + SIU extensions via I-record

✅ 5 Hz logging (200 ms)

✅ Start / stop via BOOT button

✅ Optional start / stop via RC PWM channel

✅ Robust handling of GPS dropouts (repeats last known position)

✅ Status LED with clear state patterns

✅ Serial debug output every 2 seconds

📄 What This Logger Records
B Record Fields

Each B record contains:

BHHMMSS LAT LON Fix PressAlt GNSSAlt FXA SIU


PressAlt
Absolute pressure altitude (QNE) referenced to 1013.25 hPa

Can be negative

Written as signed 5-character field (e.g. -0045)

GNSSAlt
GPS altitude above ellipsoid

Written as 00000 if invalid

Fix flag

A → valid 3D fix (lat/lon + altitude + ≥4 satellites)

V → invalid or degraded fix

This matches IGC specification expectations.

🚨 Important Altitude Notes (Very Important)

Pressure altitude is NOT AGL

Negative values near the ground are normal

Example:

Sea-level pressure = 1026 hPa

QNE altitude ≈ −100 m

OLC / WeGlide may display this as negative “AGL”, but that is viewer behavior, not a file error

✔️ Your logger is behaving correctly

🔌 Hardware
Components

ESP32-C3 (Super Mini / Dev Board)

BN-180 GPS (u-blox)

BMP180 barometric pressure sensor

microSD card module

🧷 Pin Wiring
GPS (BN-180)
Signal	ESP32-C3 Pin
TX	GPIO20
RX	GPIO21
VCC	5V (or 3.3V, module dependent)
GND	GND
Barometer (BMP180 – I²C)
Signal	ESP32-C3 Pin
SDA	GPIO4
SCL	GPIO5
VCC	3.3V
GND	GND
microSD (SPI)
Signal	ESP32-C3 Pin
CS	GPIO10
MOSI	GPIO7
MISO	GPIO2
SCK	GPIO6
VCC	3.3V
GND	GND
Controls
Function	Pin
BOOT button	GPIO9
Optional PWM input	GPIO3
Status LED	GPIO8 (active-LOW)
🎮 Start / Stop Recording
BOOT Button

Short press:

▶️ Start recording

⏹ Stop recording

Each start creates a new IGC file

PWM Control (Optional)

≥1600 µs → start recording

≤1400 µs → stop recording

Timeout: if PWM signal disappears for 500 ms, recording stops only if PWM started it

Use a single servo cable:

Red → 5V

Black/Brown → GND

White/Yellow → PWM signal

💡 LED Status Patterns
State	LED
SD missing / error	ON 2s / OFF 2s
GPS time not valid	Fast blink (~5 Hz)
GPS time OK, no fix	Slow blink (1 Hz)
GPS fix OK (ready)	Solid ON
Recording active	Double blink per second
🖥 Serial Debug Output

Printed every 2 seconds at 115200 baud:

GPS time=OK fix=OK sats=8 baro=OK sd=OK recording=YES owner=BOOT
ALT baroQNE=-48.0 m  gps=53.2 m


Useful for:

Verifying GPS fix quality

Checking pressure altitude behavior

Confirming SD card health

📂 SD Card Output

Files stored in /IGC/

Filename format:

YYYYMMDD_HHMMSS.IGC


New file every time recording starts

No power-cycle required
