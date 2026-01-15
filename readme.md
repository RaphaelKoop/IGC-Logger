ESP32-C3 RC Glider IGC Logger

Lightweight IGC flight logger for RC gliders based on ESP32-C3, using a BN-180 GPS, BMP180 barometer, and microSD card.
Compatible with WeGlide, SeeYou, and other IGC viewers.

✈️ Features

✅ IGC file output (B-records)

✅ 5 Hz logging (200 ms interval)

✅ GPS position & altitude

✅ Barometric altitude (BMP180)

✅ microSD storage

✅ Start/stop recording via BOOT button

✅ Single on-board LED status

✅ No WiFi, no BLE, no unnecessary background tasks

✅ Designed for long RC flights (2–3+ hours)

📦 Hardware Used

ESP32-C3 Super Mini

BN-180 GPS module

BMP180 barometric pressure sensor

microSD SPI module

(optional) external capacitor for power stability

🔌 Wiring Overview
ESP32-C3 Pin Assignment
Function	GPIO
GPS RX	GPIO 20
GPS TX	GPIO 21
SD CS	GPIO 10
SD MOSI	GPIO 7
SD MISO	GPIO 2
SD SCK	GPIO 6
I²C SDA	GPIO 4
I²C SCL	GPIO 5
BOOT button	GPIO 9
On-board LED	GPIO 8
Power	5V or 3.3V
Ground	GND
🛰️ BN-180 GPS
GPS pin	ESP32-C3
VCC	5V
GND	GND
TX	GPIO 20
RX	GPIO 21

GPS module LED indicates fix status (fast blink = no fix, slow blink = fix).

🌡️ BMP180 Barometer (I²C)
BMP180 pin	ESP32-C3
VCC	3.3V
GND	GND
SDA	GPIO 4
SCL	GPIO 5

⚠️ Do not power BMP180 from 5V

💾 microSD Card (SPI)
SD pin	ESP32-C3
CS	GPIO 10
MOSI	GPIO 7
MISO	GPIO 2
SCK	GPIO 6
VCC	3.3V
GND	GND

⚠️ Use 3.3V-compatible SD modules only

🔋 Powering the Logger (Important)

Recommended:

5V BEC from receiver or

5V step-down from flight battery

Highly recommended:

220–470 µF capacitor between 5V and GND near ESP32
→ prevents resets during SD writes

🟢 LED Status Overview

The logger uses one on-board LED (active-LOW).

LED pattern	Meaning
ON 2 s / OFF 2 s	❌ SD card missing or error
Fast blink (~5 Hz)	GPS time not available yet
Slow blink (1 Hz)	GPS time OK, no fix
Solid ON	GPS fix OK (ready)
Double blink every second	🔴 Recording active

SD error overrides all other states.

🔘 Recording Control

Use the BOOT button on the ESP32-C3

Short press:

▶️ Start recording

⏹ Stop recording

Files are written to:

/IGC/YYYYMMDD_HHMMSS.IGC

📈 Logging Details

Log rate: 5 Hz (200 ms)

Logged data (IGC B-records):

UTC time

Latitude / Longitude

Fix validity

Barometric altitude

GPS altitude

Ground speed, climb, glide ratio, etc. are calculated by WeGlide from position changes.

🧪 Serial Debug Output

Via USB serial @ 115200 baud:

GPS time / fix / satellites

SD & baro status

Recording state

Baro altitude

GPS altitude

Altitude difference