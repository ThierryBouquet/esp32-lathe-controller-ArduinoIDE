# ESP32-S3 Lathe Controller (Arduino Step-1)

- Board: **ESP32S3 Dev Module**
- Install libs via Library Manager: **U8g2**, **ArduinoJson**
- Flash and connect to AP: **ESP32-Lathe** / **lathe1234**
- Open `http://192.168.4.1/` for the Web UI (static).
- VFD shows two lines reflecting brightness settings.

Pins (GP1294 SW SPI): MOSI=13, CLK=12, CS=11, RST=9, FILAMENT_EN=14, LDR=10
## Repository Setup

- **Build/Flash:** Open the project in the Arduino IDE and select an ESP32S3 Dev Module board, or use `arduino-cli` to compile and upload.
- **Libraries:** Install `U8g2` and `ArduinoJson` via Library Manager or `arduino-cli lib install`.
- **Ignore files:** See `.gitignore` for common build and editor artifacts.