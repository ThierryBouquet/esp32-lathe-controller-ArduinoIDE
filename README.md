# ESP32-S3 Lathe Controller (Arduino Step-1)

- Board: **ESP32S3 Dev Module**
- Install libs via Library Manager: **U8g2**, **ArduinoJson**
- Flash and connect to AP: **ESP32-Lathe** / **lathe1234**
- Open `http://192.168.4.1/` for the Web UI (static).
- VFD shows two lines reflecting brightness settings.

Pins (GP1294 SW SPI): MOSI=13, CLK=12, CS=11, RST=9, FILAMENT_EN=14, LDR=10