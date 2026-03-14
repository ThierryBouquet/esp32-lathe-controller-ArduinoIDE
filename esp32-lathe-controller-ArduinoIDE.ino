#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <time.h>
#include "state.h"
#include "config_defaults.h"
#include "hal_display_gp1294.h"

lathe_state_t g_state = {
  .rpm_meas = 0, .rpm_set = 0, .css_v = 0, .dia_mm = 0,
  .mode_css = false, .spindle_on = false,
  .spindle_motor_on = false,
  .speed_pot_raw = 0, .pwm_duty = 0, .speed_percent = 0.0,
  .bright_auto = false, .bright_manual = 200,
  .rpm_setpoint = 3000, .css_setpoint = 250,
  .dro_x_pos = 0.0, .dro_z_pos = 0.0,
  .dro_connected = false,
  .wifi_connected = false,
  .time_text = "--:--:--"
};

WebServer server(80);
unsigned long lastDrawMs = 0;
unsigned long lastTimeUpdate = 0;
unsigned long lastWiFiCheck = 0;
unsigned long lastDroRead = 0;
bool ntpConfigured = false;

// RPM measurement variables
volatile unsigned long rpmPulseCount = 0;
volatile unsigned long lastPulseTime = 0;
const unsigned long RPM_CALC_INTERVAL = 1000; // Calculate RPM every 1 second

void IRAM_ATTR rpmPulseISR() {
  rpmPulseCount++;
  lastPulseTime = millis();
}

// Calculate Modbus CRC16
uint16_t modbuscrc(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Read 32-bit signed position from DRO via Modbus RTU
float readDroPosition(uint16_t regAddress) {
  uint8_t request[8];
  request[0] = DRO_MODBUS_ADDR;  // Slave address
  request[1] = 0x03;              // Function code 03 (Read Holding Registers)
  request[2] = (regAddress >> 8) & 0xFF;  // Register address high byte
  request[3] = regAddress & 0xFF;         // Register address low byte
  request[4] = 0x00;              // Number of registers high byte
  request[5] = 0x02;              // Number of registers low byte (2 = 32 bits)
  
  uint16_t crc = modbuscrc(request, 6);
  request[6] = crc & 0xFF;        // CRC low byte
  request[7] = (crc >> 8) & 0xFF; // CRC high byte
  
  // Set RS485 to transmit mode
  digitalWrite(PIN_RS485_DE, HIGH);
  delayMicroseconds(100);
  
  // Send request
  Serial2.write(request, 8);
  Serial2.flush();
  
  // Set RS485 to receive mode
  delayMicroseconds(100);
  digitalWrite(PIN_RS485_DE, LOW);
  
  // Wait for response (timeout after 50ms)
  unsigned long startTime = millis();
  while (Serial2.available() < 9 && (millis() - startTime) < 50) {
    delayMicroseconds(100);
  }
  
  if (Serial2.available() >= 9) {
    uint8_t response[9];
    Serial2.readBytes(response, 9);
    
    // Verify response
    if (response[0] == DRO_MODBUS_ADDR && response[1] == 0x03 && response[2] == 0x04) {
      // Verify CRC
      uint16_t receivedCrc = response[7] | (response[8] << 8);
      uint16_t calculatedCrc = modbuscrc(response, 7);
      
      if (receivedCrc == calculatedCrc) {
        // Extract 32-bit signed value
        int32_t rawValue = ((int32_t)response[3] << 24) | 
                          ((int32_t)response[4] << 16) | 
                          ((int32_t)response[5] << 8) | 
                          response[6];
        // Convert to float (assuming 0.01mm resolution)
        return rawValue / 100.0;
      }
    }
  }
  
  // Clear any remaining bytes
  while (Serial2.available()) {
    Serial2.read();
  }
  
  return 0.0; // Return 0 on error
}

// Read both X and Z positions from DRO
void updateDroPositions() {
  // Read X axis (Shaft A - 0x1000)
  float x_pos = readDroPosition(0x1000);
  
  // Small delay between reads
  delay(2);
  
  // Read Z axis (Shaft B - 0x1100)
  float z_pos = readDroPosition(0x1100);
  
  // Check if at least one read was successful (non-zero or valid)
  // This is a simple check - you might want more sophisticated error detection
  if (x_pos != 0.0 || z_pos != 0.0) {
    g_state.dro_connected = true;
    g_state.dro_x_pos = x_pos;
    g_state.dro_z_pos = z_pos;
  } else {
    g_state.dro_connected = false;
  }
}

// NTP Configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;        // Switzerland: UTC+1 (CET)
const int daylightOffset_sec = 3600;    // +1 hour for daylight saving (CEST)

void updateTime() {
  // Only try to get time if WiFi is connected
  if (!g_state.wifi_connected) {
    strcpy(g_state.time_text, "--:--:--");
    return;
  }
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 50)) {  // Very short 50ms timeout
    strcpy(g_state.time_text, "--:--:--");
    Serial.println("DEBUG: getLocalTime failed");
    return;
  }
  
  // Check if time is actually synced (year should be > 2020)
  if (timeinfo.tm_year + 1900 < 2020) {
    strcpy(g_state.time_text, "--:--:--");
    Serial.print("DEBUG: Time not synced yet, year: ");
    Serial.println(timeinfo.tm_year + 1900);
    return;
  }
  
  snprintf(g_state.time_text, sizeof(g_state.time_text), "%02d:%02d:%02d", 
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  Serial.print("DEBUG: Time updated: ");
  Serial.println(g_state.time_text);
}

static const char* INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32 Lathe</title><style>body{font:16px system-ui;margin:20px}label{display:block;margin:.5em 0}"
".info-section{background:#f5f5f5;padding:1em;margin:1em 0;border-radius:8px}"
".status-section{background:#e8f4f8;padding:1em;margin:1em 0;border-radius:8px}"
"</style></head><body>"
"<h2>ESP32 Lathe Controller</h2>"
""
"<div class='status-section'><h3>📊 Live Measurements</h3>"
"<label>Mode: <span id='mode' style='font-weight:bold'></span></label>"
"<label>Measured RPM: <span id='measuredRpm' style='font-weight:bold'>0</span></label>"
"<label>Calculated CSS: <span id='calculatedCss' style='font-weight:bold'>0</span> m/min</label>"
"<label>Diameter: <span id='diameter' style='font-weight:bold'>0</span> mm</label>"
"<label>DRO X Position: <span id='droX' style='font-weight:bold'>0.00</span> mm</label>"
"<label>DRO Z Position: <span id='droZ' style='font-weight:bold'>0.00</span> mm</label>"
"<label>DRO Status: <span id='droStatus' style='font-weight:bold'></span></label>"
"</div>"
""
"<div class='info-section'><h3>⚙️ Motor Control</h3>"
"<label>Spindle Switch: <span id='spindleSwitch' style='font-weight:bold'></span></label>"
"<label>Speed Setting: <span id='speedPercent' style='font-weight:bold'>0</span>%</label>"
"<label>PWM Duty: <span id='pwmDuty' style='font-weight:bold'>0</span>/255</label>"
"</div>"
""
"<div class='info-section'><h3>🎯 Setpoints</h3>"
"<label>RPM Setpoint: <span id='rpmSetpoint' style='font-weight:bold'>0</span> RPM</label>"
"<label>CSS Setpoint: <span id='cssSetpoint' style='font-weight:bold'>0</span> m/min</label>"
"</div>"
""
"<div class='info-section'><h3>System</h3>"
"<label>WiFi: <span id='wifiStatus' style='font-weight:bold'></span></label>"
"<label>Time: <span id='time' style='font-weight:bold'>--:--:--</span></label>"
"</div>"
""
"<div class='info-section'><h3>Brightness</h3>"
"<label>Mode: <span id='brightMode' style='font-weight:bold'></span></label>"
"<label>Level: <span id='brightLevel' style='font-weight:bold'>0</span></label>"
"</div>"
"<pre id='status'></pre>"
"<script>\n"
"async function load(){const s=await fetch('/status').then(r=>r.json());\n"
"  document.getElementById('mode').textContent=s.mode_css?'CSS Mode':'RPM Mode';\n"
"  document.getElementById('mode').style.color=s.mode_css?'blue':'green';\n"
"  document.getElementById('measuredRpm').textContent=Math.round(s.rpm_meas);\n"
"  document.getElementById('calculatedCss').textContent=s.css_v.toFixed(1);\n"
"  document.getElementById('diameter').textContent=s.dia_mm.toFixed(2);\n"
"  document.getElementById('droX').textContent=s.dro_x_pos.toFixed(2);\n"
"  document.getElementById('droZ').textContent=s.dro_z_pos.toFixed(2);\n"
"  document.getElementById('droStatus').textContent=s.dro_connected?'Connected ✓':'Disconnected ✗';\n"
"  document.getElementById('droStatus').style.color=s.dro_connected?'green':'red';\n"
"  document.getElementById('rpmSetpoint').textContent=s.rpm_setpoint;\n"
"  document.getElementById('cssSetpoint').textContent=s.css_setpoint;\n"
"  document.getElementById('wifiStatus').textContent=s.wifi_connected?'Connected ✓':'Disconnected ✗';\n"
"  document.getElementById('wifiStatus').style.color=s.wifi_connected?'green':'red';\n"
"  document.getElementById('time').textContent=s.time;\n"
"  document.getElementById('spindleSwitch').textContent=s.spindle_motor_on?'ON ✓':'OFF ✗';\n"
"  document.getElementById('spindleSwitch').style.color=s.spindle_motor_on?'green':'red';\n"
"  document.getElementById('speedPercent').textContent=s.speed_percent.toFixed(1);\n"
"  document.getElementById('pwmDuty').textContent=s.pwm_duty;\n"
"  document.getElementById('brightMode').textContent=s.bright_auto?'Auto':'Manual';\n"
"  document.getElementById('brightLevel').textContent=s.bright_manual;\n"
"  document.getElementById('status').textContent=JSON.stringify(s,null,2);\n"
"}\n"
"load();\n"
"setInterval(load,1000);\n"
"</script></body></html>";

static void apply_brightness() {
  // For now: always use manual slider value; auto mode mapping will come later
  hal_display_set_brightness(g_state.bright_manual);
}

void updateSpindleControl() {
  // Read potentiometer (12-bit ADC: 0-4095)
  g_state.speed_pot_raw = analogRead(PIN_SPEED_POT);
  
  // Convert to percentage (0-100%)
  g_state.speed_percent = (g_state.speed_pot_raw / 4095.0) * 100.0;
  
  // Map to 8-bit PWM duty cycle (0-255)
  g_state.pwm_duty = map(g_state.speed_pot_raw, 0, 4095, 0, 255);
  
  // Map potentiometer to RPM setpoint range
  g_state.rpm_setpoint = map(g_state.speed_pot_raw, 0, 4095, DEF_RPM_MIN, DEF_RPM_MAX);
  
  // Only output PWM if spindle switch is ON
  if (g_state.spindle_motor_on) {
    ledcWrite(PIN_SPINDLE_PWM, g_state.pwm_duty);
  } else {
    ledcWrite(PIN_SPINDLE_PWM, 0); // Turn off PWM when switch is off
  }
}

void calculateRPM() {
  static unsigned long lastCalcTime = 0;
  unsigned long currentTime = millis();
  
  // Calculate RPM every second
  if (currentTime - lastCalcTime >= RPM_CALC_INTERVAL) {
    unsigned long pulsesSinceLastCalc;
    noInterrupts();
    pulsesSinceLastCalc = rpmPulseCount;
    rpmPulseCount = 0;
    interrupts();
    
    // Calculate RPM: pulses per second * 60 / pulses per revolution, adjusted for belt ratio
    g_state.rpm_meas = (pulsesSinceLastCalc * 60.0) / (RPM_CALC_INTERVAL / 1000.0) * DEF_BELT_RATIO;
    
    // If no pulses for 2 seconds, consider spindle stopped
    if (currentTime - lastPulseTime > 2000) {
      g_state.rpm_meas = 0;
    }
    
    lastCalcTime = currentTime;
  }
}

void calculateCSS() {
  // CSS = π × diameter × RPM / 1000
  // Using current diameter from DRO X position (absolute value as diameter)
  if (g_state.dro_x_pos != 0) {
    g_state.dia_mm = fabs(g_state.dro_x_pos) * 2.0; // X position is radius, diameter = 2*radius
  } else {
    g_state.dia_mm = 0;
  }
  
  // Calculate CSS in m/min
  if (g_state.rpm_meas > 0 && g_state.dia_mm > 0) {
    g_state.css_v = (PI * g_state.dia_mm * g_state.rpm_meas) / 1000.0;
  } else {
    g_state.css_v = 0;
  }
}

void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  StaticJsonDocument<512> doc;
  doc["bright_auto"] = g_state.bright_auto;
  doc["bright_manual"] = g_state.bright_manual;
  doc["mode_css"] = g_state.mode_css;
  doc["rpm_meas"] = g_state.rpm_meas;
  doc["css_v"] = g_state.css_v;
  doc["dia_mm"] = g_state.dia_mm;
  doc["rpm_setpoint"] = g_state.rpm_setpoint;
  doc["css_setpoint"] = g_state.css_setpoint;
  doc["dro_x_pos"] = g_state.dro_x_pos;
  doc["dro_z_pos"] = g_state.dro_z_pos;
  doc["dro_connected"] = g_state.dro_connected;
  doc["wifi_connected"] = g_state.wifi_connected;
  doc["time"] = g_state.time_text;
  doc["spindle_motor_on"] = g_state.spindle_motor_on;
  doc["speed_pot_raw"] = g_state.speed_pot_raw;
  doc["pwm_duty"] = g_state.pwm_duty;
  doc["speed_percent"] = g_state.speed_percent;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}



void drawVfd() {
  // Use the new graphical display function
  hal_display_draw_lathe();
}

void startWiFiSTA() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32-lathe");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("WiFi connecting in background to SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("Password length: ");
  Serial.println(strlen(WIFI_PASSWORD));
  
  // Non-blocking - WiFi will connect in background
  // Connection status is checked in loop()
  g_state.wifi_connected = false;
}

void startWeb() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);

  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Longer delay for serial to stabilize
  
  // Send multiple newlines and test message
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("*** SERIAL TEST ***");
  Serial.println("*** SERIAL TEST ***");
  Serial.println("*** SERIAL TEST ***");
  Serial.println("=====================================");
  Serial.println("ESP32 Lathe Controller Starting...");
  Serial.println("=====================================");
  
  // Turn off onboard RGB LED (WS2812 on GPIO 48)
  neopixelWrite(48, 0, 0, 0);
  
  // Initialize mode switch input pin
  pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);
  Serial.println("Mode switch input pin initialized (pin 36)");
  
  // Initialize spindle motor switch input pin
  pinMode(PIN_SPINDLE_SWITCH, INPUT_PULLUP);
  Serial.println("Spindle motor switch input pin initialized (pin 3)");
  
  // Initialize spindle RPM input pin with interrupt
  pinMode(PIN_SPINDLE_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SPINDLE_RPM), rpmPulseISR, RISING);
  Serial.println("Spindle RPM input pin initialized with interrupt (pin 46)");
  
  // Initialize PWM for spindle motor control
  ledcAttach(PIN_SPINDLE_PWM, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(PIN_SPINDLE_PWM, 0); // Start with motor off
  Serial.println("PWM channel configured for spindle motor (pin 8)");
  
  // Initialize ADC for speed potentiometer
  pinMode(PIN_SPEED_POT, INPUT);
  analogReadResolution(12); // Set ADC to 12-bit resolution (0-4095)
  Serial.println("Speed potentiometer ADC initialized (pin 9)");
  
  // Initialize RS485 direction control pin
  pinMode(PIN_RS485_DE, OUTPUT);
  digitalWrite(PIN_RS485_DE, LOW); // Receive mode by default
  
  // Initialize Serial2 for Modbus communication with DRO
  Serial2.begin(DRO_BAUD_RATE, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  Serial.println("Modbus RTU Serial initialized for DRO");
  
  // Initialize display first
  Serial.println("Initializing display...");
  hal_display_init();
  apply_brightness();
  Serial.println("Display initialized");
  
  // Connect to WiFi network
  Serial.print("WiFi SSID: ");
  Serial.println(WIFI_SSID);
  startWiFiSTA();
  
  // Configure NTP - will work once WiFi connects
  Serial.println("Configuring NTP (will sync once WiFi connected)...");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  ntpConfigured = true;
  
  // Start web server
  Serial.println("Starting web server...");
  startWeb();
  Serial.println("Web server started");
  
  // Initialize time
  updateTime();
  
  Serial.println("=====================================");
  Serial.println("Setup complete!");
  Serial.println("=====================================");
}

void loop() {
  server.handleClient();
  
  unsigned long now = millis();
  
  // Read mode switch (pin 3) - inverted: LOW = CSS, HIGH = RPM
  g_state.mode_css = !digitalRead(PIN_MODE_SWITCH);
  
  // Read spindle on/off switch (pin 36): LOW = OFF, HIGH = ON
  g_state.spindle_motor_on = digitalRead(PIN_SPINDLE_SWITCH);
  
  // Update spindle motor PWM based on potentiometer
  updateSpindleControl();
  
  // Calculate RPM from pulse count
  calculateRPM();
  
  // Calculate CSS based on RPM and diameter
  calculateCSS();
  
  // Read DRO positions every 200ms
  if (now - lastDroRead >= 200) {
    lastDroRead = now;
    updateDroPositions();
  }
  
  // Check WiFi status every 500ms (lightweight check)
  if (now - lastWiFiCheck >= 500) {
    lastWiFiCheck = now;
    bool currentWiFiStatus = (WiFi.status() == WL_CONNECTED);
    if (currentWiFiStatus != g_state.wifi_connected) {
      g_state.wifi_connected = currentWiFiStatus;
      if (currentWiFiStatus) {
        Serial.print("WiFi connected! IP: ");
        Serial.println(WiFi.localIP());
        
        // Start mDNS responder
        if (MDNS.begin("esp32-lathe")) {
          Serial.println("mDNS responder started");
          Serial.println("Access via: http://esp32-lathe.local/");
          MDNS.addService("http", "tcp", 80);
        } else {
          Serial.println("Error starting mDNS");
        }
        
        // Re-configure NTP in case it wasn't set up before
        if (!ntpConfigured) {
          configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
          ntpConfigured = true;
          Serial.println("NTP configured after WiFi connection");
        }
      } else {
        Serial.println("WiFi disconnected");
        strcpy(g_state.time_text, "--:--:--");
      }
    }
  }
  
  // Update time every 5 seconds (non-blocking)
  if (now - lastTimeUpdate >= 5000) {
    lastTimeUpdate = now;
    if (g_state.wifi_connected) {
      updateTime();
    } else {
      strcpy(g_state.time_text, "--:--:--");
    }
  }
  
  // Update display at ~10Hz to reduce flicker
  if (now - lastDrawMs >= 100) {
    lastDrawMs = now;
    drawVfd();
  }
}
