#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <time.h>
#include "state.h"
#include "config_defaults.h"
#include "hal_display_gp1294.h"

lathe_state_t g_state = {
  .rpm_meas = 0, .rpm_set = 0, .css_v = 0, .dia_mm = 0,
  .mode_css = false, .spindle_on = false,
  .bright_auto = false, .bright_manual = 200,
  .test_rpm = 2950, .test_css = 230,
  .test_x_coord = -2.54, .test_z_coord = -0.89,
  .wifi_connected = true,
  .time_text = "--:--:--"
};

WebServer server(80);
unsigned long lastDrawMs = 0;
unsigned long lastTimeUpdate = 0;
unsigned long lastWiFiCheck = 0;
bool ntpConfigured = false;

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
"<title>ESP32 Lathe</title><style>body{font:16px system-ui;margin:20px}label{display:block;margin:.5em 0}input[type=range]{width:100%}"
".row{display:flex;gap:1em;align-items:center} .pill{padding:.25em .6em;border:1px solid #ccc;border-radius:999px}"
".slider-row{display:grid;grid-template-columns:150px 1fr 80px;gap:1em;align-items:center;margin:0.5em 0}"
".test-section{background:#f5f5f5;padding:1em;margin:1em 0;border-radius:8px}"
"</style></head><body>"
"<h2>ESP32 Lathe – WebUI</h2>"
"<div class='pill'>VFD animations: on-device only. WebUI is static.</div>"
""
"<div class='test-section'><h3>🧪 Testing Controls</h3>"
"<div class='slider-row'><label>RPM:</label><input id='rpm' type='range' min='0' max='6000' step='1'><span id='rpmVal'></span></div>"
"<div class='slider-row'><label>CSS:</label><input id='css' type='range' min='0' max='500' step='1'><span id='cssVal'></span></div>"
"<div class='slider-row'><label>X Coordinate:</label><input id='xcoord' type='range' min='-999.99' max='999.99' step='0.01'><span id='xVal'></span></div>"
"<div class='slider-row'><label>Z Coordinate:</label><input id='zcoord' type='range' min='-99.99' max='99.99' step='0.01'><span id='zVal'></span></div>"
"<label>WiFi Status: <span id='wifiStatus' style='font-weight:bold'></span></label>"
"<label><input id='modeToggle' type='checkbox'> CSS Mode (unchecked = RPM Mode)</label>"
"</div>"
""
"<section><h3>Brightness</h3>"
"<label><input id='auto' type='checkbox'> Auto mode</label>"
"<div id='manualRow' class='row'><label>Manual level: <span id='lv'></span></label><input id='level' type='range' min='0' max='500' step='1'></div>"
"</section>"
"<pre id='status'></pre>"
"<script>\n"
"async function load(){const s=await fetch('/status').then(r=>r.json());\n"
"  document.getElementById('auto').checked=s.bright_auto;\n"
"  document.getElementById('level').value=s.bright_manual;\n"
"  document.getElementById('lv').textContent=s.bright_manual;\n"
"  document.getElementById('rpm').value=s.test_rpm;document.getElementById('rpmVal').textContent=s.test_rpm+' RPM';\n"
"  document.getElementById('css').value=s.test_css;document.getElementById('cssVal').textContent=s.test_css+' CSS';\n"
"  document.getElementById('xcoord').value=s.test_x_coord;document.getElementById('xVal').textContent=s.test_x_coord+' mm';\n"
"  document.getElementById('zcoord').value=s.test_z_coord;document.getElementById('zVal').textContent=s.test_z_coord+' mm';\n"
"  document.getElementById('wifiStatus').textContent=s.wifi_connected?'Connected ✓':'Disconnected ✗';\n"
"  document.getElementById('wifiStatus').style.color=s.wifi_connected?'green':'red';\n"
"  document.getElementById('modeToggle').checked=s.mode_css;\n"
"  document.getElementById('manualRow').style.display=s.bright_auto?'none':'flex';\n"
"  document.getElementById('status').textContent=JSON.stringify(s,null,2);\n"
"}\n"
"async function save(){const body={\n"
"  bright_auto:document.getElementById('auto').checked,\n"
"  bright_manual:parseInt(document.getElementById('level').value),\n"
"  test_rpm:parseInt(document.getElementById('rpm').value),\n"
"  test_css:parseInt(document.getElementById('css').value),\n"
"  test_x_coord:parseFloat(document.getElementById('xcoord').value),\n"
"  mode_css:document.getElementById('modeToggle').checked\n"
"};\n"
"  await fetch('/config',{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});\n"
"  load();\n"
"}\n"
"['auto','level','rpm','css','xcoord','zcoord','modeToggle'].forEach(id=>{\n"
"  const el=document.getElementById(id);\n"
"  el.addEventListener('input',e=>{\n"
"    if(id==='level') document.getElementById('lv').textContent=e.target.value;\n"
"    if(id==='rpm') document.getElementById('rpmVal').textContent=e.target.value+' RPM';\n"
"    if(id==='css') document.getElementById('cssVal').textContent=e.target.value+' CSS';\n"
"    if(id==='xcoord') document.getElementById('xVal').textContent=e.target.value+' mm';\n"
"    if(id==='zcoord') document.getElementById('zVal').textContent=e.target.value+' mm';\n"
"    if(id==='auto') document.getElementById('manualRow').style.display=e.target.checked?'none':'flex';\n"
"    save();\n"
"  });\n"
"});\n"
"load();\n"
"</script></body></html>";

static void apply_brightness() {
  // For now: always use manual slider value; auto mode mapping will come later
  hal_display_set_brightness(g_state.bright_manual);
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
  doc["test_rpm"] = g_state.test_rpm;
  doc["test_css"] = g_state.test_css;
  doc["test_x_coord"] = g_state.test_x_coord;
  doc["test_z_coord"] = g_state.test_z_coord;
  doc["wifi_connected"] = g_state.wifi_connected;
  doc["time"] = g_state.time_text;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleConfigPut() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "Missing body"); return; }
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) { server.send(400, "text/plain", "Bad JSON"); return; }
  
  if (doc.containsKey("bright_auto"))   g_state.bright_auto   = doc["bright_auto"].as<bool>();
  if (doc.containsKey("bright_manual")) g_state.bright_manual = (uint16_t)doc["bright_manual"].as<int>();
  if (doc.containsKey("test_rpm"))      g_state.test_rpm      = (uint16_t)doc["test_rpm"].as<int>();
  if (doc.containsKey("test_css"))      g_state.test_css      = (uint16_t)doc["test_css"].as<int>();
  if (doc.containsKey("test_x_coord"))  g_state.test_x_coord  = doc["test_x_coord"].as<float>();
  if (doc.containsKey("test_z_coord"))  g_state.test_z_coord  = doc["test_z_coord"].as<float>();
  if (doc.containsKey("mode_css"))      g_state.mode_css      = doc["mode_css"].as<bool>();

  // Immediately apply brightness to the VFD
  apply_brightness();

  server.send(200, "text/plain", "OK");
}

void drawVfd() {
  // Use the new graphical display function
  hal_display_draw_lathe();
}

void startWiFiSTA() {
  WiFi.mode(WIFI_STA);
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
  server.on("/config", HTTP_PUT, handleConfigPut);
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
  
  // Check WiFi status every 500ms (lightweight check)
  if (now - lastWiFiCheck >= 500) {
    lastWiFiCheck = now;
    bool currentWiFiStatus = (WiFi.status() == WL_CONNECTED);
    if (currentWiFiStatus != g_state.wifi_connected) {
      g_state.wifi_connected = currentWiFiStatus;
      if (currentWiFiStatus) {
        Serial.print("WiFi connected! IP: ");
        Serial.println(WiFi.localIP());
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
