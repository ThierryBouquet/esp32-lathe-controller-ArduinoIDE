#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "state.h"
#include "config_defaults.h"
#include "hal_display_gp1294.h"

lathe_state_t g_state = {
  .rpm_meas = 0, .rpm_set = 0, .css_v = 0, .dia_mm = 0,
  .mode_css = false, .spindle_on = false,
  .bright_auto = false, .bright_manual = 200
};

WebServer server(80);
unsigned long lastDrawMs = 0;

static const char* INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32 Lathe</title><style>body{font:16px system-ui;margin:20px}label{display:block;margin:.5em 0}input[type=range]{width:100%}"
".row{display:flex;gap:1em;align-items:center} .pill{padding:.25em .6em;border:1px solid #ccc;border-radius:999px}</style></head><body>"
"<h2>ESP32 Lathe – WebUI</h2>"
"<div class='pill'>VFD animations: on-device only. WebUI is static.</div>"
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
"  document.getElementById('manualRow').style.display=s.bright_auto?'none':'flex';\n"
"  document.getElementById('status').textContent=JSON.stringify(s,null,2);\n"
"}\n"
"async function save(){const body={bright_auto:document.getElementById('auto').checked,bright_manual:parseInt(document.getElementById('level').value)};\n"
"  await fetch('/config',{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});\n"
"  load();\n"
"}\n"
"['auto','level'].forEach(id=>document.getElementById(id).addEventListener('input',e=>{\n"
"  if(id==='level') document.getElementById('lv').textContent=e.target.value; save();\n"
"  if(id==='auto'){document.getElementById('manualRow').style.display=e.target.checked?'none':'flex'; save();}\n"
"}));\n"
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
  StaticJsonDocument<256> doc;
  doc["bright_auto"] = g_state.bright_auto;
  doc["bright_manual"] = g_state.bright_manual;
  doc["mode_css"] = g_state.mode_css;
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleConfigPut() {
  if (!server.hasArg("plain")) { server.send(400, "text/plain", "Missing body"); return; }
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) { server.send(400, "text/plain", "Bad JSON"); return; }
  if (doc.containsKey("bright_auto"))   g_state.bright_auto   = doc["bright_auto"].as<bool>();
  if (doc.containsKey("bright_manual")) g_state.bright_manual = (uint16_t)doc["bright_manual"].as<int>();

  // Immediately apply brightness to the VFD
  apply_brightness();

  server.send(200, "text/plain", "OK");
}

void drawVfd() {
  char l1[40], l2[40];
  snprintf(l1, sizeof(l1), "Mode:%s  Lvl:%u", g_state.bright_auto?"AUTO":"MAN", g_state.bright_manual);
  snprintf(l2, sizeof(l2), "RPM:%4.0f  CSS:%3.0f", g_state.rpm_meas, g_state.css_v);
  hal_display_draw2(l1, l2);
}

void startWiFiAP() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  (void)ok;
}

void startWeb() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/config", HTTP_PUT, handleConfigPut);
  server.begin();
}

void setup() {
  // Serial.begin(115200);
  hal_display_init();
  apply_brightness();    // set initial brightness from defaults/state
  startWiFiAP();
  startWeb();
}

void loop() {
  server.handleClient();
  unsigned long now = millis();
  if (now - lastDrawMs >= 250) { // ~4 Hz to reduce visible flicker
    lastDrawMs = now;
    drawVfd();
  }
}
