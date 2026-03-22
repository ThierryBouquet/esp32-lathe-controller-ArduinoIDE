// ESP32 Lathe Controller – FreeRTOS multi-task architecture
// Core 1: control_task (PID, 200 Hz) — dedicated
// Core 0: sensor_dro (10 Hz), sensor_rpm (100 Hz), sensor_io (50 Hz),
//         display (10 Hz), web (event-driven), system (1 Hz)

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include <esp_task_wdt.h>
#include "state.h"
#include "config_defaults.h"
#include "hal_display_gp1294.h"

// ── Global state & synchronisation ──────────────────────────────

lathe_state_t g_state = {
  .rpm_meas = 0, .css_v = 0, .dia_mm = 0,
  .mode_css = false,
  .spindle_motor_on = false,
  .speed_pot_raw = 0, .speed_percent = 0.0,
  .rpm_setpoint = 3000, .css_setpoint = 250,
  .pwm_duty = 0,
  .bright_auto = false, .bright_manual = 200,
  .dro_x_pos = 0.0, .dro_z_pos = 0.0,
  .dro_connected = false,
  .dro_lost_in_css = false,
  .autotune_active = false, .autotune_done = false,
  .autotune_kp = 0, .autotune_ki = 0, .autotune_kd = 0,
  .wifi_connected = false,
  .time_text = "--:--:--"
};

lathe_config_t g_config = {
  .kp = DEF_KP, .ki = DEF_KI, .kd = DEF_KD,
  .out_min = DEF_OUT_MIN, .out_max = DEF_OUT_MAX,
  .rpm_min = DEF_RPM_MIN, .rpm_max = DEF_RPM_MAX,
  .css_min = DEF_CSS_MIN, .css_max = DEF_CSS_MAX,
  .belt_ratio = DEF_BELT_RATIO
};

SemaphoreHandle_t g_state_mutex  = NULL;
SemaphoreHandle_t g_config_mutex = NULL;
static portMUX_TYPE rpm_spinlock = portMUX_INITIALIZER_UNLOCKED;

// ── NVS persistent config ───────────────────────────────────────

static Preferences prefs;

static void nvsLoadConfig() {
  prefs.begin("lathe", true);  // read-only
  g_config.kp      = prefs.getFloat("kp",      DEF_KP);
  g_config.ki      = prefs.getFloat("ki",      DEF_KI);
  g_config.kd      = prefs.getFloat("kd",      DEF_KD);
  g_config.out_min = prefs.getFloat("out_min", DEF_OUT_MIN);
  g_config.out_max = prefs.getFloat("out_max", DEF_OUT_MAX);
  g_config.rpm_min = prefs.getUShort("rpm_min", DEF_RPM_MIN);
  g_config.rpm_max = prefs.getUShort("rpm_max", DEF_RPM_MAX);
  g_config.css_min = prefs.getUShort("css_min", DEF_CSS_MIN);
  g_config.css_max = prefs.getUShort("css_max", DEF_CSS_MAX);
  prefs.end();
  Serial.println("Config loaded from NVS");
}

static void nvsSaveConfig() {
  lathe_config_t cfg;
  xSemaphoreTake(g_config_mutex, portMAX_DELAY);
  cfg = g_config;
  xSemaphoreGive(g_config_mutex);

  prefs.begin("lathe", false);  // read-write
  prefs.putFloat("kp",      cfg.kp);
  prefs.putFloat("ki",      cfg.ki);
  prefs.putFloat("kd",      cfg.kd);
  prefs.putFloat("out_min", cfg.out_min);
  prefs.putFloat("out_max", cfg.out_max);
  prefs.putUShort("rpm_min", cfg.rpm_min);
  prefs.putUShort("rpm_max", cfg.rpm_max);
  prefs.putUShort("css_min", cfg.css_min);
  prefs.putUShort("css_max", cfg.css_max);
  prefs.end();
  Serial.println("Config saved to NVS");
}

static void nvsResetConfig() {
  xSemaphoreTake(g_config_mutex, portMAX_DELAY);
  g_config.kp      = DEF_KP;
  g_config.ki      = DEF_KI;
  g_config.kd      = DEF_KD;
  g_config.out_min = DEF_OUT_MIN;
  g_config.out_max = DEF_OUT_MAX;
  g_config.rpm_min = DEF_RPM_MIN;
  g_config.rpm_max = DEF_RPM_MAX;
  g_config.css_min = DEF_CSS_MIN;
  g_config.css_max = DEF_CSS_MAX;
  xSemaphoreGive(g_config_mutex);

  prefs.begin("lathe", false);
  prefs.clear();
  prefs.end();
  Serial.println("Config reset to defaults");
}

// ── RPM pulse ISR ───────────────────────────────────────────────

static volatile unsigned long rpmPulseCount = 0;
static volatile unsigned long lastPulseTime = 0;

void IRAM_ATTR rpmPulseISR() {
  portENTER_CRITICAL_ISR(&rpm_spinlock);
  rpmPulseCount++;
  lastPulseTime = millis();
  portEXIT_CRITICAL_ISR(&rpm_spinlock);
}

// ── Modbus CRC ──────────────────────────────────────────────────

static uint16_t modbuscrc(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; }
      else                      { crc >>= 1; }
    }
  }
  return crc;
}

// ── DRO Modbus read (called only from sensor_dro_task) ──────────

static float readDroPosition(uint16_t regAddress) {
  uint8_t request[8];
  request[0] = DRO_MODBUS_ADDR;
  request[1] = 0x03;
  request[2] = (regAddress >> 8) & 0xFF;
  request[3] = regAddress & 0xFF;
  request[4] = 0x00;
  request[5] = 0x02;

  uint16_t crc = modbuscrc(request, 6);
  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;

  digitalWrite(PIN_RS485_DE, HIGH);
  delayMicroseconds(100);
  Serial2.write(request, 8);
  Serial2.flush();
  delayMicroseconds(100);
  digitalWrite(PIN_RS485_DE, LOW);

  // Wait for response — yield to RTOS instead of busy-waiting
  unsigned long startTime = millis();
  while (Serial2.available() < 9 && (millis() - startTime) < 50) {
    vTaskDelay(1);   // yield at least 1 tick (~1ms)
  }

  if (Serial2.available() >= 9) {
    uint8_t response[9];
    Serial2.readBytes(response, 9);

    if (response[0] == DRO_MODBUS_ADDR && response[1] == 0x03 && response[2] == 0x04) {
      uint16_t receivedCrc  = response[7] | (response[8] << 8);
      uint16_t calculatedCrc = modbuscrc(response, 7);

      if (receivedCrc == calculatedCrc) {
        // Log raw bytes for DRO scaling diagnosis
        Serial.printf("DRO reg=0x%04X raw=[%02X %02X %02X %02X]\n",
                      regAddress, response[3], response[4], response[5], response[6]);

        int32_t rawValue = ((int32_t)response[3] << 24) |
                           ((int32_t)response[4] << 16) |
                           ((int32_t)response[5] << 8)  |
                           response[6];

        Serial.printf("  assembled=%d (0x%08X) /6553600=%.4f\n", rawValue, rawValue, rawValue/6553600.0f);
        return rawValue / 6553600.0f;
      }
    }
  }

  while (Serial2.available()) Serial2.read();
  return 0.0f;
}

// ── NTP helper (called from system_task when WiFi is up) ────────

static const char* ntpServer = "pool.ntp.org";
// POSIX TZ for Switzerland: CET-1CEST,M3.5.0,M10.5.0/3
// Automatically switches between CET (UTC+1) and CEST (UTC+2)

static void updateTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 50) || timeinfo.tm_year + 1900 < 2020) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    strcpy(g_state.time_text, "--:--:--");
    xSemaphoreGive(g_state_mutex);
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  memcpy(g_state.time_text, buf, sizeof(g_state.time_text));
  xSemaphoreGive(g_state_mutex);
}

// ── Web server & HTML ───────────────────────────────────────────

static WebServer server(80);

static const char* INDEX_HTML =
"<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32 Lathe</title><style>body{font:16px system-ui;margin:20px}label{display:block;margin:.5em 0}"
".info-section{background:#f5f5f5;padding:1em;margin:1em 0;border-radius:8px}"
".status-section{background:#e8f4f8;padding:1em;margin:1em 0;border-radius:8px}"
".tune-section{background:#fff3e0;padding:1em;margin:1em 0;border-radius:8px}"
".tune-section input{width:80px;margin-left:.5em}"
".tune-section button{margin-top:.5em;padding:.4em 1.2em}"
"</style></head><body>"
"<h2>ESP32 Lathe Controller</h2>"
""
"<div class='status-section'><h3>&#x1f4ca; Live Measurements</h3>"
"<label>Mode: <span id='mode' style='font-weight:bold'></span></label>"
"<label>Measured RPM: <span id='measuredRpm' style='font-weight:bold'>0</span></label>"
"<label>Calculated CSS: <span id='calculatedCss' style='font-weight:bold'>0</span> m/min</label>"
"<label>Diameter: <span id='diameter' style='font-weight:bold'>0</span> mm</label>"
"<label>DRO X Position: <span id='droX' style='font-weight:bold'>0.00</span> mm</label>"
"<label>DRO Z Position: <span id='droZ' style='font-weight:bold'>0.00</span> mm</label>"
"<label>DRO Status: <span id='droStatus' style='font-weight:bold'></span></label>"
"<label id='droWarn' style='color:orange;font-weight:bold;display:none'>&#x26a0; DRO lost in CSS mode - RPM frozen!</label>"
"</div>"
""
"<div class='info-section'><h3>&#x2699;&#xfe0f; Motor Control</h3>"
"<label>Spindle Switch: <span id='spindleSwitch' style='font-weight:bold'></span></label>"
"<label>Speed Setting: <span id='speedPercent' style='font-weight:bold'>0</span>%</label>"
"<label>PWM Duty: <span id='pwmDuty' style='font-weight:bold'>0</span>/255</label>"
"</div>"
""
"<div class='info-section'><h3>&#x1f3af; Setpoints</h3>"
"<label>RPM Setpoint: <span id='rpmSetpoint' style='font-weight:bold'>0</span> RPM</label>"
"<label>CSS Setpoint: <span id='cssSetpoint' style='font-weight:bold'>0</span> m/min</label>"
"</div>"
""
"<div class='tune-section'><h3>&#x1f527; PID Tuning</h3>"
"<label>Kp <input id='t_kp' type='number' step='0.01'></label>"
"<label>Ki <input id='t_ki' type='number' step='0.01'></label>"
"<label>Kd <input id='t_kd' type='number' step='0.001'></label>"
"<label>Out min <input id='t_omin' type='number' step='1'></label>"
"<label>Out max <input id='t_omax' type='number' step='1'></label>"
"<label>RPM min <input id='t_rmin' type='number' step='1'></label>"
"<label>RPM max <input id='t_rmax' type='number' step='1'></label>"
"<label>CSS min <input id='t_cmin' type='number' step='1'></label>"
"<label>CSS max <input id='t_cmax' type='number' step='1'></label>"
"<br><button onclick='saveConfig()'>Apply</button>"
" <button onclick='persistConfig()'>Save to Flash</button>"
" <button onclick='resetConfig()'>Reset Defaults</button>"
" <span id='cfgMsg'></span>"
"</div>"
""
"<div class='tune-section' style='background:#e8ffe8'><h3>&#x1f50d; PID Auto-Tune</h3>"
"<p>Runs relay auto-tune at the current RPM setpoint. Spindle must be ON.</p>"
"<button id='atBtn' onclick='startAutotune()'>Start Auto-Tune</button>"
" <span id='atMsg'></span>"
"<div id='atResult' style='display:none;margin-top:.5em'>"
"<label>Suggested Kp: <b id='at_kp'></b></label>"
"<label>Suggested Ki: <b id='at_ki'></b></label>"
"<label>Suggested Kd: <b id='at_kd'></b></label>"
"<button onclick='applyAutotune()'>Apply These Values</button>"
"</div></div>"
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
"let cfgLoaded=false;\n"
"async function loadConfig(){\n"
"  const c=await fetch('/config').then(r=>r.json());\n"
"  document.getElementById('t_kp').value=c.kp;\n"
"  document.getElementById('t_ki').value=c.ki;\n"
"  document.getElementById('t_kd').value=c.kd;\n"
"  document.getElementById('t_omin').value=c.out_min;\n"
"  document.getElementById('t_omax').value=c.out_max;\n"
"  document.getElementById('t_rmin').value=c.rpm_min;\n"
"  document.getElementById('t_rmax').value=c.rpm_max;\n"
"  document.getElementById('t_cmin').value=c.css_min;\n"
"  document.getElementById('t_cmax').value=c.css_max;\n"
"}\n"
"async function saveConfig(){\n"
"  const b={kp:+document.getElementById('t_kp').value,"
"ki:+document.getElementById('t_ki').value,"
"kd:+document.getElementById('t_kd').value,"
"out_min:+document.getElementById('t_omin').value,"
"out_max:+document.getElementById('t_omax').value,"
"rpm_min:+document.getElementById('t_rmin').value,"
"rpm_max:+document.getElementById('t_rmax').value,"
"css_min:+document.getElementById('t_cmin').value,"
"css_max:+document.getElementById('t_cmax').value};\n"
"  const r=await fetch('/config',{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});\n"
"  document.getElementById('cfgMsg').textContent=r.ok?'Applied \\u2713':'Error \\u2717';\n"
"  setTimeout(()=>document.getElementById('cfgMsg').textContent='',2000);\n"
"}\n"
"async function persistConfig(){\n"
"  const r=await fetch('/config/save',{method:'POST'});\n"
"  document.getElementById('cfgMsg').textContent=r.ok?'Saved to flash \\u2713':'Error \\u2717';\n"
"  setTimeout(()=>document.getElementById('cfgMsg').textContent='',2000);\n"
"}\n"
"async function resetConfig(){\n"
"  const r=await fetch('/config/reset',{method:'POST'});\n"
"  if(r.ok){document.getElementById('cfgMsg').textContent='Reset \\u2713';loadConfig();}\n"
"  else document.getElementById('cfgMsg').textContent='Error \\u2717';\n"
"  setTimeout(()=>document.getElementById('cfgMsg').textContent='',2000);\n"
"}\n"
"async function startAutotune(){\n"
"  document.getElementById('atBtn').disabled=true;\n"
"  document.getElementById('atMsg').textContent='Running...';\n"
"  document.getElementById('atResult').style.display='none';\n"
"  const r=await fetch('/autotune',{method:'POST'});\n"
"  document.getElementById('atBtn').disabled=false;\n"
"  if(!r.ok){document.getElementById('atMsg').textContent='Error: spindle must be ON';return;}\n"
"  document.getElementById('atMsg').textContent='Monitoring...';\n"
"}\n"
"function checkAutotune(s){\n"
"  if(s.autotune_active){document.getElementById('atMsg').textContent='Running...';document.getElementById('atBtn').disabled=true;}\n"
"  else if(s.autotune_done){\n"
"    document.getElementById('atMsg').textContent='Done!';\n"
"    document.getElementById('atBtn').disabled=false;\n"
"    document.getElementById('atResult').style.display='block';\n"
"    document.getElementById('at_kp').textContent=s.autotune_kp.toFixed(3);\n"
"    document.getElementById('at_ki').textContent=s.autotune_ki.toFixed(3);\n"
"    document.getElementById('at_kd').textContent=s.autotune_kd.toFixed(3);\n"
"  }\n"
"}\n"
"async function applyAutotune(){\n"
"  const s=await fetch('/status').then(r=>r.json());\n"
"  document.getElementById('t_kp').value=s.autotune_kp.toFixed(3);\n"
"  document.getElementById('t_ki').value=s.autotune_ki.toFixed(3);\n"
"  document.getElementById('t_kd').value=s.autotune_kd.toFixed(3);\n"
"  saveConfig();\n"
"}\n"
"async function load(){const s=await fetch('/status').then(r=>r.json());\n"
"  if(!cfgLoaded){cfgLoaded=true;loadConfig();}\n"
"  document.getElementById('mode').textContent=s.mode_css?'CSS Mode':'RPM Mode';\n"
"  document.getElementById('mode').style.color=s.mode_css?'blue':'green';\n"
"  document.getElementById('measuredRpm').textContent=Math.round(s.rpm_meas);\n"
"  document.getElementById('calculatedCss').textContent=s.css_v.toFixed(1);\n"
"  document.getElementById('diameter').textContent=s.dia_mm.toFixed(2);\n"
"  document.getElementById('droX').textContent=s.dro_x_pos.toFixed(2);\n"
"  document.getElementById('droZ').textContent=s.dro_z_pos.toFixed(2);\n"
"  document.getElementById('droStatus').textContent=s.dro_connected?'Connected \\u2713':'Disconnected \\u2717';\n"
"  document.getElementById('droStatus').style.color=s.dro_connected?'green':'red';\n"
"  document.getElementById('droWarn').style.display=s.dro_lost_in_css?'block':'none';\n"
"  checkAutotune(s);\n"
"  document.getElementById('rpmSetpoint').textContent=s.rpm_setpoint;\n"
"  document.getElementById('cssSetpoint').textContent=s.css_setpoint;\n"
"  document.getElementById('wifiStatus').textContent=s.wifi_connected?'Connected \\u2713':'Disconnected \\u2717';\n"
"  document.getElementById('wifiStatus').style.color=s.wifi_connected?'green':'red';\n"
"  document.getElementById('time').textContent=s.time;\n"
"  document.getElementById('spindleSwitch').textContent=s.spindle_motor_on?'ON \\u2713':'OFF \\u2717';\n"
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

static void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", INDEX_HTML);
}

static void handleStatus() {
  // Snapshot state under mutex – keeps lock time minimal
  lathe_state_t snap;
  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  snap = g_state;
  xSemaphoreGive(g_state_mutex);

  StaticJsonDocument<768> doc;
  doc["bright_auto"]      = snap.bright_auto;
  doc["bright_manual"]    = snap.bright_manual;
  doc["mode_css"]         = snap.mode_css;
  doc["rpm_meas"]         = snap.rpm_meas;
  doc["css_v"]            = snap.css_v;
  doc["dia_mm"]           = snap.dia_mm;
  doc["rpm_setpoint"]     = snap.rpm_setpoint;
  doc["css_setpoint"]     = snap.css_setpoint;
  doc["dro_x_pos"]        = snap.dro_x_pos;
  doc["dro_z_pos"]        = snap.dro_z_pos;
  doc["dro_connected"]    = snap.dro_connected;
  doc["dro_lost_in_css"]   = snap.dro_lost_in_css;
  doc["autotune_active"]   = snap.autotune_active;
  doc["autotune_done"]     = snap.autotune_done;
  doc["autotune_kp"]       = snap.autotune_kp;
  doc["autotune_ki"]       = snap.autotune_ki;
  doc["autotune_kd"]       = snap.autotune_kd;
  doc["wifi_connected"]   = snap.wifi_connected;
  doc["time"]             = snap.time_text;
  doc["spindle_motor_on"] = snap.spindle_motor_on;
  doc["speed_pot_raw"]    = snap.speed_pot_raw;
  doc["pwm_duty"]         = snap.pwm_duty;
  doc["speed_percent"]    = snap.speed_percent;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

static void handleConfigGet() {
  lathe_config_t cfg;
  xSemaphoreTake(g_config_mutex, portMAX_DELAY);
  cfg = g_config;
  xSemaphoreGive(g_config_mutex);

  StaticJsonDocument<256> doc;
  doc["kp"]         = cfg.kp;
  doc["ki"]         = cfg.ki;
  doc["kd"]         = cfg.kd;
  doc["out_min"]    = cfg.out_min;
  doc["out_max"]    = cfg.out_max;
  doc["rpm_min"]    = cfg.rpm_min;
  doc["rpm_max"]    = cfg.rpm_max;
  doc["css_min"]    = cfg.css_min;
  doc["css_max"]    = cfg.css_max;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

static void handleConfigPut() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}");
    return;
  }
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "application/json", "{\"error\":\"bad json\"}");
    return;
  }

  xSemaphoreTake(g_config_mutex, portMAX_DELAY);
  if (doc.containsKey("kp"))         g_config.kp         = doc["kp"];
  if (doc.containsKey("ki"))         g_config.ki         = doc["ki"];
  if (doc.containsKey("kd"))         g_config.kd         = doc["kd"];
  if (doc.containsKey("out_min"))    g_config.out_min    = doc["out_min"];
  if (doc.containsKey("out_max"))    g_config.out_max    = doc["out_max"];
  if (doc.containsKey("rpm_min"))    g_config.rpm_min    = doc["rpm_min"];
  if (doc.containsKey("rpm_max"))    g_config.rpm_max    = doc["rpm_max"];
  if (doc.containsKey("css_min"))    g_config.css_min    = doc["css_min"];
  if (doc.containsKey("css_max"))    g_config.css_max    = doc["css_max"];
  xSemaphoreGive(g_config_mutex);

  server.send(200, "application/json", "{\"ok\":true}");
}

// ── NVS save/reset endpoints ────────────────────────────────────

static void handleConfigSave() {
  nvsSaveConfig();
  server.send(200, "application/json", "{\"ok\":true}");
}

static void handleConfigReset() {
  nvsResetConfig();
  server.send(200, "application/json", "{\"ok\":true}");
}

// ── PID Auto-Tune (relay method, runs in control_task) ──────────
// Triggered via /autotune POST. control_task detects the flag and
// runs a relay experiment: oscillates PWM between out_min and out_max
// around the setpoint, measures oscillation period + amplitude,
// then computes Ziegler-Nichols PID gains.

static volatile bool autotune_request = false;

static void handleAutotuneStart() {
  // Require spindle to be ON
  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  bool on = g_state.spindle_motor_on;
  xSemaphoreGive(g_state_mutex);

  if (!on) {
    server.send(400, "application/json", "{\"error\":\"spindle must be ON\"}");
    return;
  }

  autotune_request = true;

  xSemaphoreTake(g_state_mutex, portMAX_DELAY);
  g_state.autotune_active = true;
  g_state.autotune_done   = false;
  xSemaphoreGive(g_state_mutex);

  server.send(200, "application/json", "{\"ok\":true}");
}

// ═════════════════════════════════════════════════════════════════
//  FreeRTOS Tasks
// ═════════════════════════════════════════════════════════════════

// ── (1) sensor_dro_task – DRO Modbus communication, 10 Hz ──────

void sensor_dro_task(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(500));       // let Serial2 settle after boot
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    float x = readDroPosition(0x1000);
    vTaskDelay(pdMS_TO_TICKS(2));               // inter-axis gap
    float z = readDroPosition(0x1100);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    if (x != 0.0f || z != 0.0f) {
      g_state.dro_connected = true;
      g_state.dro_x_pos = x;
      g_state.dro_z_pos = z;
    } else {
      g_state.dro_connected = false;
    }
    xSemaphoreGive(g_state_mutex);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
  }
}

// ── (2) sensor_rpm_task – RPM measurement, 100 Hz ──────────────

void sensor_rpm_task(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  unsigned long lastCalcTime = millis();

  for (;;) {
    unsigned long now = millis();

    if (now - lastCalcTime >= 1000) {
      unsigned long pulses, lastPulse;
      portENTER_CRITICAL(&rpm_spinlock);
      pulses    = rpmPulseCount;
      rpmPulseCount = 0;
      lastPulse = lastPulseTime;
      portEXIT_CRITICAL(&rpm_spinlock);

      xSemaphoreTake(g_config_mutex, portMAX_DELAY);
      float belt = g_config.belt_ratio;
      xSemaphoreGive(g_config_mutex);

      float elapsed = (now - lastCalcTime) / 1000.0f;
      float rpm = (pulses * 60.0f) / elapsed * belt;
      if (now - lastPulse > 2000) rpm = 0;

      // Read DRO X for CSS calculation
      float dro_x;
      xSemaphoreTake(g_state_mutex, portMAX_DELAY);
      dro_x = g_state.dro_x_pos;
      xSemaphoreGive(g_state_mutex);

      float dia = (dro_x != 0) ? fabs(dro_x) * 2.0f : 0;
      float css = (rpm > 0 && dia > 0) ? (PI * dia * rpm) / 1000.0f : 0;

      xSemaphoreTake(g_state_mutex, portMAX_DELAY);
      g_state.rpm_meas = rpm;
      g_state.dia_mm   = dia;
      g_state.css_v    = css;
      xSemaphoreGive(g_state_mutex);

      lastCalcTime = now;
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}

// ── (3) sensor_io_task – potentiometer & switches, 50 Hz ───────

void sensor_io_task(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    uint16_t pot      = analogRead(PIN_SPEED_POT);
    bool mode_css     = !digitalRead(PIN_MODE_SWITCH);
    bool spindle_on   = !digitalRead(PIN_SPINDLE_SWITCH);  // HIGH = spindle off

    float pct = (pot / 4095.0f) * 100.0f;

    // Read ranges from runtime config
    xSemaphoreTake(g_config_mutex, portMAX_DELAY);
    uint16_t r_min = g_config.rpm_min, r_max = g_config.rpm_max;
    uint16_t c_min = g_config.css_min, c_max = g_config.css_max;
    xSemaphoreGive(g_config_mutex);

    uint16_t rpm_sp = map(pot, 0, 4095, r_min, r_max);
    uint16_t css_sp = map(pot, 0, 4095, c_min, c_max);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.speed_pot_raw    = pot;
    g_state.speed_percent    = pct;
    g_state.rpm_setpoint     = rpm_sp;
    g_state.css_setpoint     = css_sp;
    g_state.mode_css         = mode_css;
    g_state.spindle_motor_on = spindle_on;
    xSemaphoreGive(g_state_mutex);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));
  }
}

// ── (4) control_task – PID loop + auto-tune, 200 Hz ────────────

void control_task(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  const float dt = 1.0f / CONTROL_RATE_HZ;

  float integral      = 0.0f;
  float prev_rpm_meas = 0.0f;

  // Auto-tune relay state
  bool  at_running    = false;
  float at_setpoint   = 0.0f;
  float at_out_high, at_out_low;
  bool  at_relay_high = false;
  int   at_crossings  = 0;
  unsigned long at_last_cross_ms = 0;
  float at_period_sum = 0.0f;
  float at_rpm_min    = 1e6f;
  float at_rpm_max    = -1e6f;
  const int AT_CYCLES = 5;   // number of full oscillation cycles to measure

  for (;;) {
    // ── Snapshot inputs ─────────────────────────────────────
    float rpm_meas, dro_x;
    uint16_t rpm_sp, css_sp;
    bool mode_css, spindle_on, dro_connected;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    rpm_meas      = g_state.rpm_meas;
    dro_x         = g_state.dro_x_pos;
    rpm_sp        = g_state.rpm_setpoint;
    css_sp        = g_state.css_setpoint;
    mode_css      = g_state.mode_css;
    spindle_on    = g_state.spindle_motor_on;
    dro_connected = g_state.dro_connected;
    xSemaphoreGive(g_state_mutex);

    // Snapshot PID gains + limits from runtime config
    float kp, ki, kd, out_min, out_max;
    uint16_t rpm_min_cfg, rpm_max_cfg;
    xSemaphoreTake(g_config_mutex, portMAX_DELAY);
    kp      = g_config.kp;
    ki      = g_config.ki;
    kd      = g_config.kd;
    out_min = g_config.out_min;
    out_max = g_config.out_max;
    rpm_min_cfg = g_config.rpm_min;
    rpm_max_cfg = g_config.rpm_max;
    xSemaphoreGive(g_config_mutex);

    // ── Compute target RPM ──────────────────────────────────
    float target_rpm;
    bool dro_lost = false;

    if (mode_css) {
      // Safety: if DRO disconnects in CSS mode, hold last RPM
      if (!dro_connected) {
        target_rpm = rpm_meas;   // freeze at current RPM
        dro_lost = true;
      } else {
        float diameter_mm = 2.0f * fabs(dro_x);
        if (diameter_mm > 0.001f) {
          target_rpm = (css_sp * 1000.0f) / (PI * diameter_mm);
        } else {
          target_rpm = 0.0f;
        }
      }
      target_rpm = constrain(target_rpm, (float)rpm_min_cfg, (float)rpm_max_cfg);
    } else {
      target_rpm = (float)rpm_sp;
    }

    // Update DRO safety flag for display/WebUI
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.dro_lost_in_css = dro_lost;
    xSemaphoreGive(g_state_mutex);

    // ── Check for auto-tune request ─────────────────────────
    if (autotune_request && !at_running && spindle_on) {
      autotune_request = false;
      at_running    = true;
      at_setpoint   = target_rpm;
      at_out_high   = out_max;
      at_out_low    = out_min;
      at_relay_high = true;
      at_crossings  = 0;
      at_period_sum = 0.0f;
      at_rpm_min    = 1e6f;
      at_rpm_max    = -1e6f;
      at_last_cross_ms = millis();
      integral = 0.0f;
      Serial.printf("Auto-tune started: setpoint=%.0f RPM\n", at_setpoint);
    }

    // ── Auto-tune relay logic ───────────────────────────────
    if (at_running) {
      if (!spindle_on) {
        // Abort if spindle turned off
        at_running = false;
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_state.autotune_active = false;
        xSemaphoreGive(g_state_mutex);
        Serial.println("Auto-tune aborted: spindle OFF");
      } else {
        // Track min/max RPM during oscillation
        if (rpm_meas < at_rpm_min) at_rpm_min = rpm_meas;
        if (rpm_meas > at_rpm_max) at_rpm_max = rpm_meas;

        // Detect zero crossings (RPM crossing setpoint)
        bool above = (rpm_meas >= at_setpoint);
        if (above && !at_relay_high) {
          // Crossed upward → switch relay low
          at_relay_high = true;
          unsigned long now = millis();
          if (at_crossings > 0) {
            at_period_sum += (now - at_last_cross_ms);
          }
          at_last_cross_ms = now;
          at_crossings++;
        } else if (!above && at_relay_high) {
          // Crossed downward → switch relay high
          at_relay_high = false;
        }

        // Apply relay output
        float output = at_relay_high ? at_out_low : at_out_high;
        uint8_t duty = (uint8_t)constrain(output, 0.0f, 255.0f);
        ledcWrite(PIN_SPINDLE_PWM, duty);

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_state.pwm_duty = duty;
        xSemaphoreGive(g_state_mutex);

        // Check if we have enough cycles
        if (at_crossings >= AT_CYCLES * 2 + 1) {
          at_running = false;

          // Calculate Ku and Tu
          float Tu = (at_period_sum / (float)at_crossings) / 1000.0f * 2.0f;  // full period in seconds
          float amplitude = (at_rpm_max - at_rpm_min) / 2.0f;
          float d_output = at_out_high - at_out_low;
          float Ku = (4.0f * d_output) / (PI * amplitude);

          // Ziegler-Nichols PID formula
          float zn_kp = 0.6f * Ku;
          float zn_ki = 2.0f * zn_kp / Tu;
          float zn_kd = zn_kp * Tu / 8.0f;

          Serial.printf("Auto-tune done: Tu=%.3fs Ku=%.3f -> Kp=%.3f Ki=%.3f Kd=%.3f\n",
                        Tu, Ku, zn_kp, zn_ki, zn_kd);

          xSemaphoreTake(g_state_mutex, portMAX_DELAY);
          g_state.autotune_active = false;
          g_state.autotune_done   = true;
          g_state.autotune_kp     = zn_kp;
          g_state.autotune_ki     = zn_ki;
          g_state.autotune_kd     = zn_kd;
          xSemaphoreGive(g_state_mutex);

          // Reset PID state for clean handover
          integral = 0.0f;
          prev_rpm_meas = rpm_meas;
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / CONTROL_RATE_HZ));
        continue;  // skip normal PID while auto-tuning
      }
    }

    // ── PID controller ──────────────────────────────────────
    float output;

    if (!spindle_on) {
      output = 0.0f;
      integral = 0.0f;
      prev_rpm_meas = rpm_meas;
    } else {
      float error = target_rpm - rpm_meas;

      float p_term = kp * error;

      integral += ki * error * dt;
      integral = constrain(integral, out_min, out_max);

      float d_term = -kd * (rpm_meas - prev_rpm_meas) / dt;
      prev_rpm_meas = rpm_meas;

      output = p_term + integral + d_term;
      output = constrain(output, out_min, out_max);
    }

    // ── Write PWM output ────────────────────────────────────
    uint8_t duty = (uint8_t)output;
    ledcWrite(PIN_SPINDLE_PWM, duty);

    // ── Update shared state ─────────────────────────────────
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.pwm_duty = duty;
    xSemaphoreGive(g_state_mutex);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / CONTROL_RATE_HZ));
  }
}

// ── (5) display_task – VFD refresh, 10 Hz ──────────────────────

void display_task(void *pvParameters) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    // Snapshot state to avoid holding mutex during SPI transfer
    lathe_state_t snap;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    snap = g_state;
    xSemaphoreGive(g_state_mutex);

    hal_display_draw_lathe(&snap);
    hal_display_set_brightness(snap.bright_manual);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
  }
}

// ── (6) web_task – HTTP server, event-driven ────────────────────

void web_task(void *pvParameters) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));        // yield when idle
  }
}

// ── (7) system_task – WiFi, NTP, mDNS housekeeping, ~1 Hz ──────

void system_task(void *pvParameters) {
  bool mdnsStarted = false;
  uint8_t ntpCounter = 0;

  for (;;) {
    wl_status_t wifiSt = WiFi.status();
    bool wifiNow  = (wifiSt == WL_CONNECTED);
    bool wifiPrev;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    wifiPrev = g_state.wifi_connected;
    g_state.wifi_connected = wifiNow;
    xSemaphoreGive(g_state_mutex);

    if (wifiNow && !wifiPrev) {
      Serial.print("WiFi connected! IP: ");
      Serial.println(WiFi.localIP());
      if (!mdnsStarted && MDNS.begin("esp32-lathe")) {
        MDNS.addService("http", "tcp", 80);
        mdnsStarted = true;
        Serial.println("mDNS: http://esp32-lathe.local/");
      }
      // Start OTA once WiFi is up
      ArduinoOTA.setHostname("esp32-lathe");
      ArduinoOTA.onStart([]() { Serial.println("OTA update starting..."); });
      ArduinoOTA.onEnd([]()   { Serial.println("OTA update complete!"); });
      ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error: %u\n", e); });
      ArduinoOTA.begin();
      Serial.println("OTA ready");
    } else if (!wifiNow && wifiPrev) {
      Serial.println("WiFi disconnected");
      xSemaphoreTake(g_state_mutex, portMAX_DELAY);
      strcpy(g_state.time_text, "--:--:--");
      xSemaphoreGive(g_state_mutex);
    }

    // Handle OTA updates
    if (wifiNow) {
      ArduinoOTA.handle();
    }

    // Update NTP every ~5 seconds
    if (wifiNow && ++ntpCounter >= 5) {
      ntpCounter = 0;
      updateTime();
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ═════════════════════════════════════════════════════════════════
//  Arduino entry points
// ═════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  // Wait for USB CDC to enumerate so early prints are visible
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(200);
  Serial.println("\n=====================================");
  Serial.println("ESP32 Lathe Controller (FreeRTOS)");
  Serial.println("=====================================");
  Serial.flush();

  // Disable task watchdog for Core 0 IDLE — our FreeRTOS tasks use
  // proper vTaskDelay, but long SPI transfers (display) can still
  // starve IDLE0 beyond the default 5-second WDT timeout.
  esp_err_t wdtErr = esp_task_wdt_deinit();
  Serial.printf("WDT deinit: %s\n", esp_err_to_name(wdtErr));
  Serial.flush();

  // Turn off onboard RGB LED (WS2812 on GPIO 48)
  neopixelWrite(48, 0, 0, 0);

  // ── GPIO setup ──────────────────────────────────────────────
  pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);
  pinMode(PIN_SPINDLE_SWITCH, INPUT_PULLDOWN);
  pinMode(PIN_SPINDLE_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SPINDLE_RPM), rpmPulseISR, RISING);

  pinMode(PIN_SPEED_POT, INPUT);
  analogReadResolution(12);

  // ── RS485 / Modbus ──────────────────────────────────────────
  pinMode(PIN_RS485_DE, OUTPUT);
  digitalWrite(PIN_RS485_DE, LOW);
  Serial2.begin(DRO_BAUD_RATE, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  // ── Display (runs startup animation) ────────────────────────
  Serial.println("Init display..."); Serial.flush();
  hal_display_init();
  hal_display_set_brightness(g_state.bright_manual);
  Serial.println("Display OK"); Serial.flush();

  // ── Spindle PWM (after SPI init to avoid GPIO conflict) ─────
  ledcAttach(PIN_SPINDLE_PWM, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(PIN_SPINDLE_PWM, 0);

  // ── WiFi (non-blocking) ─────────────────────────────────────
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("esp32-lathe");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("WiFi: connecting to %s ...\n", WIFI_SSID); Serial.flush();
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", ntpServer);

  // ── Web server routes ───────────────────────────────────────
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/config", HTTP_GET, handleConfigGet);
  server.on("/config", HTTP_PUT, handleConfigPut);
  server.on("/config/save", HTTP_POST, handleConfigSave);
  server.on("/config/reset", HTTP_POST, handleConfigReset);
  server.on("/autotune", HTTP_POST, handleAutotuneStart);
  server.begin();

  // ── Create mutexes ───────────────────────────────────────
  g_state_mutex  = xSemaphoreCreateMutex();
  g_config_mutex = xSemaphoreCreateMutex();

  // ── Load saved config from NVS ──────────────────────────────
  nvsLoadConfig();

  // ── Create tasks ────────────────────────────────────────────
  xTaskCreatePinnedToCore(sensor_dro_task, "dro",     STACK_DRO,     NULL, PRIO_DRO,     NULL, CORE_SENSORS);
  xTaskCreatePinnedToCore(sensor_rpm_task, "rpm",     STACK_RPM,     NULL, PRIO_RPM,     NULL, CORE_SENSORS);
  xTaskCreatePinnedToCore(sensor_io_task,  "io",      STACK_IO,      NULL, PRIO_IO,      NULL, CORE_SENSORS);
  xTaskCreatePinnedToCore(control_task,    "control", STACK_CONTROL, NULL, PRIO_CONTROL, NULL, CORE_CONTROL);
  xTaskCreatePinnedToCore(display_task,    "display", STACK_DISPLAY, NULL, PRIO_DISPLAY, NULL, CORE_SENSORS);
  xTaskCreatePinnedToCore(web_task,        "web",     STACK_WEB,     NULL, PRIO_WEB,     NULL, CORE_SENSORS);
  xTaskCreatePinnedToCore(system_task,     "system",  STACK_SYSTEM,  NULL, PRIO_SYSTEM,  NULL, CORE_SENSORS);

  Serial.println("All tasks started");
  Serial.println("=====================================");
  Serial.flush();
  Serial.println("=====================================");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
