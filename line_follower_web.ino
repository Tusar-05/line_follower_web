// ================================================================
//  PID Line Follower with Web Dashboard
//  ESP32 + 8-IR + TB6612FNG + N20 Motors + LM2596
//
//  Features:
//   - WiFi Access Point (no router needed)
//   - Real-time web dashboard via WebSocket
//   - Live PID tuning (Kp, Ki, Kd, Base Speed)
//   - Start / Stop / Calibrate controls
//   - Real-time sensor bar visualization
//   - Settings saved to EEPROM/Preferences
// ================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ── Mobile Hotspot Credentials ──────────────────────────────
const char* STA_SSID     = "POCO";
const char* STA_PASSWORD = "niggaswifi";

// ── Backup Access Point (Direct connection if hotspot is off) ─
const char* AP_SSID      = "LineFollower";
const char* AP_PASSWORD  = "12345678";   // min 8 chars

// ── Motor Driver Pins ───────────────────────────────────────
#define AIN1  16
#define AIN2  17
#define PWMA  18
#define BIN1  19
#define BIN2  21
#define PWMB  22
#define STBY  23

// ── IR Sensor Pins (ESP32 DevKit V1) ────────────────────────
const int IR_PINS[8] = {36, 39, 34, 35, 32, 33, 25, 26};
const int WEIGHTS[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

// Set to true if your 8-IR module has digital outputs (D1-D8)
// Set to false if your module has analog outputs (A1-A8)
const bool SENSORS_ARE_DIGITAL = false;
const bool SENSOR_ACTIVE_LOW   = true;  // Most digital IR modules output LOW on black line

// ── LEDC PWM Compatibility (ESP32 Core v2.x & v3.x) ─────────
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#define CH_PWMA  0
#define CH_PWMB  1
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  // ESP32 Arduino Core 3.0+ API
  #define PWM_INIT() do { \
      ledcAttach(PWMA, PWM_FREQ, PWM_RES); \
      ledcAttach(PWMB, PWM_FREQ, PWM_RES); \
  } while(0)
  #define PWM_WRITE_A(val) ledcWrite(PWMA, val)
  #define PWM_WRITE_B(val) ledcWrite(PWMB, val)
#else
  // ESP32 Arduino Core 2.x legacy API
  #define PWM_INIT() do { \
      ledcSetup(CH_PWMA, PWM_FREQ, PWM_RES); \
      ledcAttachPin(PWMA, CH_PWMA); \
      ledcSetup(CH_PWMB, PWM_FREQ, PWM_RES); \
      ledcAttachPin(PWMB, CH_PWMB); \
  } while(0)
  #define PWM_WRITE_A(val) ledcWrite(CH_PWMA, val)
  #define PWM_WRITE_B(val) ledcWrite(CH_PWMB, val)
#endif

// ── PID & Speed (defaults, overwritten by saved prefs) ──────
float Kp = 0.05;
float Ki = 0.0001;
float Kd = 0.8;
int   baseSpeed = 150;
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;

// ── Calibration ─────────────────────────────────────────────
int sensorMin[8];
int sensorMax[8];
bool isCalibrated = false;

// ── PID State ───────────────────────────────────────────────
float lastError = 0;
float integral  = 0;
float currentPosition = 0;
float currentError = 0;
float currentCorrection = 0;

// ── Robot State ─────────────────────────────────────────────
enum RobotState { STOPPED, RUNNING, CALIBRATING, PID_TEST };
RobotState robotState = STOPPED;

// ── Networking ──────────────────────────────────────────────
WebServer server(80);
WebSocketsServer webSocket(81);
Preferences preferences;

// ── Timing ──────────────────────────────────────────────────
unsigned long lastWsSend = 0;
const int WS_SEND_INTERVAL = 100;  // ms between dashboard updates

// ── Sensor readings cache ───────────────────────────────────
int sensorNormalized[8];

// =============================================================
//                    WEB DASHBOARD HTML
// =============================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Line Follower Dashboard</title>
<style>
  :root {
    --bg: #0f172a; --surface: #1e293b; --surface2: #334155;
    --accent: #38bdf8; --accent2: #818cf8; --green: #4ade80;
    --red: #f87171; --yellow: #fbbf24; --text: #f1f5f9;
    --text2: #94a3b8; --radius: 12px;
  }
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
    background: var(--bg); color: var(--text);
    min-height: 100vh; padding: 16px;
  }
  .header {
    text-align: center; padding: 12px 0 20px;
  }
  .header h1 { font-size: 1.5rem; color: var(--accent); }
  .header .status {
    display: inline-flex; align-items: center; gap: 8px;
    margin-top: 8px; font-size: 0.85rem; color: var(--text2);
  }
  .status-dot {
    width: 10px; height: 10px; border-radius: 50%;
    background: var(--red); animation: pulse 1.5s infinite;
  }
  .status-dot.connected { background: var(--green); }
  @keyframes pulse { 0%,100%{opacity:1}50%{opacity:0.4} }

  .grid {
    max-width: 800px; margin: 0 auto;
    display: grid; grid-template-columns: 1fr; gap: 16px;
  }
  @media(min-width:600px) {
    .grid { grid-template-columns: 1fr 1fr; }
    .card.full { grid-column: 1 / -1; }
  }

  .card {
    background: var(--surface); border-radius: var(--radius);
    padding: 20px; border: 1px solid var(--surface2);
  }
  .card h2 {
    font-size: 0.85rem; text-transform: uppercase;
    letter-spacing: 1px; color: var(--text2); margin-bottom: 16px;
  }

  /* Controls */
  .btn-group { display: flex; gap: 8px; flex-wrap: wrap; }
  .btn {
    flex: 1; min-width: 100px; padding: 12px 16px;
    border: none; border-radius: 8px; font-size: 0.95rem;
    font-weight: 600; cursor: pointer; transition: all 0.2s;
    color: #fff;
  }
  .btn:active { transform: scale(0.96); }
  .btn.start { background: var(--green); color: #0f172a; }
  .btn.stop { background: var(--red); }
  .btn.calibrate { background: var(--yellow); color: #0f172a; }
  .btn.test { background: var(--accent); color: #0f172a; }
  .btn:disabled { opacity: 0.4; cursor: not-allowed; }

  /* PID sliders */
  .pid-group { margin-bottom: 16px; }
  .pid-label {
    display: flex; justify-content: space-between;
    margin-bottom: 6px; font-size: 0.9rem;
  }
  .pid-label span:first-child { font-weight: 600; color: var(--accent); }
  .pid-label span:last-child {
    font-family: 'Courier New', monospace; color: var(--text);
    background: var(--surface2); padding: 2px 8px; border-radius: 4px;
  }
  input[type=range] {
    width: 100%; height: 6px; -webkit-appearance: none;
    background: var(--surface2); border-radius: 3px; outline: none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none; width: 20px; height: 20px;
    border-radius: 50%; background: var(--accent); cursor: pointer;
    border: 2px solid var(--bg);
  }
  .num-input {
    width: 80px; padding: 6px 10px; background: var(--surface2);
    border: 1px solid var(--surface2); border-radius: 6px;
    color: var(--text); font-family: 'Courier New', monospace;
    font-size: 0.9rem; text-align: center;
  }
  .num-input:focus { border-color: var(--accent); outline: none; }
  .pid-row {
    display: flex; align-items: center; gap: 10px;
  }
  .pid-row input[type=range] { flex: 1; }

  /* Sensor bars */
  .sensor-container { display: flex; gap: 6px; height: 120px; align-items: flex-end; }
  .sensor-bar-wrapper {
    flex: 1; display: flex; flex-direction: column;
    align-items: center; height: 100%;
  }
  .sensor-bar {
    flex: 1; width: 100%; border-radius: 4px 4px 0 0;
    background: var(--surface2); position: relative;
    overflow: hidden; display: flex; align-items: flex-end;
  }
  .sensor-fill {
    width: 100%; background: var(--accent); border-radius: 4px 4px 0 0;
    transition: height 0.15s ease; min-height: 2px;
  }
  .sensor-label {
    font-size: 0.7rem; color: var(--text2); margin-top: 4px;
    font-weight: 600;
  }

  /* Position indicator */
  .position-track {
    height: 40px; background: var(--surface2); border-radius: 20px;
    position: relative; margin: 16px 0 8px; overflow: hidden;
  }
  .position-center {
    position: absolute; left: 50%; top: 0; bottom: 0;
    width: 2px; background: var(--text2); opacity: 0.3;
  }
  .position-dot {
    position: absolute; top: 50%; width: 20px; height: 20px;
    border-radius: 50%; background: var(--accent);
    transform: translate(-50%, -50%); transition: left 0.15s ease;
    box-shadow: 0 0 10px var(--accent);
  }
  .position-labels {
    display: flex; justify-content: space-between;
    font-size: 0.75rem; color: var(--text2);
  }

  /* Stats */
  .stats-grid {
    display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px;
  }
  .stat {
    text-align: center; padding: 12px;
    background: var(--surface2); border-radius: 8px;
  }
  .stat-value {
    font-size: 1.3rem; font-weight: 700;
    font-family: 'Courier New', monospace; color: var(--accent);
  }
  .stat-label { font-size: 0.7rem; color: var(--text2); margin-top: 4px; }

  /* Robot state badge */
  .robot-state {
    text-align: center; padding: 8px;
    border-radius: 8px; font-weight: 600;
    font-size: 0.95rem; margin-bottom: 12px;
  }
  .robot-state.stopped { background: rgba(248,113,113,0.15); color: var(--red); }
  .robot-state.running { background: rgba(74,222,128,0.15); color: var(--green); }
  .robot-state.calibrating { background: rgba(251,191,36,0.15); color: var(--yellow); }
  .robot-state.pid_test { background: rgba(56,189,248,0.15); color: var(--accent); }

  /* Log */
  .log {
    max-height: 120px; overflow-y: auto; font-family: 'Courier New', monospace;
    font-size: 0.78rem; color: var(--text2); padding: 10px;
    background: var(--bg); border-radius: 8px; line-height: 1.6;
  }
  .log::-webkit-scrollbar { width: 4px; }
  .log::-webkit-scrollbar-thumb { background: var(--surface2); border-radius: 2px; }

  .save-btn {
    margin-top: 12px; width: 100%; padding: 10px; border: none;
    border-radius: 8px; background: var(--accent2); color: #fff;
    font-weight: 600; cursor: pointer; font-size: 0.9rem;
  }
  .save-btn:active { transform: scale(0.97); }
</style>
</head>
<body>
<div class="header">
  <h1>&#x1F916; Line Follower Dashboard</h1>
  <div class="status">
    <div class="status-dot" id="statusDot"></div>
    <span id="statusText">Connecting...</span>
  </div>
</div>

<div class="grid">

  <!-- Robot Control -->
  <div class="card">
    <h2>&#x1F3AE; Robot Control</h2>
    <div class="robot-state stopped" id="robotState">STOPPED</div>
    <div class="btn-group">
      <button class="btn start" id="btnStart" onclick="sendCmd('start')">&#x25B6; Start</button>
      <button class="btn stop" id="btnStop" onclick="sendCmd('stop')">&#x23F9; Stop</button>
    </div>
    <div class="btn-group" style="margin-top:8px">
      <button class="btn calibrate" id="btnCal" onclick="sendCmd('calibrate')">&#x1F504; Calibrate</button>
      <button class="btn test" id="btnTest" onclick="sendCmd('pid_test')">&#x1F9EA; PID Test</button>
    </div>
    <div class="btn-group" style="margin-top:8px">
      <button class="btn" style="background:#6366f1;color:#fff" onclick="sendCmd('test_motors')">&#x26A1; Test Motors</button>
    </div>
  </div>

  <!-- Live Stats -->
  <div class="card">
    <h2>&#x1F4CA; Live Stats</h2>
    <div class="stats-grid">
      <div class="stat">
        <div class="stat-value" id="valPos">0</div>
        <div class="stat-label">Position</div>
      </div>
      <div class="stat">
        <div class="stat-value" id="valErr">0</div>
        <div class="stat-label">Error</div>
      </div>
      <div class="stat">
        <div class="stat-value" id="valCorr">0</div>
        <div class="stat-label">Correction</div>
      </div>
      <div class="stat">
        <div class="stat-value" id="valLM">0</div>
        <div class="stat-label">Left Motor</div>
      </div>
      <div class="stat">
        <div class="stat-value" id="valRM">0</div>
        <div class="stat-label">Right Motor</div>
      </div>
      <div class="stat">
        <div class="stat-value" id="valLoop">0</div>
        <div class="stat-label">Loop (Hz)</div>
      </div>
    </div>
  </div>

  <!-- Sensor Visualization -->
  <div class="card full">
    <h2>&#x1F50D; Sensor Array</h2>
    <div class="sensor-container" id="sensorBars"></div>
    <div class="position-track">
      <div class="position-center"></div>
      <div class="position-dot" id="posDot" style="left:50%"></div>
    </div>
    <div class="position-labels">
      <span>&#x2B05; LEFT</span>
      <span>CENTER</span>
      <span>RIGHT &#x27A1;</span>
    </div>
  </div>

  <!-- PID Tuning -->
  <div class="card full">
    <h2>&#x2699;&#xFE0F; PID Tuning</h2>

    <div class="pid-group">
      <div class="pid-label"><span>Kp (Proportional)</span><span id="kpVal">0.050</span></div>
      <div class="pid-row">
        <input type="range" id="kpSlider" min="0" max="1" step="0.001" value="0.05"
               oninput="updateSlider('kp')">
        <input type="text" class="num-input" id="kpInput" value="0.050"
               onchange="updateInput('kp')">
      </div>
    </div>

    <div class="pid-group">
      <div class="pid-label"><span>Ki (Integral)</span><span id="kiVal">0.00010</span></div>
      <div class="pid-row">
        <input type="range" id="kiSlider" min="0" max="0.01" step="0.00001" value="0.0001"
               oninput="updateSlider('ki')">
        <input type="text" class="num-input" id="kiInput" value="0.00010"
               onchange="updateInput('ki')">
      </div>
    </div>

    <div class="pid-group">
      <div class="pid-label"><span>Kd (Derivative)</span><span id="kdVal">0.800</span></div>
      <div class="pid-row">
        <input type="range" id="kdSlider" min="0" max="10" step="0.01" value="0.8"
               oninput="updateSlider('kd')">
        <input type="text" class="num-input" id="kdInput" value="0.800"
               onchange="updateInput('kd')">
      </div>
    </div>

    <div class="pid-group">
      <div class="pid-label"><span>Base Speed</span><span id="speedVal">150</span></div>
      <div class="pid-row">
        <input type="range" id="speedSlider" min="0" max="255" step="1" value="150"
               oninput="updateSlider('speed')">
        <input type="text" class="num-input" id="speedInput" value="150"
               onchange="updateInput('speed')">
      </div>
    </div>

    <button class="save-btn" onclick="savePID()">&#x1F4BE; Save to ESP32 (persists on reboot)</button>
  </div>

  <!-- Log -->
  <div class="card full">
    <h2>&#x1F4DD; Event Log</h2>
    <div class="log" id="log"></div>
  </div>
</div>

<script>
  // Build sensor bars
  const sensorBarsEl = document.getElementById('sensorBars');
  for (let i = 0; i < 8; i++) {
    sensorBarsEl.innerHTML += '<div class="sensor-bar-wrapper">' +
      '<div class="sensor-bar"><div class="sensor-fill" id="sf' + i + '" style="height:0%"></div></div>' +
      '<div class="sensor-label">S' + (i+1) + '</div></div>';
  }

  let ws;
  let reconnectTimer;

  function connectWS() {
    ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onopen = function() {
      document.getElementById('statusDot').classList.add('connected');
      document.getElementById('statusText').textContent = 'Connected';
      addLog('WebSocket connected');
      clearTimeout(reconnectTimer);
    };
    ws.onclose = function() {
      document.getElementById('statusDot').classList.remove('connected');
      document.getElementById('statusText').textContent = 'Disconnected';
      addLog('Connection lost, reconnecting...');
      reconnectTimer = setTimeout(connectWS, 2000);
    };
    ws.onerror = function() { ws.close(); };
    ws.onmessage = function(evt) { handleMessage(JSON.parse(evt.data)); };
  }

  function handleMessage(d) {
    if (d.type === 'telemetry') {
      for (let i = 0; i < 8; i++) {
        var pct = (d.sensors[i] / 1000) * 100;
        document.getElementById('sf' + i).style.height = pct + '%';
      }
      var posNorm = ((d.position + 3500) / 7000) * 100;
      document.getElementById('posDot').style.left = posNorm + '%';
      document.getElementById('valPos').textContent = d.position.toFixed(0);
      document.getElementById('valErr').textContent = d.error.toFixed(1);
      document.getElementById('valCorr').textContent = d.correction.toFixed(1);
      document.getElementById('valLM').textContent = d.leftMotor;
      document.getElementById('valRM').textContent = d.rightMotor;
      document.getElementById('valLoop').textContent = d.loopHz;
      var stateEl = document.getElementById('robotState');
      stateEl.textContent = d.state;
      stateEl.className = 'robot-state ' + d.state.toLowerCase();
    }
    else if (d.type === 'pid_values') {
      setSlider('kp', d.kp);
      setSlider('ki', d.ki);
      setSlider('kd', d.kd);
      setSlider('speed', d.baseSpeed);
      addLog('PID values loaded from ESP32');
    }
    else if (d.type === 'log') {
      addLog(d.message);
    }
  }

  function setSlider(name, value) {
    var slider = document.getElementById(name + 'Slider');
    var input = document.getElementById(name + 'Input');
    slider.value = value;
    var fmt = name === 'speed' ? String(Math.round(value)) :
              name === 'ki' ? value.toFixed(5) : value.toFixed(3);
    document.getElementById(name + 'Val').textContent = fmt;
    input.value = fmt;
  }

  function updateSlider(name) {
    var slider = document.getElementById(name + 'Slider');
    var val = parseFloat(slider.value);
    var fmt = name === 'speed' ? String(Math.round(val)) :
              name === 'ki' ? val.toFixed(5) : val.toFixed(3);
    document.getElementById(name + 'Val').textContent = fmt;
    document.getElementById(name + 'Input').value = fmt;
    sendPID();
  }

  function updateInput(name) {
    var input = document.getElementById(name + 'Input');
    var val = parseFloat(input.value);
    if (isNaN(val)) return;
    document.getElementById(name + 'Slider').value = val;
    var fmt = name === 'speed' ? String(Math.round(val)) :
              name === 'ki' ? val.toFixed(5) : val.toFixed(3);
    document.getElementById(name + 'Val').textContent = fmt;
    sendPID();
  }

  function sendPID() {
    var kp = parseFloat(document.getElementById('kpSlider').value);
    var ki = parseFloat(document.getElementById('kiSlider').value);
    var kd = parseFloat(document.getElementById('kdSlider').value);
    var speed = parseInt(document.getElementById('speedSlider').value);

    if (ws && ws.readyState === 1) {
      ws.send(JSON.stringify({
        cmd: 'set_pid',
        kp: kp,
        ki: ki,
        kd: kd,
        baseSpeed: speed
      }));
    }
    fetch('/set_pid?kp=' + kp + '&ki=' + ki + '&kd=' + kd + '&speed=' + speed).catch(function(){});
  }

  function savePID() {
    addLog('Saving PID to flash...');
    if (ws && ws.readyState === 1) {
      ws.send(JSON.stringify({ cmd: 'save_pid' }));
    }
    fetch('/cmd?action=save_pid')
      .then(function() { addLog('PID saved to flash'); })
      .catch(function(){});
  }

  function sendCmd(c) {
    addLog('Cmd: ' + c);
    if (ws && ws.readyState === 1) {
      ws.send(JSON.stringify({ cmd: c }));
    }
    fetch('/cmd?action=' + encodeURIComponent(c))
      .then(function() { addLog('Cmd ' + c + ' delivered'); })
      .catch(function(e) { addLog('Cmd delivery error: ' + e); });
  }

  function addLog(msg) {
    var el = document.getElementById('log');
    var t = new Date().toLocaleTimeString();
    el.innerHTML += t + '  ' + msg + '\n';
    el.scrollTop = el.scrollHeight;
  }

  // Automatic HTTP fallback polling if WebSocket is not connected
  setInterval(function() {
    if (!ws || ws.readyState !== 1) {
      fetch('/telemetry')
        .then(function(r) { return r.json(); })
        .then(function(d) {
          handleMessage(d);
          document.getElementById('statusDot').className = 'status-dot connected';
          document.getElementById('statusText').textContent = 'Connected (HTTP)';
        })
        .catch(function() {
          document.getElementById('statusDot').className = 'status-dot';
          document.getElementById('statusText').textContent = 'Connecting...';
        });
    }
  }, 200);

  connectWS();
</script>
</body>
</html>
)rawliteral";

// =============================================================
//  Motor Control
// =============================================================
void motorA(int speed) {
    if (speed >= 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        speed = -speed;
    }
    PWM_WRITE_A(constrain(speed, 0, 255));
}

void motorB(int speed) {
    if (speed >= 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        speed = -speed;
    }
    PWM_WRITE_B(constrain(speed, 0, 255));
}

void stopMotors() {
    PWM_WRITE_A(0);
    PWM_WRITE_B(0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}

// =============================================================
//  Calibration
// =============================================================
void startCalibration() {
    robotState = CALIBRATING;
    Serial.println("Calibrating...");

    for (int i = 0; i < 8; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
    }

    unsigned long start = millis();
    while (millis() - start < 3000) {
        motorA(100);
        motorB(-100);
        for (int i = 0; i < 8; i++) {
            int val = analogRead(IR_PINS[i]);
            if (val < sensorMin[i]) sensorMin[i] = val;
            if (val > sensorMax[i]) sensorMax[i] = val;
        }
        delay(5);
    }
    stopMotors();
    isCalibrated = true;
    robotState = STOPPED;

    // Save calibration
    preferences.begin("lf", false);
    for (int i = 0; i < 8; i++) {
        char key[8];
        sprintf(key, "smin%d", i);
        preferences.putInt(key, sensorMin[i]);
        sprintf(key, "smax%d", i);
        preferences.putInt(key, sensorMax[i]);
    }
    preferences.putBool("cal", true);
    preferences.end();

    Serial.println("Calibration saved!");
}

// =============================================================
//  Sensor Reading
// =============================================================
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

float readPosition() {
    long weightedSum = 0;
    long totalValue  = 0;

    for (int i = 0; i < 8; i++) {
        if (SENSORS_ARE_DIGITAL) {
            bool onLine = digitalRead(IR_PINS[i]);
            if (SENSOR_ACTIVE_LOW) onLine = !onLine;
            sensorNormalized[i] = onLine ? 1000 : 0;
        } else {
            int raw = analogRead(IR_PINS[i]);
            int range = sensorMax[i] - sensorMin[i];
            if (range == 0) {
                sensorNormalized[i] = 0;
            } else {
                sensorNormalized[i] = (long)(raw - sensorMin[i]) * 1000 / range;
                sensorNormalized[i] = constrain(sensorNormalized[i], 0, 1000);
            }
        }
        weightedSum += (long)sensorNormalized[i] * WEIGHTS[i];
        totalValue  += sensorNormalized[i];
    }

    if (totalValue == 0) {
        return (lastError > 0) ? 3500 : -3500;
    }
    return (float)weightedSum / totalValue;
}

// =============================================================
//  PID Control
// =============================================================
unsigned long loopCount = 0;
unsigned long loopCountTime = 0;
int loopHz = 0;

void pidControl() {
    currentPosition = readPosition();
    currentError = currentPosition;

    float P = currentError;
    integral += currentError;
    integral = constrain(integral, -10000, 10000);
    float I = integral;
    float D = currentError - lastError;
    lastError = currentError;

    currentCorrection = (Kp * P) + (Ki * I) + (Kd * D);

    leftMotorSpeed  = constrain(baseSpeed + (int)currentCorrection, MIN_SPEED, MAX_SPEED);
    rightMotorSpeed = constrain(baseSpeed - (int)currentCorrection, MIN_SPEED, MAX_SPEED);

    motorA(leftMotorSpeed);
    motorB(rightMotorSpeed);

    // Loop rate counter
    loopCount++;
    if (millis() - loopCountTime >= 1000) {
        loopHz = loopCount;
        loopCount = 0;
        loopCountTime = millis();
    }
}

// =============================================================
//  PID Test Control (stationary — no forward motion)
//  Robot stays in place but corrects if pushed off the line.
// =============================================================
void pidTestControl() {
    currentPosition = readPosition();
    currentError = currentPosition;

    float P = currentError;
    integral += currentError;
    integral = constrain(integral, -10000, 10000);
    float I = integral;
    float D = currentError - lastError;
    lastError = currentError;

    currentCorrection = (Kp * P) + (Ki * I) + (Kd * D);

    // If well-centered, keep motors completely stopped to prevent buzzing
    if (abs(currentError) < 60) {
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
        stopMotors();
    } else {
        // No base speed — only differential correction drives the motors
        // Allows bidirectional rotation to pivot back onto the line
        leftMotorSpeed  = constrain((int)currentCorrection, -MAX_SPEED, MAX_SPEED);
        rightMotorSpeed = constrain(-(int)currentCorrection, -MAX_SPEED, MAX_SPEED);
        motorA(leftMotorSpeed);
        motorB(rightMotorSpeed);
    }

    // Loop rate counter
    loopCount++;
    if (millis() - loopCountTime >= 1000) {
        loopHz = loopCount;
        loopCount = 0;
        loopCountTime = millis();
    }
}

// =============================================================
//  WebSocket Event Handler
// =============================================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED: {
            Serial.printf("[WS] Client #%u connected\n", num);
            // Send current PID values to new client
            StaticJsonDocument<200> doc;
            doc["type"] = "pid_values";
            doc["kp"] = Kp;
            doc["ki"] = Ki;
            doc["kd"] = Kd;
            doc["baseSpeed"] = baseSpeed;
            String json;
            serializeJson(doc, json);
            webSocket.sendTXT(num, json);
            break;
        }

        case WStype_DISCONNECTED:
            Serial.printf("[WS] Client #%u disconnected\n", num);
            break;

        case WStype_TEXT: {
            StaticJsonDocument<256> doc;
            DeserializationError err = deserializeJson(doc, payload);
            if (err) return;

            const char* cmd = doc["cmd"];
            if (strcmp(cmd, "set_pid") == 0) {
                Kp = doc["kp"] | Kp;
                Ki = doc["ki"] | Ki;
                Kd = doc["kd"] | Kd;
                baseSpeed = doc["baseSpeed"] | baseSpeed;
            } else {
                handleCommand(cmd);
            }
            break;
        }
        default: break;
    }
}

void handleCommand(const char* cmd) {
    if (strcmp(cmd, "start") == 0) {
        robotState = RUNNING;
        integral = 0;
        lastError = 0;
        sendLog("Robot STARTED");
    }
    else if (strcmp(cmd, "stop") == 0) {
        robotState = STOPPED;
        stopMotors();
        sendLog("Robot STOPPED");
    }
    else if (strcmp(cmd, "calibrate") == 0) {
        sendLog("Calibration started...");
        startCalibration();
        sendLog("Calibration complete!");
    }
    else if (strcmp(cmd, "pid_test") == 0) {
        robotState = PID_TEST;
        integral = 0;
        lastError = 0;
        sendLog("PID TEST mode — push robot off line to test response");
    }
    else if (strcmp(cmd, "save_pid") == 0) {
        preferences.begin("lf", false);
        preferences.putFloat("kp", Kp);
        preferences.putFloat("ki", Ki);
        preferences.putFloat("kd", Kd);
        preferences.putInt("speed", baseSpeed);
        preferences.end();
        sendLog("PID saved to flash!");
    }
    else if (strcmp(cmd, "test_motors") == 0) {
        sendLog("Testing Left Motor...");
        motorA(200);
        delay(700);
        motorA(0);
        delay(250);

        sendLog("Testing Right Motor...");
        motorB(200);
        delay(700);
        motorB(0);
        stopMotors();
        sendLog("Motor test complete!");
    }
}

void sendLog(const char* msg) {
    StaticJsonDocument<200> doc;
    doc["type"] = "log";
    doc["message"] = msg;
    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
    Serial.println(msg);
}

// =============================================================
//  Send Telemetry to Dashboard
// =============================================================
String getTelemetryJson() {
    StaticJsonDocument<512> doc;
    doc["type"] = "telemetry";

    JsonArray sensors = doc.createNestedArray("sensors");
    for (int i = 0; i < 8; i++) {
        sensors.add(sensorNormalized[i]);
    }

    doc["position"]   = currentPosition;
    doc["error"]      = currentError;
    doc["correction"] = currentCorrection;
    doc["leftMotor"]  = leftMotorSpeed;
    doc["rightMotor"] = rightMotorSpeed;
    doc["loopHz"]     = loopHz;

    const char* stateStr =
        robotState == RUNNING     ? "RUNNING" :
        robotState == CALIBRATING ? "CALIBRATING" :
        robotState == PID_TEST    ? "PID_TEST" : "STOPPED";
    doc["state"] = stateStr;

    String json;
    serializeJson(doc, json);
    return json;
}

void sendTelemetry() {
    if (millis() - lastWsSend < WS_SEND_INTERVAL) return;
    lastWsSend = millis();

    // Read sensors when idle/stopped (for dashboard visualization)
    if (robotState != RUNNING && robotState != PID_TEST) {
        readPosition();
    }

    webSocket.broadcastTXT(getTelemetryJson());
}

// =============================================================
//  Load Saved Settings
// =============================================================
void loadSettings() {
    // Default calibration values so the robot works out of the box
    for (int i = 0; i < 8; i++) {
        sensorMin[i] = 0;
        sensorMax[i] = 4095;
    }

    preferences.begin("lf", true);  // read-only
    Kp = preferences.getFloat("kp", 0.05);
    Ki = preferences.getFloat("ki", 0.0001);
    Kd = preferences.getFloat("kd", 0.8);
    baseSpeed = preferences.getInt("speed", 150);
    isCalibrated = preferences.getBool("cal", false);

    if (isCalibrated) {
        for (int i = 0; i < 8; i++) {
            char key[8];
            sprintf(key, "smin%d", i);
            sensorMin[i] = preferences.getInt(key, 0);
            sprintf(key, "smax%d", i);
            sensorMax[i] = preferences.getInt(key, 4095);
        }
        Serial.println("Loaded saved calibration data");
    } else {
        Serial.println("Using default sensor range (0-4095)");
    }
    preferences.end();

    Serial.printf("Loaded PID: Kp=%.4f Ki=%.5f Kd=%.3f Speed=%d\n", Kp, Ki, Kd, baseSpeed);
}

// =============================================================
//  Setup
// =============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== PID Line Follower with Web Dashboard ===");

    // Motor pins
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

    PWM_INIT();

    // ── Quick Hardware Motor Self-Test on Boot ──────────────────
    // Spins Left motor for 300ms, then Right motor for 300ms
    Serial.println("\n[Motor Test] Testing Left Motor...");
    motorA(200);
    delay(300);
    motorA(0);
    delay(150);

    Serial.println("[Motor Test] Testing Right Motor...");
    motorB(200);
    delay(300);
    motorB(0);
    stopMotors();
    Serial.println("[Motor Test] Motor self-test complete.\n");

    // IR sensor pins
    for (int i = 0; i < 8; i++) {
        pinMode(IR_PINS[i], SENSORS_ARE_DIGITAL ? INPUT_PULLUP : INPUT);
    }
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Load saved PID & calibration
    loadSettings();

    // Configure WiFi in Dual Mode (Connects to POCO hotspot AND broadcasts AP)
    WiFi.mode(WIFI_AP_STA);
    WiFi.setAutoReconnect(true);

    // 1. Start backup AP so you can always connect directly if needed
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.println("\n[WiFi] Broadcasted AP: " + String(AP_SSID));
    Serial.print("[WiFi] Direct AP IP: http://");
    Serial.println(WiFi.softAPIP());

    // 2. Connect to POCO hotspot
    Serial.printf("[WiFi] Connecting to hotspot '%s'...", STA_SSID);
    WiFi.begin(STA_SSID, STA_PASSWORD);

    // Wait up to 8 seconds for hotspot connection
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 8000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WiFi] CONNECTED to POCO Hotspot!");
        Serial.print("[WiFi] Dashboard URL: http://");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n[WiFi] Hotspot not found or timed out.");
        Serial.println("[WiFi] Ensure POCO hotspot AP Band is set to 2.4 GHz!");
        Serial.println("[WiFi] You can still connect directly to 'LineFollower' WiFi at http://192.168.4.1");
    }

    // 3. Start mDNS so you can visit http://linefollower.local
    if (MDNS.begin("linefollower")) {
        Serial.println("[mDNS] Responder started: http://linefollower.local");
    }

    // HTTP server routes (Port 80)
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", INDEX_HTML);
    });

    server.on("/telemetry", HTTP_GET, []() {
        if (robotState != RUNNING && robotState != PID_TEST) {
            readPosition();
        }
        server.send(200, "application/json", getTelemetryJson());
    });

    server.on("/cmd", HTTP_GET, []() {
        if (server.hasArg("action")) {
            String act = server.arg("action");
            handleCommand(act.c_str());
            server.send(200, "text/plain", "OK");
        } else {
            server.send(400, "text/plain", "Missing action");
        }
    });

    server.on("/set_pid", HTTP_GET, []() {
        if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
        if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
        if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
        if (server.hasArg("speed")) baseSpeed = server.arg("speed").toInt();
        server.send(200, "text/plain", "OK");
    });

    server.begin();
    Serial.println("[HTTP] Server started on port 80");

    // WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("[WS] Server started on port 81");

    loopCountTime = millis();
}

// =============================================================
//  Main Loop
// =============================================================
void loop() {
    webSocket.loop();
    server.handleClient();

    if (robotState == RUNNING) {
        pidControl();
    } else if (robotState == PID_TEST) {
        pidTestControl();
    }

    sendTelemetry();

    if (robotState == RUNNING || robotState == PID_TEST) {
        delay(2);  // PID loop ~500 Hz
    } else {
        delay(10); // Idle — lower CPU usage
    }
}
