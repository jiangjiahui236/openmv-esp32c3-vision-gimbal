#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <string.h>

// ESP32-C3 will create this Wi-Fi hotspot. Connect your phone/computer to it.
// Password must be at least 8 characters. Set to "" if you want an open hotspot.
const char *AP_SSID = "Gimbal-ESP32C3";
const char *AP_PASSWORD = "change-me";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// Set these to the ESP32-C3 GPIO pins that are physically connected to Nano A4/A5.
// Nano A4 = SDA, Nano A5 = SCL. GND must be common.
const int I2C_SDA_PIN = 4;
const int I2C_SCL_PIN = 5;
const byte NANO_I2C_ADDRESS = 0x12;

// Optional 1.3 inch I2C OLED on the same SDA/SCL bus.
// Most 1.3 inch 128x64 modules use SH1106 at 0x3C. If your module is SSD1306,
// set OLED_CONTROLLER_SH1106 to false so the column offset becomes 0.
const byte OLED_I2C_ADDRESS = 0x3C;
const bool OLED_CONTROLLER_SH1106 = true;
const bool OLED_ROTATE_180 = true;
const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;
const unsigned long OLED_UPDATE_INTERVAL_MS = 80;
const unsigned long OLED_WAKEUP_DURATION_MS = 1600;
const unsigned long OLED_SACCADE_BLINK_MS = 90;
const unsigned long OLED_SEARCH_TRANSITION_MS = 220;

// OpenMV P4(TX) -> ESP32-C3 OPENMV_RX_PIN.
// ESP32-C3 OPENMV_TX_PIN -> OpenMV P5(RX), used for the connection heartbeat.
// GND must be common.
const int OPENMV_RX_PIN = 1;
const int OPENMV_TX_PIN = 0;
const unsigned long OPENMV_TIMEOUT_MS = 1000;
const unsigned long OPENMV_HEARTBEAT_INTERVAL_MS = 500;
const unsigned long OPENMV_CONFIG_INTERVAL_MS = 1000;

const int YAW_CENTER = 90;
const int PITCH_CENTER = 90;
const int YAW_MIN = 30;
const int YAW_MAX = 150;
const int PITCH_MIN = 45;
const int PITCH_MAX = 135;
const int TARGET_FACE = 0;
const int TARGET_RED_CIRCLE = 1;

WebServer server(80);
Preferences preferences;

int currentYaw = YAW_CENTER;
int currentPitch = PITCH_CENTER;
bool lastNanoOk = false;
bool autoTrackEnabled = true;
unsigned long lastOpenMvMs = 0;
unsigned long lastOpenMvHeartbeatMs = 0;
unsigned long lastOpenMvConfigMs = 0;
unsigned long lastOledUpdateMs = 0;
unsigned long oledWakeupStartMs = 0;
unsigned long lastSearchBlinkMs = 0;
bool searchBlinkActive = false;
char openmvLine[24];
byte openmvLineLength = 0;
bool oledPresent = false;
byte oledBuffer[OLED_WIDTH * OLED_HEIGHT / 8];

float cfgYawKp = 0.012;
float cfgPitchKp = 0.012;
int cfgDeadZone = 18;
float cfgMaxStep = 1.0;
int cfgControlMs = 80;
float cfgFilterAlpha = 0.25;
int cfgTargetMode = TARGET_FACE;

void constrainConfig() {
  cfgYawKp = constrain(cfgYawKp, 0.002, 2.000);
  cfgPitchKp = constrain(cfgPitchKp, 0.002, 2.000);
  cfgDeadZone = constrain(cfgDeadZone, 2, 160);
  cfgMaxStep = constrain(cfgMaxStep, 0.2, 90.0);
  cfgControlMs = constrain(cfgControlMs, 10, 1000);
  cfgFilterAlpha = constrain(cfgFilterAlpha, 0.05, 1.00);
  cfgTargetMode = constrain(cfgTargetMode, TARGET_FACE, TARGET_RED_CIRCLE);
}

void loadConfigFromFlash() {
  preferences.begin("gimbal", true);
  cfgYawKp = preferences.getFloat("ykp", cfgYawKp);
  cfgPitchKp = preferences.getFloat("pkp", cfgPitchKp);
  cfgDeadZone = preferences.getInt("dz", cfgDeadZone);
  cfgMaxStep = preferences.getFloat("step", cfgMaxStep);
  cfgControlMs = preferences.getInt("ms", cfgControlMs);
  cfgFilterAlpha = preferences.getFloat("alpha", cfgFilterAlpha);
  cfgTargetMode = TARGET_FACE;
  preferences.end();
  constrainConfig();
  Serial.print("Loaded config from flash: ykp=");
  Serial.print(cfgYawKp, 4);
  Serial.print(" pkp=");
  Serial.print(cfgPitchKp, 4);
  Serial.print(" dz=");
  Serial.print(cfgDeadZone);
  Serial.print(" step=");
  Serial.print(cfgMaxStep, 2);
  Serial.print(" ms=");
  Serial.print(cfgControlMs);
  Serial.print(" alpha=");
  Serial.print(cfgFilterAlpha, 2);
  Serial.print(" target=");
  Serial.println(cfgTargetMode == TARGET_RED_CIRCLE ? "red-circle" : "face");
}

bool saveConfigToFlash() {
  preferences.begin("gimbal", false);
  bool ok = true;
  ok = ok && preferences.putFloat("ykp", cfgYawKp) > 0;
  ok = ok && preferences.putFloat("pkp", cfgPitchKp) > 0;
  ok = ok && preferences.putInt("dz", cfgDeadZone) > 0;
  ok = ok && preferences.putFloat("step", cfgMaxStep) > 0;
  ok = ok && preferences.putInt("ms", cfgControlMs) > 0;
  ok = ok && preferences.putFloat("alpha", cfgFilterAlpha) > 0;
  preferences.end();
  return ok;
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Gimbal Control</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #111418;
      --panel: #1b2027;
      --panel2: #242b34;
      --text: #eef2f7;
      --muted: #9aa6b2;
      --accent: #2dd4bf;
      --line: #303946;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      display: grid;
      place-items: center;
      padding: 18px;
    }
    main {
      width: min(980px, 100%);
      display: grid;
      gap: 16px;
    }
    .layout {
      display: grid;
      grid-template-columns: minmax(360px, 520px) minmax(420px, 1fr);
      align-items: start;
      gap: 16px;
    }
    .control-column,
    .tuning-column {
      display: grid;
      gap: 16px;
    }
    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
    }
    h1 {
      margin: 0;
      font-size: 22px;
      font-weight: 650;
      letter-spacing: 0;
    }
    .status {
      min-width: 76px;
      padding: 6px 10px;
      border: 1px solid var(--line);
      border-radius: 8px;
      color: var(--muted);
      text-align: center;
      font-size: 13px;
    }
    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
    }
    .readout {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    .meter {
      background: var(--panel2);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
    }
    .meter span {
      display: block;
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 4px;
    }
    .meter strong {
      font-size: 28px;
      font-weight: 700;
    }
    .joystick-wrap {
      display: grid;
      place-items: center;
      padding: 22px 0 18px;
    }
    #joystick {
      width: min(78vw, 320px);
      aspect-ratio: 1;
      border-radius: 50%;
      background:
        linear-gradient(90deg, transparent calc(50% - 1px), #35404d calc(50% - 1px), #35404d calc(50% + 1px), transparent calc(50% + 1px)),
        linear-gradient(0deg, transparent calc(50% - 1px), #35404d calc(50% - 1px), #35404d calc(50% + 1px), transparent calc(50% + 1px)),
        radial-gradient(circle, #27313c 0 12%, #202832 13% 100%);
      border: 1px solid var(--line);
      position: relative;
      touch-action: none;
    }
    #knob {
      width: 78px;
      height: 78px;
      border-radius: 50%;
      background: var(--accent);
      border: 1px solid rgba(255, 255, 255, 0.5);
      box-shadow: 0 10px 24px rgba(0, 0, 0, 0.35);
      position: absolute;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
    }
    .actions {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    button {
      min-height: 44px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel2);
      color: var(--text);
      font-size: 15px;
    }
    button:active {
      transform: translateY(1px);
    }
    .target-row {
      display: grid;
      gap: 8px;
    }
    select {
      width: 100%;
      min-height: 42px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel2);
      color: var(--text);
      font-size: 15px;
      padding: 0 10px;
    }
    .sliders {
      display: grid;
      gap: 12px;
    }
    .tuning {
      display: grid;
      gap: 12px;
    }
    .save-row {
      display: grid;
      gap: 6px;
      padding-bottom: 4px;
    }
    #saveBtn {
      width: 100%;
    }
    .save-help {
      margin: 0;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.45;
    }
    .save-message {
      min-height: 18px;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.45;
    }
    .save-message.ok {
      color: var(--accent);
    }
    .save-message.error {
      color: #fb7185;
    }
    .tune-item {
      display: grid;
      gap: 6px;
    }
    .tune-row {
      display: grid;
      grid-template-columns: 78px 38px minmax(0, 1fr) 38px 54px;
      align-items: center;
      gap: 8px;
      color: var(--muted);
      font-size: 13px;
    }
    .tune-row button {
      min-height: 36px;
      width: 38px;
      padding: 0;
      font-size: 18px;
      line-height: 1;
    }
    .tune-value {
      color: var(--text);
      text-align: right;
      font-variant-numeric: tabular-nums;
    }
    .tune-help {
      margin: 0;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.45;
    }
    label {
      display: grid;
      gap: 8px;
      color: var(--muted);
      font-size: 13px;
    }
    input[type="range"] {
      width: 100%;
      accent-color: var(--accent);
    }
    @media (max-width: 900px) {
      .layout {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <main>
    <div class="topbar">
      <h1>云台控制</h1>
      <div class="status" id="status">连接中</div>
    </div>

    <div class="layout">
      <div class="control-column">
        <section class="panel readout">
          <div class="meter"><span>Yaw</span><strong id="yawText">90</strong></div>
          <div class="meter"><span>Pitch</span><strong id="pitchText">90</strong></div>
        </section>

        <section class="panel joystick-wrap">
          <div id="joystick">
            <div id="knob"></div>
          </div>
        </section>

        <section class="panel sliders">
          <label>Yaw
            <input id="yawSlider" type="range" min="30" max="150" value="90">
          </label>
          <label>Pitch
            <input id="pitchSlider" type="range" min="45" max="135" value="90">
          </label>
        </section>

        <div class="actions">
          <button id="centerBtn" type="button">回中</button>
          <button id="modeBtn" type="button">自动追踪</button>
        </div>

        <section class="panel target-row">
          <label>追踪目标
            <select id="targetSelect">
              <option value="0">人脸</option>
              <option value="1">红圆</option>
            </select>
          </label>
        </section>
      </div>

      <div class="tuning-column">
        <section class="panel tuning" id="tuningPanel"></section>
      </div>
    </div>
  </main>

  <script>
    const yawMin = 30, yawMax = 150, pitchMin = 45, pitchMax = 135;
    const joystick = document.getElementById('joystick');
    const knob = document.getElementById('knob');
    const yawText = document.getElementById('yawText');
    const pitchText = document.getElementById('pitchText');
    const yawSlider = document.getElementById('yawSlider');
    const pitchSlider = document.getElementById('pitchSlider');
    const statusEl = document.getElementById('status');
    const modeBtn = document.getElementById('modeBtn');
    const targetSelect = document.getElementById('targetSelect');
    const tuningPanel = document.getElementById('tuningPanel');

    let yaw = 90;
    let pitch = 90;
    let autoMode = true;
    let targetMode = 0;
    let activePointer = null;
    let lastSend = 0;
    let pendingTimer = null;
    let requestBusy = false;
    let queuedYaw = 90;
    let queuedPitch = 90;
    let sentYaw = null;
    let sentPitch = null;
    const sendIntervalMs = 50;
    let cfgTimer = null;
    const tuneParams = [
      {
        key: 'ykp', label: 'Yaw Kp', value: 0.012, min: 0.002, max: 2.00, step: 0.002, digits: 3,
        help: '左右追踪增益。调大：左右响应更快，但容易打过头、来回摆；调小：更稳，但追球变慢。'
      },
      {
        key: 'pkp', label: 'Pitch Kp', value: 0.012, min: 0.002, max: 2.00, step: 0.002, digits: 3,
        help: '上下追踪增益。调大：上下响应更快，但容易上下抖；调小：更平滑，但上下跟随变慢。'
      },
      {
        key: 'dz', label: '死区', value: 18, min: 2, max: 160, step: 1, digits: 0,
        help: '目标离中心多少像素内不动作。调大：中心附近更不抖，但允许偏差更大；调小：更贴中心，但容易细碎抖动。'
      },
      {
        key: 'step', label: '步进', value: 1.0, min: 0.2, max: 90.0, step: 0.2, digits: 1,
        help: '每次控制最多转多少度。调大：追得更猛、更快，但可能冲过目标；调小：动作更柔和，但可能跟不上快速移动。'
      },
      {
        key: 'ms', label: '周期', value: 80, min: 10, max: 1000, step: 10, digits: 0,
        help: '控制更新间隔，单位 ms。调小：更新更频繁、延迟更低，但更容易抖；调大：动作更稳，但反应会变慢。'
      },
      {
        key: 'alpha', label: '滤波', value: 0.25, min: 0.05, max: 1.0, step: 0.05, digits: 2,
        help: '识别点滤波系数。调大：更跟手，但识别框跳动会传到舵机；调小：更平滑抗跳动，但会有滞后。'
      },
    ];
    const storageKey = 'gimbalControlSettingsV1';

    function clamp(value, min, max) {
      return Math.max(min, Math.min(max, value));
    }

    function setStatus(text) {
      statusEl.textContent = text;
    }

    function loadSettings() {
      try {
        const saved = JSON.parse(localStorage.getItem(storageKey) || '{}');
        tuneParams.forEach((param) => {
          if (typeof saved[param.key] === 'number') {
            param.value = clamp(saved[param.key], param.min, param.max);
          }
        });
        if (typeof saved.autoMode === 'boolean') {
          autoMode = saved.autoMode;
        }
      } catch (error) {
        localStorage.removeItem(storageKey);
      }
    }

    function saveSettings() {
      try {
        const saved = { autoMode };
        tuneParams.forEach((param) => {
          saved[param.key] = Number(param.value);
        });
        localStorage.setItem(storageKey, JSON.stringify(saved));
      } catch (error) {
      }
    }

    function applyStatusConfig(data) {
      tuneParams.forEach((param) => {
        if (typeof data[param.key] === 'number') {
          param.value = clamp(data[param.key], param.min, param.max);
        }
      });
      if (typeof data.auto === 'boolean') {
        autoMode = data.auto;
      }
      if (typeof data.target === 'number') {
        targetMode = clamp(Math.round(data.target), 0, 1);
      }
      saveSettings();
    }

    function updateUi() {
      yawText.textContent = Math.round(yaw);
      pitchText.textContent = Math.round(pitch);
      yawSlider.value = Math.round(yaw);
      pitchSlider.value = Math.round(pitch);
      targetSelect.value = String(targetMode);
    }

    function formatTune(param) {
      return Number(param.value).toFixed(param.digits);
    }

    function buildTuningPanel() {
      tuningPanel.innerHTML = '';
      const saveRow = document.createElement('div');
      saveRow.className = 'save-row';
      saveRow.innerHTML = `
        <button id="saveBtn" type="button">写入 Flash</button>
        <p class="save-help">保存当前调参值到 ESP32 Flash，断电或换设备打开网页后仍使用这组参数。</p>
        <div class="save-message" id="saveMessage"></div>
      `;
      tuningPanel.appendChild(saveRow);

      tuneParams.forEach((param) => {
        const item = document.createElement('div');
        item.className = 'tune-item';
        item.innerHTML = `
          <div class="tune-row">
            <span>${param.label}</span>
            <button type="button" data-key="${param.key}" data-dir="-1">-</button>
            <input type="range" min="${param.min}" max="${param.max}" step="${param.step}" value="${param.value}" data-key="${param.key}">
            <button type="button" data-key="${param.key}" data-dir="1">+</button>
            <span class="tune-value" id="value-${param.key}">${formatTune(param)}</span>
          </div>
          <p class="tune-help">${param.help}</p>
        `;
        tuningPanel.appendChild(item);
      });

      tuningPanel.querySelectorAll('input[type="range"]').forEach((input) => {
        input.addEventListener('input', () => {
          const param = tuneParams.find((item) => item.key === input.dataset.key);
          param.value = Number(input.value);
          document.getElementById(`value-${param.key}`).textContent = formatTune(param);
          saveSettings();
          sendConfig();
        });
      });

      tuningPanel.querySelectorAll('.tune-row button').forEach((button) => {
        button.addEventListener('click', () => {
          const param = tuneParams.find((item) => item.key === button.dataset.key);
          const dir = Number(button.dataset.dir);
          param.value = clamp(Number(param.value) + dir * param.step, param.min, param.max);
          const input = tuningPanel.querySelector(`input[data-key="${param.key}"]`);
          input.value = param.value;
          document.getElementById(`value-${param.key}`).textContent = formatTune(param);
          saveSettings();
          sendConfig(true);
        });
      });

      document.getElementById('saveBtn').addEventListener('click', saveConfigToFlash);
    }

    async function loadDeviceStatus() {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        if (!response.ok) return false;
        applyStatusConfig(await response.json());
        return true;
      } catch (error) {
        return false;
      }
    }

    function configQuery() {
      const params = new URLSearchParams();
      tuneParams.forEach((param) => params.set(param.key, formatTune(param)));
      params.set('target', String(targetMode));
      return params.toString();
    }

    function sendConfig(immediate = false) {
      clearTimeout(cfgTimer);
      const run = async () => {
        try {
          const response = await fetch(`/cfg?${configQuery()}`, { cache: 'no-store' });
          setStatus(response.ok ? '参数已发' : '参数失败');
        } catch (error) {
          setStatus('离线');
        }
      };
      if (immediate) {
        run();
      } else {
        cfgTimer = setTimeout(run, 80);
      }
    }

    async function saveConfigToFlash() {
      clearTimeout(cfgTimer);
      const saveBtn = document.getElementById('saveBtn');
      const saveMessage = document.getElementById('saveMessage');
      saveBtn.disabled = true;
      saveBtn.textContent = '写入中...';
      saveMessage.className = 'save-message';
      saveMessage.textContent = '正在写入 ESP32 Flash';
      setStatus('写入中');
      try {
        const response = await fetch(`/save?${configQuery()}`, { cache: 'no-store' });
        const data = await response.json();
        if (response.ok && data.ok) {
          saveMessage.className = 'save-message ok';
          const targetText = Number(data.target) === 1 ? '红圆' : '人脸';
          saveMessage.textContent = `写入成功：Kp ${Number(data.ykp).toFixed(3)} / ${Number(data.pkp).toFixed(3)}，死区 ${data.dz}，步进 ${Number(data.step).toFixed(1)}，周期 ${data.ms}ms，滤波 ${Number(data.alpha).toFixed(2)}，当前目标 ${targetText}`;
          setStatus('已写入Flash');
        } else {
          saveMessage.className = 'save-message error';
          saveMessage.textContent = '写入失败：ESP32 返回错误';
          setStatus('写入失败');
        }
      } catch (error) {
        saveMessage.className = 'save-message error';
        saveMessage.textContent = '写入失败：网页没有连上 ESP32';
        setStatus('离线');
      } finally {
        saveBtn.disabled = false;
        saveBtn.textContent = '写入 Flash';
      }
    }

    async function flushAngles() {
      if (requestBusy) return;

      const now = Date.now();
      const delay = Math.max(0, sendIntervalMs - (now - lastSend));
      if (delay > 0) {
        clearTimeout(pendingTimer);
        pendingTimer = setTimeout(flushAngles, delay);
        return;
      }

      requestBusy = true;
      lastSend = Date.now();
      sentYaw = queuedYaw;
      sentPitch = queuedPitch;
      try {
        const response = await fetch(`/set?yaw=${sentYaw}&pitch=${sentPitch}`, { cache: 'no-store' });
        setStatus(response.ok ? '已发送' : '失败');
      } catch (error) {
        setStatus('离线');
      } finally {
        requestBusy = false;
        if (queuedYaw !== sentYaw || queuedPitch !== sentPitch) {
          flushAngles();
        }
      }
    }

    function sendAngles(immediate = false) {
      queuedYaw = Math.round(yaw);
      queuedPitch = Math.round(pitch);
      clearTimeout(pendingTimer);

      if (immediate) {
        lastSend = 0;
      }
      flushAngles();
    }

    async function setMode(enabled) {
      autoMode = enabled;
      modeBtn.textContent = autoMode ? '自动追踪' : '手动控制';
      saveSettings();
      try {
        await fetch(`/mode?auto=${autoMode ? 1 : 0}`, { cache: 'no-store' });
      } catch (error) {
        setStatus('离线');
      }
    }

    function setAngles(nextYaw, nextPitch, immediate = false) {
      if (autoMode) {
        setMode(false);
      }
      yaw = clamp(nextYaw, yawMin, yawMax);
      pitch = clamp(nextPitch, pitchMin, pitchMax);
      updateUi();
      sendAngles(immediate);
    }

    function updateFromPointer(event) {
      const rect = joystick.getBoundingClientRect();
      const radius = rect.width / 2;
      const centerX = rect.left + radius;
      const centerY = rect.top + radius;
      const maxTravel = radius - knob.offsetWidth / 2 - 8;

      let dx = event.clientX - centerX;
      let dy = event.clientY - centerY;
      const distance = Math.hypot(dx, dy);
      if (distance > maxTravel) {
        dx = dx / distance * maxTravel;
        dy = dy / distance * maxTravel;
      }

      knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

      const x = dx / maxTravel;
      const y = dy / maxTravel;
      const nextYaw = 90 + x * ((yawMax - yawMin) / 2);
      const nextPitch = 90 + y * ((pitchMax - pitchMin) / 2);
      setAngles(nextYaw, nextPitch);
    }

    function centerKnob() {
      knob.style.transform = 'translate(-50%, -50%)';
    }

    joystick.addEventListener('pointerdown', (event) => {
      activePointer = event.pointerId;
      joystick.setPointerCapture(activePointer);
      updateFromPointer(event);
    });

    joystick.addEventListener('pointermove', (event) => {
      if (event.pointerId === activePointer) {
        updateFromPointer(event);
      }
    });

    function releasePointer(event) {
      if (event.pointerId !== activePointer) return;
      activePointer = null;
      centerKnob();
      setAngles(90, 90, true);
    }

    joystick.addEventListener('pointerup', releasePointer);
    joystick.addEventListener('pointercancel', releasePointer);

    yawSlider.addEventListener('input', () => setAngles(Number(yawSlider.value), pitch));
    pitchSlider.addEventListener('input', () => setAngles(yaw, Number(pitchSlider.value)));

    document.getElementById('centerBtn').addEventListener('click', () => {
      centerKnob();
      setAngles(90, 90, true);
    });

    modeBtn.addEventListener('click', () => {
      setMode(!autoMode);
    });

    targetSelect.addEventListener('change', () => {
      targetMode = clamp(Number(targetSelect.value), 0, 1);
      saveSettings();
      setMode(true);
      sendConfig(true);
    });

    async function init() {
      loadSettings();
      await loadDeviceStatus();
      updateUi();
      buildTuningPanel();
      setMode(autoMode);
      sendConfig(true);
    }

    init();
  </script>
</body>
</html>
)rawliteral";

bool sendToNano(int yaw, int pitch) {
  Wire.beginTransmission(NANO_I2C_ADDRESS);
  Wire.write((byte)constrain(yaw, 0, 180));
  Wire.write((byte)constrain(pitch, 0, 180));
  lastNanoOk = Wire.endTransmission() == 0;
  return lastNanoOk;
}

bool oledCommand(byte command) {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  Wire.write(0x00);
  Wire.write(command);
  return Wire.endTransmission() == 0;
}

bool oledCommand2(byte command, byte value) {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  Wire.write(0x00);
  Wire.write(command);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

void oledClearBuffer() {
  memset(oledBuffer, 0, sizeof(oledBuffer));
}

void oledSetPixel(int x, int y, bool on = true) {
  if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
    return;
  }

  int index = x + (y / 8) * OLED_WIDTH;
  byte mask = 1 << (y & 7);
  if (on) {
    oledBuffer[index] |= mask;
  } else {
    oledBuffer[index] &= ~mask;
  }
}

void oledDrawLine(int x0, int y0, int x1, int y1, bool on = true) {
  int dx = abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (true) {
    oledSetPixel(x0, y0, on);
    if (x0 == x1 && y0 == y1) {
      break;
    }
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void oledDrawHLine(int x, int y, int w, bool on = true) {
  for (int i = 0; i < w; i++) {
    oledSetPixel(x + i, y, on);
  }
}

void oledDrawVLine(int x, int y, int h, bool on = true) {
  for (int i = 0; i < h; i++) {
    oledSetPixel(x, y + i, on);
  }
}

void oledFillRoundRect(int x, int y, int w, int h, int r, bool on = true) {
  for (int yy = 0; yy < h; yy++) {
    for (int xx = 0; xx < w; xx++) {
      bool inside = (xx >= r && xx < w - r) || (yy >= r && yy < h - r);
      if (!inside) {
        int cx = xx < r ? r : w - r - 1;
        int cy = yy < r ? r : h - r - 1;
        int dx = xx - cx;
        int dy = yy - cy;
        inside = dx * dx + dy * dy <= r * r;
      }
      if (inside) {
        oledSetPixel(x + xx, y + yy, on);
      }
    }
  }
}

void oledDrawRobotEye(int cx, int cy, int w, int h) {
  w = constrain(w, 4, OLED_WIDTH);
  h = constrain(h, 2, OLED_HEIGHT);
  int radius = min(w, h) / 4;
  radius = constrain(radius, 1, 12);
  oledFillRoundRect(cx - w / 2, cy - h / 2, w, h, radius);
}

void oledDrawBlinkEyes(int leftX, int rightX, int y, int w) {
  oledDrawRobotEye(leftX, y, w, 5);
  oledDrawRobotEye(rightX, y, w, 5);
}

void oledDrawWakeupFace(unsigned long now) {
  unsigned long elapsed = now - oledWakeupStartMs;
  long duration = (long)OLED_WAKEUP_DURATION_MS;
  int h = map(constrain((long)elapsed, 0L, duration), 0L, duration, 2L, 32L);
  oledDrawRobotEye(43, 32, 34, h);
  oledDrawRobotEye(85, 32, 34, h);
}

void oledDrawFocusFace(unsigned long now) {
  bool blink = (now % 3600) > 3500;
  int eyeShiftX = constrain((currentYaw - YAW_CENTER) / 5, -18, 18);
  int eyeShiftY = constrain((currentPitch - PITCH_CENTER) / 7, -11, 11);

  if (blink) {
    oledDrawBlinkEyes(43 + eyeShiftX, 85 + eyeShiftX, 32 + eyeShiftY, 34);
    return;
  }

  oledDrawRobotEye(43 + eyeShiftX, 32 + eyeShiftY, 34, 32);
  oledDrawRobotEye(85 + eyeShiftX, 32 + eyeShiftY, 34, 32);
}

void oledDrawCenterFace(unsigned long now) {
  bool blink = (now % 3200) > 3100;
  if (blink) {
    oledDrawBlinkEyes(43, 85, 32, 34);
    return;
  }

  oledDrawRobotEye(43, 32, 34, 32);
  oledDrawRobotEye(85, 32, 34, 32);
}

void oledDrawSearchingFace(unsigned long now) {
  if (now - lastSearchBlinkMs >= 2600) {
    lastSearchBlinkMs = now;
    searchBlinkActive = true;
  } else if (now - lastSearchBlinkMs >= OLED_SACCADE_BLINK_MS) {
    searchBlinkActive = false;
  }

  const int positions[][2] = {
    {-16, 0}, {16, 0}, {0, -10}, {0, 10}
  };
  const unsigned long starts[] = {0, 1000, 2000, 3000};
  const unsigned long durations[] = {1000, 1000, 1000, 1000};

  unsigned long phase = now % 4000;
  int index = 0;
  if (phase >= 3000) {
    index = 3;
  } else if (phase >= 2000) {
    index = 2;
  } else if (phase >= 1000) {
    index = 1;
  }

  int nextIndex = (index + 1) % 4;
  unsigned long local = phase - starts[index];
  unsigned long transitionStart = durations[index] > OLED_SEARCH_TRANSITION_MS
                                  ? durations[index] - OLED_SEARCH_TRANSITION_MS
                                  : 0;
  int eyeX = positions[index][0];
  int eyeY = positions[index][1];

  if (local >= transitionStart) {
    long t = map((long)(local - transitionStart),
                 0L, (long)OLED_SEARCH_TRANSITION_MS, 0L, 100L);
    t = constrain(t, 0L, 100L);
    t = t * t * (300 - 2 * t) / 10000;
    eyeX = positions[index][0] + (positions[nextIndex][0] - positions[index][0]) * t / 100;
    eyeY = positions[index][1] + (positions[nextIndex][1] - positions[index][1]) * t / 100;
  }

  int h = searchBlinkActive ? 6 : 30;
  int leftW = 34;
  int rightW = 34;
  int leftH = h;
  int rightH = h;
  if (!searchBlinkActive) {
    if (eyeX < -8) {
      leftW = 40;
      leftH = 34;
      rightW = 30;
      rightH = 28;
    } else if (eyeX > 8) {
      leftW = 30;
      leftH = 28;
      rightW = 40;
      rightH = 34;
    }
  }

  oledDrawRobotEye(43 + eyeX, 32 + eyeY, leftW, leftH);
  oledDrawRobotEye(85 + eyeX, 32 + eyeY, rightW, rightH);
}

void oledDrawErrorFace() {
  oledDrawRobotEye(43, 32, 34, 32);
  oledDrawRobotEye(85, 32, 34, 32);
  oledDrawLine(33, 22, 53, 42, false);
  oledDrawLine(53, 22, 33, 42, false);
  oledDrawLine(75, 22, 95, 42, false);
  oledDrawLine(95, 22, 75, 42, false);
}

void oledFlush() {
  if (!oledPresent) {
    return;
  }

  int columnOffset = OLED_CONTROLLER_SH1106 ? 2 : 0;
  for (byte page = 0; page < 8; page++) {
    oledCommand(0xB0 + page);
    oledCommand(0x00 + (columnOffset & 0x0F));
    oledCommand(0x10 + (columnOffset >> 4));

    for (int col = 0; col < OLED_WIDTH; col += 16) {
      Wire.beginTransmission(OLED_I2C_ADDRESS);
      Wire.write(0x40);
      for (int i = 0; i < 16; i++) {
        Wire.write(oledBuffer[page * OLED_WIDTH + col + i]);
      }
      if (Wire.endTransmission() != 0) {
        oledPresent = false;
        Serial.println("OLED I2C write failed, display disabled");
        return;
      }
    }
  }
}

bool oledInit() {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.println("OLED not found at 0x3C");
    return false;
  }

  bool ok = true;
  ok = ok && oledCommand(0xAE);
  ok = ok && oledCommand2(0xD5, 0x80);
  ok = ok && oledCommand2(0xA8, 0x3F);
  ok = ok && oledCommand2(0xD3, 0x00);
  ok = ok && oledCommand(0x40);
  if (OLED_CONTROLLER_SH1106) {
    ok = ok && oledCommand2(0xAD, 0x8B);
  } else {
    ok = ok && oledCommand2(0x8D, 0x14);
  }
  ok = ok && oledCommand(OLED_ROTATE_180 ? 0xA0 : 0xA1);
  ok = ok && oledCommand(OLED_ROTATE_180 ? 0xC0 : 0xC8);
  ok = ok && oledCommand2(0xDA, 0x12);
  ok = ok && oledCommand2(0x81, 0x7F);
  ok = ok && oledCommand2(0xD9, 0x22);
  ok = ok && oledCommand2(0xDB, 0x35);
  ok = ok && oledCommand(0xA4);
  ok = ok && oledCommand(0xA6);
  ok = ok && oledCommand(0xAF);

  oledPresent = ok;
  if (ok) {
    oledClearBuffer();
    oledDrawWakeupFace(millis());
    oledFlush();
  }
  Serial.println(oledPresent ? "OLED ready at 0x3C" : "OLED init failed");
  return oledPresent;
}

void updateOled() {
  if (!oledPresent) {
    return;
  }

  unsigned long now = millis();
  if (now - lastOledUpdateMs < OLED_UPDATE_INTERVAL_MS) {
    return;
  }
  lastOledUpdateMs = now;

  bool openMvConnected = now - lastOpenMvMs < OPENMV_TIMEOUT_MS;

  oledClearBuffer();
  if (now - oledWakeupStartMs < OLED_WAKEUP_DURATION_MS) {
    oledDrawWakeupFace(now);
  } else if (!lastNanoOk) {
    oledDrawErrorFace();
  } else if (autoTrackEnabled && !openMvConnected) {
    oledDrawSearchingFace(now);
  } else if (autoTrackEnabled) {
    oledDrawFocusFace(now);
  } else {
    oledDrawCenterFace(now);
  }
  oledFlush();
}

void setCurrentAngles(int yaw, int pitch) {
  currentYaw = constrain(yaw, YAW_MIN, YAW_MAX);
  currentPitch = constrain(pitch, PITCH_MIN, PITCH_MAX);
  sendToNano(currentYaw, currentPitch);
}

void setAutoTrackEnabled(bool enabled) {
  autoTrackEnabled = enabled;
}

void handleOpenMvLine(char *line) {
  char *comma = strchr(line, ',');
  if (comma == NULL) {
    return;
  }

  *comma = '\0';
  int yaw = atoi(line);
  int pitch = atoi(comma + 1);

  lastOpenMvMs = millis();
  if (autoTrackEnabled) {
    setCurrentAngles(yaw, pitch);
  }
}

void readOpenMvSerial() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      openmvLine[openmvLineLength] = '\0';
      if (openmvLineLength > 0) {
        handleOpenMvLine(openmvLine);
      }
      openmvLineLength = 0;
      continue;
    }

    if (openmvLineLength < sizeof(openmvLine) - 1) {
      openmvLine[openmvLineLength++] = c;
    } else {
      openmvLineLength = 0;
    }
  }
}

void sendOpenMvHeartbeat() {
  unsigned long now = millis();
  if (now - lastOpenMvHeartbeatMs < OPENMV_HEARTBEAT_INTERVAL_MS) {
    return;
  }
  lastOpenMvHeartbeatMs = now;
  Serial1.print("OK\n");
}

void sendOpenMvConfig() {
  Serial1.print("CFG,");
  Serial1.print(cfgYawKp, 4);
  Serial1.print(",");
  Serial1.print(cfgPitchKp, 4);
  Serial1.print(",");
  Serial1.print(cfgDeadZone);
  Serial1.print(",");
  Serial1.print(cfgMaxStep, 2);
  Serial1.print(",");
  Serial1.print(cfgControlMs);
  Serial1.print(",");
  Serial1.print(cfgFilterAlpha, 2);
  Serial1.print(",");
  Serial1.print(cfgTargetMode);
  Serial1.print("\n");
}

void sendOpenMvConfigPeriodically() {
  unsigned long now = millis();
  if (now - lastOpenMvConfigMs < OPENMV_CONFIG_INTERVAL_MS) {
    return;
  }
  lastOpenMvConfigMs = now;
  sendOpenMvConfig();
}

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleSet() {
  if (!server.hasArg("yaw") || !server.hasArg("pitch")) {
    server.send(400, "text/plain", "missing yaw or pitch");
    return;
  }

  currentYaw = constrain(server.arg("yaw").toInt(), YAW_MIN, YAW_MAX);
  currentPitch = constrain(server.arg("pitch").toInt(), PITCH_MIN, PITCH_MAX);
  setAutoTrackEnabled(false);

  bool ok = sendToNano(currentYaw, currentPitch);

  if (ok) {
    server.send(200, "application/json",
                String("{\"ok\":true,\"yaw\":") + currentYaw +
                ",\"pitch\":" + currentPitch + "}");
  } else {
    server.send(502, "application/json", "{\"ok\":false,\"error\":\"i2c\"}");
  }
}

void handleStatus() {
  server.send(200, "application/json",
              String("{\"yaw\":") + currentYaw +
              ",\"pitch\":" + currentPitch +
              ",\"auto\":" + (autoTrackEnabled ? "true" : "false") +
              ",\"openmv\":" + (millis() - lastOpenMvMs < OPENMV_TIMEOUT_MS ? "true" : "false") +
              ",\"ykp\":" + String(cfgYawKp, 4) +
              ",\"pkp\":" + String(cfgPitchKp, 4) +
              ",\"dz\":" + cfgDeadZone +
              ",\"step\":" + String(cfgMaxStep, 2) +
              ",\"ms\":" + cfgControlMs +
              ",\"alpha\":" + String(cfgFilterAlpha, 2) +
              ",\"target\":" + cfgTargetMode +
              "}");
}

void handleMode() {
  if (!server.hasArg("auto")) {
    server.send(400, "text/plain", "missing auto");
    return;
  }

  setAutoTrackEnabled(server.arg("auto").toInt() != 0);
  server.send(200, "application/json",
              String("{\"auto\":") + (autoTrackEnabled ? "true" : "false") + "}");
}

void applyConfigArgs() {
  if (server.hasArg("ykp")) {
    cfgYawKp = server.arg("ykp").toFloat();
  }
  if (server.hasArg("pkp")) {
    cfgPitchKp = server.arg("pkp").toFloat();
  }
  if (server.hasArg("dz")) {
    cfgDeadZone = server.arg("dz").toInt();
  }
  if (server.hasArg("step")) {
    cfgMaxStep = server.arg("step").toFloat();
  }
  if (server.hasArg("ms")) {
    cfgControlMs = server.arg("ms").toInt();
  }
  if (server.hasArg("alpha")) {
    cfgFilterAlpha = server.arg("alpha").toFloat();
  }
  if (server.hasArg("target")) {
    cfgTargetMode = server.arg("target").toInt();
  }
  constrainConfig();
}

String configJson(bool ok) {
  return String("{\"ok\":") + (ok ? "true" : "false") +
         ",\"ykp\":" + String(cfgYawKp, 4) +
         ",\"pkp\":" + String(cfgPitchKp, 4) +
         ",\"dz\":" + cfgDeadZone +
         ",\"step\":" + String(cfgMaxStep, 2) +
         ",\"ms\":" + cfgControlMs +
         ",\"alpha\":" + String(cfgFilterAlpha, 2) +
         ",\"target\":" + cfgTargetMode +
         "}";
}

void handleConfig() {
  applyConfigArgs();
  sendOpenMvConfig();
  server.send(200, "application/json", configJson(true));
}

void handleSave() {
  applyConfigArgs();
  sendOpenMvConfig();
  bool ok = saveConfigToFlash();
  Serial.print("Save config to flash: ");
  Serial.println(ok ? "OK" : "FAILED");
  Serial.print("  ykp=");
  Serial.print(cfgYawKp, 4);
  Serial.print(" pkp=");
  Serial.print(cfgPitchKp, 4);
  Serial.print(" dz=");
  Serial.print(cfgDeadZone);
  Serial.print(" step=");
  Serial.print(cfgMaxStep, 2);
  Serial.print(" ms=");
  Serial.print(cfgControlMs);
  Serial.print(" alpha=");
  Serial.print(cfgFilterAlpha, 2);
  Serial.print(" target=");
  Serial.println(cfgTargetMode == TARGET_RED_CIRCLE ? "red-circle" : "face");
  server.send(ok ? 200 : 500, "application/json", configJson(ok));
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(57600, SERIAL_8N1, OPENMV_RX_PIN, OPENMV_TX_PIN);
  delay(300);
  loadConfigFromFlash();
  oledWakeupStartMs = millis();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  oledInit();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.println(apStarted ? "Wi-Fi hotspot started" : "Wi-Fi hotspot failed");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD[0] == '\0' ? "(open)" : AP_PASSWORD);
  Serial.print("Open http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.on("/mode", handleMode);
  server.on("/cfg", handleConfig);
  server.on("/save", handleSave);
  server.begin();

  sendToNano(YAW_CENTER, PITCH_CENTER);
  sendOpenMvConfig();
}

void loop() {
  readOpenMvSerial();
  sendOpenMvHeartbeat();
  sendOpenMvConfigPeriodically();
  server.handleClient();
  updateOled();
}
