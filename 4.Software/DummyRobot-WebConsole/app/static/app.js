import * as THREE from "/assets/vendor/three.module.js";
import { OrbitControls } from "/assets/vendor/OrbitControls.js";
import { STLLoader } from "/assets/vendor/STLLoader.js";

const POLL_INTERVAL_MS = 900;
const JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"];
const POSE_NAMES = ["X", "Y", "Z", "A", "B", "C"];
const STORAGE_LANG_KEY = "dummy-webconsole-lang";
const STORAGE_SO101_PORT_KEY = "dummy-webconsole-so101-port";
const BUILD_ID = "20260415j";

const I18N = {
  zh: {
    "hero.kicker": "\u0044\u0075\u006d\u006d\u0079\u0052\u006f\u0062\u006f\u0074 / Web \u4e0a\u4f4d\u673a\u63a7\u5236\u53f0",
    "hero.title": "Dummy WEBUI",
    "viewer.tag": "\u5b9e\u65f6\u89c6\u56fe",
    "viewer.title": "3D \u673a\u68b0\u81c2\u59ff\u6001",
    "viewer.connection": "\u8fde\u63a5\u72b6\u6001",
    "viewer.enabled": "\u4f7f\u80fd\u72b6\u6001",
    "viewer.response": "\u6700\u540e\u54cd\u5e94",
    "connection.tag": "\u8bbe\u5907\u63a5\u5165",
    "connection.title": "\u8fde\u63a5",
    "connection.mode": "\u8bbe\u5907\u6a21\u5f0f",
    "connection.modeReal": "\u771f\u5b9e\u8bbe\u5907",
    "connection.modeVirtual": "\u865a\u62df\u8bbe\u5907",
    "connection.virtualHint": "\u865a\u62df\u6a21\u5f0f\u4f7f\u7528\u6570\u5b57\u5b6a\u751f\uff0c\u65e0\u9700\u4e32\u53e3\u786c\u4ef6\u5373\u53ef\u4f53\u9a8c\u3002",
    "connection.port": "\u4e32\u53e3",
    "connection.baudrate": "\u6ce2\u7279\u7387",
    "connection.timeout": "\u8d85\u65f6",
    "status.tag": "\u8bbe\u5907\u72b6\u6001",
    "status.title": "\u72b6\u6001\u5feb\u7167",
    "status.mode": "\u6a21\u5f0f",
    "status.port": "\u8fde\u63a5\u7aef",
    "status.updated": "\u66f4\u65b0\u65f6\u95f4",
    "status.command": "\u6700\u540e\u547d\u4ee4",
    "status.error": "\u6700\u540e\u9519\u8bef",
    "commands.tag": "\u5feb\u6377\u64cd\u4f5c",
    "commands.title": "\u673a\u5668\u4eba\u547d\u4ee4",
    "commands.raw": "\u539f\u59cb\u547d\u4ee4",
    "commands.rawPlaceholder": "#GETJPOS \u6216 &0,0,90,0,0,0",
    "telemetry.tag": "\u5b9e\u65f6\u9065\u6d4b",
    "telemetry.joints": "\u5173\u8282\u89d2",
    "telemetry.toolTag": "\u5de5\u5177\u5750\u6807",
    "telemetry.pose": "\u672b\u7aef\u4f4d\u59ff",
    "manual.tag": "\u624b\u52a8\u70b9\u52a8",
    "manual.title": "\u5355\u5173\u8282\u63a7\u5236",
    "manual.note": "\u6bcf\u4e2a\u5173\u8282\u53ef\u5355\u72ec\u53d1\u9001\uff0c\u4e5f\u53ef\u4e00\u6b21\u6027\u4e0b\u53d1\u5168\u90e8\u516d\u8f74\u76ee\u6807\u3002",
    "manual.webDriven": "\u5f53\u524d\u7531 Web \u754c\u9762\u9a71\u52a8\u865a\u62df Dummy \u673a\u68b0\u81c2\u3002",
    "manual.so101Driven": "\u5f53\u524d\u7531 SO101 \u5b9e\u65f6\u955c\u50cf\u9a71\u52a8\uff0cWeb \u624b\u52a8\u5173\u8282\u63a7\u5236\u5df2\u53ea\u8bfb\u3002",
    "teleop.tag": "\u9065\u64cd\u6865\u63a5",
    "teleop.title": "\u9065\u64cd\u6e90",
    "teleop.source": "\u9065\u64cd\u6e90",
    "teleop.sourceManual": "Manual Web",
    "teleop.sourceSo101": "SO101 Mirror",
    "teleop.port": "SO101 \u7aef\u53e3",
    "teleop.rate": "\u6e90\u5237\u65b0\u9891\u7387",
    "teleop.start": "\u542f\u52a8\u955c\u50cf",
    "teleop.stop": "\u505c\u6b62\u955c\u50cf",
    "teleop.noteManual": "\u865a\u62df Dummy \u5f53\u524d\u7531 Web \u754c\u9762\u63a7\u5236\u3002",
    "teleop.noteReady": "SO101 \u955c\u50cf\u5df2\u9009\u4e2d\uff0c\u8fde\u63a5\u865a\u62df\u8bbe\u5907\u540e\u53ef\u542f\u52a8\u3002",
    "teleop.noteActive": "SO101 \u955c\u50cf\u8fd0\u884c\u4e2d\uff0c\u6b63\u5728\u5b9e\u65f6\u9a71\u52a8\u865a\u62df Dummy\u3002",
    "teleop.noteVirtualOnly": "\u9065\u64cd\u6865\u63a5\u4ec5\u5728\u865a\u62df\u8bbe\u5907\u6a21\u5f0f\u4e0b\u53ef\u7528\u3002",
    "teleop.stateSource": "\u6e90",
    "teleop.stateActive": "\u6fc0\u6d3b",
    "teleop.stateConnected": "\u5df2\u8fde\u63a5",
    "teleop.stateSample": "\u6700\u540e\u91c7\u6837",
    "teleop.stateFps": "\u6e90 FPS",
    "teleop.stateError": "\u6700\u540e\u9519\u8bef",
    "debug.tag": "\u8c03\u8bd5\u8f7d\u8377",
    "debug.title": "\u539f\u59cb\u72b6\u6001 JSON",
    "debug.note": "\u524d\u7aef\u901a\u8fc7 /api/state \u8f6e\u8be2\uff0c\u5e76\u7528\u4e8e 3D \u4e0e\u72b6\u6001\u540c\u6b65\u3002",
    "actions.refresh": "\u5237\u65b0",
    "actions.connect": "\u8fde\u63a5",
    "actions.disconnect": "\u65ad\u5f00",
    "actions.enable": "\u4f7f\u80fd",
    "actions.disable": "\u5931\u80fd",
    "actions.home": "\u56de\u96f6",
    "actions.reset": "\u590d\u4f4d",
    "actions.stop": "\u6025\u505c",
    "actions.send": "\u53d1\u9001",
    "actions.sendAllJoints": "\u53d1\u9001\u5168\u90e8\u5173\u8282",
    "actions.resetView": "\u91cd\u7f6e\u89c6\u89d2",
    "status.connected": "\u5df2\u8fde\u63a5",
    "status.disconnected": "\u672a\u8fde\u63a5",
    "status.online": "\u5728\u7ebf",
    "status.offline": "\u79bb\u7ebf",
    "status.safeIdle": "\u7a7a\u95f2 / \u5b89\u5168",
    "status.motorsEnabled": "\u7535\u673a\u5df2\u4f7f\u80fd",
    "status.noDevices": "\u672a\u68c0\u6d4b\u5230\u4e32\u53e3\u8bbe\u5907",
    "status.modeReal": "\u771f\u5b9e",
    "status.modeVirtual": "\u865a\u62df",
    "status.virtualSession": "\u865a\u62df\u4f1a\u8bdd",
    "status.active": "\u6fc0\u6d3b",
    "status.inactive": "\u672a\u6fc0\u6d3b",
    "status.yes": "\u662f",
    "status.no": "\u5426",
    "viewer.portPrefix": "\u4f1a\u8bdd",
    "viewer.syncPrefix": "\u540c\u6b65",
    "viewer.modelState": "\u6a21\u578b\u72b6\u6001",
    "viewer.centered": "\u7a33\u5b9a\u5173\u8282\u6a21\u578b",
    "viewer.realModel": "\u771f\u5b9e STL \u6a21\u578b",
    "viewer.fallbackModel": "\u7b80\u5316\u9aa8\u67b6\u6a21\u578b",
    "joint.axis": "\u8f74",
    "joint.send": "\u53d1\u9001",
    "joint.minus": "-5 \u5ea6",
    "joint.plus": "+5 \u5ea6",
    "joint.unit": "\u5ea6",
  },
  en: {
    "hero.kicker": "DummyRobot / Web Operator Console",
    "hero.title": "Dummy WEBUI",
    "viewer.tag": "Realtime View",
    "viewer.title": "3D Robot Pose",
    "viewer.connection": "Connection",
    "viewer.enabled": "Enabled",
    "viewer.response": "Last Response",
    "connection.tag": "Device Access",
    "connection.title": "Connection",
    "connection.mode": "Device Mode",
    "connection.modeReal": "Real Device",
    "connection.modeVirtual": "Virtual Device",
    "connection.virtualHint": "Virtual mode runs a digital twin without a serial device.",
    "connection.port": "Serial Port",
    "connection.baudrate": "Baudrate",
    "connection.timeout": "Timeout",
    "status.tag": "Machine State",
    "status.title": "Status Snapshot",
    "status.mode": "Mode",
    "status.port": "Endpoint",
    "status.updated": "Updated",
    "status.command": "Last Command",
    "status.error": "Last Error",
    "commands.tag": "Quick Actions",
    "commands.title": "Robot Commands",
    "commands.raw": "Raw Command",
    "commands.rawPlaceholder": "#GETJPOS or &0,0,90,0,0,0",
    "telemetry.tag": "Live Telemetry",
    "telemetry.joints": "Joint Angles",
    "telemetry.toolTag": "Tool Frame",
    "telemetry.pose": "End Effector Pose",
    "manual.tag": "Manual Jog",
    "manual.title": "Single Joint Control",
    "manual.note": "Each slider can be sent independently, or send all six joints together.",
    "manual.webDriven": "The virtual Dummy robot is currently driven by the web UI.",
    "manual.so101Driven": "SO101 mirror is active. Manual joint controls are now read-only.",
    "teleop.tag": "Teleop Bridge",
    "teleop.title": "Teleop Source",
    "teleop.source": "Teleop Source",
    "teleop.sourceManual": "Manual Web",
    "teleop.sourceSo101": "SO101 Mirror",
    "teleop.port": "SO101 Port",
    "teleop.rate": "Source Hz",
    "teleop.start": "Start Mirror",
    "teleop.stop": "Stop Mirror",
    "teleop.noteManual": "The virtual Dummy robot is currently driven by the web UI.",
    "teleop.noteReady": "SO101 mirror is selected. Connect a virtual device, then start mirroring.",
    "teleop.noteActive": "SO101 mirror is active and is currently driving the virtual Dummy robot.",
    "teleop.noteVirtualOnly": "The teleop bridge is only available in virtual device mode.",
    "teleop.stateSource": "Source",
    "teleop.stateActive": "Active",
    "teleop.stateConnected": "Connected",
    "teleop.stateSample": "Last Sample",
    "teleop.stateFps": "Source FPS",
    "teleop.stateError": "Last Error",
    "debug.tag": "Debug Payload",
    "debug.title": "Raw State JSON",
    "debug.note": "Polled from /api/state for frontend and 3D sync.",
    "actions.refresh": "Refresh",
    "actions.connect": "Connect",
    "actions.disconnect": "Disconnect",
    "actions.enable": "Enable",
    "actions.disable": "Disable",
    "actions.home": "Home",
    "actions.reset": "Reset",
    "actions.stop": "Stop",
    "actions.send": "Send",
    "actions.sendAllJoints": "Send All Joints",
    "actions.resetView": "Reset View",
    "status.connected": "Connected",
    "status.disconnected": "Disconnected",
    "status.online": "Online",
    "status.offline": "Offline",
    "status.safeIdle": "Safe / Idle",
    "status.motorsEnabled": "Motors Enabled",
    "status.noDevices": "No serial devices detected",
    "status.modeReal": "Real",
    "status.modeVirtual": "Virtual",
    "status.virtualSession": "Virtual Session",
    "status.active": "Active",
    "status.inactive": "Inactive",
    "status.yes": "Yes",
    "status.no": "No",
    "viewer.portPrefix": "SOURCE",
    "viewer.syncPrefix": "SYNC",
    "viewer.modelState": "Model State",
    "viewer.centered": "Stable articulated rig",
    "viewer.realModel": "Real STL model",
    "viewer.fallbackModel": "Simplified rig",
    "joint.axis": "Axis",
    "joint.send": "Send",
    "joint.minus": "-5 deg",
    "joint.plus": "+5 deg",
    "joint.unit": "deg",
  },
};

const stateJson = document.getElementById("state-json");
const portSelect = document.getElementById("port-select");
const portField = document.getElementById("port-field");
const virtualHint = document.getElementById("virtual-hint");
const refreshPortsButton = document.getElementById("refresh-ports");
const serialSettings = document.getElementById("serial-settings");
const connectionPill = document.getElementById("connection-pill");
const modePill = document.getElementById("mode-pill");
const jointControlContainer = document.getElementById("joint-controls");
const jointBars = document.getElementById("joint-bars");
const poseBars = document.getElementById("pose-bars");
const rawCommandInput = document.getElementById("raw-command");
const teleopPanel = document.getElementById("teleop-panel");
const teleopFields = document.getElementById("teleop-so101-fields");
const teleopPortInput = document.getElementById("teleop-port-input");
const teleopRateInput = document.getElementById("teleop-rate-input");
const teleopStatusNote = document.getElementById("teleop-status-note");
const manualLockNote = document.getElementById("manual-lock-note");
const sendAllJointsButton = document.getElementById("send-joints-btn");

let currentLang = localStorage.getItem(STORAGE_LANG_KEY) || "zh";
let selectedMode = "real";
let jointDrafts = JOINT_NAMES.map(() => null);
let latestState = {
  connected: false,
  enabled: false,
  mode: "real",
  joints: [0, 0, 0, 0, 0, 0],
  pose: [0, 0, 0, 0, 0, 0],
  last_command: null,
  last_response: null,
  last_error: null,
  updated_at: null,
  teleop: {
    source: "manual",
    active: false,
    connected: false,
    fps: null,
    last_sample_at: null,
    last_error: null,
    port: null,
  },
};

syncViewportMode();
const viewer = createRobotViewer(document.getElementById("robot-view"));
teleopPortInput.value = localStorage.getItem(STORAGE_SO101_PORT_KEY) || "";

function t(key) {
  return I18N[currentLang]?.[key] || I18N.en[key] || key;
}

function modeLabel(mode) {
  return mode === "virtual" ? t("status.modeVirtual") : t("status.modeReal");
}

function endpointLabel() {
  const mode = latestState.connected ? latestState.mode : selectedMode;
  if (mode === "virtual") {
    const teleop = teleopState();
    if (teleop.source === "so101" && teleop.active) {
      return teleopSourceLabel("so101");
    }
    return t("status.virtualSession");
  }
  return latestState.port || "-";
}

function formatDegrees(value) {
  return `${Number(value).toFixed(1)} ${t("joint.unit")}`;
}

function formatNumber(value, digits = 2) {
  return Number(value || 0).toFixed(digits);
}

function formatTimestamp(timestamp) {
  if (!timestamp) {
    return "-";
  }
  return new Date(timestamp * 1000).toLocaleTimeString();
}

function teleopSourceLabel(source) {
  return source === "so101" ? t("teleop.sourceSo101") : t("teleop.sourceManual");
}

function teleopState() {
  return latestState.teleop || {
    source: "manual",
    active: false,
    connected: false,
    fps: null,
    last_sample_at: null,
    last_error: null,
    port: null,
  };
}

function effectiveMode() {
  return latestState.connected ? latestState.mode : selectedMode;
}

function isTeleopReadOnly() {
  const teleop = teleopState();
  return effectiveMode() === "virtual" && teleop.source === "so101" && Boolean(teleop.active);
}

function clearJointDrafts() {
  jointDrafts = JOINT_NAMES.map(() => null);
}

function getDraftAwareJoints(serverJoints = latestState.joints || []) {
  return JOINT_NAMES.map((_, index) => {
    const draft = jointDrafts[index];
    return draft === null || draft === undefined ? Number(serverJoints[index] ?? 0) : Number(draft);
  });
}

function shouldUseCompactStage() {
  const dpr = window.devicePixelRatio || 1;
  const viewportWidth = window.visualViewport?.width || window.innerWidth || document.documentElement.clientWidth || 0;
  return viewportWidth <= 1220 || (dpr >= 1.8 && viewportWidth <= 1720);
}

function syncViewportMode() {
  document.body.classList.toggle("compact-stage", shouldUseCompactStage());
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const contentType = response.headers.get("content-type") || "";
  const payload = contentType.includes("application/json") ? await response.json() : await response.text();
  if (!response.ok) {
    const detail = typeof payload === "string" ? payload : payload.detail || JSON.stringify(payload);
    throw new Error(detail || `Request failed: ${response.status}`);
  }
  return payload;
}

function applyTranslations() {
  document.documentElement.lang = currentLang === "zh" ? "zh-CN" : "en";
  document.title = "Dummy WEBUI";
  document.querySelectorAll("[data-i18n]").forEach((element) => {
    element.textContent = t(element.dataset.i18n);
  });
  document.querySelectorAll("[data-i18n-placeholder]").forEach((element) => {
    element.placeholder = t(element.dataset.i18nPlaceholder);
  });
  document.querySelectorAll("[data-lang]").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.lang === currentLang);
  });
  applyModeUI();
  createJointControls();
  applyState(latestState);
}

function applyModeUI() {
  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.mode === selectedMode);
  });
  const formVirtual = selectedMode === "virtual";
  const virtualContext = effectiveMode() === "virtual";
  document.body.classList.toggle("virtual-device-mode", formVirtual);
  portField.hidden = formVirtual;
  serialSettings.hidden = formVirtual;
  virtualHint.hidden = !formVirtual;
  refreshPortsButton.hidden = formVirtual;
  refreshPortsButton.disabled = formVirtual;
  teleopPanel.hidden = !virtualContext;
  teleopFields.hidden = teleopState().source !== "so101";
}

function createJointControls() {
  jointControlContainer.innerHTML = "";
  const controlsLocked = isTeleopReadOnly();
  JOINT_NAMES.forEach((jointName, index) => {
    const currentValue = latestState.joints?.[index] ?? 0;
    const card = document.createElement("div");
    card.className = "joint-card";
    card.innerHTML = `
      <div class="joint-card-head">
        <div>
          <span>${jointName}</span>
          <strong id="joint-value-${index}">${formatDegrees(currentValue)}</strong>
        </div>
        <span>${t("joint.axis")} ${index + 1}</span>
      </div>
      <input type="range" min="-180" max="180" step="1" value="${Number(currentValue)}" data-joint-index="${index}" ${controlsLocked ? "disabled" : ""}>
      <div class="joint-actions">
        <button class="ghost" data-step-joint="${index}" data-step="-5" ${controlsLocked ? "disabled" : ""}>${t("joint.minus")}</button>
        <button data-send-joint="${index}" ${controlsLocked ? "disabled" : ""}>${t("joint.send")}</button>
        <button class="ghost" data-step-joint="${index}" data-step="5" ${controlsLocked ? "disabled" : ""}>${t("joint.plus")}</button>
      </div>
    `;
    jointControlContainer.appendChild(card);
  });
}

function syncTeleopUI() {
  const teleop = teleopState();
  const virtualContext = effectiveMode() === "virtual";
  const controlsLocked = isTeleopReadOnly();

  if (controlsLocked) {
    clearJointDrafts();
  }

  teleopPanel.hidden = !virtualContext;
  teleopFields.hidden = teleop.source !== "so101";
  document.querySelectorAll("[data-teleop-source]").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.teleopSource === teleop.source);
  });

  document.getElementById("teleop-state-source").textContent = teleopSourceLabel(teleop.source);
  document.getElementById("teleop-state-active").textContent = teleop.active ? t("status.active") : t("status.inactive");
  document.getElementById("teleop-state-connected").textContent = teleop.connected ? t("status.yes") : t("status.no");
  document.getElementById("teleop-state-sample").textContent = formatTimestamp(teleop.last_sample_at);
  document.getElementById("teleop-state-fps").textContent =
    teleop.fps !== null && teleop.fps !== undefined ? formatNumber(teleop.fps, 1) : "-";
  document.getElementById("teleop-state-error").textContent = teleop.last_error || "-";

  if (teleop.port && teleop.source === "so101" && teleopPortInput.value !== teleop.port) {
    teleopPortInput.value = teleop.port;
    localStorage.setItem(STORAGE_SO101_PORT_KEY, teleop.port);
  }

  const teleopReady = virtualContext && teleop.source === "so101";
  document.getElementById("teleop-start-btn").disabled = !teleopReady || teleop.active;
  document.getElementById("teleop-stop-btn").disabled = !teleop.active;
  sendAllJointsButton.disabled = controlsLocked;

  document.querySelectorAll("#joint-controls input, #joint-controls button").forEach((element) => {
    element.disabled = controlsLocked;
  });

  manualLockNote.textContent = controlsLocked ? t("manual.so101Driven") : t("manual.webDriven");
  if (!virtualContext) {
    teleopStatusNote.textContent = t("teleop.noteVirtualOnly");
  } else if (teleop.active && teleop.source === "so101") {
    teleopStatusNote.textContent = t("teleop.noteActive");
  } else if (teleop.source === "so101") {
    teleopStatusNote.textContent = t("teleop.noteReady");
  } else {
    teleopStatusNote.textContent = t("teleop.noteManual");
  }
}

function renderBarList(container, values, labels, maxAbs) {
  container.innerHTML = "";
  labels.forEach((label, index) => {
    const value = Number(values?.[index] ?? 0);
    const normalized = ((value + maxAbs) / (maxAbs * 2)) * 100;
    const width = Math.max(0, Math.min(100, normalized));
    const card = document.createElement("div");
    card.className = "bar-card";
    card.innerHTML = `
      <div class="bar-head">
        <span>${label}</span>
        <strong>${formatNumber(value)}</strong>
      </div>
      <div class="bar-track"><div class="bar-fill" style="width:${width}%"></div></div>
    `;
    container.appendChild(card);
  });
}

function updateJointInputs(joints = []) {
  document.querySelectorAll("[data-joint-index]").forEach((inputElement) => {
    const index = Number(inputElement.dataset.jointIndex);
    const draft = jointDrafts[index];
    const value = Number(draft === null || draft === undefined ? joints[index] ?? 0 : draft);
    inputElement.value = String(value);
    const label = document.getElementById(`joint-value-${index}`);
    if (label) {
      label.textContent = formatDegrees(value);
    }
  });
}

function applyState(state) {
  latestState = {
    ...latestState,
    ...state,
    mode: state.mode || latestState.mode || selectedMode,
    joints: state.joints || latestState.joints || [0, 0, 0, 0, 0, 0],
    pose: state.pose || latestState.pose || [0, 0, 0, 0, 0, 0],
    teleop: {
      ...teleopState(),
      ...(state.teleop || {}),
    },
  };

  selectedMode = latestState.connected ? latestState.mode : selectedMode;
  applyModeUI();

  const updatedTime = latestState.updated_at ? new Date(latestState.updated_at * 1000).toLocaleTimeString() : "-";
  const modeText = modeLabel(latestState.mode);
  const endpointText = endpointLabel();
  const connectionText = latestState.connected
    ? latestState.mode === "virtual"
      ? `${t("status.connected")} / ${endpointText}`
      : `${t("status.connected")} ${endpointText}`.trim()
    : t("status.disconnected");
  const teleop = teleopState();
  const statusText = latestState.mode === "virtual" && teleop.source === "so101" && teleop.active
    ? `${teleopSourceLabel("so101")} / ${t("status.active")}`
    : latestState.enabled
      ? t("status.motorsEnabled")
      : `${modeText} / ${t("status.safeIdle")}`;

  connectionPill.textContent = connectionText;
  modePill.textContent = statusText;
  document.getElementById("state-connection").textContent = latestState.connected ? t("status.online") : t("status.offline");
  document.getElementById("state-enabled").textContent = String(Boolean(latestState.enabled));
  document.getElementById("state-response").textContent = latestState.last_response || t("viewer.centered");
  document.getElementById("state-mode").textContent = modeText;
  document.getElementById("state-port").textContent = endpointText;
  document.getElementById("state-updated").textContent = updatedTime;
  document.getElementById("state-command").textContent = latestState.last_command || "-";
  document.getElementById("state-error").textContent = latestState.last_error || "-";
  document.getElementById("viewer-port").textContent = `${t("viewer.portPrefix")} ${endpointText}`;
  document.getElementById("viewer-updated").textContent = `${t("viewer.syncPrefix")} ${updatedTime}`;

  renderBarList(jointBars, latestState.joints, latestState.joint_names || JOINT_NAMES, 180);
  renderBarList(poseBars, latestState.pose, latestState.pose_labels || POSE_NAMES, 320);
  updateJointInputs(latestState.joints);
  syncTeleopUI();
  viewer.setSessionMode(latestState.mode, latestState.connected);
  viewer.setJointAngles(isTeleopReadOnly() ? latestState.joints : getDraftAwareJoints(latestState.joints));
  viewer.setConnectionState(latestState.connected);
  stateJson.textContent = JSON.stringify(latestState, null, 2);
}

async function refreshPorts() {
  if (selectedMode === "virtual") {
    portSelect.innerHTML = "";
    const option = document.createElement("option");
    option.textContent = "VIRTUAL";
    option.value = "VIRTUAL";
    portSelect.appendChild(option);
    return;
  }

  const payload = await api("/api/ports");
  const ports = payload.ports || [];
  portSelect.innerHTML = "";
  if (!ports.length) {
    const option = document.createElement("option");
    option.textContent = t("status.noDevices");
    option.value = "";
    portSelect.appendChild(option);
    return;
  }
  ports.forEach((port) => {
    const option = document.createElement("option");
    option.value = port;
    option.textContent = port;
    portSelect.appendChild(option);
  });
}

async function pollState() {
  try {
    applyState(await api("/api/state"));
  } catch (error) {
    document.getElementById("state-error").textContent = error.message;
    stateJson.textContent = error.message;
    viewer.setConnectionState(false);
  }
}

async function runAction(action) {
  try {
    await action();
  } catch (error) {
    document.getElementById("state-error").textContent = error.message;
    stateJson.textContent = error.message;
  }
}

async function sendSingleJoint(index) {
  const slider = document.querySelector(`[data-joint-index="${index}"]`);
  const target = Number(jointDrafts[index] === null || jointDrafts[index] === undefined ? slider.value : jointDrafts[index]);
  await api("/api/move/joint", {
    method: "POST",
    body: JSON.stringify({
      joint_index: index + 1,
      target,
      speed: 10,
      sequential: false,
    }),
  });
  jointDrafts[index] = null;
  await pollState();
}

function bindLanguageToggle() {
  document.querySelectorAll("[data-lang]").forEach((button) => {
    button.addEventListener("click", async () => {
      currentLang = button.dataset.lang;
      localStorage.setItem(STORAGE_LANG_KEY, currentLang);
      applyTranslations();
      await refreshPorts();
    });
  });
}

function bindModeToggle() {
  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.addEventListener("click", async () => {
      selectedMode = button.dataset.mode;
      applyModeUI();
      await refreshPorts();
    });
  });
}

function bindActions() {
  refreshPortsButton.addEventListener("click", () => runAction(refreshPorts));
  document.getElementById("reset-view-btn").addEventListener("click", () => viewer.resetView());
  teleopPortInput.addEventListener("change", () => {
    localStorage.setItem(STORAGE_SO101_PORT_KEY, teleopPortInput.value.trim());
  });

  document.getElementById("connect-btn").addEventListener("click", async () => {
    await runAction(async () => {
      clearJointDrafts();
      await api("/api/connect", {
        method: "POST",
        body: JSON.stringify({
          mode: selectedMode,
          port: selectedMode === "virtual" ? "" : portSelect.value,
          baudrate: Number(document.getElementById("baudrate-input").value),
          timeout: Number(document.getElementById("timeout-input").value),
        }),
      });
      await pollState();
    });
  });

  document.querySelectorAll("[data-teleop-source]").forEach((button) => {
    button.addEventListener("click", async () => {
      await runAction(async () => {
        const payload = await api("/api/teleop/source", {
          method: "POST",
          body: JSON.stringify({ source: button.dataset.teleopSource }),
        });
        applyState(payload.state || latestState);
      });
    });
  });

  document.getElementById("teleop-start-btn").addEventListener("click", async () => {
    await runAction(async () => {
      localStorage.setItem(STORAGE_SO101_PORT_KEY, teleopPortInput.value.trim());
      clearJointDrafts();
      const payload = await api("/api/teleop/start", {
        method: "POST",
        body: JSON.stringify({
          port: teleopPortInput.value.trim(),
          poll_hz: Number(teleopRateInput.value || 20),
          leader_id: "dummy-webui-so101",
        }),
      });
      applyState(payload.state || latestState);
    });
  });

  document.getElementById("teleop-stop-btn").addEventListener("click", async () => {
    await runAction(async () => {
      const payload = await api("/api/teleop/stop", {
        method: "POST",
        body: JSON.stringify({}),
      });
      clearJointDrafts();
      applyState(payload.state || latestState);
    });
  });

  document.getElementById("disconnect-btn").addEventListener("click", async () => {
    await runAction(async () => {
      clearJointDrafts();
      await api("/api/disconnect", { method: "POST" });
      await pollState();
    });
  });

  document.querySelectorAll("[data-command]").forEach((button) => {
    button.addEventListener("click", async () => {
      await runAction(async () => {
        await api(`/api/commands/${button.dataset.command}`, { method: "POST" });
        await pollState();
      });
    });
  });

  document.getElementById("send-raw-btn").addEventListener("click", async () => {
    const command = rawCommandInput.value.trim();
    if (!command) return;
    await runAction(async () => {
      await api("/api/commands/raw", {
        method: "POST",
        body: JSON.stringify({ command, expect_response: true }),
      });
      await pollState();
    });
  });

  document.getElementById("send-joints-btn").addEventListener("click", async () => {
    await runAction(async () => {
      const joints = getDraftAwareJoints();
      await api("/api/move/joints", {
        method: "POST",
        body: JSON.stringify({ joints, speed: 10, sequential: false }),
      });
      clearJointDrafts();
      await pollState();
    });
  });

  jointControlContainer.addEventListener("click", async (event) => {
    const target = event.target;
    if (!(target instanceof HTMLElement)) return;
    if (isTeleopReadOnly()) return;

    if (target.dataset.sendJoint !== undefined) {
      await runAction(async () => {
        await sendSingleJoint(Number(target.dataset.sendJoint));
      });
      return;
    }

    if (target.dataset.stepJoint !== undefined) {
      const jointIndex = Number(target.dataset.stepJoint);
      const delta = Number(target.dataset.step);
      const slider = document.querySelector(`[data-joint-index="${jointIndex}"]`);
      const nextValue = Math.max(-180, Math.min(180, Number(slider.value) + delta));
      jointDrafts[jointIndex] = nextValue;
      slider.value = String(nextValue);
      document.getElementById(`joint-value-${jointIndex}`).textContent = formatDegrees(nextValue);
      const previewAngles = getDraftAwareJoints();
      viewer.setJointAngles(previewAngles);
    }
  });

  jointControlContainer.addEventListener("input", (event) => {
    const target = event.target;
    if (!(target instanceof HTMLInputElement) || target.dataset.jointIndex === undefined) return;
    if (isTeleopReadOnly()) return;
    const index = Number(target.dataset.jointIndex);
    jointDrafts[index] = Number(target.value);
    document.getElementById(`joint-value-${index}`).textContent = formatDegrees(target.value);
    const previewAngles = getDraftAwareJoints();
    viewer.setJointAngles(previewAngles);
  });
}

function bindViewportMode() {
  let rafId = 0;
  let lastCompact = document.body.classList.contains("compact-stage");

  const updateLayout = () => {
    rafId = 0;
    syncViewportMode();
    const nextCompact = document.body.classList.contains("compact-stage");
    if (nextCompact !== lastCompact) {
      lastCompact = nextCompact;
      viewer.resetView();
      return;
    }
  };

  const scheduleUpdate = () => {
    if (rafId) {
      return;
    }
    rafId = window.requestAnimationFrame(updateLayout);
  };

  window.addEventListener("resize", scheduleUpdate);
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", scheduleUpdate);
  }
}

function createRobotViewer(container) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf6fafe);
  scene.fog = new THREE.Fog(0xf6fafe, 9, 22);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.appendChild(renderer.domElement);

  const camera = new THREE.PerspectiveCamera(40, 1, 0.1, 100);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI * 0.49;
  controls.minDistance = 3.4;
  controls.maxDistance = 12;

  scene.add(new THREE.AmbientLight(0xffffff, 1.45));

  const keyLight = new THREE.DirectionalLight(0xffffff, 1.45);
  keyLight.position.set(5, 7, 6);
  scene.add(keyLight);

  const rimLight = new THREE.DirectionalLight(0x8ed7d3, 0.95);
  rimLight.position.set(-5, 5, -4);
  scene.add(rimLight);

  const grid = new THREE.GridHelper(10, 20, 0x9ec5f0, 0xd9e6f3);
  scene.add(grid);

  const floor = new THREE.Mesh(
    new THREE.CircleGeometry(3.6, 64),
    new THREE.MeshStandardMaterial({
      color: 0xffffff,
      roughness: 0.9,
      metalness: 0.06,
      transparent: true,
      opacity: 0.96,
    }),
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -0.001;
  scene.add(floor);

  const materials = {
    shell: new THREE.MeshStandardMaterial({ color: 0x66a8f3, roughness: 0.36, metalness: 0.7 }),
    arm: new THREE.MeshStandardMaterial({ color: 0x59cabf, roughness: 0.38, metalness: 0.54 }),
    wrist: new THREE.MeshStandardMaterial({ color: 0xe7a06d, roughness: 0.32, metalness: 0.54 }),
    dark: new THREE.MeshStandardMaterial({ color: 0x7e97b1, roughness: 0.58, metalness: 0.18 }),
  };

  const robotRoot = new THREE.Group();
  scene.add(robotRoot);
  const primitiveMeshes = [];

  function track(mesh) {
    primitiveMeshes.push(mesh);
    return mesh;
  }

  const baseBody = track(new THREE.Mesh(new THREE.BoxGeometry(1.18, 0.8, 1.08), materials.dark));
  baseBody.position.set(0, 0.4, 0);
  robotRoot.add(baseBody);

  const baseCap = track(new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.46, 0.16, 48), materials.shell));
  baseCap.position.set(0, 0.88, 0);
  robotRoot.add(baseCap);

  const halo = new THREE.Mesh(
    new THREE.TorusGeometry(1.12, 0.018, 12, 96),
    new THREE.MeshBasicMaterial({ color: 0x55cab9, transparent: true, opacity: 0.22 }),
  );
  halo.rotation.x = Math.PI / 2;
  halo.position.y = 0.01;
  scene.add(halo);

  const j1 = new THREE.Group();
  j1.position.set(0, 0.96, 0);
  robotRoot.add(j1);

  const shoulderTower = track(new THREE.Mesh(new THREE.CylinderGeometry(0.24, 0.27, 0.52, 40), materials.arm));
  shoulderTower.position.set(0, 0.26, 0);
  j1.add(shoulderTower);

  const j2 = new THREE.Group();
  j2.position.set(0, 0.48, 0);
  j1.add(j2);

  const shoulderShell = track(new THREE.Mesh(new THREE.CapsuleGeometry(0.2, 0.46, 6, 12), materials.arm));
  shoulderShell.rotation.z = Math.PI / 2;
  j2.add(shoulderShell);

  const upperArm = track(new THREE.Mesh(new THREE.BoxGeometry(0.34, 1.72, 0.34), materials.shell));
  upperArm.position.set(0, 0.86, 0);
  j2.add(upperArm);

  const j3 = new THREE.Group();
  j3.position.set(0, 1.72, 0);
  j2.add(j3);

  const elbowShell = track(new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.34, 32), materials.arm));
  elbowShell.rotation.z = Math.PI / 2;
  j3.add(elbowShell);

  const forearm = track(new THREE.Mesh(new THREE.BoxGeometry(0.28, 1.36, 0.28), materials.shell));
  forearm.position.set(0, 0.68, 0);
  j3.add(forearm);

  const j4 = new THREE.Group();
  j4.position.set(0, 1.36, 0);
  j3.add(j4);

  const wristBase = track(new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.4, 28), materials.arm));
  wristBase.rotation.x = Math.PI / 2;
  wristBase.position.set(0, 0, 0.02);
  j4.add(wristBase);

  const j5 = new THREE.Group();
  j5.position.set(0, 0, 0.28);
  j4.add(j5);

  const wristLink = track(new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.22, 0.58), materials.wrist));
  wristLink.position.set(0, 0, 0.29);
  j5.add(wristLink);

  const j6 = new THREE.Group();
  j6.position.set(0, 0, 0.58);
  j5.add(j6);

  const toolMount = track(new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.26, 24), materials.shell));
  toolMount.rotation.z = Math.PI / 2;
  toolMount.position.set(0, 0, 0.12);
  j6.add(toolMount);

  const gripper = new THREE.Group();
  gripper.position.set(0.18, 0, 0.22);
  j6.add(gripper);

  const jawLeft = track(new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.06, 0.08), materials.wrist));
  jawLeft.position.set(0.14, 0.09, 0);
  gripper.add(jawLeft);

  const jawRight = track(new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.06, 0.08), materials.wrist));
  jawRight.position.set(0.14, -0.09, 0);
  gripper.add(jawRight);

  const toolFrame = new THREE.AxesHelper(0.45);
  toolFrame.position.set(0.48, 0, 0.22);
  j6.add(toolFrame);

  const jointGroups = [j1, j2, j3, j4, j5, j6];
  const stlLoader = new STLLoader();
  const stlScale = 0.01;
  const stlOrigin = new THREE.Vector3(-15.0, -33.8, -35.0);
  const stlPivots = {
    j1: new THREE.Vector3(-15.0, -33.8, -35.0),
    j2: new THREE.Vector3(-15.0, -29.5, 0.5),
    j3: new THREE.Vector3(-17.5, 128.5, 0.0),
    j4: new THREE.Vector3(-17.5, 175.75, 0.0),
    j5: new THREE.Vector3(-15.45, 198.0, 22.75),
    j6: new THREE.Vector3(-26.25, 198.0, 129.5),
  };
  const stlRoot = new THREE.Group();
  stlRoot.position.set(0, 0.02, 0);
  scene.add(stlRoot);
  const stlFixed = new THREE.Group();
  stlRoot.add(stlFixed);
  const stlJ1 = new THREE.Group();
  const stlJ2 = new THREE.Group();
  const stlJ3 = new THREE.Group();
  const stlJ4 = new THREE.Group();
  const stlJ5 = new THREE.Group();
  const stlJ6 = new THREE.Group();
  stlRoot.add(stlJ1);
  stlJ1.position.copy(stlPivots.j1.clone().sub(stlOrigin).multiplyScalar(stlScale));
  stlJ1.add(stlJ2);
  stlJ2.position.copy(stlPivots.j2.clone().sub(stlPivots.j1).multiplyScalar(stlScale));
  stlJ2.add(stlJ3);
  stlJ3.position.copy(stlPivots.j3.clone().sub(stlPivots.j2).multiplyScalar(stlScale));
  stlJ3.add(stlJ4);
  stlJ4.position.copy(stlPivots.j4.clone().sub(stlPivots.j3).multiplyScalar(stlScale));
  stlJ4.add(stlJ5);
  stlJ5.position.copy(stlPivots.j5.clone().sub(stlPivots.j4).multiplyScalar(stlScale));
  stlJ5.add(stlJ6);
  stlJ6.position.copy(stlPivots.j6.clone().sub(stlPivots.j5).multiplyScalar(stlScale));
  const stlJointGroups = [stlJ1, stlJ2, stlJ3, stlJ4, stlJ5, stlJ6];
  const stlParts = [
    { file: "\u57fa\u5ea7.stl", parent: stlFixed, pivot: stlOrigin, material: materials.dark, absolute: true },
    { file: "J1.stl", parent: stlJ1, pivot: stlPivots.j1, material: materials.arm, absolute: false },
    { file: "J2J3.stl", parent: stlJ2, pivot: stlPivots.j2, material: materials.shell, absolute: false },
    { file: "J4.stl", parent: stlJ4, pivot: stlPivots.j4, material: materials.arm, absolute: false },
    { file: "J5A.stl", parent: stlJ5, pivot: stlPivots.j5, material: materials.shell, absolute: false },
    { file: "J5B.stl", parent: stlJ5, pivot: stlPivots.j5, material: materials.wrist, absolute: false },
    { file: "J6.stl", parent: stlJ6, pivot: stlPivots.j6, material: materials.arm, absolute: false },
  ];
  let usingRealModel = false;
  let userCameraOverride = false;
  let defaultTarget = new THREE.Vector3(0, 1.85, 0);
  let defaultCamera = new THREE.Vector3(4.7, 3.45, 5.55);
  let currentViewObject = robotRoot;

  function syncModelVisibility(mode = "real", connected = false) {
    const showRealShell = usingRealModel;
    stlRoot.visible = showRealShell;
    primitiveMeshes.forEach((mesh) => {
      mesh.visible = !showRealShell;
    });
    toolFrame.visible = !showRealShell;
    currentViewObject = showRealShell ? stlRoot : robotRoot;
  }

  function syncStlJoints(joints) {
    const angles = (joints || []).map((value) => THREE.MathUtils.degToRad(Number(value || 0)));
    stlJointGroups[0].rotation.set(0, angles[0] || 0, 0);
    stlJointGroups[1].rotation.set(0, 0, angles[1] || 0);
    stlJointGroups[2].rotation.set(0, 0, angles[2] || 0);
    stlJointGroups[3].rotation.set(0, angles[3] || 0, 0);
    stlJointGroups[4].rotation.set(angles[4] || 0, 0, 0);
    stlJointGroups[5].rotation.set(0, 0, angles[5] || 0);
  }

  async function loadRealModel() {
    const loadedMeshes = [];
    const jobs = stlParts.map((part) =>
      new Promise((resolve, reject) => {
        stlLoader.load(
          `/models/${encodeURIComponent(part.file)}`,
          (geometry) => {
            geometry.computeVertexNormals();
            geometry.translate(-stlOrigin.x, -stlOrigin.y, -stlOrigin.z);
            if (!part.absolute) {
              geometry.translate(-(part.pivot.x - stlOrigin.x), -(part.pivot.y - stlOrigin.y), -(part.pivot.z - stlOrigin.z));
            }
            const mesh = new THREE.Mesh(geometry, part.material);
            mesh.scale.setScalar(stlScale);
            part.parent.add(mesh);
            loadedMeshes.push(mesh);
            resolve();
          },
          undefined,
          reject,
        );
      }),
    );

    const results = await Promise.allSettled(jobs);
    const rejected = results.find((result) => result.status === "rejected");
    if (rejected) {
      loadedMeshes.forEach((mesh) => {
        mesh.parent?.remove(mesh);
        mesh.geometry.dispose();
      });
      throw new Error("Failed to load STL robot shell");
    }

    usingRealModel = true;
    centerRealModel();
    syncModelVisibility("real", false);
    currentViewObject = stlRoot;
  }

  function centerRealModel() {
    const stlBox = new THREE.Box3().setFromObject(stlRoot);
    const rigBox = new THREE.Box3().setFromObject(robotRoot);
    if (stlBox.isEmpty() || rigBox.isEmpty()) {
      return;
    }
    const stlCenter = stlBox.getCenter(new THREE.Vector3());
    const rigCenter = rigBox.getCenter(new THREE.Vector3());
    stlRoot.position.x += rigCenter.x - stlCenter.x;
    stlRoot.position.z += rigCenter.z - stlCenter.z;

    const alignedBox = new THREE.Box3().setFromObject(stlRoot);
    const rigFloor = rigBox.min.y + 0.02;
    stlRoot.position.y += rigFloor - alignedBox.min.y;
  }

  function updateDefaultView(targetObject) {
    if (targetObject === robotRoot) {
      defaultTarget = new THREE.Vector3(0, 1.55, 0.12);
      defaultCamera = new THREE.Vector3(4.45, 3.2, 5.1);
      camera.near = 0.05;
      camera.far = 80;
      camera.updateProjectionMatrix();
      controls.minDistance = 2.8;
      controls.maxDistance = 16;
      return;
    }
    const box = new THREE.Box3().setFromObject(targetObject);
    if (box.isEmpty()) {
      return;
    }
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const sphere = box.getBoundingSphere(new THREE.Sphere());
    const verticalFov = THREE.MathUtils.degToRad(camera.fov);
    const horizontalFov = 2 * Math.atan(Math.tan(verticalFov * 0.5) * camera.aspect);
    const fitHeightDistance = size.y / (2 * Math.tan(verticalFov * 0.5));
    const fitWidthDistance = size.x / (2 * Math.tan(horizontalFov * 0.5));
    const fitDistance = Math.max(fitHeightDistance, fitWidthDistance, sphere.radius * 1.18) * 1.4;
    const direction = new THREE.Vector3(1.08, 0.9, 1.26).normalize();

    defaultTarget = center.clone().add(new THREE.Vector3(0, size.y * 0.04, 0));
    defaultCamera = defaultTarget.clone().addScaledVector(direction, fitDistance);
    camera.near = Math.max(0.05, fitDistance / 80);
    camera.far = Math.max(60, fitDistance * 8);
    camera.updateProjectionMatrix();

    controls.minDistance = Math.max(2.2, defaultCamera.distanceTo(defaultTarget) * 0.45);
    controls.maxDistance = Math.max(12, defaultCamera.distanceTo(defaultTarget) * 3.5);
  }

  function resize() {
    const width = Math.max(container.clientWidth, 1);
    const height = Math.max(container.clientHeight, 1);
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }

  function refreshDefaultView() {
    updateDefaultView(currentViewObject);
    if (!userCameraOverride) {
      resetView();
    }
  }

  function resetView() {
    userCameraOverride = false;
    updateDefaultView(currentViewObject);
    controls.target.copy(defaultTarget);
    camera.position.copy(defaultCamera);
    camera.lookAt(defaultTarget);
    controls.update();
  }

  function setJointAngles(joints) {
    const angles = (joints || []).map((value) => THREE.MathUtils.degToRad(Number(value || 0)));
    jointGroups[0].rotation.set(0, angles[0] || 0, 0);
    jointGroups[1].rotation.set(0, 0, angles[1] || 0);
    jointGroups[2].rotation.set(0, 0, angles[2] || 0);
    jointGroups[3].rotation.set(0, angles[3] || 0, 0);
    jointGroups[4].rotation.set(angles[4] || 0, 0, 0);
    jointGroups[5].rotation.set(0, 0, angles[5] || 0);
    if (usingRealModel) {
      syncStlJoints(joints);
    }
  }

  function setConnectionState(connected) {
    halo.material.color.setHex(connected ? 0x55cab9 : 0xf26d5f);
  }

  controls.addEventListener("start", () => {
    userCameraOverride = true;
  });

  if (typeof ResizeObserver !== "undefined") {
    const observer = new ResizeObserver(() => {
      resize();
      refreshDefaultView();
    });
    observer.observe(container);
  } else {
    window.addEventListener("resize", () => {
      resize();
      refreshDefaultView();
    });
  }

  resize();
  resetView();
  loadRealModel()
    .then(() => {
      syncStlJoints([0, 0, 0, 0, 0, 0]);
      refreshDefaultView();
    })
    .catch((error) => {
      currentViewObject = robotRoot;
      toolFrame.visible = true;
      refreshDefaultView();
      console.warn("Dummy STL model unavailable, using simplified rig.", error);
    });
  renderer.setAnimationLoop(() => {
    controls.update();
    renderer.render(scene, camera);
  });

  return { resetView, setJointAngles, setConnectionState, setSessionMode: syncModelVisibility };
}

async function init() {
  document.getElementById("viewer-build").textContent = `BUILD ${BUILD_ID}`;
  bindLanguageToggle();
  bindModeToggle();
  bindActions();
  bindViewportMode();
  createJointControls();
  applyTranslations();
  await refreshPorts();
  await pollState();
  window.setInterval(pollState, POLL_INTERVAL_MS);
}

init();
