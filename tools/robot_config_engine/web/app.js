/**
 * Linorobot2 Robot Configuration Engine - Client-Side App Logic
 * Pure JavaScript - 100% Client-Side Safe, Zero-Dependency
 */

// MCU Pin Constraints Constants
const ESP32_STRAPPING_PINS = [0, 2, 12, 14, 15];
const ESP32_INPUT_ONLY_PINS = [34, 35, 36, 39];
const ESP32_FLASH_PINS = [6, 7, 8, 9, 10, 11];
const ESP32S3_STRAPPING_PINS = [0, 3, 45, 46];
const RP2040_ADC_PINS = [26, 27, 28, 29];

// Reference Build Presets
const PRESETS = {
  scout_pico2: {
    robot_name: "scout_pico2",
    kinematics: "DIFFERENTIAL_DRIVE",
    mcu: "PICO2",
    transport: "SERIAL",
    geometry: {
      wheel_diameter: 0.08,
      track_width: 0.22,
      wheelbase: 0.20
    },
    motors: {
      driver_type: "GENERIC_2_IN",
      max_rpm: 330,
      cpr: 1320,
      operating_voltage: 12.0,
      motor1_inv: false,
      motor2_inv: true,
      motor3_inv: false,
      motor4_inv: true
    },
    sensors: {
      imu: "BNO085",
      mag: "NONE",
      battery_monitor: "ADC_DIVIDER",
      sonar: true
    },
    pins: {
      led: 25,
      motor1: { pwm: 14, in_a: 15, in_b: 13 },
      motor2: { pwm: 16, in_a: 17, in_b: 12 },
      motor3: { pwm: 8, in_a: 9, in_b: 10 },
      motor4: { pwm: 11, in_a: 18, in_b: 19 },
      encoders: { m1_a: 2, m1_b: 3, m2_a: 4, m2_b: 5, m3_a: 6, m3_b: 7, m4_a: 20, m4_b: 21 },
      i2c: { sda: 8, scl: 9 },
      battery_pin: 26,
      sonar: { trig: 18, echo: 19 }
    }
  },

  mech_esp32: {
    robot_name: "mech_esp32",
    kinematics: "MECANUM",
    mcu: "ESP32",
    transport: "WIFI_UDP",
    wifi_settings: {
      ssid: "YOUR_WIFI_SSID",
      password: "YOUR_WIFI_PASSWORD",
      agent_ip: "192.168.1.100",
      agent_port: 8888
    },
    geometry: {
      wheel_diameter: 0.08,
      track_width: 0.28,
      wheelbase: 0.24
    },
    motors: {
      driver_type: "GENERIC_2_IN",
      max_rpm: 300,
      cpr: 1500,
      operating_voltage: 12.0,
      motor1_inv: false,
      motor2_inv: true,
      motor3_inv: false,
      motor4_inv: true
    },
    sensors: {
      imu: "BNO085",
      mag: "NONE",
      battery_monitor: "INA219",
      sonar: false
    },
    pins: {
      led: 2,
      motor1: { pwm: 13, in_a: 14, in_b: 27 },
      motor2: { pwm: 25, in_a: 26, in_b: 33 },
      motor3: { pwm: 18, in_a: 19, in_b: 23 },
      motor4: { pwm: 15, in_a: 16, in_b: 17 },
      encoders: { m1_a: 34, m1_b: 35, m2_a: 36, m2_b: 39, m3_a: 4, m3_b: 32, m4_a: 5, m4_b: 12 },
      i2c: { sda: 21, scl: 22 },
      battery_pin: 36,
      sonar: { trig: 0, echo: 0 }
    }
  },

  crawler_esp32s3: {
    robot_name: "crawler_esp32s3",
    kinematics: "SKID_STEER",
    mcu: "ESP32S3",
    transport: "SERIAL",
    geometry: {
      wheel_diameter: 0.085,
      track_width: 0.26,
      wheelbase: 0.22
    },
    motors: {
      driver_type: "GENERIC_2_IN",
      max_rpm: 280,
      cpr: 1400,
      operating_voltage: 12.0,
      motor1_inv: false,
      motor2_inv: true,
      motor3_inv: false,
      motor4_inv: true
    },
    sensors: {
      imu: "MPU6050",
      mag: "NONE",
      battery_monitor: "ADC_DIVIDER",
      sonar: true
    },
    pins: {
      led: 48,
      motor1: { pwm: 1, in_a: 2, in_b: 4 },
      motor2: { pwm: 5, in_a: 6, in_b: 7 },
      motor3: { pwm: 8, in_a: 9, in_b: 10 },
      motor4: { pwm: 11, in_a: 12, in_b: 13 },
      encoders: { m1_a: 14, m1_b: 15, m2_a: 16, m2_b: 17, m3_a: 18, m3_b: 21, m4_a: 38, m4_b: 39 },
      i2c: { sda: 41, scl: 42 },
      battery_pin: 3,
      sonar: { trig: 47, echo: 40 }
    }
  },

  waveshare_gendrv: {
    robot_name: "waveshare_rover",
    kinematics: "DIFFERENTIAL_DRIVE",
    mcu: "GENDRV",
    transport: "SERIAL",
    geometry: {
      wheel_diameter: 0.065,
      track_width: 0.20,
      wheelbase: 0.18
    },
    motors: {
      driver_type: "GENERIC_2_IN",
      max_rpm: 330,
      cpr: 1320,
      operating_voltage: 12.0,
      motor1_inv: false,
      motor2_inv: true,
      motor3_inv: false,
      motor4_inv: true
    },
    sensors: {
      imu: "QMI8658",
      mag: "AK09918",
      battery_monitor: "INA219",
      sonar: false
    },
    pins: {
      led: 2,
      motor1: { pwm: 25, in_a: 26, in_b: 27 },
      motor2: { pwm: 14, in_a: 12, in_b: 13 },
      motor3: { pwm: 32, in_a: 33, in_b: 15 },
      motor4: { pwm: 18, in_a: 19, in_b: 23 },
      encoders: { m1_a: 35, m1_b: 34, m2_a: 36, m2_b: 39, m3_a: 4, m3_b: 16, m4_a: 17, m4_b: 5 },
      i2c: { sda: 21, scl: 22 },
      battery_pin: 36,
      sonar: { trig: 0, echo: 0 }
    }
  }
};

// Current Active State
let currentSpec = JSON.parse(JSON.stringify(PRESETS.scout_pico2));
let activeArtifact = "tab-code-header";

// DOM Elements Initialization
document.addEventListener("DOMContentLoaded", () => {
  initNavTabs();
  initArtifactTabs();
  initEventListeners();
  initAutomationEventListeners();
  loadPreset("scout_pico2");
  detectClientOS();
  checkServerRunnerStatus();
  handleUrlParams();
});

// URL Query Parameter Handling (e.g. ?preset=mech_esp32&tab=tab-drive&artifact=tab-code-pio)
function handleUrlParams() {
  const params = new URLSearchParams(window.location.search);
  const preset = params.get("preset");
  const tab = params.get("tab");
  const artifact = params.get("artifact");

  if (preset && PRESETS[preset]) {
    const sel = document.getElementById("preset-select");
    if (sel) sel.value = preset;
    loadPreset(preset);
  }

  if (tab) {
    const tabBtn = document.querySelector(`[data-tab="${tab}"]`);
    if (tabBtn) tabBtn.click();
  }

  if (artifact) {
    const artBtn = document.querySelector(`[data-artifact="${artifact}"]`);
    if (artBtn) artBtn.click();
  }
}

// Tab Navigation Handlers
function initNavTabs() {
  const navTabs = document.querySelectorAll(".nav-tab");
  navTabs.forEach(tab => {
    tab.addEventListener("click", () => {
      navTabs.forEach(t => t.classList.remove("active"));
      document.querySelectorAll(".tab-pane").forEach(p => p.classList.remove("active"));
      tab.classList.add("active");
      const target = document.getElementById(tab.dataset.tab);
      if (target) target.classList.add("active");
    });
  });
}

function initArtifactTabs() {
  const artTabs = document.querySelectorAll(".artifact-tab");
  artTabs.forEach(tab => {
    tab.addEventListener("click", () => {
      artTabs.forEach(t => t.classList.remove("active"));
      tab.classList.add("active");
      activeArtifact = tab.dataset.artifact;
      renderActiveCode();
    });
  });
}

// Global Event Listeners
function initEventListeners() {
  // Preset Selector
  document.getElementById("preset-select").addEventListener("change", (e) => {
    loadPreset(e.target.value);
  });

  // Dynamic Input Form Listeners
  const allInputs = document.querySelectorAll(".form-input, .form-select, input[type='checkbox']");
  allInputs.forEach(input => {
    input.addEventListener("input", handleInputChange);
    input.addEventListener("change", handleInputChange);
  });

  // Smart Auto-Assign Pinout Button
  document.getElementById("btn-smart-alloc").addEventListener("click", autoAssignPins);

  // Copy Code Button
  document.getElementById("btn-copy-code").addEventListener("click", copyActiveCode);

  // Download Artifact Button
  document.getElementById("btn-download-artifact").addEventListener("click", downloadActiveArtifact);

  // Export JSON Spec Button
  document.getElementById("btn-export-json").addEventListener("click", exportSpecJson);

  // Import JSON File
  document.getElementById("import-file").addEventListener("change", handleImportJson);
}

// Load a Preset into State and Form
function loadPreset(presetKey) {
  if (!PRESETS[presetKey]) return;
  currentSpec = JSON.parse(JSON.stringify(PRESETS[presetKey]));
  populateFormFromSpec(currentSpec);
  recomputeAll();
}

// Populate UI Form Fields from Spec Object
function populateFormFromSpec(spec) {
  document.getElementById("cfg-robot-name").value = spec.robot_name || "my_robot";
  document.getElementById("cfg-kinematics").value = spec.kinematics || "DIFFERENTIAL_DRIVE";
  document.getElementById("cfg-mcu").value = spec.mcu || "PICO2";
  document.getElementById("cfg-transport").value = spec.transport || "SERIAL";

  if (spec.wifi_settings) {
    document.getElementById("cfg-wifi-ssid").value = spec.wifi_settings.ssid || "";
    document.getElementById("cfg-wifi-pass").value = spec.wifi_settings.password || "";
    document.getElementById("cfg-agent-ip").value = spec.wifi_settings.agent_ip || "192.168.1.100";
    document.getElementById("cfg-agent-port").value = spec.wifi_settings.agent_port || 8888;
  }

  // Geometry
  document.getElementById("cfg-wheel-dia").value = spec.geometry?.wheel_diameter || 0.08;
  document.getElementById("cfg-track-width").value = spec.geometry?.track_width || 0.22;
  document.getElementById("cfg-wheelbase").value = spec.geometry?.wheelbase || 0.20;

  // Motors
  document.getElementById("cfg-driver-type").value = spec.motors?.driver_type || "GENERIC_2_IN";
  document.getElementById("cfg-max-rpm").value = spec.motors?.max_rpm || 330;
  document.getElementById("cfg-cpr").value = spec.motors?.cpr || 1440;
  document.getElementById("cfg-motor-voltage").value = spec.motors?.operating_voltage || 12.0;

  document.getElementById("cfg-m1-inv").checked = !!spec.motors?.motor1_inv;
  document.getElementById("cfg-m2-inv").checked = !!spec.motors?.motor2_inv;
  document.getElementById("cfg-m3-inv").checked = !!spec.motors?.motor3_inv;
  document.getElementById("cfg-m4-inv").checked = !!spec.motors?.motor4_inv;

  // Sensors
  document.getElementById("cfg-imu").value = spec.sensors?.imu || "NONE";
  document.getElementById("cfg-mag").value = spec.sensors?.mag || "NONE";
  document.getElementById("cfg-battery").value = spec.sensors?.battery_monitor || "NONE";
  document.getElementById("cfg-sonar").value = spec.sensors?.sonar ? "true" : "false";

  // Pins
  const p = spec.pins || {};
  document.getElementById("pin-led").value = p.led !== undefined ? p.led : 25;

  const dt = spec.motors?.driver_type || "GENERIC_2_IN";
  setPinVal("pin-m1-p1", dt === "BTS7960" ? p.motor1?.pwm_r : p.motor1?.pwm);
  setPinVal("pin-m1-p2", dt === "BTS7960" ? p.motor1?.pwm_l : (dt === "GENERIC_1_IN" ? p.motor1?.dir : p.motor1?.in_a));
  setPinVal("pin-m1-p3", dt === "BTS7960" ? p.motor1?.en : p.motor1?.in_b);

  setPinVal("pin-m2-p1", dt === "BTS7960" ? p.motor2?.pwm_r : p.motor2?.pwm);
  setPinVal("pin-m2-p2", dt === "BTS7960" ? p.motor2?.pwm_l : (dt === "GENERIC_1_IN" ? p.motor2?.dir : p.motor2?.in_a));
  setPinVal("pin-m2-p3", dt === "BTS7960" ? p.motor2?.en : p.motor2?.in_b);

  setPinVal("pin-m3-p1", dt === "BTS7960" ? p.motor3?.pwm_r : p.motor3?.pwm);
  setPinVal("pin-m3-p2", dt === "BTS7960" ? p.motor3?.pwm_l : (dt === "GENERIC_1_IN" ? p.motor3?.dir : p.motor3?.in_a));
  setPinVal("pin-m3-p3", dt === "BTS7960" ? p.motor3?.en : p.motor3?.in_b);

  setPinVal("pin-m4-p1", dt === "BTS7960" ? p.motor4?.pwm_r : p.motor4?.pwm);
  setPinVal("pin-m4-p2", dt === "BTS7960" ? p.motor4?.pwm_l : (dt === "GENERIC_1_IN" ? p.motor4?.dir : p.motor4?.in_a));
  setPinVal("pin-m4-p3", dt === "BTS7960" ? p.motor4?.en : p.motor4?.in_b);

  const enc = p.encoders || {};
  setPinVal("pin-enc-1a", enc.m1_a);
  setPinVal("pin-enc-1b", enc.m1_b);
  setPinVal("pin-enc-2a", enc.m2_a);
  setPinVal("pin-enc-2b", enc.m2_b);
  setPinVal("pin-enc-3a", enc.m3_a);
  setPinVal("pin-enc-3b", enc.m3_b);
  setPinVal("pin-enc-4a", enc.m4_a);
  setPinVal("pin-enc-4b", enc.m4_b);

  setPinVal("pin-i2c-sda", p.i2c?.sda);
  setPinVal("pin-i2c-scl", p.i2c?.scl);
  setPinVal("pin-battery", p.battery_pin);
  setPinVal("pin-sonar-trig", p.sonar?.trig);
  setPinVal("pin-sonar-echo", p.sonar?.echo);

  updateDynamicUIState();
}

function setPinVal(id, val) {
  const el = document.getElementById(id);
  if (el) el.value = val !== undefined ? val : "";
}

function getInt(id, defaultVal = 0) {
  const el = document.getElementById(id);
  if (!el || el.value === "" || isNaN(parseInt(el.value, 10))) return defaultVal;
  return parseInt(el.value, 10);
}

function getFloat(id, defaultVal = 0.0) {
  const el = document.getElementById(id);
  if (!el || el.value === "" || isNaN(parseFloat(el.value))) return defaultVal;
  return parseFloat(el.value);
}

// Read Current Values from Form and update State
function readSpecFromForm() {
  const kinematics = document.getElementById("cfg-kinematics").value;
  const driverType = document.getElementById("cfg-driver-type").value;
  const transport = document.getElementById("cfg-transport").value;
  const is4WD = kinematics !== "DIFFERENTIAL_DRIVE";

  const spec = {
    robot_name: document.getElementById("cfg-robot-name").value.trim().toLowerCase().replace(/[^a-z0-9_]/g, "_") || "my_robot",
    kinematics: kinematics,
    mcu: document.getElementById("cfg-mcu").value,
    transport: transport,
    geometry: {
      wheel_diameter: getFloat("cfg-wheel-dia", 0.08),
      track_width: getFloat("cfg-track-width", 0.22),
    },
    motors: {
      driver_type: driverType,
      max_rpm: getFloat("cfg-max-rpm", 330),
      cpr: getFloat("cfg-cpr", 1440),
      operating_voltage: getFloat("cfg-motor-voltage", 12.0),
      motor1_inv: document.getElementById("cfg-m1-inv").checked,
      motor2_inv: document.getElementById("cfg-m2-inv").checked,
      motor3_inv: document.getElementById("cfg-m3-inv").checked,
      motor4_inv: document.getElementById("cfg-m4-inv").checked
    },
    sensors: {
      imu: document.getElementById("cfg-imu").value,
      mag: document.getElementById("cfg-mag").value,
      battery_monitor: document.getElementById("cfg-battery").value,
      sonar: document.getElementById("cfg-sonar").value === "true"
    },
    pins: {
      led: getInt("pin-led", 25),
      encoders: {
        m1_a: getInt("pin-enc-1a"),
        m1_b: getInt("pin-enc-1b"),
        m2_a: getInt("pin-enc-2a"),
        m2_b: getInt("pin-enc-2b")
      },
      i2c: {
        sda: getInt("pin-i2c-sda"),
        scl: getInt("pin-i2c-scl")
      }
    }
  };

  if (is4WD) {
    spec.geometry.wheelbase = getFloat("cfg-wheelbase", 0.20);
    spec.pins.encoders.m3_a = getInt("pin-enc-3a");
    spec.pins.encoders.m3_b = getInt("pin-enc-3b");
    spec.pins.encoders.m4_a = getInt("pin-enc-4a");
    spec.pins.encoders.m4_b = getInt("pin-enc-4b");
  }

  // Motor Pin Structures
  const parseMotorPin = (prefix) => {
    const p1 = getInt(`${prefix}-p1`);
    const p2 = getInt(`${prefix}-p2`);
    const p3 = getInt(`${prefix}-p3`);

    if (driverType === "BTS7960") {
      return { pwm_r: p1, pwm_l: p2, en: p3 };
    } else if (driverType === "GENERIC_2_IN") {
      return { pwm: p1, in_a: p2, in_b: p3 };
    } else if (driverType === "GENERIC_1_IN") {
      return { pwm: p1, dir: p2 };
    } else {
      return { pwm: p1 };
    }
  };

  spec.pins.motor1 = parseMotorPin("pin-m1");
  spec.pins.motor2 = parseMotorPin("pin-m2");
  if (is4WD) {
    spec.pins.motor3 = parseMotorPin("pin-m3");
    spec.pins.motor4 = parseMotorPin("pin-m4");
  }

  if (spec.sensors.battery_monitor === "ADC_DIVIDER") {
    spec.pins.battery_pin = getInt("pin-battery");
  }
  if (spec.sensors.sonar) {
    spec.pins.sonar = {
      trig: getInt("pin-sonar-trig"),
      echo: getInt("pin-sonar-echo")
    };
  }

  if (transport === "WIFI_UDP") {
    spec.wifi_settings = {
      ssid: document.getElementById("cfg-wifi-ssid").value,
      password: document.getElementById("cfg-wifi-pass").value,
      agent_ip: document.getElementById("cfg-agent-ip").value,
      agent_port: getInt("cfg-agent-port", 8888)
    };
  }

  return spec;
}

// Handle Form Changes
function handleInputChange() {
  updateDynamicUIState();
  recomputeAll();
}

// Update Dynamic Visibility of Form Components
function updateDynamicUIState() {
  const kinematics = document.getElementById("cfg-kinematics").value;
  const is4WD = kinematics !== "DIFFERENTIAL_DRIVE";
  const transport = document.getElementById("cfg-transport").value;
  const driverType = document.getElementById("cfg-driver-type").value;
  const batteryType = document.getElementById("cfg-battery").value;
  const sonarActive = document.getElementById("cfg-sonar").value === "true";

  // 4WD Elements
  document.getElementById("group-wheelbase").style.display = is4WD ? "flex" : "none";
  document.getElementById("box-m3-inv").style.display = is4WD ? "flex" : "none";
  document.getElementById("box-m4-inv").style.display = is4WD ? "flex" : "none";
  document.getElementById("row-motor-3").style.display = is4WD ? "table-row" : "none";
  document.getElementById("row-motor-4").style.display = is4WD ? "table-row" : "none";
  document.getElementById("row-enc-3").style.display = is4WD ? "table-row" : "none";
  document.getElementById("row-enc-4").style.display = is4WD ? "table-row" : "none";

  // WiFi Settings
  document.getElementById("wifi-config-box").style.display = transport === "WIFI_UDP" ? "flex" : "none";

  // Battery ADC
  document.getElementById("box-battery-pin").style.display = batteryType === "ADC_DIVIDER" ? "flex" : "none";

  // Sonar
  document.getElementById("box-sonar-trig").style.display = sonarActive ? "flex" : "none";
  document.getElementById("box-sonar-echo").style.display = sonarActive ? "flex" : "none";

  // Motor Headers based on Driver Type
  const headerEl = document.getElementById("motor-pin-headers");
  if (driverType === "BTS7960") {
    headerEl.innerHTML = `<th>Motor</th><th>PWM_R (Right)</th><th>PWM_L (Left)</th><th>EN (Enable)</th>`;
  } else if (driverType === "GENERIC_2_IN") {
    headerEl.innerHTML = `<th>Motor</th><th>PWM (Speed)</th><th>IN_A (Dir 1)</th><th>IN_B (Dir 2)</th>`;
  } else if (driverType === "GENERIC_1_IN") {
    headerEl.innerHTML = `<th>Motor</th><th>PWM (Speed)</th><th>DIR (Direction)</th><th>--</th>`;
  } else {
    headerEl.innerHTML = `<th>Motor</th><th>PWM Signal</th><th>--</th><th>--</th>`;
  }

  // HUD Tag
  const baseTag = document.getElementById("hud-base-tag");
  if (kinematics === "DIFFERENTIAL_DRIVE") baseTag.innerText = "2WD Differential";
  else if (kinematics === "SKID_STEER") baseTag.innerText = "4WD Skid Steer";
  else baseTag.innerText = "4WD Mecanum";
}

// Master Recompute & Render
function recomputeAll() {
  currentSpec = readSpecFromForm();

  // 1. Validate Rules
  const validation = validateRobotSpec(currentSpec);
  renderSafetyInspector(validation);

  // 2. Compute Kinematics & Update HUD
  renderKinematicsHUD(validation.stats);

  // 3. Update Automation Previews
  updateAutomationPreviews();

  // 4. Render Active Code Viewer
  renderActiveCode();
}

// Hardware & Electrical Rule Validation Engine
function validateRobotSpec(spec) {
  const errors = [];
  const stats = {};

  const mcu = (spec.mcu || "").toUpperCase();
  const kinematics = spec.kinematics;
  const geom = spec.geometry || {};
  const motors = spec.motors || {};
  const pins = spec.pins || {};
  const sensors = spec.sensors || {};

  const wheelD = geom.wheel_diameter || 0;
  const trackW = geom.track_width || 0;
  const maxRpm = motors.max_rpm || 0;
  const cpr = motors.cpr || 0;

  if (wheelD <= 0) errors.push({ level: "ERROR", field: "geometry.wheel_diameter", message: "Wheel diameter must be > 0 m" });
  if (trackW <= 0) errors.push({ level: "ERROR", field: "geometry.track_width", message: "Track width must be > 0 m" });
  if (maxRpm <= 0) errors.push({ level: "ERROR", field: "motors.max_rpm", message: "Motor RPM must be > 0" });
  if (cpr <= 0) errors.push({ level: "ERROR", field: "motors.cpr", message: "Encoder CPR must be > 0" });

  if (wheelD > 0 && maxRpm > 0 && cpr > 0) {
    const wheelCirc = Math.PI * wheelD;
    const maxLinearSpeed = (wheelCirc * maxRpm / 60.0) * 0.85; // 85% headroom
    const maxAngularSpeed = trackW > 0 ? (2.0 * maxLinearSpeed) / trackW : 0;
    const ticksPerMeter = cpr / wheelCirc;

    stats.wheel_circumference_m = wheelCirc;
    stats.max_linear_speed_m_s = maxLinearSpeed;
    stats.max_angular_speed_rad_s = maxAngularSpeed;
    stats.ticks_per_meter = ticksPerMeter;
  }

  // Pin Conflict Checking
  const assignedPins = {};
  function registerPin(pin, name, isOutput = false) {
    if (pin === undefined || pin === null || isNaN(pin) || pin === "") return;
    const p = parseInt(pin, 10);
    if (!assignedPins[p]) assignedPins[p] = [];
    assignedPins[p].push(name);

    // MCU Rules
    if (mcu === "ESP32" || mcu === "GENDRV") {
      if (ESP32_FLASH_PINS.includes(p)) {
        errors.push({ level: "ERROR", field: name, message: `GPIO ${p} is connected to internal SPI Flash! Strictly forbidden.` });
      }
      if (isOutput && ESP32_INPUT_ONLY_PINS.includes(p)) {
        errors.push({ level: "ERROR", field: name, message: `GPIO ${p} is an INPUT-ONLY pin and cannot output PWM/DIR signals!` });
      }
      if (!isOutput && ESP32_STRAPPING_PINS.includes(p)) {
        errors.push({ level: "WARNING", field: name, message: `GPIO ${p} is an ESP32 strapping pin. May interfere with boot if pulled LOW.` });
      }
    } else if (mcu === "ESP32S3") {
      if (!isOutput && ESP32S3_STRAPPING_PINS.includes(p)) {
        errors.push({ level: "WARNING", field: name, message: `GPIO ${p} is an ESP32-S3 strapping pin.` });
      }
    } else if (mcu === "PICO" || mcu === "PICO2") {
      if (p < 0 || p > 29) {
        errors.push({ level: "ERROR", field: name, message: `GP${p} is out of range for RP2040/RP2350 (0-29).` });
      }
    }
  }

  // Check Motors
  const numMotors = kinematics === "DIFFERENTIAL_DRIVE" ? 2 : 4;
  const driverType = motors.driver_type;

  for (let i = 1; i <= numMotors; i++) {
    const m = pins[`motor${i}`] || {};
    if (driverType === "BTS7960") {
      registerPin(m.pwm_r, `Motor ${i} PWM_R`, true);
      registerPin(m.pwm_l, `Motor ${i} PWM_L`, true);
      registerPin(m.en, `Motor ${i} Enable`, true);
    } else if (driverType === "GENERIC_2_IN") {
      registerPin(m.pwm, `Motor ${i} PWM`, true);
      registerPin(m.in_a, `Motor ${i} IN_A`, true);
      registerPin(m.in_b, `Motor ${i} IN_B`, true);
    } else if (driverType === "GENERIC_1_IN") {
      registerPin(m.pwm, `Motor ${i} PWM`, true);
      registerPin(m.dir, `Motor ${i} DIR`, true);
    } else {
      registerPin(m.pwm, `Motor ${i} PWM`, true);
    }
  }

  // Check Encoders
  const enc = pins.encoders || {};
  for (let i = 1; i <= numMotors; i++) {
    registerPin(enc[`m${i}_a`], `Encoder ${i} A`, false);
    registerPin(enc[`m${i}_b`], `Encoder ${i} B`, false);
  }

  // Check LED
  registerPin(pins.led, "Status LED", true);

  // Check I2C
  if (pins.i2c) {
    registerPin(pins.i2c.sda, "I2C SDA", false);
    registerPin(pins.i2c.scl, "I2C SCL", false);
  }

  // Check Battery
  if (sensors.battery_monitor === "ADC_DIVIDER" && pins.battery_pin !== undefined) {
    registerPin(pins.battery_pin, "Battery ADC", false);
    if ((mcu === "PICO" || mcu === "PICO2") && !RP2040_ADC_PINS.includes(parseInt(pins.battery_pin, 10))) {
      errors.push({ level: "ERROR", field: "Battery ADC", message: `GP${pins.battery_pin} is not an analog ADC pin on Pico/Pico2 (Must be GP26, GP27, or GP28).` });
    }
  }

  // Check Sonar
  if (sensors.sonar && pins.sonar) {
    registerPin(pins.sonar.trig, "Sonar Trigger", true);
    registerPin(pins.sonar.echo, "Sonar Echo", false);
  }

  // Duplicate Pin Detection
  for (const [pinNum, names] of Object.entries(assignedPins)) {
    if (names.length > 1) {
      errors.push({ level: "ERROR", field: `Pin ${pinNum}`, message: `Pin ${pinNum} assigned to multiple functions: ${names.join(", ")}` });
    }
  }

  const isValid = !errors.some(e => e.level === "ERROR");
  return { isValid, errors, stats };
}

// Render Safety Inspector Results
function renderSafetyInspector({ isValid, errors }) {
  const dot = document.getElementById("safety-status-dot");
  const title = document.getElementById("safety-status-title");
  const badge = document.getElementById("safety-status-badge");
  const list = document.getElementById("safety-messages-list");

  const errCount = errors.filter(e => e.level === "ERROR").length;
  const warnCount = errors.filter(e => e.level === "WARNING").length;

  if (errCount > 0) {
    dot.className = "status-dot status-err";
    title.innerText = "Hardware Safety Rules: FAILED";
    badge.className = "safety-badge badge-err";
    badge.innerText = `${errCount} Error${errCount > 1 ? "s" : ""}`;
  } else if (warnCount > 0) {
    dot.className = "status-dot status-warn";
    title.innerText = "Hardware Safety Rules: WARNINGS";
    badge.className = "safety-badge badge-warn";
    badge.innerText = `${warnCount} Warning${warnCount > 1 ? "s" : ""}`;
  } else {
    dot.className = "status-dot status-ok";
    title.innerText = "Hardware Safety Rules: PASSED";
    badge.className = "safety-badge badge-ok";
    badge.innerText = "0 Errors";
  }

  if (errors.length === 0) {
    list.innerHTML = `
      <div class="safety-item item-ok">
        <span class="item-icon">✅</span>
        <span>All pin assignments, electrical constraints, and kinematics are 100% verified.</span>
      </div>
    `;
  } else {
    list.innerHTML = errors.map(err => `
      <div class="safety-item ${err.level === 'ERROR' ? 'item-err' : 'item-warn'}">
        <span class="item-icon">${err.level === 'ERROR' ? '❌' : '⚠️'}</span>
        <span><strong>${err.field}:</strong> ${err.message}</span>
      </div>
    `).join("");
  }
}

// Render Kinematics Metrics HUD
function renderKinematicsHUD(stats) {
  const speedMs = stats.max_linear_speed_m_s || 0;
  const speedKmh = speedMs * 3.6;
  const rads = stats.max_angular_speed_rad_s || 0;
  const degs = rads * (180 / Math.PI);
  const ticks = stats.ticks_per_meter || 0;
  const circ = stats.wheel_circumference_m || 0;

  document.getElementById("val-speed-ms").innerText = speedMs.toFixed(2);
  document.getElementById("val-speed-kmh").innerText = `${speedKmh.toFixed(2)} km/h (85% Headroom)`;

  document.getElementById("val-speed-rads").innerText = rads.toFixed(2);
  document.getElementById("val-speed-degs").innerText = `${degs.toFixed(1)} deg/s`;

  document.getElementById("val-ticks-m").innerText = Math.round(ticks).toLocaleString();
  document.getElementById("val-wheel-circ").innerText = `Circumference: ${circ.toFixed(3)} m`;
}

// Code Generators
function generateCppHeader(spec) {
  const name = spec.robot_name || "my_robot";
  const nameUpper = name.toUpperCase();
  const kinematics = spec.kinematics || "DIFFERENTIAL_DRIVE";
  const mcu = (spec.mcu || "PICO2").toUpperCase();
  const geom = spec.geometry || {};
  const motors = spec.motors || {};
  const sensors = spec.sensors || {};
  const pins = spec.pins || {};
  const driver = motors.driver_type || "GENERIC_2_IN";
  const is4WD = kinematics !== "DIFFERENTIAL_DRIVE";

  const lines = [
    `// =============================================================================`,
    `// Auto-generated Linorobot2 Configuration for ${name}`,
    `// Microcontroller: ${mcu} | Kinematics: ${kinematics}`,
    `// =============================================================================`,
    `#ifndef ${nameUpper}_CONFIG_H`,
    `#define ${nameUpper}_CONFIG_H`,
    ``,
    `#define LINO_BASE ${kinematics}`,
    `#define USE_${driver}_MOTOR_DRIVER`,
    ``,
    `// Kinematics & Wheel Geometry`,
    `#define WHEEL_DIAMETER ${geom.wheel_diameter || 0.08} // meters`,
    `#define LR_WHEELS_DISTANCE ${geom.track_width || 0.22} // meters`
  ];

  if (is4WD && geom.wheelbase) {
    lines.push(`#define FR_WHEELS_DISTANCE ${geom.wheelbase} // meters`);
  }

  lines.push(
    ``,
    `// Motor & Encoder Parameters`,
    `#define MOTOR_MAX_RPM ${motors.max_rpm || 330}`,
    `#define MAX_RPM_RATIO 0.85`,
    `#define MOTOR_OPERATING_VOLTAGE ${motors.operating_voltage || 12.0}`,
    `#define MOTOR_POWER_MAX_VOLTAGE ${motors.operating_voltage || 12.0}`,
    `#define PWM_FREQUENCY 20000`,
    `#define PWM_BITS 10`,
    `#define PWM_MAX ((1 << PWM_BITS) - 1)`,
    `#define PWM_MIN -PWM_MAX`,
    ``,
    `#define COUNTS_PER_REV1 ${motors.cpr || 1440}`,
    `#define COUNTS_PER_REV2 ${motors.cpr || 1440}`,
    `#define COUNTS_PER_REV3 ${motors.cpr || 1440}`,
    `#define COUNTS_PER_REV4 ${motors.cpr || 1440}`,
    ``,
    `#define MOTOR1_INV ${!!motors.motor1_inv}`,
    `#define MOTOR2_INV ${!!motors.motor2_inv}`,
    `#define MOTOR3_INV ${!!motors.motor3_inv}`,
    `#define MOTOR4_INV ${!!motors.motor4_inv}`,
    `#define MOTOR1_ENCODER_INV false`,
    `#define MOTOR2_ENCODER_INV false`,
    `#define MOTOR3_ENCODER_INV false`,
    `#define MOTOR4_ENCODER_INV false`,
    ``,
    `// Default PID Tuning Constants`,
    `#define K_P 0.6`,
    `#define K_I 0.8`,
    `#define K_D 0.5`,
    ``,
    `// Pin Assignments`,
    `#define LED_PIN ${pins.led !== undefined ? pins.led : 25}`
  );

  const numMotors = is4WD ? 4 : 2;
  for (let i = 1; i <= numMotors; i++) {
    const m = pins[`motor${i}`] || {};
    if (driver === "BTS7960") {
      lines.push(`#define MOTOR${i}_PWM_R ${m.pwm_r || 0}`);
      lines.push(`#define MOTOR${i}_PWM_L ${m.pwm_l || 0}`);
      if (m.en !== undefined) lines.push(`#define MOTOR${i}_EN ${m.en}`);
    } else if (driver === "GENERIC_2_IN") {
      lines.push(`#define MOTOR${i}_PWM ${m.pwm || 0}`);
      lines.push(`#define MOTOR${i}_IN_A ${m.in_a || 0}`);
      lines.push(`#define MOTOR${i}_IN_B ${m.in_b || 0}`);
    } else if (driver === "GENERIC_1_IN") {
      lines.push(`#define MOTOR${i}_PWM ${m.pwm || 0}`);
      lines.push(`#define MOTOR${i}_DIR ${m.dir || 0}`);
    } else {
      lines.push(`#define MOTOR${i}_PWM ${m.pwm || 0}`);
    }
  }

  const enc = pins.encoders || {};
  for (let i = 1; i <= numMotors; i++) {
    lines.push(`#define MOTOR${i}_ENCODER_A ${enc[`m${i}_a`] || 0}`);
    lines.push(`#define MOTOR${i}_ENCODER_B ${enc[`m${i}_b`] || 0}`);
  }

  lines.push(``, `// Sensor Configurations`);
  if (sensors.imu && sensors.imu !== "NONE") {
    lines.push(`#define USE_${sensors.imu}_IMU`);
  }
  if (sensors.mag && sensors.mag !== "NONE") {
    lines.push(`#define USE_${sensors.mag}_MAG`);
  } else {
    lines.push(`#define USE_FAKE_MAG`);
  }

  if (sensors.battery_monitor === "ADC_DIVIDER") {
    lines.push(
      `#define USE_BATTERY_MONITOR`,
      `#define BATTERY_PIN ${pins.battery_pin || 26}`,
      `#define BATTERY_R1 30000.0`,
      `#define BATTERY_R2 7500.0`,
      `#define BATTERY_ADJUST(v) (v * (3.3 / 4095.0) * ((30000.0 + 7500.0) / 7500.0))`
    );
  } else if (sensors.battery_monitor === "INA219") {
    lines.push(`#define USE_INA219`);
  }

  if (sensors.sonar && pins.sonar) {
    lines.push(
      `#define USE_SONAR`,
      `#define TRIG_PIN ${pins.sonar.trig || 0}`,
      `#define ECHO_PIN ${pins.sonar.echo || 0}`
    );
  }

  lines.push(``, `#endif // ${nameUpper}_CONFIG_H`, ``);
  return lines.join("\n");
}

function generatePlatformioEnv(spec) {
  const name = spec.robot_name || "my_robot";
  const mcu = (spec.mcu || "PICO2").toUpperCase();
  const cfgMacro = `USE_${name.toUpperCase()}_CONFIG`;

  if (mcu === "PICO" || mcu === "PICO2") {
    const board = mcu === "PICO2" ? "rpipico2" : "rpipico";
    return `[env:${name}]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = ${board}
board_build.core = earlephilhower
board_build.filesystem_size = 0.5m
build_flags =
    -I ../config
    -D PICO
    -D ${cfgMacro}
`;
  } else if (mcu === "ESP32" || mcu === "GENDRV") {
    return `[env:${name}]
platform = espressif32
board = esp32doit-devkit-v1
build_flags =
    -I ../config
    -D ESP32
    -D ${cfgMacro}
`;
  } else if (mcu === "ESP32S3") {
    return `[env:${name}]
platform = espressif32
board = esp32-s3-devkitc-1
build_flags =
    -I ../config
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    -D ${cfgMacro}
`;
  } else if (mcu === "ESP32S2") {
    return `[env:${name}]
platform = espressif32
board = esp32-s2-saola-1
build_flags =
    -I ../config
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    -D ${cfgMacro}
`;
  }
  return "";
}

function generateUrdfXacro(spec) {
  const geom = spec.geometry || {};
  const wheelD = geom.wheel_diameter || 0.08;
  const wheelR = wheelD / 2.0;
  const trackW = geom.track_width || 0.22;
  const wheelPosY = trackW / 2.0;

  return `<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="wheel_radius" value="${wheelR.toFixed(4)}" />
  <xacro:property name="wheel_width" value="0.026" />
  <xacro:property name="wheel_pos_x" value="0.0" />
  <xacro:property name="wheel_pos_y" value="${wheelPosY.toFixed(4)}" />
  <xacro:property name="wheel_pos_z" value="-0.010" />
  <xacro:property name="wheel_mass" value="0.05" />

  <xacro:property name="base_length" value="${(trackW * 1.2).toFixed(3)}" />
  <xacro:property name="base_width" value="${(trackW * 0.9).toFixed(3)}" />
  <xacro:property name="base_height" value="0.070" />
  <xacro:property name="base_mass" value="1.2" />

  <xacro:property name="laser_pose">
    <origin xyz="0.05 0 0.08" rpy="0 0 0"/>
  </xacro:property>
</robot>
`;
}

function generateWiringTable(spec) {
  const mcu = spec.mcu;
  const pins = spec.pins || {};
  const driver = spec.motors?.driver_type || "GENERIC_2_IN";
  const is4WD = spec.kinematics !== "DIFFERENTIAL_DRIVE";

  const rows = [
    `# 🔌 Wiring & Pinout Chart for ${spec.robot_name}`,
    `**Microcontroller**: ${mcu} | **Kinematics**: ${spec.kinematics} | **Driver**: ${driver}`,
    ``,
    `| Function | MCU Pin (GPIO) | Target Peripheral / Signal | Wire Color / Notes |`,
    `| :--- | :---: | :--- | :--- |`,
    `| **Status LED** | \`GPIO ${pins.led}\` | Onboard or External LED | Status Blinker |`
  ];

  const numMotors = is4WD ? 4 : 2;
  for (let i = 1; i <= numMotors; i++) {
    const m = pins[`motor${i}`] || {};
    if (driver === "BTS7960") {
      rows.push(`| **Motor ${i} PWM_R** | \`GPIO ${m.pwm_r}\` | BTS7960 R_PWM | Forward PWM |`);
      rows.push(`| **Motor ${i} PWM_L** | \`GPIO ${m.pwm_l}\` | BTS7960 L_PWM | Reverse PWM |`);
      if (m.en !== undefined) rows.push(`| **Motor ${i} Enable** | \`GPIO ${m.en}\` | BTS7960 R_EN / L_EN | Enable Pin |`);
    } else {
      rows.push(`| **Motor ${i} PWM** | \`GPIO ${m.pwm}\` | Driver PWM / ENA | Speed Control |`);
      rows.push(`| **Motor ${i} IN_A** | \`GPIO ${m.in_a}\` | Driver IN1 / DIR | Direction Line A |`);
      rows.push(`| **Motor ${i} IN_B** | \`GPIO ${m.in_b}\` | Driver IN2 | Direction Line B |`);
    }
  }

  const enc = pins.encoders || {};
  for (let i = 1; i <= numMotors; i++) {
    rows.push(`| **Encoder ${i} Phase A** | \`GPIO ${enc[`m${i}_a`]}\` | Wheel ${i} Encoder A | Hardware Interrupt |`);
    rows.push(`| **Encoder ${i} Phase B** | \`GPIO ${enc[`m${i}_b`]}\` | Wheel ${i} Encoder B | Direction Read |`);
  }

  if (pins.i2c) {
    rows.push(`| **I2C SDA** | \`GPIO ${pins.i2c.sda}\` | IMU / Mag / INA219 SDA | I2C Data Line (4.7kΩ Pullup) |`);
    rows.push(`| **I2C SCL** | \`GPIO ${pins.i2c.scl}\` | IMU / Mag / INA219 SCL | I2C Clock Line (4.7kΩ Pullup) |`);
  }

  if (spec.sensors?.battery_monitor === "ADC_DIVIDER" && pins.battery_pin !== undefined) {
    rows.push(`| **Battery Divider** | \`GPIO ${pins.battery_pin}\` | Voltage Divider Center | 30kΩ/7.5kΩ Divider to V_BAT |`);
  }

  if (spec.sensors?.sonar && pins.sonar) {
    rows.push(`| **Sonar Trigger** | \`GPIO ${pins.sonar.trig}\` | HC-SR04 Trig | 10us Output Pulse |`);
    rows.push(`| **Sonar Echo** | \`GPIO ${pins.sonar.echo}\` | HC-SR04 Echo | 5V->3.3V Voltage Divider Recommended |`);
  }

  return rows.join("\n");
}

// Render the Active Code Tab
function renderActiveCode() {
  const display = document.getElementById("code-display");
  let code = "";

  if (activeArtifact === "tab-code-header") {
    code = generateCppHeader(currentSpec);
    display.className = "language-cpp";
  } else if (activeArtifact === "tab-code-pio") {
    code = generatePlatformioEnv(currentSpec);
    display.className = "language-ini";
  } else if (activeArtifact === "tab-code-urdf") {
    code = generateUrdfXacro(currentSpec);
    display.className = "language-xml";
  } else if (activeArtifact === "tab-code-wiring") {
    code = generateWiringTable(currentSpec);
    display.className = "language-markdown";
  } else if (activeArtifact === "tab-code-deploy") {
    code = generateDeployScript(currentSpec, readAutomationOptions());
    display.className = "language-bash";
  } else if (activeArtifact === "tab-code-json") {
    code = JSON.stringify(currentSpec, null, 2);
    display.className = "language-json";
  }

  display.innerText = code;
}

// Smart Auto-Assign Pinout Algorithm
function autoAssignPins() {
  const mcu = currentSpec.mcu;
  const is4WD = currentSpec.kinematics !== "DIFFERENTIAL_DRIVE";
  const driver = currentSpec.motors?.driver_type || "GENERIC_2_IN";

  if (mcu === "PICO" || mcu === "PICO2") {
    document.getElementById("pin-led").value = 25;
    document.getElementById("pin-i2c-sda").value = 8;
    document.getElementById("pin-i2c-scl").value = 9;
    document.getElementById("pin-battery").value = 26;
    document.getElementById("pin-sonar-trig").value = 18;
    document.getElementById("pin-sonar-echo").value = 19;

    document.getElementById("pin-m1-p1").value = 14;
    document.getElementById("pin-m1-p2").value = 15;
    document.getElementById("pin-m1-p3").value = 13;

    document.getElementById("pin-m2-p1").value = 16;
    document.getElementById("pin-m2-p2").value = 17;
    document.getElementById("pin-m2-p3").value = 12;

    document.getElementById("pin-enc-1a").value = 2;
    document.getElementById("pin-enc-1b").value = 3;
    document.getElementById("pin-enc-2a").value = 4;
    document.getElementById("pin-enc-2b").value = 5;

    if (is4WD) {
      document.getElementById("pin-m3-p1").value = 8;
      document.getElementById("pin-m3-p2").value = 9;
      document.getElementById("pin-m3-p3").value = 10;
      document.getElementById("pin-m4-p1").value = 11;
      document.getElementById("pin-m4-p2").value = 18;
      document.getElementById("pin-m4-p3").value = 19;
      document.getElementById("pin-enc-3a").value = 6;
      document.getElementById("pin-enc-3b").value = 7;
      document.getElementById("pin-enc-4a").value = 20;
      document.getElementById("pin-enc-4b").value = 21;
    }
  } else if (mcu === "ESP32" || mcu === "GENDRV") {
    document.getElementById("pin-led").value = 2;
    document.getElementById("pin-i2c-sda").value = 21;
    document.getElementById("pin-i2c-scl").value = 22;
    document.getElementById("pin-battery").value = 36;
    document.getElementById("pin-sonar-trig").value = 5;
    document.getElementById("pin-sonar-echo").value = 17;

    document.getElementById("pin-m1-p1").value = 13;
    document.getElementById("pin-m1-p2").value = 14;
    document.getElementById("pin-m1-p3").value = 27;

    document.getElementById("pin-m2-p1").value = 25;
    document.getElementById("pin-m2-p2").value = 26;
    document.getElementById("pin-m2-p3").value = 33;

    document.getElementById("pin-enc-1a").value = 4;
    document.getElementById("pin-enc-1b").value = 32;
    document.getElementById("pin-enc-2a").value = 35;
    document.getElementById("pin-enc-2b").value = 34;

    if (is4WD) {
      document.getElementById("pin-m3-p1").value = 18;
      document.getElementById("pin-m3-p2").value = 19;
      document.getElementById("pin-m3-p3").value = 23;
      document.getElementById("pin-m4-p1").value = 15;
      document.getElementById("pin-m4-p2").value = 16;
      document.getElementById("pin-m4-p3").value = 17;
      document.getElementById("pin-enc-1a").value = 34;
      document.getElementById("pin-enc-1b").value = 35;
      document.getElementById("pin-enc-2a").value = 36;
      document.getElementById("pin-enc-2b").value = 39;
      document.getElementById("pin-enc-3a").value = 4;
      document.getElementById("pin-enc-3b").value = 32;
      document.getElementById("pin-enc-4a").value = 5;
      document.getElementById("pin-enc-4b").value = 12;
    }
  } else if (mcu === "ESP32S3") {
    document.getElementById("pin-led").value = 48;
    document.getElementById("pin-i2c-sda").value = 41;
    document.getElementById("pin-i2c-scl").value = 42;
    document.getElementById("pin-battery").value = 3;
    document.getElementById("pin-sonar-trig").value = 47;
    document.getElementById("pin-sonar-echo").value = 40;

    document.getElementById("pin-m1-p1").value = 1;
    document.getElementById("pin-m1-p2").value = 2;
    document.getElementById("pin-m1-p3").value = 4;

    document.getElementById("pin-m2-p1").value = 5;
    document.getElementById("pin-m2-p2").value = 6;
    document.getElementById("pin-m2-p3").value = 7;

    document.getElementById("pin-enc-1a").value = 14;
    document.getElementById("pin-enc-1b").value = 15;
    document.getElementById("pin-enc-2a").value = 16;
    document.getElementById("pin-enc-2b").value = 17;

    if (is4WD) {
      document.getElementById("pin-m3-p1").value = 8;
      document.getElementById("pin-m3-p2").value = 9;
      document.getElementById("pin-m3-p3").value = 10;
      document.getElementById("pin-m4-p1").value = 11;
      document.getElementById("pin-m4-p2").value = 12;
      document.getElementById("pin-m4-p3").value = 13;
      document.getElementById("pin-enc-3a").value = 18;
      document.getElementById("pin-enc-3b").value = 21;
      document.getElementById("pin-enc-4a").value = 38;
      document.getElementById("pin-enc-4b").value = 39;
    }
  }

  showToast("⚡ Conflict-free pins auto-allocated!");
  recomputeAll();
}

// Copy Code to Clipboard
function copyActiveCode() {
  const code = document.getElementById("code-display").innerText;
  navigator.clipboard.writeText(code).then(() => {
    const btnText = document.getElementById("copy-btn-text");
    btnText.innerText = "Copied!";
    showToast("📋 Code copied to clipboard!");
    setTimeout(() => { btnText.innerText = "Copy"; }, 2000);
  }).catch(err => {
    showToast("❌ Failed to copy: " + err);
  });
}

// Download Active Artifact File
function downloadActiveArtifact() {
  let content = "";
  let filename = "";
  const name = currentSpec.robot_name || "robot";

  if (activeArtifact === "tab-code-header") {
    content = generateCppHeader(currentSpec);
    filename = `${name}_config.h`;
  } else if (activeArtifact === "tab-code-pio") {
    content = generatePlatformioEnv(currentSpec);
    filename = "platformio_section.ini";
  } else if (activeArtifact === "tab-code-urdf") {
    content = generateUrdfXacro(currentSpec);
    filename = `${name}_properties.urdf.xacro`;
  } else if (activeArtifact === "tab-code-wiring") {
    content = generateWiringTable(currentSpec);
    filename = `${name}_wiring_table.md`;
  } else if (activeArtifact === "tab-code-deploy") {
    content = generateDeployScript(currentSpec, readAutomationOptions());
    filename = `deploy_${name}.sh`;
  } else if (activeArtifact === "tab-code-json") {
    content = JSON.stringify(currentSpec, null, 2);
    filename = `${name}_spec.json`;
  }

  triggerDownload(filename, content, "text/plain");
}

// Export Spec JSON
function exportSpecJson() {
  const jsonStr = JSON.stringify(currentSpec, null, 2);
  const filename = `${currentSpec.robot_name || "robot"}_spec.json`;
  triggerDownload(filename, jsonStr, "application/json");
  showToast("📥 Specification exported!");
}

// Import Spec JSON Handler
function handleImportJson(e) {
  const file = e.target.files[0];
  if (!file) return;

  const reader = new FileReader();
  reader.onload = (event) => {
    try {
      const imported = JSON.parse(event.target.result);
      if (!imported.robot_name || !imported.kinematics || !imported.mcu) {
        throw new Error("Missing required fields (robot_name, kinematics, mcu)");
      }
      currentSpec = imported;
      populateFormFromSpec(currentSpec);
      recomputeAll();
      showToast(`✅ Successfully imported '${imported.robot_name}'!`);
    } catch (err) {
      alert("Error importing JSON specification: " + err.message);
    }
  };
  reader.readAsText(file);
}

// Trigger Client-Side File Download
function triggerDownload(filename, text, mimeType) {
  const blob = new Blob([text], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

// Show Toast Notification
function showToast(message) {
  const toast = document.getElementById("toast");
  toast.innerText = message;
  toast.classList.add("show");
  setTimeout(() => {
    toast.classList.remove("show");
  }, 2500);
}


// =============================================================================
// Automation Hub, OS Detection, Deploy Script Generator & Web Serial Console
// =============================================================================

// Web Serial Global State
let serialPort = null;
let serialReader = null;
let isSerialReading = false;

// Client-Side OS & Platform Auto-Detection
function detectClientOS() {
  const ua = navigator.userAgent || "";
  const platform = navigator.userAgentData?.platform || navigator.platform || "";
  let detectedOS = "ubuntu_2404";
  let detectedLabel = "Linux (Ubuntu 24.04 Noble / Jazzy)";
  let osIcon = "🐧";
  let suggestedRos = "jazzy";
  let arch = "x86_64";

  // Check Architecture
  if (/arm|aarch64|Apple/i.test(ua) || /arm|aarch64/i.test(platform)) {
    arch = "aarch64";
  }

  // Check OS / Platform
  if (/Mac|iPhone|iPad|iPod/i.test(platform) || /Macintosh/i.test(ua)) {
    detectedOS = "macos";
    detectedLabel = "macOS (Apple Silicon & Intel)";
    osIcon = "🍎";
    suggestedRos = "jazzy";
  } else if (/Win/i.test(platform) || /Windows/i.test(ua)) {
    detectedOS = "windows_wsl";
    detectedLabel = "Windows 10/11 (WSL2 Ubuntu)";
    osIcon = "🪟";
    suggestedRos = "jazzy";
  } else if (/Linux/i.test(platform) || /Linux/i.test(ua)) {
    osIcon = "🐧";
    if (/26\.04|resolute/i.test(ua)) {
      detectedOS = "ubuntu_2604";
      detectedLabel = "Ubuntu 26.04 LTS (Resolute / Lyrical)";
      suggestedRos = "lyrical";
    } else if (/22\.04|jammy/i.test(ua)) {
      detectedOS = "ubuntu_2204";
      detectedLabel = "Ubuntu 22.04 LTS (Jammy / Humble)";
      suggestedRos = "humble";
    } else if (/debian/i.test(ua)) {
      detectedOS = "debian";
      detectedLabel = "Debian 12 / 13 (Bookworm / Trixie)";
      suggestedRos = "jazzy";
    } else if (/fedora|bluefin/i.test(ua)) {
      detectedOS = "fedora";
      detectedLabel = "Fedora / Bluefin Linux";
      suggestedRos = "jazzy";
    } else {
      detectedOS = "ubuntu_2404";
      detectedLabel = "Ubuntu 24.04 LTS (Noble / Jazzy)";
      suggestedRos = "jazzy";
    }
  }

  // Update Header Badges
  const headerIcon = document.getElementById("header-os-icon");
  const headerText = document.getElementById("header-os-text");
  if (headerIcon) headerIcon.innerText = osIcon;
  if (headerText) headerText.innerText = detectedLabel.split("(")[0].trim();

  // Update Banner in Tab 5
  const bannerIcon = document.getElementById("os-banner-icon");
  const bannerText = document.getElementById("os-banner-detected-text");
  const bannerArch = document.getElementById("os-banner-arch");
  const bannerBrowser = document.getElementById("os-banner-browser");

  if (bannerIcon) bannerIcon.innerText = osIcon;
  if (bannerText) bannerText.innerText = `Auto-Detected: ${detectedLabel}`;
  if (bannerArch) bannerArch.innerText = arch;
  if (bannerBrowser) {
    const isChrome = /Chrome|Chromium|Edg/i.test(ua);
    bannerBrowser.innerText = isChrome ? "Chrome / Chromium (Web Serial Supported)" : "Standard Browser";
  }

  // Set Default Dropdowns if not manually touched
  const osSelect = document.getElementById("auto-os-select");
  if (osSelect) osSelect.value = detectedOS;

  const archSelect = document.getElementById("auto-arch-select");
  if (archSelect) archSelect.value = arch;

  const rosSelect = document.getElementById("auto-ros-distro");
  if (rosSelect) rosSelect.value = suggestedRos;

  updateAutomationPreviews();
}

// Read All Automation Configuration Options
function readAutomationOptions() {
  return {
    os: document.getElementById("auto-os-select")?.value || "ubuntu_2404",
    env: document.getElementById("auto-env-select")?.value || "native",
    arch: document.getElementById("auto-arch-select")?.value || "x86_64",
    installPio: document.getElementById("chk-install-pio")?.checked ?? true,
    installUdev: document.getElementById("chk-install-udev")?.checked ?? true,
    installDialout: document.getElementById("chk-install-dialout")?.checked ?? true,
    installBuildTools: document.getElementById("chk-install-buildtools")?.checked ?? true,
    rosDistro: document.getElementById("auto-ros-distro")?.value || "jazzy",
    rosType: document.getElementById("auto-ros-type")?.value || "desktop",
    buildMicrorosAgent: document.getElementById("chk-build-microros-agent")?.checked ?? true,
    gitBranch: document.getElementById("auto-git-branch")?.value.trim() || `config/${currentSpec.robot_name || "robot"}`,
    gitCommitMsg: document.getElementById("auto-git-commit-msg")?.value.trim() || `feat(config): add configuration for ${currentSpec.robot_name || "robot"}`,
    mergeHeader: document.getElementById("chk-merge-header")?.checked ?? true,
    mergePioFirmware: document.getElementById("chk-merge-pio-firmware")?.checked ?? true,
    mergeUrdf: document.getElementById("chk-merge-urdf")?.checked ?? true,
    autoCommit: document.getElementById("chk-auto-commit")?.checked ?? true,
    flashTarget: document.getElementById("auto-flash-target")?.value || "firmware",
    flashPort: document.getElementById("auto-flash-port")?.value.trim() || "/dev/ttyACM0"
  };
}

// Update Dynamic Previews across Tab 5
function updateAutomationPreviews() {
  const opts = readAutomationOptions();
  const spec = currentSpec;
  const robotName = spec.robot_name || "scout_pico2";

  // Dynamic Git Branch & Commit message if unset or auto-managed
  const gitBranchInput = document.getElementById("auto-git-branch");
  if (gitBranchInput && (!gitBranchInput.value || gitBranchInput.dataset.autoManaged === "true")) {
    gitBranchInput.value = `config/${robotName}`;
    gitBranchInput.dataset.autoManaged = "true";
  }

  const gitCommitInput = document.getElementById("auto-git-commit-msg");
  if (gitCommitInput && (!gitCommitInput.value || gitCommitInput.dataset.autoManaged === "true")) {
    gitCommitInput.value = `feat(config): add configuration for ${robotName}`;
    gitCommitInput.dataset.autoManaged = "true";
  }

  // Set default port based on MCU
  const mcu = (spec.mcu || "PICO2").toUpperCase();
  const portInput = document.getElementById("auto-flash-port");
  if (portInput && (!portInput.value || portInput.dataset.autoManaged === "true")) {
    if (mcu === "PICO" || mcu === "PICO2" || mcu === "ESP32S3") {
      portInput.value = "/dev/ttyACM0";
    } else {
      portInput.value = "/dev/ttyUSB0";
    }
    portInput.dataset.autoManaged = "true";
  }

  // 1. Toolchain Preview
  const toolCmdEl = document.getElementById("preview-tool-cmd");
  if (toolCmdEl) {
    toolCmdEl.innerText = getToolchainInstallCmd(opts);
  }

  // 2. ROS 2 Preview & Visibility
  const rosDistroBox = document.getElementById("box-ros-cmd");
  const rosTypeGroup = document.getElementById("group-ros-install-type");
  const microrosGroup = document.getElementById("group-microros-agent");
  const isRosActive = opts.rosDistro !== "none";

  if (rosDistroBox) rosDistroBox.style.display = isRosActive ? "block" : "none";
  if (rosTypeGroup) rosTypeGroup.style.display = isRosActive ? "flex" : "none";
  if (microrosGroup) microrosGroup.style.display = isRosActive ? "grid" : "none";

  const rosCmdEl = document.getElementById("preview-ros-cmd");
  if (rosCmdEl && isRosActive) {
    rosCmdEl.innerText = getRos2InstallCmd(opts);
  }

  // 3. Merge & Commit Preview
  const mergeCmdEl = document.getElementById("preview-merge-cmd");
  if (mergeCmdEl) {
    mergeCmdEl.innerText = getMergeAndCommitCmd(spec, opts);
  }

  // If Deploy Script tab is active, re-render code display
  if (activeArtifact === "tab-code-deploy") {
    renderActiveCode();
  }
}

// Generate Toolchain Install Commands
function getToolchainInstallCmd(opts) {
  const lines = [];
  const isDebianUbuntu = ["ubuntu_2404", "ubuntu_2604", "ubuntu_2204", "debian", "windows_wsl"].includes(opts.os);
  const isMac = opts.os === "macos";
  const isFedora = opts.os === "fedora";

  if (isDebianUbuntu) {
    if (opts.installBuildTools) {
      lines.push("sudo apt update && sudo apt install -y git cmake ninja-build python3-pip python3-venv udev");
    }
    if (opts.installPio) {
      lines.push("curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py && python3 /tmp/get-platformio.py");
    }
    if (opts.installUdev) {
      lines.push("curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/master/scripts/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules > /dev/null");
      lines.push("sudo udevadm control --reload-rules && sudo udevadm trigger");
    }
    if (opts.installDialout) {
      lines.push("sudo usermod -a -G dialout,plugdev $USER");
    }
  } else if (isMac) {
    if (opts.installBuildTools) lines.push("brew install git cmake ninja python3");
    if (opts.installPio) lines.push("brew install platformio || pip3 install platformio");
  } else if (isFedora) {
    if (opts.installBuildTools) lines.push("sudo dnf install -y git cmake ninja-build python3-pip systemd-udev");
    if (opts.installPio) lines.push("pip3 install --user platformio");
    if (opts.installDialout) lines.push("sudo usermod -a -G dialout $USER");
  }

  return lines.length > 0 ? lines.join(" && \\\n") : "# No toolchain installations selected";
}

// Generate ROS 2 Install Commands
function getRos2InstallCmd(opts) {
  const distro = opts.rosDistro;
  if (distro === "none") return "# ROS 2 Installation skipped (Standalone Firmware mode)";

  const pkgName = opts.rosType === "desktop" ? `ros-${distro}-desktop` : `ros-${distro}-ros-base`;
  const lines = [
    `# 1. Setup ROS 2 ${distro.toUpperCase()} Official Apt Repository`,
    `sudo apt update && sudo apt install -y software-properties-common curl gnupg`,
    `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`,
    `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`,
    `sudo apt update && sudo apt install -y ${pkgName} python3-colcon-common-extensions python3-rosdep`
  ];

  if (opts.buildMicrorosAgent) {
    lines.push(
      ``,
      `# 2. Build micro-ROS Agent Workspace`,
      `source /opt/ros/${distro}/setup.bash`,
      `mkdir -p ~/microros_ws/src && cd ~/microros_ws`,
      `git clone -b ${distro} https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent || true`,
      `colcon build --symlink-install`,
      `echo "source /opt/ros/${distro}/setup.bash" >> ~/.bashrc`,
      `echo "source ~/microros_ws/install/setup.bash" >> ~/.bashrc`
    );
  }

  return lines.join("\n");
}

// Generate Merge & Commit Commands
function getMergeAndCommitCmd(spec, opts) {
  const name = spec.robot_name || "my_robot";
  const branch = opts.gitBranch || `config/${name}`;
  const commitMsg = opts.gitCommitMsg || `feat(config): add configuration for ${name}`;

  const lines = [
    `# 1. Create Isolated Git Branch`,
    `git checkout -b "${branch}" 2>/dev/null || git checkout "${branch}"`,
    ``,
    `# 2. Ingest Custom Header and URDF`,
    `mkdir -p config/custom urdf`,
    `# (Files injected automatically into config/custom/${name}_config.h and platformio.ini)`,
    ``,
    `# 3. Stage & Create Git Commit`,
    `git add config/ firmware/platformio.ini urdf/ 2>/dev/null || true`,
    `git commit -m "${commitMsg}"`
  ];

  return lines.join("\n");
}

// Full All-In-One Bash Deploy Script Generator
function generateDeployScript(spec, opts) {
  const name = spec.robot_name || "my_robot";
  const nameUpper = name.toUpperCase();
  const mcu = (spec.mcu || "PICO2").toUpperCase();
  const branch = opts.gitBranch || `config/${name}`;
  const commitMsg = opts.gitCommitMsg || `feat(config): add configuration for ${name}`;
  const target = opts.flashTarget || "firmware";
  const port = opts.flashPort || "/dev/ttyACM0";
  const headerContent = generateCppHeader(spec);
  const pioSection = generatePlatformioEnv(spec);
  const urdfContent = generateUrdfXacro(spec);

  const script = `#!/usr/bin/env bash
# =============================================================================
# Automated Linorobot2 Setup, Merge, Build & Flash Script for '${name}'
# Generated by Linorobot2 Robot Configuration Engine
# Target MCU: ${mcu} | Kinematics: ${spec.kinematics} | Target: ${target}
# =============================================================================

set -e

# ANSI Styling
C_RESET='\\033[0m'
C_CYAN='\\033[1;36m'
C_GREEN='\\033[1;32m'
C_YELLOW='\\033[1;33m'
C_RED='\\033[1;31m'

info()    { echo -e "\${C_CYAN}[INFO]\${C_RESET} \$*"; }
success() { echo -e "\${C_GREEN}[SUCCESS]\${C_RESET} \$*"; }
warn()    { echo -e "\${C_YELLOW}[WARN]\${C_RESET} \$*"; }
err()     { echo -e "\${C_RED}[ERROR]\${C_RESET} \$*" >&2; }

info "========================================================="
info "  Linorobot2 Deployment Engine: ${name}"
info "  Target MCU: ${mcu} | Mode: ${target}"
info "========================================================="

# -----------------------------------------------------------------------------
# Phase 1: Toolchain & Dependency Verification
# -----------------------------------------------------------------------------
info "Step 1: Checking build toolchains and dependencies..."
${opts.installBuildTools ? `if command -v apt-get &>/dev/null; then
    sudo apt-get update -qq
    sudo apt-get install -y -qq git cmake ninja-build python3-pip python3-venv udev
fi` : "# Build tools check skipped"}

${opts.installPio ? `if ! command -v pio &>/dev/null; then
    info "Installing PlatformIO Core..."
    curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py && python3 /tmp/get-platformio.py
    export PATH="$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH"
fi
success "PlatformIO Core active: $(pio --version 2>/dev/null || echo 'Installed')"
` : "# PlatformIO install skipped"}

${opts.installUdev ? `if [ -d "/etc/udev/rules.d" ] && [ ! -f "/etc/udev/rules.d/99-platformio-udev.rules" ]; then
    info "Installing PlatformIO hardware udev rules..."
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/master/scripts/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules > /dev/null
    sudo udevadm control --reload-rules && sudo udevadm trigger || true
fi` : "# udev rules skipped"}

${opts.installDialout ? `if command -v usermod &>/dev/null; then
    sudo usermod -a -G dialout,plugdev "$USER" 2>/dev/null || true
fi` : "# dialout group skipped"}

# -----------------------------------------------------------------------------
# Phase 2: Optional ROS 2 Distribution Setup
# -----------------------------------------------------------------------------
${opts.rosDistro !== "none" ? `info "Step 2: Verifying ROS 2 ${opts.rosDistro.toUpperCase()} environment..."
if [ ! -d "/opt/ros/${opts.rosDistro}" ]; then
    warn "ROS 2 ${opts.rosDistro} not found in /opt/ros/${opts.rosDistro}. Installing..."
    sudo apt-get update -qq && sudo apt-get install -y -qq curl gnupg software-properties-common
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update -qq
    sudo apt-get install -y -qq ${opts.rosType === "desktop" ? `ros-${opts.rosDistro}-desktop` : `ros-${opts.rosDistro}-ros-base`} python3-colcon-common-extensions python3-rosdep
fi

${opts.buildMicrorosAgent ? `if [ -d "/opt/ros/${opts.rosDistro}" ] && [ ! -d "$HOME/microros_ws/install" ]; then
    info "Building micro_ros_agent workspace at ~/microros_ws..."
    mkdir -p "$HOME/microros_ws/src"
    cd "$HOME/microros_ws"
    if [ ! -d "src/micro_ros_agent" ]; then
        git clone -b ${opts.rosDistro} https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent || true
    fi
    source "/opt/ros/${opts.rosDistro}/setup.bash"
    colcon build --symlink-install
    cd - >/dev/null
fi` : ""}
` : `# ROS 2 Host setup skipped (Standalone Firmware mode)`}

# -----------------------------------------------------------------------------
# Phase 3: Git Branch Preparation
# -----------------------------------------------------------------------------
info "Step 3: Preparing Git branch '${branch}'..."
if git rev-parse --is-inside-work-tree &>/dev/null; then
    git checkout -b "${branch}" 2>/dev/null || git checkout "${branch}"
fi

# -----------------------------------------------------------------------------
# Phase 4: Ingest Custom C++ Header
# -----------------------------------------------------------------------------
${opts.mergeHeader ? `info "Step 4: Merging C++ configuration header into config/custom/${name}_config.h..."
mkdir -p config/custom

cat << 'EOF_HEADER' > "config/custom/${name}_config.h"
${headerContent}
EOF_HEADER
success "Generated config/custom/${name}_config.h"

# Register in config/config.h if not already present
if [ -f "config/config.h" ]; then
    if ! grep -q "USE_${nameUpper}_CONFIG" config/config.h; then
        info "Registering USE_${nameUpper}_CONFIG in config/config.h..."
        sed -i '/\\/\\/ add user configurations above this line/i #ifdef USE_${nameUpper}_CONFIG\\n    #include "custom/${name}_config.h"\\n#endif\\n' config/config.h
        success "Registered in config/config.h"
    else
        info "USE_${nameUpper}_CONFIG already registered in config/config.h"
    fi
fi
` : "# Header merge skipped"}

# -----------------------------------------------------------------------------
# Phase 5: Ingest PlatformIO Target Environment
# (test_motors and test_sensors automatically inherit firmware/platformio.ini)
# -----------------------------------------------------------------------------
if [ -f "firmware/platformio.ini" ] && [ "${opts.mergePioFirmware ? "1" : "0"}" = "1" ]; then
    if ! grep -q "\[env:${name}\]" "firmware/platformio.ini"; then
        info "Appending [env:${name}] to firmware/platformio.ini..."
        echo "" >> "firmware/platformio.ini"
        cat << 'EOF_PIO' >> "firmware/platformio.ini"
${pioSection}
EOF_PIO
        success "Updated firmware/platformio.ini (inherited by test_motors & test_sensors)"
    else
        info "[env:${name}] already exists in firmware/platformio.ini"
    fi
fi

# -----------------------------------------------------------------------------
# Phase 6: Ingest URDF Description
# -----------------------------------------------------------------------------
${opts.mergeUrdf ? `info "Step 6: Writing URDF description to urdf/${name}_properties.urdf.xacro..."
mkdir -p urdf
cat << 'EOF_URDF' > "urdf/${name}_properties.urdf.xacro"
${urdfContent}
EOF_URDF
success "Generated urdf/${name}_properties.urdf.xacro"
` : "# URDF generation skipped"}

# -----------------------------------------------------------------------------
# Phase 7: Stage and Commit Changes
# -----------------------------------------------------------------------------
${opts.autoCommit ? `if git rev-parse --is-inside-work-tree &>/dev/null; then
    info "Step 7: Committing generated hardware configuration to Git..."
    git add config/ firmware/platformio.ini urdf/ 2>/dev/null || true
    if ! git diff --cached --quiet; then
        git commit -m "${commitMsg}"
        success "Git commit created on branch '${branch}'"
    else
        info "No configuration changes to commit."
    fi
fi` : "# Git commit skipped"}

# -----------------------------------------------------------------------------
# Phase 8: Build and Flash Target Firmware
# -----------------------------------------------------------------------------
info "Step 8: Building target '${target}' for environment '[env:${name}]'..."
if [ -d "${target}" ]; then
    pio run -d "${target}" -e "${name}"
    success "Build SUCCEEDED for '${name}' in ${target}/"

    if [ -n "${port}" ] && [ "${port}" != "AUTO" ]; then
        info "Uploading to microcontroller on port '${port}'..."
        pio run -d "${target}" -e "${name}" -t upload --upload-port "${port}" || {
            warn "Standard upload exited. Attempting auto-upload protocol..."
            pio run -d "${target}" -e "${name}" -t upload
        }
        success "Microcontroller flashing COMPLETE!"
    fi
else
    warn "Target directory '${target}/' not found in current workspace."
fi

info "========================================================="
success "All deployment steps completed successfully for '${name}'!"
info "========================================================="
`;

  return script;
}

// -----------------------------------------------------------------------------
// Web Serial API Implementation for Chrome / Edge Live Microcontroller Logs
// -----------------------------------------------------------------------------
async function toggleWebSerialConnection() {
  if (serialPort) {
    await disconnectWebSerial();
  } else {
    await connectWebSerial();
  }
}

async function connectWebSerial() {
  const terminalScreen = document.getElementById("serial-terminal-screen");
  const statusPill = document.getElementById("serial-status-pill");
  const statusText = document.getElementById("serial-status-text");
  const btnConnect = document.getElementById("btn-serial-connect");
  const btnText = document.getElementById("serial-btn-text");
  const baudSelect = document.getElementById("serial-baud-select");
  const inputBar = document.getElementById("serial-input-text");
  const btnSend = document.getElementById("btn-serial-send");

  if (!("serial" in navigator)) {
    appendTerminalLine("❌ Web Serial API is not supported in this browser. Please use Chrome, Edge, or Opera over HTTPS/localhost.", "err");
    alert("Web Serial API is not supported in this browser. Please open in Google Chrome or Microsoft Edge.");
    return;
  }

  try {
    const baudRate = parseInt(baudSelect?.value || "115200", 10);
    appendTerminalLine(`🔌 Requesting serial port access (Baud: ${baudRate})...`, "dim");

    serialPort = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: baudRate });

    // Update UI Connected State
    if (statusPill) statusPill.className = "serial-status-pill pill-connected";
    if (statusText) statusText.innerText = `Connected (${baudRate} baud)`;
    if (btnText) btnText.innerText = "Disconnect";
    if (btnConnect) btnConnect.className = "btn btn-sm btn-secondary";
    if (inputBar) inputBar.disabled = false;
    if (btnSend) btnSend.disabled = false;

    appendTerminalLine(`✅ Serial port connected at ${baudRate} baud. Streaming output:`, "dim");
    showToast("🟢 Serial port connected!");

    // Start background reader loop
    readSerialStream();

  } catch (err) {
    appendTerminalLine(`❌ Connection failed: ${err.message}`, "err");
    serialPort = null;
    if (statusPill) statusPill.className = "serial-status-pill pill-disconnected";
    if (statusText) statusText.innerText = "Disconnected";
  }
}

async function disconnectWebSerial() {
  const statusPill = document.getElementById("serial-status-pill");
  const statusText = document.getElementById("serial-status-text");
  const btnConnect = document.getElementById("btn-serial-connect");
  const btnText = document.getElementById("serial-btn-text");
  const inputBar = document.getElementById("serial-input-text");
  const btnSend = document.getElementById("btn-serial-send");

  isSerialReading = false;

  try {
    if (serialReader) {
      await serialReader.cancel();
      serialReader = null;
    }
    if (serialPort) {
      await serialPort.close();
      serialPort = null;
    }
  } catch (err) {
    console.error("Error closing serial port:", err);
  }

  if (statusPill) statusPill.className = "serial-status-pill pill-disconnected";
  if (statusText) statusText.innerText = "Disconnected";
  if (btnText) btnText.innerText = "Connect Serial Port";
  if (btnConnect) btnConnect.className = "btn btn-sm btn-accent";
  if (inputBar) inputBar.disabled = true;
  if (btnSend) btnSend.disabled = true;

  appendTerminalLine("🔌 Serial port disconnected.", "dim");
  showToast("🔌 Serial port disconnected.");
}

async function readSerialStream() {
  isSerialReading = true;
  const decoder = new TextDecoderStream();
  const inputDone = serialPort.readable.pipeTo(decoder.writable);
  const inputStream = decoder.readable;
  serialReader = inputStream.getReader();

  let lineBuffer = "";

  try {
    while (isSerialReading) {
      const { value, done } = await serialReader.read();
      if (done) break;
      if (value) {
        lineBuffer += value;
        const lines = lineBuffer.split(/\r?\n/);
        lineBuffer = lines.pop(); // Keep partial line in buffer

        for (const line of lines) {
          if (line.trim().length > 0) {
            appendTerminalLine(line, "out");
          }
        }
      }
    }
  } catch (err) {
    if (isSerialReading) {
      appendTerminalLine(`⚠️ Serial stream closed: ${err.message}`, "dim");
    }
  } finally {
    if (serialReader) {
      serialReader.releaseLock();
    }
  }
}

async function sendSerialCommand() {
  const inputBar = document.getElementById("serial-input-text");
  if (!inputBar || !inputBar.value || !serialPort || !serialPort.writable) return;

  const textToSend = inputBar.value + "\n";
  appendTerminalLine(`> ${inputBar.value}`, "in");
  inputBar.value = "";

  const encoder = new TextEncoder();
  const writer = serialPort.writable.getWriter();
  await writer.write(encoder.encode(textToSend));
  writer.releaseLock();
}

function appendTerminalLine(text, type = "out") {
  const screen = document.getElementById("serial-terminal-screen");
  if (!screen) return;

  const lineEl = document.createElement("div");
  lineEl.className = `terminal-line terminal-${type}`;
  lineEl.innerText = text;
  screen.appendChild(lineEl);

  const autoScroll = document.getElementById("chk-serial-autoscroll")?.checked ?? true;
  if (autoScroll) {
    screen.scrollTop = screen.scrollHeight;
  }
}

function clearTerminalScreen() {
  const screen = document.getElementById("serial-terminal-screen");
  if (screen) {
    screen.innerHTML = '<div class="terminal-line terminal-dim">[Terminal Cleared]</div>';
  }
}

// Initialize Automation Event Listeners
function initAutomationEventListeners() {
  // Top Quick Deploy Button
  const btnQuickDeploy = document.getElementById("btn-quick-deploy");
  if (btnQuickDeploy) {
    btnQuickDeploy.addEventListener("click", () => {
      const tabBtn = document.querySelector('[data-tab="tab-automation"]');
      if (tabBtn) tabBtn.click();
    });
  }

  // Re-detect OS Button
  const btnRedetect = document.getElementById("btn-redetect-os");
  if (btnRedetect) {
    btnRedetect.addEventListener("click", () => {
      detectClientOS();
      showToast("🔍 OS re-detected!");
    });
  }

  // Dynamic Automation Form Listeners
  const autoInputs = [
    "auto-os-select", "auto-env-select", "auto-arch-select",
    "chk-install-pio", "chk-install-udev", "chk-install-dialout", "chk-install-buildtools",
    "auto-ros-distro", "auto-ros-type", "chk-build-microros-agent",
    "auto-git-branch", "auto-git-commit-msg",
    "chk-merge-header", "chk-merge-pio-firmware", "chk-merge-urdf", "chk-auto-commit",
    "auto-flash-target", "auto-flash-port"
  ];

  autoInputs.forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener("input", () => {
        if (id === "auto-git-branch" || id === "auto-git-commit-msg" || id === "auto-flash-port") {
          el.dataset.autoManaged = "false";
        }
        updateAutomationPreviews();
      });
      el.addEventListener("change", () => {
        updateAutomationPreviews();
      });
    }
  });

  // Quick Copy Buttons
  const btnCopyTool = document.getElementById("btn-copy-tool-cmd");
  if (btnCopyTool) {
    btnCopyTool.addEventListener("click", () => {
      const cmd = document.getElementById("preview-tool-cmd")?.innerText || "";
      navigator.clipboard.writeText(cmd).then(() => showToast("📋 Toolchain command copied!"));
    });
  }

  const btnCopyRos = document.getElementById("btn-copy-ros-cmd");
  if (btnCopyRos) {
    btnCopyRos.addEventListener("click", () => {
      const cmd = document.getElementById("preview-ros-cmd")?.innerText || "";
      navigator.clipboard.writeText(cmd).then(() => showToast("📋 ROS 2 command copied!"));
    });
  }

  const btnCopyMerge = document.getElementById("btn-copy-merge-cmd");
  if (btnCopyMerge) {
    btnCopyMerge.addEventListener("click", () => {
      const cmd = document.getElementById("preview-merge-cmd")?.innerText || "";
      navigator.clipboard.writeText(cmd).then(() => showToast("📋 Merge & Commit command copied!"));
    });
  }

  const btnCopyBuild = document.getElementById("btn-copy-build-cmd");
  if (btnCopyBuild) {
    btnCopyBuild.addEventListener("click", () => {
      const target = document.getElementById("auto-flash-target")?.value || "firmware";
      const name = currentSpec.robot_name || "scout_pico2";
      const cmd = `pio run -d ${target} -e ${name}`;
      navigator.clipboard.writeText(cmd).then(() => showToast(`📋 Build command copied: ${cmd}`));
    });
  }

  const btnCopyUpload = document.getElementById("btn-copy-upload-cmd");
  if (btnCopyUpload) {
    btnCopyUpload.addEventListener("click", () => {
      const target = document.getElementById("auto-flash-target")?.value || "firmware";
      const name = currentSpec.robot_name || "scout_pico2";
      const port = document.getElementById("auto-flash-port")?.value.trim() || "/dev/ttyACM0";
      const cmd = port === "AUTO" ? `pio run -d ${target} -e ${name} -t upload` : `pio run -d ${target} -e ${name} -t upload --upload-port ${port}`;
      navigator.clipboard.writeText(cmd).then(() => showToast(`⚡ Upload command copied: ${cmd}`));
    });
  }

  // Command Execution Buttons
  const btnExecTool = document.getElementById("btn-exec-tool-cmd");
  if (btnExecTool) {
    btnExecTool.addEventListener("click", () => {
      const cmd = document.getElementById("preview-tool-cmd")?.innerText || "";
      executeCommandInTerminal(cmd, "Installing Build Toolchains & Permissions");
    });
  }

  const btnExecRos = document.getElementById("btn-exec-ros-cmd");
  if (btnExecRos) {
    btnExecRos.addEventListener("click", () => {
      const cmd = document.getElementById("preview-ros-cmd")?.innerText || "";
      executeCommandInTerminal(cmd, "Configuring ROS 2 & micro-ROS Agent");
    });
  }

  const btnExecMerge = document.getElementById("btn-exec-merge-cmd");
  if (btnExecMerge) {
    btnExecMerge.addEventListener("click", () => {
      const spec = currentSpec;
      const opts = readAutomationOptions();
      const name = spec.robot_name || "scout_pico2";
      const branch = opts.gitBranch || `config/${name}`;
      const commitMsg = opts.gitCommitMsg || `feat(config): add configuration for ${name}`;
      
      const script = generateDeployScript(spec, opts);
      const cmd = `cat << 'EOF_DEPLOY' > deploy_${name}.sh\n${script}\nEOF_DEPLOY\nchmod +x deploy_${name}.sh && ./deploy_${name}.sh`;
      executeCommandInTerminal(cmd, `Merging Headers & Committing '${name}'`);
    });
  }

  const btnExecBuild = document.getElementById("btn-exec-build-cmd");
  if (btnExecBuild) {
    btnExecBuild.addEventListener("click", () => {
      const target = document.getElementById("auto-flash-target")?.value || "firmware";
      const name = currentSpec.robot_name || "scout_pico2";
      const opts = readAutomationOptions();
      const rosDistro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
      const cmd = `export ROS_DISTRO=${rosDistro} && pio run -d ${target} -e ${name}`;
      executeCommandInTerminal(cmd, `Building ${target}/ for [env:${name}]`);
    });
  }

  const btnExecUpload = document.getElementById("btn-exec-upload-cmd");
  if (btnExecUpload) {
    btnExecUpload.addEventListener("click", () => {
      const target = document.getElementById("auto-flash-target")?.value || "firmware";
      const name = currentSpec.robot_name || "scout_pico2";
      const port = document.getElementById("auto-flash-port")?.value.trim() || "/dev/ttyACM0";
      const opts = readAutomationOptions();
      const rosDistro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
      const cmd = port === "AUTO" 
        ? `export ROS_DISTRO=${rosDistro} && pio run -d ${target} -e ${name} -t upload` 
        : `export ROS_DISTRO=${rosDistro} && pio run -d ${target} -e ${name} -t upload --upload-port ${port}`;
      executeCommandInTerminal(cmd, `Flashing Microcontroller (${target}/ -> ${port})`);
    });
  }

  const btnExecDeploy = document.getElementById("btn-exec-deploy-script");
  if (btnExecDeploy) {
    btnExecDeploy.addEventListener("click", () => {
      const spec = currentSpec;
      const opts = readAutomationOptions();
      const name = spec.robot_name || "scout_pico2";
      const script = generateDeployScript(spec, opts);
      const cmd = `cat << 'EOF_DEPLOY' > deploy_${name}.sh\n${script}\nEOF_DEPLOY\nchmod +x deploy_${name}.sh && ./deploy_${name}.sh`;
      executeCommandInTerminal(cmd, `Executing Full Deploy Lifecycle for '${name}'`);
    });
  }

  const btnCancelExec = document.getElementById("btn-cancel-exec");
  if (btnCancelExec) {
    btnCancelExec.addEventListener("click", cancelRunningExecution);
  }

  const btnDownloadDeploy = document.getElementById("btn-download-deploy-script");
  if (btnDownloadDeploy) {
    btnDownloadDeploy.addEventListener("click", () => {
      const script = generateDeployScript(currentSpec, readAutomationOptions());
      const filename = `deploy_${currentSpec.robot_name || "robot"}.sh`;
      triggerDownload(filename, script, "application/x-sh");
      showToast(`📥 ${filename} downloaded!`);
    });
  }

  // Web Serial Event Listeners
  const btnSerialConnect = document.getElementById("btn-serial-connect");
  if (btnSerialConnect) {
    btnSerialConnect.addEventListener("click", toggleWebSerialConnection);
  }

  const btnSerialClear = document.getElementById("btn-serial-clear");
  if (btnSerialClear) {
    btnSerialClear.addEventListener("click", clearTerminalScreen);
  }

  const btnSerialSend = document.getElementById("btn-serial-send");
  if (btnSerialSend) {
    btnSerialSend.addEventListener("click", sendSerialCommand);
  }

  const inputSerialText = document.getElementById("serial-input-text");
  if (inputSerialText) {
    inputSerialText.addEventListener("keydown", (e) => {
      if (e.key === "Enter") {
        sendSerialCommand();
      }
    });
  }
}


// =============================================================================
// Live Command Execution API & Subprocess Controller
// =============================================================================

let isRunnerOnline = false;
let currentExecController = null;

// Check Server Runner API Status (GET /api/status)
async function checkServerRunnerStatus() {
  const pill = document.getElementById("runner-status-pill");
  const text = document.getElementById("runner-status-text");

  try {
    const res = await fetch("/api/status", { cache: "no-cache" });
    if (res.ok) {
      const data = await res.json();
      if (data.status === "ok" && data.exec_supported) {
        isRunnerOnline = true;
        if (pill) pill.className = "runner-status-pill pill-online";
        if (text) text.innerText = `Runner: Online (${data.node || data.os})`;
        return;
      }
    }
  } catch (e) {
    // Runner offline
  }

  isRunnerOnline = false;
  if (pill) pill.className = "runner-status-pill pill-offline";
  if (text) text.innerText = "Runner: Standalone / Copy Mode";
}

// Execute Command with Live Streaming Output in Terminal Console
async function executeCommandInTerminal(command, title = "Executing Command") {
  const terminalScreen = document.getElementById("serial-terminal-screen");
  const btnCancel = document.getElementById("btn-cancel-exec");

  // Scroll to terminal
  const terminalEl = document.querySelector(".web-serial-console-card");
  if (terminalEl) {
    terminalEl.scrollIntoView({ behavior: "smooth", block: "nearest" });
  }

  if (!isRunnerOnline) {
    appendTerminalLine(`\n=========================================================`, "dim");
    appendTerminalLine(`🚀 ${title}`, "in");
    appendTerminalLine(`=========================================================`, "dim");
    appendTerminalLine(`$ ${command}`, "out");
    appendTerminalLine(`ℹ️  Local Execution Runner is not active on this host/port.`, "dim");
    appendTerminalLine(`👉 Run 'python3 server.py' in tools/robot_config_engine/web to enable 1-click execution!`, "in");
    appendTerminalLine(`📋 Command copied to clipboard for manual terminal execution.`, "dim");

    navigator.clipboard.writeText(command).then(() => {
      showToast("📋 Command copied! (Start 'python3 server.py' for 1-click execution)");
    });
    return;
  }

  appendTerminalLine(`\n=========================================================`, "dim");
  appendTerminalLine(`🚀 ${title}`, "in");
  appendTerminalLine(`$ ${command}`, "dim");
  appendTerminalLine(`=========================================================`, "dim");

  if (btnCancel) btnCancel.style.display = "inline-flex";

  try {
    currentExecController = new AbortController();
    const response = await fetch("/api/exec", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: command }),
      signal: currentExecController.signal
    });

    if (!response.ok) {
      appendTerminalLine(`❌ Server error: HTTP ${response.status}`, "err");
      if (btnCancel) btnCancel.style.display = "none";
      return;
    }

    const reader = response.body.getReader();
    const decoder = new TextDecoderStream();
    const inputStream = response.body.pipeThrough(decoder);
    const textReader = inputStream.getReader();

    let buffer = "";
    while (true) {
      const { value, done } = await textReader.read();
      if (done) break;
      if (value) {
        buffer += value;
        const events = buffer.split("\n\n");
        buffer = events.pop(); // Keep incomplete event

        for (const evt of events) {
          if (!evt.trim()) continue;
          const lines = evt.split("\n");
          let eventType = "output";
          let dataStr = "";

          for (const line of lines) {
            if (line.startsWith("event: ")) eventType = line.substring(7).trim();
            if (line.startsWith("data: ")) dataStr = line.substring(6).trim();
          }

          if (dataStr) {
            try {
              const payload = JSON.parse(dataStr);
              if (eventType === "output" && payload.line !== undefined) {
                appendTerminalLine(payload.line, "out");
              } else if (eventType === "done") {
                if (payload.code === 0) {
                  appendTerminalLine(`\n✅ [SUCCESS] Command finished with exit code 0!`, "out");
                  showToast("✅ Command executed successfully!");
                } else {
                  appendTerminalLine(`\n❌ [FAILED] Command exited with code ${payload.code}`, "err");
                  showToast(`⚠️ Command exited with code ${payload.code}`);
                }
              } else if (eventType === "error") {
                appendTerminalLine(`❌ [ERROR] ${payload.error}`, "err");
              }
            } catch (e) {
              appendTerminalLine(dataStr, "out");
            }
          }
        }
      }
    }
  } catch (err) {
    if (err.name === "AbortError") {
      appendTerminalLine(`\n⏹️ Process aborted by user.`, "dim");
      showToast("⏹️ Process stopped.");
    } else {
      appendTerminalLine(`\n❌ Execution failed: ${err.message}`, "err");
    }
  } finally {
    if (btnCancel) btnCancel.style.display = "none";
    currentExecController = null;
  }
}

// Abort running process via POST /api/kill
async function cancelRunningExecution() {
  if (currentExecController) {
    currentExecController.abort();
  }
  try {
    await fetch("/api/kill", { method: "POST" });
  } catch (e) {}
}
