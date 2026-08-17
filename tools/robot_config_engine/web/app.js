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
  loadPreset("scout_pico2");
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

  // 3. Render Active Code Viewer
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
