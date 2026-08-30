// Hardware USB MCU Classifier & Auto-detection Engine
function guessMcuFromUsbDetail(detail) {
  if (!detail) return null;
  const vid = (detail.vid || "").toLowerCase();
  const pid = (detail.pid || "").toLowerCase();
  const chip = (detail.chip || "").toLowerCase();
  const product = (detail.product || "").toLowerCase();
  const mfr = (detail.manufacturer || "").toLowerCase();

  if (detail.is_bootsel || (vid === "2e8a" && (pid === "0003" || pid === "000a"))) {
    return { mcu: "PICO", baudrate: 921600, chipName: "Raspberry Pi Pico (RP2040) [BOOTSEL Mode]", preset: "pico_bare", isBootsel: true };
  }
  if (detail.is_bootsel || (vid === "2e8a" && (pid === "000f" || pid === "0005"))) {
    return { mcu: "PICO2", baudrate: 921600, chipName: "Raspberry Pi Pico 2 (RP2350) [BOOTSEL Mode]", preset: "pico2_bare", isBootsel: true };
  }
  if (detail.is_bootsel || (vid === "2e8a" && (pid === "0003" || pid === "000a"))) {
    return { mcu: "PICO", baudrate: 921600, chipName: "Raspberry Pi Pico (RP2040) [BOOTSEL Mode]", preset: "pico_bare", isBootsel: true };
  }
  if (detail.is_bootsel || (vid === "2e8a" && (pid === "000f" || pid === "0005"))) {
    return { mcu: "PICO2", baudrate: 921600, chipName: "Raspberry Pi Pico 2 (RP2350) [BOOTSEL Mode]", preset: "pico2_bare", isBootsel: true };
  }
  if (vid === "2e8a" || chip.includes("rp2350") || chip.includes("pico 2") || product.includes("pico 2")) {
    return { mcu: "PICO2", baudrate: 921600, chipName: "Raspberry Pi Pico 2 (RP2350)", preset: "pico2_bare" };
  }
  if (chip.includes("rp2040") || chip.includes("pico") || product.includes("pico")) {
    return { mcu: "PICO", baudrate: 921600, chipName: "Raspberry Pi Pico (RP2040)", preset: "pico_bare" };
  }
  if (vid === "303a" || chip.includes("esp32-s3") || product.includes("esp32-s3")) {
    return { mcu: "ESP32S3", baudrate: 921600, chipName: "ESP32-S3 (Native USB CDC)", preset: "esp32s3_bare" };
  }
  if (chip.includes("gendrv") || product.includes("general driver") || (chip.includes("cp2102") && mfr.includes("silicon"))) {
    return { mcu: "ESP32", baudrate: 921600, chipName: "ESP32 DevKit (CP2102N)", preset: "esp32_bare" };
  }
  if (chip.includes("esp32") || chip.includes("cp210") || chip.includes("ch340") || chip.includes("ftdi")) {
    return { mcu: "ESP32", baudrate: 921600, chipName: "ESP32 Microcontroller", preset: "esp32_bare" };
  }
  return null;
}

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
  scout_pico2w: {
    robot_name: "scout_pico2w",
    kinematics: "DIFFERENTIAL_DRIVE",
    mcu: "PICO2W",
    transport: "SERIAL",
    wifi_telemetry: false,
    geometry: { wheel_diameter: 0.08, track_width: 0.22, wheelbase: 0.20, weight: 3.5 },
    motors: { driver_type: "GENERIC_2_IN", max_rpm: 330, cpr: 1320, motor1_inv: false, motor2_inv: true },
    sensors: { imu: "FAKE", mag: "NONE", battery_monitor: "NONE", sonar: false }
    ,
    pins: {
      motor1: { pwm: 14, in_a: 15, in_b: 13 },
      motor2: { pwm: 16, in_a: 17, in_b: 12 },
      encoders: { m1_a: 2, m1_b: 3, m2_a: 4, m2_b: 5 },
      i2c: { sda: 8, scl: 9 },
      battery_pin: 26,
      sonar: { trig: 18, echo: 19 }
    }
  },
  scout_pico2: {
    robot_name: "scout_pico2",
    kinematics: "DIFFERENTIAL_DRIVE",
    mcu: "PICO2",
    transport: "SERIAL",
    geometry: {
      wheel_diameter: 0.08,
      track_width: 0.22,
      wheelbase: 0.20,
      weight: 3.5
    },
    motors: {
      driver_type: "GENERIC_2_IN",
      max_rpm: 330,
      cpr: 1320,
      operating_voltage: 12.0,
      rated_torque: 1.5,
      rated_voltage: 12.0,
      motor1_inv: false,
      motor2_inv: true,
      motor3_inv: false,
      motor4_inv: true
    },
    sensors: {
      imu: "FAKE",
      mag: "NONE",
      battery_monitor: "NONE",
      sonar: false
    }
    ,
    pins: {
      led: 25,
      motor1: { pwm: 10, in_a: 11, in_b: 12 },
      motor2: { pwm: 13, in_a: 14, in_b: 15 },
      motor3: { pwm: 16, in_a: 17, in_b: 18 },
      motor4: { pwm: 19, in_a: 20, in_b: 21 },
      encoders: { m1_a: 2, m1_b: 3, m2_a: 4, m2_b: 5, m3_a: 6, m3_b: 7, m4_a: 8, m4_b: 9 },
      i2c: { sda: 0, scl: 1 },
      battery_pin: 26,
      sonar: { trig: 22, echo: 27 }
    }
  },

  mech_pico2: {
    robot_name: "mech_pico2",
    kinematics: "MECANUM",
    mcu: "PICO2",
    transport: "SERIAL",
    geometry: {
      wheel_diameter: 0.08,
      track_width: 0.26,
      wheelbase: 0.22,
      weight: 4.0
    },
    motors: {
      driver_type: "GENERIC_2_IN",
      max_rpm: 330,
      cpr: 1440,
      operating_voltage: 12.0,
      rated_torque: 2.0,
      rated_voltage: 12.0,
      motor1_inv: false,
      motor2_inv: true,
      motor3_inv: false,
      motor4_inv: true
    },
    sensors: {
      imu: "FAKE",
      mag: "NONE",
      battery_monitor: "ADC_DIVIDER",
      battery_capacity: 3.0,
      battery_nominal_voltage: 11.1,
      battery_min_voltage: 9.0,
      battery_max_voltage: 12.6,
      battery_dip: 0.98,
      sonar: true
    },
    pins: {
      led: 25,
      motor1: { pwm: 10, in_a: 11, in_b: 12 },
      motor2: { pwm: 13, in_a: 14, in_b: 15 },
      motor3: { pwm: 16, in_a: 17, in_b: 18 },
      motor4: { pwm: 19, in_a: 20, in_b: 21 },
      encoders: { m1_a: 2, m1_b: 3, m2_a: 4, m2_b: 5, m3_a: 6, m3_b: 7, m4_a: 8, m4_b: 9 },
      i2c: { sda: 0, scl: 1 },
      battery_pin: 26,
      sonar: { trig: 22, echo: 27 }
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
      imu: "FAKE",
      mag: "NONE",
      battery_monitor: "NONE",
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

// Current Active Spec State
let currentSpec = {};
let activeArtifact = "tab-code-header";

function isESP32(mcu) {
  return typeof mcu === "string" && (mcu.startsWith("ESP32") || mcu === "GENDRV");
}

// DOM Elements Initialization
document.addEventListener("DOMContentLoaded", () => {
  initNavTabs();
  initArtifactTabs();
  initEventListeners();
  initAutomationEventListeners();
  initAdcCalibrationStudio();
  initRos2TopicInspector();
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
  document.getElementById("import-file").addEventListener("change", handleImportFile);

  // AI I2C Sensor Auto-Detection Button
  const btnI2cDetect = document.getElementById("btn-auto-detect-i2c");
  if (btnI2cDetect) {
    btnI2cDetect.addEventListener("click", runI2cSensorAutoDetect);
  }
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
  if (document.getElementById("cfg-baudrate")) {
    document.getElementById("cfg-baudrate").value = String(spec.baudrate || spec.telemetry?.baudrate || (spec.mcu === "GENDRV" ? 1500000 : 921600));
  }
  if (document.getElementById("cfg-serial-interface")) {
    document.getElementById("cfg-serial-interface").value = spec.serial_interface || "CDC";
  }

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
  if (document.getElementById("cfg-weight")) document.getElementById("cfg-weight").value = spec.geometry?.weight || 3.5;

  // Motors
  document.getElementById("cfg-driver-type").value = spec.motors?.driver_type || "GENERIC_2_IN";
  document.getElementById("cfg-max-rpm").value = spec.motors?.max_rpm || 330;
  document.getElementById("cfg-cpr").value = spec.motors?.cpr || 1440;
  document.getElementById("cfg-motor-voltage").value = spec.motors?.operating_voltage || 12.0;
  if (document.getElementById("cfg-motor-torque")) document.getElementById("cfg-motor-torque").value = spec.motors?.rated_torque || 1.5;

  document.getElementById("cfg-m1-inv").checked = !!spec.motors?.motor1_inv;
  document.getElementById("cfg-m2-inv").checked = !!spec.motors?.motor2_inv;
  document.getElementById("cfg-m3-inv").checked = !!spec.motors?.motor3_inv;
  document.getElementById("cfg-m4-inv").checked = !!spec.motors?.motor4_inv;

  // Sensors
  document.getElementById("cfg-imu").value = spec.sensors?.imu || "NONE";
  document.getElementById("cfg-mag").value = spec.sensors?.mag || "NONE";
  document.getElementById("cfg-battery").value = spec.sensors?.battery_monitor || "NONE";
  if (document.getElementById("cfg-bat-cap")) document.getElementById("cfg-bat-cap").value = spec.sensors?.battery_capacity || 2.2;
  if (document.getElementById("cfg-bat-nom")) document.getElementById("cfg-bat-nom").value = spec.sensors?.battery_nominal_voltage || 11.1;
  if (document.getElementById("cfg-bat-min")) document.getElementById("cfg-bat-min").value = spec.sensors?.battery_min_voltage || 9.0;
  if (document.getElementById("cfg-bat-max")) document.getElementById("cfg-bat-max").value = spec.sensors?.battery_max_voltage || 12.6;
  if (document.getElementById("cfg-bat-dip")) document.getElementById("cfg-bat-dip").value = spec.sensors?.battery_dip || 0.98;
  if (document.getElementById("cfg-bat-r1")) document.getElementById("cfg-bat-r1").value = spec.sensors?.battery_r1 || 30000;
  if (document.getElementById("cfg-bat-r2")) document.getElementById("cfg-bat-r2").value = spec.sensors?.battery_r2 || 7500;
  if (document.getElementById("cfg-bat-cap-val")) document.getElementById("cfg-bat-cap-val").value = spec.sensors?.battery_adc_cap || 1000;
  document.getElementById("cfg-sonar").value = spec.sensors?.sonar ? "true" : "false";

  // Dual-Core
  if (document.getElementById("cfg-dual-core")) {
    document.getElementById("cfg-dual-core").checked = spec.dual_core !== false;
  }

  // Hardware Watchdog
  if (document.getElementById("cfg-wdt-enable")) {
    const wdtEnabled = !!spec.watchdog?.enabled || (typeof spec.watchdog === "number" && spec.watchdog > 0);
    document.getElementById("cfg-wdt-enable").checked = wdtEnabled;
    const wdtTimeoutGroup = document.getElementById("group-wdt-timeout");
    if (wdtTimeoutGroup) wdtTimeoutGroup.style.display = wdtEnabled ? "flex" : "none";
    if (document.getElementById("cfg-wdt-timeout")) {
      const to = spec.watchdog?.timeout_sec || (typeof spec.watchdog === "number" ? spec.watchdog : 60);
      document.getElementById("cfg-wdt-timeout").value = to;
    }
  }

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

  const robotName = spec.robot_name || "my_robot";
  const gitBranchInput = document.getElementById("auto-git-branch");
  if (gitBranchInput && gitBranchInput.dataset.autoManaged !== "false") {
    gitBranchInput.value = `config/${robotName}`;
  }
  const gitCommitInput = document.getElementById("auto-git-commit-msg");
  if (gitCommitInput && gitCommitInput.dataset.autoManaged !== "false") {
    gitCommitInput.value = `feat(config): add configuration for ${robotName}`;
  }
  const portInput = document.getElementById("auto-flash-port");
  if (portInput && portInput.dataset.autoManaged !== "false") {
    const isEsp = (spec.mcu || "").toUpperCase().startsWith("ESP");
    portInput.value = isEsp ? "/dev/ttyUSB0" : "/dev/ttyACM0";
  }

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
    serial_interface: document.getElementById("cfg-serial-interface") ? document.getElementById("cfg-serial-interface").value : "CDC",
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
      battery_capacity: getFloat("cfg-bat-cap", 2.2),
      battery_nominal_voltage: getFloat("cfg-bat-nom", 11.1),
      battery_min_voltage: getFloat("cfg-bat-min", 9.0),
      battery_max_voltage: getFloat("cfg-bat-max", 12.6),
      battery_dip: getFloat("cfg-bat-dip", 0.98),
      battery_r1: getFloat("cfg-bat-r1", 30000.0),
      battery_r2: getFloat("cfg-bat-r2", 7500.0),
      battery_adc_cap: getFloat("cfg-bat-cap-val", 1000.0),
      sonar: document.getElementById("cfg-sonar").value === "true"
    },
    baudrate: document.getElementById("cfg-baudrate") ? parseInt(document.getElementById("cfg-baudrate").value, 10) : 921600,
    telemetry: {
      use_dual_core: document.getElementById("cfg-dual-core") ? document.getElementById("cfg-dual-core").checked : true,
      baudrate: document.getElementById("cfg-baudrate") ? parseInt(document.getElementById("cfg-baudrate").value, 10) : 921600
    },
    dual_core: document.getElementById("cfg-dual-core") ? document.getElementById("cfg-dual-core").checked : true,
    watchdog: {
      enabled: document.getElementById("cfg-wdt-enable") ? document.getElementById("cfg-wdt-enable").checked : false,
      timeout_sec: getInt("cfg-wdt-timeout", 60)
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

  // WiFi & Serial Baudrate Settings
  const mcuVal = document.getElementById("cfg-mcu") ? document.getElementById("cfg-mcu").value : "";
  const isWifiMcu = mcuVal.includes("W") || mcuVal.includes("ESP32") || mcuVal === "GENDRV";
  const wifiTelemetry = document.getElementById("cfg-wifi-telemetry") ? document.getElementById("cfg-wifi-telemetry").checked : false;
  document.getElementById("wifi-config-box").style.display = (transport === "WIFI_UDP" || (isWifiMcu && wifiTelemetry)) ? "block" : "none";
  if (document.getElementById("group-serial-baudrate")) {
    document.getElementById("group-serial-baudrate").style.display = transport === "SERIAL" ? "block" : "none";
  }

  // Watchdog Timeout Group
  const wdtEnable = document.getElementById("cfg-wdt-enable") ? document.getElementById("cfg-wdt-enable").checked : false;
  const wdtTimeoutGroup = document.getElementById("group-wdt-timeout");
  if (wdtTimeoutGroup) wdtTimeoutGroup.style.display = wdtEnable ? "flex" : "none";

  // Serial Interface for S2/S3
  const mcu = document.getElementById("cfg-mcu").value;
  const isS2S3 = (mcu === "ESP32S2" || mcu === "ESP32S3") && transport === "SERIAL";
  const ifaceGroup = document.getElementById("group-serial-interface");
  if (ifaceGroup) ifaceGroup.style.display = isS2S3 ? "flex" : "none";

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

  // 4. Update ADC Voltage Calculations & CAD Vector Schematic
  updateAdcVoltageCalculations(currentSpec);

  // 5. Render Active Code Viewer
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
    } else if (["PICO", "PICO2", "PICOW", "PICO2W"].includes(mcu)) {
      if (p < 0 || p > 29) {
        errors.push({ level: "ERROR", field: name, message: `GP${p} is out of range for RP2040/RP2350 (0-29).` });
      }
      if (["PICOW", "PICO2W"].includes(mcu) && [23, 24, 25, 29].includes(p)) {
        errors.push({ level: "WARNING", field: name, message: `GP${p} is connected to the CYW43439 WiFi chip on Pico W / Pico 2 W.` });
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

  // Check Battery Voltage & ADC Resistor Divider Safety (Limit <= 3.0V)
  if (sensors.battery_monitor === "ADC_DIVIDER") {
    const r1 = parseFloat(sensors.battery_r1 || 30000);
    const r2 = parseFloat(sensors.battery_r2 || 7500);
    const maxV = parseFloat(sensors.battery_max_voltage || 12.6);
    if (r1 > 0 && r2 > 0 && maxV > 0) {
      const totalR = r1 + r2;
      const ratio = r2 / totalR;
      const adcMaxV = maxV * ratio;
      stats.adc_voltage_max_v = adcMaxV;
      stats.adc_divider_ratio = ratio;
      if (adcMaxV > 3.0) {
        errors.push({
          level: "WARNING",
          field: "ADC Resistor Divider",
          message: `ADC pin voltage will reach ${adcMaxV.toFixed(2)}V at full charge (${maxV.toFixed(1)}V), exceeding the 3.0V safe limit! Increase R1 or reduce R2.`
        });
      }
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

// Update ADC Calculations & CAD Vector Circuit Schematic
function updateAdcVoltageCalculations(spec) {
  const s = spec || currentSpec;
  const sensors = s.sensors || {};
  const r1 = parseFloat(sensors.battery_r1 !== undefined ? sensors.battery_r1 : (document.getElementById("cfg-bat-r1")?.value || 30000));
  const r2 = parseFloat(sensors.battery_r2 !== undefined ? sensors.battery_r2 : (document.getElementById("cfg-bat-r2")?.value || 7500));
  const cap = parseFloat(sensors.battery_adc_cap !== undefined ? sensors.battery_adc_cap : (document.getElementById("cfg-bat-cap-val")?.value || 1000));
  const maxV = parseFloat(sensors.battery_max_voltage !== undefined ? sensors.battery_max_voltage : (document.getElementById("cfg-bat-max")?.value || 12.6));
  const nomV = parseFloat(sensors.battery_nominal_voltage !== undefined ? sensors.battery_nominal_voltage : (document.getElementById("cfg-bat-nom")?.value || 11.1));
  const batPin = s.pins?.battery_pin !== undefined ? s.pins.battery_pin : (document.getElementById("pin-battery")?.value || 33);
  const mcu = s.mcu || "ESP32";

  const totalR = r1 + r2;
  const ratio = totalR > 0 ? (r2 / totalR) : 0.2;
  const adcMaxV = maxV * ratio;
  const adcNomV = nomV * ratio;
  const isSafe = adcMaxV <= 3.0;

  // 1. Update Tab 3 Battery Safety Banner
  const safetyBanner = document.getElementById("bat-adc-safety-banner");
  const iconEl = document.getElementById("safety-banner-icon");
  const titleEl = document.getElementById("safety-banner-title");
  const descEl = document.getElementById("safety-banner-desc");
  const ratioLbl = document.getElementById("lbl-divider-ratio");
  const maxVLbl = document.getElementById("lbl-adc-max-v");

  if (safetyBanner) {
    safetyBanner.className = `adc-safety-banner ${isSafe ? 'safe' : 'warning'}`;
    if (iconEl) iconEl.innerText = isSafe ? "🛡️" : "⚠️";
    if (titleEl) titleEl.innerText = isSafe ? "ADC Input Voltage: Safe (≤ 3.0V)" : "HIGH VOLTAGE WARNING: Exceeds 3.0V Maximum ADC Limit!";
    if (ratioLbl) ratioLbl.innerText = `${ratio.toFixed(4)} (1:${(1 / ratio).toFixed(1)})`;
    if (maxVLbl) maxVLbl.innerText = `${adcMaxV.toFixed(2)} V`;
    if (descEl) {
      if (isSafe) {
        descEl.innerHTML = `Divider Ratio: <code>${ratio.toFixed(4)} (1:${(1 / ratio).toFixed(1)})</code> | Max ADC Pin Voltage: <code>${adcMaxV.toFixed(2)} V</code> at full charge (${maxV.toFixed(1)}V). Nominal: <code>${adcNomV.toFixed(2)} V</code>. Filter Cap: <code>${cap} pF</code>.`;
      } else {
        descEl.innerHTML = `⚠️ ADC input voltage reaches <code>${adcMaxV.toFixed(2)} V</code> at full charge (${maxV.toFixed(1)}V), exceeding the 3.0V safe linear limit! Risk of saturation or non-linearity. Increase R1 or decrease R2.`;
      }
    }
  }

  // 2. Update CAD Vector Schematic (SVG)
  const svgR1 = document.getElementById("cad-svg-r1");
  if (svgR1) svgR1.textContent = `R1: ${(r1 / 1000).toFixed(1)} kΩ`;

  const svgR2 = document.getElementById("cad-svg-r2-val");
  if (svgR2) svgR2.textContent = `${(r2 / 1000).toFixed(1)} kΩ`;

  const svgCap = document.getElementById("cad-svg-cap");
  if (svgCap) svgCap.textContent = `C: ${cap} pF (${(cap / 1000).toFixed(1)}nF)`;

  const svgVbat = document.getElementById("cad-svg-vbat");
  if (svgVbat) svgVbat.textContent = `${maxV.toFixed(1)} V (Full)`;

  const svgAdcPin = document.getElementById("cad-svg-adc-pin");
  if (svgAdcPin) svgAdcPin.textContent = `${isESP32(mcu) ? 'ESP32 ADC' : 'MCU ADC'} (GPIO ${batPin || 33})`;

  const svgVadcRead = document.getElementById("cad-svg-vadc-read");
  if (svgVadcRead) svgVadcRead.textContent = `V_ADC = ${adcMaxV.toFixed(2)} V (Max)`;

  const cadBadgeRatio = document.getElementById("cad-badge-ratio");
  if (cadBadgeRatio) cadBadgeRatio.textContent = `Ratio: ${ratio.toFixed(4)} (1:${(1 / ratio).toFixed(1)})`;

  const cadBadgeSafety = document.getElementById("cad-badge-safety");
  if (cadBadgeSafety) {
    cadBadgeSafety.className = `schematic-badge ${isSafe ? 'badge-safe' : 'badge-warning'}`;
    cadBadgeSafety.textContent = isSafe ? `🛡️ V_ADC: ${adcMaxV.toFixed(2)}V ≤ 3.0V` : `⚠️ V_ADC: ${adcMaxV.toFixed(2)}V > 3.0V (Exceeds Limit!)`;
  }

  const cadWarnBox = document.getElementById("cad-svg-warning-box");
  if (cadWarnBox) {
    cadWarnBox.style.display = isSafe ? "none" : "inline";
    const cadWarnText = document.getElementById("cad-svg-warning-text");
    if (cadWarnText) cadWarnText.textContent = `Max ${adcMaxV.toFixed(2)}V exceeds 3.0V limit`;
  }
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

  if (document.getElementById("val-max-accel") && stats.max_accel_m_s2 !== undefined) {
    document.getElementById("val-max-accel").innerText = stats.max_accel_m_s2.toFixed(2);
    document.getElementById("val-thrust-sub").innerText = `Thrust: ${(stats.total_thrust_n || 0).toFixed(2)} N @ ${(stats.weight_kg || 3.5).toFixed(1)} kg`;
  }
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
    `#define LED_PIN ${pins.led !== undefined ? pins.led : (["PICOW", "PICO2W"].includes(mcu) ? "LED_BUILTIN" : (mcu.includes("PICO") ? 25 : 2))}`
  );

  const numActive = is4WD ? 4 : 2;
  for (let i = 1; i <= 4; i++) {
    if (i <= numActive) {
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
    } else {
      if (driver === "BTS7960") {
        lines.push(`#define MOTOR${i}_PWM_R -1`);
        lines.push(`#define MOTOR${i}_PWM_L -1`);
      } else if (driver === "GENERIC_2_IN") {
        lines.push(`#define MOTOR${i}_PWM -1`);
        lines.push(`#define MOTOR${i}_IN_A -1`);
        lines.push(`#define MOTOR${i}_IN_B -1`);
      } else if (driver === "GENERIC_1_IN") {
        lines.push(`#define MOTOR${i}_PWM -1`);
        lines.push(`#define MOTOR${i}_DIR -1`);
      } else {
        lines.push(`#define MOTOR${i}_PWM -1`);
      }
    }
  }

  const enc = pins.encoders || {};
  for (let i = 1; i <= 4; i++) {
    if (i <= numActive) {
      lines.push(`#define MOTOR${i}_ENCODER_A ${enc[`m${i}_a`] !== undefined ? enc[`m${i}_a`] : 0}`);
      lines.push(`#define MOTOR${i}_ENCODER_B ${enc[`m${i}_b`] !== undefined ? enc[`m${i}_b`] : 0}`);
    } else {
      lines.push(`#define MOTOR${i}_ENCODER_A -1`);
      lines.push(`#define MOTOR${i}_ENCODER_B -1`);
    }
  }

  lines.push(``, `// Sensor Configurations`);
  if (sensors.imu === "FAKE" || sensors.imu === "FAKE_IMU") {
    lines.push(`#define USE_FAKE_IMU`);
  } else if (sensors.imu && sensors.imu !== "NONE") {
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
      `#define BATTERY_R2 7500.0`
    );
    if (spec.adc_lut || sensors.adc_lut) {
      const lutData = spec.adc_lut || sensors.adc_lut;
      lines.push(
        `#define USE_ADC_LUT`,
        formatAdcLutArray(lutData),
        `#define BATTERY_ADJUST(v) (ADC_LUT[v] * (3.3 / 4096.0 * (30000.0 + 7500.0) / 7500.0))`
      );
    } else {
      lines.push(
        `#define BATTERY_ADJUST(v) (v * (3.3 / 4095.0) * ((30000.0 + 7500.0) / 7500.0))`
      );
    }
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

  // Communication & Network Settings
  const transport = (spec.transport || "SERIAL").toUpperCase();
  const wifi = spec.wifi_settings || {};
  const isWifiTransport = transport.includes("WIFI") || transport.includes("UDP");
  const enableWifiNet = isWifiTransport || !!spec.enable_ota_syslog;

  if (!isWifiTransport) {
    const baudrate = spec.baudrate || spec.telemetry?.baudrate || (mcu === "GENDRV" ? 1500000 : 921600);
    lines.push(
      ``,
      `// Serial Communication Settings`,
      `#define BAUDRATE ${baudrate}`
    );
  }

  // Hardware Task Watchdog (WDT)
  if (spec.watchdog && (spec.watchdog.enabled || spec.watchdog > 0)) {
    const to = spec.watchdog.timeout_sec || (typeof spec.watchdog === "number" ? spec.watchdog : 60);
    lines.push(``, `// Hardware Task Watchdog`, `#define WDT_TIMEOUT ${to} // Seconds`);
  } else {
    lines.push(``, `// #define WDT_TIMEOUT 60 // Hardware Task Watchdog disabled`);
  }


  if (isWifiTransport) {
    const agentIpStr = wifi.agent_ip || "192.168.1.100";
    const ipParts = agentIpStr.split(".").map(p => p.trim());
    const ipFormatted = ipParts.length === 4 ? `{ ${ipParts.join(", ")} }` : "{ 192, 168, 1, 100 }";
    const ssid = wifi.ssid || "YOUR_WIFI_SSID";
    const password = wifi.password || "YOUR_WIFI_PASSWORD";
    const agentPort = wifi.agent_port || 8888;

    lines.push(
      ``,
      `// WiFi & micro-ROS Agent Settings`,
      `#define WIFI_SSID "${ssid}"`,
      `#define WIFI_PASSWORD "${password}"`,
      `#define AGENT_IP ${ipFormatted}`,
      `#define AGENT_PORT ${agentPort}`,
      `#define USE_WIFI`,
      `#define WIFI_AP_LIST { { "${ssid}", "${password}" }, { NULL, NULL } }`,
      `#define USE_ARDUINO_OTA`,
      `#define USE_SYSLOG`,
      `#define SYSLOG_SERVER "${agentIpStr}"`,
      `#define SYSLOG_PORT 514`,
      `#define DEVICE_HOSTNAME "${spec.robot_name || "robot"}"`,
      `#define APP_NAME "hardware"`
    );
  } else if (enableWifiNet) {
    const ssid = wifi.ssid || "YOUR_WIFI_SSID";
    const password = wifi.password || "YOUR_WIFI_PASSWORD";
    const agentIpStr = wifi.agent_ip || "192.168.1.100";
    lines.push(
      ``,
      `// Background WiFi (OTA & Syslog telemetry while micro-ROS runs on Serial)`,
      `#define WIFI_SSID "${ssid}"`,
      `#define WIFI_PASSWORD "${password}"`,
      `#define WIFI_AP_LIST { { "${ssid}", "${password}" }, { NULL, NULL } }`,
      `#define USE_ARDUINO_OTA`,
      `#define USE_SYSLOG`,
      `#define SYSLOG_SERVER "${agentIpStr}"`,
      `#define SYSLOG_PORT 514`,
      `#define DEVICE_HOSTNAME "${spec.robot_name || "robot"}"`,
      `#define APP_NAME "hardware"`
    );
  }

  if (spec.dual_core !== false) {
    lines.push(
      ``,
      `// Dual-Core Task Allocation (Strict 50Hz Real-Time PID on Core 1)`,
      `#define USE_DUAL_CORE`
    );
  }

  lines.push(``, `#endif // ${nameUpper}_CONFIG_H`, ``);
  return lines.join("\n");
}

function generatePlatformioEnv(spec) {
  const name = spec.robot_name || "my_robot";
  const mcu = (spec.mcu || "PICO2").toUpperCase();
  const cfgMacro = `USE_${name.toUpperCase()}_CONFIG`;
  const isWifi = spec.transport === "WIFI" || spec.transport === "UDP";
  const wifiTransportLine = isWifi ? "board_microros_transport = wifi\n" : "";
  const wifiFlag = isWifi ? "\n    -D USE_STAY_CONNECTED" : "";

  if (mcu === "PICO" || mcu === "PICO2" || mcu === "PICOW" || mcu === "PICO2W") {
    const boardMap = { PICO: "rpipico", PICO2: "rpipico2", PICOW: "rpipicow", PICO2W: "rpipico2w" };
    const board = boardMap[mcu] || "rpipico";
    const picoMacro = (mcu === "PICOW" || mcu === "PICO2W") ? `    -D ${mcu}\n` : "";
    return `[env:${name}]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = ${board}
monitor_port = /dev/ttyACM0
upload_port = /dev/ttyACM0
upload_protocol = picotool
board_microros_user_meta = atomic.meta
${wifiTransportLine}lib_deps =
    \${env.lib_deps}
    https://github.com/gbr1/rp2040-encoder-library.git
build_flags =
    -I ../config
    -D PICO
${picoMacro}    -D ${cfgMacro}${wifiFlag}
`;
  } else if (mcu === "ESP32" || mcu === "GENDRV") {
    return `[env:${name}]
platform = espressif32
board = nodemcu-32s
board_build.f_flash = 80000000L
board_build.flash_mode = qio
board_build.partitions = min_spiffs.csv
monitor_speed = 921600
monitor_port = /dev/ttyUSB0
upload_port = /dev/ttyUSB0
upload_protocol = esptool
${wifiTransportLine}lib_deps =
    \${env.lib_deps}
    madhephaestus/ESP32Servo
    madhephaestus/ESP32Encoder
build_flags =
    -I ../config
    -D __PGMSPACE_H_
    -D ${cfgMacro}${wifiFlag}
`;
  } else if (mcu === "ESP32S3") {
    const isBridge = (spec.serial_interface || "CDC") === "BRIDGE";
    const port = isBridge ? "/dev/ttyUSB0" : "/dev/ttyACM0";
    const cdcFlag = isBridge ? "" : "\n    -D ARDUINO_USB_CDC_ON_BOOT";
    return `[env:${name}]
platform = espressif32
board = esp32-s3-devkitc-1
board_build.f_flash = 80000000L
board_build.flash_mode = qio
monitor_speed = 921600
monitor_port = ${port}
upload_port = ${port}
upload_protocol = esptool
${wifiTransportLine}lib_deps =
    \${env.lib_deps}
    madhephaestus/ESP32Servo
    madhephaestus/ESP32Encoder
build_flags =
    -I ../config${cdcFlag}
    -D __PGMSPACE_H_
    -D ${cfgMacro}${wifiFlag}
`;
  } else if (mcu === "ESP32S2") {
    const isBridge = (spec.serial_interface || "CDC") === "BRIDGE";
    const port = isBridge ? "/dev/ttyUSB0" : "/dev/ttyACM0";
    const cdcFlag = isBridge ? "" : "\n    -D ARDUINO_USB_CDC_ON_BOOT";
    return `[env:${name}]
platform = espressif32
board = esp32-s2-saola-1
monitor_speed = 921600
monitor_port = ${port}
upload_port = ${port}
upload_protocol = esptool
${wifiTransportLine}lib_deps =
    \${env.lib_deps}
    madhephaestus/ESP32Servo
    madhephaestus/ESP32Encoder
build_flags =
    -I ../config${cdcFlag}
    -D __PGMSPACE_H_
    -D ${cfgMacro}${wifiFlag}
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

  if (mcu === "PICO" || mcu === "PICO2" || mcu === "PICOW" || mcu === "PICO2W") {
    document.getElementById("pin-led").value = (mcu === "PICOW" || mcu === "PICO2W") ? 32 : 25;
    document.getElementById("pin-i2c-sda").value = 0;
    document.getElementById("pin-i2c-scl").value = 1;
    document.getElementById("pin-battery").value = 26;
    document.getElementById("pin-sonar-trig").value = 22;
    document.getElementById("pin-sonar-echo").value = 27;

    document.getElementById("pin-m1-p1").value = 10;
    document.getElementById("pin-m1-p2").value = 11;
    document.getElementById("pin-m1-p3").value = 12;

    document.getElementById("pin-m2-p1").value = 13;
    document.getElementById("pin-m2-p2").value = 14;
    document.getElementById("pin-m2-p3").value = 15;

    document.getElementById("pin-enc-1a").value = 2;
    document.getElementById("pin-enc-1b").value = 3;
    document.getElementById("pin-enc-2a").value = 4;
    document.getElementById("pin-enc-2b").value = 5;

    if (is4WD) {
      document.getElementById("pin-m3-p1").value = 16;
      document.getElementById("pin-m3-p2").value = 17;
      document.getElementById("pin-m3-p3").value = 18;
      document.getElementById("pin-m4-p1").value = 19;
      document.getElementById("pin-m4-p2").value = 20;
      document.getElementById("pin-m4-p3").value = 21;
      document.getElementById("pin-enc-3a").value = 6;
      document.getElementById("pin-enc-3b").value = 7;
      document.getElementById("pin-enc-4a").value = 8;
      document.getElementById("pin-enc-4b").value = 9;
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

// Import Spec / Header File Handler (.h, .hpp, .json)
async function handleImportFile(e) {
  const file = e.target.files[0];
  if (!file) return;

  const isJson = file.name.endsWith('.json');
  const reader = new FileReader();
  reader.onload = async (event) => {
    try {
      const content = event.target.result;
      if (isJson) {
        const imported = JSON.parse(content);
        baseConfigSpec = JSON.parse(JSON.stringify(imported));
        currentSpec = imported;
        populateFormFromSpec(currentSpec);
        recomputeAll();
        showToast(`✅ Successfully imported '${imported.robot_name || file.name}'!`);
      } else {
        const res = await fetch('/api/parse', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ content: content, is_json: false })
        });
        const data = await res.json();
        if (data.status === 'ok' && data.spec) {
          baseConfigSpec = JSON.parse(JSON.stringify(data.spec));
          currentSpec = data.spec;
          populateFormFromSpec(currentSpec);
          recomputeAll();
          showToast(`✅ Imported and parsed C++ header '${file.name}'!`);
        } else {
          throw new Error(data.error || 'Failed to parse header');
        }
      }
    } catch (err) {
      alert('Error importing configuration: ' + err.message);
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

// Robot Host SBC Environment Auto-Detection (Execution Destination)
function syncRobotHostInfo(data) {
  if (!data) return;

  const node = data.node || "Robot Computer";
  const arch = data.machine || "aarch64";
  const distroId = (data.distro_id || "").toLowerCase();
  const distroName = data.distro_name || data.os || "Linux";
  const ports = data.serial_ports || [];
  const rosDistros = data.installed_ros_distros || [];

  // Update Header Badges
  const headerIcon = document.getElementById("header-os-icon");
  const headerText = document.getElementById("header-os-text");
  if (headerIcon) headerIcon.innerText = "🤖";
  if (headerText) headerText.innerText = `Robot Host: ${node} (${arch})`;

  // Update Banner in Tab 5
  const bannerIcon = document.getElementById("os-banner-icon");
  const bannerText = document.getElementById("os-banner-detected-text");
  const bannerArch = document.getElementById("os-banner-arch");
  const bannerPorts = document.getElementById("os-banner-ports");
  const bannerStatus = document.getElementById("os-banner-status");

  if (bannerIcon) bannerIcon.innerText = "🤖";
  if (bannerText) bannerText.innerText = `Target Robot Computer: ${node} (${distroName})`;
  if (bannerArch) bannerArch.innerText = arch;
  if (bannerPorts) {
    bannerPorts.innerText = ports.length > 0 ? ports.join(", ") : "No USB serial MCUs detected (plug in Pico / ESP32)";
    bannerPorts.style.color = ports.length > 0 ? "var(--success)" : "var(--text-dim)";
  }
  if (bannerStatus) {
    bannerStatus.innerText = data.has_pio ? "PlatformIO Ready" : (isRunnerOnline ? "Runner Online" : "Disconnected");
    bannerStatus.className = `badge-pill ${data.has_pio || isRunnerOnline ? "badge-ok" : "badge-warn"}`;
  }

  // Set Default Dropdowns if not manually touched
  const osSelect = document.getElementById("auto-os-select");
  if (osSelect && !osSelect.dataset.userModified) {
    if (distroId.includes("ubuntu")) {
      if (data.release && data.release.includes("26.04")) osSelect.value = "ubuntu_2604";
      else if (data.release && data.release.includes("22.04")) osSelect.value = "ubuntu_2204";
      else osSelect.value = "ubuntu_2404";
    } else if (distroId.includes("debian")) {
      osSelect.value = "debian";
    } else if (distroId.includes("fedora") || distroId.includes("bluefin")) {
      osSelect.value = "fedora";
    }
  }

  const archSelect = document.getElementById("auto-arch-select");
  if (archSelect && !archSelect.dataset.userModified) {
    archSelect.value = arch.includes("arm") || arch.includes("aarch64") ? "aarch64" : "x86_64";
  }

  const rosSelect = document.getElementById("auto-ros-distro");
  if (rosSelect && !rosSelect.dataset.userModified && rosDistros.length > 0) {
    if (rosDistros.includes("jazzy")) rosSelect.value = "jazzy";
    else if (rosDistros.includes("humble")) rosSelect.value = "humble";
    else if (rosDistros.includes("lyrical")) rosSelect.value = "lyrical";
    else if (rosDistros.includes("rolling")) rosSelect.value = "rolling";
  }

  // Auto-populate detected serial port into flash / agent controls with USB VID:PID & chip info
  const chipsContainer = document.getElementById("flash-port-chips");
  if (chipsContainer) {
    const portDetails = data.port_details || [];
    if (ports.length > 0) {
      chipsContainer.innerHTML = `<span class="chips-label">Detected USB Devices:</span> ` + ports.map(p => {
        const detail = portDetails.find(d => d.port === p) || {};
        const isBootsel = !!detail.is_bootsel;
        const isOk = detail.accessible !== false;
        const vidPid = detail.vid && detail.pid ? `[${detail.vid}:${detail.pid}]` : "";
        const chip = detail.chip || (detail.product ? detail.product : (isBootsel ? "RP2 Bootloader" : "USB Serial"));
        const icon = isBootsel ? "⚡📦" : (isOk ? "⚡" : "⚠️");
        const badgeClass = isBootsel ? "port-chip port-chip-bootsel" : (isOk ? "port-chip" : "port-chip port-chip-warn");
        const title = `${p} ${vidPid} - ${chip} | Read/Write: ${isOk ? "Granted" : "Denied (dialout required)"}`;
        return `<button type="button" class="${badgeClass}" data-port="${p}" title="${title}">
          <span class="chip-icon">${icon}</span>
          <span class="chip-port">${p}</span>
          ${vidPid ? `<span class="chip-vidpid">${vidPid}</span>` : ""}
          <span class="chip-desc">${chip}</span>
          ${!isOk ? `<span class="chip-lock">[No RW Access]</span>` : ""}
        </button>`;
      }).join(" ");

      chipsContainer.querySelectorAll(".port-chip").forEach(btn => {
        btn.addEventListener("click", () => {
          const p = btn.dataset.port;
          const flashPortInput = document.getElementById("auto-flash-port");
          if (flashPortInput) {
            flashPortInput.value = p;
            flashPortInput.dataset.autoManaged = "false";
          }
          const detail = (data.port_details || []).find(d => d.port === p);
          const guessed = guessMcuFromUsbDetail(detail);
          if (guessed) {
            const mcuSelect = document.getElementById("cfg-mcu");
            if (mcuSelect && mcuSelect.value !== guessed.mcu) {
              mcuSelect.value = guessed.mcu;
              mcuSelect.dispatchEvent(new Event("change"));
            }
            showToast(`⚡ Selected ${p} (${guessed.chipName}) — Target MCU: ${guessed.mcu}`);
          } else {
            showToast(`🔌 Selected upload port: ${p}`);
          }
          updateAutomationPreviews();
        });
      });
    } else {
      chipsContainer.innerHTML = `<span class="chips-empty">No USB ports detected. You can type any serial path above.</span>`;
    }
  }

  if (ports.length > 0) {
    const flashPortInput = document.getElementById("auto-flash-port");
    if (flashPortInput && (!flashPortInput.value || flashPortInput.value === "/dev/ttyACM0" || flashPortInput.dataset.autoManaged === "true")) {
      flashPortInput.value = ports[0];
    }
    const multiInput = document.getElementById("auto-agent-multiserial-ports");
    if (multiInput && (!multiInput.value || multiInput.dataset.autoManaged === "true")) {
      multiInput.value = ports.join(" ");
    }

    // Hardware USB Auto-Detection & Bare Module Preset Adaptation
    if (!window.hasAutoDetectedHardware && data.port_details && data.port_details.length > 0) {
      window.hasAutoDetectedHardware = true;
      const primaryDetail = data.port_details[0];
      const guessed = guessMcuFromUsbDetail(primaryDetail);
      if (guessed) {
        const mcuSelect = document.getElementById("cfg-mcu");
        if (mcuSelect) {
          mcuSelect.value = guessed.mcu;
          mcuSelect.dispatchEvent(new Event("change"));
        }
        // Force Bare Module Sensor Defaults (Zero-Hang on Bare Board)
        const imuSelect = document.getElementById("cfg-imu");
        if (imuSelect) imuSelect.value = "FAKE";
        const magSelect = document.getElementById("cfg-mag");
        if (magSelect) magSelect.value = "NONE";
        const batSelect = document.getElementById("cfg-battery");
        if (batSelect) batSelect.value = "NONE";
        const sonarSelect = document.getElementById("cfg-sonar");
        if (sonarSelect) sonarSelect.value = "false";
        
        showToast(`⚡ Auto-detected Hardware: ${guessed.chipName} on ${ports[0]} (Bare Module Default Loaded)`);
      }
    }
  }

  updateAutomationPreviews();
}

function detectClientOS() {
  checkServerRunnerStatus();
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

  // 4. micro-ROS Agent Preview
  const agentCmdEl = document.getElementById("preview-agent-cmd");
  if (agentCmdEl) {
    const distro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
    const port = document.getElementById("auto-flash-port")?.value || (isESP32(spec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0");
    const multiPorts = document.getElementById("auto-agent-multiserial-ports")?.value.trim() || "/dev/ttyACM0 /dev/ttyUSB0";
    const baud = document.getElementById("auto-agent-baud")?.value || "921600";
    const isWiFi = spec.transport === "WIFI" || spec.transport === "UDP";
    if (isWiFi) {
      agentCmdEl.innerText = `source /opt/ros/${distro}/setup.bash && [ -f "$HOME/uros_ws/install/setup.bash" ] && source "$HOME/uros_ws/install/setup.bash"; ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`;
    } else {
      agentCmdEl.innerText = `# Single MCU:\nros2 run micro_ros_agent micro_ros_agent serial --dev ${port} -b ${baud}\n# Multi-MCU:\nros2 run micro_ros_agent micro_ros_agent multiserial --devs "${multiPorts}" -b ${baud}`;
    }
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
    lines.push("sudo dpkg --configure -a 2>/dev/null || true");
    if (opts.installBuildTools) {
      lines.push("sudo apt-get update && sudo apt-get install -y git cmake ninja-build python3-pip python3-venv udev");
    }
    if (opts.installPio) {
      lines.push("curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py && python3 /tmp/get-platformio.py && export PATH=\"$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH\"");
    }
    if (opts.installUdev) {
      lines.push("curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules > /dev/null");
      lines.push("sudo udevadm control --reload-rules && sudo udevadm trigger");
    }
    if (opts.installDialout) {
      lines.push("sudo usermod -a -G dialout,plugdev $USER || true");
    }
  } else if (isMac) {
    if (opts.installBuildTools) lines.push("brew install git cmake ninja python3");
    if (opts.installPio) lines.push("brew install platformio || pip3 install platformio");
  } else if (isFedora) {
    if (opts.installBuildTools) lines.push("sudo dnf install -y git cmake ninja-build python3-pip systemd-udev || true");
    if (opts.installPio) lines.push("curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py && python3 /tmp/get-platformio.py && export PATH=\"$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH\"");
    if (opts.installUdev) {
      lines.push("curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules > /dev/null || true");
      lines.push("sudo udevadm control --reload-rules && sudo udevadm trigger || true");
    }
    if (opts.installDialout) lines.push("sudo usermod -a -G dialout $USER 2>/dev/null || true");
  }

  return lines.length > 0 ? lines.join(" && \\\n") : "# No toolchain installations selected";
}

// Generate ROS 2 Install Commands
function getRos2InstallCmd(opts) {
  const distro = opts.rosDistro;
  if (distro === "none") return "# ROS 2 Installation skipped (Standalone Firmware mode)";

  const pkgName = opts.rosType === "desktop" ? `ros-${distro}-desktop` : `ros-${distro}-ros-base`;
  const lines = [
    `set -e`,
    `sudo dpkg --configure -a 2>/dev/null || true`,
    ``,
    `# 1. Setup ROS 2 ${distro.toUpperCase()} Official Repository`,
    `if command -v apt-get &>/dev/null; then`,
    `    sudo apt-get update -qq && sudo apt-get install -y -qq software-properties-common curl gnupg`,
    `    sudo add-apt-repository -y universe`,
    `    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`,
    `    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`,
    `    sudo apt-get update -qq && sudo apt-get install -y -qq ${pkgName} ros-${distro}-ros-workspace python3-catkin-pkg-modules python3-colcon-common-extensions python3-rosdep`,
    `elif command -v distrobox &>/dev/null; then`,
    `    distrobox create -n ${distro} -i docker.io/library/ubuntu:24.04 2>/dev/null || true`,
    `    distrobox enter ${distro} -- bash -c "sudo apt-get update && sudo apt-get install -y ${pkgName} ros-${distro}-ros-workspace python3-catkin-pkg-modules python3-colcon-common-extensions python3-rosdep"`,
    `else`,
    `    echo "Package manager apt-get not found. Ensure ROS 2 ${distro} is installed on host/container."`,
    `    exit 1`,
    `fi`
  ];

  if (opts.buildMicrorosAgent) {
    lines.push(
      ``,
      `# 2. Build micro-ROS Agent Workspace (micro-ROS-Agent + micro_ros_msgs)`,
      `if [ -f "/opt/ros/${distro}/setup.bash" ]; then`,
      `    source /opt/ros/${distro}/setup.bash`,
      `    mkdir -p ~/uros_ws/src && cd ~/uros_ws`,
      `    if [ ! -d "src/micro_ros_agent" ]; then`,
      `        git clone -b ${distro} https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent`,
      `    fi`,
      `    if [ ! -d "src/micro_ros_msgs" ]; then`,
      `        git clone -b ${distro} https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs`,
      `    fi`,
      `    colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`,
      `    grep -q "source /opt/ros/${distro}/setup.bash" ~/.bashrc || echo "source /opt/ros/${distro}/setup.bash" >> ~/.bashrc`,
      `    grep -q "source \$HOME/uros_ws/install/setup.bash" ~/.bashrc || echo "source \$HOME/uros_ws/install/setup.bash" >> ~/.bashrc`,
      `else`,
      `    echo "❌ Error: ROS 2 setup not found at /opt/ros/${distro}/setup.bash"`,
      `    exit 1`,
      `fi`
    );
  }

  return lines.join("\n");
}

// Generate Merge & Commit Commands
function getMergeAndCommitCmd(spec, opts) {
  const name = spec.robot_name || "my_robot";
  const nameUpper = name.toUpperCase();
  const branch = opts.gitBranch || `config/${name}`;
  const commitMsg = opts.gitCommitMsg || `feat(config): add configuration for ${name}`;
  const headerContent = generateCppHeader(spec);
  const pioSection = generatePlatformioEnv(spec);
  const urdfContent = generateUrdfXacro(spec);

  const lines = [
    `set -e`,
    `# 1. Create Isolated Git Branch`,
    `git checkout -b "${branch}" 2>/dev/null || git checkout "${branch}"`,
    ``,
    `# 2. Ingest Custom Header`,
    `mkdir -p config/custom urdf`,
    `cat << 'EOF_HEADER' > "config/custom/${name}_config.h"`,
    `${headerContent}`,
    `EOF_HEADER`,
    ``,
    `# Register in config/config.h`,
    `if [ -f "config/config.h" ]; then`,
    `    if ! grep -F -q "USE_${nameUpper}_CONFIG" config/config.h; then`,
    `        sed -i '/\\/\\/ add user configurations above this line/i #ifdef USE_${nameUpper}_CONFIG\\n    #include "custom/${name}_config.h"\\n#endif\\n' config/config.h`,
    `    fi`,
    `fi`,
    ``,
    `# 3. Append PlatformIO Target Environment`,
    `if [ -f "firmware/platformio.ini" ]; then`,
    `    if ! grep -F -q "[env:${name}]" "firmware/platformio.ini"; then`,
    `        echo "" >> "firmware/platformio.ini"`,
    `        cat << 'EOF_PIO' >> "firmware/platformio.ini"`,
    `${pioSection}`,
    `EOF_PIO`,
    `    fi`,
    `fi`,
    ``,
    `# 4. Ingest URDF Description`,
    `cat << 'EOF_URDF' > "urdf/${name}_properties.urdf.xacro"`,
    `${urdfContent}`,
    `EOF_URDF`,
    ``,
    `# 5. Stage & Create Git Commit`,
    `git add config/ firmware/platformio.ini urdf/ 2>/dev/null || true`,
    `if ! git diff --cached --quiet; then`,
    `    git commit -m "${commitMsg}"`,
    `else`,
    `    echo "Configuration files already up to date on branch ${branch}."`,
    `fi`
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
info "Step 1/9: Checking build toolchains and dependencies..."
${opts.installBuildTools ? `if ! command -v pio &>/dev/null || ! command -v cmake &>/dev/null || ! command -v git &>/dev/null; then
    if command -v apt-get &>/dev/null; then
        info "First-time setup: installing missing build tools (apt-get update)..."
        if [ "$EUID" -eq 0 ]; then
            apt-get update -y
            info "Installing build packages: git, cmake, ninja-build, python3-pip, python3-venv, udev..."
            apt-get install -y git cmake ninja-build python3-pip python3-venv udev
        elif sudo -n true 2>/dev/null || [ -t 0 ]; then
            sudo apt-get update -y
            info "Installing build packages: git, cmake, ninja-build, python3-pip, python3-venv, udev..."
            sudo apt-get install -y git cmake ninja-build python3-pip python3-venv udev
        else
            warn "Running in non-interactive environment: attempting passwordless package install..."
            sudo -n apt-get update -y 2>/dev/null || true
            sudo -n apt-get install -y git cmake ninja-build python3-pip python3-venv udev 2>/dev/null || warn "Please ensure build dependencies are installed."
        fi
    fi
else
    info "Build toolchains already installed (pio, cmake, git ready). Skipping redundant package updates."
fi
elif command -v dnf &>/dev/null; then
    info "Installing build packages via dnf..."
    if [ "$EUID" -eq 0 ]; then
        dnf install -y git cmake ninja-build python3-pip systemd-udev || true
    else
        sudo dnf install -y git cmake ninja-build python3-pip systemd-udev || true
    fi
fi` : "# Build tools check skipped"}

${opts.installPio ? `if ! command -v pio &>/dev/null; then
    info "Installing PlatformIO Core..."
    curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py && python3 /tmp/get-platformio.py
    export PATH="$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH"
fi
success "PlatformIO Core active: $(pio --version 2>/dev/null || echo 'Installed')"
` : "# PlatformIO install skipped"}

${opts.installUdev ? `if [ -d "/etc/udev/rules.d" ]; then
    info "Installing PlatformIO hardware udev rules..."
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules > /dev/null || true
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
    if command -v apt-get &>/dev/null; then
        warn "ROS 2 ${opts.rosDistro} not found in /opt/ros/${opts.rosDistro}. Installing via apt..."
        sudo apt-get update -qq && sudo apt-get install -y -qq curl gnupg software-properties-common
        sudo add-apt-repository -y universe
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt-get update -qq
        sudo apt-get install -y -qq ${opts.rosType === "desktop" ? `ros-${opts.rosDistro}-desktop` : `ros-${opts.rosDistro}-ros-base`} ros-${opts.rosDistro}-ros-workspace python3-catkin-pkg-modules python3-colcon-common-extensions python3-rosdep
    elif command -v distrobox &>/dev/null; then
        info "Running on immutable/container host: ensuring distrobox container '${opts.rosDistro}' is configured..."
        distrobox create -n ${opts.rosDistro} -i docker.io/library/ubuntu:24.04 2>/dev/null || true
    else
        warn "Package manager 'apt-get' not found on this host. Skipping apt install."
    fi
fi

${opts.buildMicrorosAgent ? `if [ -d "/opt/ros/${opts.rosDistro}" ] && [ ! -d "$HOME/uros_ws/install" ]; then
    info "Building micro_ros_agent workspace at ~/uros_ws..."
    mkdir -p "$HOME/uros_ws/src"
    cd "$HOME/uros_ws"
    if [ ! -d "src/micro_ros_agent" ]; then
        git clone -b ${opts.rosDistro} https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent || true
    fi
    if [ ! -d "src/micro_ros_msgs" ]; then
        git clone -b ${opts.rosDistro} https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs || true
    fi
    source "/opt/ros/${opts.rosDistro}/setup.bash"
    colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
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
    if ! grep -F -q "USE_${nameUpper}_CONFIG" config/config.h; then
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
    if ! grep -F -q "[env:${name}]" "firmware/platformio.ini"; then
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
info "Step 8/9: Building target '${target}' for environment '[env:${name}]'..."
info "💡 Note: First-time micro-ROS compilation builds IDL packages and may take 2-4 minutes on lower-spec SBCs (e.g. Raspberry Pi). Subsequent builds will be cached and fast."
if [ -d "${target}" ]; then
    pio run -d "${target}" -e "${name}"
    success "Build SUCCEEDED for '${name}' in ${target}/"

    if [ -n "${port}" ] && [ "${port}" != "AUTO" ]; then
        info "Releasing port '${port}' (closing active micro-ROS agents and serial monitors)..."
        fuser -k "${port}" 2>/dev/null || true
        pkill -f "micro_ros_agent.*${port}" 2>/dev/null || true
        pkill -f "miniterm.*${port}" 2>/dev/null || true
        pkill -f "screen.*${port}" 2>/dev/null || true
        sleep 0.8

        info "Uploading firmware to microcontroller on port '${port}'..."
        pio run -d "${target}" -e "${name}" -t upload --upload-port "${port}" || {
            warn "Initial upload attempt exited. Retrying upload after resetting serial port..."
            fuser -k "${port}" 2>/dev/null || true
            sleep 1
            pio run -d "${target}" -e "${name}" -t upload --upload-port "${port}" || {
                warn "Attempting fallback auto-upload protocol..."
                pio run -d "${target}" -e "${name}" -t upload
            }
        }
        success "Microcontroller flashing COMPLETE!"
    fi
else
    warn "Target directory '${target}/' not found in current workspace."
fi

# -----------------------------------------------------------------------------
# Phase 9: Start micro-ROS Agent & Discover Active ROS 2 Topics (for firmware)
# -----------------------------------------------------------------------------
if [ "${target}" = "firmware" ] && [ "${opts.rosDistro}" != "none" ]; then
    info "Step 9/9: Launching micro-ROS Agent & discovering active ROS 2 topics..."
    
    # Source ROS 2 environment
    if [ -f "/opt/ros/${opts.rosDistro}/setup.bash" ]; then
        source "/opt/ros/${opts.rosDistro}/setup.bash"
    fi
    if [ -f "$HOME/uros_ws/install/setup.bash" ]; then
        source "$HOME/uros_ws/install/setup.bash"
    fi

    # Terminate any existing micro_ros_agent on this port/protocol
    pkill -f "micro_ros_agent.*${port}" 2>/dev/null || true
    pkill -f "micro_ros_agent.*udp4" 2>/dev/null || true
    sleep 0.5

    AGENT_LOG="/tmp/micro_ros_agent_${name}.log"
    info "Starting micro-ROS agent in background (logging to \${AGENT_LOG})..."
    
    ${spec.transport === "WIFI" || spec.transport === "UDP" ? `
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 > "\${AGENT_LOG}" 2>&1 &
    AGENT_PID=$!
    ` : `
    ros2 run micro_ros_agent micro_ros_agent serial --dev "${port}" -b "${spec.baudrate || 921600}" > "\${AGENT_LOG}" 2>&1 &
    AGENT_PID=$!
    `}

    info "Agent PID: \${AGENT_PID}. Waiting for session handshake from microcontroller..."
    
    # Poll for micro-ROS session establishment (up to 8 seconds)
    CONNECTED=0
    for i in $(seq 1 8); do
        sleep 1
        if grep -qi "session established" "\${AGENT_LOG}" 2>/dev/null || ros2 node list 2>/dev/null | grep -q "linorobot2"; then
            CONNECTED=1
            break
        fi
        echo -n "."
    done
    echo ""

    if [ "\${CONNECTED}" -eq 1 ]; then
        success "🎉 micro-ROS Agent connected successfully! Active ROS 2 Session Established."
    else
        info "micro-ROS Agent active in background. Querying active ROS 2 graph..."
    fi

    info "========================================================="
    info "  Active ROS 2 Nodes on Robot Network:"
    info "========================================================="
    ros2 node list 2>/dev/null || echo "  (no nodes discovered yet)"
    
    info "========================================================="
    info "  Active ROS 2 Topics & Message Types:"
    info "========================================================="
    ros2 topic list -t 2>/dev/null || echo "  (no topics discovered yet)"
    info "========================================================="
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
      executeCommandInTerminal(cmd, "Installing ROS 2 Distribution & Packages");
    });
  }

  // micro-ROS Agent Quick Actions
  const btnCopyAgent = document.getElementById("btn-copy-agent-cmd");
  if (btnCopyAgent) {
    btnCopyAgent.addEventListener("click", () => {
      const cmd = document.getElementById("preview-agent-cmd")?.innerText || "";
      navigator.clipboard.writeText(cmd).then(() => showToast("📋 micro-ROS Agent command copied!"));
    });
  }

  const btnExecAgentSerial = document.getElementById("btn-exec-agent-serial");
  if (btnExecAgentSerial) {
    btnExecAgentSerial.addEventListener("click", () => {
      const opts = readAutomationOptions();
      const distro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
      const port = document.getElementById("auto-flash-port")?.value.trim() || "/dev/ttyUSB0";
      const baud = document.getElementById("auto-agent-baud")?.value || "921600";
      const cmd = `source /opt/ros/${distro}/setup.bash && [ -f "$HOME/uros_ws/install/setup.bash" ] && source "$HOME/uros_ws/install/setup.bash"; ros2 run micro_ros_agent micro_ros_agent serial --dev ${port} -b ${baud}`;
      executeCommandInTerminal(cmd, `Launching Single Serial micro-ROS Agent (${port} @ ${baud}) on Robot SBC`);
    });
  }

  const btnExecAgentMultiSerial = document.getElementById("btn-exec-agent-multiserial");
  if (btnExecAgentMultiSerial) {
    btnExecAgentMultiSerial.addEventListener("click", () => {
      const opts = readAutomationOptions();
      const distro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
      const multiPorts = document.getElementById("auto-agent-multiserial-ports")?.value.trim() || "/dev/ttyACM0 /dev/ttyUSB0";
      const baud = document.getElementById("auto-agent-baud")?.value || "921600";
      const cmd = `source /opt/ros/${distro}/setup.bash && [ -f "$HOME/uros_ws/install/setup.bash" ] && source "$HOME/uros_ws/install/setup.bash"; ros2 run micro_ros_agent micro_ros_agent multiserial --devs "${multiPorts}" -b ${baud}`;
      executeCommandInTerminal(cmd, `Launching Multi-Serial micro-ROS Agent (${multiPorts} @ ${baud}) on Robot SBC`);
    });
  }

  const btnExecAgentUdp = document.getElementById("btn-exec-agent-udp");
  if (btnExecAgentUdp) {
    btnExecAgentUdp.addEventListener("click", () => {
      const opts = readAutomationOptions();
      const distro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
      const cmd = `source /opt/ros/${distro}/setup.bash && [ -f "$HOME/uros_ws/install/setup.bash" ] && source "$HOME/uros_ws/install/setup.bash"; ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`;
      executeCommandInTerminal(cmd, `Launching UDP WiFi micro-ROS Agent (:8888) on Robot SBC`);
    });
  }

  // Flash Port & Multi-Serial Input Event Listeners
  const flashPortElem = document.getElementById("auto-flash-port");
  if (flashPortElem) {
    flashPortElem.addEventListener("input", () => {
      flashPortElem.dataset.autoManaged = "false";
      updateAutomationPreviews();
    });
  }

  const multiPortsElem = document.getElementById("auto-agent-multiserial-ports");
  if (multiPortsElem) {
    multiPortsElem.addEventListener("input", () => {
      multiPortsElem.dataset.autoManaged = "false";
      updateAutomationPreviews();
    });
  }

  const agentBaudElem = document.getElementById("auto-agent-baud");
  if (agentBaudElem) {
    agentBaudElem.addEventListener("change", () => {
      updateAutomationPreviews();
    });
  }

  const btnExecMerge = document.getElementById("btn-exec-merge-cmd");
  if (btnExecMerge) {
    btnExecMerge.addEventListener("click", () => {
      const spec = currentSpec;
      const opts = readAutomationOptions();
      const name = spec.robot_name || "scout_pico2";
      const cmd = getMergeAndCommitCmd(spec, opts);
      executeCommandInTerminal(cmd, `Merging Headers & Committing '${name}'`);
    });
  }

  // Robust Direct Upload / Build Command Generator
  function getDirectUploadOrBuildCmd(target, isUpload = true) {
    const spec = currentSpec;
    const name = spec.robot_name || "scout_pico2";
    const nameUpper = name.toUpperCase();
    const opts = readAutomationOptions();
    const rosDistro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
    const port = document.getElementById("auto-flash-port")?.value.trim() || (isESP32(spec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0");
    const headerContent = generateCppHeader(spec);
    const pioSection = generatePlatformioEnv(spec);
    const urdfContent = generateUrdfXacro(spec);
    const targetDir = (target === "test_acc") ? "calibration" : target;

    const lines = [
      `set -e`,
      `export PATH="$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH"`,
      `export ROS_DISTRO=${rosDistro}`,
      `if [ -f "/opt/ros/${rosDistro}/setup.bash" ]; then source "/opt/ros/${rosDistro}/setup.bash"; fi`,
      `if [ -f "$HOME/uros_ws/install/setup.bash" ]; then source "$HOME/uros_ws/install/setup.bash"; fi`,
      ``,
      `# 1. Ensure custom robot headers and urdf are synced`,
      `mkdir -p config/custom urdf`,
      `cat << 'EOF_HEADER' > "config/custom/${name}_config.h"`,
      `${headerContent}`,
      `EOF_HEADER`,
      ``,
      `# 2. Ensure USE_${nameUpper}_CONFIG is registered in config/config.h`,
      `if [ -f "config/config.h" ]; then`,
      `    if ! grep -F -q "USE_${nameUpper}_CONFIG" config/config.h; then`,
      `        sed -i '/\\/\\/ add user configurations above this line/i #ifdef USE_${nameUpper}_CONFIG\\n    #include "custom/${name}_config.h"\\n#endif\\n' config/config.h`,
      `    fi`,
      `fi`,
      ``,
      `# 3. Ensure [env:${name}] is defined in firmware/platformio.ini`,
      `if [ -f "firmware/platformio.ini" ]; then`,
      `    if ! grep -F -q "[env:${name}]" "firmware/platformio.ini"; then`,
      `        echo "" >> "firmware/platformio.ini"`,
      `        cat << 'EOF_PIO' >> "firmware/platformio.ini"`,
      `${pioSection}`,
      `EOF_PIO`,
      `    fi`,
      `fi`,
      ``,
      `# 4. Ingest URDF Description`,
      `cat << 'EOF_URDF' > "urdf/${name}_properties.urdf.xacro"`,
      `${urdfContent}`,
      `EOF_URDF`,
      ``,
      `# 5. Compile & Upload via PlatformIO`
    ];

    if (isUpload) {
      lines.push(`# 5. Stop serial micro-ROS agent & monitors to release shared USB port (WiFi agent kept running)`);
      lines.push(`echo "Releasing USB port '${port}' (stopping serial micro-ROS agent and serial monitors)..."`);
      lines.push(`pkill -f "micro_ros_agent.*serial" 2>/dev/null || true`);
      lines.push(`pkill -f "micro_ros_agent.*multiserial" 2>/dev/null || true`);
      lines.push(`pkill -f "miniterm" 2>/dev/null || true`);
      lines.push(`pkill -f "screen.*${port}" 2>/dev/null || true`);
      if (port !== "AUTO" && port !== "BOOTSEL") {
        lines.push(`[ -e "${port}" ] && fuser -k "${port}" 2>/dev/null || true`);
      }
      lines.push(`sleep 0.8`);
      lines.push(``);
      lines.push(`# 6. Flash Firmware onto Microcontroller`);
      if (port === "AUTO" || port === "BOOTSEL") {
        lines.push(`pio run -d ${targetDir} -e ${name} -t upload`);
      } else {
        lines.push(`pio run -d ${targetDir} -e ${name} -t upload --upload-port ${port}`);
      }
    } else {
      lines.push(`pio run -d ${targetDir} -e ${name}`);
    }

    return lines.join("\n");
  }

  const btnExecBuild = document.getElementById("btn-exec-build-cmd");
  if (btnExecBuild) {
    btnExecBuild.addEventListener("click", () => {
      const target = document.getElementById("auto-flash-target")?.value || "firmware";
      const name = currentSpec.robot_name || "scout_pico2";
      const cmd = getDirectUploadOrBuildCmd(target, false);
      executeCommandInTerminal(cmd, `Building ${target}/ for [env:${name}]`);
    });
  }

  // Direct 1-Click Upload Buttons for test_sensors, test_motors, test_acc, adc_calibrate, firmware
  const directTargets = [
    { btnId: "btn-upload-sensors", target: "test_sensors", desc: "Sensor Diagnostics (I2C Scanner)" },
    { btnId: "btn-upload-motors", target: "test_motors", desc: "Motor Diagnostics (CPR & Direction)" },
    { btnId: "btn-upload-acc", target: "test_acc", desc: "Dynamic Acceleration Profiler" },
    { btnId: "btn-upload-adc", target: "adc_calibrate", desc: "ESP32 ADC Calibration & Linearization" },
    { btnId: "btn-upload-firmware", target: "firmware", desc: "Main Robot Firmware" }
  ];

  directTargets.forEach(({ btnId, target, desc }) => {
    const btn = document.getElementById(btnId);
    if (btn) {
      btn.addEventListener("click", () => {
        const targetSel = document.getElementById("auto-flash-target");
        if (targetSel) targetSel.value = target;
        updateAutomationPreviews();
        const port = document.getElementById("auto-flash-port")?.value.trim() || (isESP32(currentSpec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0");
        const cmd = getDirectUploadOrBuildCmd(target, true);
        executeCommandInTerminal(cmd, `⚡ Uploading ${desc} (${target}/ -> ${port})`);
      });
    }
  });

  const btnExecUpload = document.getElementById("btn-exec-upload-cmd");
  if (btnExecUpload) {
    btnExecUpload.addEventListener("click", () => {
      const target = document.getElementById("auto-flash-target")?.value || "firmware";
      const port = document.getElementById("auto-flash-port")?.value.trim() || (isESP32(currentSpec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0");
      const cmd = getDirectUploadOrBuildCmd(target, true);
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
      const cmd = `cat << 'EOF_DEPLOY_SCRIPT_WRAPPER' > deploy_${name}.sh\n${script}\nEOF_DEPLOY_SCRIPT_WRAPPER\nchmod +x deploy_${name}.sh && bash ./deploy_${name}.sh`;
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

  // Stream SBC Serial Monitor
  const btnSbcMonitor = document.getElementById("btn-sbc-monitor");
  if (btnSbcMonitor) {
    btnSbcMonitor.addEventListener("click", () => {
      const port = document.getElementById("auto-flash-port")?.value.trim() || (isESP32(currentSpec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0");
      const baud = document.getElementById("serial-baud-select")?.value || "921600";
      const cmd = `python3 -u -c "import serial, sys, time; ser = serial.Serial('${port}', ${baud}, timeout=0.2); print('📡 Connected to ${port} @ ${baud} baud. Streaming live microcontroller output (test_sensors / telemetry)... Click ⏹️ Stop Process to disconnect.\\n', flush=True); [sys.stdout.write(l.decode('utf-8', errors='replace')) or sys.stdout.flush() for l in iter(ser.readline, b'')] || true"`;
      executeCommandInTerminal(cmd, `📡 Streaming Serial Telemetry (${port} @ ${baud} baud)`);
    });
  }

  // Web Serial Event Listeners (Client Browser USB)
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
        const distroLabel = data.distro_name || data.distro_id || data.os;
        if (text) text.innerText = `Robot SBC: Online [${data.node || "host"}] (${data.machine || ""})`;

        syncRobotHostInfo(data);
        return;
      }
    }
  } catch (e) {
    // Runner offline
  }

  isRunnerOnline = false;
  if (pill) pill.className = "runner-status-pill pill-offline";
  if (text) text.innerText = "Robot SBC: Disconnected / Copy Mode";
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
    await checkServerRunnerStatus();
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

  // Update Hero Deploy Badge with Live Phase & Timer
  const heroBadge = document.getElementById("hero-deploy-badge");
  const heroStatusText = document.getElementById("hero-deploy-status-text");
  const heroTimer = document.getElementById("hero-deploy-timer");
  let deployStartTime = Date.now();
  let deployTimerInterval = null;

  if (heroBadge && title.includes("Full Deploy")) {
    heroBadge.className = "deploy-hero-badge deploying";
    if (heroStatusText) heroStatusText.innerText = "Deploying...";
    if (heroTimer) {
      heroTimer.style.display = "inline";
      heroTimer.innerText = "(0s)";
      deployTimerInterval = setInterval(() => {
        const sec = Math.floor((Date.now() - deployStartTime) / 1000);
        heroTimer.innerText = `(${sec}s)`;
      }, 1000);
    }
  }


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
    const decoder = new TextDecoder("utf-8");

    let buffer = "";
    let adcStreamBuffer = "";
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value) {
        buffer += decoder.decode(value, { stream: true });
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
              if ((eventType === "output" || eventType === "log" || eventType === "message") && (payload.line !== undefined || payload.text !== undefined || payload.data !== undefined)) {
                const textLine = payload.line !== undefined ? payload.line : (payload.text !== undefined ? payload.text : payload.data);
                appendTerminalLine(textLine, "out");

                // Parse deployment phase for Hero Badge
                if (heroStatusText && title.includes("Full Deploy")) {
                  if (textLine.includes("Step 1")) heroStatusText.innerText = "1/9: Checking Tools";
                  else if (textLine.includes("Step 2")) heroStatusText.innerText = "2/9: Verifying ROS 2";
                  else if (textLine.includes("Step 4")) heroStatusText.innerText = "4/9: Ingesting C++ Header";
                  else if (textLine.includes("Step 6")) heroStatusText.innerText = "6/9: Ingesting URDF";
                  else if (textLine.includes("Step 8") || textLine.includes("Compiling")) heroStatusText.innerText = "8/9: Compiling Firmware (Please Wait...)";
                  else if (textLine.includes("Uploading firmware") || textLine.includes("Writing at")) heroStatusText.innerText = "8/9: Flashing Microcontroller...";
                  else if (textLine.includes("Step 9") || textLine.includes("Starting micro-ROS agent")) heroStatusText.innerText = "9/9: Launching Agent & Topics...";
                  else if (textLine.includes("Deployment SUCCEEDED") || textLine.includes("ACTIVE TOPICS")) {
                    heroBadge.className = "deploy-hero-badge succeeded";
                    heroStatusText.innerText = "✅ Deployed (@ 50Hz)";
                    if (deployTimerInterval) clearInterval(deployTimerInterval);
                  }
                }


                // Live ADC Calibration Stream Parser
                if (payload.line.includes("Test Linearity") || payload.line.includes("Generating LUT") || payload.line.includes("ADC_LUT")) {
                  const statusText = document.getElementById("adc-status-text");
                  if (statusText) {
                    statusText.innerText = payload.line.trim();
                    statusText.style.color = "#38bdf8";
                  }
                }

                
                // Live I2C Sensor Detection JSON Parser
                if (textLine.includes("[I2C_JSON]")) {
                  try {
                    const jsonStr = textLine.substring(textLine.indexOf("[I2C_JSON]") + 10).trim();
                    const i2cData = JSON.parse(jsonStr);
                    if (i2cData.status === "ok") {
                      handleI2cAutoDetectedSensors(i2cData);
                    }
                  } catch (err) {
                    console.error("Failed to parse I2C_JSON:", err);
                  }
                }

                adcStreamBuffer += payload.line + "\n";
                if (adcStreamBuffer.includes("const int16_t ADC_LUT[4096]") && adcStreamBuffer.includes("};")) {
                  try {
                    const startIdx = adcStreamBuffer.indexOf("{");
                    const endIdx = adcStreamBuffer.lastIndexOf("}");
                    if (startIdx !== -1 && endIdx > startIdx) {
                      const inner = adcStreamBuffer.substring(startIdx + 1, endIdx);
                      const parsedPoints = inner.split(/[\s,]+/).map(s => parseInt(s.trim(), 10)).filter(n => !isNaN(n));
                      if (parsedPoints.length >= 4000) {
                        currentAdcLut = parsedPoints.slice(0, 4096);
                        while (currentAdcLut.length < 4096) currentAdcLut.push(4095);
                        renderAdcChart(currentRawAdcCurve || currentAdcLut, currentAdcLut, null);
                        const codeDisplay = document.getElementById("adc-lut-code-display");
                        if (codeDisplay) codeDisplay.innerText = formatAdcLutArray(currentAdcLut);
                        const btnMerge = document.getElementById("btn-adc-merge-lut");
                        if (btnMerge) btnMerge.disabled = false;
                        const statusText = document.getElementById("adc-status-text");
                        if (statusText) {
                          statusText.innerText = "🎉 Hardware ADC Calibration Complete! 4096 points captured.";
                          statusText.style.color = "var(--success)";
                        }
                        showToast("🎉 Hardware ADC Calibration Complete! LUT captured.");
                      }
                    }
                  } catch (e) {
                    console.error("ADC parse error:", e);
                  }
                }
              } else if (eventType === "done") {
                const exitCode = payload.code !== undefined ? payload.code : (payload.exit_code !== undefined ? payload.exit_code : 0);
                if (exitCode === 0) {
                  appendTerminalLine(`\n✅ [SUCCESS] Command finished with exit code 0!`, "out");
                  showToast("✅ Command executed successfully!");
                } else {
                  appendTerminalLine(`\n❌ [FAILED] Command exited with code ${exitCode}`, "err");
                  showToast(`⚠️ Command exited with code ${exitCode}`);
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

// Formats 4096-entry ADC LUT into clean C++ header syntax (16 items per row)
function formatAdcLutArray(lutData) {
  if (typeof lutData === "string") return lutData;
  if (!Array.isArray(lutData) || lutData.length === 0) {
    return `const int16_t ADC_LUT[4096] = { /* insert adc_calibrate data here */ };`;
  }
  const rows = [];
  for (let i = 0; i < lutData.length; i += 16) {
    const slice = lutData.slice(i, i + 16);
    rows.push("    " + slice.map(n => Math.round(n)).join(", ") + (i + 16 < lutData.length ? "," : ""));
  }
  return `const int16_t ADC_LUT[4096] = {\n${rows.join("\n")}\n};`;
}

// ==============================================================================
// 🔋 ESP32 ADC Calibration Studio & LUT Linearizer
// ==============================================================================
let currentAdcLut = null;
let currentRawAdcCurve = null;

function initAdcCalibrationStudio() {
  const canvas = document.getElementById("adc-chart-canvas");
  const tooltip = document.getElementById("adc-chart-tooltip");
  const btnRun = document.getElementById("btn-adc-run-cal");
  const btnSim = document.getElementById("btn-adc-sim-cal");
  const btnMerge = document.getElementById("btn-adc-merge-lut");
  const btnCopy = document.getElementById("btn-copy-lut-code");

  if (!canvas) return;

  // Initialize with Theoretical / Empirical ESP32 Model
  generateAdcModelLut(false);

  if (btnSim) {
    btnSim.addEventListener("click", () => {
      generateAdcModelLut(true);
    });
  }

  if (btnRun) {
    btnRun.addEventListener("click", () => {
      runHardwareAdcCalibration();
    });
  }

  if (btnMerge) {
    btnMerge.addEventListener("click", () => {
      mergeAdcLutIntoConfig();
    });
  }

  if (btnCopy) {
    btnCopy.addEventListener("click", () => {
      if (currentAdcLut) {
        const code = formatAdcLutArray(currentAdcLut);
        navigator.clipboard.writeText(code).then(() => {
          showToast("📋 ADC_LUT[4096] array copied to clipboard!");
        });
      } else {
        showToast("⚠️ No LUT table generated yet.");
      }
    });
  }

  // Interactive Hover / Tooltip on Canvas
  canvas.addEventListener("mousemove", (e) => {
    if (!currentRawAdcCurve || !currentAdcLut) return;
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const paddingLeft = 45;
    const paddingRight = 15;
    const chartWidth = rect.width - paddingLeft - paddingRight;

    if (x >= paddingLeft && x <= paddingLeft + chartWidth) {
      const ratio = (x - paddingLeft) / chartWidth;
      const index = Math.min(4095, Math.max(0, Math.round(ratio * 4095)));
      renderAdcChart(currentRawAdcCurve, currentAdcLut, index);

      if (tooltip) {
        const rawVal = currentRawAdcCurve[index];
        const lutVal = currentAdcLut[index];
        const vInput = (index * 3.3 / 4095).toFixed(3);
        const vRaw = (rawVal * 3.3 / 4095).toFixed(3);
        const vCal = (lutVal * 3.3 / 4095).toFixed(3);
        const delta = lutVal - rawVal;

        tooltip.style.display = "block";
        tooltip.style.left = `${Math.min(rect.width - 180, Math.max(10, x - 70))}px`;
        tooltip.style.top = `15px`;
        tooltip.innerHTML = `
          <div><strong>Input: ${index}</strong> (${vInput}V)</div>
          <div style="color:#f43f5e;">Raw ADC: ${Math.round(rawVal)} (${vRaw}V)</div>
          <div style="color:#10b981;">Linearized: ${Math.round(lutVal)} (${vCal}V)</div>
          <div style="color:#a5b4fc;">Offset Δ: ${delta >= 0 ? "+" : ""}${Math.round(delta)} counts</div>
        `;
      }
    } else {
      if (tooltip) tooltip.style.display = "none";
      renderAdcChart(currentRawAdcCurve, currentAdcLut, null);
    }
  });

  canvas.addEventListener("mouseleave", () => {
    if (tooltip) tooltip.style.display = "none";
    if (currentRawAdcCurve && currentAdcLut) {
      renderAdcChart(currentRawAdcCurve, currentAdcLut, null);
    }
  });

  // Handle responsive window resize for canvas
  window.addEventListener("resize", () => {
    if (currentRawAdcCurve && currentAdcLut) {
      renderAdcChart(currentRawAdcCurve, currentAdcLut, null);
    }
  });
}

// Generate Mathematical / Empirical ESP32 ADC Transfer Function & Inverted LUT
function generateAdcModelLut(showToastMsg = true) {
  const rawCurve = new Float32Array(4096);
  const lut = new Int16Array(4096);

  // ESP32 Non-linear ADC Simulation:
  // 1. Deadband near 0V: 0 to ~140mV outputs 0
  // 2. Non-linear knee at low voltage (140mV - 500mV)
  // 3. Linear response from 500mV to ~2900mV
  // 4. Saturation compression from 2900mV to 3300mV (maxes out around 3.15V)
  for (let i = 0; i < 4096; i++) {
    const v = i * (3.3 / 4095.0); // True Input Voltage (0 to 3.3V)
    let rawAdc = 0;

    if (v < 0.14) {
      rawAdc = 0;
    } else if (v < 0.50) {
      const t = (v - 0.14) / (0.50 - 0.14);
      rawAdc = 620 * (t * t * 0.7 + t * 0.3);
    } else if (v <= 2.90) {
      const t = (v - 0.50) / (2.90 - 0.50);
      rawAdc = 620 + t * (3780 - 620);
    } else {
      const t = Math.min(1.0, (v - 2.90) / (3.15 - 2.90));
      rawAdc = 3780 + Math.sin(t * Math.PI / 2) * (4095 - 3780);
    }
    rawCurve[i] = Math.min(4095, Math.max(0, rawAdc));
  }

  // Generate Inverted Lookup Table (LUT) to Linearize ADC readings:
  // For each ADC reading y (0..4095), find input x such that rawCurve[x] == y
  for (let adc = 0; adc < 4096; adc++) {
    if (adc <= 0) {
      lut[0] = 0;
      continue;
    }
    if (adc >= 4095) {
      lut[4095] = 4095;
      continue;
    }
    let low = 0;
    let high = 4095;
    while (low < high) {
      const mid = (low + high) >> 1;
      if (rawCurve[mid] < adc) {
        low = mid + 1;
      } else {
        high = mid;
      }
    }
    let corrected = low;
    if (low > 0 && low < 4095) {
      const y0 = rawCurve[low - 1];
      const y1 = rawCurve[low];
      if (y1 > y0) {
        corrected = (low - 1) + (adc - y0) / (y1 - y0);
      }
    }
    lut[adc] = Math.min(4095, Math.max(0, Math.round(corrected)));
  }

  currentRawAdcCurve = Array.from(rawCurve);
  currentAdcLut = Array.from(lut);

  // Update UI Elements
  renderAdcChart(currentRawAdcCurve, currentAdcLut, null);

  const codeDisplay = document.getElementById("adc-lut-code-display");
  if (codeDisplay) {
    codeDisplay.innerText = formatAdcLutArray(currentAdcLut);
  }

  const btnMerge = document.getElementById("btn-adc-merge-lut");
  if (btnMerge) btnMerge.disabled = false;

  const statusText = document.getElementById("adc-status-text");
  if (statusText) {
    statusText.innerText = "Model LUT generated. Ready to merge into robot configuration.";
    statusText.style.color = "var(--success)";
  }

  if (showToastMsg) {
    showToast("🧪 Model ADC LUT generated! Graph and 4096-point table ready.");
  }
}

// Interactive High-Performance Canvas Renderer
function renderAdcChart(rawData, lutData, hoveredIndex = null) {
  const canvas = document.getElementById("adc-chart-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();

  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.scale(dpr, dpr);

  const w = rect.width;
  const h = rect.height;
  const padLeft = 45;
  const padBottom = 30;
  const padTop = 15;
  const padRight = 15;
  const plotW = w - padLeft - padRight;
  const plotH = h - padTop - padBottom;

  // Clear background
  ctx.fillStyle = "#090d16";
  ctx.fillRect(0, 0, w, h);

  // Draw Grid Lines & Axis Labels
  ctx.lineWidth = 1;
  ctx.strokeStyle = "rgba(255, 255, 255, 0.07)";
  ctx.fillStyle = "#64748b";
  ctx.font = "10px Inter, -apple-system, sans-serif";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";

  const numGrid = 4;
  for (let i = 0; i <= numGrid; i++) {
    const yVal = (4095 / numGrid) * i;
    const yPos = padTop + plotH - (i / numGrid) * plotH;
    const volts = (yVal * 3.3 / 4095).toFixed(1);

    ctx.beginPath();
    ctx.moveTo(padLeft, yPos);
    ctx.lineTo(padLeft + plotW, yPos);
    ctx.stroke();

    ctx.fillText(`${volts}V`, padLeft - 6, yPos);
  }

  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let i = 0; i <= numGrid; i++) {
    const xVal = (4095 / numGrid) * i;
    const xPos = padLeft + (i / numGrid) * plotW;
    const volts = (xVal * 3.3 / 4095).toFixed(1);

    ctx.beginPath();
    ctx.moveTo(xPos, padTop);
    ctx.lineTo(xPos, padTop + plotH);
    ctx.stroke();

    ctx.fillText(`${volts}V`, xPos, padTop + plotH + 6);
  }

  // 1. Plot Ideal Reference Line (Cyan Dashed)
  ctx.save();
  ctx.strokeStyle = "rgba(56, 189, 248, 0.4)";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(padLeft, padTop + plotH);
  ctx.lineTo(padLeft + plotW, padTop);
  ctx.stroke();
  ctx.restore();

  // 2. Plot Raw Non-Linear Curve (Rose/Coral)
  if (rawData && rawData.length > 0) {
    ctx.save();
    ctx.strokeStyle = "#f43f5e";
    ctx.lineWidth = 2;
    ctx.beginPath();
    const step = Math.max(1, Math.floor(rawData.length / plotW));
    for (let i = 0; i < rawData.length; i += step) {
      const xPos = padLeft + (i / 4095) * plotW;
      const yPos = padTop + plotH - (rawData[i] / 4095) * plotH;
      if (i === 0) ctx.moveTo(xPos, yPos);
      else ctx.lineTo(xPos, yPos);
    }
    ctx.stroke();
    ctx.restore();
  }

  // 3. Plot Calibrated Output Curve (Neon Emerald Green)
  if (lutData && rawData && lutData.length > 0) {
    ctx.save();
    ctx.strokeStyle = "#10b981";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    const step = Math.max(1, Math.floor(lutData.length / plotW));
    for (let i = 0; i < lutData.length; i += step) {
      const rawVal = Math.round(rawData[i]);
      const calVal = (rawVal >= 0 && rawVal < lutData.length) ? lutData[rawVal] : rawVal;
      const xPos = padLeft + (i / 4095) * plotW;
      const yPos = padTop + plotH - (calVal / 4095) * plotH;
      if (i === 0) ctx.moveTo(xPos, yPos);
      else ctx.lineTo(xPos, yPos);
    }
    ctx.stroke();
    ctx.restore();
  }

  // 4. Draw Crosshair & Indicators on Hover
  if (hoveredIndex !== null && hoveredIndex >= 0 && hoveredIndex < 4096) {
    const xPos = padLeft + (hoveredIndex / 4095) * plotW;
    const rawVal = rawData ? rawData[hoveredIndex] : hoveredIndex;
    const rawYPos = padTop + plotH - (rawVal / 4095) * plotH;
    const calVal = (lutData && Math.round(rawVal) < lutData.length) ? lutData[Math.round(rawVal)] : hoveredIndex;
    const calYPos = padTop + plotH - (calVal / 4095) * plotH;

    // Vertical Crosshair
    ctx.strokeStyle = "rgba(165, 180, 252, 0.5)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(xPos, padTop);
    ctx.lineTo(xPos, padTop + plotH);
    ctx.stroke();

    // Raw Point Dot
    ctx.fillStyle = "#f43f5e";
    ctx.beginPath();
    ctx.arc(xPos, rawYPos, 4.5, 0, Math.PI * 2);
    ctx.fill();

    // Calibrated Point Dot
    ctx.fillStyle = "#10b981";
    ctx.beginPath();
    ctx.arc(xPos, calYPos, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }
}

// Live Hardware ADC Calibration Runner
function runHardwareAdcCalibration() {
  const spec = currentSpec;
  const port = document.getElementById("auto-flash-port")?.value.trim() || (isESP32(spec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0");
  const cmd = getDirectUploadOrBuildCmd("adc_calibrate", true);

  const statusText = document.getElementById("adc-status-text");
  if (statusText) {
    statusText.innerText = `Compiling & Uploading adc_calibrate to ${port}...`;
    statusText.style.color = "#38bdf8";
  }

  executeCommandInTerminal(cmd, `⚡ Uploading & Running ADC Calibration (${port})`);
}

// Merge Generated ADC LUT Table into Robot Configuration
function mergeAdcLutIntoConfig() {
  if (!currentAdcLut || currentAdcLut.length === 0) {
    showToast("⚠️ No LUT table generated to merge.");
    return;
  }

  currentSpec.adc_lut = Array.from(currentAdcLut);
  if (!currentSpec.sensors) currentSpec.sensors = {};
  currentSpec.sensors.battery_monitor = "ADC_DIVIDER";
  currentSpec.sensors.adc_lut = Array.from(currentAdcLut);

  // Re-generate C++ Headers and update previews
  updateAutomationPreviews();
  renderActiveCode();

  const name = currentSpec.robot_name || "robot";
  showToast(`✅ ADC LUT successfully merged into '${name}_config.h'!`);

  const statusText = document.getElementById("adc-status-text");
  if (statusText) {
    statusText.innerText = `✅ ADC_LUT successfully merged into config/custom/${name}_config.h`;
    statusText.style.color = "var(--success)";
  }
}



// Global Existing Config Base State
let baseConfigSpec = null;
let existingConfigsList = [];

// Fetch available existing configs from server
async function loadExistingConfigsList() {
  const sel = document.getElementById('existing-config-select');
  if (!sel) return;
  try {
    const res = await fetch('/api/configs');
    if (!res.ok) return;
    const data = await res.json();
    existingConfigsList = data.configs || [];
    sel.innerHTML = '<option value="">Select Existing Config...</option>';
    existingConfigsList.forEach(c => {
      const opt = document.createElement('option');
      opt.value = c.filename;
      opt.innerText = `${c.filename} (${c.mcu} | ${c.kinematics})`;
      sel.appendChild(opt);
    });
  } catch (e) {
    console.log('Configs API not available:', e);
  }
}

// Load selected existing configuration into form for modification
async function loadExistingConfig(filename) {
  if (!filename) return;
  const cfg = existingConfigsList.find(c => c.filename === filename);
  if (!cfg) return;
  try {
    const res = await fetch('/api/parse', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ content: cfg.content, is_json: false })
    });
    const data = await res.json();
    if (data.status === 'ok' && data.spec) {
      baseConfigSpec = JSON.parse(JSON.stringify(data.spec));
      currentSpec = data.spec;
      populateFormFromSpec(currentSpec);
      recomputeAll();
      showToast(`📂 Loaded '${filename}'! You can now modify and merge settings.`);
    }
  } catch (e) {
    showToast(`⚠️ Failed to parse ${filename}: ` + e.message);
  }
}

// Merge modified UI form settings into base configuration
async function mergeCurrentConfig() {
  readFormIntoSpec();
  try {
    const res = await fetch('/api/merge', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        base_spec: baseConfigSpec || currentSpec,
        override_spec: currentSpec
      })
    });
    const data = await res.json();
    if (data.status === 'ok') {
      currentSpec = data.merged_spec;
      const count = data.changes ? data.changes.length : 0;
      updateAutomationPreviews();
      renderActiveCode();
      showToast(`🔄 Merged ${count} updated setting(s) successfully!`);
    }
  } catch (e) {
    showToast('⚠️ Merge error: ' + e.message);
  }
}

// Save merged configuration directly to config/custom/
async function saveConfigToFirmware() {
  readFormIntoSpec();
  const name = currentSpec.robot_name || 'robot';
  const filename = `${name}_config.h`;
  const headerCode = generateCppConfigHeader(currentSpec);
  try {
    const res = await fetch('/api/save', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        filename: filename,
        header: headerCode,
        spec: currentSpec
      })
    });
    const data = await res.json();
    if (data.status === 'saved') {
      showToast(`💾 Saved to config/custom/${filename}! Ready to build & flash.`);
    }
  } catch (e) {
    showToast('⚠️ Save error: ' + e.message);
  }
}


  // Existing Config & Merge Handlers
  loadExistingConfigsList();
  initSyslogControls();
  const btnLoadCfg = document.getElementById("btn-load-config");
  if (btnLoadCfg) {
    btnLoadCfg.addEventListener("click", () => {
      const sel = document.getElementById("existing-config-select");
      if (sel && sel.value) loadExistingConfig(sel.value);
      else showToast("Select an existing config first");
    });
  }
  const btnMergeCfg = document.getElementById("btn-merge-config");
  if (btnMergeCfg) btnMergeCfg.addEventListener("click", mergeCurrentConfig);
  const btnSaveCfg = document.getElementById("btn-save-config");
  if (btnSaveCfg) btnSaveCfg.addEventListener("click", saveConfigToFirmware);


// =============================================================================
// Live ROS 2 Topic Streamer, Hz Table & Inspector
// =============================================================================
let activeRos2EventSource = null;
let activeRos2StreamMode = null; // 'echo' or 'hz'
let selectedRos2Topic = "/odom/unfiltered";

async function fetchRos2Topics() {
  try {
    const res = await fetch("/api/ros2/topics");
    if (!res.ok) return;
    const data = await res.json();
    if (data.status === "ok" && Array.isArray(data.topics) && data.topics.length > 0) {
      renderRos2TopicTable(data.topics);
      updateRos2Dropdown(data.topics);
      const graphStatus = document.getElementById("ros2-graph-status");
      if (graphStatus) {
        graphStatus.textContent = `● ${data.topics.length} Topics Active`;
        graphStatus.className = "badge-pill badge-ok";
      }
    }
  } catch (err) {
    console.warn("fetchRos2Topics error:", err);
  }
}

function updateRos2Dropdown(topics) {
  const select = document.getElementById("ros2-topic-select");
  if (!select) return;
  const cur = select.value;
  select.innerHTML = "";
  topics.forEach(t => {
    const opt = document.createElement("option");
    opt.value = t.topic;
    opt.textContent = `${t.topic} (${t.type.split("/").pop()})`;
    select.appendChild(opt);
  });
  if (topics.some(t => t.topic === selectedRos2Topic)) {
    select.value = selectedRos2Topic;
  } else if (topics.some(t => t.topic === cur)) {
    select.value = cur;
  }
}

function renderRos2TopicTable(topics) {
  const tbody = document.getElementById("ros2-topic-table-body");
  if (!tbody) return;
  tbody.innerHTML = "";

  topics.forEach(t => {
    const isSelected = (t.topic === selectedRos2Topic);
    const tr = document.createElement("tr");
    tr.className = "topic-row" + (isSelected ? " active" : "");
    tr.setAttribute("data-topic", t.topic);

    let icon = "📡";
    if (t.topic.includes("odom")) icon = "🏎️";
    else if (t.topic.includes("imu")) icon = "🧭";
    else if (t.topic.includes("bat")) icon = "🔋";
    else if (t.topic.includes("cmd")) icon = "🎮";

    let hzClass = "topic-hz-badge";
    if (t.hz.includes("1.0") || t.hz.includes("Event")) hzClass += " hz-low";
    if (t.hz.includes("Sub")) hzClass += " hz-sub";

    tr.innerHTML = `
      <td style="text-align: center;"><input type="radio" name="topic-select-radio" value="${t.topic}" ${isSelected ? "checked" : ""}></td>
      <td><span class="topic-table-name font-mono">${icon} ${t.topic}</span></td>
      <td><span class="topic-table-type font-mono text-muted">${t.type}</span></td>
      <td style="text-align: center;"><span class="${hzClass}" id="hz-cell-${t.topic.replace(/[^a-zA-Z0-9]/g, '_')}">${t.hz}</span></td>
      <td style="text-align: center;"><span class="badge-pill ${t.status === 'active' ? 'badge-ok' : 'badge-info'}">${t.status === 'active' ? 'Active' : 'Sub'}</span></td>
      <td style="text-align: right;">
        <button type="button" class="btn btn-xs btn-accent btn-table-stream" data-topic="${t.topic}">▶️ Stream</button>
        ${t.status === 'active' ? `<button type="button" class="btn btn-xs btn-ghost btn-table-hz" data-topic="${t.topic}">⚡ Hz</button>` : ''}
      </td>
    `;

    // Row Click selects topic & starts live display
    tr.addEventListener("click", (e) => {
      if (e.target.tagName === "BUTTON") return;
      selectTopicAndDisplay(t.topic);
    });

    const btnStream = tr.querySelector(".btn-table-stream");
    if (btnStream) {
      btnStream.addEventListener("click", (e) => {
        e.stopPropagation();
        selectTopicAndDisplay(t.topic, "echo");
      });
    }

    const btnHz = tr.querySelector(".btn-table-hz");
    if (btnHz) {
      btnHz.addEventListener("click", async (e) => {
        e.stopPropagation();
        selectTopicAndDisplay(t.topic, "hz");
      });
    }

    tbody.appendChild(tr);
  });
}

function selectTopicAndDisplay(topic, mode = "echo") {
  selectedRos2Topic = topic;

  // Update table selection state
  document.querySelectorAll("#ros2-topic-table tbody tr").forEach(r => {
    if (r.getAttribute("data-topic") === topic) {
      r.classList.add("active");
      const rad = r.querySelector("input[type='radio']");
      if (rad) rad.checked = true;
    } else {
      r.classList.remove("active");
    }
  });

  // Update dropdown
  const select = document.getElementById("ros2-topic-select");
  if (select && select.value !== topic) {
    select.value = topic;
  }

  // Update HUD
  const hudTopic = document.getElementById("hud-topic-name");
  if (hudTopic) hudTopic.textContent = topic;

  startRos2Stream(topic, mode);
}

async function measureAllTopicHz() {
  const tbody = document.getElementById("ros2-topic-table-body");
  if (!tbody) return;
  const rows = tbody.querySelectorAll("tr.topic-row");
  for (const row of rows) {
    const topic = row.getAttribute("data-topic");
    if (!topic || topic.includes("cmd_vel")) continue;
    const badge = row.querySelector(".topic-hz-badge");
    if (badge) {
      badge.textContent = "Measuring...";
      badge.style.opacity = "0.6";
    }
    try {
      const res = await fetch(`/api/ros2/hz_single?topic=${encodeURIComponent(topic)}`);
      const data = await res.json();
      if (data.status === "ok" && data.hz && badge) {
        badge.textContent = data.hz;
        badge.style.opacity = "1.0";
      }
    } catch (e) {
      if (badge) badge.style.opacity = "1.0";
    }
  }
}

function stopRos2Stream() {
  if (activeRos2EventSource) {
    activeRos2EventSource.close();
    activeRos2EventSource = null;
  }
  activeRos2StreamMode = null;
  const echoBtn = document.getElementById("btn-ros2-echo");
  if (echoBtn) {
    const icon = document.getElementById("ros2-echo-icon");
    const text = document.getElementById("ros2-echo-text");
    if (icon) icon.textContent = "📡";
    if (text) text.textContent = "Stream Topic";
    echoBtn.className = "btn btn-sm btn-accent";
  }
  const cancelBtn = document.getElementById("btn-cancel-exec");
  if (cancelBtn) cancelBtn.style.display = "none";
}

function startRos2Stream(topic, mode = "echo") {
  stopRos2Stream();

  const screen = document.getElementById("serial-terminal-screen");
  if (screen) {
    screen.innerHTML = `<div class="terminal-line terminal-info">📡 Connecting to live ROS 2 topic stream: <strong>${topic}</strong> (mode: ${mode})...</div>`;
  }

  const hudTopic = document.getElementById("hud-topic-name");
  if (hudTopic) hudTopic.textContent = topic;

  const echoBtn = document.getElementById("btn-ros2-echo");
  if (echoBtn) {
    const icon = document.getElementById("ros2-echo-icon");
    const text = document.getElementById("ros2-echo-text");
    if (icon) icon.textContent = "⏹️";
    if (text) text.textContent = "Stop Stream";
    echoBtn.className = "btn btn-sm btn-danger";
  }
  const cancelBtn = document.getElementById("btn-cancel-exec");
  if (cancelBtn) {
    cancelBtn.style.display = "inline-flex";
    cancelBtn.onclick = stopRos2Stream;
  }

  activeRos2StreamMode = mode;
  const url = `/api/ros2/stream?topic=${encodeURIComponent(topic)}&mode=${encodeURIComponent(mode)}`;
  activeRos2EventSource = new EventSource(url);

  activeRos2EventSource.onmessage = function(e) {
    try {
      const data = JSON.parse(e.data);
      if (data.type === "line" || data.type === "start") {
        const text = data.text;
        appendTerminalLine(text);

        // Parse metrics for Live HUD
        if (text.includes("average rate:")) {
          const m = text.match(/average rate:\s*([0-9.]+)/);
          if (m) {
            const hzVal = document.getElementById("hud-topic-hz");
            if (hzVal) hzVal.textContent = `${parseFloat(m[1]).toFixed(1)} Hz`;
            const cellBadge = document.getElementById(`hz-cell-${topic.replace(/[^a-zA-Z0-9]/g, '_')}`);
            if (cellBadge) cellBadge.textContent = `${parseFloat(m[1]).toFixed(1)} Hz`;
          }
        }
        if (text.includes("linear:") || text.includes("x:")) {
          const m = text.match(/x:\s*([0-9.-]+)/);
          if (m && Math.abs(parseFloat(m[1])) > 0.0001) {
            const v1 = document.getElementById("hud-topic-val1");
            if (v1) v1.textContent = `${parseFloat(m[1]).toFixed(3)} m/s`;
          }
        }
        if (text.includes("angular:") || text.includes("z:")) {
          const m = text.match(/z:\s*([0-9.-]+)/);
          if (m && Math.abs(parseFloat(m[1])) > 0.0001) {
            const v2 = document.getElementById("hud-topic-val2");
            if (v2) v2.textContent = `${parseFloat(m[1]).toFixed(3)} rad/s`;
          }
        }
        if (text.includes("voltage:")) {
          const m = text.match(/voltage:\s*([0-9.]+)/);
          if (m) {
            const v1 = document.getElementById("hud-topic-val1");
            if (v1) v1.textContent = `${parseFloat(m[1]).toFixed(2)} V`;
          }
        }
      } else if (data.type === "error") {
        appendTerminalLine(`[ERROR] ${data.text}`, "err");
      }
    } catch (err) {
      appendTerminalLine(e.data, "dim");
    }
  };

  activeRos2EventSource.onerror = function() {
    appendTerminalLine("[INFO] ROS 2 topic stream ended.", "info");
    stopRos2Stream();
  };
}

function initRos2TopicInspector() {
  const echoBtn = document.getElementById("btn-ros2-echo");
  const hzBtn = document.getElementById("btn-ros2-hz");
  const refreshBtn = document.getElementById("btn-ros2-refresh");
  const inspRefreshBtn = document.getElementById("btn-ros2-inspector-refresh");
  const measureAllBtn = document.getElementById("btn-measure-all-hz");
  const topicSelect = document.getElementById("ros2-topic-select");

  if (echoBtn) {
    echoBtn.addEventListener("click", () => {
      if (activeRos2EventSource) {
        stopRos2Stream();
      } else {
        const t = topicSelect ? topicSelect.value : selectedRos2Topic;
        selectTopicAndDisplay(t, "echo");
      }
    });
  }
  if (hzBtn) {
    hzBtn.addEventListener("click", () => {
      const t = topicSelect ? topicSelect.value : selectedRos2Topic;
      selectTopicAndDisplay(t, "hz");
    });
  }
  if (refreshBtn) refreshBtn.addEventListener("click", fetchRos2Topics);
  if (inspRefreshBtn) inspRefreshBtn.addEventListener("click", fetchRos2Topics);
  if (measureAllBtn) measureAllBtn.addEventListener("click", measureAllTopicHz);

  if (topicSelect) {
    topicSelect.addEventListener("change", () => {
      selectTopicAndDisplay(topicSelect.value, activeRos2StreamMode || "echo");
    });
  }

  fetchRos2Topics();
  setInterval(fetchRos2Topics, 10000);
}

// -----------------------------------------------------------------------------
// AI I2C Sensor Auto-Detection & Auto-Configuration Handler
// -----------------------------------------------------------------------------
function handleI2cAutoDetectedSensors(data) {
  console.log("Auto-detected I2C Hardware:", data);
  const resultsDiv = document.getElementById("i2c-detect-results");
  let summaryParts = [];

  // 1. Auto-select IMU driver
  const imuSelect = document.getElementById("cfg-imu");
  if (imuSelect) {
    if (data.imu && data.imu !== "NONE") {
      for (let opt of imuSelect.options) {
        if (opt.value.toUpperCase().includes(data.imu.toUpperCase())) {
          imuSelect.value = opt.value;
          summaryParts.push(`IMU: <strong>${data.imu}</strong>`);
          break;
        }
      }
    } else {
      imuSelect.value = "FAKE";
    }
    imuSelect.dispatchEvent(new Event("change"));
  }

  // 2. Auto-select Magnetometer driver
  const magSelect = document.getElementById("cfg-mag");
  if (magSelect) {
    if (data.mag && data.mag !== "NONE") {
      for (let opt of magSelect.options) {
        if (opt.value.toUpperCase().includes(data.mag.toUpperCase())) {
          magSelect.value = opt.value;
          summaryParts.push(`MAG: <strong>${data.mag}</strong>`);
          break;
        }
      }
    } else {
      magSelect.value = "NONE";
    }
    magSelect.dispatchEvent(new Event("change"));
  }

  // 3. Auto-configure Current / Power Monitor
  const batSelect = document.getElementById("cfg-battery");
  if (batSelect) {
    if (data.current && data.current.includes("INA219")) {
      batSelect.value = "INA219";
      summaryParts.push(`Power: <strong>INA219</strong>`);
    }
    batSelect.dispatchEvent(new Event("change"));
  }

  // Update UI results box
  if (resultsDiv) {
    resultsDiv.style.display = "block";
    if (summaryParts.length > 0) {
      resultsDiv.innerHTML = `✅ <span style="color:#34d399;">Auto-Configured Drivers:</span> ${summaryParts.join(" | ")} <span class="badge-pill badge-ok ml-2">Drivers Applied</span>`;
    } else {
      resultsDiv.innerHTML = `ℹ️ <span style="color:#94a3b8;">No physical I2C chips detected. Configured to Bare Module mode (Fake IMU/MAG).</span>`;
    }
  }

  // Regenerate configuration headers & code
  if (typeof updateGeneratedCode === "function") {
    updateGeneratedCode();
  }

  const toastMsg = summaryParts.length > 0
    ? `🎉 Auto-Detected: ${summaryParts.join(", ")}! Drivers configured.`
    : `ℹ️ Bus scanned: No I2C chips detected (Bare Module Mode preserved).`;
  showToast(toastMsg);
}

function runI2cSensorAutoDetect() {
  const mcuEl = document.getElementById("cfg-mcu");
  const mcu = mcuEl ? mcuEl.value.toLowerCase() : "gendrv";
  let envName = "esp32";
  let baud = 921600;
  if (mcu.includes("gendrv")) {
    envName = "gendrv";
    baud = 1500000;
  } else if (mcu.includes("pico2")) {
    envName = "pico2";
  } else if (mcu.includes("pico")) {
    envName = "pico";
  } else if (mcu.includes("s3")) {
    envName = "esp32s3";
  }

  const portSelect = document.getElementById("auto-flash-port");
  const port = (portSelect && portSelect.value) ? portSelect.value.trim() : (envName === "gendrv" ? "/dev/ttyUSB0" : "/dev/ttyACM0");
  const uploadFlag = (port && !port.includes("BOOTSEL")) ? ` --upload-port ${port}` : "";

  // Flash i2c_detect and stream output via python miniterm reader
  const monitorPy = "import serial, time; s = serial.Serial('" + port + "', " + baud + ", timeout=2); time.sleep(0.4); t = time.time();\nwhile time.time() - t < 3:\n  l = s.readline().decode('utf-8', errors='ignore').strip()\n  if l: print(l)\ns.close()";
  const cmd = `cd i2c_detect && pio run -e ${envName} -t upload${uploadFlag} && ~/.platformio/penv/bin/python -c "${monitorPy}"`;
  executeCommandInTerminal(cmd, `🔍 AI Auto-Detecting I2C Sensors (${envName} -> ${port})`);
}


// =============================================================================
// 📡 Live UDP Syslog Server & Streamer Management
// =============================================================================
let activeSyslogEventSource = null;
let isSyslogRunning = false;

async function checkSyslogStatus() {
  try {
    const res = await fetch('/api/syslog/status');
    if (!res.ok) return;
    const data = await res.json();
    updateSyslogUIState(data);
    if (data.running && !activeSyslogEventSource) {
      connectSyslogStream();
    }
  } catch (e) {
    // server might be offline or reloading
  }
}

function updateSyslogUIState(data) {
  isSyslogRunning = !!data.running;
  const badge = document.getElementById('syslog-status-badge');
  const badgeText = document.getElementById('syslog-status-text');
  const btnStart = document.getElementById('btn-syslog-start');
  const btnStop = document.getElementById('btn-syslog-stop');
  const tab5Text = document.getElementById('tab5-syslog-text');
  const tab5Btn = document.getElementById('btn-tab5-launch-syslog');
  const statFile = document.getElementById('syslog-stat-file');
  const statPackets = document.getElementById('syslog-stat-packets');
  const statClient = document.getElementById('syslog-stat-client');
  const portInput = document.getElementById('cfg-syslog-port');

  if (portInput && data.port) {
    portInput.value = data.port;
  }

  if (badge && badgeText) {
    if (data.running) {
      badge.className = 'syslog-status-badge running';
      badgeText.textContent = `Syslog: UDP :${data.port}`;
    } else {
      badge.className = 'syslog-status-badge stopped';
      badgeText.textContent = 'Syslog: Offline';
    }
  }

  if (btnStart && btnStop) {
    if (data.running) {
      btnStart.style.display = 'none';
      btnStop.style.display = 'inline-flex';
    } else {
      btnStart.style.display = 'inline-flex';
      btnStop.style.display = 'none';
    }
  }

  if (tab5Text) {
    tab5Text.textContent = data.running ? `Syslog: :${data.port}` : 'Syslog Server';
  }
  if (tab5Btn) {
    if (data.running) {
      tab5Btn.classList.add('btn-accent');
      tab5Btn.classList.remove('btn-ghost');
    } else {
      tab5Btn.classList.remove('btn-accent');
      tab5Btn.classList.add('btn-ghost');
    }
  }

  if (statFile && data.logfile) statFile.textContent = data.logfile;
  if (statPackets && typeof data.packets === 'number') statPackets.textContent = data.packets;
  if (statClient && data.last_sender) statClient.textContent = data.last_sender;
}

async function startSyslogServer(requestedPort) {
  const portInput = document.getElementById('cfg-syslog-port');
  const port = requestedPort || (portInput ? parseInt(portInput.value) || 514 : 514);
  try {
    const res = await fetch('/api/syslog/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ port: port })
    });
    const data = await res.json();
    if (data.status === 'running' || data.status === 'already_running') {
      updateSyslogUIState(data);
      connectSyslogStream();
      const fallbackNote = data.fallback ? ` (fell back to :${data.port})` : '';
      showToast(`🟢 Syslog Server listening on UDP port ${data.port}${fallbackNote}!`);
      
      // Append notification to terminal
      const terminalScreen = document.getElementById('serial-terminal-screen');
      if (terminalScreen) {
        const lineEl = document.createElement('div');
        lineEl.className = 'terminal-line terminal-in';
        lineEl.textContent = `[SYSLOG SERVER] Listening on UDP 0.0.0.0:${data.port} | Saving to ${data.logfile}`;
        terminalScreen.appendChild(lineEl);
        terminalScreen.scrollTop = terminalScreen.scrollHeight;
      }
    } else {
      showToast(`⚠️ Failed to start syslog: ${data.error || 'Unknown error'}`);
    }
  } catch (e) {
    showToast(`⚠️ Syslog start error: ${e.message}`);
  }
}

async function stopSyslogServer() {
  try {
    const res = await fetch('/api/syslog/stop', { method: 'POST' });
    const data = await res.json();
    updateSyslogUIState(data);
    if (activeSyslogEventSource) {
      activeSyslogEventSource.close();
      activeSyslogEventSource = null;
    }
    showToast(`⏹️ Syslog Server stopped. Total packets: ${data.packets || 0}`);
    const terminalScreen = document.getElementById('serial-terminal-screen');
    if (terminalScreen) {
      const lineEl = document.createElement('div');
      lineEl.className = 'terminal-line terminal-dim';
      lineEl.textContent = `[SYSLOG SERVER] Stopped (Captured ${data.packets || 0} packets).`;
      terminalScreen.appendChild(lineEl);
      terminalScreen.scrollTop = terminalScreen.scrollHeight;
    }
  } catch (e) {
    showToast(`⚠️ Syslog stop error: ${e.message}`);
  }
}

function connectSyslogStream() {
  if (activeSyslogEventSource) {
    activeSyslogEventSource.close();
  }
  activeSyslogEventSource = new EventSource('/api/syslog/stream');
  
  activeSyslogEventSource.addEventListener('status', (e) => {
    try {
      const data = JSON.parse(e.data);
      updateSyslogUIState(data);
    } catch (_) {}
  });

  activeSyslogEventSource.addEventListener('syslog', (e) => {
    try {
      const data = JSON.parse(e.data);
      const statPackets = document.getElementById('syslog-stat-packets');
      const statClient = document.getElementById('syslog-stat-client');
      if (statPackets) {
        const cur = parseInt(statPackets.textContent) || 0;
        statPackets.textContent = cur + 1;
      }
      if (statClient) statClient.textContent = data.sender;

      const chkStream = document.getElementById('chk-stream-syslog-terminal');
      if (!chkStream || chkStream.checked) {
        const terminalScreen = document.getElementById('serial-terminal-screen');
        if (terminalScreen) {
          const lineEl = document.createElement('div');
          lineEl.className = 'terminal-line terminal-syslog';
          const timePart = data.time ? data.time.split(' ')[1] || data.time : '';
          const escapedMsg = (data.message || '').replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
          lineEl.innerHTML = `<span class="badge-syslog">SYSLOG</span> <span class="syslog-time">${timePart}</span> <span class="syslog-sender">[${data.sender}]</span> <span class="syslog-msg">${escapedMsg}</span>`;
          terminalScreen.appendChild(lineEl);
          
          const chkAutoScroll = document.getElementById('chk-serial-autoscroll');
          if (!chkAutoScroll || chkAutoScroll.checked) {
            terminalScreen.scrollTop = terminalScreen.scrollHeight;
          }
        }
      }
    } catch (_) {}
  });

  activeSyslogEventSource.onerror = () => {
    // will auto-reconnect
  };
}

async function viewSyslogFiles() {
  try {
    const res = await fetch('/api/syslog/logs?tail=30');
    if (!res.ok) return;
    const data = await res.json();
    const terminalScreen = document.getElementById('serial-terminal-screen');
    if (terminalScreen) {
      const lineEl = document.createElement('div');
      lineEl.className = 'terminal-line terminal-in';
      lineEl.textContent = `=== 📂 Syslog Files in logs/ (${data.files.length} files) ===`;
      terminalScreen.appendChild(lineEl);
      for (const f of data.files) {
        const fEl = document.createElement('div');
        fEl.className = 'terminal-line terminal-out';
        fEl.textContent = `  - ${f.name} (${f.size} bytes)`;
        terminalScreen.appendChild(fEl);
      }
      if (data.lines && data.lines.length > 0) {
        const hEl = document.createElement('div');
        hEl.className = 'terminal-line terminal-in';
        hEl.textContent = `--- Tail of ${data.current_file} ---`;
        terminalScreen.appendChild(hEl);
        for (const l of data.lines) {
          const lEl = document.createElement('div');
          lEl.className = 'terminal-line terminal-syslog';
          lEl.textContent = `  ${l}`;
          terminalScreen.appendChild(lEl);
        }
      }
      terminalScreen.scrollTop = terminalScreen.scrollHeight;
    }
    showToast(`📂 Found ${data.files.length} log files in logs/`);
  } catch (e) {
    showToast(`⚠️ Could not list logs: ${e.message}`);
  }
}

function initSyslogControls() {
  const btnStart = document.getElementById('btn-syslog-start');
  if (btnStart) btnStart.addEventListener('click', () => startSyslogServer());

  const btnStop = document.getElementById('btn-syslog-stop');
  if (btnStop) btnStop.addEventListener('click', () => stopSyslogServer());

  const btnTab5 = document.getElementById('btn-tab5-launch-syslog');
  if (btnTab5) {
    btnTab5.addEventListener('click', () => {
      if (isSyslogRunning) stopSyslogServer();
      else startSyslogServer();
    });
  }

  const btnViewLogs = document.getElementById('btn-syslog-view-logs');
  if (btnViewLogs) btnViewLogs.addEventListener('click', () => viewSyslogFiles());

  checkSyslogStatus();
}
