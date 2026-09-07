// Hardware USB MCU Classifier & Auto-detection Engine
function guessMcuFromUsbDetail(detail) {
  if (!detail) return null;
  const vid = (detail.vid || "").toLowerCase();
  const pid = (detail.pid || "").toLowerCase();
  const chip = (detail.chip || "").toLowerCase();
  const product = (detail.product || "").toLowerCase();

  // Raspberry Pi (VID 2e8a): distinguish RP2350 (Pico 2) from RP2040 (Pico) by
  // USB PID / product string — BOOTSEL is only a mode label, never the deciding
  // factor (a Pico 2 in BOOTSEL is 2e8a:000f, still an RP2350).
  if (vid === "2e8a" || chip.includes("rp204") || chip.includes("rp235") || product.includes("pico")) {
    const isPico2 = ["000f", "0005", "0009"].includes(pid)
      || chip.includes("rp2350") || chip.includes("pico 2") || product.includes("pico 2");
    const bootsel = !!detail.is_bootsel;
    return isPico2
      ? { mcu: "PICO2", baudrate: 921600, preset: "bare", isBootsel: bootsel,
          chipName: "Raspberry Pi Pico 2 (RP2350)" + (bootsel ? " [BOOTSEL Mode]" : "") }
      : { mcu: "PICO", baudrate: 921600, preset: "bare", isBootsel: bootsel,
          chipName: "Raspberry Pi Pico (RP2040)" + (bootsel ? " [BOOTSEL Mode]" : "") };
  }
  if (vid === "303a" || chip.includes("esp32-s3") || product.includes("esp32-s3")) {
    return { mcu: "ESP32S3", baudrate: 921600, chipName: "ESP32-S3 (Native USB CDC)", preset: "bare" };
  }
  // CP2102N (Silicon Labs) is rated for and HW-verified at 1.5M baud (same
  // bridge chip as GenDrv) — default straight to the high-throughput rate.
  // CP2102N and plain CP2102 share the same VID:PID (10c4:ea60); server.py's
  // chip string is the only thing that tells them apart (via the USB product
  // descriptor), so trust it exactly rather than re-guessing from vid/mfr —
  // a plain CP2102 (max ~1 Mbps per datasheet) must NOT take this branch.
  if (chip.includes("gendrv") || product.includes("general driver") || chip.includes("cp2102n")) {
    return { mcu: "ESP32", baudrate: 1500000, supportsHighBaud: true, chipName: "ESP32 DevKit (CP2102N)", preset: "bare" };
  }
  // FTDI FT232-family bridges are also reliable well past 921600.
  if (chip.includes("ftdi")) {
    return { mcu: "ESP32", baudrate: 1500000, supportsHighBaud: true, chipName: "ESP32 Microcontroller (FTDI)", preset: "bare" };
  }
  // CH340/CH341 and unidentified/plain CP210x bridges: no 1.5M reliability
  // guarantee (plain CP2102 is rated ~1 Mbps max) — keep the conservative
  // baseline and don't offer a speed the chip isn't confirmed to support.
  if (chip.includes("esp32") || chip.includes("cp210") || chip.includes("ch340")) {
    return { mcu: "ESP32", baudrate: 921600, supportsHighBaud: false, chipName: "ESP32 Microcontroller", preset: "bare" };
  }
  return null;
}

// Reflect a detected USB bridge chip onto the baud-rate UI: hide the 1.5M
// option when the chip isn't confirmed to support it (e.g. a plain CP2102,
// max ~1 Mbps), show/select it when it is, and display which chip drove
// the decision. Native-USB-CDC boards (Pico/Pico2/ESP32-S3) don't set
// `supportsHighBaud` and are left untouched — there's no bridge chip to
// rate-limit them.
// A freshly detected board is almost never a wired-up robot yet -- it is a
// module someone just plugged in. Default it to fake wheel mode so the very
// next thing they do (teleop, SLAM, Nav2 from Console) actually works, instead
// of a silent zero-RPM robot. Only ever applied while the user has not touched
// the simulation controls themselves, so an explicit choice is never undone.
let userTouchedSimulation = false;

function defaultSimulationForDetectedModule(chipName) {
  if (userTouchedSimulation) return;
  const fw = document.getElementById("cfg-fake-wheel");
  if (!fw || fw.checked) return;
  fw.checked = true;
  fw.dispatchEvent(new Event("change"));
  showToast(`🎮 ${chipName || "New module"} detected — fake wheel mode on so you can drive it untouched. Turn it off once the drivetrain is wired.`);
}

function applyDetectedChipToBaudUi(guessed) {
  const baudSelect = document.getElementById("cfg-baudrate");
  const hint = document.getElementById("cfg-baudrate-chip-hint");
  if (!baudSelect) return;
  const opt1500k = baudSelect.querySelector('option[value="1500000"]');
  if (guessed && typeof guessed.supportsHighBaud === "boolean") {
    if (opt1500k) {
      opt1500k.hidden = !guessed.supportsHighBaud;
      opt1500k.disabled = !guessed.supportsHighBaud;
    }
    if (!guessed.supportsHighBaud && baudSelect.value === "1500000") {
      baudSelect.value = "921600";
    }
    if (guessed.baudrate) baudSelect.value = String(guessed.baudrate);
    if (hint) {
      hint.textContent = guessed.supportsHighBaud
        ? `⚡ Detected: ${guessed.chipName} — 1.5M baud supported, suggested.`
        : `Detected: ${guessed.chipName} — 1.5M not supported by this chip, hidden.`;
      hint.style.display = "block";
    }
  } else if (guessed) {
    if (guessed.baudrate) baudSelect.value = String(guessed.baudrate);
    if (hint) hint.style.display = "none";
  }
}

/**
 * Linorobot2 Robot Configuration Engine - Client-Side App Logic
 * Pure JavaScript - 100% Client-Side Safe, Zero-Dependency
 */

// When a board is auto-detected, propose a unique robot name postfixed with
// the MCU family (e.g. rover_esp32s3) — but never overwrite a name the user
// deliberately typed or picked. Returns null when the current name looks
// intentional.
function proposeMcuRobotName(mcu) {
  const el = document.getElementById("cfg-robot-name");
  if (!el || !mcu) return null;
  const cur = (el.value || "").trim().toLowerCase();
  let presetNames = [];
  try { presetNames = Object.values(PRESETS || {}).map(p => p && p.robot_name).filter(Boolean); } catch (e) {}
  const replaceable = new Set(["", "my_robot", "robot", "scout_pico2", ...presetNames]);
  if (!replaceable.has(cur) && !/^rover_[a-z0-9]+(_\d+)?$/.test(cur)) return null;
  const base = `rover_${String(mcu).toLowerCase().replace(/[^a-z0-9]/g, "")}`;
  const taken = new Set((typeof existingConfigsList !== "undefined" ? existingConfigsList : [])
    .map(c => String(c.filename || "").replace(/_config\.h$/, "")));
  if (!taken.has(base)) return base;
  for (let i = 2; i < 50; i++) if (!taken.has(`${base}_${i}`)) return `${base}_${i}`;
  return base;
}

function applyMcuRobotName(mcu) {
  const name = proposeMcuRobotName(mcu);
  if (!name) return;
  const el = document.getElementById("cfg-robot-name");
  el.value = name;
  el.dispatchEvent(new Event("input"));
}

// MCU Pin Constraints Constants
const ESP32_STRAPPING_PINS = [0, 2, 12, 14, 15];
const ESP32_INPUT_ONLY_PINS = [34, 35, 36, 39];
const ESP32_FLASH_PINS = [6, 7, 8, 9, 10, 11];
const ESP32S3_STRAPPING_PINS = [0, 3, 45, 46];
const RP2040_ADC_PINS = [26, 27, 28, 29];

// Reference Build Presets
const PRESETS = {
  // Bare module: nothing wired. Every pin is -1 ("no connection") so the
  // firmware never drives a line that might be connected to an external
  // peripheral — including the I2C bus (see firmware.ino: SDA_PIN/SCL_PIN < 0
  // skips Wire.begin()). This is the initial state.
  bare: {
    robot_name: "bare_module", kinematics: "DIFFERENTIAL_DRIVE", mcu: "PICO2", transport: "SERIAL",
    geometry: { wheel_diameter: 0.065, track_width: 0.20, wheelbase: 0.15, weight: 2.5 },
    motors: { driver_type: "BTS7960", max_rpm: 330, cpr: 1320, operating_voltage: 12.0,
              motor1_inv: false, motor2_inv: true, motor3_inv: false, motor4_inv: true },
    sensors: { imu: "FAKE", mag: "NONE", battery_monitor: "NONE", sonar: false },
    pins: {
      led: -1,
      motor1: { pwm: -1, in_a: -1, in_b: -1, pwm_r: -1, pwm_l: -1, en: -1, dir: -1 },
      motor2: { pwm: -1, in_a: -1, in_b: -1, pwm_r: -1, pwm_l: -1, en: -1, dir: -1 },
      motor3: { pwm: -1, in_a: -1, in_b: -1, pwm_r: -1, pwm_l: -1, en: -1, dir: -1 },
      motor4: { pwm: -1, in_a: -1, in_b: -1, pwm_r: -1, pwm_l: -1, en: -1, dir: -1 },
      encoders: { m1_a: -1, m1_b: -1, m2_a: -1, m2_b: -1, m3_a: -1, m3_b: -1, m4_a: -1, m4_b: -1 },
      i2c: { sda: -1, scl: -1 }, battery_pin: -1, sonar: { trig: -1, echo: -1 }
    }
  },
  pico2_diff: {
    robot_name: "pico2_diff", kinematics: "DIFFERENTIAL_DRIVE", mcu: "PICO2", transport: "SERIAL",
    geometry: { wheel_diameter: 0.065, track_width: 0.20, wheelbase: 0.15, weight: 2.5 },
    motors: { driver_type: "BTS7960", max_rpm: 330, cpr: 1320, operating_voltage: 12.0,
              motor1_inv: false, motor2_inv: true, motor3_inv: false, motor4_inv: true },
    sensors: { imu: "FAKE", mag: "NONE", battery_monitor: "NONE", sonar: false },
    pins: {
      led: 25,
      motor1: { pwm_r: 10, pwm_l: 11, en: 12 }, motor2: { pwm_r: 13, pwm_l: 14, en: 15 },
      encoders: { m1_a: 2, m1_b: 3, m2_a: 4, m2_b: 5 },
      i2c: { sda: 0, scl: 1 }, battery_pin: 26, sonar: { trig: 22, echo: 27 }
    }
  },
  pico2_mecanum: {
    robot_name: "pico2_mecanum", kinematics: "MECANUM", mcu: "PICO2", transport: "SERIAL",
    geometry: { wheel_diameter: 0.08, track_width: 0.26, wheelbase: 0.22, weight: 4.0 },
    motors: { driver_type: "BTS7960", max_rpm: 330, cpr: 1440, operating_voltage: 12.0,
              motor1_inv: false, motor2_inv: true, motor3_inv: false, motor4_inv: true },
    sensors: { imu: "FAKE", mag: "NONE", battery_monitor: "NONE", sonar: false },
    pins: {
      led: 25,
      motor1: { pwm_r: 6, pwm_l: 7, en: 8 }, motor2: { pwm_r: 9, pwm_l: 10, en: 11 },
      motor3: { pwm_r: 12, pwm_l: 13, en: 14 }, motor4: { pwm_r: 15, pwm_l: 16, en: 17 },
      encoders: { m1_a: 0, m1_b: 1, m2_a: 2, m2_b: 3, m3_a: 4, m3_b: 5, m4_a: 18, m4_b: 19 },
      i2c: { sda: 20, scl: 21 }, battery_pin: 26, sonar: { trig: 22, echo: 27 }
    }
  },
  pico2_skid: {
    robot_name: "pico2_skid", kinematics: "SKID_STEER", mcu: "PICO2", transport: "SERIAL",
    geometry: { wheel_diameter: 0.08, track_width: 0.24, wheelbase: 0.20, weight: 4.0 },
    motors: { driver_type: "BTS7960", max_rpm: 330, cpr: 1440, operating_voltage: 12.0,
              motor1_inv: false, motor2_inv: true, motor3_inv: false, motor4_inv: true },
    sensors: { imu: "FAKE", mag: "NONE", battery_monitor: "NONE", sonar: false },
    pins: {
      led: 25,
      motor1: { pwm_r: 6, pwm_l: 7, en: 8 }, motor2: { pwm_r: 9, pwm_l: 10, en: 11 },
      motor3: { pwm_r: 12, pwm_l: 13, en: 14 }, motor4: { pwm_r: 15, pwm_l: 16, en: 17 },
      encoders: { m1_a: 0, m1_b: 1, m2_a: 2, m2_b: 3, m3_a: 4, m3_b: 5, m4_a: 18, m4_b: 19 },
      i2c: { sda: 20, scl: 21 }, battery_pin: 26, sonar: { trig: 22, echo: 27 }
    }
  },
  scout_pico2w: {
    robot_name: "scout_pico2w",
    kinematics: "DIFFERENTIAL_DRIVE",
    mcu: "PICO2W",
    transport: "SERIAL",
    wifi_telemetry: false,
    geometry: { wheel_diameter: 0.08, track_width: 0.22, wheelbase: 0.20, weight: 3.5 },
    motors: { driver_type: "BTS7960", max_rpm: 330, cpr: 1320, motor1_inv: false, motor2_inv: true },
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
    robot_name: "waveshare_gendrv",
    kinematics: "DIFFERENTIAL_DRIVE",
    mcu: "GENDRV",
    transport: "SERIAL",
    baudrate: 1500000,
    dual_core: true,
    geometry: {
      wheel_diameter: 0.065,
      track_width: 0.20,
      wheelbase: 0.18
    },
    motors: {
      driver_type: "BTS7960",
      max_rpm: 330,
      cpr: 1320,
      operating_voltage: 12.0,
      motor1_inv: false,
      motor2_inv: false,
      motor3_inv: false,
      motor4_inv: false
    },
    sensors: {
      imu: "QMI8658",
      mag: "AK09918",
      battery_monitor: "INA219",
      sonar: false
    },
    pins: {
      led: -1,
      motor1: { in_a: 17, in_b: 21 },
      motor2: { in_a: 23, in_b: 22 },
      encoders: { m1_a: 34, m1_b: 35, m2_a: 16, m2_b: 27 },
      i2c: { sda: 32, scl: 33 },
      battery_pin: -1,
      sonar: { trig: -1, echo: -1 }
    }
  },

  // ── Simulation-first designs ────────────────────────────────────────────
  // A board on a desk with nothing wired to it, that still drives and maps.
  // Both run fake wheel mode and the fake LD19, so cmd_vel produces odometry
  // and /scan, and Console can go straight to teleop, SLAM and Nav2.
  gendrv_serial_ld19: {
    robot_name: "gendrv_serial_ld19",
    kinematics: "DIFFERENTIAL_DRIVE",
    mcu: "GENDRV",
    transport: "SERIAL",
    baudrate: 1500000,
    // Off, and stated here rather than left to the generator to suppress: the
    // fake LiDAR drives the simulated pose and its UART from the loop while a
    // dual-core control task drives the same pose from the other core, with no
    // lock between them, and the board crash-loops. Anyone reading this design
    // should see the choice, not discover it from the generated header.
    dual_core: false,
    geometry: { wheel_diameter: 0.065, track_width: 0.20, wheelbase: 0.18, weight: 3.5 },
    motors: {
      driver_type: "BTS7960", max_rpm: 330, cpr: 1320, operating_voltage: 12.0,
      motor1_inv: false, motor2_inv: false, motor3_inv: false, motor4_inv: false
    },
    sensors: { imu: "QMI8658", mag: "AK09918", battery_monitor: "INA219", sonar: false },
    // Spelled out rather than left to the form defaults. A reference design
    // that relies on defaults silently changes when they do -- which is how
    // this one shipped a 6x4 m room after the default grew to 16x12.
    simulation: {
      fake_wheel: true, fake_ld19: true, fake_sonar: true,
      map_width: 10.0, map_height: 6.0,
      wall_obstacle: true, wall_x1: 2.0, wall_y1: -1.5, wall_x2: 2.0, wall_y2: 1.5
    },
    pins: {
      led: -1,
      motor1: { in_a: 17, in_b: 21 },
      motor2: { in_a: 23, in_b: 22 },
      encoders: { m1_a: 34, m1_b: 35, m2_a: 16, m2_b: 27 },
      i2c: { sda: 32, scl: 33 },
      battery_pin: -1,
      // IO4 is the gendrv's LiDAR data line and is wired on the reference
      // board, so the emulated scan has a route out over this UART -- the same
      // line a real LD19 would feed in on. On a board where it is not wired,
      // clear this to -1: the generator then leaves fake LD19 disabled rather
      // than writing packets into an unconnected pad. (The WiFi design needs no
      // pin at all: it sends the scan over UDP.)
      lidar_rxd: 4,
      sonar: { trig: -1, echo: -1 }
    }
  },

  esp32_wifi_ld19: {
    robot_name: "esp32_wifi_ld19",
    kinematics: "DIFFERENTIAL_DRIVE",
    mcu: "ESP32",
    transport: "WIFI_UDP",
    wifi_settings: {
      ssid: "YOUR_WIFI_SSID",
      password: "YOUR_WIFI_PASSWORD",
      agent_ip: "192.168.1.100",
      agent_port: 8888
    },
    // The simulated scan is delivered over WiFi UDP. That is the whole point of
    // this design: no LiDAR UART pin, and no second cable to the board.
    telemetry: { use_lidar_udp: true },
    // Same reason as the serial design: simulation and a dual-core control
    // task race on the pose with no lock and crash-loop the board.
    dual_core: false,
    geometry: { wheel_diameter: 0.065, track_width: 0.20, wheelbase: 0.15, weight: 3.5 },
    motors: {
      driver_type: "GENERIC_2_IN", max_rpm: 300, cpr: 1500, operating_voltage: 12.0,
      motor1_inv: false, motor2_inv: true, motor3_inv: false, motor4_inv: true
    },
    sensors: { imu: "FAKE", mag: "NONE", battery_monitor: "NONE", sonar: false },
    // Spelled out rather than left to the form defaults. A reference design
    // that relies on defaults silently changes when they do -- which is how
    // this one shipped a 6x4 m room after the default grew to 16x12.
    simulation: {
      fake_wheel: true, fake_ld19: true, fake_sonar: true,
      map_width: 10.0, map_height: 6.0,
      wall_obstacle: true, wall_x1: 2.0, wall_y1: -1.5, wall_x2: 2.0, wall_y2: 1.5
    },
    pins: {
      led: 2,
      motor1: { pwm: 13, in_a: 14, in_b: 27 },
      motor2: { pwm: 25, in_a: 26, in_b: 33 },
      encoders: { m1_a: 34, m1_b: 35, m2_a: 36, m2_b: 39 },
      i2c: { sda: 21, scl: 22 },
      battery_pin: -1,
      // no lidar_rxd: this design ships the scan over WiFi UDP, so no UART pin
      // is involved and none needs wiring
      sonar: { trig: -1, echo: -1 }
    }
  }
};

// Only the Waveshare General Driver board has fixed, known wiring. Every other
// preset is a DIY build with no committed pinout, so its pins are forced to
// bare (-1 / "no connection") — a preset must never drive a pin the user has
// not actually wired. Kinematics, geometry, driver and sensor picks are kept.
const PINNED_PRESETS = ["bare", "waveshare_gendrv", "gendrv_serial_ld19", "esp32_wifi_ld19"];
for (const [key, p] of Object.entries(PRESETS)) {
  if (!PINNED_PRESETS.includes(key)) {
    p.pins = JSON.parse(JSON.stringify(PRESETS.bare.pins));
  }
  // Every reference design starts in fake wheel mode. A design is a starting
  // point for a robot that does not exist yet, so the board almost never has
  // motors and encoders wired when it is first flashed -- and without this it
  // would report zero RPM, leaving teleop, SLAM and Nav2 dead on arrival.
  // Turn it off in the Simulation section once the real drivetrain is wired.
  p.simulation = Object.assign({ fake_wheel: true }, p.simulation || {});
}


// Any manual toggle of the simulation controls is an explicit choice, and must
// not be overridden the next time a module is detected.
for (const id of ["cfg-fake-wheel", "cfg-fake-ld19"]) {
  const el = document.getElementById(id);
  if (el) el.addEventListener("change", () => {
    userTouchedSimulation = true;
    updateSimulationBanner();
  });
}

// Simulation mode is easy to forget once it is on -- the config looks normal
// and the board reports plausible odometry, so a real robot flashed with it
// would sit still while the topics claimed it was driving. Keep a standing
// banner up for as long as any simulated source is enabled.
function updateSimulationBanner() {
  const banner = document.getElementById("sim-mode-banner");
  if (!banner) return;
  const wheel = document.getElementById("cfg-fake-wheel")?.checked;
  const lidar = document.getElementById("cfg-fake-ld19")?.checked;
  const parts = [];
  if (wheel) parts.push("wheels and encoders are simulated");
  if (lidar) parts.push("the LiDAR scan is simulated");
  banner.style.display = (wheel || lidar) ? "flex" : "none";
  const what = document.getElementById("sim-mode-what");
  if (what) what.textContent = parts.join(", and ");
}

const btnSimOff = document.getElementById("btn-sim-banner-off");
if (btnSimOff) {
  btnSimOff.addEventListener("click", () => {
    for (const id of ["cfg-fake-wheel", "cfg-fake-ld19"]) {
      const el = document.getElementById(id);
      if (el && el.checked) {
        el.checked = false;
        el.dispatchEvent(new Event("change"));
      }
    }
    userTouchedSimulation = true;
    updateSimulationBanner();
    showToast("Simulation off — this config now drives the real motors and encoders.");
  });
}

// Current Active Spec State
let currentSpec = {};
let activeArtifact = "tab-code-header";

function isESP32(mcu) {
  return typeof mcu === "string" && (mcu.startsWith("ESP32") || mcu === "GENDRV");
}

// Only the classic ESP32, the ESP32-S2 and the ESP32-based Waveshare General
// Driver carry a true hardware DAC — required for the adc_calibrate sweep.
// ESP32-S3 / C3 / C6, RP2040/RP2350 and Teensy have none.
function mcuHasDac(mcu) {
  return mcu === "ESP32" || mcu === "ESP32S2" || mcu === "GENDRV";
}

// Valid DAC GPIOs per MCU: ESP32 -> 25/26, ESP32-S2 -> 17/18.
function dacPinsForMcu(mcu) {
  return mcu === "ESP32S2" ? [17, 18] : [25, 26];
}

// DOM Elements Initialization
document.addEventListener("DOMContentLoaded", async () => {
  initNavTabs();
  initArtifactTabs();
  initEventListeners();
  initAutomationEventListeners();
  initWorkflowHeaderListeners();
  loadRemoteConfig();
  initAdcCalibrationStudio();
  initRos2TopicInspector();
  detectClientOS();
  checkServerRunnerStatus();
  initGitVersionBadge();
  initBranchPicker();
  handleUrlParams();
  const restored = await restoreWorkflowAndBoardSetup();
  if (!restored) {
    // A first-time visitor has no robot yet, so open on the simulated design:
    // flash it and Console can drive, map and navigate with nothing wired.
    loadPreset("gendrv_serial_ld19");
  }
  // A restored session bypasses the preset defaults entirely, so the banner has
  // to be brought up from whatever state the form actually ended in.
  updateSimulationBanner();
});


// =============================================================================
// Workflow & Board Setup Persistence (Browser localStorage + Server-side .workflow_settings.json)
// =============================================================================
let saveWorkflowDebounceTimer = null;
let isRestoringSettings = false;

function updateRemoteBadge(val) {
  const badge = document.getElementById("hdr-remote-badge");
  if (!badge) return;
  const host = (val || "").trim();
  if (host) {
    badge.className = "badge-pill badge-ok";
    badge.textContent = `🤖 ${host.includes("@") ? host.split("@")[1] : host}`;
    badge.title = `Remote Target: ${host}`;
  } else {
    badge.className = "badge-pill badge-ok";
    badge.textContent = "💻 Local";
    badge.title = "Target: Local Machine";
  }
}

function saveWorkflowAndBoardSetup() {
  if (isRestoringSettings) return;
  clearTimeout(saveWorkflowDebounceTimer);
  saveWorkflowDebounceTimer = setTimeout(async () => {
    try {
      const rosDistro = document.getElementById("hdr-ros-distro")?.value || "jazzy";
      const buildEngine = document.getElementById("hdr-build-engine")?.value || "docker";
      const agentEngine = document.getElementById("hdr-agent-engine")?.value || "docker";
      const containerRegistry = document.getElementById("hdr-container-registry")?.value || "auto";
      const customRegistry = (document.getElementById("hdr-custom-registry")?.value || "").trim();
      const remoteFlash = (document.getElementById("hdr-remote-flash")?.value || "").trim();
      const flashPort = (document.getElementById("auto-flash-port")?.value || "").trim();
      const multiPorts = (document.getElementById("auto-agent-multiserial-ports")?.value || "").trim();
      const robotName = (document.getElementById("cfg-robot-name")?.value || "").trim();
      const branch = (document.getElementById("auto-git-branch")?.value || "").trim();
      const preset = document.getElementById("preset-select")?.value || "bare";
      
      const payload = {
        ros_distro: rosDistro,
        build_engine: buildEngine,
        agent_engine: agentEngine,
        container_registry: containerRegistry,
        custom_registry: customRegistry,
        remote_flash: remoteFlash,
        flash_port: flashPort,
        multi_ports: multiPorts,
        robot_name: robotName,
        branch: branch,
        preset: preset,
        spec: currentSpec || null
      };

      try {
        localStorage.setItem("linorobot2_workflow_settings", JSON.stringify(payload));
      } catch (e) {}

      await fetch("/api/workflow/settings", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      });
    } catch (err) {
      console.warn("Failed to persist workflow settings:", err);
    }
  }, 350);
}

async function restoreWorkflowAndBoardSetup() {
  isRestoringSettings = true;
  window.hasAutoDetectedHardware = true;
  let settings = null;

  try {
    const res = await fetch("/api/workflow/settings");
    if (res.ok) {
      const data = await res.json();
      if (data.status === "ok" && data.settings) {
        settings = data.settings;
      }
    }
  } catch (e) {
    console.warn("Server workflow settings fetch failed, checking localStorage", e);
  }

  if (!settings || !settings.ros_distro) {
    try {
      const localStr = localStorage.getItem("linorobot2_workflow_settings");
      if (localStr) settings = JSON.parse(localStr);
    } catch (e) {}
  }

  if (!settings) return;

  if (settings.ros_distro) {
    const hdrRos = document.getElementById("hdr-ros-distro");
    if (hdrRos) hdrRos.value = settings.ros_distro;
    const autoRos = document.getElementById("auto-ros-distro");
    if (autoRos) autoRos.value = settings.ros_distro;
  }

  if (settings.build_engine) {
    const hdrBuild = document.getElementById("hdr-build-engine");
    if (hdrBuild) hdrBuild.value = settings.build_engine;
    const chkDocker = document.getElementById("chk-docker-builder");
    if (chkDocker) chkDocker.checked = (settings.build_engine !== "native");
  }

  if (settings.agent_engine) {
    const hdrAgent = document.getElementById("hdr-agent-engine");
    if (hdrAgent) hdrAgent.value = settings.agent_engine;
  }

  if (settings.container_registry) {
    const regMode = (settings.container_registry === "custom" || (!["auto", "cluster", "dockerhub"].includes(settings.container_registry))) ? "custom" : settings.container_registry;
    const customVal = settings.custom_registry || (!["auto", "cluster", "dockerhub"].includes(settings.container_registry) ? settings.container_registry : "");
    const hdrReg = document.getElementById("hdr-container-registry");
    const cfgReg = document.getElementById("cfg-container-registry");
    if (hdrReg) hdrReg.value = regMode;
    if (cfgReg) cfgReg.value = regMode;
    const hdrCustom = document.getElementById("hdr-custom-registry");
    const cfgCustom = document.getElementById("cfg-custom-registry");
    const grpCustom = document.getElementById("group-cfg-custom-registry");
    if (hdrCustom) {
      hdrCustom.value = customVal;
      hdrCustom.style.display = (regMode === "custom" || regMode === "cluster") ? "inline-block" : "none";
    }
    if (cfgCustom) cfgCustom.value = customVal;
    if (grpCustom) grpCustom.style.display = (regMode === "custom" || regMode === "cluster") ? "block" : "none";
  }

  if (settings.remote_flash !== undefined) {
    const hdrRemote = document.getElementById("hdr-remote-flash");
    if (hdrRemote) hdrRemote.value = settings.remote_flash;
    const sshHost = document.getElementById("remote-ssh-host");
    if (sshHost) sshHost.value = settings.remote_flash;
    updateRemoteBadge(settings.remote_flash);
    if (typeof updateRemoteUI === "function") updateRemoteUI();
  }



  const hasSavedSpec = settings.spec && typeof settings.spec === "object"
    && Object.keys(settings.spec).length > 0;

  if (settings.preset && document.getElementById("preset-select")) {
    document.getElementById("preset-select").value = settings.preset;
    // Only the dropdown was being restored. With no saved spec to populate
    // from, the form stayed on raw HTML defaults while the dropdown claimed a
    // design was loaded -- so the fields never matched the build named above
    // them. Load the design for real instead.
    if (!hasSavedSpec) loadPreset(settings.preset);
  }

  if (hasSavedSpec) {
    currentSpec = settings.spec;
    if (typeof populateFormFromSpec === "function") {
      populateFormFromSpec(currentSpec);
      if (typeof recomputeAll === "function") recomputeAll();
    }
  }

  if (settings.robot_name) {
    const nameInput = document.getElementById("cfg-robot-name");
    if (nameInput) nameInput.value = settings.robot_name;
    if (currentSpec) currentSpec.robot_name = settings.robot_name;
  }
  if (settings.branch) {
    const branchInput = document.getElementById("auto-git-branch");
    if (branchInput) {
      branchInput.value = settings.branch;
      branchInput.dataset.autoManaged = "false";
    }
  }

  if (settings.flash_port && document.getElementById("auto-flash-port")) {
    const pEl = document.getElementById("auto-flash-port");
    pEl.value = settings.flash_port;
    pEl.dataset.autoManaged = "false";
  }

  if (settings.multi_ports && document.getElementById("auto-agent-multiserial-ports")) {
    const mpEl = document.getElementById("auto-agent-multiserial-ports");
    mpEl.value = settings.multi_ports;
    mpEl.dataset.autoManaged = "false";
  }

  if (typeof updateAutomationPreviews === "function") {
    updateAutomationPreviews();
  }
  if (settings.flash_port && document.getElementById("auto-flash-port")) {
    document.getElementById("auto-flash-port").value = settings.flash_port;
    document.getElementById("auto-flash-port").dataset.autoManaged = "false";
  }
  setTimeout(() => { isRestoringSettings = false; }, 250);
  return true;
}

function initWorkflowHeaderListeners() {
  const hdrRos = document.getElementById("hdr-ros-distro");
  const hdrBuild = document.getElementById("hdr-build-engine");
  const hdrAgent = document.getElementById("hdr-agent-engine");
  const hdrReg = document.getElementById("hdr-container-registry");
  const hdrCustomReg = document.getElementById("hdr-custom-registry");
  const cfgReg = document.getElementById("cfg-container-registry");
  const cfgCustomReg = document.getElementById("cfg-custom-registry");
  const grpCustomReg = document.getElementById("group-cfg-custom-registry");
  const hdrRemote = document.getElementById("hdr-remote-flash");

  const syncRegistryUI = (val, customVal) => {
    // `cluster` and `custom` both take a user-supplied host:port
    const isCustom = (val === "custom" || val === "cluster");
    if (hdrReg) hdrReg.value = val;
    if (cfgReg) cfgReg.value = val;
    if (hdrCustomReg) {
      if (customVal !== undefined) hdrCustomReg.value = customVal;
      hdrCustomReg.style.display = isCustom ? "inline-block" : "none";
      if (isCustom) hdrCustomReg.focus();
    }
    if (cfgCustomReg) {
      if (customVal !== undefined) cfgCustomReg.value = customVal;
    }
    if (grpCustomReg) grpCustomReg.style.display = isCustom ? "block" : "none";
  };

  if (hdrReg) {
    hdrReg.addEventListener("change", () => {
      syncRegistryUI(hdrReg.value);
      updateAutomationPreviews();
      saveWorkflowAndBoardSetup();
      showToast(`📦 Registry: ${hdrReg.options[hdrReg.selectedIndex]?.text || hdrReg.value}`);
    });
  }

  if (cfgReg) {
    cfgReg.addEventListener("change", () => {
      syncRegistryUI(cfgReg.value);
      updateAutomationPreviews();
      saveWorkflowAndBoardSetup();
    });
  }

  if (hdrCustomReg) {
    hdrCustomReg.addEventListener("input", () => {
      if (cfgCustomReg) cfgCustomReg.value = hdrCustomReg.value;
      updateAutomationPreviews();
      saveWorkflowAndBoardSetup();
    });
  }

  if (cfgCustomReg) {
    cfgCustomReg.addEventListener("input", () => {
      if (hdrCustomReg) hdrCustomReg.value = cfgCustomReg.value;
      updateAutomationPreviews();
      saveWorkflowAndBoardSetup();
    });
  }
  const btnRootless = document.getElementById("btn-rootless-info");
  const modalRootless = document.getElementById("modal-rootless-docker");

  if (hdrRos) {
    hdrRos.addEventListener("change", () => {
      const autoRos = document.getElementById("auto-ros-distro");
      if (autoRos) autoRos.value = hdrRos.value;
      updateAutomationPreviews();
      saveWorkflowAndBoardSetup();
      showToast(`🌐 ROS 2 Distro: ${hdrRos.options[hdrRos.selectedIndex]?.text || hdrRos.value}`);
    });
  }

  const autoRos = document.getElementById("auto-ros-distro");
  if (autoRos) {
    autoRos.addEventListener("change", () => {
      if (hdrRos) hdrRos.value = autoRos.value;
      saveWorkflowAndBoardSetup();
    });
  }

  if (hdrBuild) {
    hdrBuild.addEventListener("change", () => {
      const chkDocker = document.getElementById("chk-docker-builder");
      if (chkDocker) chkDocker.checked = (hdrBuild.value !== "native");
      const dockerDetails = document.getElementById("docker-builder-details");
      if (dockerDetails) dockerDetails.style.display = (hdrBuild.value === "docker") ? "block" : "none";
      updateAutomationPreviews();
      saveWorkflowAndBoardSetup();
      showToast(`🔨 Build Engine: ${hdrBuild.value.toUpperCase()}`);
      ensureContainerEngine(hdrBuild.value);
    });
  }

  if (hdrAgent) {
    hdrAgent.addEventListener("change", () => {
      updateAutomationPreviews();
      saveWorkflowAndBoardSetup();
      showToast(`🤖 micro-ROS Agent: ${hdrAgent.options[hdrAgent.selectedIndex]?.text || hdrAgent.value}`);
      ensureContainerEngine(hdrAgent.value);
    });
  }

  if (hdrRemote) {
    let remoteDebounce = null;
    hdrRemote.addEventListener("input", () => {
      clearTimeout(remoteDebounce);
      const val = hdrRemote.value.trim();
      updateRemoteBadge(val);
      const hostInput = document.getElementById("remote-ssh-host");
      if (hostInput) {
        if (val.includes("@")) {
          const parts = val.split("@");
          const userInput = document.getElementById("remote-ssh-user");
          if (userInput) userInput.value = parts[0];
          hostInput.value = parts[1];
        } else {
          hostInput.value = val;
        }
      }
      remoteDebounce = setTimeout(() => {
        saveWorkflowAndBoardSetup();
        if (typeof testRemoteConnection === "function") testRemoteConnection();
      }, 500);
    });
  }

  // Rootless modal listeners
  if (btnRootless && modalRootless) {
    btnRootless.addEventListener("click", () => {
      openRootlessModal();
    });

    const btnClose1 = document.getElementById("btn-close-rootless-modal");
    const btnClose2 = document.getElementById("btn-close-rootless-modal-ok");
    if (btnClose1) btnClose1.addEventListener("click", () => modalRootless.style.display = "none");
    if (btnClose2) btnClose2.addEventListener("click", () => modalRootless.style.display = "none");

    modalRootless.addEventListener("click", (e) => {
      if (e.target === modalRootless) modalRootless.style.display = "none";
    });

    const btnCopyUbu = document.getElementById("btn-copy-rootless-ubuntu");
    if (btnCopyUbu) {
      btnCopyUbu.addEventListener("click", () => {
        const text = document.getElementById("code-rootless-ubuntu")?.innerText || "";
        navigator.clipboard.writeText(text).then(() => showToast("📋 Rootless Docker setup script copied!"));
      });
    }

    const btnCopyPod = document.getElementById("btn-copy-rootless-podman");
    if (btnCopyPod) {
      btnCopyPod.addEventListener("click", () => {
        const text = document.getElementById("code-rootless-podman")?.innerText || "";
        navigator.clipboard.writeText(text).then(() => showToast("📋 Podman commands copied!"));
      });
    }

    const btnTestDaemon = document.getElementById("btn-test-rootless-daemon");
    if (btnTestDaemon) {
      btnTestDaemon.addEventListener("click", async () => {
        const statusText = document.getElementById("rootless-status-text");
        if (statusText) statusText.textContent = "Checking daemon socket...";
        try {
          const res = await fetch("/api/docker/status");
          const d = await res.json();
          if (d.has_docker) {
            const rootlessNote = d.is_rootless_docker ? " (Rootless active ✅)" : " (Standard rootful — rootless recommended for Linux)";
            if (statusText) statusText.textContent = `✅ Docker Engine active${rootlessNote}`;
            showToast(`Docker: Active${rootlessNote}`);
          } else if (d.has_podman) {
            if (statusText) statusText.textContent = "✅ Podman active (Rootless by default)";
            showToast("Podman: Active (Rootless)");
          } else {
            if (statusText) statusText.textContent = "❌ Neither Docker nor Podman detected on host.";
            showToast("No container engine found.");
          }
        } catch (e) {
          if (statusText) statusText.textContent = "❌ Error contacting server: " + e.message;
        }
      });
    }
  }
}


// Automated Rootless Docker & Container Engine Setup Helpers
async function triggerRootlessDockerSetup(isManual = false) {
  const statusText = document.getElementById("rootless-status-text");
  const btnSetup = document.getElementById("btn-setup-rootless-docker");
  if (statusText) statusText.textContent = "⚡ Configuring Rootless Docker daemon...";
  if (btnSetup) {
    btnSetup.disabled = true;
    btnSetup.textContent = "Setting up...";
  }

  try {
    const res = await fetch("/api/docker/setup_rootless", { method: "POST" }).then(r => r.json());
    if (statusText) {
      if (res.success || res.is_rootless) {
        statusText.textContent = `✅ ${res.message || "Rootless Docker active!"}`;
        if (!isManual) showToast("🐳 Rootless Docker was automatically configured for this session.");
      } else {
        statusText.textContent = `⚠️ ${res.message || "Setup completed with warnings. Check logs."}`;
      }
    }
    return res;
  } catch (err) {
    if (statusText) statusText.textContent = `⚠️ Setup error: ${err.message}`;
    return { success: false, error: err.message };
  } finally {
    if (btnSetup) {
      btnSetup.disabled = false;
      btnSetup.textContent = "⚡ Setup Rootless Now";
    }
  }
}

async function checkAndAutoSetupRootlessDocker() {
  try {
    const status = await fetch("/api/docker/status").then(r => r.json());
    if (status.platform_system === "Linux" && status.has_docker && !status.is_rootless_docker) {
      console.log("[Robot Config Engine] Auto-configuring rootless Docker...");
      await triggerRootlessDockerSetup(false);
    }
  } catch (e) {}
}

async function ensureContainerEngine(engine) {
  if (engine === "native") return { installed: true };
  const targetEngine = (engine === "podman" || engine === "podman_systemd") ? "podman" : "docker";

  try {
    const status = await fetch("/api/docker/status").then(r => r.json());
    if (targetEngine === "podman" && !status.has_podman) {
      showToast("🦭 Podman not found on system. Installing automatically...", 5000);
      const res = await fetch("/api/container/install", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ engine: "podman" })
      }).then(r => r.json());
      if (res.installed) {
        showToast("✅ Podman installed successfully!", 4000);
      } else {
        showToast("⚠️ Podman auto-install failed: " + res.message, 6000);
      }
      return res;
    } else if (targetEngine === "docker") {
      if (!status.has_docker) {
        showToast("🐳 Docker not found on system. Installing Rootless Docker automatically...", 6000);
        const res = await fetch("/api/container/install", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ engine: "docker" })
        }).then(r => r.json());
        if (res.installed) {
          showToast("✅ Docker (Rootless) installed and configured!", 4000);
        } else {
          showToast("⚠️ Docker auto-install failed: " + res.message, 6000);
        }
        return res;
      } else if (!status.is_rootless_docker && status.platform_system === "Linux") {
        await triggerRootlessDockerSetup(false);
      }
    }
  } catch (e) {
    console.warn("Container auto-check error:", e);
  }
}


async function openRootlessModal() {
  const modal = document.getElementById("modal-rootless-docker");
  if (!modal) return;
  modal.style.display = "flex";
  const statusText = document.getElementById("rootless-status-text");
  try {
    const res = await fetch("/api/docker/rootless_info");
    const info = await res.json();
    if (statusText) {
      if (info.is_rootless) {
        statusText.textContent = `✅ Rootless Docker is active for user '${info.user}' (UID ${info.uid})`;
      } else if (info.has_docker) {
        statusText.textContent = `⚠️ Docker is running in standard (rootful) mode. Run setup below to enable rootless daemon.`;
      } else if (info.has_podman) {
        statusText.textContent = `✅ Podman is available (Rootless by default, no daemon needed).`;
      } else {
        statusText.textContent = `ℹ️ Neither Docker nor Podman found. Follow setup below to install.`;
      }
    }
  } catch (e) {
    if (statusText) statusText.textContent = "ℹ️ Container info unavailable";
  }
}

async function checkAndWarnRootlessDocker() {
  try {
    const res = await fetch("/api/docker/status");
    const d = await res.json();
    if (d.has_docker && d.platform_system === "Linux" && !d.is_rootless_docker) {
      showToast("💡 Tip: On Linux, click '❓ Guide' to setup rootless Docker and avoid root-owned files.", 6000);
    }
  } catch (e) {}
}

// Mirror the Robot SBC's current git branch into the Tab 5 merge/commit
// branch field until the user edits it. "Run Merge & Commit" then checks that
// branch out (creating it if new).
function applyCurrentBranch(branch) {
  const el = document.getElementById("auto-git-branch");
  if (!el || !branch || branch === "(detached)") return;
  if (el.dataset.autoManaged === "false") return;
  el.value = branch;
  el.dataset.autoManaged = "true";
}

// =============================================================================
// Header Branch Picker — clicking the "Branch:" field (or its ▾) drops a list
// of the Robot SBC's local git branches (GET /api/gitinfo → branches[]).
// Picking one fills the field; typing a new name still creates it on Merge/Commit.
// =============================================================================
function initBranchPicker() {
  const input = document.getElementById("auto-git-branch");
  const caret = document.getElementById("btn-branch-menu");
  const menu = document.getElementById("branch-menu");
  if (!input || !menu) return;

  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => (
    { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]
  ));

  const close = () => {
    menu.hidden = true;
    if (caret) caret.setAttribute("aria-expanded", "false");
  };

  const pick = (name) => {
    input.value = name;
    input.dataset.autoManaged = "false";   // user chose it — stop mirroring HEAD
    input.dispatchEvent(new Event("input", { bubbles: true }));
    input.dispatchEvent(new Event("change", { bubbles: true }));
    if (typeof updateAutomationPreviews === "function") updateAutomationPreviews();
    close();
    // Check the branch out on the Robot SBC right now (same command the
    // "= name" button uses: create if missing, otherwise just switch).
    const cmd = [
      `set -e`,
      `if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then echo "Not a git repository."; exit 1; fi`,
      `git checkout -b "${name}" 2>/dev/null || git checkout "${name}"`,
      `echo "Now on branch: $(git rev-parse --abbrev-ref HEAD)"`,
    ].join("\n");
    executeCommandInTerminal(cmd, `Switching to Git branch '${name}'`);
  };

  const render = (info) => {
    const branches = (info && Array.isArray(info.branches)) ? info.branches : [];
    const cur = info && info.branch;
    const typed = input.value.trim();
    if (!branches.length) {
      menu.innerHTML = `<div class="branch-empty">No branches reported by the Robot SBC.</div>`;
      return;
    }
    menu.innerHTML = branches.map((b) => `
      <button type="button" role="option" class="branch-item${b === cur ? " is-current" : ""}${b === typed ? " is-active" : ""}" data-branch="${esc(b)}">
        <span class="branch-cur-dot"></span><span>${esc(b)}</span>${b === cur ? '<span style="margin-left:auto;font-size:0.68rem;color:var(--text-muted)">current</span>' : ""}
      </button>`).join("");
    menu.querySelectorAll(".branch-item").forEach((btn) => {
      btn.addEventListener("click", () => pick(btn.dataset.branch));
    });
  };

  const open = async () => {
    menu.hidden = false;
    if (caret) caret.setAttribute("aria-expanded", "true");
    menu.innerHTML = `<div class="branch-empty">Loading branches…</div>`;
    let info = null;
    try {
      const res = await fetch("/api/gitinfo", { cache: "no-cache" });
      if (res.ok) info = await res.json();
    } catch (_) { /* offline: fall through to empty */ }
    render(info);
  };

  const toggle = (e) => {
    if (e) e.stopPropagation();
    if (menu.hidden) open(); else close();
  };

  input.addEventListener("click", (e) => { e.stopPropagation(); if (menu.hidden) open(); });
  input.addEventListener("keydown", (e) => {
    if ((e.key === "ArrowDown" || e.key === "ArrowUp") && menu.hidden) { e.preventDefault(); open(); }
  });
  if (caret) caret.addEventListener("click", toggle);
  // Re-filter the visible list as the user types a name.
  input.addEventListener("input", () => {
    if (menu.hidden) return;
    menu.querySelectorAll(".branch-item").forEach((btn) => {
      const hit = btn.dataset.branch.toLowerCase().includes(input.value.trim().toLowerCase());
      btn.style.display = hit ? "" : "none";
      btn.classList.toggle("is-active", btn.dataset.branch === input.value.trim());
    });
  });
  document.addEventListener("click", (e) => {
    if (!menu.hidden && !menu.contains(e.target) && e.target !== input && e.target !== caret) close();
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && !menu.hidden) { close(); input.blur(); }
  });
}

// =============================================================================
// Header Git Version Badge — shows the 7-char commit the web server booted on;
// click to reveal the branch, remotes and last 10 commits (GET /api/gitinfo).
// =============================================================================
function initGitVersionBadge() {
  const badge = document.getElementById("git-version-badge");
  const text = document.getElementById("git-version-text");
  const popover = document.getElementById("git-version-popover");
  if (!badge || !text || !popover) return;

  const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g, (c) => (
    { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]
  ));

  let loaded = null;

  const render = (info) => {
    const remotes = (info.remotes || []).map((r) => `
      <div class="gv-line">
        <span class="gv-remote-name">${esc(r.name)}</span>
        <span class="gv-val">${esc(r.url)}</span>
      </div>`).join("") || `<div class="gv-line"><span class="gv-val">(no remotes)</span></div>`;

    const commits = (info.commits || []).map((c) => `
      <li>
        <div><span class="gv-hash">${esc(c.hash)}</span> <span class="gv-subject">${esc(c.subject)}</span></div>
        <div class="gv-meta">${esc(c.author)} · ${esc(c.date)} (${esc(c.reldate)})</div>
      </li>`).join("") || `<li><span class="gv-meta">(no commit history)</span></li>`;

    const movedNote = info.moved_since_start
      ? `<div class="gv-note">⚠ HEAD is now at <code>${esc(info.version)}</code> — the server is still running the <code>${esc(info.version_at_start)}</code> build. Restart server.py to pick up the new code.</div>`
      : "";
    const dirtyNote = info.dirty
      ? `<div class="gv-note">Working tree has uncommitted changes.</div>`
      : "";

    popover.innerHTML = `
      <h4>Version</h4>
      <div class="gv-line"><span class="gv-key">server @</span><span class="gv-val">${esc(info.version_at_start)}</span></div>
      <div class="gv-line"><span class="gv-key">branch</span><span class="gv-val">${esc(info.branch)}</span></div>
      <h4>Remotes</h4>
      ${remotes}
      <h4>Last 10 commits</h4>
      <ol class="gv-commits">${commits}</ol>
      ${movedNote}
      ${dirtyNote}`;
  };

  const load = async () => {
    try {
      const res = await fetch("/api/gitinfo", { cache: "no-cache" });
      if (!res.ok) return null;
      return await res.json();
    } catch (e) {
      return null;
    }
  };

  const closePopover = () => {
    popover.hidden = true;
    badge.setAttribute("aria-expanded", "false");
  };

  const openPopover = async () => {
    // Always re-fetch: the branch and recent commits change during a session
    // whenever a robot-config Full Deploy writes a config commit / branch.
    const fresh = await load();
    if (fresh) { loaded = fresh; render(loaded); applyCurrentBranch(fresh.branch); }
    if (!loaded) return;
    popover.hidden = false;
    badge.setAttribute("aria-expanded", "true");
  };

  badge.addEventListener("click", (e) => {
    e.stopPropagation();
    if (popover.hidden) openPopover();
    else closePopover();
  });
  document.addEventListener("click", (e) => {
    if (!popover.hidden && !popover.contains(e.target) && e.target !== badge) closePopover();
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && !popover.hidden) closePopover();
  });

  // Prime the badge label at startup.
  load().then((info) => {
    if (!info) { text.textContent = "no-git"; return; }
    loaded = info;
    render(info);
    text.textContent = info.version_at_start || "unknown";
    if (info.dirty || info.moved_since_start) badge.classList.add("is-dirty");
    applyCurrentBranch(info.branch);
  });
}

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

  // Header "= name" button: point the branch field at the robot name and
  // switch the Robot SBC to that branch (creating it if new).
  const btnBranchToName = document.getElementById("btn-branch-to-name");
  if (btnBranchToName) {
    btnBranchToName.addEventListener("click", () => {
      const name = (currentSpec.robot_name
        || document.getElementById("cfg-robot-name")?.value || "robot").trim();
      const el = document.getElementById("auto-git-branch");
      if (el) { el.value = name; el.dataset.autoManaged = "false"; updateAutomationPreviews(); }
      const cmd = [
        `set -e`,
        `if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then echo "Not a git repository."; exit 1; fi`,
        `git checkout -b "${name}" 2>/dev/null || git checkout "${name}"`,
        `echo "Now on branch: $(git rev-parse --abbrev-ref HEAD)"`,
      ].join("\n");
      executeCommandInTerminal(cmd, `Switching to Git branch '${name}'`);
    });
  }

  // Dynamic Input Form Listeners
  const allInputs = document.querySelectorAll(".form-input, .form-select, input[type='checkbox']");
  allInputs.forEach(input => {
    input.addEventListener("input", handleInputChange);
    input.addEventListener("change", handleInputChange);
  });
  const otaHostEl = document.getElementById("cfg-ota-hostname");
  if (otaHostEl) otaHostEl.addEventListener("input", () => { otaHostEl.dataset.autoManaged = "false"; });
  // Typing in a covariance field marks it user-owned so a later sensor change
  // won't overwrite it with the datasheet default.
  ["cfg-cov-accel", "cfg-cov-gyro", "cfg-cov-ori", "cfg-cov-mag", "cfg-cov-env",
   "cfg-cov-pose", "cfg-cov-twist"].forEach((id) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener("input", () => { delete el.dataset.autoCov; });
  });
  // show/hide password toggles
  [["cfg-wifi-pass-show", "cfg-wifi-pass"], ["cfg-ota-password-show", "cfg-ota-password"]].forEach(([chk, fld]) => {
    const c = document.getElementById(chk), f = document.getElementById(fld);
    if (c && f) c.addEventListener("change", () => { f.type = c.checked ? "text" : "password"; });
  });
  ["cfg-agent-ip", "cfg-syslog-ip", "cfg-lidar-ip"].forEach((id) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener("input", () => { el.dataset.autoManaged = "false"; });
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
  // Board Config Loader
  document.getElementById("btn-load-board")?.addEventListener("click", openBoardSelector);
  document.getElementById("btn-board-cancel")?.addEventListener("click", () => {
    document.getElementById("modal-board-select").style.display = "none";
  });
  document.getElementById("btn-board-confirm")?.addEventListener("click", async () => {
    const envName = document.getElementById("board-select").value;
    document.getElementById("modal-board-select").style.display = "none";
    if (envName) await loadBoardConfig(envName);
  });
  // Close modal on backdrop click
  document.getElementById("modal-board-select")?.addEventListener("click", (e) => {
    if (e.target === document.getElementById("modal-board-select"))
      document.getElementById("modal-board-select").style.display = "none";
  });
  // JSON Import file listener
  document.getElementById("import-json-file")?.addEventListener("change", handleImportFile);

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
  const sel = document.getElementById("preset-select");
  if (sel && sel.value !== presetKey) sel.value = presetKey;
  populateFormFromSpec(currentSpec);
  recomputeAll();
  updateSimulationBanner();
}

// Map a parsed backend header spec (parser.py shape) onto the frontend form shape.
// The Web UI form / presets / export use `sensors.imu`, `sensors.mag`,
// `sensors.battery_monitor`, `sensors.sonar`, `dual_core`, `baudrate`, `transport`,
// whereas the C++ parser emits `sensors.imu_type`, `sensors.mag_type`,
// `sensors.use_ina219`, `sensors.sonar_trig/echo`, `telemetry.*`.
// This shim bridges the two so board-config / C++-header imports populate the form.
// Idempotent: never overwrites a frontend-shape key that is already present.
function normalizeSpecToFormShape(spec) {
  if (!spec) return spec;
  if (!spec.sensors) spec.sensors = {};
  const s = spec.sensors;

  // Only bridge parser-style keys when at least one is present and the
  // front-end-style key has not already been supplied (stay idempotent).
  const hasParserKeys =
    s.imu_type != null || s.mag_type != null || s.use_ina219 !== undefined ||
    s.sonar_trig !== undefined || s.battery_monitor == null;

  if (hasParserKeys) {
    // IMU: USE_QMI8658_IMU -> QMI8658, USE_FAKE_IMU -> FAKE
    if (s.imu == null && s.imu_type != null) {
      const v = String(s.imu_type).replace(/^USE_/, "").replace(/_IMU$/, "");
      if (v && v !== "IMU") s.imu = v;
    }
    // MAG: USE_AK09918_MAG -> AK09918, USE_FAKE_MAG -> NONE (no mag option exists)
    if (s.mag == null && s.mag_type != null) {
      let v = String(s.mag_type).replace(/^USE_/, "").replace(/_MAG$/, "");
      if (v === "FAKE") v = "NONE";
      if (v && v !== "MAG") s.mag = v;
    }
    // Battery monitor: INA219 vs ADC divider vs none
    if (s.battery_monitor == null) {
      if (s.use_ina219) s.battery_monitor = "INA219";
      else if (typeof s.battery_pin === "number" && s.battery_pin >= 0) s.battery_monitor = "ADC_DIVIDER";
    }
    // Sonar enabled when both trigger & echo pins are set
    if (s.sonar == null &&
        typeof s.sonar_trig === "number" && s.sonar_trig >= 0 &&
        typeof s.sonar_echo === "number" && s.sonar_echo >= 0) {
      s.sonar = true;
    }
    // Environmental barometer: parser emits env_type / use_bmp280
    if (s.env == null) {
      if (s.env_type) s.env = String(s.env_type).toUpperCase();
      else if (s.use_bmp280) s.env = "BMP280";
    }
    // Battery numeric fields use different names in the parser
    if (s.battery_capacity == null && s.battery_cap != null)            s.battery_capacity = s.battery_cap;
    if (s.battery_min_voltage == null && s.battery_min != null)         s.battery_min_voltage = s.battery_min;
    if (s.battery_max_voltage == null && s.battery_max != null)         s.battery_max_voltage = s.battery_max;
    if (s.battery_nominal_voltage == null && s.battery_nominal != null) s.battery_nominal_voltage = s.battery_nominal;
    if (s.battery_r1 == null && s.battery_divider_r1 != null)           s.battery_r1 = s.battery_divider_r1;
    if (s.battery_r2 == null && s.battery_divider_r2 != null)           s.battery_r2 = s.battery_divider_r2;
  }

  // Dual-core: parser puts it under telemetry / top-level may lack it
  if (spec.dual_core == null && spec.telemetry && typeof spec.telemetry.use_dual_core === "boolean") {
    spec.dual_core = spec.telemetry.use_dual_core;
  }
  // Baudrate & transport fallbacks from the telemetry block
  if (spec.baudrate == null && spec.telemetry && spec.telemetry.baudrate != null) spec.baudrate = spec.telemetry.baudrate;
  if (spec.transport == null && spec.telemetry && spec.telemetry.transport) spec.transport = spec.telemetry.transport;

  return spec;
}

// Populate UI Form Fields from Spec Object
function populateFormFromSpec(spec) {
  spec = normalizeSpecToFormShape(spec);
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
    // A real SSID came from config/custom/wifi_config.h — show the WiFi block.
    const _ss = (spec.wifi_settings.ssid || "").trim();
    if (_ss && _ss !== "YOUR_WIFI_SSID") {
      const bg = document.getElementById("cfg-wifi-telemetry");
      if (bg) bg.checked = true;
    }
  }

  // Geometry
  document.getElementById("cfg-wheel-dia").value = spec.geometry?.wheel_diameter || 0.08;
  document.getElementById("cfg-track-width").value = spec.geometry?.track_width || 0.22;
  document.getElementById("cfg-wheelbase").value = spec.geometry?.wheelbase || 0.20;
  if (document.getElementById("cfg-weight")) document.getElementById("cfg-weight").value = spec.geometry?.weight || 3.5;
  if (document.getElementById("cfg-fake-wheel")) document.getElementById("cfg-fake-wheel").checked = Boolean(spec.simulation?.fake_wheel);
  // loading a spec is not the user reaching for the checkbox
  userTouchedSimulation = false;
  updateSimulationBanner();
  if (document.getElementById("cfg-fake-ld19")) {
    const fakeLidar = Boolean(spec.simulation?.fake_ld19);
    document.getElementById("cfg-fake-ld19").checked = fakeLidar;
    const block = document.getElementById("fake-ld19-settings-block");
    if (block) block.style.display = fakeLidar ? "block" : "none";
  }
  // Defaults must match the firmware (fake_ld19.h: 16.0 x 12.0). These run on
  // every preset load and overwrite the markup, so a stale value here silently
  // shrinks the simulated room for every design that does not set its own.
  if (document.getElementById("cfg-map-width")) document.getElementById("cfg-map-width").value = spec.simulation?.map_width ?? 10.0;
  if (document.getElementById("cfg-map-height")) document.getElementById("cfg-map-height").value = spec.simulation?.map_height ?? 6.0;
  if (document.getElementById("cfg-wall-obstacle")) document.getElementById("cfg-wall-obstacle").checked = spec.simulation?.wall_obstacle !== false;
  if (document.getElementById("cfg-wall-x1")) document.getElementById("cfg-wall-x1").value = spec.simulation?.wall_x1 ?? 2.0;
  if (document.getElementById("cfg-wall-y1")) document.getElementById("cfg-wall-y1").value = spec.simulation?.wall_y1 ?? -1.5;
  if (document.getElementById("cfg-wall-x2")) document.getElementById("cfg-wall-x2").value = spec.simulation?.wall_x2 ?? 2.0;
  if (document.getElementById("cfg-wall-y2")) document.getElementById("cfg-wall-y2").value = spec.simulation?.wall_y2 ?? 1.5;
  // Defaults on with the fake LiDAR: the range comes from the same raycast, so
  // it needs no pins, and a beginner driving a simulated robot into a wall
  // should get the same protection a real one would have.
  if (document.getElementById("cfg-fake-sonar")) document.getElementById("cfg-fake-sonar").checked = spec.simulation?.fake_sonar !== false;

  // Motors
  document.getElementById("cfg-driver-type").value = spec.motors?.driver_type || "BTS7960";
  document.getElementById("cfg-max-rpm").value = spec.motors?.max_rpm || 330;
  document.getElementById("cfg-cpr").value = spec.motors?.cpr || 1440;
  document.getElementById("cfg-motor-voltage").value = spec.motors?.operating_voltage || 12.0;
  if (document.getElementById("cfg-motor-torque")) document.getElementById("cfg-motor-torque").value = spec.motors?.rated_torque || 1.5;

  for (let i = 1; i <= 4; i++) {
    const el = document.getElementById(`pin-m${i}-inv`);
    if (el) el.checked = !!spec.motors?.[`motor${i}_inv`];
  }

  // Sensors
  document.getElementById("cfg-imu").value = spec.sensors?.imu || "NONE";
  document.getElementById("cfg-mag").value = spec.sensors?.mag || "NONE";
  document.getElementById("cfg-battery").value = spec.sensors?.battery_monitor || "NONE";
  if (document.getElementById("cfg-bat-cap")) document.getElementById("cfg-bat-cap").value = spec.sensors?.battery_capacity || 2.2;
  if (document.getElementById("cfg-bat-nom")) document.getElementById("cfg-bat-nom").value = spec.sensors?.battery_nominal_voltage || 11.1;
  if (document.getElementById("cfg-bat-min")) document.getElementById("cfg-bat-min").value = spec.sensors?.battery_min_voltage || 9.0;
  if (document.getElementById("cfg-bat-max")) document.getElementById("cfg-bat-max").value = spec.sensors?.battery_max_voltage || 12.6;
  if (document.getElementById("cfg-bat-dip")) document.getElementById("cfg-bat-dip").value = spec.sensors?.battery_dip ?? "";
  if (document.getElementById("cfg-bat-r1")) document.getElementById("cfg-bat-r1").value = spec.sensors?.battery_r1 || 30000;
  if (document.getElementById("cfg-bat-r2")) document.getElementById("cfg-bat-r2").value = spec.sensors?.battery_r2 || 7500;
  if (document.getElementById("cfg-bat-cap-val")) document.getElementById("cfg-bat-cap-val").value = spec.sensors?.battery_adc_cap || 1000;
  const _dacPinLoad = spec.pins?.dac_pin ?? spec.sensors?.dac_pin;
  if (document.getElementById("cfg-dac-pin") && _dacPinLoad != null && _dacPinLoad >= 0) {
    document.getElementById("cfg-dac-pin").value = String(_dacPinLoad);
  }
  document.getElementById("cfg-sonar").value = spec.sensors?.sonar ? "true" : "false";
  if (document.getElementById("cfg-env")) document.getElementById("cfg-env").value = spec.sensors?.env || "NONE";

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
  const _pick = (...v) => v.find((x) => x !== undefined && x !== null);
  for (let i = 1; i <= 4; i++) {
    const m = p[`motor${i}`] || {};
    let p1, p2, p3;
    if (dt === "BTS7960") {
      // Two outputs: RPWM (IN_A) and LPWM (IN_B). Accept legacy pwm_r/pwm_l/en.
      p1 = _pick(m.in_a, m.pwm_l, m.pwm_r, m.pwm);
      p2 = _pick(m.in_b, m.en);
      p3 = undefined;
    } else if (dt === "GENERIC_1_IN") {
      p1 = m.pwm; p2 = _pick(m.dir, m.in_a); p3 = m.in_b;
    } else {
      p1 = m.pwm; p2 = m.in_a; p3 = m.in_b;
    }
    setPinVal(`pin-m${i}-p1`, p1);
    setPinVal(`pin-m${i}-p2`, p2);
    setPinVal(`pin-m${i}-p3`, p3);
  }

  const enc = p.encoders || {};
  setPinVal("pin-enc-1a", enc.m1_a);
  setPinVal("pin-enc-1b", enc.m1_b);
  setPinVal("pin-enc-2a", enc.m2_a);
  setPinVal("pin-enc-2b", enc.m2_b);
  setPinVal("pin-enc-3a", enc.m3_a);
  setPinVal("pin-enc-3b", enc.m3_b);
  setPinVal("pin-enc-4a", enc.m4_a);
  setPinVal("pin-enc-4b", enc.m4_b);

  // Encoder invert flags
  for (let i = 1; i <= 4; i++) {
    const cb = document.getElementById(`pin-enc-${i}-inv`);
    if (cb) cb.checked = !!enc[`m${i}_inv`];
  }

  setPinVal("pin-i2c-sda", p.i2c?.sda);
  setPinVal("pin-i2c-scl", p.i2c?.scl);

  // advanced telemetry (syslog / LiDAR-UDP / OTA)
  const adv = spec.advanced || {};
  const setVal = (id, v) => { const el = document.getElementById(id); if (el && v !== undefined && v !== null && v !== "") el.value = v; };
  const setCk  = (id, v) => { const el = document.getElementById(id); if (el) el.checked = !!v; };
  setVal("cfg-syslog-ip", adv.syslog_ip);
  setVal("cfg-syslog-port", adv.syslog_port);
  setVal("cfg-wifi-monitor", adv.wifi_monitor);
  setVal("cfg-lidar-ip", adv.lidar_ip);
  setVal("cfg-lidar-port", adv.lidar_port);
  // Reference designs carry this under pins (it is a pin), the form and the
  // rest of the spec keep it under advanced. Read both, or a preset that sets
  // pins.lidar_rxd loads as -1 and the generated header says LIDAR_RXD -1 --
  // the firmware then starts the LD19 emulator with no output and silently
  // transmits nothing.
  setVal("cfg-lidar-rxd", spec.pins?.lidar_rxd ?? adv.lidar_rxd);
  setVal("cfg-lidar-uart", adv.lidar_serial);
  setVal("cfg-lidar-baud", adv.lidar_baudrate);
  setVal("cfg-ota-hostname", adv.ota_hostname);
  setVal("cfg-ota-password", adv.ota_password);
  setVal("cfg-ota-ip", adv.ota_ip);
  setVal("cfg-topic-prefix", adv.topic_prefix);
  setCk("cfg-lidar-udp", spec.telemetry?.use_lidar_udp);
  setCk("cfg-ota-enable", adv.ota_enable);

  // PID constants + IMU / mag tuning
  const pid = spec.pid || {};
  setVal("cfg-pid-kp", pid.kp);
  setVal("cfg-pid-ki", pid.ki);
  setVal("cfg-pid-kd", pid.kd);
  const tune = spec.imu_tuning || {};
  const setNum = (id, v) => { const el = document.getElementById(id); if (el) el.value = (v === undefined || v === null) ? "" : v; };
  if (Array.isArray(tune.mag_bias)) {
    setNum("cfg-mag-bias-x", tune.mag_bias[0]);
    setNum("cfg-mag-bias-y", tune.mag_bias[1]);
    setNum("cfg-mag-bias-z", tune.mag_bias[2]);
  } else {
    setNum("cfg-mag-bias-x", null); setNum("cfg-mag-bias-y", null); setNum("cfg-mag-bias-z", null);
  }
  const covScalar = (v) => Array.isArray(v) ? v[0] : v;
  const covList = (v) => Array.isArray(v) ? v.join(", ") : (v ?? "");
  setNum("cfg-cov-accel", covScalar(tune.accel_cov));
  setNum("cfg-cov-gyro", covScalar(tune.gyro_cov));
  setNum("cfg-cov-ori", covScalar(tune.ori_cov));
  setNum("cfg-cov-mag", covScalar(tune.mag_cov));
  setNum("cfg-cov-pose", covList(tune.pose_cov));
  setNum("cfg-cov-twist", covList(tune.twist_cov));
  setNum("cfg-cov-env", covList(tune.env_cov));
  setPinVal("pin-battery", p.battery_pin);
  setPinVal("pin-sonar-trig", p.sonar?.trig);
  setPinVal("pin-sonar-echo", p.sonar?.echo);

  const robotName = spec.robot_name || "my_robot";
  // The git branch field mirrors the Robot SBC's current branch (populated
  // from /api/gitinfo), not the robot name — leave it to applyCurrentBranch().
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
  // Fill datasheet covariance defaults for the loaded IMU/mag/barometer, but
  // only into fields the imported spec left blank (explicit values are kept).
  applySensorCovDefaults();
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

// Like getFloat but returns null (not a default) when the field is left blank,
// so optional numeric settings can be "unset".
function getFloatOrNull(id) {
  const el = document.getElementById(id);
  if (!el || el.value.trim() === "" || isNaN(parseFloat(el.value))) return null;
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
      // emitted as ROBOT_WEIGHT, which fake wheel mode uses as the simulated mass
      weight: getFloat("cfg-weight", 3.5),
    },
    simulation: {
      fake_wheel: document.getElementById("cfg-fake-wheel")
        ? document.getElementById("cfg-fake-wheel").checked
        : false,
      fake_ld19: document.getElementById("cfg-fake-ld19")
        ? document.getElementById("cfg-fake-ld19").checked
        : false,
      map_width: getFloat("cfg-map-width", 10.0),
      map_height: getFloat("cfg-map-height", 6.0),
      wall_obstacle: document.getElementById("cfg-wall-obstacle")
        ? document.getElementById("cfg-wall-obstacle").checked
        : true,
      wall_x1: getFloat("cfg-wall-x1", 2.0),
      wall_y1: getFloat("cfg-wall-y1", -1.5),
      wall_x2: getFloat("cfg-wall-x2", 2.0),
      wall_y2: getFloat("cfg-wall-y2", 1.5),
      fake_sonar: document.getElementById("cfg-fake-sonar")
        ? document.getElementById("cfg-fake-sonar").checked
        : true,
    },
    motors: {
      driver_type: driverType,
      max_rpm: getFloat("cfg-max-rpm", 330),
      cpr: getFloat("cfg-cpr", 1440),
      operating_voltage: getFloat("cfg-motor-voltage", 12.0),
      motor1_inv: !!document.getElementById("pin-m1-inv")?.checked,
      motor2_inv: !!document.getElementById("pin-m2-inv")?.checked,
      motor3_inv: !!document.getElementById("pin-m3-inv")?.checked,
      motor4_inv: !!document.getElementById("pin-m4-inv")?.checked
    },
    pid: {
      kp: getFloat("cfg-pid-kp", 0.6),
      ki: getFloat("cfg-pid-ki", 0.8),
      kd: getFloat("cfg-pid-kd", 0.5)
    },
    sensors: {
      imu: document.getElementById("cfg-imu").value,
      mag: document.getElementById("cfg-mag").value,
      battery_monitor: document.getElementById("cfg-battery").value,
      battery_capacity: getFloat("cfg-bat-cap", 2.2),
      battery_nominal_voltage: getFloat("cfg-bat-nom", 11.1),
      battery_min_voltage: getFloat("cfg-bat-min", 9.0),
      battery_max_voltage: getFloat("cfg-bat-max", 12.6),
      battery_dip: getFloatOrNull("cfg-bat-dip"),   // blank = disabled (no BATTERY_DIP emitted)
      battery_r1: getFloat("cfg-bat-r1", 30000.0),
      battery_r2: getFloat("cfg-bat-r2", 7500.0),
      battery_adc_cap: getFloat("cfg-bat-cap-val", 1000.0),
      sonar: document.getElementById("cfg-sonar").value === "true",
      env: document.getElementById("cfg-env")?.value || "NONE"
    },
    imu_tuning: (() => {
      const bx = getFloatOrNull("cfg-mag-bias-x"), by = getFloatOrNull("cfg-mag-bias-y"), bz = getFloatOrNull("cfg-mag-bias-z");
      const t = {};
      if (bx !== null || by !== null || bz !== null) t.mag_bias = [bx || 0, by || 0, bz || 0];
      const ac = getFloatOrNull("cfg-cov-accel"), gc = getFloatOrNull("cfg-cov-gyro"), oc = getFloatOrNull("cfg-cov-ori");
      if (ac !== null) t.accel_cov = ac;
      if (gc !== null) t.gyro_cov = gc;
      if (oc !== null) t.ori_cov = oc;
      const mc = getFloatOrNull("cfg-cov-mag");
      if (mc !== null) t.mag_cov = mc;
      // POSE/TWIST_COV: scalar, or a comma list (usually 6); kept as typed.
      const covField = (id) => {
        const raw = (document.getElementById(id)?.value || "").trim();
        if (!raw) return null;
        if (raw.includes(",")) {
          const arr = raw.split(",").map((x) => Number(x.trim()));
          return arr.some((x) => Number.isNaN(x)) ? null : arr;
        }
        const n = Number(raw);
        return Number.isNaN(n) ? null : n;
      };
      const pc = covField("cfg-cov-pose"), tc = covField("cfg-cov-twist");
      if (pc !== null) t.pose_cov = pc;
      if (tc !== null) t.twist_cov = tc;
      // ENV_COV: BMP280 FluidPressure/Temperature/RelativeHumidity .variance
      // { pressure Pa^2, temperature C^2, humidity (0..1)^2 } — scalar or 3-list.
      const evc = covField("cfg-cov-env");
      if (evc !== null) t.env_cov = evc;
      return t;
    })(),
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
        m2_b: getInt("pin-enc-2b"),
        m1_inv: !!(document.getElementById("pin-enc-1-inv")?.checked),
        m2_inv: !!(document.getElementById("pin-enc-2-inv")?.checked),
        m3_inv: !!(document.getElementById("pin-enc-3-inv")?.checked),
        m4_inv: !!(document.getElementById("pin-enc-4-inv")?.checked)
      },
      i2c: {
        sda: getInt("pin-i2c-sda"),
        scl: getInt("pin-i2c-scl")
      }
    }
  };

  // Advanced telemetry: syslog / LiDAR-UDP / Arduino OTA (Sensors tab)
  const _v = (id) => (document.getElementById(id)?.value || "").trim();
  const _ck = (id) => !!document.getElementById(id)?.checked;
  spec.advanced = {
    syslog_ip: _v("cfg-syslog-ip"),
    syslog_port: getInt("cfg-syslog-port", 514),
    wifi_monitor: getInt("cfg-wifi-monitor", 0),
    lidar_ip: _v("cfg-lidar-ip"),
    lidar_port: getInt("cfg-lidar-port", 8889),
    lidar_rxd: getInt("cfg-lidar-rxd", -1),
    lidar_serial: getInt("cfg-lidar-uart", 1),
    lidar_baudrate: getInt("cfg-lidar-baud", 230400),
    ota_hostname: _v("cfg-ota-hostname"),
    ota_password: _v("cfg-ota-password"),
    ota_ip: _v("cfg-ota-ip"),
    ota_enable: _ck("cfg-ota-enable"),
    topic_prefix: _v("cfg-topic-prefix")
  };
  spec.telemetry = Object.assign({}, spec.telemetry, { use_lidar_udp: _ck("cfg-lidar-udp") });
  // Background WiFi (OTA/syslog while micro-ROS stays on serial) turns on
  // whenever any of these wireless features are requested.
  if (_ck("cfg-lidar-udp") || _ck("cfg-ota-enable")) spec.enable_ota_syslog = true;

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
      // Two outputs only: p1 = RPWM (MOTORx_IN_A), p2 = LPWM (MOTORx_IN_B).
      return { in_a: p1, in_b: p2 };
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
    // DAC sweep pin — only meaningful where a hardware DAC exists.
    if (mcuHasDac(spec.mcu)) {
      const dp = getInt("cfg-dac-pin", -1);
      if (dp >= 0) spec.pins.dac_pin = dp;
    }
  }
  if (spec.sensors.sonar) {
    spec.pins.sonar = {
      trig: getInt("pin-sonar-trig"),
      echo: getInt("pin-sonar-echo")
    };
  }

  // WiFi credentials are needed both for WIFI_UDP transport AND for the
  // "Enable Background WiFi for OTA & Syslog" mode (micro-ROS stays on serial
  // but the firmware still joins WiFi for USE_SYSLOG / USE_ARDUINO_OTA).
  const _ssidRaw = (document.getElementById("cfg-wifi-ssid")?.value || "").trim();
  const _ssidReal = _ssidRaw && _ssidRaw !== "YOUR_WIFI_SSID";
  // "Enable Background WiFi" only takes effect once a real SSID is entered —
  // otherwise a bare first-run would emit WIFI_AP_LIST with the placeholder and
  // the firmware would hang at boot trying to join "YOUR_WIFI_SSID".
  const bgWifi = !!document.getElementById("cfg-wifi-telemetry")?.checked && _ssidReal;
  if (transport === "WIFI_UDP" || bgWifi) {
    spec.wifi_settings = {
      ssid: _ssidRaw,
      password: document.getElementById("cfg-wifi-pass").value,
      agent_ip: document.getElementById("cfg-agent-ip").value,
      agent_port: getInt("cfg-agent-port", 8888)
    };
  }
  if (bgWifi && transport !== "WIFI_UDP") spec.enable_ota_syslog = true;

  return spec;
}

// Handle Form Changes
// Datasheet-derived default measurement variances. Auto-filled into the
// Covariance Overrides when a sensor is enabled, so the generated header
// ships realistic ACCEL/GYRO/ORI/MAG/ENV_COV instead of the firmware's
// generic 1e-5 fallback. variance ≈ (noise_density · √(~100 Hz bandwidth))².
// A value the user types is never overwritten (see applySensorCovDefaults).
const SENSOR_COV_DEFAULTS = {
  imu: {   // { accel: (m/s²)² , gyro: (rad/s)² , ori?: rad² (fused output only) }
    BNO085:   { accel: 2.2e-4, gyro: 1.5e-6, ori: 4e-3 },
    LSM6DSOX: { accel: 4.7e-5, gyro: 4.4e-7 },
    ICM20948: { accel: 5.1e-4, gyro: 6.9e-6 },
    QMI8658:  { accel: 8e-5,   gyro: 2e-6 },
    MPU9250:  { accel: 9e-4,   gyro: 3e-6 },
    MPU9150:  { accel: 1.5e-3, gyro: 3e-6 },
    MPU6050:  { accel: 1.5e-3, gyro: 3e-6 },
    GY85:     { accel: 1.8e-3, gyro: 4.4e-5 },
  },
  mag: {   // Tesla²
    AK09918:  2.3e-14, ICM20948: 2.3e-14,
    QMC5883L: 4e-14,   HMC5883L: 4e-14,
    AK8963:   9e-14,   AK8975:   9e-14,
  },
  env: {   // [ pressure Pa² (σ≈1.7 Pa, IIR-x4) , temperature °C² (±0.5°C accuracy) , humidity (0..1)² (±3% RH) ]
    BMP280: [3, 0.25, 0],
    BME280: [3, 0.25, 9e-4],
  },
};

// Fill the ACCEL/GYRO/ORI/MAG/ENV_COV fields from SENSOR_COV_DEFAULTS for the
// currently-selected IMU / magnetometer / barometer. Only touches a field that
// is empty or that we filled ourselves (data-auto-cov); a user-typed value —
// or one being typed right now — is left alone. Clearing a sensor back to
// NONE/FAKE also clears its auto-filled covariance.
function applySensorCovDefaults() {
  const setAuto = (id, val) => {
    const el = document.getElementById(id);
    if (!el || document.activeElement === el) return;
    const userTyped = (el.value || "").trim() !== "" && el.dataset.autoCov !== "1";
    if (userTyped) return;
    if (val == null || val === "") {
      if (el.dataset.autoCov === "1") { el.value = ""; delete el.dataset.autoCov; }
      return;
    }
    el.value = Array.isArray(val) ? val.join(", ") : String(val);
    el.dataset.autoCov = "1";
  };
  const d = SENSOR_COV_DEFAULTS.imu[document.getElementById("cfg-imu")?.value];
  setAuto("cfg-cov-accel", d ? d.accel : null);
  setAuto("cfg-cov-gyro",  d ? d.gyro  : null);
  setAuto("cfg-cov-ori",   d && d.ori != null ? d.ori : null);
  setAuto("cfg-cov-mag", SENSOR_COV_DEFAULTS.mag[document.getElementById("cfg-mag")?.value] ?? null);
  setAuto("cfg-cov-env", SENSOR_COV_DEFAULTS.env[document.getElementById("cfg-env")?.value] ?? null);
}

function handleInputChange() {
  updateDynamicUIState();
  applySensorCovDefaults();
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

  // Simulation Fake LD19 Elements
  const fakeLidarEl = document.getElementById("cfg-fake-ld19");
  const fakeLidarBox = document.getElementById("fake-ld19-settings-block");
  if (fakeLidarEl && fakeLidarBox) {
    fakeLidarBox.style.display = fakeLidarEl.checked ? "block" : "none";
  }

  // 4WD Elements
  document.getElementById("group-wheelbase").style.display = is4WD ? "flex" : "none";
  // motor 3/4 invert checkboxes live in the pinout matrix rows, which
  // already show/hide with row-motor-3 / row-motor-4
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
  // Console serial monitor baud follows the firmware BAUDRATE until the user
  // picks one explicitly (override for bootloader / MicroPython / other boards).
  // "followedValue" is the last value we pushed; if the select no longer holds
  // it, the user changed it by hand -> stop following.
  const _cfgBaud = document.getElementById("cfg-baudrate");
  const _monBaud = document.getElementById("serial-baud-select");
  if (_cfgBaud && _monBaud) {
    const _followed = _monBaud.dataset.followedValue;
    if (_monBaud.dataset.autoManaged !== "false" &&
        (_followed === undefined || _followed === _monBaud.value)) {
      if ([..._monBaud.options].some((o) => o.value === _cfgBaud.value)) {
        _monBaud.value = _cfgBaud.value;
      }
      _monBaud.dataset.followedValue = _monBaud.value;
    } else {
      _monBaud.dataset.autoManaged = "false";
    }
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

  // DAC pin selector + ADC Calibration Studio availability.
  // The hardware DAC (and therefore adc_calibrate) only exists on ESP32 /
  // ESP32-S2 / GENDRV; on every other MCU the studio is greyed out and the
  // adc_calibrate build target is disabled.
  const hasDac = mcuHasDac(mcu);
  const dacSel = document.getElementById("cfg-dac-pin");
  if (dacSel) {
    const allowed = dacPinsForMcu(mcu).map(String);
    [...dacSel.options].forEach((o) => { o.hidden = !allowed.includes(o.value); });
    if (!allowed.includes(dacSel.value)) dacSel.value = allowed[0];
    // Gated on hasDac alone, matching "Run Hardware Calibration" itself —
    // adc_calibrate is a standalone diagnostic and doesn't require
    // battery_monitor to be set to ADC_DIVIDER. Requiring that too meant the
    // pin selector could be invisible while the Run button sat right there
    // enabled, silently sweeping whatever pin was last selected.
    const dacRow = document.getElementById("adc-dacpin-row");
    if (dacRow) dacRow.style.display = hasDac ? "flex" : "none";
    // Keep the schematic label + status line in sync with the chosen pin.
    const _dacFamily = mcu === "ESP32S2" ? "ESP32-S2 DAC" : "ESP32 DAC";
    const svgDac = document.getElementById("cad-svg-dac-pin");
    if (svgDac) svgDac.textContent = `${_dacFamily} (GPIO ${dacSel.value})`;
    const st = document.getElementById("adc-status-text");
    if (st && /Ready for calibration/.test(st.textContent)) {
      st.textContent = `Ready for calibration. GPIO ${dacSel.value} (DAC) -> battery ADC pin.`;
    }
  }
  const adcCard = document.getElementById("adc-studio-card");
  if (adcCard) adcCard.classList.toggle("is-disabled", !hasDac);
  const noDacNote = document.getElementById("adc-nodac-note");
  if (noDacNote) noDacNote.style.display = hasDac ? "none" : "block";
  const btnRunCal = document.getElementById("btn-adc-run-cal");
  if (btnRunCal) { btnRunCal.disabled = !hasDac; btnRunCal.classList.toggle("is-disabled", !hasDac); }
  // The "Upload adc_calibrate" 1-click button is hidden outright (like the
  // Target Firmware Binary option) on MCUs without a hardware DAC.
  const btnUploadAdc = document.getElementById("btn-upload-adc");
  if (btnUploadAdc) {
    btnUploadAdc.style.display = hasDac ? "" : "none";
    btnUploadAdc.disabled = !hasDac;
  }
  const adcTargetOpt = document.querySelector('#auto-flash-target option[value="adc_calibrate"]');
  if (adcTargetOpt) {
    // Hide the adc_calibrate target entirely on MCUs without a hardware DAC.
    adcTargetOpt.hidden = !hasDac;
    adcTargetOpt.disabled = !hasDac;
    const tgtSel = document.getElementById("auto-flash-target");
    if (!hasDac && tgtSel && tgtSel.value === "adc_calibrate") tgtSel.value = "firmware";
  }

  // Sonar
  document.getElementById("box-sonar-trig").style.display = sonarActive ? "flex" : "none";
  document.getElementById("box-sonar-echo").style.display = sonarActive ? "flex" : "none";

  // Motor pin matrix: header + how many of the 3 pin columns the driver
  // actually uses. Columns beyond that are hidden, and any blank value in
  // them is parked at -1 ("no connection") so it never reads as GPIO 0.
  // A value the user has typed into a hidden column is left untouched.
  const headerEl = document.getElementById("motor-pin-headers");
  const invTh = `<th title="Invert this motor's spin direction">Invert</th>`;
  let pinTh, activePins;
  if (driverType === "BTS7960") {
    pinTh = `<th>RPWM &rarr; IN_A</th><th>LPWM &rarr; IN_B</th>`;
    activePins = 2;
  } else if (driverType === "GENERIC_2_IN") {
    pinTh = `<th>PWM (Speed)</th><th>IN_A (Dir 1)</th><th>IN_B (Dir 2)</th>`;
    activePins = 3;
  } else if (driverType === "GENERIC_1_IN") {
    pinTh = `<th>PWM (Speed)</th><th>DIR (Direction)</th>`;
    activePins = 2;
  } else { // ESC
    pinTh = `<th>PWM Signal</th>`;
    activePins = 1;
  }
  headerEl.innerHTML = `<th>Motor</th>` + pinTh + invTh;
  for (let i = 1; i <= 4; i++) {
    for (let c = 1; c <= 3; c++) {
      const el = document.getElementById(`pin-m${i}-p${c}`);
      if (!el) continue;
      const used = c <= activePins;
      const cell = el.closest("td");
      if (cell) cell.style.display = used ? "" : "none";
      if (!used && el.value.trim() === "") el.value = "-1";
    }
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

  // 6. Auto-persist workflow and board setup
  if (typeof saveWorkflowAndBoardSetup === "function") {
    saveWorkflowAndBoardSetup();
  }
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
    // A negative value means "no connection / pin not assigned" — skip conflict
    // detection and MCU-specific GPIO range checks for unassigned slots.
    if (p < 0) return;
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
      registerPin(m.in_a !== undefined ? m.in_a : m.pwm_l, `Motor ${i} RPWM (IN_A)`, true);
      registerPin(m.in_b !== undefined ? m.in_b : m.en, `Motor ${i} LPWM (IN_B)`, true);
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
    `#define MOTOR1_ENCODER_INV ${!!(pins?.encoders?.m1_inv)}`,
    `#define MOTOR2_ENCODER_INV ${!!(pins?.encoders?.m2_inv)}`,
    `#define MOTOR3_ENCODER_INV ${!!(pins?.encoders?.m3_inv)}`,
    `#define MOTOR4_ENCODER_INV ${!!(pins?.encoders?.m4_inv)}`,
    ``,
    `// Velocity PID Tuning Constants`,
    `#define K_P ${spec.pid?.kp ?? 0.6}`,
    `#define K_I ${spec.pid?.ki ?? 0.8}`,
    `#define K_D ${spec.pid?.kd ?? 0.5}`,
    ``,
    `// Pin Assignments`,
    `#define LED_PIN ${pins.led !== undefined ? pins.led : (["PICOW", "PICO2W"].includes(mcu) ? "LED_BUILTIN" : (mcu.includes("PICO") ? 25 : 2))}`
  );

  // ENCODER PINS (all 4, unconditional -- matches the hand-written headers)
  const enc = pins.encoders || {};
  const pinVal = (v) => (v !== undefined && v !== null && v !== "") ? v : -1;
  lines.push(``, `// ENCODER PINS`);
  for (let i = 1; i <= 4; i++) {
    lines.push(`#define MOTOR${i}_ENCODER_A ${pinVal(enc[`m${i}_a`])}`);
    lines.push(`#define MOTOR${i}_ENCODER_B ${pinVal(enc[`m${i}_b`])}`);
  }

  // MOTOR PINS -- firmware.ino reads MOTORx_PWM / MOTORx_IN_A / MOTORx_IN_B for
  // EVERY driver (see firmware/src/firmware.ino motorN_controller(...) and
  // firmware/lib/motor/default_motor.h). Keep this identical to generator.py.
  lines.push(``, `// MOTOR PINS`);
  lines.push(`#ifdef USE_${driver}_MOTOR_DRIVER`);
  for (let i = 1; i <= 4; i++) {
    const m = pins[`motor${i}`] || {};
    if (driver === "BTS7960") {
      // BTS7960 uses two outputs only: IN_A = RPWM, IN_B = LPWM.
      // MOTORx_PWM is an unused arg in the driver class -> fixed placeholder.
      lines.push(`  #define MOTOR${i}_PWM -1 //DON'T TOUCH THIS! This is just a placeholder`);
      lines.push(`  #define MOTOR${i}_IN_A ${pinVal(m.in_a !== undefined ? m.in_a : m.pwm_l)}`);
      lines.push(`  #define MOTOR${i}_IN_B ${pinVal(m.in_b !== undefined ? m.in_b : m.en)}`);
    } else if (driver === "GENERIC_1_IN") {
      lines.push(`  #define MOTOR${i}_PWM ${pinVal(m.pwm)}`);
      lines.push(`  #define MOTOR${i}_IN_A ${pinVal(m.dir !== undefined ? m.dir : m.in_a)}`);
      lines.push(`  #define MOTOR${i}_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder`);
    } else if (driver === "ESC") {
      lines.push(`  #define MOTOR${i}_PWM ${pinVal(m.pwm)}`);
      lines.push(`  #define MOTOR${i}_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder`);
      lines.push(`  #define MOTOR${i}_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder`);
    } else {
      // GENERIC_2_IN (default)
      lines.push(`  #define MOTOR${i}_PWM ${pinVal(m.pwm)}`);
      lines.push(`  #define MOTOR${i}_IN_A ${pinVal(m.in_a)}`);
      lines.push(`  #define MOTOR${i}_IN_B ${pinVal(m.in_b)}`);
    }
  }
  lines.push(`  #define PWM_MAX pow(2, PWM_BITS) - 1`);
  lines.push(`  #define PWM_MIN -PWM_MAX`);
  lines.push(`#endif`);

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

  // IMU / magnetometer tuning — MAG_BIAS is #ifdef-gated in firmware.ino;
  // ACCEL_COV / GYRO_COV / ORI_COV are #ifndef-gated in imu_interface.h.
  const tune = spec.imu_tuning || {};
  if (Array.isArray(tune.mag_bias) && tune.mag_bias.length === 3) {
    lines.push(`#define MAG_BIAS { ${tune.mag_bias.map(Number).join(", ")} }`);
  }
  const covLine = (macro, v, n = 3) => {
    if (v == null || v === "") return;
    let t;
    if (Array.isArray(v)) t = v.map(Number);
    else {
      const s = String(v).trim();
      t = s.includes(",") ? s.split(",").map((x) => Number(x.trim()))
                          : Array(n).fill(Number(s));
    }
    if (t.some((x) => Number.isNaN(x))) return;
    lines.push(`#define ${macro} { ${t.join(", ")} }`);
  };
  covLine("ACCEL_COV", tune.accel_cov);
  covLine("GYRO_COV", tune.gyro_cov);
  covLine("ORI_COV", tune.ori_cov);
  covLine("MAG_COV", tune.mag_cov);
  covLine("POSE_COV", tune.pose_cov, 6);
  covLine("TWIST_COV", tune.twist_cov, 6);
  covLine("ENV_COV", tune.env_cov, 3);   // { pressure, temperature, humidity } variance

  if (sensors.battery_monitor === "ADC_DIVIDER") {
    const numFmt = (x) => { const f = Number(x); return Number.isInteger(f) ? String(f) : String(f).replace(/0+$/,"").replace(/\.$/,""); };
    const r1k = numFmt((sensors.battery_r1 || 30000) / 1000);
    const r2k = numFmt((sensors.battery_r2 || 7500) / 1000);
    const isRp2 = mcu.includes("PICO") || mcu.includes("RP2");
    lines.push(`#define BATTERY_PIN ${pins.battery_pin ?? 26}`);
    // DAC output pin for the adc_calibrate sweep — hardware DAC only exists on
    // ESP32 / ESP32-S2 / GENDRV.
    const _dacPin = pins.dac_pin ?? sensors.dac_pin;
    if (mcuHasDac(mcu) && _dacPin != null && Number(_dacPin) >= 0) {
      lines.push(`#define DAC_PIN ${Number(_dacPin)}`);
    }
    // firmware/lib/battery/battery.cpp calls BATTERY_ADJUST(reading) with no
    // #ifndef fallback — an ADC battery config MUST define it. ESP32
    // analogReadMilliVolts() returns mV; RP2040 analogRead() returns 12-bit counts.
    if (spec.adc_lut || sensors.adc_lut) {
      lines.push(
        `#define USE_ADC_LUT`,
        formatAdcLutArray(spec.adc_lut || sensors.adc_lut),
        `#define BATTERY_ADJUST(v) (ADC_LUT[v] * (3.3 / 4096 * (${r1k} + ${r2k}) / ${r2k}))`
      );
    } else if (isRp2) {
      lines.push(`#define BATTERY_ADJUST(v) ((v) * (3.3 / 4096 * (${r1k} + ${r2k}) / ${r2k}))`);
    } else {
      lines.push(`#define BATTERY_ADJUST(v) ((v) * ((${r1k} + ${r2k}) / ${r2k}) / 1000.0)`);
    }
    if (sensors.battery_min_voltage != null) lines.push(`#define BATTERY_MIN ${numFmt(sensors.battery_min_voltage)}`);
    if (sensors.battery_max_voltage != null) lines.push(`#define BATTERY_MAX ${numFmt(sensors.battery_max_voltage)}`);
    if (sensors.battery_capacity != null)    lines.push(`#define BATTERY_CAP ${numFmt(sensors.battery_capacity)}`);
    if (sensors.battery_dip != null)         lines.push(`#define BATTERY_DIP ${numFmt(sensors.battery_dip)}`);
  } else if (sensors.battery_monitor === "INA219") {
    lines.push(`#define USE_INA219`);
  }

  // Environmental barometer — BMP280 & BME280 share the macro; the driver
  // reads the chip id at runtime and only publishes /humidity for a BME280.
  if (sensors.env && sensors.env !== "NONE") {
    lines.push(`#define USE_BMP280`);
  }

  if (sensors.sonar && pins.sonar) {
    lines.push(
      `#define USE_SONAR`,
      `#define TRIG_PIN ${pins.sonar.trig || 0}`,
      `#define ECHO_PIN ${pins.sonar.echo || 0}`
    );
  }

  // I2C bus + BOARD_INIT. SDA_PIN/SCL_PIN are ONLY consulted through the
  // BOARD_INIT macro (firmware.ino: `#ifdef BOARD_INIT ... #else Wire.begin();
  // #endif` — firmware is never touched for this). BOARD_INIT is synthesised
  // only when the user has defined I2C pins; a bare module leaves SDA/SCL
  // unset and no BOARD_INIT is emitted (default Wire.begin() applies).
  const i2c = pins.i2c || {};
  const i2cUsed = (sensors.imu && sensors.imu !== "FAKE" && sensors.imu !== "FAKE_IMU" && sensors.imu !== "NONE")
               || (sensors.mag && sensors.mag !== "NONE")
               || sensors.battery_monitor === "INA219"
               || (sensors.env && sensors.env !== "NONE");
  const sda = i2c.sda, scl = i2c.scl;
  if (i2cUsed && sda !== undefined && scl !== undefined && sda >= 0 && scl >= 0) {
    lines.push(``, `// I2C Bus`, `#define SDA_PIN ${sda}`, `#define SCL_PIN ${scl}`);
    const bi = [`#define BOARD_INIT { \\`];
    bi.push(`    Wire.begin(SDA_PIN, SCL_PIN); \\`);
    bi.push(`    Wire.setClock(400000); \\`);
    bi.push(`}`);
    lines.push(...bi);
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

  // ROS 2 topic namespace prefix (firmware.ino concatenates it onto each
  // topic string literal; #ifndef-gated so blank == bare topic names).
  const topicPrefix = (spec.advanced?.topic_prefix || "").trim();
  if (topicPrefix) {
    const tp = topicPrefix.endsWith("/") ? topicPrefix : topicPrefix + "/";
    lines.push(``, `// ROS 2 Topic Namespace`, `#define TOPIC_PREFIX "${tp}"`);
  }

  // Hardware Task Watchdog (WDT)
  if (spec.watchdog && (spec.watchdog.enabled || spec.watchdog > 0)) {
    const to = spec.watchdog.timeout_sec || (typeof spec.watchdog === "number" ? spec.watchdog : 60);
    lines.push(``, `// Hardware Task Watchdog`, `#define WDT_TIMEOUT ${to} // Seconds`);
  } else {
    lines.push(``, `// #define WDT_TIMEOUT 60 // Hardware Task Watchdog disabled`);
  }


  // IPv4 "a.b.c.d" -> C brace-init `{ a, b, c, d }` (the form every stock
  // header uses; converts cleanly to IPAddress for AGENT_IP / SYSLOG_SERVER
  // / LIDAR_SERVER).
  const ipC = (s, fb) => {
    const p = String(s || fb || "192.168.1.100").split(".").map(x => x.trim());
    return p.length === 4 && p.every(x => /^\d+$/.test(x)) ? `{ ${p.join(", ")} }` : "{ 192, 168, 1, 100 }";
  };
  const adv = spec.advanced || {};

  // Never emit WIFI_AP_LIST with a placeholder SSID — initWifis() blocks at
  // boot until it joins, so a placeholder would brick a bare board.
  const _haveRealSsid = wifi.ssid && wifi.ssid.trim() && wifi.ssid.trim() !== "YOUR_WIFI_SSID";

  // For a WiFi-UDP transport the PlatformIO env pins board_microros_transport =
  // wifi, so firmware.ino compiles `set_microros_net_transports(AGENT_IP,
  // AGENT_PORT)` unconditionally — those two macros MUST exist even before the
  // user fills in credentials. They are not secrets: emit compile-safe
  // #ifndef-guarded defaults here, and let the git-ignored wifi_config.h
  // override them once real values are entered.
  if (isWifiTransport) {
    const agentIp = adv.agent_ip || wifi.agent_ip || "192.168.1.100";
    lines.push(
      ``,
      `// micro-ROS WiFi transport (credentials & host IPs -> git-ignored wifi_config.h)`,
      `#if __has_include("wifi_config.h")`,
      `  #include "wifi_config.h"`,
      `#endif`,
      `#define USE_WIFI`,
      `#ifndef AGENT_IP`,
      `  #define AGENT_IP ${ipC(agentIp)}`,
      `#endif`,
      `#ifndef AGENT_PORT`,
      `  #define AGENT_PORT ${wifi.agent_port || 8888}`,
      `#endif`
    );
  }

  if (enableWifiNet && _haveRealSsid) {
    // Secrets (WIFI_AP_LIST, AGENT_IP, SYSLOG_SERVER, LIDAR_SERVER, OTA_PASSWORD)
    // live ONLY in the git-ignored config/custom/wifi_config.h — this tracked
    // header carries the non-secret feature flags and pulls the rest in.
    if (!isWifiTransport) {
      lines.push(
        ``,
        `// Background WiFi (OTA & Syslog telemetry while micro-ROS runs on Serial)`,
        `// Credentials & host IPs are in the git-ignored config/custom/wifi_config.h`,
        `// (regenerated by the deploy from your form; template: wifi_config.h.template).`,
        `#if __has_include("wifi_config.h")`,
        `  #include "wifi_config.h"`,
        `#endif`
      );
    }
    lines.push(`#define USE_ARDUINO_OTA`);
    lines.push(`#define OTA_HOSTNAME "${adv.ota_hostname || spec.robot_name || "linorobot2"}"`);
    lines.push(
      `#define USE_SYSLOG`,
      `#define SYSLOG_PORT ${adv.syslog_port || 514}`,
      `#define DEVICE_HOSTNAME "${spec.robot_name || "robot"}"`,
      `#define APP_NAME "hardware"`
    );
    if (adv.wifi_monitor && adv.wifi_monitor > 0) {
      lines.push(`#define WIFI_MONITOR ${adv.wifi_monitor} // min. period to send WiFi RSSI to syslog`);
    }
    // LiDAR-over-UDP forwarding — only when the user ticked "Forward LiDAR
    // over UDP" (a non-blank IP field default must NOT enable it; USE_LIDAR_UDP
    // pulls in ESP32-only HardwareSerial APIs that break the Pico build).
    if (spec.telemetry?.use_lidar_udp) {
      lines.push(
        `#define USE_LIDAR_UDP`,
        `#define LIDAR_PORT ${adv.lidar_port || 8889}`,
        `#define LIDAR_BAUDRATE ${adv.lidar_baudrate || 230400}`,
        `#define LIDAR_SERIAL ${adv.lidar_serial != null ? adv.lidar_serial : 1} // UART number`,
        `#define LIDAR_RXD ${adv.lidar_rxd != null ? adv.lidar_rxd : -1}`
      );
    }
  }

  // Simulation and the dual-core control task do not mix: the fake LiDAR
  // touches the simulated pose and its UART from the loop while the control
  // task drives the same pose from the other core, with no lock between them,
  // and the board crash-loops. Simulation wins -- it is the whole point of
  // this config -- so the second core is left alone here.
  const simEnabled = !!(spec.simulation?.fake_wheel || spec.simulation?.fake_ld19);
  if (spec.dual_core !== false && !simEnabled) {
    lines.push(
      ``,
      `// Dual-Core Task Allocation (Strict 50Hz Real-Time PID on Core 1)`,
      `#define USE_DUAL_CORE`
    );
  }

  // Simulation & Fake Hardware Modes
  const sim = spec.simulation || {};
  // Mirror the warning the Python generator emits: a simulated config is
  // indistinguishable from a real one at a glance, and publishes plausible
  // topics, so a wired robot flashed with it sits still while /odom claims
  // otherwise. Keep it above the defines it refers to.
  if (sim.fake_wheel || sim.fake_ld19) {
    lines.push(
      ``,
      `// ===========================================================================`,
      `// SIMULATION MODE IS ENABLED -- THIS IS NOT A REAL ROBOT CONFIG`,
      `// Comment out the USE_FAKE_* defines below before flashing a robot whose`,
      `// motors, encoders or LiDAR are actually wired, or they will be ignored.`,
      `// ===========================================================================`
    );
  }
  if (sim.fake_wheel) {
    lines.push(
      ``,
      `// Drivetrain Simulation (Bare Module Mode)`,
      `// Simulates DC motor back-EMF, inertia, and encoders on-board.`,
      `#define USE_FAKE_WHEEL`
    );
    if (sim.robot_mass != null) {
      lines.push(`#define ROBOT_WEIGHT ${sim.robot_mass}`);
    }
    if (sim.wheel_noise_rpm != null) {
      lines.push(`#define FAKE_WHEEL_NOISE_RPM ${sim.wheel_noise_rpm}`);
    }
  }

  // The emulated scan leaves the board either over the LiDAR UART pin or over
  // WiFi UDP. With neither, there is nowhere for it to go, so the emulator is
  // not enabled at all: turning it on would burn the loop building packets and
  // writing them into an unconnected pad, while the board reported success and
  // no scan ever reached ROS. The pin is unset by default because on a stock
  // board nothing is wired to it -- wire it, set the pin, and this switches on.
  const lidarRxd = spec.pins?.lidar_rxd ?? spec.advanced?.lidar_rxd;
  // A negative pin is "not wired", not a pin number.
  const lidarPinWired = lidarRxd != null && lidarRxd >= 0;
  const lidarOverUdp = !!spec.telemetry?.use_lidar_udp;
  const ld19Routed = lidarPinWired || lidarOverUdp;

  if (sim.fake_ld19 && !ld19Routed) {
    lines.push(
      ``,
      `// Fake LD19 requested but NOT enabled: no LiDAR UART pin is wired and`,
      `// LiDAR-over-UDP is off, so the simulated scan would have no route out.`,
      `// Any route over the serial pin needs a physical wire: fake LD19`,
      `// transmits the scan on it, and a real LD19 forwarded over UDP must`,
      `// be wired into it, since UDP only carries what the board receives.`,
      `// Fake LD19 over WiFi UDP is the one option needing no wire at all.`,
      `// Wire IO4 (gendrv) and set the pin, or turn on LiDAR over WiFi UDP.`
    );
  }

  if (sim.fake_ld19 && ld19Routed) {
    lines.push(
      ``,
      `// Fake LD19 LiDAR Simulation Mode`,
      `// Embeds a real-time 2D raycast LD19 emulator inside the firmware.`,
      `// Outputs standard 47-byte packets @ 230400 baud on LIDAR_RXD / UDP.`,
      `#define USE_FAKE_LD19`
    );
    if (lidarPinWired) {
      lines.push(`#define LIDAR_RXD ${lidarRxd}`);
    }
    if (sim.map_width != null) {
      lines.push(`#define FAKE_MAP_WIDTH ${sim.map_width}`);
    }
    if (sim.map_height != null) {
      lines.push(`#define FAKE_MAP_HEIGHT ${sim.map_height}`);
    }
    if (sim.wall_obstacle) {
      lines.push(`#define FAKE_WALL_OBSTACLE 1`);
      if (sim.wall_x1 != null) lines.push(`#define FAKE_WALL_X1 ${sim.wall_x1}`);
      if (sim.wall_y1 != null) lines.push(`#define FAKE_WALL_Y1 ${sim.wall_y1}`);
      if (sim.wall_x2 != null) lines.push(`#define FAKE_WALL_X2 ${sim.wall_x2}`);
      if (sim.wall_y2 != null) lines.push(`#define FAKE_WALL_Y2 ${sim.wall_y2}`);
    }
    if (sim.fake_sonar !== false) {
      lines.push(
        ``,
        `// Simulated forward sonar, taken from the nearest return in a cone of`,
        `// the same raycast that feeds the scan -- no pins, no wiring.`,
        `#define USE_FAKE_SONAR`,
        `// Firmware-side hazard stop: blocks forward motion inside`,
        `// SAFETY_STOP_RANGE and publishes the state on /safety_stop. Reverse is`,
        `// left available so the robot can back out of what it stopped for.`,
        `#define USE_SAFETY_STOP`
      );
    }
  }

  lines.push(``, `#endif // ${nameUpper}_CONFIG_H`, ``);
  return lines.join("\n");
}

// Secrets file — WiFi credentials + host IPs. Written to config/custom/wifi_config.h,
// which is git-ignored, so `git add config/` never commits them. Empty string
// when the spec has no real WiFi SSID (nothing to write).
function generateWifiConfig(spec) {
  const wifi = spec.wifi_settings || {};
  const adv = spec.advanced || {};
  const ssid = (wifi.ssid || "").trim();
  if (!ssid || ssid === "YOUR_WIFI_SSID") return "";
  const transport = (spec.transport || "SERIAL").toUpperCase();
  const isWifiTransport = /WIFI|UDP/.test(transport);
  const enableWifiNet = isWifiTransport || !!spec.enable_ota_syslog;
  if (!enableWifiNet) return "";
  const password = wifi.password || "";
  const ipC = (s, fb) => {
    const p = String(s || fb || "192.168.1.100").split(".").map(x => x.trim());
    return p.length === 4 && p.every(x => /^\d+$/.test(x)) ? `{ ${p.join(", ")} }` : "{ 192, 168, 1, 100 }";
  };
  const agentIpStr = adv.agent_ip || wifi.agent_ip || "192.168.1.100";
  const L = [
    `// config/custom/wifi_config.h  —  generated by the Robot Configuration Engine`,
    `// GIT-IGNORED: your WiFi credentials and host IPs are never committed.`,
    `// Delete this file to let the studio regenerate it, or edit it by hand`,
    `// (multiple APs, static IPs, …). Template: wifi_config.h.template.`,
    ``,
    `#define WIFI_AP_LIST { { "${ssid}", "${password}" }, { NULL, NULL } }`,
  ];
  if (isWifiTransport) L.push(`#define AGENT_IP ${ipC(agentIpStr)}`);
  L.push(`#define SYSLOG_SERVER ${ipC(adv.syslog_ip, agentIpStr)}`);
  if (spec.telemetry?.use_lidar_udp) L.push(`#define LIDAR_SERVER ${ipC(adv.lidar_ip, agentIpStr)}`);
  if (adv.ota_password) L.push(`#define OTA_PASSWORD "${adv.ota_password}"`);
  L.push(``);
  return L.join("\n");
}

function generatePlatformioEnv(spec) {
  const name = spec.robot_name || "my_robot";
  const mcu = (spec.mcu || "PICO2").toUpperCase();
  const cfgMacro = `USE_${name.toUpperCase()}_CONFIG`;
  const isWifi = /WIFI|UDP/i.test(spec.transport || "");
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
    const otaIp = (spec.advanced?.ota_enable && spec.advanced?.ota_ip) ? spec.advanced.ota_ip : "";
    const upl = otaIp
      ? `monitor_port = /dev/ttyUSB0\nupload_port = ${otaIp}\nupload_protocol = espota`
      : `monitor_port = /dev/ttyUSB0\nupload_port = /dev/ttyUSB0\nupload_protocol = esptool`;
    return `[env:${name}]
platform = espressif32
board = nodemcu-32s
board_build.f_flash = 80000000L
board_build.flash_mode = qio
board_build.partitions = min_spiffs.csv
monitor_speed = ${firmwareBaud(spec)}
${upl}
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
monitor_speed = ${firmwareBaud(spec)}
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
monitor_speed = ${firmwareBaud(spec)}
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
      const inA = m.in_a !== undefined ? m.in_a : m.pwm_l;
      const inB = m.in_b !== undefined ? m.in_b : m.en;
      rows.push(`| **Motor ${i} RPWM** | \`GPIO ${inA}\` | BTS7960 RPWM -> MOTOR${i}_IN_A | Forward PWM |`);
      rows.push(`| **Motor ${i} LPWM** | \`GPIO ${inB}\` | BTS7960 LPWM -> MOTOR${i}_IN_B | Reverse PWM |`);
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

// JSON Import File Handler (for saved spec JSON files)
async function handleImportFile(e) {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = async (event) => {
    try {
      const content = event.target.result;
      const imported = JSON.parse(content);
      baseConfigSpec = JSON.parse(JSON.stringify(imported));
      currentSpec = imported;
      populateFormFromSpec(currentSpec);
      recomputeAll();
      showToast(`✅ Successfully imported '${imported.robot_name || file.name}'!`);
    } catch (err) {
      alert('Error importing JSON: ' + err.message);
    }
  };
  reader.readAsText(file);
}

// ─── Board Config Selector (reads firmware/platformio.ini via server) ───────

let boardList = [];

async function openBoardSelector() {
  const modal = document.getElementById('modal-board-select');
  const sel = document.getElementById('board-select');
  const info = document.getElementById('board-select-info');
  const confirmBtn = document.getElementById('btn-board-confirm');

  modal.style.display = 'flex';
  sel.innerHTML = '<option value="">Loading…</option>';
  confirmBtn.disabled = true;
  info.textContent = '';

  try {
    const res = await fetch('/api/boards');
    const data = await res.json();
    boardList = (data.boards || []).filter(b => b.config_exists);

    if (boardList.length === 0) {
      sel.innerHTML = '<option value="">No board configs found</option>';
      info.textContent = 'No *_config.h files found in config/custom/. Check your repository.';
      return;
    }

    sel.innerHTML = '<option value="">— Select a board environment —</option>' +
      boardList.map(b => {
        const desc = (b.display_name && b.display_name !== b.env) ? ` — ${b.display_name}` : '';
        return `<option value="${b.env}">${b.env}${desc}  [${b.transport}]</option>`;
      }).join('');

    sel.addEventListener('change', () => {
      const chosen = boardList.find(b => b.env === sel.value);
      if (chosen) {
        const speed = chosen.monitor_speed ? ` · ${chosen.monitor_speed} baud` : '';
        const wifi  = chosen.wifi ? ' · WiFi transport' : ' · Serial transport';
        info.innerHTML = `<b>[env:${chosen.env}]</b>${chosen.display_name && chosen.display_name !== chosen.env ? ` · ${chosen.display_name}` : ''}${speed}${wifi}<br>
          <span style="opacity:.7;">Config: <code>${chosen.config_file}</code></span>`;
        confirmBtn.disabled = false;
      } else {
        info.textContent = '';
        confirmBtn.disabled = true;
      }
    });
  } catch (err) {
    sel.innerHTML = '<option value="">Error loading boards</option>';
    info.textContent = 'Could not reach /api/boards — is the server running?';
  }
}

async function loadBoardConfig(envName) {
  try {
    showToast('⏳ Loading board configuration…');
    const res = await fetch('/api/load_board_config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ env: envName })
    });
    const data = await res.json();
    if (data.status === 'ok' && data.spec) {
      baseConfigSpec = JSON.parse(JSON.stringify(data.spec));
      currentSpec = data.spec;
      populateFormFromSpec(currentSpec);
      recomputeAll();
      const board = data.board || {};
      showToast(`✅ Loaded '${envName}' (${board.display_name || envName})`);
    } else {
      throw new Error(data.error || 'Failed to load board config');
    }
  } catch (err) {
    alert('Error loading board config: ' + err.message);
  }
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
            applyDetectedChipToBaudUi(guessed);
            defaultSimulationForDetectedModule(guessed.chipName);
            const mcuSelect = document.getElementById("cfg-mcu");
            if (mcuSelect && mcuSelect.value !== guessed.mcu) {
              mcuSelect.value = guessed.mcu;
              mcuSelect.dispatchEvent(new Event("change"));
            }
            applyMcuRobotName(guessed.mcu);
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
    if (flashPortInput && (!flashPortInput.value || flashPortInput.dataset.autoManaged === "true")) {
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
        // Load the all-pins-(-1) bare-module template so nothing is driven on a
        // freshly plugged board, then stamp the detected MCU + a unique name.
        loadPreset("bare");
        const mcuSelect = document.getElementById("cfg-mcu");
        if (mcuSelect) {
          mcuSelect.value = guessed.mcu;
          mcuSelect.dispatchEvent(new Event("change"));
        }
        applyMcuRobotName(guessed.mcu);
        applyDetectedChipToBaudUi(guessed);
            defaultSimulationForDetectedModule(guessed.chipName);
        recomputeAll();
        showToast(`⚡ Auto-detected Hardware: ${guessed.chipName} on ${ports[0]} (Bare Module Default Loaded — all pins N/C)`);
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
  const rosDistro = document.getElementById("hdr-ros-distro")?.value || document.getElementById("auto-ros-distro")?.value || "jazzy";
  const buildEngine = document.getElementById("hdr-build-engine")?.value || (document.getElementById("chk-docker-builder")?.checked ? "docker" : "native");
  const agentEngine = document.getElementById("hdr-agent-engine")?.value || "docker";
  const regMode = document.getElementById("hdr-container-registry")?.value || "auto";
  const customReg = (document.getElementById("hdr-custom-registry")?.value || "").trim();
  const containerRegistry = (regMode === "custom" && customReg) ? customReg : regMode;
  const remoteFlashTarget = (document.getElementById("hdr-remote-flash")?.value || document.getElementById("remote-ssh-host")?.value || "").trim();

  return {
    os: document.getElementById("auto-os-select")?.value || "ubuntu_2404",
    env: document.getElementById("auto-env-select")?.value || "native",
    arch: document.getElementById("auto-arch-select")?.value || "x86_64",
    installPio: document.getElementById("chk-install-pio")?.checked ?? true,
    installUdev: document.getElementById("chk-install-udev")?.checked ?? true,
    installDialout: document.getElementById("chk-install-dialout")?.checked ?? true,
    installBuildTools: document.getElementById("chk-install-buildtools")?.checked ?? true,
    rosDistro: rosDistro,
    buildEngine: buildEngine,
    agentEngine: agentEngine,
    containerRegistry: containerRegistry,
    customRegistry: customReg,
    remoteFlashTarget: remoteFlashTarget,
    rosType: document.getElementById("auto-ros-type")?.value || "desktop",
    buildMicrorosAgent: document.getElementById("chk-build-microros-agent")?.checked ?? true,
    gitBranch: document.getElementById("auto-git-branch")?.value.trim() || "",
    gitCommitMsg: document.getElementById("auto-git-commit-msg")?.value.trim() || `feat(config): add configuration for ${currentSpec.robot_name || "robot"}`,
    mergeHeader: document.getElementById("chk-merge-header")?.checked ?? true,
    mergePioFirmware: document.getElementById("chk-merge-pio-firmware")?.checked ?? true,
    mergeUrdf: document.getElementById("chk-merge-urdf")?.checked ?? true,
    autoCommit: document.getElementById("chk-auto-commit")?.checked ?? true,
    flashTarget: document.getElementById("auto-flash-target")?.value || "firmware",
    flashPort: (() => {
      const otaOn = document.getElementById("cfg-ota-enable")?.checked;
      const otaIp = (document.getElementById("cfg-ota-ip")?.value || "").trim();
      if (otaOn && otaIp) return otaIp;   // espota target
      return document.getElementById("auto-flash-port")?.value.trim() || "/dev/ttyACM0";
    })(),
    otaFlash: !!document.getElementById("cfg-ota-enable")?.checked &&
              !!(document.getElementById("cfg-ota-ip")?.value || "").trim(),
    otaPass: (document.getElementById("cfg-ota-password")?.value || "").trim()
  };
}

// The firmware serial BAUDRATE chosen on Tab 1 (#cfg-baudrate). Single source
// of truth for the micro-ROS agent -b flag and the console serial monitor.
function firmwareBaud(spec) {
  spec = spec || (typeof currentSpec !== "undefined" ? currentSpec : null) || {};
  return spec.baudrate
    || spec.telemetry?.baudrate
    || ((spec.mcu || "").toUpperCase() === "GENDRV" ? 1500000 : 921600);
}

// Update Dynamic Previews across Tab 5
function updateAutomationPreviews() {
  const opts = readAutomationOptions();
  const spec = currentSpec;
  const robotName = spec.robot_name || "scout_pico2";

  // The git branch field tracks the Robot SBC's current branch (see
  // applyCurrentBranch), not the robot name — do not overwrite it here.
  const gitCommitInput = document.getElementById("auto-git-commit-msg");
  if (gitCommitInput && (!gitCommitInput.value || gitCommitInput.dataset.autoManaged === "true")) {
    gitCommitInput.value = `feat(config): add configuration for ${robotName}`;
    gitCommitInput.dataset.autoManaged = "true";
  }

  // OTA mDNS hostname mirrors the robot name until the user edits it
  const otaHostInput = document.getElementById("cfg-ota-hostname");
  if (otaHostInput && (!otaHostInput.value || otaHostInput.dataset.autoManaged === "true")) {
    otaHostInput.value = robotName;
    otaHostInput.dataset.autoManaged = "true";
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
    // Agent -b always matches the firmware BAUDRATE chosen on Tab 1.
    const baud = firmwareBaud(spec);
    const isWiFi = /WIFI|UDP/i.test(spec.transport || "");
    const agentEngine = document.getElementById("hdr-agent-engine")?.value || "docker";
    agentCmdEl.innerText = microRosAgentCmd(isWiFi ? "udp" : "serial", { distro, port, baud, agentEngine });
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

// Idempotent shell snippet that GUARANTEES a working `pio` on PATH, installing
// PlatformIO Core (and the few OS packages its installer needs) the first time.
// Standalone actions — I2C auto-detect, ADC calibrate — otherwise just run a bare
// `pio ...` and die with "command not found" (exit 127) on a fresh SBC / distrobox
// that has never run a Full Deploy. Mirrors Phase 1 of generateDeployScript.
// Meant to be the first lines of a command passed to executeCommandInTerminal().
function ensurePioToolchainSnippet() {
  return [
    `export PATH="$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH"`,
    `_pio_ok() { command -v pio >/dev/null 2>&1 && pio --version >/dev/null 2>&1; }`,
    `if ! _pio_ok; then`,
    `  echo ">>> PlatformIO not found - installing build toolchain (first run only, ~1-2 min)..."`,
    `  if command -v pio >/dev/null 2>&1; then echo ">>> (removing a broken PlatformIO venv)"; rm -rf "$HOME/.platformio/penv"; hash -r; fi`,
    `  if ! python3 -c "import venv, ensurepip" >/dev/null 2>&1 || ! command -v curl >/dev/null 2>&1; then`,
    `    if command -v apt-get >/dev/null 2>&1; then`,
    `      (sudo -n apt-get update -y 2>/dev/null || sudo apt-get update -y) || true`,
    `      sudo -n apt-get install -y python3 python3-venv python3-pip curl git 2>/dev/null || sudo apt-get install -y python3 python3-venv python3-pip curl git || true`,
    `    elif command -v dnf >/dev/null 2>&1; then`,
    `      (sudo -n dnf install -y python3 python3-pip curl git 2>/dev/null || sudo dnf install -y python3 python3-pip curl git) || true`,
    `    else`,
    `      echo ">>> WARN: no apt-get/dnf - assuming python3+curl already present"`,
    `    fi`,
    `  fi`,
    `  curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py || { echo "ERROR: cannot download the PlatformIO installer (network?)"; exit 1; }`,
    `  python3 /tmp/get-platformio.py || { echo "ERROR: PlatformIO Core installation failed"; exit 1; }`,
    `  export PATH="$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH"; hash -r`,
    `fi`,
    `_pio_ok || { echo "ERROR: PlatformIO Core still not working ('pio --version' fails)"; exit 1; }`,
    `echo ">>> PlatformIO ready: $(pio --version 2>/dev/null || echo installed)"`,
  ].join("\n");
}

// micro-ROS agent launch with graceful degradation:
//   native `ros2 run micro_ros_agent`  (from /opt/ros or ~/uros_ws)
//   -> apt `ros-<distro>-micro-ros-agent`  (if the ROS 2 apt repo is present)
//   -> docker / podman `microros/micro-ros-agent:<distro>`  (-> :rolling tag)
//   -> clear error pointing at the "Install ROS 2" button.
// mode: "serial" | "multiserial" | "udp".
// The container registry is a user preference (header dropdown / Automation
// tab), never a hardcoded host: a LAN or cluster registry is site specific, so
// there is no sensible default to ship. Returns the hosts to probe, empty when
// the user picked Docker Hub or left the custom host blank.
function configuredRegistryHosts(opt) {
  opt = opt || {};
  const mode = opt.containerRegistry
    || document.getElementById("hdr-container-registry")?.value
    || "auto";
  if (mode === "dockerhub") return [];
  const host = (opt.customRegistry
    || document.getElementById("hdr-custom-registry")?.value
    || "").trim();
  if (mode === "cluster" || mode === "custom" || mode === "auto") {
    return host ? [host] : [];
  }
  // anything else in the field is itself a registry host
  return [mode];
}

function microRosAgentCmd(mode, opt) {
  const distro   = opt.distro || "jazzy";
  const baud     = opt.baud || 921600;
  const udpPort  = opt.udpPort || 8888;
  const port     = (opt.port || "/dev/ttyUSB0").trim();
  const ports    = (opt.ports || "/dev/ttyACM0 /dev/ttyUSB0").trim();

  let agentArgs, deviceFlags, permPorts;
  if (mode === "multiserial") {
    agentArgs = `multiserial --devs "${ports}" -b ${baud}`;
    deviceFlags = ports.split(/\s+/).filter(Boolean).map(d => `--device ${d}`).join(" ");
    permPorts = ports;
  } else if (mode === "udp") {
    agentArgs = `udp4 --port ${udpPort}`;
    deviceFlags = "";
    permPorts = "";
  } else { // serial
    agentArgs = `serial --dev ${port} -b ${baud}`;
    deviceFlags = `--device ${port}`;
    permPorts = port;
  }

  const argsEcho = agentArgs.replace(/"/g, '\\"');   // safe inside a "…" echo
  const preflightScript = mode === "udp"
    ? `echo ">>> Checking if micro-ROS UDP port ${udpPort} is in use..."\n` +
      `_PIDS=$(fuser "${udpPort}/udp" 2>/dev/null || true)\n` +
      `_CONT_DK=$(command -v docker >/dev/null 2>&1 && docker ps --filter "name=uros_agent_${mode}" --filter "name=microros_agent" -q 2>/dev/null || true)\n` +
      `_CONT_PM=$(command -v podman >/dev/null 2>&1 && podman ps --filter "name=uros_agent_${mode}" --filter "name=microros_agent" -q 2>/dev/null || true)\n` +
      `if [ -n "$_PIDS" ] || [ -n "$_CONT_DK" ] || [ -n "$_CONT_PM" ]; then\n` +
      `  echo ">>> [Port Conflict] micro-ROS UDP :${udpPort} in use (PIDs: \${_PIDS:-none}). Releasing..."\n` +
      `  [ -n "$_CONT_DK" ] && docker stop $_CONT_DK >/dev/null 2>&1 || true\n` +
      `  [ -n "$_CONT_PM" ] && podman stop $_CONT_PM >/dev/null 2>&1 || true\n` +
      `  [ -n "$_PIDS" ] && kill -TERM $_PIDS 2>/dev/null || true\n` +
      `  pkill -f "[m]icro_ros_agent.*udp.*${udpPort}" 2>/dev/null || true\n` +
      `  sleep 0.5\n` +
      `fi`
    : `for _chk_port in ${permPorts}; do\n` +
      `  if [ -e "$_chk_port" ]; then\n` +
      `    _PIDS=$(fuser "$_chk_port" 2>/dev/null || true)\n` +
      `    _CONT_DK=$(command -v docker >/dev/null 2>&1 && docker ps --filter "name=uros_agent" --filter "name=microros_agent" -q 2>/dev/null || true)\n` +
      `    _CONT_PM=$(command -v podman >/dev/null 2>&1 && podman ps --filter "name=uros_agent" --filter "name=microros_agent" -q 2>/dev/null || true)\n` +
      `    if [ -n "$_PIDS" ] || [ -n "$_CONT_DK" ] || [ -n "$_CONT_PM" ]; then\n` +
      `      echo ">>> [Port Conflict] Serial port '$_chk_port' occupied (PIDs: \${_PIDS:-none}). Releasing..."\n` +
      `      [ -n "$_CONT_DK" ] && docker stop $_CONT_DK >/dev/null 2>&1 || true\n` +
      `      [ -n "$_CONT_PM" ] && podman stop $_CONT_PM >/dev/null 2>&1 || true\n` +
      `      if [ -n "$_PIDS" ]; then\n` +
      `        fuser -k -TERM "$_chk_port" 2>/dev/null || true\n` +
      `        sleep 0.5\n` +
      `        fuser -k -KILL "$_chk_port" 2>/dev/null || true\n` +
      `      fi\n` +
      `      pkill -f "[m]icro_ros_agent.*\${_chk_port##*/}" 2>/dev/null || true\n` +
      `      pkill -f "[m]initerm.*\${_chk_port##*/}" 2>/dev/null || true\n` +
      `      pkill -f "[s]creen.*\${_chk_port##*/}" 2>/dev/null || true\n` +
      `      sleep 0.5\n` +
      `    fi\n` +
      `  fi\n` +
      `done`;

  return [
    `DISTRO="${distro}"`,
    `# 0. Pre-flight Port Conflict Check & Automatic Release`,
    preflightScript,
    permPorts
      ? `for _d in ${permPorts}; do [ -e "$_d" ] && [ ! -w "$_d" ] && sudo chmod a+rw "$_d" 2>/dev/null || true; done`
      : `# udp transport - no serial port to free`,
    ``,
    `# 1. Native ROS 2 agent (from /opt/ros/<distro> or ~/uros_ws)`,
    `if [ -f "/opt/ros/$DISTRO/setup.bash" ]; then`,
    `  source "/opt/ros/$DISTRO/setup.bash"`,
    `  [ -f "$HOME/uros_ws/install/setup.bash" ] && source "$HOME/uros_ws/install/setup.bash"`,
    `  if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then`,
    `    echo ">>> micro-ROS agent: native 'ros2 run' (ROS 2 $DISTRO)"`,
    `    exec ros2 run micro_ros_agent micro_ros_agent ${agentArgs}`,
    `  fi`,
    `  if command -v apt-get >/dev/null 2>&1; then`,
    `    echo ">>> micro_ros_agent package missing - trying apt install ros-$DISTRO-micro-ros-agent ..."`,
    `    if sudo -n apt-get install -y "ros-$DISTRO-micro-ros-agent" 2>/dev/null || sudo apt-get install -y "ros-$DISTRO-micro-ros-agent"; then`,
    `      source "/opt/ros/$DISTRO/setup.bash"`,
    `      if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then`,
    `        echo ">>> micro-ROS agent: native 'ros2 run' (apt-installed)"`,
    `        exec ros2 run micro_ros_agent micro_ros_agent ${agentArgs}`,
    `      fi`,
    `    fi`,
    `  fi`,
    `fi`,
    ``,
    `# 2. Containerized Execution (Docker / Podman / Podman+systemd)`,
    `_ENGINE="${opt.agentEngine || 'docker'}"`,
    `# Registry hosts the user configured, most preferred first. Empty means`,
    `# "go straight to Docker Hub" -- there is no default LAN host to ship.`,
    `_REGS="${configuredRegistryHosts(opt).join(" ")}"`,
    `# Resolve the agent image: the configured registry first (so a LAN cache`,
    `# serves it and the pull stays local), then Docker Hub, and each of those`,
    `# for the running distro before falling back to the :rolling tag.`,
    `_resolve_agent_img() {`,
    `  local eng="$1" reg tag`,
    `  for reg in $_REGS ""; do`,
    `    for tag in "$DISTRO" rolling; do`,
    `      IMG="microros/micro-ros-agent:$tag"`,
    `      [ -n "$reg" ] && IMG="$reg/$IMG"`,
    `      $eng pull "$IMG" >/dev/null 2>&1 && return 0`,
    `    done`,
    `  done`,
    `  # nothing pulled; name it anyway so the run reports the real error`,
    `  IMG="microros/micro-ros-agent:$DISTRO"`,
    `  return 1`,
    `}`,
    `if [ "$_ENGINE" = "podman_systemd" ]; then`,
    `  echo ">>> micro-ROS agent: Podman + systemd user service ($DISTRO)"`,
    `  _resolve_agent_img podman || true`,
    `  podman run -d --replace --name "microros_agent_${mode}" --net=host ${deviceFlags} "$IMG" ${agentArgs}`,
    `  mkdir -p "$HOME/.config/systemd/user"`,
    `  podman generate systemd --new --name "microros_agent_${mode}" > "$HOME/.config/systemd/user/microros-agent.service" 2>/dev/null || true`,
    `  systemctl --user daemon-reload 2>/dev/null || true`,
    `  systemctl --user enable --now microros-agent.service 2>/dev/null || true`,
    `  loginctl enable-linger "$USER" 2>/dev/null || true`,
    `  echo ">>> micro-ROS agent running as persistent systemd user service: microros-agent.service"`,
    `  exit 0`,
    `elif [ "$_ENGINE" = "podman" ]; then`,
    `  echo ">>> micro-ROS agent: Podman rootless container ($DISTRO)"`,
    `  _resolve_agent_img podman || true`,
    `  exec podman run --rm --replace -it --name "uros_agent_${mode}" --net=host ${deviceFlags} "$IMG" ${agentArgs}`,
    `else`,
    `  _DK=""`,
    `  command -v docker >/dev/null 2>&1 && _DK=docker`,
    `  [ -z "$_DK" ] && command -v podman >/dev/null 2>&1 && _DK=podman`,
    `  if [ -n "$_DK" ]; then`,
    `    echo ">>> micro-ROS agent: $_DK container ($DISTRO)"`,
    `    _resolve_agent_img "$_DK" || true`,
    `    echo ">>> image: $IMG"`,
    `    $_DK rm -f "uros_agent_${mode}" 2>/dev/null || true`,
    `    exec $_DK run --rm --net=host --name "uros_agent_${mode}" ${deviceFlags} "$IMG" ${agentArgs}`,
    `  fi`,
    `fi`,
    ``,
    `echo "ERROR: no micro-ROS agent available (no ROS 2 $DISTRO, no docker/podman)."`,
    `echo "  fix: click 'Install ROS 2 Distribution & Packages' above, or install Docker, then retry."`,
    `echo "  manual: docker run --rm --net=host ${deviceFlags} microros/micro-ros-agent:$DISTRO ${argsEcho}"`,
    `exit 1`,
  ].join("\n");
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
      `        git clone -b ${distro} https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent || git clone -b rolling https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent`,
      `    fi`,
      `    if [ ! -d "src/micro_ros_msgs" ]; then`,
      `        git clone -b ${distro} https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs || git clone -b rolling https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs`,
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
// Search-and-merge, never overwrite: every place that writes
// config/custom/<name>_config.h ships the current spec as JSON and has the
// Robot SBC merge it — via the SAME parser.py/generator.py the test suite is
// built on — into whatever is already on disk (overlay/web-UI values win for
// anything they model; hand-edited or not-yet-modeled #defines on disk, and
// e.g. a previously calibrated ADC_LUT the current form doesn't carry, are
// left alone). A brand-new robot name just gets the fresh header — there is
// nothing to merge with yet.
function getConfigSyncScript(spec, name) {
  const specJson = JSON.stringify(spec);
  return [
    `mkdir -p config/custom urdf`,
    `cat << 'EOF_SPEC' > "/tmp/lr_spec_${name}.json"`,
    specJson,
    `EOF_SPEC`,
    `LR_ROBOT_NAME="${name}" LR_SPEC_JSON="/tmp/lr_spec_${name}.json" python3 << 'EOF_MERGE_PY'`,
    `import sys, json, os`,
    `sys.path.insert(0, "tools/robot_config_engine")`,
    `from parser import parse_header_to_spec, merge_configurations`,
    `from generator import generate_config_header`,
    `name = os.environ["LR_ROBOT_NAME"]`,
    `path = f"config/custom/{name}_config.h"`,
    `with open(os.environ["LR_SPEC_JSON"]) as f:`,
    `    overlay = json.load(f)`,
    `if os.path.exists(path):`,
    `    with open(path) as f:`,
    `        base = parse_header_to_spec(f.read())`,
    `    merged, changes = merge_configurations(base, overlay)`,
    `    if changes:`,
    `        print(f"[config-sync] merged {len(changes)} field(s) into existing {path}")`,
    `    else:`,
    `        print(f"[config-sync] {path} already matches — no changes")`,
    `else:`,
    `    merged = overlay`,
    `    print(f"[config-sync] creating new {path}")`,
    `header = generate_config_header(merged)`,
    `with open(path, "w") as f:`,
    `    f.write(header)`,
    `EOF_MERGE_PY`,
    `rm -f "/tmp/lr_spec_${name}.json"`,
  ];
}

function getMergeAndCommitCmd(spec, opts) {
  const name = spec.robot_name || "my_robot";
  const nameUpper = name.toUpperCase();
  const branch = (opts.gitBranch || "").trim();
  const commitMsg = opts.gitCommitMsg || `feat(config): add configuration for ${name}`;
  const pioSection = generatePlatformioEnv(spec);
  const urdfContent = generateUrdfXacro(spec);

  const lines = [
    `set -e`,
    ...(branch ? [
      `# 1. Switch to (or create) the target branch`,
      `if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then`,
      `    git checkout -b "${branch}" 2>/dev/null || git checkout "${branch}"`,
      `fi`,
      ``,
    ] : []),
    `# 2. Ingest Custom Header (search-and-merge with any existing file)`,
    ...getConfigSyncScript(spec, name),
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
    `    echo "Configuration files already up to date on branch $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')."`,
    `fi`
  ];

  return lines.join("\n");
}

// Full All-In-One Bash Deploy Script Generator
function generateDeployScript(spec, opts) {
  const name = spec.robot_name || "my_robot";
  const nameUpper = name.toUpperCase();
  const mcu = (spec.mcu || "PICO2").toUpperCase();
  const commitMsg = opts.gitCommitMsg || `feat(config): add configuration for ${name}`;
  const target = opts.flashTarget || "firmware";
  const port = opts.flashPort || "/dev/ttyACM0";
  const otaAuth = (opts.otaFlash && opts.otaPass) ? ` --upload-flags "--auth=${opts.otaPass}"` : "";
  const wifiConfigContent = generateWifiConfig(spec);
  const pioSection = generatePlatformioEnv(spec);
  const urdfContent = generateUrdfXacro(spec);
  // Escape a value for safe interpolation inside a double-quoted bash string
  const shDq = (s) => String(s).replace(/\\/g, "\\\\").replace(/"/g, '\\"').replace(/`/g, "\\`").replace(/\$/g, "\\$");
  const commitMsgEsc = shDq(commitMsg);
  const rosPkg = opts.rosType === "desktop" ? `ros-${opts.rosDistro}-desktop` : `ros-${opts.rosDistro}-ros-base`;
  const isContainerBuild = opts.buildEngine === "docker" || opts.buildEngine === "podman";
  const buildRegistryProbeList = () =>
    configuredRegistryHosts(opts).map(h => `"${h}"`);
  const regListStr = buildRegistryProbeList().join(" ");

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

# Grant read/write on USB serial devices when no udev rule / 'dialout' group does.
# Keeps flashing and the micro-ROS agent working on a fresh SBC without a reboot.
ensure_serial_rw() {
    local want="\$1" d
    for d in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "\$d" ] || continue
        [ -w "\$d" ] && continue
        warn "No write access to \$d (missing udev rule?) - running 'sudo chmod a+rw \$d'"
        sudo chmod a+rw "\$d" 2>/dev/null || true
    done
    if [ -n "\$want" ] && [ "\$want" != "AUTO" ] && [ "\$want" != "BOOTSEL" ] && [ -e "\$want" ] && [ ! -w "\$want" ]; then
        sudo chmod a+rw "\$want" 2>/dev/null || true
    fi
}

# Cap build parallelism on low-memory hosts (Raspberry Pi 4/5 4 GB, etc.) so the
# first libmicroros / micro-ROS-Agent compile does not OOM. 6 GB+ builds at full speed.
_MEM_KB=\$(awk '/MemTotal/{print \$2}' /proc/meminfo 2>/dev/null || echo 8000000)
if [ "\${_MEM_KB:-8000000}" -lt 6000000 ]; then
    export MAKEFLAGS="-j2"
    export PLATFORMIO_BUILD_JOBS=2
    COLCON_JOBS="--parallel-workers 2"
    info "Low memory (\$((_MEM_KB/1024)) MB) detected - capping builds at 2 parallel jobs to avoid OOM."
else
    COLCON_JOBS=""
fi

info "========================================================="
info "  Linorobot2 Deployment Engine: ${name}"
info "  Target MCU: ${mcu} | Mode: ${target}"
info "========================================================="

# -----------------------------------------------------------------------------
# Phase 1: Toolchain & Dependency Verification
# -----------------------------------------------------------------------------
info "Step 1/9: Checking build toolchains and dependencies..."
${(opts.buildEngine === "docker" || opts.agentEngine === "docker") ? `
if ! command -v docker >/dev/null 2>&1; then
    if command -v distrobox-host-exec >/dev/null 2>&1 && distrobox-host-exec docker --version >/dev/null 2>&1; then
        docker() { distrobox-host-exec docker "$@"; }
    elif command -v apt-get >/dev/null 2>&1; then
        info "Installing docker CLI via apt-get..."
        sudo apt-get update -qq && sudo apt-get install -y -qq docker.io || warn "Could not install docker.io package."
    fi
fi
if [ -z "$DOCKER_HOST" ] && [ -S "/run/user/$(id -u)/docker.sock" ]; then
    export DOCKER_HOST="unix:///run/user/$(id -u)/docker.sock"
fi
` : ""}
${(opts.buildEngine === "podman" || opts.agentEngine === "podman" || opts.agentEngine === "podman_systemd") ? `
if ! command -v podman >/dev/null 2>&1; then
    if command -v distrobox-host-exec >/dev/null 2>&1 && distrobox-host-exec podman --version >/dev/null 2>&1; then
        podman() { distrobox-host-exec podman "$@"; }
    fi
fi
` : ""}
${isContainerBuild ? `
info "Containerized build engine selected (${opts.buildEngine}). Host PlatformIO installation skipped."
${opts.buildEngine === "docker" ? `
if ! docker image inspect linorobot2-builder:latest >/dev/null 2>&1; then
    REG_PULLED=0
    ${buildRegistryProbeList().length > 0 ? `
    for reg in ${regListStr}; do
        if curl -fsSL -m 2 "https://${reg}/v2/" >/dev/null 2>&1 || curl -fsSL -m 2 "http://${reg}/v2/" >/dev/null 2>&1; then
            info "Registry active at ${reg}. Pulling linorobot2-builder:latest..."
            if docker pull "${reg}/linorobot2-builder:latest" >/dev/null 2>&1; then
                docker tag "${reg}/linorobot2-builder:latest" linorobot2-builder:latest
                REG_PULLED=1
                break
            fi
        fi
    done
    ` : `info "Docker Hub / direct build selected for builder image."`}
    if [ "$REG_PULLED" -eq 0 ] && [ -f "tools/robot_config_engine/docker/Dockerfile.builder" ]; then
        info "Building linorobot2-builder:latest container image locally..."
        docker build -t linorobot2-builder:latest -f tools/robot_config_engine/docker/Dockerfile.builder tools/robot_config_engine/docker/
    fi
fi
` : `
if ! podman image exists linorobot2-builder:latest >/dev/null 2>&1; then
    REG_PULLED=0
    ${buildRegistryProbeList().length > 0 ? `
    for reg in ${regListStr}; do
        if curl -fsSL -m 2 "https://${reg}/v2/" >/dev/null 2>&1 || curl -fsSL -m 2 "http://${reg}/v2/" >/dev/null 2>&1; then
            info "Registry active at ${reg}. Pulling linorobot2-builder:latest with podman..."
            if podman pull "${reg}/linorobot2-builder:latest" >/dev/null 2>&1; then
                podman tag "${reg}/linorobot2-builder:latest" linorobot2-builder:latest
                REG_PULLED=1
                break
            fi
        fi
    done
    ` : `info "Docker Hub / direct build selected for builder image with podman."`}
    if [ "$REG_PULLED" -eq 0 ] && [ -f "tools/robot_config_engine/docker/Dockerfile.builder" ]; then
        info "Building linorobot2-builder:latest container image with podman..."
        podman build -t linorobot2-builder:latest -f tools/robot_config_engine/docker/Dockerfile.builder tools/robot_config_engine/docker/
    fi
fi
`}
` : `
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
    elif command -v dnf &>/dev/null; then
        info "Installing build packages via dnf..."
        if [ "$EUID" -eq 0 ]; then
            dnf install -y git cmake ninja-build python3-pip systemd-udev || true
        else
            sudo dnf install -y git cmake ninja-build python3-pip systemd-udev || true
        fi
    fi
else
    info "Build toolchains already installed (pio, cmake, git ready). Skipping redundant package updates."
fi` : "# Build tools check skipped"}

${opts.installPio ? `export PATH="$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH"
# "pio on PATH" is not enough — a stale/half-installed penv leaves a \`pio\`
# entry-point that can no longer import platformio. Test that it actually runs.
pio_ok() { command -v pio >/dev/null 2>&1 && pio --version >/dev/null 2>&1; }
if ! pio_ok; then
    if command -v pio >/dev/null 2>&1; then
        warn "PlatformIO on PATH is broken (cannot import 'platformio') — rebuilding a clean venv..."
        rm -rf "$HOME/.platformio/penv"
        hash -r
    fi
    info "Installing PlatformIO Core..."
    if ! curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py; then
        err "Could not download the PlatformIO installer (raw.githubusercontent.com unreachable)."
        exit 1
    fi
    if ! python3 /tmp/get-platformio.py; then
        err "PlatformIO Core installation failed."
        exit 1
    fi
    export PATH="$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH"
    hash -r
fi
if ! pio_ok; then
    err "PlatformIO Core is not working after installation ('pio --version' fails)."
    exit 1
fi
# The penv sometimes ships pip \`cmake\` / \`colcon\` shims that shadow the
# working system tool but can't import their own modules — drop the dead ones.
for _t in cmake colcon; do
    _p="$HOME/.platformio/penv/bin/$_t"
    if [ -e "$_p" ] && ! "$_p" --version >/dev/null 2>&1; then
        warn "Removing broken PlatformIO penv shim: $_p"
        rm -f "$_p"
    fi
done
hash -r
success "PlatformIO Core active: $(pio --version 2>/dev/null || echo 'Installed')"
` : "# PlatformIO install skipped"}
`}

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
${(opts.agentEngine !== "native" && opts.agentEngine !== "none") ? `info "Step 2: Containerized micro-ROS Agent selected (${opts.agentEngine}). Skipping host ROS 2 & ~/uros_ws installation."` :
(opts.rosDistro !== "none" ? `info "Step 2: Verifying ROS 2 ${opts.rosDistro.toUpperCase()} environment..."
if [ ! -d "/opt/ros/${opts.rosDistro}" ]; then
    if command -v apt-get &>/dev/null; then
        warn "ROS 2 ${opts.rosDistro} not found in /opt/ros/${opts.rosDistro}. Installing via apt..."
        sudo apt-get update -qq || warn "apt-get update reported errors; continuing."
        sudo apt-get install -y -qq curl gnupg software-properties-common || warn "Could not install apt prerequisites."
        sudo add-apt-repository -y universe || warn "Could not enable the 'universe' repository."
        if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
            sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg || warn "Could not download the ROS 2 archive key."
        fi
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt-get update -qq || warn "apt-get update failed after adding the ROS 2 repository."
        sudo apt-get install -y -qq ${rosPkg} || err "Failed to install ${rosPkg}."
        sudo apt-get install -y -qq ros-${opts.rosDistro}-ros-workspace python3-catkin-pkg-modules python3-colcon-common-extensions python3-rosdep || warn "Some ROS 2 build helper packages are unavailable; continuing."
    elif command -v distrobox &>/dev/null; then
        info "Running on immutable/container host: ensuring distrobox container '${opts.rosDistro}' is configured..."
        distrobox create -n ${opts.rosDistro} -i docker.io/library/ubuntu:24.04 2>/dev/null || true
    else
        warn "Package manager 'apt-get' not found on this host. Skipping apt install."
    fi
fi

if [ ! -f "/opt/ros/${opts.rosDistro}/setup.bash" ]; then
    err "ROS 2 ${opts.rosDistro} is not available at /opt/ros/${opts.rosDistro}/setup.bash."
    err "Install it manually, or set ROS Distro to 'none' to build and flash firmware only."
    exit 1
fi
success "ROS 2 ${opts.rosDistro} available at /opt/ros/${opts.rosDistro}"

${opts.buildMicrorosAgent ? `# A failing agent-workspace build is NON-FATAL: the config merge/commit and the
# firmware build + flash below do not need it. Only Phase 9 (agent handshake +
# topic discovery) is skipped. AGENT_WS_OK tracks whether the agent is usable.
AGENT_WS_OK=1
UROS_AGENT_BIN="$HOME/uros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
# Rebuild if the agent binary is absent — colcon writes install/setup.bash even
# when a package fails to compile, so that file alone is not proof of a build.
if [ -d "/opt/ros/${opts.rosDistro}" ] && [ ! -x "$UROS_AGENT_BIN" ]; then
    info "Building micro_ros_agent workspace at ~/uros_ws..."
    if ! command -v colcon >/dev/null 2>&1; then
        info "Installing colcon build tooling..."
        if command -v apt-get &>/dev/null; then
            sudo apt-get install -y -qq python3-colcon-common-extensions python3-rosdep || warn "Could not install colcon tooling via apt."
        fi
    fi
    if ! command -v colcon >/dev/null 2>&1; then
        warn "colcon is not available — skipping the micro-ROS agent build (firmware still builds & flashes)."
        AGENT_WS_OK=0
    else
        mkdir -p "$HOME/uros_ws/src"
        pushd "$HOME/uros_ws" >/dev/null
        # micro-ROS cuts a branch per ROS distro, but newer distros (e.g. lyrical)
        # land in ROS before micro-ROS tags them — fall back to 'rolling', which
        # builds fine against the current ROS headers.
        uros_clone() {
            local repo="\$1" dest="\$2"
            git clone -b "${opts.rosDistro}" "\$repo" "\$dest" 2>/dev/null && return 0
            warn "No '${opts.rosDistro}' branch for \$(basename \$repo .git); trying 'rolling'..."
            git clone -b rolling "\$repo" "\$dest" 2>/dev/null && return 0
            git clone "\$repo" "\$dest" 2>/dev/null && return 0
            warn "Could not clone \$repo"
            return 1
        }
        [ -d "src/micro_ros_agent" ] || uros_clone https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent
        [ -d "src/micro_ros_msgs" ]  || uros_clone https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs
        if [ ! -d "src/micro_ros_agent" ] || [ ! -d "src/micro_ros_msgs" ]; then
            warn "micro-ROS agent sources are missing — skipping the agent build."
            AGENT_WS_OK=0
        else
            set +e
            source "/opt/ros/${opts.rosDistro}/setup.bash"
            colcon build --symlink-install \${COLCON_JOBS} --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
            COLCON_RC=\$?
            set -e
        fi
        popd >/dev/null
        if [ "\$AGENT_WS_OK" = "1" ] && { [ "\${COLCON_RC:-1}" -ne 0 ] || [ ! -x "$UROS_AGENT_BIN" ]; }; then
            warn "micro-ROS agent workspace build FAILED at $HOME/uros_ws — micro-ROS-Agent may not support ROS 2 ${opts.rosDistro} yet (vendored spdlog vs system libfmt / GCC on newer distros)."
            warn "Continuing anyway: config merge, git commit, firmware build and flash below are unaffected."
            warn "To bridge the board afterwards, run an agent from a container, e.g.:  docker run --rm --net=host --privileged -v /dev:/dev microros/micro-ros-agent:${opts.rosDistro} serial --dev ${port} -b ${spec.baudrate || 921600}"
            AGENT_WS_OK=0
        fi
        [ "\$AGENT_WS_OK" = "1" ] && success "micro-ROS agent workspace ready at $HOME/uros_ws"
    fi
fi` : "AGENT_WS_OK=0  # agent build not requested"}
` : `# ROS 2 Host setup skipped (Standalone Firmware mode)`)}

# -----------------------------------------------------------------------------
# Phase 3: Git Branch
# -----------------------------------------------------------------------------
# Full Deploy runs on whatever branch is checked out. To land the config on a
# different branch, set it in the studio's "Git Branch Name" field and use
# "Run Merge & Commit" (which checks that branch out first).
${`if git rev-parse --is-inside-work-tree &>/dev/null; then info "Working on Git branch '$(git rev-parse --abbrev-ref HEAD)'."; fi`}

# -----------------------------------------------------------------------------
# Phase 4: Ingest Custom C++ Header
# -----------------------------------------------------------------------------
${opts.mergeHeader ? `info "Step 4: Merging C++ configuration header into config/custom/${name}_config.h (search-and-merge with any existing file)..."
${getConfigSyncScript(spec, name).join("\n")}
success "Synced config/custom/${name}_config.h"
${wifiConfigContent ? `
# WiFi credentials go to the git-ignored config/custom/wifi_config.h, never
# into the tracked robot header. An existing file is left untouched.
if [ ! -f "config/custom/wifi_config.h" ]; then
    info "Writing config/custom/wifi_config.h (git-ignored — holds your WiFi credentials)..."
    cat << 'EOF_WIFI_CFG' > "config/custom/wifi_config.h"
${wifiConfigContent}
EOF_WIFI_CFG
    success "Wrote config/custom/wifi_config.h (not tracked by git)"
else
    info "Keeping existing config/custom/wifi_config.h (delete it to regenerate)"
fi
` : ""}
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
        if ! git config user.email >/dev/null 2>&1; then
            warn "No git identity configured; setting a repository-local placeholder."
            git config user.email "deploy@linorobot2.local" || true
            git config user.name "Linorobot2 Deploy Engine" || true
        fi
        if git commit -m "${commitMsgEsc}"; then
            success "Git commit created on branch '$(git rev-parse --abbrev-ref HEAD)'"
        else
            warn "git commit failed; the generated configuration remains staged."
        fi
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
    ${opts.buildEngine === "podman" ? `info "🦭 [Podman Rootless] Compiling inside Podman container..."
    podman run --rm -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace:Z" -w /workspace -e ROS_DISTRO=${opts.rosDistro} linorobot2-builder:latest pio run -d "${target}" -e "${name}"` :
      opts.buildEngine === "docker" ? `info "🐳 [Docker Rootless] Compiling inside Docker container..."
    docker run --rm -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace" -w /workspace -e ROS_DISTRO=${opts.rosDistro} linorobot2-builder:latest pio run -d "${target}" -e "${name}"` :
      `pio run -d "${target}" -e "${name}"`
    }
    success "Build SUCCEEDED for '${name}' in ${target}/"

    if [ -n "${port}" ] && [ "${port}" != "AUTO" ]; then
        ensure_serial_rw "${port}"
        info "Releasing port '${port}' (closing active micro-ROS agents and serial monitors)..."
        fuser -k "${port}" 2>/dev/null || true
        pkill -f "micro_ros_agent.*${port}" 2>/dev/null || true
        pkill -f "miniterm.*${port}" 2>/dev/null || true
        pkill -f "screen.*${port}" 2>/dev/null || true
        sleep 0.8

        info "Uploading firmware to microcontroller on port '${port}'..."
        ${opts.buildEngine === "docker" ? `docker run --rm --privileged -v /dev:/dev -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace" -w /workspace -e ROS_DISTRO=${opts.rosDistro} linorobot2-builder:latest pio run -d "${target}" -e "${name}" -t upload --upload-port "${port}"${otaAuth} || {
            warn "Initial upload attempt exited. Retrying upload after resetting serial port..."
            fuser -k "${port}" 2>/dev/null || true
            sleep 1
            docker run --rm --privileged -v /dev:/dev -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace" -w /workspace -e ROS_DISTRO=${opts.rosDistro} linorobot2-builder:latest pio run -d "${target}" -e "${name}" -t upload --upload-port "${port}"${otaAuth} || {
                warn "Attempting fallback auto-upload protocol..."
                docker run --rm --privileged -v /dev:/dev -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace" -w /workspace -e ROS_DISTRO=${opts.rosDistro} linorobot2-builder:latest pio run -d "${target}" -e "${name}" -t upload || {
                    err "All upload attempts failed for port '${port}'."
                    err "Check the cable, hold BOOT/RESET if the board requires it, and make sure no serial monitor holds the port."
                    exit 1
                }
            }
        }` :
        opts.buildEngine === "podman" ? `podman run --rm --privileged -v /dev:/dev -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace:Z" -w /workspace -e ROS_DISTRO=${opts.rosDistro} linorobot2-builder:latest pio run -d "${target}" -e "${name}" -t upload --upload-port "${port}"${otaAuth} || {
            warn "Initial upload attempt exited. Retrying upload after resetting serial port..."
            fuser -k "${port}" 2>/dev/null || true
            sleep 1
            podman run --rm --privileged -v /dev:/dev -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace:Z" -w /workspace -e ROS_DISTRO=${opts.rosDistro} linorobot2-builder:latest pio run -d "${target}" -e "${name}" -t upload --upload-port "${port}"${otaAuth} || {
                warn "Attempting fallback auto-upload protocol..."
                podman run --rm --privileged -v /dev:/dev -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace:Z" -w /workspace -e ROS_DISTRO=${opts.rosDistro} linorobot2-builder:latest pio run -d "${target}" -e "${name}" -t upload || {
                    err "All upload attempts failed for port '${port}'."
                    err "Check the cable, hold BOOT/RESET if the board requires it, and make sure no serial monitor holds the port."
                    exit 1
                }
            }
        }` :
        `pio run -d "${target}" -e "${name}" -t upload --upload-port "${port}"${otaAuth} || {
            warn "Initial upload attempt exited. Retrying upload after resetting serial port..."
            fuser -k "${port}" 2>/dev/null || true
            sleep 1
            pio run -d "${target}" -e "${name}" -t upload --upload-port "${port}"${otaAuth} || {
                warn "Attempting fallback auto-upload protocol..."
                pio run -d "${target}" -e "${name}" -t upload || {
                    err "All upload attempts failed for port '${port}'."
                    err "Check the cable, hold BOOT/RESET if the board requires it, and make sure no serial monitor holds the port."
                    exit 1
                }
            }
        }`}
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
    
    # Terminate any existing micro_ros_agent on this port/protocol
    pkill -f "micro_ros_agent.*${port}" 2>/dev/null || true
    pkill -f "micro_ros_agent.*udp4" 2>/dev/null || true
    ${opts.agentEngine === "docker" ? `docker rm -f microros_agent 2>/dev/null || true` :
      (opts.agentEngine === "podman" || opts.agentEngine === "podman_systemd") ? `podman rm -f microros_agent 2>/dev/null || true` : ""}
    sleep 0.5
    ensure_serial_rw "${port}"

    AGENT_LOG="/tmp/micro_ros_agent_${name}.log"

    # After a BOOTSEL / AUTO flash the board re-enumerates as a CDC device; the
    # agent must target that, not the literal "BOOTSEL".
    AGENT_DEV="${port}"
    if [ "\${AGENT_DEV}" = "BOOTSEL" ] || [ "\${AGENT_DEV}" = "AUTO" ] || [ ! -e "\${AGENT_DEV}" ]; then
        for _i in $(seq 1 10); do
            AGENT_DEV=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n1)
            [ -n "\${AGENT_DEV}" ] && break
            sleep 1
        done
        info "Post-flash serial device: \${AGENT_DEV:-<none found>}"
    fi
    ensure_serial_rw "\${AGENT_DEV}"

    ${opts.agentEngine === "docker" ? `
    ${buildRegistryProbeList().length > 0 ? `
    for reg in ${regListStr}; do
        if curl -fsSL -m 2 "https://${reg}/v2/" >/dev/null 2>&1 || curl -fsSL -m 2 "http://${reg}/v2/" >/dev/null 2>&1; then
            if docker pull "${reg}/microros/micro-ros-agent:${opts.rosDistro}" >/dev/null 2>&1; then
                docker tag "${reg}/microros/micro-ros-agent:${opts.rosDistro}" "microros/micro-ros-agent:${opts.rosDistro}"
                break
            fi
        fi
    done
    ` : `info "Pulling micro-ROS agent from upstream Docker Hub..."`}
    info "Starting micro-ROS agent in Docker container (logging to \${AGENT_LOG})..."
    ${/WIFI|UDP/i.test(spec.transport || "") ? `
    docker run -d --name microros_agent --rm --net=host -p 8888:8888/udp microros/micro-ros-agent:${opts.rosDistro} udp4 --port 8888 > "\${AGENT_LOG}" 2>&1
    ` : `
    docker run -d --name microros_agent --rm --net=host --privileged -v /dev:/dev microros/micro-ros-agent:${opts.rosDistro} serial --dev "\${AGENT_DEV}" -b "${spec.baudrate || 921600}" > "\${AGENT_LOG}" 2>&1
    `}

    info "Docker micro-ROS agent container started. Waiting for session handshake from microcontroller..."
    CONNECTED=0
    for i in $(seq 1 8); do
        sleep 1
        if docker logs microros_agent 2>&1 | grep -qi "session established" || grep -qi "session established" "\${AGENT_LOG}" 2>/dev/null; then
            CONNECTED=1
            break
        fi
        echo -n "."
    done
    echo ""
    ` :
    (opts.agentEngine === "podman" || opts.agentEngine === "podman_systemd") ? `
    ${buildRegistryProbeList().length > 0 ? `
    for reg in ${regListStr}; do
        if curl -fsSL -m 2 "https://${reg}/v2/" >/dev/null 2>&1 || curl -fsSL -m 2 "http://${reg}/v2/" >/dev/null 2>&1; then
            if podman pull "${reg}/microros/micro-ros-agent:${opts.rosDistro}" >/dev/null 2>&1; then
                podman tag "${reg}/microros/micro-ros-agent:${opts.rosDistro}" "microros/micro-ros-agent:${opts.rosDistro}"
                break
            fi
        fi
    done
    ` : `info "Pulling micro-ROS agent from upstream with podman..."`}
    info "Starting micro-ROS agent in Podman container (logging to \${AGENT_LOG})..."
    ${/WIFI|UDP/i.test(spec.transport || "") ? `
    podman run -d --name microros_agent --replace --net=host -p 8888:8888/udp microros/micro-ros-agent:${opts.rosDistro} udp4 --port 8888 > "\${AGENT_LOG}" 2>&1
    ` : `
    podman run -d --name microros_agent --replace --net=host --privileged -v /dev:/dev microros/micro-ros-agent:${opts.rosDistro} serial --dev "\${AGENT_DEV}" -b "${spec.baudrate || 921600}" > "\${AGENT_LOG}" 2>&1
    `}

    info "Podman micro-ROS agent container started. Waiting for session handshake from microcontroller..."
    CONNECTED=0
    for i in $(seq 1 8); do
        sleep 1
        if podman logs microros_agent 2>&1 | grep -qi "session established" || grep -qi "session established" "\${AGENT_LOG}" 2>/dev/null; then
            CONNECTED=1
            break
        fi
        echo -n "."
    done
    echo ""
    ` : `
    # Native ROS 2 Execution
    set +e
    [ -f "/opt/ros/${opts.rosDistro}/setup.bash" ] && source "/opt/ros/${opts.rosDistro}/setup.bash"
    [ -f "$HOME/uros_ws/install/setup.bash" ] && source "$HOME/uros_ws/install/setup.bash"
    set -e

    if ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
        warn "micro_ros_agent is not on the ROS 2 graph — the agent workspace did not build for '${opts.rosDistro}'."
        warn "Firmware was built and flashed. Bridge the board with an agent from a supported distro/container, e.g.:"
        warn "  docker run --rm --net=host --privileged -v /dev:/dev microros/micro-ros-agent:${opts.rosDistro} serial --dev ${port} -b ${spec.baudrate || 921600}"
    else
        AGENT_LOG="/tmp/micro_ros_agent_${name}.log"
        info "Starting micro-ROS agent in background (logging to \${AGENT_LOG})..."
        ${/WIFI|UDP/i.test(spec.transport || "") ? `
        ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 > "\${AGENT_LOG}" 2>&1 &
        AGENT_PID=\$!
        ` : `
        ros2 run micro_ros_agent micro_ros_agent serial --dev "\${AGENT_DEV}" -b "${spec.baudrate || 921600}" > "\${AGENT_LOG}" 2>&1 &
        AGENT_PID=\$!
        `}

        info "Agent PID: \${AGENT_PID}. Waiting for session handshake from microcontroller..."
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
    fi
    `}

    if [ "\${CONNECTED}" -eq 1 ]; then
        success "🎉 micro-ROS Agent connected successfully! Active ROS 2 Session Established."
    else
        info "micro-ROS Agent active. Querying active ROS 2 graph..."
    fi

    if command -v ros2 >/dev/null 2>&1; then
        info "========================================================="
        info "  Active ROS 2 Nodes on Robot Network:"
        info "========================================================="
        ros2 node list 2>/dev/null || echo "  (no nodes discovered yet)"
        
        info "========================================================="
        info "  Active ROS 2 Topics & Message Types:"
        info "========================================================="
        ros2 topic list -t 2>/dev/null || echo "  (no topics discovered yet)"
        info "========================================================="
    else
        info "========================================================="
        info "  micro-ROS Agent active in container."
        ${opts.agentEngine === "docker" ? `info "  Logs: docker logs -f microros_agent"` : `info "  Logs: podman logs -f microros_agent"`}
        info "========================================================="
    fi
fi

info "========================================================="
success "All deployment steps completed successfully for '${name}'!"
info "========================================================="
`;

  return script;
}

// Stage the generated deploy script on the Robot SBC and run it. Shared by the
// header "Run Full Deploy" button and the Automation-tab hero card.
function runFullDeploy() {
  const spec = currentSpec;
  const opts = readAutomationOptions();
  const name = spec.robot_name || "scout_pico2";
  const script = generateDeployScript(spec, opts);
  const safeName = String(name).replace(/[^A-Za-z0-9._-]/g, "_");
  const cmd = `cat << 'EOF_DEPLOY_SCRIPT_WRAPPER' > "deploy_${safeName}.sh"\n${script}\nEOF_DEPLOY_SCRIPT_WRAPPER\nchmod +x "deploy_${safeName}.sh" && bash "./deploy_${safeName}.sh"`;
  executeCommandInTerminal(cmd, `Executing Full Deploy Lifecycle for '${name}'`);
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

// The console (#serial-terminal-screen) is fed by four unbounded streams —
// /api/exec deploy output, `ros2 topic echo`, the UDP syslog stream and the
// Web Serial monitor. Without a cap the <div> count grows until the browser
// tab runs out of memory. Keep at most MAX_TERMINAL_LINES rows and drop the
// oldest in chunks; autoscroll is coalesced onto one rAF so a fast stream
// doesn't force a synchronous reflow per line.
const MAX_TERMINAL_LINES = 4000;
const TERMINAL_TRIM_CHUNK = 512;
let _terminalScrollQueued = false;

function _trimTerminal(screen) {
  const over = screen.childElementCount - MAX_TERMINAL_LINES;
  if (over <= 0) return;
  for (let i = 0, n = over + TERMINAL_TRIM_CHUNK; i < n && screen.firstChild; i++) {
    screen.removeChild(screen.firstChild);
  }
}

function _queueTerminalScroll(screen) {
  const autoScroll = document.getElementById("chk-serial-autoscroll")?.checked ?? true;
  if (!autoScroll || _terminalScrollQueued) return;
  _terminalScrollQueued = true;
  requestAnimationFrame(() => {
    _terminalScrollQueued = false;
    screen.scrollTop = screen.scrollHeight;
  });
}

function appendTerminalLine(text, type = "out") {
  const screen = document.getElementById("serial-terminal-screen");
  if (!screen) return;
  const lineEl = document.createElement("div");
  lineEl.className = `terminal-line terminal-${type}`;
  lineEl.innerText = text;
  screen.appendChild(lineEl);
  _trimTerminal(screen);
  _queueTerminalScroll(screen);
}

// Like appendTerminalLine but the caller supplies already-escaped innerHTML
// (the syslog stream renders coloured multi-span rows). Goes through the same
// line cap.
function appendTerminalHtml(html, type = "out") {
  const screen = document.getElementById("serial-terminal-screen");
  if (!screen) return;
  const lineEl = document.createElement("div");
  lineEl.className = `terminal-line terminal-${type}`;
  lineEl.innerHTML = html;
  screen.appendChild(lineEl);
  _trimTerminal(screen);
  _queueTerminalScroll(screen);
}

function clearTerminalScreen() {
  const screen = document.getElementById("serial-terminal-screen");
  if (screen) {
    screen.innerHTML = '<div class="terminal-line terminal-dim">[Terminal Cleared]</div>';
  }
}

// Robust Direct Upload / Build Command Generator (module scope so the ADC
// Calibration Studio's runHardwareAdcCalibration() can reuse it too).
function getDirectUploadOrBuildCmd(target, isUpload = true) {
  const spec = currentSpec;
  const name = spec.robot_name || "scout_pico2";
  const nameUpper = name.toUpperCase();
  const opts = readAutomationOptions();
  const rosDistro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
  const otaOn = document.getElementById("cfg-ota-enable")?.checked;
  const otaIp = (document.getElementById("cfg-ota-ip")?.value || "").trim();
  const otaPass = (document.getElementById("cfg-ota-password")?.value || "").trim();
  const port = (otaOn && otaIp) ? otaIp
             : (document.getElementById("auto-flash-port")?.value.trim() || (isESP32(spec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0"));
  const wifiConfigContent = generateWifiConfig(spec);
  const pioSection = generatePlatformioEnv(spec);
  const urdfContent = generateUrdfXacro(spec);
  const targetDir = (target === "i2c_detect") ? "tools/i2c_detect" : target;
  // test_motors/, test_acc/, test_sensors/ and adc_calibrate/ each carry a
  // platformio.ini with `extra_configs = ../firmware/platformio.ini`, so they
  // inherit [env:<robot>] and build with the user's exact config. (The legacy
  // calibration/ project has its own fixed env list and is not used here.)
  const buildEnv = name;

  const lines = [
    `set -e`,
    `export PATH="$HOME/.platformio/penv/bin:$HOME/.local/bin:$PATH"`,
    `export ROS_DISTRO=${rosDistro}`,
    // test_sensors/test_motors/test_acc/adc_calibrate each carry their own
    // platformio.ini (`extra_configs = ../firmware/platformio.ini`) and so
    // build against the exact same lib_deps/env as firmware/, incl.
    // micro_ros_platformio -- test_sensors/test_motors/test_acc genuinely
    // link it (firmware/lib/imu's message-typed interface calls
    // micro_ros_string_utilities_set() for frame_id), it's not dead weight.
    // But each is its own PlatformIO project directory, and by default
    // PlatformIO downloads+builds every lib_dep separately per project
    // directory -- so without this, each one redoes micro_ros_platformio's
    // very slow first-time build (compiles the full micro-ROS static lib)
    // from scratch, even though it's identical to what firmware/ already
    // built for the same board/env. Point PLATFORMIO_LIBDEPS_DIR at
    // firmware/'s own libdeps location (its default, unchanged) so
    // PlatformIO treats the library as already installed & built there.
    ...(["test_sensors", "test_motors", "test_acc", "adc_calibrate"].includes(target) ? [
      `export PLATFORMIO_LIBDEPS_DIR="$(pwd)/firmware/.pio/libdeps"`,
    ] : []),
    ``,
    `echo "=== [1/2] Build tools (PlatformIO) ==============================="`,
    ensurePioToolchainSnippet(),   // first-run bootstrap so a bare SBC/box doesn't exit 127
    `echo "=== [2/2] ${isUpload ? "Building & flashing" : "Building"} ${target} ==============="`,
    `[ "$(awk '/MemTotal/{print $2}' /proc/meminfo 2>/dev/null || echo 8000000)" -lt 6000000 ] && export PLATFORMIO_BUILD_JOBS=2 MAKEFLAGS="-j2" || true`,
    `if [ -f "/opt/ros/${rosDistro}/setup.bash" ]; then source "/opt/ros/${rosDistro}/setup.bash"; fi`,
    `if [ -f "$HOME/uros_ws/install/setup.bash" ]; then source "$HOME/uros_ws/install/setup.bash"; fi`,
    ``,
    `# 1. Ensure custom robot headers and urdf are synced (search-and-merge with any existing file)`,
    ...getConfigSyncScript(spec, name),
    ...(wifiConfigContent ? [
      `if [ ! -f "config/custom/wifi_config.h" ]; then`,
      `    cat << 'EOF_WIFI_CFG' > "config/custom/wifi_config.h"   # git-ignored, holds WiFi credentials`,
      `${wifiConfigContent}`,
      `EOF_WIFI_CFG`,
      `fi`,
    ] : []),
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
    // "Step 4" (the Merge & Commit section) is folded in here so it always
    // runs before a build/upload/calibration, not just when the user
    // remembers to click "Run Merge & Commit" separately. Respects the same
    // "Stage changes and create Git commit automatically" checkbox.
    ...(opts.autoCommit ? [
      `# 5. Stage & commit the synced configuration (Step 4's auto-commit)`,
      `git add config/ firmware/platformio.ini urdf/ 2>/dev/null || true`,
      `if ! git diff --cached --quiet 2>/dev/null; then`,
      `    git commit -m "${(opts.gitCommitMsg || `feat(config): add configuration for ${name}`).replace(/"/g, '\\"')}"`,
      `else`,
      `    echo "Configuration files already up to date on branch $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo '?')."`,
      `fi`,
      ``,
    ] : []),
    `# 6. Compile & Upload via PlatformIO`
  ];

  if (isUpload) {
    if (remoteConfig.targetMode === "remote") {
      const host = remoteConfig.remoteHost;
      const user = remoteConfig.remoteUser;
      const sshPort = remoteConfig.remotePort || 22;
      const sshKey = remoteConfig.remoteKey;
      const sshOpts = `-o StrictHostKeyChecking=no -p ${sshPort}` + (sshKey ? ` -i ${sshKey}` : "");
      const isPico = !isESP32(spec.mcu);
      const binExt = isPico ? "uf2" : "bin";
      const binFile = `${targetDir}/.pio/build/${buildEnv}/firmware.${binExt}`;
      const remoteDest = `/tmp/${target}_firmware.${binExt}`;

      lines.push(`# 6. Compile locally on Workstation via PlatformIO`);
      const buildEngine = document.getElementById("hdr-build-engine")?.value || (document.getElementById("chk-docker-builder")?.checked ? "docker" : "native");
      const dockerImg = document.getElementById("docker-builder-image")?.value.trim() || "linorobot2-builder:latest";
      if (buildEngine === "docker") {
        lines.push(`echo "🐳 Compiling inside Docker rootless container (${dockerImg})..."`);
        lines.push(`docker run --rm -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace" -w /workspace -e ROS_DISTRO=${rosDistro} ${dockerImg} pio run -d ${targetDir} -e ${buildEnv}`);
      } else if (buildEngine === "podman") {
        lines.push(`echo "🦭 Compiling inside Podman container (${dockerImg})..."`);
        lines.push(`podman run --rm -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace:Z" -w /workspace -e ROS_DISTRO=${rosDistro} ${dockerImg} pio run -d ${targetDir} -e ${buildEnv}`);
      } else {
        lines.push(`pio run -d ${targetDir} -e ${buildEnv}`);
      }
      lines.push(``);
      lines.push(`# 7. Ensure minimal tools on remote Robot PC (${host})`);
      lines.push(`echo "Checking flash tools on ${user}@${host}..."`);
      lines.push(`ssh ${sshOpts} ${user}@${host} "which esptool.py >/dev/null 2>&1 || python3 -m esptool version >/dev/null 2>&1 || python3 -m pip install -q --user esptool 2>/dev/null || true"`);
      lines.push(``);
      lines.push(`# 8. Stop serial micro-ROS agent & monitors on remote Robot PC`);
      lines.push(`echo "Releasing remote USB port '${port}' on ${host}..."`);
      lines.push(`ssh ${sshOpts} ${user}@${host} "docker stop microros_agent 2>/dev/null || true; pkill -f '[m]icro_ros_agent.*serial' 2>/dev/null || true; pkill -f '[m]initerm' 2>/dev/null || true; [ -e '${port}' ] && fuser -k '${port}' 2>/dev/null || true"`);
      lines.push(``);
      lines.push(`# 9. Transfer firmware binary to remote Robot PC via SCP`);
      lines.push(`echo "Transferring ${binFile} -> ${user}@${host}:${remoteDest}..."`);
      lines.push(`scp ${sshOpts} ${binFile} ${user}@${host}:${remoteDest}`);
      lines.push(``);
      lines.push(`# 10. Flash Microcontroller remotely on Robot PC`);
      if (isPico) {
        lines.push(`echo "🐳 [Docker Anywhere] Flashing RP2040/RP2350 via picotool Docker container on remote robot..."`);
        lines.push(`ssh ${sshOpts} ${user}@${host} "docker run --rm --privileged -v /dev:/dev -v /dev/bus/usb:/dev/bus/usb -v /tmp:/tmp linorobot2-flasher:latest -c 'picotool load -f ${remoteDest} -x || picotool load -u -v -x ${remoteDest} || cp ${remoteDest} /media/*/RPI-RP2/ 2>/dev/null || cp ${remoteDest} /mnt/RPI-RP2/ 2>/dev/null' || picotool load -f ${remoteDest} -x || sudo cp ${remoteDest} /media/*/RPI-RP2/ 2>/dev/null"`);
      } else {
        lines.push(`echo "🐳 [Docker Anywhere] Flashing ESP32 via esptool Docker container on remote robot..."`);
        lines.push(`ssh ${sshOpts} ${user}@${host} "docker run --rm --privileged -v /dev:/dev -v /tmp:/tmp linorobot2-flasher:latest -c 'esptool --port ${port} --baud 460800 --before default_reset --after hard_reset write_flash 0x10000 ${remoteDest}' || python3 -m esptool --port ${port} --baud 460800 write_flash 0x10000 ${remoteDest} || esptool.py --port ${port} --baud 460800 write_flash 0x10000 ${remoteDest}"`);
      }
    } else {
      lines.push(`# 7. Stop serial micro-ROS agent & monitors to release shared USB port (WiFi agent kept running)`);
    lines.push(`echo "Releasing USB port '${port}' (stopping serial micro-ROS agent and serial monitors)..."`);
    // NOTE the "[m]" / "[s]" first-character classes: this whole script is passed
    // to `bash -c` as one argument, so a plain `pkill -f "micro_ros_agent.*serial"`
    // matches the running shell's OWN /proc/<pid>/cmdline and SIGTERMs the deploy
    // (exit -15). The bracket makes the pattern text non-self-matching while it
    // still matches a real `micro_ros_agent ... serial` process.
    lines.push(`pkill -f "[m]icro_ros_agent.*serial" 2>/dev/null || true`);
    lines.push(`pkill -f "[m]initerm" 2>/dev/null || true`);
    lines.push(`pkill -f "[s]creen.*${port}" 2>/dev/null || true`);
    if (port !== "AUTO" && port !== "BOOTSEL" && !(otaOn && otaIp)) {
      lines.push(`[ -e "${port}" ] && fuser -k "${port}" 2>/dev/null || true`);
      // Grant r/w when no udev rule / 'dialout' membership does.
      lines.push(`for _d in /dev/ttyUSB* /dev/ttyACM*; do [ -e "$_d" ] && [ ! -w "$_d" ] && sudo chmod a+rw "$_d" 2>/dev/null || true; done`);
    }
    lines.push(`sleep 0.8`);
    lines.push(``);
    lines.push(`# 8. Flash Firmware onto Microcontroller`);
    if (port === "AUTO" || port === "BOOTSEL") {
      lines.push(`pio run -d ${targetDir} -e ${buildEnv} -t upload`);
    } else if (otaOn && otaIp) {
      const authFlag = otaPass ? ` --upload-flags "--auth=${otaPass}"` : "";
      lines.push(`echo "OTA flashing ${buildEnv} over WiFi -> ${otaIp}"`);
      lines.push(`pio run -d ${targetDir} -e ${buildEnv} -t upload --upload-port ${otaIp}${authFlag}`);
    } else {
      const buildEngine = document.getElementById("hdr-build-engine")?.value || (document.getElementById("chk-docker-builder")?.checked ? "docker" : "native");
      const dockerImg = document.getElementById("docker-builder-image")?.value.trim() || "linorobot2-builder:latest";
      if (buildEngine === "docker") {
        lines.push(`echo "🐳 Compiling & flashing via Docker rootless container (${dockerImg})..."`);
        lines.push(`docker run --rm --privileged -v /dev:/dev -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace" -w /workspace -e ROS_DISTRO=${rosDistro} ${dockerImg} pio run -d ${targetDir} -e ${buildEnv} -t upload --upload-port ${port}`);
      } else if (buildEngine === "podman") {
        lines.push(`echo "🦭 Compiling & flashing via Podman container (${dockerImg})..."`);
        lines.push(`podman run --rm --privileged -v /dev:/dev -v linorobot2_pio_cache:/root/.platformio -v "$(pwd):/workspace:Z" -w /workspace -e ROS_DISTRO=${rosDistro} ${dockerImg} pio run -d ${targetDir} -e ${buildEnv} -t upload --upload-port ${port}`);
      } else {
        lines.push(`pio run -d ${targetDir} -e ${buildEnv} -t upload --upload-port ${port}`);
      }
      }
    }
  } else {
    lines.push(`pio run -d ${targetDir} -e ${buildEnv}`);
  }

  return lines.join("\n");
}

// After flashing a diagnostic sketch (test_sensors / test_motors / test_acc),
// append a read-only serial monitor so its output streams into the console.
// Dependency-free (stty + cat, like the "Stream SBC Serial" button); runs until
// the user hits Stop. No-op for BOOTSEL / AUTO ports or non-diagnostic targets.
function withSerialMonitor(cmd, target, port) {
  const streamable = ["test_sensors", "test_motors", "test_acc", "adc_calibrate"];
  if (!streamable.includes(target)) return cmd;
  if (!port || port.includes("BOOTSEL") || port.includes("AUTO")) return cmd;
  const baud = firmwareBaud();
  if (remoteConfig.targetMode === "remote") {
    const host = remoteConfig.remoteHost;
    const user = remoteConfig.remoteUser;
    const sshPort = remoteConfig.remotePort || 22;
    const sshKey = remoteConfig.remoteKey;
    const sshOpts = `-o StrictHostKeyChecking=no -p ${sshPort}` + (sshKey ? ` -i ${sshKey}` : "");
    return cmd + [
      ``,
      `echo`,
      `echo "=== Remote Serial monitor: ${user}@${host}:${port} @ ${baud} — press ⏹ Stop to end ==="`,
      `sleep 1`,
      `ssh ${sshOpts} -t ${user}@${host} "stty -F '${port}' ${baud} raw -echo 2>/dev/null || true; exec cat '${port}'"`,
    ].join("\n");
  }
  return cmd + [
    ``,
    `echo`,
    `echo "=== Serial monitor: ${port} @ ${baud} — press ⏹ Stop to end ==="`,
    `sleep 1`,
    `stty -F "${port}" ${baud} raw -echo 2>/dev/null || true`,
    `exec cat "${port}"`,
  ].join("\n");
}


// Initialize Automation Event Listeners

// =============================================================================
// 🤖 Remote Target Mode & SSH Management
// =============================================================================
let remoteConfig = {
  targetMode: "local",
  remoteHost: "192.168.1.100",
  remoteUser: "ubuntu",
  remotePort: 22,
  remoteKey: "",
  remoteSerialPort: "/dev/ttyUSB0",
  remoteBaud: 1500000
};

async function loadRemoteConfig() {
  try {
    const res = await fetch("/api/remote/config");
    if (!res.ok) return;
    const data = await res.json();
    if (data.config) {
      remoteConfig = {
        targetMode: data.config.target_mode || (data.config.remote_host ? "remote" : "local"),
        remoteHost: data.config.remote_host || "",
        remoteUser: data.config.remote_user || "ubuntu",
        remotePort: data.config.remote_port || 22,
        remoteKey: data.config.remote_key || "",
        remoteSerialPort: data.config.remote_serial_port || "/dev/ttyUSB0",
        remoteBaud: data.config.remote_baud || 1500000
      };
      const hostInput = document.getElementById("remote-ssh-host");
      if (hostInput) hostInput.value = remoteConfig.remoteHost;
      updateRemoteUI();
      if (remoteConfig.remoteHost) {
        testRemoteConnection();
      } else {
        detectClientOS();
      }
    }
  } catch (e) {
    console.warn("Failed to load remote config", e);
  }
}

function updateRemoteUI() {
  const hostInput = document.getElementById("remote-ssh-host");
  const userInput = document.getElementById("remote-ssh-user");
  const portInput = document.getElementById("remote-ssh-port");
  const keyInput = document.getElementById("remote-ssh-key");
  const badgeTarget = document.getElementById("target-mode-status-badge");
  const groupUser = document.getElementById("group-ssh-user");
  const groupPort = document.getElementById("group-ssh-port");
  const groupKey = document.getElementById("group-ssh-key");
  const toolsBar = document.getElementById("remote-tools-bar");

  let hostVal = "";
  if (hostInput) {
    hostVal = hostInput.value.trim();
  } else {
    hostVal = (remoteConfig.remoteHost || "").trim();
  }

  const isRemote = Boolean(hostVal);
  remoteConfig.targetMode = isRemote ? "remote" : "local";
  remoteConfig.remoteHost = hostVal;

  if (userInput) userInput.value = remoteConfig.remoteUser || "ubuntu";
  if (portInput) portInput.value = remoteConfig.remotePort || 22;
  if (keyInput) keyInput.value = remoteConfig.remoteKey || "";

  if (groupUser) groupUser.style.display = isRemote ? "block" : "none";
  if (groupPort) groupPort.style.display = isRemote ? "block" : "none";
  if (groupKey) groupKey.style.display = isRemote ? "block" : "none";
  if (toolsBar) toolsBar.style.display = isRemote ? "flex" : "none";

  if (badgeTarget) {
    if (isRemote) {
      badgeTarget.className = "badge-pill badge-ok";
      badgeTarget.textContent = `🤖 Remote: ${hostVal}`;
    } else {
      badgeTarget.className = "badge-pill badge-ok";
      badgeTarget.textContent = "💻 Local Machine";
    }
  }
}

async function saveRemoteConfig() {
  const hostInput = document.getElementById("remote-ssh-host");
  const userInput = document.getElementById("remote-ssh-user");
  const portInput = document.getElementById("remote-ssh-port");
  const keyInput = document.getElementById("remote-ssh-key");

  if (hostInput) remoteConfig.remoteHost = hostInput.value.trim();
  if (userInput) remoteConfig.remoteUser = userInput.value.trim();
  if (portInput) remoteConfig.remotePort = parseInt(portInput.value, 10) || 22;
  if (keyInput) remoteConfig.remoteKey = keyInput.value.trim();

  try {
    await fetch("/api/remote/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        target_mode: remoteConfig.targetMode,
        remote_host: remoteConfig.remoteHost,
        remote_user: remoteConfig.remoteUser,
        remote_port: remoteConfig.remotePort,
        remote_key: remoteConfig.remoteKey
      })
    });
  } catch (e) {
    console.warn("Failed to save remote config", e);
  }
}

async function testRemoteConnection() {
  const hostInput = document.getElementById("remote-ssh-host");
  const hostVal = hostInput ? hostInput.value.trim() : "";
  if (!hostVal) {
    remoteConfig.targetMode = "local";
    remoteConfig.remoteHost = "";
    await saveRemoteConfig();
    updateRemoteUI();
    detectClientOS();
    return;
  }
  remoteConfig.targetMode = "remote";
  remoteConfig.remoteHost = hostVal;
  await saveRemoteConfig();
  updateRemoteUI();

  const badgeSsh = document.getElementById("badge-remote-ssh");
  const badgeTools = document.getElementById("badge-remote-tools");
  const badgeDoc = document.getElementById("badge-remote-docker");

  if (badgeSsh) { badgeSsh.className = "badge-pill badge-neutral"; badgeSsh.textContent = `Connecting to ${hostVal}...`; }

  try {
    const res = await fetch("/api/remote/test", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        remote_host: remoteConfig.remoteHost,
        remote_user: remoteConfig.remoteUser,
        remote_port: remoteConfig.remotePort,
        remote_key: remoteConfig.remoteKey
      })
    });
    const data = await res.json();
    if (data.ok) {
      if (badgeSsh) { badgeSsh.className = "badge-pill badge-ok"; badgeSsh.textContent = `SSH: Connected (${data.os_info || "Linux"})`; }
      if (badgeTools) {
        badgeTools.className = "badge-pill badge-ok";
        badgeTools.textContent = "Minimal Tools: Auto-Ready (esptool/picotool)";
      }
      if (badgeDoc) {
        badgeDoc.className = data.has_docker ? "badge-pill badge-ok" : "badge-pill badge-neutral";
        badgeDoc.textContent = data.has_docker ? "Docker: Available" : "Docker: Not found";
      }
      const badgeRobDoc = document.getElementById("badge-docker-robot");
      if (badgeRobDoc) {
        badgeRobDoc.className = data.has_docker ? "badge-pill badge-ok" : "badge-pill badge-warn";
        badgeRobDoc.textContent = data.has_docker ? "🤖 Robot Docker: Ready" : "🤖 Robot Docker: Installing...";
      }
      showToast(`✅ Connected to robot PC: ${hostVal}! Minimal tools verified.`);
      probeRemotePorts();
    } else {
      if (badgeSsh) { badgeSsh.className = "badge-pill badge-error"; badgeSsh.textContent = `SSH Failed: ${data.error || "error"}`; }
    }
  } catch (e) {
    if (badgeSsh) { badgeSsh.className = "badge-pill badge-error"; badgeSsh.textContent = "SSH Network Error"; }
  }
}

async function probeRemotePorts() {
  await saveRemoteConfig();
  try {
    const url = `/api/remote/serial_ports?host=${encodeURIComponent(remoteConfig.remoteHost)}&user=${encodeURIComponent(remoteConfig.remoteUser)}&port=${remoteConfig.remotePort}`;
    const res = await fetch(url);
    const data = await res.json();
    if (data.status === "ok" && data.serial_ports) {
      const portInput = document.getElementById("auto-flash-port");
      const chipsWrap = document.getElementById("flash-port-chips");
      if (data.serial_ports.length > 0 && portInput && portInput.dataset.autoManaged !== "false") {
        portInput.value = data.serial_ports[0];
      }
      if (chipsWrap) {
        chipsWrap.innerHTML = `<span class="chips-label">Remote USB Devices (${remoteConfig.remoteHost}):</span> ` + (data.serial_ports || []).map(p => {
          const detail = (data.port_details || []).find(d => d.port === p) || {};
          const vidPid = detail.vid && detail.pid ? `[${detail.vid}:${detail.pid}]` : "";
          const chip = detail.chip || detail.product || "Serial Device";
          const icon = "🌐⚡";
          const badgeClass = "port-chip";
          const title = `[Remote ${remoteConfig.remoteHost}] ${p} ${vidPid} - ${chip}`;
          return `<button type="button" class="${badgeClass}" data-port="${p}" title="${title}">
            <span class="chip-icon">${icon}</span>
            <span class="chip-port">${p}</span>
            ${vidPid ? `<span class="chip-vidpid">${vidPid}</span>` : ""}
            <span class="chip-desc">${chip}</span>
          </button>`;
        }).join(" ");

        chipsWrap.querySelectorAll(".port-chip").forEach(btn => {
          btn.addEventListener("click", () => {
            const p = btn.dataset.port;
            if (portInput) portInput.value = p;
            const detail = (data.port_details || []).find(d => d.port === p);
            const guessed = guessMcuFromUsbDetail(detail);
            if (guessed) {
              applyDetectedChipToBaudUi(guessed);
            defaultSimulationForDetectedModule(guessed.chipName);
              const mcuSelect = document.getElementById("cfg-mcu");
              if (mcuSelect && guessed.mcu && !mcuSelect.dataset.userModified) {
                mcuSelect.value = guessed.mcu;
                mcuSelect.dispatchEvent(new Event("change"));
              }
            }
            updateAutomationPreviews();
          });
        });
      }

      // Automatically apply detected chip info from first discovered port from the beginning
      if (data.port_details && data.port_details.length > 0) {
        const firstDetail = data.port_details[0];
        const guessed = guessMcuFromUsbDetail(firstDetail);
        if (guessed) {
          applyDetectedChipToBaudUi(guessed);
            defaultSimulationForDetectedModule(guessed.chipName);
          const mcuSelect = document.getElementById("cfg-mcu");
          if (mcuSelect && guessed.mcu && !mcuSelect.dataset.userModified) {
            mcuSelect.value = guessed.mcu;
            mcuSelect.dispatchEvent(new Event("change"));
          }
        }
      }
      updateAutomationPreviews();
      showToast(`🔍 Discovered ${data.serial_ports.length} remote serial port(s) with Chip IDs on ${remoteConfig.remoteHost}!`);
    }
  } catch (e) {
    console.warn("Failed to probe remote ports", e);
  }
}

async function queryRemoteChipId() {
  const port = document.getElementById("auto-flash-port")?.value.trim() || "/dev/ttyUSB0";
  const callout = document.getElementById("remote-chip-id-output");
  if (callout) {
    callout.style.display = "block";
    callout.textContent = `Querying chip ID for ${port} over SSH on ${remoteConfig.remoteHost}...`;
  }
  try {
    const url = `/api/remote/chip_id?host=${encodeURIComponent(remoteConfig.remoteHost)}&user=${encodeURIComponent(remoteConfig.remoteUser)}&port=${encodeURIComponent(port)}`;
    const res = await fetch(url);
    const data = await res.json();
    if (callout) {
      callout.textContent = data.output || data.error || "No response received";
    }
  } catch (e) {
    if (callout) callout.textContent = "Error querying chip ID: " + e.message;
  }
}

function installRemoteTools() {
  const host = remoteConfig.remoteHost;
  const user = remoteConfig.remoteUser;
  const port = remoteConfig.remotePort;
  const key = remoteConfig.remoteKey;
  const sshOpts = `-o StrictHostKeyChecking=no -p ${port}` + (key ? ` -i ${key}` : "");

  const remoteScript = [
    `echo "=================================================================="`,
    `echo " 🤖 Setting up Minimal Tools on Remote Robot PC (${user}@${host})"`,
    `echo "=================================================================="`,
    `ssh ${sshOpts} ${user}@${host} 'bash -s' << 'EOF_ROBOT_SETUP'`,
    `set -e`,
    `echo "=== [1/4] Ensuring Python & pip on Robot PC ==="`,
    `if command -v apt-get >/dev/null 2>&1; then`,
    `    sudo apt-get update -y && sudo apt-get install -y python3-pip python3-serial 2>/dev/null || true`,
    `fi`,
    `echo "=== [2/4] Installing / Updating esptool ==="`,
    `python3 -m pip install -U --user --break-system-packages esptool 2>/dev/null || \\`,
    `python3 -m pip install -U --user esptool 2>/dev/null || \\`,
    `sudo apt-get install -y python3-esptool 2>/dev/null || true`,
    `echo "=== [3/4] Installing / Verifying picotool & udev rules ==="`,
    `if command -v picotool >/dev/null 2>&1; then`,
    `    echo "picotool already installed: $(which picotool)"`,
    `else`,
    `    sudo apt-get install -y picotool 2>/dev/null || {`,
    `        echo "Installing picotool from pre-compiled binary..."`,
    `        ARCH=$(uname -m)`,
    `        if [ "$ARCH" = "x86_64" ]; then`,
    `            curl -sSL "https://github.com/raspberrypi/picotool/releases/download/2.0.0/picotool-2.0.0-x86_64.tar.gz" -o /tmp/picotool.tar.gz 2>/dev/null && tar -xzf /tmp/picotool.tar.gz -C /tmp/ 2>/dev/null && sudo mv /tmp/picotool /usr/local/bin/ 2>/dev/null || true`,
    `        elif [ "$ARCH" = "aarch64" ]; then`,
    `            curl -sSL "https://github.com/raspberrypi/picotool/releases/download/2.0.0/picotool-2.0.0-aarch64.tar.gz" -o /tmp/picotool.tar.gz 2>/dev/null && tar -xzf /tmp/picotool.tar.gz -C /tmp/ 2>/dev/null && sudo mv /tmp/picotool /usr/local/bin/ 2>/dev/null || true`,
    `        fi`,
    `    }`,
    `fi`,
    `if [ -d /etc/udev/rules.d ] && [ ! -f /etc/udev/rules.d/99-pico.rules ]; then`,
    `    echo 'ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="666", GROUP="dialout"' | sudo tee /etc/udev/rules.d/99-pico.rules >/dev/null 2>&1 || true`,
    `    echo 'ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", MODE="666", GROUP="dialout"' | sudo tee -a /etc/udev/rules.d/99-pico.rules >/dev/null 2>&1 || true`,
    `    echo 'ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000f", MODE="666", GROUP="dialout"' | sudo tee -a /etc/udev/rules.d/99-pico.rules >/dev/null 2>&1 || true`,
    `    sudo udevadm control --reload-rules 2>/dev/null || true`,
    `fi`,
    `echo "=== [4/4] Configuring serial port & dialout permissions ==="`,
    `sudo usermod -a -G dialout $USER 2>/dev/null || true`,
    `for _d in /dev/ttyUSB* /dev/ttyACM*; do [ -e "$_d" ] && sudo chmod a+rw "$_d" 2>/dev/null || true; done`,
    `echo`,
    `echo "================================================================"`,
    `echo "✅ Robot PC minimal tools setup complete!"`,
    `echo "   Python:   $(python3 --version 2>/dev/null || echo 'not found')"`,
    `echo "   esptool:  $(which esptool.py 2>/dev/null || python3 -m esptool version 2>/dev/null || echo 'installed')"`,
    `echo "   picotool: $(which picotool 2>/dev/null || echo 'installed')"`,
    `echo "   docker:   $(which docker 2>/dev/null || echo 'not installed')"`,
    `echo "================================================================"`,
    `EOF_ROBOT_SETUP`
  ].join("\n");

  executeCommandInTerminal(remoteScript, `⚡ Setting Up Minimal Tools on Robot PC (${host})`).then(() => {
    testRemoteConnection();
  });
}


function initAutomationEventListeners() {
  // Remote Target Mode Listeners
    // Docker Anywhere Builder Listeners
  const chkDocker = document.getElementById("chk-docker-builder");
  const dockerDetails = document.getElementById("docker-builder-details");
  if (chkDocker) {
    chkDocker.addEventListener("change", () => {
      if (dockerDetails) dockerDetails.style.display = chkDocker.checked ? "block" : "none";
      updateAutomationPreviews();
      if (chkDocker.checked) {
        showToast("🐳 Docker Anywhere Builder enabled (compilation will run in container)");
      }
    });
  }

  const btnTestDocker = document.getElementById("btn-test-docker-builder");
  if (btnTestDocker) {
    btnTestDocker.addEventListener("click", async () => {
      try {
        const res = await fetch("/api/docker/status");
        const d = await res.json();
        if (d.has_docker) {
          showToast(`✅ Docker Engine active! Builder Image: ${d.has_builder_image ? 'Found (linorobot2-builder:latest)' : 'Ready to pull/build'}`);
        } else {
          showToast("❌ Docker not found on host. Please install or start Docker Desktop.");
        }
      } catch (e) {
        showToast("❌ Failed to contact Docker API: " + e.message);
      }
    });
  }

  // Seamless Remote Target Mode Listener (setting remote pc name/ip enables remote target mode)
  const hostInput = document.getElementById("remote-ssh-host");
  let hostDebounce = null;
  if (hostInput) {
    hostInput.addEventListener("input", () => {
      clearTimeout(hostDebounce);
      hostDebounce = setTimeout(() => {
        testRemoteConnection();
      }, 500);
    });
    hostInput.addEventListener("change", () => {
      testRemoteConnection();
    });
  }

  const btnRemoteProbe = document.getElementById("btn-remote-probe-ports");
  if (btnRemoteProbe) btnRemoteProbe.addEventListener("click", probeRemotePorts);


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
      const baud = firmwareBaud();
      const cmd = microRosAgentCmd("serial", { distro, port, baud });
      executeCommandInTerminal(cmd, `Launching Single Serial micro-ROS Agent (${port} @ ${baud}) on Robot SBC`);
    });
  }

  const btnExecAgentMultiSerial = document.getElementById("btn-exec-agent-multiserial");
  if (btnExecAgentMultiSerial) {
    btnExecAgentMultiSerial.addEventListener("click", () => {
      const opts = readAutomationOptions();
      const distro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
      const multiPorts = document.getElementById("auto-agent-multiserial-ports")?.value.trim() || "/dev/ttyACM0 /dev/ttyUSB0";
      const baud = firmwareBaud();
      const cmd = microRosAgentCmd("multiserial", { distro, ports: multiPorts, baud });
      executeCommandInTerminal(cmd, `Launching Multi-Serial micro-ROS Agent (${multiPorts} @ ${baud}) on Robot SBC`);
    });
  }

  const btnExecAgentUdp = document.getElementById("btn-exec-agent-udp");
  if (btnExecAgentUdp) {
    btnExecAgentUdp.addEventListener("click", () => {
      const opts = readAutomationOptions();
      const distro = opts.rosDistro !== "none" ? opts.rosDistro : "jazzy";
      const cmd = microRosAgentCmd("udp", { distro, udpPort: 8888 });
      executeCommandInTerminal(cmd, `Launching UDP WiFi micro-ROS Agent (:8888) on Robot SBC`);
    });
  }

  document.getElementById("btn-check-agent-port")?.addEventListener("click", checkAndDisplayAgentPortStatus);
  document.getElementById("btn-release-agent-port")?.addEventListener("click", releaseAgentPortFromUI);

  // Flash Port & Multi-Serial Input Event Listeners
  const flashPortElem = document.getElementById("auto-flash-port");
  if (flashPortElem) {
    flashPortElem.addEventListener("input", () => {
      flashPortElem.dataset.autoManaged = "false";
      updateAutomationPreviews();
      saveWorkflowAndBoardSetup();
      if (typeof checkAndDisplayAgentPortStatus === "function") checkAndDisplayAgentPortStatus();
    });
  }

  const multiPortsElem = document.getElementById("auto-agent-multiserial-ports");
  if (multiPortsElem) {
    multiPortsElem.addEventListener("input", () => {
      multiPortsElem.dataset.autoManaged = "false";
      updateAutomationPreviews();
      saveWorkflowAndBoardSetup();
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
        const cmd = withSerialMonitor(getDirectUploadOrBuildCmd(target, true), target, port);
        executeCommandInTerminal(cmd, `⚡ Uploading ${desc} (${target}/ -> ${port})`);
      });
    }
  });

  const btnExecUpload = document.getElementById("btn-exec-upload-cmd");
  if (btnExecUpload) {
    btnExecUpload.addEventListener("click", () => {
      const target = document.getElementById("auto-flash-target")?.value || "firmware";
      const port = document.getElementById("auto-flash-port")?.value.trim() || (isESP32(currentSpec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0");
      const cmd = withSerialMonitor(getDirectUploadOrBuildCmd(target, true), target, port);
      executeCommandInTerminal(cmd, `Flashing Microcontroller (${target}/ -> ${port})`);
    });
  }

  const btnExecDeploy = document.getElementById("btn-exec-deploy-script");
  if (btnExecDeploy) {
    btnExecDeploy.addEventListener("click", runFullDeploy);
  }

  // Header "Run Full Deploy" — jump to the Automation tab (so its terminal is
  // visible) and kick off the same deploy. Keeps the first-run path short:
  // set Robot Name -> "= name" -> Run Full Deploy.
  const btnHeaderDeploy = document.getElementById("btn-header-deploy");
  if (btnHeaderDeploy) {
    btnHeaderDeploy.addEventListener("click", () => {
      const tab = document.querySelector('[data-tab="tab-automation"]');
      if (tab) tab.click();
      runFullDeploy();
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

  // Stream SBC Serial Monitor — raw-read the MCU's serial port on the Robot SBC
  // and pipe its output into the console. Pure coreutils (stty + cat), no
  // pyserial dependency; press ⏹️ Stop Process to disconnect.
  const btnSbcMonitor = document.getElementById("btn-sbc-monitor");
  if (btnSbcMonitor) {
    btnSbcMonitor.addEventListener("click", () => {
      const port = document.getElementById("auto-flash-port")?.value.trim() || (isESP32(currentSpec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0");
      const baud = document.getElementById("serial-baud-select")?.value || "921600";
      let cmd;
      if (remoteConfig.targetMode === "remote") {
        const host = remoteConfig.remoteHost;
        const user = remoteConfig.remoteUser;
        const sshPort = remoteConfig.remotePort || 22;
        const sshKey = remoteConfig.remoteKey;
        const sshOpts = `-o StrictHostKeyChecking=no -p ${sshPort}` + (sshKey ? ` -i ${sshKey}` : "");
        cmd = [
          `echo "Connecting to remote serial port on ${user}@${host}:${port} @ ${baud} baud..."`,
          `ssh ${sshOpts} -t ${user}@${host} "stty -F '${port}' ${baud} raw -echo 2>/dev/null || true; exec cat '${port}'"`,
        ].join("\n");
        executeCommandInTerminal(cmd, `📡 Streaming Remote Serial (${user}@${host}:${port} @ ${baud} baud)`);
      } else {
        cmd = [
          `if [ ! -e "${port}" ]; then echo "❌ ${port} not present — plug in / flash the board first."; exit 1; fi`,
          `stty -F "${port}" ${baud} raw -echo 2>/dev/null || true`,
          `echo "📡 Connected to ${port} @ ${baud} baud. Streaming live microcontroller output (test_sensors / telemetry). Press ⏹️ Stop Process to disconnect."`,
          `exec cat "${port}"`,
        ].join("\n");
        executeCommandInTerminal(cmd, `📡 Streaming Serial Telemetry (${port} @ ${baud} baud)`);
      }
    });
  }

  // Web Serial Event Listeners (Client Browser USB)
  const btnSerialConnect = document.getElementById("btn-serial-connect");
  if (btnSerialConnect) {
    btnSerialConnect.addEventListener("click", toggleWebSerialConnection);
  }

  // Web Serial is Chromium-only (and needs HTTPS or localhost). On a browser
  // without it, grey out Connect / the command input / Send so it's clear the
  // in-browser serial console isn't available here.
  if (!("serial" in navigator)) {
    const why = "In-browser serial needs Chrome / Edge / Opera over HTTPS or localhost";
    [["btn-serial-connect", "serial-btn-text", "Serial N/A"],
     ["btn-serial-send", null, null],
     ["serial-input-text", null, null]].forEach(([id, txtId, txt]) => {
      const el = document.getElementById(id);
      if (!el) return;
      el.disabled = true;
      el.classList.add("is-disabled");
      el.title = why;
      if (txtId && txt) { const t = document.getElementById(txtId); if (t) t.textContent = txt; }
      if (id === "serial-input-text") el.placeholder = why;
    });
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
        if (data.host_ip) applyHostIpDefaults(data.host_ip);
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

// Seed the network-target IP fields with this host's LAN address (until the
// user overrides them). Agent / Syslog / LiDAR all point back at the machine
// running the Web UI by default.
function applyHostIpDefaults(hostIp) {
  ["cfg-agent-ip", "cfg-syslog-ip", "cfg-lidar-ip"].forEach((id) => {
    const el = document.getElementById(id);
    if (!el || el.dataset.autoManaged === "false") return;
    if (!el.value || el.value === "192.168.1.100") { el.value = hostIp; el.dataset.autoManaged = "true"; }
  });
}

// Bring the live terminal on-screen: activate the Automation tab (the console
// lives there) and scroll the console card into view. Used by the ROS 2 topic
// stream / Hz buttons, which sit further up the same tab, so their output
// otherwise lands out of sight.
function jumpToTerminal() {
  const autoTab = document.querySelector('[data-tab="tab-automation"]');
  if (autoTab && !autoTab.classList.contains("active")) autoTab.click();
  const card = document.querySelector(".web-serial-console-card");
  if (card) card.scrollIntoView({ behavior: "smooth", block: "start" });
}

// Execute Command with Live Streaming Output in Terminal Console
async function executeCommandInTerminal(command, title = "Executing Command") {
  const terminalScreen = document.getElementById("serial-terminal-screen");
  const btnCancel = document.getElementById("btn-cancel-exec");
  // Only an adc_calibrate run streams the 4096-entry LUT; for every other
  // command the per-line accumulate + substring scan below is dead weight that
  // grows O(n) in memory and O(n^2) in CPU over a long build log.
  const _isAdcCal = /\badc_calibrate\b/.test(command);
  // Declared here (function scope), not inside the try{} below, because it's
  // set from the SSE "done" handler (inside try) and read in finally{} —
  // try/catch/finally each introduce their own block scope for let/const, so
  // a let declared inside try is a ReferenceError from finally.
  let _adcLutCommitPending = false;

  // Surface the live terminal: the console lives on the Automation tab, so a
  // command fired from any other tab (I2C auto-detect, ADC calibrate, branch
  // switch, …) would otherwise stream out of sight. Activate that tab, then
  // scroll its console into view. No-op when already there. Remember where the
  // user was so we can hop back when the command finishes (see finally block).
  const _origTabBtn = document.querySelector(".nav-tab.active");
  const _origScrollY = window.scrollY;
  const _autoTabBtn = document.querySelector('[data-tab="tab-automation"]');
  let _switchedTab = false;
  if (_autoTabBtn && !_autoTabBtn.classList.contains("active")) {
    _autoTabBtn.click();
    _switchedTab = true;
  }
  const _restoreOrigTab = () => {
    // only if we moved them AND they're still sitting on Automation (didn't
    // navigate away themselves mid-run)
    if (_switchedTab && _origTabBtn && _autoTabBtn && _autoTabBtn.classList.contains("active")) {
      _origTabBtn.click();
      // ...and back to the scroll position they launched from (the trigger
      // button is often at the top of its tab; scrollIntoView had jumped us
      // down to the console).
      window.scrollTo({ top: _origScrollY, behavior: "auto" });
    }
  };
  const terminalEl = document.querySelector(".web-serial-console-card");
  if (terminalEl) {
    // block:"start" (not "nearest") — the console sits well below the flash /
    // build buttons on the Automation tab, so "nearest" left it at the very
    // bottom edge, mostly clipped.
    terminalEl.scrollIntoView({ behavior: "smooth", block: "start" });
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
    _restoreOrigTab();
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

  // Generic "task running" indicator for every executeCommandInTerminal call
  const execPill = document.getElementById("exec-progress-pill");
  const execText = document.getElementById("exec-progress-text");
  const execTimer = document.getElementById("exec-progress-timer");
  const execTitle = title.replace(/^[^\w]+\s*/, "");
  let execTimerInterval = null;
  const setExecPhase = (p) => { if (execText) execText.textContent = p ? `${execTitle} · ${p}` : execTitle; };
  if (execPill) {
    execPill.style.display = "inline-flex";
    execPill.className = "runner-status-pill pill-checking";
    setExecPhase("");
    if (execTimer) {
      execTimer.textContent = "(0s)";
      execTimerInterval = setInterval(() => {
        const s = Math.floor((Date.now() - deployStartTime) / 1000);
        execTimer.textContent = s < 60 ? `(${s}s)` : `(${Math.floor(s/60)}:${String(s%60).padStart(2,"0")})`;
      }, 1000);
    }
  }

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
      let detail = "";
      try { detail = (await response.json())?.error || ""; } catch (_) {}
      appendTerminalLine(`❌ ${detail || `Server error: HTTP ${response.status}`}`, "err");
      if (detail) showToast(`⚠️ ${detail}`);
      if (btnCancel) btnCancel.style.display = "none";
      return;
    }

    const reader = response.body.getReader();
    const decoder = new TextDecoder("utf-8");

    let buffer = "";
    let adcStreamBuffer = "";
    let _adcLutFreshlyCaptured = false;   // set once THIS run's sweep lands a LUT
    // _adcLutCommitPending is declared at function scope above (see comment there).
    let _sseDone = false;   // set on the `event: done` line — the socket may be
                            // kept alive, so we can't rely on the stream closing
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

                if (execPill) {
                  const L = String(textLine);
                  const step = L.match(/Step (\d)\/9/);
                  const phaseBanner = L.match(/^=== \[\d\/\d\]\s+(.+?)\s+=+\s*$/);
                  if (step) setExecPhase(`step ${step[1]}/9`);
                  else if (phaseBanner) setExecPhase(phaseBanner[1].replace(/\s*\(.*$/, "").toLowerCase());
                  else if (/PlatformIO not found|installing build toolchain|get-platformio|Installing PlatformIO Core/.test(L)) setExecPhase("installing build tools");
                  else if (/apt-get|Installing |Reading package lists|Unpacking |Setting up /.test(L)) setExecPhase("installing packages");
                  else if (/Cloning into|Receiving objects|Resolving deltas/.test(L)) setExecPhase("cloning sources");
                  else if (/colcon build|Starting >>>|Finished <<</.test(L)) setExecPhase("building agent workspace");
                  else if (/Compiling |Archiving |Indexing |Building \.pio/.test(L)) setExecPhase("compiling firmware");
                  else if (/Linking \.pio|Retrieving maximum program size|Checking size/.test(L)) setExecPhase("linking");
                  else if (/esptool|Writing at |Wrote \d+ bytes|Hash of data verified|Uploading \.pio|Hard resetting|picotool/.test(L)) setExecPhase("flashing MCU");
                  else if (/Scanning I2C bus|Identified Hardware Matrix|Device ACK received/.test(L)) setExecPhase("scanning I2C bus");
                  else if (/\[I2C_JSON\]/.test(L)) setExecPhase("sensors detected");
                  else if (/micro_ros_agent|session established|Waiting for session handshake/.test(L)) setExecPhase("starting micro-ROS agent");
                  else if (/Active ROS 2 Topics|ros2 topic list/.test(L)) setExecPhase("discovering ROS 2 topics");
                  else if (/Test Linearity|Generating LUT|ADC_LUT/.test(L)) setExecPhase("calibrating ADC");
                }

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
                if (_isAdcCal && (textLine.includes("Test Linearity") || textLine.includes("Generating LUT") || textLine.includes("ADC_LUT"))) {
                  const statusText = document.getElementById("adc-status-text");
                  if (statusText) {
                    statusText.innerText = String(textLine).trim();
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

                if (_isAdcCal) adcStreamBuffer += textLine + "\n";
                if (_isAdcCal && adcStreamBuffer.includes("ADC_LUT[4096]") && adcStreamBuffer.split("ADC_LUT[4096]").pop().includes("};")) {
                  try {
                    // Anchor on the LUT declaration so earlier build-log braces /
                    // numbers can't leak into the parsed array.
                    const tail = adcStreamBuffer.substring(adcStreamBuffer.indexOf("ADC_LUT[4096]"));
                    const startIdx = tail.indexOf("{");
                    const endIdx = tail.indexOf("};");
                    if (startIdx !== -1 && endIdx > startIdx) {
                      const inner = tail.substring(startIdx + 1, endIdx);
                      const parsedPoints = inner.split(/[\s,]+/).map(s => parseInt(s.trim(), 10)).filter(n => !isNaN(n));
                      if (parsedPoints.length >= 4000) {
                        currentAdcLut = parsedPoints.slice(0, 4096);
                        while (currentAdcLut.length < 4096) currentAdcLut.push(4095);
                        renderAdcChart(currentRawAdcCurve || currentAdcLut, currentAdcLut, null);
                        const codeDisplay = document.getElementById("adc-lut-code-display");
                        if (codeDisplay) codeDisplay.innerText = formatAdcLutArray(currentAdcLut);
                        // "Run Hardware Calibration" is the whole pipeline now:
                        // sweep -> capture -> chart -> merge into the live spec.
                        // No separate "Merge LUT Table" click needed.
                        mergeAdcLutIntoConfig();
                        _adcLutFreshlyCaptured = true;
                        const statusText = document.getElementById("adc-status-text");
                        if (statusText) {
                          statusText.innerText = "🎉 Hardware ADC Calibration Complete! 4096 points captured & merged.";
                          statusText.style.color = "var(--success)";
                        }
                        showToast("🎉 Hardware ADC Calibration Complete! LUT captured & merged.");
                      }
                    }
                  } catch (e) {
                    console.error("ADC parse error:", e);
                  }
                }
              } else if (eventType === "done") {
                const exitCode = payload.code !== undefined ? payload.code : (payload.exit_code !== undefined ? payload.exit_code : 0);
                if (execPill) {
                  if (execTimerInterval) { clearInterval(execTimerInterval); execTimerInterval = null; }
                  if (execText) execText.textContent = exitCode === 0 ? `${execTitle} · done ✓` : `${execTitle} · exit ${exitCode} ✗`;
                  execPill.className = exitCode === 0 ? "runner-status-pill pill-online" : "runner-status-pill pill-offline";
                  setTimeout(() => { execPill.style.display = "none"; execPill.className = "runner-status-pill pill-checking"; }, 2500);
                }
                if (exitCode === 0) {
                  appendTerminalLine(`\n✅ [SUCCESS] Command finished with exit code 0!`, "out");
                  showToast("✅ Command executed successfully!");
                  // The header written at the START of this run predates the
                  // LUT (the sweep hadn't run yet), so it's still on disk
                  // without it. Now that the port is free again, write it for
                  // real — search-and-merge like every other config write —
                  // and commit it, so the calibration is durably saved without
                  // a second manual click.
                  if (_isAdcCal && _adcLutFreshlyCaptured) _adcLutCommitPending = true;
                } else {
                  appendTerminalLine(`\n❌ [FAILED] Command exited with code ${exitCode}`, "err");
                  showToast(`⚠️ Command exited with code ${exitCode}`);
                }
                _sseDone = true;   // stop reading; hop back to the launching tab now
                _restoreOrigTab();
              } else if (eventType === "error") {
                appendTerminalLine(`❌ [ERROR] ${payload.error}`, "err");
              }
            } catch (e) {
              appendTerminalLine(dataStr, "out");
            }
          }
        }
      }
      if (_sseDone) { try { await reader.cancel(); } catch (_) {} break; }
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
    if (typeof execTimerInterval !== "undefined" && execTimerInterval) clearInterval(execTimerInterval);
    // The hero-badge timer is otherwise only stopped on the "Deployment
    // SUCCEEDED" line, so a failed / aborted / disconnected deploy leaks a
    // 1 Hz setInterval (and stacks one more on every retry).
    if (typeof deployTimerInterval !== "undefined" && deployTimerInterval) {
      clearInterval(deployTimerInterval);
      deployTimerInterval = null;
      const _hb = document.getElementById("hero-deploy-badge");
      if (_hb && _hb.className.indexOf("succeeded") === -1) {
        _hb.className = "deploy-hero-badge";
        const _ht = document.getElementById("hero-deploy-status-text");
        if (_ht) _ht.innerText = "Idle";
        const _htm = document.getElementById("hero-deploy-timer");
        if (_htm) _htm.style.display = "none";
      }
    }
    const _ep = document.getElementById("exec-progress-pill");
    if (_ep && _ep.className.indexOf("pill-online") === -1 && _ep.className.indexOf("pill-offline") === -1) {
      _ep.style.display = "none";
    }
    currentExecController = null;
    // Action done — return the user to the tab they launched it from.
    _restoreOrigTab();
    // Fire the deferred LUT sync+commit now that the port/exec slot is free
    // (never from inside the SSE handling above — that would race the
    // server's single-flight /api/exec against this very command).
    if (_adcLutCommitPending) {
      const _name = currentSpec.robot_name || "robot";
      executeCommandInTerminal(
        getMergeAndCommitCmd(currentSpec, readAutomationOptions()),
        `🔀 Auto-merging & committing calibrated ADC_LUT for '${_name}'`
      );
    }
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
  const btnCopy = document.getElementById("btn-copy-lut-code");

  if (!canvas) return;

  // Theoretical/empirical placeholder curve so the chart isn't blank before
  // the first hardware run. "Run Hardware Calibration" is now the only
  // action in the studio — it sweeps, captures, charts, merges into the
  // live spec, and syncs + commits the result to disk in one click; there
  // used to be separate "Generate Model LUT" and "Merge LUT Table" buttons
  // for those steps.
  generateAdcModelLut(false);

  if (btnRun) {
    btnRun.addEventListener("click", () => {
      runHardwareAdcCalibration();
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

  const statusText = document.getElementById("adc-status-text");
  if (statusText) {
    statusText.innerText = "Placeholder model curve — click Run Hardware Calibration for a real, merged LUT.";
    statusText.style.color = "var(--text-muted)";
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
  if (!mcuHasDac((spec.mcu || "").toUpperCase())) {
    showToast("⚠️ ADC calibration needs a hardware DAC — only ESP32 / ESP32-S2.");
    return;
  }
  const port = document.getElementById("auto-flash-port")?.value.trim() || (isESP32(spec.mcu) ? "/dev/ttyUSB0" : "/dev/ttyACM0");
  const baud = spec.baudrate || spec.telemetry?.baudrate || ((spec.mcu || "").toUpperCase() === "GENDRV" ? 1500000 : 921600);
  const uploadCmd = getDirectUploadOrBuildCmd("adc_calibrate", true);

  // adc_calibrate sweeps the DAC, prints `const int16_t ADC_LUT[4096] = {...};`
  // once (~1-3 min after reset), then idles. The upload command alone never
  // reads the port back, so append a short serial reader — the parser in
  // executeCommandInTerminal() picks the LUT block out of the stream. Staged to
  // a file (quoted heredoc) to sidestep python -c newline/quote escaping.
  const monScript = [
    `cat > /tmp/lr_adc_monitor.py << 'EOF_ADC_MON'`,
    `import serial, sys, time`,
    `s = serial.Serial('${port}', ${baud}, timeout=2)`,
    `time.sleep(0.5); t = time.time(); seen = False`,
    `while time.time() - t < 360:`,
    `    l = s.readline().decode('utf-8', errors='ignore').rstrip()`,
    `    if not l: continue`,
    `    print(l); sys.stdout.flush()`,
    `    if 'ADC_LUT[4096]' in l: seen = True`,
    `    if seen and l.strip().endswith('};'): break`,
    `s.close()`,
    `EOF_ADC_MON`,
    `~/.platformio/penv/bin/python -c 'import serial' 2>/dev/null || ~/.platformio/penv/bin/python -m pip install -q pyserial 2>/dev/null || true`,
    `~/.platformio/penv/bin/python /tmp/lr_adc_monitor.py`
  ].join("\n");
  const cmd = `${uploadCmd}\necho "--- adc_calibrate flashed; capturing LUT over serial (${port} @ ${baud} baud, up to 6 min) ---"\n${monScript}`;

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
  showToast(`✅ ADC LUT merged into the '${name}' configuration — writing it to config/custom/${name}_config.h next.`);

  const statusText = document.getElementById("adc-status-text");
  if (statusText) {
    statusText.innerText = `✅ ADC_LUT merged into the live spec — syncing to config/custom/${name}_config.h...`;
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
// Sync to Config — one click, one result. Used to be two separate
// steps ("Merge & Diff" previewed an in-memory merge; "Save to Firmware" then
// wrote whatever currentSpec happened to be, which silently skipped the
// merge if you saved without merging first). Now it always searches for an
// existing config/custom/<robot>_config.h and merges into it — your changes
// here win, anything else already in that file is kept — then writes the
// result in the same click. A brand-new robot name just gets a fresh file.
async function mergeAndSaveConfig() {
  currentSpec = readSpecFromForm();
  const name = currentSpec.robot_name || 'robot';
  const filename = `${name}_config.h`;

  // Prefer a base the user explicitly loaded this session (Load Existing
  // Config); otherwise look up whatever's on disk for this robot name right
  // now — a fresh /api/configs call, not the possibly-stale cached list, so
  // a file created by another action (e.g. a Full Deploy) this session is
  // still found.
  let baseContent = null;
  if (!baseConfigSpec) {
    try {
      const listRes = await fetch('/api/configs');
      if (listRes.ok) {
        const listData = await listRes.json();
        existingConfigsList = listData.configs || [];
        const hit = existingConfigsList.find(c => c.filename === filename);
        if (hit) baseContent = hit.content;
      }
    } catch (e) { /* offline — fall through to a fresh save */ }
  }

  let headerCode, changeCount = null;
  if (baseConfigSpec || baseContent) {
    try {
      const mergeBody = baseConfigSpec
        ? { base_spec: baseConfigSpec, override_spec: currentSpec }
        : { base_content: baseContent, override_spec: currentSpec };
      const res = await fetch('/api/merge', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(mergeBody)
      });
      const data = await res.json();
      if (data.status === 'ok') {
        currentSpec = data.merged_spec;
        changeCount = data.changes ? data.changes.length : 0;
        headerCode = data.header;
      } else {
        headerCode = generateCppHeader(currentSpec);
      }
    } catch (e) {
      showToast('⚠️ Merge error (saving unmerged): ' + e.message);
      headerCode = generateCppHeader(currentSpec);
    }
  } else {
    headerCode = generateCppHeader(currentSpec);
  }

  try {
    const res = await fetch('/api/save', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        filename: filename,
        header: headerCode,
        wifi_config: generateWifiConfig(currentSpec),
        spec: currentSpec
      })
    });
    const data = await res.json();
    if (data.status === 'saved') {
      updateAutomationPreviews();
      renderActiveCode();
      const mergeNote = changeCount === null ? "new file" : `merged ${changeCount} field(s)`;
      showToast(`💾 Saved config/custom/${filename} (${mergeNote}).` +
        (data.wifi_config_written ? ` WiFi keys → git-ignored wifi_config.h.` : ``));
    } else {
      showToast('⚠️ Save error: ' + (data.error || 'unknown'));
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
  const btnMergeSaveCfg = document.getElementById("btn-merge-save-config");
  if (btnMergeSaveCfg) btnMergeSaveCfg.addEventListener("click", mergeAndSaveConfig);


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
  const rows = [...tbody.querySelectorAll("tr.topic-row")];
  // 7 s window so a 1 Hz topic (/battery, /pressure, /temperature, …) is
  // averaged over ~7 samples instead of 2. Run every row in parallel so the
  // total wait is ~7 s regardless of how many topics there are.
  await Promise.all(rows.map(async (row) => {
    const topic = row.getAttribute("data-topic");
    if (!topic || topic.includes("cmd_vel")) return;
    const badge = row.querySelector(".topic-hz-badge");
    if (badge) { badge.textContent = "Measuring…"; badge.style.opacity = "0.6"; }
    try {
      const res = await fetch(`/api/ros2/hz_single?topic=${encodeURIComponent(topic)}&secs=7`);
      const data = await res.json();
      if (data.status === "ok" && data.hz && badge) {
        badge.textContent = data.hz;
        badge.style.opacity = "1.0";
      } else if (badge) {
        badge.style.opacity = "1.0";
      }
    } catch (e) {
      if (badge) badge.style.opacity = "1.0";
    }
  }));
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

  // Surface the console so the streamed messages are actually visible (the
  // topic table / Hz buttons are higher up the same tab).
  jumpToTerminal();

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
  const _b = document.getElementById("btn-auto-detect-i2c");
  if (_b) { _b.disabled = false; if (_b.dataset._t) _b.textContent = _b.dataset._t; }
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

  // 4. Auto-configure Environmental barometer (env category — BMP280 / BME280)
  const envSelect = document.getElementById("cfg-env");
  if (envSelect) {
    const envDev = (data.devices || []).find((d) => d.category === "env");
    if (envDev) {
      const model = String(envDev.model || "BMP280").toUpperCase();
      for (let opt of envSelect.options) {
        if (opt.value.toUpperCase() === model) { envSelect.value = opt.value; break; }
      }
      if (envSelect.value === "NONE") envSelect.value = "BMP280";
      summaryParts.push(`Env: <strong>${envSelect.value}</strong>`);
      envSelect.dispatchEvent(new Event("change"));
    }
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
  const mcu = (document.getElementById("cfg-mcu")?.value || "GENDRV").toUpperCase();
  // tools/i2c_detect/platformio.ini envs: gendrv, esp32, esp32s3, pico, pico2
  // tools/i2c_detect always runs at 921600 (its own BAUDRATE default / [env]
  // monitor_speed) — NOT the 1.5 Mbaud the GenDrv micro-ROS firmware uses.
  let envName = "esp32", baud = 921600;
  if (mcu.includes("GENDRV")) { envName = "gendrv"; }
  else if (mcu.includes("PICO2")) envName = "pico2";
  else if (mcu.includes("PICO"))  envName = "pico";
  else if (mcu.includes("S3"))    envName = "esp32s3";
  else if (mcu.includes("S2"))    envName = "esp32";   // no s2 env; override pins carry it
  const distro = mcu.includes("GENDRV") ? "gendrv" : envName;

  const portSelect = document.getElementById("auto-flash-port");
  const port = (portSelect && portSelect.value) ? portSelect.value.trim()
             : (envName === "gendrv" || envName === "esp32" || envName === "esp32s3" ? "/dev/ttyUSB0" : "/dev/ttyACM0");
  const uploadFlag = (port && !port.includes("BOOTSEL")) ? ` --upload-port ${port}` : "";

  // Scan the pins currently set in the Pinout matrix (not whatever the tracked
  // board header happens to define). Passed as build flags the tools/i2c_detect
  // firmware honours ahead of BOARD_INIT.
  const sda = parseInt(document.getElementById("pin-i2c-sda")?.value, 10);
  const scl = parseInt(document.getElementById("pin-i2c-scl")?.value, 10);
  const havePins = Number.isInteger(sda) && Number.isInteger(scl) && sda >= 0 && scl >= 0;
  const pfx = havePins ? `PLATFORMIO_BUILD_FLAGS="-D I2C_SDA_OVERRIDE=${sda} -D I2C_SCL_OVERRIDE=${scl}" ` : "";
  const pinNote = havePins ? ` on SDA ${sda} / SCL ${scl}` : "";

  // i2c_detect resets on port open, then boots (~1.5 s) and prints the scan +
  // the [I2C_JSON] line, repeating every 5 s. Read up to 20 s, stop at the
  // first JSON (or the "no devices" line).
  const monitorPy = "import serial, sys, time; s = serial.Serial('" + port + "', " + baud + ", timeout=2); t = time.time();\nwhile time.time() - t < 20:\n  l = s.readline().decode('utf-8', errors='ignore').strip()\n  if not l: continue\n  print(l); sys.stdout.flush()\n  if '[I2C_JSON]' in l or 'No I2C devices detected' in l: break\ns.close()";
  const rd = document.getElementById("i2c-detect-results");
  if (rd) {
    rd.style.display = "block";
    rd.innerHTML = `<span style="color:#38bdf8;">⏳ Watch the <strong>Automation &amp; Flash</strong> tab terminal — installs tools (first run only) → builds &amp; flashes <code>tools/i2c_detect</code>${pinNote} → scans the bus. Returns here when done.</span>`;
  }
  const btn = document.getElementById("btn-auto-detect-i2c");
  let _btnRestore = null;
  if (btn) {
    btn.disabled = true; btn.dataset._t = btn.textContent; btn.textContent = "⏳ Scanning…";
    _btnRestore = () => { btn.disabled = false; if (btn.dataset._t) btn.textContent = btn.dataset._t; };
    setTimeout(() => { if (btn.disabled) _btnRestore(); }, 360000);
  }
  // (executeCommandInTerminal activates the Automation tab so this streams on-screen.)
  const rwFix = `for _d in /dev/ttyUSB* /dev/ttyACM*; do [ -e "$_d" ] && [ ! -w "$_d" ] && sudo chmod a+rw "$_d" 2>/dev/null || true; done`;
  // Guarantee the pyserial the mini-monitor below imports (penv python has pip).
  const ensurePyserial = `~/.platformio/penv/bin/python -c 'import serial' 2>/dev/null || ~/.platformio/penv/bin/python -m pip install -q pyserial 2>/dev/null || python3 -m pip install -q --user pyserial 2>/dev/null || true`;
  // Explicit phase banners so the streamed terminal output reads as
  // installing tools -> building/flashing -> scanning.
  let cmd;
  if (remoteConfig.targetMode === "remote") {
    const host = remoteConfig.remoteHost;
    const user = remoteConfig.remoteUser;
    const sshPort = remoteConfig.remotePort || 22;
    const sshKey = remoteConfig.remoteKey;
    const sshOpts = `-o StrictHostKeyChecking=no -p ${sshPort}` + (sshKey ? ` -i ${sshKey}` : "");
    const isPico = envName.includes("pico");
    const binExt = isPico ? "uf2" : "bin";
    const binFile = `tools/i2c_detect/.pio/build/${envName}/firmware.${binExt}`;
    const remoteDest = `/tmp/i2c_detect.${binExt}`;

    cmd = [
      `set -e`,
      `echo; echo "=== [1/4] Build tools (Laptop PlatformIO) ========================"`,
      ensurePioToolchainSnippet(),
      `echo; echo "=== [2/4] Building i2c_detect firmware locally (${envName}) ===="`,
      `cd tools/i2c_detect && ${pfx}pio run -e ${envName}`,
      `cd ../..`,
      `echo; echo "=== [3/4] Transferring & Flashing on remote robot (${user}@${host}) ===="`,
      `scp ${sshOpts} ${binFile} ${user}@${host}:${remoteDest}`,
      isPico 
        ? `ssh ${sshOpts} ${user}@${host} "picotool load -f ${remoteDest} -x || sudo cp ${remoteDest} /media/*/RPI-RP2/ 2>/dev/null"`
        : `ssh ${sshOpts} ${user}@${host} "which esptool.py >/dev/null 2>&1 || python3 -m esptool version >/dev/null 2>&1 || python3 -m pip install -q --user esptool 2>/dev/null || true; python3 -m esptool --port ${port} --baud 460800 --before default_reset --after hard_reset write_flash 0x10000 ${remoteDest} || esptool.py --port ${port} --baud 460800 write_flash 0x10000 ${remoteDest}"`,
      `echo; echo "=== [4/4] Scanning I2C bus over remote serial (${port} @ ${baud}) ===="`,
      `ssh ${sshOpts} ${user}@${host} "python3 -c \"${monitorPy}\" || (stty -F '${port}' ${baud} raw -echo 2>/dev/null && timeout 15 cat '${port}')"`,
    ].join("\n");
  } else {
    cmd = [
      `set -e`,
      rwFix,
      `echo; echo "=== [1/3] Build tools (PlatformIO) ================================"`,
      ensurePioToolchainSnippet(),
      ensurePyserial,
      `echo; echo "=== [2/3] Building & flashing i2c_detect firmware (${envName}) ===="`,
      `cd tools/i2c_detect`,
      `${pfx}pio run -e ${envName} -t upload${uploadFlag}`,
      `echo; echo "=== [3/3] Scanning the I2C bus${pinNote} ========================="`,
      `~/.platformio/penv/bin/python -c "${monitorPy}"`,
    ].join("\n");
  }
  const p = executeCommandInTerminal(cmd, `🔍 AI Auto-Detecting I2C Sensors (${envName}${pinNote} -> ${port})`);
  if (p && typeof p.finally === "function" && _btnRestore) p.finally(_btnRestore);
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
  // /api/syslog/status reports data.running; the start/stop responses report
  // data.status ("running"/"already_running"/"stopped"). Accept either.
  const running = (typeof data.running === "boolean")
    ? data.running
    : (data.status === "running" || data.status === "already_running");
  data = { ...data, running };
  isSyslogRunning = running;
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
  const otaIp = document.getElementById('cfg-ota-ip');
  if (otaIp && !otaIp.value && data.last_sender) otaIp.value = data.last_sender;
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
      const ipEl = document.getElementById('cfg-syslog-ip');
      if (ipEl && data.host_ip) ipEl.value = data.host_ip;
      const portEl = document.getElementById('cfg-syslog-port');
      if (portEl && data.port) portEl.value = data.port;   // reflect the bound port in the config
      if (data.host_ip) showToast(`📡 Syslog target set to ${data.host_ip}:${data.port}`);
      recomputeAll();
      connectSyslogStream();
      const fallbackNote = data.fallback ? ` (fell back to :${data.port})` : '';
      showToast(`🟢 Syslog Server listening on UDP port ${data.port}${fallbackNote}!`);
      
      appendTerminalLine(`[SYSLOG SERVER] Listening on UDP 0.0.0.0:${data.port} | Saving to ${data.logfile}`, "in");
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
    appendTerminalLine(`[SYSLOG SERVER] Stopped (Captured ${data.packets || 0} packets).`, "dim");
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
        const esc = (s) => String(s == null ? '' : s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
        const timePart = data.time ? data.time.split(' ')[1] || data.time : '';
        appendTerminalHtml(
          `<span class="badge-syslog">SYSLOG</span> <span class="syslog-time">${esc(timePart)}</span> <span class="syslog-sender">[${esc(data.sender)}]</span> <span class="syslog-msg">${esc(data.message)}</span>`,
          'syslog'
        );
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


async function checkAndDisplayAgentPortStatus() {
  const iconElem = document.getElementById("agent-port-indicator-icon");
  const textElem = document.getElementById("agent-port-indicator-text");
  const btnRelease = document.getElementById("btn-release-agent-port");
  if (!iconElem || !textElem) return;

  const port = document.getElementById("auto-flash-port")?.value.trim() || "/dev/ttyUSB0";
  const remoteHost = (document.getElementById("hdr-remote-flash")?.value || "").trim();
  let user = "ubuntu";
  let host = "";
  if (remoteHost) {
    if (remoteHost.includes("@")) {
      [user, host] = remoteHost.split("@");
    } else {
      host = remoteHost;
    }
  }

  iconElem.textContent = "⏳";
  textElem.textContent = `Checking ${port}...`;

  try {
    const url = `/api/agent/port_check?port=${encodeURIComponent(port)}&mode=serial&host=${encodeURIComponent(host)}&user=${encodeURIComponent(user)}`;
    const res = await fetch(url).then(r => r.json());
    if (res.status === "ok") {
      if (res.in_use) {
        iconElem.textContent = "⚠️";
        textElem.innerHTML = `<strong>Conflict:</strong> ${res.summary || "Port is occupied"} <span class="text-secondary" style="font-size:0.75rem;">(${res.details || ""})</span>`;
        if (btnRelease) btnRelease.style.display = "inline-block";
      } else {
        iconElem.textContent = "🟢";
        textElem.innerHTML = `Port <code>${res.target}</code> is free and ready`;
        if (btnRelease) btnRelease.style.display = "none";
      }
    } else {
      iconElem.textContent = "ℹ️";
      textElem.textContent = res.details || "Port check unavailable";
      if (btnRelease) btnRelease.style.display = "none";
    }
  } catch (err) {
    iconElem.textContent = "ℹ️";
    textElem.textContent = "Port check skipped (offline)";
    if (btnRelease) btnRelease.style.display = "none";
  }
}

async function releaseAgentPortFromUI() {
  const iconElem = document.getElementById("agent-port-indicator-icon");
  const textElem = document.getElementById("agent-port-indicator-text");
  const btnRelease = document.getElementById("btn-release-agent-port");
  const port = document.getElementById("auto-flash-port")?.value.trim() || "/dev/ttyUSB0";
  const remoteHost = (document.getElementById("hdr-remote-flash")?.value || "").trim();
  let user = "ubuntu";
  let host = "";
  if (remoteHost) {
    if (remoteHost.includes("@")) {
      [user, host] = remoteHost.split("@");
    } else {
      host = remoteHost;
    }
  }

  if (iconElem) iconElem.textContent = "⏳";
  if (textElem) textElem.textContent = `Releasing ${port}...`;

  try {
    const res = await fetch("/api/agent/port_release", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port, mode: "serial", host, user })
    }).then(r => r.json());

    if (res.status === "ok") {
      await checkAndDisplayAgentPortStatus();
    } else {
      if (textElem) textElem.textContent = `Release error: ${res.error || "failed"}`;
    }
  } catch (err) {
    if (textElem) textElem.textContent = `Release failed: ${err.message}`;
  }
}
