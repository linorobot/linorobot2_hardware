# Copyright (c) 2026 Thomas Chou, Paul Bouchier, Linorobot contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import re
from typing import Dict, Any


def _n(v) -> str:
    """Format a number the way the hand-written headers do: whole values as
    plain ints (12, not 12.0), fractional values without trailing zeros."""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    if f == int(f):
        return str(int(f))
    return repr(f).rstrip("0").rstrip(".")


def _format_adc_lut(lut) -> str:
    """`const int16_t ADC_LUT[4096] = { ... };`, 16 values/row — matches the
    web UI's formatAdcLutArray() so a browser-generated and a server-generated
    header are byte-identical."""
    vals = [int(round(float(x))) for x in (lut or [])]
    if not vals:
        return "const int16_t ADC_LUT[4096] = { /* insert adc_calibrate data here */ };"
    rows = []
    for i in range(0, len(vals), 16):
        chunk = vals[i:i + 16]
        row = "    " + ", ".join(str(v) for v in chunk)
        if i + 16 < len(vals):
            row += ","
        rows.append(row)
    return "const int16_t ADC_LUT[4096] = {\n" + "\n".join(rows) + "\n};"


def _ipv4_c(ip: str) -> str:
    parts = [p.strip() for p in str(ip).split(".")]
    if len(parts) == 4 and all(p.isdigit() for p in parts):
        return "{ " + ", ".join(parts) + " }"
    return "{ 192, 168, 1, 100 }"


def generate_config_header(spec: Dict[str, Any]) -> str:
    """
    Generate a Linorobot2 C++ configuration header from a spec dict.
    Pin assignments are emitted in the same macro format used by the
    original hand-written config headers so that round-trip
    parse → generate → re-parse is lossless.
    """
    name     = spec.get("robot_name", "custom_robot").upper()
    kine     = spec.get("kinematics", "DIFFERENTIAL_DRIVE")
    geom     = spec.get("geometry", {})
    motors   = spec.get("motors", {})
    sensors  = spec.get("sensors", {})
    pins     = spec.get("pins", {})
    telemetry = spec.get("telemetry", {})
    driver   = motors.get("driver_type", "BTS7960")

    # ── Header guard & kinematics ────────────────────────────────────
    lines = [
        "// Copyright (c) 2026 Thomas Chou, Paul Bouchier, Linorobot contributors",
        f"// Auto-generated configuration for {spec.get('robot_name', 'custom_robot')}",
        f"#ifndef {name}_CONFIG_H",
        f"#define {name}_CONFIG_H",
        "",
        f"#define LINO_BASE {kine}",
        "",
        "// Robot Physical Geometry",
        f"#define WHEEL_DIAMETER {_n(geom.get('wheel_diameter', 0.065))}",
        f"#define LR_WHEELS_DISTANCE {_n(geom.get('track_width', 0.20))}",
    ]
    # FR_WHEELS_DISTANCE only matters for 4-wheel bases
    if "wheelbase" in geom and kine != "DIFFERENTIAL_DRIVE":
        lines.append(f"#define FR_WHEELS_DISTANCE {_n(geom['wheelbase'])}")
    if geom.get("weight") is not None:
        lines.append(f"#define ROBOT_WEIGHT {_n(geom['weight'])}")

    # ── Motor driver & characteristics ───────────────────────────────
    # Simulation mode (Fake Wheel and/or Fake LD19)
    sim = spec.get("simulation") or {}
    if sim.get("fake_wheel") or sim.get("fake_ld19"):
        # A simulated config looks entirely normal from the outside and the
        # board reports plausible topics, so flashing one to a real robot ends
        # with a machine that sits still while /odom insists it is driving.
        # Say so where it cannot be missed.
        lines.extend([
            "",
            "// ===========================================================================",
            "// SIMULATION MODE IS ENABLED -- THIS IS NOT A REAL ROBOT CONFIG",
            "// Comment out the USE_FAKE_* defines below before flashing a robot whose",
            "// motors, encoders or LiDAR are actually wired, or they will be ignored.",
            "// ===========================================================================",
        ])
    if sim.get("fake_wheel"):
        lines.extend([
            "",
            "// Simulated drivetrain -- no motors or encoders need to be attached",
            "#define USE_FAKE_WHEEL",
        ])
        if sim.get("robot_mass") is not None:
            lines.append(f"#define FAKE_ROBOT_MASS {sim['robot_mass']:.2f} // kg")
        if sim.get("wheel_noise_rpm") is not None:
            lines.append(f"#define FAKE_WHEEL_NOISE_RPM {sim['wheel_noise_rpm']:.2f} // +/- peak encoder noise")

    # The emulated scan leaves the board either over the LiDAR UART pin or over
    # WiFi UDP. With neither, there is nowhere for it to go, so the emulator is
    # not enabled at all: turning it on would burn the loop building packets and
    # writing them into an unconnected pad, while the board reported success and
    # no scan ever reached ROS. The pin is unset by default because on a stock
    # board nothing is wired to it -- wire it, set the pin, and this switches on.
    lidar_rxd = pins.get("lidar_rxd")
    if lidar_rxd is None:
        lidar_rxd = (spec.get("advanced") or {}).get("lidar_rxd")
    # A negative pin is "not wired", not a pin number.
    lidar_pin_wired = lidar_rxd is not None and int(lidar_rxd) >= 0
    lidar_over_udp = bool((spec.get("telemetry") or {}).get("use_lidar_udp"))
    ld19_routed = lidar_pin_wired or lidar_over_udp

    if sim.get("fake_ld19") and not ld19_routed:
        lines.extend([
            "",
            "// Fake LD19 requested but NOT enabled: no LiDAR UART pin is wired",
            "// and LiDAR-over-UDP is off, so the simulated scan would have no",
            "// route out. Any route over the serial pin needs a physical wire:",
            "// fake LD19 transmits the scan on it, and a real LD19 forwarded",
            "// over UDP must be wired into it, since UDP only carries what the",
            "// board receives. Fake LD19 over WiFi UDP needs no wire at all.",
            "// Wire IO4 (gendrv) and set the pin, or turn on LiDAR over UDP.",
        ])

    if sim.get("fake_ld19") and ld19_routed:
        lines.extend([
            "",
            "// Simulated LD19 LiDAR -- raycasts 2D room + obstacle wall to LIDAR_RXD",
            "#define USE_FAKE_LD19",
        ])
        if lidar_pin_wired:
            lines.append(f"#define LIDAR_RXD {int(lidar_rxd)}")
        if sim.get("lidar_baudrate") is not None:
            lines.append(f"#define LIDAR_BAUDRATE {int(sim['lidar_baudrate'])}")
        if sim.get("map_width") is not None:
            lines.append(f"#define FAKE_MAP_WIDTH {sim['map_width']:.1f}f // room width (m)")
        if sim.get("map_height") is not None:
            lines.append(f"#define FAKE_MAP_HEIGHT {sim['map_height']:.1f}f // room height (m)")
        if sim.get("wall_obstacle") is not None:
            wall_val = 1 if sim["wall_obstacle"] else 0
            lines.append(f"#define FAKE_WALL_OBSTACLE {wall_val}")
        if sim.get("wall_x1") is not None:
            lines.append(f"#define FAKE_WALL_X1 {sim['wall_x1']:.2f}f")
        if sim.get("wall_y1") is not None:
            lines.append(f"#define FAKE_WALL_Y1 {sim['wall_y1']:.2f}f")
        if sim.get("wall_x2") is not None:
            lines.append(f"#define FAKE_WALL_X2 {sim['wall_x2']:.2f}f")
        if sim.get("wall_y2") is not None:
            lines.append(f"#define FAKE_WALL_Y2 {sim['wall_y2']:.2f}f")
        # Defaults on: the range comes from the same raycast as the scan, so it
        # needs no pins, and a simulated robot driven into a wall should get the
        # same protection a real one would have.
        if sim.get("fake_sonar", True):
            lines.extend([
                "",
                "// Simulated forward sonar -- nearest return in a cone of the same",
                "// raycast that feeds the scan. No pins, no wiring.",
                "#define USE_FAKE_SONAR",
                "// Firmware-side hazard stop: blocks forward motion inside",
                "// SAFETY_STOP_RANGE and publishes the state on /safety_stop.",
                "// Reverse stays available so the robot can back out.",
                "#define USE_SAFETY_STOP",
            ])

    lines.extend([
        "",
        "// Motor Driver & Characteristics",
        f"#define USE_{driver}_MOTOR_DRIVER",
        f"#define MOTOR_MAX_RPM {motors.get('max_rpm', 300)}",
        f"#define MAX_RPM_RATIO 0.85",
        f"#define MOTOR_OPERATING_VOLTAGE {_n(motors.get('operating_voltage', 12.0))}",
        f"#define MOTOR_POWER_MAX_VOLTAGE {_n(motors.get('max_voltage', 12.0))}",
        f"#define MOTOR_POWER_MEASURED_VOLTAGE {_n(motors.get('measured_voltage', motors.get('max_voltage', 12.0)))}",
        "",
        f"#define PWM_BITS {motors.get('pwm_bits', 10)}",
        f"#define PWM_FREQUENCY {motors.get('pwm_frequency', 20000)}",
        f"#define PWM_MAX pow(2, PWM_BITS) - 1",
        f"#define PWM_MIN -PWM_MAX",
        "",
        f"#define COUNTS_PER_REV1 {motors.get('cpr', 1320)}",
        f"#define COUNTS_PER_REV2 {motors.get('cpr', 1320)}",
        f"#define COUNTS_PER_REV3 {motors.get('cpr', 1320)}",
        f"#define COUNTS_PER_REV4 {motors.get('cpr', 1320)}",
        "",
        "// INVERT MOTOR DIRECTIONS",
        f"#define MOTOR1_INV {str(motors.get('motor1_inv', False)).lower()}",
        f"#define MOTOR2_INV {str(motors.get('motor2_inv', True)).lower()}",
        f"#define MOTOR3_INV {str(motors.get('motor3_inv', False)).lower()}",
        f"#define MOTOR4_INV {str(motors.get('motor4_inv', True)).lower()}",
        "",
        "// INVERT ENCODER COUNTS",
        f"#define MOTOR1_ENCODER_INV {str(pins.get('encoders', {}).get('m1_inv', False)).lower()}",
        f"#define MOTOR2_ENCODER_INV {str(pins.get('encoders', {}).get('m2_inv', False)).lower()}",
        f"#define MOTOR3_ENCODER_INV {str(pins.get('encoders', {}).get('m3_inv', False)).lower()}",
        f"#define MOTOR4_ENCODER_INV {str(pins.get('encoders', {}).get('m4_inv', False)).lower()}",
        "",
        "// Velocity PID Tuning Constants",
        f"#define K_P {_n(spec.get('pid', {}).get('kp', 0.6))}",
        f"#define K_I {_n(spec.get('pid', {}).get('ki', 0.8))}",
        f"#define K_D {_n(spec.get('pid', {}).get('kd', 0.5))}",
        "",
        "// Pin Assignments",
    ])

    # ── LED pin ──────────────────────────────────────────────────────
    mcu_name = spec.get("mcu", "").upper()
    led_pin = pins.get("led")
    if led_pin is None:
        if mcu_name in ["PICOW", "PICO2W"]:
            led_pin = "LED_BUILTIN"
        elif "PICO" in mcu_name:
            led_pin = 25
        elif "GENDRV" in mcu_name or "ESP32" in mcu_name:
            led_pin = "LED_BUILTIN"
        else:
            led_pin = 13
    lines.append(f"#define LED_PIN {led_pin}")

    # ── Encoder pins (all 4 motors, always) ──────────────────────────
    # These are unconditional #defines (not inside #ifdef blocks)
    enc = pins.get("encoders", {})
    lines.append("")
    lines.append("// ENCODER PINS")
    for i in range(1, 5):
        a = enc.get(f"m{i}_a", -1)
        b = enc.get(f"m{i}_b", -1)
        lines.append(f"#define MOTOR{i}_ENCODER_A {a}")
        lines.append(f"#define MOTOR{i}_ENCODER_B {b}")

    # ── Motor driver pins inside #ifdef block ─────────────────────────
    # Emit in exactly the same format as the hand-written headers:
    # one #ifdef USE_*_MOTOR_DRIVER block containing all 4 motors.
    lines.append("")
    lines.append(f"// MOTOR PINS")
    lines.append(f"#ifdef {_driver_macro(driver)}")
    for i in range(1, 5):
        m = pins.get(f"motor{i}", {})
        if driver == "BTS7960":
            # BTS7960 uses two outputs only: IN_A = RPWM, IN_B = LPWM.
            # MOTOR{i}_PWM is an unused arg in the driver class -> fixed placeholder.
            lines.append(f"  #define MOTOR{i}_PWM -1 //DON'T TOUCH THIS! This is just a placeholder")
            lines.append(f"  #define MOTOR{i}_IN_A {m.get('in_a', m.get('pwm_l', -1))}")
            lines.append(f"  #define MOTOR{i}_IN_B {m.get('in_b', m.get('en', -1))}")
        elif driver == "GENERIC_2_IN":
            lines.append(f"  #define MOTOR{i}_PWM {m.get('pwm', -1)}")
            lines.append(f"  #define MOTOR{i}_IN_A {m.get('in_a', -1)}")
            lines.append(f"  #define MOTOR{i}_IN_B {m.get('in_b', -1)}")
        elif driver == "GENERIC_1_IN":
            lines.append(f"  #define MOTOR{i}_PWM {m.get('pwm', -1)}")
            lines.append(f"  #define MOTOR{i}_IN_A {m.get('dir', m.get('in_a', -1))}")
            lines.append(f"  #define MOTOR{i}_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder")
        elif driver == "ESC":
            lines.append(f"  #define MOTOR{i}_PWM {m.get('pwm', -1)}")
            lines.append(f"  #define MOTOR{i}_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder")
            lines.append(f"  #define MOTOR{i}_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder")
    lines.append(f"  #define PWM_MAX pow(2, PWM_BITS) - 1")
    lines.append(f"  #define PWM_MIN -PWM_MAX")
    lines.append(f"#endif")

    # ── Servo for Ackermann ───────────────────────────────────────────
    if kine == "ACKERMANN":
        servo_pin = pins.get("servo", 21)
        lines.append(f"#define STEERING_SERVO_PIN {servo_pin}")

    # ── Sensor configuration ──────────────────────────────────────────
    lines.extend(["", "// Sensor Configuration"])

    # Form-shape (sensors.imu, short name e.g. "QMI8658") checked before
    # parser-shape (sensors.imu_type, full macro): parser.py's default spec
    # always sets imu_type (even "USE_FAKE_IMU" as a placeholder), so an
    # overlay that only sets imu — every web-UI form does — would otherwise
    # always lose to a stale/default imu_type already in the base spec on
    # any merge (same key-mismatch class as battery/sonar/INA219 above).
    imu = sensors.get("imu") or sensors.get("imu_type", "USE_FAKE_IMU")
    if imu and imu not in ["NONE", "USE_FAKE_IMU", "FAKE"]:
        macro = imu if imu.startswith("USE_") else f"USE_{imu}_IMU"
        lines.append(f"#define {macro}")
    else:
        lines.append("// #define USE_FAKE_IMU")

    mag = sensors.get("mag") or sensors.get("mag_type", "USE_FAKE_MAG")   # see imu comment above
    if mag and mag not in ["NONE", "USE_FAKE_MAG", "FAKE"]:
        macro = mag if mag.startswith("USE_") else f"USE_{mag}_MAG"
        lines.append(f"#define {macro}")
    else:
        lines.append("// #define USE_FAKE_MAG")

    # IMU / magnetometer tuning. MAG_BIAS is #ifdef-gated in firmware.ino;
    # ACCEL_COV / GYRO_COV / ORI_COV are #ifndef-gated in imu_interface.h
    # (blank == firmware default of 0.00001).
    tune = spec.get("imu_tuning", {})
    mb = tune.get("mag_bias")
    if isinstance(mb, (list, tuple)) and len(mb) == 3:
        lines.append(f"#define MAG_BIAS {{ {', '.join(_n(x) for x in mb)} }}")
    # ACCEL/GYRO/ORI/MAG_COV are 3-vectors; POSE/TWIST_COV are 6-vectors — a
    # scalar expands, a list is used as-is. All #ifndef-gated in firmware.
    # When a known sensor is enabled and the spec left a covariance blank, fall
    # back to a datasheet-derived default (variance ≈ (noise_density·√100 Hz)²)
    # so the header ships realistic values instead of the firmware's 1e-5.
    _imu_key = (sensors.get("imu") or str(sensors.get("imu_type") or "")
                ).replace("USE_", "").replace("_IMU", "").upper()
    _mag_key = (sensors.get("mag") or str(sensors.get("mag_type") or "")
                ).replace("USE_", "").replace("_MAG", "").upper()
    _env_key = str(sensors.get("env") or sensors.get("env_type") or "").upper()
    _imu_cov = {
        "BNO085": {"accel_cov": 2.2e-4, "gyro_cov": 1.5e-6, "ori_cov": 4e-3},
        "LSM6DSOX": {"accel_cov": 4.7e-5, "gyro_cov": 4.4e-7},
        "ICM20948": {"accel_cov": 5.1e-4, "gyro_cov": 6.9e-6},
        "QMI8658": {"accel_cov": 8e-5, "gyro_cov": 2e-6},
        "MPU9250": {"accel_cov": 9e-4, "gyro_cov": 3e-6},
        "MPU9150": {"accel_cov": 1.5e-3, "gyro_cov": 3e-6},
        "MPU6050": {"accel_cov": 1.5e-3, "gyro_cov": 3e-6},
        "GY85": {"accel_cov": 1.8e-3, "gyro_cov": 4.4e-5},
    }.get(_imu_key, {})
    _mag_cov = {"AK09918": 2.3e-14, "ICM20948": 2.3e-14, "QMC5883L": 4e-14,
                "HMC5883L": 4e-14, "AK8963": 9e-14, "AK8975": 9e-14}.get(_mag_key)
    # pressure Pa² (σ≈1.7 Pa), temperature °C² (±0.5°C accuracy), humidity (0..1)² (±3% RH)
    _env_cov = {"BMP280": [3, 0.25, 0], "BME280": [3, 0.25, 9e-4]}.get(_env_key)
    _cov_defaults = dict(_imu_cov)
    if _mag_cov is not None:
        _cov_defaults["mag_cov"] = _mag_cov
    if _env_cov is not None:
        _cov_defaults["env_cov"] = _env_cov

    for _macro, _key, _len in (
        ("ACCEL_COV", "accel_cov", 3), ("GYRO_COV", "gyro_cov", 3),
        ("ORI_COV", "ori_cov", 3), ("MAG_COV", "mag_cov", 3),
        ("POSE_COV", "pose_cov", 6), ("TWIST_COV", "twist_cov", 6),
        ("ENV_COV", "env_cov", 3),   # BMP280 pressure/temperature/humidity .variance
    ):
        _v = tune.get(_key)
        if _v is None or _v == "":
            _v = _cov_defaults.get(_key)
        if _v is None or _v == "":
            continue
        _t = list(_v) if isinstance(_v, (list, tuple)) else [_v] * _len
        lines.append(f"#define {_macro} {{ {', '.join(_n(x) for x in _t)} }}")

    # I2C — BOARD_INIT is synthesised only when the user has actually defined
    # I2C pins (SDA_PIN / SCL_PIN >= 0). A bare module leaves them unset: no
    # SDA_PIN/SCL_PIN and no BOARD_INIT, so firmware.ino's default Wire.begin()
    # path applies.
    # pins.i2c (form-shape) checked before sensors.i2c_sda/scl (parser-shape):
    # the web-UI form always writes pins.i2c, never sensors.i2c_sda/scl, so
    # the parser-shape key from an existing base spec would otherwise always
    # win on merge (same key-mismatch class as imu/mag above).
    _i2c_pins = pins.get("i2c", {})
    i2c_sda = _i2c_pins.get("sda")
    i2c_scl = _i2c_pins.get("scl")
    i2c_sda = sensors.get("i2c_sda", -1) if i2c_sda is None else i2c_sda
    i2c_scl = sensors.get("i2c_scl", -1) if i2c_scl is None else i2c_scl
    have_imported_board_init = any(d.get("name") == "BOARD_INIT" for d in spec.get("raw_defines", []))
    if i2c_sda >= 0 and i2c_scl >= 0:
        lines.append(f"#define SDA_PIN {i2c_sda}")
        lines.append(f"#define SCL_PIN {i2c_scl}")
        # SDA_PIN/SCL_PIN are only consulted through BOARD_INIT
        # (firmware.ino: `#ifdef BOARD_INIT ... #else Wire.begin(); #endif`).
        # An imported header's own BOARD_INIT is kept verbatim by the
        # passthrough below; only synthesise one when the import lacked it.
        if not have_imported_board_init:
            is_rp2 = "PICO" in mcu_name or "RP2" in mcu_name
            bi = ["#define BOARD_INIT { \\"]
            if is_rp2:
                bi.append("    Wire.setSDA(SDA_PIN); \\")
                bi.append("    Wire.setSCL(SCL_PIN); \\")
                bi.append("    Wire.begin(); \\")
            else:
                bi.append("    Wire.begin(SDA_PIN, SCL_PIN); \\")
            bi.append("    Wire.setClock(400000); \\")
            bi.append("}")
            lines.extend(bi)

    # Battery. Same key-name split as everywhere else in this block: the
    # web-UI form (readSpecFromForm()) only ever sets battery_monitor to the
    # string "INA219"/"ADC_DIVIDER"/"NONE"; only a spec parsed from an
    # existing header also carries the legacy boolean use_ina219 parser.py
    # emits. Checking use_ina219 alone meant a brand-new robot (nothing to
    # merge with, so the raw form overlay reaches generate_config_header()
    # untouched) silently got NO #define USE_INA219 at all, even with i2c
    # auto-detect having correctly set battery_monitor="INA219" in the form.
    if sensors.get("battery_monitor") == "INA219" or sensors.get("use_ina219", False):
        lines.append("#define USE_INA219")
    else:
        bat_pin = pins.get("battery_pin", sensors.get("battery_pin", -1))
        if bat_pin is not None and bat_pin >= 0:
            lines.append(f"#define BATTERY_PIN {bat_pin}")
            # DAC output pin for the adc_calibrate sweep. Only the classic ESP32,
            # the ESP32-S2 and the ESP32-based Waveshare General Driver carry a
            # hardware DAC; everything else (ESP32-S3/C3, RP2040/RP2350, Teensy)
            # has none, so no DAC_PIN is emitted there.
            _dac_pin = pins.get("dac_pin", sensors.get("dac_pin"))
            _mcu_has_dac = mcu_name in ("ESP32", "ESP32S2", "GENDRV")
            if _mcu_has_dac and _dac_pin is not None and int(_dac_pin) >= 0:
                lines.append(f"#define DAC_PIN {int(_dac_pin)}")
            r1 = sensors.get("battery_r1", sensors.get("battery_divider_r1", 30000.0))
            r2 = sensors.get("battery_r2", sensors.get("battery_divider_r2", 7500.0))
            # firmware/lib/battery/battery.cpp: `return BATTERY_ADJUST(reading);`
            # (no #ifndef fallback) — an ADC battery config MUST define it.
            adc_lut = sensors.get("adc_lut")
            r1k, r2k = _n(float(r1) / 1000.0), _n(float(r2) / 1000.0)
            if adc_lut:
                # A real calibration (adc_lut present) is always fresh, explicit
                # user intent — "I just ran Run Hardware Calibration" — so it
                # wins unconditionally, even over a BATTERY_ADJUST this same
                # generator wrote into an earlier version of this file (which
                # a re-parse can't tell apart from a hand customization, so it
                # would otherwise sit in raw_defines and block a rewrite here
                # forever). The line below still ends up in `emitted` further
                # down, so the stale raw_defines copy is skipped, not doubled.
                lines.append("#define USE_ADC_LUT")
                lines.append(_format_adc_lut(adc_lut))
                lines.append(f"#define BATTERY_ADJUST(v) (ADC_LUT[v] * (3.3 / 4096 * ({r1k} + {r2k}) / {r2k}))")
            elif not any(d.get("name") == "BATTERY_ADJUST" for d in spec.get("raw_defines", [])):
                # firmware/lib/battery/battery.cpp: `return BATTERY_ADJUST(reading);`
                # (no #ifndef fallback) — an ADC battery config MUST define it.
                if "PICO" in mcu_name or "RP2" in mcu_name:
                    lines.append(f"#define BATTERY_ADJUST(v) ((v) * (3.3 / 4096 * ({r1k} + {r2k}) / {r2k}))")
                else:  # ESP32 family: analogReadMilliVolts() returns millivolts
                    lines.append(f"#define BATTERY_ADJUST(v) ((v) * (({r1k} + {r2k}) / {r2k}) / 1000.0)")

    for macro, keys in [
        ("BATTERY_CAP", ("battery_cap", "battery_capacity")),
        ("BATTERY_MIN", ("battery_min", "battery_min_voltage")),
        ("BATTERY_MAX", ("battery_max", "battery_max_voltage")),
        ("BATTERY_DIP", ("battery_dip",)),
    ]:
        val = next((sensors[k] for k in keys if sensors.get(k) is not None), None)
        if val is not None:
            lines.append(f"#define {macro} {_n(val)}")

    # Sonar. firmware.ino/range.cpp actually gate on TRIG_PIN/ECHO_PIN alone —
    # USE_SONAR isn't read anywhere — but app.js's generateCppHeader() (the
    # client-side preview) emits it as a documentation marker next to them,
    # matching USE_INA219/USE_BMP280/etc.; kept in parity here so a
    # server-merged header (search-and-merge writes) and the browser preview
    # aren't cosmetically different, and so re-parsing what we ourselves wrote
    # is unambiguous either way.
    # Sonar pins live in two places depending on who built the spec: the
    # web-UI form (readSpecFromForm()) only ever writes pins.sonar.trig/echo;
    # a spec parsed from an existing header (or a hand-built overlay like a
    # test fixture) may instead carry the flat sensors.sonar_trig/echo
    # parser.py also emits. A deep-merge can leave a stale -1 sitting in
    # whichever location the overlay didn't touch, so -1 ("no pin", the
    # sentinel used everywhere in this spec) is treated as absent rather
    # than a real value — whichever location holds a real pin wins;
    # pins.sonar (the current, form-shape location) is the tiebreaker.
    def _valid_pin(v):
        return v if isinstance(v, (int, float)) and v >= 0 else None
    _sonar_pins = pins.get("sonar") or {}
    sonar_trig = _valid_pin(_sonar_pins.get("trig"))
    if sonar_trig is None: sonar_trig = _valid_pin(sensors.get("sonar_trig"))
    sonar_echo = _valid_pin(_sonar_pins.get("echo"))
    if sonar_echo is None: sonar_echo = _valid_pin(sensors.get("sonar_echo"))
    if sonar_trig is None: sonar_trig = -1
    if sonar_echo is None: sonar_echo = -1
    if sonar_trig >= 0 and sonar_echo >= 0:
        lines.append("#define USE_SONAR")
        lines.append(f"#define TRIG_PIN {sonar_trig}")
        lines.append(f"#define ECHO_PIN {sonar_echo}")

    # Environmental barometer (BMP280 / BME280). BMP280 & BME280 share the
    # macro; the driver reads the chip id at runtime and only publishes
    # /humidity for a BME280.
    env = sensors.get("env") or sensors.get("env_type")
    if (env and str(env).upper() not in ("NONE", "FAKE")) or sensors.get("use_bmp280"):
        lines.append("#define USE_BMP280")
        _bmp_addr = sensors.get("bmp280_addr")
        if _bmp_addr not in (None, ""):
            _a = _bmp_addr if isinstance(_bmp_addr, str) else hex(_bmp_addr)
            lines.append(f"#define BMP280_ADDR {_a}")

    # ── Telemetry ─────────────────────────────────────────────────────
    baudrate = telemetry.get("baudrate") or spec.get("baudrate", 921600)
    lines.extend(["", "// Telemetry & micro-ROS Communication",
                  f"#define BAUDRATE {baudrate}"])

    adv = spec.get("advanced", {})

    use_wifi = telemetry.get("use_wifi", False) or ("WIFI" in str(telemetry.get("transport", "")).upper())
    if (use_wifi or telemetry.get("use_syslog", False)
            or telemetry.get("use_arduino_ota", False) or telemetry.get("use_lidar_udp", False)):
        # Secrets (WIFI_AP_LIST, AGENT_IP, SYSLOG_SERVER, LIDAR_SERVER,
        # OTA_PASSWORD) live ONLY in the git-ignored config/custom/wifi_config.h;
        # this tracked header keeps the non-secret feature flags and pulls the
        # rest in via __has_include (see generate_wifi_config()).
        lines.append('#if __has_include("wifi_config.h")')
        lines.append('  #include "wifi_config.h"')
        lines.append('#endif')
    if use_wifi:
        # The PlatformIO env pins board_microros_transport = wifi, so
        # firmware.ino compiles set_microros_net_transports(AGENT_IP, AGENT_PORT)
        # unconditionally — both macros must exist even before credentials are
        # entered. Not secrets: emit #ifndef-guarded defaults, overridden by
        # wifi_config.h once real values exist.
        agent_ip = adv.get("agent_ip") or telemetry.get("agent_ip") or "192.168.1.100"
        lines.append("#define USE_WIFI")
        lines.append("#ifndef AGENT_IP")
        lines.append(f"  #define AGENT_IP {_ipv4_c(agent_ip)}")
        lines.append("#endif")
        lines.append("#ifndef AGENT_PORT")
        lines.append(f"  #define AGENT_PORT {telemetry.get('agent_port', 8888)}")
        lines.append("#endif")
    if use_wifi and telemetry.get("use_syslog", False):
        lines.append("#define USE_SYSLOG")
        lines.append(f"#define SYSLOG_PORT {adv.get('syslog_port', 514)}")
        if adv.get("wifi_monitor"):
            lines.append(f"#define WIFI_MONITOR {adv['wifi_monitor']} // min. period to send WiFi RSSI to syslog")
    if use_wifi and telemetry.get("use_arduino_ota", False):
        lines.append("#define USE_ARDUINO_OTA")
        if adv.get("ota_hostname"):
            lines.append(f'#define OTA_HOSTNAME "{adv["ota_hostname"]}"')
    if use_wifi and telemetry.get("use_lidar_udp", False):
        lines.append("#define USE_LIDAR_UDP")
        lines.append(f"#define LIDAR_PORT {adv.get('lidar_port', 8889)}")
        if adv.get("lidar_baudrate") is not None:
            lines.append(f"#define LIDAR_BAUDRATE {adv['lidar_baudrate']}")
        if adv.get("lidar_rxd") is not None:
            lines.append(f"#define LIDAR_RXD {adv['lidar_rxd']}")
        if adv.get("lidar_serial") is not None:
            lines.append(f"#define LIDAR_SERIAL {adv['lidar_serial']}")
    # Simulation and the dual-core control task do not mix: the fake LiDAR
    # touches the simulated pose and its UART from the loop while the control
    # task drives the same pose from the other core with no lock between them,
    # and the board crash-loops. Simulation wins -- it is the point of the
    # config -- so the second core is left alone.
    sim_cfg = spec.get("simulation") or {}
    simulating = bool(sim_cfg.get("fake_wheel") or sim_cfg.get("fake_ld19"))
    if telemetry.get("use_dual_core", False) and not simulating:
        lines.append("#define USE_DUAL_CORE")
    if adv.get("use_short_brake"):
        lines.append("#define USE_SHORT_BRAKE")
    if adv.get("node_name"):
        lines.append(f'#define NODE_NAME "{adv["node_name"]}"')
    if adv.get("topic_prefix"):
        _tp = str(adv["topic_prefix"])
        if not _tp.endswith("/"):
            _tp += "/"
        lines.append(f'#define TOPIC_PREFIX "{_tp}"')
    if adv.get("device_hostname"):
        lines.append(f'#define DEVICE_HOSTNAME "{adv["device_hostname"]}"')
    if adv.get("app_name"):
        lines.append(f'#define APP_NAME "{adv["app_name"]}"')

    # ── Verbatim passthrough of anything the import carried that we did
    #    not otherwise model, so parse → generate never silently drops a
    #    macro (covariances, RCCHECK, board hooks, …).
    _blob = "\n".join(lines)
    # names the generator produced, commented forms (// #define X) included
    emitted = set(re.findall(r'^[ \t]*(?://\s*)?#define[ \t]+([A-Za-z_]\w*)', _blob, re.M))
    # structural macro families the generator always owns from the modeled spec
    _owned_re = re.compile(r'^(MOTOR\d|COUNTS_PER_REV\d?$'
                            r'|MOTOR_(MAX_RPM|OPERATING_VOLTAGE|POWER_MAX_VOLTAGE|POWER_MEASURED_VOLTAGE)$'
                            r'|MAX_RPM_RATIO$|PWM_(BITS|FREQUENCY|MAX|MIN)$|K_[PID]$|LINO_BASE$'
                            r'|WHEEL_DIAMETER$|LR_WHEELS_DISTANCE$|FR_WHEELS_DISTANCE$|ROBOT_WEIGHT$'
                            r'|LED_PIN$|BAUDRATE$|SDA_PIN$|SCL_PIN$|TRIG_PIN$|ECHO_PIN$|DAC_PIN$)')
    def _keep(nm):
        return (nm and nm not in emitted
                and not nm.startswith("USE_")   # every USE_* feature flag is a modeled decision
                and not _owned_re.match(nm))
    extras = [d for d in spec.get("raw_defines", []) if _keep(d.get("name"))]
    if extras:
        lines.append("")
        lines.append("// Preserved from the imported configuration")
        for d in extras:
            lines.append(d["text"])

    lines.extend(["", f"#endif // {name}_CONFIG_H", ""])
    return "\n".join(lines)


def _driver_macro(driver: str) -> str:
    return {
        "BTS7960":    "USE_BTS7960_MOTOR_DRIVER",
        "GENERIC_2_IN": "USE_GENERIC_2_IN_MOTOR_DRIVER",
        "GENERIC_1_IN": "USE_GENERIC_1_IN_MOTOR_DRIVER",
        "ESC":        "USE_ESC_MOTOR_DRIVER",
    }.get(driver, "USE_GENERIC_2_IN_MOTOR_DRIVER")


def generate_platformio_env(spec: Dict[str, Any], ros_distro: str = "jazzy") -> str:
    """A `[env:<robot>]` block for firmware/platformio.ini, mirroring the Web
    UI's generatePlatformioEnv(). Inherits micro-ROS distro etc. from the
    file-level [env]; ros_distro is accepted for CLI compatibility and only
    emitted when it is not the default."""
    name = spec.get("robot_name", "my_robot")
    mcu = str(spec.get("mcu", "PICO2")).upper()
    cfg_macro = f"USE_{name.upper()}_CONFIG"

    transport = str(spec.get("transport")
                    or spec.get("telemetry", {}).get("transport") or "")
    is_wifi = bool(re.search(r"WIFI|UDP", transport, re.I))
    wifi_transport_line = "board_microros_transport = wifi\n" if is_wifi else ""
    wifi_flag = "\n    -D USE_STAY_CONNECTED" if is_wifi else ""
    distro_line = ""
    if ros_distro and ros_distro not in ("jazzy", "${sysenv.ROS_DISTRO}"):
        distro_line = f"board_microros_distro = {ros_distro}\n"

    adv = spec.get("advanced", {})
    ota_ip = adv.get("ota_ip") if adv.get("ota_enable") and adv.get("ota_ip") else ""

    if mcu in ("PICO", "PICO2", "PICOW", "PICO2W"):
        board = {"PICO": "rpipico", "PICO2": "rpipico2",
                 "PICOW": "rpipicow", "PICO2W": "rpipico2w"}.get(mcu, "rpipico")
        pico_macro = f"    -D {mcu}\n" if mcu in ("PICOW", "PICO2W") else ""
        return (
            f"[env:{name}]\n"
            "platform = https://github.com/maxgerhardt/platform-raspberrypi.git\n"
            f"board = {board}\n"
            "monitor_port = /dev/ttyACM0\n"
            "upload_port = /dev/ttyACM0\n"
            "upload_protocol = picotool\n"
            "board_microros_user_meta = atomic.meta\n"
            f"{distro_line}{wifi_transport_line}lib_deps =\n"
            "    ${env.lib_deps}\n"
            "    https://github.com/gbr1/rp2040-encoder-library.git\n"
            "build_flags =\n"
            "    -I ../config\n"
            "    -D PICO\n"
            f"{pico_macro}    -D {cfg_macro}{wifi_flag}\n"
        )

    # monitor_speed must match the BAUDRATE the firmware is built with, or
    # `pio device monitor` shows garbage on a board configured for anything
    # other than 921600 (the GenDrv reference designs run at 1.5 Mbaud).
    monitor_baud = (spec.get("telemetry") or {}).get("baudrate") or spec.get("baudrate", 921600)

    if mcu in ("ESP32", "GENDRV"):
        if ota_ip:
            upl = (f"monitor_port = /dev/ttyUSB0\nupload_port = {ota_ip}\n"
                   "upload_protocol = espota")
        else:
            upl = ("monitor_port = /dev/ttyUSB0\nupload_port = /dev/ttyUSB0\n"
                   "upload_protocol = esptool")
        return (
            f"[env:{name}]\n"
            "platform = espressif32\n"
            "board = nodemcu-32s\n"
            "board_build.f_flash = 80000000L\n"
            "board_build.flash_mode = qio\n"
            "board_build.partitions = min_spiffs.csv\n"
            f"monitor_speed = {monitor_baud}\n"
            f"{upl}\n"
            f"{distro_line}{wifi_transport_line}lib_deps =\n"
            "    ${env.lib_deps}\n"
            "    madhephaestus/ESP32Servo\n"
            "    madhephaestus/ESP32Encoder\n"
            "build_flags =\n"
            "    -I ../config\n"
            "    -D __PGMSPACE_H_\n"
            f"    -D {cfg_macro}{wifi_flag}\n"
        )

    if mcu in ("ESP32S3", "ESP32S2"):
        board = "esp32-s3-devkitc-1" if mcu == "ESP32S3" else "esp32-s2-saola-1"
        is_bridge = str(spec.get("serial_interface", "CDC")) == "BRIDGE"
        port = "/dev/ttyUSB0" if is_bridge else "/dev/ttyACM0"
        cdc_flag = "" if is_bridge else "\n    -D ARDUINO_USB_CDC_ON_BOOT"
        flash_lines = ("board_build.f_flash = 80000000L\nboard_build.flash_mode = qio\n"
                       if mcu == "ESP32S3" else "")
        return (
            f"[env:{name}]\n"
            "platform = espressif32\n"
            f"board = {board}\n"
            f"{flash_lines}"
            f"monitor_speed = {monitor_baud}\n"
            f"monitor_port = {port}\n"
            f"upload_port = {port}\n"
            "upload_protocol = esptool\n"
            f"{distro_line}{wifi_transport_line}lib_deps =\n"
            "    ${env.lib_deps}\n"
            "    madhephaestus/ESP32Servo\n"
            "    madhephaestus/ESP32Encoder\n"
            "build_flags =\n"
            f"    -I ../config{cdc_flag}\n"
            "    -D __PGMSPACE_H_\n"
            f"    -D {cfg_macro}{wifi_flag}\n"
        )

    return ""


def generate_wifi_config(spec: Dict[str, Any]) -> str:
    """Contents of the git-ignored config/custom/wifi_config.h — WiFi credentials
    and host IPs only. Returns "" when the spec has no real WiFi SSID.
    Mirrors the Web UI's generateWifiConfig()."""
    wifi = spec.get("wifi_settings", {}) or {}
    adv = spec.get("advanced", {}) or {}
    tele = spec.get("telemetry", {}) or {}
    ssid = str(wifi.get("ssid", "")).strip()
    if not ssid or ssid == "YOUR_WIFI_SSID":
        return ""
    transport = str(spec.get("transport", "") or tele.get("transport", "")).upper()
    is_wifi_transport = "WIFI" in transport or "UDP" in transport
    if not (is_wifi_transport or tele.get("use_syslog") or tele.get("use_arduino_ota")
            or tele.get("use_lidar_udp") or spec.get("enable_ota_syslog")):
        return ""
    password = wifi.get("password", "") or ""
    agent_ip = adv.get("agent_ip") or wifi.get("agent_ip") or "192.168.1.100"
    out = [
        "// config/custom/wifi_config.h  --  generated by the Robot Configuration Engine",
        "// GIT-IGNORED: your WiFi credentials and host IPs are never committed.",
        "// Delete this file to let the studio regenerate it, or edit it by hand.",
        "",
        f'#define WIFI_AP_LIST {{ {{ "{ssid}", "{password}" }}, {{ NULL, NULL }} }}',
    ]
    if is_wifi_transport:
        out.append(f"#define AGENT_IP {_ipv4_c(agent_ip)}")
    out.append(f"#define SYSLOG_SERVER {_ipv4_c(adv.get('syslog_ip') or agent_ip)}")
    if tele.get("use_lidar_udp"):
        out.append(f"#define LIDAR_SERVER {_ipv4_c(adv.get('lidar_ip') or agent_ip)}")
    if adv.get("ota_password"):
        out.append(f'#define OTA_PASSWORD "{adv["ota_password"]}"')
    out.append("")
    return "\n".join(out)


def generate_urdf_xacro(spec: Dict[str, Any]) -> str:
    """URDF xacro property block, mirroring the Web UI's generateUrdfXacro()."""
    geom = spec.get("geometry", {})
    wheel_d = geom.get("wheel_diameter", 0.08)
    wheel_r = wheel_d / 2.0
    track_w = geom.get("track_width", 0.22)
    wheel_pos_y = track_w / 2.0
    return (
        '<?xml version="1.0"?>\n'
        '<robot xmlns:xacro="http://ros.org/wiki/xacro">\n'
        f'  <xacro:property name="wheel_radius" value="{wheel_r:.4f}" />\n'
        '  <xacro:property name="wheel_width" value="0.026" />\n'
        '  <xacro:property name="wheel_pos_x" value="0.0" />\n'
        f'  <xacro:property name="wheel_pos_y" value="{wheel_pos_y:.4f}" />\n'
        '  <xacro:property name="wheel_pos_z" value="-0.010" />\n'
        '  <xacro:property name="wheel_mass" value="0.05" />\n'
        f'  <xacro:property name="base_length" value="{track_w * 1.2:.3f}" />\n'
        f'  <xacro:property name="base_width" value="{track_w * 0.9:.3f}" />\n'
        '  <xacro:property name="base_height" value="0.070" />\n'
        '  <xacro:property name="base_mass" value="1.2" />\n'
        '\n'
        '  <xacro:property name="laser_pose">\n'
        '    <origin xyz="0.05 0 0.08" rpy="0 0 0"/>\n'
        '  </xacro:property>\n'
        '</robot>\n'
    )
