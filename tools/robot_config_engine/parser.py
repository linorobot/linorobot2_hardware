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
import json
from typing import Dict, Any, Tuple, List


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ifdef_block(content: str, macro: str) -> str:
    """Return text inside the first #ifdef <macro> ... #endif block (non-nested)."""
    pattern = rf'#ifdef\s+{re.escape(macro)}\s*\n(.*?)(?:\n[ \t]*#endif\b|\Z)'
    m = re.search(pattern, content, re.DOTALL)
    return m.group(1) if m else ""


def _define_int(text: str, macro: str, default: int = -1) -> int:
    """Extract integer value of an uncommented #define macro."""
    m = re.search(rf'^[ \t]*#define\s+{re.escape(macro)}\s+(-?\d+)', text, re.MULTILINE)
    return int(m.group(1)) if m else default


def _define_bool(text: str, macro: str) -> bool:
    """Return True if an uncommented #define for macro exists."""
    return bool(re.search(rf'^[ \t]*#define\s+{re.escape(macro)}\b', text, re.MULTILINE))


def _define_float(text: str, macro: str, default: float = 0.0) -> float:
    m = re.search(rf'^[ \t]*#define\s+{re.escape(macro)}\s+([\d.]+)', text, re.MULTILINE)
    return float(m.group(1)) if m else default


def _define_truthy(text: str, macro: str) -> bool:
    """Return True if #define MACRO is 'true' (case-insensitive)."""
    m = re.search(rf'^[ \t]*#define\s+{re.escape(macro)}\s+(true|false)\b', text, re.MULTILINE | re.IGNORECASE)
    return m.group(1).lower() == "true" if m else False


def _define_num(text: str, macro: str):
    """Extract a signed int/float #define value, or None if absent."""
    m = re.search(rf'^[ \t]*#define\s+{re.escape(macro)}\s+(-?\d+(?:\.\d+)?)', text, re.MULTILINE)
    if not m:
        return None
    v = m.group(1)
    return float(v) if "." in v else int(v)


def _define_triple(text: str, macro: str):
    """Extract `#define MACRO { a, b, c }` as a list of 3 numbers, or None."""
    return _define_vec(text, macro, 3)


_COV_NUM = r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'


def _define_vec(text: str, macro: str, n: int):
    """Extract `#define MACRO { v1, ..., vn }` as a list of n numbers, or None.
    Accepts decimals and scientific notation (e.g. 1e-12)."""
    m = re.search(rf'^[ \t]*#define\s+{re.escape(macro)}\s*\{{\s*'
                  + r'\s*,\s*'.join([f'({_COV_NUM})'] * n) + r'\s*\}',
                  text, re.MULTILINE)
    if not m:
        return None
    out = []
    for x in m.groups():
        try:
            out.append(int(x))
        except ValueError:
            out.append(float(x))
    return out


# ---------------------------------------------------------------------------
# Main parser
# ---------------------------------------------------------------------------

def parse_header_to_spec(content: str) -> Dict[str, Any]:
    """
    Parses a Linorobot2 C++ configuration header file into a structured
    dictionary spec including full pin assignments.
    """
    spec: Dict[str, Any] = {
        "robot_name": "custom_robot",
        "mcu": "PICO2",
        "kinematics": "DIFFERENTIAL_DRIVE",
        "geometry": {
            "wheel_diameter": 0.065,
            "track_width": 0.20,
            "wheelbase": 0.15,
            "weight": 2.5
        },
        "motors": {
            "driver_type": "BTS7960",
            "max_rpm": 300,
            "operating_voltage": 12.0,
            "max_voltage": 12.0,
            "measured_voltage": 12.0,
            "pwm_bits": 10,
            "pwm_frequency": 20000,
            "cpr": 1320,
            "motor1_inv": False,
            "motor2_inv": True,
            "motor3_inv": False,
            "motor4_inv": True
        },
        "sensors": {
            "imu_type": "USE_FAKE_IMU",
            "mag_type": "USE_FAKE_MAG",
            "battery_pin": -1,
            # Same key names generator.py's web-UI/overlay spec shape uses
            # (battery_r1/r2, battery_*_voltage, battery_capacity) — a base
            # spec parsed from disk and an overlay from the form MUST agree
            # on field names, or merge_configurations()'s deep-merge creates
            # a second, differently-named copy instead of overwriting the
            # one that's actually read, and the overlay's value is silently
            # ignored. Defaults match generator.py's own fallbacks so an
            # untouched field behaves the same whether the spec came from a
            # freshly parsed header or a fresh form.
            "battery_r1": 30000.0,
            "battery_r2": 7500.0,
            "battery_dip": None,
            "battery_min_voltage": None,
            "battery_max_voltage": None,
            "battery_capacity": None,
            "use_ina219": False,
            "sonar_trig": -1,
            "sonar_echo": -1,
            "i2c_sda": -1,
            "i2c_scl": -1
        },
        "telemetry": {
            "baudrate": 921600,
            "transport": "SERIAL",
            "agent_port": 8888,
            "use_wifi": False,
            "use_syslog": False,
            "use_arduino_ota": False,
            "use_lidar_udp": False,
            "use_dual_core": False
        },
        "pins": {
            "led": 13,
            "encoders": {
                "m1_a": 0, "m1_b": 0,
                "m2_a": 0, "m2_b": 0,
                "m3_a": -1, "m3_b": -1,
                "m4_a": -1, "m4_b": -1,
                "m1_inv": False, "m2_inv": False,
                "m3_inv": False, "m4_inv": False
            },
            "motor1": {"pwm": -1, "in_a": -1, "in_b": -1, "pwm_r": -1, "pwm_l": -1, "en": -1, "dir": -1},
            "motor2": {"pwm": -1, "in_a": -1, "in_b": -1, "pwm_r": -1, "pwm_l": -1, "en": -1, "dir": -1},
            "motor3": {"pwm": -1, "in_a": -1, "in_b": -1, "pwm_r": -1, "pwm_l": -1, "en": -1, "dir": -1},
            "motor4": {"pwm": -1, "in_a": -1, "in_b": -1, "pwm_r": -1, "pwm_l": -1, "en": -1, "dir": -1},
            "i2c": {"sda": -1, "scl": -1},
            "battery_pin": -1,
            "sonar": {"trig": -1, "echo": -1}
        }
    }

    # ------------------------------------------------------------------
    # Robot Name from header guard
    # ------------------------------------------------------------------
    guard_match = re.search(r'#ifndef\s+([A-Za-z0-9_]+)_CONFIG_H', content)
    if guard_match:
        spec["robot_name"] = guard_match.group(1).lower()

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------
    k_match = re.search(r'^[ \t]*#define\s+LINO_BASE\s+([A-Za-z0-9_]+)', content, re.MULTILINE)
    if k_match:
        spec["kinematics"] = k_match.group(1).strip()

    # ------------------------------------------------------------------
    # Geometry
    # ------------------------------------------------------------------
    wd = re.search(r'^[ \t]*#define\s+WHEEL_DIAMETER\s+([\d.]+)', content, re.MULTILINE)
    if wd:
        spec["geometry"]["wheel_diameter"] = float(wd.group(1))
    tw = re.search(r'^[ \t]*#define\s+LR_WHEELS_DISTANCE\s+([\d.]+)', content, re.MULTILINE)
    if tw:
        spec["geometry"]["track_width"] = float(tw.group(1))
    wb = re.search(r'^[ \t]*#define\s+FR_WHEELS_DISTANCE\s+([\d.]+)', content, re.MULTILINE)
    if wb:
        spec["geometry"]["wheelbase"] = float(wb.group(1))
    wt = re.search(r'^[ \t]*#define\s+ROBOT_WEIGHT\s+([\d.]+)', content, re.MULTILINE)
    if wt:
        spec["geometry"]["weight"] = float(wt.group(1))

    # ------------------------------------------------------------------
    # Simulation Mode (Fake Wheel & Fake LD19)
    # ------------------------------------------------------------------
    if re.search(r'^[ 	]*#define\s+USE_FAKE_WHEEL', content, re.MULTILINE):
        spec.setdefault("simulation", {})["fake_wheel"] = True
        m = re.search(r'^[ 	]*#define\s+FAKE_ROBOT_MASS\s+([\d.]+)', content, re.MULTILINE)
        if m:
            spec["simulation"]["robot_mass"] = float(m.group(1))
        m = re.search(r'^[ 	]*#define\s+FAKE_WHEEL_NOISE_RPM\s+([\d.]+)', content, re.MULTILINE)
        if m:
            spec["simulation"]["wheel_noise_rpm"] = float(m.group(1))

    if re.search(r'^[ 	]*#define\s+USE_FAKE_LD19', content, re.MULTILINE):
        spec.setdefault("simulation", {})["fake_ld19"] = True
        m = re.search(r'^[ 	]*#define\s+FAKE_MAP_WIDTH\s+([\d.]+)', content, re.MULTILINE)
        if m:
            spec["simulation"]["map_width"] = float(m.group(1))
        m = re.search(r'^[ 	]*#define\s+FAKE_MAP_HEIGHT\s+([\d.]+)', content, re.MULTILINE)
        if m:
            spec["simulation"]["map_height"] = float(m.group(1))
        m = re.search(r'^[ 	]*#define\s+FAKE_WALL_OBSTACLE\s+(\d+)', content, re.MULTILINE)
        if m:
            spec["simulation"]["wall_obstacle"] = bool(int(m.group(1)))
        m = re.search(r'^[ 	]*#define\s+FAKE_WALL_X1\s+([-\d.]+)', content, re.MULTILINE)
        if m:
            spec["simulation"]["wall_x1"] = float(m.group(1))
        m = re.search(r'^[ 	]*#define\s+FAKE_WALL_Y1\s+([-\d.]+)', content, re.MULTILINE)
        if m:
            spec["simulation"]["wall_y1"] = float(m.group(1))
        m = re.search(r'^[ 	]*#define\s+FAKE_WALL_X2\s+([-\d.]+)', content, re.MULTILINE)
        if m:
            spec["simulation"]["wall_x2"] = float(m.group(1))
        m = re.search(r'^[ 	]*#define\s+FAKE_WALL_Y2\s+([-\d.]+)', content, re.MULTILINE)
        if m:
            spec["simulation"]["wall_y2"] = float(m.group(1))

    # ------------------------------------------------------------------
    # Motor Driver
    # ------------------------------------------------------------------
    if _define_bool(content, "USE_BTS7960_MOTOR_DRIVER"):
        spec["motors"]["driver_type"] = "BTS7960"
    elif _define_bool(content, "USE_GENERIC_1_IN_MOTOR_DRIVER"):
        spec["motors"]["driver_type"] = "GENERIC_1_IN"
    elif _define_bool(content, "USE_GENERIC_2_IN_MOTOR_DRIVER"):
        spec["motors"]["driver_type"] = "GENERIC_2_IN"
    elif _define_bool(content, "USE_ESC_MOTOR_DRIVER"):
        spec["motors"]["driver_type"] = "ESC"

    # ------------------------------------------------------------------
    # Motor Parameters
    # ------------------------------------------------------------------
    m = re.search(r'^[ \t]*#define\s+MOTOR_MAX_RPM\s+(\d+)', content, re.MULTILINE)
    if m:
        spec["motors"]["max_rpm"] = int(m.group(1))
    m = re.search(r'^[ \t]*#define\s+MOTOR_OPERATING_VOLTAGE\s+([\d.]+)', content, re.MULTILINE)
    if m:
        spec["motors"]["operating_voltage"] = float(m.group(1))
    m = re.search(r'^[ \t]*#define\s+MOTOR_POWER_MAX_VOLTAGE\s+([\d.]+)', content, re.MULTILINE)
    if m:
        spec["motors"]["max_voltage"] = float(m.group(1))
    m = re.search(r'^[ \t]*#define\s+MOTOR_POWER_MEASURED_VOLTAGE\s+([\d.]+)', content, re.MULTILINE)
    if m:
        spec["motors"]["measured_voltage"] = float(m.group(1))
    m = re.search(r'^[ \t]*#define\s+COUNTS_PER_REV1\s+(\d+)', content, re.MULTILINE)
    if m:
        spec["motors"]["cpr"] = int(m.group(1))
    m = re.search(r'^[ \t]*#define\s+PWM_BITS\s+(\d+)', content, re.MULTILINE)
    if m:
        spec["motors"]["pwm_bits"] = int(m.group(1))
    m = re.search(r'^[ \t]*#define\s+PWM_FREQUENCY\s+(\d+)', content, re.MULTILINE)
    if m:
        spec["motors"]["pwm_frequency"] = int(m.group(1))

    # Motor direction invert flags
    for i in range(1, 5):
        inv = re.search(rf'^[ \t]*#define\s+MOTOR{i}_INV\s+(true|false)', content, re.MULTILINE | re.IGNORECASE)
        if inv:
            spec["motors"][f"motor{i}_inv"] = (inv.group(1).lower() == "true")

    # ------------------------------------------------------------------
    # IMU / Mag selection
    # ------------------------------------------------------------------
    imu_map = [
        ("USE_FAKE_IMU", "USE_FAKE_IMU"),
        ("USE_GY85_IMU", "USE_GY85_IMU"),
        ("USE_MPU6050_IMU", "USE_MPU6050_IMU"),
        ("USE_MPU9150_IMU", "USE_MPU9150_IMU"),
        ("USE_MPU9250_IMU", "USE_MPU9250_IMU"),
        ("USE_QMI8658_IMU", "USE_QMI8658_IMU"),
        ("USE_LSM6DSOX_IMU", "USE_LSM6DSOX_IMU"),
        ("USE_ICM20948_IMU", "USE_ICM20948_IMU"),
        ("USE_BNO085_IMU", "USE_BNO085_IMU"),
    ]
    for macro, name in imu_map:
        if _define_bool(content, macro):
            spec["sensors"]["imu_type"] = name
            # Also the form-shape short name (generator.py checks this
            # first — see its comment): "USE_QMI8658_IMU" -> "QMI8658",
            # "USE_FAKE_IMU" -> "FAKE", matching the <option value=...> the
            # web-UI dropdown uses, so a later merge treats imu/imu_type as
            # the same field instead of two differently-named copies.
            spec["sensors"]["imu"] = name.replace("USE_", "").replace("_IMU", "")
            break

    mag_map = [
        ("USE_FAKE_MAG", "USE_FAKE_MAG"),
        ("USE_HMC5883L_MAG", "USE_HMC5883L_MAG"),
        ("USE_AK8963_MAG", "USE_AK8963_MAG"),
        ("USE_AK8975_MAG", "USE_AK8975_MAG"),
        ("USE_AK09918_MAG", "USE_AK09918_MAG"),
        ("USE_ICM20948_MAG", "USE_ICM20948_MAG"),
        ("USE_QMC5883L_MAG", "USE_QMC5883L_MAG"),
    ]
    for macro, name in mag_map:
        if _define_bool(content, macro):
            spec["sensors"]["mag_type"] = name
            spec["sensors"]["mag"] = name.replace("USE_", "").replace("_MAG", "")
            break

    # ------------------------------------------------------------------
    # Sensors: Sonar, I2C, Battery
    # ------------------------------------------------------------------
    trig = re.search(r'^[ \t]*#define\s+TRIG_PIN\s+(\d+)', content, re.MULTILINE)
    echo = re.search(r'^[ \t]*#define\s+ECHO_PIN\s+(\d+)', content, re.MULTILINE)
    if trig:
        spec["sensors"]["sonar_trig"] = int(trig.group(1))
    if echo:
        spec["sensors"]["sonar_echo"] = int(echo.group(1))

    sda = re.search(r'^[ \t]*#define\s+SDA_PIN\s+(\d+)', content, re.MULTILINE)
    scl = re.search(r'^[ \t]*#define\s+SCL_PIN\s+(\d+)', content, re.MULTILINE)
    if sda:
        spec["sensors"]["i2c_sda"] = int(sda.group(1))
    if scl:
        spec["sensors"]["i2c_scl"] = int(scl.group(1))

    bat = re.search(r'^[ \t]*#define\s+BATTERY_PIN\s+(\d+)', content, re.MULTILINE)
    if bat:
        spec["sensors"]["battery_pin"] = int(bat.group(1))
    dacp = re.search(r'^[ \t]*#define\s+DAC_PIN\s+(\d+)', content, re.MULTILINE)
    if dacp:
        spec["sensors"]["dac_pin"] = int(dacp.group(1))
    # adc_calibrate-derived 12-bit lookup table (not a #define — a plain const
    # array — so it needs its own round-trip; otherwise a parse->merge->
    # regenerate cycle would silently drop a previously calibrated LUT).
    lut_m = re.search(r'const\s+int16_t\s+ADC_LUT\s*\[\s*4096\s*\]\s*=\s*\{(.*?)\}\s*;', content, re.DOTALL)
    if lut_m:
        lut_vals = [int(v) for v in re.findall(r'-?\d+', lut_m.group(1))]
        if lut_vals:
            spec["sensors"]["adc_lut"] = lut_vals
    if _define_bool(content, "USE_INA219"):
        spec["sensors"]["use_ina219"] = True
        # Also set the form-shape key (see generator.py's comment on this):
        # otherwise an overlay's battery_monitor="ADC_DIVIDER"/"NONE" can't
        # actually override a parsed use_ina219=True (deep-merge only
        # overwrites a key the overlay itself sets), and the reverse — this
        # value never reaching a brand-new, un-merged spec — is the bug that
        # prompted this comment in the first place.
        spec["sensors"]["battery_monitor"] = "INA219"

    # Environmental barometer (BMP280 / BME280)
    if _define_bool(content, "USE_BMP280"):
        spec["sensors"]["use_bmp280"] = True
        spec["sensors"]["env_type"] = "BMP280"
        _ba = re.search(r'^[ \t]*#define\s+BMP280_ADDR\s+(0x[0-9A-Fa-f]+|\d+)', content, re.MULTILINE)
        if _ba:
            spec["sensors"]["bmp280_addr"] = _ba.group(1)

    for _bk, _bm in [("battery_dip", "BATTERY_DIP"), ("battery_min_voltage", "BATTERY_MIN"),
                     ("battery_max_voltage", "BATTERY_MAX"), ("battery_capacity", "BATTERY_CAP")]:
        _bmm = re.search(rf'^[ \t]*#define\s+{_bm}\s+([\d.]+)', content, re.MULTILINE)
        if _bmm:
            spec["sensors"][_bk] = float(_bmm.group(1))

    # ------------------------------------------------------------------
    # Velocity PID constants + IMU / magnetometer tuning
    # ------------------------------------------------------------------
    pid = {}
    for _k, _m in (("kp", "K_P"), ("ki", "K_I"), ("kd", "K_D")):
        _v = _define_num(content, _m)
        if _v is not None:
            pid[_k] = _v
    if pid:
        spec["pid"] = pid

    tuning: Dict[str, Any] = {}
    _mb = _define_triple(content, "MAG_BIAS")
    if _mb is not None:
        tuning["mag_bias"] = _mb
    for _k, _m in (("accel_cov", "ACCEL_COV"), ("gyro_cov", "GYRO_COV"),
                   ("ori_cov", "ORI_COV"), ("mag_cov", "MAG_COV")):
        _t = _define_vec(content, _m, 3)
        if _t is not None:
            tuning[_k] = _t[0] if _t[0] == _t[1] == _t[2] else _t
    for _k, _m in (("pose_cov", "POSE_COV"), ("twist_cov", "TWIST_COV")):
        _t = _define_vec(content, _m, 6)
        if _t is not None:
            tuning[_k] = _t[0] if len(set(_t)) == 1 else _t
    _ec = _define_vec(content, "ENV_COV", 3)
    if _ec is not None:
        tuning["env_cov"] = _ec[0] if _ec[0] == _ec[1] == _ec[2] else _ec
    if tuning:
        spec["imu_tuning"] = tuning

    # ------------------------------------------------------------------
    # Telemetry / Transport
    # ------------------------------------------------------------------
    baud = re.search(r'^[ \t]*#define\s+BAUDRATE\s+(\d+)', content, re.MULTILINE)
    if baud:
        spec["telemetry"]["baudrate"] = int(baud.group(1))

    if _define_bool(content, "USE_WIFI"):
        spec["telemetry"]["use_wifi"] = True
        spec["telemetry"]["transport"] = "WIFI_UDP"
    _wifi_on = spec["telemetry"]["use_wifi"]
    if _wifi_on and _define_bool(content, "USE_SYSLOG"):
        spec["telemetry"]["use_syslog"] = True
    if _wifi_on and _define_bool(content, "USE_ARDUINO_OTA"):
        spec["telemetry"]["use_arduino_ota"] = True
    if _define_bool(content, "USE_LIDAR_UDP"):
        spec["telemetry"]["use_lidar_udp"] = True
    if _define_bool(content, "USE_DUAL_CORE"):
        spec["telemetry"]["use_dual_core"] = True

    # Simulated drivetrain. _define_bool ignores commented-out defines, which
    # matters because lino_base_config.h ships these options commented out.
    if _define_bool(content, "USE_FAKE_WHEEL"):
        sim = spec.setdefault("simulation", {})
        sim["fake_wheel"] = True
        mass = _define_num(content, "FAKE_ROBOT_MASS")
        if mass is not None:
            sim["robot_mass"] = mass
        noise = _define_num(content, "FAKE_WHEEL_NOISE_RPM")
        if noise is not None:
            sim["wheel_noise_rpm"] = noise

    # ------------------------------------------------------------------
    # MCU Family detection — the header-guard name is authoritative
    # (a mere comment mentioning "ESP32" must not flip a pico_config.h).
    # ------------------------------------------------------------------
    guard = (guard_match.group(1).upper() if guard_match else "")
    cu = content.upper()

    def _mcu_from(text):
        if "ESP32_S3" in text or "ESP32S3" in text:
            return "ESP32S3"
        if "ESP32_S2" in text or "ESP32S2" in text:
            return "ESP32S2"
        if "GENDRV" in text or "WAVESHARE" in text:
            return "GENDRV"
        if "ESP32" in text:
            return "ESP32"
        if "PICO2W" in text:
            return "PICO2W"
        if "PICO2" in text or "RP2350" in text:
            return "PICO2"
        if "PICOW" in text:
            return "PICOW"
        if "PICO" in text or "RP2040" in text:
            return "PICO"
        return None

    spec["mcu"] = _mcu_from(guard) or _mcu_from(cu) or spec["mcu"]

    # ------------------------------------------------------------------
    # Pins: LED
    # ------------------------------------------------------------------
    led = re.search(r'^[ \t]*#define\s+LED_PIN\s+(\S+)', content, re.MULTILINE)
    if led:
        raw = led.group(1).strip()
        # Integer pin number (allowing -1 = "no connection / no addressable
        # LED" — see firmware ledInit/ledWrite helpers). Anything else is
        # treated as a symbolic alias (e.g. LED_BUILTIN).
        if re.fullmatch(r'-?\d+', raw):
            spec["pins"]["led"] = int(raw)
        else:
            spec["pins"]["led"] = raw  # e.g. LED_BUILTIN

    # ------------------------------------------------------------------
    # Pins: Encoder A/B (these are outside any #ifdef block)
    # ------------------------------------------------------------------
    enc = spec["pins"]["encoders"]
    for i, key_a, key_b in [
        (1, "m1_a", "m1_b"),
        (2, "m2_a", "m2_b"),
        (3, "m3_a", "m3_b"),
        (4, "m4_a", "m4_b"),
    ]:
        a = re.search(rf'^[ \t]*#define\s+MOTOR{i}_ENCODER_A\s+(-?\d+)', content, re.MULTILINE)
        b = re.search(rf'^[ \t]*#define\s+MOTOR{i}_ENCODER_B\s+(-?\d+)', content, re.MULTILINE)
        if a:
            enc[key_a] = int(a.group(1))
        if b:
            enc[key_b] = int(b.group(1))

    # ------------------------------------------------------------------
    # Pins: Encoder invert flags
    # ------------------------------------------------------------------
    for i, key in [(1, "m1_inv"), (2, "m2_inv"), (3, "m3_inv"), (4, "m4_inv")]:
        inv = re.search(
            rf'^[ \t]*#define\s+MOTOR{i}_ENCODER_INV\s+(true|false)',
            content, re.MULTILINE | re.IGNORECASE
        )
        if inv:
            enc[key] = (inv.group(1).lower() == "true")

    # ------------------------------------------------------------------
    # Pins: Motor driver pins — parse the active #ifdef block
    # ------------------------------------------------------------------
    driver = spec["motors"]["driver_type"]

    driver_macro_map = {
        "BTS7960":    "USE_BTS7960_MOTOR_DRIVER",
        "GENERIC_2_IN": "USE_GENERIC_2_IN_MOTOR_DRIVER",
        "GENERIC_1_IN": "USE_GENERIC_1_IN_MOTOR_DRIVER",
        "ESC":        "USE_ESC_MOTOR_DRIVER",
    }
    block_macro = driver_macro_map.get(driver, "USE_GENERIC_2_IN_MOTOR_DRIVER")
    drv_block = _ifdef_block(content, block_macro)

    # If the block is empty (possible when the main file uses a flat structure
    # without #ifdef guards), fall back to the whole content
    search_in = drv_block if drv_block.strip() else content

    for i in range(1, 5):
        pwm  = _define_int(search_in, f"MOTOR{i}_PWM")
        in_a = _define_int(search_in, f"MOTOR{i}_IN_A")
        in_b = _define_int(search_in, f"MOTOR{i}_IN_B")
        mk   = f"motor{i}"

        if driver == "BTS7960":
            # BTS7960 uses two outputs only: MOTORx_IN_A = RPWM, MOTORx_IN_B = LPWM.
            # MOTORx_PWM is an unused placeholder in the header / driver class.
            spec["pins"][mk] = {
                "in_a": in_a,
                "in_b": in_b,
                "pwm":  pwm,
                # legacy keys kept for older importers
                "pwm_l": in_a,
                "en":    in_b,
                "pwm_r": -1,
                "dir":   -1,
            }
        elif driver == "GENERIC_1_IN":
            spec["pins"][mk] = {
                "pwm":   pwm,
                "dir":   in_a,
                "in_a":  in_a,
                "in_b":  -1,
                "pwm_r": -1, "pwm_l": -1, "en": -1,
            }
        elif driver == "ESC":
            spec["pins"][mk] = {
                "pwm":   pwm,
                "in_a":  -1, "in_b": -1, "dir": -1,
                "pwm_r": -1, "pwm_l": -1, "en":  -1,
            }
        else:  # GENERIC_2_IN (default)
            spec["pins"][mk] = {
                "pwm":   pwm,
                "in_a":  in_a,
                "in_b":  in_b,
                "dir":   -1,
                "pwm_r": -1, "pwm_l": -1, "en": -1,
            }

    # ------------------------------------------------------------------
    # Pins: I2C, Battery, Sonar — mirror into pins sub-section
    # ------------------------------------------------------------------
    spec["pins"]["i2c"]["sda"] = spec["sensors"]["i2c_sda"]
    spec["pins"]["i2c"]["scl"] = spec["sensors"]["i2c_scl"]
    spec["pins"]["battery_pin"] = spec["sensors"]["battery_pin"]
    if "dac_pin" in spec["sensors"]:
        spec["pins"]["dac_pin"] = spec["sensors"]["dac_pin"]
    spec["pins"]["sonar"]["trig"] = spec["sensors"]["sonar_trig"]
    spec["pins"]["sonar"]["echo"] = spec["sensors"]["sonar_echo"]

    # ------------------------------------------------------------------
    # Modeled advanced telemetry values (editable in the Web UI, so they
    # must round-trip through the spec, not the verbatim passthrough).
    # ------------------------------------------------------------------
    adv = spec.setdefault("advanced", {})

    def _ipv4(macro: str):
        mm = re.search(rf'^[ \t]*#define\s+{macro}\s*\{{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}}',
                       content, re.MULTILINE)
        return ".".join(mm.groups()) if mm else None
    def _num(macro: str):
        mm = re.search(rf'^[ \t]*#define\s+{macro}\s+(\d+)', content, re.MULTILINE)
        return int(mm.group(1)) if mm else None
    def _str(macro: str):
        mm = re.search(rf'^[ \t]*#define\s+{macro}\s+"([^"]*)"', content, re.MULTILINE)
        return mm.group(1) if mm else None

    for key, fn, macro in [
        ("syslog_ip",     _ipv4, "SYSLOG_SERVER"),
        ("syslog_port",   _num,  "SYSLOG_PORT"),
        ("wifi_monitor",  _num,  "WIFI_MONITOR"),
        ("lidar_ip",      _ipv4, "LIDAR_SERVER"),
        ("lidar_port",    _num,  "LIDAR_PORT"),
        ("lidar_baudrate", _num, "LIDAR_BAUDRATE"),
        ("lidar_rxd",     _num,  "LIDAR_RXD"),
        ("lidar_serial",  _num,  "LIDAR_SERIAL"),
        ("agent_ip",      _ipv4, "AGENT_IP"),
        ("ota_hostname",  _str,  "OTA_HOSTNAME"),
        ("ota_password",  _str,  "OTA_PASSWORD"),
        ("topic_prefix",  _str,  "TOPIC_PREFIX"),
    ]:
        v = fn(macro)
        if v is not None:
            adv[key] = v
    if _define_bool(content, "USE_SHORT_BRAKE"):
        adv["use_short_brake"] = True

    # ------------------------------------------------------------------
    # Verbatim passthrough of every other uncommented #define, so an
    # imported header round-trips without silently dropping anything
    # (covariances, RCCHECK, board hooks, LIDAR_* the UI does not model…).
    # The generator re-emits these only if it did not already produce a
    # #define of the same name.
    # ------------------------------------------------------------------
    # A #define is "unconditional" if every preprocessor guard enclosing it is
    # benign — the header guard, an `#if __has_include(...)` wifi_config probe,
    # or `#ifndef BAUDRATE`. Anything guarded by a feature macro we model
    # (USE_WIFI, USE_SYSLOG, USE_*_MOTOR_DRIVER, USE_LIDAR_UDP, …) is logic we
    # regenerate ourselves and must NOT hoist to the top level.
    guard_name = f"{spec['robot_name'].upper()}_CONFIG_H"
    benign = {guard_name, "BAUDRATE"}
    # Macros now captured into the structured spec above (editable in the UI);
    # the generator re-emits them from the spec, so keep them out of the
    # verbatim passthrough or they would double-emit / ignore a UI clear.
    modeled = {"K_P", "K_I", "K_D", "MAG_BIAS", "ACCEL_COV", "GYRO_COV",
               "ORI_COV", "MAG_COV", "POSE_COV", "TWIST_COV", "ENV_COV",
               "TOPIC_PREFIX", "USE_BMP280", "BMP280_ADDR", "DAC_PIN",
               "USE_ADC_LUT", "ADC_LUT", "USE_SONAR"}
    src_lines = content.split("\n")
    raw_defines: List[Dict[str, str]] = []
    stack: List[str] = []
    i = 0
    while i < len(src_lines):
        ln = src_lines[i]
        bare = ln.strip()
        mif = re.match(r'#if(n?def)?\b(.*)', bare)
        if mif:
            rest = mif.group(2)
            if "__has_include" in rest:
                stack.append("__has_include")
            else:
                tok = re.search(r'([A-Za-z_]\w*)', rest)
                stack.append(tok.group(1) if tok else "?")
        elif bare.startswith("#endif"):
            if stack:
                stack.pop()
        elif re.match(r'^[ \t]*#define[ \t]', ln) and all(
                g in benign or g == "__has_include" for g in stack):
            nm = re.match(r'^[ \t]*#define[ \t]+([A-Za-z_]\w*)', ln)
            block = [ln]
            while block[-1].rstrip().endswith("\\") and i + 1 < len(src_lines):
                i += 1
                block.append(src_lines[i])
            if nm and nm.group(1) not in modeled:
                raw_defines.append({"name": nm.group(1), "text": "\n".join(block).rstrip()})
        i += 1
    spec["raw_defines"] = raw_defines

    return spec


def merge_wifi_config(spec: Dict[str, Any], wifi_content: str) -> Dict[str, Any]:
    """Fold the git-ignored config/custom/wifi_config.h into a parsed spec so
    the Web UI form shows the real SSID / password / host IPs. Mutates & returns
    `spec`. Safe to call with an empty string (no-op)."""
    if not wifi_content:
        return spec
    ap = re.search(r'#define\s+WIFI_AP_LIST\s*\{\s*\{\s*"([^"]*)"\s*,\s*"([^"]*)"',
                   wifi_content)
    ssid_m = re.search(r'#define\s+WIFI_SSID\s+"([^"]*)"', wifi_content)
    pw_m = re.search(r'#define\s+WIFI_PASSWORD\s+"([^"]*)"', wifi_content)
    ssid = ap.group(1) if ap else (ssid_m.group(1) if ssid_m else "")
    password = ap.group(2) if ap else (pw_m.group(1) if pw_m else "")
    if not ssid:
        return spec

    def _ip(macro):
        mm = re.search(rf'#define\s+{macro}\s*\{{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}}',
                       wifi_content)
        return ".".join(mm.groups()) if mm else None

    ws = spec.setdefault("wifi_settings", {})
    ws["ssid"] = ssid
    ws["password"] = password
    adv = spec.setdefault("advanced", {})
    agent_ip = _ip("AGENT_IP")
    if agent_ip:
        adv["agent_ip"] = agent_ip
        ws["agent_ip"] = agent_ip
    syslog_ip = _ip("SYSLOG_SERVER")
    if syslog_ip:
        adv["syslog_ip"] = syslog_ip
    lidar_ip = _ip("LIDAR_SERVER")
    if lidar_ip:
        adv["lidar_ip"] = lidar_ip
    ota_pw = re.search(r'#define\s+OTA_PASSWORD\s+"([^"]*)"', wifi_content)
    if ota_pw:
        adv["ota_password"] = ota_pw.group(1)
    # Present a real SSID -> the studio should show the WiFi block.
    spec["enable_ota_syslog"] = True
    return spec


def merge_configurations(base_spec: Dict[str, Any], override_spec: Dict[str, Any]) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """
    Deep-merges an override specification into an existing base specification.
    Returns (merged_spec, list_of_changes).
    """
    merged = json.loads(json.dumps(base_spec))  # Deep copy
    changes = []

    def deep_merge(target, source, prefix=""):
        for k, v in source.items():
            field_name = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict) and k in target and isinstance(target[k], dict):
                deep_merge(target[k], v, field_name)
            else:
                old_val = target.get(k)
                if old_val != v:
                    changes.append({"field": field_name, "old": old_val, "new": v})
                    target[k] = v

    deep_merge(merged, override_spec)
    _sync_sensor_alias(merged, changes, "imu", "imu_type", "_IMU")
    _sync_sensor_alias(merged, changes, "mag", "mag_type", "_MAG")
    return merged, changes


def _sync_sensor_alias(merged: Dict[str, Any], changes: List[Dict[str, Any]], short_key: str, full_key: str, suffix: str) -> None:
    """Keep sensors.<short_key> (form-shape, e.g. "imu") and
    sensors.<full_key> (parser-shape, e.g. "imu_type") in agreement after a
    merge. generator.py checks the short key first (see its comments there),
    so an overlay that only touches one of the pair must still make the
    *other* one agree, or a base spec's stale copy of whichever key the
    overlay didn't touch would win (or lose) unpredictably depending on
    which key happened to be set in that particular caller -- this is the
    same key-mismatch bug class as the battery/sonar/INA219/i2c fixes,
    showing up here because two different, equally legitimate callers use
    the two different key names (the real web-UI form always sends the
    short key; a hand-built/legacy overlay -- e.g. a merge test fixture, or
    an older API caller -- may send only the full key).
    """
    sensors = merged.get("sensors")
    if not isinstance(sensors, dict):
        return
    touched_short = any(c["field"] == f"sensors.{short_key}" for c in changes)
    touched_full = any(c["field"] == f"sensors.{full_key}" for c in changes)
    if touched_full and not touched_short:
        v = sensors.get(full_key)
        if v:
            sensors[short_key] = str(v).replace("USE_", "").replace(suffix, "")
    elif touched_short and not touched_full:
        v = sensors.get(short_key)
        if v:
            sensors[full_key] = v if str(v).startswith("USE_") else f"USE_{v}{suffix}"
