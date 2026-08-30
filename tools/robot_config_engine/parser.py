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

def parse_header_to_spec(content: str) -> Dict[str, Any]:
    """
    Parses a Linorobot2 C++ configuration header file into a structured dictionary spec.
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
            "battery_divider_r1": 10000,
            "battery_divider_r2": 1000,
            "battery_dip": 0.98,
            "battery_min": 9.0,
            "battery_max": 12.6,
            "battery_cap": 2.0,
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
        "pins": {}
    }

    # Extract Robot Name from header guard or define
    guard_match = re.search(r'#ifndef\s+([A-Za-z0-9_]+)_CONFIG_H', content)
    if guard_match:
        spec["robot_name"] = guard_match.group(1).lower()

    # Kinematics
    k_match = re.search(r'#define\s+LINO_BASE\s+([A-Za-z0-9_]+)', content)
    if k_match:
        spec["kinematics"] = k_match.group(1).strip()

    # Geometry
    wd_match = re.search(r'#define\s+WHEEL_DIAMETER\s+([\d\.]+)', content)
    if wd_match:
        spec["geometry"]["wheel_diameter"] = float(wd_match.group(1))

    tw_match = re.search(r'#define\s+LR_WHEELS_DISTANCE\s+([\d\.]+)', content)
    if tw_match:
        spec["geometry"]["track_width"] = float(tw_match.group(1))

    wb_match = re.search(r'#define\s+FR_WHEELS_DISTANCE\s+([\d\.]+)', content)
    if wb_match:
        spec["geometry"]["wheelbase"] = float(wb_match.group(1))

    wt_match = re.search(r'#define\s+ROBOT_WEIGHT\s+([\d\.]+)', content)
    if wt_match:
        spec["geometry"]["weight"] = float(wt_match.group(1))

    # Motor Driver
    if re.search(r'^\s*#define\s+USE_BTS7960_MOTOR_DRIVER', content, re.MULTILINE):
        spec["motors"]["driver_type"] = "BTS7960"
    elif re.search(r'^\s*#define\s+USE_GENERIC_1_IN_MOTOR_DRIVER', content, re.MULTILINE):
        spec["motors"]["driver_type"] = "GENERIC_1_IN"
    elif re.search(r'^\s*#define\s+USE_GENERIC_2_IN_MOTOR_DRIVER', content, re.MULTILINE):
        spec["motors"]["driver_type"] = "GENERIC_2_IN"
    elif re.search(r'^\s*#define\s+USE_ESC_MOTOR_DRIVER', content, re.MULTILINE):
        spec["motors"]["driver_type"] = "ESC"

    # Motor parameters
    rpm_match = re.search(r'#define\s+MOTOR_MAX_RPM\s+(\d+)', content)
    if rpm_match:
        spec["motors"]["max_rpm"] = int(rpm_match.group(1))

    cpr_match = re.search(r'#define\s+COUNTS_PER_REV1\s+(\d+)', content)
    if cpr_match:
        spec["motors"]["cpr"] = int(cpr_match.group(1))

    pwm_bits_match = re.search(r'#define\s+PWM_BITS\s+(\d+)', content)
    if pwm_bits_match:
        spec["motors"]["pwm_bits"] = int(pwm_bits_match.group(1))

    pwm_freq_match = re.search(r'#define\s+PWM_FREQUENCY\s+(\d+)', content)
    if pwm_freq_match:
        spec["motors"]["pwm_frequency"] = int(pwm_freq_match.group(1))

    # Invert flags
    for i in range(1, 5):
        inv_match = re.search(rf'#define\s+MOTOR{i}_INV\s+(true|false)', content, re.IGNORECASE)
        if inv_match:
            spec["motors"][f"motor{i}_inv"] = (inv_match.group(1).lower() == "true")

    # IMU selection
    imu_map = [
        ("USE_FAKE_IMU", "USE_FAKE_IMU"),
        ("USE_GY85_IMU", "USE_GY85_IMU"),
        ("USE_MPU6050_IMU", "USE_MPU6050_IMU"),
        ("USE_MPU9150_IMU", "USE_MPU9150_IMU"),
        ("USE_MPU9250_IMU", "USE_MPU9250_IMU"),
        ("USE_QMI8658_IMU", "USE_QMI8658_IMU"),
        ("USE_BNO085_IMU", "USE_BNO085_IMU")
    ]
    for macro, name in imu_map:
        if re.search(rf'^\s*#define\s+{macro}', content, re.MULTILINE):
            spec["sensors"]["imu_type"] = name
            break

    # Mag selection
    mag_map = [
        ("USE_FAKE_MAG", "USE_FAKE_MAG"),
        ("USE_HMC5883L_MAG", "USE_HMC5883L_MAG"),
        ("USE_AK8963_MAG", "USE_AK8963_MAG"),
        ("USE_AK8975_MAG", "USE_AK8975_MAG"),
        ("USE_AK09918_MAG", "USE_AK09918_MAG"),
        ("USE_QMC5883L_MAG", "USE_QMC5883L_MAG")
    ]
    for macro, name in mag_map:
        if re.search(rf'^\s*#define\s+{macro}', content, re.MULTILINE):
            spec["sensors"]["mag_type"] = name
            break

    # Sonar
    trig_match = re.search(r'^\s*#define\s+TRIG_PIN\s+(\d+)', content, re.MULTILINE)
    echo_match = re.search(r'^\s*#define\s+ECHO_PIN\s+(\d+)', content, re.MULTILINE)
    if trig_match:
        spec["sensors"]["sonar_trig"] = int(trig_match.group(1))
    if echo_match:
        spec["sensors"]["sonar_echo"] = int(echo_match.group(1))

    # I2C
    sda_match = re.search(r'#define\s+SDA_PIN\s+(\d+)', content)
    scl_match = re.search(r'#define\s+SCL_PIN\s+(\d+)', content)
    if sda_match:
        spec["sensors"]["i2c_sda"] = int(sda_match.group(1))
    if scl_match:
        spec["sensors"]["i2c_scl"] = int(scl_match.group(1))

    # Battery
    bat_pin = re.search(r'#define\s+BATTERY_PIN\s+(\d+)', content)
    if bat_pin:
        spec["sensors"]["battery_pin"] = int(bat_pin.group(1))
    if re.search(r'^\s*#define\s+USE_INA219', content, re.MULTILINE):
        spec["sensors"]["use_ina219"] = True

    bat_dip = re.search(r'#define\s+BATTERY_DIP\s+([\d\.]+)', content)
    if bat_dip:
        spec["sensors"]["battery_dip"] = float(bat_dip.group(1))

    # Telemetry / Transport
    baud_match = re.search(r'#define\s+BAUDRATE\s+(\d+)', content)
    if baud_match:
        spec["telemetry"]["baudrate"] = int(baud_match.group(1))

    if re.search(r'^\s*#define\s+USE_WIFI', content, re.MULTILINE) or "WIFI_AP_LIST" in content:
        spec["telemetry"]["use_wifi"] = True
        spec["telemetry"]["transport"] = "WIFI_UDP"

    if re.search(r'^\s*#define\s+USE_SYSLOG', content, re.MULTILINE):
        spec["telemetry"]["use_syslog"] = True

    if re.search(r'^\s*#define\s+USE_ARDUINO_OTA', content, re.MULTILINE):
        spec["telemetry"]["use_arduino_ota"] = True

    if re.search(r'^\s*#define\s+USE_LIDAR_UDP', content, re.MULTILINE):
        spec["telemetry"]["use_lidar_udp"] = True

    if re.search(r'^\s*#define\s+USE_DUAL_CORE', content, re.MULTILINE):
        spec["telemetry"]["use_dual_core"] = True

    # Detect MCU family from header comments or defines
    content_upper = content.upper()
    if "ESP32_S3" in content_upper or "ESP32S3" in content_upper:
        spec["mcu"] = "ESP32S3"
    elif "ESP32_S2" in content_upper or "ESP32S2" in content_upper:
        spec["mcu"] = "ESP32S2"
    elif "GENDRV" in content_upper or "WAVESHARE" in content_upper:
        spec["mcu"] = "GENDRV"
    elif "ESP32" in content_upper:
        spec["mcu"] = "ESP32"
    elif "PICO2W" in content_upper:
        spec["mcu"] = "PICO2W"
    elif "PICO2" in content_upper or "RP2350" in content_upper:
        spec["mcu"] = "PICO2"
    elif "PICOW" in content_upper:
        spec["mcu"] = "PICOW"
    elif "PICO" in content_upper or "RP2040" in content_upper:
        spec["mcu"] = "PICO"

    return spec


def merge_configurations(base_spec: Dict[str, Any], override_spec: Dict[str, Any]) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """
    Deep-merges an override specification into an existing base specification.
    Returns (merged_spec, list_of_changes).
    """
    merged = json.loads(json.dumps(base_spec)) # Deep copy
    changes = []

    def deep_merge(target, source, prefix=""):
        for k, v in source.items():
            field_name = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict) and k in target and isinstance(target[k], dict):
                deep_merge(target[k], v, field_name)
            else:
                old_val = target.get(k)
                if old_val != v:
                    changes.append({
                        "field": field_name,
                        "old": old_val,
                        "new": v
                    })
                    target[k] = v

    deep_merge(merged, override_spec)
    return merged, changes
