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

import unittest
import os
from validator import validate_robot_spec
from generator import generate_config_header
from parser import parse_header_to_spec, merge_configurations

class TestRobotConfigEngine(unittest.TestCase):
    def setUp(self):
        self.valid_pico_spec = {
            "robot_name": "scout_pico2",
            "kinematics": "DIFFERENTIAL_DRIVE",
            "mcu": "PICO2",
            "transport": "SERIAL",
            "geometry": {
                "wheel_diameter": 0.065,
                "track_width": 0.20,
                "weight": 3.5
            },
            "motors": {
                "driver_type": "GENERIC_2_IN",
                "max_rpm": 330,
                "cpr": 1320,
                "rated_torque": 1.5,
                "rated_voltage": 12.0
            },
            "sensors": {
                "imu": "MPU6050",
                "mag": "NONE",
                "battery_monitor": "ADC_DIVIDER",
                "battery_capacity": 2.2,
                "battery_nominal_voltage": 11.1,
                "battery_min_voltage": 9.0,
                "battery_max_voltage": 12.6,
                "sonar": True
            },
            "pins": {
                "led": 25,
                "motor1": { "pwm": 14, "in_a": 12, "in_b": 13 },
                "motor2": { "pwm": 15, "in_a": 10, "in_b": 11 },
                "encoders": { "m1_a": 2, "m1_b": 3, "m2_a": 4, "m2_b": 5 },
                "i2c": { "sda": 8, "scl": 9 },
                "battery_pin": 26,
                "sonar": { "trig": 16, "echo": 17 }
            }
        }

    def test_validation_success(self):
        valid, errors, stats = validate_robot_spec(self.valid_pico_spec)
        self.assertTrue(valid)
        self.assertEqual(len(errors), 0)
        self.assertAlmostEqual(stats["max_linear_speed_m_s"], 0.954, places=2)
        self.assertIn("max_accel_m_s2", stats)

    def test_pin_conflict_detection(self):
        bad_spec = dict(self.valid_pico_spec)
        bad_spec["pins"] = dict(self.valid_pico_spec["pins"])
        bad_spec["pins"]["motor1"] = { "pwm": 2, "in_a": 12, "in_b": 13 }
        valid, errors, stats = validate_robot_spec(bad_spec)
        self.assertFalse(valid)
        self.assertTrue(any("assigned to multiple functions" in str(e) for e in errors))

    def test_header_generation(self):
        header = generate_config_header(self.valid_pico_spec)
        self.assertIn("#define LINO_BASE DIFFERENTIAL_DRIVE", header)
        self.assertIn("#define USE_GENERIC_2_IN_MOTOR_DRIVER", header)
        self.assertIn("#define USE_MPU6050_IMU", header)
        self.assertIn("#define BATTERY_CAP 2.20", header)
        self.assertIn("#define BATTERY_MIN 9.00", header)
        self.assertIn("#define BATTERY_MAX 12.60", header)
        self.assertIn("#define ROBOT_WEIGHT 3.50", header)

    def test_header_parsing_fake_sensors(self):
        # Sample C++ header with Fake IMU and Fake Mag
        raw_header = """
#ifndef TESTBOT_CONFIG_H
#define TESTBOT_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define WHEEL_DIAMETER 0.0650
#define LR_WHEELS_DISTANCE 0.2000
#define USE_BTS7960_MOTOR_DRIVER
#define MOTOR_MAX_RPM 300
#define COUNTS_PER_REV1 1320
#define USE_FAKE_IMU
#define USE_FAKE_MAG
#define BAUDRATE 921600
#endif
"""
        parsed = parse_header_to_spec(raw_header)
        self.assertEqual(parsed["robot_name"], "testbot")
        self.assertEqual(parsed["kinematics"], "DIFFERENTIAL_DRIVE")
        self.assertEqual(parsed["motors"]["driver_type"], "BTS7960")
        self.assertEqual(parsed["sensors"]["imu_type"], "USE_FAKE_IMU")
        self.assertEqual(parsed["sensors"]["mag_type"], "USE_FAKE_MAG")
        self.assertEqual(parsed["telemetry"]["baudrate"], 921600)

    def test_user_modify_and_merge_workflow(self):
        # 1. User starts with an existing header (Fake IMU/Mag)
        raw_header = """
#ifndef MYROBOT_CONFIG_H
#define MYROBOT_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define WHEEL_DIAMETER 0.0800
#define LR_WHEELS_DISTANCE 0.2200
#define USE_GENERIC_2_IN_MOTOR_DRIVER
#define MOTOR_MAX_RPM 330
#define COUNTS_PER_REV1 1440
#define USE_FAKE_IMU
#define USE_FAKE_MAG
#define BAUDRATE 921600
#endif
"""
        base_spec = parse_header_to_spec(raw_header)
        self.assertEqual(base_spec["sensors"]["imu_type"], "USE_FAKE_IMU")

        # 2. User modifies settings in Web UI: adds QMI8658 IMU, AK09918 Mag, Sonar, and Dual-Core
        modified_settings = {
            "sensors": {
                "imu_type": "USE_QMI8658_IMU",
                "mag_type": "USE_AK09918_MAG",
                "sonar_trig": 19,
                "sonar_echo": 18
            },
            "telemetry": {
                "use_dual_core": True,
                "baudrate": 921600
            }
        }

        merged_spec, changes = merge_configurations(base_spec, modified_settings)
        self.assertGreater(len(changes), 0)

        # 3. Generate merged C++ header
        updated_header = generate_config_header(merged_spec)
        self.assertIn("#define USE_QMI8658_IMU", updated_header)
        self.assertIn("#define USE_AK09918_MAG", updated_header)
        self.assertIn("#define TRIG_PIN 19", updated_header)
        self.assertIn("#define ECHO_PIN 18", updated_header)
        self.assertIn("#define USE_DUAL_CORE", updated_header)
        self.assertNotIn("#define USE_FAKE_IMU", updated_header)
        self.assertNotIn("#define USE_FAKE_MAG", updated_header)

if __name__ == "__main__":
    unittest.main()
