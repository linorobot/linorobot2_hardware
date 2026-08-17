#!/usr/bin/env python3
"""
Unit tests for Linorobot2 Hardware Rule Validator and Configuration Generator.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from validator import validate_robot_spec
from generator import generate_cpp_header, generate_platformio_env, generate_urdf_xacro


class TestConfigEngine(unittest.TestCase):
    def setUp(self):
        self.valid_pico2_spec = {
            "robot_name": "rover_pico2",
            "kinematics": "DIFFERENTIAL_DRIVE",
            "mcu": "PICO2",
            "transport": "SERIAL",
            "geometry": {
                "wheel_diameter": 0.065,
                "track_width": 0.20
            },
            "motors": {
                "driver_type": "GENERIC_2_IN",
                "max_rpm": 330,
                "cpr": 1320
            },
            "sensors": {
                "imu": "MPU6050",
                "mag": "QMC5883L",
                "battery_monitor": "ADC_DIVIDER"
            },
            "pins": {
                "led": 25,
                "motor1": { "pwm": 14, "in_a": 12, "in_b": 13 },
                "motor2": { "pwm": 15, "in_a": 10, "in_b": 11 },
                "encoders": { "m1_a": 2, "m1_b": 3, "m2_a": 4, "m2_b": 5 },
                "i2c": { "sda": 8, "scl": 9 },
                "battery_pin": 26
            }
        }

    def test_valid_spec(self):
        valid, errors, stats = validate_robot_spec(self.valid_pico2_spec)
        self.assertTrue(valid)
        self.assertEqual(len([e for e in errors if e.level == "ERROR"]), 0)
        self.assertAlmostEqual(stats["wheel_circumference_m"], 0.2042, places=3)
        self.assertAlmostEqual(stats["max_linear_speed_m_s"], 0.955, places=2)

    def test_esp32_input_only_output_error(self):
        bad_esp32 = {
            "robot_name": "bad_bot",
            "kinematics": "DIFFERENTIAL_DRIVE",
            "mcu": "ESP32",
            "transport": "SERIAL",
            "geometry": { "wheel_diameter": 0.065, "track_width": 0.20 },
            "motors": { "driver_type": "GENERIC_2_IN", "max_rpm": 330, "cpr": 1320 },
            "pins": {
                "motor1": { "pwm": 34, "in_a": 12, "in_b": 13 },  # GPIO 34 is input-only!
                "motor2": { "pwm": 15, "in_a": 10, "in_b": 11 },
                "encoders": { "m1_a": 18, "m1_b": 19, "m2_a": 21, "m2_b": 22 }
            }
        }
        valid, errors, _ = validate_robot_spec(bad_esp32)
        self.assertFalse(valid)
        error_msgs = [e.message for e in errors if e.level == "ERROR"]
        self.assertTrue(any("INPUT-ONLY" in m for m in error_msgs))

    def test_duplicate_pin_assignment(self):
        dup_pin = {
            "robot_name": "dup_bot",
            "kinematics": "DIFFERENTIAL_DRIVE",
            "mcu": "PICO2",
            "transport": "SERIAL",
            "geometry": { "wheel_diameter": 0.065, "track_width": 0.20 },
            "motors": { "driver_type": "GENERIC_2_IN", "max_rpm": 330, "cpr": 1320 },
            "pins": {
                "led": 14,
                "motor1": { "pwm": 14, "in_a": 12, "in_b": 13 },  # GP14 used for both LED and PWM
                "motor2": { "pwm": 15, "in_a": 10, "in_b": 11 },
                "encoders": { "m1_a": 2, "m1_b": 3, "m2_a": 4, "m2_b": 5 }
            }
        }
        valid, errors, _ = validate_robot_spec(dup_pin)
        self.assertFalse(valid)
        error_msgs = [e.message for e in errors if e.level == "ERROR"]
        self.assertTrue(any("assigned to multiple functions" in m for m in error_msgs))

    def test_code_generators(self):
        header = generate_cpp_header(self.valid_pico2_spec)
        self.assertIn("#define ROVER_PICO2_CONFIG_H", header)
        self.assertIn("#define WHEEL_DIAMETER 0.065", header)
        self.assertIn("#define USE_MPU6050_IMU", header)

        pio_env = generate_platformio_env(self.valid_pico2_spec)
        self.assertIn("[env:rover_pico2]", pio_env)
        self.assertIn("board = rpipico2", pio_env)

        urdf = generate_urdf_xacro(self.valid_pico2_spec)
        self.assertIn('<xacro:property name="wheel_radius" value="0.0325" />', urdf)


if __name__ == "__main__":
    unittest.main()
