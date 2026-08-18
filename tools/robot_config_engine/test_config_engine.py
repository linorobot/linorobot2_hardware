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
from validator import validate_robot_spec
from generator import generate_config_header, generate_platformio_env, generate_urdf_xacro

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

    def test_lyrical_dependency_resolution(self):
        pio_jazzy = generate_platformio_env(self.valid_pico_spec, ros_distro="jazzy")
        self.assertIn("${env.lib_deps}", pio_jazzy)

        pio_lyrical = generate_platformio_env(self.valid_pico_spec, ros_distro="lyrical")
        self.assertIn("https://github.com/hippo5329/micro_ros_platformio.git#feat/ros2-lyrical-support", pio_lyrical)

    def test_header_generation(self):
        header = generate_config_header(self.valid_pico_spec)
        self.assertIn("#define LINO_BASE DIFFERENTIAL_DRIVE", header)
        self.assertIn("#define USE_GENERIC_2_IN_MOTOR_DRIVER", header)
        self.assertIn("#define USE_MPU6050_IMU", header)
        self.assertIn("#define BATTERY_CAP 2.20", header)
        self.assertIn("#define BATTERY_MIN 9.00", header)
        self.assertIn("#define BATTERY_MAX 12.60", header)
        self.assertIn("#define ROBOT_WEIGHT 3.50", header)

    def test_pico2w_generation(self):
        pico2w_spec = dict(self.valid_pico_spec)
        pico2w_spec["mcu"] = "PICO2W"
        pico2w_spec["robot_name"] = "scout_pico2w"
        valid, errors, stats = validate_robot_spec(pico2w_spec)
        self.assertTrue(valid)
        pio = generate_platformio_env(pico2w_spec)
        self.assertIn("board = rpipico2w", pio)
        self.assertIn("-D PICO2W", pio)

if __name__ == "__main__":
    unittest.main()
