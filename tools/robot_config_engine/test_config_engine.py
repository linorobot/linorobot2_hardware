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

import json
import unittest
import os
from validator import validate_robot_spec
from generator import generate_config_header
from parser import parse_header_to_spec, merge_configurations
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "web"))
import server

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

    def _esp32_diff_spec(self):
        """A minimal, otherwise-valid ESP32 differential spec for pin-rule tests."""
        return {
            "robot_name": "esp32_bot",
            "kinematics": "DIFFERENTIAL_DRIVE",
            "mcu": "ESP32",
            "transport": "SERIAL",
            "geometry": {"wheel_diameter": 0.065, "track_width": 0.20, "weight": 3.5},
            "motors": {"driver_type": "GENERIC_2_IN", "max_rpm": 330, "cpr": 1320,
                       "rated_torque": 1.5, "rated_voltage": 12.0},
            "sensors": {"imu": "MPU6050", "mag": "NONE", "battery_monitor": "NONE"},
            "pins": {
                "led": 13,
                "motor1": {"pwm": 25, "in_a": 26, "in_b": 27},
                "motor2": {"pwm": 32, "in_a": 33, "in_b": 4},
                "encoders": {"m1_a": 16, "m1_b": 17, "m2_a": 18, "m2_b": 19},
                "i2c": {"sda": 21, "scl": 22},
            },
        }

    def test_esp32_strapping_pin_as_motor_output_warns(self):
        spec = self._esp32_diff_spec()
        spec["pins"]["motor1"]["pwm"] = 12  # MTDI strapping pin, driven as PWM output
        ok, errors, _ = validate_robot_spec(spec)
        self.assertFalse(ok)  # GPIO 12 as output is an ERROR
        self.assertTrue(any("GPIO 12" in str(e) and "boot" in str(e).lower() for e in errors))

    def test_esp32_strapping_pin_15_output_is_warning_not_error(self):
        spec = self._esp32_diff_spec()
        spec["pins"]["led"] = 15  # strapping pin, but not the flash-voltage one
        ok, errors, _ = validate_robot_spec(spec)
        self.assertTrue(ok)
        self.assertTrue(any("GPIO 15" in str(e) and e.level == "WARNING" for e in errors))

    def test_esp32_input_only_encoder_pullup_warning(self):
        spec = self._esp32_diff_spec()
        spec["pins"]["encoders"]["m1_a"] = 34  # input-only, no internal pull-up
        ok, errors, _ = validate_robot_spec(spec)
        self.assertTrue(ok)
        self.assertTrue(any("pull-up" in str(e) for e in errors))

    def test_esp32_adc2_battery_pin_with_wifi_errors(self):
        spec = self._esp32_diff_spec()
        spec["transport"] = "WIFI_UDP"
        spec["sensors"]["battery_monitor"] = "ADC_DIVIDER"
        spec["pins"]["battery_pin"] = 4  # ADC2 -> unusable with WiFi
        ok, errors, _ = validate_robot_spec(spec)
        self.assertFalse(ok)
        self.assertTrue(any("ADC2" in str(e) for e in errors))
        # Same pin is fine on serial transport.
        spec["transport"] = "SERIAL"
        ok2, errors2, _ = validate_robot_spec(spec)
        self.assertFalse(any("ADC2" in str(e) for e in errors2))

    def test_header_generation(self):
        header = generate_config_header(self.valid_pico_spec)
        self.assertIn("#define LINO_BASE DIFFERENTIAL_DRIVE", header)
        self.assertIn("#define USE_GENERIC_2_IN_MOTOR_DRIVER", header)
        self.assertIn("#define USE_MPU6050_IMU", header)
        self.assertIn("#define BATTERY_CAP 2.2", header)
        self.assertIn("#define BATTERY_MIN 9", header)
        self.assertIn("#define BATTERY_MAX 12.6", header)
        self.assertIn("#define ROBOT_WEIGHT 3.5", header)
        # Sonar enabled in the fixture (sensors.sonar=True, pins.sonar trig/echo)
        self.assertIn("#define USE_SONAR", header)
        self.assertIn("#define TRIG_PIN 16", header)
        self.assertIn("#define ECHO_PIN 17", header)

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

    def test_negative_led_pin_roundtrip(self):
        # LED_PIN -1 means "no addressable LED" (e.g. Waveshare GenDrv).
        # It must parse to int -1 and regenerate as "#define LED_PIN -1".
        parsed = parse_header_to_spec("#define LED_PIN -1\n")
        self.assertEqual(parsed["pins"]["led"], -1)
        header = generate_config_header(parsed)
        self.assertIn("#define LED_PIN -1", header)

    def test_bts7960_two_output_roundtrip(self):
        # BTS7960 drives exactly two pins: MOTORx_IN_A (RPWM) and MOTORx_IN_B
        # (LPWM). MOTORx_PWM is an unused placeholder and must be emitted as -1
        # with no BOARD_INIT enable-drive lines.
        spec = json.loads(json.dumps(self.valid_pico_spec))
        spec["motors"]["driver_type"] = "BTS7960"
        spec["pins"]["motor1"] = {"in_a": 12, "in_b": 13}
        spec["pins"]["motor2"] = {"in_a": 10, "in_b": 11}
        header = generate_config_header(spec)
        self.assertIn("#define MOTOR1_PWM -1", header)
        self.assertIn("#define MOTOR1_IN_A 12", header)
        self.assertIn("#define MOTOR1_IN_B 13", header)
        self.assertNotIn("pinMode(MOTOR1_PWM", header)

        parsed = parse_header_to_spec(header)
        self.assertEqual(parsed["motors"]["driver_type"], "BTS7960")
        self.assertEqual(parsed["pins"]["motor1"]["in_a"], 12)
        self.assertEqual(parsed["pins"]["motor1"]["in_b"], 13)

        ok, errors, _ = validate_robot_spec(spec)
        self.assertTrue(ok, errors)

        # A duplicate across the two real outputs is still a conflict.
        spec["pins"]["motor2"]["in_a"] = 12
        ok, errors, _ = validate_robot_spec(spec)
        self.assertFalse(ok)

    def test_negative_encoder_pins_valid(self):
        # Negative values stand for "no connection" and must not trigger
        # pin-conflict or out-of-range errors, even when repeated.
        neg = json.loads(json.dumps(self.valid_pico_spec))
        neg["pins"] = {
            "led": 25,
            "encoders": {
                "m1_a": -1, "m1_b": -1, "m2_a": -1, "m2_b": -1,
                "m3_a": -1, "m3_b": -1, "m4_a": -1, "m4_b": -1,
                "m1_inv": False, "m2_inv": False, "m3_inv": False, "m4_inv": False,
            },
            "motor1": {"pwm": 14, "in_a": 12, "in_b": 13},
            "motor2": {"pwm": 15, "in_a": 10, "in_b": 11},
            "motor3": {"pwm": -1, "in_a": -1, "in_b": -1},
            "motor4": {"pwm": -1, "in_a": -1, "in_b": -1},
            "i2c": {"sda": 4, "scl": 5},
        }
        ok, errors, _ = validate_robot_spec(neg)
        self.assertTrue(ok)
        self.assertEqual(errors, [])

        # But a genuine positive-pin duplicate must still be an error.
        neg["pins"]["motor2"]["in_a"] = 14  # duplicates motor1.in_a == 14
        ok, errors, _ = validate_robot_spec(neg)
        self.assertFalse(ok)
        self.assertTrue(any("assigned to multiple functions" in str(e) for e in errors))

    def test_bmp280_env_roundtrip(self):
        raw = """
#ifndef ENVBOT_CONFIG_H
#define ENVBOT_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_BTS7960_MOTOR_DRIVER
#define USE_BMP280
#define BMP280_ADDR 0x76
#define ENV_COV { 1.0, 0.01, 0.0025 }
#define BAUDRATE 921600
#endif
"""
        spec = parse_header_to_spec(raw)
        self.assertTrue(spec["sensors"].get("use_bmp280"))
        self.assertEqual(spec["sensors"].get("env_type"), "BMP280")
        self.assertEqual(spec["sensors"].get("bmp280_addr"), "0x76")
        self.assertEqual(spec["imu_tuning"]["env_cov"], [1.0, 0.01, 0.0025])

        h = generate_config_header(spec)
        self.assertEqual(h.count("#define USE_BMP280"), 1)
        self.assertIn("#define BMP280_ADDR 0x76", h)
        self.assertIn("#define ENV_COV { 1, 0.01, 0.0025 }", h)  # _n() trims 1.0 -> 1
        self.assertEqual(h.count("#define ENV_COV"), 1)

        # From the front-end form shape (sensors.env = "BME280")
        fe = json.loads(json.dumps(self.valid_pico_spec))
        fe["sensors"]["env"] = "BME280"
        h2 = generate_config_header(fe)
        self.assertIn("#define USE_BMP280", h2)

    def test_modern_imu_roundtrip(self):
        for imu, macro in (("LSM6DSOX", "USE_LSM6DSOX_IMU"),
                           ("ICM20948", "USE_ICM20948_IMU")):
            fe = json.loads(json.dumps(self.valid_pico_spec))
            fe["sensors"]["imu"] = imu
            h = generate_config_header(fe)
            self.assertIn(f"#define {macro}", h)

        # Enabling a known IMU with no explicit covariance -> datasheet default
        fe = json.loads(json.dumps(self.valid_pico_spec))
        fe["sensors"]["imu"] = "LSM6DSOX"
        fe.pop("imu_tuning", None)
        h = generate_config_header(fe)
        self.assertIn("#define ACCEL_COV { 4.7e-05, 4.7e-05, 4.7e-05 }", h)
        self.assertIn("#define GYRO_COV { 4.4e-07, 4.4e-07, 4.4e-07 }", h)
        # An explicit value still wins over the default.
        fe["imu_tuning"] = {"accel_cov": 0.02}
        h = generate_config_header(fe)
        self.assertIn("#define ACCEL_COV { 0.02, 0.02, 0.02 }", h)
        self.assertEqual(h.count("#define ACCEL_COV"), 1)
        raw = ("#ifndef ICMBOT_CONFIG_H\n#define ICMBOT_CONFIG_H\n"
               "#define USE_ICM20948_IMU\n#define USE_ICM20948_MAG\n#endif\n")
        spec = parse_header_to_spec(raw)
        self.assertEqual(spec["sensors"]["imu_type"], "USE_ICM20948_IMU")
        self.assertEqual(spec["sensors"]["mag_type"], "USE_ICM20948_MAG")
        h = generate_config_header(spec)
        self.assertEqual(h.count("#define USE_ICM20948_IMU"), 1)
        self.assertIn("#define USE_ICM20948_MAG", h)

    def test_pid_magbias_covariance_topicprefix_roundtrip(self):
        raw = """
#ifndef TUNEBOT_CONFIG_H
#define TUNEBOT_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_BTS7960_MOTOR_DRIVER
#define K_P 0.75
#define K_I 0.9
#define K_D 0.42
#define MAG_BIAS { 12.5, -3.0, 7.25 }
#define ACCEL_COV { 0.01, 0.01, 0.01 }
#define GYRO_COV { 0.002, 0.002, 0.002 }
#define ORI_COV { 0.03, 0.03, 0.03 }
#define TOPIC_PREFIX "robot1/"
#define BAUDRATE 921600
#endif
"""
        spec = parse_header_to_spec(raw)
        self.assertEqual(spec["pid"], {"kp": 0.75, "ki": 0.9, "kd": 0.42})
        self.assertEqual(spec["imu_tuning"]["mag_bias"], [12.5, -3.0, 7.25])
        self.assertEqual(spec["imu_tuning"]["accel_cov"], 0.01)
        self.assertEqual(spec["imu_tuning"]["gyro_cov"], 0.002)
        self.assertEqual(spec["advanced"]["topic_prefix"], "robot1/")

        h = generate_config_header(spec)
        self.assertIn("#define K_P 0.75", h)
        self.assertIn("#define K_I 0.9", h)
        self.assertIn("#define K_D 0.42", h)
        self.assertIn("#define MAG_BIAS { 12.5, -3, 7.25 }", h)
        self.assertIn("#define ACCEL_COV { 0.01, 0.01, 0.01 }", h)
        self.assertIn("#define GYRO_COV { 0.002, 0.002, 0.002 }", h)
        self.assertIn('#define TOPIC_PREFIX "robot1/"', h)
        # exactly one definition of each (no passthrough double-emit)
        self.assertEqual(h.count("#define ACCEL_COV"), 1)
        self.assertEqual(h.count("#define TOPIC_PREFIX"), 1)
        self.assertEqual(h.count("#define K_P"), 1)

    def test_dac_pin_roundtrip_and_mcu_gating(self):
        # ESP32 header with a non-default DAC pin -> parsed, then re-emitted.
        raw = """
#ifndef DACBOT_CONFIG_H
#define DACBOT_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_BTS7960_MOTOR_DRIVER
#define BATTERY_PIN 33
#define DAC_PIN 26
#define BATTERY_ADJUST(v) ((v) * ((30 + 7.5) / 7.5) / 1000.0)
#define BAUDRATE 921600
#endif
"""
        spec = parse_header_to_spec(raw)
        self.assertEqual(spec["sensors"].get("dac_pin"), 26)
        self.assertEqual(spec["pins"].get("dac_pin"), 26)
        spec["mcu"] = "ESP32"
        h = generate_config_header(spec)
        self.assertIn("#define DAC_PIN 26", h)
        self.assertEqual(h.count("#define DAC_PIN"), 1)

        # Same spec on a DAC-less MCU (Pico) -> DAC_PIN must NOT be emitted.
        pico = json.loads(json.dumps(self.valid_pico_spec))
        pico["pins"]["dac_pin"] = 25
        hp = generate_config_header(pico)
        self.assertNotIn("#define DAC_PIN", hp)

        # ESP32 form-shape spec with a DAC pin -> emitted.
        esp = json.loads(json.dumps(self.valid_pico_spec))
        esp["mcu"] = "ESP32"
        esp["pins"]["dac_pin"] = 25
        he = generate_config_header(esp)
        self.assertIn("#define DAC_PIN 25", he)

    def test_adc_lut_roundtrip_and_merge_preserves_it(self):
        # A previously-calibrated header with a short LUT (real ones are
        # always 4096 entries; a short one exercises the same regex/format).
        raw = """
#ifndef LUTBOT_CONFIG_H
#define LUTBOT_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define BATTERY_PIN 33
#define DAC_PIN 25
#define USE_ADC_LUT
const int16_t ADC_LUT[4096] = {
    0, 50, 53, 57, 60, 64, 65, 66,
    3959, 3967
};
#define BAUDRATE 921600
#endif
"""
        spec = parse_header_to_spec(raw)
        self.assertEqual(spec["sensors"].get("adc_lut"),
                          [0, 50, 53, 57, 60, 64, 65, 66, 3959, 3967])

        spec["mcu"] = "ESP32"
        h = generate_config_header(spec)
        self.assertIn("#define USE_ADC_LUT", h)
        self.assertIn("const int16_t ADC_LUT[4096] = {", h)
        self.assertIn("ADC_LUT[v]", h)   # LUT-linearized BATTERY_ADJUST
        self.assertEqual(h.count("#define USE_ADC_LUT"), 1)

        # merge_configurations: an overlay that doesn't touch sensors.adc_lut
        # must NOT drop the calibrated LUT already on disk.
        overlay_no_lut = {"sensors": {"battery_min_voltage": 9.5}}
        merged, _ = merge_configurations(spec, overlay_no_lut)
        self.assertEqual(merged["sensors"]["adc_lut"], spec["sensors"]["adc_lut"])
        self.assertEqual(merged["sensors"]["battery_min_voltage"], 9.5)

        # An overlay that DOES set a new LUT (e.g. a fresh calibration run)
        # replaces the old one — overlay wins, per merge_configurations.
        overlay_new_lut = {"sensors": {"adc_lut": [1, 2, 3]}}
        merged3, changes = merge_configurations(spec, overlay_new_lut)
        self.assertEqual(merged3["sensors"]["adc_lut"], [1, 2, 3])
        self.assertTrue(any(c["field"] == "sensors.adc_lut" for c in changes))

    def test_adc_lut_wins_over_stale_raw_battery_adjust(self):
        # Regression: a header this generator wrote WITHOUT a LUT (e.g. the
        # config-sync that runs at the start of an adc_calibrate upload,
        # before the sweep has produced anything yet) contains a literal
        # #define BATTERY_ADJUST(...). A re-parse can't tell that apart from
        # a hand-written override, so it lands in raw_defines — which used to
        # make the "only emit BATTERY_ADJUST if raw_defines doesn't already
        # have one" guard block a LATER regenerate (the auto-commit that
        # follows a completed sweep) from ever emitting the LUT-based
        # formula, silently dropping the whole calibration. The LUT must win
        # unconditionally once it exists, and BATTERY_ADJUST must appear only
        # once (not doubled by the raw_defines passthrough).
        raw = """
#ifndef PRELUT_CONFIG_H
#define PRELUT_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define BATTERY_PIN 33
#define DAC_PIN 25
#define BATTERY_ADJUST(v) ((v) * ((30 + 7.5) / 7.5) / 1000.0)
#define BAUDRATE 921600
#endif
"""
        base = parse_header_to_spec(raw)
        self.assertTrue(any(d.get("name") == "BATTERY_ADJUST" for d in base.get("raw_defines", [])))

        overlay = {"sensors": {"battery_monitor": "ADC_DIVIDER", "adc_lut": list(range(4096))}, "mcu": "ESP32"}
        merged, _ = merge_configurations(base, overlay)
        h = generate_config_header(merged)
        self.assertEqual(h.count("#define BATTERY_ADJUST"), 1)
        self.assertIn("#define USE_ADC_LUT", h)
        self.assertIn("ADC_LUT[v]", h)   # the LUT-linearized formula, not the plain one

    def test_ina219_form_shape_emits_use_ina219(self):
        # Regression, found running a real GenDrv Full Deploy through the web
        # UI: generator.py checked only sensors.use_ina219 (a boolean parser.py
        # sets when it parses an existing #define USE_INA219). The web-UI form
        # (readSpecFromForm()) only ever sets sensors.battery_monitor to the
        # string "INA219" — never that boolean — so a brand-new robot (i2c
        # auto-detect having just set battery_monitor="INA219" in the form,
        # nothing on disk yet to merge with) silently got NO #define USE_INA219
        # at all, and /battery never appeared on the robot.
        overlay = {
            "robot_name": "gendrv_test", "mcu": "GENDRV", "kinematics": "DIFFERENTIAL_DRIVE",
            "motors": {"driver_type": "BTS7960", "max_rpm": 150, "cpr": 900},
            "geometry": {"wheel_diameter": 0.056, "track_width": 0.224, "weight": 2},
            "sensors": {"battery_monitor": "INA219", "sonar": False},
        }
        h = generate_config_header(overlay)
        self.assertIn("#define USE_INA219", h)

        # And parser.py now sets the same form-shape key when it parses one
        # back, so a later merge treats it as the SAME field (overlay wins),
        # not two differently-named copies that silently diverge.
        raw = "#ifndef X_H\n#define X_H\n#define LINO_BASE DIFFERENTIAL_DRIVE\n#define USE_INA219\n#define BAUDRATE 921600\n#endif\n"
        parsed = parse_header_to_spec(raw)
        self.assertTrue(parsed["sensors"]["use_ina219"])
        self.assertEqual(parsed["sensors"]["battery_monitor"], "INA219")

    def test_merge_overlay_wins_over_parsed_battery_fields(self):
        # Regression: parser.py used to store BATTERY_MIN/MAX/CAP and the
        # divider resistors under different key names than the web-UI form
        # overlay uses (battery_min vs. battery_min_voltage, etc). Because
        # merge_configurations() does a deep key-merge, that mismatch meant
        # an overlay's new value landed under its own key while the OLD
        # value stayed live under the base's key — and generator.py's
        # fallback chain checked the stale key first, so the overlay's
        # change was silently dropped. This locks in that the same key is
        # used both ways, so a merge genuinely overwrites.
        raw = """
#ifndef VOLTBOT_CONFIG_H
#define VOLTBOT_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define BATTERY_PIN 33
#define BATTERY_MIN 9
#define BAUDRATE 921600
#endif
"""
        base = parse_header_to_spec(raw)
        self.assertEqual(base["sensors"]["battery_min_voltage"], 9.0)
        self.assertEqual(base["sensors"]["battery_r1"], 30000.0)
        self.assertEqual(base["sensors"]["battery_r2"], 7500.0)

        # Overlay changes the min-cutoff voltage (form field name) — must win.
        overlay = {"sensors": {"battery_min_voltage": 7.5}}
        merged, changes = merge_configurations(base, overlay)
        self.assertEqual(merged["sensors"]["battery_min_voltage"], 7.5)
        self.assertFalse("battery_min" in merged["sensors"])  # no stale duplicate key

        base["mcu"] = "ESP32"
        merged["mcu"] = "ESP32"
        h_before = generate_config_header(base)
        h_after = generate_config_header(merged)
        self.assertIn("#define BATTERY_MIN 9", h_before)
        self.assertIn("#define BATTERY_MIN 7.5", h_after)
        self.assertNotIn("#define BATTERY_MIN 9", h_after)

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

    def test_imu_mag_form_shape_wins_on_merge(self):
        # Regression, same key-mismatch class as INA219/battery/sonar: the
        # real web-UI form (readSpecFromForm()) only ever sets the form-shape
        # sensors.imu / sensors.mag (short names, e.g. "QMI8658") — never the
        # parser-shape sensors.imu_type / sensors.mag_type. A base spec
        # parsed from an existing FAKE_IMU/FAKE_MAG header always carries a
        # non-null imu_type/mag_type placeholder, so a form-shape-only
        # overlay was previously silently ignored on merge.
        raw = """
#ifndef X_CONFIG_H
#define X_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_GENERIC_2_IN_MOTOR_DRIVER
#define USE_FAKE_IMU
#define USE_FAKE_MAG
#define BAUDRATE 921600
#endif
"""
        base = parse_header_to_spec(raw)
        overlay = {"sensors": {"imu": "QMI8658", "mag": "AK09918"}}
        merged, changes = merge_configurations(base, overlay)
        self.assertGreater(len(changes), 0)
        header = generate_config_header(merged)
        self.assertIn("#define USE_QMI8658_IMU", header)
        self.assertIn("#define USE_AK09918_MAG", header)
        self.assertNotIn("#define USE_FAKE_IMU", header)
        self.assertNotIn("#define USE_FAKE_MAG", header)

    def test_i2c_pins_form_shape_wins_on_merge(self):
        # Regression, same key-mismatch class: the web-UI form always sets
        # pins.i2c.sda / pins.i2c.scl, never sensors.i2c_sda / i2c_scl. A
        # base spec parsed from an existing header carries the latter, and
        # generator.py used to check it first, so a fresh i2c pin choice
        # from the form was silently dropped on merge.
        raw = """
#ifndef X_CONFIG_H
#define X_CONFIG_H
#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_GENERIC_2_IN_MOTOR_DRIVER
#define SDA_PIN 21
#define SCL_PIN 22
#define BAUDRATE 921600
#endif
"""
        base = parse_header_to_spec(raw)
        overlay = {"pins": {"i2c": {"sda": 32, "scl": 33}}}
        merged, changes = merge_configurations(base, overlay)
        header = generate_config_header(merged)
        self.assertIn("#define SDA_PIN 32", header)
        self.assertIn("#define SCL_PIN 33", header)


    def test_agent_port_parse_output_free(self):
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "web"))
        from server import _parse_port_check_output
        raw = """---FUSER---
---CONTAINERS---
---PROCESSES---
"""
        res = {"status": "ok", "in_use": False, "pids": [], "holder_type": "none"}
        out = _parse_port_check_output(raw, "/dev/ttyUSB0", "serial", 8888, res)
        self.assertFalse(out["in_use"])
        self.assertEqual(out["holder_type"], "none")

    def test_agent_port_parse_output_container(self):
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "web"))
        from server import _parse_port_check_output
        raw = """---FUSER---
---CONTAINERS---
e4d82a17cb21|uros_agent_serial|microros/micro-ros-agent:jazzy|micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600
---PROCESSES---
"""
        res = {"status": "ok", "in_use": False, "pids": [], "holder_type": "none"}
        out = _parse_port_check_output(raw, "/dev/ttyUSB0", "serial", 8888, res)
        self.assertTrue(out["in_use"])
        self.assertEqual(out["holder_type"], "container")
        self.assertEqual(out["container_name"], "uros_agent_serial")
        self.assertTrue(out["is_microros"])

    def test_agent_port_parse_output_process(self):
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "web"))
        from server import _parse_port_check_output
        raw = """---FUSER---
54321 /dev/ttyUSB0:
---CONTAINERS---
---PROCESSES---
54321 ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
"""
        res = {"status": "ok", "in_use": False, "pids": [], "holder_type": "none"}
        out = _parse_port_check_output(raw, "/dev/ttyUSB0", "serial", 8888, res)
        self.assertTrue(out["in_use"])
        self.assertEqual(out["holder_type"], "process")
        self.assertIn("54321", out["pids"])
        self.assertTrue(out["is_microros"])

    def test_agent_port_parse_output_udp(self):
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "web"))
        from server import _parse_port_check_output
        raw = """---FUSER---
98765 8888/udp:
---CONTAINERS---
---PROCESSES---
98765 python3 -m micro_ros
"""
        res = {"status": "ok", "in_use": False, "pids": [], "holder_type": "none"}
        out = _parse_port_check_output(raw, "/dev/ttyUSB0", "udp", 8888, res)
        self.assertTrue(out["in_use"])
        self.assertEqual(out["holder_type"], "process")
        self.assertIn("98765", out["pids"])

    def test_workflow_settings_persistence(self):
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "web"))
        from server import load_workflow_settings, save_workflow_settings, WORKFLOW_SETTINGS_PATH
        saved = save_workflow_settings({"ros_distro": "rolling", "build_engine": "podman"})
        self.assertEqual(saved["ros_distro"], "rolling")
        self.assertEqual(saved["build_engine"], "podman")
        loaded = load_workflow_settings()
        self.assertEqual(loaded["ros_distro"], "rolling")
        self.assertEqual(loaded["build_engine"], "podman")
        # Clean up
        if os.path.exists(WORKFLOW_SETTINGS_PATH):
            try:
                os.remove(WORKFLOW_SETTINGS_PATH)
            except Exception:
                pass

    def test_rootless_info(self):
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "web"))
        from server import get_rootless_info
        info = get_rootless_info()
        self.assertEqual(info["status"], "ok")
        self.assertIn("commands", info)
        self.assertIn("ubuntu_debian", info["commands"])

    def test_container_install_and_rootless_setup(self):
        res_d = server.install_container_engine("docker")
        self.assertIn("status", res_d)
        self.assertIn("installed", res_d)
        self.assertEqual(res_d["engine"], "docker")

        res_p = server.install_container_engine("podman")
        self.assertIn("status", res_p)
        self.assertIn("installed", res_p)
        self.assertEqual(res_p["engine"], "podman")

        res_rootless = server.setup_rootless_docker()
        self.assertIn("status", res_rootless)
        self.assertIn("is_rootless", res_rootless)


    def test_docker_builder_and_agent_deploy_script(self):
        # Verify Dockerfile.builder exists
        df_path = os.path.join(os.path.dirname(__file__), "docker", "Dockerfile.builder")
        self.assertTrue(os.path.exists(df_path), "Dockerfile.builder must exist in docker/")
        with open(df_path, "r", encoding="utf-8") as f:
            df_content = f.read()
        self.assertIn("platformio", df_content)

        # Verify app.js deploy script generation rules for containerized workflow
        app_js_path = os.path.join(os.path.dirname(__file__), "web", "app.js")
        with open(app_js_path, "r", encoding="utf-8") as f:
            app_js = f.read()
        self.assertIn("Containerized micro-ROS Agent selected", app_js)
        self.assertIn("linorobot2-builder:latest", app_js)
        self.assertIn("microros/micro-ros-agent", app_js)

    def test_configurable_container_registry(self):
        # 1. Check default workflow settings in server.py
        self.assertIn("container_registry", server.DEFAULT_WORKFLOW_SETTINGS)
        self.assertEqual(server.DEFAULT_WORKFLOW_SETTINGS["container_registry"], "auto")

        # 2. Check UI elements in index.html
        html_path = os.path.join(os.path.dirname(__file__), "web", "index.html")
        with open(html_path, "r", encoding="utf-8") as f:
            html = f.read()
        self.assertIn('id="hdr-container-registry"', html)
        self.assertIn('id="cfg-container-registry"', html)
        self.assertIn('id="hdr-custom-registry"', html)

        # 3. Check app.js configurable probe generator
        app_js_path = os.path.join(os.path.dirname(__file__), "web", "app.js")
        with open(app_js_path, "r", encoding="utf-8") as f:
            app_js = f.read()
        self.assertIn("buildRegistryProbeList", app_js)
        self.assertIn("containerRegistry", app_js)


class TestFakeWheelMode(unittest.TestCase):
    """Fake wheel mode: the checkbox has to reach the generated header."""

    BASE = {
        "robot_name": "sim", "kinematics": "DIFFERENTIAL_DRIVE", "mcu": "ESP32",
        "geometry": {"wheel_diameter": 0.065, "track_width": 0.20, "weight": 3.5},
        "motors": {}, "sensors": {}, "pins": {},
    }

    def _header(self, simulation=None, lidar_rxd=None, lidar_udp=False):
        """lidar_rxd / lidar_udp give the simulated scan a route off the board.
        Without one the emulator is deliberately left disabled, so any test that
        expects USE_FAKE_LD19 has to supply one."""
        spec = dict(self.BASE)
        if simulation is not None:
            spec["simulation"] = simulation
        if lidar_rxd is not None:
            spec["pins"] = dict(spec.get("pins") or {}, lidar_rxd=lidar_rxd)
        if lidar_udp:
            spec["telemetry"] = dict(spec.get("telemetry") or {}, use_lidar_udp=True)
        return generate_config_header(spec)

    def test_unchecked_emits_nothing(self):
        self.assertNotIn("USE_FAKE_WHEEL", self._header())
        self.assertNotIn("USE_FAKE_WHEEL", self._header({"fake_wheel": False}))

    def test_checked_emits_define(self):
        self.assertIn("#define USE_FAKE_WHEEL", self._header({"fake_wheel": True}))

    def test_tuning_values_are_optional(self):
        self.assertNotIn("FAKE_ROBOT_MASS", self._header({"fake_wheel": True}))
        tuned = self._header({"fake_wheel": True, "robot_mass": 6.0, "wheel_noise_rpm": 1.5})
        self.assertIn("#define FAKE_ROBOT_MASS 6", tuned)
        self.assertIn("#define FAKE_WHEEL_NOISE_RPM 1.5", tuned)

    def test_robot_weight_is_emitted_for_the_simulated_mass(self):
        # with no explicit mass, the firmware falls back to ROBOT_WEIGHT
        self.assertIn("#define ROBOT_WEIGHT 3.5", self._header({"fake_wheel": True}))

    def test_round_trips_through_the_parser(self):
        header = self._header({"fake_wheel": True, "robot_mass": 6.0, "wheel_noise_rpm": 1.5})
        sim = parse_header_to_spec(header)["simulation"]
        self.assertTrue(sim["fake_wheel"])
        self.assertEqual(sim["robot_mass"], 6.0)
        self.assertEqual(sim["wheel_noise_rpm"], 1.5)

    def test_commented_sample_is_not_parsed_as_enabled(self):
        # lino_base_config.h ships the options commented out
        self.assertNotIn("simulation", parse_header_to_spec("// #define USE_FAKE_WHEEL\n"))


    def test_fake_ld19_emits_define_and_geometry(self):
        sim_spec = {
            "fake_ld19": True,
            "map_width": 8.0,
            "map_height": 5.0,
            "wall_obstacle": True,
            "wall_x1": 1.2,
            "wall_y1": -0.5,
            "wall_x2": 1.2,
            "wall_y2": 0.5
        }
        header = self._header(sim_spec, lidar_rxd=4)
        self.assertIn("#define USE_FAKE_LD19", header)
        self.assertIn("#define LIDAR_RXD 4", header)
        self.assertIn("#define FAKE_MAP_WIDTH 8.0f", header)
        self.assertIn("#define FAKE_MAP_HEIGHT 5.0f", header)
        self.assertIn("#define FAKE_WALL_OBSTACLE 1", header)
        self.assertIn("#define FAKE_WALL_X1 1.20f", header)

    def test_fake_ld19_round_trips_through_parser(self):
        sim_spec = {
            "fake_wheel": True,
            "fake_ld19": True,
            "map_width": 6.0,
            "map_height": 4.0,
            "wall_obstacle": True,
            "wall_x1": 1.0,
            "wall_y1": -0.8,
            "wall_x2": 1.0,
            "wall_y2": 0.8
        }
        header = self._header(sim_spec, lidar_rxd=4)
        parsed = parse_header_to_spec(header)["simulation"]
        self.assertTrue(parsed["fake_wheel"])
        self.assertTrue(parsed["fake_ld19"])
        self.assertEqual(parsed["map_width"], 6.0)
        self.assertEqual(parsed["map_height"], 4.0)
        self.assertTrue(parsed["wall_obstacle"])
        self.assertEqual(parsed["wall_x1"], 1.0)
        self.assertEqual(parsed["wall_y1"], -0.8)
        self.assertEqual(parsed["wall_x2"], 1.0)
        self.assertEqual(parsed["wall_y2"], 0.8)



class TestSimulationDefaultsAndWarning(unittest.TestCase):
    """Reference designs ship simulated, and a simulated header must say so."""

    BASE = {
        "robot_name": "sim", "kinematics": "DIFFERENTIAL_DRIVE", "mcu": "ESP32",
        "geometry": {"wheel_diameter": 0.065, "track_width": 0.20, "weight": 3.5},
        "motors": {}, "sensors": {}, "pins": {},
    }

    def _header(self, **over):
        spec = dict(self.BASE)
        spec.update(over)
        return generate_config_header(spec)

    def test_warning_present_for_fake_wheel(self):
        self.assertIn("SIMULATION MODE IS ENABLED", self._header(simulation={"fake_wheel": True}))

    def test_warning_present_for_fake_lidar_alone(self):
        # a real drivetrain with a simulated scan still is not a real robot config
        self.assertIn("SIMULATION MODE IS ENABLED", self._header(simulation={"fake_ld19": True}))

    def test_no_warning_on_a_real_robot_config(self):
        self.assertNotIn("SIMULATION MODE IS ENABLED", self._header())
        self.assertNotIn("SIMULATION MODE IS ENABLED", self._header(simulation={"fake_wheel": False}))

    def test_warning_precedes_the_defines_it_refers_to(self):
        header = self._header(simulation={"fake_wheel": True, "fake_ld19": True})
        self.assertLess(header.index("SIMULATION MODE IS ENABLED"),
                        header.index("#define USE_FAKE_WHEEL"))

    def test_simulated_wifi_design_emits_lidar_udp(self):
        # the esp32_wifi_ld19 reference design relays the simulated scan over UDP
        header = self._header(transport="WIFI_UDP",
                              wifi_settings={"ssid": "S", "password": "P",
                                             "agent_ip": "192.168.1.100", "agent_port": 8888},
                              telemetry={"transport": "WIFI_UDP", "use_lidar_udp": True},
                              simulation={"fake_wheel": True, "fake_ld19": True})
        self.assertIn("#define USE_FAKE_LD19", header)
        # the header carries the switch; the address lives in the secrets file
        self.assertIn("#define USE_LIDAR_UDP", header)

    def test_fake_lidar_emits_its_output_pin(self):
        # without LIDAR_RXD the firmware starts the emulator with tx_pin -1 and
        # transmits nothing: it builds, the board looks fine, no scan appears
        header = self._header(simulation={"fake_ld19": True}, pins={"lidar_rxd": 4})
        self.assertIn("#define USE_FAKE_LD19", header)
        self.assertIn("#define LIDAR_RXD 4", header)

    def test_fake_lidar_pin_is_emitted_for_serial_transport_too(self):
        # it used to be emitted only on the WiFi + LiDAR-UDP path
        header = self._header(transport="SERIAL", simulation={"fake_ld19": True},
                              pins={"lidar_rxd": 4})
        self.assertIn("#define LIDAR_RXD 4", header)

    def test_serial_design_defines_a_lidar_pin_and_udp_design_does_not(self):
        # the scan leaves over the LiDAR UART on the serial design, and over
        # WiFi UDP on the other -- where no UART pin is involved at all
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "web", "app.js")) as f:
            app_js = f.read()
        import re

        def design_body(key):
            # bound at the next preset key or the end of the PRESETS object,
            # since the last entry closes differently from the others
            body = app_js.split(key + ": {", 1)[1]
            end = re.search(r"\n  \},|\n  \}\n\};", body)
            return body[:end.start()] if end else body

        # an actual assignment, not a mention of the name in a comment
        assigned = lambda body: re.search(r"^\s*lidar_rxd:\s*-?\d", body, re.M) is not None

        serial_design = design_body("gendrv_serial_ld19")
        self.assertTrue(assigned(serial_design), "serial design needs a LiDAR UART pin")

        udp_design = design_body("esp32_wifi_ld19")
        self.assertFalse(assigned(udp_design),
                         "UDP design must not pin a UART: the scan goes over WiFi")
        self.assertIn("use_lidar_udp: true", udp_design)

    def test_warns_when_a_simulated_scan_has_nowhere_to_go(self):
        spec = {
            "robot_name": "s", "kinematics": "DIFFERENTIAL_DRIVE", "mcu": "ESP32",
            "geometry": {"wheel_diameter": 0.065, "track_width": 0.20},
            "motors": {}, "sensors": {}, "pins": {},
            "simulation": {"fake_ld19": True},
        }
        _, errors, _ = validate_robot_spec(spec)
        msgs = " ".join(e.message for e in errors if e.level == "WARNING")
        self.assertIn("left DISABLED", msgs)
        self.assertIn("no route off the board", msgs)

    def test_no_warning_when_the_scan_goes_out_over_udp(self):
        spec = {
            "robot_name": "s", "kinematics": "DIFFERENTIAL_DRIVE", "mcu": "ESP32",
            "geometry": {"wheel_diameter": 0.065, "track_width": 0.20},
            "motors": {}, "sensors": {}, "pins": {},
            "telemetry": {"transport": "WIFI_UDP", "use_lidar_udp": True},
            "simulation": {"fake_ld19": True},
        }
        _, errors, _ = validate_robot_spec(spec)
        self.assertNotIn("nowhere to go", " ".join(e.message for e in errors))

    def test_no_warning_when_a_lidar_pin_is_set(self):
        spec = {
            "robot_name": "s", "kinematics": "DIFFERENTIAL_DRIVE", "mcu": "GENDRV",
            "geometry": {"wheel_diameter": 0.065, "track_width": 0.20},
            "motors": {}, "sensors": {}, "pins": {"lidar_rxd": 4},
            "simulation": {"fake_ld19": True},
        }
        _, errors, _ = validate_robot_spec(spec)
        self.assertNotIn("nowhere to go", " ".join(e.message for e in errors))



class TestReferenceDesignsAreSimulated(unittest.TestCase):
    """Every reference design in the studio starts in fake wheel mode, and the
    two simulation-first designs exist. Parsed out of app.js, which is where
    the presets live."""

    @classmethod
    def setUpClass(cls):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "web", "app.js")) as f:
            cls.app_js = f.read()

    def test_simulation_designs_exist(self):
        self.assertIn("gendrv_serial_ld19:", self.app_js)
        self.assertIn("esp32_wifi_ld19:", self.app_js)

    def test_simulation_designs_are_selectable(self):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "web", "index.html")) as f:
            html = f.read()
        self.assertIn('value="gendrv_serial_ld19"', html)
        self.assertIn('value="esp32_wifi_ld19"', html)

    def test_every_preset_defaults_to_fake_wheel(self):
        # the normalization loop applies it to all presets, so it cannot be
        # forgotten when a new design is added
        self.assertIn("p.simulation = Object.assign({ fake_wheel: true }, p.simulation || {});",
                      self.app_js)

    def test_simulation_designs_keep_their_own_pinout(self):
        # fake wheels are driven by the PID's PWM, but the design still needs a
        # coherent pinout rather than the all -1 bare map
        self.assertIn("const PINNED_PRESETS =", self.app_js)
        for key in ("gendrv_serial_ld19", "esp32_wifi_ld19"):
            self.assertIn(f'"{key}"', self.app_js.split("const PINNED_PRESETS =")[1][:200])

    def test_new_module_defaults_to_simulation(self):
        self.assertIn("function defaultSimulationForDetectedModule", self.app_js)
        self.assertIn("defaultSimulationForDetectedModule(guessed.chipName);", self.app_js)

    def test_user_choice_is_not_overridden(self):
        self.assertIn("if (userTouchedSimulation) return;", self.app_js)


class TestDualCoreAndSimulation(unittest.TestCase):
    """The dual-core control task and simulation mode must never both reach the
    header. The fake LiDAR drives the simulated pose and its UART from the loop
    while the control task drives the same pose from the other core with no lock
    between them, and the board crash-loops -- a failure that only shows up on
    hardware, as a reconnect storm, long after the config was generated."""

    BASE = {
        "robot_name": "sim", "kinematics": "DIFFERENTIAL_DRIVE", "mcu": "ESP32",
        "geometry": {"wheel_diameter": 0.065, "track_width": 0.20, "weight": 3.5},
        "motors": {}, "sensors": {}, "pins": {},
    }

    def _header(self, dual_core, simulation=None, lidar_rxd=4):
        spec = dict(self.BASE)
        spec["telemetry"] = {"use_dual_core": dual_core}
        if simulation is not None:
            spec["simulation"] = simulation
        # give the scan a route, or the emulator is left disabled on purpose
        spec["pins"] = dict(spec.get("pins") or {}, lidar_rxd=lidar_rxd)
        return generate_config_header(spec)

    def test_dual_core_emitted_on_a_real_robot(self):
        self.assertIn("#define USE_DUAL_CORE", self._header(True))

    def test_dual_core_absent_when_not_asked_for(self):
        self.assertNotIn("USE_DUAL_CORE", self._header(False))

    def test_simulation_suppresses_dual_core(self):
        for sim in ({"fake_wheel": True}, {"fake_ld19": True},
                    {"fake_wheel": True, "fake_ld19": True}):
            with self.subTest(sim=sim):
                self.assertNotIn("USE_DUAL_CORE", self._header(True, sim))

    def test_fake_sonar_and_safety_stop_default_on_with_the_lidar(self):
        """The range comes from the same raycast as the scan, so it costs no
        pins -- a simulated robot should get the protection a real one has."""
        header = self._header(False, {"fake_wheel": True, "fake_ld19": True})
        self.assertIn("#define USE_FAKE_SONAR", header)
        self.assertIn("#define USE_SAFETY_STOP", header)

    def test_fake_sonar_can_be_turned_off(self):
        header = self._header(
            False, {"fake_wheel": True, "fake_ld19": True, "fake_sonar": False})
        self.assertNotIn("USE_FAKE_SONAR", header)
        self.assertNotIn("USE_SAFETY_STOP", header)

    def test_no_sonar_without_the_fake_lidar(self):
        """The cone is raycast against the simulated room; with no room there is
        nothing to measure."""
        self.assertNotIn("USE_FAKE_SONAR", self._header(False, {"fake_wheel": True}))


class TestReferenceDesignsAreExplicit(unittest.TestCase):
    """The simulated reference designs must state their own settings. Relying on
    the form defaults is how gendrv_serial_ld19 came to ship a 6x4 m room after
    the default grew to 16x12, and a LIDAR_RXD of -1 that silenced the scan."""

    @classmethod
    def setUpClass(cls):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "web", "app.js")) as f:
            cls.app_js = f.read()

    def _preset(self, name):
        body = self.app_js.split(f"{name}: {{", 1)[1]
        # up to the start of the next top-level preset, which is enough context
        return body[:2500]

    def _preset_code(self, name):
        """Preset body with // comments removed, so an assertion about a key
        cannot be satisfied (or defeated) by prose mentioning it."""
        return "\n".join(
            line.split("//", 1)[0] for line in self._preset(name).splitlines())

    def test_simulated_designs_disable_dual_core(self):
        for name in ("gendrv_serial_ld19", "esp32_wifi_ld19"):
            with self.subTest(design=name):
                self.assertIn("dual_core: false", self._preset(name))

    def test_simulated_designs_pin_their_room(self):
        for name in ("gendrv_serial_ld19", "esp32_wifi_ld19"):
            with self.subTest(design=name):
                body = self._preset(name)
                self.assertIn("map_width: 10.0", body)
                self.assertIn("map_height: 6.0", body)
                self.assertIn("fake_sonar: true", body)

    def test_serial_design_declares_its_wired_lidar_pin(self):
        """IO4 is wired to the LiDAR data line on the reference gendrv, so the
        design names it and the scan has a route out. A board without that wire
        sets -1, and TestFakeLd19NeedsARouteOffTheBoard covers what then
        happens: the emulator is left disabled rather than writing into a pad."""
        self.assertIn("lidar_rxd: 4", self._preset_code("gendrv_serial_ld19"))

    def test_udp_design_has_no_lidar_pin_at_all(self):
        """It ships the scan over WiFi, so the pin is not merely unset -- the
        design has no business naming one."""
        self.assertNotIn("lidar_rxd", self._preset_code("esp32_wifi_ld19"))


class TestFakeLd19NeedsARouteOffTheBoard(unittest.TestCase):
    """The emulated scan leaves over the LiDAR UART pin or over WiFi UDP. With
    neither, the emulator is not enabled at all: it would burn the control loop
    building packets and writing them into an unconnected pad while the board
    reported success and no scan ever reached ROS. On a stock board nothing is
    wired to that pin, so unset is the honest default."""

    BASE = {
        "robot_name": "sim", "kinematics": "DIFFERENTIAL_DRIVE", "mcu": "ESP32",
        "geometry": {"wheel_diameter": 0.065, "track_width": 0.20, "weight": 3.5},
        "motors": {}, "sensors": {},
    }

    def _header(self, pins=None, telemetry=None):
        spec = dict(self.BASE)
        spec["simulation"] = {"fake_wheel": True, "fake_ld19": True}
        spec["pins"] = pins or {}
        spec["telemetry"] = telemetry or {}
        return generate_config_header(spec)

    def test_disabled_when_nothing_carries_the_scan(self):
        header = self._header()
        self.assertNotIn("#define USE_FAKE_LD19", header)
        self.assertNotIn("#define LIDAR_RXD", header)
        self.assertIn("NOT enabled", header)

    def test_disabled_when_the_pin_is_explicitly_unwired(self):
        self.assertNotIn("#define USE_FAKE_LD19", self._header(pins={"lidar_rxd": -1}))

    def test_enabled_once_the_pin_is_wired(self):
        header = self._header(pins={"lidar_rxd": 4})
        self.assertIn("#define USE_FAKE_LD19", header)
        self.assertIn("#define LIDAR_RXD 4", header)

    def test_enabled_over_udp_without_any_pin(self):
        header = self._header(telemetry={"use_lidar_udp": True})
        self.assertIn("#define USE_FAKE_LD19", header)
        self.assertNotIn("#define LIDAR_RXD", header)

    def test_fake_wheel_survives_a_disabled_lidar(self):
        """Losing the scan must not cost the drivetrain simulation, which is
        what makes a bare module drivable in the first place."""
        self.assertIn("#define USE_FAKE_WHEEL", self._header())


class TestContainerRegistryIsAPreference(unittest.TestCase):
    """The registry is a user setting, never a hardcoded host: a LAN or cluster
    registry is site specific, so there is no default to ship."""

    @classmethod
    def setUpClass(cls):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "web", "app.js")) as f:
            cls.app_js = f.read()

    def test_no_registry_host_is_hardcoded(self):
        """A bare registry hostname in the source would override the user's
        choice on every machine that runs the studio."""
        for host in ("registry-1632", ".ts.net", "localhost:5000"):
            with self.subTest(host=host):
                self.assertNotIn(host, self.app_js)

    def test_agent_image_goes_through_the_preference(self):
        self.assertIn("function configuredRegistryHosts", self.app_js)
        self.assertIn("_resolve_agent_img", self.app_js)

    def test_agent_image_no_longer_pulls_docker_hub_directly(self):
        """Every containerised agent path must resolve the image first."""
        self.assertNotIn('podman pull "$IMG"', self.app_js)
        self.assertNotIn('$_DK pull "$IMG"', self.app_js)


if __name__ == "__main__":
    unittest.main()
