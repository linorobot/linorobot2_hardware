#!/usr/bin/env python3
"""
Linorobot2 Hardware Rule Validator
Validates robot hardware configurations against electrical and MCU-specific constraints.
"""

import math
from typing import Dict, List, Tuple, Any

ESP32_STRAPPING_PINS = {0, 2, 12, 14, 15}
ESP32_INPUT_ONLY_PINS = {34, 35, 36, 39}
ESP32_FLASH_PINS = {6, 7, 8, 9, 10, 11}
ESP32S3_STRAPPING_PINS = {0, 3, 45, 46}
RP2040_ADC_PINS = {26, 27, 28, 29}


class ValidationError:
    def __init__(self, level: str, field: str, message: str):
        self.level = level  # "ERROR" or "WARNING"
        self.field = field
        self.message = message

    def __str__(self):
        return f"[{self.level}] {self.field}: {self.message}"


def validate_robot_spec(spec: Dict[str, Any]) -> Tuple[bool, List[ValidationError], Dict[str, Any]]:
    errors: List[ValidationError] = []
    stats: Dict[str, Any] = {}

    robot_name = spec.get("robot_name", "unnamed_robot")
    mcu = spec.get("mcu", "").upper()
    kinematics = spec.get("kinematics", "")
    geometry = spec.get("geometry", {})
    motors = spec.get("motors", {})
    pins = spec.get("pins", {})
    sensors = spec.get("sensors", {})

    # 1. Physics & Kinematics Calculations
    wheel_d = geometry.get("wheel_diameter", 0.0)
    track_w = geometry.get("track_width", 0.0)
    max_rpm = motors.get("max_rpm", 0.0)
    cpr = motors.get("cpr", 0.0)

    if wheel_d <= 0:
        errors.append(ValidationError("ERROR", "geometry.wheel_diameter", "Wheel diameter must be positive (> 0 m)"))
    if track_w <= 0:
        errors.append(ValidationError("ERROR", "geometry.track_width", "Track width must be positive (> 0 m)"))
    if max_rpm <= 0:
        errors.append(ValidationError("ERROR", "motors.max_rpm", "Max motor RPM must be positive (> 0)"))
    if cpr <= 0:
        errors.append(ValidationError("ERROR", "motors.cpr", "Encoder CPR must be positive (> 0)"))

    if wheel_d > 0 and max_rpm > 0 and cpr > 0:
        wheel_circ = math.pi * wheel_d
        max_linear_speed = (wheel_circ * max_rpm / 60.0) * 0.85  # 85% PID headroom
        max_angular_speed = (2.0 * max_linear_speed) / track_w if track_w > 0 else 0.0
        ticks_per_meter = cpr / wheel_circ

        stats["wheel_circumference_m"] = round(wheel_circ, 4)
        stats["max_linear_speed_m_s"] = round(max_linear_speed, 3)
        stats["max_angular_speed_rad_s"] = round(max_angular_speed, 3)
        stats["ticks_per_meter"] = round(ticks_per_meter, 1)

    # 2. Pin Conflict Detection (No single pin assigned twice)
    assigned_pins: Dict[int, List[str]] = {}

    def register_pin(pin: Any, name: str, is_output: bool = False):
        if pin is None or not isinstance(pin, int):
            return
        if pin not in assigned_pins:
            assigned_pins[pin] = []
        assigned_pins[pin].append(name)

        # Check MCU Specific Constraints
        if mcu in ["ESP32", "GENDRV"]:
            if pin in ESP32_FLASH_PINS:
                errors.append(ValidationError("ERROR", name, f"GPIO {pin} is connected to internal SPI Flash! Strictly forbidden."))
            if is_output and pin in ESP32_INPUT_ONLY_PINS:
                errors.append(ValidationError("ERROR", name, f"GPIO {pin} is an INPUT-ONLY pin and cannot output PWM/DIR signals!"))
            if not is_output and pin in ESP32_STRAPPING_PINS:
                errors.append(ValidationError("WARNING", name, f"GPIO {pin} is an ESP32 strapping pin. Connecting encoders may cause boot failures if pulled LOW at power-on."))
        elif mcu == "ESP32S3":
            if not is_output and pin in ESP32S3_STRAPPING_PINS:
                errors.append(ValidationError("WARNING", name, f"GPIO {pin} is an ESP32-S3 strapping pin."))
        elif mcu in ["PICO", "PICO2"]:
            if pin < 0 or pin > 29:
                errors.append(ValidationError("ERROR", name, f"GP{pin} is out of range for RP2040/RP2350 (0-29)."))

    # Register motor pins
    driver_type = motors.get("driver_type", "")
    num_motors = 2 if kinematics == "DIFFERENTIAL_DRIVE" else 4

    for i in range(1, num_motors + 1):
        m_pins = pins.get(f"motor{i}", {})
        if driver_type == "BTS7960":
            register_pin(m_pins.get("pwm_r"), f"motor{i}.pwm_r", is_output=True)
            register_pin(m_pins.get("pwm_l"), f"motor{i}.pwm_l", is_output=True)
            register_pin(m_pins.get("en"), f"motor{i}.en", is_output=True)
        elif driver_type == "GENERIC_2_IN":
            register_pin(m_pins.get("pwm"), f"motor{i}.pwm", is_output=True)
            register_pin(m_pins.get("in_a"), f"motor{i}.in_a", is_output=True)
            register_pin(m_pins.get("in_b"), f"motor{i}.in_b", is_output=True)
        elif driver_type == "GENERIC_1_IN":
            register_pin(m_pins.get("pwm"), f"motor{i}.pwm", is_output=True)
            register_pin(m_pins.get("dir"), f"motor{i}.dir", is_output=True)

    # Register encoder pins
    enc_pins = pins.get("encoders", {})
    for i in range(1, num_motors + 1):
        register_pin(enc_pins.get(f"m{i}_a"), f"encoders.m{i}_a", is_output=False)
        register_pin(enc_pins.get(f"m{i}_b"), f"encoders.m{i}_b", is_output=False)

    # Register I2C pins
    i2c = pins.get("i2c", {})
    register_pin(i2c.get("sda"), "i2c.sda", is_output=False)
    register_pin(i2c.get("scl"), "i2c.scl", is_output=False)

    # Register LED pin
    register_pin(pins.get("led"), "pins.led", is_output=True)

    # Register Battery ADC pin
    if sensors.get("battery_monitor") == "ADC_DIVIDER":
        bat_pin = pins.get("battery_pin")
        register_pin(bat_pin, "pins.battery_pin", is_output=False)
        if mcu in ["PICO", "PICO2"] and bat_pin is not None and bat_pin not in RP2040_ADC_PINS:
            errors.append(ValidationError("ERROR", "pins.battery_pin", f"GP{bat_pin} is not an analog ADC pin on RP2040/RP2350 (Must be GP26, GP27, or GP28)."))

    # Register Sonar pins
    if sensors.get("sonar", False):
        sonar = pins.get("sonar", {})
        register_pin(sonar.get("trig"), "sonar.trig", is_output=True)
        register_pin(sonar.get("echo"), "sonar.echo", is_output=False)

    # Check for duplicate pin assignments
    for pin_num, pin_usages in assigned_pins.items():
        if len(pin_usages) > 1:
            errors.append(ValidationError("ERROR", "pins", f"Pin {pin_num} is assigned to multiple functions: {', '.join(pin_usages)}"))

    is_valid = not any(e.level == "ERROR" for e in errors)
    return is_valid, errors, stats


if __name__ == "__main__":
    sample_spec = {
        "robot_name": "rover_pico",
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
    valid, errs, st = validate_robot_spec(sample_spec)
    print("Valid:", valid)
    print("Stats:", st)
    for err in errs:
        print(err)
