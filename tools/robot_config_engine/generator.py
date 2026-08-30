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

from typing import Dict, Any

def generate_config_header(spec: Dict[str, Any]) -> str:
    name = spec.get("robot_name", "custom_robot").upper()
    kinematics = spec.get("kinematics", "DIFFERENTIAL_DRIVE")
    geom = spec.get("geometry", {})
    motors = spec.get("motors", {})
    sensors = spec.get("sensors", {})
    pins = spec.get("pins", {})
    telemetry = spec.get("telemetry", {})
    driver = motors.get("driver_type", "BTS7960")

    lines = [
        "// Copyright (c) 2026 Thomas Chou, Paul Bouchier, Linorobot contributors",
        f"// Auto-generated configuration for {spec.get('robot_name', 'custom_robot')}",
        f"#ifndef {name}_CONFIG_H",
        f"#define {name}_CONFIG_H",
        "",
        f"#define LINO_BASE {kinematics}",
        "",
        "// Robot Physical Geometry",
        f"#define WHEEL_DIAMETER {geom.get('wheel_diameter', 0.065):.4f} // meters",
        f"#define LR_WHEELS_DISTANCE {geom.get('track_width', 0.20):.4f} // meters",
    ]

    if "wheelbase" in geom:
        lines.append(f"#define FR_WHEELS_DISTANCE {geom['wheelbase']:.4f} // meters")
    if "weight" in geom:
        lines.append(f"#define ROBOT_WEIGHT {geom['weight']:.2f} // kg")

    lines.extend([
        "",
        "// Motor Driver & Characteristics",
        f"#define USE_{driver}_MOTOR_DRIVER",
        f"#define MOTOR_MAX_RPM {motors.get('max_rpm', 300)}",
        f"#define MAX_RPM_RATIO 0.85",
        f"#define MOTOR_OPERATING_VOLTAGE {motors.get('operating_voltage', 12.0):.1f}",
        f"#define MOTOR_POWER_MAX_VOLTAGE {motors.get('max_voltage', 12.0):.1f}",
        f"#define MOTOR_POWER_MEASURED_VOLTAGE {motors.get('max_voltage', 12.0):.1f}",
    ])

    if "rated_torque" in motors:
        lines.append(f"#define MOTOR_RATED_TORQUE {motors['rated_torque']:.2f} // kg*cm")
    if "rated_voltage" in motors:
        lines.append(f"#define MOTOR_RATED_VOLTAGE {motors['rated_voltage']:.1f} // V")

    lines.extend([
        "",
        f"#define PWM_BITS {motors.get('pwm_bits', 10)}",
        f"#define PWM_FREQUENCY {motors.get('pwm_frequency', 20000)}",
        f"#define PWM_MAX (1 << PWM_BITS) - 1",
        f"#define PWM_MIN -PWM_MAX",
        "",
        f"#define COUNTS_PER_REV1 {motors.get('cpr', 1320)}",
        f"#define COUNTS_PER_REV2 {motors.get('cpr', 1320)}",
        f"#define COUNTS_PER_REV3 {motors.get('cpr', 1320)}",
        f"#define COUNTS_PER_REV4 {motors.get('cpr', 1320)}",
        "",
        f"#define MOTOR1_INV {str(motors.get('motor1_inv', False)).lower()}",
        f"#define MOTOR2_INV {str(motors.get('motor2_inv', True)).lower()}",
        f"#define MOTOR3_INV {str(motors.get('motor3_inv', False)).lower()}",
        f"#define MOTOR4_INV {str(motors.get('motor4_inv', True)).lower()}",
        "#define MOTOR1_ENCODER_INV false",
        "#define MOTOR2_ENCODER_INV false",
        "#define MOTOR3_ENCODER_INV false",
        "#define MOTOR4_ENCODER_INV false",
        "",
        "// Default PID Tuning Constants",
        "#define K_P 0.6",
        "#define K_I 0.8",
        "#define K_D 0.5",
        "",
        "// Pin Assignments",
    ])

    mcu_name = spec.get("mcu", "").upper()
    led_pin = pins.get("led")
    if led_pin is None:
        led_pin = "LED_BUILTIN" if mcu_name in ["PICOW", "PICO2W"] else (25 if "PICO" in mcu_name else 2)
    lines.append(f"#define LED_PIN {led_pin}")

    # Motor Pins
    num_active_motors = 2 if kinematics == "DIFFERENTIAL_DRIVE" else 4
    for i in range(1, 5):
        if i <= num_active_motors:
            m = pins.get(f"motor{i}", {})
            if driver == "BTS7960":
                lines.append(f"#define MOTOR{i}_PWM_FORWARD {m.get('pwm_fwd', 4)}")
                lines.append(f"#define MOTOR{i}_PWM_REVERSE {m.get('pwm_rev', 5)}")
                lines.append(f"#define MOTOR{i}_ENABLE {m.get('enable', 12)}")
            elif driver == "GENERIC_2_IN":
                lines.append(f"#define MOTOR{i}_PWM {m.get('pwm', 4)}")
                lines.append(f"#define MOTOR{i}_IN_A {m.get('in_a', 5)}")
                lines.append(f"#define MOTOR{i}_IN_B {m.get('in_b', 6)}")
            elif driver == "GENERIC_1_IN":
                lines.append(f"#define MOTOR{i}_PWM {m.get('pwm', 4)}")
                lines.append(f"#define MOTOR{i}_DIR {m.get('dir', 5)}")
            elif driver == "ESC":
                lines.append(f"#define MOTOR{i}_PWM {m.get('pwm', 4)}")
        else:
            if driver == "BTS7960":
                lines.append(f"#define MOTOR{i}_PWM_FORWARD -1")
                lines.append(f"#define MOTOR{i}_PWM_REVERSE -1")
                lines.append(f"#define MOTOR{i}_ENABLE -1")
            elif driver == "GENERIC_2_IN":
                lines.append(f"#define MOTOR{i}_PWM -1")
                lines.append(f"#define MOTOR{i}_IN_A -1")
                lines.append(f"#define MOTOR{i}_IN_B -1")
            elif driver == "GENERIC_1_IN":
                lines.append(f"#define MOTOR{i}_PWM -1")
                lines.append(f"#define MOTOR{i}_DIR -1")
            elif driver == "ESC":
                lines.append(f"#define MOTOR{i}_PWM -1")

    lines.append("")
    # Encoder Pins
    for i in range(1, 5):
        if i <= num_active_motors:
            enc = pins.get(f"encoder{i}", {})
            lines.append(f"#define MOTOR{i}_ENCODER_A {enc.get('a', 14)}")
            lines.append(f"#define MOTOR{i}_ENCODER_B {enc.get('b', 15)}")
        else:
            lines.append(f"#define MOTOR{i}_ENCODER_A -1")
            lines.append(f"#define MOTOR{i}_ENCODER_B -1")

    # Servo for Ackermann
    if kinematics == "ACKERMANN":
        servo_pin = pins.get("servo", 21)
        lines.append(f"#define STEERING_SERVO_PIN {servo_pin}")

    lines.append("")
    lines.append("// Sensor Configuration")

    # IMU
    imu = sensors.get("imu_type") or sensors.get("imu", "USE_FAKE_IMU")
    if imu and imu not in ["NONE", "USE_FAKE_IMU", "FAKE"]:
        macro = imu if imu.startswith("USE_") else f"USE_{imu}_IMU"
        lines.append(f"#define {macro}")
    else:
        lines.append("#define USE_FAKE_IMU")

    # Magnetometer
    mag = sensors.get("mag_type") or sensors.get("mag", "USE_FAKE_MAG")
    if mag and mag not in ["NONE", "USE_FAKE_MAG", "FAKE"]:
        macro = mag if mag.startswith("USE_") else f"USE_{mag}_MAG"
        lines.append(f"#define {macro}")
    else:
        lines.append("#define USE_FAKE_MAG")

    # I2C Bus Pins
    i2c_sda = sensors.get("i2c_sda", -1)
    i2c_scl = sensors.get("i2c_scl", -1)
    if i2c_sda >= 0 and i2c_scl >= 0:
        lines.append(f"#define SDA_PIN {i2c_sda}")
        lines.append(f"#define SCL_PIN {i2c_scl}")

    # Battery
    bat_mon = sensors.get("battery_monitor", "NONE")
    bat_pin = pins.get("battery_pin", sensors.get("battery_pin", -1))
    if sensors.get("use_ina219", False) or bat_mon == "INA219":
        lines.append("#define USE_INA219")
    elif bat_mon == "ADC_DIVIDER" or bat_pin >= 0:
        lines.append("#define USE_BATTERY_MONITOR")
        lines.append(f"#define BATTERY_PIN {bat_pin if bat_pin >= 0 else 26}")
        r1 = sensors.get("battery_r1", sensors.get("battery_divider_r1", 30000.0))
        r2 = sensors.get("battery_r2", sensors.get("battery_divider_r2", 7500.0))
        lines.append(f"#define BATTERY_R1 {r1:.1f}")
        lines.append(f"#define BATTERY_R2 {r2:.1f}")

    cap = sensors.get("battery_capacity", sensors.get("battery_cap"))
    if cap is not None:
        lines.append(f"#define BATTERY_CAP {float(cap):.2f}")
    bmin = sensors.get("battery_min_voltage", sensors.get("battery_min"))
    if bmin is not None:
        lines.append(f"#define BATTERY_MIN {float(bmin):.2f}")
    bmax = sensors.get("battery_max_voltage", sensors.get("battery_max"))
    if bmax is not None:
        lines.append(f"#define BATTERY_MAX {float(bmax):.2f}")
    bdip = sensors.get("battery_dip")
    if bdip is not None:
        lines.append(f"#define BATTERY_DIP {float(bdip):.2f}")

    # Sonar
    sonar_trig = sensors.get("sonar_trig", -1)
    sonar_echo = sensors.get("sonar_echo", -1)
    if sonar_trig >= 0 and sonar_echo >= 0:
        lines.append("#define USE_SONAR")
        lines.append(f"#define TRIG_PIN {sonar_trig}")
        lines.append(f"#define ECHO_PIN {sonar_echo}")

    # Baudrate
    baudrate = telemetry.get("baudrate") or spec.get("baudrate", 921600)
    lines.append("")
    lines.append("// Telemetry & micro-ROS Communication")
    lines.append(f"#define BAUDRATE {baudrate}")

    # WiFi
    use_wifi = telemetry.get("use_wifi", False) or ("WIFI" in str(telemetry.get("transport", "")).upper())
    if use_wifi:
        lines.append("#define USE_WIFI")
        lines.append(f"#define AGENT_PORT {telemetry.get('agent_port', 8888)}")
    if telemetry.get("use_syslog", False):
        lines.append("#define USE_SYSLOG")
    if telemetry.get("use_arduino_ota", False):
        lines.append("#define USE_ARDUINO_OTA")
    if telemetry.get("use_lidar_udp", False):
        lines.append("#define USE_LIDAR_UDP")
    if telemetry.get("use_dual_core", False):
        lines.append("#define USE_DUAL_CORE")

    lines.append("")
    lines.append(f"#endif // {name}_CONFIG_H\n")
    return "\n".join(lines)
