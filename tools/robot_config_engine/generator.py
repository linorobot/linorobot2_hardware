#!/usr/bin/env python3
"""
Linorobot2 Configuration & Artifact Generator
Generates C++ hardware headers, platformio.ini snippets, and URDF properties.
"""

from typing import Dict, Any


def generate_cpp_header(spec: Dict[str, Any]) -> str:
    name = spec["robot_name"]
    name_upper = name.upper()
    kinematics = spec["kinematics"]
    mcu = spec["mcu"].upper()
    geom = spec.get("geometry", {})
    motors = spec.get("motors", {})
    sensors = spec.get("sensors", {})
    pins = spec.get("pins", {})
    driver = motors.get("driver_type", "GENERIC_2_IN")

    base_macro = "LINO_BASE DIFFERENTIAL_DRIVE"
    if kinematics == "SKID_STEER":
        base_macro = "LINO_BASE SKID_STEER"
    elif kinematics == "MECANUM":
        base_macro = "LINO_BASE MECANUM"

    driver_macro = f"USE_{driver}_MOTOR_DRIVER"

    lines = [
        f"// Auto-generated Linorobot2 Configuration for {name}",
        f"// Microcontroller: {mcu} | Kinematics: {kinematics}",
        "#ifndef " + f"{name_upper}_CONFIG_H",
        "#define " + f"{name_upper}_CONFIG_H",
        "",
        f"#define {base_macro}",
        f"#define {driver_macro}",
        "",
        "// Kinematics & Wheel Geometry",
        f"#define WHEEL_DIAMETER {geom.get('wheel_diameter', 0.065)} // meters",
        f"#define LR_WHEELS_DISTANCE {geom.get('track_width', 0.20)} // meters",
    ]

    if "wheelbase" in geom:
        lines.append(f"#define FR_WHEELS_DISTANCE {geom.get('wheelbase')} // meters")

    lines.extend([
        "",
        "// Motor & Encoder Parameters",
        f"#define MOTOR_MAX_RPM {motors.get('max_rpm', 330)}",
        f"#define MAX_RPM_RATIO 0.85",
        f"#define MOTOR_OPERATING_VOLTAGE {motors.get('operating_voltage', 12.0)}",
        f"#define MOTOR_POWER_MAX_VOLTAGE {motors.get('max_voltage', 12.0)}",
        f"#define PWM_FREQUENCY {motors.get('pwm_frequency', 20000)}",
        f"#define PWM_BITS {motors.get('pwm_bits', 10)}",
        f"#define PWM_MAX ((1 << PWM_BITS) - 1)",
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
        f"#define MOTOR1_ENCODER_INV false",
        f"#define MOTOR2_ENCODER_INV false",
        f"#define MOTOR3_ENCODER_INV false",
        f"#define MOTOR4_ENCODER_INV false",
        "",
        "// Default PID Tuning Constants",
        "#define K_P 0.6",
        "#define K_I 0.8",
        "#define K_D 0.5",
        "",
        "// Pin Assignments",
    ])

    if "led" in pins:
        lines.append(f"#define LED_PIN {pins['led']}")

    # Motor Pins (All 4 motors defined; unused motors set to -1 for 2WD)
    num_active_motors = 2 if kinematics == "DIFFERENTIAL_DRIVE" else 4
    for i in range(1, 5):
        if i <= num_active_motors:
            m = pins.get(f"motor{i}", {})
            if driver == "BTS7960":
                lines.append(f"#define MOTOR{i}_PWM_R {m.get('pwm_r', 0)}")
                lines.append(f"#define MOTOR{i}_PWM_L {m.get('pwm_l', 0)}")
                if "en" in m:
                    lines.append(f"#define MOTOR{i}_EN {m.get('en')}")
            elif driver == "GENERIC_2_IN":
                lines.append(f"#define MOTOR{i}_PWM {m.get('pwm', 0)}")
                lines.append(f"#define MOTOR{i}_IN_A {m.get('in_a', 0)}")
                lines.append(f"#define MOTOR{i}_IN_B {m.get('in_b', 0)}")
            elif driver == "GENERIC_1_IN":
                lines.append(f"#define MOTOR{i}_PWM {m.get('pwm', 0)}")
                lines.append(f"#define MOTOR{i}_DIR {m.get('dir', 0)}")
            else:
                lines.append(f"#define MOTOR{i}_PWM {m.get('pwm', 0)}")
        else:
            if driver == "BTS7960":
                lines.append(f"#define MOTOR{i}_PWM_R -1")
                lines.append(f"#define MOTOR{i}_PWM_L -1")
            elif driver == "GENERIC_2_IN":
                lines.append(f"#define MOTOR{i}_PWM -1")
                lines.append(f"#define MOTOR{i}_IN_A -1")
                lines.append(f"#define MOTOR{i}_IN_B -1")
            elif driver == "GENERIC_1_IN":
                lines.append(f"#define MOTOR{i}_PWM -1")
                lines.append(f"#define MOTOR{i}_DIR -1")
            else:
                lines.append(f"#define MOTOR{i}_PWM -1")

    # Encoders (All 4 encoders defined; unused encoders set to -1 for 2WD)
    enc = pins.get("encoders", {})
    for i in range(1, 5):
        if i <= num_active_motors:
            lines.append(f"#define MOTOR{i}_ENCODER_A {enc.get(f'm{i}_a', 0)}")
            lines.append(f"#define MOTOR{i}_ENCODER_B {enc.get(f'm{i}_b', 0)}")
        else:
            lines.append(f"#define MOTOR{i}_ENCODER_A -1")
            lines.append(f"#define MOTOR{i}_ENCODER_B -1")

    # Sensors
    lines.append("")
    lines.append("// Sensor Configurations")
    imu_type = sensors.get("imu", "NONE")
    if imu_type != "NONE":
        lines.append(f"#define USE_{imu_type}_IMU")

    mag_type = sensors.get("mag", "NONE")
    if mag_type != "NONE":
        lines.append(f"#define USE_{mag_type}_MAG")
        if "mag_bias" in sensors:
            b = sensors["mag_bias"]
            lines.append(f"#define MAG_BIAS {{ {b[0]}, {b[1]}, {b[2]} }}")
    else:
        lines.append("#define USE_FAKE_MAG")

    bat_type = sensors.get("battery_monitor", "NONE")
    if bat_type == "ADC_DIVIDER":
        lines.append("#define USE_BATTERY_MONITOR")
        lines.append(f"#define BATTERY_PIN {pins.get('battery_pin', 0)}")
        lines.append("#define BATTERY_R1 30000.0")
        lines.append("#define BATTERY_R2 7500.0")
        lines.append("#define BATTERY_ADJUST(v) (v * (3.3 / 4095.0) * ((30000.0 + 7500.0) / 7500.0))")
    elif bat_type == "INA219":
        lines.append("#define USE_INA219")

    if sensors.get("sonar", False):
        sonar = pins.get("sonar", {})
        lines.append("#define USE_SONAR")
        lines.append(f"#define TRIG_PIN {sonar.get('trig', 0)}")
        lines.append(f"#define ECHO_PIN {sonar.get('echo', 0)}")

    lines.extend(["", "#endif", ""])
    return "\n".join(lines)


def generate_platformio_env(spec: Dict[str, Any]) -> str:
    name = spec["robot_name"]
    mcu = spec["mcu"].upper()
    cfg_macro = f"USE_{name.upper()}_CONFIG"
    transport = str(spec.get("transport", "SERIAL")).upper()
    is_wifi = "WIFI" in transport

    wifi_line = "board_microros_transport = wifi\n" if is_wifi else ""
    wifi_flag = "    -D USE_STAY_CONNECTED\n" if is_wifi else ""

    if mcu in ["PICO", "PICO2"]:
        board = "rpipico2" if mcu == "PICO2" else "rpipico"
        return f"""[env:{name}]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = {board}
monitor_port = /dev/ttyACM0
upload_port = /dev/ttyACM0
upload_protocol = picotool
board_microros_user_meta = atomic.meta
{wifi_line}lib_deps =
    ${{env.lib_deps}}
    https://github.com/gbr1/rp2040-encoder-library.git
build_flags =
    -I ../config
    -D PICO
    -D {cfg_macro}
{wifi_flag}"""
    elif mcu in ["ESP32", "GENDRV"]:
        return f"""[env:{name}]
platform = espressif32
board = nodemcu-32s
board_build.f_flash = 80000000L
board_build.flash_mode = qio
board_build.partitions = min_spiffs.csv
monitor_speed = 921600
monitor_port = /dev/ttyUSB0
upload_port = /dev/ttyUSB0
upload_protocol = esptool
{wifi_line}lib_deps =
    ${{env.lib_deps}}
    madhephaestus/ESP32Servo
    madhephaestus/ESP32Encoder
build_flags =
    -I ../config
    -D __PGMSPACE_H_
    -D {cfg_macro}
{wifi_flag}"""
    elif mcu == "ESP32S3":
        return f"""[env:{name}]
platform = espressif32
board = esp32-s3-devkitc-1
board_build.f_flash = 80000000L
board_build.flash_mode = qio
monitor_speed = 921600
monitor_port = /dev/ttyACM0
upload_port = /dev/ttyACM0
upload_protocol = esptool
{wifi_line}lib_deps =
    ${{env.lib_deps}}
    madhephaestus/ESP32Servo
    madhephaestus/ESP32Encoder
build_flags =
    -I ../config
    -D ARDUINO_USB_CDC_ON_BOOT
    -D __PGMSPACE_H_
    -D {cfg_macro}
{wifi_flag}"""
    elif mcu == "ESP32S2":
        return f"""[env:{name}]
platform = espressif32
board = esp32-s2-saola-1
monitor_speed = 921600
monitor_port = /dev/ttyACM0
upload_port = /dev/ttyACM0
upload_protocol = esptool
{wifi_line}lib_deps =
    ${{env.lib_deps}}
    madhephaestus/ESP32Servo
    madhephaestus/ESP32Encoder
build_flags =
    -I ../config
    -D ARDUINO_USB_CDC_ON_BOOT
    -D __PGMSPACE_H_
    -D {cfg_macro}
{wifi_flag}"""
    return ""

def generate_urdf_xacro(spec: Dict[str, Any]) -> str:
    geom = spec.get("geometry", {})
    wheel_d = geom.get("wheel_diameter", 0.065)
    wheel_r = wheel_d / 2.0
    track_w = geom.get("track_width", 0.20)
    wheel_pos_y = track_w / 2.0

    return f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="wheel_radius" value="{wheel_r:.4f}" />
  <xacro:property name="wheel_width" value="0.026" />
  <xacro:property name="wheel_pos_x" value="0.0" />
  <xacro:property name="wheel_pos_y" value="{wheel_pos_y:.4f}" />
  <xacro:property name="wheel_pos_z" value="-0.010" />
  <xacro:property name="wheel_mass" value="0.05" />

  <xacro:property name="base_length" value="{track_w * 1.2:.3f}" />
  <xacro:property name="base_width" value="{track_w * 0.9:.3f}" />
  <xacro:property name="base_height" value="0.070" />
  <xacro:property name="base_mass" value="1.2" />

  <xacro:property name="laser_pose">
    <origin xyz="0.05 0 0.08" rpy="0 0 0"/>
  </xacro:property>
</robot>
"""
