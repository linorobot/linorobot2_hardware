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
    name = spec["robot_name"].upper()
    kinematics = spec["kinematics"]
    geom = spec.get("geometry", {})
    motors = spec.get("motors", {})
    sensors = spec.get("sensors", {})
    pins = spec.get("pins", {})
    driver = motors.get("driver_type", "GENERIC_2_IN")

    lines = [
        "// Copyright (c) 2026 Thomas Chou, Paul Bouchier, Linorobot contributors",
        f"// Auto-generated configuration for {spec['robot_name']}",
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

    if bat_type in ["ADC_DIVIDER", "INA219"]:
        if "battery_capacity" in sensors:
            lines.append(f"#define BATTERY_CAP {sensors['battery_capacity']:.2f} // Ah (Capacity)")
        if "battery_min_voltage" in sensors:
            lines.append(f"#define BATTERY_MIN {sensors['battery_min_voltage']:.2f} // Volts (0% charge)")
        if "battery_max_voltage" in sensors:
            lines.append(f"#define BATTERY_MAX {sensors['battery_max_voltage']:.2f} // Volts (100% charge)")
        if "battery_nominal_voltage" in sensors:
            lines.append(f"#define BATTERY_NOMINAL {sensors['battery_nominal_voltage']:.2f} // Volts")
        if "battery_dip" in sensors:
            lines.append(f"#define BATTERY_DIP {sensors['battery_dip']:.2f} // Voltage drop alert ratio")

    if sensors.get("sonar", False):
        sonar = pins.get("sonar", {})
        lines.append("#define USE_SONAR")
        lines.append(f"#define TRIG_PIN {sonar.get('trig', 0)}")
        lines.append(f"#define ECHO_PIN {sonar.get('echo', 0)}")

    # WiFi & Network Settings (for WiFi micro-ROS transport or Syslog/OTA)
    transport = str(spec.get("transport", "")).upper()
    wifi = spec.get("wifi_settings", {})
    if "WIFI" in transport or wifi or spec.get("mcu", "").upper() in ["PICOW", "PICO2W"]:
        agent_ip_str = wifi.get("agent_ip", "192.168.1.100")
        ip_parts = [p.strip() for p in agent_ip_str.split(".")]
        ip_formatted = f"{{ {', '.join(ip_parts)} }}" if len(ip_parts) == 4 else "{ 192, 168, 1, 100 }"
        ssid = wifi.get("ssid", "YOUR_WIFI_SSID")
        password = wifi.get("password", "YOUR_WIFI_PASSWORD")
        agent_port = wifi.get("agent_port", 8888)

        lines.extend([
            "",
            "// WiFi & micro-ROS Agent Settings",
            f'#define WIFI_SSID "{ssid}"',
            f'#define WIFI_PASSWORD "{password}"',
            f"#define AGENT_IP {ip_formatted}",
            f"#define AGENT_PORT {agent_port}",
            "#define USE_WIFI",
            f'#define WIFI_AP_LIST {{ {{ "{ssid}", "{password}" }}, {{ NULL, NULL }} }}',
            "#define USE_ARDUINO_OTA",
            "#define USE_SYSLOG",
            f'#define SYSLOG_SERVER "{agent_ip_str}"',
            "#define SYSLOG_PORT 514",
            f'#define DEVICE_HOSTNAME "{spec.get("robot_name", "robot")}"',
            '#define APP_NAME "hardware"'
        ])

    lines.extend(["", "#endif", ""])
    return "\n".join(lines)


def generate_platformio_env(spec: Dict[str, Any], ros_distro: str = "jazzy") -> str:
    name = spec["robot_name"]
    mcu = spec["mcu"].upper()
    cfg_macro = f"USE_{name.upper()}_CONFIG"
    transport = str(spec.get("transport", "SERIAL")).upper()
    is_wifi = "WIFI" in transport

    wifi_line = "board_microros_transport = wifi\n" if is_wifi else ""
    wifi_flag = "    -D USE_STAY_CONNECTED\n" if is_wifi else ""

    # Micro-ROS platformio dependency resolution
    if str(ros_distro).lower() == "lyrical":
        microros_dep = "    https://github.com/hippo5329/micro_ros_platformio.git#feat/ros2-lyrical-support"
    else:
        microros_dep = "    ${env.lib_deps}"

    if mcu in ["PICO", "PICO2", "PICOW", "PICO2W"]:
        board_map = {
            "PICO": "rpipico",
            "PICO2": "rpipico2",
            "PICOW": "rpipicow",
            "PICO2W": "rpipico2w"
        }
        board = board_map.get(mcu, "rpipico")
        pico_macro = f"    -D {mcu}\n" if mcu in ["PICOW", "PICO2W"] else ""
        return f"""[env:{name}]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = {board}
monitor_port = /dev/ttyACM0
upload_port = /dev/ttyACM0
upload_protocol = picotool
board_microros_user_meta = atomic.meta
{wifi_line}lib_deps =
{microros_dep}
    https://github.com/gbr1/rp2040-encoder-library.git
build_flags =
    -I ../config
    -D PICO
{pico_macro}    -D {cfg_macro}
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
{microros_dep}
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
{microros_dep}
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
{microros_dep}
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
    robot_mass = geom.get("weight", 1.2)

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
  <xacro:property name="base_mass" value="{robot_mass:.2f}" />

  <xacro:property name="laser_pose">
    <origin xyz="0.05 0 0.08" rpy="0 0 0"/>
  </xacro:property>
</robot>
"""
