// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "syslog.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "mag.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "battery.h"
#include "range.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Range range_msg;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;
MAG mag;
unsigned total_motors = 4;

void setup()
{
#ifdef BOARD_INIT // board specific setup
    BOARD_INIT;
#endif

    Serial.begin(BAUDRATE);
    pinMode(LED_PIN, OUTPUT);
#ifdef SDA_PIN // specify I2C pins
#ifdef ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
#else // teensy
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
#endif
#endif
    initWifis();
    initOta();

    imu.init();
    mag.init();
    initBattery();
    initRange();

    if(Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }
    motor1_encoder.getRPM();
    motor2_encoder.getRPM();
    motor3_encoder.getRPM();
    motor4_encoder.getRPM();
}

void loop() {
    static unsigned tk = 0; // tick
    const unsigned run_time = 8; // run time of each motor
    const unsigned cycle = run_time * total_motors;
    unsigned current_motor = tk / run_time % total_motors;
    unsigned direction = tk / cycle % 2; // 0 forward, 1 reverse
    const int pwm_max = (1 << PWM_BITS) - 1;
    static float max_rpm, stopping;

    digitalWrite(LED_PIN, direction ? LOW : HIGH);
    motor1_controller.spin((current_motor == 0) ? (direction ? -pwm_max : pwm_max) : 0);
    motor2_controller.spin((current_motor == 1) ? (direction ? -pwm_max : pwm_max) : 0);
    motor3_controller.spin((current_motor == 2) ? (direction ? -pwm_max : pwm_max) : 0);
    motor4_controller.spin((current_motor == 3) ? (direction ? -pwm_max : pwm_max) : 0);

    delay(1000);
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();
    if (current_motor == 0 && tk % run_time == run_time - 1) max_rpm = current_rpm1;
    if (current_motor == 1 && tk % run_time == 0) stopping = current_rpm1;
    if (current_motor == 1 && tk % run_time == run_time - 1) max_rpm = current_rpm2;
    if (total_motors == 2 && current_motor == 0 && tk % run_time == 0) stopping = current_rpm2;
    if (current_motor == 2 && tk % run_time == 0) stopping = current_rpm2;
    if (current_motor == 2 && tk % run_time == run_time - 1) max_rpm = current_rpm3;
    if (current_motor == 3 && tk % run_time == 0) stopping = current_rpm3;
    if (current_motor == 3 && tk % run_time == run_time - 1) max_rpm = current_rpm4;
    if (total_motors == 4 && current_motor == 0 && tk % run_time == 0) stopping = current_rpm4;
    if (tk && tk % run_time == 0) {
        Kinematics::velocities max_linear = kinematics.getVelocities(max_rpm, max_rpm, max_rpm, max_rpm);
	Kinematics::velocities max_angular = kinematics.getVelocities(-max_rpm, max_rpm,-max_rpm, max_rpm);
	printf("MOTOR%d SPEED %6.2f m/s %6.2f rad/s STOP %6.3f m\n", current_motor ? current_motor : total_motors,
	       max_linear.linear_x, max_angular.angular_z, max_linear.linear_x * stopping / max_rpm);
	syslog(LOG_INFO, "MOTOR%d SPEED %6.2f m/s %6.2f rad/s STOP %6.3f m\n", current_motor ? current_motor : total_motors,
	       max_linear.linear_x, max_angular.angular_z, max_linear.linear_x * stopping / max_rpm);
    }
    printf("MOTOR%d %s RPM %8.1f %8.1f %8.1f %8.1f\n",
	   current_motor + 1, direction ? "REV" : "FWD",
	   current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    syslog(LOG_INFO, "MOTOR%d %s RPM %8.1f %8.1f %8.1f %8.1f\n",
	   current_motor + 1, direction ? "REV" : "FWD",
	   current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    tk++;
    runWifis();
    runOta();
}
