// Copyright (c) 2021 Juan Miguel Jimeno
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

Odometry odometry;
IMU imu;
MAG mag;

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
}

void loop() {
    delay(1000);
    imu_msg = imu.getData();
    mag_msg = mag.getData();
#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg.magnetic_field.x -= mag_bias[0];
    mag_msg.magnetic_field.y -= mag_bias[1];
    mag_msg.magnetic_field.z -= mag_bias[2];
#endif
    battery_msg = getBattery();
    range_msg = getRange();
    printf("ACC %5.2f %5.2f %5.2f GYR %5.2f %5.2f %5.2f MAG %5.2f %5.2f %5.2f\n"
	   " BAT %5.2fV RANGE %5.2fm\n",
	   imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
	   imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.x,
	   mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z,
	   battery_msg.voltage, range_msg.range);
    syslog(LOG_INFO, "ACC %5.2f %5.2f %5.2f GYR %5.2f %5.2f %5.2f MAG %5.2f %5.2f %5.2f\n"
	   " BAT %5.2fV RANGE %5.2fm\n",
	   imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
	   imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.x,
	   mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z,
	   battery_msg.voltage, range_msg.range);
    runWifis();
    runOta();
}
