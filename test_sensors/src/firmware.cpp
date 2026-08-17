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
#include <micro_ros_utilities/string_utilities.h>
#include <stdio.h>
#include <i2cdetect.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include "config.h"
#include "syslog.h"
#include "imu.h"
#include "mag.h"
#include "wifis.h"
#include "ota.h"

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;

IMU imu;
MAG mag;

void setup()
{
    Serial.begin(BAUDRATE);
#ifdef BOARD_INIT // board specific setup
    BOARD_INIT;
#endif

    initWifis();
    initOta();

    delay(2000);
    Serial.println("\n==========================================");
    Serial.println("   Linorobot2 Hardware Sensor Diagnostics ");
    Serial.println("==========================================");
    Serial.println("Scanning I2C bus...");
    i2cdetect();  // default range from 0x03 to 0x77

    Serial.println("Initializing IMU & Magnetometer...");
    bool imu_ok = imu.init();
    if (!imu_ok)
    {
        Serial.println("[-] IMU initialization FAILED!");
    }
    else
    {
        Serial.println("[+] IMU initialized successfully.");
    }

    bool mag_ok = mag.init();
    if (!mag_ok)
    {
        Serial.println("[-] Magnetometer initialization FAILED or not detected.");
    }
    else
    {
        Serial.println("[+] Magnetometer initialized successfully.");
    }

#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE
#endif
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
    Serial.println("Starting real-time sensor stream (1 Hz)...\n");
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

    Serial.printf("ACC [m/s^2] X:%5.2f Y:%5.2f Z:%5.2f | GYR [rad/s] X:%5.2f Y:%5.2f Z:%5.2f | MAG [uT] X:%5.2f Y:%5.2f Z:%5.2f\n",
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
        mag_msg.magnetic_field.x * 1000000.0f, mag_msg.magnetic_field.y * 1000000.0f,
        mag_msg.magnetic_field.z * 1000000.0f
    );

    syslog(LOG_INFO, "ACC %5.2f %5.2f %5.2f GYR %5.2f %5.2f %5.2f MAG %5.2f %5.2f %5.2f",
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
        mag_msg.magnetic_field.x * 1000000.0f, mag_msg.magnetic_field.y * 1000000.0f,
        mag_msg.magnetic_field.z * 1000000.0f
    );

    runWifis();
    runOta();
}
