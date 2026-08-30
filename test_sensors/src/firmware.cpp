// Copyright (c) 2026 Thomas Chou
// Copyright (c) 2026 Paul Bouchier
// Copyright (c) 2026 Linorobot contributors
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
#include <math.h>
#include <i2cdetect.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>

#include "config.h"
#include "syslog.h"
#include "imu.h"
#include "mag.h"
#include "battery.h"
#include "range.h"
#include "wifis.h"
#include "ota.h"

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
#endif

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Range range_msg;

IMU imu;
MAG mag;

static unsigned long lastLogTime = 0;

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

    initBattery();
    initRange();

#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE
#endif
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
    Serial.println("Starting real-time sensor stream (50 Hz poll, 1 Hz output)...\n");
}

void loop()
{
    // Poll sensors at 50 Hz (20ms interval) to keep IMU state machines (e.g. BNO085) running smoothly
    delay(20);
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

    unsigned long currentTime = millis();
    if (currentTime - lastLogTime >= 1000)
    {
        lastLogTime = currentTime;

        // Convert quaternion to Euler angles (Roll, Pitch, Yaw) if orientation is available
        float qx = imu_msg.orientation.x;
        float qy = imu_msg.orientation.y;
        float qz = imu_msg.orientation.z;
        float qw = imu_msg.orientation.w;
        bool has_orientation = (qw != 0.0f || qx != 0.0f || qy != 0.0f || qz != 0.0f);

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        if (has_orientation)
        {
            float sinr_cosp = 2.0f * (qw * qx + qy * qz);
            float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
            roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

            float sinp = 2.0f * (qw * qy - qz * qx);
            if (fabs(sinp) >= 1.0f)
                pitch = copysign(90.0f, sinp);
            else
                pitch = asin(sinp) * RAD_TO_DEG;

            float siny_cosp = 2.0f * (qw * qz + qx * qy);
            float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
            yaw = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
        }

        if (has_orientation)
        {
            Serial.printf("ACC [m/s^2] X:%5.2f Y:%5.2f Z:%5.2f | GYR [rad/s] X:%5.2f Y:%5.2f Z:%5.2f | RPY [deg] R:%5.1f P:%5.1f Y:%5.1f\n",
                imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
                imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
                roll, pitch, yaw
            );
        }
        else
        {
            Serial.printf("ACC [m/s^2] X:%5.2f Y:%5.2f Z:%5.2f | GYR [rad/s] X:%5.2f Y:%5.2f Z:%5.2f | MAG [uT] X:%5.2f Y:%5.2f Z:%5.2f\n",
                imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
                imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
                mag_msg.magnetic_field.x * 1000000.0f, mag_msg.magnetic_field.y * 1000000.0f,
                mag_msg.magnetic_field.z * 1000000.0f
            );
        }

#if defined(BATTERY_PIN) || defined(USE_INA219) || defined(TRIG_PIN)
        Serial.printf("  BAT: %5.2fV | RANGE: %5.2fm\n", battery_msg.voltage, range_msg.range);
#endif

        syslog(LOG_INFO, "ACC %5.2f %5.2f %5.2f GYR %5.2f %5.2f %5.2f MAG %5.2f %5.2f %5.2f BAT %5.2fV",
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
            mag_msg.magnetic_field.x * 1000000.0f, mag_msg.magnetic_field.y * 1000000.0f,
            mag_msg.magnetic_field.z * 1000000.0f,
            battery_msg.voltage
        );
    }

    runWifis();
    runOta();
}
