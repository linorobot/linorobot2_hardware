// Copyright (c) 2026 Paul Bouchier
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

// This program is a simple calibration utility for the BNO085 IMU sensor. 
// It uses 6-DOF (Game Rotation Vector) to tare the sensor's orientation,
// effectively setting the current physical alignment of the mower as the
// zero reference for roll and pitch.

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <i2cdetect.h>

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
#include "SparkFun_BNO080_Arduino_Library.h"
#include "mag.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

BNO080 bno085;
int nextPrintTime;

void setup()
{
    Serial.begin(BAUDRATE);
    while (!Serial) delay(10); // Wait for Serial Monitor

#ifdef BOARD_INIT // board specific setup
    BOARD_INIT;
#endif

    initWifis();
    initOta();
    i2cdetect();  // default range from 0x03 to 0x77

    Serial.println(F("\n=============================================="));
    Serial.println(F("    BNO085 6-DOF (Mower) Tare Procedure       "));
    Serial.println(F("=============================================="));

    Wire.begin();
    Wire.setClock(400000);

    if (bno085.begin() == false) {
        Serial.println(F("[ERROR] BNO085 hardware not found."));
        while (1);
    }

    // 1. Wipe out any previous tare transformations to start fresh
    Serial.println(F("[ACTION] Clearing old orientation mappings..."));
    bno085.clearTare(); 
    delay(1000); 

    // 2. Enable Game Rotation Vector (6-DOF, ignores magnetometer)
    // Stream data at 50Hz (20ms interval)
    bno085.enableGameRotationVector(20); 
    Serial.println(F("[STATUS] Settle period: Let the mower sit still for 4 seconds..."));
    delay(4000); 

    // 3. Tare ALL axes using the Game Rotation Vector
    // The first argument 'false' forces the library to execute TARE_AXIS_ALL.
    Serial.println(F("[ACTION] Taring ALL axes (Roll, Pitch, and Yaw)..."));
    bno085.tareNow(false, TARE_GAME_ROTATION_VECTOR); 
    delay(500);

    // 4. Save this specific structural mounting position permanently to internal Flash
    Serial.println(F("[ACTION] Writing profile to non-volatile flash storage..."));
    bno085.saveTare();
    delay(500);

    Serial.println(F("[SUCCESS] Mower physical alignment complete!"));
    Serial.println(F("=============================================="));
    Serial.println(F("The mount orientation is now treated as 0° Roll and 0° Pitch."));
    Serial.println(F("==============================================\n"));
    
    // Freeze dynamic calibration to protect your newly tared baseline
    bno085.endCalibration();

    nextPrintTime = millis() + 200;

#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE;
#endif
    syslog(LOG_INFO, "BNO085 calibration done %lu", __FUNCTION__, millis());
}

void loop() {
    float roll, pitch, yaw;

    if (bno085.dataAvailable() == true) {
        // Read the tared 6-DOF values
        roll  = bno085.getRoll()  * RAD_TO_DEG;
        pitch = bno085.getPitch() * RAD_TO_DEG;
        yaw   = bno085.getYaw()   * RAD_TO_DEG;
    }

    if (millis() > nextPrintTime) {
        nextPrintTime = millis() + 200;

        Serial.print(F("Roll: "));
        Serial.print(roll, 2);
        Serial.print(F(" | Pitch: "));
        Serial.print(pitch, 2);
        Serial.print(F(" | Yaw: "));
        Serial.println(yaw, 2);
    } 

    runWifis();
    runOta();
}
