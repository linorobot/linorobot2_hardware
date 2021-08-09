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

#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

// include the header of your new driver here similar to default_imu.h
#include "default_imu.h"

// now you can create a config constant that you can use in lino_base_config.h
#ifdef USE_GY85_IMU
    // pass your built in class to IMU macro
    #define IMU GY85IMU
#endif

#ifdef USE_MPU6050_IMU
    #define IMU MPU6050IMU
#endif

#ifdef USE_MPU9150_IMU
    #define IMU MPU9150IMU
#endif

#ifdef USE_MPU9250_IMU
    #define IMU MPU9250IMU
#endif

#ifdef USE_FAKE_IMU
    #define IMU FakeIMU
#endif

#endif

