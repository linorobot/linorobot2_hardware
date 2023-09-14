/**
 * @file motor.h
 * @brief Configuration header for selecting motor driver.
 * 
 * This header allows users to select which motor driver they want to use.
 * By defining a specific macro, the corresponding motor driver class will be selected.
 * 
 * To add a new motor config, include the header of your new driver and then create a config constant that you can
 * use in lino_base_config.h. Pass your built-in class to Motor macro with a conditional define
 */


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

#ifndef MOTOR_H
#define MOTOR_H

#include "default_motor.h" // Default motor driver definitions.

// Define the motor driver based on user's macro definition.
#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
    #define Motor Generic2  ///< Use Generic2 motor driver.
#endif

#ifdef USE_GENERIC_1_IN_MOTOR_DRIVER
    #define Motor Generic1  ///< Use Generic1 motor driver.
#endif

#ifdef USE_BTS7960_MOTOR_DRIVER
    #define Motor BTS7960   ///< Use BTS7960 motor driver.
#endif

#ifdef USE_ESC_MOTOR_DRIVER
    #define Motor ESC       ///< Use ESC motor driver.
#endif

#endif  // MOTOR_H
