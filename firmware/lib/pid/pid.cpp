/**
 * @file pid.cpp
 * @brief Implements the PID controller class.
 *
 * This file provides the implementation of the PID controller class methods.
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

#include "Arduino.h"
#include "pid.h"

/**
 * @brief Construct a new PID:: PID object.
 */
PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd)
{
}

/**
 * @brief Compute the PID output based on setpoint and measured value.
 */
double PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    // Calculate error
    error = setpoint - measured_value;

    // Update integral and derivative terms
    integral_ += error;
    derivative_ = error - prev_error_;

    // Reset integral and derivative if setpoint and error are zero
    if(setpoint == 0 && error == 0)
    {
        integral_ = 0;
        derivative_ = 0;
    }

    // Calculate PID output
    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    // Constrain PID output between min and max values
    return constrain(pid, min_val_, max_val_);
}

/**
 * @brief Update the PID constants.
 */
void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
