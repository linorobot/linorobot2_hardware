/**
 * @file pid.h
 * @brief Defines the PID controller class.
 *
 * This file provides the definition of the PID controller class, which is used to compute
 * control outputs based on setpoints and measured values.
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

#ifndef PID_H
#define PID_H

#include "Arduino.h"

/**
 * @brief A Proportional-Integral-Derivative (PID) controller class.
 */
class PID
{
    public:
        /**
         * @brief Construct a new PID object.
         * 
         * @param min_val Minimum output value.
         * @param max_val Maximum output value.
         * @param kp Proportional constant.
         * @param ki Integral constant.
         * @param kd Derivative constant.
         */
        PID(float min_val, float max_val, float kp, float ki, float kd);

        /**
         * @brief Compute the PID output.
         * 
         * @param setpoint Desired setpoint.
         * @param measured_value Current measured value.
         * @return double PID output.
         */
        double compute(float setpoint, float measured_value);

        /**
         * @brief Update the PID constants.
         * 
         * @param kp New proportional constant.
         * @param ki New integral constant.
         * @param kd New derivative constant.
         */
        void updateConstants(float kp, float ki, float kd);

    private:
        float min_val_;
        float max_val_;
        float kp_;
        float ki_;
        float kd_;
        double integral_;
        double derivative_;
        double prev_error_;
};

#endif
