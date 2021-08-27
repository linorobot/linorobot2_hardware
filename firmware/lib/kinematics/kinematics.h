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

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

#define RPM_TO_RPS 1/60

class Kinematics
{
    public:
        enum base {DIFFERENTIAL_DRIVE, SKID_STEER, MECANUM};

        base base_platform_;

        struct rpm
        {
            float motor1;
            float motor2;
            float motor3;
            float motor4;
        };
        
        struct velocities
        {
            float linear_x;
            float linear_y;
            float angular_z;
        };

        struct pwm
        {
            int motor1;
            int motor2;
            int motor3;
            int motor4;
        };
        Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                   float motor_operating_voltage, float motor_power_max_voltage,
                   float wheel_diameter, float wheels_y_distance);
        velocities getVelocities(float rpm1, float rpm2, float rpm3, float rpm4);
        rpm getRPM(float linear_x, float linear_y, float angular_z);
        float getMaxRPM();

    private:
        rpm calculateRPM(float linear_x, float linear_y, float angular_z);
        int getTotalWheels(base robot_base);

        float max_rpm_;
        float wheels_y_distance_;
        float pwm_res_;
        float wheel_circumference_;
        int total_wheels_;
};

#endif