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

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>

/**
 * @brief Provides odometry calculations for a robot.
 * 
 * This class calculates the odometry of a robot based on its linear and angular velocities.
 */
class Odometry
{
    public:
        /**
         * @brief Constructor for the Odometry class.
         * 
         * Initializes the odometry message with default values.
         */
        Odometry();

        /**
         * @brief Update the odometry based on velocities.
         * 
         * @param vel_dt Time step for the velocity update.
         * @param linear_vel_x Linear velocity in the x direction.
         * @param linear_vel_y Linear velocity in the y direction.
         * @param angular_vel_z Angular velocity about the z axis.
         */
        void update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);

        /**
         * @brief Get the current odometry data.
         * 
         * @return The current odometry message.
         */
        nav_msgs__msg__Odometry getData();

    private:
        /**
         * @brief Convert Euler angles to a quaternion.
         * 
         * @param x Roll angle.
         * @param y Pitch angle.
         * @param z Yaw angle.
         * @param q Output quaternion.
         */
        const void euler_to_quat(float x, float y, float z, float* q);

        nav_msgs__msg__Odometry odom_msg_;  ///< Odometry message.
        float x_pos_;                       ///< Current x position.
        float y_pos_;                       ///< Current y position.
        float heading_;                     ///< Current heading or yaw.
};

#endif
