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

#ifndef IMU_INTERFACE
#define IMU_INTERFACE

#include <sensor_msgs/msg/imu.h>

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

class IMUInterface
{
    protected:
        sensor_msgs__msg__Imu imu_msg_;
        const float g_to_accel_ = 9.81;
        const float mgauss_to_utesla_ = 0.1;
        const float utesla_to_tesla_ = 0.000001;

        float accel_cov_ = 0.02;
        float gyro_cov_ = 0.04;
        //doesn't get an accurate model but close enough
        // https://github.com/ENSTABretagneRobotics/razor_imu_9dof/blob/indigo-devel/nodes/imu_node.py
    public:
        IMUInterface()
        {
            imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");
        }

        virtual geometry_msgs__msg__Vector3 readAccelerometer() = 0;
        virtual geometry_msgs__msg__Vector3 readGyroscope() = 0;
        virtual bool init() = 0;

        sensor_msgs__msg__Imu getData()
        {
            imu_msg_.angular_velocity = readGyroscope();
            imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[8] = gyro_cov_;
            
            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

            return imu_msg_;
        }
};

#endif
