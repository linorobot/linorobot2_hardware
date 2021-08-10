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

#include "odometry.h"

Odometry::Odometry():
    x_pos_(0.0),
    y_pos_(0.0),
    heading_(0.0)
{
    odom_msg_.header.frame_id = micro_ros_string_utilities_set(odom_msg_.header.frame_id, "odom");
    odom_msg_.child_frame_id = micro_ros_string_utilities_set(odom_msg_.child_frame_id, "base_footprint");
}

void Odometry::update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
{
    float delta_heading = angular_vel_z * vel_dt; //radians
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; //m
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    float q[4];
    euler_to_quat(0, 0, heading_, q);

    //robot's position in x,y, and z
    odom_msg_.pose.pose.position.x = x_pos_;
    odom_msg_.pose.pose.position.y = y_pos_;
    odom_msg_.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    odom_msg_.pose.pose.orientation.x = (double) q[1];
    odom_msg_.pose.pose.orientation.y = (double) q[2];
    odom_msg_.pose.pose.orientation.z = (double) q[3];
    odom_msg_.pose.pose.orientation.w = (double) q[0];

    odom_msg_.pose.covariance[0] = 0.001;
    odom_msg_.pose.covariance[7] = 0.001;
    odom_msg_.pose.covariance[35] = 0.001;

    //linear speed from encoders
    odom_msg_.twist.twist.linear.x = linear_vel_x;
    odom_msg_.twist.twist.linear.y = linear_vel_y;
    odom_msg_.twist.twist.linear.z = 0.0;

    //angular speed from encoders
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;
    odom_msg_.twist.twist.angular.z = angular_vel_z;

    odom_msg_.twist.covariance[0] = 0.0001;
    odom_msg_.twist.covariance[7] = 0.0001;
    odom_msg_.twist.covariance[35] = 0.0001;
}

nav_msgs__msg__Odometry Odometry::getData()
{
    return odom_msg_;
} 

const void Odometry::euler_to_quat(float x, float y, float z, float* q) 
{
    //SOURCE:  https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_tf_publisher/micro-ros_tf_publisher.ino
    const float p_o = 3.14 / 180.0;
    float c1 = cos((y * p_o) / 2);
    float c2 = cos((z * p_o) / 2);
    float c3 = cos((x * p_o) / 2);

    float s1 = sin((y * p_o) / 2);
    float s2 = sin((z * p_o) / 2);
    float s3 = sin((x * p_o) / 2);

    q[0] = c1 * c2 * c3 - s1 * s2 * s3;
    q[1] = s1 * s2 * c3 + c1 * c2 * s3;
    q[2] = s1 * c2 * c3 + c1 * s2 * s3;
    q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}