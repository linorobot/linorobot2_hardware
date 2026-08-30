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

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

//include IMU base interface
#include "imu_interface.h"

//include sensor API headers
#include "I2Cdev.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MPU9250.h"
#include "QMI8658.h"

#include "syslog.h"

class GY85IMU: public IMUInterface 
{
    private:
        //constants specific to the sensor
        const float accel_scale_ = 1 / 256.0;
        const float gyro_scale_ = 1 / 14.375;

        // driver objects to be used
        ADXL345 accelerometer_;
        ITG3200 gyroscope_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        GY85IMU()
        {
            // accel_cov_ = 0.001; //you can overwrite the convariance values here
            // gyro_cov_ = 0.001; //you can overwrite the convariance values here
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            accelerometer_.initialize();
            ret = accelerometer_.testConnection();
            if(!ret)
                return false;

            gyroscope_.initialize();
            ret = gyroscope_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from accelerometer and return as a Vector3 object
            int16_t ax, ay, az;
            
            accelerometer_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            // here you can override readAccelerometer function and use the sensor's driver API
            // to grab the data from gyroscope and return as a Vector3 object
            int16_t gx, gy, gz;

            gyroscope_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};


class MPU6050IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU6050 accelgyro_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU6050IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelgyro_.initialize();
            ret = accelgyro_.testConnection();
            if(!ret)
                return false;

            accelgyro_.CalibrateAccel();
            accelgyro_.CalibrateGyro();
            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelgyro_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            accelgyro_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};

class MPU9250IMU: public IMUInterface 
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;

        MPU9250 accelgyro_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        MPU9250IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            accelgyro_.initialize();
            ret = accelgyro_.testConnection();
            if(!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            int16_t ax, ay, az;
            
            accelgyro_.getAcceleration(&ax, &ay, &az);

            accel_.x = ax * (double) accel_scale_ * g_to_accel_;
            accel_.y = ay * (double) accel_scale_ * g_to_accel_;
            accel_.z = az * (double) accel_scale_ * g_to_accel_;

            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            accelgyro_.getRotation(&gx, &gy, &gz);

            gyro_.x = gx * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.y = gy * (double) gyro_scale_ * DEG_TO_RAD;
            gyro_.z = gz * (double) gyro_scale_ * DEG_TO_RAD;

            return gyro_;
        }
};

class FakeIMU: public IMUInterface 
{
    private:
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        FakeIMU()
        {
        }

        bool startSensor() override
        {
            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            return gyro_;
        }
};

class QMI8658IMU: public IMUInterface 
{
    private:
        QMI8658 qmi8658_;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        QMI8658IMU()
        {
        }

        bool startSensor() override
        {
	    if (qmi8658_.begin() == 0){
	        // Serial.println("qmi8658_init fail");
	        return false;
	    }
	    return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
	    float ac[3];
            qmi8658_.read_acc(ac);
            accel_.x = ac[0];
            accel_.y = ac[1];
            accel_.z = ac[2];
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
	    float gy[3];
            qmi8658_.read_gyro(gy);
            gyro_.x = gy[0];
            gyro_.y = gy[1];
            gyro_.z = gy[2];
            return gyro_;
        }

        sensor_msgs__msg__Imu getData()
        {
            float ac[3], gy[3];
            qmi8658_.read_sensor_data(ac, gy);
            accel_.x = ac[0];
            accel_.y = ac[1];
            accel_.z = ac[2];

            gyro_.x = gy[0] - gyro_cal_.x;
            gyro_.y = gy[1] - gyro_cal_.y;
            gyro_.z = gy[2] - gyro_cal_.z;

            if (gyro_.x > -0.01 && gyro_.x < 0.01) gyro_.x = 0;
            if (gyro_.y > -0.01 && gyro_.y < 0.01) gyro_.y = 0;
            if (gyro_.z > -0.01 && gyro_.z < 0.01) gyro_.z = 0;

            imu_msg_.angular_velocity = gyro_;
            imu_msg_.angular_velocity_covariance[0] = gyro_cov[0];
            imu_msg_.angular_velocity_covariance[4] = gyro_cov[1];
            imu_msg_.angular_velocity_covariance[8] = gyro_cov[2];

            imu_msg_.linear_acceleration = accel_;
            imu_msg_.linear_acceleration_covariance[0] = accel_cov[0];
            imu_msg_.linear_acceleration_covariance[4] = accel_cov[1];
            imu_msg_.linear_acceleration_covariance[8] = accel_cov[2];

            return imu_msg_;
        }
};

// Note: Sparkfun library redefines I2C_BUFFER_LENGTH, so we undefine it for this class
#ifdef I2C_BUFFER_LENGTH
#undef I2C_BUFFER_LENGTH
#endif
#include <SparkFun_BNO080_Arduino_Library.h>

class BNO085IMU: public IMUInterface 
{
    private:
        BNO080 bno085_;
        const int bno085UpdateRateMs = 20;   // 50Hz update rate (standard for ROS IMU messages)
        const float accel_cov_ = 0.01;
        const float gyro_cov_ = 0.001;
        const float ori_xy_cov_ = 0.01;
        const float ori_z_cov_ = 0.05;

        unsigned long nextUpdateTime = 0;

        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

        // State Machine Enumeration
        enum IMUState {
        STATE_DISCONNECTED,
        STATE_SEND_CONFIG,
        STATE_CHECK_RESET,
        STATE_VALIDATE_STREAM,
        STATE_RUNNING
        };

        // Global State Variables for the IMU State Machine
        IMUState imuState = STATE_DISCONNECTED;
        unsigned long stateTimer = 0;
        int validPacketCount = 0;
        int configAttempts = 0;

    public:
        BNO085IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            if (bno085_.begin() == 0){
                // Serial.println("bno085_init fail");
                syslog(LOG_ERR, "%s BNO085 IMU init fail %lu", __FUNCTION__, millis());
                imuState = STATE_DISCONNECTED;
                return false;
            }
            syslog(LOG_INFO, "%s BNO085 IMU init success %lu", __FUNCTION__, millis());
            imuState = STATE_SEND_CONFIG;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            accel_.x = bno085_.getAccelX();
            accel_.y = bno085_.getAccelY();
            accel_.z = bno085_.getAccelZ();
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            gyro_.x = bno085_.getGyroX() * DEG_TO_RAD;
            gyro_.y = bno085_.getGyroY() * DEG_TO_RAD;
            gyro_.z = bno085_.getGyroZ() * DEG_TO_RAD;
            return gyro_;
        }

        sensor_msgs__msg__Imu getData()
        {
            if (!runIMUStateMachine()) {
                logImuDataUnavailable();
                return imu_msg_;
            }
            imu_msg_.angular_velocity = readGyroscope();

            if(imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01 )
                imu_msg_.angular_velocity.x = 0;

            if(imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01 )
                imu_msg_.angular_velocity.y = 0;

            if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01 )
                imu_msg_.angular_velocity.z = 0;

            imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[8] = gyro_cov_;

            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

            imu_msg_.orientation.x = bno085_.getQuatI();
            imu_msg_.orientation.y = bno085_.getQuatJ();
            imu_msg_.orientation.z = bno085_.getQuatK();
            imu_msg_.orientation.w = bno085_.getQuatReal();

            imu_msg_.orientation_covariance[0] = ori_xy_cov_;
            imu_msg_.orientation_covariance[4] = ori_xy_cov_;
            imu_msg_.orientation_covariance[8] = ori_z_cov_;

            return imu_msg_;
        }

        // The BNO085 IMU has an I2C interface that doesn't work well with the ESP32.
        // To work around this, we implement a state machine to manage the IMU's initialization and data streaming.
        bool runIMUStateMachine()
        {
        switch (imuState) {

          case STATE_DISCONNECTED:
            if (millis() - stateTimer >= 500) {
                stateTimer = millis();
                if (bno085_.begin() == true) {
                syslog(LOG_INFO, "%s [I2C] Link achieved. Moving to configuration step.", __FUNCTION__);
                imuState = STATE_SEND_CONFIG;
                }
            }
            break;

          case STATE_SEND_CONFIG:
            configAttempts++;
            syslog(LOG_INFO, "%s [CONFIG] Transmitting 6-DOF Profile (Try # %d)...", __FUNCTION__, configAttempts);

            bno085_.hasReset(); // Clear historical reset tracking bits

            bno085_.enableGameRotationVector(bno085UpdateRateMs);
            bno085_.enableGyro(bno085UpdateRateMs);
            bno085_.enableAccelerometer(bno085UpdateRateMs);
            bno085_.endCalibration(); // Anchor our saved physical Tare profile

            stateTimer = millis();
            imuState = STATE_CHECK_RESET;
            break;

          case STATE_CHECK_RESET:
            if (millis() - stateTimer >= 400) {
                if (bno085_.hasReset()) {
                syslog(LOG_INFO, "%s [WARNING] Reset flag caught during parsing. Cyclical retry...", __FUNCTION__);
                imuState = STATE_SEND_CONFIG;
                } else {
                syslog(LOG_INFO, "%s [VALIDATION] Checking telemetry stream integrity...", __FUNCTION__);
                validPacketCount = 0;
                stateTimer = millis();
                imuState = STATE_VALIDATE_STREAM;
                }
            }
            break;

          case STATE_VALIDATE_STREAM:
            // Note: myIMU.dataAvailable() internally executes and evaluates getReadings()
            if (bno085_.dataAvailable() == true) {
                // 2. Verify the active packet type matches our navigation profile.
                // This isolates the actual 6-DOF frame and strips out 0.06 diagnostic responses.
                if (bno085_.getReadings() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
                    float testYaw = bno085_.getYaw();
                    if (!isnan(testYaw)) {
                        validPacketCount++;
                    }
                }
            }

            if (validPacketCount >= 10) {
                syslog(LOG_INFO, "%s [SUCCESS] Navigation data streams verified numeric!", __FUNCTION__);
                imuState = STATE_RUNNING;
            }
            else if (millis() - stateTimer >= 1500) {
                syslog(LOG_INFO, "%s [TIMEOUT] Stream unpopulated or stuck on NaN. Soft resetting...", __FUNCTION__);
                bno085_.softReset();
                stateTimer = millis();
                imuState = STATE_CHECK_RESET;
            }
            break;

          case STATE_RUNNING:
            if (bno085_.dataAvailable() == true) {
                // Enforce the layout check in real-time execution to prevent background packets from causing spikes
                if (bno085_.getReadings() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
// Uncomment the following line to log the IMU data to syslog for debugging purposes
// #define DEBUG_BNO085
#ifdef DEBUG_BNO085
                    float roll = bno085_.getRoll() * RAD_TO_DEG;
                    float pitch = bno085_.getPitch() * RAD_TO_DEG;
                    float yaw = bno085_.getYaw() * RAD_TO_DEG;

                    if (millis() >= nextUpdateTime) {
                        syslog(LOG_INFO, "%s BNO085 IMU data read complete %lu, roll: %0.2f, pitch: %0.2f, yaw: %0.2f", __FUNCTION__, millis(), roll, pitch, yaw);
                        nextUpdateTime = millis() + 500;
                    }
#endif
                }
                return true;  // IMU is fully initialized and running and we can use its data
            } else {
                syslog(LOG_INFO, "%s [WARNING] Data stream interrupted. Revalidating...", __FUNCTION__);
                imuState = STATE_DISCONNECTED;
            }
            break;
        }
        return false;  // if we don't return true from STATE_RUNNING, we are not fully initialized yet
        }

        void logImuDataUnavailable()
        {
            static unsigned long lastLogTime = 0;
            unsigned long currentTime = millis();
            if (currentTime - lastLogTime >= 1000) { // Log every 1 second
                syslog(LOG_INFO, "%s BNO085 IMU data not available %lu", __FUNCTION__, currentTime);
                lastLogTime = currentTime;
            }
        }
};

#endif
//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//HMC8553L https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf


//MPU9150 https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//MPU9250 https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf

//http://www.sureshjoshi.com/embedded/invensense-imus-what-to-know/
//https://stackoverflow.com/questions/19161872/meaning-of-lsb-unit-and-unit-lsb
