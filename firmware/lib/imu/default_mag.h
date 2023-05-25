// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
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

#ifndef DEFAULT_MAG
#define DEFAULT_MAG

//include MAG base interface
#include "mag_interface.h"

//include sensor API headers
#include "I2Cdev.h"
#include "HMC5883L.h"
#include "AK8963.h"
#include "AK8975.h"
#include "AK09918.h"
#include "QMC5883L.h"

class HMC5883LMAG: public MAGInterface
{
    private:
        //constants specific to the sensor

        // driver objects to be used
        HMC5883L magnetometer_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 mag_;

    public:
        HMC5883LMAG()
        {
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            magnetometer_.initialize();
            ret = magnetometer_.testConnection();
            if (!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            // here you can override readMagnetometer function and use the sensor's driver API
            // to grab the data from magnetometer and return as a Vector3 object
            int16_t ax, ay, az;

            magnetometer_.getHeading(&ax, &ay, &az);

            mag_.x = ax;
            mag_.y = ay;
            mag_.z = az;

            return mag_;
        }
};

class AK8963MAG: public MAGInterface
{
    private:
        //constants specific to the sensor

        // driver objects to be used
        AK8963 magnetometer_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 mag_;

    public:
        AK8963MAG()
        {
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            magnetometer_.initialize();
            ret = magnetometer_.testConnection();
            if (!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            // here you can override readMagnetometer function and use the sensor's driver API
            // to grab the data from magnetometer and return as a Vector3 object
            int16_t ax, ay, az;

            magnetometer_.getHeading(&ax, &ay, &az);

            mag_.x = ax;
            mag_.y = ay;
            mag_.z = az;

            return mag_;
        }
};

class AK8975MAG: public MAGInterface
{
    private:
        //constants specific to the sensor

        // driver objects to be used
        AK8975 magnetometer_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 mag_;

    public:
        AK8975MAG()
        {
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            magnetometer_.initialize();
            ret = magnetometer_.testConnection();
            if (!ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            // here you can override readMagnetometer function and use the sensor's driver API
            // to grab the data from magnetometer and return as a Vector3 object
            int16_t ax, ay, az;

            magnetometer_.getHeading(&ax, &ay, &az);

            mag_.x = ax;
            mag_.y = ay;
            mag_.z = az;

            return mag_;
        }
};

class AK09918MAG: public MAGInterface
{
    private:
        //constants specific to the sensor

        // driver objects to be used
        AK09918 magnetometer_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 mag_;

    public:
        AK09918MAG()
        {
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            ret = magnetometer_.initialize();
            if (ret)
                return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            // here you can override readMagnetometer function and use the sensor's driver API
            // to grab the data from magnetometer and return as a Vector3 object
            int16_t ax, ay, az;

            magnetometer_.getHeading(&ax, &ay, &az);

            mag_.x = ax;
            mag_.y = ay;
            mag_.z = az;

            return mag_;
        }
};

class QMC5883LMAG: public MAGInterface
{
    private:
        //constants specific to the sensor

        // driver objects to be used
        QMC5883L magnetometer_;

        // returned vector for sensor reading
        geometry_msgs__msg__Vector3 mag_;

    public:
        QMC5883LMAG()
        {
        }

        bool startSensor() override
        {
            // here you can override startSensor() function and use the sensor's driver API
            // to initialize and test the sensor's connection during boot time
            Wire.begin();
            bool ret;
            magnetometer_.initialize();

            return true;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            // here you can override readMagnetometer function and use the sensor's driver API
            // to grab the data from magnetometer and return as a Vector3 object
            int16_t ax, ay, az;

            magnetometer_.getHeading(&ax, &ay, &az);
            mag_.x = ax;
            mag_.y = ay;
            mag_.z = az;

            return mag_;
        }
};

class FakeMAG: public MAGInterface
{
    private:
        geometry_msgs__msg__Vector3 mag_;

    public:
        FakeMAG()
        {
        }

        bool startSensor() override
        {
            return true;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            return mag_;
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
