/*
    AK09918.h
    A library for Grove - IMU 9DOF(ICM20600 + AK09918)

    Copyright (c) 2018 seeed technology inc.
    Website    : www.seeed.cc
    Author     : Jerry Yip
    Create Time: 2018-06
    Version    : 0.1
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/




#ifndef __IMU_9DOF_AK09918_H__
#define __IMU_9DOF_AK09918_H__

#include "I2Cdev.h"

/***************************************************************
    AK09918 I2C Register
 ***************************************************************/
#define AK09918_I2C_ADDR    0x0c    // I2C address (Can't be changed)
#define AK09918_WIA1        0x00    // Company ID
#define AK09918_WIA2        0x01    // Device ID
#define AK09918_RSV1        0x02    // Reserved 1
#define AK09918_RSV2        0x03    // Reserved 2
#define AK09918_ST1         0x10    // DataStatus 1
#define AK09918_HXL         0x11    // X-axis data 
#define AK09918_HXH         0x12
#define AK09918_HYL         0x13    // Y-axis data
#define AK09918_HYH         0x14
#define AK09918_HZL         0x15    // Z-axis data
#define AK09918_HZH         0x16
#define AK09918_TMPS        0x17    // Dummy
#define AK09918_ST2         0x18    // Datastatus 2
#define AK09918_CNTL1       0x30    // Dummy
#define AK09918_CNTL2       0x31    // Control settings
#define AK09918_CNTL3       0x32    // Control settings

#define AK09918_SRST_BIT    0x01    // Soft Reset
#define AK09918_HOFL_BIT    0x08    // Sensor Over Flow
#define AK09918_DOR_BIT     0x02    // Data Over Run
#define AK09918_DRDY_BIT    0x01    // Data Ready

// #define AK09918_MEASURE_PERIOD 9    // Must not be changed
// AK09918 has following seven operation modes:
// (1) Power-down mode: AK09918 doesn't measure
// (2) Single measurement mode: measure when you call any getData() function
// (3) Continuous measurement mode 1: 10Hz, measure 10 times per second,
// (4) Continuous measurement mode 2: 20Hz, measure 20 times per second,
// (5) Continuous measurement mode 3: 50Hz, measure 50 times per second,
// (6) Continuous measurement mode 4: 100Hz, measure 100 times per second,
// (7) Self-test mode
enum AK09918_mode_type_t {
    AK09918_POWER_DOWN = 0x00,
    AK09918_NORMAL = 0x01,
    AK09918_CONTINUOUS_10HZ = 0x02,
    AK09918_CONTINUOUS_20HZ = 0x04,
    AK09918_CONTINUOUS_50HZ = 0x06,
    AK09918_CONTINUOUS_100HZ = 0x08,
    AK09918_SELF_TEST = 0x10, // ignored by switchMode() and initialize(), call selfTest() to use this mode
};

enum AK09918_err_type_t {
    AK09918_ERR_OK = 0,                 // ok
    AK09918_ERR_DOR = 1,                // data skipped
    AK09918_ERR_NOT_RDY = 2,            // not ready
    AK09918_ERR_TIMEOUT = 3,            // read/write timeout
    AK09918_ERR_SELFTEST_FAILED = 4,    // self test failed
    AK09918_ERR_OVERFLOW = 5,           // sensor overflow, means |x|+|y|+|z| >= 4912uT
    AK09918_ERR_WRITE_FAILED = 6,       // fail to write
    AK09918_ERR_READ_FAILED = 7,        // fail to read

};

class AK09918 {
  public:
    AK09918();

    // default to AK09918_CONTINUOUS_10HZ mode
    AK09918_err_type_t initialize(AK09918_mode_type_t mode = AK09918_NORMAL);
    // At AK09918_CONTINUOUS_** mode, check if data is ready to read
    AK09918_err_type_t isDataReady();
    // At AK09918_CONTINUOUS_** mode, check if data is skipped
    AK09918_err_type_t isDataSkip();
    // Get magnet data in uT
    AK09918_err_type_t getData(int32_t* axis_x, int32_t* axis_y, int32_t* axis_z);
    // Get raw I2C magnet data
    AK09918_err_type_t getRawData(int32_t* axis_x, int32_t* axis_y, int32_t* axis_z);


    // Return the working mode of AK09918
    AK09918_mode_type_t getMode();
    // Switch the working mode of AK09918
    AK09918_err_type_t switchMode(AK09918_mode_type_t mode);
    // Start a self-test, if pass, return AK09918_ERR_OK
    AK09918_err_type_t selfTest();
    // Reset AK09918
    AK09918_err_type_t reset();
    // Get details of AK09918_err_type_t
    String strError(AK09918_err_type_t err);
    // Get device ID
    uint16_t getDeviceID();



  private:
    uint8_t _getRawMode();
    uint8_t _addr;
    AK09918_mode_type_t _mode;
    uint8_t _buffer[16];

};


#endif // __IMU_9DOF_AK09918_H__
