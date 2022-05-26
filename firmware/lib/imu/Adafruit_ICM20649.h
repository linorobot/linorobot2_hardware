/*!
 *  @file Adafruit_ICM20649.h
 *
 * 	I2C Driver for the Adafruit ICM20649 6-DoF Wide-Range Accelerometer and
 *Gyro library
 *
 * 	This is a library for the Adafruit ICM20649 breakout:
 * 	https://www.adafruit.com/products/4464
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_ICM20649_H
#define _ADAFRUIT_ICM20649_H

#include "Adafruit_ICM20X.h"

#define ICM20649_I2CADDR_DEFAULT 0x68 ///< ICM20X default i2c address

/** The accelerometer data range */
typedef enum {
  ICM20649_ACCEL_RANGE_4_G,
  ICM20649_ACCEL_RANGE_8_G,
  ICM20649_ACCEL_RANGE_16_G,
  ICM20649_ACCEL_RANGE_30_G,
} icm20649_accel_range_t;

/** The gyro data range */
typedef enum {
  ICM20649_GYRO_RANGE_500_DPS,
  ICM20649_GYRO_RANGE_1000_DPS,
  ICM20649_GYRO_RANGE_2000_DPS,
  ICM20649_GYRO_RANGE_4000_DPS,
} icm20649_gyro_range_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ST ICM20649 6-DoF Accelerometer and Gyro
 */
class Adafruit_ICM20649 : public Adafruit_ICM20X {
public:
  Adafruit_ICM20649();
  virtual ~Adafruit_ICM20649(){};
  bool begin_I2C(uint8_t i2c_addr = ICM20649_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensor_id = 0);

  icm20649_accel_range_t getAccelRange(void);
  void setAccelRange(icm20649_accel_range_t new_accel_range);

  icm20649_gyro_range_t getGyroRange(void);
  void setGyroRange(icm20649_gyro_range_t new_gyro_range);

private:
  void scaleValues(void);
};

#endif
