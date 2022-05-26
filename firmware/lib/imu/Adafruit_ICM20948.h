/*!
 *  @file Adafruit_ICM20948.h
 *
 * 	I2C Driver for the Adafruit ICM20948 9-DoF Accelerometer, Gyro, and
 *Magnetometer library
 *
 * 	This is a library for the Adafruit ICM20948 breakout:
 * 	https://www.adafruit.com/products/4554
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_ICM20948_H
#define _ADAFRUIT_ICM20948_H

#include "Adafruit_ICM20X.h"

#define ICM20948_I2CADDR_DEFAULT 0x69 ///< ICM20948 default i2c address
#define ICM20948_MAG_ID 0x09          ///< The chip ID for the magnetometer

#define ICM20948_UT_PER_LSB 0.15 ///< mag data LSB value (fixed)

#define AK09916_WIA2 0x01  ///< Magnetometer
#define AK09916_ST1 0x10   ///< Magnetometer
#define AK09916_HXL 0x11   ///< Magnetometer
#define AK09916_HXH 0x12   ///< Magnetometer
#define AK09916_HYL 0x13   ///< Magnetometer
#define AK09916_HYH 0x14   ///< Magnetometer
#define AK09916_HZL 0x15   ///< Magnetometer
#define AK09916_HZH 0x16   ///< Magnetometer
#define AK09916_ST2 0x18   ///< Magnetometer
#define AK09916_CNTL2 0x31 ///< Magnetometer
#define AK09916_CNTL3 0x32 ///< Magnetometer

/** The accelerometer data range */
typedef enum {
  ICM20948_ACCEL_RANGE_2_G,
  ICM20948_ACCEL_RANGE_4_G,
  ICM20948_ACCEL_RANGE_8_G,
  ICM20948_ACCEL_RANGE_16_G,
} icm20948_accel_range_t;

/** The gyro data range */
typedef enum {
  ICM20948_GYRO_RANGE_250_DPS,
  ICM20948_GYRO_RANGE_500_DPS,
  ICM20948_GYRO_RANGE_1000_DPS,
  ICM20948_GYRO_RANGE_2000_DPS,
} icm20948_gyro_range_t;

/**
 * @brief Data rates/modes for the embedded AsahiKASEI AK09916 3-axis
 * magnetometer
 *
 */
typedef enum {
  AK09916_MAG_DATARATE_SHUTDOWN = 0x0, ///< Stops measurement updates
  AK09916_MAG_DATARATE_SINGLE =
      0x1, ///< Takes a single measurement then switches to
           ///< AK09916_MAG_DATARATE_SHUTDOWN
  AK09916_MAG_DATARATE_10_HZ = 0x2,  ///< updates at 10Hz
  AK09916_MAG_DATARATE_20_HZ = 0x4,  ///< updates at 20Hz
  AK09916_MAG_DATARATE_50_HZ = 0x6,  ///< updates at 50Hz
  AK09916_MAG_DATARATE_100_HZ = 0x8, ///< updates at 100Hz
} ak09916_data_rate_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ST ICM2948 9-DoF Accelerometer, gyro, and magnetometer
 */
class Adafruit_ICM20948 : public Adafruit_ICM20X {
public:
  Adafruit_ICM20948();
  ~Adafruit_ICM20948(){};
  bool begin_I2C(uint8_t i2c_addr = ICM20948_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensor_id = 0);

  icm20948_accel_range_t getAccelRange(void);
  void setAccelRange(icm20948_accel_range_t new_accel_range);

  icm20948_gyro_range_t getGyroRange(void);
  void setGyroRange(icm20948_gyro_range_t new_gyro_range);

  ak09916_data_rate_t getMagDataRate(void);
  bool setMagDataRate(ak09916_data_rate_t rate);

private:
  uint8_t readMagRegister(uint8_t reg_addr);
  bool writeMagRegister(uint8_t reg_addr, uint8_t value);

  uint8_t getMagId(void);
  bool auxI2CBusSetupFailed(void);

  bool setupMag(void);
  void scaleValues(void);
};

#endif