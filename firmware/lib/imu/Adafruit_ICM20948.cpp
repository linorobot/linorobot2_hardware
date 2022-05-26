/*!   @file Adafruit_ICM20948.cpp
 */
#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_ICM20948.h"
#include "Adafruit_ICM20X.h"

/*!
 *    @brief  Instantiates a new ICM20948 class!
 */

Adafruit_ICM20948::Adafruit_ICM20948(void) {}
/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            An optional parameter to set the sensor ids to differentiate
 * similar sensors The passed value is assigned to the accelerometer and the
 * gyro get +1 and the temperature sensor +2.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_ICM20948::begin_I2C(uint8_t i2c_address, TwoWire *wire,
                                  int32_t sensor_id) {

  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    Serial.println("I2C begin Failed");
    return false;
  }
  bool init_success = _init(sensor_id);
  if (!setupMag()) {
    Serial.println("failed to setup mag");
    return false;
  }

  return init_success;
}

// A million thanks to the SparkFun folks for their library that I pillaged to
// write this method! See their Arduino library here:
// https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
bool Adafruit_ICM20948::auxI2CBusSetupFailed(void) {
  // check aux I2C bus connection by reading the magnetometer chip ID
  bool aux_i2c_setup_failed = true;
  for (int i = 0; i < I2C_MASTER_RESETS_BEFORE_FAIL; i++) {
    if (getMagId() != ICM20948_MAG_ID) {
      resetI2CMaster();
    } else {
      aux_i2c_setup_failed = false;
      break;
    }
  }
  return aux_i2c_setup_failed;
}

uint8_t Adafruit_ICM20948::getMagId(void) {
  // verify the magnetometer id
  return readExternalRegister(0x8C, 0x01);
}

bool Adafruit_ICM20948::setupMag(void) {
  uint8_t buffer[2];

  setI2CBypass(false);

  configureI2CMaster();

  enableI2CMaster(true);

  if (auxI2CBusSetupFailed()) {
    return false;
  }

  // set mag data rate
  if (!setMagDataRate(AK09916_MAG_DATARATE_100_HZ)) {
    Serial.println("Error setting magnetometer data rate on external bus");
    return false;
  }

  // TODO: extract method
  // Set up Slave0 to proxy Mag readings
  _setBank(3);
  // set up slave0 to proxy reads to mag
  buffer[0] = ICM20X_B3_I2C_SLV0_ADDR;
  buffer[1] = 0x8C;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  buffer[0] = ICM20X_B3_I2C_SLV0_REG;
  buffer[1] = 0x10;
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  buffer[0] = ICM20X_B3_I2C_SLV0_CTRL;
  buffer[1] = 0x89; // enable, read 9 bytes
  if (!i2c_dev->write(buffer, 2)) {
    return false;
  }

  return true;
}

/**
 * @brief
 *
 * @param slv_addr
 * @param mag_reg_addr
 * @param num_finished_checks
 * @return uint8_t
 */
uint8_t Adafruit_ICM20948::readMagRegister(uint8_t mag_reg_addr) {
  return readExternalRegister(0x8C, mag_reg_addr);
}

bool Adafruit_ICM20948::writeMagRegister(uint8_t mag_reg_addr, uint8_t value) {
  return writeExternalRegister(0x0C, mag_reg_addr, value);
}

void Adafruit_ICM20948::scaleValues(void) {

  icm20948_gyro_range_t gyro_range = (icm20948_gyro_range_t)current_gyro_range;
  icm20948_accel_range_t accel_range =
      (icm20948_accel_range_t)current_accel_range;

  float accel_scale = 1.0;
  float gyro_scale = 1.0;

  if (gyro_range == ICM20948_GYRO_RANGE_250_DPS)
    gyro_scale = 131.0;
  if (gyro_range == ICM20948_GYRO_RANGE_500_DPS)
    gyro_scale = 65.5;
  if (gyro_range == ICM20948_GYRO_RANGE_1000_DPS)
    gyro_scale = 32.8;
  if (gyro_range == ICM20948_GYRO_RANGE_2000_DPS)
    gyro_scale = 16.4;

  if (accel_range == ICM20948_ACCEL_RANGE_2_G)
    accel_scale = 16384.0;
  if (accel_range == ICM20948_ACCEL_RANGE_4_G)
    accel_scale = 8192.0;
  if (accel_range == ICM20948_ACCEL_RANGE_8_G)
    accel_scale = 4096.0;
  if (accel_range == ICM20948_ACCEL_RANGE_16_G)
    accel_scale = 2048.0;

  gyroX = rawGyroX / gyro_scale;
  gyroY = rawGyroY / gyro_scale;
  gyroZ = rawGyroZ / gyro_scale;

  accX = rawAccX / accel_scale;
  accY = rawAccY / accel_scale;
  accZ = rawAccZ / accel_scale;

  magX = rawMagX * ICM20948_UT_PER_LSB;
  magY = rawMagY * ICM20948_UT_PER_LSB;
  magZ = rawMagZ * ICM20948_UT_PER_LSB;
}

/**************************************************************************/
/*!
    @brief Get the accelerometer's measurement range.
    @returns The accelerometer's measurement range (`icm20948_accel_range_t`).
*/
icm20948_accel_range_t Adafruit_ICM20948::getAccelRange(void) {
  return (icm20948_accel_range_t)readAccelRange();
}

/**************************************************************************/
/*!

    @brief Sets the accelerometer's measurement range.
    @param  new_accel_range
            Measurement range to be set. Must be an
            `icm20948_accel_range_t`.
*/
void Adafruit_ICM20948::setAccelRange(icm20948_accel_range_t new_accel_range) {
  writeAccelRange((uint8_t)new_accel_range);
}

/**************************************************************************/
/*!
    @brief Get the gyro's measurement range.
    @returns The gyro's measurement range (`icm20948_gyro_range_t`).
*/
icm20948_gyro_range_t Adafruit_ICM20948::getGyroRange(void) {
  return (icm20948_gyro_range_t)readGyroRange();
}

/**************************************************************************/
/*!

    @brief Sets the gyro's measurement range.
    @param  new_gyro_range
            Measurement range to be set. Must be an
            `icm20948_gyro_range_t`.
*/
void Adafruit_ICM20948::setGyroRange(icm20948_gyro_range_t new_gyro_range) {
  writeGyroRange((uint8_t)new_gyro_range);
}

/**
 * @brief Get the current magnetometer measurement rate
 *
 * @return ak09916_data_rate_t the current rate
 */
ak09916_data_rate_t Adafruit_ICM20948::getMagDataRate(void) {

  uint8_t raw_mag_rate = readMagRegister(AK09916_CNTL2);
  return (ak09916_data_rate_t)(raw_mag_rate);
}
/**
 * @brief Set the magnetometer measurement rate
 *
 * @param rate The rate to set.
 *
 * @return true: success false: failure
 */
bool Adafruit_ICM20948::setMagDataRate(ak09916_data_rate_t rate) {
  /*
   * Following the datasheet, the sensor will be set to
   * AK09916_MAG_DATARATE_SHUTDOWN followed by a 100ms delay, followed by
   * setting the new data rate.
   *
   * See page 9 of https://www.y-ic.es/datasheet/78/SMDSW.020-2OZ.pdf
   */

  // don't need to read/mask because there's nothing else in the register and
  // it's right justified
  bool success = writeMagRegister(AK09916_CNTL2, AK09916_MAG_DATARATE_SHUTDOWN);
  delay(1);
  return writeMagRegister(AK09916_CNTL2, rate) && success;
}
