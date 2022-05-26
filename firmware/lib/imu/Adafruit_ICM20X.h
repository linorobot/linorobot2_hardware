/*!
 *  @file Adafruit_ICM20X.h
 *
 * 	I2C Driver for the Adafruit ICM20X 6-DoF Wide-Range Accelerometer and
 *Gyro library
 *
 * 	This is a library for the Adafruit ICM20X breakouts:
 * 	https://www.adafruit.com/product/4464
 * 	https://www.adafruit.com/product/4554
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_ICM20X_H
#define _ADAFRUIT_ICM20X_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Misc configuration macros
#define I2C_MASTER_RESETS_BEFORE_FAIL                                          \
  5 ///< The number of times to try resetting a stuck I2C master before giving
    ///< up
#define NUM_FINISHED_CHECKS                                                    \
  100 ///< How many times to poll I2C_SLV4_DONE before giving up and resetting

// Bank 0
#define ICM20X_B0_WHOAMI 0x00         ///< Chip ID register
#define ICM20X_B0_USER_CTRL 0x03      ///< User Control Reg. Includes I2C Master
#define ICM20X_B0_LP_CONFIG 0x05      ///< Low Power config
#define ICM20X_B0_REG_INT_PIN_CFG 0xF ///< Interrupt config register
#define ICM20X_B0_REG_INT_ENABLE 0x10 ///< Interrupt enable register 0
#define ICM20X_B0_REG_INT_ENABLE_1 0x11 ///< Interrupt enable register 1
#define ICM20X_B0_I2C_MST_STATUS                                               \
  0x17 ///< Records if I2C master bus data is finished
#define ICM20X_B0_REG_BANK_SEL 0x7F ///< register bank selection register
#define ICM20X_B0_PWR_MGMT_1 0x06   ///< primary power management register
#define ICM20X_B0_ACCEL_XOUT_H 0x2D ///< first byte of accel data
#define ICM20X_B0_GYRO_XOUT_H 0x33  ///< first byte of accel data

// Bank 2
#define ICM20X_B2_GYRO_SMPLRT_DIV 0x00    ///< Gyroscope data rate divisor
#define ICM20X_B2_GYRO_CONFIG_1 0x01      ///< Gyro config for range setting
#define ICM20X_B2_ACCEL_SMPLRT_DIV_1 0x10 ///< Accel data rate divisor MSByte
#define ICM20X_B2_ACCEL_SMPLRT_DIV_2 0x11 ///< Accel data rate divisor LSByte
#define ICM20X_B2_ACCEL_CONFIG_1 0x14     ///< Accel config for setting range

// Bank 3
#define ICM20X_B3_I2C_MST_ODR_CONFIG 0x0 ///< Sets ODR for I2C master bus
#define ICM20X_B3_I2C_MST_CTRL 0x1       ///< I2C master bus config
#define ICM20X_B3_I2C_MST_DELAY_CTRL 0x2 ///< I2C master bus config
#define ICM20X_B3_I2C_SLV0_ADDR                                                \
  0x3 ///< Sets I2C address for I2C master bus slave 0
#define ICM20X_B3_I2C_SLV0_REG                                                 \
  0x4 ///< Sets register address for I2C master bus slave 0
#define ICM20X_B3_I2C_SLV0_CTRL 0x5 ///< Controls for I2C master bus slave 0
#define ICM20X_B3_I2C_SLV0_DO 0x6   ///< Sets I2C master bus slave 0 data out

#define ICM20X_B3_I2C_SLV4_ADDR                                                \
  0x13 ///< Sets I2C address for I2C master bus slave 4
#define ICM20X_B3_I2C_SLV4_REG                                                 \
  0x14 ///< Sets register address for I2C master bus slave 4
#define ICM20X_B3_I2C_SLV4_CTRL 0x15 ///< Controls for I2C master bus slave 4
#define ICM20X_B3_I2C_SLV4_DO 0x16   ///< Sets I2C master bus slave 4 data out
#define ICM20X_B3_I2C_SLV4_DI 0x17   ///< Sets I2C master bus slave 4 data in

#define ICM20948_CHIP_ID 0xEA ///< ICM20948 default device id from WHOAMI
#define ICM20649_CHIP_ID 0xE1 ///< ICM20649 default device id from WHOAMI

/** Options for `enableAccelDLPF` */
typedef enum {
  ICM20X_ACCEL_FREQ_246_0_HZ = 0x1,
  ICM20X_ACCEL_FREQ_111_4_HZ = 0x2,
  ICM20X_ACCEL_FREQ_50_4_HZ = 0x3,
  ICM20X_ACCEL_FREQ_23_9_HZ = 0x4,
  ICM20X_ACCEL_FREQ_11_5_HZ = 0x5,
  ICM20X_ACCEL_FREQ_5_7_HZ = 0x6,
  ICM20X_ACCEL_FREQ_473_HZ = 0x7,
} icm20x_accel_cutoff_t;

/** Options for `enableGyroDLPF` */
typedef enum {
  ICM20X_GYRO_FREQ_196_6_HZ = 0x0,
  ICM20X_GYRO_FREQ_151_8_HZ = 0x1,
  ICM20X_GYRO_FREQ_119_5_HZ = 0x2,
  ICM20X_GYRO_FREQ_51_2_HZ = 0x3,
  ICM20X_GYRO_FREQ_23_9_HZ = 0x4,
  ICM20X_GYRO_FREQ_11_6_HZ = 0x5,
  ICM20X_GYRO_FREQ_5_7_HZ = 0x6,
  ICM20X_GYRO_FREQ_361_4_HZ = 0x7,

} icm20x_gyro_cutoff_t;

class Adafruit_ICM20X;

/** Adafruit Unified Sensor interface for accelerometer component of ICM20X */
class Adafruit_ICM20X_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the ICM20X class */
  Adafruit_ICM20X_Accelerometer(Adafruit_ICM20X *parent) {
    _theICM20X = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x20A;
  Adafruit_ICM20X *_theICM20X = NULL;
};

/** Adafruit Unified Sensor interface for gyro component of ICM20X */
class Adafruit_ICM20X_Gyro : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the gyro sensor
      @param parent A pointer to the ICM20X class */
  Adafruit_ICM20X_Gyro(Adafruit_ICM20X *parent) { _theICM20X = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x20B;
  Adafruit_ICM20X *_theICM20X = NULL;
};

/** Adafruit Unified Sensor interface for magnetometer component of ICM20X */
class Adafruit_ICM20X_Magnetometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the magnetometer
     sensor
      @param parent A pointer to the ICM20X class */
  Adafruit_ICM20X_Magnetometer(Adafruit_ICM20X *parent) { _theICM20X = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x20C;
  Adafruit_ICM20X *_theICM20X = NULL;
};

/** Adafruit Unified Sensor interface for temperature component of ICM20X */
class Adafruit_ICM20X_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the ICM20X class */
  Adafruit_ICM20X_Temp(Adafruit_ICM20X *parent) { _theICM20X = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x20D;
  Adafruit_ICM20X *_theICM20X = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the ST ICM20X 6-DoF Accelerometer and Gyro
 */
class Adafruit_ICM20X {
public:
  Adafruit_ICM20X();
  ~Adafruit_ICM20X();

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
                 int32_t sensor_id = 0);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensor_id = 0);

  uint8_t getGyroRateDivisor(void);
  void setGyroRateDivisor(uint8_t new_gyro_divisor);

  uint16_t getAccelRateDivisor(void);
  void setAccelRateDivisor(uint16_t new_accel_divisor);

  bool enableAccelDLPF(bool enable, icm20x_accel_cutoff_t cutoff_freq);
  bool enableGyrolDLPF(bool enable, icm20x_gyro_cutoff_t cutoff_freq);

  void reset(void);

  // TODO: bool-ify
  void setInt1ActiveLow(bool active_low);
  void setInt2ActiveLow(bool active_low);

  Adafruit_Sensor *getAccelerometerSensor(void);
  Adafruit_Sensor *getGyroSensor(void);
  Adafruit_Sensor *getMagnetometerSensor(void);
  Adafruit_Sensor *getTemperatureSensor(void);

  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro,
                sensors_event_t *temp, sensors_event_t *mag = NULL);

  uint8_t readExternalRegister(uint8_t slv_addr, uint8_t reg_addr);
  bool writeExternalRegister(uint8_t slv_addr, uint8_t reg_addr, uint8_t value);
  bool configureI2CMaster(void);
  bool enableI2CMaster(bool enable_i2c_master);
  void resetI2CMaster(void);
  void setI2CBypass(bool bypass_i2c);

protected:
  float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in rad/s
      gyroY,         ///< Last reading's gyro Y axis in rad/s
      gyroZ,         ///< Last reading's gyro Z axis in rad/s
      magX,          ///< Last reading's mag X axis in rad/s
      magY,          ///< Last reading's mag Y axis in rad/s
      magZ;          ///< Last reading's mag Z axis in rad/s

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  Adafruit_ICM20X_Accelerometer *accel_sensor =
      NULL;                                 ///< Accelerometer data object
  Adafruit_ICM20X_Gyro *gyro_sensor = NULL; ///< Gyro data object
  Adafruit_ICM20X_Magnetometer *mag_sensor =
      NULL;                                 ///< Magnetometer sensor data object
  Adafruit_ICM20X_Temp *temp_sensor = NULL; ///< Temp sensor data object
  uint16_t _sensorid_accel,                 ///< ID number for accelerometer
      _sensorid_gyro,                       ///< ID number for gyro
      _sensorid_mag,                        ///< ID number for mag
      _sensorid_temp;                       ///< ID number for temperature

  void _read(void);
  virtual void scaleValues(void);
  virtual bool begin_I2C(uint8_t i2c_add, TwoWire *wire, int32_t sensor_id);
  // virtual bool _init(int32_t sensor_id);
  bool _init(int32_t sensor_id);
  int16_t rawAccX, ///< temp variables
      rawAccY,     ///< temp variables
      rawAccZ,     ///< temp variables
      rawTemp,     ///< temp variables
      rawGyroX,    ///< temp variables
      rawGyroY,    ///< temp variables
      rawGyroZ,    ///< temp variables
      rawMagX,     ///< temp variables
      rawMagY,     ///< temp variables
      rawMagZ;     ///< temp variables

  uint8_t current_accel_range; ///< accelerometer range cache
  uint8_t current_gyro_range;  ///< gyro range cache
  // virtual void _setBank(uint8_t bank_number);
  void _setBank(uint8_t bank_number);

  uint8_t readAccelRange(void);
  void writeAccelRange(uint8_t new_accel_range);

  uint8_t readGyroRange(void);
  void writeGyroRange(uint8_t new_gyro_range);

private:
  friend class Adafruit_ICM20X_Accelerometer; ///< Gives access to private
                                              ///< members to Accelerometer
                                              ///< data object
  friend class Adafruit_ICM20X_Gyro; ///< Gives access to private members to
                                     ///< Gyro data object
  friend class Adafruit_ICM20X_Magnetometer; ///< Gives access to private
                                             ///< members to Magnetometer data
                                             ///< object

  friend class Adafruit_ICM20X_Temp; ///< Gives access to private members to
                                     ///< Temp data object

  void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
  void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillMagEvent(sensors_event_t *mag, uint32_t timestamp);
  uint8_t auxillaryRegisterTransaction(bool read, uint8_t slv_addr,
                                       uint8_t reg_addr, uint8_t value = -1);
};

#endif
