/*
 * @Description: QMI8658
 * @Author: zjw
 * @Date: 2022-10-24
 * @LastEditTime: 2022-10-24
 * @LastEditors: zjw
 */

#ifndef _QMI8658_H_
#define _QMI8658_H_

#include <Arduino.h>
#include "QMI8658reg.h"

typedef struct
{
	float roll;
  float pitch;
  float yaw ;
} EulerAngles;

class QMI8658
{
  uint8_t last_status; // status of last I2C transmission
  uint8_t read_reg(uint8_t reg);

  void write_reg(uint8_t reg,uint8_t value);

public:
  uint16_t readWord_reg(uint8_t reg);
  // bool init(void);
  bool GetEulerAngles(float *pitch,float *roll, float *yaw);

  void config_acc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr,
                      enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable);
  void config_gyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr,
                      enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable);

  void read_sensor_data(float acc[3], float gyro[3]);
  void read_acc(float acc[3]);
  void read_gyro(float gyro[3]);
  void read_xyz(float acc[3], float gyro[3]);
  void axis_convert(float data_a[3], float data_g[3], int layout);
  void config_reg(unsigned char low_power);
  void enableSensors(unsigned char enableFlags);
  unsigned char get_id(void);
  unsigned char begin(void);
  void dump_reg(void);
  void qmi8658_on_demand_cali(void);

public:
  int16_t ax, ay, az, gx, gy, gz;
  float pith, roll, yaw;
  unsigned long now, lastTime = 0;
  float dt;      //微分时间
  float agz = 0; //角度变量
  long gzo = 0;  //陀螺仪偏移量
};

/*----------------------------------------------------------------------------------------------
  QMI8658C UI Sensor Configuration Settings and Output Data
*/
///<Configuration Registers>
#define QMI8658_ADDR 0X6B  //device address
#define WHO_AM_I 0X00 //Device identifier
#define CTRL1 0x02    //Serial Interface and Sensor Enable
#define CTRL2 0x03    //Accelerometer Settings
#define CTRL3 0x04    //Gyroscope Settings
#define CTRL4 0X05    //Magnetometer Settings
#define CTRL5 0X06    //Sensor Data Processing Settings
#define CTRL7 0x08    //Enable Sensors and Configure Data Reads
#define CTRL8 0X09    //Reserved – Special Settings

///<Sensor Data Output Registers>
#define AccX_L 0x35
#define AccX_H 0x36
#define AccY_L 0x37
#define AccY_H 0x38
#define AccZ_L 0x39
#define AccZ_H 0x3A
#define TEMP_L 0x33

#define GyrX_L 0x3B
#define GyrX_H 0x3C
#define GyrY_L 0x3D
#define GyrY_H 0x3E
#define GyrZ_L 0x3F
#define GyrZ_H 0x40
// int16_t QMI8658C_readBytes(unsigned char tmp);
//extern QMI8658C _QMI8658C;
#endif
