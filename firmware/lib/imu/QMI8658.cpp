/*
 * @Description: QMI8658
 * @Author: zjw
 * @Date: 2022-10-24
 * @LastEditTime: 2022-10-24
 * @LastEditors: zjw
 */

#include "QMI8658.h"

#include <Wire.h>
#include <math.h>
//#define QMI8658_UINT_MG_DPS
//#define M_PI			(3.14159265358979323846f)
#define ONE_G			(9.807f)
#define QFABS(x)		(((x)<0.0f)?(-1.0f*(x)):(x))

static qmi8658_state g_imu;


#define Kp 10.0f
#define Ki 0.008f
/* #define pi 3.14159265f */
#define halfT 0.002127f        /*half the sample period*/
/* 参与计算的加速度单位g 陀螺仪单位是弧度/s()【度*pi/180=弧度】*/


/* 魔法函数InvSqrt()相当于1.0/sqrt() */
static float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

EulerAngles get_euler_angles(float gx, float gy, float gz, float ax, float ay, float az)
{
  EulerAngles eulaer;
  float roll, pitch,yaw ;
  float exInt, eyInt, ezInt;
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */

    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;

    float q1q1 = q1*q1;
    float q1q3 = q1*q3;

    float q2q2 = q2*q2;
    float q2q3 = q2*q3;

    float q3q3 = q3*q3;


    if( ax*ay*az==0)
        return eulaer;
    /* 对加速度数据进行归一化处理 */
    recipNorm = invSqrt( ax* ax +ay*ay + az*az);
    ax = ax *recipNorm;
    ay = ay *recipNorm;
    az = az *recipNorm;
    /* DCM矩阵旋转 */
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;
    /* 在机体坐标系下做向量叉积得到补偿数据 */
    ex = ay*vz - az*vy ;
    ey = az*vx - ax*vz ;
    ez = ax*vy - ay*vx ;
    /* 对误差进行PI计算，补偿角速度 */
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;
    /* 按照四元素微分公式进行四元素更新 */
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    q0 = q0*recipNorm;
    q1 = q1*recipNorm;
    q2 = q2*recipNorm;
    q3 = q3*recipNorm;

    roll =  atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3f;
    pitch =  asinf(2*q1*q3 - 2*q0*q2)*57.3f;
    yaw  =  -atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 -2*q3*q3 + 1)*57.3f;

    eulaer.pitch = pitch;
    eulaer.roll = roll;
    eulaer.yaw = yaw;

    // printf("pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);
    return eulaer;
}


void QMI8658::write_reg(uint8_t reg,uint8_t value)
{
  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(reg);
  Wire.write(value);
  last_status = Wire.endTransmission();
}

uint8_t QMI8658::read_reg(uint8_t reg)
{
	uint8_t ret=0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(reg);
    last_status = Wire.endTransmission();
    Wire.requestFrom(QMI8658_ADDR, 1);
    ret = Wire.read();
    Wire.endTransmission();
	}
	return ret;
}

uint16_t QMI8658::readWord_reg(uint8_t reg)
{
	uint8_t retH=0;
  uint8_t retL=0;

  Wire.beginTransmission(QMI8658_ADDR);
  Wire.write(reg);
  last_status = Wire.endTransmission();
  Wire.requestFrom(QMI8658_ADDR, 2);
  retL = Wire.read();
  retH = Wire.read();
  Wire.endTransmission();

	return ((retH << 8) | retL);
}


bool QMI8658::GetEulerAngles(float *pitch,float *roll, float *yaw)
{
  float acc[3];
  float gyro[3];
  read_xyz(acc,gyro);

  EulerAngles ea = get_euler_angles(gyro[0], gyro[1],gyro[2], acc[0], acc[1], acc[2]);
  *pitch = ea.pitch;
  *roll = ea.roll;
  *yaw = ea.yaw;
  return true;
}



void QMI8658::read_sensor_data(float acc[3], float gyro[3])
{
	unsigned char	buf_reg[12];
	short 			raw_acc_xyz[3];
	short 			raw_gyro_xyz[3];

	raw_acc_xyz[0] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Ax_L) ));
	raw_acc_xyz[1] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Ay_L) ));
	raw_acc_xyz[2] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Az_L) ));

	raw_gyro_xyz[0] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Gx_L) ));
	raw_gyro_xyz[1] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Gy_L) ));
	raw_gyro_xyz[2] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Gz_L) ));

#if defined(QMI8658_UINT_MG_DPS)
	// mg
	acc[0] = (float)(raw_acc_xyz[0]*1000.0f)/g_imu.ssvt_a;
	acc[1] = (float)(raw_acc_xyz[1]*1000.0f)/g_imu.ssvt_a;
	acc[2] = (float)(raw_acc_xyz[2]*1000.0f)/g_imu.ssvt_a;
#else
	// m/s2
	acc[0] = (float)(raw_acc_xyz[0]*ONE_G)/g_imu.ssvt_a;
	acc[1] = (float)(raw_acc_xyz[1]*ONE_G)/g_imu.ssvt_a;
	acc[2] = (float)(raw_acc_xyz[2]*ONE_G)/g_imu.ssvt_a;
#endif

#if defined(QMI8658_UINT_MG_DPS)
	// dps
	gyro[0] = (float)(raw_gyro_xyz[0]*1.0f)/g_imu.ssvt_g;
	gyro[1] = (float)(raw_gyro_xyz[1]*1.0f)/g_imu.ssvt_g;
	gyro[2] = (float)(raw_gyro_xyz[2]*1.0f)/g_imu.ssvt_g;
#else
	// rad/s
	gyro[0] = (float)(raw_gyro_xyz[0]*M_PI)/(g_imu.ssvt_g*180);		// *pi/180
	gyro[1] = (float)(raw_gyro_xyz[1]*M_PI)/(g_imu.ssvt_g*180);
	gyro[2] = (float)(raw_gyro_xyz[2]*M_PI)/(g_imu.ssvt_g*180);
#endif
}

void QMI8658::read_acc(float acc[3])
{
	unsigned char	buf_reg[12];
	short 		raw_acc_xyz[3];

	raw_acc_xyz[0] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Ax_L) ));
	raw_acc_xyz[1] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Ay_L) ));
	raw_acc_xyz[2] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Az_L) ));

#if defined(QMI8658_UINT_MG_DPS)
	// mg
	acc[0] = (float)(raw_acc_xyz[0]*1000.0f)/g_imu.ssvt_a;
	acc[1] = (float)(raw_acc_xyz[1]*1000.0f)/g_imu.ssvt_a;
	acc[2] = (float)(raw_acc_xyz[2]*1000.0f)/g_imu.ssvt_a;
#else
	// m/s2
	acc[0] = (float)(raw_acc_xyz[0]*ONE_G)/g_imu.ssvt_a;
	acc[1] = (float)(raw_acc_xyz[1]*ONE_G)/g_imu.ssvt_a;
	acc[2] = (float)(raw_acc_xyz[2]*ONE_G)/g_imu.ssvt_a;
#endif
}

void QMI8658::read_gyro(float gyro[3])
{
	unsigned char	buf_reg[12];
	short 		raw_gyro_xyz[3];

	raw_gyro_xyz[0] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Gx_L) ));
	raw_gyro_xyz[1] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Gy_L) ));
	raw_gyro_xyz[2] = (short)((unsigned short)( readWord_reg(Qmi8658Register_Gz_L) ));

#if defined(QMI8658_UINT_MG_DPS)
	// dps
	gyro[0] = (float)(raw_gyro_xyz[0]*1.0f)/g_imu.ssvt_g;
	gyro[1] = (float)(raw_gyro_xyz[1]*1.0f)/g_imu.ssvt_g;
	gyro[2] = (float)(raw_gyro_xyz[2]*1.0f)/g_imu.ssvt_g;
#else
	// rad/s
	gyro[0] = (float)(raw_gyro_xyz[0]*M_PI)/(g_imu.ssvt_g*180);		// *pi/180
	gyro[1] = (float)(raw_gyro_xyz[1]*M_PI)/(g_imu.ssvt_g*180);
	gyro[2] = (float)(raw_gyro_xyz[2]*M_PI)/(g_imu.ssvt_g*180);
#endif
}



void QMI8658::axis_convert(float data_a[3], float data_g[3], int layout)
{
	float raw[3],raw_g[3];

	raw[0] = data_a[0];
	raw[1] = data_a[1];
	//raw[2] = data[2];
	raw_g[0] = data_g[0];
	raw_g[1] = data_g[1];
	//raw_g[2] = data_g[2];

	if(layout >=4 && layout <= 7)
	{
		data_a[2] = -data_a[2];
		data_g[2] = -data_g[2];
	}

	if(layout%2)
	{
		data_a[0] = raw[1];
		data_a[1] = raw[0];

		data_g[0] = raw_g[1];
		data_g[1] = raw_g[0];
	}
	else
	{
		data_a[0] = raw[0];
		data_a[1] = raw[1];

		data_g[0] = raw_g[0];
		data_g[1] = raw_g[1];
	}

	if((layout==1)||(layout==2)||(layout==4)||(layout==7))
	{
		data_a[0] = -data_a[0];
		data_g[0] = -data_g[0];
	}
	if((layout==2)||(layout==3)||(layout==6)||(layout==7))
	{
		data_a[1] = -data_a[1];
		data_g[1] = -data_g[1];
	}
}



void QMI8658::read_xyz(float acc[3], float gyro[3])
{
	unsigned char	status;
	unsigned char data_ready = 0;

#if defined(QMI8658_SYNC_SAMPLE_MODE)
	qmi8658_read_reg(Qmi8658Register_StatusInt, &status, 1);
	if(status&0x01)
	{
		data_ready = 1;
		qmi8658_delay_us(6);	// delay 6us
	}
#else
	status = read_reg(Qmi8658Register_Status0);
	if(status&0x03)
	{
		data_ready = 1;
	}
#endif
	if(data_ready)
	{
		read_sensor_data(acc, gyro);
		axis_convert(acc, gyro, 0);
#if defined(QMI8658_USE_CALI)
		qmi8658_data_cali(1, acc);
		qmi8658_data_cali(2, gyro);
#endif
		g_imu.imu[0] = acc[0];
		g_imu.imu[1] = acc[1];
		g_imu.imu[2] = acc[2];
		g_imu.imu[3] = gyro[0];
		g_imu.imu[4] = gyro[1];
		g_imu.imu[5] = gyro[2];
	}
	else
	{
		acc[0] = g_imu.imu[0];
		acc[1] = g_imu.imu[1];
		acc[2] = g_imu.imu[2];
		gyro[0] = g_imu.imu[3];
		gyro[1] = g_imu.imu[4];
		gyro[2] = g_imu.imu[5];
		Serial.print("data ready fail!\n");
	}
}



void QMI8658::config_acc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
	unsigned char ctl_dada;

	switch(range)
	{
		case Qmi8658AccRange_2g:
			g_imu.ssvt_a = (1<<14);
			break;
		case Qmi8658AccRange_4g:
			g_imu.ssvt_a = (1<<13);
			break;
		case Qmi8658AccRange_8g:
			g_imu.ssvt_a = (1<<12);
			break;
		case Qmi8658AccRange_16g:
			g_imu.ssvt_a = (1<<11);
			break;
		default:
			range = Qmi8658AccRange_8g;
			g_imu.ssvt_a = (1<<12);
	}
	if(stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
	else
		ctl_dada = (unsigned char)range|(unsigned char)odr;

	write_reg(Qmi8658Register_Ctrl2, ctl_dada);
// set LPF & HPF
	ctl_dada = read_reg(Qmi8658Register_Ctrl5);
	ctl_dada &= 0xf0;
	if(lpfEnable == Qmi8658Lpf_Enable)
	{
		ctl_dada |= A_LSP_MODE_3;
		ctl_dada |= 0x01;
	}
	else
	{
		ctl_dada &= ~0x01;
	}
	//ctl_dada = 0x00;
	write_reg(Qmi8658Register_Ctrl5,ctl_dada);
// set LPF & HPF
}

void QMI8658::config_gyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
	// Set the CTRL3 register to configure dynamic range and ODR
	unsigned char ctl_dada;

	// Store the scale factor for use when processing raw data
	switch (range)
	{
		case Qmi8658GyrRange_16dps:
			g_imu.ssvt_g = 2048;
			break;
		case Qmi8658GyrRange_32dps:
			g_imu.ssvt_g = 1024;
			break;
		case Qmi8658GyrRange_64dps:
			g_imu.ssvt_g = 512;
			break;
		case Qmi8658GyrRange_128dps:
			g_imu.ssvt_g = 256;
			break;
		case Qmi8658GyrRange_256dps:
			g_imu.ssvt_g = 128;
			break;
		case Qmi8658GyrRange_512dps:
			g_imu.ssvt_g = 64;
			break;
		case Qmi8658GyrRange_1024dps:
			g_imu.ssvt_g = 32;
			break;
		case Qmi8658GyrRange_2048dps:
			g_imu.ssvt_g = 16;
			break;
//		case Qmi8658GyrRange_4096dps:
//			g_imu.ssvt_g = 8;
//			break;
		default:
			range = Qmi8658GyrRange_512dps;
			g_imu.ssvt_g = 64;
			break;
	}

	if(stEnable == Qmi8658St_Enable)
		ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
	else
		ctl_dada = (unsigned char)range | (unsigned char)odr;
	write_reg(Qmi8658Register_Ctrl3, ctl_dada);

// Conversion from degrees/s to rad/s if necessary
// set LPF & HPF
	ctl_dada = read_reg(Qmi8658Register_Ctrl5);
	ctl_dada &= 0x0f;
	if(lpfEnable == Qmi8658Lpf_Enable)
	{
		ctl_dada |= G_LSP_MODE_3;
		ctl_dada |= 0x10;
	}
	else
	{
		ctl_dada &= ~0x10;
	}
	//ctl_dada = 0x00;
	write_reg(Qmi8658Register_Ctrl5,ctl_dada);
// set LPF & HPF
}

void QMI8658::enableSensors(unsigned char enableFlags)
{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
	qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags | 0x80);
#elif defined(QMI8658_USE_FIFO)
	//qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags|QMI8658_DRDY_DISABLE);
	write_reg(Qmi8658Register_Ctrl7, enableFlags);
#else
	qmi8658_write_reg(Qmi8658Register_Ctrl7, enableFlags);
#endif
	g_imu.cfg.enSensors = enableFlags&0x03;

	delay(1);
}

void QMI8658::config_reg(unsigned char low_power)
{
	enableSensors(QMI8658_DISABLE_ALL);
	if(low_power)
	{
		g_imu.cfg.enSensors = QMI8658_ACC_ENABLE;
		g_imu.cfg.accRange = Qmi8658AccRange_8g;
		g_imu.cfg.accOdr = Qmi8658AccOdr_LowPower_21Hz;
		g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
		g_imu.cfg.gyrOdr = Qmi8658GyrOdr_250Hz;
	}
	else
	{
		g_imu.cfg.enSensors = QMI8658_ACCGYR_ENABLE;
		g_imu.cfg.accRange = Qmi8658AccRange_8g;
		g_imu.cfg.accOdr = Qmi8658AccOdr_250Hz;
		g_imu.cfg.gyrRange = Qmi8658GyrRange_1024dps;
		g_imu.cfg.gyrOdr = Qmi8658GyrOdr_250Hz;
	}

	if(g_imu.cfg.enSensors & QMI8658_ACC_ENABLE)
	{
		config_acc(g_imu.cfg.accRange, g_imu.cfg.accOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
	}
	if(g_imu.cfg.enSensors & QMI8658_GYR_ENABLE)
	{
		config_gyro(g_imu.cfg.gyrRange, g_imu.cfg.gyrOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
	}
}

unsigned char QMI8658::get_id(void)
{
	unsigned char qmi8658_chip_id = 0x00;
	unsigned char qmi8658_revision_id = 0x00;
	unsigned char qmi8658_slave[2] = {QMI8658_SLAVE_ADDR_L, QMI8658_SLAVE_ADDR_H};
	int retry = 0;
	unsigned char iCount = 0;
	unsigned char firmware_id[3];
	unsigned char uuid[6];
	unsigned int uuid_low, uuid_high;

	while(iCount<2)
	{
		g_imu.slave = qmi8658_slave[iCount];
		retry = 0;
		while((qmi8658_chip_id != 0x05)&&(retry++ < 5))
		{
			qmi8658_chip_id = read_reg(Qmi8658Register_WhoAmI);
			Serial.printf("Qmi8658Register_WhoAmI = 0x%x\n", qmi8658_chip_id);
		}
		if(qmi8658_chip_id == 0x05)
		{
			qmi8658_on_demand_cali();

			g_imu.cfg.ctrl8_value = 0xc0;
			//QMI8658_INT1_ENABLE, QMI8658_INT2_ENABLE
			write_reg(Qmi8658Register_Ctrl1, 0x60|QMI8658_INT2_ENABLE|QMI8658_INT1_ENABLE);
			qmi8658_revision_id = read_reg(Qmi8658Register_Revision);
			// qmi8658_read_reg(Qmi8658Register_firmware_id, firmware_id, 3);
			// qmi8658_read_reg(Qmi8658Register_uuid, uuid, 6);
			write_reg(Qmi8658Register_Ctrl7, 0x00);
			write_reg(Qmi8658Register_Ctrl8, g_imu.cfg.ctrl8_value);
			// uuid_low = (unsigned int)((unsigned int)(uuid[2]<<16)|(unsigned int)(uuid[1]<<8)|(uuid[0]));
			// uuid_high = (unsigned int)((unsigned int)(uuid[5]<<16)|(unsigned int)(uuid[4]<<8)|(uuid[3]));
			// qmi8658_log("qmi8658_init slave=0x%x Revision=0x%x\n", g_imu.slave, qmi8658_revision_id);
			// qmi8658_log("Firmware ID[0x%x 0x%x 0x%x]\n", firmware_id[2], firmware_id[1],firmware_id[0]);
			// qmi8658_log("UUID[0x%x %x]\n", uuid_high ,uuid_low);
			break;
		}
		iCount++;
	}

	return qmi8658_chip_id;
}


void QMI8658::qmi8658_on_demand_cali(void)
{
	Serial.print("qmi8658_on_demand_cali start\n");
	write_reg(Qmi8658Register_Reset, 0xb0);
	delay(10);	// delay
	write_reg(Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_On_Demand_Cali);
	delay(2200);	// delay 2000ms above
	write_reg(Qmi8658Register_Ctrl9, (unsigned char)qmi8658_Ctrl9_Cmd_NOP);
	delay(100);	// delay
	Serial.print("qmi8658_on_demand_cali done\n");
}

unsigned char QMI8658::begin(void)
{
	if(get_id() == 0x05)
	{
#if defined(QMI8658_USE_AMD)
		qmi8658_config_amd();
#endif
#if defined(QMI8658_USE_PEDOMETER)
		qmi8658_config_pedometer(125);
		qmi8658_enable_pedometer(1);
#endif
		config_reg(0);
		enableSensors(g_imu.cfg.enSensors);
		dump_reg();
#if defined(QMI8658_USE_CALI)
		memset(&g_cali, 0, sizeof(g_cali));
#endif
		return 1;
	}
	else
	{
		// Serial.print("qmi8658_init fail\n");
		return 0;
	}
}


void QMI8658::dump_reg(void)
{
	// unsigned char read_data[8];

	// qmi8658_read_reg(Qmi8658Register_Ctrl1, read_data, 8);
	// qmi8658_log("Ctrl1[0x%x]\nCtrl2[0x%x]\nCtrl3[0x%x]\nCtrl4[0x%x]\nCtrl5[0x%x]\nCtrl6[0x%x]\nCtrl7[0x%x]\nCtrl8[0x%x]\n",
	// 				read_data[0],read_data[1],read_data[2],read_data[3],read_data[4],read_data[5],read_data[6],read_data[7]);
}
