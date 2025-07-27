#include "MPU6050.h"

#define PI 3.141593
#define ACC_SCALE 16384.0
#define GYRO_SCALE 131.0

volatile float pitch = 0, tilt_angle = 0;
volatile float ax = 0, ay = 0, az = 0;
volatile float gy = 0, gz = 0;
float acc_offsetX = 0, acc_offsetY = 0, acc_offsetZ = 0;
float gyro_offsetY = 0, gyro_offsetZ = 0;
volatile int curtime, pretime = 0, delta = 0;



void MPU_init(uint i2c_port)
{
	i2c_init(i2c_port, 160u, 0);
	MPU_tx(i2c_port, sample_rate, 	0u);			
	MPU_tx(i2c_port, gyro_config, 	0u);			//  +2000'/s full scale range
	MPU_tx(i2c_port, acc_config, 	0u);			//	+2g full scale range
	MPU_tx(i2c_port, pwr_manager, 	1u);			//  PLL with X axis gyroscope reference
	DelayMs(10);
	MPU_Calib();
}

void MPU_tx(uint8_t i2c, uint8_t reg, uint8_t data)
{
	i2c_write(i2c, 0x68, reg, data); 
}

void MPU_Calib(void)
{	
	int samples = 100;
	int tmp = samples;
	while(tmp--)
	{
		ax = MPU_read(MPU_PORT, acc_x)/ACC_SCALE;
		ay = MPU_read(MPU_PORT, acc_y)/ACC_SCALE;
		az = MPU_read(MPU_PORT, acc_z)/ACC_SCALE;
		gy = MPU_read(MPU_PORT, gyro_y)/GYRO_SCALE;
		gz = MPU_read(MPU_PORT, gyro_z)/GYRO_SCALE; 
		
		acc_offsetX += ax;
		acc_offsetY += ay;
		acc_offsetZ += az;
		
		gyro_offsetY += gy;
		gyro_offsetZ += gz;
		
	}
		acc_offsetX /= samples;
		acc_offsetY /= samples;
		acc_offsetZ /= samples;
		
		acc_offsetZ -= 1;
	
		gyro_offsetY /= samples;
		gyro_offsetZ /= samples;
	
}



float MPU_Get_Angle(void)
{	
	// /*Use Timer4 to check of the time between two consecutive measurements - 5ms*/
	// curtime = TIM4->CNT;
	// delta = (curtime > pretime) ? (curtime - pretime) : ((TIM4->ARR) + curtime - pretime);
	// pretime = curtime;
	
	ax = MPU_read(MPU_PORT, 59)/ACC_SCALE - acc_offsetX;
	ay = MPU_read(MPU_PORT, 61)/ACC_SCALE - acc_offsetY;
	az = MPU_read(MPU_PORT, 63)/ACC_SCALE - acc_offsetZ;
	gy = MPU_read(MPU_PORT, 69)/GYRO_SCALE - gyro_offsetY;
	gz = MPU_read(MPU_PORT, 71)/GYRO_SCALE - gyro_offsetZ;
	
	pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0 / PI;

	tilt_angle = Kalman_Filter_y(pitch, gy);
	return tilt_angle;
}

int16_t MPU_read(uint8_t i2c, uint8_t reg)
{
	uint8_t value[2] = {0};
	int16_t raw = 0;
	i2c_read_1Byte(i2c, 0x68, reg, value);
	i2c_read_1Byte(i2c, 0x68, reg+1, value+1);
	raw = (int16_t)value[0]<<8 | value[1];
	return raw;
}




