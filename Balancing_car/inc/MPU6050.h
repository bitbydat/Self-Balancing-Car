#ifndef __MPU6050_H
#define __MPU6050_H


#include "allheader.h"


#define MPU_PORT 1

/*Registers of MPU*/
#define sample_rate	        25
#define config			    26
#define gyro_config	        27
#define acc_config	        28
#define level_int		    55
#define interrupts	        56
#define pwr_manager	        107
#define acc_x				59
#define acc_y				61
#define acc_z				63
#define gyro_x				67
#define gyro_y				69
#define gyro_z				71

extern volatile float tilt_angle, gy, gz;
extern volatile int curtime, pretime, delta;

void MPU_init(uint i2c_port);
void MPU_tx(uint8_t i2c, uint8_t reg, uint8_t data);
int16_t MPU_read(uint8_t i2c, uint8_t reg);
float MPU_Get_Angle(void);
void MPU_Calib(void);





#endif



