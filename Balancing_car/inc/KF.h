#ifndef __KF_H
#define __KF_H

#include "allheader.h"


float Kalman_Filter_y(float Accel,float Gyro);
float KF_Y(float acce_X, float acce_Z, float gyro_Y);
void mul(int A_row, int A_col, int B_row, int B_col, float A[][A_col], float B[][B_col], float C[][B_col]);




#endif


