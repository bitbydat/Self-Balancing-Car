#ifndef __PID_H
#define __PID_H

#include "allheader.h"


#define Adjust_Angle 10
#define Turn_Speed_Set 20;

int Balance_PID(float Angle,float Gyro);
int Turn_PD(float gyro);
int myabs(int a);

extern volatile float turn_target, move_target;

#endif
