#ifndef __ALLHEADER_H
#define __ALLHEADER_H

#include <stdio.h>
#include <math.h>
#include "stm32f10x.h"   
#include "systick_time.h"
#include "gp_drive.h"
#include "timer_drive.h"
#include "uart_drive.h"
#include "i2c_drive.h"
#include "HC05.h"
#include "MPU6050.h"
#include "PWM.h"
#include "PID.h"
#include "KF.h"


#define VALID 1
#define INVALID 0


/* main */
extern volatile uint8_t enable_flag;

void Clock_init(void);
void Button_Init(void);
uint8_t Button_Debounce(uint8_t Port, uint8_t Pin);
uint8_t Get_Button_State(uint8_t Port, uint8_t Pin);



#endif



