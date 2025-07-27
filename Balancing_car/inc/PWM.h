#ifndef __PWM_H
#define __PWM_H


#include "allheader.h"


#define L_PWMA   TIM1->CCR1  //PA8
#define L_PWMB   TIM1->CCR2  //PA9
#define R_PWMA   TIM1->CCR3  //PA10
#define R_PWMB   TIM1->CCR4  //PA11

#define MOTOR_IGNORE_PULSE 600


typedef struct
{
    int balance;
    int turn;
    int motorL;
    int motorR;
} pwm_t;

void PWM_motor_init(uint32_t arr);
void PWM_Set(int motor_left,int motor_right);
int PWM_IgnoreZone(int pulse);
int PWM_Limit(int pwm,int max,int min);
void PWM_Update_Value(pwm_t *pwm);
uint8_t Car_Safety_Check(float angle);
int myabs(int a);


#endif


