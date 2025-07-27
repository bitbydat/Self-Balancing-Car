#include "PWM.h"


void PWM_motor_init(uint32_t arr)
{
	//Config PB15 as wake-up pin of DRV8833
	init_GP(PB, 15, OUT50, O_GP_PP);
	GPIOB->ODR |=  GPIO_ODR_ODR15;		
	//Left motor 
	timer_pwm_micro(PA, 8, arr, 0);			//Timer 1 - channel 1
	timer_pwm_micro(PA, 9, arr, 0);			//Timer 1 - channel 2
	//Right motor 
	timer_pwm_micro(PA, 10, arr, 0);		//Timer 1 - channel 3
	timer_pwm_micro(PA, 11, arr, 0);    	//Timer 1 - channel 4
}

void PWM_Set(int motor_left, int motor_right)
{
	if(motor_left == 0)
	{
		L_PWMA = 0;
		L_PWMB = 0;
	}
	if(motor_right == 0)
	{
		R_PWMA = 0;
		R_PWMB = 0;
	}
	//Left wheel
  	if(motor_left > 0)
	{
		L_PWMB = myabs(motor_left);
		L_PWMA = 0;
	}		
	else
	{
		L_PWMB = 0;
		L_PWMA = myabs(motor_left);
	}
	
	//Right wheel
	if(motor_right>0) 
	{
		R_PWMB = myabs(motor_right);
		R_PWMA = 0;
	}
	else 
	{
		R_PWMB = 0;
		R_PWMA = myabs(motor_right);	
	}

}

int PWM_IgnoreZone(int pulse)
{
	if (pulse >= 0) return pulse + MOTOR_IGNORE_PULSE;
  	if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
	return pulse;
}

int PWM_Limit(int pwm_in, int max, int min)
{
	if(pwm_in > max) pwm_in = max;
	if(pwm_in < min) pwm_in = min;
	return pwm_in;
}

void PWM_Update_Value(pwm_t *pwm)
{
	pwm->balance =	Balance_PID(tilt_angle, gy);
	pwm->turn 		= Turn_PD(gz);

	pwm->motorL = pwm->balance - pwm->turn;
	pwm->motorR = pwm->balance + pwm->turn;

	pwm->motorL = PWM_IgnoreZone(pwm->motorL);
	pwm->motorR = PWM_IgnoreZone(pwm->motorR);

	pwm->motorL = PWM_Limit(pwm->motorL, 1000, -1000);
	pwm->motorR = PWM_Limit(pwm->motorR, 1000, -1000);
}

uint8_t Car_Safety_Check(float angle)
{
	if(angle<-25 || angle>25 || enable_flag==0)			//Turn off the motor if the inclination angle is greater than 30 degrees			
	{	                                                                                   
		L_PWMA = 0;
		L_PWMB = 0;
		R_PWMA = 0;
		R_PWMB = 0;
		return 0;
	}
	else return 1;
}

int myabs(int a)
{ 		   
	return (a<0) ? (-a) : (a);
}

