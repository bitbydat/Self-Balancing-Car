#include "PID.h"

//Balance loop PD control parameters
float Balance_Kp =2500;//  Range 0-288
float Balance_Ki =20;
float Balance_Kd =40; //  Range 0-2

//Speed loop PD control parameters 
float Turn_Kp=1000; // Range 0-240
float Turn_Kd=15; // Range 0-2

volatile float turn_target, move_target;



int Balance_PID(float Angle,float Gyro)
{  
   float Angle_bias, Gyro_bias;
   static float Angle_Integral;
   int balance;
   
   Angle_bias = 0 - Angle - move_target;
   Angle_Integral = 0.86*Angle_Integral +  0.14*Angle_bias;
   Gyro_bias = 0 - Gyro; 
   // Calculate the motor PWM PD control for balance control kp is the P coefficient kd is the D coefficient
   balance = - Balance_Kp/100*Angle_bias - Angle_Integral*Balance_Ki/100 - Gyro_bias*Balance_Kd/100; 
   
   //Clear points after motor shutdown 
   if(Car_Safety_Check(tilt_angle) == INVALID) Angle_Integral=0;									
   return balance;
}


int Turn_PD(float gyro)
{
	 static float turn_PWM; 	
	
	 turn_PWM = turn_target*Turn_Kp/100 + gyro*Turn_Kd/100;
	// Steering ring PWM: Right turn is positive, left turn is negative 
	 return turn_PWM;								 				
}