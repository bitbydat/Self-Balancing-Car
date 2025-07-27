#include "HC05.h"


 volatile char order;
 volatile char neworder = 0;

void HC05_init(uint8_t hc05_port)
{
	UART_init(hc05_port, 9600);
}


void USART2_IRQHandler ()
{
	if(USART2->SR & USART_SR_RXNE)
	{
		order = USART2->DR;
		neworder = 1;
	}
}

void Check_Bluetooth_Order()
{
	if(neworder)
		{
			switch (order) 
				{
				case run_car	 : move_target = Adjust_Angle; 	 	  	break;																															
				case back_car	 : move_target = - Adjust_Angle; 	  	break;																															
				case left_car	 : turn_target = - Turn_Speed_Set;    	break;																															
				case right_car   : turn_target = Turn_Speed_Set;  		break;	
				case stop_car	 : turn_target = 0; move_target = 0;	break;
				default			 : turn_target = 0; move_target = 0;	break;
				}
			neworder = 0;
		}
}



