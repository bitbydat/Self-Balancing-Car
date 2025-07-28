#include "allheader.h"

volatile uint8_t enable_flag = 0;
volatile uint8_t measure_due_flag = 0;
uint16_t millis_tick = 0, last_ticks = 0, pre_ticks = 0;
pwm_t pwm;


int main(void)
{	
	Clock_init();
	systick_int_start();	
	/*Enable control buttons*/
	Button_Init();
	/*Enable PWM for motors*/
	PWM_motor_init(1000);
	/*Enable HC05 Bluetooth Module on USART */
	HC05_init(HC05_PORT);
	/*Enable MPU on I2C and calibration */
	MPU_init(MPU_PORT);
	// tim_start_micros(4, 50000);
	/*Enable Timer2 to interrupt after every 5ms period*/
	tim_start_millis(2, 5);	
	
	while(1)
	{
		enable_flag = Get_Button_State(PB, 11);
		Check_Bluetooth_Order();
		/*Check if it's time to measure MPU6050*/
		if(measure_due_flag)
		{
			MPU_Get_Angle();
			PWM_Update_Value(&pwm);
		  	if(Car_Safety_Check(tilt_angle) == VALID) 
			{
				PWM_Set(pwm.motorL, pwm.motorR); 
			}
			measure_due_flag = 0;
		}
	}
}

void Clock_init(void)
{
    //if 48 MHz < SYSCLK < 72 MHz, the SYSCLK wait two states to the FLASH access time
	FLASH->ACR |= FLASH_ACR_LATENCY_2;
    //Enable HSE clock
	RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));
    //Select PLL source as HSE clock 8MHz
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE;
    //Multiple PLL input clock by 9 
	RCC->CFGR |= RCC_CFGR_PLLMULL9;
    //Divide APB1 by 2 (36MHz max)
	RCC->CFGR |= RCC_CFGR_PPRE1_2;
    //Enbale PLL clock
	RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    //Select SYSCLK source as PLL clock
	RCC->CFGR |= RCC_CFGR_SW_PLL;
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));
}

void TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_SR_UIF) 
	{
	  TIM2->SR &= ~TIM_SR_UIF;
	  //Set flag to measure MPU6050 every 5ms period
	  measure_due_flag = 1;
	}
}


void Button_Init(void)
{
	init_GP(PB, 0, IN, I_PP);
	GPIOB->ODR |= GPIO_ODR_ODR0;

	init_GP(PB, 11, IN, I_PP);
	GPIOB->ODR |= GPIO_ODR_ODR11;
}

uint8_t Get_Button_State(uint8_t Port, uint8_t Pin)
{
	static uint8_t button_state = 0;
	last_ticks = millis_tick;
	if((last_ticks - pre_ticks) >= 2)
	{
		if(Button_Debounce(Port, Pin))
		{
			button_state ^= 1;
		}
		pre_ticks = millis_tick;
	}
	return button_state;
}

uint8_t Button_Debounce(uint8_t Port, uint8_t Pin)
{
	static uint16_t state = 1;
	volatile uint32_t * IDR;
	uint32_t offset = 0x02;

	switch (Port)
	{
		case 1:  IDR = (volatile uint32_t *)(&GPIO_A + offset); break;
		case 2:  IDR = (volatile uint32_t *)(&GPIO_B + offset); break;
		case 3:  IDR = (volatile uint32_t *)(&GPIO_C + offset); break;
		default: IDR = NULL; 					break;
	}
 	state = (state<<1) | (uint16_t)((*IDR & (1<<Pin))>>Pin) ;
  	return (state == 0x000F);
}

void SysTick_Handler()
{
	millis_tick++;
}

