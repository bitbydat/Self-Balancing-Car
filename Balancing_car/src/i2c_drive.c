#include "i2c_drive.h"
#include "gp_drive.h"
#include "systick_time.h"


/*I2C library rebuild*/
void i2c_init(char i2c, unsigned short CCR, uint8_t dma_mode)
{
	RCC->APB2ENR |= 1;  //Enable clock AFIO
	
	if(i2c == 1)
	{
	RCC->APB1ENR |= (1<<21); //Enable clock I2C1 on APB1
		
	init_GP(PB, 6, OUT50, O_AF_OD); //Init PB6 as SCL
	init_GP(PB, 7, OUT50, O_AF_OD);	//Init PB7 as SDA
	//I2C reset 	
	I2C1->CR1 |=  (1<<15);									
	I2C1->CR1 &= ~(1<<15);
	if(dma_mode)
	{
		I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;
		I2C1->CR1 &= ~I2C_CR1_ENGC;
		I2C1->CR2	|= I2C_CR2_LAST;
		I2C1->CR2 |= I2C_CR2_DMAEN; 
	}
	
	//Peripheral clock 36MHz	(use PLL clock)
	I2C1->CR2 |= 0x24;

//	I2C1->CCR |= I2C_CCR_FS;		//Fast mode enable
	//Config the clock control registers: 160-standard mode | 27-fast mode
	I2C1->CCR |= CCR;
	//Config the rise time register: 33-standard mode | 4-fast mode
	I2C1->TRISE = 0x21;			
	//Enable I2C
	I2C1->CR1 |= I2C_CR1_PE; 
	}
	else
	{
	RCC->APB1ENR |= (1<<22); //Enable clock I2C2 on APB1
		
	init_GP(PB, 10, OUT50, O_AF_OD); //Init PB10 as SCL
	init_GP(PB, 11, OUT50, O_AF_OD);	//Init PB11 as SDA
	//I2C reset 	
	I2C2->CR1 |=  (1<<15);									
	I2C2->CR1 &= ~(1<<15);
	//Peripheral clock 36MHz	(use PLL clock)
	I2C2->CR2 |= 0x24;
		
	I2C2->CCR = CCR;
		
	I2C2->TRISE = 0x9;
	I2C2->CR1 |= I2C_CR1_PE; 
	}
}

void i2c_start(uint8_t i2c)
{
	if(i2c == 1)
	{	
			//Enable the ACK: 
			I2C1->CR1 |= I2C_CR1_ACK;
			//Enable start generation
			I2C1->CR1 |= I2C_CR1_START;
			//wait SB = 1 start generation finished
			while(!(I2C1->SR1 & I2C_SR1_SB)) {};
	}
	else
	{
			//Enable the ACK: 
			I2C2->CR1 |= (1<<10);
			//Enable start generation
			I2C2->CR1 |= 0x100;
			//wait SB = 1 start generation finished
			while(!(I2C2->SR1 & 0x01)) {};
	}
}

void i2c_address(uint8_t i2c ,uint8_t address, uint8_t RW)
{
	if(i2c == 1)
	{
	I2C1->DR = (address << 1u) | RW ;
	//wait ADDR = 1
	while(!(I2C1->SR1 & 0x02)){}			
	//cleared ADDR bit by reading SR1 register followed by reading SR2.
	(void)I2C1->SR1;
	(void)I2C1->SR2;
	}
	else
	{
	I2C2->DR = (address << 1u) | RW ;
	//wait ADDR = 1
	while(!(I2C2->SR1 & 0x02)){}
	//cleared ADDR bit by reading SR1 register followed by reading SR2.
	(void)I2C2->SR1;
	(void)I2C2->SR2;
	}
}
void i2c_data(uint8_t i2c ,uint8_t data)
{
	if(i2c == 1)
	{
		//wait TXE = 1: Data register empty
		while(!(I2C1->SR1 & 0x80)){}
		I2C1->DR = data;
		//wait BTF = 1:  Data byte transfer finished
		while (!(I2C1->SR1 & (1<<2)));
	}
	else
	{
		while(!(I2C2->SR1 & 0x80)){}
		I2C2->DR = data;	
		while (!(I2C2->SR1 & (1<<2)));
	}
}

void i2c_stop(uint8_t i2c)
{
	if(i2c == 1)
	{
		//Stop generation after the current byte transfer
		I2C1->CR1 |= 0x200;
	}
	else
	{
		I2C2->CR1 |= 0x200;
	}
}

void i2c_write(uint8_t i2c ,uint8_t address, uint8_t reg, uint8_t data)
{
	i2c_start(i2c);
	i2c_address(i2c, address, 0);
	i2c_data(i2c, reg);
	i2c_data(i2c, data);
	i2c_stop(i2c);
}


void i2c_read(uint8_t i2c, uint8_t address, uint8_t *data)
{
		if(i2c == 1)
		{
		i2c_start(i2c);
		I2C1->DR = (address<<1u) | 0x01;
		while(!(I2C1->SR1 & 0x02)){}  //wait ADDR = 1
		I2C1->CR1 &= ~(0x400);				//disable ACK bit
		(void)I2C1->SR1;
		(void)I2C1->SR2;							//program ADDR = 0
		I2C1->CR1 |= 0x200;						//stop generation
		while(!(I2C1->SR1 & 0x40)){}	//wait RxNE = 1
		*data = I2C1->DR;	
		i2c_stop(i2c);
		}
		else
		{
		i2c_start(i2c);
		I2C2->DR = (address<<1u) | 0x01;
		while(!(I2C2->SR1 & 0x02)){}
		I2C2->CR1 &= ~(0x400);
		(void)I2C2->SR1;
		(void)I2C2->SR2;
		I2C2->CR1 |= 0x200;
		while(!(I2C2->SR1 & 0x40)){}
		*data = I2C2->DR;	
		i2c_stop(i2c);
		}
}


void i2c_read_1Byte(uint8_t i2c, uint8_t address, uint8_t reg, uint8_t *data)
{
	i2c_start(i2c);	
	i2c_address(i2c, address, 0);
	i2c_data(i2c, reg);
	i2c_read(i2c, address, data);
}


void i2c_read_Bytes(uint8_t i2c, uint8_t address, uint8_t reg, uint8_t *data, uint8_t data_size)
{
	i2c_start(i2c);	
	i2c_address(i2c, address, 0);
	i2c_data(i2c, reg);
	if(i2c==1)
	{
		i2c_start(i2c);
		I2C1->DR = (address<<1u) | 0x01;
		while(!(I2C1->SR1 & 0x02)){}  //wait ADDR = 1
		(void)I2C1->SR1;
		(void)I2C1->SR2;							//program ADDR = 0
		for (uint8_t i = 0; i < data_size; i++, data++)
		{
				while (!(I2C1->SR1 & I2C_SR1_RXNE));
				*data = I2C1->DR;
				if (i + 1 == data_size)
				{
						I2C1->CR1 &= ~I2C_CR1_ACK;							
						I2C1->CR1 |= I2C_CR1_STOP;
				}
		}
	}
	else
	{
		i2c_start(i2c);
		I2C2->DR = (address<<1u) | 0x01;
		while(!(I2C2->SR1 & 0x02)){}
		I2C2->CR1 &= ~(0x400);
		(void)I2C2->SR1;
		(void)I2C2->SR2;
		for (uint8_t i = 0; i < data_size; i++, data++)
		{
				while (!(I2C2->SR1 & I2C_SR1_RXNE));
				*data = I2C2->DR;
				if (i + 1 == data_size)
				{
						I2C2->CR1 &= ~I2C_CR1_ACK;
						I2C2->CR1 |= I2C_CR1_STOP;
				}
		}
	}
}

void i2c_dma_tx_init(uint8_t i2c, unsigned short CCR, uint8_t buffer[], uint8_t buffer_size)
{
	RCC->AHBENR |= 0x1; //Enable clock for DMA1
	
	i2c_init(i2c, CCR, 1);
	
	if(i2c == 1)
	{
	DMA1_Channel6->CCR = 0;
	DMA1_Channel6->CPAR = (unsigned long)&I2C1->DR;
	DMA1_Channel6->CMAR = (unsigned long)buffer;
	DMA1_Channel6->CNDTR = buffer_size;
	DMA1_Channel6->CCR |=	 DMA_CCR1_TCIE
													|DMA_CCR1_MINC
														|DMA_CCR1_CIRC;	
	NVIC_EnableIRQ(DMA1_Channel6_IRQn); //NVIC_EnableIRQ(DMA1_Channel6_IRQn);	
	DMA1_Channel6->CCR |= DMA_CCR1_EN;
	
	I2C1->CR2 |= I2C_CR2_DMAEN;
	}
	else
	{
	DMA1_Channel4->CCR = 0;
	DMA1_Channel4->CPAR = (unsigned long)&I2C2->DR;
	DMA1_Channel4->CMAR = (unsigned long)buffer;
	DMA1_Channel4->CNDTR = buffer_size;
	DMA1_Channel4->CCR |=	 DMA_CCR1_TCIE
													|DMA_CCR1_MINC
														|DMA_CCR1_CIRC;	
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);	
	DMA1_Channel4->CCR |= DMA_CCR1_EN;
	
	I2C2->CR2 |= I2C_CR2_DMAEN;
	}


}

void i2c_tx_dma_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//	i2c_init(i2c, CCR, 1);
	DMA1_Channel6 ->CCR  = 0x00;
	while(DMA1_Channel6->CCR & DMA_CCR6_EN){};
//	DMA1_Channel6->CPAR = (unsigned long)&I2C1->DR;
//	DMA1_Channel6->CMAR = (unsigned long)buffer;
//	DMA1_Channel6->CNDTR = buffer_size;
	DMA1_Channel6->CCR |= DMA_CCR6_MINC | DMA_CCR6_DIR | DMA_CCR6_TCIE
													| DMA_CCR6_HTIE | DMA_CCR6_TEIE ;
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

void i2c_tx_dma(uint8_t SensorAddr, uint8_t buffer[], uint8_t buffer_size)
{
	while(I2C1->SR2 & I2C_SR2_BUSY){};
		I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)) {};
			(void)I2C1->SR1;
	I2C1->DR = SensorAddr<<1; 
		while(((I2C1->SR1)&I2C_SR1_ADDR)==0){};
	DMA_tx(buffer, buffer_size);
		(void)I2C1->SR1;
		(void)I2C1->SR2;
			
	
}

static void DMA_tx(uint8_t buffer[], uint8_t buffer_size)
{
	if(NULL != buffer)
  {
    DMA1_Channel6->CCR&=~DMA_CCR6_EN;
	while((DMA1_Channel6->CCR)&DMA_CCR6_EN){;}

    /* Set memory address */
    DMA1_Channel6->CMAR = (uint32_t)buffer;
		DMA1_Channel6->CPAR=(uint32_t)&(I2C1->DR);
    /* Set number of data items */
    DMA1_Channel6->CNDTR = buffer_size;

    /* Clear all interrupt flags */
    DMA1->IFCR = (DMA_IFCR_CTEIF6 | DMA_IFCR_CTCIF6
        | DMA_IFCR_CHTIF6 | DMA_IFCR_CGIF6);

    /* Enable DMA1_Stream4 */
    DMA1_Channel6->CCR |= DMA_CCR6_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

void i2c_rx_dma_init(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//	i2c_init(i2c, CCR, 1);
	DMA1_Channel7 ->CCR = 0x00;
	while(DMA1_Channel7->CCR & DMA_CCR7_EN){};
//	DMA1_Channel6->CPAR = (unsigned long)&I2C1->DR;
//	DMA1_Channel6->CMAR = (unsigned long)buffer;
//	DMA1_Channel6->CNDTR = buffer_size;
	DMA1_Channel7->CCR |= DMA_CCR6_MINC | DMA_CCR6_TCIE
													| DMA_CCR6_HTIE | DMA_CCR6_TEIE ;
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	
}

void i2c_rx_dma(uint8_t SensorAddr, uint8_t RegisterAddr, uint8_t buffer[], uint8_t buffer_size)
{
	i2c_start(1);	
	i2c_address(1, SensorAddr, 0);
	i2c_data(1, RegisterAddr);
	
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){};
	I2C1->DR = (SensorAddr<<1 | (uint8_t)0x1);
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};
	if(buffer_size <= 2)
	{
		I2C1->CR1 &= ~I2C_CR1_ACK;
		(void)I2C1->SR1;
		(void)I2C1->SR2;
		I2C1->CR1 |= I2C_CR1_STOP;	
	}
		(void)I2C1->SR1;
		(void)I2C1->SR2;
	DMA_rx(buffer, buffer_size);
		
}	

static void DMA_rx(uint8_t buffer[], uint8_t buffer_size)
{
	if(NULL != buffer)
	{
		DMA1_Channel7->CCR &= ~DMA_CCR7_EN;
		while((DMA1_Channel7->CCR) & DMA_CCR7_EN){};
		DMA1_Channel7->CMAR = (uint32_t)buffer;
		DMA1_Channel7->CPAR = (uint32_t)&(I2C1->DR);
		DMA1_Channel7->CNDTR = buffer_size;
		
		/* Clear all interrupt flags */
	  DMA1->IFCR = (DMA_IFCR_CTEIF7 | DMA_IFCR_CTCIF7
	      | DMA_IFCR_CHTIF7 | DMA_IFCR_CGIF7);
			
		DMA1_Channel7->CCR |= DMA_CCR1_EN;
	}
	else
	{}
}	


