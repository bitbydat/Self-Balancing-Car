#ifndef __I2C_DRIVE_H
#define __I2C_DRIVE_H

#include <stdio.h>
#include <stdint.h>
#include "stm32f10x.h"




void i2c_init(char i2c, unsigned short CCR, uint8_t dma_mode);
void i2c_start(uint8_t i2c);
void i2c_address(uint8_t i2c ,uint8_t address, uint8_t RW);
void i2c_data(uint8_t i2c , uint8_t data);
void i2c_stop(uint8_t i2c);
void i2c_write(uint8_t i2c ,uint8_t address, uint8_t reg, uint8_t data);
void i2c_read(uint8_t i2c, uint8_t address, uint8_t *data);

void i2c_tx_dma_init(void);
void i2c_tx_dma(uint8_t SensorAddr, uint8_t buffer[], uint8_t buffer_size);
static void DMA_tx(uint8_t buffer[], uint8_t buffer_size);

void i2c_rx_dma_init(void);
void i2c_rx_dma(uint8_t SensorAddr, uint8_t RegisterAddr, uint8_t buffer[], uint8_t buffer_size);
static void DMA_rx(uint8_t buffer[], uint8_t buffer_size);

void i2c_read_1Byte(uint8_t i2c, uint8_t address, uint8_t reg, uint8_t *data);
void i2c_read_Bytes(uint8_t i2c, uint8_t address, uint8_t reg, uint8_t *data, uint8_t data_size);


#endif




