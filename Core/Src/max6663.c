/*
 * max6663.c
 *
 *  Created on: Mar 19, 2025
 *      Author: GeorgeVigelette
 */

#include "max6663.h"

#include <stdbool.h>

#define DELAY_US 5 // Adjust as per your desired I2C clock speed

// GPIO Configuration
static void SW_I2C_SET_SCL(uint8_t state) {
    HAL_GPIO_WritePin(SMBCLK_GPIO_Port, SMBCLK_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void SW_I2C_SET_SDA(uint8_t state) {
    HAL_GPIO_WritePin(SMBDAT_GPIO_Port, SMBDAT_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t SW_I2C_READ_SDA(void) {
    return HAL_GPIO_ReadPin(SMBDAT_GPIO_Port, SMBDAT_Pin);
}

void sw_delay_us(uint32_t delay) {
	for (volatile uint32_t i = 0; i < (delay * 48); i++) {
		__NOP(); // No Operation instruction to consume clock cycles
	}
}

// Initialize GPIO for I2C bit-banging
void sw_I2C_BUS_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure SCL pin
    GPIO_InitStruct.Pin = SMBCLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SMBCLK_GPIO_Port, &GPIO_InitStruct);

    // Configure SDA pin
    GPIO_InitStruct.Pin = SMBDAT_Pin;
    HAL_GPIO_Init(SMBDAT_GPIO_Port, &GPIO_InitStruct);

    // Set both pins high
    HAL_GPIO_WritePin(SMBDAT_GPIO_Port, SMBDAT_Pin, GPIO_PIN_SET);

    SW_I2C_SET_SCL(1);
    SW_I2C_SET_SDA(1);
}

void sw_I2C_start_cond(void)
{
	SW_I2C_SET_SDA(1);
	SW_I2C_SET_SCL(1);
	sw_delay_us(DELAY_US);
	SW_I2C_SET_SDA(0);
	sw_delay_us(DELAY_US);
	SW_I2C_SET_SCL(0);
	sw_delay_us(DELAY_US);
}

void sw_I2C_stop_cond(void)
{
	SW_I2C_SET_SDA(0);
	SW_I2C_SET_SCL(1);
	sw_delay_us(DELAY_US);
	SW_I2C_SET_SDA(1);
	sw_delay_us(DELAY_US);
}

void sw_I2C_write_bit(uint8_t b)
{
    if (b > 0)
    {
		SW_I2C_SET_SDA(1);
    }
    else
    {
    	SW_I2C_SET_SDA(0);
    }

	sw_delay_us(DELAY_US);
	SW_I2C_SET_SCL(1);
	sw_delay_us(DELAY_US);
	SW_I2C_SET_SCL(0);
}

uint8_t sw_I2C_read_bit(void)
{
    uint8_t bit;
	SW_I2C_SET_SDA(1);
	sw_delay_us(DELAY_US);
	SW_I2C_SET_SCL(1);
    bit = SW_I2C_READ_SDA();
	sw_delay_us(DELAY_US);
	SW_I2C_SET_SCL(0);
    return bit;
}

void sw_I2C_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
    	sw_I2C_write_bit((byte >> (7 - i)) & 0x01);
    }
    // Read ACK/NACK bit
    sw_I2C_read_bit();
}

uint8_t sw_I2C_read_byte(uint8_t ack) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte = (byte << 1) | sw_I2C_read_bit();
    }
    // Send ACK or NACK
    // Send ACK or NACK
    if (ack) {
    	sw_I2C_write_bit(0); // Send ACK (0 on SDA line)
    } else {
    	sw_I2C_write_bit(1); // Send NACK (1 on SDA line)
    }

    return byte;
}
