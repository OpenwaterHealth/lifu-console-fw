/*
 * max6663.c
 *
 *  Created on: Mar 19, 2025
 *      Author: GeorgeVigelette
 */

#include "max6663.h"

// GPIO Configuration
static void I2C_SetSCL(uint8_t state) {
    HAL_GPIO_WritePin(SMBCLK_GPIO_Port, SMBCLK_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void I2C_SetSDA(uint8_t state) {
    HAL_GPIO_WritePin(SMBDAT_GPIO_Port, SMBDAT_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t I2C_ReadSDA(void) {
    return HAL_GPIO_ReadPin(SMBDAT_GPIO_Port, SMBDAT_Pin) == GPIO_PIN_SET;
}

// Initialize GPIO for I2C bit-banging
void I2C_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure SCL pin
    GPIO_InitStruct.Pin = SMBCLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SMBCLK_GPIO_Port, &GPIO_InitStruct);

    // Configure SDA pin
    GPIO_InitStruct.Pin = SMBDAT_Pin;
    HAL_GPIO_Init(SMBDAT_GPIO_Port, &GPIO_InitStruct);

    I2C_SetSCL(1);
    I2C_SetSDA(1);
}

// I2C Start Condition
void I2C_Start(void) {
    I2C_SetSDA(1);
    I2C_SetSCL(1);
    I2C_DELAY();
    I2C_SetSDA(0);
    I2C_DELAY();
    I2C_SetSCL(0);
}

// I2C Stop Condition
void I2C_Stop(void) {
    I2C_SetSDA(0);
    I2C_SetSCL(1);
    I2C_DELAY();
    I2C_SetSDA(1);
}

// Write Byte
void I2C_Write(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        I2C_SetSDA((data & 0x80) != 0);
        I2C_DELAY();
        I2C_SetSCL(1);
        I2C_DELAY();
        I2C_SetSCL(0);
        data <<= 1;
    }

    // Acknowledge bit
    I2C_SetSDA(1);
    I2C_SetSCL(1);
    I2C_DELAY();
    I2C_SetSCL(0);
}

// Read Byte
uint8_t I2C_Read(uint8_t ack) {
    uint8_t data = 0;

    I2C_SetSDA(1);  // Release SDA for input
    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        I2C_SetSCL(1);
        I2C_DELAY();
        if (I2C_ReadSDA()) {
            data |= 0x01;
        }
        I2C_SetSCL(0);
    }

    // Acknowledge bit
    I2C_SetSDA(ack ? 0 : 1);
    I2C_SetSCL(1);
    I2C_DELAY();
    I2C_SetSCL(0);

    return data;
}

// Write a byte to a specific register
uint8_t I2C_WriteRegister(uint8_t addr, uint8_t reg, uint8_t data) {
    I2C_Start();
    I2C_Write(addr << 1);  // Write mode
    I2C_Write(reg);        // Register address
    I2C_Write(data);       // Data to write
    I2C_Stop();
    return 0;  // Add error handling if desired
}

// Read a byte from a specific register
uint8_t I2C_ReadRegister(uint8_t addr, uint8_t reg) {
    uint8_t data;

    I2C_Start();
    I2C_Write(addr << 1);  // Write mode
    I2C_Write(reg);        // Register address
    I2C_Start();           // Restart condition
    I2C_Write((addr << 1) | 1); // Read mode
    data = I2C_Read(0);    // Read data with NACK
    I2C_Stop();

    return data;
}
