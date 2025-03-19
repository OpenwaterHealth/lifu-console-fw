/*
 * max6663.h
 *
 *  Created on: Mar 19, 2025
 *      Author: GeorgeVigelette
 */

#ifndef INC_MAX6663_H_
#define INC_MAX6663_H_

#include "main.h"

#define MAX6653_TOP_I2C_ADDRESS 0x58
#define MAX6653_BOT_I2C_ADDRESS 0x5A

// I2C Timing Constants (adjust based on timing requirements)
#define I2C_DELAY()   do { __NOP(); __NOP(); __NOP(); __NOP(); } while (0)

// I2C Functions
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(uint8_t data);
uint8_t I2C_Read(uint8_t ack);
uint8_t I2C_WriteRegister(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t I2C_ReadRegister(uint8_t addr, uint8_t reg);


#endif /* INC_MAX6663_H_ */
