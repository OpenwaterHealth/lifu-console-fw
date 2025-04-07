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
void sw_I2C_BUS_Init(void);

void sw_I2C_start_cond(void);
void sw_I2C_stop_cond(void);

void sw_I2C_write_bit(uint8_t b);
uint8_t sw_I2C_read_bit(void);
void sw_I2C_write_byte(uint8_t byte);
uint8_t I2C_read_byte(uint8_t ack);

#endif /* INC_MAX6663_H_ */
