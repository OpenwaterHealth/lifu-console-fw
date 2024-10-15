/*
 * max6639.h
 *
 *  Created on: Oct 15, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_MAX6639_H_
#define INC_MAX6639_H_

#include "main.h"

/* The MAX6639 registers, valid channel numbers: 0, 1 */
#define MAX6639_REG_TEMP(ch)			(0x00 + (ch))
#define MAX6639_REG_STATUS			0x02
#define MAX6639_REG_OUTPUT_MASK			0x03
#define MAX6639_REG_GCONFIG			0x04
#define MAX6639_REG_TEMP_EXT(ch)		(0x05 + (ch))
#define MAX6639_REG_ALERT_LIMIT(ch)		(0x08 + (ch))
#define MAX6639_REG_OT_LIMIT(ch)		(0x0A + (ch))
#define MAX6639_REG_THERM_LIMIT(ch)		(0x0C + (ch))
#define MAX6639_REG_FAN_CONFIG1(ch)		(0x10 + (ch) * 4)
#define MAX6639_REG_FAN_CONFIG2a(ch)		(0x11 + (ch) * 4)
#define MAX6639_REG_FAN_CONFIG2b(ch)		(0x12 + (ch) * 4)
#define MAX6639_REG_FAN_CONFIG3(ch)		(0x13 + (ch) * 4)
#define MAX6639_REG_FAN_CNT(ch)			(0x20 + (ch))
#define MAX6639_REG_TARGET_CNT(ch)		(0x22 + (ch))
#define MAX6639_REG_FAN_PPR(ch)			(0x24 + (ch))
#define MAX6639_REG_TARGTDUTY(ch)		(0x26 + (ch))
#define MAX6639_REG_FAN_START_TEMP(ch)		(0x28 + (ch))
#define MAX6639_REG_DEVID			0x3D
#define MAX6639_REG_MANUID			0x3E
#define MAX6639_REG_DEVREV			0x3F

/* Register bits */
#define MAX6639_GCONFIG_STANDBY			0x80
#define MAX6639_GCONFIG_POR			0x40
#define MAX6639_GCONFIG_DISABLE_TIMEOUT		0x20
#define MAX6639_GCONFIG_CH2_LOCAL		0x10
#define MAX6639_GCONFIG_PWM_FREQ_HI		0x08

#define MAX6639_FAN_CONFIG1_PWM			0x80
#define MAX6639_FAN_CONFIG3_FREQ_MASK		0x03
#define MAX6639_FAN_CONFIG3_THERM_FULL_SPEED	0x40

#define MAX6639_NUM_CHANNELS			2

int max6639_temp_read_input(I2C_HandleTypeDef *devI2C, int channel, long *temp);
int max6639_temp_read_fault(I2C_HandleTypeDef *devI2C, int channel, long *fault);
int max6639_temp_read_max(I2C_HandleTypeDef *devI2C, int channel, long *max);
int max6639_temp_read_crit(I2C_HandleTypeDef *devI2C, int channel, long *crit);
int max6639_temp_read_emergency(I2C_HandleTypeDef *devI2C, int channel, long *emerg);
int max6639_get_status(I2C_HandleTypeDef *devI2C, unsigned int *status);
int max6639_temp_set_max(I2C_HandleTypeDef *devI2C, int channel, long val);
int max6639_temp_set_crit(I2C_HandleTypeDef *devI2C, int channel, long val);
int max6639_temp_set_emergency(I2C_HandleTypeDef *devI2C, int channel, long val);
int max6639_set_ppr(I2C_HandleTypeDef *devI2C, int channel, uint8_t ppr);
int max6639_read_fan(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long *fan_val);
int max6639_write_fan(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long val);
int max6639_read_pwm(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long *pwm_val);
int max6639_write_pwm(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long val);
int max6639_read_temp(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long *val);

#endif /* INC_MAX6639_H_ */
