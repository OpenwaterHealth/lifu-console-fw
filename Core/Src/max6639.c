/*
 * max6639.c
 *
 *  Created on: Oct 15, 2024
 *      Author: GeorgeVigelette
 */


#include "max6639.h"


static const int rpm_ranges[] = { 2000, 4000, 8000, 16000 };

/* Supported PWM frequency */
static const unsigned int freq_table[] = { 20, 33, 50, 100, 5000, 8333, 12500,
					   25000 };


#define FAN_FROM_REG(val, rpm_range)	((val) == 0 || (val) == 255 ? \
				0 : (rpm_ranges[rpm_range] * 30) / (val))
#define TEMP_LIMIT_TO_REG(val)	clamp_val((val) / 1000, 0, 255)


/*
 * Client data (each client gets its own)
 */
struct max6639_data {
	struct regmap *regmap;

	/* Register values initialized only once */
	uint8_t ppr[MAX6639_NUM_CHANNELS];	/* Pulses per rotation 0..3 for 1..4 ppr */
	uint8_t rpm_range[MAX6639_NUM_CHANNELS]; /* Index in above rpm_ranges table */

	/* Optional regulator for FAN supply */
	struct regulator *reg;
};

int max6639_temp_read_input(I2C_HandleTypeDef *devI2C, int channel, long *temp)
{

	return 0;
}

int max6639_temp_read_fault(I2C_HandleTypeDef *devI2C, int channel, long *fault)
{
	return 0;
}

int max6639_temp_read_max(I2C_HandleTypeDef *devI2C, int channel, long *max)
{
	return 0;
}

int max6639_temp_read_crit(I2C_HandleTypeDef *devI2C, int channel, long *crit)
{
	return 0;
}

int max6639_temp_read_emergency(I2C_HandleTypeDef *devI2C, int channel, long *emerg)
{
	return 0;
}

int max6639_get_status(I2C_HandleTypeDef *devI2C, unsigned int *status)
{
	return 0;
}

int max6639_temp_set_max(I2C_HandleTypeDef *devI2C, int channel, long val)
{
	return 0;
}

int max6639_temp_set_crit(I2C_HandleTypeDef *devI2C, int channel, long val)
{
	return 0;
}

int max6639_temp_set_emergency(I2C_HandleTypeDef *devI2C, int channel, long val)
{
	return 0;
}

int max6639_set_ppr(I2C_HandleTypeDef *devI2C, int channel, uint8_t ppr)
{
	return 0;
}

int max6639_read_fan(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long *fan_val)
{
	return 0;
}

int max6639_write_fan(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long val)
{
	return 0;
}

int max6639_read_pwm(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long *pwm_val)
{
	return 0;
}

int max6639_write_pwm(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long val)
{
	return 0;
}

int max6639_read_temp(I2C_HandleTypeDef *devI2C, uint32_t attr, int channel, long *val)
{
	return 0;
}
