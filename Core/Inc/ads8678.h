/*
 * ads8678.h
 *
 *  Created on: Nov 12, 2025
 *      Author: gvigelet
 */

#ifndef INC_ADS8678_H_
#define INC_ADS8678_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	// Parameters
	SPI_HandleTypeDef* spi;
	uint8_t api_retry_count;
	uint16_t spi_timeout_ms;
	GPIO_TypeDef *cs_port;   uint16_t cs_pin;
	GPIO_TypeDef *rst_port;  uint16_t rst_pin;

} ADS8678__HandleTypeDef;


HAL_StatusTypeDef ADS8678_init(ADS8678__HandleTypeDef *dev);
HAL_StatusTypeDef ADS8678_ReadChannel(ADS8678__HandleTypeDef *dev, uint8_t channel, uint16_t *result);
HAL_StatusTypeDef ADS8678_ReadAllChannels(ADS8678__HandleTypeDef *dev, uint16_t *results);

#ifdef __cplusplus
}
#endif

#endif /* INC_ADS8678_H_ */
