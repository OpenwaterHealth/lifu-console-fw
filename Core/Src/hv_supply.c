/*
 * hv_supply.c
 *
 *  Created on: Dec 3, 2024
 *      Author: GeorgeVigelette
 */


#include "hv_supply.h"

extern SPI_HandleTypeDef hspi1;

static HAL_StatusTypeDef HV_DAC_ReadReg(uint32_t *data) {
    uint8_t rxBuffer[4];
    HAL_StatusTypeDef status = HAL_ERROR;

    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Receive(&hspi1, rxBuffer, 4, HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        *data = (rxBuffer[0] << 24) | (rxBuffer[1] << 16) | (rxBuffer[2] << 8) | rxBuffer[3];
    }

    return status;
}

static HAL_StatusTypeDef HV_DAC_WriteReg(uint32_t data) {
    uint8_t txBuffer[4];

    txBuffer[0] = (data >> 24) & 0xFF;
    txBuffer[1] = (data >> 16) & 0xFF;
    txBuffer[2] = (data >> 8) & 0xFF;
    txBuffer[3] = data & 0xFF;

    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, txBuffer, 4, HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);

    return status;
}

void HV_ReadStatusRegister(uint32_t *status_data) {
    uint32_t command = 0xF4000000;
    HV_DAC_WriteReg(command);
    HV_DAC_ReadReg(status_data);
}

uint32_t HV_SetDACValue(DAC_Channel_t channel, DAC_BitDepth_t bitDepth, uint16_t value) {
    uint32_t command = 0x0;

	HV_Disable();

    switch (bitDepth) {
        case DAC_BIT_12:
            value &= 0x0FFF;
            value <<= 4;
            break;
        case DAC_BIT_14:
            value &= 0x3FFF;
            value <<= 2;
            break;
        case DAC_BIT_16:
            value &= 0xFFFF;
            break;
        default:
            return 0;
    }

    command = (0x0 << 28) | (0x3 << 24) | (channel << 20) | (value << 4);
    HV_DAC_WriteReg(command);

    return command;
}

void HV_Enable(void) {
    HAL_GPIO_WritePin(HV_SHUTDOWN_GPIO_Port, HV_SHUTDOWN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HB_LED_GPIO_Port, HB_LED_Pin, GPIO_PIN_RESET);
}

void HV_Disable(void) {
    HAL_GPIO_WritePin(HV_SHUTDOWN_GPIO_Port, HV_SHUTDOWN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HB_LED_GPIO_Port, HB_LED_Pin, GPIO_PIN_SET);
}

void HV_ClearDAC(void) {
	HV_Disable();
    HAL_GPIO_WritePin(CLR_GPIO_Port, CLR_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(CLR_GPIO_Port, CLR_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(CLR_GPIO_Port, CLR_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
}
