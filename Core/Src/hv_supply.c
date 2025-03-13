/*
 * hv_supply.c
 *
 *  Created on: Dec 3, 2024
 *      Author: GeorgeVigelette
 */


#include "hv_supply.h"

static uint16_t current_hvp_val = 0;
static uint16_t current_hvm_val = 0;

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
    if(value >2500)	// prevent the power supply from blowing a cap
    	value = 2500;
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

uint16_t HV_SetVoltage(uint16_t value) {
	current_hvp_val = value;
	current_hvm_val = value;
    return value;
}

uint16_t HV_GetVoltage() {
    return current_hvp_val;
}

uint16_t HV_GetOnVoltage() {
	uint32_t value = 0;
	HV_ReadStatusRegister(&value);
    return (uint16_t)value;
}

void HV_Enable(void) {
    HV_SetDACValue(DAC_CHANNEL_HVP, DAC_BIT_12, current_hvp_val);
    HV_SetDACValue(DAC_CHANNEL_HVM, DAC_BIT_12, current_hvm_val);
    HAL_GPIO_WritePin(HV_SHUTDOWN_GPIO_Port, HV_SHUTDOWN_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(HV_ON_GPIO_Port, HV_ON_Pin, GPIO_PIN_RESET);
}

void HV_Disable(void) {

	HAL_GPIO_WritePin(HV_SHUTDOWN_GPIO_Port, HV_SHUTDOWN_Pin, GPIO_PIN_SET);
    HV_SetDACValue(DAC_CHANNEL_HVP, DAC_BIT_12, 0);
    HV_SetDACValue(DAC_CHANNEL_HVM, DAC_BIT_12, 0);

    HAL_GPIO_WritePin(HV_ON_GPIO_Port, HV_ON_Pin, GPIO_PIN_SET);
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

void V12_Enable(void)
{
    HAL_GPIO_WritePin(V12_ENABLE_GPIO_Port, V12_ENABLE_Pin, GPIO_PIN_RESET);
}

void V12_Disable(void)
{
    HAL_GPIO_WritePin(V12_ENABLE_GPIO_Port, V12_ENABLE_Pin, GPIO_PIN_SET);
}

void System_Enable(void)
{
    HAL_GPIO_WritePin(SYS_EN_GPIO_Port, SYS_EN_Pin, GPIO_PIN_SET);
}

void System_Disable(void)
{
    HAL_GPIO_WritePin(SYS_EN_GPIO_Port, SYS_EN_Pin, GPIO_PIN_RESET);
}

bool getHVOnStatus()
{
    GPIO_PinState pinState = HAL_GPIO_ReadPin(HV_SHUTDOWN_GPIO_Port, HV_SHUTDOWN_Pin);

    if (pinState == GPIO_PIN_SET) {
        // Pin is HIGH
        return false;
    } else {
        // Pin is LOW
        return true;
    }
}

bool get12VOnStatus()
{
    GPIO_PinState pinState = HAL_GPIO_ReadPin(V12_ENABLE_GPIO_Port, V12_ENABLE_Pin);

    if (pinState == GPIO_PIN_SET) {
        // Pin is HIGH
        return false;
    } else {
        // Pin is LOW
        return true;
    }
}
