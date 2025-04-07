/*
 * hv_supply.h
 *
 *  Created on: Dec 3, 2024
 *      Author: GeorgeVigelette
 */

#ifndef HV_SUPPLY_MANAGER_H
#define HV_SUPPLY_MANAGER_H

#include "main.h"
#include <stdbool.h>

// Enum for DAC Channels
typedef enum {
    DAC_CHANNEL_HVP = 0,   		// channel A
    DAC_CHANNEL_HVM = 1,   		// channel B
    DAC_CHANNEL_HVP_REG = 2,  	// channel C
    DAC_CHANNEL_HVM_REG = 3		// channel D
} DAC_Channel_t;

// Enum for DAC Bit Depth
typedef enum {
    DAC_BIT_12 = 12,
    DAC_BIT_14 = 14,
    DAC_BIT_16 = 16
} DAC_BitDepth_t;

// Function Prototypes
void HV_ReadStatusRegister(uint32_t *status_data);
uint32_t HV_SetDACValue(DAC_Channel_t channel, DAC_BitDepth_t bitDepth, uint16_t value);
uint16_t HV_SetVoltage(uint16_t value);
uint16_t HV_GetVoltage();
uint16_t HV_GetOnVoltage();
void set_use_exact(bool val);
void HV_Enable(void);
void HV_Disable(void);
void HV_ClearDAC(void);
void V12_Enable(void);
void V12_Disable(void);
bool getHVOnStatus();
bool get12VOnStatus();
float getHVReading();
void System_Disable(void);
void System_Enable(void);
uint16_t set_hvm(uint16_t value);
uint16_t set_hvp(uint16_t value);
uint16_t set_hrm(uint16_t value);
uint16_t set_hrp(uint16_t value);

#endif // HV_SUPPLY_MANAGER_H
