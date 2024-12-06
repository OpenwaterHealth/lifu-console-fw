/*
 * hv_supply.h
 *
 *  Created on: Dec 3, 2024
 *      Author: GeorgeVigelette
 */

#ifndef HV_SUPPLY_MANAGER_H
#define HV_SUPPLY_MANAGER_H

#include "main.h"

// Enum for DAC Channels
typedef enum {
    DAC_CHANNEL_HVP = 0,   // channel A
    DAC_CHANNEL_HVM = 1,   // channel B
    DAC_CHANNEL_VGND = 2,  // channel C
    DAC_CHANNEL_D
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
uint32_t HV_SetVoltage(uint16_t value);
void HV_Enable(void);
void HV_Disable(void);
void HV_ClearDAC(void);

#endif // HV_SUPPLY_MANAGER_H
