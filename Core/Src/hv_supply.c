/*
 * hv_supply.c
 *
 *  Created on: Dec 3, 2024
 *      Author: GeorgeVigelette
 */


#include "hv_supply.h"

#include <stdio.h>
#include <stdbool.h>

#define VOLTAGE_DIVIDER_RATIO 0.03543
#define ADC_MAX 4095.0   // 12-bit ADC resolution

#define DAC_MAX_VALUE 4095
#define STEP_SIZE 50
#define PAUSE_DURATION_MS 100  // 500ms pause every STEP_SIZE

static uint16_t current_hvp_val = 0;
static uint16_t current_hrp_val = 0;
static uint16_t current_hvm_val = 0;
static uint16_t current_hrm_val = 0;
static bool _use_exact = false;

extern SPI_HandleTypeDef hspi1;

float getHVReading() {
	uint16_t adcValue = 0;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);
	HAL_Delay(5);
    HAL_ADC_Start(&hadc);  // Start ADC in continuous mode
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY); // Wait for conversion
    adcValue = HAL_ADC_GetValue(&hadc);

    HAL_ADC_Stop(&hadc);
	// printf("Raw ADC Value 0x%04X \r\n", adcValue);
	return adcValue * VOLTAGE_DIVIDER_RATIO;
}

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

void set_use_exact(bool val)
{
	_use_exact = val;
}

void HV_ReadStatusRegister(uint32_t *status_data) {
    uint32_t command = 0xF4000000;
    HV_DAC_WriteReg(command);
    HV_DAC_ReadReg(status_data);
}

uint32_t HV_SetDACValue(DAC_Channel_t channel, DAC_BitDepth_t bitDepth, uint16_t value) {
    uint32_t command = 0x0;
    if(value >4095)	// prevent the power supply from blowing a cap
    	value = 4095;
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

uint16_t HV_SetVoltage(uint16_t value_hvp, uint16_t value_hvm) {
	current_hvp_val = value_hvp;
	current_hvm_val = value_hvm;
    printf("current_hvp_val: %u\r\n", current_hvp_val);
    printf("current_hvm_val: %u\r\n", current_hvm_val);

    // We set the HV to be 20% above target voltage, setting regulator
    // values to original target voltage.
    current_hrp_val = 1000; // why is this hardcoded?
    current_hrm_val = 1000; // why is this hardcoded?

	// current_hrp_val = (uint16_t) (current_hvp_val * 1.5); 
	// current_hrm_val = (uint16_t) (current_hvm_val * 1.5);

    printf("current_hrp_val: %u\r\n", current_hrp_val);
    printf("current_hrm_val: %u\r\n", current_hrm_val);

    return 0; // we don't use this return value?
}

uint16_t set_hvm(uint16_t value) {
	current_hvm_val = value;
    return value;
}

uint16_t set_hvp(uint16_t value) {
	current_hvp_val = value;
    return value;
}

uint16_t set_hrm(uint16_t value) {
	current_hrm_val = value;
    return value;
}

uint16_t set_hrp(uint16_t value) {
	current_hrp_val = value;
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

void HV_Enable_Exact(void) {
	printf("Using Exact Function\r\n");
    uint16_t set_hvp_val = 0;
    uint16_t set_hvm_val = 0;
    uint16_t set_hrp_val = 0;
    uint16_t set_hrm_val = 0;


    printf("Set Precise DAC Settings\r\n");

	printf("set HVP DAC %d\r\n", current_hvp_val);
	printf("set HVP REG DAC %d\r\n", current_hrp_val);

	printf("set HVM DAC %d\r\n", current_hvm_val);
	printf("set HVM REG DAC %d\r\n", current_hrm_val);

    HV_SetDACValue(DAC_CHANNEL_HVP_REG, DAC_BIT_12, 0);
    HV_SetDACValue(DAC_CHANNEL_HVM_REG, DAC_BIT_12, 0);
    HV_SetDACValue(DAC_CHANNEL_HVP, DAC_BIT_12, 0);
    HV_SetDACValue(DAC_CHANNEL_HVM, DAC_BIT_12, 0);

    HAL_GPIO_WritePin(HV_SHUTDOWN_GPIO_Port, HV_SHUTDOWN_Pin, GPIO_PIN_RESET);

    do{

        // Increment
    	set_hvp_val = set_hvp_val + STEP_SIZE;
    	set_hvm_val = set_hvm_val + STEP_SIZE;

    	if(set_hvp_val >= current_hvp_val) set_hvp_val = current_hvp_val;
    	if(set_hvm_val >= current_hvm_val) set_hvm_val = current_hvm_val;
    	HV_SetDACValue(DAC_CHANNEL_HVP, DAC_BIT_12, set_hvp_val);
        HV_SetDACValue(DAC_CHANNEL_HVM, DAC_BIT_12, set_hvm_val);

        HAL_Delay(PAUSE_DURATION_MS);

    }while (set_hvp_val < current_hvp_val || set_hvm_val < current_hvm_val);

    do{

        // Increment
    	set_hrp_val = set_hrp_val + STEP_SIZE;
    	set_hrm_val = set_hrm_val + STEP_SIZE;

    	if(set_hrp_val >= current_hrp_val) set_hrp_val = current_hrp_val;
    	if(set_hrm_val >= current_hrm_val) set_hrm_val = current_hrm_val;
        HV_SetDACValue(DAC_CHANNEL_HVP_REG, DAC_BIT_12, set_hrp_val);
        HV_SetDACValue(DAC_CHANNEL_HVM_REG, DAC_BIT_12, set_hrm_val);

        HAL_Delay(PAUSE_DURATION_MS);

    }while (set_hrp_val < current_hrp_val || set_hrm_val < current_hrm_val);


	printf("HVP: %d, HVM: %d\r\n", current_hvp_val, current_hvm_val);
	printf("REG HVP: %d, HVM: %d\r\n", current_hrp_val, current_hrm_val);

    HAL_GPIO_WritePin(HV_ON_GPIO_Port, HV_ON_Pin, GPIO_PIN_RESET);

    float hvReading = getHVReading();
    // Integer scaling for 2 decimal places
    printf("HV+ reading: %d.%02dV\r\n", (int)hvReading, (int)(hvReading * 100) % 100);
}


void HV_Enable(void) {

	if(_use_exact) return HV_Enable_Exact();

    uint16_t set_hvp_val = 0;
    uint16_t set_hvm_val = 0;
    uint16_t set_hrp_val = 0;
    uint16_t set_hrm_val = 0;
    float hrp_intercept = 0.0;
    float hrm_intercept = 0.0;
    float hrp_slope = 0.0;
    float hrm_slope = 0.0;

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

	printf("set HVP DAC %d\r\n", current_hvp_val);
	printf("set HVP REG DAC %d\r\n", current_hrp_val);

	printf("set HVM DAC %d\r\n", current_hvm_val);
	printf("set HVM REG DAC %d\r\n", current_hrm_val);

	// Get target voltage
	float target_voltage = (((float)current_hvp_val/4095) * 162.0);
    float target_voltage_reg = target_voltage / 1.2;

    if (target_voltage_reg < 50) {
        ////// NOT CALCULATED YET //////
        // Voltage adjustment formulas for 5V-50V
        // Positive Switching Supply (HVP)
        hrp_intercept = 0;
        hrp_slope = 0;

        // Negative Switching Supply (HVM)
        hrm_intercept = 0;
        hrm_slope = 0;
    } else {
        // Voltage adjustment formulas for 50V-100V
        // Positive Regulator Supply (HVP_REG)
        hrp_intercept = -1.757067669;
        hrp_slope = 0.02652781955;

        // Negative Regulator Supply (HVM_REG)
        hrm_intercept = -1.327744361;
        hrm_slope = 0.02620601504;
    }

	printf("Target HV Voltage %d.%02dV\r\n", (int)target_voltage, (int)(target_voltage * 100) % 100);
    printf("Target REG Voltage %d.%02dV\r\n", (int)target_voltage_reg, (int)(target_voltage_reg * 100) % 100);
	// if(target_voltage>69)
	// {
	// 	set_hrp_val = 0;
	// 	set_hrm_val = 0;
	// }else{
	// 	set_hrp_val = 0;
	// 	set_hrm_val = 0;
	// }

    current_hrp_val = (uint16_t) (target_voltage_reg - hrp_intercept) / hrp_slope;
    current_hrm_val = (uint16_t) (target_voltage_reg - hrm_intercept) / hrm_slope;

    printf("set adjusted HRP DAC %d\r\n", current_hvm_val);
	printf("set adjusted HRM REG DAC %d\r\n", current_hrm_val);

	// Set DACs to 0
    HV_SetDACValue(DAC_CHANNEL_HVP_REG, DAC_BIT_12, 0);
    HV_SetDACValue(DAC_CHANNEL_HVM_REG, DAC_BIT_12, 0);
    HV_SetDACValue(DAC_CHANNEL_HVP, DAC_BIT_12, 0);
    HV_SetDACValue(DAC_CHANNEL_HVM, DAC_BIT_12, 0);

	// turn HV OFF
    HAL_GPIO_WritePin(HV_SHUTDOWN_GPIO_Port, HV_SHUTDOWN_Pin, GPIO_PIN_RESET);

    do{

    	// read HV on ADC this should be 0 when regulator is off
        float test_hvReading = getHVReading();
        printf("SET PS HV+ reading: %d.%02dV hvp: 0x%04X hrp: 0x%04X\r\n", (int)test_hvReading, (int)(test_hvReading * 100) % 100, set_hvp_val, set_hrp_val);

        // when voltage is greater than or equal to 5v over requested value stop increasing the switching voltage
        if(test_hvReading >= (target_voltage+5)) break;

        // Increment switching supply DAC output by step size
    	set_hvp_val = set_hvp_val + STEP_SIZE;
    	set_hvm_val = set_hvm_val + STEP_SIZE;

    	// if Step value is greater than or equal to desired limit the setting
    	if(set_hvp_val >= current_hvp_val) set_hvp_val = current_hvp_val;
    	if(set_hvm_val >= current_hvm_val) set_hvm_val = current_hvm_val;

    	// set switching supply DACs
    	HV_SetDACValue(DAC_CHANNEL_HVP, DAC_BIT_12, set_hvp_val);
        HV_SetDACValue(DAC_CHANNEL_HVM, DAC_BIT_12, set_hvm_val);

        // pause between each step
        HAL_Delay(PAUSE_DURATION_MS);

    }while (set_hvp_val < current_hvp_val || set_hvm_val < current_hvm_val);

    do{
        // Increment Regulator DAC output by step size
    	set_hrp_val = set_hrp_val + STEP_SIZE;
    	set_hrm_val = set_hrm_val + STEP_SIZE;

        // if Step value is greater than or equal to desired limit the setting
    	if(set_hrp_val >= current_hrp_val) set_hrp_val = current_hrp_val;
    	if(set_hrm_val >= current_hrm_val) set_hrm_val = current_hrm_val;

    	// set regulator DACs
    	HV_SetDACValue(DAC_CHANNEL_HVP_REG, DAC_BIT_12, set_hrp_val);
    	HV_SetDACValue(DAC_CHANNEL_HVM_REG, DAC_BIT_12, set_hrm_val);

    	// pause between each step and allow time to settle prior to reading
        HAL_Delay(PAUSE_DURATION_MS);

        // read HV on ADC
        float test_hvReading = getHVReading();
        printf("SET REG HV+ reading: %d.%02dV hvp: 0x%04X hrp: 0x%04X\r\n", (int)test_hvReading, (int)(test_hvReading * 100) % 100, set_hvp_val, set_hrp_val);

        // if we are at target voltage or above stop
    	if(test_hvReading >= target_voltage) break;

    }while (set_hrp_val < current_hrp_val || set_hrm_val < current_hrm_val);  // limit max output

	printf("set HVP: %d, HVM: %d\r\n", set_hvp_val, set_hvm_val);
	printf("set REG HVP: %d, HVM: %d\r\n", set_hrp_val, set_hrm_val);

	// turn HV ON
    HAL_GPIO_WritePin(HV_ON_GPIO_Port, HV_ON_Pin, GPIO_PIN_RESET);

    // pause between each step and allow time to settle
    HAL_Delay(PAUSE_DURATION_MS);
    float hvReading = getHVReading();
    // Integer scaling for 2 decimal places
    printf("HV+ reading: %d.%02dV\r\n", (int)hvReading, (int)(hvReading * 100) % 100);
}

void HV_Disable(void) {

	HAL_GPIO_WritePin(HV_SHUTDOWN_GPIO_Port, HV_SHUTDOWN_Pin, GPIO_PIN_SET);
    HV_SetDACValue(DAC_CHANNEL_HVM_REG, DAC_BIT_12, 0);
    HV_SetDACValue(DAC_CHANNEL_HVP_REG, DAC_BIT_12, 0);
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
        return true;
    } else {
        // Pin is LOW
        return false;
    }
}
