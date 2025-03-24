/*
 * if_commands.c
 *
 *  Created on: Nov 15, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "if_commands.h"
#include "common.h"
#include "i2c_master.h"
#include "hv_supply.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // For rand() and srand()
#include <time.h>    // For seeding random number generator

extern uint8_t FIRMWARE_VERSION_DATA[3];

extern MAX31875_Init_t temp_sensor_1;
extern MAX31875_Init_t temp_sensor_2;

static uint32_t id_words[3] = {0};
static uint16_t ret_voltage = 0;

volatile float last_temperature1 = 0;
volatile float last_temperature2 = 0;
volatile uint8_t last_btFan_speed = 0;
volatile uint8_t last_tpFan_speed = 0;
volatile uint8_t rgb_state = 0; // 0 = off, 1 == RED, 2 == GREEN, 3 == BLUE

static void print_uart_packet(const UartPacket* packet) {
    printf("ID: 0x%04X\r\n", packet->id);
    printf("Packet Type: 0x%02X\r\n", packet->packet_type);
    printf("Command: 0x%02X\r\n", packet->command);
    printf("Data Length: %d\r\n", packet->data_len);
    printf("CRC: 0x%04X\r\n", packet->crc);
    printf("Data: ");
    for (int i = 0; i < packet->data_len; i++) {
        printf("0x%02X ", packet->data[i]);
    }
    printf("\r\n");
}

static void POWER_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
		case OW_CMD_PING:
			uartResp->command = OW_CMD_PING;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			break;
		case OW_CMD_PONG:
			uartResp->command = OW_CMD_PONG;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			break;
		case OW_CMD_VERSION:
			uartResp->command = OW_CMD_VERSION;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
			uartResp->data = FIRMWARE_VERSION_DATA;
			break;
		case OW_CMD_ECHO:
			// exact copy
			uartResp->command = OW_CMD_ECHO;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = cmd.data_len;
			uartResp->data = cmd.data;
			break;
		case OW_CMD_TOGGLE_LED:
			uartResp->command = OW_CMD_TOGGLE_LED;
			HAL_GPIO_TogglePin(HB_LED_GPIO_Port, HB_LED_Pin);
			break;
		case OW_CMD_HWID:
			uartResp->command = OW_CMD_HWID;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			id_words[0] = HAL_GetUIDw0();
			id_words[1] = HAL_GetUIDw1();
			id_words[2] = HAL_GetUIDw2();
			uartResp->data_len = 16;
			uartResp->data = (uint8_t *)&id_words;
			break;
		case OW_POWER_12V_ON:
			uartResp->command = OW_POWER_12V_ON;
			HAL_GPIO_WritePin(V12_ENABLE_GPIO_Port, V12_ENABLE_Pin, GPIO_PIN_SET);
			break;
		case OW_POWER_12V_OFF:
			uartResp->command = OW_POWER_12V_OFF;
			HAL_GPIO_WritePin(V12_ENABLE_GPIO_Port, V12_ENABLE_Pin, GPIO_PIN_RESET);
			break;
		case OW_POWER_HV_ON:
			HV_Enable();
			uartResp->command = OW_POWER_HV_ON;
		    HAL_GPIO_WritePin(HV_ON_GPIO_Port, HV_ON_Pin, GPIO_PIN_RESET);
			break;
		case OW_POWER_HV_OFF:
			HV_Disable();
			uartResp->command = OW_POWER_HV_OFF;
		    HAL_GPIO_WritePin(HV_ON_GPIO_Port, HV_ON_Pin, GPIO_PIN_SET);
			break;
		case OW_POWER_SET_HV:
			uartResp->command = OW_POWER_SET_HV;
			uartResp->addr = 0;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = 0;
			if(cmd.data_len == 2)
			{
				uint16_t dac_value = ((uint16_t)cmd.data[0] << 8) | (uint16_t)cmd.data[1];
				HV_SetVoltage(dac_value);
			}
			break;
		case OW_POWER_GET_HV:
			uartResp->command = OW_POWER_GET_HV;
			ret_voltage = HV_GetVoltage();
			uartResp->data_len = 2;
			uartResp->data=(uint8_t*)&ret_voltage;
			break;
		case OW_POWER_STATUS:
			uartResp->command = OW_POWER_STATUS;
			break;
		case OW_POWER_GET_TEMP1:
			// last_temperature1 = 30.0f + (rand() % 41);  // Random float between 30.0 and 70.0
			last_temperature1 = MAX31875_Get_Temp(&temp_sensor_1);
			uartResp->command = OW_POWER_GET_TEMP1;
			uartResp->data_len = 4;
			uartResp->data = (uint8_t *)&last_temperature1;
			break;
		case OW_POWER_GET_TEMP2:
			// last_temperature2 = 30.0f + (rand() % 41);  // Random float between 30.0 and 70.0
			last_temperature1 = MAX31875_Get_Temp(&temp_sensor_2);
			uartResp->command = OW_POWER_GET_TEMP2;
			uartResp->data_len = 4;
			uartResp->data = (uint8_t *)&last_temperature2;		
			break;
		case OW_POWER_SET_FAN:
			uartResp->command = OW_POWER_SET_FAN;
			if(cmd.addr == 0){
				last_btFan_speed = cmd.data[0];
			}
			else if(cmd.addr == 1){
				last_tpFan_speed = cmd.data[0];
			}else{
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			}

			break;
		case OW_POWER_GET_FAN:
			uartResp->command = OW_POWER_GET_FAN;
			if(cmd.addr == 0){
				uartResp->data_len = 1;
				uartResp->data = (uint8_t *)&last_btFan_speed;
			}
			else if(cmd.addr == 1){
				uartResp->data_len = 1;
				uartResp->data = (uint8_t *)&last_tpFan_speed;
			}else{
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			}

			break;
		case OW_POWER_SET_RGB:
			uartResp->command = OW_POWER_SET_RGB;
			if(uartResp->reserved > 3){
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			}
			else
			{
				rgb_state = cmd.reserved;
			}
			break;
		case OW_POWER_GET_RGB:
			uartResp->command = OW_POWER_GET_RGB;
			uartResp->reserved = rgb_state;
			break;
		case OW_POWER_GET_HVON:
			uartResp->command = OW_POWER_GET_HVON;
			if(getHVOnStatus())
			{
				uartResp->reserved = 1;
			}
			else
			{
				uartResp->reserved = 0;
			}

			break;
		case OW_POWER_GET_12VON:
			uartResp->command = OW_POWER_GET_12VON;
			if(get12VOnStatus()){
				uartResp->reserved = 1;
			}else{
				uartResp->reserved = 0;
			}

			break;
		case OW_CMD_NOP:
			uartResp->command = OW_CMD_NOP;
			break;
		case OW_CMD_RESET:
			uartResp->command = OW_CMD_RESET;

			__HAL_TIM_CLEAR_FLAG(&htim17, TIM_FLAG_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim17, 0);
			if(HAL_TIM_Base_Start_IT(&htim17) != HAL_OK){
				uartResp->packet_type = OW_ERROR;
			}
			break;
		case OW_POWER_SET_DACS:
			uartResp->command = OW_POWER_SET_DACS;
			uartResp->addr = 0;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = 0;
			if(cmd.data_len == 8)
			{
				uint16_t hvp_dac_value = ((uint16_t)cmd.data[0] << 8) | (uint16_t)cmd.data[1];
				uint16_t hvp_dac_reg = ((uint16_t)cmd.data[2] << 8) | (uint16_t)cmd.data[3];
				uint16_t hvm_dac_value = ((uint16_t)cmd.data[4] << 8) | (uint16_t)cmd.data[5];
				uint16_t hvm_dac_reg = ((uint16_t)cmd.data[6] << 8) | (uint16_t)cmd.data[7];

		        // Debug print
		        printf("Received HVP DAC Value: %u (0x%04X)\r\n", hvp_dac_value, hvp_dac_value);
		        set_hvp(hvp_dac_value);
		        printf("Received HVP DAC Register: %u (0x%04X)\r\n", hvp_dac_reg, hvp_dac_reg);
		        set_hrp(hvp_dac_reg);
		        printf("Received HVM DAC Value: %u (0x%04X)\r\n", hvm_dac_value, hvm_dac_value);
		        set_hvm(hvm_dac_value);
		        printf("Received HVM DAC Register: %u (0x%04X)\r\n", hvm_dac_reg, hvm_dac_reg);
		        set_hrm(hvm_dac_reg);

			}else{
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
			}
			break;
		default:
			uartResp->packet_type = OW_UNKNOWN;
			break;
	}

}

UartPacket process_if_command(UartPacket cmd)
{
	UartPacket uartResp;
	// I2C_TX_Packet i2c_packet;
	(void)print_uart_packet;

	uartResp.id = cmd.id;
	uartResp.packet_type = OW_RESP;
	uartResp.addr = 0;
	uartResp.reserved = 0;
	uartResp.data_len = 0;
	uartResp.data = 0;
	switch (cmd.packet_type)
	{
	case OW_JSON:
		//JSON_ProcessCommand(&uartResp, cmd);
		break;
	case OW_CMD:
	case OW_POWER:
		//process by the USTX Controller
		POWER_ProcessCommand(&uartResp, cmd);
		break;
	default:
		uartResp.data_len = 0;
		uartResp.packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}

	return uartResp;

}

