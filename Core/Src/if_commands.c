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

extern uint8_t FIRMWARE_VERSION_DATA[3];
static uint32_t id_words[3] = {0};

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
	uint16_t ret_data;

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
			printf("Toggle LED\r\n");
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
			ret_data = 0;
			ret_data = HV_GetVoltage();
			uartResp->data_len = 2;
			uartResp->data=(uint8_t*)&ret_data;
			break;
		case OW_POWER_GET_RUN_HV:
			uartResp->command = OW_POWER_GET_HV;
			ret_data = 0;
			ret_data = HV_GetOnVoltage();
			uartResp->data_len = 2;
			uartResp->data=(uint8_t*)&ret_data;
			break;
		case OW_POWER_STATUS:
			uartResp->command = OW_POWER_STATUS;
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

