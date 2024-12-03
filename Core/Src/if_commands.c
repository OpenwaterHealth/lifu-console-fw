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

static void process_basic_command(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_PING:
		uartResp->command = OW_CMD_PING;
		break;
	case OW_CMD_PONG:
		uartResp->command = OW_CMD_PONG;
		break;
	case OW_CMD_VERSION:
		uartResp->command = OW_CMD_VERSION;
		uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
		uartResp->data = FIRMWARE_VERSION_DATA;
		break;
	case OW_CMD_HWID:
		uartResp->command = OW_CMD_HWID;
		id_words[0] = HAL_GetUIDw0();
		id_words[1] = HAL_GetUIDw1();
		id_words[2] = HAL_GetUIDw2();
		uartResp->data_len = 16;
		uartResp->data = (uint8_t *)&id_words;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	case OW_CMD_TOGGLE_LED:
		printf("Toggle LED\r\n");
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		HAL_GPIO_TogglePin(SYS_RDY_GPIO_Port, SYS_RDY_Pin);
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}
}


static void POWER_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
		case OW_CMD_PING:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			break;
		case OW_CMD_PONG:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			break;
		case OW_CMD_VERSION:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
			uartResp->data = FIRMWARE_VERSION_DATA;
			break;
		case OW_CMD_ECHO:
			// exact copy
			uartResp->id = cmd.id;
			uartResp->packet_type = cmd.packet_type;
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = cmd.data_len;
			uartResp->data = cmd.data;
			break;
		case OW_CMD_TOGGLE_LED:
			printf("Toggle LED\r\n");
			uartResp->id = cmd.id;
			uartResp->packet_type = cmd.packet_type;
			uartResp->command = cmd.command;
			HAL_GPIO_TogglePin(SYS_RDY_GPIO_Port, SYS_RDY_Pin);
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
		case OW_PWR_SET_HV:
		case OW_PWR_GET_HV:
		case OW_PWR_HV_ON:
		case OW_PWR_HV_OFF:
		case OW_PWR_HV_STATUS:
			uartResp->id = cmd.id;
			uartResp->command = cmd.command;
			uartResp->addr = 0;
			uartResp->reserved = 0;
			uartResp->data_len = 0;
			break;
		default:
			uartResp->addr = 0;
			uartResp->reserved = 0;
			uartResp->data_len = 0;
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
	case OW_CMD:
		process_basic_command(&uartResp, cmd);
		break;
	case OW_JSON:
		//JSON_ProcessCommand(&uartResp, cmd);
		break;
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

