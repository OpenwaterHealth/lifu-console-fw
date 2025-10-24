/*
 * common.h
 *
 *  Created on: Apr 3, 2024
 *      Author: gvigelet
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include <stdint.h>
#include <stdbool.h>

#define COMMAND_MAX_SIZE 2048

typedef enum {
	OW_START_BYTE = 0xAA,
	OW_END_BYTE = 0xDD,
} USTX_ProtocolTypes;

typedef enum {
	OW_ACK = 0xE0,
	OW_NAK = 0xE1,
	OW_CMD = 0xE2,
	OW_RESP = 0xE3,
	OW_DATA = 0xE4,
	OW_JSON = 0xE5,
	OW_I2C_PASSTHRU = 0xE9,
	OW_CONTROLLER = 0xEA,
	OW_POWER = 0xEB,
	OW_BAD_PARSE = 0xEC,
	OW_BAD_CRC = 0xED,
	OW_UNKNOWN = 0xEE,
	OW_ERROR = 0xEF,

} UartPacketTypes;

typedef enum {
	OW_CODE_SUCCESS = 0x00,
	OW_CODE_IDENT_ERROR = 0xFD,
	OW_CODE_DATA_ERROR = 0xFE,
	OW_CODE_ERROR = 0xFF,
} UstxErrorCodes;

typedef enum {
	OW_CMD_PING = 0x00,
	OW_CMD_PONG = 0x01,
	OW_CMD_VERSION = 0x02,
	OW_CMD_ECHO = 0x03,
	OW_CMD_TOGGLE_LED = 0x04,
	OW_CMD_HWID = 0x05,
	OW_CMD_DFU = 0x0D,
	OW_CMD_NOP = 0x0E,
	OW_CMD_RESET = 0x0F,
} UstxGlobalCommands;

typedef enum {
	OW_CTRL_SCAN_I2C = 0x10,
	OW_CTRL_WRITE_I2C = 0x11,
	OW_CTRL_READ_I2C = 0x12,
	OW_CTRL_SET_SWTRIG = 0x13,
	OW_CTRL_GET_SWTRIG = 0x14,
	OW_CTRL_START_SWTRIG = 0x15,
	OW_CTRL_STOP_SWTRIG = 0x16,
	OW_CTRL_STATUS_SWTRIG = 0x17,
} UstxControllerCommands;

typedef enum {
	OW_POWER_STATUS = 0x30,
	OW_POWER_SET_HV = 0x31,
	OW_POWER_GET_HV = 0x32,
	OW_POWER_HV_ON = 0x33,
	OW_POWER_HV_OFF = 0x34,
	OW_POWER_12V_ON = 0x35,
	OW_POWER_12V_OFF = 0x36,
	OW_POWER_GET_TEMP1 = 0x37,
	OW_POWER_GET_TEMP2 = 0x38,
	OW_POWER_SET_FAN = 0x39,
	OW_POWER_GET_FAN = 0x3A,
	OW_POWER_SET_RGB = 0x3B,
	OW_POWER_GET_RGB = 0x3C,
	OW_POWER_GET_HVON = 0x3D,
	OW_POWER_GET_12VON = 0x3E,
	OW_POWER_SET_DACS = 0x3F,
} UstxPowerCommands;

typedef struct  {
	uint16_t id;
	uint8_t packet_type;
	uint8_t command;
	uint8_t addr;
	uint8_t reserved;
	uint16_t data_len;
	uint8_t* data;
	uint16_t crc;
} UartPacket;

typedef struct  {
	uint32_t magic_num;
	float hv_settng;
	bool auto_on;
	uint8_t reserved;
	uint8_t reserved1;
	uint8_t reserved2;
	uint16_t data_len;
	uint8_t* data;
	uint16_t crc;
} LifuConfig;



#endif /* INC_COMMON_H_ */
