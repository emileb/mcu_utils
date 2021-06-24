/*
 * modbus.h
 *
 *  Created on: Jan 16, 2021
 *      Author: emile
 */

#ifndef MODBUS_H_
#define MODBUS_H_

#include <stdint.h>

#include "st_uart.h"


#define MODBUS_CMD_MULTI_READ 0x3
#define MODBUS_CMD_SINGLE_WRITE 0x6

#define MODBUS_MAX_DATA_LEN_BYTES 32

typedef struct
{

	tUartDevice *uartDev;
	uint8_t id;

	uint8_t lastSentCmd;
	uint8_t lastReceivedCmd;
	uint16_t lastReadReg;

	uint32_t state;
	uint8_t receiveData[MODBUS_MAX_DATA_LEN_BYTES];
	int32_t receivePos;

	uint16_t writeReg;
	uint16_t writeData;

	int32_t dataLen;
	int32_t dataStart;
} tModBusDevice;

void modbus_init(tModBusDevice *device, tUartDevice *uartDev, uint8_t id);

void modbus_sendDiag(tModBusDevice *device, uint8_t id);

uint16_t modbus_sendMultiReadReg(tModBusDevice *device, uint8_t id, uint16_t regStart, uint16_t num);

void modbus_sendWriteReg(tModBusDevice *device, uint8_t id, uint16_t reg, uint16_t data);

bool modbus_checkReceive(tModBusDevice *device);

void modbus_reset(tModBusDevice *device);

#endif /* MODBUS_H_ */
