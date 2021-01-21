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

#define MODBUS_MAX_DATA_LEN 16

typedef struct
{

	tUartDevice *uartDev;
	uint8_t id;

	uint8_t lastSentCmd;

	uint32_t state;
	uint8_t receiveData[MODBUS_MAX_DATA_LEN];
	int32_t receivePos;

	int32_t dataLen;
	int32_t dataStart;
} tModBusDevice;

void modbus_init(tModBusDevice *device, tUartDevice *uartDev, uint8_t id);

void modbus_sendDiag(tModBusDevice *device);


uint16_t modbus_sendReadReg(tModBusDevice *device, uint16_t reg);

void modbus_sendWriteReg(tModBusDevice *device, uint16_t reg, uint16_t data);

bool modbus_checkReceive(tModBusDevice *device);

#endif /* MODBUS_H_ */
