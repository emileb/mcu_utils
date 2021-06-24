#include <stdbool.h>
#include <string.h>

#include "modbus.h"


typedef enum
{
	STATE_ID, STATE_CMD, STATE_DATA_LEN, STATE_DATA, STATE_CRC,

} eReceiveState;

static uint16_t calcCrc(uint8_t *data, int32_t length)
{
	int32_t j;

	uint32_t crcReg = 0xFFFF;

	while (length--)
	{
		crcReg ^= *data++;

		for (j = 0; j < 8; j++)
		{
			if (crcReg & 0x01)
			{
				crcReg = (crcReg >> 1) ^ 0xA001;
			}
			else
			{
				crcReg = (crcReg >> 1);
			}
		}
	}

	return crcReg;
}

static void resetReceiveState(tModBusDevice *device)
{
	device->state = STATE_ID;
	device->receivePos = 0;
	device->dataLen = 0;
	device->lastReadReg = 0;
	device->lastReceivedCmd = 0;
	device->lastSentCmd = 0;
}

static bool receiveDecode(tModBusDevice *device)
{
	bool messageReady = false;

	uint16_t crc = calcCrc(device->receiveData, device->receivePos - 2);

	uint16_t crcMsg = (device->receiveData[device->receivePos - 2])
			| (crcMsg = device->receiveData[device->receivePos - 1] << 8);

	if (crc == crcMsg)
	{
		messageReady = true;
	}
	else
	{
		messageReady = false;
	}

	return messageReady;
}

static void receiveError(tModBusDevice *device)
{
	resetReceiveState(device);
	uartResetRxFifo(device->uartDev);
}

static bool processByte(tModBusDevice *device, uint8_t b)
{
	bool messageReady = false;

	device->receiveData[device->receivePos++] = b;

	switch (device->state)
	{
	case STATE_ID:
	{
		if (b == device->id) // First byte received should be the ID
			device->state = STATE_CMD;
		else
			receiveError(device);

		break;
	}
	case STATE_CMD:
	{

		device->lastReceivedCmd = b;

		if (b == MODBUS_CMD_MULTI_READ) // Need to read the data length
			device->state = STATE_DATA_LEN;
		else if (b == MODBUS_CMD_SINGLE_WRITE) // Receive the data, we know the length
		{
			device->dataLen = 4; // 2 for reg, 2 for data
			device->dataStart = device->receivePos; // Save start position of the data
			device->state = STATE_DATA;
		}
		else
			// Unknown command
			receiveError(device);

		break;
	}
	case STATE_DATA_LEN:
	{
		if (b < MODBUS_MAX_DATA_LEN_BYTES)
		{
			device->dataLen = b;
			device->dataStart = device->receivePos; // Save start position of the data
			device->state = STATE_DATA;
		}
		else
			receiveError(device);

		break;
	}
	case STATE_DATA:
	{
		if ((device->receivePos - device->dataStart) >= device->dataLen)
		{
			device->state = STATE_CRC;
		}

		break;
	}
	case STATE_CRC:
	{
		if ((device->receivePos - device->dataStart) >= device->dataLen + 2) // Get last 2 bytes of CRC
		{
			// Check CRC, returns true if OK
			messageReady = receiveDecode(device);

			if(messageReady)
			{
				// Extract the write reg and data for easy use by the application
				if(device->lastReceivedCmd == MODBUS_CMD_SINGLE_WRITE)
				{
					device->writeReg = device->receiveData[device->dataStart + 0] << 8 |  device->receiveData[device->dataStart + 1];
					device->writeData = device->receiveData[device->dataStart + 2] << 8 |  device->receiveData[device->dataStart + 3];
				}
			}

			resetReceiveState(device);
		}

		break;
	}
	}

	return messageReady;
}

void modbus_reset(tModBusDevice *device)
{
	// Try to clear out any old received data
	uartRxCheck(device->uartDev);
	uartResetRxFifo(device->uartDev);

	resetReceiveState(device);
}

bool modbus_checkReceive(tModBusDevice *device)
{
	bool messageReady = false;

	uartRxCheck(device->uartDev);

	while (uartRxCount(device->uartDev))
	{
		uint8_t b = uartGetByte(device->uartDev);

		if (processByte(device, b))
		{
			messageReady = true;
			break;
		}
	}

	return messageReady;
}

static void sendMessage(tModBusDevice *device, uint8_t *msg, int32_t len)
{
	uint16_t crc = calcCrc(msg, len);

	// Modbus is sensitive to gaps in the message, so always send as one whole block
	uint8_t msgWhole[len + 2];

	memcpy(msgWhole, msg, len);
	memcpy(msgWhole + len, &crc, 2);

	uartTxBuffer(device->uartDev, msgWhole, len + 2);
}

void modbus_init(tModBusDevice *device, tUartDevice *uartDev, uint8_t id)
{
	device->uartDev = uartDev;
	device->id = id;
}

void modbus_sendDiag(tModBusDevice *device, uint8_t id)
{
	uint8_t message[] =
	{ device->id, 0x08, 0x00, 0x00, 0x86, 0x31 };

	uint16_t crc = calcCrc(message, sizeof(message));

	uartTxBuffer(device->uartDev, message, sizeof(message));
	uartTxBuffer(device->uartDev, (uint8_t*) &crc, 2);
}

uint16_t modbus_sendMultiReadReg(tModBusDevice *device, uint8_t id, uint16_t regStart, uint16_t num)
{
	const uint8_t cmd = MODBUS_CMD_MULTI_READ;

	uint8_t message[] =
	{ id, cmd, (regStart >> 8) & 0xFF, regStart & 0xFF, (num >> 8) & 0xFF, num & 0xFF};

	sendMessage(device, message, sizeof(message));

	device->lastSentCmd = cmd;
	device->lastReadReg = regStart;
	resetReceiveState(device);

	return 0;
}

void modbus_sendWriteReg(tModBusDevice *device, uint8_t id, uint16_t reg, uint16_t data)
{
	const uint8_t cmd = MODBUS_CMD_SINGLE_WRITE;

	uint8_t message[] =
	{ id, cmd, (reg >> 8) & 0xFF, reg & 0xFF, (data >> 8) & 0xFF, data & 0xFF };

	sendMessage(device, message, sizeof(message));

	device->lastSentCmd = cmd;
	resetReceiveState(device);

	return;
}

