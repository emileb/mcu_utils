
#include "fifo.h"
#include "st_uart.h"
#include "debug.h"

#define TIMEOUT 1000


static void pushIntoFifo(tUartDevice *uartDev, uint8_t *data, size_t size)
{
	for (int32_t n = 0; n < size; n++)
	{
		fifo_put(&uartDev->fifo, data[n]);
	}
}

void uartInit(tUartDevice *uartDev, UART_HandleTypeDef *uart, DMA_HandleTypeDef *dmaRx, DMA_HandleTypeDef *dmaTx)
{

	uartDev->uart = uart;
	uartDev->dmaRx = dmaRx;
	uartDev->dmaTx = dmaTx;
	uartDev->dmaOldPos = 0;
	fifo_init(&uartDev->fifo, uartDev->fifoBuffer, RX_FIFO_SIZE);

}

void uartStartRx(tUartDevice *uartDev)
{
	__HAL_DMA_ENABLE_IT (uartDev->dmaRx, DMA_IT_TC);

	HAL_UART_Receive_DMA(uartDev->uart, uartDev->dmaRxBuffer, RX_DMA_SIZE);
}

void uartRxCheck(tUartDevice *uartDev)
{

	size_t pos;

	pos = RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(uartDev->dmaRx);

	if (pos != uartDev->dmaOldPos)
	{
		if (pos > uartDev->dmaOldPos)
		{
			pushIntoFifo(uartDev, &uartDev->dmaRxBuffer[uartDev->dmaOldPos], pos - uartDev->dmaOldPos);
		}
		else
		{

			pushIntoFifo(uartDev, &uartDev->dmaRxBuffer[uartDev->dmaOldPos], RX_DMA_SIZE - uartDev->dmaOldPos);

			if (pos > 0)
			{
				pushIntoFifo(uartDev, &uartDev->dmaRxBuffer[0], pos);
			}
		}
	}

	uartDev->dmaOldPos = pos;

	if (uartDev->dmaOldPos == RX_DMA_SIZE)
	{
		uartDev->dmaOldPos = 0;
	}
}

size_t uartRxCount(tUartDevice *uartDev)
{
	return fifo_getNbr(&uartDev->fifo);
}

uint8_t uartGetByte(tUartDevice *uartDev)
{
	return fifo_get(&uartDev->fifo);
}

bool uartTxBuffer(tUartDevice *uartDev, uint8_t *data, size_t len)
{
	if(uartDev->dmaTx)
	{
		memcpy(uartDev->dmaTxBuffer, data, len);

		HAL_UART_Transmit_DMA(uartDev->uart, uartDev->dmaTxBuffer, len);
	}
	else
	{
		HAL_UART_Transmit(uartDev->uart, data, len, TIMEOUT);
	}
	return true;
}


void uartResetRxFifo(tUartDevice *uartDev)
{
	fifo_init(&uartDev->fifo, uartDev->fifo.buf, uartDev->fifo.size);
}
