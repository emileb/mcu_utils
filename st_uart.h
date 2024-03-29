
#ifndef ST_UART_H_
#define ST_UART_H_


#include "fifo.h"

#if  defined(STM32F746xx)
	#include "stm32f7xx_hal.h"
#elif defined(STM32G431xx)
	#include "stm32g4xx_hal.h"
#elif defined(STM32F302x8)
	#include "stm32f3xx_hal.h"
#elif defined(STM32G474xx)
	#include "stm32g4xx_hal.h"
#endif

#define RX_DMA_SIZE 64
#define RX_FIFO_SIZE 64

typedef struct
{

	UART_HandleTypeDef *uart;
	DMA_HandleTypeDef *dmaRx;
	DMA_HandleTypeDef *dmaTx;
	uint8_t dmaTxBuffer[RX_DMA_SIZE];
	uint8_t dmaRxBuffer[RX_DMA_SIZE];
	size_t dmaOldPos;
	fifo_t fifo;
	uint8_t fifoBuffer[RX_FIFO_SIZE];

} tUartDevice;

void uartInit(tUartDevice *uartDev, UART_HandleTypeDef *usart, DMA_HandleTypeDef *dmaRx, DMA_HandleTypeDef *dmaTx);

void uartStartRx(tUartDevice *uartDev);

void uartRxCheck(tUartDevice *uartDev);

size_t uartRxCount(tUartDevice *uartDev);

uint8_t uartGetByte(tUartDevice *uartDev);

bool uartTxBuffer(tUartDevice *uartDev, uint8_t *data, size_t len);

void uartResetRxFifo(tUartDevice *uartDev);

#endif
