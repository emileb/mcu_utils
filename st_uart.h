#include "fifo.h"

#define RX_DMA_SIZE 64

#define RX_FIFO_SIZE 64

typedef struct
{

	UART_HandleTypeDef *usart;
	DMA_HandleTypeDef *dma;
	uint8_t dmaRxBuffer[RX_DMA_SIZE];
	size_t dmaOldPos;
	fifo_t fifo;
	uint8_t fifoBuffer[RX_FIFO_SIZE];

} tUartDevice;

void uartInit(tUartDevice *uartDev, UART_HandleTypeDef *usart, DMA_HandleTypeDef *dma);

void uartStartRx(tUartDevice *uartDev);

void uartRxCheck(tUartDevice *uartDev);

size_t uartRxCount(tUartDevice *uartDev);

uint8_t uartGetByte(tUartDevice *uartDev);

bool uartTxBuffer(tUartDevice *uartDev, uint8_t *data, size_t len);