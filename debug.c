
#include <stdarg.h>
#include <stdio.h>

#include "debug.h"


#define MAX_MESSAGE_LEN 256

static tUartDevice *m_uartDev;

static uint8_t m_buffer[MAX_MESSAGE_LEN];

void debug_init(tUartDevice *uartDev)
{
	m_uartDev = uartDev;
}

void debug_write(const char* format, ...)
{
	va_list list;

	va_start(list, format);

	int32_t len = vsnprintf((char*)m_buffer, MAX_MESSAGE_LEN, format, list);

	uartTxBuffer(m_uartDev, m_buffer, len);
}
