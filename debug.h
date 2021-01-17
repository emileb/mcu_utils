/*
 * debug.h
 *
 *  Created on: Jan 9, 2021
 *      Author: emile
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "st_uart.h"

#define PRINTF( format, ...) do {debug_write(format, ##__VA_ARGS__); }while(0)

void debug_init(tUartDevice *uartDev);

void debug_write(const char *format, ...);

#endif
