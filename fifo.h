/*
 * fifo.h
 *
 *  Created on: Jan 9, 2021
 *      Author: emile
 */

#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	uint8_t *buf;
	int32_t head;
	int32_t tail;
	int32_t size;
	int32_t nbr;
} fifo_t;

void fifo_init(fifo_t *f, uint8_t *buf, int32_t size);

int32_t fifo_getNbr(fifo_t *f);

bool fifo_put(fifo_t *f, uint8_t b);

uint8_t fifo_get(fifo_t *f);

#endif /* FIFO_H_ */
