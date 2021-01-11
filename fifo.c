
// Lockless fifo.

#include "fifo.h"

void fifo_init(fifo_t *f, uint8_t *buf, int32_t size)
{
	f->tail = 0;
	f->head = size - 1;
	f->size = size;
	f->buf = buf;
	f->nbr = 0;
}

int32_t fifo_getNbr(fifo_t *f)
{
	return f->nbr;
}

bool fifo_put(fifo_t *f, uint8_t b)
{
	if (f->nbr >= f->size)
	{
		return false;
	}
	else
	{
		// Move head
		f->head++;
		f->head %= f->size;
		f->buf[f->head] = b;
		f->nbr++;

		return true;
	}
}

uint8_t fifo_get(fifo_t *f)
{
	if (f->nbr == 0)
	{
		return false;
	}
	else
	{
		uint8_t ret = f->buf[f->tail];

		// Move tail
		f->tail++;
		f->tail %= f->size;

		f->nbr--;

		return ret;
	}
}
