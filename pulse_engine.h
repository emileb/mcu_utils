/*
 * stepper.h
 *
 *  Created on: Apr 6, 2020
 *      Author: emile
 */

#ifndef _PULSE_ENGINE_H_
#define _PULSE_ENGINE_H_

#define NBR_PULSE 1

typedef struct
{
	GPIO_TypeDef * port;
	uint32_t pin;

	int32_t position;
	uint32_t freqency;

	uint32_t countDown;

	int32_t timeLeft;
}tPulseEngine;



void pulse_engine_init( tPulseEngine pulseEngine[], uint32_t nbr );

#endif /* APPLICATION_STEPPER_H_ */
