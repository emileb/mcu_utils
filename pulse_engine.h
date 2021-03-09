/*
 * stepper.h
 *
 *  Created on: Apr 6, 2020
 *      Author: emile
 */

#ifndef APPLICATION_STEPPER_H_
#define APPLICATION_STEPPER_H_

#define NBR_PULSE 1

typedef struct
{
	GPIO_TypeDef * port;
	uint32_t pin;

	int32_t position;
	uint32_t freqency;
	int32_t timeLeft;
}tPulseEngine;

extern volatile tPulseEngine pulseEngine[NBR_PULSE];

#endif /* APPLICATION_STEPPER_H_ */
