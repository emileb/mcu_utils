/*
 * stepper.c
 *
 *  Created on: Apr 6, 2020
 *      Author: emile
 */


#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_bus.h"

#include "stepper.h"

volatile tPulseEngine pulseEngine[NBR_PULSE] = {0};


void gpio_init()
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	for( int n = 0; n < NBR_PULSE; n++ )
	{
		LL_GPIO_SetPinMode(pulseEngine[n].port, pulseEngine[n].pin, LL_GPIO_MODE_OUTPUT);
		LL_GPIO_SetPinPull(pulseEngine[n].port, pulseEngine[n].pin, LL_GPIO_PULL_NO);
		LL_GPIO_SetPinSpeed(pulseEngine[n].port, pulseEngine[n].pin, LL_GPIO_SPEED_FREQ_HIGH);
	}
}


void timer_init()
{
	pulseEngine[0].port = GPIOB;
	pulseEngine[0].pin = LL_GPIO_PIN_3;
	pulseEngine[0].freqency = 1000;

	//pulseEngine[1].port = GPIOC;
	//pulseEngine[1].pin = LL_GPIO_PIN_3;
	//pulseEngine[1].freqency = 18332;


	gpio_init();


	/***********************************************/
	/* Configure the NVIC to handle TIM2 interrupt */
	/***********************************************/
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);

	/******************************/
	/* Peripheral clocks enabling */
	/******************************/
	/* Enable the timer peripheral clock */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

	/***************************/
	/* Time base configuration */
	/***************************/
	/* Set counter mode */
	/* Reset value is LL_TIM_COUNTERMODE_UP */
	//LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);

	/* Set the pre-scaler value to have TIM2 counter clock equal to 10 kHz */
	LL_TIM_SetPrescaler(TIM2, __LL_TIM_CALC_PSC(SystemCoreClock, 1000000));

	LL_TIM_SetPrescaler(TIM2,0);


	/* Enable TIM2_ARR register preload. Writing to or reading from the         */
	/* auto-reload register accesses the preload register. The content of the   */
	/* preload register are transferred into the shadow register at each update */
	/* event (UEV).                                                             */
	//LL_TIM_EnableARRPreload(TIM2); // DO NOT TURN THIS ON


	/* Set the auto-reload value to have a counter frequency of 100 Hz */
	/* TIM2CLK = SystemCoreClock / (APB prescaler & multiplier)               */
	uint32_t TimOutClock = SystemCoreClock/1;
	LL_TIM_SetAutoReload(TIM2, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM2), 10000));

	LL_TIM_SetAutoReload(TIM2,65535);

	/*********************************/
	/* Output waveform configuration */
	/*********************************/
	/* Set output mode */
	/* Reset value is LL_TIM_OCMODE_FROZEN */
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);

	/* Set output channel polarity */
	/* Reset value is LL_TIM_OCPOLARITY_HIGH */
	//LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);

	/* Set compare value to half of the counter period (50% duty cycle ) */
	//LL_TIM_OC_SetCompareCH2(TIM2, ( (LL_TIM_GetAutoReload(TIM2) + 1 ) / 4));

	/* Enable TIM2_CCR1 register preload. Read/Write operations access the      */
	/* preload register. TIM2_CCR1 preload value is loaded in the active        */
	/* at each update event.                                                    */
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH2);

	/**************************/
	/* TIM2 interrupts set-up */
	/**************************/
	/* Enable the capture/compare interrupt for channel 1*/
	LL_TIM_EnableIT_UPDATE(TIM2);
	 LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);

	 LL_TIM_DisableMasterSlaveMode(TIM2);
	/**********************************/
	/* Start output signal generation */
	/**********************************/
	/* Enable output channel 1 */
	//LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);

	/* Enable counter */
	LL_TIM_EnableCounter(TIM2);

	/* Force update generation */
	LL_TIM_GenerateEvent_UPDATE(TIM2);

}

#define TICK_FREQ 72000000

#define MAX_TICKS 65535

#define MIN_TICKS 700

int32_t lastTime = 0;

void TIM2_IRQHandler(void)
{

	/* Check whether CC1 interrupt is pending */
	if(LL_TIM_IsActiveFlag_UPDATE(TIM2) == 1)
	{
		/* Clear the update interrupt flag*/
		LL_TIM_ClearFlag_UPDATE(TIM2);
	}

	// SET DIRECTION HERE

	int32_t nextUpdateTime = MAX_TICKS + 1;

	for( int n = 0; n < NBR_PULSE; n++ )
	{
		// Take off the time just passed
		pulseEngine[n].timeLeft -= lastTime;

		if( pulseEngine[n].timeLeft <= 0)
		{
			LL_GPIO_SetOutputPin(pulseEngine[n].port, pulseEngine[n].pin);

			// Save position
			pulseEngine[n].position++;

			// Calculate number of ticks needed
			uint32_t nextTick = TICK_FREQ / pulseEngine[n].freqency;

			// Note timeLeft might be negative
			pulseEngine[n].timeLeft += nextTick;
		}

		int32_t timeNext;
		if(pulseEngine[n].timeLeft >= MAX_TICKS * 2)
		{
			timeNext = MAX_TICKS;
		}
		else if(pulseEngine[n].timeLeft > MAX_TICKS) // Avoid a small value if timeLeft is just above MAX_TICKS
		{
			timeNext = pulseEngine[n].timeLeft / 2;
		}
		else
		{
			timeNext = pulseEngine[n].timeLeft;
		}

		// Find if this is the shortest time to wait
		if(timeNext < nextUpdateTime)
			nextUpdateTime = timeNext;

	}

	// Timer duration can never be shorter than the time this IRQ takes to run
	// This will cause a small amount of jitter on the signal
	if( nextUpdateTime < MIN_TICKS )
		nextUpdateTime = MIN_TICKS;

	LL_TIM_SetAutoReload(TIM2, nextUpdateTime);
	lastTime = nextUpdateTime;

	for( int n = 0; n < NBR_PULSE; n++ )
	{
		// Falling edge
		LL_GPIO_ResetOutputPin(pulseEngine[n].port, pulseEngine[n].pin);
	}
}
