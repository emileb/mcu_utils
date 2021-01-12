/*
 * STemWin.h
 *
 *  Created on: Jan 11, 2021
 *      Author: emile
 */


void GRAPHICS_Init(void);

void GRAPHICS_IncTick(void);


void TouchDriver_Init(I2C_HandleTypeDef *i2cDev);

void TouchDriver_Poll(void);
