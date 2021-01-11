/* Includes ------------------------------------------------------------------*/

#include "stm32f7xx_hal.h"
#include "GUI.h"
#include "stm32f746_touchscreen.h"

#define TS_I2C_ADDRESS                   ((uint16_t)0x70)

static TS_DrvTypeDef *tsDriver;
static uint16_t tsXBoundary, tsYBoundary;
static uint8_t tsOrientation;
static uint8_t I2cAddress;



static I2C_HandleTypeDef *m_I2CHandle;

/**
 * @brief  Initializes and configures the touch screen functionalities and
 *         configures all necessary hardware resources (GPIOs, I2C, clocks..).
 * @param  ts_SizeX: Maximum X size of the TS area on LCD
 * @param  ts_SizeY: Maximum Y size of the TS area on LCD
 * @retval TS_OK if all initializations are OK. Other value if error.
 */
static uint8_t BSP_TS_Init(I2C_HandleTypeDef *i2cDev, uint16_t ts_SizeX, uint16_t ts_SizeY) {
	uint8_t status = TS_OK;

	m_I2CHandle = i2cDev;

	tsXBoundary = ts_SizeX;
	tsYBoundary = ts_SizeY;

	/* Read ID and verify if the touch screen driver is ready */
	ft5336_ts_drv.Init(TS_I2C_ADDRESS);
	if (ft5336_ts_drv.ReadID(TS_I2C_ADDRESS) == FT5336_ID_VALUE) {
		/* Initialize the TS driver structure */
		tsDriver = &ft5336_ts_drv;
		I2cAddress = TS_I2C_ADDRESS;
		tsOrientation = TS_SWAP_XY;

		/* Initialize the TS driver */
		tsDriver->Start(I2cAddress);
	} else {
		status = TS_DEVICE_NOT_FOUND;
	}

	return status;
}

/**
 * @brief  DeInitializes the TouchScreen.
 * @retval TS state
 */
static uint8_t BSP_TS_DeInit(void) {
	/* Actually ts_driver does not provide a DeInit function */
	return TS_OK;
}

/**
 * @brief  Configures and enables the touch screen interrupts.
 * @retval TS_OK if all initializations are OK. Other value if error.
 */
static uint8_t BSP_TS_ITConfig(void) {

	/* Enable the TS ITs */
	tsDriver->EnableIT(I2cAddress);

	return TS_OK;
}

/**
 * @brief  Gets the touch screen interrupt status.
 * @retval TS_OK if all initializations are OK. Other value if error.
 */
static uint8_t BSP_TS_ITGetStatus(void) {
	/* Return the TS IT status */
	return (tsDriver->GetITStatus(I2cAddress));
}

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
/**
 * @brief  Update gesture Id following a touch detected.
 * @param  TS_State: Pointer to touch screen current state structure
 * @retval TS_OK if all initializations are OK. Other value if error.
 */
static uint8_t BSP_TS_Get_GestureId(TS_StateTypeDef *TS_State) {
	uint32_t gestureId = 0;
	uint8_t ts_status = TS_OK;

	/* Get gesture Id */
	ft5336_TS_GetGestureID(I2cAddress, &gestureId);

	/* Remap gesture Id to a TS_GestureIdTypeDef value */
	switch (gestureId) {
	case FT5336_GEST_ID_NO_GESTURE:
		TS_State->gestureId = GEST_ID_NO_GESTURE;
		break;
	case FT5336_GEST_ID_MOVE_UP:
		TS_State->gestureId = GEST_ID_MOVE_UP;
		break;
	case FT5336_GEST_ID_MOVE_RIGHT:
		TS_State->gestureId = GEST_ID_MOVE_RIGHT;
		break;
	case FT5336_GEST_ID_MOVE_DOWN:
		TS_State->gestureId = GEST_ID_MOVE_DOWN;
		break;
	case FT5336_GEST_ID_MOVE_LEFT:
		TS_State->gestureId = GEST_ID_MOVE_LEFT;
		break;
	case FT5336_GEST_ID_ZOOM_IN:
		TS_State->gestureId = GEST_ID_ZOOM_IN;
		break;
	case FT5336_GEST_ID_ZOOM_OUT:
		TS_State->gestureId = GEST_ID_ZOOM_OUT;
		break;
	default:
		ts_status = TS_ERROR;
		break;
	} /* of switch(gestureId) */

	return (ts_status);
}
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

/**
 * @brief  Returns status and positions of the touch screen.
 * @param  TS_State: Pointer to touch screen current state structure
 * @retval TS_OK if all initializations are OK. Other value if error.
 */
static uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State) {
	static uint32_t _x[TS_MAX_NB_TOUCH] = { 0, 0 };
	static uint32_t _y[TS_MAX_NB_TOUCH] = { 0, 0 };
	uint8_t ts_status = TS_OK;
	uint16_t x[TS_MAX_NB_TOUCH];
	uint16_t y[TS_MAX_NB_TOUCH];
	uint16_t brute_x[TS_MAX_NB_TOUCH];
	uint16_t brute_y[TS_MAX_NB_TOUCH];
	uint16_t x_diff;
	uint16_t y_diff;
	uint32_t index;
#if (TS_MULTI_TOUCH_SUPPORTED == 1)
	uint32_t weight = 0;
	uint32_t area = 0;
	uint32_t event = 0;
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

	/* Check and update the number of touches active detected */
	TS_State->touchDetected = tsDriver->DetectTouch(I2cAddress);

	if (TS_State->touchDetected) {
		for (index = 0; index < TS_State->touchDetected; index++) {
			/* Get each touch coordinates */
			tsDriver->GetXY(I2cAddress, &(brute_x[index]), &(brute_y[index]));

			if (tsOrientation == TS_SWAP_NONE) {
				x[index] = brute_x[index];
				y[index] = brute_y[index];
			}

			if (tsOrientation & TS_SWAP_X) {
				x[index] = 4096 - brute_x[index];
			}

			if (tsOrientation & TS_SWAP_Y) {
				y[index] = 4096 - brute_y[index];
			}

			if (tsOrientation & TS_SWAP_XY) {
				y[index] = brute_x[index];
				x[index] = brute_y[index];
			}

			x_diff =
					x[index] > _x[index] ?
							(x[index] - _x[index]) : (_x[index] - x[index]);
			y_diff =
					y[index] > _y[index] ?
							(y[index] - _y[index]) : (_y[index] - y[index]);

			if ((x_diff + y_diff) > 5) {
				_x[index] = x[index];
				_y[index] = y[index];
			}

			if (I2cAddress == FT5336_I2C_SLAVE_ADDRESS) {
				TS_State->touchX[index] = x[index];
				TS_State->touchY[index] = y[index];
			} else {
				/* 2^12 = 4096 : indexes are expressed on a dynamic of 4096 */
				TS_State->touchX[index] = (tsXBoundary * _x[index]) >> 12;
				TS_State->touchY[index] = (tsYBoundary * _y[index]) >> 12;
			}

#if (TS_MULTI_TOUCH_SUPPORTED == 1)

			/* Get touch info related to the current touch */
			ft5336_TS_GetTouchInfo(I2cAddress, index, &weight, &area, &event);

			/* Update TS_State structure */
			TS_State->touchWeight[index] = weight;
			TS_State->touchArea[index] = area;

			/* Remap touch event */
			switch (event) {
			case FT5336_TOUCH_EVT_FLAG_PRESS_DOWN:
				TS_State->touchEventId[index] = TOUCH_EVENT_PRESS_DOWN;
				break;
			case FT5336_TOUCH_EVT_FLAG_LIFT_UP:
				TS_State->touchEventId[index] = TOUCH_EVENT_LIFT_UP;
				break;
			case FT5336_TOUCH_EVT_FLAG_CONTACT:
				TS_State->touchEventId[index] = TOUCH_EVENT_CONTACT;
				break;
			case FT5336_TOUCH_EVT_FLAG_NO_EVENT:
				TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
				break;
			default:
				ts_status = TS_ERROR;
				break;
			} /* of switch(event) */

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

		} /* of for(index=0; index < TS_State->touchDetected; index++) */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
		/* Get gesture Id */
		ts_status = BSP_TS_Get_GestureId(TS_State);
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

	} /* end of if(TS_State->touchDetected != 0) */

	return (ts_status);
}


/**
 * @brief  Clears all touch screen interrupts.
 */
void BSP_TS_ITClear(void) {
	/* Clear TS IT pending bits */
	tsDriver->ClearIT(I2cAddress);
}



/**
 * @brief  Function used to reset all touch data before a new acquisition
 *         of touch information.
 * @param  TS_State: Pointer to touch screen current state structure
 * @retval TS_OK if OK, TE_ERROR if problem found.
 */
uint8_t BSP_TS_ResetTouchData(TS_StateTypeDef *TS_State) {
	uint8_t ts_status = TS_ERROR;
	uint32_t index;

	if (TS_State != (TS_StateTypeDef*) NULL) {
		TS_State->gestureId = GEST_ID_NO_GESTURE;
		TS_State->touchDetected = 0;

		for (index = 0; index < TS_MAX_NB_TOUCH; index++) {
			TS_State->touchX[index] = 0;
			TS_State->touchY[index] = 0;
			TS_State->touchArea[index] = 0;
			TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
			TS_State->touchWeight[index] = 0;
		}

		ts_status = TS_OK;

	} /* of if (TS_State != (TS_StateTypeDef *)NULL) */

	return (ts_status);
}

/********************************* LINK TOUCHSCREEN *********************************/


/**
 * @brief  Manages error callback by re-initializing I2C.
 * @param  i2c_handler : I2C handler
 * @param  Addr: I2C Address
 * @retval None
 */
static void I2Cx_Error(I2C_HandleTypeDef *i2c_handler, uint8_t Addr) {
	/* De-initialize the I2C communication bus */
	HAL_I2C_DeInit(i2c_handler);

	/* Re-Initialize the I2C communication bus */
	HAL_I2C_Init(i2c_handler);
}

/**
 * @brief  Reads multiple data.
 * @param  i2c_handler : I2C handler
 * @param  Addr: I2C address
 * @param  Reg: Reg address
 * @param  MemAddress: Memory address
 * @param  Buffer: Pointer to data buffer
 * @param  Length: Length of the data
 * @retval Number of read data
 */
static HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef *i2c_handler,
		uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer,
		uint16_t Length) {
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(i2c_handler, Addr, (uint16_t) Reg, MemAddress,
			Buffer, Length, 1000);

	/* Check the communication status */
	if (status != HAL_OK) {
		/* I2C error occurred */
		I2Cx_Error(i2c_handler, Addr);
	}
	return status;
}

/**
 * @brief  Writes a value in a register of the device through BUS in using DMA mode.
 * @param  i2c_handler : I2C handler
 * @param  Addr: Device address on BUS Bus.
 * @param  Reg: The target register address to write
 * @param  MemAddress: Memory address
 * @param  Buffer: The target register value to be written
 * @param  Length: buffer size to be written
 * @retval HAL status
 */
static HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef *i2c_handler,
		uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer,
		uint16_t Length) {
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(i2c_handler, Addr, (uint16_t) Reg, MemAddress,
			Buffer, Length, 1000);

	/* Check the communication status */
	if (status != HAL_OK) {
		/* Re-Initiaize the I2C Bus */
		I2Cx_Error(i2c_handler, Addr);
	}
	return status;
}

void TS_IO_Init() {

}

/**
 * @brief  Writes a single data.
 * @param  Addr: I2C address
 * @param  Reg: Reg address
 * @param  Value: Data to be written
 * @retval None
 */
void TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value) {
	I2Cx_WriteMultiple(m_I2CHandle, Addr, (uint16_t) Reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &Value, 1);
}

/**
 * @brief  Reads a single data.
 * @param  Addr: I2C address
 * @param  Reg: Reg address
 * @retval Data to be read
 */
uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg) {
	uint8_t read_value = 0;

	I2Cx_ReadMultiple(m_I2CHandle, Addr, Reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &read_value, 1);

	return read_value;
}

/**
 * @brief  TS delay
 * @param  Delay: Delay in ms
 * @retval None
 */
void TS_IO_Delay(uint32_t Delay) {
	HAL_Delay(Delay);
}

void TouchDriver_Init(I2C_HandleTypeDef *i2cDev) {
	BSP_TS_Init(i2cDev, 800, 480);
}

void TouchDriver_Poll(void) {
	GUI_PID_STATE TS_State; /* Structure that reports the touch state to STemWin */
	static TS_StateTypeDef prev_state; /* Previous touch state from the touch sensor used from BSP package */
	TS_StateTypeDef ts; /* Actual touch state from the touch sensor used from BSP package */
	BSP_TS_GetState(&ts); /* Read the touch state from touch sensor (BSP API)*/
	TS_State.Pressed = ts.touchDetected; /* Store pressed state to STemWin structure*/

	/* Check if the touch is pressed */
	if ((prev_state.touchDetected != ts.touchDetected)) {
		prev_state.touchDetected = ts.touchDetected;
		/* Check touch variations */
		if ((ts.touchX[0] != 0) && (ts.touchY[0] != 0)) {
			prev_state.touchX[0] = ts.touchX[0];
			prev_state.touchY[0] = ts.touchY[0];
		}
		TS_State.Layer = 0;
		TS_State.x = prev_state.touchX[0];
		TS_State.y = prev_state.touchY[0];

		/* Send touch state to STemWin */
		GUI_TOUCH_StoreStateEx(&TS_State);
	}
}

