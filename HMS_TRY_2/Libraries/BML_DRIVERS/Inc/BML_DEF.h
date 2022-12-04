/*
 * BML_DEF.h
 *
 *  Created on: Oct 25, 2022
 *      Author: hp
 */

#ifndef BML_DRIVERS_INC_BML_DEF_H_
#define BML_DRIVERS_INC_BML_DEF_H_


#include "stm32f0xx_hal.h"
#include "stm32f070xb.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include "stdbool.h"
#include "gpio.h"


typedef enum {
	BML_OK = 0x00U, BML_ERROR = 0x01U, BML_BUSY = 0x02U, BML_TIMEOUT = 0x03U
} BML_StatusTypeDef;

#define		LOW		0
#define		HIGH	1

#define freertos_included	1

#if freertos_included == 1
#include "cmsis_os.h"
#define 	Delay(x)								osDelay(x)
#else
#define 	Delay(x)								HAL_Delay(x)
#endif

#define		I2C_Delay(x)							Delay(x)
#define		ADC_Delay(x)							Delay(x)
#define		USART_Delay(x)							Delay(x)
#define		RCC_Delay(x)							Delay(x)


#endif /* BML_DRIVERS_INC_BML_DEF_H_ */
