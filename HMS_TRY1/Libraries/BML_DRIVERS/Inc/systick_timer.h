/*
 * systick_timer.h
 *
 *  Created on: 02-Nov-2022
 *      Author: hp
 */

#ifndef BML_DRIVERS_INC_SYSTICK_TIMER_H_
#define BML_DRIVERS_INC_SYSTICK_TIMER_H_

#include "BML_DEF.h"

uint32_t sysTicks = 0;

//============================= SYSTICK CONFIG FUNCTION===============================//
void sysDelayInit();
void sysDelay(uint32_t ms);
void sysDelayReset();
uint32_t sysDelayVal();
void sysIncrementTicks();
void SysTick_Handler(void);
//====================================================================================//

#endif /* BML_DRIVERS_INC_SYSTICK_TIMER_H_ */
