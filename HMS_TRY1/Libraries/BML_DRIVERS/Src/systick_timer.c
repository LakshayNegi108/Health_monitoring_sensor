/*
 * systick_timer.c
 *
 *  Created on: 02-Nov-2022
 *      Author: hp
 */
#include "systick_timer.h"


void sysDelayInit() {
	SysTick_Config(SystemCoreClock / 1000);
}

void sysDelay(uint32_t ms) {
	sysTicks = 0;
	while (sysTicks < ms)
		;
}
void sysDelayReset() {
	sysTicks = 0;
}

uint32_t sysDelayVal() {
	return sysTicks;
}

void sysIncrementTicks() {
	sysTicks++;
}

//void SysTick_Handler(void){		// Comment out Systick_handler in HAL_IT file
//	sysIncrementTicks();
//}

