/*
 * timer.h
 *
 *  Created on: 29-Oct-2022
 *      Author: hp
 */

#ifndef BML_DRIVERS_INC_SOFTWARE_TIMER_H_
#define BML_DRIVERS_INC_SOFTWARE_TIMER_H_

#include "BML_DEF.h"

uint16_t ticks = 0;

//============================= TIMER CONFIG FUNCTION=================================//
void timer_config(TIM_TypeDef *tim, uint16_t prescaler_val,
		uint16_t auto_reload_val);
void timer_EN(TIM_TypeDef *tim);
void timer_DI(TIM_TypeDef *tim);
void timer_EVENT(TIM_TypeDef *tim);
uint16_t timer_cnt_val(TIM_TypeDef *tim);
void timer_IT_EN(TIM_TypeDef *tim);
void ticks_timer(uint8_t clkSpeed);
uint16_t ticks_val();
void ticks_reset();
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
//====================================================================================//

#endif /* BML_DRIVERS_INC_SOFTWARE_TIMER_H_ */
