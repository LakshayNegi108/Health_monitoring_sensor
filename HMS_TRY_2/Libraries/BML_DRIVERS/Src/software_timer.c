/*
 * timer.c
 *
 *  Created on: 29-Oct-2022
 *      Author: hp
 */
#include <software_timer.h>

//************************** Basic TIMER FUNCTIONS *******************************//
void timer_config(TIM_TypeDef *tim, uint16_t prescaler_val,
		uint16_t auto_reload_val) {

	if (tim == TIM1) {						// ENable clock for particular timer
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	} else if (tim == TIM3) {
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	} else if (tim == TIM6) {
		RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	} else if (tim == TIM7) {
		RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	} else if (tim == TIM14) {
		RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	} else if (tim == TIM15) {
		RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	} else if (tim == TIM16) {
		RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	} else if (tim == TIM17) {
		RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	}

	// Peripheral clock/Desired clk = PSC x ARR = Value

	tim->ARR = auto_reload_val;	// fill auto-reload register with value: auto_reload_val
	tim->PSC = prescaler_val;// fill prescale value register with value: prescaler_val

}

void timer_EN(TIM_TypeDef *tim) {
	tim->CR1 |= TIM_CR1_CEN;		// Enable counter to count the value
}

void timer_DI(TIM_TypeDef *tim) {
	tim->CR1 &= ~TIM_CR1_CEN;		// Disable counter to count the value
}

void timer6_7_CR1(TIM_TypeDef *tim, uint8_t AUTO_RELOAD_PREL_EN,// Function to configure the rest of the timer functions
		uint8_t ONE_PULSE_MODE, uint8_t UPDATE_REQ_SRC, uint8_t UPDATE_DIS) {
	if (AUTO_RELOAD_PREL_EN == 1) {
		tim->CR1 |= TIM_CR1_ARPE;
	}
	if (ONE_PULSE_MODE == 1) {
		tim->CR1 |= TIM_CR1_OPM;
	}
	if (UPDATE_REQ_SRC == 1) {
		tim->CR1 |= TIM_CR1_URS;
	}
	if (UPDATE_DIS == 1) {
		tim->CR1 |= TIM_CR1_UDIS;
	}
}

void timer_EVENT(TIM_TypeDef *tim) {
	tim->EGR |= TIM_EGR_UG;	// Generate software event to reset counter timer
}

uint16_t timer_cnt_val(TIM_TypeDef *tim) {
	uint16_t val = tim->CNT;		// counter register value
	return val;
}

void timer_IT_EN(TIM_TypeDef *tim) {
	tim->DIER |= TIM_DIER_UIE;
}

void ticks_timer(uint8_t clkSpeed) {			// similar to millis
	if (clkSpeed == 8) {
		timer_config(TIM6, 0, 7499);//Initialize timer with configuration to 1 milli second
	} else if (clkSpeed == 48) {
		timer_config(TIM6, 45, 999);//Initialize timer with configuration to 1 milli second
	}
	timer_IT_EN(TIM6);
	NVIC_EnableIRQ(TIM6_IRQn);
	timer_EN(TIM6);
}
uint16_t ticks_val() {
	return ticks;
}
void ticks_reset() {
	ticks = 0;
}
void TIM6_IRQHandler(void) {		// Timer Interrupt helping in delay
	ticks++;
	TIM6->SR &= ~TIM_SR_UIF;
}

void delay_ms(uint32_t ms) {		// blocking mode delay function
	timer_config(TIM6, 0, 47899);
	timer_IT_EN(TIM6);
	NVIC_EnableIRQ(TIM6_IRQn);

	timer_EN(TIM6);	// good for turning timer off when not in need to save power
	ticks = 0;
	while (ticks < ms)
		;
	timer_DI(TIM6);
}
