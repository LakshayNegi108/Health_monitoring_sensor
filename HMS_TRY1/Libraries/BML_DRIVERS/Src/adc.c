/*
 * adc.c
 *
 *  Created on: Oct 28, 2022
 *      Author: hp
 */
#include "adc.h"

BML_StatusTypeDef adc_calib(uint16_t timeout) {
	// Calibrate ADC
	uint16_t i = timeout;
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0) {
		ADC_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
		/* For robust implementation, add here time-out management */
	}

	i = timeout;

	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{
		ADC_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	}
	return BML_OK;
}
BML_StatusTypeDef adc_ready(uint16_t timeout) {
	uint16_t i = timeout;
	// ADC ENABLE
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
	{
		ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADEN; /* (3) */
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
	{
		ADC_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
		/* For robust implementation, add here time-out management */
	}
	return BML_OK;
}
BML_StatusTypeDef adc_clk_config(uint16_t timeout) {
	uint16_t i = timeout;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* (1) */
	RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
	{
		ADC_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
		/* For robust implementation, add here time-out management */
	}
	return BML_OK;
}
void adc_sample_time(uint8_t sample_time) {
	ADC1->SMPR |= sample_time;
}
BML_StatusTypeDef adc_en(uint16_t timeout) {
	if (adc_clk_config(timeout) == BML_OK && adc_calib(timeout) == BML_OK
			&& adc_ready(timeout) == BML_OK) {
		return BML_OK;
	}
	return BML_ERROR;
}
BML_StatusTypeDef adc_read(uint16_t *arr, uint8_t arr_len, uint16_t timeout) {
	uint16_t i = timeout;

	ADC1->CR |= ADC_CR_ADSTART;

	for (int i = 0; i < arr_len; i++) {
		while ((ADC1->ISR & ADC_ISR_EOC) == LOW) {
//			ADC_Delay(1);
//			i--;
//			if (i <= 0) {
//				return BML_TIMEOUT;
//			}
		}
		*(arr + i) = ADC1->DR;
//		i = timeout;
	}
	ADC1->CR |= ADC_CR_ADSTP;
	return BML_OK;
}

void adc_start() {
	ADC1->CR |= ADC_CR_ADSTART;
}
void adc_stop() {
	ADC1->CR |= ADC_CR_ADSTP;
}
BML_StatusTypeDef adc_disable(uint16_t timeout) {
	uint16_t i = timeout;
	ADC1->CR |= ADC_CR_ADSTP; /* (1) */
	while ((ADC1->CR & ADC_CR_ADSTP) != 0) /* (2) */
	{
		ADC_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
		/* For robust implementation, add here time-out management */
	}
	i = timeout;
	ADC1->CR |= ADC_CR_ADDIS; /* (3) */
	while ((ADC1->CR & ADC_CR_ADEN) != 0) /* (4) */
	{
		ADC_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
		/* For robust implementation, add here time-out management */
	}
}
void adc_chsel(GPIO_TypeDef *port, uint16_t pin) {

	if (port == GPIOA) {
		switch (pin) {
		case 0:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL0;
			break;
		case 1:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL1;
			break;
		case 2:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL2;
			break;
		case 3:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL3;
			break;
		case 4:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL4;
			break;
		case 5:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL5;
			break;
		case 6:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL6;
			break;
		case 7:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL7;
			break;
		default:
			break;
		}
	} else if (port == GPIOB) {
		switch (pin) {
		case 0:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL8;
			break;
		case 1:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL9;
			break;
		default:
			break;
		}
	} else if (port == GPIOC) {
		switch (pin) {
		case 0:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
			break;
		case 1:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL11;
			break;
		case 2:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL12;
			break;
		case 3:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL13;
			break;
		case 4:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL14;
			break;
		case 5:
			ADC1->CHSELR |= ADC_CHSELR_CHSEL15;
			break;
		default:
			break;
		}
	}
}
void adc_config(uint8_t sample_time, uint8_t CONT, uint8_t SCANDIR,
		uint8_t AUTOFF, uint8_t WAIT, uint8_t DISCEN, uint8_t OVRMOD) {
	adc_sample_time(sample_time);
	if (DISCEN && !CONT) {
		ADC1->CFGR1 |= ADC_CFGR1_DISCEN;
	}
	if (AUTOFF) {
		ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;
	}
	if (WAIT) {
		ADC1->CFGR1 |= ADC_CFGR1_WAIT;
	}
	if (CONT && !DISCEN) {
		ADC1->CFGR1 |= ADC_CFGR1_CONT;
	}
	if (OVRMOD) {
		ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;
	}
	if (SCANDIR) {
		ADC1->CFGR1 |= ADC_CFGR1_SCANDIR;
	}
}
void adc_IT_config(uint8_t flag_type) {
	if (flag_type == analog_watchdog_flag) {
		ADC1->IER |= ADC_IER_AWDIE;
	} else if (flag_type == ADC_overrun_flag) {
		ADC1->IER |= ADC_IER_OVRIE;
	} else if (flag_type == end_of_seq_flag) {
		ADC1->IER |= ADC_IER_EOSIE;
	} else if (flag_type == end_of_conv_flag) {
		ADC1->IER |= ADC_IER_EOCIE;
	} else if (flag_type == end_of_samp_flag) {
		ADC1->IER |= ADC_IER_EOSMPIE;
	}
	NVIC_EnableIRQ(ADC1_IRQn);
}
