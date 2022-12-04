/*
 * adc.h
 *
 *  Created on: Oct 28, 2022
 *      Author: hp
 */

#ifndef BML_DRIVERS_INC_ADC_H_
#define BML_DRIVERS_INC_ADC_H_

#include "BML_DEF.h"

//=============| Definitions for ADC| ================//

#define sample_time_0	(uint8_t)0x00
#define sample_time_1	(uint8_t)0x01
#define sample_time_2	(uint8_t)0x02
#define sample_time_3	(uint8_t)0x03
#define sample_time_4	(uint8_t)0x04
#define sample_time_5	(uint8_t)0x05
#define sample_time_6	(uint8_t)0x06
#define sample_time_7	(uint8_t)0x07

#define	DISCEN_off		(uint8_t)0x00
#define	AUTOFF_off		(uint8_t)0x00
#define	WAIT_off		(uint8_t)0x00
#define	CONT_off		(uint8_t)0x00
#define	OVRMOD_off		(uint8_t)0x00
#define SCANDIR_off		(uint8_t)0x00

#define	DISCEN_on		(uint8_t)0x01
#define	AUTOFF_on		(uint8_t)0x01
#define	WAIT_on			(uint8_t)0x01
#define	CONT_on			(uint8_t)0x01
#define	OVRMOD_on		(uint8_t)0x01
#define SCANDIR_on		(uint8_t)0x01

#define ADC_EOC_FLAG		(ADC1->ISR & ADC_ISR_EOC)
#define ADC_AWD_FLAG		(ADC1->ISR & ADC_ISR_AWD)
#define ADC_OVR_FLAG		(ADC1->ISR & ADC_ISR_OVR)
#define ADC_EOSEQ_FLAG		(ADC1->ISR & ADC_ISR_EOSEQ)
#define ADC_EOSMP_FLAG		(ADC1->ISR & ADC_ISR_EOSMP)
//====================================================//


typedef enum{
	analog_watchdog_flag, ADC_overrun_flag, end_of_seq_flag, end_of_conv_flag, end_of_samp_flag
} flag_type;

BML_StatusTypeDef adc_calib(uint16_t timeout);
BML_StatusTypeDef adc_ready(uint16_t timeout);
BML_StatusTypeDef adc_clk_config(uint16_t timeout);
void adc_sample_time(uint8_t sample_time);
BML_StatusTypeDef adc_en(uint16_t timeout);
BML_StatusTypeDef adc_read(uint16_t *arr, uint8_t arr_len, uint16_t timeout);
void adc_start();
void adc_stop();
BML_StatusTypeDef adc_disable(uint16_t timeout);
void adc_chsel(GPIO_TypeDef *port, uint16_t pin);
void adc_config(uint8_t sample_time, uint8_t CONT, uint8_t SCANDIR,
		uint8_t AUTOFF, uint8_t WAIT, uint8_t DISCEN, uint8_t OVRMOD);
void adc_IT_config(uint8_t flag_type);


#endif /* BML_DRIVERS_INC_ADC_H_ */
