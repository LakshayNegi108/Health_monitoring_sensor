/*
 * uart.h
 *
 *  Created on: 29-Oct-2022
 *      Author: hp
 */

#ifndef BML_DRIVERS_INC_UART_H_
#define BML_DRIVERS_INC_UART_H_

#include <gpio.h>
#include "BML_DEF.h"

//=============| Definitions for USART Specific Functions| ================//
#define USART1_CLK_EN	(RCC->APB1ENR |= RCC_APB2ENR_USART1EN)
#define USART2_CLK_EN	(RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#define USART3_CLK_EN	(RCC->APB1ENR |= RCC_APB1ENR_USART3EN)
#define USART4_CLK_EN	(RCC->APB1ENR |= RCC_APB1ENR_USART4EN)
//=========================================================================//

typedef struct{
	USART_TypeDef *usart;
	uint32_t	BAUD_RATE;
}USART_TYPE;

#define UART_TYPE			USART2
#define UART_TX_PORT		PORTA
#define UART_RX_PORT		PORTA
#define UART_TX_PIN			2
#define UART_RX_PIN			3

void USART_CLK_EN(USART_TypeDef *usart);
void USART_PIN_CONFIG(USART_TypeDef *usart, GPIO_TypeDef *tx_port,
		GPIO_TypeDef *rx_port, uint32_t tx_pinNumber, uint32_t rx_pinNumber);
void USART_BDR_SEL(USART_TypeDef *usart, uint32_t bd_rate);
void USART_INIT(USART_TypeDef *usart, GPIO_TypeDef *tx_port,
		GPIO_TypeDef *rx_port, uint32_t tx_pinNumber, uint32_t rx_pinNumber,
		uint32_t bd_rate);
void USART_IT_EN(USART_TypeDef *usart, uint8_t TX_IT, uint8_t RX_IT, uint8_t IDLE_IT);
BML_StatusTypeDef USART_WRITE(USART_TypeDef *usart, uint8_t *ch, uint32_t len, uint16_t timeout);
uint8_t USART_READ(USART_TypeDef *usart);

//============================= SERIAL PRINTING FUNCTION==============================//
void uart_print_config(uint32_t baudRate);
void print(const char *fmt, ...);
//====================================================================================//

#endif /* BML_DRIVERS_INC_UART_H_ */
