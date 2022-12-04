/*
 * uart.c
 *
 *  Created on: 29-Oct-2022
 *      Author: hp
 */
#include "uart.h"

void USART_INIT(USART_TypeDef *usart, GPIO_TypeDef *tx_port,
		GPIO_TypeDef *rx_port, uint32_t tx_pinNumber, uint32_t rx_pinNumber,
		uint32_t bd_rate) {

	USART_CLK_EN(usart);

	USART_PIN_CONFIG(usart, tx_port, rx_port, tx_pinNumber, rx_pinNumber);

	USART_BDR_SEL(usart, bd_rate);

	usart->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

}

void USART_IT_EN(USART_TypeDef *usart, uint8_t TX_IT, uint8_t RX_IT,
		uint8_t IDLE_IT) {
	if (usart == USART1) {
		NVIC_EnableIRQ(USART1_IRQn);
	} else if (usart == USART2) {
		NVIC_EnableIRQ(USART2_IRQn);
	} else if (usart == USART3 || usart == USART4) {
		NVIC_EnableIRQ(USART3_4_IRQn);
	}

	if (TX_IT) {
		usart->CR1 |= USART_CR1_TXEIE;
	}
	if (RX_IT) {
		usart->CR1 |= USART_CR1_RXNEIE;
	}
	if (IDLE_IT) {
		usart->CR1 |= USART_CR1_IDLEIE;
	}
}
// Initialize UART pins, Baudrate
void USART_CLK_EN(USART_TypeDef *usart) {
	if (usart == USART1) {
		USART1_CLK_EN;
	} else if (usart == USART2) {
		USART2_CLK_EN;
	} else if (usart == USART3) {
		USART3_CLK_EN;
	} else if (usart == USART4) {
		USART4_CLK_EN;
	}
}

void USART_PIN_CONFIG(USART_TypeDef *usart, GPIO_TypeDef *tx_port,
		GPIO_TypeDef *rx_port, uint32_t tx_pinNumber, uint32_t rx_pinNumber) {

	gpio_config(tx_port, tx_pinNumber, ALT_MODE, HIGH_SPEED, DI_PUPD, PHPL);
	gpio_config(rx_port, rx_pinNumber, ALT_MODE, HIGH_SPEED, DI_PUPD, PHPL);

	if (usart == USART1) {
		// TX
		if (tx_port == PORTA && tx_pinNumber == 9) {
			gpio_altfn(tx_port, tx_pinNumber, USART1_TX_PA9);
		} else if (tx_port == PORTB && tx_pinNumber == 6) {
			gpio_altfn(tx_port, tx_pinNumber, USART1_TX_PB6);
		}

		// RX
		if (rx_port == PORTA && rx_pinNumber == 10) {
			gpio_altfn(rx_port, rx_pinNumber, USART1_RX_PA10);
		} else if (rx_port == PORTB && rx_pinNumber == 7) {
			gpio_altfn(rx_port, rx_pinNumber, USART1_RX_PB7);
		}
	} else if (usart == USART2) {
		// TX
		if (tx_port == PORTA && tx_pinNumber == 2) {
			gpio_altfn(tx_port, tx_pinNumber, USART2_TX_PA2);
		} else if (tx_port == PORTA && tx_pinNumber == 14) {
			gpio_altfn(tx_port, tx_pinNumber, USART2_TX_PA14);
		}

		// RX
		if (rx_port == PORTA && rx_pinNumber == 3) {
			gpio_altfn(rx_port, rx_pinNumber, USART2_RX_PA3);
		} else if (rx_port == PORTA && rx_pinNumber == 15) {
			gpio_altfn(rx_port, rx_pinNumber, USART2_RX_PA15);
		}
	} else if (usart == USART3) {
		// TX
		if (tx_port == PORTC && tx_pinNumber == 4) {
			gpio_altfn(tx_port, tx_pinNumber, USART3_TX_PC4);
		} else if (tx_port == PORTB && tx_pinNumber == 10) {
			gpio_altfn(tx_port, tx_pinNumber, USART3_TX_PB10);
		} else if (tx_port == PORTC && tx_pinNumber == 10) {
			gpio_altfn(tx_port, tx_pinNumber, USART3_TX_PC10);
		}

		// RX
		if (rx_port == PORTC && rx_pinNumber == 5) {
			gpio_altfn(rx_port, rx_pinNumber, USART3_RX_PC5);
		} else if (rx_port == PORTC && rx_pinNumber == 11) {
			gpio_altfn(rx_port, rx_pinNumber, USART3_RX_PC11);
		} else if (rx_port == PORTB && rx_pinNumber == 11) {
			gpio_altfn(rx_port, rx_pinNumber, USART3_RX_PB11);
		}
	} else if (usart == USART4) {
		gpio_altfn(tx_port, tx_pinNumber, USART4_TX_PA0);
		gpio_altfn(rx_port, rx_pinNumber, USART4_RX_PA1);
	}
}

void USART_BDR_SEL(USART_TypeDef *usart, uint32_t bd_rate) {

//==============================================================//
// clkPer / (baudR x 16) = 48MHz / (9600 x 16) = 312.5 = 1388   //
//==============================================================//
	uint32_t baud = (uint32_t) (SystemCoreClock / bd_rate);
	usart->BRR |= baud;
}

/* Write a character to USART1 */
BML_StatusTypeDef USART_WRITE(USART_TypeDef *usart, uint8_t *ch, uint32_t len, uint16_t timeout) {
	uint16_t i = timeout;
	for (uint8_t i = 0; i < len; i++) {
		if (len == 1) {
			usart->TDR = ch;
		} else {
			usart->TDR = ch[i];
		}
		while (!(usart->ISR & USART_ISR_TXE)){
//			USART_Delay(1);
//			i--;
//			if(i <= 0){
//				return BML_TIMEOUT;
//			}
		}
	}
	return BML_OK;
}

/* Read a character from USART */
uint8_t USART_READ(USART_TypeDef *usart) {
	uint8_t chr = 0;
	if ((usart->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		chr = (uint8_t) (usart->RDR); /* Receive data, clear flag */
	}
	return chr;
}

//******************************* UART SERIAL PRINTING ***************************************//

void print(const char *fmt, ...) {
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	uint16_t len = strlen(buffer);
	USART_WRITE(UART_TYPE, buffer, len, 1000);

}

void uart_print_config(uint32_t baudRate) {
	USART_INIT(UART_TYPE, UART_TX_PORT, UART_RX_PORT, UART_TX_PIN, UART_RX_PIN, baudRate);
}

//*******************************************************************************************//
