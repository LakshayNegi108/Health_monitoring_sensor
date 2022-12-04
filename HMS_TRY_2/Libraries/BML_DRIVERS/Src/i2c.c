/*
 * i2c.c
 *
 *  Created on: Oct 23, 2022
 *      Author: hp
 */
#include "i2c.h"

void I2C_EN(I2C_TypeDef *i2c) {
	i2c->CR1 |= I2C_CR1_PE;
}

void I2C_DI(I2C_TypeDef *i2c) {
	i2c->CR1 &= ~I2C_CR1_PE;
}

void I2C_INIT(I2C_TypeDef *i2c, uint32_t timing) {
	if (i2c == I2C1) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	} else if (i2c == I2C2) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}
	i2c->CR1 = 0; /* software reset I2C1 */
	i2c->TIMINGR = timing; /* 100 KHz, peripheral clock is 8/48 MHz */
	I2C_EN(i2c);
}

void I2C_DEINIT(I2C_TypeDef *i2c) {

	i2c->CR1 = 0; /* software reset I2C1 */
	i2c->TIMINGR = 0; /* 100 KHz, peripheral clock is 8/48 MHz */

	if (i2c == I2C1) {
		RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
	} else if (i2c == I2C2) {
		RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
	}
}

void I2C_CR2(I2C_TypeDef *i2c, uint32_t val) {
	i2c->CR2 = val;
}

BML_StatusTypeDef I2C_DeviceReady(I2C_TypeDef *i2c, uint8_t saddr,
		uint16_t timeout) {
	uint16_t i = timeout;
	uint8_t flag = 0;
	while (i2c->ISR & I2C_ISR_BUSY) {/* wait until bus not busy */
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_BUSY;
		}
	}
	I2C_CR2(i2c, I2C_CR2_START | (saddr << 1));

	I2C_Delay(1);

	if (!(i2c->ISR & I2C_ISR_NACKF)) {
		flag = 1;
	} else {
		flag = 0;
	}

	i2c->CR2 = 0;
	I2C_DI(i2c);
	I2C_EN(i2c);
	if (flag == LOW) {
		return BML_ERROR;
	}
	return BML_OK;
}

BML_StatusTypeDef I2C_MasterByteWrite(I2C_TypeDef *i2c, uint8_t saddr,
		uint8_t maddr, uint8_t data, uint16_t timeout) {
	uint16_t i = timeout;
	while (i2c->ISR & I2C_ISR_BUSY) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_BUSY;
		}
	} /* wait until bus not busy */

	i = timeout;

	I2C_CR2(i2c, I2C_CR2_START | I2C_CR2_AUTOEND | (2 << 16) | (saddr << 1));
	/* generate start, autoend, byte count 2 and send slave address */

	while (!(i2c->ISR & I2C_ISR_TXIS)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* wait until TXIS is set */
	i2c->TXDR = maddr; /* send register address */

	i = timeout;

	while (!(i2c->ISR & I2C_ISR_TXIS)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* wait until TXIS is set */
	i2c->TXDR = data; /* send data */

	i = timeout;

	while (!(i2c->ISR & I2C_ISR_STOPF)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	}/* wait until stop flag is set */

	i2c->ICR = I2C_ICR_STOPCF; /* clear stop flag */
	I2C_CR2(i2c, I2C_CR2_AUTOEND);

	return BML_OK;
}

BML_StatusTypeDef I2C_MasterByteRead(I2C_TypeDef *i2c, uint8_t saddr,
		uint8_t maddr, uint8_t *data, uint16_t timeout) {
	uint16_t i = timeout;
	while (i2c->ISR & I2C_ISR_BUSY) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_BUSY;
		}
	} /* wait until bus not busy */

	i = timeout;

	I2C_CR2(i2c, I2C_CR2_START | (1 << 16) | (saddr << 1));
	/* generate start and send slave address */
	while (!(i2c->ISR & I2C_ISR_TXIS)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* wait until start flag is set */
	i = timeout;

	i2c->TXDR = maddr; /* send memory address */

	while (!(i2c->ISR & I2C_ISR_TC)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* wait until transfer complete */

	i = timeout;

	/* generate restart, send slave address, read 1 byte, set for auto end */
	I2C_CR2(i2c,
			I2C_CR2_START | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND | (1 << 16)
					| (saddr << 1));

	while (!(i2c->ISR & I2C_ISR_RXNE)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* Wait until RXNE flag is set */
	*data = i2c->RXDR; /* Read data from DR */

	i = timeout;

	while (!(i2c->ISR & I2C_ISR_STOPF)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* wait until stop flag is set */

	i2c->ICR = I2C_ISR_STOPF; /* clear stop flag */
	I2C_CR2(i2c, I2C_CR2_AUTOEND);

	return BML_OK;
}

BML_StatusTypeDef I2C_MasterMultiWrite(I2C_TypeDef *i2c, uint8_t saddr,
		uint8_t maddr, uint32_t n, uint8_t *data, uint16_t timeout) {
	int x;
	uint16_t i = timeout;

	while (i2c->ISR & I2C_ISR_BUSY) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_BUSY;
		}
	} /* wait until bus not busy */
	I2C_CR2(i2c,
	I2C_CR2_START | I2C_CR2_AUTOEND | ((n + 1) << 16) | (saddr << 1));
	/* set byte count and send slave address */
	/* generate start, set auto end, */
	i = timeout;
	while (!(i2c->ISR & I2C_ISR_TXIS)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* wait until TXIS is set */
	i2c->TXDR = maddr; /* send memory address */
	i = timeout;
	for (x = 0; x < n; x++) /* send n bytes of data */
	{
		while (!(i2c->ISR & I2C_ISR_TXIS)) {
			I2C_Delay(1);
			i--;
			if (i <= 0) {
				return BML_TIMEOUT;
			}
		} /* wait until TXIS is set */
		i2c->TXDR = *data++; /* send a byte of data */
	}
	i = timeout;
	while (!(i2c->ISR & I2C_ISR_STOPF)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* wait until stop flag is set */

	i2c->ICR = I2C_ISR_STOPF; /* clear stop flag */
	I2C_CR2(i2c, I2C_CR2_AUTOEND);

	return BML_OK;
}

BML_StatusTypeDef I2C_MasterMultiRead(I2C_TypeDef *i2c, uint8_t saddr,
		uint8_t maddr, uint32_t n, uint8_t *data, uint16_t timeout) {
	uint16_t i = timeout;
	while (i2c->ISR & I2C_ISR_BUSY) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_BUSY;
		}
	} /* wait until bus not busy */
	i = timeout;

	I2C_CR2(i2c, I2C_CR2_START | (1 << 16) | (saddr << 1));/* generate start and send slave address */

	while (!(i2c->ISR & I2C_ISR_TXIS)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	}/* wait until transmitter is ready */

	i = timeout;

	i2c->TXDR = maddr; /* send memory address */
	while (!(i2c->ISR & I2C_ISR_TC)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* wait until transfer complete */

	/* generate restart, send slave address, repeat read n bytes, set for auto end */

	I2C_CR2(i2c,
			I2C_CR2_START | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND
					| ((n & 0xFF) << 16) | (saddr << 1));

	for (; n > 0; n--) {
		while (!(i2c->ISR & I2C_ISR_RXNE)) {
			I2C_Delay(1);
			i--;
			if (i <= 0) {
				return BML_TIMEOUT;
			}
		} /* Wait until RXNE flag is set */
		i = timeout;
		*data++ = i2c->RXDR; /* Read data from DR */
	}

	while (!(i2c->ISR & I2C_ISR_STOPF)) {
		I2C_Delay(1);
		i--;
		if (i <= 0) {
			return BML_TIMEOUT;
		}
	} /* wait until stop flag is set */
	i = timeout;
	i2c->ICR = I2C_ISR_STOPF; /* clear stop flag */
	I2C_CR2(i2c, I2C_CR2_AUTOEND);

	return BML_OK;
}
