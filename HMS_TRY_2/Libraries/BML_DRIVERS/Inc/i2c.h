/*
 * i2c.h
 *
 *  Created on: Oct 23, 2022
 *      Author: hp
 */

#ifndef BML_DRIVERS_INC_I2C_H_
#define BML_DRIVERS_INC_I2C_H_

#include "BML_DEF.h"

#define STANDARD_MODE		(uint32_t)0x2000090E
#define FAST_MODE_400KHZ	(uint32_t)0x0000020B
#define	FAST_MODE_300KHZ	(uint32_t)0x00000A17
#define	FAST_MODE_200KHZ	(uint32_t)0x00000212

typedef struct{
	I2C_TypeDef *i2c;
	uint32_t	TIMING;
}I2C_TYPE;

void I2C_EN(I2C_TypeDef *i2c);
void I2C_DI(I2C_TypeDef *i2c);
void I2C_INIT(I2C_TypeDef *i2c, uint32_t timing);
void I2C_DEINIT(I2C_TypeDef *i2c);
void I2C_CR2(I2C_TypeDef *i2c, uint32_t val);
BML_StatusTypeDef I2C_DeviceReady(I2C_TypeDef *i2c, uint8_t saddr, uint16_t timeout);
BML_StatusTypeDef I2C_MasterByteWrite(I2C_TypeDef *i2c, uint8_t saddr, uint8_t maddr, uint8_t data, uint16_t timeout);
BML_StatusTypeDef I2C_MasterByteRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t maddr, uint8_t *data, uint16_t timeout);
BML_StatusTypeDef I2C_MasterMultiWrite(I2C_TypeDef *i2c, uint8_t saddr, uint8_t maddr, uint32_t n,
		uint8_t *data, uint16_t timeout);
BML_StatusTypeDef I2C_MasterMultiRead(I2C_TypeDef *i2c, uint8_t saddr, uint8_t maddr, uint32_t n,
		uint8_t *data, uint16_t timeout);

#endif /* BML_DRIVERS_INC_I2C_H_ */
