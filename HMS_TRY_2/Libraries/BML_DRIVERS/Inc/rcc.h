/*
 * rcc.h
 *
 *  Created on: 29-Oct-2022
 *      Author: hp
 */

#ifndef BML_DRIVERS_INC_RCC_H_
#define BML_DRIVERS_INC_RCC_H_

#include "BML_DEF.h"

//=============================| RCC CONFIG FUNCTION |================================//
BML_StatusTypeDef RCC_CONFIG_48MHZ();
BML_StatusTypeDef RCC_CONFIG_48MHZ_BML(uint16_t timeout);
//====================================================================================//

#endif /* BML_DRIVERS_INC_RCC_H_ */
