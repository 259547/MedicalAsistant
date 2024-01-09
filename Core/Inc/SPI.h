/*
 * SPI.h
 *
 *  Created on: Jan 8, 2024
 *      Author: Hubert
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "main.h"

HAL_StatusTypeDef SPIIncSend(char* dataOut,char * dataIn, uint16_t len);
HAL_StatusTypeDef SPIImuSend(char* dataOut,char * dataIn, uint16_t len);
HAL_StatusTypeDef SPIPsSend(char* dataOut,char * dataIn, uint16_t len);

//Prototypu:

int32_t IncPing(void);

//IMU:

int32_t ImuPing(void);

#endif /* INC_SPI_H_ */
