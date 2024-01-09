/*
 * SPI.c
 *
 *  Created on: Jan 8, 2024
 *      Author: Hubert
 */


#include "SPI.h"


extern SPI_HandleTypeDef hspi2;


HAL_StatusTypeDef SPIIncSend(char* dataOut,char * dataIn, uint16_t len){

	HAL_StatusTypeDef retVal;

	HAL_GPIO_WritePin(CS_INC_GPIO_Port, CS_INC_Pin, 0);
	HAL_Delay(1);

	retVal = HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)dataOut, (uint8_t*)dataIn, len, 100);

	HAL_GPIO_WritePin(CS_INC_GPIO_Port, CS_INC_Pin, 1);

	return retVal;
}

HAL_StatusTypeDef SPIImuSend(char* dataOut,char * dataIn, uint16_t len){

	HAL_StatusTypeDef retVal;

	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, 0);
	HAL_Delay(1);

	retVal = HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)dataOut, (uint8_t*)dataIn, len, 100);

	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, 1);

	return retVal;
}

HAL_StatusTypeDef SPIPsSend(char* dataOut,char * dataIn, uint16_t len){

	HAL_StatusTypeDef retVal;

	HAL_GPIO_WritePin(CS_PS_GPIO_Port, CS_PS_Pin, 0);
	HAL_Delay(1);

	retVal = HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)dataOut, (uint8_t*)dataIn, len, 100);

	HAL_GPIO_WritePin(CS_PS_GPIO_Port, CS_PS_Pin, 1);

	return retVal;
}


