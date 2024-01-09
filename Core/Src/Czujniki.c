/*
 * Czujniki.c
 *
 *  Created on: Jan 8, 2024
 *      Author: Hubert
 */


#include "main.h"
#include "SPI.h"
#include "Czujniki.h"
#include "math.h"

typedef union{
	char buf[4];
	uint32_t dane;
}transfer;


////// INCLINOMETER //////

//Ret 0 if the sensor works
int32_t IncPing(void){
	transfer In,Out;
	Out.buf[0] = 0x0F | (0x80);
	Out.buf[1] = 0x00;
	//6Bh - who I am
	SPIIncSend(Out.buf, In.buf, 2);

	if(In.buf[1] == 0x6B)
			return 0;

	return 1;
}









/////// IMU //////

//ret 0 if sensor works
int32_t ImuPing(void){
	transfer In,Out;
	Out.buf[0] = 0x0F | (0x80);
	Out.buf[1] = 0x00;
	In.dane = 0;
	//6Bh - who I am
	SPIImuSend(Out.buf, In.buf, 2);

	if(In.buf[1] == 0x22)
			return 0;

	return 1;
}


////// PS //////

#define CTRL_REG_DEF		(0x10)
#define CTRL_REG_DEF_OSM	(CTRL_REG_DEF | 1)
#define SA_READ				(0x80)

static float referencePressure = 101325.0f;

void PSSetSeaLevelPressure(float pressure){
	referencePressure = pressure;
	return;
}

__attribute__((weak)) float PSGetSeaLevelPressure(void){

	return referencePressure;
}

__attribute__((weak)) float PSCalculateAlt(float pressure, float temp){


	return ((pow((PSGetSeaLevelPressure() / pressure), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

int32_t PsPing(void){
	transfer In,Out;
	Out.buf[0] = 0x0F | SA_READ;
	Out.buf[1] = 0x00;
	In.dane = 0;
	//6Bh - who I am
	SPIPsSend(Out.buf, In.buf, 2);

	if(In.buf[1] == 0xB1)
			return 0;

	return 1;
}


//No error checking!
int32_t PSRead(PSReading * data){
	HAL_StatusTypeDef retVal;
	transfer In,Out;
	//Force one shoot:
	Out.buf[0] = 0x11;
	Out.buf[1] = CTRL_REG_DEF_OSM;
	retVal = SPIPsSend(Out.buf, In.buf, 2);
	//Wait for one shoot done:
	Out.buf[0] = 0x11 | SA_READ;
	Out.buf[1] = 0;
	while(In.buf[1] != CTRL_REG_DEF){
		retVal = SPIPsSend(Out.buf, In.buf, 2);
		//todo: add timeout!
	}
	//Read values:
	//Pressure:
	Out.dane = 0;
	In.dane = 0;
	Out.buf[0] = 0x28 | SA_READ;
	retVal = SPIPsSend(Out.buf, In.buf, 4);

	//24 bit 2's complement
	In.dane >>=8; //skip nulls
	if(In.buf[3] & 0x80){
		//negative
		In.buf[4]=0xff;
	}

	data->pres = (float)*(int32_t *)(In.buf)/40.96f;


	//Temperature:
	Out.dane = 0;
	In.dane = 0;
	Out.buf[0] = 0x2B | SA_READ;
	retVal = SPIPsSend(Out.buf, In.buf, 3);

	data->temp = (float)*((int16_t *)(In.buf+1));
	data->temp /= 100;

	//calculate Altitude:

	data->alt = PSCalculateAlt(data->pres,data->temp);

	return retVal;
}

