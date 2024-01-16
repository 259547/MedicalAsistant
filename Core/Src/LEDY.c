/*
 * LEDY.c
 *
 *  Created on: Jan 13, 2024
 *      Author: Hubert
 */


#include "LEDY.h"

extern I2C_HandleTypeDef hi2c3;


typedef struct{
	uint8_t lower;
	uint8_t higher;
}LEDS_REG;

LEDS_REG rejestryLedow[4] = {{0x55,0x55},{0x55,0x55},{0x55,0x55},{0x55,0x55}}; //default values
uint8_t LEDS_ADDRESSES[4] = {0x60,0x61,0x62,0x64};


static void LED_ENABLE(uint8_t device_no, uint8_t led_no){
	if(device_no > 3)
		return;
	if(led_no > 6) //7 th is disabled by common reset
			return;

	if(led_no > 3)
		rejestryLedow[device_no].higher |= (1<<((led_no & 0x3)*2));
	else
		rejestryLedow[device_no].lower |= (1<<((led_no & 0x3)*2));

	return;
}

static void LED_DISABLE(uint8_t device_no, uint8_t led_no){
	if(device_no > 3)
		return;
	if(led_no > 6) //7 th is disabled by common reset
			return;

	if(led_no > 3)
		rejestryLedow[device_no].higher &= ~(1<<((led_no & 0x3)*2));
	else
		rejestryLedow[device_no].lower &= ~(1<<((led_no & 0x3)*2));

	return;
}

void LEDChangeState(uint8_t device_no, uint8_t led_no, uint8_t state){
	if(device_no > 3)
			return;
	if(led_no > 6) //7 th is disabled by common reset
			return;

	if(state)
		LED_ENABLE(device_no, led_no);
	else
		LED_DISABLE(device_no, led_no);

	//send new config:

	if(led_no > 3){
		//address 6
		HAL_I2C_Mem_Write(&hi2c3,
							(LEDS_ADDRESSES[device_no]<<1),
							6,
							1,
							&(rejestryLedow[device_no].higher),
							1,
							100);
	}
	else{
		//address 5, lower LEDS
		HAL_I2C_Mem_Write(&hi2c3,
					(LEDS_ADDRESSES[device_no]<<1),
					5,
					1,
					&(rejestryLedow[device_no].lower),
					1,
					100);
	}


	return;
}
