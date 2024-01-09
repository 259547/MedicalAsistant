/*
 * Czujniki.h
 *
 *  Created on: Jan 9, 2024
 *      Author: Hubert
 */

#ifndef INC_CZUJNIKI_H_
#define INC_CZUJNIKI_H_

#include "main.h"


////// PS //////

typedef struct{

	float temp;
	float pres;
	float alt;
}PSReading;

int32_t PSRead(PSReading * data);
void PSSetSeaLevelPressure(float pressure);
float PSGetSeaLevelPressure(void);
float PSCalculateAlt(float pressure, float temp);

#endif /* INC_CZUJNIKI_H_ */
