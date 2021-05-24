#include <fcts_maths.h>
#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <chprintf.h>
#include <stdbool.h>
#include <stdint.h>
#include <deplacement.h>
#include <detection.h>


float StepsToCm(int32_t nbSteps)
{
	return (float)((float)(nbSteps*PERIM_ROUE_CM)/(float)(TICS_1_TOUR));
}

float MmToCm(uint16_t ValeurMm)
{
	return (float)((float)ValeurMm)/((float)DIX);
}

int16_t CmToSteps(float ValeurCm)
{
	return (int16_t)((int16_t)ValeurCm*TICS_1_TOUR)/((int16_t)PERIM_ROUE_CM);
}

int16_t Distance_to_temps(float objectif, float acceleration, float delta_t)
{
	float temps =0;
	temps = sqrtf((2.0*(float)(CmToSteps(objectif)))/acceleration);
    chprintf((BaseSequentialStream *)&SD3, "  %f  ", temps);

	return (uint16_t)(temps/delta_t);
} 

