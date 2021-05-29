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
#include <arm_math.h>

#include <fcts_maths.h>
#include <deplacement.h>
#include <detection.h>



float StepsToCm(int32_t nbSteps)
{
	return (float)((float)(nbSteps*PERIM_ROUE_CM)/(float)(TICS_1_TOUR));
}

int32_t CmToSteps(float ValeurCm)
{
	return (int32_t)((ValeurCm*(float)TICS_1_TOUR)/PERIM_ROUE_CM);
}

float angle_to_arc(float angle, float diametre)
{
	return angle*(PI/ANGLE360)*diametre; 
}
