#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mesure.h>
#include <math.h>
#include <arm_math.h>
#include <motors.h>
#include <chprintf.h>
#include <lumiere.h>
#include <stdbool.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <deplacement.h>


_Bool prox_distance(void)
{
	/* Si les capteurs de proximite situes a un angle de 17� ont atteint la limite de collision
	 * alors la fonction return true pour arreter le robot*/
	if((get_calibrated_prox(PROX_FRONT_R17)>LIMITE_COLLISION) ||
	  (get_calibrated_prox(PROX_FRONT_L17)>LIMITE_COLLISION))
		return false;
	else
		return true;
}

uint8_t get_compteur(void){
	return compteur;
}

//decremente le compteur
void dec_compteur(void)
{
	compteur--; 
}



