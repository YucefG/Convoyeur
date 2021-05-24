#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <arm_math.h>
#include <motors.h>
#include <chprintf.h>
#include <lumiere.h>
#include <stdbool.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <detection.h>
#include <analyse_couleur.h>

#define UN			1
#define DEUX		2

_Bool prox_distance(bool charge)
{
	/* Si les capteurs de proximite situes a un angle de 17° ont atteint la limite de collision
	 * alors la fonction return true pour arreter le robot*/
	if (charge==false)
	{
		if((get_calibrated_prox(PROX_FRONT_R17)>LIMITE_COLLISION) ||
	 	 (get_calibrated_prox(PROX_FRONT_L17)>LIMITE_COLLISION))
			return false;
		else
			return true;
	}
	else
		return true;
}

bool detection_porte(void)
{
	bool porte = true;

	// S'il l'objet passe a travers une porte il met porte a false
	if((get_calibrated_prox(PROX_FRONT_R49)!=0) && (get_calibrated_prox(PROX_FRONT_L49)!=0))
		porte = false;
	else
		porte = true;

	return porte;
}



