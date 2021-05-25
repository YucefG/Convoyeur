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
#include <deplacement.h>
#include <analyse_couleur.h>


//
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

//b_portes est true si on fait attention aux portes, false sinon
bool detection_balise(bool b_balise)
{
	if(b_balise)
	{
		// S'il l'objet passe a travers une porte il met porte a false
		if(get_calibrated_prox(PROX_L)>20)
			return false;
		else
			return true;
	}
	else
		return true;
}

//b_portes est true si on fait attention aux portes, false sinon
bool detection_porte(bool b_portes)
{
	if(b_portes)
	{
		// S'il l'objet passe a travers une porte il met porte a false
		if((get_calibrated_prox(PROX_R)>20) && (get_calibrated_prox(PROX_L)>20))
			return false;
		else
			return true;
	}
	else
		return true;
}

uint8_t detection_couleur(void)
{
	uint8_t numero_couleur = 0;

	init_vitesse_mot();
	init_pos_mot();

}

