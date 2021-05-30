#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <arm_math.h>
#include <motors.h>
#include <chprintf.h>
#include <stdbool.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <detection.h>
#include <deplacement.h>
#include <analyse_couleur.h>
#include <lumiere.h>



/* 
 *	Si les capteurs de proximite situes a un angle de 17° ont atteint la limite de collision
 * 	alors la fonction return false pour arreter le robot
 */
_Bool prox_distance(bool charge)
{
	if (charge==PAS_CHARGE)
	{
		if((get_calibrated_prox(PROX_FRONT_R17)>LIMITE_COLLISION) ||
	 	 (get_calibrated_prox(PROX_FRONT_L17)>LIMITE_COLLISION))
			return OBSTACLE_FRONT_DETECTE;
		else
			return RIEN_FRONT_DETECTE;
	}
	else
		return RIEN_FRONT_DETECTE;
}

/* 
 *	Si les capteurs de proximite situes a un angle de 90° ont detecte des portes
 * 	alors la fonction return false
 */
bool detection_balise(bool b_balise)
{
	if(b_balise)
	{
		if(get_calibrated_prox(PROX_L)>SEUIL_BALISE)
			return BALISE_DETECTE;
		else
			return BALISE_NON_DETECTE;
	}
	else
		return BALISE_NON_DETECTE;
}

/* 
 *	b_portes est true si on fait attention aux portes, false sinon
 * 	si b_portes: true, alors si les deux capteurs a 90 deg sont actifs au meme temps, 
 *	on return true, false sinon.  
 */
bool detection_porte(bool b_portes)
{
	if(b_portes)
	{
		if((get_calibrated_prox(PROX_R)>SEUIL_PORTE) && (get_calibrated_prox(PROX_L)>SEUIL_PORTE))
			return PORTE_DETECTE;
		else
			return PORTE_NON_DETECTE;
	}
	else
		return PORTE_DETECTE;
}
