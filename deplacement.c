#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <chprintf.h>
#include <stdbool.h>
#include <arm_math.h>
#include <fonctions_maths.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <selector.h>

#include <deplacement.h>

			

int16_t pi_regulator(float distance, float goal)
{
	float error = 0;
	float speed = 0;
	static float sum_error = 0;
	error = goal - distance;
	//disables the PI regulator if the error is to small
	if(fabs(error) < ERROR_THRESHOLD)
	{
		onRoad = ARRET ;
		return VITESSE_NULLE;
	}
	else
		onRoad = EN_CHEMIN;

	sum_error += error;
	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR)
	{
		sum_error = MAX_SUM_ERROR;
	}
	else if(sum_error < -MAX_SUM_ERROR)
	{
		sum_error = -MAX_SUM_ERROR;
	}
	
	speed = KP * error + KI * sum_error;

	//Mettre le booleen onRoad a 0 car on arrive a destination
	if(abs(speed)<SEUIL_VIT_NUL_PI)
	{
		onRoad = ARRET ;
		return VITESSE_NULLE;
	}

	//Ajustement de la vitesse max positive ou négative car problémes au max du hardware
	if(speed>MAX_VITESSE_PI)
		speed = MAX_VITESSE_PI;

	if(speed<-MAX_VITESSE_PI)
		speed = -MAX_VITESSE_PI;
    
    return (int16_t)speed;
}

/*
*	Effectue une rotation de 90 degres a une vitesse donnee. 
*/
void turn_90(int16_t speed)
{
	static int16_t tics90 = TICS_90;
	while((left_motor_get_pos()<tics90)&&(right_motor_get_pos()<tics90))
	{
		left_motor_set_speed(speed);
		right_motor_set_speed(-speed); 
	}
	init_vitesse_mot();
}

//initialise les compteurs de moteur.
void init_pos_mot(void)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
//met les vitesses de moteur a zero. 
void init_vitesse_mot(void)
{
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}