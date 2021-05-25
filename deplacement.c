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
#include <fcts_maths.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <selector.h>
#include <analyse_couleur.h>


#include <detection.h>
#include <deplacement.h>


//si true: deplacement permis, sinon arret
bool onRoad = EN_CHEMIN;
//va donner la  couleur de l'objet, impose l'acceleration, et la poubelle a viser
uint8_t couleur_objet = 0; 
//donne une memoire temporelle sur la vitesse avant nouvelle imposee
int16_t vitesse_prec =0;
//distance d'acceleration attention a quand le remettre a zero, pas dans les whiles!!!
float dist_acc =0;


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

/*
* 	
*	
*	
*	si zone_bornes = true, on impose vitesse faible, sinon vitesse max
*/
void marche_avant_s(float objectif, bool demarrage_s, bool freinage_s, bool charge, bool zone_bornes, bool b_portes)
{
	chprintf((BaseSequentialStream *)&SD3, "   tics au compteur: %i  ",right_motor_get_pos());
	chprintf((BaseSequentialStream *)&SD3, "   tics objectif : %i  ",CmToSteps(objectif));
	chprintf((BaseSequentialStream *)&SD3, "   sens : %i  ",right_motor_get_pos()<CmToSteps(objectif));



	onRoad = 1 ; //a mettre a 1 avant chaque while de cette fonction
	bool sens = (right_motor_get_pos()<CmToSteps(objectif)) ; // true si avant, false si arriere
	//calcul de l'acceleration
	int16_t v_a_max =0;
	int16_t vitesse_palier =0 ;
	if(zone_bornes)
		vitesse_palier = VITESSE_INTERM;
	else
		vitesse_palier = MAX_VITESSE;
	vitesse_prec = 0;
	//on calcule le nb de tics pour une pente dacc 0->vit max avec acc = max autorise
	float temps_rampe = (float)vitesse_palier/ACCELERATION_MAX; 
	int16_t tics_rampe = (0.5)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe;
	
	//partage en 3 zones de vitesse, zone ascendante, constante, descendante
	int16_t tics1 = tics_rampe;
	int16_t tics3 = tics_rampe;



	//objectif trop court pour atteindre vit max
	//risque de probleme si objectif trop court pour une seule rampe PLUS un des deux bool est fals; probleme de vitesse 
	if((abs(CmToSteps(objectif))<(tics1+tics3))&&(demarrage_s==true)&&(freinage_s==true))
	{
		tics1 = abs(CmToSteps(objectif) / 2);
		tics3 = abs(CmToSteps(objectif) / 2);
	}


	if (demarrage_s==false)
	{
		tics1 = 0;
	}

	if (freinage_s==false)
	{
		tics3 = 0; 
	}

	systime_t time = 0;
	time = chVTGetSystemTime(); 


	while((abs(right_motor_get_pos())<tics1) && onRoad && prox_distance(charge) && detection_porte(b_portes))
	{

		if(sens)
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((4.0)*(0.001));
		}
		else
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((4.0)*(0.001));
		}
		vitesse_prec = v_a_max;
		time = chVTGetSystemTime();
		marche_avant(vitesse_prec);
		chThdSleepMilliseconds(4);	
		chprintf((BaseSequentialStream *)&SD3, "   direction: %i  ",sens);
	}

	while((abs(right_motor_get_pos())+3)<(abs(CmToSteps(objectif))-tics3)
			&& prox_distance(charge) && onRoad && detection_porte(b_portes))
	{
		palClearPad(GPIOD, GPIOD_LED1);
		palClearPad(GPIOD, GPIOD_LED3);
		palClearPad(GPIOD, GPIOD_LED5);
		palClearPad(GPIOD, GPIOD_LED7);		
		if(sens)
		{
			marche_avant(vitesse_palier);
			vitesse_prec = vitesse_palier;
		}
		else
		{
			marche_avant(-vitesse_palier);
			vitesse_prec = -vitesse_palier;
		}	
		chprintf((BaseSequentialStream *)&SD3, "   direction: %i  ",sens);
		chprintf((BaseSequentialStream *)&SD3, "   VITESSE PALIER");



	}
	palSetPad(GPIOD, GPIOD_LED1);
	palSetPad(GPIOD, GPIOD_LED3);
	palSetPad(GPIOD, GPIOD_LED5);
	palSetPad(GPIOD, GPIOD_LED7);

			int compteur =0;


	chprintf((BaseSequentialStream *)&SD3, "   tics 2: : %i  ",right_motor_get_pos());
	while(abs(right_motor_get_pos())<(abs(CmToSteps(objectif))) && prox_distance(charge) && onRoad)
	{
		compteur++;
		if(sens)
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((4.0)*(0.001));
			if(v_a_max<0)
				onRoad = 0;;
		}
		else
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((4.0)*(0.001));
			if(v_a_max>-0)
				onRoad = 0;
		}

		vitesse_prec = v_a_max;
		time = chVTGetSystemTime();
		marche_avant(vitesse_prec);
		chThdSleepMilliseconds(4);
//			chprintf((BaseSequentialStream *)&SD3, "   direction: %i  ",sens);
	}

//	chprintf((BaseSequentialStream *)&SD3, "   compteur : %i  ",compteur);

	chprintf((BaseSequentialStream *)&SD3, "   direction: %i  ",sens);
	chprintf((BaseSequentialStream *)&SD3, "   tics rampe: %i  ",tics_rampe);
	chprintf((BaseSequentialStream *)&SD3, "   tics1: %i  ",tics1);
	chprintf((BaseSequentialStream *)&SD3, "   tics3: %i  ",tics3);
	chprintf((BaseSequentialStream *)&SD3, "   tics au compteur: %i  ",right_motor_get_pos());
	chprintf((BaseSequentialStream *)&SD3, "   tics objectif : %i  ",CmToSteps(objectif));
	
	init_vitesse_mot();
//	chThdSleepMilliseconds(1000);

}

/*
* 	
*	
*	
*	si zone_bornes = true, on impose vitesse faible, sinon vitesse max
*/
void rotation_s(float angle)
{
	onRoad = 1 ; //a mettre a 1 avant chaque while de cette fonction
	float objectif = angle_to_arc(angle,D_ENTRE_ROUES_CM);
	bool sens = (right_motor_get_pos()<CmToSteps(objectif)) ; // true si avant, false si arriere
	//calcul de l'acceleration
	int16_t v_a_max =0;
	int16_t vitesse_palier =0 ;
	vitesse_palier = VITESSE_INTERM;
	vitesse_prec = 0;
	//on calcule le nb de tics pour une pente dacc 0->vit max avec acc = max autorise
	float temps_rampe = (float)vitesse_palier/ACCELERATION_MAX; 
	int16_t tics_rampe = (0.5)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe;
	
	//partage en 3 zones de vitesse, zone ascendante, constante, descendante
	int16_t tics1 = tics_rampe;
	int16_t tics3 = tics_rampe;

	//objectif trop court pour atteindre vit max
	//risque de probleme si objectif trop court pour une seule rampe PLUS un des deux bool est fals; probleme de vitesse 
	if((abs(CmToSteps(objectif))<(tics1+tics3)))
	{
		tics1 = abs(CmToSteps(objectif) / 2);
		tics3 = abs(CmToSteps(objectif) / 2);
	}

//	systime_t time = 0;
//	time = chVTGetSystemTime(); 
	while((abs(right_motor_get_pos())<tics1) && onRoad)
	{
		if(sens)
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((4.0)*(0.001));
		}
		else
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((4.0)*(0.001));
		}
		vitesse_prec = v_a_max;
//		time = chVTGetSystemTime();
		tourner(vitesse_prec);
		chThdSleepMilliseconds(4);	
	}

	while((abs(right_motor_get_pos()))<(abs(CmToSteps(objectif))-tics3) && onRoad)
	{	
		if(sens)
		{
			tourner(vitesse_palier);
			vitesse_prec = vitesse_palier;
		}
		else
		{
			tourner(-vitesse_palier);
			vitesse_prec = -vitesse_palier;
		}	
	}
	int compteur =0;

	while(abs(right_motor_get_pos())<(abs(CmToSteps(objectif))) && onRoad)
	{
		compteur++;
		if(sens)
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((4.0)*(0.001));
			if(v_a_max<0)
				onRoad = 0;;
		}
		else
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((4.0)*(0.001));
			if(v_a_max>-0)
				onRoad = 0;
		}
		vitesse_prec = v_a_max;
//		time = chVTGetSystemTime();
		tourner(vitesse_prec);
		chThdSleepMilliseconds(4);
	}
	init_vitesse_mot();
	chprintf((BaseSequentialStream *)&SD3, "   direction: %i  ",sens);
	chprintf((BaseSequentialStream *)&SD3, "   tics rampe: %i  ",tics_rampe);
	chprintf((BaseSequentialStream *)&SD3, "   tics1: %i  ",tics1);
	chprintf((BaseSequentialStream *)&SD3, "   tics3: %i  ",tics3);
	chprintf((BaseSequentialStream *)&SD3, "   tics au compteur: %i  ",right_motor_get_pos());
	chprintf((BaseSequentialStream *)&SD3, "   tics objectif : %i  ",CmToSteps(objectif));
}


/*
* 	Donne la meme vitesse aux de ux moteurs pour aller tout droit, en avant ou en arriere
*/
 
void marche_avant(int16_t speed)
{
	lumiere_eteinte();			
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
	lumiere_eteinte();		
	palClearPad(GPIOD, GPIOD_LED1);	 
}

void tourner(int16_t speed)
{
	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);
}

void detect_eject(void)
{
	init_pos_mot();
	turn_90(-600);
	chThdSleepMilliseconds(1000);
	if(detec_rouge())
	{
		init_pos_mot();
		turn_90(600);
		init_pos_mot();
		turn_90(600);
		init_pos_mot();
        marche_avant_s(20.0, true, false, true, false, false);
    	init_pos_mot();
        marche_avant_s(-20.0, false, true, true, false, false);
	}
	else
	{
		init_pos_mot();
		turn_90(600);
		init_pos_mot();
		marche_avant_s(-40.0, true, false, true, false, false);
	}
}

