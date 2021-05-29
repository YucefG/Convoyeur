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
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <selector.h>

#include <analyse_couleur.h>
#include <detection.h>
#include <deplacement.h>
#include <fcts_maths.h>
#include <lumiere.h>


//si true: deplacement permis, sinon arret
bool onRoad = EN_ROUTE;
//va donner la  couleur de l'objet, impose l'acceleration, et la poubelle a viser
uint8_t couleur_objet = 0;
//donne une memoire temporelle sur la vitesse avant nouvelle imposee
int16_t vitesse_prec =0;
//distance d'acceleration attention a quand le remettre a zero, pas dans les whiles!!!
float dist_acc =0;
static uint8_t compte_d = 0;
static uint8_t compte_g = 0;


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
void marche_avant_s(float objectif, bool demarrage_s, bool freinage_s, bool charge, bool zone_bornes, bool b_balise)
{
	onRoad = EN_ROUTE ; //a mettre a 1 avant chaque while de cette fonction
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
	int16_t tics_rampe = (UN_DEMI)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe;
	
	//partage en 3 zones de vitesse, zone ascendante, constante, descendante
	int16_t tics1 = tics_rampe;
	int16_t tics3 = tics_rampe;
		//objectif trop court pour atteindre vit max
	//risque de probleme si objectif trop court pour une seule rampe PLUS un des deux bool est fals; probleme de vitesse 
	if((abs(CmToSteps(objectif))<(tics1+tics3))&&(demarrage_s==DEMAR_DOUX)&&(freinage_s==FREIN_DOUX))
	{
		tics1 = abs(CmToSteps(objectif) / VAL2);
		tics3 = abs(CmToSteps(objectif) / VAL2);
	}

	if (demarrage_s==DEMAR_CHOC)
	{
		tics1 = 0;
	}
	if (freinage_s==FREIN_CHOC)
	{
		tics3 = 0; 
	}

	while((abs(right_motor_get_pos())<tics1) && onRoad && prox_distance(charge) && detection_balise(b_balise))
	{

		if(sens)
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
		}
		else
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
		}
		vitesse_prec = v_a_max;
		marche_avant(vitesse_prec);
		chThdSleepMilliseconds(DELTA_T_MS);	
	}

	while((abs(right_motor_get_pos())+OFFSET)<(abs(CmToSteps(objectif))-tics3)
			&& prox_distance(charge) && onRoad && detection_balise(b_balise))
	{	
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
	}

	while(abs(right_motor_get_pos())<(abs(CmToSteps(objectif))) && prox_distance(charge) && onRoad)
	{
		if(sens)
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
			if(v_a_max<0)
				onRoad = ARRET;
		}
		else
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
			if(v_a_max>-0)
				onRoad = ARRET;
		}

		vitesse_prec = v_a_max;
		marche_avant(vitesse_prec);
		chThdSleepMilliseconds(DELTA_T_MS);
		}
		init_vitesse_mot();
}

/*
* 	
*	
*	
*	si zone_bornes = true, on impose vitesse faible, sinon vitesse max
*/
void rotation_s(float angle)
{
	onRoad = EN_ROUTE ; //a mettre a 1 avant chaque while de cette fonction
	float objectif = angle_to_arc(angle,D_ENTRE_ROUES_CM);
	bool sens = (right_motor_get_pos()<CmToSteps(objectif)) ; // true si avant, false si arriere
	//calcul de l'acceleration
	int16_t v_a_max =0;
	int16_t vitesse_palier =0 ;
	vitesse_palier = VITESSE_INTERM;
	vitesse_prec = 0;
	//on calcule le nb de tics pour une pente dacc 0->vit max avec acc = max autorise
	float temps_rampe = (float)vitesse_palier/ACCELERATION_MAX; 
	int16_t tics_rampe = (UN_DEMI)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe;
	
	//partage en 3 zones de vitesse, zone ascendante, constante, descendante
	int16_t tics1 = tics_rampe;
	int16_t tics3 = tics_rampe;

	//objectif trop court pour atteindre vit max
	//risque de probleme si objectif trop court pour une seule rampe PLUS un des deux bool est fals; probleme de vitesse 
	if((abs(CmToSteps(objectif))<(tics1+tics3)))
	{
		tics1 = abs(CmToSteps(objectif) / VAL2);
		tics3 = abs(CmToSteps(objectif) / VAL2);
	}

	while((abs(right_motor_get_pos())<tics1) && onRoad)
	{
		if(sens)
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
		}
		else
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
		}
		vitesse_prec = v_a_max;
	//		time = chVTGetSystemTime();
		tourner(vitesse_prec);
		chThdSleepMilliseconds(DELTA_T_MS);	
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

	while(abs(right_motor_get_pos())<(abs(CmToSteps(objectif))) && onRoad)
	{
		if(sens)
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
			if(v_a_max<0)
				onRoad = ARRET;
		}
		else
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
			if(v_a_max>-0)
				onRoad = ARRET;
		}
		vitesse_prec = v_a_max;
	//		time = chVTGetSystemTime();
		tourner(vitesse_prec);
		chThdSleepMilliseconds(DELTA_T_MS);
	}
	init_vitesse_mot();
	chThdSleepMilliseconds(PAUSE200);
}	



/*
* 	Donne la meme vitesse aux de ux moteurs pour aller tout droit, en avant ou en arriere
*/
 
void marche_avant(int16_t speed)
{
	left_motor_set_speed(speed);
	right_motor_set_speed(speed);
	leds_OFF();		
	palClearPad(GPIOD, GPIOD_LED1);	 
}

void tourner(int16_t speed)
{
	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);
}

void eject_colis(bool b_eject)
{
	init_pos_mot();
	rotation_s(-ANGLE90);
	chThdSleepMilliseconds(PAUSE100);
	if (b_eject)
	{
	/*	init_pos_mot();
		rotation_s(180.0); */
		init_pos_mot();
		rotation_s(ANGLE90);
		init_pos_mot();
		rotation_s(ANGLE90);
		init_pos_mot();
		
        marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_CHOC, PAS_CHARGE, AUTOROUTE, PORTE_FALSE);
        //permet de revenir d'exactement la bonne distance
    	int16_t memoire = right_motor_get_pos();
    	float memoire_cm = StepsToCm(memoire);
        init_pos_mot();

        init_vitesse_mot();
        chThdSleepMilliseconds(PAUSE100);
        playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
        marche_avant_s(-memoire_cm, DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);
        init_pos_mot();

        //2e coup
        
		marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_CHOC, PAS_CHARGE, AUTOROUTE, PORTE_FALSE);

        //permet de revenir d'exactement la bonne distance
    	memoire = right_motor_get_pos();
    	memoire_cm = StepsToCm(memoire);
        init_pos_mot();

        init_vitesse_mot();
        chThdSleepMilliseconds(PAUSE100);
        marche_avant_s(-memoire_cm, DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);
        init_pos_mot();

        marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_CHOC, PAS_CHARGE, AUTOROUTE, PORTE_FALSE);

        //permet de revenir d'exactement la bonne distance
    	memoire = right_motor_get_pos();
    	memoire_cm = StepsToCm(memoire);
        init_pos_mot();
        init_vitesse_mot();
        chThdSleepMilliseconds(PAUSE100);
        marche_avant_s(-memoire_cm, DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);
        init_pos_mot();


		rotation_s(-ANGLE90);
	}
	else
	{
		init_pos_mot();
		rotation_s(ANGLE90);
		init_pos_mot();
	}
}

bool recup_colis(bool b_recup)
{
	bool colis_recupere = COLIS_NON_RECUP; 
	//recupere le colis
	if(b_recup)
	{
		init_pos_mot();
		rotation_s(ANGLE180);
		init_pos_mot();
		marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_CHOC, PAS_CHARGE, ZONE_BORNES, PORTE_FALSE);
        //permet de revenir d'exactement la bonne distance
    	int16_t memoire = right_motor_get_pos();
    	float memoire_cm = StepsToCm(memoire);
    	if(memoire_cm < SEUIL)
    	{
    		colis_recupere = COLIS_NON_RECUP;
    	}
    	else
    	{
    		colis_recupere = COLIS_RECUP;

    		//intialise les moteurs avant de s'axer devant l'objet
			init_vitesse_mot();
			init_pos_mot();
			axage();

			//initialise les moteurs avant d'avancer de 2cm
			init_vitesse_mot();
			init_pos_mot();
			marche_avant_s(POUSSEE_3CM,  DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);

			//initialise les moteurs avant de reculer de 2cm
			init_vitesse_mot();
			init_pos_mot();
			chThdSleepMilliseconds(PAUSE400);
			marche_avant_s(-POUSSEE_3CM,  DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);

			//initialise les moteurs avant de se desaxer
			init_vitesse_mot();
			init_pos_mot();
			chThdSleepMilliseconds(PAUSE400);
			re_axage();
			init_vitesse_mot();
			init_pos_mot();
		}
		//initialise les moteurs avant de faire marche arriere
		init_vitesse_mot();
		init_pos_mot();
		chThdSleepMilliseconds(PAUSE400);
		marche_avant_s(-memoire_cm, DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);

		 //initialise les moteurs avant de partir en direction de la poubelle
		init_vitesse_mot();
		init_pos_mot();
		chThdSleepMilliseconds(PAUSE400);
		rotation_s(-ANGLE90);
	}
	else
	{
		//ne recupere pas le colis
		init_vitesse_mot();
		init_pos_mot();
		chThdSleepMilliseconds(PAUSE400);
		rotation_s(ANGLE90);
		colis_recupere = COLIS_NON_RECUP; 
	}
	return colis_recupere;
}


/* Cette fonction permet au robot de bien se centrer
 * afin de recuperer l'objet de maniere parfaite sur sa tete
 * */
void axage(void)
{
	uint8_t axage = 0;
	axage = abs(get_calibrated_prox(PROX_FRONT_R17) - get_calibrated_prox(PROX_FRONT_L17));
	chThdSleepMilliseconds(PAUSE1000);
	while((get_calibrated_prox(PROX_FRONT_R17) > get_calibrated_prox(PROX_FRONT_L17)) &&
				axage > SEUIL_AXAGE)
		{
			// recentrer
			chThdSleepMilliseconds(PAUSE100);
			left_motor_set_speed(VIT100);
			right_motor_set_speed(-VIT100);
			compte_d++;
		}

		while((get_calibrated_prox(PROX_FRONT_R17) < get_calibrated_prox(PROX_FRONT_L17)) &&
				axage > SEUIL_AXAGE)
		{
			// recentrer
			chThdSleepMilliseconds(PAUSE100);
			left_motor_set_speed(-VIT100);
			right_motor_set_speed(VIT100);
			compte_g ++;
		}
}

void re_axage(void)
{
	while(compte_d != 0)
	{
	    chThdSleepMilliseconds(PAUSE100);
		left_motor_set_speed(-VIT100);
		right_motor_set_speed(VIT100);
		compte_d --;
	}
	while(compte_g != 0)
	{
		chThdSleepMilliseconds(PAUSE100);
		left_motor_set_speed(VIT100);
		right_motor_set_speed(-VIT100);
		compte_g --;
	}
}

void next_balise(void)
{
	init_pos_mot();
	init_vitesse_mot();
	marche_avant_s(SORTIE_BALISE,DEMAR_DOUX,FREIN_CHOC,CHARGE,ZONE_BORNES,PORTE_FALSE);
	init_pos_mot();
	marche_avant_s(DIST_MAX,DEMAR_CHOC,FREIN_DOUX,CHARGE,ZONE_BORNES,PORTE_TRUE);
	init_vitesse_mot();
}

void next_porte(int16_t speed)
{
	init_pos_mot();
	init_vitesse_mot();
	while(detection_porte(PORTE_TRUE)==PORTE_FALSE)
		marche_avant(speed);
	init_vitesse_mot();
}






//a modifier et a nettoyer
void retour_base(void)
{
	init_pos_mot();
	init_vitesse_mot();
	marche_avant_s(10.0, DEMAR_DOUX,FREIN_DOUX,CHARGE,AUTOROUTE,PORTE_FALSE);
	init_pos_mot();
	init_vitesse_mot();
	marche_avant_s(-10.0, DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE,PORTE_FALSE);
	init_pos_mot();
	init_vitesse_mot();
	/*	init_pos_mot();
		rotation_s(180.0); */
	init_pos_mot();
	rotation_s(ANGLE180);
	init_pos_mot();
	init_vitesse_mot();
	//marche_avant_s(StepsToCm(tics_retour),true,true,false,false,false);
	marche_avant_s(DIST_MAX, DEMAR_DOUX,FREIN_DOUX,PAS_CHARGE,AUTOROUTE,PORTE_FALSE);
	init_pos_mot();
}

/*
*		si bool_a_route = truem on accelere de vit_min a vit_max
*		fonction utilisable que en marche avant
*/
void balise_to_route(bool balise_a_route)
{
	int16_t v_a_max = 0;
	float temps_rampe = (float)(MAX_VITESSE-VITESSE_INTERM)/ACCELERATION_MAX; 
	int16_t tics_rampe = (UN_DEMI)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe+(float)VITESSE_INTERM*temps_rampe;
	if(balise_a_route)
		vitesse_prec = VITESSE_INTERM;
	else
		vitesse_prec = MAX_VITESSE;
	init_pos_mot();
	while(abs(right_motor_get_pos())<tics_rampe)
	{
		if(balise_a_route)
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
		}
		else
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((DELTA_T_MS_F)*(MS_TO_S));
		}
		vitesse_prec = v_a_max;
		marche_avant(vitesse_prec);
		chThdSleepMilliseconds(DELTA_T_MS);	
	}
}

void calibration_angle(void)
{
	leds_OFF();
	int16_t prox_prec_R = get_calibrated_prox(PROX_R);
	int16_t prox_prec_L = get_calibrated_prox(PROX_L);
	bool arret = false;
	while(arret == false)
	{
		tourner(200);
		chThdSleepMilliseconds(30);
		if((get_calibrated_prox(PROX_R)>500 && (prox_prec_R-get_calibrated_prox(PROX_R))>5)
		 && (get_calibrated_prox(PROX_L)>500  && (prox_prec_L-get_calibrated_prox(PROX_L))>5)) 
			arret = true; 
		prox_prec_R = get_calibrated_prox(PROX_R);
		prox_prec_L = get_calibrated_prox(PROX_L);

		
	} 
	init_vitesse_mot();

	leds_ON();
	chThdSleepMilliseconds(1000);
	leds_OFF();
	leds_ON();
	chThdSleepMilliseconds(1000);
	leds_OFF();

	
}
