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

#define ZERO					0
#define	SEUIL_AXAGE				20
#define CENT					100
#define MILLE					1000

//si true: deplacement permis, sinon arret
bool onRoad = EN_CHEMIN;
//va donner la  couleur de l'objet, impose l'acceleration, et la poubelle a viser
uint8_t couleur_objet = ZERO;
//donne une memoire temporelle sur la vitesse avant nouvelle imposee
int16_t vitesse_prec =0;
//distance d'acceleration attention a quand le remettre a zero, pas dans les whiles!!!
float dist_acc =ZERO;
static uint8_t compte_d = ZERO;
static uint8_t compte_g = ZERO;


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
	//	chprintf((BaseSequentialStream *)&SD3, "   DANS MARCHE AVANT  ");


	//	chprintf((BaseSequentialStream *)&SD3, "   tics au compteur: %i  ",right_motor_get_pos());
	//	chprintf((BaseSequentialStream *)&SD3, "   tics objectif : %i  ",CmToSteps(objectif));
	//	chprintf((BaseSequentialStream *)&SD3, "   sens : %i  ",right_motor_get_pos()<CmToSteps(objectif));

		//chThdSleepMilliseconds(5000);

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

	//	systime_t time = 0;
	//	time = chVTGetSystemTime();

	//	chprintf((BaseSequentialStream *)&SD3, "   prox_distance(charge): %i  ",prox_distance(charge));
	//	chprintf((BaseSequentialStream *)&SD3, "   detection_balise(b_balise): %i  ",detection_balise(b_balise));
	//	chprintf((BaseSequentialStream *)&SD3, "   tics1: %i  ",tics1);

	//chThdSleepMilliseconds(5000);


	while((abs(right_motor_get_pos())<tics1) && onRoad && prox_distance(charge) && detection_balise(b_balise))
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
	//	time = chVTGetSystemTime();
		marche_avant(vitesse_prec);
		chThdSleepMilliseconds(4);	
	}

	while((abs(right_motor_get_pos())+3)<(abs(CmToSteps(objectif))-tics3)
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
	//		chprintf((BaseSequentialStream *)&SD3, "   direction: %i  ",sens);



	}
	palSetPad(GPIOD, GPIOD_LED1);
	palSetPad(GPIOD, GPIOD_LED3);
	palSetPad(GPIOD, GPIOD_LED5);
	palSetPad(GPIOD, GPIOD_LED7);

			int compteur =0;


	//	chprintf((BaseSequentialStream *)&SD3, "   tics 2: : %i  ",right_motor_get_pos());
	while(abs(right_motor_get_pos())<(abs(CmToSteps(objectif))) && prox_distance(charge) && onRoad)
	{
		compteur++;
		if(sens)
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((4.0)*(0.001));
			if(v_a_max<0)
				onRoad = 0;
		}
		else
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((4.0)*(0.001));
			if(v_a_max>-0)
				onRoad = 0;
		}

		vitesse_prec = v_a_max;
	//	time = chVTGetSystemTime();
		marche_avant(vitesse_prec);
		chThdSleepMilliseconds(4);
	//			chprintf((BaseSequentialStream *)&SD3, "   direction: %i  ",sens);
		}

	//	chprintf((BaseSequentialStream *)&SD3, "   compteur : %i  ",compteur);

		/*chprintf((BaseSequentialStream *)&SD3, "   direction: %i  ",sens);
		chprintf((BaseSequentialStream *)&SD3, "   tics rampe: %i  ",tics_rampe);
		chprintf((BaseSequentialStream *)&SD3, "   tics1: %i  ",tics1);
		chprintf((BaseSequentialStream *)&SD3, "   tics3: %i  ",tics3);
		chprintf((BaseSequentialStream *)&SD3, "   tics au compteur: %i  ",right_motor_get_pos());
		chprintf((BaseSequentialStream *)&SD3, "   tics objectif : %i  ",CmToSteps(objectif));*/
		init_vitesse_mot();
			//chprintf((BaseSequentialStream *)&SD3, "  FINNNNNN  ");

		//chThdSleepMilliseconds(10000);
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
	chThdSleepMilliseconds(200);

	//chprintf((BaseSequentialStream *)&SD3, "   direction: %i  ",sens);
	//chprintf((BaseSequentialStream *)&SD3, "   tics rampe: %i  ",tics_rampe);
	//chprintf((BaseSequentialStream *)&SD3, "   tics1: %i  ",tics1);
	//chprintf((BaseSequentialStream *)&SD3, "   tics3: %i  ",tics3);
	//chprintf((BaseSequentialStream *)&SD3, "   tics au compteur: %i  ",right_motor_get_pos());
	//chprintf((BaseSequentialStream *)&SD3, "   tics objectif : %i  ",CmToSteps(objectif));
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

void eject_colis(bool b_eject)
{
	init_pos_mot();
	rotation_s(-90.0);
	chThdSleepMilliseconds(100);
	if (b_eject)
	{
	/*	init_pos_mot();
		rotation_s(180.0); */
		init_pos_mot();
		rotation_s(90.0);
		init_pos_mot();
		rotation_s(90.0);
		init_pos_mot();
		
        marche_avant_s(60.0, true, false, false, false, false);
        //permet de revenir d'exactement la bonne distance
    	int16_t memoire = right_motor_get_pos();
    	float memoire_cm = StepsToCm(memoire);
        init_pos_mot();

        init_vitesse_mot();
        chThdSleepMilliseconds(100);
        playMelody(MARIO_FLAG, ML_FORCE_CHANGE, NULL);
        marche_avant_s(-memoire_cm, true, true, true, false, false);
        init_pos_mot();

        //2e coup
        
		marche_avant_s(60.0, true, false, false, false, false);

        //permet de revenir d'exactement la bonne distance
    	memoire = right_motor_get_pos();
    	memoire_cm = StepsToCm(memoire);
        init_pos_mot();

        init_vitesse_mot();
        chThdSleepMilliseconds(100);
        marche_avant_s(-memoire_cm, true, true, true, false, false);
        init_pos_mot();

        marche_avant_s(60.0, true, false, false, false, false);

        //permet de revenir d'exactement la bonne distance
    	memoire = right_motor_get_pos();
    	memoire_cm = StepsToCm(memoire);
        init_pos_mot();
        init_vitesse_mot();
        chThdSleepMilliseconds(100);
        marche_avant_s(-memoire_cm, true, true, true, false, false);
        init_pos_mot();


		rotation_s(-90.0);
	}
	else
	{
		init_pos_mot();
		rotation_s(90.0);
		init_pos_mot();
	}
}

void detect_eject(uint8_t compteur)
{
	init_pos_mot();
	rotation_s(-90.0);
	chThdSleepMilliseconds(100);
	if(detec_rouge() && compteur == 1) //&& detection_objet_recup()==0)
	{
		chprintf((BaseSequentialStream *)&SD3, "   j'ai detect?la poubelle rouge, donc compteur = : %u  ",compteur);
		init_pos_mot();
		rotation_s(180.0);
		init_pos_mot();
        marche_avant_s(40.0, true, false, false, false, false);
        //permet de revenir d'exactement la bonne distance
    	int16_t memoire = right_motor_get_pos();
    	float memoire_cm = StepsToCm(memoire);
    	init_pos_mot();
        marche_avant_s(-memoire_cm, false, true, true, false, false);
        compteur = 2;
        init_pos_mot();
		rotation_s(-90.0);

	}
	else if(detec_rouge()== false && compteur == 2)
	{
		chprintf((BaseSequentialStream *)&SD3, "   j'ai detect?la poubelle bleue, donc compteur = : %u  ",compteur);
		init_pos_mot();
		rotation_s(180.0);
		init_pos_mot();
        marche_avant_s(40.0, true, false, false, false, false);
		//permet de revenir d'exactement la bonne distance
		int16_t memoire = right_motor_get_pos();
    	float memoire_cm = StepsToCm(memoire);
		init_pos_mot();
        marche_avant_s(-memoire_cm, false, true, true, false, false);
        compteur = 0;
        init_pos_mot();
		rotation_s(-90.0);
	}
	else
	{
		init_pos_mot();
		rotation_s(90.0);
		init_pos_mot();
	}
}

bool recup_colis(bool b_recup)
{
	bool colis_recupere = false; 
	//oriente la caméra sur l'objet de gauche
	init_vitesse_mot();
	init_pos_mot();
	chThdSleepMilliseconds(400);
	rotation_s(-90.0);
	//recupere le colis
	if(b_recup)
	{
		/*	init_pos_mot();
		rotation_s(180.0); */
		init_pos_mot();
		rotation_s(90.0);
		init_pos_mot();
		rotation_s(90.0);
		init_pos_mot();
		marche_avant_s(40.0, true, false, false, true, false);
        //permet de revenir d'exactement la bonne distance
    	int16_t memoire = right_motor_get_pos();
    	float memoire_cm = StepsToCm(memoire);
    	if(memoire_cm < SEUIL)
    	{
    		colis_recupere = false;
    	}
    	else
    	{
    		colis_recupere = true;

    		//intialise les moteurs avant de s'axer devant l'objet
			init_vitesse_mot();
			init_pos_mot();
			axage();

			//initialise les moteurs avant d'avancer de 2cm
			init_vitesse_mot();
			init_pos_mot();
			marche_avant_s(2.0,  true, true, true, false, false);

			//initialise les moteurs avant de reculer de 2cm
			init_vitesse_mot();
			init_pos_mot();
			chThdSleepMilliseconds(400);
			marche_avant_s(-2.0,  true, true, true, false, false);

			//initialise les moteurs avant de se desaxer
			init_vitesse_mot();
			init_pos_mot();
			chThdSleepMilliseconds(400);
			re_axage();
		}
		//initialise les moteurs avant de faire marche arriere
		init_vitesse_mot();
		init_pos_mot();
		chThdSleepMilliseconds(400);
		marche_avant_s(-memoire_cm, true, true, true, false, false);

		 //initialise les moteurs avant de partir en direction de la poubelle
		init_vitesse_mot();
		init_pos_mot();
		chThdSleepMilliseconds(400);
		rotation_s(-90.0);
	}
	else
	{
		//ne recupere pas le colis
		init_vitesse_mot();
		init_pos_mot();
		chThdSleepMilliseconds(400);
		rotation_s(90.0);
		colis_recupere = false; 
	}
	return colis_recupere;
}

uint8_t detect_recup(uint8_t compteur)
{
	chprintf((BaseSequentialStream *)&SD3, "  pour l'instant je n'ai rien detecte = : %u  ",compteur);
	init_pos_mot();
	rotation_s(-90.0);
	chThdSleepMilliseconds(1000);

	if ((detec_rouge() && ((compteur == 0)||(compteur == 1)))) 
	{
		recup_colis(true);
	}
	else if((detec_rouge() == false) && (compteur != 1)) 
	{
		recup_colis(true);
    }
    else
    {
    	recup_colis(false);
    }
   
	return compteur;
}
/* Cette fonction permet au robot de bien se centrer
 * afin de recuperer l'objet de maniere parfaite sur sa tete
 * */
void axage(void)
{
	uint8_t axage = ZERO;
	axage = abs(get_calibrated_prox(PROX_FRONT_R17) - get_calibrated_prox(PROX_FRONT_L17));
	chThdSleepMilliseconds(MILLE);
	while((get_calibrated_prox(PROX_FRONT_R17) > get_calibrated_prox(PROX_FRONT_L17)) &&
				axage > SEUIL_AXAGE)
		{
			// recentrer
			chThdSleepMilliseconds(CENT);
			left_motor_set_speed(CENT);
			right_motor_set_speed(-CENT);
			compte_d++;
		}

		while((get_calibrated_prox(PROX_FRONT_R17) < get_calibrated_prox(PROX_FRONT_L17)) &&
				axage > SEUIL_AXAGE)
		{
			// recentrer
			chThdSleepMilliseconds(CENT);
			left_motor_set_speed(-CENT);
			right_motor_set_speed(CENT);
			compte_g ++;
		}
}

void re_axage(void)
{
	while(compte_d != ZERO)
		{
			chThdSleepMilliseconds(CENT);
			left_motor_set_speed(-CENT);
			right_motor_set_speed(CENT);
			compte_d --;
		}

		while(compte_g != ZERO)
		{
			chThdSleepMilliseconds(CENT);
			left_motor_set_speed(CENT);
			right_motor_set_speed(-CENT);
			compte_g --;
		}
}

void next_balise(void)
{
		init_pos_mot();
		init_vitesse_mot();
	//	float temps_rampe = (float)VITESSE_INTERM/ACCELERATION_MAX;
	//	int16_t tics_rampe = (0.5)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe;
		marche_avant_s(5.0,true,false,true,true,false);
		init_pos_mot();
		marche_avant_s(50.0,false,true,true,true,true);
		init_vitesse_mot();

}

void next_porte(int16_t speed)
{
		init_pos_mot();
		init_vitesse_mot();

		while(detection_porte(true)==false)
			marche_avant(speed);

		init_vitesse_mot();
}

void retour_base(int16_t tics_retour)
{
	init_pos_mot();
	init_vitesse_mot();
	marche_avant_s(10.0,true,true,true,false,false);
	init_pos_mot();
	init_vitesse_mot();
	marche_avant_s(-10.0,  true, true, true, false,false);
	init_pos_mot();
	init_vitesse_mot();
	/*	init_pos_mot();
		rotation_s(180.0); */
	init_pos_mot();
	rotation_s(90.0);
	init_pos_mot();
	rotation_s(90.0);
	init_pos_mot();
	init_vitesse_mot();
	//marche_avant_s(StepsToCm(tics_retour),true,true,false,false,false);
	marche_avant_s(800.0,true,true,false,false,false);
	init_pos_mot();
	rotation_s(360.0);
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
	int16_t tics_rampe = (0.5)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe+(float)VITESSE_INTERM*temps_rampe;
	if(balise_a_route)
		vitesse_prec = VITESSE_INTERM;
	else
		vitesse_prec = MAX_VITESSE;
	init_pos_mot();
	while(abs(right_motor_get_pos())<tics_rampe)
	{
		if(balise_a_route)
		{
			v_a_max = vitesse_prec + ACCELERATION_MAX*((4.0)*(0.001));
		}
		else
		{
			v_a_max = vitesse_prec - ACCELERATION_MAX*((4.0)*(0.001));
		}
		vitesse_prec = v_a_max;
		marche_avant(vitesse_prec);
		chThdSleepMilliseconds(4);	
	}
}
