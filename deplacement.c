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
#include <audio/play_melody.h>
#include <sensors/proximity.h>


#include <analyse_couleur.h>
#include <detection.h>
#include <deplacement.h>
#include <fcts_maths.h>
#include <lumiere.h>


//Compteurs de cycles des moteurs, pour memoriser la correction apres axage_steps()
static int16_t tics_d = 0;
static int16_t tics_g = 0;


//Met les compteurs des moteurs à 0
void init_pos_mot(void)
{
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}
//Met les vitesses des moteurs à 0 
void init_vitesse_mot(void)
{
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

/*
* 	Fonction de deplacement rectiligne:
*	En entrée: distance à parcourir en cm (float); choix sur le demarrage et le freinage, si en douceur ou en choc;
*	choix si arrêt devant un obstacle ou pas; choix sur la vitessse du trajet, à MAX_VITESSE ou VITESSE_INTERM;
*	choix si arrêt lors de la présence d'un obstacle unique à la gauche. 
*/
void marche_avant_s(float objectif, bool demarrage_s, bool freinage_s, bool charge, bool zone_bornes, bool b_balise)
{
	//true si en route, false si on veut un arrêt
	bool onRoad = EN_ROUTE;
	// true si avant, false si arriere
	bool sens = (right_motor_get_pos()<CmToSteps(objectif)); 
	
	//calcul de l'acceleration
	int16_t v_a_max = 0;
	int16_t vitesse_palier = 0;
	if(zone_bornes)
		vitesse_palier = VITESSE_INTERM;
	else
		vitesse_palier = MAX_VITESSE;
	int16_t vitesse_prec = 0;
	//on calcule le nb de cycles pour une pente d'acc de 0 à vitesse max avec acceleration = ACCELERATION_MAX
	float temps_rampe = (float)vitesse_palier/ACCELERATION_MAX; 
	int16_t tics_rampe = (UN_DEMI)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe;
	//partage en 3 zones de vitesse, zone ascendante, constante, descendante
	int16_t tics1 = tics_rampe;
	int16_t tics3 = tics_rampe;
	//cas ou: objectif trop court pour atteindre vit max
	if((abs(CmToSteps(objectif))<(tics1+tics3))&&(demarrage_s==DEMAR_DOUX)&&(freinage_s==FREIN_DOUX))
	{
		tics1 = abs(CmToSteps(objectif) / VAL2);
		tics3 = abs(CmToSteps(objectif) / VAL2);
	}
	//cas ou: demarrage brutal
	if (demarrage_s==DEMAR_CHOC)
	{
		tics1 = 0;
	}
	//cas ou: freinage brutal
	if (freinage_s==FREIN_CHOC)
	{
		tics3 = 0; 
	}

	//Zone ou la vitesse augmente (selon le sens) de maniere linéaire 
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
	//Zone ou la vitesse reste constance (selon le sens) selon le max de vitesse choisi 
	while((abs(right_motor_get_pos())+OFFSET_STEPS)<(abs(CmToSteps(objectif))-tics3)
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
	//Zone ou la vitesse diminue (selon le sens) de maniere linéaire 
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
* 	Fonction de rotation sans choc: 
*	En entrée: un angle en degrés (float), un demarrage et un freinage
*	en douceur, y sont imposés.	
*/
void rotation_s(float angle)
{
	//true si en route, false si on veut un arrêt
	bool onRoad = EN_ROUTE;
	//calcul de l'arc de cercle que parcourt chaque moteur pour tourner en cm
	float objectif = angle_to_arc(angle,D_ENTRE_ROUES_CM);
	//true si sens horaire, false si antihoraire
	bool sens = (right_motor_get_pos()<CmToSteps(objectif)); 
	//calcul de l'acceleration
	int16_t v_a_max = 0;
	int16_t vitesse_palier = 0;
	vitesse_palier = VITESSE_INTERM;
	int16_t vitesse_prec = 0;
	//on calcule le nb de cycles pour une pente d'acc de 0 à vitesse max avec acceleration = ACCELERATION_MAX
	float temps_rampe = (float)vitesse_palier/ACCELERATION_MAX; 
	int16_t tics_rampe = (UN_DEMI)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe;
	//partage en 3 zones de vitesse, zone ascendante, constante, descendante
	int16_t tics1 = tics_rampe;
	int16_t tics3 = tics_rampe;

	//objectif trop court pour atteindre vit max
	if((abs(CmToSteps(objectif))<(tics1+tics3)))
	{
		tics1 = abs(CmToSteps(objectif) / VAL2);
		tics3 = abs(CmToSteps(objectif) / VAL2);
	}

	//Zone ou la vitesse augmente (selon le sens) de maniere linéaire 
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
		tourner(vitesse_prec);
		chThdSleepMilliseconds(DELTA_T_MS);	
	}
	//Zone ou la vitesse reste constance (selon le sens) selon le max de vitesse choisi 
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
	//Zone ou la vitesse diminue (selon le sens) de maniere linéaire 
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
		tourner(vitesse_prec);
		chThdSleepMilliseconds(DELTA_T_MS);
	}
	init_vitesse_mot();
	chThdSleepMilliseconds(PAUSE200);
}	

/*
 * 	Impose la meme vitesse sur les deux moteurs
 */
void marche_avant(int16_t speed)
{
	left_motor_set_speed(speed);
	right_motor_set_speed(speed); 
}

/*
 *	Impose deux vitesses opposées sur les moteurs, et si speed>0 : rotation horaire
 */
void tourner(int16_t speed)
{
	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);
}

/*
 *	Si b_recup = true : faire 180 degres, charger jusqu'a obstacle, si la distance parcourue est < a SEUIL_STOP,
 *	revenir a la position avant charge, faire 90 degres dans le sens anti-horaire et renvoyer false: le colis n'a 	
 *	pas été récupéré. Si la distance parcourue est > a SEUIL_STOP, il y a charge jusqu'a obstacle, axage, recuperation
 *	du colis en poussant un support de 3 cm, re-axage, retour a la base avant charge, rotation de 90 degres dans 
 *	le sens anti-horaire, et finalement un return de true, car colis récupéré. 
 *  si b_recup = false : 90 degres dans le sens horaire, et return d'un false car colis non récupéré. 
 */
bool recup_colis(bool b_recup)
{
	bool colis_recupere = COLIS_NON_RECUP; 
	if(b_recup)
	{
		init_pos_mot();
		rotation_s(ANGLE180);
		init_pos_mot();
		marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_CHOC, PAS_CHARGE, ZONE_BORNES, PORTE_FALSE);
        //permet de revenir d'exactement la bonne distance
    	int16_t memoire = right_motor_get_pos();
    	float memoire_cm = StepsToCm(memoire);
    	//si on arrete la trajectoire sur une distance courte de SEUIL_STOP, pas de colis à recupérer: arret
    	if(memoire_cm < SEUIL_STOP)
    		colis_recupere = COLIS_NON_RECUP;
    	else
    	{
    		colis_recupere = COLIS_RECUP;
    		//intialise les moteurs avant de s'axer devant l'objet
			init_vitesse_mot();
			init_pos_mot();
			axage_steps();
			//initialise les moteurs avant d'avancer de 3cm
			init_vitesse_mot();
			init_pos_mot();
			marche_avant_s(POUSSEE_3CM,  DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);
			//initialise les moteurs avant de reculer de 3cm
			init_vitesse_mot();
			init_pos_mot();
			chThdSleepMilliseconds(PAUSE400);
			marche_avant_s(-POUSSEE_3CM,  DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);
			//initialise les moteurs avant de se desaxer
			init_vitesse_mot();
			init_pos_mot();
			chThdSleepMilliseconds(PAUSE400);
			re_axage_steps();
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

/*
 *	Si b_eject = true : rotation de 180 degres, charge jusqu'a detection de la poubelle avec arret brutal
 * 	pour ejecter le colis sur l'epuck, accompagné d'une mélodie, puis retour a la position avant charge, 
 *	et 2 charges supplémentaires pour etre sur d'avoir ejecté le colis. Rotation de 90 deg dans le 
 *	sens anti-horaire.
 *	si b_eject = false : Rotation de 90 deg dans le sens anti-horaire. 
 */
void eject_colis(bool b_eject)
{
	init_pos_mot();
	rotation_s(-ANGLE90);
	chThdSleepMilliseconds(PAUSE100);
	if (b_eject)
	{
		init_pos_mot();
		rotation_s(ANGLE180);

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
    	memoire = right_motor_get_pos();
    	memoire_cm = StepsToCm(memoire);
        init_pos_mot();
        init_vitesse_mot();
        chThdSleepMilliseconds(PAUSE100);
        marche_avant_s(-memoire_cm, DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);
        init_pos_mot();
   		//3 coup
        marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_CHOC, PAS_CHARGE, AUTOROUTE, PORTE_FALSE);
    	memoire = right_motor_get_pos();
    	memoire_cm = StepsToCm(memoire);
        init_pos_mot();
        init_vitesse_mot();
        chThdSleepMilliseconds(PAUSE100);
        marche_avant_s(-memoire_cm, DEMAR_DOUX, FREIN_DOUX, CHARGE, AUTOROUTE, PORTE_FALSE);
        init_pos_mot();

        //rotation de -90 deg pour se remettre sur la route initiale
		rotation_s(-ANGLE90);
	}
	else
	{
		init_pos_mot();
		rotation_s(ANGLE90);
		init_pos_mot();
	}
}


/* 
 *	Cette fonction permet au robot de bien se centrer
 *	afin de recuperer l'objet de maniere parfaite sur sa tete
 */
void axage_steps(void)
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
	}

	while((get_calibrated_prox(PROX_FRONT_R17) < get_calibrated_prox(PROX_FRONT_L17)) &&
			axage > SEUIL_AXAGE)
	{
		// recentrer
		chThdSleepMilliseconds(PAUSE100);
		left_motor_set_speed(-VIT100);
		right_motor_set_speed(VIT100);
	}
	// Memoire de cycles a reutiliser pour le re-axage
	tics_d = right_motor_get_pos();
	tics_g = left_motor_get_pos();
}

/* 
 *	Cette fonction permet au robot de se re-axer
 *	avant de retourner a la position initiale de charge, 
 *	lors de la fonction de recuperation de colis. 
 */
void re_axage_steps(void)
{
	right_motor_set_pos(tics_d);
	left_motor_set_pos(tics_g);
	if(tics_d >= 0 && tics_g<=0)
	{
		while(right_motor_get_pos() > 0)
		{
			right_motor_set_speed(-VIT100);
			left_motor_set_speed(VIT100);
		}
	}
	if(tics_d <= 0 && tics_g>=0)
	{
		while(right_motor_get_pos() < 0)
		{
			right_motor_set_speed(VIT100);
			left_motor_set_speed(-VIT100);
		}
	}	
}

/* 
 *	Permet de sortir de la zone de detection d'une balise pour 
 *	rejoindre une autre zone. 
 */
void next_balise(void)
{
	init_pos_mot();
	init_vitesse_mot();
	//obstacles non detectés autour de lui, afin qu'il ne prenne pas la balise précédente
	//pour une nouvelle. 
	marche_avant_s(SORTIE_BALISE,DEMAR_DOUX,FREIN_CHOC,CHARGE,ZONE_BORNES,PORTE_FALSE);
	init_pos_mot();
	//les balises et portes sont de nouveau détéctables autour du robot
	marche_avant_s(DIST_MAX,DEMAR_CHOC,FREIN_DOUX,CHARGE,ZONE_BORNES,PORTE_TRUE);
	init_vitesse_mot();
}

/* 
 *	Permet de garder une vitesse constante, donnee en entree 
 *	tant qu'une porte n'a pas encore été franchie. 
 */
void next_porte(int16_t speed)
{
	init_pos_mot();
	init_vitesse_mot();
	while(detection_porte(PORTE_TRUE)==PORTE_FALSE)
		marche_avant(speed);
	init_vitesse_mot();
}

/* 
 *	Permet de retourner a la base, en contournant grâce a des murs, la zone de travail.
 *	L'epuck, a cause des chocs, risquant d'etre désaxé, va se reaxer face au mur, 	
 *	faire 90 degres dans le sens antihoraire, longer le mur, se reaxer face a un mur
 *	perpendiculaire, faire 90 degres, prendre une autoroute le long de la zone de travail,
 *	puis arrivé a la hauteur de la position de départ, on se reaxe face a un mur perpendiculaire,
 *	on fait 90 degres dans le sens anti-horaire, et longeons le mur, jusqu'a un obstacle.
 * 	Cette dernière position sera la position de départ pour un nouveau cycle de recup/eject du colis.
 */

void retour_base(void)
{
	init_pos_mot();
	init_vitesse_mot();
	marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_DOUX, PAS_CHARGE, ZONE_BORNES, PORTE_FALSE);
	init_pos_mot();
	init_vitesse_mot();
	axage_steps();
	init_pos_mot();
	init_vitesse_mot();
	rotation_s(-ANGLE90);
	init_pos_mot();
	init_vitesse_mot();
	marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_DOUX, PAS_CHARGE, ZONE_BORNES, PORTE_FALSE);
	init_pos_mot();
	init_vitesse_mot();
	axage_steps();
	init_pos_mot();
	rotation_s(-ANGLE90);
	init_pos_mot();
	marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_DOUX, PAS_CHARGE, AUTOROUTE, PORTE_FALSE);
	init_pos_mot();
	init_vitesse_mot();
	axage_steps();
	init_pos_mot();
	rotation_s(-ANGLE90);
	init_pos_mot();
	marche_avant_s(DIST_MAX, DEMAR_DOUX, FREIN_DOUX, PAS_CHARGE, ZONE_BORNES, PORTE_FALSE);
	init_pos_mot();
	init_vitesse_mot();
	axage_steps();
	init_pos_mot();
	rotation_s(-ANGLE90);
	init_pos_mot();
	jeu_de_lumiere();
}

/*
*	si bool_a_route = true, on accelere de VITESSE_INTERM a MAX_VITESSE,
*	sinon on décélère de MAX_VITESSE a VITESSE_INTERM. 
*	fonction utilisable que en marche avant.
*/
void balise_to_route(bool balise_a_route)
{
	int16_t v_a_max = 0;
	float temps_rampe = (float)(MAX_VITESSE-VITESSE_INTERM)/ACCELERATION_MAX; 
	int16_t tics_rampe = (UN_DEMI)*ACCELERATION_MAX*(float)temps_rampe*(float)temps_rampe+(float)VITESSE_INTERM*temps_rampe;
	int16_t vitesse_prec = 0;
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
