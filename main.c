#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <msgbus/messagebus.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <selector.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <audio/audio_thread.h>
#include <arm_math.h>

#include <main.h>
#include <deplacement.h>
#include <fcts_maths.h>
#include <detection.h>
#include <analyse_couleur.h>
#include <lumiere.h>

// On initialise ici le bus afin de pouvoir utiliser les capteurs de proximite "proximity"
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
    halInit();
    chSysInit();
   // mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //Initialsation des moteurs pas a pas du robot
    motors_init();
	//Start des capteurs de proximite
	proximity_start();
	//Start la camera du robot
	dcmi_start();
    po8030_start();
    //Mise du white balance a zero pour mieux distinguer les couleurs RGB
    po8030_set_awb(0);
    // Start de l'analyse d'image
	process_image_start();
	// Calibration du capteur de courte proximite
	calibrate_ir();
	//Start les melodies utilisees par le robot
	dac_start();
	playMelodyStart();


    /* Boucle infinie */
    while (WHILE_INFINI) 
    {	
        /* 
        *   Si selector == 0 : moteurs à l'arret + jeu de lumieres
        */
    	if(get_selector()==SELECTEUR0)
    	{
            init_vitesse_mot();
            init_pos_mot();
            jeu_de_lumiere();
    	}
        
        /* 
        *   Si selector == 1 : fonction principale:
        *   tant que colis_a_deposer == true, on refait la fonction principale:
        *   detecter la 1ere balise rouge, lancer la fonction recup_colis,
        *   prendre l'autoroute, deposer le colis, apres avoir détecté une 2e balise 
        *   rouge, puis revenir a la base de depart avec retour_base. 
        */
    	if(get_selector()==SELECTEUR1)
    	{
            bool colis_a_deposer = COLIS_RECUP; 
            //tant que le robot n'est pas bloqué sur son chemin pour recup le colis
            while(colis_a_deposer==COLIS_RECUP)
            {
                init_pos_mot();
                //avancer jusq'a detection d'une balise sur la gauche
                next_balise();
                init_pos_mot();

                //orientation devant le gobelet
                init_pos_mot();
                rotation_s(-ANGLE90);
                chThdSleepMilliseconds(PAUSE400);
                init_pos_mot();
                init_vitesse_mot();

                /*
                *   si balise est rouge, recup du colis: si colis recup, colis_a_deposer == true
                *   sinon colis_a_deposer == false
                */
                colis_a_deposer = recup_colis(balise_rouge());
                init_pos_mot();

                //sortie de la zone de recuperation
                marche_avant_s(SORTIE_BALISE, DEMAR_DOUX, FREIN_CHOC, CHARGE, ZONE_BORNES, PORTE_FALSE);
                init_pos_mot();

                //vitesse constante jusqu'a portes
                next_porte(VITESSE_INTERM);
                init_pos_mot();

                //acceleration de VIT_INTERMEDIAIRE a MAX_VITESSE
                balise_to_route(INTERM_TO_MAX);
                init_pos_mot();

                //eloignement de la porte
                marche_avant_s(SORTIE_BALISE, DEMAR_CHOC, FREIN_CHOC, CHARGE, AUTOROUTE, PORTE_FALSE);
                init_pos_mot();

                //vitesse constante jusqu'a portes
                next_porte(MAX_VITESSE);
                init_pos_mot();

                //acceleration de MAX_VITESSE a VIT_INTERMEDIAIRE
                balise_to_route(MAX_TO_INTERM);
                init_pos_mot();

                //sortie de l'autoroute
                init_pos_mot();

                //entree dans la zone de balises
                marche_avant_s(DIST_MAX,DEMAR_CHOC,FREIN_DOUX,CHARGE,ZONE_BORNES,PORTE_TRUE);
                init_pos_mot();

                //si colis_a_deposer == true, eject du colis, sinon pas d'eject
                eject_colis(colis_a_deposer);

                //retour a la base
                init_pos_mot();
                retour_base();
            } 
            while(WHILE_INFINI)
            {
                init_vitesse_mot();
                jeu_de_lumiere();
            }           
    	}
 
    	else
    	{
    		// Mode static sans jeu de lumiere 
			init_vitesse_mot();
    	}
    }
 }

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
