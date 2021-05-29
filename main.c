#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <msgbus/messagebus.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <selector.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <audio/audio_thread.h>
#include <arm_math.h>
#include <deplacement.h>
#include <fcts_maths.h>
#include <detection.h>
#include <analyse_couleur.h>
#include <lumiere.h>



// Define pour ne pas avoir de magic number dans le code
#define DEUX_CENTS  200

// On initialise ici le bus afin de pouvoir utiliser les capteurs de proximite "proximity"
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Fonction qui nous a permis d'afficher sur REALterm  les chprintf du robot
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);


    //Start la communication en serie
    serial_start();
    //Start la communication USB
    usb_start();
    //Initialsation des moteurs pas a pas du robot
    motors_init();
    //Start le capteur longue distance
	VL53L0X_start();
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
    while (1) {

    	//Selecteur == 0 : mode static + jeu de LEDs
    	if(get_selector()==0)
    	{
            init_vitesse_mot();
            init_pos_mot();
    //		lumiere_demarrage();
            chThdSleepMilliseconds(1000);
            chprintf((BaseSequentialStream *)&SD3, "%i", Distance_to_temps(30.0, ACCELERATION_MAX, DELTA_T_S));
            jeu_de_lumiere();

    	}
        
    	

    	if(get_selector()==1)
    	{
            

            //tant que le robot n'est pas bloqué
            bool colis_a_deposer = true; 
            while(1)
            {
                int16_t tics_retour = 0;
                init_pos_mot();

                next_balise();

                tics_retour += right_motor_get_pos();
                init_pos_mot();
                //recuperation du colis unique
                colis_a_deposer = recup_colis(true);
                init_pos_mot();
                //sortie de la zone de recuperation
                marche_avant_s(5, true, false, true, true, false);
                tics_retour += right_motor_get_pos();
                init_pos_mot();

                //passage par portes plus eloignement des portes
                next_porte(VITESSE_INTERM);
                tics_retour += right_motor_get_pos();
                init_pos_mot();

                balise_to_route(true);
                tics_retour += right_motor_get_pos();
                init_pos_mot();
                
                marche_avant_s(5, false, false, true, false, false);
                tics_retour += right_motor_get_pos();
                init_pos_mot();

                next_porte(MAX_VITESSE);
                tics_retour += right_motor_get_pos();
                init_pos_mot();

                balise_to_route(false);
                tics_retour += right_motor_get_pos();
                init_pos_mot();
                //sortie de l'autoroute
                tics_retour += right_motor_get_pos();
                init_pos_mot();
                //entree dans la zone de balises
                marche_avant_s(50.0,false,true,true,true,true);
                tics_retour += right_motor_get_pos();
                eject_colis(colis_a_deposer);
                retour_base(tics_retour);
            }            
    	}

        /*
        *  Selecteur == 15 :
        *  
        * 
        *   
        */
        if(get_selector()==15)
        {
            rotation_s(180.0);
            init_pos_mot();
            init_vitesse_mot();
            rotation_s(180);
            init_pos_mot();
            init_vitesse_mot();
        }
        
        /*
        *  Selecteur == 14 :
        *  
        *  
        *  
        */

        if((get_selector()==14))
        {
   /*     	uint8_t compteur = 0;
        	next_balise();
        	compteur = detect_recup(compteur);
        	next_balise();
            compteur = detect_recup(compteur);
            //sortie de la zone de balises
            marche_avant_s(5, true, false, true, true, false);
            balise_to_route(true);
            init_pos_mot();
            marche_avant_s(15, false, false, true, false, false);
            init_pos_mot();
            balise_to_route(false);
            init_pos_mot();
            //entree dans la zone de balises
            marche_avant_s(50.0,false,true,true,true,true);
        	compteur = detect_eject(compteur);
            next_balise();
            compteur = detect_eject(compteur);
        	retour_base(); */
        }




    	else
    	{
    		// Mode static sans jeu de lumiere 
    		lumiere_eteinte();
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
