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

    	}
        
        /*
        *  Selecteur == 1 :
        *       Tests pour l'instant - accelerometre
        *    
        */
        if(get_selector()==2)
        {
            init_vitesse_mot();
            init_pos_mot();
    //      lumiere_demarrage();
   //         systime_t time; 
            while(1)
            {
      /*          //tests: 
                //marche avant avec accel et frein controles
                trajet_rectiligne(20.0, true, true, true);
                
                chThdSleepMilliseconds(400); 
                init_vitesse_mot();
                init_pos_mot();

                //marche avant avec acoup au demarrage et freinage controle
                trajet_rectiligne(20.0, false, true, true); 

                chThdSleepMilliseconds(400); 
                init_vitesse_mot();
                init_pos_mot();

                //marche avant avec acoups
                trajet_rectiligne(20.0, false, false, true); 
                chThdSleepMilliseconds(400); 
                init_vitesse_mot();
                init_pos_mot();
                
                //marche avant avec acoup a la fin
                trajet_rectiligne(20.0, true, false, true); 
                chThdSleepMilliseconds(400); 
                init_vitesse_mot();
                init_pos_mot();*/

            }
        }
    	
        /*
        * Selecteur == 2:
    	* Le robot au son  "purge" se met a analyser les objets dans son arene
    	* et sortira uniquement les rouges.
    	* Selecteur == 3:
        * le robot sort tous les objets.
        */
    	if(get_selector()==1)
    	{
           
            //boucle infinie pour la fin de la tache
            while(1)
            {
                //tests: 
                //marche avant avec accel et frein controles

            	init_vitesse_mot();
            	init_pos_mot();
            	marche_avant_s(10.0,  true, true, true, false, false);
                init_vitesse_mot();
                init_pos_mot();
            	axage();
                init_vitesse_mot();
                init_pos_mot();
            	marche_avant_s(2.0,  true, true, true, false, false);

                init_vitesse_mot();
                init_pos_mot();
                chThdSleepMilliseconds(400);

                marche_avant_s(-12.0, true, true, true, false, false);            //charge pour faire tomber lobjet
                
                init_vitesse_mot();
                init_pos_mot();
                chThdSleepMilliseconds(400);


                turn_90(-500);

                init_vitesse_mot();
                init_pos_mot();

                marche_avant_s(20.0, true, true, true, false, true);
                detect_eject();

                init_vitesse_mot();
                init_pos_mot();

                turn_90(-600);

                while(1)
                {

                }







          /*      marche_avant_s(30.0, true, false, true, false);
                
                chThdSleepMilliseconds(400); 
                init_vitesse_mot();
                init_pos_mot();
                marche_avant_s(-30.0, true, false, true, false);
                
                chThdSleepMilliseconds(400); 
                init_vitesse_mot();
                init_pos_mot();
                marche_avant_s(30.0, true, true, false, false);
                
                chThdSleepMilliseconds(400); 
                init_vitesse_mot();
                init_pos_mot();
                marche_avant_s(-30.0, true, true, false, false);
                
                chThdSleepMilliseconds(400); 
                init_vitesse_mot();
                init_pos_mot(); */
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
            init_vitesse_mot();
            init_pos_mot();
            chThdSleepMilliseconds(3000);
            rotation_s(180.0);
            init_vitesse_mot();
            init_pos_mot();
            chThdSleepMilliseconds(3000);

             rotation_s(360.0);
            init_vitesse_mot();
            init_pos_mot();
            chThdSleepMilliseconds(3000);
            rotation_s(-360.0);
            init_vitesse_mot();
            init_pos_mot();
            chThdSleepMilliseconds(3000);

        }
        
        /*
        *  Selecteur == 14 :
        *  
        *  
        *  
        */

        if((get_selector()==14))
        {
        	init_vitesse_mot();
        	init_pos_mot();
        	marche_avant_s(50.0, true, true, true, false, true);
        	detect_eject();
            while(1)
            {
                
            }
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
