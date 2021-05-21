#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <msgbus/messagebus.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <chprintf.h>
#include <motors.h>
#include <selector.h>
#include <sensors/proximity.h>
#include <audio/play_melody.h>
#include <audio/audio_thread.h>
#include <arm_math.h>
#include <lumiere.h>


// Pour enlever les magiques numbers
#define ZERO		0
#define UN			1
#define CINQ		5
#define MILLE		1000

// On initialise ici le bus afin de pouvoir utiliser les capteurs de proximite "proximity"
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", CINQ);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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
    while (UN) {

    	//Selecteur == 0 : mode static + jeu de LEDs
    	if(get_selector()==ZERO)
    		jeu_de_lumiere();

    	/* Selecteur == 1 :
    	 * Dans ce selecteur on va positionner les robot devant une couleur (RGB)
    	 * on va ensuite poser un objet sur le robot
    	 * le robot va se diriger dans le direction de la poubelle
    	 * de la couleur vue précedemment
    	 * le robot va devoir gerer son acceleration pour jeter l'objet dans la poubelle
    	*/
        if(get_selector()==UN)
        {

            //boucle infinie pour la fin de la tache
            while(UN)
            {
                 lumiere_clignote();
                 chThdSleepMilliseconds(MILLE);
            }
        }
    	else
    	{
    		// Mode static sans jeu de lumiere
    		lumiere_eteinte();
    	}
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
