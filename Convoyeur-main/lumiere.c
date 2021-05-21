#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <chprintf.h>

#define CENT				100
#define DEUX_CENTS			200

// Cette fonction eteint toutes les lumieres
void lumiere_eteinte(void)
{

	palSetPad(GPIOD, GPIOD_LED1);
	palSetPad(GPIOD, GPIOD_LED3);
	palSetPad(GPIOD, GPIOD_LED5);
	palSetPad(GPIOD, GPIOD_LED7);

}


void lumiere_clignote(void)
{
	palTogglePad(GPIOD, GPIOD_LED1);
	palTogglePad(GPIOD, GPIOD_LED3);
	palTogglePad(GPIOD, GPIOD_LED5);
	palTogglePad(GPIOD, GPIOD_LED7);
}


//Cette fonction allume toutes les lumieres
void lumiere_allumee(void)
{
	palClearPad(GPIOD, GPIOD_LED1);
	palClearPad(GPIOD, GPIOD_LED3);
	palClearPad(GPIOD, GPIOD_LED5);
	palClearPad(GPIOD, GPIOD_LED7);
}
//Cette fonction est un jeu de lumiere lorsque le selecteur est a 0
void jeu_de_lumiere(void)
{
	palClearPad(GPIOD, GPIOD_LED1);
	palClearPad(GPIOD, GPIOD_LED5);

	chThdSleepMilliseconds(CENT);// Regler la rapidite du jeu de lumiere

    palSetPad(GPIOD, GPIOD_LED1);
    palClearPad(GPIOD, GPIOD_LED5);

    chThdSleepMilliseconds(CENT);

	palClearPad(GPIOD, GPIOD_LED3);
	palClearPad(GPIOD, GPIOD_LED7);

	chThdSleepMilliseconds(CENT);

    palSetPad(GPIOD, GPIOD_LED3);
    palClearPad(GPIOD, GPIOD_LED5);

    palClearPad(GPIOB, GPIOB_LED_BODY);

    chThdSleepMilliseconds(DEUX_CENTS);

    palSetPad(GPIOB, GPIOB_LED_BODY);
    lumiere_allumee();

   	chThdSleepMilliseconds(DEUX_CENT);
   	lumiere_eteinte();

   	palClearPad(GPIOB, GPIOB_LED_BODY);

    palClearPad(GPIOD, GPIOD_LED1);
    lumiere_allumee();
}

