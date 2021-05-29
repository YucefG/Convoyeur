#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <motors.h>
#include <chprintf.h>
#include <selector.h>
#include <lumiere.h>


void leds_OFF(void)
{

	palSetPad(GPIOD, GPIOD_LED1);
	palSetPad(GPIOD, GPIOD_LED3);
	palSetPad(GPIOD, GPIOD_LED5);
	palSetPad(GPIOD, GPIOD_LED7);
}

void leds_ON(void)
{

	palClearPad(GPIOD, GPIOD_LED1);
	palClearPad(GPIOD, GPIOD_LED3);
	palClearPad(GPIOD, GPIOD_LED5);
	palClearPad(GPIOD, GPIOD_LED7);
}

// appelé au mode 0
void jeu_de_lumiere(void)
{
	leds_OFF();

	chThdSleepMilliseconds(TEMPS_COURT);// Regler la rapidite du jeu de lumiere

	palClearPad(GPIOD, GPIOD_LED1);
	palClearPad(GPIOD, GPIOD_LED5);

	chThdSleepMilliseconds(TEMPS_COURT);// Regler la rapidite du jeu de lumiere

    palSetPad(GPIOD, GPIOD_LED1);
    palSetPad(GPIOD, GPIOD_LED5);

    chThdSleepMilliseconds(TEMPS_COURT);

	palClearPad(GPIOD, GPIOD_LED3);
	palClearPad(GPIOD, GPIOD_LED7);

	chThdSleepMilliseconds(TEMPS_COURT);

    palSetPad(GPIOD, GPIOD_LED3);
    palSetPad(GPIOD, GPIOD_LED7);

    palClearPad(GPIOB, GPIOB_LED_BODY);
    chThdSleepMilliseconds(TEMPS_LONG);

     palSetPad(GPIOB, GPIOB_LED_BODY);
     leds_ON();

    chThdSleepMilliseconds(TEMPS_LONG);
   	leds_OFF();

   	palClearPad(GPIOB, GPIOB_LED_BODY);

    leds_OFF();
 }
