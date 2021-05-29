#include "ch.h"
#include "hal.h"
#include "camera/dcmi_camera.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <camera/po8030.h>

#include <main.h>
#include <analyse_couleur.h>


//Moyenne des pixels dans le rouge
static uint16_t moyenne_r = 0;
//Semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


// Thread deja codee pour la capture d'une image
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(WHILE_INFINI){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be dones
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

// Thread pour enregistrer les valeurs du rouge
static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg){

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_r[IMAGE_BUFFER_SIZE] = {0};	//tableau pour la couleur rouge

    while(WHILE_INFINI)
    {
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		for(uint16_t i=0; i<RATIO_8B_16B*IMAGE_BUFFER_SIZE; i+=RATIO_8B_16B)
		{
			image_r[i/RATIO_8B_16B] = (uint8_t)img_buff_ptr[i]&MASQUE_ROUGE; //rouge
		}
		moyenne_r = moyenne_ligne(image_r);
		chprintf((BaseSequentialStream *)&SD3, "   %u   ", moyenne_r );
	}
}

// Fonction qui fait la moyenne des pixels obtenus
uint32_t moyenne_ligne(uint8_t *buffer)
{
	uint32_t mean = 0;
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++)
	{
		mean += buffer[i];
	}
	return mean /= IMAGE_BUFFER_SIZE;
}

/*
* 		Return true si objet rouge en face de l'epuck,
* 		selon un seuil arbitraire. Sinon, retourne false
*  		 
*/
bool balise_rouge(void)
{
	if(moyenne_r<SEUIL_ROUGE)
		return false;
	else
		return true;
}

void process_image_start(void)
{
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
