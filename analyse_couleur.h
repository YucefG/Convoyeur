#ifndef ANALYSE_COULEUR_H
#define ANALYSE_COULEUR_H


//defines du hardware
#define IMAGE_BUFFER_SIZE		640

//defines arbitraires 
#define SEUIL_ROUGE				120
#define RATIO_8B_16B			2
#define MASQUE_ROUGE			0xF8
#define WHILE_INFINI			1


uint32_t moyenne_ligne(uint8_t *buffer);
bool balise_rouge(void);
void process_image_start(void);


#endif /* ANALYSE_COULEUR_H */
