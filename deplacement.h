#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H

// Defines que l'on peut modifier
#define VITESSE_ARC						9
#define VITESSE_ROTA_ANGLES				200

//Defines optimises deconseille de modifier
#define ACCELERATION_MAX				500.0f		//a calculer (float)
#define MAX_VITESSE						1100
#define VITESSE_INTERM					500
#define SEUIL_VIT_NUL	  				40
#define DELTA_T_MS						5	
#define DELTA_T_S						0.005	


#define MARGE_POUSSEE					80
#define MAX_VITESSE_PI					1000
#define SEUIL_AJUSTEMENT 				20
#define SEUIL_VIT_NUL_PI				40
#define COEFF_MM_2_STEPS				100/13
#define COEFFSTEPSCM					100
#define PERIM_ROUE_CM					13.0f

//Constantes arbitraires
#define MARCHE_AVANT 					true
#define MARCHE_ARRIERE 					false
#define CHARGE 							true
#define PAS_CHARGE						false
#define COMPTEUR_BASE					0
#define MM_2_CM							0.1f
#define CENT							100
#define ABSENCE_OBJET					0
#define PRESENCE_OBJET					1
#define EN_CHEMIN						true
#define ARRET							false
#define VITESSE_NULLE					0

int16_t pi_regulator(float distance, float goal);
void turn_90(int16_t speed);
void init_pos_mot(void);
void init_vitesse_mot(void);
void trajet_rectiligne(float objectif, bool demarrage_s, bool freinage_s, bool charge);
void marche_avant_s(float objectif, bool demarrage_s, bool freinage_s, bool charge, bool zone_bornes, bool b_balise);
void rotation_s(float angle);
void tourner(int16_t speed);
void marche_avant(int16_t speed);
void detect_eject(uint8_t comptage);
uint8_t detect_recup(uint8_t compteur);
void axage(void);
void re_axage(void);
void next_balise();
void retour_base(void)
void balise_to_route(bool balise_a_route);



#endif /* DEPLACEMENT_H */
