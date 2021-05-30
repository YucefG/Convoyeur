#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H


//defines optimises deconseille de modifier
#define ACCELERATION_MAX				500.0f		
#define MAX_VITESSE						1100
#define VITESSE_INTERM					500
#define SEUIL_VIT_NUL	  				40
#define OFFSET_STEPS					3
#define DELTA_T_MS						4	
#define DELTA_T_MS_F					4.0f
#define DELTA_T_S						0.004	
#define SEUIL_STOP 						10.0f
#define DIST_MAX						100.0f
#define	SEUIL_AXAGE						20
#define POUSSEE_3CM						3.0f
#define SORTIE_BALISE					5.0f

//defines de comprehension
#define EN_ROUTE						true
#define ARRET							false
#define DEMAR_DOUX						true
#define DEMAR_CHOC						false
#define FREIN_DOUX						true
#define FREIN_CHOC						false
#define CHARGE							true
#define PAS_CHARGE						false
#define ZONE_BORNES						true
#define AUTOROUTE						false
#define PORTE_TRUE						true
#define PORTE_FALSE						false
#define COLIS_RECUP 					true
#define COLIS_NON_RECUP					false
#define INTERM_TO_MAX					true
#define MAX_TO_INTERM					false

//valeurs recurrentes
#define PAUSE100						100
#define PAUSE200						200
#define PAUSE400						400
#define PAUSE1000						1000
#define VIT100							100
#define UN_DEMI							0.5f
#define VAL2							2 
#define MS_TO_S							0.001f
#define ANGLE90							90.0f
#define ANGLE180						180.0f
#define PERIM_ROUE_CM					13.0f


void init_pos_mot(void);
void init_vitesse_mot(void);
void marche_avant_s(float objectif, bool demarrage_s, bool freinage_s, bool charge, bool zone_bornes, bool b_balise);
void rotation_s(float angle);
void tourner(int16_t speed);
void marche_avant(int16_t speed);
bool recup_colis(bool b_recup);
void eject_colis(bool b_eject);
void axage_steps(void);
void re_axage_steps(void);
void next_balise(void);
void retour_base(void);
void next_porte(int16_t speed);
void balise_to_route(bool balise_a_route);


#endif /* DEPLACEMENT_H */
