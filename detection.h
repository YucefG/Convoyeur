#ifndef MESURE_H
#define MESURE_H

//Defines du hardware non modifiable
#define ANGLE360				360.0f
#define D_ENTRE_ROUES_CM		5.3f		//en cm
#define TICS_1_TOUR				1000


#define PROX_FRONT_R17			0
#define PROX_FRONT_R49			1
#define PROX_R					2
#define PROX_BACK_R				3
#define PROX_BACK_L				4
#define PROX_L					5
#define PROX_FRONT_L49			6
#define PROX_FRONT_L17			7


//Defines optimises 
#define LIMITE_COLLISION		1000
#define SEUIL_PORTE				20
#define SEUIL_BALISE			20

#define OBSTACLE_FRONT_DETECTE	false
#define RIEN_FRONT_DETECTE		true

#define BALISE_DETECTE			false
#define BALISE_NON_DETECTE		true

#define PORTE_DETECTE			true
#define PORTE_NON_DETECTE		false


bool prox_distance(bool charge);
bool detection_balise(bool b_balise);
bool detection_porte(bool b_portes);

#endif /* MESURE_H */
