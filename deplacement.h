#ifndef DEPLACEMENT_H
#define DEPLACEMENT_H


int16_t pi_regulator(float distance, float goal);
void turn_90(int16_t speed);
void init_pos_mot(void);
void init_vitesse_mot(void);

#endif /* DEPLACEMENT_H */
