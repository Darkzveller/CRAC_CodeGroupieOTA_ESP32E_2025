#include <Arduino.h>

#ifndef _MOUVEMENT_H
#define _MOUVEMENT_H

void rotation(int consigne, int vitesse);
void ligne_droite(int consigne, int vitesse);
void x_y_theta(float coordonnee_x, float coordonnee_y, float theta_fin, int vitesse);
void asser_polaire_tick(float coordonnee_x, float coordonnee_y, float theta_cons,bool nbr_passage);

void  recalage();
#endif
