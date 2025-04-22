#include <Arduino.h>

#ifndef _MOUVEMENT_H
#define _MOUVEMENT_H

void rotation(int consigne, int vitesse);
void ligne_droite(int consigne, int vitesse);
void x_y_theta(float coordonnee_x, float coordonnee_y, float theta_fin, int vitesse);
void asser_polaire_tick(float coordonnee_x, float coordonnee_y, float theta_cons,bool nbr_passage);

// void  recalage();
bool recalage(uint8_t direction, uint8_t type_modif, uint16_t nouvelle_valeur, uint16_t consigne_rotation);
bool toucher_objet_solid();
#endif
