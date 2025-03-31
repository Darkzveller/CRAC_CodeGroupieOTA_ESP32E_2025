#include <Arduino.h>

#ifndef _USE_FUNCTION_H
#define _USE_FUNCTION_H

int16_t fusion_octet(int octet0, int octet1);
int convert_angle_deg_to_tick(float angle);
float convert_tick_to_angle_deg(float nbr_tick);

float convert_angle_radian_to_tick(float angle);
float convert_tick_to_angle_rad(float nbr_tick);

int convert_distance_mm_to_tick(float distance);
float convert_distance_tick_to_mm(float nbr_tick);

void pourcentage_erreur(float val_theorique, float valeur_experimentale);

float normaliser_angle_deg(float angle_deg);
float normaliser_angle_rad(float angle_rad);


#endif


