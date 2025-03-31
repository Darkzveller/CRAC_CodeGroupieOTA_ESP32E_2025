#include "Variable.h"
#include "USE_FUNCTION.h"

#include <mat.h>

int16_t fusion_octet(int octet0, int octet1)
{

    int16_t octet16 = (octet0 << 8) | octet1;
    return octet16;
}

int convert_angle_deg_to_tick(float angle)
{
    float distance_a_faire_en_mm = angle * perimetre_robot / 360.0;
    int consigne_roue_odo = distance_a_faire_en_mm * (TIC_PER_TOUR / (2.0 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));

    return consigne_roue_odo;
}
float convert_tick_to_angle_deg(float nbr_tick)
{

    float angle_deg = nbr_tick * (2.0 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0) / TIC_PER_TOUR;
    angle_deg = angle_deg / perimetre_robot * 360.0;
    // int consigne_roue_odo = distance * (TIC_PER_TOUR / (2.0 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));
    return angle_deg;
}

float convert_angle_radian_to_tick(float angle)
{
    float distance_a_faire_en_mm = angle * perimetre_robot / (2 * M_PI);
    int consigne_roue_odo = distance_a_faire_en_mm * (TIC_PER_TOUR / (2.0 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));

    return consigne_roue_odo;
}
float convert_tick_to_angle_rad(float nbr_tick)
{

    float angle_rad = nbr_tick * (2.0 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0) / TIC_PER_TOUR;
    angle_rad = angle_rad / perimetre_robot * (2 * M_PI);
    // int consigne_roue_odo = distance * (TIC_PER_TOUR / (2.0 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));
    return angle_rad;
}

int convert_distance_mm_to_tick(float distance)
{

    int consigne_roue_odo = distance * (TIC_PER_TOUR / (2.0 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));
    return consigne_roue_odo;
}
float convert_distance_tick_to_mm(float nbr_tick)
{

    float distance_mm = nbr_tick * (2.0 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0) / TIC_PER_TOUR;
    // int consigne_roue_odo = distance * (TIC_PER_TOUR / (2.0 * M_PI * SIZE_WHEEL_DIAMETER_mm / 2.0));
    return distance_mm;
}

void pourcentage_erreur(float val_theorique, float valeur_experimentale)
{
    float pourcent_mes = 100 * ((val_theorique)-valeur_experimentale) / (val_theorique);
    Serial.print(pourcent_mes);
}

float normaliser_angle_deg(float angle_deg) {
    return fmod(angle_deg + 180.0, 360.0) - 180.0;
}

float normaliser_angle_rad(float angle_rad) {
    return fmod(angle_rad + M_PI / 2, 2.0 * M_PI) - (M_PI / 2.0);
}
