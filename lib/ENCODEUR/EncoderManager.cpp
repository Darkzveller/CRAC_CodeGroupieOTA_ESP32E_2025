#include "Variable.h" // Remonte d'un niveau pour atteindre lib
#include "EncoderManager.h"

ESP32Encoder encodergauche;
ESP32Encoder encoderdroite;
float offset = 0;
void setup_encodeur()
{
    ESP32Encoder::useInternalWeakPullResistors = UP; // Utilise les résistances internes
    encoderdroite.attachHalfQuad(PIN_ENCODEUR_1, PIN_ENCODEUR_2);
    encodergauche.attachHalfQuad(PIN_ENCODEUR_3, PIN_ENCODEUR_4);

    encoderdroite.clearCount();
    encodergauche.clearCount();
}

void read_encodeurdroit()
{
    long double val_tick = encoderdroite.getCount() * COEFF_ROUE_DROITE;
    odo_tick_droit = val_tick;
    delta_odo_tick_droit = odo_tick_droit-odo_tick_droit_last;
    double dist = (2.0 * M_PI * (SIZE_WHEEL_DIAMETER_mm / 2.0) / TIC_PER_TOUR) * val_tick;
    odo_dist_droit = dist;
    odo_tick_droit_last = odo_tick_droit;
    // Serial.printf("dista = %4.2f\n", dist);

    /*
        long val_tick = encoderdroite.getCount() * COEFF_ROUE_DROITE;
        odo_tick_droit = val_tick;

        float number_tour = val_tick * 1.0 / TIC_PER_TOUR;
        float angle_actuelle_radians = 2.0 * M_PI * number_tour / 1.0;
        theta_droit = angle_actuelle_radians;
        float distance_parcourue_translation = angle_actuelle_radians * SIZE_WHEEL_mm / (2.0 * M_PI);
        odo_dist_droit = distance_parcourue_translation;
        float vitesse_angulaire = (angle_actuelle_radians - angle_precedent_droit) / Te;
        angle_precedent_droit = angle_actuelle_radians;
    */
    // Serial.printf("Tick= %.0f toru = %f angle %f vit %f dist %f\n", val_tick, number_tour, angle_actuelle_radians, vitesse_angulaire, distance_parcourue_translation);
    // if (grandeur_A_Mesure == SHOW_TICK)
    // {
    //     return val_tick;
    // }

    // if (grandeur_A_Mesure == SHOW_NUMBER_TOUR)
    // {
    //     return number_tour;
    // }
    // if (grandeur_A_Mesure == SHOW_ANGLE_RADIANS)
    // {
    //     return angle_actuelle_radians;
    // }
    // if (grandeur_A_Mesure == SHOW_DIST_MM)
    // {
    //     return distance_parcourue_translation;
    // }
    // if (grandeur_A_Mesure == SHOW_VITESSE_RAD_PAR_SEC)
    // {
    //     return vitesse_angulaire;
    // }
}

void read_encodeurgauche()
{
    long double val_tick = encodergauche.getCount() * COEFF_ROUE_GAUCHE;
    odo_tick_gauche = val_tick;
    delta_odo_tick_gauche = odo_tick_gauche-odo_tick_gauche_last;

    double dist = (2.0 * M_PI * (SIZE_WHEEL_DIAMETER_mm / 2.0) / TIC_PER_TOUR) * val_tick;
    odo_dist_gauche = dist;
    odo_tick_gauche_last = odo_tick_gauche;
    // Serial.printf("dista droite= %4.2f\n", dist);

    /*
        long val_tick = encodergauche.getCount() * COEFF_ROUE_GAUCHE;
        odo_tick_gauche = val_tick;

        float number_tour = val_tick * 1.0 / TIC_PER_TOUR;
        float angle_actuelle_radians = 2.0 * M_PI * number_tour / 1.0;
        theta_gauche = angle_actuelle_radians;
        float distance_parcourue_translation = angle_actuelle_radians * (SIZE_WHEEL_mm / 2) / (2.0 * M_PI);
        odo_dist_gauche = distance_parcourue_translation;

        float vitesse_angulaire = (angle_actuelle_radians - angle_precedent_gauche) / Te;
        angle_precedent_gauche = angle_actuelle_radians;

        Serial.printf("Tick= %.0d toru = %f angle %f vit %f dist %f\n", val_tick, number_tour, angle_actuelle_radians, vitesse_angulaire, distance_parcourue_translation);
        // if (grandeur_A_Mesure == SHOW_TICK)
        // {
        //     return val_tick;
        // }
    */
    // if (grandeur_A_Mesure == SHOW_NUMBER_TOUR)
    // {
    //     return number_tour;
    // }
    // if (grandeur_A_Mesure == SHOW_ANGLE_RADIANS)
    // {
    //     return angle_actuelle_radians;
    // }
    // if (grandeur_A_Mesure == SHOW_DIST_MM)
    // {
    //     return distance_parcourue_translation;
    // }
    // if (grandeur_A_Mesure == SHOW_VITESSE_RAD_PAR_SEC)
    // {
    //     return vitesse_angulaire;
    // }
}

void reset_encodeur()
{
    encoderdroite.clearCount();
    encodergauche.clearCount();
}
void read_x_y_theta()
{

    read_encodeurdroit();
    read_encodeurgauche();
    delta_droit = odo_dist_droit - odo_last_d;
    delta_gauche = odo_dist_gauche - odo_last_g;
    odo_last_d = odo_dist_droit;
    odo_last_g = odo_dist_gauche;

    distance_parcourue = 0.5 * (delta_droit + delta_gauche);
    vitesse_rob_roue_droite = delta_droit / Te;
    vitesse_rob_roue_gauche = delta_gauche / Te;

    /*
        theta_robot = ((delta_droit - delta_gauche) * 0.5) / ENTRAXE;

        // Mise à jour des coordonnées x et y
        odo_x += cos(theta_robot_prec + theta_robot / 2.0) * distance_parcourue;
        odo_y += sin(theta_robot_prec + theta_robot / 2.0) * distance_parcourue;

        theta_robot_prec += theta_robot;

        if (theta_robot_prec > PI)
        {
            theta_robot_prec = theta_robot_prec - 2 * PI;
        }
        if (theta_robot_prec < -PI)
        {
            theta_robot_prec = theta_robot_prec + 2 * PI;
        }
    */
    /*
  theta_robot_prec est modifié que lorsque l'on tourne
lorsqu'on est en ligne droite il est égal à 0 car delta gauche = delta droite
il nous permet donc de savoir dans quel sens on tourne
theta c'est la somme des angles qu'on a parcourue depuis le début
et lorsqu'on a fait un tour complet on le remet à 0
pour calculer ordo_x on ajoute à notre angle theta total notre nouvel angle afin de savoir dans quel sens on tourne et donc de ne pas modifier la valeur de x lorsque l'on tourne
pareil pour ordo_y

    */
    theta_robot_prec = ((-delta_droit + delta_gauche) * 0.5) / ENTRAXE;
    theta_robot += theta_robot_prec;
    // Mise à jour des coordonnées x et y
    odo_y += cos(theta_robot) * distance_parcourue;
    odo_x += sin(theta_robot) * distance_parcourue;

    vitesse_rob = distance_parcourue / Te;

    /*
        if (theta_robot > (2 * PI))
        {
            theta_robot = theta_robot * 0.0;
        }
        if (theta_robot < -(2 * PI))
        {   
            theta_robot = theta_robot * 0.0;
        }*/
    // Serial.printf("dparcourue %4.2f distdroit %4.2f  dist gauche %4.2f ", distance_parcourue, odo_dist_droit, odo_dist_gauche);
    // Serial.printf(" x %4.2f mm y %4.2f mm theta %4.2f deg\n", odo_x, odo_y, theta_robot * 180 / 3.14);

    // Serial.printf("%4.2f %4.2f %4.2f %4.2f\n", odo_x, odo_y, theta_robot * 180 / 3.14, theta_robot_prec * 180 / 3.14);
}
