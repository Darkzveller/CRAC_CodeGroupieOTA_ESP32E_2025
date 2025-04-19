#include "Variable.h" // Remonte d'un niveau pour atteindre lib
#include "MOTEUR.h"
#include "ASSERVISSEMENT.h"
#include "OTA.h"
float facteur_ajustement_consigne = 0.5;
void asservissement_roue_folle_droite_tick(double consigne, double observation)
{
    bool sens = 0;
    double sortie_roue_folle;

    double resolution_calculer = (pow(2, resolution_pwm) - 1) * POURCENT_MAX_PWM;
    double erreur = consigne - observation;
    double proportionnel = erreur * coeff_P_roue_folle_tick_droite;

    double deriver = coeff_D_roue_folle_tick_droite * (erreur - erreur_prec_roue_folle_droite_tick) / Te;

    somme_integral_roue_folle_droite_tick += erreur * Te;
    if (somme_integral_roue_folle_droite_tick > integral_limit_roue_folle_tick)
    {
        somme_integral_roue_folle_droite_tick = integral_limit_roue_folle_tick;
    }
    else if (somme_integral_roue_folle_droite_tick < -integral_limit_roue_folle_tick)
    {
        somme_integral_roue_folle_droite_tick = -integral_limit_roue_folle_tick;
    }

    double integral = coeff_I_roue_folle_tick_droite * somme_integral_roue_folle_droite_tick;

    double commande = proportionnel + deriver + integral;

    erreur_prec_roue_folle_droite_tick = erreur;

    // Gestion des bornes de la commande
    if (commande > 0)
    {
        sens = false;
        if (commande > resolution_calculer)
        {
            sortie_roue_folle = resolution_calculer;
        }
        else
        {
            sortie_roue_folle = commande;
        }
    }
    else
    {
        sens = true;
        if (commande < -resolution_calculer)
        {

            sortie_roue_folle = resolution_calculer;
        }
        else
        {
            sortie_roue_folle = -commande;
        }
    }
    // if (etat_actuel_vit_roue_folle_droite != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE)
    // {
    moteur_droit(sortie_roue_folle, sens);
    // }
}

void asservissement_roue_folle_gauche_tick(double consigne, double observation)
{
    bool sens = 0;
    double sortie_roue_folle;

    double resolution_calculer = (pow(2, resolution_pwm) - 1) * POURCENT_MAX_PWM;
    double erreur = consigne - observation;

    double proportionnel = erreur * coeff_P_roue_folle_tick_gauche;

    double deriver = coeff_D_roue_folle_tick_gauche * (erreur - erreur_prec_roue_folle_gauche_tick) / Te;

    somme_integral_roue_folle_gauche_tick += erreur * Te;
    if (somme_integral_roue_folle_gauche_tick > integral_limit_roue_folle_tick)
    {
        somme_integral_roue_folle_gauche_tick = integral_limit_roue_folle_tick;
    }
    else if (somme_integral_roue_folle_gauche_tick < -integral_limit_roue_folle_tick)
    {
        somme_integral_roue_folle_gauche_tick = -integral_limit_roue_folle_tick;
    }
    // if ((erreur < 1) || (erreur > -1))
    // {

    //   somme_integral_roue_folle_gauche_tick = 0;
    // }

    double integral = coeff_I_roue_folle_tick_gauche * somme_integral_roue_folle_gauche_tick;

    double commande = proportionnel + deriver + integral;

    erreur_prec_roue_folle_gauche_tick = erreur;

    // Gestion des bornes de la commande
    if (commande > 0)
    {
        sens = false;
        if (commande > resolution_calculer)
        {
            sortie_roue_folle = resolution_calculer;
        }
        else
        {
            sortie_roue_folle = commande;
        }
    }
    else
    {
        sens = true;
        if (commande < -resolution_calculer)
        {

            sortie_roue_folle = resolution_calculer;
        }
        else
        {
            sortie_roue_folle = -commande;
        }
    }
    // if (etat_actuel_vit_roue_folle_gauche != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE)
    // {
    moteur_gauche(sortie_roue_folle, sens);
    // }
}

double regulation_vitesse_roue_folle_droite(float cons, float Vmax_consigne)
{
    if (type_ligne_droite)
    {
        cons = cons * facteur_ajustement_consigne;
    }
    float erreur_vit;
    float vit = Vmax_consigne;
    float accel = Amax;
    float decc = Dmax;

    float distance_restante;
    float Ta = vit / accel;
    float Td = vit / decc;
    float Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
    distance_accel_droite = 0.5 * Ta * Ta * accel;
    distance_decl_droite = 0.5 * Td * Td * decc;

    // erreur_vit = vit - (vitesse_rob * Vmax);
    erreur_vit = vitesse_rob - vitesse_rob_roue_droite;

    somme_erreur_vit_roue_folle_droite += erreur_vit * Te;

    if (somme_erreur_vit_roue_folle_droite > integral_limit)
    {
        somme_erreur_vit_roue_folle_droite = integral_limit;
    }
    else if (somme_erreur_vit_roue_folle_droite < -integral_limit)
    {
        somme_erreur_vit_roue_folle_droite = -integral_limit;
    }

    float derivee_erreur_vit = (erreur_vit - erreur_vit_precedente_roue_folle_droite) / Te;
    erreur_vit_precedente_roue_folle_droite = erreur_vit;
    float commande_vit = kp_vit * erreur_vit + ki_vit * somme_erreur_vit_roue_folle_droite + kd_vit * derivee_erreur_vit;
    switch (etat_actuel_vit_roue_folle_droite)
    {
    case ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE:
        if (start_asservissement_roue_droite)
        {
            etat_actuel_vit_roue_folle_droite = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE;
        }
        break;
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE:
        acc_actuel_droite = acc_actuel_droite + commande_vit * Te;

        if (acc_actuel_droite > accel)
        {
            acc_actuel_droite = accel;
        }
        else if (acc_actuel_droite < -accel)
        {
            acc_actuel_droite = -accel;
        }

        consigne_vit_droite = vitesse_rob_roue_droite + acc_actuel_droite * Te;

        if (consigne_vit_droite > Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }
        else if (consigne_vit_droite < Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }

        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;

        Ta_counter_droite++;
        if (Ta_counter_droite >= fabs(Ta))
        {
            etat_actuel_vit_roue_folle_droite = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE;
        }
        break;

    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE:
        consigne_vit_droite = vit;

        if (consigne_vit_droite > Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }
        else if (consigne_vit_droite < Vmax_consigne)
        {
            consigne_vit_droite = Vmax_consigne;
        }

        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;

        if (fabs(cons - odo_tick_droit) < fabs(distance_decl_droite))
        {
            etat_actuel_vit_roue_folle_droite = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE;
        }
        break;

    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE:

        consigne_vit_droite = vit - decc * Te;
        consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;

        // distance_restante = cons - odo_tick_droit;
        // consigne_vit_droite = vitesse_rob + acc_actuel_droite * Te;
        // consigne_dist_droite = odo_tick_droit + consigne_vit_droite * Te;

        // if (consigne_dist_droite > 0)
        // {
        //     // Diminue progressivement la vitesse consigne
        //     consigne_dist_droite -= decc * Te;
        //     // Active le freinage dans le sens avant
        //     freinage_moteur_droit(true, consigne_dist_droite);

        //     // Assure que la consigne de vitesse ne descend pas en dessous de zéro
        //     if (consigne_dist_droite <= 0)
        //     {
        //         consigne_dist_droite = 0;
        //     }
        // }
        // else if (consigne_dist_droite < 0)
        // {

        //     // Augmente progressivement la vitesse consigne vers zéro
        //     consigne_dist_droite += decc * Te;
        //     // Active le freinage dans le sens arrière
        //     freinage_moteur_droit(true, consigne_dist_droite);

        //     // Assure que la consigne de vitesse ne dépasse pas zéro
        //     if (consigne_dist_droite >= 0)
        //     {
        //         consigne_dist_droite = 0;
        //     }
        // }

        if (((cons - odo_tick_droit) < limit_reprise_asser) || ((cons - odo_tick_droit) > -limit_reprise_asser))
        {
            etat_actuel_vit_roue_folle_droite = ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE;
            stop_moteur_droit();
        }
        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE:
        if (type_ligne_droite)
        {
            consigne_dist_droite = cons * 1 / facteur_ajustement_consigne;
        }

        consigne_dist_droite = cons;
        start_asservissement_roue_droite = false;
        consigne_odo_droite_prec = odo_tick_droit;

        // if ((delta_droit < 100) || (delta_droit > 100))
        // {
        etat_actuel_vit_roue_folle_droite = ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE;
        // }
        break;

    case ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE:

        etat_actuel_vit_roue_folle_droite = ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE;

        break;
    }
    // Serial.printf(" consB %.0f ", cons);
    // Serial.printf(" consigne_dist_droite %.0f ", consigne_dist_droite);

    return consigne_dist_droite;
}

double regulation_vitesse_roue_folle_gauche(float cons, float Vmax_consigne)
{

    float erreur_vit;
    float vit = Vmax_consigne;
    float accel = Amax;
    float decc = Dmax;

    float distance_restante;
    float vitesse_cible;

    float Ta = vit / accel;
    float Td = vit / decc;
    float Tc = (2.0 * cons - accel * (Ta * Ta + Td * Td)) / (2 * vit);
    distance_accel_gauche = 0.5 * Ta * Ta * accel;
    distance_decl_gauche = 0.5 * Td * Td * decc;
    // Serial.printf("distnace decc gauche %f ", distance_decl_gauche);
    erreur_vit = vitesse_rob - vitesse_rob_roue_gauche;
    somme_erreur_vit_roue_folle_gauche += erreur_vit * Te;

    if (somme_erreur_vit_roue_folle_gauche > integral_limit)
    {
        somme_erreur_vit_roue_folle_gauche = integral_limit;
    }
    else if (somme_erreur_vit_roue_folle_gauche < -integral_limit)
    {
        somme_erreur_vit_roue_folle_gauche = -integral_limit;
    }

    float derivee_erreur_vit = (erreur_vit - erreur_vit_precedente_roue_folle_gauche) / Te;
    erreur_vit_precedente_roue_folle_gauche = erreur_vit;
    float commande_vit = kp_vit * erreur_vit + ki_vit * somme_erreur_vit_roue_folle_gauche + kd_vit * derivee_erreur_vit;
    switch (etat_actuel_vit_roue_folle_gauche)
    {
    case ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE:
        if (start_asservissement_roue_gauche)
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        break;
    case ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        acc_actuel_gauche = acc_actuel_gauche + commande_vit * Te;

        if (acc_actuel_gauche > accel)
        {
            acc_actuel_gauche = accel;
        }
        else if (acc_actuel_gauche < -accel)
        {
            acc_actuel_gauche = -accel;
        }

        consigne_vit_gauche = vitesse_rob_roue_gauche + acc_actuel_gauche * Te;

        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }
        else if (consigne_vit_gauche < Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;

        Ta_counter_gauche++;
        if (Ta_counter_gauche >= fabs(Ta))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        break;

    case ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE:
        consigne_vit_gauche = vit;

        if (consigne_vit_gauche > Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }
        else if (consigne_vit_gauche < Vmax_consigne)
        {
            consigne_vit_gauche = Vmax_consigne;
        }

        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;
        // Serial.printf(" Xconsigne_dist_gauche %.0f ", consigne_dist_gauche);

        // Serial.printf(" Xconsigne_vit_gauche %.0f ", consigne_vit_gauche);
        // Serial.printf(" consigne_odo_gauche_prec %.0f ", consigne_odo_gauche_prec);
        // Serial.printf(" delta %.0f ", odo_tick_gauche-consigne_odo_gauche_prec);

        if (fabs(cons - odo_tick_gauche) < fabs(distance_decl_gauche))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE;
        }
        break;

    case ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE:
        consigne_vit_gauche = vit - decc * Te;
        consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;

        /*
                distance_restante = cons - odo_tick_gauche;
                // acc_actuel_gauche = acc_actuel_gauche - commande_vit * Te;
                consigne_vit_gauche = vitesse_rob - acc_actuel_gauche * Te;
                consigne_dist_gauche = odo_tick_gauche + consigne_vit_gauche * Te;

                if (consigne_dist_gauche > 0)
                {

                    // Diminue progressivement la vitesse consigne
                    consigne_dist_gauche -= decc * Te;
                    // Active le freinage dans le sens avant
                    freinage_moteur_droit(true, consigne_dist_gauche);

                    // Assure que la consigne de vitesse ne descend pas en dessous de zéro
                    if (consigne_dist_gauche <= 0)
                    {
                        consigne_dist_gauche = 0;
                    }
                }
                else if (consigne_dist_gauche < 0)
                {

                    // Augmente progressivement la vitesse consigne vers zéro
                    consigne_dist_gauche += decc * Te;
                    // Active le freinage dans le sens arrière
                    freinage_moteur_droit(true, consigne_dist_gauche);

                    // Assure que la consigne de vitesse ne dépasse pas zéro
                    if (consigne_dist_gauche >= 0)
                    {
                        consigne_dist_gauche = 0;
                    }
                }
        */
        if (((cons - odo_tick_gauche) < limit_reprise_asser) || ((cons - odo_tick_gauche) > -limit_reprise_asser))
        {
            etat_actuel_vit_roue_folle_gauche = ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE;
            stop_moteur_gauche();
        }
        break;

    case ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE:
        if (type_ligne_droite)
        {
            consigne_dist_gauche = cons * 1 / facteur_ajustement_consigne;
        }

        consigne_dist_gauche = cons;
        start_asservissement_roue_gauche = false;
        consigne_odo_gauche_prec = odo_tick_gauche;

        // if ((delta_gauche < 100) || (delta_gauche > 100))
        // {
        etat_actuel_vit_roue_folle_gauche = ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE;
        // }

        break;

    case ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE:

        etat_actuel_vit_roue_folle_gauche = ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE;

        break;
    }
    // Serial.printf(" consB %.0f ", cons);
    return consigne_dist_gauche;
}

bool return_flag_asser_roue()
{

    if ((start_asservissement_roue_droite == false) && (start_asservissement_roue_gauche == false))
    {

        return true;
    }
    else
    {
        return false;
    }
}

void lauch_flag_asser_roue(bool mode)
{
    start_asservissement_roue_droite = mode;
    start_asservissement_roue_gauche = mode;
}

double asservissement_angle_correction(double consigne_angle, double observation_angle)
{

    static double somme_integral_angle = 0.0;
    double erreur = consigne_angle - observation_angle;

    double proportionnel = coeff_P_angle * erreur;

    // Terme dérivé
    double deriver = coeff_D_angle * (erreur - erreur_prec_angle) / Te;

    // Terme intégral avec anti-windup
    somme_integral_angle += erreur * Te;
    if (somme_integral_angle > integral_limit_angle)
    {
        somme_integral_angle = integral_limit_angle;
    }
    else if (somme_integral_angle < -integral_limit_angle)
    {
        somme_integral_angle = -integral_limit_angle;
    }
    if (fabs(erreur) < 5)
    {
        somme_integral_angle = 0;
    }
    double integral = coeff_I_angle * somme_integral_angle;

    // Calcul de la correction
    double correction = proportionnel + deriver + integral;

    // Mise à jour de l'erreur précédente
    erreur_prec_angle = erreur;

    return correction; // Retourne la correction à appliquer
}
