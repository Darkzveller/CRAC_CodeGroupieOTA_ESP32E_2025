#include "Variable.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "MOTEUR.h"
#include <mat.h>
#include "USE_FUNCTION.h"
void rotation(int consigne, int vitesse)
{
    if (consigne >= 0)
    {
        sens = 1;
    }
    else
    {
        sens = -1;
    }
    float vitesse_croisiere_gauche = vitesse * -sens;
    float vitesse_croisiere_droit = vitesse * sens;

    int consigne_gauche = consigne * -1.0;
    int consigne_droite = consigne * 1.0;
    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);

    consigne_position_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_position_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    if ((etat_actuel_vit_roue_folle_gauche != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE) && (etat_actuel_vit_roue_folle_droite != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE))
    {
        float ecart = consigne_odo_gauche_prec + consigne_odo_droite_prec;
        consigne_position_gauche = -consigne_position_droite + ecart;
    }
    else
    {
        consigne_position_gauche = consigne_position_gauche;
    }
    // Serial.printf("consigne_position_droite %.0f",consigne_position_droite);
    // Serial.printf("consigne_position_gauche %.0f",consigne_position_gauche);

    // // // On force les consignes à être égales et opposées
    // consigne_position_droite = sens * consigne_regulation_moyenne;
    // consigne_position_gauche = -sens * consigne_regulation_moyenne;
}

void ligne_droite(int consigne, int vitesse)
{
    if (consigne >= 0)
    {
        sens = 1;
    }
    else if (consigne < 0)
    {
        sens = -1;
    }
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * sens;

    int consigne_gauche = consigne;
    int consigne_droite = consigne;

    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);

    // Calcul initial des consignes de vitesse pour chaque roue
    consigne_position_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_position_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    // Correction d'angle basée sur l'écart entre les consignes gauche et droite
    correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));

    // // Appliquer la correction à la consigne de vitesse
    consigne_position_droite -= correction;
    consigne_position_gauche += correction;
    // Serial.printf("consigne_position_droite %.0f",consigne_position_droite);
    // Serial.printf("consigne_position_gauche %.0f",consigne_position_gauche);
}

void asser_polaire_tick(float coordonnee_x, float coordonnee_y, float theta_cons, bool nbr_passage)
{
    static int sens = 1;
    static int compteur = 0;

    // Calcul de la distance à parcourir
    erreur_distance = convert_distance_mm_to_tick(sqrt(pow(coordonnee_x - odo_x, 2) + pow(coordonnee_y - odo_y, 2)));

    // Calcul de l'orientation
    erreur_orient = atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot;
    erreur_orient = normaliser_angle_rad(erreur_orient);

    // Serial.printf(" er_o_before %.3f ", convert_tick_to_angle_deg(convert_angle_radian_to_tick(erreur_orient)));

    // Déterminer si on doit inverser le sens au premier passage
    if (compteur == 0)
    {
        float erreur_orient_deg = degrees(erreur_orient); // car convert_tick_to_angle_deg demande un tick normalement
        if ((erreur_orient_deg > 90.0f) || (erreur_orient_deg < -90.0f))
        {
            sens = -1;
        }
        compteur = 1;
    }

    // Si sens inversé, corriger l'orientation
    if (sens == -1)
    {
        // Serial.printf("SENS ");
        erreur_orient += radians(180.0);
        erreur_orient = normaliser_angle_rad(erreur_orient); // Très important après ajout de 180°
    }

    // Maintenant on convertit erreur_orient en tick
    erreur_orient = convert_angle_radian_to_tick(erreur_orient);
    erreur_orient = constrain(erreur_orient, -1250, 1250);

    consigne_rot_polaire_tick = erreur_orient;
    // Vérification de fin de mouvement
    if (convert_distance_tick_to_mm(erreur_distance) <= 7.5)
    {
        // Serial.printf(" Vrai ");
        consigne_odo_gauche_prec = odo_tick_gauche;
        consigne_odo_droite_prec = odo_tick_droit;
        consigne_odo_x_prec = odo_x;
        consigne_odo_y_prec = odo_y;
        consigne_theta_prec = degrees(theta_robot);

        flag_fin_mvt = true;
        calcul_decl_polaire_tick = false;
        compteur = 0;
    }
    else if ((erreur_orient > convert_angle_deg_to_tick(20)) || (erreur_orient < convert_angle_deg_to_tick(-20))) // Gestion de la consigne de déplacement

    {
        // Serial.printf(" Vrai 2");
        consigne_dist_polaire_tick = 0;
    }
    else
    {
        // Serial.printf(" Vrai 3");
        consigne_dist_polaire_tick = SPEED_ULTRA;
    }

    // Inverser la consigne de distance si besoin
    if (sens == -1)
    {
        // Serial.printf("dist négatif ");
        consigne_dist_polaire_tick = -consigne_dist_polaire_tick;
    }

    // Commandes des moteurs
    consigne_position_gauche = odo_tick_gauche + consigne_dist_polaire_tick + coeff_rot_polaire_tick * consigne_rot_polaire_tick;
    consigne_position_droite = odo_tick_droit + consigne_dist_polaire_tick - coeff_rot_polaire_tick * consigne_rot_polaire_tick;

    // Debug Serial
    // Serial.printf(" cs_x %.1f ", coordonnee_x);
    // Serial.printf(" cs_y %.1f ", coordonnee_y);
    // Serial.printf(" Odo_x %.1f ", odo_x);
    // Serial.printf(" Odo_y %.1f ", odo_y);
    // Serial.printf(" theta %.3f ", degrees(theta_robot));
    // Serial.printf(" er_d %.3f ", convert_distance_tick_to_mm(erreur_distance));
    // Serial.printf(" er_o %.3f ", convert_tick_to_angle_deg(erreur_orient));
            // Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
        // Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);

    Serial.println();
}

bool recalage(uint8_t direction, uint8_t type_modif, uint16_t nouvelle_valeur, uint16_t consigne_rotation)
{
    bool flag_modif_fait = false;
    enum MouvementRecalage
    {
        IMMOBILE = 0,
        AVANT = 1,
        ARRIERE = 2
    };

    enum TypeModification
    {
        AUCUNE = 0,
        MODIF_X = 1,
        MODIF_Y = 2,
        MODIF_THETA = 3
    };
    int8_t sens = 0;

    // Machine à états pour le mouvement
    switch (direction)
    {
    case IMMOBILE:
        // Pas de mouvement , rien à faire ici
        break;
    case AVANT:
    case ARRIERE:

        sens = (direction == AVANT) ? 1 : -1; // condition ? valeur_si_vrai : valeur_si_faux;

        consigne_position_gauche = odo_tick_gauche + sens * SPEED_NORMAL;
        consigne_position_droite = odo_tick_droit + sens * SPEED_NORMAL;

        correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));
        consigne_position_droite -= correction;
        consigne_position_gauche += correction;
        break;
    }

    // Mise à jour de l’odométrie si contact
    if (toucher_objet_solid())
    {
        if (type_modif == MODIF_X)
        {
            // odo_x = convert_distance_mm_to_tick(nouvelle_valeur);
            odo_x = nouvelle_valeur;
            flag_modif_fait = true;
            Serial.printf("modif effec x");
            // delay( 2000);
        }
        else if (type_modif == MODIF_Y)
        {
            // odo_y = convert_distance_mm_to_tick(nouvelle_valeur);
            odo_y = nouvelle_valeur;
            flag_modif_fait = true;

            Serial.printf("modif effec x");
            // delay( 2000);
        }
    }
    if (type_modif == MODIF_THETA)
    {
        theta_robot = radians(convert_angle_deg_to_tick(nouvelle_valeur));
        consigne_theta_prec = convert_angle_deg_to_tick(nouvelle_valeur);
    }

    // Rotation demandée ?
    if (consigne_rotation != 0)
    {
        Serial.printf("Déclenchement rotation\n");
        liste.general_purpose = TYPE_DEPLACEMENT_ROTATION;
        liste.angle = convert_angle_deg_to_tick(consigne_rotation);
        liste.vitesse_croisiere = SPEED_NORMAL;
        lauch_flag_asser_roue(true);
    }

    // Serial.printf(" Odo x %.3f ", odo_x);
    // Serial.printf(" odo_y %.3f ", odo_y);
    // Serial.printf(" teheta %.3f ", degrees(theta_robot));
    // Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
    // Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);
    // Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
    // Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);
    // Serial.println();
    return flag_modif_fait;
}
bool toucher_objet_solid()
{

    static unsigned long last_time_droite = millis();
    static unsigned long last_time_gauche = millis();
    static int last_odo_droite = odo_tick_droit;
    static int last_odo_gauche = odo_tick_gauche;
    const unsigned long timeout = 500; // Temps avant de détecter un blocage (ms)

    static bool roue_droite_bloquer = false;
    static bool roue_gauche_bloquer = false;

    if (millis() - last_time_droite > timeout)
    {
        if (last_odo_droite == odo_tick_droit)
        {
            Serial.printf("R_D block");
            roue_droite_bloquer = true;
        }

        last_odo_droite = odo_tick_droit;
        last_time_droite = millis();
    }

    if (millis() - last_time_gauche > timeout)
    {
        if (last_odo_gauche == odo_tick_gauche)
        {
            Serial.printf("R_G block");
            roue_gauche_bloquer = true;
        }

        last_odo_gauche = odo_tick_gauche;
        last_time_gauche = millis();
    }

    if ((roue_gauche_bloquer == true) && (roue_droite_bloquer == true))
    {
        return true;
    }
    else
    {
        return false;
    }
}
