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
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * -sens;

    int consigne_gauche = consigne;
    int consigne_droite = consigne * -1.0;
    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);

    consigne_position_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_position_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    if ((etat_actuel_vit_roue_folle_gauche != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE) || (etat_actuel_vit_roue_folle_droite != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE))
    {
        float ecart = consigne_odo_gauche_prec + consigne_odo_droite_prec;
        consigne_position_gauche = -consigne_position_droite + ecart;
    }
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
}

// void x_y_theta(float coordonnee_x, float coordonnee_y, float theta_fin, int vitesse)
// {

//     float hypothenuse = sqrt(pow(coordonnee_x, 2) + pow(coordonnee_y, 2));
//     // TOA : Tangente = Opposé / Adjacent.
//     theta_premiere_rotation = degrees(atan2(coordonnee_y, coordonnee_x));

//     theta_deuxieme_rotation = theta_fin - theta_premiere_rotation;
//     int time = 100;
//     switch (etat_x_y_theta)
//     {
//     case -1:
//         break;
//     case 0:

//         rotation(convert_angle_deg_to_tick(fabs(theta_premiere_rotation)), vitesse);

//         if (return_flag_asser_roue())
//         {

//             stop_motors();
//             delay(time);
//             etat_x_y_theta = 1;
//             lauch_flag_asser_roue(true);
//             consigne_theta_prec = degrees(theta_robot);
//         }
//         break;

//     case 1:

//         ligne_droite((convert_distance_mm_to_tick(hypothenuse)), vitesse);

//         if (return_flag_asser_roue())
//         {
//             stop_motors();
//             delay(time);
//             etat_x_y_theta = 2;
//             lauch_flag_asser_roue(true);
//         }
//         break;
//     case 2:

//         rotation((convert_angle_deg_to_tick(theta_deuxieme_rotation)), vitesse);
//         if (return_flag_asser_roue())
//         {
//             stop_motors();
//             delay(time);
//             consigne_theta_prec = degrees(theta_robot);

//             etat_x_y_theta = -1;
//         }
//         break;

//     default:
//         break;
//     }
// }

void asser_polaire_tick(float coordonnee_x, float coordonnee_y, float theta_cons, bool nbr_passage)
{
    // coordonnee_x = 200;
    // coordonnee_y = 0;
    erreur_distance = convert_distance_mm_to_tick(sqrt(pow(coordonnee_x - odo_x, 2) + pow(coordonnee_y - odo_y, 2))); // On détermine la distance restante a parcourir
    // erreur_orient = convert_angle_radian_to_tick((atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot));  // On détermine l'angle a parcour pour arriver a destination
    erreur_orient = atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot; // On détermine l'angle a parcour pour arriver a destination
    erreur_orient = normaliser_angle_rad(erreur_orient);
    erreur_orient = convert_angle_radian_to_tick(erreur_orient);
    erreur_orient = constrain(erreur_orient, -1250, 1250);
    consigne_rot_polaire_tick = erreur_orient;

    if ((erreur_distance <= distance_decl_polaire_tick) && (nbr_passage == true))
    {
        float facteur_deccel = erreur_distance / distance_decl_polaire_tick;
        consigne_dist_polaire_tick = consigne_dist_polaire_tick_max * facteur_deccel;
        if (convert_distance_tick_to_mm(erreur_distance) <= 10.0)
        {
            Serial.printf(" Vrai ");
            consigne_odo_gauche_prec = odo_tick_gauche;
            consigne_odo_droite_prec = odo_tick_droit;
            consigne_odo_x_prec = odo_x;
            consigne_odo_y_prec = odo_y;
            consigne_theta_prec = degrees(theta_robot);
            flag_fin_mvt = true;
            calcul_decl_polaire_tick = false;
        }
        Serial.printf(" Vrai 4");
    }
    else if ((erreur_orient > convert_angle_deg_to_tick(20)) || (erreur_orient < convert_angle_deg_to_tick(-20)))
    {
        Serial.printf(" Vrai 2");
        // coeff_rot_polaire_tick = 0.5;
        consigne_dist_polaire_tick = 0;
    }
    else
    {
        Serial.printf(" Vrai 3");
        // coeff_rot_polaire_tick = 0.1;
        consigne_dist_polaire_tick = consigne_dist_polaire_tick_max;
    }

    consigne_position_gauche = odo_tick_gauche + coeff_dist_polaire_tick * consigne_dist_polaire_tick + coeff_rot_polaire_tick * consigne_rot_polaire_tick; // commande en tick qu'on souhaite atteindre
    consigne_position_droite = odo_tick_droit + coeff_dist_polaire_tick * consigne_dist_polaire_tick - coeff_rot_polaire_tick * consigne_rot_polaire_tick;  // commande en tick qu'on souhaite atteindre

    // asservissement_roue_folle_gauche_tick(round(commande_gauche), odo_tick_gauche); // PID en tick des roue avec pour 1ere argument la consigne et le deuxieme argument l'observation sur la roue odo
    // asservissement_roue_folle_droite_tick(round(commande_droite), odo_tick_droit);  // PID en tick des roue avec pour 1ere argument la consigne et le deuxieme argument l'observation sur la roue odo

    if (calcul_decl_polaire_tick == false)
    {
        calcul_decl_polaire_tick = true;
        distance_decl_polaire_tick = 0.5 * pow(2, consigne_dist_polaire_tick_max / coeff_decc_distance_polaire_tick) * coeff_decc_distance_polaire_tick;
    }

    Serial.printf(" cs x %.1f ", coordonnee_x);
    Serial.printf(" cs_y %.1f ", coordonnee_y);

    Serial.printf(" Odo x %.1f ", odo_x);
    Serial.printf(" odo_y %.1f ", odo_y);
    Serial.printf(" teheta %.3f ", degrees(theta_robot));

    Serial.printf(" er_d %.3f ", convert_distance_tick_to_mm(erreur_distance));
    Serial.printf(" er_o %.3f ", convert_tick_to_angle_deg(erreur_orient));

    Serial.printf(" cmd_d %.1f ", consigne_dist_polaire_tick);
    Serial.printf(" cmd_r %.1f ", consigne_rot_polaire_tick);
    Serial.printf(" cff_r %.1f ", coeff_rot_polaire_tick);
    Serial.printf(" cff_d %.1f ", coeff_dist_polaire_tick);

    // Serial.printf(" dist_dcl %.1f ", convert_distance_tick_to_mm(distance_decl_polaire_tick));
    // Serial.printf(" coef_decl %.1f ", coeff_decc_distance_polaire_tick);

    Serial.printf(" odo_g %.0f ", odo_tick_gauche);
    Serial.printf(" odo_d %.0f ", odo_tick_droit);

    // Serial.printf(" angl_tick %.1f ", (float)convert_angle_deg_to_tick(90));
    // Serial.printf(" angl_deg %.1f ", convert_tick_to_angle_deg(convert_angle_deg_to_tick(90)));

    Serial.println();
}

// void recalage()
// {
//     // static int g = 40;
//     // static int d = 40;

//     consigne_position_droite = odo_tick_droit + SPEED_TORTUE;
//     consigne_position_gauche = odo_tick_gauche + SPEED_TORTUE;

//     if (consigne_position_droite + odo_tick_droit == SPEED_TORTUE)
//     {
//         Serial.printf(" ne bouge pas droite ");
//     }

//     if (consigne_position_gauche + odo_tick_gauche == SPEED_TORTUE)
//     {
//         Serial.printf(" ne bouge pas gauche ");
//     }
//     // double consigne_angle = consigne_theta_prec;     // Consigne d'angle cible (par exemple, maintenir 0°)
//     // double observation_angle = degrees(theta_robot); // Angle actuel du robot
//     // double correction = asservissement_angle_correction(consigne_angle, observation_angle);

//     // // // Appliquer la correction à la consigne de vitesse
//     // consigne_position_droite -= correction;
//     // consigne_position_gauche += correction;
//     Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
//     Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);

//     Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
//     Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);

//     Serial.printf(" odo_tick_droit_last %.0f ", odo_tick_droit_last);
//     Serial.printf(" odo_tick_gauche_last %.0f ", odo_tick_gauche_last);

//     Serial.printf(" calcl_droite %.0f ", consigne_position_droite + odo_tick_droit);
//     Serial.printf(" calcl_droite %.0f ", consigne_position_droite - odo_tick_droit);
//     Serial.println();

// }
/*
void recalage()
{
    static unsigned long last_time = millis();
    static int last_odo_droite = odo_tick_droit;
    static int last_odo_gauche = odo_tick_gauche;
    static bool roue_bloquee_droite = false;
    static bool roue_bloquee_gauche = false;
    const unsigned long timeout = 500; // Temps avant de détecter un blocage (en ms)

    // Mise à jour des consignes
    consigne_position_droite = odo_tick_droit + SPEED_TORTUE;
    consigne_position_gauche = odo_tick_gauche + SPEED_TORTUE;

    // Vérification si les roues bougent
    if (millis() - last_time > timeout)
    {
        if (last_odo_droite == odo_tick_droit)
        {
            Serial.print("⚠️ Roue droite bloquée !");
            roue_bloquee_droite = true;
        }
        else
        {
            roue_bloquee_droite = false;
        }

        if (last_odo_gauche == odo_tick_gauche)
        {
            Serial.print("⚠️ Roue gauche bloquée !");
            roue_bloquee_gauche = true;
        }
        else
        {
            roue_bloquee_gauche = false;
        }

        // Mise à jour des dernières valeurs
        last_odo_droite = odo_tick_droit;
        last_odo_gauche = odo_tick_gauche;
        last_time = millis();
    }

    // Si une des roues est bloquée, on stoppe le moteur
    if (roue_bloquee_droite || roue_bloquee_gauche)
    {
        Serial.print("🛑 Arrêt du moteur !");
        consigne_position_droite = odo_tick_droit;
        consigne_position_gauche = odo_tick_gauche;
    }

    // Affichage des valeurs pour le debug
    Serial.printf("Consigne Droite: %.0f | Consigne Gauche: %.0f", consigne_position_droite, consigne_position_gauche);
    Serial.printf("Odo Droite: %.0f | Odo Gauche: %.0f\n", odo_tick_droit, odo_tick_gauche);

    // Envoi des nouvelles consignes aux moteurs
    // asservissement_roue_folle_droite_tick(consigne_position_droite, odo_tick_droit);
    // asservissement_roue_folle_gauche_tick(consigne_position_gauche, odo_tick_gauche);
}
*/

void recalage()
{
    static unsigned long last_time_droite = millis();
    static unsigned long last_time_gauche = millis();
    static int last_odo_droite = odo_tick_droit;
    static int last_odo_gauche = odo_tick_gauche;
    const unsigned long timeout = 1250; // Temps avant de détecter un blocage (ms)

    static bool etat_recalage_droite = false;
    static bool etat_recalage_gauche = false;

    static bool init_done = false; // Flag pour exécuter le code une seule fois

    static unsigned long last_time_asser = millis();

    enum RecalageGlobal
    {
        LANCEMENT,
        Attente_toucher_1_mur,
        RECULER_MILIEU_CASE,
        ATTENTE_SECONDE_1,
        ROTATION_POUR_ATTEINDRE_DEUXIEME_MUR,ATTENTE_SECONDE_2,
        ATTENTE_TOUCHER_2_MUR
    };
    static RecalageGlobal etat_recalage_global = LANCEMENT;

    switch (etat_recalage_global)
    {
    case LANCEMENT:
        etat_recalage_droite = false;
        etat_recalage_gauche = false;
        etat_recalage_global = Attente_toucher_1_mur;
        break;

    case Attente_toucher_1_mur:
        consigne_position_gauche = odo_tick_gauche + SPEED_NORMAL;
        consigne_position_droite = odo_tick_droit + SPEED_NORMAL;

        correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));
        // // Appliquer la correction à la consigne de vitesse
        consigne_position_droite -= correction;
        consigne_position_gauche += correction;

        if (millis() - last_time_droite > timeout)
        {
            if (last_odo_droite == odo_tick_droit)
            {
                Serial.printf("⚠️ Roue DROITE bloquée !");
                etat_recalage_droite = true;
            }

            last_odo_droite = odo_tick_droit;
            last_time_droite = millis();
        }

        if (millis() - last_time_gauche > timeout)
        {
            if (last_odo_gauche == odo_tick_gauche)
            {
                Serial.printf("⚠️ Roue GAUCHE bloquée !");
                etat_recalage_gauche = true;
            }

            last_odo_gauche = odo_tick_gauche;
            last_time_gauche = millis();
        }
        if ((etat_recalage_gauche == true) && (etat_recalage_droite == true))
        {
            etat_recalage_global = RECULER_MILIEU_CASE;
            etat_recalage_droite = false;
            etat_recalage_gauche = false;
            consigne_odo_droite_prec = odo_tick_droit;
            consigne_odo_gauche_prec = odo_tick_gauche;

            liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;

            correction = 0;
        }
        Serial.printf("Attente_toucher_1_mur ");

        break;

    case RECULER_MILIEU_CASE:

        if (!init_done) // Vérifie si le code a déjà été exécuté
        {
            liste.general_purpose = TYPE_DEPLACEMENT_LIGNE_DROITE;
            liste.distance = convert_distance_mm_to_tick(-200);
            liste.vitesse_croisiere = SPEED_TORTUE;

            lauch_flag_asser_roue(true);
            Serial.printf("\n OK \n");
            Serial.printf(" distance %f ", convert_distance_tick_to_mm(liste.distance));
            Serial.printf(" liste.distance %d ", liste.distance);
            Serial.println();
            theta_robot = 0;
            init_done = true; // Marque l'initialisation comme faite
        }

        if (return_flag_asser_roue())
        {
            etat_recalage_global = ATTENTE_SECONDE_1;
        }
        Serial.printf("RECULER_MILIEU_CASE ");
        break;
    case ATTENTE_SECONDE_1:
        // stop_motors();
        // delay(2000);
        if (millis() - last_time_asser > timeout * 10)
        {
            last_time_asser = millis();
            etat_recalage_global = ROTATION_POUR_ATTEINDRE_DEUXIEME_MUR;
        }

        break;
    case ROTATION_POUR_ATTEINDRE_DEUXIEME_MUR:
        if (init_done) // Vérifie si le code a déjà été exécuté
        {
            liste.general_purpose = TYPE_DEPLACEMENT_ROTATION;
            liste.angle = convert_angle_deg_to_tick(90);
            liste.vitesse_croisiere = SPEED_TORTUE;

            lauch_flag_asser_roue(true);
            Serial.printf("\n OK \n");
            Serial.printf(" angle %f ", convert_tick_to_angle_deg(liste.angle));
            Serial.printf(" liste.angle %d ", liste.distance);
            Serial.println();

            init_done = false; // Marque l'initialisation comme faite
        }

        if (return_flag_asser_roue())
        {
            etat_recalage_global = ATTENTE_SECONDE_2;
        }

        Serial.printf("ROTATION D MUR ");

        break;
        case ATTENTE_SECONDE_2:
        stop_motors();
        delay(2000);
        // if (millis() - last_time_asser > timeout )
        // {
            // last_time_asser = millis();
            etat_recalage_global = ATTENTE_TOUCHER_2_MUR;
        // }

    case ATTENTE_TOUCHER_2_MUR:
        consigne_position_gauche = odo_tick_gauche + SPEED_TORTUE;
        consigne_position_droite = odo_tick_droit + SPEED_TORTUE;

        correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));
        // // Appliquer la correction à la consigne de vitesse
        consigne_position_droite -= correction;
        consigne_position_gauche += correction;

        if (millis() - last_time_droite > timeout)
        {
            if (last_odo_droite == odo_tick_droit)
            {
                Serial.printf("⚠️ Roue DROITE bloquée !");
                etat_recalage_droite = true;
            }

            last_odo_droite = odo_tick_droit;
            last_time_droite = millis();
        }

        if (millis() - last_time_gauche > timeout)
        {
            if (last_odo_gauche == odo_tick_gauche)
            {
                Serial.printf("⚠️ Roue GAUCHE bloquée !");
                etat_recalage_gauche = true;
            }

            last_odo_gauche = odo_tick_gauche;
            last_time_gauche = millis();
        }
        if ((etat_recalage_gauche == true) && (etat_recalage_droite == true))
        {
            // etat_recalage_global = RECULER_MILIEU_CASE;
            etat_recalage_droite = false;
            etat_recalage_gauche = false;
            consigne_odo_droite_prec = odo_tick_droit;
            consigne_odo_gauche_prec = odo_tick_gauche;

            liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;

            correction = 0;
        }

        Serial.printf("ATTENTE_TOUCHER_2_MUR ");

        break;

    default:
        break;
    }
    // Serial.printf("Consigne DROITE: %.0f | Odo DROITE: %.0f\n", etat_droite, consigne_position_droite, odo_tick_droit);
    // Serial.printf("Etat GAUCHE: %d | Consigne GAUCHE: %.0f | Odo GAUCHE: %.0f\n", etat_gauche, consigne_position_gauche, odo_tick_gauche);

    Serial.printf(" corr_a %.0f ", correction);

    Serial.printf(" cs_d %.0f ", consigne_position_droite);
    Serial.printf(" cs_g %.0f ", consigne_position_gauche);

    Serial.printf(" odo_g %.0f ", odo_tick_gauche);
    Serial.printf(" odo_d %.0f ", odo_tick_droit);

    // Affichage des valeurs pour debug
    // Serial.printf(" Etat DROITE: %d | Consigne DROITE: %.0f | Odo DROITE: %.0f ", etat_recalage_droite, consigne_position_droite, odo_tick_droit);
    // Serial.printf(" Etat GAUCHE: %d | Consigne GAUCHE: %.0f | Odo GAUCHE: %.0f\n", etat_recalage_gauche, consigne_position_gauche, odo_tick_gauche);

    Serial.println();
    // Envoi des nouvelles consignes aux moteurs
    // asservissement_roue_folle_droite_tick(consigne_position_droite, odo_tick_droit);
    // asservissement_roue_folle_gauche_tick(consigne_position_gauche, odo_tick_gauche);
}
