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
    erreur_distance = convert_distance_mm_to_tick(sqrt(pow(coordonnee_x - odo_x, 2) + pow(coordonnee_y - odo_y, 2))); // On détermine la distance restante a parcourir
    erreur_orient = atan2(coordonnee_y - odo_y, coordonnee_x - odo_x) - theta_robot;                                  // On détermine l'angle a parcour pour arriver a destination
    erreur_orient = normaliser_angle_rad(erreur_orient);
    erreur_orient = convert_angle_radian_to_tick(erreur_orient);
    erreur_orient = constrain(erreur_orient, -1500, 1500);
    consigne_rot_polaire_tick = erreur_orient;

    if (convert_distance_tick_to_mm(erreur_distance) <= 10.0)
    {
        // Serial.printf(" Vrai ");
        consigne_odo_gauche_prec = odo_tick_gauche;
        consigne_odo_droite_prec = odo_tick_droit;
        consigne_odo_x_prec = odo_x;
        consigne_odo_y_prec = odo_y;
        consigne_theta_prec = degrees(theta_robot);
        flag_fin_mvt = true;
        calcul_decl_polaire_tick = false;
    }
    consigne_position_gauche = odo_tick_gauche + SPEED_ULTRA + coeff_rot_polaire_tick * consigne_rot_polaire_tick; // commande en tick qu'on souhaite atteindre
    consigne_position_droite = odo_tick_droit + SPEED_ULTRA - coeff_rot_polaire_tick * consigne_rot_polaire_tick;  // commande en tick qu'on souhaite atteindre

    // Serial.printf(" cs x %.1f ", coordonnee_x);
    // Serial.printf(" cs_y %.1f ", coordonnee_y);

    // Serial.printf(" Odo x %.1f ", odo_x);
    // Serial.printf(" odo_y %.1f ", odo_y);
    // Serial.printf(" teheta %.3f ", degrees(theta_robot));

    // Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
    // Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);

    // Serial.printf(" er_d %.3f ", convert_distance_tick_to_mm(erreur_distance));
    // Serial.printf(" er_o %.3f ", convert_tick_to_angle_deg(erreur_orient));

    // Serial.printf(" cmd_d %.1f ", consigne_dist_polaire_tick);
    // Serial.printf(" cmd_r %.1f ", consigne_rot_polaire_tick);
    // Serial.printf(" cff_r %.1f ", coeff_rot_polaire_tick);
    // Serial.printf(" cff_d %.1f ", coeff_dist_polaire_tick);

    // Serial.printf(" dist_dcl %.1f ", convert_distance_tick_to_mm(distance_decl_polaire_tick));
    // Serial.printf(" coef_decl %.1f ", coeff_decc_distance_polaire_tick);

    // Serial.printf(" odo_g %.0f ", odo_tick_gauche);
    // Serial.printf(" odo_d %.0f ", odo_tick_droit);

    // Serial.println();
}
/*
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
        ROTATION_POUR_ATTEINDRE_DEUXIEME_MUR,
        ATTENTE_SECONDE_2,
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
*/
/*
void recalage(uint8_t marche_avant_arriere_ou_sans_bouger, uint8_t modification_x_ou_y_ou_attrape_objet_terrain, uint16_t nouvelle_valeur_x_ou_y, uint16_t consigne_rotation_recalage)
{

    static unsigned long last_time_asser = millis();

    if (marche_avant_arriere_ou_sans_bouger == 0)
    { // Cela signifie que je mets a jour mes données sans bouger
    }
    else if (marche_avant_arriere_ou_sans_bouger == 1)
    { // Cela signifie je vais faire bouger le robot faire l'avant pour mettre a jour ma donnée
        consigne_position_gauche = odo_tick_gauche + SPEED_NORMAL;
        consigne_position_droite = odo_tick_droit + SPEED_NORMAL;

        correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));
        // // Appliquer la correction à la consigne de vitesse
        consigne_position_droite -= correction;
        consigne_position_gauche += correction;
    }
    else if (marche_avant_arriere_ou_sans_bouger == 2)
    { // Cela signifie je vais faire bouger le robot faire l'arriere pour mettre a jour ma donnée
        consigne_position_gauche = odo_tick_gauche - SPEED_NORMAL;
        consigne_position_droite = odo_tick_droit - SPEED_NORMAL;

        correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));
        // // Appliquer la correction à la consigne de vitesse
        consigne_position_droite -= correction;
        consigne_position_gauche += correction;
    }

    if (modification_x_ou_y_ou_attrape_objet_terrain == 0)
    { // On est dans le cas ou souhaiter forcer la rencontre avec les objets
    }
    else if (modification_x_ou_y_ou_attrape_objet_terrain == 1)
    { // modification de la valeur en x
        if (toucher_objet_solid())
        {
            odo_x = nouvelle_valeur_x_ou_y;
        }
    }
    else if (modification_x_ou_y_ou_attrape_objet_terrain == 2)
    { // modification de la valeur en y
        if (toucher_objet_solid())
        {
            odo_y = nouvelle_valeur_x_ou_y;
        }
    }

    if(consigne_rotation_recalage!=0)
    {
        Serial.printf("padajdlkaj");
        liste.general_purpose = TYPE_DEPLACEMENT_ROTATION;
        liste.angle = convert_angle_deg_to_tick(consigne_rotation_recalage);
        liste.vitesse_croisiere = SPEED_NORMAL;
        lauch_flag_asser_roue(true);

    }
    Serial.printf(" Odo x %.3f ", odo_x);
    Serial.printf(" odo_y %.3f ", odo_y);
    Serial.printf(" teheta %.3f ", degrees(theta_robot));
    Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
    Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);
    Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
    Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);
}
*/

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



