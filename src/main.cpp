#include <mat.h>
#include <Arduino.h>
#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "ID_UART.h"
#include "UART1.h"
#include "USE_FUNCTION.h"
#include "I2C_ESP32E.h"

void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        read_x_y_theta();

        if (detect_obstacle)
        {

            switch (liste.general_purpose)
            {
            case TYPE_DEPLACEMENT_LIGNE_DROITE:
                // Serial.printf("TYPE_DEPLACEMENT_LIGNE_DROITE ");

                ligne_droite(liste.distance, liste.vitesse_croisiere);
                // Serial.println();

                if (return_flag_asser_roue())
                {
                    send_message_bw16(ACKNOWLEDGE_BASE_ROULANTE, TYPE_DEPLACEMENT_LIGNE_DROITE, 0, 0, 0, 0, 0, 0, 0);
                    liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
                }

                break;
            case TYPE_DEPLACEMENT_ROTATION:

                // Serial.printf("TYPE_DEPLACEMENT_ROTATION ");

                rotation(liste.angle, liste.vitesse_croisiere);
                // Serial.println();

                if (return_flag_asser_roue())
                {
                    consigne_theta_prec = degrees(theta_robot);
                    send_message_bw16(ACKNOWLEDGE_BASE_ROULANTE, TYPE_DEPLACEMENT_ROTATION, 0, 0, 0, 0, 0, 0, 0);
                    liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
                }
                break;
            case TYPE_DEPLACEMENT_IMMOBILE:
                consigne_position_droite = consigne_odo_droite_prec;
                consigne_position_gauche = consigne_odo_gauche_prec;
                // Serial.printf(" TYPE_DEPLACEMENT_IMMOBILE");
                liste.general_purpose = TYPE_VIDE;
                send_message_bw16(ACKNOWLEDGE_BASE_ROULANTE, TYPE_DEPLACEMENT_IMMOBILE, 0, 0, 0, 0, 0, 0, 0);

                break;
            case TYPE_DEPLACEMENT_X_Y_POLAIRE:
                // Serial.printf(" TYPE_DEPLACEMENT_X_Y_POLAIRE ");
                asser_polaire_tick(liste.x_polaire, liste.y_polaire, 0, liste.nbr_passage = true);

                if (flag_fin_mvt)
                {
                    send_message_bw16(ACKNOWLEDGE_BASE_ROULANTE, TYPE_DEPLACEMENT_X_Y_POLAIRE, 0, 0, 0, 0, 0, 0, 0);
                    liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
                }
                break;
            case TYPE_DEPLACEMENT_RECALAGE:
                // Serial.printf(" TYPE_DEPLACEMENT_RECALAGE ");

                if (recalage(liste.direction_recalage, liste.type_modif_x_y_theta_recalge_rien, liste.nouvelle_valeur_x_y_theta_rien, liste.consigne_rotation_recalge))
                {
                    consigne_odo_droite_prec = odo_tick_droit;
                    consigne_odo_gauche_prec = odo_tick_gauche;
                    send_message_bw16(ACKNOWLEDGE_BASE_ROULANTE, TYPE_DEPLACEMENT_RECALAGE, 0, 0, 0, 0, 0, 0, 0);
                    liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
                    // Serial.printf(" Odo x %.3f ", odo_x);
                    // Serial.printf(" odo_y %.3f ", odo_y);
                    // Serial.printf(" teheta %.3f ", degrees(theta_robot));
                    // Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
                    // Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);
                    // Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
                    // Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);
                    // Serial.println();
                }
                break;

            case TYPE_VIDE:
                // Serial.printf(" TYPE_VIDE ");
                // Serial.printf(" Odo x %.3f ", odo_x);
                // Serial.printf(" odo_y %.3f ", odo_y);
                // Serial.printf(" teheta %.3f ", degrees(theta_robot));
                // Serial.println();
                break;

            default:
                break;
            }
        }
        if (!stop_start_match_star)
        {
            asservissement_roue_folle_droite_tick(consigne_position_droite, odo_tick_droit);
            asservissement_roue_folle_gauche_tick(consigne_position_gauche, odo_tick_gauche);
        }
        flag_controle = 1;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}

void comm_avec_bw16(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        read_message_bw16();
        switch (rxMsg.id)
        {

        case ESP32_RESTART:
            Serial.println("ESP32_RESTART");
            liste.general_purpose = TYPE_VIDE;
            stop_motors();
            esp_restart();

            break;

        case ROTATION:
            // Pour se simplifier j'ai préférer décomposer les etapes quitte a prendre un peu plus de temps cpu

            liste.general_purpose = TYPE_DEPLACEMENT_ROTATION;

            // liste.angle = TIC_PER_TOUR * angle / 80.0;

            liste.angle = convert_angle_deg_to_tick(fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            liste.vitesse_croisiere = rxMsg.data[2];

            lauch_flag_asser_roue(true);
            rxMsg.id = 0;
            Serial.printf("ROTATION ");
            Serial.printf(" angle %f ", (float)fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            Serial.printf(" liste.angle %f", (float)liste.angle);
            Serial.printf(" liste.vitesse_croisiere %d ", liste.vitesse_croisiere);
            Serial.println();
            break;

        case LIGNE_DROITE:

            liste.general_purpose = TYPE_DEPLACEMENT_LIGNE_DROITE;
            liste.distance = convert_distance_mm_to_tick(fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            liste.vitesse_croisiere = rxMsg.data[2];
            lauch_flag_asser_roue(true);
            rxMsg.id = 0;

            Serial.printf("LIGNE_DROITE ");
            Serial.printf(" rxMsg.data[0] %d ", rxMsg.data[0]);
            Serial.printf(" rxMsg.data[1] %d ", rxMsg.data[1]);

            Serial.printf(" distance %d ", fusion_octet(rxMsg.data[0], rxMsg.data[1]));
            Serial.printf(" liste.distance %d ", liste.distance);
            Serial.printf(" liste.vitesse_croisiere %d ", liste.vitesse_croisiere);
            Serial.println();

            break;

        case POLAIRE:

            liste.general_purpose = TYPE_DEPLACEMENT_X_Y_POLAIRE;
            liste.x_polaire = fusion_octet(rxMsg.data[0], rxMsg.data[1]);
            liste.y_polaire = fusion_octet(rxMsg.data[2], rxMsg.data[3]);
            liste.nbr_passage = rxMsg.data[4];
            flag_fin_mvt = false;

            rxMsg.id = 0;

            Serial.printf(" POLAIRE ");
            Serial.printf(" liste.x_polaire %f ", liste.x_polaire);
            Serial.printf(" liste.y_polaire %f ", liste.y_polaire);

            Serial.println();

            break;
        case RECALAGE:

            liste.general_purpose = TYPE_DEPLACEMENT_RECALAGE;
            liste.direction_recalage = rxMsg.data[0];
            liste.type_modif_x_y_theta_recalge_rien = rxMsg.data[1];
            liste.nouvelle_valeur_x_y_theta_rien = fusion_octet(rxMsg.data[2], rxMsg.data[3]);
            liste.consigne_rotation_recalge = convert_angle_deg_to_tick(fusion_octet(rxMsg.data[4], rxMsg.data[5]));

            rxMsg.id = 0;
            Serial.printf(" RECALAGE ");
            Serial.printf(" liste.direction_recalage %d ", liste.direction_recalage);
            Serial.printf(" liste.type_modif_x_y_theta_recalge_rien %d ", liste.type_modif_x_y_theta_recalge_rien);
            Serial.printf(" liste.nouvelle_valeur_x_y_theta_rien %d ", liste.nouvelle_valeur_x_y_theta_rien);
            Serial.printf(" liste.consigne_rotation_recalge %d ", liste.consigne_rotation_recalge);

            Serial.println();

            break;
        case STOP_ROBOT_FIN_MATCH:
            stop_start_match_star = true;
            break;
        case START_ROBOT_MATCH:
            stop_start_match_star = false;
            break;

        case 0:
            break;

        default:
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Tcan));
    }
}

void tache_i2c(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        read_tof();
        lcd.setCursor(0, 0);
        lcd.printf("x%3.0fy%3.0ft%3.0fS%1dD%1d", odo_x, odo_y, degrees(theta_robot), stop_start_match_star, detect_obstacle);

        lcd.setCursor(7, 1);
        lcd.printf("TG%3dD%3d", mesure_tof_save[0], mesure_tof_save[1]);
        start_stop_moteur_star(stop_start_match_star);
        // Serial.printf("stop_start_match_star %d", stop_start_match_star);
        // Serial.println();

        flag_controle = 1;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}

void setup()
{ // calcul coeff filtre
    // delay(10000);
    // Initialisation de la communication série à 115200 bauds
    Serial.begin(115200);
    // Serial.println("Booting with OTA"); // Message indiquant le démarrage avec OTA
    // Appel à la fonction de configuration OTA (non définie dans ce code, mais probablement ailleurs)
    // setupOTA();
    // Initialisation des moteurs
    setup_motors();
    stop_motors();
    // Initialisation des encodeurs
    setup_encodeur();
    // Initialisation de l'UART1
    setupUART1(1000E3);
    // Initialisation du MUTEX i2c
    init_mutex(false);
    // Initialisation des TOF
    init_tof();
    // Initialisation du LCD
    init_lcd_groove(false);

    Serial.println("on commence");

    // Serial.printf("avncement_gauche enter : %.0f\n", avncement_gauche);
    // Serial.printf("avncement_droite enter : %.0f\n", avncement_droite);

    reset_encodeur();
    delay(1500);
    reset_encodeur();

    xTaskCreate(
        controle,   // nom de la fonction
        "controle", // nom de la tache que nous venons de vréer
        10000,      // taille de la pile en octet
        NULL,       // parametre
        10,         // tres haut niveau de priorite
        NULL        // descripteur
    );
    xTaskCreate(
        tache_i2c,   // nom de la fonction
        "tache_i2c", // nom de la tache que nous venons de vréer
        10000,       // taille de la pile en octet
        NULL,        // parametre
        11,          // tres haut niveau de priorite
        NULL         // descripteur
    );
    xTaskCreate(
        comm_avec_bw16,   // nom de la fonction
        "comm_avec_bw16", // nom de la tache que nous venons de vréer
        10000,            // taille de la pile en octet
        NULL,             // parametre
        9,                // tres haut niveau de priorite
        NULL              // descripteur
    );
}

// Boucle principale, exécutée en continu après le setup
void loop()
{
    if (flag_controle)
    {
        // Serial.printf(" Odo x %.3f ", odo_x);
        // Serial.printf(" odo_y %.3f ", odo_y);
        // Serial.printf(" teheta %.3f ", degrees(theta_robot));
        // Serial.printf(" direction_recalage %d ", liste.direction_recalage);
        // Serial.printf(" type_modif_x_y_theta_recalge_rien %d ", liste.type_modif_x_y_theta_recalge_rien);
        // Serial.printf(" nouvelle_valeur_x_y_theta_rien %d ", liste.nouvelle_valeur_x_y_theta_rien);
        // Serial.printf(" consigne_rotation_recalge %d ", liste.consigne_rotation_recalge);

        // Serial.printf(" er_d %.3f ", convert_distance_tick_to_mm(erreur_distance));
        // Serial.printf(" er_o %.3f ", convert_tick_to_angle_deg(erreur_orient));

        // Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
        // Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);

        // Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
        // Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);

        // Serial.printf(" delta_tick_droit %.0f ", delta_odo_tick_droit);
        // Serial.printf(" delta_tick_gauche %.0f ", delta_odo_tick_gauche);

        // Serial.printf("cs_dist_mm %f", convert_distance_tick_to_mm(liste.distance));
        // Serial.printf(" cs_dist_tic %d", (liste.distance));

        // // // Serial.printf(" vitesse robo %f ", vitesse_rob);

        // Serial.printf(" etat_x_y_theta x %d ", etat_x_y_theta);
        // Serial.print("Etat actuel : " + toStringG(etat_actuel_vit_roue_folle_gauche));
        // Serial.print(" " + toStringD(etat_actuel_vit_roue_folle_droite));
        // Serial.println();
        flag_controle = 0;
    }
}

void serialEvent()
{
    while (Serial.available() > 0) // tant qu'il y a des caractères à lire
    {
        // reception(Serial.read());
        char caractere = Serial.read();
        receptionWIFI(caractere);
        Serial.print(caractere);
    }
}
