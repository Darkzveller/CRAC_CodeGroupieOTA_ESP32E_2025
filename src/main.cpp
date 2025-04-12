#include <mat.h>
#include <Arduino.h>
#include "OTA.h" // Inclusion de la bibliothèque OTA (Over-The-Air) pour les mises à jour sans fil
#include "Variable.h"
#include "MOTEUR.h"
#include "EncoderManager.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "ID_CAN.h"
// #include "CAN_ESP32E.h"
#include "USE_FUNCTION.h"

float rectificateur_coeeff = 0;
// struct Ordre_deplacement
// {
//     int general_purpose;
//     float angle;
//     int sens_rotation;
//     int16_t distance;
//     int vitesse_croisiere;
//     int sens_ligne_droite;
//     float consigne_distance_recalage;
//     int vitesse_recalage;
//     int sens_recalage;
//     float x;
//     float y;
//     float theta;
//     float vitesse_x_y_theta;
//     float x_polaire;
//     float y_polaire;
//     int nbr_passage;
// };
// Ordre_deplacement liste = {TYPE_DEPLACEMENT_IMMOBILE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static bool polaire_true_or_false = false;
void controle(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        read_x_y_theta();
        // recalage();
        switch (liste.general_purpose)
        {
        case TYPE_DEPLACEMENT_LIGNE_DROITE:
            Serial.printf("TYPE_DEPLACEMENT_LIGNE_DROITE ");
            ligne_droite(liste.distance, liste.vitesse_croisiere);
            if (return_flag_asser_roue())
            {
                // sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 8, TYPE_DEPLACEMENT_LIGNE_DROITE, 0, 0, 0, 0, 0, 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
            }

            break;
        case TYPE_DEPLACEMENT_ROTATION:

            // Serial.printf("TYPE_DEPLACEMENT_ROTATION ");

            rotation(liste.angle, liste.vitesse_croisiere);

            if (return_flag_asser_roue())
            {
                consigne_theta_prec = degrees(theta_robot);
                // sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 8, TYPE_DEPLACEMENT_ROTATION, 0, 0, 0, 0, 0, 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
            }
            break;
        case TYPE_DEPLACEMENT_IMMOBILE:
            consigne_position_droite = consigne_odo_droite_prec;
            consigne_position_gauche = consigne_odo_gauche_prec;
            // Serial.printf(" TYPE_DEPLACEMENT_IMMOBILE");
            liste.general_purpose = TYPE_VIDE;
            // sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 8, TYPE_DEPLACEMENT_IMMOBILE, 0, 0, 0, 0, 0, 0, 0);

            break;
        case TYPE_DEPLACEMENT_X_Y_POLAIRE:
            // Serial.printf(" TYPE_DEPLACEMENT_X_Y_POLAIRE ");
            asser_polaire_tick(liste.x_polaire, liste.y_polaire, 0, liste.nbr_passage = true);

            if (flag_fin_mvt)
            {
                // sendCANMessage(ACKNOWLEDGE_BASE_ROULANTE, 0, 0, 8, TYPE_DEPLACEMENT_X_Y_POLAIRE, 0, 0, 0, 0, 0, 0, 0);
                liste.general_purpose = TYPE_DEPLACEMENT_IMMOBILE;
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

        // asservissement_roue_folle_droite_tick(consigne_position_droite +=60, odo_tick_droit);
        // asservissement_roue_folle_gauche_tick(consigne_position_gauche +=60, odo_tick_gauche);

        moteur_droit( 4095,false);
        moteur_gauche(4095,false);
        flag_controle = 1;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}
/*
void odo(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {

        read_x_y_theta();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
    }
}
*/
void bus_can(void *parameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // readCANMessage();

        // // Attendre la fin du mouvement avant de passer à l'ordre suivant
        // if (flag_fin_mvt)
        // {
        //     FIFO_lecture = (FIFO_lecture + 1) % SIZE_FIFO;
        //     flag_fin_mvt = false; // Réinitialiser le flag pour le prochain ordre

        //     // vTaskDelay(pdMS_TO_TICKS(Tcan)); // Temporisation pour éviter une boucle trop rapide
        //     // continue;                        // Retourne au début de la boucle en attendant que flag_fin_mvt soit vrai
        // }

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
        case 0:
            break;

        default:
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Tcan));
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
    // Initialisation du Can
    // setupCAN(1000E3);

    // Boucle jusqu'à ce qu'un client soit connecté via le port série WiFi
    // while (!SerialWIFI.available())
    // {
    //     delay(500);                                             // Attente de 500 ms avant de vérifier à nouveau
    //     Serial.println("Aucun client connecté, en attente..."); // Message indiquant qu'il n'y a pas de client connecté
    // }
    // delay(10000);
    // affichage_commande_wifi();
    Serial.println("on commence");

    // Serial.printf("avncement_gauche enter : %.0f\n", avncement_gauche);
    // Serial.printf("avncement_droite enter : %.0f\n", avncement_droite);

    reset_encodeur();
    delay(500);
    reset_encodeur();

    xTaskCreate(
        controle,   // nom de la fonction
        "controle", // nom de la tache que nous venons de vréer
        10000,      // taille de la pile en octet
        NULL,       // parametre
        10,         // tres haut niveau de priorite
        NULL        // descripteur
    );
    // xTaskCreate(
    //     odo,   // nom de la fonction
    //     "odo", // nom de la tache que nous venons de vréer
    //     10000, // taille de la pile en octet
    //     NULL,  // parametre
    //     11,    // tres haut niveau de priorite
    //     NULL   // descripteur
    // );
    xTaskCreate(
        bus_can,   // nom de la fonction
        "bus_can", // nom de la tache que nous venons de vréer
        10000,     // taille de la pile en octet
        NULL,      // parametre
        9,         // tres haut niveau de priorite
        NULL       // descripteur
    );
}

// Boucle principale, exécutée en continu après le setup
void loop()
{
    if (flag_controle)
    {
        Serial.printf(" Odo x %.3f ", odo_x);
        Serial.printf(" odo_y %.3f ", odo_y);
        Serial.printf(" teheta %.3f ", degrees(theta_robot));
        // Serial.printf(" delta_droit %.0f ", delta_droit);
        Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
        Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);

        Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
        Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);

        // Serial.printf(" delta_tick_droit %.0f ", delta_odo_tick_droit);
        // Serial.printf(" delta_tick_gauche %.0f ", delta_odo_tick_gauche);

        // Serial.printf("cs_dist_mm %f", convert_distance_tick_to_mm(liste.distance));
        // Serial.printf(" cs_dist_tic %d", (liste.distance));

        // // // Serial.printf(" vitesse robo %f ", vitesse_rob);

        // // Serial.printf(" etat_x_y_theta x %d ", etat_x_y_theta);
        Serial.print("Etat actuel : " + toStringG(etat_actuel_vit_roue_folle_gauche));
        Serial.print(" " + toStringD(etat_actuel_vit_roue_folle_droite));
        Serial.println();
        flag_controle = 0;
    }
}

void reception(char ch)
{
    static int x_low_byte, x_high_byte;
    static int y_low_byte, y_high_byte;
    static int t_low_byte, t_high_byte;

    static int i = 0;
    static String chaine = "";
    String commande;
    String valeur;
    int index, length;
    int cmd = 0;

    if ((ch == 13) or (ch == 10))
    {
        index = chaine.indexOf(' ');
        length = chaine.length();
        if (index == -1)
        {
            commande = chaine;
            valeur = "";
        }
        else
        {
            commande = chaine.substring(0, index);
            valeur = chaine.substring(index + 1, length);
        }
        if (commande == "ROTATION")
        {
            int8_t sens = 0;
            cmd = valeur.toInt();
            if (cmd > 0)
            {
                if (cmd >= 1)
                {
                    sens = 1;
                }
            }
            else if (cmd < 0)
            {
                if (cmd <= -1)
                {
                    sens = -1;
                }
            }
            // cmd = fabs(cmd);
            cmd = cmd;
            uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
            uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort

            TelnetStream.println();

            TelnetStream.printf("Send command Rotation with cons");
            TelnetStream.printf(" cmd %d", cmd);
            TelnetStream.printf(" sens %d", sens);
            TelnetStream.println();

            Serial.println();
            Serial.printf("Send command Rotation with cons");
            Serial.printf(" cmd %d", cmd);
            Serial.printf(" sens %d", sens);
            Serial.println();

            rxMsg.id = ROTATION;
            rxMsg.data[0] = highByte;
            rxMsg.data[1] = lowByte;
            rxMsg.data[2] = SPEED_TORTUE;

            // sendCANMessage(ROTATION, 0, 0, 8, highByte, lowByte, 0x7B, 0, 0, 0, 0, 0);
        }
        if (commande == "LIGNE")
        {
            int8_t sens = 0;
            cmd = valeur.toInt();
            if (cmd > 0)
            {
                if (cmd >= 1)
                {
                    sens = 1;
                }
            }
            else if (cmd < 0)
            {
                if (cmd <= -1)
                {
                    sens = -1;
                }
            }
            // cmd = fabs(cmd);
            cmd = cmd;
            uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
            uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
            TelnetStream.println();
            TelnetStream.printf("Send command LIGNE with cons");
            TelnetStream.printf(" cmd %d", cmd);
            TelnetStream.printf(" lowByte %d", lowByte);
            TelnetStream.printf(" highByte %d", highByte);

            TelnetStream.println();
            Serial.println();
            Serial.printf("Send command LIGNE with cons");
            Serial.printf(" cmd %d", cmd);
            Serial.printf(" sens %d", sens);
            Serial.println();

            rxMsg.id = LIGNE_DROITE;
            rxMsg.data[0] = highByte;
            rxMsg.data[1] = lowByte;
            rxMsg.data[2] = SPEED_TORTUE;

            // sendCANMessage(LIGNE_DROITE, 0, 0, 8, highByte, lowByte, 0x7B, 0, 0, 0, 0, 0);
        }

        if ((commande == "RESTART") || (commande == "restart"))
        {
            TelnetStream.println();

            TelnetStream.printf("Send command RESTART");
            TelnetStream.println();
            Serial.printf("Send command RESTART ");
            Serial.println();
            rxMsg.id = ESP32_RESTART;
            // sendCANMessage(ESP32_RESTART, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        }
        if (commande == "xp")
        {
            cmd = valeur.toInt();

            uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
            uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
            x_low_byte = lowByte;
            x_high_byte = highByte;
            TelnetStream.println();
            TelnetStream.printf("Send command xp with cons");
            TelnetStream.printf(" cmd %d", cmd);
            TelnetStream.println();
            Serial.println();
            Serial.printf("Send command xp with cons");
            Serial.printf(" cmd %d", cmd);
            Serial.println();
        }
        if (commande == "yp")
        {
            cmd = valeur.toInt();

            uint8_t lowByte = cmd & 0xFF;         // Octet de poids faible
            uint8_t highByte = (cmd >> 8) & 0xFF; // Octet de poids fort
            y_low_byte = lowByte;
            y_high_byte = highByte;
            TelnetStream.println();

            TelnetStream.printf("Send command yp with cons");
            TelnetStream.printf(" cmd %d", cmd);
            TelnetStream.println();

            Serial.println();
            Serial.printf("Send command yp with cons");
            Serial.printf(" cmd %d", cmd);
            Serial.println();

            rxMsg.data[0] = x_high_byte;
            rxMsg.data[1] = x_low_byte;
            rxMsg.data[2] = y_high_byte;
            rxMsg.data[3] = y_low_byte;
            liste.nbr_passage = rxMsg.data[4];

            // sendCANMessage(POLAIRE, 0, 0, 8, x_high_byte, x_low_byte, y_high_byte, y_low_byte, 0, 0, 0, 0);
        }

        chaine = "";
    }
    else
    {
        chaine += ch;
    }
}
void serialEvent()
{
    while (Serial.available() > 0) // tant qu'il y a des caractères à lire
    {
        // reception(Serial.read());
        char caractere = Serial.read();
        reception(caractere);
        Serial.print(caractere);
    }
}
