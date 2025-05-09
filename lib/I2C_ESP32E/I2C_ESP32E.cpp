#include <Arduino.h>
#include "I2C_ESP32E.h"
#include "Variable.h"
#include <Wire.h>
#include <Vl53l0x.h>

int mesures[2];
Vl53l0x monCapteur[2];
VL53L0X_RangingMeasurementData_t measure;

#define NBR_TOF 2
#define MAX_VALUE_DETECTION_TOF 300
#define MIN_VALUE_DETECTION_TOF 30

uint8_t offset_tof = 1;

void init_tof()
{

    Wire.begin(); // Spécifie SDA et SCL pour ESP32
    Wire.setClock(400000);

    // fonction pour lancer les tof ! Tout ce dont on a besoin c'est du nombre de tof à init

    for (int i = 0; i < NBR_TOF; i++)
    { // jusqu'à ce qu'on soit à nb_tof -> mini est de 1 tof donc nb_tof = 1
        // Serial.printf(" i %d\n", i);

        Wire.beginTransmission(0x70);
        // Serial.println("Wire.beginTransmission(0x70)");
        offset_tof = i + 1;
        // Serial.println("offset_tof = i + 1;");

        Wire.write(1 << offset_tof);
        // Serial.println("Wire.write(1 << offset_tof);");

        Wire.endTransmission();
        // Serial.println("Wire.endTransmission();");

        // delay(1000);
        static uint8_t id_tof_actuelle = scanI2C();// Permettre 
        // Serial.print("id_tof_actuelle ");
        Serial.println(id_tof_actuelle, HEX);

        // if ( (id_tof_actuelle == 0x50) || (id_tof_actuelle == 0x51))
                if  (id_tof_actuelle != 0x29) 

        {
            Serial.printf("va,a ");
            if (!monCapteur[i].begin(id_tof_actuelle+i, false))
            {
                delay(1000);                           // delay obligatoire pour lui laisser le temps de bien boot
                monCapteur[i].changeAddress(0x50 + i); // on change l'adresse après le delay
                Serial.printf("Capteur déjà initialisé à 0x%X\n", 0x50 + i);
            }
        }
        else if (!monCapteur[i].begin(0x29, false))
        {
            Serial.printf("Le capteur %d a été initialiser\n", i);                  // on regarde si le démarrage est bon
            // delay(1000);                           // delay obligatoire pour lui laisser le temps de bien boot
            monCapteur[i].changeAddress(0x50 + i); // on change l'adresse après le delay
        }
        else
        {
            Serial.printf("Aucun capteur détecté sur le canal %d\n", i);
        }
    }

    Wire.beginTransmission(0x70);
    Wire.write(0x0F); // le multiplexeur est entièrement ouvert
    Wire.endTransmission();

    // delay(1000); // delay pour laisser le temps au cas où
}

void read_tof()
{
    for (int i = 0; i < NBR_TOF; i++)
    {
        monCapteur[i].performSingleRangingMeasurement(&measure);
        mesures[i] = measure.RangeMilliMeter; // on range toutes les valeurs dans une liste
        if (mesures[i] >= MAX_VALUE_DETECTION_TOF)
        {
            mesures[i] = MAX_VALUE_DETECTION_TOF;
        }
        if (mesures[i] <= MIN_VALUE_DETECTION_TOF)
        {
            mesures[i] = 0;
        }
        Serial.printf(" Cpateur %d ", i);
        Serial.print(mesures[i]); // on print les deux au cas où y ait une merde
    }
    // Serial.println();
}
uint8_t scanI2C() {
    byte error, address;
    Serial.println("Scanning I2C...");
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            Serial.println(address, HEX);
            if ((address != 0x70) && (address != 0x7F))
            {
                return address;
            }
        }
    }
}
