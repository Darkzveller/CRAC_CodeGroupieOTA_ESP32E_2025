#include <Arduino.h>
#include <Wire.h>
#include <Vl53l0x.h>

int mesures[2];
Vl53l0x monCapteur[2];
VL53L0X_RangingMeasurementData_t measure;

int nb_tof = 2;
int offset_tof = 1;

void init_tof() {

  //fonction pour lancer les tof ! Tout ce dont on a besoin c'est du nombre de tof à init

  for (int i = 0; i < nb_tof; i++) {  // jusqu'à ce qu'on soit à nb_tof -> mini est de 1 tof donc nb_tof = 1
    Wire.beginTransmission(0x70);
    Serial.println("test");
    offset_tof = i + 1;
    Wire.write(1 << offset_tof);
    Wire.endTransmission();
    delay(1000);
    if (!monCapteur[i].begin(0x29, false)) Serial.print("good");  // on regarde si le démarrage est bon
    delay(1000);                                                  // delay obligatoire pour lui laisser le temps de bien boot
    monCapteur[i].changeAddress(0x50 + i);                        // on change l'adresse après le delay
  }

  Wire.beginTransmission(0x70);
  Wire.write(0x0F);  // le multiplexeur est entièrement ouvert
  Wire.endTransmission();

  delay(1000);  // delay pour laisser le temps au cas où
}

void read_tof() {
  for (int i = 0; i < nb_tof; i++) {
    monCapteur[i].performSingleRangingMeasurement(&measure);
    mesures[i] = measure.RangeMilliMeter;  // on range toutes les valeurs dans une liste
    Serial.println(mesures[i]);            //on print les deux au cas où y ait une merde
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("oui\n");
  Wire.begin();  // Spécifie SDA et SCL pour ESP32
  Wire.setClock(400000);
  init_tof();

}

void loop() 
{
  read_tof();
  Serial.println();
}
