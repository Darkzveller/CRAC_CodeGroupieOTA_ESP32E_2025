#include <Arduino.h>
#include <Wire.h>
#include "rgb_lcd.h"
rgb_lcd lcd;

// put function declarations here:
int myFunction(int, int);

void setup() {
  Serial.begin(115200);
    Serial.printf("Demmarage Ldccd\n");
    lcd.begin(16, 2, true);
    lcd.setRGB(127, 127, 0); // Couleur jaune pour l'initialisation
    lcd.print("Init LCD...");
    delay(1000);
    lcd.clear();
    // xSemaphoreGive(i2cMutex); // Libérer le mutex après l'initialisation
    // }
    Serial.println("Écran LCD initialisé.");
}

void loop() {
lcd.setCursor(0, 1);
    // print the number of seconds since reset:
    lcd.print(millis() / 1000);

    delay(100);

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}