#include <Arduino.h>
#include "I2C_ESP32E.h"
#include "Grove_LCD_RGB_Backlight\rgb_lcd.h"
#include <Wire.h>
rgb_lcd lcd;

// put function declarations here:
int myFunction(int, int);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  init_tof();

  lcd.begin(16, 2, false);
  lcd.setRGB(255, 0, 0);
  lcd.setCursor(0, 0);
  lcd.print("Init OK !");
  delay(500);
}

void loop()
{
  read_tof();
  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}