#include <Grove_LCD_RGB_Backlight\rgb_lcd.h>
#ifndef _I2C_ESP32E_H  
#define _I2C_ESP32E_H

extern rgb_lcd lcd;

void read_tof();
void init_tof();
uint8_t scanI2C();
void init_lcd_groove(bool activate_wire);

void init_mutex(bool activate_mutex);
#endif
