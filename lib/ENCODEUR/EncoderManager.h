// EncoderManager.h
#include <Arduino.h>
#include <ESP32Encoder.h>

#ifndef ENCODERMANAGER_H
#define ENCODERMANAGER_H

void setup_encodeur();
void read_encodeurdroit();
void read_encodeurgauche();
void reset_encodeur();
void read_x_y_theta();
#endif
