#include <Arduino.h>

#ifndef _ID_H
#define _ID_H

#define STOP_ROBOT_FIN_MATCH 0x001
#define START_ROBOT_MATCH 0x002

#define ROTATION 0x020
#define LIGNE_DROITE 0x021
#define RECALAGE 0x022
#define IMMOBILE 0x023
#define XYTHETA 0x024

#define ESP32_RESTART 0x025

#define POLAIRE 0x026
#define ODO_SEND 0x027

#define ACKNOWLEDGE_BASE_ROULANTE 0x034

#define CARTE_MAITRE 0x041


#define PAMI_ONE 0x5B0
#define PAMI_TWO 0x5B1
#define PAMI_THREE 0x5B2
#define PAMI_FOUR 0x5B3
#define PAMI_FIVE 0x5B4


#endif
