#!/usr/bin/env python
# coding: utf-8

# In[1]:


dico_can_ids = {
    "STOP_ROBOT_FIN_MATCH"              : 0x001,
    "START_ROBOT_MATCH"                 : 0x003,
    "ROTATION"                          : 0x020,
    "LIGNE_DROITE"                      : 0x021,
    "RECALAGE"                          : 0x022,
    "IMMOBILE"                          : 0x023,
    "XYTHETA"                           : 0x024,
    "ESP32_RESTART"                     : 0x025,
    "BASE_ROULANTE"                     : 0x026,
    "ODO_SEND"                          : 0x027,
    "COEFF_P_ASSERVISSEMENT_POSITION"   : 0x028,
    "COEFF_D_ASSERVISSEMENT_POSITION"   : 0x029,
    "COEFF_I_ASSERVISSEMENT_POSITION"   : 0x030,
    "COEFF_P_ASSERVISSEMENT_VITESSE"    : 0x031,
    "COEFF_D_ASSERVISSEMENT_VITESSE"    : 0x032,
    "COEFF_I_ASSERVISSEMENT_VITESSE"    : 0x033,
    "ACKNOWLEDGE_BASE_ROULANTE"         : 0x034,
    "LIDAR"                             : 0x040,
    "CARTE_MAITRE"                      : 0x041,
    "ARU"                               : 0x003,
    "COURT_CIRCUITBATT"                 : 0x005,
    "BATT_MAIN"                         : 0x100,
    "BATT_1"                            : 0x101,
    "BATT_2"                            : 0x102,
    "BATT_3"                            : 0x103,
    "CELLULE_BAT"                       : 0x104,
    "INTERRUPTEUR_BATT1"                : 0x105,
    "INTERRUPTEUR_BATT2"                : 0x106,
    "INTERRUPTEUR_BATT3"                : 0x107,
    "BOOT_CARTE_PUISSANCE"              : 0x108,
    "BRAS_HERKULEX_DEVANT"              : 0x516,
    "HERKULEX_ASCENSEUR_AVANT"          : 0x518,
    "BRAS_HERKULEX_ARRIERE"             : 0x517,
    "HERKULEX_ASCENSEUR_ARRIERE"        : 0x519,
    "PAMI_ONE"                          : 0x5B0,
    "PAMI_TWO"                          : 0x5B1,
    "PAMI_THREE"                        : 0x5B2,
    "PAMI_FOUR"                         : 0x5B3,
    "PAMI_FIVE"                         : 0x5B4,

}
