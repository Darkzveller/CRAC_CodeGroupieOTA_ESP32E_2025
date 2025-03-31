#include <Arduino.h>

#ifndef Variable_H
#define Variable_H
extern float tension_bat ;
// Parametre FreeRTOS
#define Te 2.5
#define Tcan 10

// Déclaration des variables externes
#define frequence 19500
#define resolution_pwm 12
#define POURCENT_MAX_PWM 0.75
// Moteur droit
#define PWM_1 17
#define M1_INA 26
#define M1_INB 25
#define channel_1 0
// Moteur Gauche
#define PWM_2 18
#define M2_INA 16
#define M2_INB 15
#define channel_2 1
// Encodeur + Parametre physique du robot
#define ENTRAXE 110.0
#define LARGEUR_ROBOT_mm 225.0
#define TIC_PER_TOUR 2048.0
#define RESOLUTION_ROUE_CODEUSE 10.0
#define COEFF_ROUE_DROITE 1.0
#define COEFF_ROUE_GAUCHE 1.0
#define SIZE_WHEEL_DIAMETER_mm 50.0
extern float perimetre_robot;

extern double theta_robot_prec;
extern double theta_robot;

extern double odo_x, odo_y;
extern double odo_last_d;
extern double odo_last_g;

extern double odo_tick_droit;
extern double odo_tick_gauche;
extern double odo_tick_droit_last;
extern double odo_tick_gauche_last;

extern double delta_odo_tick_droit;
extern double delta_odo_tick_gauche;

extern double delta_droit;
extern double delta_gauche;

extern double distance_parcourue;

extern double vitesse_rob ;
extern double vitesse_rob_roue_droite;
extern double vitesse_rob_roue_gauche;

extern float consigne_odo_droite_prec;
extern float consigne_odo_gauche_prec;
extern float consigne_theta_prec;

extern double consigne_odo_x_prec;
extern double consigne_odo_y_prec;

extern double odo_dist_gauche;
extern double odo_dist_droit;

#define PIN_ENCODEUR_1 23
#define PIN_ENCODEUR_2 22

#define PIN_ENCODEUR_3 36
#define PIN_ENCODEUR_4 39
// uint8_t  tab_encodeur_droit[2] = {23, 22};
// uint8_t  tab_encodeur_gauche[2] = {36, 39};

//************Liste Ordre Deplacement */

struct Ordre_deplacement
{
    int general_purpose;
    float angle;
    int sens_rotation;
    int16_t distance;
    int vitesse_croisiere;
    int sens_ligne_droite;
    float consigne_distance_recalage;
    int vitesse_recalage;
    int sens_recalage;
    float x;
    float y;
    float theta;
    float vitesse_x_y_theta;
    float x_polaire;
    float y_polaire;
    int nbr_passage;
};

// Déclaration de la variable globale (définie dans `variable.cpp`)
extern Ordre_deplacement liste;


//************Asservissement ROUE FOLLE EN TICK */
#define SPEED_TORTUE 45
#define SPEED_NORMAL 60
#define SPEED_ULTRA  100
extern float coeff_P_roue_folle_tick_gauche;
extern float coeff_D_roue_folle_tick_gauche;
extern float coeff_I_roue_folle_tick_gauche;

extern float coeff_P_roue_folle_tick_droite;
extern float coeff_D_roue_folle_tick_droite;
extern float coeff_I_roue_folle_tick_droite;

extern float erreur_prec_roue_folle_droite_tick;
extern float erreur_prec_roue_folle_gauche_tick;
extern float integral_limit_roue_folle_tick;

extern float somme_integral_roue_folle_droite_tick;
extern float somme_integral_roue_folle_gauche_tick;

//************************Asservissement vitesse Roue folle en TICK */
extern float Vmax;
extern float Amax;
extern float Dmax;
extern float limit_reprise_asser;

extern float acc_actuel_droite;
extern double consigne_vit_droite;
extern double consigne_dist_droite;

extern float acc_actuel_gauche;
extern double consigne_vit_gauche;
extern double consigne_dist_gauche;

extern double Ta_counter_droite;
extern double Ta_counter_gauche;
extern double Td_counter_droite;
extern double Td_counter_gauche;
extern double Tc_counter_droite;
extern double Tc_counter_gauche;
extern double T_attente_droite;
extern double T_counter_attente_droite;

extern volatile bool type_ligne_droite;

extern double T_attente_gauche;
extern double T_counter_attente_gauche;

extern float distance_accel_droite;
extern float distance_decl_droite;
extern float distance_accel_gauche;
extern float distance_decl_gauche;

extern float kp_vit;
extern float ki_vit;
extern float kd_vit;
extern float erreur_vit_precedente_roue_folle_droite;
extern float integral_limit;
extern float somme_erreur_vit_roue_folle_droite;
extern float somme_erreur_vit_roue_folle_gauche;
extern float erreur_vit_precedente_roue_folle_gauche;

extern bool start_asservissement_roue_gauche;
extern bool start_asservissement_roue_droite;

// Enumérations pour les états des roues en vitesse
enum Etat_vitesse_roue_folle_droite
{
  ETAT_ATTENTE_Vitesse_ROUE_FOLLE_DROITE,
  ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_DROITE,
  ETAT_CROISIERE_Vitesse_ROUE_FOLLE_DROITE,
  ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE,
  ETAT_ARRET_Vitesse_ROUE_FOLLE_DROITE,
  ETAT_VIDE_Vitesse_ROUE_FOLLE_DROITE
};
extern Etat_vitesse_roue_folle_droite etat_actuel_vit_roue_folle_droite;

enum Etat_vitesse_roue_folle_gauche
{
  ETAT_ATTENTE_Vitesse_ROUE_FOLLE_GAUCHE,
  ETAT_ACCELERATION_Vitesse_ROUE_FOLLE_GAUCHE,
  ETAT_CROISIERE_Vitesse_ROUE_FOLLE_GAUCHE,
  ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE,
  ETAT_ARRET_Vitesse_ROUE_FOLLE_GAUCHE,
  ETAT_VIDE_Vitesse_ROUE_FOLLE_GAUCHE
};
extern Etat_vitesse_roue_folle_gauche etat_actuel_vit_roue_folle_gauche;

// Déclaration des fonctions de conversion
String toStringG(Etat_vitesse_roue_folle_gauche etat);
String toStringD(Etat_vitesse_roue_folle_droite etat);

//************************Convitesse de vitesse */
extern float consigne_position_droite;
extern float consigne_position_gauche;

//************************Freinage */
extern float coeff_P_freinage;
extern float coeff_D_freinage;
extern float coeff_I_freinage;
extern float integral_limit_freinage;
extern float somme_erreur_freinage_roue_folle_droite;
extern float erreur_prec_freinage_roue_folle_droite;
extern float somme_erreur_freinage_roue_folle_gauche;
extern float erreur_prec_freinage_roue_folle_gauche;
//************************Asser Correction d'angle */

extern double coeff_P_angle ;           
extern double coeff_I_angle ;         
extern double coeff_D_angle ;           
extern double integral_limit_angle ; 
// Variables globales pour le PID
extern double erreur_prec_angle ;    // Erreur précédente
extern double somme_integral_angle; // Somme des erreurs pour le calcul intégral
extern double correction ;

//************************Mouvement */
extern int sens ;

extern int etat_x_y_theta;
extern double theta_premiere_rotation;
extern double theta_deuxieme_rotation;
extern float consigne_regulation_moyenne ;

//************************Polaire en tick */
extern float erreur_distance ;
extern float erreur_orient ;
extern float consigne_dist_polaire_tick_max ;
extern float coeff_rot_polaire_tick ;
extern float coeff_dist_polaire_tick ;
extern float consigne_rot_polaire_tick ;
extern float consigne_dist_polaire_tick;

extern float coeff_decc_distance_polaire_tick ;
extern float distance_decl_polaire_tick;

extern bool calcul_decl_polaire_tick;


//***********LOOP******************* */

extern bool flag_controle;

//***********CAN******************* */

#define SIZE_FIFO 32

typedef struct CANMessage
{
  bool extd = false;
  bool rtr = false;
  uint32_t id = 0;
  uint8_t lenght = 0;
  uint8_t data[8] = {0};
} CANMessage;
extern CANMessage rxMsg; // data received by CAN to control the robot
extern int etat_lecture_can;
extern unsigned char FIFO_ecriture;
extern signed char FIFO_lecture;
extern signed char FIFO_occupation;
extern signed char FIFO_max_occupation;

//***********Ordre de déplacement******************* */
#define TYPE_DEPLACEMENT_IMMOBILE 1
#define TYPE_DEPLACEMENT_LIGNE_DROITE 2
#define TYPE_DEPLACEMENT_ROTATION 3
#define TYPE_DEPLACEMENT_X_Y_THETA 4
#define TYPE_DEPLACEMENT_RECALAGE 5
#define TYPE_VIDE 6
#define TYPE_DEPLACEMENT_X_Y_POLAIRE 7 

extern bool flag_fin_mvt;

//*********** Qu'est ce qu'on voit au borne de la batterie******************* */
extern float courant;
extern float tension;
extern float puissance;
#endif
