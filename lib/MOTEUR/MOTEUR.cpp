#include "Variable.h" // Remonte d'un niveau pour atteindre lib
#include "MOTEUR.h"
#include <math.h>

void setup_motors()
{
    // Moteur droit
    pinMode(M1_INA, OUTPUT);
    pinMode(M1_INB, OUTPUT);
    ledcSetup(channel_1, frequence, resolution_pwm);
    ledcAttachPin(PWM_1, channel_1);
    // Moteur gauche
    pinMode(M2_INA, OUTPUT);
    pinMode(M2_INB, OUTPUT);
    ledcAttachPin(PWM_2, channel_2);
}

void moteur_droit(int pwm, bool sens)
{
    // Serial.printf("Pwm MotorD %4d ", pwm);
    // Serial.printf(" S %d ",sens);

    if (sens == true)
    {
        digitalWrite(M1_INA, 1);
        digitalWrite(M1_INB, 0);
    }
    else
    {
        digitalWrite(M1_INA, 0);
        digitalWrite(M1_INB, 1);
    }
    ledcWrite(channel_1, pwm);
}

void moteur_gauche(int pwm, bool sens)
{
    // Serial.printf("Pwm MotorG %4d ", pwm);
    // Serial.printf(" S %d ",sens);
    if (sens == true)
    {
        digitalWrite(M2_INA, 1);
        digitalWrite(M2_INB, 0);
    }
    else
    {
        digitalWrite(M2_INA, 0);
        digitalWrite(M2_INB, 1);
    }
    ledcWrite(channel_2, pwm);
}

void stop_motors()
{
    bool stop = true;
    digitalWrite(M2_INA, stop);
    digitalWrite(M2_INB, stop);
    digitalWrite(M1_INA, stop);
    digitalWrite(M1_INB, stop);
}
void stop_moteur_droit()
{

    bool stop = true;
    digitalWrite(M1_INA, stop);
    digitalWrite(M1_INB, stop);
}

void stop_moteur_gauche()
{

    bool stop = true;
    digitalWrite(M2_INA, stop);
    digitalWrite(M2_INB, stop);
}

void freinage_moteur_gauche(bool on_off, int Vmax_consigne)
{
    float max_freinage_pwm = 1;
    if (on_off)
    {
        if (Vmax_consigne > 0)
        {
            // moteur_gauche((pow(2, resolution_pwm) - 1) * max_freinage_pwm, true);
            moteur_gauche(fabs(Vmax_consigne), true);
        }
        else
        {
            // moteur_gauche((pow(2, resolution_pwm) - 1) * max_freinage_pwm, false);
            moteur_gauche(fabs(Vmax_consigne), false);
        }
    }
}
void freinage_moteur_droit(bool on_off, int Vmax_consigne)
{
    float max_freinage_pwm = 1;

    if (on_off)
    {
        if (Vmax_consigne > 0)
        {
            // moteur_droit((pow(2, resolution_pwm) - 1) * max_freinage_pwm, true);
            moteur_droit(fabs(Vmax_consigne), true);
        }
        else
        {
            // moteur_droit((pow(2, resolution_pwm) - 1) * max_freinage_pwm, false);
            moteur_droit(fabs(Vmax_consigne), false);
        }
    }
}
float mini = 1.5;
void moteur_droit_polaire(int pwm)
{
    // Serial.printf("Pwm MotorD %4d ", pwm);

    if (pwm > 0)
    {
        digitalWrite(M1_INA, 1);
        digitalWrite(M1_INB, 0);
    }
    else
    {
        digitalWrite(M1_INA, 0);
        digitalWrite(M1_INB, 1);
    }
    if (fabs(pwm) < mini)
    {
        pwm = mini;
    }
    ledcWrite(channel_1, fabs(pwm));
}

void moteur_gauche_polaire(int pwm)
{

    // Serial.printf(" Pwm MotorG %4d ", pwm);
    if (pwm > 0)
    {
        digitalWrite(M2_INA, 1);
        digitalWrite(M2_INB, 0);
    }
    else
    {
        digitalWrite(M2_INA, 0);
        digitalWrite(M2_INB, 1);
    }
    if (fabs(pwm) < mini)
    {
        pwm = mini;
    }
    ledcWrite(channel_2, fabs(pwm));
}

void start_stop_moteur_star(bool activate){

    pinMode(PIN_MOTEUR_STAR,OUTPUT);
    digitalWrite(PIN_MOTEUR_STAR,activate);
}

