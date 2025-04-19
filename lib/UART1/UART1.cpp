#include <Arduino.h>
#include "UART1.h"
#include "Variable.h"
void setupUART1(int baudrate)
{
    Serial1.begin(baudrate);

}

void send_message_bw16(int id, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7)
{
    Serial1.write(id);
    Serial1.write(data0);
    Serial1.write(data1);
    Serial1.write(data2);
    Serial1.write(data3);
    Serial1.write(data4);
    Serial1.write(data5);
    Serial1.write(data6);
    Serial1.write(data7);
}

void read_message_bw16()
{
    uint8_t BUFFER_SIZE = 9;
    uint8_t buffer[BUFFER_SIZE];
    if (Serial1.available() >= 9)
    { // J'attends la reception de 9 donnée qui sont l'id et mes 8 data
        uint8_t bytesRead = Serial1.readBytes((char *)buffer, BUFFER_SIZE);

        if (bytesRead == BUFFER_SIZE)
        {
            // Traitement du buffer
            // Serial.print("Trame reçue : ");
            for (int i = 1; i <= BUFFER_SIZE; i++)
            {
                rxMsg.data[i] = buffer[i];
                // Serial.print(buffer[i], HEX);// Affiche chaque octet de données en hexadécimal
                // Serial.print(" ");
            }
            // Serial.println();
        }
    }
}

