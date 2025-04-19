#ifndef _UART1_H  
#define _UART1_H

void setupUART1(int baudrate);
void send_message_bw16(int id, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7);
void read_message_bw16();

#endif
