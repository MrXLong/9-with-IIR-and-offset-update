/*
 * File:   RS232.h
 * Author: Alejandro
 *
 * Created on 10. July 2015, 12:00
 */

#ifndef RS232_H
#define	RS232_H

#define FCY 40000000ULL

#define BAUDRATE_RS232 115200
#define BRGVAL_RS232   ((FCY/BAUDRATE_RS232)/16)-1

// Size of the buffer for received data (in Byte)
#define RS232_TxBufSize                           42//24
// Size of the buffer for received data (in Byte)
#define RS232_RxBufSize                           21            


#endif	/* RS232_H */

extern char RS232Rx[RS232_RxBufSize];
extern char RS232Tx[RS232_TxBufSize];
extern volatile int send_rs232;
extern char uart3RxBuff[RS232_RxBufSize]; 
extern char uart3TxBuff[RS232_TxBufSize];

void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void);
void cfgDMA0_UART_Rx(void);
void cfgDMA1_UART_Tx(void);
void cfgUART3(void);
void initRS232communication(void);
void send_data_RS232(void);
void echoInt(int);
void echoDouble(double);
void echoNewLine(void);
void echoEmptyBuff(void);
void echoString(char* str);

void parseData(void);

void pollUserInput(void);


