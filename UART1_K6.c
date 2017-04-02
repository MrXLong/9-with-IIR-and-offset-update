/*
 * File:   UART1_K6.c
 * Author: LJ
 *
 * Created on December 1, 2016, 1:43 PM
 */


#define FP 40000000
#define BAUDRATE 9600
#define BRGVAL ((FP/BAUDRATE)/16)-1

#include "p33EP512MU810.h"
#include "SRF02.h"
#include <stdio.h>
#include <stdlib.h>
#include <libpic30.h>
#include "UART1_K6.h"


#define T_Length        5
char ascii_table[T_Length] = {'0','0','0',0x63,0x6D};


void Configure_Uart1_K6(void) {
    
    U1MODEbits.STSEL = 0; // 1-Stop bit 
    U1MODEbits.PDSEL = 0; // No Parity, 8-Data bits 
    U1MODEbits.ABAUD = 0; // Auto-Baud disabled 
    U1MODEbits.BRGH = 0; // Standard-Speed mode 
    U1BRG = BRGVAL; // Baud Rate setting for 115200 
    U1STAbits.UTXISEL0 = 0; // Interrupt after one TX character is transmitted 
    U1STAbits.UTXISEL1 = 0; 
    IEC0bits.U1TXIE = 1; // Enable UART TX interrupt 
    IFS0bits.U1RXIF = 0;    /*Reset RX interrupt flag */ 
    IPC2bits.U1RXIP = 2;	/*set high priority*/ 
    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received; 
    IEC0bits.U1RXIE = 0; // Disable UART RX interrupt 
    U1MODEbits.UARTEN = 1; // Enable UART 
     U1STAbits.UTXEN = 1; // Enable UART TX 
 
//    __delay_ms(2);
}
//Sonar_SendCommand (0, 0x51);

void output_with_K6(float to_output, int length, int scale, int sign_abs)
{
    int i = 0;
    int j = 0;
    
    char temp[length];
    i = (int)(to_output * scale + 0.5);
    
    if(sign_abs == 1){
        i = abs(i);
    }
    
    for(j = 0; j <= length - 1; j++)
    {
        temp[j] = i%10 + 0x30;
        i /= 10 ;
    }
    for(j = length - 1; j >= 0; j--){
        U1TXREG = temp[j];
        
    }
}
//
void __attribute__((__interrupt__)) _U1TXInterrupt()
{
    IFS0bits.U1TXIF = 0;
}

