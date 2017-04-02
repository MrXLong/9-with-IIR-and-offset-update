/* 
 * File:   UART.h
 * Author: Simon
 *
 * Created on December 18, 2014, 1:14 PM
 */

#ifndef UART_H
#define	UART_H

#include "p33EP512MU810.h"
#include <stdio.h>
#define FCY 40000000ULL
#include "Pins.h"
#include "UM7.h"
#include "PID.h"

#define BAUDRATE 9600
#define BRGVAL   ((FCY/BAUDRATE)/16)-1

//Size of the transmission buffer in bytes.
#define RS232_TxBufSize                         255
//Size of the reception buffer in bytes.
#define RS232_RxBufSize                         250

#define SizeQueue         128
// Header protocol to send a command
#define CommandHeader               0x23
// Header protocol to send a report
#define ReportHeader                0x2A
// Track-Merlin identification number
#define TrackMerlinIIIID            0x03
// Device identification  number
#define IdRaspberryPi               0x00
// Maximum message size
#define MaxDataSize                 0x0E

// Creating the struct Features, inside of the union Payload, inside of a struct is possible get the data in different ways
typedef struct PayloadFeatures{//8 Bytes
    int Duration;
    float LinearVelocity;
    float AngularVelocity;
}Features;

union PayloadType{//8 Bytes
    //struct PayloadFeatures Features;
    Features Features;
    char Byte[10];
};

typedef struct MessageRS232{// Auxiliar variable to safe and use the first message received by DMA
    unsigned char Header[2];
    unsigned char DestinationId;
    unsigned char PayloadSize;
    unsigned char SourceID;
    unsigned char PacketType;//Command
    union PayloadType Payload;
    unsigned char Checksum;
}Message;

extern Message MessageReceived;// Message received from the RS232
extern Message queue[SizeQueue];// Incoming message ready to store or execute in real time
extern Message messageToTransmit;// Message to send via RS232

extern unsigned int messagePosition;// Pointer to indicate the position where the missing message begins
extern unsigned int messageSize;
extern unsigned int bufferAorB;// 0 means bufferA, 1 means bufferB
extern unsigned char DATARS232[6];

//DMA1 buffers. They contain the received data through RS232.
extern char uart1RxBuff[RS232_RxBufSize];
extern char uart1TxBuff[RS232_TxBufSize];

extern unsigned int incomingMessage;// Flag to indicate if there are a new message ready to use
extern int positionDMA2TxBuffer;// Pointer to indicate the position to write in the DMA2 buffer
void cfgDMA1_UART_Rx(void);
void cfgDMA2_UART_Tx(void);
void cfgUART1(void);
void initRS232(void);
void sendDataToRS232_DMA(void);

void __attribute__((interrupt, auto_psv)) _DMA1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void);

#endif	/* UART_H */

