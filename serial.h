/****************************************************************************

 Header File:		serial.h

 In this Header file the functions of inserting and taking from the buffer
 transmit and buffer received are defined and all the functions required for
 the RS232 communication and the transmit and receive buffers.

****************************************************************************/
#ifndef __serial_H
#define __serial_H

//#include "p33EP512MU810.h"
#include <stdint.h>
#define FCY 40000000ULL

//** Baudrate **************************************************************
#define BAUDRATE 115200
#define BRGVAL   ((FCY/BAUDRATE)/16)-1

//Size of the transmission buffer in bytes.
#define TxBufSize                         255
//Size of the reception buffer in bytes.
#define RxBufSize                         250

//DMA1 buffers. They contain the received data through RS232.
//extern char uart3RxBuff[RxBufSize];
//extern char uart3TxBuff[TxBufSize];

//void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void);
//void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void);
//void cfgDMA0_UART_Rx(void);
//void cfgDMA1_UART_Tx(void);
//void cfgUART3(void);

//extern unsigned int incomingMessage;        // Flag to indicate if there are a new message ready to use
//extern int positionDMA1TxBuffer;            // Pointer to indicate the position to write in the DMA2 buffer
/****************************************************************************
Function:     initSerial
Parameters:   none
Return value: none
This function initializes the UART3 module to use the uart connection using
DMA via channel 0,1.
****************************************************************************/
void initSerial();

/****************************************************************************
Function:     sendDataToSerial_DMA
Parameters:   none
Return value: none
Sending data to uart DMA.
****************************************************************************/
void sendDataToSerial_DMA();

void echoString(char* str);
void force_send(void);
void send_mavlink(uint8_t* buffer, uint16_t length);


int16_t udb_serial_callback_get_byte_to_send(void);
int16_t mavlink_serial_send(const uint8_t buf[], uint16_t len);
void mp_mavlink_transmit(uint8_t ch);
void udb_serial_start_sending_data(void);

void mav_msg_receive(uint8_t rxchar);

#endif //__serial_H
