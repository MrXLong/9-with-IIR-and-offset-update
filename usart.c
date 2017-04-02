#include <p33EP512MU810.h>
#include <uart.h>
#include "usart.h"
#include <dma.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Received data is stored in array Buf  */
char RxBuf[rx_buf_size];
char * Receiveddata = RxBuf;

char TxBuf[tx_buf_size];
char *TransmitData = TxBuf;

///* This is UART3 transmit ISR */
//void __attribute__((interrupt, no_auto_psv)) _U3TXInterrupt(void)
//{
//    IFS5bits.U3TXIF = 0;
//} 
//
///* This is UART3 receive ISR */
//void __attribute__((interrupt, no_auto_psv)) _U3RXInterrupt(void)
//{
//    IFS5bits.U3RXIF = 0;
///* Read the receive buffer till atleast one or more character can be read */ 
//    while( DataRdyUART3())
//      {
//        ( *( Receiveddata)++) = ReadUART3();
//    } 
//} 
//
///****************************************************************************
//  Function Name: _DMA0Interrupt
//  Description:   UART3 data reception interrupt handler.
//  Inputs:        None
//  Returns:       None
//****************************************************************************/
//void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)               //Transmiting data
//{
//
//    IFS0bits.DMA0IF = 0;                                                        //Clear the DMA0 Interrupt Flag
//}
//
///****************************************************************************
//  Function Name: _DMA1Interrupt
//  Description:   UART3 transmission complete
//  Inputs:        None
//  Returns:       None
//****************************************************************************/
//void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)               //Transmiting data
//{
//    LATBbits.LATB4 = ~LATBbits.LATB4;
//    IFS0bits.DMA1IF = 0;        //Clear the DMA1 Interrupt Flag
//}





void initUart()
{
    /* Holds the value of baud register   */
    unsigned int ubrg3;   
    /* Holds the value of uart config reg */
    unsigned int U3MODEvalue;
    /* Holds the information regarding uart
    TX & RX interrupt modes */   
    unsigned int U3STAvalue; 
    
    unsigned int config;
    unsigned int ireq;
    unsigned int sta_address;
    unsigned int stb_address;
    unsigned int pad_address;
    unsigned int count;

    

    /* Turn off UART1module */
    CloseUART3();
    
    //DMA0: RECEIVE
    //DMA1: TRANSMIT
    CloseDMA0();
    CloseDMA1();
    ConfigIntDMA0(DMA0_INT_DISABLE);
    ConfigIntDMA1(DMA1_INT_DISABLE);

    /* Configure uart1 receive and transmit interrupt */
    ConfigIntUART3(UART_RX_INT_EN & UART_RX_INT_PR6 & 
                       UART_TX_INT_EN & UART_TX_INT_PR2);

    /* Configure UART1 module to transmit 8 bit data with one stopbit. Also Enable loopback mode  */
    ubrg3 = BRGVAL_RS232;

    U3MODEvalue = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
                      UART_MODE_FLOW & UART_UEN_00  & UART_DIS_WAKE
                      & UART_DIS_LOOPBACK & UART_EN_ABAUD & UART_NO_PAR_8BIT  &
                      UART_BRGH_SIXTEEN & UART_1STOPBIT;
    U3STAvalue  = 0x3FFF &  UART_IrDA_POL_INV_ZERO &
                      UART_SYNC_BREAK_DISABLED &
                      UART_TX_ENABLE & UART_INT_RX_CHAR &
                      UART_ADR_DETECT_DIS &
                      UART_RX_OVERRUN_CLEAR;   
    

    //CONFIG DMA0 (RX)
    config = DMA0_SIZE_BYTE & PERIPHERAL_TO_DMA0 & DMA0_INTERRUPT_BLOCK & 
            DMA0_NORMAL & DMA0_REGISTER_POST_INCREMENT & DMA0_CONTINUOUS;
    ireq = DMA0_AUTOMATIC;
    sta_address = __builtin_dmaoffset(RxBuf);
    stb_address = 0x0000;
    pad_address = (volatile unsigned int) & U3RXREG;
    count = rx_buf_size-1;
    
    OpenUART3(U3MODEvalue, U3STAvalue, ubrg3);   
    
    OpenDMA0(config, ireq, sta_address, stb_address, pad_address, count); 
    DMA0REQ = 0b01010010; 
    
    //CONFIG DMA1 (TX)
    config = DMA1_SIZE_BYTE & DMA1_TO_PERIPHERAL & DMA1_INTERRUPT_BLOCK & 
         DMA1_NORMAL & DMA1_REGISTER_POST_INCREMENT & DMA1_ONE_SHOT;
    ireq = DMA1_AUTOMATIC;
    sta_address = __builtin_dmaoffset(TxBuf);
    pad_address = (volatile unsigned int) & U3TXREG;
    count = tx_buf_size-1;
 
    OpenDMA1(config, ireq, sta_address, stb_address, pad_address, count);
    DMA1REQ = 0b01010011;
    
    IFS0bits.DMA0IF = 0;            //Clear DMA interrupt
    ConfigIntDMA0(DMA0_INT_ENABLE);
    IFS0bits.DMA1IF  = 0;           // Clear DMA interrupt
    ConfigIntDMA1(DMA1_INT_ENABLE);
    
}


void testUart(void)
{
    /* Data to be transmitted using UART communication module */
    char Txdata[] = {'M','i','c','r','o','c','h','i','p','I','C','D','2'};
    strcpy(TxBuf, Txdata);
    DMA1CNT = strlen(TxBuf)-1;
    /* Load transmit buffer and transmit the same till null character is encountered */
    DMA1CONbits.CHEN = 1;
    U3TXREG = *TxBuf;
    
/* Wait for  transmission to complete */
    while(BusyUART3());
    
/* Read all the data remaining in receive buffer which are unread */
    while(DataRdyUART3())
    {
        (*( Receiveddata)++) = ReadUART3() ;
    } 
    
}

void Delay1Second()
{
    int i=0;
    
    for(i=0;i<100;i++)
    {
        __delay_ms(10);
    }
}

void putString(char* str)
{
    char Txdata[] = {'H','E','L','L','O'};
    strcpy(TxBuf,Txdata);
    DMA1CNT = strlen(TxBuf)-1;
    //while(!U3STAbits.TRMT);
    DMA1CONbits.CHEN = 1;  
    putsUART3((unsigned int *)TxBuf);
    while(BusyUART3());
}

