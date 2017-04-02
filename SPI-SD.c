/*
 * File:   SPI-SD.c
 * Author: Alejandro
 *
 * Created on 25. August 2015, 14:48
 */

#include <stdlib.h>
#include "p33EP512MU810.h"
#include "SPI-SD.h"
#include "Timer.h"


unsigned char spi1RxBuff[SPISD_RxBufSize];
unsigned char spi1TxBuff[SPISD_TxBufSize];

unsigned char SPISDRx[SPISD_RxBufSize];

int res = 0;

void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void)
{
    IFS0bits.SPI1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _DMA9Interrupt(void)                       //Transmiting data
{
    IFS7bits.DMA9IF = 0;                                                                // Clear the DMA9 Interrupt Flag
}

void __attribute__((interrupt, no_auto_psv)) _DMA10Interrupt(void)                       //Reciving data
{
    SPISDRx[0] = spi1RxBuff [0];

    IFS7bits.DMA10IF = 0;                                                                // Clear the DMA10 Interrupt Flag
}

void delay_ms(int d){
    int a = timer.ms;
    int b = 0;
    do{
        b = timer.ms;
    }while(abs(b-a) < d);
}


void cfgSPI1(void){
    // Interrupt Controller Settings
    IFS0bits.SPI1IF = 0;
    // SPI1CON1 Register Settings
    SPI1CON1bits.MODE16 = 0; // Communication is byte-wide(0); word-wide(1) (16 bits)
    SPI1CON1bits.MSTEN = 1; // Master mode enabled
//    SPI1CON1bits.SPRE = 0b101; //  Secondary Prescale bits (Master mode) by 3            // clock 208333
    // SPI1CON2 Register Settings
    SPI1CON2bits.FRMEN = 0; // Framed mode disabled
    // SPI1STAT Register Settings
    SPI1STATbits.SPISIDL = 0; // Continue module operation in Idle mode
    SPI1STATbits.SPIBEC = 0; // Buffer Length = 1 Word
    SPI1STATbits.SPIROV = 0;  // No Receive Overflow has occurred
    SPI1STATbits.SPIEN = 1; // Enable SPI module
    // Force First Word After Enabling SPI
    DMA9REQbits.FORCE=1;
    while (DMA9REQbits.FORCE == 1);
    IEC0bits.SPI1IE = 1;
}

void cfgDMA9_SPISD_Tx(void)//Transmiting
// DMA3 configuration
// Direction: Read from UART1 Receive Register and write to DMA RAM
// AMODE: Register Indirect with Post-Increment mode
// IRQ: UART2 RX
{
	DMA9CON = 0x2001;                           // One-Shot, Post-Increment, RAM-to-Peripheral (10000000000001)
        DMA9CONbits.SIZE = 1;                       // 0 = Word, 1 = Byte
        DMA9CONbits.AMODE = 0;                      // Configure DMA for Register Indirect mode with post-increment
	DMA9CNT = SPISD_TxBufSize - 1;              // DMA1 Transfer Count Register, Anzahl DMA-Transfers = CNT +1 (7 DMA-Requests)
	DMA9REQ = 0x000A;                       // SPI1TX  Transmitter
        DMA9REQbits.FORCE = 0;                      // Manual Start

	DMA9PAD = (volatile unsigned int) &SPI1BUF; // Pheripal Address Register, siehe Datenblatt DMA, S.42, Example 22-10
	DMA9STAL = __builtin_dmaoffset(spi1TxBuff);// In that way one character is transmiting everytime, Start Address Register
        DMA9STAH = 0x0000;                          // primary Start Address bits (source or destination) Expample 22-10

	IFS7bits.DMA9IF  = 0;                       // Clear DMA interrupt
	IEC7bits.DMA9IE  = 0;                       // Enable DMA interrupt
        DMA9CONbits.CHEN = 1;                       // Enable DMA channel (Transmit)
                                                    //nach Ende des Datenblocks wird das CHEN-Bit automatisch wieder auf 0 gesetzt
}

void cfgDMA10_SPISD_Rx(void)//Receiving
// DMA3 configuration
// Direction: Read from UART1 Receive Register and write to DMA RAM
// AMODE: Register Indirect with Post-Increment mode
// IRQ: UART2 RX
{
	DMA10CON = 0x0002;                       // Continuous, Post-Inc., Periph-RAM
        DMA10CONbits.SIZE = 1;                   // 0 = Word, 1 = Byte
        DMA10CONbits.MODE = 0b00;                // Continuous, PING PONG mode disabled
	DMA10CNT = SPISD_RxBufSize-1;
	DMA10REQ = 0x000A;                      //0b00001010

	DMA10PAD = (volatile unsigned int) &SPI1BUF;
        DMA10STAL = __builtin_dmaoffset(spi1RxBuff);

	IFS7bits.DMA10IF  = 0;			// Clear DMA interrupt
	IEC7bits.DMA10IE  = 1;			// Enable DMA interrupt
        DMA7CONbits.CHEN = 1;			// Enable Receiving-DMA Channel
}

void cfgSPIMD(void){        //without DMA
    /* The following code sequence shows SPI register configuration for Master mode */
    IFS0bits.SPI1IF = 0; // Clear the Interrupt flag
    IEC0bits.SPI1IE = 0; // Disable the interrupt
    // SPI1CON1 Register Settings
    SPI1CON1bits.DISSCK = 0; // Internal serial clock is enabled
    SPI1CON1bits.DISSDO = 0; // SDOx pin is controlled by the module
    SPI1CON1bits.MODE16 = 1; // Communication is word-wide (16 bits)
    SPI1CON1bits.MSTEN = 1; // Master mode enabled
    SPI1CON1bits.SMP = 0; // Input data is sampled at the middle of data output time
    SPI1CON1bits.CKE = 0; // Serial output data changes on transition from
    // Idle clock state to active clock state
    SPI1CON1bits.CKP = 0; // Idle state for clock is a low level;
    // active state is a high level
    SPI1STATbits.SPIEN = 1; // Enable SPI module
    // Interrupt Controller Settings
    IFS0bits.SPI1IF = 0;  // Clear the Interrupt flag
    IEC0bits.SPI1IE = 1; // Enable the interrupt
}

void initSPISDcommunication(void)
{
    cfgSPIMD(); //cfg without DMA
 /*   cfgSPI1();
    cfgDMA9_SPISD_Tx();
    cfgDMA10_SPISD_Rx();
    send_data_spi_sd(0xAA);*/
    SPI1BUF = 0xFFFF;

}

void send_data_spi_sd(int data){
    spi1TxBuff[0] = data;
    DMA9REQbits.FORCE = 1;
}

int SD_CMD0 (void){ //GO_IDLE_STATE

    send_data_spi_sd(0xFF);

    send_data_spi_sd(0x40);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x95);

    send_data_spi_sd(0xFF);
    int res = SPISDRx[0];
    return (res);
}

int SD_CMD1 (void){ //SEND_OP_COND

    send_data_spi_sd(0xFF);

    send_data_spi_sd(0x41);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0xF9);

    send_data_spi_sd(0xFF);
    int res = SPISDRx[0];
    return (res);
}

int SD_CMD8 (void){ //SEND_IF_COND

    send_data_spi_sd(0xFF);

    send_data_spi_sd(0x48);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x01);
    send_data_spi_sd(0xAA);
    send_data_spi_sd(0x87);

    send_data_spi_sd(0xFF);
    int res = SPISDRx[0];
    return (res);

}

int SD_CMD13 (void){ //SEND_STATUS

    send_data_spi_sd(0xFF);

    send_data_spi_sd(0x4D);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x0D);

    send_data_spi_sd(0xFF);
    int res = SPISDRx[0];
    return (res);

}

int SD_CMD41 (void){ //SEND_OP_COND

    send_data_spi_sd(0xFF);

    send_data_spi_sd(0x69);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0xE5);

    send_data_spi_sd(0xFF);
    int res = SPISDRx[0];
    return (res);

}

int SD_CMD45 (void){

    send_data_spi_sd(0xFF);

    send_data_spi_sd(0x77);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x65);

    send_data_spi_sd(0xFF);
    int res = SPISDRx[0];
    return (res);

}

int SD_CMD58 (void){ //READ_OCR

    send_data_spi_sd(0xFF);

    send_data_spi_sd(0x7A);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0x00);
    send_data_spi_sd(0xFD);

    send_data_spi_sd(0xFF);
    int res = SPISDRx[0];
    return (res);

}

void SD_init(SD_DATA data)
{
/*
    int i = 0;
    int aux = 0;
    delay_ms(1);
    for(i=0;i<10;i++){
        send_data_spi_sd(0xFF);
    }
    //set iddle mode
    while(SD_CMD0() != 0x01);
////    delay_ms(5);
    // at this stage, the card is in idle mode and ready for start up
    aux = SD_CMD8();
    if(aux == 0x05){
        data.v1 = 1;
    }
    aux = SD_CMD41();
    aux = SD_CMD58();
    if(aux == 0x05){
        data.acmd58 = aux;
    }
*/

}
