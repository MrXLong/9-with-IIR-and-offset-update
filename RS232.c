/****************************************************************************
 * File:   RS232.c
 * Created by: Alejandro Perez on 20.08.2015

 * Description: This file contains set up of UART communication between
 * the microcontroller and the UART3(JP11), which is intended to be used as a
 * way to communicate between the PC and the uC. This allows a medium for
 * exchanging data between the PC and uC in real-time. It's original intention
 * is to tune PID parameters while the quad is active.

 * Last Modified: 12.01.2015 (MW)
 * Modification: Set up UART3 to exchange data with the PC as it pertains
 * to GPS position control. i.e. send waypoints/setpoints from PC to quad,
 * send intended movement data from quad to PC.
****************************************************************************/


#include "p33EP512MU810.h"
#include "RS232.h"
#include "GPS.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libpic30.h>
#include <uart.h>
#include "PID.h"
#include "stddef.h"


//*** Global variables ******************************************************
char uart3RxBuff[RS232_RxBufSize]; 
char uart3TxBuff[RS232_TxBufSize];

volatile int send_rs232 = 0;
volatile int usr_input=0;

char RS232Rx[RS232_RxBufSize];      //intermediate buffer for echoing receive buffer



/* This is UART3 transmit ISR */
void __attribute__((interrupt, no_auto_psv)) _U3TXInterrupt(void)
{
    IFS5bits.U3TXIF = 0;
} 

/* This is UART3 receive ISR */
void __attribute__((interrupt, no_auto_psv)) _U3RXInterrupt(void)
{
//BELOW PARTIALLY WORKS w RX INT EN and DMACNT LARGE
//    while(DataRdyUART3())
//    {
//        uart3RxBuff[rx_cnt] = ReadUART3();
//        rx_cnt++;
//        if((rx_cnt>(RS232_RxBufSize-1)) || uart3RxBuff[rx_cnt-1] == 0x0D)
//        {
//            DMA0CNT = rx_cnt-1;
//            DMA0CONbits.CHEN = 1;
//            DMA0REQbits.FORCE = 1;   
//            rx_cnt = 0;
//        }
//    }
    IFS5bits.U3RXIF = 0;
/* Read the receive buffer till atleast one or more character can be read */ 
//...
} 

/****************************************************************************
  Function Name: _DMA0Interrupt
  Description:   UART3 data reception interrupt handler.
  Inputs:        None
  Returns:       None
 * NOTE: In current configuration, to use this DMA, data must be sent in
 * bursts which have the exact size of the receive buffer.
****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)                       //Transmiting data
{
    //int i = 0;
    //int bufSize=RS232_RxBufSize;
    
//    while(i<bufSize && uart3RxBuff[i]!=0x0D)
//    {
//        RS232Rx[i]=uart3RxBuff[i];   
//        i++;
//    }


    usr_input = 1;
    LATBbits.LATB4 = ~LATBbits.LATB4;
    U3STAbits.OERR = 0;
    IFS0bits.DMA0IF = 0;                    // Clear the DMA0 Interrupt Flag
}

/****************************************************************************
  Function Name: _DMA1Interrupt
  Description:   UART3 transmission complete
  Inputs:        None
  Returns:       None
****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)                       //Transmiting data
{
    //LATBbits.LATB4 = ~LATBbits.LATB4;
    //memset(uart3TxBuff, ' ', RS232_TxBufSize);
    IFS0bits.DMA1IF = 0;                                                                // Clear the DMA1 Interrupt Flag
}

void cfgDMA0_UART_Rx(void)//Receiving
// DMA0 configuration
// Direction: Read from UART3 Receive Register and write to DMA RAM
// AMODE: Register Indirect with Post-Increment mode
// IRQ: UART3 RX
{   
    DMA0CONbits.CHEN = 0;           //Disable DMA Channel	
    IEC0bits.DMA0IE  = 0;			//Disable DMA interrupt
    IFS0bits.DMA0IF = 0;            //Clear DMA interrupt
    DMA0CONbits.SIZE = 1;           //0: Word, 1: Byte
    DMA0CONbits.DIR = 0;            //Peripheral->RAM
    DMA0CONbits.MODE = 0;           //One-shot

    
	DMA0CNT = RS232_RxBufSize-1;        //DMA interrupt occurs after receiving of entire buffer
	DMA0REQ = 0b01010010;               // UART3RX [DS70348C-page 22-22]
	DMA0PAD = (volatile unsigned int) & U3RXREG;
    DMA0STAL = __builtin_dmaoffset(uart3RxBuff);
    DMA0STAH = 0x0000;
    
    _DMA0IP = 4; 
    IEC0bits.DMA0IE  = 1;			// Enable DMA interrupt
    DMA0CONbits.CHEN = 1;			// Enable Receiving-DMA Channel
}

void cfgDMA1_UART_Tx(void)//Transmitting
// DMA1 configuration
// Direction: Read from DMA RAM and write to UART3 Transmit register
// AMODE: Register Indirect with Post-Increment mode
// IRQ: UART3 TX
{
    DMA1CONbits.CHEN = 0;                       //Disable DMA channel
    IEC0bits.DMA1IE  = 0;                       //Disable DMA interrupt
    IFS0bits.DMA1IF  = 0;                       // Clear DMA interrupt flag
    DMA1CONbits.SIZE = 1;                       //0: Word; 1: Byte
    DMA1CONbits.DIR = 1;                        //RAM->Peripheral
    DMA1CONbits.MODE = 1;                       //One-Shot
    DMA1CONbits.AMODE1 = 0;
    DMA1CONbits.AMODE0 = 0;
    
	DMA1CNT = RS232_TxBufSize-1;                // DMA1 Transfer Count Register            
	DMA1REQ = 0b01010011;                       // UART3TX [DS70348C-page 22-22]

	DMA1PAD = (volatile unsigned int) & U3TXREG; // Peripheral Address Register, siehe Datenblatt DMA, S.42, Example 22-10
    
	DMA1STAL = __builtin_dmaoffset(uart3TxBuff);// In that way one character is transmiting everytime, Start Address Register
    DMA1STAH = 0x0000;                          // primary Start Address bits (source or destination) Expample 22-10
    
    _DMA1IP = 3;
    IEC0bits.DMA1IE  = 1;                       // Enable DMA interrupt
    DMA1CONbits.CHEN = 1;                       // Enable DMA channel (Transmit)
}

void cfgUART3(void)                                 // UART3 Configuration for RX und TX
{
    U3MODEbits.UARTEN = 0;                      //Disable UART3 Module
    U3MODEbits.LPBACK = 0;                      //Disable Loop Back Mode
	U3BRG = BRGVAL_RS232;                       //U3BRD = ((FCY/BAUDRATE_RS232)/16)-1
 
	U3MODEbits.PDSEL = 0;                       //No Parity, 8-bit data (U3MODEbits.PDSEL=?)
	U3MODEbits.STSEL = 0;                       //1-Stop Bit (U3MODEbits.STSEL=?)
    U3MODEbits.ABAUD = 0;                       //Autobaud Disabled
    
    //Interrupt Mode
	U3STAbits.UTXISEL0 = 0;                     //(b13:15)  Interrupt after one Tx character is transmitted
	U3STAbits.UTXISEL1 = 0;
	U3STAbits.URXISEL = 0;                      //(b6:7)    Interrupt after one RX character is received
    
    //Enable Interrupts
    //IEC5bits.U3RXIE = 1;                        //Enable Receive Interrupt
    //IEC5bits.U3TXIE = 1;                        //Enable Transmit Interrupt
    IFS5bits.U3RXIF = 0;                        //Clear Interrupt Flag
    IFS5bits.U3TXIF = 0;                        //Clear Interrupt Flag
    
    U3MODEbits.UARTEN = 1;                      // (b15)    Enable UART3
    U3STAbits.UTXEN = 1;                        // (b10)    Enable UART3 Tx
}

void initRS232communication(void)
{
    cfgDMA1_UART_Tx();
    cfgDMA0_UART_Rx();
    cfgUART3();
}

void pollUserInput()
{
    if(usr_input)
    {
        parseData();
        memset(uart3RxBuff,0x00,RS232_RxBufSize);   //after parsing, clear buffers
        memset(RS232Rx,0x00,RS232_RxBufSize);
        usr_input = 0;                              //reset boolean
    }
}

/****************************************************************************
  Function Name: parseData
  Description:   parses data sent over UART3
  Inputs:        None
  Returns:       None
  Data Format:   type,axis,loop,coeff,value
****************************************************************************/
void parseData()
{
    char *pch;                              //character pointer
    const char d[2] = ",";                  //seperation character
    char temp[6];
    int dataType=0;
    int axis=0;
    int loop=0;
    int coeff=0;
    double value=0;
    
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int startCharFound = 0;
    unsigned int startCharPos = 0;
    unsigned int endCharFound = 0;
    unsigned int endCharPos = 0;
    int size=0;
    
    while(!startCharFound || !endCharFound)
    {
        if(uart3RxBuff[i]=='$')
        {
            startCharFound = 1;
            startCharPos = i;
        }
        if (!startCharFound)
        {
            startCharPos++;
        }
        
        if((startCharFound) &&  uart3RxBuff[i]=='*')
        {
           endCharFound = 1; 
           endCharPos = i;
        }
        if (!endCharFound)
        {
            endCharPos++;
        }  
        
        if(i==RS232_RxBufSize)
            i=0;
        else
            i++;
    }
    
    size = endCharPos-startCharPos;   //size of message including '$' and '*'
    
    //if end char is in a position lower than start char, size will be negative
    if (size < 0)
        size = RS232_RxBufSize+size;
    
    for(j=0; j<=size; j++)      
    {
        RS232Rx[j] = uart3RxBuff[startCharPos+j+1];
    }
    
    //what type of data?
    //0:set PID data; 1:list PID data
    //which Axis?   0:aile; 1:elev; 2:rudder
    //which loop?   0:rate(inner); 1:angle(outer)
    //which coefficeint   0:Proportional; 1:Integral; 2;Derivative
    //desired value
    
    pch = strtok(RS232Rx,d);
    strcpy(temp,pch);
    dataType=atoi(temp);
    
    pch = strtok(NULL,d);
    strcpy(temp,pch);
    axis=atoi(temp);
    
    pch = strtok(NULL,d);
    strcpy(temp,pch);
    loop=atoi(temp);
    
    pch = strtok(NULL,d);
    strcpy(temp,pch);
    coeff=atoi(temp);
    
    pch = strtok(NULL,d);
    strcpy(temp,pch);
    value=atof(temp);
     
    switch(dataType)
    {
        case 0: //Set PID data
            setPID(axis,loop,coeff,(float)value,ptr_FIX);
        break;
        case 1: //List Current PID values
            listPID();
        break;
        default:
            listPID();
        break;
    }
    
    
}

/****************************************************************************
  Function Name: setPID
  Description:   sets PID coefficients with data sent over UART3
  Inputs:        None
  Returns:       None
  Data Format:   type,axis,loop,coeff,value
****************************************************************************/
void setPID(int axis,int loop,int coeff,float value,s_controller_fixedpoint* ctrlr)
{
    int offset = 0;
    int offset_base = 0;
    char axis_name[5];
    char loop_name[6];
    char coeff_name[2];
    
    float *fltPtr = NULL;      //Must be of s_controller_fixedpoint member type

    
    switch(axis)
    {
        case 0:
            offset = offsetof(s_controller_fixedpoint, Kp_aile_a);
            strcpy(axis_name,"aile");
        break;
        case 1:
            offset = offsetof(s_controller_fixedpoint, Kp_elev_a);
            strcpy(axis_name,"elev");
        break;
        case 2:
            offset = offsetof(s_controller_fixedpoint, Kp_rudd_a);
            strcpy(axis_name,"rudd");
        break;
        default:
            offset = offsetof(s_controller_fixedpoint, Kp_aile_a);
            strcpy(axis_name,"aile");
        break;
    }
    offset_base = offsetof(s_controller_fixedpoint, Kp_aile_a);
    offset -= offset_base;
    
    //switch base to item chosen before (aile,elev,rudd)
    offset_base = offset;
    switch(loop)
    {
        case 0:
            offset = offsetof(s_controller_fixedpoint, Kp_aile_r);
            strcpy(loop_name,"rate");
        break;
        case 1:
            offset = offsetof(s_controller_fixedpoint, Kp_aile_a);
            strcpy(loop_name,"angle");
        break;
        default:
            offset = offsetof(s_controller_fixedpoint, Kp_aile_r);
            strcpy(loop_name,"rate");
        break;
    }
    offset += offset_base;

    //switch base to item chosen before (angle, rate)
    offset_base = offset;
    switch(coeff)
    {
        case 0:
            offset = offsetof(s_controller_fixedpoint, Kp_aile_a);
            strcpy(coeff_name,"P");
        break;
        case 1:
            offset = offsetof(s_controller_fixedpoint, Ki_aile_a);
            strcpy(coeff_name,"I");
        break;
        case 2:
            offset = offsetof(s_controller_fixedpoint, Kd_aile_a);
            strcpy(coeff_name,"D");
        break;
        default:
            offset = offsetof(s_controller_fixedpoint, Kp_aile_a);
            strcpy(coeff_name,"P");
        break;
    }
    offset += offset_base;
    
    
    fltPtr = (float*)ctrlr + offset/sizeof(float);
    ctrlr = (s_controller_fixedpoint*)fltPtr;
    
    ctrlr->Kp_aile_a = value;
    
    //respond to change in variable
    while(!U3STAbits.TRMT);
    sprintf(uart3TxBuff, "\r\n%s %s %s changed to %2.3f", axis_name,loop_name,coeff_name,(double)value);
    DMA1CNT = strlen(uart3TxBuff)-1;
    DMA1CONbits.CHEN = 1;
    DMA1REQbits.FORCE = 1;   
}

/****************************************************************************
  Function Name: listPID
  Description:   outputs PID coefficients over UART3
  Inputs:        None
  Returns:       None
  Data Format:   type,axis,loop,coeff,value
****************************************************************************/
void listPID()
{
    echoString("Aile Values:");
    echoString("Rate:");
    echoDouble(coeff_fixedpoint.Kp_aile_r);
    echoDouble(coeff_fixedpoint.Ki_aile_r);
    echoDouble(coeff_fixedpoint.Kd_aile_r);
    echoString("Angle:");
    echoDouble(coeff_fixedpoint.Kp_aile_a);
    echoDouble(coeff_fixedpoint.Ki_aile_a);
    echoDouble(coeff_fixedpoint.Kd_aile_a);
    echoString("Elev Values:");
    echoString("Rate:");
    echoDouble(coeff_fixedpoint.Kp_elev_r);
    echoDouble(coeff_fixedpoint.Ki_elev_r);
    echoDouble(coeff_fixedpoint.Kd_elev_r);
    echoString("Angle:");
    echoDouble(coeff_fixedpoint.Kp_elev_a);
    echoDouble(coeff_fixedpoint.Ki_elev_a);
    echoDouble(coeff_fixedpoint.Kd_elev_a);
    echoString("Rudd Values:");
    echoString("Rate:");
    echoDouble(coeff_fixedpoint.Kp_rudd_r);
    echoDouble(coeff_fixedpoint.Ki_rudd_r);
    echoDouble(coeff_fixedpoint.Kd_rudd_r);
    echoString("Angle:");
    echoDouble(coeff_fixedpoint.Kp_rudd_a);
    echoDouble(coeff_fixedpoint.Ki_rudd_a);
    echoDouble(coeff_fixedpoint.Kd_rudd_a);
}


void send_data_RS232(void){

    // Sending Test ptr_pulsewidth->gearCap
/*
    uart3TxBuff[0] = ptr_pulsewidth->aux1Cap /256; // HByte;
    uart3TxBuff[1] = ptr_pulsewidth->aux1Cap %256; //LByte;
    uart3TxBuff[2] = ptr_pulsewidth->aux1;
    uart3TxBuff[3] = 4;
    uart3TxBuff[4] = ptr_pulsewidth->gearCap /256; // HByte;;
    uart3TxBuff[5] = ptr_pulsewidth->gearCap %256; //LByte;
    uart3TxBuff[6] = ptr_pulsewidth->gear;
    uart3TxBuff[7] = 8;
    uart3TxBuff[8] = 9;
    uart3TxBuff[9] = 10;
    uart3TxBuff[10] = 11;
    uart3TxBuff[11] = 12;
    uart3TxBuff[12] = 13;
    uart3TxBuff[13] = 14;
    uart3TxBuff[14] = 15;
    uart3TxBuff[15] = 16;
    uart3TxBuff[16] = 17;
    uart3TxBuff[17] = 18;
    uart3TxBuff[18] = 19;
    uart3TxBuff[19] = 20;
    uart3TxBuff[20] = 21;
    uart3TxBuff[21] = 22;
    uart3TxBuff[22] = 23;
    uart3TxBuff[23] = 0xFF;
    */


/*
    //Controller PID Data
    uart3TxBuff[0] = ptr_FIX->Kp_elev_a * 100;
    uart3TxBuff[1] = ptr_FIX->Kp_aile_a * 100;
    uart3TxBuff[2] = ptr_FIX->Ki_aile_a * 100;
    uart3TxBuff[3] = ptr_FIX->Kp_elev_r * 100;
    uart3TxBuff[4] = ptr_FIX->Kp_aile_r * 100;
    uart3TxBuff[5] = 0xFB;
    uart3TxBuff[6] = ptr_FIX->Ki_elev_r * 100;
    uart3TxBuff[7] = ptr_FIX->Ki_aile_r * 100;
    uart3TxBuff[8] = 0xFC;
    uart3TxBuff[9] = ptr_FIX->Kd_elev_r * 1000;
    uart3TxBuff[10] = ptr_FIX->Kd_aile_r * 1000;
    uart3TxBuff[11] = 0xFD;
    uart3TxBuff[12] = ptr_FIX->Kp_rudd_a * 100;
    uart3TxBuff[13] = ptr_FIX->Kp_rudd_r * 100;
    uart3TxBuff[14] = ptr_FIX->Ki_rudd_r * 100;
    uart3TxBuff[15] = abs(UM7_Sensors.eulerRollAngle * (EulerConstant))%256;;
    uart3TxBuff[16] = (ptr_FIX->actual_angle_elev);
    uart3TxBuff[17] = (ptr_FIX->actual_angle_aile);
    uart3TxBuff[18] = abs(ptr_FIX->actual_angle_rudd);
    uart3TxBuff[19] = ptr_FIX->actual_angle_rudd;
    uart3TxBuff[20] = ptr_FIX->rudd_angle_target;
    uart3TxBuff[21] = 0xA;
    uart3TxBuff[22] = RS232Rx[0];
    uart3TxBuff[23] = ptr_pulsewidth->gear;
    uart3TxBuff[24] = ptr_pulsewidth->aux1;

    uart3TxBuff[25] = 0xFF;
*/

    // RC data and angles
/*
    uart3TxBuff[0] = ptr_pulsewidth->thro /256; // HByte
    uart3TxBuff[1] = ptr_pulsewidth->thro %256; //LByte
    uart3TxBuff[2] = ptr_pulsewidth->elev /256; // HByte
    uart3TxBuff[3] = ptr_pulsewidth->elev %256; //LByte
    uart3TxBuff[4] = ptr_pulsewidth->rudd /256; // HByte
    uart3TxBuff[5] = ptr_pulsewidth->rudd %256; //LByte
    uart3TxBuff[6] = ptr_pulsewidth->aile /256; // HByte
    uart3TxBuff[7] = ptr_pulsewidth->aile %256; //LByte
    uart3TxBuff[8] = abs(ptr_FIX->setpoint_angle_elev);
    uart3TxBuff[9] = abs(ptr_FIX->setpoint_angle_aile);
    uart3TxBuff[10] = abs(ptr_FIX->setpoint_angle_rudd);
    uart3TxBuff[11] = abs(UM7_Sensors.eulerRollAngle * (EulerConstant)) %256;
    uart3TxBuff[12] = abs(ptr_FIX->actual_angle_aile) %256;
    uart3TxBuff[13] = abs(UM7_Sensors.eulerPitchAngle * (EulerConstant)) %256;
    uart3TxBuff[14] = abs(ptr_FIX->actual_angle_elev) %256;
    uart3TxBuff[15] = abs(UM7_Sensors.eulerYawAngle * (EulerConstant)) %256;
    uart3TxBuff[16] = abs(ptr_FIX->actual_angle_rudd) %256;
    uart3TxBuff[17] = abs(UM7_Sensors.eulerRollRate * (GyroConstant)) %256;
    uart3TxBuff[18] = abs(ptr_FIX->actual_rate_aile) %256;
    uart3TxBuff[19] = abs(UM7_Sensors.eulerPitchRate * (GyroConstant)) %256;
    uart3TxBuff[20] = abs(ptr_FIX->actual_rate_elev) %256;
    uart3TxBuff[21] = abs(UM7_Sensors.eulerYawRate * (GyroConstant)) %256;
    uart3TxBuff[22] = abs(ptr_FIX->actual_rate_rudd) %256;
    uart3TxBuff[23] = 0xFF;

*/


/*
    uart3TxBuff[0] = abs(ptr_FIX->setpoint_angle_elev);
    uart3TxBuff[1] = abs(UM7_Sensors.eulerPitchAngle*EulerConstant)%256;//abs(ptr_FIX->actual_angle_elev) %256;
    uart3TxBuff[2] = abs(ptr_FIX->elev_error_a) %256;
    uart3TxBuff[3] = ptr_FIX->elev_output_a;

    uart3TxBuff[4] = abs(ptr_FIX->setpoint_angle_aile);
    uart3TxBuff[5] = abs(ptr_FIX->actual_angle_aile) %256;
    uart3TxBuff[6] = abs(ptr_FIX->aile_error_a) %256;
    uart3TxBuff[7] = ptr_FIX->aile_output_a;

    uart3TxBuff[8] = abs(ptr_FIX->setpoint_angle_rudd);
    uart3TxBuff[9] = abs(ptr_FIX->actual_angle_rudd) %256;
    uart3TxBuff[10] = abs(ptr_FIX->rudd_error_a) %256;
    uart3TxBuff[11] = ptr_FIX->rudd_output_a;

    uart3TxBuff[12] = 0XFA;

    uart3TxBuff[13] = abs(ptr_FIX->actual_rate_elev) %256;
    uart3TxBuff[14] = abs(ptr_FIX->elev_error_r) %256;
    uart3TxBuff[15] = ptr_FIX->elev_output_r;

    uart3TxBuff[16] = abs(ptr_FIX->actual_rate_aile) %256;
    uart3TxBuff[17] = abs(ptr_FIX->aile_error_r) %256;
    uart3TxBuff[18] = ptr_FIX->aile_output_r;

    uart3TxBuff[19] = abs(ptr_FIX->actual_rate_rudd) %256;
    uart3TxBuff[20] = abs(ptr_FIX->rudd_error_r) %256;
    uart3TxBuff[21] = ptr_FIX->rudd_output_r;
    uart3TxBuff[22] = 0xFF;
    uart3TxBuff[23] = 0xFF;
*/


 /*   uart3TxBuff[0] = 0xFA;
    uart3TxBuff[1] = UM7_Sensors.eulerRollAngle * (EulerConstant);
    uart3TxBuff[2] = ptr_FIX->actual_angle_aile;
    uart3TxBuff[3] = 0xFB;
    uart3TxBuff[4] = abs(UM7_Sensors.eulerPitchAngle*EulerConstant);
    uart3TxBuff[5] = ptr_FIX->actual_angle_elev;
    uart3TxBuff[6] = 0xFC;
    uart3TxBuff[7] = UM7_Sensors.eulerYawAngle * (EulerConstant);
    uart3TxBuff[8] = ptr_FIX->actual_angle_rudd;
    uart3TxBuff[9] = 0xFD;
    uart3TxBuff[10] = UM7_Sensors.eulerRollRate * (GyroConstant);
    uart3TxBuff[11] = ptr_FIX->actual_rate_aile;
    uart3TxBuff[12] = 0xFE;
    uart3TxBuff[13] = UM7_Sensors.eulerPitchRate * (GyroConstant);
    uart3TxBuff[14] = ptr_FIX->actual_rate_elev;
    uart3TxBuff[15] = 0xFF;
    uart3TxBuff[16] = UM7_Sensors.eulerYawRate * (GyroConstant);
    uart3TxBuff[17] = ptr_FIX->actual_rate_rudd;

    uart3TxBuff[18] = 0xFF;
    uart3TxBuff[19] = 0xFF;
    uart3TxBuff[20] = 0xFF;
    uart3TxBuff[21] = 0xFF;
    uart3TxBuff[22] = 0xFF;
    uart3TxBuff[23] = 0xFF;
 */
/*
    uart3TxBuff[0] = ptr_FIX->setpoint_angle_elev;
    uart3TxBuff[1] = ptr_FIX->actual_angle_elev;
    uart3TxBuff[2] = ptr_FIX->elev_error_a;
    uart3TxBuff[3] = ptr_FIX->elev_integral_a;
    uart3TxBuff[4] = ptr_FIX->Kp_elev_a * 2;
    uart3TxBuff[5] = ptr_FIX->Ki_elev_a;
    uart3TxBuff[6] = ptr_FIX->elev_output_a;
    uart3TxBuff[7] = 0xFF;
    uart3TxBuff[8] = ptr_FIX->setpoint_angle_aile;
    uart3TxBuff[9] = ptr_FIX->actual_angle_aile;
    uart3TxBuff[10] = ptr_FIX->aile_error_a;
    uart3TxBuff[11] = ptr_FIX->aile_integral_a;
    uart3TxBuff[12] = ptr_FIX->Kp_aile_a *2;
    uart3TxBuff[13] = ptr_FIX->Ki_aile_a;
    uart3TxBuff[14] = ptr_FIX->aile_output_a;
    uart3TxBuff[15] = 0xFF;
    uart3TxBuff[16] = ptr_FIX->setpoint_angle_rudd;
    uart3TxBuff[17] = ptr_FIX->actual_angle_rudd;
    uart3TxBuff[18] = ptr_FIX->rudd_error_a;
    uart3TxBuff[19] = ptr_FIX->rudd_integral_a;
    uart3TxBuff[20] = ptr_FIX->Kp_rudd_a * 2;
    uart3TxBuff[21] = ptr_FIX->Ki_rudd_a;
    uart3TxBuff[22] = ptr_FIX->rudd_output_a;
    uart3TxBuff[23] = 0xFF;
*/

 /*
    uart3TxBuff[0] = ptr_FIX->setpoint_angle_elev;
    uart3TxBuff[1] = ptr_FIX->actual_angle_elev;
    uart3TxBuff[2] = ptr_FIX->elev_output_a;
    uart3TxBuff[3] = 0xFF;
    uart3TxBuff[4] = ptr_FIX->setpoint_angle_aile;
    uart3TxBuff[5] = ptr_FIX->actual_angle_aile;
    uart3TxBuff[6] = ptr_FIX->aile_output_a;
    uart3TxBuff[7] = 0xFF;
    uart3TxBuff[8] = ptr_FIX->setpoint_angle_rudd;
    uart3TxBuff[9] = ptr_FIX->actual_angle_rudd;
    uart3TxBuff[10] = ptr_FIX->rudd_output_a;
    uart3TxBuff[11] = 0xFF;
    uart3TxBuff[12] = ptr_FIX->actual_rate_elev;
    uart3TxBuff[13] = ptr_FIX->elev_output_r;
    uart3TxBuff[14] = 0xFF;
    uart3TxBuff[15] = ptr_FIX->actual_rate_aile;
    uart3TxBuff[16] = ptr_FIX->aile_output_r;
    uart3TxBuff[17] = 0xFF;
    uart3TxBuff[18] = ptr_FIX->actual_rate_rudd;
    uart3TxBuff[19] = ptr_FIX->rudd_output_r;
    uart3TxBuff[20] = 0xFF;
    uart3TxBuff[21] = 0xFF;
    uart3TxBuff[22] = 0xFF;
    uart3TxBuff[23] = 0xFF;
*/

        //GPS data
/*
    uart3TxBuff[0] = Sonar_Dist[0];
    uart3TxBuff[1] = GPSRx[1];
    uart3TxBuff[2] = GPSRx[2];
    uart3TxBuff[3] = GPSRx[3];
    uart3TxBuff[4] = GPSRx[4];
    uart3TxBuff[5] = GPSRx[5];
    uart3TxBuff[6] = GPSRx[6];
    uart3TxBuff[7] = abs(ptr_GPSdata->latitude) / 256;
    uart3TxBuff[8] = abs(ptr_GPSdata->latitude) % 256;
    uart3TxBuff[9] = ptr_GPSdata->NS;
    uart3TxBuff[10] = abs(ptr_GPSdata->longitude)/ 256;
    uart3TxBuff[11] = abs(ptr_GPSdata->longitude)% 256;
    uart3TxBuff[12] = ptr_GPSdata->EW;
    uart3TxBuff[13] = GPSRx[13];
    uart3TxBuff[14] = ptr_GPScommdata->latitude_[0];
    uart3TxBuff[15] = ptr_GPScommdata->latitude_[1];
    uart3TxBuff[16] = ptr_GPScommdata->latitude_[2];
    uart3TxBuff[17] = ptr_GPScommdata->latitude_[3];
    uart3TxBuff[18] = ptr_GPScommdata->longitude_[0];
    uart3TxBuff[19] = ptr_GPScommdata->longitude_[1];
    uart3TxBuff[20] = ptr_GPScommdata->longitude_[2];
    uart3TxBuff[21] = ptr_GPScommdata->longitude_[3];
    uart3TxBuff[22] = ptr_GPScommdata->longitude_[4];
    uart3TxBuff[23] = ptr_pulsewidth->gear;
    uart3TxBuff[24] = ptr_pulsewidth->aux1;

    uart3TxBuff[25] = 0xFF;

 

    DMA1REQbits.FORCE = 1;

    send_rs232 = 0;
*/
}

//void echoInt(int data)
//{
//    int i = 0;
//    while(U3STAbits.TRMT == 0);
//    
//    for(i=0; i<RS232_TxBufSize; i++)
//        uart3TxBuff[i] = '0';
//    
//    sprintf(uart3TxBuff, "\r\n%d", data);
//    
//    DMA1CONbits.CHEN = 1;   
//    DMA1REQbits.FORCE = 1;
//}

void echoDouble(double data)
{
    int i = 0;
    
    while(U3STAbits.TRMT == 0);
    
    for(i=0; i<RS232_TxBufSize; i++)
    {
        //uart3TxBuff[i] = '-';
    }
    
    
    sprintf(uart3TxBuff, "\r\n%09.9f", data);
    
    DMA1CONbits.CHEN = 1;   
    DMA1REQbits.FORCE = 1;
}

void echoNewLine(void)
{
    while(U3STAbits.TRMT == 0);
    sprintf(uart3TxBuff, "\r\n");
    
    DMA1CONbits.CHEN = 1;                          
    DMA1REQbits.FORCE = 1; 
}

void echoEmptyBuff(void)
{
    int i=0;
    
    while(U3STAbits.TRMT == 0);
    
    for(i=0; i<RS232_TxBufSize; i++)
        uart3TxBuff[i] = ' ';

    
    DMA1CONbits.CHEN = 1;                          
    DMA1REQbits.FORCE = 1;                          
}

void echoString(char* str)
{
    while(U3STAbits.URXDA); //wait while data is receiving
    while(!U3STAbits.TRMT);

    sprintf(uart3TxBuff, "\r\n%s", str);
    //strcpy(uart3TxBuff, str);
    DMA1CNT = strlen(uart3TxBuff)-1;
    /* Load transmit buffer and transmit the same till null character is encountered */
    //DMA1CONbits.CHEN = 1;       
    //U3TXREG = *uart3TxBuff;
    DMA1CONbits.CHEN = 1;
    DMA1REQbits.FORCE = 1;    
}
