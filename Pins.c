#include "Pins.h"
#include "p33EP512MU810.h"

void init_Pins (void)
{
    //Pin & Port Konfiguration

    //Ports B,E,F = digital
    ANSELB = 0;
    
    //MW below instructions are superfluous because of above instruction
    ANSELBbits.ANSB12 = 0;  
    ANSELBbits.ANSB13 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB15 = 0;

    ANSELE = 0;
    ANSELD = 0;
    
    //set up Analog Inputs
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;

     
    //Datenrichtung festlegen:

    //Input Capture (Input)
    TRISBbits.TRISB12 = 1;      //rudd
    TRISBbits.TRISB13 = 1;      //elev
    TRISBbits.TRISB14 = 1;      //aile
    TRISBbits.TRISB15 = 1;      //thro
    TRISBbits.TRISB10 = 1;      //Aux1
    TRISBbits.TRISB11 = 1;      //gear

    //PWM (Output)
    TRISEbits.TRISE0 = 0;
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE2 = 0;
    TRISEbits.TRISE3 = 0;
    
    //LED
    TRISBbits.TRISB2 = 0;  //Pin B2
    TRISBbits.TRISB3 = 0;  //Pin B3
    TRISBbits.TRISB4 = 0;  //Pin B4
    TRISBbits.TRISB5 = 0;  //Pin B5
    TRISEbits.TRISE8 = 0;  //Pin E8
    TRISEbits.TRISE9 = 0;  //Pin E9

    //UART
    TRISFbits.TRISF4 = 1;       //RX 1
    TRISFbits.TRISF5 = 0;       //TX 1

    TRISDbits.TRISD14 = 1;      //RX 2
    TRISDbits.TRISD15 = 0;      //TX 2

    TRISFbits.TRISF13 = 1;      //RX 3
    TRISFbits.TRISF12 = 0;      //TX 3

    
    //GPIO
    TRISEbits.TRISE6 = 0;       //Pin to measure the time for while loops Pass

    //LCD
    TRISFbits.TRISF3 = 0; //RS
    TRISFbits.TRISF2 = 0; // R/W
    TRISFbits.TRISF8 = 0; //E
    TRISAbits.TRISA5 = 0; // 4
    TRISAbits.TRISA14 = 0; // 5
    TRISAbits.TRISA15 = 0; // 6
    TRISDbits.TRISD11 = 0; // 7

    // I2C Ultrasonic
    I2C2CONbits.I2CEN = 0; //disable I2C driver... use port directly in eeprom.c
    

    //CM2CONbits.CON = 1; //enable comparator channel 2
    

    //done on the Main.  _FPOR( ALTI2C1_ON & ALTI2C2_ON);    // Configure SPI pins
    
    

}
void remap_Pins (void)
{
    // Configure Input Functions
    
    //Remappable Pins, Input Capture
    RPINR7bits.IC1R = 47;   //thro
    RPINR7bits.IC2R = 46;   //aile
    RPINR8bits.IC3R = 45;   //elev
    RPINR8bits.IC4R = 44;   //rudd
    RPINR9bits.IC5R = 42;   //Aux1
    RPINR9bits.IC6R = 43;   //gear

    RPINR18bits.U1RXR = 100;    //UART1 RX an RP100
    RPINR19bits.U2RXR = 78;     //UART1 RX an RPI78
    RPINR27bits.U3RXR = 109;    //UART3 RX an RP109

    
    
    // Configure Output Functions
    
    //Remappable Pins, UART
    RPOR9bits.RP101R = 0b000001;       //UART1 TX an RP101   0b000001
    RPOR4bits.RP79R = 0b000011;        //UART2 TX an RP79
    RPOR11bits.RP108R = 0b011011;      //UART3 TX an RP108   0b011011

    
   // SPI-SD
    RPOR7bits.RP97R = 0b000111;         //RP97 is SPI Slave select
    RPOR12bits.RP112R = 0b000110;       //RP112 tied to SPI1 Clock Output
    RPOR13bits.RP113R = 0b000101;       //RP113 tied to SPI1 Data Output
    RPINR20bits.SDI1R = 122;            //RP122 tied to SDI1 Data Input

}

/* Assign U1Rx To Pin RP16
RPINR18bits.U1RXR = 0x10;
// Assign U1CTS To Pin RP17
RPINR18bits.U1CTSR = 0x11;
*/
/* Assign U1Tx To Pin RP34
RPOR1bits.RP34 = 1;
// Assign U1RTS To Pin RP35

RPOR1bits.RP35 = 2;
*/


//S. 15 IO Ports!!!!

