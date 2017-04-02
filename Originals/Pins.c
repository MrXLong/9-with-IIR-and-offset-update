#include "Pins.h"
#include "p33EP512MU810.h"
#include "stdio.h"

void init_Pins (void)
{
    //Pin & Port Konfiguration

    //Ports B,E,F = digital
    ANSELB = 0;
    ANSELBbits.ANSB12 = 0;
    ANSELBbits.ANSB13 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB15 = 0;
    ANSELE = 0;
    ANSELD = 0;
    
    
    //Datenrichtung festlegen:

    //Input Capture (Input)
    TRISBbits.TRISB12 = 1;
    TRISBbits.TRISB13 = 1;
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB15 = 1;

    //PWM (Output)
    TRISEbits.TRISE0 = 0;
    TRISEbits.TRISE1 = 0;
    TRISEbits.TRISE2 = 0;
    TRISEbits.TRISE3 = 0;


    //LED
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISBbits.TRISB5 = 0;
    TRISEbits.TRISE8 = 0;
    TRISEbits.TRISE8 = 0;

    //UART
    TRISFbits.TRISF4 = 1;       //RX 1
    TRISFbits.TRISF5 = 0;       //TX 1

    TRISDbits.TRISD14 = 1;      //RX 2
    TRISDbits.TRISD15 = 0;      //TX 2

    //GPIO
    TRISEbits.TRISE6 = 0;       //Pin zum Messen der Zeit für while-Schleifen-Durchlauf
}
void remap_Pins (void)
{
    // Configure Input Functions
    //Remappable Pins, Input Capture

    RPINR7bits.IC1R = 47;
    RPINR7bits.IC2R = 46;
    RPINR8bits.IC3R = 45;
    RPINR8bits.IC4R = 44;

    RPINR18bits.U1RXR = 100;    //UART1 RX an RP100
    RPINR19bits.U2RXR = 78;     //UART1 RX an RPI78

    // Configure Output Functions
    //Remappable Pins, UART
    
    RPOR9bits.RP101R = 1;       //UART1 TX an RP101
    RPOR4bits.RP79R = 1;        //UART2 TX an RP79

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