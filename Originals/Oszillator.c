# include "Oszillator.h"
# include "p33EP512MU810.h"

void initOscillator (void)
{
//Primary Oscillator (Posc)
//XT-Mode (3.5 MHZ - 10 MHz)

    //                      M                    40
    //  Fosc = FIN ×  ------------ = 8 MHz × --------- = 80 MHz
    //                   N1 × N2                2 × 2
    //                               PLLDIV + 2
    //         FIN ×  ---------------------------------------
    //                  ( PLLPRE + 2 ) × 2 ( PLLPOST + 1 )
	
	//Fcy=Fosc/2
	//PLL = phase-locked loop (Phasenregelschleife)

    //N1 = PLLPRE + 2
    //N2 = 2 x (PLLPOST + 1)
    //M = PLLDIV + 2
    //Fast RC Oscillator (internal clock source????????) = default oscillator mode
    //COSC = 0;
    PLLFBDbits.PLLDIV=38;                               //Divisor=38 -> M= 38+2 = 40
    CLKDIVbits.PLLPOST=0;				//Output divided by 2  -> N2=2
    CLKDIVbits.PLLPRE=0;				//Input divided by 2   -> N1=2
    CLKDIVbits.DOZE=0;                                  //Prozessor clock FCY divided by 2 (FCY=40MHz)
}




