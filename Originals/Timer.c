#include "Timer.h"
#include <xc.h>
#include "p33EP512MU810.h"
#define FCY 40000000ULL
#include <stdlib.h>
#include <stdint.h>
#include "stdio.h"

#include "Pins.h"
#include "Oszillator.h"
#include "Interrupts.h"

#include "Mixer.h"
#include "PID.h"
#include "UM7.h"

    //				Fcy
    //  Schritte × Prescaler = -----
    //				F		

void init_timers (void)
{
    //Timer 1, Synchronisation der Input Capture Module
    T1CONbits.TON = 0;                  // Disable Timer
    T1CONbits.TCS = 0;                  // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;                // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b11;             // Select 1:256 Prescaler
    TMR1 = 0x00;                        // Clear timer register
    PR1 = 65000;                        // Load the period value, Timer Überlauf alle 1ms
    IPC0bits.T1IP = 2;                  // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;                  // Clear Timer 1 Interrupt Flag
    T1CONbits.TON = 1;                  // Start Timer


	//Timer 2, Input Capture 1 (Gas)
	T2CONbits.TON = 0;			//Disable Timer2
	T2CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
	T2CONbits.TGATE = 0;                    //Disable Gated Timer mode
	T2CONbits.TCKPS = 0b01;                 //Select 1:8 Prescaler
	TMR2 = 0x00;				//Clear timer2 register
	PR2 = 40000;				//Period Value = 6ms (30000 Schritte); Auflösung = 0,2µs/Schritt
	IPC1bits.T2IP = 3;                      //Set Timer2 Interrupt Priority Level = 3
	IFS0bits.T2IF = 0;			//Clear Timer2 Interrupt Flag
	T2CONbits.TON = 1;			//Start Timer2

    //Timer 3, Input Capture 2 (Rollen)
    T3CONbits.TON = 0;			//Disable Timer3
    T3CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
    T3CONbits.TGATE = 0;		//Disable Gated Timer mode
    T3CONbits.TCKPS = 0b01;             //Select 1:8 Prescaler
    TMR3 = 0x00;			//Clear timer2 register
    PR3 = 40000;			//Period Value = 6ms (30000 Schritte); Auflösung = 0,2µs/Schritt
    IPC2bits.T3IP = 6;                  //Set Timer3 Interrupt Priority Level = 3
    IFS0bits.T3IF = 0;			//Clear Timer3 Interrupt Flag
    T3CONbits.TON = 1;			//Start Timer3

	//Timer 4, Input Capture 3 (Nicken)
	T4CONbits.TON = 0;			//Disable Timer4
	T4CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
	T4CONbits.TGATE = 0;                    //Disable Gated Timer mode
	T4CONbits.TCKPS = 0b01;                 //Select 1:8 Prescaler
	TMR4 = 0x00;				//Clear timer4 register
	PR4 = 40000;				//Period Value = 6ms (30000 Schritte); Auflösung = 0,2µs/Schritt
	IPC6bits.T4IP = 4;                      //Set Timer4 Interrupt Priority Level = 3
	IFS1bits.T4IF = 0;			//Clear Timer4 Interrupt Flag
	T4CONbits.TON = 1;			//Start Timer4
	
    //Timer 5, Input Capture 4 (Gieren)
    T5CONbits.TON = 0;			//Disable Timer5
    T5CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
    T5CONbits.TGATE = 0;		//Disable Gated Timer mode
    T5CONbits.TCKPS = 0b01;             //Select 1:8 Prescaler
    TMR5 = 0x00;			//Clear timer5 register
    PR5 = 40000;			//Period Value = 6ms (30000 Schritte); Auflösung = 0,2µs/Schritt
    IPC7bits.T5IP = 4;                  //Set Timer5 Interrupt Priority Level = 3
    IFS1bits.T5IF = 0;			//Clear Timer5 Interrupt Flag
    T5CONbits.TON = 1;			//Start Timer5

        //Timer 6, Überwachung Sensordaten-Empfang
        T6CONbits.TON = 0;			//Disable Timer6
	T6CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
	T6CONbits.TGATE = 0;                    //Disable Gated Timer mode
	T6CONbits.TCKPS = 0b10;                 //Select 1:64 Prescaler
	TMR6 = 0x00;				//Clear timer6 register
	PR6 = 62500;				//Auflösung = 1,6ms/Schritt, Abtastzeit: 100ms
	IPC11bits.T6IP = 2;                     //Set Timer6 Interrupt Priority Level = 2
	IFS2bits.T6IF = 0;			//Clear Timer6 Interrupt Flag
	T6CONbits.TON = 0;			//Start Timer6
}

/*
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
	IFS0bits.T2IF = 0; //Clear Timer2 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
	IFS0bits.T2IF = 0; //Clear Timer2 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)
{
	IFS0bits.T3IF = 0; //Clear Timer3 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void)
{
	IFS1bits.T4IF = 0; //Clear Timer4 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void)
{
	IFS1bits.T5IF = 0; //Clear Timer5 interrupt flag
}
*/

void __attribute__((__interrupt__, no_auto_psv)) _T6Interrupt(void)
{
    watch = 1;
    //detect += 1;
    IFS2bits.T6IF = 0;                      //Clear Timer6 interrupt flag
}