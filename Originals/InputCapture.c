
#include "InputCapture.h"
#include <xc.h>
#include "p33EP512MU810.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#define FCY 40000000ULL
#include "Pins.h"
#include "Oszillator.h"
#include "Interrupts.h"
#include "Timer.h"
#include "PWM.h"

// Variablen zum Speichern der Buffer-Werte
unsigned int cap1_1, cap1_2, cap2_1, cap2_2, cap3_1, cap3_2, cap4_1, cap4_2 = 0;

// Variable to indicate whether the first or second interrupt cycle (rising or falling edge)
int isr_thr = 0;
int isr_ail = 0;
int isr_ele = 0;
int isr_rud = 0;

// Variablen zum Anzeigen, dass ein neuer Pulsweiten-Wert (Sollwert) ansteht
volatile int ic_thr = 0;
volatile int ic_ail = 0;
volatile int ic_ele = 0;
volatile int ic_rud = 0;

int trash = 0;

pw pulsewidth;
pw *ptr_pulsewidth = &pulsewidth;


//Initialisierung Input Capture 1-4

void InputCap1_Thro (void)
{
    IFS0bits.IC1IF = 0;             // Clear IC1 interrupt status flag
    IEC0bits.IC1IE = 1;             // Enable IC1 interrupts
    IPC0bits.IC1IP = 3;             // Set module interrupt priority = 2
    IC1CON1bits.ICSIDL = 0;         // Input capture will continue to operate in CPU idle mode
    IC1CON1bits.ICTSEL = 0b001;     // T2CKL is clock source for IC1
    IC1CON1bits.ICM = 0b001;        // Capture mode; every falling and rising edge (ICI wird hier nicht benutzt)
    IC1CON2bits.IC32 = 0;           // Cascade module operation is disabled
    IC1CON2bits.ICTRIG = 0;         // Input source used to synchronize the input capture timer to a timer of
                                    // another module (Synchronization mode)
    IC1CON2bits.TRIGSTAT = 0;       // IC1TMR has not been triggered and is being held clear
    IC1CON2bits.SYNCSEL = 0b01011;  // Timer1 synchronizes IC1
}

void InputCap2_Aile (void)
{
    IFS0bits.IC2IF = 0;             // Clear IC2 interrupt status flag
    IEC0bits.IC2IE = 1;             // Enable IC2 interrupts
    IPC1bits.IC2IP = 6;             // Set module interrupt priority = 2
    IC2CON1bits.ICSIDL = 0;         // Input capture will continue to operate in CPU idle mode
    IC2CON1bits.ICTSEL = 0b000;     // T3CKL is clock source for IC2
    IC2CON1bits.ICM = 0b001;        // Capture mode; every falling and rising edge (ICI wird hier nicht benutzt)
    IC2CON2bits.IC32 = 0;           // Cascade module operation is disabled
    IC2CON2bits.ICTRIG = 0;         // Input source used to synchronize the input capture timer to a timer of
                                    // another module (Synchronization mode)
    IC2CON2bits.TRIGSTAT = 0;       // IC2TMR has not been triggered and is being held clear
    IC2CON2bits.SYNCSEL = 0b01011;  // Timer1 synchronizes IC2
}

void InputCap3_Elev (void)
{
    IFS2bits.IC3IF = 0;             // Clear IC3 interrupt status flag
    IEC2bits.IC3IE = 1;             // Enable IC3 interrupts
    IPC9bits.IC3IP = 5;             // Set module interrupt priority = 2
    IC3CON1bits.ICSIDL = 0;         // Input capture will continue to operate in CPU idle mode
    IC3CON1bits.ICTSEL = 0b010;     // T4CKL is clock source for IC3
    IC3CON1bits.ICM = 0b001;        // Capture mode; every falling and rising edge (ICI wird hier nicht benutzt)
    IC3CON2bits.IC32 = 0;           // Cascade module operation is disabled
    IC3CON2bits.ICTRIG = 0;         // Input source used to synchronize the input capture timer to a timer of
                                    // another module (Synchronization mode)
    IC3CON2bits.TRIGSTAT = 0;       // IC3TMR has not been triggered and is being held clear
    IC3CON2bits.SYNCSEL = 0b01011;  // Timer1 synchronizes IC3
}

void InputCap4_Rudd (void)
{
    IFS2bits.IC4IF = 0;             // Clear IC4 interrupt status flag
    IEC2bits.IC4IE = 1;             // Enable IC4 interrupts
    IPC9bits.IC4IP = 4;             // Set module interrupt priority = 2
    IC4CON1bits.ICSIDL = 0;         // Input capture will continue to operate in CPU idle mode
    IC4CON1bits.ICTSEL = 0b011;     // T5CKL is clock source for IC4
    IC4CON1bits.ICM = 0b001;        // Capture mode; every falling and rising edge (ICI wird hier nicht benutzt)
    IC4CON2bits.IC32 = 0;           // Cascade module operation is disabled
    IC4CON2bits.ICTRIG = 0;         // Input source used to synchronize the input capture timer to a timer of
                                    // another module (Synchronization mode)
    IC4CON2bits.TRIGSTAT = 0;       // IC4TMR has not been triggered and is being held clear
    IC4CON2bits.SYNCSEL = 0b01011;  // Timer1 synchronizes IC4
}

// Input Capture ISR 1-4

void __attribute__ ((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{
    ic_thr = 0;
    isr_thr += 1;

	if (isr_thr==1)					// Interrupt steigende Flanke
	{
            T2CONbits.TON = 1;                          // Start Timer2
            IFS0bits.IC1IF = 0;                         // Reset interrupt flag
            cap1_1 = IC1BUF;
	}

	if (isr_thr==2)					// Interrupt fallende Flanke
	{
            cap1_2 = IC1BUF;
            ptr_pulsewidth->throCap = cap1_2 - cap1_1;  // ermittelter Wert in Zwischenvaraible schreiben

            if ((ptr_pulsewidth->throCap > 9500) || (ptr_pulsewidth->throCap < 5420))   // wenn ermittelter Wert über 100% oder unter 0% liegt: Wert verwerfen
            {                                                                           // und Berechnungs-Schleife nicht freigeben und Interrupt auf 0 setzen
                //trash = IC1BUF;                         // prophylaktisches Leeren des Buffers
                T2CONbits.TON = 0;			// Stop Timer2
                IFS0bits.IC1IF = 0;                     // Reset interrupt flag
                TMR2 = 0x00;				// Clear timer2 register
                isr_thr = 0;
                ic_thr = 0;
            }

            else                                        // ansonten Wert freigeben und entsprechendes Flag (neuer Wert) setzen
            {
                ptr_pulsewidth->thro = ptr_pulsewidth->throCap;
                T2CONbits.TON = 0;                      // Stop Timer2
                IFS0bits.IC1IF = 0;                     // Reset interrupt flag
                TMR2 = 0x00;				// Clear timer2 register
                isr_thr = 0;				// Zähler zurücksetzen
                ic_thr = 1;                             // neuer Wert steht an
            }
        }
 }

void __attribute__ ((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{
    ic_ail = 0;
    isr_ail += 1;

	if (isr_ail==1)					// Interrupt steigende Flanke
	{
            T3CONbits.TON = 1;                          // Start Timer3
            cap2_1 = IC2BUF;
            IFS0bits.IC2IF = 0;                         // Reset interrupt flag
	}

	if (isr_ail==2)					// Interrupt fallende Flanke
	{
            cap2_2 = IC2BUF;
            ptr_pulsewidth->aileCap = cap2_2 - cap2_1;  // ermittelter Wert in Zwischenvaraible schreiben

            if ((ptr_pulsewidth->aileCap > 9500) || (ptr_pulsewidth->aileCap < 5420))   // wenn ermittelter Wert über 100% oder unter 0% liegt: Wert verwerfen
            {                                                                           // und Berechnungs-Schleife nicht freigeben und Interrupt auf 0 setzen
                //trash = IC2BUF;                         // prophylaktisches Leeren des Buffers
                T3CONbits.TON = 0;			// Stop Timer3
                IFS0bits.IC2IF = 0;                     // Reset interrupt flag
                TMR3 = 0x00;				// Clear timer2 register
                isr_ail = 0;
                ic_ail = 0;
            }

            else
            {
                ptr_pulsewidth->aile = ptr_pulsewidth->aileCap;
                T3CONbits.TON = 0;                      // Stop Timer3
                IFS0bits.IC2IF = 0;                     // Reset interrupt flag
                TMR3 = 0x00;				// Clear timer2 register
                isr_ail = 0;				// Zähler zurücksetzen
                ic_ail = 1;                             // neuer Wert steht an
            }
        }
}

void __attribute__ ((__interrupt__, no_auto_psv)) _IC3Interrupt(void)
{
     ic_ele = 0;
     isr_ele += 1;

	if (isr_ele==1)					// Interrupt steigende Flanke
	{
            T4CONbits.TON = 1;                          // Start Timer4
            cap3_1 = IC3BUF;
            IFS2bits.IC3IF = 0;                         // Reset interrupt flag
	}

	if (isr_ele==2)					// Interrupt fallende Flanke
	{
            cap3_2 = IC3BUF;
            ptr_pulsewidth->elevCap = cap3_2 - cap3_1;  // ermittelter Wert in Zwischenvaraible schreiben

            if ((ptr_pulsewidth->elevCap > 9500) || (ptr_pulsewidth->elevCap < 5420))   // wenn ermittelter Wert über 100% oder unter 0% liegt: Wert verwerfen
            {                                                                           // und Berechnungs-Schleife nicht freigeben und Interrupt auf 0 setzen
                //trash = IC3BUF;                         // prophylaktisches Leeren des Buffers
                T4CONbits.TON = 0;			// Stop Timer4
                IFS2bits.IC3IF = 0;                     // Reset interrupt flag
                TMR4 = 0x00;				// Clear timer2 register
                isr_ele = 0;
                ic_ele = 0;
            }

            else
            {
                ptr_pulsewidth->elev = ptr_pulsewidth->elevCap;
                T4CONbits.TON = 0;                       // Stop Timer4
                IFS2bits.IC3IF = 0;                      // Reset interrupt flag
                TMR4 = 0x00;                             // Clear timer4 register
                isr_ele = 0;                             // Zähler zurücksetzen
                ic_ele = 1;                              // neuer Wert steht an
            }
        }
}


void __attribute__ ((__interrupt__, no_auto_psv)) _IC4Interrupt(void)
{
    ic_rud = 0;
    isr_rud += 1;

        if (isr_rud==1)					// Interrupt steigende Flanke
        {
            T5CONbits.TON = 1;
            cap4_1 = IC4BUF;
            IFS2bits.IC4IF = 0;                         // Reset interrupt flag
        }

	if (isr_rud==2)					// Interrupt fallende Flanke
	{
            cap4_2 = IC4BUF;
            ptr_pulsewidth->ruddCap = cap4_2 - cap4_1;  // ermittelter Wert in Zwischenvaraible schreiben

            if ((ptr_pulsewidth->ruddCap > 9500) || (ptr_pulsewidth->ruddCap < 5420))   // wenn ermittelter Wert über 100% oder unter 0% liegt: Wert verwerfen
            {                                                                           // und Berechnungs-Schleife nicht freigeben und Interrupt auf 0 setzen
                //trash = IC4BUF;                         // prophylaktisches Leeren des Buffers
                T5CONbits.TON = 0;			// Stop Timer5
                IFS2bits.IC4IF = 0;                     // Reset interrupt flag
                TMR5 = 0x00;				// Clear timer2 register
                isr_rud = 0;
                ic_rud = 0;
            }

            else
            {
                ptr_pulsewidth->rudd = ptr_pulsewidth->ruddCap;
                T5CONbits.TON = 0;                      // Stop Timer5
                IFS2bits.IC4IF = 0;                     // Reset interrupt flag
                TMR5 = 0x00;                            // Clear timer4 register
                isr_rud = 0;                            // Zähler zurücksetzen
                ic_rud = 1;                             // neuer Wert steht an
            }
	}
}