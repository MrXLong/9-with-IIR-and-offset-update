#include "Interrupts.h"
#include "p33EP512MU810.h"

void enableInterrupts(void)
{
        //Enable Timerx interrupt = 1
    IEC0bits.T1IE = 0;
	IEC0bits.T2IE = 0;
	IEC0bits.T3IE = 0;
	IEC1bits.T4IE = 0;
	IEC1bits.T5IE = 0;
    IEC2bits.T6IE = 1;
    IEC3bits.T7IE = 1;
    IEC3bits.T8IE = 1;
    IEC3bits.T9IE = 1;
	INTCON2bits.GIE = 1;                    //Global Interrupt Enable
        
	return;
}
