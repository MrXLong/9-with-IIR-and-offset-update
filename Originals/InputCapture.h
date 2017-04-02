/* 
 * File:   InputCapture.h
 * Author: Simon
 *
 * Created on December 30, 2014, 11:23 AM
 */

#ifndef INPUTCAPTURE_H
#define	INPUTCAPTURE_H

#include "p33EP512MU810.h"
#include <stdint.h>        //Includes uint16_t definition

//Input Capture ISR-Variablen

extern unsigned int cap1_1;
extern unsigned int cap1_2;
extern unsigned int cap2_1;
extern unsigned int cap2_2;
extern unsigned int cap3_1;
extern unsigned int cap3_2;
extern unsigned int cap4_1;
extern unsigned int cap4_2;

extern int isr_thr;
extern int isr_ail;
extern int isr_ele;
extern int isr_rud;

extern volatile int ic_thr;
extern volatile int ic_ail;
extern volatile int ic_ele;
extern volatile int ic_rud;

extern int trash;

typedef struct
{
    int throCap;
    int aileCap;
    int elevCap;
    int ruddCap;
    volatile int thro;
    volatile int aile;
    volatile int elev;
    volatile int rudd;
    int split_thro;
    int split_aile;     //Not Used
    int split_elev;     //Not Used
    int split_rudd;     //Not Used
    int test_rudd_rate;
}pw;

extern pw pulsewidth;
extern pw *ptr_pulsewidth;

void getPWM (pw *);

void set_test (void);
void InputCap1_Thro (void);
void InputCap2_Aile (void);
void InputCap3_Elev (void);
void InputCap4_Rudd (void);

extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC2Interrupt(void);
extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC1Interrupt(void);

extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC3Interrupt(void);
extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC4Interrupt(void);

#endif	/* INPUTCAPTURE_H */

