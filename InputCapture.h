/* 
 * File:   InputCapture.h
 * Author: Simon
 *
 * Created on December 30, 2014, 11:23 AM
 */

#ifndef INPUTCAPTURE_H
#define	INPUTCAPTURE_H


//Input Capture ISR-Variablen

extern unsigned int cap1_1;
extern unsigned int cap1_2;
extern unsigned int cap2_1;
extern unsigned int cap2_2;
extern unsigned int cap3_1;
extern unsigned int cap3_2;
extern unsigned int cap4_1;
extern unsigned int cap4_2;
extern unsigned int cap5_1;
extern unsigned int cap5_2;
extern unsigned int cap6_1;
extern unsigned int cap6_2;

extern int isr_thr;
extern int isr_ail;
extern int isr_ele;
extern int isr_rud;
extern int isr_aux1;
extern int isr_gear;

extern volatile int ic_thr;
extern volatile int ic_ail;
extern volatile int ic_ele;
extern volatile int ic_rud;
extern volatile int ic_aux1;
extern volatile int ic_gear;

extern int trash;

typedef struct
{
    int throCap;
    int aileCap;
    int elevCap;
    int ruddCap;
    int aux1Cap;
    int gearCap;
    volatile unsigned int thro;
    volatile int aile;
    volatile int elev;
    volatile int rudd;
    volatile int aux1;
    volatile int gear;
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
void InputCap5_Aux1 (void);
void InputCap6_Gear (void);
void InitInputCapture(void);

extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC2Interrupt(void);
extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC1Interrupt(void);

extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC3Interrupt(void);
extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC4Interrupt(void);

extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC5Interrupt(void);
extern void __attribute__ ((__interrupt__, no_auto_psv)) _IC6Interrupt(void);

#endif	/* INPUTCAPTURE_H */
