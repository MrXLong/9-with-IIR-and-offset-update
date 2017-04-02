/* 
 * File:   Timer.h
 * Author: Simon
 *
 * Created on December 23, 2014, 4:36 PM
 */

#ifndef TIMER_H
#define	TIMER_H

#include "defines.h"

#define FCY 40000000ULL

//CPU LOAD TAKEN FROM MATRIX PILOT
#define CPU_LOAD_PERCENT (6553600/((FCY)/4096))
//static uint16_t _cpu_timer = 0;
uint8_t udb_cpu_load(void);

typedef struct 
{
    unsigned int ms;
    unsigned int s;
    unsigned int min;
    unsigned int ctr;
}Timer;

extern Timer timer;

//counter for each periodic function
//should be better way to do this
extern unsigned int telemetry_ctr;
extern unsigned int autopilot_ctr;

void init_timers(void);

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T6Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T7Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T8Interrupt(void);

extern volatile int store_data;
extern int detect;

extern volatile int watch;                                                     // Flag for defective sensor communication and emergency call program (Timer 6)
extern volatile int fly_var;
extern int GPSlocked;

extern unsigned int motor1_led,motor2_led,motor3_led,motor4_led;

extern boolean tx_watchdog_enabled;

#endif	/* TIMER_H */

