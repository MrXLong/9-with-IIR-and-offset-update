/* 
 * File:   Timer.h
 * Author: Simon
 *
 * Created on December 23, 2014, 4:36 PM
 */

#ifndef TIMER_H
#define	TIMER_H

#include "PID.h"
#include "UM7.h"
#include "InputCapture.h"
#include <libpic30.h>
#include "Interrupts.h"
#include <stdlib.h>
#include <stdint.h>        //Includes uint16_t definition
#include "stdio.h"
#include "Pins.h"
#include "Oszillator.h"
#include <xc.h>
#include "p33EP512MU810.h"
#define FCY 40000000ULL
#include "libq.h"

void init_timers(void);

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void);
void __attribute__((__interrupt__, no_auto_psv)) _T6Interrupt(void);

extern volatile int watch;
extern int detect;

#endif	/* TIMER_H */

