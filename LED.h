/* 
 * File:   LED.h
 * Author: Simon
 *
 * Created on 28. Januar 2015, 15:42
 */

#ifndef LED_H
#define	LED_H

#include "UM7.h"        //necessary to dereference UM7DataSensor
#include "InputCapture.h"   //necessary to dereference pw


void rollLED (void);
void pitchLED (void);
void startupLED (void);
void errorLED (void);
void rdy2goLED (void);
void setValueTestLED (pw *, UM7DataSensor *);
void euler_test_LED (pw *, UM7DataSensor *);
void pitch(void);
void yaw(void);
void aux1(void);

#endif	/* LED_H */

