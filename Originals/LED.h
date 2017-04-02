/* 
 * File:   LED.h
 * Author: Simon
 *
 * Created on 28. Januar 2015, 15:42
 */

#ifndef LED_H
#define	LED_H

#include "p33EP512MU810.h"
#include <stdlib.h>
#include <stdint.h>        //Includes uint16_t definition
#include "stdio.h"
#include "Pins.h"
#include "UM7.h"
#include "Mixer.h"

void rollLED (void);
void pitchLED (void);
void startupLED (void);
void errorLED (void);
void rdy2goLED (void);
void setValueTestLED (pw *, UM7DataSensor *);
void euler_test_LED (pw *, UM7DataSensor *);

#endif	/* LED_H */

