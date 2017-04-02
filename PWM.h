/* 
 * File:   PWM.h
 * Author: Simon
 *
 * Created on December 18, 2014, 1:11 PM
 */

#ifndef PWM_H
#define	PWM_H

#include "Mixer.h"      //necessary to dereference mix
#include "InputCapture.h"       //necessary to dereference pw
#include "defines.h"

//void armed (pw *);
void stickConfig(pw *);
//int disarmed (pw *);
void initPWMcenter (void);
void intitPWMedge (void);
//void setPWM (struct pwm *);
void PWM_out (mix *);
//int stop (void);

boolean checkTransmitter();

#endif	/* PWM_H */

