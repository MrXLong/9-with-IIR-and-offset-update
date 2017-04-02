/* 
 * File:   PWM.h
 * Author: Simon
 *
 * Created on December 18, 2014, 1:11 PM
 */


#ifndef PWM_H
#define	PWM_H

#include "p33EP512MU810.h"
#include "Mixer.h"
#include "PID.h"
#include "UM7.h"
#include "InputCapture.h"

void wait (pw *);
void initPWMcenter (void);
void intitPWMedge (void);
//void setPWM (struct pwm *);
void PWM_out (mix *);
//int stop (void);

#endif	/* PWM_H */

