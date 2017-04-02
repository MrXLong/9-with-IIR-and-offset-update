/*
 * File:   Mixer.h
 * Author: Simon
 *
 * Created on January 5, 2015, 6:59 PM
 */

#ifndef MIXER_H
#define	MIXER_H

#include "UM7.h"    //necessary to dereference UM7DataSensor
#include "InputCapture.h"   //necessary to dereference pw
#include "PID.h"      //necessary to dereference s_controller_fixedpoint

// representative values for the stick-neutral position (roll, pitch, yaw = 50%) of setpoint
#define neutral 7500
#define offset_aile 7531
#define offset_elev 7530
#define offset_rudd 7531

// +/- 2 View to neutral (7500) as the neutral range
#define neutral_limit_rudd 1004     // neutral_limit_rudd = ratio_setValue_rudd * 512 * 2 + 1 (for Q9 format neutral range)
#define neutral_limit_aile 656      // neutral_limit_aile = ratio_setValue_aile * 512 *2 +1
#define neutral_limit_elev 656

// Factors for the influence of the controller manipulated variables on the motors (Faktor = 1/ratio_x)
#define ratio_aile 4
#define ratio_elev 4
#define ratio_rudd 2
#define ratio_rudd_up 1.9           // positive Stellgröße vergrößern (more lift)
#define ratio_rudd_down 2.1         // negative Stellgröße verkleinern (less downforce)

// Factors of the influence of the set values to the regulation (Faktor = ratio_Sollwert_x)
#define ratio_setValue_aile 0.62
#define ratio_setValue_elev 0.62
#define ratio_setValue_rudd 2.0

extern unsigned int thro[4];
extern int aile[4];
extern int elev[4];
extern int rudd[4];

extern volatile int thr_mix;
extern volatile int ail_mix;
extern volatile int ele_mix;
extern volatile int rud_mix;

extern int gather;
extern volatile int yaw_rate;

extern int Switch;

typedef struct
{
    int thro;
    int aile;
    int elev;
    int rudd;
}manualMix;

extern manualMix PWM;
extern manualMix *ptr_PWM;

typedef struct
{
    int m1;
    int m2;
    int m3;
    int m4;
}mix; //Input for the PWM output

extern mix MIX;
extern mix *ptr_MIX;

void setValue (pw *);
void setValueGas (pw *ptr_PW);
void actualValue_aile_elev (UM7DataSensor *);
void actualValue_rudd (UM7DataSensor *, s_controller_fixedpoint *);
void throMix (s_controller_fixedpoint *);
void aileMix (s_controller_fixedpoint *);
void elevMix (s_controller_fixedpoint *);
void ruddMix (s_controller_fixedpoint *);
void motorMix (unsigned int thr[], int ail[], int ele[], int rud[]);
void setValueEmergency (pw *);
void emergencyMix (manualMix *);
long  map(long x, long in_min, long in_max, long out_min, long out_max);
void main_Switch (pw *ptr_PW);

_Q16 _Q16ftoi(float f);

void compensateYaw(int offset);

#endif	/* MIXER_H */
