#include "Mixer.h"

// Initializing the array for mixer
int thro [4] = {0, 0, 0, 0};
int aile [4] = {0, 0, 0, 0};
int elev [4] = {0, 0, 0, 0};
int rudd [4] = {0, 0, 0, 0};

// Initialization of variables to display a made mixing current values
volatile int thr_mix = 0;
volatile int ail_mix = 0;
volatile int ele_mix = 0;
volatile int rud_mix = 0;

int gather = 0;
volatile int yaw_rate = 0;

// initializations structures
manualMix PWM;
manualMix *ptr_PWM = &PWM;

mix MIX;
mix *ptr_MIX = &MIX;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------- setpoints (Input Capture) -- Data acquire-------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setValue (pw *ptr_PW)
{
    //Deviation of actual throttle position to neutral position, in Q9 format transformed with factorization for influence (ratio_setpoint_x)
    ptr_FIX->setpoint_angle_aile = (ptr_PW->aile - offset_aile) * _Q16ftoi(ratio_setValue_aile);
    ptr_FIX->setpoint_angle_elev = (ptr_PW->elev - neutral) * _Q16ftoi(ratio_setValue_elev);
    ptr_FIX->setpoint_rate_rudd   = (ptr_PW->rudd - offset_rudd) * _Q16ftoi(ratio_setValue_rudd);
    
    ptr_FIX->setpoint_angle_aile = ptr_FIX->setpoint_angle_aile >> 7;
    ptr_FIX->setpoint_angle_elev = ptr_FIX->setpoint_angle_elev >> 7;
    ptr_FIX->setpoint_rate_rudd   = ptr_FIX->setpoint_rate_rudd >> 7;
}

// actual values (UM7) ROLL AND PITCH ANGLE FROM THE UM7
void actualValue_aile_elev (UM7DataSensor *ptr_eulerAngle)
{
    ptr_FIX->actual_angle_aile = (ptr_eulerAngle->eulerRollAngle * _Q16ftoi(ratio_eulerAngle)); // Euler-roll-angle (actual value) is multiplied by 0.7326049 to 1: 1 to fit on setpoint
    ptr_FIX->actual_angle_elev = (ptr_eulerAngle->eulerPitchAngle * _Q16ftoi(ratio_eulerAngle));
    ptr_FIX->actual_angle_aile = ptr_FIX->actual_angle_aile >> 7;
    ptr_FIX->actual_angle_elev = ptr_FIX->actual_angle_elev >> 7;
}

// actual values (UM7) YAW ANGLE FROM THE UM7
void actualValue_rudd (UM7DataSensor *ptr_eulerAngle, s_controller_fixedpoint *ptr_fix)
{
    ptr_fix->actual_rate_rudd = ptr_eulerAngle->eulerYawRate * _Q16ftoi(ratio_eulerRate);
    ptr_fix->actual_rate_rudd = ptr_fix->actual_rate_rudd >> 7;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------- Set data in the Mixer Array -------------------------------------------//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// For manipulated variable "gas" mixer (gas is 1: 1 pass), for automatic mode and manual mode (emergency mode)
void throMix (pw *ptr_PW)
{
//    thro[0] = ptr_PW->thro;
    thro[1] = ptr_PW->thro;
//    thro[2] = ptr_PW->thro;
    thro[3] = ptr_PW->thro;

    thr_mix = 1;
}

// For manipulated variable mixer "rolling"
void aileMix (s_controller_fixedpoint *ptr_fix_aile)                             // left
{
    aile[0] = (int)((ptr_fix_aile->aile_output)/ratio_aile);           // roll: Motor1 faster
    aile[1] = (int)((ptr_fix_aile->aile_output)/(-ratio_aile));        // roll: Motor2 slower
    aile[2] = (int)((ptr_fix_aile->aile_output)/(-ratio_aile));        // roll: Motor3 slower
    aile[3] = (int)((ptr_fix_aile->aile_output)/ratio_aile);           // roll: Motor4 faster
    ail_mix = 1;
}

// For manipulated variable mixer "pitch"
void elevMix (s_controller_fixedpoint *ptr_fix_elev)                        // ahead (go front)
{
     elev[0] = (int)((ptr_fix_elev->elev_output)/(-ratio_elev));       // pitch: Motor1 slower
     elev[1] = (int)((ptr_fix_elev->elev_output)/(-ratio_elev));       // pitch: Motor2 slower
     elev[2] = (int)((ptr_fix_elev->elev_output)/ratio_elev);          // pitch: Motor3 faster
     elev[3] = (int)((ptr_fix_elev->elev_output)/ratio_elev);          // pitch: Motor4 faster
     ele_mix = 1;
}

// For manipulated variable mixer "yaw"
void ruddMix (s_controller_fixedpoint *ptr_fix)                             // left
{
    rudd[0] = (int)((ptr_fix->rudd_output)/(-ratio_rudd));             // yaw: Motor 1 slower
    rudd[1] = (int)((ptr_fix->rudd_output)/ratio_rudd);                // yaw: Motor 2 faster
    rudd[2] = (int)((ptr_fix->rudd_output)/(-ratio_rudd));             // yaw: Motor 3 slower
    rudd[3] = (int)((ptr_fix->rudd_output)/ratio_rudd);                // yaw: Motor 4 faster
    rud_mix = 1;
}

// Setpoints for manual mode (emergency mode)
void setValueEmergency (pw *ptr_PW)
{
    ptr_PWM->aile = (ptr_PW->aile - neutral);
    ptr_PWM->elev = (ptr_PW->elev - neutral);
    ptr_PWM->rudd = (ptr_PW->rudd - neutral);
}

// For "rolling", "Pitch" mixer, "yaw" for manual mode (emergency mode)
void emergencyMix (manualMix *ptr_pwm)
{
    aile[0] = (int)((ptr_pwm->aile)/4);
    aile[1] = (int)((ptr_pwm->aile)/(-4));
    aile[2] = (int)((ptr_pwm->aile)/(-4));
    aile[3] = (int)((ptr_pwm->aile)/4);

    elev[0] = (int)((ptr_pwm->elev)/(-4));
    elev[1] = (int)((ptr_pwm->elev)/(-4));
    elev[2] = (int)((ptr_pwm->elev)/4);
    elev[3] = (int)((ptr_pwm->elev)/4);

    rudd[0] = (int)((ptr_pwm->rudd)/(-2));
    rudd[1] = (int)((ptr_pwm->rudd)/2);
    rudd[2] = (int)((ptr_pwm->rudd)/(-2));
    rudd[3] = (int)((ptr_pwm->rudd)/2);

    ail_mix = 1;
    ele_mix = 1;
    rud_mix = 1;
}

// final motor mixer, here all 4 mixed manipulated variables are combined to form a control value per motor
void motorMix (int thr[], int ail[], int ele[], int rud[])
{
    ptr_MIX->m1 = (thr[0]+ ail[0] + ele[0] + rud[0]);
    if (ptr_MIX->m1 > 9500)     // PWM limit 1.9ms 9500 input capture value.
    {
        ptr_MIX->m1 = 9500;
    }
    ptr_MIX->m2 = (thr[1]+ ail[1] + ele[1] + rud[1]);
    if (ptr_MIX->m2 > 9500)     // PWM limit 1.9ms 9500 input capture value.
    {
        ptr_MIX->m2 = 9500;
    }
    ptr_MIX->m3 = (thr[2]+ ail[2] + ele[2] + rud[2]);
    if (ptr_MIX->m3 > 9500)     // PWM limit 1.9ms 9500 input capture value.
    {
        ptr_MIX->m3 = 9500;
    }
    ptr_MIX->m4 = (thr[3]+ ail[3] + ele[3] + rud[3]);
    if (ptr_MIX->m4 > 9500)     // PWM limit 1.9ms 9500 input capture value.
    {
        ptr_MIX->m4 = 9500;
    }

    //Resetting the flags for displaying new upcoming mixer values
    thr_mix = 0;
    ail_mix = 0;
    ele_mix = 0;
    rud_mix = 0;
}
