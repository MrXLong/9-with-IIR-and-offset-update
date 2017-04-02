/* 
 * File:   PID.h
 * Author: Simon
 *
 * Created on December 30, 2014, 2:16 PM
 */

#ifndef PID_H
#define	PID_H

#include <p33EP512MU810.h>
#include <stdio.h>
#include "libq.h"
#include <dsp.h>
#include "UM7.h"


// Struktur für den Fixedpoint-Regler "aile" und "elev"
typedef struct
{
    // Variables for Roll (aile) (phi)
    _Q16 Kp_aile;                          //_Q16 required 32 Bit, _Q15 required 16 Bit
    _Q16 Ki_aile;
    _Q16 Kd_aile;
    _Q16 aile_dt;
    _Q16 setpoint_angle_aile;          //setpoint_angle
    _Q16 actual_angle_aile;           //actual_angle
    _Q16 aile_Max;                   //for current saturation
    _Q16 aile_pre_error;
    _Q16 aile_integral;
    _Q16 aile_error;
    _Q16 aile_derivative;
    _Q16 aile_output;
    _Q16 aile_iMax;                 //Maximun value Integral part

    // Variables for Pitch(elev) (theta)
    _Q16 Kp_elev;                          //_Q16 required 32 Bit, _Q15 required 16 Bit
    _Q16 Ki_elev;
    _Q16 Kd_elev;
    _Q16 elev_dt;
    _Q16 setpoint_angle_elev;          //setpoint
    _Q16 actual_angle_elev;           //actual_angle
    _Q16 elev_Max;                   //for current saturation
    _Q16 elev_pre_error;
    _Q16 elev_integral;
    _Q16 elev_error;
    _Q16 elev_derivative;
    _Q16 elev_output;
    _Q16 elev_iMax;                 //Maximun value Integral part

    // Variables for Yaw (rudd) (Psi)
    _Q16 Kp_rudd;                         //_Q16 required 32 Bit, _Q15 required 16 Bit
    _Q16 Ki_rudd;
    _Q16 Kd_rudd;
    _Q16 rudd_dt;
    _Q16 setpoint_rate_rudd;          //setpoint
    _Q16 actual_rate_rudd;           //actual_angle
    _Q16 rudd_Max;                   //for current saturation
    _Q16 rudd_pre_error;
    _Q16 rudd_integral;
    _Q16 rudd_error;
    _Q16 rudd_derivative;
    _Q16 rudd_output;
    _Q16 rudd_iMax;                 //Maximun value Integral part
    //

    _Q16 epsilon;               // if(abs(error) > epsilon){integral = integral + error*dt;}

}s_controller_fixedpoint;

extern s_controller_fixedpoint coeff_fixedpoint;
extern s_controller_fixedpoint *ptr_FIX;

// Structure for the controller specification
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float samp_time; //sample time
    float setpoint; //setpoint
    int manipulated_pwm;
}s_controller_specifications;

extern s_controller_specifications aile_elev;
extern s_controller_specifications rudder;

void set_PID_coefficients(void);
void get_PID_coefficients_fixedpoint(s_controller_fixedpoint* controller_fix, s_controller_specifications controller_specifications_aile_elev, s_controller_specifications controller_specifications_rudder);

void PIDcal_fixedpoint_aile(s_controller_fixedpoint* controller_fix);
void PIDcal_fixedpoint_elev(s_controller_fixedpoint* controller_fix);
void PIDcal_fixedpoint_rudd(s_controller_fixedpoint* controller_fix);

#endif	/* PID_H */
