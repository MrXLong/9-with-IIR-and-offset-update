/****************************************************************************
 * File:   PID.c
 * Created by: Alejandro Perez on 01.07.2015

 * Description: This file contains all methods for calculating PID control
 * outputs and setting any values associated with the PID controllers.
 * The file contains functions for calculating both the angle and rate PID
 * values for each axis of rotation on the Quadrokopter. 
 
 * for further reference visit:
 * http://blog.owenson.me/build-your-own-quadcopter-flight-controller/

 * Last Modified: 01.06.2016 (MW)
 
 * NOTE: All PID functions could be combined to a single PID function such
 * as the PID_position function created by MW. This would help simplify the file
 * and prevent superfluous calculations.
 * 
 * NOTE2: In the PID calculations, an 'if' case was added such that the integral
 * value is not calculated if the angle or rate is unchanged. 01.05.2016(MW)
****************************************************************************/
#include "PID.h"
#include "RS232.h"
#include "GPS.h"
#include <p33EP512MU810.h>
#include <stddef.h>
#include <string.h>
#include <libpic30.h>
#include "defines.h"
#include "gain_variables.h" //for global PID values
#include "options.h"    //defined gain values
#include "MAVLink.h"    //for debug to screen

//Switches for PID testing
#define PitchActive 1
#define RollActive  1
#define YawActive   1

//flight mode defined in PID.h

//global functions used by mavlink to change PID gains
float rollkp;
float rollki;
float rollkd;

float pitchkp;
float pitchki;
float pitchkd;

float yawkp;
float yawki;
float yawkd;

float hoverkp;
float hoverki;
float hoverkd;

int32_t nom_hov_thr; //this should not be here

int normalizeGains=0;   //global variable requires reset after multiplying_gains is called

//aile_elev, rudder, gas are all data structures of type
//controller_specification. They are global variables declared in main.
//In PID loops, data structure being used is controller_fixedpoint.
//the transition between these two structures is made in get_PID_coefficients
//this needs to be changed as we do not need both of these functions
void set_PID_coefficients(void)
{
    
    // PID-Coefficient aile & elev (rate)
    //aile_elev.Kp_r = 0.50;//0.55;//0.6;//0.55;  
    //aile_elev.Kp_r = 0.57;  //MW 26.07.2016
    //aile_elev.Ki_r = 0.48;   //MW 26.07.2016   
    //aile_elev.Kd_r = 0.007;
    //aile_elev.Kd_r = 0.005; //MW 26.07.2016
    aile_elev.Kp_r = 0.2;  //MW 26.07.2016
    aile_elev.Ki_r = 0.1;   //MW 26.07.2016   

    // PID-Coefficient rudder (rate)
    rudder.Kp_r = 0.2;//0.8;//2.7;
    rudder.Ki_r = 0.1;//2;//0.7;//1;
    rudder.Kd_r = 0;
    
    // PID-Coefficient aile & elev (angle)
    aile_elev.Kp_a = 1.5;  //MW 18.10.2016
    aile_elev.Ki_a = 0.2;
    aile_elev.Kd_a = 0;
    aile_elev.samp_time = 0.01;

    // PID-Coefficient rudder (angle)
    rudder.Kp_a = 0.8;//0.6;//0.7;
    rudder.Ki_a = 0.2;
    rudder.Kd_a = 0;
    rudder.samp_time = 0.01;
    

    // PID-Coefficient gas      (thr)
    //gas.Kp_g = 0.65;
    gas.Kp_g = 1;
    gas.Ki_g = 0;
    gas.Kd_g = 0;
    gas.samp_time = 0.01;
    
    //epsilon
    coeff_fixedpoint.epsilon = (1);

    //PID Limits
    coeff_fixedpoint.gas_Max = 9500;
    
    if(PitchActive)
    {
        coeff_fixedpoint.elev_Max_a = 45;    
        coeff_fixedpoint.elev_iMax_a = 25;
        
        if(AcroMode)
            coeff_fixedpoint.elev_Max_r = 200;
        else
            coeff_fixedpoint.elev_Max_r = 90;
        
        coeff_fixedpoint.elev_iMax_r = 25;
    }
    else
    {
        coeff_fixedpoint.elev_Max_a = 0;
        coeff_fixedpoint.elev_iMax_a = 0;
        coeff_fixedpoint.elev_Max_r = 0;
        coeff_fixedpoint.elev_iMax_r = 0;
    }
    
    if(RollActive)
    {
        coeff_fixedpoint.aile_Max_a = 45;
        coeff_fixedpoint.aile_iMax_a = 25; 
        coeff_fixedpoint.aile_Max_r = 90;
        coeff_fixedpoint.aile_iMax_r = 25;
    }
    else
    {
        coeff_fixedpoint.aile_Max_a = 0;
        coeff_fixedpoint.aile_iMax_a = 0;
        coeff_fixedpoint.aile_Max_r = 0;
        coeff_fixedpoint.aile_iMax_r = 0;
    }
    
    if(YawActive)
    {
        coeff_fixedpoint.rudd_Max_a = 150;
        coeff_fixedpoint.rudd_iMax_a = 25; 
        coeff_fixedpoint.rudd_Max_r = 300;
        coeff_fixedpoint.rudd_iMax_r = 25;    
    }
    else
    {        
        coeff_fixedpoint.rudd_Max_a = 0;
        coeff_fixedpoint.rudd_iMax_a = 0; 
        coeff_fixedpoint.rudd_Max_r = 0;
        coeff_fixedpoint.rudd_iMax_r = 0;    
    }
    
//    coeff_fixedpoint.aile_iMax_r = 50 / aile_elev.Ki_r;
//    coeff_fixedpoint.elev_iMax_r = 50 / aile_elev.Ki_r;
//    coeff_fixedpoint.rudd_iMax_r = 50 / rudder.Ki_r;    
//    coeff_fixedpoint.gas_iMax = 1000 / rudder.Ki_g;

    coeff_fixedpoint.aile_pre_error_r = 0;
    coeff_fixedpoint.elev_pre_error_r = 0;
    
    coeff_fixedpoint.height_pre_error = 0;
    coeff_fixedpoint.prev_error_longitude = 0;
    coeff_fixedpoint.prev_error_latitude = 0;
    coeff_fixedpoint.prev_error_direction = 0;
    coeff_fixedpoint.prev_error_speed = 0;
    
    coeff_fixedpoint.prev_aile_output_a=0;
    coeff_fixedpoint.prev_elev_output_a=0;
    
    hoverkp = HOVERKP;
    hoverki = HOVERKI;
    hoverkd = HOVERKD;
    
    nom_hov_thr = NOMINAL_HVR_THR_VAL;    //this should not go here
    normalizeGains=0;
}


/****************************************************************************
 * Function Name: multiply_gains
 * Created by: MattW on 15.05.2016
 
 * Description: This function multiplies all PID gains which are listed
 * within it by a given multiplication factor passed by the function call. 
 * This function is used a sort of state controller to instantly reduce
 * gains or increase gains if the quadcopter enters a given state.
 
 * Inputs:  float
 * Returns: void 
 * Calls:   None
 
 * Last Modified: 15.05.2016 (MW)
****************************************************************************/
void multiply_gains(float factor)
{
    if(!normalizeGains)
    {
        aile_elev.Kp_a *= factor;
        aile_elev.Ki_a *= factor;
        aile_elev.Kd_a *= factor;
        aile_elev.Kp_r *= factor;
        aile_elev.Ki_r *= factor;
        aile_elev.Kd_r *= factor;

        rudder.Kp_a *= factor;
        rudder.Ki_a *= factor;
        rudder.Kd_a *= factor;
        rudder.Kp_r *= factor;
        rudder.Ki_r *= factor;
        rudder.Kd_r *= factor;
        
        normalizeGains=1;
    }
}

void get_PID_coefficients(s_controller_fixedpoint* controller_fix, s_controller_specifications controller_specifications_aile_elev, s_controller_specifications controller_specifications_rudder, s_controller_specifications controller_specifications_gas)
{
    //-----------------------------------( Angle )
    //---------------------------Kp
    controller_fix->Kp_aile_a = (controller_specifications_aile_elev.Kp_a);                  //Fixedpoint Kp aile; -> ex. _Q16(Kp=1.5) = 1.5*2^16=98304
    controller_fix->Kp_elev_a = (controller_specifications_aile_elev.Kp_a);                  //Fixedpoint Kp elev
    controller_fix->Kp_rudd_a = (controller_specifications_rudder.Kp_a);                     //Fixedpoint Kp aile

    //---------------------------Ki                                                 //Possible to add a condition IF Ki = 0, assing 0 inmediatly
    controller_fix->Ki_aile_a = (controller_specifications_aile_elev.Ki_a);                  //Fixedpoint Kp aile
    controller_fix->Ki_elev_a = (controller_specifications_aile_elev.Ki_a);                  //Fixedpoint Kp elev
    controller_fix->Ki_rudd_a = (controller_specifications_rudder.Ki_a);                     //Fixedpoint Kp aile

    //---------------------------Kd
    controller_fix->Kd_aile_a = (controller_specifications_aile_elev.Kd_a);                  //Fixedpoint Kp aile
    controller_fix->Kd_elev_a = (controller_specifications_aile_elev.Kd_a);                  //Fixedpoint Kp elev
    controller_fix->Kd_rudd_a = (controller_specifications_rudder.Kd_a);                     //Fixedpoint Kp aile

   //-----------------------------------( Rate )
    //---------------------------Kp
    controller_fix->Kp_aile_r = (controller_specifications_aile_elev.Kp_r);                  //Fixedpoint Kp aile; -> ex. _Q16(Kp=1.5) = 1.5*2^16=98304
    controller_fix->Kp_elev_r = (controller_specifications_aile_elev.Kp_r);                  //Fixedpoint Kp elev
    controller_fix->Kp_rudd_r = (controller_specifications_rudder.Kp_r);                     //Fixedpoint Kp aile

    //---------------------------Ki                                                 //Possible to add a condition IF Ki = 0, assing 0 inmediatly
    controller_fix->Ki_aile_r = (controller_specifications_aile_elev.Ki_r);                  //Fixedpoint Kp aile
    controller_fix->Ki_elev_r = (controller_specifications_aile_elev.Ki_r);                  //Fixedpoint Kp elev
    controller_fix->Ki_rudd_r = (controller_specifications_rudder.Ki_r);                     //Fixedpoint Kp aile

    //---------------------------Kd
    controller_fix->Kd_aile_r = (controller_specifications_aile_elev.Kd_r);                  //Fixedpoint Kp aile
    controller_fix->Kd_elev_r = (controller_specifications_aile_elev.Kd_r);                  //Fixedpoint Kp elev
    controller_fix->Kd_rudd_r = (controller_specifications_rudder.Kd_r);                     //Fixedpoint Kp aile

    //--------------------------- sampling time "dt"
    controller_fix->aile_dt = (controller_specifications_aile_elev.samp_time);
    controller_fix->elev_dt = (controller_specifications_aile_elev.samp_time);
    controller_fix->rudd_dt = (controller_specifications_rudder.samp_time);
    
    //-----------------------------------( Gas )
    //---------------------------Kp
    controller_fix->Kp_gas = (controller_specifications_gas.Kp_g);
    //---------------------------Ki
    controller_fix->Ki_gas = (controller_specifications_gas.Ki_g);
    //---------------------------Kd
    controller_fix->Kd_gas = (controller_specifications_gas.Kd_g);
        //--------------------------- sampling time "dt"
    controller_fix->gas_dt = (controller_specifications_gas.samp_time);

    controller_fix->Kp_lHeight = 50;
    
//    return;
}

//this function is a temporary hack to set global PID values to those
//used by the PID controller such that these values can be seen in MAVLink
//this function is called from get_PID_coefficients which is called once in main on initilization
void get_global_PID()
{
    //coeff_fixedpoint is global variable declared in main.c
    rollkp = coeff_fixedpoint.Kp_aile_a;
    rollki = coeff_fixedpoint.Ki_aile_a;
    rollkd = coeff_fixedpoint.Kd_aile_a;
    
    pitchkp = coeff_fixedpoint.Kp_elev_a;
    pitchki = coeff_fixedpoint.Ki_elev_a;
    pitchkd = coeff_fixedpoint.Kd_elev_a;
    
    yawkp = coeff_fixedpoint.Kp_rudd_a;
    yawki = coeff_fixedpoint.Ki_rudd_a;
    yawkd = coeff_fixedpoint.Kd_rudd_a;
    
}

//this function is a temporary hack to set controller PID gains to those set
//in GCS and sent over MAVLink
//this function is called in MAVParams.c when PID params are changed
void set_global_PID()
{
    //prt_FIX is global variable declared in main.c
    ptr_FIX->Kp_aile_a = rollkp;
    ptr_FIX->Ki_aile_a = rollki;
    ptr_FIX->Kd_aile_a = rollkd;
    
    ptr_FIX->Kp_elev_a = pitchkp;
    ptr_FIX->Ki_elev_a = pitchki;
    ptr_FIX->Kd_elev_a = pitchkd;
    
    ptr_FIX->Kp_rudd_a = yawkp;
    ptr_FIX->Ki_rudd_a = yawki;
    ptr_FIX->Kd_rudd_a = yawkd;
    
    ptr_FIX->Kp_hover = hoverkp;
    ptr_FIX->Ki_hover = hoverki;
    ptr_FIX->Kd_hover = hoverkd;    
}

/*
void get_PID_coefficients_RS232(s_controller_fixedpoint* controller_fix){

    int aux = (int)(RS232Rx[0]) ;

    // KP aile_elev ANGLE
    if(aux == 1){
        controller_fix->Kp_elev_a +=  0.01;
        controller_fix->Kp_aile_a +=  0.01;
    }
    if(aux == 2){
        controller_fix->Kp_elev_a +=  0.1;
        controller_fix->Kp_aile_a +=  0.1;
    }
    if(aux == 3){
        controller_fix->Kp_elev_a +=  1;
        controller_fix->Kp_aile_a +=  1;
    }
    if(aux == 4){
        controller_fix->Kp_elev_a -=  0.01;
        controller_fix->Kp_aile_a -=  0.01;
    }
    if(aux == 5){
        controller_fix->Kp_elev_a -=  0.1;
        controller_fix->Kp_aile_a -=  0.1;
    }
    if(aux == 6){
        controller_fix->Kp_elev_a -=  1;
        controller_fix->Kp_aile_a -=  1;
    }

    //KP aile_elev RATE
    if(aux == 21){
        controller_fix->Kp_elev_r +=  0.01;
        controller_fix->Kp_aile_r +=  0.01;
    }
    if(aux == 22){
        controller_fix->Kp_elev_r +=  0.1;
        controller_fix->Kp_aile_r +=  0.1;
    }
    if(aux == 23){
        controller_fix->Kp_elev_r +=  1;
        controller_fix->Kp_aile_r +=  1;
    }
    if(aux == 24){
        controller_fix->Kp_elev_r -=  0.01;
        controller_fix->Kp_aile_r -=  0.01;
    }
    if(aux == 25){
        controller_fix->Kp_elev_r -=  0.1;
        controller_fix->Kp_aile_r -=  0.1;
    }
    if(aux == 26){
        controller_fix->Kp_elev_r -=  1;
        controller_fix->Kp_aile_r -=  1;
    }

    //KI aile_elev RATE
    if(aux == 31){
        controller_fix->Ki_elev_r +=  0.01;
        controller_fix->Ki_aile_r +=  0.01;
    }
    if(aux == 32){
        controller_fix->Ki_elev_r +=  0.1;
        controller_fix->Ki_aile_r +=  0.1;
    }
    if(aux == 33){
        controller_fix->Ki_elev_r +=  1;
        controller_fix->Ki_aile_r +=  1;
    }
    if(aux == 34){
        controller_fix->Ki_elev_r -=  0.01;
        controller_fix->Ki_aile_r -=  0.01;
    }
    if(aux == 35){
        controller_fix->Ki_elev_r -=  0.1;
        controller_fix->Ki_aile_r -=  0.1;
    }
    if(aux == 36){
        controller_fix->Ki_elev_r -=  1;
        controller_fix->Ki_aile_r -=  1;
    }

    //Kp rudd ANGLE
    if(aux == 41){
        controller_fix->Kp_rudd_a +=  0.01;
    }
    if(aux == 42){
        controller_fix->Kp_rudd_a +=  0.1;
    }
    if(aux == 43){
        controller_fix->Kp_rudd_a +=  1;
    }
    if(aux == 44){
        controller_fix->Kp_rudd_a -=  0.01;
    }
    if(aux == 45){
        controller_fix->Kp_rudd_a -=  0.1;
    }
    if(aux == 46){
        controller_fix->Kp_rudd_a -=  1;
    }

    //Kp rudd ANGLE
    if(aux == 51){
        controller_fix->Kp_rudd_r +=  0.01;
    }
    if(aux == 52){
        controller_fix->Kp_rudd_r +=  0.1;
    }
    if(aux == 53){
        controller_fix->Kp_rudd_r +=  1;
    }
    if(aux == 54){
        controller_fix->Kp_rudd_r -=  0.01;
    }
    if(aux == 55){
        controller_fix->Kp_rudd_r -=  0.1;
    }
    if(aux == 56){
        controller_fix->Kp_rudd_r -=  1;
    }
    //Ki rudd ANGLE
    if(aux == 61){
        controller_fix->Ki_rudd_r +=  0.01;
    }
    if(aux == 62){
        controller_fix->Ki_rudd_r +=  0.1;
    }
    if(aux == 63){
        controller_fix->Ki_rudd_r +=  1;
    }
    if(aux == 64){
        controller_fix->Ki_rudd_r -=  0.01;
    }
    if(aux == 65){
        controller_fix->Ki_rudd_r -=  0.1;
    }
    if(aux == 66){
        controller_fix->Ki_rudd_r -=  1;
    }

    // Kd aile_elev ANGLE
    if(aux == 71){
        controller_fix->Kd_elev_r +=  0.001;
        controller_fix->Kd_aile_r +=  0.001;
    }
    if(aux == 72){
        controller_fix->Kd_elev_r +=  0.01;
        controller_fix->Kd_aile_r +=  0.01;
    }
    if(aux == 73){
        controller_fix->Kd_elev_r +=  0.1;
        controller_fix->Kd_aile_r +=  0.1;
    }
    if(aux == 74){
        controller_fix->Kd_elev_r -=  0.001;
        controller_fix->Kd_aile_r -=  0.001;
    }
    if(aux == 75){
        controller_fix->Kd_elev_r -=  0.01;
        controller_fix->Kd_aile_r -=  0.01;
    }
    if(aux == 76){
        controller_fix->Kd_elev_r -=  0.1;
        controller_fix->Kd_aile_r -=  0.1;
    }

    // Ki aile_elev ANGLE
    if(aux == 81){
        controller_fix->Ki_elev_a +=  0.01;
        controller_fix->Ki_aile_a +=  0.01;
    }
    if(aux == 82){
        controller_fix->Ki_elev_a +=  0.1;
        controller_fix->Ki_aile_a +=  0.1;
    }
    if(aux == 83){
        controller_fix->Ki_elev_a +=  1;
        controller_fix->Ki_aile_a +=  1;
    }
    if(aux == 84){
        controller_fix->Ki_elev_a -=  0.01;
        controller_fix->Ki_aile_a -=  0.01;
    }
    if(aux == 85){
        controller_fix->Ki_elev_a -=  0.1;
        controller_fix->Ki_aile_a -=  0.1;
    }
    if(aux == 86){
        controller_fix->Ki_elev_a -=  1;
        controller_fix->Ki_aile_a -=  1;
    }

    RS232Rx[0] = 0;
}
*/

/****************************************************************************
            FOLLOWING 3 FUNCTIONS ARE FOR ANGLE PID CALCULATIONS
****************************************************************************/
void PIDcal_fixedpoint_aile_angle(s_controller_fixedpoint* controller_fix)
{
    controller_fix->aile_error_a = controller_fix->setpoint_angle_aile - controller_fix->actual_angle_aile;
    
    if(controller_fix->actual_angle_aile != controller_fix->prev_angle_aile)
    {
        if(controller_fix->aile_error_a > controller_fix->epsilon || controller_fix->aile_error_a < -controller_fix->epsilon)
        {
            controller_fix->aile_integral_a += controller_fix->aile_error_a * controller_fix->aile_dt;
            
            if(controller_fix->aile_integral_a > controller_fix->aile_iMax_a)
                controller_fix->aile_integral_a = controller_fix->aile_iMax_a;
            else if(controller_fix->aile_integral_a < -controller_fix->aile_iMax_a)
                controller_fix->aile_integral_a = -controller_fix->aile_iMax_a;
        }
    }
    
    controller_fix->aile_output_a = controller_fix->Kp_aile_a * controller_fix->aile_error_a + controller_fix->Ki_aile_a * controller_fix->aile_integral_a;
    
    if(controller_fix->aile_output_a > controller_fix->aile_Max_a)
        controller_fix->aile_output_a = controller_fix->aile_Max_a;
    if(controller_fix->aile_output_a < -controller_fix->aile_Max_a)
        controller_fix->aile_output_a = -controller_fix->aile_Max_a;
    
    //controller_fix->aile_rate_ref = (controller_fix->aile_output_a - controller_fix->prev_aile_output_a)/rudder.samp_time;
    
    //controller_fix->prev_aile_output_a = controller_fix->aile_output_a;
}

void PIDcal_fixedpoint_elev_angle(s_controller_fixedpoint* controller_fix)
{
    controller_fix->elev_error_a = controller_fix->setpoint_angle_elev - controller_fix->actual_angle_elev;
    
    if(controller_fix->actual_angle_elev != controller_fix->prev_angle_elev)
    {
        if(controller_fix->elev_error_a > controller_fix->epsilon || controller_fix->elev_error_a < -controller_fix->epsilon)
        {
            controller_fix->elev_integral_a += controller_fix->elev_error_r * controller_fix->elev_dt;
            
            if(controller_fix->elev_integral_a > controller_fix->elev_iMax_r)
                controller_fix->elev_integral_a = controller_fix->elev_iMax_r;
            else if(controller_fix->elev_integral_a < -controller_fix->elev_iMax_r)
                controller_fix->elev_integral_a = -controller_fix->elev_iMax_r;
        }
    }
    controller_fix->elev_output_a = controller_fix->Kp_elev_a * controller_fix->elev_error_a + controller_fix->Ki_elev_a * controller_fix->elev_integral_a;
    
    if(controller_fix->elev_output_a > controller_fix->elev_Max_a)
        controller_fix->elev_output_a = controller_fix->elev_Max_a;
    if(controller_fix->elev_output_a < -controller_fix->elev_Max_a)
        controller_fix->elev_output_a = -controller_fix->elev_Max_a;
    
    //controller_fix->elev_rate_ref = (controller_fix->elev_output_a - controller_fix->prev_elev_output_a)/rudder.samp_time;
    
    //controller_fix->prev_elev_output_a = controller_fix->elev_output_a;
}

void PIDcal_fixedpoint_rudd_angle(s_controller_fixedpoint* controller_fix)
{
    controller_fix->rudd_error_a = controller_fix->setpoint_angle_rudd - controller_fix->actual_angle_rudd;
    //controller_fix->rudd_error_a = (controller_fix->setpoint_angle_rudd + controller_fix->rudd_offset) - controller_fix->actual_angle_rudd;

    if(controller_fix->actual_angle_aile != controller_fix->prev_angle_aile)
    {
        if(controller_fix->rudd_error_a > controller_fix->epsilon || controller_fix->rudd_error_a < -controller_fix->epsilon)
        {
            controller_fix->rudd_integral_a += controller_fix->rudd_error_a * controller_fix->rudd_dt;
            
            if(controller_fix->rudd_integral_a > controller_fix->rudd_iMax_a)
                controller_fix->rudd_integral_a = controller_fix->rudd_iMax_a;
            else if(controller_fix->rudd_integral_a < -controller_fix->rudd_iMax_a)
                controller_fix->rudd_integral_a = -controller_fix->rudd_iMax_a;
        }
    }
    
    controller_fix->rudd_output_a = controller_fix->Kp_rudd_a * controller_fix->rudd_error_a + controller_fix->Ki_rudd_a * controller_fix->rudd_integral_a;
    
    if(controller_fix->rudd_output_a > controller_fix->rudd_Max_a)
        controller_fix->rudd_output_a = controller_fix->rudd_Max_a;
    if(controller_fix->rudd_output_a < -controller_fix->rudd_Max_a)
        controller_fix->rudd_output_a = -controller_fix->rudd_Max_a;
}
/****************************************************************************
            ***********END ANGLE PID CALCULATIONS*************
****************************************************************************/

/****************************************************************************
            FOLLOWING 3 FUNCTIONS ARE FOR RATE PID CALCULATIONS
****************************************************************************/
void PIDcal_fixedpoint_aile_rate(s_controller_fixedpoint* controller_fix)
{
    //controller_fix->aile_error_r = controller_fix->aile_rate_ref - controller_fix->actual_rate_aile;
    if(AcroMode)
        controller_fix->aile_error_r = controller_fix->setpoint_angle_aile - controller_fix->actual_rate_aile;
    else
        controller_fix->aile_error_r = controller_fix->aile_output_a - controller_fix->actual_rate_aile;
    
//    if(controller_fix->aile_error_r > controller_fix->epsilon || controller_fix->aile_error_r < -controller_fix->epsilon){
    if(controller_fix->actual_rate_aile != controller_fix->prev_rate_aile)
    {
        controller_fix->aile_integral_r += controller_fix->aile_error_r * controller_fix->aile_dt;
        
        if(controller_fix->aile_integral_r > controller_fix->aile_iMax_r)
            controller_fix->aile_integral_r = controller_fix->aile_iMax_r;
        else if(controller_fix->aile_integral_r < -controller_fix->aile_iMax_r)
            controller_fix->aile_integral_r = -controller_fix->aile_iMax_r;
    
        if((controller_fix->aile_error_r - controller_fix->aile_pre_error_r) > controller_fix->epsilon || (controller_fix->aile_error_r - controller_fix->aile_pre_error_r) < -controller_fix->epsilon)
            controller_fix->aile_derivative_r = (controller_fix->aile_error_r - controller_fix->aile_pre_error_r) / controller_fix->aile_dt; 
    }

    controller_fix->aile_output_r = controller_fix->Kp_aile_r * controller_fix->aile_error_r + controller_fix->Ki_aile_r * controller_fix->aile_integral_r + controller_fix->Kd_aile_r * controller_fix->aile_derivative_r;
    
    if(controller_fix->aile_output_r > controller_fix->aile_Max_r)
        controller_fix->aile_output_r = controller_fix->aile_Max_r;
    if(controller_fix->aile_output_r < -controller_fix->aile_Max_r)
        controller_fix->aile_output_r = -controller_fix->aile_Max_r;
    
    controller_fix->aile_pre_error_r = controller_fix->aile_error_r;
}

void PIDcal_fixedpoint_elev_rate(s_controller_fixedpoint* controller_fix)
{   
    //controller_fix->elev_error_r = controller_fix->elev_rate_ref - controller_fix->actual_rate_elev;
    if(AcroMode)
        controller_fix->elev_error_r = controller_fix->setpoint_angle_elev - controller_fix->actual_rate_elev;
    else
        controller_fix->elev_error_r = controller_fix->elev_output_a - controller_fix->actual_rate_elev;
    
//    if(controller_fix->elev_error_r > controller_fix->epsilon || controller_fix->elev_error_r < -controller_fix->epsilon){
    if(controller_fix->actual_rate_elev != controller_fix->prev_rate_elev)
    {
        controller_fix->elev_integral_r += controller_fix->elev_error_r * controller_fix->elev_dt;
        
        if(controller_fix->elev_integral_r > controller_fix->elev_iMax_r)
            controller_fix->elev_integral_r = controller_fix->elev_iMax_r;
        else if(controller_fix->elev_integral_r < -controller_fix->elev_iMax_r)
            controller_fix->elev_integral_r = -controller_fix->elev_iMax_r;
        
        if((controller_fix->elev_error_r - controller_fix->elev_pre_error_r) > controller_fix->epsilon || (controller_fix->elev_error_r - controller_fix->elev_pre_error_r) < -controller_fix->epsilon)
            controller_fix->elev_derivative_r = (controller_fix->elev_error_r - controller_fix->elev_pre_error_r) / controller_fix->elev_dt;

    }
    
    controller_fix->elev_output_r = controller_fix->Kp_elev_r * controller_fix->elev_error_r + controller_fix->Ki_elev_r * controller_fix->elev_integral_r + controller_fix->Kd_elev_r * controller_fix->elev_derivative_r;
    
    if(controller_fix->elev_output_r > controller_fix->elev_Max_r)
        controller_fix->elev_output_r = controller_fix->elev_Max_r;
    if(controller_fix->elev_output_r < -controller_fix->elev_Max_r)
        controller_fix->elev_output_r = -controller_fix->elev_Max_r;
    
    controller_fix->elev_pre_error_r = controller_fix->elev_error_r;
}

void PIDcal_fixedpoint_rudd_rate(s_controller_fixedpoint* controller_fix)
{
    if(AcroMode)
        controller_fix->rudd_error_r = controller_fix->setpoint_angle_rudd - controller_fix->actual_rate_rudd;
    else
        controller_fix->rudd_error_r = controller_fix->rudd_output_a - controller_fix->actual_rate_rudd;
    
    if(controller_fix->actual_rate_rudd != controller_fix->prev_rate_rudd)
    {
        if(controller_fix->rudd_error_r > controller_fix->epsilon || controller_fix->rudd_error_r < -controller_fix->epsilon)
        {
            controller_fix->rudd_integral_r += controller_fix->rudd_error_r * controller_fix->rudd_dt;
            if(controller_fix->rudd_integral_r > controller_fix->rudd_iMax_r)
                controller_fix->rudd_integral_r = controller_fix->rudd_iMax_r;
            else if(controller_fix->rudd_integral_r < -controller_fix->rudd_iMax_r)
                controller_fix->rudd_integral_r = -controller_fix->rudd_iMax_r;
        }
    
        if((controller_fix->rudd_error_r - controller_fix->rudd_pre_error_r) > controller_fix->epsilon || (controller_fix->rudd_error_r - controller_fix->rudd_pre_error_r) < -controller_fix->epsilon)
            controller_fix->rudd_derivative_r = (controller_fix->rudd_error_r - controller_fix->rudd_pre_error_r) / controller_fix->rudd_dt;
        
    }
    
    controller_fix->rudd_output_r = controller_fix->Kp_rudd_a * controller_fix->rudd_error_r + controller_fix->Ki_rudd_r * controller_fix->rudd_integral_r + controller_fix->Kd_rudd_r * controller_fix->rudd_derivative_r;
    
    if (controller_fix->rudd_output_r > controller_fix->rudd_Max_r)
        controller_fix->rudd_output_r = controller_fix->rudd_Max_r;

    if (controller_fix->rudd_output_r < -controller_fix->rudd_Max_r)
        controller_fix->rudd_output_r = -controller_fix->rudd_Max_r;

    controller_fix->rudd_pre_error_r = controller_fix->rudd_error_r;
}
/****************************************************************************
            ***********END RATE PID CALCULATIONS*************
****************************************************************************/

/****************************************************************************
 * Function Name: reset_ipart
 * Created by: Alejandro on 01.07.2015
 
 * Description: Resets the integral portion of all PID controllers. This 
 * function is called when the quadcopter is known to not be flying. i.e.
 * when the motors are in standby. This prevents overshoot from the controller.
 
 * Inputs:  *s_controller_fixedpoint
 * Returns: void 
 * Calls:   None
****************************************************************************/
void reset_ipart(s_controller_fixedpoint* controller_fix)
{

    controller_fix->aile_integral_r = 0;
    controller_fix->elev_integral_r = 0;
    controller_fix->rudd_integral_r = 0;
    controller_fix->gas_integral = 0;
    controller_fix->integral_latitude = 0;
    controller_fix->integral_longitude = 0;
    controller_fix->integral_direction = 0;
    controller_fix->integral_speed = 0;
    controller_fix->integral_hover = 0;

}

//Alejandro Code
/*
void PIDcal_height(s_controller_fixedpoint* controller_fix)
{
    controller_fix->height_error = controller_fix->setpoint_height - controller_fix->actual_height;

    //if e is not between 1 and -1
    if(controller_fix->height_error > controller_fix->epsilon || controller_fix->height_error < -controller_fix->epsilon)
    {
        //I = I + e*dt
        controller_fix->gas_integral += controller_fix->height_error * controller_fix->gas_dt;
        
        //if I > I_max, I = I_max
        if(controller_fix->gas_integral > controller_fix->gas_iMax)
        {
            controller_fix->gas_integral = controller_fix->gas_iMax;
        }
        //if I < -I_max, I = -I_max
        else if(controller_fix->gas_integral < -controller_fix->gas_iMax)
        {
            controller_fix->gas_integral = -controller_fix->gas_iMax;
        }
        
    }
    //if not -1<(e-prev_e)<1, D = (e-prev_e)/dt
    if((controller_fix->height_error- controller_fix->height_pre_error) > controller_fix->epsilon || (controller_fix->height_error - controller_fix->height_pre_error) < -controller_fix->epsilon)
    {
        controller_fix->gas_derivative = (controller_fix->height_error - controller_fix->height_pre_error) / controller_fix->gas_dt;
    }
    
    //set = Kp*e+Ki*I+Kd*D
    controller_fix->setpoint_gas = controller_fix->Kp_gas * controller_fix->height_error + controller_fix->Ki_gas * controller_fix->gas_integral + controller_fix->Kd_gas * controller_fix->gas_derivative;
    
    if (controller_fix->setpoint_gas > controller_fix->gas_Max)
    {
        controller_fix->setpoint_gas = controller_fix->gas_Max;
    }
    if (controller_fix->setpoint_gas < -controller_fix->gas_Max)
    {
        controller_fix->setpoint_gas = -controller_fix->gas_Max;
    }
    
    controller_fix->height_pre_error = controller_fix->height_error;    //prev error = error

//JUST P-Controller//    controller_fix->setpoint_gas = controller_fix->Kp_gas * controller_fix->height_error;

    controller_fix->setpoint_gas = controller_fix->setpoint_gas + controller_fix->thr_holded;  // Value of Thr when Hold Z was activated + output of PID
}*/

/****************************************************************************
 * Function Name: PID_position
 * Created by: MattW on 05.01.2016
 
 * Description: This is a general PID function which may be used for any
 * PID controller as long as the values for all of the PID controllers are 
 * stored within the same structure. The function is passed the desired
 * setpoint, the current position, and a switch character to indicate the
 * desired PID controller to calculate for. The PID calculations are done
 * and then the function returns the output of the PID.
 
 * Inputs:  double, double, char, *s_controller_fixedpoint
 * Returns: float 
 * Calls:   offsetof() [included in std c libraries]
 
 * Last Modified: 05.01.2016 (MW)
 * 
 * NOTE: 'offsetof' function returns offset in BYTES
 * Pointer arithmetic states that incrementing a pointer increments the 
 * pointer by however many bytes that type of pointer requires. So incrementing
 * the 's_controller_fixedpoint' directly does not increment the pointer to the
 * members as expected. Therefore, a typecast must be made from the 
 * 's_controller_fixedpoint' type to the member type, then that pointer is
 * incremented by calculated byte offset divided by member size. Then that 
 * value is recast into pointer of type 's_controller_fixedpoint' 
 * NOTE: Future development... add other members of s_controller_fixedpoint
 * structure to the if else switch.
****************************************************************************/
double PID_position(double setpoint, double position, controlled_position_t member)
{
    double output = 0;
    double error = 0;
    double prev_error = 0;
    
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    float dt = 0;
    
    int offset = 0;
    int offset_base = 0;
    
    double *dblPtr = NULL;      //Must be of s_controller_fixedpoint member type
    
    s_controller_fixedpoint* ctrlr = ptr_FIX;
   
    
//    if(member == 'l')
//    {
//        offset = offsetof(s_controller_fixedpoint, prev_error_latitude);
//    }
//    else if(member == 'a')
//    {
//        offset = offsetof(s_controller_fixedpoint, prev_error_altitude);
//    }
//    else if(member == 'c')
//    {
//        offset = offsetof(s_controller_fixedpoint, prev_error_direction);
//    }
//    else if(member == 's')
//    {
//        offset = offsetof(s_controller_fixedpoint, prev_error_speed);
//    }
//    else if(member == 'h')
//    {
//        offset = offsetof(s_controller_fixedpoint, prev_error_lHeight);
//    }
//    else
//    {
//        offset = offsetof(s_controller_fixedpoint, prev_error_longitude);
//    }
    
    switch(member)
    {
        case ALTITUDE:
            offset = offsetof(s_controller_fixedpoint, prev_error_hover);
        break;
        case LATITUDE:
            offset = offsetof(s_controller_fixedpoint, prev_error_latitude);
        break;
        case LONGITUDE:
            offset = offsetof(s_controller_fixedpoint, prev_error_longitude);
        break;
        case GROUND_SPEED:
            offset = offsetof(s_controller_fixedpoint, prev_error_speed);
        break;
        case GROUND_COURSE:
            offset = offsetof(s_controller_fixedpoint, prev_error_direction);
        break;
        default:
            //not sure what default case should be
        break;
    }
    
    offset_base = offsetof(s_controller_fixedpoint, prev_error_longitude);
    
    offset = offset - offset_base;
  
    //SEE NOTE ABOVE regarding why I did this
    dblPtr = (double*)ctrlr + offset/sizeof(double);
    ctrlr = (s_controller_fixedpoint*)dblPtr;
    

    //get controller coefficients
    Kp = ctrlr->Kp_longitude;
    Ki = ctrlr->Ki_longitude;
    Kd = ctrlr->Kd_longitude;
    
    dt = ctrlr->dt_longitude;
    
    error = setpoint - position;
    prev_error = ctrlr->prev_error_longitude;
    
            
    //Integral += Error*dt;
    ctrlr->integral_longitude += error*dt;
  
    //derivative = (error - prev_error)/dt;
    ctrlr->derivative_longitude = (error - prev_error)/dt;
    
    //Output = Kp*error + Ki*integral + Kd*derivative;
    output = Kp*error + Ki*ctrlr->integral_longitude + Kd*ctrlr->derivative_longitude;
    
    ctrlr->prev_error_longitude = error;
    
    return output;
}

