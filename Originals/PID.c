
#include "PID.h"
#include "libq.h"
#include "Mixer.h"

void set_PID_coefficients(void)
{

    // PID-maximum sizes
    // to obtain from these Q9 values fixed-point values must be divided by 2 ^ 9
    coeff_fixedpoint.aile_Max = _Q16ftoi(9500); //1048576000; -->   // with this maximum manipulated variable is Aile [0-3] up to 1000 (equal to 25%)
    coeff_fixedpoint.elev_Max = _Q16ftoi(9500); //1048576000;
    coeff_fixedpoint.rudd_Max = _Q16ftoi(9500); //190000000;
    coeff_fixedpoint.aile_iMax = 23000000;
    coeff_fixedpoint.elev_iMax = 23000000;
    coeff_fixedpoint.rudd_iMax = 23000000;



    // PID-Coefficient aile & elev
    aile_elev.Kp = 0.4;                           // Declared in main.c
    aile_elev.Ki = 1.3 /2;
    aile_elev.Kd = 0 /8;
    aile_elev.samp_time = 0.01;
   
    // PID-Coefficient rudder
    rudder.Kp = 0.6;
    rudder.Ki = 5.12;
    rudder.Kd = 0;
    rudder.samp_time = 0.01;

    //epsilon
    coeff_fixedpoint.epsilon = _Q16ftoi(0.01);                                       // epsilon = 655.36
    coeff_fixedpoint.epsilon = coeff_fixedpoint.epsilon >> 7;                       //WHY SHIFT?    epsilon = 40.96
}

// Determine coefficients for the PID Fixed Point Controller
void get_PID_coefficients_fixedpoint(s_controller_fixedpoint* controller_fix, s_controller_specifications controller_specifications_aile_elev, s_controller_specifications controller_specifications_rudder)
{
    //---------------------------Kp
    controller_fix->Kp_aile = _Q16ftoi(controller_specifications_aile_elev.Kp);                  //Fixedpoint Kp aile; -> ex. _Q16(Kp=1.5) = 1.5*2^16=98304
    controller_fix->Kp_elev = _Q16ftoi(controller_specifications_aile_elev.Kp);                  //Fixedpoint Kp elev
    controller_fix->Kp_rudd = _Q16ftoi(controller_specifications_rudder.Kp);                     //Fixedpoint Kp aile
    controller_fix->Kp_aile = controller_fix->Kp_aile >> 7;                                      //Kp/2^4;  1.5*2^(16-4) = 2^12 -> ex. Kp=1.5 :-> 1.5*2^12=6144
    controller_fix->Kp_elev = controller_fix->Kp_elev >> 7;
    controller_fix->Kp_rudd = controller_fix->Kp_rudd >> 7;
    //---------------------------Ki                                                 //Possible to add a condition IF Ki = 0, assing 0 inmediatly
    controller_fix->Ki_aile = _Q16ftoi(controller_specifications_aile_elev.Ki);                  //Fixedpoint Kp aile
    controller_fix->Ki_elev = _Q16ftoi(controller_specifications_aile_elev.Ki);                  //Fixedpoint Kp elev
    controller_fix->Ki_rudd = _Q16ftoi(controller_specifications_rudder.Ki);                     //Fixedpoint Kp aile
    controller_fix->Ki_aile = controller_fix->Ki_aile >> 7;
    controller_fix->Ki_elev = controller_fix->Ki_elev >> 7;
    controller_fix->Ki_rudd = controller_fix->Ki_rudd >> 7;
    //---------------------------Kd
    controller_fix->Kd_aile = _Q16ftoi(controller_specifications_aile_elev.Kd);                  //Fixedpoint Kp aile
    controller_fix->Kd_elev = _Q16ftoi(controller_specifications_aile_elev.Kd);                  //Fixedpoint Kp elev
    controller_fix->Kd_rudd = _Q16ftoi(controller_specifications_rudder.Kd);                     //Fixedpoint Kp aile
    controller_fix->Kd_aile = controller_fix->Kd_aile >> 7;
    controller_fix->Kd_elev = controller_fix->Kd_elev >> 7;
    controller_fix->Kd_rudd = controller_fix->Kd_rudd >> 7;
   
//    return;
}


// PID algorithm calculation


void PIDcal_fixedpoint_aile(s_controller_fixedpoint* controller_fix)
{
    _Q16 Output_AUX;

    //Calculate P,I,D
    //P
    controller_fix->aile_error = controller_fix->setpoint_angle_aile - controller_fix->actual_angle_aile;

    //In case error too small, then stop integration
    //I
    if(abs(controller_fix->aile_error) > (controller_fix->epsilon))
    {
        controller_fix->aile_integral =  controller_fix->aile_integral + controller_fix->aile_error * controller_fix->aile_dt;

    }
    //D
    controller_fix->aile_derivative = ( controller_fix->aile_error - controller_fix->aile_pre_error) / controller_fix->aile_dt;

    Output_AUX = controller_fix->Kp_aile * controller_fix->aile_error + controller_fix->Ki_aile * controller_fix->aile_integral + controller_fix->Kd_aile * controller_fix->aile_derivative;
    //Saturation Filter
    if (Output_AUX > controller_fix->aile_Max)
    {
        Output_AUX = controller_fix->aile_Max;
    }
    controller_fix->aile_output = Output_AUX;
    controller_fix->aile_output = controller_fix->aile_output >> 18;                    //WHY? %%%%%%%%%%%%%%%%%%%%%
}

void PIDcal_fixedpoint_elev(s_controller_fixedpoint* controller_fix)
{
    _Q16 Output_AUX;

    //Calculate P,I,D
    //P
    controller_fix->elev_error = controller_fix->setpoint_angle_elev - controller_fix->actual_angle_elev;

    //In case error too small, then stop integration
    //I
    if(abs(controller_fix->elev_error) > (controller_fix->epsilon))
    {
        controller_fix->elev_integral = controller_fix->elev_integral + controller_fix->elev_error * controller_fix->elev_dt;
    }
    //D
   controller_fix->elev_derivative = ( controller_fix->elev_error - controller_fix->elev_pre_error) / controller_fix->elev_dt;

    Output_AUX = controller_fix->Kp_elev * controller_fix->elev_error + controller_fix->Ki_elev * controller_fix->elev_integral + controller_fix->Kd_elev * controller_fix->elev_derivative;
    //Saturation Filter
    if (Output_AUX > controller_fix->elev_Max)
    {
        Output_AUX = controller_fix->elev_Max;
    }
    controller_fix->elev_output = Output_AUX;
    controller_fix->elev_output = controller_fix->elev_output >> 18;                    //WHY? %%%%%%%%%%%%%%%%%%%%%
}

void PIDcal_fixedpoint_rudd(s_controller_fixedpoint* controller_fix)
{
    _Q16 Output_AUX;

    //Calculate P,I,D
    //P
    controller_fix->rudd_error = controller_fix->setpoint_rate_rudd - controller_fix->actual_rate_rudd;

    //In case error too small, then stop integration
    //I
    if(abs(controller_fix->rudd_error) > (controller_fix->epsilon))
    {
        controller_fix->rudd_integral = controller_fix->rudd_integral + controller_fix->rudd_error * controller_fix->rudd_dt;
    }
    //D
    controller_fix->rudd_derivative = controller_fix->rudd_error - controller_fix->rudd_pre_error;

    Output_AUX = controller_fix->Kp_rudd * controller_fix->rudd_error + controller_fix->Ki_rudd * controller_fix->rudd_integral + controller_fix->Kd_rudd * controller_fix->rudd_derivative;
    //Saturation Filter
    if (Output_AUX > controller_fix->rudd_Max)
    {
        Output_AUX = controller_fix->rudd_Max;
    }
    controller_fix->rudd_output = Output_AUX;
    controller_fix->rudd_output = controller_fix->rudd_output >> 18;                    //WHY? %%%%%%%%%%%%%%%%%%%%%
}