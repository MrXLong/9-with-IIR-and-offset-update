/*
 * File:   PID.h
 * Author: Simon
 *
 * Created on December 30, 2014, 2:16 PM
 */

#ifndef PID_H
#define	PID_H

#define AcroMode 0       //flight Mode       


//MW THIS and PID.c need to be changed. The structure of this file and the
//PID.c file is horrible. We are using multiple data structures for the same
//values, and we also should't have this enormous data structure. However,
//current set up works and I don't have extra time to change how these files 
//operate and then test them, so I will leave it for the next student.
//Please take care when changing these, as these two files are at the core of the
//control code.

typedef enum
{
    ALTITUDE = 0,
    LATITUDE,
    LONGITUDE,
    GROUND_SPEED,
    GROUND_COURSE
} controlled_position_t;  //better name for this variable?

// Struktur für den Fixedpoint-Regler "aile" und "elev"
typedef struct
{
    // Variables for Roll (aile) (phi)
    float Kp_aile_a;           //angle               //_Q16 required 32 Bit, _Q15 required 16 Bit
    float Ki_aile_a;
    float Kd_aile_a;
    float Kp_aile_r;           //rate
    float Ki_aile_r;
    float Kd_aile_r;
    float aile_dt;
    float setpoint_angle_aile;          //setpoint_angle
    float actual_angle_aile;           //actual_angle
    float actual_rate_aile;           //actual_rate
    float prev_angle_aile;           //actual_angle
    float prev_rate_aile;           //actual_rate
    int aile_Max_a;                  //for current saturation
    int aile_Max_r;                  //for current saturation
    float aile_pre_error_a;
    float aile_pre_error_r;
    float aile_integral_a;
    float aile_integral_r;
    float aile_error_a;
    float aile_error_r;
    float aile_derivative_a;
    float aile_derivative_r;
    float aile_output_a;
    float prev_aile_output_a;
    float aile_rate_ref;
    float aile_output_r;
    float aile_iMax_a;                 //Maximun value Integral part
    float aile_iMax_r;

    // Variables for Pitch(elev) (theta)
    float Kp_elev_a;          //angle                //_Q16 required 32 Bit, _Q15 required 16 Bit
    float Ki_elev_a;
    float Kd_elev_a;
    float Kp_elev_r;           //rate
    float Ki_elev_r;
    float Kd_elev_r;
    float elev_dt;
    float setpoint_angle_elev;          //setpoint
    float actual_angle_elev;           //actual_angle
    float actual_rate_elev;           //actual_rate
    float prev_angle_elev;           //actual_angle
    float prev_rate_elev;           //actual_rate
    int elev_Max_a;                  //for current saturation
    int elev_Max_r;                  //for current saturation
    float elev_pre_error_a;
    float elev_pre_error_r;
    float elev_integral_a;
    float elev_integral_r;
    float elev_error_a;
    float elev_error_r;
    float elev_derivative_a;
    float elev_derivative_r;
    float elev_output_a;
    float prev_elev_output_a;
    float elev_rate_ref;
    float elev_output_r;
    float elev_iMax_a;                 //Maximun value Integral part
    float elev_iMax_r;

    // Variables for Yaw (rudd) (Psi)
    float Kp_rudd_a;           //angle              //_Q16 required 32 Bit, _Q15 required 16 Bit
    float Ki_rudd_a;
    float Kd_rudd_a;
    float Kp_rudd_r;           //rate
    float Ki_rudd_r;
    float Kd_rudd_r;
    float rudd_dt;
    float rudd_offset;
    float rudd_angle_target;
    float setpoint_angle_rudd;          //setpoint
    float actual_angle_rudd;          //actual_angle
    float actual_rate_rudd;          //actual_rate
    float prev_angle_rudd;          //actual_angle
    float prev_rate_rudd;          //actual_rate
    int rudd_Max_a;                 //for current saturation
    int rudd_Max_r;                 //for current saturation
    float rudd_pre_error_a;
    float rudd_pre_error_r;
    float rudd_integral_a;
    float rudd_integral_r;
    float rudd_error_a;
    float rudd_error_r;
    float rudd_derivative_a;
    float rudd_derivative_r;
    float rudd_output_a;
    float rudd_output_r;
    float rudd_iMax_a;                 //Maximun value Integral part
    float rudd_iMax_r;
    

    float epsilon;               // if(abs(error) > epsilon){integral = integral + error*dt;}

    //variable Gas

    float Kp_gas;
    float Ki_gas;
    float Kd_gas;
    float gas_dt;
    //unsigned int setpoint_height;   //height setpoint
    float actual_height;            //actual_height
    int gas_Max;                    //for current saturation
    float height_pre_error;
    float gas_integral;
    float height_error;
    float gas_derivative;
    float gas_iMax;                 //Maximun value Integral part
    unsigned int setpoint_gas;   // output

    unsigned int thr_holded;
    
    
    //parameters for GPS position control
    float Kp_longitude;
    float Ki_longitude;
    float Kd_longitude;
    float dt_longitude;
    double integral_longitude;
    double derivative_longitude;
    double prev_error_longitude;
    
    float Kp_latitude;
    float Ki_latitude;
    float Kd_latitude;
    float dt_latitude;
    double integral_latitude;
    double derivative_latitude;
    double prev_error_latitude;
    
    float Kp_direction;
    float Ki_direction;
    float Kd_direction;
    float dt_direction;
    double integral_direction;
    double derivative_direction;
    double prev_error_direction;
    
    float Kp_speed;
    float Ki_speed;
    float Kd_speed;
    float dt_speed;
    double integral_speed;
    double derivative_speed;
    double prev_error_speed;
    
    float Kp_hover;
    float Ki_hover;
    float Kd_hover;
    float dt_hover;
    double integral_hover;
    double derivative_hover;
    double prev_error_hover;
    
    float Kp_lHeight;
    float Ki_lHeight;
    float Kd_lHeight;
    float dt_lHeight;
    double integral_lHeight;
    double derivative_lHeight;
    double prev_error_lHeight;
   
   
}s_controller_fixedpoint;

extern s_controller_fixedpoint coeff_fixedpoint;
extern s_controller_fixedpoint *ptr_FIX;

// Structure for the controller specification
typedef struct
{
    float Kp_a; //angle
    float Ki_a;
    float Kd_a;
    float Kp_r; //rate
    float Ki_r;
    float Kd_r;
    float Kp_g; //gas
    float Ki_g;
    float Kd_g;
    float samp_time; //sample time
    float setpoint; //setpoint
    int manipulated_pwm;
}s_controller_specifications;

extern s_controller_specifications aile_elev;
extern s_controller_specifications rudder;
extern s_controller_specifications gas;

void set_PID_coefficients(void);
void get_PID_coefficients(s_controller_fixedpoint* controller_fix, s_controller_specifications controller_specifications_aile_elev, s_controller_specifications controller_specifications_rudder, s_controller_specifications controller_specifications_gas);
// PID-------------------------angle
void PIDcal_fixedpoint_aile_angle(s_controller_fixedpoint* controller_fix);
void PIDcal_fixedpoint_elev_angle(s_controller_fixedpoint* controller_fix);
void PIDcal_fixedpoint_rudd_angle(s_controller_fixedpoint* controller_fix);
// PID-------------------------rate
void PIDcal_fixedpoint_aile_rate(s_controller_fixedpoint* controller_fix);
void PIDcal_fixedpoint_elev_rate(s_controller_fixedpoint* controller_fix);
void PIDcal_fixedpoint_rudd_rate(s_controller_fixedpoint* controller_fix);

void reset_ipart(s_controller_fixedpoint* controller_fix);

void get_PID_coefficients_RS232(s_controller_fixedpoint* controller_fix);

void PIDcal_height(s_controller_fixedpoint* controller_fix);
double PID_position(double setpoint, double position, controlled_position_t member);

void multiply_gains(float factor);

void setPID(int,int,int,float,s_controller_fixedpoint* ctrlr);
void listPID();

void get_global_PID();
void set_global_PID();

#endif	/* PID_H */
