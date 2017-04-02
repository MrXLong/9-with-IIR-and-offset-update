#include "Mixer.h"
#include "state.h" //to change state with transmitter switches

#define LEDdivisor 150

// Initializing the array for mixer
unsigned int thro [4] = {0, 0, 0, 0};
int aile [4] = {0, 0, 0, 0};
int elev [4] = {0, 0, 0, 0};
int rudd [4] = {0, 0, 0, 0};

// Initialization of variables to display a made mixing current values
volatile int thr_mix = 0;
volatile int ail_mix = 0;
volatile int ele_mix = 0;
volatile int rud_mix = 0;

int Switch = 0;

int gather = 0;
volatile int yaw_rate = 0;

volatile unsigned int motor1_led,motor2_led,motor3_led,motor4_led;

// initializations structures
manualMix PWM;
manualMix *ptr_PWM = &PWM;

mix MIX;
mix *ptr_MIX = &MIX;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------- setpoints (Input Capture) -- Data acquire-------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

long  map(long x, long in_min, long in_max, long out_min, long out_max){            //function to scale variables
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setValue (pw *ptr_PW)
{
    //Deviation of actual throttle position from neutral position.
    ptr_FIX->setpoint_angle_aile = -map(ptr_PW->aile ,5500, 9500, -45, 45);    //    +- 45°  -> 1080
    ptr_FIX->setpoint_angle_elev = map(ptr_PW->elev, 5500, 9500, -45, 45);    //    +- 45°  -> 1080
    ptr_FIX->rudd_angle_target = -map(ptr_PW->rudd, 5565, 9500, -150, 150);  //    +- 180° -> 1600   
    //MW perhaps the min and max input values should be revisited, especially for rudder max
}

void setValueGas (pw *ptr_PW)
{
    ptr_FIX->setpoint_gas = ptr_PW->thro;
}


// actual values (UM7) ROLL AND PITCH ANGLE FROM THE UM7
void actualValue_aile_elev (UM7DataSensor *ptr_eulerAngle)
{
    ptr_FIX->prev_angle_aile = ptr_FIX->actual_angle_aile;
    ptr_FIX->prev_angle_elev = ptr_FIX->actual_angle_elev;
    
    ptr_FIX->actual_angle_aile = (ptr_eulerAngle->eulerRollAngle * (EulerConstant)); // angle. data from sensor divided by 91.02222
    ptr_FIX->actual_angle_elev = (ptr_eulerAngle->eulerPitchAngle * (EulerConstant)); //eulerAngle * EulerConstant       = [Degrees (º)]

    ptr_FIX->prev_rate_aile = ptr_FIX->actual_rate_aile;
    ptr_FIX->prev_rate_elev = ptr_FIX->actual_rate_elev;
    
    ptr_FIX->actual_rate_aile = (ptr_eulerAngle->eulerRollRate * (GyroConstant)); // rate. data from sensor divided by 16
    ptr_FIX->actual_rate_elev = (ptr_eulerAngle->eulerPitchRate * (GyroConstant)); //processedGyro * GyroConstant     = [degrees/sec (º/s)]
}

// actual values (UM7) YAW ANGLE FROM THE UM7
// must adjust to degrees
//SHOULDNT THIS REFERENCE SAME PTR_FIX VALUES AS OTHERS? MW
void actualValue_rudd (UM7DataSensor *ptr_eulerAngle, s_controller_fixedpoint *ptr_fix)
{
    ptr_fix->prev_angle_rudd = ptr_fix->actual_angle_rudd;
    ptr_fix->actual_angle_rudd = ptr_eulerAngle->eulerYawAngle*(EulerConstant);
    
//    if(ptr_fix->actual_angle_rudd >180)
//        ptr_fix->actual_angle_rudd -= 360;
//    if(ptr_fix->actual_angle_rudd <=-180)
//        ptr_fix->actual_angle_rudd += 360;
    
    ptr_fix->prev_rate_rudd = ptr_fix->actual_rate_rudd;
    ptr_fix->actual_rate_rudd = ptr_eulerAngle->eulerYawRate*(GyroConstant);
}

void compensateYaw(int offset)
{
     //compensate offset
    ptr_FIX->actual_angle_rudd+=offset;
    if(ptr_FIX->actual_angle_rudd >180)
        ptr_FIX->actual_angle_rudd -= 360;
    if(ptr_FIX->actual_angle_rudd <=-180)
        ptr_FIX->actual_angle_rudd += 360;
}

void main_Switch (pw *ptr_PW)
{

     if (ptr_PW->gear == 0 && ptr_PW->aux1 == 0)
     {
        Switch = 0;  //free mode
        changeMode(STABILIZE, TRANSMITTER);
     }
     if (ptr_PW->gear == 0 && ptr_PW->aux1 == 1)
     {
        Switch = 1; //altitude hold
        changeMode(STABILIZE, TRANSMITTER);
     }
     if (ptr_PW->gear == 1 && ptr_PW->aux1 == 0)
     {
        Switch = 2; //guided mode
        changeMode(GUIDED, TRANSMITTER);
     }
     if (ptr_PW->gear == 1 && ptr_PW->aux1 == 1)
     {
        Switch = 3;
     }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------- Set data in the Mixer Array -------------------------------------------//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// For manipulated variable "gas" mixer (gas is 1: 1 pass), for automatic mode and manual mode (emergency mode)
void throMix (s_controller_fixedpoint *ptr_thro)
{
    thro[0] = ptr_thro->setpoint_gas;
    thro[1] = ptr_thro->setpoint_gas;
    thro[2] = ptr_thro->setpoint_gas;
    thro[3] = ptr_thro->setpoint_gas;

    thr_mix = 1;
}

// For manipulated variable mixer "rolling"
void aileMix (s_controller_fixedpoint *ptr_fix_aile)                             // left
{
    aile[0] = -(map(ptr_fix_aile->aile_output_r, -45, 45, -1080, 1080));       // roll: Motor1 faster
    aile[1] = (map(ptr_fix_aile->aile_output_r, -45, 45, -1080, 1080));        // roll: Motor2 slower
    aile[2] = (map(ptr_fix_aile->aile_output_r, -45, 45, -1080, 1080));        // roll: Motor3 slower
    aile[3] = -(map(ptr_fix_aile->aile_output_r, -45, 45, -1080, 1080));        // roll: Motor4 faster
    ail_mix = 1;
}

// For manipulated variable mixer "pitch"
void elevMix (s_controller_fixedpoint *ptr_fix_elev)                        // ahead (go front)
{
     elev[0] = -(map(ptr_fix_elev->elev_output_r, -45, 45, -1080, 1080));       // pitch: Motor1 slower
     elev[1] = -(map(ptr_fix_elev->elev_output_r, -45, 45, -1080, 1080));       // pitch: Motor2 slower
     elev[2] = (map(ptr_fix_elev->elev_output_r, -45, 45, -1080, 1080));          // pitch: Motor3 faster
     elev[3] = (map(ptr_fix_elev->elev_output_r, -45, 45, -1080, 1080));          // pitch: Motor4 faster
     ele_mix = 1;
}

// For manipulated variable mixer "yaw"
void ruddMix (s_controller_fixedpoint *ptr_fix_rudd)                             // left
{
    rudd[0] = (map(ptr_fix_rudd->rudd_output_r, -150, 150, -1600, 1600));             // yaw: Motor 1 slower
    rudd[1] = -(map(ptr_fix_rudd->rudd_output_r, -150, 150, -1600, 1600));                // yaw: Motor 2 faster
    rudd[2] = (map(ptr_fix_rudd->rudd_output_r, -150, 150, -1600, 1600));             // yaw: Motor 3 slower
    rudd[3] = -(map(ptr_fix_rudd->rudd_output_r, -150, 150, -1600, 1600));                // yaw: Motor 4 faster
    rud_mix = 1;
}
/*
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
}*/

// final motor mixer, here all 4 mixed manipulated variables are combined to form a control value per motor
void motorMix (unsigned int thr[], int ail[], int ele[], int rud[])
{
    ptr_MIX->m1 = (thr[0] + ail[0] + ele[0] + rud[0]); //Front Right
    motor1_led = (MIX.m1-5800)/LEDdivisor;
    if (ptr_MIX->m1 > 9500)     // PWM limit 1.9ms 9500 input capture value.
    {
        ptr_MIX->m1 = 9500;
    }
    ptr_MIX->m2 = (thr[1] + ail[1] + ele[1] + rud[1]);  //Front Left
    motor2_led = (MIX.m2-5800)/LEDdivisor;
    if (ptr_MIX->m2 > 9500)     // PWM limit 1.9ms 9500 input capture value.
    {
        ptr_MIX->m2 = 9500;
    }
    ptr_MIX->m3 = (thr[2] + ail[2] + ele[2] + rud[2]); //Back Left
    motor3_led = (MIX.m3-5800)/LEDdivisor;
    if (ptr_MIX->m3 > 9500)     // PWM limit 1.9ms 9500 input capture value.
    {
        ptr_MIX->m3 = 9500;
    }
    ptr_MIX->m4 = (thr[3] + ail[3] + ele[3] + rud[3]);  //Back Right
    motor4_led = (MIX.m4-5800)/LEDdivisor;
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
