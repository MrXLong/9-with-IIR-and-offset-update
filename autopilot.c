//contains methods to handle movements based on 
//mavlink commands


#include "autopilot.h"
#include "PID.h"    //to use PID function
#include "SRF02.h" //to access ultrasonic
#include "options.h" //for control settings
#include "state.h"  //to get modes
#include "MAVLink.h"    //for debug print
#include "gain_variables.h" //to access nominal throttle values


int32_t min_throttle;

float guidance_v_nominal_throttle;
unsigned int actual_height = 0;
unsigned int throttle_setpoint=0;
unsigned int target_height=0;
unsigned int height_min=0;
unsigned int height_max=0;
boolean takeoff_requested = false;
boolean land_requested = false;

//control loop for autopilot functions
//this function is called from main at a certain frequency
//which is defined as AUTOPILOT_FREQ in options.h
void autopilot_periodic(void)
{
    //check if vehicle is in flight or not
    //check if vehicle is too far from home, if so set mode back to home

    //for testing, make sure that quad doesn't reach the maximum of the ultrasonic
    //because then the feedback is unknown and the quad could be keep gaining height
    //if(actual_height < (SONAR_MAX-5))
    if(1)
        guidance_vert_run();
    else
        changeMode(STABILIZE, TRANSMITTER);
    
    //if everything is good run guidance_vert and guidance_horiz loops
}
void setPitchAngle(float desired_angle)
{
    ptr_FIX->setpoint_angle_elev = desired_angle;
}

void setRollAngle(float desired_angle)
{
    ptr_FIX->setpoint_angle_aile = desired_angle;
}

void setYawAngle(float desired_angle)
{
    ptr_FIX->rudd_angle_target = desired_angle;
}

void setThrottle(unsigned int desired_throttle)
{
    ptr_FIX->setpoint_gas = desired_throttle;
}

void setPitchRate(float desired_rate)
{
    //placeholder
}
void setRollRate(float desired_rate)
{
    //placeholder
}
void setYawRate(float desired_rate)
{
    //placeholder
}

void guidance_vert_init()
{
    //make sure PID gains are set to proper values
    ptr_FIX->Kp_hover = HOVERKP;
    ptr_FIX->Ki_hover = HOVERKI;
    ptr_FIX->Kd_hover = HOVERKD;
    
    ptr_FIX->dt_hover = 1.0/AUTOPILOT_FREQ;
    
    //error should be zero
    ptr_FIX->integral_hover = 0;
    ptr_FIX->derivative_hover = 0;
    ptr_FIX->prev_error_hover = 0;
    
    //No Pitch/Roll Movement
    setPitchAngle(0);
    setRollAngle(0);
    
    //set bounds for height control
    //currently, height is bound by ultrasonic capabilities
    height_max = SONAR_MAX;
    height_min = SONAR_MIN;
    
    guidance_v_nominal_throttle = nom_hov_thr;
    //get nominal hover throttle (reference point for throttle required to hover in place)
}

//this function is called when the appropriate mavlink command is given
//this function should initialize vertical guidance variables and begin
//the vertical guidance
void simple_takeoff(unsigned int target_height_cm)
{
    guidance_vert_init();   //prepare for automated guidance in vertical direction
    
    //convert parameter to global target height value
    target_height = target_height_cm;
    Bound(target_height,height_min,height_max);
    
    //if mode changes, should immediately abandon takeoff
    switch(getMode())
    {
        case GUIDED:
                //start engines
                throttle_setpoint = MIN_THROTTLE_VALUE+25;
        break;
        default:
        break;
    } 
   
    takeoff_requested = true;
}

//this function is called when vertical guidance is required
//it should handle requests for vertical guidance based on the current
//mode/state of the quadcopter
//right now this function only allows for hovering at desired height
void guidance_vert_run()
{
    actual_height = Sonar_Dist[0];
    
    if(takeoff_requested)
    {
        run_hover_loop(target_height);  //hover at current height
        land_requested = false;
    }
    else if(land_requested)
    {
        if(actual_height > height_min)
        {
            target_height-=1; //subtract 1 cm from target height each loop pass
            run_hover_loop(target_height);
        }
        else
        {
            setThrottle(MIN_THROTTLE_VALUE);
        }
        
        takeoff_requested = false;
    }
    else //reset hover commands
    {
        ptr_FIX->integral_hover = 0;
        ptr_FIX->derivative_hover = 0;
        ptr_FIX->prev_error_hover = 0;
        throttle_setpoint = MIN_THROTTLE_VALUE;
        takeoff_requested = false;
        land_requested = false;
    }

}

//this loop controls throttle with PID to navigate to desired height 
//it should take the desired height as a parameter 
//there is a nominal pwm throttle value at which the quadcopter hovers
//the PID should control around this nominal value
void run_hover_loop(unsigned int my_target_height)
{
    int hover_PID = 0;
    
    hover_PID = (int)PID_position((double)target_height, (double)actual_height, ALTITUDE);
    throttle_setpoint = guidance_v_nominal_throttle + hover_PID;
    
    Bound(throttle_setpoint,(MIN_THROTTLE_VALUE+20),(MAX_THROTTLE_VALUE-20));
    setThrottle(throttle_setpoint);
    //should only sum integral part when in flight (should be taken care of in main reset_i_part function)
}

void land_now(void)
{
    guidance_vert_init();   //prepare for automated guidance in vertical direction
    
    target_height = actual_height;  //maintain height
    switch(getMode())
    {
        case GUIDED:
                //immediately set throttle to hover speed
                throttle_setpoint = guidance_v_nominal_throttle;
        break;
        default:
        break;
    }
    
    land_requested = true;
}

float rad_to_degree(float radians)
{
    return (radians*180)/M_PI;
}

float deg_to_radian(float degrees)
{
    return (degrees*M_PI)/180;
}