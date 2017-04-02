#include "p33EP512MU810.h"

#include "state.h"
#include "MAVLink.h"    //for debug to screen
#include "Timer.h"      //transmitter watchdog

//the purpose of this file is to handle requests to change the vehicle state
//requests to change state can come from the remote controller or from
//the gcs, and could theoretically come from within the flight controller itself

vehicle_states_t vehicle_state;   //global variable for vehicle state
mav_mode_t vehicle_mode;            //global variable for vehicle mode
uint8_t vehicle_base_mode=0;   //global variable for vehicle base mode

void changeState(vehicle_states_t desired_state, requestor_t change_requestor)
{
    vehicle_states_t current_state = getState();
    
    switch(current_state)
    {
        case DISARMED:
            if(desired_state == ARMED)
            {
                if(change_requestor == TRANSMITTER)
                //we know that transmitter was active and we can look for loss of signal
                {
                    tx_watchdog_enabled = true;
                }
                    
                    
                //make checks and arm quad
                //set global state to armed and arm motors
                SDC1 = 5400;        //M1
                PDC1 = 5400;       //M2
                SDC2 = 5400;      //M3
                PDC2 = 5400;     //M4
                
                vehicle_state = ARMED;
            }
            else if(desired_state == DISARMED)
            {
                //return message that quad is already disarmed
                //no other action required?
                
                vehicle_state = DISARMED;
            }
            break;
        case ARMED:
            if(desired_state == DISARMED)
            {
                //make checks and disarm quad
                //set global state to disarmed and disarm motors
                //motors remain in standby until armed again
                SDC1 = 5400;        //M1
                PDC1 = 5400;       //M2
                SDC2 = 5400;      //M3
                PDC2 = 5400;     //M4
                
                vehicle_state = DISARMED;
            }
            else if(desired_state == ARMED)
            {
                //return message that quad is already armed
                //no other action required?
            }
            break;
        case UNKNOWN_STATE:
            //give error and disarm quad
            vehicle_state = UNKNOWN_STATE;
            break;
        default:
            break;
        
    }
}

vehicle_states_t getState()
{
    //needs a better check than this
    //assume what global variable says is true
    if(vehicle_state == ARMED)
        return ARMED;
    else if(vehicle_state == DISARMED)
        return DISARMED;
    //use LED latch
    return UNKNOWN_STATE;
}

//Mode Change requests coming from Transmitter should have higher priority
//than those coming from GCS
//FAILSAFE should have highest priority
void changeMode(mav_mode_t desired_mode, requestor_t change_requestor)
{
    mav_mode_t current_mode = getMode();
    
    switch(current_mode)
    {
        case PREFLIGHT:
            //maybe do a motor/sensor check before allowing mode change from here
            if(desired_mode == STABILIZE)
                vehicle_mode = STABILIZE;
            else
                //placeholder
        break;
        case TEST:
            //placeholder
        break;
        case AUTO:
            //placeholder for future development
        break;
        case GUIDED:
            if(desired_mode == STABILIZE)
            {
                //quad should immediately stop movement and hover in place
                //before switching control to RC
                vehicle_mode = STABILIZE;
            }
            else if(desired_mode == PREFLIGHT)
            {
                //quad should land first
                vehicle_mode = PREFLIGHT;
            }
            else
                //placeholder
        break;
        case STABILIZE:
            if(desired_mode == GUIDED)
            {
                //should make some basic checks before switching modes
                vehicle_mode = GUIDED;
            }
            else if(desired_mode == PREFLIGHT)
            {
                //quad should be landed first
                vehicle_mode = PREFLIGHT;
            }
        break;
        case HIL_EN:
            //placeholder
        break;
        case MANUAL:
            //placeholder
        break;
        default:    //not sure what best default mode is here
        break;
    }
    
    
}

mav_mode_t getMode()
{
    //needs a better check than this
    //assume what global variable says is true
    switch(vehicle_mode)
    {
        case PREFLIGHT:
            return PREFLIGHT;
        break;
        case TEST:
            return TEST;
        break;
        case AUTO:
            return AUTO;
        break;
        case GUIDED:
            return GUIDED;
        break;
        case STABILIZE:
            return STABILIZE;
        break;
        case HIL_EN:
            return HIL_EN;
        break;
        case MANUAL:
            return MANUAL;
        break;
        default:    //not sure what best default mode is here
            return STABILIZE;
        break;
    }
    
}

//function used only for getting the mavlink compatible state by
//combining armed/unarmed information with mav_mode information
uint8_t getBaseMode()
{
    vehicle_states_t armed_state = getState();
    mav_mode_t mav_mode = getMode();
    uint8_t base_mode = 0;
    
    switch(mav_mode)
    {
        case PREFLIGHT:
            base_mode |= PREFLIGHT;
        break;
        case TEST:
            //placeholder
        break;
        case AUTO:
            //placeholder
        break;
        case GUIDED:
            base_mode |= (GUIDED+STABILIZE+MANUAL);
        break;
        case STABILIZE:
            base_mode |= (STABILIZE+MANUAL);
        break;
        case HIL_EN:
            //placeholder
        break;
        case MANUAL:
            base_mode |= MANUAL;
        break;
    }
    
    if(armed_state==ARMED)
        base_mode |= 128; //MAV_MODE_FLAG_SAFETY_ARMED
    
    //update global variable
    vehicle_base_mode = base_mode;
    
    return base_mode;
}

void activateFailsafe(void)
{
    DPRINT("ENTERING FAILSAFE \r\n");
    //changeMode(GUIDED, FAILSAFE);
    //change mode to guided so GCS can control if necessary
    
    //initiate landing sequence
    //land_now();
}

