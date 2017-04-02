/* 
 * File:   state.h
 * Author: Matt
 *
 * Created on 20. September 2016, 14:26
 */

#ifndef STATE_H
#define	STATE_H

#include "defines.h"
#include "MAVLink/include/common/mavlink.h" //include for access to mav_mode

//used just for arming/disarming/standby/etc
typedef enum e_vehicle_states
{
    UNKNOWN_STATE = -1,
    DISARMED = 0,
    ARMED = 1
} vehicle_states_t;

//used to determine origin of any requests for priority handling
typedef enum e_requestor
{
    UNKNOWN_REQUESTOR = -1,
    FAILSAFE = 0,
    TRANSMITTER = 1,
    GCS = 2,
    COMPANION_COMP = 3,
} requestor_t;


typedef enum e_mav_mode
{
    PREFLIGHT=0, /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
	TEST=2, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
    AUTO=4, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */
	GUIDED=8, /* System is allowed to be active, under autonomous control, manual setpoint | */
	STABILIZE=16, /* System is allowed to be active, under assisted RC control. | */
    HIL_EN=32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
    MANUAL=64, /* System is allowed to be active, under manual (RC) control, no stabilization | */
    //safety armed flag is set with ARMED/UNARMED flag
} mav_mode_t;

extern mav_mode_t vehicle_mode;         //global variable for vehicle mode
extern vehicle_states_t vehicle_state;   //global variable for vehicle state
extern uint8_t  vehicle_base_mode;      //global variable for vehicle base mode

void changeState(vehicle_states_t desired_state, requestor_t change_requestor);
vehicle_states_t getState(void);

void changeMode(mav_mode_t desired_mode, requestor_t change_requestor);
mav_mode_t getMode(void);

uint8_t getBaseMode();

void activateFailsafe();

#endif	/* STATE_H */

