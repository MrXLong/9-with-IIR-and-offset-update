// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


#ifndef GAIN_VARIABLES_H
#define GAIN_VARIABLES_H


#define GAINS_VARIABLE                      0

// Variable altitude and airspeed
// BEWARE: This uses an alternative library for altitude control
// Your aircraft will not behave the same way as when using non variable gains.
#define ALTITUDE_GAINS_VARIABLE             0

// Aileron/Roll Control Gains
// ROLLKP is the proportional gain, approximately 0.25
// ROLLKD is the derivative (gyro) gain, approximately 0.125
// YAWKP_AILERON is the proportional feedback gain for ailerons in response to yaw error
// YAWKD_AILERON is the derivative feedback gain for ailerons in response to yaw rotation
// AILERON_BOOST is the additional gain multiplier for the manually commanded aileron deflection

// MAVLINK, QGROUND CONTROL (Ground Control Station) can change these variables
extern float rollkp;
extern float rollki;
extern float rollkd;

extern float pitchkp;
extern float pitchki;
extern float pitchkd;

extern float yawkp;
extern float yawki;
extern float yawkd;

extern float hoverkp;
extern float hoverki;
extern float hoverkd;

extern int32_t nom_hov_thr;
extern int32_t min_throttle;

// Gains for Hovering
// Gains are named based on plane's frame of reference (roll means ailerons)
// HOVER_ROLLKP is the roll-proportional feedback gain applied to the ailerons while navigating a hover
// HOVER_ROLLKD is the roll gyro feedback gain applied to ailerons while stabilizing a hover
// HOVER_PITCHGAIN is the pitch-proportional feedback gain applied to the elevator while stabilizing a hover
// HOVER_PITCHKD is the pitch gyro feedback gain applied to elevator while stabilizing a hover
// HOVER_PITCH_OFFSET is the neutral pitch angle for the plane (in degrees) while stabilizing a hover
// HOVER_YAWKP is the yaw-proportional feedback gain applied to the rudder while stabilizing a hover
// HOVER_YAWKD is the yaw gyro feedback gain applied to rudder while stabilizing a hover
// HOVER_YAW_OFFSET is the neutral yaw angle for the plane (in degrees) while stabilizing a hover
// HOVER_PITCH_TOWARDS_WP is the max angle in degrees to pitch the nose down towards the WP while navigating
// HOVER_NAV_MAX_PITCH_RADIUS is the radius around a waypoint in meters, within which the HOVER_PITCH_TOWARDS_WP
//                            value is proportionally scaled down.

extern uint16_t hoverrollkp;
extern uint16_t hoverrollkd;
extern uint16_t hoverpitchgain;
extern uint16_t hoverpitchkd;
extern uint16_t hoveryawkp;
extern uint16_t hoveryawkd;

//#define HOVER_PITCH_OFFSET                0.0 // + leans towards top, - leans towards bottom
//#define HOVER_YAW_OFFSET                  0.0
//#define HOVER_PITCH_TOWARDS_WP            30.0
//#define HOVER_NAV_MAX_PITCH_RADIUS        20



#define ACCEL_RANGE         4       // 4 g range

// note : it is possible to use other accelerometer ranges on the MPU6000
#define SCALEGYRO           3.0016  // 500 degree/second range
//#define SCALEACCEL          1.29    // 4 g range
#define SCALEACCEL          1.27    // 4 g range measured by WJP on a few UDB5s


#endif //GAIN_VARIABLES_H
