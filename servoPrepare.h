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


#ifndef SERVOPREPARE_H
#define SERVOPREPARE_H
#include "p33EP512MU810.h"




extern int throttle_control;
extern int pitch_control;
extern int elevatorLoadingTrim;
extern int roll_control;
extern int yaw_control;
extern int wind_gain;

extern int tiltError[3];
extern int rotationRateError[3];
extern int desiredRotationRateRadians[3];

void init_servoPrepare(void);


//HEARTBEAT

// number of heartbeats per second
#define HEARTBEAT_HZ 40

// number of servo updates per second
#define SERVO_HZ 40

// frequency of PID loop (HEARTBEAT_HZ / PID_HZ must be an integer)
#define PID_HZ 40

// Read-only value increments with each heartbeat
extern unsigned int udb_heartbeat_counter;


inline void heartbeat(void);
inline unsigned int heartbeat_cnt(void);
inline unsigned char heartbeat_chk(unsigned int  hertz);


#endif // SERVOPREPARE_H
