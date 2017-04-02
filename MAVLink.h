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


#ifndef _MAVLINK_H_
#define _MAVLINK_H_

#include "nv_memory_options.h"
#include "mavlink_options.h"
#include "MAVLink/include/matrixpilot_mavlink_bridge_header.h"
#include "defines.h"

//#define MAVLINK_SEND_UART_BYTES mavlink_serial_send

//const declarations for set_attitude_target bitmask
//#define ACCEPT_ROLL_RATE     127
//#define ACCEPT_PITCH_RATE    191
//#define ACCEPT_YAW_RATE      223
//#define ACCEPT_THROTTLE      253
//#define ACCEPT_ATTITUDE      254     

#define IGNORE_ROLL_RATE     128
#define IGNORE_PITCH_RATE    64
#define IGNORE_YAW_RATE      32
#define IGNORE_THROTTLE      2
#define IGNORE_ATTITUDE      1    




typedef struct mavlink_flag_bits {
//	uint16_t unused                         : 2;
	uint16_t mavlink_send_specific_variable : 1;
	uint16_t mavlink_send_variables         : 1;
	uint16_t mavlink_send_waypoint_count    : 1;
	uint16_t mavlink_sending_waypoints      : 1;
	uint16_t mavlink_receiving_waypoints    : 1;
	uint16_t mavlink_send_specific_waypoint : 1;
	uint16_t mavlink_request_specific_waypoint : 1;
	uint16_t mavlink_send_waypoint_reached  : 1;
	uint16_t mavlink_send_waypoint_changed  : 1;
} mavlink_flags_t;

extern mavlink_flags_t mavlink_flags;

//boolean mavlink_check_target(uint8_t target_system, uint8_t target_component);



void init_mavlink(void);
void MAVLinkSetMode(mavlink_message_t* handle_msg);

boolean mavlink_check_target(uint8_t target_system, uint8_t target_component);

void mavlink_send();
void mavlink_output_40hz(void);

void reflectCmdLong(mavlink_message_t* handle_msg);
void SetAttitudeTarget(mavlink_message_t* handle_msg);

#ifdef USE_DEBUG_IO
#ifdef USE_MAVLINK_IO
void mav_printf(const char * format, ...);
#define DPRINT mav_printf
#endif // USE_MAVLINK_IO
#endif // USE_DEBUG_IO


#endif // _MAVLINK_H_
