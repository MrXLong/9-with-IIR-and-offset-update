// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009, 2010 MatrixPilot Team
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


//#include "defines.h"

#include <stdio.h>
#include "MAVLink.h"
#include "MAVParams.h"
#include "MAVLink/include/common/mavlink.h"
#include "nv_memory_options.h"

#include "options.h"

#include <math.h>

#include "parameter_table.h"
#include "analog2digital.h" //for def of adc channels

#if (USE_NV_MEMORY == 1)
#include "data_services.h"
#endif


int16_t magGain[3] = { 16384 , 16384 , 16384 };    // magnetometer calibration gains
int16_t rawMagCalib[3] = { 0 , 0 , 0 };
int16_t magFieldRaw[3];
int16_t udb_magFieldBody[3];                    // magnetic field in the body frame of reference 
int16_t udb_magOffset[3] = { 0 , 0 , 0 };       // magnetic offset in the body frame of reference

int16_t height_target_min;
int16_t height_target_max;
int16_t height_margin;
fractional alt_hold_throttle_min;
fractional alt_hold_throttle_max;
int16_t alt_hold_pitch_min;
int16_t alt_hold_pitch_max;
int16_t alt_hold_pitch_high;
int16_t rtl_pitch_down;
int16_t desiredSpeed;

union intbb dcm_declination_angle;
int16_t udb_pwTrim[];  


struct ADchannel udb_vref;  

int16_t minimum_groundspeed;
int16_t minimum_airspeed;
int16_t maximum_airspeed;
int16_t cruise_airspeed;

int16_t airspeed_pitch_adjust_rate;
int16_t airspeed_pitch_ki_limit;
fractional airspeed_pitch_ki;
int16_t airspeed_pitch_min_aspd;
int16_t airspeed_pitch_max_aspd;

int16_t send_variables_counter = 0;
int16_t send_by_index = 0;

extern uint16_t maxstack;

// ROUTINES FOR CHANGING UAV ONBOARD PARAMETERS
// All paramaters are sent as type (mavlink_param_union_t) between Ground Control Station and MatrixPilot.
// So paramaters have to be converted between type (mavlink_param_union_t) and their normal representation.
// An explanation of the MAVLink protocol for changing paramaters can be found at:
// http://www.qgroundcontrol.org/parameter_interface



boolean mavlink_parameter_out_of_bounds(mavlink_param_union_t parm, int16_t i)
{
	switch (mavlink_parameter_parsers[mavlink_parameters_list[i].udb_param_type].mavlink_type)
	{
		case MAVLINK_TYPE_FLOAT:
			if (parm.param_float < mavlink_parameters_list[i].min.param_float)
				return true;
			if (parm.param_float > mavlink_parameters_list[i].max.param_float)
				return true;
			break;
		case MAVLINK_TYPE_UINT32_T:
			if (parm.param_int32 < mavlink_parameters_list[i].min.param_int32)
				return true;
			if (parm.param_int32 > mavlink_parameters_list[i].max.param_int32)
				return true;
			break;
		case MAVLINK_TYPE_INT32_T:
			if (parm.param_int32 < mavlink_parameters_list[i].min.param_int32)
				return true;
			if (parm.param_int32 > mavlink_parameters_list[i].max.param_int32)
				return true;
			break;
		default:
			return true;
			break;
	}
	return false;
}

void mavlink_send_param_float(int16_t i)
{
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    *((float*)mavlink_parameters_list[i].pparam),
	    MAVLINK_TYPE_FLOAT, count_of_parameters_list, i);
}

void mavlink_set_param_float(mavlink_param_union_t setting, int16_t i)
{
	if (setting.type != MAVLINK_TYPE_FLOAT) return;

	*((float*)mavlink_parameters_list[i].pparam) = setting.param_float;
}

void mavlink_send_param_gyroscale_Q14(int16_t i)
{
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    (float)(*((int16_t*) mavlink_parameters_list[i].pparam) / (SCALEGYRO * 16384.0)),
	    MAVLINK_TYPE_FLOAT, count_of_parameters_list, i); // 16384.0 is RMAX defined as a float.
}

void mavlink_set_param_gyroscale_Q14(mavlink_param_union_t setting, int16_t i)
{
	if (setting.type != MAVLINK_TYPE_FLOAT) return;

	*((int16_t*)mavlink_parameters_list[i].pparam) = (int16_t)(setting.param_float * (SCALEGYRO * 16384.0));
}

void mavlink_send_param_Q14(int16_t i)
{
#if (QGROUNDCTONROL_PID_COMPATIBILITY == 1) // see mavlink_options.h for details
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    (floor((((float)(*((int16_t*)mavlink_parameters_list[i].pparam) / 16384.0)) * 10000) + 0.5) / 10000.0),
	    MAVLINK_TYPE_FLOAT, count_of_parameters_list, i); // 16384.0 is RMAX defined as a float.
#else
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    (float)(*((int16_t*) mavlink_parameters_list[i].pparam) / 16384.0),
	    MAVLINK_TYPE_FLOAT, count_of_parameters_list, i); // 16384.0 is RMAX defined as a float.
#endif
}

void mavlink_set_param_Q14(mavlink_param_union_t setting, int16_t i)
{
	if (setting.type != MAVLINK_TYPE_FLOAT) return;

	*((int16_t*)mavlink_parameters_list[i].pparam) = (int16_t)(setting.param_float * 16384.0);
}

int16_t udb_pwIn[NUM_INPUTS+1];     // pulse widths of radio inputs
int16_t udb_pwTrim[NUM_INPUTS+1];   // initial pulse widths for trimming

void mavlink_send_param_pwtrim(int16_t i)
{
	// Check that the size of the udb_pwtrim array is not exceeded
	if (mavlink_parameters_list[i].pparam >= (uint8_t*)(&udb_pwTrim[0] + (sizeof(udb_pwTrim[0]) * NUM_INPUTS)))
		return;

	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    (float)(*((int16_t*) mavlink_parameters_list[i].pparam) / 2.0),
	    MAVLINK_TYPE_FLOAT, count_of_parameters_list, i); // 16384.0 is RMAX defined as a float.
}

void mavlink_set_param_pwtrim(mavlink_param_union_t setting, int16_t i)
{
	if (setting.type != MAVLINK_TYPE_FLOAT) return;

	// Check that the size of the ubb_pwtrim array is not exceeded
	if (mavlink_parameters_list[i].pparam >= (uint8_t*)(&udb_pwTrim[0] + (sizeof(udb_pwTrim[0]) * NUM_INPUTS)))
		return;
	*((int16_t*)mavlink_parameters_list[i].pparam) = (int16_t)(setting.param_float * 2.0);
}

void mavlink_send_param_int16(int16_t i)
{
	param_union_t param;

	param.param_int32 = *((int16_t*)mavlink_parameters_list[i].pparam);
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    param.param_float, MAVLINK_TYPE_INT32_T, count_of_parameters_list, i); // 16384.0 is RMAX defined as a float.
}

void mavlink_set_param_int16(mavlink_param_union_t setting, int16_t i)
{
	if (setting.type != MAVLINK_TYPE_INT32_T) return;

	*((int16_t*)mavlink_parameters_list[i].pparam) = (int16_t) setting.param_int32;
}

void mavlink_send_int_circular(int16_t i)
{
	param_union_t param;
	union longww deg_angle;

	deg_angle.WW = 0;
	deg_angle._.W0 = *((int16_t*) mavlink_parameters_list[i].pparam);
	deg_angle.WW = __builtin_mulss(deg_angle._.W0, (int16_t)(16384 * 180.0 / 256.0));
	deg_angle.WW >>= 5;
	if (deg_angle._.W0 > 0x8000) deg_angle._.W1++; // Take care of the rounding error
	param.param_int32 = deg_angle._.W1; // >> 6;
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    param.param_float, MAVLINK_TYPE_INT32_T, count_of_parameters_list, i);
}

void mavlink_set_int_circular(mavlink_param_union_t setting, int16_t i)
{
	union longww dec_angle;

	if (setting.type != MAVLINK_TYPE_INT32_T) return;

	dec_angle.WW = __builtin_mulss((int16_t)setting.param_int32, (int16_t)(16384 * (256.0 / 180.0)));
	dec_angle.WW <<= 9;
	if (dec_angle._.W0 > 0x8000) dec_angle.WW += 0x8000; // Take care of the rounding error
	*((int16_t*)mavlink_parameters_list[i].pparam) = dec_angle._.W1;
}

void mavlink_send_dm_airspeed_in_cm(int16_t i)
{
	param_union_t param;
	union longww airspeed;

	airspeed.WW = 0;
	airspeed._.W0 = *((int16_t*)mavlink_parameters_list[i].pparam);
	airspeed.WW = __builtin_mulss(airspeed._.W0, 10.0);
	param.param_int32 = airspeed._.W0;
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    param.param_float, MAVLINK_TYPE_INT32_T, count_of_parameters_list, i);
}

void mavlink_set_dm_airspeed_from_cm(mavlink_param_union_t setting, int16_t i)
{
	union longww airspeed;

	if (setting.type != MAVLINK_TYPE_INT32_T) return;

	airspeed.WW = __builtin_mulss((int16_t)setting.param_int32, (16384 / 10.0));
	airspeed.WW <<= 2;
	*((int16_t*)mavlink_parameters_list[i].pparam) = airspeed._.W1;
}

void mavlink_send_cm_airspeed_in_m(int16_t i)
{
	param_union_t param;

	param.param_float = (float)*((int16_t*)mavlink_parameters_list[i].pparam);
	param.param_float *= 0.01;
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    param.param_float, MAVLINK_TYPE_FLOAT, count_of_parameters_list, i);
}

void mavlink_set_cm_airspeed_from_m(mavlink_param_union_t setting, int16_t i)
{
	union longww airspeed;

	if (setting.type != MAVLINK_TYPE_FLOAT) return;

	airspeed.WW = __builtin_mulss((int16_t)setting.param_int32, (16384 / 10.0));
	airspeed.WW <<= 2;
	*((int16_t*)mavlink_parameters_list[i].pparam) = (int16_t)(setting.param_float * 100.0);
}

void mavlink_send_dm_airspeed_in_m(int16_t i)
{
	param_union_t param;

	param.param_float = (float)*((int16_t*)mavlink_parameters_list[i].pparam);
	param.param_float *= 0.1;
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    param.param_float, MAVLINK_TYPE_FLOAT, count_of_parameters_list, i);
}

void mavlink_set_dm_airspeed_from_m(mavlink_param_union_t setting, int16_t i)
{
	union longww airspeed;

	if (setting.type != MAVLINK_TYPE_FLOAT) return;

	airspeed.WW = __builtin_mulss((int16_t)setting.param_int32, (16384 / 10.0));
	airspeed.WW <<= 2;
	*((int16_t*)mavlink_parameters_list[i].pparam) = (int16_t)(setting.param_float * 10.0);
}

// send angle in dcm units
void mavlink_send_dcm_angle(int16_t i)
{
	param_union_t param;
	union longww deg_angle;

	deg_angle.WW = 0;
	deg_angle._.W0 = *((int16_t*)mavlink_parameters_list[i].pparam);
//	deg_angle.WW = __builtin_mulss(deg_angle._.W0, 40);
	deg_angle.WW = __builtin_mulss(deg_angle._.W0, (int16_t)(57.3 * 16.0)); //(RMAX * 180.0 / 256.0));
	deg_angle.WW >>= 2;
	if (deg_angle._.W0 > 0x8000) deg_angle._.W1++; // Take care of the rounding error
	param.param_int32 = deg_angle._.W1; // >> 6;
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    param.param_float, MAVLINK_TYPE_INT32_T, count_of_parameters_list, i);
}

// set angle in dcm units
void mavlink_set_dcm_angle(mavlink_param_union_t setting, int16_t i)
{
	union longww dec_angle;

	if (setting.type != MAVLINK_TYPE_INT32_T) return;

	dec_angle.WW = __builtin_mulss((int16_t)setting.param_int32, (16384 * (16.0 / 57.3))); //(int16_t)(RMAX * 64 / 57.3)
	dec_angle.WW <<= 12;
	if (dec_angle._.W0 > 0x8000) dec_angle.WW += 0x8000; // Take care of the rounding error
	*((int16_t*)mavlink_parameters_list[i].pparam) = dec_angle._.W1;
}

// send angle rate in units of angle per frame
void mavlink_send_frame_anglerate(int16_t i)
{
	param_union_t param;
	union longww deg_angle;

	deg_angle.WW = 0;
	deg_angle._.W0 = *((int16_t*)mavlink_parameters_list[i].pparam);
//	deg_angle.WW = __builtin_mulss(deg_angle._.W0, 40);
	deg_angle.WW = __builtin_mulss(deg_angle._.W0, (int16_t)(57.3 * 40.0)); //(RMAX * 180.0 / 256.0));
	deg_angle.WW <<= 2;
	if (deg_angle._.W0 > 0x8000) deg_angle._.W1++; // Take care of the rounding error
	param.param_int32 = deg_angle._.W1; // >> 6;
	mavlink_msg_param_value_send(MAVLINK_COMM_0, mavlink_parameters_list[i].name,
	    param.param_float, MAVLINK_TYPE_INT32_T, count_of_parameters_list, i);
}

// set angle rate in units of angle per frame
void mavlink_set_frame_anglerate(mavlink_param_union_t setting, int16_t i)
{
	union longww dec_angle;

	if (setting.type != MAVLINK_TYPE_INT32_T) return;

	dec_angle.WW = __builtin_mulss((int16_t)setting.param_int32, (128.0 * 7.15)); //(int16_t)(RMAX * 128 / (57.3 * 40.0))
	dec_angle.WW <<= 9;
	if (dec_angle._.W0 > 0x8000) dec_angle.WW += 0x8000; // Take care of the rounding error
	*((int16_t*)mavlink_parameters_list[i].pparam) = dec_angle._.W1;
}

// END OF GENERAL ROUTINES FOR CHANGING UAV ONBOARD PARAMETERS

static int16_t get_param_index(const char* key)
{
	int16_t i;

	// iterate known parameters
	for (i = 0; i < count_of_parameters_list; i++)
	{
		// compare key with parameter name
		if (!strcmp(key, (const char*)mavlink_parameters_list[i].name)) // TODO: why are we casting this to const, should not be required - RobD
		{
			return i;
		}
	}
	//DPRINT("unknown parameter name: %s\r\n", key);
	return -1;
}


void MAVParamsSet(const mavlink_message_t* handle_msg)
{
	mavlink_param_set_t packet;
	int16_t i;

	mavlink_msg_param_set_decode(handle_msg, &packet);
	if (mavlink_check_target(packet.target_system, packet.target_component) == true)
	{
		DPRINT("\tfailed target system check on parameter set\r\n");
//		send_text((uint8_t*)"failed target system check on parameter set \r\n");
//		break;
	}
	else
	{
		// set parameter
		i = get_param_index((const char*)packet.param_id);
		if (i != -1)
		{
			mavlink_param_union_t param;
			param.type = packet.param_type;
			param.param_float = packet.param_value;

			if ((mavlink_parameters_list[i].readonly == false) &&
			    (mavlink_parameter_out_of_bounds(param, i) == false))
			{
				mavlink_parameter_parsers[mavlink_parameters_list[i].udb_param_type].set_param(param, i);
				DPRINT("parameter[%i] %s, %f set\r\n", i, (const char*)packet.param_id, (double)param.param_float);
                //this should normally not be necessary, but in the current version
                //of the code structure, it is needed to set the PID values of our
                //controller structure to the global PID values which MAVLink reads and changes - MW
                set_global_PID();
			}
			else
			{
				DPRINT("parameter[%i] %s, %f out of bounds\r\n", i, (const char*)packet.param_id, (double)param.param_float);
			}
			// Send the parameter back to GCS as acknowledgement of success, or otherwise
			if (mavlink_flags.mavlink_send_specific_variable == 0)
			{
				send_by_index = i;
				mavlink_flags.mavlink_send_specific_variable = 1;
			}
		}
		else
		{
			//DPRINT("Attempt to set unknown parameter name: %s\r\n", (const char*)packet.param_id);
		}
/*
		// iterate known parameters
		for (i = 0; i < count_of_parameters_list; i++)
		{
			// compare key with parameter name
			if (!strcmp(key, (const char*)mavlink_parameters_list[i].name)) // TODO: why are we casting this to const, should not be required - RobD
			{
				mavlink_param_union_t param;
				param.type = packet.param_type;
				param.param_float = packet.param_value;

				DPRINT("found parameter[%i] %s, %f\r\n", i, key, param.param_float);

				if ((mavlink_parameters_list[i].readonly == false) &&
				    (mavlink_parameter_out_of_bounds(param, i) == false))
				{
					mavlink_parameter_parsers[mavlink_parameters_list[i].udb_param_type].set_param(param, i);
					DPRINT("parameter set\r\n");
				}
				{
					// After setting parameter, re-send it to GCS as acknowledgement of success.
					if (mavlink_flags.mavlink_send_specific_variable == 0)
					{
						send_by_index = i;
						mavlink_flags.mavlink_send_specific_variable = 1;
					}
				}
			}
		}
 */
	}
}

void MAVParamsRequestList(const mavlink_message_t* handle_msg)
{
	mavlink_param_request_list_t packet;

	//send_text((uint8_t*)"param request list\r\n");
	//DPRINT("param request list\r\n");
	mavlink_msg_param_request_list_decode(handle_msg, &packet);
	if (packet.target_system == mavlink_system.sysid)
	{
		// Start sending parameters
		send_variables_counter = 0;
		mavlink_flags.mavlink_send_variables = 1;
	}
}

void MAVParamsRequestRead(const mavlink_message_t* handle_msg)
{
	mavlink_param_request_read_t packet;

	mavlink_msg_param_request_read_decode(handle_msg, &packet);
	if (packet.target_system == mavlink_system.sysid)
	{
		packet.param_index = get_param_index((const char*)packet.param_id);
		//DPRINT("Requested specific parameter %u %s\r\n", packet.param_index, (const char*)packet.param_id);
		if ((packet.param_index >= 0) && (packet.param_index <= count_of_parameters_list))
		{
			send_by_index = packet.param_index;
			mavlink_flags.mavlink_send_specific_variable = 1;
			//DPRINT("Sending specific parameter\r\n");
		}
	}
}

boolean MAVParamsHandleMessage(mavlink_message_t* handle_msg)
{
	switch (handle_msg->msgid)
	{
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{
            mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Request for Param List");
			MAVParamsRequestList(handle_msg);
			break;
		}
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		{
            mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Request for Param List");
			MAVParamsRequestRead(handle_msg);
			break;
		}
		case MAVLINK_MSG_ID_PARAM_SET:
		{
			MAVParamsSet(handle_msg);
			break;
		}
		default:
			return false;
	}
	return true;
}

void MAVParamsOutput_40hz(void)
{
	// SEND VALUES OF PARAMETERS IF THE LIST HAS BEEN REQUESTED
	if (mavlink_flags.mavlink_send_variables == 1)
	{
		if (send_variables_counter < count_of_parameters_list)
		{
            //{&mavlink_send_param_int16, &mavlink_set_param_int16, MAVLINK_TYPE_INT32_T}[{"PID_ROLLKP",{.param_float=0.0},{.param_float=0.5},UDB_TYPE_Q14,PARAMETER_READWRITE,(void*)&rollkp,sizeof(rollkp)}[0].0]
		    mavlink_parameter_parsers[mavlink_parameters_list[send_variables_counter].udb_param_type].send_param(send_variables_counter);
            //mavlink_msg_param_value_send(MAVLINK_COMM_0,"TEST\n", 123.456, MAVLINK_TYPE_FLOAT, 1, 1);
			send_variables_counter++;
		}
		else
		{
			send_variables_counter = 0;
			mavlink_flags.mavlink_send_variables = 0;
		}
	}

	// SEND SPECIFICALLY REQUESTED PARAMETER
	if (mavlink_flags.mavlink_send_specific_variable == 1)
	{
		mavlink_parameter_parsers[mavlink_parameters_list[send_by_index].udb_param_type].send_param(send_by_index);
		mavlink_flags.mavlink_send_specific_variable = 0;
	}
}
