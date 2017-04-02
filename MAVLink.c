//Matt Woodard

#include "MAVLink.h"
#include "MAVLink/include/common/mavlink.h"
#include "serial.h"         //functionality for serial port
#include "PID.h"            //access controller data structure
#include "InputCapture.h"   //to get channel values
#include "Timer.h"
#include "MAVParams.h"  
#include "state.h"  //to change states
#include "Mixer.h"  //For mixer values
#include "SRF02.h"  //for ultrasonic info
#include "autopilot.h"  //for commanded movements
#include "data_services.h" //for loading and saving configuration
#include "data_storage.h"
#include "analog2digital.h" //for battery voltage
#include "SD-SPI.h" //debug
#include "Filter.h"

#define _ADDED_C_LIB 1  //needed to get vsnprintf
#include <stdarg.h>
#include <stdio.h>


mavlink_system_t mavlink_system;
mavlink_flags_t mavlink_flags;


////////////////////////////////////////////////////////////////////////////////////////
//
// MAIN MATRIXPILOT MAVLINK CODE FOR RECEIVING COMMANDS FROM THE GROUND CONTROL STATION
//

uint8_t streamRates[MAV_DATA_STREAM_ENUM_END];

uint16_t mavlink_command_ack_command = 0;
boolean mavlink_send_command_ack = false;
uint16_t mavlink_command_ack_result = 0;
boolean handling_of_message_completed = true;

uint8_t mavlink_counter_40hz = 0;
uint64_t usec = 0; // A measure of time in microseconds (should be from Unix Epoch).
uint32_t msec = 0; // A measure of time in microseconds (should be from Unix Epoch).

static void handleMessage(void);

inline void preflight_storage_complete_callback(boolean success);


uint8_t mavlink_message_index = 0;
mavlink_status_t r_mavlink_status;

mavlink_message_t msg[2];
mavlink_status_t r_mavlink_status;

// Define global system variables
uint8_t companion_id = COMPANION_ID;    //given in options file

uint8_t system_type = MAV_TYPE_QUADROTOR;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC; //MAV_AUTOPILOT_GENERIC
uint8_t system_mode = MAV_STATE_STANDBY; ///< Booting up
uint8_t base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED  /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
                    &~MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 
                    &~MAV_MODE_FLAG_TEST_ENABLED  /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
                    &~MAV_MODE_FLAG_AUTO_ENABLED  /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
                    &~MAV_MODE_FLAG_GUIDED_ENABLED  /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
                    &~MAV_MODE_FLAG_HIL_ENABLED  /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
                    &~MAV_MODE_FLAG_MANUAL_INPUT_ENABLED  /* 0b01000000 remote control input is enabled. | */
                    &~MAV_MODE_FLAG_SAFETY_ARMED 
                    &~MAV_MODE_FLAG_ENUM_END; /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */

uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter


uint8_t system_status = MAV_STATE_STANDBY; ///< System ready for flight



uint64_t capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
                MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
                MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;

uint32_t onboard_control_sensors =  MAV_SYS_STATUS_SENSOR_3D_GYRO |
                                    MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                    MAV_SYS_STATUS_SENSOR_3D_MAG |
                                    MAV_SYS_STATUS_SENSOR_GPS |
                                    MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
                                    MAV_SYS_STATUS_SENSOR_RC_RECEIVER;

void init_mavlink(void)
{
    get_global_PID();   //get PID values for MAVLink... this is a hack until PID file structure is changed
    
	int16_t index;
                    
	handleMessage();
	mavlink_system.sysid = MAVLINK_SYSID; // System ID, 1-255, ID of your Plane for GCS
	mavlink_system.compid = 1; // Component/Subsystem ID,  (1-255) dsPIC on quad is component 1.

	// Fill stream rates array with zeros to default all streams off;
	for (index = 0; index < MAV_DATA_STREAM_ENUM_END; index++)
		streamRates[index] = 0;

	// QGroundControl GCS lets user send message to increase stream rate
	streamRates[MAV_DATA_STREAM_RC_CHANNELS] = MAVLINK_RATE_RC_CHAN;
	streamRates[MAV_DATA_STREAM_RAW_SENSORS] = MAVLINK_RATE_RAW_SENSORS;
	streamRates[MAV_DATA_STREAM_POSITION]    = MAVLINK_RATE_POSITION;
	streamRates[MAV_DATA_STREAM_EXTRA1]      = MAVLINK_RATE_SUE;
	streamRates[MAV_DATA_STREAM_EXTRA2]      = MAVLINK_RATE_POSITION_SENSORS;
    
//    uint32_t flight_sw = 50593800;
//    uint32_t mid_sw = 0;
//    uint32_t os_sw = 0;
//    uint32_t board = 0;
//    uint8_t version[8];
//    uint8_t v2[8];
//    uint8_t v3[8];
//    memset(v2, 0, 8 * sizeof(v2[0]));
//    memset(v3, 0, 8 * sizeof(v2[0]));
//    
//    version[0] = 100;
//    version[1] = 48;
//    version[2] = 100;
//    version[3] = 100;
//    version[4] = 49;
//    version[5] = 48;
//    version[6] = 97;
//    version[7] = 101;
//    
//    capabilities = 3023;
//    
//    mavlink_msg_autopilot_version_send(MAVLINK_COMM_0,capabilities,flight_sw,mid_sw,os_sw,board,version,v2,v3,0,0,0);
}





void mav_msg_receive(uint8_t rxchar)
{
	if (mavlink_parse_char(0, rxchar, &msg[mavlink_message_index], &r_mavlink_status))
	{
		// Check that handling of previous message has completed before calling again
		if (handling_of_message_completed == true)
		{
			// Switch between incoming message buffers
			if (mavlink_message_index == 0) mavlink_message_index = 1;
			else mavlink_message_index = 0;
			handling_of_message_completed = false;
            handleMessage();
        }
    }
}


boolean mavlink_check_target(uint8_t target_system, uint8_t target_component)
{
	if ((target_system == mavlink_system.sysid)
	    // QGroundControl sends parameter refresh list to component 25 (regardless)
	    // But "Transmit" of parameter updates are sent using a specific component ID of 1 by QGroundControl.
	    // Only use mavlink_check_target if you expect all of the sysid, and compid to be correct.
	    && (target_component == mavlink_system.compid))
	{
		return false;
	}
	else
	{
		return false;
//		return true; 
	}
}

static void command_ack(uint16_t command, uint16_t result)
{
	if (mavlink_send_command_ack == false)
	{
		mavlink_command_ack_result = result;
		mavlink_command_ack_command = command;
		mavlink_send_command_ack = true;
	}
}

void MAVLinkRequestDataStream(mavlink_message_t* handle_msg) // MAVLINK_MSG_ID_REQUEST_DATA_STREAM
{
	int16_t freq = 0; // packet frequency
	mavlink_request_data_stream_t packet;
	mavlink_msg_request_data_stream_decode(handle_msg, &packet);

	DPRINT("MAVLINK_MSG_ID_REQUEST_DATA_STREAM %u\r\n", handle_msg->msgid);
	// QgroundControl sends data stream request to component ID 1, which is not our component for UDB.
	if (packet.target_system != mavlink_system.sysid) return;

	if (packet.start_stop == 0) freq = 0; // stop sending
	else if (packet.start_stop == 1) freq = packet.req_message_rate; // start sending
	else return;
	if (packet.req_stream_id == MAV_DATA_STREAM_ALL)
	{
		// Warning: mavproxy automatically sets all.  Do not include all here, it will overide defaults.
		streamRates[MAV_DATA_STREAM_RAW_SENSORS] = freq;
		streamRates[MAV_DATA_STREAM_RC_CHANNELS] = freq;
	}
	else
	{
		if (packet.req_stream_id < MAV_DATA_STREAM_ENUM_END)
			streamRates[packet.req_stream_id] = freq;
	}
}

void MAVLinkCommandLong(mavlink_message_t* handle_msg) // MAVLINK_MSG_ID_COMMAND_LONG
{
	mavlink_command_long_t packet;
	mavlink_msg_command_long_decode(handle_msg, &packet);

    DPRINT("MAVLINK_MSG_ID_COMMAND_LONG %u\r\n", handle_msg->msgid);
	//if (mavlink_check_target(packet.target, packet.target_component) == false) break;
	{
		switch (packet.command)
		{
//#if (USE_NV_MEMORY == 1)
//			case MAV_CMD_PREFLIGHT_STORAGE:
//				DPRINT("MAV_CMD_PREFLIGHT_STORAGE %u\r\n", packet.command);
//				if (packet.param1 == MAV_PFS_CMD_WRITE_ALL) // 1
//				{
//					if (packet.param2 == MAV_PFS_CMD_WRITE_ALL)
//						data_services_save_all(STORAGE_FLAG_STORE_CALIB | STORAGE_FLAG_STORE_WAYPOINTS, &preflight_storage_complete_callback);
//					else
//						data_services_save_all(STORAGE_FLAG_STORE_CALIB, &preflight_storage_complete_callback);
//				}
//				else if (packet.param1 == MAV_PFS_CMD_READ_ALL) // 0
//				{
//					if (packet.param2 == MAV_PFS_CMD_READ_ALL)
//						data_services_load_all(STORAGE_FLAG_STORE_CALIB | STORAGE_FLAG_STORE_WAYPOINTS, &preflight_storage_complete_callback);
//					else
//						data_services_load_all(STORAGE_FLAG_STORE_CALIB, &preflight_storage_complete_callback);
//				}
//				else
//					command_ack(packet.command, MAV_CMD_ACK_ERR_NOT_SUPPORTED);
//				break;
//			case MAV_CMD_PREFLIGHT_STORAGE_ADVANCED:
//				DPRINT("MAV_CMD_PREFLIGHT_STORAGE_ADVANCED %u\r\n", packet.command);
//				switch ((uint16_t)packet.param1)
//				{
//					case MAV_PFS_CMD_CLEAR_SPECIFIC:
//						storage_clear_area(packet.param2, &preflight_storage_complete_callback);
//						break;
//					case MAV_PFS_CMD_WRITE_SPECIFIC:
//						data_services_save_specific(packet.param2, &preflight_storage_complete_callback);
//						break;
//					case MAV_PFS_CMD_READ_SPECIFIC:
//						data_services_load_specific(packet.param2, &preflight_storage_complete_callback);
//						break;
//					default:
//						command_ack(packet.command, MAV_CMD_ACK_ERR_NOT_SUPPORTED);
//						break;
//				}
//				break;
//#endif // (USE_NV_MEMORY == 1)
			case MAV_CMD_PREFLIGHT_STORAGE:
                DPRINT("MAV_CMD_PREFLIGHT_STORAGE %u\r\n", packet.command);
				switch ((uint16_t)packet.param1)
				{
					case 0: // Read
						DPRINT("Read (ROM)\r\n");
						//config_load();
                        //data_services_load_all(STORAGE_FLAG_STORE_CALIB, &preflight_storage_complete_callback);
						break;
					case 1: // Write
						DPRINT("Write (ROM)\r\n");
						//data_services_save_all(STORAGE_FLAG_STORE_CALIB, &preflight_storage_complete_callback);
						break;
					default:
						DPRINT("245 packet.param1 %f packet.param2 %f\r\n", (double)packet.param1, (double)packet.param2);
						break;
				}
			case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: // halt
				DPRINT("Reboot/Shutdown - packet.command %u\r\n", packet.command);
				break;
			case MAV_CMD_NAV_TAKEOFF: // start
				DPRINT("Takeoff and hover at %f meters\r\n", packet.param7);
                //check that we are not in flight
                //if in flight send back error
                //otherwise send message preparing for takeoff and hover to given
                //height and location
                
                //first step is to make simple takeoff using only quad capabilities
                //second step is that RPi will catch this command and strip out 
                //desired latitude and longitude and altitude and make computations
                //to get to those and give those to quad as move cmd.
                simple_takeoff((unsigned int)(100*packet.param7)); //takeoff to given height -- convert meters to cm
                //setPitchAngle(packet.param1);
                //setYawAngle(packet.param4);
                command_ack(packet.command, MAV_RESULT_ACCEPTED);
				break;
			case MAV_CMD_NAV_LAND: // land
				DPRINT("Land - packet.command %u\r\n", packet.command);
                //currently this method doesnt use an parameters, it just lands
                //in place using the ultrasonic sensor
                land_now();
                command_ack(packet.command, MAV_RESULT_ACCEPTED);
				break;
            
            case MAV_CMD_DO_SET_MODE:
            {
                    MAVLinkSetMode(handle_msg); //can set this function to return acknowledgement result depending on if mode is supported or not
                    command_ack(packet.command, MAV_RESULT_ACCEPTED);
            break;
            }
            case MAV_CMD_COMPONENT_ARM_DISARM:
                if(packet.param1>0.5) // arm
                {
                    mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Arm Requested");
                    changeState(ARMED, GCS);
                }
                else    //assume disarm
                {
                    mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Disarm Requested");
                    changeState(DISARMED, GCS);
                }
                command_ack(packet.command, MAV_RESULT_ACCEPTED);
            break;
            case MAV_CMD_CONDITION_YAW:
                if(getMode()==GUIDED)
                {
                    DPRINT("Set Yaw to %f degrees \r\n", packet.param1, packet.param3);
                    setThrottle(MIN_THROTTLE_VALUE+10);    //FOR DEMO PURPOSES ONLY
                    setYawAngle(packet.param3*packet.param1);
                    setYawRate(packet.param2);
                }
            break;
            case MAV_CMD_DO_SET_PARAMETER:
                DPRINT("Set individual parameter %f with %f\r\n", packet.param1, packet.param2);
                MAVParamsSet(handle_msg);
            break;
            //video control is via companion computer
            //reflect the message back
            case MAV_CMD_VIDEO_START_CAPTURE:
//                mavlink_msg_command_long_send(MAVLINK_COMM_0, mavlink_system.sysid, 
//                                              companion_id, packet.command, 0, 
//                                              packet.param1, packet.param2,
//                                              packet.param3, packet.param4,
//                                              packet.param5, packet.param6,
//                                              packet.param7);
                reflectCmdLong(handle_msg);
            break;
            case MAV_CMD_VIDEO_STOP_CAPTURE:
                reflectCmdLong(handle_msg);
            break;
			default:
				DPRINT("packet.command %u\r\n", packet.command);
				command_ack(packet.command, MAV_CMD_ACK_ERR_NOT_SUPPORTED);
				break;
		}
	}
}

//this function simply reflects mavlink command long back to the companion
//computer. It is called from within the MAVLinkCommandLong handling function
void reflectCmdLong(mavlink_message_t* handle_msg)
{
    mavlink_command_long_t packet;
	mavlink_msg_command_long_decode(handle_msg, &packet);
    
    mavlink_msg_command_long_send(MAVLINK_COMM_0, mavlink_system.sysid, 
                                  companion_id, packet.command, 0, packet.param1, 
                                  packet.param2, packet.param3, packet.param4,
                                  packet.param5, packet.param6, packet.param7);

}

void MAVLinkSetMode(mavlink_message_t* handle_msg) // MAVLINK_MSG_ID_SET_MODE:
{
	mavlink_set_mode_t packet;
    uint8_t prev_base_mode = base_mode;

	mavlink_msg_set_mode_decode(handle_msg, &packet);
//	if (mavlink_check_target(packet.target_system, packet.target_component) == false) break;
	{
        base_mode = packet.base_mode; //set base_mode for vehicle to new requested mode
		switch (packet.base_mode)
		{
            case MAV_MODE_PREFLIGHT:   //System is not ready to fly, booting, calibrating, etc. No flag is set.
                changeState(DISARMED, GCS);
                changeMode(PREFLIGHT, GCS);
                mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Preflight Mode Requested");
            break;
            case MAV_MODE_STABILIZE_DISARMED:   //System is allowed to be active, under assisted RC control.
                changeState(DISARMED, GCS);
                changeMode(STABILIZE, GCS);
                mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Stabilize Disarmed Requested"); 
            break;
            case MAV_MODE_STABILIZE_ARMED:   //System is allowed to be active, under assisted RC control.
                changeState(ARMED, GCS);
                changeMode(STABILIZE, GCS);
                mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Manual Armed Mode Requested");
            break;
            case MAV_MODE_MANUAL_DISARMED:  //System is allowed to be active, under manual (RC) control, no stabilization
                changeState(DISARMED, GCS);
                changeMode(MANUAL, GCS);
                mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Manual Disarmed Mode Requested");
            break;
            case MAV_MODE_MANUAL_ARMED:  //System is allowed to be active, under manual (RC) control, no stabilization
                changeState(ARMED, GCS);
                changeMode(MANUAL, GCS);
                mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Manual Armed Mode Requested");
            break;
            case MAV_MODE_GUIDED_DISARMED:  	//System is allowed to be active, under autonomous control, manual setpoint
                changeState(DISARMED, GCS);
                changeMode(GUIDED, GCS);
                mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Guided Disarmed Mode Requested");
            break;
            case MAV_MODE_GUIDED_ARMED:     //System is allowed to be active, under autonomous control, manual setpoint
                changeState(ARMED, GCS);
                changeMode(GUIDED, GCS);
                mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"Guided Armed Mode Requested");
            break;
//			case 192: // Manual
//				DPRINT("Manual %u\r\n", packet.base_mode);
//				break;
//			case 208: // Manual/Stabilised
//				DPRINT("Manual/Stabilised %u\r\n", packet.base_mode);
//				break;
//			case 216: // Manual/Guided
//				DPRINT("Manual/Guided %u\r\n", packet.base_mode);
//				break;
//			case 156: // Auto
//				DPRINT("Auto %u\r\n", packet.base_mode);
//				break;
//			case 65: // Disarm System
//				DPRINT("Disarm System %u\r\n", packet.base_mode);
//				break;
/*
			case MAV_ACTION_LAUNCH:
				//send_text((uint8_t*) "Action: Launch !\r\n");
				//DPRINT("Action: Launch !\r\n");
				//set_mode(TAKEOFF);
				break;
			case MAV_ACTION_RETURN:
				//send_text((uint8_t*) "Action: Return !\r\n");
				//DPRINT("Action: Return !\r\n");
				//set_mode(RTL);
				break;
			case MAV_ACTION_EMCY_LAND:
				//send_text((uint8_t*) "Action: Emergency Land !\r\n");
				//DPRINT("Action: Emergency Land !\r\n");
				//set_mode(LAND);
				break;
			case MAV_ACTION_HALT:
				//send_text((uint8_t*) "Action: Halt !\r\n");
				//DPRINT("Action: Halt !\r\n");
				//loiter_at_location();
				break;
			case MAV_ACTION_MOTORS_START:
			case MAV_ACTION_CONFIRM_KILL:
			case MAV_ACTION_EMCY_KILL:
			case MAV_ACTION_MOTORS_STOP:
			case MAV_ACTION_SHUTDOWN:
				//set_mode(MANUAL);
				break;
			case MAV_ACTION_CONTINUE:
				//process_next_command();
				break;
			case MAV_ACTION_SET_MANUAL:
				//set_mode(MANUAL);
				break;
			case MAV_ACTION_SET_AUTO:
				//set_mode(AUTO);
				break;
			case MAV_ACTION_STORAGE_READ:
				//send_text((uint8_t*)"Action: Storage Read\r\n");
				//DPRINT("Action: Storage Read\r\n");
				break;
			case MAV_ACTION_STORAGE_WRITE:
				//send_text((uint8_t*)"Action: Storage Write\r\n");
				//DPRINT("Action: Storage Write\r\n");
				break;
			case MAV_ACTION_CALIBRATE_RC:
				//send_text((uint8_t*)"Action: Calibrate RC\r\n");
				//DPRINT("Action: Calibrate RC\r\n");
				break;
			case MAV_ACTION_CALIBRATE_GYRO:
			case MAV_ACTION_CALIBRATE_MAG:
			case MAV_ACTION_CALIBRATE_ACC:
			case MAV_ACTION_CALIBRATE_PRESSURE:
			case MAV_ACTION_REBOOT:
				//startup_IMU_ground();
				break;
			case MAV_ACTION_REC_START: break;
			case MAV_ACTION_REC_PAUSE: break;
			case MAV_ACTION_REC_STOP: break;
			case MAV_ACTION_TAKEOFF:
				//send_text((uint8_t*)"Action: Take Off !\r\n");
				//DPRINT("Action: Take Off !\r\n");
				//set_mode(TAKEOFF);
				break;
			case MAV_ACTION_NAVIGATE:
				// send_text((uint8_t*)"Action: Navigate !\r\n");
				// DPRINT("Action: Navigate !\r\n");
				//set_mode(AUTO);
				break;
			case MAV_ACTION_LAND:
				//set_mode(LAND);
				break;
			case MAV_ACTION_LOITER:
				//set_mode(LOITER);
				break;
 */
			default:
                base_mode = prev_base_mode; //if command could not be completed, don't save new mode to variable
                mavlink_msg_statustext_send(MAVLINK_COMM_0,MAV_SEVERITY_INFO,"Mode not possible with this vehicle");
            break;
		}
	}
}	

//function to accept attitude change from mavlink
//note that this function is not used properly... the attitudes being
//passed should be the desired attitude quaternion
//However, as it is used now, the desired radian value is passed for each axis.
//also throttle should be normalized from 0->1 but we will just use pwm value for now
//currently, this function can be used directly without checking mode
//perhaps this should be changed later for safety.
void SetAttitudeTarget(mavlink_message_t* handle_msg)
{
    mavlink_set_attitude_target_t packet;
	mavlink_msg_set_attitude_target_decode(handle_msg, &packet);
    
    if(packet.type_mask & ~IGNORE_ROLL_RATE)
    {
        setRollRate(packet.body_roll_rate);   
    }
    if(packet.type_mask & ~IGNORE_PITCH_RATE)
    {
        setPitchRate(packet.body_pitch_rate); 
    }
    if(packet.type_mask & ~IGNORE_YAW_RATE)
    {
        setYawRate(packet.body_yaw_rate);
    }
    if(packet.type_mask & ~IGNORE_THROTTLE)
    {
        setThrottle(packet.thrust);
    }
    if(packet.type_mask & ~IGNORE_ATTITUDE)
    {
        setRollAngle(rad_to_degree(packet.q[1]));
        setPitchAngle(rad_to_degree(packet.q[2]));
        setYawAngle(rad_to_degree(packet.q[3]));
    }
//    switch(packet.type_mask)
//    {
//        case(ACCEPT_ROLL_RATE):   //accept roll rate
//            setRollRate(packet.body_roll_rate);
//        //break;
//        case(ACCEPT_PITCH_RATE):   //accept pitch rate
//            setPitchRate(packet.body_pitch_rate); 
//       //break;
//        case(ACCEPT_YAW_RATE):   //accept yaw rate
//            setYawRate(packet.body_yaw_rate);
//        //break;
//        case(ACCEPT_THROTTLE):   //accept throttle
//            setThrottle(packet.thrust);
//        //break;
//        case(ACCEPT_ATTITUDE):   //accept attitude
//            setRollAngle(packet.q[1]);
//            setPitchAngle(packet.q[2]);
//            setYawAngle(packet.q[3]);
//        break;
//        default:    //only change attitudes
//            setRollAngle(packet.q[1]);
//            setPitchAngle(packet.q[2]);
//            setYawAngle(packet.q[3]);
//        break;
//    }
                
    
}

//if high level command is sent to autopilot, "reflect" it back for
//message handling by the companion computer
//these are the messages which the RPi needs to handle
boolean MAVCompanionHandleMessage(mavlink_message_t* handle_msg)
{
	switch (handle_msg->msgid)
	{
#if (FLIGHT_PLAN_TYPE == FP_WAYPOINTS)  //flight plan allows for waypoints
		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            //mavlink_msg_command_long_send(MAVLINK_COMM_0, mavlink_system.sysid, companion_id, 200, 0,0,0,0,0,0,0,0); //DEBUG
			mavlink_msg_mission_request_list_send(MAVLINK_COMM_0, mavlink_system.sysid, companion_id);
			break;
		case MAVLINK_MSG_ID_MISSION_REQUEST:
            //mavlink_msg_mission_request_send(MAVLINK_COMM_0, mavlink_system.sysid, companion_id);
			break;
		case MAVLINK_MSG_ID_MISSION_ACK:
			//mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, companion_id);
			break;
		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
			mavlink_msg_mission_clear_all_send(MAVLINK_COMM_0, mavlink_system.sysid, companion_id);
			break;
		case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
			//mavlink_msg_mission_set_current_send(MAVLINK_COMM_0, mavlink_system.sysid, companion_id);
			break;
		case MAVLINK_MSG_ID_MISSION_COUNT:
            //mavlink_msg_mission_count_send(MAVLINK_COMM_0, mavlink_system.sysid, companion_id);
			break;
		case MAVLINK_MSG_ID_MISSION_ITEM:
            //mavlink_msg_mission_item_send(MAVLINK_COMM_0, mavlink_system.sysid, companion_id);
			break;
#endif // (FLIGHT_PLAN_TYPE == FP_WAYPOINTS)
        
		default:
			return false;
	}
	return true;
}

// Portions of the following code in handleMessage() are templated off source code written by James Goppert for the
// ArdupilotMega, and are used by his kind permission and also in accordance with the GPS V3 licensing
// of that code.

// This is the main routine for taking action against a parsed message from the GCS
static void handleMessage(void)
{
	mavlink_message_t* handle_msg;

	if (mavlink_message_index == 0)
	{
		handle_msg = &msg[1];
	}
	else
	{
		handle_msg = &msg[0];
	}

	//DPRINT("MAV MSG 0x%x\r\n", handle_msg->msgid);

	handling_of_message_completed |= MAVParamsHandleMessage(handle_msg);
    handling_of_message_completed |= MAVCompanionHandleMessage(handle_msg);
	//handling_of_message_completed |= MAVMissionHandleMessage(handle_msg);
	//handling_of_message_completed |= MAVFlexiFunctionsHandleMessage(handle_msg);
    
    handling_of_message_completed = false;
    
	if (handling_of_message_completed == true)
	{
		return;
	}

	switch (handle_msg->msgid)
	{
		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
			MAVLinkRequestDataStream(handle_msg);
			break;
		case MAVLINK_MSG_ID_COMMAND_LONG:
			MAVLinkCommandLong(handle_msg);
			break;
//		case MAVLINK_MSG_ID_COMMAND:
//			DPRINT("MAVLINK_MSG_ID_COMMAND %u\r\n", handle_msg->msgid);
//			break;
//		case MAVLINK_MSG_ID_ACTION:
//			DPRINT("MAVLINK_MSG_ID_ACTION %u\r\n", handle_msg->msgid);
//		case 11:
		case MAVLINK_MSG_ID_SET_MODE:
			MAVLinkSetMode(handle_msg);
        break;
        case MAVLINK_MSG_ID_HEARTBEAT:
            //mavlink_msg_statustext_send(0,MAV_SEVERITY_DEBUG,"GCS Heartbeat Received");
        break;
        case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
            if(getMode()==GUIDED)
                SetAttitudeTarget(handle_msg);
        break;
        default:
//			DPRINT("handle_msg->msgid %u NOT HANDLED\r\n", handle_msg->msgid);
		break;
	}
	handling_of_message_completed = true;
}

////////////////////////////////////////////////////////////////////////////////
//
// Callbacks for triggering command complete messaging
//

inline void preflight_storage_complete_callback(boolean success)
{
	if (mavlink_send_command_ack == false)
	{
		if (success == true)
			mavlink_command_ack_result = MAV_CMD_ACK_OK;
		else
			mavlink_command_ack_result = MAV_CMD_ACK_ERR_FAIL;
		mavlink_command_ack_command = MAV_CMD_PREFLIGHT_STORAGE;
		mavlink_send_command_ack = true;
	}
}



/****************************************************************************
                    MAVLINK SENDING FUNCTIONS
****************************************************************************/
void mavlink_send()
{
//    mavlink_msg_heartbeat_send(0, system_type, autopilot_type, system_mode, custom_mode, system_status);
//    
//    float roll = (3.14*coeff_fixedpoint.actual_angle_aile)/180;
//    float pitch = (-3.14*coeff_fixedpoint.actual_angle_elev)/180;
//    float yaw = (3.14*coeff_fixedpoint.actual_angle_rudd)/180;
//    float rollspeed = (3.14*coeff_fixedpoint.actual_rate_aile)/180;
//    float pitchspeed = (-3.14*coeff_fixedpoint.actual_rate_elev)/180;
//    float yawspeed = (3.14*coeff_fixedpoint.actual_rate_rudd)/180;
//    mavlink_msg_attitude_send(0,0,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed);
    
}

const uint8_t mavlink_freq_table[] = {0, 40, 20, 13, 10, 8, 7, 6, 5, 4, 4};

static boolean is_this_the_moment_to_send(uint8_t counter, uint8_t max_counter)
{
	if (counter % max_counter == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static boolean mavlink_frequency_send(uint8_t frequency, uint8_t counter)
{
	uint8_t max_counter;

	if (frequency == 0)
	{
		return false;
	}
	else if (frequency > 0 && frequency < 11)
	{
		max_counter = mavlink_freq_table[frequency];
		return is_this_the_moment_to_send(counter, max_counter);
	}
	else if (frequency > 10  && frequency < 14)
	{
		max_counter = 4;
		return is_this_the_moment_to_send(counter, max_counter);
	}
	else if (frequency > 13 && frequency < 17)
	{
		max_counter = 3;
		return is_this_the_moment_to_send(counter, max_counter);
	}
	else if (frequency > 16 && frequency < 24)
	{
		max_counter = 2;
		return is_this_the_moment_to_send(counter, max_counter);
	}
	else if (frequency > 23)
	{
		return true; // send data on every call
	}
	else
	{
		return false; // should never reach this line
	}
}



void mavlink_output_40hz(void)
{
//	static float previous_earth_pitch = 0.0;
//	static float previous_earth_roll = 0.0;
//	static float previous_earth_yaw = 0.0;
//
//	//struct relative2D matrix_accum;
//	float earth_pitch;              // pitch in radians with respect to earth
//	float earth_roll;               // roll in radians of the plane with respect to earth frame
//	float earth_yaw;                // yaw in radians with respect to earth frame
//	float earth_pitch_velocity;     // radians / sec with respect to earth
//	float earth_roll_velocity;      // radians / sec with respect to earth
//	float earth_yaw_velocity;       // radians / sec with respect to earth
//	int16_t accum;                  // general purpose temporary storage
//	//union longbbbb accum_A_long;    // general purpose temporary storage
//	//union longbbbb accum_B_long;    // general purpose temporary storage
//	int32_t lat, lon, alt, relative_alt = 0;
//	uint16_t mavlink_heading = 0;

	enum MAV_CUSTOM_UDB_MODE_FLAG
	{
		MAV_CUSTOM_UDB_MODE_MANUAL = 1,     // Manual Mode. MatrixPilot passes all PWM signals from Receiver straight out to Servos
		MAV_CUSTOM_UDB_MODE_STABILIZE = 2,  // Stabilzed Mode. MatrixPilot assists in flying plane. Pilot still commands plane from transmitter.
		MAV_CUSTOM_UDB_MODE_AUTONOMOUS = 3, // Autonmous Mode. Plane is primarily flying using waypoints or Logo flight language (although pilot can mix in control from transmitter).
		MAV_CUSTOM_UDB_MODE_RTL = 4,        // Return to Launch or Failsafe Mode. This mode means plane has lost contact with pilot's control transmitter.
	};

	uint8_t spread_transmission_load = 0;   // Used to spread sending of different message types over a period of 1 second.

	if (++mavlink_counter_40hz >= 40) mavlink_counter_40hz = 0;

	usec += 25000;  // Frequency sensitive code
	msec += 25;     // Frequency sensitive code

	// Note that message types are arranged in order of importance so that if the serial buffer fills up,
	// critical message types are more likely to still be transmitted.

	// HEARTBEAT
	spread_transmission_load = 1;
	if (mavlink_frequency_send(MAVLINK_RATE_HEARTBEAT, mavlink_counter_40hz + spread_transmission_load))
	{
        base_mode = getBaseMode();
//        if(vehicle_state == ARMED)
//            base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
//        else if(vehicle_state == DISARMED)
//            base_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
//		if (flags._.GPS_steering == 0 && flags._.pitch_feedback == 0)
//		{
//			mavlink_base_mode = MAV_MODE_MANUAL_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
//			mavlink_custom_mode = MAV_CUSTOM_UDB_MODE_MANUAL;
//		}
//		else if (flags._.GPS_steering == 0 && flags._.pitch_feedback == 1)
//		{
//			mavlink_base_mode = MAV_MODE_GUIDED_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
//			mavlink_custom_mode = MAV_CUSTOM_UDB_MODE_STABILIZE;
//		}
//		else if (flags._.GPS_steering == 1 && flags._.pitch_feedback == 1 && udb_flags._.radio_on == 1)
//		{
//			mavlink_base_mode = MAV_MODE_AUTO_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
//			mavlink_custom_mode = MAV_CUSTOM_UDB_MODE_AUTONOMOUS;
//		}
//		else if (flags._.GPS_steering == 1 && flags._.pitch_feedback == 1 && udb_flags._.radio_on == 0)
//		{
//			mavlink_base_mode = MAV_MODE_AUTO_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; // Return to Landing (lost contact with transmitter)
//			mavlink_custom_mode = MAV_CUSTOM_UDB_MODE_RTL;
//		}
//		else
//		{
//			mavlink_base_mode = MAV_MODE_TEST_ARMED; // Unknown state
//			mavlink_custom_mode = MAV_CUSTOM_UDB_MODE_MANUAL;
//		}
		mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, autopilot_type, base_mode, custom_mode, system_status);
	}
    
    // GPS RAW INT - Data from GPS Sensor sent as raw integers.
	spread_transmission_load = 4;
//	if (mavlink_frequency_send(streamRates[MAV_DATA_STREAM_RAW_SENSORS], mavlink_counter_40hz + spread_transmission_load))
//	{
//		int16_t gps_fix_type;
//		if (gps_nav_valid())
//			gps_fix_type = 3;
//		else
//			gps_fix_type = 0;
//		mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, usec, gps_fix_type, lat_gps.WW, lon_gps.WW, alt_sl_gps.WW, hdop, 65535, sog_gps.BB, cog_gps.BB, svs);
//	}

	// GLOBAL POSITION INT - derived from fused sensors
	// Note: This code assumes that Dead Reckoning is running.
	spread_transmission_load = 6;
	if (mavlink_frequency_send(streamRates[MAV_DATA_STREAM_POSITION], mavlink_counter_40hz + spread_transmission_load))
	{
		//accum_A_long.WW = IMUlocationy._.W1 + (int32_t)(lat_origin.WW / 90.0);  // meters North from Equator
		//lat = (int32_t) accum_A_long.WW * 90;                                   // degrees North from Equator
        
//		if (cos_lat == 0)
//		{
//			// We are at the north or south poles, where there is no longitude
//			lon = 0;
//		}
//		else
//		{
//			accum_A_long.WW = IMUlocationx._.W1;
//			accum_A_long.WW = accum_A_long.WW * 16384;      // Compiler uses (shift left 14) for this multiplication
//			accum_B_long.WW = (accum_A_long.WW + 8192) / cos_lat;   // 8192 improves rounding accuracy
//			lon = lon_origin.WW + (accum_B_long.WW * 90);   // degrees
//		}
        
		//accum_A_long.WW = IMUlocationz._.W1;
		//relative_alt = accum_A_long.WW * 1000;
		//alt = relative_alt + (alt_origin.WW * 10);          // In millimeters; more accurate if used IMUlocationz._.W0

		//mavlink_heading = get_geo_heading_angle() * 100;    // mavlink global position expects heading value x 100
		//mavlink_msg_global_position_int_send(MAVLINK_COMM_0, msec, lat, lon, alt, relative_alt,
		//    IMUvelocityy._.W1, IMUvelocityx._.W1, -IMUvelocityz._.W1, //  IMUVelocity upper word gives V in cm / second
		 //   mavlink_heading); // heading should be from 0 to 35999 meaning 0 to 359.99 degrees.
        
	}

	// ATTITUDE
	//  Roll: Earth Frame of Reference
	spread_transmission_load = 12;
	if (mavlink_frequency_send(streamRates[MAV_DATA_STREAM_POSITION], mavlink_counter_40hz + spread_transmission_load))
	{
        float roll = deg_to_radian(coeff_fixedpoint.actual_angle_aile);
        float pitch = -deg_to_radian(coeff_fixedpoint.actual_angle_elev);
        float yaw = deg_to_radian(coeff_fixedpoint.actual_angle_rudd);
        float rollspeed = deg_to_radian(coeff_fixedpoint.actual_rate_aile);
        float pitchspeed = -deg_to_radian(coeff_fixedpoint.actual_rate_elev);
        float yawspeed = deg_to_radian(coeff_fixedpoint.actual_rate_rudd);

		mavlink_msg_attitude_send(MAVLINK_COMM_0,msec, roll, pitch, yaw,
		    rollspeed, pitchspeed, yawspeed);
	}

	// SYSTEM STATUS
	spread_transmission_load = 18;
	if (mavlink_frequency_send(MAVLINK_RATE_SYSTEM_STATUS, mavlink_counter_40hz + spread_transmission_load))
	{
        calc_battery_life();
        //battery_voltage = ((udb_vcc.value + (int32_t)32768) * (MAX_VOLTAGE))* VOLTAGE_RATIO;
		mavlink_msg_sys_status_send(MAVLINK_COMM_0,
		    onboard_control_sensors,              // Sensors fitted
		    onboard_control_sensors,              // Sensors enabled ... assume all enabled for now
		    onboard_control_sensors,              // Sensor health ... assume all operational for now
		    udb_cpu_load() * 10,
		    battery.millivolts,              // Battery voltage in mV
		    -1,                           //Current
		    battery.remaining,           // Percentage battery remaining 100 percent is 1000
		    r_mavlink_status.packet_rx_drop_count,
		    0,              // errors_comm
		    0,              // errors_count1
		    0,              // errors_count2
		    0,              // errors_count3
		    0);             // errors_count4
	}

	// RC CHANNELS
	spread_transmission_load = 24;
	if (mavlink_frequency_send(streamRates[MAV_DATA_STREAM_RAW_SENSORS], mavlink_counter_40hz + spread_transmission_load))
	{
        //RC channels
        //NOTE this message is not used exactly as intended according to mavlink standard
		mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0, msec,
            (uint8_t)0,     // port number for more than 8 servos
            (uint16_t)pulsewidth.thro,
            (uint16_t)pulsewidth.elev,
            (uint16_t)pulsewidth.rudd,
            (uint16_t)pulsewidth.aile,
            (uint16_t)pulsewidth.gear,
            (uint16_t)pulsewidth.aux1,
             UINT16_MAX,
             UINT16_MAX,
             UINT8_MAX); // 255 denotes not in use
                
		mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_0, msec,
            (uint8_t)0,     // port number for more than 8 servos
            map(pulsewidth.thro, 5500, 9500, -100, 100),
		    map(pulsewidth.elev, 5500, 9500, -100, 100),
		    map(pulsewidth.rudd, 5500, 9500, -100, 100),
		    map(pulsewidth.aile, 5500, 9500, -100, 100),
		    pulsewidth.gear/45, //fix -- 45 is guess
		    pulsewidth.aux1/45, //fix -- 45 is guess
		    UINT16_MAX,
		    UINT16_MAX,  
		    UINT8_MAX);  // 255 denotes not in use
        
        //NOTE this message is not used exactly as intended according to mavlink standard
        mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, usec,
            (uint8_t)0,     // port number for more than 8 servos
            (uint16_t)MIX.m1,
            (uint16_t)MIX.m2,
            (uint16_t)MIX.m3,
            (uint16_t)MIX.m4,
             UINT16_MAX,
             UINT16_MAX,
             UINT16_MAX,
             UINT16_MAX); // max_value denotes not in use
        
        //Ultrasonics
        mavlink_msg_distance_sensor_send(MAVLINK_COMM_0, msec,
                (uint16_t)SONAR_MIN,    //defined in SRF02.h
                (uint16_t)SONAR_MAX,
                10086,
//                height_after_kalman,
//                Sonar_Dist[0],    //value is periodically updated from main.c
                MAV_DISTANCE_SENSOR_ULTRASOUND,
                (uint8_t)Sonar_SensAddr1,   //use i2c address as mavlink id
                0, //sensor orientaion... shouldn't matter right now,
                0   //measurement covariance unknown
                );
        

	}

	// RAW SENSORS - ACCELOREMETERS and GYROS
	// It is expected that these values are graphed to allow users to check basic sensor operation,
	// and to graph noise on the signals. As this code if for testing and graphing basic hardware, it uses
	// UDB conventions coordinate conventions for X,Y and Z axis rather than MAVLink conventions.
	// See:- http://code.google.com/p/gentlenav/wiki/UDBCoordinateSystems and the "Aviation Convention" diagram.

	spread_transmission_load = 30;
	if (mavlink_frequency_send(streamRates[MAV_DATA_STREAM_RAW_SENSORS], mavlink_counter_40hz + spread_transmission_load))
    {
        
        //NEED TO reconfigure IMU to send all of these values
//        mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, msec,
//                )
//                
//        mavlink_msg_raw_imu_send(MAVLINK_COMM_0, msec,
//        )
	}

	spread_transmission_load = 36;
	if (mavlink_frequency_send(streamRates[MAV_DATA_STREAM_EXTRA2], mavlink_counter_40hz + spread_transmission_load))
	{
		//mavlink_msg_altitudes_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t alt_gps, int32_t alt_imu, int32_t alt_barometric, int32_t alt_optical_flow, int32_t alt_range_finder, int32_t alt_extra)
	}

	spread_transmission_load = 40;
//	if (mavlink_frequency_send(streamRates[MAV_DATA_STREAM_EXTRA2], mavlink_counter_40hz + spread_transmission_load))
//	{
//		//mavlink_msg_airspeeds_send(mavlink_channel_t chan, uint32_t time_boot_ms, int16_t airspeed_imu, int16_t airspeed_pitot, int16_t airspeed_hot_wire, int16_t airspeed_ultrasonic, int16_t aoa, int16_t aoy)
//	}

//	spread_transmission_load = 10;
//	if (mavlink_frequency_send(streamRates[MAV_DATA_STREAM_EXTRA1], mavlink_counter_40hz + spread_transmission_load)) // SUE code historically ran at 8HZ
//	{
//		MAVUDBExtraOutput_40hz();
//	}
    
//  // Send FORCE information
//  spread_transmission_load = 15;
//	if (mavlink_frequency_send(MAVLINK_RATE_FORCE, mavlink_counter_40hz + spread_transmission_load))
//	{
//      mavlink_msg_force_send(MAVLINK_COMM_0, msec, aero_force[0], aero_force[1], aero_force[2]);
//	}
	MAVParamsOutput_40hz();
	//MAVMissionOutput_40hz();
	//MAVFlexiFunctionsOutput_40hz();

	// Acknowledge a command if flagged to do so.
	if (mavlink_send_command_ack == true)
	{
		mavlink_msg_command_ack_send(MAVLINK_COMM_0, mavlink_command_ack_command, mavlink_command_ack_result);
		mavlink_send_command_ack = false;
	}
    
//MW THIS SHOULD BE MOVED SOMEWHERE ELSE
#if (USE_NV_MEMORY == 1)
		nv_memory_service_trigger();
		storage_service_trigger();
		data_services_trigger();
#endif
}

//use for sending messages to GCS
void mav_printf(const char* format, ...)
{
	char buf[200];
	va_list arglist;

	va_start(arglist, format);
	vsnprintf(buf, sizeof(buf), format, arglist);
	// mavlink_msg_statustext_send(MAVLINK_COMM_1, severity, text);
	// severity: Severity of status, 0 = info message, 255 = critical fault (uint8_t)
	mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_DEBUG, buf);
	va_end(arglist);
}