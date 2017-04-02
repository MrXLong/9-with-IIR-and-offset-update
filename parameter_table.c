// pyparam generated file - DO NOT EDIT

#include "defines.h"
#include "mavlink_options.h"

#if (SILSIM == 0 && USE_MAVLINK == 1)

#include "parameter_table.h"
#include "data_storage.h"


const mavlink_parameter_parser mavlink_parameter_parsers[] = {
	{ &mavlink_send_param_int16, &mavlink_set_param_int16, MAVLINK_TYPE_INT32_T},
	{ &mavlink_send_param_Q14, &mavlink_set_param_Q14, MAVLINK_TYPE_FLOAT},
	{ &mavlink_send_param_gyroscale_Q14, &mavlink_set_param_gyroscale_Q14, MAVLINK_TYPE_FLOAT},
	{ &mavlink_send_int_circular, &mavlink_set_int_circular, MAVLINK_TYPE_INT32_T},
	{ &mavlink_send_dm_airspeed_in_cm, &mavlink_set_dm_airspeed_from_cm, MAVLINK_TYPE_INT32_T},
	{ &mavlink_send_dm_airspeed_in_m, &mavlink_set_dm_airspeed_from_m, MAVLINK_TYPE_FLOAT},
	{ &mavlink_send_cm_airspeed_in_m, &mavlink_set_cm_airspeed_from_m, MAVLINK_TYPE_FLOAT},
	{ &mavlink_send_frame_anglerate, &mavlink_set_frame_anglerate, MAVLINK_TYPE_INT32_T},
	{ &mavlink_send_dcm_angle, &mavlink_set_dcm_angle, MAVLINK_TYPE_INT32_T},
	{ &mavlink_send_param_float, &mavlink_set_param_float, MAVLINK_TYPE_FLOAT},
	};

#ifdef _MSC_VER
mavlink_parameter mavlink_parameters_list[] = {
#else
const mavlink_parameter mavlink_parameters_list[] = {
#endif // _MSC_VER
	{"PID_ROLLKP", {.param_float=0.0}, {.param_float=5.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&rollkp, sizeof(rollkp) },
	{"PID_ROLLKI", {.param_float=0.0}, {.param_float=5.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&rollki, sizeof(rollki) },
	{"PID_ROLLKD", {.param_float=0.0}, {.param_float=5.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&rollkd, sizeof(rollkd) },
	{"PID_PITCHKP", {.param_float=0.0}, {.param_float=5.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&pitchkp, sizeof(pitchkp) },
	{"PID_PITCHKI", {.param_float=0.0}, {.param_float=5.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&pitchki, sizeof(pitchki) },
	{"PID_PITCHKD", {.param_float=0.0}, {.param_float=5.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&pitchkd, sizeof(pitchkd) },
	{"PID_YAWKP", {.param_float=0.0}, {.param_float=5.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&yawkp, sizeof(yawkp) },
	{"PID_YAWKI", {.param_float=0.0}, {.param_float=5.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&yawki, sizeof(yawki) },
	{"PID_YAWKD", {.param_float=0.0}, {.param_float=5.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&yawkd, sizeof(yawkd) },

	{"PID_HOVERKP", {.param_float=0.0}, {.param_float=50.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&hoverkp, sizeof(hoverkp) },
	{"PID_HOVERKI", {.param_float=0.0}, {.param_float=50.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&hoverki, sizeof(hoverki) },
	{"PID_HOVERKD", {.param_float=0.0}, {.param_float=20.0}, UDB_TYPE_FLOAT, PARAMETER_READWRITE, (void*)&hoverkd, sizeof(hoverkd) },
	{"NOM_HOVER_THR", {.param_int32=0}, {.param_int32=9500}, UDB_TYPE_INT, PARAMETER_READWRITE, (void*)&nom_hov_thr, sizeof(nom_hov_thr) },
	{"MIN_THR", {.param_int32=0}, {.param_int32=9500}, UDB_TYPE_INT, PARAMETER_READWRITE, (void*)&min_throttle, sizeof(min_throttle) },

};

const uint16_t count_of_parameters_list = sizeof(mavlink_parameters_list) / sizeof(mavlink_parameter);


#endif  // (SILSIM == 0 && USE_MAVLINK == 1)
