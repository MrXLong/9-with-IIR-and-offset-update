// pyparam generated file - DO NOT EDIT

// this module is a horrible hack to work around VC++ not allowing
// static initialisation of named union member variables
#ifdef _MSC_VER

#include "defines.h" 
#include "mavlink_options.h"

#if (USE_MAVLINK == 1)

#include "parameter_table.h"
#include "data_storage.h"

void parameter_table_init(void)
{
	mavlink_parameters_list[0].min.param_float=0.0; mavlink_parameters_list[0].max.param_float=5.0; // rollkp - PID_ROLLKP
	mavlink_parameters_list[1].min.param_float=0.0; mavlink_parameters_list[1].max.param_float=5.0; // rollki - PID_ROLLKI
	mavlink_parameters_list[2].min.param_float=0.0; mavlink_parameters_list[2].max.param_float=5.0; // rollkd - PID_ROLLKD
	mavlink_parameters_list[3].min.param_float=0.0; mavlink_parameters_list[3].max.param_float=5.0; // pitchkp - PID_PITCHKP
	mavlink_parameters_list[4].min.param_float=0.0; mavlink_parameters_list[4].max.param_float=5.0; // pitchki - PID_PITCHKI
	mavlink_parameters_list[5].min.param_float=0.0; mavlink_parameters_list[5].max.param_float=5.0; // pitchkd - PID_PITCHKD
	mavlink_parameters_list[6].min.param_float=0.0; mavlink_parameters_list[6].max.param_float=5.0; // yawkp - PID_YAWKP
	mavlink_parameters_list[7].min.param_float=0.0; mavlink_parameters_list[7].max.param_float=5.0; // yawki - PID_YAWKI
	mavlink_parameters_list[8].min.param_float=0.0; mavlink_parameters_list[8].max.param_float=5.0; // yawkd - PID_YAWKD

	mavlink_parameters_list[9].min.param_float=0.0; mavlink_parameters_list[9].max.param_float=50.0; // hoverkp - PID_HOVERKP
	mavlink_parameters_list[10].min.param_float=0.0; mavlink_parameters_list[10].max.param_float=50.0; // hoverki - PID_HOVERKI
	mavlink_parameters_list[11].min.param_float=0.0; mavlink_parameters_list[11].max.param_float=20.0; // hoverkd - PID_HOVERKD
	mavlink_parameters_list[12].min.param_int32=0; mavlink_parameters_list[12].max.param_int32=9500; // nom_hov_thr - NOM_HOVER_THR
	mavlink_parameters_list[13].min.param_int32=0; mavlink_parameters_list[13].max.param_int32=9500; // min_throttle - MIN_THR

};

#endif // (USE_MAVLINK == 1)
#endif // _MSC_VER
