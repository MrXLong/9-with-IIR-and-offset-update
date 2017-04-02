// pyparam generated file - DO NOT EDIT


#include "parameter_table.h"
#include "data_services.h"

#if(USE_NV_MEMORY == 1)

const mavlink_parameter_block mavlink_parameter_blocks[] = {
	{ STORAGE_HANDLE_CONTROL_GAINS, 0, 9, STORAGE_FLAG_LOAD_AT_STARTUP | STORAGE_FLAG_LOAD_AT_REBOOT | STORAGE_FLAG_STORE_CALIB, NULL },
	{ STORAGE_HANDLE_HOVER_SETTINGS, 9, 5, STORAGE_FLAG_LOAD_AT_STARTUP | STORAGE_FLAG_LOAD_AT_REBOOT | STORAGE_FLAG_STORE_CALIB, NULL },
};


const uint16_t mavlink_parameter_block_count = sizeof(mavlink_parameter_blocks) / sizeof(mavlink_parameter_block);

#endif // USE_NV_MEMORY

