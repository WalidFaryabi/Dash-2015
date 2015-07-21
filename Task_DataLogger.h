#ifndef DATALOGGERTASK_H_
#define DATALOGGERTASK_H_

#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/queue.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/semphr.h"

#define BUFFER_LENGTH 16384
//#define BUFFER_LENGTH 32768
//#define BUFFER_LENGTH 4096
#define BUFFER_OFFSET 35
#define MAX_MESSAGE_SIZE 35
#define FIXED_MESSAGE_LENGTH 19
#define BUFFER_ADJUST ((BUFFER_LENGTH/BUFFER_OFFSET)*BUFFER_OFFSET)
#define PREALLOCATION_BYTES 20000000
#define NEW_PREALLOCATION_THRESHOLD ( (PREALLOCATION_BYTES-100000)/BUFFER_LENGTH)

#define LEN_PRESET_FILENAME 20

//************************************************//
//--------------PRESET RELATED--------------------//
//************************************************//
extern QueueHandle_t xPresetQueue;
extern char preset_file_name[LEN_PRESET_FILENAME];
struct presetParameterStruct {
	float p_term;
	float i_term;
	float d_term;
	float max_min_term;
	float max_decrease_term;
	float desired_slip_term;
	float max_integral_term;
	uint8_t selected_preset;
};


extern QueueHandle_t xDataloggerCommandQueue;
extern QueueHandle_t xDataloggerStatusQueue;

extern TaskHandle_t dataLoggerHandle;

extern uint32_t file_size_byte_counter;
extern uint32_t start_time;
extern uint32_t	stop_time; 
extern uint8_t number_of_files_sdcard;
extern uint8_t SD_card_is_full; 

enum EDataloggerStates		{DATALOGGER_IDLE,DATALOGGER_FILE_OPEN, DATALOGGER_LOGGING,DATALOGGER_USB_CONNECTED};
extern enum EDataloggerStates dataloggerState;
	
enum EDataloggerCommands	{CREATE_NEW_FILE,START_LOGGING, CLOSE_FILE, DELETE_ALL_FILES,GET_PARAMETERS_FROM_FILE, NO_COMMAND};
enum EDataloggerStatus		{STATUS_FILE_OPEN,STATUS_IS_LOGGING,STATUS_NO_FILE_OPEN,STATUS_USB_CONNECTED,STATUS_USB_NOT_CONNECTED};
void dataLoggerTask();




#endif /* DATALOGGERTASK_H_ */