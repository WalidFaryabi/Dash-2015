#include "Task_DataLogger.h"
#include "sam4e-base/RevolveDrivers/pio.h"

#include "DriversNotInBase/SD_FAT/sd_mmc.h"
#include "DriversNotInBase/SD_FAT/fat/ff.h"
#include "DriversNotInBase/config_sd_msc/ctrl_access.h"

#include "Task_USBMSC.h"
#include "CanHandler.h"
#include "sam4e-base/RevolveDrivers/can.h"
#include "DriversNotInBase/IO_DashInit.h"

#include <stdio.h>
#include <string.h>

static void getPresetParametersFromFile(char file_name[]);

static struct presetParameterStruct presetParametersToMenuTask;
Ctrl_status status;
FRESULT res, filePresent;
FATFS fs;
FIL file_object;
FILINFO fno;

/*
-Both the USB MSC task and datalogger task accesses the SD card but this is implemented in a mutual exclusion way so no synchronisation is needed
-Disk initialiation and FS init should be done before the task scheduler runs according to a post at "http://www.freertos.org/FreeRTOS_Support_Forum_Archive/August_2009/freertos_FreeRTOS_and_Filesystem_integration_3367836.html"
*/

static void createFileName(char file_name[]);
static void createOpenSeekNewFile();
static void logDataToCurrentFile();
static void deleteAllFiles();

struct SensorMsg {
	struct CanMessage can_msg;
	uint32_t time_stamp;
};
struct SensorMsg SensorPacketReceive = {
	.can_msg.data.u64 = 0,
	.can_msg.dataLength = 0,
	.can_msg.messageID = 0,
	.time_stamp = 0
};

struct CanMessage txmsg = {
	.data.u8[0] = 10,
	.data.u8[1] = 5,
	.dataLength = 2,
	.messageID = 10
};
struct CanMessage benchmsg = {
	.data.u64 = 0,
	.dataLength = 8,
	.messageID = 510
};


static char dataLogger_buffer[BUFFER_LENGTH] = "";
static char can_data_string[17] = "";

enum EDataloggerStates dataloggerState = DATALOGGER_IDLE;
QueueHandle_t xDataloggerCommandQueue = NULL;
QueueHandle_t xPresetQueue = NULL;

UINT byte_written;
static uint32_t timeStamp = 0;
static uint32_t offset = 0;
static uint8_t current_message_length = 0;
 

uint32_t file_size_byte_counter = 0;
uint8_t number_of_files_sdcard = 0;
static uint32_t preallocation_counter = 0;

char preset_file_name[LEN_PRESET_FILENAME] = "";
static char fileName[10] = "";
TaskHandle_t dataLoggerHandle = NULL;

uint32_t start_time = 0;
uint32_t stop_time = 0;
void dataLoggerTask() {
	memset(&fs, 0, sizeof(FATFS));
	res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
	if (FR_INVALID_DRIVE == res) {
		//failed
	}
	enum EDataloggerCommands currentCommand = NO_COMMAND;	
	
	// Create and start logging to a new file automatically
	xSemaphoreTake(file_access_mutex,portMAX_DELAY);
	createOpenSeekNewFile();
	dataloggerState = DATALOGGER_LOGGING;
	
	while(1) {
		
		if (xQueueReceive(xDataloggerCommandQueue,&currentCommand,0) != pdPASS) {
			currentCommand = NO_COMMAND;
		}
		else {
			uint8_t heo = 5;
		}
		
		switch (dataloggerState) {
			case DATALOGGER_IDLE:
				switch (currentCommand) {
					case START_LOGGING:
						createOpenSeekNewFile();
						dataloggerState = DATALOGGER_LOGGING;
						break;
					case DELETE_ALL_FILES:
						deleteAllFiles();
						break;
					case GET_PARAMETERS_FROM_FILE:
						getPresetParametersFromFile(preset_file_name);
						break;
				}
				if ( (pio_readPin(DETECT_USB_PIO,DETECT_USB_PIN) == 1 )  && (dataLoggerHandle == xSemaphoreGetMutexHolder(file_access_mutex)) ) {
					dataloggerState = DATALOGGER_USB_CONNECTED;
					xSemaphoreGive(file_access_mutex);
					// Disable CAN interrupts while USB is connected
					can_disableRXInterrupt(CAN0_IRQn);
					can_disableRXInterrupt(CAN1_IRQn);
				}
				// Send command to datalogger to extract data from SD card and send it to task_menu via a queue ?
				
				vTaskDelay(40/portTICK_RATE_MS);
			break;		
			case DATALOGGER_LOGGING:
				if ( (pio_readPin(DETECT_USB_PIO,DETECT_USB_PIN) == 1 )  && (dataLoggerHandle == xSemaphoreGetMutexHolder(file_access_mutex)) ) {
					dataloggerState = DATALOGGER_USB_CONNECTED;
					f_truncate(&file_object);
					f_close(&file_object);
					xSemaphoreGive(file_access_mutex);
					// Disable CAN interrupts if USB is connected
					can_disableRXInterrupt(CAN0_IRQn);
					can_disableRXInterrupt(CAN1_IRQn);
					break;
				}
				switch(currentCommand) {
					case CLOSE_FILE:
						offset = 0;
						f_truncate(&file_object);
						f_close(&file_object);					
						dataloggerState = DATALOGGER_IDLE;
						break;
					default:
						logDataToCurrentFile();
						break;
				}
				// The delay is in the queuereceive function now.
				//vTaskDelay(1); // This needs confirmation based on speed requirements etc.
			break;
			
			case DATALOGGER_USB_CONNECTED:
				if ( (pio_readPin(DETECT_USB_PIO,DETECT_USB_PIN) == 0) && (dataLoggerHandle != xSemaphoreGetMutexHolder(file_access_mutex)) )   {
					// If USB is not connected and the datalogger doesnt own the file mutex -> aquire it
					xSemaphoreTake(file_access_mutex,portMAX_DELAY);
					dataloggerState = DATALOGGER_IDLE;
					// Leaving USB CONNECTED STATE : ENABLE CAN INTERRUPTS
					can_enableRXInterrupt(CAN0_IRQn);
					can_enableRXInterrupt(CAN1_IRQn);
					
				}
				vTaskDelay(5/portTICK_RATE_MS);
			break;
		}
	}
}

static void deleteAllFiles() {
	fno.lfname = 0;
	char test_name[10] = "";
	number_of_files_sdcard = 0;
	for (uint8_t i = 0; i < 100; i++) {
		snprintf(test_name,10, "log%02d.txt",i);
		filePresent = f_stat(test_name, &fno);
		if (filePresent == FR_OK) {
			f_unlink(test_name); // Delete the file if it exists
		}
		else {
			break;
		}
	}
}

static void createOpenSeekNewFile() {
	createFileName(fileName);
	res = f_open(&file_object, (char const *)fileName, FA_OPEN_ALWAYS | FA_WRITE);
	f_lseek(&file_object,PREALLOCATION_BYTES);
	f_lseek(&file_object,0);
	//f_truncate(&file_object);
	file_size_byte_counter = 0;
	offset = 0;
}

static void logDataToCurrentFile() {
	//if (xQueueReceive(xDataLoggerQueue,&SensorPacketReceive,0) == pdPASS) { // Blocks until it has received an element
	if (xQueueReceive(xDataLoggerQueue,&SensorPacketReceive,1/portTICK_RATE_MS) == pdPASS) { // Blocks until it has received an element
		//can_sendMessage(CAN0, txmsg);
		if (offset >= (BUFFER_LENGTH-MAX_MESSAGE_SIZE)) {
			offset = 0;			
			
			file_size_byte_counter	+= 1;
			preallocation_counter	+= 1;
			taskENTER_CRITICAL();
			start_time = RTT->RTT_VR;
			//In case of *bw is less than btw, it means the volume got full during the write operation
			f_write(&file_object,dataLogger_buffer, BUFFER_LENGTH, &byte_written);
			if ( byte_written < BUFFER_LENGTH) {
				// DISK IS FULL !!
				dataloggerState = DATALOGGER_IDLE;
				*dataLogger_buffer = 0;
				stop_time = RTT->RTT_VR;
				taskEXIT_CRITICAL();
			}
			else {
				stop_time = RTT->RTT_VR;
				f_sync(&file_object);
				*dataLogger_buffer = 0;
				taskEXIT_CRITICAL();
			}		
			//benchmsg.data.u32[0] = (BUFFER_LENGTH)/(stop_time-start_time);
			//can_sendMessage(CAN0,benchmsg);
		}
		else if (preallocation_counter > NEW_PREALLOCATION_THRESHOLD) {
			f_lseek(&file_object, f_tell(&file_object)	+ PREALLOCATION_BYTES	);
			f_lseek(&file_object, f_tell(&file_object)  - PREALLOCATION_BYTES	);
			preallocation_counter = 0;
		}
		if (SensorPacketReceive.can_msg.dataLength == 0) {
			// Handle special case of empty message. Add a zero to make revolve analyze happy
			current_message_length = FIXED_MESSAGE_LENGTH + 2;
		}
		else {
			current_message_length = FIXED_MESSAGE_LENGTH + (SensorPacketReceive.can_msg.dataLength*2);
		}
		
// 		for (int i = 0; i < SensorPacketReceive.can_msg.dataLength; i++){
// 			sprintf(can_data_string, "%s%02X", can_data_string, SensorPacketReceive.can_msg.data.u8[i]);
// 		}
// 		
		if ( SensorPacketReceive.can_msg.dataLength == 0) {
			sprintf(can_data_string, "%02x",0x00);
		}
		else {	
			for (uint8_t i = 0; i < SensorPacketReceive.can_msg.dataLength;i++) {
				sprintf(can_data_string, "%s%02x",can_data_string, SensorPacketReceive.can_msg.data.u8[i]);
			}
		}
		snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%s}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID,can_data_string);
		
		*can_data_string = 0;
		
// 		switch (SensorPacketReceive.can_msg.dataLength) {
// 			case 0:
// 			snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%x}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID,0);
// 			break;
// 			case 1:
// 			snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%x}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID, SensorPacketReceive.can_msg.data.u8[0]);
// 			break;
// 			case 2:
// 			snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%x}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID, SensorPacketReceive.can_msg.data.u16[0]);
// 			break;
// 			case 3:
// 			snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%lx}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID, SensorPacketReceive.can_msg.data.u32[0]	& 0x00FFFFFF);
// 			break;
// 			case 4:
// 			snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%lx}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID, SensorPacketReceive.can_msg.data.u32[0]);
// 			break;
// 			case 5:
// 			snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%llx}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID, SensorPacketReceive.can_msg.data.u64 & 0x000000FFFFFFFFFF);
// 			break;
// 			case 6:
// 			snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%llx}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID, SensorPacketReceive.can_msg.data.u64 & 0x0000FFFFFFFFFFFF);
// 			break;
// 			case 7:
// 			snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%llx}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID, SensorPacketReceive.can_msg.data.u64 & 0x00FFFFFFFFFFFFFF);
// 			break;
// 			case 8:
// 			snprintf(dataLogger_buffer + offset, current_message_length,"[%08lx,{%03lx:%llx}]\n",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID, SensorPacketReceive.can_msg.data.u64);
// 			break;
// 		}
		//snprintf(dataLogger_buffer + offset, BUFFER_OFFSET,"[%08lx,{%03lx:%016llx}]",SensorPacketReceive.time_stamp, SensorPacketReceive.can_msg.messageID, SensorPacketReceive.can_msg.data.u64);
		offset += current_message_length;
		
	}
}


static void createFileName(char file_name[]) {
	//FRESULT fr;
	//FILINFO fno;
	fno.lfname = 0;
	char test_name[10] = "";
	for (uint8_t i = 0; i < 100; i++) {
		snprintf(test_name,10, "log%02d.txt",i);
		filePresent = f_stat(test_name, &fno);
		//filePresent = f_stat("log00.txt", &fno);
		if (filePresent != FR_OK) {
			//strncpy(file_name,test_name,5);
			snprintf(file_name,10,"log%02d.txt",i);
			number_of_files_sdcard = i+1;
			break;
		}
	}
}

static void getPresetParametersFromFile(char file_name[]) {
	uint32_t integer_part;
	uint32_t decimal_part;
	char integer_part_s[10];
	char decimal_part_s[10];
	char number[20];
	char parameter_name[20];
	char * ptr_to_token;
	char * ptr_to_token_parameter_name;
	
	char line[50];
	res = f_open(&file_object, (char const *)file_name, FA_OPEN_EXISTING | FA_READ);
	if (res == FR_OK) {
		while ( f_gets(line, sizeof line, &file_object) ) {
			
			ptr_to_token_parameter_name = strtok(line,":");
			strcpy(parameter_name,ptr_to_token_parameter_name);
			
			strcpy(number, line + strlen(parameter_name));
			// Get the integer part
			ptr_to_token = strtok(number,".");
			strcpy(integer_part_s,ptr_to_token);
			// Get the decimal part
			ptr_to_token = strtok(NULL,".");
			strcpy(decimal_part_s,ptr_to_token);
			integer_part = atoi(integer_part_s);
			decimal_part = atoi(decimal_part_s);
			float f = integer_part + (float) decimal_part/100;
			switch (line[0]) {
				case '1':
					presetParametersToMenuTask.p_term = f;
				break;
				case '2':
					presetParametersToMenuTask.i_term = f;
				break;
				case '3':
					presetParametersToMenuTask.d_term = f;
				break;
				case '4':
					presetParametersToMenuTask.max_min_term = f;
				break;
				case '5':
					presetParametersToMenuTask.max_decrease_term = f;
				break;
				case '6':
					presetParametersToMenuTask.desired_slip_term = f;
				break;
				case '7':
					presetParametersToMenuTask.max_integral_term = f;
				break;	
			}
		}	
		// Close file
		f_close(&file_object);
		// Filled the preset struct.. Send it to task menu
		xQueueSendToBack(xPresetQueue,&presetParametersToMenuTask,0);
	}
}


			//Note to self: Use str function is set to 0 which means no string functions are used in fatfs
			
			
			// f_unlink to delete files

			/*1 : always write in multiples of 4 bytes or sectors:
			  2 : Finn ut sector size. Skriv i multiples av sector sizes  
			  3 : Benchmarken til chanfat viser at å skrive 16 KB er 12 ganger så raskt som en sektor
			  4 : Kan jeg i det hele tatt lage en buffer på 16KB i programmet mitt nå ?   
			  5 : Prøv preallokering i bolker. Start med 100 mb.. når nærmer seg det. Fyll på med nye 100 mb  
			*/ 
			
			//Worst case scenario of 2 mbit/s of 8 bit messages on can buses -> 952 KB/S for sd card to write 
			// Add new line to written data
			//Bench results:
			//4KB buffers -> 1.5 -2 MB/s
			//16KB buffers -> 3-4 MB/s
			
			// 16 KB write bench med fopen og fclose. Ender opp på 500 KBIT etter at filen begynner å bli stor
			// 16 KB Only fsync -> 1.8 mb/s and 80 KB/s -> overloaded at 2000 messages / second.
			// If preallocation is used the speed is more stable and overload happens at somewhere around 5000-6000 messages / second
			
			// If i know the size of each write I can automatically move the write area pointer to the correct EOF and append. 
			// This means i can use f_seek with the correct pointer offset to get the correct EOF very fast
			// Research preallocation of files if this solution is not fast enough
			// Change the filename construction
			// Open and append and then close for each buffer write
			// Ready to write to the file // 
			//Get Can messages from queue and transfer them to SD card