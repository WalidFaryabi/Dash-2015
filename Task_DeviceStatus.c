

#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/queue.h"
#include "sam4e-base/FreeRTOS/Source/include/timers.h"

#include "Task_Menu.h"
#include "CanHandler.h"
#include "canID_definitions.h"
#include "DriversNotInBase/revolve_can_definitions.h"



#define NUM_DEVICES 16
TimerHandle_t deviceTimer[NUM_DEVICES];
/*
ECU = 0
TRQ_0 = 1
BSPD = 2
TEL = 3
ADC_FR = 4
ADC_FL = 5
ADC_RR = 6
ADC_RL = 7
INV = 8
FAN = 9
BMS = 10
GLVBMS = 11
IMU = 12
STEER_POS = 13
IMD = 14
TRQ_1 = 15
*/
void vDeviceTimerCallback(TimerHandle_t pxTimer) {
	uint8_t device_id;
	device_id = (uint8_t) pvTimerGetTimerID(pxTimer);
	switch (device_id) {
		case 0:
		deviceState.ECU = DEAD;
		break;
		case 1:
		deviceState.TRQ_0 = DEAD;
		break;
		case 2:
		deviceState.BSPD = DEAD;
		break;
		case 3:
		deviceState.TEL = DEAD;
		break;
		case 4:
		deviceState.ADC_FR = DEAD;
		break;
		case 5:
		deviceState.ADC_FL = DEAD;
		break;
		case 6:
		deviceState.ADC_RR = DEAD;
		break;
		case 7:
		deviceState.ADC_RL = DEAD;
		break;
		case 8:
		deviceState.INV  = DEAD;
		break;
		case 9:
		deviceState.FAN = DEAD;
		break;
		case 10:
		deviceState.BMS = DEAD;
		break;
		case 11:
		deviceState.GLVBMS = DEAD;
		break;
		case 12:
		deviceState.IMU = DEAD;
		break;
		case 13:
		deviceState.STEER_POS = DEAD;
		break;
		case 14:
		deviceState.IMD = DEAD;
		break;
		case 15:
		deviceState.TRQ_1 = DEAD;
		break;
	}
}

void deviceStatusTask() {
	TickType_t xLastWakeTime;
	portBASE_TYPE xStatus;
	const portTickType xTicksToWait = 1 / portTICK_RATE_MS;
	uint8_t device_id;
	for (uint8_t x = 0 ; x < NUM_DEVICES; x++) {
		deviceTimer[x] = xTimerCreate("Device_Timer",1500/portTICK_RATE_MS, pdTRUE, (void *) x ,vDeviceTimerCallback);
		if (deviceTimer[x] == NULL) {	
		//Timer not created .. Heap?
		}
		else if (xTimerStart(deviceTimer[x],0) != pdPASS) {
			//Start not successfull
		}
	}
	while(1) {
		while (xQueueReceive( xDeviceStatusQueue, &device_id, xTicksToWait) == pdPASS ) {
		//xStatus = xQueueReceive( xDeviceStatusQueue, &device_id, xTicksToWait);
		//if ( xStatus == pdPASS) {
			switch (device_id) {
				case ALIVE_ECU:
					xTimerReset(deviceTimer[0],1/portTICK_RATE_MS);
					deviceState.ECU = ALIVE;
					break;
				case ALIVE_TRQ_CAN_0:
					xTimerReset(deviceTimer[1],1/portTICK_RATE_MS);
					deviceState.TRQ_0 = ALIVE;
					break;
				case ALIVE_TRQ_CAN_1:
					xTimerReset(deviceTimer[15],1/portTICK_RATE_MS);
					deviceState.TRQ_1 = ALIVE;
					break;		
				case ALIVE_UNINIT_TRQ_CAN_0:
					xTimerReset(deviceTimer[1],1/portTICK_RATE_MS);
					deviceState.TRQ_0 = UNITIALIZED;
					break;
				case ALIVE_UNINIT_TRQ_CAN_1:
					xTimerReset(deviceTimer[15],1/portTICK_RATE_MS);
					deviceState.TRQ_1 = UNITIALIZED;
					break;						
				case ALIVE_BSPD:
					xTimerReset(deviceTimer[2],1/portTICK_RATE_MS);
					deviceState.BSPD = ALIVE;
					break;
				case ALIVE_TELEMETRY:
					xTimerReset(deviceTimer[3],1/portTICK_RATE_MS);
					deviceState.TEL = ALIVE;
					break;
				case ALIVE_ADC_FR:
					xTimerReset(deviceTimer[4],1/portTICK_RATE_MS);
					deviceState.ADC_FR = ALIVE;
					break;
				case ALIVE_ADC_FL:
					xTimerReset(deviceTimer[5],1/portTICK_RATE_MS);
					deviceState.ADC_FL = ALIVE;
					break;
				case ALIVE_ADC_RR:
					xTimerReset(deviceTimer[6],1/portTICK_RATE_MS);
					deviceState.ADC_RR = ALIVE;
					break;
				case ALIVE_ADC_RL:
					xTimerReset(deviceTimer[7],1/portTICK_RATE_MS);
					deviceState.ADC_RL = ALIVE;
					break;
				case ALIVE_INVERTER:
					xTimerReset(deviceTimer[8],1/portTICK_RATE_MS);
					deviceState.INV = ALIVE;
					break;
				case ALIVE_FAN:
					xTimerReset(deviceTimer[9],1/portTICK_RATE_MS);
					deviceState.FAN = ALIVE;
					break;
				case ALIVE_BMS:
					xTimerReset(deviceTimer[10],1/portTICK_RATE_MS);
					deviceState.BMS = ALIVE;
					break;
				case ALIVE_GLVBMS:
					xTimerReset(deviceTimer[11],1/portTICK_RATE_MS);
					deviceState.GLVBMS = ALIVE;
					break;
				case ALIVE_IMU:
					xTimerReset(deviceTimer[12],1/portTICK_RATE_MS);
					deviceState.IMU = ALIVE;
					break;
				case ALIVE_STEER_POS:
					xTimerReset(deviceTimer[13],1/portTICK_RATE_MS);
					deviceState.STEER_POS = ALIVE;
					break;
				case ALIVE_IMD:
					xTimerReset(deviceTimer[14],1/portTICK_RATE_MS);
					deviceState.IMD = ALIVE;
					break;
				default:
				break;
			}
		}
		vTaskDelay(100/portTICK_RATE_MS);
	}
}