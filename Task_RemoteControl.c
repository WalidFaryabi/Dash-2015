/*
 * CFile1.c
 *
 * Created: 10.02.2015 11:51:29
 *  Author: will
 */ 

#include "Task_RemoteControl.h"

#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/semphr.h"

#include "CanHandler.h"
#include "sam4e-base/RevolveDrivers/can.h"
#include "Task_Menu.h"

//deler køopplegg med CAN, og btn struct med dashboard



//Buttontask vil kun gjøre noe med buttons hvis unhandledbuttonaction = false. 
//Man er derfor sikret at telemtrikommandoer ikke vil kødde buttontask

void Task_remoteControl() {
		struct CanMessage txmsg = {
			.data.u8[0] = 10,
			.data.u8[1] = 5,
			.dataLength = 2,
			.messageID = 10
		};
	
	TickType_t xLastwakeTime;
	portBASE_TYPE xStatus;
	uint8_t telemetri_btn = 0;
	const portTickType xTicksToWait = 5 / portTICK_RATE_MS;
	while(1) {
		//try to get message from queue
		xStatus = xQueueReceive( xRemoteControlQueue, &telemetri_btn, portMAX_DELAY);
		if ( xStatus == pdPASS) {
			//do something with data in lreceivedvalue
			xSemaphoreTake(xButtonStruct,portMAX_DELAY); // Wait indefinetely for access to Button struct
			//can_sendMessage(CAN0,txmsg);
			if (btn.unhandledButtonAction == false) {
				switch(telemetri_btn) {
					case 1: // navigation_up
						btn.btn_type = NAVIGATION;
						btn.unhandledButtonAction = true;
						btn.navigation = UP;
						break;
					case 2: //navigation_down
						btn.btn_type = NAVIGATION;
						btn.unhandledButtonAction = true;
						btn.navigation = DOWN;
						break;
					case 3: // navigation_left
						btn.btn_type = NAVIGATION;
						btn.unhandledButtonAction = true;
						btn.navigation = LEFT;
						break;
					case 4: // navigation_right
						btn.btn_type = NAVIGATION;
						btn.unhandledButtonAction = true;
						btn.navigation = RIGHT;
						break;
					case 5: // rotary_cw
						btn.btn_type = ROTARY;
						btn.unhandledButtonAction = true;
						btn.rotary_cw = true;
						break;
					case 6: // rotary_ccw
						btn.btn_type = ROTARY;
						btn.unhandledButtonAction = true;
						btn.rotary_ccw = true;
						break;
					case 7: // acknowledge
						btn.btn_type = PUSH_ACK;
						btn.unhandledButtonAction = true;
						break;
					default:
						break;	
				}
			}
			xSemaphoreGive(xButtonStruct);
			
		}
		vTaskDelayUntil(&xLastwakeTime,30/portTICK_RATE_MS);
	}

}