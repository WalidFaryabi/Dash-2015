/*
 * canHandler.c
 *
 * Created: 04.02.2015 22:49:40
 *  Author: Will
 */ 
#include "sam.h"
#include "CanHandler.h"
#include "sam4e-base/RevolveDrivers/can.h"
#include "DriversNotInBase/revolve_can_definitions.h"
#include "canID_definitions.h"

/*How to get Can messages into a byte buffer of a suitable size:
ID :.id = CANR_FCN_CMD_ID | CANR_GRP_DASH_ID | CANR_MODULE_ID0 -> convert to what */

#define DATALOGGER_QUEUE_SIZE 250

struct SensorMessage {
	struct CanMessage CanMsg;
	uint32_t time_stamp;
};
struct SensorMessage SensorPacket;


QueueHandle_t xDeviceStatusQueue = NULL;
QueueHandle_t xRemoteControlQueue = NULL;
QueueHandle_t xDataLoggerQueue = NULL;
QueueHandle_t xDashQueue = NULL;


void init_CanHandling() {
	//Init SensorPacket
	SensorPacket.CanMsg.data.u64  = 0;
	SensorPacket.CanMsg.dataLength = 0;
	SensorPacket.CanMsg.messageID = 0;
	SensorPacket.time_stamp = 0;
	
	// Queue Inits
	// xDataLoggerQueue needs the whole can message
	xDataLoggerQueue	= xQueueCreate(DATALOGGER_QUEUE_SIZE,sizeof(SensorPacket));
	xDashQueue			= xQueueCreate(20,sizeof(SensorPacket));
	// Only needs one byte to identify the different buttons
	xRemoteControlQueue = xQueueCreate(20,sizeof(uint8_t));
	xDeviceStatusQueue	= xQueueCreate(20,sizeof(uint8_t));


	can_init(CAN0,120000000,CAN_BPS_1000K);
	can_init(CAN1,120000000,CAN_BPS_1000K);
	uint32_t accept[7] = {0};
	uint32_t id_mask[7] = {0};
	can_setupFilters(CAN0,accept,id_mask);
	can_setupFilters(CAN1,accept,id_mask);
}

// To read the RTT timer value register: (in ms)
//RTT->RTT_VR;

static uint32_t dataloggerFailCounter = 0; // For testing capacity of datalogging

void CAN0_Handler() {
	NVIC_ClearPendingIRQ(CAN0_IRQn);
	if ( (CAN0->CAN_SR & 0b11111110)) {
		//Some receive mailbox has interrupted. Get message
		struct CanMessage message;
		can_popMessage(CAN0, &message);
		SensorPacket.CanMsg = message;
		SensorPacket.time_stamp = RTT->RTT_VR;
		
		struct CanMessage dataloggerFail = {
			.data.u8[0] = 1,
			.dataLength = 1,
			.messageID = 0x002
			};
		
		// Convert can message to json format 
		long lHigherPriorityTaskWoken = pdFALSE;
		if (xQueueSendToBackFromISR(xDataLoggerQueue,&SensorPacket,&lHigherPriorityTaskWoken) != pdTRUE) {
			dataloggerFailCounter += 1;
			dataloggerFail.data.u32[0] = dataloggerFailCounter;
			// Send a can message to inform that datalogger has fallen behind
			// Thread safety on can ?
			// Remember to turn of all other can sendmessages when testing this !!!!
			while (can_sendMessage(CAN0,dataloggerFail) != TRANSFER_OK);
			//Queue is full. What to do ?
		}
		
		switch (message.messageID) {
			case ID_ALIVE:
				xQueueSendToBackFromISR(xDeviceStatusQueue, &message.data.u8[0], NULL);
			break;
			case ID_TELEMETRY_CONTROL:
				xQueueSendToBackFromISR(xRemoteControlQueue, &message.data.u8[0],NULL);
			break;
			//***************************************//
			//--------------DASH TASK----------------//
			//***************************************//
			case ID_BMS_MAX_MIN_VALUES:
			case ID_TRQ_CONF_CH0:
			case ID_TRQ_CONF_CH1:
			case ID_ECU_CAR_STATES:
			case 11:
				xQueueSendToBackFromISR(xDashQueue,&message,NULL);
			break;
			
			
			
		}
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
	}	
}

void CAN1_Handler() {
	NVIC_ClearPendingIRQ(CAN1_IRQn);
	
	if ( (CAN1->CAN_SR & 0b11111110)) {
		//Some receive mailbox has interrupted. Get message
		struct CanMessage message;
		can_popMessage(CAN1, &message);
		SensorPacket.CanMsg = message;
		SensorPacket.time_stamp = RTT->RTT_VR;
		
		struct CanMessage dataloggerFail = {
			.data.u8[0] = 2,
			.dataLength = 1,
			.messageID = 0x660
		};
		//can_sendMessage(CAN1,dataloggerFail);
		// Convert can message to json format
		long lHigherPriorityTaskWoken = pdFALSE;
		if (xQueueSendToBackFromISR(xDataLoggerQueue,&SensorPacket,&lHigherPriorityTaskWoken) != pdTRUE) {
			dataloggerFailCounter += 1;
			dataloggerFail.data.u32[0] = dataloggerFailCounter;
			// Send a can message to inform that datalogger has fallen behind
			// Thread safety on can ?
			// Remember to turn of all other can sendmessages when testing this !!!!
			//while (can_sendMessage(CAN1,dataloggerFail) != TRANSFER_OK);
			//Queue is full. What to do ?
		}
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
	}
}