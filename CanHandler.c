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

// Hack to read a fraction of the inverter messages, since they are sent way too often. 
#define INVERTER_MESSAGE_DELAY_COUNTER 80
static uint32_t inverter_time_counter_status = 0;
static uint32_t inverter_time_counter_voltage = 0;

QueueHandle_t xDeviceStatusQueue = NULL;
QueueHandle_t xRemoteControlQueue = NULL;
QueueHandle_t xDataLoggerQueue = NULL;
QueueHandle_t xDashQueue = NULL;

uint32_t can_send_to_datalogger_queue_failed = 0;

static uint8_t torque_alive = 0;

void init_CanHandling() {
	//Init SensorPacket
	SensorPacket.CanMsg.data.u64  = 0;
	SensorPacket.CanMsg.dataLength = 0;
	SensorPacket.CanMsg.messageID = 0;
	SensorPacket.time_stamp = 0;
	
	// Queue Inits
	// xDataLoggerQueue needs the whole can message
	xDataLoggerQueue	= xQueueCreate(DATALOGGER_QUEUE_SIZE,sizeof(SensorPacket));
	xDashQueue			= xQueueCreate(20,sizeof(struct CanMessage));
	// Only needs one byte to identify the different buttons
	xRemoteControlQueue = xQueueCreate(20,sizeof(uint8_t));
	xDeviceStatusQueue	= xQueueCreate(20,sizeof(uint8_t));

	uint32_t accept[7] = {0};
	uint32_t id_mask[7] = {0};
	//can_init(CAN0,120000000,CAN_BPS_1000K,accept,id_mask,can0_Handler);
	//can_init(CAN1,120000000,CAN_BPS_1000K,accept,id_mask,can1_Handler);
	can_init(CAN0,120000000,CAN_BPS_1000K);
	can_init(CAN1,120000000,CAN_BPS_1000K);
	can_setupFilters(CAN0,accept,id_mask);
	can_setupFilters(CAN1,accept,id_mask);
	//can_enableRXInterrupt(CAN0);
	//can_enableRXInterrupt(CAN1);
}

// To read the RTT timer value register: (in ms)
//RTT->RTT_VR;


void CAN0_Handler() {
	NVIC_ClearPendingIRQ(CAN0_IRQn);
	if ( (CAN0->CAN_SR & 0b11111110)) {
		//Some receive mailbox has interrupted. Get message
		struct CanMessage message;
		long lHigherPriorityTaskWoken = pdFALSE;
		if (can_popMessage(CAN0, &message) == GOT_NEW_MESSAGE) {
			//can_popMessage(CAN0, &message);
			SensorPacket.CanMsg = message;
			SensorPacket.time_stamp = RTT->RTT_VR;	
			if (xQueueSendToBackFromISR(xDataLoggerQueue,&SensorPacket,&lHigherPriorityTaskWoken) != pdTRUE) {
				can_send_to_datalogger_queue_failed += 1;
			}
		
			switch (message.messageID) {
				case ID_ALIVE:	
					switch (message.data.u8[0]) {
						case ALIVE_TRQ:
						// Tells de
						torque_alive = ALIVE_TRQ_CAN_0;
						xQueueSendToBackFromISR(xDeviceStatusQueue, &torque_alive, NULL);
						break;
						case ALIVE_TRQ_UNINIT:
						torque_alive = ALIVE_UNINIT_TRQ_CAN_0;
						xQueueSendToBackFromISR(xDeviceStatusQueue, &torque_alive, NULL);
						break;
						default:
						xQueueSendToBackFromISR(xDeviceStatusQueue, &message.data.u8[0], NULL);
						break;
					}
				break;
				case ID_TELEMETRY_CONTROL:
					xQueueSendToBackFromISR(xRemoteControlQueue, &message.data.u8[0],NULL);
				break;
				//***************************************//
				//--------------DASH TASK----------------//
				//***************************************//
				case ID_TRQ_CONF:
					message.messageID = ID_TRQ_CONF_CH0;
					xQueueSendToBackFromISR(xDashQueue,&message,NULL);
				break;
				case ID_TORQUE_ENCODER_1_DATA:
					xQueueSendToBackFromISR(xDashQueue,&message,NULL);
					break;
				case ID_TORQUE_ENCODER_0_DATA:
				xQueueSendToBackFromISR(xDashQueue,&message,NULL);
				break;
				case ID_IN_ECU_TRACTION_CONTROL:
				case ID_ECU_PARAMETER_CONFIRMED:
				
				//*************************//
				//*****BATTERY SYSTEMS*****//
				case ID_BMS_STATE_MESSAGE:
				case BMS_MAXMIN_VTG_ID:
				case BMS_TOTVTG_ID:
				case BMS_MAXMIN_TEMP_ID:
				case BMS_CURRENT_ID:
				
				case GLVBMS_MAXMIN_VAL_ID:
				case GLVBMS_TOTVTG_ID:
				//case CAN_INVERTER_DATA_VOLTAGE_ID:
				//IMU
// 				case ID_IMU_ROT_DATA:
// 				case ID_IMU_G_FORCE_DATA:
// 				case ID_IMU_VELOCITY_DATA:
// 				case ID_IMU_POSITION_DATA:
				
				case ID_ECU_CAR_STATES:
				case ID_IN_ECU_LC:
				//************//
				//*****ADC****//
				case ID_SPEED_FL:
				case ID_SPEED_FR:
				case ID_SPEED_RR:
				case ID_SPEED_RL:
				case ID_TEMP_COOLING:
				case ID_TEMP_GEARBOX:
				case ID_TEMP_MOTOR_2:
				case ID_BRAKE_PRESSURE_FL:
				case ID_BRAKE_PRESSURE_FR:
				case ID_DAMPER_FL:
				case ID_DAMPER_FR:
				case ID_DAMPER_RL:
				case ID_DAMPER_RR:
				case ECU_CURRENT_IMPLAUSIBILITY_DATA:
				case ID_BSPD_STATUS:
				case ID_IMD_STATUS:
				case ID_FAN_STATUS:
					xQueueSendToBackFromISR(xDashQueue,&message,NULL);
				break;	
				case CAN_INVERTER_DATA_STATUS_ID:
					inverter_time_counter_status += 1;
					if (inverter_time_counter_status == INVERTER_MESSAGE_DELAY_COUNTER){
						inverter_time_counter_status = 0;
						xQueueSendToBackFromISR(xDashQueue,&message,NULL);
					}
				break;
				
				case CAN_INVERTER_DATA_VOLTAGE_ID:
					inverter_time_counter_voltage += 1;
					if (inverter_time_counter_voltage == INVERTER_MESSAGE_DELAY_COUNTER){
						inverter_time_counter_voltage = 0;
						xQueueSendToBackFromISR(xDashQueue,&message,NULL);
					}
				break;
			}
		}
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
	}	
}

void CAN1_Handler() {
	NVIC_ClearPendingIRQ(CAN1_IRQn);
	
	if ( (CAN1->CAN_SR & 0b11111110)) {
		//Some receive mailbox has interrupted. Get message
		struct CanMessage message;
		long lHigherPriorityTaskWoken = pdFALSE;
		if (can_popMessage(CAN1, &message) == GOT_NEW_MESSAGE) {
			SensorPacket.CanMsg = message;
			SensorPacket.time_stamp = RTT->RTT_VR;
	
			if (xQueueSendToBackFromISR(xDataLoggerQueue,&SensorPacket,&lHigherPriorityTaskWoken) != pdTRUE) {
				can_send_to_datalogger_queue_failed += 1;
			}
			switch (message.messageID) {
				case ID_ALIVE:
					switch (message.data.u8[0]) {
						case ALIVE_TRQ:
							torque_alive = ALIVE_TRQ_CAN_1;
							xQueueSendToBackFromISR(xDeviceStatusQueue, &torque_alive, NULL);
							break;
						case ALIVE_TRQ_UNINIT:
							torque_alive = ALIVE_UNINIT_TRQ_CAN_1;
							xQueueSendToBackFromISR(xDeviceStatusQueue, &torque_alive, NULL);
							break;
						default:
							xQueueSendToBackFromISR(xDeviceStatusQueue, &message.data.u8[0], NULL);
							break;
					}
				break;
				//***************************************//
				//--------------DASH TASK----------------//
				//***************************************//
				case ID_TRQ_CONF:
					message.messageID = ID_TRQ_CONF_CH1;
					xQueueSendToBackFromISR(xDashQueue,&message,NULL);
				break;
				
				case ID_BMS_STATE_MESSAGE:
				case BMS_MAXMIN_VTG_ID:
				case BMS_TOTVTG_ID:
				case BMS_TEMP_ID:
				case BMS_MAXMIN_TEMP_ID:
				case BMS_CURRENT_ID:
					
				case GLVBMS_MAXMIN_VAL_ID:
				case GLVBMS_TOTVTG_ID:
				//case CAN_INVERTER_DATA_VOLTAGE_ID:
					
				//*********ECU RELATED***********//
				case ID_ECU_CAR_STATES:
				case ID_IN_ECU_LC:
				case ID_IN_ECU_TRACTION_CONTROL:
				case ID_ECU_PARAMETER_CONFIRMED:
				//*******************************//
				
				//*******TORQUE AND STEERING*****//
				case ID_TORQUE_ENCODER_0_DATA:
				case ID_TORQUE_ENCODER_1_DATA:
				//*******************************//
				
				//*************ADC***************//
				case ID_SPEED_FL:
				case ID_SPEED_FR:
				case ID_SPEED_RR:
				case ID_SPEED_RL:
				case ID_TEMP_COOLING:
				case ID_TEMP_GEARBOX:
				case ID_BRAKE_PRESSURE_FL:
				case ID_BRAKE_PRESSURE_FR:
				case ID_DAMPER_FL:
				case ID_DAMPER_FR:
				case ID_DAMPER_RL:
				case ID_DAMPER_RR:
				//*******************************//
				case ID_IMD_SHUTDOWN:
				case ECU_CURRENT_IMPLAUSIBILITY_DATA:
				case ID_BSPD_STATUS:
				case ID_IMD_STATUS:
					xQueueSendToBackFromISR(xDashQueue,&message,NULL);
				break;
				case CAN_INVERTER_DATA_STATUS_ID:
					inverter_time_counter_status += 1;
					if (inverter_time_counter_status == INVERTER_MESSAGE_DELAY_COUNTER){
						inverter_time_counter_status = 0;
						xQueueSendToBackFromISR(xDashQueue,&message,NULL);
					}
				break;
				
				case CAN_INVERTER_DATA_VOLTAGE_ID:
					inverter_time_counter_voltage += 1;
					if (inverter_time_counter_voltage == INVERTER_MESSAGE_DELAY_COUNTER){
						inverter_time_counter_voltage = 0;
						xQueueSendToBackFromISR(xDashQueue,&message,NULL);
					}
				break;
			}
		}
		portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
	}
}
