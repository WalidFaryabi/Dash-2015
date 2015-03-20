#include "DriversNotInBase/revolve_can_definitions.h"

#ifndef CANID_DEFINITIONS_H_
#define CANID_DEFINITIONS_H_

#define ID_ALIVE					0x667

#define ID_TELEMETRY_CONTROL		0x678
#define ID_ECU_CAR_STATES			0x230
#define ID_ECU_LC					0x231
#define ID_TORQUE_ENCODER_0_DATA	0x210
#define ID_TORQUE_ENCODER_1_DATA	0x211
#define ID_BMS_MAX_MIN_VALUES		0x62D

#define ID_TRQ_CONF_CH0				(CANR_FCN_BOOT_ID | CANR_GRP_SENS_ROTARY_ID | CANR_MODULE_ID0_ID)
#define ID_TRQ_CONF_CH1				(CANR_FCN_BOOT_ID | CANR_GRP_SENS_ROTARY_ID | CANR_MODULE_ID1_ID)	

#define ID_STEERING_CONF			(CANR_FCN_BOOT_ID | CANR_GRP_SENS_ROTARY_ID | CANR_MODULE_ID4_ID)

#define ID_STEERING_ENCODER_DATA	0x638

#define ID_brake_pressure_front		0x618
#define ID_brake_pressure_rear		0x619

#define ID_speed_FL_and_avg			0x620
#define ID_speed_FR_and_avg			0x621
#define ID_speed_RL_and_avg			0x622
#define ID_speed_RR_and_avg			0x623

#define ID_DASH_ACKNOWLEDGE			0x660


#define ID_FINISHED_RTDS			0x263


//***************************//
//*****CALIBRATION IDs*******//
//***************************//
#define ID_TORQUE_PEDAL_CALIBRATION	0x460
#define ID_STEERING_CALIBRATION		0x461




#endif /* CANID_DEFINITIONS_H_ */