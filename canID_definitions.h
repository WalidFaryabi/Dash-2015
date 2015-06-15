#include "DriversNotInBase/revolve_can_definitions.h"

#ifndef CANID_DEFINITIONS_H_
#define CANID_DEFINITIONS_H_

#define ID_ALIVE					0x667

#define ID_TELEMETRY_CONTROL		0x678

//***************************************//
//***********OUT GOING*******************//
//***************************************//
#define ID_ECU_DRIVE				0x260
#define ID_ECU_TRACTION_CONTROL		0x261
#define ID_ECU_LC					0x262
#define ID_FINISHED_RTDS			0x263
#define ID_ECU_PARAMETERS			0x264
#define ID_ECU_P_TERM				0x265
#define ID_ECU_I_TERM				0x266


#define ID_TORQUE_PEDAL_CALIBRATION	0x460
#define ID_STEERING_CALIBRATION		0x461

#define ID_DASH_ACKNOWLEDGE			0x660
#define ID_I_AM_ALIVE				0x661

//***************************************//
//************INCOMING*******************//
//***************************************//

#define	ID_ECU_CAR_STATES			0x230
#define ID_ECU_PARAMETER_CONFIRMED	0x231
#define ID_IN_ECU_LC				0x232
#define ID_IN_ECU_TRACTION_CONTROL	0x234
#define ID_IN_ECU_SELECTED_PRESET	0x235



#define ID_BMS_TRACTIVE_SYSTEM_ACTIVE 0x62B
#define ID_BMS_MAX_MIN_VALUES		0x62D
#define ID_BMS_STATE_MESSAGE		0x42A

#define ID_TRQ_CONF					0x210
#define ID_TRQ_CONF_CH0				0x999				//(CANR_FCN_BOOT_ID | CANR_GRP_SENS_ROTARY_ID | CANR_MODULE_ID0_ID)
#define ID_TRQ_CONF_CH1				0x998				//(CANR_FCN_BOOT_ID | CANR_GRP_SENS_ROTARY_ID | CANR_MODULE_ID1_ID)	
#define ID_TORQUE_ENCODER_0_DATA	0x212
#define ID_TORQUE_ENCODER_1_DATA	0x213

#define ID_STEERING_ENCODER_DATA	0x614

#define ID_STEERING_CONF			0x214				//(CANR_FCN_BOOT_ID | CANR_GRP_SENS_ROTARY_ID | CANR_MODULE_ID4_ID)
#define ID_STEERING_ENCODER_DATA	0x614

#define ID_IMD_SHUTDOWN				0x671

#define ID_BRAKE_PRESSURE_FR		0x618
#define ID_BRAKE_PRESSURE_FL		0x619

#define ID_SPEED_FR					0x620
#define ID_SPEED_FL					0x621
#define ID_SPEED_RR					0x622
#define ID_SPEED_RL					0x623

#define ID_DAMPER_FR				0x650
#define ID_DAMPER_FL				0x651
#define ID_DAMPER_RR				0x652
#define ID_DAMPER_RL				0x653

#define ID_TEMP_COOLING				0x656
#define ID_TEMP_GEARBOX				0x657

#define ID_IMU_ROT_DATA				0x649	
#define ID_IMU_G_FORCE_DATA			0x648
#define ID_IMU_POSITION_DATA		0x64A
#define ID_IMU_VELOCITY_DATA		0x64B
	

#define ALIVE_TRQ_CAN_0				0x14
#define ALIVE_TRQ_CAN_1				0x15
#define ALIVE_UNINIT_TRQ_CAN_0		0x16
#define ALIVE_UNINIT_TRQ_CAN_1		0x17

#endif /* CANID_DEFINITIONS_H_ */