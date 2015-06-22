#ifndef DASHBOARDMENU_H_
#define DASHBOARDMENU_H_

#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/semphr.h"
#include "sam4e-base/FreeRTOS/Source/include/timers.h"

#include "DriversNotInBase/spi.h"

#include <stdbool.h>

typedef enum {MAIN_SCREEN,SYSTEM_MONITOR,TEMP_VOLT,MAIN_MENU,MAIN_MENU_DOWN, KERS_OPTION,DEVICE_STATUS,
			  SPEED,TRQ_CALIB,PERSISTENT_MSG,ECU_OPTIONS,LC_HANDLER,ERROR_HANDLER,SNAKE_GAME,
			  STEER_CALIB,DL_OPTIONS,LOCKED_SEL,PRESET_SEL,PRESET_PROCEDURE,PRESET_CONFIRM, 
			  IMU_INFORMATION,SENSOR_DATA, TS_STATUS,FAN_OPTIONS} EMenuName;
			  
typedef enum {NO_SETTING,TORQUE_SETTING,KERS_SETTING, TRACTION_CONTROL_SETTING, DL_PREALLOCATE,PRESET_1_SETTING,PRESET_2_SETTING,
			  PRESET_3_SETTING,PRESET_4_SETTING,PRESET_5_SETTING,PRESET_6_SETTING,PRESET_7_SETTING,PRESET_8_SETTING, CONFIRM_YES, CONFIRM_NO,
			  PUMP_SETTING, RADIATOR_FAN_SETTING, MONO_FAN_SETTING, BATTERY_FAN_SETTING,ALL_FAN_SETTING } EAdjustmentParameter;
			  
typedef enum {NAVIGATION,PUSH_ACK,ROTARY,ROT_ACK,START,LAUNCH_CONTROL,NONE_BTN} EButtonType;
typedef enum {UP,DOWN,LEFT,RIGHT,NAV_DEFAULT} ENavigationDirection;
typedef enum {CCW,CW} ERotary_direction;
typedef enum {ENABLE_SWITCH,DISABLE_SWITCH} ESwitchState;	
typedef enum {TRACTIVE_SYSTEM_ON,TRACTIVE_SYSTEM_OFF,DRIVE_ENABLED, 
			LC_PROCEDURE,LC_STANDBY,LC_COUNTDOWN,LC_WAITING_FOR_ECU_TO_ARM_LC,LC_ARMED,LC_ABORTED,LC_ARMING_TIMED_OUT} ECarState;

//typedef enum {LC_OFF,LC_REQUESTED,LC_PROCEDURE,LC_ARMED,LC_LAUNCHED} ELaunchControlStates;
typedef enum {ALIVE,DEAD,UNITIALIZED} EAlive;
typedef enum {TRQ_ENC_LEFT, TRQ_ENC_RIGHT,TRQ_ENC_NONE} ETrqEncSide;
typedef enum {TRQ_MAX_CONFIRMED,TRQ_MIN_CONFIRMED,TRQ_NOCALIB,TRQ_DEFAULT} ETrqConfStates;
	
typedef enum {TRQ_CALIBRATION_OFF,TRQ_CALIBRATION_WAITING_MAX_CONFIRMATION,TRQ_CALIBRATION_WAITING_MIN_CONFIRMATION,
	TRQ_CALIBRATION_MAX_CONFIRMED,TRQ_CALIBRATION_MIN_CONFIRMED, TRQ_FAIL_BOTH_CH,TRQ_FAIL_CH0,
	TRQ_FAIL_CH1, TRQ_TIMEOUT_BOTH_CH,TRQ_TIMEOUT_CH0,TRQ_TIMEOUT_CH1} ETorquePedalCalibrationState ;
	
typedef enum {STEER_C_OFF, STEER_C_WAITING_LEFT, STEER_C_LEFT_CONFIRMED, STEER_C_WAITING_RIGHT, STEER_C_RIGHT_CONFIRMED, STEER_C_FAIL, STEER_C_TIMEOUT} ESteerCalibState;
typedef enum {STEER_CONF_LEFT,STEER_CONF_RIGHT, STEER_CONF_FAILED, STEER_CONF_DEFAULT} ESteerConfStates;
	
typedef enum {PRESET_PROCEDURE_OFF,PRESET_PROCEDURE_INIT, PRESET_PROCEDURE_WAITING, 
			  PRESET_PROCEDURE_SEND_P_TERM, WAIT_P_TERM, PRESET_PROCEDURE_SEND_I_TERM, WAIT_I_TERM,
			  PRESET_PROCEDURE_SEND_D_TERM, WAIT_D_TERM, PRESET_PROCEDURE_SEND_MAX_MIN_TERM, WAIT_MAX_MIN_TERM,
			  PRESET_PROCEDURE_SEND_MAX_DECREASE_TERM, WAIT_MAX_DECREASE_TERM,PRESET_PROCEDURE_SEND_DESIRED_SLIP_TERM, 
			  WAIT_DESIRED_SLIP_TERM, PRESET_PROCEDURE_SEND_MAX_INTEGRAL_TERM, WAIT_MAX_INTEGRAL_TERM, PRESET_PROCEDURE_SEND_SELECTED_PRESET,
			  WAIT_SELECTED_PRESET, PRESET_PROCEDURE_FINISHED,PRESET_PROCEDURE_FAILED} EPresetStates;
			  
			  
			  
//*******************************************************************************//
//************************ DEFINES***********************************************//
//*******************************************************************************//

//***********************************************************************************
//----------------------------------THRESHOLDS AND CRITICAL VALUES-----------------//
//***********************************************************************************
#define TORQUE_PEDAL_IN_THRESHOLD	100 // 10 % of total range which is 1000
#define BRAKE_PEDAL_IN_THRESHOLD	1600

#define BATTERY_TEMP_CRITICAL_HIGH 90
#define MAX_CELL_VOLTAGE_TRESHOLD 4.2
#define MIN_CELL_VOLTAGE_TRESHOLD 3.2
#define BATTERY_PACK_MAX_VOLTAGE_TRESHOLD 600
#define BATTERY_PACK_MIN_VOLTAGE_TRESHOLD 475

#define GLV_MAX_CELL_VOLTAGE_TRESHOLD 4.2
#define GLV_MIN_CELL_VOLTAGE_TRESHOLD 3.2
#define GLV_PACK_MAX_VOLTAGE_TRESHOLD 28
#define GLV_PACK_MIN_VOLTAGE_TRESHOLD 20

// BMS/GLVBMS TEMPERATURE CONVERSION COEFFICIENTS
#define TEMP_C_1 0.000000000007175
#define TEMP_C_2 0.000000367
#define TEMP_C_3 0.009898
#define TEMP_C_4 124.831

// IMU CONVERSION CONSTANTS
#define IMU_ROT_G_C	0.000125
#define IMU_VEL_C	0.00076923
#define IMU_POS_C	10.0

#define BMS_MAX_TEMP_TRESHOLD		45
#define GLVBMS_MAX_TEMP_THRESHOLD	45

#define GLV_BATTERY_FULL_VOLTAGE	29.4
#define GLV_BATTERY_EMPTY_VOLTAGE	21.7
#define GLV_BATTERY_VOLTAGE_RANGE	(GLV_BATTERY_FULL_VOLTAGE-GLV_BATTERY_EMPTY_VOLTAGE)

#define HV_BATTERY_TOTAL_CURRENT	12.5	
#define HV_BATTERY_FULL_VOLTAGE		604.8
#define HV_BATTERY_EMPTY_VOLTAGE	446.4
#define HV_BATTERY_VOLTAGE_RANGE	(HV_BATTERY_FULL_VOLTAGE - HV_BATTERY_EMPTY_VOLTAGE)

// BMS STATE VECTOR
#define BMS_OVER_VOLTAGE		1
#define BMS_UNDER_VOLTAGE		2
#define BMS_OVER_CURRENT		4
#define BMS_OVER_TEMPERATURE	8
#define BMS_VIC_STATUS			16

// INVERTER DATA STATUS VECTOR
#define INVERTER_ENCODER_NOT_FOUND	(1<<16)	// Bit 16
#define INVERTER_ENABLED			(1<<24) // Bit 24

// ECU IMPLAUSIBILITIES VECTOR
//0x00 = NONE, 0x01 TPS_MISMATCH, 0x02 TPS_BPS_IMPLAUSIBILIY, 0x03 SENSOR_ERROR, 0x04 OUTDATED DATA
#define ECU_NO_IMPLAUSIBILITY		0x00
#define ECU_TPS_MISMATCH			0x01
#define ECU_TPS_BPS_IMPLAUSIBILITY	0x02
#define	ECU_SENSOR_ERROR			0x03
#define ECU_OUTDATED_DATA			0x04

// BSPD STATUS
#define BSPD_SHUTDOWN_NOT_ACTIVE	(1<<4)

// IMD STATUS
//0hz(IMD off or short to KL31), 10hz(Normal condition), 20hz(Under voltage), 30hz(Speed start), 40hz(IMD error), 50hz(GroundError), melding hvert sekund
#define IMD_OFF				0
#define IMD_NORMAL			10
#define IMD_UNDER_VOLTAGE	20
#define IMD_SPEED_START		30
#define IMD_ERROR			40
#define IMD_GROUND_ERROR	50

typedef struct ButtonsOnDashboard {
	bool unhandledButtonAction;
	EButtonType btn_type;
	ENavigationDirection navigation;
	bool rotary_cw; // Clockwise
	bool rotary_ccw; // Counter clockwise
	} Buttons;
	
typedef struct ConfirmationMessagesReceivedOverCan {
	bool drive_disabled_confirmed;
	bool drive_enabled_confirmed;
	bool lc_request_confirmed;
	bool lc_ready;
	bool lc_off;
	ESteerConfStates conf_steer;
	ETrqConfStates conf_trq_ch0;
	ETrqConfStates conf_trq_ch1;
	
	bool ECU_parameter_confirmed;
	
	// maybe confirm variable settings ?
	} ConfirmationMsgs;
	
typedef struct DeviceStatus {
	EAlive ECU;
	EAlive TRQ_0;
	EAlive TRQ_1;
	EAlive BSPD;
	EAlive TEL;
	EAlive ADC_FR;
	EAlive ADC_FL;
	EAlive ADC_RR;
	EAlive ADC_RL;
	EAlive INV; // Inverter
	EAlive FAN;
	EAlive BMS;
	EAlive GLVBMS;
	EAlive IMU;
	EAlive STEER_POS;
	EAlive IMD;
} DeviceState;

typedef struct ModuleErrorsReceivedOverCan{
	uint16_t BMS_state_vector;
	uint32_t inverter_data_status;
	uint8_t bspd_status;
	uint8_t ECU_implausibility;
	uint8_t IMD_status;
	bool ams_error; // Assume there will be sent msgs for NOTOK and OK
	bool imd_error; // Assume there will be sent msgs for NOTOK and OK
	
	uint8_t ecu_error; // Store only one error ID at a time
	uint8_t bms_fault; // Store only one error ID at a time
	uint8_t bms_warning;
	} ModuleError;
	

typedef struct StatusMessagesCan {
	bool shut_down_circuit_closed;
	} StatusMsg;

typedef struct ParameterValues { // Values of adjustable variables
	uint32_t min_torque;
	uint32_t torque;
	uint32_t confirmed_torque;
	uint32_t max_torque;
	
	uint16_t min_kers_value;
	uint16_t kers_value;
	uint16_t confirmed_kers_value;
	uint16_t max_kers_value;
	
	uint8_t traction_control_value;
	uint8_t confirmed_traction_control_value;
	
	uint8_t min_fan_duty_cycle;
	uint8_t max_fan_duty_cycle;
	uint8_t all_fan_setting;
	uint8_t radiator_fan_value;
	uint8_t mono_fan_value;
	uint8_t battery_fan_value;
	uint8_t pump_setting_value;
	/* If acknowledge is pressed while adjusting a parameter, a timer is started, if a confirmation msg is received 
	before it times out the parameter of the current parameter adjustment menu is confirmed. This is handled in
	getDashMessages function. If the user presses left the previous confirmed parameter will be displayed again*/
	} ParameterValue;
	
typedef struct SensorValuesReceivedOverCan {
	uint16_t brake_pressure_fr;
	uint16_t brake_pressure_fl;
	uint16_t temp_sensor_cooling;
	uint16_t temp_sensor_gearbox;
	uint32_t bms_discharge_limit;
} SensorValues;

typedef struct SensorValuesConvertedToPhysicalValues {
	// Pakker i 32 bit av gangen ps
	uint16_t torque_encoder_ch0;
	uint16_t torque_encoder_ch1;
	
	uint16_t brake_pressure_fr;
	uint16_t brake_pressure_fl;
	
	int16_t steering_enc_data;
	
	float cooling_temperature; // dele på 100
	float gearbox_temperature; // dele på 100
	float motor_2_temperature;

	float Inverter_voltage;
	
	float BMS_max_temp;
	uint16_t BMS_max_temp_cell_id;
	float BMS_min_temp;
	uint16_t BMS_min_temp_cell_id;
	
	float GLVBMS_max_temp;
	uint16_t GLVBMS_max_temp_cell_id;
	float GLVBMS_min_temp;
	uint16_t GLVBMS_min_temp_cell_id;
	
	
	float battery_voltage;
	float current_counter;
	
	uint16_t min_cell_id;
	uint16_t max_cell_id;
	float max_cell_voltage;
	float min_cell_voltage;
	
	float GLV_voltage;
	float GLV_voltage_max_cell;
	float GLV_voltage_min_cell;
	uint16_t GLV_max_cell_id;
	uint16_t GLV_min_cell_id;
	
	uint16_t GLV_battery_percent;
	uint16_t HV_battery_percent;
	
	float IMU_rot_x;
	float IMU_rot_y;
	float IMU_rot_z;
	
	float IMU_G_x;
	float IMU_G_y;
	float IMU_G_z;
	
	float IMU_pos_x;
	float IMU_pos_y;
	
	float IMU_vel_x;
	float IMU_vel_y;
	float IMU_vel_z;
	

	} SensorPhysicalValues;


typedef struct MenuStructure {
	const char *text;
	uint8_t num_menupoints;
	uint8_t up;
	uint8_t down;
	uint8_t left;
	uint8_t right;
	uint8_t push_button;
	uint8_t position;
	EMenuName current_menu;
	EAdjustmentParameter current_setting;
	void (*rotaryActionFunc) (ERotary_direction rot, ParameterValue *parameter);
	void (*dataloggerFunc)(void);
	} MenuEntry;


/*declaring/defining variables that are used in dashboard.c, drawFunctions.c , buttonTask.c */
// Semaphore to protect the shared resource button struct
// between dashTask and buttonTask
extern SemaphoreHandle_t xButtonStruct;
extern SemaphoreHandle_t can_mutex_0;
extern SemaphoreHandle_t can_mutex_1;
Buttons btn;
DeviceState deviceState;

extern uint8_t selected;


void dashTask();

#endif 