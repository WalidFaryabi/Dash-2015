#ifndef DASHBOARDMENU_H_
#define DASHBOARDMENU_H_

#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/semphr.h"
#include "sam4e-base/FreeRTOS/Source/include/timers.h"

#include "DriversNotInBase/spi.h"

#include <stdbool.h>

typedef enum {MAIN_SCREEN,SYSTEM_MONITOR,TEMP_VOLT,MAIN_MENU,VARIABLE,DEVICE_STATUS,
			  SPEED,TRQ_CALIB,PERSISTENT_MSG,ECU_OPTIONS,LC_HANDLER,ERROR_HANDLER,SNAKE_GAME,
			  STEER_CALIB,DL_OPTIONS} EMenuName;
typedef enum {NO_SETTING,TORQUE_SETTING,KERS_SETTING, ECU_P_SETTING, ECU_D_SETTING, 
			  ECU_I_SETTING, ECU_LC_RT_SETTING, ECU_LC_INIT_TORQ_SETTING,DL_PREALLOCATE} EAdjustmentParameter;
			  
typedef enum {NAVIGATION,PUSH_ACK,ROTARY,SYS_ACK,START,LAUNCH_CONTROL,NONE_BTN} EButtonType;
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
	ESteerConfStates conf_steer;
	ETrqConfStates conf_trq_ch0;
	ETrqConfStates conf_trq_ch1;
	
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
	bool ams_error; // Assume there will be sent msgs for NOTOK and OK
	bool imd_error; // Assume there will be sent msgs for NOTOK and OK
	
	uint8_t ecu_error; // Store only one error ID at a time
	uint8_t bms_fault; // Store only one error ID at a time
	uint8_t bms_warning;
	} ModuleError;
	

typedef struct StatusMessagesCan {
	bool shut_down_circuit_closed;
	} StatusMsg;

typedef struct VariableValues { // Values of adjustable variables
	uint32_t min_torque;
	uint32_t torque;
	uint32_t prev_confirmed_torque;
	uint32_t max_torque;
	
	uint32_t min_P_term;
	uint32_t P_term;
	uint32_t prev_confirmed_P_term;
	uint32_t max_P_term;
	
	uint32_t min_I_term;
	uint32_t I_term;
	uint32_t prev_confirmed_I_term;
	uint32_t max_I_term;
	
	uint32_t min_D_term;
	uint32_t D_term;
	uint32_t prev_confirmed_D_term;
	uint32_t max_D_term;
	
	uint32_t min_R_term;
	uint32_t R_term;
	uint32_t prev_confirmed_R_term;
	uint32_t max_R_term;
	
	uint32_t min_T_term;
	uint32_t T_term;
	uint32_t prev_confirmed_T_term;
	uint32_t max_T_term;
	/* If acknowledge is pressed while adjusting a variable, a timer is started, if a confirmation msg is received 
	before it times out the variable of the current variable adjustment menu is confirmed. This is handled in
	getDashMessages function. If the user presses left the previous confirmed variable will be displayed again*/
	
	} Variables;
	
typedef struct SensorValuesReceivedOverCan {
	uint16_t brake_pressure_fr;
	uint16_t brake_pressure_fl;
	uint16_t temp_sensor_cooling;
	uint16_t temp_sensor_gearbox;
	
	uint32_t bms_discharge_limit;
} SensorValues;

typedef struct SensorRealValues {
	uint8_t torque_encoder_ch0;
	uint8_t torque_encoder_ch1;
	uint8_t brake_pressure_rear;
	uint8_t brake_pressure_front;
	uint8_t steering_angle;
	
	int32_t min_battery_temperature;
	int32_t battery_temperature;
	int32_t max_battery_temperature;
	int32_t motor_temperature;
	int32_t IGBT_temperature;
	int32_t gearbox_temperature;
	uint32_t battery_voltage;
	
	
	uint8_t max_battery_temperature_lsb;
	uint8_t max_battery_temperature_msb;
	
	uint8_t min_battery_temperature_lsb;
	uint8_t min_battery_temperature_msb;
	
	uint8_t min_cell_id;
	uint8_t min_cell_voltage_msb;
	uint8_t min_cell_voltage_lsb;
	
	uint8_t max_cell_id;
	uint8_t max_cell_voltage_msb;
	uint8_t max_cell_voltage_lsb;
	
	uint8_t GLV_voltage_msb;
	uint8_t GLV_voltage_lsb;
	
	uint8_t car_speed;
	uint8_t bms_discharge_limit;
	uint8_t brake_pedal_actuation;
	} SensorRealValue;


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
	void (*rotaryActionFunc) (ERotary_direction rot, Variables *var);
	void (*dataloggerFunc)(void);
	} MenuEntry;


/*declaring/defining variables that are used in dashboard.c, drawFunctions.c , buttonTask.c */
// Semaphore to protect the shared resource button struct
// between dashTask and buttonTask
extern SemaphoreHandle_t xButtonStruct;
extern SemaphoreHandle_t can_mutex_0;
extern SemaphoreHandle_t can_mutex_1;
Buttons btn;
DeviceState device_state;

extern uint8_t selected;


void dashTask();

#endif 