#include "Task_Menu.h"
#include "error_names.h"

#include "DriversNotInBase/revolve_logo.h"
#include "DriversNotInBase/high_voltage_symbol.h"
#include "DriversNotInBase/revolve_can_definitions.h"
#include "DriversNotInBase/IO_DashInit.h"

#include "Task_DataLogger.h"
#include "CanMessages.h"
#include "canID_definitions.h"
#include "CanHandler.h"
#include "sam4e-base/RevolveDrivers/can.h" // Need access to can struct and can functions

#include "snakeGame.h"
#include "DriversNotInBase/FT800.h"

#include <string.h>

//VANDRING GASSPEDAL

typedef enum {PEDAL_IN, PEDAL_OUT} EPedalPosition;
//**********************************************************************************//
//------------------------------------THE MAIN DASH FUNCTIONS-----------------------//
//**********************************************************************************//
static void changeCarState(ConfirmationMsgs *conf_msgs, StatusMsg *status, SensorRealValue *sensor_real);
static void dashboardControlFunction(Buttons *btn, ModuleError *error, SensorValues *sensor_values,StatusMsg *status,
ConfirmationMsgs *conf_msgs,DeviceState *device_state, Variables *var, SensorRealValue *sensor_real_value);
static void HandleButtonActions(Buttons *btn, SensorRealValue *sensor_real,DeviceState *device_state,
Variables *var, ModuleError *error,ConfirmationMsgs *conf_msgs);
static void NavigateMenu(DeviceState *device_state, Variables *var, ModuleError *error, SensorRealValue *sensor_real);

static void getDashMessages(Variables *var, ConfirmationMsgs *conf_msg, ModuleError *error, SensorValues *sensor_values,StatusMsg *status,SensorRealValue *sensor_real);

static void LEDHandler(SensorRealValue *sensor_real_value, ModuleError *error,DeviceState *devices);
//***********************************************************************************
//------------------------------------MENU HELPER FUNCTIONS------------------------//
//***********************************************************************************
static void can_freeRTOSSendMessage(Can *can,struct CanMessage message);

static void setVariableBasedOnConfirmation(Variables *var);
static void clearAllButtons();

static bool checkForError(ModuleError *error);
static void HandleErrors(ModuleError *error);

static EPedalPosition getTorquePedalPosition(SensorRealValue *sensor_real);
static EPedalPosition getBrakePedalPosition(SensorRealValue *sensor_real);
static bool bmsNotCharged(SensorRealValue *sensor_real);

static void calibrateSteering(ConfirmationMsgs *conf_msgs,bool ack_pressed);
static void calibrateTorquePedal(ConfirmationMsgs *conf_msgs,bool ack_pressed);
static bool checkDeviceStatus(DeviceState *devices);

static void presetProcedureHandling(bool ackPressed,ConfirmationMsgs *conf_msgs);
//---------------INIT THE STRUCTURES USED---------------//
static void init_sensorRealValue_struct(SensorRealValue *sensorReal);
static void init_sensor_value_struct(SensorValues *sensor_values);
static void init_status_struct(StatusMsg *status);
static void init_conf_msgs_struct(ConfirmationMsgs *conf_msgs);
static void init_error_struct(ModuleError *error);
static void init_variable_struct(Variables *var);
//--------------FUNCTIONS TO ADJUST VARIABLES---------------//
static void adjustParameters(ERotary_direction dir, Variables *var);
//Adjustparameters will replace these functions
static void slider_torque_update(ERotary_direction dir, Variables *var);
static void slider_P_term_update(ERotary_direction dir, Variables *var);
static void slider_I_term_update(ERotary_direction dir, Variables *var);
static void slider_D_term_update(ERotary_direction dir, Variables *var);
static void slider_R_term_update(ERotary_direction dir, Variables *var);
static void slider_T_term_update(ERotary_direction dir, Variables *var);

//-------------------TIMERS------------------//
static void vRTDSCallback(TimerHandle_t xTimer);
static void vCalibrationTimerCallback(TimerHandle_t xTimer);
static void vLcTimerCallback (TimerHandle_t lcTimer);
static void vVarConfTimerCallback(TimerHandle_t xTimer);
static void vMenuUpdateCallback(TimerHandle_t pxTimer);
static void createAndStartMenuUpdateTimers();
static void vTSLedTimerCallback(TimerHandle_t pxtimer);

//***********************************************************************************
//------------------------------------CALCULATION FUNCTIONS-------------------------//
//***********************************************************************************
static void sensorValueToRealValue(SensorValues *sensor_values,SensorRealValue *sensor_real);

//***********************************************************************************
//------------------------------------DRAWING FUNCTIONS----------------------------//
//***********************************************************************************
static void DrawMainScreen(SensorRealValue *sensor,uint8_t low_volt, uint8_t high_volt, DeviceState *devices);
static void DrawLowVoltageBattery(uint8_t battery_left_percent);
static void DrawHighVoltageBattery(uint8_t battery_left_percent);
static void DrawHighVoltageSymbol();

static void DrawSpeedScreen(SensorRealValue *sensor_real);
static void DrawSystemMonitorScreen(ModuleError *error,SensorRealValue *val);
static void DrawTempAndVoltScreen(SensorRealValue *tempvolt);

static void DrawMainMenu();
static void DrawAdjustmentMenu();
static void DrawECUAdjustmentScreen(Variables *var);
static void DrawDeviceStatusMenu(DeviceState *device_state);

static void DrawTorqueCalibrationScreen(ConfirmationMsgs *conf_msg);
static void DrawSteerCalibScreen();
static void DrawDriveEnableWarning(bool torque_pedal, bool brake_pedal, bool bms_discharge);
static void DrawLaunchControlProcedure();
static void DrawDataloggerInterface();

static void DrawFloat(uint16_t x, uint16_t y, uint8_t font_size, float f);
static void DrawPresetProcedure();

//***********************************************************************************
//------------------------------------DATALOGGER FUNCTIONS-------------------------//
//***********************************************************************************
static void createFileCommand();
static void startLoggingCommand();
static void closeFileCommand();
static void deleteAllFilesCommand();
static void slider_preallocateAmount();



//***********************************************************************************
//----------------------------------------MENU-------------------------------------//
//***********************************************************************************
const char menu_000[] = "Main";									// 0
const char menu_001[] = "Speed";								// 1
const char menu_002[] = "SYSTEM MONITOR";						// 2
const char menu_003[] = "TEMPERATURES AND VOLTAGES";			// 3

const char menu_200[] = "MAIN MENU";							// 4
const char menu_201[] = "DEVICE STATUS";						// 5
const char menu_202[] = "STEERING CALIBRATION";					// 6
const char menu_203[] = "TORQUE CALIBRATION";					// 7
const char menu_204[] = "ECU SETTINGS";							// 8
const char menu_205[] = "IMU SETTINGS";							// 9
const char menu_206[] = "SNAKE";								// 10
const char menu_207[] = "DATALOGGER";							// 11
const char menu_208[] = "PRESETS";								// 12

const char menu_300[] = "DEVICE STATUS";						// 13
const char menu_400[] = "KERS settings";						// 14
const char menu_500[] = "Trq. Enc. Calibration";				// 15

const char menu_600[] = "ECU OPTIONS";							// 16			
const char menu_601[] = "MAX TORQUE";							// 17
const char menu_602[] = "P TERM";								// 18
const char menu_603[] = "D TERM";								// 19
const char menu_604[] = "I TERM";								// 20

const char menu_605[] = "ECU T";								// 21
const char menu_606[] = "ECU R";								// 22

const char menu_900[] = "Drive enable message";					// 23
const char menu_901[] = "Launch control element";				// 24
const char menu_902[] = "Error element";						// 25

const char menu_700[] = "START";								// 26
const char menu_701[] = "CLOSE FILE";							// 27
const char menu_702[] = "DELETE FILES";							// 28
const char menu_703[] = "PREALLOCATE";							// 29

uint8_t selected = 0; // Menu Index
const MenuEntry menu[] = {
	// text  num,   U   D   L   R  Push Pos  cur_menu			current_setting				Rotaryfunction dataloggerFunc
	{menu_000, 1,	0,	1,  2,  5,	0,  0,	MAIN_SCREEN,		NO_SETTING,					0,0 },						//0
		
	{menu_001, 1,	0,	1,	1,	1,	1,  0,  SPEED,				NO_SETTING,					0,0 },						//1	
	{menu_002, 1,	2,	2,	3,	0,	2,  1,  SYSTEM_MONITOR,		NO_SETTING,					0,0 },						//2
	{menu_003, 1,	3,	3,	3,	2,	3,  1,	TEMP_VOLT,			NO_SETTING,					0,0 },						//3
			
	{menu_200, 9,	4,	4,	4,	4,	4,  0,	MAIN_MENU,			NO_SETTING,					0,0},						//4  main menu
	{menu_201, 9,	5,	6,	0,	9,  14,	1,  MAIN_MENU,			NO_SETTING,					0,0},						//5	 Device Status  
	{menu_202, 9,	5,	7,	0,	10, 19,	2,  MAIN_MENU,			NO_SETTING,					0,0},						//6  Steer calib
	{menu_203, 9,	6,	8,	0,	11, 20,	3,  MAIN_MENU,			NO_SETTING,					0,0},						//7	 Torque Pedal Calibration
	{menu_204, 9,	7,	9,	0,	12, 21,	4,  MAIN_MENU,			NO_SETTING,					0,0},						//8  ECU Options
	{menu_205, 9,	8,	10,	5,	9,  9,	5,  MAIN_MENU,			NO_SETTING,					0,0},						//9  IMU options
	{menu_206, 9,	9,	11,	6,	10, 18,	6,  MAIN_MENU,			NO_SETTING,					0,0},						//10 Snake
	{menu_207, 9,	10,	12,	7,	11, 25,	7,  MAIN_MENU,			NO_SETTING,					0,0},						//11 Datalogger
	{menu_208, 9,	11,	12,	8,	12, 8,	8,  MAIN_MENU,			NO_SETTING,					0,0},						//12 Preset parameters
	
	{menu_400, 1,	10,	10,	6,	10, 10,	1,  VARIABLE,			KERS_SETTING,				0,0},						//13  KERS adjustment screen
		
	{menu_300, 1,	14,	14,	5,	14, 14,	1,  DEVICE_STATUS,		NO_SETTING,					0,0},						//14  Device status Screen
		
	{menu_900, 1,	15,	15,	15,	15,  15, 0, PERSISTENT_MSG,		NO_SETTING,					0,0},						//15 Drive enable message
	{menu_901, 1,	16,	16,	16,	16,  16, 0, LC_HANDLER,			NO_SETTING,					0,0},						//16 LC handler
	{menu_902, 1,	17,	17,	17,	17,  17, 0, ERROR_HANDLER,		NO_SETTING,					0,0},						//17 Error handler
	{"snake",  1,	18,	18,	18,	18,  18, 0, SNAKE_GAME,			NO_SETTING,					0,0},						//18 Play snake
	{"SteerCal",1,	19,	19,	19,	19,  19, 0, STEER_CALIB,		NO_SETTING,					0,0},						//19 Steer calib
	{menu_500, 1,	20,	20,	20,	20,	 20, 0, TRQ_CALIB,			NO_SETTING,					0,0},						//20 Torque calibration screen
		
	{menu_601, 4,	21,	22,	8,	21, 21,	0,  ECU_OPTIONS,		TORQUE_SETTING,				slider_torque_update,0},	//21 Max torque slider
	{menu_602, 4,	21,	23,	8,	22, 22,	1,  ECU_OPTIONS,		ECU_P_SETTING,				slider_P_term_update,0},	//22 ECU P slider
	{menu_603, 4,	22,	24,	8,	23, 23,	2,  ECU_OPTIONS,		ECU_D_SETTING,				slider_D_term_update,0},	//23 ECU D slider
	{menu_604, 4,	23,	24,	8,	24, 24,	3,  ECU_OPTIONS,		ECU_I_SETTING,				slider_I_term_update,0},	//24 ECU I slider
	
	{menu_700, 3,	25,	26,	11,	25, 25,	0,  DL_OPTIONS,			NO_SETTING,				0,startLoggingCommand},			//25 Create file
	{menu_701, 3,	25,	27,	11,	26, 26,	1,  DL_OPTIONS,			NO_SETTING,				0,closeFileCommand},			//26 Start logging
	{menu_702, 3,	26,	27, 11,	27, 27,	2,  DL_OPTIONS,			NO_SETTING,				0,deleteAllFilesCommand},		//27 Close file
		
	{"LOCKED", 1,	28,	28, 28,	28, 28,	0,  LOCKED_SEL,			NO_SETTING,				0,0},							//28 Locked menu position
		
	{"VERYWET10", 8,29,	30, 12,	33, 37,	0,  PRESET_SEL,			PRESET_1_SETTING,		0,0},							//29 Preset option
	{"VERYWET20", 8,29,	31, 12,	34, 37,	1,  PRESET_SEL,			PRESET_2_SETTING,		0,0},							//30 Preset option
	{"WET10",	  8,30,	32, 12,	35, 37,	2,  PRESET_SEL,			PRESET_3_SETTING,		0,0},							//31 Preset option
	{"VWET20",	  8,31,	33, 12,	36, 37,	3,  PRESET_SEL,			PRESET_4_SETTING,		0,0},							//32 Preset option
	{"DRY10",     8,32,	34, 29,	33, 37,	4,  PRESET_SEL,			PRESET_5_SETTING,		0,0},							//33 Preset option
	{"DRY20",     8,33,	35, 30,	34, 37,	5,  PRESET_SEL,			PRESET_6_SETTING,		0,0},							//34 Preset option
	{"DRY25",     8,34,	36, 31,	35, 37,	6,	PRESET_SEL,			PRESET_7_SETTING,		0,0},							//35 Preset option
	{"GEN",       8,35,	36, 32,	36, 37,	7,	PRESET_SEL,			PRESET_8_SETTING,		0,0},							//36 Preset option
	
	{"PRELOCK",   1,37,	37, 37,	37, 37,	0,	PRESET_PROCEDURE,	NO_SETTING,				0,0}							//37 Preset option
		
	
	//{menu_703, 4,	27,	28,	11,	28, 28,	3,  DL_OPTIONS,			NO_SETTING,				0,deleteAllFilesCommand}		//28 Delete all files
	//{menu_704, 4,	23,	24,	8,	24, 24,	3,  DL_OPTIONS,			DL_PREALLOCATE,			0,slider_preallocateAmount}	//29 Amount to preallocate
};



//********************************************************************//
//-----------------------------DEFINES--------------------------------//
//********************************************************************//
// Define position of some menu elements to simplify programming
#define MAIN_MENU_POS				5
#define ECU_SETTINGS_MENU_POS		21
#define ECU_SETTINGS_VARIABLES_POS	21
#define DRIVE_ENABLE_WARNING_SEL	15
#define ERROR_HANDLER_POS			17
#define LC_HANDLER_POS				16
#define LOCKED_SEL_POS				28
#define PRESET_PROCEDURE_POS		37

#define NUM_MENUS_UPDATE 1 // Number of menus to specifiy a certain update frequency for
#define RTDS_DURATION_MS 3000
#define WATCHDOG_RESET_COMMAND  ( (0xA5 << 24) | (1<<0)) // Command to write to WDT CR register to reset the counter
//***********************************************************************************
//-------------------------SEMAPHORE, TIMERS AND QUEUES----------------------------//
//***********************************************************************************
SemaphoreHandle_t		xButtonStruct	= NULL;
SemaphoreHandle_t		spi_semaphore	= NULL;
SemaphoreHandle_t		can_mutex_0		= NULL;
SemaphoreHandle_t		can_mutex_1		= NULL;
static TimerHandle_t	TSLedTimer;
static TimerHandle_t	RTDSTimer;
static TimerHandle_t	LC_timer;
static TimerHandle_t	calibrationTimer;
static TimerHandle_t	parameterConfTimer;
static TimerHandle_t	timer_menuUpdate[NUM_MENUS_UPDATE];
static bool				trq_calib_timed_out				= false;
static bool				steer_calib_timed_out			= false;
static bool				parameter_confirmation_timed_out = false;
static uint8_t			lc_timer_count					= 0; // Countdown timer for launch control
//***********************************************************************************
//---------------------------FILE GLOBAL STATE VARIABLES------------------------------//
//***********************************************************************************
static ECarState					car_state			= TRACTIVE_SYSTEM_OFF;
static ESteerCalibState				steer_calib_state	= STEER_C_OFF;
static ETorquePedalCalibrationState trq_calib_state		= TRQ_CALIBRATION_OFF;
static EPresetStates				presetProcedureState= PRESET_PROCEDURE_OFF;

//***********************************************************************************
//------------------------------------GLOBAL STRUCTS-------------------------------//
//***********************************************************************************
typedef struct menuUpdateFrequency { // Private global for this source
	bool update_menu;
} menuUpdateStatus;
menuUpdateStatus menuUpdate = {
	.update_menu = false
};
Buttons btn = {
	.btn_type				= NONE_BTN,
	.navigation				= NAV_DEFAULT,
	.rotary_ccw				= false,
	.rotary_cw				= false,
	.unhandledButtonAction  = false
};
DeviceState device_state = {
	.ECU		= DEAD,
	.TRQ_0		= DEAD,
	.TRQ_1		= DEAD,
	.BSPD		= DEAD,
	.TEL		= DEAD,
	.ADC_FR		= DEAD,
	.ADC_FL		= DEAD,
	.ADC_RR		= DEAD,
	.ADC_RL		= DEAD,
	.INV		= DEAD,
	.FAN		= DEAD,
	.BMS		= DEAD,
	.GLVBMS		= DEAD,
	.IMU		= DEAD,
	.STEER_POS	= DEAD,
	.IMD		= DEAD
};
typedef enum {STEP_POINT_ONE, STEP_ONE, STEP_TEN}EStepSize ;
static struct StepSizeForVariables {
	EStepSize p_term;
	EStepSize i_term;
	EStepSize d_term;
	EStepSize torque;
	};
static struct StepSizeForVariables StepSizeVar = {
	.p_term = STEP_ONE,
	.i_term = STEP_ONE,
	.d_term = STEP_ONE,
	.torque = STEP_ONE
	};
	
static struct presetParameterStruct {
	float p_term;
	float i_term;
	float a;
	float b;
};
static struct presetParameterStruct presetParameters;

// stuff
EAdjustmentParameter presetSetting;

//***********************************************************************************
//----------------------------------THRESHOLDS AND CRITICAL VALUES-----------------//
//***********************************************************************************
#define TORQUE_PEDAL_THRESHOLD	25 
#define BRAKE_PEDAL_THRESHOLD	4500
#define BATTERY_TEMP_CRITICAL_HIGH 90

//***********************************************************************************
//--------------------------------FILE GLOBALS-------------------------------------//
//***********************************************************************************
static bool RTDS_finished_playing = false;
static uint8_t prev_selected = 0;
//***********************************************************************************
//------------------------------------THE MAIN DASH FUNCTIONS-----------------------//
//***********************************************************************************
// Find out if all sensorhandling should be put in a task so that the dash task can be run with a slower frequency
void dashTask() {
	TickType_t xLastWakeTime;
	RTDSTimer				= xTimerCreate("RTDSTimer",RTDS_DURATION_MS/portTICK_RATE_MS,pdFALSE,0,vRTDSCallback);
	LC_timer				= xTimerCreate("lcTimer",1000/portTICK_RATE_MS,pdTRUE,(void *) 1,vLcTimerCallback);
	calibrationTimer		= xTimerCreate("CalibTimer",3000/portTICK_RATE_MS,pdFALSE,(void *) 1,vCalibrationTimerCallback);
	parameterConfTimer		= xTimerCreate("varTimer",1000/portTICK_RATE_MS,pdFALSE,0,vVarConfTimerCallback);
	TSLedTimer				= xTimerCreate("TSLed", 300/portTICK_RATE_MS,pdTRUE,0,vTSLedTimerCallback);
	createAndStartMenuUpdateTimers();
	
	xPresetQueue			= xQueueCreate(1,sizeof(presetParameters));
	
	//Init states
	SensorValues		sensor_values;
	SensorRealValue		sensor_real;
	StatusMsg			status;
	ConfirmationMsgs	conf_msgs;
	ModuleError			error;
	Variables			var;

	init_sensorRealValue_struct(&sensor_real);
	init_sensor_value_struct(&sensor_values);
	init_status_struct(&status);
	init_conf_msgs_struct(&conf_msgs);
	init_error_struct(&error);
	init_variable_struct(&var);
	//ECarState car_state = TRACTIVE_SYSTEM_OFF;
	
	//Start FT800 in a clean way
	vTaskDelay(20/portTICK_RATE_MS);
	//delay_ms(50);
	pio_setOutput(FT800_POWERDOWN_PIO,FT800_POWERDOWN_PIN,PIN_LOW);
	vTaskDelay(200/portTICK_RATE_MS);
	//delay_ms(200);
	pio_setOutput(FT800_POWERDOWN_PIO,FT800_POWERDOWN_PIN,PIN_HIGH);
	vTaskDelay(20/portTICK_RATE_MS);
	//delay_ms(50);
	xSemaphoreTake(spi_handlerIsDoneSempahore,0);
	FT800_Init();
	//Need to delay 50 ms after the init of ft800 before any transfers to it happen
	vTaskDelay(50/portTICK_RATE_MS);
	spi_setBaudRateHz(120000000,20000000,0); // Increase speed after init
	//Upload the highvoltage icon to the FT800 
	/*uint32_t ram_offset=0;
	for(int i=0; i<4130; i++){
		wr16(ram_offset, high_voltage_symbol[i]);
		ram_offset +=2;
	}*/
	
	while(1) {
		// The delay for this task is given by the wait time specified in the queuereceive function
		// in the getdashmessages function. This is done to ensure that as long as there are 
		// messages in the queue the menu task will service them. The update frequency of the visuals 
		// is controlled by software timers to ensure that no processing power is wasted on spi comms.
		WDT->WDT_CR = WATCHDOG_RESET_COMMAND; // Restart watchdog timer
		//Get relevant CAN messages from the specified freeRTOS queue
		getDashMessages(&var,&conf_msgs,&error,&sensor_values, &status,&sensor_real);
		xSemaphoreTake(xButtonStruct, portMAX_DELAY);
		dashboardControlFunction(&btn,&error,&sensor_values,&status,&conf_msgs, &device_state,&var,&sensor_real);
		xSemaphoreGive(xButtonStruct);
		//vTaskDelay(35/portTICK_RATE_MS);
		//vTaskDelayUntil(&xLastWakeTime,150/portTICK_RATE_MS);
	}
}

static void dashboardControlFunction(Buttons *btn, ModuleError *error, SensorValues *sensor_values, 
	StatusMsg *status, ConfirmationMsgs *conf_msgs,DeviceState *device_state,Variables *var, SensorRealValue *sensor_real_value) {
		
	changeCarState(conf_msgs, status, sensor_real_value);
	LEDHandler(sensor_real_value,error,device_state);
	if (checkForError(error)) {
		HandleErrors(error);
	}
	
	if (RTDS_finished_playing) {
		can_freeRTOSSendMessage(CAN0, FinishedRTDS);
		RTDS_finished_playing = false;
	}
	
	if (btn->unhandledButtonAction) {
		HandleButtonActions(btn,sensor_real_value, device_state, var,error,conf_msgs);
	}
	
	//UPDATE SCREENS ON THE DISPLAY AND CALL MENU LOCATION DEPENDENT FUNCTIONS
	switch (menu[selected].current_menu) {
		case ECU_OPTIONS:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawECUAdjustmentScreen(var);
		}
		
		break;
		
		case SYSTEM_MONITOR:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawSystemMonitorScreen(error, sensor_real_value);
		}
		
		break;
		
		case TEMP_VOLT:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawTempAndVoltScreen(sensor_real_value);
		}
		
		break;

		case DEVICE_STATUS:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawDeviceStatusMenu(device_state);
		}
		
		break;
		case MAIN_SCREEN:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawMainScreen(sensor_real_value,50,10, device_state);
		}
		break;
		case MAIN_MENU:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawMainMenu();
		}
		
		break;
		case DL_OPTIONS:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawDataloggerInterface();
		}
		
		break;
		
		case TRQ_CALIB:
		calibrateTorquePedal(conf_msgs,false);
		break;
		case STEER_CALIB:
		calibrateSteering(conf_msgs,false);
		break;
		case SNAKE_GAME:
		if (snakeGameState == SNAKE_OFF) {
			snakeGameState = SNAKE_PREPPING;
			snakeControlFunction(false,UP);
		}
		else {
			snakeControlFunction(false,UP);
		}
		break;
		case PRESET_PROCEDURE:
			if (presetProcedureState == PRESET_PROCEDURE_OFF) {
				presetProcedureState = PRESET_PROCEDURE_INIT;
				presetProcedureHandling(false,conf_msgs);
			}
			else {
				presetProcedureHandling(false,conf_msgs);
			}
		break;
	}
}


static void presetProcedureHandling(bool ackPressed, ConfirmationMsgs *conf_msgs) {
	static enum EDataloggerCommands close_file = CLOSE_FILE;
	static enum EDataloggerCommands get_files = GET_PARAMETERS_FROM_FILE;
	switch (presetProcedureState) {
		case PRESET_PROCEDURE_INIT:
			// Stop and close file for datalogger
			// Send command to extract data from file 
			xQueueSendToBack(xDataloggerCommandQueue,&close_file,10);
			xQueueSendToBack(xDataloggerCommandQueue,&get_files,10);
			presetProcedureState = PRESET_PROCEDURE_WAITING;
		break;
		case PRESET_PROCEDURE_WAITING:
			if (xQueueReceive(xPresetQueue,&presetParameters, 1000) == pdPASS) {
				// Received all the preset parameters from the datalogger task
				presetProcedureState = PRESET_PROCEDURE_SEND_P_TERM;
				// Send the struct to ECU one by one.. wait for confirmation for each
			}
			else {
				// Timed out
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		case PRESET_PROCEDURE_SEND_P_TERM:
			EcuPTerm.data.f[0] = presetParameters.p_term;
			can_freeRTOSSendMessage(CAN0, EcuPTerm);
			xTimerReset(parameterConfTimer,0);
			parameter_confirmation_timed_out = false;
		break;
		case WAIT_P_TERM:
			if (conf_msgs->ECU_parameter_confirmed == true) {
				conf_msgs->ECU_parameter_confirmed = false;
				presetProcedureState = PRESET_PROCEDURE_SEND_I_TERM;
			}
			else if (parameter_confirmation_timed_out == true) {
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		case PRESET_PROCEDURE_SEND_I_TERM:
			EcuITerm.data.f[0] = presetParameters.i_term;
			can_freeRTOSSendMessage(CAN0, EcuITerm);
			xTimerReset(parameterConfTimer,0);
			parameter_confirmation_timed_out = false;
		break;
		case WAIT_I_TERM:
			if (conf_msgs->ECU_parameter_confirmed == true) {
				conf_msgs->ECU_parameter_confirmed = false;
				presetProcedureState = PRESET_PROCEDURE_FINISHED;
			}
			else if (parameter_confirmation_timed_out == true) {
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		
		case PRESET_PROCEDURE_FINISHED:
			DrawPresetProcedure();
			if (ackPressed == true) {
				selected = MAIN_MENU_POS;
				presetProcedureState = PRESET_PROCEDURE_OFF;
			}
		// Tell the user that the preset is loaded to ecu
		// Wait for acknowledge to send user back to main menu
		break;
		
		case PRESET_PROCEDURE_FAILED:
			DrawPresetProcedure();
			if (ackPressed == true) {
				selected = MAIN_MENU_POS;
				presetProcedureState = PRESET_PROCEDURE_OFF;
			}
		break;
	}
}

static void changeCarState(ConfirmationMsgs *conf_msgs, StatusMsg *status, SensorRealValue *sensor_real ) {
	switch(car_state) {
		case TRACTIVE_SYSTEM_OFF:
			if (status->shut_down_circuit_closed == true) {
				car_state = TRACTIVE_SYSTEM_ON;
			}
			break;
		case TRACTIVE_SYSTEM_ON:
			if (status->shut_down_circuit_closed == false ) {
				car_state = TRACTIVE_SYSTEM_OFF;
				
			}
			else if (conf_msgs->drive_enabled_confirmed == true) {
				car_state = DRIVE_ENABLED;
				conf_msgs->drive_enabled_confirmed = false;
			}
			break;
		case DRIVE_ENABLED:
			if (status->shut_down_circuit_closed == false) {
				can_sendMessage(CAN0,TorquePedalCalibrationMax);
				car_state = TRACTIVE_SYSTEM_OFF;
			}
			else if (conf_msgs->drive_disabled_confirmed == true) {
				car_state = TRACTIVE_SYSTEM_ON;
				can_sendMessage(CAN0,FinishedRTDS);
				conf_msgs->drive_disabled_confirmed = false;
			}
			else if (conf_msgs->lc_request_confirmed == true) {
				car_state = LC_PROCEDURE;
				// Go to the menu element for launch control. This is done so that handlebuttons etc can run without any problems
				selected = LC_HANDLER_POS; 
				conf_msgs->lc_request_confirmed = false;
			}
			
			break;
		case LC_PROCEDURE:
			DrawLaunchControlProcedure();
			//if ( (getTorquePedalPosition(sensor_real) == PEDAL_IN) && (getBrakePedalPosition(sensor_real) == PEDAL_IN) ) {
			if ( btn.btn_type == PUSH_ACK) {
				car_state = LC_STANDBY;
				
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
			//}
			//else if (getTorquePedalPosition(sensor_real) == false) {
			//	car_state = DRIVE_ENABLED;
			//}
			else if (status->shut_down_circuit_closed == false) {
				car_state = TRACTIVE_SYSTEM_OFF;
			}
			else if (conf_msgs->drive_disabled_confirmed == true) {
				can_sendMessage(CAN0,EcuParameters);
				car_state = TRACTIVE_SYSTEM_ON;
				conf_msgs->drive_disabled_confirmed = false;
			}
			else if (btn.btn_type == LAUNCH_CONTROL) {
				car_state = LC_ABORTED;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
			break;
		
		case LC_STANDBY:
			
			if ( (getBrakePedalPosition(sensor_real) == PEDAL_OUT) && (getTorquePedalPosition(sensor_real) == PEDAL_IN) ) {
				car_state = LC_COUNTDOWN;
				xTimerStart(LC_timer,1000/portTICK_RATE_MS);
				lc_timer_count = 0;
			}
			else if (getTorquePedalPosition(sensor_real) == PEDAL_OUT) {
				car_state = LC_ABORTED;
			}
			else if (btn.btn_type == LAUNCH_CONTROL) {
				car_state = LC_ABORTED;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
			break;
		case LC_COUNTDOWN:
			DrawLaunchControlProcedure();
			//if (getBrakePedalPosition(sensor_real) == PEDAL_OUT) && (getTorquePedalPosition(sensor_real) == PEDAL_IN) {
				if (lc_timer_count == 5) {
					car_state = LC_WAITING_FOR_ECU_TO_ARM_LC;
					xTimerReset(LC_timer,5/portTICK_RATE_MS);
					lc_timer_count = 0;
					//Send can message that countdown is finished
					can_freeRTOSSendMessage(CAN0,RequestLCArmed);
				}
			//}
			/*else {
				xTimerStop(LC_timer,5/portTICK_RATE_MS);
				lc_timer_count = 0;
				car_state = LC_ABORTED;
			}*/
			break;		
		case LC_WAITING_FOR_ECU_TO_ARM_LC:
			if (conf_msgs->lc_ready == true) {
				conf_msgs->lc_ready = false;
				car_state = LC_ARMED;
				xTimerStop(LC_timer,0);
				lc_timer_count = 0;
			}
			else if (lc_timer_count > 2) {
				car_state = LC_ARMING_TIMED_OUT;
				lc_timer_count = 0;
				xTimerStop(LC_timer,0);
			}
			break;
		case LC_ARMED:
			DrawLaunchControlProcedure();
			//if (getTorquePedalPosition(sensor_real) == false) {
			//	car_state = DRIVE_ENABLED;
			//	selected = 0;
			//}
			break;
		case LC_ABORTED:
			DrawLaunchControlProcedure();
			if ( btn.btn_type == PUSH_ACK) {
				car_state = DRIVE_ENABLED;
				selected = 0;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
		break;
		case LC_ARMING_TIMED_OUT:
			DrawLaunchControlProcedure();
			if ( btn.btn_type == PUSH_ACK) {
				car_state = DRIVE_ENABLED;
				selected = 0;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
		break;
	}					
}

static void HandleButtonActions(Buttons *btn, SensorRealValue *sensor_real ,DeviceState *device_state, 
						 Variables *var, ModuleError *error, ConfirmationMsgs *conf_msgs) { 
	if (btn->btn_type == NAVIGATION) {
		NavigateMenu(device_state, var,error, sensor_real);
		
	}
	// If changing a variable and acknowledge button is pressed the selection will be confirmed,
	// the value sent by CAN.
	else if (btn->btn_type == PUSH_ACK) {
		// Check if there is an error first.. Since acknowledge button is used for both variables and errors / faults
		switch (menu[selected].current_menu) {
			
			case DL_OPTIONS:
				menu[selected].dataloggerFunc();
			break;
			case ERROR_HANDLER:
				//can_sendMessage(CAN0,ackMsg);
				selected = 0; // Return to main menu
			break;
			
			case PRESET_SEL:
				if (presetProcedureState != PRESET_PROCEDURE_OFF) {
					presetSetting = menu[selected].current_setting;
					presetProcedureHandling(true,conf_msgs);
				}
			break;
				
			case ECU_OPTIONS:
				switch (menu[selected].current_setting) {
					case TORQUE_SETTING:
						//EcuParameters.data = var->
						EcuParameters.data.u8[0] = CAN_SET_TORQUE;
						EcuParameters.dataLength = 2;
						if ( (var->torque <= 100) && (var->torque >= 0) ) {
							EcuParameters.data.u8[1] = var->torque;
							can_freeRTOSSendMessage(CAN0,EcuParameters);
						}
					break;
					case ECU_P_SETTING:
						EcuPTerm.data.f[0] = var->P_term;
						can_freeRTOSSendMessage(CAN0,EcuPTerm);
					break;
					
					case ECU_D_SETTING:
					
					break;				
					case ECU_I_SETTING:
						EcuITerm.data.f[0] = var->I_term;
						can_freeRTOSSendMessage(CAN0,EcuITerm);
					break;
					
				}
				prev_selected = selected;
				selected = LOCKED_SEL_POS;
				parameter_confirmation_timed_out = false;
				// The menu selection is set to a locked position, the menu will return to its position after 
				// the timer runs out or the variable is confirmed.
				xTimerReset(parameterConfTimer,20/portTICK_RATE_MS); //Start variable confirmation timer
			break;
			case PERSISTENT_MSG:
				//If a persistent msg is acknowledged the user is returned to the main screen
				selected = 0; //Return to main screen
				DrawMainScreen(sensor_real,20,20,device_state);
			break;
			case TRQ_CALIB:
				calibrateTorquePedal(conf_msgs,true);
			break;
			case STEER_CALIB:
				calibrateSteering(conf_msgs,true);
			break;
			
			case SNAKE_GAME:
				if (snakeGameState == SNAKE_OFF) {
					snakeGameState = SNAKE_PREPPING;
					snakeControlFunction(false,UP);
				}
			break;
		}
		// This is for choosing a selected menu
		selected = menu[selected].push_button;
	}
	
	else if (btn->btn_type == ROT_ACK) {
		switch (menu[selected].current_setting) {
			case TORQUE_SETTING:
				switch (StepSizeVar.torque){
					case STEP_POINT_ONE:
					StepSizeVar.torque = STEP_ONE;
					break;
					case ONE:
					StepSizeVar.torque = STEP_TEN;
					break;
					case STEP_TEN:
					StepSizeVar.torque = STEP_POINT_ONE;
					break;
				}
			case ECU_P_SETTING:
				switch (StepSizeVar.p_term) {
					case STEP_POINT_ONE:
						StepSizeVar.p_term = STEP_ONE;
						break;
					case ONE:
						StepSizeVar.p_term = STEP_TEN;
						break;
					case STEP_TEN:
						StepSizeVar.p_term = STEP_POINT_ONE;
						break;
				}
			break;
			case ECU_D_SETTING:
				switch (StepSizeVar.d_term) {
					case STEP_POINT_ONE:
					StepSizeVar.d_term = STEP_ONE;
					break;
					case ONE:
					StepSizeVar.d_term = STEP_TEN;
					break;
					case STEP_TEN:
					StepSizeVar.d_term = STEP_POINT_ONE;
					break;
				}
			break;
			case ECU_I_SETTING:
				switch (StepSizeVar.i_term) {
					case STEP_POINT_ONE:
					StepSizeVar.i_term = STEP_ONE;
					break;
					case ONE:
					StepSizeVar.i_term = STEP_TEN;
					break;
					case STEP_TEN:
					StepSizeVar.i_term = STEP_POINT_ONE;
					break;
				}
			break;
		}
	}
	else if (btn->btn_type == LAUNCH_CONTROL) {
		if (car_state == DRIVE_ENABLED) {
			//Request Launch control from ECU
			//Send CAN message
			can_freeRTOSSendMessage(CAN0,RequestLCInit);

		}
	}
	else if (btn->btn_type == ROTARY) {
		// If currently looking at a variable to adjust, call the specific function to adjust this value
		if (menu[selected].current_menu == ECU_OPTIONS){ //Add the different option names
			if (menu[selected].rotaryActionFunc != 0) {
				if (btn->rotary_cw == true) {
					menu[selected].rotaryActionFunc(CW,var);
				}
				else{
					menu[selected].rotaryActionFunc(CCW,var);
				}
			}
		}
		
	}
	
	
	else if (btn->btn_type == START) {
		if ( (car_state != TRACTIVE_SYSTEM_OFF) || (car_state != TRACTIVE_SYSTEM_ON) )  {
			// Request shut down
			can_freeRTOSSendMessage(CAN0, RequestDriveDisable);
		}
		else if (car_state == TRACTIVE_SYSTEM_ON) {
			//Request car start
			// If criterias satisfied
			can_freeRTOSSendMessage(CAN0, RequestDriveEnable);
		}
		
		/*if (btn->drive_switch_disable == true) {
			btn->drive_switch_disable = false;
			if (car_state == DRIVE_ENABLED) {
				// Send CAN message to ECU to disable drive
				 
				// Cofirmation is handled in control function
			}
		}
		else if (btn->drive_switch_enable == true) {
			btn->drive_switch_enable = false;
			bool drive_enable_criterias = true;
			bool torque_pedal_not_pressed = false;
			bool brake_pedal_not_pressed = false;
			bool bms_discharge = false;
			if (car_state == TRACTIVE_SYSTEM_ON) {
				if (getTorquePedalPosition(sensor_real) ) {
					drive_enable_criterias = false;
					torque_pedal_not_pressed = true;
				}
				if (getBrakePedalPosition(sensor_real)) {
					brake_pedal_not_pressed = true;
					drive_enable_criterias = false;
				}
				if (bmsNotCharged(sensor_real)) {
					bms_discharge = true;
					drive_enable_criterias = false;	
				}	
				if (drive_enable_criterias == true) {
					// Send CAN message to ECU to enable drive
					// Confirmation and state change handled in control function
				}
				else {
					// Setting selected to drive enable message menu element. The only way to remove the
					// warning is to acknowledge
					selected = DRIVE_ENABLE_WARNING_SEL;
					DrawDriveEnableWarning(torque_pedal_not_pressed,brake_pedal_not_pressed,bms_discharge);
				}
			}
		}*/
	}
	btn->rotary_ccw = false;
	btn->rotary_cw  = false;
	btn->navigation = NAV_DEFAULT;
	btn->unhandledButtonAction = false;
	btn->btn_type = NONE_BTN;
}

static void NavigateMenu(DeviceState *device_state, Variables *var, ModuleError *error, SensorRealValue *sensor_real) {
	
	switch (btn.navigation) {
		case UP:
			switch (menu[selected].current_menu) {
				case ECU_OPTIONS:
				parameter_confirmation_timed_out = true;
				setVariableBasedOnConfirmation(var);
				break;
				case SNAKE_GAME:
				if (snakeGameState != SNAKE_OFF) {
					snakeControlFunction(true,UP);
				}
				break;
			}
			selected = menu[selected].up;
			break;
		
		case DOWN:
			switch (menu[selected].current_menu) {
				case ECU_OPTIONS:
				parameter_confirmation_timed_out = true;
				setVariableBasedOnConfirmation(var);
				break;
				case SNAKE_GAME:
				if (snakeGameState != SNAKE_OFF) {
					snakeControlFunction(true,DOWN);
				}
				break;
			}
			selected = menu[selected].down;
			break;
		case LEFT:
			if (menu[selected].current_menu == SNAKE_GAME) {
				if (snakeGameState != SNAKE_OFF) {
					snakeControlFunction(true,LEFT);
				}
			}
			selected = menu[selected].left;
			break;
		
		case RIGHT:
			if (menu[selected].current_menu == SNAKE_GAME) {
				if (snakeGameState != SNAKE_OFF) {
					snakeControlFunction(true,RIGHT);
				}
			}
			selected = menu[selected].right;
			break;
	}

	switch (menu[selected].current_menu) {
		case MAIN_SCREEN:
		DrawMainScreen(sensor_real,20,20,device_state);
		break;
		case SPEED:
		DrawSpeedScreen(sensor_real);
		break;
		case SYSTEM_MONITOR:
		DrawSystemMonitorScreen(error,sensor_real);
		break;
		case TEMP_VOLT:
		DrawTempAndVoltScreen(sensor_real);
		break;
		case MAIN_MENU:
		DrawMainMenu();
		break;
		case DEVICE_STATUS:
		DrawDeviceStatusMenu(device_state);
		break;
		case ECU_OPTIONS:
		DrawECUAdjustmentScreen(var);
		break;
		case DL_OPTIONS:
		DrawDataloggerInterface();
		break;
		//case SNAKE_GAME:
// 		if (snakeGameState == SNAKE_OFF) {
// 			snakeGameState = SNAKE_PREPPING;
// 			snakeControlFunction(false,UP);
// 		}
// 		break;
	}
}

static void LEDHandler(SensorRealValue *sensor_real_value, ModuleError *error,DeviceState *devices) {
	if (error->ams_error == true) {
		pio_setOutput(AMS_LED_PIO,AMS_LED_PIN,PIN_HIGH);
	}
	else {
		pio_setOutput(AMS_LED_PIO,AMS_LED_PIN,PIN_LOW);
	}
	
	if (error->imd_error == true) {
		pio_setOutput(IMD_LED_PIO,IMD_LED_PIN,PIN_HIGH);
	}
	else {
		pio_setOutput(IMD_LED_PIO,IMD_LED_PIN,PIN_LOW);
	}
	if ( (sensor_real_value->battery_temperature > BATTERY_TEMP_CRITICAL_HIGH) ) {
		pio_setOutput(TEMP_LED_PIO,TEMP_LED_PIN,PIN_HIGH);
	}
	else {
		//set low
		pio_setOutput(TEMP_LED_PIO,TEMP_LED_PIN,PIN_LOW);
	}
	if (error->ecu_error != 0) {
		pio_setOutput(ECU_LED_PIO,ECU_LED_PIN,PIN_HIGH);
	}
	else {
		pio_setOutput(ECU_LED_PIO,ECU_LED_PIN,PIN_LOW);
	}
	if (checkDeviceStatus(devices) == false) {
		//Set ouput high
		pio_setOutput(DEVICE_LED_PIO,DEVICE_LED_PIN,PIN_HIGH);
	}
	else {
		//set low
		pio_setOutput(DEVICE_LED_PIO,DEVICE_LED_PIN,PIN_LOW);
	}
	
	switch (car_state) {
		case TRACTIVE_SYSTEM_OFF: 
			if (xTimerIsTimerActive(TSLedTimer) == pdTRUE) {
				xTimerStop(TSLedTimer,0);
			}
			pio_setOutput(TS_LED_PIO,TS_LED_PIN,PIN_LOW);
		break;
		case TRACTIVE_SYSTEM_ON:
		 // Blink Green led
		// Start a software timer. which alternates between setting the led high and low 
		if (xTimerIsTimerActive(TSLedTimer) == pdFALSE) {
			xTimerReset(TSLedTimer,0);
		}
		break;
		case DRIVE_ENABLED:
		case LC_PROCEDURE:
		case LC_WAITING_FOR_ECU_TO_ARM_LC:
		case LC_COUNTDOWN:
		case LC_ARMED:
		// Constant Green led
		// Turn off the timer 
		if (xTimerIsTimerActive(TSLedTimer) == pdTRUE) {
			xTimerStop(TSLedTimer,0);
		}
		pio_setOutput(TS_LED_PIO,TS_LED_PIN,PIN_HIGH);
		break;
	}
}

static void getDashMessages(Variables *var, ConfirmationMsgs *conf_msg, ModuleError *error, SensorValues *sensor_values,StatusMsg *status,SensorRealValue *sensor_real) {
	struct CanMessage txmsg = {
		.data.u8[0] = 10,
		.data.u8[1] = 5,
		.dataLength = 2,
		.messageID = 10
	};
	//can_sendMessage(CAN0,txmsg);
	struct CanMessage ReceiveMsg;
	if (xQueueReceive(xDashQueue,&ReceiveMsg,5) == pdTRUE) {
		//can_sendMessage(CAN0,txmsg);
		//Received a Can message over the queue
		
		switch (ReceiveMsg.messageID) {
			case 11:
				//Temporary testing: Variable confirmation ID
				setVariableBasedOnConfirmation(var);
				parameter_confirmation_timed_out = false; // Reset the global status flag
			break;
			case ID_TRQ_CONF_CH0:
				switch (ReceiveMsg.data.u8[0]) {
					case 0x0F:
						conf_msg->conf_trq_ch0 = TRQ_MIN_CONFIRMED;
						break;
					case 0xF0:
						conf_msg->conf_trq_ch0 = TRQ_MAX_CONFIRMED;
						break;
					case 0x00:
						conf_msg->conf_trq_ch0 = TRQ_NOCALIB;
						break;
				}
			break;
			case ID_TRQ_CONF_CH1:
				switch (ReceiveMsg.data.u8[0]) {
					case 0x0F:
						conf_msg->conf_trq_ch0 = TRQ_MIN_CONFIRMED;
						break;
					case 0xF0:
						conf_msg->conf_trq_ch0 = TRQ_MAX_CONFIRMED;
						break;
					case 0x00:
						conf_msg->conf_trq_ch0 = TRQ_NOCALIB;
						break;
				}
			break;
			case ID_STEERING_CONF:
				switch (ReceiveMsg.data.u8[0]) {
					case 0x0F:
					conf_msg->conf_steer = STEER_CONF_LEFT;
					break;
					case 0xF0:
					conf_msg->conf_steer = STEER_CONF_RIGHT;
					break;
					case 0x00:
					conf_msg->conf_steer = STEER_CONF_FAILED;
					break;
				}
			break;
			case ID_ECU_CAR_STATES:
				switch (ReceiveMsg.data.u8[0]) {
					case 0x0F:
					//TS active
					status->shut_down_circuit_closed = true;
					break;
					case 0x02:
					//Play RTDS. The timer callback will turn it off after RTDS_DURATION_MS has passed.
					// It will also send a can message telling the ECU the dash is done with RTDS.
					xTimerStart(RTDSTimer,200/portTICK_RATE_MS);
					pio_setOutput(BUZZER_PIO,BUZZER_PIN,PIN_HIGH);
					break;
					case 0x0E:
					//Ready to drive, drive enabled
					conf_msg->drive_enabled_confirmed = true;
					break;
					case 0x04:
					//Drive disabled
					conf_msg->drive_disabled_confirmed = true;
					break;
				}
			break;
			case ID_IN_ECU_LC:
				switch (ReceiveMsg.data.u8[0]) {
					case 1:
					//Last year has only lc ready and launch end. Makes more sense with lc request confirmed and lc ready and maybe launch end
					//Launch ready
					conf_msg->lc_ready = true;
					break;
					case 2:
					// Launch request accepted Starting countdown
					conf_msg->lc_request_confirmed = true;
					break;
				}
			break;
		
			
			case ID_BMS_MAX_MIN_VALUES:
				sensor_real->max_cell_voltage_lsb = ReceiveMsg.data.u8[0];
				sensor_real->max_cell_voltage_msb = ReceiveMsg.data.u8[1];
				
				sensor_real->min_cell_voltage_lsb = ReceiveMsg.data.u8[2];
				sensor_real->min_cell_voltage_lsb = ReceiveMsg.data.u8[3];
				
				sensor_real->max_battery_temperature_lsb = ReceiveMsg.data.u8[4];
				sensor_real->max_battery_temperature_msb = ReceiveMsg.data.u8[5];
				
				sensor_real->min_battery_temperature_lsb = ReceiveMsg.data.u8[6];
				sensor_real->min_battery_temperature_msb = ReceiveMsg.data.u8[7];
			break;
			
			case ID_TORQUE_ENCODER_0_DATA:
				sensor_real->torque_encoder_ch0 = ReceiveMsg.data.u8[0];
				break;
				
			case ID_TORQUE_ENCODER_1_DATA:
				sensor_real->torque_encoder_ch1 = ReceiveMsg.data.u8[0];
				break;	
				
				
			case ID_SPEED_FL:
			case ID_SPEED_FR:
			case ID_SPEED_RR:
			case ID_SPEED_RL:
				break;
			case ID_TEMP_COOLING:
				sensor_values->temp_sensor_cooling = ( (ReceiveMsg.data.u8[0] << 8) | ReceiveMsg.data.u8[1]);
				break;
			case ID_TEMP_GEARBOX:
				sensor_values->temp_sensor_gearbox = ( (ReceiveMsg.data.u8[0] << 8) | ReceiveMsg.data.u8[1]);
				break;
			case ID_BRAKE_PRESSURE_FL:
				sensor_values->brake_pressure_fl = ( (ReceiveMsg.data.u8[0] << 8) | ReceiveMsg.data.u8[1]);
				break;
				//(brk_pres_front-3960)/120)
			case ID_BRAKE_PRESSURE_FR:
				sensor_values->brake_pressure_fr = ( (ReceiveMsg.data.u8[0] << 8) | ReceiveMsg.data.u8[1]);
				break;
			case ID_DAMPER_FL:
			case ID_DAMPER_FR:
			case ID_DAMPER_RL:
			case ID_DAMPER_RR:
				break;
			case ID_IMD_SHUTDOWN:
				error->imd_error = true;
				break;
		}
	}
}

//***********************************************************************************
//------------------------------------MENU HELPER FUNCTIONS-------------------------//
//***********************************************************************************

static void can_freeRTOSSendMessage(Can *can,struct CanMessage message) {
	
	if (can == CAN0) {
		xSemaphoreTake(can_mutex_0, portMAX_DELAY);
		
		can_sendMessage(can,message);
		
		xSemaphoreGive(can_mutex_0);
	}
	else if (can == CAN1) {
		xSemaphoreTake(can_mutex_1, portMAX_DELAY);
		
		can_sendMessage(can,message);
		
		xSemaphoreGive(can_mutex_1);
	}
	

}

static bool checkForError(ModuleError *error) {
	if ( (error->ecu_error == 0) && (error->bms_fault == 0) && (error->bms_warning == 0) ) {
		return false;
	}
	else {
		return true;
	}
}
static void HandleErrors(ModuleError *error) {
	selected = ERROR_HANDLER_POS;
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd(COLOR_RGB(255,0,0));
	if (error->ecu_error != 0) {
		cmd_text(5,20,31,OPT_FLAT,ecu_error_names[error->ecu_error]);
	}
	if (error->bms_fault != 0) {
		cmd_text(5,55,31,OPT_FLAT,bms_fault_names[error->bms_fault]);
	}

	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();

}


static bool trq_ch0_ok = false;
static bool trq_ch1_ok = false;
static bool trq_noCalib_ch0 = false;
static bool trq_noCalib_ch1 = false;
static void calibrateTorquePedal(ConfirmationMsgs *conf_msgs,bool ack_pressed) {
	
	
	switch(trq_calib_state) {
		case TRQ_CALIBRATION_OFF:
			DrawTorqueCalibrationScreen(conf_msgs);
			if (ack_pressed == true) {
				conf_msgs->conf_trq_ch0 = TRQ_DEFAULT;
				conf_msgs->conf_trq_ch1 = TRQ_DEFAULT;
				trq_calib_timed_out = false;
				trq_calib_state = TRQ_CALIBRATION_WAITING_MAX_CONFIRMATION;
				//Send CAN message that max is being calibrated
				can_freeRTOSSendMessage(CAN0,TorquePedalCalibrationMax);
				can_freeRTOSSendMessage(CAN1,TorquePedalCalibrationMax);
				xTimerReset(calibrationTimer,2/portTICK_RATE_MS);
			}
			break;
		case TRQ_CALIBRATION_WAITING_MAX_CONFIRMATION:
			if (trq_calib_timed_out == false) {
				switch (conf_msgs->conf_trq_ch0) {
					case TRQ_MAX_CONFIRMED:
						trq_ch0_ok = true;
						break;
					case TRQ_NOCALIB:
						trq_ch0_ok = true;
						break;
				}	
				switch (conf_msgs->conf_trq_ch1) {
					case TRQ_MAX_CONFIRMED:
						trq_ch1_ok = true;
						break;
					case TRQ_NOCALIB:
						trq_noCalib_ch1 = true;
						break;
				}			
				if (trq_ch0_ok && trq_ch1_ok) {
					trq_calib_state = TRQ_CALIBRATION_MAX_CONFIRMED;
				}
				else if ( (trq_noCalib_ch0 == true) && (trq_noCalib_ch1 == false) ) {
					// Ch0 calib error
					trq_calib_state = TRQ_FAIL_CH0;
				}
				else if ( (trq_noCalib_ch0 == false) && (trq_noCalib_ch1 == true) ) {
					// Ch1 calib error
					trq_calib_state = TRQ_FAIL_CH1;
				}
				else if (trq_noCalib_ch0 && trq_noCalib_ch1) {
					//Fail
					trq_calib_state = TRQ_FAIL_BOTH_CH;
				}
			}
			else {
				//Timed out
				if ( (trq_ch0_ok == false) && (trq_ch1_ok == false) ) {
					//Both timed out
					trq_calib_state = TRQ_TIMEOUT_BOTH_CH;
				}
				else if ( (trq_ch0_ok == true) && (trq_ch1_ok == false) ) {
					// trq_ch1 timed out
					trq_calib_state = TRQ_TIMEOUT_CH1;
				}
				else if ( (trq_ch0_ok == false) && (trq_ch1_ok == true) ) {
					// Trq_ch0 timed out
					trq_calib_state = TRQ_TIMEOUT_CH0;
				}
			}
		break;
			
		case TRQ_CALIBRATION_MAX_CONFIRMED:
			DrawTorqueCalibrationScreen(conf_msgs);
			if (ack_pressed == true) {
				conf_msgs->conf_trq_ch0 = TRQ_DEFAULT; // Reset the confirmation message
				conf_msgs->conf_trq_ch1 = TRQ_DEFAULT;
				trq_ch0_ok = false;
				trq_ch1_ok = false;
				trq_noCalib_ch0 = false;
				trq_noCalib_ch1 = false;
				trq_calib_state = TRQ_CALIBRATION_WAITING_MIN_CONFIRMATION;
				// Send CAN message that min torque is being calibrated
				can_freeRTOSSendMessage(CAN0,TorquePedalCalibrationMin);
				can_freeRTOSSendMessage(CAN1,TorquePedalCalibrationMin);
				trq_calib_timed_out = false; // Reset time out flag
				xTimerReset(calibrationTimer,5/portTICK_RATE_MS);
			}
		break;

		case TRQ_CALIBRATION_WAITING_MIN_CONFIRMATION:
			if (trq_calib_timed_out == false) {
				switch (conf_msgs->conf_trq_ch0) {
					case TRQ_MIN_CONFIRMED:
					trq_ch0_ok = true;
					break;
					case TRQ_NOCALIB:
					trq_ch0_ok = true;
					break;
				}
				switch (conf_msgs->conf_trq_ch1) {
					case TRQ_MIN_CONFIRMED:
					trq_ch1_ok = true;
					break;
					case TRQ_NOCALIB:
					trq_noCalib_ch1 = true;
					break;

				}
				// Make swithc case with trq0 and tr1 combinged into a 2 bit variable
				if (trq_ch0_ok && trq_ch1_ok) {
					trq_calib_state = TRQ_CALIBRATION_MIN_CONFIRMED;
				}
				else if ( (trq_noCalib_ch0 == true) && (trq_noCalib_ch1 == false) ) {
					// Ch0 calib error
					trq_calib_state = TRQ_FAIL_CH0;
				}
				else if ( (trq_noCalib_ch0 == false) && (trq_noCalib_ch1 == true) ) {
					// Ch1 calib error
					trq_calib_state = TRQ_FAIL_CH1;
				}
				else if (trq_noCalib_ch0 && trq_noCalib_ch1) {
					//Fail
					trq_calib_state = TRQ_FAIL_BOTH_CH;
				}
			}
			else {
				//Timed out
				if ( (trq_ch0_ok == false) && (trq_ch1_ok == false) ) {
					//Both timed out
					trq_calib_state = TRQ_TIMEOUT_BOTH_CH;
				}
				else if ( (trq_ch0_ok == true) && (trq_ch1_ok == false) ) {
					// trq_ch1 timed out
					trq_calib_state = TRQ_TIMEOUT_CH1;
				}
				else if ( (trq_ch0_ok == false) && (trq_ch1_ok == true) ) {
					// Trq_ch0 timed out
					trq_calib_state = TRQ_TIMEOUT_CH0;
				}
			}
		break;

		case TRQ_CALIBRATION_MIN_CONFIRMED:
			DrawTorqueCalibrationScreen(conf_msgs);
			if (ack_pressed == true) {
				trq_ch0_ok = false;
				trq_ch1_ok = false;
				trq_noCalib_ch0 = false;
				trq_noCalib_ch1 = false;
				conf_msgs->conf_trq_ch0 = TRQ_DEFAULT; // Reset the confirmation message
				conf_msgs->conf_trq_ch1 = TRQ_DEFAULT;
				
				trq_calib_timed_out = false; // Reset time out flag
				trq_calib_state = TRQ_CALIBRATION_OFF;
				selected = MAIN_MENU_POS; // Back to adjustment menu
			}
		break;
	
		case TRQ_FAIL_BOTH_CH:
		case TRQ_FAIL_CH0:
		case TRQ_FAIL_CH1:
		case TRQ_TIMEOUT_CH0:
		case TRQ_TIMEOUT_CH1:
		case TRQ_TIMEOUT_BOTH_CH:
			DrawTorqueCalibrationScreen(conf_msgs);
			if (ack_pressed == true) {
				conf_msgs->conf_trq_ch0 = TRQ_DEFAULT;
				conf_msgs->conf_trq_ch1 = TRQ_DEFAULT;
				trq_ch0_ok = false;
				trq_ch1_ok = false;
				trq_noCalib_ch0 = false;
				trq_noCalib_ch1 = false;
				
				trq_calib_timed_out = false;
				trq_calib_state = TRQ_CALIBRATION_OFF;
				selected = MAIN_MENU_POS;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
			break;
	}
}
static void calibrateSteering(ConfirmationMsgs *conf_msgs,bool ack_pressed) {
	switch (steer_calib_state) {
		
		case STEER_C_OFF:
			DrawSteerCalibScreen();
			if (ack_pressed) {
				conf_msgs->conf_steer = STEER_CONF_DEFAULT;
				steer_calib_timed_out = false;
				steer_calib_state = STEER_C_WAITING_LEFT;
				// Send CAN message that left is calibrated
				can_freeRTOSSendMessage(CAN0,SteeringCalibrationLeft);
				can_freeRTOSSendMessage(CAN1,SteeringCalibrationLeft);
				xTimerReset(calibrationTimer,15/portTICK_RATE_MS);	
			}
			break;
		case STEER_C_WAITING_LEFT:
			if (steer_calib_timed_out == false) {
				switch (conf_msgs->conf_steer) {
					case STEER_CONF_LEFT:
						steer_calib_state = STEER_C_LEFT_CONFIRMED;
					break;
					case STEER_CONF_FAILED:
						steer_calib_state = STEER_C_FAIL;
					break;
				}
			}
			else if (steer_calib_timed_out == true) {
				steer_calib_state = STEER_C_TIMEOUT;
			}
		break;
		
		case STEER_C_LEFT_CONFIRMED:
			DrawSteerCalibScreen();
			if (ack_pressed == true) {
				conf_msgs->conf_steer = STEER_CONF_DEFAULT;
				steer_calib_state = STEER_C_WAITING_RIGHT;
				steer_calib_timed_out = false;
				// Send CAN message right is being calibrated
				can_freeRTOSSendMessage(CAN0,SteeringCalibrationRight);
				can_freeRTOSSendMessage(CAN1,SteeringCalibrationRight);
				xTimerReset(calibrationTimer,5/portTICK_RATE_MS);
			}
			break;
			
		case STEER_C_WAITING_RIGHT:
			if (steer_calib_timed_out == false) {
				switch (conf_msgs->conf_steer) {
					case STEER_CONF_LEFT:
						steer_calib_state = STEER_C_RIGHT_CONFIRMED;
					break;
					case STEER_CONF_FAILED:
						steer_calib_state = STEER_C_FAIL;
					break;
				}
			}
			else if (steer_calib_timed_out == true) {
				steer_calib_state = STEER_C_TIMEOUT;
			}
			break;
						
		case STEER_C_RIGHT_CONFIRMED:
			DrawSteerCalibScreen();
			if (ack_pressed == true) {
				conf_msgs->conf_steer = STEER_CONF_DEFAULT;
				steer_calib_state = STEER_C_OFF;
				selected = MAIN_MENU_POS;
				steer_calib_timed_out = false;
			}
			break;
		case STEER_C_FAIL:
			DrawSteerCalibScreen();
			if (ack_pressed == true) {
				conf_msgs->conf_steer = STEER_CONF_DEFAULT;
				steer_calib_state = STEER_C_OFF;
				selected = MAIN_MENU_POS;
				steer_calib_timed_out = false;	
			}
		break;
		case STEER_C_TIMEOUT:
			DrawSteerCalibScreen();
			if (ack_pressed == true) {
				conf_msgs->conf_steer = STEER_CONF_DEFAULT;
				steer_calib_state = STEER_C_OFF;
				selected = MAIN_MENU_POS;
				steer_calib_timed_out = false;
			}
		break;
	}
}


static EPedalPosition getTorquePedalPosition(SensorRealValue *sensor_real) {
	if ( (sensor_real->torque_encoder_ch0 < TORQUE_PEDAL_THRESHOLD ) && (sensor_real->torque_encoder_ch1 < TORQUE_PEDAL_THRESHOLD) ) {
		return PEDAL_IN;
	}
	else return PEDAL_OUT;
}
static EPedalPosition getBrakePedalPosition(SensorRealValue *sensor_real) {
	if (sensor_real->brake_pedal_actuation < BRAKE_PEDAL_THRESHOLD) { //brakePedalEngagedThreshold) {
		return PEDAL_IN;
	}
	else return PEDAL_OUT;
}
static bool bmsNotCharged(SensorRealValue *sensor_real) {
	uint8_t bms_discharge_threshold = 50;
	if ( sensor_real->bms_discharge_limit < bms_discharge_threshold ) {
		return false;
	}
	else return true;
}

static void vRTDSCallback(TimerHandle_t xTimer) {
	pio_setOutput(BUZZER_PIO,BUZZER_PIN,PIN_LOW);
	RTDS_finished_playing = true;
	//can_sendMessage(CAN0,FinishedRTDS);
}
static void vLcTimerCallback (TimerHandle_t lcTimer) {
	lc_timer_count += 1;
	if (lc_timer_count == 6) {
		xTimerStop(lcTimer,0);
	}
}
static void vCalibrationTimerCallback(TimerHandle_t xTimer){
	trq_calib_timed_out = true;
	steer_calib_timed_out = true;
}
static void vVarConfTimerCallback(TimerHandle_t xTimer) {
	parameter_confirmation_timed_out = true;
	selected = prev_selected;
}
static void vMenuUpdateCallback(TimerHandle_t pxTimer) {
	uint8_t menu_id;
	menu_id = (uint8_t) pvTimerGetTimerID(pxTimer);
	switch (menu_id) {
		case 0: // Main Screen
		menuUpdate.update_menu = true;
		xTimerReset(timer_menuUpdate[0],0);
		break;
	}
}
static void vTSLedTimerCallback(TimerHandle_t pxtimer){
	if (pio_readPin(TS_LED_PIO,TS_LED_PIN) == 1) {
		pio_setOutput(TS_LED_PIO,TS_LED_PIN,PIN_LOW);
	}
	else {
		pio_setOutput(TS_LED_PIO,TS_LED_PIN, PIN_HIGH);
	}
}

static void init_sensorRealValue_struct(SensorRealValue *sensorReal) {
	sensorReal->torque_encoder_ch0 = 0;
	sensorReal->torque_encoder_ch1 = 0;
	sensorReal->brake_pressure_rear = 0;
	sensorReal->brake_pressure_front = 0;
	sensorReal->steering_angle = 0;
	
	sensorReal->min_battery_temperature = 0;
	sensorReal->battery_temperature = 0;
	sensorReal->max_battery_temperature = 0;
	sensorReal->motor_temperature = 0;
	sensorReal->IGBT_temperature = 0;
	sensorReal->gearbox_temperature = 0;
	
	sensorReal->battery_voltage = 0;
	sensorReal->min_cell_voltage_lsb = 0;
	sensorReal->min_cell_voltage_msb = 0;
	sensorReal->min_cell_id = 0;
	sensorReal->max_cell_voltage_msb = 0;
	sensorReal->max_cell_voltage_lsb = 0;
	sensorReal->max_cell_id = 0;
	sensorReal->GLV_voltage_msb = 0;
	sensorReal->GLV_voltage_lsb = 0;
	
	sensorReal->car_speed = 0;
	sensorReal->bms_discharge_limit = 0;
	sensorReal->brake_pedal_actuation = 0;
}
static void init_sensor_value_struct(SensorValues *sensor_values) {
	sensor_values->bms_discharge_limit = 0;
	sensor_values->brake_pressure_fr = 0;
	sensor_values->brake_pressure_fl = 0;
	sensor_values->temp_sensor_cooling = 0;
	sensor_values->temp_sensor_gearbox = 0;

}
static void init_status_struct(StatusMsg *status) {
	status->shut_down_circuit_closed = false;
}
static void init_conf_msgs_struct(ConfirmationMsgs *conf_msgs) {
	conf_msgs->drive_disabled_confirmed = false;
	conf_msgs->drive_enabled_confirmed = false;
	conf_msgs->lc_request_confirmed = false;
	conf_msgs->lc_ready = false;
	conf_msgs->conf_trq_ch0 = TRQ_DEFAULT;
	conf_msgs->conf_trq_ch1 = TRQ_DEFAULT;
	conf_msgs->ECU_parameter_confirmed = false;
	}
static void init_error_struct(ModuleError *error) {
	
	error->ams_error = false;
	error->imd_error = false;
	error->ecu_error   = 0;
	error->bms_fault   = 0;
	error->bms_warning = 0;
}
static void init_variable_struct(Variables *var) {
	var->min_torque = 0;
	var->torque = 0;
	var->prev_confirmed_torque = 0;
	var->max_torque = 100;
	
	var->min_P_term = 0;
	var->P_term = 10;
	var->prev_confirmed_P_term = 10;
	var->max_P_term = 11;
	
	var->min_D_term = 0;
	var->D_term = 5;
	var->prev_confirmed_D_term = 5;
	var->max_D_term = 13;
	
	var->min_I_term = 0;
	var->I_term = 1;
	var->prev_confirmed_I_term = 1;
	var->max_I_term = 14;
	
	var->min_T_term = 0;
	var->T_term = 2;
	var->prev_confirmed_T_term = 2;
	var->max_T_term = 15;
	
	var->min_R_term = 0;
	var->R_term = 6;
	var->prev_confirmed_R_term = 6;
	var->max_R_term = 16;
}
static void clearAllButtons() {
	btn.unhandledButtonAction = false;
	btn.btn_type = NONE_BTN;
	btn.navigation = NAV_DEFAULT;
	btn.rotary_ccw = false;
	btn.rotary_cw  = false;

}

static bool checkDeviceStatus(DeviceState *devices) {
	if (devices->ECU == ALIVE && ( (devices->TRQ_0 == ALIVE) || (devices->TRQ_0 == UNITIALIZED) ) && ( (devices->TRQ_1 == ALIVE) || (devices->TRQ_1 == UNITIALIZED))
	 && devices->BSPD == ALIVE && devices->TEL == ALIVE && devices->ADC_FR == ALIVE && \
	devices->ADC_FL== ALIVE && devices->ADC_RR == ALIVE && devices->ADC_RL == ALIVE && devices->INV == ALIVE && devices->FAN == ALIVE && \
	devices->BMS == ALIVE && devices->GLVBMS == ALIVE && devices->IMU == ALIVE && devices->STEER_POS == ALIVE && devices->IMD == ALIVE) {
		return true;
	}
	else {
		return false;
	}
}

static void setVariableBasedOnConfirmation(Variables *var) {
	
	switch (menu[selected].current_setting) {
		case TORQUE_SETTING:
			if (parameter_confirmation_timed_out == true) {
				var->torque = var->prev_confirmed_torque;
			}
			else {
				var->prev_confirmed_torque = var->torque;
			}
		break;
		case ECU_P_SETTING:
			if (parameter_confirmation_timed_out == true) {
				var->P_term = var->prev_confirmed_P_term;
			}
			else {
				var->prev_confirmed_P_term = var->P_term;
			}
		break;
		
		case ECU_I_SETTING:
			if (parameter_confirmation_timed_out == true) {
				var->I_term = var->prev_confirmed_I_term;
			}
			else {
				var->prev_confirmed_I_term = var->I_term;
			}
		break;
		
	}
	if (menu[selected].current_setting == TORQUE_SETTING) {
		if (parameter_confirmation_timed_out == true) {
			var->torque = var->prev_confirmed_torque;
		}
		else {
			var->prev_confirmed_torque = var->torque;
		}
	}
	else if (menu[selected].current_setting == ECU_P_SETTING) {
		if (parameter_confirmation_timed_out == true) {
			var->P_term = var->prev_confirmed_P_term;
		}
		else {
			var->prev_confirmed_P_term = var->P_term;
		}
		
	}
	else if (menu[selected].current_setting == ECU_D_SETTING) {
		if (parameter_confirmation_timed_out == true) {
			var->D_term = var->prev_confirmed_D_term;
		}
		else {
			var->prev_confirmed_D_term = var->D_term;
		}
		
	}
	else if (menu[selected].current_setting == ECU_I_SETTING) {
		if (parameter_confirmation_timed_out == true) {
			var->I_term = var->prev_confirmed_I_term;
		}
		else {
			var->prev_confirmed_I_term = var->I_term;
		}
		
	}
	else if (menu[selected].current_setting == ECU_LC_RT_SETTING) {
		if (parameter_confirmation_timed_out == true) {
			var->R_term = var->prev_confirmed_R_term;
		}
		else {
			var->prev_confirmed_R_term = var->R_term;
		}
	}
	else if (menu[selected].current_setting == ECU_LC_INIT_TORQ_SETTING) {
		if (parameter_confirmation_timed_out == true) {
			var->T_term = var->prev_confirmed_T_term;
		}
		else {
			var->prev_confirmed_T_term = var->T_term;
		}
	}
}



static void createAndStartMenuUpdateTimers() {
	timer_menuUpdate[0] = xTimerCreate("MainScreen",100/portTICK_RATE_MS, pdTRUE, (void *) 0 ,vMenuUpdateCallback);
	if (timer_menuUpdate[0] == NULL) {
		
	}
	else if (xTimerStart(timer_menuUpdate[0],0) != pdPASS) {
		
	}
}

//***********************************************************************************
//------------------------------------CALCULATION FUNCTIONS-------------------------//
//***********************************************************************************
static void sensorValueToRealValue(SensorValues *sensor_values,SensorRealValue *sensor_real ) {
	//Formulas for converting raw sensor data to useful data here
}

//***********************************************************************************
//------------------------------------DRAWING FUNCTIONS----------------------------//
//***********************************************************************************

static void DrawMainScreen(SensorRealValue *sensor,uint8_t low_volt, uint8_t high_volt,DeviceState *devices) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	DrawHighVoltageSymbol();
	cmd(COLOR_RGB(255,255, 255));
	cmd_text(2,10,23,OPT_FLAT,"TRACTIVE SYSTEM");
	cmd_text(2,40,23,OPT_FLAT,"DRIVE ENABLE");
	cmd_text(2,70,23,OPT_FLAT,"LAUNCH CONTROL");
	cmd_text(2,100,23,OPT_FLAT,"DEVICES");
	
	switch (car_state) {
		case TRACTIVE_SYSTEM_OFF:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(190,10,23,OPT_FLAT,"OFF"); // TS OFF
			cmd_text(190,40,23,OPT_FLAT,"OFF"); // Drive enable OFF
			cmd_text(190,70,23,OPT_FLAT, "OFF"); // Launch control
			break;
		case TRACTIVE_SYSTEM_ON:
			cmd(COLOR_RGB(0,255,0));
			cmd_text(190,10,23,OPT_FLAT,"ON"); // TS ON
			cmd(COLOR_RGB(255,0,0));
			cmd_text(190,40,23,OPT_FLAT,"OFF"); // Drive enable OFF
			cmd_text(190,70,23,OPT_FLAT, "OFF"); // Launch control
			break;
		case DRIVE_ENABLED:
			cmd(COLOR_RGB(0,255,0));
			cmd_text(190,10,23,OPT_FLAT,"ON"); // TS ON
			cmd_text(190,40,23,OPT_FLAT,"ON"); // Drive enable ON
			cmd(COLOR_RGB(255,0,0));
			cmd_text(190,70,23,OPT_FLAT, "OFF"); // Launch control
			break;
		case LC_ARMED:
			cmd(COLOR_RGB(0,255,0));
			cmd_text(190,10,23,OPT_FLAT,"ON"); // TS ON
			cmd_text(190,40,23,OPT_FLAT,"ON"); // Drive enable ON
			cmd_text(190,70,23,OPT_FLAT, "ARMED"); // Launch control
			break;
	}
	//Modules ok, battery temp and motor temp
	if (checkDeviceStatus(devices)) {
		cmd(COLOR_RGB(0,250,0));
		cmd_text(190,10,23,OPT_FLAT,"OK");
	}
	else {
		cmd(COLOR_RGB(250,0,0));
		cmd_text(190,100,23,OPT_FLAT,"NR");
	}
	
	
	cmd(BEGIN(LINE_STRIP));
	cmd(COLOR_RGB(250,250,0)); // Make the main battery rectangle grey
	cmd(LINE_WIDTH(2*16));
	cmd(VERTEX2F(245*16, 0));
	cmd(VERTEX2F(480*16, 0));
	cmd(VERTEX2F(480*16, 272*16));
	cmd(VERTEX2F(245*16,272*16));
	cmd(VERTEX2F(245*16,0));
	
// 	cmd(VERTEX2F(245*16,0));
// 	cmd(VERTEX2F(0,0));
// 	cmd(VERTEX2F(0,75*16));
// 	cmd(VERTEX2F(243*16,75*16));

	DrawLowVoltageBattery(low_volt);
	DrawHighVoltageBattery(high_volt);
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawLowVoltageBattery(uint8_t battery_left_percent) {
	uint8_t battery_size = 170;
	uint16_t x_top = 260;
	uint16_t x_bottom = 340;
	uint8_t y_start_pos = 30;
	uint8_t bar_height = 30;
	uint8_t spacing = 15;
	
	uint16_t y_end =  200;
	uint16_t y_start = (battery_size*(100-battery_left_percent))/100 +y_start_pos;
	
	uint16_t x_center_text = x_top +(x_bottom - x_top)/2;
	uint16_t y_center_text = y_start_pos + (y_end - y_start_pos)/2;
	cmd(BEGIN(RECTS));
	cmd(COLOR_RGB(100,100,100)); // Make the main battery rectangle grey
	cmd(LINE_WIDTH(16*5));
	//Draw the frame
	//Draw rectangle in grey, then draw bars on top..
	cmd(VERTEX2II(x_top-5,y_start_pos, 0,0)); // Top left coordinates
	cmd(VERTEX2II(x_bottom+5, y_end,  0,0)); // Bottom rightcoordinates
	
	cmd(VERTEX2II(x_center_text-10,y_start_pos-10, 0,0)); // Top left coordinates
	cmd(VERTEX2II(x_center_text+10 , y_start_pos,  0,0)); // Bottom rightcoordinates
	
	//Draw number in % of remaining battery at the center
	if ( battery_left_percent <= 15) {
		cmd(COLOR_RGB(250,0,0)); // Change color to red
	}
	else if (battery_left_percent < 50) {
		cmd(COLOR_RGB(255,255,0));
	}
	else {
		cmd(COLOR_RGB(0,240,0)); // Change color back to green
	}
	cmd(VERTEX2II(x_top, y_start,0,0)); // Top left coordinates
	cmd(VERTEX2II(x_bottom,battery_size + y_start_pos, 0,0)); // Bottom rightcoordinates
	cmd(COLOR_RGB(255,255,255));
	cmd_number(x_center_text, y_center_text, 31, OPT_CENTER, battery_left_percent );
	cmd_text(x_center_text+35, y_center_text, 31, OPT_CENTER,"%");
	cmd(COLOR_RGB(255,255,0));
	cmd_text(x_center_text, 235, 31, OPT_CENTER,"GLV");
}
static void DrawHighVoltageBattery(uint8_t battery_left_percent) {

	uint8_t battery_size = 170;
	uint16_t x_top = 380;
	uint16_t x_bottom = 460;
	uint8_t y_start_pos = 30;
	uint8_t bar_height = 30;
	uint8_t spacing = 15;
	
	uint16_t y_end =  200; 
	uint16_t y_start = (battery_size*(100-battery_left_percent))/100 +y_start_pos;
	
	uint16_t x_center_text = x_top +(x_bottom - x_top)/2;
	uint16_t y_center_text = y_start_pos + (y_end - y_start_pos)/2;
	//cmd(CMD_DLSTART);
	//cmd(CLEAR(1, 1, 1)); // clear screen

	cmd(BEGIN(RECTS));
	cmd(COLOR_RGB(100,100,100)); // Make the main battery rectangle grey
	cmd(LINE_WIDTH(16*5));
	//Draw the frame
	//Draw rectangle in grey, then draw bars on top..
	cmd(VERTEX2II(x_top-5,y_start_pos, 0,0)); // Top left coordinates
	cmd(VERTEX2II(x_bottom+5, y_end,  0,0)); // Bottom rightcoordinates
	
	cmd(VERTEX2II(x_center_text-10,y_start_pos-10, 0,0)); // Top left coordinates
	cmd(VERTEX2II(x_center_text+10 , y_start_pos,  0,0)); // Bottom rightcoordinates
	
	
	//Draw number in % of remaining battery at the center

	if ( battery_left_percent <= 15) {
		cmd(COLOR_RGB(250,0,0)); // Change color to red
	}
	else if (battery_left_percent < 50) {
		cmd(COLOR_RGB(255,255,0));
	}
	else {
		cmd(COLOR_RGB(0,240,0)); // Change color back to green
	}
	cmd(VERTEX2II(x_top, y_start,0,0)); // Top left coordinates
	cmd(VERTEX2II(x_bottom,battery_size + y_start_pos, 0,0)); // Bottom rightcoordinates
	cmd(COLOR_RGB(255,255,255));
	cmd_number(x_center_text, y_center_text, 31, OPT_CENTER, battery_left_percent );
	cmd_text(x_center_text+35, y_center_text, 31, OPT_CENTER, "%");
	//cmd(DISPLAY()); // display the image
	//cmd(CMD_SWAP);
	//cmd_exec();
}

static void DrawSpeedScreen(SensorRealValue *sensor_real) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(240,140,31,OPT_CENTER,"120");
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawSystemMonitorScreen(ModuleError *error,SensorRealValue *val) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	cmd(COLOR_RGB(255,255,0));
	cmd_text(240,15,30,OPT_CENTER,"SYSTEM MONITOR");
	cmd(COLOR_RGB(255,255,255));

	cmd_text(60, 200, 26, OPT_CENTER, "Trq enc 0");
	cmd_number(50, 230, 31, OPT_CENTER, val->torque_encoder_ch0);
	cmd_text(95, 235, 22, OPT_CENTER, "%");

	cmd_text(60, 130, 26, OPT_CENTER, "Trq enc 1");
	cmd_number(50, 160, 31, OPT_CENTER, val->torque_encoder_ch1);
	cmd_text(95, 165, 22, OPT_CENTER, "%");
	
	
	cmd_text(180, 200, 26, OPT_CENTER, "Brk prs R");
	cmd_number(165, 230, 31, OPT_CENTER, val->brake_pressure_rear);
	cmd_text(225, 235, 22, OPT_CENTER, "%");
	
	cmd_text(180, 130, 26, OPT_CENTER, "Brk prs F");
	cmd_number(165, 160, 31, OPT_CENTER, val->brake_pressure_front);
	cmd_text(225, 165, 22, OPT_CENTER, "%");
	
	cmd_text(300, 200, 26, OPT_CENTER, "Steer ang");
	cmd_number(290, 230, 31, OPT_CENTER, val->steering_angle);
	
	
	cmd_text(120, 60, 28, OPT_CENTER, "ECU error");
	cmd_text(125, 90, 26, OPT_CENTER, ecu_error_names[error->ecu_error]);
	
	
	cmd_text(360, 60, 28, OPT_CENTER, "BMS fault code");
	cmd_text(360, 120, 28, OPT_CENTER, "BMS warning");
	cmd_text(365, 90, 26, OPT_CENTER, bms_fault_names[error->bms_fault]);
	cmd_text(365, 150, 26, OPT_CENTER, bms_warning_names[error->bms_warning]);
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	
}
static void DrawTempAndVoltScreen(SensorRealValue *tempvolt) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd(COLOR_RGB(255,255,0));
	cmd_text(240,15,30,OPT_CENTER,"TEMPERATURES AND VOLTAGES");
	
	cmd(COLOR_RGB(255,255,255));
	cmd_text(60, 200, 26, OPT_CENTER, "Battery min");
	cmd_number(50, 230, 31, OPT_CENTER, tempvolt->min_battery_temperature);
	cmd_text(95, 235, 22, OPT_CENTER, "C");
	
	cmd_text(60, 130, 26, OPT_CENTER, "Battery avg");
	if(tempvolt->battery_temperature>55){
		cmd(COLOR_RGB(255,0,0));
		cmd_number(50, 160, 31, OPT_CENTER,tempvolt->battery_temperature);
		cmd_text(95, 165, 22, OPT_CENTER, "C");
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		cmd_number(50, 160, 31, OPT_CENTER, tempvolt->battery_temperature);
		cmd_text(95, 165, 22, OPT_CENTER, "C");
	}
	
	cmd_text(60, 60, 26, OPT_CENTER, "Battery max");
	if(tempvolt->max_battery_temperature>55){
		cmd(COLOR_RGB(255,0,0));
		cmd_number(50, 90, 31, OPT_CENTER, tempvolt->max_battery_temperature);
		cmd_text(95, 95, 22, OPT_CENTER, "C");
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		cmd_number(50, 90, 31, OPT_CENTER, tempvolt->max_battery_temperature);
		cmd_text(95, 95, 22, OPT_CENTER, "C");
	}
	
	
	cmd_text(180, 60, 26, OPT_CENTER, "Motor" );
	if(tempvolt->motor_temperature>110){
		cmd(COLOR_RGB(255,0,0));
		cmd_number(170, 90, 31, OPT_CENTER, tempvolt->motor_temperature);
		cmd_text(215, 95, 29, OPT_CENTER, "C");
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		cmd_number(170, 90, 31, OPT_CENTER, tempvolt->motor_temperature);
		cmd_text(215, 95, 29, OPT_CENTER, "C");
	}
	
	cmd_text(180, 130, 26, OPT_CENTER, "IGBT" );
	cmd_number(170, 160, 31, OPT_CENTER, tempvolt->IGBT_temperature);
	cmd_text(215, 165, 29, OPT_CENTER, "C");
	
	
	cmd_text(180, 200, 26, OPT_CENTER, "Gearbox" );
	if(tempvolt->gearbox_temperature>70){
		cmd(COLOR_RGB(255,0,0));
		cmd_number(170, 230, 31, OPT_CENTER, tempvolt->gearbox_temperature);
		cmd_text(215, 235, 29, OPT_CENTER, "C");
		cmd(COLOR_RGB(255,255,255));
		
	}
	else {
		cmd_number(170, 230, 31, OPT_CENTER,  tempvolt->gearbox_temperature);
		cmd_text(215, 235, 29, OPT_CENTER, "C");
	}
	
	cmd_text(300, 130, 26, OPT_CENTER, "Btry pack");
	if ( (tempvolt->battery_voltage > 600) || (tempvolt->battery_voltage < 475) ) {
		cmd(COLOR_RGB(255,0,0));
		cmd_number(290, 160, 31, OPT_CENTER, tempvolt->battery_voltage);
		cmd_text(335, 165, 22, OPT_CENTER, "V");
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		cmd_number(290, 160, 31, OPT_CENTER, tempvolt->battery_voltage);
		cmd_text(335, 165, 22, OPT_CENTER, "V");
	}
	
	cmd_text(290, 200, 26, OPT_CENTER, "Min cell" );
	cmd_number(330, 200, 26, OPT_CENTER, tempvolt->min_cell_id);
	
	if( (tempvolt->min_cell_voltage_msb*10 +tempvolt->min_cell_voltage_lsb) < 32 ) {
		//check_voltage_fl=true;
		cmd(COLOR_RGB(255,0,0));
		cmd_number(280, 230, 31, OPT_CENTER, tempvolt->min_cell_voltage_msb);
		cmd_text(295, 230, 31, OPT_CENTER, "." );
		cmd_number(310, 230, 31, OPT_CENTER, tempvolt->min_cell_voltage_lsb);
		cmd_text(330, 235, 22, OPT_CENTER, "V");
		cmd(COLOR_RGB(255,255,255));
		}
	else {
		cmd_number(280, 230, 31, OPT_CENTER, tempvolt->min_cell_voltage_msb);
		cmd_text(295, 230, 31, OPT_CENTER, "." );
		cmd_number(310, 230, 31, OPT_CENTER, tempvolt->min_cell_voltage_lsb);
		cmd_text(330, 235, 22, OPT_CENTER, "V");
	}
	
	cmd_text(290, 60, 26, OPT_CENTER, "Max cell" );
	cmd_number(335, 60, 26, OPT_CENTER,tempvolt->max_cell_id);
	if( (tempvolt->max_cell_voltage_msb*10 + tempvolt->max_cell_voltage_lsb) > 42){
		//check_voltage_fl = true;
		cmd(COLOR_RGB(255,0,0));
		cmd_number(280, 95, 31, OPT_CENTER, tempvolt->max_cell_voltage_msb);
		cmd_text(295, 95, 31, OPT_CENTER, "." );
		cmd_number(310, 95, 31, OPT_CENTER, tempvolt->max_cell_voltage_lsb);
		cmd_text(330, 100, 22, OPT_CENTER, "V");
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		cmd_number(280, 95, 31, OPT_CENTER, tempvolt->max_cell_voltage_msb);
		cmd_text(295, 95, 31, OPT_CENTER, "." );
		cmd_number(310, 95, 31, OPT_CENTER, tempvolt->max_cell_voltage_lsb);
		cmd_text(330, 100, 22, OPT_CENTER, "V");
	}
	
	cmd_text(420, 200, 26, OPT_CENTER, "GLV" );
	if ( (tempvolt->GLV_voltage_msb*10 + tempvolt->GLV_voltage_lsb)  <19) {
		cmd(COLOR_RGB(255,0,0));
		cmd_number(390, 230, 31, OPT_CENTER, tempvolt->GLV_voltage_msb);
		cmd_text(420, 230, 31, OPT_CENTER, "." );
		cmd_number(435, 230, 31, OPT_CENTER, tempvolt->GLV_voltage_lsb);
		cmd_text(455, 235, 22, OPT_CENTER, "V");
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		cmd_number(390, 230, 31, OPT_CENTER, tempvolt->GLV_voltage_msb);
		cmd_text(420, 230, 31, OPT_CENTER, "." );
		cmd_number(435, 230, 31, OPT_CENTER, tempvolt->GLV_voltage_lsb);
		cmd_text(455, 235, 22, OPT_CENTER, "V");
	}
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}


static void DrawMainMenu() {
	
	uint8_t pos = selected - menu[selected].position; // First position in current menu
	uint8_t end_position  = pos + menu[pos].num_menupoints - 1;  // Last position in current menu
	uint16_t y_position = 20;
	uint16_t x_position = 10;
	
	uint16_t x_position_text = 120;
	uint16_t y_position_text = 50;
	
	
	uint16_t button_width = 230;
	uint16_t button_heigth = 60;
	uint8_t vert_spacing = 62;
		
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	//cmd_text(236,20 , 29, OPT_CENTER, menu[pos].text);
	pos = pos +1;
	cmd(COLOR_RGB(255,255,255));
	for (pos; pos <= (end_position-4); pos ++) {
		if ( pos == selected) {
			
			cmd(BEGIN(RECTS));
			cmd(COLOR_RGB(255,255,20)); // Make the main battery rectangle grey
			cmd(LINE_WIDTH(16*5));
			//cmd_fgcolor(0xffff33);
			//cmd(COLOR_RGB(0,0,0));
			cmd(VERTEX2F(x_position*16,y_position*16));
			cmd(VERTEX2F((x_position+button_width)*16, (y_position+button_heigth)*16));
			//cmd_button(x_position, y_position, button_width, button_heigth, 28, 0, menu[pos].text);
			//cmd(COLOR_RGB(255,255,255));
			cmd(COLOR_RGB(0,0,0));
			cmd_text(x_position_text,y_position_text,28,OPT_CENTER,menu[pos].text);
			
			cmd_coldstart();
		}
		else {
			//cmd_fgcolor(0x302020);
			//cmd_fgcolor(0x0);
			cmd(COLOR_RGB(255,255,255));
			cmd_text(x_position_text,y_position_text,28,OPT_CENTER,menu[pos].text);
			//cmd_button(x_position, y_position, button_width, button_heigth, 28, 0, menu[pos].text);
		}
		y_position += vert_spacing;
		y_position_text += vert_spacing;
	}
	
	y_position = 20;
	x_position = 250;
	
	x_position_text = 360;
	y_position_text = 50;
	for (pos ; pos <= end_position; pos ++) {
		if ( pos == selected) {		
			cmd(BEGIN(RECTS));
			cmd(COLOR_RGB(255,255,20)); // Make the main battery rectangle grey
			cmd(LINE_WIDTH(16*5));
			//cmd_fgcolor(0xffff33);
			//cmd(COLOR_RGB(0,0,0));
			cmd(VERTEX2F(x_position*16,y_position*16));
			cmd(VERTEX2F((x_position+button_width)*16, (y_position+button_heigth)*16));
			//cmd_button(x_position, y_position, button_width, button_heigth, 28, 0, menu[pos].text);
			//cmd(COLOR_RGB(255,255,255));
			cmd(COLOR_RGB(0,0,0));
			cmd_text(x_position_text,y_position_text,28,OPT_CENTER,menu[pos].text);
					
			cmd_coldstart();
		}
		else {
			//cmd_fgcolor(0x302020);
			//cmd_fgcolor(0x0);
			cmd(COLOR_RGB(255,255,255));
			cmd_text(x_position_text,y_position_text,28,OPT_CENTER,menu[pos].text);
			//cmd_button(x_position, y_position, button_width, button_heigth, 28, 0, menu[pos].text);
		}
		y_position += vert_spacing;
		y_position_text += vert_spacing;
	}
		
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}

static void DrawAdjustmentMenu() {
	uint8_t y_position = 60;
	uint8_t pos = selected - menu[selected].position; // First position in current menu
	uint8_t end_position  = pos + menu[pos].num_menupoints - 1;  // Last position in current menu
	
	uint16_t x_position = 120;
	uint16_t button_width = 260;
	uint16_t button_heigth = 40;
	uint8_t vert_spacing = 45;
	
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(236,20 , 29, OPT_CENTER, menu[pos].text);
	pos = pos +1;
	for (pos; pos <= end_position; pos ++) {
		if ( pos == selected) {
			cmd_fgcolor(0xb9b900);
			cmd(COLOR_RGB(0,0,0));
			cmd_button(x_position, y_position, button_width, button_heigth, 28, 0, menu[pos].text);
			cmd(COLOR_RGB(255,255,255));
			cmd_coldstart();
		}
		else {
			cmd_button(x_position, y_position, button_width, button_heigth, 28, 0, menu[pos].text);
		}
		y_position += vert_spacing;
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	// Delay in 2014. Why?
}




static void DrawECUAdjustmentScreen(Variables *var) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	/*Draw 5 variable names, 5 sliders, 5 min values, 5 max values and current value at center of each slider in big font.
	Also draw a frame around the current selected variable. */
// 	cmd(BEGIN(LINE_STRIP));
// 	cmd(COLOR_RGB(250,250,0));
// 	cmd(VERTEX2F(245*16, 0));
// 	cmd(VERTEX2F(480*16, 0));
// 	cmd(VERTEX2F(480*16, 272*16));
// 	cmd(VERTEX2F(245*16,272*16));
// 	cmd(VERTEX2F(245*16,0));
	uint8_t menu_pos = ECU_SETTINGS_MENU_POS;
	uint8_t variable_pos = ECU_SETTINGS_VARIABLES_POS;
	uint8_t end_menu_pos = menu_pos + menu[ECU_SETTINGS_MENU_POS].num_menupoints - 1;
	uint8_t end_variable_pos = variable_pos + menu[ECU_SETTINGS_VARIABLES_POS].num_menupoints -1;
	
	uint32_t y_menu_position = 52;
	uint32_t x_menu_position = 5;
	uint32_t vertical_menu_spacing = 55;
	uint8_t font_size = 27;
	
	cmd_text(240,10,30,OPT_CENTER,"ECU OPTIONS");
	for (menu_pos; menu_pos <= end_menu_pos ; menu_pos ++) {
		if (selected == menu_pos) {
			cmd(COLOR_RGB(255,255,0));
			cmd_text(x_menu_position,y_menu_position,font_size,OPT_FLAT,menu[menu_pos].text);
		}
		else {
			cmd(COLOR_RGB(255,255,255));
			cmd_text(x_menu_position,y_menu_position,font_size,OPT_FLAT,menu[menu_pos].text);
		}
		y_menu_position += vertical_menu_spacing;
	}
	uint32_t x_slider_position = 230;
	uint32_t y_slider_position = 60;
	uint8_t vertical_slider_spacing = 55;
	uint32_t slider_width = 190;
	uint8_t slider_heigth = 10;
	
	uint8_t x_num_adj = 34;
	uint8_t x_max_num_adj = 215;
	uint8_t y_num_adj = 10;
	uint8_t num_font_size = 27;
	
	uint16_t range_P_term = var->max_P_term - var->min_P_term;
	uint16_t range_I_term = var->max_I_term - var->min_I_term;
	uint16_t range_D_term = var->max_D_term - var->min_D_term;
	uint16_t range_T_term = var->max_T_term - var->min_T_term;
	uint16_t range_R_term = var->max_R_term - var->min_R_term;
	//Knob : fgcolor
	//Left of knob : COLOR_RGB
	//Right of knob : bgcolor
	uint32_t color_right = 0x0;
	uint32_t color_knob  = 0x0000FF;

	for (variable_pos; variable_pos <= end_variable_pos; variable_pos ++) {
		
		if (menu[variable_pos].current_setting == TORQUE_SETTING) {
			if (selected == variable_pos) {
				cmd_fgcolor(0x000000); // Try black knob
				cmd_bgcolor(color_right); // Yellow right of knob
				cmd(COLOR_RGB(255,255,0)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->torque,100);
				cmd_coldstart();
			}
			else {
				cmd_coldstart();
				cmd_bgcolor(color_right);
				cmd(COLOR_RGB(140,140,140)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->torque,100);
			}
			cmd(COLOR_RGB(255,255,255));
			cmd_number(x_slider_position-x_num_adj,y_slider_position-y_num_adj,num_font_size,OPT_FLAT,var->min_torque);
			cmd_number(x_slider_position+x_max_num_adj,y_slider_position - y_num_adj,num_font_size,OPT_FLAT,var->max_torque);
			cmd(COLOR_RGB(255,0,0));
			cmd_number(x_slider_position+(slider_width/2),y_slider_position +y_num_adj,num_font_size,OPT_CENTER,var->torque);

		}
		else if (menu[variable_pos].current_setting == ECU_P_SETTING) {
			if (selected == variable_pos) {
				cmd_fgcolor(0xff33ff); // Pink knob
				cmd_bgcolor(color_right); // Yellow right of knob
				cmd(COLOR_RGB(255,255,0)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT, var->P_term - var->min_P_term, range_P_term);
				cmd_coldstart();
			}
			else {
				cmd_coldstart();
				cmd_bgcolor(color_right);
				cmd(COLOR_RGB(140,140,140)); 
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT, var->P_term - var->min_P_term, range_P_term);
			}
			cmd(COLOR_RGB(255,255,255));
			cmd_number(x_slider_position-x_num_adj,y_slider_position-y_num_adj,num_font_size,OPT_FLAT,var->min_P_term);
			cmd_number(x_slider_position+x_max_num_adj,y_slider_position - y_num_adj,num_font_size,OPT_FLAT,var->max_P_term);
			cmd(COLOR_RGB(255,0,0));
			cmd_number(x_slider_position+(slider_width/2),y_slider_position +y_num_adj,num_font_size,OPT_CENTER,var->P_term);
		}
		else if (menu[variable_pos].current_setting == ECU_D_SETTING) {
			if (selected == variable_pos) {
				cmd_fgcolor(0xff33ff); // Pink knob
				cmd_bgcolor(color_right); // Yellow right of knob
				cmd(COLOR_RGB(255,255,0)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->D_term - var->min_D_term,range_D_term);
				cmd_coldstart();
			}
			else {
				cmd_coldstart();
				cmd_bgcolor(color_right);
				cmd(COLOR_RGB(140,140,140)); 
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->D_term - var->min_D_term,range_D_term);
			}
			cmd(COLOR_RGB(255,255,255));
			cmd_number(x_slider_position-x_num_adj,y_slider_position-y_num_adj,num_font_size,OPT_FLAT,var->min_D_term);
			cmd_number(x_slider_position+x_max_num_adj,y_slider_position - y_num_adj,num_font_size,OPT_FLAT,var->max_D_term);
			cmd(COLOR_RGB(255,0,0));
			cmd_number(x_slider_position+(slider_width/2),y_slider_position+y_num_adj,num_font_size,OPT_CENTER,var->D_term);
		}
		else if (menu[variable_pos].current_setting == ECU_I_SETTING) {
			if (selected == variable_pos) {
				cmd_fgcolor(0xff33ff); // Pink knob
				cmd_bgcolor(color_right); // Yellow right of knob
				cmd(COLOR_RGB(255,255,0)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->I_term - var->min_I_term,range_I_term);
				cmd_coldstart();
			}
			else {
				cmd_coldstart();
				cmd_bgcolor(color_right);
				cmd(COLOR_RGB(140,140,140)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->I_term - var->min_I_term,range_I_term);
			}
			cmd(COLOR_RGB(255,255,255));
			cmd_number(x_slider_position-x_num_adj,y_slider_position-y_num_adj,num_font_size,OPT_FLAT,var->min_I_term);
			cmd_number(x_slider_position+x_max_num_adj,y_slider_position - y_num_adj,num_font_size,OPT_FLAT,var->max_I_term);
			cmd(COLOR_RGB(255,0,0));
			cmd_number(x_slider_position+(slider_width/2),y_slider_position+y_num_adj,num_font_size,OPT_CENTER,var->I_term);
		}
		
		else if (menu[variable_pos].current_setting == ECU_LC_RT_SETTING) {
			if (selected == variable_pos) {
				cmd_fgcolor(0xff33ff); // Pink knob
				cmd_bgcolor(color_right); // Yellow right of knob
				cmd(COLOR_RGB(255,255,0)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->T_term - var->min_T_term,range_T_term);
				cmd_coldstart();
			}
			else {
				cmd_coldstart();
				cmd_bgcolor(color_right);
				cmd(COLOR_RGB(140,140,140)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->I_term - var->min_T_term,range_T_term);
			}
			cmd(COLOR_RGB(255,255,255));
			cmd_number(x_slider_position-x_num_adj,y_slider_position-y_num_adj,num_font_size,OPT_FLAT,var->min_T_term);
			cmd_number(x_slider_position+x_max_num_adj,y_slider_position - y_num_adj,num_font_size,OPT_FLAT,var->max_T_term);
			cmd_number(x_slider_position+(slider_width/2),y_slider_position- y_num_adj,num_font_size,OPT_CENTER,var->T_term);
		}

		else if (menu[variable_pos].current_setting == ECU_LC_INIT_TORQ_SETTING) {
			if (selected == variable_pos) {
				cmd_fgcolor(0xff33ff); // Pink knob
				cmd_bgcolor(color_right); // Yellow right of knob
				cmd(COLOR_RGB(255,255,0)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->R_term - var->min_R_term,range_R_term);
				cmd_coldstart();
			}
			else {
				cmd_coldstart();
				cmd_bgcolor(color_right);
				cmd(COLOR_RGB(140,140,140)); // Yellow left of knob
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,var->R_term - var->min_R_term,range_R_term);
			}
			cmd(COLOR_RGB(255,255,255));
			cmd_number(x_slider_position-x_num_adj,y_slider_position-y_num_adj,num_font_size,OPT_FLAT,var->min_R_term);
			cmd_number(x_slider_position+x_max_num_adj,y_slider_position - y_num_adj,num_font_size,OPT_FLAT,var->max_R_term);
			cmd_number(x_slider_position+(slider_width/2),y_slider_position,num_font_size,OPT_CENTER,var->R_term);
		}
		
		y_slider_position += vertical_slider_spacing;
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawDeviceStatusMenu(DeviceState *device_state) {
	uint8_t bar_len = 100;
	// Title
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(236, 20, 29, OPT_CENTER, "Device Status");
	
	// COLUMN 1
	switch (device_state->TRQ_0) {
		case ALIVE:
			cmd(COLOR_RGB(0,0,0));
			cmd_fgcolor(0x00FF00);
			cmd_button(70, 40, bar_len, 25, 26, OPT_CENTER, "TRQ0");
		break;
		case DEAD:
			cmd(COLOR_RGB(255,255,255));
			cmd_fgcolor(0xFF0000);
			cmd_button(70, 40, bar_len, 25, 26, OPT_CENTER, "TRQ0");
		break;
		case UNITIALIZED:
			cmd(COLOR_RGB(255,255,255));
			cmd_fgcolor(0xFFFF00);
			cmd_button(70, 40, bar_len, 25, 26, OPT_CENTER, "TRQ0");
		break;
	}
	switch (device_state->TRQ_1) {
		case ALIVE:
			cmd(COLOR_RGB(0,0,0));
			cmd_fgcolor(0x00FF00);
			cmd_button(70, 70, bar_len, 25, 26, OPT_CENTER, "TRQ1");
		break;
		case DEAD:
			cmd(COLOR_RGB(255,255,255));
			cmd_fgcolor(0xFF0000);
			cmd_button(70, 70, bar_len, 25, 26, OPT_CENTER, "TRQ1");
		break;
		case UNITIALIZED:
			cmd(COLOR_RGB(255,255,255));
			cmd_fgcolor(0xFFFF00);
			cmd_button(70, 70, bar_len, 25, 26, OPT_CENTER, "TRQ1");
		break;
	}
	if (device_state->IMU == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(70, 100, bar_len, 25, 26, OPT_CENTER, "IMU");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(70, 100, bar_len, 25, 26, OPT_CENTER, "IMU");
	}
	if (device_state->ECU == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(70, 130, bar_len, 25, 26, OPT_CENTER, "ECU");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(70, 130, bar_len, 25, 26, OPT_CENTER, "ECU");
	}
	
	if (device_state->TEL == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(70, 160, bar_len, 25, 26, OPT_CENTER, "TEL");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(70, 160, bar_len, 25, 26, OPT_CENTER, "TEL");
	}
	if (device_state->GLVBMS == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(70, 190, bar_len, 25, 26, OPT_CENTER, "GLVBMS");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(70, 190, bar_len, 25, 26, OPT_CENTER, "GLVBMS");
	}
	//END COLUMN 1
	// COLUMN 2
	if (device_state->INV == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(190, 70, bar_len, 25, 26, OPT_CENTER, "INV");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(190, 70, bar_len, 25, 26, OPT_CENTER, "INV");
	}
	
	if (device_state->STEER_POS == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(190, 100, bar_len, 25, 26, OPT_CENTER, "STEER POS");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(190, 100, bar_len, 25, 26, OPT_CENTER, "STEER POS");
	}
	
	if (device_state->IMD == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(190, 130, bar_len, 25, 26, OPT_CENTER, "IMD");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(190, 130, bar_len, 25, 26, OPT_CENTER, "IMD");
	}
	if (device_state->FAN == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(190, 160, bar_len, 25, 26, OPT_CENTER, "FAN");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(190, 160, bar_len, 25, 26, OPT_CENTER, "FAN");
	}
	
	if (device_state->BSPD == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(190, 190, bar_len, 25, 26, OPT_CENTER, "BSPD");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(190, 190, bar_len, 25, 26, OPT_CENTER, "BSPD");
	}
	
	// END COLUMN 2
	// COLUMN 3
	if (device_state->ADC_FL == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(310, 70, bar_len, 25, 26, OPT_CENTER, "ADC_FL");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(310, 70, bar_len, 25, 26, OPT_CENTER, "ADC_FL");
	}
	if (device_state->ADC_FR == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(310, 100, bar_len, 25, 26, OPT_CENTER, "ADC_FR");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(310, 100, bar_len, 25, 26, OPT_CENTER, "ADC_FR");
	}
	if (device_state->ADC_RR == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(310, 130, bar_len, 25, 26, OPT_CENTER, "ADC_RR");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(310, 130, bar_len, 25, 26, OPT_CENTER, "ADC_RR");
	}
	if (device_state->ADC_RL == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(310, 160, bar_len, 25, 26, OPT_CENTER, "ADC_RL");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(310, 160, bar_len, 25, 26, OPT_CENTER, "ADC_RL");
	}
	
	if (device_state->BMS == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(310, 190, bar_len, 25, 26, OPT_CENTER, "BMS");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(310, 190, bar_len, 25, 26, OPT_CENTER, "BMS");
	}
	// END COLUMN 3
	
	cmd(COLOR_RGB(255,255,255));
	cmd_coldstart(); // Reset to default values for objects. Color etc.
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}

static void DrawDriveEnableWarning(bool torque_pedal, bool brake_pedal, bool bms_discharge) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	if (torque_pedal && brake_pedal && bms_discharge) {
		cmd_text(240,100,29,OPT_CENTER,"REMOVE FOOT FROM TORQUE PEDAL BEFORE ENABLING DRIVE" );
		cmd_text(240,130,29,OPT_CENTER,"PRESS BRAKE PEDAL BEFORE ENABLING DRIVE" );
		cmd_text(240,160,29,OPT_CENTER,"WAIT FOR BMS DISCHARGE LIMIT BEFORE ENABLING DRIVE" );
		cmd_text(240,190,29,OPT_CENTER,"TURN DRIVE ENABLE SWITCH OFF AND PRESS ACKNOWLEDGE TO TRY AGAIN" );
	}
	else if (torque_pedal && !brake_pedal && !bms_discharge) {
		cmd_text(240,100,29,OPT_CENTER,"REMOVE FOOT FROM TORQUE PEDAL BEFORE ENABLING DRIVE" );
		cmd_text(240,190,29,OPT_CENTER,"TURN DRIVE ENABLE SWITCH OFF AND PRESS ACKNOWLEDGE TO TRY AGAIN" );
	}
	else if (brake_pedal && !torque_pedal && !bms_discharge) {
		cmd_text(240,130,29,OPT_CENTER,"PRESS BRAKE PEDAL BEFORE ENABLING DRIVE" );
		cmd_text(240,190,29,OPT_CENTER,"TURN DRIVE ENABLE SWITCH OFF AND PRESS ACKNOWLEDGE TO TRY AGAIN" );
	}
	else if (bms_discharge && !torque_pedal && !brake_pedal) {
		cmd_text(240,160,29,OPT_CENTER,"WAIT FOR BMS DISCHARGE LIMIT BEFORE ENABLING DRIVE" );
		cmd_text(240,190,29,OPT_CENTER,"TURN DRIVE ENABLE SWITCH OFF AND PRESS ACKNOWLEDGE TO TRY AGAIN" );
	}
	else if (bms_discharge && torque_pedal && !brake_pedal) {
		cmd_text(240,100,29,OPT_CENTER,"REMOVE FOOT FROM TORQUE PEDAL BEFORE ENABLING DRIVE" );
		cmd_text(240,160,29,OPT_CENTER,"WAIT FOR BMS DISCHARGE LIMIT BEFORE ENABLING DRIVE" );
		cmd_text(240,190,29,OPT_CENTER,"TURN DRIVE ENABLE SWITCH OFF AND PRESS ACKNOWLEDGE TO TRY AGAIN" );
	}
	else if (bms_discharge && brake_pedal && !torque_pedal) {
		cmd_text(240,130,29,OPT_CENTER,"PRESS BRAKE PEDAL BEFORE ENABLING DRIVE" );
		cmd_text(240,160,29,OPT_CENTER,"WAIT FOR BMS DISCHARGE LIMIT BEFORE ENABLING DRIVE" );
		cmd_text(240,190,29,OPT_CENTER,"TURN DRIVE ENABLE SWITCH OFF AND PRESS ACKNOWLEDGE TO TRY AGAIN" );
	}
	else if (brake_pedal && torque_pedal && !bms_discharge) {
		cmd_text(240,100,29,OPT_CENTER,"REMOVE FOOT FROM TORQUE PEDAL BEFORE ENABLING DRIVE" );
		cmd_text(240,130,29,OPT_CENTER,"PRESS BRAKE PEDAL BEFORE ENABLING DRIVE" );
		cmd_text(240,190,29,OPT_CENTER,"TURN DRIVE ENABLE SWITCH OFF AND PRESS ACKNOWLEDGE TO TRY AGAIN" );
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawLaunchControlProcedure() {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	if (car_state == LC_PROCEDURE) {
		cmd_text(5,20,27,OPT_FLAT, "LAUNCH CONTROL REQUEST ACCEPTED");
		//cmd_text(5,50,30,OPT_FLAT, "KEEP THE TORQUE PEDAL PUSHED IN");
		cmd_text(5,80,27,OPT_FLAT, "TO START COUNTDOWN PRESS ACKNOWLEDGE BUTTON");
		cmd_text(5,130,27,OPT_FLAT, "TO ABORT LAUNCH CONTROL PUSH THE BRAKES IN");
	}
	else if (car_state == LC_COUNTDOWN) {
		if (lc_timer_count == 1) {
			cmd_text(240,130,31,OPT_CENTER,"5");
		}
		else if (lc_timer_count == 2) {
			cmd_text(240,130,31,OPT_CENTER,"4");
		}
		else if(lc_timer_count == 3) {
			cmd_text(240,130,31,OPT_CENTER,"3");
		}
		else if (lc_timer_count == 4) {
			cmd_text(240,130,31,OPT_CENTER,"2");
		}
		else if (lc_timer_count == 5) {
			cmd_text(240,130,31,OPT_CENTER,"1");
		}
	}
	else if (car_state == LC_ARMED) {
		cmd_text(240,130,27,OPT_CENTER, "LAUNCH CONTROL IS ARMED. PUSH TORQUE PEDAL TO LAUNCH");
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawTorqueCalibrationScreen(ConfirmationMsgs *conf_msg) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	switch (trq_calib_state) {
		case TRQ_CALIBRATION_OFF:
			cmd_text(5,20,28,OPT_FLAT,"1: PUSH AND HOLD THE TORQUE PEDAL IN");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			cmd_text(5,180,30,OPT_FLAT,"3: WAIT FOR CONFIRMATION");
			break;
		case TRQ_CALIBRATION_MAX_CONFIRMED:
			cmd_text(5,20,30,OPT_FLAT ,"1: RELEASE THE TORQUE PEDAL");
			cmd_text(5,60,30,OPT_FLAT ,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,100,30,OPT_FLAT,"   BUTTON");
			cmd_text(5,140,30,OPT_FLAT,"3: WAIT FOR CONFIRMATION");
			break;
		case TRQ_CALIBRATION_MIN_CONFIRMED:
			cmd(COLOR_RGB(0,255,0));
			cmd_text(5,20,30,OPT_FLAT,"1: TORQUE PEDAL CALIBRATION");
			cmd_text(5,60,30,OPT_FLAT,"    WAS SUCCESSFUL");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
		case TRQ_FAIL_BOTH_CH:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(5,20,30,OPT_FLAT,"1: BOTH SENSORS FAILED");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
		case TRQ_FAIL_CH0:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(5,20,30,OPT_FLAT,"1: ENCODER ON CAN 0 FAILED");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
		case TRQ_FAIL_CH1:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(5,20,30,OPT_FLAT,"1: ENCODER ON CAN 1 FAILED");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
		case TRQ_TIMEOUT_BOTH_CH:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(5,20,26,OPT_FLAT,"1: BOTH ENCODERS TIMED OUT");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
		case TRQ_TIMEOUT_CH0:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(5,20,28,OPT_FLAT,"1: ENCODER ON CAN 0 TIMED OUT");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
		case TRQ_TIMEOUT_CH1:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(5,20,28,OPT_FLAT,"1: ENCODER ON CAN 1 TIMED OUT");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	
}
static void DrawSteerCalibScreen() {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	switch (steer_calib_state) {
		case STEER_C_OFF:
			cmd_text(5,20,27,OPT_FLAT,"1: TURN STEERING WHEEL TO THE LEFT");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			cmd_text(5,180,30,OPT_FLAT,"3: WAIT FOR CONFIRMATION");
			break;
		case STEER_C_LEFT_CONFIRMED:
			cmd_text(5,20,27,OPT_FLAT ,"1: TURN STEERING WHEEL TO THE RIGHT");
			cmd_text(5,60,30,OPT_FLAT ,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,100,30,OPT_FLAT,"   BUTTON");
			cmd_text(5,140,30,OPT_FLAT,"3: WAIT FOR CONFIRMATION");
			break;
		case STEER_C_RIGHT_CONFIRMED:
			cmd(COLOR_RGB(0,255,0));
			cmd_text(5,20,30,OPT_FLAT,"1: STEERING CALIBRATION");
			cmd_text(5,60,30,OPT_FLAT,"    WAS SUCCESSFUL");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
		case STEER_C_FAIL:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(5,20,30,OPT_FLAT,"1: STEERING CALIBRATION ");
			cmd_text(5,60,30,OPT_FLAT,"    FAILED");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
		case STEER_C_TIMEOUT:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(5,20,30,OPT_FLAT,"1: STEERING CALIBRATION ");
			cmd_text(5,60,30,OPT_FLAT,"    TIMED OUT");
			cmd_text(5,100,30,OPT_FLAT,"2: PUSH THE ACKNOWLEDGE");
			cmd_text(5,140,30,OPT_FLAT,"    BUTTON");
			break;
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	
}
static void DrawHighVoltageSymbol() {
	//cmd(CMD_COLDSTART);
	cmd(BITMAP_SOURCE(0));
	cmd(BITMAP_LAYOUT(RGB565, 140, 70));
	cmd(BITMAP_SIZE(NEAREST, BORDER, BORDER, 70, 59));
	cmd(BEGIN(BITMAPS));
	cmd(VERTEX2II(390,210,0,0));
	//cmd(VERTEX2II(100-16, 160-16, 0, 0));
}


static void DrawDataloggerInterface() {
	uint8_t pos = selected - menu[selected].position; // First position in current menu
	uint8_t end_position  = pos + menu[pos].num_menupoints - 1;  // Last position in current menu
	uint16_t y_position = 20;
	uint16_t x_position = 10;
		
	uint16_t x_position_text = 120;
	uint16_t y_position_text = 50;
	
	uint16_t button_width = 230;
	uint16_t button_heigth = 60;
	uint8_t vert_spacing = 62;
		
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen

	//pos = pos + 1;
	cmd(COLOR_RGB(255,255,255));
	for (pos; pos <= (end_position); pos ++) {
		if ( pos == selected) {
			cmd(BEGIN(RECTS));
			cmd(COLOR_RGB(255,255,20)); 
			cmd(LINE_WIDTH(16*5));
			cmd(VERTEX2F(x_position*16,y_position*16));
			cmd(VERTEX2F((x_position+button_width)*16, (y_position+button_heigth)*16));
			cmd(COLOR_RGB(0,0,0));
			cmd_text(x_position_text,y_position_text,28,OPT_CENTER,menu[pos].text);
	
			cmd_coldstart();
		}
		else {
			cmd(COLOR_RGB(255,255,255));
			cmd_text(x_position_text,y_position_text,28,OPT_CENTER,menu[pos].text);
		}
		y_position += vert_spacing;
		y_position_text += vert_spacing;
	}
	uint8_t font_size = 28;
	uint32_t x_position_status_text = 260;
	uint32_t x_pos_free_space = 420;
	uint32_t datalogger_write_speed = 0;
	cmd(COLOR_RGB(255,255,255));
	switch (dataloggerState) {
		case DATALOGGER_IDLE:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(x_position_status_text,20,font_size,OPT_FLAT,"NOT LOGGING");
			cmd(COLOR_RGB(255,255,255));
			//cmd_text(x_position_status_text,80,font_size,OPT_FLAT,"W [KB]: 0");
			cmd_text(x_position_status_text,50,font_size,OPT_FLAT,"FILES:");
			cmd_number(390,50,font_size,OPT_FLAT, number_of_files_sdcard);
			cmd_text(x_position_status_text,200,font_size,OPT_FLAT,"USB NOT CONNECTED");
		break;

		case DATALOGGER_LOGGING:
			cmd(COLOR_RGB(0,255,0));
			cmd_text(x_position_status_text,20,font_size,OPT_FLAT,"LOGGING TO FILE");
			cmd(COLOR_RGB(255,255,255));
			cmd_text(x_position_status_text,50,font_size,OPT_FLAT,"FILES:");
			cmd_number(390,50,font_size,OPT_FLAT, number_of_files_sdcard);
			
			cmd_text(x_position_status_text,80,font_size,OPT_FLAT,"W [KB]:");
			cmd_number(400,80,font_size,OPT_FLAT,file_size_byte_counter*16);
			
			datalogger_write_speed = BUFFER_LENGTH/(stop_time-start_time);
			cmd_text(x_position_status_text,110,font_size,OPT_FLAT,"W [KB/s]:");
			cmd_number(400,110,font_size,OPT_FLAT,datalogger_write_speed);
				
			cmd_text(x_position_status_text,200,font_size,OPT_FLAT,"USB NOT CONNECTED");
		break;
		
		case DATALOGGER_USB_CONNECTED:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(x_position_status_text,20,font_size,OPT_FLAT,"NOT LOGGING");
			cmd(COLOR_RGB(255,255,255));
			cmd_text(x_position_status_text,50,font_size,OPT_FLAT,"FILES:");
			cmd_number(390,50,font_size,OPT_FLAT, number_of_files_sdcard);
			//cmd_text(x_position_status_text,80,font_size,OPT_FLAT,"W [KB]: 0");
			cmd(COLOR_RGB(0,255,0));
			cmd_text(x_position_status_text,200,font_size,OPT_FLAT,"USB CONNECTED");
		break; 
		
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}

static void DrawFloat(uint16_t x, uint16_t y, uint8_t font_size, float f) {
	int integer_part = (int) f;
	int fractional_part = (int) ( (f- integer_part)*10);
	cmd_number(x,y,font_size,OPT_CENTER,integer_part);
	cmd_text(x+7,y,font_size,OPT_CENTER,".");
	cmd_number(x+13,y,font_size,OPT_CENTER,fractional_part);
}

static void DrawPresetProcedure() {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	switch (presetProcedureState) {
		case PRESET_PROCEDURE_FINISHED:
			cmd_text(240,130,28,OPT_CENTER,"PRESETS SUCESSFULLY TRANSFERRED TO THE ECU");
		break;
		case PRESET_PROCEDURE_FAILED:
			cmd(COLOR_RGB(255,0,0));;
			cmd_text(240,130,28,OPT_CENTER,"PRESET PROCEDURE FAILED");
		break;
	}
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}

//***********************************************************************************
//--------------------------SLIDER VARIABLE UPDATE FUNCTIONS-----------------------//
//***********************************************************************************


// static void adjustParameter(EAdjustmentParameter parameter_type, ERotary_direction dir, Variables *var) {
// 	switch (parameter_type) {
// 		case TORQUE_SETTING:
// 			if (dir == CW && var->torque <=(var->max_torque - step)) {
// 				var->torque += step;
// 			}
// 			else if (dir == CCW && var->torque >=(var->min_torque+step)) {
// 				var->torque -= step;
// 			}
// 		
// 	}
// }

static void adjustParameters(ERotary_direction dir, Variables *var) {
	switch (menu[selected].current_setting) {
		case TORQUE_SETTING:
			if ( (dir == CW) && ( (var->torque + StepSizeVar.torque) <= var->max_torque )  ) {
				var->torque += StepSizeVar.torque;
			}
			else if ( (dir == CCW) && ( (var->torque - StepSizeVar.torque)  >= var->min_torque) ) {
				var->torque -= StepSizeVar.torque;
			}
		break;
		case ECU_P_SETTING:
			if ( (dir == CW) && ( (var->P_term + StepSizeVar.p_term) <= var->max_P_term )  ) {
				var->P_term += StepSizeVar.p_term;
			}
			else if ( (dir == CCW) && ( (var->P_term - StepSizeVar.p_term)  >= var->min_P_term) ) {
				var->P_term -= StepSizeVar.p_term;
			}
		break;
		case ECU_I_SETTING:
			if ( (dir == CW) && ( (var->I_term + StepSizeVar.i_term) <= var->max_I_term )  ) {
				var->I_term += StepSizeVar.i_term;
			}
			else if ( (dir == CCW) && ( (var->I_term - StepSizeVar.i_term)  >= var->min_I_term) ) {
				var->I_term -= StepSizeVar.i_term;
			}
		break;
	}
}

static void slider_torque_update(ERotary_direction dir, Variables *var) {
	uint8_t step = 25;
	if (dir == CW && var->torque <=(var->max_torque - step)) {
		var->torque += step;
	}
	else if (dir == CCW && var->torque >=(var->min_torque+step)) {
		var->torque -= step;
	}
}
static void slider_P_term_update(ERotary_direction dir, Variables *var) {
	uint8_t step = 1;
	if (dir == CW && var->P_term <=(var->max_P_term - step)) {
		var->P_term += step;
	}
	else if (dir == CCW && var->P_term >=(var->min_P_term+step)) {
		var->P_term -= step;
	}
}
static void slider_I_term_update(ERotary_direction dir, Variables *var) {
	uint8_t step = 1;
	if (dir == CW && var->I_term <=(var->max_I_term - step)) {
		var->I_term += step;
	}
	else if (dir == CCW && var->I_term >=(var->min_I_term+step)) {
		var->I_term -= step;
	}
}
static void slider_D_term_update(ERotary_direction dir, Variables *var) {
	uint8_t step = 1;
	if (dir == CW && var->D_term <=(var->max_D_term - step)) {
		var->D_term += step;
	}
	else if (dir == CCW && var->D_term >=(var->min_D_term+step)) {
		var->D_term -= step;
	}
}
static void slider_R_term_update(ERotary_direction dir, Variables *var) {
	uint8_t step = 1;
	if (dir == CW && var->R_term <=(var->max_R_term - step)) {
		var->R_term += step;
	}
	else if (dir == CCW && var->R_term >=(var->min_R_term+step)) {
		var->R_term -= step;
	}
}
static void slider_T_term_update(ERotary_direction dir, Variables *var) {
	uint8_t step = 1;
	if (dir == CW && var->T_term <=(var->max_T_term - step)) {
		var->T_term += step;
	}
	else if (dir == CCW && var->T_term >=(var->min_T_term+step)) {
		var->T_term -= step;
	}
}


//***********************************************************************************
//------------------------------------DATALOGGER FUNCTIONS-------------------------//
//***********************************************************************************


static void startLoggingCommand() {
	static enum EDataloggerCommands command = START_LOGGING;
	xQueueSendToBack(xDataloggerCommandQueue,&command,0);
}
static void closeFileCommand() {
	static enum EDataloggerCommands command = CLOSE_FILE;
	xQueueSendToBack(xDataloggerCommandQueue,&command,0);
}
static void deleteAllFilesCommand() {
	static enum EDataloggerCommands command = DELETE_ALL_FILES;
	xQueueSendToBack(xDataloggerCommandQueue,&command,0);
}
static void slider_preallocateAmount() {
	
}