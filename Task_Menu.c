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
#include <math.h>

//**********************************************************************************//
//------------------------------------ENUMS FOR THIS C FILE-------------------------//
//**********************************************************************************//
typedef enum {PEDAL_IN, PEDAL_OUT} EPedalPosition;
typedef enum {TRACTION_CONTROL_ON,TRACTION_CONTROL_OFF} ETractionControlStatus;

//**********************************************************************************//
//------------------------------------THE MAIN DASH FUNCTIONS-----------------------//
//**********************************************************************************//
static void changeCarState(ConfirmationMsgs *confMsg, StatusMsg *status, SensorPhysicalValues *sensorPhysicalValue);
static void dashboardControlFunction(Buttons *btn, ModuleError *error, SensorValues *sensorValue,StatusMsg *status,
ConfirmationMsgs *confMsg,DeviceState *deviceState, ParameterValue *parameter, SensorPhysicalValues *sensorPhysicalValue);
static void HandleButtonActions(Buttons *btn, SensorPhysicalValues *sensorPhysicalValue,DeviceState *deviceState,
ParameterValue *parameter, ModuleError *error,ConfirmationMsgs *confMsg);
static void NavigateMenu(DeviceState *deviceState, ParameterValue *parameter, ModuleError *error, SensorPhysicalValues *sensorPhysicalValue);

static void getDashMessages(ParameterValue *parameter, ConfirmationMsgs *confMsg, ModuleError *error, SensorValues *sensorValue,StatusMsg *status,SensorPhysicalValues *sensorPhysicalValue);

static void LEDHandler(SensorPhysicalValues *sensorPhysicalValue, ModuleError *error,DeviceState *devices);
//***********************************************************************************
//------------------------------------MENU HELPER FUNCTIONS------------------------//
//***********************************************************************************
static void can_freeRTOSSendMessage(Can *can,struct CanMessage message);

static void setParameterBasedOnConfirmation(ParameterValue *parameter);
static void clearAllButtons();

static bool checkForError(ModuleError *error);
static void HandleErrors(ModuleError *error);

static EPedalPosition getTorquePedalPosition(SensorPhysicalValues *sensorPhysicalValue);
static EPedalPosition getBrakePedalPosition(SensorPhysicalValues *sensorPhysicalValue);

static void calibrateSteering(ConfirmationMsgs *confMsg,bool ack_pressed);
static void calibrateTorquePedal(ConfirmationMsgs *confMsg,bool ack_pressed);
static bool checkDeviceStatus(DeviceState *devices);

static void presetProcedureHandling(bool ackPressed,ConfirmationMsgs *confMsg);
//---------------INIT THE STRUCTURES USED---------------//
static void initSensorRealValueStruct(SensorPhysicalValues *sensorReal);
static void initSensorValueStruct(SensorValues *sensorValue);
static void initStatusStruct(StatusMsg *status);
static void initConfirmationMessagesStruct(ConfirmationMsgs *confMsg);
static void initErrorMessagesStruct(ModuleError *error);
static void initParameterStruct(ParameterValue *parameter);
//--------------FUNCTIONS TO ADJUST VARIABLES---------------//
static void adjustParameters(ERotary_direction dir, ParameterValue *parameter);
//-------------------TIMERS------------------//
static void vRTDSCallback(TimerHandle_t xTimer);
static void vCalibrationTimerCallback(TimerHandle_t xTimer);
static void vLcTimerCallback (TimerHandle_t lcTimer);
static void vVarConfTimerCallback(TimerHandle_t xTimer);
static void vMenuUpdateCallback(TimerHandle_t pxTimer);
static void createAndStartMenuUpdateTimers();
static void vTSLedTimerCallback(TimerHandle_t pxtimer);
static void iAmAliveTimerCallback(TimerHandle_t pxTimer);

//***********************************************************************************
//------------------------------------CALCULATION FUNCTIONS-------------------------//
//***********************************************************************************
static void sensorValueToRealValue(SensorValues *sensorValue,SensorPhysicalValues *sensorPhysicalValue);

//***********************************************************************************
//------------------------------------DRAWING FUNCTIONS----------------------------//
//***********************************************************************************
static void DrawMainScreen(SensorPhysicalValues *sensor,DeviceState *devices);
static void DrawLowVoltageBattery(uint8_t battery_left_percent);
static void DrawHighVoltageBattery(uint8_t battery_left_percent);
static void DrawHighVoltageSymbol();

static void DrawSpeedScreen(SensorPhysicalValues *sensorPhysicalValue);
static void DrawSystemMonitorScreen(ModuleError *error,SensorPhysicalValues *val);
static void DrawBatteryInfoScreen(SensorPhysicalValues *tempvolt);
static void DrawSensorInformationScreen(SensorPhysicalValues *sensorData);
static void DrawTSStatusScreen(SensorPhysicalValues *tempvolt, ModuleError *Error);
static void DrawMainMenu();
static void DrawMainMenuDown();
static void DrawAdjustmentMenu();
static void DrawECUAdjustmentScreen(ParameterValue *parameter);
static void DrawFANControlScreen(ParameterValue *parameter);
static void DrawDeviceStatusMenu(DeviceState *deviceState);

static void DrawTorqueCalibrationScreen(ConfirmationMsgs *confMsg);
static void DrawSteerCalibScreen();
static void DrawDriveEnableWarning();
static void DrawLaunchControlProcedure();
static void DrawDataloggerInterface();

static void DrawIMUScreen(SensorPhysicalValues *sensorData);
static void Draw4DigitFloat(uint16_t x, uint16_t y, uint8_t font_size, float f);
static void Draw3DigitFloat(uint16_t x, uint16_t y, uint8_t font_size, float f);
static void DrawFloat(uint16_t x, uint16_t y, uint8_t font_size, float f);
static void DrawPresetMenu();
static void DrawPresetProcedure();
static void DrawPresetConfirmation();
static void DrawParallellogramMainScreen();
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
const char menu_205[] = "FAN CONTROL";							// 9
const char menu_206[] = "IMU DATA";								// 10
const char menu_207[] = "DATALOGGER";							// 11
const char menu_208[] = "PRESETS";								// 12

const char menu_300[] = "DEVICE STATUS";						// 13
const char menu_400[] = "KERS settings";						// 14
const char menu_500[] = "Trq. Enc. Calibration";				// 15

const char menu_600[] = "ECU OPTIONS";							// 16			
const char menu_601[] = "MAX TORQUE";							// 17
const char menu_602[] = "KERS";								// 18
const char menu_603[] = "TRACTION CONTROL";								// 19
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
	{menu_000, 1,	42,	1,  2,  5,	0,  0,	MAIN_SCREEN,		NO_SETTING,					0,0 },						//0
		
	{menu_001, 1,	0,	1,	1,	1,	1,  0,  SPEED,				NO_SETTING,					0,0 },						//1	
	{menu_002, 1,	2,	2,	3,	0,	2,  1,  SYSTEM_MONITOR,		NO_SETTING,					0,0 },						//2
	{menu_003, 1,	3,	3,	41,	2,	3,  1,	TEMP_VOLT,			NO_SETTING,					0,0 },						//3
			
	{menu_200, 9,	4,	4,	4,	4,	4,  0,	MAIN_MENU,			NO_SETTING,					0,0},						//4  main menu
	{menu_201, 9,	5,	6,	0,	9,  14,	1,  MAIN_MENU,			NO_SETTING,					0,0},						//5	 Device Status  
	{menu_202, 9,	5,	7,	0,	10, 19,	2,  MAIN_MENU,			NO_SETTING,					0,0},						//6  Steer calib
	{menu_203, 9,	6,	8,	0,	11, 20,	3,  MAIN_MENU,			NO_SETTING,					0,0},						//7	 Torque Pedal Calibration
	{menu_204, 9,	7,	43,	0,	12, 21,	4,  MAIN_MENU,			NO_SETTING,					0,0},						//8  ECU Options
		
	{menu_205, 9,	8,	10,	5,	9,  51,	5,  MAIN_MENU,			NO_SETTING,					0,0},						//9  FAN CONTROL
	{menu_206, 9,	9,	11,	6,	10, 40,	6,  MAIN_MENU,			NO_SETTING,					0,0},						//10 IMU DATA
	{menu_207, 9,	10,	12,	7,	11, 25,	7,  MAIN_MENU,			NO_SETTING,					0,0},						//11 Datalogger
	{menu_208, 9,	11,	43,	8,	12, 29,	8,  MAIN_MENU,			NO_SETTING,					0,0},						//12 Preset parameters
	
	{menu_400, 1,	10,	10,	6,	10, 10,	1,  KERS_OPTION,		KERS_SETTING,				0,0},						//13  KERS adjustment screen
		
	{menu_300, 1,	14,	14,	5,	14, 14,	1,  DEVICE_STATUS,		NO_SETTING,					0,0},						//14  Device status Screen
		
	{menu_900, 1,	15,	15,	15,	15,  15, 0, PERSISTENT_MSG,		NO_SETTING,					0,0},						//15 Drive enable message
	{menu_901, 1,	16,	16,	16,	16,  16, 0, LC_HANDLER,			NO_SETTING,					0,0},						//16 LC handler
	{menu_902, 1,	17,	17,	17,	17,  17, 0, ERROR_HANDLER,		NO_SETTING,					0,0},						//17 Error handler
	{"snake",  1,	18,	18,	18,	18,  18, 0, SNAKE_GAME,			NO_SETTING,					0,0},						//18 Play snake
	{"SteerCal",1,	19,	19,	19,	19,  19, 0, STEER_CALIB,		NO_SETTING,					0,0},						//19 Steer calib
	{menu_500, 1,	20,	20,	20,	20,	 20, 0, TRQ_CALIB,			NO_SETTING,					0,0},						//20 Torque calibration screen
		
	{menu_601, 3,	21,	22,	8,	21, 21,	0,  ECU_OPTIONS,		TORQUE_SETTING,				adjustParameters,0},		//21 Max torque slider
	{menu_602, 3,	21,	23,	8,	22, 22,	1,  ECU_OPTIONS,		KERS_SETTING,				adjustParameters,0},		//22 KERS
	{menu_603, 3,	22,	23,	8,	23, 23,	2,  ECU_OPTIONS,		TRACTION_CONTROL_SETTING,	adjustParameters,0},		//23 TRACTION CONTROL ON / OFF
	
	{menu_604, 0,	23,	24,	8,	24, 24,	3,  ECU_OPTIONS,		NO_SETTING,					0,0},						//24 EMPTY
	
	{menu_700, 3,	25,	26,	11,	25, 25,	0,  DL_OPTIONS,			NO_SETTING,				0,startLoggingCommand},			//25 Create file
	{menu_701, 3,	25,	27,	11,	26, 26,	1,  DL_OPTIONS,			NO_SETTING,				0,closeFileCommand},			//26 Start logging
	{menu_702, 3,	26,	27, 11,	27, 27,	2,  DL_OPTIONS,			NO_SETTING,				0,deleteAllFilesCommand},		//27 Close file
		
	{"LOCKED", 1,	28,	28, 28,	28, 28,	0,  LOCKED_SEL,			NO_SETTING,				0,0},							//28 Locked menu position
		
	{"VERYWET10", 8,29,	30, 12,	33, 38,	0,  PRESET_SEL,			PRESET_1_SETTING,		0,0},							//29 Preset option
	{"VERYWET20", 8,29,	31, 12,	34, 38,	1,  PRESET_SEL,			PRESET_2_SETTING,		0,0},							//30 Preset option
	{"WET10",	  8,30,	32, 12,	35, 38,	2,  PRESET_SEL,			PRESET_3_SETTING,		0,0},							//31 Preset option
	{"VWET20",	  8,31,	33, 12,	36, 38,	3,  PRESET_SEL,			PRESET_4_SETTING,		0,0},							//32 Preset option
	{"DRY10",     8,32,	34, 29,	33, 38,	4,  PRESET_SEL,			PRESET_5_SETTING,		0,0},							//33 Preset option
	{"DRY20",     8,33,	35, 30,	34, 38,	5,  PRESET_SEL,			PRESET_6_SETTING,		0,0},							//34 Preset option
	{"DRY25",     8,34,	36, 31,	35, 38,	6,	PRESET_SEL,			PRESET_7_SETTING,		0,0},							//35 Preset option
	{"GEN",       8,35,	36, 32,	36, 38,	7,	PRESET_SEL,			PRESET_8_SETTING,		0,0},							//36 Preset option
		
	{"YES",       2,37,	37, 37,	38, 39,	0,	PRESET_CONFIRM,		CONFIRM_YES,			0,0},							//37 Preset Yees
	{"NO",        2,38,	38, 37,	38, 29,	1,	PRESET_CONFIRM,		CONFIRM_NO,				0,0},							//38 Preset 
		
	{"PRELOCK",   1,39,	39, 39,	39, 39,	0,	PRESET_PROCEDURE,	NO_SETTING,				0,0},							//39 Preset option
	{"IMU INFO",  1,40,	40, 9,	40, 40,	0,	IMU_INFORMATION,	NO_SETTING,				0,0},							//40 IMU Information screen
	{"SENSOR DATA",1,41,41, 41,	 3, 41,	0,	SENSOR_DATA,		NO_SETTING,				0,0},							//41 Sensor Data
	{"TS STATUS" ,1,42,  0, 42,	42, 42,	0,	TS_STATUS,			NO_SETTING,				0,0},							//42 TS STATUS
		
	{"SNAKE", 8,	8,	44,	0,	47, 18,	0,  MAIN_MENU_DOWN,		NO_SETTING,					0,0},						//43 SNAKE
	{"EMPTY", 8,	43,	45,	0,	48, 44,	1,  MAIN_MENU_DOWN,		NO_SETTING,					0,0},						//44 EMPTY
	{"EMPTY", 8,	44,	46,	0,	49, 45,	2,  MAIN_MENU_DOWN,		NO_SETTING,					0,0},						//45 EMPTY
	{"EMPTY", 8,	45,	46,	0,	50, 46,	3,  MAIN_MENU_DOWN,		NO_SETTING,					0,0},						//46 EMPTY
		
	{"EMPTY", 8,	12,	48,	43,	47, 47,	4,  MAIN_MENU_DOWN,		NO_SETTING,					0,0},						//47 EMPTY
	{"EMPTY", 8,	47,	49,	44,	48, 48,	5,  MAIN_MENU_DOWN,		NO_SETTING,					0,0},						//48 EMPTY
	{"EMPTY", 8,	48,	50,	45,	49, 49,	6,  MAIN_MENU_DOWN,		NO_SETTING,					0,0},						//49 EMPTY
	{"EMPTY", 8,	49,	50,	46,	50, 50,	7,  MAIN_MENU_DOWN,		NO_SETTING,					0,0},						//50 EMPTY
		
	{"RADIATOR FAN",		5,	51,	52,	9,	51, 51,	0,  FAN_OPTIONS,	RADIATOR_FAN_SETTING,		adjustParameters,0},	//51 RADIATOR FAN
	{"MONO FAN",			5,	51,	53,	9,	52, 52,	1,  FAN_OPTIONS,	MONO_FAN_SETTING,			adjustParameters,0},	//52 MONO FAN SLIDER
	{"BATTERY FAN",			5,	52,	54,	9,	53, 53,	2,  FAN_OPTIONS,	BATTERY_FAN_SETTING,		adjustParameters,0},	//53 BATTERY FAN SLIDER
	{"TOGGLE ALL FANS",		5,	53,	55,	9,	54, 54,	3,  FAN_OPTIONS,	ALL_FAN_SETTING,			adjustParameters,0},	//54 TOGGLE ALL FANS
	{"TOGGLE PUMP",			5,	54,	55,	9,	55, 55,	4,  FAN_OPTIONS,	PUMP_SETTING,				adjustParameters,0}		//55 TOGGLE PUMP
		
		
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

#define NUM_MENUS_UPDATE 2 // Number of menus to specifiy a certain update frequency for
#define RTDS_DURATION_MS 2300/portTICK_RATE_MS
#define WATCHDOG_RESET_COMMAND  ( (0xA5 << 24) | (1<<0)) // Command to write to WDT CR register to reset the counter

// Identifiers for parameters sent to ECU
#define P_TERM			0x01
#define I_TERM			0x02
#define D_TERM			0x03
#define MAX_MIN_VALUE	0x04
#define MAX_DECREASE	0x05
#define DESIRED_SLIP	0x06
#define MAX_INTEGRAL	0x07
#define MAX_TORQUE		0xF0
#define KERS_ADJUST		0xF1
#define SELECTED_PRESET 0xF2
//***********************************************************************************
//-------------------------SEMAPHORE, TIMERS AND QUEUES----------------------------//
//***********************************************************************************
SemaphoreHandle_t		xButtonStruct	= NULL;
SemaphoreHandle_t		spi_semaphore	= NULL;
SemaphoreHandle_t		can_mutex_0		= NULL;
SemaphoreHandle_t		can_mutex_1		= NULL;
static TimerHandle_t	TSLedTimer;
static TimerHandle_t	RTDSTimer;
static TimerHandle_t	LcTimer;
static TimerHandle_t	calibrationTimer;
static TimerHandle_t	parameterConfTimer;
static TimerHandle_t	timerMenuUpdate[NUM_MENUS_UPDATE];
static TimerHandle_t	iAmAliveTimer;
static bool				trq_calib_timed_out				= false;
static bool				steer_calib_timed_out			= false;
static bool				parameter_confirmation_timed_out = false;
static uint8_t			lc_timer_count					= 0; // Countdown timer for launch control
//***********************************************************************************
//---------------------------FILE GLOBAL STATE VARIABLES------------------------------//
//***********************************************************************************
static ECarState					carState					= TRACTIVE_SYSTEM_OFF;
static ESteerCalibState				steeringCalibrationState	= STEER_C_OFF;
static ETorquePedalCalibrationState torquePedalCalibrationState	= TRQ_CALIBRATION_OFF;
static EPresetStates				presetProcedureState		= PRESET_PROCEDURE_OFF;
static ETractionControlStatus		tractionControlState		= TRACTION_CONTROL_OFF;

//***********************************************************************************
//------------------------------------GLOBAL STRUCTS-------------------------------//
//***********************************************************************************
typedef struct menuUpdateFrequency { // Private global for this source
	bool update_menu;
	bool update_procedures;
} menuUpdateStatus;
menuUpdateStatus menuUpdate = {
	.update_menu = false,
	.update_procedures = false
};
Buttons btn = {
	.btn_type				= NONE_BTN,
	.navigation				= NAV_DEFAULT,
	.rotary_ccw				= false,
	.rotary_cw				= false,
	.unhandledButtonAction  = false
};
DeviceState deviceState = {
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
typedef enum {STEP_ONE = 1, STEP_TEN=10} EStepSize ;
struct StepSizeForVariables {
	EStepSize kers;
	EStepSize torque;
	EStepSize radiator_fan;
	EStepSize mono_fan;
	EStepSize battery_fan;
	};
static struct StepSizeForVariables StepSizeVar = {
	.kers = STEP_ONE,
	.torque = STEP_TEN,
	.radiator_fan = STEP_TEN,
	.battery_fan = STEP_TEN,
	.mono_fan    = STEP_TEN
	};
	
static struct presetParameterStruct presetParameters;

// Identify which preset that has been chosen
EAdjustmentParameter presetSetting;

//***********************************************************************************
//--------------------------------FILE GLOBALS-------------------------------------//
//***********************************************************************************
static uint8_t selected_preset_file = 100;
static bool RTDS_finished_playing = false;
static uint8_t prev_selected = 0;
static bool send_alive = false;

//Legge til tilstand til BSPD IMD og bMS i en meny
//***********************************************************************************
//------------------------------------THE MAIN DASH FUNCTIONS-----------------------//
//***********************************************************************************
void dashTask() {
	TickType_t xLastWakeTime;
	RTDSTimer				= xTimerCreate("RTDSTimer",	RTDS_DURATION_MS,		pdFALSE,	0,			vRTDSCallback);
	LcTimer					= xTimerCreate("lcTimer",	1000/portTICK_RATE_MS,	pdTRUE,		(void *) 1,	vLcTimerCallback);
	calibrationTimer		= xTimerCreate("CalibTimer",3000/portTICK_RATE_MS,	pdFALSE,	(void *) 1,	vCalibrationTimerCallback);
	parameterConfTimer		= xTimerCreate("parTimer",	1000/portTICK_RATE_MS,	pdFALSE,	0,			vVarConfTimerCallback);
	TSLedTimer				= xTimerCreate("TSLed",		300/portTICK_RATE_MS,	pdTRUE,		0,			vTSLedTimerCallback);	
	iAmAliveTimer			= xTimerCreate("iAmAlive",	1000/portTICK_RATE_MS,	pdTRUE,		0,			iAmAliveTimerCallback);
	xTimerReset(iAmAliveTimer,0);
	createAndStartMenuUpdateTimers();
	
	//Init states
	SensorValues			sensorValue;
	SensorPhysicalValues	sensorPhysicalValue;
	StatusMsg				status;
	ConfirmationMsgs		confMsg;
	ModuleError				error;
	ParameterValue			parameter;

	initSensorRealValueStruct(&sensorPhysicalValue);
	initSensorValueStruct(&sensorValue);
	initStatusStruct(&status);
	initConfirmationMessagesStruct(&confMsg);
	initErrorMessagesStruct(&error);
	initParameterStruct(&parameter);
	//ECarState carState = TRACTIVE_SYSTEM_OFF;
	
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
	uint32_t ram_offset=0;
	for(int i=0; i<4130; i++){
		wr16(ram_offset, high_voltage_symbol[i]);
		ram_offset +=2;
	}
	
	while(1) {
		if (send_alive) {
			//can_sendMessage(CAN0,IAmAlive);
			can_freeRTOSSendMessage(CAN0,IAmAlive);
			can_freeRTOSSendMessage(CAN1,IAmAlive);
			send_alive = false;
		}
		// The delay for this task is given by the wait time specified in the queuereceive function
		// in the getdashmessages function. This is done to ensure that as long as there are 
		// messages in the queue the menu task will service them. The update frequency of the visuals 
		// is controlled by software timers to ensure that no processing power is wasted on spi comms.
		WDT->WDT_CR = WATCHDOG_RESET_COMMAND; // Restart watchdog timer
		//Get relevant CAN messages from the specified freeRTOS queue
		getDashMessages(&parameter,&confMsg,&error,&sensorValue, &status,&sensorPhysicalValue);
		xSemaphoreTake(xButtonStruct, portMAX_DELAY);
		
		dashboardControlFunction(&btn,&error,&sensorValue,&status,&confMsg, &deviceState,&parameter,&sensorPhysicalValue);
		xSemaphoreGive(xButtonStruct);
		//vTaskDelay(35/portTICK_RATE_MS);
		//vTaskDelayUntil(&xLastWakeTime,150/portTICK_RATE_MS);
	}
}

static void dashboardControlFunction(Buttons *btn, ModuleError *error, SensorValues *sensorValue, 
	StatusMsg *status, ConfirmationMsgs *confMsg,DeviceState *deviceState,ParameterValue *parameter, SensorPhysicalValues *sensorPhysicalValue) {
		
	changeCarState(confMsg, status, sensorPhysicalValue);
	LEDHandler(sensorPhysicalValue,error,deviceState);
	if (checkForError(error)) {
		HandleErrors(error);
	}
	
	if (RTDS_finished_playing) {
		can_freeRTOSSendMessage(CAN1, FinishedRTDS);
		RTDS_finished_playing = false;
	}
	
	if (btn->unhandledButtonAction) {
		HandleButtonActions(btn,sensorPhysicalValue, deviceState, parameter,error,confMsg);
	}
	
	//UPDATE SCREENS ON THE DISPLAY AND CALL MENU LOCATION DEPENDENT FUNCTIONS
	switch (menu[selected].current_menu) {
		case ECU_OPTIONS:
			if ((menuUpdate.update_menu == true) ) {
				menuUpdate.update_menu = false;
				DrawECUAdjustmentScreen(parameter);
			}
		break;
		
		case FAN_OPTIONS:
			if ((menuUpdate.update_menu == true) ) {
				menuUpdate.update_menu = false;
				DrawFANControlScreen(parameter);
			}			
		break;
		
		case TS_STATUS:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawTSStatusScreen(sensorPhysicalValue,error);
		}
		break;
		case SYSTEM_MONITOR:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawSystemMonitorScreen(error, sensorPhysicalValue);
		}
		
		break;
		
		case TEMP_VOLT:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawBatteryInfoScreen(sensorPhysicalValue);
		}
		
		break;
		case SENSOR_DATA:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawSensorInformationScreen(sensorPhysicalValue);
		}
		break;

		case DEVICE_STATUS:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawDeviceStatusMenu(deviceState);
		}
		
		break;
		case MAIN_SCREEN:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawMainScreen(sensorPhysicalValue, deviceState);
		}
		break;
		case MAIN_MENU:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawMainMenu();
		}	
		break;
		case MAIN_MENU_DOWN:
			if ((menuUpdate.update_menu == true) ) {
				menuUpdate.update_menu = false;
				DrawMainMenuDown();
			}
		break;
		
		case DL_OPTIONS:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawDataloggerInterface();
		}
		break;

	
		case TRQ_CALIB:
		if (menuUpdate.update_procedures == true) {
			menuUpdate.update_procedures = false;
			calibrateTorquePedal(confMsg,false);
		}
		
		break;
		case STEER_CALIB:
			if (menuUpdate.update_procedures == true) {
				menuUpdate.update_procedures = false;
				calibrateSteering(confMsg,false);
			}
		
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
		case PRESET_SEL:
		if ((menuUpdate.update_menu == true) ) {
			menuUpdate.update_menu = false;
			DrawPresetMenu();
		}
		break;
		case PRESET_CONFIRM:
		if (menuUpdate.update_procedures == true) {
			menuUpdate.update_procedures = false;
			DrawPresetConfirmation();
		}
		break;	
		case PRESET_PROCEDURE:	
		if (menuUpdate.update_procedures == true) {
			menuUpdate.update_procedures = false;
			presetProcedureHandling(false,confMsg);
		}
		break;
		case IMU_INFORMATION:
			if (menuUpdate.update_menu == true) {
				menuUpdate.update_menu = false;
				DrawIMUScreen(sensorPhysicalValue);
			}
			break;
	}
}


static void presetProcedureHandling(bool ackPressed, ConfirmationMsgs *confMsg) {
	static enum EDataloggerCommands close_file = CLOSE_FILE;
	static enum EDataloggerCommands get_files = GET_PARAMETERS_FROM_FILE;
	switch (presetProcedureState) {
		case PRESET_PROCEDURE_INIT:
			// Stop and close file for datalogger
			// Send command to extract data from file
			
			
			xQueueSendToBack(xDataloggerCommandQueue,&close_file,10/portTICK_RATE_MS);
			xQueueSendToBack(xDataloggerCommandQueue,&get_files,10/portTICK_RATE_MS);
			presetProcedureState = PRESET_PROCEDURE_WAITING;
		break;
		case PRESET_PROCEDURE_WAITING:
			if (xQueueReceive(xPresetQueue,&presetParameters, 1000/portTICK_RATE_MS) == pdPASS) {
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
			EcuParametersFromFile.data.u8[0] = P_TERM;
			EcuParametersFromFile.data.f[1] = presetParameters.p_term;
			can_freeRTOSSendMessage(CAN1, EcuParametersFromFile);
			xTimerReset(parameterConfTimer,0);
			confMsg->ECU_parameter_confirmed = false;
			parameter_confirmation_timed_out = false;
			presetProcedureState = WAIT_P_TERM;
		break;
		
		case WAIT_P_TERM:
			if (confMsg->ECU_parameter_confirmed == true) {
				confMsg->ECU_parameter_confirmed = false;
				presetProcedureState = PRESET_PROCEDURE_SEND_I_TERM;
			}
			else if (parameter_confirmation_timed_out == true) {
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		
		case PRESET_PROCEDURE_SEND_I_TERM:
			EcuParametersFromFile.data.u8[0] = I_TERM;
			EcuParametersFromFile.data.f[1] = presetParameters.i_term;
			can_freeRTOSSendMessage(CAN1, EcuParametersFromFile);
			xTimerReset(parameterConfTimer,0);
			parameter_confirmation_timed_out = false;
			presetProcedureState = WAIT_I_TERM;
		break;
		
		case WAIT_I_TERM:
			if (confMsg->ECU_parameter_confirmed == true) {
				confMsg->ECU_parameter_confirmed = false;
				presetProcedureState = PRESET_PROCEDURE_FINISHED;
			}
			else if (parameter_confirmation_timed_out == true) {
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		
		case PRESET_PROCEDURE_SEND_D_TERM:
			EcuParametersFromFile.data.u8[0] = D_TERM;
			EcuParametersFromFile.data.f[1] = presetParameters.d_term;
			can_freeRTOSSendMessage(CAN1, EcuParametersFromFile);
			xTimerReset(parameterConfTimer,0);
			parameter_confirmation_timed_out = false;
			presetProcedureState = WAIT_I_TERM;
		break;
		
		case WAIT_D_TERM:
			if (confMsg->ECU_parameter_confirmed == true) {
				confMsg->ECU_parameter_confirmed = false;
				presetProcedureState = PRESET_PROCEDURE_FINISHED;
			}
			else if (parameter_confirmation_timed_out == true) {
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		
		case PRESET_PROCEDURE_SEND_MAX_MIN_TERM:
			EcuParametersFromFile.data.u8[0] = MAX_MIN_VALUE;
			EcuParametersFromFile.data.f[1] = presetParameters.max_min_term;
			can_freeRTOSSendMessage(CAN1, EcuParametersFromFile);
			xTimerReset(parameterConfTimer,0);
			parameter_confirmation_timed_out = false;
			presetProcedureState = WAIT_I_TERM;
		break;
		
		case WAIT_MAX_MIN_TERM:
		if (confMsg->ECU_parameter_confirmed == true) {
			confMsg->ECU_parameter_confirmed = false;
			presetProcedureState = PRESET_PROCEDURE_FINISHED;
		}
		else if (parameter_confirmation_timed_out == true) {
			presetProcedureState = PRESET_PROCEDURE_FAILED;
		}
		break;
		
		case PRESET_PROCEDURE_SEND_MAX_DECREASE_TERM:
			EcuParametersFromFile.data.u8[0] = MAX_DECREASE;
			EcuParametersFromFile.data.f[1] = presetParameters.max_decrease_term;
			can_freeRTOSSendMessage(CAN1, EcuParametersFromFile);
			xTimerReset(parameterConfTimer,0);
			parameter_confirmation_timed_out = false;
			presetProcedureState = WAIT_I_TERM;
		break;
		case WAIT_MAX_DECREASE_TERM:
			if (confMsg->ECU_parameter_confirmed == true) {
				confMsg->ECU_parameter_confirmed = false;
				presetProcedureState = PRESET_PROCEDURE_FINISHED;
			}
			else if (parameter_confirmation_timed_out == true) {
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		case PRESET_PROCEDURE_SEND_DESIRED_SLIP_TERM:
			EcuParametersFromFile.data.u8[0] = DESIRED_SLIP;
			EcuParametersFromFile.data.f[1] = presetParameters.desired_slip_term;
			can_freeRTOSSendMessage(CAN1, EcuParametersFromFile);
			xTimerReset(parameterConfTimer,0);
			parameter_confirmation_timed_out = false;
			presetProcedureState = WAIT_I_TERM;
		break;
		case WAIT_DESIRED_SLIP_TERM:
			if (confMsg->ECU_parameter_confirmed == true) {
				confMsg->ECU_parameter_confirmed = false;
				presetProcedureState = PRESET_PROCEDURE_FINISHED;
			}
			else if (parameter_confirmation_timed_out == true) {
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		
		case PRESET_PROCEDURE_SEND_MAX_INTEGRAL_TERM:
			EcuParametersFromFile.data.u8[0] = MAX_INTEGRAL;
			EcuParametersFromFile.data.f[1] = presetParameters.max_integral_term;
			can_freeRTOSSendMessage(CAN1, EcuParametersFromFile);
			xTimerReset(parameterConfTimer,0);
			parameter_confirmation_timed_out = false;
			presetProcedureState = WAIT_I_TERM;
		break;
		
		case WAIT_MAX_INTEGRAL_TERM:
			if (confMsg->ECU_parameter_confirmed == true) {
				confMsg->ECU_parameter_confirmed = false;
				presetProcedureState = PRESET_PROCEDURE_SEND_SELECTED_PRESET;
			}
			else if (parameter_confirmation_timed_out == true) {
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		case PRESET_PROCEDURE_SEND_SELECTED_PRESET:
			EcuParametersFromFile.data.u8[0] = SELECTED_PRESET;
			EcuParametersFromFile.data.u8[1] = presetParameters.selected_preset;
			can_freeRTOSSendMessage(CAN1, EcuParametersFromFile);
			xTimerReset(parameterConfTimer,0);
			parameter_confirmation_timed_out = false;
			presetProcedureState = WAIT_SELECTED_PRESET;
		break;
		
		case WAIT_SELECTED_PRESET:
			if (confMsg->ECU_parameter_confirmed == true) {
				confMsg->ECU_parameter_confirmed = false;
				presetProcedureState = PRESET_PROCEDURE_FINISHED;
			}
			else if (parameter_confirmation_timed_out == true) {
				presetProcedureState = PRESET_PROCEDURE_FAILED;
			}
		break;
		case PRESET_PROCEDURE_FINISHED:
			DrawPresetProcedure();
			if (ackPressed == true) {
				btn.unhandledButtonAction = false;
				btn.btn_type = NONE_BTN;
				selected = MAIN_MENU_POS;
				presetProcedureState = PRESET_PROCEDURE_OFF;
			}
		// Tell the user that the preset is loaded to ecu
		// Wait for acknowledge to send user back to main menu
		break;
		
		case PRESET_PROCEDURE_FAILED:
			DrawPresetProcedure();
			if (ackPressed == true) {
				btn.unhandledButtonAction = false;
				btn.btn_type = NONE_BTN;
				selected = MAIN_MENU_POS;
				presetProcedureState = PRESET_PROCEDURE_OFF;
			}
		break;
	}
}

static void changeCarState(ConfirmationMsgs *confMsg, StatusMsg *status, SensorPhysicalValues *sensorPhysicalValue ) {
	switch(carState) {
		case TRACTIVE_SYSTEM_OFF:
			if (status->shut_down_circuit_closed == true) {
				carState = TRACTIVE_SYSTEM_ON;
			}
			break;
		case TRACTIVE_SYSTEM_ON:
			if (status->shut_down_circuit_closed == false ) {
				carState = TRACTIVE_SYSTEM_OFF;
				
			}
			else if (confMsg->drive_enabled_confirmed == true) {
				carState = DRIVE_ENABLED;
				confMsg->drive_enabled_confirmed = false;
			}
			break;
		case DRIVE_ENABLED:
			if (status->shut_down_circuit_closed == false) {
				carState = TRACTIVE_SYSTEM_OFF;
			}
			else if (confMsg->drive_disabled_confirmed == true) {
				carState = TRACTIVE_SYSTEM_ON;
				confMsg->drive_disabled_confirmed = false;
			}
			else if (confMsg->lc_request_confirmed == true) {
				carState = LC_PROCEDURE;
				// Go to the menu element for launch control. This is done so that handlebuttons etc can run without any problems
				selected = LC_HANDLER_POS; 
				confMsg->lc_request_confirmed = false;
			}
			
			break;
		case LC_PROCEDURE:
			DrawLaunchControlProcedure();
			//if ( (getTorquePedalPosition(sensorPhysicalValue) == PEDAL_IN) && (getBrakePedalPosition(sensorPhysicalValue) == PEDAL_IN) ) {
			if ( btn.btn_type == PUSH_ACK) {
				carState = LC_STANDBY;
				
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
			//}
			//else if (getTorquePedalPosition(sensorPhysicalValue) == false) {
			//	carState = DRIVE_ENABLED;
			//}
			else if (status->shut_down_circuit_closed == false) {
				carState = TRACTIVE_SYSTEM_OFF;
			}
			else if (confMsg->drive_disabled_confirmed == true) {
				can_sendMessage(CAN0,EcuParametersFromFile);
				carState = TRACTIVE_SYSTEM_ON;
				confMsg->drive_disabled_confirmed = false;
			}
			else if (btn.btn_type == LAUNCH_CONTROL) {
				carState = LC_ABORTED;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
			break;
		
		case LC_STANDBY:
			
			if ( (getBrakePedalPosition(sensorPhysicalValue) == PEDAL_OUT) && (getTorquePedalPosition(sensorPhysicalValue) == PEDAL_IN) ) {
				carState = LC_COUNTDOWN;
				xTimerStart(LcTimer,1000/portTICK_RATE_MS);
				lc_timer_count = 0;
			}
			else if (getTorquePedalPosition(sensorPhysicalValue) == PEDAL_OUT) {
				carState = LC_ABORTED;
			}
			else if (btn.btn_type == LAUNCH_CONTROL) {
				carState = LC_ABORTED;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
			break;
		case LC_COUNTDOWN:
			DrawLaunchControlProcedure();
			//if (getBrakePedalPosition(sensorPhysicalValue) == PEDAL_OUT) && (getTorquePedalPosition(sensorPhysicalValue) == PEDAL_IN) {
				if (lc_timer_count == 5) {
					carState = LC_WAITING_FOR_ECU_TO_ARM_LC;
					xTimerReset(LcTimer,5/portTICK_RATE_MS);
					lc_timer_count = 0;
					//Send can message that countdown is finished
					can_freeRTOSSendMessage(CAN1,RequestLCArmed);
				}
			//}
			/*else {
				xTimerStop(LcTimer,5/portTICK_RATE_MS);
				lc_timer_count = 0;
				carState = LC_ABORTED;
			}*/
			break;		
		case LC_WAITING_FOR_ECU_TO_ARM_LC:
			if (confMsg->lc_ready == true) {
				confMsg->lc_ready = false;
				carState = LC_ARMED;
				xTimerStop(LcTimer,0);
				lc_timer_count = 0;
			}
			else if (lc_timer_count > 2) {
				carState = LC_ARMING_TIMED_OUT;
				lc_timer_count = 0;
				xTimerStop(LcTimer,0);
			}
			break;
		case LC_ARMED:
			DrawLaunchControlProcedure();
			//DrawMainScreen()
			if (confMsg->lc_off == true) {
				confMsg->lc_off = false;
				carState = DRIVE_ENABLED;
				selected = 0;
				//should display main screen
			}
			//if (getTorquePedalPosition(sensorPhysicalValue) == false) {
			//	carState = DRIVE_ENABLED;
			//	selected = 0;
			//}
			break;
		case LC_ABORTED:
			DrawLaunchControlProcedure();
			if ( btn.btn_type == PUSH_ACK) {
				carState = DRIVE_ENABLED;
				selected = 0;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
		break;
		case LC_ARMING_TIMED_OUT:
			DrawLaunchControlProcedure();
			if ( btn.btn_type == PUSH_ACK) {
				carState = DRIVE_ENABLED;
				selected = 0;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
		break;
	}					
}

static void HandleButtonActions(Buttons *btn, SensorPhysicalValues *sensorPhysicalValue ,DeviceState *deviceState, 
						 ParameterValue *parameter, ModuleError *error, ConfirmationMsgs *confMsg) { 
	if (btn->btn_type == NAVIGATION) {
		NavigateMenu(deviceState, parameter,error, sensorPhysicalValue);
		
	}
	else if (btn->btn_type == PUSH_ACK) {
		switch (menu[selected].current_menu) {
			case DL_OPTIONS:
				menu[selected].dataloggerFunc();
			break;
			case ERROR_HANDLER:
				//can_sendMessage(CAN0,ackMsg);
				selected = 0; // Return to main menu
			break;
			case PRESET_SEL:
				
				if (presetProcedureState == PRESET_PROCEDURE_OFF) {
					//presetProcedureState = PRESET_PROCEDURE_INIT;
					// Get the file name off the preset file to get data from
					// This file name is global and used in the datalogger task 
					// to open the correct file.
					switch (menu[selected].current_setting) {
						case PRESET_1_SETTING:
							strcpy(preset_file_name,"VWET10.txt");
							presetParameters.selected_preset = 0;
						break;
						case PRESET_2_SETTING:
							strcpy(preset_file_name,"VWET20.txt");
							presetParameters.selected_preset = 1;
						break;
						case PRESET_3_SETTING:
							strcpy(preset_file_name,"WET10.txt");
							presetParameters.selected_preset = 2;
						break;
						case PRESET_4_SETTING:
							strcpy(preset_file_name,"WET20.txt");
							presetParameters.selected_preset = 3;
						break;
						case PRESET_5_SETTING:
							strcpy(preset_file_name,"DRY10.txt");
							presetParameters.selected_preset = 4;
						break;
						case PRESET_6_SETTING:
							strcpy(preset_file_name,"DRY20.txt");
							presetParameters.selected_preset = 5;
						break;
						case PRESET_7_SETTING:
							strcpy(preset_file_name,"DRY25.txt");
							presetParameters.selected_preset = 6;
						break;
						case PRESET_8_SETTING:
							strcpy(preset_file_name,"GENERAL.txt");
							presetParameters.selected_preset = 7;
						break;
					}
					//presetSetting = menu[selected].current_setting;
					//presetProcedureHandling(false,confMsg);
					selected = menu[selected].push_button;
				}
			break;
			case PRESET_CONFIRM:
				if (menu[selected].current_setting == CONFIRM_YES) {
					presetProcedureState = PRESET_PROCEDURE_INIT;
				}
				selected = menu[selected].push_button;
			case PRESET_PROCEDURE:
				presetProcedureHandling(true,confMsg);
				break;
			case ECU_OPTIONS:
				switch (menu[selected].current_setting) {
					case TORQUE_SETTING:
						//EcuParametersFromFile.data = parameter->
						EcuParametersFromFile.data.u8[0] = MAX_TORQUE;
						EcuParametersFromFile.dataLength = 2;
						if ( (parameter->torque <= 100) && (parameter->torque >= 0) ) {
							EcuParametersFromFile.data.u8[1] = parameter->torque;
							can_freeRTOSSendMessage(CAN1,EcuParametersFromFile);
						}
					break;
					case KERS_SETTING:
						EcuParametersFromFile.data.u8[0] = KERS_ADJUST;
						EcuParametersFromFile.dataLength = 2;
						if ( (parameter->kers_value <= parameter->max_kers_value) && (parameter->kers_value >= 0) ) {
							EcuParametersFromFile.data.u8[1] = parameter->kers_value;
							can_freeRTOSSendMessage(CAN1,EcuParametersFromFile);
						}
						break;
					case TRACTION_CONTROL_SETTING:
						if (parameter->traction_control_value == 0) {
							EcuTractionControl.data.u8[0] =  0xF0;
						}
						else if (parameter->traction_control_value == 1) {
							EcuTractionControl.data.u8[0] = 0x0F;
						}
						can_freeRTOSSendMessage(CAN1,EcuTractionControl);
						
					break;
					
				}
				//prev_selected = selected;
				//selected = LOCKED_SEL_POS;
				parameter_confirmation_timed_out = false;
				// The menu selection is set to a locked position, the menu will return to its position after 
				// the timer runs out or the variable is confirmed.
				xTimerReset(parameterConfTimer,20/portTICK_RATE_MS); //Start variable confirmation timer
			break;
			case FAN_OPTIONS:
				switch (menu[selected].current_setting) {
					case ALL_FAN_SETTING:
						if (parameter->all_fan_setting == 0) {
							can_freeRTOSSendMessage(CAN0,TurnOnAllFans);
							parameter->all_fan_setting = 1;
						}
						else if (parameter->all_fan_setting == 1) {
							can_freeRTOSSendMessage(CAN0,TurnOffAllFans);
							parameter->all_fan_setting = 0;
						}
					break;
					case RADIATOR_FAN_SETTING:
						AdjustDutyCycleRadiatorFan.data.u8[2] = parameter->radiator_fan_value;
						can_freeRTOSSendMessage(CAN0,AdjustDutyCycleRadiatorFan);
					break;
					case MONO_FAN_SETTING:
						AdjustDutyCycleMonoFan.data.u8[2] = parameter->mono_fan_value;
						can_freeRTOSSendMessage(CAN0,AdjustDutyCycleMonoFan);
					break;
					case BATTERY_FAN_SETTING:
						AdjustDutyCycleBatteryFan.data.u8[2] = parameter->battery_fan_value;
						can_freeRTOSSendMessage(CAN0,AdjustDutyCycleBatteryFan);
					break;
					case PUMP_SETTING:
						if (parameter->pump_setting_value == 0) {
							can_freeRTOSSendMessage(CAN0,TurnOnPump);
							parameter->pump_setting_value = 1;
						}
						else {
							can_freeRTOSSendMessage(CAN0,TurnOffPump);
							parameter->pump_setting_value = 0;
						}
					break;
				}
			break;
				
			case PERSISTENT_MSG:
				//If a persistent msg is acknowledged the user is returned to the main screen
				selected = 0; //Return to main screen
				DrawMainScreen(sensorPhysicalValue,deviceState);
			break;
			case TRQ_CALIB:
				calibrateTorquePedal(confMsg,true);
			break;
			case STEER_CALIB:
				calibrateSteering(confMsg,true);
			break;
			
			case SNAKE_GAME:
				if (snakeGameState == SNAKE_OFF) {
					snakeGameState = SNAKE_PREPPING;
					snakeControlFunction(false,UP);
				}
			break;
			default:
			selected = menu[selected].push_button;
			break;
		}
	}
	
	else if (btn->btn_type == ROT_ACK) {
		switch (menu[selected].current_setting) {
			case TORQUE_SETTING:
				switch (StepSizeVar.torque){
					case ONE:
					StepSizeVar.torque = STEP_TEN;
					break;
					case STEP_TEN:
					StepSizeVar.torque = STEP_ONE;
					break;
				}
			break;
			case KERS_SETTING:
				switch (StepSizeVar.kers){
					case ONE:
					StepSizeVar.kers = STEP_TEN;
					break;
					case STEP_TEN:
					StepSizeVar.kers = STEP_ONE;
					break;
				}
			break;
			case RADIATOR_FAN_SETTING:
				switch (StepSizeVar.radiator_fan) {
					case ONE:
						StepSizeVar.radiator_fan = STEP_TEN;
						break;
					case STEP_TEN:
						StepSizeVar.radiator_fan = STEP_ONE;
					break;
				}
			break;
			case MONO_FAN_SETTING:
				switch (StepSizeVar.mono_fan) {
					case ONE:
					StepSizeVar.mono_fan = STEP_TEN;
					break;
					case STEP_TEN:
					StepSizeVar.mono_fan = STEP_ONE;
					break;
				}
			break;
			case BATTERY_FAN_SETTING:
				switch (StepSizeVar.battery_fan) {
					case ONE:
					StepSizeVar.battery_fan = STEP_TEN;
					break;
					case STEP_TEN:
					StepSizeVar.battery_fan = STEP_ONE;
					break;
				}	
			break;			
		}
	}
	else if (btn->btn_type == LAUNCH_CONTROL) {
		if (carState == DRIVE_ENABLED) {
			//Request Launch control from ECU
			//Send CAN message
			can_freeRTOSSendMessage(CAN1,RequestLCInit);

		}
	}
	else if (btn->btn_type == ROTARY) {
		// If currently looking at a variable to adjust, call the specific function to adjust this value
		if (  (menu[selected].current_menu == ECU_OPTIONS) || (menu[selected].current_menu == FAN_OPTIONS) ){ //Add the different option names
			if (menu[selected].rotaryActionFunc != 0) {
				if (btn->rotary_cw == true) {
					menu[selected].rotaryActionFunc(CW,parameter);
				}
				else{
					menu[selected].rotaryActionFunc(CCW,parameter);
				}
			}
		}
	}
	else if (btn->btn_type == START) {
		if ( (carState != TRACTIVE_SYSTEM_OFF) && (carState != TRACTIVE_SYSTEM_ON) )  {
			// Request shut down
			can_freeRTOSSendMessage(CAN1, RequestDriveDisable);
		}
		else if (carState == TRACTIVE_SYSTEM_ON) {
			//Request car start
			// If criterias satisfied
			
			if ( (getTorquePedalPosition(sensorPhysicalValue) == PEDAL_OUT) && (getBrakePedalPosition(sensorPhysicalValue) == PEDAL_IN) ) {
				can_freeRTOSSendMessage(CAN1, RequestDriveEnable);
			}
			else {
				selected = DRIVE_ENABLE_WARNING_SEL;
				DrawDriveEnableWarning();
			}
		}
	}
	btn->rotary_ccw = false;
	btn->rotary_cw  = false;
	btn->navigation = NAV_DEFAULT;
	btn->unhandledButtonAction = false;
	btn->btn_type = NONE_BTN;
}

static void NavigateMenu(DeviceState *deviceState, ParameterValue *parameter, ModuleError *error, SensorPhysicalValues *sensorPhysicalValue) {
	
	switch (btn.navigation) {
		case UP:
			switch (menu[selected].current_menu) {
				case ECU_OPTIONS:
				parameter_confirmation_timed_out = true;
				setParameterBasedOnConfirmation(parameter);
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
				setParameterBasedOnConfirmation(parameter);
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
		DrawMainScreen(sensorPhysicalValue,deviceState);
		break;
		case SPEED:
		DrawSpeedScreen(sensorPhysicalValue);
		break;
		case SYSTEM_MONITOR:
		DrawSystemMonitorScreen(error,sensorPhysicalValue);
		break;
		case TEMP_VOLT:
		DrawBatteryInfoScreen(sensorPhysicalValue);
		break;
		case MAIN_MENU:
		DrawMainMenu();
		break;
		case DEVICE_STATUS:
		DrawDeviceStatusMenu(deviceState);
		break;
		case ECU_OPTIONS:
		DrawECUAdjustmentScreen(parameter);
		break;
		case DL_OPTIONS:
		DrawDataloggerInterface();
		break;
		case PRESET_SEL:
		DrawPresetMenu();
		break;
		//case SNAKE_GAME:
// 		if (snakeGameState == SNAKE_OFF) {
// 			snakeGameState = SNAKE_PREPPING;
// 			snakeControlFunction(false,UP);
// 		}
// 		break;
	}
}

static void LEDHandler(SensorPhysicalValues *sensorPhysicalValue, ModuleError *error,DeviceState *devices) {
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
	if ( (sensorPhysicalValue->BMS_max_temp > BMS_MAX_TEMP_TRESHOLD) ) {
		pio_setOutput(TEMP_LED_PIO,TEMP_LED_PIN,PIN_HIGH);
	}
	else {
		//set low
		pio_setOutput(TEMP_LED_PIO,TEMP_LED_PIN,PIN_LOW);
	}
	
	if ( sensorPhysicalValue->min_cell_voltage < HV_MIN_CELL_VOLTAGE_TORQUE_LIMIT_TRESHOLD ) {
		pio_setOutput(VOLT_LED_PIO,VOLT_LED_PIN,PIN_HIGH);
	}
// 	if ( (sensorPhysicalValue->battery_voltage > BATTERY_PACK_MAX_VOLTAGE_TRESHOLD) || (sensorPhysicalValue->battery_voltage < BATTERY_PACK_MIN_VOLTAGE_TRESHOLD) ) {
// 		pio_setOutput(VOLT_LED_PIO,VOLT_LED_PIN,PIN_HIGH);
// 	}
	else {
		pio_setOutput(VOLT_LED_PIO,VOLT_LED_PIN,PIN_LOW);
	}
	
// 	if (error->ecu_error != 0) {
// 		pio_setOutput(ECU_LED_PIO,ECU_LED_PIN,PIN_HIGH);
// 	}
// 	else {
// 		pio_setOutput(ECU_LED_PIO,ECU_LED_PIN,PIN_LOW);
// 	}
// 	
	if (  !(error->bspd_status & BSPD_SHUTDOWN_NOT_ACTIVE)   || (error->ECU_implausibility == ECU_TPS_BPS_IMPLAUSIBILITY) ) {
		pio_setOutput(DEVICE_LED_PIO,DEVICE_LED_PIN,PIN_HIGH);
	}
	else {
		pio_setOutput(DEVICE_LED_PIO,DEVICE_LED_PIN,PIN_LOW);
	}
	
// 	if (checkDeviceStatus(devices) == false) {
// 		//Set ouput high
// 		pio_setOutput(DEVICE_LED_PIO,DEVICE_LED_PIN,PIN_HIGH);
// 	}
// 	else {
// 		//set low
// 		pio_setOutput(DEVICE_LED_PIO,DEVICE_LED_PIN,PIN_LOW);
// 	}
	
	switch (carState) {
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

static void getDashMessages(ParameterValue *parameter, ConfirmationMsgs *confMsg, ModuleError *error, SensorValues *sensorValue,StatusMsg *status,SensorPhysicalValues *sensorPhysicalValue) {
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
			
			case CAN_INVERTER_DATA_STATUS_ID:
				// bits.. 
				error->inverter_data_status = ReceiveMsg.data.u32[0];
			break;
			case CAN_INVERTER_DATA_VOLTAGE_ID:
				sensorPhysicalValue->Inverter_voltage = ReceiveMsg.data.f[0];
			break;
			case ECU_CURRENT_IMPLAUSIBILITY_ID:
				error->ECU_implausibility = ReceiveMsg.data.u8[0];
			break;
			
			case ID_BSPD_STATUS:
				error->bspd_status = ReceiveMsg.data.u8[0];
			break;
			case ID_IMD_STATUS:
				error->IMD_status = ReceiveMsg.data.u8[0];
			break;
			case ID_TRQ_CONF_CH0:
				switch (ReceiveMsg.data.u8[0]) {
					case 0x0F:
						confMsg->conf_trq_ch0 = TRQ_MIN_CONFIRMED;
						break;
					case 0xF0:
						confMsg->conf_trq_ch0 = TRQ_MAX_CONFIRMED;
						break;
					case 0x00:
						confMsg->conf_trq_ch0 = TRQ_NOCALIB;
						break;
				}
			break;
			case ID_TRQ_CONF_CH1:
				switch (ReceiveMsg.data.u8[0]) {
					case 0x0F:
						confMsg->conf_trq_ch1 = TRQ_MIN_CONFIRMED;
						break;
					case 0xF0:
						confMsg->conf_trq_ch1 = TRQ_MAX_CONFIRMED;
						break;
					case 0x00:
						confMsg->conf_trq_ch1 = TRQ_NOCALIB;
						break;
				}
			break;
			case ID_STEERING_CONF:
				switch (ReceiveMsg.data.u8[0]) {
					case 0x0F:
					confMsg->conf_steer = STEER_CONF_LEFT;
					break;
					case 0xF0:
					confMsg->conf_steer = STEER_CONF_RIGHT;
					break;
					case 0x00:
					confMsg->conf_steer = STEER_CONF_FAILED;
					break;
				}
			break;
			case BMS_STATE_MSG_ID:
			// Using "shut_down_circuit_closed" as tractive system activated variable
				error->BMS_state_vector = ReceiveMsg.data.u16[0];
				if (ReceiveMsg.data.u8[2] == 2 ) {
					status->shut_down_circuit_closed = true;
					error->ams_error = false;
				}
				else if ( ReceiveMsg.data.u8[2] == 3) {
					error->ams_error = true;
					status->shut_down_circuit_closed = false;
				}
				else {
					status->shut_down_circuit_closed = false;
					error->ams_error = false;
				}
			break;
			
			case ID_ECU_CAR_STATES:
				switch (ReceiveMsg.data.u8[0]) {
					case 0x01:
					//Play RTDS. The timer callback will turn it off after RTDS_DURATION_MS has passed.
					// It will also send a can message telling the ECU the dash is done with RTDS.
					xTimerStart(RTDSTimer,200/portTICK_RATE_MS);
					pio_setOutput(BUZZER_PIO,BUZZER_PIN,PIN_HIGH);
					break;
					case 0x02:
					//Ready to drive, drive enabled
					confMsg->drive_enabled_confirmed = true;
					break;
					case 0x03:
					//Drive disabled
					confMsg->drive_disabled_confirmed = true;
					break;
				}
			break;
			case ID_ECU_PARAMETER_CONFIRMED:
				confMsg->ECU_parameter_confirmed = true;
				if (menu[selected].current_menu == ECU_OPTIONS) {
					setParameterBasedOnConfirmation(parameter);
				}
			break;
			case ID_IN_ECU_LC:
				switch (ReceiveMsg.data.u8[0]) {
					case 0xF0:
					//Launch ready
					confMsg->lc_ready = true;
					break;
					case 0x0F:
					// Launch request accepted Starting countdown
					confMsg->lc_request_confirmed = true;
					break;
					case 0x00:
						confMsg->lc_off = true;
					break;
				}
			break;
			case ID_IN_ECU_TRACTION_CONTROL:
				switch (ReceiveMsg.data.u8[0]) {
					case 0x0F:
						//ON
						tractionControlState = TRACTION_CONTROL_ON;
					break;
					case 0xF0:
						//OFF
						tractionControlState = TRACTION_CONTROL_OFF;
					break;
				}
			break;
			case ID_IN_ECU_SELECTED_PRESET:
				selected_preset_file = ReceiveMsg.data.u8[0];
			break;
			case GLVBMS_MAXMIN_VAL_ID:
				sensorPhysicalValue->GLV_voltage_max_cell = (ReceiveMsg.data.u16[0]/(float) 10000);
				sensorPhysicalValue->GLV_voltage_min_cell = (ReceiveMsg.data.u16[1]/(float) 10000);
				
				sensorPhysicalValue->GLVBMS_max_temp =  -TEMP_C_1*pow(ReceiveMsg.data.u16[2],3) + TEMP_C_2*pow(ReceiveMsg.data.u16[2],2) - TEMP_C_3*(ReceiveMsg.data.u16[2]) + TEMP_C_4;
				sensorPhysicalValue->GLVBMS_min_temp =  -TEMP_C_1*pow(ReceiveMsg.data.u16[3],3) + TEMP_C_2*pow(ReceiveMsg.data.u16[3],2) - TEMP_C_3*(ReceiveMsg.data.u16[3]) + TEMP_C_4;
			break;
			case GLVBMS_TOTVTG_ID:
				
				sensorPhysicalValue->GLV_voltage = (ReceiveMsg.data.u32[0]/(float) 10000);
				if (( sensorPhysicalValue->GLV_voltage > GLV_BATTERY_EMPTY_VOLTAGE) && (sensorPhysicalValue->GLV_voltage < GLV_BATTERY_FULL_VOLTAGE)) {
					sensorPhysicalValue->GLV_battery_percent = ( (sensorPhysicalValue->GLV_voltage - GLV_BATTERY_EMPTY_VOLTAGE) / GLV_BATTERY_VOLTAGE_RANGE )*100;
				}
			break;
			case BMS_TOTVTG_ID:
				sensorPhysicalValue->battery_voltage = (ReceiveMsg.data.u32[0]/(float) 10000);
				// Deprecated in favor of current based SOC
				//if (( sensorPhysicalValue->battery_voltage > HV_BATTERY_EMPTY_VOLTAGE) && (sensorPhysicalValue->battery_voltage < HV_BATTERY_FULL_VOLTAGE)) {
				//	sensorPhysicalValue->HV_battery_percent =  ( (sensorPhysicalValue->battery_voltage - HV_BATTERY_EMPTY_VOLTAGE) / HV_BATTERY_VOLTAGE_RANGE )*100;
				//}
				
			break;
			case BMS_CURRENT_ID:
				sensorPhysicalValue->current_counter = ((float) (ReceiveMsg.data.i32[1])) * (float) 0.01;
				sensorPhysicalValue->HV_battery_percent = 100.0 - (sensorPhysicalValue->current_counter / HV_BATTERY_TOTAL_CURRENT) * (float) 100;
				if ( (sensorPhysicalValue->HV_battery_percent > 100 ) || (sensorPhysicalValue->HV_battery_percent < 0) ) {
					sensorPhysicalValue->HV_battery_percent = 100;
				}
			break;
			case BMS_MAXMIN_VTG_ID:
// 				sensorPhysicalValue->max_cell_id = ReceiveMsg.data.u16[2];
// 				sensorPhysicalValue->min_cell_id = ReceiveMsg.data.u16[3];
				sensorPhysicalValue->max_cell_id = 0;
				sensorPhysicalValue->min_cell_id = 0;
				
				sensorPhysicalValue->max_cell_voltage = (ReceiveMsg.data.u16[0]/(float) 10000);
				sensorPhysicalValue->min_cell_voltage = (ReceiveMsg.data.u16[1]/(float) 10000);
				
// 				sensorPhysicalValue->max_battery_temperature_lsb = ReceiveMsg.data.u8[4];
// 				sensorPhysicalValue->max_battery_temperature_msb = ReceiveMsg.data.u8[5];
// 				
// 				sensorPhysicalValue->min_battery_temperature_lsb = ReceiveMsg.data.u8[6];
// 				sensorPhysicalValue->min_battery_temperature_msb = ReceiveMsg.data.u8[7];
			break;
			case BMS_MAXMIN_TEMP_ID:
				sensorPhysicalValue->BMS_max_temp_cell_id = ReceiveMsg.data.u16[2];
				sensorPhysicalValue->BMS_min_temp_cell_id = ReceiveMsg.data.u16[3];
				
				sensorPhysicalValue->BMS_max_temp =  -TEMP_C_1*pow(ReceiveMsg.data.u16[0],3) + TEMP_C_2*pow(ReceiveMsg.data.u16[0],2) - TEMP_C_3*(ReceiveMsg.data.u16[0]) + TEMP_C_4;
				sensorPhysicalValue->BMS_min_temp =  -TEMP_C_1*pow(ReceiveMsg.data.u16[1],3) + TEMP_C_2*pow(ReceiveMsg.data.u16[1],2) - TEMP_C_3*(ReceiveMsg.data.u16[1]) + TEMP_C_4;
			
			break;
			
// 			case GLVBMS_TEMP_ID:
// 				sensorPhysicalValue->GLVBMS_max_temp_cell_id = ReceiveMsg.data.u16[3];
// 				sensorPhysicalValue->GLVBMS_min_temp_cell_id = ReceiveMsg.data.u16[4];
// 				
// 				//Temp = -7.175*10^(-12) * (x^3) + 3.67*10^(-7) * (x^2) - 9.898 *10 ^(-3) * ( x ) + 124.831
// 			
// 				sensorPhysicalValue->GLVBMS_max_temp = -TEMP_C_1*pow(ReceiveMsg.data.u16[0],3) + TEMP_C_2*pow(ReceiveMsg.data.u16[0],2) - TEMP_C_3*(ReceiveMsg.data.u16[0]) + TEMP_C_4;
// 				sensorPhysicalValue->GLVBMS_min_temp = -TEMP_C_1*pow(ReceiveMsg.data.u16[1],3) + TEMP_C_2*pow(ReceiveMsg.data.u16[1],2) - TEMP_C_3*(ReceiveMsg.data.u16[1]) + TEMP_C_4;
// 			break;
			case ID_FAN_STATUS:
				if ( menu[selected].current_menu != FAN_OPTIONS) {
					parameter->radiator_fan_value	= ReceiveMsg.data.u8[1];
					parameter->battery_fan_value	= ReceiveMsg.data.u8[2];
					parameter->mono_fan_value		= ReceiveMsg.data.u8[3];
					parameter->pump_setting_value	= ReceiveMsg.data.u8[4];
				}	
			break;

			case ID_STEERING_ENCODER_DATA:
				sensorPhysicalValue->steering_enc_data = ReceiveMsg.data.u8[0] << 8 | ReceiveMsg.data.u8[1];
			break;
			
			case ID_TORQUE_ENCODER_0_DATA:
				sensorPhysicalValue->torque_encoder_ch0 = ReceiveMsg.data.u8[0] << 8 | ReceiveMsg.data.u8[1];
			break;
				
			case ID_TORQUE_ENCODER_1_DATA:
				sensorPhysicalValue->torque_encoder_ch1 = ReceiveMsg.data.u8[0] << 8 | ReceiveMsg.data.u8[1];
			break;	
			case ID_SPEED_FL:
			case ID_SPEED_FR:
			case ID_SPEED_RR:
			case ID_SPEED_RL:
			break;
			case ID_TEMP_COOLING:
				sensorPhysicalValue->cooling_temperature = ( (ReceiveMsg.data.u8[0] << 8) | ReceiveMsg.data.u8[1]) / (float) 100;
			break;
			case ID_TEMP_GEARBOX:
				sensorPhysicalValue->gearbox_temperature = ( (ReceiveMsg.data.u8[0] << 8) | ReceiveMsg.data.u8[1]) / (float) 100;
			break;
			case ID_BRAKE_PRESSURE_FL:
				sensorPhysicalValue->brake_pressure_fl = ( (ReceiveMsg.data.u8[0] << 8) | ReceiveMsg.data.u8[1]);
				break;
			case ID_BRAKE_PRESSURE_FR:
				sensorPhysicalValue->brake_pressure_fr = ( (ReceiveMsg.data.u8[0] << 8) | ReceiveMsg.data.u8[1]); // Linear
			break;
			case ID_DAMPER_FL:
			case ID_DAMPER_FR:
			case ID_DAMPER_RL:
			case ID_DAMPER_RR:
				break;
			case ID_IMD_SHUTDOWN:
				error->imd_error = true;
				break;
				
			//*********************************************************//
			//************************IMU DATA*************************//
			//*********************************************************//
			case ID_IMU_ROT_DATA:
				sensorPhysicalValue->IMU_rot_x	=	((float) (ReceiveMsg.data.u16[0]))*IMU_ROT_G_C;
				sensorPhysicalValue->IMU_rot_y	=	((float) (ReceiveMsg.data.u16[1]))*IMU_ROT_G_C;
				sensorPhysicalValue->IMU_rot_z	=	((float) (ReceiveMsg.data.u16[2]))*IMU_ROT_G_C;
			break;
			case ID_IMU_G_FORCE_DATA:
				sensorPhysicalValue->IMU_G_x	=	((float) (ReceiveMsg.data.u16[0]))*IMU_ROT_G_C;
				sensorPhysicalValue->IMU_G_y	=	((float) (ReceiveMsg.data.u16[1]))*IMU_ROT_G_C;
				sensorPhysicalValue->IMU_G_z	=	((float) (ReceiveMsg.data.u16[2]))*IMU_ROT_G_C;
			break;
			case ID_IMU_POSITION_DATA:
			break;
			case ID_IMU_VELOCITY_DATA:
				sensorPhysicalValue->IMU_vel_x	=	((float) (ReceiveMsg.data.u16[0]))*IMU_VEL_C;
				sensorPhysicalValue->IMU_vel_y	=	((float) (ReceiveMsg.data.u16[1]))*IMU_VEL_C;
				sensorPhysicalValue->IMU_vel_z	=	((float) (ReceiveMsg.data.u16[2]))*IMU_VEL_C;
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
static void calibrateTorquePedal(ConfirmationMsgs *confMsg,bool ack_pressed) {
	
	
	switch(torquePedalCalibrationState) {
		case TRQ_CALIBRATION_OFF:
			DrawTorqueCalibrationScreen(confMsg);
			if (ack_pressed == true) {
				confMsg->conf_trq_ch0 = TRQ_DEFAULT;
				confMsg->conf_trq_ch1 = TRQ_DEFAULT;
				trq_calib_timed_out = false;
				torquePedalCalibrationState = TRQ_CALIBRATION_WAITING_MAX_CONFIRMATION;
				//Send CAN message that max is being calibrated
				can_freeRTOSSendMessage(CAN0,TorquePedalCalibrationMax);
				can_freeRTOSSendMessage(CAN1,TorquePedalCalibrationMax);
				xTimerReset(calibrationTimer,2/portTICK_RATE_MS);
			}
			break;
		case TRQ_CALIBRATION_WAITING_MAX_CONFIRMATION:
			if (trq_calib_timed_out == false) {
				switch (confMsg->conf_trq_ch0) {
					case TRQ_MAX_CONFIRMED:
						trq_ch0_ok = true;
						break;
					case TRQ_NOCALIB:
						trq_ch0_ok = true;
						break;
				}	
				switch (confMsg->conf_trq_ch1) {
					case TRQ_MAX_CONFIRMED:
						trq_ch1_ok = true;
						break;
					case TRQ_NOCALIB:
						trq_noCalib_ch1 = true;
						break;
				}			
				if (trq_ch0_ok && trq_ch1_ok) {
					torquePedalCalibrationState = TRQ_CALIBRATION_MAX_CONFIRMED;
				}
				else if ( (trq_noCalib_ch0 == true) && (trq_noCalib_ch1 == false) ) {
					// Ch0 calib error
					torquePedalCalibrationState = TRQ_FAIL_CH0;
				}
				else if ( (trq_noCalib_ch0 == false) && (trq_noCalib_ch1 == true) ) {
					// Ch1 calib error
					torquePedalCalibrationState = TRQ_FAIL_CH1;
				}
				else if (trq_noCalib_ch0 && trq_noCalib_ch1) {
					//Fail
					torquePedalCalibrationState = TRQ_FAIL_BOTH_CH;
				}
			}
			else {
				//Timed out
				if ( (trq_ch0_ok == false) && (trq_ch1_ok == false) ) {
					//Both timed out
					torquePedalCalibrationState = TRQ_TIMEOUT_BOTH_CH;
				}
				else if ( (trq_ch0_ok == true) && (trq_ch1_ok == false) ) {
					// trq_ch1 timed out
					torquePedalCalibrationState = TRQ_TIMEOUT_CH1;
				}
				else if ( (trq_ch0_ok == false) && (trq_ch1_ok == true) ) {
					// Trq_ch0 timed out
					torquePedalCalibrationState = TRQ_TIMEOUT_CH0;
				}
			}
		break;
			
		case TRQ_CALIBRATION_MAX_CONFIRMED:
			DrawTorqueCalibrationScreen(confMsg);
			if (ack_pressed == true) {
				confMsg->conf_trq_ch0 = TRQ_DEFAULT; // Reset the confirmation message
				confMsg->conf_trq_ch1 = TRQ_DEFAULT;
				trq_ch0_ok = false;
				trq_ch1_ok = false;
				trq_noCalib_ch0 = false;
				trq_noCalib_ch1 = false;
				torquePedalCalibrationState = TRQ_CALIBRATION_WAITING_MIN_CONFIRMATION;
				// Send CAN message that min torque is being calibrated
				can_freeRTOSSendMessage(CAN0,TorquePedalCalibrationMin);
				can_freeRTOSSendMessage(CAN1,TorquePedalCalibrationMin);
				trq_calib_timed_out = false; // Reset time out flag
				xTimerReset(calibrationTimer,5/portTICK_RATE_MS);
			}
		break;

		case TRQ_CALIBRATION_WAITING_MIN_CONFIRMATION:
			if (trq_calib_timed_out == false) {
				switch (confMsg->conf_trq_ch0) {
					case TRQ_MIN_CONFIRMED:
					trq_ch0_ok = true;
					break;
					case TRQ_NOCALIB:
					trq_ch0_ok = true;
					break;
				}
				switch (confMsg->conf_trq_ch1) {
					case TRQ_MIN_CONFIRMED:
					trq_ch1_ok = true;
					break;
					case TRQ_NOCALIB:
					trq_noCalib_ch1 = true;
					break;

				}
				// Make swithc case with trq0 and tr1 combinged into a 2 bit variable
				if (trq_ch0_ok && trq_ch1_ok) {
					torquePedalCalibrationState = TRQ_CALIBRATION_MIN_CONFIRMED;
				}
				else if ( (trq_noCalib_ch0 == true) && (trq_noCalib_ch1 == false) ) {
					// Ch0 calib error
					torquePedalCalibrationState = TRQ_FAIL_CH0;
				}
				else if ( (trq_noCalib_ch0 == false) && (trq_noCalib_ch1 == true) ) {
					// Ch1 calib error
					torquePedalCalibrationState = TRQ_FAIL_CH1;
				}
				else if (trq_noCalib_ch0 && trq_noCalib_ch1) {
					//Fail
					torquePedalCalibrationState = TRQ_FAIL_BOTH_CH;
				}
			}
			else {
				//Timed out
				if ( (trq_ch0_ok == false) && (trq_ch1_ok == false) ) {
					//Both timed out
					torquePedalCalibrationState = TRQ_TIMEOUT_BOTH_CH;
				}
				else if ( (trq_ch0_ok == true) && (trq_ch1_ok == false) ) {
					// trq_ch1 timed out
					torquePedalCalibrationState = TRQ_TIMEOUT_CH1;
				}
				else if ( (trq_ch0_ok == false) && (trq_ch1_ok == true) ) {
					// Trq_ch0 timed out
					torquePedalCalibrationState = TRQ_TIMEOUT_CH0;
				}
			}
		break;

		case TRQ_CALIBRATION_MIN_CONFIRMED:
			DrawTorqueCalibrationScreen(confMsg);
			if (ack_pressed == true) {
				trq_ch0_ok = false;
				trq_ch1_ok = false;
				trq_noCalib_ch0 = false;
				trq_noCalib_ch1 = false;
				confMsg->conf_trq_ch0 = TRQ_DEFAULT; // Reset the confirmation message
				confMsg->conf_trq_ch1 = TRQ_DEFAULT;
				
				trq_calib_timed_out = false; // Reset time out flag
				torquePedalCalibrationState = TRQ_CALIBRATION_OFF;
				selected = MAIN_MENU_POS; // Back to adjustment menu
			}
		break;
	
		case TRQ_FAIL_BOTH_CH:
		case TRQ_FAIL_CH0:
		case TRQ_FAIL_CH1:
		case TRQ_TIMEOUT_CH0:
		case TRQ_TIMEOUT_CH1:
		case TRQ_TIMEOUT_BOTH_CH:
			DrawTorqueCalibrationScreen(confMsg);
			if (ack_pressed == true) {
				confMsg->conf_trq_ch0 = TRQ_DEFAULT;
				confMsg->conf_trq_ch1 = TRQ_DEFAULT;
				trq_ch0_ok = false;
				trq_ch1_ok = false;
				trq_noCalib_ch0 = false;
				trq_noCalib_ch1 = false;
				
				trq_calib_timed_out = false;
				torquePedalCalibrationState = TRQ_CALIBRATION_OFF;
				selected = MAIN_MENU_POS;
				btn.btn_type = NONE_BTN;
				btn.unhandledButtonAction = false;
			}
			break;
	}
}
static void calibrateSteering(ConfirmationMsgs *confMsg,bool ack_pressed) {
	switch (steeringCalibrationState) {
		
		case STEER_C_OFF:
			DrawSteerCalibScreen();
			if (ack_pressed) {
				confMsg->conf_steer = STEER_CONF_DEFAULT;
				steer_calib_timed_out = false;
				steeringCalibrationState = STEER_C_WAITING_LEFT;
				// Send CAN message that left is calibrated
				can_freeRTOSSendMessage(CAN0,SteeringCalibrationLeft);
				can_freeRTOSSendMessage(CAN1,SteeringCalibrationLeft);
				xTimerReset(calibrationTimer,15/portTICK_RATE_MS);	
			}
			break;
		case STEER_C_WAITING_LEFT:
			if (steer_calib_timed_out == false) {
				switch (confMsg->conf_steer) {
					case STEER_CONF_LEFT:
						steeringCalibrationState = STEER_C_LEFT_CONFIRMED;
					break;
					case STEER_CONF_FAILED:
						steeringCalibrationState = STEER_C_FAIL;
					break;
				}
			}
			else if (steer_calib_timed_out == true) {
				steeringCalibrationState = STEER_C_TIMEOUT;
			}
		break;
		
		case STEER_C_LEFT_CONFIRMED:
			DrawSteerCalibScreen();
			if (ack_pressed == true) {
				confMsg->conf_steer = STEER_CONF_DEFAULT;
				steeringCalibrationState = STEER_C_WAITING_RIGHT;
				steer_calib_timed_out = false;
				// Send CAN message right is being calibrated
				can_freeRTOSSendMessage(CAN0,SteeringCalibrationRight);
				can_freeRTOSSendMessage(CAN1,SteeringCalibrationRight);
				xTimerReset(calibrationTimer,5/portTICK_RATE_MS);
			}
			break;
			
		case STEER_C_WAITING_RIGHT:
			if (steer_calib_timed_out == false) {
				switch (confMsg->conf_steer) {
					case STEER_CONF_LEFT:
						steeringCalibrationState = STEER_C_RIGHT_CONFIRMED;
					break;
					case STEER_CONF_FAILED:
						steeringCalibrationState = STEER_C_FAIL;
					break;
				}
			}
			else if (steer_calib_timed_out == true) {
				steeringCalibrationState = STEER_C_TIMEOUT;
			}
			break;
						
		case STEER_C_RIGHT_CONFIRMED:
			DrawSteerCalibScreen();
			if (ack_pressed == true) {
				confMsg->conf_steer = STEER_CONF_DEFAULT;
				steeringCalibrationState = STEER_C_OFF;
				selected = MAIN_MENU_POS;
				steer_calib_timed_out = false;
			}
			break;
		case STEER_C_FAIL:
			DrawSteerCalibScreen();
			if (ack_pressed == true) {
				confMsg->conf_steer = STEER_CONF_DEFAULT;
				steeringCalibrationState = STEER_C_OFF;
				selected = MAIN_MENU_POS;
				steer_calib_timed_out = false;	
			}
		break;
		case STEER_C_TIMEOUT:
			DrawSteerCalibScreen();
			if (ack_pressed == true) {
				confMsg->conf_steer = STEER_CONF_DEFAULT;
				steeringCalibrationState = STEER_C_OFF;
				selected = MAIN_MENU_POS;
				steer_calib_timed_out = false;
			}
		break;
	}
}


static EPedalPosition getTorquePedalPosition(SensorPhysicalValues *sensorPhysicalValue) {
	if ( (sensorPhysicalValue->torque_encoder_ch0 < TORQUE_PEDAL_IN_THRESHOLD ) && (sensorPhysicalValue->torque_encoder_ch1 < TORQUE_PEDAL_IN_THRESHOLD) ) {
		return PEDAL_OUT;
	}
	else return PEDAL_IN;
}
static EPedalPosition getBrakePedalPosition(SensorPhysicalValues *sensorPhysicalValue) {
	if ((sensorPhysicalValue->brake_pressure_fr < BRAKE_PEDAL_IN_THRESHOLD)  && (sensorPhysicalValue->brake_pressure_fl < BRAKE_PEDAL_IN_THRESHOLD) )  { //brakePedalEngagedThreshold) {
		return PEDAL_OUT;
	}
	else return PEDAL_IN;
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
	//selected = prev_selected;
}
static void vMenuUpdateCallback(TimerHandle_t pxTimer) {
	uint8_t menu_id;
	menu_id = (uint8_t) pvTimerGetTimerID(pxTimer);
	switch (menu_id) {
		case 0: // Main Screen
		menuUpdate.update_menu = true;
		//xTimerReset(timerMenuUpdate[0],0);
		break;
		case 1:
		menuUpdate.update_procedures = true;
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
static void iAmAliveTimerCallback(TimerHandle_t pxTimer) {
	send_alive = true;
}

static void initSensorRealValueStruct(SensorPhysicalValues *sensorReal) {
	sensorReal->torque_encoder_ch0 = 0;
	sensorReal->torque_encoder_ch1 = 0;
	
	sensorReal->brake_pressure_fl = 0;
	sensorReal->brake_pressure_fr = 0;
	
	sensorReal->steering_enc_data = 0;
	sensorReal->cooling_temperature = 0;
	sensorReal->gearbox_temperature = 0;
	
	sensorReal->BMS_max_temp = 0;
	sensorReal->BMS_min_temp = 0;
	sensorReal->BMS_max_temp_cell_id = 0;
	sensorReal->BMS_min_temp_cell_id = 0;
	sensorReal->current_counter = 0;
	
	sensorReal->GLVBMS_max_temp = 0;
	sensorReal->GLVBMS_max_temp_cell_id = 0;
	sensorReal->GLVBMS_min_temp = 0;
	sensorReal->GLVBMS_min_temp_cell_id = 0;
	
	sensorReal->battery_voltage = 0;
	sensorReal->min_cell_voltage = 0;
	sensorReal->min_cell_id = 0;
	sensorReal->max_cell_voltage = 0;
	sensorReal->max_cell_id = 0;
	
	sensorReal->GLV_max_cell_id = 0;
	sensorReal->GLV_min_cell_id = 0;
	sensorReal->GLV_voltage_max_cell = 0;
	sensorReal->GLV_voltage_min_cell = 0;
	sensorReal->GLV_voltage = 0;
	
	sensorReal->GLV_battery_percent = 100;
	sensorReal->HV_battery_percent = 100;
	
	sensorReal->IMU_rot_x = 0;
	sensorReal->IMU_rot_y = 0;
	sensorReal->IMU_rot_z = 0;
	
	sensorReal->IMU_G_x = 0;
	sensorReal->IMU_G_y = 0;
	sensorReal->IMU_G_z = 0;
	
	sensorReal->IMU_pos_x = 0;
	sensorReal->IMU_pos_y = 0;

	sensorReal->IMU_vel_x = 0;
	sensorReal->IMU_vel_y = 0;
	sensorReal->IMU_vel_z = 0;
	
	

}
static void initSensorValueStruct(SensorValues *sensorValue) {
	sensorValue->bms_discharge_limit = 0;
	sensorValue->brake_pressure_fr = 0;
	sensorValue->brake_pressure_fl = 0;
	sensorValue->temp_sensor_cooling = 0;
	sensorValue->temp_sensor_gearbox = 0;

}
static void initStatusStruct(StatusMsg *status) {
	status->shut_down_circuit_closed = false;
}
static void initConfirmationMessagesStruct(ConfirmationMsgs *confMsg) {
	confMsg->drive_disabled_confirmed	= false;
	confMsg->drive_enabled_confirmed	= false;
	confMsg->lc_request_confirmed		= false;
	confMsg->lc_ready					= false;
	confMsg->lc_off						= false;
	confMsg->conf_trq_ch0				= TRQ_DEFAULT;
	confMsg->conf_trq_ch1				= TRQ_DEFAULT;
	confMsg->ECU_parameter_confirmed	= false;
	}
static void initErrorMessagesStruct(ModuleError *error) {
	error->BMS_state_vector = 0;
	error->inverter_data_status = 0;
	error->bspd_status = 0xFF;
	error->ECU_implausibility = 0;
	error->IMD_status = 0;
	error->ams_error = false;
	error->imd_error = false;
	error->ecu_error   = 0;
	error->bms_fault   = 0;
	error->bms_warning = 0;
}
static void initParameterStruct(ParameterValue *parameter) {
	parameter->min_torque = 0;
	parameter->torque = 50;
	parameter->confirmed_torque = 50;
	parameter->max_torque = 100;
	
	parameter->confirmed_kers_value = 5;
	parameter->min_kers_value = 0;
	parameter->max_kers_value = 20;
	parameter->kers_value = 10;
	
	parameter->confirmed_traction_control_value = 1;
	parameter->traction_control_value = 1;
	
	parameter->min_fan_duty_cycle	= 0;
	parameter->max_fan_duty_cycle	= 100;
	parameter->all_fan_setting		= 0;
	parameter->radiator_fan_value	= 0;
	parameter->mono_fan_value		= 0;
	parameter->battery_fan_value	= 0;
	parameter->pump_setting_value   = 0;		
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
	devices->BMS == ALIVE && devices->GLVBMS == ALIVE && devices->STEER_POS == ALIVE && devices->IMD == ALIVE) {
		return true;
	}
	else {
		return false;
	}
}

static void setParameterBasedOnConfirmation(ParameterValue *parameter) {
	
	switch (menu[selected].current_setting) {
		case TORQUE_SETTING:
			if (parameter_confirmation_timed_out == true) {
				parameter->torque = parameter->confirmed_torque;
			}
			else {
				parameter->confirmed_torque = parameter->torque;
			}
		break;	
		case KERS_SETTING:
			if (parameter_confirmation_timed_out == true) {
				parameter->kers_value = parameter->confirmed_kers_value;
			}
			else {
				parameter->confirmed_kers_value = parameter->kers_value;
			}
		break;
		case TRACTION_CONTROL_SETTING:
			if (parameter_confirmation_timed_out == true) {
				parameter->traction_control_value = parameter->confirmed_traction_control_value;
			}
			else {
				parameter->confirmed_traction_control_value = parameter->traction_control_value;
				if (parameter->confirmed_traction_control_value == 1) {
					tractionControlState = TRACTION_CONTROL_ON;
				}
				else {
					tractionControlState = TRACTION_CONTROL_OFF;
				}
			}
		break;
	}
}



static void createAndStartMenuUpdateTimers() {
	timerMenuUpdate[0] = xTimerCreate("MainScreen",100/portTICK_RATE_MS, pdTRUE, (void *) 0 ,vMenuUpdateCallback);
	timerMenuUpdate[1] = xTimerCreate("procedure",50/portTICK_RATE_MS, pdTRUE, (void *) 1 ,vMenuUpdateCallback);
	xTimerStart(timerMenuUpdate[0],0);
	xTimerStart(timerMenuUpdate[1],0);
// 	if (timerMenuUpdate[0] == NULL) {
// 		
// 	}
// 	else if (xTimerStart(timerMenuUpdate[0],0) != pdPASS) {
// 		
// 	}
}

//***********************************************************************************
//------------------------------------CALCULATION FUNCTIONS-------------------------//
//***********************************************************************************
static void sensorValueToRealValue(SensorValues *sensorValue,SensorPhysicalValues *sensorPhysicalValue ) {
	//Formulas for converting raw sensor data to useful data here
}

//***********************************************************************************
//------------------------------------DRAWING FUNCTIONS----------------------------//
//***********************************************************************************

static void DrawMainScreen(SensorPhysicalValues *sensor,DeviceState *devices) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	uint8_t text_font = 21;
	DrawHighVoltageSymbol();
	cmd(COLOR_RGB(255,255, 255));
	cmd_text(15,10,text_font,OPT_FLAT,"TRACTIVE SYSTEM");
	cmd_text(15,55,text_font,OPT_FLAT,"DRIVE ENABLE");
	cmd_text(15,100,text_font,OPT_FLAT, "TRACTION CONTROL");
	cmd_text(15,145,text_font,OPT_FLAT,"LAUNCH CONTROL");
	//cmd_text(2,100,23,OPT_FLAT,"DEVICES");
	
	
	switch (carState) {
		case TRACTIVE_SYSTEM_OFF:
			cmd(COLOR_RGB(255,0,0));
			cmd_text(190,10,text_font,OPT_FLAT,"OFF"); // TS OFF
			cmd_text(190,55,text_font,OPT_FLAT,"OFF"); // Drive enable OFF
			cmd_text(190,145,text_font,OPT_FLAT, "OFF"); // Launch control
			break;
		case TRACTIVE_SYSTEM_ON:
			cmd(COLOR_RGB(0,255,0));
			cmd_text(190,10,text_font,OPT_FLAT,"ON"); // TS ON
			cmd(COLOR_RGB(255,0,0));
			cmd_text(190,55,text_font,OPT_FLAT,"OFF"); // Drive enable OFF
			cmd_text(190,145,text_font,OPT_FLAT, "OFF"); // Launch control
			break;
		case DRIVE_ENABLED:
			cmd(COLOR_RGB(0,255,0));
			cmd_text(190,10,text_font,OPT_FLAT,"ON"); // TS ON
			cmd_text(190,55,text_font,OPT_FLAT,"ON"); // Drive enable ON
			cmd(COLOR_RGB(255,0,0));
			cmd_text(190,145,text_font,OPT_FLAT, "OFF"); // Launch control
			break;
		case LC_ARMED:
			cmd(COLOR_RGB(0,255,0));
			cmd_text(190,10,text_font,OPT_FLAT,"ON"); // TS ON
			cmd_text(190,55,text_font,OPT_FLAT,"ON"); // Drive enable ON
			cmd_text(190,145,text_font,OPT_FLAT, "ON"); // Launch control
			break;
	}
	if (tractionControlState == TRACTION_CONTROL_ON) {
		cmd(COLOR_RGB(0,255,0));
		cmd_text(190,100,text_font,OPT_FLAT, "ON");
	}
	else {
		cmd(COLOR_RGB(255,0,0));
		cmd_text(190,100,text_font,OPT_FLAT, "OFF");
	}
	//Modules ok, battery temp and motor temp
	/*if (checkDeviceStatus(devices)) {
		cmd(COLOR_RGB(0,250,0));
		cmd_text(190,10,23,OPT_FLAT,"OK");
	}
	else {
		cmd(COLOR_RGB(250,0,0));
		cmd_text(190,100,23,OPT_FLAT,"NR");
	}*/
	
	cmd(BEGIN(LINE_STRIP));
	cmd(COLOR_RGB(60,80,110)); 
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

	DrawLowVoltageBattery(sensor->GLV_battery_percent);
	DrawHighVoltageBattery(sensor->HV_battery_percent);
	DrawParallellogramMainScreen();
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}

static void DrawParallellogramMainScreen() {
	uint16_t x_pos = 2;
	uint8_t x_shift = 10;
	uint8_t y_pos = 1;
	uint8_t y_shift = 35;
	uint8_t x_width = 225;
	uint8_t y_spacing = 45;
	
	cmd(LINE_WIDTH(2*16));
	cmd(COLOR_RGB(60,80,110));
	for (uint8_t i = 0; i < 4; i++) {
		cmd(BEGIN(LINE_STRIP));
		cmd(VERTEX2F( (x_pos+x_shift)*16,(y_pos+y_shift+y_spacing*i)*16));
		cmd(VERTEX2F(x_pos*16,(y_pos+y_spacing*i)*16));
		cmd(VERTEX2F((x_pos+x_width)*16, (y_pos+y_spacing*i)*16));
		cmd(VERTEX2F((x_pos+x_width+x_shift)*16,(y_pos + y_shift+y_spacing*i)*16));
		cmd(VERTEX2F((x_pos+x_shift)*16,(y_pos+y_shift+y_spacing*i)*16));
	}
// 	cmd(VERTEX2F( (x_pos+x_shift)*16,(y_pos+y_shift)*16));
// 	cmd(VERTEX2F(x_pos*16,y_pos*16));
// 	cmd(VERTEX2F((x_pos+x_width)*16,y_pos*16));
// 	cmd(VERTEX2F((x_pos+x_width+x_shift)*16,(y_pos + y_shift)*16));
// 	cmd(VERTEX2F((x_pos+x_shift)*16,(y_pos+y_shift)*16));	
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
	cmd_number(x_center_text-15, y_center_text, 30, OPT_CENTER, battery_left_percent );
	cmd_text(x_center_text+30, y_center_text, 30, OPT_CENTER,"%");
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
	cmd_number(x_center_text-15, y_center_text, 30, OPT_CENTER, battery_left_percent );
	cmd_text(x_center_text+30, y_center_text, 30, OPT_CENTER,"%");
	//cmd_text(x_center_text+35, y_center_text, 31, OPT_CENTER, "%");
	//cmd(DISPLAY()); // display the image
	//cmd(CMD_SWAP);
	//cmd_exec();
}

static void DrawSpeedScreen(SensorPhysicalValues *sensorPhysicalValue) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(240,140,31,OPT_CENTER,"120");
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawSystemMonitorScreen(ModuleError *error,SensorPhysicalValues *val) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	cmd(COLOR_RGB(255,255,255));
	cmd_text(240,15,30,OPT_CENTER,"SYSTEM MONITOR");
	cmd(COLOR_RGB(255,255,255));

	
	uint32_t trq_data_0 = val->torque_encoder_ch0;
	uint32_t trq_data_1 = val->torque_encoder_ch1;
	cmd_text(60, 200, 26, OPT_CENTER, "TRQ ENC 0");
	if (val->torque_encoder_ch0 > 1000) {
		cmd_text(50,230,26,OPT_CENTER,"ERROR");
	}
	else {
		cmd_number(50, 230, 30, OPT_CENTER, trq_data_0);
	}
	
	
	cmd_text(60, 130, 26, OPT_CENTER, "TRQ ENC 1");
	if (val->torque_encoder_ch1 > 1000) {
		cmd_text(50,160,26,OPT_CENTER,"ERROR");
	}
	else {
		cmd_number(50, 160, 30, OPT_CENTER, trq_data_1);
	}
	//cmd_text(95, 235, 22, OPT_CENTER, "%");
	//cmd_text(95, 165, 22, OPT_CENTER, "%");
	
	
	cmd_text(180, 200, 26, OPT_CENTER, "BRK FL");
	cmd_number(165, 230, 31, OPT_CENTER, val->brake_pressure_fl);
	//cmd_text(225, 235, 22, OPT_CENTER, "%");
	
	cmd_text(180, 130, 26, OPT_CENTER, "BRK FR");
	cmd_number(165, 160, 31, OPT_CENTER, val->brake_pressure_fr);
	//cmd_text(225, 165, 22, OPT_CENTER, "%");
	
	cmd_text(300, 200, 26, OPT_CENTER, "STEER ANGLE");
	cmd_number(290, 230, 31, OPT_CENTER, val->steering_enc_data);
	
	
	cmd_text(55, 60, 28, OPT_CENTER, "ECU ERROR");
	cmd_text(55, 90, 26, OPT_CENTER, ecu_error_names[error->ecu_error]);
	
	
	cmd_text(300, 50, 28, OPT_CENTER, "BMS STATE VECTOR");
	//bit0 = overvoltage, bit1 = undervoltage, bit2 = overcurrent, bit3 = overtemp, bit 4 = vic status,
	if ( error->BMS_state_vector & BMS_OVER_VOLTAGE) {
		cmd(COLOR_RGB(255,0,0));
		cmd_text(200, 95, 30, OPT_CENTER, "OV");
	}
	else {
		cmd(COLOR_RGB(0,255,0));
		cmd_text(200, 95, 30, OPT_CENTER, "OV");
	}
	if (error->BMS_state_vector & BMS_UNDER_VOLTAGE) {
		cmd(COLOR_RGB(255,0,0));
		cmd_text(250, 95, 30, OPT_CENTER, "UV");
	}
	else {
		cmd(COLOR_RGB(0,255,0));
		cmd_text(250, 95, 30, OPT_CENTER, "UV");
	}
	
	if (error->BMS_state_vector & BMS_OVER_CURRENT) {
		cmd(COLOR_RGB(255,0,0));
		cmd_text(300, 95, 30, OPT_CENTER, "OC");
	}
	else {
		cmd(COLOR_RGB(0,255,0));
		cmd_text(300, 95, 30, OPT_CENTER, "OC");
	}
	if (error->BMS_state_vector & BMS_OVER_TEMPERATURE) {
		cmd(COLOR_RGB(255,0,0));
		cmd_text(350, 95, 30, OPT_CENTER, "OT");
	}
	else {
		cmd(COLOR_RGB(0,255,0));
		cmd_text(350, 95, 30, OPT_CENTER, "OT");
	}
	if (error->BMS_state_vector & BMS_VIC_STATUS) {
		cmd(COLOR_RGB(255,0,0));
		cmd_text(400, 95, 30, OPT_CENTER, "VIC");
	}
	else {
		cmd(COLOR_RGB(0,255,0));
		cmd_text(400, 95, 30, OPT_CENTER, "VIC");
	}
// 	cmd_text(300, 95, 30, OPT_CENTER, "OC");
// 	cmd_text(350, 95, 30, OPT_CENTER, "OT");
// 	cmd_text(400, 95, 30, OPT_CENTER, "VIC");
	
	
// 	cmd_text(360, 60, 28, OPT_CENTER, "BMS fault code");
// 	cmd_text(360, 120, 28, OPT_CENTER, "BMS warning");
// 	cmd_text(365, 90, 26, OPT_CENTER, bms_fault_names[error->bms_fault]);
// 	cmd_text(365, 150, 26, OPT_CENTER, bms_warning_names[error->bms_warning]);
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	
}
static void DrawIMUScreen(SensorPhysicalValues *sensorData) {
	uint8_t f_txt = 26;
	uint8_t f_nr = 25;
	
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd(COLOR_RGB(255,255,255));
	cmd_text(240,15,30,OPT_CENTER,"IMU DATA");
	//***************************************************************//
	//***********************COLUMN 1********************************//
	//***************************************************************//
	cmd_text(60, 60, 26, OPT_CENTER, "ROT X");
	Draw4DigitFloat(50,90,f_nr,sensorData->IMU_rot_x);
	
	cmd_text(60, 130, 26, OPT_CENTER, "ROT Y");
	Draw4DigitFloat(50,160,f_nr,sensorData->IMU_rot_y);
	
	cmd_text(60, 200, f_txt, OPT_CENTER, "ROT Z");
	Draw4DigitFloat(50,230,f_nr,sensorData->IMU_rot_z);
	//***************************************************************//
	//***********************COLUMN 2********************************//
	//***************************************************************//
	cmd_text(180, 60, 26, OPT_CENTER, "G FORCE X");
	Draw4DigitFloat(170,90,f_nr,sensorData->IMU_G_x);

	cmd_text(180, 130, 26, OPT_CENTER, "G FORCE Y" );
	Draw4DigitFloat(170,160,f_nr,sensorData->IMU_G_y);
	
	cmd_text(180, 200, 26, OPT_CENTER, "G FORCE Z");
	Draw4DigitFloat(170,230,f_nr,sensorData->IMU_G_z);
	//***************************************************************//
	//***********************COLUMN 3********************************//
	//***************************************************************//
	cmd_text(290, 60, f_txt, OPT_CENTER, "POS X" );
	Draw4DigitFloat(280,90,f_nr,sensorData->IMU_pos_x);
	
	cmd_text(290, 130, f_txt, OPT_CENTER, "POS Y" );
	Draw4DigitFloat(280,160,f_nr,sensorData->IMU_pos_y);
	//***************************************************************//
	//***********************COLUMN 4********************************//
	//***************************************************************//
	cmd_text(400, 60, f_txt, OPT_CENTER, "VEL X" );
	Draw4DigitFloat(400,90,f_nr,sensorData->IMU_vel_x);
	
	cmd_text(400, 130, f_txt, OPT_CENTER, "VEL Y" );
	Draw4DigitFloat(400,160,f_nr,sensorData->IMU_vel_y);
	
	cmd_text(420, 200, 26, OPT_CENTER, "VEL Z" );
	Draw4DigitFloat(410,230,f_nr,sensorData->IMU_vel_z);
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void Draw4DigitFloat(uint16_t x, uint16_t y, uint8_t font_size, float f) {
	int integer_part; 
	int fractional_part;
	integer_part = (int) f;
	if (integer_part < 0 ) {
		cmd_text(x-29,y,font_size,OPT_CENTER,"-");
		integer_part = -(int) f;
		f = -f;
	}
	
	fractional_part = (int) ( (f- integer_part)*10);
	cmd_number(x,y,font_size,OPT_CENTER,integer_part);
	cmd_text(x+30,y,font_size,OPT_CENTER,".");
	cmd_number(x+40,y,font_size,OPT_CENTER,fractional_part);
}
static void Draw3DigitFloat(uint16_t x, uint16_t y, uint8_t font_size, float f) {
	int integer_part;
	int fractional_part;
	integer_part = (int) f;
	if (integer_part < 0 ) {
		cmd_text(x-20,y,font_size,OPT_CENTER,"-");
		integer_part = -(int) f;
		f = -f;
	}
	
	fractional_part = (int) ( (f- integer_part)*10);
	cmd_number(x,y,font_size,OPT_CENTER,integer_part);
	cmd_text(x+20,y,font_size,OPT_CENTER,".");
	cmd_number(x+30,y,font_size,OPT_CENTER,fractional_part);
}

static void DrawBatteryInfoScreen(SensorPhysicalValues *tempvolt) {
	uint8_t f_txt = 26;
	uint8_t f_nr = 25;
	
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd(COLOR_RGB(255,255,255));
	cmd_text(240,15,30,OPT_CENTER,"BATTERY INFO");
	
	cmd(COLOR_RGB(255,255,255));
	cmd_text(60, 60, 26, OPT_CENTER, "BMS MAX");
	cmd_number(105, 60, 29, OPT_CENTER,tempvolt->BMS_max_temp_cell_id);
	if(tempvolt->BMS_max_temp > BMS_MAX_TEMP_TRESHOLD){
		cmd(COLOR_RGB(255,0,0));
		Draw3DigitFloat(50,90,29,tempvolt->BMS_max_temp);
		//cmd_number(50, 90, 31, OPT_CENTER, tempvolt->BMS_max_temp);
		cmd_text(100, 90, 29, OPT_CENTER, "C");
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		Draw3DigitFloat(50,90,29,tempvolt->BMS_max_temp);
		//cmd_number(50, 90, 31, OPT_CENTER, tempvolt->BMS_max_temp);
		cmd_text(100, 90, 29, OPT_CENTER, "C");
	}
	
	cmd_text(60, 130, 26, OPT_CENTER, "BMS MIN");
	cmd_number(105, 130, 29, OPT_CENTER,tempvolt->BMS_min_temp_cell_id);
	Draw3DigitFloat(50,160,29,tempvolt->BMS_min_temp);
	//cmd_number(50, 160, 31, OPT_CENTER, tempvolt->BMS_min_temp);
	cmd_text(100, 160, 29, OPT_CENTER, "C");
	
	cmd(COLOR_RGB(255,255,255));
	cmd_text(60, 200, f_txt, OPT_CENTER, "ACC. CURR");
	Draw3DigitFloat(50,230,29,tempvolt->current_counter);
	//cmd_number(50, 230, 31, OPT_CENTER, 0);
	cmd_text(100, 230, 22, OPT_CENTER, "Ah");
	
	
	cmd_text(180, 60, 26, OPT_CENTER, "GLVBMS MAX");
	//cmd_number(235, 60, 27, OPT_CENTER, tempvolt->GLVBMS_max_temp_cell_id);
	if(tempvolt->GLVBMS_max_temp > GLVBMS_MAX_TEMP_THRESHOLD){
		cmd(COLOR_RGB(255,0,0));
		Draw3DigitFloat(170,90,29,tempvolt->GLVBMS_max_temp);
		
		//cmd_number(170, 90, 31, OPT_CENTER, tempvolt->GLVBMS_max_temp);
		cmd_text(220, 90, 29, OPT_CENTER, "C");
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		Draw3DigitFloat(170,90,29,tempvolt->GLVBMS_max_temp);
		//cmd_number(170, 90, 31, OPT_CENTER, tempvolt->GLVBMS_max_temp);
		cmd_text(220, 90, 29, OPT_CENTER, "C");
	}
	
	cmd_text(180, 130, 26, OPT_CENTER, "GLVBMS MIN" );
	//cmd_number(235, 130, 27, OPT_CENTER, 0);//tempvolt->GLVBMS_min_temp_cell_id);
	Draw3DigitFloat(170,160,29,tempvolt->GLVBMS_min_temp);
	//cmd_number(170, 160, 31, OPT_CENTER, tempvolt->GLVBMS_min_temp);
	cmd_text(220, 160, 29, OPT_CENTER, "C");
	
	
	int integer_part = (int) tempvolt->Inverter_voltage;
	int fractional_part = (int) ( (tempvolt->Inverter_voltage - integer_part)*10);
	cmd_text(180, 200, 26, OPT_CENTER, "INV VOLT" );
	cmd_number(170,230,f_nr,OPT_CENTER,integer_part);
	cmd_text(170+40,230,31,OPT_CENTER,".");
	cmd_number(170+53,230,f_nr,OPT_CENTER,fractional_part);
	

	
	//cmd_number(170, 230, 31, OPT_CENTER, 0);
	//cmd_text(215, 235, 29, OPT_CENTER, "C");


	cmd_text(290, 60, f_txt, OPT_CENTER, "BMS MAX" );
	cmd_text(330, 100, f_txt, OPT_CENTER, "V");
	cmd_number(335, 60, 29, OPT_CENTER,tempvolt->max_cell_id);
	if( tempvolt->max_cell_voltage > MAX_CELL_VOLTAGE_TRESHOLD ){
		cmd(COLOR_RGB(255,0,0));
		DrawFloat(280,95,f_nr,tempvolt->max_cell_voltage);
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		DrawFloat(280,95,f_nr,tempvolt->max_cell_voltage);
	}
	
	cmd_text(290, 130, f_txt, OPT_CENTER, "BMS MIN" );
	cmd_text(330, 165, f_txt, OPT_CENTER, "V");
	cmd_number(330, 130, 29, OPT_CENTER, tempvolt->min_cell_id);
	
	if( tempvolt->min_cell_voltage < MIN_CELL_VOLTAGE_TRESHOLD )   {
		cmd(COLOR_RGB(255,0,0));
		DrawFloat(280,160,f_nr,tempvolt->min_cell_voltage);
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		DrawFloat(280,160,f_nr,tempvolt->min_cell_voltage);
	}
	integer_part = (int) tempvolt->battery_voltage;
	fractional_part = (int) ( (tempvolt->battery_voltage - integer_part)*10);
	
	cmd_text(300, 200, 26, OPT_CENTER, "BMS");
	cmd_text(360, 235, 22, OPT_CENTER, "V");
	if ( (tempvolt->battery_voltage > BATTERY_PACK_MAX_VOLTAGE_TRESHOLD) || (tempvolt->battery_voltage < BATTERY_PACK_MIN_VOLTAGE_TRESHOLD) ) {
		cmd(COLOR_RGB(255,0,0));
		cmd_number(290,230,f_nr,OPT_CENTER,integer_part);
		cmd_text(290+40,230,31,OPT_CENTER,".");
		cmd_number(290+53,230,f_nr,OPT_CENTER,fractional_part);
		//DrawFloat(290,230,f_nr,tempvolt->battery_voltage);
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		cmd_number(290,230,f_nr,OPT_CENTER,integer_part);
		cmd_text(290+40,230,31,OPT_CENTER,".");
		cmd_number(290+53,230,f_nr,OPT_CENTER,fractional_part);
		//DrawFloat(290,230,f_nr,tempvolt->battery_voltage);
	}

	cmd_text(400, 60, f_txt, OPT_CENTER, "GLVBMS MAX" );
	cmd_text(460, 100, f_txt, OPT_CENTER, "V");
	cmd_number(460, 60, 29, OPT_CENTER,tempvolt->GLV_max_cell_id);
	if (tempvolt->GLV_voltage_max_cell > GLV_MAX_CELL_VOLTAGE_TRESHOLD) {
		cmd(COLOR_RGB(255,0,0));
		DrawFloat(400,95,f_nr,tempvolt->GLV_voltage_max_cell);
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		DrawFloat(400,95,f_nr,tempvolt->GLV_voltage_max_cell);
	}
	
	cmd_text(400, 130, f_txt, OPT_CENTER, "GLVBMS MIN" );
	cmd_text(460, 160, f_txt, OPT_CENTER, "V");
	cmd_number(460, 130, 29, OPT_CENTER,tempvolt->GLV_min_cell_id);
	if (tempvolt->GLV_voltage_max_cell < GLV_MIN_CELL_VOLTAGE_TRESHOLD) {
		cmd(COLOR_RGB(255,0,0));
		DrawFloat(400,160,f_nr,tempvolt->GLV_voltage_min_cell);
		cmd(COLOR_RGB(255,255,255));
	}
	else {
		DrawFloat(400,160,f_nr,tempvolt->GLV_voltage_min_cell);
	}
	
	cmd_text(420, 200, 26, OPT_CENTER, "GLV" );
	integer_part = (int) tempvolt->GLV_voltage;
	fractional_part = (int) ( (tempvolt->GLV_voltage - integer_part)*10);
	cmd_number(410,230,f_nr,OPT_CENTER,integer_part);
	cmd_text(410+30,230,31,OPT_CENTER,".");
	cmd_number(410+40,230,f_nr,OPT_CENTER,fractional_part);
	
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}

static void DrawTSStatusScreen(SensorPhysicalValues *tempvolt, ModuleError *Error) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd(COLOR_RGB(255,255,255));
	uint8_t txt_size = 26;
	
	//**************** ECU **********************************//
	cmd_text(85, 25, 28, OPT_CENTER, "ECU IMPL");
	if (Error->ECU_implausibility == ECU_NO_IMPLAUSIBILITY) {
		cmd_text(85, 50, txt_size, OPT_CENTER, "NONE");
	}
	else if (Error->ECU_implausibility == ECU_TPS_MISMATCH) {
		cmd_text(85, 50, txt_size, OPT_CENTER, "TPS MISMATCH");
	}
	else if (Error->ECU_implausibility == ECU_TPS_BPS_IMPLAUSIBILITY) {
		cmd_text(85, 50, txt_size, OPT_CENTER, "ECU BSPD");
	}
	else if (Error->ECU_implausibility == ECU_SENSOR_ERROR) {
		cmd_text(85, 50, txt_size, OPT_CENTER, "SENSOR ERROR");
	}
	else if (Error->ECU_implausibility == ECU_OUTDATED_DATA) {
		cmd_text(85, 50, txt_size, OPT_CENTER, "OUTDATED DATA");
	}
	
	//************************ IMD *****************************//
	cmd_text(85, 80, 28, OPT_CENTER, "IMD STATUS");
	if (Error->IMD_status == IMD_OFF) {
		cmd_text(85, 105, txt_size, OPT_CENTER, "OFF");
	}
	else if (Error->IMD_status == IMD_NORMAL) {
		cmd_text(85, 105, txt_size, OPT_CENTER, "NORMAL");
	}
	else if (Error->IMD_status == IMD_UNDER_VOLTAGE) {
		cmd_text(85, 105, txt_size, OPT_CENTER, "UNDER VOLTAGE");
	}
	else if (Error->IMD_status == IMD_SPEED_START) {
		cmd_text(85, 105, txt_size, OPT_CENTER, "SPEED START");
	}
	else if (Error->IMD_status == IMD_ERROR) {
		cmd_text(85, 105, txt_size, OPT_CENTER, "ERROR");
	}
	else if (Error->IMD_status == IMD_GROUND_ERROR) {
		cmd_text(85, 105, txt_size, OPT_CENTER, "GROUND ERROR");
	}
	
	
	//************************ INVERTER ************************//
	cmd_text(350, 25, 28, OPT_CENTER, "INV ENC POS");
	if ( Error->inverter_data_status & INVERTER_ENCODER_NOT_FOUND) {
		cmd_text(350, 50, txt_size, OPT_CENTER, "NOT FOUND");
	}
	else {
		cmd_text(350, 50, txt_size, OPT_CENTER, "POSITION FOUND");
	}
	
	cmd_text(350, 80, 28, OPT_CENTER, "INV STATUS");
	if ( Error->inverter_data_status & INVERTER_ENABLED) {
		cmd_text(350, 105, txt_size, OPT_CENTER, "ENABLED");
	}
	else {
		cmd_text(350, 105, txt_size, OPT_CENTER, "DISABLED");
	}
	

	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
	
}

static void DrawSensorInformationScreen(SensorPhysicalValues *sensorData) {
	uint8_t f_txt = 26;
	uint8_t f_nr = 25;
	
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd(COLOR_RGB(255,255,255));
	cmd_text(240,15,30,OPT_CENTER,"SENSOR DATA");
	//***************************************************************//
	//***********************COLUMN 1********************************//
	//***************************************************************//
	cmd_text(60, 60, 26, OPT_CENTER, "GEARBOX TMP");
	Draw4DigitFloat(50,90,f_nr,sensorData->gearbox_temperature);
	
	cmd_text(60, 130, 26, OPT_CENTER, "COOL TEMP");
	Draw4DigitFloat(50,160,f_nr,sensorData->cooling_temperature);
	
// 	cmd_text(60, 200, f_txt, OPT_CENTER, "EMPTY");
// 	Draw4DigitFloat(50,230,f_nr,sensorData->IMU_rot_z);
	//***************************************************************//
	//***********************COLUMN 2********************************//
	//***************************************************************//
// 	cmd_text(180, 60, 26, OPT_CENTER, "G FORCE X");
// 	Draw4DigitFloat(170,90,f_nr,sensorData->IMU_G_x);
// 
// 	cmd_text(180, 130, 26, OPT_CENTER, "G FORCE Y" );
// 	Draw4DigitFloat(170,160,f_nr,sensorData->IMU_G_y);
// 	
// 	cmd_text(180, 200, 26, OPT_CENTER, "G FORCE Z");
// 	Draw4DigitFloat(170,230,f_nr,sensorData->IMU_G_z);
	//***************************************************************//
	//***********************COLUMN 3********************************//
	//***************************************************************//
// 	cmd_text(290, 60, f_txt, OPT_CENTER, "POS X" );
// 	Draw4DigitFloat(280,90,f_nr,sensorData->IMU_pos_x);
// 	
// 	cmd_text(290, 130, f_txt, OPT_CENTER, "POS Y" );
// 	Draw4DigitFloat(280,160,f_nr,sensorData->IMU_pos_y);
	//***************************************************************//
	//***********************COLUMN 4********************************//
	//***************************************************************//
// 	cmd_text(400, 60, f_txt, OPT_CENTER, "VEL X" );
// 	Draw4DigitFloat(400,90,f_nr,sensorData->IMU_vel_x);
// 	
// 	cmd_text(400, 130, f_txt, OPT_CENTER, "VEL Y" );
// 	Draw4DigitFloat(400,160,f_nr,sensorData->IMU_vel_y);
// 	
// 	cmd_text(420, 200, 26, OPT_CENTER, "VEL Z" );
// 	Draw4DigitFloat(410,230,f_nr,sensorData->IMU_vel_z);
	
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
			cmd(COLOR_RGB(255,255,20));
			//cmd(COLOR_RGB(70,50,110));
			cmd(LINE_WIDTH(16*1));
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
static void DrawMainMenuDown() {
	
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
	//pos = pos +1;
	cmd(COLOR_RGB(255,255,255));
	for (pos; pos <= (end_position-4); pos ++) {
		if ( pos == selected) {
			
			cmd(BEGIN(RECTS));
			cmd(COLOR_RGB(255,255,20));
			//cmd(COLOR_RGB(70,50,110));
			cmd(LINE_WIDTH(16*1));
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
			//cmd_fgcolor(0xb9b900);
			cmd_fgcolor(0x322984);
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

static void DrawECUAdjustmentScreen(ParameterValue *parameter) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen

	uint8_t menu_pos = ECU_SETTINGS_MENU_POS;
	uint8_t variable_pos = ECU_SETTINGS_VARIABLES_POS;
	uint8_t end_menu_pos = menu_pos + menu[ECU_SETTINGS_MENU_POS].num_menupoints - 1;
	uint8_t end_variable_pos = variable_pos + menu[ECU_SETTINGS_VARIABLES_POS].num_menupoints -1;
	
	uint32_t y_menu_position = 56;
	uint32_t x_menu_position = 25;
	uint32_t vertical_menu_spacing = 55;
	uint8_t font_size = 27;

	cmd_text(240,20,29,OPT_CENTER,"ECU OPTIONS");
	for (menu_pos; menu_pos <= end_menu_pos ; menu_pos ++) {
		if (selected == menu_pos) {
			cmd(COLOR_RGB(0,255,0));
			cmd_text(x_menu_position,y_menu_position,font_size,OPT_FLAT,menu[menu_pos].text);
		}
		else {
			cmd(COLOR_RGB(255,255,255));
			cmd_text(x_menu_position,y_menu_position,font_size,OPT_FLAT,menu[menu_pos].text);
		}
		y_menu_position += vertical_menu_spacing;
	}
	uint32_t x_slider_position = 200;
	uint32_t y_slider_position = 60;
	uint8_t vertical_slider_spacing = 55;
	uint32_t slider_width = 190;
	uint8_t slider_heigth = 10;
	
	uint8_t x_num_adj = 34;
	uint8_t x_max_num_adj = 215;
	uint8_t y_num_adj = 10;
	uint8_t num_font_size = 27;
	
	uint8_t shape_spacing = 55;
	
	//Knob : fgcolor
	//Left of knob : COLOR_RGB
	//Right of knob : bgcolor
	uint32_t color_right = 0x605F69;
	uint32_t color_knob  = 0x0000FF;
	//static void DrawParallellogram(uint16_t y_top_left);
	
	cmd(LINE_WIDTH(16*2));
	for(variable_pos; variable_pos <= end_variable_pos; variable_pos ++) {
		switch (menu[variable_pos].current_setting) {
			case TORQUE_SETTING:
				if (selected == variable_pos) {
					cmd(COLOR_RGB(60,80,110));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,80*16));
					cmd(VERTEX2F(5*16,50*16));
					cmd(VERTEX2F(450*16,50*16));
					cmd(VERTEX2F(470*16,80*16));
					cmd(VERTEX2F(25*16,80*16));
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right); // 
					cmd(COLOR_RGB(255,255,0)); // 
				}
				else {
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right);
					cmd(COLOR_RGB(255,255,255));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,80*16));
					cmd(VERTEX2F(5*16,50*16));
					cmd(VERTEX2F(450*16,50*16));
					cmd(VERTEX2F(470*16,80*16));
					cmd(VERTEX2F(25*16,80*16));
					
				}
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,parameter->torque,100);
				cmd(COLOR_RGB(255,255,255));
				//cmd(COLOR_RGB(255,0,0));
				cmd_number(425,y_slider_position +4,num_font_size,OPT_CENTER,parameter->torque);
			break;
			
			case KERS_SETTING:
				if (selected == variable_pos) {
					cmd(COLOR_RGB(60,80,110));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(80+shape_spacing)*16));
					cmd(VERTEX2F(5*16,(50+shape_spacing)*16));
					cmd(VERTEX2F(450*16,(50+shape_spacing)*16));
					cmd(VERTEX2F(470*16,(80+shape_spacing)*16));
					cmd(VERTEX2F(25*16,(80+shape_spacing)*16));
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right); //
					cmd(COLOR_RGB(255,255,0)); //
				}
				else {
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right);
					cmd(COLOR_RGB(255,255,255));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(80+shape_spacing)*16));
					cmd(VERTEX2F(5*16,(50+shape_spacing)*16));
					cmd(VERTEX2F(450*16,(50+shape_spacing)*16));
					cmd(VERTEX2F(470*16,(80+shape_spacing)*16));
					cmd(VERTEX2F(25*16,(80+shape_spacing)*16));
				}
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,parameter->kers_value,parameter->max_kers_value);
				cmd(COLOR_RGB(255,255,255));
				//cmd(COLOR_RGB(255,0,0));
				cmd_number(425,y_slider_position +4,num_font_size,OPT_CENTER,parameter->kers_value);
				break;			
			break;
			
			case TRACTION_CONTROL_SETTING:
			// Create square and a toggle thin
				if (selected == variable_pos) {
					cmd(COLOR_RGB(60,80,110));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(80+shape_spacing*2)*16));
					cmd(VERTEX2F(5*16,(50+shape_spacing*2)*16));
					cmd(VERTEX2F(450*16,(50+shape_spacing*2)*16));
					cmd(VERTEX2F(470*16,(80+shape_spacing*2)*16));
					cmd(VERTEX2F(25*16,(80+shape_spacing*2)*16));
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right); //
					cmd(COLOR_RGB(255,255,0)); //
				}
				else {
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right);
					cmd(COLOR_RGB(255,255,255));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(80+shape_spacing*2)*16));
					cmd(VERTEX2F(5*16,(50+shape_spacing*2)*16));
					cmd(VERTEX2F(450*16,(50+shape_spacing*2)*16));
					cmd(VERTEX2F(470*16,(80+shape_spacing*2)*16));
					cmd(VERTEX2F(25*16,(80+shape_spacing*2)*16));
				}
				if ( parameter->traction_control_value == 0) {
					cmd_text(320,175,24,OPT_CENTER,"OFF");
				}
				else {
					cmd_text(320,175,24,OPT_CENTER,"ON");
				}
			break;
		}	
		y_slider_position += vertical_slider_spacing;
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawFANControlScreen(ParameterValue *parameter) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen

	uint8_t menu_pos = 51;
	uint8_t variable_pos = 51;
	uint8_t end_menu_pos = menu_pos + menu[51].num_menupoints - 1;
	uint8_t end_variable_pos = variable_pos + menu[51].num_menupoints -1;
	
	uint32_t y_menu_position = 28;
	uint32_t x_menu_position = 25;
	uint32_t vertical_menu_spacing = 40;
	uint8_t font_size = 27;

	//cmd_text(240,20,29,OPT_CENTER,"FAN CONTROL");
	
	for (menu_pos; menu_pos <= end_menu_pos ; menu_pos ++) {
		if (selected == menu_pos) {
			cmd(COLOR_RGB(0,255,0));
			cmd_text(x_menu_position,y_menu_position,font_size,OPT_FLAT,menu[menu_pos].text);
		}
		else {
			cmd(COLOR_RGB(255,255,255));
			cmd_text(x_menu_position,y_menu_position,font_size,OPT_FLAT,menu[menu_pos].text);
		}
		y_menu_position += vertical_menu_spacing;
	}
	uint32_t x_slider_position = 200;
	uint32_t y_slider_position = 30;
	uint8_t vertical_slider_spacing = 40;
	uint32_t slider_width = 190;
	uint8_t slider_heigth = 10;
	
	uint8_t x_num_adj = 34;
	uint8_t x_max_num_adj = 215;
	uint8_t y_num_adj = 10;
	uint8_t num_font_size = 27;
	
	uint8_t shape_spacing = 40; // 55
	
	//Knob : fgcolor
	//Left of knob : COLOR_RGB
	//Right of knob : bgcolor
	uint32_t color_right = 0x605F69;
	uint32_t color_knob  = 0x0000FF;
	//static void DrawParallellogram(uint16_t y_top_left);
	
	cmd(LINE_WIDTH(16*2));
	
	uint8_t py1 = 20;
	uint8_t py2 = 50; 
	
	for(variable_pos; variable_pos <= end_variable_pos; variable_pos ++) {
		switch (menu[variable_pos].current_setting) {

			case RADIATOR_FAN_SETTING:
				if (selected == variable_pos) {
					cmd(COLOR_RGB(60,py2,110));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,py2*16));
					cmd(VERTEX2F(5*16,py1*16));
					cmd(VERTEX2F(450*16,py1*16));
					cmd(VERTEX2F(470*16,py2*16));
					cmd(VERTEX2F(25*16,py2*16));
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right); //
					cmd(COLOR_RGB(255,255,0)); //
				}
				else {
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right);
					cmd(COLOR_RGB(255,255,255));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,py2*16));
					cmd(VERTEX2F(5*16,py1*16));
					cmd(VERTEX2F(450*16,py1*16));
					cmd(VERTEX2F(470*16,py2*16));
					cmd(VERTEX2F(25*16,py2*16));
				}
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,parameter->radiator_fan_value,parameter->max_fan_duty_cycle);
				cmd(COLOR_RGB(255,255,255));
				//cmd(COLOR_RGB(255,0,0));
				cmd_number(425,y_slider_position +4,num_font_size,OPT_CENTER,parameter->radiator_fan_value);
			break;
			case MONO_FAN_SETTING:
				if (selected == variable_pos) {
					cmd(COLOR_RGB(60,py2,110));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(py2+shape_spacing)*16));
					cmd(VERTEX2F(5*16,(py1+shape_spacing)*16));
					cmd(VERTEX2F(450*16,(py1+shape_spacing)*16));
					cmd(VERTEX2F(470*16,(py2+shape_spacing)*16));
					cmd(VERTEX2F(25*16,(py2+shape_spacing)*16));
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right); //
					cmd(COLOR_RGB(255,255,0)); //
				}
				else {
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right);
					cmd(COLOR_RGB(255,255,255));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(py2+shape_spacing)*16));
					cmd(VERTEX2F(5*16,(py1+shape_spacing)*16));
					cmd(VERTEX2F(450*16,(py1+shape_spacing)*16));
					cmd(VERTEX2F(470*16,(py2+shape_spacing)*16));
					cmd(VERTEX2F(25*16,(py2+shape_spacing)*16));
				}
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,parameter->mono_fan_value,parameter->max_fan_duty_cycle);
				cmd(COLOR_RGB(255,255,255));
				//cmd(COLOR_RGB(255,0,0));
				cmd_number(425,y_slider_position +4,num_font_size,OPT_CENTER,parameter->mono_fan_value);
			break;
			case BATTERY_FAN_SETTING:
				if (selected == variable_pos) {
					cmd(COLOR_RGB(60,py2,110));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*2)*16));
					cmd(VERTEX2F(5*16,(py1+shape_spacing*2)*16));
					cmd(VERTEX2F(450*16,(py1+shape_spacing*2)*16));
					cmd(VERTEX2F(470*16,(py2+shape_spacing*2)*16));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*2)*16));
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right); //
					cmd(COLOR_RGB(255,255,0)); //
				}
				else {
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right);
					cmd(COLOR_RGB(255,255,255));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*2)*16));
					cmd(VERTEX2F(5*16,(py1+shape_spacing*2)*16));
					cmd(VERTEX2F(450*16,(py1+shape_spacing*2)*16));
					cmd(VERTEX2F(470*16,(py2+shape_spacing*2)*16));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*2)*16));
				}
			
				cmd_slider(x_slider_position,y_slider_position,slider_width,slider_heigth,OPT_FLAT,parameter->battery_fan_value,parameter->max_fan_duty_cycle);
				cmd(COLOR_RGB(255,255,255));
				//cmd(COLOR_RGB(255,0,0));
				cmd_number(425,y_slider_position +4,num_font_size,OPT_CENTER,parameter->battery_fan_value);
			break;
			
			case ALL_FAN_SETTING:
				if (selected == variable_pos) {
					cmd(COLOR_RGB(60,80,110));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*3)*16));
					cmd(VERTEX2F(5*16,(py1+shape_spacing*3)*16));
					cmd(VERTEX2F(450*16,(py1+shape_spacing*3)*16));
					cmd(VERTEX2F(470*16,(py2+shape_spacing*3)*16));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*3)*16));
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right); //
					cmd(COLOR_RGB(255,255,0)); //
				}
				else {
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right);
					cmd(COLOR_RGB(255,255,255));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*3)*16));
					cmd(VERTEX2F(5*16,(py1+shape_spacing*3)*16));
					cmd(VERTEX2F(450*16,(py1+shape_spacing*3)*16));
					cmd(VERTEX2F(470*16,(py2+shape_spacing*3)*16));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*3)*16));
				}
				if ( parameter->all_fan_setting == 0) {
					cmd_text(320,155,24,OPT_CENTER,"OFF");
				}
				else { // 230 as y last value
					cmd_text(320,155,24,OPT_CENTER,"ON");
				}
			break;
			case PUMP_SETTING:
				// Create square and a toggle thin
				if (selected == variable_pos) {
					cmd(COLOR_RGB(60,80,110));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*4)*16));
					cmd(VERTEX2F(5*16,(py1+shape_spacing*4)*16));
					cmd(VERTEX2F(450*16,(py1+shape_spacing*4)*16));
					cmd(VERTEX2F(470*16,(py2+shape_spacing*4)*16));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*4)*16));
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right); //
					cmd(COLOR_RGB(255,255,0)); //
				}
				else {
					cmd_fgcolor(0x000000); // Try black knob
					cmd_bgcolor(color_right);
					cmd(COLOR_RGB(255,255,255));
					cmd(BEGIN(LINE_STRIP));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*4)*16));
					cmd(VERTEX2F(5*16,(py1+shape_spacing*4)*16));
					cmd(VERTEX2F(450*16,(py1+shape_spacing*4)*16));
					cmd(VERTEX2F(470*16,(py2+shape_spacing*4)*16));
					cmd(VERTEX2F(25*16,(py2+shape_spacing*4)*16));
				}
			
				if ( parameter->pump_setting_value == 0) {
					cmd_text(320,195,24,OPT_CENTER,"OFF");
				}
				else { // 230 as y last value
					cmd_text(320,195,24,OPT_CENTER,"ON");
				}
			break;
			
		}
		y_slider_position += vertical_slider_spacing;
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}

static void DrawDeviceStatusMenu(DeviceState *deviceState) {
	uint8_t bar_len = 100;
	// Title
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd_text(236, 20, 29, OPT_CENTER, "Device Status");
	
	// COLUMN 1
	switch (deviceState->TRQ_0) {
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
	switch (deviceState->TRQ_1) {
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
	if (deviceState->IMU == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(70, 100, bar_len, 25, 26, OPT_CENTER, "IMU");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(70, 100, bar_len, 25, 26, OPT_CENTER, "IMU");
	}
	if (deviceState->ECU == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(70, 130, bar_len, 25, 26, OPT_CENTER, "ECU");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(70, 130, bar_len, 25, 26, OPT_CENTER, "ECU");
	}
	
	if (deviceState->TEL == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(70, 160, bar_len, 25, 26, OPT_CENTER, "TEL");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(70, 160, bar_len, 25, 26, OPT_CENTER, "TEL");
	}
	if (deviceState->GLVBMS == ALIVE) {
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
	if (deviceState->INV == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(190, 70, bar_len, 25, 26, OPT_CENTER, "INV");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(190, 70, bar_len, 25, 26, OPT_CENTER, "INV");
	}
	
	if (deviceState->STEER_POS == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(190, 100, bar_len, 25, 26, OPT_CENTER, "STEER POS");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(190, 100, bar_len, 25, 26, OPT_CENTER, "STEER POS");
	}
	
	if (deviceState->IMD == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(190, 130, bar_len, 25, 26, OPT_CENTER, "IMD");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(190, 130, bar_len, 25, 26, OPT_CENTER, "IMD");
	}
	if (deviceState->FAN == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(190, 160, bar_len, 25, 26, OPT_CENTER, "FAN");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(190, 160, bar_len, 25, 26, OPT_CENTER, "FAN");
	}
	
	if (deviceState->BSPD == ALIVE) {
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
	if (deviceState->ADC_FL == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(310, 70, bar_len, 25, 26, OPT_CENTER, "ADC_FL");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(310, 70, bar_len, 25, 26, OPT_CENTER, "ADC_FL");
	}
	if (deviceState->ADC_FR == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(310, 100, bar_len, 25, 26, OPT_CENTER, "ADC_FR");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(310, 100, bar_len, 25, 26, OPT_CENTER, "ADC_FR");
	}
	if (deviceState->ADC_RR == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(310, 130, bar_len, 25, 26, OPT_CENTER, "ADC_RR");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(310, 130, bar_len, 25, 26, OPT_CENTER, "ADC_RR");
	}
	if (deviceState->ADC_RL == ALIVE) {
		cmd(COLOR_RGB(0,0,0));
		cmd_fgcolor(0x00FF00);
		cmd_button(310, 160, bar_len, 25, 26, OPT_CENTER, "ADC_RL");
	}
	else {
		cmd(COLOR_RGB(255,255,255));
		cmd_fgcolor(0xFF0000);
		cmd_button(310, 160, bar_len, 25, 26, OPT_CENTER, "ADC_RL");
	}
	
	if (deviceState->BMS == ALIVE) {
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

static void DrawDriveEnableWarning() {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	cmd_text(240,100,28,OPT_CENTER,"REMOVE FOOT FROM TORQUE PEDAL BEFORE ENABLING DRIVE" );
	cmd_text(240,130,28,OPT_CENTER,"PRESS BRAKE PEDAL BEFORE ENABLING DRIVE" );
	cmd_text(240,160,29,OPT_CENTER,"PRESS ACKNOWLEDGE" );
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawLaunchControlProcedure() {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	if (carState == LC_PROCEDURE) {
		cmd_text(5,20,27,OPT_FLAT, "LAUNCH CONTROL REQUEST ACCEPTED");
		//cmd_text(5,50,30,OPT_FLAT, "KEEP THE TORQUE PEDAL PUSHED IN");
		cmd_text(5,80,27,OPT_FLAT, "TO START COUNTDOWN PRESS ACKNOWLEDGE BUTTON");
		cmd_text(5,130,27,OPT_FLAT, "TO ABORT LAUNCH CONTROL PUSH THE BRAKES IN");
	}
	else if (carState == LC_COUNTDOWN) {
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
	else if (carState == LC_ARMED) {
		cmd_text(240,130,27,OPT_CENTER, "LAUNCH CONTROL IS ARMED. PUSH TORQUE PEDAL TO LAUNCH");
	}
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawTorqueCalibrationScreen(ConfirmationMsgs *confMsg) {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	switch (torquePedalCalibrationState) {
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
	
	switch (steeringCalibrationState) {
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
			// Reset fail counter
			can_send_to_datalogger_queue_failed = 0;
			
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
			
			cmd_text(x_position_status_text,140,font_size,OPT_FLAT,"FAILS:");
			cmd_number(425,146,font_size,OPT_CENTER,can_send_to_datalogger_queue_failed);
			
			cmd_text(x_position_status_text,200,font_size,OPT_FLAT,"USB NOT CONNECTED");
		break;
		
		case DATALOGGER_USB_CONNECTED:
			// Reset fail counter
			can_send_to_datalogger_queue_failed = 0;
			
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
	cmd_text(x+14,y,font_size,OPT_CENTER,".");
	cmd_number(x+28,y,font_size,OPT_CENTER,fractional_part);
}

static void DrawPresetMenu() {
	uint8_t pos = selected - menu[selected].position; // First position in current menu
	uint8_t end_position  = pos + menu[pos].num_menupoints - 1;  // Last position in current menu
	uint8_t position_preset_currently_in_ecu = pos + selected_preset_file;
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
	cmd(COLOR_RGB(255,255,255));
	for (pos; pos <= (end_position-4); pos ++) {
		if ( pos == selected) {
			
			cmd(BEGIN(RECTS));
			cmd(COLOR_RGB(255,255,20));
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
		else if ( pos == position_preset_currently_in_ecu) {
			cmd(BEGIN(RECTS));
			cmd(COLOR_RGB(0,255,0));
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
			cmd(COLOR_RGB(255,255,20)); 
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
		else if ( pos == position_preset_currently_in_ecu) {
			cmd(BEGIN(RECTS));
			cmd(COLOR_RGB(0,255,0));
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

static void DrawPresetProcedure() {
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	switch (presetProcedureState) {
		case PRESET_PROCEDURE_FINISHED:
			cmd_text(240,130,27,OPT_CENTER,"PRESETS SUCESSFULLY TRANSFERRED TO THE ECU");
		break;
		case PRESET_PROCEDURE_FAILED:
			cmd(COLOR_RGB(255,0,0));;
			cmd_text(240,130,27,OPT_CENTER,"PRESET PROCEDURE FAILED");
		break;
	}
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}
static void DrawPresetConfirmation() {
	uint16_t yes_x_pos = 160;
	uint16_t bar_width = 75;
	uint8_t yes_y_top_pos = 135;
	uint8_t bar_heigth = 40;
	
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	
	// Draw two rectangles with Yes and no inside
	// 
	cmd(BEGIN(RECTS));
	//cmd(LINE_WIDTH(16*5));
	if ( menu[selected].current_setting == CONFIRM_YES) {
		cmd(COLOR_RGB(250,250,0)); 
		cmd(VERTEX2F(yes_x_pos*16, yes_y_top_pos*16)); // Top left coordinates
		cmd(VERTEX2F( (yes_x_pos + bar_width)*16, (yes_y_top_pos + bar_heigth)*16 )); // Bottom rightcoordinates
		
		cmd(COLOR_RGB(100,100,100)); 
		cmd(VERTEX2F(245*16,yes_y_top_pos*16)); // Top left coordinates
		cmd(VERTEX2F( (245+bar_width)*16, (yes_y_top_pos + bar_heigth)*16)); // Bottom rightcoordinates
	}
	else {
		cmd(COLOR_RGB(100,100,100)); 
		cmd(VERTEX2F(yes_x_pos*16, yes_y_top_pos*16)); // Top left coordinates
		cmd(VERTEX2F( (yes_x_pos + bar_width)*16, (yes_y_top_pos + bar_heigth)*16 )); // Bottom rightcoordinates
		cmd(COLOR_RGB(250,250,0)); 
		cmd(VERTEX2F(245*16,yes_y_top_pos*16)); // Top left coordinates
		cmd(VERTEX2F( (245+bar_width)*16, (yes_y_top_pos + bar_heigth)*16)); // Bottom rightcoordinates
	}
	cmd(COLOR_RGB(255,255,255));
	cmd_text(yes_x_pos + (bar_width/2),yes_y_top_pos + (bar_heigth/2), 28, OPT_CENTER,"YES");
	cmd_text(245 + (bar_width/2),yes_y_top_pos + (bar_heigth/2), 28, OPT_CENTER,"NO");
	cmd_text(240,100,28,OPT_CENTER,"SEND THIS PARAMETER FILE TO THE ECU ? ");
	
	
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}

//***********************************************************************************
//--------------------------SLIDER VARIABLE UPDATE FUNCTIONS-----------------------//
//***********************************************************************************

static void adjustParameters(ERotary_direction dir, ParameterValue *parameter) {
	switch (menu[selected].current_setting) {
		case TORQUE_SETTING:
			if ( (dir == CW) && ( (parameter->torque + StepSizeVar.torque) <= parameter->max_torque )  ) {
				parameter->torque += StepSizeVar.torque;
			}
			else if ( (dir == CCW) && ( parameter->torque   >= (parameter->min_torque + StepSizeVar.torque) ) ) {
				parameter->torque -= StepSizeVar.torque;
			}
		break;
		case KERS_SETTING:
			if ( (dir == CW) && ( (parameter->kers_value + StepSizeVar.kers) <= parameter->max_kers_value )  ) {
				parameter->kers_value += StepSizeVar.kers;
			}
			else if ( (dir == CCW) && ( parameter->kers_value   >= (parameter->min_kers_value + StepSizeVar.kers) ) ) {
				parameter->kers_value -= StepSizeVar.kers;
			}
		break;
		case TRACTION_CONTROL_SETTING:
			if ( (dir == CW) && ( parameter->traction_control_value == 0  ) ) {
				parameter->traction_control_value = 1;
			}
			else if ( (dir == CCW) && (parameter->traction_control_value == 1) ) {
				parameter->traction_control_value = 0;
			}
		break;
		case RADIATOR_FAN_SETTING:
			if ( (dir == CW) && ( (parameter->radiator_fan_value + StepSizeVar.radiator_fan) <= parameter->max_fan_duty_cycle )  ) {
				parameter->radiator_fan_value += StepSizeVar.radiator_fan;
			}
			else if ( (dir == CCW) && ( parameter->radiator_fan_value   >= (parameter->min_fan_duty_cycle + StepSizeVar.radiator_fan) ) ) {
				parameter->radiator_fan_value -= StepSizeVar.radiator_fan;
			}
		break;
		case BATTERY_FAN_SETTING:
			if ( (dir == CW) && ( (parameter->battery_fan_value + StepSizeVar.battery_fan) <= parameter->max_fan_duty_cycle )  ) {
				parameter->battery_fan_value += StepSizeVar.battery_fan;
			}
			else if ( (dir == CCW) && ( parameter->battery_fan_value   >= (parameter->min_fan_duty_cycle + StepSizeVar.battery_fan) ) ) {
				parameter->battery_fan_value -= StepSizeVar.battery_fan;
			}
		break;
		case MONO_FAN_SETTING:
			if ( (dir == CW) && ( (parameter->mono_fan_value + StepSizeVar.mono_fan) <= parameter->max_fan_duty_cycle )  ) {
				parameter->mono_fan_value += StepSizeVar.mono_fan;
			}
			else if ( (dir == CCW) && ( parameter->mono_fan_value   >= (parameter->min_fan_duty_cycle + StepSizeVar.mono_fan) ) ) {
				parameter->mono_fan_value -= StepSizeVar.mono_fan;
			}
		break;
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