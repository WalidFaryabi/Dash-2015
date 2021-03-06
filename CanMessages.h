#ifndef CANMESSAGES_H_
#define CANMESSAGES_H_


#include "sam4e-base/RevolveDrivers/can.h"
#include "DriversNotInBase/revolve_can_definitions.h"
#include "canID_definitions.h"

// Identifier for Modules on FAN card
#define RADIATOR_FAN_ID		1
#define BATTERY_FAN_ID		2
#define MONO_FAN_ID			3
#define PUMP_ID				4
#define TOGGLE_ALL_FANS_ID	7

struct CanMessage IAmAlive = {
	.dataLength = 1,
	.data.u8[0] = ALIVE_DASH,
	.messageID = ID_ALIVE
};

struct CanMessage TorquePedalCalibrationMax = {
	.data.u8[0] = 0xF0,
	.dataLength = 1,
	.messageID = ID_TORQUE_PEDAL_CALIBRATION
};
struct CanMessage TorquePedalCalibrationMin = {
	.data.u8[0] = 0x0F,
	.dataLength = 1,
	.messageID = ID_TORQUE_PEDAL_CALIBRATION
};
struct CanMessage SteeringCalibrationLeft = {
	.data.u8[0] = 0xF0,
	.dataLength = 1,
	.messageID = ID_STEERING_CALIBRATION
};
struct CanMessage SteeringCalibrationRight = {
	.data.u8[0] = 0x0F,
	.dataLength = 1,
	.messageID = ID_STEERING_CALIBRATION
};

struct CanMessage Acknowledge = {
	.messageID = ID_DASH_ACKNOWLEDGE,
	.dataLength = 0
};

struct CanMessage FinishedRTDS = {
	.messageID = ID_FINISHED_RTDS,
	.dataLength = 0
};
struct CanMessage RequestDriveDisable = {
	.messageID = ID_ECU_DRIVE,
	.dataLength = 1,
	.data.u8[0] = 0
};
struct CanMessage RequestDriveEnable = {
	.messageID = ID_ECU_DRIVE,
	.dataLength = 1,
	.data.u8[0] = 1
};

struct CanMessage RequestLCInit = {
	.messageID = ID_ECU_LC,
	.dataLength = 1,
	.data.u8[0] = 1
};

struct CanMessage RequestLCArmed = {
	.messageID = ID_ECU_LC,
	.dataLength = 1,
	.data.u8[0] = 2
};

struct CanMessage RequestLCDisable = {
	.messageID = ID_ECU_LC,
	.dataLength = 1,
	.data.u8[0] = 0x03
};

struct CanMessage TurnOffPump = {
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = PUMP_ID,
	.data.u8[1] = 0,
	.data.u8[2] = 0
	};
struct CanMessage TurnOnPump = {
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = PUMP_ID,
	.data.u8[1] = 0,
	.data.u8[2] = 1
};	
struct CanMessage TurnOffAllFans = {
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = TOGGLE_ALL_FANS_ID,
	.data.u8[1] = 0,
	.data.u8[2] = 0
	};
struct CanMessage TurnOnAllFans = {
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = TOGGLE_ALL_FANS_ID,
	.data.u8[1] = 0,
	.data.u8[2] = 1
};
struct CanMessage AdjustDutyCycleRadiatorFan = {
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = RADIATOR_FAN_ID,
	.data.u8[1] = 0
};
struct CanMessage AdjustDutyCycleBatteryFan = {
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = BATTERY_FAN_ID,
	.data.u8[1] = 0
};
struct CanMessage AdjustDutyCycleMonoFan = {
	.messageID = ID_FAN_CONTROL,
	.dataLength = 3,
	.data.u8[0] = MONO_FAN_ID,
	.data.u8[1] = 0
};
// 0-100
// radiator 1
// batter 2
// mono 3 
// sette byte [2] til 0-100 


struct CanMessage EcuParametersFromFile = {
	// Byte 1 describes what parameter is being set in the ECU , 0x01 for Torque, 0x02 for Kers amount, 0x03 for KERS ON/OFF, 0x04 ..
	// Byte 2 is for Torque percentage
	// Byte 3 is for Kers Amount
	// Byte 4 is for Kers ON/OFF
	// Byte 5 ...
	.messageID		= ID_ECU_PARAMETERS,
	.dataLength		= 8
};

struct CanMessage EcuTractionControl = {
	.messageID = ID_ECU_TRACTION_CONTROL,
	.dataLength = 1
};



	



#endif /* CANMESSAGES_H_ */