#ifndef CANMESSAGES_H_
#define CANMESSAGES_H_


#include "sam4e-base/RevolveDrivers/can.h"
#include "DriversNotInBase/revolve_can_definitions.h"
#include "canID_definitions.h"


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

#define CAN_SET_TORQUE		0x01
#define CAN_SET_KERS_AMOUNT 0x02
#define CAN_SET_KERS_TOGGLE	0x03
struct CanMessage EcuParameters = {
	// Byte 1 describes what parameter is being set in the ECU , 0x01 for Torque, 0x02 for Kers amount, 0x03 for KERS ON/OFF, 0x04 ..
	// Byte 2 is for Torque percentage
	// Byte 3 is for Kers Amount
	// Byte 4 is for Kers ON/OFF
	// Byte 5 ...
	.messageID		= ID_ECU_PARAMETERS,
	.dataLength		= 2,
	.data.u32[0]	= 0
};

struct CanMessage EcuPTerm = {
	.messageID = ID_ECU_P_TERM,
	.dataLength = 4
};

struct CanMessage EcuITerm = {
	.messageID = ID_ECU_I_TERM,
	.dataLength = 4
};



	



#endif /* CANMESSAGES_H_ */