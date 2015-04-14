#ifndef _CAN_H_
#define _CAN_H_

#include "sam.h"

#define CAN_BPS_1000K                 1000
#define CAN_BPS_800K                  800
#define CAN_BPS_500K                  500
#define CAN_BPS_250K                  250
#define CAN_BPS_125K                  125
#define CAN_BPS_50K                   50
#define CAN_BPS_25K                   25
#define CAN_BPS_10K                   10
#define CAN_BPS_5K                    5

enum CanTXstatus {TRANSFER_OK = 0, TRANSFER_BUSY = 1};
enum CANReceiveStatus {GOT_NEW_MESSAGE = 0, NO_NEW_MESSAGE = 1};

typedef union can_data_t{
	uint64_t u64;
	int64_t  i64;
	uint32_t u32[2];
	int32_t  i32[2];
	uint16_t u16[4];
	int16_t  i16[4];
	uint8_t  u8[8];
	int8_t   i8[8];
	float	 f32[2];
	float	f[2];
}CanData ;

struct CanMessage{
	CanData data;
	uint8_t dataLength;
	uint32_t messageID;
};

void can_init(Can *can, uint32_t peripheral_clock_hz, uint32_t baudrate_kbps);
void can_setupFilters(Can *can, uint32_t acceptence_masks[7], uint32_t id_masks[7]);
enum CanTXstatus can_sendMessage(Can *can, struct CanMessage message);	//NOT THREADSAFE
enum CANReceiveStatus can_popMessage(Can *can, struct CanMessage *message); //NOT THREADSAFE
//must enable interrupt in nvic for this to work. Handle the interrupt by defining a CAN_HANDLER() function
void can_enableRXInterrupt(Can *can);


#endif /* _CAN_H_ */