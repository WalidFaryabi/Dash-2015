#include "can.h"
#include "pio.h"
#include "pmc.h"



/** The max value for CAN baudrate prescale. */
#define CAN_BAUDRATE_MAX_DIV       128

/** Define the scope for TQ (time quantum). */
#define CAN_MIN_TQ_NUM             8
#define CAN_MAX_TQ_NUM             25


/** Define the mailbox mode. */
#define CAN_MB_DISABLE_MODE           0
#define CAN_MB_RX_MODE                1 /**< The first message received is stored in mailbox data registers. 
											  Data remain available until the next transfer request.*/
#define CAN_MB_RX_OVER_WR_MODE        2 /**< The last message received is stored in mailbox data register. The next message always
											overwrites the previous one. The application has to check whether a new message has not
											overwritten the current one while reading the data registers.*/
#define CAN_MB_TX_MODE                3 /**<The message stored in the mailbox data registers will try to win the bus arbitration immediately
											or later according to or not the Time Management Unit configuration.
											The application is notified that the message has been sent or aborted.*/
#define CAN_MB_CONSUMER_MODE          4 /**A remote frame is sent by the mailbox. The answer received is stored in mailbox data register.
											This extends Receive mailbox features. Data remain available until the next transfer request.*/
#define CAN_MB_PRODUCER_MODE          5 /**<The message prepared in the mailbox data registers will be sent after receiving the next remote
											frame. This extends transmit mailbox features.*/

#define TX_BOX_ID 0
#define NUMBER_OF_MAILBOXES 8

#define CAN_WPMR_WP

/*
*Init help functions and data structures
*/



//ASF-stuff
typedef struct {
	uint8_t tq;      /**< CAN_BIT_SYNC + uc_prog + uc_phase1 + uc_phase2
	                       = uc_tq, 8 <= uc_tq <= 25. */
	uint8_t prog;    /**< Propagation segment, (3-bits + 1), 1~8; */
	uint8_t phase1;  /**< Phase segment 1, (3-bits + 1), 1~8; */
	uint8_t phase2;  /**< Phase segment 2, (3-bits + 1), 1~8, CAN_BIT_IPT
	                       <= uc_phase2; */
	uint8_t sjw;     /**< Resynchronization jump width, (2-bits + 1),
	                       min(uc_phase1, 4); */
	uint8_t sp;      /**< Sample point value, 0~100 in percent. */
} can_bit_timing_t;

/** Values of bit time register for different baudrates, Sample point =
 * ((1 + uc_prog + uc_phase1) / uc_tq) * 100%. */
const can_bit_timing_t can_bit_time[] = {
	{ 8, (2 + 1), (1 + 1), (1 + 1), (2 + 1), 75},
	{ 9, (1 + 1), (2 + 1), (2 + 1), (1 + 1), 67},
	{10, (2 + 1), (2 + 1), (2 + 1), (2 + 1), 70},
	{11, (3 + 1), (2 + 1), (2 + 1), (3 + 1), 72},
	{12, (2 + 1), (3 + 1), (3 + 1), (3 + 1), 67},
	{13, (3 + 1), (3 + 1), (3 + 1), (3 + 1), 77},
	{14, (3 + 1), (3 + 1), (4 + 1), (3 + 1), 64},
	{15, (3 + 1), (4 + 1), (4 + 1), (3 + 1), 67},
	{16, (4 + 1), (4 + 1), (4 + 1), (3 + 1), 69},
	{17, (5 + 1), (4 + 1), (4 + 1), (3 + 1), 71},
	{18, (4 + 1), (5 + 1), (5 + 1), (3 + 1), 67},
	{19, (5 + 1), (5 + 1), (5 + 1), (3 + 1), 68},
	{20, (6 + 1), (5 + 1), (5 + 1), (3 + 1), 70},
	{21, (7 + 1), (5 + 1), (5 + 1), (3 + 1), 71},
	{22, (6 + 1), (6 + 1), (6 + 1), (3 + 1), 68},
	{23, (7 + 1), (7 + 1), (6 + 1), (3 + 1), 70},
	{24, (6 + 1), (7 + 1), (7 + 1), (3 + 1), 67},
	{25, (7 + 1), (7 + 1), (7 + 1), (3 + 1), 68}
};

void can_writeProtectionEnable(Can *can){
	can->CAN_WPMR = CAN_WPMR_WPKEY_PASSWD | CAN_WPMR_WPEN;
}

void can_writeProtectionDisable(Can *can){
	can->CAN_WPMR = CAN_WPMR_WPKEY_PASSWD;
}

void can_enable(Can *can)
{
	can_writeProtectionDisable(can);	
	can->CAN_MR |= CAN_MR_CANEN;
	can_writeProtectionEnable(can);
}

void can_disable(Can *can)
{
	can_writeProtectionEnable(can);
	can->CAN_MR &= ~CAN_MR_CANEN;
	can_writeProtectionDisable(can);
}



//ASF-stuff
static uint32_t can_setBaudrate(Can *can, uint32_t mck, uint32_t baudrate)
{
	can_writeProtectionDisable(can);
	uint8_t tq;
	uint8_t prescale;
	uint32_t mod;
	uint32_t cur_mod;
	can_bit_timing_t *bitTime;

	/* Check whether the baudrate prescale will be greater than the max
	 * divide value. */
	if (((mck + (baudrate * CAN_MAX_TQ_NUM * 1000 - 1)) /
			(baudrate * CAN_MAX_TQ_NUM * 1000)) >
			CAN_BAUDRATE_MAX_DIV) {
		return 0;
	}

	/* Check whether the input MCK is too small. */
	if ((mck / 2)  < baudrate * CAN_MIN_TQ_NUM * 1000) {
		return 0;
	}

	/* Initialize it as the minimum Time Quantum. */
	tq = CAN_MIN_TQ_NUM;

	/* Initialize the remainder as the max value. When the remainder is 0,
	 *get the right TQ number. */
	mod = 0xffffffff;
	/* Find out the approximate Time Quantum according to the baudrate. */
	for (uint8_t i = CAN_MIN_TQ_NUM; i <= CAN_MAX_TQ_NUM; i++) {
		if ((mck / (baudrate * i * 1000)) <=
				CAN_BAUDRATE_MAX_DIV) {
			cur_mod = mck % (baudrate * i * 1000);
			if (cur_mod < mod) {
				mod = cur_mod;
				tq = i;
				if (!mod) {
					break;
				}
			}
		}
	}

	/* Calculate the baudrate prescale value. */
	prescale = mck / (baudrate * tq * 1000);
	if (prescale < 2) {
		return 0;
	}

	/* Get the right CAN BIT Timing group. */
	bitTime = (can_bit_timing_t *)&can_bit_time[tq - CAN_MIN_TQ_NUM];

	/* Before modifying the CANBR register, disable the CAN controller. */
	can_disable(can);

	/* Write into the CAN baudrate register. */
	can->CAN_BR = CAN_BR_PHASE2(bitTime->phase2 - 1) |
			CAN_BR_PHASE1(bitTime->phase1 - 1) |
			CAN_BR_PROPAG(bitTime->prog - 1) |
			CAN_BR_SJW(bitTime->sjw - 1) |
			CAN_BR_BRP(prescale - 1);
	can_writeProtectionEnable(can);
	return 1;
}


void can_setPeripheralMux(Can *can){
	if(can == CAN0){
		pio_setMux(PIOB, 3, A);
		pio_setMux(PIOB, 2, A);
	}
	else if(can == CAN1){
		pio_setMux(PIOC, 12, C);
		pio_setMux(PIOC, 15, C);
	}
}

void can_enablePMC(Can *can){
	can_writeProtectionDisable(can);
	if(can == CAN0){
		pmc_enable_periph_clk(37);
		//PMC->PMC_PCER1 |= 1<<5;			//ENABLE CLOCK, PID = 37
	}
	else if(can == CAN1){
		pmc_enable_periph_clk(38);
		//PMC->PMC_PCER1 |= 1<<6;			//ENABLE CLOCK, PID = 38
	}
	can_writeProtectionEnable(can);
}

void can_setupTXMailbox(Can *can, uint8_t mailbox_id, uint8_t priority){
	can_writeProtectionDisable(can);
	/* Set the priority in Transmit mode. */
	can->CAN_MB[mailbox_id].CAN_MMR = 
		(can->CAN_MB[mailbox_id].CAN_MMR & ~CAN_MMR_PRIOR_Msk) |
		(priority << CAN_MMR_PRIOR_Pos);
	
	/* Set box into TX mode*/
	can->CAN_MB[mailbox_id].CAN_MMR = 
		(can->CAN_MB[mailbox_id].CAN_MMR & ~CAN_MMR_MOT_Msk) |
		(CAN_MB_TX_MODE << CAN_MMR_MOT_Pos);
	can_writeProtectionEnable(can);
}

void can_setupRXMailbox(Can *can, uint8_t mailbox_id, uint32_t acceptence_mask, uint32_t id_mask){
	can_writeProtectionDisable(can);
	/* Set acceptance mask  */
	can->CAN_MB[mailbox_id].CAN_MAM = acceptence_mask | CAN_MAM_MIDE;
	/* Set message ID mask */
	can->CAN_MB[mailbox_id].CAN_MAM = id_mask | CAN_MAM_MIDE;
	/* Set box into RX mode */
	can->CAN_MB[mailbox_id].CAN_MMR =
		(can->CAN_MB[mailbox_id].CAN_MMR & ~CAN_MMR_MOT_Msk) |
		(CAN_MB_RX_MODE << CAN_MMR_MOT_Pos);
	can_writeProtectionEnable(can);
}

void can_disableMailbox(Can *can, uint8_t mailbox_id){
	can_writeProtectionDisable(can);
	can->CAN_MB[mailbox_id].CAN_MMR = 0;
	can->CAN_MB[mailbox_id].CAN_MAM = 0;
	can->CAN_MB[mailbox_id].CAN_MID = 0;
	can->CAN_MB[mailbox_id].CAN_MDL = 0;
	can->CAN_MB[mailbox_id].CAN_MDH = 0;
	can->CAN_MB[mailbox_id].CAN_MCR = 0;
	can_writeProtectionEnable(can);
}

void can_init(Can *can, uint32_t peripheral_clock_hz, uint32_t baudrate_kbps){
	can_writeProtectionDisable(can);
	can_disable(can);
	can_setPeripheralMux(can);
	can_enablePMC(can);
	if(!can_setBaudrate(can, peripheral_clock_hz, baudrate_kbps)){
		return -1; //Illegal combination of peripheral_clock_hz and baudrate_kbps
	}
	
	//Reset 8 all mailboxes
	for (int i = 0; i<8; i++)
	{
		can_disableMailbox(can, i);
	}
	can_enable(can);
	/* Wait until the CAN is synchronized with the bus activity. */
	while(!(can->CAN_SR & CAN_SR_WAKEUP));
	
	//init TX mailbox:
	can_setupTXMailbox(can, TX_BOX_ID, 0);
	can_writeProtectionEnable(can);
}

void can_setupFilters(Can *can, uint32_t acceptence_masks[7], uint32_t id_masks[7]){
	can_writeProtectionDisable(can);
	for (int mailboxID = 0; mailboxID < NUMBER_OF_MAILBOXES; mailboxID++)
	{
		if ( mailboxID != TX_BOX_ID)
		{
			can_setupRXMailbox(can, mailboxID, acceptence_masks[mailboxID], id_masks[mailboxID]);
		}
	}
	can_enableRXInterrupt(can);
	can_writeProtectionEnable(can);
}

enum CanTXstatus can_sendMessage(Can *can, struct CanMessage message){
	//check if mailbox is ready
	if ( !(can->CAN_MB[TX_BOX_ID].CAN_MSR & CAN_MSR_MRDY) )
	{
		return TRANSFER_BUSY;
	}
	can_writeProtectionDisable(can);
	//write message id to mailboxz
	can->CAN_MB[TX_BOX_ID].CAN_MID = CAN_MID_MIDvA(message.messageID);
	
	//write data
	can->CAN_MB[TX_BOX_ID].CAN_MDL = message.data.u32[0];
	if (message.dataLength > 4)
	{
		can->CAN_MB[TX_BOX_ID].CAN_MDH = message.data.u32[1];
	}
	
	//write data length and initate transfer
	can->CAN_MB[TX_BOX_ID].CAN_MCR = CAN_MCR_MTCR | CAN_MCR_MDLC(message.dataLength);
	can_writeProtectionEnable(can);
	return TRANSFER_OK;	
}

void can_read_message(Can *can, uint8_t mailboxID, struct CanMessage *message){
	//Get data length
	message->dataLength = (can->CAN_MB[mailboxID].CAN_MSR & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos;
	
	//Get the message id
	message->messageID = (can->CAN_MB[mailboxID].CAN_MID & CAN_MID_MIDvA_Msk) >> CAN_MID_MIDvA_Pos;
	
	//Get the data
	message->data.u32[0] =  can->CAN_MB[mailboxID].CAN_MDL;
	if (message->dataLength > 4)
	{
		message->data.u32[1] =  can->CAN_MB[mailboxID].CAN_MDH;
	}
	
	//Allow new incomming message
	can->CAN_MB[mailboxID].CAN_MCR = CAN_MCR_MTCR;
}

void can_enableRXInterrupt(Can *can){
	if (can == CAN0){
		NVIC_DisableIRQ(CAN0_IRQn);
		NVIC_ClearPendingIRQ(CAN0_IRQn);
		NVIC_SetPriority(CAN0_IRQn,5); // must be >= 5
		NVIC_EnableIRQ(CAN0_IRQn);
		}
		else if (can == CAN1){
		NVIC_DisableIRQ(CAN1_IRQn);
		NVIC_ClearPendingIRQ(CAN1_IRQn);
		NVIC_SetPriority(CAN1_IRQn,5); // must be >= 5
		NVIC_EnableIRQ(CAN1_IRQn);
	}
	can_writeProtectionDisable(can);
	//enable all interrupts on mailboxes that are in RX mode
	can->CAN_IER = 0xff & ~(1<<TX_BOX_ID);
	can_writeProtectionEnable(can);
}

void can_disableRXInterrupt(Can *can){
	if (can == CAN0){
		NVIC_DisableIRQ(CAN0_IRQn);
		NVIC_ClearPendingIRQ(CAN0_IRQn);
		}else if(can == CAN1){
		NVIC_DisableIRQ(CAN1_IRQn);
		NVIC_ClearPendingIRQ(CAN1_IRQn);
	}
}

enum CANReceiveStatus can_popMessage(Can *can, struct CanMessage *message){
	
	for (int i = 0; i < NUMBER_OF_MAILBOXES; i++)
	{
		if (i != TX_BOX_ID && (can->CAN_MB[i].CAN_MSR & CAN_MSR_MRDY))
		{
			can_read_message(can, i, message);
			return GOT_NEW_MESSAGE;
		}
	}
	return NO_NEW_MESSAGE;
}