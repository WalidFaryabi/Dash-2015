#include "sam.h"
#include "../config_sd_msc/conf_board.h"

#include "sd_mmc_protocol.h"
//#include "sysclk.h"
#include "../../sam4e-base/RevolveDrivers/pmc.h"
#include "hsmci.h"

/**
 * \ingroup sam_drivers_hsmci
 * \defgroup sam_drivers_hsmci_internal High Speed MultiMedia Card Interface
 * (HSMCI) implementation
 *
 * @{
 */

// Check configurations
#if (!defined SD_MMC_HSMCI_MEM_CNT) || (SD_MMC_HSMCI_MEM_CNT == 0)
#  warning SD_MMC_HSMCI_MEM_CNT must be defined in board.h file.
#  define SD_MMC_HSMCI_MEM_CNT 1
#endif
#ifndef CONF_BOARD_SD_MMC_HSMCI
#  warning CONF_BOARD_SD_MMC_HSMCI must be defined in conf_board.h file.
#endif
#if (SAM3XA)
#  if (SD_MMC_HSMCI_MEM_CNT > 2)
#    warning Wrong define SD_MMC_HSMCI_MEM_CNT in board.h,\
     this part have 2 slots maximum on HSMCI.
#  endif
#else
#  if (SD_MMC_HSMCI_MEM_CNT > 1)
#    warning Wrong define SD_MMC_HSMCI_MEM_CNT in board.h,\
     this part have 1 slots maximum on HSMCI.
#  endif
#endif
#ifndef SD_MMC_HSMCI_SLOT_0_SIZE
#  warning SD_MMC_HSMCI_SLOT_0_SIZE must be defined in board.h.
#  define SD_MMC_HSMCI_SLOT_0_SIZE 1
#endif
#if (SD_MMC_HSMCI_MEM_CNT > 2)
#  ifndef SD_MMC_HSMCI_SLOT_1_SIZE
#    warning SD_MMC_HSMCI_SLOT_1_SIZE must be defined in board.h.
#    define SD_MMC_HSMCI_SLOT_1_SIZE 1
#  endif
#endif

// Enable debug information for SD/MMC SPI module
#ifdef HSMCI_DEBUG
#  include <stdio.h>
#  define hsmci_debug(...)      printf(__VA_ARGS__)
#else
#  define hsmci_debug(...)
#endif


//! Current position (byte) of the transfer started by hsmci_adtc_start()
static uint32_t hsmci_transfert_pos;
//! Size block requested by last hsmci_adtc_start()
static uint16_t hsmci_block_size;
//! Total number of block requested by last hsmci_adtc_start()
static uint16_t hsmci_nb_block;

static void hsmci_reset(void);
static void hsmci_set_speed(uint32_t speed, uint32_t mck);
static bool hsmci_wait_busy(void);
static bool hsmci_send_cmd_execute(uint32_t cmdr, sdmmc_cmd_def_t cmd,
		uint32_t arg);

/**
 * \brief Reset the HSMCI interface
 */
static void hsmci_reset(void)
{
	uint32_t mr = HSMCI->HSMCI_MR;
	uint32_t dtor = HSMCI->HSMCI_DTOR;
	uint32_t sdcr = HSMCI->HSMCI_SDCR;
	uint32_t cstor = HSMCI->HSMCI_CSTOR;
	uint32_t cfg = HSMCI->HSMCI_CFG;
	HSMCI->HSMCI_CR = HSMCI_CR_SWRST;
	HSMCI->HSMCI_MR = mr;
	HSMCI->HSMCI_DTOR = dtor;
	HSMCI->HSMCI_SDCR = sdcr;
	HSMCI->HSMCI_CSTOR = cstor;
	HSMCI->HSMCI_CFG = cfg;
#ifdef HSMCI_SR_DMADONE
	HSMCI->HSMCI_DMA = 0;
#endif
	// Enable the HSMCI
	HSMCI->HSMCI_CR = HSMCI_CR_PWSEN | HSMCI_CR_MCIEN;
}

/**
 * \brief Set speed of the HSMCI clock.
 *
 * \param speed    HSMCI clock speed in Hz.
 * \param mck      MCK clock speed in Hz.
 */
static void hsmci_set_speed(uint32_t speed, uint32_t mck)
{
	uint32_t clkdiv;
	uint32_t rest;

	// Speed = MCK clock / (2 * (CLKDIV + 1))
	if (speed > 0) {
		clkdiv = mck / (2 * speed);
		rest = mck % (2 * speed);
		if (rest > 0) {
			// Ensure that the card speed not be higher than expected.
			clkdiv++;
		}
		if (clkdiv > 0) {
			clkdiv -= 1;
		}
	} else {
		clkdiv = 0;
	}
	HSMCI->HSMCI_MR &= ~HSMCI_MR_CLKDIV_Msk;
	HSMCI->HSMCI_MR |= HSMCI_MR_CLKDIV(clkdiv);
}

/** \brief Wait the end of busy signal on data line
 *
 * \return true if success, otherwise false
 */
static bool hsmci_wait_busy(void)
{
	uint32_t busy_wait = 1000000;
	uint32_t sr;

	do {
		sr = HSMCI->HSMCI_SR;
		if (busy_wait-- == 0) {
			hsmci_debug("%s: timeout\n\r", __func__);
			hsmci_reset();
			return false;
		}
	} while (!((sr & HSMCI_SR_NOTBUSY) && ((sr & HSMCI_SR_DTIP) == 0)));
	return true;
}


/** \brief Send a command
 *
 * \param cmdr       CMDR resgister bit to use for this command
 * \param cmd        Command definition
 * \param arg        Argument of the command
 *
 * \return true if success, otherwise false
 */
static bool hsmci_send_cmd_execute(uint32_t cmdr, sdmmc_cmd_def_t cmd,
		uint32_t arg)
{
	uint32_t sr;

	cmdr |= HSMCI_CMDR_CMDNB(cmd) | HSMCI_CMDR_SPCMD_STD;
	if (cmd & SDMMC_RESP_PRESENT) {
		cmdr |= HSMCI_CMDR_MAXLAT;
		if (cmd & SDMMC_RESP_136) {
			cmdr |= HSMCI_CMDR_RSPTYP_136_BIT;
		} else if (cmd & SDMMC_RESP_BUSY) {
			cmdr |= HSMCI_CMDR_RSPTYP_R1B;
		} else {
			cmdr |= HSMCI_CMDR_RSPTYP_48_BIT;
		}
	}
	if (cmd & SDMMC_CMD_OPENDRAIN) {
		cmdr |= HSMCI_CMDR_OPDCMD_OPENDRAIN;
	}

	// Write argument
	HSMCI->HSMCI_ARGR = arg;
	// Write and start command
	HSMCI->HSMCI_CMDR = cmdr;

	// Wait end of command
	do {
		sr = HSMCI->HSMCI_SR;
		if (cmd & SDMMC_RESP_CRC) {
			if (sr & (HSMCI_SR_CSTOE | HSMCI_SR_RTOE
					| HSMCI_SR_RENDE | HSMCI_SR_RCRCE
					| HSMCI_SR_RDIRE | HSMCI_SR_RINDE)) {
				hsmci_debug("%s: CMD 0x%08x sr 0x%08x error\n\r",
						__func__, cmd, sr);
				hsmci_reset();
				return false;
			}
		} else {
			if (sr & (HSMCI_SR_CSTOE | HSMCI_SR_RTOE
					| HSMCI_SR_RENDE
					| HSMCI_SR_RDIRE | HSMCI_SR_RINDE)) {
				hsmci_debug("%s: CMD 0x%08x sr 0x%08x error\n\r",
						__func__, cmd, sr);
				hsmci_reset();
				return false;
			}
		}
	} while (!(sr & HSMCI_SR_CMDRDY));

	if (cmd & SDMMC_RESP_BUSY) {
		if (!hsmci_wait_busy()) {
			return false;
		}
	}
	return true;
}


//-------------------------------------------------------------------
//--------------------- PUBLIC FUNCTIONS ----------------------------

void hsmci_init(void)
{
	pmc_enable_periph_clk(ID_HSMCI);

	// Set the Data Timeout Register to 2 Mega Cycles
	HSMCI->HSMCI_DTOR = HSMCI_DTOR_DTOMUL_1048576 | HSMCI_DTOR_DTOCYC(2);
	// Set Completion Signal Timeout to 2 Mega Cycles
	HSMCI->HSMCI_CSTOR = HSMCI_CSTOR_CSTOMUL_1048576 | HSMCI_CSTOR_CSTOCYC(2);
	// Set Configuration Register
	HSMCI->HSMCI_CFG = HSMCI_CFG_FIFOMODE | HSMCI_CFG_FERRCTRL;
	// Set power saving to maximum value
	HSMCI->HSMCI_MR = HSMCI_MR_PWSDIV_Msk;

	// Enable the HSMCI and the Power Saving
	HSMCI->HSMCI_CR = HSMCI_CR_MCIEN | HSMCI_CR_PWSEN;
}

uint8_t hsmci_get_bus_width(uint8_t slot)
{
	switch (slot) {
	case 0:
		return SD_MMC_HSMCI_SLOT_0_SIZE;
#if (SD_MMC_HSMCI_MEM_CNT == 2)
	case 1:
		return SD_MMC_HSMCI_SLOT_1_SIZE;
#endif
	default:
		return 0; // Slot number wrong
	}
}

bool hsmci_is_high_speed_capable(void)
{
	return true;
}

void hsmci_select_device(uint8_t slot, uint32_t clock, uint8_t bus_width, bool high_speed)
{
	uint32_t hsmci_slot = HSMCI_SDCR_SDCSEL_SLOTA;
	uint32_t hsmci_bus_width = HSMCI_SDCR_SDCBUS_1;

	if (high_speed) {
		HSMCI->HSMCI_CFG |= HSMCI_CFG_HSMODE;
	} else {
		HSMCI->HSMCI_CFG &= ~HSMCI_CFG_HSMODE;
	}

	hsmci_set_speed(clock, 120000000); //sysclk_get_cpu_hz())

	switch (slot) {
	case 0:
		hsmci_slot = HSMCI_SDCR_SDCSEL_SLOTA;
		break;
#if (SD_MMC_HSMCI_MEM_CNT == 2)
	case 1:
		hsmci_slot = HSMCI_SDCR_SDCSEL_SLOTB;
		break;
#endif
	}
	switch (bus_width) {
	case 1:
		hsmci_bus_width = HSMCI_SDCR_SDCBUS_1;
		break;

	case 4:
		hsmci_bus_width = HSMCI_SDCR_SDCBUS_4;
		break;

	case 8:
		hsmci_bus_width = HSMCI_SDCR_SDCBUS_8;
		break;
	}
	HSMCI->HSMCI_SDCR = hsmci_slot | hsmci_bus_width;
}

void hsmci_deselect_device(uint8_t slot)
{
	//UNUSED(slot);
	// Nothing to do
}

void hsmci_send_clock(void)
{
	// Configure command
	HSMCI->HSMCI_MR &= ~(HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE);
	// Write argument
	HSMCI->HSMCI_ARGR = 0;
	// Write and start initialization command
	HSMCI->HSMCI_CMDR = HSMCI_CMDR_RSPTYP_NORESP
			| HSMCI_CMDR_SPCMD_INIT
			| HSMCI_CMDR_OPDCMD_OPENDRAIN;
	// Wait end of initialization command
	while (!(HSMCI->HSMCI_SR & HSMCI_SR_CMDRDY));
}

bool hsmci_send_cmd(sdmmc_cmd_def_t cmd, uint32_t arg)
{
	// Configure command
	HSMCI->HSMCI_MR &= ~(HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE);

#ifdef HSMCI_MR_PDCMODE
	// Disable PDC for HSMCI
	HSMCI->HSMCI_MR &= ~HSMCI_MR_PDCMODE;
#endif
	HSMCI->HSMCI_BLKR = 0;
	return hsmci_send_cmd_execute(0, cmd, arg);
}

uint32_t hsmci_get_response(void)
{
	return HSMCI->HSMCI_RSPR[0];
}

void hsmci_get_response_128(uint8_t* response)
{
	uint32_t response_32;

	for (uint8_t i = 0; i < 4; i++) {
		response_32 = HSMCI->HSMCI_RSPR[0];
		*response = (response_32 >> 24) & 0xFF;
		response++;
		*response = (response_32 >> 16) & 0xFF;
		response++;
		*response = (response_32 >>  8) & 0xFF;
		response++;
		*response = (response_32 >>  0) & 0xFF;
		response++;
	}
}

bool hsmci_adtc_start(sdmmc_cmd_def_t cmd, uint32_t arg, uint16_t block_size, uint16_t nb_block, bool access_block)
{
	uint32_t cmdr;
	
#ifdef HSMCI_MR_PDCMODE
	if (access_block) {
		// Enable PDC for HSMCI
		HSMCI->HSMCI_MR |= HSMCI_MR_PDCMODE;
	} else {
		// Disable PDC for HSMCI
		HSMCI->HSMCI_MR &= ~HSMCI_MR_PDCMODE;
	}
#endif

	// Enabling Read/Write Proof allows to stop the HSMCI Clock during
	// read/write  access if the internal FIFO is full.
	// This will guarantee data integrity, not bandwidth.
	HSMCI->HSMCI_MR |= HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF;
	// Force byte transfer if needed
	if (block_size & 0x3) {
		HSMCI->HSMCI_MR |= HSMCI_MR_FBYTE;
	} else {
		HSMCI->HSMCI_MR &= ~HSMCI_MR_FBYTE;
	}

	if (cmd & SDMMC_CMD_WRITE) {
		cmdr = HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_WRITE;
	} else {
		cmdr = HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_READ;
	}

	if (cmd & SDMMC_CMD_SDIO_BYTE) {
			cmdr |= HSMCI_CMDR_TRTYP_BYTE;
			// Value 0 corresponds to a 512-byte transfer
			HSMCI->HSMCI_BLKR = ((block_size % 512) << HSMCI_BLKR_BCNT_Pos);
	} else {
		HSMCI->HSMCI_BLKR = (block_size << HSMCI_BLKR_BLKLEN_Pos) |
				(nb_block << HSMCI_BLKR_BCNT_Pos);
		if (cmd & SDMMC_CMD_SDIO_BLOCK) {
			cmdr |= HSMCI_CMDR_TRTYP_BLOCK;
		} else if (cmd & SDMMC_CMD_STREAM) {
			cmdr |= HSMCI_CMDR_TRTYP_STREAM;
		} else if (cmd & SDMMC_CMD_SINGLE_BLOCK) {
			cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
		} else if (cmd & SDMMC_CMD_MULTI_BLOCK) {
			cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
		} else {
			//assert(false); // Incorrect flags
		}
	}
	hsmci_transfert_pos = 0;
	hsmci_block_size = block_size;
	hsmci_nb_block = nb_block;

	return hsmci_send_cmd_execute(cmdr, cmd, arg);
}

bool hsmci_adtc_stop(sdmmc_cmd_def_t cmd, uint32_t arg)
{
	return hsmci_send_cmd_execute(HSMCI_CMDR_TRCMD_STOP_DATA, cmd, arg);
}

bool hsmci_read_word(uint32_t* value)
{
	uint32_t sr;

	//assert(((uint32_t)hsmci_block_size * hsmci_nb_block) > hsmci_transfert_pos);

	// Wait data available
	do {
		sr = HSMCI->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			hsmci_debug("%s: DMA sr 0x%08x error\n\r",
					__func__, sr);
			hsmci_reset();
			return false;
		}
	} while (!(sr & HSMCI_SR_RXRDY));

	// Read data
	*value = HSMCI->HSMCI_RDR;
	hsmci_transfert_pos += 4;
	if (((uint32_t)hsmci_block_size * hsmci_nb_block) > hsmci_transfert_pos) {
		return true;
	}

	// Wait end of transfer
	// Note: no need of timeout, because it is include in HSMCI
	do {
		sr = HSMCI->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			hsmci_debug("%s: DMA sr 0x%08x error\n\r",
					__func__, sr);
			hsmci_reset();
			return false;
		}
	} while (!(sr & HSMCI_SR_XFRDONE));
	return true;
}

bool hsmci_write_word(uint32_t value)
{
	uint32_t sr;

	//assert(((uint32_t)hsmci_block_size * hsmci_nb_block) > hsmci_transfert_pos);

	// Wait data available
	do {
		sr = HSMCI->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			hsmci_debug("%s: DMA sr 0x%08x error\n\r",
					__func__, sr);
			hsmci_reset();
			return false;
		}
	} while (!(sr & HSMCI_SR_TXRDY));

	// Write data
	HSMCI->HSMCI_TDR = value;
	hsmci_transfert_pos += 4;
	if (((uint32_t)hsmci_block_size * hsmci_nb_block) > hsmci_transfert_pos) {
		return true;
	}

	// Wait end of transfer
	// Note: no need of timeout, because it is include in HSMCI, see DTOE bit.
	do {
		sr = HSMCI->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			hsmci_debug("%s: DMA sr 0x%08x error\n\r",
					__func__, sr);
			hsmci_reset();
			return false;
		}
	} while (!(sr & HSMCI_SR_NOTBUSY));
	//assert(HSMCI->HSMCI_SR & HSMCI_SR_FIFOEMPTY);
	return true;
}


#ifdef HSMCI_MR_PDCMODE
bool hsmci_start_read_blocks(void *dest, uint16_t nb_block)
{
	uint32_t nb_data;

	nb_data = nb_block * hsmci_block_size;
	//assert(nb_data <= (((uint32_t)hsmci_block_size * hsmci_nb_block) - hsmci_transfert_pos));
	//assert(nb_data <= (PERIPH_RCR_RXCTR_Msk >> PERIPH_RCR_RXCTR_Pos));

	// Handle unaligned memory address
	if (((uint32_t)dest & 0x3) || (hsmci_block_size & 0x3)) {
		HSMCI->HSMCI_MR |= HSMCI_MR_FBYTE;
	} else {
		HSMCI->HSMCI_MR &= ~HSMCI_MR_FBYTE;
	}

	// Configure PDC transfert
	HSMCI->HSMCI_RPR = (uint32_t)dest;
	HSMCI->HSMCI_RCR = (HSMCI->HSMCI_MR & HSMCI_MR_FBYTE) ?
			nb_data : nb_data / 4;
	HSMCI->HSMCI_RNCR = 0;
	// Start transfert
	HSMCI->HSMCI_PTCR = HSMCI_PTCR_RXTEN;
	hsmci_transfert_pos += nb_data;
	return true;
}

bool hsmci_wait_end_of_read_blocks(void)
{
	uint32_t sr;
	// Wait end of transfert
	// Note: no need of timeout, because it is include in HSMCI, see DTOE bit.
	do {
		sr = HSMCI->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			hsmci_debug("%s: PDC sr 0x%08x error\n\r",
					__func__, sr);
			HSMCI->HSMCI_PTCR = HSMCI_PTCR_RXTDIS | HSMCI_PTCR_TXTDIS;
			hsmci_reset();
			return false;
		}

	} while (!(sr & HSMCI_SR_RXBUFF));

	if (hsmci_transfert_pos < ((uint32_t)hsmci_block_size * hsmci_nb_block)) {
		return true;
	}
	// It is the last transfer, then wait command completed
	// Note: no need of timeout, because it is include in HSMCI, see DTOE bit.
	do {
		sr = HSMCI->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			hsmci_debug("%s: PDC sr 0x%08x last transfer error\n\r",
					__func__, sr);
			hsmci_reset();
			return false;
		}
	} while (!(sr & HSMCI_SR_XFRDONE));
	return true;
}

bool hsmci_start_write_blocks(const void *src, uint16_t nb_block)
{
	uint32_t nb_data;

	nb_data = nb_block * hsmci_block_size;
	//assert(nb_data <= (((uint32_t)hsmci_block_size * hsmci_nb_block) - hsmci_transfert_pos));
	//assert(nb_data <= (PERIPH_TCR_TXCTR_Msk >> PERIPH_TCR_TXCTR_Pos));

	// Handle unaligned memory address
	if (((uint32_t)src & 0x3) || (hsmci_block_size & 0x3)) {
		HSMCI->HSMCI_MR |= HSMCI_MR_FBYTE;
	} else {
		HSMCI->HSMCI_MR &= ~HSMCI_MR_FBYTE;
	}

	// Configure PDC transfert
	HSMCI->HSMCI_TPR = (uint32_t)src;
	HSMCI->HSMCI_TCR = (HSMCI->HSMCI_MR & HSMCI_MR_FBYTE) ?
			nb_data : nb_data / 4;
	HSMCI->HSMCI_TNCR = 0;
	// Start transfert
	HSMCI->HSMCI_PTCR = HSMCI_PTCR_TXTEN;
	hsmci_transfert_pos += nb_data;
	return true;
}

bool hsmci_wait_end_of_write_blocks(void)
{
	uint32_t sr;

	// Wait end of transfert
	// Note: no need of timeout, because it is include in HSMCI, see DTOE bit.
	do {
		sr = HSMCI->HSMCI_SR;
		if (sr &
				(HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			hsmci_debug("%s: PDC sr 0x%08x error\n\r",
					__func__, sr);
			hsmci_reset();
			HSMCI->HSMCI_PTCR = HSMCI_PTCR_RXTDIS | HSMCI_PTCR_TXTDIS;
			return false;
		}
	} while (!(sr & HSMCI_SR_TXBUFE));


	if (hsmci_transfert_pos < ((uint32_t)hsmci_block_size * hsmci_nb_block)) {
		return true;
	}
	// It is the last transfer, then wait command completed
	// Note: no need of timeout, because it is include in HSMCI, see DTOE bit.
	do {
		sr = HSMCI->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			hsmci_debug("%s: PDC sr 0x%08x last transfer error\n\r",
					__func__, sr);
			hsmci_reset();
			return false;
		}
	} while (!(sr & HSMCI_SR_NOTBUSY));
	uint8_t h;
	//assert(HSMCI->HSMCI_SR & HSMCI_SR_FIFOEMPTY);
	return true;
}

#endif // HSMCI_MR_PDCMODE