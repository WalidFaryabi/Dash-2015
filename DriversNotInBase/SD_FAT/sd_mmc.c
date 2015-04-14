

#include <string.h>
#include "../config_sd_msc/conf_board.h"
//#include "board.h"
#include "../config_sd_msc/conf_sd_mmc.h"
#include "sd_mmc_protocol.h"
#include "sd_mmc.h"
#include "../delay.h"
#include "../../sam4e-base/RevolveDrivers/pio.h"
//#include "ioport.h"

//#define FREERTOS_USED 
#ifdef FREERTOS_USED
#include "../../FreeRTOS/Source/include/FreeRTOS.h"
#include "../../FreeRTOS/Source/include/task.h"
#include "../../FreeRTOS/Source/portable/GCC/ARM_CM4F/portmacro.h"
#include "../../FreeRTOS/Source/include/projdefs.h"

//#include "FreeRTOS.h"
//#include "task.h"
//#include "portmacro.h"
//#include "projdefs.h"
#endif

/**
 * \ingroup sd_mmc_stack
 * \defgroup sd_mmc_stack_internal Implementation of SD/MMC/SDIO Stack
 * @{
 */

// Enable debug information for SD/MMC module
#ifdef SD_MMC_DEBUG
#  include <stdio.h>
#  define sd_mmc_debug(...)      printf(__VA_ARGS__)
#else
#  define sd_mmc_debug(...)
#endif


#define SD_MMC_HSMCI_MEM_CNT 1
#define SD_MMC_HSMCI_SLOT_0_SIZE  4
#include "hsmci.h"
//#define hsmci  hsmci
#define SD_MMC_MEM_CNT        SD_MMC_HSMCI_MEM_CNT
#define sd_mmc_is_spi()       false

/*
#define hsmci_init                     ATPASTE2(hsmci, _init)
#define hsmci_select_device            ATPASTE2(hsmci, _select_device)
#define hsmci_deselect_device          ATPASTE2(hsmci, _deselect_device)
#define hsmci_get_bus_width            ATPASTE2(hsmci, _get_bus_width)
#define hsmci_is_high_speed_capable    ATPASTE2(hsmci, _is_high_speed_capable)
#define hsmci_send_clock               ATPASTE2(hsmci, _send_clock)
#define hsmci_send_cmd                 ATPASTE2(hsmci, _send_cmd)
#define hsmci_get_response             ATPASTE2(hsmci, _get_response)
#define hsmci_get_response_128         ATPASTE2(hsmci, _get_response_128)
#define hsmci_adtc_start               ATPASTE2(hsmci, _adtc_start)
#define hsmci_adtc_stop                ATPASTE2(hsmci, _send_cmd)
#define hsmci_read_word                ATPASTE2(hsmci, _read_word)
#define hsmci_write_word               ATPASTE2(hsmci, _write_word)
#define hsmci_start_read_blocks        ATPASTE2(hsmci, _start_read_blocks)
#define hsmci_wait_end_of_read_blocks  ATPASTE2(hsmci, _wait_end_of_read_blocks)
#define hsmci_start_write_blocks       ATPASTE2(hsmci, _start_write_blocks)
#define hsmci_wait_end_of_write_blocks ATPASTE2(hsmci, _wait_end_of_write_blocks)
*/
#ifdef SDIO_SUPPORT_ENABLE
#  define IS_SDIO()  (sd_mmc_card->type & CARD_TYPE_SDIO)
#else
#  define IS_SDIO()  false
#endif




#define sd_mmc_is_mci()  (!sd_mmc_is_spi())

//! This SD MMC stack supports only the high voltage
#define SD_MMC_VOLTAGE_SUPPORT \
		(OCR_VDD_27_28 | OCR_VDD_28_29 | \
		OCR_VDD_29_30 | OCR_VDD_30_31 | \
		OCR_VDD_31_32 | OCR_VDD_32_33)

//! SD/MMC card states
enum card_state {
	SD_MMC_CARD_STATE_READY    = 0, //!< Ready to use
	SD_MMC_CARD_STATE_DEBOUNCE = 1, //!< Debounce on going
	SD_MMC_CARD_STATE_INIT     = 2, //!< Initialization on going
	SD_MMC_CARD_STATE_UNUSABLE = 3, //!< Unusable card
	SD_MMC_CARD_STATE_NO_CARD  = 4, //!< No SD/MMC card inserted
};

//! SD/MMC card information structure
struct sd_mmc_card {
	uint32_t clock;            //!< Card access clock
	uint32_t capacity;         //!< Card capacity in KBytes
#if (defined SD_MMC_0_CD_GPIO)
	uint32_t cd_gpio;          //!< Card detect GPIO
#  if (defined SD_MMC_0_WP_GPIO)
	uint32_t wp_gpio;          //!< Card write protection GPIO
#  endif
#endif
	uint16_t rca;              //!< Relative card address
	enum card_state state;     //!< Card state
	card_type_t type;          //!< Card type
	card_version_t version;    //!< Card version
	uint8_t  bus_width;        //!< Number of DATA lin on bus (MCI only)
	uint8_t csd[CSD_REG_BSIZE];//!< CSD register
	uint8_t high_speed;        //!< High speed card (1)
};

//! SD/MMC card list
//! Note: Initialize card detect pin fields if present
static struct sd_mmc_card sd_mmc_cards[SD_MMC_MEM_CNT]
#if (defined SD_MMC_0_CD_GPIO) && (defined SD_MMC_0_WP_GPIO)
 = {
# define SD_MMC_CD_WP(slot, unused) \
	{.cd_gpio = SD_MMC_##slot##_CD_GPIO, \
	.wp_gpio = SD_MMC_##slot##_WP_GPIO},
	MREPEAT(SD_MMC_MEM_CNT, SD_MMC_CD_WP, ~)
# undef SD_MMC_CD_WP
}
#elif (defined SD_MMC_0_CD_GPIO)
 = {
# define SD_MMC_CD(slot, unused) \
	{.cd_gpio = SD_MMC_##slot##_CD_GPIO},
	MREPEAT(SD_MMC_MEM_CNT, SD_MMC_CD, ~)
# undef SD_MMC_CD
}
#endif
;

//! Index of current slot configurated
static uint8_t sd_mmc_slot_sel;
//! Pointer on current slot configurated
static struct sd_mmc_card *sd_mmc_card;
//! Number of block to read or write on the current transfer
static uint16_t sd_mmc_nb_block_to_tranfer = 0;
//! Number of block remaining to read or write on the current transfer
static uint16_t sd_mmc_nb_block_remaining = 0;

//! SD/MMC transfer rate unit codes (10K) list
const uint32_t sd_mmc_trans_units[7] = {
	10, 100, 1000, 10000, 0, 0, 0
};
//! SD transfer multiplier factor codes (1/10) list
const uint32_t sd_trans_multipliers[16] = {
	0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80
};
//! MMC transfer multiplier factor codes (1/10) list
const uint32_t mmc_trans_multipliers[16] = {
	0, 10, 12, 13, 15, 20, 26, 30, 35, 40, 45, 52, 55, 60, 70, 80
};

//! \name MMC, SD and SDIO commands process
//! @{


static bool sd_mci_op_cond(uint8_t v2);
static bool sd_cm6_set_high_speed(void);
static bool sd_cmd8(uint8_t * v2);
static bool sd_mmc_cmd9_mci(void);
static void sd_decode_csd(void);
static bool sd_mmc_cmd13(void);
static bool sd_acmd6(void);
static bool sd_acmd51(void);
//! @}

//! \name Internal function to process the initialization and install
//! @{
static sd_mmc_err_t sd_mmc_select_slot(uint8_t slot);
static void sd_mmc_configure_slot(void);
static void sd_mmc_deselect_slot(void);
static bool sd_mmc_mci_card_init(void);

//! @}


//! \name Internal functions to manage a large timeout after a card insertion
//! @{
#define SD_MMC_DEBOUNCE_TIMEOUT   1000 // Unit ms

#if SAM
static bool sd_mmc_sam_systick_used;
#  ifdef FREERTOS_USED
		static xTimeOutType xTimeOut;
#endif

static inline void SD_MMC_START_TIMEOUT(void)
{
	if (!SysTick->CTRL) {
		sd_mmc_sam_systick_used = true;
		SysTick->LOAD = (120000000 / (8 * 1000)) // SysTick->LOAD = (sysclk_get_cpu_hz() / (8 * 1000))
				* SD_MMC_DEBOUNCE_TIMEOUT;
		SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
	} else {
		sd_mmc_sam_systick_used = false;
#ifdef FREERTOS_USED
		// Note: the define INCLUDE_vTaskDelay must be set to one
		// in FreeRTOSConfig.h file.
		vTaskSetTimeOutState(&xTimeOut);
#else
		delay_ms(SD_MMC_DEBOUNCE_TIMEOUT);
#endif
	}
}

static inline bool SD_MMC_IS_TIMEOUT(void)
{
	if (!sd_mmc_sam_systick_used) {
#ifdef FREERTOS_USED
		portTickType xTicksToWait =
				SD_MMC_DEBOUNCE_TIMEOUT / portTICK_RATE_MS;
		return (xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == pdTRUE);
#else
		return true;
#endif
	}
	if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
		SysTick->CTRL = 0;
		return true;
	}
	return false;
}

static inline void SD_MMC_STOP_TIMEOUT(void)
{
	if (sd_mmc_sam_systick_used) {
		SysTick->CTRL = 0;
	}
}
#endif
//! @}

/**
 * \brief Initialize the SD card in MCI mode.
 *
 * \note
 * This function runs the initialization procedure and the identification
 * process, then it sets the SD/MMC card in transfer state.
 * At last, it will automaticly enable maximum bus width and transfer speed.
 *
 * \return true if success, otherwise false
 */


/**
 * \brief CMD9: Addressed card sends its card-specific
 * data (CSD) on the CMD line mci.
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_cmd9_mci(void)
{
	if (!hsmci_send_cmd(SDMMC_MCI_CMD9_SEND_CSD, (uint32_t)sd_mmc_card->rca << 16)) {
		return false;
	}
	hsmci_get_response_128(sd_mmc_card->csd);
	return true;
}



static bool sd_mmc_mci_card_init(void)
{
	uint8_t v2 = 0;

	// In first, try to install SD/SDIO card
	sd_mmc_card->type = CARD_TYPE_SD;
	sd_mmc_card->version = CARD_VER_UNKNOWN;
	sd_mmc_card->rca = 0;
	sd_mmc_debug("Start SD card install\n\r");

	// Card need of 74 cycles clock minimum to start
	hsmci_send_clock();

	// CMD0 - Reset all cards to idle state.
	if (!hsmci_send_cmd(SDMMC_MCI_CMD0_GO_IDLE_STATE, 0)) {
		return false;
	}
	if (!sd_cmd8(&v2)) {
		return false;
	}
	// Try to get the SDIO card's operating condition
	//if (!sdio_op_cond()) {
	//	return false;
	//}

	if (sd_mmc_card->type & CARD_TYPE_SD) {
		// Try to get the SD card's operating condition
		if (!sd_mci_op_cond(v2)) {
			// It is not a SD card
			sd_mmc_debug("Start MMC Install\n\r");
			sd_mmc_card->type = CARD_TYPE_MMC;
			//return sd_mmc_mci_install_mmc();
		}
	}

	if (sd_mmc_card->type & CARD_TYPE_SD) {
		// SD MEMORY, Put the Card in Identify Mode
		// Note: The CID is not used in this stack
		if (!hsmci_send_cmd(SDMMC_CMD2_ALL_SEND_CID, 0)) {
			return false;
		}
	}
	// Ask the card to publish a new relative address (RCA).
	if (!hsmci_send_cmd(SD_CMD3_SEND_RELATIVE_ADDR, 0)) {
		return false;
	}
	sd_mmc_card->rca = (hsmci_get_response() >> 16) & 0xFFFF;

	// SD MEMORY, Get the Card-Specific Data
	if (sd_mmc_card->type & CARD_TYPE_SD) {
		if (!sd_mmc_cmd9_mci()) {
			return false;
		}
		sd_decode_csd();
	}
	// Select the and put it into Transfer Mode
	if (!hsmci_send_cmd(SDMMC_CMD7_SELECT_CARD_CMD,
			(uint32_t)sd_mmc_card->rca << 16)) {
		return false;
	}
	// SD MEMORY, Read the SCR to get card version
	if (sd_mmc_card->type & CARD_TYPE_SD) {
		if (!sd_acmd51()) {
			return false;
		}
	}
	if (IS_SDIO()) {
		//if (!sdio_get_max_speed()) {
		//	return false;
		//}
	}
	if ((4 <= hsmci_get_bus_width(sd_mmc_slot_sel))) {
		// TRY to enable 4-bit mode
		if (IS_SDIO()) {
			//if (!sdio_cmd52_set_bus_width()) {
			//	return false;
			//}
		}
		if (sd_mmc_card->type & CARD_TYPE_SD) {
			if (!sd_acmd6()) {
				return false;
			}
		}
		// Switch to selected bus mode
		sd_mmc_configure_slot();
	}
	if (hsmci_is_high_speed_capable()) {
		// TRY to enable High-Speed Mode
		if (IS_SDIO()) {
			//if (!sdio_cmd52_set_high_speed()) {
			//	return false;
			//}
		}
		if (sd_mmc_card->type & CARD_TYPE_SD) {
			if (sd_mmc_card->version > CARD_VER_SD_1_0) {
				if (!sd_cm6_set_high_speed()) {
					return false;
				}
			}
		}
		// Valid new configuration
		sd_mmc_configure_slot();
	}
	// SD MEMORY, Set default block size
	if (sd_mmc_card->type & CARD_TYPE_SD) {
		if (!hsmci_send_cmd(SDMMC_CMD16_SET_BLOCKLEN, SD_MMC_BLOCK_SIZE)) {
			return false;
		}
	}
	return true;
}


/**
 * \brief CMD6 for SD - Switch card in high speed mode
 *
 * \note CMD6 for SD is valid under the "trans" state.
 * \note sd_mmc_card->high_speed is updated.
 * \note sd_mmc_card->clock is updated.
 *
 * \return true if success, otherwise false
 */
static bool sd_cm6_set_high_speed(void)
{
	uint8_t switch_status[SD_SW_STATUS_BSIZE];

	if (!hsmci_adtc_start(SD_CMD6_SWITCH_FUNC,
			SD_CMD6_MODE_SWITCH
			| SD_CMD6_GRP6_NO_INFLUENCE
			| SD_CMD6_GRP5_NO_INFLUENCE
			| SD_CMD6_GRP4_NO_INFLUENCE
			| SD_CMD6_GRP3_NO_INFLUENCE
			| SD_CMD6_GRP2_DEFAULT
			| SD_CMD6_GRP1_HIGH_SPEED,
			SD_SW_STATUS_BSIZE, 1, true)) {
		return false;
	}
	if (!hsmci_start_read_blocks(switch_status, 1)) {
		return false;
	}
	if (!hsmci_wait_end_of_read_blocks()) {
		return false;
	}

	if (hsmci_get_response() & CARD_STATUS_SWITCH_ERROR) {
		sd_mmc_debug("%s: CMD6 CARD_STATUS_SWITCH_ERROR\n\r", __func__);
		return false;
	}
	if (SD_SW_STATUS_FUN_GRP1_RC(switch_status)
			== SD_SW_STATUS_FUN_GRP_RC_ERROR) {
		// No supported, it is not a protocol error
		return true;
	}
	if (SD_SW_STATUS_FUN_GRP1_BUSY(switch_status)) {
		sd_mmc_debug("%s: CMD6 SD_SW_STATUS_FUN_GRP1_BUSY\n\r", __func__);
		return false;
	}
	// CMD6 function switching period is within 8 clocks
	// after the end bit of status data.
	hsmci_send_clock();
	sd_mmc_card->high_speed = 1;
	sd_mmc_card->clock *= 2;
	return true;
}




/**
 * \brief CMD8 for SD card - Send Interface Condition Command.
 *
 * \note
 * Send SD Memory Card interface condition, which includes host supply
 * voltage information and asks the card whether card supports voltage.
 * Should be performed at initialization time to detect the card type.
 *
 * \param v2 Pointer to v2 flag to update
 *
 * \return true if success, otherwise false
 *         with a update of \ref sd_mmc_err.
 */
static bool sd_cmd8(uint8_t * v2)
{
	uint32_t resp;

	*v2 = 0;
	// Test for SD version 2
	if (!hsmci_send_cmd(SD_CMD8_SEND_IF_COND,
			SD_CMD8_PATTERN | SD_CMD8_HIGH_VOLTAGE)) {
		return true; // It is not a V2
	}
	// Check R7 response
	resp = hsmci_get_response();
	if (resp == 0xFFFFFFFF) {
		// No compliance R7 value
		return true; // It is not a V2
	}
	if ((resp & (SD_CMD8_MASK_PATTERN | SD_CMD8_MASK_VOLTAGE))
				!= (SD_CMD8_PATTERN | SD_CMD8_HIGH_VOLTAGE)) {
		sd_mmc_debug("%s: CMD8 resp32 0x%08x UNUSABLE CARD\n\r",
				__func__, resp);
		return false;
	}
	sd_mmc_debug("SD card V2\n\r");
	*v2 = 1;
	return true;
}




/**
 * \brief Decodes SD CSD register
 */
static void sd_decode_csd(void)
{
 	uint32_t unit;
	uint32_t mul;
	uint32_t tran_speed;

	// Get SD memory maximum transfer speed in Hz.
	tran_speed = CSD_TRAN_SPEED(sd_mmc_card->csd);
	unit = sd_mmc_trans_units[tran_speed & 0x7];
	mul = sd_trans_multipliers[(tran_speed >> 3) & 0xF];
	sd_mmc_card->clock = unit * mul * 1000;

	/*
	 * Get card capacity.
	 * ----------------------------------------------------
	 * For normal SD/MMC card:
	 * memory capacity = BLOCKNR * BLOCK_LEN
	 * Where
	 * BLOCKNR = (C_SIZE+1) * MULT
	 * MULT = 2 ^ (C_SIZE_MULT+2)       (C_SIZE_MULT < 8)
	 * BLOCK_LEN = 2 ^ READ_BL_LEN      (READ_BL_LEN < 12)
	 * ----------------------------------------------------
	 * For high capacity SD card:
	 * memory capacity = (C_SIZE+1) * 512K byte
	 */
	if (CSD_STRUCTURE_VERSION(sd_mmc_card->csd) >= SD_CSD_VER_2_0) {
		sd_mmc_card->capacity =
				(SD_CSD_2_0_C_SIZE(sd_mmc_card->csd) + 1)
				* 512;
	} else {
		uint32_t blocknr = ((SD_CSD_1_0_C_SIZE(sd_mmc_card->csd) + 1) *
				(1 << (SD_CSD_1_0_C_SIZE_MULT(sd_mmc_card->csd) + 2)));
		sd_mmc_card->capacity = blocknr *
				(1 << SD_CSD_1_0_READ_BL_LEN(sd_mmc_card->csd))
				/ 1024;
	}
}

/**
 * \brief CMD13 - Addressed card sends its status register.
 * This function waits the clear of the busy flag
 *
 * \return true if success, otherwise false
 */
static bool sd_mmc_cmd13(void)
{
	uint32_t nec_timeout;

	/* Wait for data ready status.
	 * Nec timing: 0 to unlimited
	 * However a timeout is used.
	 * 200 000 * 8 cycles
	 */
	nec_timeout = 200000;
	do {
		if (sd_mmc_is_spi()) {
			if (!hsmci_send_cmd(SDMMC_SPI_CMD13_SEND_STATUS, 0)) {
				return false;
			}
			// Check busy flag
			if (!(hsmci_get_response() & 0xFF)) {
				break;
			}
		} else {
			if (!hsmci_send_cmd(SDMMC_MCI_CMD13_SEND_STATUS,
					(uint32_t)sd_mmc_card->rca << 16)) {
				return false;
			}
			// Check busy flag
			if (hsmci_get_response() & CARD_STATUS_READY_FOR_DATA) {
				break;
			}
		}
		if (nec_timeout-- == 0) {
			sd_mmc_debug("%s: CMD13 Busy timeout\n\r", __func__);
			return false;
		}
	} while (1);

	return true;
}


/**
 * \brief ACMD6 - Define the data bus width to 4 bits bus
 *
 * \return true if success, otherwise false
 */
static bool sd_acmd6(void)
{
	// CMD55 - Indicate to the card that the next command is an
	// application specific command rather than a standard command.
	if (!hsmci_send_cmd(SDMMC_CMD55_APP_CMD, (uint32_t)sd_mmc_card->rca << 16)) {
		return false;
	}
	// 10b = 4 bits bus
	if (!hsmci_send_cmd(SD_ACMD6_SET_BUS_WIDTH, 0x2)) {
		return false;
	}
	sd_mmc_card->bus_width = 4;
	sd_mmc_debug("%d-bit bus width enabled.\n\r", (int)sd_mmc_card->bus_width);
	return true;
}

/**
 * \brief ACMD51 - Read the SD Configuration Register.
 *
 * \note
 * SD Card Configuration Register (SCR) provides information on the SD Memory
 * Card's special features that were configured into the given card. The size
 * of SCR register is 64 bits.
 *
 *
 * \return true if success, otherwise false
 */
static bool sd_acmd51(void)
{
	uint8_t scr[SD_SCR_REG_BSIZE];

	// CMD55 - Indicate to the card that the next command is an
	// application specific command rather than a standard command.
	if (!hsmci_send_cmd(SDMMC_CMD55_APP_CMD, (uint32_t)sd_mmc_card->rca << 16)) {
		return false;
	}
	if (!hsmci_adtc_start(SD_ACMD51_SEND_SCR, 0,
			SD_SCR_REG_BSIZE, 1, true)) {
		return false;
	}
	if (!hsmci_start_read_blocks(scr, 1)) {
		return false;
	}
	if (!hsmci_wait_end_of_read_blocks()) {
		return false;
	}

	// Get SD Memory Card - Spec. Version
	switch (SD_SCR_SD_SPEC(scr)) {
	case SD_SCR_SD_SPEC_1_0_01:
		sd_mmc_card->version = CARD_VER_SD_1_0;
		break;

	case SD_SCR_SD_SPEC_1_10:
		sd_mmc_card->version = CARD_VER_SD_1_10;
		break;

	case SD_SCR_SD_SPEC_2_00:
		if (SD_SCR_SD_SPEC3(scr) == SD_SCR_SD_SPEC_3_00) {
			sd_mmc_card->version = CARD_VER_SD_3_0;
		} else {
			sd_mmc_card->version = CARD_VER_SD_2_0;
		}
		break;

	default:
		sd_mmc_card->version = CARD_VER_SD_1_0;
		break;
	}
	return true;
}

/**
 * \brief Select a card slot and initialize the associated hsmci
 *
 * \param slot  Card slot number
 *
 * \retval SD_MMC_ERR_SLOT     Wrong slot number
 * \retval SD_MMC_ERR_NO_CARD  No card present on slot
 * \retval SD_MMC_ERR_UNUSABLE Unusable card
 * \retval SD_MMC_INIT_ONGOING Card initialization requested
 * \retval SD_MMC_OK           Card present
 */
static sd_mmc_err_t sd_mmc_select_slot(uint8_t slot)
{
	if (slot >= SD_MMC_MEM_CNT) {
		return SD_MMC_ERR_SLOT;
	}
	//assert(sd_mmc_nb_block_remaining == 0);

#if (defined SD_MMC_0_CD_GPIO)
	//! Card Detect pins
	if (ioport_get_pin_level(sd_mmc_cards[slot].cd_gpio)
			!= SD_MMC_0_CD_DETECT_VALUE) {
		if (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_DEBOUNCE) {
			SD_MMC_STOP_TIMEOUT();
		}
		sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_NO_CARD;
		return SD_MMC_ERR_NO_CARD;
	}
	if (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_NO_CARD) {
		// A card plug on going, but this is not initialized
		sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_DEBOUNCE;
		// Debounce + Power On Setup
		SD_MMC_START_TIMEOUT();
		return SD_MMC_ERR_NO_CARD;
	}
	if (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_DEBOUNCE) {
		if (!SD_MMC_IS_TIMEOUT()) {
			// Debounce on going
			return SD_MMC_ERR_NO_CARD;
		}
		// Card is not initialized
		sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_INIT;
		// Set 1-bit bus width and low clock for initialization
		sd_mmc_cards[slot].clock = SDMMC_CLOCK_INIT;
		sd_mmc_cards[slot].bus_width = 1;
		sd_mmc_cards[slot].high_speed = 0;
	}
	if (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_UNUSABLE) {
		return SD_MMC_ERR_UNUSABLE;
	}
#else
	// No pin card detection, then always try to install it
	if ((sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_NO_CARD)
			|| (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_UNUSABLE)) {
		// Card is not initialized
		sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_INIT;
		// Set 1-bit bus width and low clock for initialization
		sd_mmc_cards[slot].clock = SDMMC_CLOCK_INIT;
		sd_mmc_cards[slot].bus_width = 1;
		sd_mmc_cards[slot].high_speed = 0;
	}
#endif

	// Initialize interface
	sd_mmc_slot_sel = slot;
	sd_mmc_card = &sd_mmc_cards[slot];
	sd_mmc_configure_slot();
	return (sd_mmc_cards[slot].state == SD_MMC_CARD_STATE_INIT) ?
			SD_MMC_INIT_ONGOING : SD_MMC_OK;
}

/**
 * \brief Configures the hsmci with the selected card configuration
 */
static void sd_mmc_configure_slot(void)
{
	hsmci_select_device(sd_mmc_slot_sel, sd_mmc_card->clock,sd_mmc_card->bus_width, sd_mmc_card->high_speed);
	
	//hsmci_select_device(sd_mmc_slot_sel, sd_mmc_card->clock,
	//		sd_mmc_card->bus_width, sd_mmc_card->high_speed);
}

/**
 * \brief Deselect the current card slot
 */
static void sd_mmc_deselect_slot(void)
{
	if (sd_mmc_slot_sel < SD_MMC_MEM_CNT) {
		hsmci_deselect_device(sd_mmc_slot_sel);
	}
}




//-------------------------------------------------------------------
//--------------------- PUBLIC FUNCTIONS ----------------------------

void sd_mmc_init(void)
{
	pio_setMux(PIOA,26,C); //MCDA2
	pio_setMux(PIOA,27,C); //MCDA3
	pio_setMux(PIOA,28,C); //MCCDA
	pio_setMux(PIOA,29,C); //MCCK
	pio_setMux(PIOA,30,C); //MCDA0
	pio_setMux(PIOA,31,C); //MCDA1
	//! Enable the PMC clock for the card detect pins
#if (defined SD_MMC_0_CD_GPIO) && (!defined SAM4L)
# include "pmc.h"
# define SD_MMC_ENABLE_CD_PIN(slot, unused) \
	pmc_enable_periph_clk(SD_MMC_##slot##_CD_PIO_ID);
	MREPEAT(SD_MMC_MEM_CNT, SD_MMC_ENABLE_CD_PIN, ~)
# undef SD_MMC_ENABLE_CD_PIN
#endif
	//! Enable the PMC clock for the card write protection pins
#if (defined SD_MMC_0_WP_GPIO) && (!defined SAM4L)
# include "pmc.h"
# define SD_MMC_ENABLE_WP_PIN(slot, unused) \
	pmc_enable_periph_clk(SD_MMC_##slot##_WP_PIO_ID);
	MREPEAT(SD_MMC_MEM_CNT, SD_MMC_ENABLE_WP_PIN, ~)
# undef SD_MMC_ENABLE_WP_PIN
#endif
	for (uint8_t slot = 0; slot < SD_MMC_MEM_CNT; slot++) {
		sd_mmc_cards[slot].state = SD_MMC_CARD_STATE_NO_CARD;
	}
	sd_mmc_slot_sel = 0xFF; // No slot configurated
	hsmci_init();
	//hsmci_init();
}

uint8_t sd_mmc_nb_slot(void)
{
	return SD_MMC_MEM_CNT;
}

sd_mmc_err_t sd_mmc_check(uint8_t slot)
{
	sd_mmc_err_t sd_mmc_err;

	sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_INIT_ONGOING) {
		sd_mmc_deselect_slot();
		return sd_mmc_err;
	}

	// Initialization of the card requested
	if (sd_mmc_mci_card_init() ) {
		sd_mmc_debug("SD/MMC card ready\n\r");
		sd_mmc_card->state = SD_MMC_CARD_STATE_READY;
		sd_mmc_deselect_slot();
		// To notify that the card has been just initialized
		// It is necessary for USB Device MSC
		return SD_MMC_INIT_ONGOING;
	}
	sd_mmc_debug("SD/MMC card initialization failed\n\r");
	sd_mmc_card->state = SD_MMC_CARD_STATE_UNUSABLE;
	sd_mmc_deselect_slot();
	return SD_MMC_ERR_UNUSABLE;
}

card_type_t sd_mmc_get_type(uint8_t slot)
{
	if (SD_MMC_OK != sd_mmc_select_slot(slot)) {
		return CARD_TYPE_UNKNOWN;
	}
	sd_mmc_deselect_slot();
	return sd_mmc_card->type;
}

card_version_t sd_mmc_get_version(uint8_t slot)
{
	if (SD_MMC_OK != sd_mmc_select_slot(slot)) {
		return CARD_VER_UNKNOWN;
	}
	sd_mmc_deselect_slot();
	return sd_mmc_card->version;
}

uint32_t sd_mmc_get_capacity(uint8_t slot)
{
	if (SD_MMC_OK != sd_mmc_select_slot(slot)) {
		return 0;
	}
	sd_mmc_deselect_slot();
	return sd_mmc_card->capacity;
}

bool sd_mmc_is_write_protected(uint8_t slot)
{
	//UNUSED(slot);
#if (defined SD_MMC_0_WP_GPIO)
	//! Card Detect pins
	if (ioport_get_pin_level(sd_mmc_cards[slot].wp_gpio)
			== SD_MMC_0_WP_DETECT_VALUE) {
		return true;
	}
#endif
	return false;
}

sd_mmc_err_t sd_mmc_init_read_blocks(uint8_t slot, uint32_t start,
		uint16_t nb_block)
{
	sd_mmc_err_t sd_mmc_err;
	uint32_t cmd, arg, resp;

	sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_OK) {
		return sd_mmc_err;
	}

	// Wait for data ready status
	if (!sd_mmc_cmd13()) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}

	if (nb_block > 1) {
		cmd = SDMMC_CMD18_READ_MULTIPLE_BLOCK;
	} else {
		cmd = SDMMC_CMD17_READ_SINGLE_BLOCK;
	}
	/*
	 * SDSC Card (CCS=0) uses byte unit address,
	 * SDHC and SDXC Cards (CCS=1) use block unit address (512 Bytes unit).
	 */
	if (sd_mmc_card->type & CARD_TYPE_HC) {
		arg = start;
	} else {
		arg = (start * SD_MMC_BLOCK_SIZE);
	}

	if (!hsmci_adtc_start(cmd, arg, SD_MMC_BLOCK_SIZE, nb_block, true)) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}
	// Check response
	if (sd_mmc_is_mci()) {
		resp = hsmci_get_response();
		if (resp & CARD_STATUS_ERR_RD_WR) {
			sd_mmc_debug("%s: Read blocks %02d resp32 0x%08x CARD_STATUS_ERR_RD_WR\n\r",
					__func__, (int)SDMMC_CMD_GET_INDEX(cmd), resp);
			sd_mmc_deselect_slot();
			return SD_MMC_ERR_COMM;
		}
	}
	sd_mmc_nb_block_remaining = nb_block;
	sd_mmc_nb_block_to_tranfer = nb_block;
	return SD_MMC_OK;
}

sd_mmc_err_t sd_mmc_start_read_blocks(void *dest, uint16_t nb_block)
{
	//assert(sd_mmc_nb_block_remaining >= nb_block);
	if (!hsmci_start_read_blocks(dest, nb_block)) {
		sd_mmc_nb_block_remaining = 0;
		return SD_MMC_ERR_COMM;
	}
	sd_mmc_nb_block_remaining -= nb_block;
	return SD_MMC_OK;
}

sd_mmc_err_t sd_mmc_wait_end_of_read_blocks(bool abort)
{
	if (!hsmci_wait_end_of_read_blocks()) {
		return SD_MMC_ERR_COMM;
	}
	if (abort) {
		sd_mmc_nb_block_remaining = 0;
	} else if (sd_mmc_nb_block_remaining) {
		return SD_MMC_OK;
	}

	// All blocks are transfered then stop read operation
	if (sd_mmc_nb_block_to_tranfer == 1) {
		// Single block transfer, then nothing to do
		sd_mmc_deselect_slot();
		return SD_MMC_OK;
	}
	// WORKAROUND for no compliance card (Atmel Internal ref. !MMC7 !SD19):
	// The errors on this command must be ignored
	// and one retry can be necessary in SPI mode for no compliance card.
	if (!hsmci_adtc_stop(SDMMC_CMD12_STOP_TRANSMISSION, 0)) {
		hsmci_adtc_stop(SDMMC_CMD12_STOP_TRANSMISSION, 0);
	}
	sd_mmc_deselect_slot();
	return SD_MMC_OK;
}

sd_mmc_err_t sd_mmc_init_write_blocks(uint8_t slot, uint32_t start,
		uint16_t nb_block)
{
	sd_mmc_err_t sd_mmc_err;
	uint32_t cmd, arg, resp;

	sd_mmc_err = sd_mmc_select_slot(slot);
	if (sd_mmc_err != SD_MMC_OK) {
		return sd_mmc_err;
	}
	if (sd_mmc_is_write_protected(slot)) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_WP;
	}

	if (nb_block > 1) {
		cmd = SDMMC_CMD25_WRITE_MULTIPLE_BLOCK;
	} else {
		cmd = SDMMC_CMD24_WRITE_BLOCK;
	}
	/*
	 * SDSC Card (CCS=0) uses byte unit address,
	 * SDHC and SDXC Cards (CCS=1) use block unit address (512 Bytes unit).
	 */
	if (sd_mmc_card->type & CARD_TYPE_HC) {
		arg = start;
	} else {
		arg = (start * SD_MMC_BLOCK_SIZE);
	}
	if (!hsmci_adtc_start(cmd, arg, SD_MMC_BLOCK_SIZE, nb_block, true)) {
		sd_mmc_deselect_slot();
		return SD_MMC_ERR_COMM;
	}
	// Check response
	if (sd_mmc_is_mci()) {
		resp = hsmci_get_response();
		if (resp & CARD_STATUS_ERR_RD_WR) {
			sd_mmc_debug("%s: Write blocks %02d r1 0x%08x CARD_STATUS_ERR_RD_WR\n\r",
					__func__, (int)SDMMC_CMD_GET_INDEX(cmd), resp);
			sd_mmc_deselect_slot();
			return SD_MMC_ERR_COMM;
		}
	}
	sd_mmc_nb_block_remaining = nb_block;
	sd_mmc_nb_block_to_tranfer = nb_block;
	return SD_MMC_OK;
}

sd_mmc_err_t sd_mmc_start_write_blocks(const void *src, uint16_t nb_block)
{
	//assert(sd_mmc_nb_block_remaining >= nb_block);
	if (!hsmci_start_write_blocks(src, nb_block)) {
		sd_mmc_nb_block_remaining = 0;
		return SD_MMC_ERR_COMM;
	}
	sd_mmc_nb_block_remaining -= nb_block;
	return SD_MMC_OK;
}

sd_mmc_err_t sd_mmc_wait_end_of_write_blocks(bool abort)
{
	if (!hsmci_wait_end_of_write_blocks()) {
		return SD_MMC_ERR_COMM;
	}
	if (abort) {
		sd_mmc_nb_block_remaining = 0;
	} else if (sd_mmc_nb_block_remaining) {
		return SD_MMC_OK;
	}

	// All blocks are transfered then stop write operation
	if (sd_mmc_nb_block_to_tranfer == 1) {
		// Single block transfer, then nothing to do
		sd_mmc_deselect_slot();
		return SD_MMC_OK;
	}

	if (sd_mmc_is_mci()) {
		// Note: SPI multiblock writes terminate using a special
		// token, not a STOP_TRANSMISSION request.
		if (!hsmci_adtc_stop(SDMMC_CMD12_STOP_TRANSMISSION, 0)) {
			sd_mmc_deselect_slot();
			return SD_MMC_ERR_COMM;
		}
	}
	sd_mmc_deselect_slot();
	return SD_MMC_OK;
}


/**
 * \brief Ask to all cards to send their operations conditions (MCI only).
 * - ACMD41 sends operation condition command.
 * - ACMD41 reads OCR
 *
 * \param v2   Shall be 1 if it is a SD card V2
 *
 * \return true if success, otherwise false
 */
static bool sd_mci_op_cond(uint8_t v2)
{
	uint32_t arg, retry, resp;

	/*
	 * Timeout 1s = 400KHz / ((6+6+6+6)*8) cylces = 2100 retry
	 * 6 = cmd byte size
	 * 6 = response byte size
	 * 6 = cmd byte size
	 * 6 = response byte size
	 */
	retry = 2100;
	do {
		// CMD55 - Indicate to the card that the next command is an
		// application specific command rather than a standard command.
		if (!hsmci_send_cmd(SDMMC_CMD55_APP_CMD, 0)) {
			sd_mmc_debug("%s: CMD55 Fail\n\r", __func__);
			return false;
		}

		// (ACMD41) Sends host OCR register
		arg = SD_MMC_VOLTAGE_SUPPORT;
		if (v2) {
			arg |= SD_ACMD41_HCS;
		}
		// Check response
		if (!hsmci_send_cmd(SD_MCI_ACMD41_SD_SEND_OP_COND, arg)) {
			sd_mmc_debug("%s: ACMD41 Fail\n\r", __func__);
			return false;
		}
		resp = hsmci_get_response();
		if (resp & OCR_POWER_UP_BUSY) {
			// Card is ready
			if ((resp & OCR_CCS) != 0) {
				sd_mmc_card->type |= CARD_TYPE_HC;
			}
			break;
		}
		if (retry-- == 0) {
			sd_mmc_debug("%s: ACMD41 Timeout on busy, resp32 0x%08x \n\r",
					__func__, resp);
			return false;
		}
	} while (1);
	return true;
}

//! @}
