/*
 * MSCTask.c
 *
 * Created: 04.02.2015 12:03:27
 *  Author: Will
 */ 

#include "Task_USBMSC.h"

#include "DriversNotInBase/SD_FAT/definitionsCompiler.h"
#include "DriversNotInBase/usb/main.h"
#include "DriversNotInBase/usb/udc.h"
#include "DriversNotInBase/usb/udi_msc.h"
#include "DriversNotInBase/usb/udp_device.h"
#include "DriversNotInBase/config_sd_msc/ctrl_access.h"

#include "sam4e-base/RevolveDrivers/pio.h"

#include "DriversNotInBase/IO_DashInit.h"

static volatile bool main_b_msc_enable = false;
SemaphoreHandle_t main_trans_semphr;
SemaphoreHandle_t file_access_mutex = NULL;

TaskHandle_t mscTaskHandle = NULL;
void usbMscTask() {
	
	
	// Create a semaphore to manage the memories data transfer
	main_trans_semphr = xSemaphoreCreateBinary();
	NVIC_DisableIRQ((IRQn_Type) ID_UDP); //Interrupts must be turned off
	udd_enable_periph_ck(); // UDP peripheral clock must be enabled to change UDP register settings
	UDP->UDP_TXVC |= 1 <<8;
	UDP->UDP_TXVC &= ~(1<<9);
	udc_stop(); // Stop UDP and detach (disable pullup on DDP which fucks with voltage level on detect pin)
	
	while(1) {
		//static bool led_status = true;
		//pio_setOutput(ECU_LED_PIO,ECU_LED_PIN,led_status);
		//led_status != led_status;
		if (pio_readPin(DETECT_USB_PIO,DETECT_USB_PIN) == 1){
			xSemaphoreTake(file_access_mutex,portMAX_DELAY);
			udc_start();
			while (1) {
				if( xSemaphoreTake( main_trans_semphr, 0 ) == pdTRUE ) {
					udi_msc_process_trans();
				}
				if ( (pio_readPin(DETECT_USB_PIO,DETECT_USB_PIN) == 0)) {
					xSemaphoreGive(file_access_mutex);
					NVIC_DisableIRQ((IRQn_Type) ID_UDP); //Interrupts must be turned off
					udd_enable_periph_ck(); // UDP peripheral clock must be enabled to change UDP register settings
					UDP->UDP_TXVC |= 1 <<8;
					UDP->UDP_TXVC &= ~(1<<9);
					udc_stop(); // Stop UDP and detach (disable pullup on DDP which fucks with voltage level on detect pin)
					break;
				}
			}
		}
		else {
			vTaskDelay(150/portTICK_RATE_MS);
		}
	}
}

static void main_memories_trans_task(void *pvParameters)
{
	UNUSED(pvParameters);
	while (true) {
		// Wait for a semaphore which signals that a transfer is requested
		if( xSemaphoreTake( main_trans_semphr, portMAX_DELAY ) == pdTRUE ) {
			udi_msc_process_trans();
		}
	}
}

void main_msc_notify_trans(void)
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	// One transfer is requested
	// It is now time for main_memories_trans_task() to run
	xSemaphoreGiveFromISR( main_trans_semphr, &xHigherPriorityTaskWoken );
}

void main_suspend_action(void)
{
	//ui_powerdown();
}

void main_resume_action(void)
{
	//ui_wakeup();
}

void main_sof_action(void)
{
	if (!main_b_msc_enable)
	return;
	//ui_process(udd_get_frame_number());
}

bool main_msc_enable(void)
{
	main_b_msc_enable = true;
	return true;
}

void main_msc_disable(void)
{
	main_b_msc_enable = false;
}