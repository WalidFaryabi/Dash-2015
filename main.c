#include "sam.h"
#include <string.h>

#include "sam4e-base/FreeRTOS/Source/include/FreeRTOS.h"
#include "sam4e-base/FreeRTOS/Source/include/task.h"
#include "sam4e-base/FreeRTOS/Source/include/semphr.h"


#include "DriversNotInBase/Interrupt/interrupt_sam_nvic.h"
#include "DriversNotInBase/usb/main.h"
#include "DriversNotInBase/SD_FAT/sd_mmc.h"
#include "DriversNotInBase/usb/udc.h"
#include "DriversNotInBase/usb/udp_device.h"
#include "DriversNotInBase/config_sd_msc/ctrl_access.h"
#include "DriversNotInBase/SD_FAT/fat/ff.h"


#include "sam4e-base/RevolveDrivers/eefc.h"
#include "sam4e-base/RevolveDrivers/pmc.h"
#include "sam4e-base/RevolveDrivers/pio.h"

#include "DriversNotInBase/delay.h"
#include "DriversNotInBase/IO_DashInit.h"


#include "Task_ButtonInput.h"
#include "Task_DataLogger.h"
#include "Task_DeviceStatus.h"
#include "Task_Menu.h"
#include "Task_RemoteControl.h"
#include "Task_USBMSC.h"

#include "CanHandler.h"

#include "sam4e-base/FreeRTOSConfig.h"


/*
Changed:
PMC and Pio modified
Feertosconfig. Changed max timer queue, increased freertos heap (find a good reason for this)
*/


static void hardwareInit();

int main(void) {
	
	hardwareInit();
	
	file_access_mutex = xSemaphoreCreateMutex();
	xButtonStruct = xSemaphoreCreateMutex();
	spi_handlerIsDoneSempahore = xSemaphoreCreateBinary();
	spi_mutex = xSemaphoreCreateMutex();
	can_mutex_0 = xSemaphoreCreateMutex();
	can_mutex_1 = xSemaphoreCreateMutex();
	
	
	BaseType_t status;
	uint32_t bytesremaining;
	
	status = xTaskCreate(dashTask,"dashTask",2000, NULL,  tskIDLE_PRIORITY + 3, NULL);
	bytesremaining = xPortGetFreeHeapSize();
	status = xTaskCreate(usbMscTask,"MscTask",1000, NULL, tskIDLE_PRIORITY + 1, &mscTaskHandle);
	bytesremaining = xPortGetFreeHeapSize();
	status = xTaskCreate(dataLoggerTask,"Datalogger",2500,NULL,tskIDLE_PRIORITY +4, &dataLoggerHandle);
	bytesremaining = xPortGetFreeHeapSize();
	status = xTaskCreate(Task_remoteControl,"remote",500, NULL, tskIDLE_PRIORITY + 2,NULL);
	bytesremaining = xPortGetFreeHeapSize();
	xTaskCreate(deviceStatusTask,"device",500,NULL,tskIDLE_PRIORITY + 4,NULL);
	xTaskCreate(Task_ButtonInput, "buttonTask", configMINIMAL_STACK_SIZE, NULL,  tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(Task_RotaryEncoder,"rotary", configMINIMAL_STACK_SIZE,NULL, tskIDLE_PRIORITY + 2, NULL);
	vTaskStartScheduler();
	
	while(1) {
		
	}
}

static void hardwareInit() {
	WDT->WDT_MR |= 1<<15; // Disable Watch dog timer
	irq_initialize_vectors();
	cpu_irq_enable();
	init_flash();
	struct PmcInit pmcInit = {
		.freq = EXTERNAL,
		.css = PLLA_CLOCK,
		.pres = CLK_2,
		.divide = 1,
		.multiply = 15 // Endra driver til å trekke fra 1 automatisk
	};
	pmc_init(pmcInit);
	//Timer init for datalogger used in Canhandler
	RTT->RTT_MR |= 1<<20;
	RTT->RTT_MR = 0;
	RTT->RTT_MR &= ~(1<<17);
	RTT->RTT_MR = 33; // Sets divider to 33 which gives a 1 ms tick period
	SystemCoreClock = 120000000; // Set the system core clock parameter in freertos config
	//USB DIV = 5 ( 16*15/5 = 48 Mhz)
	delay_set_frequency(120000000);
	
	//NVIC Setup
	NVIC_SetPriorityGrouping(0U); // Set all to preempt interupts
	
	dash_io_init();
	init_CanHandling();
	sd_mmc_init();	
	struct SpiMaster SpiMasterSettings = {
		.NVIC_spi_interrupt_priority = 10,
		.cs_0 = PA11,
		.cs_1 = none1,
		.cs_2 = none2,
		.cs_3 = none3
		};
	struct SpiDevice Spi0 = {
		.chip_select = 0,
		.bits_per_transfer = 8,
		.delay_between_two_consecutive_transfers = 0,
		.peripheral_clock_hz = 120000000,
		.spi_baudRate_hz = 8000000,
		.time_until_first_valid_SPCK = 0,
		.spi_mode = MODE_0
	};

	spi_masterInit(SpiMasterSettings);
	spi_deviceInit(Spi0);
}


void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is
	called if a stack overflow is detected. */
	vAssertCalled( __LINE__, __FILE__ );
}
/*-----------------------------------------------------------*/

void vAssertCalled( uint32_t ulLine, const char *pcFile )
{
/* The following two variables are just to ensure the parameters are not
optimised away and therefore unavailable when viewed in the debugger. */
volatile uint32_t ulLineNumber = ulLine, ulSetNonZeroInDebuggerToReturn = 0;
volatile const char * const pcFileName = pcFile;

	taskENTER_CRITICAL();
	while( ulSetNonZeroInDebuggerToReturn == 0 )
	{
		/* If you want to set out of this function in the debugger to see the
		assert() location then set ulSetNonZeroInDebuggerToReturn to a non-zero
		value. */
	}
	taskEXIT_CRITICAL();

	( void ) pcFileName;
	( void ) ulLineNumber;
}
/*-----------------------------------------------------------*/

/* Provided to keep the linker happy. */
void _exit_( int status )
{
	( void ) status;
	vAssertCalled( __LINE__, __FILE__ );
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Provided to keep the linker happy. */
int _read( void )
{
	return 0;
}
/*-----------------------------------------------------------*/

/* Provided to keep the linker happy. */
int _write( int x )
{
	( void ) x;
	return 0;
}
/*-----------------------------------------------------------*/