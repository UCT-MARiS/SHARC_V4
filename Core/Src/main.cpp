#include "main.hpp"
#include "hal_interface.hpp"
#include "hal_impl.hpp"
#include "u-blox_m9n_Library.h" 

#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wformat"

#ifdef __cplusplus 
extern "C" {
#endif
#include "init.h"
#include "string.h"
#include <stdarg.h>
#include "stdio.h"

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <event_groups.h>
#ifdef __cplusplus 
}
#endif

//======================== 0. Peripheral Handles ============================================================
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
HAL_Impl halImpl;

//======================== 0. END ============================================================================

//======================== 1. Function Prototypes ============================================================
// Function pointers for HAL functions
static void LED_task(void *args); 
//======================== 1. END ============================================================================

int main(void) {
    
//======================== 1. SYSTEM INIT & CLOCK CONFIG ========================//
    HAL_Init();
    SystemClock_Config();
    setupHAL(&halImpl);

    halImpl.MX_USART2_UART_Init();


	//printmsg("SHARC BUOY STARTING! \r\n");
//=================================== 1. END ====================================//

    // Create a blinking LED task for the on-board LED.
    static StaticTask_t exampleTaskTCB;
    static StackType_t exampleTaskStack[ 512 ];

    // Create the blink task
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;  // 500 ms delay

    TaskHandle_t returnStatus = xTaskCreateStatic( LED_task,
                                  "Blink_LED",
                                  configMINIMAL_STACK_SIZE,
                                  (void*)xDelay,
                                  configMAX_PRIORITIES - 1U,
                                  &( exampleTaskStack[ 0 ] ),
                                  &( exampleTaskTCB ) );

    
    printmsg("Task creation status: %d\r\n", returnStatus);

    // Create GNSS Instance
    SFE_UBLOX_GNSS myGNSS;
    //myGNSS.enableDebugging(&hlpuart1, true); // Enable debug messages over LPUART1
 
    if (myGNSS.begin(&huart2) == true)
    {
        printmsg("GNSS serial connected \r\n");
        myGNSS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output UBX only
        myGNSS.saveConfiguration(); //Save the current settings to flash and BBR;

        long latitude = myGNSS.getLatitude();   //Returns the latitude in degrees x10^-7 as a long integer
        printmsg("Latitude: %d\r\n", latitude);

        long longitude = myGNSS.getLongitude(); //Returns the longitude in degrees x10^-7 as a long integer
        printmsg("Longitude: %d\r\n", longitude);

        long PDOP = myGNSS.getPDOP(); //Positional Dilution of Precision
        printmsg("PDOP: %d\r\n", PDOP);

        long Alitude_MSL = myGNSS.getAltitudeMSL(); //Returns the altitude above Mean Sea Level in mm
        printmsg("Altitude MSL: %d\r\n", Alitude_MSL);

        long speed = myGNSS.getGroundSpeed(); //Returns the ground speed in mm/s
        printmsg("Speed: %d\r\n", speed);

        long heading = myGNSS.getHeading(); //Returns the heading in degrees
        printmsg("Heading: %d\r\n", heading);

        long year = myGNSS.getYear(); //Returns the year (UTC)
        printmsg("Year: %d\r\n", year);









        // Start scheduler 
        vTaskStartScheduler();
    }
    else
        printmsg("Unable to connect \r\n");

    // Start scheduler 
    // vTaskStartScheduler();

while(1){
	halImpl.HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	halImpl.HAL_Delay(1000);
}


}


//Debug Print
void printmsg(char *format,...) {
    char str[80];

    /*Extract the the argument list using VA apis */
    va_list args;
    va_start(args, format);
    vsprintf(str, format,args);
    halImpl.HAL_UART_Transmit(&hlpuart1,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
    va_end(args);
}


static void LED_task(void *pvParameters) {

    TickType_t pxDelay = (TickType_t)pvParameters;

    for(;;) {
        halImpl.HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        vTaskDelay(pxDelay);

    }
}

#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )

    void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName )
    {
        /* Check pcTaskName for the name of the offending task,
         * or pxCurrentTCB if pcTaskName has itself been corrupted. */
        ( void ) xTask;
        ( void ) pcTaskName;
    }

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */



