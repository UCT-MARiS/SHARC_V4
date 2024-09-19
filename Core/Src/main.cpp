#include "main.hpp"
#include "hal_interface.hpp"
#include "hal_impl.hpp"

#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wformat"

#ifdef __cplusplus 
extern "C" {
#endif

//HAL Includes
#include "init.h"

//Standard Includes
#include "string.h"
#include <stdarg.h>
#include "stdio.h"

// DSP Library Includes
#include "arm_math.h"
#include "fast_math_functions.h"
#include "transform_functions.h"
#include "wave_functions.h"

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
HAL_Impl halImpl;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

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

    // FFT operation using CMSIS DSP library
    const uint32_t fftSize = 1024;
    const uint32_t ifftFlag = 0;
    const uint32_t doBitReverse = 1;

    // Input signal (example: sine wave)
    float32_t inputSignal[fftSize];
    float32_t fs1 = 20.0f;
    float32_t fs2 = 10.0f;

    for (uint32_t i = 0; i < fftSize; i++) {
        inputSignal[i] = arm_sin_f32(2 * PI * i * fs1 / fftSize);
    }

    // Output buffer
    float32_t outputSignal[fftSize];

    // Welch PSD estimate
    pwelch(inputSignal, fftSize, outputSignal);

    // Print FFT result
    for (uint32_t i = 0; i < fftSize/2; i++) {
        printmsg("%f, \r\n", i, outputSignal[i]);
    }
    
    // Start scheduler 
    vTaskStartScheduler();

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

    TickType_t xLastWakeTime;

    const TickType_t xDelay3ms = pdMS_TO_TICKS( 3 );
    
    /*
    * The xLastWakeTime variable needs to be initialized with the current tick
    * count. Note that this is the only time the variable is explicitly
    * written to. After this xLastWakeTime is managed automatically by the
    * vTaskDelayUntil() API function.
    */
    xLastWakeTime = xTaskGetTickCount();
    /* As per most tasks, this task is implemented in an infinite loop. */

    for(;;) {
        halImpl.HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        vTaskDelay(pxDelay);

    }
}

/**
 * @brief Default mode is to put t  he Cortex-M4 in sleep mode when the RTOS is idle.
 * 
 */
void vApplicationIdleHook(void) {
        // Put the Cortex-M4 into sleep mode
        __WFI();
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



