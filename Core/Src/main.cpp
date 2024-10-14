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

//FATFS includes
#include "fatfs.h"
#include "HAL_SD.h" //SD Card Interface - closely coupled to HAL


#ifdef __cplusplus 
}
#endif

//======================== 0. Peripheral Handles ============================================================
UART_HandleTypeDef hlpuart1;
HAL_Impl halImpl;
SD_HandleTypeDef hsd1;
RTC_HandleTypeDef hrtc;

//======================== 0. END ============================================================================

//======================== 1. Function Prototypes ============================================================

//Tasks
static void LED_task(void *args); 
static void WaveProcTask(void *pvParameters);
static void updateRTC(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate);

//Debugging
void printmsg(char *format,...);

//======================== 1. END ============================================================================

int main(void) {
    
//======================== 2. SYSTEM INIT & CLOCK CONFIG ========================//
    // Initialize the HAL Library
    if(HAL_Init() != HAL_OK)
    {
        printmsg("HAL Library Initialization Failed! \r\n");
        Error_Handler();
    }
    
    // Note: 
    SystemClock_Config(); // Configure the system clock
    setupHAL(&halImpl); // Initialize the HAL with the specified interface
    __enable_irq(); // Enable global interrupts
    // Message to indicate that the system has started
    printmsg("SHARC BUOY STARTING! \r\n");

//=================================== 2. END ====================================//

//======================== 3. SENSOR INITIALIZATION ========================//

    //Set RTC time and print out value
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
    sDate.Month = RTC_MONTH_OCTOBER;
    sDate.Date = 0xA; 
    sDate.Year = 0xA; // 10/10/2024

    updateRTC(&hrtc, &sTime, &sDate);
    printmsg("RTC Time: %02d:%02d:%02d \r\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
    printmsg("RTC Date: %02d/%02d/%02d \r\n", sDate.Date, sDate.Month, sDate.Year);

//=================================== 3. END ====================================//

//======================== 4. TASK CREATION ============================================================

    // Create a blinking LED task for the on-board LED.
    static StaticTask_t exampleTaskTCB;
    static StackType_t exampleTaskStack[ 512 ];

    // Create the blink task
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;  // 500 ms delay

    TaskHandle_t returnStatus = xTaskCreateStatic( LED_task,
                                  "Blink_LED",
                                  configMINIMAL_STACK_SIZE,
                                  (void*)xDelay,
                                  configMAX_PRIORITIES - 2U,
                                  &( exampleTaskStack[ 0 ] ),
                                  &( exampleTaskTCB ) );


    // Create a task to check SD card functions.
    static StaticTask_t waveProcTaskTCB;
    static StackType_t waveProcTaskStack[ 16384 ];

    TaskHandle_t WaveProcTaskHandle = xTaskCreateStatic(
        WaveProcTask,          // Function that implements the task.
        "WaveProcTask",        // Text name for the task.
        16384,                 // Stack size in words, not bytes.
        NULL,                // Parameter passed into the task.
        configMAX_PRIORITIES - 1U, // Priority at which the task is created.
        waveProcTaskStack,     // Array to use as the task's stack.
        &waveProcTaskTCB       // Variable to hold the task's data structure.
    );

    if (WaveProcTaskHandle == NULL) {
        printmsg("Failed to create SDCardTask\r\n");
    }
//======================== 4. END ============================================================


    // Start scheduler 
    vTaskStartScheduler();

// Never reach this point
while(1){
	//  Do nothing
}


}

//======================== 5. DEBUGGING FUNCTIONS ============================================================`

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

//======================== 5. END ============================================================

//======================== 6. General Functions ==============================================
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief Function to update the RTC time and date
 * 
 * @param hrtc 
 * @param sTime 
 * @param sDate 
 */
void updateRTC(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate)
{
    HAL_RTC_SetTime(hrtc, sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(hrtc, sDate, RTC_FORMAT_BIN);
}

//======================== 6. WRAPPER FUNCTIONS ==============================================

//======================== 6. END ============================================================

//======================== 7. TASKS ============================================================

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


static void WaveProcTask(void *pvParameters) {

    printmsg("SD Card Task Started \r\n");

    SD_Init();

  //Read Test

  float32_t zAcc[1024];


  waveLogNo = 1;
  waveDirNo = 68;
  uint32_t fpointer = 0;


//Wave Read Test
uint32_t rawDataSize = 102400;
uint32_t decimation_number = 25;
uint32_t fileSize = 30000;
uint32_t fileReadSize = 0;
float32_t accumulatedResult[4096] = {0}; // Array to accumulate results
int resultIndex = 0;

for(int i = 0; i < 128; i++) {

    fileReadSize += 1024;

    if (fileReadSize >= 28000) {
        waveLogNo++;
        fileReadSize = 0;
    }

    SD_Wave_Open(&File, &Dir, &fno, waveDirNo, waveLogNo);
    SD_Wave_Read_Fast(&File, zAcc, waveDirNo, waveLogNo, Z_ACC, &fpointer);
    SD_File_Close(&File);

    // LPF Decimate Test
    float32_t decimatedResult[32]; // 1024 / 25 = 40.96, rounded up to 41
    lpf_decimate(zAcc, decimatedResult);

    // Accumulate the result
    for (int j = 0; j < 32 && resultIndex < 4096; j++, resultIndex++) {
        accumulatedResult[resultIndex] = decimatedResult[j];
    }

    //printmsg("Wave Log: %d, Wave Dir: %d, Z-Acc: %.2f, Decimated_Result: %.2f \r\n", waveLogNo, waveDirNo, zAcc[0], decimatedResult[3]);
}

// Print out the accumulated result array
//for (int i = 0; i < 4096; i++) {
//    printmsg("%.2f, \r\n", i, accumulatedResult[i]);
//}

vTaskDelete(NULL);

}

/**
 * @brief Default mode is to put the Cortex-M4 in sleep mode when the RTOS is idle.
 * 
 */
void vApplicationIdleHook(void) {
        // Put the Cortex-M4 into sleep mode
       // __WFI();
    }


//======================== 7. END ============================================================

//======================== 8. FreeRTOS Configuration ============================================================
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

//======================== 8. END ============================================================


