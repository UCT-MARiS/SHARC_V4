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
    static StackType_t waveProcTaskStack[ 24000 ];

    TaskHandle_t WaveProcTaskHandle = xTaskCreateStatic(
        WaveProcTask,          // Function that implements the task.
        "WaveProcTask",        // Text name for the task.
        24000,                 // Stack size in words, not bytes.
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

    // Initialize the SD card
    SD_Init();

    // Result arrays
    float32_t psd[512];                // Power spectral density  
    float32_t moments[5];              // Spectral moments
    float32_t wave_params[5];          // Wave parameters

    // Variables for processing
    float32_t zAcc[1024];               // Input signal from SD card
    float32_t accumulatedResult[4096];  // Accumulated results for all iterations
    float32_t decimatedResult[32];      // Decimated result for each iteration 1024 / 32 = 32
    memset(accumulatedResult, 0, sizeof(accumulatedResult));  // Initialize to zero

    // Index variables
    int resultIndex = 0;
    uint32_t fpointer = 0;
    uint32_t waveLogNo = 0;
    uint32_t waveDirNo = 68;

    // Define and initialize the FIR decimator instance
    static arm_fir_decimate_instance_f32 S;

    // Clear the state buffer before initialization
    memset(firStateF32, 0, sizeof(firStateF32));
    arm_fir_decimate_init_f32(&S, NUM_TAPS, DECIMATION_CONSTANT, firCoeffs32, firStateF32, BLOCK_SIZE);

    // Check the sum of the FIR coefficients
    /*float32_t sum = 0.0f;
    for(int i = 0; i < NUM_TAPS_ARRAY_SIZE; i++) {
        sum += firCoeffs32[i];
    }
    printmsg("Sum of FIR coefficients: %.6f\n", sum);*/

// 128 ITERATIONS * (1024 SAMPLES / 32 DECIMATION CONSTANT) = 4096 WAVE SAMPLE SIZE
    for(int i = 0; i < 128; i++) {

        SD_Wave_Open(&File, &Dir, &fno, waveDirNo, waveLogNo);
        SD_Wave_Read_Fast(&File, zAcc, waveDirNo, waveLogNo, Z_ACC, &fpointer);
        SD_File_Close(&File);

        // Clip the acceleration signal
        // base value 8192 at rest (1g)
        // 600/8192*9.81 = 0.715 m/s^2 
        // H = Az_max / (4*pi^2*f^2) = 0.6 / (4*pi^2*0.2^2) = 9.5 m
        arm_clip_f32(zAcc, zAcc, 7692.0, 8692.0, FFT_SIZE);

        // Decimate the input signal using the FIR filter 
        lpf_decimate(&S, zAcc, decimatedResult);

        // Calibrate the decimated result
        calibrate(decimatedResult, decimatedResult);

        // Accumulate the result, skipping discarded samples
        for (int j = 0; j < 32 && resultIndex < 4096; j++, resultIndex++) {
            accumulatedResult[resultIndex] += decimatedResult[j];
        }

}

    pwelch(accumulatedResult, INPUT_SIGNAL_SIZE, psd);

    // Remove the DC component
    psd[0] = 0.0f;

    // Remove frequency components below 0.02Hz
    uint32_t idx = 0;
    uint32_t freqIndex = (uint32_t)(0.02 / (SAMPLING_FREQUENCY / (float32_t)FFT_SIZE));
    while (idx < freqIndex) {
        psd[idx] = 0.0f;
        idx++;
    }

    // Integrate the acceleration PSD to obtain the displacement PSD using CMSIS DSP functions with static arrays
    float32_t Freqs[512];
    for (int i = 0; i < 512; i++) {
        Freqs[i] = i * (SAMPLING_FREQUENCY / FFT_SIZE);
    }

    integrate_psd_cmsis_static(psd, Freqs, psd, PSD_SIZE);

    compute_spectral_moments(psd, PSD_SIZE, moments);

    calculate_wave_parameters(moments, wave_params);

    // Print out the accumulated result array
    for (int i = 0; i < 4096; i++) {
        printmsg("%.2f, \r\n", i, accumulatedResult[i]);
        vTaskDelay(2);
    }

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


