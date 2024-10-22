#include "main.hpp"
#include "hal_interface.hpp"
#include "hal_impl.hpp"

//GNSS Includes
#include "u-blox_m9n_Library.h"

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

//GNSS Handles
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
SFE_UBLOX_GNSS myGNSS;

//======================== 0. END ============================================================================

//======================== 1. Function Prototypes ============================================================

//Tasks
static void LED_task(void *args); 
static void updateRTC(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate);
static void GNSSWaveProcTask(void *pvParameters);
static void GNSSLogTask(void *pvParameters);

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

    begin_UART_DMA(&huart4); // Start the UART DMA
    myGNSS.circular_buffer_init(&myGNSS.RX_Buffer);

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

    sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
    sDate.Month = RTC_MONTH_OCTOBER;
    sDate.Date = 0x0; 
    sDate.Year = 0x0;

    updateRTC(&hrtc, &sTime, &sDate);
    printmsg("RTC Time: %02d:%02d:%02d \r\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
    printmsg("RTC Date: %02d/%02d/%02d \r\n", sDate.Date, sDate.Month, sDate.Year);


    // GNSS Initialization
    #define DELAY_1000_MS (1000 / portTICK_PERIOD_MS)
    #define DELAY_200_MS (200 / portTICK_PERIOD_MS)
    #define DELAY_10_MS (10 / portTICK_PERIOD_MS)

//=================================== 3. END ====================================//

//======================== 4. TASK CREATION ============================================================

    // Create a blinking LED task for the on-board LED.
    static StaticTask_t exampleTaskTCB;
    static StackType_t exampleTaskStack[ 512 ];

    // Create the blink task
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;  // 500 ms delay

    TaskHandle_t returnStatus_LED = xTaskCreateStatic( LED_task,
                                  "Blink_LED",
                                  configMINIMAL_STACK_SIZE,
                                  (void*)xDelay,
                                  configMAX_PRIORITIES-4U,
                                  &( exampleTaskStack[ 0 ] ),
                                  &( exampleTaskTCB ) );

    // Check if the tasks were created successfully
    if (returnStatus_LED == NULL) {
    // Task creation failed
        printmsg("LED Task Creation Failed\r\n");
    }
    else {
        printmsg("LED Task Created Successfully\r\n");
    }

/*     // Create a task to log vertical velocity from the GNSS.
    static StaticTask_t GNSSLogTaskTCB;
    static StackType_t GNSSLogTaskStack[ 8192 ];

    TaskHandle_t GNSSLogTaskHandle = xTaskCreateStatic(
        GNSSLogTask,          // Function that implements the task.
        "GNSSLogTask",        // Text name for the task.
        8192,                 // Stack size in words, not bytes.
        NULL,                // Parameter passed into the task.
        configMAX_PRIORITIES - 1U, // Priority at which the task is created.
        GNSSLogTaskStack,     // Array to use as the task's stack.
        &GNSSLogTaskTCB       // Variable to hold the task's data structure.
    );

    if (GNSSLogTaskHandle == NULL) {
        printmsg("Failed to create SDCardTask\r\n");
    } */

    // Create a task to check SD card functions.
    static StaticTask_t waveProcTaskTCB;
    static StackType_t waveProcTaskStack[ 24000 ];

    TaskHandle_t GNSSWaveProcTaskHandle = xTaskCreateStatic(
        GNSSWaveProcTask,          // Function that implements the task.
        "GNSSWaveProcTask",        // Text name for the task.
        24000,                 // Stack size in words, not bytes.
        NULL,                // Parameter passed into the task.
        configMAX_PRIORITIES - 1U, // Priority at which the task is created.
        waveProcTaskStack,     // Array to use as the task's stack.
        &waveProcTaskTCB       // Variable to hold the task's data structure.
    );

    if (GNSSWaveProcTaskHandle == NULL) {
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

void begin_UART_DMA(UART_HandleTypeDef *huart)
{
    // Start the UART receive DMA
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t*)&myGNSS.receivedBytes, sizeof(myGNSS.receivedBytes));
    __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT); // Disable half-transfer interrupt
    if (status != HAL_OK)
    {
        printmsg("DMA Error: %d \r\n", status);
    }
}

// UART receive event callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == myGNSS._serialPort->Instance)
    {
        // Read the received byte and store it in the circular buffer
        myGNSS.circular_buffer_write(&myGNSS.RX_Buffer, myGNSS.receivedBytes, Size);
        // Set the flag to indicate data has been received
        myGNSS.dataReceived = true;

        // Restart the DMA transfer
        HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t*)&myGNSS.receivedBytes, sizeof(myGNSS.receivedBytes));
        __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT); // Disable half-transfer interrupt
    }
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

static void GNSSLogTask(void *pvParameters) {

    printmsg("GNSS Task Started \r\n");

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    //Temporary variables related to IMU sampling
	int32_t velTemp[3]; // Optionally, all NED coordinates can be stored here
	uint8_t tempFIFOBuf[500];
    
	int GNSS_On = 1;

    waveDirNo = 0;
    waveLogNo = 0;

    uint32_t gnss_sample_count = 0;

    SD_Init();

	SD_Wave_Open(&File, &Dir, &fno, waveDirNo, waveLogNo);
    //myGNSS.enableDebugging(&hlpuart1);
	if (myGNSS.begin(&huart4) == true) {
        printmsg("GNSS serial connected \r\n");

        while(myGNSS.getFixType() == 0)// Wait for GNSS fix
        {
            vTaskDelay(DELAY_1000_MS);
        }

        xLastWakeTime = xTaskGetTickCount();

		while (GNSS_On) {
			if (gnss_sample_count == WAVELOGBUFNO)
			{
				SD_File_Close(&File); //NB Remember to close file
				waveLogNo++;
				gnss_sample_count = 0;
				SD_Wave_Open(&File, &Dir, &fno, waveDirNo, waveLogNo);
				printmsg("New Wave log! \r\n");
			}

            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

            HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); //unlocks time stamp

            velTemp[0] = myGNSS.getNedDownVel(); //Only vertical velocity stored

            printmsg("%d \r\n", velTemp[0]);

            sprintf((char*) tempFIFOBuf,
            "%02d/%02d/%02d %02d:%02d:%02d, %ld \r\n",
            sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds,
            velTemp[0]);
            SD_File_Write(&File, tempFIFOBuf);

            gnss_sample_count++;

            if (waveLogNo == WAVELOGNO + 1) {
				SD_File_Close(&File); //NB Remember to close file
				waveDirNo++;
				waveLogNo = 0;
			}

            if(waveDirNo == 1000)
		    {
			  SD_File_Close(&File);
              printmsg("1000 directories reached! \r\n");
			  break;
		    }
		}

	}
        else {
        printmsg("Unable to connect \r\n");
    }
}

static void GNSSWaveProcTask(void *pvParameters) {

    printmsg("SD Card Task Started \r\n");

    // Initialize the SD card
    SD_Init();

    // Result arrays
    float32_t wave_params[5];          // Wave parameters
    float32_t moments[5];              // Spectral moments

    // Variables for processing
    int32_t zVel[281];               // Input signal from SD card
    float32_t accumulatedResult[1124];  // Accumulated results for all iterations
    memset(accumulatedResult, 0, sizeof(accumulatedResult));  // Initialize to zero

    // Index variables
    int resultIndex = 0;
    uint32_t fpointer = 0;
    uint32_t waveLogNo = 1;
    uint32_t waveDirNo = 0;

    for(int i = 0; i < 4; i++) {
        fpointer = 0;
        SD_Wave_Open(&File, &Dir, &fno, waveDirNo, waveLogNo);
        SD_GNSS_Wave_Read_Fast(&File, zVel, waveDirNo, waveLogNo, Z_VEL, &fpointer); // X_ACC for IMU corresponds to Z_VEL for GNSS
        SD_File_Close(&File);

        // Accumulate the result, skipping discarded samples
        for (int j = 0; j < 281 && resultIndex < 1124; j++, resultIndex++) {
            //printmsg("%d \r\n", zVel[j]);
            accumulatedResult[resultIndex] = zVel[j]/1000.0f;  // Convert to m/s
        }

        waveLogNo++;
    }

    // Process the accumulated result
        // Apply Bandpass filter
    float32_t filteredSignal[SIGNAL_LENGTH];
    memset(filteredSignal, 0, sizeof(filteredSignal));  // Initialize to zero

    uint32_t i;
    arm_fir_instance_f32 S;
    arm_status status;
    float32_t  *inputF32, *outputF32;

    // Private variables for FIR 
    static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

    // FIR filter coefficients
    const float32_t firCoeffs32[NUM_TAPS] = {
        0.0003f,  0.0017f,  0.0000f,  0.0018f,  0.0012f,  0.0004f,  0.0034f, -0.0007f,  0.0033f,  0.0014f, -0.0015f,  0.0053f, -0.0063f,  0.0025f, -0.0032f,
    -0.0108f,  0.0034f, -0.0225f, -0.0040f, -0.0162f, -0.0315f, -0.0011f, -0.0529f, -0.0122f, -0.0344f, -0.0641f,  0.0103f, -0.1082f,  0.0058f, -0.0481f,
    -0.1736f,  0.5560f,  0.5560f, -0.1736f, -0.0481f,  0.0058f, -0.1082f,  0.0103f, -0.0641f, -0.0344f, -0.0122f, -0.0529f, -0.0011f, -0.0315f, -0.0162f,
    -0.0040f, -0.0225f,  0.0034f, -0.0108f, -0.0032f,  0.0025f, -0.0063f,  0.0053f, -0.0015f,  0.0014f,  0.0033f, -0.0007f,  0.0034f,  0.0004f,  0.0012f,
        0.0018f,  0.0000f,  0.0017f,  0.0003f
    };

    // Global variables for FIR
    uint32_t blockSize = BLOCK_SIZE;
    uint32_t numBlocks = SIGNAL_LENGTH/BLOCK_SIZE;

    /* Initialize input and output buffer pointers */
    inputF32 = &accumulatedResult[0];
    outputF32 = &filteredSignal[0];

    /* Call FIR init function to initialize the instance structure. */
    arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

    /* ----------------------------------------------------------------------
    ** Call the FIR process function for every blockSize samples
    ** ------------------------------------------------------------------- */

    for(i=0; i < numBlocks; i++)
    {
        arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
    }

    // Remove the first 100 samples
    float32_t truncated_signal[TRUNCATED_LENGTH];
    for (uint32_t i = 0; i < TRUNCATED_LENGTH; i++) {
        truncated_signal[i] = filteredSignal[i + 100];
    }
  
    // Output buffer
    float32_t vPSD[FFT_SIZE/2];
    memset(vPSD, 0, sizeof(vPSD));  // Initialize to zero

    // Welch PSD estimate
    pwelch(truncated_signal, SIGNAL_LENGTH, vPSD);

    float32_t PSD[FFT_SIZE/2];

    // Multiply the PSD by |(jw)^2| (i.e. double integral in frequency domain)
    HPFDoubleIntegration(vPSD, PSD);

    // Compute spectral moments
    compute_spectral_moments((float*)PSD, FFT_SIZE/2, moments);

    // Calculate wave parameters
    calculate_wave_parameters(moments, wave_params);

    printmsg("Significant Wave Height (Hs): %f meters\n", wave_params[0]);
    printmsg("Mean zero crossing period (Tz): %f seconds\n", wave_params[4]);
    printmsg("Mean Wave Period (Tm01): %f seconds\n", wave_params[2]);
    printmsg("Peak Period (Tp): %f seconds\n", wave_params[1]);

    // Print out the PSD array
    for (int i = 0; i < 128; i++) {
        printmsg("%d, %.4f, \r\n", i, PSD[i]);
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