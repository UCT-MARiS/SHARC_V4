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
static void GNSSLogTask(void *pvParameters);
void GPS_task(void *pvParameters);
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

    sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
    sDate.Month = RTC_MONTH_OCTOBER;
    sDate.Date = 0xA; 
    sDate.Year = 0xA; // 10/10/2024

    updateRTC(&hrtc, &sTime, &sDate);
    printmsg("RTC Time: %02d:%02d:%02d \r\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
    printmsg("RTC Date: %02d/%02d/%02d \r\n", sDate.Date, sDate.Month, sDate.Year);


    // GNSS Initialization
    #define DELAY_1000_MS (1000 / portTICK_PERIOD_MS)
    #define DELAY_200_MS (200 / portTICK_PERIOD_MS)
    #define DELAY_10_MS (10 / portTICK_PERIOD_MS)
    uint32_t microseconds;



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

/*     // Create a GPS task
    TaskHandle_t GPS_task_handle;
    BaseType_t returnStatus_GPS = xTaskCreate( GPS_task,
                                  "GPS_Task",
                                  ((uint16_t)256),
                                  NULL,
                                  configMAX_PRIORITIES-1U,
                                  &(GPS_task_handle) ); 


    if (returnStatus_GPS != pdPASS) {
        printmsg("GPS Task Creation Failed \r\n");
    }
    else{
        printmsg("GPS Task Created Successfully \r\n");
    }  */

       // Create a task to check SD card functions.
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
	int32_t velTemp[3];
	int16_t vDOPTemp;
    uint8_t svTemp;
	uint8_t tempFIFOBuf[500];
	uint8_t gnss[12] = { 0 };
	uint8_t data_status;
    
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

            //HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

			//HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); //unlocks time stamp
            HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

            HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); //unlocks time stamp
            
/*             velTemp[0] = myGNSS.getNedNorthVel();
            #velTemp[1] = myGNSS.getNedEastVel();
            velTemp[2] = myGNSS.getNedDownVel();

            vDOPTemp = myGNSS.getVerticalDOP();

            svTemp = myGNSS.getSIV();

            printmsg("vN      vE      vD      vDOP      SIV \r\n");
            printmsg("%d    %d      %d      %d      %d \r\n", velTemp[0], velTemp[1], velTemp[2], vDOPTemp, svTemp);

            sprintf((char*) tempFIFOBuf,
            "%02d/%02d/%02d %02d:%02d:%02d, %ld, %ld, %ld, %ld %ld \r\n",
            sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds,
            velTemp[0], velTemp[1], velTemp[2], vDOPTemp, svTemp);
            SD_File_Write(&File, tempFIFOBuf); */

            velTemp[0] = myGNSS.getNedDownVel();

            //vDOPTemp = myGNSS.getVerticalDOP();

            //svTemp = myGNSS.getSIV();

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
              printmsg("Donezo! \r\n");
			  break;
		    }
		}

	}
        else {
        printmsg("Unable to connect \r\n");
    }
}

void GPS_task(void *pvParameters){
    uint32_t dataIndex = 0;
    TickType_t xLastWakeTime;
    RTC_TimeTypeDef sTime;
    //myGNSS.enableDebugging(&hlpuart1);
    if (myGNSS.begin(&huart4) == true){
        printmsg("GNSS serial connected \r\n");
    }
    else {
        printmsg("Unable to connect \r\n"); 
        vTaskSuspend(NULL);
    }

// Configure the GNSS receiver
{
    myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only

    while(!myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS)){}
    printmsg("GPS Enabled \r\n");

    while(!myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS)){}
    printmsg("SBAS Disabled \r\n");

    while(!myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS)){}
    printmsg("GLONASS Disabled \r\n");

    while(!myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO)){}
    printmsg("Galileo Disabled \r\n");
    
    while(!myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU)){}
    printmsg("Beidou Disabled \r\n");

    while(!myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_IMES)){}
    printmsg("IMES Disabled \r\n");

    while(!myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS)){}
    printmsg("QZSS Disabled \r\n");

    myGNSS.saveConfiguration(); //Save the current settings to flash and BBR;

    #define DELAY_5_S (5000 / portTICK_PERIOD_MS)
    vTaskDelay(DELAY_5_S);

    if (myGNSS.getDiffSoln()){
        printmsg("Differential Corrections Applied \r\n");
    }
    else{
        printmsg("No Differential Corrections Applied \r\n");
    }

    uint16_t VDOP = myGNSS.getVerticalDOP(); // Vertical Dilution of Precision (Scaled by 100)
    uint16_t NumSat = myGNSS.getSIV(); // Number of Sattelites Used in Fix
    uint16_t fixType = myGNSS.getFixType(); // Get the current fix type

    printmsg("VDOP: %.3f \r\n", VDOP/100.0f);
    printmsg("Number of Satellites: %d \r\n", NumSat);
    switch (fixType) {
    case 0:
        printmsg("Fix Type: No Fix (0)\r\n");
        break;
    case 1:
        printmsg("Fix Type: Dead Reckoning only (1)\r\n");
        break;
    case 2:
        printmsg("Fix Type: 2D-Fix (2)\r\n");
        break;
    case 3:
        printmsg("Fix Type: 3D-Fix (3)\r\n");
        break;
    case 4:
        printmsg("Fix Type: GNSS + Dead Reckoning combined (4)\r\n");
        break;
    case 5:
        printmsg("Fix Type: Time only fix (5)\r\n");
        break;
    default:
        printmsg("Fix Type: Unknown (%d)\r\n", fixType);
        break;
    }

    myGNSS.setNavigationFrequency(2); // Set the navigation rate to 5 Hz

    printmsg("Navigation Frequency: 2 Hz \r\n");

    printmsg("Antenna Type: Patch Antenna \r\n");
}
    /*
    * The xLastWakeTime variable needs to be initialized with the current tick
    * count. Note that this is the only time the variable is explicitly
    * written to. After this xLastWakeTime is managed automatically by the
    * vTaskDelayUntil() API function.
    */
    xLastWakeTime = xTaskGetTickCount();

    for (;;){
        //vTaskDelay(DELAY_1000_MS);
         vTaskDelayUntil(&xLastWakeTime, DELAY_1000_MS);
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        float32_t nedDownVel = myGNSS.getNedDownVel(); //Returns the NED Down in mm/s
        nedDownVel = nedDownVel/1000;
        printmsg("%d,%f\r\n",sTime.Seconds, nedDownVel);
        dataIndex++;

        if (dataIndex == 281){

            uint16_t VDOP = myGNSS.getVerticalDOP(); // Vertical Dilution of Precision (Scaled by 100)
            uint16_t NumSat = myGNSS.getSIV(); // Number of Sattelites Used in Fix
            uint16_t fixType = myGNSS.getFixType(); // Get the current fix type

            printmsg("VDOP: %.3f \r\n", VDOP/100.0f);
            printmsg("Number of Satellites: %d \r\n", NumSat);
            switch (fixType) {
            case 0:
                printmsg("Fix Type: No Fix (0)\r\n");
                break;
            case 1:
                printmsg("Fix Type: Dead Reckoning only (1)\r\n");
                break;
            case 2:
                printmsg("Fix Type: 2D-Fix (2)\r\n");
                break;
            case 3:
                printmsg("Fix Type: 3D-Fix (3)\r\n");
                break;
            case 4:
                printmsg("Fix Type: GNSS + Dead Reckoning combined (4)\r\n");
                break;
            case 5:
                printmsg("Fix Type: Time only fix (5)\r\n");
                break;
            default:
                printmsg("Fix Type: Unknown (%d)\r\n", fixType);
                break;
            }

            vTaskEndScheduler();
        }
    }
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


