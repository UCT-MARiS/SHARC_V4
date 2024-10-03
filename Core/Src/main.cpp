#include "main.hpp"
#include "hal_interface.hpp"
#include "hal_impl.hpp"
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
#ifdef __cplusplus 
}
#endif

//======================== 0. Peripheral Handles ========================================================================================
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
HAL_Impl halImpl;

SFE_UBLOX_GNSS myGNSS;
//======================== 0. END ============================================================================================================= 

//======================== 1. Function Prototypes ========================================================================================
static void LED_task(void *args); 
void GPS_task(void *args);
//======================== 1. END =======================================================================================================

int main(void){

//======================== 1. SYSTEM INIT & CLOCK CONFIG ======================================================================================
    HAL_Init();
    SystemClock_Config();
    setupHAL(&halImpl);

    // Initialize the circular buffer
    myGNSS.circular_buffer_init(&myGNSS.RX_Buffer);

    // Begin DMA Transfer
    begin_UART_DMA(&huart2);

	//printmsg("SHARC BUOY STARTING! \r\n");
//=================================== 1. END ============================================================================================

//=================================== 2. RTOS Tasks ======================================================================================

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

    // Create a GPS task
    TaskHandle_t GPS_task_handle;
    BaseType_t returnStatus_GPS = xTaskCreate( GPS_task,
                                  "GPS_Task",
                                  ((uint16_t)256),
                                  NULL,
                                  configMAX_PRIORITIES-1U,
                                  &(GPS_task_handle) ); 

    // Check if the tasks were created successfully
    if (returnStatus_LED == NULL) {
    // Task creation failed
        printmsg("LED Task Creation Failed\r\n");
    }
    else {
        printmsg("LED Task Created Successfully\r\n");
    }

    if (returnStatus_GPS != pdPASS) {
        printmsg("GPS Task Creation Failed \r\n");
    }
    else{
        printmsg("GPS Task Created Successfully \r\n");
    } 
/* 
    // FFT operation using CMSIS DSP library
    const uint32_t fftSize = FFT_SIZE;

    // Input signal (example: sine wave)
    float32_t inputSignal[fftSize];
    float32_t fs1 = 0.1f;
    float32_t fs2 = 0.25;

    for (uint32_t i = 0; i < fftSize; i++) {
        inputSignal[i] = 1*arm_sin_f32(2 * PI * i * fs2 / F_SAMPLE);
        //printmsg("%d,%f\r\n", i, inputSignal[i]);
    }

    // Output buffer
    float32_t outputSignal[PSD_SIZE];
    arm_fill_f32(0.0f, outputSignal, PSD_SIZE);

    // Welch PSD estimate
    pwelch(inputSignal, fftSize, outputSignal);

    // Multiply the PSD by |(jw)^2| (i.e. double integral in frequency domain)
    doubleIntegration(outputSignal, outputSignal, PSD_SIZE);

    Wave_Data_t waveData;
    waveParamExtract(outputSignal, &waveData.Hm0, &waveData.TM01, &waveData.Hrms, &waveData.T0, &waveData.Tp, &waveData.M0, &waveData.M1, &waveData.M2);

    // Print FFT result
     for (uint32_t i = 0; i < PSD_SIZE; i++) {
        //printmsg("%d,%f \r\n", i, outputSignal[i]);
    }

    printmsg("Hm0 %.8f \r\n", waveData.Hm0);
	printmsg("Hrms %.8f \r\n", waveData.Hrms);
	printmsg("M0 %.8f \r\n", waveData.M0);
	printmsg("M1 %.8f \r\n", waveData.M1);
	printmsg("M2 %.8f \r\n", waveData.M2);
	printmsg("Tm01 %.8f \r\n", waveData.TM01);
	printmsg("T0 %.8f \r\n", waveData.T0);
	printmsg("Tp %.8f \r\n", waveData.Tp); */
    //myGNSS.enableDebugging(&hlpuart1, true); // Enable debug messages over LPUART1 
    vTaskStartScheduler();
/*     if (myGNSS.begin(&huart2) == true)
    {
        uint32_t firstEpoch = myGNSS.getUnixEpoch(); // Get the current time in Unix Epoch format
        printmsg("Epoch: %d\r\n", firstEpoch);
        printmsg("GNSS serial connected \r\n");
        myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
        myGNSS.saveConfiguration(); //Save the current settings to flash and BBR;   

        // Start scheduler 
        vTaskStartScheduler();
        
    }
    else
        printmsg("Unable to connect \r\n");  */

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

void begin_UART_DMA(UART_HandleTypeDef *huart)
{
    // Start the UART receive DMA
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t*)&myGNSS.receivedBytes, sizeof(myGNSS.receivedBytes));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // Disable half-transfer interrupt
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
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // Disable half-transfer interrupt
    }
}

#define DELAY_1000_MS (1000 / portTICK_PERIOD_MS)
#define DELAY_200_MS (200 / portTICK_PERIOD_MS)
#define DELAY_10_MS (10 / portTICK_PERIOD_MS)
uint32_t microseconds;

void GPS_task(void *pvParameters){
    uint32_t dataIndex = 0;
    TickType_t xLastWakeTime;

    if (myGNSS.begin(&huart2) == true){
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

    myGNSS.setNavigationFrequency(1); // Set the navigation rate to 5 Hz

    printmsg("Navigation Frequency: 1 Hz \r\n");

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
        float32_t nedDownVel = myGNSS.getNedDownVel(); //Returns the NED Down in mm/s
        nedDownVel = nedDownVel/1000;
        printmsg("%d,%f\r\n",dataIndex, nedDownVel);
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

static void LED_task(void *pvParameters) {

    TickType_t pxDelay = (TickType_t)pvParameters;

    TickType_t xLastWakeTime;

    const TickType_t xDelay500ms = pdMS_TO_TICKS( 500 );
    
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
        vTaskDelayUntil(&xLastWakeTime, pxDelay);
    }
}

/**
 * @brief Default mode is to put the Cortex-M4 in sleep mode when the RTOS is idle.
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
        printmsg("Stack Overflow in Task %s\r\n", pcTaskName);
        ( void ) xTask;
        ( void ) pcTaskName;
    }

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */