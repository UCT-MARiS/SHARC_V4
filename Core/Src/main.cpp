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

//ICM42688P Includes
#include "ICM42688P.h"

#ifdef __cplusplus 
}
#endif

//======================== 0. Peripheral Handles ============================================================
UART_HandleTypeDef hlpuart1;
HAL_Impl halImpl;
SPI_HandleTypeDef hspi2; // Ensure this is correctly defined and initialized
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
//======================== 0. END ============================================================================

//======================== 1. Function Prototypes ============================================================

//Tasks
static void LED_task(void *args); 

//Debugging
void printmsg(char *format,...);

//Wrapper functions for external libraries
int8_t spi_read(uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr);
int8_t spi_write(uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr);
void gpio_write_nss_pin(uint8_t state);
int8_t spi_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t len, void *intf_ptr);

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

    // IMU Initialization
    icm42688p_dev_t dev;
    dev.read = spi_read;
    dev.write = spi_write;
    dev.delay_ms = HAL_Delay; // can only be used before the RTOS scheduler is started    
    dev.gpio_write_nss_pin = gpio_write_nss_pin;
    dev.transmit_receive = spi_transmit_receive;
    dev.intf_ptr = &hspi2;

    int retry_count = 0;
    const int max_retries = 3;
    int8_t status;

    while (retry_count < max_retries) {
        status = icm42688p_init(&dev);
        if (status == ICM42688P_OK) {
            printmsg("ICM42688P initialized successfully \r\n");
            break;
        } else {
            printmsg("Failed to initialize ICM42688P, retrying... (%d/%d) \r\n", retry_count + 1, max_retries);
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second before retrying
        }
    }

    if (status != ICM42688P_OK) {
        printmsg("Failed to initialize ICM42688P after %d attempts \r\n", max_retries);
    }



//=================================== 3. END ====================================//
    // int16_t accel_data[3];
    // while(1)
    // {
    //     // Read accelerometer data
    //     if(icm42688p_read_accel_data(&dev, accel_data) != ICM42688P_OK) {
    //         printmsg("Failed to read accelerometer data \r\n");
    //     }
    //     else{}
    //         HAL_Delay(1000);
    //         printmsg("Accel Data: %d, %d, %d \r\n", accel_data[0], accel_data[1], accel_data[2]);
    // }


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
                                  configMAX_PRIORITIES - 1U,
                                  &( exampleTaskStack[ 0 ] ),
                                  &( exampleTaskTCB ) );

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

//======================== 6. WRAPPER FUNCTIONS ==============================================
/* These functions allow external libraries to interface with the HAL */

/* Wrapper functions for SPI communication with the ICM42688P */

/**
 * @brief spi_read
 * 
 * @param reg_addr 
 * @param data 
 * @param len 
 * @param intf_ptr 
 * @return int8_t 
 */
int8_t spi_read(uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)intf_ptr;
    HAL_StatusTypeDef status;

    // Send the register address
    status = HAL_SPI_Transmit(hspi, &reg_addr, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return -1; // Return an error code
    }

    // Receive the data
    status = HAL_SPI_Receive(hspi, data, len, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return -1; // Return an error code
    }

    return 0; // Success
}

/**
 * @brief spi_write
 * 
 * @param reg_addr 
 * @param data 
 * @param len 
 * @param intf_ptr 
 * @return int8_t 
 */
int8_t spi_write(uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)intf_ptr;
    HAL_StatusTypeDef status;

    // Send the register address
    status = HAL_SPI_Transmit(hspi, &reg_addr, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return -1; // Return an error code
    }

    // Send the data
    status = HAL_SPI_Transmit(hspi, (uint8_t *)data, len, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return -1; // Return an error code
    }

    return 0; // Success
}

/**
 * @brief Wrapper to write to the NSS pin of the ICM42688P
 * 
 * @param state 
 */
void gpio_write_nss_pin(uint8_t state) 
{
    halImpl.HAL_GPIO_WritePin(IMU2_CS_GPIO_Port, IMU2_CS_Pin, (GPIO_PinState) state);
}

int8_t spi_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t len, void *intf_ptr) 
{
    // Check for null pointers
    if (tx_data == NULL || rx_data == NULL || intf_ptr == NULL) {
        printmsg("Error: Null pointer passed to spi_transmit_receive\r\n");
        return HAL_ERROR; // Return an error code
    }

    // Check for valid length
    if (len == 0) {
        printmsg("Error: Length is zero in spi_transmit_receive\r\n");
        return HAL_ERROR; // Return an error code
    }

    // Perform the SPI transmit/receive operation
    uint8_t status = halImpl.HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, len, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Debugging message
        printmsg("HAL_SPI_TransmitReceive failed with status: %d\r\n", status);
        return HAL_ERROR; // Return an error code if the operation fails
    }

    return HAL_OK; // Return success
}


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

/**
 * @brief Default mode is to put the Cortex-M4 in sleep mode when the RTOS is idle.
 * 
 */
void vApplicationIdleHook(void) {
        // Put the Cortex-M4 into sleep mode
        __WFI();
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


