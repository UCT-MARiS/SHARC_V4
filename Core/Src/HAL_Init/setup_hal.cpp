/**
 * @brief Initializes the HAL (Hardware Abstraction Layer).
 * 
 * This function calls the HAL initialization functions to configure the system clock, GPIO, and UART.
 * It is responsible for setting up the necessary hardware resources for the application.
 *
 * @param hal Pointer to the abstract interface for the HAL.
 */

// setupHAL.cpp
#include "main.hpp"
#include "hal_interface.hpp"
#include "hal_impl.hpp"

void setupHAL(IHAL* hal) {
    hal->HAL_Init();
    hal->SystemClock_Config();
    hal->MX_GPIO_Init();
    hal->MX_LPUART1_UART_Init();
    hal->MX_USART2_UART_Init();
}
