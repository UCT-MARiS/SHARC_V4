/**
 * @file hal_impl.cpp
 * @author MichaelNoyce (NYCMIC002@myuct.ac.za)
 * @brief Implementation of the HAL (Hardware Abstraction Layer) interface.
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "hal_impl.hpp"

void HAL_Impl::HAL_Init() {
    ::HAL_Init();
}

void HAL_Impl::SystemClock_Config() {
    ::SystemClock_Config();
}

void HAL_Impl::MX_GPIO_Init() {
    ::MX_GPIO_Init();
}

void HAL_Impl::MX_LPUART1_UART_Init() {
    ::MX_LPUART1_UART_Init();
}

void HAL_Impl::HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    ::HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}

HAL_StatusTypeDef HAL_Impl::HAL_UART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout) {
    return ::HAL_UART_Transmit(huart, pData, Size, Timeout);
}

void HAL_Impl::HAL_Delay(uint32_t Delay) {
    ::HAL_Delay(Delay);
}