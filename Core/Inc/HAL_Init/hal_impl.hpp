/**
 * @file hal_impl.hpp
 * @author Michael Noyce 
 * @brief Actual implementation of the HAL functions
 * @version 1.0
 * @date 2024-09-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef HAL_IMPL_HPP
#define HAL_IMPL_HPP

#include "hal_interface.hpp"
#include "main.hpp"

extern "C" {
    #include "init.h"
}

class HAL_Impl : public IHAL {
public:
    void HAL_Init() override;
    void SystemClock_Config() override;
    void MX_GPIO_Init() override;
    void MX_LPUART1_UART_Init() override;
    void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) override;
    HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout) override;
    void HAL_Delay(uint32_t Delay) override;
    void MX_SPI1_Init() override;
    void MX_SPI2_Init() override;
    HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) override; 

};

#endif // HAL_IMPL_HPP
