/**
 * @file hal_interface.hpp
 * @author Michael Noyce 
 * @brief Virtual interface for HAL functions
 * @version 1.0
 * @date 2024-09-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef HAL_INTERFACE_HPP
#define HAL_INTERFACE_HPP


#ifdef __cplusplus 
extern "C" {
#endif

 #include "stm32l4xx_hal.h"

#ifdef __cplusplus 
}
#endif


#ifdef __cplusplus
class IHAL {
public:
    virtual ~IHAL() = default;

    virtual void HAL_Init() = 0;
    virtual void SystemClock_Config() = 0;
    virtual void MX_GPIO_Init() = 0;
    virtual void MX_LPUART1_UART_Init() = 0;
    virtual void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) = 0;
    virtual HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout) = 0;
    virtual void HAL_Delay(uint32_t Delay) = 0;
    virtual void MX_SPI1_Init() = 0;
    virtual void MX_SPI2_Init() = 0;
    virtual HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) = 0;
};
#endif // __cplusplus

#endif // HAL_INTERFACE_HPP
