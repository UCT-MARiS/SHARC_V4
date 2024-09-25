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
    void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) override;
    HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout) override;
    void HAL_Delay(uint32_t Delay) override;
    void MX_SPI2_Init() override;
    HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) override;
    HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout) override;
    HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) override;
};

#endif // HAL_IMPL_HPP
