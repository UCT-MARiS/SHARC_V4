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

void HAL_Impl::HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState){
    ::HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
}

HAL_StatusTypeDef HAL_Impl::HAL_UART_Transmit(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout) {
    return ::HAL_UART_Transmit(huart, pData, Size, Timeout);
}

void HAL_Impl::HAL_Delay(uint32_t Delay) {
    ::HAL_Delay(Delay);
}

void HAL_Impl::MX_SPI2_Init() {
    ::MX_SPI2_Init();
}

HAL_StatusTypeDef HAL_Impl::HAL_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) {
    return ::HAL_SPI_Transmit(hspi, pData, Size, Timeout);
}

HAL_StatusTypeDef HAL_Impl::HAL_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout) {
    return ::HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, Timeout);
}

HAL_StatusTypeDef HAL_Impl::HAL_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) {
    return ::HAL_SPI_Receive(hspi, pData, Size, Timeout);
}