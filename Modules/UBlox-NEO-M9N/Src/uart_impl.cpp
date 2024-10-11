#include "uart_interface.hpp"

class UARTImpl : public UARTInterface {
public:
    bool getFlag(UART_HandleTypeDef* huart, uint32_t flag) override {
        return __HAL_UART_GET_FLAG(huart, flag);
    }

    HAL_StatusTypeDef receive(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t size, uint32_t timeout) override {
        return HAL_UART_Receive(huart, pData, size, timeout);
    }
};