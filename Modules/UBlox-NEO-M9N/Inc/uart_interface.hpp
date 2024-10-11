#ifndef UART_INTERFACE_HPP
#define UART_INTERFACE_HPP

#include "stm32l4xx_hal_uart.h"

class UARTInterface {
public:
    virtual ~UARTInterface() = default;
    virtual bool getFlag(UART_HandleTypeDef* huart, uint32_t flag) = 0;
    virtual HAL_StatusTypeDef receive(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t size, uint32_t timeout) = 0;
};

#endif // UART_INTERFACE_HPP