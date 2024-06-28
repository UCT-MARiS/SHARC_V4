#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include <cstdint>
#include <gmock/gmock.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_uart.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4r5xx.h"
#include "core_cm4.h"  

#ifdef __cplusplus
}
#endif


// Mock class definition
class MockHAL {
public:
    MOCK_METHOD(void, setupHAL, (), ());
    MOCK_METHOD(void, HAL_Init, (), ());
    MOCK_METHOD(void, SystemClock_Config, (), ());
    MOCK_METHOD(void, MX_GPIO_Init, (), ());
    MOCK_METHOD(void, MX_LPUART1_UART_Init, (), ());
    MOCK_METHOD(void, HAL_GPIO_TogglePin, (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin), ());
    MOCK_METHOD(HAL_StatusTypeDef, HAL_UART_Transmit, (UART_HandleTypeDef*, const uint8_t*, uint16_t, uint32_t), ());
    MOCK_METHOD(void, HAL_Delay, (uint32_t Delay), ());
};


#endif // MOCK_HAL_H
