#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include <cstdint>
#include <gmock/gmock.h>
#include "hal_interface.hpp"

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
class MockHAL : public IHAL {
public:
    MOCK_METHOD(void, HAL_Init, (), (override));
    MOCK_METHOD(void, SystemClock_Config, (), (override));
    MOCK_METHOD(void, MX_GPIO_Init, (), (override));
    MOCK_METHOD(void, MX_LPUART1_UART_Init, (), (override));
    MOCK_METHOD(void, HAL_GPIO_TogglePin, (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin), (override));
    MOCK_METHOD(HAL_StatusTypeDef, HAL_UART_Transmit, (UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout), (override));
    MOCK_METHOD(void, HAL_Delay, (uint32_t Delay), (override));
};

// Global pointer to mock instance
extern MockHAL* mockHALInstance;


#endif // MOCK_HAL_H
