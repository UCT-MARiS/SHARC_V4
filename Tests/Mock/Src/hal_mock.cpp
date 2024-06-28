#include "hal_mock.h"
#include <cstdio>      // Ensure this header is included for printf
#include <gmock/gmock.h>

// Global pointer to mock instance
MockHAL* mockHALInstance = nullptr;

// C++ functions that use the mock instance
void setupHAL_Mock() {
    if (mockHALInstance) {
        mockHALInstance->setupHAL();
    } else {
        printf("setupHAL called\n");
    }
}

void SystemClock_Config_Mock() {
    if (mockHALInstance) {
        mockHALInstance->SystemClock_Config();
    } else {
        printf("SystemClock_Config called\n");
    }
}

void MX_GPIO_Init_Mock() {
    if (mockHALInstance) {
        mockHALInstance->MX_GPIO_Init();
    } else {
        printf("MX_GPIO_Init called\n");
    }
}

void MX_LPUART1_UART_Init_Mock() {
    if (mockHALInstance) {
        mockHALInstance->MX_LPUART1_UART_Init();
    } else {
        printf("MX_LPUART1_UART_Init called\n");
    }
}

void HAL_GPIO_TogglePin_Mock(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    if (mockHALInstance) {
        mockHALInstance->HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
    } else {
        printf("HAL_GPIO_TogglePin called on GPIO Pin: %u\n", GPIO_Pin);
    }
}

HAL_StatusTypeDef HAL_UART_Transmit_Mock(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    if (mockHALInstance) {
        return mockHALInstance->HAL_UART_Transmit(huart, pData, Size, Timeout);
    } else {
        printf("HAL_UART_Transmit called with data: %.*s\n", Size, pData);
        return HAL_OK;
    }
}

void HAL_Delay_Mock(uint32_t Delay) {
    if (mockHALInstance) {
        mockHALInstance->HAL_Delay(Delay);
    } else {
        printf("HAL_Delay called with delay: %u\n", Delay);
    } 
} 

// Extern "C" functions calling the C++ mock functions
extern "C" {

    void setupHAL() {
        setupHAL_Mock();
    }

    void SystemClock_Config(void) {
        SystemClock_Config_Mock();
    }

    void MX_GPIO_Init(void) {
        MX_GPIO_Init_Mock();
    }

    void MX_LPUART1_UART_Init(void) {
        MX_LPUART1_UART_Init_Mock();
    }

    void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
        HAL_GPIO_TogglePin_Mock(GPIOx, GPIO_Pin);
    }

    HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout) {
         return HAL_UART_Transmit_Mock(huart, pData, Size, Timeout);
    }


    void HAL_Delay(uint32_t Delay) {
        HAL_Delay_Mock(Delay);
    } 
}