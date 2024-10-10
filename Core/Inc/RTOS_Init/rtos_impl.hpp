#ifndef RTOS_IMPL_HPP
#define RTOS_IMPL_HPP

#include "rtos_interface.hpp"
#include "FreeRTOS.h"
#include "task.h"

// Derived class to implement FreeRTOS functions
class FreeRTOS_Impl : public IRTOS {
public:
    ~FreeRTOS_Impl() override = default;
    TaskHandle_t xTaskCreateStatic(TaskFunction_t pxTaskCode, const char * const pcName, const uint32_t ulStackDepth, void * const pvParameters, UBaseType_t uxPriority, StackType_t * const puxStackBuffer, StaticTask_t * const pxTaskBuffer) override;
    void vTaskStartScheduler(void) override;
    void vTaskDelete(TaskHandle_t xTaskToDelete) override;
    void vTaskDelay(const TickType_t xTicksToDelay) override;
};

#endif // RTOS_IMPL_HPP