#include "rtos_impl.hpp"

TaskHandle_t FreeRTOS_Impl::xTaskCreateStatic(TaskFunction_t pxTaskCode, const char * const pcName, const uint32_t ulStackDepth, void * const pvParameters, UBaseType_t uxPriority, StackType_t * const puxStackBuffer, StaticTask_t * const pxTaskBuffer) {
    return ::xTaskCreateStatic(pxTaskCode, pcName, ulStackDepth, pvParameters, uxPriority, puxStackBuffer, pxTaskBuffer);
}

void FreeRTOS_Impl::vTaskStartScheduler(void) {
    ::vTaskStartScheduler();
}

void FreeRTOS_Impl::vTaskDelete(TaskHandle_t xTaskToDelete) {
    ::vTaskDelete(xTaskToDelete);
}

void FreeRTOS_Impl::vTaskDelay(const TickType_t xTicksToDelay) {
    ::vTaskDelay(xTicksToDelay);
}
