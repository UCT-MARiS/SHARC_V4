#ifndef RTOS_INTERFACE_HPP
#define RTOS_INTERFACE_HPP

#ifdef __cplusplus 
extern "C" {
#endif

 #include "FreeRTOS.h"
 #include "task.h"

#ifdef __cplusplus 
}
#endif


// Virtual class to hold FreeRTOS functions
class IRTOS {
public:
    virtual ~IRTOS() = default;

    virtual TaskHandle_t xTaskCreateStatic(TaskFunction_t pxTaskCode, const char * const pcName, const uint32_t ulStackDepth, void * const pvParameters, UBaseType_t uxPriority, StackType_t * const puxStackBuffer, StaticTask_t * const pxTaskBuffer) = 0;
    virtual void vTaskStartScheduler(void) = 0;
    virtual void vTaskDelete(TaskHandle_t) = 0;
    virtual void vTaskDelay(const TickType_t) = 0;
};

#endif // RTOS_INTERFACE_HPP

