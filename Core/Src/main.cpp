#include "main.hpp"
#include "hal_interface.hpp"
#include "hal_impl.hpp"

#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wformat"

#ifdef __cplusplus 
extern "C" {
#endif
#include "init.h"
#include "string.h"
#include <stdarg.h>
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#ifdef __cplusplus 
}
#endif

//======================== 0. Peripheral Handles ============================================================
UART_HandleTypeDef hlpuart1;
HAL_Impl halImpl;

//======================== 0. END ============================================================================

//======================== 1. Function Prototypes ============================================================
// Function pointers for HAL functions

//======================== 1. END ============================================================================

int main(void) {
    
//======================== 1. SYSTEM INIT & CLOCK CONFIG ========================//
    setupHAL(&halImpl);
	printmsg("SHARC BUOY STARTING! \r\n");
//=================================== 1. END ====================================//

while(1){
	halImpl.HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	halImpl.HAL_Delay(100);
}


}


//Debug Print
void printmsg(char *format,...) {
    char str[80];

    /*Extract the the argument list using VA apis */
    va_list args;
    va_start(args, format);
    vsprintf(str, format,args);
    halImpl.HAL_UART_Transmit(&hlpuart1,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
    va_end(args);
}



