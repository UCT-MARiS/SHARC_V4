#include "main.hpp"

#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wformat"

#ifdef __cplusplus 
extern "C" {
#endif
#include "init.h"
#include "string.h"
#include <stdarg.h>
#include "stdio.h"
#ifdef __cplusplus 
}
#endif

//======================== 0. Peripheral Handles ============================================================
UART_HandleTypeDef hlpuart1;

//======================== 0. END ============================================================================

//======================== 1. Function Prototypes ============================================================
void printmsg(char *format,...);
//======================== 1. END ============================================================================

int main(void) {
//======================== 1. SYSTEM INIT & CLOCK CONFIG ========================//
	HAL_Init();		//Init Flash prefetch, systick timer, NVIC and LL functions
	SystemClock_Config();	//configure clock
	MX_GPIO_Init();
	MX_LPUART1_UART_Init(); 
	printmsg("SHARC BUOY STARTING! \r\n");
//=================================== 1. END ====================================//

while(1){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	HAL_Delay(100);
}


}


//Debug Print
void printmsg(char *format,...) {
    char str[80];

    /*Extract the the argument list using VA apis */
    va_list args;
    va_start(args, format);
    vsprintf(str, format,args);
    HAL_UART_Transmit(&hlpuart1,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
    va_end(args);
}



