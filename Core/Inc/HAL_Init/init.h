#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

//=============================0. Definitions=====================================

//============================ 1. Function Prototypes ============================

/**
 * @brief Configure system clock
 * 
 */
void SystemClock_Config(void);

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
void MX_GPIO_Init(void);

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
void MX_LPUART1_UART_Init(void);

#ifdef __cplusplus
}
#endif
