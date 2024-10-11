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

/**
 * @brief SDMMC1 Initialization Function
 * 
 */
void MX_SDMMC1_SD_Init(void);

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
void MX_RTC_Init(void);

/**
 * @brief DMA Initialization Function
 * @param None
 * @retval None
 */
void MX_DMA_Init(void);

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
void MX_USART4_UART_Init(void);

#ifdef __cplusplus
}
#endif
