# Configures the STM32CubeL4 HAL
# It is expected the https://github.com/STMicroelectronics/STM32CubeL4 is cloned to ${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4.

# Generates static libraries:
# - SHARC::HAL
set(STM32CUBEL4_HAL_INCLUDE_DIRECTORIES
	${CMAKE_SOURCE_DIR}/Core/Inc
	${CMAKE_SOURCE_DIR}/Core/Inc/HAL_Init
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Inc
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/CMSIS/Device/ST/STM32L4xx/Include
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/CMSIS/Include
)

file(GLOB STM32CUBEL4_HAL_SOURCES
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi_ex.c
)

# Workaround - Broken template files should not be compiled.
list(FILTER STM32CUBEL4_HAL_SOURCES EXCLUDE REGEX ".*_template.c")

# HAL Library
add_library(STM32CUBEL4_HAL STATIC
	${STM32CUBEL4_HAL_SOURCES}
)

set(STM32CUBEL4_HAL_COMPILE_DEFINITIONS
	USE_HAL_DRIVER
	STM32L4R5xx
)

target_compile_definitions(STM32CUBEL4_HAL PUBLIC
	${STM32CUBEL4_HAL_COMPILE_DEFINITIONS}
)

target_include_directories(STM32CUBEL4_HAL SYSTEM
	PUBLIC ${STM32CUBEL4_HAL_INCLUDE_DIRECTORIES}
)

add_library(SHARC::HAL ALIAS STM32CUBEL4_HAL)