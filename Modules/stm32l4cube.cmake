# Configures the STM32CubeL4 HAL and FATFS libraries.
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
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/*_hal*.c
	${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/*_ll*.c
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