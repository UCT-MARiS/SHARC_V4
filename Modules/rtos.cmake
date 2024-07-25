# Configures FreeRTOS Kernel for STM32L4 (ARM_CM4F) microcontroller
# It is expected the https://github.com/STMicroelectronics/FreeRTOS-Kernel is cloned to ${CMAKE_SOURCE_DIR}/Modules/FreeRTOS-Kernel.

# Generates static libraries:
# - SHARC::RTOS

# Path to FreeRTOS Kernel
set(RTOS_DIR FreeRTOS-Kernel)
# Modify this to the path where your micrcontroller specific port is
set(RTOS_DIR_MCU ${RTOS_DIR}/portable/GCC/ARM_CM4F) # For cortex-m4 microcontroller
set(RTOS_HEAP    ${RTOS_DIR}/portable/MemMang/heap_4.c) # Select which heap implementation to use

# Include directories for the Kernel and specific MCU port
set(FREERTOS_KERNEL_INCLUDE_DIRECTORIES
	${CMAKE_SOURCE_DIR}/Modules/FreeRTOS-Kernel/include
    ${CMAKE_SOURCE_DIR}/Modules/FreeRTOS-Kernel/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/Core/Inc/RTOS_Init
)

# Source files for the Kernel, specific MCU port and heap implementation
file(GLOB FREERTOS_KERNEL_SOURCES
	${CMAKE_SOURCE_DIR}/Modules/FreeRTOS-Kernel/*.c
    ${CMAKE_SOURCE_DIR}/Modules/FreeRTOS-Kernel/portable/MemMang/heap_4.c
    ${CMAKE_SOURCE_DIR}/Modules/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/*.c
)

# FreeRTOS Library
add_library(FREERTOS_KERNEL STATIC
	${FREERTOS_KERNEL_SOURCES}
)

target_include_directories(FREERTOS_KERNEL SYSTEM
	PUBLIC ${FREERTOS_KERNEL_INCLUDE_DIRECTORIES}
)

add_library(SHARC::RTOS ALIAS FREERTOS_KERNEL)
