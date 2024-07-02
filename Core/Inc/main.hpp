#pragma once

#include "hal_interface.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

/**
 * @brief Sets up the Hardware Abstraction Layer (HAL) with the specified interface.
 * 
 * This function initializes the HAL using the provided interface object.
 * 
 * @param hal The interface object that implements the IHAL interface.
 */
void setupHAL(IHAL* hal);