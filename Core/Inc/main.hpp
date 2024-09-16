#pragma once

#include "hal_interface.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "arm_math.h"

#ifdef __cplusplus
}
#endif

#define SAMPLE_RATE 1024
#define DURATION 300
#define NPERSEG (SAMPLE_RATE * DURATION / 2)
#define NOVERLAP (NPERSEG / 2)
#define MAX_DATA_PAIRS 100
#define MAX_PARAMS 10
#define MAX_LENGTH (SAMPLE_RATE * DURATION)

/**
 * @brief Sets up the Hardware Abstraction Layer (HAL) with the specified interface.
 * 
 * This function initializes the HAL using the provided interface object.
 * 
 * @param hal The interface object that implements the IHAL interface.
 */
void setupHAL(IHAL* hal);

void initUARTInterrupt(UART_HandleTypeDef *huart);

void initUARTDMA(UART_HandleTypeDef *huart);

void generate_wave(const float32_t params[MAX_PARAMS][3], int num_params, float32_t duration, float32_t sample_rate, float32_t time[MAX_LENGTH], float32_t combined_wave[MAX_LENGTH]);
void compute_welch(const float32_t signal[MAX_LENGTH], int length, float32_t sample_rate, float32_t frequencies[NPERSEG], float32_t psd[NPERSEG]);
void integrate_displacement(const float32_t psd[NPERSEG], const float32_t frequencies[NPERSEG], float32_t displacement_spectrum[NPERSEG]);
void apply_bandpass_filter(float32_t spectrum[NPERSEG], const float32_t frequencies[NPERSEG], float32_t low_cut, float32_t high_cut);
void calculate_wave_parameters(const float32_t displacement_spectrum[NPERSEG], const float32_t frequencies[NPERSEG], float32_t& significant_wave_height, float32_t& mean_wave_period, float32_t& average_zero_crossing_period);
