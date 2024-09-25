/**
 * @file wave_functions.h
 * @author Michael Noyce 
 * @brief 
 * @version 2.1
 * @date 2024-09-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#ifndef WAVE_FUNCTIONS_H
#define WAVE_FUNCTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"  // CMSIS DSP Library

// Define constants for the pwelch function
#define FFT_SIZE 1024   // Size of the FFT
#define WINDOW_SIZE FFT_SIZE  // Window size equals FFT size
#define OVERLAP 0.5f    // 50% overlap between segments

// Define constants for compute_spectral_moments
#define SAMPLING_FREQUENCY 4        // Sampling frequency
 
/**
 * @brief Function to calculate Welch's Power Spectral Density (PSD) estimate.
 * 
 * @param input_signal Pointer to the input signal array.
 * @param signal_size The length of the input signal.
 * @param psd_output Pointer to the array where the power spectrum will be stored.
 */
void pwelch(float32_t* input_signal, uint32_t signal_size, float32_t* psd_output);


/**
 * @brief Compute spectral moments of a power spectral density (PSD) estimate.
 * @param psd Pointer to the PSD estimate.
 * @param N Length of the PSD estimate.
 * @param moments Pointer to the array where the moments will be stored.
 * 
 */
void compute_spectral_moments(float32_t *psd, uint32_t N, float32_t *moments);


/**
 * @brief Calculate ocean wave parameters from spectral moments.
 * 
 * @param moments Array of spectral moments.
 * @param wave_params Array to store calculated wave parameters.
 */
void calculate_wave_parameters(float32_t *moments, float32_t *wave_params);

#ifdef __cplusplus
}
#endif

#endif // PWELCH_H
