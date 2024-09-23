/**
 * @file wave_functions.h
 * @author Michael Noyce 
 * @brief 
 * @version 0.1
 * @date 2024-09-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#ifndef PWELCH_H
#define PWELCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"  // CMSIS DSP Library

// Define constants for the pwelch function
#define FFT_SIZE 1024   // Size of the FFT
#define WINDOW_SIZE FFT_SIZE  // Window size equals FFT size
#define OVERLAP 0.5f    // 50% overlap between segments
#define NF 1024*1024.0f // Normalization factor for length of PSD i.e. N^2
#define CF 2.0f         // Correction factor i.e. both sides of the spectrum

/**
 * @brief Function to calculate Welch's Power Spectral Density (PSD) estimate.
 * 
 * @param input_signal Pointer to the input signal array.
 * @param signal_size The length of the input signal.
 * @param psd_output Pointer to the array where the power spectrum will be stored.
 */
void pwelch(float32_t* input_signal, uint32_t signal_size, float32_t* psd_output);

/**
 * @brief Initializes the Hamming window using CMSIS-DSP library functions.
 */
void init_hamming_window(float32_t* window);

#ifdef __cplusplus
}
#endif

#endif // PWELCH_H
