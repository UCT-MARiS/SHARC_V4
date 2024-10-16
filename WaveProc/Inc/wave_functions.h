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
#define INPUT_SIGNAL_SIZE 4096  // Size of the input signal
#define FFT_SIZE 1024   // Size of the FFT
#define WINDOW_SIZE FFT_SIZE  // Window size equals FFT size
#define OVERLAP 0.5f    // 50% overlap between segments

//Define constants for the integrate_psd_cmsis_static function
#define PSD_SIZE FFT_SIZE / 2  // Size of the input signal

// Define constants for compute_spectral_moments
#define SAMPLING_FREQUENCY 3.125f        // Sampling frequency

// Fourier Filter
    // Cutoff frequencies in Hz
#define F_CUTOFF1 0.02f
#define F_CUTOFF2 0.03f
    // Cosine taper width
#define TAPER_WIDTH (F_CUTOFF2 - F_CUTOFF1)
    // Number of frequency bins for real FFT
#define NUM_FREQ_BINS (FFT_SIZE / 2 + 1)
    // Buffers for FFT computation
static float32_t fftOutput[FFT_SIZE];          // FFT output (interleaved complex numbers)
    // Filter function in the frequency domain
static float32_t filter[NUM_FREQ_BINS];        // Frequency response of the filter
    // FFT instance
static arm_rfft_fast_instance_f32 S_local;

// Define constants for lpf_decimate function
    //LPF Decimate Defintions
#define SNR_THRESHOLD_F32    75.0f
#define BLOCK_SIZE           128 // Must be a multiple of 32   
    /* Must be a multiple of 16 */
#define NUM_TAPS_ARRAY_SIZE  64
#define NUM_TAPS              64
#define DECIMATION_CONSTANT 32
extern float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
extern const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE];

/**
 * @brief Decimate the input signal using an FIR filter.
 * @note The filter only decimates if the block size is a multiple of the decimation factor.
 * @param S Pointer to the FIR decimate instance.
 * @param testInput Pointer to the input signal array.
 * @param testOutput Pointer to the output signal array.
 */
void lpf_decimate(arm_fir_decimate_instance_f32* S, float32_t* testInput, float32_t* testOutput);

/**
 * @brief Calibration Function for the ICM20689
 * @param rawData
 * @param calOutput
 */
void calibrate(float32_t* rawData, float32_t* calOutput);

/**
 * @brief Detrending function
 * @param rawData 
 * @param detrendOutput
 * @param previousSample 
 */
void detrend(float32_t* calOutput, float32_t* detrendOutput, float32_t* previousSample);


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


/**
 * @brief Integrate acceleration PSD to obtain displacement PSD using CMSIS DSP functions with static arrays.
 *
 * @param[in]  pSaa     Pointer to the input acceleration PSD array.
 * @param[in]  pFreqs   Pointer to the frequency array corresponding to the PSD.
 * @param[out] pSxx     Pointer to the output displacement PSD array.
 * @param[in]  length   Length of the input/output arrays (must be <= MAX_LENGTH).
 */
void integrate_psd_cmsis_static(const float32_t *pSaa, const float32_t *pFreqs, float32_t *pSxx, uint32_t length);

/**
 * @brief Implements fourier filtering in the frequency domain.
 * 
 */
void init_hpf_fourier(void);

/**
 * @brief Implements fourier filtering in the frequency domain.
 * 
 * @param input 
 * @param output 
 */
void fourier_filter(float32_t* input, float32_t* output);


#ifdef __cplusplus
}
#endif

#endif // WAVE_FUNCTIONS_H
