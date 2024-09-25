/**
 * @file wave_functions.c
 * @author Michael Noyce
 * @brief 
 * @version 2.1
 * @date 2024-09-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include "wave_functions.h"

// Define the sampling frequency if not already defined
#ifndef SAMPLING_FREQUENCY
#define SAMPLING_FREQUENCY 256.0f  // Example value, replace with actual sampling frequency
#endif


// Input and output buffers (these could be defined elsewhere or passed in by the user)
static float32_t segment[WINDOW_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t window[WINDOW_SIZE];
static float32_t moments[5];  // Array to store spectral moments

/**
 * @brief Function to calculate Welch's Power Spectral Density (PSD) estimate.
 * 
 * @param input_signal Pointer to the input signal array.
 * @param signal_size The length of the input signal.
 * @param psd_output Pointer to the array where the power spectrum will be stored.
 */
void pwelch(float32_t* input_signal, uint32_t signal_size, float32_t* psd_output) {
    uint32_t step_size = WINDOW_SIZE * (1 - OVERLAP);  // Step size with overlap
    uint32_t num_steps = (signal_size - WINDOW_SIZE) / step_size + 1;  // Calculate number of full steps

     // Initialize window function
    float32_t window[WINDOW_SIZE];
    arm_hanning_f32(window, WINDOW_SIZE);
    
    // Compute the normalization factor (sum of squares of window coefficients)
    float32_t NF = 0.0f;
    for (uint32_t n = 0; n < WINDOW_SIZE; n++) {
        NF += window[n] * window[n];
    }

    // Apply Hamming window to each segment and compute FFT
    for (uint32_t i = 0; i < num_steps; i++) {
        // Extract segment from the input signal
        arm_copy_f32(&input_signal[i * step_size], segment, WINDOW_SIZE);

        // Apply the Hamming window to the segment
        arm_mult_f32(segment, window, segment, WINDOW_SIZE);

        // Perform FFT on the windowed segment
        arm_rfft_fast_instance_f32 fft_instance;
        arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
        arm_rfft_fast_f32(&fft_instance, segment, fft_output, 0);

        // Calculate magnitude squared (periodogram)
        float32_t mag_output[FFT_SIZE / 2];
        arm_cmplx_mag_squared_f32(fft_output, mag_output, FFT_SIZE / 2);

        // Accumulate power spectrum (average)
        if (i == 0) {
            arm_copy_f32(mag_output, psd_output, FFT_SIZE / 2);
        } else {
            arm_add_f32(psd_output, mag_output, psd_output, FFT_SIZE / 2);
        }
    }

    // Final averaging and scaling
    // Divide by number of segments and normalize by window power
    // Correct for the fact that we only use half of the FFT output (i.e. *2)
    // Divide by SAMPLING FREQUENCY to get power spectral density
    float32_t scaling_factor = 2.0f / (num_steps * NF * SAMPLING_FREQUENCY);
    arm_scale_f32(psd_output, scaling_factor, psd_output, FFT_SIZE / 2);

    // Correct the DC and Nyquist components (do not double them)
    psd_output[0] /= 2.0f;  // DC component
    if (FFT_SIZE % 2 == 0) {
        psd_output[(FFT_SIZE / 2) - 1] /= 2.0f;  // Nyquist component for even FFT size
    }
}

/**
 * @brief Calculate the spectral moments of a power spectral density (PSD) estimate.
 * 
 * @param psd psd estimate
 * @param PSD_N psd length
 * @param moments moments array 
 */
void compute_spectral_moments(float32_t *psd, uint32_t PSD_N, float32_t *moments)
{

    float32_t df = (float32_t) SAMPLING_FREQUENCY / (2 * (float32_t) PSD_N);  // Frequency resolution
    uint32_t k;
    float32_t f;
    float32_t M0 = 0.0f, M1 = 0.0f, M2 = 0.0f, M3 = 0.0f, M4 = 0.0f;

    // Loop over frequency bins
    for (k = 0; k < PSD_N; k++)
    {
        f = k * df;
        float32_t psd_k = psd[k];
        float32_t f1 = f;
        float32_t f2 = f1 * f1;
        float32_t f3 = f2 * f1;
        float32_t f4 = f2 * f2;

        M0 += psd_k;
        M1 += f1 * psd_k;
        M2 += f2 * psd_k;
        M3 += f3 * psd_k;
        M4 += f4 * psd_k;
    }

    // Multiply moments by frequency resolution
    M0 *= df;
    M1 *= df;
    M2 *= df;
    M3 *= df;
    M4 *= df;

    // Store computed moments
    moments[0] = M0;
    moments[1] = M1;
    moments[2] = M2;
    moments[3] = M3;
    moments[4] = M4;
}

/**
 * @brief Calculate ocean wave parameters from spectral moments.
 * 
 * @param moments Array of spectral moments.
 * @param wave_params Array to store calculated wave parameters.
 */
void calculate_wave_parameters(float32_t *moments, float32_t *wave_params) {
    // Significant wave height (Hs)
    wave_params[0] = 4.0f * sqrtf(moments[0]);

    // Peak period (Tp)
    wave_params[1] = sqrtf(moments[0] / moments[2]);

    // Mean period (Tm)
    wave_params[2] = moments[0] / moments[1];

    // Energy period (Te)
    wave_params[3] = sqrtf(moments[2] / moments[0]);

    // Mean zero-crossing period (Tz)
    wave_params[4] = sqrtf(moments[0] / moments[2]);
}
