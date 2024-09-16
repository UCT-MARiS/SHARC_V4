#include "wave_functions.h"

// Input and output buffers (these could be defined elsewhere or passed in by the user)
static float32_t segment[WINDOW_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t window[WINDOW_SIZE];

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
        for (uint32_t j = 0; j < FFT_SIZE / 2; j++) {
            fft_output[j] = fft_output[j] * fft_output[j];
        }

        // Accumulate power spectrum (average)
        if (i == 0) {
            arm_copy_f32(fft_output, psd_output, FFT_SIZE / 2);
        } else {
            arm_add_f32(psd_output, fft_output, psd_output, FFT_SIZE / 2);
        }
    }

    // Final averaging by number of segments
    arm_scale_f32(psd_output, 1.0f / num_steps, psd_output, FFT_SIZE / 2);
}

/**
 * @brief Initializes the Hamming window using CMSIS-DSP library functions.
 */
void init_hamming_window(void) {
    // Initialize the Hamming window using CMSIS-DSP function
    arm_hamming_window_f32(window, WINDOW_SIZE);
}
