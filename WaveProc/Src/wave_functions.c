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
#define SAMPLING_FREQUENCY 100.0f  // Example value, replace with actual sampling frequency
#endif


//LPF Filter variables
//  Assume 100 Hz input signal 4 Hz output signal
//  fc = 2 Hz, fs = 100 Hz, M = 25
//  calculated with the following MATLAB code:
/*
%function to determine impulse response of filter 
n = 32; %length of filter, fir1 requires n-1 
%29 chosen as standard trade off between stm32 processing power and
%accuracy 
fs = 100;
fnyquist = fs/2; %2*fmax, or fs/2 
fc = 2;  
b = fir1((n-1), fc/fnyquist); %second argument is the normalised cutoff frequency 
% hamming window and lowpass filter used as defaults in fir1

[h,f]=freqz(b,1,512); %amplitude-frequency filter response
figure(1)
plot(f*fs/(2*pi),20*log10(abs(h)))%normalised frequency vs  
xlabel('frequency/Hz');ylabel('gain/dB');title('The gain response of lowpass filter');

%CMSIS requires filter coefficients to be in reverse time order
b_flip = fliplr(b);

%Convert to float32_t format
str = sprintf([repmat('%.8g,',[1,16]) '\n'],b_flip); %b is the numerator of the FIR filter (which has no denominator)
writematrix(str);
*/
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
		0.00036790519,0.0017519181,-0.0017866308,-0.0024278967,0.0057034342,0.0013490959,-0.012672325,0.0056445545,0.020155499,-0.023080794,-0.021611854,0.055620945,0.0040563898,-0.11766473,0.085950419,0.49864407,
0.49864407,0.085950419,-0.11766473,0.0040563898,0.055620945,-0.021611854,-0.023080794,0.020155499,0.0056445545,-0.012672325,0.0013490959,0.0057034342,-0.0024278967,-0.0017866308,0.0017519181,0.00036790519,
};


// Input and output buffers (these could be defined elsewhere or passed in by the user)
static float32_t segment[WINDOW_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t window[WINDOW_SIZE];
static float32_t moments[5];  // Array to store spectral moments


/**
 * @brief Decimate the input signal using an FIR filter.
 * @param testInput
 * @param testOutput
 */
void lpf_decimate(float32_t* testInput, float32_t* testOutput)
{

	uint32_t blockSize = BLOCK_SIZE;
	uint32_t numBlocks = FFT_SIZE/BLOCK_SIZE;
	arm_fir_decimate_instance_f32 S;
	uint32_t i;
	float32_t  *inputF32, *outputF32;

	/* Initialize input and output buffer pointers */
	inputF32 = &testInput[0];
	outputF32 = &testOutput[0];

    arm_fir_decimate_init_f32(&S, NUM_TAPS, DECIMATION_CONSTANT, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

    for(i=0; i < numBlocks; i++)
    {
      arm_fir_decimate_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize/DECIMATION_CONSTANT), blockSize);
    }

}



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

/**
 * @brief Integrate acceleration PSD to obtain displacement PSD using CMSIS DSP functions with static arrays.
 *
 * @param[in]  pSaa     Pointer to the input acceleration PSD array.
 * @param[in]  pFreqs   Pointer to the frequency array corresponding to the PSD.
 * @param[out] pSxx     Pointer to the output displacement PSD array.
 * @param[in]  length   Length of the input/output arrays (must be <= MAX_LENGTH).
 */
void integrate_psd_cmsis_static(const float32_t *pSaa, const float32_t *pFreqs, float32_t *pSxx, uint32_t length)
{
    // Check that length does not exceed MAX_LENGTH
    if (length > INPUT_SIGNAL_SIZE)
    {
        // Handle error: length exceeds maximum allowed size
        return;
    }

    // Static buffers
    static float32_t omega[INPUT_SIGNAL_SIZE];
    static float32_t omega2[INPUT_SIGNAL_SIZE];
    static float32_t omega4[INPUT_SIGNAL_SIZE];
    static float32_t recip_omega4[INPUT_SIGNAL_SIZE];

    // Step 1: omega = 2 * PI * pFreqs
    arm_scale_f32(pFreqs, 2.0f * PI, omega, length);

    // Step 2: omega2 = omega * omega
    arm_mult_f32(omega, omega, omega2, length);

    // Step 3: omega4 = omega2 * omega2
    arm_mult_f32(omega2, omega2, omega4, length);

    // Step 4: Avoid division by zero or very small omega4
    float32_t threshold = 1e-20f;
    for (uint32_t i = 0; i < length; i++)
    {
        if (omega4[i] < threshold)
        {
            omega4[i] = threshold;
        }
    }

    // Step 5: Compute reciprocal of omega4
    // Using CMSIS DSP function for vector reciprocal approximation if available
    // If not, compute reciprocals manually
    #ifdef ARM_MATH_NEON
        // If NEON is available, you can use optimized functions
        arm_vrecip_f32(omega4, recip_omega4, length);
    #else
        for (uint32_t i = 0; i < length; i++)
        {
            recip_omega4[i] = 1.0f / omega4[i];
        }
    #endif

    // Step 6: pSxx = pSaa * recip_omega4
    arm_mult_f32(pSaa, recip_omega4, pSxx, length);
}
