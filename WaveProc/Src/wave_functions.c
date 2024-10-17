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

//LPF Filter variables
//  Assume 100 Hz input signal 3.125 Hz output signal
//  fc = 1.5 Hz, fs = 100 Hz, M = 32
//  calculated with the following MATLAB code:
/*
%function to determine impulse response of filter 
n = 64; %length of filter, fir1 requires n-1 
%64 chosen as standard trade off between stm32 processing power and
%accuracy 
fs = 100;
fnyquist = fs/2; %2*fmax, or fs/2 
fc = 1.5;  
b = fir1((n-1), fc/fnyquist, "low"); %second argument is the normalised cutoff frequency 
% hamming window and lowpass filter used as defaults in fir1

[h,f]=freqz(b,1,512);%amplitude-frequency filter response
figure(1)
plot(f*fs/(2*pi),20*log10(abs(h)))%normalised frequency vs  
xlabel('frequency/Hz');ylabel('gain/dB');title('The gain response of lowpass filter');
grid on
*/
float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
0.00016994838,0.00027708213,0.00041562635,0.00060343095,0.00085871918,0.0011994835,0.0016428637,0.0022045268,0.0028980667,0.003734445,0.0047214889,0.0058634654,0.0071607451,0.0086095686,0.010201925,0.011925552,
0.01376405,0.015697126,0.017700948,0.019748609,0.02181069,0.023855901,0.025851793,0.027765514,0.029564592,0.031217718,0.032695517,0.033971279,0.035021622,0.035827086,0.036372626,0.036647991,
0.036647991,0.036372626,0.035827086,0.035021622,0.033971279,0.032695517,0.031217718,0.029564592,0.027765514,0.025851793,0.023855901,0.02181069,0.019748609,0.017700948,0.015697126,0.01376405,
0.011925552,0.010201925,0.0086095686,0.0071607451,0.0058634654,0.0047214889,0.003734445,0.0028980667,0.0022045268,0.0016428637,0.0011994835,0.00085871918,0.00060343095,0.00041562635,0.00027708213,0.00016994838
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
void lpf_decimate(arm_fir_decimate_instance_f32* S, float32_t* testInput, float32_t* testOutput)
{
    uint32_t blockSize = BLOCK_SIZE;
    uint32_t numBlocks = FFT_SIZE / blockSize; // Assuming FFT_SIZE is 1024
    uint32_t i;

    for(i = 0; i < numBlocks; i++)
    {
        arm_fir_decimate_f32(S, 
                             testInput + (i * blockSize), 
                             testOutput + (i * (blockSize / DECIMATION_CONSTANT)), 
                             blockSize);
    }
}

/**
 * @brief Calibration function for the ICM20649
 * @param rawData
 * @param calOutput
 */
void calibrate(float32_t* rawData, float32_t* calOutput)
{
    // Define constants for scaling and bias removal
    const float32_t scale_factor = 9.81f / 8192.0f; // scale factor for 16-bit signed data
    const float32_t bias_offset = 0.0255f - 9.81f; //bias minus gravity

    // Use CMSIS-DSP function to scale the raw data
    arm_scale_f32(rawData, scale_factor, calOutput, FFT_SIZE / DECIMATION_CONSTANT );

    // Use CMSIS-DSP function to add the bias offset
    arm_offset_f32(calOutput, bias_offset, calOutput, FFT_SIZE / DECIMATION_CONSTANT );

}

/**
 * @brief Detrending function
 * @param rawData
 * @param detrendOutput
 * @param previousSample
 */
void detrend(float32_t* calOutput, float32_t* detrendOutput, float32_t* previousSample)
{

	//Determining Mean of data
	uint32_t i;
	float32_t sumTotal, dataMean;

	for (i= 0; i < FFT_SIZE/DECIMATION_CONSTANT; i++)
	{
		sumTotal = sumTotal + calOutput[i];
	}

	dataMean = sumTotal/(FFT_SIZE/DECIMATION_CONSTANT);

	for (i= 0; i < (FFT_SIZE/DECIMATION_CONSTANT); i++)
	{
		detrendOutput[i] = calOutput[i] - dataMean;
	}
	// RC Highpass function	for detrending (moving average filter)
	float32_t k = 0.9995;
	float32_t s[FFT_SIZE];
	float32_t detrendTemp[FFT_SIZE];


	s[0] = 0;

	for (i= 1; i < FFT_SIZE; i++)
	{
		 s[i] = detrendTemp[i] + k*(s[i-1]);
		 detrendTemp[i] = detrendTemp[i] - (1 - k)*(s[i]);
	}

	detrendOutput = detrendTemp;

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

    // Remove the DC component
    psd_output[0] = 0.0f;

    // Remove frequency components below 0.02Hz
    uint32_t idx = 0;
    uint32_t freqIndex = (uint32_t)(0.02 / (SAMPLING_FREQUENCY / (float32_t)FFT_SIZE));
    while (idx < freqIndex) {
        psd_output[idx] = 0.0f;
        idx++;
    }

    // Correct the Nyquist component (do not double it)
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
    if (length > PSD_SIZE)
    {
        // Handle error: length exceeds maximum allowed size
        return;
    }

    // Static buffers
    static float32_t omega[PSD_SIZE];
    static float32_t omega2[PSD_SIZE];
    static float32_t omega4[PSD_SIZE];
    static float32_t recip_omega4[PSD_SIZE];

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

/**
 * @brief Implements fourier filtering in the frequency domain.
 * 
 */
void init_hpf_fourier(void) {
    // Initialize the FFT instance
    arm_rfft_fast_init_f32(&S_local, FFT_SIZE);

    // Frequency resolution
    float32_t freqResolution = SAMPLING_FREQUENCY / FFT_SIZE;

    // Generate the filter function
    for (uint32_t k = 0; k < NUM_FREQ_BINS; k++) {
        float32_t freq = k * freqResolution;

        // Initialize the filter coefficient
        float32_t H = 1.0f;

        if (freq < F_CUTOFF1) {
            H = 0.0f;
        } else if (freq >= F_CUTOFF1 && freq <= F_CUTOFF2) {
            float32_t taper = 0.5f * (1.0f - cosf(PI * (freq - F_CUTOFF1) / TAPER_WIDTH));
            H = taper;
        } else {
            H = 1.0f;
        }

        /* // Apply double integration scaling
        if (freq > 0.0f) {
            float32_t scaling = 1.0f / powf(2.0f * PI * freq, 2);
            H *= scaling;
        } else {
            H = 0.0f;
        } */ 
        filter[k] = H;
    }
}

/**
 * @brief Implements fourier filtering in the frequency domain.
 * 
 * @param input 
 * @param output 
 */
void fourier_filter(float32_t* input, float32_t* output) {
    
    // Perform forward FFT
    arm_rfft_fast_f32(&S_local, input, fftOutput, 0);

    // Apply the filter in the frequency domain
    for (uint32_t k = 0; k < NUM_FREQ_BINS; k++) {
        float32_t H = filter[k];

        if (k == 0) {
            // DC component
            fftOutput[0] *= H;
            fftOutput[1] *= H;
        } else if (k < FFT_SIZE / 2) {
            // Positive frequencies
            uint32_t idx = 2 * k;
            fftOutput[idx]     *= H;  // Real part
            fftOutput[idx + 1] *= H;  // Imaginary part
        } else if (k == FFT_SIZE / 2) {
            // Nyquist frequency
            fftOutput[1] *= H;
        }
    }

    // Perform inverse FFT
    arm_rfft_fast_f32(&S_local, fftOutput, output, 1);

    // Scale the output by 1/FFT_SIZE
    arm_scale_f32(output, 1.0f / FFT_SIZE, output, FFT_SIZE);
}