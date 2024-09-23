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

#define FFT_LENGTH 512
#define SIGNAL_LENGTH 1024
#define SEGMENT_NO 2
#define F_SAMPLE 5
#define FFT 0
#define IFFT 1
#define DECIMATION_CONSTANT 1

/* Structs ---------------------------------------------------*/

/*
 *  @brief Wave Data Structure
 */
typedef struct
{
	float32_t Hm0; //Significant waveheight
	float32_t Hrms; //Root mean square waveheight
	float32_t T0; //Mean zero crossing period
	float32_t TM01; //Mean spectral wave period
	float32_t Tp; // Peak wave period
	float32_t M0; //0th Spectral moment
	float32_t M1; //1st spectral moment
	float32_t M2; //2nd spectral moment
	float32_t PSD[30]; //PSD Period in spectral bins

}Wave_Data_t;

/**
 * @brief Sets up the Hardware Abstraction Layer (HAL) with the specified interface.
 * 
 * This function initializes the HAL using the provided interface object.
 * 
 * @param hal The interface object that implements the IHAL interface.
 */
void setupHAL(IHAL* hal);

/**
 * @brief Initializes the UART interrupt for communication.
 * 
 * This function sets up the UART interrupt to enable communication with other devices.
 * 
 * @param huart Pointer to the UART handle structure.
 */

void initUARTInterrupt(UART_HandleTypeDef *huart);

/**
 * @brief Performs a busy-wait delay.
 * 
 * This function creates a delay by busy-waiting for a specified count.
 * 
 * @param count The number of counts to wait.
 */

void delay(volatile uint32_t count);


/**
 * @brief Performs a Fast Fourier Transform (FFT) on a segment of data.
 * 
 * This function performs an FFT on a segment of data and stores the result in the output buffer.
 * 
 * @param segment The input segment of data.
 * @param fftOutput The output buffer for the FFT result.
 * @param fftSize The size of the FFT.
 * @param ifftFlag The flag to indicate whether to perform an IFFT.
 */

void performFFT(float32_t* segment, float32_t* fftOutput, uint32_t fftSize, uint32_t ifftFlag);

/**
 * @brief Performs the Welch method to calculate the Power Spectral Density (PSD).
 * 
 * This function calculates the PSD using the Welch method.
 * 
 * @param PSD The output buffer for the PSD.
 * @param timeSeries The input time series data.
 */

void WelchMethod(float32_t* PSD, float32_t* timeSeries);

/**
 * @brief Processes a single segment of data using the Hanning window.
 * 
 * This function processes a single segment of data using the Hanning window.
 * 
 * @param singleSegment The output buffer for the processed segment.
 * @param timeSeries The input time series data.
 * @param segmentIndex The index of the segment.
 */

void singleSegmentPipeline(float32_t* singleSegment, float32_t* timeSeries, uint32_t segmentIndex);

/**
 * @brief Extracts wave parameters from the PSD.
 * 
 * This function extracts wave parameters from the PSD.
 * 
 * @param PSD The input PSD.
 * @param Hm0 The output buffer for the significant wave height.
 * @param Tm01 The output buffer for the mean spectral wave period.
 * @param Hrms The output buffer for the root mean square wave height.
 * @param T0 The output buffer for the mean zero crossing period.
 * @param Tp The output buffer for the peak wave period.
 * @param M0 The output buffer for the 0th spectral moment.
 * @param M1 The output buffer for the 1st spectral moment.
 * @param M2 The output buffer for the 2nd spectral moment.
 */

void waveParamExtract(float32_t* PSD, float32_t* Hm0, float32_t* Tm01, float32_t* Hrms, float32_t* T0, float32_t* Tp, float32_t*  M0, float32_t* M1, float32_t* M2);

/**
 * @brief Integrates the wave amplitude array in the frequency domain.
 * 
 * This function integrates the wave amplitude array in the frequency domain. 
 * Note that it does not divide by jOmega (rather it divides by Omega) because the absolute
 * value of the amplitude array is taken before integration.
 * 
 * @param waveAmplitudeArray The input wave amplitude array.
 */
void frequencyDomainIntegration(float32_t* velocityFFT);