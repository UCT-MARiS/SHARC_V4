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
#define FFT 0
#define IFFT 1

/**
 * @brief Sets up the Hardware Abstraction Layer (HAL) with the specified interface.
 * 
 * This function initializes the HAL using the provided interface object.
 * 
 * @param hal The interface object that implements the IHAL interface.
 */
void setupHAL(IHAL* hal);

/**
 * @brief Initializes the UART DMA for communication.
 * 
 * This function sets up the UART DMA to enable communication with other devices.
 * 
 * @param huart Pointer to the UART handle structure.
 */

void begin_UART_DMA(UART_HandleTypeDef *huart);

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
 * @brief Integrates the wave amplitude array in the frequency domain.
 * 
 * This function integrates the wave amplitude array in the frequency domain. 
 * Note that it does not divide by jOmega (rather it divides by Omega) because the absolute
 * value of the amplitude array is taken before integration.
 * 
 * @param waveAmplitudeArray The input wave amplitude array.
 */
void frequencyDomainIntegration(float32_t* velocityFFT);