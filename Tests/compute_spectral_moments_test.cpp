#include <gtest/gtest.h>
#include "wave_functions.h"  // Include the pwelch header
#include <cmath>     // For

class ComputeSpectralMomentsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize any shared resources or setup code here
    }

    void TearDown() override {
        // Clean up any resources used in the tests
    }
};

// Test case for checking the correct computation of spectral moments
TEST_F(ComputeSpectralMomentsTest, CorrectComputation) {
    const uint32_t PSD_N = 5;
    float32_t psd[PSD_N] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    float32_t moments[5] = {0.0f};

    compute_spectral_moments(psd, PSD_N, moments);

    // Expected values (these should be calculated based on the input PSD and the function logic)
    float32_t expected_moments[5] = {6.0f, 6.4f, 8.32f, 11.6224f, 16.93696f};

    for (int i = 0; i < 5; ++i) {
        EXPECT_NEAR(moments[i], expected_moments[i], 1e-1);
    }
}

// Test case for checking the computation of spectral moments for a sine wave
TEST_F(ComputeSpectralMomentsTest, sinePSD) {
    
    // FFT operation using CMSIS DSP library
    const uint32_t inputSize = 4096;
    const uint32_t fftSize = 1024;

    // Input signal (example: sine wave)
    float32_t inputSignal[inputSize];
    float32_t fs1 = 0.1f; // Frequency of the sine wave
    float32_t fsample = 4.0f; // Sampling frequency

    for (uint32_t i = 0; i < inputSize; i++) {
        inputSignal[i] = arm_sin_f32(2 * PI * i * fs1 / fsample);
    }

    // Output buffer
    float32_t outputSignal[fftSize/2];

    // Welch PSD estimate
    pwelch(inputSignal, inputSize, outputSignal);

    // Spectral moments
    float32_t moments[5];

    // Compute spectral moments
    compute_spectral_moments(outputSignal, fftSize/2, moments);

    // Check moments
    EXPECT_NEAR(moments[0], 0.499988, 1e-5);
    EXPECT_NEAR(moments[1], 0.049999, 1e-5);
    EXPECT_NEAR(moments[2], 0.005002, 1e-5);
    EXPECT_NEAR(moments[3], 0.000501, 1e-5);
    EXPECT_NEAR(moments[4], 0.000050, 1e-5);

}

// Test case for a zero PSD
TEST_F(ComputeSpectralMomentsTest, ZeroPSD) {
    const uint32_t PSD_N = 5;
    float32_t psd[PSD_N] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float32_t moments[5] = {0.0f};

    compute_spectral_moments(psd, PSD_N, moments);

    // Expected values (these should be calculated based on the input PSD and the function logic)
    float32_t expected_moments[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 5; ++i) {
        EXPECT_NEAR(moments[i], expected_moments[i], 1e-5);
    }
}
