#include <gtest/gtest.h>
#include "wave_functions.h"  // Include the pwelch header
#include <cmath>     // For fabs

// Define constants for the test
#define SIGNAL_SIZE 1024  // Length of the input signal
#define TOLERANCE 1e-10    // Tolerance for floating-point comparisons

// Test fixture class for pwelch
class PwelchTest : public ::testing::Test {
protected:
    // Test data: Input signal and PSD output
    float32_t input_signal[SIGNAL_SIZE];
    float32_t window[WINDOW_SIZE];
    float32_t psd_output[FFT_SIZE / 2];

    // Setup function to initialize the test environment
    void SetUp() override {
        // Initialize the input signal with some test data (simple sine wave)
        for (uint32_t i = 0; i < SIGNAL_SIZE; ++i) {
            input_signal[i] = sinf(2.0f * M_PI * i / SIGNAL_SIZE);  // Example sine wave
        }

        // Zero the PSD output array
        memset(psd_output, 0, sizeof(psd_output));
    }

    // Tear down function for cleanup (not needed in this case)
    void TearDown() override {
    }
};

// Test pwelch function with valid input signal
TEST_F(PwelchTest, PwelchBasicFunctionality) {

    // Call the pwelch function
    pwelch(input_signal, SIGNAL_SIZE, psd_output);

    // Check that the PSD output is not all zero
    float32_t psd_sum = 0;
    for (uint32_t i = 0; i < FFT_SIZE / 2; ++i) {
        psd_sum += fabsf(psd_output[i]);
    }

    // The power spectrum should have non-zero values for a sine wave
    EXPECT_GT(psd_sum, 0.0f);
}

// Test pwelch with a random noise input
TEST_F(PwelchTest, PwelchNoiseSignal) {

    // Generate random noise signal
    for (uint32_t i = 0; i < SIGNAL_SIZE; ++i) {
        input_signal[i] = (float32_t)rand() / RAND_MAX;  // Random noise between 0 and 1
    }

    // Call the pwelch function with the random noise signal
    pwelch(input_signal, SIGNAL_SIZE, psd_output);

    // Check that the PSD output has non-zero values (random noise should have power)
    float32_t psd_sum = 0;
    for (uint32_t i = 0; i < FFT_SIZE / 2; ++i) {
        psd_sum += fabsf(psd_output[i]);
    }

    EXPECT_GT(psd_sum, 0.0f);
}


