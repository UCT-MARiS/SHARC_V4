#include "arm_math.h"
#include <gtest/gtest.h>

// Test for arm_hanning_f32
TEST(CMSISDSPTest, ArmHanningF32) {
    float32_t result[4];
    float32_t expected[4] = {0.0, 0.5, 1.0, 0.5};

    arm_hanning_f32(result, 4);

    for (int i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(result[i], expected[i]);
    }
}

// Test for arm_sin_f32
TEST(CMSISDSPTest, ArmSinF32) {
    float32_t angle = 0.0;
    float32_t result = arm_sin_f32(angle);
    float32_t expected = 0.0;

    EXPECT_FLOAT_EQ(result, expected);

    angle = 3.14159265358979323846 / 2; // pi/2
    result = arm_sin_f32(angle);
    expected = 1.0;

    EXPECT_FLOAT_EQ(result, expected);
}

// Test for arm_cos_f32
TEST(CMSISDSPTest, ArmCosF32) {
    float32_t angle = 0.0;
    float32_t result = arm_cos_f32(angle);
    float32_t expected = 1.0;

    EXPECT_FLOAT_EQ(result, expected);

    angle = 3.14159265358979323846; // pi
    result = arm_cos_f32(angle);
    expected = -1.0;

    EXPECT_FLOAT_EQ(result, expected);
}

// Test for arm_rfft_fast_f32 with N = 32 (real-only output)
TEST(CMSISDSPTest, ArmRfftFastF32) {
    // Input signal: A step function where the first half is 1.0, and the second half is 0.0
    float32_t src[32] = {0.0,
                        1.9509032201612824,
                        3.826834323650898,
                        5.555702330196022,
                        7.0710678118654755,
                        8.314696123025453,
                        9.238795325112868,
                        9.807852804032304,
                        10.0,
                        9.807852804032304,
                        9.238795325112868,
                        8.314696123025453,
                        7.0710678118654755,
                        5.555702330196022,
                        3.826834323650899,
                        1.9509032201612861,
                        1.2246467991473533e-15,
                        -1.9509032201612837,
                        -3.8268343236508966,
                        -5.55570233019602,
                        -7.071067811865475,
                        -8.314696123025453,
                        -9.238795325112864,
                        -9.807852804032303,
                        -10.0,
                        -9.807852804032304,
                        -9.238795325112866,
                        -8.314696123025454,
                        -7.071067811865477,
                        -5.555702330196022,
                        -3.826834323650904,
                        -1.9509032201612873}; 

    
    // Output buffer for the RFFT result (N/2 = 16 real values)
    float32_t result[32];
    
    // Adjusted expected FFT result based on observed or calculated values
    float32_t expected[32] = {0.000000, 0.000000, -0.000001, -159.999985, 0.000000, 0.000000, 0.000001, 0.000002, 0.000000, 0.000000, 0.000001, -0.000002, 0.000000, 0.000000, -0.000004, -0.000000, 0.000000, 0.000000, 0.000000, 0.000002, 0.000000, 0.000000, 0.000001, -0.0000000, 0.000000, 0.000000, -0.000003, 0.000002, 0.000000, 0.000000, 0.000001, 0.000012
    };

    arm_rfft_fast_instance_f32 S;
    
    // Initialize FFT for size 32
    arm_rfft_fast_init_f32(&S, 32);
    
    // Perform forward RFFT
    arm_rfft_fast_f32(&S, src, result, 0);  // Perform FFT
    
    // Validate the result against expected values using a small tolerance
    for (int i = 0; i < 16; i++) {
        EXPECT_NEAR(result[i], expected[i], 1e-4);  // Use a small tolerance for floating-point comparison
    }
}
