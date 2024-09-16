#include "arm_math.h"
#include <gtest/gtest.h>

// Test for arm_hanning_f32
TEST(CMSISDSPTest, ArmHanningF32) {
    float32_t result[4];
    float32_t expected[4] = {0.0, 0.75, 0.75, 0.0};

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

// Test for arm_rfft_fast_f32
TEST(CMSISDSPTest, ArmRfftFastF32) {
    float32_t src[8] = {1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    float32_t result[8];
    float32_t expected[8] = {4.0, 0.0, 0.0, 0.0, -4.0, 0.0, 0.0, 0.0};

    arm_rfft_fast_instance_f32 S;
    arm_rfft_fast_init_f32(&S, 8);
    arm_rfft_fast_f32(&S, src, result, 0);

    for (int i = 0; i < 8; i++) {
        EXPECT_FLOAT_EQ(result[i], expected[i]);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
