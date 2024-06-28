#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "hal_mock.h"
#include "main.hpp"

extern "C" {
#include "string.h"
#include <stdarg.h>
#include "stdio.h"
}

using ::testing::_;
using ::testing::Return;

class MainTest : public ::testing::Test {
protected:
    // Creating an instance of the mock class
    MockHAL mockHal;
    MockHAL* mockHALInstance = &mockHal;
    
    void SetUp() override {
        // Set the mock object to be used by the main program
        mockHALInstance = &mockHal;
    }

    void TearDown() override {
        // Reset the mock object if needed
        mockHALInstance = nullptr;
    }
};

TEST_F(MainTest, MainFunctionCalls) {

    std::cout << "Starting MainFunctionCalls test" << std::endl;

    EXPECT_CALL(mockHal, HAL_Init()).Times(1);
    EXPECT_CALL(mockHal, SystemClock_Config()).Times(1);
    EXPECT_CALL(mockHal, MX_GPIO_Init()).Times(1);
    EXPECT_CALL(mockHal, MX_LPUART1_UART_Init()).Times(1);

    // Call main
    setupHAL();
}


// Main function for running tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
