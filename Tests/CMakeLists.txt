# add preprocesor defintions
add_definitions(-DSTM32L4R5xx)

# Define the test sources
set(TEST_SOURCES
    hello_test.cpp
    main_test.cpp
    ${CMAKE_SOURCE_DIR}/Core/Src/HAL_Init/setup_hal.cpp
    # Add more test files as needed
)

# Add mock HAL source files
set(MOCK_HAL_SOURCES
    ${CMAKE_SOURCE_DIR}/Tests/Mock/Src/hal_mock.cpp
    # Add other mock HAL source files as needed
)

# Create an executable for the tests
add_executable(run_tests ${TEST_SOURCES} ${MOCK_HAL_SOURCES})

target_include_directories(run_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Core/Inc/HAL_Init
    ${CMAKE_SOURCE_DIR}/Tests/Mock/Inc
)

# Link the test executable with Google Test
target_link_libraries(run_tests gtest gtest_main gmock)

# Add the test executable to CTest
add_test(NAME run_tests COMMAND run_tests)