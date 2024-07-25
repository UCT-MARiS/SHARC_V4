# add preprocesor defintions
add_definitions(-DSTM32L4R5xx)

# Define the test sources
set(TEST_SOURCES
    hello_test.cpp
    main_test.cpp
    ${CMAKE_SOURCE_DIR}/Core/Src/HAL_Init/setup_hal.cpp
    sensor_test.cpp
    # Add more test files as needed
)

# Create an executable for the tests
add_executable(run_tests ${TEST_SOURCES}) 


# Include directories
set(MAIN_INCLUDE_DIRECTROIES
    ${CMAKE_SOURCE_DIR}/Core/Inc
)

set(MOCK_INCLUDE_DIRECTORIES
    ${CMAKE_SOURCE_DIR}/Tests/Mock/Inc
)

set(HAL_INCLUDE_DIRECTORIES
    ${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/CMSIS/Device/ST/STM32L4xx/Include
    ${CMAKE_SOURCE_DIR}/Modules/STM32CubeL4/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Core/Inc/HAL_Init
)

set(SENSOR_INCLUDE_DIRECTORIES
    ${CMAKE_SOURCE_DIR}/Core/Inc/Sensor_Interface
)

target_include_directories(run_tests PRIVATE
    ${MAIN_INCLUDE_DIRECTROIES}
    ${MOCK_INCLUDE_DIRECTORIES}
    ${HAL_INCLUDE_DIRECTORIES}
    ${SENSOR_INCLUDE_DIRECTORIES}
)

# Link the test executable with Google Test
target_link_libraries(run_tests gtest gtest_main gmock)

# Add the test executable to CTest
add_test(NAME run_tests COMMAND run_tests)