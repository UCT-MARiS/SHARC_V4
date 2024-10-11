# Set the base directory for FATFS sources
set(GNSS_DIR "${CMAKE_SOURCE_DIR}/Modules/UBlox-NEO-M9N")


# Add the UBlox-NEO_M9N driver source files
add_library(UBlox-NEO_M9N
    path/to/UBlox-NEO_M9N/driver_file1.c
    path/to/UBlox-NEO_M9N/driver_file2.c
    # Add more driver source files as needed
)

# Include directories for the UBlox-NEO_M9N driver
target_include_directories(UBlox-NEO_M9N PUBLIC path/to/UBlox-NEO_M9N/include)

# Link the UBlox-NEO_M9N driver to your main application or other modules
target_link_libraries(your_main_target UBlox-NEO_M9N)