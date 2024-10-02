# fatfs.cmake

# Set the base directory for FATFS sources
set(FATFS_DIR "${CMAKE_SOURCE_DIR}/Modules/FATFS")

# List of FATFS source files
set(FATFS_SOURCES
    "${FATFS_DIR}/App/fatfs.c"
    "${FATFS_DIR}/Target/sd_diskio.c"
    "${FATFS_DIR}/Target/bsp_driver_sd.c"
    "${FATFS_DIR}/FatFs_12/src/ff.c"
    "${FATFS_DIR}/FatFs_12/src/diskio.c"
    "${FATFS_DIR}/FatFs_12/src/ff_gen_drv.c"
    "${FATFS_DIR}/FatFs_12/src/option/syscall.c"
    # Add any additional FATFS source files if needed
)

# Include directories for FATFS
set(FATFS_INCLUDE_DIRS
    "${FATFS_DIR}/FatFs_12/src"
    "${FATFS_DIR}/App"
    "${FATFS_DIR}/Target"
    # Add any additional include directories if needed
)

# Create a static library target for FATFS
add_library(fatfs STATIC ${FATFS_SOURCES})

# Set the include directories for the fatfs target
target_include_directories(fatfs PUBLIC ${FATFS_INCLUDE_DIRS})

# Optionally, add compile definitions based on your configuration
# For example, enable long file name support and set maximum LFN length
target_compile_definitions(fatfs PUBLIC
    _USE_LFN=0       # Disable long file name support ("Static LFN work area cannot be used at thread-safe configuration")
    # Note max length of Short File Name is 13 characters
    _FS_REENTRANT=1  # Enable reentrancy (thread safety) if needed
)

# FATFS depends on the RTOS library
target_link_libraries(fatfs 
                    PUBLIC SHARC::RTOS
                    PUBLIC SHARC::HAL
)


add_library(SHARC::FATFS ALIAS fatfs)
