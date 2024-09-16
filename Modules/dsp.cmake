# Include only the filtering, FFT, and window function libraries
set(DSP_LIBS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP/Source")

add_library(DSPFiltering STATIC
    ${DSP_LIBS_DIR}/FilteringFunctions/arm_fir_f32.c
    ${DSP_LIBS_DIR}/FilteringFunctions/arm_fir_init_f32.c
    # Add other filtering source files as needed
)

add_library(DSPFFT STATIC
    ${DSP_LIBS_DIR}/TransformFunctions/arm_cfft_f32.c
    ${DSP_LIBS_DIR}/TransformFunctions/arm_cfft_init_f32.c  
    ${DSP_LIBS_DIR}/TransformFunctions/arm_rfft_fast_f32.c
    ${DSP_LIBS_DIR}/TransformFunctions/arm_rfft_fast_init_f32.c
    ${DSP_LIBS_DIR}/TransformFunctions/arm_cfft_radix8_f32.c
    ${DSP_LIBS_DIR}/TransformFunctions/arm_bitreversal2.c
    ${DSP_LIBS_DIR}/CommonTables/arm_common_tables.c
    ${DSP_LIBS_DIR}/CommonTables/arm_const_structs.c
    # Add other FFT source files as needed
)


add_library(DSPFastMath STATIC
    ${DSP_LIBS_DIR}/FastMathFunctions/arm_sin_f32.c
    ${DSP_LIBS_DIR}/FastMathFunctions/arm_cos_f32.c
    # Add other fast math source files as needed
)


add_library(DSPWindow STATIC
    ${DSP_LIBS_DIR}/WindowFunctions/arm_hamming_f32.c
    ${DSP_LIBS_DIR}/WindowFunctions/arm_hanning_f32.c
    # Add other window function source files as needed
)

target_include_directories(DSPFiltering PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP/Include
${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-Core/
)

target_include_directories(DSPFFT PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-Core/
${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP/Include/dsp
${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP/Include
)

target_include_directories(DSPWindow PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-Core/
${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP/Include)

target_include_directories(DSPFastMath PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-Core/
${CMAKE_CURRENT_SOURCE_DIR}/CMSIS-DSP/Include)

add_library(SHARC::DSPFiltering ALIAS DSPFiltering)
add_library(SHARC::DSPFFT ALIAS DSPFFT)
add_library(SHARC::DSPWindow ALIAS DSPWindow)
add_library(SHARC::DSPFastMath ALIAS DSPFastMath)
