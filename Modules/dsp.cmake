# Include only the filtering, FFT, and window function libraries
set(DSP_LIBS_DIR "${CMAKE_SOURCE_DIR}/Modules/CMSIS-DSP/Source")
set(DSP_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/Modules/CMSIS-DSP/Include")


add_library(DSPBasicMath STATIC
    ${DSP_LIBS_DIR}/BasicMathFunctions/arm_abs_f32.c
    ${DSP_LIBS_DIR}/BasicMathFunctions/arm_add_f32.c
    ${DSP_LIBS_DIR}/BasicMathFunctions/arm_mult_f32.c
    ${DSP_LIBS_DIR}/BasicMathFunctions/arm_scale_f32.c
    ${DSP_LIBS_DIR}/BasicMathFunctions/arm_offset_f32.c
    ${DSP_LIBS_DIR}/BasicMathFunctions/arm_clip_f32.c
    ${DSP_LIBS_DIR}/SupportFunctions/arm_q31_to_float.c
    ${DSP_LIBS_DIR}/SupportFunctions/arm_copy_f32.c
    ${DSP_LIBS_DIR}/ComplexMathFunctions/arm_cmplx_mag_f32.c
    ${DSP_LIBS_DIR}/ComplexMathFunctions/arm_cmplx_mag_squared_f32.c
    )

add_library(DSPFiltering STATIC
    ${DSP_LIBS_DIR}/FilteringFunctions/arm_fir_f32.c
    ${DSP_LIBS_DIR}/FilteringFunctions/arm_fir_init_f32.c
    ${DSP_LIBS_DIR}/FilteringFunctions/arm_fir_decimate_init_f32.c
    ${DSP_LIBS_DIR}/FilteringFunctions/arm_fir_decimate_f32.c
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

target_include_directories(DSPBasicMath PUBLIC 
${DSP_INCLUDE_DIR}
${DSP_INCLUDE_DIR}/dsp
${CMAKE_SOURCE_DIR}/Modules/CMSIS-Core/
)

target_include_directories(DSPFiltering PUBLIC 
${DSP_INCLUDE_DIR}
${DSP_INCLUDE_DIR}/dsp
${CMAKE_SOURCE_DIR}/Modules/CMSIS-Core/
)

target_include_directories(DSPFFT PUBLIC 
${DSP_INCLUDE_DIR}
${DSP_INCLUDE_DIR}/dsp
${CMAKE_SOURCE_DIR}/Modules/CMSIS-Core/
)

target_include_directories(DSPWindow PUBLIC 
${DSP_INCLUDE_DIR}
${DSP_INCLUDE_DIR}/dsp
${CMAKE_SOURCE_DIR}/Modules/CMSIS-Core/
)

target_include_directories(DSPFastMath PUBLIC 
${DSP_INCLUDE_DIR}
${DSP_INCLUDE_DIR}/dsp
${CMAKE_SOURCE_DIR}/Modules/CMSIS-Core/
)

add_library(SHARC::DSPBasicMath ALIAS DSPBasicMath)
add_library(SHARC::DSPFiltering ALIAS DSPFiltering)
add_library(SHARC::DSPFFT ALIAS DSPFFT)
add_library(SHARC::DSPWindow ALIAS DSPWindow)
add_library(SHARC::DSPFastMath ALIAS DSPFastMath)
