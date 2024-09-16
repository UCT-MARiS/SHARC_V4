set(WAVE_LIBS_DIR "${CMAKE_SOURCE_DIR}/WaveProc")


add_library(WaveProc STATIC
    ${WAVE_LIBS_DIR}/Src/wave_functions.c
)

target_include_directories(WaveProc PUBLIC
    ${WAVE_LIBS_DIR}/Inc
)

target_link_libraries(WaveProc
    SHARC::DSPFiltering
    SHARC::DSPFFT
    SHARC::DSPWindow
    SHARC::DSPFastMath
)