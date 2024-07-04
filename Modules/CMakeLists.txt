if(CMAKE_CROSSCOMPILING)
	include(stm32l4cube.cmake)
	include(rtos.cmake)
endif()

include(gtest.cmake)
