if(CMAKE_CROSSCOMPILING)
	include(stm32l4cube.cmake)
	include(rtos.cmake)
endif()

if (NOT CMAKE_CROSSCOMPILING)
	include(gtest.cmake)
endif()