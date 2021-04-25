cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

set(projname "StepperMotor")

project(${projname} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

include_directories(lib/StepperMotor_lib)

#Add stepper_motor library
add_library(StepperMotor_lib INTERFACE)
target_sources(StepperMotor_lib INTERFACE /src/StepperMotor.c)
target_include_directories(StepperMotor_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
#Link pico_stdlib to stepper_motor library
target_link_libraries(StepperMotor_lib INTERFACE pico_stdlib)

add_executable(${projname}
	main.c
	lib/StepperMotor_lib/src/StepperMotor.c
	)

target_link_libraries(${projname}
	pico_stdlib
	pico_multicore
	)

pico_enable_stdio_usb(${projname} 1)
pico_enable_stdio_uart(${projname} 0)

pico_add_extra_outputs(${projname})
