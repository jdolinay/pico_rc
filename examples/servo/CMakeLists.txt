cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(servo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(servo
  main.c
)

# SPECIFY preprocessor definitions for this project
target_compile_definitions(servo PRIVATE
  RC_SERVO_MIN_PULSE=1100
  RC_SERVO_MAX_PULSE=1900  
)

# set the location of the pico_rc library
add_subdirectory("../../../pico_rc" pico_rc)

# pull in common dependencies and rc library
target_link_libraries(servo 
  pico_stdlib
  pico_rc
)

# create map/bin/hex file etc.
#pico_add_extra_outputs(servo)

	