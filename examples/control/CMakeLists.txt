cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(control C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


add_executable(control
  main.c
)


target_compile_definitions(control PRIVATE
  RC_MAX_CHANNELS=3
  #RC_MAX_PULSE_WIDTH=1850
  #RC_MIN_PULSE_WIDTH=1000
)

add_subdirectory("../../../pico_rc" pico_rc)

# pull in common dependencies
target_link_libraries(control 
  pico_stdlib
  pico_rc
)

# create map/bin/hex file etc.
pico_add_extra_outputs(control)

	