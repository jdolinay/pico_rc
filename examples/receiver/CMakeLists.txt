cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(receiver C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


add_executable(receiver
  main.c
)


target_compile_definitions(receiver PRIVATE
  RC_MAX_CHANNELS=8
  #RC_MAX_PULSE_WIDTH=1850
  #RC_MIN_PULSE_WIDTH=1000
)

add_subdirectory("../../../pico_rc" pico_rc)

# pull in common dependencies
target_link_libraries(receiver 
  pico_stdlib  
  pico_rc
)

# create map/bin/hex file etc.
#pico_add_extra_outputs(pulseio)

	