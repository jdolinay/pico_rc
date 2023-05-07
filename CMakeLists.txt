if (NOT TARGET pico_rc_headers)
    add_library(pico_rc_headers INTERFACE)
    
	 # Pull in PICO SDK (must be before project)
    #include(pico_sdk_import.cmake)
    #if (PICO_SDK_VERSION_STRING VERSION_LESS "1.2.0")
    #    message(FATAL_ERROR "Require at least Raspberry Pi Pico SDK version 1.2.0")
    #endif()

	target_include_directories(pico_rc_headers INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
    #target_link_libraries(pico_rc_headers INTERFACE hardware_timer_headers pico_sync_headers pico_util_headers)
	
endif()

	#if (NOT PICO_RC_PATH)
    #    set(PICO_RC_PATH ${CMAKE_CURRENT_LIST_DIR})
    #endif()
    #set(PICO_RC_PATH "${PICO_RC_PATH}" CACHE PATH "Path to Pico RC library")



if (NOT TARGET pico_rc)
    pico_add_impl_library(pico_rc)

    target_sources(pico_rc INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/rc.c
    )
    target_link_libraries(pico_rc INTERFACE hardware_pwm hardware_clocks)
endif()
