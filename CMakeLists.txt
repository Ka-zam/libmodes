# libmodes - Mode-S/ADS-B decoder library (dump1090 refactored)
# Ka-zam fork with dump1090-fa 2.4MHz demodulator

cmake_minimum_required(VERSION 3.14)
project(libmodes C)

# Option to enable 2.4MHz demodulator (default: ON)
option(LIBMODES_2400 "Enable 2.4MHz oversampling demodulator from dump1090-fa" ON)

# Core source files (legacy 2MHz demodulator + unified API)
set(LIBMODES_SOURCES
    src/mode-s.c
    src/decoder_api.c
)

# Additional source files for 2.4MHz demodulator
if(LIBMODES_2400)
    list(APPEND LIBMODES_SOURCES
        src/demod_2400.c
        src/crc.c
        src/icao_filter.c
    )
endif()

add_library(libmodes STATIC ${LIBMODES_SOURCES})

target_include_directories(libmodes PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(libmodes PRIVATE
    -Wall -Wextra
    $<$<CONFIG:Release>:-O2>
    $<$<CONFIG:Debug>:-g -O0>
)

# Define LIBMODES_2400 if enabled
if(LIBMODES_2400)
    target_compile_definitions(libmodes PUBLIC LIBMODES_2400=1)
    message(STATUS "libmodes: 2.4MHz demodulator enabled")
else()
    message(STATUS "libmodes: Using legacy 2MHz demodulator only")
endif()

# libmodes needs math library
target_link_libraries(libmodes PRIVATE m)
