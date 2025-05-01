cmake_minimum_required(VERSION 3.10)
project(KukaRSI C)

set(CMAKE_C_STANDARD 99)
include_directories(include)

# Platform-specific libraries
if (WIN32)
    set(PLATFORM_LIBS ws2_32 winmm)
else()
    set(PLATFORM_LIBS pthread)
endif()

# RSI static library
add_library(kuka_rsi STATIC
    src/kuka_rsi.c
)
target_include_directories(kuka_rsi PUBLIC include)

# Monitor app
add_executable(monitor app/monitor.c)
target_link_libraries(monitor kuka_rsi ${PLATFORM_LIBS})

# Jogger app
add_executable(jogger app/jogger.c)
target_link_libraries(jogger kuka_rsi ${PLATFORM_LIBS})

# Optional flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
