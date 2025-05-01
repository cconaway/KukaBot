cmake_minimum_required(VERSION 3.10)
project(KukaRSI C)

# Set C standard
set(CMAKE_C_STANDARD 99)

# Include headers
include_directories(include)

# Source files
set(RSI_SOURCES
        src/kuka_rsi.c
)

# Common dependencies for Windows or Unix
if (WIN32)
    set(PLATFORM_LIBS ws2_32 winmm)
else()
    set(PLATFORM_LIBS pthread)
endif()

# === MONITOR APP ===
add_executable(monitor
        app/monitor.c
        ${RSI_SOURCES}
)

target_link_libraries(monitor ${PLATFORM_LIBS})