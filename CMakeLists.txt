# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Prefer an existing local SDK installation if present; otherwise fall back to
# fetching a copy into the project. This avoids CMake add_subdirectory errors
# when the fetched copy references files outside the expected source tree.
set(local_pico_sdk_path "${USERHOME}/.pico-sdk/sdk/${sdkVersion}")
if(EXISTS ${local_pico_sdk_path})
    message(STATUS "Using local Pico SDK at ${local_pico_sdk_path}")
    set(PICO_SDK_FETCH_FROM_GIT OFF CACHE BOOL "Fetch Pico SDK from GitHub if no local SDK is available" FORCE)
    set(PICO_SDK_PATH "${local_pico_sdk_path}" CACHE PATH "Path to the Raspberry Pi Pico SDK" FORCE)
else()
    set(PICO_SDK_FETCH_FROM_GIT ON CACHE BOOL "Fetch Pico SDK from GitHub if no local SDK is available" FORCE)
    set(PICO_SDK_FETCH_FROM_GIT_PATH "${CMAKE_CURRENT_LIST_DIR}/.pico-sdk" CACHE FILEPATH "Location for the Pico SDK checkout" FORCE)
    set(PICO_SDK_FETCH_FROM_GIT_TAG "${sdkVersion}" CACHE STRING "Pico SDK tag" FORCE)
    unset(ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "" CACHE PATH "Path to the Raspberry Pi Pico SDK" FORCE)
endif()
# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(rp2040 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(rp2040 
                        src/main.c
                      src/hal/mpu/mpu.c
                      src/hal/rx/crossfire.c
                      src/core/attitude.c
                      src/control/pid.c
                      src/control/mixer.c
                      src/hal/esc/dshot.c
                      src/hal/gps/gps.c
                      src/hal/battery/battery.c
                      src/telemetry/telemetry.c
                      src/debug/axis_debug.c
    )

pico_generate_pio_header(rp2040 ${CMAKE_CURRENT_LIST_DIR}/src/hal/esc/dshot.pio)

pico_set_program_name(rp2040 "rp2040")
pico_set_program_version(rp2040 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rp2040 0)
pico_enable_stdio_usb(rp2040 1)

# Add the standard library to the build
target_link_libraries(rp2040
        pico_stdlib
        hardware_spi
        hardware_dma
        hardware_pio
        hardware_uart
        hardware_adc)

# Add the standard include files to the build
target_include_directories(rp2040 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
)

pico_add_extra_outputs(rp2040)

