# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(picoScope C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(picoScope
               picoScope.c
               )

# enable usb output, disable uart output
pico_enable_stdio_usb(picoScope 1)
pico_enable_stdio_uart(picoScope 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(picoScope)

target_link_libraries(picoScope 
                      pico_stdlib
                      pico_cyw43_arch_none
                      hardware_adc
                      hardware_pwm
                      hardware_dma
                      pico_multicore
                      )
