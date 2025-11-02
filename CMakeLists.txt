cmake_minimum_required(VERSION 3.27)

# Initialize the Pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(app)
pico_sdk_init()
add_executable(app
    src/wrappers.c
)

if (STDIO_USB)
    pico_enable_stdio_uart(app 0)
    pico_enable_stdio_usb(app 1)
endif()

if (STDIO_UART)
    pico_enable_stdio_uart(app 1)
    pico_enable_stdio_usb(app 0)
endif()

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/config     # Where you place lwipopts.h
    # ${PICO_SDK_PATH}/lib/lwip
    # Other pico-sdk includes...
)

# Adjust libraries as needed
target_link_libraries(app
    pico_stdlib
    hardware_adc
    pico_cyw43_arch_lwip_threadsafe_background
    ${CMAKE_SOURCE_DIR}/zig-out/lib.o
    # wrappers
    -Wl,--wrap=gpioc_lo_out_put
    -Wl,--wrap=gpioc_lo_out_xor
    -Wl,--wrap=gpioc_lo_out_set
    -Wl,--wrap=gpioc_lo_out_clr
    -Wl,--wrap=gpioc_hi_out_put
    -Wl,--wrap=gpioc_hi_out_xor
    -Wl,--wrap=gpioc_hi_out_set
    -Wl,--wrap=gpioc_hi_out_clr
    -Wl,--wrap=gpioc_hilo_out_put
    -Wl,--wrap=gpioc_hilo_out_xor
    -Wl,--wrap=gpioc_hilo_out_set
    -Wl,--wrap=gpioc_hilo_out_clr
    -Wl,--wrap=gpioc_lo_oe_put
    -Wl,--wrap=gpioc_lo_oe_xor
    -Wl,--wrap=gpioc_lo_oe_set
    -Wl,--wrap=gpioc_lo_oe_clr
    -Wl,--wrap=gpioc_hi_oe_put
    -Wl,--wrap=gpioc_hi_oe_xor
    -Wl,--wrap=gpioc_hi_oe_set
    -Wl,--wrap=gpioc_hi_oe_clr
    -Wl,--wrap=gpioc_hilo_oe_put
    -Wl,--wrap=gpioc_hilo_oe_xor
    -Wl,--wrap=gpioc_hilo_oe_set
    -Wl,--wrap=gpioc_hilo_oe_clr
    -Wl,--wrap=gpioc_bit_out_put
    -Wl,--wrap=gpioc_bit_out_xor
    -Wl,--wrap=gpioc_bit_out_set
    -Wl,--wrap=gpioc_bit_out_clr
    -Wl,--wrap=gpioc_bit_out_xor2
    -Wl,--wrap=gpioc_bit_out_set2
    -Wl,--wrap=gpioc_bit_out_clr2
    -Wl,--wrap=gpioc_bit_oe_put
    -Wl,--wrap=gpioc_bit_oe_xor
    -Wl,--wrap=gpioc_bit_oe_set
    -Wl,--wrap=gpioc_bit_oe_clr
    -Wl,--wrap=gpioc_bit_oe_xor2
    -Wl,--wrap=gpioc_bit_oe_set2
    -Wl,--wrap=gpioc_bit_oe_clr2
    -Wl,--wrap=gpioc_index_out_put
    -Wl,--wrap=gpioc_index_out_xor
    -Wl,--wrap=gpioc_index_out_set
    -Wl,--wrap=gpioc_index_out_clr
    -Wl,--wrap=gpioc_index_oe_put
    -Wl,--wrap=gpioc_index_oe_xor
    -Wl,--wrap=gpioc_index_oe_set
    -Wl,--wrap=gpioc_index_oe_clr
    -Wl,--wrap=gpioc_lo_out_get
    -Wl,--wrap=gpioc_hi_out_get
    -Wl,--wrap=gpioc_hilo_out_get
    -Wl,--wrap=gpioc_lo_oe_get
    -Wl,--wrap=gpioc_hi_oe_get
    -Wl,--wrap=gpioc_hilo_oe_get
    -Wl,--wrap=gpioc_lo_in_get
    -Wl,--wrap=gpioc_hi_in_get
    -Wl,--wrap=gpioc_hilo_in_get
)

# Generate binary
pico_add_extra_outputs(app)
