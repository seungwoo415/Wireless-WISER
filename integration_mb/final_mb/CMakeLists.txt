cmake_minimum_required(VERSION 3.20.0)
set(BOARD adafruit_feather_nrf52840) # added
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(final_mb)

target_sources(app PRIVATE 
    src/main.c
    src/ble.c 
    src/uart.c
    )
