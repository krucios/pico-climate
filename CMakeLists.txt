cmake_minimum_required(VERSION 3.13)

# Must always be before project()
include(pico_sdk_import.cmake)

project(home_climate_control_device)

pico_sdk_init()

add_executable(climate_control_device
    main.c
)
pico_enable_stdio_usb(climate_control_device 1)
pico_add_extra_outputs(climate_control_device)

add_executable(i2c_scan
    i2c_scan.c
)
pico_enable_stdio_usb(i2c_scan 1)
pico_add_extra_outputs(i2c_scan)

add_subdirectory(bme680)
add_subdirectory(bsec)

target_link_libraries(climate_control_device pico_stdlib hardware_i2c bsec)
target_link_libraries(i2c_scan pico_stdlib hardware_i2c)