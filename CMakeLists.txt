#[[
SPDX short identifier: BSD-3-Clause
BSD 3-Clause License

Copyright (c) 2025, Shalx
]]

add_library(pio_i2c_sniffer STATIC ${CMAKE_CURRENT_LIST_DIR}/pio_i2c_sniffer.c)

target_compile_definitions(pio_i2c_sniffer PRIVATE -DI2C_SNIFFER_PIO_INSTANCE=pio0)  # Define the PIO instance for the I2C sniffer
target_compile_definitions(pio_i2c_sniffer PRIVATE -DI2C_SNIFFER_IRQ=PIO0_IRQ_0)     # Define the PIO instance and IRQ for the I2C sniffer
target_compile_definitions(pio_i2c_sniffer PUBLIC  -DI2C_MAX_FRAMES_PER_MESSAGE=16)  # Define the maximum number of frames per message
target_compile_definitions(pio_i2c_sniffer PUBLIC  -DI2C_QUEUE_LEN=16)  # Define the maximum number of frames per message
set(I2C_SNIFFER_SDA_PIN 0)
set(I2C_SNIFFER_SCL_PIN 1)

target_include_directories(pio_i2c_sniffer PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(pio_i2c_sniffer PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

configure_file(${CMAKE_CURRENT_LIST_DIR}/pio_i2c_sniffer.pio.in pio_i2c_sniffer.pio @ONLY)
pico_generate_pio_header(pio_i2c_sniffer ${CMAKE_CURRENT_BINARY_DIR}/pio_i2c_sniffer.pio)
target_link_libraries(pio_i2c_sniffer PRIVATE pico_stdlib hardware_pio)