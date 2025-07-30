/*
SPDX short identifier: BSD-3-Clause
BSD 3-Clause License

Copyright (c) 2025, Shalx
*/
#pragma once
typedef struct
{
    union{
        uint8_t data; // 8 bits for data
        struct
        {
            uint8_t rw : 1;      // 1 bit for Read/Write
            uint8_t address : 7; // 7 bits for address
        };
    };
    
    bool ack:1;  // 1 bit for ACK/NACK
    bool start:1;
} i2c_frame_t;

typedef struct 
{
    i2c_frame_t msg[I2C_MAX_FRAMES_PER_MESSAGE]; // Array of packets
    uint8_t length;
}i2c_message_t;

void i2c_sniffer_init();
bool i2c_sniffer_get_message(i2c_message_t *msg);
void i2c_print_message(const i2c_message_t *msg);