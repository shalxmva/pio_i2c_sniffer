/*
SPDX short identifier: BSD-3-Clause
BSD 3-Clause License

Copyright (c) 2025, Shalx

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"

#include "pio_i2c_sniffer.pio.h"
#include "pio_i2c_sniffer.h"


static int off1 = 0;
static volatile i2c_message_t queue[I2C_QUEUE_LEN] = {0}; // Array to hold received messages
static volatile uint8_t back_idx = 0;
static volatile uint8_t front_idx = 0;


static void i2c_start_stop_rx_irq(void)
{
    irq_set_enabled(I2C_SNIFFER_IRQ, false);

    if(pio_interrupt_get(I2C_SNIFFER_PIO_INSTANCE, 1))  //Check STOP IRQ
    {
        pio_sm_exec(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_DATA, pio_encode_jmp(off1));
        pio_interrupt_clear(I2C_SNIFFER_PIO_INSTANCE, 1);
        front_idx++;
        if(front_idx >= I2C_QUEUE_LEN) {
            front_idx = 0; 
        }
    } 
    else if(pio_interrupt_get(I2C_SNIFFER_PIO_INSTANCE, 0)) // Check START IRQ
    {
        pio_interrupt_clear(I2C_SNIFFER_PIO_INSTANCE, 0);
        queue[front_idx].length = 0; // Reset the message length for the new message
    }
    else
    {
        uint32_t data = pio_sm_get(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_DATA); // Read data from the PIO state machine
        uint8_t index = queue[front_idx].length;
        queue[front_idx].msg[index].ack = !(data&0x01); // Extract ACK/NACK bit
        queue[front_idx].msg[index].data = (data>>1); // Store the raw data
        queue[front_idx].length++;
    }

    irq_set_enabled(I2C_SNIFFER_IRQ, true);
}

static void enable_pio_interrupts()
{
    pio_set_irq0_source_enabled(I2C_SNIFFER_PIO_INSTANCE, pis_interrupt0, true);
    pio_set_irq0_source_enabled(I2C_SNIFFER_PIO_INSTANCE, pis_interrupt1, true);
    pio_set_irq0_source_enabled(I2C_SNIFFER_PIO_INSTANCE, pis_sm1_rx_fifo_not_empty, true);

    irq_set_exclusive_handler(I2C_SNIFFER_IRQ, i2c_start_stop_rx_irq);
    irq_set_enabled(I2C_SNIFFER_IRQ, true);
}

void i2c_sniffer_init() {
    enable_pio_interrupts();
    pio_claim_sm_mask(I2C_SNIFFER_PIO_INSTANCE, 15);

    pio_gpio_init(I2C_SNIFFER_PIO_INSTANCE, SDA_PIN);
    pio_gpio_init(I2C_SNIFFER_PIO_INSTANCE, SCL_PIN);
    gpio_disable_pulls(SDA_PIN);
    gpio_disable_pulls(SCL_PIN);

    // Disable PIO state machines
    pio_set_sm_mask_enabled(I2C_SNIFFER_PIO_INSTANCE, 0xF, false);

    int off0 = pio_add_program(I2C_SNIFFER_PIO_INSTANCE, &i2c_events_program);
    off1 = pio_add_program(I2C_SNIFFER_PIO_INSTANCE, &i2c_machine_program);


    pio_sm_config config0 = i2c_events_program_get_default_config(off0);
    pio_sm_config config1 = i2c_machine_program_get_default_config(off1);

    sm_config_set_in_shift(
        &config0,
        false,  // ShiftDir : true: shift ISR to right, false: shift ISR to left
        false,   // AutoPush : true: enabled, false: disabled
        10      // AutoPush threshold: <0-32>
    );

    sm_config_set_in_pins(&config0, SDA_PIN);
    sm_config_set_jmp_pin(&config0, SCL_PIN);
    sm_config_set_clkdiv(&config0, 1);
    pio_sm_set_config(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_EVENT, &config0);

    pio_sm_set_consecutive_pindirs(I2C_SNIFFER_PIO_INSTANCE, 0, SDA_PIN, 2, false);
    pio_sm_clear_fifos(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_EVENT);
    pio_sm_restart(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_EVENT);
 
    pio_sm_init(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_EVENT, off0, &config0);

    sm_config_set_in_shift(
        &config1,
        false,  // ShiftDir : true: shift ISR to right, false: shift ISR to left
        false,   // AutoPush : true: enabled, false: disabled
        10      // AutoPush threshold: <0-32>
    );

    sm_config_set_in_pins(&config1, SDA_PIN);
    sm_config_set_jmp_pin(&config1, SDA_PIN);
    sm_config_set_fifo_join(&config1, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&config1, 1);
    pio_sm_set_config(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_DATA, &config1);
    pio_sm_set_consecutive_pindirs(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_DATA, SDA_PIN, 2, false);
    pio_sm_clear_fifos(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_DATA);
    pio_sm_restart(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_DATA);
    pio_sm_init(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_DATA, off1, &config1);

    pio_sm_set_enabled(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_EVENT, true);
    pio_sm_set_enabled(I2C_SNIFFER_PIO_INSTANCE, I2C_SM_DATA,  true);
}

bool i2c_sniffer_get_message(i2c_message_t *msg)
{
    if(front_idx == back_idx) {
        return false; // No new message available
    }

    *msg = queue[back_idx]; // Copy the message to the provided pointer

    back_idx++;
    if(back_idx >= I2C_QUEUE_LEN) {
        back_idx = 0; // Reset to avoid overflow
    }
    
    return true; // Message successfully retrieved
}

void i2c_print_message(const i2c_message_t *msg)
{
    printf("Address:0x%02X%s %s[",
           msg->msg[0].address,
           msg->msg[0].rw ? "r" : "w",
           msg->msg[0].ack ? "a" : "n");
    
    for (uint8_t i = 1; i < msg->length; i++) {
        printf("%02X%s",
               msg->msg[i].data,
               msg->msg[i].ack ? "a" : "n");
    }
    printf("]\n");
}





