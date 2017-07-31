/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Example application for demonstrating the RIOT network stack
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>

#include "shell.h"
#include "msg.h"

#include "dac.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int range_rx(int argc, char **argv);
extern int range_tx(int argc, char **argv);
int write_voltage(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "range_tx", "act as the transmitter for sound ranging", range_tx},
    { "range_rx", "act as the receiver for sound ranging", range_rx},
    { "writev", "write a value over SPI to a DAC", write_voltage},
    { NULL, NULL, NULL }
};

int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");

    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    /* auto-run */
    char *temp[2];
    temp[0] = "writev";
    temp[1] = "40";
    write_voltage(2, temp);

    //char *temp[2];
    // temp[0] = "range_scan_tx";
    // temp[1] = "100000";
    // range_scan_tx(2, temp);
    //reboot();

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}


int write_voltage(int argc, char **argv){
    if(argc < 2)
    {
        printf("usage: %s <value>\n", argv[0]);
        return 0;
    }

    uint32_t value = atoi(argv[1]);

    if( value < 0 || value > 255){
        printf("Error: value must be between 0 and 255");
        return 0;
    }
    
    if(init_dac(DEFAULT_DAC_CS, SPI_CLK_400KHZ) == SPI_OK){
        printf("Setting voltage to %lu%%\n", value*100/255);
        set_voltage((uint8_t) value, DAC_GAIN_1);
        stop_dac();
        return 1;

    } else{
        printf("SPI failed\n");
        return 0;
    }
}