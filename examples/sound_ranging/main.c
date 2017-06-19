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

#include "shell.h"
#include "msg.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);
extern int range_tx(int argc, char **argv);
extern int range_rx(int argc, char **argv);
extern int scan_tx(int argc, char **argv);
extern int scan_rx_start(int argc, char **argv);
extern int scan_rx_stop(int argc, char **argv);
extern int orient_rx(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { "range_tx", "act as the transmitter for sound ranging", range_tx},
    { "range_rx", "act as the receiver for sound ranging", range_rx},
    { "scan_tx", "act as the receiver for sound ranging", scan_tx},
    { "scan_rx_start", "starts a continuous adc read for ultrasound", scan_rx_start},
    { "scan_rx_stop", "stops a continuous adc read for ultrasound", scan_rx_stop},
    { "orient_rx", "starts an orientation reading", orient_rx},
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
    char *temp[1];
    temp[0] = "range_tx";
    range_tx(1, temp);

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}
