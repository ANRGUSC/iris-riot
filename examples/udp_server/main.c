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
#include "cc2538_rf.h"

#define MAIN_QUEUE_SIZE     (8)
#define PORT_NUM 8888
#define POWER_LEVEL -20

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);
extern int udp_rx(int port, char *message);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

int main(void)
{
    //Set status to nonzero and not error code (1)
    int status = -1;

    //Change radio broadcast power
    cc2538_set_tx_power(POWER_LEVEL);
    int power = cc2538_get_tx_power();
    printf("Power: %d\n", power);

    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT border router example application");

    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    //Automatically register the border router with the server
    char *temp[5];
    temp[0] = "udp";
    temp[1] = "send";
    temp[2] = "fd00:dead:beef::2";
    temp[3] = "8888";
    temp[4] = "$$ACK$$";

    while(status != 0)
    {
        status = udp_rx(PORT_NUM, "$$START$$");
    }
    udp_cmd(5, temp);

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}
