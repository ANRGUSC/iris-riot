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
#include "xtimer.h"
#include "thread.h"
#include "cc2538_rf.h"

#define MAIN_QUEUE_SIZE     (16)
#define POWER_LEVEL -20

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];
static char rx_thread_stack[THREAD_STACKSIZE_MAIN];

extern int udp_cmd(int argc, char **argv);
extern void *udp_rx(void *arg);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

int main(void)
{

    //Change radio broadcast power
    cc2538_set_tx_power(POWER_LEVEL);

    //Get the main thread pid to send to udp_rx thread
    //kernel_pid_t p = thread_getpid();
    //kernel_pid_t *arg = &p;
    
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");

    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    //Create thread to receive messages and deal with start signal
    thread_create(rx_thread_stack, sizeof(rx_thread_stack), 
        THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, udp_rx, /*arg*/NULL,
        "udp_rx");

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}
