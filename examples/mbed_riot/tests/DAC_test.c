

#include <stdio.h>

#include "shell.h"
#include "msg.h"

#include "xtimer.h"
#include "dac.h"

#define MAIN_QUEUE_SIZE     (8)

/**
 * This is the chip select pin for the DAC. This corresponds to PA3 or DIO4.
 */
#define SPI_DAC_CS               GPIO_PIN(0,3) 

static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];


int write_voltage(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "writev", "write a value over SPI to a DAC", write_voltage},
    { NULL, NULL, NULL }
};

int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    printf("RIOT network stack example application");

    /* start shell */
    printf("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    /* auto-run */
    // char *temp[3];
    // temp[0] = "range_rx";
    // temp[1] = "50";          //num pkts
    // temp[2] = "1000000";     //interval_in_us
    // range_rx(3, temp);

    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}

int write_voltage(int argc, char **argv){
    if(argc < 2){
        printf("usage: %s <value>\n", argv[0]);
        return 0;
    }

    uint32_t value = atoi(argv[1]);

    if( value < 0 || value > 255){
        printf("Error: value must be between 0 and 255");
        return 0;
    }
    
    if(init_dac(SPI_DAC_CS, SPI_CLK_400KHZ) == SPI_OK){
        printf("Setting voltage to %d%%\n", value*100/255);
        set_voltage((uint8_t) value, DAC_GAIN_1);
        stop_dac();
        return 1;
    } else{
        printf("SPI failed\n");
        return 0;
    }
}




