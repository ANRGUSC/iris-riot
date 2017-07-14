/*````````````````````````````````````````````````````````````````````````````````````
 * Copyright (C) 2016 Autonomous Networks Research Group
 *                     University of Southern California
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
 * @brief       Simple program example to test ultrasound ranging.
 *
 * @author      Jason A. Tran <jasontra@usc.edu>
 *
 * @}
 */

#include "range.h"
#define ENABLE_DEBUG (1)
#include "debug.h"

#define MAXSAMPLES_ONE_PIN            18000
#define MAXSAMPLES_TWO_PIN            18000

#define RX_ONE_PIN                    GPIO_PIN(3, 3) //aka GPIO_PD3 - maps to DIO0
#define RX_TWO_PIN                    GPIO_PIN(3, 2) //aka GPIO_PD2 - maps to DIO1
#define RX_XOR_PIN                    GPIO_PIN(3, 1) //aka GPIO_PD1 - maps to DIO2

#define TX_PIN                        GPIO_PIN(3, 2) //aka GPIO_PD2 - maps to DIO1 //for usb openmote
//#define TX_PIN                      GPIO_PIN(3, 0) //aka GPIO_PD0 - maps to DIO3 



//static unsigned int gpio_lines[]={GPIO_PIN(3, 3), GPIO_PIN(3, 2), GPIO_PIN(3, 1)};


static range_data_t* time_diffs;


range_data_t* range_rx(uint32_t timeout_usec, uint8_t range_mode, uint16_t num_samples){ 
    // Check correct argument usage.
    uint8_t mode = range_mode;
    uint32_t maxsamps; //number of iterations in the gpio polling loop before calling it a timeout
                       //
    gpio_rx_line_t lines = (gpio_rx_line_t){RX_ONE_PIN, RX_TWO_PIN, RX_XOR_PIN};
    
    if(mode == TWO_SENSOR_MODE){
        maxsamps = MAXSAMPLES_TWO_PIN;
    } else {
        maxsamps = MAXSAMPLES_ONE_PIN;
    }

    if(gpio_init(TX_PIN, GPIO_OUT) < 0) {
        DEBUG("Error initializing GPIO_PIN.\n");
        return 1;
    }
    // clearing output for the ultrasonic sensor
    gpio_clear(TX_PIN);

    time_diffs = malloc(sizeof(range_data_t)*num_samples);
    
    uint32_t timeout = timeout_usec;
    if(timeout <= 0){
        DEBUG("timeout must be greater than 0");
        return NULL;
    }

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

   
    int i;
    for(i = 0; i < num_samples; i++){


        range_rx_init(TX_NODE_ID, thread_getpid(), lines, maxsamps, mode);

block:
        if(xtimer_msg_receive_timeout(&msg,timeout)<0){
            DEBUG("RF ping missed\n");
            return NULL;
        }

        if(msg.type == RF_RCVD){
            if(xtimer_msg_receive_timeout(&msg,timeout)<0){
                DEBUG("Ultrsnd ping missed\n");
                return NULL;
            }
            if(msg.type == ULTRSND_RCVD){
                time_diffs[i] = *(range_data_t*) msg.content.ptr;
            } else{
                goto block;
            }

        }
        if(time_diffs[i].tdoa > 0){
            printf("range: TDoA = %d\n", time_diffs[i].tdoa);
            switch (range_mode){
                case ONE_SENSOR_MODE:
                    break;

                case TWO_SENSOR_MODE:
                    if(time_diffs[i].error!=0){
                        printf("range: Missed pin %d\n", time_diffs[i].error);
                    } else{
                        printf("range: OD = %d\n", time_diffs[i].orient_diff);
                    }
                    break;

                case XOR_SENSOR_MODE:
                    printf("range: OD = %d\n", time_diffs[i].orient_diff);
                    break;
            }
            if(i == num_samples-1){
                time_diffs[i].error += 10;
            }
        }
        else{
            printf("Ultrsnd ping missed\n");
        }


    }

    return time_diffs;
}

/*----------------------------------------------------------------------------*/
int range_tx( void )
{

    /* for sending L2 pkt */
    kernel_pid_t dev;
    uint8_t hw_addr[MAX_ADDR_LEN];
    size_t hw_addr_len;
    gnrc_pktsnip_t *pkt, *hdr;
    gnrc_netif_hdr_t *nethdr;
    uint8_t flags = 0x00;    
    char buf[3] = {0x00, 0x00, 0x00};

    int16_t tx_power = TX_POWER;

    kernel_pid_t ifs[GNRC_NETIF_NUMOF];
    size_t numof = gnrc_netif_get(ifs); 

    /* there should be only one network interface on the board */
    if (numof == 1) {
        gnrc_netapi_set(ifs[0], NETOPT_TX_POWER, 0, &tx_power, sizeof(int16_t));
    }

    /* enable output on Port D pin 3 */
    if(gpio_init(TX_PIN, GPIO_OUT) < 0) {
        DEBUG("Error initializing GPIO_PIN.\n");
        return 1;
    }
    // clearing output for the ultrasonic sensor
    gpio_clear(TX_PIN);

    range_tx_init(TX_PIN);
    // Broadcasting flag setup.
    flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
    
    /** Send L2 Packet **/
    /* network interface */
    dev = ifs[0];
    hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);
    
    /* put packet together */
    buf[0] = RANGE_FLAG_BYTE0;
    buf[1] = RANGE_FLAG_BYTE1;
    buf[2] = TX_NODE_ID;
    pkt = gnrc_pktbuf_add(NULL, &buf, 3, GNRC_NETTYPE_UNDEF);
    if (pkt == NULL) {
        DEBUG("error: packet buffer full\n");
        return 1;
    }
   
    hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
    if (hdr == NULL) {
        DEBUG("error: packet buffer full\n");
        gnrc_pktbuf_release(pkt);
        return 1;
    }
    LL_PREPEND(pkt, hdr);
    nethdr = (gnrc_netif_hdr_t *)hdr->data;
    nethdr->flags = flags;
    /* ready to send */

    //make sure no packets are to be sent!!
    if (gnrc_netapi_send(dev, pkt) < 1) {
        DEBUG("error: unable to send\n");
        gnrc_pktbuf_release(pkt);
        return 1;
    }   
    //gnrc_pktbuf_release(pkt);
    range_tx_off(); //turn off just in case
    DEBUG("RF and ultrasound pings sent\n");  

    return 0;
}
