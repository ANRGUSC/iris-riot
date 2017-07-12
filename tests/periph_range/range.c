/*````````````````````````````````````````````````````````````````````````````````````
 * Copyright (C) 2016 Autonomous Networks Research Group
 *                     University of Southern California
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**

 *
 * @file
 * @brief       Program example to test ultrasound ranging.
 *
 * @author      Yutong Gu <yutonggu@usc.edu
 *
 * 
 */

#include "range.h"
#define ENABLE_DEBUG (1)
#include "debug.h"

#define MAXSAMPLES_ONE_PIN            18000
#define MAXSAMPLES_TWO_PIN            18000

#define RX_ONE_PIN                    GPIO_PIN(3, 3)
#define RX_TWO_PIN                    GPIO_PIN(3, 2)
#define RX_XOR_PIN                    GPIO_PIN(3, 1)


//#define TX_PIN                        GPIO_PIN(3, 0)
#define TX_PIN                        GPIO_PIN(3, 2) //for usb openmote




//static unsigned int gpio_lines[]={GPIO_PIN(3, 3), GPIO_PIN(3, 2), GPIO_PIN(3, 1)};
static range_data_t* time_diffs;

/*----------------------------------------------------------------------------*/
int range_rx(int argc, char **argv)
{
    // Check correct argument usage.
    if(argc != 4){
        printf("usage: %s <num_samples> <delay_usecs> <ranging_mode>\n", argv[0]);
        return 1;
    }
    
    gpio_rx_line_t line = (gpio_rx_line_t){RX_ONE_PIN, RX_TWO_PIN, RX_XOR_PIN};

    uint32_t maxsamps = 0;
    uint32_t timeout = 500000;
    uint32_t num_samps = atoi(argv[1]);
    uint32_t delay = atoi(argv[2]);
    uint8_t mode = atoi(argv[3]);
    

    if(delay < 0){
        printf("Error: delay_usecs must be greater than or equal to 0");
        return 1;
    }
    if(num_samps <= 0){
        printf("Error: num_samples must be greater than 0");
        return 1;
    }


    switch (mode){
        case 0:
            mode = ONE_SENSOR_MODE;
            printf("ONE_SENSOR_MODE:\n");
            break;
        case 1:
            mode = TWO_SENSOR_MODE;
            printf("TWO_SENSOR_MODE:\n");
            break;
        case 2:
            mode = XOR_SENSOR_MODE;
            printf("XOR_SENSOR_MODE:\n");
            break;
        default:
            printf("Invalid ranging mode entry\nValid entries are:\n0: ONE_SENSOR_MODE\n1: TWO_SENSOR_MODE\n2: XOR_SENSOR_MODE\n");
            return 1;
    }

    msg_t msg; 
    msg_t msg_queue[QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, QUEUE_SIZE);

   int i = 0;
   for(i = 0; i < num_samps; i++){
        printf("Trial %d of %lu:\n", i, num_samps);
        if(mode == TWO_SENSOR_MODE){
            maxsamps = MAXSAMPLES_TWO_PIN;
        } else {
            maxsamps = MAXSAMPLES_ONE_PIN;
        }

        range_rx_init(TX_NODE_ID, thread_getpid(), line, maxsamps, mode);

block:
        if(xtimer_msg_receive_timeout(&msg,timeout)<0){
            printf("RF ping missed\n");
            continue;
        }
        _unregister_thread();

        if(msg.type == 143){
            if(xtimer_msg_receive_timeout(&msg,timeout)<0){
                printf("Ultrsnd ping missed\n");
                continue;
            }
            if(msg.type == 144){
                time_diffs = (range_data_t*) msg.content.ptr;
            } else{
                goto block;
            }

        }
        if(time_diffs->tdoa > 0){
            printf("range: TDoA = %d\n", time_diffs->tdoa);
            switch (mode){
                case ONE_SENSOR_MODE:
                    break;

                case TWO_SENSOR_MODE:
                    if(time_diffs->error!=0){
                        printf("range: OD failed - missed pin %d\n", time_diffs->error);
                    } else{
                        printf("range: OD = %d\n", time_diffs->orient_diff);
                    }
                    break;

                case XOR_SENSOR_MODE:
                    printf("range: OD = %d\n", time_diffs->orient_diff);
                    break;
            }
        } else{
            printf("Ultrsnd ping missed\n");
        }
        xtimer_usleep(delay);
    }
    

    return 0;
}

/*----------------------------------------------------------------------------*/
int range_tx(int argc, char **argv)
{

    // Check correct argument usage.
    if (argc < 2) {
        printf("usage: %s <delay_usecs>\n",argv[0]);
        return 1;
    }

    if(atoi(argv[1]) < 100000){
        printf("error: delay must be a minimum of 100ms");
        return 1;
    }

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

    while(true){
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
    }
<<<<<<< 61582a63084e4f226c43285358f7efd0cc71993b
=======

>>>>>>> fixed periph_range test to work with changes in last commit
    return 0;
}
