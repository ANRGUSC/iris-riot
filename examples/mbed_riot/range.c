/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Yutong Gu
 * Richard Kim
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to deal
 * with the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * - Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimers.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimers in the 
 *     documentation and/or other materials provided with the distribution.
 * - Neither the names of Autonomous Networks Research Group, nor University of 
 *     Southern California, nor the names of its contributors may be used to 
 *     endorse or promote products derived from this Software without specific 
 *     prior written permission.
 * - A citation to the Autonomous Networks Research Group must be included in 
 *     any publications benefiting from the use of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH 
 * THE SOFTWARE.
 */
/**
 * @ingroup     examples
 * @{
 *
 * @file        range.c
 * @brief       Ultrasound ranging library for localization
 *
 * @author      Yutong Gu <yutonggu@usc.edu>
 *
 * @}
 */

#include "range.h"
#include "debug.h"

#define ENABLE_DEBUG (0)
#define OMNI_RANGING (0)

#define MAXSAMPLES_ONE_PIN            18000
#define MAXSAMPLES_TWO_PIN            18000

#define RX_ONE_PIN                    GPIO_PIN(3, 3) //aka GPIO_PD3 - maps to DIO0
#define RX_TWO_PIN                    GPIO_PIN(3, 2) //aka GPIO_PD2 - maps to DIO1

#define RX_LOGIC_PIN                  GPIO_PIN(3, 1) //aka GPIO_PD1 - maps to DIO2

#define TX_PIN                        GPIO_PIN(3, 2) //aka GPIO_PD2 - maps to DIO1 //for usb openmote
//#define TX_PIN                      GPIO_PIN(3, 0) //aka GPIO_PD0 - maps to DIO3  //for regular openmote

static range_data_t* time_diffs;

static gpio_rx_line_t gpio_lines = (gpio_rx_line_t){RX_ONE_PIN, RX_TWO_PIN, RX_LOGIC_PIN};


/**
 * @brief      This function gets the ranging data, packages them into packets, and sends them down the hdlc to the mbed
 *
 * @param      params    The ranging parameters
 * @param[in]  hdlc_pid  The hdlc pid
 */
void range_and_send(range_params_t *params, kernel_pid_t hdlc_pid, uint16_t src_port, uint16_t mbed_port){
    int i = 0;
    int num_iter = 0;
    int remainder = 0;
    int exit = 0;
    range_hdr_t range_hdr;
    uart_pkt_hdr_t uart_hdr_tx;
    int pkt_size = RANGE_DATA_LEN * DATA_PER_PKT + sizeof(uint8_t) + UART_PKT_HDR_LEN;
    char send_data[pkt_size];
    hdlc_pkt_t hdlc_snd_pkt =  { .data = send_data, .length = pkt_size};
    hdlc_pkt_t *hdlc_rcv_pkt;
    range_data_t* time_diffs;
    msg_t msg_snd, msg_rcv;

    num_iter = params->num_samples / DATA_PER_PKT;
    remainder = params->num_samples % DATA_PER_PKT;
    DEBUG("num_samples = %d\n",params->num_samples);
    DEBUG("num_iter = %d\n",num_iter);
    DEBUG("remainder = %d\n",remainder);
    DEBUG("data_per_pkt = %d\n", DATA_PER_PKT);

    range_hdr.last_pkt = 0;

    for(i = 0; i <= num_iter; i++){
        DEBUG("i= %d\n",i);
        
        uart_hdr_tx.src_port = src_port;
        uart_hdr_tx.dst_port = mbed_port;
        uart_hdr_tx.pkt_type = SOUND_RANGE_DONE;
        uart_pkt_insert_hdr(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &uart_hdr_tx);

        UART1->cc2538_uart_ctl.CTLbits.UARTEN = 0;

        if(i <= num_iter-1){

            hdlc_snd_pkt.length = RANGE_DATA_LEN*DATA_PER_PKT + sizeof(uint8_t) + UART_PKT_HDR_LEN;

            time_diffs = range_rx((uint32_t) RANGE_TIMEO_USEC, params->ranging_mode, DATA_PER_PKT);
            DEBUG("sampling %d\n",DATA_PER_PKT);

            if(i == num_iter-1 && remainder == 0){
                DEBUG("message complete\n");
                range_hdr.last_pkt = 1;
                i++; 
            }
            memcpy(&range_hdr.data, time_diffs, RANGE_DATA_LEN * DATA_PER_PKT);

            uart_pkt_cpy_data(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &range_hdr, sizeof(uint8_t) + RANGE_DATA_LEN*DATA_PER_PKT);
            
        } else{
            if(remainder != 0){ 
                hdlc_snd_pkt.length = RANGE_DATA_LEN * remainder + sizeof(uint8_t) + UART_PKT_HDR_LEN;

                time_diffs = range_rx((uint32_t) RANGE_TIMEO_USEC, params->ranging_mode, remainder);

                DEBUG("sampling %d\n",remainder);
                DEBUG("message complete\n");
    
                range_hdr.last_pkt = 1;
                memcpy(&range_hdr.data, time_diffs, RANGE_DATA_LEN * remainder);

                uart_pkt_cpy_data(hdlc_snd_pkt.data, hdlc_snd_pkt.length, &range_hdr, sizeof(uint8_t) + RANGE_DATA_LEN*remainder);
                
            } 
        }
        
        UART1->cc2538_uart_ctl.CTLbits.RXE = 1;
        UART1->cc2538_uart_ctl.CTLbits.TXE = 1;
        UART1->cc2538_uart_ctl.CTLbits.HSE = UART_CTL_HSE_VALUE;
        UART1->cc2538_uart_dr.ECR = 0xFF;
        UART1->cc2538_uart_lcrh.LCRH &= ~FEN;
        UART1->cc2538_uart_lcrh.LCRH |= FEN;
        UART1->cc2538_uart_ctl.CTLbits.UARTEN = 1;

        if(time_diffs == NULL){
            DEBUG("An error occured while ranging\n");
            break;
        }
            
            //sending the data back down the hdlc

        DEBUG("Sending data back down hdlc\n");
        

        msg_snd.type = HDLC_MSG_SND;
        msg_snd.content.ptr = &hdlc_snd_pkt;
        if(!msg_try_send(&msg_snd, hdlc_pid)) {
            /* TODO: use xtimer_msg_receive_timeout() instead */
            /* this is where applications can decide on a timeout */
            DEBUG("HDLC busy retrying...\n");
            msg_rcv.type = HDLC_RESP_RETRY_W_TIMEO;
            msg_rcv.content.value = RTRY_TIMEO_USEC;
            msg_send_to_self(&msg_rcv);
        }

        while(1)
        {
            msg_receive(&msg_rcv);

            switch (msg_rcv.type)
            {
                case HDLC_RESP_SND_SUCC:
                    DEBUG("Successfully sent pkt\n");
                    exit = 1;
                    break;
                case HDLC_RESP_RETRY_W_TIMEO:
                    xtimer_usleep(msg_rcv.content.value);
                    DEBUG("Range thread: retrying\n");
                    if(!msg_try_send(&msg_snd, hdlc_pid)) {
                        DEBUG("Range thread: HDLC msg queue full!\n");
                        msg_send_to_self(&msg_rcv);
                    }
                    break;
                case HDLC_PKT_RDY:
                    hdlc_rcv_pkt = (hdlc_pkt_t *) msg_rcv.content.ptr;
                    DEBUG("Range thread: received pkt while trying to send\n");
                    hdlc_pkt_release(hdlc_rcv_pkt);
                    break;
                default:
                    /* error */
                    LED3_ON;
                    break;
            }

            if(exit) {
                exit = 0;
                break;
            }
        }
        free(time_diffs);
    }
}

range_data_t* range_rx(uint32_t timeout_usec, uint8_t range_mode, uint16_t num_samples){ 
    // Check correct argument usage.
    uint8_t mode = range_mode;
    uint32_t maxsamps; //number of iterations in the gpio polling loop before calling it a timeout
    
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


        range_rx_init(TX_NODE_ID, thread_getpid(), gpio_lines, maxsamps, mode);


        if(xtimer_msg_receive_timeout(&msg,timeout)<0){
            DEBUG("RF ping missed\n");
            range_rx_stop();
            time_diffs[i] = (range_data_t) {0, 0, RF_MISSED};
            continue;
        }

        if(msg.type == RF_RCVD){
            if(xtimer_msg_receive_timeout(&msg, ULTRSND_TIMEOUT) < 0){
                DEBUG("Ultrsnd ping missed\n");
                range_rx_stop();
                time_diffs[i] = (range_data_t) {0, 0, ULTRSND_MISSED};
                continue;
            }
            if(msg.type == ULTRSND_RCVD){
                time_diffs[i] = *(range_data_t*) msg.content.ptr;
            } else{
                range_rx_stop();
                i--;
                continue;
            }

        }

        DEBUG("range: TDoA = %d\n", time_diffs[i].tdoa);
        switch (range_mode){
            case ONE_SENSOR_MODE:
                break;

            case TWO_SENSOR_MODE:
                if(time_diffs[i].status > 2){
                    DEBUG("range: Missed pin %d\n", MISSED_PIN_UNMASK - time_diffs[i].status);
                } else{
                    DEBUG("range: OD = %d\n", time_diffs[i].orient_diff);
                }
                break;

            case XOR_SENSOR_MODE:
                DEBUG("range: OD = %d\n", time_diffs[i].orient_diff);
                break;
            case OMNI_SENSOR_MODE:
                break;
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
    
    range_tx_off(); //turn off just in case
    DEBUG("RF and ultrasound pings sent\n");  

    return 0;
}
