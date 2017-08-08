#ifndef RANGE_PARAM_H
#define RANGE_PARAM_H

#define TX_POWER             7       //valid range: -27 to 7
#define TX_NODE_IPV6_ADDR   "fe80::212:4b00:613:622" //fe80::212:4b00:433:ed81"

#define CLIENT_PORT         8000
#define SERVER_PORT         8888

#define MAX_ADDR_LEN        (8U)
#define MAIN_QUEUE_SIZE     (8U)

#define RANGE_RX_HW_ADDR    "ff:ff"
#define QUEUE_SIZE          8
#define CC2538_RSSI_OFFSET  73

#define TX_NODE_ID 			0x00

#define DEFAULT_ULTRASOUND_THRESH           45
#define MAX_SOUND_SAMPLES                   500

#define REMOTE_CTRL_FLAG                    0x99 /* 10011001 */

#define RANGE_REQ_TIMEO_USEC                1000000

/* TDMA Localization Settings */
/* sent in network byte order (big-endian) */
#define TDMA_ANCHOR_ID_REQ_U16_FLAG         0x5444  /* 'T' and 'D' for TDma */
#define TDMA_ANCHOR_ID_RESP_U16_FLAG        0x4d41  /* 'M' and 'A' for tdMA */
#define TDMA_SLOT_TIME_USEC                 100000
#define TDMA_BOOTSTRAP_CHANNEL              11
#define TDMA_TOTAL_ANCHOR_NODES             3       /* experiment specific */

#endif /* RANGE_PARAM_H */