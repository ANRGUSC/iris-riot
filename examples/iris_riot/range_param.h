#ifndef RANGE_PARAM_H
#define RANGE_PARAM_H

#define TX_POWER             7       //valid range: -27 to 7

#define QUEUE_SIZE          8

#define MAX_ADDR_LEN        (8U)

#define TX_NODE_ID 			0x01

#define RANGE_RX_HW_ADDR    "ff:ff"

#define DEFAULT_ULTRASOUND_THRESH           45
#define MAX_SOUND_SAMPLES                   500

/* TDMA Localization Settings */
/* sent in network byte order (big-endian) */
#define TDMA_ANCHOR_ID_REQ_U16_FLAG         0x5444  /* 'T' and 'D' for TDma */
#define TDMA_ANCHOR_ID_RESP_U16_FLAG        0x4d41  /* 'M' and 'A' for tdMA */
#define TDMA_SLOT_TIME_USEC                 200000

#define TDMA_TOTAL_ANCHOR_NODES             3       /* experiment specific */

#define TDMA_BOOTSTRAP_CHANNEL              11
#define RSSI_LOCALIZATION_CHAN              26

#endif
