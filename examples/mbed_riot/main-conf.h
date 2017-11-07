/**
 * All macros for network addresses
 */

/* MQTT Related Ports */
#define MBED_MQTT_PORT                      200
#define RIOT_MQTT_PORT                      170
#define MAIN_THR_PORT                       165
#define NULL_PKT_TYPE                       0xFF


/* RSSI dump thread port number */
#define RSSI_DUMP_PORT                      9000

#define GET_SET_RANGING_THR_PORT            9100

/* for ARREST, set follower's short hwaddr here */
#define ARREST_FOLLOWER_SHORT_HWADDR        "e7:e9"
#define ARREST_LEADER_SHORT_HWADDR          "ed:81"
#define ARREST_LDR_CTRL_SHORT_HWADDR        "ed:4f"

#define RSSI_LOCALIZATION_CHAN              26
#define ARREST_DATA_CHANNEL                 25

#define ARREST_LEADER_LONG_HWADDR           "00:12:4b:00:04:33:ed:81"
#define ARREST_LEADER_SOUNDRF_IPV6_ADDR     "fe80::212:4b00:433:ed5e"

#define ARREST_LEADER_SOUNDRF_PORT          9200

#define ARREST_LEADER_SOUNDRF_ID            170

#define ARREST_FOLLOWER_RANGE_THR_PORT      9300

#define ARREST_FOLLOWER_IPV6_ADDR           "fe80::212:4b00:433:eca3"

#define RANGE_REQ_FLAG                      0x12
#define RANGE_RDY_FLAG                      0x34
#define RANGE_GO_FLAG                       0x56

#define DEFAULT_ULTRASOUND_THRESH           45
#define MAX_SOUND_SAMPLES                   500

#define REMOTE_CTRL_FLAG                    0x99 /* 10011001 */

#define RANGE_REQ_TIMEO_USEC                1000000

/* TDMA Localization Settings */
/* sent in network byte order (big-endian) */
#define TDMA_ANCHOR_ID_REQ_U16_FLAG         0x5444  /* 'T' and 'D' for TDma */
#define TDMA_ANCHOR_ID_RESP_U16_FLAG        0x4d41  /* 'M' and 'A' for tdMA */

#define TDMA_SLOT_TIME_USEC                 200000
#define TDMA_BOOTSTRAP_CHANNEL              11
#define TDMA_TOTAL_ANCHOR_NODES             4       /* experiment specific */

#define ID_LENGTH       					9






