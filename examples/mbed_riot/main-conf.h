/**
 * All macros for network addresses
 */

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

#define ARREST_FOLLOWER_IPV6_ADDR           "fe80::212:4b00:433:ece1"

#define RANGE_REQ_FLAG                      0x12
#define RANGE_RDY_FLAG                      0x34
#define RANGE_GO_FLAG                       0x56

#define DEFAULT_ULTRASOUND_THRESH           50
#define MAX_SOUND_SAMPLES                   500

#define REMOTE_CTRL_FLAG                    0x99 /* 10011001 */

#define RANGE_REQ_TIMEO_USEC                1000000





