/**
 * All macros for network addresses
 */
#ifndef _APP_CONF_H
#define _APP_CONF_H


/* RSSI dump thread port number */
#define RSSI_RIOT_PORT					    220
#define RSSI_MBED_DUMP_PORT         		9111

//On the mbed side 
#define INTER_THREAD						0xaa
#define SEND_RSSI_PUB						0xab

#define RESET_TIMEOUT						60000000

#endif



