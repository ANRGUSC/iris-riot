
#include <stdio.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"

// #define ENABLE_DEBUG (0)
// #include "debug.h"

static gnrc_netreg_entry_t server = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, 
                                        KERNEL_PID_UNDEF};

int udp_send(char* data, int data_length, uint16_t port, ipv6_addr_t* address);
void udp_start_server(uint16_t port); 
void udp_stop_server(void);
int parse_udp_packet(msg_t *msg, char* data);