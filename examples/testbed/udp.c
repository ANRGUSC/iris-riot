
#include <stdio.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

static gnrc_netreg_entry_t server = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, 
                                        KERNEL_PID_UNDEF};

int udp_send(char* data, int data_length, uint16_t port, ipv6_addr_t* address)
{

    gnrc_pktsnip_t *payload, *udp, *ip;
    
    // Allocate payload 
    payload = gnrc_pktbuf_add(NULL, data, data_length, GNRC_NETTYPE_UNDEF);
       
    if (payload == NULL) 
    {
        DEBUG("Error: Unable to copy data to packet buffer");
        return 1;
    }

    // Allocate UDP header
    udp = gnrc_udp_hdr_build(payload, port, port);

    if (udp == NULL) 
    {
        DEBUG("Error: Unable to allocate UDP header");
        gnrc_pktbuf_release(payload);
        return 2;
    }
        
    // Allocate IPv6 header
    ip = gnrc_ipv6_hdr_build(udp, NULL, &address);

    if (ip == NULL) 
    {
        DEBUG("Error: Unable to allocate IPv6 header");
        gnrc_pktbuf_release(udp);
        return 3;
    }

    // Send packet

    if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
        DEBUG("Error: Unable to locate UDP thread");
        gnrc_pktbuf_release(ip);
        return 4;
    }
	
	return 0;
}



void udp_start_server(uint16_t port) 
{
    // Debug if server is already running
    if (server.target.pid != KERNEL_PID_UNDEF) 
    {
        DEBUG("Error: Server already running on port\n");
        return;
    }

    // Start server 
    server.next = NULL;
    server.demux_ctx = (uint32_t) port; 
    server.pid = thread_getpid();
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
}


void udp_stop_server(void)
{
    // Debug if server is running
    if (server.target.pid == KERNEL_PID_UNDEF) 
    {
        DEBUG("Error: Server was not running\n");
        return;
    }

    // Stop server
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &server);
    server.target.pid = KERNEL_PID_UNDEF;
}



int parse_udp_packet(msg_t *msg, char* data)
{
	
}