#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "timex.h"
#include "xtimer.h"

#include "thread.h"
#include "msg.h"

#define QUEUE_SIZE 16
#define PORT_NUM "8888"
#define IPV6_ADDR "fd00:dead:beef::2"

static gnrc_netreg_entry_t receiver = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, 
										{KERNEL_PID_UNDEF}};

extern int udp_cmd(int argc, char **argv);

/* this is used to unregister the thread from receiving UDP packets sent to the 
   port in the "receiver" struct */
static void _unregister_thread(void)
{
	gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &receiver);
	receiver.target.pid = KERNEL_PID_UNDEF;
}

//Check if any udp message received on port and check against parameter message
void *udp_rx(void *arg)
{
	msg_t msg;
	msg_t msg_queue[QUEUE_SIZE];
	//Uncomment if find need to send messages to main
	//kernel_pid_t pid = *(kernel_pid_t*)arg;
	printf("Entered function\n");

	//Setup the message queue
	msg_init_queue(msg_queue, QUEUE_SIZE);


	//Register thread to chosen UDP port
	receiver.next = NULL;
	receiver.demux_ctx = atoi(PORT_NUM);
	receiver.target.pid = thread_getpid();
	gnrc_netreg_register(GNRC_NETTYPE_UDP, &receiver);
	gnrc_pktsnip_t *pkt;
	gnrc_pktsnip_t *snip;
	char *word;

	//Variables to send to udp_cmd to register with server
    //For some reason the server seems to need a couple acks before registering
    char *temp[5];
    temp[0] = "udp";
  	temp[1] = "send";
    temp[2] = IPV6_ADDR;
    temp[3] = PORT_NUM;
    temp[4] = "$$ACK$$";


	while(1)
	{
		//Block until message received
		msg_receive(&msg);
		printf("Data received\n");

		pkt = msg.content.ptr;

		switch(msg.type)
		{
			//If right type of data received, check if start signal or send
			//to main thread
			case GNRC_NETAPI_MSG_TYPE_RCV:
				snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
				word = snip->data;
				//Needed because garbage character seems to be added to message
				//word[strlen(word)-1] = 0;
				//if statement runs if two strings not equal
				if(strcmp(word, "$$START$$"))
				{
					printf("Message received: %s\n", word);
				}
				else
				{
					udp_cmd(5, temp);
    				printf("Start signal received\n");
				}
				break;
			default:
				printf("Received unexpected data\n");
				break;
		}
		gnrc_pktbuf_release(pkt);
	}
	//Shouldn't run
	_unregister_thread();
	return NULL;
}