#include <stdio.h>
#include <inttypes.h>

#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "timex.h"
#include "xtimer.h"

#include "thread.h"
#include "msg.h"

#define QUEUE_SIZE 8

static gnrc_netreg_entry_t receiver = { NULL, GNRC_NETREG_DEMUX_CTX_ALL, 
<<<<<<< HEAD
										{KERNEL_PID_UNDEF}};
=======
										KERNEL_PID_UNDEF};
//use for testing purposes
//extern int udp_cmd(int argc, char **argv);
>>>>>>> develop

/* this is used to unregister the thread from receiving UDP packets sent to the 
   port in the "receiver" struct */
static void _unregister_thread(void)
{
	gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &receiver);
	receiver.target.pid = KERNEL_PID_UNDEF;
}

//Check if any udp message received on port and check against parameter message
int udp_rx(int port, char *message)
{
	msg_t msg;
	msg_t msg_queue[QUEUE_SIZE];
	printf("Entered function\n");

	//Setup the message queue
	msg_init_queue(msg_queue, QUEUE_SIZE);


	//Register thread to chosen UDP port
	receiver.next = NULL;
	receiver.demux_ctx = port;
	receiver.target.pid = thread_getpid();
	gnrc_netreg_register(GNRC_NETTYPE_UDP, &receiver);
	gnrc_pktsnip_t *pkt;
	gnrc_pktsnip_t *snip;
	char *word;

<<<<<<< HEAD
	if(xtimer_msg_receive_timeout(&msg, 1000000) < 0)
=======
	/*if(xtimer_msg_receive_timeout(&msg, 1000000) < 0)
>>>>>>> develop
	{
		printf("System Timeout: Not connected to server\n");
		_unregister_thread();
		return 1;
<<<<<<< HEAD
	}
=======
	}*/
	msg_receive(&msg);
>>>>>>> develop

	pkt = msg.content.ptr;

	if(msg.type == GNRC_NETAPI_MSG_TYPE_RCV)
	{
		snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_UNDEF);
		word = snip->data;
<<<<<<< HEAD
=======
		//word[strlen(word)-1] = 0;
		printf("%s of length %d\n", word, strlen(word));
>>>>>>> develop
	}
	else
	{
		printf("Received unexpected data\n");
		gnrc_pktbuf_release(pkt);
		_unregister_thread();
		return 1;
	}

	if(strcmp(word, message))	//Two not equal because returns 0 if equal
	{
		printf("Data received not identical to requested data\n");
		gnrc_pktbuf_release(pkt);
		_unregister_thread();
		return 1;
	}
	else
	{
		gnrc_pktbuf_release(pkt);
		_unregister_thread();
		return 0;
	}

}