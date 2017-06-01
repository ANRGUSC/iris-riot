# New Border Router and Server

This example includes a modified version of the gnrc_border_router that can
communicate via UDP.  Also included are two python scripts: 
	udp_server - which runs a server to connect to UDP capable motes or
	client scripts
	udp_client - a simple client program that can be used to send UDP data
	to the server

## Starting Up the Border Router and Server

Run the python script in a terminal and choose an open port to run on
(ex 8888). Flash the border router onto an openmote, and use ifconfig to find
the IPv6 global address in the localhost (ex fd00:dead:beef::1).

To connect the border router and the python server, send the start signal to
the python server:
  * udp send fd00:dead:beef::1 8888 $$START$$
with a general message syntax:
  * udp send <IPv6 Address> <Port> message

The $$START$$ signal registers the connecting client or mote's address in a
list in the server, as the server publishes the data it receives to all
registered addresses except the one that sent it.
> **Note:** Connect the border router with a start signal before trying to
connect any other motes to the border router.

## Using the Client Program

Run the python script in a terminal and choose the port used for the server.
Any string message can be sent in the terminal.

## Connecting Other Motes

Motes with UDP capabilities (currently only tested for anrg_networking) can
connect to the server via the border router.  With the border router running
and registered in the server, motes can send to the server using the udp send
command and the localhost global IPv6 address.

## Miscellaneous

After receiving a message from any source, the server sends an $$ACK$$ reply
acknowledging the transmission back to the source.  This is currently used by
the client script to tell it to stop sending a message, but could be
implemented to work with other sources as well.

The border router is based on the gnrc_border_router with the addition of
some of the modules of gnrc_networking to allow for UDP transmissions. The
README files for both examples have been included in the old README's file for
reference.