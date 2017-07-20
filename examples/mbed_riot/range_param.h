#ifndef RANGE_PARAM_H
#define RANGE_PARAM_H

#define TX_POWER            7
#define TX_NODE_IPV6_ADDR   "fe80::212:4b00:613:622" //fe80::212:4b00:433:ed81"

#define CLIENT_PORT         8000
#define SERVER_PORT         8888

#define MAX_ADDR_LEN        (8U)
#define RANGE_RX_HW_ADDR    "ff:ff"
#define QUEUE_SIZE          8
#define CC2538_RSSI_OFFSET  73

#define RANGE_REQ_FLAG      0x12 // 18
#define RANGE_RDY_FLAG      0x34 // 52
#define RANGE_GO_FLAG       0x56 // 86

#define TX_NODE_ID 0x00

#define LEADER_HW_ADDR                	ff:ff // TODO: Replace with the correct MAC address of the leader.

#define LEAD_REQ						0x10  //pray that these aren't taken
#define LEAD_ACK	                    0x85    
#define FOLLOW_ASSIGN                   'MA'




#define FOLLOW_SYNC                     0x32    

#define FOLLOW_GO                       0x76
#define LEAD_INFO                       0X98

// #define BUFFER_SIZE_OF_PACKET           3       // size of buf that holds address and other data.
// #define BUFFER_SIZE_OF_PACKET_LEADER    4       // size of buf that holds address and other data.

#endif


// void _send_message(uint8_t *buf[BUFFER_SIZE_OF_PACKET])
// {
//     /** Send L2 Packet **/
//     /* network interface */
//     dev = ifs[0];
//     hw_addr_len = gnrc_netif_addr_from_str(hw_addr, sizeof(hw_addr), RANGE_RX_HW_ADDR);
    
//     /* put packet together */
//     pkt = gnrc_pktbuf_add(NULL, buf, 3, GNRC_NETTYPE_UNDEF);
//     if (pkt == NULL) {
//         DEBUG("error: packet buffer full\n");
//         return 1;
//     }
   
//     hdr = gnrc_netif_hdr_build(NULL, 0, hw_addr, hw_addr_len);
//     if (hdr == NULL) {
//         DEBUG("error: packet buffer full\n");
//         gnrc_pktbuf_release(pkt);
//         return 1;
//     }
//     LL_PREPEND(pkt, hdr);
//     nethdr = (gnrc_netif_hdr_t *)hdr->data;
//     nethdr->flags = flags;
//     /* ready to send */
    
//     //make sure no packets are to be sent!!
//     if (gnrc_netapi_send(dev, pkt) < 1) {
//         DEBUG("error: unable to send\n");
//         gnrc_pktbuf_release(pkt);
//         return 1;
//     }   
// }



















