/*
 *  lan.c
 *
 *  Author: Kestutis Bivainis
 *
 *  Adapted from:
 *  http://we.easyelectronics.ru/electro-and-pc/podklyuchenie-mikrokontrollera-k-lokalnoy-seti.html
 */

#include "lan.h"
#include "enc28j60.h"
#include "counter.h"
#include <string.h>

// MAC address
static uint8_t mac_addr[6] = MAC_ADDR;

// IP address/mask/gateway
#ifndef WITH_DHCP
  static uint32_t ip_addr = IP_ADDR;
  static uint32_t ip_mask = IP_SUBNET_MASK;
  static uint32_t ip_gateway = IP_DEFAULT_GATEWAY;
#endif

#define ip_broadcast (ip_addr | ~ip_mask)


// Packet buffer
uint8_t net_buf[ENC28J60_MAXFRAME];


// ARP cache
static uint8_t arp_cache_wr;
static arp_cache_entry_t arp_cache[ARP_CACHE_SIZE];

// TCP connection pool
#ifdef WITH_TCP
  tcp_state_t tcp_pool[TCP_MAX_CONNECTIONS];
#endif

// DHCP
#ifdef WITH_DHCP
  dhcp_status_code_t dhcp_status;
  static uint32_t dhcp_server;
  static uint32_t dhcp_renew_time;
  static uint32_t dhcp_retry_time;
  static uint32_t dhcp_transaction_id;
  static uint32_t ip_addr;
  static uint32_t ip_mask;
  static uint32_t ip_gateway;
#endif


// Function prototypes
void eth_send(eth_frame_t *frame, uint16_t len);
void eth_reply(eth_frame_t *frame, uint16_t len);
void eth_resend(eth_frame_t *frame, uint16_t len);

uint8_t *arp_resolve(uint32_t node_ip_addr);

uint8_t ip_send(eth_frame_t *frame, uint16_t len);
void ip_reply(eth_frame_t *frame, uint16_t len);
void ip_resend(eth_frame_t *frame, uint16_t len);

uint16_t ip_cksum(uint32_t sum, uint8_t *buf, uint16_t len);


/*
 * DHCP
 */

#ifdef WITH_DHCP

#define dhcp_add_option(ptr, optcode, type, value) \
  ((dhcp_option_t*)ptr)->code = optcode; \
  ((dhcp_option_t*)ptr)->len = sizeof(type); \
  *(type*)(((dhcp_option_t*)ptr)->data) = value; \
  ptr += sizeof(dhcp_option_t) + sizeof(type); \
  if(sizeof(type)&1) *(ptr++) = 0;

void dhcp_filter(eth_frame_t *frame, uint16_t len) {

  ip_packet_t *ip = (void*)(frame->data);
  udp_packet_t *udp = (void*)(ip->data);
  dhcp_message_t *dhcp = (void*)(udp->data);
  dhcp_option_t *option;
  uint8_t *op, optlen;
  uint32_t offered_net_mask = 0, offered_gateway = 0;
  uint32_t lease_time = 0, renew_time = 0, renew_server = 0;
  uint8_t type = 0;
  uint32_t temp;

  // Check if DHCP messages directed to us
  if( (len >= sizeof(dhcp_message_t)) &&
    (dhcp->operation == DHCP_OP_REPLY) &&
    (dhcp->transaction_id == dhcp_transaction_id) &&
    (dhcp->magic_cookie == DHCP_MAGIC_COOKIE) ) {

    len -= sizeof(dhcp_message_t);

    // parse DHCP message
    op = dhcp->options;
    while(len >= sizeof(dhcp_option_t)) {
      option = (void*)op;
      if(option->code == DHCP_CODE_PAD) {
        op++;
        len--;
      }
      else if(option->code == DHCP_CODE_END) {
        break;
      }
      else {
        switch(option->code) {
          case DHCP_CODE_MESSAGETYPE:
            type = *(option->data);
            break;
          case DHCP_CODE_SUBNETMASK:
            offered_net_mask = *(uint32_t*)(option->data);
            break;
          case DHCP_CODE_GATEWAY:
            offered_gateway = *(uint32_t*)(option->data);
            break;
          case DHCP_CODE_DHCPSERVER:
            renew_server = *(uint32_t*)(option->data);
            break;
          case DHCP_CODE_LEASETIME:
            temp = *(uint32_t*)(option->data);
            lease_time = ntohl(temp);
            if(lease_time > 21600)
              lease_time = 21600;
            break;
          /*case DHCP_CODE_RENEWTIME:
            temp = *(uint32_t*)(option->data);
            renew_time = ntohl(temp);
            if(renew_time > 21600)
              renew_time = 21600;
            break;*/
        }

        optlen = sizeof(dhcp_option_t) + option->len;
        op += optlen;
        len -= optlen;
      }
    }

    if(!renew_server)
      renew_server = ip->from_addr;

    switch(type) {
      // DHCP offer?
      case DHCP_MESSAGE_OFFER:
        //printf("DHCP Offer\r\n");
        if((dhcp_status == DHCP_WAITING_OFFER) &&
           (dhcp->offered_addr != 0)) {

          dhcp_status = DHCP_WAITING_ACK;

          // send DHCP request
          ip->to_addr = inet_addr(255,255,255,255);

          udp->to_port = DHCP_SERVER_PORT;
          udp->from_port = DHCP_CLIENT_PORT;

          op = dhcp->options;
          dhcp_add_option(op, DHCP_CODE_MESSAGETYPE, uint8_t, DHCP_MESSAGE_REQUEST);
          dhcp_add_option(op, DHCP_CODE_REQUESTEDADDR, uint32_t, dhcp->offered_addr);
          dhcp_add_option(op, DHCP_CODE_DHCPSERVER,  uint32_t, renew_server);
          *(op++) = DHCP_CODE_END;

          dhcp->operation = DHCP_OP_REQUEST;
          dhcp->offered_addr = 0;
          dhcp->server_addr = 0;
          dhcp->flags = DHCP_FLAG_BROADCAST;

          udp_send(frame, (uint8_t*)op - (uint8_t*)dhcp);
        }
        break;

      // DHCP ack?
      case DHCP_MESSAGE_ACK:
        //printf("DHCP Ack\r\n");
        if((dhcp_status == DHCP_WAITING_ACK) &&
           (lease_time)) {

          if(!renew_time)
            renew_time = lease_time/2;

          dhcp_status = DHCP_ASSIGNED;
          dhcp_server = renew_server;
          dhcp_renew_time = rtime() + renew_time;
          dhcp_retry_time = rtime() + lease_time;

          // network up
          ip_addr = dhcp->offered_addr;
          ip_mask = offered_net_mask;
          ip_gateway = offered_gateway;

          //printf("IP:%d.%d.%d.%d\r\n",(uint8_t)(ip_addr),(uint8_t)(ip_addr>>8),(uint8_t)(ip_addr>>16),(uint8_t)(ip_addr>>24));
          //printf("Mask:%d.%d.%d.%d\r\n",(uint8_t)(ip_mask),(uint8_t)(ip_mask>>8),(uint8_t)(ip_mask>>16),(uint8_t)(ip_mask>>24));
          //printf("Gateway:%d.%d.%d.%d\r\n",(uint8_t)(ip_gateway),(uint8_t)(ip_gateway>>8),(uint8_t)(ip_gateway>>16),(uint8_t)(ip_gateway>>24));
        }
        break;
    }
  }
}

void dhcp_poll() {

  eth_frame_t *frame = (void*)net_buf;
  ip_packet_t *ip = (void*)(frame->data);
  udp_packet_t *udp = (void*)(ip->data);
  dhcp_message_t *dhcp = (void*)(udp->data);
  uint8_t *op;

  // Too slow (
  /*// Link is down
  if(!(enc28j60_read_phy(PHSTAT1) & PHSTAT1_LLSTAT))
  {
    dhcp_status = DHCP_INIT;
    dhcp_retry_time = rtime() + 2;

    // network down
    ip_addr = 0;
    ip_mask = 0;
    ip_gateway = 0;

    return;
  }*/

  // time to initiate DHCP
  //  (startup/lease end)
  if(rtime() >= dhcp_retry_time) {

    dhcp_status = DHCP_WAITING_OFFER;
    dhcp_retry_time = rtime() + 15;
    dhcp_transaction_id = gettc() + (gettc() << 16);

    // network down
    ip_addr = 0;
    ip_mask = 0;
    ip_gateway = 0;

    // send DHCP discover
    ip->to_addr = inet_addr(255,255,255,255);

    udp->to_port = DHCP_SERVER_PORT;
    udp->from_port = DHCP_CLIENT_PORT;

    memset(dhcp, 0, sizeof(dhcp_message_t));
    dhcp->operation = DHCP_OP_REQUEST;
    dhcp->hw_addr_type = DHCP_HW_ADDR_TYPE_ETH;
    dhcp->hw_addr_len = 6;
    dhcp->transaction_id = dhcp_transaction_id;
    dhcp->flags = DHCP_FLAG_BROADCAST;
    memcpy(dhcp->hw_addr, mac_addr, 6);
    dhcp->magic_cookie = DHCP_MAGIC_COOKIE;

    op = dhcp->options;
    dhcp_add_option(op, DHCP_CODE_MESSAGETYPE,
      uint8_t, DHCP_MESSAGE_DISCOVER);
    *(op++) = DHCP_CODE_END;

    udp_send(frame, (uint8_t*)op - (uint8_t*)dhcp);
  }

  // time to renew lease
  if((rtime() >= dhcp_renew_time) &&
     (dhcp_status == DHCP_ASSIGNED)) {

    dhcp_transaction_id = gettc() + (gettc() << 16);

    // send DHCP request
    ip->to_addr = dhcp_server;

    udp->to_port = DHCP_SERVER_PORT;
    udp->from_port = DHCP_CLIENT_PORT;

    memset(dhcp, 0, sizeof(dhcp_message_t));
    dhcp->operation = DHCP_OP_REQUEST;
    dhcp->hw_addr_type = DHCP_HW_ADDR_TYPE_ETH;
    dhcp->hw_addr_len = 6;
    dhcp->transaction_id = dhcp_transaction_id;
    dhcp->client_addr = ip_addr;
    memcpy(dhcp->hw_addr, mac_addr, 6);
    dhcp->magic_cookie = DHCP_MAGIC_COOKIE;

    op = dhcp->options;
    dhcp_add_option(op, DHCP_CODE_MESSAGETYPE,
      uint8_t, DHCP_MESSAGE_REQUEST);
    dhcp_add_option(op, DHCP_CODE_REQUESTEDADDR,
      uint32_t, ip_addr);
    dhcp_add_option(op, DHCP_CODE_DHCPSERVER,
      uint32_t, dhcp_server);
    *(op++) = DHCP_CODE_END;

    if(!udp_send(frame, (uint8_t*)op - (uint8_t*)dhcp)) {
      dhcp_renew_time = rtime() + 5;
      return;
    }

    dhcp_status = DHCP_WAITING_ACK;
  }
}

#endif


/*
 * TCP (ver. 3.0)
 * lots of indian bydlocode here
 *
 * History:
 *  1.0 first attempt
 *  2.0 second attempt, first suitable working variant
 *  2.1 added normal seq/ack management
 *  3.0 added rexmit feature
 */

#ifdef WITH_TCP

// packet sending mode
tcp_sending_mode_t tcp_send_mode;

// "ack sent" flag
uint8_t tcp_ack_sent;

// send TCP packet
// must be set manually:
//  - tcp.flags
uint8_t tcp_xmit(tcp_state_t *st, eth_frame_t *frame, uint16_t len) {

  uint8_t status = 1;
  uint16_t temp, plen = len;

  ip_packet_t *ip = (void*)(frame->data);
  tcp_packet_t *tcp = (void*)(ip->data);

  if(tcp_send_mode == TCP_SENDING_SEND) {
    // set packet fields
    ip->to_addr = st->remote_addr;
    ip->from_addr = ip_addr;
    ip->protocol = IP_PROTOCOL_TCP;
    tcp->to_port = st->remote_port;
    tcp->from_port = st->local_port;
  }

  if(tcp_send_mode == TCP_SENDING_REPLY) {
    // exchange src/dst ports
    temp = tcp->from_port;
    tcp->from_port = tcp->to_port;
    tcp->to_port = temp;
  }

  if(tcp_send_mode != TCP_SENDING_RESEND) {
    // fill packet header ("static" fields)
    tcp->window = htons(TCP_WINDOW_SIZE);
    tcp->urgent_ptr = 0;
  }

  if(tcp->flags & TCP_FLAG_SYN) {
    // add MSS option (max. segment size)
    tcp->data_offset = (sizeof(tcp_packet_t) + 4) << 2;
    tcp->data[0] = 2;//option: MSS
    tcp->data[1] = 4;//option len
    tcp->data[2] = TCP_SYN_MSS>>8;
    tcp->data[3] = TCP_SYN_MSS&0xff;
    plen = 4;
  }
  else {
    tcp->data_offset = sizeof(tcp_packet_t) << 2;
  }

  // set stream pointers
  tcp->seq_num = htonl(st->seq_num);
  tcp->ack_num = htonl(st->ack_num);

  // set checksum
  plen += sizeof(tcp_packet_t);
  tcp->cksum = 0;
  tcp->cksum = ip_cksum(plen + IP_PROTOCOL_TCP, (uint8_t*)tcp - 8, plen + 8);

  // send packet
  switch(tcp_send_mode) {
    case TCP_SENDING_SEND:
      status = ip_send(frame, plen);
      tcp_send_mode = TCP_SENDING_RESEND;
      break;
    case TCP_SENDING_REPLY:
      ip_reply(frame, plen);
      tcp_send_mode = TCP_SENDING_RESEND;
      break;
    case TCP_SENDING_RESEND:
      ip_resend(frame, plen);
      break;
  }

  // advance sequence number
  st->seq_num += len;
  if( (tcp->flags & TCP_FLAG_SYN) || (tcp->flags & TCP_FLAG_FIN) )
    st->seq_num++;

  // set "ACK sent" flag
  if( (tcp->flags & TCP_FLAG_ACK) && (status) )
    tcp_ack_sent = 1;

  return status;
}

// sending SYN to peer
// return: 0xff - error, other value - connection id (not established)
uint8_t tcp_open(uint32_t addr, uint16_t port, uint16_t local_port) {

  eth_frame_t *frame = (void*)net_buf;
  ip_packet_t *ip = (void*)(frame->data);
  tcp_packet_t *tcp = (void*)(ip->data);
  tcp_state_t *st = 0, *pst;
  uint8_t id;
  uint32_t seq_num;

  // search for free conection slot
  for(id = 0; id < TCP_MAX_CONNECTIONS; ++id) {
    pst = tcp_pool + id;
    if(pst->status == TCP_CLOSED) {
      st = pst;
      break;
    }
  }

  // free connection slot found
  if(st) {
    // add new connection
    seq_num = gettc() + (gettc() << 16);

    st->status = TCP_SYN_SENT;
    st->event_time = gettc();
    st->seq_num = seq_num;
    st->ack_num = 0;
    st->remote_addr = addr;
    st->remote_port = port;
    st->local_port = local_port;

#ifdef WITH_TCP_REXMIT
    st->is_closing = 0;
    st->rexmit_count = 0;
    st->seq_num_saved = seq_num;
#endif

    // send packet
    tcp_send_mode = TCP_SENDING_SEND;
    tcp->flags = TCP_FLAG_SYN;
    if(tcp_xmit(st, frame, 0))
      return id;

    st->status = TCP_CLOSED;
  }

  return 0xff;
}

// send TCP data
// dont use somewhere except tcp_write callback!
void tcp_send(uint8_t id, eth_frame_t *frame, uint16_t len, uint8_t options) {

  ip_packet_t *ip = (void*)(frame->data);
  tcp_packet_t *tcp = (void*)(ip->data);
  tcp_state_t *st = tcp_pool + id;
  uint8_t flags = TCP_FLAG_ACK;

  // check if connection established
  if(st->status != TCP_ESTABLISHED)
    return;

  // send PSH/ACK
  if(options & TCP_OPTION_PUSH)
    flags |= TCP_FLAG_PSH;

  // send FIN/ACK
  if(options & TCP_OPTION_CLOSE) {
    flags |= TCP_FLAG_FIN;
    st->status = TCP_FIN_WAIT;
  }

  // send packet
  tcp->flags = flags;
  tcp_xmit(st, frame, len);
}

// processing tcp packets
void tcp_filter(eth_frame_t *frame, uint16_t len) {

  ip_packet_t *ip = (void*)(frame->data);
  tcp_packet_t *tcp = (void*)(ip->data);
  tcp_state_t *st = 0, *pst;
  uint8_t id, tcpflags;

  if(ip->to_addr != ip_addr)
    return;

  // tcp data length
  len -= tcp_head_size(tcp);

  // me needs only SYN/FIN/ACK/RST
  tcpflags = tcp->flags & (TCP_FLAG_SYN|TCP_FLAG_ACK|TCP_FLAG_RST|TCP_FLAG_FIN);

  // sending packets back
  tcp_send_mode = TCP_SENDING_REPLY;
  tcp_ack_sent = 0;

  // search connection pool for connection
  //  to specific port from specific host/port
  for(id = 0; id < TCP_MAX_CONNECTIONS; ++id) {
    pst = tcp_pool + id;
    if((pst->status != TCP_CLOSED) &&
       (ip->from_addr == pst->remote_addr) &&
       (tcp->from_port == pst->remote_port) &&
       (tcp->to_port == pst->local_port)) {

      st = pst;
      break;
    }
  }

  // connection not found/new connection
  if(!st) {
    // received SYN - initiating new connection
    if( tcpflags == TCP_FLAG_SYN ) {
      // search for free slot for connection
      for(id = 0; id < TCP_MAX_CONNECTIONS; ++id) {
        pst = tcp_pool + id;
        if(pst->status == TCP_CLOSED) {
          st = pst;
          break;
        }
      }

      // slot found and app accepts connection?
      if(st && tcp_listen(id, frame)) {
        // add embrionic connection to pool
        st->status = TCP_SYN_RECEIVED;
        st->event_time = gettc();
        st->seq_num = gettc() + (gettc() << 16);
        st->ack_num = ntohl(tcp->seq_num) + 1;
        st->remote_addr = ip->from_addr;
        st->remote_port = tcp->from_port;
        st->local_port = tcp->to_port;

#ifdef WITH_TCP_REXMIT
        st->is_closing = 0;
        st->rexmit_count = 0;
        st->seq_num_saved = st->seq_num;
#endif

        // send SYN/ACK
        tcp->flags = TCP_FLAG_SYN|TCP_FLAG_ACK;
        tcp_xmit(st, frame, 0);
      }
    }
  }

  else {
    // connection reset by peer?
    if(tcpflags & TCP_FLAG_RST) {
      if((st->status == TCP_ESTABLISHED) ||
         (st->status == TCP_FIN_WAIT)) {

        tcp_closed(id, 1);
      }
      st->status = TCP_CLOSED;
      return;
    }

    // me needs only ack packet
    if((ntohl(tcp->seq_num) != st->ack_num) ||
       (ntohl(tcp->ack_num) != st->seq_num) ||
       (!(tcpflags & TCP_FLAG_ACK))) {

      return;
    }

#ifdef WITH_TCP_REXMIT
    // save sequence number
    st->seq_num_saved = st->seq_num;

    // reset rexmit counter
    st->rexmit_count = 0;
#endif

    // update ack pointer
    st->ack_num += len;
    if((tcpflags & TCP_FLAG_FIN) || (tcpflags & TCP_FLAG_SYN))
      st->ack_num++;

    // reset timeout counter
    st->event_time = gettc();

    switch(st->status) {
      // SYN sent by me (active open, step 1)
      // awaiting SYN/ACK (active open, step 2)
      case TCP_SYN_SENT:

        // received packet must be SYN/ACK
        if(tcpflags != (TCP_FLAG_SYN|TCP_FLAG_ACK)) {
          st->status = TCP_CLOSED;
          break;
        }

        // send ACK (active open, step 3)
        tcp->flags = TCP_FLAG_ACK;
        tcp_xmit(st, frame, 0);

        // connection is now established
        st->status = TCP_ESTABLISHED;

        // app can send some data
        tcp_read(id, frame, 0);

        break;

      // SYN received my me (passive open, step 1)
      // SYN/ACK sent by me (passive open, step 2)
      // awaiting ACK (passive open, step 3)
      case TCP_SYN_RECEIVED:

        // received packet must be ACK
        if(tcpflags != TCP_FLAG_ACK) {
          st->status = TCP_CLOSED;
          break;
        }

        // connection is now established
        st->status = TCP_ESTABLISHED;

        // app can send some data
        tcp_read(id, frame, 0);

        break;

      // connection established
      // awaiting ACK or FIN/ACK
      case TCP_ESTABLISHED:

        // received FIN/ACK?
        // (passive close, step 1)
        if(tcpflags == (TCP_FLAG_FIN|TCP_FLAG_ACK))  {
          // feed data to app
          if(len)
            tcp_write(id, frame, len);

          // send FIN/ACK (passive close, step 2)
          tcp->flags = TCP_FLAG_FIN|TCP_FLAG_ACK;
          tcp_xmit(st, frame, 0);

          // connection is now closed
          st->status = TCP_CLOSED;
          tcp_closed(id, 0);
        }

        // received ACK
        else if(tcpflags == TCP_FLAG_ACK) {
          // feed data to app
          if(len)
            tcp_write(id, frame, len);

          // app can send some data
          tcp_read(id, frame, 0);

          // send ACK
          if((len) && (!tcp_ack_sent)) {
            tcp->flags = TCP_FLAG_ACK;
            tcp_xmit(st, frame, 0);
          }
        }

        break;

      // FIN/ACK sent by me (active close, step 1)
      // awaiting ACK or FIN/ACK
      case TCP_FIN_WAIT:

        // received FIN/ACK?
        // (active close, step 2)
        if(tcpflags == (TCP_FLAG_FIN|TCP_FLAG_ACK))  {
          // feed data to app
          if(len)
            tcp_write(id, frame, len);

          // send ACK (active close, step 3)
          tcp->flags = TCP_FLAG_ACK;
          tcp_xmit(st, frame, 0);

          // connection is now closed
          st->status = TCP_CLOSED;
          tcp_closed(id, 0);
        }

        // received ACK+data?
        // (buffer flushing by peer)
        else if((tcpflags == TCP_FLAG_ACK) && (len)) {
          // feed data to app
          tcp_write(id, frame, len);

          // send ACK
          tcp->flags = TCP_FLAG_ACK;
          tcp_xmit(st, frame, 0);

#ifdef WITH_TCP_REXMIT
          // our data+FIN/ACK acked
          st->is_closing = 1;
#endif
        }

        break;

      default:
        break;
    }
  }
}

// periodic event
void tcp_poll() {

#ifdef WITH_TCP_REXMIT
  eth_frame_t *frame = (void*)net_buf;
  ip_packet_t *ip = (void*)(frame->data);
  tcp_packet_t *tcp = (void*)(ip->data);
#endif

  uint8_t id;
  tcp_state_t *st;

  for(id = 0; id < TCP_MAX_CONNECTIONS; ++id) {
    st = tcp_pool + id;

#ifdef WITH_TCP_REXMIT
    // connection timed out?
    if((st->status != TCP_CLOSED) &&
       (gettc() - st->event_time > TCP_REXMIT_TIMEOUT)) {
      // rexmit limit reached?
      if(st->rexmit_count > TCP_REXMIT_LIMIT) {
        // close connection
        st->status = TCP_CLOSED;
        tcp_closed(id, 1);
      }

      // should rexmit?
      else {
        // reset timeout counter
        st->event_time = gettc();

        // increment rexmit counter
        st->rexmit_count++;

        // load previous state
        st->seq_num = st->seq_num_saved;

        // will send packets
        tcp_send_mode = TCP_SENDING_SEND;
        tcp_ack_sent = 0;

        // rexmit
        switch(st->status) {
          // rexmit SYN
          case TCP_SYN_SENT:
            tcp->flags = TCP_FLAG_SYN;
            tcp_xmit(st, frame, 0);
            break;

          // rexmit SYN/ACK
          case TCP_SYN_RECEIVED:
            tcp->flags = TCP_FLAG_SYN|TCP_FLAG_ACK;
            tcp_xmit(st, frame, 0);
            break;

          // rexmit data+FIN/ACK or ACK (in FIN_WAIT state)
          case TCP_FIN_WAIT:

            // data+FIN/ACK acked?
            if(st->is_closing) {
              tcp->flags = TCP_FLAG_ACK;
              tcp_xmit(st, frame, 0);
              break;
            }

            // rexmit data+FIN/ACK
            st->status = TCP_ESTABLISHED;

          // rexmit data
          case TCP_ESTABLISHED:
            tcp_read(id, frame, 1);
            if(!tcp_ack_sent) {
              tcp->flags = TCP_FLAG_ACK;
              tcp_xmit(st, frame, 0);
            }
            break;

          default:
            break;
        }
      }
    }
#else
    // check if connection timed out
    if((st->status != TCP_CLOSED) &&
       (gettc() - st->event_time > TCP_CONN_TIMEOUT)) {
      // kill connection
      st->status = TCP_CLOSED;
      tcp_closed(id, 1);
    }
#endif
  }
}

#endif


/*
 * UDP
 */

#ifdef WITH_UDP

// send UDP packet
// fields must be set:
//  - ip.dst
//  - udp.src_port
//  - udp.dst_port
// uint16_t len is UDP data payload length
uint8_t udp_send(eth_frame_t *frame, uint16_t len) {

  ip_packet_t *ip = (void*)(frame->data);
  udp_packet_t *udp = (void*)(ip->data);

  len += sizeof(udp_packet_t);

  ip->protocol = IP_PROTOCOL_UDP;
  ip->from_addr = ip_addr;

  udp->len = htons(len);
  udp->cksum = 0;
  udp->cksum = ip_cksum(len + IP_PROTOCOL_UDP,(uint8_t*)udp-8, len+8);

  return ip_send(frame, len);
}

// reply to UDP packet
// len is UDP data payload length
void udp_reply(eth_frame_t *frame, uint16_t len) {

  ip_packet_t *ip = (void*)(frame->data);
  udp_packet_t *udp = (void*)(ip->data);
  uint16_t temp;

  len += sizeof(udp_packet_t);

  ip->to_addr = ip_addr;

  temp = udp->from_port;
  udp->from_port = udp->to_port;
  udp->to_port = temp;

  udp->len = htons(len);

  udp->cksum = 0;
  udp->cksum = ip_cksum(len + IP_PROTOCOL_UDP,(uint8_t*)udp-8, len+8);

  ip_reply(frame, len);
}

// process UDP packet
void udp_filter(eth_frame_t *frame, uint16_t len) {

  ip_packet_t *ip = (void*)(frame->data);
  udp_packet_t *udp = (void*)(ip->data);

  if(len >= sizeof(udp_packet_t)) {
    len = ntohs(udp->len) - sizeof(udp_packet_t);
    switch(udp->to_port) {
#ifdef WITH_DHCP
    case DHCP_CLIENT_PORT:
      dhcp_filter(frame, len);
      break;
#endif
    default:
      udp_packet(frame, len);
      break;
    }
  }
}

#endif


/*
 * ICMP
 */

#ifdef WITH_ICMP

// process ICMP packet
void icmp_filter(eth_frame_t *frame, uint16_t len) {

  ip_packet_t *packet = (void*)frame->data;
  icmp_echo_packet_t *icmp = (void*)packet->data;

  if(len >= sizeof(icmp_echo_packet_t)) {
    if(icmp->type == ICMP_TYPE_ECHO_RQ) {
      icmp->type = ICMP_TYPE_ECHO_RPLY;
      icmp->cksum += 8; // update cksum
      ip_reply(frame, len);
    }
  }
}

#endif


/*
 * IP
 */

// calculate IP checksum
uint16_t ip_cksum(uint32_t sum, uint8_t *buf, uint16_t len) {

  while(len >= 2) {
    sum += ((uint16_t)*buf << 8) | *(buf+1);
    buf += 2;
    len -= 2;
  }

  if(len)
    sum += (uint16_t)*buf << 8;

  while(sum >> 16)
    sum = (sum & 0xffff) + (sum >> 16);

  return ~htons((uint16_t)sum);
}

// send IP packet
// fields must be set:
//  - ip.dst
//  - ip.proto
// len is IP packet payload length
uint8_t ip_send(eth_frame_t *frame, uint16_t len) {

  ip_packet_t *ip = (void*)(frame->data);
  uint32_t route_ip;
  uint8_t *mac_addr_to;

  // set frame.dst
  if(ip->to_addr == ip_broadcast) {
    // use broadcast MAC
    memset(frame->to_addr, 0xff, 6);
  }
  else {
    // apply route
    if(((ip->to_addr ^ ip_addr) & ip_mask) == 0)
      route_ip = ip->to_addr;
    else
      route_ip = ip_gateway;

    // resolve mac address
#pragma diag_suppress 1293
    if(!(mac_addr_to = arp_resolve(route_ip)))
#pragma diag_default 1293
      return 0;
    memcpy(frame->to_addr, mac_addr_to, 6);
  }

  // set frame.type
  frame->type = ETH_TYPE_IP;

  // fill IP header
  len += sizeof(ip_packet_t);

  ip->ver_head_len = 0x45;
  ip->tos = 0;
  ip->total_len = htons(len);
  ip->fragment_id = 0;
  ip->flags_framgent_offset = 0;
  ip->ttl = IP_PACKET_TTL;
  ip->cksum = 0;
  ip->from_addr = ip_addr;
  ip->cksum = ip_cksum(0, (void*)ip, sizeof(ip_packet_t));

  // send frame
  eth_send(frame, len);
  return 1;
}

// send IP packet back
// len is IP packet payload length
void ip_reply(eth_frame_t *frame, uint16_t len) {

  ip_packet_t *packet = (void*)(frame->data);

  len += sizeof(ip_packet_t);

  packet->total_len = htons(len);
  packet->fragment_id = 0;
  packet->flags_framgent_offset = 0;
  packet->ttl = IP_PACKET_TTL;
  packet->cksum = 0;
  packet->to_addr = packet->from_addr;
  packet->from_addr = ip_addr;
  packet->cksum = ip_cksum(0, (void*)packet, sizeof(ip_packet_t));

  eth_reply((void*)frame, len);
}

// can be called directly after
//  ip_send/ip_reply with new data
void ip_resend(eth_frame_t *frame, uint16_t len) {

  ip_packet_t *ip = (void*)(frame->data);

  len += sizeof(ip_packet_t);
  ip->total_len = htons(len);
  ip->cksum = 0;
  ip->cksum = ip_cksum(0, (void*)ip, sizeof(ip_packet_t));

  eth_resend(frame, len);
}

// process IP packet
void ip_filter(eth_frame_t *frame, uint16_t len) {

  uint16_t hcs;
  ip_packet_t *packet = (void*)(frame->data);

  //if(len >= sizeof(ip_packet_t))
  //{
    hcs = packet->cksum;
    packet->cksum = 0;

    //printf("My addr:%d.%d.%d.%d Proto:%d to addr:%d.%d.%d.%d\r\n",(uint8_t)(ip_addr),(uint8_t)(ip_addr>>8),(uint8_t)(ip_addr>>16),(uint8_t)(ip_addr>>24),packet->protocol,(uint8_t)(packet->to_addr),(uint8_t)(packet->to_addr>>8),(uint8_t)(packet->to_addr>>16),(uint8_t)(packet->to_addr>>24));

    if((packet->ver_head_len == 0x45) &&
       (ip_cksum(0, (void*)packet, sizeof(ip_packet_t)) == hcs) &&
       ((packet->to_addr == ip_addr) ||
        (packet->to_addr == ip_broadcast) ||
        (packet->to_addr == 0xffffffff))) {

      len = ntohs(packet->total_len) -
        sizeof(ip_packet_t);

      switch(packet->protocol) {
#ifdef WITH_ICMP
        case IP_PROTOCOL_ICMP:
          icmp_filter(frame, len);
          break;
#endif

#ifdef WITH_UDP
        case IP_PROTOCOL_UDP:
          udp_filter(frame, len);
          break;
#endif

#ifdef WITH_TCP
        case IP_PROTOCOL_TCP:
          tcp_filter(frame, len);
          break;
#endif
      }
    }
  //}
}


/*
 * ARP
 */

// search ARP cache
uint8_t *arp_search_cache(uint32_t node_ip_addr) {

  uint8_t i;
  for(i = 0; i < ARP_CACHE_SIZE; ++i) {
    if(arp_cache[i].ip_addr == node_ip_addr)
      return arp_cache[i].mac_addr;
  }
  return 0;
}

// resolve MAC address
// returns 0 if still resolving
// (invalidates net_buffer if not resolved)
uint8_t *arp_resolve(uint32_t node_ip_addr) {

  eth_frame_t *frame = (void*)net_buf;
  arp_message_t *msg = (void*)(frame->data);
  uint8_t *mac;

  // search arp cache
#pragma diag_suppress  1293
  if((mac = arp_search_cache(node_ip_addr)))
#pragma diag_default  1293
    return mac;

  // send request
  memset(frame->to_addr, 0xff, 6);
  frame->type = ETH_TYPE_ARP;

  msg->hw_type = ARP_HW_TYPE_ETH;
  msg->proto_type = ARP_PROTO_TYPE_IP;
  msg->hw_addr_len = 6;
  msg->proto_addr_len = 4;
  msg->type = ARP_TYPE_REQUEST;
  memcpy(msg->mac_addr_from, mac_addr, 6);
  msg->ip_addr_from = ip_addr;
  memset(msg->mac_addr_to, 0x00, 6);
  msg->ip_addr_to = node_ip_addr;

  eth_send(frame, sizeof(arp_message_t));
  return 0;
}

// process arp packet
void arp_filter(eth_frame_t *frame, uint16_t len) {

  arp_message_t *msg = (void*)(frame->data);

  if(len >= sizeof(arp_message_t)) {
    if((msg->hw_type == ARP_HW_TYPE_ETH) &&
       (msg->proto_type == ARP_PROTO_TYPE_IP) &&
       (msg->ip_addr_to == ip_addr)) {

      switch(msg->type) {
        case ARP_TYPE_REQUEST:
          msg->type = ARP_TYPE_RESPONSE;
          memcpy(msg->mac_addr_to, msg->mac_addr_from, 6);
          memcpy(msg->mac_addr_from, mac_addr, 6);
          msg->ip_addr_to = msg->ip_addr_from;
          msg->ip_addr_from = ip_addr;
          eth_reply(frame, sizeof(arp_message_t));
          break;
        case ARP_TYPE_RESPONSE:
          if(!arp_search_cache(msg->ip_addr_from)) {
            arp_cache[arp_cache_wr].ip_addr = msg->ip_addr_from;
            memcpy(arp_cache[arp_cache_wr].mac_addr, msg->mac_addr_from, 6);
            arp_cache_wr++;
            if(arp_cache_wr == ARP_CACHE_SIZE)
              arp_cache_wr = 0;
          }
          break;
      }
    }
  }
}

/*
 * Ethernet
 */

// send new Ethernet frame to same host
//  (can be called directly after eth_send)
void eth_resend(eth_frame_t *frame, uint16_t len) {

  enc28j60_send_packet((void*)frame, len +
    sizeof(eth_frame_t));
}


// send Ethernet frame
// fields must be set:
//  - frame.dst
//  - frame.type
void eth_send(eth_frame_t *frame, uint16_t len) {

  memcpy(frame->from_addr, mac_addr, 6);
  enc28j60_send_packet((void*)frame, len + sizeof(eth_frame_t));
}

// send Ethernet frame back
void eth_reply(eth_frame_t *frame, uint16_t len) {

  memcpy(frame->to_addr, frame->from_addr, 6);
  memcpy(frame->from_addr, mac_addr, 6);
  enc28j60_send_packet((void*)frame, len + sizeof(eth_frame_t));
}

// process Ethernet frame
void eth_filter(eth_frame_t *frame, uint16_t len) {

  if(len >= sizeof(eth_frame_t)) {
    //printf("Frame:%d\r\n",frame->type);
    switch(frame->type) {
      case ETH_TYPE_ARP:
        arp_filter(frame, len - sizeof(eth_frame_t));
        break;
      case ETH_TYPE_IP:
        ip_filter(frame, len - sizeof(eth_frame_t));
        break;
    }
  }
}


/*
 * LAN
 */

void lan_init(SPI_TypeDef *spi_number,GPIO_TypeDef* CS_port,uint16_t CS_pin) {

  enc28j60_init(mac_addr,spi_number,CS_port,CS_pin);

#ifdef WITH_DHCP
  dhcp_retry_time = rtime() + 2;
#endif
}

void lan_poll(void) {

  uint16_t len;
  eth_frame_t *frame = (void*)net_buf;
#pragma diag_suppress  1293
  while((len = enc28j60_recv_packet(net_buf, sizeof(net_buf))))
#pragma diag_default 1293
    eth_filter(frame, len);

#ifdef WITH_DHCP
  dhcp_poll();
#endif

#ifdef WITH_TCP
  tcp_poll();
#endif

}

uint8_t lan_up() {

  return ip_addr != 0;
}
