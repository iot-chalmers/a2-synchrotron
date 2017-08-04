/*
 * Copyright (c) 2014, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/**
 * \author Simon Duquennoy <simonduq@sics.se>
 */

#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__



#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 16

#undef ENABLE_COOJA_DEBUG
#define ENABLE_COOJA_DEBUG 0
/* Contiki netstack: RADIO */
#undef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO   cc2420_driver

/* The cc2420 transmission power (min:0, max: 31) */
#define RF_POWER                31

/* The cc2420 RSSI threshold (-32 is the reset value for -77 dBm) */
#define RSSI_THR        (-32 - 14)

/* No Downwards routes */
#undef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_NO_DOWNWARD_ROUTES
//#define RPL_CONF_MOP RPL_MOP_STORING_NO_MULTICAST

#undef UIP_CONF_IP_FORWARD
#define UIP_CONF_IP_FORWARD 0

/* RPL Trickle timer tuning */
#undef RPL_CONF_DIO_INTERVAL_MIN
#define RPL_CONF_DIO_INTERVAL_MIN 12 /* 4.096 s */

#undef RPL_CONF_DIO_INTERVAL_DOUBLINGS
//#define RPL_CONF_DIO_INTERVAL_DOUBLINGS 4 /* Max factor: x16. 4.096 s * 16 = 65.536 s */
#define RPL_CONF_DIO_INTERVAL_DOUBLINGS 6 /* Max factor: x64. 4.096 s * 64 = 262.144 s */

/* Default link metric */
#undef RPL_CONF_INIT_LINK_METRIC
#define RPL_CONF_INIT_LINK_METRIC 2 /* default 5 */
#undef RPL_CONF_MIN_HOPRANKINC
#define RPL_CONF_MIN_HOPRANKINC 256
#undef RPL_CONF_MAX_HOPRANKINC
#define RPL_CONF_MAX_HOPRANKINC 0 /* default (7 * RPL_MIN_HOPRANKINC) */
/* Do not accept RPL neighbors with rank greater than ours + 1.5 */
#define RPL_CONF_MAX_NBRHOPINC (RPL_MIN_HOPRANKINC + RPL_MIN_HOPRANKINC/2)

/* The neighbor table size */
#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS 22

/* The routing table size */
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES  0

/* No RPL DIS */
#undef RPL_CONF_DIS_SEND
#define RPL_CONF_DIS_SEND 0

/* Space saving */
#undef UIP_CONF_TCP
#define UIP_CONF_TCP             0
#undef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG     0
#undef UIP_CONF_DS6_ADDR_NBU
#define UIP_CONF_DS6_ADDR_NBU    1
#undef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE   160
#undef UIP_CONF_UDP_CONNS
#define UIP_CONF_UDP_CONNS       2
#undef UIP_CONF_FWCACHE_SIZE
#define UIP_CONF_FWCACHE_SIZE    4

#undef PROCESS_CONF_NO_PROCESS_NAMES
#define PROCESS_CONF_NO_PROCESS_NAMES 1

/* Disable UDP checksum, needed as we have mutable fields (hop count) in the data packet */
#undef UIP_CONF_UDP_CHECKSUMS
#define UIP_CONF_UDP_CHECKSUMS   0

#define RPL_CONF_MAX_INSTANCES    1 /* default 1 */
#define RPL_CONF_MAX_DAG_PER_INSTANCE 1 /* default 2 */

#undef UIP_CONF_ND6_SEND_NA
#define UIP_CONF_ND6_SEND_NA 0

/* Contiki netstack: MAC */
#undef NETSTACK_CONF_MAC

/* Contiki netstack: RDC */
#undef NETSTACK_CONF_RDC


#undef RF_CHANNEL
#define RF_CHANNEL 26
#undef CC2420_CONF_AUTOACK
#define CC2420_CONF_AUTOACK 1
#define NETSTACK_CONF_MAC     csma_driver
#define NETSTACK_CONF_RDC     contikimac_driver
#undef CONTIKIMAC_CONF_COMPOWER
#define CONTIKIMAC_CONF_COMPOWER 0
//#define CONTIKIMAC_CONF_CYCLE_TIME (RTIMER_ARCH_SECOND / 22)
#undef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8
#undef CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION
#define CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION 1
#undef CSMA_CONF_MAX_MAC_TRANSMISSIONS
#define CSMA_CONF_MAX_MAC_TRANSMISSIONS 9
#undef CSMA_CONF_MAX_NEIGHBOR_QUEUES
#define CSMA_CONF_MAX_NEIGHBOR_QUEUES 6

/* Contiki netstack: FRAMER */
#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_802154


#endif /* __PROJECT_CONF_H__ */
