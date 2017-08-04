/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2017 Beshr Al Nahas and Olaf Landsiedel.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/**
 * \file
 *         A2-Synchrotron MAC implementation.
 *         Must be used with nordc as NETSTACK_CONF_RDC
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */

#undef ENABLE_COOJA_DEBUG
#define ENABLE_COOJA_DEBUG COOJA
#include "dev/cooja-debug.h"

#include <string.h>
#include "contiki.h"
#include "dev/radio.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "sys/process.h"
#include "sys/rtimer.h"

#include "chaos-platform-specific.h"
#include "dev/watchdog.h"
#include "rtimer-arch.h"

#include "chaos.h"
#include "chaos-scheduler.h"
#include "chaos-log.h"
#include "node.h"
#include "chaos-control.h"
#include "chaos-random-generator.h"

#define CHAOS_TX_RTIMER_GUARD 1
#define CHAOS_RX_RTIMER_GUARD 1

/* busy wait at the end of a premature slot, or wait only at the beginning of a slot? */
#define BUSYWAIT_UNTIL_SLOT_END 1
#define TURNOFF_AT_SLOT_END 1

#define CHAOS_FIFOP_THRESHOLD 3

#define TX_RANDOM_DELAY_MAX 0
#if TX_RANDOM_DELAY_MAX
#define TX_RANDOM_DELAY (chaos_random_generator_fast() & TX_RANDOM_DELAY_MAX)
#else
#define TX_RANDOM_DELAY (0)
#endif

#define ROUND_START_FROM_SLOT(t_sfd_actual, slot_number, t_slot_length) \
((t_sfd_actual) - (slot_number) * ((t_slot_length)))

/* local state */
/* allocate tx, rx packets as uint32_t arrays to guarantee alignment */
static uint32_t tx_packet_32t[(RADIO_MAX_PACKET_LEN + 3)/ 4];
static uint32_t dummy_packet_32t[(RADIO_MAX_PACKET_LEN + 3)/ 4]={0};
const uint32_t * chaos_get_dummy_packet_32t(){
  return dummy_packet_32t;
}
static uint32_t rx_packet_32t[(RADIO_MAX_PACKET_LEN + 3)/ 4];
static uint8_t * const tx_packet = (uint8_t *) tx_packet_32t;
static uint8_t * const rx_packet = (uint8_t *) rx_packet_32t;
static chaos_header_t* const tx_header = (chaos_header_t*)tx_packet_32t;
static chaos_header_t* const rx_header = (chaos_header_t*)rx_packet_32t;

uint8_t chaos_slot_log[MAX_SLOTS_IN_ROUND] = {0};
rtimer_clock_t chaos_slot_timing_log_max[SLOT_TIMING_SIZE] = {0};
rtimer_clock_t chaos_slot_timing_log_min[SLOT_TIMING_SIZE] = {0};
rtimer_clock_t chaos_slot_timing_log_current[SLOT_TIMING_SIZE] = {0};
uint32_t chaos_slot_timing_rx_sum = 0;
uint32_t chaos_slot_timing_tx_sum = 0;

uint16_t chaos_slot_stats[CHAOS_SLOT_STATS_SIZE] = {0};
#define SET_SLOT_STATUS(SLOT, RXTX, SUCCESS) \
  do { \
    if((RXTX) == CHAOS_TX){ \
      chaos_slot_log[SLOT] = ((SUCCESS) == CHAOS_TXRX_OK) ? 12 : 11;  \
    } else if((RXTX) == CHAOS_RX){ \
      chaos_slot_log[SLOT] = (SUCCESS);  \
    } else { \
      chaos_slot_log[SLOT] = 10;  \
    } \
    chaos_slot_stats[chaos_slot_log[SLOT] % CHAOS_SLOT_STATS_SIZE]++; \
  } while(0)

static uint8_t* app_flags = 0;
static int flag_delta = 0;

/* these variables are modified in rtimer interrupt context,
 * and are accessed outside the interrupt through get functions.
 * We make them volatile to force the compiler optimizations off
 * that were reading wrong values
 */
volatile static int round_synced = 0;
volatile static uint16_t sync_round = 0;
volatile static uint8_t next_round_id = 0;
volatile static rtimer_clock_t next_round_begin = 0, t_slot_start_dco = 0;
volatile static rtimer_clock_t round_offset_to_radio_on = 0;
volatile static vht_clock_t  round_rtimer = 0,
    round_offset_to_radio_on_vht = 0,
    round_start = 0;
static vht_clock_t t_slot_start = 0;

/* lower is better */
static uint8_t chaos_rank = CHAOS_MAX_RANK;
/* lower is better */
static uint8_t chaos_time_rank = CHAOS_MAX_RANK;

#if CHAOS_HW_SECURITY
#include "net/linkaddr.h"
static uint8_t *chaos_security_key = CHAOS_SECURITY_KEY;
static uint8_t chaos_nonce[16];
/*---------------------------------------------------------------------------*/
const uint8_t *
chaos_get_extended_address(uint16_t addr)
{
  /* workaround for short addresses: derive EUI64 as in RFC 6282 */
  //static linkaddr_extended_t template
  static uint8_t template[8] = { 0x00 , 0x00 , 0x00 ,
                                            0xFF , 0xFE , 0x00 , 0x00 , 0x00 };

  //template.u16[3] = LLSEC802154_HTONS(addr);
  template[6] = addr >> 8u;
  template[7] = addr & 0xff;
  return template;
}
/*---------------------------------------------------------------------------*/
//#define CCM_STAR_AUTH_FLAGS(Adata, L) ((Adata ? (0x08) : 0) | ((L-1) & 0x07))
#define CCM_STAR_AUTH_FLAGS(a_len, mic_len) ((a_len ? (1u << 6) : 0) | (((mic_len - 2u) >> 1) << 3) | 1u)
//void
//chaos_make_nonce(
//    uint8_t *nonce,
//    const uint8_t *extended_source_address,
//    uint32_t security_frame_counter)
//{
//  /* 1 byte||          8 bytes        ||    4 bytes    || 1 byte          || 2 bytes */
//  /* flags || extended_source_address || frame_counter || key seq counter || block counter */
//
//  uint8_t mic_length = (2 << (CHAOS_SECURITY_LEVEL & 3));
//  nonce[0] = CCM_STAR_AUTH_FLAGS(CHAOS_CLEARTEXT_HEADER_SIZE, mic_length);
////  memcpy(nonce + 1, extended_source_address, 8);
////  nonce[9] = (security_frame_counter) >> 24u;
////  nonce[10] = (security_frame_counter) >> 16u;
////  nonce[11] = (security_frame_counter) >> 8u;
////  nonce[12] = security_frame_counter & 0xff;
//
//  //test with a const nonce
//  memset(nonce + 1, 0xa5, 8);
//  nonce[9] = 0;
//  nonce[10] = 0;
//  nonce[11] = 0;
//  nonce[12] = 0;
//
//  nonce[13] = 0;
//  nonce[14] = 0;
//  nonce[15] = 1; /* must be set to 1 as it is incremented in HW */
//}
void
chaos_make_const_nonce(uint8_t *nonce)
{
  /* 1 byte||          8 bytes        ||    4 bytes    || 1 byte          || 2 bytes */
  /* flags || extended_source_address || frame_counter || key seq counter || block counter */

  uint8_t mic_length = (2 << (CHAOS_SECURITY_LEVEL & 3));
  nonce[0] = CCM_STAR_AUTH_FLAGS(CHAOS_CLEARTEXT_HEADER_SIZE, mic_length);
//  memcpy(nonce + 1, extended_source_address, 8);
//  nonce[9] = (security_frame_counter) >> 24u;
//  nonce[10] = (security_frame_counter) >> 16u;
//  nonce[11] = (security_frame_counter) >> 8u;
//  nonce[12] = security_frame_counter & 0xff;

  //test with a const nonce
  memset(nonce + 1, 0xa5, 8);
  nonce[9] = 0;
  nonce[10] = 0;
  nonce[11] = 0;
  nonce[12] = 0;
  nonce[13] = 0;
  nonce[14] = 0;
  nonce[15] = 1; /* must be set to 1 as it is incremented in HW */
}
/*---------------------------------------------------------------------------*/
//void
//chaos_update_nonce(
//    uint8_t *nonce,
//    uint32_t security_frame_counter)
//{
//  /* 1 byte||          8 bytes        ||    4 bytes    || 1 byte          || 2 bytes */
//  /* flags || extended_source_address || frame_counter || key seq counter || block counter */
////  nonce[9] = (security_frame_counter) >> 24u;
////  nonce[10] = (security_frame_counter) >> 16u;
////  nonce[11] = (security_frame_counter) >> 8u;
////  nonce[12] = security_frame_counter & 0xff;
//  nonce[9] = 0;
//  nonce[10] = 0;
//  nonce[11] = 0;
//  nonce[12] = 0;
//}
/*---------------------------------------------------------------------------*/
uint8_t*
chaos_get_nonce_pointer()
{
  return chaos_nonce;
}
/*---------------------------------------------------------------------------*/
uint8_t*
chaos_get_security_key_pointer()
{
  return chaos_security_key;
}
#endif /* CHAOS_HW_SECURITY */
/*---------------------------------------------------------------------------*/
static void
on(void)
{
  cc2420_fast_on();
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
  cc2420_fast_off();
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return (CHAOS_INTERVAL/RTIMER_SECOND) * CLOCK_SECOND;
}
/*---------------------------------------------------------------------------*/
/*
 * Timing
 */

//static ALWAYS_INLINE void
//chaos_clock_delay(register volatile uint16_t i)
//{
//  /* delay for i*4 CPU CYCLES
//   * equivalent to:**/
////    do {
////      _NOP();
////    } while(i--);
//
//  asm volatile("add #-1,%[d]" : : [d] "r" (i));
//  asm volatile("nop");
//  asm volatile("jnz $-4");
//}

/* Schedule link operation conditionally, and YIELD if success only */
#define CHAOS_SCHEDULE_AND_YIELD(pt, tm, ref_time, offset) \
  do { \
    if(chaos_schedule_round(tm, ref_time, offset, 1)) { \
      PT_YIELD(pt); \
    } \
  } while(0);
/*---------------------------------------------------------------------------*/
/* Function send for CHAOS-MAC, puts the packet in packetbuf in the MAC queue */
static void
send_packet(mac_callback_t sent, void *ptr)
{

}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
#ifdef NETSTACK_DECRYPT
  //NETSTACK_DECRYPT();
#endif /* NETSTACK_DECRYPT */
}

static ALWAYS_INLINE uint8_t
chaos_tx_slot(rtimer_clock_t* sfd_dco){
  uint8_t tx_status = 0;
  LEDS_ON(LEDS_GREEN);
  /* block until tx ends */
  tx_status = NETSTACK_RADIO_fast_send(tx_packet, (uint16_t*)sfd_dco);

//  tx_status = chaos_random_generator_fast()+2;
  if(tx_status) {
    BUSYWAIT_UNTIL(!CC2420_SFD_IS_1, CHAOS_PACKET_DURATION(CHAOS_PACKET_RADIO_LENGTH(tx_header->length)));
  }
  LEDS_OFF(LEDS_GREEN);
  return tx_status ? CHAOS_TXRX_OK : CHAOS_TXRX_ERROR;
}

static ALWAYS_INLINE int
chaos_post_rx(int rx_state, uint8_t app_id, int round_synced, uint16_t round_number)
{
  /* Update channel black-list and merge */
  chaos_multichannel_update_black_list(IS_INITIATOR(), round_synced, rx_state, app_id, rx_header, tx_header);

  /* Merge app flags */
  flag_delta = 0;
  if(rx_state == CHAOS_TXRX_OK) {
    if(app_id == rx_header->id) {
      //COOJA_DEBUG_STR("valid packet");
      /* Processing */
      //TODO: some more header processing
      //TODO: incl. checking and setting header fields
      if( !chaos_apps[app_id]->requires_node_index || chaos_has_node_index ){
        tx_header->length = rx_header->length;
      }
#if CHAOS_USE_SRC_ID
      tx_header->src_node_id = node_id;
#endif
      tx_header->id = rx_header->id;
      next_round_id = tx_header->next_round_id = rx_header->next_round_id;
      next_round_begin = tx_header->next_round_start = rx_header->next_round_start;

      flag_delta = (tx_header->join != rx_header->join) /*|| (tx_header->leave != rx_header->leave)*/;
      tx_header->join |= rx_header->join;
      //tx_header->leave |= rx_header->leave;
    } else {
      app_flags = NULL;
      rx_state = CHAOS_RX_HEADER_ERROR; //we received an unexpected packet type
      //COOJA_DEBUG_STR("invalid packet or timeout");
    }
  } else {
    //flush SW rx buffer on invalid rx
    //memset(rx_packet_32t, 0, sizeof(rx_packet_32t));
    rx_header->length = 0;
  }
  return rx_state;
}

static ALWAYS_INLINE int
chaos_rx_slot(vht_clock_t* sfd_vht, int round_synced, uint8_t app_id, uint8_t association)
{
	//COOJA_DEBUG_STR("RX slot begin");
  int rx_state = CHAOS_TXRX_UNKOWN;
  rtimer_clock_t slot_length = (association) ? (ASSOCIATION_SLOT_LEN + ((chaos_random_generator_fast() > CHAOS_RANDOM_MAX/2) ? ASSOCIATION_SLOT_LEN / 8 : 0)) : chaos_apps[app_id]->slot_length; //in rtimer ticks
  NETSTACK_RADIO_flushrx();
  LEDS_ON(LEDS_GREEN);
  SET_PIN_ADC2;
	rx_state = NETSTACK_RADIO_fast_rx(sfd_vht, round_synced, app_id, association, rx_packet, slot_length);
//	rx_state = chaos_random_generator_fast()+2;
	UNSET_PIN_ADC1;
  LEDS_OFF(LEDS_GREEN);
	//COOJA_DEBUG_STR("RX slot end");
	return rx_state;
}

static volatile vht_clock_t t_go_goal = 0;
static volatile vht_clock_t t_go_actual = 0;
static volatile vht_clock_t t_sfd_goal = 0, t_sfd_goal_log = 0;
static vht_clock_t t_sfd_actual = 0;
static vht_rtimer_dco_clockt_t t_go_goal_vht_rtimer_dco;
static volatile rtimer_clock_t t_go_dco_delay;
static volatile rtimer_clock_t t_go_rtimer;
static volatile rtimer_clock_t wait_end_dco;
static volatile rtimer_clock_t wait_end_rtimer;
static volatile rtimer_clock_t call_dco, t_txrx_end = 0;

int __attribute__ ((section(".chaos_do_tx")))
chaos_do_tx( void ){

  int status;
  rtimer_clock_t sfd_dco;
  rtimer_clock_t call_dco_delay;

  t_go_goal = t_sfd_goal - CHAOS_TX_DELAY_VHT;
  t_go_goal_vht_rtimer_dco = vht_to_vht_rtimer_dco(t_go_goal);
  t_go_rtimer = t_go_goal_vht_rtimer_dco.rtimer - CHAOS_TX_RTIMER_GUARD - 1;
  t_go_dco_delay = t_go_goal_vht_rtimer_dco.dco + RTIMER_TO_DCO(CHAOS_TX_RTIMER_GUARD);

  //wait until we get there
  while(RTIMER_LT(RTIMER_NOW(), t_go_rtimer));
  on();
  //wait for the last tick using the faster unsafe function
  while(RTIMER_NOW_FAST() == t_go_rtimer);
  //wait for dco part
  wait_end_dco = VHT_DCO_NOW();
  wait_end_rtimer = RTIMER_NOW_FAST();
  call_dco_delay = DCO_NOW();
  chaos_clock_delay_exact(t_go_dco_delay-(call_dco_delay-wait_end_dco));
  call_dco = DCO_NOW();
  //go
  status = chaos_tx_slot(&sfd_dco);
#if TURNOFF_AT_SLOT_END
  off();
#endif
  t_txrx_end = DCO_NOW();
  t_go_actual = RTIMER_DCO_TO_VHT(wait_end_rtimer, call_dco-wait_end_dco);
  t_sfd_actual = RTIMER_DCO_TO_VHT(wait_end_rtimer, sfd_dco-wait_end_dco);
  t_sfd_goal_log = t_sfd_goal;
  //printf("rtdco %u rtg %u go_dco_delay %u call_dco_delay %u wait_end_dco %u res %u\n", t_go_goal_vht_rtimer_dco.dco, RTIMER_TO_DCO(CHAOS_TX_RTIMER_GUARD), t_go_dco_delay, call_dco_delay, wait_end_dco, t_go_dco_delay-(tmp-wait_end_dco));
  return status;
}

int __attribute__ ((section(".chaos_do_rx")))
chaos_do_rx( const uint8_t app_id ){

  int status = CHAOS_TXRX_UNKOWN;
  rtimer_clock_t call_dco_delay;

  if(round_synced) {
    t_go_goal = t_sfd_goal - RTIMER_TO_VHT(RX_GUARD_TIME / 2) - CHAOS_RX_DELAY_VHT;
  } else {
    t_go_goal = t_sfd_goal - RTIMER_TO_VHT(ROUND_GUARD_TIME / 2) - CHAOS_RX_DELAY_VHT;
  }

  t_go_goal_vht_rtimer_dco = vht_to_vht_rtimer_dco(t_go_goal);
  t_go_rtimer = t_go_goal_vht_rtimer_dco.rtimer - CHAOS_RX_RTIMER_GUARD - 1;
  t_go_dco_delay = t_go_goal_vht_rtimer_dco.dco + RTIMER_TO_DCO(CHAOS_RX_RTIMER_GUARD);

  //wait until we get there
  while(RTIMER_LT(RTIMER_NOW(), t_go_rtimer));
  on();
  //wait for the last tick using the faster unsafe function
  while(RTIMER_NOW_FAST() == t_go_rtimer);
  //wait for dco part
  wait_end_dco = VHT_DCO_NOW();
  wait_end_rtimer = RTIMER_NOW_FAST();
  call_dco_delay = DCO_NOW();
  chaos_clock_delay_exact(t_go_dco_delay-(call_dco_delay-wait_end_dco));
  call_dco = DCO_NOW();
  //go
  status = chaos_rx_slot(&t_sfd_actual, round_synced, app_id, 0);
#if TURNOFF_AT_SLOT_END
  off();
#endif
  t_txrx_end = DCO_NOW();
  t_go_actual = RTIMER_DCO_TO_VHT(wait_end_rtimer, call_dco-wait_end_dco);
  t_sfd_goal_log = t_sfd_goal;

  return status;
}

uint16_t
chaos_round(const uint16_t round_number, const uint8_t app_id, const uint8_t* const payload, const uint8_t payload_length_app, const rtimer_clock_t slot_length_app_dco,
    const uint16_t max_slots,  const uint8_t app_flags_len, process_callback_t process){
  RTIMER_DCO_SYNC();

  static vht_clock_t t_round_on = 0;
  round_start = VHT_NOW();
  round_rtimer = chaos_control_get_round_rtimer(); //TODO change all units to VHT

  SET_PIN_ADC7;

  //vht_clock_t t_slot_start = 0, t_last_slot = 0;
  /* Profiling processing times and txrx */
  //vht_clock_t t_processing = 0, t_tx = 0, t_rx = 0;

  int chaos_slot_status = CHAOS_TXRX_UNKOWN;
  t_go_goal = 0;
  t_go_actual = 0;
  t_sfd_goal = 0, t_sfd_goal_log = 0;
  t_sfd_actual = 0;

  //init
  // XXX if not initiator, payload_length_app is usually 0!!
  vht_clock_t slot_length_app = RTIMER_TO_VHT(chaos_apps[app_id]->slot_length);
  uint8_t payload_length = MIN(CHAOS_MAX_PAYLOAD_LEN, payload_length_app);
  static uint16_t slot_number;
  static uint16_t sync_slot;

  sync_slot = 0;
  slot_number = 0;
  chaos_slot_timing_rx_sum = 0;
  chaos_slot_timing_tx_sum = 0;
  memset((void *)tx_packet, 0, RADIO_MAX_PACKET_LEN);
  memset((void *)rx_packet, 0, RADIO_MAX_PACKET_LEN);
  memset(chaos_slot_log, 0, sizeof(chaos_slot_log));
  memset(chaos_slot_stats, 0, sizeof(chaos_slot_stats));
  memset(chaos_slot_timing_log_max, 0, sizeof(chaos_slot_timing_log_max));
  memset(chaos_slot_timing_log_min, 0xff, sizeof(chaos_slot_timing_log_min));

  //TODO OL: for each packet we received: check if it matches this number, otherwise: cancel this round? and resync?
  tx_header->round_number = round_number;
  //TODO OL: init more: check header struct..
  tx_header->chaos_fcf_0 = CHAOS_FCF_0;
  tx_header->chaos_fcf_1 = CHAOS_FCF_1;
  tx_header->dst_pan_id = CHAOS_PANID;
#if CHAOS_USE_DST_ID
  tx_header->dst_node_id = FRAME802154_BROADCASTADDR;
#endif
#if CHAOS_USE_SRC_ID
  tx_header->src_node_id = CHAOS_USE_SRC_ID==2 ? INITIATOR_NODE_ID : node_id;
#endif
  //TODO OL: check that each received packet matches app_id, round_number, ... -> if not: cancel this round and resync.
  tx_header->id = app_id;
#if CHAOS_HW_SECURITY
  tx_header->security_control = CHAOS_SECURITY_CONTROL;
#endif /* CHAOS_HW_SECURITY */
  int i;
  for(i = 0; i < chaos_app_count; i++){
    if( chaos_apps[i]->round_begin_sniffer != NULL ){
      chaos_apps[i]->round_begin_sniffer(tx_header);
    }
  }

  if(IS_INITIATOR()) {
    next_round_id = tx_header->next_round_id = scheduler_get_next_round_app_id();
    next_round_begin = tx_header->next_round_start = scheduler_get_next_round_begin();
    chaos_rank = 0;
    chaos_time_rank = 0;
    round_synced = 1;
    sync_slot = 0;
  } else {
    round_synced = 0;
    sync_slot = 0xffff;
    //chaos_rank = CHAOS_MAX_RANK;
    //chaos_time_rank = CHAOS_MAX_RANK;
  }
#if CHAOS_USE_SRC_RANK
  tx_header->src_rank = chaos_rank;
//  tx_header->src_time_rank = chaos_time_rank;
#endif
  chaos_multichannel_round_init(IS_INITIATOR(), tx_header);

  memcpy(tx_header->payload, payload, payload_length);
  tx_header->length = CHAOS_PAYLOAD_LEN_TO_PACKET_LENGTH(payload_length);

  chaos_state_t chaos_state = CHAOS_INIT;
  if( NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC || !chaos_apps[app_id]->requires_node_index || (chaos_apps[app_id]->requires_node_index && chaos_has_node_index )){
    //LEDS_TOGGLE(LEDS_BLUE);
    chaos_state = process(0, 0, chaos_state, 0, CHAOS_PAYLOAD_LENGTH(tx_header), rx_header->payload, tx_header->payload, &app_flags);
  } else {
    CHAOS_LOG_ADD_MSG("! hasIdx %u id %u", chaos_has_node_index, node_id);
  }

//  CHAOS_LOG_ADD_MSG("Ini %u id %u hIdx %u nxt %s", IS_INITIATOR(), node_id, chaos_has_node_index, CHAOS_STATE_TO_STRING(next_state));
//  CHAOS_LOG_ADD_MSG("!n %u g %u d %u l %i", t_round_on, round_rtimer + scheduler_get_next_round_begin(), -((round_rtimer + scheduler_get_next_round_begin()) + t_round_on), too_late);

  HOP_CHANNEL(round_number, slot_number);

//  CHAOS_LOG_ADD_MSG("rr s %u n %u",
//      round_start,
//      RTIMER_NOW());

//  uint8_t too_late = chaos_schedule_check_timer_miss(round_rtimer, (- ROUND_GUARD_TIME), RTIMER_NOW());
//
//  if(too_late) {
//    LEDS_ON(LEDS_RED);
//    /* skip first slot */
//    slot_number++;
//    HOP_CHANNEL(round_number, slot_number);
//  }
  //first slot
  t_sfd_goal = round_rtimer + (slot_number) * slot_length_app;
//  chaos_state = CHAOS_RX;
  //COOJA_DEBUG_PRINTF("rr v %lx rb v %lx t_sfd_goal 0x%lx", round_rtimer, RTIMER_TO_VHT(chaos_control_get_current_round_begin()), t_sfd_goal);
  t_round_on = VHT_NOW();
  round_offset_to_radio_on_vht = t_round_on - round_start;
  round_offset_to_radio_on = VHT_TO_RTIMER(round_offset_to_radio_on_vht);

  //pet the watchdog to keep it calm during the round :)
  watchdog_periodic();
  LEDS_OFF(LEDS_BLUE);
  while( slot_number < max_slots && chaos_state != CHAOS_OFF ){
    t_slot_start_dco = DCO_NOW();
    t_slot_start = VHT_NOW();

    //COOJA_DEBUG_PRINTF("rs %lx rr %x rrv %lx ro %lx ss %lx", round_start, round_rtimer, round_rtimer, t_round_on, t_slot_start);

    //
    LEDS_ON(LEDS_BLUE);
    //for long rounds, pet the watchdog every slot to keep it calm :)
    watchdog_periodic();

    if(chaos_state == CHAOS_TX){
      tx_header->slot_number = slot_number;
      tx_header->slot_number_msb = slot_number > 255;
      tx_header->round_number = round_number;
#if CHAOS_USE_SRC_RANK
      tx_header->src_rank = chaos_rank;
      //tx_header->src_time_rank = chaos_time_rank;
#endif
#if CHAOS_HW_SECURITY
      //tx_header->src_node_id = node_id;
//        tx_header->chaos_security_frame_counter.round_number = round_number;
//        tx_header->chaos_security_frame_counter.slot_number = slot_number;
//        tx_header->chaos_security_frame_counter.seq_number = slot_number;
////        chaos_update_nonce(chaos_nonce, tx_header->chaos_security_frame_counter.security_frame_counter);
//        chaos_make_nonce(chaos_get_nonce_pointer(),
//            chaos_get_extended_address(tx_header->src_node_id),
//            tx_header->chaos_security_frame_counter.security_frame_counter);
        chaos_make_const_nonce(chaos_nonce);
        cc2420_set_nonce(chaos_nonce, 0); /* upload tx nonce */
#endif /* CHAOS_HW_SECURITY */

        chaos_slot_status = chaos_do_tx();

        chaos_slot_timing_log_current[TX] = t_txrx_end - call_dco;
        chaos_slot_timing_tx_sum += chaos_slot_timing_log_current[TX];
        if(slot_number > sync_slot){ //ignore first slot
          chaos_slot_timing_log_current[TX_PREPARE] = call_dco - t_slot_start_dco;
          chaos_slot_timing_log_max[TX_PREPARE] = MAX(chaos_slot_timing_log_current[TX_PREPARE], chaos_slot_timing_log_max[TX_PREPARE]);
          chaos_slot_timing_log_min[TX_PREPARE] = MIN(chaos_slot_timing_log_current[TX_PREPARE], chaos_slot_timing_log_min[TX_PREPARE]);
          chaos_slot_timing_log_max[TX] = MAX(chaos_slot_timing_log_current[TX], chaos_slot_timing_log_max[TX]);
          chaos_slot_timing_log_min[TX] = MIN(chaos_slot_timing_log_current[TX], chaos_slot_timing_log_min[TX]);
        }

//      COOJA_DEBUG_STRX("delay_exact_dco", delay_exact_dco, 6);

      //self correct for tx
//        if ( CHAOS_ENABLE_SFD_SYNC && chaos_slot_status == CHAOS_TXRX_OK && ((t_sfd_goal > t_sfd_actual
//            && t_sfd_goal - t_sfd_actual < MAX(TX_RANDOM_DELAY, DCO_TO_VHT(20)))
//            || (t_sfd_goal < t_sfd_actual
//                && t_sfd_actual - t_sfd_goal < MAX(TX_RANDOM_DELAY, DCO_TO_VHT(20))))) {
//          // XXX only self-correct if it is a meaningful value
////          t_sfd_goal = t_sfd_actual; //sometimes a transmission is delayed be a rx coming in the same time
//        }
      chaos_rank += ( !IS_INITIATOR() ) ? CHAOS_RANK_TX_INCREMENT : 0;
    } else {

      chaos_slot_status = chaos_do_rx(app_id);
      chaos_slot_timing_log_current[RX] = t_txrx_end - call_dco;
      chaos_slot_timing_rx_sum += chaos_slot_timing_log_current[RX];
      if(slot_number > sync_slot){
        chaos_slot_timing_log_current[RX_PREPARE] = call_dco - t_slot_start_dco;
        chaos_slot_timing_log_max[RX_PREPARE] = MAX(chaos_slot_timing_log_current[RX_PREPARE], chaos_slot_timing_log_max[RX_PREPARE]);
        chaos_slot_timing_log_min[RX_PREPARE] = MIN(chaos_slot_timing_log_current[RX_PREPARE], chaos_slot_timing_log_min[RX_PREPARE]);
        chaos_slot_timing_log_max[RX] = MAX(chaos_slot_timing_log_current[RX], chaos_slot_timing_log_max[RX]);
        chaos_slot_timing_log_min[RX] = MIN(chaos_slot_timing_log_current[RX], chaos_slot_timing_log_min[RX]);
      }

      /* it could be a valid packet but an unexpected app id.
       * Shall we use it for synchronization anyway?
       * Now we don't */

      chaos_slot_status = chaos_post_rx(chaos_slot_status, app_id, round_synced, round_number);

      if(chaos_slot_status == CHAOS_TXRX_OK){
        //slot_number = rx_header->slot_number;
        //slot_number |= rx_header->slot_number_msb ? 0x100 : 0;

        uint8_t rx_good_rank = 0;
#if CHAOS_USE_SRC_RANK

        // update rank during the round
        // use moving average like RPL ETX
        chaos_rank = ((uint16_t)chaos_rank * RANK_ALPHA +
                   (uint16_t)rx_header->src_rank * (RANK_SCALE - RANK_ALPHA)) / RANK_SCALE;

        //chaos_rank = rx_header->src_rank + 1 > 0 ? rx_header->src_rank + 1 : CHAOS_MAX_RANK;
        rx_good_rank = rx_header->src_rank <= chaos_rank;

#else
        rx_good_rank = 1;
        chaos_rank = IS_INITIATOR() ? 0 : CHAOS_MAX_RANK;
#endif
        if(!IS_INITIATOR()) {
          //never correct initiator sync because it is the time source

          /* correct sfd based on RX if better rank, otherwise skip */
//          uint8_t rx_good_time_rank = rx_header->src_time_rank <= chaos_time_rank;
//          if( (rx_good_time_rank && CHAOS_ENABLE_SFD_SYNC) || !round_synced) {
//            t_sfd_goal = t_sfd_actual; //actual end sfd
//            chaos_time_rank = rx_header->src_time_rank;
//          }
          if( rx_good_rank && CHAOS_ENABLE_SFD_SYNC == 1 ){
            /* only correct of meaningful value (within guard time) */
            if( ABS_VHT(t_sfd_goal - t_sfd_actual) < (round_synced ? RTIMER_TO_VHT(RX_GUARD_TIME) : RTIMER_TO_VHT(ROUND_GUARD_TIME)) ){
              t_sfd_goal = t_sfd_actual;
            }
          }
          if( !round_synced ){
            slot_number = rx_header->slot_number;
            slot_number |= rx_header->slot_number_msb ? 0x100 : 0;
            round_rtimer = ROUND_START_FROM_SLOT(t_sfd_actual, slot_number, slot_length_app);
            t_sfd_goal = t_sfd_actual;
            round_synced = 1;
            sync_slot = slot_number;
            next_round_begin = rx_header->next_round_start;
            next_round_id = rx_header->next_round_id;
  //          CHAOS_LOG_ADD_MSG("!rr %u, f %u, %u %ul\n", round_rtimer, t_sfd_actual_rtimer, slot_number, slot_length_app);
          }
        }
      } else {
        chaos_rank += ( !IS_INITIATOR() ) ? CHAOS_RANK_IDLE_INCREMENT : 0;
      }

    }
    SET_SLOT_STATUS(slot_number, chaos_state, chaos_slot_status);

//
    //XXX_number * slot_length_app;
#if WITH_CHAOS_LOG
    chaos_state_t chaos_state_backup_log;
    chaos_state_backup_log = chaos_state;
#endif
    /* increment time rank */
    //t_sfd_goal = round_rtimer + slot
//    if( !IS_INITIATOR() ){
//      chaos_time_rank = chaos_time_rank + 1 > 0 ? chaos_time_rank + 1 : CHAOS_MAX_RANK;
//    } else {
//      chaos_time_rank = 0;
//      chaos_rank = 0;
//    }
    uint8_t timing_log_state = chaos_state == CHAOS_TX ? TX_POST : RX_POST;
    rtimer_clock_t t_post_txrx_end = DCO_NOW();
    if(slot_number > sync_slot){
      chaos_slot_timing_log_current[timing_log_state] = t_post_txrx_end - t_txrx_end;
      chaos_slot_timing_log_max[timing_log_state] = MAX(chaos_slot_timing_log_current[timing_log_state], chaos_slot_timing_log_max[timing_log_state]);
      chaos_slot_timing_log_min[timing_log_state] = MIN(chaos_slot_timing_log_current[timing_log_state], chaos_slot_timing_log_min[RX]);
    }
    /* process app */
    if( !chaos_apps[app_id]->requires_node_index || chaos_has_node_index ){
      chaos_state = process(round_number, slot_number, chaos_state, (chaos_slot_status == CHAOS_TXRX_OK), (chaos_slot_status == CHAOS_TXRX_OK) ? CHAOS_PAYLOAD_LENGTH(rx_header) : 0, rx_header->payload, tx_header->payload, &app_flags);
      int app_do_sync = ( chaos_state == CHAOS_RX_SYNC ) || ( chaos_state == CHAOS_TX_SYNC );
      chaos_state = ( chaos_state == CHAOS_RX_SYNC ) ? chaos_state = CHAOS_RX : (( chaos_state == CHAOS_TX_SYNC ) ? chaos_state = CHAOS_TX : chaos_state);
      if( chaos_slot_status == CHAOS_TXRX_OK && app_do_sync  && CHAOS_ENABLE_SFD_SYNC == 2){
        /* the application reports a packet coming from the initiator, so we can synchronize on it;
         * e.g., we got a phase transition that only the initiator can issue */
        slot_number = rx_header->slot_number;
        slot_number |= rx_header->slot_number_msb ? 0x100 : 0;
        round_rtimer = ROUND_START_FROM_SLOT(t_sfd_actual, slot_number, slot_length_app);
        t_sfd_goal = t_sfd_actual;
      }
    }
#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
    else if( chaos_state == CHAOS_RX
        && chaos_slot_status == CHAOS_TXRX_OK ){
      /* work as a forwarder if not joined */
      flag_delta |= tx_header->length != rx_header->length;
      if( !flag_delta ){
        flag_delta |= memcmp(tx_header->payload, rx_header->payload, rx_header->length);
      }
      if( flag_delta ){
        memcpy(tx_header->payload, rx_header->payload, rx_header->length);
        tx_header->length = rx_header->length;
        chaos_state = CHAOS_TX;
        flag_delta = 0;
      }
    } else {
      chaos_state = CHAOS_RX;
    }
#endif /* NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC */

    t_sfd_goal += slot_length_app;

    rtimer_clock_t t_app_processing_end = DCO_NOW();
    const chaos_app_t* current_app = scheduler_get_current_app();
    int is_join_app = strcmp(current_app->name, "join");
    if(slot_number > sync_slot){
      if((!chaos_apps[app_id]->requires_node_index || chaos_has_node_index) && !is_join_app){
        chaos_slot_timing_log_current[APP_PROCESSING] = t_app_processing_end - t_post_txrx_end;
        chaos_slot_timing_log_max[APP_PROCESSING] = MAX(chaos_slot_timing_log_current[APP_PROCESSING], chaos_slot_timing_log_max[APP_PROCESSING]);
        chaos_slot_timing_log_min[APP_PROCESSING] = MIN(chaos_slot_timing_log_current[APP_PROCESSING], chaos_slot_timing_log_min[APP_PROCESSING]);
      } else if(is_join_app){
        chaos_slot_timing_log_current[JOIN_PROCESSING] = t_app_processing_end - t_post_txrx_end;
        chaos_slot_timing_log_max[JOIN_PROCESSING] = MAX(chaos_slot_timing_log_current[JOIN_PROCESSING], chaos_slot_timing_log_max[JOIN_PROCESSING]);
        chaos_slot_timing_log_min[JOIN_PROCESSING] = MIN(chaos_slot_timing_log_current[JOIN_PROCESSING], chaos_slot_timing_log_min[JOIN_PROCESSING]);
      }
    }
    /* log */
    //log hack!
//    typedef struct __attribute__((packed)) chaos_max_struct {
//      uint16_t dummy_start;
//      uint16_t max;
//      uint8_t dummy[APP_DUMMY_LEN];
//      uint8_t flags[];
//    } chaos_max_t;

    CHAOS_LOG_ADD(chaos_log_txrx, {
        log->txrx.state = chaos_state_backup_log;
        log->txrx.rx_status = chaos_slot_status;
        log->txrx.t_sfd_delta = t_sfd_actual - t_sfd_goal_log;

//        log->txrx.t_slot_start = t_slot_start;
//        log->txrx.t_go_goal = t_go_goal;
//        log->txrx.t_go_actual = t_go_actual;
        log->txrx.t_go_delta = t_go_actual - t_go_goal;
//        log->txrx.t_sfd_goal = t_sfd_goal_log;
//        log->txrx.t_sfd_actual = t_sfd_actual;
//        log->txrx.t_slot_length_app = slot_length_app;
//        log->txrx.t_slot_length_actual = ABS(t_sfd_goal_dco - t_sfd_goal_dco_log) / 4;
#if CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE
        log->txrx.channel_black_list_local = chaos_multichannel_get_black_list_local();
        log->txrx.channel_black_list_merged = chaos_multichannel_get_black_list_merged();
        log->txrx.channel_black_list_committed = chaos_multichannel_get_black_list_committed();
#endif /* CHAOS_MULTI_CHANNEL */
#if CHAOS_USE_SRC_ID
        log->txrx.src_node_id = (chaos_state_backup_log == CHAOS_RX ) ? rx_header->src_node_id : tx_header->src_node_id;
#endif
#if CHAOS_LOG_FLAGS
        if(!round_synced || !app_flags || !app_flags_len ) {
          memset(log->txrx.app_flags, 0, LOG_APP_FLAGS_LEN);
        } else {
          memcpy(log->txrx.app_flags, app_flags, MIN(app_flags_len, LOG_APP_FLAGS_LEN));
        }
#endif /* CHAOS_LOG_FLAGS */
#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
        void* payload = ( chaos_state_backup_log == CHAOS_RX ) ? rx_header->payload : tx_header->payload;
        log->txrx.join_committed = join_is_committed_from_payload( payload );
        log->txrx.join_has_node_index = chaos_has_node_index;
        log->txrx.join_slot_count = join_get_slot_count_from_payload( payload );
#endif
        /* profiling */
        //log->txrx.t_post_processing = (DCO_NOW() - t_processing)/4;
        //XXX log hack!
//        chaos_max_t* log_hack = (chaos_state_backup_log == CHAOS_RX ) ? (chaos_max_t*)(rx_header->payload) : (chaos_max_t*)(tx_header->payload);
//        log->txrx.t_post_processing = log_hack->max;
//        uint8_t log_hack = ( chaos_state_backup_log == CHAOS_RX ) ? rx_header->payload[0] : tx_header->payload[0];
//        log->txrx.t_post_processing = chaos_rank;
    });


    RTIMER_DCO_SYNC();
    /* move to next slot */
    slot_number++;
    /* change channel */
    HOP_CHANNEL(round_number, slot_number);
    LEDS_OFF(LEDS_BLUE);

#if BUSYWAIT_UNTIL_SLOT_END
    //t_last_slot = VHT_NOW() - t_slot_start;
    /* busy wait until end of slot if we still have time */
    rtimer_clock_t sfd_goal_rtimer = VHT_TO_RTIMER(t_sfd_goal);
    rtimer_clock_t slot_guard_time = ((round_synced ? RX_GUARD_TIME/2 : ROUND_GUARD_TIME/2))
        + (2) + VHT_TO_RTIMER(PREP_RX_VHT + 2*RX_LEDS_DELAY) /* for led toggling */
        + ( (chaos_state == CHAOS_RX) ? VHT_TO_RTIMER(CHAOS_RX_DELAY_VHT)
                                      : VHT_TO_RTIMER(CHAOS_TX_DELAY_VHT) );
    if(RTIMER_LT(RTIMER_NOW(), sfd_goal_rtimer - slot_guard_time)) {
      while(RTIMER_LT(RTIMER_NOW(), sfd_goal_rtimer - slot_guard_time));
    }
#endif /* BUSYWAIT_UNTIL_SLOT_END */

    LEDS_OFF(LEDS_RED);
    rtimer_clock_t t_slot_end = DCO_NOW();
    if(slot_number > sync_slot + 1){
      chaos_slot_timing_log_current[SLOT_END_PROCCESSING] = t_slot_end - t_app_processing_end;
      chaos_slot_timing_log_max[SLOT_END_PROCCESSING] = MAX(chaos_slot_timing_log_current[SLOT_END_PROCCESSING], chaos_slot_timing_log_max[SLOT_END_PROCCESSING]);
      chaos_slot_timing_log_min[SLOT_END_PROCCESSING] = MIN(chaos_slot_timing_log_current[SLOT_END_PROCCESSING], chaos_slot_timing_log_min[SLOT_END_PROCCESSING]);
      chaos_slot_timing_log_current[SLOT_TIME_ALL] = t_slot_end - t_slot_start_dco;
      chaos_slot_timing_log_max[SLOT_TIME_ALL] = MAX(chaos_slot_timing_log_current[SLOT_TIME_ALL], chaos_slot_timing_log_max[SLOT_TIME_ALL]);
      chaos_slot_timing_log_min[SLOT_TIME_ALL] = MIN(chaos_slot_timing_log_current[SLOT_TIME_ALL], chaos_slot_timing_log_min[SLOT_TIME_ALL]);
      if(chaos_slot_timing_log_current[SLOT_TIME_ALL] >= chaos_slot_timing_log_min[SLOT_TIME_ALL]){
        chaos_slot_timing_log_min[SLOTNUMBER] = slot_number;
      }
      if(chaos_slot_timing_log_current[SLOT_TIME_ALL] >= chaos_slot_timing_log_max[SLOT_TIME_ALL]){
        chaos_slot_timing_log_max[SLOTNUMBER] = slot_number;
      }
    }
  }

  LEDS_OFF(LEDS_RED);
  off();
  for(i = 0; i < chaos_app_count; i++){
    if( chaos_apps[i]->round_end_sniffer != NULL ){
      chaos_apps[i]->round_end_sniffer(tx_header);
    }
  }
  CHAOS_LOG_ADD_MSG("{I}GT s%u r%u", RX_GUARD_TIME, ROUND_GUARD_TIME);
  CHAOS_LOG_ADD_MSG("{I}Pw %u #%u a%u p%u", CHAOS_TX_POWER, CHAOS_NUMBER_OF_CHANNELS, CHAOS_MULTI_CHANNEL_ADAPTIVE, CHAOS_MULTI_CHANNEL_PARALLEL_SEQUENCES);
  UNSET_PIN_ADC7;

  //pet the watchdog to keep it calm after the round :)
  watchdog_periodic();

  //TODO: write result to payload packet
  return slot_number;
}

/* Associate:
 * If we are a master, start right away.
 * Otherwise, associate with a master
 */
uint8_t chaos_associate(rtimer_clock_t* t_sfd_actual_rtimer_ptr, uint16_t *round_number_ptr, uint16_t* slot_number_ptr, uint8_t* app_id_ptr)
{
  COOJA_DEBUG_STR("Association...");
  chaos_multichannel_init();
  int associated = 0;
  volatile uint8_t rx_status = CHAOS_TXRX_UNKOWN;
  uint16_t slot_number = 0;
  uint16_t round_number = 0;

  if(IS_INITIATOR()) {
    round_synced = 1;
    round_rtimer = VHT_NOW();
    associated = 1;
	} else {
	  SET_PIN_ADC6;
	  chaos_rank = CHAOS_MAX_RANK;
		vht_clock_t sfd_vht;
    const chaos_app_t* app =  NULL;
    uint16_t association_counter = 0;
    round_synced = 0;
    HOP_CHANNEL(round_number, chaos_random_generator_produce());
    on();

    /* repeat until we get a valid packet */
    do {
      associated = 0;
      *t_sfd_actual_rtimer_ptr = 0;
      sfd_vht = 0;
      do {
        /* try to get get a valid packet */
        rx_status = chaos_rx_slot(&sfd_vht, 0, 0, 1);
        *t_sfd_actual_rtimer_ptr = VHT_TO_RTIMER(sfd_vht);
        associated += (rx_status == CHAOS_TXRX_OK);
        watchdog_periodic(); /* association could take a long time */
        /* hop channel after a number of slots without a successful association */
        if(++association_counter > CHAOS_ASSOCIATION_HOP_CHANNEL_THERSHOLD) {
          HOP_CHANNEL(round_number, ++slot_number);
          on();
          association_counter = 0;
          round_number++;
          printf("{rd-%u st-%u ch-%u} ASC %s\n", round_number, slot_number, chaos_multichannel_get_current_channel(), CHAOS_RX_STATE_TO_STRING(rx_status));
        }
      } while(associated < 1 /*&& association_counter < CHAOS_ASSOCIATION_HOP_CHANNEL_THERSHOLD */); //XXX
        if( associated ){
          *app_id_ptr = rx_header->id;
          if(*app_id_ptr < chaos_app_count){
            app = chaos_apps[*app_id_ptr];
          }
        } else {
          app = NULL;
        }
      if(app == NULL) {
        /* associate again */
        associated = 0;
        COOJA_DEBUG_STR("!associate: app is null");
      }
    } while(app == NULL /*&& association_counter < CHAOS_ASSOCIATION_HOP_CHANNEL_THERSHOLD */); /* XXX  */
    if( associated ){
      round_number = *round_number_ptr = rx_header->round_number;
      sync_round = round_number;
      slot_number = rx_header->slot_number;
      slot_number |= rx_header->slot_number_msb ? 0x100 : 0;
      *slot_number_ptr = slot_number;
      vht_clock_t slot_length = RTIMER_TO_VHT(app->slot_length);
      round_rtimer = ROUND_START_FROM_SLOT(sfd_vht, slot_number, slot_length);
      COOJA_DEBUG_PRINTF("sfd_vht %lu, slot_number %u, slot_length %lu, round_timer %lu",
          sfd_vht, slot_number, slot_length, round_rtimer);
      round_synced = 1;
      next_round_begin = rx_header->next_round_start;
      next_round_id = rx_header->next_round_id;
      off();
      slot_number++; //for logging to be similar to after association
    }
	}

#if CHAOS_USE_SRC_RANK
    chaos_rank = rx_header->src_rank + CHAOS_INITIAL_MIN_RANK > 0 ? rx_header->src_rank + CHAOS_INITIAL_MIN_RANK : CHAOS_MAX_RANK;
#else
    chaos_rank = IS_INITIATOR() ? 0 : CHAOS_MAX_RANK;
#endif

  CHAOS_LOG_ADD(chaos_log_txrx, {
      log->txrx.state = CHAOS_OFF;
      log->txrx.rx_status = rx_status;
#if CHAOS_USE_SRC_ID
      log->txrx.src_node_id = rx_header->src_node_id;
#endif
//      log->txrx.t_go_delta = round_rtimer;
      log->txrx.t_sfd_delta = round_rtimer;
      //TODO: some more logging here?
  });
  UNSET_PIN_ADC6;
  //CHAOS_LOG_ADD_MSG("s %u %u", slot_number-1, rx_header->length);
  //CHAOS_LOG_ADD_MSG("r %u %u", round_rtimer, next_round_begin);
  return associated;
}

vht_clock_t get_round_offset_to_radio_on(){
  return round_offset_to_radio_on_vht;
}

vht_clock_t get_round_rtimer(){
  return (round_rtimer);
}

vht_clock_t get_round_start(){
  return (round_start);
}

int get_round_synced(){
  return round_synced;
}

rtimer_clock_t get_next_round_begin(){
  return next_round_begin;
}

/* only valid if round_synced */
uint8_t get_next_round_id(){
  return next_round_id;
}

uint16_t get_sync_round(){
  return sync_round;
}
/*---------------------------------------------------------------------------*/
static void
chaos_init(void)
{
  memset(chaos_slot_log, 0, sizeof(chaos_slot_log));
  memset(tx_packet_32t, 0, sizeof(tx_packet_32t));
  memset(rx_packet_32t, 0, sizeof(rx_packet_32t));
  chaos_slot_timing_tx_sum = 0;
  chaos_slot_timing_rx_sum = 0;
  RTIMER_DCO_SYNC();
  chaos_random_generator_init();
	/* Init CHAOS sub-modules */
  chaos_radio_init();
	chaos_log_init();
	//watchdog_stop(); //let's not mess with watchdog config.
  /* init flocklab gpio */
  INIT_PIN_ADC0_OUT; //tx Go
  INIT_PIN_ADC1_OUT; //rx SFD
  INIT_PIN_ADC2_OUT; //rx Go
  INIT_PIN_ADC7_OUT; //round
  INIT_PIN_ADC6_OUT; //associate
  LEDS_BLINK();
//  uint8_t slot_number = 0; //hack for using CHAOS_LOG
//  uint16_t round_number = 0;
  /* delay for 0.5s */
//  int i;
//  for(i = 0; i < 10; i++) {
//    /* ~50ms delay after bootup */
//    chaos_clock_delay(54350);
//  }
}
/*---------------------------------------------------------------------------*/
extern struct process chaos_process;
static int
turn_on(void)
{
  process_start(&chaos_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on)
{
  PRINTF("CHAOS: turn_off not supported\n");
  return 1;
}
/*---------------------------------------------------------------------------*/
const struct mac_driver chaosmac_driver = {
  "ChaosMac",
  chaos_init,
  send_packet,
  packet_input,
  turn_on,
  turn_off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
static void
chaos_net_init(void)
{
  //queuebuf_init();
  //packetbuf_clear();
}
/*---------------------------------------------------------------------------*/
static void
chaos_net_input(void)
{

}
/*---------------------------------------------------------------------------*/
/* empty handler for network layer in Contiki */
const struct network_driver chaosnet_driver = {
  "ChaosNet",
  chaos_net_init,
  chaos_net_input
};

