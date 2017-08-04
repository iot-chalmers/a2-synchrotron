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
/*
 * \file
 *         Collect library
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#include "contiki.h"
#include <string.h>
#include "chaos-collect.h"

#include "chaos.h"
#include "chaos-random-generator.h"
#include "node.h"
#include "chaos-config.h"

#undef ENABLE_COOJA_DEBUG
#define ENABLE_COOJA_DEBUG COOJA
#include "dev/cooja-debug.h"

/* Continue final flood until we receive a full packet? */
#define RELIABLE_FF 1

#ifndef N_TX_COMPLETE
#define N_TX_COMPLETE 9
#endif

#ifndef CHAOS_RESTART_MIN
#define CHAOS_RESTART_MIN 6
#endif

#ifndef CHAOS_RESTART_MAX
#define CHAOS_RESTART_MAX 10
#endif

#define FLAGS_LEN_X(X)   (((X) >> 3) + (((X) & 7) ? 1 : 0))
#define FLAGS_LEN   (FLAGS_LEN_X(chaos_node_count))
//#define LAST_FLAGS  ((1 << (((chaos_node_count - 1) % 8) + 1)) - 1)
#define LAST_FLAGS  ((1 << (((chaos_node_count - 1) & 7) + 1)) - 1)
//#define FLAG_SUM    (((FLAGS_LEN - 1) * 0xFF) + LAST_FLAGS)
#define FLAG_SUM    (((FLAGS_LEN - 1) << 8) - (FLAGS_LEN - 1) + LAST_FLAGS)

#define CHECK_FLAG(FLAGS, INDEX, OFFSET) (((FLAGS)[INDEX]) & (1 << (OFFSET)))
#define SET_FLAG(FLAGS, INDEX, OFFSET) (((FLAGS)[INDEX]) |= (1 << (OFFSET)))
#define NODE_INDEX_TO_FLAG_INDEX(I) ((I)/8)
#define NODE_INDEX_TO_FLAG_OFFSET(I) ((I)%8)

#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
#define NODES_ESTIMATE (MAX_NODE_COUNT)
#warning "APP: due to packet size limitation: maximum network size = MAX_NODE_COUNT"
#else
#define NODES_ESTIMATE (CHAOS_NODES)
#endif

#define FLAGS_ESTIMATE (FLAGS_LEN_X(NODES_ESTIMATE))

typedef struct __attribute__((packed)) {
  uint8_t values_and_flags[0]; //sizeof(collect_value_t)+1
  //uint8_t* values_and_flags_[];
} chaos_collect_t;

typedef struct __attribute__((packed)) {
  chaos_collect_t collect_store;
  union {
    struct __attribute__((packed)) {
      collect_value_t values[NODES_ESTIMATE];
      uint8_t flags[FLAGS_ESTIMATE];
    };
  };
} chaos_collect_local_t;

static uint16_t completion_slot, off_slot;
static int tx = 0;
static int complete = 0,
rx_progress = 0; /* signals we received more information than what we have or a complete packet */

static int tx_count_complete = 0;
static int invalid_rx_count = 0;
static int got_valid_rx = 0;
static unsigned short restart_threshold;
static chaos_collect_local_t collect_store_local; /* used only for house keeping and reporting */

int chaos_collect_get_flags_length() {
  return FLAGS_LEN;
}

static inline int chaos_collect_get_values_length() {
  return chaos_node_count * sizeof(collect_value_t);
}

int chaos_collect_get_all_length() {
  return /* sizeof(chaos_collect_t) +*/ chaos_collect_get_flags_length() + chaos_collect_get_values_length();
}

static inline uint8_t* chaos_collect_get_flags(chaos_collect_t* cc) {
  return cc->values_and_flags + chaos_collect_get_values_length();
}

static inline collect_value_t* chaos_collect_get_values(chaos_collect_t* cc) {
  return (collect_value_t*)cc->values_and_flags;
}

static chaos_state_t
process(uint16_t round_count, uint16_t slot_count, chaos_state_t current_state, int chaos_txrx_success, size_t payload_length, uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags)
{
  chaos_collect_t* tx_store = (chaos_collect_t*)tx_payload; /* stores node state as well */
  chaos_collect_t* rx_store = (chaos_collect_t*)rx_payload;
  static uint8_t no_delta_count = 0;
  uint8_t rx_delta = 0;

  collect_value_t* tx_values = chaos_collect_get_values(tx_store);
  collect_value_t* rx_values = chaos_collect_get_values(rx_store);
  uint8_t* tx_flags = chaos_collect_get_flags(tx_store);
  uint8_t* rx_flags = chaos_collect_get_flags(rx_store);

  /* merge valid rx data & flags */
  if(!complete && chaos_txrx_success && current_state == CHAOS_RX) {
    got_valid_rx = 1;
    tx = 0;
    int i;
    //merge collected values
    for( i = 0; i < CHAOS_NODES; i++ ) {
      uint8_t rx_set = CHECK_FLAG(rx_flags, NODE_INDEX_TO_FLAG_INDEX(i), NODE_INDEX_TO_FLAG_OFFSET(i));
      //uint8_t tx_set = CHECK_FLAG(tx_flags, NODE_INDEX_TO_FLAG_INDEX(i), NODE_INDEX_TO_FLAG_OFFSET(i));

      if (rx_set /* && !tx_set */){
        tx_values[i] = rx_values[i];
      }
    }

    //merge flags and do tx decision based on flags
    uint16_t flag_sum = 0;
    for( i = 0; i < FLAGS_LEN; i++){
      COOJA_DEBUG_STR("f");
      tx |= (rx_flags[i] != tx_flags[i]);
      tx_flags[i] |= rx_flags[i];
      flag_sum += tx_flags[i];
    }

    /* compare current rx with last valid rx */
#if LIMIT_TX_NO_DELTA
    rx_delta = memcmp(rx_flags, chaos_collect_get_flags(collect_store_local), chaos_collect_get_flags_length());
    memcpy(&collect_store_local.collect_store, rx_store, sizeof(chaos_collect_t) + chaos_collect_get_flags_length() + chaos_collect_get_values_length());
    no_delta_count = rx_delta ? no_delta_count+1 : 0;
#else
    rx_delta = 1;
#endif /* LIMIT_TX_NO_DELTA */

    //all flags are set?
    if( flag_sum >= FLAG_SUM ){
      //Final flood: transmit result aggressively
      if(!complete){ //store when we reach completion
        completion_slot = slot_count;
      }
      tx = 1;
      complete = 1;
      LEDS_ON(LEDS_GREEN);
    }
  } else if(complete && chaos_txrx_success && current_state == CHAOS_RX){
    //merge flags and do tx decision based on flags
    //tx = 0;
    int i;
    uint16_t rx_flag_sum = 0;
    for( i = 0; i < FLAGS_LEN; i++){
      COOJA_DEBUG_STR("f c");
      rx_flag_sum += rx_flags[i];
    }
    rx_progress |= (rx_flag_sum >= FLAG_SUM); /* received a complete packet */
  }

  /* decide next state */
  chaos_state_t next_state = CHAOS_RX;
  if( IS_INITIATOR() && current_state == CHAOS_INIT ){
    next_state = CHAOS_TX; //for the first tx of the initiator: no increase of tx_count here
    got_valid_rx = 1; //to enable retransmissions
  } else if(current_state == CHAOS_RX && chaos_txrx_success){
    invalid_rx_count = 0;
    if( tx ){
      /* if we have not received a delta, then we limit tx rate */
      next_state = (complete || rx_delta) ? CHAOS_TX : ((chaos_random_generator_fast() & (1024-1)) > 1024/(no_delta_count+1) ? CHAOS_TX : CHAOS_RX);
      if( complete ){
        tx_count_complete++;
      }
    }
  } else if(current_state == CHAOS_RX && !chaos_txrx_success && got_valid_rx){
    invalid_rx_count++;
    if(invalid_rx_count > restart_threshold){
      next_state = CHAOS_TX;
      invalid_rx_count = 0;
      if( complete ){
        tx_count_complete++;
      }
      restart_threshold = chaos_random_generator_fast() % (CHAOS_RESTART_MAX - CHAOS_RESTART_MIN) + CHAOS_RESTART_MIN;
    }
  } else if(current_state == CHAOS_TX && !chaos_txrx_success){ /* we missed tx go time. Retry */
    got_valid_rx = 1; //????? check me!
    next_state = CHAOS_TX;
  } else if(current_state == CHAOS_TX && (rx_progress || !RELIABLE_FF) && tx_count_complete >= N_TX_COMPLETE){
    next_state = CHAOS_OFF;
    LEDS_OFF(LEDS_GREEN);
  }

  /* for reporting the final result */
  if(complete || slot_count >= CHAOS_COLLECT_ROUND_MAX_SLOTS - 1) {
    memcpy(&collect_store_local.collect_store, tx_store, chaos_collect_get_all_length());
  }

  /* reporting progress */
  *app_flags = (current_state == CHAOS_TX) ? tx_flags : rx_flags;

  int end = (slot_count >= CHAOS_COLLECT_ROUND_MAX_SLOTS - 1) || (next_state == CHAOS_OFF);
  if(end){
    off_slot = slot_count;
  }

  return next_state;
}

uint16_t chaos_collect_get_off_slot(){
  return off_slot;
}

int chaos_collect_is_pending(const uint16_t round_count){
  return 1;
}

int chaos_collect_round_begin(const uint16_t round_number, const uint8_t app_id, collect_value_t** final_values, uint8_t** final_flags)
{
  off_slot = CHAOS_COLLECT_ROUND_MAX_SLOTS;
  completion_slot = 0;
  tx = 0;
  got_valid_rx = 0;
  complete = 0;
  tx_count_complete = 0;
  invalid_rx_count = 0;
  rx_progress = 0;

  /* init random restart threshold */
  restart_threshold = chaos_random_generator_fast() % (CHAOS_RESTART_MAX - CHAOS_RESTART_MIN) + CHAOS_RESTART_MIN;

  memset(&collect_store_local, 0, sizeof(collect_store_local));

  /* set my flag and value */
  unsigned int flags_index = NODE_INDEX_TO_FLAG_INDEX(chaos_node_index);
  unsigned int flags_offset = NODE_INDEX_TO_FLAG_OFFSET(chaos_node_index);
  uint8_t* flags = chaos_collect_get_flags(&collect_store_local.collect_store);
  collect_value_t* values = chaos_collect_get_values(&collect_store_local.collect_store);
  SET_FLAG(flags, flags_index, flags_offset);
  values[chaos_node_index] = **final_values; /* the first value is ours */

  chaos_round(round_number, app_id, (const uint8_t const*)&collect_store_local, chaos_collect_get_all_length(), CHAOS_COLLECT_SLOT_LEN_DCO, CHAOS_COLLECT_ROUND_MAX_SLOTS, chaos_collect_get_flags_length(), process);
  *final_values = chaos_collect_get_values(&collect_store_local.collect_store);
  *final_flags = chaos_collect_get_flags(&collect_store_local.collect_store);

  return completion_slot;
}

