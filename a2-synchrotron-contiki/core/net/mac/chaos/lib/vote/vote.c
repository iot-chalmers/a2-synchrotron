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
 *         Single-phase Vote library
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#include "contiki.h"
#include <string.h>

#include "chaos.h"
#include "chaos-random-generator.h"
#include "node.h"
#include "vote.h"
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

#define LIMIT_TX_NO_DELTA 0

#define FLAGS_LEN_X(X)   (((X) >> 3) + (((X) & 7) ? 1 : 0))
#define FLAGS_LEN   (FLAGS_LEN_X(chaos_node_count))
//#define LAST_FLAGS  ((1 << (((chaos_node_count - 1) % 8) + 1)) - 1)
#define LAST_FLAGS  ((1 << (((chaos_node_count - 1) & 7) + 1)) - 1)
//#define FLAG_SUM    (((FLAGS_LEN - 1) * 0xFF) + LAST_FLAGS)
#define FLAG_SUM    (((FLAGS_LEN - 1) << 8) - (FLAGS_LEN - 1) + LAST_FLAGS)

#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
#define FLAGS_ESTIMATE FLAGS_LEN_X(MAX_NODE_COUNT)
#else
#define FLAGS_ESTIMATE FLAGS_LEN_X(CHAOS_NODES)
#endif

//#if (CHAOS_APP_PAYLOAD_LEN-FLAGS_ESTIMATE-4) > 0
//#define APP_DUMMY_LEN MIN(CHAOS_APP_PAYLOAD_LEN-FLAGS_ESTIMATE-sizeof(uint32_t), CHAOS_MAX_PAYLOAD_LEN-FLAGS_ESTIMATE-sizeof(uint32_t))
//#else
//#define APP_DUMMY_LEN 0
//#endif

typedef struct __attribute__((packed)) vote_t_struct {
  uint32_t proposal:31, vote:1;
  uint8_t flags[];
} vote_t;

typedef struct __attribute__((packed)) vote_t_local_struct {
  vote_t vote;
  uint8_t flags[FLAGS_ESTIMATE];
} vote_t_local;

static int tx = 0, agree = 0, did_tx = 0;
static uint16_t complete = 0, off_slot, completion_slot,
    rx_progress = 0; /* signals we received more information than what we have or a complete packet */

static int tx_count_complete = 0;
static int invalid_rx_count = 0;
static int got_valid_rx = 0;
static unsigned short restart_threshold;
static vote_t_local vote_local; /* used only for house keeping and reporting */
static uint8_t* tx_flags_final = 0;

static uint8_t
do_vote(uint32_t proposal){
  //dummy function to vote on a proposal
  return ( proposal > 0 );
}

static chaos_state_t
process(uint16_t round_count, uint16_t slot_count, chaos_state_t current_state, int chaos_txrx_success, size_t payload_length, uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags)
{
  static uint8_t no_delta_count = 0;
  vote_t* tx_vote = (vote_t*)tx_payload;
  vote_t* rx_vote = (vote_t*)rx_payload;
  uint8_t rx_delta = 0;

  /* merge valid rx data & flags */
  if(!complete && chaos_txrx_success && current_state == CHAOS_RX) {
    if(!got_valid_rx){
      got_valid_rx = 1;
      tx_vote->proposal = rx_vote->proposal;
      agree = do_vote(rx_vote->proposal);
    }
    //reset vote if we do not agree
    tx_vote->vote = rx_vote->vote && agree;


    //merge flags and do tx decision based on flags
    tx = 0;
    uint16_t flag_sum = 0;
    int i;
    for( i = 0; i < FLAGS_LEN; i++){
      COOJA_DEBUG_STR("f");
      tx |= (rx_vote->flags[i] != tx_vote->flags[i]);
      tx_vote->flags[i] |= rx_vote->flags[i];
      flag_sum += tx_vote->flags[i];
    }
    /* compare current rx with last valid rx */
#if LIMIT_TX_NO_DELTA
    rx_delta = memcmp(rx_vote->flags, vote_local.vote.flags, vote_get_flags_length());
    memcpy(&vote_local.vote, rx_vote, sizeof(vote_t) + vote_get_flags_length());
    no_delta_count = rx_delta ? no_delta_count+1 : 0;
#else
    rx_delta = 1;
#endif /* LIMIT_TX_NO_DELTA */

    //all flags are set?
    if( flag_sum >= FLAG_SUM ){
      //Final flood: transmit result aggressively
      tx = 1;
      if(!complete){
        completion_slot = slot_count;
      }
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
      rx_flag_sum += rx_vote->flags[i];
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

  /* reporting progress */
  *app_flags = (current_state == CHAOS_TX) ? tx_vote->flags : rx_vote->flags;

#if FAILURES_RATE
#warning "INJECT_FAILURES!!"
  if(chaos_random_generator_fast() < 1*(CHAOS_RANDOM_MAX/(FAILURES_RATE))){
    next_state = CHAOS_OFF;
  }
#endif

  int end = (slot_count >= VOTE_ROUND_MAX_SLOTS - 1) || (next_state == CHAOS_OFF);
  if(end){
    off_slot = slot_count;
    /* for reporting the final result */
    //memcpy(&vote_local.vote, tx_vote, sizeof(vote_t) + vote_get_flags_length());
    vote_local.vote.proposal = tx_vote->proposal;
    vote_local.vote.vote = tx_vote->vote;
    tx_flags_final = tx_vote->flags;
  }

  if( next_state == CHAOS_TX ){
    did_tx = 1;
  }

  return next_state;
}

int vote_get_flags_length() {
  return FLAGS_LEN;
}

int vote_is_pending(const uint16_t round_count){
  return 1;
}

uint16_t vote_get_off_slot(){
  return off_slot;
}

int vote_did_agree(){
  return agree;
}

int vote_did_tx(){
  return did_tx;
}

int vote_round_begin(const uint16_t round_number, const uint8_t app_id, uint32_t* proposal_value, uint8_t* vote_value, uint8_t** final_flags)
{
  tx = 0;
  agree = 0; did_tx = 0;
  got_valid_rx = 0;
  complete = 0;
  tx_count_complete = 0;
  invalid_rx_count = 0;
  off_slot = VOTE_ROUND_MAX_SLOTS;
  completion_slot = 0;
  rx_progress = 0;

  /* init random restart threshold */
  restart_threshold = chaos_random_generator_fast() % (CHAOS_RESTART_MAX - CHAOS_RESTART_MIN) + CHAOS_RESTART_MIN;

  memset(&vote_local, 0, sizeof(vote_local));
  if( IS_INITIATOR() ){
    vote_local.vote.proposal = *proposal_value;
    vote_local.vote.vote = do_vote(*proposal_value);
    agree = vote_local.vote.vote;
  }
  /* set my flag */
  unsigned int array_index = chaos_node_index / 8;
  unsigned int array_offset = chaos_node_index % 8;
  vote_local.vote.flags[array_index] |= 1 << (array_offset);

  chaos_round(round_number, app_id, (const uint8_t const*)&vote_local.vote, sizeof(vote_t) + vote_get_flags_length(), VOTE_SLOT_LEN_DCO, VOTE_ROUND_MAX_SLOTS, vote_get_flags_length(), process);
  *proposal_value = vote_local.vote.proposal;
  *vote_value = vote_local.vote.vote;
  memcpy(vote_local.flags, tx_flags_final, vote_get_flags_length());
  *final_flags = vote_local.flags;
  return completion_slot;
}
