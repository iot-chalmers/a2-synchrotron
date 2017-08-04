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
 *         Two-phase commit library
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#include "contiki.h"
#include <string.h>

#include "chaos.h"
#include "chaos-random-generator.h"
#include "node.h"
#include "2pc.h"
#include "chaos-config.h"

#ifndef FAILURES_RATE
#define FAILURES_RATE 0
#endif

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

#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
#define FLAGS_ESTIMATE FLAGS_LEN_X(MAX_NODE_COUNT)
#else
#define FLAGS_ESTIMATE FLAGS_LEN_X(CHAOS_NODES)
#endif

typedef struct __attribute__((packed)) {
  uint32_t value;
  uint8_t phase;
  uint8_t flags_and_votes[];
} two_pc_t;

typedef struct __attribute__((packed)) {
  two_pc_t two_pc;
  uint8_t flags_and_votes[FLAGS_ESTIMATE * 2]; //maximum of 50 * 8 nodes
} two_pc_local_t;

static int tx = 0, did_tx = 0, agree = 0;
static int complete = 0, off_slot, completion_slot, rx_progress = 0;
static int tx_count_complete = 0;
static int invalid_rx_count = 0;
static int got_valid_rx = 0;
static unsigned short restart_threshold;
static two_pc_local_t two_pc_local; /* used only for house keeping and reporting */
static uint8_t* tx_flags_final = 0;

int two_pc_get_flags_length() {
  return FLAGS_LEN;
}

static inline int two_pc_get_votes_length() {
  return FLAGS_LEN;
}

static inline uint8_t* two_pc_get_flags(two_pc_t* two_pc) {
  return two_pc->flags_and_votes;
}


static inline uint8_t* two_pc_get_votes(two_pc_t* two_pc) {
  return two_pc->flags_and_votes + two_pc_get_flags_length();
}


static chaos_state_t
process(uint16_t round_count, uint16_t slot_count, chaos_state_t current_state, int chaos_txrx_success, size_t payload_length, uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags)
{
  two_pc_t* tx_two_pc = (two_pc_t*)tx_payload;
  two_pc_t* rx_two_pc = (two_pc_t*)rx_payload;
  chaos_state_t next_state = CHAOS_RX;

  uint8_t* votes = two_pc_get_votes(tx_two_pc);
  unsigned int array_index = chaos_node_index / 8;
  unsigned int array_offset = chaos_node_index % 8;

  /* the application reports a packet coming from the initiator, so we can synchronize on it;
   * e.g., we got a phase transition that only the initiator can issue */
  int request_sync = 0;

  if( IS_INITIATOR() && current_state == CHAOS_INIT ){
    next_state = CHAOS_TX; //for the first tx of the initiator: no increase of tx_count here
    got_valid_rx = 1;      //to enable retransmissions
    //vote for the proposal: set my vote
    votes[array_index] |= 1 << (array_offset);
    agree = 1;
  } else if(current_state == CHAOS_RX && chaos_txrx_success) {
    if( !got_valid_rx ){
      //&& tx_two_pc->phase == PHASE_PROPOSE && rx_two_pc->phase == PHASE_PROPOSE -> not required to check this, this should hold
      //first valid packet: let's vote
      //compare local value and received value: if equal: accept proposal by voting for it, either vote against it
      if( tx_two_pc->value == rx_two_pc->value ){
        //vote for the proposal: set my vote
        agree = 1;
        votes[array_index] |= 1 << (array_offset);
      } else {
        //do not vote for proposal (= vote against it), but send the received value
        agree = 0; //Unnecessary
        votes[array_index] &= ~((uint8_t)1 << (array_offset));
        tx_two_pc->value = rx_two_pc->value;
      }
    }
    got_valid_rx = 1;
    tx = 0;

    //be careful: do not mix the different phases
    if( tx_two_pc->phase == rx_two_pc->phase ){
      uint16_t flag_sum = 0;
      uint16_t vote_sum = 0;
      uint16_t rx_flag_sum = 0;
      int i;
      uint8_t* tx_flags = two_pc_get_flags(tx_two_pc);
      uint8_t* rx_flags = two_pc_get_flags(rx_two_pc);
      uint8_t* tx_votes = two_pc_get_votes(tx_two_pc);
      uint8_t* rx_votes = two_pc_get_votes(rx_two_pc);
      //merge and tx if flags differ
      for( i = 0; i < FLAGS_LEN; i++){
        tx |= (rx_flags[i] != tx_flags[i]);
        tx_flags[i] |= rx_flags[i];
        tx_votes[i] |= rx_votes[i];
        flag_sum += tx_flags[i];
        vote_sum += tx_votes[i];
        rx_flag_sum += rx_flags[i];
      }

      if( IS_INITIATOR() && tx_two_pc->phase == PHASE_PROPOSE && flag_sum == FLAG_SUM  ){
        //everybody voted -> next phase
        //reset all, set own flag
        LEDS_ON(LEDS_RED);
        memset(tx_flags, 0, two_pc_get_flags_length());
        unsigned int array_index = chaos_node_index / 8;
        unsigned int array_offset = chaos_node_index % 8;
        tx_flags[array_index] |= 1 << (array_offset);
        if( vote_sum == FLAG_SUM ){
          //everybody voted to commit
          tx_two_pc->phase = PHASE_COMMIT;
        } else {
          //not everybody voted to agree -> abort
          tx_two_pc->phase = PHASE_ABORT;
        }
        tx = 1;
        leds_on(LEDS_GREEN);
      } else if( flag_sum == FLAG_SUM && (tx_two_pc->phase == PHASE_COMMIT ||  tx_two_pc->phase == PHASE_ABORT) ){
        //final phases: all flags are set? -> time for final flood and turning off
        //Final flood: transmit result aggressively
        tx = 1;
        if(!complete){
          completion_slot = slot_count;
        }
        complete = 1;
        rx_progress |= (rx_flag_sum == FLAG_SUM); /* received a complete packet */
      }
    } else if( tx_two_pc->phase < rx_two_pc->phase ){
      //received phase is more advanced than local one -> switch to received state (and set own flags)
      memcpy(tx_two_pc, rx_two_pc, sizeof(two_pc_t) + two_pc_get_flags_length() + two_pc_get_votes_length());
      uint8_t* tx_flags = two_pc_get_flags(tx_two_pc);
      unsigned int array_index = chaos_node_index / 8;
      unsigned int array_offset = chaos_node_index % 8;
      tx_flags[array_index] |= 1 << (array_offset);
      tx = 1;
      leds_on(LEDS_BLUE);
      request_sync = 1;
    } else { //tx_two_pc->phase > rx_two_pc->phase
      //local phase is more advanced. Drop received one and just transmit to allow others to catch up
      tx = 1;
    }
    if( tx ){
      next_state = CHAOS_TX;
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
  } else if(current_state == CHAOS_TX && (rx_progress || !RELIABLE_FF) && tx_count_complete >= N_TX_COMPLETE){
    next_state = CHAOS_OFF;
    leds_off(LEDS_GREEN);
  }

  *app_flags = tx_two_pc->flags_and_votes;

#if FAILURES_RATE
#warning "INJECT_FAILURES!!"
  if(/*tx_two_pc->phase == PHASE_PROPOSE && */chaos_random_generator_fast() < 1*(CHAOS_RANDOM_MAX/(FAILURES_RATE))){
    next_state = CHAOS_OFF;
  }
#endif

  int end = (slot_count >= TWO_PC_ROUND_MAX_SLOTS - 1) || (next_state == CHAOS_OFF);
  if(end){
    //memcpy(&two_pc_local,tx_two_pc, sizeof(two_pc_t) + two_pc_get_flags_length() + two_pc_get_votes_length());
    two_pc_local.two_pc.value = tx_two_pc->value;
    two_pc_local.two_pc.phase = tx_two_pc->phase;
    tx_flags_final = tx_two_pc->flags_and_votes;

    off_slot = slot_count;
  }

  if( next_state == CHAOS_TX ){
    did_tx = 1;
  }

  if( request_sync ){
    if( next_state == CHAOS_TX ){
      next_state = CHAOS_TX_SYNC;
    } else if( next_state == CHAOS_RX ){
      next_state = CHAOS_RX_SYNC;
    }
  }

  return next_state;
}

int two_pc_is_pending(const uint16_t round_count){
  return 1;
}

int two_pc_agreed(){
  return agree;
}

int two_pc_did_tx(){
  return did_tx;
}

uint16_t two_pc_get_off_slot(){
  return off_slot;
}

int two_pc_round_begin(const uint16_t round_number, const uint8_t app_id, uint32_t* two_pc_value, uint8_t* phase, uint8_t** final_flags)
{
  tx = 0;
  did_tx = 0;
  agree = 0;
  got_valid_rx = 0;
  complete = 0;
  tx_count_complete = 0;
  invalid_rx_count = 0;
  off_slot = TWO_PC_ROUND_MAX_SLOTS;
  completion_slot = 0;
  tx_flags_final = 0;
  rx_progress = 0;

  /* init random restart threshold */
  restart_threshold = chaos_random_generator_fast() % (CHAOS_RESTART_MAX - CHAOS_RESTART_MIN) + CHAOS_RESTART_MIN;

  memset(&two_pc_local, 0, sizeof(two_pc_local));
  two_pc_local.two_pc.value = *two_pc_value;
  two_pc_local.two_pc.phase = PHASE_PROPOSE;
  /* set my flag */
  unsigned int array_index = chaos_node_index / 8;
  unsigned int array_offset = chaos_node_index % 8;
  uint8_t* flags = two_pc_get_flags(&two_pc_local.two_pc);
  flags[array_index] |= 1 << (array_offset);

  chaos_round(round_number, app_id, (const uint8_t const*)&two_pc_local, sizeof(two_pc_local.two_pc) + two_pc_get_flags_length() + two_pc_get_votes_length(), TWO_PC_SLOT_LEN_DCO, TWO_PC_ROUND_MAX_SLOTS, two_pc_get_flags_length(), process);
  memcpy(two_pc_local.two_pc.flags_and_votes,tx_flags_final, two_pc_get_flags_length() + two_pc_get_votes_length());
  *two_pc_value = two_pc_local.two_pc.value;
  *final_flags = two_pc_local.flags_and_votes;
  *phase = two_pc_local.two_pc.phase;
  return completion_slot;
}

