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
 *         Three-phase commit library
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#include "contiki.h"
#include <string.h>

#include "chaos.h"
#include "chaos-random-generator.h"
#include "node.h"
#include "3pc.h"
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

#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
#define FLAGS_ESTIMATE FLAGS_LEN_X(MAX_NODE_COUNT)
#warning "APP: due to packet size limitation: maximum network size = MAX_NODE_COUNT"
#else
#define FLAGS_ESTIMATE FLAGS_LEN_X(CHAOS_NODES)
#endif

typedef struct __attribute__((packed)) {
  uint32_t value;
  uint8_t phase;
  uint8_t flags_and_votes[];
} three_pc_t;

typedef struct __attribute__((packed)) {
  three_pc_t three_pc;
  uint8_t flags_and_votes[FLAGS_ESTIMATE * 2]; //maximum of 50 * 8 nodes
} three_pc_local_t;

static int tx = 0, did_tx = 0, agree = 0;
static int complete = 0, off_slot, completion_slot, rx_progress = 0;
static int tx_count_complete = 0;
static int invalid_rx_count = 0;
static int got_valid_rx = 0;
static unsigned short restart_threshold;
static three_pc_local_t three_pc_local; /* used only for house keeping and reporting */

#if THREE_PC_LOG_FLAGS
uint8_t chaos_3pc_flags_log[THREE_PC_ROUND_MAX_SLOTS]={0};
uint8_t chaos_3pc_phase_log[THREE_PC_ROUND_MAX_SLOTS]={0};
#endif

static uint8_t bit_count(uint8_t u)
{
  return  (u -(u>>1)-(u>>2)-(u>>3)-(u>>4)-(u>>5)-(u>>6)-(u>>7));
}

int three_pc_get_flags_length() {
  return FLAGS_LEN;
}

static inline int three_pc_get_votes_length() {
  return FLAGS_LEN;
}

static inline uint8_t* three_pc_get_flags(three_pc_t* three_pc) {
  return three_pc->flags_and_votes;
}


static inline uint8_t* three_pc_get_votes(three_pc_t* three_pc) {
  return three_pc->flags_and_votes + three_pc_get_flags_length();
}


static chaos_state_t
process(uint16_t round_count, uint16_t slot_count, chaos_state_t current_state, int chaos_txrx_success, size_t payload_length, uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags)
{
  three_pc_t* tx_three_pc = (three_pc_t*)tx_payload;
  three_pc_t* rx_three_pc = (three_pc_t*)rx_payload;
  chaos_state_t next_state = CHAOS_RX;
  static int pre_commit_slot = -1;

  uint8_t* tx_votes = three_pc_get_votes(tx_three_pc);
  uint8_t* tx_flags = three_pc_get_flags(tx_three_pc);
  unsigned int array_index = chaos_node_index / 8;
  unsigned int array_offset = chaos_node_index % 8;
  int request_sync = 0;

  tx = 0;

  if( current_state == CHAOS_INIT ){
    pre_commit_slot = -1;

    /* set my flag */
    tx_flags[array_index] |= 1 << (array_offset);

    /* initiator vote */
    if(IS_INITIATOR()) {
      //vote for the proposal: set my vote
      agree = 1;
      tx_votes[array_index] |= 1 << (array_offset);
    }
  } else {
    /* do we timeout on pre-commit; thus transition to abort?? */
    int remaining_slots = THREE_PC_ROUND_MAX_SLOTS - slot_count;
    int timeout = (30 > remaining_slots) && (pre_commit_slot != -1); //XXX HARD LIMIT
    if( IS_INITIATOR() && tx_three_pc->phase == PHASE_PRE_COMMIT && timeout){
      COOJA_DEBUG_PRINTF("PHASE_PRE_COMMIT sc %d pc %d", slot_count, pre_commit_slot);
      tx_three_pc->phase = PHASE_ABORT_TIMEOUT;
      //reset all, set own flag and vote
      memset(tx_flags, 0, three_pc_get_flags_length());
      memset(tx_votes, 0, three_pc_get_votes_length());
      tx_flags[array_index] |= 1 << (array_offset);
      tx_votes[array_index] |= 1 << (array_offset);
      tx = 1;
    } else if( current_state == CHAOS_RX && chaos_txrx_success ){
      if( !got_valid_rx ){
        //&& tx_three_pc->phase == PHASE_PROPOSE && rx_three_pc->phase == PHASE_PROPOSE -> not required to check this, this should hold
        //first valid packet: let's vote
        //compare local value and received value: if equal: accept proposal by voting for it, either vote against it
        if( tx_three_pc->value == rx_three_pc->value ){
          //vote for the proposal: set my vote
          agree = 1;
          tx_votes[array_index] |= 1 << (array_offset);
        } else {
          //do not vote for proposal (= vote against it), but send the received value
          agree = 0;
          tx_votes[array_index] &= ~((uint8_t)1 << (array_offset));
          tx_three_pc->value = rx_three_pc->value;
        }
      }

      //be careful: do not mix the different phases
      if( tx_three_pc->phase == rx_three_pc->phase ){
        COOJA_DEBUG_STR("ph ==");

        uint16_t tx_flag_sum = 0, rx_flag_sum = 0;
        uint16_t vote_sum = 0;
        uint8_t* rx_flags = three_pc_get_flags(rx_three_pc);
        uint8_t* rx_votes = three_pc_get_votes(rx_three_pc);
        //merge and tx if flags differ
        int i;
        for( i = 0; i < FLAGS_LEN; i++){
          tx |= (rx_flags[i] != tx_flags[i]);
          tx_flags[i] |= rx_flags[i];
          tx_votes[i] |= rx_votes[i];
          tx_flag_sum += tx_flags[i];
          rx_flag_sum += rx_flags[i];
          vote_sum += tx_votes[i];
        }
        if( tx_flag_sum == FLAG_SUM ){
          tx = 1; /* transmit aggressively */
          if( tx_three_pc->phase == PHASE_PRE_COMMIT || tx_three_pc->phase == PHASE_PROPOSE ){
            /* XXX: Only the initiator can decide to move to next phase */
            if(IS_INITIATOR()) {
              //reset all, set own flag and vote
              memset(tx_flags, 0, three_pc_get_flags_length());
              memset(tx_votes, 0, three_pc_get_votes_length());
              tx_flags[array_index] |= 1 << (array_offset);
              tx_votes[array_index] |= 1 << (array_offset);
              //everybody voted -> next phase
              if( vote_sum == FLAG_SUM ){
                //everybody voted to commit
                tx_three_pc->phase += 1;
                //record time of pre-commit to enable the special timeout behavior
                if(tx_three_pc->phase == PHASE_PRE_COMMIT && pre_commit_slot == -1){
                  pre_commit_slot = slot_count;
                }
              } else {
                //not everybody voted to agree -> abort
                tx_three_pc->phase = PHASE_ABORT;
              }

            }
            leds_on(LEDS_GREEN);

          } else if( tx_three_pc->phase == PHASE_COMMIT ){
            if( vote_sum == FLAG_SUM ){
              //everybody voted to commit --> final flood
              if(!complete){
                completion_slot = slot_count;
              }
              complete = 1;
              rx_progress |= (rx_flag_sum == FLAG_SUM); /* received a complete packet */
            } else {
              //XXX this should not happen... not everybody have committed -> FAIL
              memset(tx_flags, 0, three_pc_get_flags_length());
              memset(tx_votes, 0, three_pc_get_votes_length());
              tx_flags[array_index] |= 1 << (array_offset);
              tx_votes[array_index] |= 1 << (array_offset);
              tx_three_pc->phase = PHASE_FAIL;
            }
          } else if( tx_three_pc->phase == PHASE_ABORT || tx_three_pc->phase == PHASE_ABORT_TIMEOUT || tx_three_pc->phase == PHASE_FAIL ){
            //final phases: all flags are set? -> time for final flood and turning off
            //Final flood: transmit result aggressively
            if(!complete){
              completion_slot = slot_count;
            }
            complete = 1;
            rx_progress |= (rx_flag_sum == FLAG_SUM); /* received a complete packet */
          }
        }
      } else if( tx_three_pc->phase < rx_three_pc->phase ){
        did_tx = 0;
        request_sync = 1;
        //received phase is more advanced than local one -> switch to received state (and set own flags)
        memcpy(tx_three_pc, rx_three_pc, sizeof(three_pc_t) + three_pc_get_flags_length() + three_pc_get_votes_length());
        tx_flags[array_index] |= 1 << (array_offset);
        COOJA_DEBUG_STR("ph <");
        // do I like the proposal?
        if( tx_three_pc->value == rx_three_pc->value ){
          //vote for the proposal: set my vote
          tx_votes[array_index] |= 1 << (array_offset);
        } else {
          //do not vote for proposal (= vote against it), but send the received value
          tx_votes[array_index] &= ~((uint8_t)1 << (array_offset));
          tx_three_pc->value = rx_three_pc->value;
        }
        tx = 1;
        leds_on(LEDS_BLUE);
      } else if( /* IS_INITIATOR() && */tx_three_pc->phase > rx_three_pc->phase ){ //tx_three_pc->phase > rx_three_pc->phase
        //local phase is more advanced. Drop received one and just transmit to allow others to catch up
        tx = 1;
        COOJA_DEBUG_STR("ph >");
      }
    }
  }

  /* decide next state */
  if(current_state == CHAOS_INIT) {
    if(IS_INITIATOR()) {
      got_valid_rx = 1;      //to enable retransmissions
      next_state = CHAOS_TX; //for the first tx of the initiator: no increase of tx_count here
    } else {
      got_valid_rx = 0;
      next_state = CHAOS_RX;
    }
  } else if(current_state == CHAOS_RX && chaos_txrx_success) {
    got_valid_rx = 1;
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

  COOJA_DEBUG_STR(PHASE_TO_STR(tx_three_pc->phase));

  *app_flags = tx_three_pc->flags_and_votes;

#if FAILURES_RATE
#warning "INJECT_FAILURES!!"
  if(chaos_random_generator_fast() < 1*(CHAOS_RANDOM_MAX/(FAILURES_RATE))){
    next_state = CHAOS_OFF;
  }
#endif

  int end = (slot_count >= THREE_PC_ROUND_MAX_SLOTS - 1) || (next_state == CHAOS_OFF);

  if(end){
    //next_state = CHAOS_OFF;
    memcpy(&three_pc_local,tx_three_pc, sizeof(three_pc_t) + three_pc_get_flags_length() + three_pc_get_votes_length());
    off_slot = slot_count;
  }

#if THREE_PC_LOG_FLAGS
  chaos_3pc_phase_log[slot_count]=tx_three_pc->phase;
  //if(current_state == CHAOS_RX && chaos_txrx_success)
  {
    uint8_t i;
    //chaos_3pc_flags_log[slot]=0;
    for(i=0; i<FLAGS_LEN; i++){
      chaos_3pc_flags_log[slot_count] += bit_count(tx_flags[i]);
    }
  }
#endif /* CHAOS_LOG_FLAGS */

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

int three_pc_is_pending(const uint16_t round_count){
  return 1;
}

int three_pc_agreed(){
  return agree;
}

int three_pc_did_tx(){
  return did_tx;
}

uint16_t three_pc_get_off_slot(){
  return off_slot;
}

int three_pc_round_begin(const uint16_t round_number, const uint8_t app_id, uint32_t* three_pc_value, uint8_t* phase, uint8_t** final_flags)
{
  tx = 0;
  did_tx = 0;
  agree = 0;
  got_valid_rx = 0;
  complete = 0;
  tx_count_complete = 0;
  invalid_rx_count = 0;
  off_slot = THREE_PC_ROUND_MAX_SLOTS;
  completion_slot = 0;
  rx_progress = 0;

  /* init random restart threshold */
  restart_threshold = chaos_random_generator_fast() % (CHAOS_RESTART_MAX - CHAOS_RESTART_MIN) + CHAOS_RESTART_MIN;

#if THREE_PC_LOG_FLAGS
  memset(&chaos_3pc_flags_log, 0, sizeof(chaos_3pc_flags_log));
  //memset(&chaos_3pc_phase_log, 0, sizeof(chaos_3pc_phase_log));
#endif

  memset(&three_pc_local, 0, sizeof(three_pc_local));
  three_pc_local.three_pc.value = *three_pc_value;
  three_pc_local.three_pc.phase = PHASE_PROPOSE;
  /* set my flag */
  unsigned int array_index = chaos_node_index / 8;
  unsigned int array_offset = chaos_node_index % 8;
  uint8_t* flags = three_pc_get_flags(&three_pc_local.three_pc);
  flags[array_index] |= 1 << (array_offset);

  //XXX using 0 as THREE_PC_ROUND_MAX_SLOTS
  chaos_round(round_number, app_id, (const uint8_t const*)&three_pc_local, sizeof(three_pc_local.three_pc) + three_pc_get_flags_length() + three_pc_get_votes_length(), THREE_PC_SLOT_LEN_DCO, THREE_PC_ROUND_MAX_SLOTS, three_pc_get_flags_length(), process);
  *three_pc_value = three_pc_local.three_pc.value;
  *final_flags = three_pc_local.flags_and_votes;
  *phase = three_pc_local.three_pc.phase;
  return completion_slot;
}

