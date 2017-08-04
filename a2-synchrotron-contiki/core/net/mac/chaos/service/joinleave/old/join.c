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
 * join.c
 *
 *  Created on: Jun 12, 2015
 *      Author: olaf
 */

#include "contiki.h"
#include <string.h>
#include <stdio.h>

#include "chaos.h"
#include "chaos-random-generator.h"
#include "chaos-control.h"
#include "node.h"
#include "join.h"
#include "testbed.h"
#include "chaos-config.h"

#define LIMIT_TX_NO_DELTA 1
#define LIMIT_TX_FOR_DETACHED_NODES 1

#define JOIN_STRESS_TESTING           (1)

#define ENABLE_COOJA_DEBUG COOJA
#include "dev/cooja-debug.h"

#define JOIN_SLOT_LEN          (8*(RTIMER_SECOND/1000))    //TODO needs calibration
#define JOIN_SLOT_LEN_DCO      (JOIN_SLOT_LEN*CLOCK_PHI)    //TODO needs calibration

#if TESTBED == flocklab || TESTBED == motes || TESTBED == cooja
#define JOIN_ROUND_MAX_SLOTS   (80)
#define JOIN_STRESS_TESTING_THRESHOLD (4)
#else
#define JOIN_ROUND_MAX_SLOTS   (110)
#define JOIN_STRESS_TESTING_THRESHOLD (10)
#endif

#ifndef JOIN_ROUNDS_AFTER_BOOTUP
#define JOIN_ROUNDS_AFTER_BOOTUP 5
#endif

#ifndef NODE_LIST_LEN
  #if TESTBED == motes
    #define NODE_LIST_LEN 9  //describes how many nodes can join in a single round
  #else
    #define NODE_LIST_LEN 20  //describes how many nodes can join in a single round
  #endif
#endif

#ifndef COMMIT_THRESHOLD
#define COMMIT_THRESHOLD (5)
#endif

#ifndef N_TX_COMPLETE
#define N_TX_COMPLETE 3
#endif

#ifndef CHAOS_RESTART_MIN
#define CHAOS_RESTART_MIN 3
#endif

#ifndef CHAOS_RESTART_MAX
#define CHAOS_RESTART_MAX 6
#endif

#define FLAGS_LEN(node_count)   ((node_count / 8) + ((node_count % 8) ? 1 : 0))
#define LAST_FLAGS(node_count)  ((1 << ((((node_count) - 1) % 8) + 1)) - 1)
#define FLAG_SUM(node_count)  ((((node_count) - 1) / 8 * 0xFF) + LAST_FLAGS(node_count))

typedef uint16_t node_id_t;
typedef uint8_t node_index_t;

typedef struct __attribute__((packed)) {
  node_id_t slot[NODE_LIST_LEN];              //slots to write node ids into
  node_index_t index[NODE_LIST_LEN]; /* assigned indices */
  uint8_t flags[FLAGS_LEN(MAX_NODE_COUNT)];  //flags used to confirm the commit
  uint8_t                   //control flags
  overflow :1,              /* available join slots too short */
  commit :2,                /* commit join */
  slot_count :5;       //number of slots into which nodes wrote their ids
  uint8_t node_count;
} join_t;

typedef struct __attribute__((packed)) {
  node_id_t slot[NODE_LIST_LEN*2];              //slots to write node ids into
  node_index_t index[NODE_LIST_LEN*2]; /* assigned indices */
} merge_t;

//local state
static uint8_t complete = 0;
static uint8_t invalid_rx_count = 0;
static uint8_t tx_timeout_enabled = 0;
static uint8_t tx_count_complete = 0;
static uint8_t nothing_new_count = 0;
static uint8_t got_valid_rx = 0;
static uint8_t pending = 0;

//initiator management
static uint8_t is_join_round = 0; // only used on initiator
static node_id_t joined_nodes[MAX_NODE_COUNT] = { 0 };
static uint8_t joined_nodes_not_committed[MAX_NODE_COUNT] = { 0 };
static int joined_nodes_not_committed_num = 0;

static void round_begin(const uint16_t round_count, const uint8_t id);
static int is_pending(const uint16_t round_count);
static void round_begin_sniffer(chaos_header_t* header);
static void round_end_sniffer(const chaos_header_t* header);

CHAOS_SERVICE(join, JOIN_SLOT_LEN, JOIN_ROUND_MAX_SLOTS, 0, is_pending, round_begin, round_begin_sniffer, round_end_sniffer);

void join_init(){

  //clear local state
  complete = 0;
  invalid_rx_count = 0;
  tx_timeout_enabled = 0;
  tx_count_complete = 0;
  nothing_new_count = 0;
  got_valid_rx = 0;
  pending = 0;

  is_join_round = 0;
  chaos_node_index = 0;
  chaos_has_node_index = 0;
  chaos_node_count = 0;

  //initiator management
  joined_nodes_not_committed_num = 0;
  memset(&joined_nodes, 0, sizeof(joined_nodes));
  memset(&joined_nodes_not_committed, 0, sizeof(joined_nodes_not_committed));
  if( IS_INITIATOR( ) ){
    chaos_has_node_index = 1;
    chaos_node_index = 0;
    joined_nodes[0] = node_id;
    chaos_node_count = 1;
  }
  printf("Chaos dynamic join: enabled\n");
}

static inline int merge_lists(join_t* join_tx, join_t* join_rx) {
  int index_rx = 0;
  int index_tx = 0;
  int index_merge = 0;
  int merge_tx = 0;
  merge_t merge;
  memset(&merge, 0, sizeof(merge));
  uint8_t num_missing_join = 0, num_new = 0;
  uint8_t missing_join[NODE_LIST_LEN] = {0};

  int merge_max_size = join_tx->slot_count + join_rx->slot_count;
  while ((index_tx < join_tx->slot_count || index_rx < join_rx->slot_count )
      && index_merge < merge_max_size) {
    if (index_tx >= join_tx->slot_count || (index_rx < join_rx->slot_count
        && join_rx->slot[index_rx] < join_tx->slot[index_tx])) {
      merge.slot[index_merge] = join_rx->slot[index_rx];
      merge.index[index_merge] = join_rx->index[index_rx];
      index_rx = MIN(index_rx + 1, join_rx->slot_count);
      merge_tx = 1; //arrays differs, so TX
    } else if (index_rx >= join_rx->slot_count || (index_tx < join_tx->slot_count
        && join_rx->slot[index_rx] > join_tx->slot[index_tx])) {
      merge.slot[index_merge] = join_tx->slot[index_tx];
      merge.index[index_merge] = join_tx->index[index_tx];
      index_tx = MIN(index_tx + 1, join_tx->slot_count);
      merge_tx = 1; //arrays differs, so TX
    } else { //(remote.slot[remote_index] == local.slot[local_index]){
      merge.slot[index_merge] = join_rx->slot[index_rx];
      merge.index[index_merge] = MAX(join_rx->index[index_rx], join_tx->index[index_tx]);
      merge_tx = join_rx->index[index_rx] != join_tx->index[index_tx]; //arrays differs, so TX
      index_rx = MIN(index_rx + 1, join_rx->slot_count);
      index_tx = MIN(index_tx + 1, join_tx->slot_count);
    }
    /* skip my id if I have an index */
    if( merge.slot[index_merge] == node_id && merge.index[index_merge] ){
      merge.slot[index_merge] = 0;
      merge.index[index_merge] = 0;
    } else {
      /* has index but no place */
      if( merge.index[index_merge] && index_merge >= NODE_LIST_LEN ){
        missing_join[num_missing_join++] = index_merge;
      }
      /* new node (no index) and got a place */
      if( !merge.index[index_merge] && index_merge < NODE_LIST_LEN ){
        num_new++;
      }
      /* move to next merge slot */
      index_merge++;
    }
  }

  /* nodes that have joined but not committed have priority over new nodes */
  if( num_missing_join && num_new ){
    /* how many new nodes we need to kick out? */
    int spaces = MIN(num_missing_join, num_new);
    //copy missing nodes to the end of the list first
    int i, j, k;
    i = NODE_LIST_LEN - 1;
    for( j = 0; j < spaces && i >= 0; i--, j++){
      join_tx->slot[i] = merge.slot[missing_join[j]];
      join_tx->index[i] = merge.index[missing_join[j]];
    }
    //copy the rest to the beginning
    j = 0; k = 0;
    while( k < i && j < index_merge) {
      join_tx->slot[k] = merge.slot[j];
      join_tx->index[k] = merge.index[j];
      j++; k++;
    }
  } else {
    memcpy(join_tx->slot, merge.slot, sizeof(join_tx->slot));
    memcpy(join_tx->index, merge.index, sizeof(join_tx->index));
  }
  join_tx->slot_count = MIN(index_merge, NODE_LIST_LEN);
  /* New overflow? */
  if (index_merge > NODE_LIST_LEN && (index_rx < join_rx->slot_count
                                       || index_tx < join_tx->slot_count)) {
    join_tx->overflow = 1;
    merge_tx = 1; //arrays differs, so TX
  }

  return merge_tx;
}

//only executed by initiator
static inline void add_node(join_t* join_tx, uint8_t i) {
  //check if this is node is already added
  node_index_t j;
  for (j = 0; j < chaos_node_count; j++) {
    if (join_tx->slot[i] == joined_nodes[j]) {
      //index of this node is j
      join_tx->index[i] = j;
      if( !joined_nodes_not_committed[ j ] ) {
        joined_nodes_not_committed[ j ] = 1;
        joined_nodes_not_committed_num++;
      }
      return;
    }
  }

  //add only if we have have space for it
  if(chaos_node_count < MAX_NODE_COUNT) {
    join_tx->index[i] = chaos_node_count;
    joined_nodes[chaos_node_count] = join_tx->slot[i];
    if( !joined_nodes_not_committed[ chaos_node_count ] ){
      joined_nodes_not_committed[ chaos_node_count ] = 1;
      joined_nodes_not_committed_num++;
    }
    chaos_node_count++;
  } else {
    join_tx->overflow = 1;
  }
  return;
}

//only executed by initiator
static inline void commit(join_t* join_tx) {
  COOJA_DEBUG_STR("commit!");
  LEDS_ON(LEDS_GREEN);
  int i;
  for (i = 0; i < join_tx->slot_count; i++) {
    if( !join_tx->index[i] && join_tx->slot[i] ){
      add_node(join_tx, i);
    }
  }
  //reset flags
  memset(join_tx->flags, 0, sizeof(join_tx->flags));
  unsigned int array_index = chaos_node_index / 8;
  unsigned int array_offset = chaos_node_index % 8;
  join_tx->flags[array_index] |= 1 << (array_offset);
  //update phase and node_count
  join_tx->node_count = chaos_node_count;
  join_tx->commit = 1;
}

static chaos_state_t process(uint16_t round_count, uint8_t slot_count,
    chaos_state_t current_state, int chaos_txrx_success, size_t payload_length,
    uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags){
  join_t* join_tx = (join_t*) tx_payload;
  join_t* join_rx = (join_t*) rx_payload;

  uint8_t delta = 0;
  uint16_t flag_sum = 0;

  if( current_state == CHAOS_RX && chaos_txrx_success ){
    //process overflow flag
    delta |= (join_tx->overflow != join_rx->overflow);
    join_tx->overflow |= join_rx->overflow;

    if (join_tx->commit == 0 && join_rx->commit == 0) {
      /*
       * have I been assigned an index without acknowledging it?
       */
      if( chaos_node_count < join_rx->node_count ){
        if( !chaos_has_node_index ){
          int i;
          for (i = 0; i < join_rx->slot_count; i++) {
            if (join_rx->index[i] != 0 && join_rx->slot[i] == node_id) {
              chaos_node_index = join_rx->index[i];
              chaos_has_node_index = 1;
              chaos_node_count = join_tx->node_count = join_rx->node_count;
              delta = 1;
              LEDS_ON(LEDS_GREEN);
              COOJA_DEBUG_STRX("late join: ", chaos_node_index, 2);
              break;
            }
          }
        }
      }

      //check if remote and local knowledge differ -> if so: merge
      if (chaos_node_count != join_rx->node_count ||
          join_tx->slot_count != join_rx->slot_count
          || memcmp(join_tx->slot, join_rx->slot, sizeof(join_rx->slot)) != 0) {
        delta |= merge_lists(join_tx, join_rx);
      }

      //XXX don't update chaos_node_count here
      //chaos_node_count = join_tx->node_count = MAX(join_rx->node_count, chaos_node_count);

      //set progress flag
      if( chaos_has_node_index ){
//        chaos_node_count = join_tx->node_count;
        unsigned int array_index = chaos_node_index / 8;
        unsigned int array_offset = chaos_node_index % 8;
        join_tx->flags[array_index] |= 1 << (array_offset);
      }

      //merge flags
      int i;
      for (i = 0; i < FLAGS_LEN(MAX_NODE_COUNT); i++) {
        delta |= (join_tx->flags[i] != join_rx->flags[i]);
        join_tx->flags[i] |= join_rx->flags[i];
        flag_sum += join_tx->flags[i];
      }

      //to commit or not to commit?
      uint8_t do_commit = IS_INITIATOR()
              && (
//                  (
//                      slot_count > JOIN_ROUND_MAX_SLOTS/2
//                      && nothing_new_count > COMMIT_THRESHOLD
//                      && join_tx->slot_count >= MIN( joined_nodes_not_committed_num, NODE_LIST_LEN ) /* at least someone new joined */
//                  ) ||
                  join_tx->overflow ||
                  join_tx->slot_count == NODE_LIST_LEN
                  || slot_count > JOIN_ROUND_MAX_SLOTS - N_TX_COMPLETE
                  )
              && flag_sum >= FLAG_SUM( chaos_node_count );

      //TODO: partial commits??

      if( do_commit ){
        //since everybody has signed, we clear the joined_nodes_not_committed list
        memset(joined_nodes_not_committed, 0, chaos_node_count * sizeof(uint8_t));
        joined_nodes_not_committed_num = 0;
        commit(join_tx); //here we fill joined_nodes_not_committed again
        delta = 1;
      }
    } else if( join_rx->commit == 1 /*&& !complete*/ ){ //commit phase
      if( join_tx->commit == 0 ){ // we are behind
        delta = 1;
        //drop local state
        memcpy(join_tx, join_rx, sizeof(join_t));
        chaos_node_count = join_rx->node_count;

        //get the index
        if( !chaos_has_node_index ){
          int i;
          for (i = 0; i < join_rx->slot_count; i++) {
            if (join_rx->slot[i] == node_id) {
              chaos_node_index = join_rx->index[i];
              chaos_has_node_index = 1;
              LEDS_ON(LEDS_GREEN);
              break;
            }
          }
        }

        //set the commit flag
        if( chaos_has_node_index ){
          unsigned int array_index = chaos_node_index / 8;
          unsigned int array_offset = chaos_node_index % 8;
          join_tx->flags[array_index] |= 1 << (array_offset);
        } else {
          join_tx->overflow = 1;
        }
      }

      //merge flags and check if complete
      int i;
      for (i = 0; i < FLAGS_LEN(join_rx->node_count); i++) {
        delta |= (join_tx->flags[i] != join_rx->flags[i]);
        join_tx->flags[i] |= join_rx->flags[i];
        flag_sum += join_tx->flags[i];
      }
      //all flags are set?
      if( flag_sum >= FLAG_SUM(join_rx->node_count) ){
        //Final flood: transmit result aggressively
        LEDS_ON(LEDS_BLUE);
        complete = 1;
      }
    } else if( join_tx->commit == 1 ){ //join_rx->commit == 0 --> neighbor is behind
      delta = 1;
    }
  }

  /* clear committed nodes */
  if( IS_INITIATOR() ){
    if( complete ){
      memset(joined_nodes_not_committed, 0, chaos_node_count * sizeof(uint8_t));
      joined_nodes_not_committed_num = 0;
    } else if( slot_count >= JOIN_ROUND_MAX_SLOTS -1 ){
      uint8_t i, j, count_committed = 0;
      for (i = 0; i < FLAGS_LEN(chaos_node_count); i++) {
        for( j = 0; j < 8; j++ ){
          if( i * 8 + j >= chaos_node_count ) {
            break;
          }
          if( join_tx->flags[i] & ( 1U<<j ) ){
            /* 0 means it is fine.
             * Since the node id 0 is reserved, it won't collide with an actual node id.
             */
//            COOJA_DEBUG_STRX("committed: ", joined_nodes[ i*8 + j ], 3);
            if( joined_nodes_not_committed[ i*8 + j ] ){
              joined_nodes_not_committed[ i*8 + j ] = 0;
              if(joined_nodes_not_committed_num) {
                joined_nodes_not_committed_num--;
              }
            }
            count_committed++;
          }
        }
      }

      if( join_tx->commit == 1 ){
        COOJA_DEBUG_STRX("ph1 committed #: ", count_committed, 2);
      } else {
        COOJA_DEBUG_STRX("ph0 committed #: ", count_committed, 2);
      }
      COOJA_DEBUG_STRX("2 joined_not_committed #", joined_nodes_not_committed_num, 2);
    }
  }

  /* shall we set pending here? */
  if(IS_INITIATOR()) {
    pending = join_tx->overflow || joined_nodes_not_committed_num > 0;
  }

  /* decide next chaos state */
  chaos_state_t next_state = CHAOS_RX;
  if ( IS_INITIATOR() && current_state == CHAOS_INIT ){
    next_state = CHAOS_TX; //for the first tx of the initiator: no increase of tx_count here
    tx_timeout_enabled = 1;
   } else if( current_state == CHAOS_RX ){
    if( chaos_txrx_success ){
      tx_timeout_enabled = 1;
      invalid_rx_count = 0;

      /* if LIMIT_TX_NO_DELTA is disabled then always tx on valid rx
       * if delta or first rx then tx else tx with 50% probability
       * or if complete and has already joined then tx else if complete and not joined then tx with 50% probability
       * else rx
       */
      uint8_t tx_on_complete = complete && ( !LIMIT_TX_FOR_DETACHED_NODES || (chaos_has_node_index || chaos_random_generator_fast() % 256 < 128) );
      uint8_t tx_on_delta = !tx_on_complete && (!LIMIT_TX_NO_DELTA || delta || !got_valid_rx || chaos_random_generator_fast() % 256 < 128);
      next_state = ( tx_on_delta || tx_on_complete ) ? CHAOS_TX : CHAOS_RX;

      if( delta || !got_valid_rx ){
        got_valid_rx = 1;
        nothing_new_count = 0;
        tx_count_complete = 0; //restart final flood if a neighbor is behind
      } else if ( complete ){
        //limit final flood for detached nodes
        if( next_state == CHAOS_TX ){
          tx_count_complete++;
        }
      } else {
        nothing_new_count++;
      }
    } else {
      invalid_rx_count++;
      if( got_valid_rx ){
        nothing_new_count++;
      }
      if( tx_timeout_enabled ){
        unsigned short threshold = chaos_random_generator_fast() % (CHAOS_RESTART_MAX - CHAOS_RESTART_MIN) + CHAOS_RESTART_MIN;
        if( invalid_rx_count > threshold ){
          next_state = CHAOS_TX;
          invalid_rx_count = 0;
          if( complete ){
            tx_count_complete++;
          }
        }
      }
    }
  } else if( current_state == CHAOS_TX ){
//    nothing_new_count++;
    if ( complete && tx_count_complete >= N_TX_COMPLETE ){
      LEDS_OFF(LEDS_BLUE);
      LEDS_OFF(LEDS_GREEN);
      next_state = CHAOS_OFF;
    }
  }

  *app_flags = ( current_state == CHAOS_TX || current_state == CHAOS_INIT ) ?
      join_tx->flags : join_rx->flags;
  return next_state;
}

static int get_flags_length(){
  return FLAGS_LEN(MAX_NODE_COUNT);
}

static int is_pending( const uint16_t round_count ){
//TODO: optimiziation, enable this after testing and bug fixing
  if( round_count < JOIN_ROUNDS_AFTER_BOOTUP ){
    pending = 1;
  }
  return pending;
}

int join_last_round_is_complete( void ){
  return complete;
}

int join_is_in_round( void ){
  return is_join_round;
}

int join_get_joined_nodes_not_committed_num( void ){
  return joined_nodes_not_committed_num;
}

uint8_t join_get_slot_count_from_payload( void* payload ){
  return ((join_t*)payload)->slot_count;
}

uint8_t join_is_committed_from_payload( void* payload ){
  return ((join_t*)payload)->commit;
}

static void round_begin( const uint16_t round_number, const uint8_t app_id ){
  complete = 0;
  tx_count_complete = 0;
  nothing_new_count = 0;
  got_valid_rx = 0;
  tx_timeout_enabled = 0;
  pending = 0;
  is_join_round = 1;
  join_t join_data;
  memset(&join_data, 0, sizeof(join_t));
  if( IS_INITIATOR() ){
    join_data.node_count = chaos_node_count;
    //copy joined but not committed node IDs/Indices to start with
    int i;
    for( i = 0;
        i < chaos_node_count
        && join_data.slot_count < NODE_LIST_LEN
        && join_data.slot_count < joined_nodes_not_committed_num;
        i++ ){
      if( joined_nodes_not_committed[i] ){
        join_data.slot[ join_data.slot_count ] = joined_nodes[i];
        join_data.index[ join_data.slot_count ] = i;
        join_data.slot_count++;
      }
    }
    COOJA_DEBUG_STRX("joined_not_committed #", join_data.slot_count, 2);
  } else if( !chaos_has_node_index ){
    join_data.slot[0] = node_id;
    join_data.slot_count = 1;
  }
  chaos_round(round_number, app_id, (const uint8_t const*)&join_data, sizeof(join_data), JOIN_SLOT_LEN_DCO, JOIN_ROUND_MAX_SLOTS, get_flags_length(), process);
}

static void round_begin_sniffer(chaos_header_t* header){
  header->join = !chaos_has_node_index /*|| is_join_round*/;
  if( IS_INITIATOR() ){
    header->join |= pending;
  }
}

static void round_end_sniffer(const chaos_header_t* header){
  pending |= IS_INITIATOR() && ((!is_join_round && header->join) || chaos_node_count < 2);
#if JOIN_STRESS_TESTING
  static uint8_t registered_chaos_node_count = 0, stable_rounds = 0;
  if( registered_chaos_node_count != chaos_node_count || is_join_round || pending ) {
    registered_chaos_node_count = chaos_node_count;
    stable_rounds = 0;
  } else {
    stable_rounds++;
    if( stable_rounds > JOIN_STRESS_TESTING_THRESHOLD ){
      join_init();
      pending = 1; //IS_INITIATOR();
      stable_rounds = 0;
      registered_chaos_node_count = chaos_node_count;
//      void watchdog_reboot(void);
//      watchdog_reboot();
    }
  }
#endif /* JOIN_STRESS_TESTING */
  is_join_round = 0;
}

