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
 *         Join library.
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
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

#define ENABLE_COOJA_DEBUG COOJA
#include "dev/cooja-debug.h"

#if 1 //FAULTY_NODE_ID
volatile uint8_t rx_pkt_crc_err[129] = {0};
volatile uint8_t rx_pkt_copy[129] = {0};
volatile join_debug_t join_debug_var = {0,0,0,0,0};
#endif

#ifndef JOIN_STRESS_TEST
#define JOIN_STRESS_TEST 0
#endif

#ifndef JOIN_ROUNDS_AFTER_BOOTUP
#define JOIN_ROUNDS_AFTER_BOOTUP (10)
#endif

#define JOIN_SLOT_LEN          (7*(RTIMER_SECOND/1000)+0*(RTIMER_SECOND/1000)/2)    //TODO needs calibration
#define JOIN_SLOT_LEN_DCO      (JOIN_SLOT_LEN*CLOCK_PHI)    //TODO needs calibration

#define JOIN_MAX_COMMIT_SLOT (JOIN_ROUND_MAX_SLOTS / 2)

#ifndef COMMIT_THRESHOLD
#define COMMIT_THRESHOLD (JOIN_MAX_COMMIT_SLOT)
#endif

#ifndef N_TX_COMPLETE
#define N_TX_COMPLETE 9
#endif

#ifndef CHAOS_RESTART_MIN
#define CHAOS_RESTART_MIN 3
#endif

#ifndef CHAOS_RESTART_MAX
#define CHAOS_RESTART_MAX 10
#endif

#ifndef NODE_LIST_LEN
#define NODE_LIST_LEN 10  //describes how many nodes can join in a single round
#endif

#ifndef JOIN_TEST_LEAVE_THRESHOLD
#define JOIN_TEST_LEAVE_THRESHOLD (JOIN_ROUNDS_AFTER_BOOTUP+10)
#endif

#define FLAGS_LEN(node_count)   ((node_count / 8) + ((node_count % 8) ? 1 : 0))
#define LAST_FLAGS(node_count)  ((1 << ((((node_count) - 1) % 8) + 1)) - 1)
#define FLAG_SUM(node_count)  ((((node_count) - 1) / 8 * 0xFF) + LAST_FLAGS(node_count))

typedef struct __attribute__((packed)) {
  uint8_t node_count;
  union {
    uint8_t commit_field;
    struct{
      uint8_t                   //control flags
        slot_count :5,       //number of slots into which nodes wrote their ids
        overflow :1,              /* available join slots too short */
        commit :2;                /* commit join */
    };
  };
  node_id_t slot[NODE_LIST_LEN];              //slots to write node ids into
  node_index_t index[NODE_LIST_LEN]; /* assigned indices */
  uint8_t flags[FLAGS_LEN(MAX_NODE_COUNT)];  //flags used to confirm the commit
} join_t;

#if JOIN_LOG_FLAGS
uint8_t chaos_join_flags_log[JOIN_ROUND_MAX_SLOTS]={0};
uint8_t chaos_join_commit_log[JOIN_ROUND_MAX_SLOTS]={0};
#endif

//local state
static uint8_t complete = 0;
static uint8_t invalid_rx_count = 0;
static uint8_t tx_timeout_enabled = 0;
static uint8_t tx_count_complete = 0;
static uint8_t delta_at_slot = 0;
//static uint8_t got_valid_rx = 0;
static uint8_t pending = 0;
static uint16_t commit_slot = 0;
static uint16_t off_slot = 0;
static uint16_t complete_slot = 0;

//initiator management
static uint8_t is_join_round = 0; // only used on initiator
static node_id_t joined_nodes[MAX_NODE_COUNT] = { 0 };
enum {
  NODE_ID = 0, NODE_IDX = 1
};
static node_id_t joined_nodes_map[MAX_NODE_COUNT][2] = { {0, 0} };
static node_id_t joined_nodes_map_tmp[MAX_NODE_COUNT][2] = { {0, 0} };

static void round_begin(const uint16_t round_count, const uint8_t id);
static int is_pending(const uint16_t round_count);
static void round_begin_sniffer(chaos_header_t* header);
static void round_end_sniffer(const chaos_header_t* header);
static int binary_search( uint16_t array[][2], int size, uint16_t search_id );
static void merge_sort( uint16_t a[][2], uint16_t aux[][2], int hi, int lo );

CHAOS_SERVICE(join, JOIN_SLOT_LEN, JOIN_ROUND_MAX_SLOTS, 0, is_pending, round_begin, round_begin_sniffer, round_end_sniffer);

static uint8_t bit_count(uint8_t u)
{
  return  (u -(u>>1)-(u>>2)-(u>>3)-(u>>4)-(u>>5)-(u>>6)-(u>>7));
}

static void do_sort_joined_nodes_map(){
  LEDS_ON(LEDS_RED);
  //need to do a precopy!!
  memcpy(joined_nodes_map_tmp, joined_nodes_map, sizeof(joined_nodes_map));
  merge_sort(joined_nodes_map, joined_nodes_map_tmp, 0, chaos_node_count-1);
  LEDS_OFF(LEDS_RED);
}

void join_print_nodes(void){
  int i;
  for( i=0; i<chaos_node_count; i++ ){
    printf("%u:%u%s", joined_nodes_map[i][NODE_ID], joined_nodes_map[i][NODE_IDX], (((i+1) & 7) == 0) ? "\n" : ", " );
  }
}

void join_init(){
  //clear node information
  chaos_node_index = 0;
  chaos_node_count = 0;
  chaos_has_node_index = 0;

  //clear local state
  commit_slot = 0;
  off_slot = JOIN_ROUND_MAX_SLOTS;
  complete_slot = 0;
  complete = 0;
  invalid_rx_count = 0;
  tx_timeout_enabled = 0;
  tx_count_complete = 0;
  delta_at_slot = 0;
  pending = 0;
  is_join_round = 0;

  //initiator management
  memset(&joined_nodes_map, 0, sizeof(joined_nodes_map));
  memset(&joined_nodes_map_tmp, 0, sizeof(joined_nodes_map_tmp));
  if( IS_INITIATOR( ) ){
    chaos_has_node_index = 1;
    chaos_node_index = 0;
    joined_nodes[0] = node_id;
    joined_nodes_map[0][NODE_IDX]=0;
    joined_nodes_map[0][NODE_ID]=node_id;
    chaos_node_count = 1;
  }
}

static inline int merge_lists(join_t* join_tx, join_t* join_rx) {
  uint8_t index_rx = 0;
  uint8_t index_tx = 0;
  uint8_t index_merge = 0;
  uint8_t delta = 0;

  node_id_t merge[NODE_LIST_LEN] = {0};
  //memset(&merge[0], 0, sizeof(merge));

  while ((index_tx < join_tx->slot_count || index_rx < join_rx->slot_count ) && index_merge < NODE_LIST_LEN) {
    if (index_tx >= join_tx->slot_count || (index_rx < join_rx->slot_count && join_rx->slot[index_rx] < join_tx->slot[index_tx])) {
      merge[index_merge] = join_rx->slot[index_rx];
      index_merge++;
      index_rx++;
      delta = 1; //arrays differs, so TX
    } else if (index_rx >= join_rx->slot_count || (index_tx < join_tx->slot_count && join_rx->slot[index_rx] > join_tx->slot[index_tx])) {
      merge[index_merge] = join_tx->slot[index_tx];
      index_merge++;
      index_tx++;
      delta = 1; //arrays differs, so TX
    } else { //(remote.slot[remote_index] == local.slot[local_index]){
      merge[index_merge] = join_rx->slot[index_rx];
      index_merge++;
      index_rx++;
      index_tx++;
    }
#if 0*FAULTY_NODE_ID
    if( merge[index_merge-1] > FAULTY_NODE_ID && !rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1] ){
      rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1]=3;
      memcpy(rx_pkt_crc_err, (uint8_t*)join_rx, sizeof(join_t));
    }
#endif
  }

  /* New overflow? */
  if (index_merge >= NODE_LIST_LEN && (index_rx < join_rx->slot_count || index_tx < join_tx->slot_count)) {
    join_tx->overflow = 1;
    delta = 1; //arrays differs, so TX
  }
  //index_merge = MIN(index_merge, NODE_LIST_LEN);
  join_tx->slot_count = index_merge;

  memcpy(join_tx->slot, merge, sizeof(merge));
  //memcpy(join_tx->slot, merge, index_merge*sizeof(node_id_t));
  return delta;
}

//only executed by initiator
static inline void add_node(join_t* join_tx, uint8_t i, uint8_t chaos_node_count_before_commit) {
  //search and check if this is node is already added
  LEDS_ON(LEDS_RED);
//  node_index_t j;
//  for (j = 0; j < old_chaos_node_count; j++) {
//    if (join_tx->slot[i] == joined_nodes[j]) {
//      //index of this node is j
//      join_tx->index[i] = j;
//      return;
//    }
//  }
  int j = binary_search(joined_nodes_map, chaos_node_count_before_commit, join_tx->slot[i]);
  LEDS_OFF(LEDS_RED);
  if( j > -1 ){
    //index of this node is j
    join_tx->index[i] = j;
    return;
  }
  //add only if we have have space for it
  if(chaos_node_count < MAX_NODE_COUNT) {
    join_tx->index[i] = chaos_node_count;
    joined_nodes[chaos_node_count] = join_tx->slot[i];
    //joined_nodes_map will be sorted later
    joined_nodes_map[chaos_node_count][NODE_ID] = join_tx->slot[i];
    joined_nodes_map[chaos_node_count][NODE_IDX] = join_tx->index[i];
    chaos_node_count++;
  } else {
    join_tx->overflow = 1;
  }
  //XXX insert the 2nd bug again
  //chaos_node_count_before_commit = chaos_node_count;
  return;
}

//only executed by initiator
static inline void commit(join_t* join_tx) {
  COOJA_DEBUG_STR("commit!");
  uint8_t chaos_node_count_before_commit = chaos_node_count;
  int i;
  for (i = 0; i < join_tx->slot_count; i++) {
    if( !join_tx->index[i] && join_tx->slot[i] ){
      add_node(join_tx, i, chaos_node_count_before_commit);
    }
  }
  //reset flags
  memset(join_tx->flags, 0, sizeof(join_tx->flags));
  //commit on my flag
  unsigned int array_index = chaos_node_index / 8;
  unsigned int array_offset = chaos_node_index % 8;
  join_tx->flags[array_index] |= 1 << (array_offset);
  //update phase and node_count
  join_tx->node_count = chaos_node_count;
  join_tx->commit = 1;
}

static chaos_state_t process(uint16_t round_count, uint16_t slot,
    chaos_state_t current_state, int chaos_txrx_success, size_t payload_length,
    uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags){
  join_t* join_tx = (join_t*) tx_payload;
  join_t* join_rx = (join_t*) rx_payload;

  uint8_t delta = 0;
  uint16_t flag_sum = 0;

#if 0*FAULTY_NODE_ID /*|| FAULTY_NODE_COUNT*/
  int i = 0;
  //check join_rx
  if( current_state == CHAOS_RX && chaos_txrx_success ){
    for(i=0; i<join_rx->slot_count && !join_debug_var.slot && rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1] != 1; i++){
      if(join_rx->slot[i] > FAULTY_NODE_ID || join_rx->node_count > FAULTY_NODE_COUNT){
        rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1]=4;
        memcpy(rx_pkt_crc_err, rx_payload, payload_length);
        join_debug_var.debug_pos=__LINE__;
        join_debug_var.info |= RX_ERR;
        break;
      }
    }
  }
  //check join_tx
  for(i=0; i<join_tx->slot_count && !join_debug_var.slot && rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1] != 1; i++){
    if(join_tx->slot[i] > FAULTY_NODE_ID ||  join_tx->node_count > FAULTY_NODE_COUNT){
      rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1]=5;
      memcpy(rx_pkt_crc_err, tx_payload, payload_length);
      join_debug_var.debug_pos=__LINE__;
      join_debug_var.info |= TX_ERR_BEFORE;
      break;
    }
  }
  //check dummy for zeros
  uint32_t *dummy_packet_32t = chaos_get_dummy_packet_32t();
  for(i=0; i<(RADIO_MAX_PACKET_LEN + 3)/ 4
          && !join_debug_var.slot
          && rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1] != 1;
      i++){
    if(dummy_packet_32t[i] != 0){
      rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1] |= 0x40;
      join_debug_var.debug_pos=__LINE__;
      join_debug_var.info |= DUMMY_ERR_BEFORE;
      break;
    }
  }
#endif

  if( current_state == CHAOS_RX && chaos_txrx_success ){

    //process overflow flag
    delta |= (join_tx->overflow != join_rx->overflow);
    join_tx->overflow |= join_rx->overflow;
    join_tx->node_count = MAX(join_rx->node_count, join_tx->node_count);

    //not yet committed
    if (join_tx->commit == 0 && join_rx->commit == 0) {
      if( IS_INITIATOR() || slot < JOIN_MAX_COMMIT_SLOT) {
        // not late and definitely still in collect phase
        //merge flags
        int i;
        for (i = 0; i < FLAGS_LEN(join_rx->node_count); i++) {
          delta |= (join_tx->flags[i] != join_rx->flags[i]);
          join_tx->flags[i] |= join_rx->flags[i];
          flag_sum += join_tx->flags[i];
        }

        //check if remote and local knowledge differ -> if so: merge
        uint8_t delta_slots = join_tx->slot_count != join_rx->slot_count;
        if(!delta_slots){
          delta_slots = memcmp(&join_tx->slot[0], &join_rx->slot[0], sizeof(join_rx->slot)) != 0;
        }
        if ( delta_slots ) {
          delta |= merge_lists(join_tx, join_rx);
        }
      } else {
        //since half of the slots passed, we need to wait for the initiator commit.
        delta = 0;
      }
      //all flags are set?
      if( flag_sum >= FLAG_SUM(join_rx->node_count) && IS_INITIATOR()){
        if(!complete){
          complete_slot = slot;
        }
        //Final flood: transmit result aggressively
        complete = join_tx->commit;
      }
    } else if( join_rx->commit == 1 ){ //commit phase
      if( join_tx->commit == 0 ){ // we are behind
        commit_slot = slot;
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
              LEDS_ON(LEDS_RED);
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
      } else {
        //merge flags
        int i;
        for (i = 0; i < FLAGS_LEN(join_rx->node_count); i++) {
          delta |= (join_tx->flags[i] != join_rx->flags[i]);
          join_tx->flags[i] |= join_rx->flags[i];
          flag_sum += join_tx->flags[i];
        }
      }
      //all flags are set?
      if( flag_sum >= FLAG_SUM(join_rx->node_count) ){
        //Final flood: transmit result aggressively
        LEDS_OFF(LEDS_RED);
        if(!complete){
          complete_slot = slot;
        }
        complete = 1;
      }
    } else if( join_tx->commit == 1 ){ //join_rx->commit == 0 --> neighbor is behind
      delta = 1;
    }
  }

  if( delta ){
    delta_at_slot = slot;
  }


  /* decide next chaos state */
  chaos_state_t next_state = CHAOS_RX;
  if ( IS_INITIATOR() && current_state == CHAOS_INIT ){
    next_state = CHAOS_TX; //for the first tx of the initiator: no increase of tx_count here
    tx_timeout_enabled = 1;

  } else if( IS_INITIATOR() && join_tx->commit == 0 &&
      ( (!delta && slot == delta_at_slot + COMMIT_THRESHOLD) /* no delta for some time */
          || join_tx->slot_count == NODE_LIST_LEN /* join list is full */
          || (slot >= JOIN_MAX_COMMIT_SLOT /* time to switch to commit phase */
              &&  join_tx->slot_count > 0  /* someone is joining */)
          //|| complete
      )){ //commit?
    if(join_tx->slot_count > 0){
      complete = 0;
    }
    commit(join_tx);
    commit_slot = slot;
    next_state = CHAOS_TX;
    invalid_rx_count = 0;
  } else if( current_state == CHAOS_RX ){
    if( chaos_txrx_success ){

      invalid_rx_count = 0;
      next_state = ( delta || complete || !tx_timeout_enabled /*first rx -> forward*/) ? CHAOS_TX : CHAOS_RX;
      tx_timeout_enabled = 1;
      if( delta ){
        tx_count_complete = 0; //restart final flood if a neighbor is behind
      }
      if ( complete ){
        if( next_state == CHAOS_TX ){
          tx_count_complete++;
        }
      }
    } else {
      invalid_rx_count++;
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
    if ( complete && tx_count_complete >= N_TX_COMPLETE ){
      next_state = CHAOS_OFF;
    }
  }

  *app_flags = ( current_state == CHAOS_TX || current_state == CHAOS_INIT ) ?
      join_tx->flags : join_rx->flags;

#if 0*FAULTY_NODE_ID
  //check join_tx
  for(i=0; i<join_tx->slot_count && !join_debug_var.slot && rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1] != 1; i++){
    if(join_tx->slot[i] > FAULTY_NODE_ID ||  join_tx->node_count > FAULTY_NODE_COUNT){
      rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1]=6;
      memcpy(rx_pkt_crc_err, tx_payload, payload_length);
      join_debug_var.debug_pos=__LINE__;
      join_debug_var.info |= TX_ERR_AFTER;
      break;
    }
  }
  //check dummy for zeros
  for(i=0; i<(RADIO_MAX_PACKET_LEN + 3)/ 4
          && !join_debug_var.slot && rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1] != 1;
      i++){
    if(dummy_packet_32t[i] != 0){
      rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1] |= 0x80;
      join_debug_var.debug_pos=__LINE__;
      join_debug_var.info |= DUMMY_ERR_AFTER;
      break;
    }
  }

  if(!join_debug_var.slot && join_debug_var.info){
    join_debug_var.slot=slot;
    join_debug_var.slot_status=chaos_txrx_success;
    join_debug_var.slot_type=current_state;
  }
#endif
  int end = (slot >= JOIN_ROUND_MAX_SLOTS - 2) || (next_state == CHAOS_OFF);
  if(IS_INITIATOR() && end){
    //sort joined_nodes_map to speed up search (to enable the use of binary search) when adding new nodes
    do_sort_joined_nodes_map();
  }
  if(end){
    off_slot = slot;
  }
#if JOIN_LOG_FLAGS
  chaos_join_commit_log[slot]=join_tx->commit_field;
  //if(current_state == CHAOS_RX && chaos_txrx_success)
  {
    uint8_t i;
    //chaos_join_flags_log[slot]=0;
    for(i=0; i<FLAGS_LEN(join_tx->node_count); i++){
      chaos_join_flags_log[slot] += bit_count(join_tx->flags[i]);
    }
  }
#endif /* CHAOS_LOG_FLAGS */
  return next_state;
}

static int get_flags_length(){
  return FLAGS_LEN(MAX_NODE_COUNT);
}

static int is_pending( const uint16_t round_count ){
  //TODO: optimiziation, enable this after testing and bug fixing
  if( round_count < JOIN_ROUNDS_AFTER_BOOTUP )
  {
    pending = 1;
  }
  return pending;
  //return 1;
}

int join_last_round_is_complete( void ){
  return complete_slot;
}

uint16_t join_get_off_slot(){
  return off_slot;
}

int join_is_in_round( void ){
  return is_join_round;
}

uint16_t join_get_commit_slot(){
  return commit_slot;
}

uint8_t join_get_slot_count_from_payload( void* payload ){
  return ((join_t*)payload)->slot_count;
}

uint8_t join_is_committed_from_payload( void* payload ){
  return ((join_t*)payload)->commit;
}

static void round_begin( const uint16_t round_number, const uint8_t app_id ){
#if FAULTY_NODE_ID
  memset(&join_debug_var, 0, sizeof(join_debug_var));
  memset(rx_pkt_crc_err, 0, sizeof(rx_pkt_crc_err));
//  memset(rx_pkt_copy, 0, sizeof(rx_pkt_copy));
#endif

  commit_slot = 0;
  off_slot = JOIN_ROUND_MAX_SLOTS;
  complete_slot = 0;
  complete = 0;
  tx_count_complete = 0;
  delta_at_slot = 0;
  tx_timeout_enabled = 0;
  pending = 0;
  is_join_round = 1;
  join_t join_data;
  memset(&join_data, 0, sizeof(join_t));
#if JOIN_LOG_FLAGS
  memset(&chaos_join_flags_log, 0, sizeof(chaos_join_flags_log));
  memset(&chaos_join_commit_log, 0, sizeof(chaos_join_commit_log));
#endif

  if( IS_INITIATOR() ){
    join_data.node_count = chaos_node_count;
    unsigned int array_index = chaos_node_index / 8;
    unsigned int array_offset = chaos_node_index % 8;
    join_data.flags[array_index] |= 1 << (array_offset);
  } else if( !chaos_has_node_index ){
    join_data.slot[0] = node_id;
    join_data.slot_count = 1;
  } else {
    unsigned int array_index = chaos_node_index / 8;
    unsigned int array_offset = chaos_node_index % 8;
    join_data.flags[array_index] |= 1 << (array_offset);
  }
  chaos_round(round_number, app_id, (const uint8_t const*)&join_data, sizeof(join_data), JOIN_SLOT_LEN_DCO, JOIN_ROUND_MAX_SLOTS, get_flags_length(), process);
}

static void round_begin_sniffer(chaos_header_t* header){
  header->join = !chaos_has_node_index /*&& !is_join_round*/;
  if( IS_INITIATOR() ){
    header->join |= pending /*&& !is_join_round*/;
  }
}

static void round_end_sniffer(const chaos_header_t* header){
  pending |= IS_INITIATOR() && ( header->join || chaos_node_count < 2);
  is_join_round = 0;
  //TODO remove me later
#if JOIN_STRESS_TEST
#warning "JOIN_STRESS_TEST"
  if( header->round_number % JOIN_TEST_LEAVE_THRESHOLD == 0 ){
    //drop slots -> leave ->rejoin
    join_init();
    pending=IS_INITIATOR();
  }
#endif
}

////sort functions
/* Merge sort code adopted from: http://algs4.cs.princeton.edu/lectures/22Mergesort.pdf */
static void merge(uint16_t a[][2], uint16_t aux[][2], int lo, int mid, int hi)
{
  int i = lo, j = mid+1, k;
  for (k = lo; k <= hi; k++){
    if (i > mid){
      aux[k][0] = a[j][0];
      aux[k][1] = a[j][1];
      j++;
    } else if(j > hi){
      aux[k][0] = a[i][0];
      aux[k][1] = a[i][1];
      i++;
    } else if (a[j][NODE_ID] <= a[i][NODE_ID]){
      aux[k][0] = a[j][0];
      aux[k][1] = a[j][1];
      j++;
    } else{
      aux[k][0] = a[i][0];
      aux[k][1] = a[i][1];
      i++;
    }
  }
}

//recursive version: needs pre-copy in aux, but faster
static void merge_sort(uint16_t a[][2], uint16_t aux[][2], int lo, int hi)
{
  if (hi <= lo) return;
  int mid = lo + (hi - lo) / 2;
  merge_sort(aux, a, lo, mid);
  merge_sort(aux, a, mid+1, hi);
  merge(aux, a, lo, mid, hi);
}

//binary search
//http://www.programmingsimplified.com/c/source-code/c-program-binary-search
static int binary_search( uint16_t array[][2], int size, uint16_t search_id ){
  int first = 0;
  int last = size - 1;
  int middle = ( first + last ) / 2;

  while( first <= last ){
    if( array[middle][NODE_ID] < search_id ){
      first = middle + 1;
    } else if( array[middle][NODE_ID] == search_id ){
      return array[middle][NODE_IDX];
    } else {
      last = middle - 1;
    }
    middle = (first + last)/2;
  }
  return -1;
}
