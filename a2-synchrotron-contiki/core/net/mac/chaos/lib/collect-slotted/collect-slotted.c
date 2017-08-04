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
 *         Collect-slotted library
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#include "collect-slotted.h"

#include "contiki.h"
#include <string.h>

#include "chaos.h"
#include "chaos-random-generator.h"
#include "node.h"
//#include "chaos-config.h"

#undef ENABLE_COOJA_DEBUG
#define ENABLE_COOJA_DEBUG COOJA
#include "dev/cooja-debug.h"

//#ifndef COMMIT_THRESHOLD
//#define COMMIT_THRESHOLD 6
//#endif

//#ifndef N_TX_COMPLETE
//#define N_TX_COMPLETE 5
//#endif


#ifndef N_TX_GLOSSY
#define N_TX_GLOSSY 3
#endif

#ifndef TX_WINOW_SIZE
#define TX_WINOW_SIZE 5
#endif

#ifndef N_GLOSSY_SLOTS
#define N_GLOSSY_SLOTS 10
#endif

#ifndef N_COLLECT_START
#define N_COLLECT_START 10
#endif

#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
#define NODES_ESTIMATE (MAX_NODE_COUNT)
#warning "APP: due to packet size limitation: maximum network size = MAX_NODE_COUNT"
#else
#define NODES_ESTIMATE (CHAOS_NODES)
#endif

//glossy mode variables
static int tx_count = 0;
static int8_t* first_rx_hop_count_local;

//collect mode variables
static uint16_t tx_slot;

typedef struct __attribute__((packed)) {
  uint8_t values[NODES_ESTIMATE];
} chaos_collect_slotted_t;

static chaos_collect_slotted_t collect_local;

static chaos_state_t
collect_process(uint16_t round_count, uint16_t slot_count, chaos_state_t current_state, int chaos_txrx_success, size_t payload_length, uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags)
{
  chaos_collect_slotted_t* tx_collect_slotted = (chaos_collect_slotted_t*)tx_payload;
  chaos_collect_slotted_t* rx_collect_slotted = (chaos_collect_slotted_t*)rx_payload;
  chaos_state_t next_state = CHAOS_RX;

  if( current_state == CHAOS_RX && chaos_txrx_success){
    //merge
    int i;
    for( i=0; i < NODES_ESTIMATE; i++){
      collect_local.values[i] |= rx_collect_slotted->values[i];
    }
  }


  if( IS_INITIATOR() && current_state == CHAOS_INIT ){
    next_state = CHAOS_TX;
    tx_count++;
    *first_rx_hop_count_local = 0;
  } else if(current_state == CHAOS_RX && chaos_txrx_success && tx_count < N_TX_GLOSSY){
    //glossy mode in the beginning of the round
    if( tx_count == 0 ){
      *first_rx_hop_count_local = slot_count + 1;
    }
    next_state = CHAOS_TX;
    tx_count++;
  } else if( round_count < N_COLLECT_START && tx_count >= N_TX_GLOSSY){
    next_state = CHAOS_OFF;
  } else if (round_count >= N_COLLECT_START && slot_count == tx_slot && !IS_INITIATOR()){
    //pick random slot and transmit
    next_state = CHAOS_TX;
    memcpy(tx_collect_slotted, &collect_local, sizeof(collect_local));
  } else if(round_count >= N_COLLECT_START && slot_count == tx_slot + 1 /*&& current_state == CHAOS_TX && chaos_txrx_success*/){
    //off after transmit
    next_state = CHAOS_OFF;
  }
  return next_state;
}


int collect_slotted_get_flags_length() {
  return 0;
}

int collect_slotted_is_pending(const uint16_t round_count){
  return 1;
}

int collect_slotted_round_begin(const uint16_t round_number, const uint8_t app_id, int8_t* first_rx_hop_count,uint8_t avg_rank, uint8_t max_rank)
{
  tx_count = 0;
  first_rx_hop_count_local = first_rx_hop_count;
  *first_rx_hop_count_local = -1;

  memset(&collect_local, 0, sizeof(collect_local));
  collect_local.values[chaos_node_index] = node_id;
  chaos_round(round_number, app_id, (const uint8_t const*)&collect_local, sizeof(chaos_collect_slotted_t), COLLECT_SLOTTED_SLOT_LEN_DCO, COLLECT_SLOTTED_ROUND_MAX_SLOTS, 0, collect_process);
  uint16_t min_tx_slot = (max_rank - avg_rank) * TX_WINOW_SIZE;
  tx_slot = chaos_random_generator_fast() % TX_WINOW_SIZE + min_tx_slot + N_GLOSSY_SLOTS;
//  printf("%u %u\n", min_tx_slot, tx_slot);
  return 0;
}

