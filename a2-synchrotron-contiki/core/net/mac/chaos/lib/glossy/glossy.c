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
 *         Glossy-style library
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#include "contiki.h"
#include <string.h>

#include "node.h"
#include "chaos.h"
#include "glossy.h"

#undef ENABLE_COOJA_DEBUG
#define ENABLE_COOJA_DEBUG COOJA
#include "dev/cooja-debug.h"

#ifndef GLOSSY_N_TX
#define GLOSSY_N_TX 3
#endif

#ifndef GLOSSY_RESTART_INTERVAL
#define GLOSSY_RESTART_INTERVAL 5
#endif

#define CHAOS_GLOSSY_RETX 1

typedef uint32_t diss_t;

static diss_t diss_local;
static uint16_t off_slot, complete_slot;
static int got_valid_rx, tx_count, idle_count;

static chaos_state_t process(uint16_t round_count, uint16_t slot_count, chaos_state_t current_state, int chaos_txrx_success, size_t payload_length, uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags){

  if( current_state == CHAOS_RX && chaos_txrx_success && !got_valid_rx) {
    memcpy((void *)tx_payload, (void *)rx_payload, payload_length);
    complete_slot = slot_count + 1;
    got_valid_rx = 1;
    diss_local = *(diss_t *)tx_payload;
  }

  chaos_state_t next_state = CHAOS_RX;
  if( current_state == CHAOS_INIT ){
    next_state = IS_INITIATOR() ? CHAOS_TX : CHAOS_RX; //for the first tx of the initiator: no increase of tx_count here
  } else if( current_state == CHAOS_RX && chaos_txrx_success){
    next_state = CHAOS_TX;
    idle_count = 0;
  } else if( current_state == CHAOS_RX && !chaos_txrx_success){
    idle_count++;
    if( (IS_INITIATOR() || (CHAOS_GLOSSY_RETX && got_valid_rx)) && (idle_count % GLOSSY_RESTART_INTERVAL == 0) ){
      //restart
      next_state = CHAOS_TX;
    }
  } else if( current_state == CHAOS_TX ){
    tx_count++;
    next_state = CHAOS_RX;
    if(tx_count > GLOSSY_N_TX){
      next_state = CHAOS_OFF;
    }
  }

  if(next_state == CHAOS_OFF){
    off_slot = slot_count;
  }

  return next_state;
}

uint16_t glossy_get_off_slot(){
  return off_slot;
}

int glossy_is_pending(const uint16_t round_count){
  return 1;
}

static int glossy_get_flags_length() {
  return 0;
}

uint16_t glossy_round_begin(const uint16_t round_number, const uint8_t app_id, uint32_t* diss_value){
  got_valid_rx = 0;
  tx_count = 0;
  idle_count = 0;
  diss_local = 0;
  off_slot = CHAOS_GLOSSY_ROUND_MAX_SLOTS;
  complete_slot = 0;

  if( IS_INITIATOR() ){
    diss_local = *diss_value;
  }
  chaos_round(round_number, app_id, (const uint8_t const*)&diss_local, sizeof(diss_local), CHAOS_GLOSSY_SLOT_LEN_DCO, CHAOS_GLOSSY_ROUND_MAX_SLOTS, glossy_get_flags_length(), process);
  *diss_value = diss_local;
  return complete_slot;
}
