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
 *         Run apps on-demand test library.
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#include "contiki.h"
#include <string.h>
#include "leds.h"

#include "chaos.h"
#include "chaos-control.h"
#include "on-demand-test.h"

#undef ENABLE_COOJA_DEBUG
#define ENABLE_COOJA_DEBUG COOJA
#include "dev/cooja-debug.h"

#define ON_DEMAND_TEST_SLOT_LEN          ((6*(RTIMER_SECOND/1000)))    //TODO needs calibration
#define ON_DEMAND_TEST_SLOT_LEN_DCO      (ON_DEMAND_TEST_SLOT_LEN*CLOCK_PHI)    //TODO needs calibration
#define ON_DEMAND_TEST_ROUND_MAX_SLOTS   40            //force radio off after CHAOS_ROUND_MAX_SLOTS slots

static int got_valid_rx;
static int pending = 0;

static void round_begin(const uint16_t round_count, const uint8_t id);
static int is_pending(const uint16_t round_count);
static void round_begin_sniffer(chaos_header_t* header);
static void round_end_sniffer(const chaos_header_t* header);

CHAOS_SERVICE(on_demand_test, ON_DEMAND_TEST_SLOT_LEN, ON_DEMAND_TEST_ROUND_MAX_SLOTS, 0, is_pending, round_begin, round_begin_sniffer, round_end_sniffer);

static chaos_state_t process(uint16_t round_count, uint16_t slot_count, chaos_state_t current_state, int rx_valid, size_t payload_length, uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags){
  if( current_state == CHAOS_RX && rx_valid ){
    memcpy((void *)tx_payload, (void *)rx_payload, payload_length);
    got_valid_rx = 1;
  }
  chaos_state_t next_state;
  if( IS_INITIATOR( ) ){
    next_state = CHAOS_TX;
  } else {
    next_state = CHAOS_RX;
  }
  *app_flags = NULL;
  return next_state;
}

static int get_flags_length() {
  return 0;
}

static void round_begin(const uint16_t round_count, const uint8_t id){
  pending = 0;
  got_valid_rx = 0;
  if( IS_INITIATOR() ){
    const char* const msg = "hi there!1234567890+qwertyuiopasdfghjklzxcvbnm,QWERTYUIOPASDFGHJKLZXCVBNM0987665554444443211234567890+";
    chaos_round(round_count, id, (const uint8_t const*)msg, strlen(msg), ON_DEMAND_TEST_SLOT_LEN_DCO, ON_DEMAND_TEST_ROUND_MAX_SLOTS, get_flags_length(), process);
  } else {
    chaos_round(round_count, id, NULL, 0, ON_DEMAND_TEST_SLOT_LEN_DCO, ON_DEMAND_TEST_ROUND_MAX_SLOTS, get_flags_length(), process);
  }
  leds_off(LEDS_GREEN);
}

static int is_pending(const uint16_t round_count){
  return pending;
}

static void round_begin_sniffer(chaos_header_t* header){
  if( !IS_INITIATOR() && header->round_number % 5 == 0 ){
    leds_on(LEDS_GREEN);
    header->join = 1;
  }
}

static void round_end_sniffer(const chaos_header_t* header){
  if( IS_INITIATOR() && header->join == 1 ){
    leds_on(LEDS_GREEN);
    pending = 1;
  }
}
