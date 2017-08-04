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
 *         A2-Synchrotron mutlichannel driver.
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */

#include "contiki.h"
#include "net/mac/chaos/chaos.h"
#include "net/mac/chaos/chaos-header.h"
#include "net/mac/chaos/chaos-multichannel.h"
#include "net/mac/chaos/chaos-random-generator.h"

#if CHAOS_MULTI_CHANNEL
#if CHAOS_MULTI_CHANNEL_ADAPTIVE
static unsigned int channel_black_list_local = 0;
static unsigned int channel_black_list_collected = 0;
static unsigned int channel_black_list_committed = 0;
static uint16_t channel_prr[CHAOS_NUMBER_OF_CHANNELS];
#endif /* CHAOS_MULTI_CHANNEL_ADAPTIVE */
uint8_t chaos_channel_hopping_sequence[] = CHAOS_HOPPING_SEQUENCE;
volatile uint16_t chaos_current_channel = 0;
#endif /* CHAOS_MULTI_CHANNEL */

void chaos_multichannel_init(void) {
#if CHAOS_MULTI_CHANNEL
  chaos_current_channel = chaos_multichannel_lookup_channel(0, 0);
#if CHAOS_MULTI_CHANNEL_ADAPTIVE
  /* moving average function */
  #define LIMIT_PRR(X) ( (X) > PRR_SCALE ? PRR_SCALE : X )
  #define UPDATE_PRR(X_NEW, S) ( LIMIT_PRR(PRR_ALPHA * (X_NEW) + ((((((PRR_SCALE - PRR_ALPHA) * (S))<<1U) + 1)/PRR_SCALE)>>1)) )
  channel_black_list_local = 0;
  channel_black_list_collected = 0;
  channel_black_list_committed = 0;
  /* Initialize channels PRR to 100% */
  memset(channel_prr, PRR_SCALE, sizeof(channel_prr));
#endif /* CHAOS_MULTI_CHANNEL_ADAPTIVE */
#endif /* CHAOS_MULTI_CHANNEL */
}

ALWAYS_INLINE void
chaos_multichannel_update_black_list(uint8_t is_initiator, uint8_t round_synced, int rx_state, uint8_t app_id, chaos_header_t* const rx_header, chaos_header_t* const tx_header) {
#if CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE
  uint8_t channel = chaos_multichannel_get_current_channel();
  uint8_t channel_idx = CHANNEL_IDX(channel);
  uint8_t success = rx_state == CHAOS_TXRX_OK;
  // count channel hits after round sync only
  if(round_synced || success) {
    if(rx_state != CHAOS_RX_NO_SFD) { /* ignore NO_SFD since it could be random back-off */
      uint8_t old_prr = channel_prr[channel_idx];
      channel_prr[channel_idx] = UPDATE_PRR(success, channel_prr[channel_idx]);
      uint8_t block_channel = channel_prr[channel_idx] < CHAOS_CHANNEL_PRR_THRESHOLD;
      channel_black_list_local = block_channel ? channel_black_list_local | (1U << (unsigned int)channel_idx) : channel_black_list_local & ~(1U << (unsigned int)channel_idx);
    }
    if(app_id == rx_header->id) {
      if(success) { /* read flags from rx_header only if it is a sane packet */
        if(!is_initiator) {
          channel_black_list_committed = rx_header->channels_black_list_committed;
        }
        /* merge learned blacklist */
        channel_black_list_collected |= rx_header->channels_black_list_collected;
      }
      /* merge local blacklist and fill tx_header */
      tx_header->channels_black_list_collected = channel_black_list_collected | channel_black_list_local;
      tx_header->channels_black_list_committed = channel_black_list_committed;
    }
    //COOJA_DEBUG_PRINTF("update_black_list ch-%u St-1 %u St %u yt %u b %u bl %x", channel, old_prr, channel_prr[channel_idx], success, block_channel, channel_black_list_local);
  }
#endif /* CHAOS_MULTI_CHANNEL */
}

ALWAYS_INLINE void
chaos_multichannel_round_init(uint8_t is_initiator, chaos_header_t* const tx_header) {
#if CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE
  if(is_initiator) {
    /* commit new blacklist */
    channel_black_list_committed = channel_black_list_collected | channel_black_list_local;
    /* if too many blocked channels, enable some
     * PS: This code is not optimized but executes in constant time
     */
    uint8_t number_of_blocked_channels = 0;
    uint8_t i;
    for(i = 0; i < 8 * sizeof(channel_black_list_committed); i++) {
      number_of_blocked_channels += (channel_black_list_committed & (1U<<i));
    }
    for(i = 0; i < 8 * sizeof(channel_black_list_committed); i++) {
      if(number_of_blocked_channels > CHAOS_BLACK_LIST_MAX_SIZE
          && ((channel_black_list_committed & (1U<<i)) ^ (channel_black_list_local & (1U<<i)))) {
        number_of_blocked_channels--;
        channel_black_list_committed &= ~(1U<<i);
      }
    }
    /* set the new relaxed blacklist in tx msg */
    tx_header->channels_black_list_committed = channel_black_list_committed;
  }
  tx_header->channels_black_list_collected = channel_black_list_local;
  /* reset merged flags, but keep local flags */
  channel_black_list_collected = 0;
#endif /* CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE */
}

ALWAYS_INLINE unsigned int
    chaos_multichannel_get_black_list_local() {
#if CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE
  return channel_black_list_local;
#else
  return 0;
#endif
}

ALWAYS_INLINE unsigned int
    chaos_multichannel_get_black_list_merged() {
#if CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE
  return channel_black_list_collected;
#else
  return 0;
#endif
}

ALWAYS_INLINE unsigned int
    chaos_multichannel_get_black_list_committed() {
#if CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE
  return channel_black_list_committed;
#else
  return 0;
#endif
}

ALWAYS_INLINE uint16_t
    chaos_multichannel_lookup_channel(uint16_t round_number, uint16_t slot_number) {
#if CHAOS_MULTI_CHANNEL
  uint8_t channel_sequence_offset = 0;
#if CHAOS_MULTI_CHANNEL_PARALLEL_SEQUENCES
  if(slot_number > 0) { /* don't mess first slot for quicker association */
    channel_sequence_offset = chaos_random_generator_fast() % CHAOS_MULTI_CHANNEL_PARALLEL_SEQUENCES;
  }
#endif /* CHAOS_MULTI_CHANNEL_PARALLEL_SEQUENCES */
  return chaos_channel_hopping_sequence[((round_number<<CHAOS_HOPPING_ROUND_SHIFT) + slot_number + channel_sequence_offset) & (CHAOS_HOPPING_SEQUENCE_SIZE-1)];
// channel: x % 16 + 11
//return ((round_number + slot_number) & (CHAOS_HOPPING_SEQUENCE_SIZE-1)) + RF_FIRST_CHANNEL;
#else
  return CHAOS_RF_CHANNEL;
#endif /* CHAOS_MULTI_CHANNEL */
}

ALWAYS_INLINE uint16_t
    chaos_multichannel_get_next_channel(uint16_t round_number, uint16_t slot_number) {
  uint16_t next_channel = chaos_multichannel_lookup_channel(round_number, slot_number);
#if CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE
  uint8_t mask = 1U<<(CHANNEL_IDX(next_channel));
  uint8_t blocked = channel_black_list_committed & mask;
  uint8_t given_a_chance = ((slot_number + (round_number<<3)) & 7) == 0; /* allow a black-listed channel every 8th slot */
  next_channel = ( blocked && !given_a_chance ) ? chaos_current_channel : next_channel;
#endif /* CHAOS_MULTI_CHANNEL */
  return next_channel;
}

/* this function has a side effect: updates chaos_current_channel */
ALWAYS_INLINE uint16_t
    chaos_multichannel_update_current_channel(uint16_t round_number, uint16_t slot_number) {
#if CHAOS_MULTI_CHANNEL
  return chaos_current_channel = chaos_multichannel_get_next_channel(round_number, slot_number);
#else
  return chaos_multichannel_get_next_channel(round_number, slot_number);
#endif /* CHAOS_MULTI_CHANNEL */
}

ALWAYS_INLINE uint16_t
    chaos_multichannel_get_current_channel() {
#if CHAOS_MULTI_CHANNEL
  return chaos_current_channel;
#else
  return CHAOS_RF_CHANNEL;
#endif /* CHAOS_MULTI_CHANNEL */
}

