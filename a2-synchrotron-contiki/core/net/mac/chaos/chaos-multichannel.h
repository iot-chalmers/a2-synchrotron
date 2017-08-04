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

#ifndef CHAOS_MULTICHANNEL_H_
#define CHAOS_MULTICHANNEL_H_

#include "contiki.h"

#if CHAOS_MULTI_CHANNEL
#define CHAOS_HOPPING_SEQUENCE_SIZE (sizeof(CHAOS_HOPPING_SEQUENCE)) /* shall be a power of two, so x % s == x & (s-1) */
/* check if a power if two */
STATIC_ASSERT(!(CHAOS_HOPPING_SEQUENCE_SIZE & (CHAOS_HOPPING_SEQUENCE_SIZE-1)), "CHAOS_HOPPING_SEQUENCE_SIZE shall be a power of two so the optimization is valid %x == &(x-1)");
#endif /* CHAOS_MULTI_CHANNEL */

/* functions */
void chaos_multichannel_init(void);
ALWAYS_INLINE void chaos_multichannel_round_init(uint8_t is_initiator, chaos_header_t* const tx_header);
ALWAYS_INLINE void chaos_multichannel_update_black_list(uint8_t is_initiator, uint8_t round_synced, int rx_state, uint8_t app_id, chaos_header_t* const rx_header, chaos_header_t* const tx_header);
ALWAYS_INLINE unsigned int chaos_multichannel_get_black_list_local();
ALWAYS_INLINE unsigned int chaos_multichannel_get_black_list_merged();
ALWAYS_INLINE unsigned int chaos_multichannel_get_black_list_committed();
ALWAYS_INLINE uint16_t chaos_multichannel_get_current_channel();
ALWAYS_INLINE uint16_t chaos_multichannel_lookup_channel(uint16_t round_number, uint16_t slot_number);
ALWAYS_INLINE uint16_t chaos_multichannel_update_current_channel(uint16_t round_number, uint16_t slot_number);
#define HOP_CHANNEL(ROUND, SLOT) ( NETSTACK_RADIO_set_channel(chaos_multichannel_update_current_channel(ROUND, SLOT)) )
#define CHANNEL_IDX(C) ((C)-RF_FIRST_CHANNEL)

#endif /* CHAOS_MULTICHANNEL_H_ */
