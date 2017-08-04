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
 *         A2-Synchrotron packet header.
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */

#ifndef CHAOS_HEADER_H
#define CHAOS_HEADER_H

#include "contiki.h"
#include "chaos-config.h"

#if CHAOS_HW_SECURITY
typedef union {
   uint32_t security_frame_counter;
   struct {
     uint8_t seq_number;
     uint8_t slot_number;
     uint16_t round_number;
   };
 } chaos_security_frame_counter_t;
#endif /* CHAOS_HW_SECURITY */

/**
 * \brief Data structure used to represent Chaos data. (20Bytes with security, src address and without rank)+ 2-6 bytes MIC + 2 CRC + 1 SFD + 4 preamble = 29Bytes *32us = 928us
 */
typedef struct __attribute__((packed)) chaos_header {
  uint8_t length;
  uint8_t chaos_fcf_0; /* frame control field p[0] */
  uint8_t chaos_fcf_1; /* frame control field p[1] */
  uint8_t slot_number;
  /* Addresses: We need to include the destination addresses,
   * otherwise the data frame destination would be the PAN coordinator,
   * per 2011 std */
  uint16_t dst_pan_id;
#if CHAOS_HW_SECURITY
  //put round number here instead of dst_node_id, so it appears in the "clear text header"
  uint16_t round_number; /**< Round number, incremented by the initiator at each Chaos round. */
#elif CHAOS_USE_DST_ID
  uint16_t dst_node_id;
#endif
#if CHAOS_USE_SRC_ID
  uint16_t src_node_id;
#endif
#if CHAOS_HW_SECURITY
  /*  Auxiliary security header
   *  Octets: 1                 | 4             | 0/1/5/9
   *          Security Control  | Frame Counter | Key Identifier
   */
  uint8_t security_control;
  chaos_security_frame_counter_t chaos_security_frame_counter; // TODO: check if the automatic alignment of round_number happens on MSB of security_frame_counter
#else
  uint16_t round_number; /**< Round number, incremented by the initiator at each Chaos round. */
#endif /* CHAOS_HW_SECURITY */
  //uint8_t config_view:6, leave:1;         //TODO: OL implement me
  uint8_t slot_number_msb:1,
          join:1,     //flags for dynamic app scheduling
          id:3,               //id application, channel control, join control, ...
          next_round_id:3;     //id of the next round
#if CHAOS_USE_SRC_RANK
  uint8_t src_rank; /* hop count */
#endif
  //uint8_t src_time_rank; // when did we sync last?
  rtimer_clock_t next_round_start;  //start time of the next round as offset to the start of this round (in 32kHz ticks) -> change this to more coarse grained
#if CHAOS_MULTI_CHANNEL_ADAPTIVE
  unsigned int channels_black_list_committed;
  unsigned int channels_black_list_collected;
#endif /* CHAOS_MULTI_CHANNEL_ADAPTIVE */
  uint8_t payload[];
} chaos_header_t;

#endif /* CHAOS_HEADER_H */
