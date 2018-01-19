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
 *         A2-Synchrotron controller.
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */

#ifndef CHAOS_CONTROL_H_
#define CHAOS_CONTROL_H_

#include "contiki.h"
#include "chaos-header.h"

typedef struct chaos_app{
  char* name;
  uint16_t slot_length;
  uint16_t max_slots;
  uint8_t requires_node_index;
  uint32_t round_interval;
  int (*is_pending)(const uint16_t round_count);
  void (*round_begin)(const uint16_t round_count, const uint8_t id);
  void (*round_begin_sniffer)(chaos_header_t* header);
  void (*round_end_sniffer)(const chaos_header_t* header);
} chaos_app_t;

uint16_t chaos_get_round_number();

rtimer_clock_t chaos_control_get_current_round_begin();

vht_clock_t chaos_control_get_round_rtimer();

uint8_t chaos_schedule_check_timer_miss(uint16_t ref_time, uint16_t offset, uint16_t now);

uint8_t chaos_schedule_check_vht_timer_miss(vht_clock_t ref_time, vht_clock_t offset, vht_clock_t now);

extern const uint8_t chaos_app_count;

extern const chaos_app_t* const chaos_apps[];

#define CHAOS_APP(name, slot_length, max_slots, requires_node_index, round_interval, is_pending, round_begin) const chaos_app_t name = {#name, slot_length, max_slots, requires_node_index, round_interval ? round_interval*(RTIMER_SECOND/10) : CHAOS_INTERVAL, is_pending, round_begin, NULL, NULL}

#define CHAOS_SERVICE(name, slot_length, max_slots, requires_node_index, is_pending, round_begin, sniffer_begin, sniffer_end) const chaos_app_t name = {#name, slot_length, max_slots, requires_node_index, CHAOS_INTERVAL, is_pending, round_begin, sniffer_begin, sniffer_end}

//you can have CHAOS_APPS only once, just like autostart in Contiki
#define CHAOS_APPS(...) const chaos_app_t* const chaos_apps[] = {__VA_ARGS__}; \
		                    const uint8_t chaos_app_count = sizeof(chaos_apps)/sizeof(chaos_apps[0]);

#endif /* CHAOS_CONTROL_H_ */
