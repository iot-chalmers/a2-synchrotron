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
 *         Collect library
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#ifndef _CHAOS_COLLECT_H_
#define _CHAOS_COLLECT_H_

#include "chaos.h"
#include "testbed.h"
#include "chaos-config.h"

#define CHAOS_COLLECT_SLOT_LEN          ((5*(RTIMER_SECOND/1000))+(1*(RTIMER_SECOND/1000)/2))    //TODO needs calibration

//force radio off after CHAOS_COLLECT_ROUND_MAX_SLOTS slots
#define CHAOS_COLLECT_ROUND_MAX_SLOTS   (255)

#define CHAOS_COLLECT_SLOT_LEN_DCO      (CHAOS_COLLECT_SLOT_LEN*CLOCK_PHI)    //TODO needs calibration

typedef uint16_t collect_value_t;
int chaos_collect_round_begin(const uint16_t round_number, const uint8_t app_id, collect_value_t** values, uint8_t** final_flags);

int chaos_collect_is_pending(const uint16_t round_count);

int chaos_collect_get_flags_length(void);

uint16_t chaos_collect_get_off_slot();

#endif /* _CHAOS_COLLECT_H_ */
