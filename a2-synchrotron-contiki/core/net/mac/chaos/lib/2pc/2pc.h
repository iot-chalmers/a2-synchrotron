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
 *         Two-phase commit library
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#ifndef _TWO_PC_H_
#define _TWO_PC_H_

#include "chaos.h"
#include "testbed.h"
#include "chaos-config.h"
#include "3pc.h" /* for phases enum */

#define TWO_PC_LOG_FLAGS 0
#define TWO_PC_SLOT_LEN          (4*(RTIMER_SECOND/1000)+3*(RTIMER_SECOND/1000)/4)    //1 rtimer tick == 2*31.52 us

//force radio off after TWO_PC_ROUND_MAX_SLOTS slots
#define TWO_PC_ROUND_MAX_SLOTS   (350)

#define TWO_PC_SLOT_LEN_DCO      (TWO_PC_SLOT_LEN*CLOCK_PHI)    //TODO needs calibration

int two_pc_round_begin(const uint16_t round_number, const uint8_t app_id, uint32_t* two_pc_value, uint8_t* phase, uint8_t** final_flags);

int two_pc_is_pending(const uint16_t round_count);

int two_pc_get_flags_length(void);

uint16_t two_pc_get_off_slot();

int two_pc_agreed();

int two_pc_did_tx();

#endif /* _TWO_PC_H_ */
