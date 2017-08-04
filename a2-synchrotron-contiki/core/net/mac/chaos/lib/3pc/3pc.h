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
 *         Three-phase commit library
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */

#ifndef _THREE_PC_H_
#define _THREE_PC_H_

#include "chaos.h"
#include "testbed.h"
#include "chaos-config.h"

enum {
  PHASE_UNKNOWN = 0, PHASE_PROPOSE = 1, PHASE_PRE_COMMIT = 2, PHASE_COMMIT = 3, PHASE_ABORT = 4, PHASE_ABORT_TIMEOUT = 5, PHASE_ABORT_TIMEOUT_APP = 6, PHASE_COMMIT_TIMEOUT = 7, PHASE_UNCERTAIN = 8, PHASE_FAIL = 9
};

#define PHASE_TO_STR(phase) (\
  (phase == PHASE_PROPOSE) ? ("PHASE_PROPOSE") : \
      (phase == PHASE_PRE_COMMIT) ? ("PHASE_PRE_COMMIT") : \
          (phase == PHASE_COMMIT) ? ("PHASE_COMMIT") : \
              (phase == PHASE_ABORT) ? ("PHASE_ABORT") : \
                  (phase == PHASE_FAIL) ? ("PHASE_FAIL") : \
                      (phase == PHASE_ABORT_TIMEOUT) ? ("PHASE_ABORT_TIMEOUT") : \
                          (phase == PHASE_ABORT_TIMEOUT_APP) ? ("PHASE_ABORT_TIMEOUT_APP") : \
                            (phase == PHASE_COMMIT_TIMEOUT) ? ("PHASE_COMMIT_TIMEOUT") : \
                                (phase == PHASE_UNCERTAIN) ? ("PHASE_UNCERTAIN") : ("PHASE_UNKNOWN") \
)

#define THREE_PC_LOG_FLAGS 0

#if THREE_PC_LOG_FLAGS
#define THREE_PC_SLOT_LEN          ((6*(RTIMER_SECOND/1000)) + (2*(RTIMER_SECOND/1000)/10))
#else
#define THREE_PC_SLOT_LEN          (4*(RTIMER_SECOND/1000)+3*(RTIMER_SECOND/1000)/4)
#endif

//force radio off after THREE_PC_ROUND_MAX_SLOTS slots
#define THREE_PC_ROUND_MAX_SLOTS   (350)

#define THREE_PC_SLOT_LEN_DCO      (THREE_PC_SLOT_LEN*CLOCK_PHI)    //TODO needs calibration

int three_pc_round_begin(const uint16_t round_number, const uint8_t app_id, uint32_t* three_pc_value, uint8_t* phase, uint8_t** final_flags);
int three_pc_is_pending(const uint16_t round_count);

int three_pc_get_flags_length(void);
uint16_t three_pc_get_off_slot();
extern uint8_t chaos_3pc_flags_log[THREE_PC_ROUND_MAX_SLOTS];
extern uint8_t chaos_3pc_phase_log[THREE_PC_ROUND_MAX_SLOTS];
#endif /* _THREE_PC_H_ */
