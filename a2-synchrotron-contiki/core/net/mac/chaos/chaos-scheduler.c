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
 *         A2-Synchrotron - scheduler.
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */

#include <stdio.h>

#include "chaos.h"
#include "chaos-header.h"
#include "chaos-control.h"
#include "chaos-scheduler.h"
#include "chaos-config.h"
#include <project-conf.h>

#undef ENABLE_COOJA_DEBUG
#define ENABLE_COOJA_DEBUG 0
#include "dev/cooja-debug.h"

//
//#ifndef CHAOS_INITIAL_JOIN_ROUNDS
//#define CHAOS_INITIAL_JOIN_ROUNDS 5
//#endif

static const chaos_app_t* current_app = NULL;
static uint8_t current_app_id = 0;
static const chaos_app_t* next_app = NULL;
static uint8_t next_app_id = 0;
static rtimer_clock_t next_round_begin = 0;

void scheduler_init(){
  if( IS_INITIATOR() ){
    int i;
    for(i = 0; i < chaos_app_count; i++){
      if( chaos_apps[i]->is_pending(0) ){
        COOJA_DEBUG_PRINTF("scheduler init: app: %s", chaos_apps[i]->name);
        next_app_id = i;
        next_app = chaos_apps[i];
        next_round_begin = current_app ? current_app->round_interval : CHAOS_INTERVAL;
        break;
      }
    }
  }
}

const chaos_app_t* scheduler_round_begin(const uint16_t round_count, uint8_t* app_id_ptr){
  current_app = next_app;
  current_app_id = next_app_id;
  next_round_begin = current_app ? current_app->round_interval : CHAOS_INTERVAL;

  if( IS_INITIATOR() && chaos_app_count > 0 ){
    int i;
    for(i = 0; i < chaos_app_count; i++){
      if( chaos_apps[i]->is_pending(round_count + 1) ){
        next_app_id = i;
        next_app = chaos_apps[next_app_id];
        COOJA_DEBUG_PRINTF("scheduler: current app: %s, next app: %s, start %u", current_app->name, next_app->name, next_round_begin);
        break;
      }
    }
  }
  *app_id_ptr = current_app_id;
  return current_app;
}

void scheduler_round_end(){
  if( !IS_INITIATOR() && get_round_synced() ){
    next_app_id = get_next_round_id();
    //XXX static per app round interval
    next_round_begin = current_app ? current_app->round_interval : CHAOS_INTERVAL;
    if( /*next_round_begin > 0 &&*/ next_app_id < chaos_app_count ){
      next_app = chaos_apps[next_app_id];
      COOJA_DEBUG_PRINTF("scheduler: current app: %s, next app: %s, next begin %u", current_app ? current_app->name : "null", next_app ? next_app->name : "null", next_round_begin);
    } else {
      next_app = NULL;
      PRINTF("Error: invalid data! Cannot schedule! next begin %u, next app id: %u\n", next_round_begin, next_app_id);
    }
  }
}

const chaos_app_t* scheduler_get_next_round_app(){
  return next_app;
}

/* correct only at round end!! */
const chaos_app_t* scheduler_get_current_app(){
  return current_app;
}

uint8_t scheduler_get_next_round_app_id(){
  return next_app_id;
}

rtimer_clock_t scheduler_get_next_round_begin(){
  return next_round_begin;
}
