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
 *         Log functions for A2, meant for logging from interrupt
 *         during a slot operation. Saves slot information
 *         and adds the log to a ringbuffer for later printout.
 * \author
 *         Adopted from: Simon Duquennoy <simonduq@sics.se>
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */

#include "contiki.h"
#include <stdio.h>
#include "lib/ringbufindex.h"
#include "net/mac/chaos/chaos.h"
#include "net/mac/chaos/chaos-log.h"
#include "net/mac/chaos/chaos-config.h"

#if WITH_CHAOS_LOG

#define CHAOS_MAX_LOGS 256
#if (CHAOS_MAX_LOGS & (CHAOS_MAX_LOGS-1)) != 0
#error CHAOS_MAX_LOGS must be power of two
#endif

#if (CHAOS_MAX_LOGS < CHAOS_ROUND_MAX_SLOTS)
#error CHAOS_MAX_LOGS must have one entry per each possible slot
#endif

static struct ringbufindex log_ringbuf;
static chaos_log_t log_array[CHAOS_MAX_LOGS];
static int log_dropped = 0;


/* Process pending log messages */
void
chaos_log_process_pending()
{
  static int last_log_dropped = 0;
  int16_t log_index;
  /* Loop on accessing (without removing) a pending log */
  /* LOG("CHAOS: logs in queue %u, total dropped %u\n", ringbufindex_elements(&log_ringbuf), log_dropped); */
  if(log_dropped != last_log_dropped) {
    LOG("CHAOS:! logs dropped %u\n", log_dropped);
    last_log_dropped = log_dropped;
  }
  while((log_index = ringbufindex_peek_get(&log_ringbuf)) != -1) {
    chaos_log_t *log = &log_array[log_index];
    switch(log->logtype) {
      case chaos_log_txrx:
        LOG("{rd-%u st-%u ch-%u} ",
            log->round_number, log->slot_number, log->channel);
        LOG(CHAOS_STATE_TO_STRING(log->txrx.state));
        LOG(CHAOS_RX_STATE_TO_STRING(log->txrx.rx_status));
        if( CHAOS_RX_STATE_GET_TIMEOUT(log->txrx.rx_status) > -1 ) {
          LOG("%u ", (unsigned int)CHAOS_RX_STATE_GET_TIMEOUT(log->txrx.rx_status));
        }
//        LOG("bw %lu ", (unsigned long)log->txrx.t_slot_start/DCO_VHT_PHI);
//        LOG("gg %lx ", (unsigned long)log->txrx.t_go_goal);
//        LOG("gg %s%lx ", (long)log->txrx.t_go_goal < 0 ? "-" : "", (long)log->txrx.t_go_goal < 0 ? -(unsigned long)log->txrx.t_go_goal : log->txrx.t_go_goal);
//        LOG("ga %lx ", (unsigned long)log->txrx.t_go_actual);
        LOG("gd %s%lu ", (long)log->txrx.t_go_delta < 0 ? "-" : "", (long)log->txrx.t_go_delta < 0 ? (-(unsigned long)log->txrx.t_go_delta)/DCO_VHT_PHI : (log->txrx.t_go_delta)/DCO_VHT_PHI);
//        LOG("sg %lx ", (unsigned long)log->txrx.t_sfd_goal);
//        LOG("sa %lx ", (unsigned long)log->txrx.t_sfd_actual);
        LOG("sd %s%lu ", (long)log->txrx.t_sfd_delta < 0 ? "-" : "", (long)log->txrx.t_sfd_delta < 0 ? (-(unsigned long)log->txrx.t_sfd_delta)/DCO_VHT_PHI : (log->txrx.t_sfd_delta)/DCO_VHT_PHI);
//        LOG("pd %u ", (unsigned int)log->txrx.t_packet_sfd_delta);
//        LOG("sl %lu ", (unsigned long)log->txrx.t_slot_length_app);
//        LOG("sc %u ", (unsigned int)log->txrx.t_slot_length_corrected);
//        LOG("st %u ", (unsigned int)log->txrx.t_slot_length_actual);
//        LOG("pp %u ", (unsigned int)log->txrx.t_post_processing);
#if CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE
        LOG("bl %x ", (unsigned int)log->txrx.channel_black_list_local);
        LOG("bm %x ", (unsigned int)log->txrx.channel_black_list_merged);
        LOG("bc %x ", (unsigned int)log->txrx.channel_black_list_committed);
#endif /* CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE */
#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
        LOG("js %u/%01u/%01u ", (unsigned int)log->txrx.join_slot_count, log->txrx.join_committed != 0, log->txrx.join_has_node_index);
#endif
#if CHAOS_LOG_FLAGS
        LOG("ap ");
        uint8_t i;
        for(i = 0; i < LOG_APP_FLAGS_LEN; i++) {
          LOG("%02x", log->txrx.app_flags[i]);
        }
#endif /* CHAOS_LOG_FLAGS */
#if CHAOS_USE_SRC_ID
        LOG(" si %u", (unsigned int)log->txrx.src_node_id);
#endif
        LOG("\n");
        break;
      case chaos_log_message:
//        LOG("{rd-%u st-%u} ",
//            log->round_number, log->slot_number);
//        LOG("%s\n", log->message);
        break;
    }
    /* Remove input from ringbuf */
    ringbufindex_get(&log_ringbuf);
  }
}

/* Prepare addition of a new log.
 * Returns pointer to log structure if success, NULL otherwise */
chaos_log_t *
chaos_log_prepare_add()
{
  int log_index = ringbufindex_peek_put(&log_ringbuf);
  if(log_index != -1) {
    chaos_log_t *log = &log_array[log_index];
    return log;
  } else {
    log_dropped++;
    return NULL;
  }
}

/* Actually add the previously prepared log */
void
chaos_log_commit()
{
  ringbufindex_put(&log_ringbuf);
}

/* Initialize log module */
void
chaos_log_init()
{
  ringbufindex_init(&log_ringbuf, CHAOS_MAX_LOGS);
}

#endif /* WITH_CHAOS_LOG */
