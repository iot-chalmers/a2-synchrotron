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
#include "sys/rtimer.h"
#include "node.h"
#include "chaos.h"
#include "chaos-config.h"
#include <project-conf.h>

#ifndef WITH_CHAOS_LOG
#define WITH_CHAOS_LOG 1
#endif

#if !NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
#define FLAGS_LEN   ((CHAOS_NODES >> 3) + ((CHAOS_NODES & 7) ? 1 : 0))
#else
#define FLAGS_LEN   ((MAX_NODE_COUNT >> 3) + ((MAX_NODE_COUNT & 7) ? 1 : 0))
#endif

#define LOG_APP_FLAGS_LEN (FLAGS_LEN)

#define LONG_HEX_SIGNED(x) ((long)(x) < 0 ? "-" : "", (long)(x) < 0 ? -(unsigned)(x) : (x))

#if WITH_CHAOS_LOG
#include "stdio.h"
#define LOG(...) PRINTF(__VA_ARGS__)

/* Structure for a log. Union of different types of logs */
typedef enum chaos_log_type_enum {
    chaos_log_txrx = 0,
    chaos_log_message,
} chaos_log_type_enum_t;

typedef struct chaos_log_struct_t {
  union {
    //char message[20]; //upto 26 w adaptive channels
    struct {
//      vht_clock_t t_slot_start;
//      vht_clock_t t_go_goal;
//      vht_clock_t t_go_actual;
      vht_clock_t t_go_delta;
//      vht_clock_t t_sfd_goal;
//      vht_clock_t t_sfd_actual;
      vht_clock_t t_sfd_delta;
//      rtimer_clock_t t_packet_sfd_delta;
//      vht_clock_t t_slot_length_app;
//      rtimer_clock_t t_post_processing;
#if CHAOS_USE_SRC_ID
      uint16_t src_node_id;
#endif
#if CHAOS_MULTI_CHANNEL && CHAOS_MULTI_CHANNEL_ADAPTIVE
      uint16_t channel_black_list_local;
      uint16_t channel_black_list_committed;
      uint16_t channel_black_list_merged;
#endif /* CHAOS_MULTI_CHANNEL */
#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
      uint8_t join_committed:1, join_has_node_index: 1, join_slot_count:6;
#endif
#if CHAOS_LOG_FLAGS
      uint8_t app_flags[LOG_APP_FLAGS_LEN];
#endif /* CHAOS_LOG_FLAGS */
      uint8_t state:4, rx_status:4;
    } txrx;
  };
  uint16_t slot_number:9, channel:6, logtype:1;
  uint16_t round_number;
} chaos_log_t;

/* Prepare addition of a new log.
 * Returns pointer to log structure if success, NULL otherwise */
chaos_log_t *chaos_log_prepare_add();
/* Actually add the previously prepared log */
void chaos_log_commit();
/* Initialize log module */
void chaos_log_init();
/* Process pending log messages */
void chaos_log_process_pending();

#define CHAOS_LOG_ADD(log_type, init_code) do { \
    chaos_log_t *log = chaos_log_prepare_add(); \
    if(log != NULL) { \
      log->logtype = (log_type); \
      log->round_number = round_number; \
      log->slot_number = slot_number; \
      log->channel = chaos_multichannel_get_current_channel(); \
      init_code \
      chaos_log_commit(); \
    } \
  } while(0);

/* #define CHAOS_LOG_ADD_MSG(...) do { \
    chaos_log_t *log = chaos_log_prepare_add(); \
    if(log != NULL) { \
      log->logtype = (chaos_log_message); \
      log->round_number = round_number; \
      log->slot_number = slot_number; \
      snprintf(log->message, sizeof(log->message), __VA_ARGS__); \
      chaos_log_commit(); \
    } \
  } while(0); */

#define CHAOS_LOG_ADD_MSG(...)

#else /* WITH_CHAOS_LOG */

#define chaos_log_init()
#define chaos_log_process_pending()
#define CHAOS_LOG_ADD(log_type, init_code)
#define CHAOS_LOG_ADD_MSG(...)
#endif /* WITH_CHAOS_LOG */
