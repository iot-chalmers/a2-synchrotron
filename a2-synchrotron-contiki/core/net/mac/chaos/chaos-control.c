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

#include <string.h>
#include "contiki.h"
#include "sys/process.h"
#include "sys/rtimer.h"
#include "net/netstack.h"

#include "chaos.h"
#include "chaos-random-generator.h"
#include "node.h"
#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
#include "join.h"
#endif
#include "chaos-log.h"
#include "chaos-scheduler.h"
#include "chaos-control.h"
#include "chaos-config.h"
//for NETSTACK_RADIO_sfd_sync
#include "chaos-platform-specific.h"
#include "leds.h"
#define ENABLE_COOJA_DEBUG COOJA
#include "dev/cooja-debug.h"

#define CHAOS_SLOT_TIMING_PROFILE 1

#define DCO_SYNC_PERIOD 10 //seconds

#define ROUND_DELAY_COMPENSATION 0


/* how many failed rounds to allow before associating again? */
//#ifndef CHAOS_FAILED_ROUNDS_RESYNC_THRESHOLD
//#define CHAOS_FAILED_ROUNDS_RESYNC_THRESHOLD (2)
//#endif

/* CHAOS Contiki processes */
PROCESS(chaos_process, "CHAOS: main process");
static PT_THREAD(chaos_round_proc(struct rtimer *t, void *ptr));
static struct pt chaos_round_proc_pt;

static volatile vht_clock_t round_rtimer = 0;
static uint16_t round_number = 0;
static volatile rtimer_clock_t current_round_begin = 0;
static volatile clock_time_t last_round_clock_time = 0;

/* used in RTIMER_DCO_SYNC() */
volatile rtimer_clock_t rtimer_ref = 0, dco_ref = 0;

static volatile uint8_t failed_rounds = 0;

rtimer_clock_t chaos_control_get_current_round_begin(){
  return current_round_begin;
}

vht_clock_t chaos_control_get_round_rtimer(){
  return round_rtimer;
}

uint16_t chaos_get_round_number(){
  return round_number;
}

//static void init_app_ids(){
//  int i;
//  for(i = 0; i < chaos_app_count; i++ ){
//    chaos_apps[i]->id = i;
//  }
//}
//int join_last_round_is_complete( void );

typedef struct{
  uint8_t                   //control flags
    slot_count :5,       //number of slots into which nodes wrote their ids
    overflow :1,              /* available join slots too short */
    commit :2;                /* commit join */
} commit_field_t;

static void
chaos_post_processing(void)
{
  COOJA_DEBUG_LINE();
  LEDS_ON(LEDS_RED);
  chaos_log_process_pending();
  int i;
  printf("{rd %u stats} ", round_number);
  for( i=0; i<CHAOS_SLOT_STATS_SIZE; i++ ){
    printf("%u ", chaos_slot_stats[i]);
  }
  printf(" end\n");
#if CHAOS_SLOT_TIMING_PROFILE
  //slot timing profile
  printf("{rd %u timing max} ", round_number);
  for( i=0; i<SLOT_TIMING_SIZE; i++ ){
    printf("%lu ", DCO_TO_US(chaos_slot_timing_log_max[i]));
  }
  printf(" end\n");
  printf("{rd %u timing min} ", round_number);
  for( i=0; i<SLOT_TIMING_SIZE; i++ ){
    printf("%lu ", DCO_TO_US(chaos_slot_timing_log_min[i]));
  }
  printf(" end\n");
#endif
  //duty cycle, per million
  uint32_t tx_permil = (chaos_slot_timing_tx_sum);
  uint32_t rx_permil = (chaos_slot_timing_rx_sum);
  uint32_t dc_permil =  (chaos_slot_timing_tx_sum + chaos_slot_timing_rx_sum);
  printf("{rd %u dc} interval %lu tx %lu + rx %lu = dc %lu [us]\n", round_number, RTIMER_TO_DCO_U32(CHAOS_INTERVAL), tx_permil, rx_permil, dc_permil);

//  printf("{rd %u slots} ", round_number);
//  for( i=0; i<sizeof(chaos_slot_log); i++ ){
//    printf("%01x%c", chaos_slot_log[i], (i % 40 == 0) ? '\n' : ' ');
//  }
//  printf(" end\n");

#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
  const chaos_app_t* current_app = scheduler_get_current_app();
  if( current_app != NULL ){
    if(strcmp(current_app->name, "join") == 0) {
      PRINTF("{rd %u commit %d join} complete %d/%d, idx %d, n %d\n", round_number, join_get_commit_slot(), join_last_round_is_complete(), join_get_off_slot(), chaos_has_node_index ? chaos_node_index : -1, chaos_node_count);
#if JOIN_LOG_FLAGS
      printf("{rd %u join slots} ", round_number);
      int i;
      for( i=0; i<join_get_off_slot(); i++ ){
        //commit_field_t cc = chaos_join_commit_log[i];
        //printf("%u", cc.slot_count, cc.overflow, cc.commit);
        printf("%u,", chaos_join_commit_log[i]);
      }
      printf("\n{rd %u join flags} ", round_number);
      for( i=0; i<join_get_off_slot(); i++ ){
        //commit_field_t cc = chaos_join_commit_log[i];
        //printf("%u", cc.slot_count, cc.overflow, cc.commit);
        printf("%u,", chaos_join_flags_log[i]);
      }
      printf("\n");
#endif /* JOIN_LOG_FLAGS */
      //print the faulty CRC but passing packet (discovered by MIC check)
      uint8_t faulty_pkt = rx_pkt_crc_err[sizeof(rx_pkt_crc_err)-1];
//      if(faulty_pkt){
//        int i;
//        printf("{rd %u crc} ", round_number);
//        for( i=0; i<sizeof(rx_pkt_crc_err); i++ ){
//          printf("%02x%c", rx_pkt_crc_err[i], (i % 32 == 0) ? '\n' : ' ');
//        }
//        printf(" end\n");
//
//        printf("{rd %u crc org} ", round_number);
//        for( i=0; i<sizeof(rx_pkt_copy); i++ ){
//          printf("%02x%c", rx_pkt_copy[i], (i % 32 == 0) ? '\n' : ' ');
//        }
//        printf(" end\n");
//
//
//      }
//
//      if(join_debug_var.info){
//        printf("{rd %u-%u err} %s %s l %u i %u\n", round_number, join_debug_var.slot, CHAOS_STATE_TO_STRING(join_debug_var.slot_type), CHAOS_RX_STATE_TO_STRING(join_debug_var.slot_status), join_debug_var.debug_pos, join_debug_var.info);
//      }

      if( IS_INITIATOR() && (chaos_node_count > FAULTY_NODE_COUNT || faulty_pkt || COOJA)){
        PRINTF("{rd %u join list} begin > ", round_number);
        join_print_nodes();
        PRINTF("< end join list\n");
      }
    }
  }
#endif /* NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC */
  chaos_random_generator_update_table();
  LEDS_OFF(LEDS_RED);
  COOJA_DEBUG_LINE();
}

static void
chaos_pre_processing(void* arg)
{
  COOJA_DEBUG_LINE();
  LEDS_ON(LEDS_GREEN);
#if !COOJA
  /* calibrate the DCO every 10 seconds */
  static unsigned long last_calibration_time = 0;
  unsigned long now = clock_seconds();
  if( now - last_calibration_time > DCO_SYNC_PERIOD ) {
    last_calibration_time = now;
    chaos_platform_sync_dco_log(round_number, 0); /* disable logging/PRINTF of sync_dco */
  }
  NETSTACK_RADIO_sfd_sync(1, 0); /* we need to run it after msp430_sync_dco because it resets the timerB (DCO) config */
#endif
  RTIMER_DCO_SYNC();
  LEDS_OFF(LEDS_GREEN);
  COOJA_DEBUG_LINE();
}

/* Protothread for a chaos round, called from rtimer interrupt
 * and scheduled from XYZ */
static
PT_THREAD(chaos_round_proc(struct rtimer *t, void *ptr))
{
  PT_BEGIN(&chaos_round_proc_pt);
  round_number++;
  uint8_t current_app_id = 0;
  const chaos_app_t* current_app = scheduler_round_begin(round_number, &current_app_id);
  if( current_app != NULL ){
    COOJA_DEBUG_STR("current_app->round_begin(round_number, current_app_id)");
    last_round_clock_time = clock_time();
    current_app->round_begin(round_number, current_app_id);
    leds_blink();
    COOJA_DEBUG_LINE();
    scheduler_round_end();
  } else {
    failed_rounds=CHAOS_FAILED_ROUNDS_RESYNC_THRESHOLD; //app is NULL!! Panic!
  }

  COOJA_DEBUG_LINE();
  process_poll(&chaos_process);
  PT_END(&chaos_round_proc_pt);
}

static
PT_THREAD(chaos_associate_proc(struct pt *pt))
{
  PT_BEGIN(pt);

  rtimer_clock_t t_sfd_actual_rtimer;
  uint16_t slot_number;
  const chaos_app_t* app =  NULL;
  do{
    COOJA_DEBUG_LINE();
    uint8_t app_id, associated;
    associated = chaos_associate(&t_sfd_actual_rtimer, &round_number, &slot_number, &app_id);
    if( associated && !IS_INITIATOR() ){
      round_rtimer = get_round_rtimer();
      if(app_id < chaos_app_count){
        app = chaos_apps[app_id];
      }
      PRINTF("Assoc: rd-%u st-%u, appid: %u\n", round_number, slot_number, app_id);
    } else {
      app =  NULL;
//      static struct etimer et;
//      etimer_set(&et, 1);
//      NETSTACK_RADIO.off();
//      PT_WAIT_UNTIL(pt, etimer_expired (&et));
    }
  } while (app == NULL && !IS_INITIATOR());

  scheduler_round_end();
  PT_END(pt);
}


/* Checks if the current time has past a ref time + offset. Assumes
 * a single overflow and ref time prior to now. */
uint8_t
chaos_schedule_check_timer_miss(uint16_t ref_time, uint16_t offset, uint16_t now)
{
  uint16_t target = ref_time + offset - (uint16_t)RTIMER_MIN_DELAY;
  uint8_t now_has_overflowed = now < ref_time;
  uint8_t target_has_overflowed = target < ref_time;

  if(now_has_overflowed == target_has_overflowed) {
    /* Both or none have overflowed, just compare now to the target */
    return target <= now;
  } else {
    /* Either now or target of overflowed.
     * If it is now, then it has passed the target.
     * If it is target, then we haven't reached it yet.
     *  */
    return now_has_overflowed;
  }
}

uint8_t
chaos_schedule_check_vht_timer_miss(vht_clock_t ref_time, vht_clock_t offset, vht_clock_t now)
{
  vht_clock_t target = ref_time + offset - RTIMER_MIN_DELAY;
  uint8_t now_has_overflowed = now < ref_time;
  uint8_t target_has_overflowed = target < ref_time;

  if(now_has_overflowed == target_has_overflowed) {
    /* Both or none have overflowed, just compare now to the target */
    return target <= now;
  } else {
    /* Either now or target of overflowed.
     * If it is now, then it has passed the target.
     * If it is target, then we haven't reached it yet.
     *  */
    return now_has_overflowed;
  }
}
/* Schedule a wakeup at a specified offset from a reference time.
 * Provides basic protection against missed deadlines and timer overflows
 * A non-zero return value signals to chaos_schedule_round a missed deadline.
 * If conditional: schedule only if the deadline is not missed.
 * Otherwise: schedule regardless of deadline miss. */
static uint8_t
chaos_schedule_round(struct rtimer *tm, rtimer_clock_t ref_time, rtimer_clock_t offset, int conditional)
{
  uint16_t now;
  uint8_t r = 0, missed = 0;
  now = RTIMER_NOW();
  missed = chaos_schedule_check_timer_miss(ref_time, offset, now);
//  uint16_t goal = ref_time + offset;
//  PRINTF("Schedule: %u + %u = %u @ %u\n", (uint16_t)ref_time, (uint16_t)offset, goal, now);
  if(missed) {
    PRINTF("Schedule: Missed\n");
    if(conditional) {
      PRINTF("Schedule: exit!\n");
      return 0;
    }
  }
  ref_time += offset;
  r = rtimer_set(tm, ref_time, 1, (void (*)(struct rtimer *, void *))chaos_round_proc, NULL /*(void*)&status*/);

  if(r != RTIMER_OK) {
    PRINTF("Schedule: RTIMER_RET: %d\n", r);
    return 0;
  }
//  PRINTF("Schedule: wup %u = %u + %u\n", ref_time, ref_time-offset, offset);
  return 1;
}


/* The main CHAOS process */
PROCESS_THREAD(chaos_process, ev, data)
{
  static struct pt associate_pt;
  static struct rtimer round_scheduler_timer;
  static struct ctimer chaos_pre_processing_ctimer;
  static int success;
  static rtimer_clock_t round_offset_to_radio_on = 0;
  /* rtimer ticks delay of rtimer wakeup from scheduled time */
  static rtimer_clock_t rtimer_delay = 0;
  static rtimer_clock_t round_scheduled_offset;
  PROCESS_BEGIN();

  scheduler_init();
  chaos_pre_processing(NULL);
  COOJA_DEBUG_LINE();

    while(1) {
      if( IS_INITIATOR() ){
        PRINTF("Chaos initiator starting with ID %u\n", node_id);
        rtimer_delay = RTIMER_LATENCY_INITIATOR;
        round_offset_to_radio_on = ROUND_OFFSET_TO_RADIO_ON_INITIATOR;
        round_rtimer = VHT_NOW();
      } else {
        rtimer_delay = RTIMER_LATENCY;
        round_offset_to_radio_on = ROUND_OFFSET_TO_RADIO_ON;
      }
      /* Associate:
       * Try to associate to a network or start one if node is CHAOS coordinator */
      failed_rounds = 0;
      init_node_index();
      PROCESS_PT_SPAWN(&associate_pt, chaos_associate_proc(&associate_pt));
      chaos_log_process_pending();
      /* Schedule next round and re-associate: if no sfd sync during the round or we missed the round start (or something else failed)*/
      do{
        current_round_begin = scheduler_get_next_round_begin();
        COOJA_DEBUG_PRINTF("rr 0x%x %lx rb 0x%x %lx rb-w 0x%x wu 0x%x fs 0x%x n 0x%x\n",
            VHT_TO_RTIMER(round_rtimer), (round_rtimer),
            (uint16_t)current_round_begin, RTIMER_TO_VHT(current_round_begin),
            (uint16_t)(current_round_begin - round_offset_to_radio_on - rtimer_delay - ROUND_GUARD_TIME),
            (uint16_t)(VHT_TO_RTIMER(round_rtimer) + current_round_begin - round_offset_to_radio_on - rtimer_delay - ROUND_GUARD_TIME),
            (uint16_t)(VHT_TO_RTIMER(round_rtimer) + current_round_begin),
            RTIMER_NOW());
        round_scheduled_offset = current_round_begin - round_offset_to_radio_on - ROUND_GUARD_TIME - rtimer_delay;
        success = chaos_schedule_round(&round_scheduler_timer, VHT_TO_RTIMER(round_rtimer), round_scheduled_offset, 1);
        round_rtimer += RTIMER_TO_VHT(current_round_begin);
        leds_blink();
        if( success ){
          COOJA_DEBUG_LINE();
          PROCESS_YIELD();
          COOJA_DEBUG_LINE();
          clock_time_t pre_processing_time = ((round_scheduled_offset * CLOCK_SECOND) / RTIMER_SECOND);
          ctimer_set_absolute(&chaos_pre_processing_ctimer, last_round_clock_time - ROUND_PRE_PROCESSING_TIME, pre_processing_time, chaos_pre_processing, NULL);
#if ROUND_DELAY_COMPENSATION

          rtimer_delay  = /* convert this */ VHT_TO_RTIMER(get_round_start() - round_rtimer) - (current_round_begin - ROUND_GUARD_TIME - round_offset_to_radio_on - rtimer_delay);
          /* add a guard */
          rtimer_delay += MAX(rtimer_delay >> 1, ROUND_OFFSET_TO_RADIO_ON);
#endif /* ROUND_DELAY_COMPENSATION */
          if( get_round_synced() ){ //sync round_rtimer
            round_rtimer = get_round_rtimer();
          }
          chaos_post_processing();
        } else {
          if(IS_INITIATOR()) {
            //TODO: should we increase round_number here? and what does the scheduler do if we skip one round?
            //work around: for now we just run the round we missed one round later (shift everything by one round)
            //compute next round start in case we missed its start
            round_number++;
          }
          PRINTF("{rd: %u} Skipping round ss %d rd %u!\n", round_number, success, rtimer_delay);
        }
#if ROUND_DELAY_COMPENSATION
        round_offset_to_radio_on = VHT_TO_RTIMER( get_round_offset_to_radio_on() );
        /* add a guard */
        round_offset_to_radio_on += MAX(round_offset_to_radio_on >> 1, ROUND_OFFSET_TO_RADIO_ON);
#endif /* ROUND_DELAY_COMPENSATION */
        failed_rounds = ( (get_round_synced() && success) || IS_INITIATOR() ) ? 0 : failed_rounds + 1;
        COOJA_DEBUG_LINE();
      } while( failed_rounds < CHAOS_FAILED_ROUNDS_RESYNC_THRESHOLD || IS_INITIATOR());
      COOJA_DEBUG_LINE();
    }

  PROCESS_END();
}

