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
 *         A2-Synchrotron MAC config
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */

#ifndef CHAOS_CONFIG_H_
#define CHAOS_CONFIG_H_

#include "contiki.h"

#define CHAOS_HW_SECURITY ((LLSEC802154_SECURITY_LEVEL > 0) && (LLSEC802154_SECURITY_LEVEL <= 7))

//#define ENABLE_COOJA_DEBUG 0
//#include "dev/cooja-debug.h"
#ifndef CHAOS_DEBUG_PRINTF
#define CHAOS_DEBUG_PRINTF 0
#endif

#ifndef CHAOS_LOG_FLAGS
#define CHAOS_LOG_FLAGS 1
#endif

#if CHAOS_DEBUG_PRINTF
#include "stdio.h"
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* to sync on rx packets */
#ifndef CHAOS_ENABLE_SFD_SYNC
#define CHAOS_ENABLE_SFD_SYNC  (!(COOJA))
#endif

#define ASSOCIATION_SLOT_LEN (7*(RTIMER_SECOND/1000))

//#define CHAOS_APP_PAYLOAD_LEN (50)
// for chaos round pre-processing that is invoked outside the interrupt context
#define ROUND_PRE_PROCESSING_TIME (3*(CLOCK_SECOND/100))
#define APP_DUMMY_LEN (0)
#define ROUND_OFFSET_TO_RADIO_ON_MIN (4*(RTIMER_SECOND/1000)/2)
#define ROUND_OFFSET_TO_RADIO_ON (8*(RTIMER_SECOND/1000)/2)
#define ROUND_OFFSET_TO_RADIO_ON_INITIATOR (ROUND_OFFSET_TO_RADIO_ON)
#define RTIMER_LATENCY ((RTIMER_SECOND/1000)) /* rtimer ticks delay of rtimer wakeup from scheduled time */
#define RTIMER_LATENCY_INITIATOR RTIMER_LATENCY
#define SFD_DETECTION_TIME_MIN (US_TO_DCOTICKS(192+64)) /* radio turn around time? */
#define RX_GUARD_TIME (10) /* rtimer ticks per slot */
#define ROUND_GUARD_TIME ((RTIMER_SECOND/1000))

/* start a round every CHAOS_INTERVAL seconds
 * One needs to ensure that  CHAOS_ROUND_MAX_SLOTS * CHAOS_SLOT_LEN does not exceed CHAOS_INTERVAL minus some buffer for app, logging etc.
 * */
#ifndef CHAOS_MULTI_CHANNEL_PARALLEL_SEQUENCES
#define CHAOS_MULTI_CHANNEL_PARALLEL_SEQUENCES 0
#endif

#ifndef CHAOS_USE_MSPGCC_RAND
#define CHAOS_USE_MSPGCC_RAND 0
#endif /* CHAOS_USE_MSPGCC_RAND */

#ifndef CHAOS_MULTI_CHANNEL
#define CHAOS_MULTI_CHANNEL 0
#endif /* CHAOS_MULTI_CHANNEL */

#ifndef CHAOS_MULTI_CHANNEL_ADAPTIVE
#define CHAOS_MULTI_CHANNEL_ADAPTIVE 0
#endif /* CHAOS_MULTI_CHANNEL_ADAPTIVE */

#ifndef CHAOS_RF_CHANNEL
#define CHAOS_RF_CHANNEL 26
#endif /* CHAOS_RF_CHANNEL */

#ifndef CHAOS_TX_POWER
#if TESTBED == twist
#define CHAOS_TX_POWER CC2420_TXPOWER_MAX
#else
#define CHAOS_TX_POWER CC2420_TXPOWER_MAX
#endif
#endif

/* Time before hopping channel when waiting for association */
#ifndef CHAOS_ASSOCIATION_HOP_CHANNEL_THERSHOLD
#define CHAOS_ASSOCIATION_HOP_CHANNEL_THERSHOLD (100)//(2*(RTIMER_SECOND/ROUND_GUARD_TIME))
#endif /* CHAOS_ASSOCIATION_HOP_CHANNEL_THERSHOLD */

#if CHAOS_MULTI_CHANNEL
/* radio specific */
#define CHAOS_NUMBER_OF_CHANNELS 16
#define RF_FIRST_CHANNEL  11

/* Default IEEE 802.15.4e hopping sequences, obtained from https://gist.github.com/twatteyne/2e22ee3c1a802b685695 */
/* 16 channels, sequence length 16 */
#define CHAOS_HOPPING_SEQUENCE_16_16 ((uint8_t[]){16, 17, 23, 18, 26, 15, 25, 22, 19, 11, 12, 13, 24, 14, 20, 21})
#define CHAOS_HOPPING_SEQUENCE_16_32 ((uint8_t[]){16, 17, 15, 18, 11, 16, 20, 19, 19, 26, 12, 17, 22, 18, 13, 11, 21, 14, 25, 23, 23, 26, 24, 24, 20, 15, 12, 13, 25, 22, 21, 14})
#define CHAOS_HOPPING_SEQUENCE_16_64 ((uint8_t[]){26, 16, 16, 18, 20, 16, 25, 21, 12, 18, 21, 17, 24, 18, 25, 21, 15, 24, 19, 24, 22, 11, 24, 18, 25, 21, 15, 14, 17, 15, 14, 13, 14, 19, 12, 13, 17, 20, 11, 11, 20, 23, 25, 13, 23, 22, 26, 26, 22, 23, 17, 11, 16, 20, 14, 13, 15, 22, 26, 23, 12, 19, 12, 19})

/* 4 channels, sequence length 16 */
#define CHAOS_HOPPING_SEQUENCE_4_16 ((uint8_t[]){20, 26, 25, 26, 15, 15, 25, 20, 26, 15, 26, 25, 20, 15, 20, 25})
#define CHAOS_HOPPING_SEQUENCE_4_16_2 ((uint8_t[]){20, 20, 26, 26, 25, 25, 26, 26, 15, 15, 25, 25, 20, 20, 26, 26})
#define CHAOS_HOPPING_SEQUENCE_2_2 ((uint8_t[]){ 26, 21 })
#define CHAOS_HOPPING_SEQUENCE_1_2_26 ((uint8_t[]){ 26, 26 })
#define CHAOS_HOPPING_SEQUENCE_1_2_21 ((uint8_t[]){ 21, 21 })

extern uint8_t chaos_channel_hopping_sequence[];
/* The hopping sequence. Its size should be a power of two. */
#define CHAOS_HOPPING_SEQUENCE CHAOS_HOPPING_SEQUENCE_16_64
#define CHAOS_HOPPING_ROUND_SHIFT (4) //for starting the next round with another portion of the hopping sequence

#if CHAOS_MULTI_CHANNEL_ADAPTIVE
/* 8bit precision but doing calculation in 16bit to avoid overflow */
#define PRR_SCALE ((uint16_t)(256U))
/* Alpha controls damping speed for old measurements.
 * Alpha closer to one(==PRR_SCALE) means quicker damping */
#define PRR_ALPHA ((uint16_t)((PRR_SCALE)/8))
#define CHAOS_CHANNEL_PRR_THRESHOLD ((uint16_t)((PRR_SCALE)/8))

/* At least X white-listed channels shall be available.
 * We enable the initiator preferred channels on shortage. */
#define CHAOS_BLACK_LIST_MAX_SIZE (CHAOS_HOPPING_SEQUENCE_SIZE/2)
#endif /* CHAOS_MULTI_CHANNEL_ADAPTIVE */
#else
#define CHAOS_NUMBER_OF_CHANNELS 1
#endif /* CHAOS_MULTI_CHANNEL */

/*****************************************************************************/
/* hack for static asserts in compile-time
 * http://www.pixelbeat.org/programming/gcc/static_assert.html */
#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
/* These can't be used after statements in c89. */
#ifdef __COUNTER__
  #define STATIC_ASSERT(e,m) \
    ;enum { ASSERT_CONCAT(static_assert_, __COUNTER__) = 1/(!!(e)) }
#else
  /* This can't be used twice on the same line so ensure if using in headers
   * that the headers are not included twice (by wrapping in #ifndef...#endif)
   * Note it doesn't cause an issue when used on same line of separate modules
   * compiled with gcc -combine -fwhole-program.  */
  #define STATIC_ASSERT(e,m) \
    ;enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }
#endif

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
//#pragma message "CHAOS_NODES = " STR(CHAOS_NODES)

 /* include the project config */
 /* PROJECT_CONF_H might be defined in the project Makefile */
 #ifdef PROJECT_CONF_H
 #include PROJECT_CONF_H
 #endif /* PROJECT_CONF_H */

#endif /* CHAOS_CONFIG_H_ */
