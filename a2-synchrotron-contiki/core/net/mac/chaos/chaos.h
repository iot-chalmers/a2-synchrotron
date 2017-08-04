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
 *         A2-Synchrotron MAC implementation.
 *         Must be used with nordc as NETSTACK_CONF_RDC
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */

#ifndef CHAOS_H_
#define CHAOS_H_

#include "contiki.h"
#include "node-id.h"
#include "net/mac/rdc.h"
#include "net/llsec/llsec802154.h"
#include "flocklab-gpio.h"
#include "chaos-header.h"
#include "chaos-multichannel.h"
#include "net/mac/chaos/node/testbed.h"
#include "chaos-config.h"

//enable the nop delay
#ifndef CHAOS_CLOCK_DELAY_EXACT
#define CHAOS_CLOCK_DELAY_EXACT 0
#endif

#define CHAOS_MAX_RANK 255
#define CHAOS_INITIAL_MIN_RANK 16
#define CHAOS_RANK_TX_INCREMENT 0
#define CHAOS_RANK_IDLE_INCREMENT 1
#define RANK_ALPHA 115
#define RANK_SCALE 128

//FCF and addresses
#define CHAOS_SECURITY_LEVEL    LLSEC802154_CONF_SECURITY_LEVEL

#if CHAOS_HW_SECURITY
  #if CHAOS_USE_DST_ID || !CHAOS_USE_SRC_ID
    #error "Config not supported: Must use SRC_ID but can't use DST_ID when HW_SECURITY (because we use the dst_node_id place holder to store round_number)"
  #endif
  #define CHAOS_SECURITY_CONTROL  (LLSEC802154_CONF_SECURITY_LEVEL /* security level */ | FRAME802154_IMPLICIT_KEY /* Key identifier mode */ | (0) /* reserved */)
  #define CHAOS_SECURITY_KEY ("123456789abcdef") //16bytes
#endif /* CHAOS_HW_SECURITY */
#if CHAOS_USE_SRC_ID
  #define CHAOS_CLEARTEXT_HEADER_SIZE 9 /* fcf, slotnumber, dst_pan_id, round_number, src_node_id */
#else
  #define CHAOS_CLEARTEXT_HEADER_SIZE 7 /* fcf, slotnumber, dst_pan_id, round_number */
#endif
#define PRE_HW_SECURITY_HEADER_LEN CHAOS_CLEARTEXT_HEADER_SIZE

/**
 * \defgroup chaos-test-settings Application settings
 * @{
 */

#include "leds.h"
#if (COOJA)
#define LEDS_ON(X) leds_on((X))
#define LEDS_OFF(X) leds_off((X))
#define LEDS_BLINK() leds_blink()
#define LEDS_TOGGLE(X) leds_toggle((X))
#define RX_LEDS_DELAY (DCO_TO_VHT(130/2))
#define TX_LEDS_DELAY (DCO_TO_VHT(130/2))
#else
#define LEDS_ON(X)
#define LEDS_OFF(X)
#define LEDS_BLINK()
#define LEDS_TOGGLE(X)
#define RX_LEDS_DELAY (0)
#define TX_LEDS_DELAY (0)
#endif

#ifndef CHAOS_DCO_RADIO_CALIBRATION
#define CHAOS_DCO_RADIO_CALIBRATION 0
#endif /* CHAOS_DCO_RADIO_CALIBRATION */

#if COOJA
#undef CHAOS_DCO_RADIO_CALIBRATION
#define CHAOS_DCO_RADIO_CALIBRATION 0
#endif /* !COOJA */

/**
 * \brief Initiator ID should be defined!
 */
#if !defined(INITIATOR_NODE) && CONTIKI_WITH_CHAOS
#error "INITIATOR_NODE_ID not defined !!"
//#define INITIATOR_NODE_ID       1
#endif

typedef enum {
	CHAOS_INIT, CHAOS_RX, CHAOS_TX, CHAOS_OFF, CHAOS_RX_SYNC, CHAOS_TX_SYNC
} chaos_state_t;

#define CHAOS_STATE_TO_STRING(S) \
  ( (S==CHAOS_INIT) ? "INI " : \
    (S==CHAOS_TX) ? "cTX " : \
    (S==CHAOS_RX) ? "cRX " : \
    (S==CHAOS_RX_SYNC) ? "sRX " : \
    (S==CHAOS_TX_SYNC) ? "sTX " : \
    (S==CHAOS_OFF) ? "OFF " : "UNK ")

typedef enum {
  CHAOS_TXRX_UNKOWN=0, CHAOS_TXRX_OK, CHAOS_RX_NO_SFD, CHAOS_RX_HEADER_ERROR, CHAOS_RX_CRC_ERROR, CHAOS_RX_MIC_ERROR, CHAOS_TXRX_ERROR, CHAOS_RX_TIMEOUT /* HACK! timeout shall always be the last one */
} chaos_rx_status_t;

#define CHAOS_RX_STATE_TO_STRING(S) \
  ( (S==CHAOS_TXRX_UNKOWN) ? "rNA " : \
    (S==CHAOS_TXRX_OK) ? "rOK " : \
    (S==CHAOS_RX_NO_SFD) ? "SFD " : \
    (S==CHAOS_RX_HEADER_ERROR) ? "HDR " : \
    (S==CHAOS_RX_CRC_ERROR) ? "CRC " : \
    (S==CHAOS_RX_MIC_ERROR) ? "MIC " : \
    (S==CHAOS_TXRX_ERROR) ? "ERR " : \
    (S>=CHAOS_RX_TIMEOUT) ? "rTO " : "rNK " )

#define CHAOS_RX_STATE_GET_TIMEOUT(S) \
  ( (S>=CHAOS_RX_TIMEOUT) ? (int)S-CHAOS_RX_TIMEOUT : -1 )

/**
 * Ratio between the frequencies of the DCO and the low-frequency clocks
 */

#define CHAOS_FCF_0             ( \
        (FRAME802154_DATAFRAME & 7) | \
        ((CHAOS_SECURITY_LEVEL & 1) << 3) | \
        ((0 & 1) << 4) | /* p->fcf.frame_pending */ \
        ((0 & 1) << 5) | /* p->fcf.ack_required */ \
        ((1 & 1) << 6)  /* p->fcf.panid_compression */ )

#if CHAOS_USE_DST_ID || CHAOS_HW_SECURITY
#define DST_ADDR_MODE FRAME802154_SHORTADDRMODE
#else
#define DST_ADDR_MODE FRAME802154_NOADDR
#endif

#if CHAOS_USE_SRC_ID
#define SRC_ADDR_MODE FRAME802154_SHORTADDRMODE
#else
#define SRC_ADDR_MODE FRAME802154_NOADDR
#endif

#define CHAOS_FCF_1       ( \
        ((DST_ADDR_MODE & 3) << 2 /* p->fcf.dest_addr_mode XXX change to FRAME802154_SHORTADDRMODE?? */) |  \
        ((FRAME802154_IEEE802154_2006 & 3) << 4 /* p->fcf.frame_version */) | \
        ((SRC_ADDR_MODE & 3) << 6 /* p->fcf.src_addr_mode */) )

#define CHAOS_FCF_0_NO_SECURITY  (CHAOS_FCF_0 & 0xf7)
#define CHAOS_FCF (CHAOS_FCF_0 | (uint16_t)CHAOS_FCF_1 << 8)
#define CHAOS_FCF_NO_SECURITY ((CHAOS_FCF_0 & 0xf7) | (uint16_t)CHAOS_FCF_1 << 8)
#define CHAOS_PANID 0xfeba

#define FOOTER_LEN                        2
#define FOOTER1_CRC_OK                    0x80
#define FOOTER1_CORRELATION               0x7f
#define CHAOS_PAYLOAD_LEN_TO_PACKET_LENGTH(payload_len) 	(sizeof(chaos_header_t) - 1 + LLSEC802154_MIC_LENGTH + FOOTER_LEN + (payload_length))
#define CHAOS_PACKET_RADIO_LENGTH(len)   ((len) + 1)
#define CHAOS_PAYLOAD_LENGTH(packet) 	  	((packet)->length - (sizeof(chaos_header_t) - 1 + LLSEC802154_MIC_LENGTH + FOOTER_LEN) )
#define CHAOS_RSSI_FIELD(packet)          ((packet)[((chaos_header_t*)(packet))->length - 1])
#define CHAOS_CRC_FIELD(packet)           ((packet)[((chaos_header_t*)(packet))->length])

//TODO: check if MIC field position is correct
#define CHAOS_MIC_FIELD(packet)           ((packet)[((chaos_header_t*)(packet))->length - FOOTER_LEN])
#define CHAOS_IS_SECURITY_ENABLED_FRAME(packet) ((((packet)->chaos_fcf_0) & (1 << 3)) ? 1 : 0)

#define RADIO_MAX_PACKET_LEN 				      (128) /* 127 + len-field(1) */
#define CHAOS_MAX_PAYLOAD_LEN             (RADIO_MAX_PACKET_LEN - (sizeof(chaos_header_t) + LLSEC802154_MIC_LENGTH + FOOTER_LEN))

enum {
	CHAOS_INITIATOR = 1, CHAOS_RECEIVER = 0
};

//
//#ifndef FF_N_TX
//#define FF_N_TX 			5
//#endif


/**
 * \brief Check if the nodeId matches the one of the initiator.
 */
#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC && (TESTBED == rennes) //|| TESTBED == euratech)
#define INITIATOR_NODE_ID (mapping[(uint16_t)INITIATOR_NODE-1])
#elif NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC && (TESTBED == euratech)
#define INITIATOR_NODE_ID (mapping[(uint16_t)INITIATOR_NODE-1])
#else
#define INITIATOR_NODE_ID ((uint16_t)INITIATOR_NODE)
#endif

#define IS_INITIATOR()              (node_id == INITIATOR_NODE_ID)

/* For target Sky and Z1 */
#define RSSI_CORRECTION_CONSTANT (-45)

/* Do the math in 32bits to save precision.
 * Round to nearest integer rather than truncate. */

#define DCOTICKS_TO_US(T)   (((T)>=0) ? \
                               ((((int32_t)(T)*1000L)+(F_CPU/2)) / (F_CPU/1000)) : \
                               ((((int32_t)(T)*1000L)-(F_CPU/2)) / (F_CPU/1000)))

#define US_TO_DCOTICKS(US)   (((US)>=0) ? \
                               ((((int32_t)(US)*F_CPU)+500000) / 1000000L) : \
                               ((((int32_t)(US)*F_CPU)-500000) / 1000000L))

/* TODO check the following */
#define RTIMER_RADIO_RATIO_4 (((uint32_t)(RTIMER_SECOND/4)<<2UL)/(62500UL/4)) /* multiplying by 8 to save one decimal digit*/
#define RADIO_TO_RTIMER(X)  ((rtimer_clock_t)(((int32_t)(X) * RTIMER_RADIO_RATIO_4 + 2) >> 2UL)) /* +2 before shifting by 2 for rounding */
#define DCO_RADIO_RATIO_8 (((F_CPU/4)<<3UL)/(62500UL/4)) /* multiplying by 8 to save one decimal digit*/
#define RADIO_TO_DCO(X)  ((rtimer_clock_t)((int32_t)((int32_t)(X) * DCO_RADIO_RATIO_8 + 4) >> 3UL)) /* +4 for rounding */
#define RADIO_BYTES_TO_DCO_PERIOD(X)  ((rtimer_clock_t)(((uint32_t)(X) * DCO_RADIO_RATIO_8 + 4) >> 2UL)) /* +4 for rounding */
#define VHT_RADIO_RATIO_2 (((((uint32_t)(RT_VHT_PHI/4)))<<2UL)/(62500U/4)) /* multiplying by 4 to save one decimal digit*/
#define RADIO_TO_VHT(X)  ((vht_clock_t)(((X) * VHT_RADIO_RATIO_2 + 2) >> 2UL)) /* +2 for rounding */

/* Calculate packet tx/rc duration based on sent packet len assuming 802.15.4 250kbps data rate
 * PHY packet length: payload_len including CHECKSUM_LEN(2) + PHY_LEN_FIELD(1) */
#define CHAOS_PACKET_DURATION(len) (RADIO_TO_RTIMER((len) * 2))
#define CHAOS_PACKET_DURATION_DCO(len) (RADIO_BYTES_TO_DCO_PERIOD((len)))
#define CHAOS_PACKET_DURATION_VHT(len) (RADIO_TO_VHT((len) * 2))

/* radio speed related */
/* ~327us+129preample */
#if COOJA /* dividing by 2 since the numbers were calculated on 4MHz and we are running on 2MHz in Cooja*/
//#define CHAOS_TX_DELAY_VHT_OFF (0xd9500uL)
//#define CHAOS_TX_DELAY_VHT (0xd9500uL)
//#define CHAOS_RX_DELAY_VHT (0xee200uL)
#define CHAOS_TX_DELAY_VHT (DCO_TO_VHT(1565) +TX_LEDS_DELAY) //+4: for SET_PIN_ADC0
#define CHAOS_RX_DELAY_VHT (DCO_TO_VHT(1037)+RX_LEDS_DELAY)

//XXX rx_delay needs calibration!
#define PREP_RX_VHT DCO_TO_VHT(500/2)
#define PREP_TX_VHT DCO_TO_VHT(300/2)
#define VHT_MIN_DELAY DCO_TO_VHT(20)
#elif CONTIKI_TARGET_SKY
#define CHAOS_TX_DELAY_VHT (DCO_TO_VHT(1539+3)) //+4: for SET_PIN_ADC0)
#define CHAOS_RX_DELAY_VHT (DCO_TO_VHT(1022))
//XXX rx_delay needs calibration!
#define PREP_RX_VHT DCO_TO_VHT(500)
#define PREP_TX_VHT DCO_TO_VHT(300)
#define VHT_MIN_DELAY DCO_TO_VHT(40)
#else
#define CHAOS_TX_DELAY_VHT (DCO_TO_VHT(1539+3))
#define CHAOS_RX_DELAY_VHT (DCO_TO_VHT(1022))
//XXX rx_delay needs calibration!
#define PREP_RX_VHT DCO_TO_VHT(500)
#define PREP_TX_VHT DCO_TO_VHT(300)
#define VHT_MIN_DELAY DCO_TO_VHT(40)
#endif


/* RX SFD time has an offset of 3uS +- 100ns == 12.58 dco ticks with 4MiHz clk.
 * or 2uS +- 100ns == 8.4 according to manual??
 * We could use SFD_OFFSET*2 to save precision */
#if COOJA
#define RX_SFD_OFFSET              (125/2)
#else
#define RX_SFD_OFFSET              (-8)
#endif
  /* ~50us delay + 129preample + ?? */
//#define DELAY_RX 6         /* between GO signal and start listening */
//#define DELAY_RX_SHORT 2         /* between GO signal and start listening */

/* Convert rtimer ticks to clock and vice versa */
#define CHAOS_CLOCK_TO_TICKS(c) (((c)*RTIMER_SECOND)/CLOCK_SECOND)

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif /* MAX */

#undef ABS
#define ABS(x) ((int16_t)(x) < 0 ? -(x) : (x))
#define ABS_VHT(x) ((int32_t)(x) < 0 ? -(x) : (x))

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))) {};   \
  } while(0)

#define BUSYWAIT_RTIMER(max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))) {};   \
  } while(0)

#define BUSYWAIT_DCO(max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = DCO_NOW();                                                  \
    while(DCO_LT(DCO_NOW(), t0 + (max_time))) {};   \
  } while(0)

typedef chaos_state_t (*process_callback_t)(uint16_t round_count, uint16_t slot_count, chaos_state_t current_state, int rx_valid, size_t payload_length, uint8_t* rx_payload, uint8_t* tx_payload, uint8_t** app_flags);

uint16_t chaos_round(const uint16_t round_number, const uint8_t app_id, const uint8_t* const payload,
    const uint8_t payload_length, const rtimer_clock_t slot_length_dco, const uint16_t max_slots, const uint8_t app_flags_len,
    process_callback_t process);

uint8_t chaos_associate(rtimer_clock_t* t_sfd_actual_rtimer_ptr, uint16_t *round_number_ptr, uint16_t* slot_number_ptr, uint8_t* app_id_ptr);

int get_round_synced();

vht_clock_t get_round_offset_to_radio_on();

vht_clock_t get_round_rtimer();

vht_clock_t get_round_start();

rtimer_clock_t get_next_round_begin();

uint8_t get_next_round_id();

uint16_t get_sync_round();

uint8_t* chaos_get_nonce_pointer();

uint8_t* chaos_get_security_key_pointer();

const uint8_t * chaos_get_extended_address(uint16_t addr);

//void
//chaos_update_nonce(
//    uint8_t *nonce,
//    uint32_t security_frame_counter);

//void
//chaos_make_nonce(
//    uint8_t *nonce,
//    const uint8_t *extended_source_address,
//    uint32_t security_frame_counter);
void
chaos_make_const_nonce(uint8_t *nonce);

const uint32_t * chaos_get_dummy_packet_32t();
#define MAX_SLOTS_IN_ROUND 512
extern uint8_t chaos_slot_log[MAX_SLOTS_IN_ROUND];
#define CHAOS_SLOT_STATS_SIZE (16)
extern uint16_t chaos_slot_stats[CHAOS_SLOT_STATS_SIZE];
enum {TX_PREPARE=0, RX_PREPARE, TX, RX, TX_POST, RX_POST, JOIN_PROCESSING, APP_PROCESSING, SLOT_END_PROCCESSING, SLOT_TIME_ALL, SLOTNUMBER, SLOT_TIMING_SIZE};
extern rtimer_clock_t chaos_slot_timing_log_max[SLOT_TIMING_SIZE];
extern rtimer_clock_t chaos_slot_timing_log_min[SLOT_TIMING_SIZE];
extern uint32_t chaos_slot_timing_rx_sum;
extern uint32_t chaos_slot_timing_tx_sum;

#endif /* CHAOS_H_ */
