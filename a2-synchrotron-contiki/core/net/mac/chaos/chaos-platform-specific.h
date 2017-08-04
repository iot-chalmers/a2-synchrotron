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
 *         A2-Synchrotron platform-specific.
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */

#ifndef CORE_NET_MAC_CHAOS_CHAOS_PLATFORM_SPECIFIC_H_
#define CORE_NET_MAC_CHAOS_CHAOS_PLATFORM_SPECIFIC_H_

#if CONTIKI_TARGET_SKY || CONTIKI_TARGET_WSN430


/* timing */
#define RTIMER_MIN_DELAY 5
void msp430_sync_dco_log(uint16_t round_number, uint8_t enable_logging);
#define chaos_platform_sync_dco_log(R, E) msp430_sync_dco_log(R, E)

/* Delay for i*4 + 1 cycles */
#define chaos_clock_delay(i) \
do{ \
  /*register uint16_t j = i;*/ /* loading into reg i: 1 cycle */       \
  asm volatile("add #-1,%[d]" : : [d] "r" (i)); /* 2 cycles */\
  asm volatile("jnz $-2"); /* 2 cycles */\
  /* uint16_t j = i / 4; \
  do {          \
    _NOP();     \
  } while(j--); */ \
} while(0)
/*---------------------------------------------------------------------------*/
#if COOJA
#define CHAOS_NOP_DELAY_CONST_COST (25-20)
#else
#define CHAOS_NOP_DELAY_CONST_COST (21-20)
#endif

#define CHAOS_NOP_DELAY_CONST (101)
#if ((F_CPU / RTIMER_ARCH_SECOND) < CHAOS_NOP_DELAY_CONST)
#error "Limit CHAOS_NOP_DELAY_CONST_JUMP to delay < 1 rtimer tick"
#endif
#define CHAOS_NOP_DELAY_MAX (412)
#if ( (F_CPU / RTIMER_ARCH_SECOND)*2 > CHAOS_NOP_DELAY_MAX + CHAOS_NOP_DELAY_CONST )
#error "Expand chaos_clock_delay_exact to fit 2 rtimer ticks at least"
#endif

#define chaos_clock_delay_exact(i) \
do{ \
  chaos_clock_delay((CHAOS_NOP_DELAY_CONST-1)/4); /* by design, we have at least 1 rtimer tick guard (== 128 @ 4MHz DCO / 32Khz RT ) */ \
  uint16_t jump = (CHAOS_NOP_DELAY_MAX + CHAOS_NOP_DELAY_CONST_COST - (uint16_t)(i - CHAOS_NOP_DELAY_CONST)) << 1; \
  if(jump <= (CHAOS_NOP_DELAY_MAX << 1)) {  /* max jump = (-511 to 512) for msp430 */ \
    asm volatile("add %[d], r0" : : [d] "m" (jump));  \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
    asm volatile("nop");             \
  } else {                           \
    LEDS_ON(LEDS_RED);             \
  }                                 \
} while(0)
/*---------------------------------------------------------------------------*/

/* radio */
#include "dev/cc2420/cc2420.h"
#define CHAOS_FIFOP_THRESHOLD 3
/*---------------------------------------------------------------------------*/
static inline void
chaos_radio_init(void)
{
  NETSTACK_RADIO_set_txpower(CHAOS_TX_POWER);
  NETSTACK_RADIO_set_channel(CHAOS_RF_CHANNEL);
  NETSTACK_RADIO_set_pan_addr(CHAOS_PANID, node_id, NULL);
  /* Disable radio interrupts so they do not interfere with RTIMER interrupts
   * Radio will be polled instead */
  NETSTACK_RADIO_set_interrupt_enable(0);
  NETSTACK_RADIO_address_decode(0);
  NETSTACK_RADIO_set_autoack(0);
  //NETSTACK_RADIO_set_fifop(CHAOS_FIFOP_THRESHOLD);
  NETSTACK_RADIO_flushrx();
  /* save start sfd and sync rtimer a and sfd timer b */
  NETSTACK_RADIO_sfd_sync(1, 0);
#if CHAOS_HW_SECURITY
  cc2420_set_security(CHAOS_SECURITY_LEVEL, CHAOS_CLEARTEXT_HEADER_SIZE);
  cc2420_set_key(chaos_get_security_key_pointer());
//  chaos_make_nonce(chaos_get_nonce_pointer(),
//      chaos_get_extended_address(node_id),
//      0);
  chaos_make_const_nonce(chaos_get_nonce_pointer());
  cc2420_set_nonce(chaos_get_nonce_pointer(), 1);
#endif /* CHAOS_HW_SECURITY */
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#elif CONTIKI_TARGET_CC2538DK

static inline void
chaos_radio_init(void)
{

}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#else

#error "Non-supported platform"

#endif /* CONTIKI_TARGET_SKY */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

#endif /* CORE_NET_MAC_CHAOS_CHAOS_PLATFORM_SPECIFIC_H_ */
