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
 *         Flocklab GPIO functions.
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *         Adopted from:  Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#ifndef CORE_NET_MAC_CHAOS_FLOCKLAB_GPIO_H_
#define CORE_NET_MAC_CHAOS_FLOCKLAB_GPIO_H_

#include "net/mac/chaos/node/testbed.h"

#define FLOCKLAB_DEBUG 0

#if (TESTBED == flocklab) && FLOCKLAB_DEBUG
#include <legacymsp430.h>

/* flocklab GPIO from FE glossy */
#define SET_PIN(a,b)          do { P##a##OUT |=  BV(b); } while (0)
#define UNSET_PIN(a,b)        do { P##a##OUT &= ~BV(b); } while (0)
#define TOGGLE_PIN(a,b)       do { P##a##OUT ^=  BV(b); } while (0)
#define INIT_PIN_IN(a,b)      do { P##a##SEL &= ~BV(b); P##a##DIR &= ~BV(b); } while (0)
#define INIT_PIN_OUT(a,b)     do { P##a##SEL &= ~BV(b); P##a##DIR |=  BV(b); } while (0)
#define PIN_IS_SET(a,b)       (    P##a##IN  &   BV(b))

#else /* FLOCKLAB_DEBUG */

#define SET_PIN(a,b)          do { } while (0)
#define UNSET_PIN(a,b)        do { } while (0)
#define TOGGLE_PIN(a,b)       do { } while (0)
#define INIT_PIN_IN(a,b)      do { } while (0)
#define INIT_PIN_OUT(a,b)     do { } while (0)
#define PIN_IS_SET(a,b)       ( 0 )

#endif /* FLOCKLAB_DEBUG */

// UserINT (P2.7)
#define SET_PIN_USERINT      SET_PIN(2,7)
#define UNSET_PIN_USERINT    UNSET_PIN(2,7)
#define TOGGLE_PIN_USERINT   TOGGLE_PIN(2,7)
#define INIT_PIN_USERINT_IN  INIT_PIN_IN(2,7)
#define INIT_PIN_USERINT_OUT INIT_PIN_OUT(2,7)
#define PIN_USERINT_IS_SET   PIN_IS_SET(2,7)

// GIO2 (P2.3)
#define SET_PIN_GIO2         SET_PIN(2,3)
#define UNSET_PIN_GIO2       UNSET_PIN(2,3)
#define TOGGLE_PIN_GIO2      TOGGLE_PIN(2,3)
#define INIT_PIN_GIO2_IN     INIT_PIN_IN(2,3)
#define INIT_PIN_GIO2_OUT    INIT_PIN_OUT(2,3)
#define PIN_GIO2_IS_SET      PIN_IS_SET(2,3)

// ADC0 (P6.0)
// used to signal TX go
#define SET_PIN_ADC0         SET_PIN(6,0)
#define UNSET_PIN_ADC0       UNSET_PIN(6,0)
#define TOGGLE_PIN_ADC0      TOGGLE_PIN(6,0)
#define INIT_PIN_ADC0_IN     INIT_PIN_IN(6,0)
#define INIT_PIN_ADC0_OUT    INIT_PIN_OUT(6,0)
#define PIN_ADC0_IS_SET      PIN_IS_SET(6,0)

// ADC1 (P6.1)
// used to signal RX SFD
#define SET_PIN_ADC1         SET_PIN(6,1)
#define UNSET_PIN_ADC1       UNSET_PIN(6,1)
#define TOGGLE_PIN_ADC1      TOGGLE_PIN(6,1)
#define INIT_PIN_ADC1_IN     INIT_PIN_IN(6,1)
#define INIT_PIN_ADC1_OUT    INIT_PIN_OUT(6,1)
#define PIN_ADC1_IS_SET      PIN_IS_SET(6,1)

// ADC2 (P6.2) -> LED3
// used to signal RX Go
#define SET_PIN_ADC2         SET_PIN(6,2)
#define UNSET_PIN_ADC2       UNSET_PIN(6,2)
#define TOGGLE_PIN_ADC2      TOGGLE_PIN(6,2)
#define INIT_PIN_ADC2_IN     INIT_PIN_IN(6,2)
#define INIT_PIN_ADC2_OUT    INIT_PIN_OUT(6,2)
#define PIN_ADC2_IS_SET      PIN_IS_SET(6,2)

// ADC6 (P6.6) -> LED2
// used to signal associate
#define SET_PIN_ADC6         SET_PIN(6,6)
#define UNSET_PIN_ADC6       UNSET_PIN(6,6)
#define TOGGLE_PIN_ADC6      TOGGLE_PIN(6,6)
#define INIT_PIN_ADC6_IN     INIT_PIN_IN(6,6)
#define INIT_PIN_ADC6_OUT    INIT_PIN_OUT(6,6)
#define PIN_ADC6_IS_SET      PIN_IS_SET(6,6)

// ADC7 (P6.7) -> LED1
// used to signal round begin
#define SET_PIN_ADC7         SET_PIN(6,7)
#define UNSET_PIN_ADC7       UNSET_PIN(6,7)
#define TOGGLE_PIN_ADC7      TOGGLE_PIN(6,7)
#define INIT_PIN_ADC7_IN     INIT_PIN_IN(6,7)
#define INIT_PIN_ADC7_OUT    INIT_PIN_OUT(6,7)
#define PIN_ADC7_IS_SET      PIN_IS_SET(6,7)

#endif /* CORE_NET_MAC_CHAOS_FLOCKLAB_GPIO_H_ */
