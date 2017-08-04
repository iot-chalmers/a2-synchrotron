/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"

#include <stdio.h> /* For printf() */
#include "net/netstack.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "node-id.h"

#define PERIOD (RTIMER_TO_VHT(RTIMER_SECOND)/1000UL)
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{

  PROCESS_BEGIN();
  NETSTACK_MAC.off(0);

  static uint16_t c = 0;
//  static uint32_t v, v0 = -1, t, t0;
//  static uint16_t l, r, h, d;
//  t0=0xf0000000UL+PERIOD;
//  while(1) {
//    leds_off(LEDS_GREEN);
//    do{
//      t = VHT_NOW();
//    } while(VHT_LT(t, t0));
//    leds_on(LEDS_GREEN);
//    watchdog_periodic();
//    t0 += PERIOD;
//
//    v = rtimer_arch_now_vht();
//    if( v < v0 ){
//  //      c++;
//  //      r = rtimer_arch_now();
//  //      d = rtimer_arch_now_dco();
//  //      printf("%u: v0 %lu v %lu l %u r %u h %u d %u\n",c, v0, v, l, r, h, d);
//      leds_blink();
//    }
//    v0 = v;
//  }

  while(1){
//    watchdog_periodic();
    printf("Hello %u : %u\n", node_id, c++);
    leds_blink();
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
