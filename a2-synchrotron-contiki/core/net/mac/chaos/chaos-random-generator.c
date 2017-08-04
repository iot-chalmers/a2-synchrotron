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
 *         A2-Synchrotron - random number generator.
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 *
 */
#include "contiki.h"
#include "chaos-random-generator.h"
#include "node-id.h"
#include "chaos-config.h"

#if CONTIKI_TARGET_SKY || CONTIKI_TARGET_WSN430
unsigned int msp430_rand(void);
#define HW_RND() msp430_rand()
#else
#warning "define a HW random generator function for proper random seeds"
#define HW_RND() DCO_NOW()
#endif

/* TODO: make a lookup table that is updated between rounds */
#if CHAOS_USE_MSPGCC_RAND
#include "lib/random.h"
uint32_t
chaos_random_generator() {
  return random_rand();
}

void
chaos_random_generator_set_seed(uint32_t seed) {
  random_init((uint16_t)(seed >> 16UL));
}
#else /* CHAOS_USE_MSPGCC_RAND */

/* 32-bits Random number generator U[0,1): lfsr113
 * Author: Pierre L'Ecuyer,
 * Source: http://www.iro.umontreal.ca/~lecuyer/myftp/papers/tausme2.ps
 * https://github.com/cmcqueen/simplerandom/blob/master/c/lecuyer/lfsr113.c
 *
 * PS: This implementation is faster than mspgcc random_rand()
 * --> 388 dco ticks for this vs. 412 (in cooja);
 **/

/**** VERY IMPORTANT **** :
* The initial seeds z1, z2, z3, z4  MUST be larger than
* 1, 7, 15, and 127 respectively.
*/
#define SEED 12345UL
static uint32_t z1 = SEED, z2 = SEED, z3 = SEED, z4 = SEED;

void
chaos_random_generator_set_seed(uint32_t seed) {
  if(seed < 127) {
    seed += SEED + node_id * node_id;
  }
  z1 = z2 = z3 = z4 = seed;
}

uint32_t
chaos_random_generator_produce() {
  uint32_t b, z;
  b  = ((z1 << 6UL) ^ z1) >> 13UL;
  z1 = ((z1 & 4294967294UL) << 18UL) ^ b;
  b  = ((z2 << 2UL) ^ z2) >> 27UL;
  z2 = ((z2 & 4294967288UL) << 2UL) ^ b;
  b  = ((z3 << 13UL) ^ z3) >> 21UL;
  z3 = ((z3 & 4294967280UL) << 7) ^ b;
  b  = ((z4 << 3UL) ^ z4) >> 12UL;
  z4 = ((z4 & 4294967168UL) << 13UL) ^ b;
  z = (z1 ^ z2 ^ z3 ^ z4);
  return z;
}
#endif /* CHAOS_USE_MSPGCC_RAND */

#define CHAOS_RND_TABLE_SIZE 512
static uint8_t random_idx = 0;
static uint32_t random_table[CHAOS_RND_TABLE_SIZE] = {0UL};

void
chaos_random_generator_update_table()
{
  random_idx = 0;
  int i;
  for(i=0; i<CHAOS_RND_TABLE_SIZE; i++) {
    random_table[i] = chaos_random_generator_produce();
  }
}

void
chaos_random_generator_init(void)
{
  uint16_t random_seed_l = HW_RND() + ((node_id << 8U) | (node_id & 0xffU));
  uint32_t random_seed = random_seed_l + ((uint32_t)random_seed_l << 16UL);
  chaos_random_generator_set_seed(random_seed);
  chaos_random_generator_update_table();
}

uint32_t
chaos_random_generator_fast() {
  return random_table[random_idx++ % CHAOS_RND_TABLE_SIZE];
}
