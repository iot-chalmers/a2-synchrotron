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
 *         Testbed configurations.
 * \author
 *         Olaf Landsiedel <olafl@chalmers.se>
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Adopted from:  Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#ifndef TESTBED_H_
#define TESTBED_H_

#define cooja 0 //any undefined testbed name will map to cooja
#define indriya 1
#define twist 2
#define flocklab 3
#define motes 4
#define flocklabewsn 5
#define ewsn 6
#define ewsnmotes 7
#define rennes 8
#define rennesmotes 9
#define euratech 10
#define iotlab 11

#ifdef TESTBED

  extern const uint16_t mapping[];


#if 1 //!NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
  #if TESTBED == indriya
  //#warning "compiling for indriya"
  //nodes 86-114 are arduino nodes now
  //nodes 23, 29, 49, 61, 106, 113, 114, 125 are broken
  //nodes 9,11,56,57,62,134,80-85 are not reporting
  //using 92 nodes only::
  #define TESTBED_MAPPING { \
      1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85, \
      115,116,117,118,119,120,121,122,123,124,125,126,127,128,130,129,132,131,133,134,135,136,137,138,139 \
  }

  #elif TESTBED == twist
  //#warning "compiling for twist"
#define TESTBED_MAPPING { \
  10, 11, 12, 13, 15, 79, 80, 81, 82, 85, 86, 87, 88, 89, 90, 91, 92, 94, 95, 96, \
  97, 99, 100, 101, 102, 103, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, \
  147, 148, 149, 150, 151, 152, 154, 188, 191, 192, 193, \
  195, 196, 197, 199, 200, 202, 213, 214, 215, 216, 218, 223, 228, 229, \
  231, 240, 241, 249, 250, 251, 262, 272 \
  }

  #elif TESTBED == flocklab
  //#warning "compiling for flocklab"
#define TESTBED_MAPPING {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30}

  #elif TESTBED == rennes
#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC //list all nodes including broken
#define TESTBED_MAPPING {0xcaf5,  0xca73,  0xc88c,  0xb1c3,  0xb704,  0xc351,  0xbc98,  0xb344,  0xc59b,  0xb429,  0xc09a,  0xbd6e,  0xc0e9,  0xca0a,  0xb583,  0xb8d6,  0xc579,  0xbeeb,  0xc5e7,  0xc2bb,  0x1c15,  0xcb97,  0xb4bc,  0xb502,  0xc89a,  0xc32c,  0xb792,  0xc3a8,  0xc7a8,  0xba43,  0xc7cf,  0xb920,  0xbd97,  0xbafb,  0xb81d,  0xb525,  0xc032,  0xc475,  0xb192,  0xbcd2,  0xc632,  0xc7c8,  0xb1a3,  0x1f52,  0xcf06,  0xb1b6,  0xc713,  0xbe69,  0xc41c,  0xb1ce,  0xbf0e,  0xb7a4,  0xb712,  0xcf01,  0xb4a4,  0xc118,  0xca5a,  0xb469,  0xb3fc,  0xc3bd,  0xc73e,  0xbc5a,  0xc872,  0xbe7c,  0xc05f,  0xb234,  0xc345,  0xc33c,  0x1c06,  0xb0c1,  0xc760,  0xb775,  0xc5b4,  0xc9ce,  0xb964,  0xc688,  0xb228,  0xc171,  0xb5b2,  0xb9e9,  0xc715,  0xc9c5,  0xb627,  0xb78c,  0xafee,  0xc0a5,  0xc751,  0xcb5a,  0xcdaa,  0x1f54,  0x2033,  0xbca8,  0xc6ab,  0xcb1b,  0xc6cb,  0xc4e2,  0xc409,  0xb91d,  0xcc99,  0xcae2,  0xbca5,  0xbe2a,  0xc3da,  0xb2d4,  0xc929,  0xc24a,  0xb225,  0xc0b1,  0xb203,  0xc159,  0xb16d,  0xcb1c,  0xb370,  0xb973,  0xca15,  0xb1f9,  0xb84e,  0xce4a,  0x2036,  0xbe91,  0xb8f2,  0xb301,  0xcac1,  0xcb96,  0xbdae,  0xbb1f,  0xc989,  0xb5db,  0xb237,  0xbab9,  0xb83d,  0xb197,  0xc031,  0xb960,  0xb69e,  0xc156,  0xb5df,  0xb0f8,  0xcc73,  0xbf21,  0xb67b,  0x1f76,  0xc6e9,  0xbc0d,  0xc8d8,  0xb4a5,  0xc2ba,  0xcecd,  0x1ca1,  0xb056,  0xbb2b,  0xcee1,  0xb204,  0xc4ca,  0xc6ac,  0xb7fb,  0xb568,  0xcbc6,  0xbf52,  0xc059,  0xb529,  0xb91c,  0xbb46,  0xb077,  0xb6c0,  0xc28c,  0xc25a,  0xb9c1,  0xb128,  0x1f3e,  0xc121,  0xb55a,  0xcd54,  0xb478,  0xc668,  0xb751,  0x1f6b,  0xc268,  0xc487,  0xc427,  0xc5cf,  0xc091,  0xc980,  0xb08e,  0xbf0b,  0xcbd4,  0xc32a,  0xcaeb,  0xc34c,  0xcbfd,  0xbac7,  0xbe83,  0xbbbf,  0xc4a9,  0xb51f,  0xc03a,  0xc995,  0xb50e,  0xafbf,  0xb5a6,  0xbb26,  0xc930,  0xb5ea,  0xbc51,  0xc708,  0xbf6f,  0xbd10,  0xb176,  0xb6d3,  0xc406,  0xb7c3,  0xb2fc,  0xcd7d,  0xb299,  0xbce7,  0xcbe0,  0xbe33,  0xcf28,  0xc84f,  0xb95e,  0xb883,  0xb7e9,  0xc60b,  0xc93b,  0xbcca,  0xc615,  0xb7d1,  0xb490,  0xc479,  0xbc67, }
#else
  #define TESTBED_MAPPING {44991, 45038, 45142, 45175, 45198, 45249, 45304, 45421, 45430, 45458, 45463, 45475, 45494, 45507, 45518, 45561, 45571, 45572, 45605, 45608, 45620, 45623, 45721, 45780, 45820, 45825, 45936, 46076, 46121, 46185, 46200, 46245, 46268, 46338, 46367, 46377, 46426, 46440, 46467, 46502, 46514, 46559, 46570, 46631, 46715, 46750, 46784, 46803, 46852, 46866, 46929, 46965, 46988, 46994, 47043, 47057, 47081, 47133, 47182, 47235, 47318, 47346, 47389, 47392, 47456, 47460, 47475, 47553, 47593, 47683, 47801, 47815, 47867, 47910, 47915, 48063, 48141, 48209, 48218, 48231, 48280, 48293, 48296, 48330, 48338, 48359, 48400, 48494, 48535, 48558, 48682, 48691, 48764, 48771, 48875, 48907, 48910, 48929, 49007, 49201, 49202, 49241, 49247, 49306, 49317, 49385, 49432, 49441, 49494, 49497, 49521, 49754, 49768, 49804, 49850, 49851, 49962, 49964, 49980, 49989, 50001, 50088, 50109, 50182, 50185, 50204, 50215, 50297, 50311, 50345, 50378, 50587, 50612, 50639, 50663, 50699, 50709, 50792, 50824, 50859, 50860, 50891, 50921, 50952, 50963, 50965, 51006, 51025, 51040, 51112, 51151, 51279, 51340, 51354, 51416, 51497, 51504, 51515, 51584, 51593, 51605, 51653, 51662, 51722, 51733, 51802, 51827, 51938, 51947, 51996, 52118, 52119, 52166, 52180, 52192, 52221, 52339, 52377, 52564, 52650, 52810, 52941, 52961, 52998, 53032, 7174, 7189, 7329, 7998, 8018, 8020, 8043, 8243, 8246, 48046, 49519, 48017, 45640,}
#endif

  #elif TESTBED == euratech
#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC //list all nodes including broken
#define TESTBED_MAPPING {0xc321,  0xc23a,  0xcf17,  0xbd11,  0xb6bc,  0xccaa,  0xb154,  0xb27b,  0xbc2d,  0xbcb6,  0xc3c4,  0xc8a8,  0xc15b,  0xb034,  0xb471,  0xbbf0,  0xb986,  0xbe29,  0xc102,  0xb0b4,  0xafc7,  0xbc13,  0xb2bb,  0xc6df,  0xc4c8,  0xc26d,  0xb966,  0xcbf3,  0xb538,  0xc946,  0xc469,  0xcafe,  0xc6dc,  0xafad,  0xb54f,  0xc92c,  0xb468,  0xc2b6,  0xc69d,  0xc0e8,  0xcdec,  0xc721,  0xc7f0,  0xc8f3,  0xce61,  0xb2bf,  0xbd2d,  0xbe4d,  0xb843,  0xba95,  0xba46,  0xbe4b,  0xb95f,  0x1ce4,  0xc87e,  0xb2f6,  0xc4c2,  0xcbe9,  0xc5c8,  0xc3d6,  0xc088,  0xc11c,  0xc11e,  0xbf1c,  0xbc63,  0xc8ad,  0xc286,  0xbc48,  0xced3,  0xc689,  0xc0dc,  0xb01f,  0xbbea,  0xcb78,  0xcced,  0xcb08,  0xc973,  0xc241,  0xbabc,  0xb5f1,  0xc8c8,  0xc640,  0xca25,  0xb659,  0xb5cc,  0xb2c1,  0xcd82,  0xb325,  0xcc1b,  0xb42b,  0xc4ea,  0xb6a3,  0xb31a,  0xb381,  0xc342,  0xc0ab,  0xc83b,  0xb36b,  0xb4db,  0xb871,  0xbde6,  0xbcc4,  0xbef4,  0x1c9e,  0xaf78,  0xcae0,  0xc3e5,  0xb83b,  0xbf14,  0xcf2e,  0xbdf3,  0xb61a,  0xceb6,  0xc735,  0xbb0a,  0xc4d2,  0xbce9,  0xba9d,  0xce3a,  0x1bfa,  0xc834,  0x1f91,  0xbbb4,  0x1f9e,  0xc74d,  0xc6f5,  0xb8db,  0x1caf,  0xc92a,  0xbba2,  0xc20a,  0xcac8,  0xbdef,  0xb798,  0xc198,  0xc2d0,  0xbf4d,  0xc3b1,  0xb723,  0xb748,  0x1bfc,  0xb584,  0xb18d,  0xb5c9,  0xc506,  0xbcd3,  0xcddf,  0xbc46,  0xb984,  0xce38,  0xb2b8,  0xbc12,  0xcb0c,  0xc5f9,  0xb246,  0xc22f,  0xb0e0,  0xcf07,  0xb8c6,  0xb282,  0xc167,  0xc534,  0xb31c,  0xbc60,  0xcdba,  0xc703,  0xcd29,  0xc3a3,  0xcc1d,  0xc6ba,  0xb320,  0xcc61,  0xb1f4,  0xb46b,  0xc7aa,  0xc540,  0xba5e,  0xb3bd,  0xbc23,  0xc625,  0xbed4,  0x1f7a,  0xc1c8,  0xbf29,  0xc304,  0xc370,  0xc91c,  0xc035,  0xc5fe,  0xc913,  0xce49,  0xbbda,  0xaff2,  0xcae8,  0xbdd1,  0xb68e,  0xcb6a,  0xcc68,  0xc36b,  0xc416,  0xb863,  0xbd65,  0xc23c,  0xbd5c,  0xce56,  0xc447,  0xc33a,  0xce00,  0xc117,  0x1f43,  0xc2c8,  0xaf75,  0xb98e,  0xbd00,  0xbbe4,  0xc013,  0x1f71,  0xc2b8,  0xc839,  0xb2ee,  0xbd79,  0xb73b,  0xcd89, }
#else
  #define TESTBED_MAPPING  {44920, 44973, 44999, 45087, 45108, 45280, 45396, 45453, 45556, 45638, 45691, 45698, 45752, 45755, 45759, 45761, 45814, 45850, 45852, 45861, 45931, 45953, 46013, 46123, 46184, 46187, 46193, 46299, 46392, 46468, 46537, 46540, 46577, 46618, 46681, 46755, 46883, 46920, 47000, 47163, 47171, 47217, 47302, 47323, 47455, 47462, 47492, 47686, 47765, 47804, 47882, 48034, 48146, 48147, 48163, 48173, 48198, 48200, 48224, 48227, 48310, 48324, 48339, 48361, 48401, 48429, 48614, 48627, 48681, 48715, 48717, 48852, 48884, 48916, 48924, 48937, 48973, 49205, 49288, 49323, 49372, 49384, 49410, 49436, 49438, 49511, 49560, 49608, 49674, 49711, 49722, 49729, 49773, 49846, 49872, 49924, 49953, 49986, 50032, 50083, 50097, 50116, 50134, 50149, 50281, 50370, 50376, 50386, 50410, 50484, 50496, 50632, 50681, 50725, 50752, 50825, 50845, 50874, 50908, 50911, 50933, 50947, 50977, 50997, 51021, 51114, 51252, 51259, 51326, 51368, 51373, 51400, 51443, 51498, 51500, 51526, 51571, 51912, 51936, 51966, 51976, 51980, 52088, 52211, 52251, 52253, 52321, 52394, 52461, 52610, 52666, 52716, 52792, 52794, 52918, 52947, 52999, 53015, 53038, 7162, 7164, 7326, 7343, 7396, 8058, 8081, 8094, 51749, 52252, 47494, 46780, 47773, 47710,}
#endif

  #elif TESTBED == rennesmotes
 #define TESTBED_MAPPING { 1, 2, 3, 21, 15004, 23735, 30661, 36815, 37277, 39914,   0xca73,  0xc88c,  0xb1c3,  0xb704,  0xc351,  0xbc98,  0xb344,  0xc59b,  0xb429,  0xc09a,  0xbd6e,  0xc0e9,  0xca0a,  0xb583,  0xb8d6,  0xc579,  0xbeeb,  0xc5e7,  0xc2bb,  0x1c15,  0xcb97,  0xb4bc,  0xb502,  0xc89a,  0xc32c,  0xb792,  0xc3a8,  0xc7a8,  0xba43,  0xc7cf,  0xb920,  0xbd97,  0xbafb,  0xb81d,  0xb525,  0xc032,  0xc475,  0xb192,  0xbcd2,  0xc632,  0xc7c8,  0xb1a3,  0x1f52,  0xcf06,  0xb1b6,  0xc713,  0xbe69,  0xc41c,  0xb1ce,  0xbf0e,  0xb7a4,  0xb712,  0xcf01,  0xb4a4,  0xc118,  0xca5a,  0xb469,  0xb3fc,  0xc3bd,  0xc73e,  0xbc5a,  0xc872,  0xbe7c,  0xc05f,  0xb234,  0xc345,  0xc33c,  0x1c06,  0xb0c1,  0xc760,  0xb775,  0xc5b4,  0xc9ce,  0xb964,  0xc688,  0xb228,  0xc171,  0xb5b2,  0xb9e9,  0xc715,  0xc9c5,  0xb627,  0xb78c,  0xafee,  0xc0a5,  0xc751,  0xcb5a,  0xcdaa,  0x1f54,  0x2033,  0xbca8,  0xc6ab,  0xcb1b,  0xc6cb,  0xc4e2,  0xc409,  0xb91d,  0xcc99,  0xcae2,  0xbca5,  0xbe2a,  0xc3da,  0xb2d4,  0xc929,  0xc24a,  0xb225,  0xc0b1,  0xb203,  0xc159,  0xb16d,  0xcb1c,  0xb370,  0xb973,  0xca15,  0xb1f9,  0xb84e,  0xce4a,  0x2036,  0xbe91,  0xb8f2,  0xb301,  0xcac1,  0xcb96,  0xbdae,  0xc989,  0xb237,  0xbab9,  0xb83d,  0xb197,  0xc031,  0xb960,  0xb69e,  0xc156,  0xb5df,  0xb0f8,  0xcc73,  0xbf21,  0xb67b,  0x1f76,  0xc6e9,  0xbc0d,  0xc8d8,  0xb4a5,  0xc2ba,  0xcecd,  0x1ca1,  0xb056,  0xbb2b,  0xcee1,  0xb204,  0xc4ca,  0xc6ac,  0xb7fb,  0xb568,  0xcbc6,  0xbf52,  0xc059,  0xb529,  0xb91c,  0xbb46,  0xb077,  0xb6c0,  0xc28c,  0xc25a,  0xb9c1,  0x1f3e,  0xc121,  0xb55a,  0xcd54,  0xb478,  0xc668,  0xb751,  0x1f6b,  0xc268,  0xc487,  0xc427,  0xc5cf,  0xc091,  0xc980,  0xb08e,  0xbf0b,  0xcbd4,  0xc32a,  0xcaeb,  0xc34c,  0xcbfd,  0xbac7,  0xbe83,  0xbbbf,  0xc4a9,  0xb51f,  0xc03a,  0xc995,  0xafbf,  0xb5a6,  0xbb26,  0xc930,  0xb5ea,  0xbc51,  0xc708,  0xbf6f,  0xbd10,  0xb176,  0xb6d3,  0xc406,  0xb7c3,  0xb2fc,  0xb299,  0xbce7,  0xcbe0,  0xbe33,  0xcf28,  0xc84f,  0xb883,  0xb7e9,  0xc60b,  0xc93b,  0xbcca,  0xc615,  0xb7d1,  0xb490,  0xc479,  0xbc67,}

  #elif TESTBED == iotlab
  #include "iotlabnodes.h"
#define TESTBED_MAPPING  { IOTLAB_MAPPING };

  #elif TESTBED == motes
  //#warning "compiling for motes"
#define TESTBED_MAPPING  {1, 2, 3, 21, 15004, 23735, 30661, 36815, 37277, 39914, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}

  #elif TESTBED == ewsn || TESTBED == flocklabewsn || TESTBED == ewsnmotes
  //#warning "compiling for ewsn -- without mapping"
  #define NO_TESTBED_ID_MAP 1
#define TESTBED_MAPPING {0}

 #elif TESTBED == cooja
  //#warning "compiling for cooja"
  #define NO_TESTBED_ID_MAP 1
#define TESTBED_MAPPING {0}

  #else
  #error "unknown testbed config"
  #endif

  #if !defined(CHAOS_NODES) && !NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
  #define CHAOS_NODES (sizeof(mapping)/sizeof(mapping[0]))
  #endif

#else

  //#warning "compiling for a testbed with join support " STR(TESTBED)

#endif /* !NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC */


#else
#define TESTBED_MAPPING {0}
#endif /* TESTBED */
#endif /* TESTBED_H_ */

