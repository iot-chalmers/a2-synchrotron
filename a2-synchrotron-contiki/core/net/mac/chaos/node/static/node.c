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
/*
 * \file
 *         Node manager
 * \author
 *         Beshr Al Nahas <beshr@chalmers.se>
 *         Olaf Landsiedel <olafl@chalmers.se>
 */
#include <stdio.h>

#include "contiki.h"
#include "node.h"
#include "node-id.h"
#include "net/mac/chaos/node/testbed.h"

uint8_t chaos_node_index = 0;
uint8_t chaos_has_node_index = 0;
const uint8_t chaos_node_count = (CHAOS_NODES);
const uint16_t mapping[] = (uint16_t[])TESTBED_MAPPING;

#if TESTBED > 0 && !NO_TESTBED_ID_MAP /* 0 is cooja or no testbed */
//execute mapping of node id to flag
void init_node_index(){
	unsigned int i;
	//lookup id
	for( i = 0; i < chaos_node_count; i++ ){
		if( node_id == mapping[i] ){
			chaos_node_index = i;
			chaos_has_node_index = 1;
		  printf("{boot} static testbed: node index: %i, node count: %i\n", chaos_node_index, chaos_node_count);
			return;
		}
	}
	chaos_node_index = 0xFF;
	chaos_has_node_index = 0;
	while(1) { /* freeze! */
	  printf("{boot} Mapping ERROR: node id %u could not resolved\n", node_id);
	}
}

#else

//execute mapping of node id to flag
void init_node_index(){
	//simple flags: node id minus one
  chaos_node_index = node_id - 1;
  chaos_has_node_index = 1;
  printf("{boot} static: node index: %i, node count: %i\n", chaos_node_index, chaos_node_count);
	return;
}

#endif
