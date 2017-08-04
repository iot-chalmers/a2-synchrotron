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

#ifndef _NODE_H
#define	_NODE_H

#include "contiki.h"
#include "chaos.h"
#include "testbed.h"
#include "chaos-config.h"

void init_node_index();

#ifndef MAX_NODE_COUNT
#define MAX_NODE_COUNT 254
#endif

#if NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC
extern volatile uint8_t chaos_node_count;  //only valid on initiator
extern volatile uint8_t chaos_node_index;
extern volatile uint8_t chaos_has_node_index;

#define CHAOS_NODES chaos_node_count
#else /* NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC */

#ifndef CHAOS_NODES
#define CHAOS_NODES 1
#endif

extern uint8_t chaos_node_index;
extern uint8_t chaos_has_node_index;
extern const uint8_t chaos_node_count;

#endif  /* NETSTACK_CONF_WITH_CHAOS_NODE_DYNAMIC */
#endif	/* _NODE_H */
