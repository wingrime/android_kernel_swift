/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __Q6_ADM_H__
#define __Q6_ADM_H__
#include <mach/qdsp6v2/q6afe.h>

/* multiple copp per stream. */
struct route_payload {
	unsigned int copp_ids[AFE_MAX_PORTS];
	unsigned short num_copps;
	unsigned int session_id;
};

int adm_open(int port, int path, int rate, int mode, int topology);

int adm_memory_map_regions(uint32_t *buf_add, uint32_t mempool_id,
				uint32_t *bufsz, uint32_t bufcnt);

int adm_memory_unmap_regions(uint32_t *buf_add, uint32_t *bufsz,
						uint32_t bufcnt);

int adm_close(int port);

int adm_matrix_map(int session_id, int path, int num_copps,
				unsigned int *port_id, int copp_id);

#ifdef CONFIG_MSM8X60_RTAC
int adm_get_copp_id(int port_id);
#endif

#endif /* __Q6_ADM_H__ */
