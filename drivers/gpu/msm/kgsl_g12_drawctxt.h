/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
 *
 */
#ifndef __GSL_DRAWCTXT_G12_H
#define __GSL_DRAWCTXT_G12_H

#include "kgsl_sharedmem.h"

struct kgsl_device;

#define GSL_HAL_NUMCMDBUFFERS       5
#define GSL_HAL_CMDBUFFERSIZE       ((1024 + 13) * sizeof(unsigned int))

#define ALIGN_IN_BYTES(dim, alignment) (((dim) + (alignment - 1)) & \
		~(alignment - 1))


#define NUMTEXUNITS             4
#define TEXUNITREGCOUNT         25
#define VG_REGCOUNT             0x39

#define PACKETSIZE_BEGIN        3
#define PACKETSIZE_G2DCOLOR     2
#define PACKETSIZE_TEXUNIT      (TEXUNITREGCOUNT * 2)
#define PACKETSIZE_REG          (VG_REGCOUNT * 2)
#define PACKETSIZE_STATE        (PACKETSIZE_TEXUNIT * NUMTEXUNITS + \
				 PACKETSIZE_REG + PACKETSIZE_BEGIN + \
				 PACKETSIZE_G2DCOLOR)
#define PACKETSIZE_STATESTREAM  (ALIGN_IN_BYTES((PACKETSIZE_STATE * \
				 sizeof(unsigned int)), 32) / \
				 sizeof(unsigned int))

struct kgsl_g12_hal_z1xxdrawctx_t {
	unsigned int id;
};

struct kgsl_g12_z1xx {
	unsigned int offs;
	unsigned int curr;
	unsigned int prevctx;

	unsigned int            *cmdbuf[GSL_HAL_NUMCMDBUFFERS];
	struct kgsl_memdesc      cmdbufdesc[GSL_HAL_NUMCMDBUFFERS];

	unsigned int numcontext;
};

extern struct kgsl_g12_z1xx g_z1xx;

int
kgsl_g12_drawctxt_create(struct kgsl_device *device,
			unsigned int type,
			unsigned int *drawctxt_id,
			unsigned int flags);

int
kgsl_g12_drawctxt_destroy(struct kgsl_device *device,
			unsigned int drawctxt_id);

#endif  /* __GSL_DRAWCTXT_H */
