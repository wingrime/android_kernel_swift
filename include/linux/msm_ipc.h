/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#ifndef _LINUX_MSM_IPC_H_
#define _LINUX_MSM_IPC_H_

#include <linux/types.h>
#include <linux/ioctl.h>

struct msm_ipc_port_addr {
	uint32_t node_id;
	uint32_t port_id;
};

struct msm_ipc_port_name {
	uint32_t service;
	uint32_t instance;
};

struct msm_ipc_addr {
	unsigned char  addrtype;
	union {
		struct msm_ipc_port_addr port_addr;
		struct msm_ipc_port_name port_name;
	} addr;
};

#define MSM_IPC_WAIT_FOREVER	(~0)  /* timeout for permanent subscription */

/*
 * Socket API
 */

#ifndef AF_MSM_IPC
#define AF_MSM_IPC		27
#endif

#ifndef PF_MSM_IPC
#define PF_MSM_IPC		AF_MSM_IPC
#endif

#define MSM_IPC_ADDR_NAME		1
#define MSM_IPC_ADDR_ID			2

struct sockaddr_msm_ipc {
	unsigned short family;
	struct msm_ipc_addr address;
	unsigned char reserved;
};

#define IPC_ROUTER_IOCTL_MAGIC (0xC3)

#define IPC_ROUTER_IOCTL_GET_VERSION \
	_IOR(IPC_ROUTER_IOCTL_MAGIC, 0, unsigned int)

#define IPC_ROUTER_IOCTL_GET_MTU \
	_IOR(IPC_ROUTER_IOCTL_MAGIC, 1, unsigned int)

#define IPC_ROUTER_IOCTL_LOOKUP_SERVER \
	_IOWR(IPC_ROUTER_IOCTL_MAGIC, 2, struct sockaddr_msm_ipc)

#define IPC_ROUTER_IOCTL_GET_CURR_PKT_SIZE \
	_IOR(IPC_ROUTER_IOCTL_MAGIC, 3, unsigned int)

#define IPC_ROUTER_IOCTL_BIND_CONTROL_PORT \
	_IOR(IPC_ROUTER_IOCTL_MAGIC, 4, unsigned int)

struct server_lookup_args {
	struct msm_ipc_port_name port_name;
	int num_entries_in_array;
	int num_entries_found;
	struct msm_ipc_port_addr port_addr[0];
};

#endif
