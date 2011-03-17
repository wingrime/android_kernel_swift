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
 *
 */

/*
 * SDIO-Abstraction-Layer API.
 */

#ifndef __SDIO_AL__
#define __SDIO_AL__

#include <linux/mmc/card.h>

#define DRV_VERSION "1.30"
#define MODULE_NAME "sdio_al"
#define SDIOC_CHAN_TO_FUNC_NUM(x)	((x)+2)
#define REAL_FUNC_TO_FUNC_IN_ARRAY(x)	((x)-1)

struct sdio_channel; /* Forward Declaration */

/**
 *  Channel Events.
 *  Available bytes notification.
 */
#define SDIO_EVENT_DATA_READ_AVAIL      0x01
#define SDIO_EVENT_DATA_WRITE_AVAIL     0x02

struct sdio_al_platform_data {
	int (*config_mdm2ap_status)(int);
	int (*get_mdm2ap_status)(void);
};

/**
 * sdio_open - open a channel for read/write data.
 *
 * @name: channel name - identify the channel to open.
 * @ch: channel handle returned.
 * @priv: caller private context pointer, passed to the notify callback.
 * @notify: notification callback for data available.
 * @channel_event: SDIO_EVENT_DATA_READ_AVAIL or SDIO_EVENT_DATA_WRITE_AVAIL
 * @return 0 on success, negative value on error.
 *
 * Warning: notify() may be called before open returns.
 */
int sdio_open(const char *name, struct sdio_channel **ch, void *priv,
	     void (*notify)(void *priv, unsigned channel_event));


/**
 * sdio_close - close a channel.
 *
 * @ch: channel handle.
 * @return 0 on success, negative value on error.
 */
int sdio_close(struct sdio_channel *ch);

/**
 * sdio_read - synchronous read.
 *
 * @ch: channel handle.
 * @data: caller buffer pointer. should be non-cacheable.
 * @len: byte count.
 * @return 0 on success, negative value on error.
 *
 * May wait if no available bytes.
 * May wait if other channel with higher priority has pending
 * transfers.
 * Client should check available bytes prior to calling this
 * api.
 */
int sdio_read(struct sdio_channel *ch, void *data, int len);

/**
 * sdio_write - synchronous write.
 *
 * @ch: channel handle.
 * @data: caller buffer pointer. should be non-cacheable.
 * @len: byte count.
 * @return 0 on success, negative value on error.
 *
 * May wait if no available bytes.
 * May wait if other channel with higher priority has pending
 * transfers.
 * Client should check available bytes prior to calling this
 * api.
 */
int sdio_write(struct sdio_channel *ch, const void *data, int len);

/**
 * sdio_write_avail - get available bytes to write.
 *
 * @ch: channel handle.
 * @return byte count on success, negative value on error.
 */
int sdio_write_avail(struct sdio_channel *ch);

/**
 * sdio_read_avail - get available bytes to read.
 *
 * @ch: channel handle.
 * @return byte count on success, negative value on error.
 */
int sdio_read_avail(struct sdio_channel *ch);

/**
 *  Set the threshold to trigger interrupt from SDIO-Card on
 *  available bytes to write.
 *
 * @ch: channel handle.
 * @threshold: bytes count;
 *
 * @return 0 on success, negative value on error.
 */
int sdio_set_write_threshold(struct sdio_channel *ch, int threshold);

/**
 *  Set the threshold to trigger interrupt from SDIO-Card on
 *  available bytes to read.
 *
 * @ch: channel handle.
 * @threshold: bytes count;
 *
 * @return 0 on success, negative value on error.
 */
int sdio_set_read_threshold(struct sdio_channel *ch, int threshold);

/**
 * sdio_downloader_setup
 * initializes the TTY driver
 *
 * @card: a pointer to mmc_card.
 * @num_of_devices: number of devices.
 * @channel_number: channel number.
 * @return 0 on success or negative value on error.
 *
 * The TTY stack needs to know in advance how many devices it should
 * plan to manage. Use this call to set up the ports that will
 * be exported through SDIO.
 */
int sdio_downloader_setup(struct mmc_card *card,
			  unsigned int num_of_devices,
			  int func_number,
			  int(*func)(void));

/**
 * test_channel_init
 * initializes a test channel
 *
 * @card: the channel name.
 * @return 0 on success or negative value on error.
 *
 */
int test_channel_init(char *name);

#endif /* __SDIO_AL__ */
