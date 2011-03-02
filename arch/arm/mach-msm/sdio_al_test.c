/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

/*
 * SDIO-Abstraction-Layer Test Module.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/platform_device.h>
#include <mach/sdio_smem.h>

#include <mach/sdio_al.h>
#include <linux/debugfs.h>

/** Module name string */
#define TEST_MODULE_NAME "sdio_al_test"

#define TEST_SIGNATURE 0x12345678
#define TEST_CONFIG_SIGNATURE 0xBEEFCAFE

#define MAX_XFER_SIZE (16*1024)
#define SMEM_MAX_XFER_SIZE 0xBC000

#define CHANNEL_NAME_SIZE 9

#define TEST_DBG(x...) if (test_ctx->runtime_debug) pr_info(x)

struct test_config_msg {
  u32 signature;
  u32 is_loopback_in_9k;
  u32 is_lpm_test;
  u32 packet_length;
  u32 num_packets;
  u32 sleep_interval;
  u32 num_iterations;
};

struct test_result_msg {
  u32 signature;
  u32 is_successful;
};

struct test_work {
	struct work_struct work;
	struct test_channel *test_ch;
};

enum sdio_channels_ids {
	SDIO_RPC,
	SDIO_QMI,
	SDIO_RMNT,
	SDIO_DIAG,
	SDIO_DUN,
	SDIO_SMEM,
	SDIO_MAX_CHANNELS
};

enum sdio_test_types {
	LOOPBACK_TEST,
	SENDER_TEST,
	A2_TEST
};


enum sdio_test_results {
	TEST_NO_RESULT,
	TEST_FAILED,
	TEST_PASSED
};

struct test_channel {
	struct sdio_channel *ch;

	char name[CHANNEL_NAME_SIZE];
	int ch_id;

	u32 *buf;
	u32 buf_size;

	struct workqueue_struct *workqueue;
	struct test_work test_work;

	u32 rx_bytes;
	u32 tx_bytes;

	wait_queue_head_t   wait_q;
	atomic_t rx_notify_count;
	atomic_t tx_notify_count;
	atomic_t any_notify_count;

	int wait_counter;

	int is_used;
	int test_type;
	int ch_ready;

	struct test_config_msg config_msg;

	int test_completed;
	int test_result;
};

struct sdio_al_test_debug {
	u32 dun_throughput;
	u32 rmnt_throughput;
	struct dentry *debug_root;
	struct dentry *debug_test_result;
	struct dentry *debug_dun_throughput;
	struct dentry *debug_rmnt_throughput;
};

struct test_context {
	dev_t dev_num;
	struct device *dev;
	struct cdev *cdev;

	struct test_channel *test_ch;

	struct test_channel *test_ch_arr[SDIO_MAX_CHANNELS];

	long testcase;

	const char *name;

	int exit_flag;

	u32 signature;

	int runtime_debug;

	struct platform_device smem_pdev;
	struct sdio_smem_client *sdio_smem;
	int smem_was_init;
	u8 *smem_buf;

	wait_queue_head_t   wait_q;
	int test_completed;
	int test_result;
	struct sdio_al_test_debug debug;
};

static struct test_context *test_ctx;

#ifdef CONFIG_DEBUG_FS
/*
*
* Trigger on/off for debug messages
* for trigger off the data messages debug level use:
* echo 0 > /sys/kernel/debugfs/sdio_al/debug_data_on
* for trigger on the data messages debug level use:
* echo 1 > /sys/kernel/debugfs/sdio_al/debug_data_on
* for trigger off the lpm messages debug level use:
* echo 0 > /sys/kernel/debugfs/sdio_al/debug_lpm_on
* for trigger on the lpm messages debug level use:
* echo 1 > /sys/kernel/debugfs/sdio_al/debug_lpm_on
*/
static int sdio_al_test_debugfs_init(void)
{
	test_ctx->debug.debug_root = debugfs_create_dir("sdio_al_test",
							       NULL);
	if (!test_ctx->debug.debug_root)
		return -ENOENT;

	test_ctx->debug.debug_test_result = debugfs_create_u32(
					"test_result",
					S_IRUGO | S_IWUGO,
					test_ctx->debug.debug_root,
					&test_ctx->test_result);

	test_ctx->debug.debug_dun_throughput = debugfs_create_u32(
					"dun_throughput",
					S_IRUGO | S_IWUGO,
					test_ctx->debug.debug_root,
					&test_ctx->debug.dun_throughput);

	test_ctx->debug.debug_rmnt_throughput = debugfs_create_u32(
					"rmnt_throughput",
					S_IRUGO | S_IWUGO,
					test_ctx->debug.debug_root,
					&test_ctx->debug.rmnt_throughput);

	if ((!test_ctx->debug.debug_dun_throughput) &&
	    (!test_ctx->debug.debug_rmnt_throughput)) {
		debugfs_remove_recursive(test_ctx->debug.debug_root);
		test_ctx->debug.debug_root = NULL;
		return -ENOENT;
	}
	return 0;
}

static void sdio_al_test_debugfs_cleanup(void)
{
       debugfs_remove(test_ctx->debug.debug_dun_throughput);
       debugfs_remove(test_ctx->debug.debug_rmnt_throughput);
       debugfs_remove(test_ctx->debug.debug_root);
}
#endif

static int channel_name_to_id(char *name)
{
	pr_info(TEST_MODULE_NAME "%s: channel name %s\n",
		__func__, name);

	if (!strcmp(name, "SDIO_RPC"))
		return SDIO_RPC;
	else if (!strcmp(name, "SDIO_QMI"))
		return SDIO_QMI;
	else if (!strcmp(name, "SDIO_RMNT"))
		return SDIO_RMNT;
	else if (!strcmp(name, "SDIO_DIAG"))
		return SDIO_DIAG;
	else if (!strcmp(name, "SDIO_DUN"))
		return SDIO_DUN;
	else if (!strcmp(name, "SDIO_SMEM"))
		return SDIO_SMEM;
	else
		return SDIO_MAX_CHANNELS;

	return SDIO_MAX_CHANNELS;
}

/**
 * Config message
 */

static void send_config_msg(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 write_avail = 0;
	int size = sizeof(test_ch->config_msg);

	pr_debug(TEST_MODULE_NAME "%s\n", __func__);

	memcpy(test_ch->buf, (void *)&test_ch->config_msg, size);

	if (test_ctx->exit_flag) {
		pr_info(TEST_MODULE_NAME ":Exit Test.\n");
		return;
	}

	pr_info(TEST_MODULE_NAME ":Sending the config message.\n");

	/* wait for data ready event */
	write_avail = sdio_write_avail(test_ch->ch);
	pr_debug(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
	if (write_avail < size) {
		wait_event(test_ch->wait_q,
			   atomic_read(&test_ch->tx_notify_count));
		atomic_dec(&test_ch->tx_notify_count);
	}

	write_avail = sdio_write_avail(test_ch->ch);
	pr_debug(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
	if (write_avail < size) {
		pr_info(TEST_MODULE_NAME ":not enough write avail.\n");
		return;
	}

	ret = sdio_write(test_ch->ch, test_ch->buf, size);
	if (ret)
		pr_err(TEST_MODULE_NAME ":%s sdio_write err=%d.\n",
			__func__, -ret);
}

/**
 * Loopback Test
 */
static void loopback_test(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;

	while (1) {

		if (test_ctx->exit_flag) {
			pr_info(TEST_MODULE_NAME ":Exit Test.\n");
			return;
		}

		TEST_DBG(TEST_MODULE_NAME "--LOOPBACK WAIT FOR EVENT--.\n");
		/* wait for data ready event */
		wait_event(test_ch->wait_q,
			   atomic_read(&test_ch->rx_notify_count));
		atomic_dec(&test_ch->rx_notify_count);

		read_avail = sdio_read_avail(test_ch->ch);
		if (read_avail == 0)
			continue;


		write_avail = sdio_write_avail(test_ch->ch);
		if (write_avail < read_avail) {
			pr_info(TEST_MODULE_NAME
				":not enough write avail.\n");
			continue;
		}

		ret = sdio_read(test_ch->ch, test_ch->buf, read_avail);
		if (ret) {
			pr_info(TEST_MODULE_NAME
			       ":worker, sdio_read err=%d.\n", -ret);
			continue;
		}
		test_ch->rx_bytes += read_avail;

		TEST_DBG(TEST_MODULE_NAME ":worker total rx bytes = 0x%x.\n",
			 test_ch->rx_bytes);


		ret = sdio_write(test_ch->ch,
				 test_ch->buf, read_avail);
		if (ret) {
			pr_info(TEST_MODULE_NAME
				":loopback sdio_write err=%d.\n",
				-ret);
			continue;
		}
		test_ch->tx_bytes += read_avail;

		TEST_DBG(TEST_MODULE_NAME
			 ":loopback total tx bytes = 0x%x.\n",
			 test_ch->tx_bytes);
	} /* end of while */
}

/**
 * Check if all tests completed
 */
static void check_test_completion(void)
{
	int i;

	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used) || (!tch->ch_ready))
			continue;
		if (!tch->test_completed) {
			pr_info(TEST_MODULE_NAME ":Channel %s test is not "
						 "completed",
				tch->name);
			return;
		}
	}
	pr_info(TEST_MODULE_NAME ":Test is completed");
	test_ctx->test_completed = 1;
	wake_up(&test_ctx->wait_q);
}

/**
 * sender Test
 */
static void sender_test(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;
	int packet_count = 0;
	int size = 512;
	u16 *buf16 = (u16 *) test_ch->buf;
	int i;
	int max_packet_count = 10000;
	int random_num = 0;

	max_packet_count = test_ch->config_msg.num_packets;

	for (i = 0 ; i < size / 2 ; i++)
		buf16[i] = (u16) (i & 0xFFFF);


	pr_info(TEST_MODULE_NAME
		 ":SENDER TEST START for chan %s\n", test_ch->name);

	while (packet_count < max_packet_count) {

		if (test_ctx->exit_flag) {
			pr_info(TEST_MODULE_NAME ":Exit Test.\n");
			return;
		}

		random_num = get_random_int();
		size = (random_num % test_ch->config_msg.packet_length) + 1;

		TEST_DBG(TEST_MODULE_NAME "SENDER WAIT FOR EVENT for chan %s\n",
			test_ch->name);

		/* wait for data ready event */
		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
		if (write_avail < size) {
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->tx_notify_count));
			atomic_dec(&test_ch->tx_notify_count);
		}

		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
		if (write_avail < size) {
			pr_info(TEST_MODULE_NAME ":not enough write avail.\n");
			continue;
		}

		test_ch->buf[0] = packet_count;

		ret = sdio_write(test_ch->ch, test_ch->buf, size);
		if (ret) {
			pr_info(TEST_MODULE_NAME ":sender sdio_write err=%d.\n",
				-ret);
			goto exit_err;
		}

		/* wait for read data ready event */
		TEST_DBG(TEST_MODULE_NAME ":sender wait for rx data for "
					  "chan %s\n",
			 test_ch->name);
		read_avail = sdio_read_avail(test_ch->ch);
		wait_event(test_ch->wait_q,
			   atomic_read(&test_ch->rx_notify_count));
		atomic_dec(&test_ch->rx_notify_count);

		read_avail = sdio_read_avail(test_ch->ch);

		if (read_avail != size) {
			pr_info(TEST_MODULE_NAME
				":read_avail size %d for chan %s not as "
				"expected size %d.\n",
				read_avail, test_ch->name, size);
			goto exit_err;
		}

		memset(test_ch->buf, 0x00, size);

		ret = sdio_read(test_ch->ch, test_ch->buf, size);
		if (ret) {
			pr_info(TEST_MODULE_NAME ":sender sdio_read for chan %s"
						 " err=%d.\n",
				test_ch->name, -ret);
			goto exit_err;
		}


		if ((test_ch->buf[0] != packet_count) && (size != 1)) {
			pr_info(TEST_MODULE_NAME ":sender sdio_read WRONG DATA"
						 " for chan %s, size=%d\n",
				test_ch->name, size);
			goto exit_err;
		}

		test_ch->tx_bytes += size;
		test_ch->rx_bytes += size;
		packet_count++;

		TEST_DBG(TEST_MODULE_NAME
			 ":sender total rx bytes = 0x%x , packet#=%d, size=%d"
			 " for chan %s\n",
			 test_ch->rx_bytes, packet_count, size, test_ch->name);
		TEST_DBG(TEST_MODULE_NAME
			 ":sender total tx bytes = 0x%x , packet#=%d, size=%d"
			 " for chan %s\n",
			 test_ch->tx_bytes, packet_count, size, test_ch->name);

	} /* end of while */

	pr_info(TEST_MODULE_NAME
		 ":SENDER TEST END: total rx bytes = 0x%x, "
		 " total tx bytes = 0x%x for chan %s\n",
		 test_ch->rx_bytes, test_ch->tx_bytes, test_ch->name);

	pr_info(TEST_MODULE_NAME ": TEST PASS for chan %s.\n",
		test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_PASSED;
	check_test_completion();
	return;

exit_err:
	pr_info(TEST_MODULE_NAME ": TEST FAIL for chan %s.\n",
		test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_FAILED;
	check_test_completion();
	return;
}

/**
 * A2 Perf Test
 */
static void a2_performance_test(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;
	int tx_packet_count = 0;
	int rx_packet_count = 0;
	int size = 0;
	u16 *buf16 = (u16 *) test_ch->buf;
	int i;
	int total_bytes = 0;
	int max_packets = 10000;
	u32 packet_size = test_ch->buf_size;

	u64 start_jiffy, end_jiffy, delta_jiffies;
	unsigned int time_msec = 0;
	u32 throughput = 0;

	max_packets = test_ch->config_msg.num_packets;
	packet_size = test_ch->config_msg.packet_length;

	for (i = 0; i < packet_size / 2; i++)
		buf16[i] = (u16) (i & 0xFFFF);

	pr_info(TEST_MODULE_NAME ": A2 PERFORMANCE TEST START for chan %s\n",
		test_ch->name);

	start_jiffy = get_jiffies_64(); /* read the current time */

	while (tx_packet_count < max_packets) {

		if (test_ctx->exit_flag) {
			pr_info(TEST_MODULE_NAME ":Exit Test.\n");
			return;
		}

		/* wait for data ready event */
		/* use a func to avoid compiler optimizations */
		write_avail = sdio_write_avail(test_ch->ch);
		read_avail = sdio_read_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, write_avail=%d, "
					 "read_avail=%d for chan %s\n",
			test_ch->name, write_avail, read_avail,
			test_ch->name);
		if ((write_avail == 0) && (read_avail == 0)) {
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->any_notify_count));
			atomic_set(&test_ch->any_notify_count, 0);
		}

		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, write_avail=%d\n",
			 test_ch->name, write_avail);
		if (write_avail > 0) {
			size = min(packet_size, write_avail) ;
			pr_debug(TEST_MODULE_NAME ":tx size = %d for chan %s\n",
				 size, test_ch->name);
			test_ch->buf[0] = tx_packet_count;
			test_ch->buf[(size/4)-1] = tx_packet_count;

			ret = sdio_write(test_ch->ch, test_ch->buf, size);
			if (ret) {
				pr_info(TEST_MODULE_NAME ":sdio_write err=%d"
							 " for chan %s\n",
					-ret, test_ch->name);
				goto exit_err;
			}
			tx_packet_count++;
			test_ch->tx_bytes += size;
		}

		read_avail = sdio_read_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, read_avail=%d\n",
			 test_ch->name, read_avail);
		if (read_avail > 0) {
			size = min(packet_size, read_avail);
			pr_debug(TEST_MODULE_NAME ":rx size = %d.\n", size);
			ret = sdio_read(test_ch->ch, test_ch->buf, size);
			if (ret) {
				pr_info(TEST_MODULE_NAME ": sdio_read size %d "
							 " err=%d"
							 " for chan %s\n",
					size, -ret, test_ch->name);
				goto exit_err;
			}
			rx_packet_count++;
			test_ch->rx_bytes += size;
		}

		TEST_DBG(TEST_MODULE_NAME
			 ":total rx bytes = %d , rx_packet#=%d"
			 " for chan %s\n",
			 test_ch->rx_bytes, rx_packet_count, test_ch->name);
		TEST_DBG(TEST_MODULE_NAME
			 ":total tx bytes = %d , tx_packet#=%d"
			 " for chan %s\n",
			 test_ch->tx_bytes, tx_packet_count, test_ch->name);

	} /* while (tx_packet_count < max_packets ) */

	end_jiffy = get_jiffies_64(); /* read the current time */

	delta_jiffies = end_jiffy - start_jiffy;
	time_msec = jiffies_to_msecs(delta_jiffies);

	pr_info(TEST_MODULE_NAME ":total rx bytes = 0x%x , rx_packet#=%d for"
				 " chan %s.\n",
		test_ch->rx_bytes, rx_packet_count, test_ch->name);
	pr_info(TEST_MODULE_NAME ":total tx bytes = 0x%x , tx_packet#=%d"
				 " for chan %s.\n",
		test_ch->tx_bytes, tx_packet_count, test_ch->name);

	total_bytes = (test_ch->tx_bytes + test_ch->rx_bytes);
	pr_err(TEST_MODULE_NAME ":total bytes = %d, time msec = %d"
				" for chan %s\n",
		   total_bytes , (int) time_msec, test_ch->name);

	throughput = (total_bytes / time_msec) * 8 / 1000;
	pr_err(TEST_MODULE_NAME ":Performance = %d Mbit/sec for chan %s\n",
	throughput, test_ch->name);

#ifdef CONFIG_DEBUG_FS
	switch (test_ch->ch_id) {
	case SDIO_DUN:
		test_ctx->debug.dun_throughput = throughput;
		break;
	case SDIO_RMNT:
		test_ctx->debug.rmnt_throughput = throughput;
		break;
	default:
		pr_err(TEST_MODULE_NAME "No debugfs for this channel "
					"throughput");
	}
#endif

	pr_err(TEST_MODULE_NAME ": A2 PERFORMANCE TEST END for chan %s.\n",
	       test_ch->name);

	pr_err(TEST_MODULE_NAME ": TEST PASS for chan %s\n", test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_PASSED;
	check_test_completion();
	return;

exit_err:
	pr_err(TEST_MODULE_NAME ": TEST FAIL for chan %s\n", test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_FAILED;
	check_test_completion();
	return;
}

/**
 * Worker thread to handle the tests types
 */
static void worker(struct work_struct *work)
{
	struct test_channel *test_ch = NULL;
	struct test_work *test_work = container_of(work,
						 struct	test_work,
						 work);
	int test_type = 0;

	test_ch = test_work->test_ch;

	if (test_ch == NULL) {
		pr_err(TEST_MODULE_NAME ":NULL test_ch\n");
		return;
	}

	test_type = test_ch->test_type;

	switch (test_type) {
	case LOOPBACK_TEST:
		loopback_test(test_ch);
		break;
	case SENDER_TEST:
		sender_test(test_ch);
		break;
	case A2_TEST:
		a2_performance_test(test_ch);
		break;
	default:
		pr_err(TEST_MODULE_NAME ":Bad Test type = %d.\n",
			(int) test_type);
	}
}


/**
 * Notification Callback
 *
 * Notify the worker
 *
 */
static void notify(void *priv, unsigned channel_event)
{
	struct test_channel *test_ch = (struct test_channel *) priv;

	pr_debug(TEST_MODULE_NAME ":notify event=%d.\n", channel_event);

	if (test_ch->ch == NULL) {
		pr_info(TEST_MODULE_NAME ":notify before ch ready.\n");
		return;
	}
	BUG_ON(test_ctx->signature != TEST_SIGNATURE);

	switch (channel_event) {
	case SDIO_EVENT_DATA_READ_AVAIL:
		atomic_inc(&test_ch->rx_notify_count);
		atomic_set(&test_ch->any_notify_count, 1);
		TEST_DBG(TEST_MODULE_NAME ":SDIO_EVENT_DATA_READ_AVAIL, "
					  "any_notify_count=%d, "
					  "rx_notify_count=%d\n",
			 atomic_read(&test_ch->any_notify_count),
			 atomic_read(&test_ch->rx_notify_count));
		break;

	case SDIO_EVENT_DATA_WRITE_AVAIL:
		atomic_inc(&test_ch->tx_notify_count);
		atomic_set(&test_ch->any_notify_count, 1);
		TEST_DBG(TEST_MODULE_NAME ":SDIO_EVENT_DATA_WRITE_AVAIL, "
					  "any_notify_count=%d, "
					  "tx_notify_count=%d\n",
			 atomic_read(&test_ch->any_notify_count),
			 atomic_read(&test_ch->tx_notify_count));
		break;

	default:
		BUG();
	}
	wake_up(&test_ch->wait_q);

}

#ifdef CONFIG_MSM_SDIO_SMEM
static int sdio_smem_test_cb(int event)
{
	struct test_channel *tch = test_ctx->test_ch_arr[SDIO_SMEM];
	pr_debug(TEST_MODULE_NAME ":%s: Received event %d\n", __func__, event);

	switch (event) {
	case SDIO_SMEM_EVENT_READ_DONE:
		tch->rx_bytes += SMEM_MAX_XFER_SIZE;
		if (tch->rx_bytes >= 40000000) {
			if (!tch->test_completed) {
				pr_info(TEST_MODULE_NAME ":SMEM test PASSED\n");
				tch->test_completed = 1;
				tch->test_result = TEST_PASSED;
				check_test_completion();
			}

		}
		break;
	case SDIO_SMEM_EVENT_READ_ERR:
		pr_err(TEST_MODULE_NAME ":Read overflow, SMEM test FAILED\n");
		tch->test_completed = 1;
		tch->test_result = TEST_FAILED;
		check_test_completion();
		return -EIO;
	default:
		pr_err(TEST_MODULE_NAME ":Unhandled event\n");
		return -EINVAL;
	}
	return 0;
}

static int sdio_smem_test_probe(struct platform_device *pdev)
{
	int ret = 0;

	test_ctx->sdio_smem = container_of(pdev, struct sdio_smem_client,
					   plat_dev);

	test_ctx->sdio_smem->buf = test_ctx->smem_buf;
	test_ctx->sdio_smem->size = SMEM_MAX_XFER_SIZE;
	test_ctx->sdio_smem->cb_func = sdio_smem_test_cb;
	ret = sdio_smem_register_client();
	if (ret)
		pr_info(TEST_MODULE_NAME "%s: Error (%d) registering sdio_smem "
					 "test client\n",
			__func__, ret);
	return ret;
}

static struct platform_driver sdio_smem_drv = {
	.probe		= sdio_smem_test_probe,
	.driver		= {
		.name	= "SDIO_SMEM_CLIENT",
		.owner	= THIS_MODULE,
	},
};
#endif


static void default_sdio_al_test_release(struct device *dev)
{
	pr_info(MODULE_NAME ":platform device released.\n");
}

/**
 * Test Main
 */
static int test_start(void)
{
	int ret = -ENOMEM;
	int i;

	pr_debug(TEST_MODULE_NAME ":Starting Test ....\n");

	test_ctx->test_completed = 0;
	test_ctx->test_result = TEST_NO_RESULT;
	test_ctx->debug.dun_throughput = 0;
	test_ctx->debug.rmnt_throughput = 0;

	/* Open The Channels */
	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used))
			continue;

		tch->rx_bytes = 0;
		tch->tx_bytes = 0;

		atomic_set(&tch->tx_notify_count, 0);
		atomic_set(&tch->rx_notify_count, 0);
		atomic_set(&tch->any_notify_count, 0);

		memset(tch->buf, 0x00, tch->buf_size);
		tch->test_result = TEST_NO_RESULT;

		tch->test_completed = 0;

		if (!tch->ch_ready) {
			pr_info(TEST_MODULE_NAME ":openning channel %s\n",
				tch->name);
			tch->ch_ready = true;
			if (tch->ch_id == SDIO_SMEM) {
				test_ctx->smem_pdev.name = "SDIO_SMEM";
				test_ctx->smem_pdev.dev.release =
					default_sdio_al_test_release;
				platform_device_register(&test_ctx->smem_pdev);
			} else {
				ret = sdio_open(tch->name , &tch->ch, tch,
						notify);
				if (ret) {
					pr_info(TEST_MODULE_NAME
						":openning channel %s failed\n",
					tch->name);
					tch->ch_ready = false;
				}
			}
		}

		if ((tch->ch_ready) && (tch->ch_id != SDIO_SMEM))
			send_config_msg(tch);
	}

	pr_debug(TEST_MODULE_NAME ":queue_work..\n");
	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used) || (!tch->ch_ready) ||
		    (tch->ch_id == SDIO_SMEM))
			continue;
		queue_work(tch->workqueue, &tch->test_work.work);
	}

	pr_info(TEST_MODULE_NAME ":Waiting for the test completion\n");

	if (!test_ctx->test_completed) {
		wait_event(test_ctx->wait_q, test_ctx->test_completed);
		pr_info(TEST_MODULE_NAME ":Test Completed\n");
		for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
			struct test_channel *tch = test_ctx->test_ch_arr[i];

			if ((!tch) || (!tch->is_used) || (!tch->ch_ready))
				continue;
			if (tch->test_result == TEST_FAILED) {
				pr_info(TEST_MODULE_NAME ":Test FAILED\n");
				test_ctx->test_result = TEST_FAILED;
				return 0;
			}
		}
		pr_info(TEST_MODULE_NAME ":Test PASSED\n");
		test_ctx->test_result = TEST_PASSED;
	}


	return 0;
}

static int set_params_loopback_9k(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->test_type = SENDER_TEST;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.is_loopback_in_9k = 1;
	tch->config_msg.is_lpm_test = 0;
	tch->config_msg.packet_length = 512;
	tch->config_msg.num_packets = 10000;
	tch->config_msg.sleep_interval = 0;
	tch->config_msg.num_iterations = 1;

	if (tch->ch_id == SDIO_RPC)
		tch->config_msg.packet_length = 128;

	return 0;
}

static int set_params_a2_perf(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->test_type = A2_TEST;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.is_loopback_in_9k = 1;
	tch->config_msg.is_lpm_test = 0;
	if (tch->ch_id == SDIO_DIAG)
		tch->config_msg.packet_length = 512;
	else
		tch->config_msg.packet_length = MAX_XFER_SIZE;

	tch->config_msg.num_packets = 10000;
	tch->config_msg.sleep_interval = 0;
	tch->config_msg.num_iterations = 1;
	return 0;
}

static int set_params_smem_test(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	return 0;
}

/**
 * Write File.
 *
 * @note Trigger the test from user space by:
 * echo 1 > /dev/sdio_al_test
 *
 */
ssize_t test_write(struct file *filp, const char __user *buf, size_t size,
		   loff_t *f_pos)
{
	int ret = 0;
	int i;

	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];
		if (!tch)
			continue;
		tch->is_used = 0;
	}

	ret = strict_strtol(buf, 10, &test_ctx->testcase);

	switch (test_ctx->testcase) {
	case 1:
		/* RPC */
		pr_debug(TEST_MODULE_NAME " --RPC sender--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]))
			return size;
		break;
	case 2:
		/* RPC, QMI and DIAG */
		pr_debug(TEST_MODULE_NAME " --RPC, QMI and DIAG sender--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_QMI]) ||
		    set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_DIAG]))
			return size;
		break;
	case 4:
		pr_debug(TEST_MODULE_NAME " --SMEM--.\n");
		if (set_params_smem_test(test_ctx->test_ch_arr[SDIO_SMEM]))
			return size;
		break;

	case 5:
		pr_debug(TEST_MODULE_NAME " --SMEM and RPC--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_smem_test(test_ctx->test_ch_arr[SDIO_SMEM]))
			return size;
		break;
	case 6:
		pr_debug(TEST_MODULE_NAME " --RmNet A2 Performance--.\n");
		if (set_params_a2_perf(test_ctx->test_ch_arr[SDIO_RMNT]))
			return size;
		break;

	case 7:
		pr_debug(TEST_MODULE_NAME " --DUN A2 Performance--.\n");
		if (set_params_a2_perf(test_ctx->test_ch_arr[SDIO_DUN]))
			return size;
		break;
	case 8:
		pr_debug(TEST_MODULE_NAME " --RmNet and DUN A2 Performance--."
					  "\n");
		if (set_params_a2_perf(test_ctx->test_ch_arr[SDIO_RMNT]) ||
		    set_params_a2_perf(test_ctx->test_ch_arr[SDIO_DUN]))
			return size;
		break;
	case 9:
		pr_debug(TEST_MODULE_NAME " --RPC sender and RmNet A2 "
					  "Performance--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_a2_perf(test_ctx->test_ch_arr[SDIO_RMNT]))
			return size;
		break;
	case 10:
		pr_debug(TEST_MODULE_NAME " --All the channels--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_QMI]) ||
		    set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_DIAG]) ||
		    set_params_a2_perf(test_ctx->test_ch_arr[SDIO_RMNT]) ||
		    set_params_a2_perf(test_ctx->test_ch_arr[SDIO_DUN]) ||
		    set_params_smem_test(test_ctx->test_ch_arr[SDIO_SMEM]))
			return size;
		break;
	case 98:
		pr_info(TEST_MODULE_NAME " set runtime debug on");
		test_ctx->runtime_debug = 1;
		return size;
	case 99:
		pr_info(TEST_MODULE_NAME " set runtime debug off");
		test_ctx->runtime_debug = 0;
		return size;
	default:
		pr_info(TEST_MODULE_NAME ":Bad Test number = %d.\n",
			(int)test_ctx->testcase);
		return 0;
	}
	ret = test_start();
	if (ret) {
		pr_err(TEST_MODULE_NAME ":test_start failed, ret = %d.\n",
			ret);

	}
	return size;
}

/**
 * Test Channel Init.
 */
int test_channel_init(char *name)
{
	struct test_channel *test_ch;
	int ch_id = 0;
#ifdef CONFIG_MSM_SDIO_SMEM
	int ret;
#endif

	pr_debug(TEST_MODULE_NAME ":%s.\n", __func__);
	pr_info(TEST_MODULE_NAME ": init test cahnnel %s.\n", name);

	ch_id = channel_name_to_id(name);
	pr_debug(TEST_MODULE_NAME ":id = %d.\n", ch_id);
	if (test_ctx->test_ch_arr[ch_id] == NULL) {
		test_ch = kzalloc(sizeof(*test_ch), GFP_KERNEL);
		if (test_ch == NULL) {
			pr_err(TEST_MODULE_NAME ":kzalloc err for allocating "
						"test_ch %s.\n",
			       name);
			return -ENOMEM;
		}
		test_ctx->test_ch_arr[ch_id] = test_ch;

		test_ch->ch_id = ch_id;

		memcpy(test_ch->name, name, CHANNEL_NAME_SIZE);

		test_ch->buf_size = MAX_XFER_SIZE;

		test_ch->buf = kzalloc(test_ch->buf_size, GFP_KERNEL);
		if (test_ch->buf == NULL) {
			kfree(test_ch);
			test_ctx->test_ch = NULL;
			return -ENOMEM;
		}

		if (test_ch->ch_id == SDIO_SMEM) {
			test_ctx->smem_buf = kzalloc(SMEM_MAX_XFER_SIZE,
						     GFP_KERNEL);
			if (test_ctx->smem_buf == NULL) {
				pr_err(TEST_MODULE_NAME ":%s: Unable to "
							"allocate smem buf\n",
				       __func__);
				kfree(test_ch);
				test_ctx->test_ch = NULL;
				return -ENOMEM;
			}

#ifdef CONFIG_MSM_SDIO_SMEM
			ret = platform_driver_register(&sdio_smem_drv);
			if (ret) {
				pr_err(TEST_MODULE_NAME ":%s: Unable to "
							"register sdio smem "
							"test client\n",
				       __func__);
				return ret;
			}
#endif
		} else {
			test_ch->workqueue =
				create_singlethread_workqueue(test_ch->name);
			test_ch->test_work.test_ch = test_ch;
			INIT_WORK(&test_ch->test_work.work, worker);

			init_waitqueue_head(&test_ch->wait_q);
		}
	} else {
		pr_err(TEST_MODULE_NAME ":trying to call test_channel_init "
					"twice for chan %d\n",
		       ch_id);
	}

	return 0;
}

static struct class *test_class;

const struct file_operations test_fops = {
	.owner = THIS_MODULE,
	.write = test_write,
};

/**
 * Module Init.
 */
static int __init test_init(void)
{
	int ret;

	pr_debug(TEST_MODULE_NAME ":test_init.\n");

	test_ctx = kzalloc(sizeof(*test_ctx), GFP_KERNEL);
	if (test_ctx == NULL) {
		pr_err(TEST_MODULE_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}
	test_ctx->test_ch = NULL;
	test_ctx->signature = TEST_SIGNATURE;

	test_ctx->name = "UNKNOWN";

	init_waitqueue_head(&test_ctx->wait_q);

#ifdef CONFIG_DEBUG_FS
	sdio_al_test_debugfs_init();
#endif

	test_class = class_create(THIS_MODULE, TEST_MODULE_NAME);

	ret = alloc_chrdev_region(&test_ctx->dev_num, 0, 1, TEST_MODULE_NAME);
	if (ret) {
		pr_err(TEST_MODULE_NAME "alloc_chrdev_region err.\n");
		return -ENODEV;
	}

	test_ctx->dev = device_create(test_class, NULL, test_ctx->dev_num,
				      test_ctx, TEST_MODULE_NAME);
	if (IS_ERR(test_ctx->dev)) {
		pr_err(TEST_MODULE_NAME ":device_create err.\n");
		return -ENODEV;
	}

	test_ctx->cdev = cdev_alloc();
	if (test_ctx->cdev == NULL) {
		pr_err(TEST_MODULE_NAME ":cdev_alloc err.\n");
		return -ENODEV;
	}
	cdev_init(test_ctx->cdev, &test_fops);
	test_ctx->cdev->owner = THIS_MODULE;

	ret = cdev_add(test_ctx->cdev, test_ctx->dev_num, 1);
	if (ret)
		pr_err(TEST_MODULE_NAME ":cdev_add err=%d\n", -ret);
	else
		pr_debug(TEST_MODULE_NAME ":SDIO-AL-Test init OK..\n");

	return ret;
}

/**
 * Module Exit.
 */
static void __exit test_exit(void)
{
	int i;

	pr_debug(TEST_MODULE_NAME ":test_exit.\n");

	test_ctx->exit_flag = true;

	msleep(100); /* allow gracefully exit of the worker thread */

	cdev_del(test_ctx->cdev);
	device_destroy(test_class, test_ctx->dev_num);
	unregister_chrdev_region(test_ctx->dev_num, 1);

	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];
		if (!tch)
			continue;
		kfree(tch->buf);
		kfree(tch);
	}

#ifdef CONFIG_DEBUG_FS
	sdio_al_test_debugfs_cleanup();
#endif

	kfree(test_ctx);

	pr_debug(TEST_MODULE_NAME ":test_exit complete.\n");
}

module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SDIO_AL Test");
MODULE_AUTHOR("Amir Samuelov <amirs@codeaurora.org>");


