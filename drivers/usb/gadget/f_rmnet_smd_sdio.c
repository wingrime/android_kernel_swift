/*
 * f_rmnet_smd_sdio.c -- RmNet SMD & SDIO function driver
 *
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2003-2004 Robert Schwebel, Benedikt Spranger
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (c) 2011 Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>

#include <linux/usb/cdc.h>
#include <linux/usb/composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/android_composite.h>
#include <linux/termios.h>
#include <linux/debugfs.h>

#include <mach/msm_smd.h>
#include <mach/sdio_cmux.h>
#include <mach/sdio_dmux.h>

static uint32_t rmnet_sdio_ctl_ch = CONFIG_RMNET_SMD_SDIO_CTL_CHANNEL;
module_param(rmnet_sdio_ctl_ch, uint, S_IRUGO);
MODULE_PARM_DESC(rmnet_sdio_ctl_ch, "RmNet control SDIO channel ID");

static uint32_t rmnet_sdio_data_ch = CONFIG_RMNET_SMD_SDIO_DATA_CHANNEL;
module_param(rmnet_sdio_data_ch, uint, S_IRUGO);
MODULE_PARM_DESC(rmnet_sdio_data_ch, "RmNet data SDIO channel ID");

static char *rmnet_smd_data_ch = CONFIG_RMNET_SDIO_SMD_DATA_CHANNEL;
module_param(rmnet_smd_data_ch, charp, S_IRUGO);
MODULE_PARM_DESC(rmnet_smd_data_ch, "RmNet data SMD channel");

#define ACM_CTRL_DTR	(1 << 0)

#define SDIO_MUX_HDR           8
#define RMNET_SDIO_NOTIFY_INTERVAL  5
#define RMNET_SDIO_MAX_NFY_SZE  sizeof(struct usb_cdc_notification)

#define RMNET_SDIO_RX_REQ_MAX             16
#define RMNET_SDIO_RX_REQ_SIZE            4096
#define RMNET_SDIO_TX_REQ_MAX             100

#define RMNET_SMD_RX_REQ_MAX		8
#define RMNET_SMD_RX_REQ_SIZE		2048
#define RMNET_SMD_TX_REQ_MAX		8
#define RMNET_SMD_TX_REQ_SIZE		2048
#define RMNET_SMD_TXN_MAX		2048

/* QMI requests & responses buffer*/
struct rmnet_sdio_qmi_buf {
	void *buf;
	int len;
	struct list_head list;
};

enum usb_rmnet_xport_type {
	USB_RMNET_XPORT_UNDEFINED,
	USB_RMNET_XPORT_SDIO,
	USB_RMNET_XPORT_SMD,
};

struct rmnet_sdio_dev {
	/* QMI lists */
	struct list_head qmi_req_q;
	struct list_head qmi_resp_q;

	/* Tx/Rx lists */
	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_queue;

	u8 dmux_write_done;
	unsigned long cbits_to_modem;

	struct work_struct ctl_rx_work;
	struct work_struct data_rx_work;
	struct work_struct set_modem_ctl_bits_work;

	struct delayed_work open_work;
	atomic_t sdio_open;
};

/* Data SMD channel */
struct rmnet_smd_info {
	struct smd_channel *ch;
	struct tasklet_struct tx_tlet;
	struct tasklet_struct rx_tlet;
#define CH_OPENED 0
	unsigned long flags;
	/* pending rx packet length */
	atomic_t rx_pkt;
	/* wait for smd open event*/
	wait_queue_head_t wait;
};

struct rmnet_smd_dev {
	/* Tx/Rx lists */
	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_queue;

	struct rmnet_smd_info smd_data;
};

struct rmnet_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;

	struct usb_ep *epout;
	struct usb_ep *epin;
	struct usb_ep *epnotify;
	struct usb_request *notify_req;

	struct rmnet_smd_dev smd_dev;
	struct rmnet_sdio_dev sdio_dev;

	u8 ifc_id;
	enum usb_rmnet_xport_type xport;
	spinlock_t lock;
	atomic_t online;
	atomic_t notify_count;
	struct workqueue_struct *wq;
	struct work_struct disconnect_work;

	/* pkt counters */
	unsigned long dpkts_tomsm;
	unsigned long dpkts_tomdm;
	unsigned long dpkts_tolaptop;
	unsigned long cpkts_tolaptop;
	unsigned long cpkts_tomdm;
};

static struct usb_interface_descriptor rmnet_interface_desc = {
	.bLength =              USB_DT_INTERFACE_SIZE,
	.bDescriptorType =      USB_DT_INTERFACE,
	.bNumEndpoints =        3,
	.bInterfaceClass =      USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =   USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =   USB_CLASS_VENDOR_SPEC,
};

/* Full speed support */
static struct usb_endpoint_descriptor rmnet_fs_notify_desc = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bEndpointAddress =     USB_DIR_IN,
	.bmAttributes =         USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(RMNET_SDIO_MAX_NFY_SZE),
	.bInterval =            1 << RMNET_SDIO_NOTIFY_INTERVAL,
};

static struct usb_endpoint_descriptor rmnet_fs_in_desc  = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bEndpointAddress =     USB_DIR_IN,
	.bmAttributes =         USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(64),
};

static struct usb_endpoint_descriptor rmnet_fs_out_desc = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bEndpointAddress =     USB_DIR_OUT,
	.bmAttributes =         USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(64),
};

static struct usb_descriptor_header *rmnet_fs_function[] = {
	(struct usb_descriptor_header *) &rmnet_interface_desc,
	(struct usb_descriptor_header *) &rmnet_fs_notify_desc,
	(struct usb_descriptor_header *) &rmnet_fs_in_desc,
	(struct usb_descriptor_header *) &rmnet_fs_out_desc,
	NULL,
};

/* High speed support */
static struct usb_endpoint_descriptor rmnet_hs_notify_desc  = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bEndpointAddress =     USB_DIR_IN,
	.bmAttributes =         USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(RMNET_SDIO_MAX_NFY_SZE),
	.bInterval =            RMNET_SDIO_NOTIFY_INTERVAL + 4,
};

static struct usb_endpoint_descriptor rmnet_hs_in_desc = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bEndpointAddress =     USB_DIR_IN,
	.bmAttributes =         USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor rmnet_hs_out_desc = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bEndpointAddress =     USB_DIR_OUT,
	.bmAttributes =         USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *rmnet_hs_function[] = {
	(struct usb_descriptor_header *) &rmnet_interface_desc,
	(struct usb_descriptor_header *) &rmnet_hs_notify_desc,
	(struct usb_descriptor_header *) &rmnet_hs_in_desc,
	(struct usb_descriptor_header *) &rmnet_hs_out_desc,
	NULL,
};

/* String descriptors */

static struct usb_string rmnet_string_defs[] = {
	[0].s = "QMI RmNet",
	{  } /* end of list */
};

static struct usb_gadget_strings rmnet_string_table = {
	.language =             0x0409, /* en-us */
	.strings =              rmnet_string_defs,
};

static struct usb_gadget_strings *rmnet_strings[] = {
	&rmnet_string_table,
	NULL,
};

static char *xport_to_str(enum usb_rmnet_xport_type t)
{
	switch (t) {
	case USB_RMNET_XPORT_SDIO:
		return "SDIO";
	case USB_RMNET_XPORT_SMD:
		return "SMD";
	default:
		return "UNDEFINED";
	}
}

static struct rmnet_sdio_qmi_buf *
rmnet_alloc_qmi(unsigned len, gfp_t kmalloc_flags)

{
	struct rmnet_sdio_qmi_buf *qmi;

	qmi = kmalloc(sizeof(struct rmnet_sdio_qmi_buf), kmalloc_flags);
	if (qmi != NULL) {
		qmi->buf = kmalloc(len, kmalloc_flags);
		if (qmi->buf == NULL) {
			kfree(qmi);
			qmi = NULL;
		}
	}

	return qmi ? qmi : ERR_PTR(-ENOMEM);
}

static void rmnet_free_qmi(struct rmnet_sdio_qmi_buf *qmi)
{
	kfree(qmi->buf);
	kfree(qmi);
}
/*
 * Allocate a usb_request and its buffer.  Returns a pointer to the
 * usb_request or a pointer with an error code if there is an error.
 */
static struct usb_request *
rmnet_alloc_req(struct usb_ep *ep, unsigned len, gfp_t kmalloc_flags)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, kmalloc_flags);

	if (len && req != NULL) {
		req->length = len;
		req->buf = kmalloc(len, kmalloc_flags);
		if (req->buf == NULL) {
			usb_ep_free_request(ep, req);
			req = NULL;
		}
	}

	return req ? req : ERR_PTR(-ENOMEM);
}

/*
 * Free a usb_request and its buffer.
 */
static void rmnet_free_req(struct usb_ep *ep, struct usb_request *req)
{
	kfree(req->buf);
	usb_ep_free_request(ep, req);
}

static int rmnet_sdio_rx_submit(struct rmnet_dev *dev, struct usb_request *req,
				gfp_t gfp_flags)
{
	struct sk_buff *skb;
	int retval;

	skb = alloc_skb(RMNET_SDIO_RX_REQ_SIZE + SDIO_MUX_HDR, gfp_flags);
	if (skb == NULL)
		return -ENOMEM;
	skb_reserve(skb, SDIO_MUX_HDR);

	req->buf = skb->data;
	req->length = RMNET_SDIO_RX_REQ_SIZE;
	req->context = skb;

	retval = usb_ep_queue(dev->epout, req, gfp_flags);
	if (retval)
		dev_kfree_skb_any(skb);

	return retval;
}

static void rmnet_sdio_start_rx(struct rmnet_dev *dev)
{
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	int status;
	struct usb_request *req;
	struct list_head *pool;
	unsigned long flags;

	pool = &sdio_dev->rx_idle;

	spin_lock_irqsave(&dev->lock, flags);
	while (!list_empty(pool)) {
		req = list_first_entry(pool, struct usb_request, list);
		list_del(&req->list);

		spin_unlock_irqrestore(&dev->lock, flags);
		status = rmnet_sdio_rx_submit(dev, req, GFP_KERNEL);
		spin_lock_irqsave(&dev->lock, flags);

		if (status) {
			ERROR(cdev, "rmnet data rx enqueue err %d\n", status);
			list_add_tail(&req->list, &sdio_dev->rx_idle);
			break;
		}
	}
	spin_unlock_irqrestore(&dev->lock, flags);
}

static void rmnet_sdio_data_receive_cb(void *priv, struct sk_buff *skb)
{
	struct rmnet_dev *dev = priv;
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	unsigned long flags;
	int status;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(&sdio_dev->tx_idle)) {
		spin_unlock_irqrestore(&dev->lock, flags);
		ERROR(cdev, "rmnet data Tx req full\n");
		return;
	}
	req = list_first_entry(&sdio_dev->tx_idle, struct usb_request, list);
	list_del(&req->list);
	spin_unlock_irqrestore(&dev->lock, flags);

	req->context = skb;
	req->buf = skb->data;
	req->length = skb->len;

	status = usb_ep_queue(dev->epin, req, GFP_KERNEL);
	if (status) {
		ERROR(cdev, "rmnet tx data enqueue err %d\n", status);
		spin_lock_irqsave(&dev->lock, flags);
		list_add_tail(&req->list, &sdio_dev->tx_idle);
		spin_unlock_irqrestore(&dev->lock, flags);
	}
	dev->dpkts_tolaptop++;
}

static void rmnet_sdio_data_write_done(void *priv, struct sk_buff *skb)
{
	struct rmnet_dev *dev = priv;
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	unsigned long flags;

	dev_kfree_skb_any(skb);

	spin_lock_irqsave(&dev->lock, flags);
	sdio_dev->dmux_write_done = 1;

	if (!atomic_read(&dev->online)) {
		DBG(cdev, "USB disconnected\n");
		spin_unlock_irqrestore(&dev->lock, flags);
		return;
	}

	if (!list_empty(&sdio_dev->rx_queue)) {
		sdio_dev->dmux_write_done = 0;
		queue_work(dev->wq, &sdio_dev->data_rx_work);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
}

static void rmnet_sdio_set_modem_ctl_bits_work(struct work_struct *w)
{
	struct rmnet_dev *dev = container_of(w, struct rmnet_dev,
			sdio_dev.set_modem_ctl_bits_work);
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;

	if (!atomic_read(&sdio_dev->sdio_open))
		return;

	sdio_cmux_tiocmset(rmnet_sdio_ctl_ch,
			sdio_dev->cbits_to_modem,
			~sdio_dev->cbits_to_modem);
}

static void rmnet_sdio_data_rx_work(struct work_struct *w)
{
	struct rmnet_dev *dev = container_of(w, struct rmnet_dev,
			sdio_dev.data_rx_work);
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct sk_buff *skb;
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	if (list_empty(&sdio_dev->rx_queue)) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return;
	}

	req = list_first_entry(&sdio_dev->rx_queue,
		struct usb_request, list);

	list_del(&req->list);
	spin_unlock_irqrestore(&dev->lock, flags);

	skb = req->context;
	ret = msm_sdio_dmux_write(rmnet_sdio_data_ch, skb);
	if (ret < 0) {
		ERROR(cdev, "rmnet SDIO data write failed\n");
		dev_kfree_skb_any(skb);
		spin_lock_irqsave(&dev->lock, flags);
		list_add_tail(&req->list, &sdio_dev->rx_idle);
		spin_unlock_irqrestore(&dev->lock, flags);
	} else {
		dev->dpkts_tomdm++;
		rmnet_sdio_rx_submit(dev, req, GFP_KERNEL);
	}

}

static void
rmnet_sdio_complete_epout(struct usb_ep *ep, struct usb_request *req)
{
	struct rmnet_dev *dev = ep->driver_data;
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	struct sk_buff *skb = req->context;
	int status = req->status;
	int queue = 0;

	if (dev->xport == USB_RMNET_XPORT_UNDEFINED) {
		dev_kfree_skb_any(skb);
		req->buf = 0;
		rmnet_free_req(ep, req);
		return;
	}

	switch (status) {
	case 0:
		/* successful completion */
		skb_put(skb, req->actual);
		queue = 1;
		break;
	case -ECONNRESET:
	case -ESHUTDOWN:
		/* connection gone */
		break;
	default:
		/* unexpected failure */
		ERROR(cdev, "RMNET %s response error %d, %d/%d\n",
			ep->name, status,
			req->actual, req->length);
		break;
	}

	spin_lock(&dev->lock);
	if (queue) {
		list_add_tail(&req->list, &sdio_dev->rx_queue);
		if (sdio_dev->dmux_write_done) {
			sdio_dev->dmux_write_done = 0;
			queue_work(dev->wq, &sdio_dev->data_rx_work);
		}
	} else {
		list_add_tail(&req->list, &sdio_dev->rx_idle);
	}
	spin_unlock(&dev->lock);
}

static void rmnet_sdio_complete_epin(struct usb_ep *ep, struct usb_request *req)
{
	struct rmnet_dev *dev = ep->driver_data;
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct sk_buff  *skb = req->context;
	struct usb_composite_dev *cdev = dev->cdev;
	int status = req->status;

	if (dev->xport == USB_RMNET_XPORT_UNDEFINED) {
		dev_kfree_skb_any(skb);
		req->buf = 0;
		rmnet_free_req(ep, req);
		return;
	}

	switch (status) {
	case 0:
		/* successful completion */
	case -ECONNRESET:
	case -ESHUTDOWN:
		/* connection gone */
		break;
	default:
		ERROR(cdev, "rmnet data tx ep error %d\n", status);
		break;
	}

	spin_lock(&dev->lock);
	list_add_tail(&req->list, &sdio_dev->tx_idle);
	spin_unlock(&dev->lock);
	dev_kfree_skb_any(skb);
}

static int rmnet_sdio_enable(struct rmnet_dev *dev)
{
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	int i;
	struct usb_request *req;

	/*
	 * If the memory allocation fails, all the allocated
	 * requests will be freed upon cable disconnect.
	 */
	for (i = 0; i < RMNET_SDIO_RX_REQ_MAX; i++) {
		req = rmnet_alloc_req(dev->epout, 0, GFP_KERNEL);
		if (IS_ERR(req))
			return PTR_ERR(req);
		req->complete = rmnet_sdio_complete_epout;
		list_add_tail(&req->list, &sdio_dev->rx_idle);
	}
	for (i = 0; i < RMNET_SDIO_TX_REQ_MAX; i++) {
		req = rmnet_alloc_req(dev->epin, 0, GFP_KERNEL);
		if (IS_ERR(req))
			return PTR_ERR(req);
		req->complete = rmnet_sdio_complete_epin;
		list_add_tail(&req->list, &sdio_dev->tx_idle);
	}

	rmnet_sdio_start_rx(dev);
	return 0;
}

static void rmnet_smd_start_rx(struct rmnet_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct rmnet_smd_dev *smd_dev = &dev->smd_dev;
	int status;
	struct usb_request *req;
	struct list_head *pool = &smd_dev->rx_idle;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	while (!list_empty(pool)) {
		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);

		spin_unlock_irqrestore(&dev->lock, flags);
		status = usb_ep_queue(dev->epout, req, GFP_ATOMIC);
		spin_lock_irqsave(&dev->lock, flags);

		if (status) {
			ERROR(cdev, "rmnet data rx enqueue err %d\n", status);
			list_add_tail(&req->list, pool);
			break;
		}
	}
	spin_unlock_irqrestore(&dev->lock, flags);
}

static void rmnet_smd_data_tx_tlet(unsigned long arg)
{
	struct rmnet_dev *dev = (struct rmnet_dev *) arg;
	struct rmnet_smd_dev *smd_dev = &dev->smd_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	int status;
	int sz;
	unsigned long flags;

	while (1) {
		if (!atomic_read(&dev->online))
			break;
		sz = smd_cur_packet_size(smd_dev->smd_data.ch);
		if (sz == 0)
			break;
		if (smd_read_avail(smd_dev->smd_data.ch) < sz)
			break;

		spin_lock_irqsave(&dev->lock, flags);
		if (list_empty(&smd_dev->tx_idle)) {
			spin_unlock_irqrestore(&dev->lock, flags);
			DBG(cdev, "rmnet data Tx buffers full\n");
			break;
		}
		req = list_first_entry(&smd_dev->tx_idle,
				struct usb_request, list);
		list_del(&req->list);
		spin_unlock_irqrestore(&dev->lock, flags);

		req->length = smd_read(smd_dev->smd_data.ch, req->buf, sz);
		status = usb_ep_queue(dev->epin, req, GFP_ATOMIC);
		if (status) {
			ERROR(cdev, "rmnet tx data enqueue err %d\n", status);
			spin_lock_irqsave(&dev->lock, flags);
			list_add_tail(&req->list, &smd_dev->tx_idle);
			spin_unlock_irqrestore(&dev->lock, flags);
			break;
		}
		dev->dpkts_tolaptop++;
	}

}

static void rmnet_smd_data_rx_tlet(unsigned long arg)
{
	struct rmnet_dev *dev = (struct rmnet_dev *) arg;
	struct rmnet_smd_dev *smd_dev = &dev->smd_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	while (1) {
		if (!atomic_read(&dev->online))
			break;
		if (list_empty(&smd_dev->rx_queue)) {
			atomic_set(&smd_dev->smd_data.rx_pkt, 0);
			break;
		}
		req = list_first_entry(&smd_dev->rx_queue,
			struct usb_request, list);
		if (smd_write_avail(smd_dev->smd_data.ch) < req->actual) {
			atomic_set(&smd_dev->smd_data.rx_pkt, req->actual);
			DBG(cdev, "rmnet SMD data channel full\n");
			break;
		}

		list_del(&req->list);
		spin_unlock_irqrestore(&dev->lock, flags);
		ret = smd_write(smd_dev->smd_data.ch, req->buf, req->actual);
		spin_lock_irqsave(&dev->lock, flags);
		if (ret != req->actual) {
			ERROR(cdev, "rmnet SMD data write failed\n");
			break;
		}
		dev->dpkts_tomsm++;
		list_add_tail(&req->list, &smd_dev->rx_idle);
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	/* We have free rx data requests. */
	rmnet_smd_start_rx(dev);
}

/* If SMD has enough room to accommodate a data rx packet,
 * write into SMD directly. Otherwise enqueue to rx_queue.
 * We will not write into SMD directly untill rx_queue is
 * empty to strictly follow the ordering requests.
 */
static void rmnet_smd_complete_epout(struct usb_ep *ep, struct usb_request *req)
{
	struct rmnet_dev *dev = req->context;
	struct rmnet_smd_dev *smd_dev = &dev->smd_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	int status = req->status;
	int ret;

	if (dev->xport == USB_RMNET_XPORT_UNDEFINED) {
		rmnet_free_req(ep, req);
		return;
	}

	switch (status) {
	case 0:
		/* normal completion */
		break;
	case -ECONNRESET:
	case -ESHUTDOWN:
		/* connection gone */
		spin_lock(&dev->lock);
		list_add_tail(&req->list, &smd_dev->rx_idle);
		spin_unlock(&dev->lock);
		return;
	default:
		/* unexpected failure */
		ERROR(cdev, "RMNET %s response error %d, %d/%d\n",
			ep->name, status,
			req->actual, req->length);
		spin_lock(&dev->lock);
		list_add_tail(&req->list, &smd_dev->rx_idle);
		spin_unlock(&dev->lock);
		return;
	}

	spin_lock(&dev->lock);
	if (!atomic_read(&smd_dev->smd_data.rx_pkt)) {
		if (smd_write_avail(smd_dev->smd_data.ch) < req->actual) {
			atomic_set(&smd_dev->smd_data.rx_pkt, req->actual);
			goto queue_req;
		}
		spin_unlock(&dev->lock);
		ret = smd_write(smd_dev->smd_data.ch, req->buf, req->actual);
		/* This should never happen */
		if (ret != req->actual)
			ERROR(cdev, "rmnet data smd write failed\n");
		/* Restart Rx */
		dev->dpkts_tomsm++;
		spin_lock(&dev->lock);
		list_add_tail(&req->list, &smd_dev->rx_idle);
		spin_unlock(&dev->lock);
		rmnet_smd_start_rx(dev);
		return;
	}
queue_req:
	list_add_tail(&req->list, &smd_dev->rx_queue);
	spin_unlock(&dev->lock);
}

static void rmnet_smd_complete_epin(struct usb_ep *ep, struct usb_request *req)
{
	struct rmnet_dev *dev = req->context;
	struct rmnet_smd_dev *smd_dev = &dev->smd_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	int status = req->status;
	int schedule = 0;

	if (dev->xport == USB_RMNET_XPORT_UNDEFINED) {
		rmnet_free_req(ep, req);
		return;
	}

	switch (status) {
	case -ECONNRESET:
	case -ESHUTDOWN:
		/* connection gone */
		spin_lock(&dev->lock);
		list_add_tail(&req->list, &smd_dev->tx_idle);
		spin_unlock(&dev->lock);
		break;
	default:
		ERROR(cdev, "rmnet data tx ep error %d\n", status);
		/* FALLTHROUGH */
	case 0:
		spin_lock(&dev->lock);
		if (list_empty(&smd_dev->tx_idle))
			schedule = 1;
		list_add_tail(&req->list, &smd_dev->tx_idle);

		if (schedule)
			tasklet_schedule(&smd_dev->smd_data.tx_tlet);
		spin_unlock(&dev->lock);
		break;
	}

}


static void rmnet_smd_notify(void *priv, unsigned event)
{
	struct rmnet_dev *dev = priv;
	struct rmnet_smd_info *smd_info = &dev->smd_dev.smd_data;
	int len = atomic_read(&smd_info->rx_pkt);

	switch (event) {
	case SMD_EVENT_DATA: {
		if (!atomic_read(&dev->online))
			break;
		if (len && (smd_write_avail(smd_info->ch) >= len))
			tasklet_schedule(&smd_info->rx_tlet);

		if (smd_read_avail(smd_info->ch))
			tasklet_schedule(&smd_info->tx_tlet);

		break;
	}
	case SMD_EVENT_OPEN:
		/* usb endpoints are not enabled untill smd channels
		 * are opened. wake up worker thread to continue
		 * connection processing
		 */
		set_bit(CH_OPENED, &smd_info->flags);
		wake_up(&smd_info->wait);
		break;
	case SMD_EVENT_CLOSE:
		/* We will never come here.
		 * reset flags after closing smd channel
		 * */
		clear_bit(CH_OPENED, &smd_info->flags);
		break;
	}
}

static int rmnet_smd_enable(struct rmnet_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct rmnet_smd_dev *smd_dev = &dev->smd_dev;
	int i, ret;
	struct usb_request *req;

	/* Data channel for network packets */
	ret = smd_open(rmnet_smd_data_ch, &smd_dev->smd_data.ch,
			dev, rmnet_smd_notify);
	if (ret) {
		ERROR(cdev, "Unable to open data smd channel\n");
		return ret;
	}

	wait_event(smd_dev->smd_data.wait, test_bit(CH_OPENED,
				&smd_dev->smd_data.flags));

	/* Allocate bulk in/out requests for data transfer.
	 * If the memory allocation fails, all the allocated
	 * requests will be freed upon cable disconnect.
	 */
	for (i = 0; i < RMNET_SMD_RX_REQ_MAX; i++) {
		req = rmnet_alloc_req(dev->epout, RMNET_SMD_RX_REQ_SIZE,
				GFP_KERNEL);
		if (IS_ERR(req))
			return PTR_ERR(req);
		req->length = RMNET_SMD_TXN_MAX;
		req->context = dev;
		req->complete = rmnet_smd_complete_epout;
		list_add_tail(&req->list, &smd_dev->rx_idle);
	}

	for (i = 0; i < RMNET_SMD_TX_REQ_MAX; i++) {
		req = rmnet_alloc_req(dev->epin, RMNET_SMD_TX_REQ_SIZE,
				GFP_KERNEL);
		if (IS_ERR(req))
			return PTR_ERR(req);
		req->context = dev;
		req->complete = rmnet_smd_complete_epin;
		list_add_tail(&req->list, &smd_dev->tx_idle);
	}

	rmnet_smd_start_rx(dev);
	return 0;
}

static void rmnet_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct rmnet_dev *dev = req->context;
	struct usb_composite_dev *cdev = dev->cdev;
	int status = req->status;

	switch (status) {
	case -ECONNRESET:
	case -ESHUTDOWN:
		/* connection gone */
		atomic_set(&dev->notify_count, 0);
		break;
	default:
		ERROR(cdev, "rmnet notifyep error %d\n", status);
		/* FALLTHROUGH */
	case 0:

		/* handle multiple pending QMI_RESPONSE_AVAILABLE
		 * notifications by resending until we're done
		 */
		if (atomic_dec_and_test(&dev->notify_count))
			break;

		status = usb_ep_queue(dev->epnotify, req, GFP_ATOMIC);
		if (status) {
			atomic_dec(&dev->notify_count);
			ERROR(cdev, "rmnet notify ep enq error %d\n", status);
		}
		break;
	}
}

static void qmi_response_available(struct rmnet_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request              *req = dev->notify_req;
	struct usb_cdc_notification     *event = req->buf;
	int status;

	/* Response will be sent later */
	if (atomic_inc_return(&dev->notify_count) != 1)
		return;

	event->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS
			| USB_RECIP_INTERFACE;
	event->bNotificationType = USB_CDC_NOTIFY_RESPONSE_AVAILABLE;
	event->wValue = cpu_to_le16(0);
	event->wIndex = cpu_to_le16(dev->ifc_id);
	event->wLength = cpu_to_le16(0);

	status = usb_ep_queue(dev->epnotify, dev->notify_req, GFP_ATOMIC);
	if (status < 0) {
		atomic_dec(&dev->notify_count);
		ERROR(cdev, "rmnet notify ep enqueue error %d\n", status);
	}
}

#define MAX_CTRL_PKT_SIZE	4096
static void rmnet_sdio_ctl_receive_cb(void *data, int size, void *priv)
{
	struct rmnet_dev *dev = priv;
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	struct rmnet_sdio_qmi_buf *qmi_resp;
	unsigned long flags;

	if (!size || !data)
		return;

	if (size > MAX_CTRL_PKT_SIZE) {
		ERROR(cdev, "ctrl pkt size:%d exceeds max pkt size:%d\n",
				size, MAX_CTRL_PKT_SIZE);
		return;
	}

	if (!atomic_read(&dev->online)) {
		DBG(cdev, "USB disconnected\n");
		return;
	}

	qmi_resp = rmnet_alloc_qmi(size, GFP_KERNEL);
	if (IS_ERR(qmi_resp)) {
		DBG(cdev, "unable to allocate memory for QMI resp\n");
		return;
	}
	memcpy(qmi_resp->buf, data, size);
	qmi_resp->len = size;
	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&qmi_resp->list, &sdio_dev->qmi_resp_q);
	spin_unlock_irqrestore(&dev->lock, flags);

	qmi_response_available(dev);
}

static void rmnet_sdio_ctl_write_done(void *data, int size, void *priv)
{
	struct rmnet_dev *dev = priv;
	struct usb_composite_dev *cdev = dev->cdev;

	VDBG(cdev, "rmnet control write done = %d bytes\n", size);
}

static void rmnet_sdio_sts_callback(int id, void *priv)
{
	struct rmnet_dev *dev = priv;
	struct usb_composite_dev *cdev = dev->cdev;

	DBG(cdev, "rmnet_sts_callback: id: %d\n", id);
}

static void rmnet_sdio_control_rx_work(struct work_struct *w)
{
	struct rmnet_dev *dev = container_of(w, struct rmnet_dev,
			sdio_dev.ctl_rx_work);
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	struct rmnet_sdio_qmi_buf *qmi_req;
	unsigned long flags;
	int ret;

	while (1) {
		spin_lock_irqsave(&dev->lock, flags);
		if (list_empty(&sdio_dev->qmi_req_q))
			goto unlock;

		qmi_req = list_first_entry(&sdio_dev->qmi_req_q,
					struct rmnet_sdio_qmi_buf, list);
		list_del(&qmi_req->list);
		spin_unlock_irqrestore(&dev->lock, flags);

		ret = sdio_cmux_write(rmnet_sdio_ctl_ch, qmi_req->buf,
					qmi_req->len);
		if (ret != qmi_req->len) {
			ERROR(cdev, "rmnet control SDIO write failed\n");
			return;
		}
		dev->cpkts_tomdm++;
		/*
		 * cmux_write API copies the buffer and gives it to sdio_al.
		 * Hence freeing the memory before write is completed.
		 */
		rmnet_free_qmi(qmi_req);
	}
unlock:
	spin_unlock_irqrestore(&dev->lock, flags);
}

static void rmnet_response_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct rmnet_dev *dev = req->context;
	struct usb_composite_dev *cdev = dev->cdev;

	switch (req->status) {
	case -ECONNRESET:
	case -ESHUTDOWN:
	case 0:
		return;
	default:
		INFO(cdev, "rmnet %s response error %d, %d/%d\n",
			ep->name, req->status,
			req->actual, req->length);
	}
}

static void rmnet_command_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct rmnet_dev *dev = req->context;
	struct usb_composite_dev *cdev = dev->cdev;
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct rmnet_sdio_qmi_buf *qmi_req;
	int len = req->actual;

	if (req->status < 0) {
		ERROR(cdev, "rmnet command error %d\n", req->status);
		return;
	}

	qmi_req = rmnet_alloc_qmi(len, GFP_ATOMIC);
	if (IS_ERR(qmi_req)) {
		ERROR(cdev, "unable to allocate memory for QMI req\n");
		return;
	}
	memcpy(qmi_req->buf, req->buf, len);
	qmi_req->len = len;
	spin_lock(&dev->lock);
	list_add_tail(&qmi_req->list, &sdio_dev->qmi_req_q);
	spin_unlock(&dev->lock);
	queue_work(dev->wq, &sdio_dev->ctl_rx_work);
}

static int
rmnet_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct rmnet_dev *dev = container_of(f, struct rmnet_dev, function);
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request      *req = cdev->req;
	int                     ret = -EOPNOTSUPP;
	u16                     w_index = le16_to_cpu(ctrl->wIndex);
	u16                     w_value = le16_to_cpu(ctrl->wValue);
	u16                     w_length = le16_to_cpu(ctrl->wLength);
	struct rmnet_sdio_qmi_buf *resp;

	if (!atomic_read(&sdio_dev->sdio_open))
		return 0;

	if (!atomic_read(&dev->online))
		return -ENOTCONN;

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SEND_ENCAPSULATED_COMMAND:
		if (w_length > req->length)
			goto invalid;
		ret = w_length;
		req->complete = rmnet_command_complete;
		req->context = dev;
		break;


	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_GET_ENCAPSULATED_RESPONSE:
		if (w_value)
			goto invalid;
		else {
			unsigned len;

			spin_lock(&dev->lock);
			if (list_empty(&sdio_dev->qmi_resp_q)) {
				DBG(cdev, "qmi resp queue empty"
					" %02x.%02x v%04x i%04x l%d\n",
					ctrl->bRequestType, ctrl->bRequest,
					w_value, w_index, w_length);
				spin_unlock(&dev->lock);
				goto invalid;
			}

			resp = list_first_entry(&sdio_dev->qmi_resp_q,
				struct rmnet_sdio_qmi_buf, list);
			list_del(&resp->list);
			spin_unlock(&dev->lock);

			len = min_t(unsigned, w_length, resp->len);
			memcpy(req->buf, resp->buf, len);
			ret = len;
			req->complete = rmnet_response_complete;
			req->context = dev;
			rmnet_free_qmi(resp);

			dev->cpkts_tolaptop++;
		}
		break;
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		/* This is a workaround for RmNet and is borrowed from the
		 * CDC/ACM standard. The host driver will issue the above ACM
		 * standard request to the RmNet interface in the following
		 * scenario: Once the network adapter is disabled from device
		 * manager, the above request will be sent from the qcusbnet
		 * host driver, with DTR being '0'. Once network adapter is
		 * enabled from device manager (or during enumeration), the
		 * request will be sent with DTR being '1'.
		 */
		if (w_value & ACM_CTRL_DTR)
			sdio_dev->cbits_to_modem |= TIOCM_DTR;
		else
			sdio_dev->cbits_to_modem &= ~TIOCM_DTR;
		queue_work(dev->wq, &sdio_dev->set_modem_ctl_bits_work);

		ret = 0;

		break;
	default:

invalid:
	DBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
		ctrl->bRequestType, ctrl->bRequest,
		w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (ret >= 0) {
		VDBG(cdev, "rmnet req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = (ret < w_length);
		req->length = ret;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (ret < 0)
			ERROR(cdev, "rmnet ep0 enqueue err %d\n", ret);
	}

	return ret;
}

static void rmnet_free_buf(struct rmnet_dev *dev)
{
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct rmnet_smd_dev *smd_dev = &dev->smd_dev;
	struct rmnet_sdio_qmi_buf *qmi;
	struct usb_request *req;
	struct list_head *pool;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	/* free all usb requests in SDIO tx pool */
	pool = &sdio_dev->tx_idle;
	while (!list_empty(pool)) {
		req = list_first_entry(pool, struct usb_request, list);
		list_del(&req->list);
		req->buf = NULL;
		rmnet_free_req(dev->epout, req);
	}

	pool = &sdio_dev->rx_idle;
	/* free all usb requests in SDIO rx pool */
	while (!list_empty(pool)) {
		req = list_first_entry(pool, struct usb_request, list);
		list_del(&req->list);
		req->buf = NULL;
		rmnet_free_req(dev->epin, req);
	}

	/* free all usb requests in SDIO rx queue */
	pool = &sdio_dev->rx_queue;
	while (!list_empty(pool)) {
		req = list_first_entry(pool, struct usb_request, list);
		list_del(&req->list);
		req->buf = NULL;
		rmnet_free_req(dev->epin, req);
	}

	/* free all usb requests in SMD tx pool */
	pool = &smd_dev->tx_idle;
	while (!list_empty(pool)) {
		req = list_first_entry(pool, struct usb_request, list);
		list_del(&req->list);
		rmnet_free_req(dev->epout, req);
	}

	pool = &smd_dev->rx_idle;
	/* free all usb requests in SMD rx pool */
	while (!list_empty(pool)) {
		req = list_first_entry(pool, struct usb_request, list);
		list_del(&req->list);
		rmnet_free_req(dev->epin, req);
	}

	/* free all usb requests in SMD rx queue */
	pool = &smd_dev->rx_queue;
	while (!list_empty(pool)) {
		req = list_first_entry(pool, struct usb_request, list);
		list_del(&req->list);
		rmnet_free_req(dev->epin, req);
	}

	/* free all buffers in qmi request queue */
	pool = &sdio_dev->qmi_req_q;
	while (!list_empty(pool)) {
		qmi = list_first_entry(pool, struct rmnet_sdio_qmi_buf, list);
		list_del(&qmi->list);
		rmnet_free_qmi(qmi);
	}

	/* free all buffers in qmi response queue */
	pool = &sdio_dev->qmi_resp_q;
	while (!list_empty(pool)) {
		qmi = list_first_entry(pool, struct rmnet_sdio_qmi_buf, list);
		list_del(&qmi->list);
		rmnet_free_qmi(qmi);
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	rmnet_free_req(dev->epnotify, dev->notify_req);
}

static void rmnet_disconnect_work(struct work_struct *w)
{
	struct rmnet_dev *dev = container_of(w, struct rmnet_dev,
			disconnect_work);
	struct rmnet_smd_dev *smd_dev = &dev->smd_dev;

	if (dev->xport == USB_RMNET_XPORT_SMD) {
		tasklet_kill(&smd_dev->smd_data.rx_tlet);
		tasklet_kill(&smd_dev->smd_data.tx_tlet);

		smd_close(smd_dev->smd_data.ch);
		smd_dev->smd_data.flags = 0;
	}

	rmnet_free_buf(dev);
	dev->xport = 0;
}

static void rmnet_disable(struct usb_function *f)
{
	struct rmnet_dev *dev = container_of(f, struct rmnet_dev, function);
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;

	if (!atomic_read(&dev->online))
		return;

	atomic_set(&dev->online, 0);

	usb_ep_fifo_flush(dev->epnotify);
	usb_ep_disable(dev->epnotify);
	usb_ep_fifo_flush(dev->epout);
	usb_ep_disable(dev->epout);

	usb_ep_fifo_flush(dev->epin);
	usb_ep_disable(dev->epin);

	/* cleanup work */
	sdio_dev->cbits_to_modem = 0;
	queue_work(dev->wq, &sdio_dev->set_modem_ctl_bits_work);
	queue_work(dev->wq, &dev->disconnect_work);
}

#define SDIO_OPEN_RETRY_DELAY	msecs_to_jiffies(2000)
#define SDIO_OPEN_MAX_RETRY	90
static void rmnet_open_sdio_work(struct work_struct *w)
{
	struct rmnet_dev *dev =
		container_of(w, struct rmnet_dev, sdio_dev.open_work.work);
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	int ret;
	static int retry_cnt;

	/* Control channel for QMI messages */
	ret = sdio_cmux_open(rmnet_sdio_ctl_ch, rmnet_sdio_ctl_receive_cb,
				rmnet_sdio_ctl_write_done,
				rmnet_sdio_sts_callback, dev);
	if (ret) {
		retry_cnt++;
		pr_debug("%s: usb rmnet sdio open retry_cnt:%d\n",
				__func__, retry_cnt);

		if (retry_cnt > SDIO_OPEN_MAX_RETRY) {
			ERROR(cdev, "Unable to open control SDIO channel\n");
			return;
		}
		queue_delayed_work(dev->wq, &sdio_dev->open_work,
					SDIO_OPEN_RETRY_DELAY);
		return;
	}
	/* Data channel for network packets */
	ret = msm_sdio_dmux_open(rmnet_sdio_data_ch, dev,
				rmnet_sdio_data_receive_cb,
				rmnet_sdio_data_write_done);
	if (ret) {
		ERROR(cdev, "Unable to open SDIO DATA channel\n");
		goto ctl_close;
	}

	sdio_dev->dmux_write_done = 1;
	atomic_set(&sdio_dev->sdio_open, 1);
	pr_info("%s: usb rmnet sdio channels are open retry_cnt:%d\n",
				__func__, retry_cnt);
	retry_cnt = 0;
	return;

ctl_close:
	sdio_cmux_close(rmnet_sdio_ctl_ch);
}

static int rmnet_set_alt(struct usb_function *f,
			unsigned intf, unsigned alt)
{
	struct rmnet_dev *dev = container_of(f, struct rmnet_dev, function);
	struct usb_composite_dev *cdev = dev->cdev;

	/* allocate notification */
	dev->notify_req = rmnet_alloc_req(dev->epnotify,
				RMNET_SDIO_MAX_NFY_SZE, GFP_ATOMIC);

	if (IS_ERR(dev->notify_req))
		return PTR_ERR(dev->notify_req);

	dev->notify_req->complete = rmnet_notify_complete;
	dev->notify_req->context = dev;
	dev->notify_req->length = RMNET_SDIO_MAX_NFY_SZE;
	usb_ep_enable(dev->epnotify, ep_choose(cdev->gadget,
				&rmnet_hs_notify_desc,
				&rmnet_fs_notify_desc));

	dev->epin->driver_data = dev;
	usb_ep_enable(dev->epin, ep_choose(cdev->gadget,
				&rmnet_hs_in_desc,
				&rmnet_fs_in_desc));
	dev->epout->driver_data = dev;
	usb_ep_enable(dev->epout, ep_choose(cdev->gadget,
				&rmnet_hs_out_desc,
				&rmnet_fs_out_desc));

	dev->dpkts_tolaptop = 0;
	dev->cpkts_tolaptop = 0;
	dev->cpkts_tomdm = 0;
	dev->dpkts_tomdm = 0;
	dev->dpkts_tomsm = 0;

	atomic_set(&dev->online, 1);

	return 0;
}

static ssize_t transport_store(
		struct device *device, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct usb_function *f = dev_get_drvdata(device);
	struct rmnet_dev *dev = container_of(f, struct rmnet_dev, function);
	int value;
	enum usb_rmnet_xport_type given_xport;
	enum usb_rmnet_xport_type t;
	struct rmnet_smd_dev *smd_dev = &dev->smd_dev;
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	struct list_head *pool;
	struct sk_buff *skb;
	struct usb_request *req;
	unsigned long flags;

	if (!atomic_read(&dev->online)) {
		pr_err("%s: usb cable is not connected\n", __func__);
		return -EINVAL;
	}

	sscanf(buf, "%d", &value);
	if (value)
		given_xport = USB_RMNET_XPORT_SDIO;
	else
		given_xport = USB_RMNET_XPORT_SMD;

	if (given_xport == dev->xport) {
		pr_err("%s: given_xport:%s cur_xport:%s doing nothing\n",
				__func__, xport_to_str(given_xport),
				xport_to_str(dev->xport));
		return 0;
	}

	pr_debug("usb_rmnet: TransportRequested: %s\n",
			xport_to_str(given_xport));

	/* prevent any other pkts to/from usb  */
	t = dev->xport;
	dev->xport = USB_RMNET_XPORT_UNDEFINED;
	if (t != USB_RMNET_XPORT_UNDEFINED) {
		usb_ep_fifo_flush(dev->epin);
		usb_ep_fifo_flush(dev->epout);
	}

	switch (t) {
	case USB_RMNET_XPORT_SDIO:
		spin_lock_irqsave(&dev->lock, flags);
		/* tx_idle */
		pool = &sdio_dev->tx_idle;
		while (!list_empty(pool)) {
			req = list_first_entry(pool, struct usb_request, list);
			list_del(&req->list);
			req->buf = NULL;
			rmnet_free_req(dev->epout, req);
		}

		/* rx_idle */
		pool = &sdio_dev->rx_idle;
		/* free all usb requests in SDIO rx pool */
		while (!list_empty(pool)) {
			req = list_first_entry(pool, struct usb_request, list);
			list_del(&req->list);
			req->buf = NULL;
			rmnet_free_req(dev->epin, req);
		}

		/* rx_queue */
		pool = &sdio_dev->rx_queue;
		while (!list_empty(pool)) {
			req = list_first_entry(pool, struct usb_request, list);
			list_del(&req->list);
			skb = req->context;
			dev_kfree_skb_any(skb);
			req->buf = NULL;
			rmnet_free_req(dev->epin, req);
		}
		spin_unlock_irqrestore(&dev->lock, flags);
		break;
	case USB_RMNET_XPORT_SMD:
		/* close smd xport */
		tasklet_kill(&smd_dev->smd_data.rx_tlet);
		tasklet_kill(&smd_dev->smd_data.tx_tlet);

		smd_close(smd_dev->smd_data.ch);
		smd_dev->smd_data.flags = 0;

		spin_lock_irqsave(&dev->lock, flags);
		/* free all usb requests in SMD tx pool */
		pool = &smd_dev->tx_idle;
		while (!list_empty(pool)) {
			req = list_first_entry(pool, struct usb_request, list);
			list_del(&req->list);
			rmnet_free_req(dev->epout, req);
		}

		pool = &smd_dev->rx_idle;
		/* free all usb requests in SMD rx pool */
		while (!list_empty(pool)) {
			req = list_first_entry(pool, struct usb_request, list);
			list_del(&req->list);
			rmnet_free_req(dev->epin, req);
		}

		/* free all usb requests in SMD rx queue */
		pool = &smd_dev->rx_queue;
		while (!list_empty(pool)) {
			req = list_first_entry(pool, struct usb_request, list);
			list_del(&req->list);
			rmnet_free_req(dev->epin, req);
		}
		spin_unlock_irqrestore(&dev->lock, flags);
		break;
	default:
		pr_debug("%s: undefined xport, do nothing\n", __func__);
	}

	dev->xport = given_xport;

	switch (dev->xport) {
	case USB_RMNET_XPORT_SDIO:
		rmnet_sdio_enable(dev);
		break;
	case USB_RMNET_XPORT_SMD:
		rmnet_smd_enable(dev);
		break;
	default:
		/* we should never come here */
		pr_err("%s: undefined transport\n", __func__);
	}

	return size;
}
static DEVICE_ATTR(transport, S_IRUGO | S_IWUSR, NULL, transport_store);

static int rmnet_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct rmnet_dev *dev = container_of(f, struct rmnet_dev, function);
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	int id, ret;
	struct usb_ep *ep;

	dev->cdev = cdev;

	/* allocate interface ID */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	dev->ifc_id = id;
	rmnet_interface_desc.bInterfaceNumber = id;

	ep = usb_ep_autoconfig(cdev->gadget, &rmnet_fs_in_desc);
	if (!ep)
		goto out;
	ep->driver_data = cdev; /* claim endpoint */
	dev->epin = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &rmnet_fs_out_desc);
	if (!ep)
		goto out;
	ep->driver_data = cdev; /* claim endpoint */
	dev->epout = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &rmnet_fs_notify_desc);
	if (!ep)
		goto out;
	ep->driver_data = cdev; /* claim endpoint */
	dev->epnotify = ep;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		rmnet_hs_in_desc.bEndpointAddress =
			rmnet_fs_in_desc.bEndpointAddress;
		rmnet_hs_out_desc.bEndpointAddress =
			rmnet_fs_out_desc.bEndpointAddress;
		rmnet_hs_notify_desc.bEndpointAddress =
			rmnet_fs_notify_desc.bEndpointAddress;
	}

	ret = device_create_file(f->dev, &dev_attr_transport);
	if (ret)
		goto out;

	queue_delayed_work(dev->wq, &sdio_dev->open_work, 0);

	return 0;

out:
	if (dev->epnotify)
		dev->epnotify->driver_data = NULL;
	if (dev->epout)
		dev->epout->driver_data = NULL;
	if (dev->epin)
		dev->epin->driver_data = NULL;

	return -ENODEV;
}

static void rmnet_smd_init(struct rmnet_smd_dev *smd_dev)
{
	struct rmnet_dev *dev = container_of(smd_dev,
			struct rmnet_dev, smd_dev);

	atomic_set(&smd_dev->smd_data.rx_pkt, 0);
	tasklet_init(&smd_dev->smd_data.rx_tlet, rmnet_smd_data_rx_tlet,
					(unsigned long) dev);
	tasklet_init(&smd_dev->smd_data.tx_tlet, rmnet_smd_data_tx_tlet,
					(unsigned long) dev);

	init_waitqueue_head(&smd_dev->smd_data.wait);

	INIT_LIST_HEAD(&smd_dev->rx_idle);
	INIT_LIST_HEAD(&smd_dev->rx_queue);
	INIT_LIST_HEAD(&smd_dev->tx_idle);
}

static void rmnet_sdio_init(struct rmnet_sdio_dev *sdio_dev)
{
	INIT_WORK(&sdio_dev->ctl_rx_work, rmnet_sdio_control_rx_work);
	INIT_WORK(&sdio_dev->data_rx_work, rmnet_sdio_data_rx_work);
	INIT_WORK(&sdio_dev->set_modem_ctl_bits_work,
			rmnet_sdio_set_modem_ctl_bits_work);

	INIT_DELAYED_WORK(&sdio_dev->open_work, rmnet_open_sdio_work);

	INIT_LIST_HEAD(&sdio_dev->qmi_req_q);
	INIT_LIST_HEAD(&sdio_dev->qmi_resp_q);

	INIT_LIST_HEAD(&sdio_dev->rx_idle);
	INIT_LIST_HEAD(&sdio_dev->rx_queue);
	INIT_LIST_HEAD(&sdio_dev->tx_idle);
}

static void
rmnet_unbind(struct usb_configuration *c, struct usb_function *f)
{
}

#if defined(CONFIG_DEBUG_FS)
static ssize_t debug_read_stats(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct rmnet_dev *dev = file->private_data;
	struct rmnet_sdio_dev *sdio_dev = &dev->sdio_dev;
	char debug_buf[512];
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dev->lock, flags);
	ret = scnprintf(debug_buf, 512,
			"dpkts_tomsm:  %lu\n"
			"dpkts_tomdm: %lu\n"
			"cpkts_tomdm: %lu\n"
			"dpkts_tolaptop: %lu\n"
			"cpkts_tolaptop:  %lu\n"
			"cbits_to_modem: %lu\n"
			"xport: %s\n",
			dev->dpkts_tomsm, dev->dpkts_tomdm,
			dev->cpkts_tomdm, dev->dpkts_tolaptop,
			dev->cpkts_tolaptop, sdio_dev->cbits_to_modem,
			xport_to_str(dev->xport));

	spin_unlock_irqrestore(&dev->lock, flags);

	return simple_read_from_buffer(ubuf, count, ppos, debug_buf, ret);
}

static ssize_t debug_reset_stats(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct rmnet_dev *dev = file->private_data;

	dev->dpkts_tolaptop = 0;
	dev->cpkts_tolaptop = 0;
	dev->cpkts_tomdm = 0;
	dev->dpkts_tomdm = 0;
	dev->dpkts_tomsm = 0;

	return count;
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

const struct file_operations rmnet_svlte_debug_stats_ops = {
	.open = debug_open,
	.read = debug_read_stats,
	.write = debug_reset_stats,
};

static void usb_debugfs_init(struct rmnet_dev *dev)
{
	struct dentry *dent;

	dent = debugfs_create_dir("usb_rmnet", 0);
	if (IS_ERR(dent))
		return;

	debugfs_create_file("status", 0444, dent, dev,
			&rmnet_svlte_debug_stats_ops);
}
#else
static void usb_debugfs_init(struct rmnet_dev *dev) {}
#endif
static int rmnet_function_add(struct usb_configuration *c)
{
	struct rmnet_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->wq = create_singlethread_workqueue("k_rmnet_work");
	if (!dev->wq) {
		ret = -ENOMEM;
		goto free_dev;
	}

	spin_lock_init(&dev->lock);
	atomic_set(&dev->notify_count, 0);
	atomic_set(&dev->online, 0);
	INIT_WORK(&dev->disconnect_work, rmnet_disconnect_work);
	rmnet_smd_init(&dev->smd_dev);
	rmnet_sdio_init(&dev->sdio_dev);

	dev->function.name = "rmnet_smd_sdio";
	dev->function.strings = rmnet_strings;
	dev->function.descriptors = rmnet_fs_function;
	dev->function.hs_descriptors = rmnet_hs_function;
	dev->function.bind = rmnet_bind;
	dev->function.unbind = rmnet_unbind;
	dev->function.setup = rmnet_setup;
	dev->function.set_alt = rmnet_set_alt;
	dev->function.disable = rmnet_disable;

	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto free_wq;

	usb_debugfs_init(dev);

	return 0;

free_wq:
	destroy_workqueue(dev->wq);
free_dev:
	kfree(dev);

	return ret;
}

#ifdef CONFIG_USB_ANDROID_RMNET_SMD_SDIO
static struct android_usb_function rmnet_function = {
	.name = "rmnet_smd_sdio",
	.bind_config = rmnet_function_add,
};

static int __init rmnet_init(void)
{
	android_register_function(&rmnet_function);
	return 0;
}
module_init(rmnet_init);

#endif /* CONFIG_USB_ANDROID_RMNET_SDIO */
