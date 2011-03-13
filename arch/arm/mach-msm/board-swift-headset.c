/*
 *  arch/arm/mach-msm/lge_headset.c
 *
 *  LGE 3.5 PI Headset detection driver.
 *  HSD is HeadSet Detection with one GPIO
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * Copyright (C) 2009 ~ 2010 LGE, Inc.
 * Author: Lee SungYoung < lsy@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

/* The Android Donut's Headset Detection Interface is following;
 * source file is android/frameworks/base/services/java/com/android/server/HeadsetObserver.java
 * HEADSET_UEVENT_MATCH = "DEVPATH=/sys/devices/virtual/switch/h2w"
 * HEADSET_STATE_PATH = /sys/class/switch/h2w/state
 * HEADSET_NAME_PATH = /sys/class/switch/h2w/name
 * INPUT = SW_HEADPHONE_INSERT
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <mach/board.h>

#undef  LGE_HSD_DEBUG_PRINT
#define LGE_HSD_DEBUG_PRINT
#undef  LGE_HSD_ERROR_PRINT
#define LGE_HSD_ERROR_PRINT

#if defined(LGE_HSD_DEBUG_PRINT)
#define HSD_DBG(fmt, args...) printk(KERN_INFO "HSD[%-18s:%5d]" fmt, __func__,__LINE__, ## args) 
#else
#define HSD_DBG(fmt, args...) do {} while(0)
#endif

#if defined(LGE_HSD_ERROR_PRINT)
#define HSD_ERR(fmt, args...) printk(KERN_ERR "HSD[%-18s:%5d]" fmt, __func__,__LINE__, ## args) 
#else
#define HSD_ERR(fmt, args...) do {} while(0)
#endif

	/* GPIO TLMM (Top Level Multiplexing) Definitions */

	/* GPIO TLMM: Function -- GPIO specific */
	
	/* GPIO TLMM: Direction */
	enum {
		GPIO_INPUT,
		GPIO_OUTPUT,
	};

	/* GPIO TLMM: Pullup/Pulldown */
	enum {
		GPIO_NO_PULL,
		GPIO_PULL_DOWN,
		GPIO_KEEPER,
		GPIO_PULL_UP,
	};
	
	/* GPIO TLMM: Drive Strength */
	enum {
		GPIO_2MA,
		GPIO_4MA,
		GPIO_6MA,
		GPIO_8MA,
		GPIO_10MA,
		GPIO_12MA,
		GPIO_14MA,
		GPIO_16MA,
	};
	
	enum {
		GPIO_ENABLE,
		GPIO_DISABLE,
	};
	
	#define GPIO_CFG(gpio, func, dir, pull, drvstr) \
		((((gpio) & 0x3FF) << 4)        |	  \
		 ((func) & 0xf)                  |	  \
		 (((dir) & 0x1) << 14)           |	  \
		 (((pull) & 0x3) << 15)          |	  \
		 (((drvstr) & 0xF) << 17))	
		 
static struct workqueue_struct *hs_detect_work_queue;
static void detect_work(struct work_struct *work);
static DECLARE_WORK(hs_detect_work, detect_work);

struct hsd_info {
   struct switch_dev sdev;
   struct input_dev *input;
   struct mutex mutex_lock;
   const char *name_on;
   const char *name_off;
   const char *state_on;
   const char *state_off;
   unsigned gpio;
   int irq;
//   struct hrtimer timer;
//   ktime_t debounce_time;
   struct work_struct work;
};

static struct hsd_info *hi;

enum {
   NO_DEVICE   = 0,
   LGE_HEADSET = 1,
};

#define LGE_HEADSET_DETECT_GPIO  29

static ssize_t lge_hsd_print_name(struct switch_dev *sdev, char *buf)
{
   switch (switch_get_state(&hi->sdev)) {
   case NO_DEVICE:
      return sprintf(buf, "No Device\n");
   case LGE_HEADSET:
      return sprintf(buf, "Headset\n");
   }
   return -EINVAL;
}

static ssize_t lge_hsd_print_state(struct switch_dev *sdev, char *buf)
{
	const char *state;

	if (switch_get_state(&hi->sdev))
		state = hi->state_on;
	else
		state = hi->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

unsigned char headset_inserted = 0;
	
static void insert_headset(void)
{
headset_inserted = 1;
	input_report_switch(hi->input, SW_HEADPHONE_INSERT, 1);
	input_sync(hi->input);

//   hi->debounce_time = ktime_set(0, 20000000);  /* 20 ms */

   mutex_lock(&hi->mutex_lock);
   switch_set_state(&hi->sdev, LGE_HEADSET);
   mutex_unlock(&hi->mutex_lock);
}

static void remove_headset(void)
{
headset_inserted = 0;   
mutex_lock(&hi->mutex_lock);
   switch_set_state(&hi->sdev, NO_DEVICE);
   mutex_unlock(&hi->mutex_lock);

	input_report_switch(hi->input, SW_HEADPHONE_INSERT, 0);
	input_sync(hi->input);

//   hi->debounce_time = ktime_set(0, 100000000);  /* 100 ms */
}

static void detect_work(struct work_struct *work)
{
   int state;
   unsigned long irq_flags;

   local_irq_save(irq_flags);
   disable_irq(hi->irq);
   local_irq_restore(irq_flags);

   state = gpio_get_value(hi->gpio);

	HSD_DBG("hs:%d\n", state);
   
   local_irq_save(irq_flags);
   enable_irq(hi->irq);
   local_irq_restore(irq_flags);

   if (state != 1) {
      if (switch_get_state(&hi->sdev) == LGE_HEADSET) {
         HSD_DBG("==== LGE headset removing\n");
         remove_headset();
      }
      return;
   }

   if (state == 1) {
      if (switch_get_state(&hi->sdev) == NO_DEVICE) {
         HSD_DBG("==== LGE headset inserting\n");
         insert_headset();
      }
   }
   else {
      HSD_ERR("Invalid state\n");
   }
}
#if 0
static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
   queue_work(hs_detect_work_queue, &hs_detect_work);
   return HRTIMER_NORESTART;
}
#endif
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
   int value1, value2;
   int retry_limit = 10;

   value1 = gpio_get_value(hi->gpio);
   do {
      value2 = gpio_get_value(hi->gpio);
   } while(retry_limit-- < 0);
   if (value1 ^ value2)
      return IRQ_HANDLED;

   HSD_DBG("value2 = %d\n", value2);

   if ((switch_get_state(&hi->sdev) ^ value2)) {
//      hrtimer_start(&hi->timer, hi->debounce_time, HRTIMER_MODE_REL);
   	queue_work(hs_detect_work_queue, &hs_detect_work);
   }

	return IRQ_HANDLED;
}

#if defined(CONFIG_DEBUG_FS)
static int hsd_debug_set(void *data, u64 val)
{
   mutex_lock(&hi->mutex_lock);
   switch_set_state(&hi->sdev, (int)val);
   mutex_unlock(&hi->mutex_lock);
   return 0;
}

static int hsd_debug_get(void *data, u64 *val)
{
   return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(hsd_debug_fops, hsd_debug_get, hsd_debug_set, "%llu\n");
static int __init hsd_debug_init(void)
{
   struct dentry *dent;

   dent = debugfs_create_dir("hsd", 0);
   if (IS_ERR(dent)) {
      HSD_ERR("Fail to create debug fs directory\n");
      return PTR_ERR(dent);
   }

   debugfs_create_file("state", 0644, dent, NULL, &hsd_debug_fops);

   return 0;
}

device_initcall(hsd_debug_init);
#endif

static int lge_hsd_probe(struct platform_device *pdev)
{
   int ret;
   struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;

	HSD_DBG("%s\n", pdata->name);

	if (!pdata) {
      HSD_ERR("The platform data is null\n");
		return -EBUSY;
   }

   hi = kzalloc(sizeof(struct hsd_info), GFP_KERNEL);
   if (!hi) {
      HSD_ERR("Failed to allloate headset per device info\n");
      return -ENOMEM;
   }

//   hi->debounce_time = ktime_set(0, 100000000); /* 100 ms */
   hi->gpio = pdata->gpio;

   mutex_init(&hi->mutex_lock);
#if 0
   hi->name_on = pdata->name_on;
   hi->name_off = pdata->name_off;
   hi->state_on = pdata->state_on;
   hi->state_off = pdata->state_off;
#endif

   hi->sdev.name = pdata->name;
   hi->sdev.print_state = lge_hsd_print_state;
   hi->sdev.print_name = lge_hsd_print_name;

   ret = switch_dev_register(&hi->sdev);
	if (ret < 0) {
      HSD_ERR("Failed to register switch device\n");
		goto err_switch_dev_register;
   }

   hs_detect_work_queue = create_workqueue("hs_detect");
   if (hs_detect_work_queue == NULL) {
      HSD_ERR("Failed to create workqueue\n");
      goto err_create_work_queue;
   }

   ret = gpio_request(hi->gpio, pdev->name);
   if (ret < 0) {
      HSD_ERR("Failed to request gpio%d\n", hi->gpio);
      goto err_request_detect_gpio;
   }

   ret = gpio_direction_input(hi->gpio); 
   if (ret < 0) {
      HSD_ERR("Failed to set gpio%d as input\n", hi->gpio);
      goto err_set_detect_gpio;
   }

   if (hi->gpio == LGE_HEADSET_DETECT_GPIO) {
      ret = gpio_tlmm_config(GPIO_CFG(hi->gpio, 0, GPIO_INPUT, GPIO_NO_PULL, 
               GPIO_2MA), GPIO_ENABLE);
      if (ret < 0) {
         HSD_ERR("Failed to configure gpio%d tlmm\n", hi->gpio);
         goto err_set_detect_gpio;
      }
   }

   hi->irq = gpio_to_irq(pdata->gpio);
   if (hi->irq < 0) {
      HSD_ERR("Failed to get interrupt number\n");
      ret = hi->irq;
      goto err_get_irq_num_failed;
   }

//   hrtimer_init(&hi->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
//   hi->timer.function = detect_event_timer_func;

	ret = request_irq(hi->irq, gpio_irq_handler, 
           IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, pdev->name, NULL);

	if (ret < 0) {
      HSD_ERR("Failed to request interrupt handler\n");
		goto err_request_detect_irq;
   }

	ret = set_irq_wake(hi->irq, 1);
   if (ret < 0) {
      HSD_ERR("Failed to set interrupt wake\n");
      goto err_request_input_dev;
   }

	hi->input = input_allocate_device();
	if (!hi->input) {
      HSD_ERR("Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	if (pdev->dev.platform_data)
			hi->input->name = "7k_headset";
	else
			hi->input->name = "hsd_headset";

	hi->input->id.vendor	= 0x0001;
	hi->input->id.product	= 1;
	hi->input->id.version	= 1;

	input_set_capability(hi->input, EV_SW, SW_HEADPHONE_INSERT);

	ret = input_register_device(hi->input);
	if (ret) {
         HSD_ERR("Failed to register input device\n");
			goto err_register_input_dev;
	}

	/* Perform initial detection */
//	gpio_switch_work(&switch_data->work);

	return 0;

err_register_input_dev:
   input_free_device(hi->input);
err_request_input_dev:
   free_irq(hi->irq, 0);
err_request_detect_irq:
err_get_irq_num_failed:
err_set_detect_gpio:
	gpio_free(hi->gpio);
err_request_detect_gpio:
   destroy_workqueue(hs_detect_work_queue);
err_create_work_queue:
   switch_dev_unregister(&hi->sdev);
err_switch_dev_register:
   HSD_ERR("Failed to register driver\n");

	return ret;
}

static int lge_hsd_remove(struct platform_device *pdev)
{
//   H2WD("");
   if (switch_get_state(&hi->sdev))
      remove_headset();
   
   input_unregister_device(hi->input);
   gpio_free(hi->gpio);
   free_irq(hi->irq, 0);
   destroy_workqueue(hs_detect_work_queue);
   switch_dev_unregister(&hi->sdev);

	return 0;
}

static struct gpio_switch_platform_data lge_hs_pdata = {
   .name = "h2w",
   .gpio = LGE_HEADSET_DETECT_GPIO,
};

static struct platform_device lge_hsd_device = {
   .name = "lge-h2w",
   .id   = -1,
   .dev = {
      .platform_data = &lge_hs_pdata,
   },
};

static struct platform_driver lge_hsd_driver = {
	.probe		= lge_hsd_probe,
	.remove		= lge_hsd_remove,
	.driver		= {
		.name	= "lge-h2w",
		.owner	= THIS_MODULE,
	},
};

static int __init lge_hsd_init(void)
{
   int ret;

   if (!machine_is_msm7x27_swift())
      return 0;

   HSD_DBG("");
   ret = platform_driver_register(&lge_hsd_driver);
   if (ret) {
      HSD_ERR("Fail to register platform driver\n");
      return ret;
   }

	return platform_device_register(&lge_hsd_device);
}

static void __exit lge_hsd_exit(void)
{
   platform_device_unregister(&lge_hsd_device);
	platform_driver_unregister(&lge_hsd_driver);
}

module_init(lge_hsd_init);
module_exit(lge_hsd_exit);

MODULE_AUTHOR("Lee SungYoung <lsy@lge.com>");
MODULE_DESCRIPTION("LGE Headset detection driver");
MODULE_LICENSE("GPL");
