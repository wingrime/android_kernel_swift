/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <asm/current.h>

#include <mach/peripheral-loader.h>
#include <mach/scm.h>
#include <mach/socinfo.h>
#include <mach/subsystem_notif.h>
#include <mach/subsystem_restart.h>

#include "smd_private.h"

#if defined(SUBSYS_RESTART_DEBUG)
#define dprintk(msg...) printk(msg)
#else
#define dprintk(msg...)
#endif

struct subsys_soc_restart_order {
	const char * const *subsystem_list;
	int count;

	struct mutex shutdown_lock;
	struct mutex powerup_lock;
	struct subsys_data *subsys_ptrs[];
};

struct restart_thread_data {
	struct subsys_data *subsys;
	int coupled;
};

static int restart_level;
static int enable_ramdumps;

static LIST_HEAD(subsystem_list);
static DEFINE_MUTEX(subsystem_list_lock);
static DEFINE_MUTEX(soc_order_reg_lock);

/* SOC specific restart orders go here */

#define DEFINE_SINGLE_RESTART_ORDER(name, order)		\
	static struct subsys_soc_restart_order __##name = {	\
		.subsystem_list = order,			\
		.count = ARRAY_SIZE(order),			\
		.subsys_ptrs = {[ARRAY_SIZE(order)] = NULL}	\
	};							\
	static struct subsys_soc_restart_order *name[] = {      \
		&__##name,					\
	}

/* MSM 8x60 restart ordering info */
static const char * const _order_8x60_all[] = {
	"external_modem",  "modem", "lpass"
};
DEFINE_SINGLE_RESTART_ORDER(orders_8x60_all, _order_8x60_all);

static const char * const _order_8x60_modems[] = {"external_modem", "modem"};
DEFINE_SINGLE_RESTART_ORDER(orders_8x60_modems, _order_8x60_modems);

/* These will be assigned to one of the sets above after
 * runtime SoC identification.
 */
static struct subsys_soc_restart_order **restart_orders;
static int n_restart_orders;

module_param(enable_ramdumps, int, S_IRUGO | S_IWUSR);

static struct subsys_soc_restart_order *_update_restart_order(
		struct subsys_data *subsys);

int get_restart_level()
{
	return restart_level;
}
EXPORT_SYMBOL(get_restart_level);

static void restart_level_changed(void)
{
	struct subsys_data *subsys;

	if (cpu_is_msm8x60() && restart_level == RESET_SUBSYS_COUPLED) {
		restart_orders = orders_8x60_all;
		n_restart_orders = ARRAY_SIZE(orders_8x60_all);
	}

	if (cpu_is_msm8x60() && restart_level == RESET_SUBSYS_MIXED) {
		restart_orders = orders_8x60_modems;
		n_restart_orders = ARRAY_SIZE(orders_8x60_modems);
	}

	mutex_lock(&subsystem_list_lock);
	list_for_each_entry(subsys, &subsystem_list, list)
		subsys->restart_order = _update_restart_order(subsys);
	mutex_unlock(&subsystem_list_lock);
}

static int restart_level_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = restart_level;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	switch (restart_level) {

	case RESET_SOC:
	case RESET_SUBSYS_COUPLED:
	case RESET_SUBSYS_INDEPENDENT:
		pr_info("Subsystem Restart: Phase %d behavior activated.\n",
				restart_level);
	break;

	case RESET_SUBSYS_MIXED:
		pr_info("Subsystem Restart: Phase 2+ behavior activated.\n");
	break;

	default:
		restart_level = old_val;
		return -EINVAL;
	break;

	}

	if (restart_level != old_val)
		restart_level_changed();

	return 0;
}

module_param_call(restart_level, restart_level_set, param_get_int,
			&restart_level, 0644);

static struct subsys_data *_find_subsystem(const char *subsys_name)
{
	struct subsys_data *subsys;

	mutex_lock(&subsystem_list_lock);
	list_for_each_entry(subsys, &subsystem_list, list)
		if (!strncmp(subsys->name, subsys_name,
				SUBSYS_NAME_MAX_LENGTH)) {
			mutex_unlock(&subsystem_list_lock);
			return subsys;
		}
	mutex_unlock(&subsystem_list_lock);

	return NULL;
}

static struct subsys_soc_restart_order *_update_restart_order(
		struct subsys_data *subsys)
{
	int i, j;

	if (!subsys)
		return NULL;

	if (!subsys->name)
		return NULL;

	mutex_lock(&soc_order_reg_lock);
	for (j = 0; j < n_restart_orders; j++) {
		for (i = 0; i < restart_orders[j]->count; i++)
			if (!strncmp(restart_orders[j]->subsystem_list[i],
				subsys->name, SUBSYS_NAME_MAX_LENGTH)) {

					restart_orders[j]->subsys_ptrs[i] =
						subsys;
					mutex_unlock(&soc_order_reg_lock);
					return restart_orders[j];
			}
	}

	mutex_unlock(&soc_order_reg_lock);

	return NULL;
}

static void _send_notification_to_order(struct subsys_data
			**restart_list, int count,
			enum subsys_notif_type notif_type)
{
	int i;

	for (i = 0; i < count; i++)
		if (restart_list[i])
			subsys_notif_queue_notification(
				restart_list[i]->notif_handle, notif_type);
}

static int subsystem_restart_thread(void *data)
{
	struct restart_thread_data *r_work = data;
	struct subsys_data **restart_list;
	struct subsys_data *subsys = r_work->subsys;
	struct subsys_soc_restart_order *soc_restart_order = NULL;

	struct mutex *powerup_lock;
	struct mutex *shutdown_lock;

	int i;
	int restart_list_count = 0;

	if (r_work->coupled)
		soc_restart_order = subsys->restart_order;

	/* It's OK to not take the registration lock at this point.
	 * This is because the subsystem list inside the relevant
	 * restart order is not being traversed.
	 */
	if (!soc_restart_order) {
		restart_list = subsys->single_restart_list;
		restart_list_count = 1;
		powerup_lock = &subsys->powerup_lock;
		shutdown_lock = &subsys->shutdown_lock;
	} else {
		restart_list = soc_restart_order->subsys_ptrs;
		restart_list_count = soc_restart_order->count;
		powerup_lock = &soc_restart_order->powerup_lock;
		shutdown_lock = &soc_restart_order->shutdown_lock;
	}

	dprintk("%s[%p]: Attempting to get shutdown lock!\n", __func__,
		current);

	/* Try to acquire shutdown_lock. If this fails, these subsystems are
	 * already being restarted - return.
	 */
	if (!mutex_trylock(shutdown_lock)) {
		kfree(data);
		do_exit(0);
	}

	dprintk("%s[%p]: Attempting to get powerup lock!\n", __func__,
			current);

	/* Now that we've acquired the shutdown lock, either we're the first to
	 * restart these subsystems or some other thread is doing the powerup
	 * sequence for these subsystems. In the latter case, panic and bail
	 * out, since a subsystem died in its powerup sequence.
	 */
	if (!mutex_trylock(powerup_lock))
		panic("%s: Subsystem died during powerup!", __func__);

	/* Now it is necessary to take the registration lock. This is because
	 * the subsystem list in the SoC restart order will be traversed
	 * and it shouldn't be changed until _this_ restart sequence completes.
	 */
	mutex_lock(&soc_order_reg_lock);

	dprintk("%s: Starting restart sequence for %s\n", __func__,
		r_work->subsys->name);

	_send_notification_to_order(restart_list,
				restart_list_count,
				SUBSYS_BEFORE_SHUTDOWN);

	for (i = 0; i < restart_list_count; i++) {

		if (!restart_list[i])
			continue;

		pr_info("subsys-restart: Shutting down %s\n",
			restart_list[i]->name);

		if (restart_list[i]->shutdown(subsys->name) < 0)
			panic("%s: Failed to shutdown %s!\n", __func__,
				restart_list[i]->name);
	}

	_send_notification_to_order(restart_list, restart_list_count,
				SUBSYS_AFTER_SHUTDOWN);

	/* Now that we've finished shutting down these subsystems, release the
	 * shutdown lock. If a subsystem restart request comes in for a
	 * subsystem in _this_ restart order after the unlock below, and
	 * before the powerup lock is released, panic and bail out.
	 */
	mutex_unlock(shutdown_lock);

	/* Collect ram dumps for all subsystems in order here */
	for (i = 0; i < restart_list_count; i++) {
		if (!restart_list[i])
			continue;

		if (restart_list[i]->ramdump)
			if (restart_list[i]->ramdump(enable_ramdumps,
							subsys->name) < 0)
				pr_warn("%s(%s): Ramdump failed.", __func__,
					restart_list[i]->name);
	}

	_send_notification_to_order(restart_list,
			restart_list_count,
			SUBSYS_BEFORE_POWERUP);

	for (i = restart_list_count - 1; i >= 0; i--) {

		if (!restart_list[i])
			continue;

		pr_info("subsys-restart: Powering up %s\n",
			restart_list[i]->name);

		if (restart_list[i]->powerup(subsys->name) < 0)
			panic("%s: Failed to powerup %s!", __func__,
				restart_list[i]->name);
	}

	_send_notification_to_order(restart_list,
				restart_list_count,
				SUBSYS_AFTER_POWERUP);

	pr_info("%s: Restart sequence for %s completed.", __func__,
			r_work->subsys->name);

	mutex_unlock(powerup_lock);

	mutex_unlock(&soc_order_reg_lock);

	dprintk("%s: Released powerup lock!\n", __func__);

	kfree(data);
	do_exit(0);
}

int subsystem_restart(const char *subsys_name)
{
	struct subsys_data *subsys;
	struct task_struct *tsk;
	struct restart_thread_data *data = NULL;

	if (!subsys_name) {
		pr_err("%s: Invalid subsystem name.", __func__);
		return -EINVAL;
	}

	pr_info("Subsystem Restart: Restart sequence requested for  %s\n",
		subsys_name);

	/* List of subsystems is protected by a lock. New subsystems can
	 * still come in.
	 */
	subsys = _find_subsystem(subsys_name);

	if (!subsys) {
		pr_warn("%s: Unregistered subsystem %s!", __func__,
				subsys_name);
		return -EINVAL;
	}

	if (restart_level != RESET_SOC) {
		data = kzalloc(sizeof(struct restart_thread_data), GFP_KERNEL);
		if (!data) {
			restart_level = RESET_SOC;
			pr_warn("%s: Failed to alloc restart data. Resetting.",
				__func__);
		} else {
			if (restart_level == RESET_SUBSYS_COUPLED ||
					restart_level == RESET_SUBSYS_MIXED)
				data->coupled = 1;
			else
				data->coupled = 0;

			data->subsys = subsys;
		}
	}

	switch (restart_level) {

	case RESET_SUBSYS_COUPLED:
	case RESET_SUBSYS_MIXED:
	case RESET_SUBSYS_INDEPENDENT:
		dprintk("%s: Restarting %s [level=%d]!\n", __func__,
			subsys_name, restart_level);

		/* Let the kthread handle the actual restarting. Using a
		 * workqueue will not work since all restart requests are
		 * serialized and it prevents the short circuiting of
		 * restart requests for subsystems already in a restart
		 * sequence.
		 */
		tsk = kthread_run(subsystem_restart_thread, data,
				"subsystem_subsystem_restart_thread");
		if (IS_ERR(tsk))
			panic("%s: Unable to create thread to restart %s",
				__func__, subsys->name);

		break;

	case RESET_SOC:

		mutex_lock(&subsystem_list_lock);
		list_for_each_entry(subsys, &subsystem_list, list)
			if (subsys->crash_shutdown)
				subsys->crash_shutdown(subsys->name);
		mutex_unlock(&subsystem_list_lock);

		panic("Resetting the SOC");
		break;

	default:
		panic("subsys-restart: Unknown restart level!\n");
	break;

	}

	return 0;
}
EXPORT_SYMBOL(subsystem_restart);

int ssr_register_subsystem(struct subsys_data *subsys)
{
	if (!subsys)
		goto err;

	if (!subsys->name)
		goto err;

	if (!subsys->powerup || !subsys->shutdown)
		goto err;

	subsys->notif_handle = subsys_notif_add_subsys(subsys->name);
	subsys->restart_order = _update_restart_order(subsys);
	subsys->single_restart_list[0] = subsys;

	mutex_init(&subsys->shutdown_lock);
	mutex_init(&subsys->powerup_lock);

	mutex_lock(&subsystem_list_lock);
	list_add(&subsys->list, &subsystem_list);
	mutex_unlock(&subsystem_list_lock);

	return 0;

err:
	return -EINVAL;
}
EXPORT_SYMBOL(ssr_register_subsystem);

static int __init ssr_init_soc_restart_orders(void)
{
	int i;

	if (cpu_is_msm8x60()) {
		for (i = 0; i < ARRAY_SIZE(orders_8x60_all); i++) {
			mutex_init(&orders_8x60_all[i]->powerup_lock);
			mutex_init(&orders_8x60_all[i]->shutdown_lock);
		}

		for (i = 0; i < ARRAY_SIZE(orders_8x60_modems); i++) {
			mutex_init(&orders_8x60_modems[i]->powerup_lock);
			mutex_init(&orders_8x60_modems[i]->shutdown_lock);
		}

		restart_orders = orders_8x60_all;
		n_restart_orders = ARRAY_SIZE(orders_8x60_all);
	}

	if (restart_orders == NULL || n_restart_orders < 1) {
		WARN_ON(1);
		return -EINVAL;
	}

	return 0;
}

static int __init subsys_restart_init(void)
{
	int ret = 0;

	restart_level = RESET_SOC;

	ret = ssr_init_soc_restart_orders();

	return ret;
}

arch_initcall(subsys_restart_init);

MODULE_DESCRIPTION("Subsystem Restart Driver");
MODULE_LICENSE("GPL v2");
