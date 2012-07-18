/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
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
 *
 */
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>

#include <mach/msm_bus.h>

#include "kgsl.h"
#include "kgsl_g12.h"
#include "kgsl_log.h"
#include "kgsl_g12_drawctxt.h"
#include "kgsl_g12_cmdstream.h"
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_sharedmem.h"
#include "kgsl_g12_vgv3types.h"
#include "kgsl_cffdump.h"

#include "g12_reg.h"

#define GSL_VGC_INT_MASK \
	 (REG_VGC_IRQSTATUS__MH_MASK | \
	  REG_VGC_IRQSTATUS__G2D_MASK | \
	  REG_VGC_IRQSTATUS__FIFO_MASK)

/* G12 MH arbiter config*/
#define KGSL_G12_CFG_G12_MHARB \
	(0x10 \
		| (0 << MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT) \
		| (0x8 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT))

#define KGSL_G12_TIMESTAMP_EPSILON 20000
#define KGSL_G12_IDLE_COUNT_MAX 1000000

static int kgsl_g12_start(struct kgsl_device *device, unsigned int init_ram);
static int kgsl_g12_stop(struct kgsl_device *device);
static int kgsl_g12_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs);

#define KGSL_2D_MMU_CONFIG					     \
	(0x01							     \
	| (MMU_CONFIG << MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT)   \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT)   \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT)  \
	| (MMU_CONFIG << MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT) \
	| (MMU_CONFIG << MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT) \
	| (MMU_CONFIG << MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT)   \
	| (MMU_CONFIG << MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT))

static struct kgsl_g12_device device_2d0 = {
	.dev = {
		.name = "kgsl-2d0",
		.id = KGSL_DEVICE_2D0,
		.mmu = {
			.config = KGSL_2D_MMU_CONFIG,
			/* turn off memory protection unit by setting
			   acceptable physical address range to include
			   all pages. */
			.mpu_base = 0x00000000,
			.mpu_range =  0xFFFFF000,
			/* va_range and va_base are set by the platform file */
			.reg = {
				.config = ADDR_MH_MMU_CONFIG,
				.mpu_base = ADDR_MH_MMU_MPU_BASE,
				.mpu_end = ADDR_MH_MMU_MPU_END,
				.va_range = ADDR_MH_MMU_VA_RANGE,
				.pt_page = ADDR_MH_MMU_PT_BASE,
				.page_fault = ADDR_MH_MMU_PAGE_FAULT,
				.tran_error = ADDR_MH_MMU_TRAN_ERROR,
				.invalidate = ADDR_MH_MMU_INVALIDATE,
				.interrupt_mask = ADDR_MH_INTERRUPT_MASK,
				.interrupt_status = ADDR_MH_INTERRUPT_STATUS,
				.interrupt_clear = ADDR_MH_INTERRUPT_CLEAR
			},
		},
		.mutex = __MUTEX_INITIALIZER(device_2d0.dev.mutex),
		.state = KGSL_STATE_INIT,
		.active_cnt = 0,
	},
	.iomemname = "kgsl_2d0_reg_memory",
	.irqname = "kgsl_2d0_irq",
	.regulator = "fs_gfx2d0",
};

static struct kgsl_g12_device device_2d1 = {
	.dev = {
		.name = "kgsl-2d1",
		.id = KGSL_DEVICE_2D1,
		.mmu = {
			.config = KGSL_2D_MMU_CONFIG,
			/* turn off memory protection unit by setting
			   acceptable physical address range to include
			   all pages. */
			.mpu_base = 0x00000000,
			.mpu_range =  0xFFFFF000,
			/* These might be better set from the platform
			   device */
			.va_base = 0x66000000,
			/* va_range is set by the platform driver */
			.reg = {
				.config = ADDR_MH_MMU_CONFIG,
				.mpu_base = ADDR_MH_MMU_MPU_BASE,
				.mpu_end = ADDR_MH_MMU_MPU_END,
				.va_range = ADDR_MH_MMU_VA_RANGE,
				.pt_page = ADDR_MH_MMU_PT_BASE,
				.page_fault = ADDR_MH_MMU_PAGE_FAULT,
				.tran_error = ADDR_MH_MMU_TRAN_ERROR,
				.invalidate = ADDR_MH_MMU_INVALIDATE,
				.interrupt_mask = ADDR_MH_INTERRUPT_MASK,
				.interrupt_status = ADDR_MH_INTERRUPT_STATUS,
				.interrupt_clear = ADDR_MH_INTERRUPT_CLEAR
			},
		},
		.mutex = __MUTEX_INITIALIZER(device_2d1.dev.mutex),
		.state = KGSL_STATE_INIT,
		.active_cnt = 0,
	},
	.iomemname = "kgsl_2d1_reg_memory",
	.irqname = "kgsl_2d1_irq",
	.regulator = "fs_gfx2d1",
};

irqreturn_t kgsl_g12_isr(int irq, void *data)
{
	irqreturn_t result = IRQ_NONE;
	unsigned int status;
	struct kgsl_device *device = (struct kgsl_device *) data;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	kgsl_g12_regread(device, ADDR_VGC_IRQSTATUS >> 2, &status);

	if (status & GSL_VGC_INT_MASK) {
		kgsl_g12_regwrite(device,
			ADDR_VGC_IRQSTATUS >> 2, status & GSL_VGC_INT_MASK);

		result = IRQ_HANDLED;

		if (status & REG_VGC_IRQSTATUS__FIFO_MASK)
			KGSL_DRV_ERR("g12 fifo interrupt\n");
		if (status & REG_VGC_IRQSTATUS__MH_MASK)
			kgsl_mh_intrcallback(device);
		if (status & REG_VGC_IRQSTATUS__G2D_MASK) {
			int count;

			KGSL_DRV_VDBG("g12 g2d interrupt\n");
			kgsl_g12_regread(device,
					 ADDR_VGC_IRQ_ACTIVE_CNT >> 2,
					 &count);

			count >>= 8;
			count &= 255;
			g12_device->timestamp += count;

			wake_up_interruptible(&(g12_device->wait_timestamp_wq));

			atomic_notifier_call_chain(
				&(device->ts_notifier_list),
				device->id, NULL);
		}
	}

	if (device->pwrctrl.nap_allowed == true) {
		device->requested_state = KGSL_STATE_NAP;
		schedule_work(&device->idle_check_ws);
	}
	mod_timer(&device->idle_timer,
			jiffies + device->pwrctrl.interval_timeout);

	return result;
}

static int kgsl_g12_cleanup_pt(struct kgsl_device *device,
			       struct kgsl_pagetable *pagetable)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	if (device->mmu.defaultpagetable == pagetable)
		device->mmu.defaultpagetable = NULL;

	kgsl_mmu_unmap(pagetable, device->mmu.dummyspace.gpuaddr,
			device->mmu.dummyspace.size);

	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);

	kgsl_mmu_unmap(pagetable, g12_device->ringbuffer.cmdbufdesc.gpuaddr,
			g12_device->ringbuffer.cmdbufdesc.size);
	return 0;
}

static int kgsl_g12_setup_pt(struct kgsl_device *device,
			     struct kgsl_pagetable *pagetable)
{
	int result = 0;
	unsigned int flags = KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN4K;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	if (device->mmu.defaultpagetable == NULL)
		device->mmu.defaultpagetable = pagetable;

	result = kgsl_mmu_map_global(pagetable, &device->mmu.dummyspace,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto error;

	result = kgsl_mmu_map_global(pagetable, &device->memstore,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto error_unmap_dummy;

	result = kgsl_mmu_map_global(pagetable,
				     &g12_device->ringbuffer.cmdbufdesc,
				     GSL_PT_PAGE_RV, flags);
	if (result)
		goto error_unmap_memstore;
	return result;

error_unmap_dummy:
	kgsl_mmu_unmap(pagetable, device->mmu.dummyspace.gpuaddr,
			device->mmu.dummyspace.size);
error_unmap_memstore:
	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);
error:
	return result;
}

int kgsl_g12_setstate(struct kgsl_device *device, uint32_t flags)
{
#ifdef CONFIG_MSM_KGSL_MMU
	unsigned int mh_mmu_invalidate = 0x00000003; /*invalidate all and tc */

	if (flags & KGSL_MMUFLAGS_PTUPDATE) {
		kgsl_g12_idle(device, KGSL_TIMEOUT_DEFAULT);
		kgsl_g12_regwrite(device, ADDR_MH_MMU_PT_BASE,
				     device->mmu.hwpagetable->base.gpuaddr);
		kgsl_g12_regwrite(device, ADDR_MH_MMU_VA_RANGE,
				     (device->mmu.hwpagetable->
				      va_base | (device->mmu.hwpagetable->
						 va_range >> 16)));
		kgsl_g12_regwrite(device, ADDR_MH_MMU_INVALIDATE,
				     mh_mmu_invalidate);
	}

	if (flags & KGSL_MMUFLAGS_TLBFLUSH)
		kgsl_g12_regwrite(device, ADDR_MH_MMU_INVALIDATE,
			     mh_mmu_invalidate);
#endif
	return 0;
}

static int __init
kgsl_g12_init_pwrctrl(struct kgsl_device *device,
		      struct platform_device *pdev)
{
	int i, result = 0;
	const char *pclk_name;
	struct clk *clk, *pclk;
	struct kgsl_platform_data *pdata = pdev->dev.platform_data;
	struct kgsl_device_platform_data *pdata_dev = NULL;
	struct kgsl_device_pwr_data *pdata_pwr = NULL;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	struct msm_bus_scale_pdata *bus_table = NULL;

	if (device->id == KGSL_DEVICE_2D0)
		pdata_dev = pdata->dev_2d0;
	else
		pdata_dev = pdata->dev_2d1;

	pdata_pwr = &pdata_dev->pwr_data;

	clk = clk_get(&pdev->dev, pdata_dev->clk.name.clk);
	pclk = clk_get(&pdev->dev, pdata_dev->clk.name.pclk);
	pclk_name = pdata_dev->clk.name.pclk;
	bus_table = pdata_dev->clk.bus_scale_table;

	/* error check resources */
	if (IS_ERR(clk)) {
		clk = NULL;
		result = PTR_ERR(clk);
		KGSL_DRV_ERR("clk_get(%s) returned %d\n",
				pdata_dev->clk.name.clk, result);
		goto done;
	}

	if (pclk_name && IS_ERR(pclk)) {
		pclk = NULL;
		result = PTR_ERR(pclk);
		KGSL_DRV_ERR("clk_get(%s) returned %d\n",
					 pclk_name, result);
		goto done;
	}

	device->pwrctrl.gpu_reg = regulator_get(NULL, g12_device->regulator);

	if (IS_ERR(device->pwrctrl.gpu_reg))
		device->pwrctrl.gpu_reg = NULL;

	device->pwrctrl.interrupt_num =
		platform_get_irq_byname(pdev, g12_device->irqname);

	if (device->pwrctrl.interrupt_num <= 0) {
		KGSL_DRV_ERR("platform_get_irq_byname() returned %d\n",
					 device->pwrctrl.interrupt_num);
		result = -EINVAL;
		goto done;
	}

	/* save resources to pwrctrl struct */
	if (pdata_pwr->set_grp_async != NULL)
		pdata_pwr->set_grp_async();

	if (pdata_pwr->num_levels > KGSL_MAX_PWRLEVELS) {
		result = -EINVAL;
		goto done;
	}
	device->pwrctrl.num_pwrlevels = pdata_pwr->num_levels;
	device->pwrctrl.active_pwrlevel = pdata_pwr->init_level;
	for (i = 0; i < pdata_pwr->num_levels; i++) {
		device->pwrctrl.pwrlevels[i].gpu_freq =
			(pdata_pwr->pwrlevel[i].gpu_freq > 0) ?
			clk_round_rate(clk,
			pdata_pwr->pwrlevel[i].gpu_freq) : 0;
		device->pwrctrl.pwrlevels[i].bus_freq =
			pdata_pwr->pwrlevel[i].bus_freq;
	}
	/* Do not set_rate for cores in sync with AXI. */
	if (pdata_pwr->pwrlevel[0].gpu_freq > 0)
		clk_set_rate(clk, device->pwrctrl.
			pwrlevels[KGSL_DEFAULT_PWRLEVEL]. gpu_freq);

	device->pwrctrl.power_flags = KGSL_PWRFLAGS_CLK_OFF |
		KGSL_PWRFLAGS_AXI_OFF | KGSL_PWRFLAGS_POWER_OFF |
		KGSL_PWRFLAGS_IRQ_OFF;
	device->pwrctrl.nap_allowed = pdata_pwr->nap_allowed;
	device->pwrctrl.grp_clk = clk;
	device->pwrctrl.grp_src_clk = clk;
	device->pwrctrl.grp_pclk = pclk;
	device->pwrctrl.pwr_rail = PWR_RAIL_GRP_2D_CLK;
	device->pwrctrl.interval_timeout = pdata_pwr->idle_timeout;

	if (internal_pwr_rail_mode(device->pwrctrl.pwr_rail,
						PWR_RAIL_CTL_MANUAL)) {
		KGSL_DRV_ERR("call internal_pwr_rail_mode failed\n");
		result = -EINVAL;
		goto done;
	}

	clk = clk_get(NULL, "ebi1_kgsl_clk");
	if (IS_ERR(clk))
		clk = NULL;
	else
		clk_set_rate(clk,
			device->pwrctrl.
				pwrlevels[device->pwrctrl.active_pwrlevel].
					 bus_freq);
	device->pwrctrl.ebi1_clk = clk;

	if (bus_table) {
		device->pwrctrl.pcl = msm_bus_scale_register_client(bus_table);
		if (!device->pwrctrl.pcl) {
			KGSL_DRV_ERR("msm_bus_scale_register_client failed "
				     "id %d table %p", device->id,
				     bus_table);
			result = -EINVAL;
			goto done;
		}
	}
done:
	return result;
}

int __init
_kgsl_g12_init(struct kgsl_device *device, struct platform_device *pdev)
{
	int status = -EINVAL;
	struct kgsl_memregion *regspace = &device->regspace;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	struct resource *res;
	struct kgsl_platform_data *pdata = pdev->dev.platform_data;

	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	kgsl_g12_init_pwrctrl(device, pdev);

	/* initilization of timestamp wait */
	init_waitqueue_head(&(g12_device->wait_timestamp_wq));

	res = platform_get_resource_byname(kgsl_driver.pdev, IORESOURCE_MEM,
					   g12_device->iomemname);

	if (res == NULL) {
		KGSL_DRV_ERR("platform_get_resource_byname failed\n");
		status = -EINVAL;
		goto error;
	}

	regspace->mmio_phys_base = res->start;
	regspace->sizebytes = resource_size(res);

	if (regspace->mmio_phys_base == 0 || regspace->sizebytes == 0) {
		KGSL_DRV_ERR("dev %d invalid regspace\n", device->id);
		status = -ENODEV;
		goto error;
	}
	if (!request_mem_region(regspace->mmio_phys_base,
				regspace->sizebytes, DRIVER_NAME)) {
		KGSL_DRV_ERR("request_mem_region failed for " \
					"register memory\n");
		status = -ENODEV;
		goto error;
	}

	regspace->mmio_virt_base = ioremap(regspace->mmio_phys_base,
					   regspace->sizebytes);
	KGSL_MEM_INFO("ioremap(regs) = %p\n", regspace->mmio_virt_base);
	if (regspace->mmio_virt_base == NULL) {
		KGSL_DRV_ERR("ioremap failed for register memory\n");
		status = -ENODEV;
		goto error_release_mem;
	}

	status = request_irq(device->pwrctrl.interrupt_num, kgsl_g12_isr,
			     IRQF_TRIGGER_HIGH, DRIVER_NAME, device);
	if (status) {
		KGSL_DRV_ERR("request_irq(%d) returned %d\n",
			      device->pwrctrl.interrupt_num, status);
		goto error_iounmap;
	}
	device->pwrctrl.have_irq = 1;
	disable_irq(device->pwrctrl.interrupt_num);

	KGSL_DRV_INFO("dev_id %d regs phys 0x%08x size 0x%08x virt %p\n",
			device->id, regspace->mmio_phys_base,
			regspace->sizebytes, regspace->mmio_virt_base);

	kgsl_cffdump_open(device->id);

	init_completion(&device->hwaccess_gate);
	init_completion(&device->suspend_gate);
	kgsl_g12_getfunctable(&device->ftbl);
	ATOMIC_INIT_NOTIFIER_HEAD(&device->ts_notifier_list);

	setup_timer(&device->idle_timer, kgsl_timer, (unsigned long) device);
	status = kgsl_create_device_workqueue(device);
	if (status)
		goto error_free_irq;

	INIT_WORK(&device->idle_check_ws, kgsl_idle_check);

	INIT_LIST_HEAD(&device->memqueue);
	status = kgsl_g12_cmdstream_init(device);
	if (status != 0)
		goto error_dest_work_q;

	device->mmu.va_range = pdata->core->pt_va_size;

	status = kgsl_mmu_init(device);
	if (status != 0)
		goto error_close_cmdstream;

	status = kgsl_sharedmem_alloc_coherent(&device->memstore,
						sizeof(device->memstore));
	if (status != 0)
		goto error_close_mmu;

	kgsl_sharedmem_set(&device->memstore, 0, 0, device->memstore.size);

	device->pdev = pdev;

	status = kgsl_register_device(device);
	if (status)
		goto error_close_mmu;

	return 0;

error_close_mmu:
	kgsl_mmu_close(device);
error_close_cmdstream:
	kgsl_g12_cmdstream_close(device);
error_dest_work_q:
	destroy_workqueue(device->work_queue);
	device->work_queue = NULL;
error_free_irq:
	free_irq(device->pwrctrl.interrupt_num, NULL);
	device->pwrctrl.have_irq = 0;
error_iounmap:
	iounmap(regspace->mmio_virt_base);
	regspace->mmio_virt_base = NULL;
error_release_mem:
	release_mem_region(regspace->mmio_phys_base, regspace->sizebytes);
error:
	return status;
}

int _kgsl_g12_close(struct kgsl_device *device)
{
	struct kgsl_memregion *regspace = &device->regspace;

	if (device->memstore.hostptr)
		kgsl_sharedmem_free(&device->memstore);

	kgsl_mmu_close(device);

	kgsl_g12_cmdstream_close(device);

	if (regspace->mmio_virt_base != NULL) {
		KGSL_MEM_INFO("iounmap(regs) = %p\n",
				regspace->mmio_virt_base);
		iounmap(regspace->mmio_virt_base);
		regspace->mmio_virt_base = NULL;
		release_mem_region(regspace->mmio_phys_base,
					regspace->sizebytes);
	}

	kgsl_pwrctrl_close(device);
	kgsl_cffdump_close(device->id);

	if (device->work_queue) {
		destroy_workqueue(device->work_queue);
		device->work_queue = NULL;
	}

	KGSL_DRV_VDBG("return %d\n", 0);
	return 0;
}

int kgsl_g12_close(void)
{
	_kgsl_g12_close(&device_2d0.dev);
	_kgsl_g12_close(&device_2d1.dev);

	return 0;
}

int kgsl_g12_init(struct platform_device *pdev)
{
	struct kgsl_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0;

	if (pdata->dev_2d0->clk.name.clk != NULL)
		ret = _kgsl_g12_init(&device_2d0.dev, pdev);

	if (!ret && pdata->dev_2d1->clk.name.clk != NULL)
		ret = _kgsl_g12_init(&device_2d1.dev, pdev);

	return ret;
}

static int kgsl_g12_start(struct kgsl_device *device, unsigned int init_ram)
{
	int status = 0;
	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	device->state = KGSL_STATE_INIT;
	device->requested_state = KGSL_STATE_NONE;
	KGSL_PWR_INFO("state -> INIT, device %d\n", device->id);

	kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_POWER_ON);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_ON);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_ON);

	/* Set up MH arbiter.  MH offsets are considered to be dword
	 * based, therefore no down shift. */
	kgsl_g12_regwrite(device, ADDR_MH_ARBITER_CONFIG,
			  KGSL_G12_CFG_G12_MHARB);

	kgsl_g12_regwrite(device, ADDR_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030F27);
	kgsl_g12_regwrite(device, ADDR_MH_CLNT_INTF_CTRL_CONFIG2, 0x004B274F);

	kgsl_g12_regwrite(device, (ADDR_VGC_IRQENABLE >> 2), 0x3);

	status = kgsl_mmu_start(device);
	if (status)
		goto error_clk_off;

	status = kgsl_g12_cmdstream_start(device);
	if (status)
		goto error_mmu_stop;

	mod_timer(&device->idle_timer, jiffies + FIRST_TIMEOUT);
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_ON);
	return 0;
error_clk_off:
	kgsl_g12_regwrite(device, (ADDR_VGC_IRQENABLE >> 2), 0);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_OFF);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_OFF);
error_mmu_stop:
	kgsl_mmu_stop(device);
	return status;
}

static int kgsl_g12_stop(struct kgsl_device *device)
{
	kgsl_g12_idle(device, KGSL_TIMEOUT_DEFAULT);

	del_timer(&device->idle_timer);

	kgsl_mmu_stop(device);

	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_IRQ_OFF);
	kgsl_pwrctrl_axi(device, KGSL_PWRFLAGS_AXI_OFF);
	kgsl_pwrctrl_clk(device, KGSL_PWRFLAGS_CLK_OFF);
	kgsl_pwrctrl_pwrrail(device, KGSL_PWRFLAGS_POWER_OFF);

	return 0;
}

static int kgsl_g12_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type,
				void *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;

	switch (type) {
	case KGSL_PROP_DEVICE_INFO:
	{
		struct kgsl_devinfo devinfo;

		if (sizebytes != sizeof(devinfo)) {
			status = -EINVAL;
			break;
		}

		memset(&devinfo, 0, sizeof(devinfo));
		devinfo.device_id = device->id+1;
		devinfo.chip_id = device->chip_id;
		devinfo.mmu_enabled = kgsl_mmu_isenabled(&device->mmu);

		if (copy_to_user(value, &devinfo, sizeof(devinfo)) !=
				0) {
			status = -EFAULT;
			break;
		}
		status = 0;
	}
	break;
	case KGSL_PROP_MMU_ENABLE:
		{
#ifdef CONFIG_MSM_KGSL_MMU
			int mmuProp = 1;
#else
			int mmuProp = 0;
#endif
			if (sizebytes != sizeof(int)) {
				status = -EINVAL;
				break;
			}
			if (copy_to_user(value, &mmuProp, sizeof(mmuProp))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;

	default:
	KGSL_DRV_ERR("invalid property: %d\n", type);
	status = -EINVAL;

	}
	return status;
}

int kgsl_g12_idle(struct kgsl_device *device, unsigned int timeout)
{
	int status = KGSL_SUCCESS;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	KGSL_DRV_VDBG("enter (device=%p, timeout=%d)\n", device, timeout);

	if (g12_device->current_timestamp > g12_device->timestamp)
		status = kgsl_g12_waittimestamp(device,
					g12_device->current_timestamp, timeout);

	if (status)
		KGSL_DRV_ERR("Error, kgsl_g12_waittimestamp() timed out\n");

	KGSL_DRV_VDBG("return %d\n", status);

	return status;
}

static unsigned int kgsl_g12_isidle(struct kgsl_device *device)
{
	int status = false;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	int timestamp = g12_device->timestamp;

	if (timestamp == g12_device->current_timestamp)
		status = true;

	return status;
}

static int kgsl_g12_resume_context(struct kgsl_device *device)
{
	/* Context is in the pre-amble, automatically restored. */

	return KGSL_SUCCESS;
}

static int kgsl_g12_suspend_context(struct kgsl_device *device)
{
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);

	g12_device->ringbuffer.prevctx = KGSL_G12_INVALID_CONTEXT;

	return KGSL_SUCCESS;
}

int kgsl_g12_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	unsigned int *reg;
	kgsl_pre_hwaccess(device);
	if ((offsetwords >= ADDR_MH_ARBITER_CONFIG &&
	     offsetwords <= ADDR_MH_AXI_HALT_CONTROL) ||
	    (offsetwords >= ADDR_MH_MMU_CONFIG &&
	     offsetwords <= ADDR_MH_MMU_MPU_END)) {
		kgsl_g12_regwrite(device, (ADDR_VGC_MH_READ_ADDR >> 2),
				  offsetwords);
		reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ ADDR_VGC_MH_DATA_ADDR);
	} else {
		if (offsetwords * sizeof(uint32_t) >=
				device->regspace.sizebytes) {
			KGSL_DRV_ERR("invalid offset %d\n", offsetwords);
			return -ERANGE;
		}

		reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
	}

	*value = readl(reg);

	return 0;
}

int kgsl_g12_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value)
{
	unsigned int *reg;

	kgsl_pre_hwaccess(device);
	if ((offsetwords >= ADDR_MH_ARBITER_CONFIG &&
	     offsetwords <= ADDR_MH_CLNT_INTF_CTRL_CONFIG2) ||
	    (offsetwords >= ADDR_MH_MMU_CONFIG &&
	     offsetwords <= ADDR_MH_MMU_MPU_END)) {
		kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_MMU,
					 offsetwords, value);
	} else {
		if (offsetwords*sizeof(uint32_t) >=
				device->regspace.sizebytes) {
			KGSL_DRV_ERR("invalid offset %d\n", offsetwords);
			return -ERANGE;
		}

		reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
		kgsl_cffdump_regwrite(device->id, offsetwords << 2, value);
		writel(value, reg);
		/* Drain write buffer */
		dsb();

		/* Memory fence to ensure all data has posted.  On some systems,
		 * like 7x27, the register block is not allocated as strongly
		 * ordered memory.  Adding a memory fence ensures ordering
		 * during ringbuffer submits.*/
		mb();
	}

	return 0;
}

static int kgsl_g12_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	int status = -EINVAL;
	struct kgsl_g12_device *g12_device = KGSL_G12_DEVICE(device);
	long timeout = 0;

	KGSL_DRV_INFO("enter (device=%p,timestamp=%d,timeout=0x%08x)\n",
			device, timestamp, msecs);

	KGSL_DRV_INFO("current (device=%p,timestamp=%d)\n",
			device, g12_device->timestamp);

	mutex_unlock(&device->mutex);
	timeout = wait_io_event_interruptible_timeout(
			g12_device->wait_timestamp_wq,
			kgsl_check_timestamp(device, timestamp),
			msecs_to_jiffies(msecs));
	mutex_lock(&device->mutex);

	if (timeout > 0)
		status = 0;
	else if (timeout == 0) {
		status = -ETIMEDOUT;
		device->state = KGSL_STATE_HUNG;
		KGSL_PWR_INFO("state -> HUNG, device %d\n", device->id);
	}
	else
		status = timeout;

	KGSL_DRV_INFO("return %d\n", status);
	return status;
}

static long kgsl_g12_ioctl_cmdwindow_write(struct kgsl_device_private *dev_priv,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_cmdwindow_write param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	result = kgsl_g12_cmdwindow_write(dev_priv->device,
					     param.target,
					     param.addr,
					     param.data);

	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_g12_ioctl(struct kgsl_device_private *dev_priv,
			unsigned int cmd,
			unsigned long arg)
{
	int result = 0;

	switch (cmd) {
	case IOCTL_KGSL_CMDWINDOW_WRITE:
		result = kgsl_g12_ioctl_cmdwindow_write(dev_priv,
							(void __user *)arg);
		break;
	default:
		KGSL_DRV_ERR("invalid ioctl code %08x\n", cmd);
		result = -EINVAL;
		break;
	}
	return result;

}

int kgsl_g12_getfunctable(struct kgsl_functable *ftbl)
{

	if (ftbl == NULL)
		return KGSL_FAILURE;
	ftbl->device_regread = kgsl_g12_regread;
	ftbl->device_regwrite = kgsl_g12_regwrite;
	ftbl->device_setstate = kgsl_g12_setstate;
	ftbl->device_idle = kgsl_g12_idle;
	ftbl->device_isidle = kgsl_g12_isidle;
	ftbl->device_suspend_context = kgsl_g12_suspend_context;
	ftbl->device_resume_context = kgsl_g12_resume_context;
	ftbl->device_start = kgsl_g12_start;
	ftbl->device_stop = kgsl_g12_stop;
	ftbl->device_getproperty = kgsl_g12_getproperty;
	ftbl->device_waittimestamp = kgsl_g12_waittimestamp;
	ftbl->device_cmdstream_readtimestamp = kgsl_g12_cmdstream_readtimestamp;
	ftbl->device_issueibcmds = kgsl_g12_cmdstream_issueibcmds;
	ftbl->device_drawctxt_create = kgsl_g12_drawctxt_create;
	ftbl->device_drawctxt_destroy = kgsl_g12_drawctxt_destroy;
	ftbl->device_ioctl = kgsl_g12_ioctl;
	ftbl->device_setup_pt = kgsl_g12_setup_pt;
	ftbl->device_cleanup_pt = kgsl_g12_cleanup_pt;

	return KGSL_SUCCESS;
}
