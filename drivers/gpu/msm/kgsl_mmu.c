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
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/bitmap.h>
#include <linux/dmapool.h>

#ifdef CONFIG_MSM_KGSL_MMU
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#endif
#include "kgsl_mmu.h"
#include "kgsl_drawctxt.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_device.h"

struct kgsl_pte_debug {
	unsigned int read:1;
	unsigned int write:1;
	unsigned int dirty:1;
	unsigned int reserved:9;
	unsigned int phyaddr:20;
};

#define GSL_PT_PAGE_BITS_MASK	0x00000007
#define GSL_PT_PAGE_ADDR_MASK	(~(KGSL_PAGESIZE - 1))

#define GSL_MMU_INT_MASK \
	(MH_INTERRUPT_MASK__AXI_READ_ERROR | \
	 MH_INTERRUPT_MASK__AXI_WRITE_ERROR)

/* pt_mutex needs to be held in this function */

static struct kgsl_pagetable *
kgsl_get_pagetable(unsigned long name)
{
	struct kgsl_pagetable *pt;

	list_for_each_entry(pt,	&kgsl_driver.pagetable_list, list) {
		if (pt->name == name)
			return pt;
	}

	return NULL;
}

static struct kgsl_pagetable *
_get_pt_from_kobj(struct kobject *kobj)
{
	unsigned long ptname;

	if (!kobj)
		return NULL;

	if (sscanf(kobj->name, "%ld", &ptname) != 1)
		return NULL;

	return kgsl_get_pagetable(ptname);
}

static ssize_t
sysfs_show_entries(struct kobject *kobj,
		   struct kobj_attribute *attr,
		   char *buf)
{
	struct kgsl_pagetable *pt;
	int ret = 0;

	mutex_lock(&kgsl_driver.pt_mutex);
	pt = _get_pt_from_kobj(kobj);

	if (pt)
		ret += sprintf(buf, "%d\n", pt->stats.entries);

	mutex_unlock(&kgsl_driver.pt_mutex);
	return ret;
}

static ssize_t
sysfs_show_mapped(struct kobject *kobj,
		  struct kobj_attribute *attr,
		  char *buf)
{
	struct kgsl_pagetable *pt;
	int ret = 0;

	mutex_lock(&kgsl_driver.pt_mutex);
	pt = _get_pt_from_kobj(kobj);

	if (pt)
		ret += sprintf(buf, "%d\n", pt->stats.mapped);

	mutex_unlock(&kgsl_driver.pt_mutex);
	return ret;
}

static ssize_t
sysfs_show_va_range(struct kobject *kobj,
		    struct kobj_attribute *attr,
		    char *buf)
{
	struct kgsl_pagetable *pt;
	int ret = 0;

	mutex_lock(&kgsl_driver.pt_mutex);
	pt = _get_pt_from_kobj(kobj);

	if (pt)
		ret += sprintf(buf, "0x%x\n", pt->va_range);

	mutex_unlock(&kgsl_driver.pt_mutex);
	return ret;
}

static ssize_t
sysfs_show_max_mapped(struct kobject *kobj,
		      struct kobj_attribute *attr,
		      char *buf)
{
	struct kgsl_pagetable *pt;
	int ret = 0;

	mutex_lock(&kgsl_driver.pt_mutex);
	pt = _get_pt_from_kobj(kobj);

	if (pt)
		ret += sprintf(buf, "%d\n", pt->stats.max_mapped);

	mutex_unlock(&kgsl_driver.pt_mutex);
	return ret;
}

static ssize_t
sysfs_show_max_entries(struct kobject *kobj,
		       struct kobj_attribute *attr,
		       char *buf)
{
	struct kgsl_pagetable *pt;
	int ret = 0;

	mutex_lock(&kgsl_driver.pt_mutex);
	pt = _get_pt_from_kobj(kobj);

	if (pt)
		ret += sprintf(buf, "%d\n", pt->stats.max_entries);

	mutex_unlock(&kgsl_driver.pt_mutex);
	return ret;
}

static struct kobj_attribute attr_entries = {
	.attr = { .name = "entries", .mode = 0444 },
	.show = sysfs_show_entries,
	.store = NULL,
};

static struct kobj_attribute attr_mapped = {
	.attr = { .name = "mapped", .mode = 0444 },
	.show = sysfs_show_mapped,
	.store = NULL,
};

static struct kobj_attribute attr_va_range = {
	.attr = { .name = "va_range", .mode = 0444 },
	.show = sysfs_show_va_range,
	.store = NULL,
};

static struct kobj_attribute attr_max_mapped = {
	.attr = { .name = "max_mapped", .mode = 0444 },
	.show = sysfs_show_max_mapped,
	.store = NULL,
};

static struct kobj_attribute attr_max_entries = {
	.attr = { .name = "max_entries", .mode = 0444 },
	.show = sysfs_show_max_entries,
	.store = NULL,
};

static struct attribute *pagetable_attrs[] = {
	&attr_entries.attr,
	&attr_mapped.attr,
	&attr_va_range.attr,
	&attr_max_mapped.attr,
	&attr_max_entries.attr,
	NULL,
};

static struct attribute_group pagetable_attr_group = {
	.attrs = pagetable_attrs,
};

static void
pagetable_remove_sysfs_objects(struct kgsl_pagetable *pagetable)
{
	if (pagetable->kobj)
		sysfs_remove_group(pagetable->kobj,
				   &pagetable_attr_group);

	kobject_put(pagetable->kobj);
}

static int
pagetable_add_sysfs_objects(struct kgsl_pagetable *pagetable)
{
	char ptname[16];
	int ret = -ENOMEM;

	snprintf(ptname, sizeof(ptname), "%d", pagetable->name);
	pagetable->kobj = kobject_create_and_add(ptname,
						 kgsl_driver.ptkobj);
	if (pagetable->kobj == NULL)
		goto err;

	ret = sysfs_create_group(pagetable->kobj, &pagetable_attr_group);

err:
	if (ret) {
		if (pagetable->kobj)
			kobject_put(pagetable->kobj);

		pagetable->kobj = NULL;
	}

	return ret;
}

static inline uint32_t
kgsl_pt_entry_get(struct kgsl_pagetable *pt, uint32_t va)
{
	return (va - pt->va_base) >> KGSL_PAGESIZE_SHIFT;
}

static inline void
kgsl_pt_map_set(struct kgsl_pagetable *pt, uint32_t pte, uint32_t val)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	writel(val, &baseptr[pte]);
}

static inline uint32_t
kgsl_pt_map_getaddr(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return readl(&baseptr[pte]) & GSL_PT_PAGE_ADDR_MASK;
}

void kgsl_mh_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;
	unsigned int reg;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	kgsl_regread(device, device->mmu.reg.interrupt_status, &status);

	if (status & MH_INTERRUPT_MASK__AXI_READ_ERROR) {
		KGSL_MEM_FATAL("axi read error interrupt\n");
	} else if (status & MH_INTERRUPT_MASK__AXI_WRITE_ERROR) {
		KGSL_MEM_FATAL("axi write error interrupt\n");
	} else if (status & MH_INTERRUPT_MASK__MMU_PAGE_FAULT) {
		kgsl_regread(device, device->mmu.reg.page_fault, &reg);
		KGSL_MEM_FATAL("mmu page fault interrupt: %08x\n", reg);
	} else {
		KGSL_MEM_DBG("bad bits in REG_MH_INTERRUPT_STATUS %08x\n",
			     status);
	}

	kgsl_regwrite(device, device->mmu.reg.interrupt_clear, status);

	/*TODO: figure out how to handle errror interupts.
	* specifically, page faults should probably nuke the client that
	* caused them, but we don't have enough info to figure that out yet.
	*/

	KGSL_MEM_VDBG("return\n");
}

static struct kgsl_pagetable *kgsl_mmu_createpagetableobject(
				unsigned int name)
{
	int status = 0;
	struct kgsl_pagetable *pagetable = NULL;

	KGSL_MEM_VDBG("enter (pt_name=%d)\n", name);

	pagetable = kzalloc(sizeof(struct kgsl_pagetable), GFP_KERNEL);
	if (pagetable == NULL) {
		KGSL_MEM_ERR("Unable to allocate pagetable object.\n");
		return NULL;
	}

	pagetable->refcnt = 1;

	spin_lock_init(&pagetable->lock);
	pagetable->tlb_flags = 0;
	pagetable->name = name;
	pagetable->va_base = kgsl_driver.pt_va_base;
	pagetable->va_range = kgsl_driver.pt_va_size;
	pagetable->last_superpte = 0;
	pagetable->max_entries = KGSL_PAGETABLE_ENTRIES(pagetable->va_range);

	pagetable->tlbflushfilter.size = (pagetable->va_range /
				(PAGE_SIZE * GSL_PT_SUPER_PTE * 8)) + 1;
	pagetable->tlbflushfilter.base = (unsigned int *)
			kzalloc(pagetable->tlbflushfilter.size, GFP_KERNEL);
	if (!pagetable->tlbflushfilter.base) {
		KGSL_MEM_ERR("Failed to create tlbflushfilter\n");
		goto err_alloc;
	}
	GSL_TLBFLUSH_FILTER_RESET();

	pagetable->pool = gen_pool_create(KGSL_PAGESIZE_SHIFT, -1);
	if (pagetable->pool == NULL) {
		KGSL_MEM_ERR("Unable to allocate virtualaddr pool.\n");
		goto err_flushfilter;
	}

	if (gen_pool_add(pagetable->pool, pagetable->va_base,
				pagetable->va_range, -1)) {
		KGSL_MEM_ERR("gen_pool_create failed for pagetable %p\n",
				pagetable);
		goto err_pool;
	}

	pagetable->base.hostptr = dma_pool_alloc(kgsl_driver.ptpool,
						 GFP_KERNEL,
						 &pagetable->base.physaddr);

	if (pagetable->base.hostptr == NULL) {
		KGSL_MEM_ERR("Unable to allocate memory for the pagetable\n");
		goto err_pool;
	}

	/* ptpool allocations are from coherent memory, so update the
	   device statistics acordingly */

	KGSL_STATS_ADD(kgsl_driver.ptsize, kgsl_driver.stats.coherent,
		       kgsl_driver.stats.coherent_max);

	pagetable->base.gpuaddr = pagetable->base.physaddr;
	pagetable->base.size = kgsl_driver.ptsize;

	status = kgsl_setup_pt(pagetable);
	if (status)
		goto err_pool;

	list_add(&pagetable->list, &kgsl_driver.pagetable_list);

	/* Create the sysfs entries */
	pagetable_add_sysfs_objects(pagetable);

	KGSL_MEM_VDBG("return %p\n", pagetable);
	return pagetable;

err_pool:
	gen_pool_destroy(pagetable->pool);
err_flushfilter:
	kfree(pagetable->tlbflushfilter.base);
err_alloc:
	kfree(pagetable);

	return NULL;
}

static void kgsl_mmu_destroypagetable(struct kgsl_pagetable *pagetable)
{
	KGSL_MEM_VDBG("enter (pagetable=%p)\n", pagetable);

	list_del(&pagetable->list);

	pagetable_remove_sysfs_objects(pagetable);

	kgsl_cleanup_pt(pagetable);

	dma_pool_free(kgsl_driver.ptpool, pagetable->base.hostptr,
		      pagetable->base.physaddr);

	if (pagetable->pool) {
		gen_pool_destroy(pagetable->pool);
		pagetable->pool = NULL;
	}

	if (pagetable->tlbflushfilter.base) {
		pagetable->tlbflushfilter.size = 0;
		kfree(pagetable->tlbflushfilter.base);
		pagetable->tlbflushfilter.base = NULL;
	}

	kfree(pagetable);
}

struct kgsl_pagetable *kgsl_mmu_getpagetable(unsigned long name)
{
	struct kgsl_pagetable *pt;

	mutex_lock(&kgsl_driver.pt_mutex);

	pt = kgsl_get_pagetable(name);

	if (pt) {
		spin_lock(&pt->lock);
		pt->refcnt++;
		spin_unlock(&pt->lock);
		goto done;
	}

	pt = kgsl_mmu_createpagetableobject(name);

done:
	mutex_unlock(&kgsl_driver.pt_mutex);
	return pt;
}

void kgsl_mmu_putpagetable(struct kgsl_pagetable *pagetable)
{
	bool dead;
	if (pagetable == NULL)
		return;

	mutex_lock(&kgsl_driver.pt_mutex);

	spin_lock(&pagetable->lock);
	dead = (--pagetable->refcnt) == 0;
	spin_unlock(&pagetable->lock);

	if (dead)
		kgsl_mmu_destroypagetable(pagetable);

	mutex_unlock(&kgsl_driver.pt_mutex);
}

int kgsl_mmu_setstate(struct kgsl_device *device,
				struct kgsl_pagetable *pagetable)
{
	int status = 0;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p, pagetable=%p)\n", device, pagetable);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		/* page table not current, then setup mmu to use new
		 *  specified page table
		 */
		KGSL_MEM_INFO("from %p to %p\n", mmu->hwpagetable, pagetable);
		if (mmu->hwpagetable != pagetable) {
			mmu->hwpagetable = pagetable;
			spin_lock(&mmu->hwpagetable->lock);
			mmu->hwpagetable->tlb_flags &= ~(1<<device->id);
			spin_unlock(&mmu->hwpagetable->lock);

			/* call device specific set page table */
			status = kgsl_setstate(mmu->device,
				KGSL_MMUFLAGS_TLBFLUSH |
				KGSL_MMUFLAGS_PTUPDATE);

		}
	}

	KGSL_MEM_VDBG("return %d\n", status);

	return status;
}

int kgsl_mmu_init(struct kgsl_device *device)
{
	/*
	 * intialize device mmu
	 *
	 * call this with the global lock held
	 */
	int status;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	mmu->device = device;

#ifndef CONFIG_MSM_KGSL_MMU
	mmu->config = 0x00000000;
#endif

	/* MMU not enabled */
	if ((mmu->config & 0x1) == 0) {
		KGSL_MEM_VDBG("return %d\n", 0);
		return 0;
	}

	/* make sure aligned to pagesize */
	BUG_ON(mmu->mpu_base & (KGSL_PAGESIZE - 1));
	BUG_ON((mmu->mpu_base + mmu->mpu_range) & (KGSL_PAGESIZE - 1));

	/* sub-client MMU lookups require address translation */
	if ((mmu->config & ~0x1) > 0) {
		/*make sure virtual address range is a multiple of 64Kb */
		BUG_ON(mmu->va_range & ((1 << 16) - 1));

		/* allocate memory used for completing r/w operations that
		 * cannot be mapped by the MMU
		 */
		status = kgsl_sharedmem_alloc_coherent(&mmu->dummyspace, 64);
		if (status != 0) {
			KGSL_MEM_ERR
			    ("Unable to allocate dummy space memory.\n");
			goto error;
		}

		kgsl_sharedmem_set(&mmu->dummyspace, 0, 0,
				   mmu->dummyspace.size);

	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;

error:
	return status;
}

int kgsl_mmu_start(struct kgsl_device *device)
{
	/*
	 * intialize device mmu
	 *
	 * call this with the global lock held
	 */
	int status;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		KGSL_MEM_INFO("MMU already started.\n");
		return 0;
	}

	/* MMU not enabled */
	if ((mmu->config & 0x1) == 0) {
		KGSL_MEM_VDBG("return %d\n", 0);
		return 0;
	}

	mmu->flags |= KGSL_FLAGS_STARTED;

	/* setup MMU and sub-client behavior */
	kgsl_regwrite(device, device->mmu.reg.config, mmu->config);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK);
	kgsl_regwrite(device, device->mmu.reg.interrupt_mask,
				GSL_MMU_INT_MASK);

	/* idle device */
	kgsl_idle(device,  KGSL_TIMEOUT_DEFAULT);

	/* define physical memory range accessible by the core */
	kgsl_regwrite(device, device->mmu.reg.mpu_base, mmu->mpu_base);
	kgsl_regwrite(device, device->mmu.reg.mpu_end,
			mmu->mpu_base + mmu->mpu_range);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);
	kgsl_regwrite(device, device->mmu.reg.interrupt_mask,
			GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);

	/* sub-client MMU lookups require address translation */
	if ((mmu->config & ~0x1) > 0) {

		kgsl_sharedmem_set(&mmu->dummyspace, 0, 0,
				   mmu->dummyspace.size);

		/* TRAN_ERROR needs a 32 byte (32 byte aligned) chunk of memory
		 * to complete transactions in case of an MMU fault. Note that
		 * we'll leave the bottom 32 bytes of the dummyspace for other
		 * purposes (e.g. use it when dummy read cycles are needed
		 * for other blocks */
		kgsl_regwrite(device, device->mmu.reg.tran_error,
						mmu->dummyspace.physaddr + 32);

		BUG_ON(mmu->defaultpagetable == NULL);
		mmu->hwpagetable = mmu->defaultpagetable;

		kgsl_regwrite(device, device->mmu.reg.pt_page,
			      mmu->hwpagetable->base.gpuaddr);
		kgsl_regwrite(device, device->mmu.reg.va_range,
			      (mmu->hwpagetable->va_base |
			      (mmu->hwpagetable->va_range >> 16)));
		status = kgsl_setstate(device, KGSL_MMUFLAGS_TLBFLUSH);
		if (status) {
			KGSL_MEM_ERR("Failed to setstate TLBFLUSH\n");
			goto error;
		}
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
error:
	/* disable MMU */
	kgsl_regwrite(device, device->mmu.reg.interrupt_mask, 0);
	kgsl_regwrite(device, device->mmu.reg.config, 0x00000000);
	return status;
}



#ifdef CONFIG_MSM_KGSL_MMU

unsigned int kgsl_virtaddr_to_physaddr(unsigned int virtaddr)
{
	unsigned int physaddr = 0;
	pgd_t *pgd_ptr = NULL;
	pmd_t *pmd_ptr = NULL;
	pte_t *pte_ptr = NULL, pte;

	pgd_ptr = pgd_offset(current->mm, virtaddr);
	if (pgd_none(*pgd) || pgd_bad(*pgd)) {
		KGSL_MEM_ERR
		    ("Invalid pgd entry found while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}

	pmd_ptr = pmd_offset(pgd_ptr, virtaddr);
	if (pmd_none(*pmd_ptr) || pmd_bad(*pmd_ptr)) {
		KGSL_MEM_ERR
		    ("Invalid pmd entry found while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}

	pte_ptr = pte_offset_map(pmd_ptr, virtaddr);
	if (!pte_ptr) {
		KGSL_MEM_ERR
		    ("Unable to map pte entry while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}
	pte = *pte_ptr;
	physaddr = pte_pfn(pte);
	pte_unmap(pte_ptr);
	physaddr <<= PAGE_SHIFT;
	return physaddr;
}

int
kgsl_mmu_map(struct kgsl_pagetable *pagetable,
				unsigned int address,
				int range,
				unsigned int protflags,
				unsigned int *gpuaddr,
				unsigned int flags)
{
	int numpages;
	unsigned int pte, ptefirst, ptelast, physaddr;
	int flushtlb, alloc_size;
	unsigned int align = flags & KGSL_MEMFLAGS_ALIGN_MASK;

	KGSL_MEM_VDBG("enter (pt=%p, physaddr=%08x, range=%08d, gpuaddr=%p)\n",
		      pagetable, address, range, gpuaddr);

	BUG_ON(protflags & ~(GSL_PT_PAGE_RV | GSL_PT_PAGE_WV));
	BUG_ON(protflags == 0);
	BUG_ON(range <= 0);

	/* Only support 4K and 8K alignment for now */
	if (align != KGSL_MEMFLAGS_ALIGN8K && align != KGSL_MEMFLAGS_ALIGN4K) {
		KGSL_MEM_ERR("Cannot map memory according to "
			     "requested flags: %08x\n", flags);
		return -EINVAL;
	}

	/* Make sure address being mapped is at 4K boundary */
	if (!IS_ALIGNED(address, KGSL_PAGESIZE) || range & ~KGSL_PAGEMASK) {
		KGSL_MEM_ERR("Cannot map address not aligned "
			     "at page boundary: address: %08x, range: %08x\n",
			     address, range);
		return -EINVAL;
	}
	alloc_size = range;
	if (align == KGSL_MEMFLAGS_ALIGN8K)
		alloc_size += KGSL_PAGESIZE;

	*gpuaddr = gen_pool_alloc(pagetable->pool, alloc_size);
	if (*gpuaddr == 0) {
		KGSL_MEM_ERR("gen_pool_alloc failed: pt=%d size=%d\n",
				pagetable->name, alloc_size);
		KGSL_MEM_ERR("  pt %d allocated=%d, entries=%d\n",
				pagetable->name, pagetable->stats.mapped,
				pagetable->stats.entries);
		return -ENOMEM;
	}

	if (align == KGSL_MEMFLAGS_ALIGN8K) {
		if (*gpuaddr & ((1 << 13) - 1)) {
			/* Not 8k aligned, align it */
			gen_pool_free(pagetable->pool, *gpuaddr, KGSL_PAGESIZE);
			*gpuaddr = *gpuaddr + KGSL_PAGESIZE;
		} else
			gen_pool_free(pagetable->pool, *gpuaddr + range,
				      KGSL_PAGESIZE);
	}

	numpages = (range >> KGSL_PAGESIZE_SHIFT);

	ptefirst = kgsl_pt_entry_get(pagetable, *gpuaddr);
	ptelast = ptefirst + numpages;

	pte = ptefirst;
	flushtlb = 0;

	/* tlb needs to be flushed when the first and last pte are not at
	* superpte boundaries */
	if ((ptefirst & (GSL_PT_SUPER_PTE - 1)) != 0 ||
		((ptelast + 1) & (GSL_PT_SUPER_PTE-1)) != 0)
		flushtlb = 1;

	spin_lock(&pagetable->lock);
	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		uint32_t val = kgsl_pt_map_getaddr(pagetable, pte);
		BUG_ON(val != 0 && val != GSL_PT_PAGE_DIRTY);
#endif
		if ((pte & (GSL_PT_SUPER_PTE-1)) == 0)
			if (GSL_TLBFLUSH_FILTER_ISDIRTY(pte / GSL_PT_SUPER_PTE))
				flushtlb = 1;
		/* mark pte as in use */
		if (flags & KGSL_MEMFLAGS_CONPHYS)
			physaddr = address;
		else if (flags & KGSL_MEMFLAGS_VMALLOC_MEM) {
			physaddr = vmalloc_to_pfn((void *)address);
			physaddr <<= PAGE_SHIFT;
		} else if (flags & KGSL_MEMFLAGS_HOSTADDR)
			physaddr = kgsl_virtaddr_to_physaddr(address);
		else
			physaddr = 0;

		if (physaddr) {
			kgsl_pt_map_set(pagetable, pte, physaddr | protflags);
		} else {
			KGSL_MEM_ERR
			("Unable to find physaddr for address: %x\n",
			     address);
			spin_unlock(&pagetable->lock);
			kgsl_mmu_unmap(pagetable, *gpuaddr, range);
			return -EFAULT;
		}
		address += KGSL_PAGESIZE;
	}

	/* Keep track of the statistics for the sysfs files */

	KGSL_STATS_ADD(1, pagetable->stats.entries,
		       pagetable->stats.max_entries);

	KGSL_STATS_ADD(alloc_size, pagetable->stats.mapped,
		       pagetable->stats.max_mapped);

	KGSL_MEM_INFO("pt %p p %08x g %08x pte f %d l %d n %d f %d\n",
		      pagetable, address, *gpuaddr, ptefirst, ptelast,
		      numpages, flushtlb);

	mb();

	/* Invalidate tlb only if current page table used by GPU is the
	 * pagetable that we used to allocate */
	if (flushtlb) {
		/*set all devices as needing flushing*/
		pagetable->tlb_flags = UINT_MAX;
		GSL_TLBFLUSH_FILTER_RESET();
	}
	spin_unlock(&pagetable->lock);


	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}

int
kgsl_mmu_unmap(struct kgsl_pagetable *pagetable, unsigned int gpuaddr,
		int range)
{
	unsigned int numpages;
	unsigned int pte, ptefirst, ptelast, superpte;

	KGSL_MEM_VDBG("enter (pt=%p, gpuaddr=0x%08x, range=%d)\n",
			pagetable, gpuaddr, range);

	BUG_ON(range <= 0);

	numpages = (range >> KGSL_PAGESIZE_SHIFT);
	if (range & (KGSL_PAGESIZE - 1))
		numpages++;

	ptefirst = kgsl_pt_entry_get(pagetable, gpuaddr);
	ptelast = ptefirst + numpages;

	KGSL_MEM_INFO("pt %p gpu %08x pte first %d last %d numpages %d\n",
		      pagetable, gpuaddr, ptefirst, ptelast, numpages);

	spin_lock(&pagetable->lock);
	superpte = ptefirst - (ptefirst & (GSL_PT_SUPER_PTE-1));
	GSL_TLBFLUSH_FILTER_SETDIRTY(superpte / GSL_PT_SUPER_PTE);
	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		BUG_ON(!kgsl_pt_map_getaddr(pagetable, pte));
#endif
		kgsl_pt_map_set(pagetable, pte, GSL_PT_PAGE_DIRTY);
		superpte = pte - (pte & (GSL_PT_SUPER_PTE - 1));
		if (pte == superpte)
			GSL_TLBFLUSH_FILTER_SETDIRTY(superpte /
				GSL_PT_SUPER_PTE);
	}

	/* Remove the statistics */
	pagetable->stats.entries--;
	pagetable->stats.mapped -= range;

	mb();
	spin_unlock(&pagetable->lock);

	gen_pool_free(pagetable->pool, gpuaddr, range);

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
#endif /*CONFIG_MSM_KGSL_MMU*/

int kgsl_mmu_map_global(struct kgsl_pagetable *pagetable,
			struct kgsl_memdesc *memdesc, unsigned int protflags,
			unsigned int flags)
{
	int result = -EINVAL;
	unsigned int gpuaddr = 0;

	if (memdesc == NULL)
		goto error;

	result = kgsl_mmu_map(pagetable, memdesc->physaddr, memdesc->size,
				protflags, &gpuaddr, flags);
	if (result)
		goto error;

	/*global mappings must have the same gpu address in all pagetables*/
	if (memdesc->gpuaddr == 0)
		memdesc->gpuaddr = gpuaddr;

	else if (memdesc->gpuaddr != gpuaddr) {
		KGSL_MEM_ERR("pt %p addr mismatch phys 0x%08x gpu 0x%0x 0x%08x",
				pagetable, memdesc->physaddr,
				memdesc->gpuaddr, gpuaddr);
		goto error_unmap;
	}
	return result;
error_unmap:
	kgsl_mmu_unmap(pagetable, gpuaddr, memdesc->size);
error:
	return result;
}

int kgsl_mmu_stop(struct kgsl_device *device)
{
	/*
	 *  stop device mmu
	 *
	 *  call this with the global lock held
	 */
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		/* disable mh interrupts */
		KGSL_MEM_DBG("disabling mmu interrupts\n");
		/* disable MMU */
		kgsl_regwrite(device, device->mmu.reg.interrupt_mask, 0);
		kgsl_regwrite(device, device->mmu.reg.config, 0x00000000);

		mmu->flags &= ~KGSL_FLAGS_STARTED;
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;


}

int kgsl_mmu_close(struct kgsl_device *device)
{
	/*
	 *  close device mmu
	 *
	 *  call this with the global lock held
	 */
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->dummyspace.gpuaddr)
		kgsl_sharedmem_free(&mmu->dummyspace);

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
