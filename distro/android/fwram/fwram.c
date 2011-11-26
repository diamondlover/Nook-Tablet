/*
 * Driver to maap reserved physical mem for OMAP4 board
 *
 * Copyright (C) 2009-2011 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>

#define BUFFER_PHYS_ADDR    0x9C000000
#define FWRAM_VERSION    "1.0"
#define FWRAM_MINOR      0

static s32 fwram_mmap(struct file *f, struct vm_area_struct *v);

static const struct file_operations fwram_fops = {
	.mmap    = fwram_mmap,
};

static struct miscdevice fwram_dev = {
	FWRAM_MINOR,
	"fwram",
	&fwram_fops
};

static s32 __init fwram_init(void)
{
	int ret;
	ret = misc_register(&fwram_dev);
	if (ret) {
		printk(KERN_ERR "fwram: can't misc_register on minor=%d\n",
			FWRAM_MINOR);
		return ret;
	}
	printk(KERN_INFO "FWRAM memory driver v" FWRAM_VERSION "\n");
	return 0;
}

static s32 fwram_mmap(struct file *f, struct vm_area_struct *vma)
{
	unsigned long pfn = BUFFER_PHYS_ADDR >> PAGE_SHIFT;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long prot;

	if (vma->vm_pgoff != 0)
		return -EINVAL;

	vma->vm_flags &= ~VM_MAYREAD;
	vma->vm_flags &= ~VM_READ;

	/* the protection requested for the new vma */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	prot = pgprot_val(vma->vm_page_prot);
	vma->vm_page_prot = __pgprot(prot);

	if (remap_pfn_range(vma, vma->vm_start,
			pfn,
			vsize, vma->vm_page_prot)) {
		printk(KERN_ERR "remap_pfn_range failed in fwram_mmap\n");
		return -EAGAIN;
	}

	return 0;
}

static void __exit fwram_exit(void)
{
	misc_deregister(&fwram_dev);
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("y-zhi@ti.com");
module_init(fwram_init);
module_exit(fwram_exit);
