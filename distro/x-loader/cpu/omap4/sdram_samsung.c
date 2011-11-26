/*
 * (C) Copyright 2004-2009
 * Texas Instruments, <www.ti.com>
 * Aneesh V	<aneesh@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
/*
 * All OMAP boards of a given revision todate use the same memory
 * configuration. So keeping this here instead of keeping in board
 * directory. If a new board has different memory part/configuration
 * in future the weakly linked alias for ddr_init() can be over-ridden
 */
#include <common.h>
#include <asm/io.h>

/* Timing regs for Samsung K3PE7E700M */

const struct ddr_regs ddr_regs_samsung4G_400_mhz = {
	/* tRRD changed from 10ns to 12.5ns because of the tFAW requirement*/
	.tim1		= 0x10eb0662,
	.tim2		= 0x20370dd2,
	.tim3		= 0x00b1c33f,
	.phy_ctrl_1	= 0x849FF408,
	.ref_ctrl	= 0x00000618,
	.config_init	= 0x80000eb2,
	.config_final	= 0x80001ab2,
	.zq_config	= 0xD00b3215,
	.mr1		= 0x83,
	.mr2		= 0x4
};

/* ddr_init() - initializes ddr */
void __ddr_init_samsung(void)
{
	u32 rev;
	const struct ddr_regs *ddr_regs = 0;


	ddr_regs = &ddr_regs_samsung4G_400_mhz;

	/*
	 * DMM Configuration:
	 * 1GB - 128 byte interleaved
	 * We also need to make sure the whole 2GB of the DMM are mapped
	 * to workaround i614.
	 */
	__raw_writel(0x80640300, DMM_BASE + DMM_LISA_MAP_1);
	__raw_writel(0x80760300, DMM_BASE + DMM_LISA_MAP_0);

	/* same memory part on both EMIFs */
	do_ddr_init(ddr_regs, ddr_regs);
}

void ddr_init_samsung(void)
	__attribute__((weak, alias("__ddr_init_samsung")));
