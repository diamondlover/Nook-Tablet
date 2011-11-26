/*
 * Copyright (C) 2005 Texas Instruments.
 *
 * (C) Copyright 2004
 * Jian Zhang, Texas Instruments, jzhang@ti.com.
 *
 * (C) Copyright 2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
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

#include <common.h>
#include <part.h>
#include <fat.h>
#include <mmc.h>
#include <i2c.h>
#include "omap4_hs.h"

typedef struct {
	u32 image;
	u32  * data;
	u32 * res;
	u32 val;
} image_type;

#ifdef CFG_PRINTF
int print_info(void)
{
	printf ("\n\nTexas Instruments X-Loader 1.41 ("
		__DATE__ " - " __TIME__ ")\n");
	return 0;
}
#endif
typedef int (init_fnc_t) (void);

init_fnc_t *init_sequence[] = {
	cpu_init,		/* basic cpu dependent setup */
	board_init,		/* basic board dependent setup */
#ifdef CFG_PRINTF
 	serial_init,		/* serial communications setup */
	print_info,
#endif
   	//nand_init,		/* board specific nand init */
	NULL,
};

#ifdef CFG_CMD_FAT
extern char * strcpy(char * dest,const char *src);
#else
char * strcpy(char * dest,const char *src)
{
	 char *tmp = dest;

	 while ((*dest++ = *src++) != '\0')
	         /* nothing */;
	 return tmp;
}
#endif

#ifdef CFG_CMD_MMC
extern block_dev_desc_t *mmc_get_dev(int dev);
int mmc_read_bootloader(int dev)
{
	unsigned char ret = 0;
	unsigned long offset = ( CFG_LOADADDR - 0x120 );

	ret = mmc_init(dev);
	if (ret != 0){
		printf("\n MMC init failed \n");
		return -1;
	}

#ifdef CFG_CMD_FAT
	long size;
	block_dev_desc_t *dev_desc = NULL;

	if (fat_boot()) {
		dev_desc = mmc_get_dev(dev);
		fat_register_device(dev_desc, 1);
		size = file_fat_read("u-boot.bin", (unsigned char *)offset, 0);
		if (size == -1)
			return -1;
	} else {
		/* FIXME: OMAP4 specific */
		 mmc_read(dev, 0x200, (unsigned char *)(CFG_LOADADDR-0x120),
							0x00060000);
	}
#endif
	return 0;
}
#endif

extern int do_load_serial_bin(ulong offset, int baudrate);

#define __raw_readl(a)	(*(volatile unsigned int *)(a))

/*************************************************************/
/* Check if button power on is still pressed after defined   */
/* duration. If not, power off the system.                   */
/*************************************************************/
#define OMAP4_32KSYNCNT_CR	0x4A304010
#define PWRON_CHECK_DURATION	330 /* 32768 = 1s */
#define TWL6030_PMC_ID		0x48
#define STS_HW_CONDITIONS	0x21
#define STS_PWRON			(1 << 0)
#define STS_PLUG_DET		(1 << 3)
#define PHOENIX_START_CONDITION	0x1F
#define STRT_ON_PWRON		(1 << 0)
#define STRT_ON_RPWRON		(1 << 1)
#define STRT_ON_USB_ID		(1 << 2)
#define STRT_ON_PLUG_DET	(1 << 3)
#define STRT_ON_RTC		(1 << 4)
#define FIRST_BAT_INS		(1 << 5)
#define RESTART_BB		(1 << 6)
#define DISABLED_STRT_EVENTS	(STRT_ON_RTC)
#define PHOENIX_DEV_ON		0x25
#define PUBLIC_SAR_RAM_1_FREE  (0x4a326000 + 0xA0C)
#define APP_DEV_OFF		(1 << 0)
#define CON_DEV_OFF		(1 << 1)
#define MOD_DEV_OFF		(1 << 2)
#define SWITCH_OFF		(APP_DEV_OFF | CON_DEV_OFF | MOD_DEV_OFF)

int select_bus(int bus, int speed);

void pwron_check_switch_off(void)
{
	u32 t32k_t1, t32k_t2;
	u8 val;

	select_bus(CFG_I2C_BUS, CFG_I2C_SPEED);

	/* if start is on disabled start event, switch off the board */
	i2c_read(TWL6030_PMC_ID, PHOENIX_START_CONDITION, 1, &val, 1);
	if (val & DISABLED_STRT_EVENTS) {
		printf("Start on disabled event.\n");
		goto switch_off;
	}

	/* if start is not on PWRON, skip the power button check */
	if (!(val & STRT_ON_PWRON)) {
		printf("Start not on PWRON, skipping power button check.\n");
		return;
	}

	/* if USB or charger is detected, skip the check so the user
	 * doesn't need to keep the button pressed for 5sec
	 */
	i2c_read(TWL6030_PMC_ID, STS_HW_CONDITIONS, 1, &val, 1);

	if (val & STS_PLUG_DET) {
		printf("Plug detected, skipping power button check.\n");
		return;
	}

	/* if start is not on PWRON skip the power button check */
	i2c_read(TWL6030_PMC_ID, PHOENIX_START_CONDITION, 1, &val, 1);

	if (val & STRT_ON_RTC) {
		printf("Start on RTC, powering down\n");
		goto switch_off;
	}

	/* Check SAR RAM for boot code, if there is one we know
	   boot was initiated by the Linux kernel, aka reboot */
	if (!strcmp((const char *) PUBLIC_SAR_RAM_1_FREE, "reboot") ||
		!strcmp((const char *) PUBLIC_SAR_RAM_1_FREE, "recovery") ||
		!strcmp((const char *) PUBLIC_SAR_RAM_1_FREE, "bootloader")) {
		printf("Boot code found, skipping power button check.\n");
		return;
	}

	printf("Checking power button state... ");

	t32k_t1 = t32k_t2 = *((volatile u32 *)OMAP4_32KSYNCNT_CR);
	do {
		if ((t32k_t2 - t32k_t1) % 328 == 0) {
			/* Check poweron button status every 10ms */
			i2c_read(TWL6030_PMC_ID, STS_HW_CONDITIONS, 1, &val, 1);
			if (val & STS_PWRON) {
				printf("RELEASED. NOK.\n");
				goto switch_off;
			}
		}
		t32k_t2 = *((volatile u32 *)OMAP4_32KSYNCNT_CR);
	} while (t32k_t2 - t32k_t1 < PWRON_CHECK_DURATION);

	printf("PRESSED. OK\n");
	return;

switch_off:
	printf("Powering off!\n");
	val = SWITCH_OFF;
	i2c_write(TWL6030_PMC_ID, PHOENIX_DEV_ON, 1, &val, 1);
	/* we should never get here */
	hang();
}

void start_armboot (void)
{
  	init_fnc_t **init_fnc_ptr;
	uchar *buf;
	char boot_dev_name[8];
	image_type image;
 
   	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		if ((*init_fnc_ptr)() != 0) {
			hang ();
		}
	}

	pwron_check_switch_off();

	image.image = 2;
	image.val = 99;

#ifdef START_LOADB_DOWNLOAD
	strcpy(boot_dev_name, "UART");
	do_load_serial_bin (CFG_LOADADDR, 115200);
#else
	buf = (uchar *) ( CFG_LOADADDR - 0x120 );
	image.data = (uchar *) (CFG_LOADADDR-0x120);

	switch (get_boot_device()) {
	case 0x03:
		strcpy(boot_dev_name, "ONENAND");
#if defined(CFG_ONENAND)
		for (i = ONENAND_START_BLOCK; i < ONENAND_END_BLOCK; i++) {
			if (!onenand_read_block(buf, i))
				buf += ONENAND_BLOCK_SIZE;
			else
				goto error;
		}
#endif
		break;
	case 0x02:
	default:
		strcpy(boot_dev_name, "NAND");
#if defined(CFG_NAND)
		for (i = NAND_UBOOT_START; i < NAND_UBOOT_END;
				i+= NAND_BLOCK_SIZE) {
			if (!nand_read_block(buf, i))
				buf += NAND_BLOCK_SIZE; /* advance buf ptr */
		}
#endif
		break;
	case 0x05:
		strcpy(boot_dev_name, "MMC/SD1");
#if defined(CONFIG_MMC)
		if (mmc_read_bootloader(0) != 0)
			goto error;
#endif
		break;
	case 0x06:
		strcpy(boot_dev_name, "EMMC");
#if defined(CONFIG_MMC)
		if (mmc_read_bootloader(1) != 0)
			goto error;
#endif
		break;
	};
#endif
	SEC_ENTRY_Std_Ppa_Call ( PPA_SERV_HAL_BN_CHK , 1 , &image );

	if ( image.val == 0 ) {
		/* go run U-Boot and never return */
		printf("Starting OS Bootloader from %s ...\n", boot_dev_name);
		((init_fnc_t *)CFG_LOADADDR)();
	}
	/* should never come here */
#if defined(CFG_ONENAND) || defined(CONFIG_MMC)
error:
#endif
	printf("Could not read bootloader!\n");
	hang();
}

void hang (void)
{
	/* call board specific hang function */
	board_hang();
	
	/* if board_hang() returns, hang here */
	printf("X-Loader hangs\n");
	for (;;);
}
