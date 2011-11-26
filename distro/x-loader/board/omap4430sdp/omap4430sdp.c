/*
 * (C) Copyright 2004-2009
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
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
#include <common.h>
#include <asm/arch/cpu.h>
#include <asm/io.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include <i2c.h>
#include <asm/mach-types.h>
#if (CONFIG_COMMANDS & CFG_CMD_NAND) && defined(CFG_NAND_LEGACY)
#include <linux/mtd/nand_legacy.h>
#endif
#include <omap4_hs.h>

#define CONFIG_OMAP4_SDC		1
#define XLOADER_SV 0

typedef enum
{
	NO_ERROR,
	FAILED,
	TIMED_OUT,
	PARAM_ERROR,
	WAITING,
	MALLOC_ERROR,
	INVALID_POINTER
} STATUS;

extern STATUS pmic_set_vpp(void);
extern STATUS pmic_close_vpp(void);


/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
	static unsigned int sw_ver = XLOADER_SV;

	pmic_set_vpp();

	SEC_ENTRY_Std_Ppa_Call( PPA_SERV_HAL_BN_INIT,1,&sw_ver);

	pmic_close_vpp();

	return 0;
}


#define		OMAP44XX_WKUP_CTRL_BASE		0x4A31E000
#if 1
#define M0_SAFE M0
#define M1_SAFE M1
#define M2_SAFE M2
#define M4_SAFE M4
#define M7_SAFE M7
#define M3_SAFE M3
#define M5_SAFE M5
#define M6_SAFE M6
#else
#define M0_SAFE M7
#define M1_SAFE M7
#define M2_SAFE M7
#define M4_SAFE M7
#define M7_SAFE M7
#define M3_SAFE M7
#define M5_SAFE M7
#define M6_SAFE M7
#endif

/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */

#define MUX_DEFAULT_OMAP4() \
	MV(CP(GPMC_AD0),	(IEN | M1)) /* sdmmc2_dat0 */ \
	MV(CP(GPMC_AD1),	(IEN | M1)) /* sdmmc2_dat1 */ \
	MV(CP(GPMC_AD2),	(IEN | M1)) /* sdmmc2_dat2 */ \
	MV(CP(GPMC_AD3),	(IEN | M1)) /* sdmmc2_dat3 */ \
	MV(CP(GPMC_AD4),	(IEN | M1)) /* sdmmc2_dat4 */ \
	MV(CP(GPMC_AD5),	(IEN | M1)) /* sdmmc2_dat5 */ \
	MV(CP(GPMC_AD6),	(IEN | M1)) /* sdmmc2_dat6 */ \
	MV(CP(GPMC_AD7),	(IEN | M1)) /* sdmmc2_dat7 */ \
	MV(CP(GPMC_AD8) , ( IEN | OFF_EN | OFF_IN | M3))  /* gpio_32 */ \
	MV(CP(GPMC_AD9) , ( IEN | M3))  /* gpio_33 */ \
	MV(CP(GPMC_AD10) , ( IEN | M3))  /* gpio_34 */ \
	MV(CP(GPMC_AD11) , ( IEN | M3))  /* gpio_35 */ \
	MV(CP(GPMC_AD12),	(M3_SAFE)) /* LCD-PWR-EN gpio_36 */ \
	MV(CP(GPMC_AD13) , ( IEN | M3))  /* gpio_37 */ \
	MV(CP(GPMC_AD14),	(OFF_EN | OFF_PD | OFF_OUT_PTD | M3)) /* BL-PWR_nEN gpio_38 */ \
	MV(CP(GPMC_AD15),	(M3)) /* TP-RESET gpio_39 */ \
	MV(CP(GPMC_A16) , ( IEN | M3))  /* gpio_40 */ \
	MV(CP(GPMC_A17) , ( IEN | M3))  /* gpio_41 */ \
	MV(CP(GPMC_A18) , ( M7))  /* not used */ \
	MV(CP(GPMC_A19) , ( M7))  /* not used */ \
	MV(CP(GPMC_A20),	(M3_SAFE)) /*lcd-cab0 gpio_44 */ \
	MV(CP(GPMC_A21),	(M3_SAFE)) /*lcd-cab1 gpio_45 */ \
	MV(CP(GPMC_A22) , ( M7))  /* not used */ \
	MV(CP(GPMC_A23) , ( M7))  /* not used */ \
	MV(CP(GPMC_A24) , ( M7))  /* not used */ \
	MV(CP(GPMC_A25) , ( IEN | M3))  /* gpio_49 */ \
	MV(CP(GPMC_NCS0) , ( IEN | M3))  /* gpio_50 */ \
	MV(CP(GPMC_NCS1) , ( IEN | M3))  /* gpio_51 */ \
	MV(CP(GPMC_NCS2) , ( M7))  /* not used */ \
	MV(CP(GPMC_NCS3) , ( M3))  /* gpio_53 */ \
	MV(CP(GPMC_NWP) , ( M7))  /* not used */ \
	MV(CP(GPMC_CLK),	(IEN | M3_SAFE)) /* gpio_55 */ \
	MV(CP(GPMC_NADV_ALE) , ( M7))  /* not used */ \
	MV(CP(GPMC_NOE) , ( PTU | IEN | OFF_EN | OFF_OUT_PTD | M1))  /* sdmmc2_clk */ \
	MV(CP(GPMC_NWE),	(IEN | M1)) /* sdmmc2_cmd */ \
	MV(CP(GPMC_NBE0_CLE) , ( M7))  /* not used */ \
	MV(CP(GPMC_NBE1),	(M3)) /* gpio_60 */ \
	MV(CP(GPMC_WAIT0) , ( M7))  /* not used */ \
	MV(CP(GPMC_WAIT1) , ( IEN | M3))  /* gpio_62 */ \
	MV(CP(C2C_DATA11) , ( PTD | IEN | M3))  /* gpio_100 */ \
	MV(CP(C2C_DATA12) , ( M3))  /* gpio_101 */ \
	MV(CP(C2C_DATA13) , ( IEN | M3))  /* gpio_102 */ \
	MV(CP(C2C_DATA14) , ( IEN | M3))  /* gpio_103 */ \
	MV(CP(C2C_DATA15) , ( M3))  /* gpio_104 */ \
	MV(CP(HDMI_HPD) , ( M3))  /* gpio_63 */ \
	MV(CP(HDMI_CEC) , ( M3))  /* gpio_64 */ /* M3 for EVT1b only */ \
	MV(CP(HDMI_DDC_SCL) , ( IEN | M3))  /* gpio_65 */ \
	MV(CP(HDMI_DDC_SDA) , ( IEN | M3))  /* gpio_66 */ \
	MV(CP(CSI21_DX0) , ( M7))  /* not used */ \
	MV(CP(CSI21_DY0) , ( M7))  /* not used */ \
	MV(CP(CSI21_DX1) , ( M7))  /* not used */ \
	MV(CP(CSI21_DY1) , ( M7))  /* not used */ \
	MV(CP(CSI21_DX2) , ( M7))  /* not used */ \
	MV(CP(CSI21_DY2) , ( M7))  /* not used */ \
	MV(CP(CSI21_DX3) , ( M7))  /* not used */ \
	MV(CP(CSI21_DY3) , ( M7))  /* not used */ \
	MV(CP(CSI21_DX4) , ( M7))  /* not used */ \
	MV(CP(CSI21_DY4) , ( M7))  /* not used */ \
	MV(CP(CSI22_DX0) , ( M7))  /* not used */ \
	MV(CP(CSI22_DY0) , ( M7))  /* not used */ \
	MV(CP(CSI22_DX1) , ( M7))  /* not used */ \
	MV(CP(CSI22_DY1) , ( M7))  /* not used */ \
	MV(CP(CAM_SHUTTER) , ( IEN | M3))  /* gpio_81 */ \
	MV(CP(CAM_STROBE) , ( IEN | M3))  /* gpio_82 */ \
	MV(CP(CAM_GLOBALRESET) , ( M3))  /* gpio_83 */ \
	MV(CP(USBB1_ULPITLL_CLK),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_clk */ \
	MV(CP(USBB1_ULPITLL_STP),	(PTU | OFF_EN | OFF_OUT_PTD | M4)) /* usbb1_ulpiphy_stp */ \
	MV(CP(USBB1_ULPITLL_DIR),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_dir */ \
	MV(CP(USBB1_ULPITLL_NXT),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_nxt */ \
	MV(CP(USBB1_ULPITLL_DAT0),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_dat0 */ \
	MV(CP(USBB1_ULPITLL_DAT1),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_dat1 */ \
	MV(CP(USBB1_ULPITLL_DAT2),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_dat2 */ \
	MV(CP(USBB1_ULPITLL_DAT3),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_dat3 */ \
	MV(CP(USBB1_ULPITLL_DAT4),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_dat4 */ \
	MV(CP(USBB1_ULPITLL_DAT5),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_dat5 */ \
	MV(CP(USBB1_ULPITLL_DAT6),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_dat6 */ \
	MV(CP(USBB1_ULPITLL_DAT7),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M4)) /* usbb1_ulpiphy_dat7 */ \
	MV(CP(USBB1_HSIC_DATA),		(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_data */ \
	MV(CP(USBB1_HSIC_STROBE),	(PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0)) /* usbb1_hsic_strobe */ \
	MV(CP(USBC1_ICUSB_DP) , ( IEN | M3))  /* gpio_98 */ \
	MV(CP(USBC1_ICUSB_DM) , ( IEN | M3))  /* gpio_99 */ \
	MV(CP(SDMMC1_CLK),	(OFF_EN | OFF_OUT_PTD | M0)) /* sdmmc1_clk */ \
	MV(CP(SDMMC1_CMD),	(IEN | M0)) /* sdmmc1_cmd */ \
	MV(CP(SDMMC1_DAT0),	(IEN | M0)) /* sdmmc1_dat0 */ \
	MV(CP(SDMMC1_DAT1),	(IEN | M0)) /* sdmmc1_dat1 */ \
	MV(CP(SDMMC1_DAT2),	(IEN | M0)) /* sdmmc1_dat2 */ \
	MV(CP(SDMMC1_DAT3),	(IEN | M0)) /* sdmmc1_dat3 */ \
	MV(CP(SDMMC1_DAT4) , ( M7))  /* not used */ \
	MV(CP(SDMMC1_DAT5) , ( M7))  /* not used */ \
	MV(CP(SDMMC1_DAT6) , ( M7))  /* not used */ \
	MV(CP(SDMMC1_DAT7) , ( M7))  /* not used */ \
	MV(CP(ABE_MCBSP2_CLKX) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_mcbsp2_clkx */ \
	MV(CP(ABE_MCBSP2_DR) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_mcbsp2_dr */ \
	MV(CP(ABE_MCBSP2_DX) , ( OFF_EN | OFF_OUT_PTD | M0))  /* abe_mcbsp2_dx */ \
	MV(CP(ABE_MCBSP2_FSX) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_mcbsp2_fsx */ \
	MV(CP(ABE_MCBSP1_CLKX),	( M3))  /* gpio_114 */ \
	MV(CP(ABE_MCBSP1_DR) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M3))  /* gpio_115 */ \
	MV(CP(ABE_MCBSP1_DX),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc3_dat2 */ \
	MV(CP(ABE_MCBSP1_FSX),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc3_dat3 */ \
	MV(CP(ABE_PDM_UL_DATA) , ( M7))  /* not used */ \
	MV(CP(ABE_PDM_DL_DATA) , ( M7))  /* not used */ \
	MV(CP(ABE_PDM_FRAME) , ( M7))  /* not used */ \
	MV(CP(ABE_PDM_LB_CLK) , ( M7))  /* not used */ \
	MV(CP(ABE_CLKS) , ( M3))  /* gpio_118 */ \
	MV(CP(ABE_DMIC_CLK1) , ( M7))  /* not used */ \
	MV(CP(ABE_DMIC_DIN1) , ( M7))  /* not used */ \
	MV(CP(ABE_DMIC_DIN2) , ( M5))  /* dmtimer11_pwm_evt */ \
	MV(CP(ABE_DMIC_DIN3) , ( M7))  /* not used */ \
	MV(CP(UART2_CTS),	(PTU | IEN | OFF_EN | OFF_OUT_PTD | M1))  /* sdmmc3_clk */ \
	MV(CP(UART2_RTS),	(PTU | IEN | OFF_EN | OFF_OUT_PTD | M1))  /* sdmmc3_cmd */ \
	MV(CP(UART2_RX),	(PTU | IEN | OFF_EN | OFF_PU | OFF_IN | M1))  /* sdmmc3_dat0 */ \
	MV(CP(UART2_TX),	(PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc3_dat1 */ \
	MV(CP(HDQ_SIO),		(M3_SAFE)) /* gpio_127 */ \
	MV(CP(I2C1_SCL),	(IEN | M0)) /* i2c1_scl */ \
	MV(CP(I2C1_SDA),	(IEN | M0)) /* i2c1_sda */ \
	MV(CP(I2C2_SCL),	(IEN | M0)) /* i2c2_scl */ \
	MV(CP(I2C2_SDA),	(IEN | M0)) /* i2c2_sda */ \
	MV(CP(I2C3_SCL) , ( M7))  /* not used */ \
	MV(CP(I2C3_SDA) , ( M7))  /* not used */ \
	MV(CP(I2C4_SCL) , ( M7))  /* not used */ \
	MV(CP(I2C4_SDA) , ( M7))  /* not used */ \
	MV(CP(MCSPI1_CLK) , ( M7))  /* not used */ \
	MV(CP(MCSPI1_SOMI) , ( M7))  /* not used */ \
	MV(CP(MCSPI1_SIMO) , ( M7))  /* not used */ \
	MV(CP(MCSPI1_CS0) , ( M7))  /* not used */ \
	MV(CP(MCSPI1_CS1),	(IEN | PTU | EN | M1)) \
	MV(CP(MCSPI1_CS2),	(M7)) /* uart1_cts */ \
	MV(CP(MCSPI1_CS3),	(M7)) \
	MV(CP(UART3_CTS_RCTX) , ( M1))  /* uart1_tx */ \
	MV(CP(UART3_RTS_SD) , ( M7))  /* not used */ \
	MV(CP(UART3_RX_IRRX),	(M3)) /* gpio_143*/ \
	MV(CP(UART3_TX_IRTX) , ( M7))  /* not used */ \
	MV(CP(SDMMC5_CLK) , ( M7))  /* not used */ \
	MV(CP(SDMMC5_CMD) , ( M7))  /* not used */ \
	MV(CP(SDMMC5_DAT0) , ( M7))  /* not used */ \
	MV(CP(SDMMC5_DAT1) , ( M7))  /* not used */ \
	MV(CP(SDMMC5_DAT2) , ( M7))  /* not used */ \
	MV(CP(SDMMC5_DAT3) , ( M7))  /* not used */ \
	MV(CP(MCSPI4_CLK) , ( IEN | OFF_EN | OFF_IN | M0))  /* mcspi4_clk */ \
	MV(CP(MCSPI4_SIMO) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi4_simo */ \
	MV(CP(MCSPI4_SOMI) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi4_somi */ \
	MV(CP(MCSPI4_CS0),	(PTD | IEN | EN | OFF_EN | OFF_PD | OFF_IN | M0)) /* mcspi4_cs0 */ \
	MV(CP(UART4_RX) , ( M7))  /* not used */ \
	MV(CP(UART4_TX) , ( M7))  /* not used */ \
	MV(CP(USBB2_ULPITLL_CLK) , ( M7))  /* not used */ \
	MV(CP(USBB2_ULPITLL_STP),	(M5)) /* dispc2_data23 */ \
	MV(CP(USBB2_ULPITLL_DIR),	(M5)) /* dispc2_data22 */ \
	MV(CP(USBB2_ULPITLL_NXT),	(M5)) /* dispc2_data21 */ \
	MV(CP(USBB2_ULPITLL_DAT0),	(M5)) /* dispc2_data20 */ \
	MV(CP(USBB2_ULPITLL_DAT1),	(M5)) /* dispc2_data19 */ \
	MV(CP(USBB2_ULPITLL_DAT2),	(M5)) /* dispc2_data18 */ \
	MV(CP(USBB2_ULPITLL_DAT3),	(M5)) /* dispc2_data15 */ \
	MV(CP(USBB2_ULPITLL_DAT4),	(M5)) /* dispc2_data14 */ \
	MV(CP(USBB2_ULPITLL_DAT5),	(M5)) /* dispc2_data13 */ \
	MV(CP(USBB2_ULPITLL_DAT6),	(M5)) /* dispc2_data12 */ \
	MV(CP(USBB2_ULPITLL_DAT7),	(M5)) /* dispc2_data11 */ \
	MV(CP(USBB2_HSIC_DATA) , ( M7))  /* not used */ \
	MV(CP(USBB2_HSIC_STROBE) , ( M7))  /* not used */ \
	MV(CP(UNIPRO_TX0) , ( PTU | IEN | M3))  /* gpio_171 : MODEM-USB-EN */ \
	MV(CP(UNIPRO_TY0) , ( PTU | IEN | M3))  /* gpio_172 : MODEM-ON */ \
	MV(CP(UNIPRO_TX1) , ( M3 ))      /* gpio_173 */ \
	MV(CP(UNIPRO_TY1) , ( M7_SAFE))  /* kpd_col3 configured by kernel */ \
	MV(CP(UNIPRO_TX2) , ( M7_SAFE))  /* kpd_col4 configured by kernel */ \
	MV(CP(UNIPRO_TY2) , ( M7_SAFE))  /* kpd_col5 configured by kernel */ \
	MV(CP(UNIPRO_RX0) , ( M7_SAFE))  /* kpd_row0 configured by kernel */ \
	MV(CP(UNIPRO_RY0) , ( M7_SAFE))  /* kpd_row1 configured by kernel */ \
	MV(CP(UNIPRO_RX1) , ( M7_SAFE))  /* kpd_row2 configured by kernel */ \
	MV(CP(UNIPRO_RY1) , ( M7_SAFE))  /* kpd_row3 configured by kernel */ \
	MV(CP(UNIPRO_RX2) , ( M7_SAFE))  /* kpd_row4 configured by kernel */ \
	MV(CP(UNIPRO_RY2) , ( M7_SAFE))  /* kpd_row5 configured by kernel */ \
	MV(CP(USBA0_OTG_CE) , ( M0))  /* not used */ \
	MV(CP(USBA0_OTG_DP) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* usba0_otg_dp */ \
	MV(CP(USBA0_OTG_DM) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* usba0_otg_dm */ \
	MV(CP(FREF_CLK1_OUT) , ( M0))  /* fref_clk1_out */ \
	MV(CP(FREF_CLK2_OUT) , ( M7))  /* not used */ \
	MV(CP(SYS_NIRQ1) , ( PTU | IEN | M0))  /* sys_nirq1 */ \
	MV(CP(SYS_NIRQ2) , ( M7))  /* not used */ \
	MV(CP(SYS_BOOT0) , ( M0))  /* SYS_BOOT0 */ \
	MV(CP(SYS_BOOT1) , ( M0))  /* SYS_BOOT1 */ \
	MV(CP(SYS_BOOT2) , ( M0))  /* SYS_BOOT2 */ \
	MV(CP(SYS_BOOT3) , ( M0))  /* SYS_BOOT3 */ \
	MV(CP(SYS_BOOT4) , ( M0))  /* SYS_BOOT4 */ \
	MV(CP(SYS_BOOT5) , ( M0))  /* SYS_BOOT5 */ \
	MV(CP(DPM_EMU0) , ( IEN | M0))  /* dpm_emu0 */ \
	MV(CP(DPM_EMU1) , ( IEN | M0))  /* dpm_emu1 */ \
	MV(CP(DPM_EMU2) , ( M7_SAFE))  /* not used */ \
	MV(CP(DPM_EMU3),	(M5)) /* dispc2_data10 */ \
	MV(CP(DPM_EMU4),	(M5)) /* dispc2_data9 */ \
	MV(CP(DPM_EMU5),	(M5)) /* dispc2_data16 */ \
	MV(CP(DPM_EMU6),	(M5)) /* dispc2_data17 */ \
	MV(CP(DPM_EMU7),	(M5)) /* dispc2_hsync */ \
	MV(CP(DPM_EMU8),	(M5)) /* dispc2_pclk */ \
	MV(CP(DPM_EMU9),	(M5)) /* dispc2_vsync */ \
	MV(CP(DPM_EMU10),	(M5)) /* dispc2_de */ \
	MV(CP(DPM_EMU11),	(M5)) /* dispc2_data8 */ \
	MV(CP(DPM_EMU12),	(M5)) /* dispc2_data7 */ \
	MV(CP(DPM_EMU13),	(M5)) /* dispc2_data6 */ \
	MV(CP(DPM_EMU14),	(M5)) /* dispc2_data5 */ \
	MV(CP(DPM_EMU15),	(M5)) /* dispc2_data4 */ \
	MV(CP(DPM_EMU16),	(M5)) /* dispc2_data3/dmtimer8_pwm_evt */ \
	MV(CP(DPM_EMU17),	(M5)) /* dispc2_data2 */ \
	MV(CP(DPM_EMU18),	(M5)) /* dispc2_data1 */ \
	MV(CP(DPM_EMU19),	(M5)) /* dispc2_data0 */ \
	MV1(WK(PAD0_SIM_IO) , ( M7_SAFE))  /* not used */ \
	MV1(WK(PAD1_SIM_CLK) , ( M7_SAFE))  /* not used */ \
	MV1(WK(PAD0_SIM_RESET) , ( M7_SAFE))  /* not used */ \
	MV1(WK(PAD1_SIM_CD) , ( M7_SAFE))  /* not used */ \
	MV1(WK(PAD0_SIM_PWRCTRL) , ( M7_SAFE))  /* not used */ \
	MV1(WK(PAD1_SR_SCL),	(IEN | M0)) /* sr_scl */ \
	MV1(WK(PAD0_SR_SDA),	(IEN | M0)) /* sr_sda */ \
	MV1(WK(PAD1_FREF_XTAL_IN) , ( M0))  /* not used */ \
	MV1(WK(PAD0_FREF_SLICER_IN) , ( M0))  /* fref_slicer_in */ \
	MV1(WK(PAD1_FREF_CLK_IOREQ) , ( M0))  /* fref_clk_ioreq */ \
	MV1(WK(PAD0_FREF_CLK0_OUT) , ( M0))  /* fref_clk0_out */ \
	MV1(WK(PAD1_FREF_CLK3_REQ),	(M0_SAFE)) /* # */ \
	MV1(WK(PAD0_FREF_CLK3_OUT),	(M0_SAFE)) /* fref_clk3_out */ \
	MV1(WK(PAD1_FREF_CLK4_REQ),	(M0_SAFE)) /* # */ \
	MV1(WK(PAD0_FREF_CLK4_OUT),	(M0_SAFE)) /* # */ \
	MV1(WK(PAD1_SYS_32K) , ( IEN | M0))  /* sys_32k */ \
	MV1(WK(PAD0_SYS_NRESPWRON) , ( M0))  /* sys_nrespwron */ \
	MV1(WK(PAD1_SYS_NRESWARM) , ( M0))  /* sys_nreswarm */ \
	MV1(WK(PAD0_SYS_PWR_REQ) , ( PTU | M0))  /* sys_pwr_req */ \
	MV1(WK(PAD1_SYS_PWRON_RESET) , ( IEN | M3))  /* gpio_wk29 */ \
	MV1(WK(PAD0_SYS_BOOT6) , ( M0))  /* SYS_BOOT6 */ \
	MV1(WK(PAD1_SYS_BOOT7) , ( M0))  /* SYS_BOOT7 */

/**********************************************************
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers
 *              specific to the hardware. Many pins need
 *              to be moved from protect to primary mode.
 *********************************************************/
void set_muxconf_regs(void)
{
	MUX_DEFAULT_OMAP4();
	return;
}

/* optionally do something like blinking LED */
void board_hang (void)
{ while (0) {};}
