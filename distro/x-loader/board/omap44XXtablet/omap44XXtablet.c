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

#define CONFIG_OMAP4_SDC		1

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
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
	MV(CP(GPMC_AD0) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat0 */ \
	MV(CP(GPMC_AD1) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat1 */ \
	MV(CP(GPMC_AD2) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat2 */ \
	MV(CP(GPMC_AD3) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat3 */ \
	MV(CP(GPMC_AD4) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat4 */ \
	MV(CP(GPMC_AD5) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat5 */ \
	MV(CP(GPMC_AD6) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat6 */ \
	MV(CP(GPMC_AD7) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_dat7 */ \
	MV(CP(GPMC_AD8) , ( PTU | IEN | M3))  /* gpio_32 */ \
	MV(CP(GPMC_AD9) , ( PTU | IEN | M3))  /* gpio_33 */ \
	MV(CP(GPMC_AD10) , ( PTU | IEN | M3))  /* gpio_34 */ \
	MV(CP(GPMC_AD11) , ( PTU | IEN | M3))  /* gpio_35 */ \
	MV(CP(GPMC_AD12) , ( PTU | IEN | M3))  /* gpio_36 */ \
	MV(CP(GPMC_AD13) , ( PTD | M3))  /* gpio_37 */ \
	MV(CP(GPMC_AD14) , ( PTD | M3))  /* gpio_38 */ \
	MV(CP(GPMC_AD15) , ( PTD | M3))  /* gpio_39 */ \
	MV(CP(GPMC_A16) , ( PTD | M3))  /* gpio_40 */ \
	MV(CP(GPMC_A17) , ( PTD | M3))  /* gpio_41 */ \
	MV(CP(GPMC_A18) , ( PTD | IEN | M3))  /* gpio_42 */ \
	MV(CP(GPMC_A19) , ( PTU | IEN | M3))  /* gpio_43 */ \
	MV(CP(GPMC_A20) , ( PTD | IEN | M3))  /* gpio_44 */ \
	MV(CP(GPMC_A21) , ( PTD | M3))  /* gpio_45 */ \
	MV(CP(GPMC_A22) , ( PTU | IEN | M3))  /* gpio_46 */ \
	MV(CP(GPMC_A23) , ( PTU | IEN | M3))  /* gpio_47 */ \
	MV(CP(GPMC_A24) , ( PTD | M3))  /* gpio_48 */ \
	MV(CP(GPMC_A25) , ( PTD | M3))  /* gpio_49 */ \
	MV(CP(GPMC_NCS0) , ( M3))  /* gpio_50 */ \
	MV(CP(GPMC_NCS1) , ( PTU | IEN | M3))  /* gpio_51 */ \
	MV(CP(GPMC_NCS2) , ( PTU | IEN | M3))  /* gpio_52 */ \
	MV(CP(GPMC_NCS3) , ( PTU | IEN | M3))  /* gpio_53 */ \
	MV(CP(GPMC_NWP) , ( PTD | M3))  /* gpio_54 */ \
	MV(CP(GPMC_CLK) , ( PTD | M3))  /* gpio_55 */ \
	MV(CP(GPMC_NADV_ALE) , ( PTD | M3))  /* gpio_56 */ \
	MV(CP(GPMC_NOE) , ( PTU | IEN | OFF_EN | OFF_OUT_PTD | M1))  /* sdmmc2_clk */ \
	MV(CP(GPMC_NWE) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M1))  /* sdmmc2_cmd */ \
	MV(CP(GPMC_NBE0_CLE) , ( PTD | M3))  /* gpio_59 */ \
	MV(CP(GPMC_NBE1) , ( PTD | M3))  /* gpio_60 */ \
	MV(CP(GPMC_WAIT0) , ( PTU | IEN | M3))  /* gpio_61 */ \
	MV(CP(GPMC_WAIT1) , ( IEN | M3))  /* gpio_62 */ \
	MV(CP(C2C_DATA11) , ( PTD | M3))  /* gpio_100 */ \
	MV(CP(C2C_DATA12) , ( M1))  /* dsi1_te0 */ \
	MV(CP(C2C_DATA13) , ( PTD | M3))  /* gpio_102 */ \
	MV(CP(C2C_DATA14) , ( PTD | M7))  /* c2c_data14 */ \
	MV(CP(C2C_DATA15) , ( PTD | M7))  /* c2c_data15 */ \
	MV(CP(HDMI_HPD) , ( M0))  /* hdmi_hpd */ \
	MV(CP(HDMI_CEC) , ( M0))  /* hdmi_cec */ \
	MV(CP(HDMI_DDC_SCL) , ( PTU | M0))  /* hdmi_ddc_scl */ \
	MV(CP(HDMI_DDC_SDA) , ( PTU | IEN | M0))  /* hdmi_ddc_sda */ \
	MV(CP(CSI21_DX0) , ( IEN | M0))  /* csi21_dx0 */ \
	MV(CP(CSI21_DY0) , ( IEN | M0))  /* csi21_dy0 */ \
	MV(CP(CSI21_DX1) , ( IEN | M0))  /* csi21_dx1 */ \
	MV(CP(CSI21_DY1) , ( IEN | M0))  /* csi21_dy1 */ \
	MV(CP(CSI21_DX2) , ( IEN | M0))  /* csi21_dx2 */ \
	MV(CP(CSI21_DY2) , ( IEN | M0))  /* csi21_dy2 */ \
	MV(CP(CSI21_DX3) , ( PTD | M7))  /* csi21_dx3 */ \
	MV(CP(CSI21_DY3) , ( PTD | M7))  /* csi21_dy3 */ \
	MV(CP(CSI21_DX4) , ( PTD | OFF_EN | OFF_PD | OFF_IN | M7))  /* csi21_dx4 */ \
	MV(CP(CSI21_DY4) , ( PTD | OFF_EN | OFF_PD | OFF_IN | M7))  /* csi21_dy4 */ \
	MV(CP(CSI22_DX0) , ( IEN | M0))  /* csi22_dx0 */ \
	MV(CP(CSI22_DY0) , ( IEN | M0))  /* csi22_dy0 */ \
	MV(CP(CSI22_DX1) , ( IEN | M0))  /* csi22_dx1 */ \
	MV(CP(CSI22_DY1) , ( IEN | M0))  /* csi22_dy1 */ \
	MV(CP(CAM_SHUTTER) , ( OFF_EN | OFF_PD | OFF_OUT_PTD | M0))  /* cam_shutter */ \
	MV(CP(CAM_STROBE) , ( OFF_EN | OFF_PD | OFF_OUT_PTD | M0))  /* cam_strobe */ \
	MV(CP(CAM_GLOBALRESET) , ( PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M3))  /* gpio_83 */ \
	MV(CP(USBB1_ULPITLL_CLK) , ( IEN | OFF_EN | OFF_IN | M1))  /* hsi1_cawake */ \
	MV(CP(USBB1_ULPITLL_STP) , ( IEN | OFF_EN | OFF_IN | M1))  /* hsi1_cadata */ \
	MV(CP(USBB1_ULPITLL_DIR) , ( IEN | OFF_EN | OFF_IN | M1))  /* hsi1_caflag */ \
	MV(CP(USBB1_ULPITLL_NXT) , ( OFF_EN | M1))  /* hsi1_acready */ \
	MV(CP(USBB1_ULPITLL_DAT0) , ( OFF_EN | M1))  /* hsi1_acwake */ \
	MV(CP(USBB1_ULPITLL_DAT1) , ( OFF_EN | M1))  /* hsi1_acdata */ \
	MV(CP(USBB1_ULPITLL_DAT2) , ( OFF_EN | M1))  /* hsi1_acflag */ \
	MV(CP(USBB1_ULPITLL_DAT3) , ( IEN | OFF_EN | OFF_IN | M1))  /* hsi1_caready */ \
	MV(CP(USBB1_ULPITLL_DAT4) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M4))  /* usbb1_ulpiphy_dat4 */ \
	MV(CP(USBB1_ULPITLL_DAT5) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M4))  /* usbb1_ulpiphy_dat5 */ \
	MV(CP(USBB1_ULPITLL_DAT6) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M4))  /* usbb1_ulpiphy_dat6 */ \
	MV(CP(USBB1_ULPITLL_DAT7) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M4))  /* usbb1_ulpiphy_dat7 */ \
	MV(CP(USBB1_HSIC_DATA) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* usbb1_hsic_data */ \
	MV(CP(USBB1_HSIC_STROBE) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* usbb1_hsic_strobe */ \
	MV(CP(USBC1_ICUSB_DP) , ( IEN | M0))  /* usbc1_icusb_dp */ \
	MV(CP(USBC1_ICUSB_DM) , ( IEN | M0))  /* usbc1_icusb_dm */ \
	MV(CP(SDMMC1_CLK) , ( PTU | OFF_EN | OFF_OUT_PTD | M0))  /* sdmmc1_clk */ \
	MV(CP(SDMMC1_CMD) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc1_cmd */ \
	MV(CP(SDMMC1_DAT0) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc1_dat0 */ \
	MV(CP(SDMMC1_DAT1) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc1_dat1 */ \
	MV(CP(SDMMC1_DAT2) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc1_dat2 */ \
	MV(CP(SDMMC1_DAT3) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc1_dat3 */ \
	MV(CP(SDMMC1_DAT4) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc1_dat4 */ \
	MV(CP(SDMMC1_DAT5) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc1_dat5 */ \
	MV(CP(SDMMC1_DAT6) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc1_dat6 */ \
	MV(CP(SDMMC1_DAT7) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc1_dat7 */ \
	MV(CP(ABE_MCBSP2_CLKX) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_mcbsp2_clkx */ \
	MV(CP(ABE_MCBSP2_DR) , ( IEN | OFF_EN | OFF_OUT_PTD | M0))  /* abe_mcbsp2_dr */ \
	MV(CP(ABE_MCBSP2_DX) , ( OFF_EN | OFF_OUT_PTD | M0))  /* abe_mcbsp2_dx */ \
	MV(CP(ABE_MCBSP2_FSX) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_mcbsp2_fsx */ \
	MV(CP(ABE_MCBSP1_CLKX) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_mcbsp1_clkx */ \
	MV(CP(ABE_MCBSP1_DR) , ( IEN | OFF_EN | OFF_OUT_PTD | M0))  /* abe_mcbsp1_dr */ \
	MV(CP(ABE_MCBSP1_DX) , ( OFF_EN | OFF_OUT_PTD | M0))  /* abe_mcbsp1_dx */ \
	MV(CP(ABE_MCBSP1_FSX) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_mcbsp1_fsx */ \
	MV(CP(ABE_PDM_UL_DATA) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_pdm_ul_data */ \
	MV(CP(ABE_PDM_DL_DATA) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_pdm_dl_data */ \
	MV(CP(ABE_PDM_FRAME) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_pdm_frame */ \
	MV(CP(ABE_PDM_LB_CLK) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_pdm_lb_clk */ \
	MV(CP(ABE_CLKS) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* abe_clks */ \
	MV(CP(ABE_DMIC_CLK1) , ( M0))  /* abe_dmic_clk1 */ \
	MV(CP(ABE_DMIC_DIN1) , ( IEN | M0))  /* abe_dmic_din1 */ \
	MV(CP(ABE_DMIC_DIN2) , ( IEN | M0))  /* abe_dmic_din2 */ \
	MV(CP(ABE_DMIC_DIN3) , ( IEN | M0))  /* abe_dmic_din3 */ \
	MV(CP(UART2_CTS) , ( PTU | IEN | M0))  /* uart2_cts */ \
	MV(CP(UART2_RTS) , ( M0))  /* uart2_rts */ \
	MV(CP(UART2_RX) , ( PTU | IEN | M0))  /* uart2_rx */ \
	MV(CP(UART2_TX) , ( M0))  /* uart2_tx */ \
	MV(CP(HDQ_SIO) , ( M3))  /* gpio_127 */ \
	MV(CP(I2C1_SCL) , ( PTU | IEN | M0))  /* i2c1_scl */ \
	MV(CP(I2C1_SDA) , ( PTU | IEN | M0))  /* i2c1_sda */ \
	MV(CP(I2C2_SCL) , ( PTU | IEN | M0))  /* i2c2_scl */ \
	MV(CP(I2C2_SDA) , ( PTU | IEN | M0))  /* i2c2_sda */ \
	MV(CP(I2C3_SCL) , ( PTU | IEN | M0))  /* i2c3_scl */ \
	MV(CP(I2C3_SDA) , ( PTU | IEN | M0))  /* i2c3_sda */ \
	MV(CP(I2C4_SCL) , ( PTU | IEN | M0))  /* i2c4_scl */ \
	MV(CP(I2C4_SDA) , ( PTU | IEN | M0))  /* i2c4_sda */ \
	MV(CP(MCSPI1_CLK) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi1_clk */ \
	MV(CP(MCSPI1_SOMI) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi1_somi */ \
	MV(CP(MCSPI1_SIMO) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi1_simo */ \
	MV(CP(MCSPI1_CS0) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi1_cs0 */ \
	MV(CP(MCSPI1_CS1) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M3))  /* mcspi1_cs1 */ \
	MV(CP(MCSPI1_CS2) , ( PTU | OFF_EN | OFF_OUT_PTU | M3))  /* gpio_139 */ \
	MV(CP(MCSPI1_CS3) , ( PTU | M3))  /* gpio_140 */ \
	MV(CP(UART3_CTS_RCTX) , ( PTU | IEN | M0))  /* uart3_tx */ \
	MV(CP(UART3_RTS_SD) , ( M0))  /* uart3_rts_sd */ \
	MV(CP(UART3_RX_IRRX) , ( IEN | M0))  /* uart3_rx */ \
	MV(CP(UART3_TX_IRTX) , ( M0))  /* uart3_tx */ \
	MV(CP(SDMMC5_CLK) , ( PTU | IEN | OFF_EN | OFF_OUT_PTD | M0))  /* sdmmc5_clk */ \
	MV(CP(SDMMC5_CMD) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc5_cmd */ \
	MV(CP(SDMMC5_DAT0) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc5_dat0 */ \
	MV(CP(SDMMC5_DAT1) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc5_dat1 */ \
	MV(CP(SDMMC5_DAT2) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc5_dat2 */ \
	MV(CP(SDMMC5_DAT3) , ( PTU | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* sdmmc5_dat3 */ \
	MV(CP(MCSPI4_CLK) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi4_clk */ \
	MV(CP(MCSPI4_SIMO) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi4_simo */ \
	MV(CP(MCSPI4_SOMI) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi4_somi */ \
	MV(CP(MCSPI4_CS0) , ( PTD | IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* mcspi4_cs0 */ \
	MV(CP(UART4_RX) , ( IEN | M0))  /* uart4_rx */ \
	MV(CP(UART4_TX) , ( M0))  /* uart4_tx */ \
	MV(CP(USBB2_ULPITLL_CLK) , ( PTD | M3))  /* gpio_157 */ \
	MV(CP(USBB2_ULPITLL_STP) , ( IEN | M5))  /* dispc2_data23 */ \
	MV(CP(USBB2_ULPITLL_DIR) , ( IEN | M5))  /* dispc2_data22 */ \
	MV(CP(USBB2_ULPITLL_NXT) , ( IEN | M5))  /* dispc2_data21 */ \
	MV(CP(USBB2_ULPITLL_DAT0) , ( IEN | M5))  /* dispc2_data20 */ \
	MV(CP(USBB2_ULPITLL_DAT1) , ( IEN | M5))  /* dispc2_data19 */ \
	MV(CP(USBB2_ULPITLL_DAT2) , ( IEN | M5))  /* dispc2_data18 */ \
	MV(CP(USBB2_ULPITLL_DAT3) , ( IEN | M5))  /* dispc2_data15 */ \
	MV(CP(USBB2_ULPITLL_DAT4) , ( IEN | M5))  /* dispc2_data14 */ \
	MV(CP(USBB2_ULPITLL_DAT5) , ( IEN | M5))  /* dispc2_data13 */ \
	MV(CP(USBB2_ULPITLL_DAT6) , ( IEN | M5))  /* dispc2_data12 */ \
	MV(CP(USBB2_ULPITLL_DAT7) , ( IEN | M5))  /* dispc2_data11 */ \
	MV(CP(USBB2_HSIC_DATA) , ( PTD | OFF_EN | OFF_OUT_PTU | M3))  /* gpio_169 */ \
	MV(CP(USBB2_HSIC_STROBE) , ( PTD | OFF_EN | OFF_OUT_PTU | M3))  /* gpio_170 */ \
	MV(CP(UNIPRO_TX0) , ( PTD | M7))  /* unipro_tx0 */ \
	MV(CP(UNIPRO_TY0) , ( PTD | M7))  /* unipro_ty0 */ \
	MV(CP(UNIPRO_TX1) , ( PTD | M7))  /* unipro_tx1 */ \
	MV(CP(UNIPRO_TY1) , ( PTD | M7))  /* unipro_ty1 */ \
	MV(CP(UNIPRO_TX2) , ( PTD | M7))  /* unipro_tx2 */ \
	MV(CP(UNIPRO_TY2) , ( PTD | M7))  /* unipro_ty2 */ \
	MV(CP(UNIPRO_RX0) , ( PTD | M7))  /* unipro_rx0 */ \
	MV(CP(UNIPRO_RY0) , ( PTD | M7))  /* unipro_ry0 */ \
	MV(CP(UNIPRO_RX1) , ( PTD | M7))  /* unipro_rx1 */ \
	MV(CP(UNIPRO_RY1) , ( PTD | IEN | M3))  /* gpio_178 */ \
	MV(CP(UNIPRO_RX2) , ( PTU | IEN | M3))  /* gpio_2 */ \
	MV(CP(UNIPRO_RY2) , ( PTD | M7))  /* unipro_ry2 */ \
	MV(CP(USBA0_OTG_CE) , ( PTD | OFF_EN | OFF_PD | OFF_OUT_PTD | M0))  /* usba0_otg_ce */ \
	MV(CP(USBA0_OTG_DP) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* usba0_otg_dp */ \
	MV(CP(USBA0_OTG_DM) , ( IEN | OFF_EN | OFF_PD | OFF_IN | M0))  /* usba0_otg_dm */ \
	MV(CP(FREF_CLK1_OUT) , ( M0))  /* fref_clk1_out */ \
	MV(CP(FREF_CLK2_OUT) , ( M0))  /* fref_clk2_out */ \
	MV(CP(SYS_NIRQ1) , ( PTU | IEN | M0))  /* sys_nirq1 */ \
	MV(CP(SYS_NIRQ2) , (M7_SAFE))  /* sys_nirq2 */ \
	MV(CP(SYS_BOOT0) , ( PTU | IEN | M3))  /* gpio_184 */ \
	MV(CP(SYS_BOOT1) , ( PTD | M3))  /* gpio_185 */ \
	MV(CP(SYS_BOOT2) , ( PTD | IEN | M3))  /* gpio_186 */ \
	MV(CP(SYS_BOOT3) , ( PTD | M3))  /* gpio_187 */ \
	MV(CP(SYS_BOOT4) , ( PTD | M3))  /* gpio_188 */ \
	MV(CP(SYS_BOOT5) , ( PTD | M3))  /* gpio_189 */ \
	MV(CP(DPM_EMU0) , ( IEN | M0))  /* dpm_emu0 */ \
	MV(CP(DPM_EMU1) , ( IEN | M0))  /* dpm_emu1 */ \
	MV(CP(DPM_EMU2) , ( IEN | M0))  /* dpm_emu2 */ \
	MV(CP(DPM_EMU3) , ( IEN | M5))  /* dispc2_data10 */ \
	MV(CP(DPM_EMU4) , ( IEN | M5))  /* dispc2_data9 */ \
	MV(CP(DPM_EMU5) , ( IEN | M5))  /* dispc2_data16 */ \
	MV(CP(DPM_EMU6) , ( IEN | M5))  /* dispc2_data17 */ \
	MV(CP(DPM_EMU7) , ( IEN | M5))  /* dispc2_hsync */ \
	MV(CP(DPM_EMU8) , ( IEN | M5))  /* dispc2_pclk */ \
	MV(CP(DPM_EMU9) , ( IEN | M5))  /* dispc2_vsync */ \
	MV(CP(DPM_EMU10) , ( IEN | M5))  /* dispc2_de */ \
	MV(CP(DPM_EMU11) , ( IEN | M5))  /* dispc2_data8 */ \
	MV(CP(DPM_EMU12) , ( IEN | M5))  /* dispc2_data7 */ \
	MV(CP(DPM_EMU13) , ( IEN | M5))  /* dispc2_data6 */ \
	MV(CP(DPM_EMU14) , ( IEN | M5))  /* dispc2_data5 */ \
	MV(CP(DPM_EMU15) , ( IEN | M5))  /* dispc2_data4 */ \
	MV(CP(DPM_EMU16) , ( M3))  /* gpio_27 */ \
	MV(CP(DPM_EMU17) , ( IEN | M5))  /* dispc2_data2 */ \
	MV(CP(DPM_EMU18) , ( IEN | M5))  /* dispc2_data1 */ \
	MV(CP(DPM_EMU19) , ( IEN | M5))  /* dispc2_data0 */ \
	MV1(WK(PAD0_SIM_IO) , ( PTD | M7))  /* sim_io */ \
	MV1(WK(PAD1_SIM_CLK) , ( PTD | M7))  /* sim_clk */ \
	MV1(WK(PAD0_SIM_RESET) , ( PTD | M7))  /* sim_reset */ \
	MV1(WK(PAD1_SIM_CD) , ( PTU | IEN | M0))  /* sim_cd */ \
	MV1(WK(PAD0_SIM_PWRCTRL) , ( M0))  /* sim_pwrctrl */ \
	MV1(WK(PAD1_SR_SCL) , ( PTU | IEN | M0))  /* sr_scl */ \
	MV1(WK(PAD0_SR_SDA) , ( PTU | IEN | M0))  /* sr_sda */ \
	MV1(WK(PAD1_FREF_XTAL_IN) , ( M0))  /* # */ \
	MV1(WK(PAD0_FREF_SLICER_IN) , ( M0))  /* fref_slicer_in */ \
	MV1(WK(PAD1_FREF_CLK_IOREQ) , ( M0))  /* fref_clk_ioreq */ \
	MV1(WK(PAD0_FREF_CLK0_OUT) , ( M2))  /* sys_drm_msecure */ \
	MV1(WK(PAD1_FREF_CLK3_REQ) , ( PTD | M3))  /* gpio_wk30 */ \
	MV1(WK(PAD0_FREF_CLK3_OUT) , ( M0))  /* fref_clk3_out */ \
	MV1(WK(PAD1_FREF_CLK4_REQ) , ( PTD | M3))  /* gpio_wk7 */ \
	MV1(WK(PAD0_FREF_CLK4_OUT) , ( PTD | IEN | M3))  /* gpio_wk8 */ \
	MV1(WK(PAD1_SYS_32K) , ( IEN | M0))  /* sys_32k */ \
	MV1(WK(PAD0_SYS_NRESPWRON) , ( M0))  /* sys_nrespwron */ \
	MV1(WK(PAD1_SYS_NRESWARM) , ( M0))  /* sys_nreswarm */ \
	MV1(WK(PAD0_SYS_PWR_REQ) , ( PTU | M0))  /* sys_pwr_req */ \
	MV1(WK(PAD1_SYS_PWRON_RESET) , ( M3))  /* gpio_wk29 */ \
	MV1(WK(PAD0_SYS_BOOT6) , ( PTD | IEN | M3))  /* gpio_wk9 */ \
	MV1(WK(PAD1_SYS_BOOT7) , ( IEN | M3))  /* gpio_wk10 */

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
