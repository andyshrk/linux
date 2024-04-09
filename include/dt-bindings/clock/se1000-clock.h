/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2020 siengine Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DTS_SE1000_CLOCK_H
#define __DTS_SE1000_CLOCK_H

/* no enum to make dts happy */
#define FIXED_1600M		0
#define FIXED_800M		1
#define FIXED_400M		2
#define FIXED_200M		3
#define FIXED_100M		4
#define FIXED_1200M		5
#define FIXED_600M		6
#define FIXED_300M		7
#define FIXED_500M		8
#define FIXED_333M		9
#define FIXED_250M		10
#define FIXED_900M		11
#define FIXED_702M		12
#define FIXED_1188M		13
#define FIXED_442P368M		14
#define FIXED_PIXEL		15
#define OSC_25M_CLK		(16)
#define OSC_19P2M_CLK		(17)
#define OSC_1M_CLK		(18)

/* se1000 system clock and reset controller id*/
#define APSS0_PLL0_OUT		(21)
#define AP0_CLK_CORE5		(22)
#define AP0_CLK_CORE4		(23)
#define AP0_CLK_CORE3		(24)
#define AP0_CLK_CORE2		(25)

#define APSS0_PLL1_OUT		(26)
#define AP0_CLK_CORE1		(27)
#define AP0_CLK_CORE0		(28)

#define APSS0_PLL2_OUT		(29)
#define AP0_CLK_SCLK		(30)
#define AP0_CLK_PCLK		(31)
#define AP0_CLK_ATCLK		(32)
#define AP0_CLK_GIC		(33)
#define AP0_CLK_PERIPH		(34)
#define AP0_CLK_PDBG		(35)

#define APSS1_PLL0_OUT		(36)
#define AP1_CLK_CORE1		(37)
#define AP1_CLK_CORE0		(38)

#define APSS1_PLL1_OUT		(39)
#define AP1_CLK_SCLK		(40)
#define AP1_CLK_PCLK		(41)
#define AP1_CLK_ATCLK		(42)
#define AP1_CLK_GIC		(43)
#define AP1_CLK_PERIPH		(44)
#define AP1_CLK_PDBG		(45)

#define DDR_PLL_OUT			(46)
#define DDR_PLL_OUT_FOUT1PH0		(47)
#define DDR_PLL_OUT_FOUT2		(48)
#define DDR_PLL_OUT_FOUT3		(49)
#define DDR_PLL_OUT_FOUT4		(50)
#define DDR0_DDR_CFG_CLK		(51)
#define DDR0_DDR_LOW_CLK		(52)
#define DDR1_DDR_CFG_CLK		(53)
#define DDR1_DDR_LOW_CLK		(54)
#define DDRNOC_MAIN_CLK			(55)
#define GPUNOC_MAIN_CLK			(56)
#define DDR_LINK_CLK			(57)

#define GPU0_PLL_OUT			(58)
#define GPU0_CLK			(59)

#define GPU1_PLL_OUT			(60)
#define GPU1_CLK			(61)

#define AUDIO_PLL_OUT			(62)
#define AUDIO_PLL_OUT_FOUT1PH0		(63)
#define AUDIO_PLL_OUT_FOUT2		(64)
#define AUDIO_PLL_OUT_FOUT3		(65)
#define AUDIO_PLL_OUT_FOUT4		(66)
#define AUD_PLL_CLK			(67)

#define PCIE_PLL_OUT			(68)
#define PCIE_PHY_REF_ALT_CLK		(69)

#define USB_PLL_OUT			(70)
#define USB3_UTMI_REF_CLK		(71)
#define USB2_PHY_CLK			(72)
#define USB2_PHYREF_CLK		USB2_PHY_CLK

#define LH_PLL0_OUT			(73)
#define LH_PLL0_OUT_FOUT1PH0		(74)
#define LH_PLL0_OUT_FOUT2		(75)
#define LH_PLL0_OUT_FOUT3		(76)
#define LH_PLL0_OUT_FOUT4		(77)

#define LH_PLL1_OUT			(78)
#define LH_PLL1_OUT_FOUT1PH0		(79)
#define LH_PLL1_OUT_FOUT2		(80)
#define LH_PLL1_OUT_FOUT3		(81)
#define LH_PLL1_OUT_FOUT4		(82)

#define LH_PLL2_OUT			(83)
#define LH_PLL2_OUT_FOUT1PH0		(84)
#define LH_PLL2_OUT_FOUT2		(85)
#define LH_PLL2_OUT_FOUT3		(86)
#define LH_PLL2_OUT_FOUT4		(87)

#define D71_AXI0_CLK			(88)
#define MDP_DPU_AXI0_CLK		D71_AXI0_CLK
#define D71_AXI1_CLK			(89)
#define MDP_DPU_AXI1_CLK		D71_AXI1_CLK
#define D71_AXI2_CLK			(90)
#define MDP_DPU_AXI2_CLK		D71_AXI2_CLK
#define MDP_APB_CLK			(91)
#define HDCP_AXI_CLK			(92)
#define DPHY_REF_CLK			(93)
#define MDP_DPHY_REL_CLK		DPHY_REF_CLK
#define DPHY_CFG_CLK			(94)
#define MDP_DPHY_CFG_CLK		DPHY_CFG_CLK
#define DPTXPHY_MBIST_CLK0		(95)
#define DPHY_MBIST_CLK			(96)
#define VPU_DEC_AXI_CLK			(97)
#define VPU_DEC_CORE_CLK		(98)
#define VPU_DEC_APB_CLK			(99)
#define VPU_ENC_AXI_CLK			(100)
#define VPU_ENC_CORE_CLK		(101)
#define VPU_ENC_APB_CLK			(102)
#define USB2_AXI_CLK			(103)
#define USB3_BUS_CLK			(104)
#define USB3_AXI_CLK			USB3_BUS_CLK
#define USB3_APB_CLK			(105)
#define USB3_CTR_REF_CLK		(106)
#define USB3_PIPEREF_CLK		USB3_CTR_REF_CLK
#define USB3_CTRL_MBIST_CLK		(107)
#define USB3_UTMIREF_CLK		USB3_CTRL_MBIST_CLK
#define SMP_CM4_CLK			(108)
#define SMP_CLK				(109)
#define SMP_SC_CLK			(110)
#define TOP_MODULE_CLK			(111)
#define CISP_AXI_CLK			(112)
#define CISP_RESET		CISP_AXI_CLK
#define CISP_APB_CLK			(113)
#define CISP_AHB_CLK			(114)
#define CISP_PIXEL_CLK			(115)
#define CISP_DPHY_CFG_CLK		(116)
#define CISP_DPHY_REF_CLK		(117)
#define PCIE_CTR_AUX_CLK		(118)
#define PCIE_CTR_AXI_ACLK		(119)
#define PCIE_CTR_AHB_HCLK		(120)
#define PCIE_PHY_APB_PCLK		(121)
#define NPU1_TT_CLK			(122)
#define NPU1_DBG_CLK			(123)
#define AUD_DSP_CLK			(124)
#define AUD_CLK				(125)
#define MAINCLK				(126)
#define IPSCLK				(127)
#define MSHCCLK				(128)
#define SDHCI_CTRL_CLK			 MSHCCLK
#define MSHC_TX_CLK			(129)
#define SDHCI_TX_CLK			MSHC_TX_CLK
#define UFSHCCLK			(130)
#define SFC_AHB_SLV_CLK			(131)
#define SFC_SMMU_CLK			(132)
#define SFC_UFS_TX_MBIST_CLK		(133)
#define SFC_UFS_RX_MBIST_CLK		(134)
#define PERI1_AXI_CLK			(135)
#define PERI1_AHB_CLK			(136)
#define PERI1_APB_CLK			(137)
#define PERI1_TIM_CLK			(138)
#define PERI1_REF_CLK			(139)
#define PERI0_AHB_CLK			(140)
#define PERI0_APB_CLK			(141)
#define PERI2_AHB_CLK			(142)
#define PERI2_APB_CLK			(143)
#define CLK_CS				(144)
#define TRACECLK			(145)
#define GPCLK0				(146)
#define GPCLK1				(147)
#define GPCLK3				(148)
#define GPCLK4				(149)
#define GPCLK5				(150)
#define GPCLK6				(151)
#define GPCLK7				(152)
#define SYSNOC_MAIN_CLK			(153)
#define AP0NOC_SERVICE_CLK		(154)
#define AP1NOC_SERVICE_CLK		(155)

#define GPU2_PLL_OUT			(156)
#define GPU2_PLL_OUT_FOUT1PH0		(157)
#define GPU2_PLL_OUT_FOUT2		(158)
#define GPU2_PLL_OUT_FOUT3		(159)
#define GPU2_PLL_OUT_FOUT4		(160)
#define GPU2_R2D_CLK			(161)
#define GPU2_R2D_PCLK			(162)
#define GPU2_V2D_CLK			(163)
#define DP_AUX16MHZ_CLK			(164)
#define MDP_DP_AUX16MHZ_CLK	DP_AUX16MHZ_CLK
#define CISP_ISP_CORE_CLK		(165)
#define CISP_GDC_CORE_CLK		(166)
#define NPU1_SS_CLK			(167)

#define DP_PLL_OUT			(168)
#define DP_PLL_OUT_FOUT1PH0		(169)
#define DP_PLL_OUT_FOUT2		(170)
#define DP_PLL_OUT_FOUT3		(171)
#define DP_PLL_OUT_FOUT4		(172)
#define PIXEL1_CLK			(173)

#define NPU_PLL_OUT			(174)
#define NPU_PLL_OUT_FOUT1PH0		(175)
#define NPU_PLL_OUT_FOUT2		(176)
#define NPU_PLL_OUT_FOUT3		(177)
#define NPU_PLL_OUT_FOUT4		(178)
#define NPU0_CLK			(179)
#define NPU1_PLL_CLK			(180)

/*saf_ss pll module*/
#define SAF_PLL0_OUT			(181)
#define SAF_PLL1_OUT			(182)
#define SAF_PLL2_OUT			(183)
#define SAF_PLL3_OUT			(184)
#define SAF_PLL3_OUT_FOUT1PH0		(185)
#define SAF_PLL3_OUT_FOUT2		(186)
#define SAF_PLL3_OUT_FOUT3		(187)
#define SAF_PLL3_OUT_FOUT4		(188)
#define SAF_DEVSEL_OUT			(189)

#define SAF_ETH1_REF_CLK		(190)
#define SAF_ETH0_REF_CLK		(191)
#define SAF_QSPI_REF_CLK		(192)
#define SAF_SBISTC_CLK			(193)
#define SAF_RAM_CLK			(194)
#define SAF_MMU0_CLK			(195)
#define SAF_ETH1_CLK			(196)
#define SAF_ETH0_CLK			(197)
#define SAF_QSPI_CLK			(198)
#define SAF_TIMERS_CLK			(199)
#define SAF_I2C5_CLK			(200)
#define SAF_I2C4_CLK			(201)
#define SAF_I2C3_CLK			(202)
#define SAF_I2C2_CLK			(203)
#define SAF_I2C1_CLK			(204)
#define SAF_I2C0_CLK			(205)
#define SAF_SSPI0_CLK			(206)
#define SAF_MSPI1_CLK			(207)
#define SAF_MSPI0_CLK			(208)
#define SAF_UART2_CLK			(209)
#define SAF_UART1_CLK			(210)
#define SAF_UART0_CLK			(211)
#define SAF_GPIO_CLK			(212)
#define SAF_SADP_PIX_CLK		(213)
#define SAF_SADP_AXI_CLK		(214)
#define SAF_SADP_APB_CLK		(215)
#define SAF_SADP_DPHY_CFG_CLK		(216)
#define SAF_SADP_DPHY_REF_CLK		(217)
#define SAF_BUS_CLK					(218)
/*pll module clkc reset id*/
#define PCIE_PHY_RESETN				(221)
#define PCIE_CTR0_RESET_N			(222)
#define PCIE_CTR1_RESET_N			(223)
#define PCIE_CTR2_RESET_N			(224)
#define PCIE_CTR3_RESET_N			(225)
#define PCIE_CTR0_POWER_UP_RST_N		(226)
#define PCIE_CTR1_POWER_UP_RST_N		(227)
#define PCIE_CTR2_POWER_UP_RST_N		(228)
#define PCIE_CTR3_POWER_UP_RST_N		(229)
#define PCIE_CTR0_BUTTON_RST_N			(230)
#define PCIE_CTR1_BUTTON_RST_N			(231)
#define PCIE_CTR2_BUTTON_RST_N			(232)
#define PCIE_CTR3_BUTTON_RST_N			(233)
#define USB3_POR_RESET				(234)

/*mdp and aud module clk*/
#define MDP_DPU0_PIXEL_CLK			(235)
#define MDP_DPU1_PIPE0_PIXEL_CLK		(236)
#define MDP_DPU1_PIPE1_PIXEL_CLK		(237)
#define MDP_DPU2_PIPE0_PIXEL_CLK		(238)
#define MDP_DPU2_PIPE1_PIXEL_CLK		(239)
#define AUD_I2S0_CLK				(240)
#define AUD_I2S1_CLK				(241)
#define AUD_I2S2_CLK				(242)
#define AUD_I2S3_CLK				(243)
#define AUD_I2S4_CLK				(244)
#define AUD_I2S5_CLK				(245)
#define AUD_I2S6_CLK				(246)
#define AUD_SPDIF_CLK				(247)
#define MDP_DPU0_PIXEL_CLK_VAR			(248)
#define MDP_DPU1_PIPE0_PIXEL_CLK_VAR		(249)
#define MDP_DPU1_PIPE1_PIXEL_CLK_VAR		(250)
#define MDP_DPU2_PIPE0_PIXEL_CLK_VAR		(251)
#define MDP_DPU2_PIPE1_PIXEL_CLK_VAR		(252)

#define GPCLK2				(253)
/* fix no Control register address clock id*/
#define DDR_CORE_CLK			(254)
#define DDR_PLLBY_CLK			(255)


#define STEP_AP0_CLK_CORE5		(278)
#define STEP_AP0_CLK_CORE4		(279)
#define STEP_AP0_CLK_CORE3		(280)
#define STEP_AP0_CLK_CORE2		(281)
#define STEP_AP0_CLK_CORE1		(282)
#define STEP_AP0_CLK_CORE0		(283)
#define STEP_AP1_CLK_CORE1		(284)
#define STEP_AP1_CLK_CORE0		(285)
#define STEP_GPU0_CLK			(286)
#define STEP_GPU1_CLK			(287)
#define STEP_GPU2_R2D_CLK		(288)
#define STEP_GPU2_V2D_CLK		(289)
#define STEP_CISP_ISP_CORE_CLK	(290)
#define STEP_NPU1_SS_CLK		(291)
#define STEP_NPU0_CLK			(292)
#define STEP_NPU1_PLL_CLK		(293)
#define STEP_VPU_DEC_CORE_CLK	(294)
#define STEP_NPU1_TT_CLK		(295)
#define STEP_AUD_DSP_CLK		(296)
#define STEP_SMP_CM4_CLK		(297)
#define STEP_AP0_CLK_SCLK		(298)
#define STEP_AP1_CLK_SCLK		(299)
#define MAX_CLK_DEV				(300)
#endif	/* __DTS_SE1000_CLOCK_H */
