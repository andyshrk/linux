// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Author: Siengine Technology, Inc.
 */

#ifndef __SIENGINE_CIF_MISC_REGS_H__
#define __SIENGINE_CIF_MISC_REGS_H__

/*
 * bit[0] : 0-dphy0 / dphy1 work as 2lane mode, 1 dphy0 + dphy1 work 4lane mode.
 * bit[1] : 0-dphy2 / dphy3 work as 2lane mode, 1 dphy2 + dphy3 work 4lane mode.
 * bit[2] : 0-dphy4 / dphy5 work as 2lane mode, 1 dphy4 + dphy5 work 4lane mode.
 * bit[3] : 0-dphy6 / dphy6 work as 2lane mode, 1 dphy6 + dphy6 work 4lane mode.
 */
#define CSI_WORK_MODE						0x0

/* bit[0:31] : round[(Fcfg_clk - 17) * 4] */
#define CSI_DPHY_CFGCLKREQRANGE				0x4

/* config csi0 hs freq */
#define CSI0_DPHY_HSFREQRANGE				0x8

/* config csi1 hs freq */
#define CSI1_DPHY_HSFREQRANGE				0xc

/* config csi2 hs freq */
#define CSI2_DPHY_HSFREQRANGE				0x10

/* config csi3 hs freq */
#define CSI3_DPHY_HSFREQRANGE				0x14

/* config csi4 hs freq */
#define CSI4_DPHY_HSFREQRANGE				0x18

/* config csi5 hs freq */
#define CSI5_DPHY_HSFREQRANGE				0x1c

/* config csi6 hs freq */
#define CSI6_DPHY_HSFREQRANGE				0x20

/* config csi7 hs freq */
#define CSI7_DPHY_HSFREQRANGE				0x24

/*
 * bit[0]     : csi0 IO continuity control
 * bit[1:11]  : csi0 IO continuity test data
 * bit[12]    : csi1 IO continuity control
 * bit[13:23] : csi1 IO continuity test data
 */
#define CSI0_SS_CON_CHECK					0x28

/*
 * bit[0]     : csi2 IO continuity control
 * bit[1:11]  : csi2 IO continuity test data
 * bit[12]    : csi3 IO continuity control
 * bit[13:23] : csi3 IO continuity test data
 */
#define CSI1_SS_CON_CHECK					0x2c

/*
 * bit[0]     : csi4 IO continuity control
 * bit[1:11]  : csi4 IO continuity test data
 * bit[12]    : csi5 IO continuity control
 * bit[13:23] : csi5 IO continuity test data
 */
#define CSI2_SS_CON_CHECK					0x30

/*
 * bit[0]     : csi6 IO continuity control
 * bit[1:11]  : csi6 IO continuity test data
 * bit[12]    : csi7 IO continuity control
 * bit[13:23] : csi7 IO continuity test data
 */
#define CSI3_SS_CON_CHECK					0x34

/*
 * bit[0] : csi0 dphy 0-close, 1-enable clock
 * bit[1] : csi1 dphy 0-close, 1-enable clock
 * bit[2] : csi2 dphy 0-close, 1-enable clock
 * bit[3] : csi3 dphy 0-close, 1-enable clock
 * bit[4] : csi4 dphy 0-close, 1-enable clock
 * bit[5] : csi5 dphy 0-close, 1-enable clock
 * bit[6] : csi6 dphy 0-close, 1-enable clock
 * bit[7] : csi7 dphy 0-close, 1-enable clock
 */
#define CSI_DPHY_CLOCK_EN					0x38

/*
 * bit[0]  : csi0 dphy start the bist test
 * bit[1]  : csi0 dphy bist test done
 * bit[2]  : csi0 dphy bist test ok
 * bit[3]  : csi1 dphy start the bist test
 * bit[4]  : csi1 dphy bist test done
 * bit[5]  : csi1 dphy bist test ok
 * bit[6]  : csi2 dphy start the bist test
 * bit[7]  : csi2 dphy bist test done
 * bit[8]  : csi2 dphy bist test ok
 * bit[9]  : csi3 dphy start the bist test
 * bit[10] : csi3 dphy bist test done
 * bit[11] : csi3 dphy bist test ok
 * bit[12] : csi4 dphy start the bist test
 * bit[13] : csi4 dphy bist test done
 * bit[14] : csi4 dphy bist test ok
 * bit[15] : csi5 dphy start the bist test
 * bit[16] : csi5 dphy bist test done
 * bit[17] : csi5 dphy bist test ok
 * bit[18] : csi6 dphy start the bist test
 * bit[19] : csi6 dphy bist test done
 * bit[20] : csi6 dphy bist test ok
 * bit[21] : csi7 dphy start the bist test
 * bit[22] : csi7 dphy bist test done
 * bit[23] : csi7 dphy bist test ok
 */
#define CSI_DPHY_BIST						0x3c

/*
 * bit[0]  : csi0 force lane0 to rx stop state
 * bit[1]  : csi0 force lane1 to rx stop state
 * bit[2]  : csi1 force lane0 to rx stop state
 * bit[3]  : csi1 force lane1 to rx stop state
 * bit[4]  : csi2 force lane0 to rx stop state
 * bit[5]  : csi2 force lane1 to rx stop state
 * bit[6]  : csi3 force lane0 to rx stop state
 * bit[7]  : csi3 force lane1 to rx stop state
 * bit[8]  : csi4 force lane0 to rx stop state
 * bit[9]  : csi4 force lane1 to rx stop state
 * bit[10] : csi5 force lane0 to rx stop state
 * bit[11] : csi5 force lane1 to rx stop state
 * bit[12] : csi6 force lane0 to rx stop state
 * bit[13] : csi6 force lane1 to rx stop state
 * bit[14] : csi7 force lane0 to rx stop state
 * bit[15] : csi7 force lane1 to rx stop state
 */
#define CSI_DPHY_FORCERXMODE				0x40

/*
 * bit[0:10]  : csi0 lane0 state
 * bit[11:21] : csi0 lane1 state
 * bit[22:23] : csi0 clk lane state
 */
#define CSI0_DPHY_STATE						0x44

/*
 * bit[0:10]  : csi1 lane0 state
 * bit[11:21] : csi1 lane1 state
 * bit[22:23] : csi1 clk lane state
 */
#define CSI1_DPHY_STATE						0x48

/*
 * bit[0:10]  : csi2 lane0 state
 * bit[11:21] : csi2 lane1 state
 * bit[22:23] : csi2 clk lane state
 */
#define CSI2_DPHY_STATE						0x4c

/*
 * bit[0:10]  : csi3 lane0 state
 * bit[11:21] : csi3 lane1 state
 * bit[22:23] : csi3 clk lane state
 */
#define CSI3_DPHY_STATE						0x50

/*
 * bit[0:10]  : csi4 lane0 state
 * bit[11:21] : csi4 lane1 state
 * bit[22:23] : csi4 clk lane state
 */
#define CSI4_DPHY_STATE						0x54

/*
 * bit[0:10]  : csi5 lane0 state
 * bit[11:21] : csi5 lane1 state
 * bit[22:23] : csi5 clk lane state
 */
#define CSI5_DPHY_STATE						0x58

/*
 * bit[0:10]  : csi6 lane0 state
 * bit[11:21] : csi6 lane1 state
 * bit[22:23] : csi6 clk lane state
 */
#define CSI6_DPHY_STATE						0x5c

/*
 * bit[0:10]  : csi7 lane0 state
 * bit[11:21] : csi7 lane1 state
 * bit[22:23] : csi7 clk lane state
 */
#define CSI7_DPHY_STATE						0x60

/*
 * bit[0]  : 0-software control clear force rx mode
 *           1-hardware control clear force rx mode,
 *             if lane stop, auto clear force rx mode
 */
#define CSI_DPHY_FORCERXMODE_CONTROL		0x64

/*
 * bit[0] : csi0 dphy basedir
 * bit[1] : csi1 dphy basedir
 * bit[2] : csi2 dphy basedir
 * bit[3] : csi3 dphy basedir
 * bit[4] : csi4 dphy basedir
 * bit[5] : csi5 dphy basedir
 * bit[6] : csi6 dphy basedir
 * bit[7] : csi7 dphy basedir
 */
#define CSI_BASEDIR							0x68

/* bit[0:7]  : dphy test stop clk en control */
#define CSI_PHY_TEST_STOP_CLK_EN			0x6c

#endif /* __SIENGINE_CIF_MISC_REGS_H__ */
