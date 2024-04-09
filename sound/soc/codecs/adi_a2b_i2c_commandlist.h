/* SPDX-License-Identifier: GPL-2.0-or-later */

/*******************************************************************************
* A2B24xx I2C Commandlist
*
* Copyright 2019 Analog Devices Inc.
*
* ADI Automotive Software Team, Bangalore
*
* Licensed under GPL-2
*****************************************************************************/

/*! \addtogroup ADI_A2B_DISCOVERY_CONFIG ADI_A2B_DISCOVERY_CONFIG
* @{
*/
#ifndef _ADI_A2B_I2C_LIST_H_
#define _ADI_A2B_I2C_LIST_H_

/*! \struct ADI_A2B_DISCOVERY_CONFIG
A2B discovery config unit structure
*/
typedef struct {
/*!  Device address */
	unsigned char nDeviceAddr;

/*!  Operation code */
	unsigned char eOpCode;

/*! Reg Sub address width (in bytes) */
	unsigned char nAddrWidth;

/*! Reg Sub address */
	unsigned int nAddr;

/*! Reg data width (in bytes) */
	unsigned char nDataWidth;

/*! Reg data count (in bytes) */
	unsigned short nDataCount;

/*! Config Data */
	unsigned char *paConfigData;

} ADI_A2B_DISCOVERY_CONFIG;

#define WRITE_VALUE   ((unsigned char) 0x00u)
#define READ_VALUE    ((unsigned char) 0x01u)
#define DELAY   ((unsigned char) 0x02u)
#define INVALID ((unsigned char) 0xffu)

#define CONFIG_LEN (0)

ADI_A2B_DISCOVERY_CONFIG gaA2BConfig[CONFIG_LEN] = {
	NULL
};

#endif /* _ADI_A2B_I2C_LIST_H_ */
