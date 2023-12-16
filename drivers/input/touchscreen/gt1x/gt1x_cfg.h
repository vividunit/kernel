/* SPDX-License-Identifier: GPL-2.0 */
/*
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef _GOODIX_GT1X_CFG_H_
#define _GOODIX_GT1X_CFG_H_

/* CFG for GT5688 */
static u8 gtp_dat_5688[] = {
	/* <1080, 1920>*/
	#include "GT5688_Config_20170713_1080_1920.cfg"
};

/* CFG for GT1151Q */
static u8 gtp_dat_gt1151q[] = {
	/* <720, 1280>*/
	#include "A26_Z0399A0_720X1280_V8_Config_20210630.cfg"
};

#endif /* _GOODIX_GT1X_CFG_H_ */
