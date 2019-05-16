// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 *  DV700 kernel driver user interface
 *
 *  Copyright (C) 2019  Digital Media Professionals Inc.
 *
 * This software is dual licensed under the terms of Apache License, Version 2.0 OR
 * the GNU General Public License version 2, as published by the Free Software Foundation,
 * and may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
/*
 * @brief Kernel-space definitions for "Raw command for execution version 1".
 */
#pragma once

#ifndef _UAPI_LINUX_DMP_DV_CMDRAW_V1_H
#define _UAPI_LINUX_DMP_DV_CMDRAW_V1_H

#include "dmp_dv_cmdraw_v0.h"

/// @brief Raw command for convolutional block version 1.
struct dmp_dv_kcmdraw_conv_v1 {
	struct dmp_dv_kcmdraw header;  // General structure information 

	struct dmp_dv_kcmdraw_conv_v0 conv_cmd;

	struct dmp_dv_kbuf u8tofp16_table;
	__u8 is_u8_input;

	__u8 rsvd[7];
};


#endif  // _UAPI_LINUX_DMP_DV_CMDRAW_V1_H
