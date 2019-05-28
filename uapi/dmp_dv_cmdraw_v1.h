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

	struct dmp_dv_kbuf u8tofp16_table; // If this table exist (fd >= 0), input buffer is in u8 format
                                     // and should be converted to fp16 with this table
  // The following is the same as v0, so should be able to reuse implementation in kernel
	struct dmp_dv_kbuf input_buf;      // Input buffer
	struct dmp_dv_kbuf output_buf;     // Output buffer
	struct dmp_dv_kbuf eltwise_buf;    // Buffer for elementwise add (0 = UBUF Input Buffer)

	__u32 topo;                   // [31:0] Output Destination of each run, 0 = UBUF, 1 = EXTMEM
	__u16 w;                      // Input Width
	__u16 h;                      // Input Height
	__u16 z;                      // Input Depth
	__u16 c;                      // Input Channels
	__u16 input_circular_offset;  // Input Depth circular offset
	__u16 output_mode;            // 0 = concat, 1 = elementwise add

	struct dmp_dv_kcmdraw_conv_v0_run run[32];  // Description of each run
};


#endif  // _UAPI_LINUX_DMP_DV_CMDRAW_V1_H
