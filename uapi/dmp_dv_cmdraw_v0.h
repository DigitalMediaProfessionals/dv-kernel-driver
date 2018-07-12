/*
 *  DV700 kernel driver user interface
 *
 *  Copyright (C) 2018  Digital Media Professionals Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
/*
 * @brief Kernel-space definitions for "Raw command for execution version 0".
 */
#pragma once

#ifndef _UAPI_LINUX_DMP_DV_CMDRAW_V0_H
#define _UAPI_LINUX_DMP_DV_CMDRAW_V0_H

#include <linux/types.h>


#include "dmp-dv.h"


/// @brief Convolution layer runs.
/// @details Members within structure are rearranged by size to avoid requirements for 64-bits padding in the middle.
typedef struct dmp_dv_cmdraw_v0_conv_run_impl {
  dmp_dv_buf weight_buf;  // Buffer with packed weights
  __u32 conv_pad;         // Bits [7:0] = left padding, bits [15:8] = right padding, bits [23:16] = top padding, bits [31:24] = bottom padding
  __u32 pool_pad;         // Bits [7:0] = left padding, bits [15:8] = right padding, bits [23:16] = top padding, bits [31:24] = bottom padding
  __u16 m;                // Number of Output Channels
  __u16 conv_enable;      // 1 = Enabled, 0 = Disabled, 3 = LRN
  __u16 p;                // Filter Size (width = height)
  __u16 pz;               // Filter Depth (1 in case of 2D convolution)
  __u16 conv_stride;      // Bits [7:0] = X stride, bits [15:8] = Y stride
  __u16 conv_dilation;    // Bits [7:0] = X dilation, bits [15:8] = Y dilation
  __u16 weight_fmt;       // Weight format (0 = random access blocks, 1 = compact stream, 2 = 8-bit qunatized stream)
  __u16 pool_enable;      // 0 = disabled, 1 = max pooling, 2 = average pooling, 4 - upsampling
  __u16 pool_avg_param;   // Usually be set to 1/pool_size^2 in FP16 when using average pooling (average pooling assumes square size)
  __u16 pool_size;        // Bits [7:0] = width, bits [15:8] = height
  __u16 pool_stride;      // Bits [7:0] = X stride, bits [15:8] = Y stride
  __u16 actfunc;          // Activation Function: 0 = None, 1 = Tanh, 2 = Leaky ReLU, 3 = Sigmoid, 4 = PReLU, 5 = ELU, 6 = ReLU6
  __u16 actfunc_param;    // Leaky ReLU parameter in FP16
  __u16 rectifi_en;       // Rectification, i.e. max(0, x) (NOTE: Can be applied after non-ReLU activation function)
  __u16 lrn;              // Bits [0]: 1 = LRN enable, 0 = LRN disable, [1]: 1 = incl. power func, 0 = excl., [8:11]: x^2 scale factor log2
  __u16 rsvd;             // padding to 64-bit size
} dmp_dv_cmdraw_v0_conv_run;


/// @brief Raw command for execution version 0.
typedef struct dmp_dv_cmdraw_v0_impl {
  __u32 size;                         // size of this structure
  __u32 version;                      // version of this structure
  dmp_dv_buf input_buf;               // Input buffer
  dmp_dv_buf output_buf;              // Output buffer
  dmp_dv_buf eltwise_buf;             // Buffer for elementwise add (0 = UBUF Input Buffer)
  __u32 topo;                         // [31:0] Output Destination of each run, 0 = UBUF, 1 = EXTMEM
  __u16 w;                            // Input Width
  __u16 h;                            // Input Height
  __u16 z;                            // Input Depth
  __u16 c;                            // Input Channels
  __u16 input_circular_offset;        // Input Depth circular offset
  __u16 output_mode;                  // 0 = concat, 1 = elementwise add
  dmp_dv_cmdraw_v0_conv_run run[32];  // description of each run
} dmp_dv_cmdraw_v0;

#endif  // _UAPI_LINUX_DMP_DV_CMDRAW_V0_H
