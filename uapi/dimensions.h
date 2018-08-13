// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 *  DV700 kernel driver user interface
 *
 *  Copyright (C) 2018  Digital Media Professionals Inc.
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
 * @brief Helper functions for computing dimensions of weights/output.
 */
#pragma once

#ifndef _UAPI_LINUX_DMP_DV_DIMENSIONS_H
#define _UAPI_LINUX_DMP_DV_DIMENSIONS_H

#include <linux/types.h>

#include "dmp_dv_cmdraw_v0.h"


/// Input/output dimensions for the supported convolutional operations.
struct conv_data_size {
    int32_t w;      // Width
    int32_t h;      // Height
    int32_t z;      // Depth
    int32_t c;      // Channels
    uint32_t size;  // Total Size = w * h * z * c * sizeof(float16)
};


/// @brief Returns output width for convolution.
/// @param width Input width.
/// @param kx Kernel width.
/// @param pad_left Left padding.
/// @param pad_right Right padding.
/// @param stride Stride.
static inline int get_conv_out_width(int width, int kx, int pad_left, int pad_right, int stride) {
  return (pad_left + width + pad_right - kx) / stride + 1;
}


/// @brief Returns size in bytes for the packed weights for convolutional operation.
/// @param c Number of input channels.
/// @param m Number of convolutional kernels (number of output channels).
/// @param k Kernel size: max(kx, ky) | 1.
/// @param quantized Use quantized weights or not.
/// @param dw Use depth-wise convolution or not.
static uint32_t get_weight_size(int c, int m, int k, int quantized, int dw) {
  uint32_t res;
  if (dw) {
    c = 1;
  }
  if (k == 5) {
    c = (c >> 1) + (c & 1);
  }
  else if (k == 3) {
    c = (c >> 3) + ((c & 7) ? 1 : 0);
  }
  else if (k == 1) {
    c = (c >> 6) + ((c & 63) ? 1 : 0);
  }

  if (quantized) {
    res = 512 + 72 * m * c + 16 * ((m + 7) / 8);
  }
  else {
    res = 144 * m * c + 16 * ((m + 7) / 8);
  }
  return res;
}


/// @brief Assigns input size.
static inline void init_conv_input_size_v0_4(
    __u16 w, __u16 h, __u16 z, __u16 c,
    struct conv_data_size *in_size) {
  in_size->w = w;
  in_size->h = h;
  in_size->z = z;
  in_size->c = c;
  in_size->size = (uint32_t)w * h * z * c * 2;  // we are using 16-bit floats
}


/// @brief Assigns input size.
static inline void init_conv_input_size_v0(
    const dmp_dv_kcmdraw_conv_v0 *cmd,
    struct conv_data_size *in_size) {
  init_conv_input_size_v0_4(cmd->w, cmd->h, cmd->z, cmd->c, in_size);
}


/// @brief Fills output size and weights size for layer configuration version 0.
static void get_conv_output_size_v0(
    dmp_dv_kcmdraw_conv_v0_run *run,
    struct conv_data_size *in_size,
    struct conv_data_size *out_size,
    uint32_t *w_size) {

  int in_w = in_size->w;
  int in_h = in_size->h;
  int in_z = in_size->z;
  int in_c = in_size->c;
  int t0_w;
  int t0_h;
  int t0_z;
  int t0_c;

  // Convolution
  if (run->conv_enable) {
    int m = run->m;
    int p = run->p;
    int px = p & 0xFF;
    int py = (p >> 8) & 0xFF;
    int pz = run->pz & 0x7F;
    int conv_pad = run->conv_pad;
    int conv_stride = run->conv_stride;
    int pad_w0 = conv_pad & 0xff;
    int pad_w1 = (conv_pad >> 8) & 0xff;
    int pad_h0 = (conv_pad >> 16) & 0xff;
    int pad_h1 = (conv_pad >> 24) & 0xff;
    int stride_w = conv_stride & 0xff;
    int stride_h = (conv_stride >> 8) & 0xff;
    int core_w = pad_w0 + in_w + pad_w1;
    int core_h = pad_h0 + in_h + pad_h1;
    if (py == 0) {
      py = px;
    }
    t0_w = (core_w - px) / stride_w + 1;
    t0_h = (core_h - py) / stride_h + 1;
    // NOTE: No padding or stride in Z (depth) implemented yet!
    t0_z = (in_z - pz + 1);
    t0_c = m;  // Number of convolution output channels
    *w_size = get_weight_size(in_c, m, ((px > py) ? px : py) | 1, (run->weight_fmt & 2),
                              (run->conv_enable & 2));
  }
  else {  // Bypass of convolution
    t0_w = in_w;
    t0_h = in_h;
    t0_z = in_z;
    t0_c = in_c;
    *w_size = 0;
  }

  // Pooling
  if (((run->pool_enable) & 0x7) != 0) {
    int pool_size = run->pool_size;
    int pool_size_w = pool_size & 0xff;
    int pool_size_h = (pool_size >> 8) & 0xff;
    int pool_pad = run->pool_pad;
    int pool_pad_w0 = pool_pad & 0xff;
    int pool_pad_w1 = (pool_pad >> 8) & 0xff;
    int pool_pad_h0 = (pool_pad >> 16) & 0xff;
    int pool_pad_h1 = (pool_pad >> 24) & 0xf;
    int pool_stride = run->pool_stride;
    int pool_stride_w = pool_stride & 0xff;
    int pool_stride_h = (pool_stride >> 8) & 0xff;
    // Unpool with argmax, or upsample
    if ((run->pool_enable == 5) || (run->pool_enable == 4)) {
      // NOTE: only 2x2 size and 2x2 stride supported currently
      out_size->w = 2 * t0_w;
      out_size->h = 2 * t0_h;
    }
    else {
      out_size->w = ((pool_pad_w0 + t0_w + pool_pad_w1) -
                     pool_size_w) / pool_stride_w + 1;
      out_size->h = ((pool_pad_h0 + t0_h + pool_pad_h1) -
                     pool_size_h) / pool_stride_h + 1;
    }
    // NOTE: No pooling in Z (depth) implemented yet!
    out_size->z = t0_z;
    // Number of channels preserved in pooling...
    out_size->c = t0_c;
  }
  else {  // Bypass of max pooling
    out_size->w = t0_w;
    out_size->h = t0_h;
    out_size->z = t0_z;
    out_size->c = t0_c;
  }
  out_size->size = out_size->w * out_size->h * out_size->z * out_size->c;
  out_size->size *= 2;
}


#endif  // _UAPI_LINUX_DMP_DV_DIMENSIONS_H
