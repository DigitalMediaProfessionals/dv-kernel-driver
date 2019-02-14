// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 *	DV700 kernel driver user interface
 *
 *	Copyright (C) 2018	Digital Media Professionals Inc.
 *
 * This software is dual licensed under the terms of Apache License, Version 2.0 OR
 * the GNU General Public License version 2, as published by the Free Software Foundation,
 * and may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
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


#define CONV_C_SUB 8
#define CONV_M_SUB 8
#define CONV_C_SUB_LOG2 3
#define CONV_M_SUB_LOG2 3


/// @brief Returns width of the grid of nodes.
static inline int conf_node_w(const struct dmp_dv_kcmdraw_conv_v0 *conf)
{
	uint32_t i = conf->topo;
	int n = 0;
	while (i) {
		if (i & 1) n++;
		i >>= 1;
	}
	return n;
}


/// @brief Returns number of "run" structures required by provided topology.
static inline int topo_num_runs(uint32_t topo)
{
	uint32_t n = 0;
	for (; topo; topo >>= 1, n++)
		;
	return n;
}


/// @brief Returns the peak utilization of unified buffer in bytes in case when tiles = 1.
static int ubuf_get_single_tile_usage(const struct dmp_dv_kcmdraw_conv_v0 *conf, int ubuf_size)
{
	int max_ubuf_addr, runs, node_w, m_tot, t, i, run, run_w,
	    is_input_node, is_output_node, max_ubuf_addr_run;
	uint32_t topo, prev_ubuf_ofm_blk_size_16, prev_ubuf_ofm_blk_size_128,
		 prev_ubuf_ofm_size_128, prev_ubuf_ofm_offset,
		 prev_ubuf_output_size, conf_conv_pad, conf_conv_stride,
		 conf_pool_pad, tile_size_in_xmem, tile_size_in,
		 tile_size_out_w, tile_size_out_h, tile_size_out,
		 tile_size_out_pool_w, tile_size_out_pool_h, tile_size_out_pool,
		 ubuf_ifm_offset, ubuf_ifm_blk_size_16, ubuf_ifm_blk_size_128,
		 ubuf_ifm_size_128, ubuf_input_size, ubuf_ofm_offset,
		 ubuf_ofm_blk_size_16, ubuf_ofm_blk_size_128, ubuf_ofm_size_128,
		 ubuf_output_size;
	uint16_t conf_input_w, conf_input_h, conf_input_c, prev_w, prev_h,
		 prev_c, conf_w, conf_h, conf_c, conf_m, conf_conv_enable,
		 conf_p, conf_pool_enable, conf_pool_size, conf_pool_stride,
		 upsample_enable, pool_max_enable, pool_avg_enable, pad_w0,
		 pad_w1, pad_h0, pad_h1, conf_px, conf_py, stride_w, stride_h,
		 pool_pad_w0, pool_pad_w1, pool_pad_h0, pool_pad_h1,
		 pool_size_w, pool_size_h, pool_stride_w, pool_stride_h,
		 fflzp_en, fftzp_en, C_blocks, M_blocks,
		 xmem_ofm_blk_w_16, xmem_ofm_blk_h_16,
		 core_w, core_h;
	//uint32_t xmem_ofm_blk_size_16;
	//uint16_t C_size_last, M_size_last;

	max_ubuf_addr = 0;

	topo = conf->topo;
	if (topo == 0)
		return max_ubuf_addr;

	conf_input_w = conf->w;
	conf_input_h = conf->h;
	conf_input_c = conf->c;

	runs = topo_num_runs(conf->topo);
	node_w = conf_node_w(conf);

	m_tot = 0;
	t = topo;
	for (i = 0; i < runs; i++) {
		if (t & 1)
			m_tot += conf->run[i].m;
		t >>= 1;
	}

	run_w = 0;
	is_input_node = 1;

	for (run = 0; run < runs; run++) {

		is_output_node = topo & 1;

		if (is_input_node) {
			conf_w = conf_input_w;
			conf_h = conf_input_h;
			conf_c = conf_input_c;
		} else {
			conf_w = prev_w;
			conf_h = prev_h;
			conf_c = prev_c;
		}

		conf_m = conf->run[run].m;
		conf_conv_enable = conf->run[run].conv_enable;
		conf_p = conf->run[run].p;
		conf_conv_pad = conf->run[run].conv_pad;
		conf_conv_stride = conf->run[run].conv_stride;
		conf_pool_enable = conf->run[run].pool_enable;
		conf_pool_size = conf->run[run].pool_size;
		conf_pool_stride = conf->run[run].pool_stride;
		conf_pool_pad = conf->run[run].pool_pad;

		conf_conv_enable = conf_conv_enable & 0x1;

		upsample_enable = (conf_pool_enable == 4); // 2x2 upsample
		pool_max_enable = (conf_pool_enable & 0x1) | upsample_enable;
		pool_avg_enable = (conf_pool_enable == 2);

		if (pool_avg_enable) {
			conf_conv_pad = conf_pool_pad;
			conf_pool_pad = 0;
			conf_conv_stride = conf_pool_stride;
			conf_pool_stride = 0x0101;
			conf_p = conf_pool_size & 0xFF;
			conf_pool_size = 0;
			conf_conv_enable = 1;
		}

		pad_w0 = conf_conv_pad & 0x7F;
		pad_w1 = (conf_conv_pad >> 8) & 0xFF;
		pad_h0 = (conf_conv_pad >> 16) & 0x7F;
		pad_h1 = (conf_conv_pad >> 24) & 0xFF;

		conf_px = conf_p & 0xFF;
		conf_py = (conf_p >> 8) & 0xFF;
		if (conf_py == 0) conf_py = conf_px;
		conf_p = (conf_px > conf_py ? conf_px : conf_py) | 1;
		pad_w1 += (conf_p - conf_px);
		pad_h0 += (conf_p - conf_py);

		stride_w = conf_conv_stride & 0xFF;
		stride_h = (conf_conv_stride >> 8) & 0xFF;

		pool_pad_w0 = conf_pool_pad & 0xFF;
		pool_pad_w1 = (conf_pool_pad >> 8) & 0xFF;
		pool_pad_h0 = (conf_pool_pad >> 16) & 0xFF;
		pool_pad_h1 = (conf_pool_pad >> 24) & 0xFF;

		pool_size_w = conf_pool_size & 0xFF;
		pool_size_h = (conf_pool_size >> 8) & 0xFF;

		pool_stride_w = conf_pool_stride & 0xFF;
		pool_stride_h = (conf_pool_stride >> 8) & 0xFF;

		if (!pool_avg_enable) {
			if (!conf_conv_enable) {
				pad_w0 = 0;
				pad_w1 = 0;
				pad_h0 = 0;
				pad_h1 = 0;
				stride_w = 1;
				stride_h = 1;
			}
			if (pool_max_enable) {
				pad_w0 += stride_w * pool_pad_w0;
				pad_w1 += stride_w * pool_pad_w1;
				pad_h0 += stride_h * pool_pad_h0;
				pad_h1 += stride_h * pool_pad_h1;
			}
		}

		fflzp_en = (conf_px == conf_py) & (stride_w == 1) & conf_conv_enable & (pad_w0 != 0);
		fftzp_en = (conf_px == conf_py) & (stride_h == 1) & conf_conv_enable & (pad_h0 >= (conf_p >> 1));

		if ((conf_c & ((1 << CONV_C_SUB_LOG2) - 1)) == 0) {
			C_blocks = (conf_c >> CONV_C_SUB_LOG2);
			//C_size_last = CONV_C_SUB;
		} else {
			C_blocks = (conf_c >> CONV_C_SUB_LOG2) + 1;
			//C_size_last = conf_c & ((1 << CONV_C_SUB_LOG2) - 1);
		}

		if ((conf_m & ((1 << CONV_M_SUB_LOG2) - 1)) == 0) {
			M_blocks = (conf_m >> CONV_M_SUB_LOG2);
			//M_size_last = CONV_M_SUB;
		} else {
			M_blocks = (conf_m >> CONV_M_SUB_LOG2) + 1;
			//M_size_last = conf_m & ((1 << CONV_M_SUB_LOG2) - 1);
		}

		if (conf_conv_enable) {
			xmem_ofm_blk_w_16 = (pad_w0 + conf_w + pad_w1 - conf_p) / stride_w + 1;
			xmem_ofm_blk_h_16 = (pad_h0 + conf_h + pad_h1 - conf_p) / stride_h + 1;
		} else {
			xmem_ofm_blk_w_16 = pad_w0 + conf_w + pad_w1;
			xmem_ofm_blk_h_16 = pad_h0 + conf_h + pad_h1;
		}
		if (pool_max_enable) {
			if (upsample_enable) {
				xmem_ofm_blk_w_16 = 2 * xmem_ofm_blk_w_16;
				xmem_ofm_blk_h_16 = 2 * xmem_ofm_blk_h_16;
			} else {
				xmem_ofm_blk_w_16 = (xmem_ofm_blk_w_16 - pool_size_w) / pool_stride_w + 1;
				xmem_ofm_blk_h_16 = (xmem_ofm_blk_h_16 - pool_size_h) / pool_stride_h + 1;
			}
		}
		//xmem_ofm_blk_size_16 = (conf_m > CONV_M_SUB ? CONV_M_SUB : conf_m) * xmem_ofm_blk_w_16 * xmem_ofm_blk_h_16;

		if (!is_output_node) {
			prev_w = xmem_ofm_blk_w_16;
			prev_h = xmem_ofm_blk_h_16;
			prev_c = conf_m;
		}

		core_w = pad_w0 + conf_w + pad_w1;
		core_h = pad_h0 + conf_h + pad_h1;

		tile_size_in_xmem = conf_h * conf_w;
		tile_size_in = (core_h - (fftzp_en ? conf_p >> 1 : 0)) * (core_w - (fflzp_en ? conf_p >> 1 : 0));
		tile_size_out_w = (core_w - conf_p) / stride_w + 1;
		tile_size_out_h = (core_h - conf_p) / stride_h + 1;
		tile_size_out = tile_size_out_w * tile_size_out_h;

		if (!conf_conv_enable) {
			tile_size_out_w = core_w;
			tile_size_out_h = core_h;
			tile_size_out = tile_size_in;
		}

		if (upsample_enable) {
			tile_size_out_pool_w = 2 * tile_size_out_w;
			tile_size_out_pool_h = 2 * tile_size_out_h;
		} else {
			tile_size_out_pool_w = (tile_size_out_w - pool_size_w) / pool_stride_w + 1;
			tile_size_out_pool_h = (tile_size_out_h - pool_size_h) / pool_stride_h + 1;
		}
		tile_size_out_pool = tile_size_out_pool_w * tile_size_out_pool_h;

		ubuf_ifm_offset = 0;

		ubuf_ifm_blk_size_16 = tile_size_in_xmem * (conf_c > CONV_C_SUB ? CONV_C_SUB : conf_c);
		ubuf_ifm_blk_size_128 = (ubuf_ifm_blk_size_16 & 0x7) == 0 ? (ubuf_ifm_blk_size_16 >> 3) : ((ubuf_ifm_blk_size_16 >> 3) + 1);
		ubuf_ifm_blk_size_128 += ((2 - ubuf_ifm_blk_size_128) & 0xF);
		ubuf_ifm_size_128 = ubuf_ifm_blk_size_128 * C_blocks;
		ubuf_ifm_size_128 += ((0 - ubuf_ifm_size_128) & 0xF);
		ubuf_input_size = (ubuf_ifm_size_128 << 4);

		if (!is_input_node) {
			ubuf_ifm_offset = prev_ubuf_ofm_offset;
			ubuf_ifm_blk_size_16 = prev_ubuf_ofm_blk_size_16;
			ubuf_ifm_blk_size_128 = prev_ubuf_ofm_blk_size_128;
			ubuf_ifm_size_128 = prev_ubuf_ofm_size_128;
			ubuf_input_size = prev_ubuf_output_size;
		}

		ubuf_ofm_offset = ubuf_ifm_offset + (((run_w == node_w - 1) && (conf_conv_enable == 0)) ? 0 : ubuf_input_size);

		ubuf_ofm_blk_size_16 = ((conf_conv_enable == 0) ? tile_size_out_pool : tile_size_out ) * (conf_m > CONV_M_SUB ? CONV_M_SUB : conf_m);
		ubuf_ofm_blk_size_128 = (ubuf_ofm_blk_size_16 & 0x7) == 0 ? (ubuf_ofm_blk_size_16 >> 3) : ((ubuf_ofm_blk_size_16 >> 3) + 1);
		ubuf_ofm_blk_size_128 += ((2 - ubuf_ofm_blk_size_128) & 0xF);
		ubuf_ofm_size_128 = ubuf_ofm_blk_size_128 * M_blocks;
		ubuf_ofm_size_128 += ((0 - ubuf_ofm_size_128) & 0xF);
		ubuf_output_size = (ubuf_ofm_size_128 << 4);

		// Ping-pong buffering...
		if (node_w == 1) {
			if (run & 1) {
				ubuf_ofm_offset = 0;
			}
			else {
				ubuf_ifm_offset = 0;
				ubuf_ofm_offset = ubuf_size - ((is_output_node ? ubuf_ofm_blk_size_128 : ubuf_ofm_size_128) << 4);
			}
		}
		// <<< END

		if (!is_output_node) {
			prev_ubuf_ofm_offset = ubuf_ofm_offset;
			prev_ubuf_ofm_blk_size_16 = ubuf_ofm_blk_size_16;
			prev_ubuf_ofm_blk_size_128 = ubuf_ofm_blk_size_128;
			prev_ubuf_ofm_size_128 = ubuf_ofm_size_128;
			prev_ubuf_output_size = ubuf_output_size;
		}

		if (is_output_node)
			max_ubuf_addr_run = ubuf_ofm_offset + (ubuf_ofm_blk_size_128 << 4);
		else
			max_ubuf_addr_run = ubuf_ofm_offset + (ubuf_ofm_size_128 << 4);

		// Ping-pong buffering...
		// NOTE: In this case max_ubuf_add_run actually means total ubuf usage, not max end address... but this meaning could be used for the other cases too...
		if (node_w == 1)
			max_ubuf_addr_run = ubuf_input_size + ((is_output_node ? ubuf_ofm_blk_size_128 : ubuf_ofm_size_128) << 4);
		// <<< END

		if (max_ubuf_addr_run > max_ubuf_addr)
			max_ubuf_addr = max_ubuf_addr_run;

		topo >>= 1;
		if (is_output_node) {
			run_w++;
		}
		is_input_node = is_output_node;
	}

	return max_ubuf_addr;
}


/// @brief Input/output dimensions for the supported convolutional operations.
struct conv_data_size {
	int32_t w;			// Width
	int32_t h;			// Height
	int32_t z;			// Depth
	int32_t c;			// Channels
	uint32_t size;	// Total Size = w * h * z * c * sizeof(float16)
};


/// @brief Returns output width for convolution.
/// @param width Input width.
/// @param kx Kernel width (for dilated convolution pass (kx - 1) * dil + 1).
/// @param pad_left Left padding.
/// @param pad_right Right padding.
/// @param stride Stride.
/// @param is_deconv 1 for deconvolution, 0 otherwise.
static inline int get_conv_out_width(
		int width, int kx, int pad_left, int pad_right,
		int stride, int is_deconv)
{
	return (pad_left + (is_deconv ? (width - 1) * stride + 1 : width) + pad_right - kx) / (is_deconv ? 1 : stride) + 1;
}


/// @brief Returns non-zero if the provided run specifies deconvolution.
static inline int is_deconv_v0(const struct dmp_dv_kcmdraw_conv_v0_run *run) {
	return (run->conv_enable & 4) ? 1 : 0;
}


/// @brief Returns non-zero if the provided run specifies 2D convolution.
static inline int is_conv_2d_v0(const struct dmp_dv_kcmdraw_conv_v0_run *run) {
	return ((run->conv_enable & 2) || (!run->conv_enable)) ? 0 : 1;
}


/// @brief Divides a by b and rounds up the result.
static inline int divup(int a, int b) {
	return a / b + (a % b ? 1 : 0);
}


/// @brief Calculates number of tiles for Convolutional and LRN layers.
/// @param w Width of input.
/// @param h Height of input.
/// @param c Number of input channels.
/// @param m Number of output channels.
/// @param p_x Horizontal filter size.
/// @param p_y Vertical filter size.
/// @param pad_x0 Left padding.
/// @param pad_x1 Right padding.
/// @param pad_y0 Top padding.
/// @param pad_y1 Bottom padding.
/// @param stride_x Horizontal stride.
/// @param stride_y Vertical stride.
/// @param dil_x Horizontal dilation.
/// @param dil_y Vertical dilation.
/// @param u_kb Size of unified buffer in Kb.
/// @param is_conv 1 for convolution, 0 for LRN.
/// @param is_deconv 1 for deconvolution, 0 otherwise.
/// @param in_u_b Number of bytes required in unified buffer for input chunk.
/// @param out_u_b Number of bytes required in unified buffer for output chunk.
/// @return Number of tiles >= 1 on success, 0 when unified buffer is too small.
static int calc_num_tiles_conv_lrn(
	int w, int h, int c, int m, int p_x, int p_y,
	int pad_x0, int pad_x1, int pad_y0, int pad_y1,
	int stride_x, int stride_y, int dil_x, int dil_y,
	int u_kb, int is_conv, int is_deconv,
	int *in_u_b, int *out_u_b)
{
	const int u = u_kb * 1024 / 2;  // size of unified buffer in number of float16

	const int C_blocks = (c >> CONV_C_SUB_LOG2) + (((c & ((1 << CONV_C_SUB_LOG2) - 1)) == 0) ? 0 : 1);

	const int is_dil = (dil_x > 1) || (dil_y > 1);

	int p, p_eff_x, p_eff_y, no_ub, no_ob, uu, t, does_not_fit, tw,
	    ts_1c, ow, oh, os, ts_blk16, ts_blk128, ts_128, ts;

	// Adjust kernel size
	if (!is_dil) {
		p_x = (p_x > p_y ? p_x : p_y) | 1;
		p_y = p_x;
	}
	p = is_dil ? 1 : p_x;
	p_eff_x = (p_x - 1) * dil_x + 1;
	p_eff_y = (p_y - 1) * dil_y + 1;

	// In some cases no unified buffer is needed at all
	no_ub = 0;
	if (is_conv) {
		if ((p <= 7) && (!is_dil)) {
			if ((c <= 8) && (m <= 8)) {
				no_ub = 1;
			}
		}
		if (no_ub) {
			*in_u_b = 0;
			*out_u_b = 0;
			return 1;
		}
	}

	// In some cases no output buffer is needed
	no_ob = 0;
	if ((is_conv) && (!is_dil)) {
		if (p <= 7) {
			if (c <= 8) {
				no_ob = 1;
			}
		} else if (p <= 11) {
			if (c <= 4) {
				no_ob = 1;
			}
		} else if (p > 11) {
			if (c == 1) {
				no_ob = 1;
			}
		}
	}

	uu = 0;
	t = 0;
	does_not_fit = 0;
	ts = 0;
	os = 0;
	do {
		t++;
		tw = divup(w, t) + ((is_dil ? p_eff_x : p) - 1);  // width of tile
		does_not_fit = (tw < (is_dil ? p_eff_x : p)) || (t > w);
		if (does_not_fit) {
			break;
		}
		ts_1c = tw * h;  // tile size for single channel

		ow = get_conv_out_width(tw, (is_dil ? p_eff_x : p), pad_x0, pad_x1, stride_x, is_deconv);
		oh = get_conv_out_width( h, (is_dil ? p_eff_y : p), pad_y0, pad_y1, stride_y, is_deconv);

		os = ow * oh * (m > 8 ? 8 : m);  // output buffer size

		ts_blk16 = ts_1c * (c > CONV_C_SUB ? CONV_C_SUB : c);
		ts_blk128 = (ts_blk16 >> 3) + ((ts_blk16 & 0x7) == 0 ? 0 : 1);
		if (1) {  // (p == 1) // Ensure size modulo 16 = 2, this to ensure 8 blocks can be read in parallel from 16 cuts in 1x1 mode
			ts_blk128 += ((2 - ts_blk128) & 0xF);
		}
		ts_128 = ts_blk128 * C_blocks;
		if (1) {  // (p == 1) // Ensure size modulo 16 = 0, this to ensure 8 blocks can be read in parallel from 16 cuts in 1x1 mode
			ts_128 += ((0 - ts_128) & 0xF);
		}
		// Input tile size in UBUF (in float16)
		ts = (ts_128 << 3);

		uu = ts + (no_ob ? 0 : os); // unified buffer utilization
	} while (uu > u);

	*in_u_b = ts << 1;
	*out_u_b = no_ob ? 0 : os << 1;

	return does_not_fit ? 0 : t;
}


/// @brief Calculates number of tiles for Convolutional layer.
/// @param w Width of input.
/// @param h Height of input.
/// @param c Number of input channels.
/// @param m Number of output channels.
/// @param p_x Horizontal filter size.
/// @param p_y Vertical filter size.
/// @param pad_x0 Left padding.
/// @param pad_x1 Right padding.
/// @param pad_y0 Top padding.
/// @param pad_y1 Bottom padding.
/// @param stride_x Horizontal stride.
/// @param stride_y Vertical stride.
/// @param dil_x Horizontal dilation.
/// @param dil_y Vertical dilation.
/// @param u_kb Size of unified buffer in Kb.
/// @param is_deconv 1 for deconvolution, 0 otherwise.
/// @param in_u_b Number of bytes required in unified buffer for input chunk.
/// @param out_u_b Number of bytes required in unified buffer for output chunk.
/// @return Number of tiles >= 1 on success, 0 when unified buffer is too small.
static inline int calc_num_tiles_conv(
	int w, int h, int c, int m, int p_x, int p_y,
	int pad_x0, int pad_x1, int pad_y0, int pad_y1,
	int stride_x, int stride_y, int dil_x, int dil_y,
	int u_kb, int is_deconv, int *in_u_b, int *out_u_b)
{
	return calc_num_tiles_conv_lrn(
		w, h, c, m, p_x, p_y, pad_x0, pad_x1, pad_y0, pad_y1,
		stride_x, stride_y, dil_x, dil_y, u_kb, 1, is_deconv,
		in_u_b, out_u_b);
}


/// @brief Calculates number of tiles for Pooling layer.
/// @param w Width of input.
/// @param h Height of input.
/// @param c Number of input channels.
/// @param in_u_b Number of bytes required in unified buffer for input chunk.
/// @param out_u_b Number of bytes required in unified buffer for output chunk.
/// @return Number of tiles >= 1 on success, 0 when unified buffer is too small.
static int calc_num_tiles_pool(int w, int h, int c,
			       int *in_u_b, int *out_u_b)
{
	const int u = 131072 * 1024 / 2;  // maximum supported buffer size in number of float16
	int t, uu;

	*in_u_b = 0;
	*out_u_b = 0;

	t = 0;
	while (t < w) {
		t++;
		uu = (w * h * (c > 8 ? 8 : c)) / t;
		if (uu <= u) {
			return t;
		}
	}
	return 0;
}


/// @brief Calculates number of tiles for LRN layer.
/// @param w Width of input.
/// @param h Height of input.
/// @param c Number of input channels.
/// @param u_kb Size of unified buffer in Kb.
/// @param in_u_b Number of bytes required in unified buffer for input chunk.
/// @param out_u_b Number of bytes required in unified buffer for output chunk.
/// @return Number of tiles >= 1 on success, 0 when unified buffer is too small.
static int calc_num_tiles_lrn(int w, int h, int c, int u_kb,
			      int *in_u_b, int *out_u_b)
{
	return calc_num_tiles_conv_lrn(
		w, h, c, c, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, u_kb, 0, 0,
		in_u_b, out_u_b);
}


/// @brief Returns number of tiles required for given command execution.
/// @param cmd Command for execution.
/// @param ubuf_size Unified buffer size.
/// @param in_u_b Number of bytes required in unified buffer for input chunk.
/// @param out_u_b Number of bytes required in unified buffer for output chunk.
/// @return Number of tiles >= 1 on success, 0 when unified buffer is too small.
static uint16_t get_conv_tiles_v0(
	const struct dmp_dv_kcmdraw_conv_v0 *cmd, int ub_size,
	int *in_u_b, int *out_u_b)
{
	int w, h, c, m, kx, ky;
	int pad[4], stride[2], dil[2];

	if (topo_num_runs(cmd->topo) > 1) {
		return 1;
	}

	w = cmd->w;
	h = cmd->h;
	c = cmd->c;
	m = cmd->run[0].m;
	kx = cmd->run[0].p & 0xFF;
	ky = (cmd->run[0].p & 0xFF00) ? (cmd->run[0].p & 0xFF00) >> 8 : kx;
	pad[0] = cmd->run[0].conv_pad & 0x7F;
	pad[1] = (cmd->run[0].conv_pad >> 8) & 0xFF;
	pad[2] = (cmd->run[0].conv_pad >> 16) & 0x7F;
	pad[3] = (cmd->run[0].conv_pad >> 24) & 0xFF;
	stride[0] = cmd->run[0].conv_stride & 0xFF;
	stride[1] = (cmd->run[0].conv_stride >> 8) & 0xFF;
	dil[0] = cmd->run[0].conv_dilation & 0xFF;
	dil[1] = (cmd->run[0].conv_dilation >> 8) & 0xFF;

	if (cmd->run[0].lrn & 1)
		return calc_num_tiles_lrn(
			w, h, c, ub_size >> 10, in_u_b, out_u_b);

	if (!is_conv_2d_v0(&cmd->run[0])) {
		if (cmd->run[0].pool_enable)
			return calc_num_tiles_pool(
				w, h, c, in_u_b, out_u_b);
		return 1;
	}

	return calc_num_tiles_conv(
		w, h, c, m, kx, ky,
		pad[0], pad[1], pad[2], pad[3],
		stride[0], stride[1], dil[0], dil[1],
		ub_size >> 10, is_deconv_v0(&cmd->run[0]),
		in_u_b, out_u_b);
}


/// @brief Returns size in bytes for the packed weights for convolutional operation.
/// @param c Number of input channels.
/// @param m Number of convolutional kernels (number of output channels).
/// @param k Kernel size: max(kx, ky) | 1.
/// @param quantized Use quantized weights or not.
/// @param dw Use depth-wise convolution or not.
/// @param prelu The activation is prelu or not.
static uint32_t get_weight_size(int c, int m, int k, int quantized, int dw, int prelu)
{
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
		res = (res + 0xf) & (~0xf);
	}
	else {
		res = 144 * m * c + 16 * ((m + 7) / 8);
	}
	if (prelu) {
		res += 16 * ((m + 7) / 8);
	}
	return res;
}


/// @brief Returns size in bytes for the packed weights for dilated convolution.
/// @param c Number of input channels.
/// @param m Number of convolutional kernels (number of output channels).
/// @param kx Kernel width.
/// @param ky Kernel height.
/// @param quantized Use quantized weights or not.
static uint32_t get_weight_size_dil(int c, int m, int kx, int ky, int quantized)
{
	uint32_t res = quantized ? 512 : 0;
	const int n = kx * ky;
	int i, d;

	for (i = 0; i < n; i++) {
		res += get_weight_size(c, m, 1, quantized, 0, 0);
		if (quantized)
			res -= 512;
		d = res & 15;
		if (d)
			res += 16 - d;
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
	in_size->size = (uint32_t)w * h * z * c * 2;	// we are using 16-bit floats
}


/// @brief Assigns input size.
static inline void init_conv_input_size_v0(
		const struct dmp_dv_kcmdraw_conv_v0 *cmd,
		struct conv_data_size *in_size) {
	init_conv_input_size_v0_4(cmd->w, cmd->h, cmd->z, cmd->c, in_size);
}


/// @brief Fills output size and weights size for layer configuration version 0.
static void get_conv_output_size_v0(
		const struct dmp_dv_kcmdraw_conv_v0_run *run,
		const struct conv_data_size *in_size,
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
		const int m = run->m;
		const int p = run->p;
		const int px = p & 0xFF;
		const int py0 = (p >> 8) & 0xFF;
		const int py = py0 ? py0 : px;
		const int pz = run->pz & 0x7F;
		const int conv_pad = run->conv_pad;
		const int conv_stride = run->conv_stride;
		const int pad_w0 = conv_pad & 0x7F;
		const int pad_w1 = (conv_pad >> 8) & 0xFF;
		const int pad_h0 = (conv_pad >> 16) & 0x7F;
		const int pad_h1 = (conv_pad >> 24) & 0xFF;
		const int stride_w = conv_stride & 0xFF;
		const int stride_h = (conv_stride >> 8) & 0xFF;
		const int dil_x0 = run->conv_dilation & 0xFF;
		const int dil_y0 = (run->conv_dilation >> 8) & 0xFF;
		const int dil_x = dil_x0 > 1 ? dil_x0 : 1;
		const int dil_y = dil_y0 > 1 ? dil_y0 : 1;

		t0_w = get_conv_out_width(in_w, (px - 1) * dil_x + 1, pad_w0, pad_w1, stride_w, is_deconv_v0(run));
		t0_h = get_conv_out_width(in_h, (py - 1) * dil_y + 1, pad_h0, pad_h1, stride_h, is_deconv_v0(run));
		// NOTE: No padding or stride in Z (depth) implemented yet.
		t0_z = (in_z - pz + 1);
		t0_c = m;  // number of output channels
		if ((dil_x == 1) && (dil_y == 1))
			*w_size = get_weight_size(
				in_c, m, ((px > py) ? px : py) | 1, (run->weight_fmt & 2),
				(run->conv_enable & 2), (run->actfunc == 4));
		else
			*w_size = get_weight_size_dil(in_c, m, px, py, (run->weight_fmt & 2));
	}
	else {	// Bypass of convolution
		t0_w = in_w;
		t0_h = in_h;
		t0_z = in_z;
		t0_c = in_c;
		*w_size = 0;
	}

	// Pooling
	if (((run->pool_enable) & 0x7) != 0) {
		int pool_size = run->pool_size;
		int pool_size_w = pool_size & 0xFF;
		int pool_size_h = (pool_size >> 8) & 0xFF;
		int pool_pad = run->pool_pad;
		int pool_pad_w0 = pool_pad & 0xFF;
		int pool_pad_w1 = (pool_pad >> 8) & 0xFF;
		int pool_pad_h0 = (pool_pad >> 16) & 0xFF;
		int pool_pad_h1 = (pool_pad >> 24) & 0xFF;
		int pool_stride = run->pool_stride;
		int pool_stride_w = pool_stride & 0xFF;
		int pool_stride_h = (pool_stride >> 8) & 0xFF;
		// Unpool with argmax, or upsample
		if ((run->pool_enable == 5) || (run->pool_enable == 4)) {
			// NOTE: only 2x2 size and 2x2 stride supported currently.
			out_size->w = 2 * t0_w;
			out_size->h = 2 * t0_h;
		}
		else {
			out_size->w = ((pool_pad_w0 + t0_w + pool_pad_w1) -
				       pool_size_w) / pool_stride_w + 1;
			out_size->h = ((pool_pad_h0 + t0_h + pool_pad_h1) -
				       pool_size_h) / pool_stride_h + 1;
		}
		// NOTE: No pooling in Z (depth) implemented yet.
		out_size->z = t0_z;
		// Number of channels preserved in pooling
		out_size->c = t0_c;
	}
	else {	// Bypass of max pooling
		out_size->w = t0_w;
		out_size->h = t0_h;
		out_size->z = t0_z;
		out_size->c = t0_c;
	}
	out_size->size = (uint32_t)out_size->w * out_size->h * out_size->z * out_size->c * 2;
}


#endif	// _UAPI_LINUX_DMP_DV_DIMENSIONS_H
