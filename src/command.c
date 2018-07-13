/*
 *  DV700 kernel driver - command conversion
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
 *
 */

#include <linux/types.h>

#include "dmp-dv.h"
#include "../uapi/dmp_dv_cmdraw_v0.h"

#define MAX_NUM_RUNS 32

struct conv_header {
	uint32_t topo; // Output Destination of each run
};

struct conv_input {
	uint16_t w; // Input Width
	uint16_t h; // Input Height
	uint16_t z; // Input Depth
	uint16_t c; // Input Channels
	uint32_t input_base_addr; // Input byte address
	uint16_t input_circular_offset; // Input Depth circular offset
	uint16_t tiles; // Number of horizontal tiles
};

struct conv_output {
	uint32_t output_base_addr; // Output address
	uint32_t eltwise_base_addr; // Elementwise add address (0 = From UBUF), also used for argmax input/output address
	uint16_t output_mode; // 0 = concat, 1 = eltwise add
	uint16_t align_0;
};

struct conv_run {
	uint16_t m; // Output Channels
	// CONV
	uint16_t conv_enable; // 1 = Enabled, 0 = Disabled, 3 = depthwise
	uint16_t p; // Filter Size (width = height)
	uint16_t pz; // Filter Depth (1 in case of 2D convolution)
	uint32_t conv_pad; // bits [7:0] = left padding, bits [15:8] = right padding, bits [23:16] = top padding, bits [31:24] = bottom padding
	uint16_t conv_stride; // bits [7:0] = X stride, bits [15:8] = Y stride
	uint16_t conv_dilation; // bits [7:0] = X dilation, bits [15:8] = Y dilation
	uint32_t weight_base_addr; // Filter Weight and Bias byte address
	uint16_t weight_fmt; // Weight format (0 = random access blocks, 1 = compact stream, 2 = 8-bit qunatized stream)
	uint16_t align_0;
	// POOL
	uint16_t pool_enable; // 0 = disabled, 1 = max pooling, 2 = average pooling, 3 = max pooling with argmax, 4 = upsample, 5 = unpool with argmax
	uint16_t pool_avg_param; // Must be set to 1/pool_size^2 in FP16 format when using average pooling (average pooling assumes square size)
	uint16_t pool_size; // bits [7:0] = width, bits [15:8] = height
	uint16_t pool_stride; // bits [7:0] = X stride, bits [15:8] = Y stride
	uint32_t pool_pad; // bits [7:0] = left padding, bits [15:8] = right padding, bits [23:16] = top padding, bits [31:24] = bottom padding
	// MISC
	uint16_t actfunc; // Activation Function: 0 = None, 1 = Tanh, 2 = Leaky ReLU, 3 = Sigmoid, 4 = PReLU, 5 = ELU, 6 = ReLU6
	uint16_t actfunc_param; // Leaky ReLU parameter (NOTE: 0x2E66 is 0.1 in FP16)
	uint16_t rectifi_en; // Rectification, i.e. abs(x) (NOTE: Can be applied after non-ReLU activation function)
	uint16_t lrn; // [0] : 1 = LRN enable, 0 = LRN disable, [1] : 1 = incl. power func, 0 = excl., [8:11] = x^2 scale factor log2
};

struct conv_configuration {
	struct conv_header header;
	struct conv_input input;
	struct conv_output output;
	struct conv_run run[MAX_NUM_RUNS];
};

static int topo_num_runs(unsigned int i)
{
	int n = 0;
	for (; i; i >>= 1, n++)
		; // Faster for low n...
	return n;
}

// Size of the configuration struct in bytes (unused run structs not counted)
static int conf_size(struct conv_configuration *conf)
{
	int n = topo_num_runs(conf->header.topo);
	return sizeof(struct conv_configuration) -
	       (MAX_NUM_RUNS - n) * sizeof(struct conv_run);
}

#if 0
// TODO: Need to update the implementation.
/**************************************
 *
 * Command buffer size verification
 *
 **************************************/
#define CONV_C_SUB 8
#define CONV_M_SUB 8

struct conv_data_size {
	unsigned short w; // Width
	unsigned short h; // Height
	unsigned short z; // Depth
	unsigned short c; // Channels
	unsigned int size; // Total Size = w * h * z * c
};

struct tb_data_size {
	unsigned int i_buf;
	unsigned int o_buf;
	unsigned int w_bank;
	unsigned int w[MAX_NUM_RUNS]; // Filter sizes
	unsigned int b[MAX_NUM_RUNS]; // Bias sizes
	struct conv_data_size co[MAX_NUM_RUNS]; // convolution out sizes
	struct conv_data_size po[MAX_NUM_RUNS]; // pool out sizes
	struct conv_data_size i;
	struct conv_data_size o;
};

static void tb_data_size_clear(struct tb_data_size *tbs)
{
	int i;
	tbs->i_buf = 0;
	tbs->o_buf = 0;
	tbs->w_bank = 0;
	for (i = 0; i < MAX_NUM_RUNS; i++) {
		tbs->w[i] = 0;
		tbs->b[i] = 0;
	}
}

static void conv_output_size_calc(struct conv_run *run, struct conv_data_size *in_size,
			   struct conv_data_size *out_size,
			   struct tb_data_size *tb_size, int run_num)
{
	short in_w = in_size->w;
	short in_h = in_size->h;
	short in_z = in_size->z;
	short in_c = in_size->c;
	short t0_w;
	short t0_h;
	short t0_z;
	short t0_c;
	// Convolution
	if (run->conv_enable) { // Convolution
		short m = run->m;
		short p = run->p;
		short px = p & 0xFF;
		short py = p >> 8;
		short pz = run->pz & 0x7F;
		unsigned int conv_pad = run->conv_pad;
		unsigned short conv_stride = run->conv_stride;
		int pad_w0 = conv_pad & 0xff;
		int pad_w1 = (conv_pad >> 8) & 0xff;
		int pad_h0 = (conv_pad >> 16) & 0xff;
		int pad_h1 = (conv_pad >> 24) & 0xff;
		int stride_w = conv_stride & 0xff;
		int stride_h = (conv_stride >> 8) & 0xff;
		int core_w = pad_w0 + in_w + pad_w1;
		int core_h = pad_h0 + in_h + pad_h1;
		int i_buf_size, o_buf_size, w_bank_size, w_size;
		if (py == 0)
			py = px;
		t0_w = (core_w - px) / stride_w + 1;
		t0_h = (core_h - py) / stride_h + 1;
		// NOTE: No padding or stride in Z (depth) implemented yet!
		t0_z = (in_z - pz + 1);
		t0_c = m; // Number of convolution output channels...
		i_buf_size = core_w * core_h * in_z * CONV_C_SUB;
		o_buf_size = t0_w * t0_h * t0_z * CONV_M_SUB;
		w_bank_size = px * py * CONV_C_SUB * CONV_M_SUB;
		if (i_buf_size > tb_size->i_buf)
			tb_size->i_buf = i_buf_size;
		if (o_buf_size > tb_size->o_buf)
			tb_size->o_buf = o_buf_size;
		if (w_bank_size > tb_size->w_bank)
			tb_size->w_bank = w_bank_size;
		w_size = px * py * pz * in_size->c * m;
		tb_size->w[run_num] = w_size;
		tb_size->b[run_num] = m;
	} else { // Bypass of convolution
		t0_w = in_w;
		t0_h = in_h;
		t0_z = in_z;
		t0_c = in_c;
		tb_size->w[run_num] = 0;
		tb_size->b[run_num] = 0;
	}
	tb_size->co[run_num].w = t0_w;
	tb_size->co[run_num].h = t0_h;
	tb_size->co[run_num].z = t0_z;
	tb_size->co[run_num].c = t0_c;
	tb_size->co[run_num].size = t0_w * t0_h * t0_z * t0_c;
	// Pooling
	if (((run->pool_enable) & 0x7) != 0) {
		unsigned short pool_size = run->pool_size;
		int pool_size_w = pool_size & 0xff;
		int pool_size_h = (pool_size >> 8) & 0xff;
		unsigned int pool_pad = run->pool_pad;
		int pool_pad_w0 = pool_pad & 0xff;
		int pool_pad_w1 = (pool_pad >> 8) & 0xff;
		int pool_pad_h0 = (pool_pad >> 16) & 0xff;
		int pool_pad_h1 = (pool_pad >> 24) & 0xf;
		unsigned short pool_stride = run->pool_stride;
		int pool_stride_w = pool_stride & 0xff;
		int pool_stride_h = (pool_stride >> 8) & 0xff;
		// unpool with argmax, or upsample
		if ((run->pool_enable == 5) || (run->pool_enable == 4)) {
			// NOTE: only 2x2 size and 2x2 stride supported currently
			out_size->w = 2 * t0_w;
			out_size->h = 2 * t0_h;
		} else {
			out_size->w = ((pool_pad_w0 + t0_w + pool_pad_w1) -
				       pool_size_w) / pool_stride_w + 1;
			out_size->h = ((pool_pad_h0 + t0_h + pool_pad_h1) -
				       pool_size_h) / pool_stride_h + 1;
		}
		// NOTE: No pooling in Z (depth) implemented yet!
		out_size->z = t0_z;
		// Number of channels preserved in pooling...
		out_size->c = t0_c;
	} else { // Bypass of max pooling
		out_size->w = t0_w;
		out_size->h = t0_h;
		out_size->z = t0_z;
		out_size->c = t0_c;
	}
	out_size->size = out_size->w * out_size->h * out_size->z * out_size->c;
	tb_size->po[run_num].w = out_size->w;
	tb_size->po[run_num].h = out_size->h;
	tb_size->po[run_num].z = out_size->z;
	tb_size->po[run_num].c = out_size->c;
	tb_size->po[run_num].size = out_size->size;
}
#endif

int dv_convert_command(void *cmd_buf, dmp_dv_kcmd *cmd_info) {
	return 0;
}
