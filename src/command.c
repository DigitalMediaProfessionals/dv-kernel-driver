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
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/hashtable.h>

#include "dmp-dv.h"
#include "../uapi/dmp_dv_cmdraw_v0.h"

#define MAX_NUM_RUNS 32
#define CMB_SIZE (16 * PAGE_SIZE)

#define REG_IO_ADDR(BAR, OF) ((void __iomem *)(BAR) + OF)

struct dmp_dmabuf_hash_entry {
	int fd;
	enum dma_data_direction dir;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	struct hlist_node hash_node;
};

struct dmp_cmb {
	dma_addr_t physical;
	void *logical;
	size_t size;

	DECLARE_HASHTABLE(dmabuf_hash, 6);
};

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

int dv_cmb_init(struct device *dev, struct dmp_cmb **cmb)
{
	*cmb = kzalloc(sizeof(**cmb), GFP_KERNEL);
	if (!(*cmb))
		return -ENOMEM;

	(*cmb)->logical =
		dma_alloc_coherent(dev, CMB_SIZE, &(*cmb)->physical, GFP_KERNEL);
	if (!(*cmb)->logical) {
		kfree(*cmb);
		*cmb = NULL;
		return -ENOMEM;
	}
	(*cmb)->size = 0;

	hash_init((*cmb)->dmabuf_hash);

	return 0;
}

void dv_cmb_finalize(struct device *dev, struct dmp_cmb *cmb)
{
	int bkt;
	struct hlist_node *tmp;
	struct dmp_dmabuf_hash_entry *obj;

	hash_for_each_safe(cmb->dmabuf_hash, bkt, tmp, obj, hash_node) {
		dma_buf_unmap_attachment(obj->attachment, obj->sgt, obj->dir);
		dma_buf_detach(obj->dma_buf, obj->attachment);
		dma_buf_put(obj->dma_buf);
		hash_del(&obj->hash_node);
		kfree(obj);
	}
	dma_free_coherent(dev, CMB_SIZE, cmb->logical, cmb->physical);
	kfree(cmb);
}

static unsigned int topo_num_runs(unsigned int i)
{
	unsigned int n = 0;
	for (; i; i >>= 1, n++)
		; // Faster for low n...
	return n;
}

// Size of the configuration struct in bytes (unused run structs not counted)
static size_t conf_size(unsigned int topo)
{
	unsigned int n = topo_num_runs(topo);
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

static int get_dma_addr(struct device *dev, struct dmp_cmb *cmb,
			dmp_dv_kbuf *buf, int is_weight, uint32_t *addr)
{
	struct dmp_dmabuf_hash_entry *obj;
	struct scatterlist *sg;
	int i, ret = 0;

	// handle invalid fd
	if (buf->fd < 0) {
		*addr = 0xDEADBEEF;
		return 0;
	}

	// find if the fd is already in the hash
	hash_for_each_possible (cmb->dmabuf_hash, obj, hash_node, buf->fd) {
		if (obj->fd == buf->fd) {
			*addr = obj->dma_addr + (uint32_t)buf->offs;
			return 0;
		}
	}

	// create a new entry in hash table
	obj = kmalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return -ENOMEM;

	obj->fd = buf->fd;
	obj->dma_buf = dma_buf_get(buf->fd);
	if (IS_ERR(obj->dma_buf)) {
		ret = PTR_ERR(obj->dma_buf);
		goto dma_buf_get_fail;
	}
	obj->attachment = dma_buf_attach(obj->dma_buf, dev);
	if (IS_ERR(obj->attachment)) {
		ret = PTR_ERR(obj->attachment);
		goto dma_buf_attach_fail;
	}
	obj->dir = (is_weight ? DMA_TO_DEVICE : DMA_BIDIRECTIONAL);
	obj->sgt = dma_buf_map_attachment(obj->attachment, obj->dir);
	if (IS_ERR(obj->sgt)) {
		ret = PTR_ERR(obj->sgt);
		goto dma_buf_map_attachment_fail;
	}
	if (obj->sgt->nents > 1) {
		ret = -EINVAL;
		goto dma_buf_invalid_nents;
	}

	// should only loop once
	for_each_sg (obj->sgt->sgl, sg, obj->sgt->nents, i) {
		obj->dma_addr = sg_dma_address(sg);
		*addr = obj->dma_addr + (uint32_t)buf->offs;
	}

	hash_add(cmb->dmabuf_hash, &obj->hash_node, obj->fd);

	return 0;

dma_buf_invalid_nents:
	dma_buf_unmap_attachment(obj->attachment, obj->sgt, obj->dir);
dma_buf_map_attachment_fail:
	dma_buf_detach(obj->dma_buf, obj->attachment);
dma_buf_attach_fail:
	dma_buf_put(obj->dma_buf);
dma_buf_get_fail:
	kfree(obj);
	return ret;
}

static int dv_convert_conv_v0(struct device *dev, struct dmp_cmb *cmb,
			      dmp_dv_kcmdraw __user *user_cmd, size_t size)
{
	dmp_dv_kcmdraw_v0 *cmd;
	struct conv_configuration *conv;
	size_t cmd_size, conv_len;
	uint32_t *cmd_buf;
	unsigned int runs, i;
	uint32_t input_base_addr;
	uint32_t output_base_addr;
	uint32_t eltwise_base_addr;
	uint32_t weight_base_addr;
	int ret;

	cmd = vmalloc(size);
	if (!cmd)
		return -ENOMEM;
	if (copy_from_user(cmd, user_cmd, size)) {
		vfree(cmd);
		return -EFAULT;
	}

	ret = get_dma_addr(dev, cmb, &cmd->input_buf, 0, &input_base_addr);
	if (ret) {
		vfree(cmd);
		return ret;
	}
	ret = get_dma_addr(dev, cmb, &cmd->output_buf, 0, &output_base_addr);
	if (ret) {
		vfree(cmd);
		return ret;
	}
	ret = get_dma_addr(dev, cmb, &cmd->eltwise_buf, 0, &eltwise_base_addr);
	if (ret) {
		vfree(cmd);
		return ret;
	}

	runs = topo_num_runs(cmd->topo);
	cmd_size = conf_size(cmd->topo) + sizeof(uint32_t) * 5;
	conv_len = (conf_size(cmd->topo) + 3) / 4;
	// TODO: check the cmb remaining size is big enough for this command
	//       and allocate new buffer when so
	cmd_buf = (uint32_t *)((uint8_t *)cmb->logical + cmb->size);
	conv = (struct conv_configuration *)(cmd_buf + 3);
	
	cmd_buf[0] = 0x0020f004; // Write one word to 0x20
	cmd_buf[1] = 0x00002000; // conv config start address in RISC-V memory
	cmd_buf[2] = 0x0021f004 | ((conv_len - 1) << 3); // Write conv_len words to 0x21
	cmd_buf[3 + conv_len] = 0x0010f004; // Write one word to 0x10
	cmd_buf[4 + conv_len] = 0x00000001; // Start conv HW

	conv->header.topo = cmd->topo;

	conv->input.w = cmd->w;
	conv->input.h = cmd->h;
	conv->input.z = cmd->z;
	conv->input.c = cmd->c;
	conv->input.input_base_addr = input_base_addr;
	conv->input.input_circular_offset = cmd->input_circular_offset;
	conv->input.tiles = 1; //TODO

	conv->output.output_base_addr = output_base_addr;
	conv->output.eltwise_base_addr = eltwise_base_addr;
	conv->output.output_mode = cmd->output_mode;

	for (i = 0; i < runs; ++i) {
		ret = get_dma_addr(dev, cmb, &cmd->run[i].weight_buf, 1,
		                   &weight_base_addr);
		if (ret) {
			vfree(cmd);
			return ret;
		}
		conv->run[i].m = cmd->run[i].m;
		conv->run[i].conv_enable = cmd->run[i].conv_enable;
		conv->run[i].p = cmd->run[i].p;
		conv->run[i].pz = cmd->run[i].pz;
		conv->run[i].conv_pad = cmd->run[i].conv_pad;
		conv->run[i].conv_stride = cmd->run[i].conv_stride;
		conv->run[i].conv_dilation = cmd->run[i].conv_dilation;
		conv->run[i].weight_base_addr = weight_base_addr;
		conv->run[i].weight_fmt = cmd->run[i].weight_fmt;
		conv->run[i].pool_enable = cmd->run[i].pool_enable;
		conv->run[i].pool_avg_param = cmd->run[i].pool_avg_param;
		conv->run[i].pool_size = cmd->run[i].pool_size;
		conv->run[i].pool_stride = cmd->run[i].pool_stride;
		conv->run[i].pool_pad = cmd->run[i].pool_pad;
		conv->run[i].actfunc = cmd->run[i].actfunc;
		conv->run[i].actfunc_param = cmd->run[i].actfunc_param;
		conv->run[i].rectifi_en = cmd->run[i].rectifi_en;
		conv->run[i].lrn = cmd->run[i].lrn;
	}

	cmb->size += cmd_size;

	vfree(cmd);
	return 0;
}

int dv_convert_command(struct device *dev, struct dmp_cmb *cmb,
		       dmp_dv_kcmd *cmd_info)
{
	int i;
	int ret = 0;
	dmp_dv_kcmdraw __user *user_cmds;
	dmp_dv_kcmdraw cmd;

	user_cmds = u64_to_user_ptr(cmd_info->cmd_pointer);

	for (i = 0; i < cmd_info->cmd_num; ++i) {
		// get size and version first;
		if (copy_from_user(&cmd, user_cmds, sizeof(cmd)))
			return -EFAULT;
		switch (cmd.version) {
		case 0:
			ret = dv_convert_conv_v0(dev, cmb, user_cmds, cmd.size);
			if (ret)
				return ret;
			break;
		default:
			pr_err(DRM_DEV_NAME ": Invalid command version.\n");
			return -EINVAL;
		}
		user_cmds = (dmp_dv_kcmdraw __user *)((uint8_t *)user_cmds +
						      cmd.size);
	}

	return ret;
}

void dv_run_command(struct dmp_cmb *cmb, void *bar_logical)
{
	uint32_t *cmd_buf;
	
	cmd_buf = (uint32_t *)((uint8_t *)cmb->logical + cmb->size);
	cmd_buf[0] = 0x0108f004; // Write one word to 0x108
	cmd_buf[1] = 0x00000001; // Set interrupt register
	cmb->size += sizeof(uint32_t) * 2;

	barrier();
	
	iowrite32(cmb->physical, REG_IO_ADDR(bar_logical, 0x0400));
	iowrite32((cmb->size + 7) / 8, REG_IO_ADDR(bar_logical, 0x0404));
	iowrite32(0x1, REG_IO_ADDR(bar_logical, 0x0408));
}
