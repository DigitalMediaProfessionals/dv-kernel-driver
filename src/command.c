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
#include <linux/list.h>
#include <linux/hashtable.h>

#include "dmp-dv.h"
#include "../uapi/dmp_dv_cmdraw_v0.h"
#include "../uapi/dimensions.h"

#define MAX_NUM_RUNS 32
#define CMB_SIZE (16 * PAGE_SIZE)

#define REG_IO_ADDR(BAR, OF) ((void __iomem *)(BAR) + OF)

// TODO: get this from HW
int UNIFIED_BUFFER_SIZE = 640 * 1024;
int MAX_CONV_KERNEL_SIZE = 7;
int MAX_FC_VECTOR_SIZE = 16384;

struct dmp_cmb_list_entry {
	dma_addr_t physical;
	void *logical;
	size_t size;
	size_t capacity;
	struct list_head list_node;
};

struct dmp_dmabuf_hash_entry {
	int fd;
	enum dma_data_direction dir;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	unsigned int buf_size;
	struct hlist_node hash_node;
};

struct dmp_cmb {
	struct list_head cmb_list;
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
	uint16_t weight_fmt; // Weight format (0 = random access blocks, 1 = compact stream, 3 = 8-bit quantized stream)
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

static struct dmp_cmb_list_entry *dv_cmb_allocate(struct device *dev)
{
	struct dmp_cmb_list_entry *cmb;
	size_t alloc_size = CMB_SIZE;

	cmb = kmalloc(sizeof(*cmb), GFP_KERNEL);
	if (!cmb)
		return NULL;

	do {
		cmb->logical = dma_alloc_coherent(dev, alloc_size,
			&cmb->physical, GFP_KERNEL);
		alloc_size >>= 1;
	} while (!cmb->logical && alloc_size >= PAGE_SIZE);
	if (!cmb->logical) {
		kfree(cmb);
		return NULL;
	}
	cmb->size = 0;
	cmb->capacity = alloc_size << 1;

	return cmb;
}

struct dmp_cmb *dv_cmb_init(struct device *dev)
{
	struct dmp_cmb *cmb;
	struct dmp_cmb_list_entry *cmb_node;

	cmb = kzalloc(sizeof(*cmb), GFP_KERNEL);
	if (!cmb)
		return NULL;

	INIT_LIST_HEAD(&cmb->cmb_list);
	cmb_node = dv_cmb_allocate(dev);
	if (!cmb_node) {
		kfree(cmb);
		return NULL;
	}
	list_add(&cmb_node->list_node, &cmb->cmb_list);

	hash_init(cmb->dmabuf_hash);

	return cmb;
}

void dv_cmb_finalize(struct device *dev, struct dmp_cmb *cmb)
{
	int bkt;
	struct dmp_cmb_list_entry *ltmp;
	struct dmp_cmb_list_entry *lobj;
	struct hlist_node *htmp;
	struct dmp_dmabuf_hash_entry *hobj;

	list_for_each_entry_safe(lobj, ltmp, &cmb->cmb_list, list_node) {
		dma_free_coherent(dev, lobj->capacity, lobj->logical,
			lobj->physical);
		list_del(&lobj->list_node);
		kfree(lobj);
	}

	hash_for_each_safe(cmb->dmabuf_hash, bkt, htmp, hobj, hash_node) {
		dma_buf_unmap_attachment(hobj->attachment, hobj->sgt, hobj->dir);
		dma_buf_detach(hobj->dma_buf, hobj->attachment);
		dma_buf_put(hobj->dma_buf);
		hash_del(&hobj->hash_node);
		kfree(hobj);
	}
	kfree(cmb);
}

// addr is uint32_t since the HW can only use 32-bit address
static int get_dma_addr(struct device *dev, struct dmp_cmb *cmb,
			dmp_dv_kbuf *buf, uint32_t *addr, uint32_t *buf_size)
{
	struct dmp_dmabuf_hash_entry *obj;
	struct scatterlist *sg;
	int i, ret = 0;

	// Handle invalid fd
	if (buf->fd < 0) {
		*addr = 0xDEADBEEF;
		*buf_size = 0;
		return 0;
	}

	if (buf->offs & 1) {  // check required alignment
		pr_warning(DRM_DEV_NAME ": got unaligned offset %llu\n",
			(unsigned long long)buf->offs);
		return -EINVAL;
	}

	// Find if the fd is already in the hash
	hash_for_each_possible (cmb->dmabuf_hash, obj, hash_node, buf->fd) {
		if (obj->fd == buf->fd) {
			if (obj->buf_size < buf->offs)
				return -EINVAL;
			*addr = obj->dma_addr + (uint32_t)buf->offs;
			*buf_size = obj->buf_size - (uint32_t)buf->offs;
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
	obj->dir = DMA_BIDIRECTIONAL;
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
		obj->buf_size = sg_dma_len(sg);
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
		if (obj->dma_addr + obj->buf_size > 4294967296ull) {
			pr_warning(DRM_DEV_NAME
				": dma_addr=%llu buf_size=%llu lies in high memory\n",
				(unsigned long long)obj->dma_addr,
				(unsigned long long)obj->buf_size);
			ret = -1;
			break;
		}
#endif
		if (obj->buf_size < buf->offs) {
			ret = -EINVAL;
			break;
		}
		else {
		    *addr = obj->dma_addr + (uint32_t)buf->offs;
		    *buf_size = obj->buf_size - (uint32_t)buf->offs;
		}
	}

	hash_add(cmb->dmabuf_hash, &obj->hash_node, obj->fd);

	return ret;

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

static uint16_t get_conv_tiles_v0(dmp_dv_kcmdraw_conv_v0 *cmd)
{
	int w, h, c, m, px, py, c_blocks, t;
	int tw, ow, oh, os, ts_blk16, ts_blk128, ts_128, ts, uu;
	int pad[4], stride[2];

	if (topo_num_runs(cmd->topo) > 1)
		return 1;
	if (cmd->run[0].conv_enable & 2 || !cmd->run[0].conv_enable)
		return 1;

	w = cmd->w;
	h = cmd->h;
	c = cmd->c;
	m = cmd->run[0].m;
	px = cmd->run[0].p & 0xFF;
	py = (cmd->run[0].p >> 8) & 0xFF;
	pad[0] = cmd->run[0].conv_pad & 0xFF;
	pad[1] = (cmd->run[0].conv_pad >> 8) & 0xFF;
	pad[2] = (cmd->run[0].conv_pad >> 16) & 0xFF;
	pad[3] = (cmd->run[0].conv_pad >> 24) & 0xFF;
	stride[0] = cmd->run[0].conv_stride & 0xFF;
	stride[1] = (cmd->run[0].conv_stride >> 8) & 0xFF;
	c_blocks = (c >> 3) + (c & 7 ? 1 : 0);

	t = 0;
	for (; t < w;) {
		++t;
		tw = (w / t + (w % t ? 1 : 0)) + px - 1; // width of tile
		ow = get_conv_out_width(tw, px, pad[0], pad[1], stride[0]);
		oh = get_conv_out_width(h, py, pad[2], pad[3], stride[1]);
		os = ow * oh * min(8, m); // output buffer size
		ts_blk16 = tw * h * min(8, c); // tile size for a segment
		ts_blk128 = (ts_blk16 >> 3) + (ts_blk16 & 7 ? 1 : 0);
		ts_blk128 += (2 - ts_blk128) & 15;
		ts_128 = ts_blk128 * c_blocks;
		ts_128 += (0 - ts_128) & 15;
		ts = ts_128 << 3; // input tile size in UBUF (in float16)
		uu = ts + os; // unified buffer utilization
		if (uu * 2 <= UNIFIED_BUFFER_SIZE)
			return t;
	}

	return 0;
}

/**************************************
 *
 * Command buffer size verification
 *
 **************************************/
static int dv_convert_conv_v0(struct device *dev, struct dmp_cmb *cmb,
			      dmp_dv_kcmdraw __user *user_cmd, size_t size)
{
	struct dmp_cmb_list_entry *cmb_node;
	dmp_dv_kcmdraw_conv_v0 *cmd;
	struct conv_configuration *conv;
	size_t cmd_size, conv_len;
	uint32_t *cmd_buf;
	unsigned int runs, i;
	uint32_t input_base_addr, input_buf_size;
	uint32_t output_base_addr, output_buf_size;
	uint32_t eltwise_base_addr, eltwise_buf_size;
	uint32_t weight_base_addr, weight_buf_size;
	struct conv_data_size conv_size;
	uint32_t weight_size = 0, total_output_size = 0;
	int ret;

	// there should be at least one run
	if (size < sizeof(dmp_dv_kcmdraw_conv_v0) - 31 *
	    sizeof(dmp_dv_kcmdraw_conv_v0_run))
		return -EINVAL;

	cmd = kmalloc(size, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;
	if (copy_from_user(cmd, user_cmd, size)) {
		kfree(cmd);
		return -EFAULT;
	}

	runs = topo_num_runs(cmd->topo);
	if (size < sizeof(dmp_dv_kcmdraw_conv_v0) - (32 - runs) *
	    sizeof(dmp_dv_kcmdraw_conv_v0_run)) {
		kfree(cmd);
		return -EINVAL;
	}

	init_conv_input_size_v0(cmd, &conv_size);
	ret = get_dma_addr(dev, cmb, &cmd->input_buf, &input_base_addr,
			   &input_buf_size);
	if (ret) {
		kfree(cmd);
		return ret;
	}
	if (input_buf_size < conv_size.size) {
		kfree(cmd);
		return -EINVAL;
	}
	ret = get_dma_addr(dev, cmb, &cmd->output_buf, &output_base_addr,
			   &output_buf_size);
	if (ret) {
		kfree(cmd);
		return ret;
	}
	ret = get_dma_addr(dev, cmb, &cmd->eltwise_buf, &eltwise_base_addr,
			   &eltwise_buf_size);
	if (ret) {
		kfree(cmd);
		return ret;
	}

	cmb_node = list_first_entry(&cmb->cmb_list, struct dmp_cmb_list_entry,
				    list_node);
	cmd_size = conf_size(cmd->topo) + sizeof(uint32_t) * 5;
	conv_len = (conf_size(cmd->topo) + 3) / 4;
	// include size of jump or interrupt commands
	if (cmb_node->size + cmd_size + 8 > cmb_node->capacity) {
		cmb_node = dv_cmb_allocate(dev);
		if (!cmb_node) {
			kfree(cmd);
			return -ENOMEM;
		}
		list_add(&cmb_node->list_node, &cmb->cmb_list);
	}
	cmd_buf = (uint32_t *)((uint8_t *)cmb_node->logical + cmb_node->size);
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
	conv->input.tiles = get_conv_tiles_v0(cmd);

	conv->output.output_base_addr = output_base_addr;
	conv->output.eltwise_base_addr = eltwise_base_addr;
	conv->output.output_mode = cmd->output_mode;

	for (i = 0; i < runs; ++i) {
		// Check kernel size for validness
		int px = cmd->run[i].p & 0xFF;
		int py = (cmd->run[i].p >> 8) & 0xFF;
		if (!py)
			py = px;
		if ((px < 1) || (py < 1) || (px > MAX_CONV_KERNEL_SIZE) ||
		    (py > MAX_CONV_KERNEL_SIZE)) {
			kfree(cmd);
			return -EINVAL;
		}
		get_conv_output_size_v0(&cmd->run[i], &conv_size, &conv_size,
					&weight_size);
		if ((cmd->topo >> i) & 1) {
			total_output_size += conv_size.size;
			init_conv_input_size_v0(cmd, &conv_size);
		}
		ret = get_dma_addr(dev, cmb, &cmd->run[i].weight_buf,
		                   &weight_base_addr, &weight_buf_size);
		if (ret) {
			kfree(cmd);
			return ret;
		}
		if (weight_base_addr != 0xDEADBEEF &&
		    weight_buf_size < weight_size) {
			kfree(cmd);
			return -EINVAL;
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
	if (output_buf_size < total_output_size ||
	    (eltwise_base_addr != 0xDEADBEEF &&
	     eltwise_buf_size < total_output_size)) {
		kfree(cmd);
		return -EINVAL;
	}

	cmb_node->size += cmd_size;

	kfree(cmd);
	return 0;
}

int dv_convert_conv_command(struct device *dev, struct dmp_cmb *cmb,
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

void dv_run_conv_command(struct dmp_cmb *cmb, void *bar_logical)
{
	struct dmp_cmb_list_entry *cmb_node;
	uint32_t *cmd_buf;
	dma_addr_t prev_addr = 0xDEADBEEF;
	size_t prev_size = 0;

	list_for_each_entry(cmb_node, &cmb->cmb_list, list_node) {
		cmd_buf = (uint32_t *)((uint8_t *)cmb_node->logical +
			  cmb_node->size);
		if (prev_size == 0) {
			cmd_buf[0] = 0x0108f004; // Write one word to 0x108
			cmd_buf[1] = 0x00000001; // Set interrupt register
		} else {
			cmd_buf[0] = 0x1 | prev_size; // Jump length
			cmd_buf[1] = prev_addr; // Jump address
		}
		prev_addr = cmb_node->physical;
		prev_size = cmb_node->size + sizeof(uint32_t) * 2;
		if (prev_size % 8) {
			cmd_buf[2] = 0;
			prev_size += sizeof(uint32_t);
		}
	}

	barrier();

	iowrite32(prev_addr, REG_IO_ADDR(bar_logical, 0x0400));
	iowrite32(prev_size / 8, REG_IO_ADDR(bar_logical, 0x0404));
	iowrite32(0x1, REG_IO_ADDR(bar_logical, 0x0408));
}

static int dv_convert_fc_v0(struct device *dev, struct dmp_cmb *cmb,
			      dmp_dv_kcmdraw __user *user_cmd, size_t size)
{
	struct dmp_cmb_list_entry *cmb_node;
	dmp_dv_kcmdraw_fc_v0 cmd;
	size_t cmd_size;
	uint32_t *cmd_buf;
	uint32_t input_base_addr, input_buf_size;
	uint32_t output_base_addr, output_buf_size;
	uint32_t weight_base_addr, weight_buf_size, weight_addr, bias_addr;
	uint32_t weight_size = 0;
	int ret;

	if (size < sizeof(dmp_dv_kcmdraw_fc_v0))
		return -EINVAL;

	if (copy_from_user(&cmd, user_cmd, size))
		return -EFAULT;

	ret = get_dma_addr(dev, cmb, &cmd.input_buf, &input_base_addr,
			   &input_buf_size);
	if (ret)
		return ret;
	if (input_buf_size < cmd.input_size * 2)
		return -EINVAL;
	ret = get_dma_addr(dev, cmb, &cmd.output_buf, &output_base_addr,
			   &output_buf_size);
	if (ret)
		return ret;
	if (output_buf_size < cmd.output_size * 2)
		return -EINVAL;
	ret = get_dma_addr(dev, cmb, &cmd.weight_buf, &weight_base_addr,
			   &weight_buf_size);
	if (ret)
		return ret;
	if (cmd.weight_fmt == 0) {
		weight_size = (uint32_t)cmd.input_size * cmd.output_size * 2;
		weight_addr = weight_base_addr;
		bias_addr = weight_base_addr + weight_size;
	} else if (cmd.weight_fmt == 1) {
		weight_size = (uint32_t)cmd.input_size * cmd.output_size + 512;
		weight_addr = weight_base_addr + 512;
		bias_addr = weight_base_addr + weight_size;
	} else
		return -EINVAL;
	weight_size += cmd.output_size * 2;
	if (weight_buf_size < weight_size)
		return -EINVAL;

	cmb_node = list_first_entry(&cmb->cmb_list, struct dmp_cmb_list_entry,
				    list_node);
	cmd_size = sizeof(uint32_t) * (16 + (cmd.weight_fmt ? 257 : 0));
	// include size of jump or interrupt commands
	if (cmb_node->size + cmd_size + 8 > cmb_node->capacity) {
		cmb_node = dv_cmb_allocate(dev);
		if (!cmb_node)
			return -ENOMEM;
		list_add(&cmb_node->list_node, &cmb->cmb_list);
	}
	cmd_buf = (uint32_t *)((uint8_t *)cmb_node->logical + cmb_node->size);

	cmd_buf[0] = 0x0011f00e; // Write 2 words to 0x11..0x12
	cmd_buf[1] = 0x00000220 | (cmd.weight_fmt ? 0x3 : 0x2); // format
	cmd_buf[2] = ((cmd.actfunc & 0x7) << 4); // mode
	cmd_buf[3] = 0x0014f026; // Write 5 words to 0x14..0x18
	cmd_buf[4] = cmd.input_size; // input size
	cmd_buf[5] = cmd.output_size; // output size
	cmd_buf[6] = output_base_addr; // output addr0
	cmd_buf[7] = cmd.actfunc_param; // leaky param
	cmd_buf[8] = bias_addr; // bias addr
	cmd_buf[9] = 0x001df004; // Write 1 word to 0x1d
	cmd_buf[10] = input_base_addr; // input addr0
	cmd_buf[11] = 0x001ff00e; // Write 2 words to 0x1f..0x20
	cmd_buf[12] = cmd.input_size * (cmd.weight_fmt ? 1 : 2); // stride
	cmd_buf[13] = weight_addr; // weight addr0
	cmd_buf[14 + (cmd.weight_fmt ? 257 : 0)] = 0x0010f004; // Write 1 word to 0x10
	cmd_buf[15 + (cmd.weight_fmt ? 257 : 0)] = 0x2;

	if (cmd.weight_fmt) {
		struct dma_buf *dma_buf;
		__u8 *quant_base;
		__u16 *quant;
		unsigned int i, n;
		unsigned long page_num;
		unsigned int page_offs;
		unsigned int quants_left;

		dma_buf = dma_buf_get(cmd.weight_buf.fd);
		dma_buf_begin_cpu_access(dma_buf, DMA_BIDIRECTIONAL);
		page_num = cmd.weight_buf.offs >> PAGE_SHIFT;
		page_offs = cmd.weight_buf.offs & (PAGE_SIZE - 1);
		quants_left = (PAGE_SIZE - page_offs) >> 1;

		quant_base = dma_buf_kmap(dma_buf, page_num);
		if (!quant_base) {
			pr_err(DRM_DEV_NAME ": dma_buf_kmap() failed\n");
			dma_buf_end_cpu_access(dma_buf, DMA_BIDIRECTIONAL);
			dma_buf_put(dma_buf);
			return -ENOMEM;
		}
		quant = (__u16*)(quant_base + page_offs);
		cmd_buf[14] = 0x0013f7fc;  // write 256 words to 0x13
		n = quants_left < 256 ? quants_left : 256;
		for (i = 0; i < n; i++) {
			cmd_buf[15 + i] = (0x21 << 24) | (i << 16) | quant[i];
		}
		dma_buf_kunmap(dma_buf, page_num, quant_base);
		if (n < 256) {  // write remaining part from second page
			quant_base = dma_buf_kmap(dma_buf, ++page_num);
			if (!quant_base) {
				pr_err(DRM_DEV_NAME ": dma_buf_kmap() failed\n");
				dma_buf_end_cpu_access(dma_buf, DMA_BIDIRECTIONAL);
				dma_buf_put(dma_buf);
				return -ENOMEM;
			}
			quant = ((__u16*)quant_base) - n;
			for (; i < 256; i++) {
				cmd_buf[15 + i] = (0x21 << 24) | (i << 16) | quant[i];
			}
			dma_buf_kunmap(dma_buf, page_num, quant_base);
		}

		dma_buf_end_cpu_access(dma_buf, DMA_BIDIRECTIONAL);
		dma_buf_put(dma_buf);
	}

	cmb_node->size += cmd_size;

	return 0;
}

int dv_convert_fc_command(struct device *dev, struct dmp_cmb *cmb,
			  struct dmp_dv_kcmd_impl *cmd_info)
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
			ret = dv_convert_fc_v0(dev, cmb, user_cmds, cmd.size);
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

void dv_run_fc_command(struct dmp_cmb *cmb, void *bar_logical)
{
	struct dmp_cmb_list_entry *cmb_node;
	uint32_t *cmd_buf;
	dma_addr_t prev_addr = 0xDEADBEEF;
	size_t prev_size = 0;

	list_for_each_entry(cmb_node, &cmb->cmb_list, list_node) {
		cmd_buf = (uint32_t *)((uint8_t *)cmb_node->logical +
			  cmb_node->size);
		if (prev_size == 0) {
			cmd_buf[0] = 0x0008f004; // Write one word to 0x8
			cmd_buf[1] = 0x00000001; // Set interrupt register
		} else {
			cmd_buf[0] = 0x1 | prev_size; // Jump length
			cmd_buf[1] = prev_addr; // Jump address
		}
		prev_addr = cmb_node->physical;
		prev_size = cmb_node->size + sizeof(uint32_t) * 2;
		if (prev_size % 8) {
			cmd_buf[2] = 0;
			prev_size += sizeof(uint32_t);
		}
	}

	barrier();

	iowrite32(prev_size / 8, REG_IO_ADDR(bar_logical, 0x0));
	iowrite32(prev_addr, REG_IO_ADDR(bar_logical, 0x4));
	iowrite32(0x1, REG_IO_ADDR(bar_logical, 0x8));
}
