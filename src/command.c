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
#include "../uapi/dmp_dv_cmdraw_v1.h"
#include "../uapi/dimensions.h"

#define MAX_NUM_RUNS 32
#define CMB_SIZE (16 * PAGE_SIZE)

#define REG_IO_ADDR(BAR, OF) ((void __iomem *)(BAR) + OF)

/// Required alignment masks for buffers (2**ALIGN - 1)
#define ALIGN_MASK_INPUT 1
#define ALIGN_MASK_OUTPUT 1
#define ALIGN_MASK_ELTWISE 1
#define ALIGN_MASK_WEIGHTS 15
#define ALIGN_MASK_TABLE 15

// struct dmp_cmb_list_entry {
// 	dma_addr_t physical;
// 	void *logical;
// 	size_t size;			// size of commands
// 	size_t capacity;		// allocated DMA memory size
// 	struct list_head list_node;
// };

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

struct program_cmd_data {
	dma_addr_t PA;
	void *VA;
	void *CA;
	uint32_t n;
	uint32_t *pptr;
	void *p1st;
	dma_addr_t PM;
	uint32_t *VM;
	uint32_t *CM;
	uint32_t *qptr;
	uint32_t ta;
};

struct dmp_cmb {
	// struct list_head cmb_list;
	DECLARE_HASHTABLE(dmabuf_hash, 6);
	uint32_t cmd_capacity;
	struct program_cmd_data pg_cmd_data;
};

// static struct dmp_cmb_list_entry *dv_cmb_allocate(struct device *dev)
// {
// 	struct dmp_cmb_list_entry *cmb;
// 	size_t alloc_size = CMB_SIZE;

// 	cmb = kmalloc(sizeof(*cmb), GFP_KERNEL);
// 	if (!cmb)
// 		return NULL;

// 	do {
// 		cmb->logical = dma_alloc_coherent(dev, alloc_size,
// 						  &cmb->physical, GFP_KERNEL);
// 		alloc_size >>= 1;
// 	} while (!cmb->logical && alloc_size >= PAGE_SIZE);
// 	if (!cmb->logical) {
// 		kfree(cmb);
// 		return NULL;
// 	}
// 	cmb->size = 0;
// 	cmb->capacity = alloc_size << 1;

// 	return cmb;
// }
static inline void IniStat(void) {
	global_vars.bufA = 0;
	global_vars.cofst = 0;
}
void program_driver(const struct dmp_dv_kcmdraw_conv_v0 *conf,
		    volatile uint32_t *conv_base, struct program_cmd_data *cptr,
		    struct st_global_vars *p_global_vars,
		    int (*f_printk)(const char *fmt, ...));
#ifndef cSZ
#define cSZ 1024*1024*2
#endif
#define mSZ 1024*1024*1

struct dmp_cmb *dv_cmb_init(struct device *dev)
{
	struct dmp_cmb *cmb;

	cmb = kzalloc(sizeof(*cmb), GFP_KERNEL);
	if (!cmb)
		return NULL;

	hash_init(cmb->dmabuf_hash); //for get_dma_addr

	// INIT_LIST_HEAD(&cmb->cmb_list);

	cmb->pg_cmd_data.VA = dma_alloc_coherent(dev, cSZ, &cmb->pg_cmd_data.PA, GFP_KERNEL);
	cmb->pg_cmd_data.VM = dma_alloc_coherent(dev, mSZ, &cmb->pg_cmd_data.PM, GFP_KERNEL);
	if (!cmb->pg_cmd_data.VA)
		return NULL;

	cmb->pg_cmd_data.ta = 0;
	cmb->pg_cmd_data.n = 0;
	cmb->pg_cmd_data.p1st = 0;
	cmb->pg_cmd_data.pptr = 0;
	cmb->pg_cmd_data.qptr = 0;
	cmb->pg_cmd_data.CA = cmb->pg_cmd_data.VA;
	cmb->pg_cmd_data.CM = cmb->pg_cmd_data.VM;
	cmb->cmd_capacity = cSZ;

	return cmb;
}

void dv_cmb_finalize(struct device *dev, struct dmp_cmb *cmb)
{
	int bkt;
	// struct dmp_cmb_list_entry *ltmp;
	// struct dmp_cmb_list_entry *lobj;
	struct hlist_node *htmp;
	struct dmp_dmabuf_hash_entry *hobj;

	// list_for_each_entry_safe(lobj, ltmp, &cmb->cmb_list, list_node) {
	// 	dma_free_coherent(dev, lobj->capacity, lobj->logical, lobj->physical);
	// 	list_del(&lobj->list_node);
	// 	kfree(lobj);
	// }

	hash_for_each_safe(cmb->dmabuf_hash, bkt, htmp, hobj, hash_node) {
		dma_buf_unmap_attachment(hobj->attachment, hobj->sgt, hobj->dir);
		dma_buf_detach(hobj->dma_buf, hobj->attachment);
		dma_buf_put(hobj->dma_buf);
		hash_del(&hobj->hash_node);
		kfree(hobj);
	}

	// Reset program_driver state
	IniStat ();
	if(cmb->pg_cmd_data.VA)
		dma_free_coherent(dev, cmb->cmd_capacity, cmb->pg_cmd_data.VA, cmb->pg_cmd_data.PA);
	if(cmb->pg_cmd_data.VM)
		dma_free_coherent(dev, mSZ, cmb->pg_cmd_data.VM, cmb->pg_cmd_data.PM);

	kfree(cmb);
}

// NOTE: addr is uint32_t since the HW can only use 32-bit address.
static int get_dma_addr(struct device *dev, struct dmp_cmb *cmb,
			struct dmp_dv_kbuf *buf,
			uint32_t *addr, uint32_t *buf_size, int align_mask)
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

	if (buf->offs & align_mask) {  // check required alignment
		pr_warn(DRM_DEV_NAME ": got unaligned offset %llu\n",
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
#if IS_ENABLED(CONFIG_ARCH_DMA_ADDR_T_64BIT)
		if (obj->dma_addr + obj->buf_size > 4294967296ull) {
			pr_warn(DRM_DEV_NAME
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

static uint32_t get_c8m8_size(uint32_t cm)
{
	return (cm >> 3) + ((cm & 7) ? 1 : 0);
}

static uint32_t dv_calculate_cmd_size(const struct dmp_dv_kcmdraw_conv_v0 *cmd, uint32_t use_ta)
{
	uint32_t req_size = 0, per_batch_size;
	uint32_t batch, c, m, topo, i, c_m_loop_count;
	const uint32_t base_size = 224; // Base command size, this is approximated value
	const uint32_t c_m_loop_size = 32; // Each additional C/M loop seems to use 32 bytes
	const uint32_t per_batch_reduction = 32; // Each batch besides the first one reduce this many bytes
	const uint32_t ta_size = 2048; // u8->f16 table size, (= 256 * 8)

	batch = cmd->input_circular_offset & 0x7fff;
	batch = batch ? batch : 1;
	c = cmd->c;
	for (topo = cmd->topo, i = 0; topo; topo >>= 1, ++i) {
		m = cmd->run[i].m;
		c_m_loop_count = get_c8m8_size(c) * get_c8m8_size(m);
		per_batch_size = (base_size + c_m_loop_count * c_m_loop_size);
		req_size += per_batch_size + (per_batch_size - per_batch_reduction) * (batch - 1);
		// next c = m of previous run, unless it is an output run
		c = (topo & 1) ? cmd->c : m;
	}

	if (use_ta) {
		req_size += ta_size;
	}

	return req_size;
}

static int dv_convert_conv_v0(struct device *dev, struct dmp_cmb *cmb,
			      struct dmp_dv_kcmdraw __user *user_cmd,
			      size_t size)
{
	struct dmp_dv_kcmdraw_conv_v0 *cmd;
	unsigned int runs, i;
	uint32_t input_base_addr, input_buf_size;
	uint32_t output_base_addr, output_buf_size;
	uint32_t eltwise_base_addr, eltwise_buf_size;
	uint32_t weight_base_addr, weight_buf_size;
	struct conv_data_size conv_size;
	uint32_t weight_size = 0;
	uint32_t tot_weight_size = 0, current_cmd_size = 0, required_cmd_size = 0;
	int ret, valid_multi_run = 1;
	uint16_t dil_x, dil_y;

	// there should be at least one run
	if (size < sizeof(struct dmp_dv_kcmdraw_conv_v0) - 31 *
			sizeof(struct dmp_dv_kcmdraw_conv_v0_run)) {
		pr_warn(DRM_DEV_NAME ": Command size is too small: %zu < %zu\n",
			size, sizeof(struct dmp_dv_kcmdraw_conv_v0) - 31 *
			sizeof(struct dmp_dv_kcmdraw_conv_v0_run));
		return -EINVAL;
	}

	cmd = kmalloc(size, GFP_KERNEL);
	if (!cmd) {
		pr_warn(DRM_DEV_NAME ": kmalloc() failed for %zu bytes\n", size);
		return -ENOMEM;
	}
	if (copy_from_user(cmd, user_cmd, size)) {
		kfree(cmd);
		pr_warn(DRM_DEV_NAME ": copy_from_user() failed for %zu bytes\n",
			size);
		return -EFAULT;
	}

	runs = topo_num_runs(cmd->topo);
	if (size < sizeof(struct dmp_dv_kcmdraw_conv_v0) - (32 - runs) *
			sizeof(struct dmp_dv_kcmdraw_conv_v0_run)) {
		kfree(cmd);
		pr_warn(DRM_DEV_NAME ": Invalid runs=%u\n", runs);
		return -EINVAL;
	}
        // Patch dilation
        for (i = 0; i < runs; ++i) {
		if (!cmd->run[i].conv_enable)
			continue;
		dil_x = cmd->run[i].conv_dilation & 0xFF;
		dil_y = (cmd->run[i].conv_dilation >> 8) & 0xFF;
		dil_x = dil_x < 1 ? 1 : dil_x;
		dil_y = dil_y < 1 ? 1 : dil_y;
		cmd->run[i].conv_dilation = dil_x | (dil_y << 8);
		if ((dil_x > 1) || (dil_y > 1))
			valid_multi_run = 0;
	}

	init_conv_input_size_v0(cmd, &conv_size);
	ret = get_dma_addr(dev, cmb, &cmd->input_buf, &input_base_addr,
			   &input_buf_size, ALIGN_MASK_INPUT);
	if (ret) {
		kfree(cmd);
		pr_warn(DRM_DEV_NAME ": get_dma_addr() failed for input\n");
		return ret;
	}
	if (input_buf_size < conv_size.size) {
		kfree(cmd);
		pr_warn(DRM_DEV_NAME ": got input buffer size %u while %u was expected\n",
			input_buf_size, conv_size.size);
		return -EINVAL;
	}
	ret = get_dma_addr(dev, cmb, &cmd->output_buf, &output_base_addr,
			   &output_buf_size, ALIGN_MASK_OUTPUT);
	if (ret) {
		kfree(cmd);
		pr_warn(DRM_DEV_NAME ": get_dma_addr() failed for output\n");
		return ret;
	}
	ret = get_dma_addr(dev, cmb, &cmd->eltwise_buf, &eltwise_base_addr,
			   &eltwise_buf_size, ALIGN_MASK_ELTWISE);
	if (ret) {
		kfree(cmd);
		pr_warn(DRM_DEV_NAME ": get_dma_addr() failed for eltwise\n");
		return ret;
	}

	cmd->input_buf.rsvd = input_base_addr;
	cmd->output_buf.rsvd = output_base_addr;
	cmd->eltwise_buf.rsvd = eltwise_base_addr;
	for (i = 0; i < runs; ++i) {
		get_conv_output_size_v0(&cmd->run[i], &conv_size, &conv_size, &weight_size);
		ret = get_dma_addr(dev, cmb, &cmd->run[i].weight_buf,
				   &weight_base_addr, &weight_buf_size, ALIGN_MASK_WEIGHTS);
		if (ret) {
			kfree(cmd);
			pr_warn(DRM_DEV_NAME ": get_dma_addr() failed for weights\n");
			return ret;
		}
		if (weight_base_addr != 0xDEADBEEF &&
			weight_buf_size < weight_size) {
			kfree(cmd);
			pr_warn(DRM_DEV_NAME ": got weights buffer size %u while expected %u\n",
				weight_buf_size, weight_size);
			return -EINVAL;
		}
		tot_weight_size += weight_size;
		cmd->run[i].weight_buf.rsvd = weight_base_addr;
	}

	// Calculate required cmd size and re-allocate cmd buffer if remaining space is not enough
	current_cmd_size = cmb->pg_cmd_data.CA - cmb->pg_cmd_data.VA;
	required_cmd_size = dv_calculate_cmd_size(cmd, cmb->pg_cmd_data.ta);
	// If use quantized weight, include weight size into command size
	if (cmd->run[0].weight_fmt & 2)
		required_cmd_size += tot_weight_size;
	if (current_cmd_size + required_cmd_size > cmb->cmd_capacity) {
		dma_addr_t new_PA;
		void *new_VA;
		uint32_t new_capacity = cmb->cmd_capacity + cSZ;
		while (new_capacity < current_cmd_size + required_cmd_size)
			new_capacity += cSZ;

		new_VA = dma_alloc_coherent(dev, new_capacity, &new_PA, GFP_KERNEL);
		memcpy(new_VA, cmb->pg_cmd_data.VA, current_cmd_size);
		// Adjust existing pointers to new cmd buffer
		cmb->pg_cmd_data.CA = new_VA + current_cmd_size;
		if (cmb->pg_cmd_data.p1st)
			cmb->pg_cmd_data.p1st = new_VA + (cmb->pg_cmd_data.p1st - cmb->pg_cmd_data.VA);
		if (cmb->pg_cmd_data.pptr)
			cmb->pg_cmd_data.pptr = new_VA + ((void*)cmb->pg_cmd_data.pptr - cmb->pg_cmd_data.VA);
		if (cmb->pg_cmd_data.qptr)
			cmb->pg_cmd_data.qptr = new_VA + ((void*)cmb->pg_cmd_data.qptr - cmb->pg_cmd_data.VA);

		dma_free_coherent(dev, cmb->cmd_capacity, cmb->pg_cmd_data.VA, cmb->pg_cmd_data.PA);
		cmb->pg_cmd_data.PA = new_PA;
		cmb->pg_cmd_data.VA = new_VA;
		cmb->cmd_capacity = new_capacity;
	}

	program_driver(cmd, (volatile uint32_t*)0x80000000, &cmb->pg_cmd_data, &global_vars, printk);
	cmb->pg_cmd_data.n++;

	kfree(cmd);
	return 0;
}

static int dv_convert_conv_v1(struct device *dev, struct dmp_cmb *cmb,
			      struct dmp_dv_kcmdraw __user *user_cmd,
			      size_t size)
{
	struct dmp_dv_kcmdraw_conv_v1 *cmd;
	struct dmp_dv_kbuf table;
	uint32_t table_base_addr, table_buf_size;
	uint16_t to_bgr;
	int ret;

	cmd = (struct dmp_dv_kcmdraw_conv_v1 *)user_cmd;
	if (copy_from_user(&table, &cmd->u8tofp16_table, sizeof(table))) {
		pr_warn(DRM_DEV_NAME ": copy_from_user() failed for %zu bytes\n",
			sizeof(table));
		return -EFAULT;
	}
	if (copy_from_user(&to_bgr, &cmd->to_bgr, sizeof(to_bgr))) {
		pr_warn(DRM_DEV_NAME ": copy_from_user() failed for %zu bytes\n",
			sizeof(to_bgr));
		return -EFAULT;
	}

	ret = get_dma_addr(dev, cmb, &table, &table_base_addr, &table_buf_size, ALIGN_MASK_TABLE);
	if (ret) {
		pr_warn(DRM_DEV_NAME ": get_dma_addr() failed for table\n");
		return ret;
	}
	if (table_base_addr != 0xDEADBEEF && table_buf_size < 6 * 256) {
		pr_warn(DRM_DEV_NAME ": got table buffer size %u while %u was expected\n",
			table_buf_size, 6 * 256);
		return -EINVAL;
	}
	if (to_bgr) {
		table_base_addr |= 1;
	}
	cmb->pg_cmd_data.ta = table_base_addr;

	// call v0 version to handle remaining commands
	user_cmd = (struct dmp_dv_kcmdraw __user *)((uint8_t *)cmd + sizeof(table) + 8);
	size -= sizeof(table) + 8;
	return dv_convert_conv_v0(dev, cmb, user_cmd, size);
}

int dv_convert_conv_command(struct device *dev, struct dmp_cmb *cmb,
			    struct dmp_dv_kcmd *cmd_info)
{
	int i;
	int ret = 0;
	struct dmp_dv_kcmdraw __user *user_cmds;
	struct dmp_dv_kcmdraw cmd;
	user_cmds = u64_to_user_ptr(cmd_info->cmd_pointer);

	for (i = 0; i < cmd_info->cmd_num; ++i) {
		// get size and version first;
		if (copy_from_user(&cmd, user_cmds, sizeof(cmd))) {
			pr_warn(DRM_DEV_NAME ": Invalid CONV command pointer: copy_from_user() failed\n");
			return -EFAULT;
		}
		switch (cmd.version) {
		case 0:
			ret = dv_convert_conv_v0(dev, cmb, user_cmds, cmd.size);
			if (ret)
				return ret;
			break;
		case 1:
			ret = dv_convert_conv_v1(dev, cmb, user_cmds, cmd.size);
			if (ret)
				return ret;
			break;
		default:
			pr_warn(DRM_DEV_NAME ": Invalid CONV command version: %u\n",
				cmd.version);
			return -EINVAL;
		}
		user_cmds = (struct dmp_dv_kcmdraw __user *)
				((uint8_t*)user_cmds + cmd.size);
	}

	return ret;
}

#define regIO(a) *(volatile uint32_t *)((uint64_t)(a))
void dv_run_conv_command(struct dmp_cmb *cmb, void *bar_logical)
{
	dma_addr_t prev_addr = 0xDEADBEEF;
	size_t sz = 0;

	if (cmb->pg_cmd_data.n) { //ins wait for SW
		int r, scmd = (61 << 24) | ( 1 << 11);
		if (regIO(cmb->pg_cmd_data.CA - 4) == scmd)
			cmb->pg_cmd_data.CA = (void*)((uint64_t)cmb->pg_cmd_data.CA - 4);
		do{
			r = (uint64_t)cmb->pg_cmd_data.CA & 15;
			WARN(r&3, "CA");
			regIO(cmb->pg_cmd_data.CA) = r < 12 ? 0 : scmd;
			cmb->pg_cmd_data.CA = (void*)((uint64_t)cmb->pg_cmd_data.CA + 4);
		} while (r < 12);
	}
	if (cmb->pg_cmd_data.p1st) {
		if (cmb->pg_cmd_data.pptr) {
			int s = ((uint64_t)cmb->pg_cmd_data.CA - ((uint64_t)cmb->pg_cmd_data.pptr >> 4 << 4) - 16 - 1) >> 2;
			WARN(s > 0xfffff, "RetOver%x", s);
			if (cmb->pg_cmd_data.qptr)
				*cmb->pg_cmd_data.qptr = (0x58 << 24) | s;
			else
				*cmb->pg_cmd_data.pptr |= (7 << 29) | s;
			cmb->pg_cmd_data.pptr=0;
		}
		sz = cmb->pg_cmd_data.p1st - cmb->pg_cmd_data.VA;
	} else {
		sz = cmb->pg_cmd_data.CA - cmb->pg_cmd_data.VA;
	}
	prev_addr = cmb->pg_cmd_data.PA;

	barrier();

	if (cmb->pg_cmd_data.n) {
		cmb->pg_cmd_data.n = 0;
	}
	if (((global_vars.opt >> 27) & 3) > 2) {
		int WS = 2;
		if (cmb->pg_cmd_data.pptr || cmb->pg_cmd_data.CM == cmb->pg_cmd_data.VM) {
			int c = 12;
			if (!cmb->pg_cmd_data.pptr) {
				*cmb->pg_cmd_data.CM++ = (1 << 31) | 8;
				*cmb->pg_cmd_data.CM++ = cmb->pg_cmd_data.PA;
				c = 4;
				cmb->pg_cmd_data.pptr = cmb->pg_cmd_data.VA;
			}
			sz = cmb->pg_cmd_data.CA - (void*)cmb->pg_cmd_data.pptr;
			cmb->pg_cmd_data.pptr = 0;
			if (WS) {
				*cmb->pg_cmd_data.CM++ = (1 << 31) | c; //WA
				*cmb->pg_cmd_data.CM++ = sz - 1;
			} else {
				*cmb->pg_cmd_data.CM++ = (c << 22) | ((sz >> 2) - 1); //Wc
			}
		}
		iowrite32(cmb->pg_cmd_data.PM, REG_IO_ADDR(bar_logical, 0x820));
		iowrite32((cmb->pg_cmd_data.CM - cmb->pg_cmd_data.VM - 1) << WS, REG_IO_ADDR(bar_logical, 0x824));
	} else {
		iowrite32(prev_addr | 1, REG_IO_ADDR(bar_logical, 0x800)); //1forRST@riscv
		iowrite32(sz - 1, REG_IO_ADDR(bar_logical, 0x854)); //irq=kick
	}
	++global_vars.conv_kick_count;
}
