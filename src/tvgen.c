/*
 *  DV700 kernel driver
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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/dma-buf.h>

#include "dmp-dv.h"
#include "../uapi/dmp_dv_cmdraw_v0.h"
#include "tvgen.h"

phys_addr_t tvgen_bar_physical[DRM_NUM_SUBDEV];

typedef struct
{
  int fd;
  struct dma_buf *dma_buf;
  dma_addr_t physical;
  u64 offs;
  size_t size;
} tvgen_buf;
static tvgen_buf buf_input;
static tvgen_buf buf_output;

struct list_init {
  u32 value;
  u32 devid;
  u32 offset;
  struct list_head list;
};
static struct list_head head_init;

static struct file* file_phi_ocp = NULL;
static struct file* file_memdump = NULL;

static struct file *file_open(const char *path)
{
    struct file *filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, O_WRONLY | O_CREAT | O_TRUNC | O_LARGEFILE, 0644);
    set_fs(oldfs);
    if (IS_ERR(filp))
    {
        err = PTR_ERR(filp);
        return NULL;
    }
    return filp;
}

static void file_close(struct file *file)
{
    filp_close(file, NULL);
}

static ssize_t file_write(struct file *file, const void* buf, ssize_t count)
{
    if(file && buf && count)
    {
        return kernel_write(file, buf, count, &file->f_pos);
    }
    return 0;
}

static void write_phi_ocp(const char* fmt, ...)
{
  char buff[32];
  va_list args;
  int count;

  va_start(args,fmt);
  count = vsprintf(buff,fmt,args);
  va_end(args);

  file_write(file_phi_ocp, buff, count);
}

static void write_memdump(const char* fmt, ...)
{
  char buff[48];
  va_list args;
  int count;

  va_start(args,fmt);
  count = vsprintf(buff,fmt,args);
  va_end(args);

  file_write(file_memdump, buff, count);
}

void tvgen_init(void)
{
  INIT_LIST_HEAD(&head_init);
}

void tvgen_release(void)
{
  // release init sequence
  struct list_init *ltmp;
  struct list_init *lobj;
  list_for_each_entry_safe(lobj, ltmp, &head_init, list)
    {
      list_del(&lobj->list);
      kfree(lobj);
    }
}

void tvgen_add_init(u32 value, u32 devid, u32 offset)
{
  struct list_init *data;
  data = kmalloc(sizeof(struct list_init), GFP_KERNEL);
  data->value = value;
  data->devid = devid;
  data->offset = offset;
  list_add(&data->list, &head_init);
}

void tvgen_start(const char* path)
{
    char* filename = (char*)kmalloc(PATH_MAX, GFP_KERNEL);
    
    pr_debug(DRM_DEV_NAME": tvgen_start\n");

    buf_input.dma_buf = 0;
    buf_input.size = 0;

    if(file_phi_ocp) { file_close(file_phi_ocp); file_phi_ocp = NULL; }
    if(file_memdump) { file_close(file_memdump); file_memdump = NULL; }

    strcpy(filename, path? path : TVGEN_DEFAULT_FILE_PATH);
    strcat(filename, TVGEN_PHI_OCP_FILENAME);
    file_phi_ocp = file_open(filename);
    if(file_phi_ocp==NULL)
    {
        pr_err(DRM_DEV_NAME": file open error [%s]\n", filename);
    }
    pr_debug(DRM_DEV_NAME": phi_ocp [%s]\n", filename);

    strcpy(filename, path? path : TVGEN_DEFAULT_FILE_PATH);
    strcat(filename, TVGEN_MEMDUMP_FILENAME);
    file_memdump = file_open(filename);
    if(file_memdump==NULL)
    {
        pr_err(DRM_DEV_NAME": file open error [%s]\n", filename);
    }
    pr_debug(DRM_DEV_NAME": memdump [%s]\n", filename);

    kfree(filename);

    // write init sequence
    {
      struct list_head *list;
      struct list_init *data;
      list_for_each(list, &head_init)
	{
	  data = list_entry(list, struct list_init, list);
	  tvgen_phi_ocp_i(data->value, data->devid, data->offset);
	}
    }
}

void tvgen_end(void)
{
    if(file_phi_ocp) { file_close(file_phi_ocp); file_phi_ocp = NULL; }
    if(file_memdump) { file_close(file_memdump); file_memdump = NULL; }

    pr_debug(DRM_DEV_NAME": tvgen_close_ocp\n");
}

// write to phi_ocp
void tvgen_phi_ocp_i(u32 value, u32 devid, u32 offset)
{
  write_phi_ocp("2_00_%08llx_f_%08x\n",
		tvgen_bar_physical[devid] + offset, value);
}
void tvgen_phi_ocp_a(u32 value, u32 addr)
{
  write_phi_ocp("2_00_%08llx_f_%08x\n", addr, value);
}

void tvgen_mem_write(const char* name, void *logical, dma_addr_t physical, ssize_t size)
{
  u32* mem = (u32*)logical;

  if(name) write_memdump("[%s]\n", name);
  
  while(size>0)
    {
      write_memdump("@%07llx %08x%08x%08x%08x\n", physical>>4, mem[3], mem[2], mem[1], mem[0]);
      mem += 4;
      physical += 16;
      size -= 16;
    }
}

void tvgen_mem_weight(const struct dmp_dv_kbuf* buf, dma_addr_t physical, u64 size)
{
  struct dma_buf *dma_buf;
  u8 *logical;

  if(!size) return;
  
  dma_buf = dma_buf_get(buf->fd);
  dma_buf_begin_cpu_access(dma_buf, DMA_BIDIRECTIONAL);
  logical = dma_buf_kmap(dma_buf, 0);

  tvgen_mem_write("weight", logical + buf->offs, physical, size);

  dma_buf_kunmap(dma_buf, 0, logical);
  dma_buf_end_cpu_access(dma_buf, DMA_BIDIRECTIONAL);
  dma_buf_put(dma_buf);
}

void tvgen_mem_input(const struct dmp_dv_kbuf* buf, dma_addr_t physical, u64 size)
{
  if(buf && !buf_input.size)
    {
      buf_input.dma_buf = dma_buf_get(buf->fd);
      buf_input.offs = buf->offs;
      buf_input.physical = physical;
      buf_input.size = size;
    }
  else if(!buf && buf_input.dma_buf)
    {
      u8 *logical;
      
      dma_buf_begin_cpu_access(buf_input.dma_buf, DMA_BIDIRECTIONAL);
      logical = dma_buf_kmap(buf_input.dma_buf, 0);

      tvgen_mem_write("input", logical + buf_input.offs, buf_input.physical, buf_input.size);

      dma_buf_kunmap(buf_input.dma_buf, 0, logical);
      dma_buf_end_cpu_access(buf_input.dma_buf, DMA_BIDIRECTIONAL);
      dma_buf_put(buf_input.dma_buf);

      buf_input.dma_buf = 0;
    }
}

void tvgen_mem_output(const struct dmp_dv_kbuf* buf, dma_addr_t physical, u64 size)
{
  // always update to get the last output
}
