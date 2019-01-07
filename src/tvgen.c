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

struct list_buf {
  struct dma_buf *dma_buf;
  dma_addr_t physical;
  u64 offs;
  size_t size;
  struct list_head list;
};
static struct list_head list_buf_input;
static struct list_head list_buf_output;
static bool is_buf_empty_input;
static struct list_buf buf_output;
static int buf_count_input;
static int buf_count_output;

struct list_init {
  u32 value;
  u32 devid;
  u32 offset;
  struct list_head list;
};
static struct list_head list_init_head;

static char file_dir[PATH_MAX];

struct file* file_phi_ocp = NULL;
struct file* file_memdump = NULL;

static struct file *file_open(const char *file_name, int id)
{
  struct file *filp = NULL;
  mm_segment_t oldfs;

  char* path = (char*)kmalloc(PATH_MAX, GFP_KERNEL);
  if(id>=0)
    sprintf(path, "%s/%s%d.txt", file_dir, file_name, id);
  else
    sprintf(path, "%s/%s.txt", file_dir, file_name);

  oldfs = get_fs();
  set_fs(get_ds());
  filp = filp_open(path, O_WRONLY | O_CREAT | O_TRUNC | O_LARGEFILE, 0644);
  set_fs(oldfs);

  kfree(path);
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
      //return __kernel_write(file, buf, count, &file->f_pos);
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

static void write_mem(struct file* file, const char* fmt, ...)
{
  char buff[48];
  va_list args;
  int count;

  va_start(args,fmt);
  count = vsprintf(buff,fmt,args);
  va_end(args);

  file_write(file, buff, count);
}

void tvgen_init(void)
{
  INIT_LIST_HEAD(&list_init_head);
  INIT_LIST_HEAD(&list_buf_input);
  INIT_LIST_HEAD(&list_buf_output);
}

void tvgen_release(void)
{
  // release init sequence
  struct list_init *ltmp;
  struct list_init *lobj;
  list_for_each_entry_safe(lobj, ltmp, &list_init_head, list)
    {
      list_del(&lobj->list);
      kfree(lobj);
    }
}

void tvgen_set_physical(int idx, phys_addr_t addr)
{
  tvgen_bar_physical[idx % DRM_NUM_SUBDEV] = addr;
}

void tvgen_add_init(u32 value, u32 devid, u32 offset)
{
  struct list_init *data;
  data = kmalloc(sizeof(struct list_init), GFP_KERNEL);
  data->value = value;
  data->devid = devid;
  data->offset = offset;
  list_add(&data->list, &list_init_head);
}

void tvgen_start(const char* path)
{
  strcpy(file_dir, path? path : TVGEN_DEFAULT_FILE_PATH);

  if(file_phi_ocp) { file_close(file_phi_ocp); file_phi_ocp = NULL; }
  if(file_memdump) { file_close(file_memdump); file_memdump = NULL; }

  pr_debug("tvgen_start\n");
  
  file_phi_ocp = file_open(TVGEN_PHI_OCP_FILENAME, -1);
  if(file_phi_ocp==NULL)
    {
      pr_err(DRM_DEV_NAME": file open error [%s%s]\n", path, TVGEN_PHI_OCP_FILENAME);
    }

  file_memdump = file_open(TVGEN_MEMDUMP_FILENAME, -1);
  if(file_memdump==NULL)
    {
      pr_err(DRM_DEV_NAME": file open error [%s%s]\n", path, TVGEN_MEMDUMP_FILENAME);
    }

  // write init sequence
  {
    struct list_head *list;
    struct list_init *data;
    list_for_each(list, &list_init_head)
      {
	data = list_entry(list, struct list_init, list);
	tvgen_phi_ocp_i(data->value, data->devid, data->offset);
      }
  }

  buf_count_input = 0;
  buf_count_output = 0;
  buf_output.dma_buf = 0;
  tvgen_set_buffer();
}

void tvgen_end(void)
{
  if(file_phi_ocp) { file_close(file_phi_ocp); file_phi_ocp = NULL; }
  if(file_memdump) { file_close(file_memdump); file_memdump = NULL; }
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

void tvgen_mem_write(struct file* file, void *logical, dma_addr_t physical, ssize_t size, char* comment)
{
  u32* mem = (u32*)logical;

  if(comment) write_mem(file,"[%s]\n", comment);
  
  while(size>0)
    {
      write_mem(file,"@%07llx %08x%08x%08x%08x\n", physical>>4, mem[3], mem[2], mem[1], mem[0]);
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

  tvgen_mem_write(file_memdump, logical + buf->offs, physical, size, "weight");

  dma_buf_kunmap(dma_buf, 0, logical);
  dma_buf_end_cpu_access(dma_buf, DMA_BIDIRECTIONAL);
  dma_buf_put(dma_buf);
}

void tvgen_mem_input(const struct dmp_dv_kbuf* buf, dma_addr_t physical, u64 size)
{
  struct list_buf *input;

  if(buf && is_buf_empty_input)
    {
      //pr_debug(DRM_DEV_NAME": set mem_input\n");

      input = kmalloc(sizeof(struct list_buf), GFP_KERNEL);
      input->dma_buf = dma_buf_get(buf->fd);
      input->offs = buf->offs;
      input->physical = physical;
      input->size = size;

      list_add(&input->list, &list_buf_input);
      is_buf_empty_input = false;
    }
  else if(!buf && !list_empty(&list_buf_input))
    {
      u8 *logical;
      struct file* file;
      u64 index = buf_count_input++;
      
      //pr_debug(DRM_DEV_NAME": put mem_input %lld\n", index);

      // get entry
      input = list_last_entry(&list_buf_input, struct list_buf, list);
      list_del(&input->list);

      dma_buf_begin_cpu_access(input->dma_buf, DMA_BIDIRECTIONAL);
      logical = dma_buf_kmap(input->dma_buf, 0);

      file = file_open(TVGEN_INPUT_FILENAME, index);
      tvgen_mem_write(file, logical + input->offs, input->physical, input->size, "input");
      file_close(file);

      dma_buf_kunmap(input->dma_buf, 0, logical);
      dma_buf_end_cpu_access(input->dma_buf, DMA_BIDIRECTIONAL);
      dma_buf_put(input->dma_buf);

      kfree(input);
    }
}

void tvgen_mem_output(const struct dmp_dv_kbuf* buf, dma_addr_t physical, u64 size)
{
  if(buf)
    {
      // always update to get the last output buffer
      if(buf_output.dma_buf) dma_buf_put(buf_output.dma_buf);
      
      buf_output.dma_buf = dma_buf_get(buf->fd);
      buf_output.offs = buf->offs;
      buf_output.physical = physical;
      buf_output.size = size;
    }
  else if(!buf && !list_empty(&list_buf_output))
    {
      u8 *logical;
      struct file* file;
      int index = buf_count_output++;
      struct list_buf *output;

      //pr_debug(DRM_DEV_NAME": put mem_output %d\n", index);
      
      // get entry
      output = list_last_entry(&list_buf_output, struct list_buf, list);
      list_del(&output->list);

      dma_buf_begin_cpu_access(output->dma_buf, DMA_BIDIRECTIONAL);
      logical = dma_buf_kmap(output->dma_buf, 0);

      file = file_open(TVGEN_OUTPUT_FILENAME, index);
      tvgen_mem_write(file, logical + output->offs, output->physical, output->size, "output");
      file_close(file);

      dma_buf_kunmap(output->dma_buf, 0, logical);
      dma_buf_end_cpu_access(output->dma_buf, DMA_BIDIRECTIONAL);
      dma_buf_put(output->dma_buf);

      kfree(output);
    }
}

void tvgen_set_buffer()
{
  is_buf_empty_input = true;
  tvgen_set_output();
}

void tvgen_set_output()
{
  if(buf_output.dma_buf)
    {
      struct list_buf *output;

      //pr_debug(DRM_DEV_NAME": set mem_output\n");

      output = kmalloc(sizeof(struct list_buf), GFP_KERNEL);
      memcpy(output, &buf_output, sizeof(struct list_buf));

      list_add(&output->list, &list_buf_output);
      buf_output.dma_buf = 0;
    }
}
