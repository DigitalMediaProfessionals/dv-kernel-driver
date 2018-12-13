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

#include "tvgen.h"
#include "dmp-dv.h"

static struct file* file_phi_ocp = NULL;
static struct file* file_memdump = NULL;

static struct file *file_open(const char *path, int flags, int rights)
{
    struct file *filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
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

    
static void file_test(struct file* file)
{
    int i;
    ssize_t ret;
    void* buf;
        
    char data[16];
    for(i=0; i<16; i++) ((char*)data)[i] = i;
    ret = file_write(file, data, 16);
    pr_debug(DRM_DEV_NAME": data write %p %ld\n", data, ret);

    buf = vmalloc(16);
    for(i=0; i<16; i++) ((char*)buf)[i] = i + 0x10;
        
    ret = file_write(file, buf, 16);
    pr_debug(DRM_DEV_NAME": vmalloc write %p %ld\n", buf, ret);

    vfree(buf);

    buf = kmalloc(16, GFP_KERNEL);
    for(i=0; i<16; i++) ((char*)buf)[i] = i + 0x20;
        
    ret = file_write(file, buf, 16);
    pr_debug(DRM_DEV_NAME": kvmalloc write %p %ld\n", buf, ret);

    kvfree(buf);

    buf = kvmalloc(16, GFP_USER);
    for(i=0; i<16; i++) ((char*)buf)[i] = i + 0x30;
        
    ret = file_write(file, buf, 16);
    pr_debug(DRM_DEV_NAME": kvmalloc_USER write %p %ld\n", buf, ret);

    kvfree(buf);
}

void tvgen_start(const char* path, const char* ocp_name, const char* mem_name)
{
    char* filename = (char*)kvmalloc(PATH_MAX, GFP_KERNEL);
    
    pr_debug(DRM_DEV_NAME": tvgen_start %p\n", filename);

    if(file_phi_ocp) { file_close(file_phi_ocp); file_phi_ocp = NULL; }
    if(file_memdump) { file_close(file_memdump); file_memdump = NULL; }

    strcpy(filename, path? path : TVGEN_DEFAULT_FILE_PATH);
    strcat(filename, "/");
    strcat(filename, ocp_name? ocp_name : TVGEN_DEFAULT_PHI_OCP_FILENAME);
    file_phi_ocp = file_open(filename, O_WRONLY | O_CREAT, 0644);
    if(file_phi_ocp==NULL)
    {
        pr_err(DRM_DEV_NAME": file open error [%s]\n", filename);
    }
    pr_debug(DRM_DEV_NAME": phi_ocp [%s]\n", filename);

    strcpy(filename, path? path : TVGEN_DEFAULT_FILE_PATH);
    strcat(filename, "/");
    strcat(filename, mem_name? mem_name : TVGEN_DEFAULT_MEMDUMP_FILENAME);
    file_memdump = file_open(filename, O_WRONLY | O_CREAT, 0644);
    if(file_memdump==NULL)
    {
        pr_err(DRM_DEV_NAME": file open error [%s]\n", filename);
    }
    pr_debug(DRM_DEV_NAME": memdump [%s]\n", filename);

    kvfree(filename);
    if(file_phi_ocp) file_test(file_phi_ocp);
}

void tvgen_end(void)
{
    if(file_phi_ocp) { file_close(file_phi_ocp); file_phi_ocp = NULL; }
    if(file_memdump) { file_close(file_memdump); file_memdump = NULL; }

    pr_debug(DRM_DEV_NAME": tvgen_close_ocp\n");
}

// write to phi_ocp
void tvgen_iowrite32(u32 value, void __iomem* addr)
{
    pr_debug(DRM_DEV_NAME": 2_00_%p_f_%08x\n", addr, value);
}
