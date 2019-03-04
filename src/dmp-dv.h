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

#pragma once

#ifndef _DMP_DV_H
#define _DMP_DV_H

#define DRM_DEV_NAME "dmp_dv"
#define DRM_NUM_SUBDEV 3
#define DRM_WAIT_TIMEOUT (2 * HZ)
#define DRM_MAX_WAIT_COUNT 10

/* IPI registers offset */
#define IPI_TRIG_OFFSET 0x0  /* IPI trigger reg offset */
#define IPI_OBS_OFFSET  0x4  /* IPI observation reg offset */
#define IPI_ISR_OFFSET  0x10 /* IPI interrupt status reg offset */
#define IPI_IMR_OFFSET  0x14 /* IPI interrupt mask reg offset */
#define IPI_IER_OFFSET  0x18 /* IPI interrupt enable reg offset */
#define IPI_IDR_OFFSET  0x1C /* IPI interrup disable reg offset */

#define IPI_MASK        0x100 /* IPI mask for kick from RPU. */

struct device;
struct dmp_dv_kcmd;
struct dmp_cmb;

extern uint32_t UNIFIED_BUFFER_SIZE;
extern uint32_t MAX_CONV_KERNEL_SIZE;

extern void *bar_logi_r5ipi;
extern void *bar_logi_r5shm;

struct dmp_cmb *dv_cmb_init(struct device *dev);
void dv_cmb_finalize(struct device *dev, struct dmp_cmb *cmb);
int dv_convert_conv_command(struct device *dev, struct dmp_cmb *cmb,
			    struct dmp_dv_kcmd *cmd_info);
void dv_run_conv_command(struct dmp_cmb *cmb, void *bar_logical);
int dv_convert_ipu_command(struct device *dev, struct dmp_cmb *cmb,
			   struct dmp_dv_kcmd *cmd_info);
void dv_run_ipu_command(struct dmp_cmb *cmb, void *bar_logical);
int dv_convert_maximizer_command(struct device *dev, struct dmp_cmb *cmb,
			  	 struct dmp_dv_kcmd *cmd_info);
void dv_run_maximizer_command(struct dmp_cmb *cmb, void *bar_logical);

#endif
