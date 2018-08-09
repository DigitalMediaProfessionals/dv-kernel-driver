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
#define DRM_NUM_SUBDEV 2
#define DRM_MAX_FIRMWARE_SIZE 0x2000
#define DRM_WAIT_TIMEOUT (2 * HZ)
#define DRM_MAX_WAIT_COUNT 10

struct device;
struct dmp_dv_kcmd_impl;
struct dmp_cmb;

extern int UNIFIED_BUFFER_SIZE;
extern int MAX_CONV_KERNEL_SIZE;

struct dmp_cmb *dv_cmb_init(struct device *dev);
void dv_cmb_finalize(struct device *dev, struct dmp_cmb *cmb);
int dv_convert_conv_command(struct device *dev, struct dmp_cmb *cmb,
			    struct dmp_dv_kcmd_impl *cmd_info);
void dv_run_conv_command(struct dmp_cmb *cmb, void *bar_logical);
int dv_convert_fc_command(struct device *dev, struct dmp_cmb *cmb,
			  struct dmp_dv_kcmd_impl *cmd_info);
void dv_run_fc_command(struct dmp_cmb *cmb, void *bar_logical);

#endif
