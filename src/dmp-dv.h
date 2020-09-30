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
#define DRM_NUM_SUBDEV 1
#define DRM_WAIT_TIMEOUT (2 * HZ)
#define DRM_MAX_WAIT_COUNT 10

struct device;
struct dmp_dv_kcmd;
struct dmp_cmb;

struct st_global_vars {
	// For the driver in general
	uint32_t UNIFIED_BUFFER_SIZE;
	uint32_t MAX_CONV_KERNEL_SIZE;
	uint32_t conv_kick_count;
	uint32_t opt;

	// For the program_driver
	unsigned bufA, bufB, bufS, bufC;
	unsigned prev_ubuf_ofm_offset;
	int cofst;
};
extern struct st_global_vars global_vars;

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
