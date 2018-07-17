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

#include <linux/dma-mapping.h>

#define DRM_DEV_NAME "dmp_dv"
#define DRM_NUM_SUBDEV 2

struct device;
struct dmp_dv_kcmd_impl;

struct dmp_cmb {
	dma_addr_t physical;
	void *logical;
	size_t size;
};

int dv_cmb_init(struct device *dev, struct dmp_cmb *cmb);
void dv_cmb_finalize(struct device *dev, struct dmp_cmb *cmb);
int dv_convert_command(struct device *dev, struct dmp_cmb *cmb,
                       struct dmp_dv_kcmd_impl *cmd_info);

#endif
