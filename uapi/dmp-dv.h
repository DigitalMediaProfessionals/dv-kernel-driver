// SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
/*
 *  DV700 kernel driver user interface
 *
 *  Copyright (C) 2018  Digital Media Professionals Inc.
 *
 * This software is dual licensed under the terms of Apache License, Version 2.0 OR
 * the GNU General Public License version 2, as published by the Free Software Foundation,
 * and may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#pragma once

#ifndef _UAPI_LINUX_DMP_DV_H
#define _UAPI_LINUX_DMP_DV_H

#include <linux/ioctl.h>
#include <linux/types.h>

/// @brief Memory buffer specification.
struct dmp_dv_kbuf {
	__s32 fd;    // ION file descriptor
	__u32 rsvd;  // padding to 64-bit size
	__u64 offs;  // offset from the start of the buffer
};

/// @brief Raw command for execution.
struct dmp_dv_kcmdraw {
	__u32 size;     // size of this structure
	__u32 version;  // version of this structure
};

/// @brief Data passed from userspace to append to cmd buffer.
struct dmp_dv_kcmd {
	__u32 cmd_num;      // number of commands stored in the cmd_pointer
	__u32 rsvd;         // padding to 64-bit size
	__u64 cmd_pointer;  // pointer to the commands data
};

#define DMP_DV_IOC_MAGIC 0x82

/**
 * DOC: DMP_DV_IOC_APPEND_CMD - Append command(s) to the command buffer.
 *
 * Append one (or more) command(s) to the end of command buffer.
 * Takes a dmp_dv_cmd struct and finds out all dma addresses for buffers used
 * by the command(s). The buffers should be fds allocated by the ION device.
 *
 * If there is not enough memory for the command buffer, returns -ENOMEM.
 */
#define DMP_DV_IOC_APPEND_CMD _IOW(DMP_DV_IOC_MAGIC, 1, struct dmp_dv_kcmd)

/**
 * DOC: DMP_DV_IOC_RUN - Run all commands in the command buffer.
 *
 * Queue all commands in the current command buffer to be run by HW.
 * Returns the command run ID.
 */
#define DMP_DV_IOC_RUN _IOR(DMP_DV_IOC_MAGIC, 2, __u64)

/**
 * DOC: DMP_DV_IOC_WAIT - Wait for the previously run to finish.
 *
 * Wait for the specified run ID to be finished.
 * Immediately return if the specified ID is already finished.
 */
#define DMP_DV_IOC_WAIT _IOW(DMP_DV_IOC_MAGIC, 3, __u64)

#ifdef _TVGEN_
#define DMP_DV_IOC_TVGEN_OPEN _IOW(DMP_DV_IOC_MAGIC, 0x10, const char*)
#define DMP_DV_IOC_TVGEN_CLOSE _IO(DMP_DV_IOC_MAGIC, 0x11)
#endif

#endif  // _UAPI_LINUX_DMP_DV_H
