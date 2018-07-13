/*
 *  DV700 kernel driver user interface
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

#ifndef _UAPI_LINUX_DMP_DV_H
#define _UAPI_LINUX_DMP_DV_H

#include <linux/ioctl.h>
#include <linux/types.h>


/// @brief Memory buffer specification.
typedef struct dmp_dv_kbuf_impl {
  __u32 fd;    // ION file descriptor
  __u32 rsvd;  // padding to 64-bit size
  __u64 offs;  // offset from the start of the buffer
} dmp_dv_kbuf;


/// @brief Raw command for execution.
typedef struct dmp_dv_kcmdraw_impl {
  __u32 size;     // size of this structure
  __u32 version;  // version of this structure
} dmp_dv_kcmdraw;


/**
 * struct dmp_dv_kcmd - cmd data passed from userspace to append to cmd buffer.
 * @cmd_num:		Number of cmds stored in the cmd_pointer.
 * @cmd_pointer:	Pointer to the array of cmd data.
 */
typedef struct dmp_dv_kcmd_impl {
	__u32 cmd_num;
	__u32 reserved0; /* align to 64bits */
	__u64 cmd_pointer;
} dmp_dv_kcmd;

#define DMP_DV_IOC_MAGIC		0x82

/**
 * DOC: DMP_DV_IOC_APPEND_CMD - Append command(s) to the command buffer.
 *
 * Append one (or more) command(s) to the end of command buffer.
 * Takes a dmp_dv_cmd struct and finds out all dma addresses for buffers used
 * by the command(s). The buffers should be fds allocated by the ION device.
 *
 * If there is not enough memory for the command buffer, returns -ENOMEM.
 */
#define DMP_DV_IOC_APPEND_CMD		_IOW(DMP_DV_IOC_MAGIC, 1, \
						struct dmp_dv_cmd)

/**
 * DOC: DMP_DV_IOC_RUN - Run all commands in the command buffer.
 *
 * Run all commands in the current command buffer. Will acquire the HW lock;
 * if the HW lock is not immediately available then will block in the kernel
 * until it get the lock.
 */
#define DMP_DV_IOC_RUN			_IO(DMP_DV_IOC_MAGIC, 2)

/**
 * DOC: DMP_DV_IOC_WAIT - Wait for the previously run to finish.
 *
 * Wait for the interrupt if this device is currently running.
 * Ignore if the hardware is not running or if it is running commands by other
 * owner.
 */
#define DMP_DV_IOC_WAIT			_IO(DMP_DV_IOC_MAGIC, 3)

/**
 * DOC: DMP_DV_IOC_GET_KICK_COUNT - Get finished kick count.
 *
 * Get the finished kick count by the HW.
 */
#define DMP_DV_IOC_GET_KICK_COUNT	_IOR(DMP_DV_IOC_MAGIC, 4, __u32)

#endif  // _UAPI_LINUX_DMP_DV_H
