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

#ifndef _TVGEN_H
#define _TVGEN_H

#define TVGEN_DEFAULT_FILE_PATH "/tmp"
#define TVGEN_PHI_OCP_FILENAME "/phi_ocp.txt"
#define TVGEN_MEMDUMP_FILENAME "/memdump.txt"

enum TVGEN_DEV_ID {
  TVGEN_DEV_CONV =0,
  TVGEN_DEV_FC,
  TVGEN_DEV_IPU
};

typedef struct
{
  phys_addr_t bar_physical;
  unsigned int reg_offset;
  int irq_addr;
} tvgen_dev;
extern tvgen_dev tvgen_dev_info[];

void tvgen_init(void);
void tvgen_release(void);
void tvgen_start(const char* path);
void tvgen_end(void);
void tvgen_w32(u32 value, u32 devid, u32 offset);
void tvgen_w32_irq(u32 value, u32 devid);
void tvgen_add_list(u32 value, u32 devid, u32 offset);

#endif//_TVGEN_H
