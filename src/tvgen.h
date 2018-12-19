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
extern phys_addr_t tvgen_bar_physical[];

void tvgen_init(void);
void tvgen_release(void);
void tvgen_start(const char* path);
void tvgen_end(void);
void tvgen_add_init(u32 value, u32 devid, u32 offset);
void tvgen_phi_ocp_i(u32 value, u32 devid, u32 offset);
void tvgen_phi_ocp_a(u32 value, u32 addr);
void tvgen_mem_write(const char* name, void *logical, dma_addr_t physical, ssize_t size);
void tvgen_mem_weight(const struct dmp_dv_kbuf* buf, dma_addr_t physical, u64 size);
void tvgen_mem_input(const struct dmp_dv_kbuf* buf, dma_addr_t physical, u64 size);
void tvgen_mem_output(const struct dmp_dv_kbuf* buf, dma_addr_t physical, u64 size);

#endif//_TVGEN_H
