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

#define TVGEN_DEFAULT_PHI_OCP_FILENAME "phi_ocp.txt"
#define TVGEN_DEFAULT_MEMDUMP_FILENAME "memdump.txt"
#define TVGEN_DEFAULT_FILE_PATH "/tmp"

void tvgen_start(const char* path, const char* ocp_name, const char* mem_name);
void tvgen_end(void);
void tvgen_iowrite32(u32 value, void __iomem* addr); // write to phi_ocp.txt

#endif//_TVGEN_H
