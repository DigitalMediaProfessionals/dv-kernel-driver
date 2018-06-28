/*
 *  DV700 kernel driver
 *  Copyright (C) 2018  Digital Media Professionals Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef PDC_H_
#define PDC_H_

void pdc_config(void __iomem *pdc_addr, int *dims, unsigned int *fbPA);
void pdc_start(void __iomem *pdc_addr);
void pdc_stop(void __iomem *pdc_addr);

#define PDC_REG_FBADDR 0x0068
#define PDC_REG_SWAP 0x0078
#define PDC_REG_STATUS 0x007C

#endif // PDC_H_
