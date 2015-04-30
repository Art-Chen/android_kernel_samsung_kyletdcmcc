/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_BOARD_H
#define __ASM_ARCH_BOARD_H

#ifdef	CONFIG_MACH_SP8810EA
#include <mach/gpio-sp8810ea.h>
#endif

#ifdef	CONFIG_MACH_SP8810EB
#include <mach/gpio-sp8810eb.h>
#endif

#ifdef	CONFIG_MACH_SP8810GA
#include <mach/gpio-sp8810ga.h>
#endif

#ifdef	CONFIG_MACH_OPENPHONE_SC6820
#include <mach/gpio-sc6820-openphone.h>
#endif

#ifdef	CONFIG_MACH_AMAZING
#include <mach/gpio-amazing.h>
#endif

#ifdef  CONFIG_MACH_CORI2G
#include <mach/gpio-cori2g.h>
#endif

#ifdef	CONFIG_MACH_KYLETD
#include <mach/gpio-kyletd.h>
#endif

#ifdef	CONFIG_MACH_VASTOI
#include <mach/gpio-vastoi.h>
#endif

#ifdef	CONFIG_MACH_Z788
#include <mach/gpio-z788.h>
#endif
/*
 * pmem area definition
 */
#include <asm/sizes.h>
#define SPRD_PMEM_SIZE		(CONFIG_SPRD_PMEM_SIZE*SZ_1M)
#define SPRD_PMEM_ADSP_SIZE	(CONFIG_SPRD_PMEM_ADSP_SIZE*SZ_1M)
#define SPRD_ROT_MEM_SIZE	(0)
#define SPRD_SCALE_MEM_SIZE	(0)
#define SPRD_IO_MEM_SIZE	(SPRD_PMEM_SIZE+SPRD_PMEM_ADSP_SIZE+ \
				SPRD_ROT_MEM_SIZE+SPRD_SCALE_MEM_SIZE)

#define SPRD_PMEM_BASE		((256*SZ_1M)-SPRD_IO_MEM_SIZE)
#define SPRD_PMEM_ADSP_BASE	(SPRD_PMEM_BASE+SPRD_PMEM_SIZE)
#define SPRD_ROT_MEM_BASE	(SPRD_PMEM_ADSP_BASE+SPRD_PMEM_ADSP_SIZE)
#define SPRD_SCALE_MEM_BASE	(SPRD_ROT_MEM_BASE+SPRD_ROT_MEM_SIZE)

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define SPRD_RAM_CONSOLE_SIZE	0x20000
#define SPRD_RAM_CONSOLE_START	(SPRD_PMEM_BASE - SPRD_RAM_CONSOLE_SIZE)
#endif

#endif
