/* arch/arm/mach-msm/include/mach/board_lg_mmc.h
 * Copyright (C) 2010 LGE, Inc.
 * Author: jihoon.lee
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
 
 #ifndef __ARCH_MSM_BOARD_LG_MMC_H
#define __ARCH_MSM_BOARD_LG_MMC_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <asm/setup.h>

 /* sdcard related macros */
#ifdef CONFIG_LGE_MMC_DETECTION
#define GPIO_SD_DETECT_N    145 // GPIO_145_AUDIO_SDAC_DOUT	SD_DETECT/ GPIO Pull-u
#define VREG_SD_LEVEL       2850 // VREG_WLAN(G1)	2.85V_SD

#define GPIO_SD_DATA_3      51
#define GPIO_SD_DATA_2      52
#define GPIO_SD_DATA_1      53
#define GPIO_SD_DATA_0      54
#define GPIO_SD_CMD         55
#define GPIO_SD_CLK         56
#endif /*CONFIG_LGE_MMC_DETECTION*/

#endif /*__ARCH_MSM_BOARD_LG_MMC_H*/