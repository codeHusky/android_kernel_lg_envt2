/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/mfd/tps65023.h>
#include <linux/bma150.h>
/* LG_FW : 2010.02.06 jinho.jang - Touch SPI */
#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
#include <linux/ts_amri5k.h>
#endif /*CONFIG_LGE_MACH_ENVT2_REVD*/
#include <linux/power_supply.h>
#include <linux/clk.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/io.h>
#include <asm/setup.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/sirc.h>
#include <mach/dma.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_touchpad.h>
#include <mach/msm_i2ckbd.h>
#include <mach/pmic.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_spi.h>
#include <mach/msm_tsif.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>

#include "devices.h"
#include "timer.h"
#include "socinfo.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#include "proc_comm.h"
#include "smd_private.h"
#include <linux/msm_kgsl.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif

#ifdef CONFIG_LGE_CHARGING
#include <linux/power_supply.h>
#include <mach/msm_battery.h>
#endif

#include <mach/lg_sensors.h>

// share cmd_pkt and length bw kernel and userspace
#if defined (CONFIG_LGE_DIAG_KERNEL_SERVICE)
#include "./lge/lg_diag_communication.h" // for diagcmd_platform_data
#endif

#ifdef CONFIG_LGE_MMC_DETECTION
#include <mach/board_lg_mmc.h>
#endif /*CONFIG_LGE_MMC_DETECTION*/

/* LG_FW : 2010.01.08 jinho.jang */
#if defined(CONFIG_LGE_TOUCHSCREEN_AVAGO)
#else
#define TOUCHPAD_SUSPEND 	34
#define TOUCHPAD_IRQ 		38
#endif

/* LG_FW : 2010.09.28 jinho.jang */
#ifndef LGE_MEM_INCREASE
#define MSM_PMEM_SF_SIZE	0x1C00000 // 28MB
#else
#define MSM_PMEM_SF_SIZE	0x1700000 // 23MB
#endif

#define SMEM_SPINLOCK_I2C	"S:6"

#if 1//LGE_5035_PATCH
#define MSM_PMEM_ADSP_SIZE	0x2B96000 /*43.6MB*/
#else
#define MSM_PMEM_ADSP_SIZE	0x2196000 /*33.6MB*/
#endif

/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA) //32bit
#define MSM_FB_SIZE         0x500000
#else
#define MSM_FB_SIZE         0x2EE000
#endif

#define MSM_AUDIO_SIZE		0x80000
#define MSM_GPU_PHYS_SIZE 	SZ_2M

#ifdef CONFIG_MSM_SOC_REV_A
#define MSM_SMI_BASE		0xE0000000
#else
#define MSM_SMI_BASE		0x00000000
#endif

#define MSM_SHARED_RAM_PHYS	(MSM_SMI_BASE + 0x00100000)

#ifdef CONFIG_LGE_MACH_ENVT2
// SCL_MODEM_TOTAL_SIZE  overflow, see SCL_MM_HEAP1_BASE in targsdcanlym.h
#define SCL_MM_HEAP1_BASE 0x2C00000
#define SCL_SMI_MEM_TOTAL_SIZE 0x4000000

#define MSM_PMEM_SMI_BASE	(MSM_SMI_BASE + SCL_MM_HEAP1_BASE)
#define MSM_PMEM_SMI_SIZE	(SCL_SMI_MEM_TOTAL_SIZE - MSM_PMEM_SMI_BASE) // 0x01400000
#else
#define MSM_PMEM_SMI_BASE	(MSM_SMI_BASE + 0x02B00000)
#define MSM_PMEM_SMI_SIZE	0x01500000
#endif

#define MSM_FB_BASE		MSM_PMEM_SMI_BASE
#define MSM_GPU_PHYS_BASE 	(MSM_FB_BASE + MSM_FB_SIZE)
#define MSM_PMEM_SMIPOOL_BASE	(MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE)
#define MSM_PMEM_SMIPOOL_SIZE	(MSM_PMEM_SMI_SIZE - MSM_FB_SIZE \
					- MSM_GPU_PHYS_SIZE)

#define PMEM_KERNEL_EBI1_SIZE	0x28000

#define PMIC_VREG_WLAN_LEVEL	2600
#define PMIC_VREG_GP6_LEVEL	2900

#define FPGA_SDCC_STATUS	0x70000280

/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
static struct resource smc91x_resources[] = {
	[0] = {
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.flags  = IORESOURCE_IRQ,
	},
};
#endif
/*---------------------------------------------------------------------------*/

#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID
/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
#ifdef CONFIG_LGE_USB_GADGET_DRIVER
    {
		.product_id         = 0x618E, 
		.functions	    	= 0x2743, /*  ACM Modem,diag,NMEA,Mass*/
        .adb_product_id     = 0x618E,
        .adb_functions      = 0x12743, /*  ACM Modem,diag,NMEA,Mass,ADB*/
    },
#ifdef CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB
    {
        .product_id           = 0x6000,
        .functions            = 0x43, /* ACM Modem,diag,NMEA*/
        .adb_product_id     = 0x6000,
	  .adb_functions	    = 0x43, /* ACM Modem,diag,NMEA,ADB*/
    }, 
#endif
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
    {
        .product_id           = 0x61E0,
        .functions            = 0xB, /* MTP*/
        .adb_product_id     = 0x61E0,
	    .adb_functions	    = 0xB, /* MTP,ADB*/
    },
#endif
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	{
		/* LG Rmnet Driver for matching LG Android Net driver */
		.product_id         = 0x61A2,
		.functions	    = 0x27384, /*  diag,NDIS,ACM Modem,NMEA,Mass*/
		.adb_product_id     = 0x61A1,
		.adb_functions	    = 0x127384, /*  diag,NDIS,ACM Modem,NMEA,Mass,ADB*/
	},
#endif
	{
		.product_id 		  = 0x61A4,
		.functions			  = 0x2743, /*  ACM Modem,diag,NMEA,Mass*/
		.adb_product_id 	= 0x61A3,
		.adb_functions		= 0x12743, /*  ACM Modem,diag,NMEA,Mass,ADB*/
	},
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		/* RNDIS */
		.product_id         = 0xF00E,
		.functions	    = 0xA,
		.adb_product_id     = 0x9024,
		.adb_functions	    = 0x1A,
	},
#endif
/* LGE_CHANGE_S : For Autorun */
#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
	{
		/* Mass Storage Only for autorun */
		.product_id         = 0x61E1,
		.functions	    	= 0x2,
		.adb_product_id     = 0x61E1,
		.adb_functions	    = 0x2,
	},
	{
		/* For AutoRun, we use UMS function as CD-ROM drive */
		.product_id         = 0x61E2,
		.functions	   		= 0xC,
		.adb_product_id     = 0x61E2,
		.adb_functions	    = 0xC,
				
	},
#endif	
/* LGE_CHANGE_E : For Autorun */
#else
	{
		/* MSC */
		.product_id         = 0xF000,
		.functions	    = 0x02,
		.adb_product_id     = 0x9015,
		.adb_functions	    = 0x12
	},
#ifdef CONFIG_USB_F_SERIAL
	{
		/* MODEM */
		.product_id         = 0xF00B,
		.functions	    = 0x06,
		.adb_product_id     = 0x901E,
		.adb_functions	    = 0x16,
	},
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		/* DIAG */
		.product_id         = 0x900E,
		.functions	    = 0x04,
		.adb_product_id     = 0x901D,
		.adb_functions	    = 0x14,
	},
#endif
#if defined(CONFIG_USB_ANDROID_DIAG) && defined(CONFIG_USB_F_SERIAL)
	{
		/* DIAG + MODEM */
		.product_id         = 0x9004,
		.functions	    = 0x64,
		.adb_product_id     = 0x901F,
		.adb_functions	    = 0x0614,
	},
	{
		/* DIAG + MODEM + NMEA*/
		.product_id         = 0x9016,
		.functions	    = 0x764,
		.adb_product_id     = 0x9020,
		.adb_functions	    = 0x7614,
	},
	{
		/* DIAG + MODEM + NMEA + MSC */
		.product_id         = 0x9017,
		.functions	    = 0x2764,
		.adb_product_id     = 0x9018,
		.adb_functions	    = 0x27614,
	},
#endif
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	{
		/* MSC + CDC-ECM */
		.product_id         = 0x9014,
		.functions	    = 0x82,
		.adb_product_id     = 0x9023,
		.adb_functions	    = 0x812,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	{
		/* DIAG + RMNET */
		.product_id         = 0x9021,
		.functions	    = 0x94,
		.adb_product_id     = 0x9022,
		.adb_functions	    = 0x914,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		/* RNDIS */
		.product_id         = 0xF00E,
		.functions	    = 0xA,
		.adb_product_id     = 0x9024,
		.adb_functions	    = 0x1A,
	},
#endif
#endif //CONFIG_LGE_USB_GADGET_DRIVER

};

#ifdef CONFIG_LGE_USB_GADGET_DRIVER
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x1004,
	.version	= 0x0100,
	.compositions   = usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.product_name	= "LG Android USB Device",
	.manufacturer_name = "LG Electronics Inc.",
	.serial_number		= NULL,//"LG_ANDROID_VS760",//NULL,	
	.nluns = 1,
};
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "GOOGLE",
	.product	= "Mass Storage",
	.release	= 0xFFFF,
};
static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &mass_storage_pdata,
	},
};
#else
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "GOOGLE",
	.product	= "Mass Storage",
	.release	= 0xFFFF,
};
static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &mass_storage_pdata,
	},
};
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.version	= 0x0100,
	.compositions   = usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
	.nluns = 1,
};
#endif //CONFIG_LGE_USB_GADGET_DRIVER

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#ifdef CONFIG_LGE_MACH_ENVT2_REVC
extern void __init bh6172_init_i2c_subpm(void);
extern void __init ce147_init_i2c_camisp(void);
#endif

/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#endif
/*---------------------------------------------------------------------------*/
#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9015,
		.functions	    = 0x12, /* 10010 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},

	{
		.product_id         = 0x901A,
		.functions	    = 0x0F, /* 01111 */
	},
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "8k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

#ifdef CONFIG_USB_FS_HOST
/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if defined(CONFIG_LGE_MACH_ENVT2)
static struct msm_gpio fsusb_config[] = {
	{ GPIO_CFG(139, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "fs_dat" },
	{ GPIO_CFG(140, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "fs_se0" },
	{ GPIO_CFG(0xff, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "fs_oe_n" },
};
#else
static struct msm_gpio fsusb_config[] = {
	{ GPIO_CFG(139, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "fs_dat" },
	{ GPIO_CFG(140, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "fs_se0" },
	{ GPIO_CFG(141, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "fs_oe_n" },
};
#endif

static int fsusb_gpio_init(void)
{
	return msm_gpios_request(fsusb_config, ARRAY_SIZE(fsusb_config));
}

static void msm_fsusb_setup_gpio(unsigned int enable)
{
	if (enable)
		msm_gpios_enable(fsusb_config, ARRAY_SIZE(fsusb_config));
	else
		msm_gpios_disable(fsusb_config, ARRAY_SIZE(fsusb_config));

}
#endif

#define MSM_USB_BASE              ((unsigned)addr)

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_180NM),
	.vendor_id          = 0x5c6,
	.product_name       = "Qualcomm HSUSB Device",
	.serial_number      = "1234567890ABCDEF",
	.manufacturer_name  = "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.config_gpio    = NULL,

#endif
};

static struct vreg *vreg_usb;
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{

	switch (PHY_TYPE(phy_info)) {
	case USB_PHY_INTEGRATED:
#if defined(CONFIG_LGE_MACH_ENVT2)
		if (on)
			mpp_config_digital_out(7,
				MPP_CFG(MPP_DLOGIC_LVL_MSMP,MPP_DLOGIC_OUT_CTRL_HIGH) ) ;
		else
			mpp_config_digital_out(7,
				MPP_CFG(MPP_DLOGIC_LVL_MSMP,MPP_DLOGIC_OUT_CTRL_LOW) ) ;
#else
		if (on)
			msm_hsusb_vbus_powerup();
		else
			msm_hsusb_vbus_shutdown();
#endif
		break;
	case USB_PHY_SERIAL_PMIC:
		if (on)
			vreg_enable(vreg_usb);
		else
			vreg_disable(vreg_usb);
		break;
	default:
		pr_err("%s: undefined phy type ( %X ) \n", __func__,
						phy_info);
	}

}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_180NM),
	.vbus_power = msm_hsusb_vbus_power,
};

#ifdef CONFIG_USB_FS_HOST
static struct msm_usb_host_platform_data msm_usb_host2_pdata = {
	.phy_info	= USB_PHY_SERIAL_PMIC,
	.config_gpio = msm_fsusb_setup_gpio,
	.vbus_power = msm_hsusb_vbus_power,
};
#endif

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION

static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_smipool_pdata = {
	.name = "pmem_smipool",
	.start = MSM_PMEM_SMIPOOL_BASE,
	.size = MSM_PMEM_SMIPOOL_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};


static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_smipool_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_smipool_pdata },
};


static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

#ifdef CONFIG_LGE_ATS_ETA_MTC
#if 0
static struct atcmd_platform_data eve_atcmd_pdata = {
	.name = "eve_atcmd",
};

static struct platform_device eve_atcmd_device = {
	.name = "eve_atcmd",
	.id = -1,
	.dev    = {
		.platform_data = &eve_atcmd_pdata
	},
};
#endif

#endif /*CONFIG_LGE_ATS_ETA_MTC*/

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		if (!strncmp(name, "mddi_toshiba_wvga_pt", 20))
			ret = 0;
		else
			ret = -ENODEV;
	} else if ((machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf())
#ifdef CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA
			&& ((!strcmp(name, "mddi_lgit_wvga")) ||(!strcmp(name, "ext_mddi_wvga"))))
#else /*origin*/
			&& !strcmp(name, "lcdc_external"))
#endif
		ret = 0;

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang - Touch SPI */
#if defined (CONFIG_LGE_MACH_ENVT2_REVD)

#ifdef CONFIG_FB_MSM_USE_DMAE_VSYNC
#define MAIN_TS_IRQ	100
#define SUB_TS_IRQ	42
#else
#define MAIN_TS_IRQ	141
#define SUB_TS_IRQ	42
#endif /*CONFIG_FB_MSM_USE_DMAE_VSYNC*/

#define SPI_BUS_NUM_TOUCH	0

static struct amri5k_platform_data m_ts_pdata = {
	.setup    = NULL,
	.teardown = NULL,
};

static struct amri5k_platform_data s_ts_pdata = {
	.setup    = NULL,
	.teardown = NULL,
};

static struct spi_board_info ts_board_info[] __initdata = {
	{
		.modalias	= "amri5k_s_ts",
		.mode		= SPI_MODE_3,
		.irq		= MSM_GPIO_TO_INT(SUB_TS_IRQ),
		.bus_num	= SPI_BUS_NUM_TOUCH,
		.chip_select	= 0,
		.max_speed_hz	= 2000000,
		.platform_data	= &s_ts_pdata,
	},
	{
		.modalias	= "amri5k_m_ts",
		.mode		= SPI_MODE_3,
		.irq		= MSM_GPIO_TO_INT(MAIN_TS_IRQ),
		.bus_num	= SPI_BUS_NUM_TOUCH,
		.chip_select	= 2,
		.max_speed_hz	= 2000000,
		.platform_data	= &m_ts_pdata,
	}
};

static struct resource qsd_spi_ts_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA1200000,
		.end	= 0xA1200000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

static struct platform_device qsd_device_ts_spi = {
	.name	        = "spi_qsd",
	.id	        = SPI_BUS_NUM_TOUCH,
	.num_resources	= ARRAY_SIZE(qsd_spi_ts_resources),
	.resource	= qsd_spi_ts_resources,
};


static void init_spi_touch(void)
{	
	gpio_configure(SUB_TS_IRQ, GPIOF_INPUT);
	gpio_configure(MAIN_TS_IRQ, GPIOF_INPUT);
	spi_register_board_info(ts_board_info, ARRAY_SIZE(ts_board_info));
	platform_device_register(&qsd_device_ts_spi);
}


#define CT_CSR_PHYS		0xA8700000
#define TCSR_SPI_MUX		(ct_csr_base + 0x54)

static int msm_qsd_ts_spi_dma_config(void)
{
	void __iomem *ct_csr_base = 0;
	u32 spi_mux;
	int ret = 0;

	ct_csr_base = ioremap(CT_CSR_PHYS, PAGE_SIZE);
	if (!ct_csr_base) {
		pr_err("%s: Could not remap %x\n", __func__, CT_CSR_PHYS);
		return -1;
	}

	spi_mux = readl(TCSR_SPI_MUX);
	switch (spi_mux) {
	case (1):
		qsd_spi_ts_resources[4].start  = DMOV_HSUART1_RX_CHAN;
		qsd_spi_ts_resources[4].end    = DMOV_HSUART1_TX_CHAN;
		qsd_spi_ts_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_ts_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_ts_resources[4].start  = DMOV_HSUART2_RX_CHAN;
		qsd_spi_ts_resources[4].end    = DMOV_HSUART2_TX_CHAN;
		qsd_spi_ts_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_ts_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_ts_resources[4].start  = DMOV_CE_OUT_CHAN;
		qsd_spi_ts_resources[4].end    = DMOV_CE_IN_CHAN;
		qsd_spi_ts_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_ts_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -1;
	}

	iounmap(ct_csr_base);
	return ret;
}

static struct msm_gpio qsd_ts_spi_gpio_config_data[] = {
//	{ GPIO_CFG(17, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_clk" },
//	{ GPIO_CFG(18, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_mosi" },
//	{ GPIO_CFG(19, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_miso" },
	{ GPIO_CFG(20, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "spi_cs0" },
	{ GPIO_CFG(147, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "spi_cs2" },
};

static int msm_qsd_ts_spi_gpio_config(void)
{
	int rc;

	rc = msm_gpios_request_enable(qsd_ts_spi_gpio_config_data,
		ARRAY_SIZE(qsd_ts_spi_gpio_config_data));
	if (rc)
		return rc;	

	return 0;
}

static void msm_qsd_ts_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_ts_spi_gpio_config_data,
		ARRAY_SIZE(qsd_ts_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_ts_spi_pdata = {
	.max_clock_speed = 19200000,
	.clk_name = "spi_clk",
	.gpio_config  = msm_qsd_ts_spi_gpio_config,
	.gpio_release = msm_qsd_ts_spi_gpio_release,
	.dma_config = msm_qsd_ts_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{	
	qsd_device_ts_spi.dev.platform_data = &qsd_ts_spi_pdata;
}


#else /*origin*/

static struct msm_gpio bma_spi_gpio_config_data[] = {
	{ GPIO_CFG(22, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "bma_irq" },
};

static int msm_bma_gpio_setup(struct device *dev)
{
	int rc;

	rc = msm_gpios_request_enable(bma_spi_gpio_config_data,
		ARRAY_SIZE(bma_spi_gpio_config_data));

	return rc;
}

static void msm_bma_gpio_teardown(struct device *dev)
{
	msm_gpios_disable_free(bma_spi_gpio_config_data,
		ARRAY_SIZE(bma_spi_gpio_config_data));
}

static struct bma150_platform_data bma_pdata = {
	.setup    = msm_bma_gpio_setup,
	.teardown = msm_bma_gpio_teardown,
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA1200000,
		.end	= 0xA1200000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

static struct platform_device qsd_device_spi = {
	.name	        = "spi_qsd",
	.id	        = 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

static struct spi_board_info msm_spi_board_info[] __initdata = {
	{
		.modalias	= "bma150",
		.mode		= SPI_MODE_3,
		.irq		= MSM_GPIO_TO_INT(22),
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 10000000,
		.platform_data	= &bma_pdata,
	},
};

#define CT_CSR_PHYS		0xA8700000
#define TCSR_SPI_MUX		(ct_csr_base + 0x54)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_csr_base = 0;
	u32 spi_mux;
	int ret = 0;

	ct_csr_base = ioremap(CT_CSR_PHYS, PAGE_SIZE);
	if (!ct_csr_base) {
		pr_err("%s: Could not remap %x\n", __func__, CT_CSR_PHYS);
		return -1;
	}

	spi_mux = readl(TCSR_SPI_MUX);
	switch (spi_mux) {
	case (1):
		qsd_spi_resources[4].start  = DMOV_HSUART1_RX_CHAN;
		qsd_spi_resources[4].end    = DMOV_HSUART1_TX_CHAN;
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[4].start  = DMOV_HSUART2_RX_CHAN;
		qsd_spi_resources[4].end    = DMOV_HSUART2_TX_CHAN;
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[4].start  = DMOV_CE_OUT_CHAN;
		qsd_spi_resources[4].end    = DMOV_CE_IN_CHAN;
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -1;
	}

	iounmap(ct_csr_base);
	return ret;
}

static struct msm_gpio qsd_spi_gpio_config_data[] = {
	{ GPIO_CFG(17, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_clk" },
	{ GPIO_CFG(18, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_mosi" },
	{ GPIO_CFG(19, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_miso" },
	{ GPIO_CFG(20, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "spi_cs0" },
	{ GPIO_CFG(21, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_16MA), "spi_pwr" },
};

static int msm_qsd_spi_gpio_config(void)
{
	int rc;

	rc = msm_gpios_request_enable(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
	if (rc)
		return rc;

	/* Set direction for SPI_PWR */
	gpio_direction_output(21, 1);

	return 0;
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 19200000,
	.clk_name = "spi_clk",
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}
#endif /*CONFIG_LGE_MACH_ENVT2_REVD*/


/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang - MDDI  */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)

/* MAIN LCD Reset */
#define MAIN_LCD_RESET_N	102
#define SUB_LCD_RESET_N		101


/*static int mddi_power_save_on;*/

static void __init mddi_panel_power_init(void)
{
/* LG_FW : 2010.02.06 jinho.jang - MDDI  */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)

#else
	struct vreg *vreg;
	int rc;	

	vreg = vreg_get(NULL, "lcd");

	if (IS_ERR(vreg)) {
		printk(KERN_ERR "%s: mddi_panel vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg));
		goto exit;
	}

	rc = vreg_set_level(vreg, 2800);
	if (rc) {
		printk(KERN_ERR "%s:  mddi_panel  bt set level failed (%d)\n",
		       __func__, rc);
		goto exit;
	}

	rc = vreg_enable(vreg);
	if (rc) {
		printk(KERN_ERR "%s:  mddi_panel  bt enable failed (%d)\n",
		       __func__, rc);
		goto exit;
	}
	
	printk(KERN_DEBUG "MDDI panel power switch: initialized\n");

	mddi_power_save_on=1;

exit:
	return;
#endif /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/
}

/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang - backlight  */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)

enum {
	GPIO_FOLDER_CLOSE=0,
	GPIO_FOLDER_OPEN,
};

extern void mlcd_backlight_set_level( int level);
extern void slcd_backlight_set_level( int level);

static int mddi_lgit_backlight_level(int level, int max, int min)
{	
	slcd_backlight_set_level(level);
		
	return 0;
}

static void mddi_lgit_config_gpio(int on)
{
	if(on)
	{
		/* Sub lcd  reset */
		gpio_tlmm_config(SUB_LCD_RESET_N,GPIO_ENABLE);
		gpio_direction_output(SUB_LCD_RESET_N,1);
	}
}

static int mddi_get_panel_num(void)
{
	if (is_folder_open() == GPIO_FOLDER_OPEN)
		return 4; //EXT_MDDI_PANEL;
	else
		return 1; //MDDI_PANEL;
}

static struct msm_panel_common_pdata mddi_lgit_pdata = {
	.backlight_level = mddi_lgit_backlight_level,
	.panel_config_gpio = mddi_lgit_config_gpio,
	.panel_num = mddi_get_panel_num,
};

struct platform_device mddi_lgit_device = {
	.name   = "mddi_lgit_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_lgit_pdata,
	}
};

static int ext_mddi_lgit_backlight_level(int level, int max, int min)
{	
	mlcd_backlight_set_level(level);
		
	return 0;
}

static void ext_mddi_lgit_config_gpio(int on)
{
	if(on)
	{		
		/* Main lcd  reset */
		gpio_tlmm_config(MAIN_LCD_RESET_N,GPIO_ENABLE);
		gpio_direction_output(MAIN_LCD_RESET_N,1);
	}
}

static int ext_mddi_get_panel_num(void)
{
	if (is_folder_open() == GPIO_FOLDER_OPEN)
		return 4; //EXT_MDDI_PANEL;
	else
		return 1; //MDDI_PANEL;
}

static struct msm_panel_common_pdata ext_mddi_lgit_pdata = {
	.backlight_level = ext_mddi_lgit_backlight_level,
	.panel_config_gpio = ext_mddi_lgit_config_gpio,
	.panel_num = ext_mddi_get_panel_num,
};

struct platform_device ext_mddi_lgit_device = {
	.name   = "ext_mddi_wvga",
	.id     = 1,
	.dev    = {
		.platform_data = &ext_mddi_lgit_pdata,
	}
};

#endif /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/
/*---------------------------------------------------------------------------*/

/* LG_FW : 2010.02.06 jinho.jang - MDDI  */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)

#else
static void msm_fb_vreg_config(const char *name, int on)
{
	struct vreg *vreg;
	int ret = 0;

	vreg = vreg_get(NULL, name);
	if (IS_ERR(vreg)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
		__func__, name, PTR_ERR(vreg));
		return;
	}

	ret = (on) ? vreg_enable(vreg) : vreg_disable(vreg);
	if (ret)
		printk(KERN_ERR "%s: %s(%s) failed!\n",
			__func__, (on) ? "vreg_enable" : "vreg_disable", name);
}
#endif /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/

static void msm_fb_mddi_power_save(int on)
{
/* LG_FW : 2010.02.06 jinho.jang - MDDI  */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)

#else

	int flag_on = !!on;	
	
	if (mddi_power_save_on == flag_on)
		return;
	
	mddi_power_save_on = flag_on;
	
	msm_fb_vreg_config("lcd", flag_on);
#endif /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/
}

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static struct mddi_platform_data mddi_ext_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

/* LG_FW : 2010.03.06 jinho.jang - mdp vsync  - default is sub fsync*/
#define SUB_FSYNC	98

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = SUB_FSYNC,
};

static void __init msm_fb_add_devices(void)
{
/* LG_FW : 2010.02.02 jinho.jang - MDDI Use */
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("emdh", &mddi_ext_pdata);
	msm_fb_register_device("tvenc", 0);
}

#else /*origin*/

static int mddi_toshiba_pmic_bl(int level)
{
	int ret = -EPERM;

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		ret = pmic_set_led_intensity(LED_LCD, level);

		if (ret)
			printk(KERN_WARNING "%s: can't set lcd backlight!\n",
						__func__);
	}

	return ret;
}

static struct msm_panel_common_pdata mddi_toshiba_pdata = {
	.pmic_backlight = mddi_toshiba_pmic_bl,
};

static struct platform_device mddi_toshiba_device = {
	.name   = "mddi_toshiba",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_toshiba_pdata,
	}
};

static void msm_fb_vreg_config(const char *name, int on)
{
	struct vreg *vreg;
	int ret = 0;

	vreg = vreg_get(NULL, name);
	if (IS_ERR(vreg)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
		__func__, name, PTR_ERR(vreg));
		return;
	}

	ret = (on) ? vreg_enable(vreg) : vreg_disable(vreg);
	if (ret)
		printk(KERN_ERR "%s: %s(%s) failed!\n",
			__func__, (on) ? "vreg_enable" : "vreg_disable", name);
}

#define MDDI_RST_OUT_GPIO 100

static int mddi_power_save_on;
static int msm_fb_mddi_power_save(int on)
{
	int flag_on = !!on;
	int ret = 0;


	if (mddi_power_save_on == flag_on)
		return ret;

	mddi_power_save_on = flag_on;

	if (!flag_on && (machine_is_qsd8x50_ffa()
				|| machine_is_qsd8x50a_ffa())) {
		gpio_set_value(MDDI_RST_OUT_GPIO, 0);
		mdelay(1);
	}

	ret = pmic_lp_mode_control(flag_on ? OFF_CMD : ON_CMD,
		PM_VREG_LP_MSME2_ID);
	if (ret)
		printk(KERN_ERR "%s: pmic_lp_mode_control failed!\n", __func__);

	msm_fb_vreg_config("gp5", flag_on);
	msm_fb_vreg_config("boost", flag_on);

	if (flag_on && (machine_is_qsd8x50_ffa()
			|| machine_is_qsd8x50a_ffa())) {
		gpio_set_value(MDDI_RST_OUT_GPIO, 0);
		mdelay(1);
		gpio_set_value(MDDI_RST_OUT_GPIO, 1);
		gpio_set_value(MDDI_RST_OUT_GPIO, 1);
		mdelay(1);
	}

	return ret;
}

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 98,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("emdh", &mddi_pdata);
	msm_fb_register_device("tvenc", 0);
	msm_fb_register_device("lcdc", 0);
}
#endif /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/
/*---------------------------------------------------------------------------*/

/* LG_FW : 2010.02.06 jinho.jang */
#if !defined(CONFIG_LGE_MACH_ENVT2)
static struct resource msm_audio_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 68,
		.end    = 68,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 69,
		.end    = 69,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 70,
		.end    = 70,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 71,
		.end    = 71,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "sdac_din",
		.start  = 144,
		.end    = 144,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "sdac_dout",
		.start  = 145,
		.end    = 145,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "sdac_wsout",
		.start  = 143,
		.end    = 143,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "cc_i2s_clk",
		.start  = 142,
		.end    = 142,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "audio_master_clkout",
		.start  = 146,
		.end    = 146,
		.flags  = IORESOURCE_IO,
	},
	{
		.name	= "audio_base_addr",
		.start	= 0xa0700000,
		.end	= 0xa0700000 + 4,
		.flags	= IORESOURCE_MEM,
	},

};

static unsigned audio_gpio_on[] = {
	GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_CLK */
	GPIO_CFG(142, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* CC_I2S_CLK */
	GPIO_CFG(143, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* SADC_WSOUT */
	GPIO_CFG(144, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* SADC_DIN */
	GPIO_CFG(145, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* SDAC_DOUT */
	GPIO_CFG(146, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* MA_CLK_OUT */
};

static void __init audio_gpio_init(void)
{
	int pin, rc;

	for (pin = 0; pin < ARRAY_SIZE(audio_gpio_on); pin++) {
		rc = gpio_tlmm_config(audio_gpio_on[pin],
			GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_gpio_on[pin], rc);
			return;
		}
	}
}

static struct platform_device msm_audio_device = {
	.name   = "msm_audio",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_audio_resources),
	.resource       = msm_audio_resources,
};
#endif /*CONFIG_LGE_MACH_ENVT2*/

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 21,
		.end	= 21,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
#ifdef CONFIG_LGE_MACH_ENVT2
/* temp sangyup.kim@lge.com 	
		.start	= 19,
		.end	= 19,
*/		
		.start	= 22,
		.end	= 22,
#else /*origin*/
		.start	= 19,
		.end	= 19,
#endif
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(21),
		.end	= MSM_GPIO_TO_INT(21),
		.flags	= IORESOURCE_IRQ,
	},
};

//!!! Temporary Definition. Need to remove... Start
struct bluesleep_platform_data {	int bluetooth_port_num;};
//!!! Temporary Definition. Need to remove... End

static struct bluesleep_platform_data vs760_bluesleep_data = {
	.bluetooth_port_num = 0,
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
	.dev = {
		.platform_data = &vs760_bluesleep_data,
	},		
};

#ifdef CONFIG_BT
struct bluetooth_platform_data {
	int (*bluetooth_power)(int on);
	int (*bluetooth_toggle_radio)(void *data, bool blocked);
};

enum {
	BT_SYSRST,
	BT_WAKE,
	BT_HOST_WAKE,
	BT_VDD_IO,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_VDD_FREG
};

static struct msm_gpio bt_config_power_off[] = {
#ifdef CONFIG_LGE_MACH_ENVT2
	{ GPIO_CFG(23, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"BT SYSRST" },
	{ GPIO_CFG(22, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"BT WAKE" },
	{ GPIO_CFG(21, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"HOST WAKE" },
#else /*origin*/
	{ GPIO_CFG(18, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"BT SYSRST" },
	{ GPIO_CFG(19, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"BT WAKE" },
	{ GPIO_CFG(21, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"HOST WAKE" },
	{ GPIO_CFG(22, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"BT VDD_IO" },
#endif
	{ GPIO_CFG(43, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(44, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(45, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"UART1DM_RX" },
	{ GPIO_CFG(46, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"UART1DM_TX" }
};

static struct msm_gpio bt_config_power_on[] = {
#ifdef CONFIG_LGE_MACH_ENVT2
	{ GPIO_CFG(23, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"BT SYSRST" },
	{ GPIO_CFG(22, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"BT WAKE" },
	{ GPIO_CFG(21, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),
		"HOST WAKE" },
#else /*origin*/
	{ GPIO_CFG(18, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"BT SYSRST" },
	{ GPIO_CFG(19, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"BT WAKE" },
	{ GPIO_CFG(21, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),
		"HOST WAKE" },
	{ GPIO_CFG(22, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"BT VDD_IO" },
#endif
	{ GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(44, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(45, 2, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),
		"UART1DM_RX" },
	{ GPIO_CFG(46, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"UART1DM_TX" }
};

static struct msm_gpio wlan_config_power_off[] = {
	{ GPIO_CFG(62, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"SDC2_CLK" },
	{ GPIO_CFG(63, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"SDC2_CMD" },
	{ GPIO_CFG(64, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"SDC2_D3" },
	{ GPIO_CFG(65, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"SDC2_D2" },
	{ GPIO_CFG(66, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"SDC2_D1" },
	{ GPIO_CFG(67, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"SDC2_D0" },
	{ GPIO_CFG(113, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"VDD_WLAN" },
	{ GPIO_CFG(138, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"WLAN_PWD" }
};

static struct msm_gpio wlan_config_power_on[] = {
	{ GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"SDC2_CLK" },
	{ GPIO_CFG(63, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
		"SDC2_CMD" },
	{ GPIO_CFG(64, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
		"SDC2_D3" },
	{ GPIO_CFG(65, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
		"SDC2_D2" },
	{ GPIO_CFG(66, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
		"SDC2_D1" },
	{ GPIO_CFG(67, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
		"SDC2_D0" },
	{ GPIO_CFG(113, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"VDD_WLAN" },
	{ GPIO_CFG(138, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"WLAN_PWD" }
};

static int bluetooth_toggle_radio(void *data, bool state)
{
	int ret;
	int (*power_control)(int enable);

    power_control = ((struct bluetooth_platform_data *)data)->bluetooth_power;
	ret = (*power_control)((state == RFKILL_USER_STATE_SOFT_BLOCKED) ? 1 : 0);
	return ret;
}

static int bluetooth_power(int on)
{
	int rc;
#if !defined(CONFIG_LGE_MACH_ENVT2)
	struct vreg *vreg_wlan;

	vreg_wlan = vreg_get(NULL, "wlan");

	if (IS_ERR(vreg_wlan)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_wlan));
		return PTR_ERR(vreg_wlan);
	}
#endif

	if (on) {
#if !defined(CONFIG_LGE_MACH_ENVT2)
		/* units of mV, steps of 50 mV */
		rc = vreg_set_level(vreg_wlan, PMIC_VREG_WLAN_LEVEL);
		if (rc) {
			printk(KERN_ERR "%s: vreg wlan set level failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
		rc = vreg_enable(vreg_wlan);
		if (rc) {
			printk(KERN_ERR "%s: vreg wlan enable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
#endif
		rc = msm_gpios_enable(bt_config_power_on,
					ARRAY_SIZE(bt_config_power_on));
		if (rc < 0) {
			printk(KERN_ERR
				"%s: bt power on gpio config failed: %d\n",
				__func__, rc);
			return rc;
		}

		if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
			rc = msm_gpios_enable
					(wlan_config_power_on,
					 ARRAY_SIZE(wlan_config_power_on));
			if (rc < 0) {
				printk
				 (KERN_ERR
				 "%s: wlan power on gpio config failed: %d\n",
					__func__, rc);
				return rc;
			}
		}

/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
		gpio_set_value(22, on); /* VDD_IO */
		gpio_set_value(18, on); /* SYSRST */
#else
		gpio_set_value(23, on); /* SYSRST */
#endif	

		if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
			gpio_set_value(138, 0); /* WLAN: CHIP_PWD */
			gpio_set_value(113, on); /* WLAN */
		}
	} else {
		if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
			gpio_set_value(138, on); /* WLAN: CHIP_PWD */
			gpio_set_value(113, on); /* WLAN */
		}

/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
		gpio_set_value(18, on); /* SYSRST */
		gpio_set_value(22, on); /* VDD_IO */
#else
		gpio_set_value(23, on); /* SYSRST */
#endif

#if !defined(CONFIG_LGE_MACH_ENVT2)
		rc = vreg_disable(vreg_wlan);
		if (rc) {
			printk(KERN_ERR "%s: vreg wlan disable failed (%d)\n",
					__func__, rc);
			return -EIO;
		}
#endif
		rc = msm_gpios_enable(bt_config_power_off,
					ARRAY_SIZE(bt_config_power_off));
		if (rc < 0) {
			printk(KERN_ERR
				"%s: bt power off gpio config failed: %d\n",
				__func__, rc);
			return rc;
		}

		if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
			rc = msm_gpios_enable
					(wlan_config_power_off,
					 ARRAY_SIZE(wlan_config_power_off));
			if (rc < 0) {
				printk
				 (KERN_ERR
				 "%s: wlan power off gpio config failed: %d\n",
					__func__, rc);
				return rc;
			}
		}
	}

	printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);

	return 0;
}

static struct bluetooth_platform_data vs760_bluetooth_data = {
	.bluetooth_power = &bluetooth_power,
	.bluetooth_toggle_radio = &bluetooth_toggle_radio,
};

static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
	.dev = {
		.platform_data = &vs760_bluetooth_data,
	},
};

static void __init bt_power_init(void)
{
	struct vreg *vreg_bt;
	int rc;

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		gpio_set_value(138, 0); /* WLAN: CHIP_PWD */
		gpio_set_value(113, 0); /* WLAN */
	}

/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
	gpio_set_value(18, 0); /* SYSRST */
	gpio_set_value(22, 0); /* VDD_IO */
#else
	gpio_set_value(23, 0); /* SYSRST */	
#endif

#if !defined(CONFIG_LGE_MACH_ENVT2)
	/* do not have vreg bt defined, gp6 is the same */
	/* vreg_get parameter 1 (struct device *) is ignored */
	vreg_bt = vreg_get(NULL, "gp6");

	if (IS_ERR(vreg_bt)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_bt));
		goto exit;
	}

	/* units of mV, steps of 50 mV */
	rc = vreg_set_level(vreg_bt, PMIC_VREG_GP6_LEVEL);
	if (rc) {
		printk(KERN_ERR "%s: vreg bt set level failed (%d)\n",
		       __func__, rc);
		goto exit;
	}
	rc = vreg_enable(vreg_bt);
	if (rc) {
		printk(KERN_ERR "%s: vreg bt enable failed (%d)\n",
		       __func__, rc);
		goto exit;
	}
#endif

	if (bluetooth_power(0))
		goto exit;

	//msm_bt_power_device.dev.platform_data = &bluetooth_power;

	printk(KERN_DEBUG "Bluetooth power switch: initialized\n");

exit:
	return;
}
#else
#define bt_power_init(x) do {} while (0)
#endif

static struct resource kgsl_resources[] = {
       {
		.name  = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
       },
       {
		.name   = "kgsl_phys_memory",
		.start = MSM_GPU_PHYS_BASE,
		.end = MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE - 1,
		.flags = IORESOURCE_MEM,
       },
       {
		.name = "kgsl_yamato_irq",
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
       },
};
static struct kgsl_platform_data kgsl_pdata = {
	.high_axi_3d = 128000, /* Max for 8K */
	.max_grp2d_freq = 0,
	.min_grp2d_freq = 0,
	.set_grp2d_async = NULL,
	.max_grp3d_freq = 0,
	.min_grp3d_freq = 0,
	.set_grp3d_async = NULL,
	.imem_clk_name = "imem_clk",
	.grp3d_clk_name = "grp_clk",
	.grp2d_clk_name = NULL,
};

static struct platform_device msm_device_kgsl = {
       .name = "kgsl",
       .id = -1,
       .num_resources = ARRAY_SIZE(kgsl_resources),
       .resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

static struct platform_device msm_device_pmic_leds = {
	.name	= "pmic-leds",
	.id	= -1,
};

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_A_SYNC      GPIO_CFG(106, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA)
#define TSIF_A_DATA      GPIO_CFG(107, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA)
#define TSIF_A_EN        GPIO_CFG(108, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA)
/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if defined(CONFIG_LGE_MACH_ENVT2)
#define TSIF_A_CLK       GPIO_CFG(0xFF, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA)
#else
#define TSIF_A_CLK       GPIO_CFG(109, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA)
#endif

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_A_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_A_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_A_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_A_SYNC, .label =  "tsif_sync", },
#if 0
	{ .gpio_cfg = 0, .label =  "tsif_error", },
	{ .gpio_cfg = 0, .label =  "tsif_null", },
#endif
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_clk = "tsif_clk",
	.tsif_ref_clk = "tsif_ref_clk",
};

#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

#ifdef CONFIG_QSD_SVS
#define TPS65023_MAX_DCDC1	1600
#else
#define TPS65023_MAX_DCDC1	CONFIG_QSD_PMIC_DEFAULT_DCDC1
#endif

static int qsd8x50_tps65023_set_dcdc1(int mVolts)
{
	int rc = 0;
#ifdef CONFIG_QSD_SVS
#ifdef CONFIG_LGE_CAM_LOCKUP_TEMP

#else
	rc = tps65023_set_dcdc1_level(mVolts);
#endif
	/* By default the TPS65023 will be initialized to 1.225V.
	 * So we can safely switch to any frequency within this
	 * voltage even if the device is not probed/ready.
	 */
	if (rc == -ENODEV && mVolts <= CONFIG_QSD_PMIC_DEFAULT_DCDC1)
		rc = 0;
#else
	/* Disallow frequencies not supported in the default PMIC
	 * output voltage.
	 */
	if (mVolts > CONFIG_QSD_PMIC_DEFAULT_DCDC1)
		rc = -EFAULT;
#endif
	return rc;
}

static struct msm_acpu_clock_platform_data qsd8x50_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.max_vdd = TPS65023_MAX_DCDC1,
	.acpu_set_vdd = qsd8x50_tps65023_set_dcdc1,
};


/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang */
#if defined(CONFIG_LGE_TOUCHSCREEN_AVAGO)
/* defined in board_vs760_gpio_i2c */
#else /*origin*/
static void touchpad_gpio_release(void)
{
	gpio_free(TOUCHPAD_IRQ);
	gpio_free(TOUCHPAD_SUSPEND);
}

static int touchpad_gpio_setup(void)
{
	int rc;
	int suspend_pin = TOUCHPAD_SUSPEND;
	int irq_pin = TOUCHPAD_IRQ;
	unsigned suspend_cfg =
		GPIO_CFG(suspend_pin, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
	unsigned irq_cfg =
		GPIO_CFG(irq_pin, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);

	rc = gpio_request(irq_pin, "msm_touchpad_irq");
	if (rc) {
		pr_err("gpio_request failed on pin %d (rc=%d)\n",
		       irq_pin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_request(suspend_pin, "msm_touchpad_suspend");
	if (rc) {
		pr_err("gpio_request failed on pin %d (rc=%d)\n",
		       suspend_pin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_tlmm_config(suspend_cfg, GPIO_ENABLE);
	if (rc) {
		pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
		       suspend_pin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_tlmm_config(irq_cfg, GPIO_ENABLE);
	if (rc) {
		pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
		       irq_pin, rc);
		goto err_gpioconfig;
	}
	return rc;

err_gpioconfig:
	touchpad_gpio_release();
	return rc;
}

static struct msm_touchpad_platform_data msm_touchpad_data = {
	.gpioirq     = TOUCHPAD_IRQ,
	.gpiosuspend = TOUCHPAD_SUSPEND,
	.gpio_setup  = touchpad_gpio_setup,
	.gpio_shutdown = touchpad_gpio_release
};

#define KBD_RST 35
#define KBD_IRQ 36

static void kbd_gpio_release(void)
{
	gpio_free(KBD_IRQ);
	gpio_free(KBD_RST);
}

static int kbd_gpio_setup(void)
{
	int rc;
	int respin = KBD_RST;
	int irqpin = KBD_IRQ;
	unsigned rescfg =
		GPIO_CFG(respin, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA);
	unsigned irqcfg =
		GPIO_CFG(irqpin, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);

	rc = gpio_request(irqpin, "gpio_keybd_irq");
	if (rc) {
		pr_err("gpio_request failed on pin %d (rc=%d)\n",
			irqpin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_request(respin, "gpio_keybd_reset");
	if (rc) {
		pr_err("gpio_request failed on pin %d (rc=%d)\n",
			respin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_tlmm_config(rescfg, GPIO_ENABLE);
	if (rc) {
		pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
			respin, rc);
		goto err_gpioconfig;
	}
	rc = gpio_tlmm_config(irqcfg, GPIO_ENABLE);
	if (rc) {
		pr_err("gpio_tlmm_config failed on pin %d (rc=%d)\n",
			irqpin, rc);
		goto err_gpioconfig;
	}
	return rc;

err_gpioconfig:
	kbd_gpio_release();
	return rc;
}

/* use gpio output pin to toggle keyboard external reset pin */
static void kbd_hwreset(int kbd_mclrpin)
{
	gpio_direction_output(kbd_mclrpin, 0);
	gpio_direction_output(kbd_mclrpin, 1);
}

static struct msm_i2ckbd_platform_data msm_kybd_data = {
	.hwrepeat = 0,
	.scanset1 = 1,
	.gpioreset = KBD_RST,
	.gpioirq = KBD_IRQ,
	.gpio_setup = kbd_gpio_setup,
	.gpio_shutdown = kbd_gpio_release,
	.hw_reset = kbd_hwreset,
};
#endif /*CONFIG_LGE_TOUCHSCREEN_AVAGO*/
/*---------------------------------------------------------------------------*/

static struct i2c_board_info msm_i2c_board_info[] __initdata = {
#if !defined(CONFIG_LGE_MACH_ENVT2) //CONFIG_LGE_CAMERA_CONFIG
	{
		I2C_BOARD_INFO("glidesensor", 0x2A),
		.irq           =  MSM_GPIO_TO_INT(TOUCHPAD_IRQ),
		.platform_data = &msm_touchpad_data
	},
	{
		I2C_BOARD_INFO("msm-i2ckbd", 0x3A),
		.type           = "msm-i2ckbd",
		.irq           =  MSM_GPIO_TO_INT(KBD_IRQ),
		.platform_data  = &msm_kybd_data
	},
#endif

#ifdef CONFIG_MT9D112
	{
		I2C_BOARD_INFO("mt9d112", 0x78 >> 1),
	},
#endif
#ifdef CONFIG_S5K3E2FX
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9P012
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
#endif
#ifdef CONFIG_MT9P012_KM
	{
		I2C_BOARD_INFO("mt9p012_km", 0x6C >> 2),
	},
#endif
#if defined(CONFIG_MT9T013) || defined(CONFIG_SENSORS_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif
	{
		I2C_BOARD_INFO("tps65023", 0x48),
	},
};


#ifdef CONFIG_S5K4E1GX
static struct i2c_board_info s5k4e1gx_i2c_bdinfo[] __initdata = {
	{
		I2C_BOARD_INFO("s5k4e1gx", 0x20 >> 2),
	},
};
#endif //CONFIG_S5K4E1GX

#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_LGE_MACH_ENVT2_REVD
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* 5M CAM_RESET */
	GPIO_CFG(1,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* 5M CAM_IO_OFF */
	GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[0] */
	GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[1] */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[2] */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[3] */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[4] */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[5] */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[6] */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[7] */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[8] */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[9] */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),  /* MCLK */
};
static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* 5M CAM_RESET */
	GPIO_CFG(1,  0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* 5M CAM_IO_OFF */
	GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[0] */
	GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[1] */
	GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[2] */
	GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[3] */
	GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[4] */
	GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[5] */
	GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[6] */
	GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[7] */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[8] */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAM_DATA[9] */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* MCLK */
};
#else //QCT Orginal
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* MCLK */
};
#endif
#ifdef CONFIG_LGE_CAMERA_CONFIG
#else
static uint32_t camera_on_gpio_ffa_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(95,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* I2C_SCL */
	GPIO_CFG(96,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* I2C_SDA */
	/* FFA front Sensor Reset */
	GPIO_CFG(137,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA),
};

static uint32_t camera_off_gpio_ffa_table[] = {
	/* FFA front Sensor Reset */
	GPIO_CFG(137,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA),
};
#endif //CONFIG_LGE_CAMERA_CONFIG 

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
#ifdef CONFIG_LGE_CAMERA_CONFIG
#else
static struct vreg *vreg_gp2;
static struct vreg *vreg_gp3;

static void msm_camera_vreg_config(int vreg_en)
{
	int rc;

	if (vreg_gp2 == NULL) {
		vreg_gp2 = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_gp2)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp2", PTR_ERR(vreg_gp2));
			return;
		}

		rc = vreg_set_level(vreg_gp2, 1800);
		if (rc) {
			printk(KERN_ERR "%s: GP2 set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_gp3 == NULL) {
		vreg_gp3 = vreg_get(NULL, "gp3");
		if (IS_ERR(vreg_gp3)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp3", PTR_ERR(vreg_gp3));
			return;
		}

		rc = vreg_set_level(vreg_gp3, 2800);
		if (rc) {
			printk(KERN_ERR "%s: GP3 set level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 enable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_enable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 enable failed (%d)\n",
				__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 disable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_disable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 disable failed (%d)\n",
				__func__, rc);
		}
	}
}
#endif //CONFIG_LGE_CAMERA_CONFIG 

static void config_camera_on_gpios(void)
{
#ifdef CONFIG_LGE_CAMERA_CONFIG
#else
	int vreg_en = 1;

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		config_gpio_table(camera_on_gpio_ffa_table,
		ARRAY_SIZE(camera_on_gpio_ffa_table));

		msm_camera_vreg_config(vreg_en);
		gpio_set_value(137, 0);
	}
#endif //CONFIG_LGE_CAMERA_CONFIG 
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
#ifdef CONFIG_LGE_CAMERA_CONFIG //2010.03.19 woochang.chun
#else
	int vreg_en = 0;

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		config_gpio_table(camera_off_gpio_ffa_table,
		ARRAY_SIZE(camera_off_gpio_ffa_table));

		msm_camera_vreg_config(vreg_en);
	}
#endif //CONFIG_LGE_CAMERA_CONFIG 
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}
#ifdef CONFIG_LGE_MACH_ENVT2_REVD
extern int mlcd_aat2870_camera_power_on(void);
extern int mlcd_aat2870_camera_power_off(void);

int camera_power_on (void)
{
	printk(KERN_ERR "%s: camera_power_on \n", __func__);
	return mlcd_aat2870_camera_power_on();
}

int camera_power_off (void)
{
	printk(KERN_ERR "%s: camera_power_off \n", __func__);
	return mlcd_aat2870_camera_power_off();
}
#endif

static struct resource msm_camera_resources[] = {
	{
		.start	= 0xA0F00000,
		.end	= 0xA0F00000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
#ifdef CONFIG_S5K4E1GX
	.camera_power_on = camera_power_on,
	.camera_power_off = camera_power_off,
#endif
};

#ifdef CONFIG_S5K4E1GX
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
	._fsrc.pmic_src.low_current  = 30,
	._fsrc.pmic_src.high_current = 100,
};
#endif 

#ifdef CONFIG_S5K4E1GX
static struct msm_camera_sensor_flash_data flash_s5k4elgx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_data = {
	.sensor_name    = "s5k4e1gx",
	.sensor_reset   = 0,
	.sensor_pwd     = 1,
	.vcm_pwd        = 0,
	.vcm_enable		= 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_s5k4elgx,
};

struct platform_device msm_camera_sensor_s5k4e1gx = {
	.name      = "msm_camera_s5k4e1gx",
	.dev       = {
	.platform_data = &msm_camera_sensor_s5k4e1gx_data,
	},
};
#endif //CONFIG_S5K4E1GX

#ifdef CONFIG_MT9D112
static struct msm_camera_sensor_flash_data flash_mt9d112 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9d112_data = {
	.sensor_name    = "mt9d112",
	.sensor_reset   = 17,
	.sensor_pwd     = 85,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9d112
};

static struct platform_device msm_camera_sensor_mt9d112 = {
	.name      = "msm_camera_mt9d112",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9d112_data,
	},
};
#endif

#ifdef CONFIG_S5K3E2FX
static struct msm_camera_sensor_flash_data flash_s5k3e2fx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
	.sensor_reset   = 17,
	.sensor_pwd     = 85,
	/*.vcm_pwd = 31, */  /* CAM1_VCM_EN, enabled in a9 */
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_s5k3e2fx
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_flash_data flash_mt9p012 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name    = "mt9p012",
	.sensor_reset   = 17,
	.sensor_pwd     = 85,
	.vcm_pwd        = 88,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9p012
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name      = "msm_camera_mt9p012",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

#ifdef CONFIG_MT9P012_KM
static struct msm_camera_sensor_flash_data flash_mt9p012_km = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_km_data = {
	.sensor_name    = "mt9p012_km",
	.sensor_reset   = 17,
	.sensor_pwd     = 85,
	.vcm_pwd        = 88,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9p012_km
};

static struct platform_device msm_camera_sensor_mt9p012_km = {
	.name      = "msm_camera_mt9p012_km",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_km_data,
	},
};
#endif

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_flash_data flash_mt9t013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 17,
	.sensor_pwd     = 85,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9t013
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name      = "msm_camera_mt9t013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif

#if defined (CONFIG_AAT1270_FLASH)
#define FLASH_EN		93
#define MOVIE_MODE_EN	160
#define FLASH_INHIBIT	91
struct aat1270_flash_platform_data aat1270_flash_pdata = {
	.gpio_flen		= FLASH_EN,
	.gpio_en_set	= MOVIE_MODE_EN,
	.gpio_inh		= FLASH_INHIBIT,
};

static struct platform_device aat1270_flash_device = {
	.name				= "aat1270_flash",
	.id					= -1,
	.dev.platform_data	= &aat1270_flash_pdata,
};
#endif
#endif /*CONFIG_MSM_CAMERA*/

#if defined(CONFIG_LGE_MACH_ENVT2)
#else
static u32 msm_calculate_batt_capacity(u32 current_voltage);
#endif

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design 	= 3200,
	.voltage_max_design	= 4200,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
#if defined(CONFIG_LGE_MACH_ENVT2)
#else
	.calculate_capacity	= &msm_calculate_batt_capacity,
#endif
};

#if defined(CONFIG_LGE_MACH_ENVT2)
#else
static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage  = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
		/ (high_voltage - low_voltage);
}
#endif

static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};


// LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/03/19 {
#ifdef CONFIG_LGE_AUDIO_PWM_VIBRATOR
/* Vibrator Functions for Android Vibrator Driver */
#include <mach/board.h>


#define VIBE_IC_VOLTAGE			3050	/* Change from 3000 to 3050 */
#define GPIO_LIN_MOTOR_PWM		105
#define GPIO_LIN_MOTOR_EN		157
#define GPIO_MOTOR_PWM		105
#define GPIO_MOTOR_EN		157

int g_nForce_32 = 0;

#define GPMN_M_DEFAULT			    21
#define GPMN_N_DEFAULT			    4475 //4500
#define GPMN_D_DEFAULT			    2250	/* 50% duty cycle */ 
#define GPMN_IMM_PWM_MULTIPLIER		4458	/* Must be integer */

/*
** Global variables for LRA PWM M,N counters.
*/
int g_nLRA_GPMN_M = GPMN_M_DEFAULT;
int g_nLRA_GPMN_N = GPMN_N_DEFAULT;
int g_nLRA_GPMN_D = GPMN_D_DEFAULT;
int g_nLRA_GPMN_CLK_PWM_MUL = GPMN_IMM_PWM_MULTIPLIER;

int t_nLRA_GPMN_M = GPMN_M_DEFAULT;
int t_nLRA_GPMN_N = GPMN_N_DEFAULT;
int t_nLRA_GPMN_CLK_PWM_MUL = GPMN_IMM_PWM_MULTIPLIER;


#define MSM_CLK_PHYS          0xA8600000 
#define MSM_CLK_SIZE          SZ_4K
volatile void __iomem *MSM_CLK_BASE ;


#define	HWIO_GP_MD_REG_ADDR				(MSM_CLK_BASE+0x54)
#define	HWIO_GP_MD_REG_OUTM(M,V)	HWIO_OUTM(GP_MD_REG,M,V)
#define	HWIO_GP_NS_REG_ADDR				(MSM_CLK_BASE+0x58)
#define	HWIO_GP_NS_REG_OUTM(M,V)	HWIO_OUTM(GP_NS_REG,M,V)

#define HWIO_GP_NS_REG_GP_ROOT_ENA_SHFT 				11	// clk src signal enable
#define HWIO_GP_NS_REG_GP_CLK_INV_SHFT					10	// clk inv no
#define HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_SHFT 	9		// gp_clk output enable
#define HWIO_GP_NS_REG_MNCNTR_EN_SHFT 					8		// mn-cnt enable
#define HWIO_GP_NS_REG_MNCNTR_RST_SHFT					7		// rst..no
#define HWIO_GP_NS_REG_MNCNTR_MODE_SHFT 				5		// mn-cnt SE mode
#define HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT 				3		// pre-divder 1/4
#define HWIO_GP_NS_REG_SRC_SEL_SHFT 		 				0		// src=tcxo

#define	HWIO_GP_MD_REG_M_VAL_SHFT								0x10
#define HWIO_GP_MD_REG_M_VAL_BMSK								(0xFFFF<<HWIO_GP_MD_REG_M_VAL_SHFT)
#define	HWIO_GP_MD_REG_D_VAL_SHFT								0x00
#define HWIO_GP_MD_REG_D_VAL_BMSK								(0xFFFF<<HWIO_GP_MD_REG_D_VAL_SHFT)
#define	HWIO_GP_NS_REG_N_VAL_SHFT								0x10
#define HWIO_GP_NS_REG_N_VAL_BMSK								(0xFFFF<<HWIO_GP_NS_REG_N_VAL_SHFT)

#define	HWIO_OUTM(REG,M,V)\
	*(volatile unsigned int *)(HWIO_##REG##_ADDR)	= \
		((*(volatile unsigned int *)(HWIO_##REG##_ADDR)) & (~(M))) | ( (V) & (M) )

#define GP_CLK_SET_Mval(mval)  \
    HWIO_GP_MD_REG_OUTM(HWIO_GP_MD_REG_M_VAL_BMSK, mval<<HWIO_GP_MD_REG_M_VAL_SHFT)
  
#define GP_CLK_SET_Dval(dval)  \
    HWIO_GP_MD_REG_OUTM(HWIO_GP_MD_REG_D_VAL_BMSK, ~(dval<<1)<<HWIO_GP_MD_REG_D_VAL_SHFT)
  
#define GP_CLK_SET_Nval(nval)  \
    HWIO_GP_NS_REG_OUTM(HWIO_GP_NS_REG_N_VAL_BMSK, ~(nval-g_nLRA_GPMN_M)<<HWIO_GP_NS_REG_N_VAL_SHFT)

#define HWIO_GP_NS_REG_CNTL_VAL_BMSK  0xfff

#ifdef CONFIG_LGE_MACH_ENVT2_REVD
extern int slcd_aat2870_set_ldo_power(int ldo_no, int on);

int vibrator_power_control(int enable)
{
	printk(KERN_ERR "%s: enable : %d \n", __func__, enable);
	return slcd_aat2870_set_ldo_power(1,enable);
}
#endif

static void GP_CLK_EN(bool en_gp_clk)
{
    static int gp_clk_control;
    
    if (en_gp_clk)
    {
        gp_clk_control = 1 << HWIO_GP_NS_REG_GP_ROOT_ENA_SHFT         // clk src signal enable
                       | 0 << HWIO_GP_NS_REG_GP_CLK_INV_SHFT          // clk inv no
                       | 1 << HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_SHFT   // gp_clk output enable
                       | 1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT           // mn-cnt enable
                       | 0 << HWIO_GP_NS_REG_MNCNTR_RST_SHFT          // rst..no
                       | 3 << HWIO_GP_NS_REG_MNCNTR_MODE_SHFT         // mn-cnt SE mode
                       | 3 << HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT         // pre-divder 1/4
                       | 0 << HWIO_GP_NS_REG_SRC_SEL_SHFT      ;      // src=tcxo
    }
    else
    {
        gp_clk_control = 0;
    }
    HWIO_GP_NS_REG_OUTM(HWIO_GP_NS_REG_CNTL_VAL_BMSK, gp_clk_control);
}

static void vibe_set_pwm_freq(void)
{

    // GP_CLK control field setting : gp_clk en, clk_src=tcxo
    GP_CLK_EN(1);
    // GP_CLK N-div setting
    GP_CLK_SET_Nval(g_nLRA_GPMN_N);
    // GP_CLK M-div setting
    GP_CLK_SET_Mval(g_nLRA_GPMN_M);
    // GP_CLK Duty Setting
    GP_CLK_SET_Dval(g_nLRA_GPMN_D);

}

void ForceOut_AmpDisable(void)
{
     gpio_set_value(GPIO_MOTOR_EN, 0);
}


void ForceOut_AmpEnable(void)
{

     vibe_set_pwm_freq(); 
     gpio_set_value(GPIO_MOTOR_EN, 1);
}

void ForceOut_Set(int nForce)
{
    if(nForce == 0)
    {
        ForceOut_AmpDisable();
            
        /* Set 50% duty cycle */
        GP_CLK_SET_Dval(g_nLRA_GPMN_D);

		GP_CLK_EN(0);
		gpio_tlmm_config( GPIO_CFG(105, 0, GPIO_OUTPUT,GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE ) ;// sleep_current
		gpio_configure( 105, GPIOF_DRIVE_OUTPUT);
		gpio_set_value(105, 0);
    }
    else
    {
#ifdef CONFIG_LGE_AUDIO_TUNNING
		 g_nLRA_GPMN_M = t_nLRA_GPMN_M;
		 g_nLRA_GPMN_N = t_nLRA_GPMN_N;
		 g_nLRA_GPMN_D = (t_nLRA_GPMN_N >> 1);
		 g_nLRA_GPMN_CLK_PWM_MUL = t_nLRA_GPMN_CLK_PWM_MUL;
#endif 		
		gpio_tlmm_config( GPIO_CFG(105, 2, GPIO_OUTPUT,GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE ) ;// sleep_current
		gpio_configure( 105, GPIOF_DRIVE_OUTPUT);
		gpio_set_value(105, 1);

         ForceOut_AmpEnable();
         gpio_set_value(GPIO_MOTOR_EN, 1);
      	 g_nForce_32 = ((nForce * g_nLRA_GPMN_CLK_PWM_MUL) >> 8) + g_nLRA_GPMN_D;
         GP_CLK_SET_Dval(g_nForce_32);
    }
}

int vibrator_power_set(int enable)
{
#ifdef CONFIG_LGE_MACH_ENVT2_REVD
	vibrator_power_control(enable);
#endif 
	return 0;
}

int vibrator_pwm_set(int enable, int amp)
{
	if (enable) {
		ForceOut_Set(amp);
	} else {
		ForceOut_Set(0);
	}
	return 0;
}

int vibrator_ic_enable_set(int enable)
{
	if (enable) {
		gpio_set_value(GPIO_LIN_MOTOR_EN, 1);
	} else {
		gpio_set_value(GPIO_LIN_MOTOR_EN, 0);
	}
	return 0;
}

static struct android_vibrator_platform_data stern_vibrator_data = {
	.enable_status = 0,
	.power_set = vibrator_power_set,
	.pwm_set = vibrator_pwm_set,
	.ic_enable_set = vibrator_ic_enable_set,
	.amp_value = 125, 
};


static struct platform_device android_vibrator_device = {
	.name 	= "android-vibrator",
	.id	= -1,
	.dev = {
		.platform_data = &stern_vibrator_data,
	},
};
#endif /*CONFIG_LGE_AUDIO_AMP_LM49250*/
// LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/03/19 }

/* LG_FW : 2010.02.02 jinho.jang -----------------------------------------------*/
#if defined(CONFIG_LGE_KEYBOARD_QWERTY_PP2106)
static struct platform_device qwerty_device = {
  .name = "pp2106",
  .id = -1,
  .dev = {
    .platform_data = 0,
  },
};
#endif
/*---------------------------------------------------------------------------*/

/* LG_FW : 2010.02.02 jinho.jang -----------------------------------------------*/
#if defined(CONFIG_LGE_ANDROID_HALL_IC)
static struct platform_device android_hallic_device = {
	.name 	= "hall-ic",
	.id	= -1,
	.dev = {
		.platform_data = 0,
	},
};
#endif
/*---------------------------------------------------------------------------*/
#if defined (CONFIG_LGE_DOCK_STATE_DETECT) && defined (CONFIG_LGE_MACH_ENVT2_REVD)
static struct platform_device hallic_dock_device = {
	.name   = "hall-ic-dock",
	.id = -1,
	.dev = {
		.platform_data = NULL,
	},
};
#endif 

#ifdef CONFIG_LGE_DIAG_KERNEL_SERVICE
static struct platform_device lg_diag_cmd_device = {
	.name = "lg_diag_cmd",
	.id = -1,
	.dev    = {
		.platform_data = 0, //&lg_diag_cmd_pdata
	},
}; 
#endif

// share cmd_pkt and length bw kernel and userspace
#if defined (CONFIG_LGE_DIAG_KERNEL_SERVICE)
static struct diagcmd_platform_data lg_fw_diagcmd_pdata = {
	.name = "lg_fw_diagcmd",
};

static struct platform_device lg_fw_diagcmd_device = {
	.name = "lg_fw_diagcmd",
	.id = -1,
	.dev    = {
		.platform_data = &lg_fw_diagcmd_pdata
	},
};
#endif

#ifdef CONFIG_LGE_AUDIO_HEADSET
static struct gpio_h2w_platform_data alohav_h2w_data = {
	.gpio_detect = 152,
	.gpio_button_detect = 37,
#ifdef CONFIG_LGE_AUDIO_HEADSET_PROTECT
	.gpio_mic_bias_en = 82
#endif 	
};

static struct platform_device alohav_h2w_device = {
	.name = "gpio-h2w",
	.id = -1,
	.dev = {
		.platform_data = &alohav_h2w_data,
	},
};
#endif /*CONFIG_LGE_AUDIO_HEADSET*/


static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

#if defined(CONFIG_USB_MSM_OTG_72K) || defined(CONFIG_USB_EHCI_MSM)
static int msm_hsusb_rpc_phy_reset(void __iomem *addr)
{
		return msm_hsusb_phy_reset();
}
#endif

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
#if defined(CONFIG_USB_MSM_OTG_72K) || defined(CONFIG_USB_EHCI_MSM)
	.phy_reset				 = msm_hsusb_rpc_phy_reset, 
#endif
	.pmic_notif_init         = msm_pm_app_rpc_init,
	.pmic_notif_deinit       = msm_pm_app_rpc_deinit,
	.pmic_register_vbus_sn   = msm_pm_app_register_vbus_sn,
	.pmic_unregister_vbus_sn = msm_pm_app_unregister_vbus_sn,
	.pmic_enable_ldo         = msm_pm_app_enable_usb_ldo,
	.pemp_level              = PRE_EMPHASIS_WITH_10_PERCENT,
	.cdr_autoreset           = CDR_AUTO_RESET_DEFAULT,
	.drv_ampl                = HS_DRV_AMPLITUDE_5_PERCENT,
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;

static struct platform_device *early_devices[] __initdata = {
#ifdef CONFIG_GPIOLIB
	&msm_gpio_devices[0],
	&msm_gpio_devices[1],
	&msm_gpio_devices[2],
	&msm_gpio_devices[3],
	&msm_gpio_devices[4],
	&msm_gpio_devices[5],
	&msm_gpio_devices[6],
	&msm_gpio_devices[7],
#endif
};

static struct platform_device *devices[] __initdata = {
	&msm_fb_device,

#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)
	&mddi_lgit_device,
	&ext_mddi_lgit_device,
#endif /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/

/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
	&mddi_toshiba_device,
	&smc91x_device,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	&android_pmem_kernel_ebi1_device,
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_smipool_device,
	&msm_device_nand,
	&msm_device_i2c,
/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
	&qsd_device_spi,
#endif
/*---------------------------------------------------------------------------*/
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID
	&mass_storage_device,
	&android_usb_device,
#endif
	&msm_device_tssc,
#if !defined(CONFIG_LGE_MACH_ENVT2)
	&msm_audio_device,
#endif
	&msm_device_uart_dm1,
	&msm_bluesleep_device,
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
#if !defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
	&msm_device_uart3,
#endif
#endif
	&msm_device_pmic_leds,
	&msm_device_kgsl,
	&hs_device,
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_S5K4E1GX
	&msm_camera_sensor_s5k4e1gx,
#endif
#ifdef CONFIG_AAT1270_FLASH
	&aat1270_flash_device,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#ifdef CONFIG_MT9P012_KM
	&msm_camera_sensor_mt9p012_km,
#endif
	&msm_batt_device,
// LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/03/19 {
#ifdef CONFIG_LGE_AUDIO_AMP_LM49250
	&android_vibrator_device, /* Motor */
#endif
// LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/03/19 }

/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.02 jinho.jang */
#if defined(CONFIG_LGE_KEYBOARD_QWERTY_PP2106)
	&qwerty_device,
#endif
/*---------------------------------------------------------------------------*/
		
/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.02 jinho.jang */
#if defined(CONFIG_LGE_ANDROID_HALL_IC)
	&android_hallic_device,
#endif

#if defined (CONFIG_LGE_DOCK_STATE_DETECT) && defined (CONFIG_LGE_MACH_ENVT2_REVD)
	&hallic_dock_device,
#endif 
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
#ifdef CONFIG_LGE_ATS_ETA_MTC 
#if 0
	&eve_atcmd_device, //vlc	
#endif
#endif /*CONFIG_LGE_ATS_ETA_MTC*/
/*---------------------------------------------------------------------------*/

#ifdef CONFIG_LGE_DIAG_KERNEL_SERVICE
  &lg_diag_cmd_device,
#endif

// share cmd_pkt and length bw kernel and userspace
#if defined (CONFIG_LGE_DIAG_KERNEL_SERVICE)
  &lg_fw_diagcmd_device,
#endif

#ifdef CONFIG_LGE_AUDIO_HEADSET
  &alohav_h2w_device,
#endif 

};

// LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/03/19 {
#ifdef CONFIG_LGE_AUDIO_PWM_VIBRATOR
static void __init vibrator_gpio_init(void)
{

    MSM_CLK_BASE = ioremap( MSM_CLK_PHYS , MSM_CLK_SIZE ) ;

	gpio_tlmm_config( GPIO_CFG(105, 2, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE ) ;
	gpio_tlmm_config( GPIO_CFG(157,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE ) ;
	gpio_configure( 157, GPIOF_DRIVE_OUTPUT);

    // sleep_current를 위해서 아래 gpio를 low로 한다.
	gpio_tlmm_config( GPIO_CFG(157,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE ) ;// sleep_current
	gpio_configure( 157, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(157, 0);

	gpio_tlmm_config( GPIO_CFG(105, 2, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE ) ;// sleep_current
	gpio_configure( 105, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(105, 0);

}
#endif
// LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/03/19 }


static void __init qsd8x50_init_irq(void)
{
	msm_init_irq();
	msm_init_sirc();
}

static void kgsl_phys_memory_init(void)
{
	request_mem_region(kgsl_resources[1].start,
		resource_size(&kgsl_resources[1]), "kgsl");
}

static void usb_mpp_init(void)
{
	unsigned rc;
	unsigned mpp_usb = 20;

	if (machine_is_qsd8x50_ffa()) {
		rc = mpp_config_digital_out(mpp_usb,
			MPP_CFG(MPP_DLOGIC_LVL_VDD,
				MPP_DLOGIC_OUT_CTRL_HIGH));
		if (rc)
			pr_err("%s: configuring mpp pin"
				"to enable 3.3V LDO failed\n", __func__);
	}
}

static void __init qsd8x50_init_usb(void)
{
	usb_mpp_init();

#ifdef CONFIG_USB_MSM_OTG_72K
	platform_device_register(&msm_device_otg);
#endif

#ifdef CONFIG_USB_FUNCTION_MSM_HSUSB
	platform_device_register(&msm_device_hsusb_peripheral);
#endif

#ifdef CONFIG_USB_MSM_72K
	platform_device_register(&msm_device_gadget_peripheral);
#endif

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa())
		return;

	vreg_usb = vreg_get(NULL, "boost");

	if (IS_ERR(vreg_usb)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_usb));
		return;
	}

	platform_device_register(&msm_device_hsusb_otg);
	msm_add_host(0, &msm_usb_host_pdata);
#ifdef CONFIG_USB_FS_HOST
	if (fsusb_gpio_init())
		return;
	msm_add_host(1, &msm_usb_host2_pdata);
#endif
}

static struct vreg *vreg_mmc;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
};

#ifdef CONFIG_LGE_MMC_DETECTION
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(GPIO_SD_DATA_3, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc1_dat_3"},
	{GPIO_CFG(GPIO_SD_DATA_2, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc1_dat_2"},
	{GPIO_CFG(GPIO_SD_DATA_1, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc1_dat_1"},
	{GPIO_CFG(GPIO_SD_DATA_0, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc1_dat_0"},
	{GPIO_CFG(GPIO_SD_CMD, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc1_cmd"},
	{GPIO_CFG(GPIO_SD_CLK, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc1_clk"},
};
#else
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc1_clk"},
};
#endif /*CONFIG_LGE_MMC_DETECTION*/

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc2_clk"},
	{GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_cmd"},
	{GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_3"},
	{GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_2"},
	{GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_1"},
	{GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_0"},
};

#if defined(CONFIG_MMC_MSM_SDC3_SUPPORT)
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	{GPIO_CFG(158, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_4"},
	{GPIO_CFG(159, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_5"},
	{GPIO_CFG(160, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_6"},
	{GPIO_CFG(161, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc3_dat_7"},
#endif
};
#endif

#if defined(CONFIG_MMC_MSM_SDC4_SUPPORT)
static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(142, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc4_clk"},
	{GPIO_CFG(143, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_cmd"},
	{GPIO_CFG(144, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_0"},
	{GPIO_CFG(145, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_1"},
	{GPIO_CFG(146, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_2"},
	{GPIO_CFG(147, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc4_dat_3"},
};
#endif

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
	},
#if defined(CONFIG_MMC_MSM_SDC3_SUPPORT)
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
	},
#endif
#if defined(CONFIG_MMC_MSM_SDC4_SUPPORT)
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
	},
#endif
};

static unsigned long vreg_sts, gpio_sts;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;
#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
	static int first_setup = 1;
#endif
	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	// 20100701 wo0gi[byoungwook.baek@lge.com], if wlan, skip the PMIC_VREG_SD_LEVEL setting [START]
	if( pdev->id == 2 )     // pdev->id (if 2 wlan, if 1 sdcard)
	{
		printk(KERN_INFO "This request is WLAN. Skip the PMIC_VREG_SD_LEVEL setting\n");
		return 0;
	}	
	// 20100701 wo0gi[byoungwook.baek@lge.com], if wlan, skip the PMIC_VREG_SD_LEVEL setting [END]
	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			rc = vreg_disable(vreg_mmc);
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
#ifdef CONFIG_LGE_MMC_DETECTION	
		rc = vreg_set_level(vreg_mmc,VREG_SD_LEVEL);
#else
		rc = vreg_set_level(vreg_mmc, PMIC_VREG_GP6_LEVEL);
#endif /*CONFIG_LGE_MMC_DETECTION*/
		if (!rc)
			rc = vreg_enable(vreg_mmc);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);

#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
	/* if first called related to sdcc1, irq should be registered as wakeup source
	 *      * cleaneye.kim@lge.com, 2010-02-19
	 *           */
	if (first_setup == 1) {
		struct mmc_platform_data *pdata = pdev->dev.platform_data;
		if (pdev->id == 1) {
			first_setup = 0;
			set_irq_wake(pdata->status_irq, 1);
			printk(KERN_INFO "%s set irq wake, status_irq : %d, on_off : %d\n",__func__, pdata->status_irq, 1);
		}
	}
#endif

	return 0;
}

#endif
#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

static int msm_sdcc_get_wpswitch(struct device *dv)
{
	void __iomem *wp_addr = 0;
	uint32_t ret = 0;
	struct platform_device *pdev;

	if (!(machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf()))
		return -1;

	pdev = container_of(dv, struct platform_device, dev);

	wp_addr = ioremap(FPGA_SDCC_STATUS, 4);
	if (!wp_addr) {
		pr_err("%s: Could not remap %x\n", __func__, FPGA_SDCC_STATUS);
		return -ENOMEM;
	}

	ret = (readl(wp_addr) >> ((pdev->id - 1) << 1)) & (0x03);
	pr_info("%s: WP/CD Status for Slot %d = 0x%x \n", __func__,
							pdev->id, ret);
	iounmap(wp_addr);
	return ((ret == 0x02) ? 1 : 0);

}
#endif

#ifdef CONFIG_LGE_MMC_DETECTION
static unsigned int envt2_sdcc_slot_status(struct device *dev)
{
	return !gpio_get_value(GPIO_SD_DETECT_N);
}

static void envt2_sdcc_gpio_init(void)
{
/* set up gpio for MMC */
	int rc = 0;
	if (gpio_request(GPIO_SD_DETECT_N, "sdc1_status_irq"))
		pr_err("failed to request gpio sdc1_status_irq\n");
	rc = gpio_tlmm_config(GPIO_CFG(GPIO_SD_DETECT_N, 0, GPIO_INPUT, GPIO_PULL_UP,
									GPIO_2MA), GPIO_ENABLE);
	if (rc)
		printk(KERN_ERR "%s: Failed to configure GPIO %d\n",
					__func__, rc);

	/* SDC1 GPIOs */
	if (gpio_request(GPIO_SD_DATA_3, "sdc1_data_3"))
		pr_err("failed to request gpio sdc1_data_3\n");
	if (gpio_request(GPIO_SD_DATA_2, "sdc1_data_2"))
		pr_err("failed to request gpio sdc1_data_2\n");
	if (gpio_request(GPIO_SD_DATA_1, "sdc1_data_1"))
		pr_err("failed to request gpio sdc1_data_1\n");
	if (gpio_request(GPIO_SD_DATA_0, "sdc1_data_0"))
		pr_err("failed to request gpio sdc1_data_0\n");
	if (gpio_request(GPIO_SD_CMD, "sdc1_cmd"))
		pr_err("failed to request gpio sdc1_cmd\n");
	if (gpio_request(GPIO_SD_CLK, "sdc1_clk"))
		pr_err("failed to request gpio sdc1_clk\n");
}
#endif /*CONFIG_LGE_MMC_DETECTION*/

// wo0gi added [START]
#if defined(CONFIG_LGE_BCM432X_PATCH)
static unsigned int bcm432x_sdcc_wlan_slot_status(struct device *dev)
{
	return gpio_get_value(CONFIG_BCM4329_GPIO_WL_RESET);
}

static struct mmc_platform_data bcm432x_sdcc_wlan_data = {
	.ocr_mask   = MMC_VDD_30_31,
#ifdef CONFIG_MACH_ENVT2		
	.translate_vdd  = NULL,
#else
	.translate_vdd  = msm_sdcc_setup_power,
#endif
	.status     = bcm432x_sdcc_wlan_slot_status,
	.status_irq = MSM_GPIO_TO_INT(CONFIG_BCM4329_GPIO_WL_RESET),
	.irq_flags      = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
// 20100919 byoungwook.baek@lge.com modify sdio clock 50->25 
	.msmsdcc_fmax	= 25000000,
//	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif  /* CONFIG_LGE_BCM432X_PATCH*/
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data qsd8x50_sdc1_data = {
#ifdef CONFIG_LGE_MMC_DETECTION
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.status 		= envt2_sdcc_slot_status,
	.status_irq    = MSM_GPIO_TO_INT(GPIO_SD_DETECT_N),
	.irq_flags		= IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#else
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
#endif /*CONFIG_LGE_MMC_DETECTION*/
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
	// remove wpswitch, only qsd8x50 and 7x30 uses
#else
	.wpswitch	= msm_sdcc_get_wpswitch,
#endif
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
// 20100424  [wo0gi]byoungwook.baek@lge.com for Wifi board file setting [START]
#ifndef CONFIG_LGE_BCM432X_PATCH
static struct mmc_platform_data qsd8x50_sdc2_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch	= msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif /* CONFIG_LGE_BCM432X_PATCH */
// 20100424  [wo0gi]byoungwook.baek@lge.com for Wifi board file setting [END]
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data qsd8x50_sdc3_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
#ifdef CONFIG_MMC_MSM_SDC3_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data qsd8x50_sdc4_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch	= msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

static void __init qsd8x50_init_mmc(void)
{
#ifdef CONFIG_LGE_MMC_DETECTION
	vreg_mmc = vreg_get(NULL, "wlan");
#else
	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa())
		vreg_mmc = vreg_get(NULL, "gp6");
	else
		vreg_mmc = vreg_get(NULL, "gp5");
#endif /*CONFIG_LGE_MMC_DETECTION*/

	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

#ifdef CONFIG_LGE_MMC_DETECTION
	envt2_sdcc_gpio_init();
#endif /*CONFIG_LGE_MMC_DETECTION*/

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	msm_add_sdcc(1, &qsd8x50_sdc1_data);
#endif

	if (machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf()) {
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
// 20100424  [wo0gi]byoungwook.baek@lge.com for Wifi board file setting [START]
#if defined(CONFIG_LGE_BCM432X_PATCH)
	gpio_request(CONFIG_BCM4329_GPIO_WL_RESET, "wlan_cd");
	//bcm432x_sdcc_wlan_data.status_irq = gpio_to_irq(CONFIG_BCM4329_GPIO_WL_RESET);
	//bcm432x_sdcc_wlan_data.status_irq = MSM_GPIO_TO_INT(CONFIG_BCM4329_GPIO_WL_RESET);
	msm_add_sdcc(2, &bcm432x_sdcc_wlan_data);
#else /* qualcomm or google */
		msm_add_sdcc(2, &qsd8x50_sdc2_data);
#endif
// 20100424  [wo0gi]byoungwook.baek@lge.com for Wifi board file setting [END]
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
		msm_add_sdcc(3, &qsd8x50_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
		msm_add_sdcc(4, &qsd8x50_sdc4_data);
#endif
	}

}

/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
static void __init qsd8x50_cfg_smc91x(void)
{
	int rc = 0;

	if (machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf()) {
		smc91x_resources[0].start = 0x70000300;
		smc91x_resources[0].end = 0x700003ff;
		smc91x_resources[1].start = MSM_GPIO_TO_INT(156);
		smc91x_resources[1].end = MSM_GPIO_TO_INT(156);
	} else if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		smc91x_resources[0].start = 0x84000300;
		smc91x_resources[0].end = 0x840003ff;
		smc91x_resources[1].start = MSM_GPIO_TO_INT(87);
		smc91x_resources[1].end = MSM_GPIO_TO_INT(87);

		rc = gpio_tlmm_config(GPIO_CFG(87, 0, GPIO_INPUT,
					       GPIO_PULL_DOWN, GPIO_2MA),
					       GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",
					__func__, rc);
		}
	} else
		printk(KERN_ERR "%s: invalid machine type\n", __func__);
}
#endif
/*---------------------------------------------------------------------------*/

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		gpio_scl = 60;
		gpio_sda = 61;
	} else {
		gpio_scl = 95;
		gpio_sda = 96;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
#ifdef CONFIG_LGE_CAMERA_CONFIG 
	.clk_freq = 400000,
#else
	.clk_freq = 100000,
#endif //CONFIG_LGE_CAMERA_CONFIG
	.rsl_id = SMEM_SPINLOCK_I2C,
	.pri_clk = 95,
	.pri_dat = 96,
	.aux_clk = 60,
	.aux_dat = 61,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(95, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(96, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
	if (gpio_request(60, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(61, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");

	msm_i2c_pdata.rmutex = (uint32_t)smem_alloc(SMEM_I2C_MUTEX, 8);
	msm_i2c_pdata.pm_lat =
		msm_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

#ifdef CONFIG_LGE_AUDIO_HEADSET
static void __init Headset_Gpio_init(void)
{
		int  ret;
		
		ret = gpio_tlmm_config(GPIO_CFG(152,  0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
		if (ret) {
				printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",__func__, ret);
		}
		ret = gpio_tlmm_config(GPIO_CFG(37,  0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
		if (ret) {
				printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",__func__, ret);
		}

	
		gpio_configure(152, GPIOF_INPUT);

		gpio_configure(37, GPIOF_INPUT);
#ifdef CONFIG_LGE_AUDIO_HEADSET_PROTECT

		ret = gpio_tlmm_config( GPIO_CFG(82, 0, GPIO_OUTPUT,GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE ) ;// sleep_current
		if (ret) {
						printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",__func__, ret);
		}

		gpio_configure( 82, GPIOF_DRIVE_OUTPUT);
		gpio_set_value(82, 0);  
#endif 
		
}

#endif /*CONFIG_LGE_AUDIO_HEADSET*/

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static unsigned pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;
static void __init pmem_kernel_smi_size_setup(char **p)
{
	pmem_kernel_smi_size = memparse(*p, p);

	/* Make sure that we don't allow more SMI memory then is
	   available - the kernel mapping code has no way of knowing
	   if it has gone over the edge */

	if (pmem_kernel_smi_size > MSM_PMEM_SMIPOOL_SIZE)
		pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;
}
__early_param("pmem_kernel_smi_size=", pmem_kernel_smi_size_setup);
#endif

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static void __init pmem_sf_size_setup(char **p)
{
	pmem_sf_size = memparse(*p, p);
}
__early_param("pmem_sf_size=", pmem_sf_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);


static unsigned audio_size = MSM_AUDIO_SIZE;
static void __init audio_size_setup(char **p)
{
	audio_size = memparse(*p, p);
}
__early_param("audio_size=", audio_size_setup);

static void __init qsd8x50_init(void)
{
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
	msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);
	platform_add_devices(early_devices, ARRAY_SIZE(early_devices));

/*---------------------------------------------------------------------------*/
/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
	qsd8x50_cfg_smc91x();
#endif

	msm_acpu_clock_init(&qsd8x50_clock_data);

	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;

	msm_gadget_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
#ifdef CONFIG_LGE_RAM_CONSOLE
	lge_add_ramconsole_devices();
#endif
#ifdef CONFIG_LGE_ERS
	lge_add_ers_devices();
	lge_add_panic_handler_devices();
#endif

#ifdef CONFIG_LGE_MACH_ENVT2_REVC//2010.03.19 woochang.chun
	bh6172_init_i2c_subpm();
	ce147_init_i2c_camisp();
#endif //CONFIG_MACH_ENVT2_REVC
	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_fb_add_devices();
#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
	qsd8x50_init_usb();
	qsd8x50_init_mmc();
	bt_power_init();
/* LG_FW : 2010.02.06 jinho.jang - Not Supported  */
#if !defined(CONFIG_LGE_MACH_ENVT2)
	audio_gpio_init();
#endif
	msm_device_i2c_init();

#ifdef CONFIG_LGE_AUDIO_HEADSET
	Headset_Gpio_init();
#endif 

// LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/03/19 {
#ifdef CONFIG_LGE_AUDIO_AMP_LM49250
	vibrator_gpio_init();
#endif
// LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/03/19 }

#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)
	mddi_panel_power_init();
#endif

#if defined(CONFIG_LGE_MACH_ENVT2)
	printk(KERN_ERR "%s: i2c_register_board_info, for #0\n",__func__);
	i2c_register_board_info(0, msm_i2c_board_info,
			ARRAY_SIZE(msm_i2c_board_info));	
#ifdef CONFIG_S5K4E1GX 
	printk(KERN_ERR "%s: i2c_register_board_info, for #1\n",__func__);
	i2c_register_board_info(1, s5k4e1gx_i2c_bdinfo, 
		ARRAY_SIZE(s5k4e1gx_i2c_bdinfo));
#endif 
#else /*origin*/
	msm_qsd_spi_init();
	i2c_register_board_info(0, msm_i2c_board_info,
				ARRAY_SIZE(msm_i2c_board_info));
	spi_register_board_info(msm_spi_board_info,
				ARRAY_SIZE(msm_spi_board_info));
#endif

#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
	msm_qsd_spi_init();
	init_spi_touch();

#endif
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	kgsl_phys_memory_init();

#ifdef CONFIG_SURF_FFA_GPIO_KEYPAD
	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa())
		platform_device_register(&keypad_device_8k_ffa);
	else
		platform_device_register(&keypad_device_surf);
#endif

/* LG_FW : 2010.02.02 jinho.jang */
#if defined(CONFIG_LGE_MACH_ENVT2)
	/* gpio-i2c related functions */
	vs760_init_gpio_i2c_devices();
#endif
}

static void __init qsd8x50_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	size = pmem_kernel_smi_size;
	if (size > MSM_PMEM_SMIPOOL_SIZE) {
		printk(KERN_ERR "pmem kernel smi arena size %lu is too big\n",
			size);

		size = MSM_PMEM_SMIPOOL_SIZE;
	}

	android_pmem_kernel_smi_pdata.start = MSM_PMEM_SMIPOOL_BASE;
	android_pmem_kernel_smi_pdata.size = size;

	pr_info("allocating %lu bytes at %lx (%lx physical)"
		"for pmem kernel smi arena\n", size,
		(long unsigned int) MSM_PMEM_SMIPOOL_BASE,
		__pa(MSM_PMEM_SMIPOOL_BASE));
#endif

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}


	size = MSM_FB_SIZE;
	addr = (void *)MSM_FB_BASE;
	msm_fb_resources[0].start = (unsigned long)addr;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("using %lu bytes of SMI at %lx physical for fb\n",
	       size, (unsigned long)addr);

#if !defined(CONFIG_LGE_MACH_ENVT2)
	size = audio_size ? : MSM_AUDIO_SIZE;
	addr = alloc_bootmem(size);
	msm_audio_resources[0].start = __pa(addr);
	msm_audio_resources[0].end = msm_audio_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for audio\n",
		size, addr, __pa(addr));
#endif
}

static void __init qsd8x50_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_qsd8x50_io();
	qsd8x50_allocate_memory_regions();
}

MACHINE_START(QSD8X50_SURF, "QCT QSD8X50 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = qsd8x50_map_io,
	.init_irq = qsd8x50_init_irq,
	.init_machine = qsd8x50_init,
	.timer = &msm_timer,
MACHINE_END

/*LG_FW : 2010.02.06 jinho.jang - LGE Change */
#if !defined(CONFIG_LGE_MACH_ENVT2)
MACHINE_START(QSD8X50_FFA, "QCT QSD8X50 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = qsd8x50_map_io,
	.init_irq = qsd8x50_init_irq,
	.init_machine = qsd8x50_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(QSD8X50A_SURF, "QCT QSD8X50A SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = qsd8x50_map_io,
	.init_irq = qsd8x50_init_irq,
	.init_machine = qsd8x50_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(QSD8X50A_FFA, "QCT QSD8X50A FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = qsd8x50_map_io,
	.init_irq = qsd8x50_init_irq,
	.init_machine = qsd8x50_init,
	.timer = &msm_timer,
MACHINE_END
#endif
