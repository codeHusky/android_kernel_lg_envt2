/* drivers/video/msm/src/panel/mddi/mddi_innotek.c
 *
 * Copyright (C) 2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <asm/gpio.h>
#include <mach/vreg.h>

/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
/******************************************************************************/
#define CONFIG_LGE_FB_MSM_24BIT //CONFIG_LGE_FB_MSM_FRAMEBUF_24
#define FEATURE_SWITCH_TYPE1_TYPE2
/******************************************************************************/

#define LGIT_WVGA_SECD 1

#define PANEL_DEBUG 0

#define SUB_LCD_RESET_N		101

static boolean is_slcd_on = TRUE;

/* The comment from AMSS codes:
 * Dot clock (10MHz) / pixels per row (320) = rows_per_second
 * Rows Per second, this number arrived upon empirically 
 * after observing the timing of Vsync pulses
 * XXX: TODO: change this values for lgit PANEL */

extern boolean mddi_vsync_detect_enabled;

static msm_fb_vsync_handler_type mddi_lgit_vsync_handler = NULL;
static void *mddi_lgit_vsync_handler_arg;
static uint16 mddi_lgit_vsync_attempts;

static struct msm_panel_common_pdata *mddi_lgit_pdata;

static int mddi_lgit_lcd_on(struct platform_device *pdev);
static int mddi_lgit_lcd_off(struct platform_device *pdev);

//LG_FW : jinho.jang - charging display check
#ifndef CONFIG_CHARGING_DISP
static int is_charging_mode;

int get_charging_disp(void)
{
	return is_charging_mode;
}

void set_charging_disp(int charging)
{
	if(charging)
	is_charging_mode = 1;
	else
	is_charging_mode = 0;
}
#endif

static int mddi_lgit_lcd_init(void);

#define DEBUG 1
#if DEBUG
#define EPRINTK(fmt, args...) printk(fmt, ##args)
#else
#define EPRINTK(fmt, args...) do { } while (0)
#endif

struct display_table {
    unsigned reg;
    unsigned char count;
    unsigned char val_list[33];
};

#define REGFLAG_DELAY             0XFFFE
#define REGFLAG_END_OF_TABLE      0xFFFF   // END OF REGISTERS MARKER

/* lcd on  register intialization */
#ifdef LCD_SLEEP_MODE
static struct display_table mddi_lgit_display_sleep_out[] = {

	/* Sleep-Out Sequence */
	{0x11, 1, {0x00}}, //sleep out	
	{0x29, 1, {0x00}}, //display on	
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct display_table mddi_lgit_display_sleep_in[] = {

	/* Sleep-In Sequence */
	{0x28, 1, {0x00}}, //display off	
	{0x10, 1, {0x00}}, //sleep in
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct display_table mddi_lgit_display_off_data[] = {

	/* Deep Standby-In Sequence */
	//{0x34, 1, {0x00}}, //tearing effect off
	{0x10, 1, {0x00}}, //sleep in
	{REGFLAG_DELAY, 80, {}},
	{0xC1, 1, {0x01}}, //Deep Standby Mode In
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct display_table mddi_lgit_display_on_data_3p5[] = {
 	/* Display Mode Setting */
    {0x20, 1, {0x00}}, 			//Display Inversion
    {0x35, 1, {0x00}}, //tearing effect on
    {0x36, 1, {0x03}},			//display 180 degree rotation FV MX 1	
	{0x44, 5, {0x03, 0x00, 0x00, 0x00, 0x16}},	

    {0xB2, 5, {0x20, 0x00, 0x00, 0x00, 0xC8}},				//Panel Characteristics Setting	
    {0xB3, 1, {0x02}},				//Panel Drive Setting    
	{0xB4, 1, {0x04}},				//Display Mode Control 	
    {0xB5, 17, {0x42, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20}},	//Display Control 1    
    {0xB6, 21, {0x05, 0x00, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x0F}}, //Display Control 2
	{0xB7, 17, {0x49, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, //Display Control 3
	
	/* Power Setting */
    {0xC0, 5, {0x01, 0x00, 0x00, 0x00, 0x11}},			//Internal Oscillator Setting*/
    {0xC1, 1, {0x00}},
    {0xC3, 17, {0x07, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04}},	//Power Control 3    
    {0xC4, 21, {0x12, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x76}}, //Power Control 4
    {0xC5, 1, {0x68/*0x74*/}},					//Power Control 5
    {0xC6, 5, {0x41, 0x00, 0x00, 0x00, 0x63}},			//Power Control 6

	/* Gamma Setting */
	{0xD0, 33, {0x22, 0x00, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x04/*0x06*/, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}},	//Positive Gamma Curve for Red
	{0xD1, 33, {0x22, 0x00, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x04/*0x06*/, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}},	//Negative Gamma Curve for Red
	{0xD2, 33, {0x22, 0x00, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x04/*0x06*/, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}},	//Positive Gamma Curve for Green
	{0xD3, 33, {0x22, 0x00, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x04/*0x06*/, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}},	//Negative Gamma Curve for Green
	{0xD4, 33, {0x22, 0x00, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x04/*0x06*/, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}},	//Positive Gamma Curve for Blue
	{0xD5, 33, {0x22, 0x00, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x04/*0x06*/, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}},	//Negative Gamma Curve for Blue

	/* DATA1_RESET Delay change */
	/* LG_FW : 2010.06.15 jinho.jang - MDDI_2 issue */
	{0xE0, 21, {0x30, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00}},
	//{0xE0, 21, {0x30, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x05}},
	/* Sleep out */
	{0x11, 1, {0x00}}, 			//Sleep Out
	{REGFLAG_DELAY, 120, {}}, //7 Frames or more
	
	/* Display On */
	{0x29, 1, {0x00}}, 			//Display On
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void slcd_display_table(struct display_table *table, unsigned int count)
{
	unsigned int i;

    for(i = 0; i < count; i++) 
	{
        unsigned reg;

        reg = table[i].reg;
        switch (reg) 
		{
            case REGFLAG_DELAY:
                mdelay(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE:
                break;
            default:
                mddi_host_register_cmd_write(
                        reg,
                        table[i].count,
                        table[i].val_list,
                        0,
                        0,
                        0);
        }

    }
	
}



static void mddi_lgit_vsync_set_handler(msm_fb_vsync_handler_type handler,	/* ISR to be executed */
					 void *arg)
{
	boolean error = FALSE;
	unsigned long flags;	

	/* Disable interrupts */
	spin_lock_irqsave(&mddi_host_spin_lock, flags);
	// INTLOCK();

	if (mddi_lgit_vsync_handler != NULL) {
		error = TRUE;
	} else {
		/* Register the handler for this particular GROUP interrupt source */
		mddi_lgit_vsync_handler = handler;
		mddi_lgit_vsync_handler_arg = arg;
	}

	/* Restore interrupts */
	spin_unlock_irqrestore(&mddi_host_spin_lock, flags);
	// MDDI_INTFREE();
	if (error) {
		//MDDI_MSG_ERR("MDDI: Previous Vsync handler never called\n");
		//printk("MDDI: Previous Vsync handler never called\n");
	} else {
		/* TODO: Enable the vsync wakeup */

		mddi_lgit_vsync_attempts = 1;
		mddi_vsync_detect_enabled = TRUE;
	}
}

void mdd_lgit_reset_high(void)
{
	gpio_set_value(SUB_LCD_RESET_N, 1);
}

void mddi_lgit_reset_low(void)
{
	gpio_set_value(SUB_LCD_RESET_N, 0);
}
EXPORT_SYMBOL(mdd_lgit_reset_high);
EXPORT_SYMBOL(mddi_lgit_reset_low);


void mddi_lgit_sec_lcd_reset(void)
{
	mddi_lgit_reset_low();
	mdelay(10);
	mdd_lgit_reset_high();
	mdelay(20);
}

static int mddi_boot_mode = 1;

static int mddi_lgit_lcd_on(struct platform_device *pdev)
{
	int panel_number = 0;
	int reg = 0;
	
	panel_number = mddi_lgit_pdata->panel_num();	

    EPRINTK("%s: started. panel_number=%d ,is_slcd_on=%d\n", __func__,panel_number,is_slcd_on);


	if(mddi_boot_mode)
	{
	/* LG_FW: 2010.05.12 jinho.jang - initialization at bootloader */
	#ifdef CONFIG_LGE_BOOTLOADER_DISP_INIT
		//mddi_lgit_sec_lcd_reset();
		slcd_display_table(mddi_lgit_display_on_data_3p5,
					sizeof(mddi_lgit_display_on_data_3p5) / sizeof(struct display_table));			
	#endif

		mddi_boot_mode = 0;
	
		return 0;
	}
	
	if((panel_number == MDDI_PANEL) && (!is_slcd_on))
	{
		
		mddi_lgit_sec_lcd_reset();
		slcd_display_table(mddi_lgit_display_on_data_3p5,
				sizeof(mddi_lgit_display_on_data_3p5) / sizeof(struct display_table));

		
/* LG_FW : 2010.08.05 jinho.jang - MDDI TYPE1 and TYPE2 Switch */
#ifdef FEATURE_SWITCH_TYPE1_TYPE2
		mddi_change_to_type_2(MDDI_HOST_PRIM);
#endif

		is_slcd_on = TRUE;
	} 
    else 
    {
        EPRINTK("%s: strange status!!!!!!!!!!!!!!!!!!\n", __func__);

    }
    EPRINTK("%s: return\n", __func__);
	return 0;
}


static int mddi_lgit_lcd_off(struct platform_device *pdev)
{
    EPRINTK("%s: started. is_slcd_on=%d\n", __func__,is_slcd_on);

	if(is_slcd_on)
	{
		
		slcd_display_table(mddi_lgit_display_off_data,
				sizeof(mddi_lgit_display_off_data) / sizeof(struct display_table));

		//mddi_lgit_reset_low();

		is_slcd_on = FALSE;
	}
    else 
    {
        EPRINTK("%s: strange status!!!!!!!!!!!!!!!!!!\n", __func__);

    }
    EPRINTK("%s: return\n", __func__);
	
	return 0;
}

void mddi_remote_lgit_lcd_off(void)
{
    EPRINTK("%s: started. is_slcd_on=%d\n", __func__,is_slcd_on);
	if(is_slcd_on)
	{
		
		slcd_display_table(mddi_lgit_display_off_data,
				sizeof(mddi_lgit_display_off_data) / sizeof(struct display_table));

		//mddi_lgit_reset_low();

		is_slcd_on = FALSE;
	}
    else 
    {
        EPRINTK("%s: strange status!!!!!!!!!!!!!!!!!!\n", __func__);

    }
    EPRINTK("%s: return\n", __func__);
	return;
}
EXPORT_SYMBOL(mddi_remote_lgit_lcd_off);

ssize_t mddi_lgit_lcd_show_onoff(struct device *dev,  struct device_attribute *attr, char *buf)
{
	EPRINTK("%s : strat\n", __func__);
	return 0;
}

extern void slcd_backlight_set_level(int level);

ssize_t mddi_lgit_lcd_store_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff; // = simple_strtol(buf, NULL, count);

	if(!is_touch_initialzed())
		return 0;
	
	EPRINTK("%s : strat\n", __func__);	
	sscanf(buf, "%d", &onoff);

	EPRINTK("%s: buf %s, onoff : %d\n", __func__,buf, onoff);
	
	if(onoff) {
		if(!is_slcd_on)
		{
			mddi_lgit_sec_lcd_reset();
			slcd_display_table(mddi_lgit_display_on_data_3p5,
							sizeof(mddi_lgit_display_on_data_3p5) / sizeof(struct display_table));

			is_slcd_on = TRUE;
		}
	}

	else {
		if(is_slcd_on)
		{
			slcd_backlight_set_level(0);

			slcd_display_table(mddi_lgit_display_off_data,
					sizeof(mddi_lgit_display_off_data) / sizeof(struct display_table));

			//mddi_lgit_reset_low();

			is_slcd_on = FALSE;
		}
	}
	return 0;
}

DEVICE_ATTR(mddi_lcd_onoff, 0666, mddi_lgit_lcd_show_onoff, mddi_lgit_lcd_store_onoff);

//LG_FW : jinho.jang - charging display check
#ifndef CONFIG_CHARGING_DISP
ssize_t mddi_lgit_lcd_show_charging_disp(struct device *dev,  struct device_attribute *attr, char *buf)
{
	EPRINTK("%s : strat\n", __func__);
	return 0;
}

ssize_t mddi_lgit_lcd_store_charging_disp(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int onoff;	
	
	EPRINTK("%s : strat\n", __func__);	
	sscanf(buf, "%d", &onoff);

	EPRINTK("%s: buf %s, onoff : %d\n", __func__,buf, onoff);	

	set_charging_disp(onoff);
	
	return 0;
}

DEVICE_ATTR(mddi_lcd_charging_disp, 0666, mddi_lgit_lcd_show_charging_disp, mddi_lgit_lcd_store_charging_disp);
#endif

static void mddi_lgit_set_backlight_board(struct msm_fb_data_type *mfd) 
{
	int level;	

	level=(int)mfd->bl_level;
	mddi_lgit_pdata->backlight_level(level, 0, 0);
}

struct msm_fb_panel_data lgit_panel_data0 = {
	.on = mddi_lgit_lcd_on,
	.off = mddi_lgit_lcd_off,
	.set_backlight = mddi_lgit_set_backlight_board,
	.set_vsync_notifier = mddi_lgit_vsync_set_handler,
};

static struct platform_device this_device_0 = {
	.name   = "mddi_lgit_wvga",
	.id	= LGIT_WVGA_SECD,
	.dev	= {
		.platform_data = &lgit_panel_data0,
	}
};

static int __init mddi_lgit_lcd_probe(struct platform_device *pdev)
{
	int err;

	if (pdev->id == 0) {
		mddi_lgit_pdata = pdev->dev.platform_data;	
		return 0;
	}
	
	/* configure lcd reset gpio */
	mddi_lgit_pdata->panel_config_gpio(TRUE);

	msm_fb_add_device(pdev);

	err = device_create_file(&pdev->dev, &dev_attr_mddi_lcd_onoff);

//LG_FW : jinho.jang - charging display check
#ifndef CONFIG_CHARGING_DISP
	err = device_create_file(&pdev->dev, &dev_attr_mddi_lcd_charging_disp);

	//default setting is non-charging
	set_charging_disp(0);
#endif

	return err;
}

static struct platform_driver this_driver = {
	.probe  = mddi_lgit_lcd_probe,
	.driver = {
		.name   = "mddi_lgit_wvga",
	},
};

static int mddi_lgit_lcd_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	ret = platform_driver_register(&this_driver);
	if (!ret) 
	{
		pinfo = &lgit_panel_data0.panel_info;
		EPRINTK("%s: setting up panel info.\n", __func__);
		pinfo->xres = 480;
		pinfo->yres = 800;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
#if defined(CONFIG_LGE_FB_MSM_24BIT) //32bit
		pinfo->bpp = 24;
#else
		pinfo->bpp = 16;
#endif
		// vsync config
		pinfo->lcd.vsync_enable = TRUE;
		pinfo->lcd.refx100 = 5962;//6800;
		pinfo->lcd.v_back_porch = 8;
		pinfo->lcd.v_front_porch = 10;
		pinfo->lcd.v_pulse_width = 0;
		pinfo->lcd.hw_vsync_mode = TRUE;
		pinfo->lcd.vsync_notifier_period = (1 * HZ);
/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
#if defined(CONFIG_LGE_FB_MSM_24BIT) //32bit
		pinfo->clk_rate = 192000000;//122880000;//96000000;//192000000;
		pinfo->clk_min = 190000000;//120000000;//90000000;//190000000;
		pinfo->clk_max = 200000000;//125000000;//100000000;//200000000;
#else
		pinfo->clk_rate = 96000000;//192000000;
		pinfo->clk_min = 90000000;//190000000;
		pinfo->clk_max = 100000000;//200000000;
#endif

		
		pinfo->fb_num = 2;

		pinfo->bl_max = 22;
		pinfo->bl_min = 1;

		ret = platform_device_register(&this_device_0);

		if (ret)
			platform_driver_unregister(&this_driver);
	}

	return ret;
}

module_init(mddi_lgit_lcd_init);

