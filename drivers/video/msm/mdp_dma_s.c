/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>

#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include "mdp.h"
#include "msm_fb.h"
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)
#include "mddihost.h"
#endif

/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
/******************************************************************************/
#define CONFIG_LGE_FB_MSM_24BIT //CONFIG_LGE_FB_MSM_FRAMEBUF_24
//#define FEATURE_SWITCH_TYPE1_TYPE2
/******************************************************************************/

/* LG_FW : 2010.03.29 jiho.jang - Dual LCD Switch */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)
int vsync_start_y_adjust_s = 4;
#endif

static void mdp_dma_s_update_lcd(struct msm_fb_data_type *mfd)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	int mddi_dest = FALSE;
	uint32 outBpp = iBuf->bpp;
	uint32 dma_s_cfg_reg;
	uint8 *src;
	struct msm_fb_panel_data *pdata =
	    (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
#if defined(CONFIG_LGE_FB_MSM_24BIT) //32bit
	dma_s_cfg_reg = DMA_PACK_ALIGN_LSB |
	    DMA_OUT_SEL_AHB | DMA_IBUF_NONCONTIGUOUS;
#else /*origin*/
	dma_s_cfg_reg = DMA_PACK_TIGHT | DMA_PACK_ALIGN_LSB |
	    DMA_OUT_SEL_AHB | DMA_IBUF_NONCONTIGUOUS;
#endif

	if (mfd->fb_imgType == MDP_BGR_565)
		dma_s_cfg_reg |= DMA_PACK_PATTERN_BGR;

/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA) //32bit
	else if (mfd->fb_imgType == MDP_RGBA_8888)
		dma_s_cfg_reg |= DMA_PACK_PATTERN_BGR;
#endif

	else
		dma_s_cfg_reg |= DMA_PACK_PATTERN_RGB;

	if (outBpp == 4)
		dma_s_cfg_reg |= DMA_IBUF_C3ALPHA_EN;

	if (outBpp == 2)
		dma_s_cfg_reg |= DMA_IBUF_FORMAT_RGB565;

/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA) //32bit
	else if (outBpp == 4)
		dma_s_cfg_reg |= DMA_IBUF_FORMAT_xRGB8888_OR_ARGB8888;
#endif

#if !defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)
	if (mfd->panel_info.pdest != DISPLAY_2) {
		printk(KERN_ERR "error: non-secondary type through dma_s!\n");
		return;
	}
#endif

	if (mfd->panel_info.type == MDDI_PANEL ||
		mfd->panel_info.type == EXT_MDDI_PANEL) {
		dma_s_cfg_reg |= DMA_OUT_SEL_MDDI;
		mddi_dest = TRUE;
	} else {
		dma_s_cfg_reg |= DMA_AHBM_LCD_SEL_SECONDARY;
		outp32(MDP_EBI2_LCD1, mfd->data_port_phys);
	}

	dma_s_cfg_reg |= DMA_DITHER_EN;

	src = (uint8 *) iBuf->buf;
	/* starting input address */
	src += (iBuf->dma_x + iBuf->dma_y * iBuf->ibuf_width) * outBpp;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	/* PIXELSIZE */
	if (mfd->panel_info.type == MDDI_PANEL) {
		MDP_OUTP(MDP_BASE + 0xa0004,
			(iBuf->dma_h << 16 | iBuf->dma_w));
		MDP_OUTP(MDP_BASE + 0xa0008, src);	/* ibuf address */
		MDP_OUTP(MDP_BASE + 0xa000c,
			iBuf->ibuf_width * outBpp);/* ystride */
	} else {
		MDP_OUTP(MDP_BASE + 0xb0004,
			(iBuf->dma_h << 16 | iBuf->dma_w));
		MDP_OUTP(MDP_BASE + 0xb0008, src);	/* ibuf address */
		MDP_OUTP(MDP_BASE + 0xb000c,
			iBuf->ibuf_width * outBpp);/* ystride */
	}

	if (mfd->panel_info.bpp == 18) {
		dma_s_cfg_reg |= DMA_DSTC0G_6BITS |	/* 666 18BPP */
		    DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;

/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA) //32bit
	} 
	else if(mfd->panel_info.bpp == 24) {		
		dma_s_cfg_reg |= DMA_DSTC0G_8BITS | /* 888 24BPP */ 
		DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS; 
	}
	else {
#else /*origin*/
	} else {
#endif /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/
		dma_s_cfg_reg |= DMA_DSTC0G_6BITS |	/* 565 16BPP */
		    DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
	}

	if (mddi_dest) {
		if (mfd->panel_info.type == MDDI_PANEL) {
			MDP_OUTP(MDP_BASE + 0xa0010,
				(iBuf->dma_y << 16) | iBuf->dma_x);
			MDP_OUTP(MDP_BASE + 0x00090, 1);
		} else {
			MDP_OUTP(MDP_BASE + 0xb0010,
				(iBuf->dma_y << 16) | iBuf->dma_x);
			MDP_OUTP(MDP_BASE + 0x00090, 2);
		}


/* LG_FW : 2010.05.02 jinho.jang - 24bit support */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA) //32bit
		if (mfd->panel_info.bpp == 18) {
		MDP_OUTP(MDP_BASE + 0x00094,
				(MDDI_VDO_PACKET_DESC << 16) |
				mfd->panel_info.mddi.vdopkt);
	} else if (mfd->panel_info.bpp == 24) {
		MDP_OUTP(MDP_BASE + 0x00094,
				(MDDI_VDO_PACKET_DESC_24 << 16) |
				mfd->panel_info.mddi.vdopkt);
	} else {
		MDP_OUTP(MDP_BASE + 0x00094,
				(MDDI_VDO_PACKET_DESC_16 << 16) |
				mfd->panel_info.mddi.vdopkt);
	}
#else /*origin*/
		MDP_OUTP(MDP_BASE + 0x00094,
				(MDDI_VDO_PACKET_DESC << 16) |
				mfd->panel_info.mddi.vdopkt);
#endif /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/
	} else {
		/* setting LCDC write window */
		pdata->set_rect(iBuf->dma_x, iBuf->dma_y, iBuf->dma_w,
				iBuf->dma_h);
	}

	if (mfd->panel_info.type == MDDI_PANEL)
		MDP_OUTP(MDP_BASE + 0xa0000, dma_s_cfg_reg);
	else
		MDP_OUTP(MDP_BASE + 0xb0000, dma_s_cfg_reg);

/* LG_FW :  vsync setting */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)
	if ((mfd->use_mdp_vsync) &&
		(mfd->ibuf.vsync_enable) && (mfd->panel_info.lcd.vsync_enable)) {
		uint32 start_y;
	
		if (vsync_start_y_adjust_s <= iBuf->dma_y)
			start_y = iBuf->dma_y - vsync_start_y_adjust_s;
		else
			start_y =
				(mfd->total_lcd_lines - 1) - (vsync_start_y_adjust_s -
								 iBuf->dma_y);
	
		/*
		* MDP VSYNC clock must be On by now so, we don't have to
		* re-enable it
		*/
		if (mfd->panel_info.type == MDDI_PANEL) {
			MDP_OUTP(MDP_BASE + 0x214, start_y);
			MDP_OUTP(MDP_BASE + 0x20c, 2);	/* enable primary(dma_s) vsync */
		} else {
			MDP_OUTP(MDP_BASE + 0x218, start_y);
			MDP_OUTP(MDP_BASE + 0x20c, 4);	/* enable externel(dma_e) vsync */			
		}
	} else {
		MDP_OUTP(MDP_BASE + 0x20c, 0);	/* disable externel vsync */
	}
#else /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/
#ifdef CONFIG_FB_MSM_MDP22
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0180, dma_s_cfg_reg);
#else
	MDP_OUTP(MDP_BASE + 0x90000, dma_s_cfg_reg);
#endif
#endif /*CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA*/

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

/* LG_FW : 2010.08.05 jinho.jang - MDDI TYPE1 and TYPE2 Switch */
#ifdef FEATURE_SWITCH_TYPE1_TYPE2
	mddi_SendGetClientStatus();
#endif

	if (mfd->panel_info.type == MDDI_PANEL)
		mdp_pipe_kickoff(MDP_DMA_S_TERM, mfd);
	else
		mdp_pipe_kickoff(MDP_DMA_E_TERM, mfd);

}

void mdp_dma_s_update(struct msm_fb_data_type *mfd)
{
	down(&mfd->dma->mutex);
	if ((mfd) && (!mfd->dma->busy) && (mfd->panel_power_on)) {
		down(&mfd->sem);
		//mdp_enable_irq(MDP_DMA_S_TERM);
		if (mfd->panel_info.type == MDDI_PANEL)
			mdp_enable_irq(MDP_DMA_S_TERM);
		else
			mdp_enable_irq(MDP_DMA_E_TERM);
		mfd->dma->busy = TRUE;
		INIT_COMPLETION(mfd->dma->comp);
		mfd->ibuf_flushed = TRUE;
		mdp_dma_s_update_lcd(mfd);
		up(&mfd->sem);

		/* wait until DMA finishes the current job */
		wait_for_completion_killable(&mfd->dma->comp);
		if (mfd->panel_info.type == MDDI_PANEL)
			mdp_disable_irq(MDP_DMA_S_TERM);
		else
			mdp_disable_irq(MDP_DMA_E_TERM);

	/* signal if pan function is waiting for the update completion */
		if (mfd->pan_waiting) {
			mfd->pan_waiting = FALSE;
			complete(&mfd->pan_comp);
		}
	}
	up(&mfd->dma->mutex);

/* LG_FW : 2010.07.20 jinho.jang */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)
	//ext_mddi_SendGetClientStatus();
#endif
}
