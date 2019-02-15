/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>


#include <linux/wakelock.h>
#include <linux/android_pmem.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>

#include <linux/gpio.h>
#include <mach/pmic.h>
#include <mach/msm_qdsp6_audio.h>
#include <linux/delay.h>
#include <mach/gpio.h>
//#include "audio_amp_test.h"
//#include "../LGE_Target/gpio_lgevs760.h"


#include  <mach/analog_audio_lm49250.h>

#if 1
#define TRACE(x...) pr_info("lm49250: "x)
#else
#define TRACE(x...) do{}while(0)
#endif


#define AUDIO_SWITCH_ADDR   0xF8 // or FA or F8

#define AMP_SHUTDOWN_CTRL		0x00 //Shutdown control
#define AMP_SINPMODE_CTRL		0x08 //stereo input mode control
#define AMP_3DMODE_CTRL		    0x10 //3D control
#define AMP_3DGAIN_CTRL		    0x18 //3D Gain control
#define AMP_LDO_CTRL		    0x20 //LDO control
#define AMP_HPGAIN_CTRL		    0x28 //headphone gain control
#define AMP_SPOPSTGAIN_CTRL		0x30 //Speaker Output Stage Gain control
#define AMP_EPMUXGAIN_CTRL		0x38 //Earpiece MUX/ Gain control
#define AMP_SPOPMUX_CTRL		0x40 //Speaker(LS) Output MUX control
#define AMP_HPOPMUX_CTRL		0x60 //Headphone(HP) Output MUX control
#define AMP_OUTONOFF_CTRL		0x80 //output on/off control
#define AMP_MONOINGAIN_CTRL		0xA0 //mono input gain control
#define AMP_LEFTINGAIN_CTRL		0xC0 //left  input gain control
#define AMP_RIGHTINGAIN_CTRL	0xE0 //right input gain control
#define AMP_REG_MAX_NUM		14

//AMP_SHUTDOWN_CTRL 
#define PWR_M				0x01 //power on(1: enable)
#define PWR_EN_V			0x01 //power on(1: enable)
#define PWR_DIS_V			0x00 //power on(1: enable)

//AMP_SINPMODE_CTRL 
#define ST_INPUT_INSEL_M    0x03 //Stereo Input Mode
#define ST_DIFF_INSEL    	0x00 //Fully Differential Input Mode
#define ST_L2_INSEL		    0x01 //Single-ended input. R2 and L2 selected
#define ST_L1_INSEL		    0x02 //Single-ended input. R1 and L1 selected
#define ST_MIX_INSEL		0x03 //Single-ended input. R1 mixed with R2 and L1 mixed with L2

#define ST_INPUT_MUTE_M			0x04//Master Power On bit 
#define ST_INPUT_MUTE_EN_V		0x04
#define ST_INPUT_MUTE_DIS_V		0x00

//AMP_3DMODE_CTRL 
#define AMP_3DHP_M		    0x01
#define AMP_3DHP_EN_V		0x01
#define AMP_3DHP_DIS_V		0x00
#define AMP_3DLS_M		    0x02
#define AMP_3DLS_EN_V		0x02
#define AMP_3DLS_DIS_V		0x00
#define AMP_3DN_M		    0x04
#define AMP_3DN_WIDER		0x00
#define AMP_3DN_NARROWER	0x04


//AMP_3DGAIN_CTRL 
#define AMP_3DGAIN_M		    0x03
#define AMP_3DGAIN_LOW		    0x00
#define AMP_3DGAIN_MED		    0x01
#define AMP_3DGAIN_HIGH		    0x02
#define AMP_3DGAIN_MAX	        0x03

//AMP_LDO_CTRL 
#define TON_M		        0x01
#define TON_35MS		    0x00
#define TON_20MS		    0x01
#define LDOH_M		        0x02
#define LDOH_EN_V		    0x02 //don't care
#define LDOH_DIS_V		    0x00 
 
//AMP_HPGAIN_CTRL
#define HPG_M 0x07  //Headphone Out Gain
#define HPG_M00DB 0x00  // 0dB
#define HPG_M01DB 0x01  //-1.2dB
#define HPG_M03DB 0x02  //-2.5dB
#define HPG_M04DB 0x03  //-4dB
#define HPG_M06DB 0x04  //-6dB
#define HPG_M09DB 0x05  //-8.5dB
#define HPG_M12DB 0x06  //-12dB
#define HPG_M18DB 0x07  //-18dB

//AMP_SPOPSTGAIN_CTRL
#define SP_LSGAIN_M 0x03
#define SP_LSLG_M 0x01
#define SP_LSLG_6DB 0x00
#define SP_LSLG_12DB 0x01
#define SP_LSRG_M 0x02
#define SP_LSRG_6DB 0x00
#define SP_LSRG_12DB 0x02
#define SP_LSDG_6DB 0x00
#define SP_LSDG_12DB 0x03
#define SP_SS_M 0x04  //Spread Spectrum
#define SP_SS_EN_V 0x04
#define SP_SS_DIS_V 0x00


//AMP_EPMUXGAIN_CTRL
#define EP_MODE_M 0x03
#define EP_SSEL_V 0x01  // output : stereo select 
#define EP_MSEL_V 0x02  // output : mono select 
#define EP_MODE_MUTE 0x00
#define EP_MODE_MONO EP_MSEL_V //select
#define EP_MODE_LR EP_SSEL_V
#define EP_MODE_MLR (EP_MSEL_V | EP_SSEL_V)
#define EP_GAIN_M 0x04
#define EP_GAIN_0DB 0x00
#define EP_GAIN_6DB 0x04


//AMP_SPOPMUX_CTRL
#define LS_MODE_M 0x1F
#define LS_MUTE 0x00
#define LSL_SSEL_V 0x01 //output LS left stero select
#define LSL_MSEL_V 0x02 //ouput LS left mono select 
#define LSR_SSEL_V 0x04
#define LSR_MSEL_V 0x08
#define LS_XSEL_V  0x10 // cross 
#define LS_MODE_MUTE 0x00
#define LS_MODE_MM (LSR_MSEL_V | LSL_MSEL_V)
#define LS_MODE_LR (LSR_SSEL_V | LSL_SSEL_V)
#define LS_MODE_MLMR (LSR_MSEL_V | LSR_SSEL_V | LSL_MSEL_V | LSL_SSEL_V)
#define LS_MODE_RL LS_XSEL_V
#define LS_MODE_MRML (LS_XSEL_V | LSR_MSEL_V | LSL_MSEL_V)
#define LS_MODE_LRLR (LS_XSEL_V | LSR_SSEL_V | LSL_SSEL_V)
#define LS_MODE_MLRMLR (LS_XSEL_V | LSR_MSEL_V | LSR_SSEL_V | LSL_MSEL_V | LSL_SSEL_V)



//AMP_HPOPMUX_CTRL
#define HP_MODE_M 0x1F
#define HPL_SSEL_V 0x01 //output HP left stero select
#define HPL_MSEL_V 0x02 //output HP left stero select
#define HPR_SSEL_V 0x04
#define HPR_MSEL_V 0x08 
#define HP_XSEL_V  0x10
#define HP_MODE_MUTE 0x00
#define HP_MODE_MM (HPR_MSEL_V | HPL_MSEL_V)
#define HP_MODE_LR (HPR_SSEL_V | HPL_SSEL_V)
#define HP_MODE_MLMR (HPR_MSEL_V | HPR_SSEL_V | HPL_MSEL_V | HPL_SSEL_V)
#define HP_MODE_RL HP_XSEL_V
#define HP_MODE_MRML (HP_XSEL_V | HPR_MSEL_V | HPL_MSEL_V)
#define HP_MODE_LRLR (HP_XSEL_V | HPR_SSEL_V | HPL_SSEL_V)
#define HP_MODE_MLRMLR (HP_XSEL_V | HPR_MSEL_V | HPR_SSEL_V | HPL_MSEL_V | HPL_SSEL_V)


//AMP_OUTONOFF_CTRL
#define OUTPUT_ONOFF_M 0x1F
#define OUTPUT_ALLOFF  0x00
#define LSL_M			0x01//0: shutdown, 1: wakeup
#define LSL_EN_V		0x01
#define LSL_DIS_V		0x00
#define LSR_M			0x02//0: shutdown, 1: wakeup
#define LSR_EN_V		0x02
#define LSR_DIS_V		0x00
#define LS_EN_V         0x03
#define HPL_M			0x04//0: shutdown, 1: wakeup
#define HPL_EN_V		0x04
#define HPL_DIS_V		0x00
#define HPR_M			0x08//0: shutdown, 1: wakeup
#define HPR_EN_V		0x08
#define HPR_DIS_V		0x00
#define HP_EN_V         0x0c  
#define EP_M			0x10//0: shutdown, 1: wakeup
#define EP_EN_V		    0x10
#define EP_DIS_V		0x00

#define AMP_VOL_M 0x1F

typedef enum {     // input mode 0,3        1,2  
	AUDIO_VOL_0 ,  //  -64dB         ,-58dB    
	AUDIO_VOL_1 ,  //  -54.5dB       ,-48.5dB  
	AUDIO_VOL_2 ,  //  -47dB 	       ,-41dB   
	AUDIO_VOL_3 ,  //  -40dB	       ,-34dB   
	AUDIO_VOL_4 ,  //  -33dB	       ,-27dB   
	AUDIO_VOL_5 ,  //  -28dB	       ,-21dB   
	AUDIO_VOL_6 ,  //  -24dB	       ,-18dB   
	AUDIO_VOL_7 ,  //  -21dB	       ,-15dB   
	AUDIO_VOL_8 ,  //  -19.5dB       ,-13.5dB  
	AUDIO_VOL_9 ,  //  -18dB 	       ,-12dB   
	AUDIO_VOL_10, //   -16.5dB       ,-10.5dB  
	AUDIO_VOL_11, //   -15dB 	       ,-9dB    
	AUDIO_VOL_12, //   -13.5dB       ,-7.5dB   
	AUDIO_VOL_13, //   -12dB 	       ,-6dB    
	AUDIO_VOL_14, //   -10.5dB       ,-4.5dB   
	AUDIO_VOL_15, //   -9.0dB        ,-3dB     
	AUDIO_VOL_16, //   -7.5dB        ,-1.5dB   
	AUDIO_VOL_17, //   -6dB 	       ,0dB     
	AUDIO_VOL_18, //   -4.5dB        ,2.5dB	
	AUDIO_VOL_19, //   -3dB 	       ,3dB     
	AUDIO_VOL_20, //   1.5dB 	       ,4.5dB   
	AUDIO_VOL_21, //   0dB 	       ,6dB     
	AUDIO_VOL_22, //   1.5dB 	       ,7.5dB   
	AUDIO_VOL_23, //   3dB 	       ,9dB     
	AUDIO_VOL_24, //   4.5dB 	       ,10.5dB  
	AUDIO_VOL_25, //   6dB 	       ,12dB    
	AUDIO_VOL_26, //   7.5dB 	       ,13dB    
	AUDIO_VOL_27, //   9dB 	       ,15dB    
	AUDIO_VOL_28, //   10.5dB        ,16.5     
	AUDIO_VOL_29, //   12dB 	       ,18dB    
	AUDIO_VOL_30, //   13.5dB        ,19.5dB   
	AUDIO_VOL_31, //   15dB 	       ,21dB    
	AUDIO_VOL_MAX
} audio_lm4857_volume_type;
 
#define SPOPSTGAIN_6DB_V		(SP_SS_DIS_V | SP_LSRG_6DB | SP_LSLG_6DB) // nochange 
#define SPOPSTGAIN_12DB_V		(SP_SS_DIS_V | SP_LSRG_12DB | SP_LSLG_12DB) // nochange 
#define SPOPSTGAIN_DOCK_V		(SP_SS_DIS_V | SP_LSRG_6DB | SP_LSLG_12DB) // nochange 


#define SHUTDOWN_DEFAULT_V		(PWR_DIS_V)
#define SINPMODE_DEFAULT_V		(ST_INPUT_MUTE_DIS_V | ST_L1_INSEL)
#define AMP_3DMODE_DEFAULT_V		(AMP_3DN_WIDER | AMP_3DLS_DIS_V | AMP_3DHP_DIS_V )
#define AMP_3DGAINE_DEFAULT_V		(AMP_3DGAIN_LOW)
#define LDO_DEFAULT_V			       (LDOH_EN_V | TON_20MS)
#define HPGAIN_DEFAULT_V			(HPG_M06DB)  // nochange 
#define SPOPSTGAIN_DEFAULT_V		(SP_SS_DIS_V | SP_LSRG_6DB | SP_LSLG_6DB) // nochange 
#define EPMUXGAIN_DEFAULT_V		(EP_GAIN_0DB | EP_MODE_MONO)
#define SPOPMUX_DEFAULT_V			(LS_MODE_LR)
#define HPOPMUX_DEFAULT_V			(HP_MODE_LR)
#define OUTONOFF_DEFAULT_V			(OUTPUT_ALLOFF)
#define MONOINGAIN_DEFAULT_V		(AUDIO_VOL_20)
#define LEFTINGAIN_DEFAULT_V		(AUDIO_VOL_20)
#define RIGHTINGAIN_DEFAULT_V		(AUDIO_VOL_20)
static u8 showdownRegVal;
static u8 stinpmodeRegVal;
static u8 amp3dmodeRegVal;
static u8 amp3gainegVal;
static u8 ldoRegVal;
static u8 hpgainRegVal;
static u8 spopgainRegVal;
static u8 epmuxgainRegVal;
static u8 spopmuxRegVal;
static u8 hpopmuxmvolRegVal;
static u8 ouponoffRegVal;
static u8 monogainRegVal;
static u8 leftgainRegVal;
static u8 rightgainRegVal;

#define AMP_ON 0x01
#define AMP_SD 0x00

#define AMP_PATH_HANDSET 0x01
#define AMP_PATH_SPEAKER_MONO 0x02
#define AMP_PATH_SPEAKER_STEREO 0x03
#define AMP_PATH_HEDSET 0x04
#define AMP_PATH_SPEAKER_HEDSET 0x05
#define AMP_PATH_NONE 0x00

extern bool amp_write_register(u8 reg);
#if defined (CONFIG_LGE_DOCK_STATE_DETECT) && defined (CONFIG_LGE_MACH_ENVT2_REVD)
extern int audio_get_state(void);
#endif 
bool  Audio_drv_SetSDCtrl(u8 mode);
bool audio_LM49250_Path_Speaker_Mono(void);
bool Audio_drv_SetMonoVol(u8 vol);
bool Audio_drv_SetStereoVol(u8 vol);
void analog_receiver_enable(int en);
void analog_headset_enable(int en);
void analog_tty_headset_enable(int en);
void analog_speaker_enable(int en);
void analog_stereo_speaker_enable(int en);
void analog_sp_hp_enable(int en);

void  Audio_drv_SetInit(void)
{  

	showdownRegVal	=	(AMP_SHUTDOWN_CTRL | SHUTDOWN_DEFAULT_V);
	stinpmodeRegVal 	=	(AMP_SINPMODE_CTRL | SINPMODE_DEFAULT_V); // change
	amp3dmodeRegVal 	=	(AMP_3DMODE_CTRL | AMP_3DMODE_DEFAULT_V);
	amp3gainegVal		=	(AMP_3DGAIN_CTRL | AMP_3DGAINE_DEFAULT_V);
	ldoRegVal			=	(AMP_LDO_CTRL | LDO_DEFAULT_V);
	hpgainRegVal		=	(AMP_HPGAIN_CTRL | HPGAIN_DEFAULT_V); //no change
	spopgainRegVal		=	(AMP_SPOPSTGAIN_CTRL | SPOPSTGAIN_DEFAULT_V); //no change
	epmuxgainRegVal 	=	(AMP_EPMUXGAIN_CTRL | EPMUXGAIN_DEFAULT_V);//no change
	spopmuxRegVal		=	(AMP_SPOPMUX_CTRL | SPOPMUX_DEFAULT_V); //no change
	hpopmuxmvolRegVal	=	(AMP_HPOPMUX_CTRL | HPOPMUX_DEFAULT_V); //no change
	ouponoffRegVal		=	(AMP_OUTONOFF_CTRL | OUTONOFF_DEFAULT_V);
	monogainRegVal		=	(AMP_MONOINGAIN_CTRL | MONOINGAIN_DEFAULT_V); //no change
	leftgainRegVal		=	(AMP_LEFTINGAIN_CTRL | LEFTINGAIN_DEFAULT_V);
	rightgainRegVal 	      =	(AMP_RIGHTINGAIN_CTRL | RIGHTINGAIN_DEFAULT_V);
	  
	amp_write_register(showdownRegVal);
	amp_write_register(stinpmodeRegVal);
	amp_write_register(amp3dmodeRegVal);
	amp_write_register(amp3gainegVal);
	amp_write_register(ldoRegVal);	
	amp_write_register(hpgainRegVal);	
	amp_write_register( spopgainRegVal);
	amp_write_register(epmuxgainRegVal);
	amp_write_register(spopmuxRegVal);
	amp_write_register(hpopmuxmvolRegVal);
	amp_write_register(ouponoffRegVal);	
	amp_write_register(monogainRegVal);	
 	amp_write_register(leftgainRegVal);	
	amp_write_register(rightgainRegVal);	

}   

audAmpDrv_ret_type audAmpLM49250_PathCtrl(audAmpDrv_path_type device_path, int value)
{
	audAmpDrv_ret_type rtn = AMPDRV_SUCCESS; //AMPDRV_FAILED;

	switch(device_path)
	{
		case AUDAMP_DRV_PATH_OFF:
			Audio_drv_SetMonoVol(AUDIO_VOL_0);
			Audio_drv_SetStereoVol(AUDIO_VOL_0);
			if(Audio_drv_SetSDCtrl(AMP_SD))
				rtn = AMPDRV_SUCCESS;
			break;
		case AUDAMP_DRV_PATH_MUTE:
			Audio_drv_SetMonoVol(AUDIO_VOL_0);
			Audio_drv_SetStereoVol(AUDIO_VOL_0);
			rtn = AMPDRV_SUCCESS;			
			break;
		case AUDAMP_DRV_PATH_EARPIECE:
			analog_receiver_enable(1);
			rtn =AMPDRV_SUCCESS;	
			break;
		case AUDAMP_DRV_PATH_LOUDSPEAKER:
			analog_speaker_enable(1);
			rtn = AMPDRV_SUCCESS;	
			break;			
		case AUDAMP_DRV_PATH_HEADPHONE:
			analog_headset_enable(1);
			rtn = AMPDRV_SUCCESS;
			break;
		default:
			rtn = AMPDRV_INVALID;
			
			break;
	}
	
	return rtn;
}

audAmpDrv_ret_type audAmpLM49250_SetParam(audAmpDrv_param_type param,int value)

{
	audAmpDrv_ret_type rtn = AMPDRV_FAILED;
	switch(param)
	{
		case AUDAMP_CMD_MONO_VOL:
			if(Audio_drv_SetMonoVol(value))
				rtn = AMPDRV_SUCCESS;
			break;
		case AUDAMP_CMD_STEREO_VOL:
			if(Audio_drv_SetStereoVol(value))
				rtn = AMPDRV_SUCCESS;
			break;		
#if 0
		case AUDAMP_CMD_LEFT_VOL:
			if(audio_LM49250_set_left_volume(value))
				rtn = AMPDRV_SUCCESS;
			break;				
		case AUDAMP_CMD_RIGHT_VOL:
			if(audio_LM49250_set_right_volume(value))
				rtn = AMPDRV_SUCCESS;
			break;
#endif
		case AUDAMP_CMD_EARPIECE_GAIN: //Speaker Output Gain
			if (value) {	
				if(amp_write_register(AMP_EPMUXGAIN_CTRL| EP_GAIN_6DB | EP_MSEL_V)) //temp eklee
					rtn = AMPDRV_SUCCESS;	
			}else{
				if(amp_write_register(AMP_EPMUXGAIN_CTRL | EP_GAIN_6DB | EP_MSEL_V)) //temp eklee
					rtn = AMPDRV_SUCCESS;	
			}	
			break;

		case AUDAMP_CMD_SPEAKER_GAIN:
			
			if (value) {	
				if(amp_write_register(AMP_SPOPSTGAIN_CTRL| SP_SS_DIS_V |	SP_LSDG_12DB)) //temp eklee
					rtn = AMPDRV_SUCCESS;	
			}else{
				if(amp_write_register(AMP_SPOPSTGAIN_CTRL | SP_SS_DIS_V |	SP_LSDG_6DB)) //temp eklee
					rtn = AMPDRV_SUCCESS;	
			}	
			break;
			
		case AUDAMP_CMD_HPH_GAIN_UP:
			if (amp_write_register(AMP_HPGAIN_CTRL|(value&HPG_M)))  //temp eklee
				rtn = AMPDRV_SUCCESS;
			break;

			break;
#if 0			
		case AUDAMP_CMD_MAINTAIN_DIAG_CTL_PARAM:
			lm49250_maintainDiagctlValuesFlag = value;
			break; 
		case AUDAMP_CMD_DIAG_VOLUME:
			lm49250_diag_volgain_adj = (audAmpDrv_Volume_type)value;
			break;

		case AUDAMP_CMD_DIAG_SPOUTGAIN:
			lm49250_diag_spoutgain_adj = (audAmpDrv_Volume_type)value;
			break;

		case AUDAMP_CMD_DIAG_HPOUTGAIN:
			lm49250_diag_hpoutgain_adj = (audAmpDrv_Volume_type)value;
			break;
#endif 			
		default:  
			rtn = AMPDRV_NOTSUPPORTED;
			break;
	}
	return rtn;

}
extern int t_nLRA_GPMN_M;
extern  t_nLRA_GPMN_N ;
extern  t_nLRA_GPMN_CLK_PWM_MUL;
int  t_amp = 120;

audAmpDrv_ret_type audAmpLM49250_GetParam(audAmpDrv_param_type param,int *getvalue)
{
	int rtn = AMPDRV_SUCCESS;
	
	if(!getvalue)return AMPDRV_FAILED;

	switch(param)
	{
		case AUDAMP_CMD_WARMUP_TIME:
			*getvalue	=	20; //100ms, 150ms delay
			break;
		case AUDAMP_CMD_MONO_VOL:
			*getvalue	= 	monogainRegVal&AMP_VOL_M;
			break;			
		case AUDAMP_CMD_STEREO_VOL:
			*getvalue	=	leftgainRegVal&AMP_VOL_M;
			if(*getvalue <	(rightgainRegVal&AMP_VOL_M))
				*getvalue  = leftgainRegVal&AMP_VOL_M;
			break;			
		case AUDAMP_CMD_LEFT_VOL:  
			*getvalue	=	leftgainRegVal&AMP_VOL_M;  
			break;					
		case AUDAMP_CMD_RIGHT_VOL:
			*getvalue	=	rightgainRegVal&AMP_VOL_M;
			break;
		case AUDAMP_CMD_EARPIECE_GAIN: 
			*getvalue	=	epmuxgainRegVal&EP_GAIN_M;
			break;
		case AUDAMP_CMD_HPH_GAIN_UP:
			*getvalue	=	hpgainRegVal&HPG_M;	
			break;    
		case AUDAMP_CMD_SPEAKER_GAIN:
			*getvalue	=	spopgainRegVal&SP_LSGAIN_M;	
			break;	  
#if 0
		case AUDAMP_CMD_GET_CURR_PATH:
			*getvalue = lm49250_currentOutputPath;
			break;
		case AUDAMP_CMD_MAINTAIN_DIAG_CTL_PARAM:
			*getvalue = lm49250_maintainDiagctlValuesFlag;
			break;			  
		case AUDAMP_CMD_DIAG_VOLUME:
			*getvalue = lm49250_diag_volgain_adj;		
#endif 			

		case AUDVIB_CMD_GET_M:
			*getvalue	=	t_nLRA_GPMN_M; //100ms, 150ms delay
			break;
		case AUDVIB_CMD_GET_N:
			*getvalue	= 	t_nLRA_GPMN_N;
			break;			
		case AUDVIB_CMD_GET_PWM:
			*getvalue	=	t_nLRA_GPMN_CLK_PWM_MUL;
			break;			
		case AUDVIB_CMD_GET_AMP:  
			*getvalue	=	t_amp;  
			break;		
		default:
			rtn = AMPDRV_NOTSUPPORTED;
			break;
	}
	return rtn;
}
#if 1


audAmpDrv_ret_type audVib_SetParam(audAmpDrv_param_type param,int value)
{
	audAmpDrv_ret_type rtn = AMPDRV_FAILED;
	switch(param)
	{
		case AUDVIB_CMD_M:
			t_nLRA_GPMN_M = value;
			TRACE("AUDVIB_CMD_M %d , %d\n", param,value);
			rtn = AMPDRV_SUCCESS;
			break;
		case AUDVIB_CMD_N:
			t_nLRA_GPMN_N = value;
			TRACE("AUDVIB_CMD_N %d , %d\n", param,value);
			rtn = AMPDRV_SUCCESS;
			break;		
		case AUDVIB_CMD_PWM: //Speaker Output Gain
		    t_nLRA_GPMN_CLK_PWM_MUL = value;
		    TRACE("AUDVIB_CMD_PWM %d , %d\n", param,value);
			rtn = AMPDRV_SUCCESS;	
			break;

		case AUDVIB_CMD_AMP:
			t_amp = value;
			TRACE("AUDVIB_CMD_AMP %d , %d\n", param,value);
			rtn = AMPDRV_SUCCESS;	
			break;
		default:  
			rtn = AMPDRV_NOTSUPPORTED;
			break;
	}
	return rtn;

}

extern	 int android_vibrator_tune(int nForce);

audAmpDrv_ret_type audVib_Play(void)
{
	int rtn = AMPDRV_SUCCESS;
	
	android_vibrator_tune(t_amp);
	rtn = AMPDRV_SUCCESS;
	return rtn;

}

audAmpDrv_ret_type audVib_Stop(void)
{
	int rtn = AMPDRV_SUCCESS;
    
	android_vibrator_tune(0);
	rtn = AMPDRV_SUCCESS;
	return rtn;

}

#endif 
bool  audio_LM49250_Path_Earpiece(void)
{
	bool result = false;
	stinpmodeRegVal = AMP_SINPMODE_CTRL | ST_DIFF_INSEL;
    spopmuxRegVal	 = AMP_SPOPMUX_CTRL | LS_MUTE;
	ouponoffRegVal = AMP_OUTONOFF_CTRL |EP_EN_V;
	
	amp_write_register(stinpmodeRegVal);
	amp_write_register(spopmuxRegVal);
	result = amp_write_register(ouponoffRegVal);

	return result;
}


bool  audio_LM49250_Path_Speaker_Mono(void)
{
    bool result = false;
	
	stinpmodeRegVal = AMP_SINPMODE_CTRL|ST_INPUT_MUTE_DIS_V| ST_L1_INSEL;
	spopgainRegVal =	(AMP_SPOPSTGAIN_CTRL | SPOPSTGAIN_6DB_V);
    spopmuxRegVal = AMP_SPOPMUX_CTRL | LS_XSEL_V | LS_MODE_LR;
	
	ouponoffRegVal = AMP_OUTONOFF_CTRL |LSR_EN_V;
	
	amp_write_register(stinpmodeRegVal);
	amp_write_register(spopgainRegVal);
	amp_write_register(spopmuxRegVal);
	result = amp_write_register(ouponoffRegVal);

	return result;
}


bool  audio_LM49250_Path_Dock_Speaker_Stereo(void)
{
	bool result = false;
	
	stinpmodeRegVal = AMP_SINPMODE_CTRL|ST_INPUT_MUTE_DIS_V| ST_L1_INSEL;
	spopgainRegVal =	(AMP_SPOPSTGAIN_CTRL | SPOPSTGAIN_6DB_V);
    spopmuxRegVal	 = AMP_SPOPMUX_CTRL | LS_XSEL_V | LS_MODE_LR;
    ouponoffRegVal = AMP_OUTONOFF_CTRL | LS_EN_V;

	amp_write_register(stinpmodeRegVal);
	amp_write_register(spopgainRegVal);
	amp_write_register(spopmuxRegVal);
	result =  amp_write_register(ouponoffRegVal);

	return result;
}


bool  audio_LM49250_Path_Speaker_Stereo(void)
{
	bool result = false;
	
	stinpmodeRegVal = AMP_SINPMODE_CTRL|ST_INPUT_MUTE_DIS_V| ST_L1_INSEL;
	spopgainRegVal =	(AMP_SPOPSTGAIN_CTRL | SPOPSTGAIN_6DB_V);
    spopmuxRegVal	 = AMP_SPOPMUX_CTRL | LS_MODE_LR;
    ouponoffRegVal = AMP_OUTONOFF_CTRL | LS_EN_V;

	amp_write_register(stinpmodeRegVal);
	amp_write_register(spopgainRegVal);
	amp_write_register(spopmuxRegVal);
	result =  amp_write_register(ouponoffRegVal);

	return result;
}

bool  audio_LM49250_Path_Headset(void)
{
	bool result = false;
	
	stinpmodeRegVal  =AMP_SINPMODE_CTRL|ST_INPUT_MUTE_DIS_V| ST_L1_INSEL;
	hpgainRegVal = (AMP_HPGAIN_CTRL | HPGAIN_DEFAULT_V);
	spopmuxRegVal =AMP_SPOPMUX_CTRL | LS_MUTE;
    ouponoffRegVal = AMP_OUTONOFF_CTRL |HP_EN_V;
	   
	amp_write_register(stinpmodeRegVal);
	amp_write_register(hpgainRegVal);
    amp_write_register(spopmuxRegVal);
	result =   amp_write_register(ouponoffRegVal);	

	return result;

}      


bool  audio_LM49250_Path_Speaker_Headset(void)
{
	bool result = false;
#if 0
	   stinpmodeRegVal = AMP_SINPMODE_CTRL|ST_INPUT_MUTE_DIS_V| ST_L1_INSEL | ST_L2_INSEL;
       spopmuxRegVal= AMP_SPOPMUX_CTRL | LS_XSEL_V | LS_MODE_LR;;
       ouponoffRegVal = AMP_OUTONOFF_CTRL|LS_EN_V | LSR_EN_V |HP_EN_V; //add
#endif 
  
	stinpmodeRegVal = AMP_SINPMODE_CTRL|ST_INPUT_MUTE_DIS_V| ST_L1_INSEL;//L | ST_L2_INSEL;
	hpgainRegVal = (AMP_HPGAIN_CTRL | HPG_M18DB);
	spopgainRegVal =	(AMP_SPOPSTGAIN_CTRL | SPOPSTGAIN_12DB_V);
	spopmuxRegVal= AMP_SPOPMUX_CTRL | LS_XSEL_V | LS_MODE_LR;
	ouponoffRegVal = AMP_OUTONOFF_CTRL | LSR_EN_V | HP_EN_V; //add

	amp_write_register(stinpmodeRegVal);
	amp_write_register(hpgainRegVal);
	amp_write_register(spopgainRegVal);
	amp_write_register(spopmuxRegVal);
	result =  amp_write_register(ouponoffRegVal);

	return result;
}

bool  audio_LM49250_Path_Off(void)
{
	bool result = false;
	
    ouponoffRegVal = AMP_OUTONOFF_CTRL | OUTPUT_ALLOFF;

	result =  amp_write_register(ouponoffRegVal);

	return result;
}

bool  Audio_drv_SetSDCtrl(u8 mode)
{
	bool result = false;
	
	showdownRegVal	=AMP_SHUTDOWN_CTRL | mode;

	result =  amp_write_register(showdownRegVal);		

	return result;

}
   
bool Audio_drv_OutGain(u8 mode)
{
	bool result = false;
#if 0
	MSG_HIGH("[Audio Drv] Audio_drv_OutGain	Vol : %d",mode,0,0);
	*spopgainRegVal=AMP_SPOPSTGAIN_CTRL|mode;
	return I2C_Write_Audio(AUDIO_SWITCH_ADDR, 1, out_gain_reg);
#endif 	

	return result;
}

bool Audio_drv_SetMonoVol(u8 vol)
{
	bool result = false;
	
	if (vol<=0)
  		vol = 0;
  	else if( vol > AUDIO_VOL_MAX) 
  		vol = AUDIO_VOL_MAX;

	monogainRegVal=AMP_MONOINGAIN_CTRL|vol;
	result =  amp_write_register(monogainRegVal);

	return result;
}

bool  Audio_drv_SetStereoVol(u8 vol)
{
	bool result = false;
	
  	if (vol<=0)
  		vol = 0;
  	else if( vol > AUDIO_VOL_MAX) 
  		vol = AUDIO_VOL_MAX;

	leftgainRegVal=AMP_LEFTINGAIN_CTRL|vol;
	rightgainRegVal=AMP_RIGHTINGAIN_CTRL|vol;

   	amp_write_register(leftgainRegVal);

   	result =  amp_write_register(rightgainRegVal);

   	return result;

}

bool  Audio_drv_SetHpLRVol(u8 vol)
{
	bool result = false;
	
  	if (vol<=0)
  		vol = 0;
  	else if( vol > AUDIO_VOL_MAX) 
  		vol = AUDIO_VOL_MAX;

	hpgainRegVal =(AMP_HPGAIN_CTRL | vol); //no change

   	result = amp_write_register(hpgainRegVal);

	return result;

}

void  Audio_drv_SetPLC(u8 mode)
{
#if 0
	MSG_HIGH("[Audio Drv] Audio_drv_SetPLC  Power limit control :0x%x",mode,0,0);

	*plc_reg = POWER_LIMIT_CTRL_REG|mode;

	return I2C_Write_Audio(AUDIO_SWITCH_ADDR, 1, plc_reg);		
#endif 	
}


void Audio_drv_SetNCC(u8 mode)
{
#if 0
	MSG_HIGH("[Audio Drv] Audio_drv_SetNCC	No Clip control :0x%x",mode,0,0);

	*ncc_reg = NO_CLIP_CTRL_REG|mode;

	return I2C_Write_Audio(AUDIO_SWITCH_ADDR, 1, ncc_reg);		
#endif 	
}


void Audio_drv_SetSSCtrl(u8 mode)
{
#if 0
	MSG_HIGH("[Audio Drv] Audio_drv_SetSSCtrl Spread Spectrum :0x%x",mode,0,0);

	*ss_reg = SS_CTRL_REG|mode;

	return I2C_Write_Audio(AUDIO_SWITCH_ADDR, 1, ss_reg);		
#endif 	
}

void analog_init(void)
{
    //Audio_drv_SetInit();  //eklee 0808
	pmic_mic_set_volt(MIC_VOLT_1_80V);
}


void analog_receiver_enable(int en)
{
	TRACE("analog_receiver_enable %d\n", en);
	if (en) {
		audio_LM49250_Path_Earpiece();
        Audio_drv_SetMonoVol(AUDIO_VOL_22);
		Audio_drv_SetSDCtrl(AMP_ON);
	}else {
	    Audio_drv_SetMonoVol(AUDIO_VOL_0);
		audio_LM49250_Path_Off();
		Audio_drv_SetSDCtrl(AMP_SD);
	}
}

void analog_headset_enable(int en)
{
	TRACE("analog_headset_enable %d\n", en);
	if (en) {
		audio_LM49250_Path_Headset();
        Audio_drv_SetStereoVol(AUDIO_VOL_19);
		Audio_drv_SetSDCtrl(AMP_ON);
	}else {
	    Audio_drv_SetStereoVol(AUDIO_VOL_0);
		audio_LM49250_Path_Off();
		Audio_drv_SetSDCtrl(AMP_SD);
	}
}

void analog_tty_headset_enable(int en)
{
	TRACE("analog_tty_headset_enable %d\n", en);
	if (en) {
		audio_LM49250_Path_Headset();
        Audio_drv_SetStereoVol(AUDIO_VOL_17);
		Audio_drv_SetSDCtrl(AMP_ON);
	}else {
	    Audio_drv_SetStereoVol(AUDIO_VOL_0);
		audio_LM49250_Path_Off();
		Audio_drv_SetSDCtrl(AMP_SD);
	}
}


void analog_speaker_select_path(void)
{
#if defined (CONFIG_LGE_DOCK_STATE_DETECT) && defined (CONFIG_LGE_MACH_ENVT2_REVD)
	if (audio_get_state()) {
		TRACE("dock -- audio_LM49250_Path_Speaker_Stereo\n");
		audio_LM49250_Path_Dock_Speaker_Stereo();
	} else 
#endif 		
		audio_LM49250_Path_Speaker_Mono();

}


void analog_speaker_enable(int en)
{
	TRACE("analog_speaker_mono_enable %d\n", en);
	if (en) {
		analog_speaker_select_path();
        Audio_drv_SetStereoVol(AUDIO_VOL_21);
		Audio_drv_SetSDCtrl(AMP_ON);
	}else {
	    Audio_drv_SetStereoVol(AUDIO_VOL_0);
		audio_LM49250_Path_Off();
		Audio_drv_SetSDCtrl(AMP_SD);
	}
}


void analog_stereo_speaker_enable(int en)
{
	TRACE("analog_stereo_speaker_enable %d\n", en);
	if (en) {
		audio_LM49250_Path_Speaker_Stereo();
        Audio_drv_SetStereoVol(AUDIO_VOL_23);  
		Audio_drv_SetSDCtrl(AMP_ON);
	}else {
	    Audio_drv_SetStereoVol(AUDIO_VOL_0);
		audio_LM49250_Path_Off();
		Audio_drv_SetSDCtrl(AMP_SD);
	}  
}


void analog_sp_hp_enable(int en)
{
	TRACE("analog_sp_hp_enable %d\n", en);
	if (en) {
		audio_LM49250_Path_Speaker_Headset();
        Audio_drv_SetStereoVol(AUDIO_VOL_17);
		Audio_drv_SetSDCtrl(AMP_ON);
	}else {
	    Audio_drv_SetStereoVol(AUDIO_VOL_0);
		audio_LM49250_Path_Off();
		Audio_drv_SetSDCtrl(AMP_SD);
	}
}

void analog_mic_enable(int en)
{
	pmic_mic_en(en);
}

static struct q6audio_analog_ops ops = {
	.init = analog_init,
	.speaker_enable = analog_speaker_enable,
	.headset_enable = analog_headset_enable,
	.receiver_enable = analog_receiver_enable,
#ifdef CONFIG_LGE_AUDIO_SUBSYSTEM
	.sp_hp_enable = analog_sp_hp_enable,
	.stereo_speaker_enable = analog_stereo_speaker_enable,
	.tty_headset_enable = analog_tty_headset_enable,
#endif 		
	.int_mic_enable = analog_mic_enable,
	.ext_mic_enable = analog_mic_enable,
};

static int __init init(void)
{
	q6audio_register_analog_ops(&ops);
	return 0;
}

device_initcall(init);

MODULE_AUTHOR("eklee.lee@lge.com");
MODULE_DESCRIPTION("LGE audio lm49250 driver");
MODULE_LICENSE("GPL");
