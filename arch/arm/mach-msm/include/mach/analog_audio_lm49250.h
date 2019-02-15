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


typedef enum{
	AMPDRV_SUCCESS,
	AMPDRV_FAILED,
	AMPDRV_INVALID,
	AMPDRV_NOTSUPPORTED	
}audAmpDrv_ret_type;
    
typedef enum{
	//Commonly used command
	AUDAMP_CMD_MONO_VOL,
	AUDAMP_CMD_STEREO_VOL,
	AUDAMP_CMD_3D_EFFECT,
	AUDAMP_CMD_EARPIECE_GAIN,
	AUDAMP_CMD_HPH_GAIN_UP,
	AUDAMP_CMD_SPEAKER_GAIN,
	AUDAMP_CMD_ALL_IN_GAIN_UP,
	AUDAMP_CMD_FADE_OUT,
	//Etc command
	AUDAMP_CMD_LEFT_VOL,
	AUDAMP_CMD_RIGHT_VOL,
	AUDAMP_CMD_3D_SPEAKER,
	AUDAMP_CMD_3D_HEADPHONE,
	AUDAMP_CMD_SLOW_WAKEUP,
	AUDAMP_CMD_DIAG_VOLUME,	
	AUDAMP_CMD_DIAG_SPOUTGAIN,
	AUDAMP_CMD_DIAG_HPOUTGAIN,       
	//specific
	AUDAMP_CMD_DSP_PLAY,
	AUDAMP_CMD_DSP_STOP,
	AUDAMP_CMD_DSP_EQALIZER,
	//Parameter value conrol
	AUDAMP_CMD_WARMUP_TIME,
	AUDAMP_CMD_EARPIECE_PATH_GAIN, // Get
	AUDAMP_CMD_STEREO_PATH_GAIN, //Get
	AUDAMP_CMD_HEADSET_PATH_GAIN, //Get
	AUDAMP_CMD_GET_CURR_PATH, //Get	
	AUDAMP_CMD_MAINTAIN_DIAG_CTL_PARAM,
	//Max9877 Specific
	AUDAMP_CMD_ZCD_MODE,
	AUDAMP_CMD_DIFF_INPUT,
	AUDAMP_CMD_BYPASS_MODE,
	AUDAMP_CMD_OSC_MODE,

	AUDVIB_CMD_GET_M,
	AUDVIB_CMD_GET_N,
	AUDVIB_CMD_GET_PWM,
	AUDVIB_CMD_GET_AMP, 
	
	AUDAMP_CMD_MAX
}audAmpDrv_param_type;

typedef enum {
//Output Path Ctrl
	AUDAMP_DRV_PATH_OFF, //Shut down
	AUDAMP_DRV_PATH_MUTE,		
	AUDAMP_DRV_PATH_EARPIECE,
	AUDAMP_DRV_PATH_LOUDSPEAKER,
	AUDAMP_DRV_PATH_HEADPHONE,
//Etc.. Reserved..
	AUDAMP_DRV_PATH_HFK,
	AUDAMP_DRV_PATH_BT,
	AUDAMP_DRV_PATH_EXT_SPEAKER,
//Input Path Control
	AUDAMP_DRV_INPUT_SINGLE, // This doesn't mean h/w structure.
	AUDAMP_DRV_INPUT_DUAL, // To Mix two inputs
	AUDAMP_DRV_PATH_MAX
} audAmpDrv_path_type;

typedef enum{
	AUDVIB_CMD_M,
	AUDVIB_CMD_N,
	AUDVIB_CMD_PWM,
	AUDVIB_CMD_AMP,
	AUDVIB_CMD_MAX
}audVibDrv_param_type;


audAmpDrv_ret_type audAmpLM49250_SetParam(audAmpDrv_param_type param,int value);
audAmpDrv_ret_type audAmpLM49250_GetParam(audAmpDrv_param_type param,int *getvalue);
audAmpDrv_ret_type audAmpLM49250_PathCtrl(audAmpDrv_path_type device_path, int value);
audAmpDrv_ret_type audVib_SetParam(audAmpDrv_param_type param,int value);
audAmpDrv_ret_type audVib_Play(void);
audAmpDrv_ret_type audVib_Stop(void);
