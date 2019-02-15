#ifndef LG_DIAG_AUDIOTEST_H
#define LG_DIAG_AUDIOTEST_H

#include "lg_comdef.h"

/*********************** BEGIN PACK() Definition ***************************/
#if defined __GNUC__
  #define PACK(x)       x __attribute__((__packed__))
  #define PACKED        __attribute__((__packed__))
#elif defined __arm
  #define PACK(x)       __packed x
  #define PACKED        __packed
#else
  #error No PACK() macro defined for this compiler
#endif
/********************** END PACK() Definition *****************************/

#define MAX_KEY_BUFF_SIZE    200
#if 0
typedef enum
{
	VER_SW=0,	  //Binary Revision
	VER_DSP,	  /* Camera DSP */
} audio_test_req_version_type;
#endif 
#if 0
typedef enum{
	AMPDRV_SUCCESS,
	AMPDRV_FAILED,
	AMPDRV_INVALID,
	AMPDRV_NOTSUPPORTED	
}audAmpDrv_ret_type;
    

typedef enum
{
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
	AUDAMP_CMD_MAX
  
} audio_test_req_amp_type;
#endif 
#if 0
typedef union
{
  audio_test_req_version_type		version;
  audio_test_req_amp_type   amp;
} audio_test_req_type;
#endif 

typedef struct diagpkt_header
{
  byte opaque_header;
}PACKED diagpkt_header_type;

/*===========================================================================
                      TYPE DEFINITIONS
===========================================================================*/


typedef struct DIAG_AUDIO_TEST_F_req_tag {
	uint8 cmd_code; 	  /* Command code */
	uint8 sub_cmd1; 	  /* Message Sub-command 1*/
	uint16	  sub_cmd2; 	  /* Message Sub-command 2 */  
	uint16	  sub_cmd3; 	  /* Message Sub-command 3 */
	uint16	  pad;			  /* Unused */	  
	uint32	  Reserved; 	  /* Reserved */
	uint32	  param1;
	uint32	  param2;
	uint32	  param3; 
	uint32	  length;		  /* Size of cast_data */  
	uint32	  cast_data[1];   /* Data..Array size equals to  length*/

} PACKED DIAG_AUDIO_TEST_F_req_type;	

typedef enum
{
  TEST_OK_S,
  TEST_FAIL_S,
  TEST_NOT_SUPPORTED_S
} PACKED audio_test_ret_stat_type;

typedef struct 
{
    byte SVState;
    uint8 SV;
    uint16 MeasuredCNo;
} PACKED CGPSResultType;

#if 0
typedef union
{
  audio_test_req_version_type		version;
  audio_test_req_amp_type		    amp;
} PACKED audio_test_rsp_type;

#endif 
/*===========================================================================
                      TYPE DEFINITIONS
===========================================================================*/
typedef enum{
	CAST_SUBCMD_GPIO_CTL=1,
	CAST_SUBCMD_I2C_CTL=2,
	CAST_SUBCMD_SND_CTL=3,
	CAST_SUBCMD_QDSP_CTL=4,
	CAST_SUBCMD_MAX
}cast_sub_cmd1_type;

typedef struct DIAG_AUDIO_TEST_F_rsp_tag {
	uint8     cmd_code; 	  /* Command code */
	uint8     sub_cmd1; 	  /* Message Sub-command1 */
	uint16	  sub_cmd2; 	  /* Message Sub-command2 */
	uint16	  sub_cmd3; 	  /* Message Sub-command3 */ 
	uint16	  status;		  /* Response Status of operation */
	uint32	  Reserved; 	  /* Reserved */
	uint32	  param1;
	uint32	  param2;
	uint32	  param3;
	uint32	  length;		  /* Size of cast_data */  
	uint32	  cast_data[1];   /* Data..Array size equals to  length*/

} PACKED DIAG_AUDIO_TEST_F_rsp_type;


#define AUDIOTEST_MSTR_TBL_SIZE   128

#define ARM9_PROCESSOR       0
#define ARM11_PROCESSOR     1


typedef enum{
	//Get Operation
	DCAST_SB2CMD_GET_CURRENT_INFO=0,
	DCAST_SB2CMD_GET_QDSP_INFO,
	DCAST_SB2CMD_GET_AUDIO_PATH_INFO,		/* snd_crnt_path_out */
	DCAST_SB2CMD_GET_AUDIO_CTL_DATA,		/* snd_cal_control_data */		//Const Data  : Read Only Const
	DCAST_SB2CMD_GET_PCM_PATH_DATA,		/* voc_pcm_cal_aptr */		//Const Data  : Read Only Const
	DCAST_SB2CMD_GET_AUDIO_PATH_CAL_DATA, /* voc_cal_audio_path_config */	//Const Data  : Read Only Const
	DCAST_SB2CMD_GET_QDSP_ADJ_DATA,		/* voc_data_audio_qdsp_data */
	DCAST_SB2CMD_GET_VOC_PATH_ADJ_DATA,	/* voc_data_voc_adj_data */
	DCAST_SB2CMD_GET_CODEC_ADJ_DATA,		/* voc_data_codec_adj_data */
	DCAST_SB2CMD_GET_MIDI_TAP_ADJ_DATA,	/* midi_tab */
	DCAST_SB2CMD_GET_VOC_AGC_ADJ_DATA,	/* voc_data_agc_data */
	DCAST_SB2CMD_GET_VOC_RXFIR_ADJ_DATA,	/* voc_data_rx_fir_pcm_filter_data */
	DCAST_SB2CMD_GET_VOC_TXFIR_ADJ_DATA,	/* voc_data_tx_fir_pcm_filter_data */
	DCAST_SB2CMD_AUDIO_AMP_CTL,
	DCAST_SB2CMD_AUDIO_PLAY_CTL,	
	//Set Operation
	DCAST_SB2CMD_SET_CURRENT_INFO,
	DCAST_SB2CMD_SET_QDSP_INFO,
	DCAST_SB2CMD_SET_AUDIO_PATH_INFO,		/* snd_crnt_path_out */
	DCAST_SB2CMD_SET_QDSP_ADJ_DATA,		/* voc_data_audio_qdsp_data */
	DCAST_SB2CMD_SET_VOC_PATH_ADJ_DATA,	/* voc_data_voc_adj_data */
	DCAST_SB2CMD_SET_CODEC_ADJ_DATA,		/* voc_data_codec_adj_data */
	DCAST_SB2CMD_SET_MIDI_TAP_ADJ_DATA,	/* midi_tab */
	DCAST_SB2CMD_SET_VOC_AGC_ADJ_DATA,		/* voc_data_agc_data */
	DCAST_SB2CMD_SET_VOC_RXFIR_ADJ_DATA,	/* voc_data_rx_fir_pcm_filter_data */
	DCAST_SB2CMD_SET_VOC_TXFIR_ADJ_DATA,	/* voc_data_tx_fir_pcm_filter_data */
	// Reserved
	DCAST_SB2CMD_SET_PCM_PATH_DATA,
	DCAST_SB2CMD_ADIE_MIC_CTL,
	//EC Operation
	DCAST_SB2CMD_EC_PARAM_DATA = 0xFF,		/* qdsp_cmd_ec_params_type */
	DCAST_SB2CMD_EAR_SHOCK_TEST,
	DCAST_SB2CMD_AGC_CONFIG,
	//scmsvc_cal_sup.h
	DCAST_SB2CMD_SET_SCM_CTRL_DATA =0x180,
	DCAST_SB2CMD_GET_SCM_CTRL_DATA =0x181,
	//EQ ctrl
	DCAST_SB2CMD_EQ_ENABLE =0x190,
	DCAST_SB2CMD_EQ_SET_FILTER =0x191,
	
	DCAST_SB2CMD_MAX,
	DCAST_SB2CMD_DUMMY=0x7FFF
}diagCastSndSubCmd2Type;
/*DCAST_SB2CMD_GET_QDSP_INFO*/
typedef enum{
	DCAST_SB3CMD_GET_QDSP_VOL_INFO=0,
	DCAST_SB3CMD_GET_QDSP_VOC_INFO,
	DCAST_SB3CMD_GET_QDSP_MAX_INFO
}diagCastQdspInfoSubCmd3Type;
/*DCAST_SB2CMD_AUDIO_AMP_CTL*/

typedef enum{
	DCAST_SB3CMD_AUDAMP_INIT,
	DCAST_SB3CMD_AUDAMP_PATHCTL,
	DCAST_SB3CMD_AUDAMP_SETPARAM,
	DCAST_SB3CMD_AUDAMP_GETPARAM,
	DCAST_SB3CMD_AUDAMP_I2CWRITE,
	DCAST_SB3CMD_AUDVIB_SETPARAM,
	DCAST_SB3CMD_AUDVIB_GETPARAM,
	DCAST_SB3CMD_AUDVIB_PLAY,
	DCAST_SB3CMD_AUDVIB_STOP,
	DCAST_SB3CMD_AUDAMP_MAX
}diagCastAudAmpSubCmd3Type;
/*DCAST_SB2CMD_AUDIO_PLAY_CTL*/


/*DCAST_SB2CMD_AUDIO_PLAY_CTL*/
typedef enum{
	DCAST_SB3CMD_PLAY_TONE,
	DCAST_SB3CMD_PLAY_SNDID,
	DCAST_SB3CMD_PLAY_MOTOR,
	DCAST_SB3CMD_PLAY_FREQ,
	DCAST_SB3CMD_PLAY_STOP,
	DCAST_SB3CMD_PLAY_GETVOL,
	DCAST_SB3CMD_PLAY_SETVOL,
	DCAST_SB3CMD_PLAY_STATUS,
	DCAST_SB3CMD_PLAY_MAX
}diagCastAudPlaySubCmd3Type;
/*DCAST_SB2CMD_SET_QDSP_INFO*/
typedef enum{
	DCAST_SB3CMD_SET_QDSP_ENABLE,
	DCAST_SB3CMD_SET_QDSP_DISABLE,
	DCAST_SB3CMD_SET_QDSP_MAX
}diagCastSetQdspSubCmd3Type;
/*DCAST_SB2CMD_ADIE_MIC_CTL*/
typedef enum{
	DCAST_SB3CMD_GET_MIC_CTRL,
	DCAST_SB3CMD_SET_MIC_GAIN,
	DCAST_SB3CMD_MIC_GAIN_CTRL_MAX
}diagCastMicCtrlSubCmd3Type;
typedef enum{
	DCAST_MIC1_MINUS_2DB=0,
	DCAST_MIC1_PULS_6DB,
	DCAST_MIC1_PULS_8DB,
	DCAST_MIC1_PULS_16DB,
	DCAST_MIC1_GAIN_MAX
}castSndMic1GainType;
typedef struct{
	uint32 mic1_gain;
	uint32 mic2_bypass;
}castSndMicGainCtrlType;
/*DCAST_SB2CMD_EC_PARAM_DATA*/
typedef enum{
	DCAST_SB3CMD_GET_EC_PARAM,
	DCAST_SB3CMD_SET_EC_PARAM,
	DCAST_SB3CMD_EC_PARAM_CTRL_MAX
}diagCastEcParamSubCmd3Type;
/*DCAST_SB2CMD_EAR_SHOCK_TEST*/
typedef enum{
	DCAST_SB3CMD_EARS_LOW_BATT=0,
	DCAST_SB3CMD_EARS_SMS_NOTI,
	DCAST_SB3CMD_EARS_ONE_MINUTE,
	DCAST_SB3CMD_EARS_POWER_OFF,
	DCAST_SB3CMD_EARS_CALL_CONNECT,
	DCAST_SB3CMD_EARS_MAX
}diagCastEarShockSubCmd3Type;
/*DCAST_SB2CMD_AGC_CONFIG*/
typedef enum{
	DCAST_SB3CMD_GET_AGC_CONFIG=0,
	DCAST_SB3CMD_SET_AGC_CONFIG,
	DCAST_SB3CMD_AGC_CONFIG_MAX
}diagCastAgcCfgSubCmd3Type;
/*DCAST_SB3CMD_GET_QDSP_VOL_INFO*/
typedef struct{
	uint32 audioState;
	uint32 mastVolume;
	uint32 rxVolume;
	uint32 txVolume;
	uint32 rxMute;
	uint32 txComfortNoiseEnable;
	uint32 codec_tx_gain;
	uint32 codec_rx_gain;
	uint32 codec_st_gain;	
	uint32 qdsp_current_image;
	uint32 num_modules;
	uint32 qdsp_app_nums;
	uint32 qdsp_clock_mask;
	uint32 qdsp_services_current_state;
	uint32 msm_pcm_path;
	uint32 reserved;
}castSndQdspVolumeType;
/*DCAST_SB3CMD_GET_QDSP_VOC_INFO*/
typedef struct{
//Agc,Avc,Agc Enable/Disable
	uint32 rxAgcEnableFlag;
	uint32 rxAvcEnableFlag;
	uint32 txAgcEnableFlag;
//Agc Control	
	uint32 compFlinkStaticGain;
	uint32 compFlinkAIGFlag;
	uint32 expFlinkThreshold;
	uint32 expFlinkSlope;
	uint32 compFlinkThreshold;
	uint32 compFlinkSlope;	
	uint32 avcRlinkSensitivityOffset;
	uint32 avcFlinkHeadroom;
	uint32 compRlinkStaticGain;
	uint32 compRlinkAIGFlag;
	uint32 expRlinkThreshold;
	uint32 expRlinkSlope;
	uint32 compRlinkThreshold;	
	uint32 compRlinkSlope;
//Rx Filter	
	uint32 rxPcmFiltCoeff0;
	uint32 rxPcmFiltCoeff1;
	uint32 rxPcmFiltCoeff2;
	uint32 rxPcmFiltCoeff3;
	uint32 rxPcmFiltCoeff4;
	uint32 rxPcmFiltCoeff5;
	uint32 rxPcmFiltCoeff6;	
//Tx Filter	
	uint32 txPcmFiltCoeff0;
	uint32 txPcmFiltCoeff1;
	uint32 txPcmFiltCoeff2;
	uint32 txPcmFiltCoeff3;
	uint32 txPcmFiltCoeff4;
	uint32 txPcmFiltCoeff5;
	uint32 txPcmFiltCoeff6;
// Ec parameters
	uint32 ec_mode;
	uint32 ecStartupMuteMode;
	uint32 ecMuteOverride;
	uint32 ecFarActivethres ;
	uint32 ecStartupErlethres ;
	uint32 esecFiltResetCheckThres;
	uint32 esecDoubletalkHangoverThres;
// Reserved
	uint32 reserved[20];
}castSndQdspVocInfoType;
typedef enum{
	DCAST_SND_NOT_SUPPORT=0,
	DCAST_SND_SUCCESS=1,
	DCAST_SND_INVALID_REQ,
	DCAST_SND_BAD_RARAM,
	DCAST_SND_COMM_FAIL,
	DCAST_SND_FAILURE,
	DCAST_SND_STATUS_MAX
}diagCastSndResType;
#if 0
typedef void*(* audiotest_func_type)(audio_test_req_type * , DIAG_AUDIO_TEST_F_rsp_type * );


typedef struct
{
  word cmd_code;
  audiotest_func_type func_ptr;
  byte  which_procesor;             // to choose which processor will do act.
}audiotest_user_table_entry_type;
#endif 

extern int testmode_volume_state;
#endif /*LG_DIAG_AUDIOTEST_H*/
