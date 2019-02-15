// LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/02/20 {
/* arch/arm/mach-msm/lge_mvs_pkt_loopback_clnt.h
 *
 * Copyright (c) 2008-2010, LG Electronics. All rights reserved.
 *
 * Original code is from Qualcomm mvs_clnt.c in AMSS region.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */
#include <linux/types.h>

#define MVSPROG			0x30000014
#define MVSVERS			0xcf2ba98e /* 0xcf2ba98e */
 /*defines for feature and tool version*/
#define MVS_TOOLVERS	0x00040022
#define MVS_FEATURES	0x00000001
  
 
#define MVSCBPROG		0x31000014
#define MVSCBVERS		0xaed32da4 /* 0xaed32da4 */
 
 
#define ONCRPC_MVS_NULL_PROC 0
#define ONCRPC_MVS_RPC_GLUE_CODE_INFO_REMOTE_PROC 1
 
#define ONCRPC_MVS_MODE_SUPPORTED_PROC 2
#define ONCRPC_MVS_SET_LOOPBACK_PROC 3
#define ONCRPC_MVS_ACQUIRE_PROC 4
#define ONCRPC_MVS_ENABLE_PROC 5
#define ONCRPC_MVS_STANDBY_PROC 6
#define ONCRPC_MVS_RELEASE_PROC 7
#define ONCRPC_MVS_AMR_SET_AMR_MODE_PROC 8
#define ONCRPC_MVS_AMR_SET_SCR_MODE_PROC 9
#define ONCRPC_MVS_VOC_SET_FRAME_RATE_PROC 10
#define ONCRPC_MVS_GSM_SET_DTX_MODE_PROC 11
#define ONCRPC_MVS_GET_G711_MODE_PROC 12
#define ONCRPC_MVS_SET_G711_MODE_PROC 13
#define ONCRPC_MVS_VOC_TX_RATE_LIMIT_PROC 14
#define ONCRPC_MVS_CLIENT_SEND_FRAMES_PROC 15
#define ONCRPC_MVS_TEST 16
#define ONCRPC_MVS_TEST2 17
#define ONCRPC_MVS_LOOPBACK_START_PROC 20
#define ONCRPC_MVS_LOOPBACK_STOP_PROC 21
#define ONCRPC_MVS_API_VERSIONS_PROC 0xFFFFFFFF
 
 /* Start callback xdr procedure numbers */
#define ONCRPC_MVS_EVENT_CB_TYPE_PROC 1
#define ONCRPC_MVS_PACKET_UL_FN_TYPE_PROC 2
#define ONCRPC_MVS_PACKET_DL_FN_TYPE_PROC 3


 /* Definitions of MVS client type */
 typedef enum {
	 MVS_CLIENT_CDMA,	  
	 MVS_CLIENT_WCDMA,	 
	 MVS_CLIENT_GSM,	  
	 MVS_CLIENT_VOIP,
	 MVS_CLIENT_QCHAT,	
	 MVS_CLIENT_VMEMO,	 
	 MVS_CLIENT_QVP,
	 MVS_CLIENT_QVP_TEST,
	 MVS_CLIENT_TEST,
	 MVS_CLIENT_MAX,
	 MVS_CLIENT_NONE = MVS_CLIENT_MAX
 } mvs_client_type;

 /* Definitions of MVS mode type */
 typedef enum {
	 MVS_MODE_NONE,
	 MVS_MODE_IS733,	   /* QCELP-13k 				   */
	 MVS_MODE_IS127,	   /* EVRC-8k					   */
	 MVS_MODE_4GV_NB,	   /* 4GV narrow band			   */
	 MVS_MODE_4GV_WB,	   /* 4GV wide band 			   */
#ifdef FEATURE_ACP
	 MVS_MODE_DFM,		   /* analog mode				   */
#endif
	 MVS_MODE_AMR,		   /* Adaptive multi-rate		   */
	 MVS_MODE_EFR,		   /* Enhanced full rate		   */
	 MVS_MODE_FR,		   /* Full rate 				   */
	 MVS_MODE_HR,		   /* Half rate 				   */
	 MVS_MODE_LINEAR_PCM,  /* enable/disable PCM interface */
	 MVS_MODE_G711, 	   /* G.711 vocoder 			   */
	 MVS_MODE_G723, 	   /* G.723 vocoder 			   */
	 MVS_MODE_MAX,
	 MVS_MODE_PCM		   /* dummy enum to pass compilation, not supported.
							  this has to be after MAX to avoid mapping issue 
							  with QDSP for now. 
							  Limitation: unpredictable behavior when 
							  clients actually use this unsupported mode
							  anyway.
						   */
 } mvs_mode_type;

 /* Definitions of QCELP-13k and EVRC-8k voice frame information */
 typedef enum {
	 MVS_VOC_0_RATE,	   /* blank frame	*/
	 MVS_VOC_8_RATE,	   /* 1/8 rate		*/
	 MVS_VOC_4_RATE,	   /* 1/4 rate		*/
	 MVS_VOC_2_RATE,	   /* 1/2 rate		*/
	 MVS_VOC_1_RATE,	   /* full rate 	*/
	 MVS_VOC_ERASURE,	   /* erasure frame */
	 MVS_VOC_RATE_MAX,
	 MVS_VOC_RATE_UNDEF = MVS_VOC_RATE_MAX
 } mvs_voc_rate_type;


 /* AMR frame type definitions */
 typedef enum {
	 MVS_AMR_SPEECH_GOOD,		   /* Good speech frame 			 */
	 MVS_AMR_SPEECH_DEGRADED,	   /* Speech degraded				 */
	 MVS_AMR_ONSET, 			   /* onset 						 */
	 MVS_AMR_SPEECH_BAD,		   /* Corrupt speech frame (bad CRC) */
	 MVS_AMR_SID_FIRST, 		   /* First silence descriptor		 */
	 MVS_AMR_SID_UPDATE,		   /* Comfort noise frame			 */
	 MVS_AMR_SID_BAD,			   /* Corrupt SID frame (bad CRC)	 */
	 MVS_AMR_NO_DATA,			   /* Nothing to transmit			 */
	 MVS_AMR_FRAME_TYPE_MAX,
	 MVS_AMR_FRAME_TYPE_UNDEF = MVS_AMR_FRAME_TYPE_MAX	/* undefined */
 } mvs_amr_frame_type;
 
 /* AMR frame mode (frame rate) definitions */
 typedef enum {
	 MVS_AMR_MODE_0475,    /* 4.75 kbit /s */
	 MVS_AMR_MODE_0515,    /* 5.15 kbit /s */
	 MVS_AMR_MODE_0590,    /* 5.90 kbit /s */
	 MVS_AMR_MODE_0670,    /* 6.70 kbit /s */
	 MVS_AMR_MODE_0740,    /* 7.40 kbit /s */
	 MVS_AMR_MODE_0795,    /* 7.95 kbit /s */
	 MVS_AMR_MODE_1020,    /* 10.2 kbit /s */
	 MVS_AMR_MODE_1220,    /* 12.2 kbit /s */
	 MVS_AMR_MODE_MAX,
	 MVS_AMR_MODE_UNDEF = MVS_AMR_MODE_MAX	/* undefined */
 } mvs_amr_mode_type;
 
 /* AMR Frame Type Index (4-bit); 3GPP TS 26.101 V6.0.0, Table 1a & 1c */
 typedef enum {
	 MVS_AMR_FRAME_TYPE_INDEX_0475 = 0, 	 /* AMR 4,75 kbit/s 		   */
	 MVS_AMR_FRAME_TYPE_INDEX_0515 = 1, 	 /* AMR 5,15 kbit/s 		   */
	 MVS_AMR_FRAME_TYPE_INDEX_0590 = 2, 	 /* AMR 5,90 kbit/s 		   */
	 MVS_AMR_FRAME_TYPE_INDEX_0670 = 3, 	 /* AMR 6,70 kbit/s (PDC-EFR)  */
	 MVS_AMR_FRAME_TYPE_INDEX_0740 = 4, 	 /* AMR 7,40 kbit/s (TDMA-EFR) */
	 MVS_AMR_FRAME_TYPE_INDEX_0795 = 5, 	 /* AMR 7,95 kbit/s 		   */
	 MVS_AMR_FRAME_TYPE_INDEX_1020 = 6, 	 /* AMR 10,2 kbit/s 		   */
	 MVS_AMR_FRAME_TYPE_INDEX_1220 = 7, 	 /* AMR 12,2 kbit/s (GSM-EFR)  */
	 MVS_AMR_FRAME_TYPE_INDEX_AMR_SID = 8,	 /* AMR SID 				   */
	 MVS_AMR_FRAME_TYPE_INDEX_NO_DATA = 15,  /* No Data 				   */
	 MVS_AMR_FRAME_TYPE_INDEX_MAX,
	 MVS_AMR_FRAME_TYPE_INDEX_UNDEF = MVS_AMR_FRAME_TYPE_INDEX_MAX
 } mvs_amr_frame_type_index_type;


 /* Event type definitions */
 typedef enum {
	 MVS_EV_COMMAND,   /* command status */
	 MVS_EV_MODE,		/* mode status	  */
	 MVS_EV_NOTIFY		/* information notification initiated from mvs	  */
 } mvs_event_enum_type;
 
 
 typedef struct {
	 mvs_event_enum_type event;
	 mvs_client_type client;
 } mvs_ev_header_type;
 
 /* Command event definitions */
 typedef enum {
	 MVS_CMD_FAILURE,
	 MVS_CMD_BUSY,
	 MVS_CMD_SUCCESS
 } mvs_cmd_status_type;
 
 typedef struct {
	 mvs_ev_header_type  hdr;
	 mvs_cmd_status_type cmd_status;
 } mvs_ev_command_type;

 /* Mode event definitions */
 typedef enum {
	 MVS_MODE_NOT_AVAIL,
	 MVS_MODE_INIT,
	 MVS_MODE_READY
 } mvs_mode_status_type;
 
 typedef struct {
	 mvs_ev_header_type   hdr;
	 mvs_mode_status_type mode_status;
	 mvs_mode_type		  mode;
 } mvs_ev_mode_type;

 /* Notify event definitions */
 typedef enum {
	 MVS_NOTIFY_BUFFER_CFG,
	 MVS_NOTIFY_BUFFER_STATUS
 } mvs_notify_enum_type;
 
 typedef struct {
	 mvs_ev_header_type 		hdr;
	 mvs_notify_enum_type		notify_id;
 } mvs_ev_notify_hdr_type;
 
 typedef enum {
	 MVS_BUFFER_DIR_RX,
	 MVS_BUFFER_DIR_TX
 } mvs_buffer_dir_enum_type;
 
 typedef struct {
	 mvs_ev_notify_hdr_type   hdr;
	 mvs_buffer_dir_enum_type dir;
	 u16 				  max_frames;
 }mvs_ev_notify_buffer_cfg_type ;
 
 typedef enum {
	 MVS_BUFFER_STATUS_INIT_FAIL,
	 MVS_BUFFER_STATUS_OVERFLOW,
	 MVS_BUFFER_STATUS_EMPTY,
	 MVS_BUFFER_STATUS_LOCKED,
	 MVS_BUFFER_STATUS_RD_SUCCESS,	   
	 MVS_BUFFER_STATUS_WR_SUCCESS	   
 } mvs_buffer_status_enum_type;
 
 typedef struct {
	 mvs_ev_notify_hdr_type 	 hdr;
	 mvs_buffer_dir_enum_type	 dir;
	 mvs_buffer_status_enum_type status;
 }mvs_ev_notify_buffer_status_type ;
 
 typedef union {
	 mvs_ev_notify_hdr_type 		   hdr;
	 mvs_ev_notify_buffer_cfg_type	   buf_cfg; 
	 mvs_ev_notify_buffer_status_type  buf_status;
 } mvs_ev_notify_type;


 /* Generic event data type definitions */
typedef union {
    mvs_ev_header_type  hdr;
    mvs_ev_command_type cmd;
    mvs_ev_mode_type    mode;
    mvs_ev_notify_type  notify;  
} mvs_event_type;


/* ==========================================================================
** Definitions of MVS frame information (frame rate and frame mode)
** ==========================================================================
*/
typedef enum {
    MVS_FRAME_MODE_NONE,
    MVS_FRAME_MODE_VOC_TX,
    MVS_FRAME_MODE_VOC_RX,
    MVS_FRAME_MODE_AMR_UL,
    MVS_FRAME_MODE_AMR_DL,
    MVS_FRAME_MODE_GSM_UL,
    MVS_FRAME_MODE_GSM_DL,
    MVS_FRAME_MODE_HR_UL,
    MVS_FRAME_MODE_HR_DL,
    MVS_FRAME_MODE_G711_UL,
    MVS_FRAME_MODE_G711_DL,
    MVS_FRAME_MODE_G723_UL,
    MVS_FRAME_MODE_G723_DL,
    MVS_FRAME_MODE_PCM_UL,
    MVS_FRAME_MODE_PCM_DL,
    MVS_FRAME_MODE_AMR_IF1_UL,
    MVS_FRAME_MODE_AMR_IF1_DL,
    MVS_FRAME_MODE_MAX,
    MVS_FRAME_MODE_32BIT_DUMMY = 0x7FFFFFFF
} mvs_frame_mode_type;

/* Definitions of HR voice frame information */
typedef enum {
    MVS_HR_SID,           /* silence descriptor         */
    MVS_HR_SPEECH_GOOD,   /* good speech frame          */
    MVS_HR_SPEECH_BAD,    /* bad speech frame           */
    MVS_HR_INVALID_SID,   /* invalid silence descriptor */
    MVS_HR_FRAME_MAX,
    MVS_HR_FRAME_UNDEF = MVS_HR_FRAME_MAX  /* undefined */
} mvs_hr_frame_type;

/* G.711 vocoder data format */
typedef enum {
    MVS_G711_MODE_MULAW,  /* G.711, mulaw format */
    MVS_G711_MODE_ALAW,   /* G.711, alaw format  */
    MVS_G711_MODE_MAX
} mvs_g711_mode_type;

/* G.723 vocoder data format */
typedef enum {
    MVS_G723_MODE_HIGH,   /* G.723.1, 6.3kbits    */
    MVS_G723_MODE_LOW,    /* G.723.1, 5.3kbits    */
    MVS_G723_MODE_SID,    /* G.723.1, SID frame   */
    MVS_G723_MODE_BLANK,  /* G.723.1, blank frame */
    MVS_G723_MODE_ERROR,  /* Error, no data recd  */
    MVS_G723_MODE_MAX
} mvs_g723_mode_type;

typedef enum {
    MVS_GSM_SID,          /* GSM FR or EFR : silence descriptor  */
    MVS_GSM_SPEECH_GOOD,  /* GSM FR or EFR : good speech frame   */
    MVS_GSM_BFI,          /* GSM FR or EFR : bad frame indicator */
    MVS_GSM_FRAME_MAX,
    MVS_GSM_FRAME_UNDEF = MVS_GSM_FRAME_MAX  /* undefined */
} mvs_gsm_frame_type;

/* GSM frame rate definitions */
typedef enum {
    MVS_GSM_RATE_EFR,     /* GSM EFR mode */
    MVS_GSM_RATE_FR,      /* GSM FR mode */
    MVS_GSM_RATE_MAX,
    MVS_GSM_RATE_UNDEF = MVS_GSM_RATE_MAX  /* undefined */
} mvs_gsm_rate_type;

/* Definitions of context in which MVS packet exchange takes place */
typedef enum {
    MVS_PKT_CONTEXT_NONE,   /* no packet exchange requested */
    MVS_PKT_CONTEXT_ISR,    /* packets are exchanged in ISR context */
    MVS_PKT_CONTEXT_VOC,    /* packets are exchanged in VOC task context */
    MVS_PKT_CONTEXT_ISR_ASYNC_RSP,  /* packets are pushed / request in ISR context  */
    /* Responce in client context */   
    MVS_PKT_CONTEXT_SIO,    /* Using SIO port for packet exchange with clients */
    MVS_PKT_CONTEXT_MAX
} mvs_pkt_context_type;


/* Definitions of MVS packet status type */
typedef enum {
    MVS_PKT_STATUS_NORMAL,
    MVS_PKT_STATUS_FAST,    /* packets were supplied too fast */
    MVS_PKT_STATUS_SLOW     /* packets were supplied too slowly */
} mvs_pkt_status_type;


typedef struct {
  mvs_frame_mode_type frame_mode;
  mvs_mode_type       mode;
  u16              buf_free_cnt;
} mvs_frame_info_hdr_type;

/* QCELP-13k and EVRC-8k and 4GV frame info type */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_voc_rate_type rate;
} mvs_voc_tx_frame_info_type;

typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_voc_rate_type rate;
} mvs_voc_rx_frame_info_type;

typedef union {
    mvs_frame_info_hdr_type hdr;
    mvs_voc_tx_frame_info_type tx_info;
    mvs_voc_rx_frame_info_type rx_info;
} mvs_voc_frame_info_type;
#ifdef FEATURE_HTORPC_METACOMMENTS
/*~ TYPE mvs_voc_frame_info_type DISC (_OBJ_.hdr.frame_mode) */
/*~ CASE MVS_FRAME_MODE_VOC_TX mvs_voc_frame_info_type.tx_info */
/*~ CASE MVS_FRAME_MODE_VOC_RX mvs_voc_frame_info_type.rx_info */
/*~ DEFAULT mvs_voc_frame_info_type.hdr */
#endif /* FEATURE_HTORPC_METACOMMENTS */

/* AMR frame info type */
/* uplink frame info structure */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_amr_frame_type  frame;
    mvs_amr_mode_type   mode;
} mvs_amr_ul_frame_info_type;

/* downlink frame info structure */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_amr_frame_type  frame;
    mvs_amr_mode_type   mode;
} mvs_amr_dl_frame_info_type;

typedef union {
    mvs_frame_info_hdr_type hdr;
    mvs_amr_ul_frame_info_type ul_info;
    mvs_amr_dl_frame_info_type dl_info;
} mvs_amr_frame_info_type;
#ifdef FEATURE_HTORPC_METACOMMENTS
/*~ TYPE mvs_amr_frame_info_type DISC (_OBJ_.hdr.frame_mode) */
/*~ CASE MVS_FRAME_MODE_AMR_UL mvs_amr_frame_info_type.ul_info */
/*~ CASE MVS_FRAME_MODE_AMR_DL mvs_amr_frame_info_type.dl_info */
/*~ DEFAULT mvs_amr_frame_info_type.hdr */
#endif /* FEATURE_HTORPC_METACOMMENTS */

/* GSM EFR and FR frame info type */
/* uplink frame info structure */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_gsm_frame_type  frame;
    mvs_gsm_rate_type   rate;
} mvs_gsm_ul_frame_info_type;

/* downlink frame info structure */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_gsm_frame_type frame;
    bool taf;                     /* time allignment flag */
} mvs_gsm_dl_frame_info_type;

typedef union {
    mvs_frame_info_hdr_type hdr;
    mvs_gsm_ul_frame_info_type ul_info;
    mvs_gsm_dl_frame_info_type dl_info;
} mvs_gsm_frame_info_type;
#ifdef FEATURE_HTORPC_METACOMMENTS
/*~ TYPE mvs_gsm_frame_info_type DISC (_OBJ_.hdr.frame_mode) */
/*~ CASE MVS_FRAME_MODE_GSM_UL mvs_gsm_frame_info_type.ul_info */
/*~ CASE MVS_FRAME_MODE_GSM_DL mvs_gsm_frame_info_type.dl_info */
/*~ DEFAULT mvs_gsm_frame_info_type.hdr */
#endif /* FEATURE_HTORPC_METACOMMENTS */

/* HR frame info type */
/* uplink frame info structure */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_hr_frame_type frame;
    bool enc_err_flag;
} mvs_hr_ul_frame_info_type;

/* downlink frame info structure */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_hr_frame_type frame;
    bool taf;
    bool bfi;
    bool ufi;
} mvs_hr_dl_frame_info_type;

typedef union {
    mvs_frame_info_hdr_type hdr;
    mvs_hr_ul_frame_info_type ul_info;
    mvs_hr_dl_frame_info_type dl_info;
} mvs_hr_frame_info_type;
#ifdef FEATURE_HTORPC_METACOMMENTS
/*~ TYPE mvs_hr_frame_info_type DISC (_OBJ_.hdr.frame_mode) */
/*~ CASE MVS_FRAME_MODE_HR_UL mvs_hr_frame_info_type.ul_info */
/*~ CASE MVS_FRAME_MODE_HR_DL mvs_hr_frame_info_type.dl_info */
/*~ DEFAULT mvs_hr_frame_info_type.hdr */
#endif /* FEATURE_HTORPC_METACOMMENTS */

/* G.711 frame info structure */
/* Uplink frame info type */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_g711_mode_type  g711_mode;
} mvs_g711_ul_frame_info_type;

/* Downlink frame info type */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_g711_mode_type  g711_mode;
} mvs_g711_dl_frame_info_type;

typedef union {
    mvs_frame_info_hdr_type hdr;
    mvs_g711_ul_frame_info_type  ul_info;
    mvs_g711_dl_frame_info_type  dl_info;
} mvs_g711_frame_info_type;
#ifdef FEATURE_HTORPC_METACOMMENTS
/*~ TYPE mvs_g711_frame_info_type DISC (_OBJ_.hdr.frame_mode) */
/*~ CASE MVS_FRAME_MODE_G711_UL mvs_g711_frame_info_type.ul_info */
/*~ CASE MVS_FRAME_MODE_G711_DL mvs_g711_frame_info_type.dl_info */
/*~ DEFAULT mvs_g711_frame_info_type.hdr */
#endif /* FEATURE_HTORPC_METACOMMENTS */

/* G.723 frame info structure */
/* Uplink frame info type */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_g723_mode_type  g723_mode;
} mvs_g723_ul_frame_info_type;

/* Downlink frame info type */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    mvs_g723_mode_type  g723_mode;
} mvs_g723_dl_frame_info_type;

typedef union {
    mvs_frame_info_hdr_type hdr;
    mvs_g723_ul_frame_info_type  ul_info;
    mvs_g723_dl_frame_info_type  dl_info;
} mvs_g723_frame_info_type;
#ifdef FEATURE_HTORPC_METACOMMENTS
/*~ TYPE mvs_g723_frame_info_type DISC (_OBJ_.hdr.frame_mode) */
/*~ CASE MVS_FRAME_MODE_G723_UL mvs_g723_frame_info_type.ul_info */
/*~ CASE MVS_FRAME_MODE_G723_DL mvs_g723_frame_info_type.dl_info */
/*~ DEFAULT mvs_g723_frame_info_type.hdr */
#endif /* FEATURE_HTORPC_METACOMMENTS */

/* PCM frame info type */
/* uplink frame info structure */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    void *info;            /* unused for now */
} mvs_pcm_ul_frame_info_type;

/* downlink frame info structure */
typedef struct {
    mvs_frame_info_hdr_type hdr;
    void *info;            /* unused for now */
} mvs_pcm_dl_frame_info_type;

typedef union {
    mvs_frame_info_hdr_type hdr;
    mvs_pcm_ul_frame_info_type ul_info;
    mvs_pcm_dl_frame_info_type dl_info;
} mvs_pcm_frame_info_type;
#ifdef FEATURE_HTORPC_METACOMMENTS
/*~ TYPE mvs_pcm_frame_info_type DISC (_OBJ_.hdr.frame_mode) */
/*~ CASE MVS_FRAME_MODE_PCM_UL mvs_pcm_frame_info_type.ul_info */
/*~ CASE MVS_FRAME_MODE_PCM_DL mvs_pcm_frame_info_type.dl_info */
/*~ DEFAULT mvs_pcm_frame_info_type.hdr */
#endif /* FEATURE_HTORPC_METACOMMENTS */

typedef struct {
    mvs_frame_info_hdr_type hdr;
    u32            packet_length;
    mvs_amr_frame_type_index_type frame_type_index;
    bool fqi;   /* Frame Quality Indicator: TRUE: good frame, FALSE: bad frame */
} mvs_amr_if1_ul_frame_info_type;

typedef struct {
    mvs_frame_info_hdr_type hdr;
    u32            packet_length;
    mvs_amr_frame_type_index_type frame_type_index;
    bool fqi;   /* Frame Quality Indicator: TRUE: good frame, FALSE: bad frame */
} mvs_amr_if1_dl_frame_info_type;

typedef union {
    mvs_frame_info_hdr_type hdr;
    mvs_amr_if1_ul_frame_info_type ul_info;
    mvs_amr_if1_dl_frame_info_type dl_info;
} mvs_amr_if1_frame_info_type;
#ifdef FEATURE_HTORPC_METACOMMENTS
/*~ TYPE mvs_amr_if1_frame_info_type DISC (_OBJ_.hdr.frame_mode) */
/*~ CASE MVS_FRAME_MODE_AMR_IF1_UL mvs_amr_if1_frame_info_type.ul_info */
/*~ CASE MVS_FRAME_MODE_AMR_IF1_DL mvs_amr_if1_frame_info_type.dl_info */
/*~ DEFAULT mvs_amr_if1_frame_info_type.hdr */
#endif /* FEATURE_HTORPC_METACOMMENTS */

/* Generic MVS frame info type definitions */
typedef union {
    mvs_frame_info_hdr_type  hdr;
    mvs_voc_frame_info_type   voc_rate;
    mvs_amr_frame_info_type   amr_rate;
    mvs_gsm_frame_info_type   gsm_rate;
    mvs_hr_frame_info_type    hr_rate;
    mvs_g711_frame_info_type  g711_rate;
    mvs_g723_frame_info_type  g723_rate;
    mvs_pcm_frame_info_type   pcm_rate;
    mvs_amr_if1_frame_info_type amr_if1_info;
} mvs_frame_info_type;


/* Event callback function type */
typedef void (*mvs_event_cb_type)(mvs_event_type *event) ;

/* Uplink packet transfer callback function type */
typedef void (*mvs_packet_ul_fn_type)(u8               *vocoder_packet,
                                      mvs_frame_info_type *frame_info,
                                      u16               packet_length,
                                      mvs_pkt_status_type *status);

/* Downlink packet transfer callback function type */
typedef void (*mvs_packet_dl_fn_type)(u8				 *vocoder_packet,
									mvs_frame_info_type  *frame_info,
									mvs_pkt_status_type  *status);


/*===========================================================================

						  INTERNAL LOOP BACK DECLARATIONS

===========================================================================*/
/**
  @ingroup ftm_audio_misc_doxy_group
  @brief Defines the numeric value corresponding to the maximum possible
  volume level accepted by the FTM_SET_VOLUME command.
*/
#define MAX_VOL 4

/**
  @ingroup ftm_audio_misc_doxy_group
  @brief Defines the # of vocoder packets stored in a local buffer between
  downlink and uplink.
*/
#define MVS_LB_PKT_DEPTH 5

/**
  @ingroup ftm_audio_misc_doxy_group
  @brief Stores whether one-time FTM audio initialization has been performed.
*/

/**
  @ingroup ftm_audio_misc_doxy_group
  @brief Global FTM audio state information.
*/
typedef s32 cad_handle_type;

void ftm_audio_dummy_mvs_event_cb(mvs_event_type *event);
void ftm_audio_mvs_ul_cb(
  u8 *vocoder_packet,
  mvs_frame_info_type *frame_info,
  u16			   packet_length,
  mvs_pkt_status_type *status
);
void ftm_audio_mvs_dl_cb(
  u8 *vocoder_packet,
  mvs_frame_info_type  *frame_info,
  mvs_pkt_status_type  *status
);

void mvs_pkt_loopback_start(int selected_device);
void mvs_pkt_loopback_stop(int selected_device);


void mvs_enable(
					  mvs_client_type client, 
					  mvs_mode_type mode, 
					  mvs_packet_ul_fn_type ul_func,
					  mvs_packet_dl_fn_type dl_func,
					  mvs_pkt_context_type context
					  );

void mvs_acquire(
					   mvs_client_type client, 
					   mvs_event_cb_type cb_func
					   );

void mvs_release(mvs_client_type client);

void mvs_loopback_start(void);
void mvs_loopback_stop(void);

// LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/01/26 }

