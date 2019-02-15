#include <linux/module.h>
#include <mach/lg_diagcmd.h>
#include <linux/input.h>
#include <linux/syscalls.h>


#ifdef CONFIG_LGE_AUDIO_TUNE_TOOL
#include "lg_diag_communication.h"
#endif
#include <mach/lg_diag_audiotest.h>
#include <linux/delay.h>
#include  <mach/analog_audio_lm49250.h>
#include <mach/msm_qdsp6_audio.h>

/* ==========================================================================
===========================================================================*/

#ifdef CONFIG_LGE_AUDIO_TUNE_TOOL
/* ==========================================================================
===========================================================================*/
#ifdef CONFIG_LGE_AUDIO_TUNE_TOOL
static struct diagcmd_dev *diagpdev;
#endif

extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
extern PACK(void *)diagpkt_err_rsp (diagpkt_cmd_code_type code, PACK(void *)req_ptr, uint16 req_len);

extern PACK(void *) diagpkt_free (PACK(void *)pkt);
extern void send_to_arm9( void*	pReq, void	*pRsp);
#if 0
extern audiotest_user_table_entry_type audiotest_mstr_tbl[AUDIOTEST_MSTR_TBL_SIZE];
#endif 
static void * diagSndAudioAmpCtl (void *req_pkt, word req_len);
static void * diagSndGetVocPathAdjData (void *req_pkt, word req_len);
static void * diagSndSetVocPathAdjData (void *req_pkt, word req_len);
//static void * diagSndAudioVibCtl (void *req_pkt, word req_len);


//extern audAmpDrv_ret_type audAmpLM49250_SetParam(audAmpDrv_param_type param,int value);
//extern audAmpDrv_ret_type audAmpLM49250_GetParam(audAmpDrv_param_type param,int *getvalue);


/* ==========================================================================
===========================================================================*/



struct statfs_local {
 __u32 f_type;
 __u32 f_bsize;
 __u32 f_blocks;
 __u32 f_bfree;
 __u32 f_bavail;
 __u32 f_files;
 __u32 f_ffree;
 __kernel_fsid_t f_fsid;
 __u32 f_namelen;
 __u32 f_frsize;
 __u32 f_spare[5];
};

/* ==========================================================================
===========================================================================*/


PACK (void *)LGF_AudioTest (
        PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
        uint16		pkt_len )		      /* length of request packet   */
{
  DIAG_AUDIO_TEST_F_req_type *req_ptr = (DIAG_AUDIO_TEST_F_req_type *) req_pkt_ptr;
  DIAG_AUDIO_TEST_F_rsp_type *rsp_ptr ;


  unsigned int rsp_len;
#if 0 

  audiotest_func_type func_ptr= NULL;

  int nIndex = 0;
#endif 

#ifdef CONFIG_LGE_AUDIO_TESTMODE
	diagpdev = diagcmd_get_dev();
#endif



  rsp_len = sizeof(DIAG_AUDIO_TEST_F_rsp_type);		

  rsp_ptr = (DIAG_AUDIO_TEST_F_rsp_type *)diagpkt_alloc(DIAG_AUDIO_TEST_F, rsp_len);
#if 0

  rsp_ptr->cmd_code = req_ptr->cmd_code;
  rsp_ptr->ret_stat_code = TEST_OK_S; // �ʱⰪ


  for( nIndex = 0 ; nIndex < AUDIOTEST_MSTR_TBL_SIZE  ; nIndex++)
  {
    if( audiotest_mstr_tbl[nIndex].cmd_code == req_ptr->sub_cmd_code)
    {
        if( audiotest_mstr_tbl[nIndex].which_procesor == ARM11_PROCESSOR)
          func_ptr = audiotest_mstr_tbl[nIndex].func_ptr;
      break;
    }     
  }

  if( func_ptr != NULL)
    rsp_ptr = func_ptr( &(req_ptr->audio_test_req), rsp_ptr);
  else
    send_to_arm9((void*)req_ptr, (void*)rsp_ptr);
#endif 

   if(req_ptr && req_ptr->sub_cmd1 == CAST_SUBCMD_SND_CTL)
   {
	 /* dispatch according to subcommand */
	 //------------------------------------------------------------------------
		switch (req_ptr->sub_cmd2)
	 	{
			case DCAST_SB2CMD_GET_VOC_PATH_ADJ_DATA:		/* voc_data_voc_adj_data */
				rsp_ptr = (void *)diagSndGetVocPathAdjData(req_ptr,pkt_len);
				break;
		 	case DCAST_SB2CMD_AUDIO_AMP_CTL:
			 	rsp_ptr = (void *)diagSndAudioAmpCtl(req_ptr,pkt_len);
			 	break;
#if 0
		 	case DCAST_SB2CMD_AUDIO_VIB_CTL:
			 	rsp_ptr = (void *)diagSndAudioVibCtl(req_ptr,pkt_len);
			 	break;
#endif 				
			case DCAST_SB2CMD_SET_VOC_PATH_ADJ_DATA:		/* voc_data_voc_adj_data */
				rsp_ptr = (void *)diagSndSetVocPathAdjData(req_ptr,pkt_len);
				break;
		 
		 	default:	 /* nothing to do */
				// rsp_ptr = (void *) diagpkt_err_rsp (DIAG_BAD_PARM_F, req_ptr, pkt_len);
			 	break;
	 }
	 //------------------------------------------------------------------------
	 /* Execption*/  
	 if (rsp_ptr == NULL){
		 return NULL;
	 }
	
  }
  else
  {
	 //return diagpkt_err_rsp (DIAG_BAD_CMD_F, req_pkt_ptr, pkt_len);
  }
  rsp_ptr->cmd_code  =	 req_ptr->cmd_code;
  rsp_ptr->sub_cmd1  =	 req_ptr->sub_cmd1;
  rsp_ptr->sub_cmd2  =	 req_ptr->sub_cmd2;
  rsp_ptr->sub_cmd3  =	 req_ptr->sub_cmd3;
   return (rsp_ptr);


}

EXPORT_SYMBOL(LGF_AudioTest);
#if 0
void* linux_app_handler(audio_test_req_type*	pReq, DIAG_AUDIO_TEST_F_rsp_type	*pRsp)
{  
	diagpkt_free(pRsp);
  return 0;
}

void* not_supported_command_handler(audio_test_req_type*	pReq, DIAG_AUDIO_TEST_F_rsp_type	*pRsp)
{
	pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
  return pRsp;
}

#endif 

typedef enum{
	HW_VOLMIN,
	HW_VOLMAX,
	HW_MAX,
}vol_type;



static void * diagSndGetVocPathAdjData (void *req_pkt, word req_len)
{
	DIAG_AUDIO_TEST_F_req_type *req = (DIAG_AUDIO_TEST_F_req_type*)req_pkt;
	DIAG_AUDIO_TEST_F_rsp_type *rsp = NULL;
	unsigned int rsp_len =sizeof(DIAG_AUDIO_TEST_F_rsp_type);

	byte *pSendData	= NULL;
	rsp_len += sizeof(int16_t)*HW_MAX*6;
	if((rsp = (DIAG_AUDIO_TEST_F_rsp_type *) diagpkt_alloc (DIAG_AUDIO_TEST_F, rsp_len))==NULL)return rsp;
	rsp->status	= DCAST_SND_SUCCESS; 
	rsp->param1	= req->param1;
	rsp->param2	= req->param2;
	rsp->param3	= req->param3; 
	rsp->length	= rsp_len - sizeof(DIAG_AUDIO_TEST_F_rsp_type); 	
	pSendData	= (byte*)&(rsp->cast_data[0]);
	q6_get_volume((int16_t*)pSendData); //((int*)pSendData,rsp->length);

	return rsp;	

}
/*===========================================================================

FUNCTION diagSndSetVocPathAdjData

DESCRIPTION
	CAST 2.0
DEPENDENCIES
  lgeaudiohw.c

RETURN VALUE
  Pointer to response packet.

SIDE EFFECTS
  None.
 
===========================================================================*/
static void * diagSndSetVocPathAdjData (void *req_pkt, word req_len)
{
	DIAG_AUDIO_TEST_F_req_type *req = (DIAG_AUDIO_TEST_F_req_type*)req_pkt;
	DIAG_AUDIO_TEST_F_rsp_type *rsp = NULL;
	unsigned int rsp_len =sizeof(DIAG_AUDIO_TEST_F_rsp_type);
	unsigned int nDataLen = sizeof(int16_t)*HW_MAX*6;
	byte *pGetData	= NULL;

	if((rsp = (DIAG_AUDIO_TEST_F_rsp_type *) diagpkt_alloc (DIAG_AUDIO_TEST_F, rsp_len))==NULL)return rsp;
	rsp->status	= DCAST_SND_SUCCESS; 
	rsp->param1	= req->param1;
	rsp->param2	= req->param2;
	rsp->param3	= req->param3; 
	rsp->length	= rsp_len - sizeof(DIAG_AUDIO_TEST_F_rsp_type); 	
	pGetData	= (byte*)&(req->cast_data[0]);

	if(req->length < nDataLen){
		rsp->status	= DCAST_SND_BAD_RARAM;
		return rsp;
	}
#ifdef CONFIG_LGE_AUDIO_TUNNING
	q6_set_volume((int16_t *)pGetData);
#endif

	return rsp;	
}


/*===========================================================================

FUNCTION diagSndAudioAmpCtl

DESCRIPTION
	CAST 2.0
DEPENDENCIES
  audio_drv.c

RETURN VALUE
  Pointer to response packet.

SIDE EFFECTS
  None.
 
===========================================================================*/

static void * diagSndAudioAmpCtl (void *req_pkt, word req_len)
{
	DIAG_AUDIO_TEST_F_rsp_type *rsp = NULL;
	DIAG_AUDIO_TEST_F_req_type *req = (DIAG_AUDIO_TEST_F_req_type*)req_pkt;
	unsigned int rsp_len =sizeof(DIAG_AUDIO_TEST_F_rsp_type);
	//byte *pSendData	= NULL;
	diagCastAudAmpSubCmd3Type ampCtl = (diagCastAudAmpSubCmd3Type)req->sub_cmd3;
//	byte data[3];
	
	if((rsp = (DIAG_AUDIO_TEST_F_rsp_type *) diagpkt_alloc (DIAG_AUDIO_TEST_F, rsp_len))==NULL)return rsp;
	rsp->status	= DCAST_SND_SUCCESS; 
	rsp->param1	= req->param1;
	rsp->param2	= req->param2;
	rsp->param3	= req->param3; 
	rsp->length	= 0; 	

	switch(ampCtl)
	{
#if 0
		case DCAST_SB3CMD_AUDAMP_INIT:
			if(AUDAMP_CURRDRV_FUNC.audAmpDrv_Init())
				rsp->status	= DCAST_SND_SUCCESS;
			else
				rsp->status	= DCAST_SND_SUCCESS;
			break;
#endif 				
		case DCAST_SB3CMD_AUDAMP_PATHCTL:
			if(AMPDRV_SUCCESS==audAmpLM49250_PathCtrl((audAmpDrv_path_type)req->param1,(int)req->param2))
				rsp->status	= DCAST_SND_SUCCESS;
			else
				rsp->status	= DCAST_SND_NOT_SUPPORT;
			break;		
		
		case DCAST_SB3CMD_AUDAMP_SETPARAM:
			if(AMPDRV_SUCCESS==audAmpLM49250_SetParam((audAmpDrv_param_type)req->param1,(int)req->param2))
				rsp->status	= DCAST_SND_SUCCESS;
			else
				rsp->status	= DCAST_SND_NOT_SUPPORT;
			break;					
		case DCAST_SB3CMD_AUDAMP_GETPARAM:
			if(AMPDRV_SUCCESS==audAmpLM49250_GetParam((audAmpDrv_param_type)req->param1,(int*)&rsp->param2))
				rsp->status	= DCAST_SND_SUCCESS;
			else
				rsp->status	= DCAST_SND_NOT_SUPPORT;
			break;					
#if 0
		case DCAST_SB3CMD_AUDAMP_I2CWRITE:
			data[0] = req->param1;
			data[1] = req->param2;
			data[2] = req->param3;
			if(data[2]==0)data[2] =1;
			if(AMPDRV_SUCCESS==AUDAMP_CURRDRV_FUNC.audAmp_I2c_write(data[1],&data[0],data[2]))
				rsp->status	= DCAST_SND_SUCCESS;
			else
				rsp->status	= DCAST_SND_FAILURE;
			break;	
#endif 			

		case DCAST_SB3CMD_AUDVIB_SETPARAM:
			if(AMPDRV_SUCCESS==audVib_SetParam((audVibDrv_param_type)req->param1,(int)req->param2))
				rsp->status	= DCAST_SND_SUCCESS;
			else
				rsp->status	= DCAST_SND_NOT_SUPPORT;
			break;		

		case DCAST_SB3CMD_AUDVIB_PLAY:	
			if(AMPDRV_SUCCESS==audVib_Play())
				rsp->status	= DCAST_SND_SUCCESS;
			else
				rsp->status	= DCAST_SND_NOT_SUPPORT;
		
			break;					

		case DCAST_SB3CMD_AUDVIB_STOP:
			if(AMPDRV_SUCCESS==audVib_Stop())
				rsp->status	= DCAST_SND_SUCCESS;
			else
				rsp->status	= DCAST_SND_NOT_SUPPORT;
	
			break;					
			

		default:
			rsp->status	=	DCAST_SND_INVALID_REQ;
			break;
	}

	return rsp;		

}
#if 0
/*===========================================================================

FUNCTION diagSndAudioAmpCtl

DESCRIPTION
	CAST 2.0
DEPENDENCIES
  audio_drv.c

RETURN VALUE
  Pointer to response packet.

SIDE EFFECTS
  None.
 
===========================================================================*/
static void * diagSndAudioVibCtl (void *req_pkt, word req_len)
{
	DIAG_AUDIO_TEST_F_rsp_type *rsp = NULL;
	DIAG_AUDIO_TEST_F_req_type *req = (DIAG_AUDIO_TEST_F_req_type*)req_pkt;
	unsigned int rsp_len =sizeof(DIAG_AUDIO_TEST_F_rsp_type);
	//byte *pSendData	= NULL;
	diagCastAudVibSubCmd3Type VibCtl = (diagCastAudVibSubCmd3Type)req->sub_cmd3;
//	byte data[3];
	
	if((rsp = (DIAG_AUDIO_TEST_F_rsp_type *) diagpkt_alloc (DIAG_AUDIO_TEST_F, rsp_len))==NULL)return rsp;
	rsp->status	= DCAST_SND_SUCCESS; 
	rsp->param1	= req->param1;
	rsp->param2	= req->param2;
	rsp->param3	= req->param3; 
	rsp->length	= 0; 	

	switch(VibCtl)
	{		
		case DCAST_SB3CMD_AUDVIB_SETPARAM:
			if(AMPDRV_SUCCESS==audVib_SetParam((audVibDrv_param_type)req->param1,(int)req->param2))
				rsp->status	= DCAST_SND_SUCCESS;
			else
				rsp->status	= DCAST_SND_NOT_SUPPORT;
			break;					
		case DCAST_SB3CMD_AUDVIB_GETPARAM:
			//if(AMPDRV_SUCCESS==audAmpLM49250_GetParam((audAmpDrv_param_type)req->param1,(int*)&rsp->param2))
			//	rsp->status	= DCAST_SND_SUCCESS;
			//else
				rsp->status	= DCAST_SND_NOT_SUPPORT;
			break;					

		case DCAST_SB3CMD_AUDVIB_PLAY:
			
			break;	

		case DCAST_SB3CMD_AUDVIB_STOP:
			
			break;	
			
			
		default:
			rsp->status	=	DCAST_SND_INVALID_REQ;
			break;
	}

	return rsp;		

}
#endif 
#if 0
/*  USAGE
  *    1. If you want to handle at ARM9 side, you have to insert fun_ptr as NULL and mark ARM9_PROCESSOR
  *    2. If you want to handle at ARM11 side , you have to insert fun_ptr as you want and mark AMR11_PROCESSOR.
  */

audiotest_user_table_entry_type audiotest_mstr_tbl[AUDIOTEST_MSTR_TBL_SIZE] =
{ 
/*    sub_command                                              fun_ptr                      which procesor              */
	{ AUDIO_TEST_VERSION 					,	NULL			, ARM11_PROCESSOR},
	{ AUDIO_TEST_AMP_CONTROL_CMD 			,	NULL			, ARM11_PROCESSOR},	
};
#endif 
#endif
