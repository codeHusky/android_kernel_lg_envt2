/*===========================================================================

                     INCLUDE FILES FOR MODULE

===========================================================================*/
#include <linux/module.h>
#include <mach/lg_diagcmd.h>
#include <mach/lg_diag_mtc.h>

#include <linux/unistd.h> /*for open/close*/
#include <linux/fcntl.h> /*for O_RDWR*/

#include <linux/fb.h> /* to handle framebuffer ioctls */
#include <linux/ioctl.h>
#include <linux/uaccess.h>

#include <linux/syscalls.h> //for sys operations

#include <linux/input.h> // for input_event
#include <linux/fs.h> // for file struct
#include <linux/types.h> // for ssize_t, size_t
#include <linux/input.h> // for event parameters
#include <linux/jiffies.h>

#ifndef LG_FW_DUAL_TOUCH
#define LG_FW_DUAL_TOUCH
#endif
/*===========================================================================

                      EXTERNAL FUNCTION AND VARIABLE DEFINITIONS

===========================================================================*/
extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
//extern PACK(void *) diagpkt_free (PACK(void *)pkt);
//extern void send_to_arm9( void*	pReq, void	*pRsp);

extern void* lg_diag_req_pkt_ptr;
extern uint16 lg_diag_req_pkt_length;
//extern uint16 lg_diag_rsp_pkt_length;

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
extern unsigned long int ats_mtc_log_mask;
extern void diagpkt_commit (PACK(void *)pkt);
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/

/*===========================================================================

            LOCAL DEFINITIONS AND DECLARATIONS FOR MODULE

  This section contains local definitions for constants, macros, types,
  variables and other items needed by this module.

===========================================================================*/
#ifndef ETA_MTC_BMP_DATA
#define ETA_MTC_BMP_DATA "/data/eta.bmp"
#define ETA_MTC_BMP_HEADER_CNT 54
#endif

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
#define JIFFIES_TO_MS(t) ((t) * 1000 / HZ)
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/

extern mtc_user_table_entry_type mtc_mstr_tbl[MTC_MSTR_TBL_SIZE];

unsigned char g_diag_mtc_check = 0;

static char bmp_data_array[MTC_SCRN_BUF_SIZE_MAX];

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
byte eta_mtc_Convert_HS_to_Factory_Key(byte hs_key);
#endif

/*===========================================================================

                      INTERNAL FUNCTION DEFINITIONS

===========================================================================*/
PACK (void *)LGF_MTCProcess (
        PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
        unsigned short		pkt_len )		      /* length of request packet   */
{
  DIAG_MTC_F_req_type *req_ptr = (DIAG_MTC_F_req_type *) req_pkt_ptr;
  DIAG_MTC_F_rsp_type *rsp_ptr = NULL;
  mtc_func_type func_ptr= NULL;
  int nIndex = 0;
//  int exec_result =0;
  g_diag_mtc_check = 1;

  lg_diag_req_pkt_ptr = req_pkt_ptr;
  lg_diag_req_pkt_length = pkt_len;

  for( nIndex = 0 ; nIndex < MTC_MSTR_TBL_SIZE  ; nIndex++)
  {
    if( mtc_mstr_tbl[nIndex].cmd_code == req_ptr->hdr.sub_cmd)
    {
      if( mtc_mstr_tbl[nIndex].which_procesor == MTC_ARM11_PROCESSOR)
        func_ptr = mtc_mstr_tbl[nIndex].func_ptr;

      break;
    }
    else if (mtc_mstr_tbl[nIndex].cmd_code == MTC_MAX_CMD)
      break;
    else
      continue;
  }

  printk(KERN_INFO "%s, cmd_code : [0x%X], sub_cmd : [0x%X]\n", __func__, req_ptr->hdr.cmd_code, req_ptr->hdr.sub_cmd);

  if( func_ptr != NULL)
  {
    rsp_ptr = func_ptr((DIAG_MTC_F_req_type*)req_ptr);
  }
 // else
 //   send_to_arm9((void*)req_ptr, (void*)rsp_ptr);

//  diagpkt_free(rsp_ptr);
  return (rsp_ptr);
//return (0);
}

EXPORT_SYMBOL(LGF_MTCProcess);

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
DIAG_MTC_F_rsp_type* mtc_logging_mask_req_proc(DIAG_MTC_F_req_type *pReq)
{
  unsigned int rsp_len;
  DIAG_MTC_F_rsp_type *pRsp;

  rsp_len = sizeof(mtc_log_req_type);
  printk(KERN_INFO "%s, rsp_len :(%d)\n", __func__, rsp_len);
  pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  if (pRsp == NULL) {
    printk(KERN_ERR "%s, diagpkt_alloc failed\n", __func__);
    return pRsp;
  }

  switch(pReq->mtc_req.log.mask)
  {
    case 0x00000000://ETA_LOGMASK_DISABLE_ALL:
    case 0xFFFFFFFF://ETA_LOGMASK_ENABLE_ALL:
    case 0x00000001://ETA_LOGITEM_KEY:
    case 0x00000002://ETA_LOGITEM_TOUCHPAD:
    case 0x00000003://ETA_LOGITME_KEYTOUCH:
      ats_mtc_log_mask = pReq->mtc_req.log.mask;
      pRsp->mtc_rsp.log.mask = ats_mtc_log_mask;
      break;

    default:
      ats_mtc_log_mask = 0x00000000; // //ETA_LOGMASK_DISABLE_ALL
      pRsp->mtc_rsp.log.mask = ats_mtc_log_mask;
      break;
  }

  return pRsp;
}

void mtc_send_key_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log)
{
  unsigned int rsp_len;
  DIAG_MTC_F_rsp_type *pRsp;
  byte hs_key_code;
  
  rsp_len = sizeof(mtc_log_data_rsp_type);
  printk(KERN_INFO "%s, rsp_len :(%d)\n", __func__, rsp_len);
  pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  if (pRsp == NULL) {
  printk(KERN_ERR "%s, diagpkt_alloc failed\n", __func__);
  	diagpkt_commit (pRsp);
  }

  pRsp->hdr.cmd_code = DIAG_MTC_F;
  pRsp->hdr.sub_cmd = MTC_LOG_REQ_CMD;

  pRsp->mtc_rsp.log_data.log_id = p_ats_mtc_key_log->log_id; //LOG_ID, 1 key, 2 touch
  pRsp->mtc_rsp.log_data.log_len = p_ats_mtc_key_log->log_len; //LOG_LEN
  
  if(p_ats_mtc_key_log->log_id == 1) // key
  {
    hs_key_code = eta_mtc_Convert_HS_to_Factory_Key((byte)p_ats_mtc_key_log->y_code);
    printk(KERN_INFO "%s, origin key : 0x%X(%C)\n", __func__, (byte)p_ats_mtc_key_log->y_code, (byte)p_ats_mtc_key_log->y_code);
    printk(KERN_INFO "%s, translated key : 0x%X(%C)\n", __func__, (byte)hs_key_code, (byte)hs_key_code);    

    pRsp->mtc_rsp.log_data.log_type.log_data_key.time = (unsigned long long)JIFFIES_TO_MS(jiffies);
    pRsp->mtc_rsp.log_data.log_type.log_data_key.hold = (unsigned char)((p_ats_mtc_key_log->x_hold)&0xFF);// hold
    pRsp->mtc_rsp.log_data.log_type.log_data_key.keycode = (unsigned char)hs_key_code;//key code
    pRsp->mtc_rsp.log_data.log_type.log_data_key.active_uiid = 0;
  }
  else // touch
  {
    pRsp->mtc_rsp.log_data.log_type.log_data_touch.time = (unsigned long long)JIFFIES_TO_MS(jiffies);
    pRsp->mtc_rsp.log_data.log_type.log_data_touch.screen_id = (unsigned char)1; // MAIN LCD
    pRsp->mtc_rsp.log_data.log_type.log_data_touch.action = (unsigned char)p_ats_mtc_key_log->action;
    pRsp->mtc_rsp.log_data.log_type.log_data_touch.x = (unsigned short)p_ats_mtc_key_log->x_hold;
    pRsp->mtc_rsp.log_data.log_type.log_data_touch.y = (unsigned short)p_ats_mtc_key_log->y_code;
    pRsp->mtc_rsp.log_data.log_type.log_data_touch.active_uiid = 0;
  }

  diagpkt_commit (pRsp);
}

EXPORT_SYMBOL(mtc_send_key_log_data);

#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/

DIAG_MTC_F_rsp_type* mtc_serialized_data_req_proc(DIAG_MTC_F_req_type *pReq)
{
  unsigned int rsp_len;
  DIAG_MTC_F_rsp_type *pRsp;
  static unsigned long bmp_sent_cnt = 0; // save PMP data sent count
  static ssize_t read_size = 0; // save BMP data read count
  unsigned int left_size = 0;
  
  struct file *phMscd_Filp = NULL;
  
  mm_segment_t old_fs;

  if(bmp_sent_cnt == 0) // read only once
  {
    old_fs=get_fs();
    set_fs(get_ds());

    phMscd_Filp = filp_open(ETA_MTC_BMP_DATA, O_RDONLY |O_LARGEFILE, 0);
    if( !phMscd_Filp)
      printk(KERN_ERR "%s, open fail screen capture \n", __func__);

    printk(KERN_INFO "%s, %s offset : %d\n", __func__, ETA_MTC_BMP_DATA, (int)phMscd_Filp->f_pos);
    phMscd_Filp->f_pos = ETA_MTC_BMP_HEADER_CNT;
    printk(KERN_INFO "%s, %s changed offset : %d for bmp header\n", __func__, ETA_MTC_BMP_DATA, (int)phMscd_Filp->f_pos);

    read_size = phMscd_Filp->f_op->read(phMscd_Filp, bmp_data_array, MTC_SCRN_BUF_SIZE_MAX, &phMscd_Filp->f_pos);
    printk(KERN_INFO "%s, read size : %d(0x%X) \n", __func__, read_size, read_size);

    filp_close(phMscd_Filp,NULL);
    set_fs(old_fs);
  }

  rsp_len = sizeof(mtc_serialized_data_rsp_type);
#ifdef MTC_DBG
  printk(KERN_INFO "%s, rsp_len :(%d)\n", __func__, rsp_len);
#endif
  pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  if (pRsp == NULL) {
    printk(KERN_ERR "%s, diagpkt_alloc failed\n", __func__);
    return pRsp;
  }

  pRsp->hdr.cmd_code = DIAG_MTC_F;
  pRsp->hdr.sub_cmd = MTC_SERIALIZED_DATA_REQ_CMD;
  
  if((bmp_sent_cnt+1)*MTC_SCRN_BUF_SIZE < read_size) // more than data pkt size
  {
    pRsp->mtc_rsp.serial_data.seqeunce = bmp_sent_cnt;
    pRsp->mtc_rsp.serial_data.length = MTC_SCRN_BUF_SIZE;

#ifdef MTC_DBG
    memset((void*)pRsp->mtc_rsp.serial_data.bmp_data, 0, MTC_SCRN_BUF_SIZE);  
#endif
    memcpy((void*)pRsp->mtc_rsp.serial_data.bmp_data, (void*)(bmp_data_array+bmp_sent_cnt*MTC_SCRN_BUF_SIZE), MTC_SCRN_BUF_SIZE);

    bmp_sent_cnt++;

    left_size = read_size - bmp_sent_cnt*MTC_SCRN_BUF_SIZE;
#ifdef MTC_DBG
    printk(KERN_INFO "%s, sent_cnt : %d(0x%X), left : %d(0x%X)\n", __func__, bmp_sent_cnt, bmp_sent_cnt, left_size, left_size);
#endif
  }
  else // less than data pkt size
  {
    pRsp->mtc_rsp.serial_data.seqeunce = 0xFFFF;

    pRsp->mtc_rsp.serial_data.length = read_size - bmp_sent_cnt*MTC_SCRN_BUF_SIZE;
    memset((void*)pRsp->mtc_rsp.serial_data.bmp_data, 0, MTC_SCRN_BUF_SIZE);
    memcpy((void*)pRsp->mtc_rsp.serial_data.bmp_data, (void*)(bmp_data_array+bmp_sent_cnt*MTC_SCRN_BUF_SIZE), pRsp->mtc_rsp.serial_data.length);
  
    bmp_sent_cnt = 0;

    printk(KERN_INFO "%s, last_pkt len : %d(0x%X)\n", __func__, pRsp->mtc_rsp.serial_data.length, pRsp->mtc_rsp.serial_data.length);
  }
  
  return pRsp;
}

byte eta_mtc_Convert_HS_to_Factory_Key(byte hs_key)
{
  byte val;
  
  switch(hs_key)
  {
// Digital keyboard buttons
    case KEY_0: val = '0';			break;	/* ZERO */
    case KEY_1: val = '1';			break;	/* ONE */
    case KEY_2: val = '2';			break;	/* TWO */
    case KEY_3: val = '3';			break;	/* THREE */
    case KEY_4: val = '4';			break;	/* FOUR */
    case KEY_5: val = '5';			break;	/* FIVE */
    case KEY_6: val = '6';			break;	/* SIX */
    case KEY_7: val = '7';			break;	/* SEVEN */
    case KEY_8: val = '8';			break;	/* EIGHT */
    case KEY_9: val = '9';			break;	/* NINE */
  	
// Star
// Sharp
    case KEY_SEND: val = 'S';		break;	/* Call establish button */
    case KEY_END: val = 'E';			break;	/* End call button */
    
// Right soft key
// Left soft key
    case 243: val = 'G';				break;	/* OK */
//VS740 case 0xF5: val = 'G';			break;	/* OK */
    
    case KEY_BACKSPACE: val = 'Y';break;	/* C-button (erase symbol) */
  
  /*	VS740
  	case KEY_RIGHT: val = '/';		break;	// Up button
  	case KEY_LEFT: val = 'V';		break;	// Down button
  	case KEY_DOWN: val = 'R';		break;	// Right button (rotate clockwise)
  	case KEY_UP: val = 'L';			break;	// Left button (rotate counter-clockwise)
  */
    case KEY_LEFT: val = '/';			break;	/* Up button */
    case KEY_RIGHT: val = 'V';		break;	/* Down button */
    case KEY_UP: val = 'R';			break;	/* Right button (rotate clockwise) */
    case KEY_DOWN: val = 'L';		break;	/* Left button (rotate counter-clockwise) */
    
    case KEY_VOLUMEUP: val = 'U';	break;	/* Volume up button */
    case KEY_VOLUMEDOWN: val = 'D';break;/* Volume down button */
    
    case KEY_CAMERA: val = 'A';		break;	/* Camera enable button */
  
// Open flip action
// Close flip action
// FF button on the flip
// Rewind button on the flip
// Play button on the flip
  
    case KEY_MENU: val = 'O';		break;	/* Menu button -internal */
    case 0xF4: val = 'O';			break;	/* Menu button -external */
    
    case KEY_BACK: val = '^';		break;	/* Back button */
  #ifdef CONFIG_LGE_AUDIO_HAPTIC_TOUCH_SOFT_KEY
    case TOUCH_BACK: val = '^';		break;	/* Back button */
  #endif /*CONFIG_LGE_AUDIO_HAPTIC_TOUCH_SOFT_KEY*/
  
// Videocall button
// Open operator portal - Dedicated button for opening WAP/WEB-site
// MP3/multimedia button - If dedicated button exists
// Extra side button - Specific side button (Bluetooth, phone book)
// Rotate second wheel up - Second rotator (ex. KG330)
// Rotate second wheel down - Second rotator (ex. KG330)
// Touch activation button - Touch keypad Screen activation (ex. KS360)
// Lock button - Handset Lock
// Dual SIM button - Dual SIM activation
  	
    case 0xF2: val = 'Z';			break;	/* Camera focus button */
  
// Multitasking Button
  	
  // Qwerty Key ¡®A¡¯ ~ ¡®Z¡¯- Small letter
    case KEY_A: val = 'a';			break;
    case KEY_B: val = 'b';			break;
    case KEY_C: val = 'c';			break;
    case KEY_D: val = 'd';			break;
    case KEY_E: val = 'e';			break;
    case KEY_F: val = 'f';			break;
    case KEY_G: val = 'g';			break;
    case KEY_H: val = 'h';			break;
    case KEY_I: val = 'i';			break;
    case KEY_J: val = 'j';			break;
    case KEY_K: val = 'k';			break;
    case KEY_L: val = 'l';			break;
    case KEY_M: val = 'm';			break;
    case KEY_N: val = 'n';			break;
    case KEY_O: val = 'o';			break;
    case KEY_P: val = 'p';			break;
    case KEY_Q: val = 'q';			break;
    case KEY_R: val = 'r';			break;
    case KEY_S: val = 's';			break;
    case KEY_T: val = 't';			break;
    case KEY_U: val = 'u';			break;
    case KEY_V: val = 'v';			break;
    case KEY_W: val = 'w';			break;
    case KEY_X: val = 'x';			break;
    case KEY_Y: val = 'y';			break;
    case KEY_Z: val = 'z';			break;
    
    case KEY_SEARCH: val = '+';		break;	/* Qwerty Function */
  	
// Qwerty Clear - KEY_BACKSPACE has already been defined
  
    case KEY_LEFTALT: val = '$';		break;	/* Qwerty Symbol */
    case KEY_LEFTSHIFT: val = '~';	break;	/* Qwerty Shift */
    case KEY_ENTER: val = '=';		break;	/* Qwerty Enter */
    case KEY_SPACE: val = '_';		break;	/* Qwerty Space */
  
// Qwerty Message
// QWERTY Left Soft Key
// QWERTY Right Soft Key
  
    case KEY_HOME: val = '!';		break;	/* Qwerty Home key - internal */
//VS740 case 0xF3: val = '!';			break;	/* Qwerty Home key - external */

// Qwerty comma key
// Qwerty dot key
// Qwerty OK key - 0xF5 OK key has already been defined
  	
    default: val = ' ';				break;
  }
  
  return val;
}

DIAG_MTC_F_rsp_type* mtc_null_rsp(DIAG_MTC_F_req_type *pReq)
{
  unsigned int rsp_len;
  DIAG_MTC_F_rsp_type *pRsp;

  rsp_len = sizeof(mtc_req_hdr_type);
  printk(KERN_INFO "%s, mtc_null_rsp rsp_len :(%d)\n", __func__, rsp_len);
   
  pRsp = (DIAG_MTC_F_rsp_type *)diagpkt_alloc(DIAG_MTC_F, rsp_len);
  if (pRsp == NULL) {
     printk(KERN_ERR "%s, diagpkt_alloc failed\n", __func__);
  }

  pRsp->hdr.cmd_code = pReq->hdr.cmd_code;
  pRsp->hdr.sub_cmd = pReq->hdr.sub_cmd;

  return pRsp;
}

DIAG_MTC_F_rsp_type* mtc_execute(DIAG_MTC_F_req_type *pReq)
{
  int ret;
  char cmdstr[100];
  int fd;

  unsigned int req_len;
  unsigned int rsp_len;
  DIAG_MTC_F_rsp_type *pRsp;

  static unsigned long bmp_sent_cnt = 0;
  
  //int lenb64 = 0;
  
  char *envp[] = {
  	"HOME=/",
  	"TERM=linux",
  	NULL,
  };
  
  char *argv[] = {
  	"sh",
  	"-c",
  	cmdstr,
  	NULL,
  };

  printk(KERN_INFO "%s\n", __func__);

  if ( (fd = sys_open((const char __user *)MTC_MODULE, O_RDONLY ,0) ) < 0 )
  {
  	printk(KERN_ERR "\n%s, can not open %s\n", __func__, MTC_MODULE);
  	sprintf(cmdstr, "%s", MTC_MODULE);
  }
  else
  {
  	sprintf(cmdstr, "%s", MTC_MODULE);
  	sys_close(fd);
  }
  
  printk(KERN_INFO "%s, execute mtc : data - %s\n\n", __func__, cmdstr);
  if ((ret =
       call_usermodehelper("/system/bin/sh", argv, envp, UMH_WAIT_PROC)) != 0) {
  	printk(KERN_ERR "%s, MTC failed to run : %i\n", __func__, ret);
  }
  else
    printk(KERN_INFO "%s, execute ok, ret = %d\n", __func__, ret);
  
  return NULL;
}

EXPORT_SYMBOL(mtc_execute);

/*  USAGE (same as testmode
  *    1. If you want to handle at ARM9 side, you have to insert fun_ptr as NULL and mark ARM9_PROCESSOR
  *    2. If you want to handle at ARM11 side , you have to insert fun_ptr as you want and mark AMR11_PROCESSOR.
  */
mtc_user_table_entry_type mtc_mstr_tbl[MTC_MSTR_TBL_SIZE] =
{ 
/*	sub_command							fun_ptr							which procesor              */
	{ MTC_INFO_REQ_CMD					,mtc_execute					, MTC_ARM11_PROCESSOR},
	{ MTC_CAPTURE_REQ_CMD				,mtc_null_rsp					, MTC_ARM11_PROCESSOR},
	{ MTC_KEY_EVENT_REQ_CMD			,mtc_execute					, MTC_ARM11_PROCESSOR},
	{ MTC_TOUCH_REQ_CMD				,mtc_execute					, MTC_ARM11_PROCESSOR},
#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
	{ MTC_LOGGING_MASK_REQ_CMD		,mtc_logging_mask_req_proc		, MTC_ARM11_PROCESSOR},
	{ MTC_LOG_REQ_CMD					,mtc_null_rsp					, MTC_ARM11_PROCESSOR}, /*mtc_send_key_log_data*/
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
	{ MTC_SERIALIZED_DATA_REQ_CMD		,mtc_serialized_data_req_proc	, MTC_ARM11_PROCESSOR},
	{ MTC_SERIALIZED_CAPTURE_REQ_CMD 	,mtc_execute					, MTC_ARM11_PROCESSOR},
	{ MTC_PHONE_RESTART_REQ_CMD		,mtc_null_rsp					, MTC_ARM9_PROCESSOR},
	{ MTC_FACTORY_RESET				,mtc_null_rsp					, MTC_ARM9_ARM11_BOTH},
	{ MTC_PHONE_REPORT					,mtc_null_rsp					, MTC_ARM9_PROCESSOR},
	{ MTC_PHONE_STATE					,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_CAPTURE_PROP					,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_NOTIFICATION_REQUEST			,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_CUR_PROC_NAME_REQ_CMD		,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_KEY_EVENT_UNIV_REQ_CMD		,mtc_null_rsp					, MTC_NOT_SUPPORTED}, /*ETA command*/
	{ MTC_MEMORY_DUMP					,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_BATTERY_POWER				,mtc_null_rsp					, MTC_ARM9_PROCESSOR},
	{ MTC_BACKLIGHT_INFO				,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_FLASH_MODE					,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_MODEM_MODE					,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_CELL_INFORMATION				,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_HANDOVER						,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_ERROR_CMD					,mtc_null_rsp					, MTC_NOT_SUPPORTED},
	{ MTC_MAX_CMD						,mtc_null_rsp					, MTC_NOT_SUPPORTED},
};
