#include <linux/module.h>
#include <mach/lg_diagcmd.h>
#include <linux/input.h>
#include <linux/syscalls.h>


#include "lg_diag_communication.h" // for struct diagcmd_dev
#include <mach/lg_diag_testmode.h>
#include <linux/delay.h>
/* ==========================================================================
===========================================================================*/

#ifdef CONFIG_LGE_DIAG_TESTMODE
/* ==========================================================================
===========================================================================*/
int testmode_volume_state =-1;

static struct diagcmd_dev *diagpdev;

extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
extern PACK(void *) diagpkt_free (PACK(void *)pkt);
extern void send_to_arm9( void*	pReq, void	*pRsp);
extern testmode_user_table_entry_type testmode_mstr_tbl[TESTMODE_MSTR_TBL_SIZE];
extern void set_opertion_mode(boolean isOnline);
extern int q6audio_set_rx_volume(int level);

/* ==========================================================================
===========================================================================*/
#define KEY_TRANS_MAP_SIZE 60

typedef struct {
  word LG_common_key_code;
  unsigned int Android_key_code;
}keycode_trans_type;

keycode_trans_type keytrans_table[KEY_TRANS_MAP_SIZE]={
    {0x4E        ,   243 },     // folder home
    {0x4F        ,   244 },     // folder menu
    {0x50        ,   KEY_SEND },
    {0x51        ,   KEY_END },
    {0x92        ,   KEY_VOLUMEUP },
    {0x93        ,   KEY_VOLUMEDOWN},
    {0x8F        ,   KEY_CAMERA },
      
    {0x2030     ,   KEY_0},     {0x2031     ,   KEY_1},     {0x2032     ,   KEY_2},     {0x2033     ,   KEY_3},
    {0x2034     ,   KEY_4},     {0x2035     ,   KEY_5},     {0x2036     ,   KEY_6},     {0x2037     ,   KEY_7}, 
    {0x2038     ,   KEY_8},     {0x2039     ,   KEY_9},      

    {0x2041     ,   KEY_A},  {0x2042     ,   KEY_B}, {0x2043     ,   KEY_C},   {0x2044     ,   KEY_D}, 
    {0x2045     ,   KEY_E},  {0x2046     ,   KEY_F},  {0x2047     ,   KEY_G},   {0x2048     ,   KEY_H}, 
    {0x2049     ,   KEY_I},   {0x204A     ,   KEY_J},  {0x204B     ,   KEY_K},   {0x204C     ,   KEY_L}, 
    {0x204D     ,   KEY_M},  {0x204E     ,   KEY_N}, {0x204F     ,   KEY_O},   {0x2050     ,   KEY_P}, 
    {0x2051     ,   KEY_Q},  {0x2052     ,   KEY_R}, {0x2053     ,   KEY_S},   {0x2054     ,   KEY_T}, 
    {0x2055     ,   KEY_U},  {0x2056     ,   KEY_V}, {0x2057     ,   KEY_W},  {0x2058     ,   KEY_X}, 
    {0x2059     ,   KEY_Y},  {0x205A     ,   KEY_Z}, 

    {0x1010     ,   KEY_LEFT},  {0x1011     ,   KEY_RIGHT},   {0x1054     ,   KEY_UP},   {0x1055     ,   KEY_DOWN},
    {0x1053     ,   245},   // navi ok
    {0x101D     ,   KEY_ENTER},
    {0x1020     ,   KEY_SPACE},

    {0x1030     ,   KEY_HOME},
    {0x1031     ,   KEY_MENU},
    {0x1032     ,   KEY_BACKSPACE},
    {0x1033     ,   KEY_BACK},
    {0x1034     ,   KEY_SEARCH},
    {0x1035     ,   KEY_LEFTALT},
    {0x1036     ,   KEY_LEFTSHIFT},
    {0x1037     ,   KEY_RIGHTSHIFT},
    {0x2A	, 227},
    {0x23	, 228},
	/* vs760 Setting */
};

keycode_trans_type modekeytrans_table[17]={
    {0x2A	, 227},
    {0x23	, 228},

    {48     ,   KEY_0},     {49     ,   KEY_1},     {50     ,   KEY_2},     {51     ,   KEY_3},
    {52     ,   KEY_4},     {53     ,   KEY_5},     {54     ,   KEY_6},     {55     ,   KEY_7}, 
    {56     ,   KEY_8},     {57     ,   KEY_9},      

    {0x50        ,   KEY_SEND },
    {0x51        ,   KEY_END },
// 2010.12.29 jihoon.lee - change KEY_CLEAR to KEY_BACKSPACE
    {0x52		 , KEY_BACKSPACE },
    {150        ,   KEY_VOLUMEUP },
    {151        ,   KEY_VOLUMEDOWN},
};

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


PACK (void *)LGF_TestMode (
        PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
        uint16		pkt_len )		      /* length of request packet   */
{
  DIAG_TEST_MODE_F_req_type *req_ptr = (DIAG_TEST_MODE_F_req_type *) req_pkt_ptr;
  DIAG_TEST_MODE_F_rsp_type *rsp_ptr;
  unsigned int rsp_len;
  testmode_func_type func_ptr= NULL;
  int nIndex = 0;

#ifdef CONFIG_LGE_AUDIO_TESTMODE
	diagpdev = diagcmd_get_dev();
#endif

// resize the rsp_len, test_mode_rsp_type is not needed out of DIAG_TEST_MODE_F_rsp_type
#if defined (CONFIG_LGE_APPS_FACTORY_RESET)
  switch(req_ptr->sub_cmd_code)
  {
    case TEST_MODE_FACTORY_RESET_CHECK_TEST:
//    case TEST_MODE_VIRTUAL_SIM_TEST:
      rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type);
      break;
    case TEST_MODE_TEST_SCRIPT_MODE:
      rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_test_script_mode_type);
      break;
    case TEST_MODE_FIRST_BOOTING_COMPLETE_CHECK:
      rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type);
      break;
    default :
      rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);
      break;
  }
#else /*CONFIG_LGE_APPS_FACTORY_RESET*/
  rsp_len = sizeof(DIAG_TEST_MODE_F_rsp_type);		
#endif /*CONFIG_LGE_APPS_FACTORY_RESET*/

  rsp_ptr = (DIAG_TEST_MODE_F_rsp_type *)diagpkt_alloc(DIAG_TEST_MODE_F, rsp_len);
  rsp_ptr->sub_cmd_code = req_ptr->sub_cmd_code;
  rsp_ptr->ret_stat_code = TEST_OK_S; // �ʱⰪ

  for( nIndex = 0 ; nIndex < TESTMODE_MSTR_TBL_SIZE  ; nIndex++)
  {
    if( testmode_mstr_tbl[nIndex].cmd_code == req_ptr->sub_cmd_code)
    {
        if( testmode_mstr_tbl[nIndex].which_procesor == ARM11_PROCESSOR)
          func_ptr = testmode_mstr_tbl[nIndex].func_ptr;
      break;
    }     
  }

  if( func_ptr != NULL)
    rsp_ptr = func_ptr( &(req_ptr->test_mode_req), rsp_ptr);
  else
    send_to_arm9((void*)req_ptr, (void*)rsp_ptr);
    
  return (rsp_ptr);
}
EXPORT_SYMBOL(LGF_TestMode);

void* linux_app_handler(test_mode_req_type*	pReq, DIAG_TEST_MODE_F_rsp_type	*pRsp)
{  
	diagpkt_free(pRsp);
  return 0;
}

void* not_supported_command_handler(test_mode_req_type*	pReq, DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
  return pRsp;
}

/* LCD QTEST */
PACK (void *)LGF_LcdQTest (
        PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
        uint16		pkt_len )		      /* length of request packet   */
{
	/* Returns 0 for executing lg_diag_app */
	return 0;
}
EXPORT_SYMBOL(LGF_LcdQTest);

/* TEST_MODE_BLUETOOTH_TEST */
void* LGF_TestModeBlueTooth(
        test_mode_req_type*	pReq,
        DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	printk(KERN_ERR "[_BTUI_] [%s:%d] BTSubCmd=<%d>\n", __func__, __LINE__, pReq->bt);

	if (diagpdev != NULL){
		update_diagcmd_state(diagpdev, "BT_TEST_MODE", pReq->bt);
		if(pReq->bt==1) msleep(5900); //6sec timeout
		else if(pReq->bt==2) ssleep(1);
		else ssleep(3);
		pRsp->ret_stat_code = TEST_OK_S;
	}
	else 
	{
		printk(KERN_ERR "[_BTUI_] [%s:%d] BTSubCmd=<%d> ERROR\n", __func__, __LINE__, pReq->bt);
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
  return pRsp;
}

void* LGF_TestPhotoSensor(
		test_mode_req_type* pReq ,
		DIAG_TEST_MODE_F_rsp_type	*pRsp)
{	
	pRsp->ret_stat_code = TEST_OK_S;

	if (diagpdev != NULL){
#ifdef CONFIG_LGE_AUDIO_TESTMODE
		update_diagcmd_state(diagpdev, "ALC", pReq->motor);
#endif
	}
	else 
	{
		printk("\n[%s] error MOTOR", __func__ );		
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
  return pRsp;
}
	
#ifdef CONFIG_LGE_AUDIO_TESTMODE
void* LGF_TestMotor(
		test_mode_req_type* pReq ,
		DIAG_TEST_MODE_F_rsp_type	*pRsp)
{	
	pRsp->ret_stat_code = TEST_OK_S;

	if (diagpdev != NULL){
		update_diagcmd_state(diagpdev, "MOTOR", pReq->motor);
	}
	else 
	{
		printk("\n[%s] error MOTOR", __func__ );		
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
  return pRsp;
}
	
void* LGF_TestAcoustic(
		test_mode_req_type* pReq ,
		DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
    pRsp->ret_stat_code = TEST_OK_S;
	
	if (diagpdev != NULL){
		update_diagcmd_state(diagpdev, "ACOUSTIC", pReq->acoustic);
	}
	else 
	{
		printk("\n[%s] error ACOUSTIC", __func__ );		
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
  return pRsp;
}

void* LGF_TestModeMP3 (
		test_mode_req_type* pReq ,
		DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;

	if (diagpdev != NULL){
		if(pReq->mp3_play == MP3_SAMPLE_FILE)
		{		
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		}
		else
		{
			update_diagcmd_state(diagpdev, "MP3", pReq->mp3_play);
		}
	}
	else 
	{
		printk("\n[%s] error MP3", __func__ );		
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
  return pRsp;
}

void* LGF_TestModeSpeakerPhone(
		test_mode_req_type* pReq ,
		DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;
	
	if (diagpdev != NULL){
		if((pReq->speaker_phone == NOMAL_Mic1) || (pReq->speaker_phone == NC_MODE_ON)
			|| (pReq->speaker_phone == ONLY_MIC2_ON_NC_ON) || (pReq->speaker_phone == ONLY_MIC1_ON_NC_ON)
		)
		{		
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
		}
		else
		{
			update_diagcmd_state(diagpdev, "SPEAKERPHONE", pReq->speaker_phone);
		}
	}
	else 
	{
		printk("\n[%s] error SPEAKERPHONE", __func__ );		
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
  return pRsp;
}

void* LGT_TestModeVolumeLevel (
		test_mode_req_type* pReq ,
		DIAG_TEST_MODE_F_rsp_type *pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;
	
	if (diagpdev != NULL){

		printk("\n[%s] LGT_TestModeVolumeLevel :%d", __func__,pReq->volume_level );		
		switch (pReq->volume_level ) 
		{
	   		case VOL_LEV_OFF:
	   			testmode_volume_state = VOL_LEV_OFF;
				break;
			
			case VOL_LEV_MIN:
				testmode_volume_state = VOL_LEV_MIN;
				break;

			case VOL_LEV_MEDIUM:
				testmode_volume_state = VOL_LEV_MEDIUM;
				break;

			case VOL_LEV_MAX:
				testmode_volume_state = VOL_LEV_MAX;
				break;	
		}		

		update_diagcmd_state(diagpdev, "VOLUMELEVEL", pReq->volume_level);
		q6audio_set_rx_volume(100);
			
	}
	else 
	{
		printk("\n[%s] error VOLUMELEVEL", __func__ );		
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
  return pRsp;
}
#endif


char key_buf[MAX_KEY_BUFF_SIZE];
boolean if_condition_is_on_key_buffering = FALSE;
int count_key_buf = 0;

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-06-04, [VS760]*/
#if 1//defined(CONFIG_DIAG_LGE_TESTMODE)
boolean lgf_factor_key_test_rsp (char key_code)
{
    int index = 0;
    word buf = (word)key_code;  // if we can not find, return the org value. 

    printk("Testmode Key Test Enter");
    /* sanity check */
    if (count_key_buf>MAX_KEY_BUFF_SIZE)
        return FALSE;
    
    for( index = 0; index < KEY_TRANS_MAP_SIZE ; index++)
    {
      if( keytrans_table[index].Android_key_code == key_code)
      {
        buf = keytrans_table[index].LG_common_key_code;
        printk("%d key is pressed", buf);
        break;
      }
    }  

    key_buf[count_key_buf++] = buf;
    return TRUE;	
}
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-06-04, [VS760]*/
#else
boolean lgf_factor_key_test_rsp (char key_code)
{
    /* sanity check */
    if (count_key_buf>MAX_KEY_BUFF_SIZE)
        return FALSE;

    key_buf[count_key_buf++] = key_code;
    return TRUE;	
}
#endif
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-06-04, [VS760]*/

EXPORT_SYMBOL(lgf_factor_key_test_rsp);

void* LGT_TestModeKeyTest(test_mode_req_type* pReq, DIAG_TEST_MODE_F_rsp_type *pRsp)
{
  pRsp->ret_stat_code = TEST_OK_S;

  if(pReq->key_test_start)
	{
		if_condition_is_on_key_buffering=TRUE;
		memset((void *)key_buf,0x00,MAX_KEY_BUFF_SIZE);
		count_key_buf=0;
  }
  else 
  {
		if_condition_is_on_key_buffering=FALSE;
		memcpy((void *)((DIAG_TEST_MODE_KEY_F_rsp_type *)pRsp)->key_pressed_buf, (void *)key_buf, MAX_KEY_BUFF_SIZE);
  }
  return pRsp;
}

#ifdef CONFIG_LGE_CAMERA_TESTMODE
void* LGF_TestCam(
		test_mode_req_type* pReq ,
		DIAG_TEST_MODE_F_rsp_type	*pRsp)
{	
		pRsp->ret_stat_code = TEST_OK_S;

		switch(pReq->camera)
		{
			case CAM_TEST_SAVE_IMAGE:
			case CAM_TEST_CAMERA_SELECT:
				pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;

			default:
				if (diagpdev != NULL){
					
					update_diagcmd_state(diagpdev, "CAMERA", pReq->camera);
				}
				else 
				{
					printk("\n[%s] error CAMERA", __func__ );
					pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
				}
				break;
		}
	  return pRsp;
}

#endif
void LGF_SendKey(word keycode)
{
	extern struct input_dev *qwerty_get_input_dev(void);
	extern struct input_dev *hs_get_input_dev(void);
	struct input_dev *idev = qwerty_get_input_dev();
	// 2010.12.29 jihoon.lee - KEY_END should be sent using hs input device
	struct input_dev *idev_hs = hs_get_input_dev();

	if(keycode != KEY_END)
	{
	  input_report_key( idev,(unsigned int)keycode , 1 ); // press event
	  input_report_key( idev,(unsigned int)keycode , 0 ); // release	event
	}
	else
	{
	  input_report_key( idev_hs,(unsigned int)keycode , 1 ); // press event
	  input_sync( idev_hs );
	  mdelay(100);
	  input_report_key( idev_hs,(unsigned int)keycode , 0 ); // release	event
	  input_sync( idev_hs );
	}
}


unsigned int LGF_KeycodeTrans(word input)
{
  int index = 0;
  unsigned int ret = (unsigned int)input;  // if we can not find, return the org value. 
 
  for( index = 0; index < KEY_TRANS_MAP_SIZE ; index++)
  {
    if( keytrans_table[index].LG_common_key_code == input)
    {
      ret = keytrans_table[index].Android_key_code;
      break;
    }
  }  

  return ret;
}

unsigned int LGF_ModeKeycodeTrans(word input)
{
  int index = 0;
  unsigned int ret = (unsigned int)input;  // if we can not find, return the org value. 
 
  for( index = 0; index < 17 ; index++)
  {
    if( modekeytrans_table[index].LG_common_key_code == input)
    {
      ret = modekeytrans_table[index].Android_key_code;
      break;
    }
  }
  
  printk(KERN_INFO "%s, input : 0x%X, ret : %d\n", __func__, input, ret);

  return ret;
}


void* LGF_TestModeKeyData(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
  
  pRsp->ret_stat_code = TEST_OK_S;

  LGF_SendKey(LGF_ModeKeycodeTrans(pReq->key_data));

  return pRsp;
}

uint8_t if_condition_is_on_air_plain_mode;

void* LGF_PowerSaveMode(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	pRsp->ret_stat_code = TEST_OK_S;
	
	switch( pReq->sleep_mode){	  
		case SLEEP_MODE_ON:   
		  	LGF_SendKey(KEY_END);
	  		break;
		case AIR_PLAIN_MODE_ON:
	  		if_condition_is_on_air_plain_mode = 1;
	  		set_opertion_mode(FALSE);  // LPM mode
	  		break;

		default :
	  		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
	
	uint8_t if_condition_is_on_air_plain_mode;
	return pRsp;

}

char external_memory_copy_test(void)
{
	char return_value = 1;
	char *src, *dest;
	off_t fd_offset;
	int fd;
	
	if ( (fd = sys_open((const char __user *) "/sdcard/SDTest.txt", O_CREAT | O_RDWR, 0) ) < 0 )
	{
		printk(KERN_ERR "[ATCMD_EMT] Can not access SD card\n");
		goto file_fail;
	}

	src = kmalloc(10, GFP_KERNEL);
	sprintf(src,"TEST");
	if((sys_write(fd, (const char __user *) src, 5)) < 0)
	{
		printk(KERN_ERR "[ATCMD_EMT] Can not write SD card \n");
		goto file_fail;
	}
	fd_offset = sys_lseek(fd, 0, 0);
	
	dest = kmalloc(10, GFP_KERNEL);
	if((sys_read(fd, (char __user *) dest, 5)) < 0)
	{
		printk(KERN_ERR "[ATCMD_EMT]Can not read SD card \n");
		goto file_fail;
	}
	if ((memcmp(src, dest, 4)) == 0)
		return_value = 0;
	else 
		return_value = 1;

	file_fail:
	sys_close(fd);
	sys_unlink((const char __user *)"/sdcard/SDTest.txt");
	return return_value;
}


void* LGF_ExternalSocketMemory(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
	__u64 blocks;
	__u64 bsize;

    struct statfs_local  sf;
    uint32 total, remained, used;
    pRsp->ret_stat_code = TEST_OK_S;

    printk(KERN_ERR "khlee debug %d \n", pReq->esm);
    
    switch( pReq->esm){
      case EXTERNAL_SOCKET_MEMORY_CHECK:
        pRsp->test_mode_rsp.memory_check = external_memory_copy_test();
        break;
        
      case EXTERNAL_FLASH_MEMORY_SIZE:
        if (sys_statfs("/sdcard", (struct statfs *)&sf) != 0) 
        {
          printk(KERN_ERR "[Testmode]can not get sdcard infomation \n");
          pRsp->ret_stat_code = TEST_FAIL_S;
          break;
        }
         printk(KERN_ERR "blocks %d  \n", sf.f_blocks);
         printk(KERN_ERR "block size %d \n", sf.f_bsize);
		 blocks = (__u64)sf.f_blocks;
		 bsize = (__u64)sf.f_bsize;
//        pRsp->test_mode_rsp.socket_memory_size = (sf.f_blocks *  sf.f_bsize) >> 20; // needs Mb. 
        pRsp->test_mode_rsp.socket_memory_size =(blocks *  bsize) >> 20; // needs Mb. 
        printk(KERN_ERR "memory size %d \n", pRsp->test_mode_rsp.socket_memory_size);
        break;

#if defined (CONFIG_LGE_APPS_FACTORY_RESET)
      case EXTERNAL_SOCKET_ERASE:
        if (external_memory_copy_test())  // if there's no sd card then return fail
        {
          pRsp->ret_stat_code = TEST_FAIL_S;
        }
        else
        {
          if (diagpdev != NULL){
            update_diagcmd_state(diagpdev, "FACTORY_RESET", 3);
            msleep(5000);          
            pRsp->ret_stat_code = TEST_OK_S;
          }
          else 
          {
            printk("\n[%s] error FACTORY_RESET", __func__ );        
            pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
          }
        }
        break;
#endif /*CONFIG_LGE_APPS_FACTORY_RESET*/

	case EXTERNAL_MEMORY_USED_SIZE_CHECK_TEST:
	  if (sys_statfs("/sdcard", (struct statfs *)&sf) != 0) 
	  {
	    printk(KERN_ERR "%s can not get sdcard infomation \n", __func__);
	    pRsp->ret_stat_code = TEST_FAIL_S;
	    break;
	  }


	  // 2010.12.15 jihoon.lee - change MB to Byte - Manufacture requests
	  printk(KERN_INFO "%s f_blocks : %ld, f_bfree : %ld,   \n", __func__, (sf.f_blocks* sf.f_bsize), (sf.f_bfree* sf.f_bsize));
	  pRsp->test_mode_rsp.socket_memory_size = (unsigned long long)((sf.f_blocks - sf.f_bfree) * sf.f_bsize);
	  break;

      default:
        pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;  
        break;
      }

    return pRsp;
}

void* LGF_MemoryVolumeCheck(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
  struct statfs_local  sf;
  unsigned int total = 0;
  unsigned int used = 0;
  unsigned int remained = 0;
  pRsp->ret_stat_code = TEST_OK_S;

  if (sys_statfs("/data", (struct statfs *)&sf) != 0) 
  {
    printk(KERN_ERR "[Testmode]can not get sdcard infomation \n");
    pRsp->ret_stat_code = TEST_FAIL_S;
  }
  else
  {

    total = (sf.f_blocks * sf.f_bsize) >> 20; 
    remained = (sf.f_bavail * sf.f_bsize) >> 20;
    used = total - remained;

    switch(pReq->mem_capa)
    {
      case MEMORY_TOTAL_CAPA_TEST:
          pRsp->test_mode_rsp.mem_capa = total;
          break;         

      case MEMORY_USED_CAPA_TEST:
          pRsp->test_mode_rsp.mem_capa = used;
          break;         

      case MEMORY_REMAIN_CAPA_TEST:
          pRsp->test_mode_rsp.mem_capa = remained;
          break;         

      default :
          pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;        
          break;
    }
  }
  
  return pRsp;
}

void* LGF_TestModeManual(test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
  pRsp->ret_stat_code = TEST_OK_S;
  pRsp->test_mode_rsp.manual_test = TRUE;

  return pRsp;
}

#if defined (CONFIG_LGE_APPS_FACTORY_RESET)
static unsigned char test_mode_factory_reset_status = FACTORY_RESET_START;
char isFactoryFlagChanged = 0;
#define BUF_PAGE_SIZE 2048

#ifdef CONFIG_LGE_MTD_DIRECT_ACCESS
#define FACTORY_RESET_STR       "FACT_RESET_"
#define FACTORY_RESET_STR_SIZE	11
#define FACTORY_RESET_BLK 1 // read / write on the first block

#ifndef PERSIST_PART_NAME
#define PERSIST_PART_NAME "persist"
#endif

#define MSLEEP_CNT 100

static unsigned int mtd_part_num = 0;
static unsigned int mtd_part_size = 0;
static int mtd_factory_blk = 0;
//extern int lge_init_mtd_access(int partition, int block);
extern int lge_init_mtd_access(char *partition_name, int block);
extern int lge_erase_block(int ebnum);
extern int lge_write_block(int ebnum, unsigned char *buf, size_t size);
extern int lge_read_block(int ebnum, unsigned char *buf);
extern unsigned int lge_get_mtd_part_info(void);
extern int lge_get_mtd_factory_mode_blk(int target_blk);
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/

void* LGF_TestModeFactoryReset(
		test_mode_req_type* pReq ,
		DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
  unsigned char pbuf[BUF_PAGE_SIZE];
  int mtd_op_result = 0;
  unsigned char startStatus; 

  pRsp->ret_stat_code = TEST_OK_S;

#ifdef CONFIG_LGE_MTD_DIRECT_ACCESS
  if(test_mode_factory_reset_status == FACTORY_RESET_START)
  {
    mtd_part_num = lge_get_mtd_part_info();
    mtd_part_size = mtd_part_num & 0x0000FFFF;
    mtd_part_num = (mtd_part_num >> 16) & 0x0000FFFF;
//    lge_init_mtd_access(mtd_part_num, 0);
    lge_init_mtd_access(PERSIST_PART_NAME, 0);

    mtd_factory_blk = lge_get_mtd_factory_mode_blk(FACTORY_RESET_BLK);
    printk("mtd info num : %d, size : %d, factory_blk : %d\n", mtd_part_num, mtd_part_size, mtd_factory_blk);
  }
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/

  switch(pReq->factory_reset)
  {
    case FACTORY_RESET_CHECK :
#ifdef CONFIG_LGE_MTD_DIRECT_ACCESS
      memset((void*)pbuf, 0, sizeof(pbuf));
      mtd_op_result = lge_read_block(mtd_factory_blk, pbuf);
      if(mtd_op_result!=0)
      {
        printk(KERN_ERR "[Testmode]lge_read_block, error num = %d \n", mtd_op_result);
      }
      else
     {
       printk(KERN_INFO "\n[Testmode]factory reset memcmp\n");
       if(memcmp(pbuf, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE) == 0) // tag read sucess
      {
        startStatus = pbuf[FACTORY_RESET_STR_SIZE] - '0';
        printk(KERN_INFO "[Testmode]factory reset backup status = %d \n", startStatus);
      }
     }  

      test_mode_factory_reset_status = FACTORY_RESET_INITIAL;
      memset((void *)pbuf, 0, sizeof(pbuf));
      sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
      printk(KERN_INFO "[Testmode]factory reset status = %d\n", test_mode_factory_reset_status);

      mtd_op_result = lge_erase_block(mtd_factory_blk);
      if(mtd_op_result!=0)
      {
        printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
      }
      else
      {
        mtd_op_result = lge_write_block(mtd_factory_blk, pbuf, BUF_PAGE_SIZE);
        if(mtd_op_result!=0)
        {
          printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
        }
      }

      printk(KERN_INFO "[Testmode]send_to_arm9 start\n");
      send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
      printk(KERN_INFO "[Testmode]send_to_arm9 end\n");

	  /*LG_FW khlee 2010.03.04 -If we start at 5, we have to go to APP reset state(3) directly */
	  if( startStatus == FACTORY_RESET_COLD_BOOT_END)
	    test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_START;
	  else
	    test_mode_factory_reset_status = FACTORY_RESET_ARM9_END;

      memset((void *)pbuf, 0, sizeof(pbuf));
      sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
      printk(KERN_INFO "[Testmode]factory reset status = %d\n", test_mode_factory_reset_status);

      mtd_op_result = lge_erase_block(mtd_factory_blk);
      if(mtd_op_result!=0)
      {
       printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
      }
      else
      {
         mtd_op_result = lge_write_block(mtd_factory_blk, pbuf, BUF_PAGE_SIZE);
         if(mtd_op_result!=0)
         {
          printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
         }
      }

#else /**/
      send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/
      pRsp->ret_stat_code = TEST_OK_S;
      break;

    case FACTORY_RESET_COMPLETE_CHECK:
#ifdef CONFIG_LGE_MTD_DIRECT_ACCESS
      pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
      printk(KERN_ERR "[Testmode]not supported\n");
#else
      printk(KERN_INFO "[Testmode]send_to_arm9 start\n");
      send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
      printk(KERN_INFO "[Testmode]send_to_arm9 end\n");
      pRsp->ret_stat_code = TEST_OK_S;
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/
      break;

    case FACTORY_RESET_STATUS_CHECK:
#ifdef CONFIG_LGE_MTD_DIRECT_ACCESS
      memset((void*)pbuf, 0, sizeof(pbuf));
      mtd_op_result = lge_read_block(mtd_factory_blk, pbuf);
      if(mtd_op_result!=0)
      {
        printk(KERN_ERR "[Testmode]lge_read_block, error num = %d \n", mtd_op_result);
        //pRsp->factory_reset = FACTORY_RESET_NA;
      }
      else
     {
       if(memcmp(pbuf, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE) == 0) // tag read sucess
      {
        test_mode_factory_reset_status = pbuf[FACTORY_RESET_STR_SIZE] - '0';
        printk(KERN_INFO "[Testmode]factory reset status = %d \n", test_mode_factory_reset_status);
        pRsp->ret_stat_code = test_mode_factory_reset_status;
      }
       else
      {
        printk(KERN_ERR "[Testmode]factory reset tag fail\n");
        test_mode_factory_reset_status = FACTORY_RESET_START;
        pRsp->ret_stat_code = test_mode_factory_reset_status;
      }
         
     }  
#else
      pRsp->ret_stat_code = 7;//FACTORY_RESET_NA;
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/

      break;

    case FACTORY_RESET_COLD_BOOT:
#ifdef CONFIG_LGE_MTD_DIRECT_ACCESS
      test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_START;
      memset((void *)pbuf, 0, sizeof(pbuf));
      sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
      printk(KERN_INFO "[Testmode]factory reset status = %d\n", test_mode_factory_reset_status);
      mtd_op_result = lge_erase_block(mtd_factory_blk);
      if(mtd_op_result!=0)
      {
        printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
      }
      else
      {
        mtd_op_result = lge_write_block(mtd_factory_blk, pbuf, BUF_PAGE_SIZE);
        if(mtd_op_result!=0)
        {
          printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
        }
      }

#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/

#if 0 // block here in order not to format sdcard, factory dll will use external socket erase command
        if (diagpdev != NULL){
          update_diagcmd_state(diagpdev, "FACTORY_RESET", 3);
          msleep(5000);          
          pRsp->ret_stat_code = TEST_OK_S;
        }
        else 
        {
          printk("\n[%s] error FACTORY_RESET", __func__ );        
          pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
        }
#endif
      break;

    case FACTORY_RESET_ERASE_USERDATA:
#ifdef CONFIG_LGE_MTD_DIRECT_ACCESS
      test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_START;
      memset((void *)pbuf, 0, sizeof(pbuf));
      sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
      printk(KERN_INFO "[Testmode-erase userdata]factory reset status = %d\n", test_mode_factory_reset_status);
      mtd_op_result = lge_erase_block(mtd_factory_blk);
      if(mtd_op_result!=0)
      {
        printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
        pRsp->ret_stat_code = TEST_FAIL_S;
      }
      else
      {
        mtd_op_result = lge_write_block(mtd_factory_blk, pbuf, BUF_PAGE_SIZE);
        if(mtd_op_result!=0)
        {
          printk(KERN_ERR "[Testmode]lge_write_block, error num = %d \n", mtd_op_result);
          pRsp->ret_stat_code = TEST_FAIL_S;
        }
        else{
          pRsp->ret_stat_code = TEST_OK_S;
        }
      }
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/
    break;

     default:
        pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
        break;
    }
  return pRsp;
}


int factory_reset_check(void)
{
  unsigned char pbuf[BUF_PAGE_SIZE];
  int mtd_op_result = 0;

	mtd_part_num = lge_get_mtd_part_info();
	mtd_part_size = mtd_part_num & 0x0000FFFF;
	mtd_part_num = (mtd_part_num >> 16) & 0x0000FFFF;

  //lge_init_mtd_access(mtd_part_num, 0); 
  lge_init_mtd_access(PERSIST_PART_NAME, 0);
  mtd_factory_blk = lge_get_mtd_factory_mode_blk(FACTORY_RESET_BLK);
 
  // check_staus
  memset((void*)pbuf, 0, sizeof(pbuf));
  mtd_op_result = lge_read_block(mtd_factory_blk, pbuf);
  if(mtd_op_result!=0)
  {
    printk(KERN_ERR "factory_reset_check : lge_read_block, error num = %d \n", mtd_op_result);
  }
  else
  {
    if(memcmp(pbuf, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE) == 0) // tag read sucess
    {
      test_mode_factory_reset_status = pbuf[FACTORY_RESET_STR_SIZE] - '0';
      printk(KERN_INFO "factory_reset_check : status = %d \n", test_mode_factory_reset_status);
    }
    else
    {
      printk(KERN_ERR "factory_reset_check : tag fail\n");
      test_mode_factory_reset_status = FACTORY_RESET_START;
    }		
  }  

// if status is cold boot start then mark it end
  if(test_mode_factory_reset_status == FACTORY_RESET_COLD_BOOT_START ||
      test_mode_factory_reset_status == 4 || /* 4 : temp value between factory reset operation and reboot */
      test_mode_factory_reset_status == FACTORY_RESET_PRIVACY_MENU ||/* value to indicate to disable usb debug setting for ui factory reset*/
      isFactoryFlagChanged == 1)
  {
    memset((void *)pbuf, 0, sizeof(pbuf));
    test_mode_factory_reset_status = FACTORY_RESET_COLD_BOOT_END;

    diagpdev = diagcmd_get_dev();
    if (diagpdev != NULL){
      update_diagcmd_state(diagpdev, "ADBSET", 0);
    }

    sprintf(pbuf, "%s%d",FACTORY_RESET_STR, test_mode_factory_reset_status);
    printk(KERN_INFO "factory_reset_check : status = %d\n", test_mode_factory_reset_status);
    mtd_op_result = lge_erase_block(mtd_factory_blk);
    if(mtd_op_result!=0)
    {
      printk(KERN_ERR "factory_reset_check : lge_erase_block, error num = %d \n", mtd_op_result);
    }
    else
    {
      mtd_op_result = lge_write_block(mtd_factory_blk, pbuf, BUF_PAGE_SIZE);
      if(mtd_op_result!=0)
      {
        printk(KERN_ERR "factory_reset_check : lge_write_block, error num = %d \n", mtd_op_result);
      }
    }
  }

  return 0;
}
EXTERN_SYMBOL(factory_reset_check);

void* LGF_TestScriptItemSet(	test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type	*pRsp)
{
#ifdef CONFIG_LGE_MTD_DIRECT_ACCESS
  int mtd_op_result = 0;
#endif

  if(pReq->test_mode_test_scr_mode == TEST_SCRIPT_ITEM_SET)
  {   
#ifdef CONFIG_LGE_MTD_DIRECT_ACCESS
  if(test_mode_factory_reset_status == FACTORY_RESET_START)
  {
    mtd_part_num = lge_get_mtd_part_info();
    mtd_part_size = mtd_part_num & 0x0000FFFF;
    mtd_part_num = (mtd_part_num >> 16) & 0x0000FFFF;
    //lge_init_mtd_access(mtd_part_num, 0);
    lge_init_mtd_access(PERSIST_PART_NAME, 0);

    mtd_factory_blk = lge_get_mtd_factory_mode_blk(FACTORY_RESET_BLK);
    printk("mtd info num : %d, size : %d, factory_blk : %d\n", mtd_part_num, mtd_part_size, mtd_factory_blk);
  }
  	
  mtd_op_result = lge_erase_block(mtd_factory_blk);
  if(mtd_op_result!=0)
  {
    printk(KERN_ERR "[Testmode]lge_erase_block, error num = %d \n", mtd_op_result);
    pRsp->ret_stat_code = TEST_FAIL_S;	
  }
  else
#endif /*CONFIG_LGE_MTD_DIRECT_ACCESS*/
    // LG_FW khlee 2010.03.16 - They want to ACL on state in test script state.
  {
    update_diagcmd_state(diagpdev, "ALC", 1);

    send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
  }
    }  
    else
      send_to_arm9((void*)(((byte*)pReq) -sizeof(diagpkt_header_type) - sizeof(word)) , pRsp);
        
  return pRsp;
	
}
#endif /*CONFIG_LGE_APPS_FACTORY_RESET*/

// this will be set if bootCompleted message comes down from the framwork via pm and led device driver, dependant to models
static int first_booting_complete_status = -1;
void set_first_booting_complete_status(int status)
{
	first_booting_complete_status = status;
}

int get_first_booting_complete_status(void)
{
	return first_booting_complete_status;
}

// this will be set if "/dev/chg_logo" file is written from the chargerlogo, dependant to models
static int first_booting_chg_mode_status = -1;
void set_first_booting_chg_mode_status(int status)
{
	first_booting_chg_mode_status = status;
}

int get_first_booting_chg_mode_status(void)
{
	return first_booting_chg_mode_status;
}

/*
status 0 : first booting completed
status 1 : first booting has not been completed
if this command is not supported, return ret_stat_code as TEST_NOT_SUPPORTED_S.

chg_status 0 : in the charging mode
chg_status 1 : normal boot mode
*/
void* LGF_TestModeFirstBooingCheck(    test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type     *pRsp)
{
	printk(KERN_INFO"%s\n", __func__);

	switch(pReq->complete_status_check)
	{
		case FIRST_BOOTING_COMPLETED_CHECK:
			if(first_booting_complete_status == 1)
				pRsp->ret_stat_code = FIRST_BOOTING_COMPLETED;
			else
				pRsp->ret_stat_code = FIRST_BOOTING_NOT_COMPLETED;
			break;
		case FIRST_BOOTING_CHG_MODE_CHECK:
			if(first_booting_chg_mode_status == 1)
				pRsp->ret_stat_code = FIRST_BOOTING_IN_CHG_MODE;
			else
				pRsp->ret_stat_code = FIRST_BOOTING_NOT_IN_CHG_MODE;
			break;
		default:
			pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}
	return pRsp;
}

void* LGF_TestModeDBIntegrityCheck(    test_mode_req_type* pReq ,DIAG_TEST_MODE_F_rsp_type     *pRsp)
{

	printk(KERN_ERR "[_DBCHECK_] [%s:%d] DBCHECKSubCmd=<%d>\n", __func__, __LINE__, pReq->bt);

	if (diagpdev != NULL){
		update_diagcmd_state(diagpdev, "DBCHECK", pReq->db_check);
		pRsp->ret_stat_code = TEST_OK_S;
	}
	else
	{
		printk("\n[%s] error DBCHECK", __func__ );
		pRsp->ret_stat_code = TEST_NOT_SUPPORTED_S;
	}

	return pRsp;
}

/*  USAGE
  *    1. If you want to handle at ARM9 side, you have to insert fun_ptr as NULL and mark ARM9_PROCESSOR
  *    2. If you want to handle at ARM11 side , you have to insert fun_ptr as you want and mark AMR11_PROCESSOR.
  */
  
testmode_user_table_entry_type testmode_mstr_tbl[TESTMODE_MSTR_TBL_SIZE] =
{ 
/*    sub_command                                                                       fun_ptr                                       which procesor              */
    { TEST_MODE_VERSION                          ,  NULL                       , ARM9_PROCESSOR},
	{ TEST_MODE_LCD 							 ,  linux_app_handler		   , ARM11_PROCESSOR},
    { TEST_MODE_MRD_USB_TEST                     ,  NULL                       , ARM9_PROCESSOR },
    { TEST_MODE_MANUAL_MODE_TEST		         ,  NULL				       , ARM9_PROCESSOR },
#if defined (CONFIG_LGE_APPS_FACTORY_RESET)
    { TEST_MODE_TEST_SCRIPT_MODE                 ,  LGF_TestScriptItemSet      , ARM11_PROCESSOR },
	{ TEST_MODE_FACTORY_RESET_CHECK_TEST		 ,	LGF_TestModeFactoryReset   , ARM11_PROCESSOR },//  
#else
    { TEST_MODE_TEST_SCRIPT_MODE                 ,  NULL                       , ARM9_PROCESSOR },
    { TEST_MODE_FACTORY_RESET_CHECK_TEST		 ,  NULL                       , ARM9_PROCESSOR },//  
#endif /*CONFIG_LGE_APPS_FACTORY_RESET*/
    { TEST_MODE_EXT_SOCKET_TEST                  ,  LGF_ExternalSocketMemory   , ARM11_PROCESSOR},
    { TEST_MODE_BLUETOOTH_TEST                   ,  LGF_TestModeBlueTooth	   , ARM11_PROCESSOR},
    { TEST_MODE_KEY_DATA_TEST                    ,  LGF_TestModeKeyData	       , ARM11_PROCESSOR},
    { TEST_MODE_MEMORY_CAPA_TEST                 ,  LGF_MemoryVolumeCheck	   , ARM11_PROCESSOR},
    { TEST_MODE_SLEEP_MODE_TEST                  ,  LGF_PowerSaveMode	       , ARM11_PROCESSOR},
#ifdef CONFIG_LGE_AUDIO_TESTMODE
	{ TEST_MODE_MOTOR					 		 ,	LGF_TestMotor	   		   , ARM11_PROCESSOR},	
    { TEST_MODE_ACOUSTIC						 ,	LGF_TestAcoustic	   	   , ARM11_PROCESSOR},
	{ TEST_MODE_MP3_TEST					 	 ,	LGF_TestModeMP3	   		   , ARM11_PROCESSOR},
    { TEST_MODE_SPEAKER_PHONE_TEST				 ,	LGF_TestModeSpeakerPhone   , ARM11_PROCESSOR},
    { TEST_MODE_VOLUME_TEST						 ,	LGT_TestModeVolumeLevel	   , ARM11_PROCESSOR},
#endif
    { TEST_MODE_KEY_TEST                         ,  LGT_TestModeKeyTest        , ARM11_PROCESSOR},
    { TEST_MODE_WIFI_TEST 					     ,  linux_app_handler		   , ARM11_PROCESSOR},
#ifdef CONFIG_LGE_CAMERA_TESTMODE
	{ TEST_MODE_CAM 						     ,  LGF_TestCam 			   , ARM11_PROCESSOR},  
#endif
    { TEST_MODE_PHOTO_SENSER_TEST 			     ,  linux_app_handler		   , ARM11_PROCESSOR},
    { TEST_MODE_ACCEL_SENSOR_TEST 		         ,  linux_app_handler		   , ARM11_PROCESSOR},
    { TEST_MODE_PROXIMITY_SENSOR_TEST  		     ,  linux_app_handler		   , ARM11_PROCESSOR},
	{ TEST_MODE_MAC_READ_WRITE					 ,  linux_app_handler		   , ARM11_PROCESSOR},
    { TEST_MODE_FIRST_BOOTING_COMPLETE_CHECK	, LGF_TestModeFirstBooingCheck	, ARM11_PROCESSOR },
    /*90~	*/
    { TEST_MODE_DB_INTEGRITY_CHECK				, LGF_TestModeDBIntegrityCheck	, ARM11_PROCESSOR },

    { TEST_MODE_RESET_PRODUCTION, 		NULL,								ARM9_PROCESSOR},
//  not yet
//
};

#endif
