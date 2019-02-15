/* LGE_CHANGES LGE_FACTORY_AT_COMMANDS  */
/* Created by princlee@lge.com  
 * arch/arm/mach-msm/rpc_server_misc.c
 *
 * Copyright (C) 2008 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include <mach/msm_rpcrouter.h>
#include <linux/syscalls.h> //for sys operations
#include <linux/fcntl.h> //LGE_CHANGE [seypark@lge.com] for AT+MTC

#if defined (CONFIG_LGE_ATS_ETA_MTC) || defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
#include <linux/jiffies.h>
#include <mach/lg_diag_mtc.h>
#include <linux/kmod.h> // for umh_wait
#include <linux/fs.h> // for file struct
#include <linux/delay.h> // for mdelay()
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/

/* Misc server definitions. */
#define ETA_MODULE "/system/bin/eta"

#define MISC_APPS_APISPROG		0x30000006
#define MISC_APPS_APISVERS		0

#define ONCRPC_LGE_ATCMD_FACTORY_LARGE_PROC 6

struct rpc_misc_apps_bases_args {
	uint32_t at_cmd;
	uint32_t at_act;
	uint32_t at_param;
};

struct rpc_misc_apps_LARGE_bases_args {
	uint32_t at_cmd;
	uint32_t at_act;
	uint32_t sendNum;
	uint32_t endofBuffer;
	uint32_t buffersize;
	AT_SEND_BUFFER_t buffer[MAX_SEND_SIZE_BUFFER];
	
};

// at_cmd value start
// !!! must same with oncrpc_xdr_types.h (others see dsatfactory.h) in ARM9 (AMSS) side  !!!
//////////////////////////////////////////////////////////////////
#define ATCMD_MTC	  80 //LGE_CHANGE_S [seypark@lge.com] for AT+MTC
//////////////////////////////////////////////////////////////////
// at_cmd value end

// at_act value  start
// !!! must same with dsatfactory.h in ARM9 (AMSS) side  !!!
//action/query/range/assign
//////////////////////////////////////////////////////////////////
#define ATCMD_ACTION	0
#define ATCMD_QUERY		1
#define ATCMD_RANGE		2
#define ATCMD_ASSIGN	3
//////////////////////////////////////////////////////////////////
// at_act value  end

#if defined (CONFIG_LGE_ATS_ETA_MTC) || defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)

#define JIFFIES_TO_MS(t) ((t) * 1000 / HZ)

//ACTION filed information
typedef enum{
	ETA_TOUCH_MOVETO = 0, /*Move the pointer to the specified location*/
	ETA_TOUCH_MOVEBY = 1, /*Move the pointer by the specified values*/
	ETA_TOUCH_TAB = 2, /*Tab at the current location*/
	ETA_TOUCH_DOUBLETAB = 3, /*Double tab at the current location*/
	ETA_TOUCH_DOWN = 4, /*Touch down at the current location*/
	ETA_TOUCH_UP = 5, /*Touch up at the current location*/
	ETA_TOUCH_DEFAULT = 0xff,
}eta_touch_event_action_type;

static char eta_prev_action = ETA_TOUCH_DEFAULT;

struct ats_mtc_key_log_type ats_mtc_key_log;
void ats_mtc_send_key_log_to_eta(struct ats_mtc_key_log_type* p_ats_mtc_key_log, int wait_mode);

static int base64_decode(char *, unsigned char *, int);
int base64_encode(char *, int, char *);

unsigned long int ats_mtc_log_mask = 0x00000000;

extern unsigned char g_diag_mtc_check;
extern void mtc_send_key_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log);
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/

// dsatHandleAT_ARM11 return Value
//////////////////////////////////////////////////////////////////
#define HANDLE_OK  0
#define HANLDE_FAIL 1
#define HANDLE_ERROR 2
#define HANDLE_OK_MIDDLE 4

// change little Endian to Big endian (there is not change function in Kernel)
char cpu_to_be8_AT(char value)
{
#define BITS_NUM_PER_BYTE  8
	char c_value = 0;
#if 0

	int loop = 0;
	int TOT_SIZE = sizeof(char)*BITS_NUM_PER_BYTE;

	for (loop = 0; loop < TOT_SIZE/2; loop++)
	{
		c_value |= (value & (1 << loop)) << (TOT_SIZE - loop -1);

	}
	// there is no ODD lengh 
	for (loop = TOT_SIZE/2; loop < TOT_SIZE; loop++)
	{
		c_value |= (value & (1 << loop)) >> (TOT_SIZE/2 - loop +1 );

	}
#else
	c_value = value;
#endif
	return c_value;
}

//LGE_CHANTE_S [seypark@lge.com] 2009-04-16 - for AT+MTC

/* LGE_CHANGE_S [jihoon.lee@lge.com] 2010-02-03 */
#if defined (CONFIG_LGE_ATS_ETA_MTC) || defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
int eta_execute(char *string, int wait_mode)
#else
static int eta_execute(char *string)
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
/* LGE_CHANGE_E [jihoon.lee@lge.com] 2010-02-03 */
{
	int ret;
	char cmdstr[100];
	int fd;
	struct file * file;
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

// BEGIN: eternalblue@lge.com.2009-10-23
// 0001794: [ARM9] ATS AT CMD added 
	if ( (fd = sys_open((const char __user *)ETA_MODULE, O_RDONLY ,0) ) < 0 )
	{
		printk(KERN_ERR "\n%s, can not open %s\n", __func__, ETA_MODULE);
		sprintf(cmdstr, "%s %s", ETA_MODULE, string);
	}
	else
	{
		printk(KERN_INFO "\n %s, execute %s\n", __func__, ETA_MODULE);
		sprintf(cmdstr, "%s %s", ETA_MODULE, string);
		sys_close(fd);
	}
// END: eternalblue@lge.com.2009-10-23

	printk(KERN_INFO "%s, mode : %d, data - %s\n\n", __func__, wait_mode, cmdstr);





#if 0
if(wait_mode != UMH_NO_WAIT)
{ 
//	static inline int call_usermodehelper(char *path, char **argv, char **envp, enum umh_wait wait)
	if ((ret =
	     call_usermodehelper("/system/bin/sh", argv, envp, wait_mode)) != 0) {
		printk(KERN_ERR "[ETA]Eta failed to run \": %i\n",
		       ret);
	}
	else
		printk(KERN_INFO "%s, execute ok, ret = %d\n", __func__, ret);

}
else
{
//	int call_usermodehelper_pipe(char *path, char **argv, char **envp, struct file **filp)
	if ((ret =
		call_usermodehelper_pipe("/system/bin/sh", argv, envp, &file))  != 0){
		printk(KERN_ERR "%s, call_usermodehelper_pipe to %s pipe failed\n", __func__, "/system/bin/sh");
	}
	else
		printk(KERN_INFO "%s, execute ok, ret = %d\n", __func__, ret);
}
#else
	if ((ret =
	     call_usermodehelper("/system/bin/sh", argv, envp, wait_mode)) != 0) {
		printk(KERN_ERR "[ETA]Eta failed to run \": %i\n",
		       ret);
	}
	else
		printk(KERN_INFO "%s, execute ok, ret = %d\n", __func__, ret);
#endif


	return ret;
}

EXPORT_SYMBOL(eta_execute);

//LGE_CHANGE_E [seypark@lge.com]
static int  dsatHandleAT_ARM11_LARGE(struct rpc_misc_apps_LARGE_bases_args *args,struct msm_rpc_server *server)
{
	int result = HANDLE_OK;
	int loop = 0;

	char ret_string[MAX_STRING_RET];
	uint32_t ret_value1 =0;
	uint32_t ret_value2 = 0;
	static AT_SEND_BUFFER_t totalBuffer[LIMIT_MAX_SEND_SIZE_BUFFER];
	static uint32_t totalBufferSize = 0;
	uint32_t at_cmd,at_act;

/* LGE_CHANGE_S [jihoon.lee@lge.com] 2010-02-03 */
#if defined (CONFIG_LGE_ATS_ETA_MTC) || defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
	int len_b64;
	//char *temp;
	//char *encoded_params;
	char *decoded_params;
	unsigned char b0;
	unsigned char b1;
	unsigned char b2;
	unsigned char b3;
	unsigned long logmask = 0x00;
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
/* LGE_CHANGE_E [jihoon.lee@lge.com] 2010-02-03 */
	memset (ret_string, 0, sizeof(ret_string));

// init for LARGE Buffer
	if(args->sendNum == 0)
	{
		// init when first send
		memset(totalBuffer, 0, sizeof(totalBuffer));
		totalBufferSize = 0;
	}
	
	args->at_cmd = be32_to_cpu(args->at_cmd);
	args->at_act = be32_to_cpu(args->at_act);
	args->sendNum = be32_to_cpu(args->sendNum);
	args->endofBuffer = be32_to_cpu(args->endofBuffer);
	args->buffersize = be32_to_cpu(args->buffersize);
		
	printk(KERN_INFO "[ETA]handle_misc_rpc_call at_cmd = 0x%X, at_act=%d, sendNum=%d:\n",
	      args->at_cmd, args->at_act,args->sendNum);
	printk(KERN_INFO "[ETA]handle_misc_rpc_call endofBuffer = %d, buffersize=%d:\n",
	      args->endofBuffer, args->buffersize);
	printk(KERN_INFO "[ETA]input buff[0] = 0x%X,buff[1]=0x%X,buff[2]=0x%X:\n",args->buffer[0],args->buffer[1],args->buffer[2]);
	// printk("len = %d\n", len);
	if(args->sendNum < MAX_SEND_LOOP_NUM)
	{
		for(loop = 0; loop < args->buffersize; loop++)
		{
			// totalBuffer[MAX_SEND_SIZE_BUFFER*args->sendNum + loop] =  be32_to_cpu(args->buffer[loop]);
			totalBuffer[MAX_SEND_SIZE_BUFFER*args->sendNum + loop] =  (args->buffer[loop]);
		}
		
		// memcpy(totalBuffer + MAX_SEND_SIZE_BUFFER*args->sendNum, args->buffer, args->buffersize);
		totalBufferSize += args->buffersize;
			
	}
	printk(KERN_INFO "[ETA]handle_misc_rpc_call buff[0] = 0x%X, buff[1]=0x%X, buff[2]=0x%X\n",
	      totalBuffer[0 + args->sendNum*MAX_SEND_SIZE_BUFFER], totalBuffer[1 + args->sendNum*MAX_SEND_SIZE_BUFFER], totalBuffer[2+args->sendNum*MAX_SEND_SIZE_BUFFER]);

	if(!args->endofBuffer )
		return HANDLE_OK_MIDDLE;

	at_cmd = args->at_cmd;
	at_act = args->at_act;

///////////////////////////////////////////////////
/* please use
static uint8_t totalBuffer[LIMIT_MAX_SEND_SIZE_BUFFER];
static uint32_t totalBufferSize = 0;
uint32_t at_cmd,at_act;
*/
///////////////////////////////////////////////////
	switch (at_cmd)
	{
//LGE_CHANTE_S [seypark@lge.com] 2009-04-16 - for AT+MTC
		case ATCMD_MTC:
		{
			int exec_result =0;

			printk(KERN_INFO "\n[ETA]ATCMD_MTC\n ");

#if defined (CONFIG_LGE_ATS_ETA_MTC)
			g_diag_mtc_check = 0;
#endif
			if(at_act != ATCMD_ACTION)
				result = HANLDE_FAIL;

			printk(KERN_INFO "[ETA]totalBuffer : [%s] size: %d\n", totalBuffer, totalBufferSize);
			exec_result = eta_execute(totalBuffer, UMH_WAIT_PROC);
			printk(KERN_INFO "[ETA]AT+MTC exec_result %d\n",exec_result);
			
/* LGE_CHANGE_S [jihoon.lee@lge.com] 2010-02-03 */
#if defined (CONFIG_LGE_ATS_ETA_MTC) || defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
/*
			if((temp = (char *)strchr((const char *)totalBuffer, (int)'=') + 1) == NULL) {
				printk(KERN_INFO "[ETA]Invalid Parameter\n");
				result = HANLDE_FAIL;
			}
			encoded_params = temp;
*/		
			decoded_params = kmalloc(sizeof(char)*totalBufferSize, GFP_KERNEL);
			printk(KERN_INFO "[ETA] encoded_addr: 0x%X, decoded_addr: 0x%X, size: %d\n",  (unsigned int)totalBuffer, (unsigned int)decoded_params, sizeof(char)*totalBufferSize);
			
			len_b64 = base64_decode((char *)totalBuffer, (unsigned char *)decoded_params, totalBufferSize);
			printk(KERN_INFO "[ETA] sub cmd: 0x%X, param1: 0x%X, param2: 0x%X (length = %d)\n",  
			decoded_params[1], decoded_params[2], decoded_params[3], strlen((const char *)decoded_params));

			switch(decoded_params[1]) 
			{
				case 0x07://MTC_LOGGING_MASK_REQ_CMD:
					printk(KERN_INFO "[ETA] logging mask request cmd : %d\n", decoded_params[1]);

					b0 = decoded_params[2];
					b1 = decoded_params[3];
					b2 = decoded_params[4];
					b3 = decoded_params[5];

					logmask = b3<<24 | b2<<16 | b1<<8 | b0;

					switch(logmask)
					{
						case 0x00000000://ETA_LOGMASK_DISABLE_ALL:
						case 0xFFFFFFFF://ETA_LOGMASK_ENABLE_ALL:
						case 0x00000001://ETA_LOGITEM_KEY:
						case 0x00000002://ETA_LOGITEM_TOUCHPAD:
						case 0x00000003://ETA_LOGITME_KEYTOUCH:
							ats_mtc_log_mask = logmask;
							break;
						default:
							ats_mtc_log_mask = 0x00000000;//ETA_LOGMASK_DISABLE_ALL;
							break;
					}
					break;
					
				default:
					break;
			}
			
			kfree(decoded_params);
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
/* LGE_CHANGE_E [jihoon.lee@lge.com] 2010-02-03 */

			sprintf(ret_string, "edcb");
			ret_value1 = 10;
			ret_value2 = 20;

		}
		break;
//LGE_CHANGE_E

		default :
			result = HANDLE_ERROR;
			break;
	}

// give to RPC server result
/////////////////////////////////////////////////////////////////
	for(loop = 0; loop < MAX_STRING_RET ; loop++)
	{
		server->retvalue.ret_string[loop]= (AT_STR_t)(ret_string[loop]);
	}
	server->retvalue.ret_string[MAX_STRING_RET-1] = 0;
	server->retvalue.ret_value1 = ret_value1;
	server->retvalue.ret_value2 = ret_value2;
	if(args->endofBuffer )
	{
		// init when first send
		memset(totalBuffer, 0, sizeof(totalBuffer));
		totalBufferSize = 0;
	}
/////////////////////////////////////////////////////////////////
	return result;
}

/* LGE_CHANGE_S [jihoon.lee@lge.com] 2010-02-03 */
#if defined (CONFIG_LGE_ATS_ETA_MTC) || defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
/*------ Base64 Encoding Table ------*/
const char MimeBase64[] = {
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
	'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
	'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
	'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
	'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
	'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
	'w', 'x', 'y', 'z', '0', '1', '2', '3',
	 '4', '5', '6', '7', '8', '9', '+', '/'
};

/*------ Base64 Decoding Table ------*/
static int DecodeMimeBase64[256] = {
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* 00-0F */
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* 10-1F */
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63, /* 20-2F */ 
	52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-1,-1,-1, /* 30-3F */ 
	-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14, /* 40-4F */ 
	15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1, /* 50-5F */ 
	-1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40, /* 60-6F */ 
	41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1, /* 70-7F */ 
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* 80-8F */ 
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* 90-9F */ 
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* A0-AF */ 
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* B0-BF */ 
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* C0-CF */ 
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* D0-DF */ 
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* E0-EF */ 
	-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1  /* F0-FF */ 
};

static int base64_decode(char *text, unsigned char *dst, int numBytes)
{
	const char* cp; 
	int space_idx = 0, phase; 
	int d, prev_d = 0; 
	unsigned char c;

	printk(KERN_INFO "[ETA] text: 0x%X, dst: 0x%X, size: %d\n",  (unsigned int)text, (unsigned int)dst, numBytes);
		
	space_idx = 0; 
	phase = 0;
	
	for ( cp = text; *cp != '\0'; ++cp ) { 
		d = DecodeMimeBase64[(int) *cp]; 
	    if ( d != -1 ) { 
	    	switch ( phase ) { 
	        	case 0: 
		        	++phase; 
	        		break; 
	        	case 1: 
	          		c = ( ( prev_d << 2 ) | ( ( d & 0x30 ) >> 4 ) );
				printk(KERN_INFO "[ETA] space_idx: 0x%X, char: 0x%X\n",  space_idx, c);
	          		if ( space_idx < numBytes )
		            dst[space_idx++] = c; 
			        ++phase; 
	    		    break; 
		        case 2: 
		        	c = ( ( ( prev_d & 0xf ) << 4 ) | ( ( d & 0x3c ) >> 2 ) );
				printk(KERN_INFO "[ETA] space_idx: 0x%X, char: 0x%X\n",  space_idx, c);
			        if ( space_idx < numBytes ) 
	         		dst[space_idx++] = c; 
	          		++phase; 
	          		break; 
	        	case 3: 
	          		c = ( ( ( prev_d & 0x03 ) << 6 ) | d ); 
				printk(KERN_INFO "[ETA] space_idx: 0x%X, char: 0x%X\n",  space_idx, c);
	          		if ( space_idx < numBytes ) 
	            	dst[space_idx++] = c; 
	          		phase = 0; 
	          		break; 
			} 	
	      	prev_d = d; 
		}
	}
	printk(KERN_INFO "[ETA] Complete..\n");
	return space_idx;
}

int base64_encode(char *text, int numBytes, char *encodedText)
{
	unsigned char input[3] = {0,0,0}; 
	unsigned char output[4] = {0,0,0,0}; 
	int  index, i, j; 
	char *p, *plen; 

	plen = text + numBytes - 1; 

	j = 0;

	for (i = 0, p = text;p <= plen; i++, p++) { 
	    index = i % 3; 
	    input[index] = *p;

		if (index == 2 || p == plen) { 
	    	output[0] = ((input[0] & 0xFC) >> 2); 
			output[1] = ((input[0] & 0x3) << 4) | ((input[1] & 0xF0) >> 4); 
			output[2] = ((input[1] & 0xF) << 2) | ((input[2] & 0xC0) >> 6); 
			output[3] = (input[2] & 0x3F);

			encodedText[j++] = MimeBase64[output[0]]; 
			encodedText[j++] = MimeBase64[output[1]]; 
			encodedText[j++] = index == 0? '=' : MimeBase64[output[2]]; 
			encodedText[j++] = index < 2? '=' : MimeBase64[output[3]];

			input[0] = input[1] = input[2] = 0; 
		} 
	}
	encodedText[j] = '\0';

	return strlen(encodedText); 	
}

void ats_eta_key_logging(int scancode, unsigned char keystate,  int wait_mode)
{
/* LG_FW khlee - when we press key, to go to online state*/
/* This is not ready yet
	if(if_condition_is_on_air_plain_mode == 1 && keystate == PP2106_IN_KEYPRESS)
	{
		if_condition_is_on_air_plain_mode = 0;
		set_opertion_mode(TRUE);
		printk(KERN_ERR"send online event");
	}
	

	if(if_condition_is_on_key_buffering == 1 && keystate == PP2106_IN_KEYPRESS)
			lgf_factor_key_test_rsp((u8)scancode);
*/

/*	23 bytes for key log -> will be encoded to 32 byte (base64)
	CMD_CODE		1 (0xF0)
	SUB_CMD 	1 (0x08)
	LOG_ID			1 (1 key, 2 touch)
	LOG_LEN 		2 (data length in bytes)
	LOG_DATA		LOG_LEN = 18
		- TIME		8 (timestamp in milliseconds)
		- HOLD		1 (Press or release)
		- KEYCODE	1
		- ACTIVE_UIID 8 (Activated UI ID)
*/

	if((ats_mtc_log_mask&0x00000001) != 0) /* ETA_LOGITEM_KEY */
	{
		ats_mtc_key_log.log_id = 1; //LOG_ID, 1 key, 2 touch
		ats_mtc_key_log.log_len = 18; //LOG_LEN

		ats_mtc_key_log.x_hold = (unsigned int)keystate; // hold
		ats_mtc_key_log.y_code = (unsigned int)scancode; // key code

		printk(KERN_INFO "%s, key code 0x%X, hold : %d \n", __func__, scancode, keystate);
		
		if(g_diag_mtc_check == 0)
			ats_mtc_send_key_log_to_eta(&ats_mtc_key_log, wait_mode);
		else
			mtc_send_key_log_data(&ats_mtc_key_log);
	}

}

EXPORT_SYMBOL(ats_eta_key_logging);


/*	23 bytes for key log -> will be encoded to 32 byte (base64)
	CMD_CODE		1 (0xF0)
	SUB_CMD		1 (0x08)
	LOG_ID			1 (1 key, 2 touch)
	LOG_LEN			2 (data length in bytes)
	LOG_DATA		LOG_LEN = 18
		- TIME		8 (timestamp in milliseconds)
		- HOLD		1 (Press or release)
		- KEYCODE 	1
		- ACTIVE_UIID 8 (Activated UI ID)
*/
/*	27 bytes for touch log -> will be encoded to ?? byte (base64)
	CMD_CODE		1 (0xF0)
	SUB_CMD 		1 (0x08)
	LOG_ID			1 (1 key, 2 touch)
	LOG_LEN 		2 (data length in bytes)
	LOG_DATA		LOG_LEN = 22
		- TIME		8 (timestamp in milliseconds)
		- SCREEN_ID 1
		- ACTION	1 (Touch-action Type)
		- X 			2 (Absolute X Coordinate)
		- Y 			2 (Absolute Y Coordinate)
		- ACTIVE_UIID 8 (Activated UI ID)
*/
void ats_mtc_send_key_log_to_eta(struct ats_mtc_key_log_type* p_ats_mtc_key_log, int wait_mode)
{
	unsigned char *eta_cmd_buf = NULL;
	unsigned char *eta_cmd_buf_encoded = NULL;
	int index =0;
	int lenb64 = 0;
	int exec_result = 0;
	unsigned long long eta_time_val = 0;

	eta_cmd_buf = kmalloc(sizeof(unsigned char)*50, GFP_KERNEL);
	eta_cmd_buf_encoded = kmalloc(sizeof(unsigned char)*50, GFP_KERNEL);
	memset(eta_cmd_buf,0x00, 50);
	memset(eta_cmd_buf_encoded,0x00, 50);
				
	index = 0;
	eta_cmd_buf[index++] = (unsigned char)0xF0; //MTC_CMD_CODE
	eta_cmd_buf[index++] = (unsigned char)0x08; //MTC_LOG_REQ_CMD

	eta_cmd_buf[index++] = (unsigned char)p_ats_mtc_key_log->log_id; //LOG_ID, 1 key, 2 touch
	eta_cmd_buf[index++] = (unsigned char)p_ats_mtc_key_log->log_len; //LOG_LEN
	eta_cmd_buf[index++] = (unsigned char)0; //LOG_LEN

	eta_time_val = (unsigned long long)JIFFIES_TO_MS(jiffies);
	eta_cmd_buf[index++] = (unsigned char)(eta_time_val & 0xff); //LSB
	eta_cmd_buf[index++] = (unsigned char)( (eta_time_val >> 8) & 0xff );
	eta_cmd_buf[index++] = (unsigned char)( (eta_time_val >> 16) & 0xff );
	eta_cmd_buf[index++] = (unsigned char)( (eta_time_val >> 24) & 0xff );
	eta_cmd_buf[index++] = (unsigned char)( (eta_time_val >> 32) & 0xff );
	eta_cmd_buf[index++] = (unsigned char)( (eta_time_val >> 40) & 0xff );
	eta_cmd_buf[index++] = (unsigned char)( (eta_time_val >> 48) & 0xff );
	eta_cmd_buf[index++] = (unsigned char)( (eta_time_val >> 56) & 0xff ); // MSB

	index = 13;
	if(p_ats_mtc_key_log->log_id == ATS_MTC_KEY_LOG_ID_KEY)
	{
		eta_cmd_buf[index++] = (unsigned char)((p_ats_mtc_key_log->x_hold)&0xFF);// hold
		eta_cmd_buf[index++] = (unsigned char)((p_ats_mtc_key_log->y_code)&0xFF);//key code

		for(index = 15; index<23; index++) // ACTIVE_UIID 8
		{
			eta_cmd_buf[index] = 0;
		}
	}
	else if(p_ats_mtc_key_log->log_id == ATS_MTC_KEY_LOG_ID_TOUCH)
	{
		eta_cmd_buf[index++] = (unsigned char)1; // MAIN LCD
		eta_cmd_buf[index++] = (unsigned char)p_ats_mtc_key_log->action;
		eta_cmd_buf[index++] = (unsigned char)((p_ats_mtc_key_log->x_hold)&0xFF);// index = 15
		eta_cmd_buf[index++] = (unsigned char)(((p_ats_mtc_key_log->x_hold)>>8)&0xFF);// index = 16
		eta_cmd_buf[index++] = (unsigned char)((p_ats_mtc_key_log->y_code)&0xFF);// index = 17
		eta_cmd_buf[index++] = (unsigned char)(((p_ats_mtc_key_log->y_code)>>8)&0xFF);// index = 18

		for(index = 19; index<27; index++) // ACTIVE_UIID 8
		{
			eta_cmd_buf[index] = 0;
		}
	}

	lenb64 = base64_encode((char *)eta_cmd_buf, index, (char *)eta_cmd_buf_encoded);
			
	exec_result = eta_execute(eta_cmd_buf_encoded, wait_mode);
	printk(KERN_INFO "[ETA]AT+MTC exec_result %d\n",exec_result);

	kfree(eta_cmd_buf);
	kfree(eta_cmd_buf_encoded);

}

EXPORT_SYMBOL(ats_mtc_send_key_log_to_eta);

/*	27 bytes for key log -> will be encoded to ?? byte (base64)
	CMD_CODE		1 (0xF0)
	SUB_CMD 		1 (0x08)
	LOG_ID			1 (1 key, 2 touch)
	LOG_LEN 		2 (data length in bytes)
	LOG_DATA		LOG_LEN = 22
	- TIME		8 (timestamp in milliseconds)
	- SCREEN_ID 1
	- ACTION	1 (Touch-action Type)
	- X 			2 (Absolute X Coordinate)
	- Y 			2 (Absolute Y Coordinate)
	- ACTIVE_UIID 8 (Activated UI ID)
*/
void ats_eta_mtc_key_logging (int pendown, int x1, int y1, int x2, int y2)
{
	ats_mtc_key_log.log_id = 2; //LOG_ID, 1 key, 2 touch
	ats_mtc_key_log.log_len = 22; //LOG_LEN

	if (!pendown) // release
	{
		ats_mtc_key_log.action = (unsigned char)ETA_TOUCH_UP;
		eta_prev_action = ETA_TOUCH_UP;
	}
	else // down
	{
		if(eta_prev_action == ETA_TOUCH_DOWN)
		{
			ats_mtc_key_log.action = (unsigned char)ETA_TOUCH_MOVETO;
			if(g_diag_mtc_check == 1)
				mdelay(50); // do not need to response all move to events
		}
		else
			ats_mtc_key_log.action = (unsigned char)ETA_TOUCH_DOWN;
		eta_prev_action = ETA_TOUCH_DOWN;
	}

	if((x2 != -1) && (y2 !=-1)) // multi touch
	{
		ats_mtc_key_log.x_hold = x2;
		ats_mtc_key_log.y_code = y2;
	}
	else
	{
		ats_mtc_key_log.x_hold = x1;
		ats_mtc_key_log.y_code = y1;
	}

	printk(KERN_INFO "%s TOUCH X : %d, Y : %d \n", __func__, ats_mtc_key_log.x_hold, ats_mtc_key_log.y_code);

	if(g_diag_mtc_check == 0)
	{
		printk(KERN_INFO "%s, mcs6000_ts\n", __func__);
		ats_mtc_send_key_log_to_eta(&ats_mtc_key_log, UMH_WAIT_PROC);
	}
	else
	{
		printk(KERN_INFO "[MTC] mcs6000_ts\n");
		mtc_send_key_log_data(&ats_mtc_key_log);
	}
}

EXPORT_SYMBOL(ats_eta_mtc_key_logging);

#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
/* LGE_CHANGE_E [jihoon.lee@lge.com] 2010-02-03 */

static int handle_misc_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	int result = RPC_ACCEPTSTAT_SUCCESS;
	int dsat_result = HANDLE_OK;
	struct rpc_misc_apps_LARGE_bases_args *args;
	
	switch (req->procedure) 
	{
		case ONCRPC_LGE_ATCMD_FACTORY_LARGE_PROC:
		{
			printk("[ETA]ONCRPC_LGE_ATCMD_FACTORY_LARGE_PROC\n");

			args = (struct rpc_misc_apps_LARGE_bases_args *)(req + 1);

// LGE_CHANGE_S  princlee
		 	memset(server->retvalue.ret_string, 0, sizeof(server->retvalue.ret_string));

			dsat_result = dsatHandleAT_ARM11_LARGE(args,server);
			printk("[ETA]dsatHandleAT_ARM11_LARGE() dsat_resulte = %d\n", dsat_result);
			if(dsat_result == HANDLE_OK)
			{
			
				result = RPC_RETURN_RESULT_OK;
			}
			else if(dsat_result == HANDLE_OK_MIDDLE)
			{
				result = RPC_RETURN_RESULT_MIDDLE_OK;
			}
			else
				result= RPC_RETURN_RESULT_ERROR;
		
			return result;
		}
//LGE_UPDATE_E
	default:
		return -ENODEV;
	}
}

static struct msm_rpc_server rpc_server = {

	.prog = MISC_APPS_APISPROG,
	.vers = MISC_APPS_APISVERS,
	.rpc_call = handle_misc_rpc_call,
};

static int __init rpc_misc_server_init(void)
{
	return msm_rpc_create_server(&rpc_server);
}


module_init(rpc_misc_server_init);
/* LGE_CHANGE_E [jihoon.lee@lge.com] 2010-02-03 */

