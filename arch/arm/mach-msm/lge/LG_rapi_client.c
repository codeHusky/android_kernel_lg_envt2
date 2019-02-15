/* LGE_CHANGES LGE_RAPI_COMMANDS  */
/* Created by khlee@lge.com  
 * arch/arm/mach-msm/lge/LG_rapi_client.c
 *
 * Copyright (C) 2009 LGE, Inc.
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
#include <linux/kernel.h>
#include <linux/err.h>
#include <mach/oem_rapi_client.h>
#include <mach/lg_diag_testmode.h>
/* LGE_CHANGES_S [woonghee.park@lge.com] 2010-02-09, [VS740]*/
#ifdef CONFIG_LGE_BATT_ID_CHECK
#include <mach/msm_battery.h>
#endif
/* LGE_CHANGES_E [woonghee.park@lge.com]*/

#define GET_INT32(buf)       (int32_t)be32_to_cpu(*((uint32_t*)(buf)))
#define PUT_INT32(buf, v)        (*((uint32_t*)buf) = (int32_t)be32_to_cpu((uint32_t)(v)))
#define GET_U_INT32(buf)         ((uint32_t)GET_INT32(buf))
#define PUT_U_INT32(buf, v)      PUT_INT32(buf, (int32_t)(v))

#define GET_LONG(buf)            ((long)GET_INT32(buf))
#define PUT_LONG(buf, v) \
	(*((u_long*)buf) = (long)be32_to_cpu((u_long)(v)))

#define GET_U_LONG(buf)	      ((u_long)GET_LONG(buf))
#define PUT_U_LONG(buf, v)	      PUT_LONG(buf, (long)(v))


#define GET_BOOL(buf)            ((bool_t)GET_LONG(buf))
#define GET_ENUM(buf, t)         ((t)GET_LONG(buf))
#define GET_SHORT(buf)           ((short)GET_LONG(buf))
#define GET_U_SHORT(buf)         ((u_short)GET_LONG(buf))

#define PUT_ENUM(buf, v)         PUT_LONG(buf, (long)(v))
#define PUT_SHORT(buf, v)        PUT_LONG(buf, (long)(v))
#define PUT_U_SHORT(buf, v)      PUT_LONG(buf, (long)(v))

#define LG_RAPI_CLIENT_MAX_OUT_BUFF_SIZE 128
#define LG_RAPI_CLIENT_MAX_IN_BUFF_SIZE 128


static uint32_t open_count;
struct msm_rpc_client *client;

static int old_cable_type;
int  LG_rapi_init(void)
{
	client = oem_rapi_client_init();
	if (IS_ERR(client)) {
		pr_err("%s: couldn't open oem rapi client\n", __func__);
		return PTR_ERR(client);
	}
	open_count++;
	
  return 0;
}

void Open_check(void)
{
 	//to double check re-open;
  if(open_count > 0) return;
  // LG_FW
  LG_rapi_init();
}

int msm_chg_LG_cable_type(void)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;
  char output[LG_RAPI_CLIENT_MAX_OUT_BUFF_SIZE];
#if defined(CONFIG_LGE_USE_OEM_RAPI)
#else
  uint32_t out_len;
  int retValue= 0;
#endif

  Open_check();

/* LGE_CHANGES_S [younsuk.song@lge.com] 2010-09-06, Add error control code. Repeat 3 times if error occurs*/
	
	int rc= -1;
	int errCount= 0;

	do 
	{
  arg.event = LG_FW_RAPI_CLIENT_EVENT_GET_LINE_TYPE;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
  arg.in_len = 0;
  arg.input = NULL;
  arg.out_len_valid = 1;
  arg.output_valid = 1;
  arg.output_size = 4;

#if defined(CONFIG_LGE_USE_OEM_RAPI)
	ret.output = NULL;
	ret.out_len = NULL;

		rc= oem_rapi_client_streaming_function(client, &arg, &ret);
	
		if (rc < 0)
			pr_err("get LG_cable_type error \r\n");
		else
			pr_info("msm_chg_LG_cable_type: %d \r\n", GET_INT32(ret.output));

	} while (rc < 0 && errCount++ < 3);

/* LGE_CHANGES_E [younsuk.song@lge.com] */
	
	memcpy(output,ret.output,*ret.out_len);

#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
	/* LGE_CHANGE [dojip.kim@lge.com] 2010-06-21, free the allocated mem */
	if (ret.output)
		kfree(ret.output);
	if (ret.out_len)
		kfree(ret.out_len);
#else
	kfree(ret.output);
	kfree(ret.out_len);
#endif

	return (GET_INT32(output));  
#else
  ret.output = output;
  ret.out_len = &out_len;

  if(oem_rapi_client_streaming_function(client, &arg, &ret) < 0) // error case
    retValue = old_cable_type;
  else
  {
    retValue = GET_INT32(output);

    if( retValue == 0)   // no init cable 
      retValue = old_cable_type;
    else            //read ok.
      old_cable_type = retValue;
  }
  return retValue;  
#endif
}


void send_to_arm9( void*	pReq, void* pRsp)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;
#if defined(CONFIG_LGE_USE_OEM_RAPI)
#else
  uint32_t out_len;
#endif

  Open_check();
  
  arg.event = LG_FW_TESTMODE_EVENT_FROM_ARM11;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
  arg.in_len = sizeof(DIAG_TEST_MODE_F_req_type);
  arg.input = (char*)pReq;
  arg.out_len_valid = 1;
  arg.output_valid = 1;
#if defined (CONFIG_LGE_APPS_FACTORY_RESET)
  switch(((DIAG_TEST_MODE_F_req_type*)pReq)->sub_cmd_code)
  {
    case TEST_MODE_FACTORY_RESET_CHECK_TEST:
      arg.output_size = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type);
      break;
    case TEST_MODE_TEST_SCRIPT_MODE:
      arg.output_size = sizeof(DIAG_TEST_MODE_F_rsp_type) - sizeof(test_mode_rsp_type) + sizeof(test_mode_req_test_script_mode_type);
      break;
    default :
      arg.output_size = sizeof(DIAG_TEST_MODE_F_rsp_type);
      break;
  }
#else
  arg.output_size = sizeof(DIAG_TEST_MODE_F_rsp_type);
#endif /*CONFIG_LGE_APPS_FACTORY_RESET*/

#if defined(CONFIG_LGE_USE_OEM_RAPI)
	ret.output = NULL;
	ret.out_len = NULL;

	oem_rapi_client_streaming_function(client, &arg, &ret);
	memcpy(pRsp,ret.output,*ret.out_len);
#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
	/* LGE_CHAGNE [dojip.kim@lge.com] 2010-06-21, free the allocated mem */
	if (ret.output)
		kfree(ret.output);
	if (ret.out_len)
		kfree(ret.out_len);
#endif
#else
  ret.output = (char*)pRsp;
  ret.out_len = &out_len;

	oem_rapi_client_streaming_function(client, &arg, &ret);
#endif
}


/* LGE_CHANGES_S [woonghee.park@lge.com] 2010-02-09, [VS740]*/
#ifdef CONFIG_LGE_BATT_ID_CHECK
void battery_info_get(struct batt_info* resp_buf)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;
  uint32_t out_len;
  int ret_val;
  struct batt_info rsp_buf;

  Open_check();

  arg.event = LG_FW_A2M_BATT_INFO_GET;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
	arg.in_len = 0;
	arg.input = NULL;
	arg.out_len_valid = 1;
	arg.output_valid = 1;
	arg.output_size = sizeof(rsp_buf);

  ret.output = (char*)&rsp_buf;
  ret.out_len = &out_len;

  ret_val = oem_rapi_client_streaming_function(client, &arg, &ret);
  if(ret_val == 0)
  {
    resp_buf->valid_batt_id = GET_U_INT32(&rsp_buf.valid_batt_id);
    resp_buf->batt_therm = GET_U_INT32(&rsp_buf.batt_therm);
    resp_buf->batt_temp = GET_INT32(&rsp_buf.batt_temp);
  }
  else
  { // In case error
    resp_buf->valid_batt_id = 1; // authenticated battery id
    resp_buf->batt_therm = 100;  // 100 battery therm adc
    resp_buf->batt_temp = 30;     // 30 degree celcius
  }
  return;
}
EXPORT_SYMBOL(battery_info_get);

void pseudo_batt_info_set(struct pseudo_batt_info_type* info)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;

  Open_check();

  arg.event = LG_FW_A2M_PSEUDO_BATT_INFO_SET;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
	arg.in_len = sizeof(struct pseudo_batt_info_type);
	arg.input = (char*)info;
	arg.out_len_valid = 0;
	arg.output_valid = 0;
	arg.output_size = 0;  //alloc memory for response

  ret.output = (char*)NULL;
  ret.out_len = 0;

  oem_rapi_client_streaming_function(client, &arg, &ret);

#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
	/* LGE_CHAGNE [dojip.kim@lge.com] 2010-06-21, free the allocated mem */
	if (ret.output)
		kfree(ret.output);
	if (ret.out_len)
		kfree(ret.out_len);
#else
  return;
#endif
}
EXPORT_SYMBOL(pseudo_batt_info_set);
#endif /*CONFIG_LGE_BATT_ID_CHECK*/
/* LGE_CHANGES_E [woonghee.park@lge.com]*/

void msm_get_MEID_type(char* sMeid)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;
#if defined(CONFIG_LGE_USE_OEM_RAPI)
#else
  char output[LG_RAPI_CLIENT_MAX_OUT_BUFF_SIZE];
  uint32_t out_len;
#endif

  Open_check();
  
  arg.event = LG_FW_MEID_GET;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
  arg.in_len = 0;
  arg.input = NULL;
  arg.out_len_valid = 1;
  arg.output_valid = 1;
  arg.output_size = 15;

#if defined(CONFIG_LGE_USE_OEM_RAPI)
	ret.output = NULL;
	ret.out_len = NULL;

	oem_rapi_client_streaming_function(client, &arg, &ret);

	memcpy(sMeid,ret.output,14); 

#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
	/* LGE_CHAGNE [dojip.kim@lge.com] 2010-06-21, free the allocated mem */
	if (ret.output)
		kfree(ret.output);
	if (ret.out_len)
		kfree(ret.out_len);
#endif

#else
  ret.output = output;
  ret.out_len = &out_len;
 
  oem_rapi_client_streaming_function(client, &arg, &ret);

  memcpy(sMeid,output,14); 
  return;  
#endif
}

void set_opertion_mode(boolean info)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;

  Open_check();

  arg.event = LG_FW_SET_OPERATIN_MODE;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
  arg.in_len = sizeof(boolean);
  arg.input = (char*)&info;
  arg.out_len_valid = 0;
  arg.output_valid = 0;
  arg.output_size = 0;  //alloc memory for response

  ret.output = (char*)NULL;
  ret.out_len = 0;

  oem_rapi_client_streaming_function(client, &arg, &ret);
#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
  /* LGE_CHAGNE [dojip.kim@lge.com] 2010-06-21, free the allocated mem */
  if (ret.output)
    kfree(ret.output);
  if (ret.out_len)
    kfree(ret.out_len);
#else
  return;
#endif
}
EXPORT_SYMBOL(set_opertion_mode);

/* LGE_CHANGES_S [bk.shin@lge.com] 2010-07-06, [VS760]*/
#ifdef CONFIG_LGE_FOLDER_TEST_COUNT
void folder_count_get(int * count)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;
  uint32_t out_len;
  int ret_val;
  struct batt_info rsp_buf;

  Open_check();

  arg.event = LG_FW_FOLDER_COUNT_GET;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
  arg.in_len = 0;
  arg.input = NULL;
  arg.out_len_valid = 1;
  arg.output_valid = 1;
  arg.output_size = sizeof(int);

  ret.output = (char*)count;
  ret.out_len = &out_len;

  ret_val = oem_rapi_client_streaming_function(client, &arg, &ret);

  return;
}
EXPORT_SYMBOL(folder_count_get);

void folder_count_put(int * count)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;

  Open_check();

  arg.event = LG_FW_FOLDER_COUNT_PUT;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
  arg.in_len = sizeof(int);
  arg.input = (char*)count;
  arg.out_len_valid = 0;
  arg.output_valid = 0;
  arg.output_size = 0;  //alloc memory for response

  ret.output = (char*)NULL;
  ret.out_len = 0;

  oem_rapi_client_streaming_function(client, &arg, &ret);

  return;
}
EXPORT_SYMBOL(folder_count_put);
#endif /*CONFIG_LGE_FOLDER_TEST_COUNT*/

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
void key_led_onoff(int * onoff)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;

  Open_check();

  arg.event = LG_FW_KEY_LED_ON;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
  arg.in_len = sizeof(int);
  arg.input = (char*)onoff;
  arg.out_len_valid = 0;
  arg.output_valid = 0;
  arg.output_size = 0;  //alloc memory for response

  ret.output = (char*)NULL;
  ret.out_len = 0;

  oem_rapi_client_streaming_function(client, &arg, &ret);

#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
  /* LGE_CHAGNE [dojip.kim@lge.com] 2010-06-21, free the allocated mem */
  if (ret.output)
    kfree(ret.output);
  if (ret.out_len)
    kfree(ret.out_len);
#else
  return;
#endif
}
EXPORT_SYMBOL(key_led_onoff);

void lcd_backlight_sleep_check(int * sleep_on)
{
  struct oem_rapi_client_streaming_func_arg arg;
  struct oem_rapi_client_streaming_func_ret ret;

  Open_check();

  arg.event = LG_FW_LCD_BL_SLEEP;
  arg.cb_func = NULL;
  arg.handle = (void*) 0;
  arg.in_len = sizeof(int);
  arg.input = (char*)sleep_on;
  arg.out_len_valid = 0;
  arg.output_valid = 0;
  arg.output_size = 0;  //alloc memory for response

  ret.output = (char*)NULL;
  ret.out_len = 0;

  oem_rapi_client_streaming_function(client, &arg, &ret);

#ifdef CONFIG_LGE_MACH_ENVT2 //THUNDER_MIGRATION
  if (ret.output)
    kfree(ret.output);
  if (ret.out_len)
    kfree(ret.out_len);
#else
  return;
#endif
}
EXPORT_SYMBOL(lcd_backlight_sleep_check);
#endif

// LGE_CHANGE [dojip.kim@lge.com] 2010-09-01

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-10-02, charger logo notification to modem */
#ifdef CONFIG_LGE_CHARGER_LOGO_MODE_NOTI
void remote_set_chg_logo_mode(int info)
{
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;
	int ret_val;

	Open_check();

	arg.event = LG_FW_CHG_LOGO_MODE;
	arg.cb_func = NULL;
	arg.handle = (void *)0;
	arg.in_len = sizeof(int);
	arg.input = (char *)&info;
	arg.out_len_valid = 0;
	arg.output_valid = 0;
	arg.output_size = 0;	//alloc memory for response

	ret.output = NULL;
	ret.out_len = NULL;

	ret_val = oem_rapi_client_streaming_function(client, &arg, &ret);

	if (ret.output)
		kfree(ret.output);
	if (ret.out_len)
		kfree(ret.out_len);

	return;
}

EXPORT_SYMBOL(remote_set_chg_logo_mode);
#endif
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-10-02, charger logo notification to modem */

// LGE_CHANGE [dojip.kim@lge.com] 2010-09-01
void remote_set_charging_stat_realtime_update(int info)
{
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;
	int ret_val;

	Open_check();

	arg.event = LG_FW_SET_CHARGING_STAT_REALTIME_UPDATE;
	arg.cb_func = NULL;
	arg.handle = (void *)0;
	arg.in_len = sizeof(int);
	arg.input = (char *)&info;
	arg.out_len_valid = 0;
	arg.output_valid = 0;
	arg.output_size = 0;	//alloc memory for response

	ret.output = NULL;
	ret.out_len = NULL;

	ret_val = oem_rapi_client_streaming_function(client, &arg, &ret);

	if (ret.output)
		kfree(ret.output);
	if (ret.out_len)
		kfree(ret.out_len);

	return;
}

EXPORT_SYMBOL(remote_set_charging_stat_realtime_update);

void remote_get_charging_stat_realtime_update(int *info)
{
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;
	uint32_t out_len;
	int ret_val;
	int resp_buf;

	Open_check();

	arg.event = LG_FW_GET_CHARGING_STAT_REALTIME_UPDATE;
	arg.cb_func = NULL;
	arg.handle = (void *)0;
	arg.in_len = 0;
	arg.input = NULL;
	arg.out_len_valid = 1;
	arg.output_valid = 1;
	arg.output_size = sizeof(int);

	ret.output = NULL;
	ret.out_len = NULL;

	ret_val = oem_rapi_client_streaming_function(client, &arg, &ret);
	if (ret_val == 0) {
		memcpy(&resp_buf, ret.output, *ret.out_len);
		*info = GET_INT32(&resp_buf);
	} else {
		*info = 0;	//default value
	}

	if (ret.output)
		kfree(ret.output);
	if (ret.out_len)
		kfree(ret.out_len);

	return;
}
EXPORT_SYMBOL(remote_get_charging_stat_realtime_update);

MODULE_AUTHOR("khlee@lge.com>");
MODULE_DESCRIPTION("LGE rapi driver");
MODULE_LICENSE("GPL");

