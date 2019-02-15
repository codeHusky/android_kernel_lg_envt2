/* arch/arm/mach-msm/qdsp6/lm49250_audio_amp.c
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include  <mach/analog_audio_lm49250.h>

//LGE_AUDIO_SUBSYSTEM eklee
#define AUDIO_AMP_TUNING 1

extern void  Audio_drv_SetInit(void);
extern void  Audio_drv_SetMonoVol(u8 vol);
extern void  Audio_drv_SetStereoVol(u8 vol);
extern void  Audio_drv_SetHpLRVol(u8 vol);


#if 0
#include <mach/qdsp6/audio_amp_test.h>
#endif //AUDIO_AMP_TUNING

#define MODULE_NAME	"amp_lm49250"

#define	DEBUG_AMP_CTL	0 // 1

#if DEBUG_AMP_CTL
#define D(fmt, args...) printk(fmt, ##args)
#else
#define D(fmt, args...) do {} while(0)
#endif

struct amp_data {
	struct i2c_client *client;
};

static struct amp_data *_data = NULL;

bool amp_write_register(u8 reg)
{
	int				 err;
	unsigned char    buf[3];
	struct i2c_msg	msg = { _data->client->addr, 0, 1, &buf[0] }; 

	if(_data == NULL) {
		printk(KERN_INFO "AMP i2c _data null error! \n");
		return false;
	}
	buf[0] = reg;

	if ((err = i2c_transfer(_data->client->adapter, &msg, 1)) < 0){
			printk(KERN_INFO "AMP i2c write error! \n");
			return false;
	}
	else{
			D(KERN_INFO "AMP i2c write ok\n");
			return true;
	}
	return true;
}

#if AUDIO_AMP_TUNING
static ssize_t mono_in_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int value;
	
	audAmpLM49250_GetParam(AUDAMP_CMD_MONO_VOL,&value);
	return sprintf(buf, "%d\n", value);
}

static ssize_t mono_in_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	audAmpLM49250_SetParam(AUDAMP_CMD_MONO_VOL ,value);	
	return size;
}


static ssize_t stereo_in_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int value;
	
	audAmpLM49250_GetParam(AUDAMP_CMD_STEREO_VOL,&value);
	return sprintf(buf, "%d\n", value);
}

static ssize_t stereo_in_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;


	sscanf(buf, "%d", &value);

	audAmpLM49250_SetParam(AUDAMP_CMD_STEREO_VOL ,value);	
	return size;
}


static ssize_t ep_out_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int value;
	
	audAmpLM49250_GetParam(AUDAMP_CMD_EARPIECE_GAIN,&value);
	return sprintf(buf, "%d\n", value);
}

static ssize_t ep_out_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	audAmpLM49250_SetParam(AUDAMP_CMD_EARPIECE_GAIN ,value);	
	return size;
}

static ssize_t sp_out_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int value;
	
	audAmpLM49250_GetParam(AUDAMP_CMD_SPEAKER_GAIN,&value);
	return sprintf(buf, "%d\n", value);
}

static ssize_t sp_out_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	audAmpLM49250_SetParam(AUDAMP_CMD_SPEAKER_GAIN ,value);	
	return size;
}

static ssize_t hp_out_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int value;
	
	audAmpLM49250_GetParam(AUDAMP_CMD_HPH_GAIN_UP,&value);
	return sprintf(buf, "%d\n", value);
}

static ssize_t hp_out_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	audAmpLM49250_SetParam(AUDAMP_CMD_HPH_GAIN_UP ,value);	
	return size;
}

#if 0
//SOUND_MODE
static ssize_t sound_in1_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	return sprintf(buf, "%d\n", audio_amp_value_ptr->in1_vol);
}

static ssize_t sound_in1_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	sscanf(buf, "%d", &value);
	audio_amp_value_ptr->in1_vol=value;

	return size;
}

static ssize_t sound_in2_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	return sprintf(buf, "%d\n", audio_amp_value_ptr->in2_vol);
}

static ssize_t sound_in2_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	sscanf(buf, "%d", &value);
	audio_amp_value_ptr->in2_vol=value;

	return size;
}

static ssize_t sound_spk_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	return sprintf(buf, "%d\n", audio_amp_value_ptr->spk_vol);
}

static ssize_t sound_spk_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	sscanf(buf, "%d", &value);
	audio_amp_value_ptr->spk_vol=value;

	return size;
}

static ssize_t sound_spk_boost_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	return sprintf(buf, "%d\n", audio_amp_value_ptr->spk_boost);
}

static ssize_t sound_spk_boost_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	sscanf(buf, "%d", &value);
	audio_amp_value_ptr->spk_boost=value;

	return size;
}

static ssize_t sound_hp_vol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	return sprintf(buf, "%d\n", audio_amp_value_ptr->hpout_vol);
}

static ssize_t sound_hp_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	sscanf(buf, "%d", &value);
	audio_amp_value_ptr->hpout_vol=value;

	return size;
}

static ssize_t sound_call_mute_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	//audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	return sprintf(buf, "%d\n", audio_call_mute);
}


static ssize_t sound_call_mute_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	extern	void  audio_amp_control_off(bool force_off);
	extern	void  audio_amp_control_on(u32 dev_id);
	//extern 	u32 backup_dev_id;
	//audio_amp_value_ptr = &audio_amp_param_table[SOUND_MODE];

	sscanf(buf, "%d", &value);
	audio_call_mute=value;

#if 1
	if(audio_call_mute == 1){
		audio_amp_control_off(1);
	}else if((audio_call_mute == 0 && backup_dev_id!=0)){
		audio_amp_control_on(backup_dev_id);
	}
#endif
	return size;
}
#endif 

static ssize_t sound_loopback_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 55);
}

static ssize_t sound_loopback_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	bool loopbackmode;
#if 0
	extern void mvs_pkt_loopback_start(int selected_device);
	extern void mvs_pkt_loopback_stop(int selected_device);
#endif 
	sscanf(buf, "%d", &value);
	loopbackmode=value;
#if 0

	if(loopbackmode == 1){
		mvs_pkt_loopback_start(1);
	}else if(loopbackmode == 0){
		mvs_pkt_loopback_stop(1);
	}
#endif 

	return size;
}



//VOICE_CALL_MODE
static DEVICE_ATTR(m_in_vol, S_IRUGO | S_IWUGO, mono_in_vol_show, mono_in_vol_store);


static DEVICE_ATTR(st_in_vol, S_IRUGO | S_IWUGO, stereo_in_vol_show, stereo_in_vol_store);

static DEVICE_ATTR(ep_out_vol, S_IRUGO | S_IWUGO, ep_out_vol_show, ep_out_vol_store);
static DEVICE_ATTR(sp_out_vol, S_IRUGO | S_IWUGO, sp_out_vol_show, sp_out_vol_store);
static DEVICE_ATTR(hp_out_vol, S_IRUGO | S_IWUGO, hp_out_vol_show, hp_out_vol_store);
#if 0
//SOUND_MODE
static DEVICE_ATTR(s_in1_vol, S_IRUGO | S_IWUGO, sound_in1_vol_show, sound_in1_vol_store);
static DEVICE_ATTR(s_in2_vol, S_IRUGO | S_IWUGO, sound_in2_vol_show, sound_in2_vol_store);
static DEVICE_ATTR(s_spk_vol, S_IRUGO | S_IWUGO, sound_spk_vol_show, sound_spk_vol_store);
static DEVICE_ATTR(s_spk_boost, S_IRUGO | S_IWUGO, sound_spk_boost_show, sound_spk_boost_store);
static DEVICE_ATTR(s_hpout_vol, S_IRUGO | S_IWUGO, sound_hp_vol_show, sound_hp_vol_store);

static DEVICE_ATTR(call_mute_state, S_IRUGO | S_IWUGO, sound_call_mute_state_show, sound_call_mute_state_store);
#endif 

static DEVICE_ATTR(loopback_mode_on, S_IRUGO | S_IWUGO, sound_loopback_mode_show, sound_loopback_mode_store);

#endif //AUDIO_AMP_TUNING



static int lm49250_amp_ctl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct amp_data *data;
	struct i2c_adapter* adapter = client->adapter;
	int err;

#if AUDIO_AMP_TUNING
	err = device_create_file(&client->dev, &dev_attr_m_in_vol);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_v_in1_vol\n");
		return err;
	}

	err = device_create_file(&client->dev, &dev_attr_st_in_vol);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_v_in2_vol\n");
		return err;
	}

	err = device_create_file(&client->dev, &dev_attr_ep_out_vol);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_v_spk_vol\n");
		return err;
	}

	err = device_create_file(&client->dev, &dev_attr_sp_out_vol);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_v_spk_boost\n");
		return err;
	}

	err = device_create_file(&client->dev, &dev_attr_hp_out_vol);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_v_hpout_vol\n");
		return err;
	}

#if 0	
	//SOUND_MODE
	err = device_create_file(&client->dev, &dev_attr_s_in1_vol);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_s_in1_vol\n");
		return err;
	}

	err = device_create_file(&client->dev, &dev_attr_s_in2_vol);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_s_in2_vol\n");
		return err;
	}

	err = device_create_file(&client->dev, &dev_attr_s_spk_vol);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_s_spk_vol\n");
		return err;
	}

	err = device_create_file(&client->dev, &dev_attr_s_spk_boost);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_s_spk_boost\n");
		return err;
	}

	err = device_create_file(&client->dev, &dev_attr_s_hpout_vol);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_s_hpout_vol\n");
		return err;
	}

 	err = device_create_file(&client->dev, &dev_attr_call_mute_state);
	if (err) {
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_call_mute_state\n");
		return err;
	}
#endif //AUDIO_AMP_TUNING
#endif 
	err = device_create_file(&client->dev, &dev_attr_loopback_mode_on);
	if (err) {   
		printk( "wm9093_amp_ctl_probe: Fail dev_attr_loopback_mode_on\n");
		return err;
	}

 			
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
		err = -EOPNOTSUPP;
		return err;
	}
	
	printk(KERN_INFO "%s()\n", __FUNCTION__);
	data = kzalloc(sizeof (struct amp_data), GFP_KERNEL);
	if (NULL == data) {
			return -ENOMEM;
	}
	_data = data;
	data->client = client;
	i2c_set_clientdata(client, data);
	printk(KERN_INFO "%s chip found\n", client->name);
	
//	lm49250_init_client(client);


	Audio_drv_SetInit();

	return 0;
}

static int lm49250_amp_ctl_remove(struct i2c_client *client)
{
	struct amp_data *data = i2c_get_clientdata(client);
	kfree (data);
	
	printk(KERN_INFO "%s()\n", __FUNCTION__);
	i2c_set_clientdata(client, NULL);
	return 0;
}


static struct i2c_device_id lm49250_amp_idtable[] = {
	{ "amp_lm49250", 0 },
};

static struct i2c_driver lm49250_amp_ctl_driver = {
	.probe = lm49250_amp_ctl_probe,
	.remove = lm49250_amp_ctl_remove,
	.id_table = lm49250_amp_idtable,
	.driver = {
		.name = MODULE_NAME,
	},
};

static int __init lm49250_amp_ctl_init(void)
{
	return i2c_add_driver(&lm49250_amp_ctl_driver);	
}

static void __exit lm49250_amp_ctl_exit(void)
{
	return i2c_del_driver(&lm49250_amp_ctl_driver);
}

module_init(lm49250_amp_ctl_init);
module_exit(lm49250_amp_ctl_exit);

MODULE_DESCRIPTION("lm49250 Amp Control");
MODULE_AUTHOR("Eklee <eklee.lee@lge.com>");
MODULE_LICENSE("GPL");
