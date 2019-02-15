/* arch/arm/mach-msm/board-griffin-bl-bd6083.c
 *
 * Author:	  kenobi(SungYoung, Lee)
 * Created:	  Jan 13, 2009
 * Copyright: LGE Inc. All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/board.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
#include <mach/pmic.h>
#endif

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
#include <linux/power_supply.h>
#endif

typedef enum {
		ALC_MODE,
		NORMAL_MODE,
} AAT2870GU_MODE;

typedef enum {
		UNINIT_STATE=-1,
		POWERON_STATE,
		NORMAL_STATE,
		ALC_STATE,
		SLEEP_STATE,
		DIMMING_STATE,
		POWEROFF_STATE
} AAT2870GU_STATE;

struct aat2870gu_platform_data {
		void	(*platform_init)(void);
		unsigned int op_mode;
		spinlock_t	lock;
};

static struct i2c_client *s_client = NULL;
struct backlight_device *s_bd;

#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
struct aat2870_device {
	struct delayed_work key_backlight_work; // to turn off keypad backlight
};

static struct aat2870_device aat2870_dev;
static struct workqueue_struct *key_backlight_work_queue;
#define KEY_BACKLIGHT_STAY_TIME	5000 //5sec
#endif
#if 0
static int slcd_aat2870gu_read(u8 reg, u8 *val);
#endif
static int slcd_aat2870gu_write(u8 reg, u8 val);

#define SLCD_LED_MAX     	0x16 // 19.8mA  256
#define SLCD_LED_NOR		0x0B // 9.9mA 127
#define SLCD_LED_MIN		0 // 0x03 // 2mA 0
#define DEFAULT_BRIGHTNESS	0x0B

#define SUB_CP_EN			27

//#define CONFIG_VS760_AAT2870GU_BACKLIGHT_DEBUG
#ifdef CONFIG_VS760_AAT2870GU_BACKLIGHT_DEBUG
#define KENOBI_TRACE(fmt, args...)	printk(KERN_ERR "KENOBI-DBG[%-18s:%5d]" fmt, __FUNCTION__, __LINE__, ## args)
#define KENOBI_ERROR(fmt, args...)  printk(KERN_ERR "KENOBI-ERR[%-18s:%5d]" fmt, __FUNCTION__, __LINE__, ## args)
#else
#define KENOBI_TRACE(fmt, args...)
#define KENOBI_ERROR(fmt, args...)
#endif

struct aat2870gu_ctrl_tbl {
	unsigned char reg;
	unsigned char val;
};

//N-1) Power ON change to Normal mode
static struct aat2870gu_ctrl_tbl pwron_to_normal_mode[] = {
	//{0x01, 0x11},	//REG1 - Main backlight - current 15.3mA		
	/* Content Adaptive Brightness Contrl */	

	/* FADE IN */	
	{0x0C, 0x0D}, // REG12 Sub Fade in 0.4sec
	{0x00, 0xFF},	// REG0 (I2C Address 00h) - LED Turn On -backlinght channel enable LED1 ~ LED7
	{0xff, 0x01},	// 1 x 100 delay
	{0xff, 0xfe}	// end of command
};

//N-5-2) Nornmal mode change to sleep with slope down
static struct aat2870gu_ctrl_tbl change_mode_from_dimming_to_sleep[] = {	
	{0x0B, 0x00}, //Fade out current 0.45mA		
	{0x0C, 0x0C}, // REG12 Main Fade out 0.4sec	
	
	{0x00, 0x00},	// Main LED =OFF
	{0xff, 0x01},	// 1 x 100 delay
	{0xff, 0xfe}	// end of command
};

static int cur_sub_lcd_level;
static int prev_sub_lcd_level;
static AAT2870GU_MODE mode = NORMAL_MODE;
static AAT2870GU_STATE state = NORMAL_STATE;
static struct backlight_device *saved_bd = NULL;


typedef enum {
	OFF_MODE,
	VIBRATOR_MODE,
	SLCD_BACKLIGHT_MODE,
} chargepump_control;

static int sub_chargepump = 0; 

/* This function is reset hardware of a AAT2870GU chip.
  * AAT2870GU Hardware Reset: H -> L is hardware reset and low state still minimum 5us.
  * Under hardware reset, all registgers and output pinss are initialized, and I2C accecess are stopped.
  * L -> H release from hardware reset.
  * If you want to chnage assigned GPIO for SUB_CP_EN, you will only change upper definition.
  */

int get_slcd_backligit_status(void)
{
	int ret;
	if((state == POWEROFF_STATE) || (state == SLEEP_STATE))
		ret = 0;

	if((state == NORMAL_STATE) || (state == POWERON_STATE))
		ret = 1;

	return ret;
}

static DEFINE_SPINLOCK(slcd_reset_lock);

static void slcd_aat2870gu_hw_rst(void)
{
	KENOBI_TRACE("Sub HW Reset\n");

	/* Sub LCD Backlight */
	spin_lock(&slcd_reset_lock);
	
	gpio_tlmm_config(GPIO_CFG(SUB_CP_EN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
	gpio_direction_output(SUB_CP_EN, 1);
	mdelay(1);

	spin_unlock(&slcd_reset_lock);
}

static void slcd_aat2870gu_hw_off(void)
{
	KENOBI_TRACE("slcd_aat2870gu_hw_off : %d \n", sub_chargepump);
	if (sub_chargepump == 0) {
		KENOBI_TRACE(" slcd_aat2870gu power off \n");
		spin_lock(&slcd_reset_lock);
		
		gpio_tlmm_config(GPIO_CFG(SUB_CP_EN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
		gpio_direction_output(SUB_CP_EN, 0);
		
		spin_unlock(&slcd_reset_lock);
	}
}

#define MAINTAIN_LDO_REGISTER_STATUS 1
#if MAINTAIN_LDO_REGISTER_STATUS
/* XXX: workaround to maintain current register value because i2c_read returns always zero. */
#define INDEX_26H_LDO_ONOFF_CONTROL     0
#define INDEX_24H_28VOUT_CONTROL    	1 /* 2.8V_Motor */


static unsigned char aat2870_ldo_reg_stats[2];
#endif /* MAINTAIN_LDO_REGISTER_STATUS */

struct ldo_vout_struct {
        unsigned char reg_value;
        unsigned mv;
};

static struct ldo_vout_struct ldo_vout_table[] = {
		/* LDOA (REG36 24h) - Bit7 - Bit4 */
        {/* 0000 */ 0x00, 1200},
        {/* 0001 */ 0x10, 1300},
        {/* 0010 */ 0x20, 1500},
        {/* 0011 */ 0x30, 1600},
        {/* 0100 */ 0x40, 1800},
        {/* 0101 */ 0x50, 2200},
        {/* 0110 */ 0x06, 2400},
        {/* 0111 */ 0x70, 2500},
        {/* 1000 */ 0x80, 2600},
        {/* 1001 */ 0x90, 2700},
        {/* 1010 */ 0xA0, 2800},
        {/* 1011 */ 0xB0, 2900},
        {/* 1100 */ 0xC0, 3000},
        {/* 1101 */ 0xD0, 3100},
        {/* 1110 */ 0xE0, 3200},
        {/* 1111 */ 0xF0, 3300},
        {/* Invalid */ 0xFF, 0},
};

static unsigned char slcd_aat2870_find_ldo_vout_reg_value(unsigned mv)
{
        int i = 0;
        do {
            if (ldo_vout_table[i].mv == mv)
                return ldo_vout_table[i].reg_value;
            else
                i++;
        } while (ldo_vout_table[i].mv != 0);

        return ldo_vout_table[i].reg_value;
}


int slcd_aat2870_set_ldo_vout(int ldo_no, unsigned vout_mv)
{
	unsigned char target_reg;
	unsigned char reg_val;
	unsigned char tmp;

	switch (ldo_no)
	{
		case 1:
		case 2:
			target_reg = 0x24;
#if MAINTAIN_LDO_REGISTER_STATUS
			tmp = aat2870_ldo_reg_stats[INDEX_24H_28VOUT_CONTROL];
#endif
			break;
		case 3:
		case 4:                
		default:
			return -1;
	}

	reg_val = slcd_aat2870_find_ldo_vout_reg_value(vout_mv);
	if (reg_val == 0xFF)
		return -1;
	KENOBI_TRACE("%s: found vout register value 0x%x\n", __func__, reg_val);

	if (s_client != NULL)
	{		
#if !MAINTAIN_LDO_REGISTER_STATUS
		aat2870gu_read(target_reg, &tmp);
#endif /* MAINTAIN_LDO_REGISTER_STATUS */

		if (ldo_no % 2) 
		{
			tmp &= ~0X0F;
		} else 
		{
			tmp &= ~0xF0;
			reg_val = reg_val << 4;
		}

		tmp |= reg_val;

		KENOBI_TRACE("%s: target register[0x%x], value[0x%x]\n", __func__, target_reg, tmp);
		if(slcd_aat2870gu_write(target_reg, tmp))
			return -1;
#if MAINTAIN_LDO_REGISTER_STATUS
		aat2870_ldo_reg_stats[INDEX_24H_28VOUT_CONTROL] = tmp;
#endif
		{
#if 0		
			tmp = 0;
			slcd_aat2870gu_read(target_reg, &tmp);
			KENOBI_TRACE("%s: VERIFY register[0x%x], value[0x%x]\n", __func__, target_reg, tmp);
#endif
		}
		return 0;
	}
	else 
	{
		return -1;
	}
}
EXPORT_SYMBOL(slcd_aat2870_set_ldo_vout);

int slcd_aat2870_set_ldo_power(int ldo_no, int on)
{
	unsigned char tmp;
	unsigned char new_val;

	KENOBI_TRACE("\n %s: ldo_no[%d], on/off[%d]\n", __func__, ldo_no, on);

	if (ldo_no > 0 && ldo_no < 5)
		new_val = 1 << (ldo_no - 1);
	else
		return -1;

	if (s_client != NULL) 
	{

		KENOBI_TRACE("%s: ldo_no[%d], on/off[%d]\n", __func__, ldo_no, on);
		
		if(ldo_no == 1) {
			if(state == SLEEP_STATE) {
				slcd_aat2870gu_hw_rst();
			}
			if(slcd_aat2870_set_ldo_vout(1,3300))
				return -1;
		}
		
#if MAINTAIN_LDO_REGISTER_STATUS
		tmp = aat2870_ldo_reg_stats[INDEX_26H_LDO_ONOFF_CONTROL];
#else
		/* TODO: check return value */
#if 0
		if(slcd_aat2870gu_read(0x26, &tmp))
			KENOBI_TRACE("i2c read failed addr:0x13, value:%d\n", tmp);
#endif					 
		KENOBI_TRACE("%s: current ldo power register value 0x%x\n", __func__, tmp);
#endif /* MAINTAIN_LDO_REGISTER_STATUS */

		if (on)
			tmp |= new_val;
		else
			tmp &= ~new_val;

		KENOBI_TRACE("%s: new ldo power register value 0x%x\n", __func__, tmp);
		if(slcd_aat2870gu_write(0x26, tmp))
			return -1;
#if MAINTAIN_LDO_REGISTER_STATUS
		aat2870_ldo_reg_stats[INDEX_26H_LDO_ONOFF_CONTROL] = tmp;
#endif
		{    
#if 0		
			tmp = 0;
			slcd_aat2870gu_read(0x26, &tmp);
			KENOBI_TRACE("%s: VERIFY register[0x13], value[0x%x]\n", __func__, tmp);
#endif			
		}

		if (ldo_no == 1) {
			if (on){
				sub_chargepump =(1<<VIBRATOR_MODE);
			} else if((on == 0) && ( sub_chargepump &(1<<VIBRATOR_MODE))){
				sub_chargepump &=~(1<<VIBRATOR_MODE);
			}
			if(state == SLEEP_STATE) {
				if(on == 0) slcd_aat2870gu_hw_off();
			}
		}

		return 0;
	} 
	else 
	{
		return -1;
	}
}
EXPORT_SYMBOL(slcd_aat2870_set_ldo_power);

#if 0
/* Read register value using pxa-i2c */
int slcd_aat2870gu_read(u8 reg, u8 *pval)
{
	int ret;
	int status;
	
	struct aat2870gu_platform_data *pdata = NULL;
	
	if (s_client == NULL) /* No global client pointer? */
	{ 	
		KENOBI_ERROR("s_client is null\n");
		return -1;
	}
		
	pdata = s_client->dev.platform_data;		

	spin_lock(&pdata->lock);

	if ((ret = i2c_smbus_read_byte_data(s_client, reg)) >= 0) 
	{
		*pval = ret;
		status = 0;
	} else 
	{
		status = -EIO;
		KENOBI_ERROR("fail to read(reg=0x%x,val=0x%x)\n", reg,*pval);
	}

	spin_unlock(&pdata->lock);		

	return status;
}
#endif
/* Write value in the register using pxa-i2c */
int slcd_aat2870gu_write(u8 reg, u8 val)
{
	int ret;
	int status;
	
	struct aat2870gu_platform_data *pdata = NULL;
	
	if (s_client == NULL) /* No global client pointer? */
	{		
		KENOBI_ERROR("s_client is null\n");
		return -1;
	}
		
	pdata = s_client->dev.platform_data;
		
	spin_lock(&pdata->lock);

	ret = i2c_smbus_write_byte_data(s_client, reg, val);
	if (ret == 0) 
	{
		status = 0;
	} 
	else 
	{
		status = -EIO;
		KENOBI_ERROR("fail to write(reg=0x%x,val=0x%x)\n", reg, val);
	}
	spin_unlock(&pdata->lock);		

	return status;
}


/* This function is set AAT2870GU register iterately from state transition table 
 * A 0xFF mean 2000us delay.
*/
static int slcd_aat2870gu_set_tbl(struct aat2870gu_ctrl_tbl *ptbl)
{
	unsigned int i = 0;

	if (ptbl == NULL) {
		KENOBI_ERROR("input ptr is null\n");
		return -EIO;
	}

	for( ;;) 
	{
		if (ptbl->reg == 0xFF) 
		{
			if (ptbl->val != 0xfe)
				mdelay(ptbl->val*10);
			else
				break;
		}	
		else 
		{
			if(slcd_aat2870gu_write(ptbl->reg, ptbl->val))
				KENOBI_TRACE("i2c failed addr:%d, value:%d\n", ptbl->reg, ptbl->val);
		}
		ptbl++;
		i++;
	}
	
	return 0;
}

static void aat2870_set_slcd_current_level(int level)
{
	unsigned char ulevel;

	KENOBI_TRACE("%s() : current backlight brightness level is : %d\n", __FUNCTION__, level);
	
	ulevel = (unsigned char)level;
	
	cur_sub_lcd_level = ulevel;
	
	if( level == 0 )
		slcd_aat2870gu_set_tbl(&change_mode_from_dimming_to_sleep[0]);
	else {
		if(cur_sub_lcd_level != prev_sub_lcd_level) {
			slcd_aat2870gu_write(0x01, ulevel); //current setting
			slcd_aat2870gu_write(0x00, 0xFF); //LED Turn On -backlinght channel enable
			prev_sub_lcd_level = cur_sub_lcd_level;
		}
	}	
}

/* This functin initial AAT2870GU chip through hardware reset and address normal or alc mode. 
 * A mode initialied normal mode. but you can control by /sys/class/backlight/aat2870gu-bl/mode
 * This function is called by probe method.
*/
void slcd_aat2870gubl_init(void)
{
	/* AAT2870GU Hardware Reset using SUB_CP_EN(GPIOxx) pin */
	slcd_aat2870gu_hw_rst();
	
	/* FIXME: Is it need? */
	msleep(5);		

	/* Enter Operation Mode(Non-ALC or ALC) according to mode */	
	switch (mode) 
	{
		case NORMAL_MODE:
			slcd_aat2870gu_write(0x01, DEFAULT_BRIGHTNESS); //current setting	
			slcd_aat2870gu_set_tbl(&pwron_to_normal_mode[0]);
			state = NORMAL_STATE;
			break;
		case ALC_MODE:
		default:
			KENOBI_ERROR("Invalid Mode\n");
			break;
	}	
}

EXPORT_SYMBOL(slcd_aat2870gubl_init);

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
static int slcd_bl_onoff = 1;
int key_led_blink_running = 0;
static int key_led_on = 1;
extern void key_led_onoff(int * onoff);
extern void lcd_backlight_sleep_check(int * sleep_on);
extern u32 msm_batt_info_status_get(void);
#endif

//LG_FW : jinho.jang - for key backlight
#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
void keypad_backlight_off(void)
{	
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK	
    key_led_on = 0;
	key_led_onoff(&key_led_on);
	return;
#else
	pmic_set_led_intensity(0, 0);
#endif
}

static void key_backlight_turnoff_work(struct work_struct *work)
{
	keypad_backlight_off();
}

void keypad_backlight_on(void)
{
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
	key_led_on = 1;
	key_led_onoff(&key_led_on);
#else
	pmic_set_led_intensity(0, 1);
#endif
	queue_delayed_work(key_backlight_work_queue, &aat2870_dev.key_backlight_work, msecs_to_jiffies(KEY_BACKLIGHT_STAY_TIME)); // 5sec
}
#endif

/* This function provide AAT2870GU sleep enter routine for power management. */
void slcd_backlight_sleep(void)
{
	if (s_client == NULL)
		return;	
	
	if ((state == POWEROFF_STATE) || (state == SLEEP_STATE))
		return;

	KENOBI_TRACE("%s() : go to SLEEP,  state is : %d\n", __FUNCTION__, state);

	if(sub_chargepump &(1<<SLCD_BACKLIGHT_MODE))
		sub_chargepump &=~(1<<SLCD_BACKLIGHT_MODE);

	aat2870_set_slcd_current_level(SLCD_LED_MIN);
	slcd_aat2870gu_hw_off();

	state = SLEEP_STATE;
	
//LG_FW : jinho.jang - for key backlight
#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#if 0
	if(!get_charging_disp())
#endif
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
		keypad_backlight_off();
#endif

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
    if(msm_batt_info_status_get() == POWER_SUPPLY_STATUS_CHARGING && slcd_bl_onoff == 1)
    {
		slcd_bl_onoff = 0;
        if(!is_folder_open())
        {
          lcd_backlight_sleep_check(&slcd_bl_onoff);
          key_led_blink_running = 1;
        }
    }
#endif
}

EXPORT_SYMBOL(slcd_backlight_sleep);

/* This function provide AAT2870GU sleep exit routine for power management. */
void slcd_backlight_resume(int level)
{
	unsigned char ulevel;
	
	if (s_client == NULL)
		return;	

	if(is_folder_open())
		return;

	KENOBI_TRACE("%s() : SLEEP Exit,  level is : %d\n", __FUNCTION__, level);

	if ((state == POWEROFF_STATE) || (state == SLEEP_STATE)) // when resume is called after sleep
	{
		ulevel = (unsigned char)level;
		slcd_aat2870gu_hw_rst();
		slcd_aat2870gu_write(0x01, ulevel); //current setting
		slcd_aat2870gu_set_tbl(&pwron_to_normal_mode[0]);
		state = NORMAL_STATE;
		prev_sub_lcd_level = level;
		
	//LG_FW : jinho.jang - for key backlight
#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#if 0
		if(!get_charging_disp())
#endif
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
			keypad_backlight_on();
#endif	
		sub_chargepump =(1<<SLCD_BACKLIGHT_MODE);
	}
	else if (state == NORMAL_STATE) //after wake up, jsut current setting
	{
		aat2870_set_slcd_current_level(level);
	}	

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
	if(slcd_bl_onoff == 0)
	{
		slcd_bl_onoff = 1;
		lcd_backlight_sleep_check(&slcd_bl_onoff);
		key_led_blink_running = 0;
	}
#endif
}

EXPORT_SYMBOL(slcd_backlight_resume);


/* LG_FW : 2010.02.25 jinho.jang - lcd backlight level set */
void slcd_backlight_set_level( int level)
{
	if (level > SLCD_LED_MAX)
		level = SLCD_LED_MAX;

	if(s_client!=NULL )
	{
		cur_sub_lcd_level = level;
		
		if(cur_sub_lcd_level == 0) {
			slcd_backlight_sleep();
		} else {
			slcd_backlight_resume(cur_sub_lcd_level);
		}

		KENOBI_TRACE("%s() : level is : %d\n", __FUNCTION__, cur_sub_lcd_level);
	}else{
		KENOBI_TRACE("%s(): No client\n",__FUNCTION__);
	}
}
EXPORT_SYMBOL(slcd_backlight_set_level);


static int slcd_aat2870gubl_suspend(struct i2c_client *i2c_dev, pm_message_t state)
{
	/* sleep mode in  */
	return 0;
}

static int slcd_aat2870gubl_resume(struct i2c_client *i2c_dev)
{
	/* sleep mode out */	
	return 0;
}

static int slcd_aat2870gubl_set_intensity(struct backlight_device *bd)
{
	return 0;
}

static int slcd_aat2870gubl_get_intensity(struct backlight_device *bd)
{
	bd->props.brightness = cur_sub_lcd_level;
	return cur_sub_lcd_level;
}

static struct backlight_ops slcd_aat2870gubl_ops = {
	.get_brightness = slcd_aat2870gubl_get_intensity,
	.update_status  = slcd_aat2870gubl_set_intensity,
};

static int __init slcd_aat2870gubl_probe(struct i2c_client *i2c_dev, const struct i2c_device_id *i2c_dev_id)
{
	struct aat2870gu_platform_data *pdata;

//LG_FW : jinho.jang - for key backlight
#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
	struct aat2870_device *dev;
	dev = &aat2870_dev;
#endif

	if(i2c_dev_id)
	{
		s_client = i2c_dev;

		KENOBI_TRACE("s_client addr=0x%x\n", s_client->addr);
		printk("s_client addr=0x%x\n", s_client->addr);

		pdata = i2c_dev->dev.platform_data;

 		/* init spinlock */
		spin_lock_init(&pdata->lock);

		pdata->platform_init();
	}

	printk("entering aat2870gu_bl probe function \n");
	s_bd = backlight_device_register("slcd_aat2870gu-bl", &i2c_dev->dev, NULL, &slcd_aat2870gubl_ops);
	if (IS_ERR(s_bd)) {
		printk("entering slcd_aat2870gu_bl probe function error \n");
		return PTR_ERR(s_bd);
	}

	i2c_set_clientdata(i2c_dev, s_bd);

	s_bd->props.power = FB_BLANK_UNBLANK;
	s_bd->props.brightness = DEFAULT_BRIGHTNESS;
	s_bd->props.max_brightness = SLCD_LED_MAX;
	cur_sub_lcd_level = DEFAULT_BRIGHTNESS;

	/* save the pointer of backlight device structure for debug */
	saved_bd = s_bd;

#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
    slcd_aat2870_set_ldo_vout(1,3300);
#endif	

//key backlight turn on / off control delaywork
//LG_FW : jinho.jang - for key backlight
#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
	INIT_DELAYED_WORK(&dev->key_backlight_work, key_backlight_turnoff_work);	

	if(is_folder_open())
	{
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
	    slcd_bl_onoff = 0;
#endif
		keypad_backlight_off();
	}
	else
	{
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
	    slcd_bl_onoff = 1;
#endif
		keypad_backlight_on();		
	}
#endif

	printk("END aat2870gu_bl probe function\n");
	return 0;
}

static int slcd_aat2870gubl_remove(struct i2c_client *i2c_dev)
{	
	s_bd->props.brightness = 0;
	s_bd->props.power = 0;

	backlight_device_unregister(s_bd);
	i2c_set_clientdata(i2c_dev, NULL);

//LG_FW : jinho.jang - for key backlight
#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
	kfree(&aat2870_dev);
#endif

	return 0;
}

static struct i2c_device_id slcd_aat2870gu_idtable[] = {
        { "slcd_aat2870gu", 0 },
};

MODULE_DEVICE_TABLE(i2c, slcd_aat2870gu_idtable);

static struct i2c_driver slcd_aat2870gu_driver = {
	.probe 		= slcd_aat2870gubl_probe,
	.remove 	= slcd_aat2870gubl_remove,
	.suspend 	= slcd_aat2870gubl_suspend,
	.resume 	= slcd_aat2870gubl_resume,
	.id_table 	= slcd_aat2870gu_idtable,
	.driver = {
		.name = "slcd_aat2870gu",
		.owner = THIS_MODULE,
	},
};

static int __init slcd_backlight_init(void)
{
	int ret = 0;
	printk("AAT2870GU init start\n");

//LG_FW : jinho.jang - for key backlight
#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
	key_backlight_work_queue = create_singlethread_workqueue("key_backlight_work_queue");
	if (!key_backlight_work_queue)
		return -ENOMEM;
	
	memset(&aat2870_dev, 0, sizeof(struct aat2870_device));
#endif

	ret = i2c_add_driver(&slcd_aat2870gu_driver);

	return ret;
}

static void __exit slcd_backlight_exit(void)
{
	i2c_del_driver(&slcd_aat2870gu_driver);
}

module_init(slcd_backlight_init);
module_exit(slcd_backlight_exit);

MODULE_LICENSE("GPL");

