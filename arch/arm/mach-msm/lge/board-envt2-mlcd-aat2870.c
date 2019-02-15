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

static struct i2c_client *m_client = NULL;

struct backlight_device *m_bd;

static int mlcd_aat2870gu_read(u8 reg, u8 *val);
static int mlcd_aat2870gu_write(u8 reg, u8 val);

#define MLCD_LED_MAX     	0x16 // 19.8mA  256
#define MLCD_LED_NOR		0x0B // 9.9mA 127
#define MLCD_LED_MIN		0 // 0x03 // 2mA 0
#define DEFAULT_BRIGHTNESS	0x0B // MLCD_LED_MAX 

#define MAIN_CP_EN			142


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
	{0xff, 0x01/*0x32*/},	// 50 x 100 delay
	{0xff, 0xfe}	// end of command
};

static struct aat2870gu_ctrl_tbl change_mode_from_normal_to_dimming[] = {
	{0x01, 0x00},	// Current 0.45mA Setting
	{0x0C, 0x08},	// Fade Out, Time 0.6sec	
	{0x00, 0xFF},	// LED Turn On -backlinght channel enable
	{0xff, 0x05},	// 50 x 100 delay
	{0xff, 0xfe}	// end of command
};

static struct aat2870gu_ctrl_tbl change_mode_from_dimming_to_normal[] = {
	{0x01, 0x11},	// Current 15.3mA Setting
	{0x0C, 0x01},	// Fade In operration, Time 1sec
	{0x00, 0xFF},	// LED Turn On -backlinght channel enable
	{0xff, 0x05},	// 50 x 100 delay
	{0xff, 0xfe}	// end of command
};

//N-5-2) Nornmal mode change to sleep with slope down
static struct aat2870gu_ctrl_tbl change_mode_from_dimming_to_sleep[] = {	
	{0x0B, 0x00}, //Fade out current 0.45mA		
	{0x0C, 0x0C}, // REG12 Main Fade out 0.4sec	
	
	{0x00, 0x00},	// Main LED =OFF
	{0xff, 0x05},	// 50 x 100 delay
	{0xff, 0xfe}	// end of command
};

static int cur_main_lcd_level;
static int prev_main_lcd_level;
static AAT2870GU_MODE mode = NORMAL_MODE;
static AAT2870GU_STATE state = NORMAL_STATE;
static struct backlight_device *saved_bd = NULL;

static int camera_power_on = 0;
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
extern int key_led_blink_running;
extern void lcd_backlight_sleep_check(int * sleep_on);
#endif

int get_mlcd_backligit_status(void)
{
	int ret;
	if((state == POWEROFF_STATE) || (state == SLEEP_STATE))
		ret = 0;

	if((state == NORMAL_STATE) || (state == POWERON_STATE))
		ret = 1;

	return ret;
}

static DEFINE_MUTEX(control_lock);

/* This function is reset hardware of a AAT2870GU chip.
  * AAT2870GU Hardware Reset: H -> L is hardware reset and low state still minimum 5us.
  * Under hardware reset, all registgers and output pinss are initialized, and I2C accecess are stopped.
  * L -> H release from hardware reset.
  * If you want to chnage assigned GPIO for MAIN_CP_EN, you will only change upper definition.
  */
static void mlcd_aat2870gu_hw_rst(void)
{

	/* Sub LCD Backlight */
	gpio_tlmm_config(GPIO_CFG(MAIN_CP_EN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
	gpio_direction_output(MAIN_CP_EN, 1);	
	mdelay(1);
}

#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
int mlcd_aat2870_camera_power_on(void)
{
	unsigned char reg36_value=0, reg37_value=0, reg38_value=0 ;

	if((state == SLEEP_STATE) || (state == POWEROFF_STATE))
		mlcd_aat2870gu_hw_rst();

	camera_power_on = 1;

	/* output voltage control */
	if(mlcd_aat2870gu_write(0x24, 0x4A)) //LDOA(CAM DVDD 1.8V),  LDOB(CAM AVDD 2.8V)
		KENOBI_TRACE("i2c fail, mlcd_aat2870_camera_power_on - LDO A,B");

	mlcd_aat2870gu_read(0x24, &reg36_value);
	KENOBI_TRACE("%s: VERIFY REG36(0x24), value[0x%x]\n", __func__, reg36_value);
	
	if(mlcd_aat2870gu_write(0x25, 0x8A)) //LDOC(CAM IO 2.6V),  LDOD(CAM AF 2.8V)
		KENOBI_TRACE("i2c fail, mlcd_aat2870_camera_power_on - LDO C,D");

	mlcd_aat2870gu_read(0x25, &reg37_value);
	KENOBI_TRACE("%s: VERIFY REG37(0x25), value[0x%x]\n", __func__, reg37_value);

	/* turn on */
	if(mlcd_aat2870gu_write(0x26, 0x0F))
		KENOBI_TRACE("i2c fail, mlcd_aat2870_camera_power_on - turn on");

	mlcd_aat2870gu_read(0x26, &reg38_value);
	KENOBI_TRACE("%s: VERIFY REG38(0x26), value[0x%x]\n", __func__, reg38_value);

	return 0;
}

int mlcd_aat2870_camera_power_off(void)
{
	/* turn off */
	if(mlcd_aat2870gu_write(0x26, 0x00))
		KENOBI_TRACE("i2c fail, mlcd_aat2870_camera_power_off");

	if((state == SLEEP_STATE) || (state == POWEROFF_STATE))
		gpio_direction_output(MAIN_CP_EN, 0);

	camera_power_on = 0;

	return 0;
}

EXPORT_SYMBOL(mlcd_aat2870_camera_power_on);
EXPORT_SYMBOL(mlcd_aat2870_camera_power_off);
#endif /*CONFIG_LGE_MACH_ENVT2_REVD*/
/* Read register value using pxa-i2c */
int mlcd_aat2870gu_read(u8 reg, u8 *pval)
{
	int ret;
	int status;
	
	struct aat2870gu_platform_data *pdata = NULL;
	
	if (m_client == NULL) /* No global client pointer? */
	{ 	
		KENOBI_ERROR("m_client is null\n");
		return -1;
	}
		
	pdata = m_client->dev.platform_data;		

	spin_lock(&pdata->lock);

	if ((ret = i2c_smbus_read_byte_data(m_client, reg)) >= 0) 
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

/* Write value in the register using pxa-i2c */
int mlcd_aat2870gu_write(u8 reg, u8 val)
{
	int ret;
	int status;
	
	struct aat2870gu_platform_data *pdata = NULL;
	
	if (m_client == NULL) /* No global client pointer? */
	{		
		KENOBI_ERROR("m_client is null\n");
		return -1;
	}
		
	pdata = m_client->dev.platform_data;
		
	spin_lock(&pdata->lock);

	ret = i2c_smbus_write_byte_data(m_client, reg, val);
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
static int mlcd_aat2870gu_set_tbl(struct aat2870gu_ctrl_tbl *ptbl)
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
			if(mlcd_aat2870gu_write(ptbl->reg, ptbl->val))
				KENOBI_TRACE("i2c failed addr:%d, value:%d\n", ptbl->reg, ptbl->val);
		}
		ptbl++;
		i++;
	}
	
	return 0;
}

static void aat2870_set_mlcd_current_level(int level)
{
	unsigned char ulevel;

	KENOBI_TRACE("%s() : current backlight brightness level is : %d\n", __FUNCTION__, level);
	
	ulevel = (unsigned char)level;
	
	cur_main_lcd_level = ulevel;
	
	if( level == 0 )
		mlcd_aat2870gu_set_tbl(&change_mode_from_dimming_to_sleep[0]);
	else {
		if(cur_main_lcd_level != prev_main_lcd_level) {
			mlcd_aat2870gu_write(0x01, ulevel); //current setting
			mlcd_aat2870gu_write(0x00, 0xFF); //LED Turn On -backlinght channel enable
			prev_main_lcd_level = cur_main_lcd_level;
		}
	}	
}
/* This functin initial AAT2870GU chip through hardware reset and address normal or alc mode. 
 * A mode initialied normal mode. but you can control by /sys/class/backlight/aat2870gu-bl/mode
 * This function is called by probe method.
*/
void mlcd_aat2870gubl_init(void)
{
	/* AAT2870GU Hardware Reset using SUB_CP_EN(GPIOxx) pin */
	mlcd_aat2870gu_hw_rst();
	
	/* FIXME: Is it need? */
	msleep(5);		

	/* Enter Operation Mode(Non-ALC or ALC) according to mode */	
	switch (mode) 
	{
		case NORMAL_MODE:
			mlcd_aat2870gu_write(0x01, DEFAULT_BRIGHTNESS); //current setting	
			mlcd_aat2870gu_set_tbl(&pwron_to_normal_mode[0]);
			state = NORMAL_STATE;
			break;
		case ALC_MODE:
		default:
			KENOBI_ERROR("Invalid Mode\n");
			break;
	}	
}

EXPORT_SYMBOL(mlcd_aat2870gubl_init);


/* This function provide AAT2870GU sleep enter routine for power management. */
void mlcd_backlight_sleep(void)
{
	if (m_client == NULL)
		return;
	
	if ((state == POWEROFF_STATE) || (state == SLEEP_STATE))
		return;

	KENOBI_TRACE("%s() : go to SLEEP,  state is : %d\n", __FUNCTION__, state);

	aat2870_set_mlcd_current_level(MLCD_LED_MIN);

	if(!camera_power_on)
		gpio_direction_output(MAIN_CP_EN, 0);
	
	state = SLEEP_STATE;
}

EXPORT_SYMBOL(mlcd_backlight_sleep);

/* This function provide AAT2870GU sleep exit routine for power management. */
void mlcd_backlight_resume(int level)
{
	unsigned char ulevel;
	
	if (m_client == NULL)
		return;

	if(!is_folder_open())
		return;

	KENOBI_TRACE("%s() : SLEEP Exit,  level is : %d\n", __FUNCTION__, level);
	
	if ((state == POWEROFF_STATE) || (state == SLEEP_STATE)) // when resume is called after sleep
	{
		ulevel = (unsigned char)level;
		mlcd_aat2870gu_hw_rst();
		mlcd_aat2870gu_write(0x01, ulevel); //current setting
		mlcd_aat2870gu_set_tbl(&pwron_to_normal_mode[0]);
		state = NORMAL_STATE;
		prev_main_lcd_level = level;		
	}
	else if (state == NORMAL_STATE) //after wake up, jsut current setting
	{
		aat2870_set_mlcd_current_level(level);
	}

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-09-17, keypad led blinking when phone is charging */
#ifdef CONFIG_LGE_CHARGER_KEY_LED_BLINK
    if(key_led_blink_running == 1)
    {
        int onoff = 1;
		lcd_backlight_sleep_check(&onoff);
		key_led_blink_running = 0;
    }
#endif
}
EXPORT_SYMBOL(mlcd_backlight_resume);


/* LG_FW : 2010.02.25 jinho.jang - lcd backlight level set */
void mlcd_backlight_set_level( int level)
{
	if (level > MLCD_LED_MAX)
		level = MLCD_LED_MAX;

	if(m_client!=NULL )
	{
		cur_main_lcd_level = level;
		
		if(cur_main_lcd_level == 0) {
			mlcd_backlight_sleep();
		} else {
			mlcd_backlight_resume(cur_main_lcd_level);
		}

		KENOBI_TRACE("%s() : level is : %d\n", __FUNCTION__, cur_main_lcd_level);
	}else{
		KENOBI_TRACE("%s(): No client\n",__FUNCTION__);
	}
}
EXPORT_SYMBOL(mlcd_backlight_set_level);


static int mlcd_aat2870gubl_suspend(struct i2c_client *i2c_dev, pm_message_t state)
{
	/* sleep mode in  */		
	return 0;
}

static int mlcd_aat2870gubl_resume(struct i2c_client *i2c_dev)
{
	/* sleep mode out */	
	return 0;
}

static int mlcd_aat2870gubl_set_intensity(struct backlight_device *bd)
{
	return 0;
}

static int mlcd_aat2870gubl_get_intensity(struct backlight_device *bd)
{
	bd->props.brightness = cur_main_lcd_level;
	return cur_main_lcd_level;
}


static struct backlight_ops mlcd_aat2870gubl_ops = {
	.get_brightness = mlcd_aat2870gubl_get_intensity,
	.update_status  = mlcd_aat2870gubl_set_intensity,
};

static int __init mlcd_aat2870gubl_probe(struct i2c_client *i2c_dev, const struct i2c_device_id *i2c_dev_id)
{
	struct aat2870gu_platform_data *pdata;

	if(i2c_dev_id)
	{
		m_client = i2c_dev;

		KENOBI_TRACE("m_client addr=0x%x\n", m_client->addr);
		printk("m_client addr=0x%x\n", m_client->addr);

		pdata = i2c_dev->dev.platform_data;

 		/* init spinlock */
		spin_lock_init(&pdata->lock);

		pdata->platform_init();
	}

	printk("entering aat2870gu_bl probe function \n");
	m_bd = backlight_device_register("mlcd_aat2870gu-bl", &i2c_dev->dev, NULL, &mlcd_aat2870gubl_ops);
	if (IS_ERR(m_bd)) {
		printk("entering mlcd_aat2870gu_bl probe function error \n");
		return PTR_ERR(m_bd);
	}

	i2c_set_clientdata(i2c_dev, m_bd);

	m_bd->props.power = FB_BLANK_UNBLANK;
	m_bd->props.brightness = DEFAULT_BRIGHTNESS;
	m_bd->props.max_brightness = MLCD_LED_MAX;
	cur_main_lcd_level = DEFAULT_BRIGHTNESS;	

	/* save the pointer of backlight device structure for debug */
	saved_bd = m_bd;

	printk("END aat2870gu_bl probe function\n");
	return 0;
}

static int mlcd_aat2870gubl_remove(struct i2c_client *i2c_dev)
{
	m_bd->props.brightness = 0;
	m_bd->props.power = 0;

	backlight_device_unregister(m_bd);
	i2c_set_clientdata(i2c_dev, NULL);

	return 0;
}

static struct i2c_device_id mlcd_aat2870gu_idtable[] = {
        { "mlcd_aat2870gu", 0 },
};

MODULE_DEVICE_TABLE(i2c, mlcd_aat2870gu_idtable);

static struct i2c_driver mlcd_aat2870gu_driver = {
	.probe 		= mlcd_aat2870gubl_probe,
	.remove 	= mlcd_aat2870gubl_remove,
	.suspend 	= mlcd_aat2870gubl_suspend,
	.resume 	= mlcd_aat2870gubl_resume,
	.id_table 	= mlcd_aat2870gu_idtable,
	.driver = {
		.name = "mlcd_aat2870gu",
		.owner = THIS_MODULE,
	},
};

static int __init mlcd_backlight_init(void)
{
	int ret = 0;
	printk("AAT2870GU init start\n");

	ret = i2c_add_driver(&mlcd_aat2870gu_driver);

	return ret;
}

static void __exit mlcd_backlight_exit(void)
{
	i2c_del_driver(&mlcd_aat2870gu_driver);
}

module_init(mlcd_backlight_init);
module_exit(mlcd_backlight_exit);

MODULE_LICENSE("GPL");

