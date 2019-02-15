/*
 *  drivers/input/keyboard/kbd_pp2016.c
 *
 *  Copyright (c) 2008 QUALCOMM USA, INC.
 *  
 *  All source code in this file is licensed under the following license
 *  except where indicated.
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, you can find it at http://www.fsf.org
 *
 *  Driver for QWERTY keyboard with I/O communications via
 *  the I2C Interface. The keyboard hardware is a reference design supporting
 *  the standard XT/PS2 scan codes (sets 1&2).
 */
 
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <mach/msm_i2ckbd.h>
//#include <mach/gpio_keypad.h> //for keyled //keyled_pm.c
#include <linux/delay.h>
//#include <asm/arch/lprintk.h>
//#include <linux/at_kpd_eve.h> //LGE_CHANGE [antispoon@lge.com,diyu@lge.com] 2009-07-17 for AT+MOT,GKPD, FKPD
#include <mach/msm_rpcrouter.h> //LGE_CHANGE [antispoon@lge.com,diyu@lge.com] 2009-07-17 for AT+MOT,GKPD, FKPD

#if defined(CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
#include <mach/lg_diag_mtc.h>
#include <linux/kmod.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>

static struct early_suspend pp2106_suspend;
static void pp2106_early_suspend(struct early_suspend *h);
static void pp2106_late_resume(struct early_suspend *h);
#endif

#define DRIVER_VERSION "v1.0"

/* LG_FW : 2010.02.08 jinho.jang */
#if defined(CONFIG_LGE_KEYBOARD_QWERTY_PP2106)
#define GPIO_RESET_PP2106M2 108
#define GPIO_IRQ_PP2106M2 	38
#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
#define GPIO_SDA_PP2106M2 	32
#define GPIO_SCL_PP2106M2 	31
#elif defined (CONFIG_LGE_MACH_ENVT2_REVC)
#define GPIO_SDA_PP2106M2 	18
#define GPIO_SCL_PP2106M2 	17
#else
#define GPIO_SDA_PP2106M2 	18
#define GPIO_SCL_PP2106M2 	17
#endif
#define KEY_DRIVER_NAME "pp2106"

#define QWERTY_KEYPAD_ROWS  8
#define QWERTY_KEYPAD_COLUMNS  8
#else
#define GPIO_RESET_PP2106M2 108
#define GPIO_IRQ_PP2106M2 	36

#define GPIO_SDA_PP2106M2 	34
#define GPIO_SCL_PP2106M2 	35
#define KEY_DRIVER_NAME "pp2106" /*"eve_qwerty"*/

#define QWERTY_KEYPAD_ROWS  8
#define QWERTY_KEYPAD_COLUMNS  8
#endif

static const char *kbd_name  = "pp2106"; //"eve_qwerty"; //"kbd_pp2016";

MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR("diyu");
MODULE_DESCRIPTION("QWERTY keyboard driver");
MODULE_LICENSE("GPL v2");

#define QWERTY_DEBUG 0
#if QWERTY_DEBUG
#define QDBG(fmt, args...) printk(fmt, ##args)
#else
#define QDBG(fmt, args...) do {} while (0)
#endif /* QWERTY_DEBUG */

/* constants relating to events sent into the input core */
enum kbd_inevents {
	QKBD_IN_KEYPRESS        = 1,
	QKBD_IN_KEYRELEASE      = 0,
	QKBD_IN_MXKYEVTS        = 256
};

enum
{
  GPIO_LOW_VALUE  = 0,
  GPIO_HIGH_VALUE = 1
} ;

enum {
  QWERTY_START_BIT = 0,
  QWERTY_1ST_BIT7,
  QWERTY_1ST_BIT6,
  QWERTY_1ST_BIT5,
  QWERTY_1ST_BIT4,
  QWERTY_1ST_BIT3,
  QWERTY_1ST_BIT2,
  QWERTY_1ST_BIT1,
  QWERTY_1ST_BIT0,
  QWERTY_ACK_BIT,
  QWERTY_MAX_BIT
} ;

#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
static unsigned short pp2106m2_keycode[QWERTY_KEYPAD_ROWS][QWERTY_KEYPAD_COLUMNS] = 
{
    /***/  /*0*/           /*1*/        /*2*/      /*3*/       /*4*/          /*5*/        /*6*/           /*7*/
    /*0*/{KEY_BACKSPACE,     KEY_SEND,	      KEY_9,	  KEY_7,	  KEY_5,          KEY_4,        KEY_3,         KEY_MENU},
    /*1*/{KEY_MULTI,       KEY_0,        KEY_8,      KEY_6,	      KEY_T,          KEY_R,        KEY_2,         KEY_HOME},
    /*2*/{KEY_VOLUMEDOWN,  KEY_P,        KEY_I,      KEY_Y,      KEY_G,          KEY_E,        KEY_1,         KEY_BACK},    
    /*3*/{KEY_VOLUMEUP,    KEY_LEFT/*KEY_RIGHT*/,     KEY_O,	      KEY_U,      KEY_H,          KEY_D,	    KEY_Q,         KEY_A},    
    /*4*/{KEY_CAMERA,      KEY_DOWN/*KEY_UP*/,     KEY_L,      KEY_J,      KEY_B,          KEY_F,        KEY_W,         KEY_CAPSLOCK},    
    /*5*/{KEY_CAMERAFOCUS, KEY_NAVI_OK,  KEY_ENTER,	  KEY_K,      KEY_V,          KEY_C,        KEY_S,         KEY_SEARCH},    
    /*6*/{KEY_RESERVED,    KEY_UP/*KEY_DOWN*/,    KEY_WWW,	  KEY_M,    KEY_COMMA/*KEY_NEW*/,      KEY_X,        KEY_Z ,        KEY_VOICECOMMAND},    
    /*7*/{KEY_RESERVED,    KEY_RIGHT/*KEY_LEFT*/,     KEY_DOT,   KEY_N,      KEY_SPACE,   KEY_DOT_COM,    KEY_SYM/*KEY_COMMA*/,    KEY_RIGHTALT},
};
#else
static unsigned short pp2106m2_keycode[QWERTY_KEYPAD_ROWS][QWERTY_KEYPAD_COLUMNS] = 
{
    /***/  /*0*/           /*1*/        /*2*/      /*3*/       /*4*/          /*5*/        /*6*/           /*7*/
    /*0*/{KEY_DELETE,     KEY_SEND,	      KEY_9,	  KEY_7,	  KEY_5,          KEY_4,        KEY_3,         KEY_MENU},
    /*1*/{KEY_MULTI,       KEY_0,        KEY_8,      KEY_6,	      KEY_T,          KEY_R,        KEY_2,         KEY_HOME},
    /*2*/{KEY_VOLUMEDOWN,  KEY_P,        KEY_I,      KEY_Y,      KEY_G,          KEY_E,        KEY_1,         KEY_BACK},    
    /*3*/{KEY_VOLUMEUP,    KEY_RIGHT,     KEY_O,	      KEY_U,      KEY_H,          KEY_D,	    KEY_Q,         KEY_A},    
    /*4*/{KEY_CAMERA,      KEY_UP,     KEY_L,      KEY_J,      KEY_N,          KEY_F,        KEY_W,         KEY_Z},    
    /*5*/{KEY_CAMERAFOCUS, KEY_NAVI_OK,  KEY_ENTER,	  KEY_K,      KEY_B,          KEY_V,        KEY_S,         KEY_MACRO},    
    /*6*/{KEY_RESERVED,    KEY_DOWN,    KEY_WWW,	  KEY_DOT,    KEY_SPACE,      KEY_C,        KEY_X ,        KEY_CAPSLOCK},    
    /*7*/{KEY_RESERVED,    KEY_LEFT,     KEY_MAIL,   KEY_M,      KEY_RESERVED,   KEY_COMMA,    KEY_RECORD,    KEY_RIGHTALT},
};
#endif

#define QKBD_PHYSLEN 128

#define KEY_SCL_PIN		GPIO_SCL_PP2106M2
#define KEY_SDA_PIN		GPIO_SDA_PP2106M2

#define QWERTY_SDA_OUTPUT()	{ gpio_configure(KEY_SDA_PIN,GPIOF_DRIVE_OUTPUT); udelay(25);}
#define QWERTY_SDA_HIGH()	{ gpio_set_value(KEY_SDA_PIN,GPIO_HIGH_VALUE); udelay(25); }
#define QWERTY_SDA_LOW()	{ gpio_set_value(KEY_SDA_PIN,GPIO_LOW_VALUE); udelay(25); }
#define QWERTY_SDA_INPUT()	{ gpio_configure(KEY_SDA_PIN,GPIOF_INPUT); udelay(25);}
#define QWERTY_SDA_READ()	gpio_get_value(KEY_SDA_PIN)
#define QWERTY_SCL_OUTPUT()	{ gpio_configure(KEY_SCL_PIN,GPIOF_DRIVE_OUTPUT); udelay(25); }
#define QWERTY_SCL_HIGH()	{ gpio_set_value(KEY_SCL_PIN,GPIO_HIGH_VALUE); udelay(25); }
#define QWERTY_SCL_LOW()	{ gpio_set_value(KEY_SCL_PIN,GPIO_LOW_VALUE); udelay(25); }
/*
 * The qwerty_kbd_record structure consolates all the data/variables
 * specific to managing the single instance of the keyboard.
 */
struct qwerty_kbd_record {
	struct i2c_client *mykeyboard;
	//struct input_dev *i2ckbd_idev;
	//struct input_dev *kbd_idev;
	int	product_info;
	char	physinfo[QKBD_PHYSLEN];
	int     mclrpin;
	uint8_t cmd;
	uint8_t noargs;
	uint8_t cargs[2];
	uint8_t kybd_exists;
	uint8_t kybd_connected;
	uint8_t prefix;
	uint8_t pfxstk[2];
	uint8_t kcnt;
	uint8_t evcnt;
	struct delayed_work kb_cmdq;
	u32 (*xlf)(struct qwerty_kbd_record *kbdrec, s32 code,s32 *kstate, s32 *i2cerr);
	unsigned short pp2106m2_keycode[QWERTY_KEYPAD_ROWS][QWERTY_KEYPAD_COLUMNS];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend k3_early_suspend;
#endif	

	//unsigned char xltable[QKBD_IN_MXKYEVTS];
};


static struct input_dev *pp2106m2_kbd_dev;
static struct qwerty_kbd_record kbd_data;
static struct work_struct qkybd_irqwork;

static int  __init qwerty_kbd_probe(struct platform_device *pdev);	                  

#ifdef CONFIG_LGE_DIAG_TESTMODE
extern uint8_t if_condition_is_on_key_buffering;
extern uint8_t lgf_factor_key_test_rsp(char);

/* LGE_CHANGE_S [bk.shin@lge.com] 2010-07-13, testmode AIR_PLAIN_MODE_ON */
extern uint8_t if_condition_is_on_air_plain_mode;
extern void set_opertion_mode(boolean isOnline);
/* LGE_CHANGE_E [bk.shin@lge.com] 2010-07-13, testmode AIR_PLAIN_MODE_ON */
#endif

#if defined(CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
extern void ats_eta_key_logging(int scancode, unsigned char keystate,  int wait_mode);

//extern unsigned long int ats_mtc_log_mask;
//extern struct ats_mtc_key_log_type ats_mtc_key_log;
//extern void ats_mtc_send_key_log_to_eta(struct ats_mtc_key_log_type* p_ats_mtc_key_log, int wait_mode);

// LG_FW : khlee to support testmode

//extern unsigned char g_diag_mtc_check;
//extern void mtc_send_key_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log);

#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/

//LGE_CHANGE_E [antispoon@lge.com,diyu@lge.com] 2009-07-17 for AT+GKPD, FKPD
__inline void P_ChipSendACK( void )
{
    QWERTY_SDA_OUTPUT();
    QWERTY_SDA_LOW();        // ACK Bit Set
#if 1
  QWERTY_SDA_INPUT();
  QWERTY_SCL_LOW();
  
  QWERTY_SCL_HIGH();
#else
	QWERTY_SCL_LOW();        // ACK Bit Upaate Pulse
	QWERTY_SCL_HIGH();

 QWERTY_SDA_INPUT();
#endif	
	printk("diyu %s\n",__FUNCTION__);
}

/*=========================================================================
FUNCTION  P_ChipGetData
DESCRIPTION
  Function to read pysical scan code from qwerty keypad chip.
DEPENDENCIES
  this function must be called after keypad h/w init function 
RETURN VALUE
  if user pressed keypad, return TRUE.
  or return FALSE;
SIDE EFFECTS
  None
===========================================================================*/
__inline int P_ChipGetData( uint32_t *p_data  )
{
	int trigger_count;
	int first_bit_count = 0;

#if 1
	printk("diyu %s\n",__FUNCTION__);
  QWERTY_SDA_OUTPUT();
  QWERTY_SCL_HIGH();
	QWERTY_SDA_LOW();
#else
  QWERTY_SDA_OUTPUT();
  QWERTY_SCL_HIGH();
	QWERTY_SDA_LOW();
	udelay(25);
	printk("diyu %s\n",__FUNCTION__);
#endif
	printk("diyu %s\n",__FUNCTION__);

	for( trigger_count=QWERTY_START_BIT; trigger_count < QWERTY_MAX_BIT ; trigger_count++ )
	{
		if ( trigger_count == QWERTY_START_BIT )
		{
			QWERTY_SCL_LOW();
			
			// START bit
			QWERTY_SDA_INPUT();	

			QWERTY_SCL_HIGH();
		}	
		else if ( trigger_count >= QWERTY_1ST_BIT7&& trigger_count <= QWERTY_1ST_BIT0 )
		{
			QWERTY_SCL_LOW();

			// Data 1
			if( QWERTY_SDA_READ() ) 
			{
				*p_data |= 0x80 >>(first_bit_count);
			}
			first_bit_count++;

			QWERTY_SCL_HIGH();
		}
		else if ( trigger_count == QWERTY_ACK_BIT )
		{
        	P_ChipSendACK();
		} 

	}

	return 0;
}

#if defined(CONFIG_LGE_MACH_ENVT2)
extern int is_folder_open(void);
#endif
/* translate from scan code(s) to intermediate xlate-lookup code */
static inline void lge_xlscancode(void)
{
	struct input_dev *idev = pp2106m2_kbd_dev; /*kbdrec->i2ckbd_idev;*/
	u32 xlkcode = 0;
	u32 buf = 0;
	u8  keystate =0;  /*press = 1 , release = 0*/
  	u8  qwerty_key_column, qwerty_key_row;

	P_ChipGetData(&buf);
	keystate = (buf & 0x80) ? QKBD_IN_KEYRELEASE : QKBD_IN_KEYPRESS;

	qwerty_key_column = qwerty_key_row = buf;
	
	qwerty_key_row &= 0x0f;
	if(qwerty_key_row) qwerty_key_row -= 1;
	qwerty_key_column >>= 4; 
	qwerty_key_column &= 0x07;

  xlkcode = (u32)pp2106m2_keycode[qwerty_key_row][qwerty_key_column];

	printk("+++++++++ Keypad : row <0x%x>, column <0x%x>, keycode <<0x%x>>\n", 
             qwerty_key_row, qwerty_key_column, xlkcode);

	// we have a translated code to feed into the input system 
	if (xlkcode) 
	{
#if defined(CONFIG_LGE_MACH_ENVT2)
		if((is_folder_open())||(xlkcode==KEY_VOLUMEDOWN)||(xlkcode==KEY_VOLUMEUP)||(xlkcode==KEY_CAMERA)||(xlkcode==KEY_CAMERAFOCUS))
		{
				input_report_key(idev, xlkcode, keystate);
				input_sync(idev);
#ifdef CONFIG_LGE_DIAG_TESTMODE
				if(if_condition_is_on_key_buffering == 1 && keystate == QKBD_IN_KEYPRESS)
					  lgf_factor_key_test_rsp((u8)xlkcode);
			
			/* LGE_CHANGE_S [bk.shin@lge.com] 2010-07-13, testmode AIR_PLAIN_MODE_ON */
				if(if_condition_is_on_air_plain_mode == 1 && keystate == QKBD_IN_KEYPRESS)
				{
					if_condition_is_on_air_plain_mode = 0;
					set_opertion_mode(TRUE);
					printk(KERN_ERR"send online event");
				}
			/* LGE_CHANGE_E [bk.shin@lge.com] 2010-07-13, testmode AIR_PLAIN_MODE_ON */
#endif
			
#if defined(CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
					ats_eta_key_logging((int)xlkcode, (unsigned char)keystate, (int)UMH_WAIT_PROC);
#endif
			
#ifdef CONFIG_KEYLED
					msm_keyled_send(); //keyled on //code is in keyled_pm.c
#endif

		}

		
#else
		input_report_key(idev, xlkcode, keystate);
		input_sync(idev);
#ifdef CONFIG_LGE_DIAG_TESTMODE
    if(if_condition_is_on_key_buffering == 1 && keystate == QKBD_IN_KEYPRESS)
		  lgf_factor_key_test_rsp((u8)xlkcode);

/* LGE_CHANGE_S [bk.shin@lge.com] 2010-07-13, testmode AIR_PLAIN_MODE_ON */
	if(if_condition_is_on_air_plain_mode == 1 && keystate == QKBD_IN_KEYPRESS)
	{
		if_condition_is_on_air_plain_mode = 0;
		set_opertion_mode(TRUE);
		printk(KERN_ERR"send online event");
	}
/* LGE_CHANGE_E [bk.shin@lge.com] 2010-07-13, testmode AIR_PLAIN_MODE_ON */
#endif

#if defined(CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
		ats_eta_key_logging((int)xlkcode, (unsigned char)keystate, (int)UMH_WAIT_PROC);
#endif

#ifdef CONFIG_KEYLED
		msm_keyled_send(); //keyled on //code is in keyled_pm.c
#endif
#endif
	}
}

static irqreturn_t qwerty_kbd_irqhandler(int irq, void *dev_id)
{
	QDBG("diyu %s\n",__FUNCTION__);
	schedule_work(&qkybd_irqwork);
	
	return IRQ_HANDLED;
}

static int qwerty_kbd_irqsetup(int kbd_irqpin)
{
// Detect Falling edge
	int rc = request_irq(MSM_GPIO_TO_INT(kbd_irqpin), &qwerty_kbd_irqhandler,
			     IRQF_TRIGGER_FALLING, kbd_name, NULL);
	printk("diyu %s\n",__FUNCTION__);

	if (rc < 0) {
		printk(KERN_ERR
		       "Could not register for  %s interrupt "
		       "(rc = %d)\n", kbd_name, rc);
		rc = -EIO;
	}

	rc = set_irq_wake(MSM_GPIO_TO_INT(kbd_irqpin), 1);
	return rc;
}

static int qwerty_kbd_release_gpio(struct qwerty_kbd_record *kbrec)
{
	int kbd_irqpin  = kbrec->mykeyboard->irq;
	int kbd_mclrpin = kbrec->mclrpin;
	printk("diyu %s\n",__FUNCTION__);

	dev_info(&kbrec->mykeyboard->dev,
		 "releasing keyboard gpio pins %d,%d\n",
		 kbd_irqpin, kbd_mclrpin);
	gpio_free(kbd_irqpin);
	gpio_free(kbd_mclrpin);
	return 0;
}

/*
 * Configure the (2) external gpio pins connected to the keyboard.
 * interrupt(input), reset(output).
 */
static int qwerty_kbd_config_gpio(struct qwerty_kbd_record *kbrec)
{
	int rc;
	struct device *kbdev = &kbrec->mykeyboard->dev;
	//int kbd_irqpin  = kbrec->mykeyboard->irq;
	//int kbd_mclrpin = kbrec->mclrpin;



QDBG("diyu %s\n",__FUNCTION__);

#if 1
	rc = gpio_request(GPIO_IRQ_PP2106M2, "gpio_keybd_irq");
	if (rc) {
		dev_err(kbdev, "gpio_request failed on pin %d (rc=%d)\n",
			GPIO_IRQ_PP2106M2, rc);
		goto err_gpioconfig;
	}

	rc = gpio_direction_input(GPIO_IRQ_PP2106M2);
	if (rc) {
		dev_err(kbdev, "gpio_direction_input failed on "
		       "pin %d (rc=%d)\n", GPIO_IRQ_PP2106M2, rc);
		goto err_gpioconfig;
	}
#endif

  rc = gpio_request(GPIO_RESET_PP2106M2, "gpio_keybd_reset");
  if (rc) {
    dev_err(kbdev, "gpio_request failed on pin %d (rc=%d)\n",
               GPIO_RESET_PP2106M2, rc);
    goto err_gpioconfig;
  }

//	rc = gpio_direction_output(kbd_mclrpin, 1);
	rc = gpio_direction_output(GPIO_RESET_PP2106M2, 1);
	if (rc) {
		dev_err(kbdev, "gpio_direction_output failed on "
		       "pin %d (rc=%d)\n", GPIO_IRQ_PP2106M2, rc);
		goto err_gpioconfig;
	}

		rc = gpio_request(GPIO_SDA_PP2106M2, "gpio_keybd_sda");
		if (rc) {
			dev_err(kbdev, "gpio_request failed on pin %d (rc=%d)\n",
								 GPIO_SDA_PP2106M2, rc);
			goto err_gpioconfig;
		}
	
	//	rc = gpio_direction_output(kbd_mclrpin, 1);
		rc = gpio_direction_input(GPIO_SDA_PP2106M2);
		if (rc) {
			dev_err(kbdev, "gpio_direction_output failed on "
						 "pin %d (rc=%d)\n", GPIO_SDA_PP2106M2, rc);
			goto err_gpioconfig;
		}

		rc = gpio_request(GPIO_SCL_PP2106M2, "gpio_keybd_scl");
		if (rc) {
			dev_err(kbdev, "gpio_request failed on pin %d (rc=%d)\n",
								 GPIO_SCL_PP2106M2, rc);
			goto err_gpioconfig;
		}
	
	//	rc = gpio_direction_output(kbd_mclrpin, 1);
		rc = gpio_direction_output(GPIO_SCL_PP2106M2, 1);
		if (rc) {
			dev_err(kbdev, "gpio_direction_output failed on "
						 "pin %d (rc=%d)\n", GPIO_SCL_PP2106M2, rc);
			goto err_gpioconfig;
		}

	return rc;

err_gpioconfig:
	qwerty_kbd_release_gpio(kbrec);
	return rc;
}

/* use gpio output pin to toggle keyboard external reset pin */
static void qwerty_kbd_hwreset(int kbd_mclrpin)
{
    //printk("++++++++++ KEY RESET!!!!\n");
	QDBG("diyu %s\n",__FUNCTION__);

	// Reset
	gpio_set_value(kbd_mclrpin, 1);
	msleep(25);
	gpio_set_value(kbd_mclrpin, 0);
	msleep(25);
	gpio_set_value(kbd_mclrpin, 1);
	msleep(25);
}

static void qwerty_kbd_fetchkeys(struct work_struct *work)
{	QDBG("diyu %s\n",__FUNCTION__);
	lge_xlscancode();
}

struct input_dev *qwerty_get_input_dev(void)
{
	return pp2106m2_kbd_dev;
}
EXPORT_SYMBOL(qwerty_get_input_dev);

static int qwerty_kbd_remove(struct platform_device *pdev)
{
	if (pp2106m2_kbd_dev) {
		input_unregister_device(pp2106m2_kbd_dev);
		kfree(pp2106m2_kbd_dev);
	}
	return 0;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void pp2106_early_suspend(struct early_suspend * h)
{	
	//turn off
	//kr3dh_misc_data->pdata->gpio_config(1);
	qwerty_kbd_hwreset(GPIO_RESET_PP2106M2); 
	return;
}

static void pp2106_late_resume(struct early_suspend * h)
{
	//turn on
	//kr3dh_misc_data->pdata->gpio_config(0);

	return;
}
#endif

static int qwerty_kbd_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* future capability*/
	return 0;
}

static int qwerty_kbd_resume(struct platform_device *pdev)
{
	/* future capability*/
	QWERTY_SDA_HIGH();
	QWERTY_SCL_HIGH();
	//qwerty_kbd_hwreset(GPIO_RESET_PP2106M2); 
	return 0;
}

static struct platform_driver qwerty_kbd_driver = {
	.driver = {
		.name = KEY_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe	 = qwerty_kbd_probe,
	.remove	 = qwerty_kbd_remove,
	.suspend = qwerty_kbd_suspend,
	.resume  = qwerty_kbd_resume,
};

static int  __init qwerty_kbd_probe(struct platform_device *pdev)
{
//	struct msm_i2ckbd_platform_data *setup_data;
	int                              rc = 0, err;
	struct qwerty_kbd_record           *rd = &kbd_data;
	int kidx = 0;
	QDBG("diyu %s\n",__FUNCTION__);

	pp2106m2_kbd_dev = input_allocate_device();
	if (!pp2106m2_kbd_dev) {
		printk(KERN_ERR "amikbd: not enough memory for input device\n");
		err = -ENOMEM;
		//goto fail1;
	}

  pp2106m2_kbd_dev->name = KEY_DRIVER_NAME;
  pp2106m2_kbd_dev->phys = "pp2106m2/input1";
  pp2106m2_kbd_dev->id.bustype = BUS_HOST;
  pp2106m2_kbd_dev->id.vendor = 0x0001;
  pp2106m2_kbd_dev->id.product = 0x0001;
  pp2106m2_kbd_dev->id.version = 0x0100;
  pp2106m2_kbd_dev->dev.parent = &pdev->dev;
  
  pp2106m2_kbd_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);

	if(pp2106m2_kbd_dev){
    pp2106m2_kbd_dev->keycode = kbd_data.pp2106m2_keycode;
    pp2106m2_kbd_dev->keycodesize = sizeof(uint8_t);
    pp2106m2_kbd_dev->keycodemax = QWERTY_KEYPAD_ROWS*QWERTY_KEYPAD_COLUMNS;
    
    pp2106m2_kbd_dev->evbit[0] = BIT(EV_KEY);
    memset(pp2106m2_kbd_dev->keycode, 0, QKBD_IN_MXKYEVTS);
    
    pp2106m2_kbd_dev->mscbit[0] = 0;
    
    // Support all keys 
/* LG_FW : 2010.05.12 jinho.jang - VS760 Key Setting */
#if defined(CONFIG_LGE_MACH_ENVT2)
	for (kidx = 0; kidx <= KEY_SYM; kidx++)
#else
    for (kidx = 0; kidx <= KEY_DISPLAY_OFF; kidx++)
#endif
      set_bit(kidx, pp2106m2_kbd_dev->keybit);
    
    input_set_drvdata(pp2106m2_kbd_dev, rd);
#ifdef CONFIG_HAS_EARLYSUSPEND
	pp2106_suspend.suspend = pp2106_early_suspend;
	pp2106_suspend.resume = pp2106_late_resume;
	register_early_suspend(&pp2106_suspend);
#endif	
	} else {
		printk("Failed to allocate input device for %s\n",	kbd_name);
	}

	rc = qwerty_kbd_config_gpio(rd);
	// Reset
	qwerty_kbd_hwreset(GPIO_RESET_PP2106M2);

	INIT_WORK(&qkybd_irqwork, qwerty_kbd_fetchkeys);
	qwerty_kbd_irqsetup(GPIO_IRQ_PP2106M2);

	err = input_register_device(pp2106m2_kbd_dev);
	if (err)
		printk("diyu %s\n", __FUNCTION__);
//		goto fail3;
	
	return rc;
}

static int __init qwerty_kbd_init(void)
{
	memset(&kbd_data, 0, sizeof(struct qwerty_kbd_record));
	QDBG("diyu - 0418 %s\n",__FUNCTION__);

	return platform_driver_register(&qwerty_kbd_driver);
}


static void __exit qwerty_kbd_exit(void)
{
	QDBG("diyu - 0418 %s\n",__FUNCTION__);
	platform_driver_unregister(&qwerty_kbd_driver);
}

module_init(qwerty_kbd_init);
module_exit(qwerty_kbd_exit);
