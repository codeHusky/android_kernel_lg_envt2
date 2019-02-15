/* drivers/input/touchscreen/mlcd_touch_spi_amri5k.c
 *
 * Copyright (c) 2009 AVAGO Technologies.
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
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <asm/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/vreg.h> //To set vreg
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/spi/spi.h>
#include <linux/debugfs.h>

#include "mlcd_touch_amri5k.h"


/* LG_FW : 2010.03.29 jiho.jang - Dual LCD Switch */
#if defined(CONFIG_LGE_TOUCHSCREEN_AVAGO)
#include <mach/board.h>

static int is_mlcd_touch_standby_mode = 0;
#endif

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
//#include <mach/lge_diag_test.h>
#include <mach/lg_diag_mtc.h>
#include <linux/kmod.h>
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>

static struct early_suspend m_ts_early_suspend;
static void amri5k_m_ts_early_suspend(struct early_suspend *h);
static void amri5k_m_ts_late_resume(struct early_suspend *h);
#endif

#undef DEBUG_PRINT	// Define only for debug mode
//#define DEBUG_PRINT
#ifdef DEBUG_PRINT
	#define dprintk(f...)	printk(KERN_ERR "AMRI5K_main_touch " f)
#else
	#define dprintk(x...)	//do { ; } while(0)
#endif

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define M_TS_DRIVER_NAME "amri5k_m_ts"

// Kernel Mode
/* LG_FW : 2010.03.29 jinho.jang */
#if defined(CONFIG_LGE_TOUCHSCREEN_AVAGO)
#define AMRI5K_REGISTER_DUMP    // 2010/06/01 - all register dump
#define AMRI5K_FIRST_INT_DISABLE // 2010/06/01 - ignore fist interrupt after interrurpt enable
#define AMRI5K_REWIRTE_REGISTER  // 2010/06/02 - Rewirte register after touch chip on
#define AMRI5K_FOLDER_ON_OFF_SCENARIO
#define AMRI5K_ESD_RECOVERY           // 2010/06/29 - esd auto recovery code
#define AMRI5K_ESD_RECOVERY_POWER     // 2010/07/19
#define AMRI5K_EXCEPT_THUMB_MULTI_TOUCH
//#define AMRI5K_EXCEPT_THUMB_SINGLE_TOUCH     // 2010/09/23
#define AMRI5K_EXTEND_VIRTUAL_CORDINATE         // 2010/07/27
#define AMRI5K_DELAY_RELEASE_TIME
#define AMRI5K_REAL_RELEASE_CHECK       // 2010/08/18
#define AMRI5K_CALIBRATION_SCAN_FREQUENCE   //2010/09/11
#define AMRI5K_NOISE_PROCESSING         // 2010/09/15
#define AMRI5K_GAIN_PROCESSING 			// 2010/10/13
#define AMRI5K_DF_COMMAND				// 2011/02/23
#endif

#define AMRI5K_CMD_WRITE		0x80
#define AMRI5K_CMD_READ			0x00

#define MAIN_TOUCH_RESET		109

#ifdef CONFIG_FB_MSM_USE_DMAE_VSYNC
#define MLCD_AMRI5K_IRQ_GPIO	100
#else
#define MLCD_AMRI5K_IRQ_GPIO	141
#endif

#define MLCD_AMRI5K_SPI_CS_N	147
#define SLCD_AMRI5K_SPI_CS_N	20


#define EVENT_SAMPLING_RATE		0 //(HZ/30) // (100 / 30) board dependent
#define TS_POLLING_TIME 5 /* msec */

// Device dependent
#define AMRI5K_X_MAX            480
#define AMRI5K_Y_MAX            800
#define AMRI5K_FORCE_MAX        1536
#define AMRI5K_AREA_MAX	        512
#define AMRI5K_SLEEP_MODE       0
#define AMRI5K_SINGLETOUCH_MODE	1
#define AMRI5K_MULTITOUCH_MODE	2	// 4->2

#define MAX_POINT_NUM			2     // 4->2
#define TOUCH_SENSITIVITY		2     // 0:low, 1: mid, 2: high
#define AMRI5K_SROM_RETRY_COUNT		10
#ifdef AMRI5K_ESD_RECOVERY
#define AMRI5K_ESD_POLLING_TIME     5000    // 5 seconds
#endif
#ifdef AMRI5K_EXCEPT_THUMB_MULTI_TOUCH
#define THUMB_AREA_SIZE         200
#define THUMB_REJECT_AREA       120/*100*/
#endif
#ifdef AMRI5K_DELAY_RELEASE_TIME
#define TS_RELEASE_TIME         50
#endif

#ifndef ON
#define ON 1
#endif
#ifndef OFF
#define OFF 0
#endif

#define M_AMRI5K_OSC_CLK      0x03
#define M_AMRI5K_HB           0x05
#ifdef AMRI5K_FIRST_INT_DISABLE
static int m_ignore_first_intterupt_flg = FALSE;
#endif
#ifdef AMRI5K_ESD_RECOVERY
static int m_touch_data_read_status = FALSE;
#endif
/*------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------*/
// type define
struct amri5k_point{
	u8 touchid;
	u16 x;
	u16 y;
	u16 force;
	u16 area;
};

struct touch_data_format{
	u8 status;
	u8 nbpoints;
	struct amri5k_point tpd[4];  // touch point data
}; 

typedef enum events{
	KEY_RELEASE_EVENT = 0,
	KEY_DOWN_EVENT,
	KEY_MULTI_RELEASE_EVENT,
	KEY_NONE_EVENT,
}keyevent;

typedef enum touch_sensitivity{
	TOUCH_SENS_LOW,
	TOUCH_SENS_MID,
	TOUCH_SENS_HIGH,
	TOUCH_SENS_MAX
}touch_sens_level;

#ifdef AMRI5K_DELAY_RELEASE_TIME
typedef enum {
    REL_WORKQUEUE_EMPTY,
    REL_WORKQUEUE_REGISTERED
}rel_status_type;
#endif

struct AMRI5K_m_ts_device {
	struct spi_device *spi;
	struct input_dev *tdev;
	struct work_struct poswork;
	int irq;
	int touchmode;
	struct touch_data_format touchdata;
	keyevent key_eType;	
	/* lock protects the cached values */
	struct mutex		lock;
#ifdef AMRI5K_ESD_RECOVERY
	struct delayed_work esdwork;
#endif
#ifdef AMRI5K_DELAY_RELEASE_TIME
    struct delayed_work releasework;
    rel_status_type rel_status;
#endif
#ifdef AMRI5K_CALIBRATION_SCAN_FREQUENCE
    unsigned int usec_time_old;
    unsigned char data_old;
    unsigned char cal_clock[3];
    unsigned char cal_clock_sel;
#endif
};

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
extern unsigned long int ats_mtc_log_mask;
extern struct ats_mtc_key_log_type ats_mtc_key_log;
extern void ats_mtc_send_key_log_to_eta(struct ats_mtc_key_log_type* p_ats_mtc_key_log, int wait_mode);

//static char ats_mtc_key_state = 0; // 0 : release, 1 : press
//extern int led_power_onoff;

extern void ats_eta_mtc_key_logging (int pendown, int x1, int y1, int x2, int y2);
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
#ifdef AMRI5K_ESD_RECOVERY_POWER
extern int AMRI5K_Slcd_InitSromDownload(void);
extern u8 AMRI5K_s_spi_read(u8 reg);
extern u8 AMRI5K_s_spi_write(unsigned int reg, unsigned int val);
#endif

static struct input_dev *AMRI5K_m_ts_input = NULL;
static struct AMRI5K_m_ts_device AMRI5K_m_ts_dev;
static struct spi_device *AMRI5K_m_ts_spi = NULL;

/*------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------*/
// global variable
static struct workqueue_struct *tm_work_queue;
#ifdef AMRI5K_NOISE_PROCESSING
static u8 m_init_delta_done = 0;
static u8 m_usb_state = 0;
#endif

static unsigned char mlcdTouchInitReg[] = {
//[Initialization Start ] -------------------------------------------------------------------------------
    0x0F,0x7A/*0x32*//*0x7B*/,          // Auto calibration enable ?
    0x0D,0xDF,          // Set the motion pin plarity in INT_DEASSERT 0x0d	 // Change Negative Edge 0xCF -> 0xDF		
	0x07,0x06,          // Set the interrupt source in INT_MASK 0x07 // 0x02->0x06 Touch status change Bit2
    0x06,0x3F,          // Set STATUS bit clear setting in AUTO_CLEAR 0x06
	0x1E,0x04,          // Motion Report CTL // Hover touch point report off 0x00 -> 0x04
    0x47,126,
	0x40,0x25,	        // DMOD_CLOCK(Sub Touch)	
    0x0A,0x04,          // SENSE_MAP 0x0A
//[Initialization End ] --------------------------------------------------------------------------------
//[Resolution Start ] --------------------------------------------------------------------------------
// Load screen display value of row(0x41), column(0x42), height(0x43+44), witdth(0x45+46)
    0x43,0x01,	        // #define AMRI5K_X_MAX		(480)  => 0x01E0
    0x44,0xE0,
    0x45,0x03,	        // #define AMRI5K_Y_MAX		(800) => 0x0320
    0x46,0x20,
//[Resolution End] ----------------------------------------------------------------------------------
//[Adjust frame rate Start] ----------------------------------------------------------------------------
    0x23,M_AMRI5K_OSC_CLK,	        // OSC_MODE #1 internal clock speed 24M (0x03) -> 33.5M     // Test 20100818
    0x22,M_AMRI5K_HB,	        // Line Scan rate -  0A -> 05 30Hz -> 60Hz 
    0x20,0x01,	        // force run mode to 60Hz
//[Adjust frame rate End] -----------------------------------------------------------------------------
//[Thumb/Cheek Detection Start ] ----------------------------------------------------------------------
    0x1A,0x09,          // Cheek Cells - 0x06->0x05
    0x1B,0x50,	        // Cheek PCT
    0x1C,0x50,          // Cheek Total Cells
//[Thumb/Cheek Detection End ] -----------------------------------------------------------------------
//[Touch Threshold Start ] -----------------------------------------------------------------------------	
    0x67,0x03,          // Max Touch
    0x68,0x00,          // Max Touch 140->300
    0x5A,0,	            // TOUCH_1_HIGH
    0x5B,180,           // TOUCH_1_LOW
#ifndef AMRI5K_NOISE_PROCESSING
    0x5C,0,             // TOUCH_2_HIGH
    0x5D,180,	        // TOUCH_2_LOW
    0x5E,1,		        // TOUCH_3_HIGH	 
    0x5F,0x40,          // TOUCH_3_LOW
#endif
//[Touch Threshold End ] -------------------------------------------------------------------------------
//  0x75,0x09,          // NAV Filter // 0x08 -> 0x09 NV0 set. NV0: Increase panel sampling. NV1:Use higher than normal output data averaging
    0x48,0x02,          // TPOINT // 0x04 -> 0x02. Report 2 fingers only.
    0x0C,0x07,          // Flipping XY  - Main lcd touch
    0x65,0x02,	        // Touch_Delay  //2010.09.10
#ifndef AMRI5K_NOISE_PROCESSING
    0x49,0x0A/*0x1A*/,  // 2010.08.31 0x1A->0x0A
#endif
    0x27,0x02,          // Add 2010.08.31
#ifndef AMRI5K_NOISE_PROCESSING
    0x2B,0x03,          // Add 2010.08.31
#endif
    0x2F,0x00,
    0X77,0X06,

#ifdef AMRI5K_NOISE_PROCESSING
//    0x6A,0x20,          // EXCEED_THRESH_1_CNT
//    0x69,0x80,          // SUM9CELL_THRESH_1
//    0x6B,0x04,          // TOUCH_DELAY_ACTIVE_TIME
    0x50,0xF4,          // TOUCH_DELAY_2
//    0x78,0x00,          // COMMAND_CONTROL
    0x79,0x0E
#endif

    };

/*------------------------------------------------------------------------------------*/
// Fuction prototype
#ifdef AMRI5K_HW_RESET_CODE
void AMRI5K_Mlcd_Touch_HwReset(void);
#endif

int AMRI5K_Mlcd_SROM_Download(void);
int AMRI5K_Mlcd_InitSromDownload(void);

/*------------------------------------------------------------------------------------*/
// Fuction list
/* Read-only message with current device setup */
static u8 AMRI5K_m_ts_spi_read_reg(u8 reg)
{
	int ret;
	u8 val;
	u8 code = reg;

	ret = spi_write(AMRI5K_m_ts_spi,&code,1);

	if (ret < 0) {
		printk(KERN_ERR "spi read error reg:0x%X return:%d", reg, ret);
		return ret;
	}

	ret = spi_read(AMRI5K_m_ts_spi,&val, 1);
	if (ret < 0) {
		printk(KERN_ERR "spi read error reg:0x%X return:%d", reg, ret);
		return ret;
	}
	
	return val;
}

static u8 AMRI5K_m_ts_spi_write_reg(unsigned int reg, unsigned int val)
{
	int ret_val;
	unsigned char buf[2];

	/* MSB must be '1' to indicate write */
	buf[0] = reg | 0x80;
	buf[1] = val;

	ret_val = spi_write(AMRI5K_m_ts_spi,buf, 2);
	if(ret_val < 0)
	{
		printk(KERN_ERR "spi write error reg:0x%X return:%d", reg, ret_val);
	}
	return ret_val;
}

#ifdef AMRI5K_ESD_RECOVERY_POWER
u8 AMRI5K_m_spi_read(u8 reg)
{
    return AMRI5K_m_ts_spi_read_reg(reg);
}

u8 AMRI5K_m_spi_write(unsigned int reg, unsigned int val)
{
    return AMRI5K_m_ts_spi_write_reg(reg, val);
}
#endif

#ifdef AMRI5K_NOISE_PROCESSING
static void AMRI5K_m_ts_usb_setting(u8 plug)
{
    if(plug) // usb connected
    {
        dprintk("usb on... \n");
        AMRI5K_m_ts_spi_write_reg(0x5C, 0x01);  // old : 400 TOUCH_2_HIGH
        AMRI5K_m_ts_spi_write_reg(0x5D, 0x5E);  // TOUCH_2_LOW
        AMRI5K_m_ts_spi_write_reg(0x5E, 0x01);  // old : 450 TOUCH_3_HIGH
        AMRI5K_m_ts_spi_write_reg(0x5F, 0x90);  // TOUCH_3_LOW
        AMRI5K_m_ts_spi_write_reg(0x49, 0x40);
        AMRI5K_m_ts_spi_write_reg(0x78, 0x90);	// Touch Delay2
//        AMRI5K_m_ts_spi_write_reg(0x2B, 0x06);
        AMRI5K_m_ts_spi_write_reg(0x4F, 0x01);
    }
    else
    {
        dprintk("usb off... \n");    
        AMRI5K_m_ts_spi_write_reg(0x5C, 0x00);  // TOUCH_2_HIGH
        AMRI5K_m_ts_spi_write_reg(0x5D, 250);   // TOUCH_2_LOW
        AMRI5K_m_ts_spi_write_reg(0x5E, 0x01);  // TOUCH_3_HIGH
        AMRI5K_m_ts_spi_write_reg(0x5F, 0x40);   // TOUCH_3_LOW
        AMRI5K_m_ts_spi_write_reg(0x49, 0x0A);
        AMRI5K_m_ts_spi_write_reg(0x78, 0x00);
//        AMRI5K_m_ts_spi_write_reg(0x2B, 0x03);
        AMRI5K_m_ts_spi_write_reg(0x4F, 0x01);
    }
}

void AMRI5K_m_ts_init_delta(u8 plug)
{
    m_usb_state = plug;
    dprintk("usb status:%d\n", plug);
    
    if(is_mlcd_touch_standby_mode)
        m_init_delta_done = 0;
    else
    {
        AMRI5K_m_ts_usb_setting(plug);
        m_init_delta_done = 1;
    }

}
#endif

#ifdef AMRI5K_EXTEND_VIRTUAL_CORDINATE
int AMRI5K_m_ts_ExtendVirtualCordinate(struct touch_data_format *ext_touch, struct amri5k_point *old_touch)
{
    int delta_x=0, delta_y=0, temp_x=0, temp_y=0;
    int ret_value=0;

    if((old_touch[1].x == 0) || (old_touch[1].y == 0))  // previous data do not exist.
    {
        ret_value = -1; // Fail
        dprintk("[Ext] first touch\n");
    }
    else
    {
        delta_x = old_touch[0].x - old_touch[1].x;
        delta_y = old_touch[0].y - old_touch[1].y;
        dprintk("[Ext] old t[0].x:%d y:%d t[1].x:%d y:%d\n", old_touch[0].x, old_touch[0].y, old_touch[1].x, old_touch[1].y);
        delta_x >>= 1;  // x data divide 2;
        delta_y >>= 1;  // y data divide 2;
        dprintk("[Ext] d(x):%d d(y):%d\n", delta_x, delta_y);
        
        temp_x = old_touch[0].x + delta_x;
        temp_y = old_touch[0].y + delta_y;
        if((temp_x < 0 || temp_x > AMRI5K_X_MAX)
        || (temp_y < 0 || temp_y > AMRI5K_Y_MAX))
        {
            ret_value = -1; // Fail
            dprintk("[Ext] Exceed Max X:%d, Y:%d\n", temp_x, temp_y);
        }
        else
        {
            ext_touch->tpd[0].x = temp_x;
            ext_touch->tpd[0].y = temp_y;            
            ret_value = 0;  // Success
            dprintk("[Ext] Success X:%d Y:%d\n", temp_x, temp_y);
        }
    }
    
    // flush old data
    old_touch[0].x = 0;
    old_touch[0].y = 0;
    old_touch[1].x = 0;
    old_touch[1].y = 0;
    return ret_value;
}
#endif

#ifdef AMRI5K_ESD_RECOVERY_POWER
static int touch_power_set(unsigned char onoff)
{
	int ret = 0;
	struct vreg *gp3_vreg = vreg_get(0, "gp3");

	if (onoff) {
		vreg_set_level(gp3_vreg, 2100);
		vreg_enable(gp3_vreg);
		msleep(10);
	} else {
		vreg_set_level(gp3_vreg, 0);
		vreg_disable(gp3_vreg);
	}

	return ret;
}
#endif

#ifdef AMRI5K_ESD_RECOVERY
static void AMRI5K_m_ts_esd_recovery_work(struct work_struct *work)
{
   struct AMRI5K_m_ts_device *priv = container_of(work, struct AMRI5K_m_ts_device, esdwork.work);
   unsigned char regData = 0;
   int error=0;
#ifdef AMRI5K_ESD_RECOVERY_POWER
   unsigned char touch_power_reset = 0;
#endif

   if(is_folder_open() && (is_mlcd_touch_standby_mode == FALSE) && (m_touch_data_read_status == FALSE))
   {
#ifdef AMRI5K_ESD_RECOVERY_POWER
        dprintk("Sub SROM Version 0x%02X\n", AMRI5K_s_spi_read(0x01));
#endif
        regData = AMRI5K_m_ts_spi_read_reg(0x01);
        dprintk("Check Register ==> ESD Recovery Code SROM:0x%02X\n\n", regData);
        if(regData != AMRI5K_SROM_VERSION)
        {
#ifdef AMRI5K_ESD_RECOVERY_POWER
            if(regData == 0xFF) // Chip status error
            {
                dprintk("ESD error Touch Power OFF\n");
                touch_power_set(0);
                mdelay(300);
                dprintk("Touch Power ON\n");
                touch_power_set(1);
                mdelay(100);
                touch_power_reset = 1;
            }
#endif
            error = AMRI5K_Mlcd_InitSromDownload();
            if(error < 0)
            {
                printk(KERN_ERR"ESD Recovery failed!! Retry...\n");
#ifdef AMRI5K_ESD_RECOVERY_POWER
                dprintk("[Retry] Touch Power OFF\n");
                touch_power_set(0);
                mdelay(300);
                dprintk("[Retry] Touch Power ON\n");
                touch_power_set(1);
                mdelay(100);
                touch_power_reset = 1;
                if(AMRI5K_Mlcd_InitSromDownload() < 0)
                {
                    printk(KERN_ERR"[Retry] SROM Init failed!!\n");
                }
                else
                {
#ifdef AMRI5K_CALIBRATION_SCAN_FREQUENCE
                    AMRI5K_m_ts_spi_write_reg(0x71, priv->cal_clock[priv->cal_clock_sel]);
                    AMRI5K_m_ts_spi_write_reg(0x70, 0x36);
                    mdelay(10);
                    AMRI5K_m_ts_spi_write_reg(0x0F, 0x7B);
                    mdelay(10);
#endif
                    dprintk("[Retry] ESD Recovery Success!!\n");
                }
#endif
            }
            else
            {
#ifdef AMRI5K_CALIBRATION_SCAN_FREQUENCE
                AMRI5K_m_ts_spi_write_reg(0x71, priv->cal_clock[priv->cal_clock_sel]);
                AMRI5K_m_ts_spi_write_reg(0x70, 0x36);
                mdelay(10);
                AMRI5K_m_ts_spi_write_reg(0x0F, 0x7B);
                mdelay(10);
#endif
                dprintk("ESD Recovery Success!!\n");
            }
#ifdef AMRI5K_ESD_RECOVERY_POWER
            if(touch_power_reset)
            {
                error = AMRI5K_Slcd_InitSromDownload();
                if(error < 0)
                {       
                    printk(KERN_ERR"ESD Recovery failed!!\n");
                }
                else
                {
                    dprintk("Sub Touch Init!!\n");
                }
                AMRI5K_s_spi_write(0x7a,0xaa); //Standby mode
                dprintk("Sub Touch standby mode set\n");
            }
#endif
        }


    }

   //   schedule_delayed_work(&priv->esdwork, msecs_to_jiffies(AMRI5K_ESD_POLLING_TIME));
    queue_delayed_work(tm_work_queue, &priv->esdwork, msecs_to_jiffies(AMRI5K_ESD_POLLING_TIME));
}
#endif

/*------------------------------------------------------------------------------------*/
static int AMRI5K_m_ts_ReadXYPosition(struct AMRI5K_m_ts_device *priv, u16 npoint)
{
	int i;
	u8  read_buf[0x26]={0,};
	unsigned char ret_value=0;
	static u8 single_touch_keydown = FALSE;
	static u8 multi_touch_release = FALSE;
	static int touch_report_cnt = 0;	
#ifdef AMRI5K_EXCEPT_THUMB_MULTI_TOUCH
    int x1,y1;
    static unsigned char thumbs_filter_flag = OFF;
#endif
#ifdef AMRI5K_REAL_RELEASE_CHECK
	static int m_real_release_check = OFF;
#endif

	if(npoint>MAX_POINT_NUM)
	{
		dprintk("Out of range [npoint=%d]\n",npoint);
		return -1;
	}
	
#ifdef AMRI5K_ESD_RECOVERY
    m_touch_data_read_status = TRUE;
#endif

#ifdef AMRI5K_ESD_RECOVERY
    ret_value = AMRI5K_m_ts_spi_read_reg(0x01);
    if(ret_value != AMRI5K_SROM_VERSION)
    {
        dprintk("ESD Recovery SROM Version error 0x%02X\n", ret_value);
        goto err_return;
    }
#endif

    ret_value = AMRI5K_m_ts_spi_read_reg(0x02);
    //dprintk("---------------------------------------------------------\n");
    dprintk("GetPosition Read status: 0x%x\n", ret_value);

    if((ret_value & 0x12) == 0x12)
    {
#ifdef AMRI5K_DELAY_RELEASE_TIME
        if(priv->rel_status == REL_WORKQUEUE_REGISTERED)
        {
            priv->rel_status = REL_WORKQUEUE_EMPTY;
            ret_value = cancel_delayed_work_sync(&priv->releasework);
            dprintk("cancel previous workqueue. ret:%d\n",ret_value);
        }
#endif
#ifdef AMRI5K_REAL_RELEASE_CHECK
		m_real_release_check = OFF;
#endif

        // Data Read mode
        AMRI5K_m_ts_spi_write_reg(0x7b,0x01);
        // Read Touch data
        ret_value = AMRI5K_m_ts_spi_read_reg(0x01);
		priv->touchdata.nbpoints = ret_value;
        //dprintk("Touch point: %d\n", priv->touchdata.nbpoints);

        if(priv->touchdata.nbpoints >= 1)
        {
			for(i=0; i < 9; i++)
            {
                ret_value = AMRI5K_m_ts_spi_read_reg(0x02+i);
				read_buf[i] = ret_value;
            }
			
            priv->touchdata.tpd[0].touchid = read_buf[0];
            priv->touchdata.tpd[0].x    = ((u16)read_buf[1]<<8) + read_buf[2];      // touch chip x(=width,1024) position
            priv->touchdata.tpd[0].y    = ((u16)read_buf[3]<<8) + read_buf[4];      // touch chip y(=height 480) position
            priv->touchdata.tpd[0].force= ((u16)read_buf[5]<<8) + read_buf[6];
            priv->touchdata.tpd[0].area = ((u16)read_buf[7]<<8) + read_buf[8];
        }

        if(priv->touchdata.nbpoints >= 2)
        {
			for(i=0; i < 9; i++)
            {
                ret_value = AMRI5K_m_ts_spi_read_reg(0x0B+i);
				read_buf[i] = ret_value;
            }
            priv->touchdata.tpd[1].touchid = read_buf[0];
            priv->touchdata.tpd[1].x    = ((u16)read_buf[1]<<8) + read_buf[2];      // touch chip x(=width,1024) position
            priv->touchdata.tpd[1].y    = ((u16)read_buf[3]<<8) + read_buf[4];      // touch chip y(=height 480) position
            priv->touchdata.tpd[1].force= ((u16)read_buf[5]<<8) + read_buf[6];
            priv->touchdata.tpd[1].area = ((u16)read_buf[7]<<8) + read_buf[8];
        }

        // Instruction Mode
        AMRI5K_m_ts_spi_write_reg(0x7b,0x00);

#ifdef DEBUG_PRINT
        for(i=0; i<priv->touchdata.nbpoints; i++)
        {
            touch_report_cnt++;
			dprintk("TP:%d Real:0x%X Touch ID:0x%X X:%d Y:%d Force:%d Area:%d T Cnt:%d\n", 
				priv->touchdata.nbpoints, priv->touchdata.tpd[i].touchid & 0x80, priv->touchdata.tpd[i].touchid & 0x7F, priv->touchdata.tpd[i].x, priv->touchdata.tpd[i].y,
				priv->touchdata.tpd[i].force, priv->touchdata.tpd[i].area, touch_report_cnt);
        }
#endif

#ifdef AMRI5K_EXCEPT_THUMB_MULTI_TOUCH
#ifdef AMRI5K_EXCEPT_THUMB_SINGLE_TOUCH
        if(priv->touchdata.nbpoints == 1)
        {
            if((priv->touchdata.tpd[0].force >= 1100)&& (thumbs_filter_flag == OFF))
            {
                thumbs_filter_flag = ON;
                dprintk("Thumb Mode Set\n");
                
                AMRI5K_m_ts_spi_write_reg(0x58,0xB5);
                AMRI5K_m_ts_spi_write_reg(0x15,0x08);
#ifdef AMRI5K_NOISE_PROCESSING
                if(m_usb_state == 0)    // usb disconnect
                {
                    AMRI5K_m_ts_spi_write_reg(0x49,0x1E);   // 2010.08.31 0x1A->0x1E
//                    AMRI5K_m_ts_spi_write_reg(0x2B,0x04);   // 2010.08.31 add
                }                                
#else
                AMRI5K_m_ts_spi_write_reg(0x49,0x1E);   // 2010.08.31 0x1A->0x1E
//                AMRI5K_m_ts_spi_write_reg(0x2B,0x04);   // 2010.08.31 add
#endif

                dprintk("Change setting. Do not report single point \n");
                goto err_return;
            }
        }
#endif

        if(priv->touchdata.nbpoints == 2)
        {
            x1 = abs(priv->touchdata.tpd[0].x - priv->touchdata.tpd[1].x);
            y1 = abs(priv->touchdata.tpd[0].y - priv->touchdata.tpd[1].y);

            dprintk("Delta X1:%d  Y1:%d\n", x1, y1);

            if((x1 <= THUMB_AREA_SIZE) && (y1 <= THUMB_AREA_SIZE)
                && (thumbs_filter_flag == OFF))
            {
                thumbs_filter_flag = ON;            
                dprintk("Thumb Mode Set\n");
                //AMRI5K_m_ts_spi_write_reg(0x1A,0x09);     // 2010.08.31
                //AMRI5K_m_ts_spi_write_reg(0x27,0x02);     // 2010.08.31
                AMRI5K_m_ts_spi_write_reg(0x58,0xB5);
                AMRI5K_m_ts_spi_write_reg(0x15,0x08);
#ifdef AMRI5K_NOISE_PROCESSING
                if(m_usb_state == 0)    // usb disconnect
                {
                    AMRI5K_m_ts_spi_write_reg(0x49,0x1E);   // 2010.08.31 0x1A->0x1E
//                    AMRI5K_m_ts_spi_write_reg(0x2B,0x04);   // 2010.08.31 add
                }                                
#else
                AMRI5K_m_ts_spi_write_reg(0x49,0x1E);   // 2010.08.31 0x1A->0x1E
//                AMRI5K_m_ts_spi_write_reg(0x2B,0x04);   // 2010.08.31 ADD
#endif

                dprintk("Change setting. Do not report multi point \n");
                goto err_return;
            }
            else if(thumbs_filter_flag == ON)
            {
                if((x1 <= THUMB_REJECT_AREA) && (y1 <= THUMB_REJECT_AREA))
                {
                    dprintk("[Thumb Mode] Finger too close\n");
                    goto err_return;
                }
            
            }
            
        }
#endif
/*
#ifdef AMRI5K_ESD_RECOVERY
        if((priv->touchdata.tpd[0].area <= 20))
        {
            dprintk("ESD Recovery AREA error!! %d\n", priv->touchdata.tpd[0].area);
            goto err_return;
        }
#endif
*/
        // Check single touch keydown
        if(priv->touchdata.nbpoints == 1)
        {
            single_touch_keydown = TRUE;
            priv->key_eType = KEY_DOWN_EVENT;
        }
        else if(priv->touchdata.nbpoints >= 2)
    	{
            multi_touch_release = TRUE;	// Report multi touch key release event
    	}
    }
    else if((ret_value & 0x14) == 0x14)
    {
#ifdef AMRI5K_REAL_RELEASE_CHECK
		m_real_release_check = ON;
#endif

        touch_report_cnt++;
        touch_report_cnt = 0;    // Init touch count
#ifndef DEBUG_PRINT
        printk(KERN_ERR "Touch Release Status:0x%02X Cnt:%d\n",ret_value,touch_report_cnt);
#endif
#ifdef AMRI5K_DELAY_RELEASE_TIME
        priv->rel_status = REL_WORKQUEUE_REGISTERED;
        ret_value = queue_delayed_work(tm_work_queue, &priv->releasework, msecs_to_jiffies(TS_RELEASE_TIME));
        dprintk("register workqueue. ret: %d\n",ret_value);
#else    
#ifdef AMRI5K_EXCEPT_THUMB_MULTI_TOUCH
        if(thumbs_filter_flag == ON)
        {
            dprintk("Thumbs mode off\n");
            thumbs_filter_flag = OFF;
            AMRI5K_m_ts_spi_write_reg(0x58,0x35);
            AMRI5K_m_ts_spi_write_reg(0x15,0x04);
#ifdef AMRI5K_NOISE_PROCESSING
            if(m_usb_state == 0)
            {
//                AMRI5K_m_ts_spi_write_reg(0x2B,0x03);   // 2010.08.31   add
                AMRI5K_m_ts_spi_write_reg(0x49,0x0A);   // 2010.08.31   add
            }
#else
//            AMRI5K_m_ts_spi_write_reg(0x2B,0x03);   // 2010.08.31   add
            AMRI5K_m_ts_spi_write_reg(0x49,0x0A);   // 2010.08.31   add
#endif
        }
#endif
    
        if(single_touch_keydown == TRUE) 
        {
            single_touch_keydown = FALSE;
            priv->key_eType = KEY_RELEASE_EVENT;
        }
	
        if(multi_touch_release == TRUE) 
        {
            multi_touch_release = FALSE;
            priv->key_eType = KEY_MULTI_RELEASE_EVENT;
        }
#endif
    }
#ifdef AMRI5K_DELAY_RELEASE_TIME
    else
    {
#ifdef AMRI5K_REAL_RELEASE_CHECK
		if (m_real_release_check == ON)
		{
			m_real_release_check = OFF;
			dprintk("m_real_release !!\n");
		}
		else
		{
			dprintk("non_m_real_release !!\n");
			goto err_return;
		}
#endif

#ifdef AMRI5K_EXCEPT_THUMB_MULTI_TOUCH
        if(thumbs_filter_flag == ON)
        {
            dprintk("Thumbs mode off\n");
            thumbs_filter_flag = OFF;
            AMRI5K_m_ts_spi_write_reg(0x58,0x35);
            AMRI5K_m_ts_spi_write_reg(0x15,0x04);
#ifdef AMRI5K_NOISE_PROCESSING
            if(m_usb_state == 0)
            {
//                AMRI5K_m_ts_spi_write_reg(0x2B,0x03);   // 2010.08.31   add
                AMRI5K_m_ts_spi_write_reg(0x49,0x0A);   // 2010.08.31   add
            }
#else
//            AMRI5K_m_ts_spi_write_reg(0x2B,0x03);   // 2010.08.31   add
            AMRI5K_m_ts_spi_write_reg(0x49,0x0A);   // 2010.08.31   add
#endif
        }
#endif
        if(single_touch_keydown == TRUE) 
        {
            single_touch_keydown = FALSE;
            priv->key_eType = KEY_RELEASE_EVENT;
        }
	
        if(multi_touch_release == TRUE) 
        {
            multi_touch_release = FALSE;
            priv->key_eType = KEY_MULTI_RELEASE_EVENT;
        }
    }
#endif

#ifdef AMRI5K_ESD_RECOVERY
    m_touch_data_read_status = FALSE;
#endif
#ifdef AMRI5K_DELAY_RELEASE_TIME
    return 0;

err_return:
#ifdef AMRI5K_ESD_RECOVERY
    m_touch_data_read_status = FALSE;
#endif
    priv->touchdata.nbpoints = 0;
    return -1;
#else
	return 0;
#endif
}

static void AMRI5K_m_ts_read_touch_event_work(struct work_struct *work)
{
    struct AMRI5K_m_ts_device *priv = container_of(work, struct AMRI5K_m_ts_device, poswork);
	struct touch_data_format *tdata = &priv->touchdata;
	u8 touchid = 0;
	int tCnt = 0;
    int error = 0;
#ifdef AMRI5K_EXTEND_VIRTUAL_CORDINATE
    static struct amri5k_point old_touch[2];
    struct touch_data_format ext_touch;
#endif

#ifdef AMRI5K_FIRST_INT_DISABLE
    if(m_ignore_first_intterupt_flg == TRUE)
    {
        m_ignore_first_intterupt_flg = FALSE;
        dprintk("IGNORE FIRST CALL \n");
        goto err_return;//return;
    }
#endif
    
#ifdef CONFIG_LGE_MULTITOUCH
	error = AMRI5K_m_ts_ReadXYPosition(priv, priv->touchmode);
	if(error < 0)
	{
        dprintk("Touch Read error!! %d\n", error);
        goto err_return; //return;
	}
#else
	AMRI5K_m_ts_ReadXYPosition(priv->client, 1);
#endif
	if(tdata->nbpoints > 0)
	{
		if(tdata->nbpoints >=2) //multi touch
		{
			while(tCnt < tdata->nbpoints)
			{
				touchid = tdata->tpd[tCnt].touchid & 0x7F;   // Bit[7] - Touch point state, 1-real touch point, 0-hovering touch point
				if (tdata->tpd[tCnt].x >= 0 && tdata->tpd[tCnt].x <= AMRI5K_X_MAX &&
					tdata->tpd[tCnt].y >= 0 && tdata->tpd[tCnt].y <= AMRI5K_Y_MAX)
				{
					input_report_abs(priv->tdev, ABS_MT_TRACKING_ID, touchid);
					input_report_abs(priv->tdev, ABS_MT_POSITION_X, tdata->tpd[tCnt].x);
					input_report_abs(priv->tdev, ABS_MT_POSITION_Y, tdata->tpd[tCnt].y);
					input_report_abs(priv->tdev, ABS_MT_TOUCH_MAJOR, tdata->tpd[tCnt].force);
					input_report_abs(priv->tdev, ABS_MT_WIDTH_MAJOR, tdata->tpd[tCnt].area);
					input_mt_sync(priv->tdev);
					dprintk("MULTI Touch Press : %d\n", tCnt);

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
					if((ats_mtc_log_mask&0x00000002) != 0) // LOGITEM_TOUCHPAD
					{
						ats_eta_mtc_key_logging(KEY_DOWN_EVENT, tdata->tpd[tCnt].x, tdata->tpd[tCnt].y, -1, -1);
					}
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/

				}
				tCnt++;
			}

			input_sync(priv->tdev);

			if(tdata->nbpoints > 1)
			{
				tdata->nbpoints = 0;   // init fot next step
				dprintk("MULTI Touch return\n");   // test
				goto err_return; //return;
			}
		}
		else //single touch
		{
			if(priv->key_eType == KEY_DOWN_EVENT)
			{				
				//priv->key_eType = KEY_NONE_EVENT;   // Initial event

				touchid = tdata->tpd[0].touchid & 0x7F;				
				if (tdata->tpd[0].x >= 0 && tdata->tpd[0].x <= AMRI5K_X_MAX &&
					tdata->tpd[0].y >= 0 && tdata->tpd[0].y <= AMRI5K_Y_MAX)
				{					
					input_report_abs(priv->tdev, ABS_MT_TRACKING_ID, touchid);
					input_report_abs(priv->tdev, ABS_MT_POSITION_X, tdata->tpd[0].x);
					input_report_abs(priv->tdev, ABS_MT_POSITION_Y, tdata->tpd[0].y);
					input_report_abs(priv->tdev, ABS_MT_TOUCH_MAJOR, tdata->tpd[0].force);
					input_report_abs(priv->tdev, ABS_MT_WIDTH_MAJOR, tdata->tpd[0].area);
					input_mt_sync(priv->tdev);
					input_sync(priv->tdev);
					dprintk("Single Touch Press \n");

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
					if((ats_mtc_log_mask&0x00000002) != 0) // LOGITEM_TOUCHPAD
					{
						ats_eta_mtc_key_logging(KEY_DOWN_EVENT, tdata->tpd[0].x, tdata->tpd[0].y, -1, -1);
					}
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
#ifdef AMRI5K_EXTEND_VIRTUAL_CORDINATE
                    old_touch[1] = old_touch[0];    // old backup
                    old_touch[0] = tdata->tpd[0];
#endif

				}
			}
		}
	}

	if(priv->key_eType == KEY_DOWN_EVENT)
	{
	    priv->key_eType = KEY_NONE_EVENT;   // Initial event
	}
	else if(priv->key_eType == KEY_RELEASE_EVENT)
	{
		priv->key_eType = KEY_NONE_EVENT;   // Initial event
#ifdef AMRI5K_EXTEND_VIRTUAL_CORDINATE
        ext_touch = *tdata;   // copy default data
        error = AMRI5K_m_ts_ExtendVirtualCordinate(&ext_touch, old_touch);

        if(error == 0)
        {
            touchid = ext_touch.tpd[0].touchid & 0x7F;
            input_report_abs(priv->tdev, ABS_MT_TRACKING_ID, touchid);
            input_report_abs(priv->tdev, ABS_MT_POSITION_X, ext_touch.tpd[0].x);
            input_report_abs(priv->tdev, ABS_MT_POSITION_Y, ext_touch.tpd[0].y);
            input_report_abs(priv->tdev, ABS_MT_TOUCH_MAJOR, 1);
            input_report_abs(priv->tdev, ABS_MT_WIDTH_MAJOR, ext_touch.tpd[0].area);
            input_mt_sync(priv->tdev);
            input_sync(priv->tdev);

            dprintk("Real:0x%X Touch ID:0x%X X:%d : Y:%d Force:%d Area:%d\n", 
                ext_touch.tpd[0].touchid & 0x80, ext_touch.tpd[0].touchid & 0x7F, ext_touch.tpd[0].x, ext_touch.tpd[0].y,
                ext_touch.tpd[0].force, ext_touch.tpd[0].area);

            dprintk("Extend Single Touch Press \n\n");

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
            if((ats_mtc_log_mask&0x00000002) != 0) // LOGITEM_TOUCHPAD
            {
                ats_eta_mtc_key_logging(KEY_DOWN_EVENT, ext_touch.tpd[0].x, ext_touch.tpd[0].y, -1, -1);
            }
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/

            // Rlesase
    		touchid = ext_touch.tpd[0].touchid & 0x7F;
    		input_report_abs(priv->tdev, ABS_MT_TRACKING_ID, touchid);
    		input_report_abs(priv->tdev, ABS_MT_POSITION_X, ext_touch.tpd[0].x);
    		input_report_abs(priv->tdev, ABS_MT_POSITION_Y, ext_touch.tpd[0].y);
    		input_report_abs(priv->tdev, ABS_MT_TOUCH_MAJOR, 0);
    		input_report_abs(priv->tdev, ABS_MT_WIDTH_MAJOR, ext_touch.tpd[0].area);
    		input_mt_sync(priv->tdev);
    		input_sync(priv->tdev);
            dprintk("Touch ID:0x%X X:%d : Y:%d Force:%d Area:%d\n", touchid, ext_touch.tpd[0].x, ext_touch.tpd[0].y, 0, ext_touch.tpd[0].area);
    		dprintk("Extend Single Key Release\n");

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
    		if((ats_mtc_log_mask&0x00000002) != 0) // LOGITEM_TOUCHPAD
    		{
    			ats_eta_mtc_key_logging(KEY_RELEASE_EVENT, ext_touch.tpd[0].x, ext_touch.tpd[0].y, -1, -1);
    		}
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
        }
        else
#endif /* AMRI5K_EXTEND_VIRTUAL_CORDINATE */
        {
            touchid = tdata->tpd[0].touchid & 0x7F;
            input_report_abs(priv->tdev, ABS_MT_TRACKING_ID, touchid);
            input_report_abs(priv->tdev, ABS_MT_POSITION_X, tdata->tpd[0].x);
            input_report_abs(priv->tdev, ABS_MT_POSITION_Y, tdata->tpd[0].y);
            input_report_abs(priv->tdev, ABS_MT_TOUCH_MAJOR, 0);
            input_report_abs(priv->tdev, ABS_MT_WIDTH_MAJOR, tdata->tpd[0].area);
            input_mt_sync(priv->tdev);
            input_sync(priv->tdev);
            dprintk("Touch ID:0x%X X:%d : Y:%d Force:%d Area:%d\n", touchid, tdata->tpd[0].x, tdata->tpd[0].y, 0, tdata->tpd[0].area);
            dprintk("Single Key Release\n");
    
#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
            if((ats_mtc_log_mask&0x00000002) != 0) // LOGITEM_TOUCHPAD
            {
                ats_eta_mtc_key_logging(KEY_RELEASE_EVENT, tdata->tpd[0].x, tdata->tpd[0].y, -1, -1);
            }
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
        }
	}
	else if(priv->key_eType == KEY_MULTI_RELEASE_EVENT)
	{
		priv->key_eType = KEY_NONE_EVENT;   // Initial event
        for(tCnt =0; tCnt < 2; tCnt++)
        {
            touchid  = tdata->tpd[tCnt].touchid & 0x7F;
            input_report_abs(priv->tdev, ABS_MT_TRACKING_ID, touchid);
            input_report_abs(priv->tdev, ABS_MT_POSITION_X, tdata->tpd[tCnt].x);
            input_report_abs(priv->tdev, ABS_MT_POSITION_Y, tdata->tpd[tCnt].y);
            input_report_abs(priv->tdev, ABS_MT_TOUCH_MAJOR, 0);
            input_report_abs(priv->tdev, ABS_MT_WIDTH_MAJOR, tdata->tpd[tCnt].area);
            input_mt_sync(priv->tdev);            
            dprintk("[all finger release] toudh id: %d x:%d y:%d area:%d\n", touchid, tdata->tpd[tCnt].x, tdata->tpd[tCnt].y, tdata->tpd[tCnt].area);            

#if defined (CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING)
            if((ats_mtc_log_mask&0x00000002) != 0) // LOGITEM_TOUCHPAD
            {
            	ats_eta_mtc_key_logging(KEY_RELEASE_EVENT, tdata->tpd[tCnt].x, tdata->tpd[tCnt].y, -1, -1);
            }
#endif /*CONFIG_LGE_ATS_ETA_MTC_KEY_LOGGING*/
        }
		input_sync(priv->tdev);            
        dprintk("Multi Key Release\n");
	}
	else
	{
		dprintk("Single Ignore Event %d\n",priv->key_eType);
	}

	tdata->nbpoints = 0;   // init fot next step
	
err_return:
    return;
}


#ifdef AMRI5K_DELAY_RELEASE_TIME
static void AMRI5K_m_ts_release_touch_event_work(struct work_struct *work)
{
    struct AMRI5K_m_ts_device *priv = container_of(work, struct AMRI5K_m_ts_device, releasework.work);
    dprintk("run release workqueue. status:%d\n", priv->rel_status);
    priv->rel_status = REL_WORKQUEUE_EMPTY;
    AMRI5K_m_ts_read_touch_event_work(&priv->poswork);
}
#endif

/* LG_FW : 2010.04.02 jinho.jang - srom init at osbl */
void AMRI5K_Mlcd_Touch_HwReset(void)
{
	/* Sub LCD Touchscreen */
	gpio_configure(MAIN_TOUCH_RESET, GPIOF_DRIVE_OUTPUT); 
	gpio_set_value(MAIN_TOUCH_RESET, 1);
	mdelay(10);
	gpio_set_value(MAIN_TOUCH_RESET, 0);
	mdelay(10);
	gpio_set_value(MAIN_TOUCH_RESET, 1);
	mdelay(10);
}

int AMRI5K_Mlcd_SROM_Download(void)
{
	unsigned char ret_value=0;
	int cnt=0;
	
	dprintk("SROM Download Start\n");
	// Disable Watchdog
	AMRI5K_m_ts_spi_write_reg(0x7d,0xad);
	// Clear BOOT_STATE
	AMRI5K_m_ts_spi_write_reg(0x0e,0x00);
	// Enable download 
	AMRI5K_m_ts_spi_write_reg(0x08,0x2b);
	// Download patchcode
	cnt = 0;
	do{
		AMRI5K_m_ts_spi_write_reg(0x09,amri5k_mlcd_srom_code[cnt]);
		udelay(15); 	// Device need to wait time during patch download.
		cnt++;
	}while(cnt < sizeof(amri5k_mlcd_srom_code));
	dprintk("SROM code length: %d\n", cnt); 
	mdelay(30); // Wait until boot up

	// Check download successful?
	cnt = 0;
	do{
		ret_value = AMRI5K_m_ts_spi_read_reg(0x0e);
		dprintk("SROM Download status:0x%02X\n", ret_value);
		if((ret_value & 0x1F) == 0x03)
		{
			break;
		}
		else if((ret_value & 0x04) == 0x04)    // Bit 2 High: Download failed
        {
            printk(KERN_ERR "%s SROM Download status error:0x%02X\n",M_TS_DRIVER_NAME, ret_value);
            return -1;
        }

		if(++cnt > 100) // Check time out
		{
			printk(KERN_ERR "%s SROM Download time out: %d\n",M_TS_DRIVER_NAME, ret_value);
			return -1;
		}
		mdelay(10); 	
	}while(1);	

	// Enable watchdog
	AMRI5K_m_ts_spi_write_reg(0x7d,0x00);
	dprintk("SROM Download Complete!!\n");

	// Read SROM Version == F/W Version
	ret_value = AMRI5K_m_ts_spi_read_reg(0x01);
	dprintk("F/W(SROM) version:0x%02X\n",ret_value);

	if(ret_value != AMRI5K_SROM_VERSION)
	{
		printk(KERN_ERR "%s Miss match F/W(SROM) version:0x%02X\n",M_TS_DRIVER_NAME, ret_value);
		return -1;
	}
	
	return 0;	// Download success
}


int AMRI5K_Mlcd_InitSromDownload(void)
{
    int ret_value=0;
    int bootCnt=0, retryCnt=0;	
	
	mutex_lock(&AMRI5K_m_ts_dev.lock);

    do
    {
        AMRI5K_Mlcd_Touch_HwReset();

        // Software Reset
        AMRI5K_m_ts_spi_write_reg(0x7a,0xaa);
        AMRI5K_m_ts_spi_write_reg(0x7a,0xbb);
        mdelay(10);

        // Read Device ID 
        ret_value = AMRI5K_m_ts_spi_read_reg(0x00);
    	dprintk("Device ID:0x%X\n",ret_value);

    	// Wait for boot complete
    	bootCnt = 0;
    	do{
    	    ret_value = AMRI5K_m_ts_spi_read_reg(0x0e);
    	    dprintk("Wait for boot complete:0x%x bootCnt:%d \n", ret_value, bootCnt);
    	}while((ret_value != 0x01) && (++bootCnt<20));

        ret_value = AMRI5K_Mlcd_SROM_Download();
        if(ret_value < 0)
        {
            printk(KERN_ERR "AMRI5K SROM Download fail. Retry Cnt:%d\n",retryCnt);
        }
    }while((ret_value < 0) && (++retryCnt < AMRI5K_SROM_RETRY_COUNT));   //Retry download

    if(retryCnt >= AMRI5K_SROM_RETRY_COUNT)
    {
        printk(KERN_ERR "AMRI5K SROM fail.\n");
        return -1;
    }

	//init register
    dprintk("Register Initialize Start \n");

    for(bootCnt=0; bootCnt < sizeof(mlcdTouchInitReg); bootCnt += 2)
    {
        AMRI5K_m_ts_spi_write_reg(mlcdTouchInitReg[bootCnt],mlcdTouchInitReg[bootCnt+1]);
        dprintk("REG: 0x%02X 0x%02X\n",mlcdTouchInitReg[bootCnt],mlcdTouchInitReg[bootCnt+1]);
    }

#ifdef AMRI5K_NOISE_PROCESSING
    AMRI5K_m_ts_usb_setting(m_usb_state);
#endif

	ret_value = AMRI5K_m_ts_spi_read_reg(0x02);	
    dprintk("Register Initialize End Status:0x%X\n",ret_value);

	mutex_unlock(&AMRI5K_m_ts_dev.lock);
	
	return 0;
	
}

void Send_Mlcd_Touch( unsigned int x, unsigned int y)
{
#ifdef CONFIG_LGE_MULTITOUCH
  input_report_abs(AMRI5K_m_ts_input, ABS_MT_TOUCH_MAJOR, 1);
  input_report_abs(AMRI5K_m_ts_input, ABS_MT_POSITION_X, x);
  input_report_abs(AMRI5K_m_ts_input, ABS_MT_POSITION_Y, y);
  input_mt_sync(AMRI5K_m_ts_input);
  input_sync(AMRI5K_m_ts_input);

  input_report_abs(AMRI5K_m_ts_input, ABS_MT_TOUCH_MAJOR, 0);
  input_report_abs(AMRI5K_m_ts_input, ABS_MT_POSITION_X, x);
  input_report_abs(AMRI5K_m_ts_input, ABS_MT_POSITION_Y, y);
  input_mt_sync(AMRI5K_m_ts_input);
  input_sync(AMRI5K_m_ts_input);
#else
  input_report_abs(AMRI5K_m_ts_input, ABS_X, x);
  input_report_abs(AMRI5K_m_ts_input, ABS_Y, y);  
  input_report_key(AMRI5K_m_ts_input, BTN_TOUCH, 0);
  input_sync(AMRI5K_m_ts_input);
#endif
}

EXPORT_SYMBOL(Send_Mlcd_Touch);

static irqreturn_t AMRI5K_m_ts_isr(int irq, void *dev_id)
{
	struct AMRI5K_m_ts_device *priv = dev_id;
	
#ifdef TOUCH_DELY_TEST
	schedule_delayed_work(&priv->poswork, msecs_to_jiffies(TS_POLLING_TIME));
#else
	queue_work(tm_work_queue, &priv->poswork);
#endif

	return IRQ_HANDLED;
}

static int AMRI5K_m_ts_open(struct input_dev *dev)
{	
	return 0;
}

static void AMRI5K_m_ts_close(struct input_dev *dev)
{
	return;
}

/* LG_FW : 2010.03.29 jinho.jang */	
#if defined(CONFIG_LGE_TOUCHSCREEN_AVAGO)
void mlcd_touch_on(void)
{
#ifdef AMRI5K_REWIRTE_REGISTER
    int regCnt = 0;
    unsigned char regData = 0;
#endif
#ifdef AMRI5K_ESD_RECOVERY_POWER
    int error=0;
#endif
#ifdef AMRI5K_DF_COMMAND
	int i=0,data1=0;
#endif

	if(&AMRI5K_m_ts_dev == NULL || AMRI5K_m_ts_input == NULL)
		return;

	if(!is_folder_open())
		return;
	
	if(is_mlcd_touch_standby_mode)
	{
		mutex_lock(&AMRI5K_m_ts_dev.lock);				
#ifdef AMRI5K_FOLDER_ON_OFF_SCENARIO
        dprintk("100ms delay\n");
        mdelay(20);	
        dprintk("touch run mode\n");
#endif
        AMRI5K_m_ts_spi_write_reg(0x7a,0xdd);
        mdelay(20); //mdelay(10);
#ifdef AMRI5K_ESD_RECOVERY_POWER                
        regData = AMRI5K_m_ts_spi_read_reg(0x01);
        if(regData != AMRI5K_SROM_VERSION)
        {
            mutex_unlock(&AMRI5K_m_ts_dev.lock);                
            error = AMRI5K_Mlcd_InitSromDownload();
            mutex_lock(&AMRI5K_m_ts_dev.lock);  

            if(error < 0)
            {       
                printk(KERN_ERR"ESD Recovery failed!!\n");
            }
            else
            {
                dprintk("ESD Recovery Success!!\n");
            }
        }
        else
#endif
        {
#ifdef AMRI5K_REWIRTE_REGISTER
            for(regCnt=0; regCnt < sizeof(mlcdTouchInitReg); regCnt+=2)
            {
                if(mlcdTouchInitReg[regCnt] == 0x0F)    // Pass 0x0F register
                {
                    continue;
                }
                
                regData = AMRI5K_m_ts_spi_read_reg(mlcdTouchInitReg[regCnt]);
                if(regData != mlcdTouchInitReg[regCnt+1])
                {   
                    AMRI5K_m_ts_spi_write_reg(mlcdTouchInitReg[regCnt], mlcdTouchInitReg[regCnt+1]);
                    dprintk("Rewrite REG: 0x%02X 0x%02X Real: 0x%02X\n",mlcdTouchInitReg[regCnt],mlcdTouchInitReg[regCnt+1], regData);
                }
            }
#endif
        }

#ifdef AMRI5K_CALIBRATION_SCAN_FREQUENCE
        AMRI5K_m_ts_dev.cal_clock_sel = 0;
        dprintk( "0x71 = 0x%x\n", AMRI5K_m_ts_dev.cal_clock[AMRI5K_m_ts_dev.cal_clock_sel]);
        AMRI5K_m_ts_spi_write_reg(0x71, AMRI5K_m_ts_dev.cal_clock[AMRI5K_m_ts_dev.cal_clock_sel]);
        AMRI5K_m_ts_spi_write_reg(0x70, 0x36);
        mdelay(10);
        AMRI5K_m_ts_spi_write_reg(0x0F, 0x7B);
        mdelay(30);
#endif

#ifdef AMRI5K_GAIN_PROCESSING
		regData = AMRI5K_m_ts_spi_read_reg(0x39);
		if (regData == 0x00)
		{
			mdelay(50);
			AMRI5K_m_ts_spi_write_reg(0x39,0x15);
			AMRI5K_m_ts_spi_write_reg(0x75,0x10);
			mdelay(10);
		}
#endif

#ifdef AMRI5K_DF_COMMAND
		while(1)
		{
			data1 = AMRI5K_m_ts_spi_read_reg( 0x0F);
			if (  ( (data1&0x01) == 0x00)|(i>100) )
				{
					break;
				}
			mdelay(20);
			dprintk( "0x0F = 0x%x\n", data1);
			i++;
		}

		AMRI5K_m_ts_spi_write_reg(0x71, 0x60);
		AMRI5K_m_ts_spi_write_reg(0x72, 0x08);
		AMRI5K_m_ts_spi_write_reg(0x70, 0x33);
		mdelay(30);
#endif


#ifdef AMRI5K_NOISE_PROCESSING
        if(m_init_delta_done == 0)
        {
            AMRI5K_m_ts_usb_setting(m_usb_state);
        }
#endif

#ifdef AMRI5K_FIRST_INT_DISABLE
        m_ignore_first_intterupt_flg = TRUE;
#endif    
        enable_irq(AMRI5K_m_ts_dev.irq);
        mutex_unlock(&AMRI5K_m_ts_dev.lock);		
        is_mlcd_touch_standby_mode = FALSE;
        dprintk("mlcd resume & int enable\n");	
	}	
		
	dprintk("mlcd touch on \n");
}

#ifdef AMRI5K_REGISTER_DUMP
static void mlcd_touch_register_dump(void)
{
    int addrCnt = 0;
    unsigned char retValue=0;

    dprintk("\n                  ---------------- Main Register Dump ------------------");

    dprintk("\n                  Addr | 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
    dprintk("\n                  ------------------------------------------------------");
    dprintk("\n                  0x%02X |",0x00);
  
    for(addrCnt=0; addrCnt < 0x80; addrCnt++)
    {
        retValue = AMRI5K_m_ts_spi_read_reg(addrCnt);

        if((addrCnt != 0) && ((addrCnt % 16) == 0))
        {
            dprintk("\n                  0x%02X |",addrCnt);
        }
        dprintk(" %02X",retValue);
    }
    dprintk("\n                  ------------------------------------------------------\n\n");

}
#endif

void mlcd_touch_off(void)
{
	if(&AMRI5K_m_ts_dev == NULL || AMRI5K_m_ts_input == NULL)
		return;	

	if(!is_mlcd_touch_standby_mode)
	{	
		disable_irq(AMRI5K_m_ts_dev.irq);

		mutex_lock(&AMRI5K_m_ts_dev.lock);		

#ifdef AMRI5K_REGISTER_DUMP
        mlcd_touch_register_dump();
#endif
        is_mlcd_touch_standby_mode = TRUE;		
		AMRI5K_m_ts_spi_write_reg(0x7a,0xaa); //Standby mode

#ifdef AMRI5K_FOLDER_ON_OFF_SCENARIO
        dprintk("touch sleep mode\n");
        mdelay(10);
        dprintk("100ms delay\n");
#endif
		mutex_unlock(&AMRI5K_m_ts_dev.lock);
		
        dprintk("mlcd shutdown & int diable\n");
	}
		
	dprintk("mlcd touch off \n");
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void amri5k_m_ts_early_suspend(struct early_suspend * h)
{	
	return;
}

static void amri5k_m_ts_late_resume(struct early_suspend * h)
{		
	return;
}
#endif

#ifdef CONFIG_LGE_TOUCH_DRIVER_VERSION
void amri5k_m_ts_version_read(unsigned char* fw_ver, unsigned char* hw_ver)
{
	unsigned char data;
    struct AMRI5K_m_ts_device *dev = NULL;

	dev = &AMRI5K_m_ts_dev;	

	data = AMRI5K_m_ts_spi_read_reg(0x01);
	printk(KERN_INFO "MCS6000 F/W Version [0x%x]\n", data);
    *fw_ver = data;

	data = AMRI5K_m_ts_spi_read_reg(0x00);
	printk(KERN_INFO "MCS6000 H/W Revision [0x%x]\n", data);
    *hw_ver = data;
}

static ssize_t amri5k_m_ts_version(struct device *dev, struct device_attribute *attr,
	char *buf)
{
  int r;
  unsigned char hw_ver, fw_ver;
 
  amri5k_m_ts_version_read(&fw_ver, &hw_ver);
  r = sprintf(buf,"HW:%02x FW:%02x",hw_ver, AMRI5K_SROM_VERSION);
 
  return r;
}

static DEVICE_ATTR(version, S_IRUGO /*| S_IWUSR*/,amri5k_m_ts_version, NULL);

int amri5k_m_ts_create_file(struct input_dev *pdev)
{
  int ret;

	ret = device_create_file(&pdev->dev, &dev_attr_version);
	if (ret) {
		printk( KERN_DEBUG "LG_FW : dev_attr_version create fail\n");
		device_remove_file(&pdev->dev, &dev_attr_version);
		return ret;
	}
	
  return ret;
}

int amri5k_m_ts_remove_file(struct input_dev *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_version);
  return 0;
}
#endif

static int AMRI5K_m_ts_probe(struct spi_device *spi)
{
	struct AMRI5K_m_ts_device *ts_dev;
	int error;
	int irq;	
#ifdef AMRI5K_CALIBRATION_SCAN_FREQUENCE
    uint8_t data1, data_old;
    uint16_t delta;
    unsigned int osc_temp,usec_time,msec_time;
    int m_clk;
    struct timeval tv;
#endif
#ifdef AMRI5K_DF_COMMAND
	int i=0;
#endif

	ts_dev = &AMRI5K_m_ts_dev;	
	AMRI5K_m_ts_spi = spi;
	
	dev_set_drvdata(&spi->dev, ts_dev);

	AMRI5K_m_ts_input = input_allocate_device();
	if (!AMRI5K_m_ts_input) {
		error = -ENOMEM;
		goto err1;
	}

	mutex_init(&ts_dev->lock);

	AMRI5K_m_ts_input->name = M_TS_DRIVER_NAME;
	AMRI5K_m_ts_input->open = AMRI5K_m_ts_open;
	AMRI5K_m_ts_input->close = AMRI5K_m_ts_close;		
	
	set_bit(EV_SYN, 	 AMRI5K_m_ts_input->evbit);
	set_bit(EV_KEY, 	 AMRI5K_m_ts_input->evbit);
	set_bit(EV_ABS, 	 AMRI5K_m_ts_input->evbit);
	
#ifdef CONFIG_LGE_AUDIO_HAPTIC_TOUCH_SOFT_KEY						
    set_bit(TOUCH_BACK, AMRI5K_m_ts_input->keybit);
	set_bit(TOUCH_SEARCH, AMRI5K_m_ts_input->keybit);
#else
    set_bit(KEY_BACK, AMRI5K_m_ts_input->keybit);
    set_bit(KEY_SEARCH, AMRI5K_m_ts_input->keybit);
#endif	

#ifdef CONFIG_LGE_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, AMRI5K_m_ts_input->absbit);
	input_set_abs_params(AMRI5K_m_ts_input, ABS_MT_POSITION_X, 0, AMRI5K_X_MAX, 0, 0);
	input_set_abs_params(AMRI5K_m_ts_input, ABS_MT_POSITION_Y, 0, AMRI5K_Y_MAX, 0, 0);
	input_set_abs_params(AMRI5K_m_ts_input, ABS_MT_TRACKING_ID, 0, 5, 0, 0);
	input_set_abs_params(AMRI5K_m_ts_input, ABS_MT_TOUCH_MAJOR, 0, AMRI5K_FORCE_MAX, 0, 0);
	input_set_abs_params(AMRI5K_m_ts_input, ABS_MT_WIDTH_MAJOR, 0, AMRI5K_AREA_MAX, 0, 0);	
#else
	set_bit(BTN_TOUCH, AMRI5K_m_ts_input->keybit);
	input_set_abs_params(AMRI5K_m_ts_input, ABS_X, 0, AMRI5K_X_MAX, 0, 0);
	input_set_abs_params(AMRI5K_m_ts_input, ABS_Y, 0, AMRI5K_Y_MAX, 0, 0);
#endif	
	error = input_register_device(AMRI5K_m_ts_input);
	if (error) {
		printk(KERN_ERR "%s: Fail to register device\n", __FUNCTION__);
		goto err1;
	}
	
	ts_dev->touchmode = AMRI5K_MULTITOUCH_MODE;
	ts_dev->spi = spi;
	ts_dev->tdev = AMRI5K_m_ts_input;

    INIT_WORK(&ts_dev->poswork, AMRI5K_m_ts_read_touch_event_work);

#ifdef AMRI5K_ESD_RECOVERY
	INIT_DELAYED_WORK(&ts_dev->esdwork, AMRI5K_m_ts_esd_recovery_work);	
#endif
#ifdef AMRI5K_DELAY_RELEASE_TIME
    INIT_DELAYED_WORK(&ts_dev->releasework, AMRI5K_m_ts_release_touch_event_work); 
#endif

	error = AMRI5K_Mlcd_InitSromDownload();	
	if(error)
	{		
		printk(KERN_ERR"failed to initial MLCD_TOUCH_AMRI5K\n");
		goto err1;		
	}

#ifdef AMRI5K_CALIBRATION_SCAN_FREQUENCE
    mdelay(100);
	data_old = AMRI5K_m_ts_spi_read_reg( 0x35);
	dprintk( "data_old:%d\n",data_old);

    do_gettimeofday(&tv);
    ts_dev->usec_time_old = tv.tv_usec;
        
    mdelay(100);
	data1 = AMRI5K_m_ts_spi_read_reg( 0x35);

    do_gettimeofday(&tv);
    usec_time = tv.tv_usec;
    if(data_old > data1)
        delta = (data1 + (0x100 - data_old)) & 0xff;
    else
        delta = data1 - data_old;

    if (usec_time > ts_dev->usec_time_old)
        msec_time = (usec_time - ts_dev->usec_time_old)/1000;
    else
        msec_time = ((1000000 - ts_dev->usec_time_old)+usec_time)/1000;
    
    osc_temp = (8192*M_AMRI5K_HB*delta)/msec_time;


    dprintk( "data_old:%d data1:%d delta:%d\n",data_old,data1,delta);
    dprintk( "msec_time:%d\n",msec_time);
    dprintk( "osc_temp = 0x%x\n", osc_temp);
    dprintk( "delta = 0x%x\n", delta);

    if (osc_temp >= 25500)
    {
         m_clk =  osc_temp/244 - 82;          // 2*122KHz

        if((m_clk < 0) || (m_clk > 31)) // m_clk error
        {
            m_clk = 24; // wreg_data=0xE2
            dprintk("error. set default mclk%d\n", m_clk);
        }
        ts_dev->cal_clock[0] = (m_clk*8)+2;
        dprintk( "m_clk = 0x%x\n", m_clk);

        m_clk =  osc_temp/278 - 82;          // 2*139KHz
        
        if((m_clk < 0) || (m_clk > 31)) // m_clk error
        {
            m_clk = 24; // wreg_data=0xE2
            dprintk("error. set default mclk%d\n", m_clk);
        }
        ts_dev->cal_clock[1] = (m_clk*8)+2;
        dprintk( "m_clk = 0x%x\n", m_clk);

        m_clk =  osc_temp/296 - 82;          // 2*148KHz
        
        if((m_clk < 0) || (m_clk > 31)) // m_clk error
        {
            m_clk = 24; // wreg_data=0xE2
            dprintk("error. set default mclk%d\n", m_clk);
        }
        ts_dev->cal_clock[2] = (m_clk*8)+2;
        dprintk( "m_clk = 0x%x\n", m_clk);
    }
    else
    {
        m_clk =  osc_temp/224 - 82;          // 2*112KHz

        if((m_clk < 0) || (m_clk > 31)) // m_clk error
        {
            m_clk = 24; // wreg_data=0xE2
            dprintk("error. set default mclk%d\n", m_clk);
        }
        ts_dev->cal_clock[0] = (m_clk*8)+2;
        dprintk( "m_clk = 0x%x\n", m_clk);

        m_clk =  osc_temp/244 - 82;          // 2*122KHz
        
        if((m_clk < 0) || (m_clk > 31)) // m_clk error
        {
            m_clk = 24; // wreg_data=0xE2
            dprintk("error. set default mclk%d\n", m_clk);
        }
        ts_dev->cal_clock[1] = (m_clk*8)+2;
        dprintk( "m_clk = 0x%x\n", m_clk);

        m_clk =  osc_temp/278 - 82;          // 2*139KHz
        
        if((m_clk < 0) || (m_clk > 31)) // m_clk error
        {
            m_clk = 24; // wreg_data=0xE2
            dprintk("error. set default mclk%d\n", m_clk);
        }
        ts_dev->cal_clock[2] = (m_clk*8)+2;
        dprintk( "m_clk = 0x%x\n", m_clk);

        
    }
    
    ts_dev->cal_clock_sel = 0;
    dprintk( "0x71 = 0x%x\n", ts_dev->cal_clock[ts_dev->cal_clock_sel]);
    AMRI5K_m_ts_spi_write_reg(0x71, ts_dev->cal_clock[ts_dev->cal_clock_sel]);
    AMRI5K_m_ts_spi_write_reg(0x70, 0x36);
    mdelay(10);
    AMRI5K_m_ts_spi_write_reg(0x0F, 0x7B);
    mdelay(10);
#endif
#ifdef AMRI5K_DF_COMMAND
	while(1)
	{
		data1 = AMRI5K_m_ts_spi_read_reg( 0x0F);
		if (  ( (data1&0x01) == 0x00)|(i>100) )
			{
				break;
			}
		mdelay(20);
		dprintk( "0x0F = 0x%x\n", data1);
		i++;
	}
	AMRI5K_m_ts_spi_write_reg(0x71, 0x60);
	AMRI5K_m_ts_spi_write_reg(0x72, 0x08);
	AMRI5K_m_ts_spi_write_reg(0x70, 0x33);
	mdelay(10);
#endif



#ifdef AMRI5K_FIRST_INT_DISABLE
    m_ignore_first_intterupt_flg = TRUE;
#endif    

	/* MLCD Touch Interrupt */
	/* Touch interrupt setting - main lcd touch default */
	error = gpio_request(MLCD_AMRI5K_IRQ_GPIO, "amri5k_m_ts_irq");
	if (error) {
		printk(KERN_ERR"Unable to request touchscreen GPIO.\n");
		goto err2;
	}
	
	gpio_direction_input(MLCD_AMRI5K_IRQ_GPIO);	
	
	irq = gpio_to_irq(MLCD_AMRI5K_IRQ_GPIO);
	if (irq < 0) {
		error = irq;
		printk(KERN_ERR"Unable to request gpio irq. err=%d\n", error);
		gpio_free(MLCD_AMRI5K_IRQ_GPIO);
		goto err2;
	}

	ts_dev->irq = irq;

	error = request_irq(ts_dev->irq, AMRI5K_m_ts_isr, IRQF_TRIGGER_FALLING, M_TS_DRIVER_NAME, ts_dev);
	if (error) {
		printk (KERN_ERR"unable to claim irq %d: err %d\n", irq, error);
		gpio_free(MLCD_AMRI5K_IRQ_GPIO);
		goto err2;
	}

#ifdef AMRI5K_ESD_RECOVERY
//    schedule_delayed_work(&ts_dev->esdwork, msecs_to_jiffies(AMRI5K_ESD_POLLING_TIME + 5000)); // Check start after minimum 5 secs
    queue_delayed_work(tm_work_queue, &ts_dev->esdwork, msecs_to_jiffies(AMRI5K_ESD_POLLING_TIME + 5000)); // Check start after minimum 5 secs

    dprintk("-----------------------------------------------------------\n");
    dprintk("Register workqueue AMRI5K_m_ts_esd_recovery_work \n");
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
		m_ts_early_suspend.suspend = amri5k_m_ts_early_suspend;
		m_ts_early_suspend.resume = amri5k_m_ts_late_resume;
		m_ts_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 40;
		register_early_suspend(&m_ts_early_suspend);
#endif

#ifdef CONFIG_LGE_TOUCH_DRIVER_VERSION
	amri5k_m_ts_create_file(AMRI5K_m_ts_input);
#endif

	return error;

 err2:
	input_unregister_device(AMRI5K_m_ts_input);
	AMRI5K_m_ts_input = NULL; /* so we dont try to free it below */
 err1:
	input_free_device(AMRI5K_m_ts_input);
	AMRI5K_m_ts_input = NULL;
	
	return error;
}


static int AMRI5K_m_ts_remove(struct spi_device *spi)
{
	struct AMRI5K_m_ts_device *priv = dev_get_drvdata(&spi->dev);

	gpio_free(MLCD_AMRI5K_IRQ_GPIO);
	input_unregister_device(priv->tdev);
	kfree(priv);

	dev_set_drvdata(&spi->dev, NULL);

	return 0;
}

static struct spi_driver AMRI5K_m_ts_driver = {
	.driver = {
		.name  = M_TS_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe         = AMRI5K_m_ts_probe,
	.remove        = __devexit_p(AMRI5K_m_ts_remove),
};

static int __init AMRI5K_m_ts_init(void)
{
	int ret;

	printk("AMRI5K Main Touch Driver Module Init\n");

	tm_work_queue = create_singlethread_workqueue("tm_work_queue");
	if (!tm_work_queue)
		return -ENOMEM;

	memset(&AMRI5K_m_ts_dev, 0, sizeof(struct AMRI5K_m_ts_device));
	ret = spi_register_driver(&AMRI5K_m_ts_driver);
	printk("AMRI5K Main Touch Driver Module Init End : %d\n", ret);
	return ret;
}

static void __exit AMRI5K_m_ts_exit(void)
{
#ifdef CONFIG_LGE_TOUCH_DRIVER_VERSION
	amri5k_m_ts_remove_file(AMRI5K_m_ts_input);
#endif

	printk(KERN_ERR"AMRI5K Driver Module Exit\n");
	spi_unregister_driver(&AMRI5K_m_ts_driver);
	if (tm_work_queue)
		destroy_workqueue(tm_work_queue);	
}

MODULE_AUTHOR("AVAGO Technology");
MODULE_DESCRIPTION("AMRI5K TouchController Driver");
MODULE_LICENSE("GPL");

module_init(AMRI5K_m_ts_init);
module_exit(AMRI5K_m_ts_exit);

