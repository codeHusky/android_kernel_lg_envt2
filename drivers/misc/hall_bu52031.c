/* drivers/misc/hall_bu52031.c
 *  reference : vibrator_eve.c
 *
 * Copyright (C) 2008 LGE, Inc.
 * Copyright (C) 2007 Google, Inc.
 *
 * Android Hall ic driver 
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
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>  
#include <linux/spinlock.h>
//#include <linux/at_kpd_eve.h> // [antispoon,diyu] 2009-07-17 for AT+GKPD
#include <linux/jiffies.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>
#include <mach/gpio.h>
//#include <mach/msm_i2ckbd.h> //by munyoug, hwang
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
#include <mach/msm_rpcrouter.h> //for rpc
#include <mach/system.h> //for reading REV. in Board.

#if defined (CONFIG_LGE_ATS_ETA_MTC)
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#endif /*CONFIG_LGE_ATS_ETA_MTC*/

#if defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
#include "../../arch/arm/mach-msm/smd_private.h"
#include "../../arch/arm/mach-msm/include/mach/lg_comdef.h"
#endif

#ifdef CONFIG_LGE_FOLDER_TEST_COUNT
#define FOLDER_TEST_ONE_COUNT 10
/* LGE_CHANGES_S [bk.shin@lge.com] 2010-07-06, [VS760]*/
extern void folder_count_get(int * count);
extern void folder_count_put(int * count);

static int nFolderCount;
static int nRest;
#endif

/* Miscellaneous device */
#define MISC_DEV_NAME		"hall-ic" 

#define REPORT_ANDROID 	1 /* definition in input_report from KERNEL to ANDROID */
#define START_PORTRAIT 	0 /* definition of starting portrait-window in ANDROID */
#define USE_IRQ			1

/*LG_FW : 2010.02.25 jinho.jang - VS760 */
#define GPIO_HALL_IC_IRQ  39

#define JIFFIES_TO_MS(t) ((t) * 1000 / HZ)
#define MS_TO_JIFFIES(j) ((j * HZ) / 1000)
#define MAX_TIMEOUT_MS   (15000)

#define SLIDE_DEBUG 1
#if SLIDE_DEBUG
#define SDBG(fmt, args...) printk(fmt, ##args)
#else
#define SDBG(fmt, args...) do {} while (0)
#endif /* SLIDE_DEBUG */

static void hall_ic_work_func(struct work_struct *work);

/* LG_FW : 2010.06.29 jinho.jang - suspend test */
#define ENVT2_HALL_IC

#define TIMERON  		1 /* Timer On : 1, Off: 0 */ 
#if TIMERON
static int timerOn=1;/* Timer On : 1, Off: 0 */ 
#else
static int timerOn=0;/* Timer On : 1, Off: 0 */ 
#endif 

/* LGE_CHANGE_S [diyu@lge.com] 2009-04-15*/
#if TIMERON
#define SLIDEUP_TIMEOUT_MS 400 /* 400: slide delay-timeout  */	

#define SLIDEUP_FAST_TIMEOUT_MS 100 /* 400: slide delay-timeout  */	

static int first_slide_timeout_ms =SLIDEUP_FAST_TIMEOUT_MS;
static int second_slide_timeout_ms =SLIDEUP_FAST_TIMEOUT_MS; /*MAX : 1000 = 1 second*/
#endif

#if defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
uint32 remote_pwr_on;
#endif

//static int i=0;
static atomic_t s_hall_ic = ATOMIC_INIT(0);
static atomic_t current_state = ATOMIC_INIT(0);
static atomic_t last_state = ATOMIC_INIT(0);
static atomic_t report_state = ATOMIC_INIT(0);
static struct workqueue_struct *hallic_wq;
static int s_hall_ic_gpio;
static int s_hallic_state ;
static int hall_ic_on =-1; 
static int check_count =0;
static int suspend_flag = 0;
static struct early_suspend hall_ic_early_suspend;
static int enable_irq_on =-1;

struct hall_ic_device {
	struct input_dev *input_dev;
	struct work_struct work;
	struct timer_list timer;
	struct input_dev *kpdev;
	spinlock_t	lock;
	int use_irq;
	int enabled;
	int sample_rate;
};

static struct hall_ic_device *_dev = NULL;

extern struct input_dev *qwerty_get_input_dev(void);

enum {
	GPIO_SLIDE_CLOSE=0,
	GPIO_SLIDE_OPEN,
};

enum {
	HALL_IC_EARLY_SUSPEND 	= 2,
	HALL_IC_EARLY_RESUME 	= 0,
	HALL_IC_SUSPEND 		= 1,
	HALL_IC_RESUME 			= 0,
};

#if defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
static uint32 lgfw_smem_remote_pwr_on(void)
{
	static uint32 *p_smem_remote_pwr_flag;

	uint32 size; 
    p_smem_remote_pwr_flag = (uint32 *)smem_get_entry( SMEM_REMOTE_PWR_ON_FLAG, &size );

	return *p_smem_remote_pwr_flag;
}
#endif

int is_folder_open(void)
{
	short gpiogetvalue;
#if defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
    if(remote_pwr_on == BOOT_UART_DIAG_M)
	return 0;
#endif
	gpiogetvalue = gpio_get_value(GPIO_HALL_IC_IRQ);
	
	return gpiogetvalue;
}

int get_folder_state(void)
{	
	return hall_ic_on;
}

void set_slide_open(int value)
{
	struct hall_ic_device *dev = _dev;
	hall_ic_on = value;
	disable_irq(s_hall_ic_gpio);
	enable_irq_on = 0;
		
	if(	hall_ic_on == 0){
		//write_gkpd_value (73); //LGE_CHANGE [antispoon@lge.com,diyu@lge.com] 2009-07-17 steal folder open ASCII (73) for AT+GKPD
		input_report_switch(dev->input_dev, SW_LID, 0);
		input_sync(dev->input_dev);
    	SDBG("Hall-ic - -> Open \n");
	}else if(hall_ic_on == 1){
		//write_gkpd_value (74);  //LGE_CHANGE [antispoon@lge.com,diyu@lge.com] 2009-07-17 steal folder closed ASCII (74) for AT+GKPD
		input_report_switch(dev->input_dev, SW_LID, 1);
		input_sync(dev->input_dev);
    	SDBG("Hall-ic - -> Close \n");
	}
	
	enable_irq(s_hall_ic_gpio);
	enable_irq_on = 1;
}
EXPORT_SYMBOL(set_slide_open);

static void hall_ic_disable(void)
{
	s_hallic_state = 0;
	SDBG("\nandroid-hall-ic: hall_ic_disble_set\n");
}

static void hall_ic_enable(void)
{
	s_hallic_state = 1;
	SDBG("\nandroid-hall-ic: hall_ic_enable_set\n");
}

static void hall_ic_set(int amp)
{
	
	SDBG("\nandroid-hall-ic: hall-ic_set\n");

	if(amp == 0) {
		hall_ic_disable();
	}else {
		hall_ic_enable();
	}
}

static void hall_ic_report_set(int amp)
{
	struct hall_ic_device *dev = _dev;
	short gpiogetvalue;

	if(amp == 1) {
			SDBG("\nandroid-hall-ic: hall-ic_set\n");
		gpiogetvalue = gpio_get_value(GPIO_HALL_IC_IRQ);
		if( gpiogetvalue == GPIO_SLIDE_OPEN ){ //OPEN
			hall_ic_on = 0;
			atomic_set(&current_state, 0 );
			#if REPORT_ANDROID
			input_report_switch(dev->input_dev, SW_LID, 0);
			input_sync(dev->input_dev);
			atomic_set(&report_state, 0 );
			atomic_set(&last_state, 0 );
			#endif
			SDBG("Hall-ic - -> Open \n");
			}else{ //CLOSE
			atomic_set(&current_state, 1 );
			hall_ic_on = 1;
			#if REPORT_ANDROID
			input_report_switch(dev->input_dev, SW_LID, 1);
			input_sync(dev->input_dev);
			atomic_set(&report_state, 1 );
			atomic_set(&last_state, 1 );
			#endif
			SDBG("Hall-ic - -> Close \n");
		}
	}else if(amp == 0) {
		gpiogetvalue = gpio_get_value(GPIO_HALL_IC_IRQ);
		if( gpiogetvalue == GPIO_SLIDE_OPEN ){ //OPEN
			hall_ic_on = 0;
			atomic_set(&current_state, 0 );
			atomic_set(&last_state, 0 );
			SDBG("Hall-ic - -> Open \n");
		}else{ //CLOSE
			hall_ic_on = 1;
			atomic_set(&current_state, 1 );
			atomic_set(&last_state, 1 );
			SDBG("Hall-ic - -> Close \n");
		}
	}	
}

static void hall_ic_report_event(int state) 
{
	short current_state_flag, last_state_flag;
	short report_state_flag;

	check_count++;
	current_state_flag = atomic_read(&current_state);
	last_state_flag = atomic_read(&last_state);
	if( current_state_flag  != last_state_flag )//Fast slide UP<->DOWN
	{ 
		check_count -=1;
		if( check_count <0 ){ 
			check_count = 0;
		}
		//SDBG("Hall-ic -> test 1 \n");
		if(second_slide_timeout_ms >= 300){
			
			//SDBG("Hall-ic -> test 2 \n");
			second_slide_timeout_ms = SLIDEUP_FAST_TIMEOUT_MS;/*200;*/
		}
		second_slide_timeout_ms += 50;
	}else //Slide slide UP<->DOWN/ finish slide-up.
	{
		//SDBG("Hall-ic -> test 3 \n");
		if(second_slide_timeout_ms < 100){
			//SDBG("Hall-ic -> test 4 \n");
			second_slide_timeout_ms = SLIDEUP_FAST_TIMEOUT_MS;
		}
		second_slide_timeout_ms -= 50;
	}

	if(check_count > 5)
	{
    	current_state_flag = atomic_read(&current_state);
    	report_state_flag = atomic_read(&report_state);
    	if (current_state_flag == 0) 
		{
			hall_ic_on = 0;
			SDBG("Hall-ic -> Open 1 (%d)\n", check_count);	  
#ifdef CONFIG_LGE_FOLDER_TEST_COUNT
			nRest++;
#endif
		} 
		else 
		{
			hall_ic_on = 1;
			SDBG("Hall-ic -> Close 1 %d\n", check_count);	  
		}
		if( report_state_flag != current_state_flag )
		{
			input_report_switch(_dev->input_dev, SW_LID, hall_ic_on);
			input_sync(_dev->input_dev);
			atomic_set(&report_state, current_state_flag);
			second_slide_timeout_ms = SLIDEUP_FAST_TIMEOUT_MS;	  
		}

		check_count=0;
		if(enable_irq_on ==0)
		{
			enable_irq(s_hall_ic_gpio);
			enable_irq_on = 1;
		}
    }	
	else
	{
		mod_timer(&_dev->timer, jiffies + (second_slide_timeout_ms * HZ / 1000));
	}
	atomic_set(&last_state, current_state_flag);
	
	return;
}

static void hall_timer(unsigned long arg)
{
	short gpiogetvalue;
	
	if(timerOn == 1){
		SDBG(" current_state %d \n", atomic_read(&current_state));
		gpiogetvalue = gpio_get_value(GPIO_HALL_IC_IRQ);
		if( gpiogetvalue == GPIO_SLIDE_OPEN ) //OPEN
		{
			atomic_set(&current_state, 0 );
			#if REPORT_ANDROID
			hall_ic_report_event(0);
			#endif
			//SDBG("Hall-ic TimerFunc - -> Open 2 \n");
			
			//SDBG(" last_state %d \n", atomic_read(&last_state) );
		}else //CLOSE
		{ 
			atomic_set(&current_state, 1 );
			#if REPORT_ANDROID
			hall_ic_report_event(1);
			#endif
			//SDBG("Hall-ic TimerFunc - -> Close 2 \n");
			//SDBG(" last_state %d \n", atomic_read(&last_state) );
			}
		}
	return;
}


static void hall_ic_work_func(struct work_struct *work)
{
	struct hall_ic_device *dev = container_of(work, struct hall_ic_device, work);
	short gpiogetvalue;
	short current_state_flag; 
	short report_state_flag;

	if(timerOn ==1)
	{
#if defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
		if(remote_pwr_on == BOOT_UART_DIAG_M)
		gpiogetvalue = GPIO_SLIDE_CLOSE;
		else
#endif
		gpiogetvalue = gpio_get_value(GPIO_HALL_IC_IRQ);

		if( gpiogetvalue == GPIO_SLIDE_OPEN )//OPEN
		{ 
			atomic_set(&current_state, 0 );
			if(hall_ic_on==1)
			{
				hall_ic_on = 0;
			#if REPORT_ANDROID
				report_state_flag = atomic_read(&report_state);
				current_state_flag = atomic_read(&current_state);
				if( report_state_flag != current_state_flag )
				{				
					report_state = current_state;
					input_report_switch(dev->input_dev, SW_LID, 0);
					input_sync(dev->input_dev);
					atomic_set(&report_state, 0 );
				}
			#endif
				SDBG("Hall-ic workFunc- -> Open \n");
			}
			else
			{
				SDBG("Hall-ic workFunc has Opened \n");
			}
			atomic_set(&last_state, 0 );
		}
		else  //CLOSE
		{
			atomic_set(&current_state, 1 );
			if(hall_ic_on==0)
			{
				hall_ic_on = 1;
			#if REPORT_ANDROID
				report_state_flag = atomic_read(&report_state);
				current_state_flag = atomic_read(&current_state);
				if( report_state_flag != current_state_flag )
				{				
					input_report_switch(dev->input_dev, SW_LID, 1);
					input_sync(dev->input_dev);
					atomic_set(&report_state, 1 );
				}
			#endif
				SDBG("Hall-ic workFunc -> Close \n");
			}else
			{
        		SDBG("Hall-ic workFunc has Closed \n");
			}
			atomic_set(&last_state, 1 );
		}
		mod_timer(&dev->timer, jiffies + (first_slide_timeout_ms * HZ / 1000));
	}
	else
	{
#if defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
		if(remote_pwr_on == BOOT_UART_DIAG_M)
		gpiogetvalue = GPIO_SLIDE_CLOSE;
		else
#endif
		gpiogetvalue = gpio_get_value(GPIO_HALL_IC_IRQ);
		if( gpiogetvalue == GPIO_SLIDE_OPEN ) //OPEN
		{
			atomic_set(&current_state, 0 );
			if(hall_ic_on==1)
			{
				hall_ic_on = 0;
			#if REPORT_ANDROID
				input_report_switch(dev->input_dev, SW_LID, 0);
				input_sync(dev->input_dev);
				atomic_set(&report_state, 0 );
			#endif
				SDBG("Hall-ic workFunc- -> Open \n");
			}else
			{
				SDBG("Hall-ic workFunc has Opened \n");
			}				
			atomic_set(&last_state, 0 );
		}else //CLOSE
		{
			atomic_set(&current_state, 1 );
			if(hall_ic_on==0)
			{
				hall_ic_on = 1;
			#if REPORT_ANDROID
				input_report_switch(dev->input_dev, SW_LID, 1);
				input_sync(dev->input_dev);
				atomic_set(&report_state, 1 );
			#endif
				SDBG("Hall-ic workFunc -> Close \n");
			}else
			{
				SDBG("Hall-ic workFunc has Closed \n");
			}
			atomic_set(&last_state, 1 );
		}
	}
}

static int hall_ic_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int hall_ic_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int hall_ic_ioctl(struct inode *inode, struct file *filp, 
		unsigned int cmd, unsigned long arg)
{
#if defined (CONFIG_LGE_ATS_ETA_MTC)
	int hall_ic_status;
#endif /*CONFIG_LGE_ATS_ETA_MTC*/

	struct hall_ic_device *dev = (struct hall_ic_device *)_dev;
	if (NULL == dev)
		return -ENODEV;

#if defined (CONFIG_LGE_ATS_ETA_MTC)
	if(_IOC_TYPE(cmd) != HALL_IC_IOCTL_BASE)
	{
		printk(KERN_ERR "%s, received command : 0x%x, cmd_type : %c\n", __func__, cmd, _IOC_TYPE(cmd));
		return -ENOTTY;
	}

	printk(KERN_INFO "%s, received command : 0x%x\n", __func__, cmd);
	switch (cmd) {
		case HALL_IC_IOCTL_STATUS:
			hall_ic_status = get_folder_state();
			printk(KERN_INFO "%s, received hall ic status command, status : %d\n", __func__, hall_ic_status);
			return put_user(hall_ic_status, (unsigned long *) arg);
			
		default :
			printk(KERN_ERR "%s, wrong command, return fault\n", __func__);
			return -EFAULT;
	}
#else /*CONFIG_LGE_ATS_ETA_MTC*/
	switch (cmd) {
		case 1:
			break;
		default:
			return -ENOTTY;
	}

	return 0;
#endif /*CONFIG_LGE_ATS_ETA_MTC*/
}

/* use miscdevice for ioctls */
static struct file_operations fops = {
	.owner   = THIS_MODULE,
	.open    = hall_ic_open,
	.release = hall_ic_release,
	.ioctl   = hall_ic_ioctl,
};

static struct miscdevice misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = MISC_DEV_NAME,
	.fops  = &fops,
};

static ssize_t hall_ic_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(gpio_get_value(GPIO_HALL_IC_IRQ)==1){ //OPEN
			hall_ic_on = 0; /*slide up*/
	}else{ //CLOSE
			hall_ic_on = 1; /*slide down*/
	}
	return sprintf(buf, "%d\n", hall_ic_on );
}

static ssize_t hall_ic_enable_store(
		struct device *dev, struct device_attribute *attr, 
		const char *buf, size_t size)
{
	int value;
	sscanf(buf, "%d", &value);

	atomic_set(&s_hall_ic, value);

	/*  value == 0  : stop hall ic	 
	 *    value ==1  : start hall ic*/
	value = !!value;

	if(value) {
		hall_ic_set(1);
		SDBG("\nandroid-hall-ic: hall_ic enabled\n");
	}else {
		hall_ic_set(0);
		SDBG("\nandroid-hall-ic: hall_ic disbled\n");
	}

	return size;
}

static ssize_t hall_ic_set_report(
		struct device *dev, struct device_attribute *attr, 
		const char *buf, size_t size)
{
	int value;
	sscanf(buf, "%d", &value);

	atomic_set(&s_hall_ic, value);

	/*  value == 0  : status_save hall ic	 
	 *    value ==1  : report_and_status_save hall ic*/

	if(value) {
		hall_ic_report_set(1);
	}else {
		hall_ic_report_set(0);
	}

	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, hall_ic_enable_show, hall_ic_enable_store);
static DEVICE_ATTR(report, S_IRUGO | S_IWUSR, NULL , hall_ic_set_report);

/* hall_ic_on = 1 (open) :  hall_ic_on = 0 (close) :  
  * HIGH : Open  //LOW : Close   */
static int hall_ic_irq_handler(int irq, void *dev_id)
{
	struct hall_ic_device *dev = dev_id;
	short gpiogetvalue;
	SDBG("\n\nCheck point : %s\n", __FUNCTION__);

#if defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
    if(remote_pwr_on == BOOT_UART_DIAG_M)
	gpiogetvalue = GPIO_SLIDE_CLOSE;
	else
#endif
	gpiogetvalue = gpio_get_value(GPIO_HALL_IC_IRQ);
	if(gpiogetvalue == GPIO_SLIDE_OPEN )
		atomic_set(&current_state, 0);
	else
		atomic_set(&current_state, 1);

	if(s_hallic_state == 1){
		SDBG("\n\nCheck point :hall_ic_on - 1 : %s\n ", __FUNCTION__);
/* LG_FW : 2010.02.06 jinho.jang - for irq fail at boot */
#if defined(CONFIG_LGE_ANDROID_HALL_IC)
		disable_irq_nosync(s_hall_ic_gpio);
#else
		disable_irq(s_hall_ic_gpio);
#endif
		enable_irq_on = 0;
		queue_work(hallic_wq, &dev->work);
	}
	
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hall_early_suspend(struct early_suspend *h) 
{
	SDBG(KERN_INFO"%s: \n",__func__);
	suspend_flag = HALL_IC_EARLY_SUSPEND;
#ifdef CONFIG_LGE_FOLDER_TEST_COUNT
	if (nRest >= FOLDER_TEST_ONE_COUNT)
	{
		nFolderCount = nFolderCount + (nRest / FOLDER_TEST_ONE_COUNT);
	  	folder_count_put(&nFolderCount);
		nRest = nRest % FOLDER_TEST_ONE_COUNT;
	}
#endif	
	
	return;
}

static void hall_late_resume(struct early_suspend *h)
{
	struct hall_ic_device *dev = (struct hall_ic_device *)_dev;
	short gpiogetvalue;
	short current_state_flag; 
	short report_state_flag;

	SDBG(KERN_INFO"%s: \n",__func__);	
	suspend_flag = HALL_IC_EARLY_RESUME;

 		gpiogetvalue = gpio_get_value(GPIO_HALL_IC_IRQ);
		if( gpiogetvalue == GPIO_SLIDE_OPEN ){ //OPEN
				atomic_set(&current_state, 0 );
				if(hall_ic_on==1){
					hall_ic_on = 0;
					#if REPORT_ANDROID
					report_state_flag = atomic_read(&report_state);
					current_state_flag = atomic_read(&current_state);
					if( report_state_flag != current_state_flag ){
						report_state = current_state;
						input_report_switch(dev->input_dev, SW_LID, 0);
						input_sync(dev->input_dev);
						atomic_set(&report_state, 0 );
					}
					#endif
					SDBG("Hall-ic workFunc- -> Open \n");
				}else{
					SDBG("Hall-ic workFunc has Opened \n");
				}				
				atomic_set(&last_state, 0 );
			}else{ //CLOSE
				atomic_set(&current_state, 1 );
				if(hall_ic_on==0){
					hall_ic_on = 1;
					#if REPORT_ANDROID
					report_state_flag = atomic_read(&report_state);
					current_state_flag = atomic_read(&current_state);
					if( report_state_flag != current_state_flag ){
						input_report_switch(dev->input_dev, SW_LID, 1);
						input_sync(dev->input_dev);
						atomic_set(&report_state, 1 );
					}
					#endif
					SDBG("Hall-ic workFunc -> Close \n");
				}else{
					SDBG("Hall-ic workFunc has Closed \n");
				}
				atomic_set(&last_state, 1 );
			}
	
	return;
}
#endif

static int hall_suspend(struct platform_device *pdev, pm_message_t state) 
{
	suspend_flag = HALL_IC_SUSPEND;
	return 0;
}

static int hall_resume(struct platform_device *pdev) 
{
	suspend_flag = HALL_IC_RESUME;
	
	return 0;
}

static int android_hall_ic_probe(struct platform_device *pdev)
{
	struct hall_ic_device *dev;
	struct input_dev *input_dev;
	int ret, err;
#if 0
	unsigned int lge_hw_rev;

 	/*for reading Board revision, by jinwoonam@lge.com*/
	lge_hw_rev = system_rev;
	if(lge_hw_rev <= LGE_PCB_VER_G){
		timerOn =1; 
		printk("Hall   		REV.B,C,D\n");
	}else if(lge_hw_rev >= LGE_PCB_VER_1_0){
		timerOn =1; 
		printk("Hall   		REV.1.0  \n");
	}
#endif

	dev = kzalloc(sizeof(struct hall_ic_device), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!dev|| !input_dev) {
			ret = -ENOMEM;
			printk("diyu input_allocate_device failed\n");
			goto err_input_register_device;
	}
	
	platform_set_drvdata(pdev, dev);
	_dev = dev; /* for miscdevice */

#if defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
	remote_pwr_on = lgfw_smem_remote_pwr_on();
#endif
	
	INIT_WORK(&dev->work, hall_ic_work_func);
	
	dev->use_irq = USE_IRQ;

	ret = misc_register(&misc_dev);
	if (ret) {
		printk("diyu failed to register miscdevice\n");
		goto err_miscdevice;
	}

	input_dev->name = "Slide Hall-ic";
	input_dev->phys = "hallic/input0";
	input_dev->dev.parent = &pdev->dev;
	input_dev->evbit[0] =  BIT_MASK(EV_SW);
	
	set_bit(SW_LID, input_dev->swbit);

	ret = input_register_device(input_dev);
	if (ret) {
		printk("diyu input_allocate_device failed\n");
		goto err_input_register_device;
	}
	dev->input_dev = input_dev;
		
	// hall_ic_on 
	s_hallic_state = 1; 
#if defined(CONFIG_LGE_UART3_SUPPORT_FACTORY_POWER_INSPECTION)
    if(remote_pwr_on == BOOT_UART_DIAG_M)
	s_hall_ic_gpio = GPIO_SLIDE_CLOSE;
	else
#endif
	s_hall_ic_gpio = gpio_to_irq(GPIO_HALL_IC_IRQ);//Do Not need. To assign at board_eve_gpio_i2c.c

	/* hall_ic_on = 1 (open) :  hall_ic_on = 0 (close) :  
	  * HIGH : Open  //LOW : Close
	  */
 	ret = request_irq(s_hall_ic_gpio, hall_ic_irq_handler,
		IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
			"hall-ic", dev);

	err = set_irq_wake(s_hall_ic_gpio, 1);
	if (err) {
		pr_err("hall-ic: set_irq_wake failed for gpio %d, "
			"irq %d\n", GPIO_HALL_IC_IRQ, s_hall_ic_gpio);
	}
 
	#if START_PORTRAIT /*Don't roatate*/
		input_report_switch(dev->input_dev, SW_LID, 1);
		input_sync(dev->input_dev);
	#endif
			
	if (ret) {
		printk("\nHALL IC IRQ Check-fail\n pdev->client->irq %d\n",s_hall_ic_gpio);
		goto err_request_irq;
	}
	
	ret = device_create_file(&pdev->dev, &dev_attr_enable);
	if (ret) {
		printk( "android-hall-ic: hall-ic_probe: Fail\n");
		device_remove_file(&pdev->dev, &dev_attr_enable);
		goto err_request_irq;
	}
	
	ret = device_create_file(&pdev->dev, &dev_attr_report);
	if (ret) {
		printk( "android-hall-ic: hall-ic_probe: Fail\n");
		device_remove_file(&pdev->dev, &dev_attr_report);
		goto err_request_irq;
	}
if(timerOn ==1){ //#if TIMERON
	setup_timer(&dev->timer, hall_timer, (unsigned long)dev);
}//#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	hall_ic_early_suspend.suspend = hall_early_suspend;
	hall_ic_early_suspend.resume = hall_late_resume;
	hall_ic_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 45;

	register_early_suspend(&hall_ic_early_suspend);
#endif

#ifdef CONFIG_LGE_FOLDER_TEST_COUNT
	folder_count_get(&nFolderCount);
	SDBG("hall-ic folder count : %d\n",nFolderCount);
#endif

	SDBG(KERN_ERR "android-hall_ic: hall_ic_probe: Done\n");
	
	return 0;
err_request_irq:
	input_unregister_device(input_dev);
err_input_register_device:
	input_free_device(input_dev);
err_miscdevice:
	kfree(dev);
	return ret;
}

static int android_hall_ic_remove(struct platform_device *pdev) 
{
	struct hall_ic_device *dev = (struct hall_ic_device *)_dev;

	device_remove_file(&pdev->dev, &dev_attr_enable);
	device_remove_file(&pdev->dev, &dev_attr_report);
	del_timer_sync(&dev->timer);
	
	return 0;
}

static struct platform_driver android_hall_ic_driver = {
	.probe		= android_hall_ic_probe,
	.remove		= android_hall_ic_remove,
	.suspend 	= hall_suspend,
	.resume		= hall_resume,
	.driver		= {
		.name		= "hall-ic",
		.owner		= THIS_MODULE,
	},
};

static int __init android_hall_ic_init(void)
{
	hallic_wq = create_singlethread_workqueue("hallic_wq");
	if (!hallic_wq){
		SDBG("\n\n Check point2 : %s\n", __FUNCTION__);
		return -ENOMEM;
	}

	SDBG( "android_hall_ic: init\n");
	return platform_driver_register(&android_hall_ic_driver);
}

static void __exit android_hall_ic_exit(void)
{
	SDBG( "android_hall_ic: exit\n");
 	platform_driver_unregister(&android_hall_ic_driver);
}

module_init(android_hall_ic_init);
module_exit(android_hall_ic_exit);

MODULE_AUTHOR("Dae il, yu <diyu@lge.com>");
MODULE_DESCRIPTION("EVE hall ic driver for Android");
MODULE_LICENSE("GPL");
