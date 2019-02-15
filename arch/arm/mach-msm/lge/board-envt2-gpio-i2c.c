/* linux/arch/arm/mach-msm/board-griffin-gpio-i2c.c
 *
 * Copyright (C) 2009 LGE, Inc.
 * Author: SungEun Kim <cleaneye@lge.com>
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
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/tsc2007.h>
#include <mach/lg_sensors.h>

#ifdef CONFIG_LGE_TOUCH
#include <linux/synaptics_i2c_rmi.h>
#endif

/* LG_FW : 2010.03.29 jiho.jang - Dual LCD Switch */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA)
#include <mach/board.h>
#endif

//inforpc
#include <mach/vreg.h>          /* set a vreg */
#include <linux/delay.h>
#if defined (CONFIG_LGE_MACH_ENVT2_CHECK_HW_REV)
#include "../devices.h"
#endif


enum
{
#ifdef CONFIG_LGE_MACH_ENVT2_REVC
	I2C_BUS_NUM_CAMISP_CE147=1,
#endif
	I2C_BUS_NUM_MLCD_TOUCH=2,
	I2C_BUS_NUM_SLCD_TOUCH,
	I2C_BUS_NUM_MLCD_CHARGE_PUMP,
	I2C_BUS_NUM_SLCD_CHARGE_PUMP,
#ifdef CONFIG_LGE_MACH_ENVT2_REVC	
	I2C_BUS_NUM_CAMERA_POWER,
#endif	
	I2C_BUS_NUM_FUEL,
	I2C_BUS_NUM_SENSOR_ACCEL,
	I2C_BUS_NUM_SENSOR_COMPASS,
	I2C_BUS_NUM_PROXI,
#ifdef CONFIG_LGE_AUDIO_SUBSYSTEM
	I2C_BUS_NUM_AMP,
#endif 
	I2C_BUS_NUM_MAX
};
//LG_FW: 2010.04.24 woochang.chun
//----------------------------------------
#define GPIO_I2C1_SDA		143
#define GPIO_I2C1_SCL		144
#define GPIO_I2C2_SDA		146
#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
#define GPIO_I2C2_SCL		107 //147
#elif defined (CONFIG_LGE_MACH_ENVT2_REVC)
#define GPIO_I2C2_SCL		147
#else
#define GPIO_I2C2_SCL		147
#endif
//----------------------------------------

typedef void (gpio_i2c_init_func_t)(void);

/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/
/* LG_FW_VS760 : 2010.03.15 - jinho.jang */
#define TOUCH_DEVICE_I2C_ID 0x6B
#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
#define MAIN_GPIO_TOUCH_I2C_SDA 31
#define MAIN_GPIO_TOUCH_I2C_SCL 34
#define MAIN_GPIO_TOUCH_RST	109

#ifdef CONFIG_FB_MSM_USE_DMAE_VSYNC
#define MAIN_GPIO_TOUCH_ATTEN 100
#else
#define MAIN_GPIO_TOUCH_ATTEN 141
#endif

#define SUB_GPIO_TOUCH_I2C_SDA 33
#define SUB_GPIO_TOUCH_I2C_SCL 32

#define SUB_GPIO_TOUCH_RST	33
#define SUB_GPIO_TOUCH_ATTEN 42
#elif defined (CONFIG_LGE_MACH_ENVT2_REVC)
#define MAIN_GPIO_TOUCH_I2C_SDA 31
#define MAIN_GPIO_TOUCH_I2C_SCL 34
#define MAIN_GPIO_TOUCH_RST	109
#define MAIN_GPIO_TOUCH_ATTEN 141

#define SUB_GPIO_TOUCH_I2C_SDA 33
#define SUB_GPIO_TOUCH_I2C_SCL 32

#define SUB_GPIO_TOUCH_RST	19
#define SUB_GPIO_TOUCH_ATTEN 42
#else
#define MAIN_GPIO_TOUCH_I2C_SDA 31
#define MAIN_GPIO_TOUCH_I2C_SCL 34
#define MAIN_GPIO_TOUCH_RST	109
#define MAIN_GPIO_TOUCH_ATTEN 141

#define SUB_GPIO_TOUCH_I2C_SDA 33
#define SUB_GPIO_TOUCH_I2C_SCL 32

#define SUB_GPIO_TOUCH_RST	19
#define SUB_GPIO_TOUCH_ATTEN 42
#endif
#ifdef CONFIG_LGE_ANDROID_HALL_IC
#define HALL_IC_IRQ 39
#endif

#ifdef CONFIG_LGE_MACH_ENVT2_REVC
/* Main LCD Touchscreen */
static struct i2c_gpio_platform_data touch_i2c_mlcd_pdata = {
	.sda_pin = MAIN_GPIO_TOUCH_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = MAIN_GPIO_TOUCH_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device touch_i2c_mlcd_device = {
	.name = "i2c-gpio",
	.id = I2C_BUS_NUM_MLCD_TOUCH,
	.dev.platform_data = &touch_i2c_mlcd_pdata,
};

static struct i2c_board_info touch_i2c_mlcd_bdinfo = {
	I2C_BOARD_INFO("AMRI5K_MTOUCH", TOUCH_DEVICE_I2C_ID),/*AMRI5K"*/
	.irq = MSM_GPIO_TO_INT(MAIN_GPIO_TOUCH_ATTEN),
};

void __init init_i2c_mlcd_touch(void)
{
	/* Main LCD Touchscreen */
	gpio_configure(MAIN_GPIO_TOUCH_I2C_SDA, GPIOF_INPUT);
	gpio_configure(MAIN_GPIO_TOUCH_I2C_SCL, GPIOF_INPUT);
	gpio_configure(MAIN_GPIO_TOUCH_ATTEN, GPIOF_INPUT);

	i2c_register_board_info(I2C_BUS_NUM_MLCD_TOUCH, &touch_i2c_mlcd_bdinfo, 1);
	platform_device_register(&touch_i2c_mlcd_device);
}

/* Default - Sub LCD Touchscreen */
static struct i2c_gpio_platform_data touch_i2c_slcd_pdata = {
	.sda_pin = SUB_GPIO_TOUCH_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = SUB_GPIO_TOUCH_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device touch_i2c_slcd_device = {
	.name = "i2c-gpio",
	.id = I2C_BUS_NUM_SLCD_TOUCH,
	.dev.platform_data = &touch_i2c_slcd_pdata,
};

static struct i2c_board_info touch_i2c_slcd_bdinfo = {
	I2C_BOARD_INFO("AMRI5K_STOUCH", TOUCH_DEVICE_I2C_ID),/*AMRI5K"*/
	.irq = MSM_GPIO_TO_INT(SUB_GPIO_TOUCH_ATTEN),
};

void __init init_i2c_slcd_touch(void)
{	
	/* Sub LCD Touchscreen */
	gpio_configure(SUB_GPIO_TOUCH_I2C_SDA, GPIOF_INPUT);
	gpio_configure(SUB_GPIO_TOUCH_I2C_SCL, GPIOF_INPUT);	
	gpio_configure(SUB_GPIO_TOUCH_ATTEN, GPIOF_INPUT);

	i2c_register_board_info(I2C_BUS_NUM_SLCD_TOUCH, &touch_i2c_slcd_bdinfo, 1);
	platform_device_register(&touch_i2c_slcd_device);
}
/*-----------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*/
#endif

/* For Backlight */
/* LG_FW : 2010.02.08 jinho.jang */
struct aat2870gu_platform_data {
	void	(*platform_init)(void);
	unsigned int op_mode;
	spinlock_t lock;
};
#if defined (CONFIG_LGE_MACH_ENVT2_CHECK_HW_REV)
#define GPIO_MLCD_REV1P0_CP_I2C_SCL 94
#define GPIO_MLCD_REV1P0_CP_I2C_SDA 104
#define GPIO_MLCD_REVE_CP_I2C_SCL 137
#define GPIO_MLCD_REVE_CP_I2C_SDA 138

#define GPIO_MLCD_CP_EN 142

#define GPIO_SLCD_REV1P0_CP_I2C_SCL 158
#define GPIO_SLCD_REV1P0_CP_I2C_SDA 159
#define GPIO_SLCD_REVE_CP_I2C_SCL 135
#define GPIO_SLCD_REVE_CP_I2C_SDA 136

#define GPIO_SLCD_CP_EN 27

#elif defined (CONFIG_LGE_MACH_ENVT2_REV1P0)
#define GPIO_MLCD_CP_I2C_SCL 94
#define GPIO_MLCD_CP_I2C_SDA 104
#define GPIO_MLCD_CP_EN 142

#define GPIO_SLCD_CP_I2C_SCL 158
#define GPIO_SLCD_CP_I2C_SDA 159
#define GPIO_SLCD_CP_EN 27

#elif defined (CONFIG_LGE_MACH_ENVT2_REVD)
#define GPIO_MLCD_CP_I2C_SCL 137
#define GPIO_MLCD_CP_I2C_SDA 138
#define GPIO_MLCD_CP_EN 142

#define GPIO_SLCD_CP_I2C_SCL 135
#define GPIO_SLCD_CP_I2C_SDA 136
#define GPIO_SLCD_CP_EN 27
#elif defined (CONFIG_LGE_MACH_ENVT2_REVC)
#define GPIO_MLCD_CP_I2C_SCL GPIO_I2C1_SCL
#define GPIO_MLCD_CP_I2C_SDA GPIO_I2C1_SDA
#define GPIO_MLCD_CP_EN 142

#define GPIO_SLCD_CP_I2C_SCL GPIO_I2C2_SCL
#define GPIO_SLCD_CP_I2C_SDA GPIO_I2C2_SDA
#define GPIO_SLCD_CP_EN 27
#else
#define GPIO_MLCD_CP_I2C_SCL GPIO_I2C1_SCL
#define GPIO_MLCD_CP_I2C_SDA GPIO_I2C1_SDA
#define GPIO_MLCD_CP_EN 142

#define GPIO_SLCD_CP_I2C_SCL GPIO_I2C2_SCL
#define GPIO_SLCD_CP_I2C_SDA GPIO_I2C2_SDA
#define GPIO_SLCD_CP_EN 27
#endif


// The I2C Slave Address of AAT2870
#define AAT2870_DEVICE_I2C_ID	0x60


#if defined (CONFIG_LGE_MACH_ENVT2_CHECK_HW_REV)
static int hw_rev1p0=0;
#endif

#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
static void mlcd_aat2870gu_init(void)
{
#ifdef BACKLIGHT_RESET
	static int pin_request = 0;

	if(!pin_request)
	{
		if(gpio_request(GPIO_MLCD_CP_EN,"mlcd_aat2870 reset"))
		{
			printk(KERN_ERR "mlcd aat2870 gpio request is failed : reset");
			return;
		}
		
		gpio_configure(GPIO_MLCD_CP_EN, GPIOF_DRIVE_OUTPUT);
		gpio_set_value(GPIO_MLCD_CP_EN, 1);
		udelay(5);		
	}
#endif
}

/* default is slcd backlight */
static struct aat2870gu_platform_data mlcd_aat2870gu_data = {
	.platform_init = mlcd_aat2870gu_init,
};

static struct i2c_board_info mlcd_cp_i2c_device = {
	I2C_BOARD_INFO("mlcd_aat2870gu", AAT2870_DEVICE_I2C_ID),
	.platform_data = &mlcd_aat2870gu_data,
};
#if defined (CONFIG_LGE_MACH_ENVT2_CHECK_HW_REV)
static struct i2c_gpio_platform_data mlcd_rev1p0_cp_i2c_platform_pdata = {
	.sda_pin = GPIO_MLCD_REV1P0_CP_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_MLCD_REV1P0_CP_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct i2c_gpio_platform_data mlcd_reve_cp_i2c_platform_pdata = {
	.sda_pin = GPIO_MLCD_REVE_CP_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_MLCD_REVE_CP_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct i2c_gpio_platform_data mlcd_cp_i2c_platform_pdata = {
	.sda_pin = GPIO_MLCD_REV1P0_CP_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_MLCD_REV1P0_CP_I2C_SDA,
	.scl_is_open_drain = 0,
	.udelay = 2,
};


#else
static struct i2c_gpio_platform_data mlcd_cp_i2c_platform_pdata = {
	.sda_pin = GPIO_MLCD_CP_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_MLCD_CP_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};
#endif
static struct platform_device mlcd_i2c_platform_device = {
	.name = "i2c-gpio",
	.id = I2C_BUS_NUM_MLCD_CHARGE_PUMP,
	.dev.platform_data = &mlcd_cp_i2c_platform_pdata,
};
 
void __init init_i2c_mlcd_charge_pump(void)
{
	int rc;
	printk(KERN_INFO"%s: \n",__func__);	
#if defined (CONFIG_LGE_MACH_ENVT2_CHECK_HW_REV)
	if(lge_read_pcb_version()==0x10)
	{
		hw_rev1p0 = 1;
	}
	if (hw_rev1p0)
	{
		gpio_configure(GPIO_MLCD_REV1P0_CP_I2C_SDA, GPIOF_DRIVE_OUTPUT);
		gpio_set_value(GPIO_MLCD_REV1P0_CP_I2C_SDA, 1);
		gpio_configure(GPIO_MLCD_REV1P0_CP_I2C_SCL, GPIOF_DRIVE_OUTPUT);
		gpio_set_value(GPIO_MLCD_REV1P0_CP_I2C_SCL, 1);	
		rc = i2c_register_board_info(I2C_BUS_NUM_MLCD_CHARGE_PUMP, &mlcd_cp_i2c_device, 1);
		mlcd_i2c_platform_device.dev.platform_data=&mlcd_rev1p0_cp_i2c_platform_pdata;
	}
	else
	{
		gpio_configure(GPIO_MLCD_REVE_CP_I2C_SDA, GPIOF_DRIVE_OUTPUT);
		gpio_set_value(GPIO_MLCD_REVE_CP_I2C_SDA, 1);
		gpio_configure(GPIO_MLCD_REVE_CP_I2C_SCL, GPIOF_DRIVE_OUTPUT);
		gpio_set_value(GPIO_MLCD_REVE_CP_I2C_SCL, 1);	
		rc = i2c_register_board_info(I2C_BUS_NUM_MLCD_CHARGE_PUMP, &mlcd_cp_i2c_device, 1);
		mlcd_i2c_platform_device.dev.platform_data=&mlcd_reve_cp_i2c_platform_pdata;
	}

#else
	gpio_configure(GPIO_MLCD_CP_I2C_SDA, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(GPIO_MLCD_CP_I2C_SDA, 1);
	gpio_configure(GPIO_MLCD_CP_I2C_SCL, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(GPIO_MLCD_CP_I2C_SCL, 1);	

	rc = i2c_register_board_info(I2C_BUS_NUM_MLCD_CHARGE_PUMP, &mlcd_cp_i2c_device, 1);
#endif
	printk(KERN_INFO"%s: %d\n",__func__, rc);

	rc =  platform_device_register(&mlcd_i2c_platform_device);

	return;
}
#endif


static void slcd_aat2870gu_init(void)
{
#ifdef BACKLIGHT_RESET
	static int pin_request = 0;

	if(!pin_request)
	{
		if(gpio_request(GPIO_SLCD_CP_EN,"slcd_aat2870 reset"))
		{
			printk(KERN_ERR "slcd aat2870 gpio request is failed : reset");
			return;
		}
		
		gpio_configure(GPIO_SLCD_CP_EN, GPIOF_DRIVE_OUTPUT);
		gpio_set_value(GPIO_SLCD_CP_EN, 1);
		udelay(5);		
	}
#endif
}

/* default is slcd backlight */
static struct aat2870gu_platform_data slcd_aat2870gu_data = {
	.platform_init = slcd_aat2870gu_init,
};

static struct i2c_board_info slcd_cp_i2c_device = {
	I2C_BOARD_INFO("slcd_aat2870gu", AAT2870_DEVICE_I2C_ID),
	.platform_data = &slcd_aat2870gu_data,
};
#if defined (CONFIG_LGE_MACH_ENVT2_CHECK_HW_REV)
static struct i2c_gpio_platform_data slcd_rev1p0_cp_i2c_platform_pdata = {
	.sda_pin = GPIO_SLCD_REV1P0_CP_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_SLCD_REV1P0_CP_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct i2c_gpio_platform_data slcd_reve_cp_i2c_platform_pdata = {
	.sda_pin = GPIO_SLCD_REVE_CP_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_SLCD_REVE_CP_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct i2c_gpio_platform_data slcd_cp_i2c_platform_pdata = {
	.sda_pin = GPIO_SLCD_REV1P0_CP_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_SLCD_REV1P0_CP_I2C_SDA,
	.scl_is_open_drain = 0,
	.udelay = 2,
};
#else
static struct i2c_gpio_platform_data slcd_cp_i2c_platform_pdata = {
	.sda_pin = GPIO_SLCD_CP_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_SLCD_CP_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};
#endif
static struct platform_device slcd_i2c_platform_device = {
	.name = "i2c-gpio",
	.id = I2C_BUS_NUM_SLCD_CHARGE_PUMP,
	.dev.platform_data = &slcd_cp_i2c_platform_pdata,
};
 
void __init init_i2c_slcd_charge_pump(void)
{
	int rc;
	printk(KERN_INFO"%s: \n",__func__);	
#if defined (CONFIG_LGE_MACH_ENVT2_CHECK_HW_REV)

		if (hw_rev1p0)
		{
			gpio_configure(GPIO_SLCD_REV1P0_CP_I2C_SDA, GPIOF_DRIVE_OUTPUT);
			gpio_set_value(GPIO_SLCD_REV1P0_CP_I2C_SDA, 1);
			gpio_configure(GPIO_SLCD_REV1P0_CP_I2C_SCL, GPIOF_DRIVE_OUTPUT);
			gpio_set_value(GPIO_SLCD_REV1P0_CP_I2C_SCL, 1); 
			rc = i2c_register_board_info(I2C_BUS_NUM_SLCD_CHARGE_PUMP, &slcd_cp_i2c_device, 1);
			slcd_i2c_platform_device.dev.platform_data=&slcd_rev1p0_cp_i2c_platform_pdata;
		}
		else
		{
			gpio_configure(GPIO_SLCD_REVE_CP_I2C_SDA, GPIOF_DRIVE_OUTPUT);
			gpio_set_value(GPIO_SLCD_REVE_CP_I2C_SDA, 1);
			gpio_configure(GPIO_SLCD_REVE_CP_I2C_SCL, GPIOF_DRIVE_OUTPUT);
			gpio_set_value(GPIO_SLCD_REVE_CP_I2C_SCL, 1);	
			rc = i2c_register_board_info(I2C_BUS_NUM_SLCD_CHARGE_PUMP, &slcd_cp_i2c_device, 1);
			slcd_i2c_platform_device.dev.platform_data=&slcd_reve_cp_i2c_platform_pdata;
		}
	
#else

	gpio_configure(GPIO_SLCD_CP_I2C_SDA, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(GPIO_SLCD_CP_I2C_SDA, 1);
	gpio_configure(GPIO_SLCD_CP_I2C_SCL, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(GPIO_SLCD_CP_I2C_SCL, 1);	

	rc = i2c_register_board_info(I2C_BUS_NUM_SLCD_CHARGE_PUMP, &slcd_cp_i2c_device, 1);
#endif	
	printk(KERN_INFO"%s: %d\n",__func__, rc);

	rc =  platform_device_register(&slcd_i2c_platform_device);

	return;
}

/* proximity sensor */
#define PROXI_GPIO_I2C_SCL	89
#define PROXI_GPIO_I2C_SDA 	88
#define PROXI_GPIO_DOUT		90//91
#define PROXI_I2C_ADDRESS	0x44 /* slave address 7bit */
//#define PROXI_LDO_NO_VCC	1

/* acceleration */
#define ACCEL_GPIO_INT	 		153
#define ACCEL_GPIO_I2C_SCL  	GPIO_I2C1_SCL 
#define ACCEL_GPIO_I2C_SDA  	GPIO_I2C1_SDA 
#define ACCEL_I2C_ADDRESS		0x18 

/* ecompass */
#define ECOM_GPIO_I2C_SCL		GPIO_I2C2_SCL 
#define ECOM_GPIO_I2C_SDA		GPIO_I2C2_SDA
#define ECOM_GPIO_RST		106
#define ECOM_GPIO_INT		28
#define ECOM_I2C_ADDRESS		0x1c /* slave address 7bit */

int init_gpio_i2c_pin(struct i2c_gpio_platform_data *i2c_adap_pdata,
		struct gpio_i2c_pin gpio_i2c_pin,
		struct i2c_board_info *i2c_board_info_data)
{
	i2c_adap_pdata->sda_pin = gpio_i2c_pin.sda_pin;
	i2c_adap_pdata->scl_pin = gpio_i2c_pin.scl_pin;

	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.sda_pin, 0, GPIO_OUTPUT,
				GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.scl_pin, 0, GPIO_OUTPUT,
				GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	gpio_set_value(gpio_i2c_pin.sda_pin, 1);
	gpio_set_value(gpio_i2c_pin.scl_pin, 1);

	if (gpio_i2c_pin.reset_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.reset_pin, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		gpio_set_value(gpio_i2c_pin.reset_pin, 1);
	}

	if (gpio_i2c_pin.irq_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_INPUT,
					GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
		i2c_board_info_data->irq =
			MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
	}

	return 0;
}


/* ecompass */
static struct gpio_i2c_pin ecom_i2c_pin[] = {
	[0] = {
		.sda_pin	= ECOM_GPIO_I2C_SDA,
		.scl_pin	= ECOM_GPIO_I2C_SCL,
		.reset_pin	= ECOM_GPIO_RST,		
		.irq_pin	= ECOM_GPIO_INT,
	},
};

static struct i2c_gpio_platform_data ecom_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device ecom_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &ecom_i2c_pdata,
};

static int ecom_power_set(unsigned char onoff)
{
	return 0;
}


static s16 m_hlayout[2][9] ={
	{-1, 0, 0, 0, -1, 0, 0, 0, 1},
	{0, 1, 0, -1, 0, 0, 0, 0, 1}
};

static s16 m_alayout[2][9] = {
	{1, 0, 0, 0, 1, 0, 0, 0, 1},
	{0, -1, 0, 1, 0, 0, 0, 0, 1}
};

static struct ecom_platform_data ecom_pdata = {
        .pin_int        = ECOM_GPIO_INT,
        .pin_rst        = ECOM_GPIO_RST,
        .power          = ecom_power_set,
        .accelerator_name = "KR3DH",
        .fdata_sign_x = 1,
        .fdata_sign_y = 1,
        .fdata_sign_z = -1,
        .fdata_order0 = 1,
        .fdata_order1 = 0,
        .fdata_order2 = 2,
        .sensitivity1g = 1024,
        .h_layout = (s16 *)m_hlayout,
        .a_layout = (s16 *)m_alayout,
};

static struct i2c_board_info ecom_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("akm8973", ECOM_I2C_ADDRESS),
		.type = "akm8973",
		.platform_data = &ecom_pdata,
	}
};

static void __init vs760_init_i2c_ecompass(void)
{
	ecom_i2c_device.id = I2C_BUS_NUM_SENSOR_COMPASS;

	init_gpio_i2c_pin(&ecom_i2c_pdata, ecom_i2c_pin[0], &ecom_i2c_bdinfo[0]);

	i2c_register_board_info(ecom_i2c_device.id, &ecom_i2c_bdinfo[0], 1);
	platform_device_register(&ecom_i2c_device);
}


/* gp2ap proximity sensor */
static struct gpio_i2c_pin proxi_i2c_pin[] = {
	[0] = {
		.sda_pin	= PROXI_GPIO_I2C_SDA,
		.scl_pin	= PROXI_GPIO_I2C_SCL,
		.reset_pin	= 0,		
		.irq_pin	= PROXI_GPIO_DOUT,
	},
};

static struct i2c_gpio_platform_data proxi_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device proxi_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &proxi_i2c_pdata,
};

static int prox_power_set(unsigned char onoff)
{
	int ret = 0;
	struct vreg *gp5_vreg = vreg_get(0, "gp5");

	if (onoff) {
		vreg_set_level(gp5_vreg, 2800);
		vreg_enable(gp5_vreg);
		msleep(10);
	} else {
		vreg_set_level(gp5_vreg, 0);
		vreg_disable(gp5_vreg);
	}

	return ret;
}

static struct proximity_platform_data proxi_pdata = {
	.irq_num	= PROXI_GPIO_DOUT,
	.power		= prox_power_set,
	.methods		= 0x0,
	.operation_mode		= 0x0,
	.debounce	 = 0,
	.cycle = 0,
};

static struct i2c_board_info proxi_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("proximity_gp2ap", PROXI_I2C_ADDRESS),
		.type = "proximity_gp2ap",
		.platform_data = &proxi_pdata,
	},
};


static void __init vs760_init_i2c_proximity(void)
{
	proxi_i2c_device.id = I2C_BUS_NUM_PROXI;

	init_gpio_i2c_pin(&proxi_i2c_pdata, proxi_i2c_pin[0], &proxi_i2c_bdinfo[0]);

	i2c_register_board_info(proxi_i2c_device.id, &proxi_i2c_bdinfo[0], 1);
	platform_device_register(&proxi_i2c_device);
}


static int kr3dh_config_gpio(int config)
{
	if (config) {	// for wake state
		gpio_tlmm_config(GPIO_CFG(ACCEL_GPIO_I2C_SCL, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(ACCEL_GPIO_I2C_SDA, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(ACCEL_GPIO_INT, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
	} else {		// for sleep state
		gpio_tlmm_config(GPIO_CFG(ACCEL_GPIO_I2C_SCL, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(ACCEL_GPIO_I2C_SDA, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(ACCEL_GPIO_INT, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
	}

	return 0;
}
static int kr_init(void){return 0;}
static void kr_exit(void){}
static int power_on(void){return 0;}
static int power_off(void){return 0;}

struct kr3dh_platform_data kr3dh_data = {
	.poll_interval = 100,
	.min_interval = 0,
	.g_range = 0x00,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,

	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,

	.power_on = power_on,
	.power_off = power_off,
	.kr_init = kr_init,
	.kr_exit = kr_exit,
	.gpio_config = kr3dh_config_gpio,
};


static struct gpio_i2c_pin accel_i2c_pin[] = {
	[0] = {
		.sda_pin	= ACCEL_GPIO_I2C_SDA,
		.scl_pin	= ACCEL_GPIO_I2C_SCL,
		.reset_pin	= 0,		
		.irq_pin	= ACCEL_GPIO_INT,
	},
};

static struct i2c_gpio_platform_data accel_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device accel_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &accel_i2c_pdata,
};


static struct i2c_board_info accel_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("KR3DH", ACCEL_I2C_ADDRESS),
		.type = "KR3DH",
		.platform_data = &kr3dh_data,
	}
};

static void __init vs760_init_i2c_acceleration(void)
{
	accel_i2c_device.id = I2C_BUS_NUM_SENSOR_ACCEL;

	init_gpio_i2c_pin(&accel_i2c_pdata, accel_i2c_pin[0], &accel_i2c_bdinfo[0]);

	i2c_register_board_info(accel_i2c_device.id, &accel_i2c_bdinfo[0], 1);
	platform_device_register(&accel_i2c_device);
}
/*-----------------------------------------------------------------------*/
/* LG_FW : 2009.09.19 cis - using gpio-i2c */
#ifdef LG_FW_CAMERA_USE_GPIO_I2C

/* For Backlight */
#define GPIO_CAMERA_I2C_SCL 60
#define GPIO_CAMERA_I2C_SDA 61
#define GPIO_CAMERA_RESET 2

// The I2C Slave Address of MT9T111
#define MT9T111_I2C_ADDRESS			0x78

struct mt9t111_camera_platform_data {
	void	(*platform_init)(void);
	unsigned int op_mode;
	spinlock_t lock;
};

static struct i2c_board_info camera_i2c_device = {
	I2C_BOARD_INFO("mt9t111", MT9T111_I2C_ADDRESS>>1),
};

static struct i2c_gpio_platform_data camera_i2c_adap_pdata = {
	.sda_pin = GPIO_CAMERA_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_CAMERA_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device vs760_camera_i2c_adap_device = {
	.name = "i2c-gpio",
	.id = I2C_BUS_NUM_CAMERA,
	.dev.platform_data = &camera_i2c_adap_pdata,
};

void __init vs760_init_i2c_camera(void)
{
	int rc;
	printk(KERN_INFO"%s: \n",__func__);	

	gpio_configure(GPIO_CAMERA_I2C_SDA, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(GPIO_CAMERA_I2C_SDA, 1);
	gpio_configure(GPIO_CAMERA_I2C_SCL, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(GPIO_CAMERA_I2C_SCL, 1);

	rc = i2c_register_board_info(I2C_BUS_NUM_CAMERA, &camera_i2c_device, 1);
	printk(KERN_INFO"%s: %d\n",__func__, rc);	

	rc = platform_device_register(&vs760_camera_i2c_adap_device);

	return;
}
#endif
#ifdef CONFIG_LGE_MACH_ENVT2_REVC //LG_FW : 2010.03.14 woochang.chun
#define CAMISP_RESET_N 0
#define CAMISP_PM_RESET_N  1
#define ISP_HOST_STBY 2
#define GPIO_CAM_POWER_I2C_SCL GPIO_I2C2_SCL	//I2C2_SCL
#define GPIO_CAM_POWER_I2C_SDA GPIO_I2C2_SDA	//I2C2_SDA
#define BH6172GU_DEVICE_I2C_ID 0x9E
#define CE147_DEVICE_I2C_ID 0x78 

static struct i2c_gpio_platform_data bh6172_i2c_pdata = {
	.sda_pin = GPIO_CAM_POWER_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_CAM_POWER_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device bh6172_i2c_device = {
	.name = "i2c-gpio",
	.id = I2C_BUS_NUM_CAMERA_POWER,
	.dev.platform_data = &bh6172_i2c_pdata,
};


static struct i2c_board_info bh6172_i2c_bdinfo = {
	I2C_BOARD_INFO("bh6172", BH6172GU_DEVICE_I2C_ID>>1 ),
};

void __init bh6172_init_i2c_subpm(void)
{
	gpio_tlmm_config(GPIO_CFG(CAMISP_PM_RESET_N, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);	
	gpio_configure(CAMISP_PM_RESET_N, GPIOF_DRIVE_OUTPUT); 
	gpio_set_value(CAMISP_PM_RESET_N, 0);

	gpio_tlmm_config(GPIO_CFG(GPIO_CAM_POWER_I2C_SCL, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE ) ;
	gpio_tlmm_config(GPIO_CFG(GPIO_CAM_POWER_I2C_SDA, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE ) ;

	platform_device_register(&bh6172_i2c_device); 
	i2c_register_board_info(I2C_BUS_NUM_CAMERA_POWER, &bh6172_i2c_bdinfo, 1);
}

void __init ce147_init_i2c_camisp(void)
{
	printk(KERN_ERR "%s: ce147_init_i2c_camisp start.. \n",__func__);

	gpio_tlmm_config(GPIO_CFG(CAMISP_RESET_N, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);	
    gpio_configure(CAMISP_RESET_N, GPIOF_DRIVE_OUTPUT); 
    gpio_set_value(CAMISP_RESET_N, 0);

	gpio_tlmm_config(GPIO_CFG(ISP_HOST_STBY, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);		
	gpio_configure(ISP_HOST_STBY, GPIOF_DRIVE_OUTPUT); 
	gpio_set_value(ISP_HOST_STBY, 0);

	printk(KERN_ERR "%s: ce147_init_i2c_camisp end.. \n",__func__);
}
#endif //CONFIG_LGE_MACH_ENVT2_REVC 
//-----------------------------------------------------------------------
#ifdef CONFIG_LGE_AUDIO_SUBSYSTEM

#define LM49250_DEVICE_I2C_ID 0xF8
#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
#define GPIO_AMP_I2C_SDA 134
#define GPIO_AMP_I2C_SCL 133
#elif defined (CONFIG_LGE_MACH_ENVT2_REVC)

#define GPIO_AMP_I2C_SDA GPIO_I2C1_SDA
#define GPIO_AMP_I2C_SCL GPIO_I2C1_SCL
#else

#define GPIO_AMP_I2C_SDA GPIO_I2C1_SDA
#define GPIO_AMP_I2C_SCL GPIO_I2C1_SCL
#endif
struct lm49250_platform_data {
	void	(*platform_init)(void);
	unsigned int op_mode;
	spinlock_t lock;
};

static void lm49250_init(void)
{
      mdelay(1000);
}

static struct  lm49250_platform_data  lm49250_data = {
	.platform_init = lm49250_init,
};


static struct i2c_gpio_platform_data amp_i2c_pdata = {
	.sda_pin = GPIO_AMP_I2C_SDA,
	.sda_is_open_drain = 0,
	.scl_pin = GPIO_AMP_I2C_SCL,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device amp_i2c_device = {
	.name = "i2c-gpio",
	.id = I2C_BUS_NUM_AMP,
	.dev.platform_data = &amp_i2c_pdata,
};

static struct i2c_board_info amp_i2c_bdinfo = {
	I2C_BOARD_INFO("amp_lm49250", LM49250_DEVICE_I2C_ID>>1 ),
	.platform_data = &lm49250_data,
};

void __init init_i2c_Amp(void)
{
#if 0
	gpio_configure(GPIO_AMP_I2C_SDA, GPIOF_INPUT);
	gpio_configure(GPIO_AMP_I2C_SCL, GPIOF_INPUT);
#endif
	int rc , ret;
#if 1
	ret = gpio_tlmm_config(GPIO_CFG(GPIO_AMP_I2C_SDA,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
	if (ret) {
			printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",__func__, ret);
	}
	ret = gpio_tlmm_config(GPIO_CFG(GPIO_AMP_I2C_SCL,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),GPIO_ENABLE);
	if (ret) {
			printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",__func__, ret);
	}
#endif 

	gpio_configure(GPIO_AMP_I2C_SDA, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(GPIO_AMP_I2C_SDA, 1);
	gpio_configure(GPIO_AMP_I2C_SCL, GPIOF_DRIVE_OUTPUT);
	gpio_set_value(GPIO_AMP_I2C_SCL, 1);
	
	
	rc = i2c_register_board_info(I2C_BUS_NUM_AMP, &amp_i2c_bdinfo, 1);

	rc = platform_device_register(&amp_i2c_device);

	return;
}
#endif

gpio_i2c_init_func_t *i2c_init_func[] = {	
#ifdef CONFIG_LGE_MACH_ENVT2_REVC
	init_i2c_mlcd_touch,
	init_i2c_slcd_touch,	
#endif
#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
	init_i2c_mlcd_charge_pump,
	init_i2c_slcd_charge_pump,
#else
	//init_i2c_mlcd_charge_pump,
	init_i2c_slcd_charge_pump,
#endif
//-----------------------------------------------------------------------
//LG_FW : 2009.09.19 cis - using gpio-i2c
#ifdef LG_FW_CAMERA_USE_GPIO_I2C
	vs760_init_i2c_camera,
#endif
//-----------------------------------------------------------------------
#if defined (CONFIG_LGE_MACH_ENVT2_REVD)
#ifdef CONFIG_LGE_AUDIO_SUBSYSTEM
	init_i2c_Amp,
#endif /*CONFIG_LGE_AUDIO_SUBSYSTEM*/
#else
#ifdef CONFIG_LGE_AUDIO_SUBSYSTEM
	init_i2c_Amp,
#endif /*CONFIG_LGE_AUDIO_SUBSYSTEM*/
#endif
	vs760_init_i2c_proximity,
	vs760_init_i2c_acceleration,
	vs760_init_i2c_ecompass,
	NULL,
};

void __init vs760_init_gpio_i2c_devices(void)
{
	gpio_i2c_init_func_t **init_func_ptr;

	for (init_func_ptr = i2c_init_func ; *init_func_ptr ; ++init_func_ptr) {
		(*init_func_ptr)();
	}

	return;
}

MODULE_AUTHOR("cleaneye@lge.com>");
MODULE_DESCRIPTION("LGE gpio-12c driver");
MODULE_LICENSE("GPL");
