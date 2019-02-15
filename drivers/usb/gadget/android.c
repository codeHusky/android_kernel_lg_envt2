/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/miscdevice.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

#include <linux/usb/android.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#if defined(CONFIG_USB_ANDROID_CDC_ECM) || defined(CONFIG_USB_ANDROID_RNDIS)
#include "u_ether.h"
#endif

#include "f_mass_storage.h"
#include "f_adb.h"
#include "u_serial.h"
#ifdef CONFIG_USB_ANDROID_DIAG
#include "f_diag.h"
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
#include "f_rmnet.h"
#endif

#include "gadget_chips.h"
#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
#include <mach/rpc_hsusb.h>
#endif
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
#include "f_mtp.h"
#endif

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

/* product id */
#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
u16 product_id;
int lg_cable_type = 0;
#else
static u16 product_id = 0x618F;
#endif

static int android_set_pid(const char *val, struct kernel_param *kp);
static int android_get_pid(char *buffer, struct kernel_param *kp);
module_param_call(product_id, android_set_pid, android_get_pid,
					&product_id, 0664);
MODULE_PARM_DESC(product_id, "USB device product id");

/* serial number */
#define MAX_SERIAL_LEN 256

#ifdef CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB
static char serial_number[MAX_SERIAL_LEN] = "\0";
#else
static char serial_number[MAX_SERIAL_LEN] = "1234567890ABCDEF";
#endif

#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
const u16 lg_autorun_pid = 0x61E2;
static char autorun_serial_number[MAX_SERIAL_LEN] = "LGANDROIDUS760";
static u16 autorun_user_mode;
static int android_set_usermode(const char *val, struct kernel_param *kp);
module_param_call(user_mode, android_set_usermode, param_get_string,
					&autorun_user_mode, 0664);
MODULE_PARM_DESC(user_mode, "USB Autorun user mode");
#endif

static struct kparam_string kps = {
	.string			= serial_number,
	.maxlen			= MAX_SERIAL_LEN,
};
static int android_set_sn(const char *kmessage, struct kernel_param *kp);
module_param_call(serial_number, android_set_sn, param_get_string,
						&kps, 0664);
MODULE_PARM_DESC(serial_number, "SerialNumber string");

static const char longname[] = "Gadget Android";
#if defined(CONFIG_USB_ANDROID_CDC_ECM) || defined(CONFIG_USB_ANDROID_RNDIS)
static u8 hostaddr[ETH_ALEN];
#endif

/* Default vendor ID, overridden by platform data */
#define VENDOR_ID		0x18D1

struct android_dev {
	struct usb_gadget *gadget;
	struct usb_composite_dev *cdev;

	int version;

	int adb_enabled;
	int nluns;
	struct mutex lock;
	struct android_usb_platform_data *pdata;
	unsigned long functions;
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
    int mtp_enabled;
#endif
};

static struct android_dev *_android_dev;

#if defined (CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
#if defined (CONFIG_LGE_USB_SUPPORT_SMEM_CABLE_TYPE)
extern int lgfw_smem_cable_type(void);
#else
/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-06-23, [VS760] */
/* to supports FS USB in the Factory ( LT cable will be connected) */
extern int msm_chg_LG_cable_type(void);
#endif
extern void msm_get_MEID_type(char* sMeid);
static struct usb_composition *android_validate_product_id(unsigned short pid);

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-06-23, [VS760]  */
/* to supports FS USB in the Factory ( LT cable will be connected) */
#define LG_FACTORY_CABLE_TYPE 3
#define LG_FACTORY_CABLE_130K_TYPE 10
#define LT_ADB_CABLE 0xff
const u16 lg_factory_pid = 0x6000;
const u16 lg_android_pid = 0x618E;
const u16 lg_ums_pid = 0x61E1;
#endif

#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB) || defined(CONFIG_LGE_USB_GADGET_MTP_DRIVER)
/* VS760 MTP PID */
const u16 lg_mtp_pid = 0x61E0;
int mtp_enable_flag = 0;
#endif

#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
static const char mtp_product_name [] = "US760 MTP Sync";
static const char lg_product_name [] = "LG Android USB Device";
#endif
/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

#define android_func_attr(function, index)				\
static ssize_t  show_##function(struct device *dev,			\
		struct device_attribute *attr, char *buf)		\
{									\
									\
	unsigned long n = _android_dev->functions;			\
	int val = 0;							\
									\
	while (n) {							\
		if ((n & 0x0F) == index)				\
			val = 1;					\
		n = n >> 4;						\
	}								\
	return sprintf(buf, "%d\n", val);				\
									\
}									\
									\
static DEVICE_ATTR(function, S_IRUGO, show_##function, NULL);

android_func_attr(adb, ANDROID_ADB);
android_func_attr(mass_storage, ANDROID_MSC);
android_func_attr(acm_modem, ANDROID_ACM_MODEM);
android_func_attr(acm_nmea, ANDROID_ACM_NMEA);
android_func_attr(diag, ANDROID_DIAG);
android_func_attr(modem, ANDROID_GENERIC_MODEM);
android_func_attr(nmea, ANDROID_GENERIC_NMEA);
android_func_attr(cdc_ecm, ANDROID_CDC_ECM);
android_func_attr(rmnet, ANDROID_RMNET);
android_func_attr(rndis, ANDROID_RNDIS);
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
android_func_attr(mtp, ANDROID_MTP);
#endif
#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
android_func_attr(autorun, ANDROID_AUTORUN);
#endif

static struct attribute *android_func_attrs[] = {
	&dev_attr_adb.attr,
	&dev_attr_mass_storage.attr,
	&dev_attr_acm_modem.attr,
	&dev_attr_acm_nmea.attr,
	&dev_attr_diag.attr,
	&dev_attr_modem.attr,
	&dev_attr_nmea.attr,
	&dev_attr_cdc_ecm.attr,
	&dev_attr_rmnet.attr,
	&dev_attr_rndis.attr,
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
	&dev_attr_mtp.attr,
#endif
#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
	&dev_attr_autorun.attr,
#endif
	NULL,
};

static struct attribute_group android_func_attr_grp = {
	.name  = "functions",
	.attrs = android_func_attrs,
};

static int  android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = -EINVAL;
	unsigned long n;
	pr_debug("android_bind_config c = 0x%x dev->cdev=0x%x\n",
		(unsigned int) c, (unsigned int) dev->cdev);
	n = dev->functions;
	while (n) {
		switch (n & 0x0F) {
		case ANDROID_ADB:
			ret = adb_function_add(dev->cdev, c);
			if (ret)
				return ret;
			break;
		case ANDROID_MSC:
#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
			/* NOTE : assume that nluns is 1 */
			ret = mass_storage_function_add(dev->cdev, c, dev->nluns, 0);
#else
			ret = mass_storage_function_add(dev->cdev, c, dev->nluns);
#endif			
			if (ret)
				return ret;
			break;
		case ANDROID_ACM_MODEM:
			ret = acm_bind_config(c, 0);
			if (ret)
				return ret;
			break;
		case ANDROID_ACM_NMEA:
			ret = acm_bind_config(c, 1);
			if (ret)
				return ret;
			break;
#ifdef CONFIG_USB_ANDROID_DIAG
		case ANDROID_DIAG:
			ret = diag_function_add(c, serial_number);
			if (ret)
				return ret;
			break;
#endif
#ifdef CONFIG_USB_F_SERIAL
		case ANDROID_GENERIC_MODEM:
			ret = gser_bind_config(c, 0);
			if (ret)
				return ret;
			break;
		case ANDROID_GENERIC_NMEA:
			ret = gser_bind_config(c, 1);
			if (ret)
				return ret;
			break;
#endif
#ifdef CONFIG_USB_ANDROID_CDC_ECM
		case ANDROID_CDC_ECM:
			ret = ecm_bind_config(c, hostaddr);
			if (ret)
				return ret;
			break;
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
		case ANDROID_RMNET:
			ret = rmnet_function_add(c);
			if (ret) {
				pr_err("failed to add rmnet function\n");
				return ret;
			}
			break;
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
		case ANDROID_RNDIS:
			ret = rndis_bind_config(c, hostaddr);
			if (ret)
				return ret;
			break;
#endif
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
        case ANDROID_MTP:
            ret = mtp_function_add(dev->cdev, c);
            if (ret)
               return ret;
            break;
#endif
#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
		case ANDROID_AUTORUN:
			/* NOTE : assume that nluns is 1 */
			ret = mass_storage_function_add(dev->cdev, c, dev->nluns, 1);
			if (ret)
				return ret;
			break;
#endif			
		default:
			ret = -EINVAL;
			return ret;
		}
		n = n >> 4;
	}
	return ret;

}

static int get_num_of_serial_ports(void)
{
	struct android_dev *dev = _android_dev;
	unsigned long n = dev->functions;
	unsigned ports = 0;

	while (n) {
		switch (n & 0x0F) {
		case ANDROID_ACM_MODEM:
		case ANDROID_ACM_NMEA:
		case ANDROID_GENERIC_MODEM:
		case ANDROID_GENERIC_NMEA:
			ports++;
		}
		n = n >> 4;
	}

	return ports;
}

#ifdef CONFIG_LGE_USB_GADGET_DRIVER
static int is_iad_enabled(void)
{
	struct android_dev *dev = _android_dev;
	unsigned long n = dev->functions;

	while (n) {
		switch (n & 0x0F) {
			case ANDROID_ACM_MODEM:
			case ANDROID_ACM_NMEA:
				return 1;
#ifdef CONFIG_USB_ANDROID_RNDIS
			case ANDROID_RNDIS:
				return 2;
#endif
#ifdef CONFIG_USB_ANDROID_CDC_ECM
			case ANDROID_CDC_ECM:
				return 2;
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
			case ANDROID_RMNET:
				return 2;
#endif
		}
		n = n >> 4;
	}

	return 0;
}
#else /* below is original */
static int is_iad_enabled(void)
{
	struct android_dev *dev = _android_dev;
	unsigned long n = dev->functions;

	while (n) {
		switch (n & 0x0F) {
		case ANDROID_ACM_MODEM:
		case ANDROID_ACM_NMEA:
#ifdef CONFIG_USB_ANDROID_RNDIS
		case ANDROID_RNDIS:
#endif
			return 1;
		}
		n = n >> 4;
	}

	return 0;
}
#endif

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.bConfigurationValue = 1,
	.bMaxPower	= 0xFA, /* 500ma */
};

static int android_unbind(struct usb_composite_dev *cdev)
{
	if (get_num_of_serial_ports())
		gserial_cleanup();

	return 0;
}

#ifdef CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB
static int do_get_usb_serial_number(char *serial_number)
{
/* LGE_CHANGES_S [younsuk.song@lge.com] 2010-06-21, Set MEID */
	memset(serial_number, 0, MAX_SERIAL_LEN);

	msm_get_MEID_type(serial_number);

	if(!strcmp(serial_number,"00000000000000")) 
		serial_number[0] = '\0';

#if defined(CONFIG_LGE_USB_SUPPORT_SMEM_CABLE_TYPE)
	if(lgfw_smem_cable_type() == LT_ADB_CABLE)
#else
	if(msm_chg_LG_cable_type() == LT_ADB_CABLE)
#endif
	{
		sprintf(serial_number,"%s","LGE_ANDROID_DE");
	}

	return 0;
/* LGE_CHANGES_E [younsuk.song@lge.com] 2010-06-21 */
}
/*
 * lge_get_usb_serial_number
 *
 * Get USB serial number from ARM9 using RPC or RAPI.
 * return -1 : If any error
 * return 0 : success
 *
 */
int lge_get_usb_serial_number(char *serial_number)
{
	return do_get_usb_serial_number(serial_number);
}
EXPORT_SYMBOL(lge_get_usb_serial_number);

#endif /*CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB*/

static int  android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum;
	int			id;
	int			ret;
	int                     num_ports;
#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
    int tempCableID;
	struct usb_composition *func;
#endif

	pr_debug("android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
#if defined(CONFIG_LGE_USB_SUPPORT_SMEM_CABLE_TYPE)
    tempCableID = lgfw_smem_cable_type();
#else
    tempCableID = msm_chg_LG_cable_type();
#endif
    lg_cable_type = tempCableID;

    if( tempCableID == LG_FACTORY_CABLE_TYPE || tempCableID == LG_FACTORY_CABLE_130K_TYPE)  //detect LT cable
    {
        product_id = lg_factory_pid;
        func = android_validate_product_id(product_id);
        dev->functions = func->functions;
    }
#endif /*CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB*/

/* LGE_CHANGES_S [jaeho.cho@lge.com] 2010-08-16, Autorun Serial Number */
#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
    if(product_id == lg_autorun_pid)
    {      
	  strings_dev[STRING_SERIAL_IDX].s = autorun_serial_number;
    }
	else
    {
	  strings_dev[STRING_SERIAL_IDX].s = serial_number;
	}
#endif

	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	/* Supporting remote wakeup for mass storage only function
	 * does n't make sense, since there are no notifications that
	 * can be sent from mass storage during suspend */
	if ((gadget->ops->wakeup) && (dev->functions != ANDROID_MSC))
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	else
		android_config_driver.bmAttributes &= ~USB_CONFIG_ATT_WAKEUP;

	if (dev->pdata->self_powered && !usb_gadget_set_selfpowered(gadget)) {
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_SELFPOWER;
		android_config_driver.bMaxPower	= 0x32; /* 100 mA */
	}
	dev->cdev = cdev;
	pr_debug("android_bind assigned dev->cdev\n");
	dev->gadget = gadget;

#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
	if (product_id == lg_factory_pid) {
		serial_number[0] = '\0';
		msm_hsusb_is_serial_num_null(1); 
		device_desc.iSerialNumber = 0; 
	} else {
		ret = lge_get_usb_serial_number(serial_number);

		/* Send Serial number to A9 for software download */
		if (serial_number[0] != '\0') {
			msm_hsusb_is_serial_num_null(0);
			msm_hsusb_send_serial_number(serial_number);
		} else {
			/* If error to get serial number, we check the
			 * pdata's serial number. If pdata's serial number
			 * (e.g. default serial number) is set, we use the 
			 * serial number.
			 */
			if (dev->pdata->serial_number == NULL) {
				msm_hsusb_is_serial_num_null(1); 
				device_desc.iSerialNumber = 0; 
			} else {
				sprintf(serial_number, "%s", dev->pdata->serial_number);
				msm_hsusb_is_serial_num_null(0);
				msm_hsusb_send_serial_number(serial_number);
			}
		}
	}

	msm_hsusb_send_productID(product_id);

	android_config_driver.bmAttributes |= USB_CONFIG_ATT_SELFPOWER;
	android_config_driver.bMaxPower = 0xFA; /* 500 mA */
#endif /*CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB*/

	num_ports = get_num_of_serial_ports();
	if (num_ports) {
		ret = gserial_setup(cdev->gadget, num_ports);
		if (ret < 0)
			return ret;
	}

	/* Android user space allows USB tethering only when usb0 is listed
	 * in network interfaces. Setup network link though RNDIS/CDC-ECM
	 * is not listed in current composition. Network links is not setup
	 * for every composition switch. It is setup one time and teared down
	 * during module removal.
	 */
#if defined(CONFIG_USB_ANDROID_CDC_ECM) || defined(CONFIG_USB_ANDROID_RNDIS)
	/* set up network link layer */
	ret = gether_setup(cdev->gadget, hostaddr);
	if (ret && (ret != -EBUSY)) {
		gserial_cleanup();
		return ret;
	}
#endif

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		pr_err("usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

#ifdef CONFIG_LGE_USB_GADGET_DRIVER
	ret = is_iad_enabled();
	switch(ret) {
		case 1 : /* CDC Class --> USB_CLASS_COMM */
			device_desc.bDeviceClass         = USB_CLASS_COMM;
			device_desc.bDeviceSubClass      = 0x00;
			device_desc.bDeviceProtocol      = 0x00;
			break;
		case 2 : /* ECM Class --> USB_CLASS_MISC */
			device_desc.bDeviceClass         = USB_CLASS_MISC;
			device_desc.bDeviceSubClass      = 0x02;
			device_desc.bDeviceProtocol      = 0x01;
			break;
		default: /* Others */
			device_desc.bDeviceClass         = USB_CLASS_PER_INTERFACE;
			device_desc.bDeviceSubClass      = 0;
			device_desc.bDeviceProtocol      = 0;
			break;
	}
#else /* below is original */	
	if (is_iad_enabled()) {
		device_desc.bDeviceClass         = USB_CLASS_MISC;
		device_desc.bDeviceSubClass      = 0x02;
		device_desc.bDeviceProtocol      = 0x01;
	} else {
		device_desc.bDeviceClass         = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceSubClass      = 0;
		device_desc.bDeviceProtocol      = 0;
	}
#endif /*CONFIG_LGE_USB_GADGET_DRIVER*/

	pr_debug("android_bind done\n");
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.unbind		= android_unbind,
};

struct usb_composition *android_validate_product_id(unsigned short pid)
{
	struct android_dev *dev = _android_dev;
	struct usb_composition *fi;
	int i;

	for (i = 0; i < dev->pdata->num_compositions; i++) {
		fi = &dev->pdata->compositions[i];
		pr_debug("pid=0x%x apid=0x%x\n",
		       fi->product_id, fi->adb_product_id);
		if ((fi->product_id == pid) || (fi->adb_product_id == pid))
			return fi;
	}
	return NULL;
}

#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
int android_switch_composition(u16 pid)
#else
static int android_switch_composition(u16 pid)
#endif
{
	struct android_dev *dev = _android_dev;
	struct usb_composition *func;
	int ret;

	/* Validate the prodcut id */
	func = android_validate_product_id(pid);
	if (!func) {
		pr_err("%s: invalid product id %x\n", __func__, pid);
		return -EINVAL;
	}

	/* Honour adb users */
	if (dev->adb_enabled) {
		product_id = func->adb_product_id;
		dev->functions = func->adb_functions;
	} else {
		product_id = func->product_id;
		dev->functions = func->functions;
	}
	
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
    if(product_id == lg_mtp_pid)
    {
        dev->mtp_enabled = 1;
		mtp_enable_flag = 1;
    }
	else
	{
	    dev->mtp_enabled = 0;
		mtp_enable_flag = 0;
	}
	
	if(dev->mtp_enabled)
	{
		/* set mtp product name */
		strings_dev[STRING_PRODUCT_IDX].s = mtp_product_name;
	}
	else
	{
		strings_dev[STRING_PRODUCT_IDX].s = lg_product_name;
	}
#endif

	usb_composite_unregister(&android_usb_driver);
	ret = usb_composite_register(&android_usb_driver);

	return ret;
}

static ssize_t android_remote_wakeup(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_gadget *gadget = _android_dev->gadget;

	if (!gadget)
		return -ENODEV;

	pr_debug("Calling remote wakeup....\n");
	usb_gadget_wakeup(gadget);

	return count;
}
static DEVICE_ATTR(remote_wakeup, S_IWUSR, 0, android_remote_wakeup);

static struct attribute *android_attrs[] = {
	&dev_attr_remote_wakeup.attr,
	NULL,
};

static struct attribute_group android_attr_grp = {
	.attrs = android_attrs,
};

#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
static int android_set_usermode(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	unsigned long tmp;

	ret = strict_strtoul(val, 16, &tmp);
	if (ret)
		return ret;

	autorun_user_mode = (unsigned int)tmp;
	pr_info("autorun user mode : %d\n", autorun_user_mode);

	return ret;
}

int get_autorun_user_mode(void)
{
	return autorun_user_mode;
}
EXPORT_SYMBOL(get_autorun_user_mode);
#endif

static int android_set_sn(const char *kmessage, struct kernel_param *kp)
{
	int len = strlen(kmessage);

	if (len >= MAX_SERIAL_LEN) {
		pr_err("serial number string too long\n");
		return -ENOSPC;
	}

	strlcpy(serial_number, kmessage, MAX_SERIAL_LEN);
	/* Chop out \n char as a result of echo */
	if (serial_number[len - 1] == '\n')
		serial_number[len - 1] = '\0';

	return 0;
}

static int android_set_pid(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	unsigned long tmp;

	ret = strict_strtoul(val, 16, &tmp);
	if (ret)
		goto out;

	/* We come here even before android_probe, when product id
	 * is passed via kernel command line.
	 */
	if (!_android_dev) {
		product_id = tmp;
		goto out;
	}

#ifdef CONFIG_LGE_USB_GADGET_DRIVER
/* This prevents from mode switching twice by init process */
	if (product_id == tmp) {
		pr_info("%s : Requested product id is same(%lx), ignore it\n", 
				__func__, tmp);
		goto out;
	}
	/* If cable is factory cable, we ignore request from user space */
	if (product_id == lg_factory_pid) {
		pr_info("%s : Factory USB cable is connected, ignore it\n", __func__);
		goto out;
	}
#endif

	mutex_lock(&_android_dev->lock);
	ret = android_switch_composition(tmp);
	mutex_unlock(&_android_dev->lock);
out:
	return ret;
}

static int android_get_pid(char *buffer, struct kernel_param *kp)
{
	int ret;

	mutex_lock(&_android_dev->lock);
	ret = sprintf(buffer, "%x", product_id);
	mutex_unlock(&_android_dev->lock);

	return ret;
}

#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
static int mtp_enable_open(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
    if(product_id == 0x6000 || product_id == 0x6001)
    {
      dev->mtp_enabled = 1;
      return 0;
    }
#endif
       mtp_enable_flag = 1;
	mutex_lock(&dev->lock);

	if (dev->mtp_enabled)
		goto out;

	dev->mtp_enabled = 1;
    product_id = lg_mtp_pid; //lg mtp pid
    
	printk(KERN_INFO "mtp_enable_open mtp\n");
	if (product_id)
		ret = android_switch_composition(product_id);
out:
	mutex_unlock(&dev->lock);

	return ret;
}

static int mtp_enable_release(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;
#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
    if(product_id == 0x6000 || product_id == 0x6001)
    {
      dev->mtp_enabled = 0;
      return 0;
    }
#endif
      mtp_enable_flag = 0;
	mutex_lock(&dev->lock);

	if (!dev->mtp_enabled)
		goto out;

	printk(KERN_INFO "mtp_enable_release mtp\n");
	dev->mtp_enabled = 0;
    product_id = lg_android_pid; //lg android pid

	if (product_id)
		ret = android_switch_composition(product_id);
out:
	mutex_unlock(&dev->lock);

	return ret;
}

static struct file_operations mtp_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    mtp_enable_open,
	.release = mtp_enable_release,
};

static struct miscdevice mtp_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mtp_enable",
	.fops = &mtp_enable_fops,
};

#endif /*CONFIG_LGE_USB_GADGET_MTP_DRIVER*/

static int adb_enable_open(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;
#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
	if (product_id == lg_autorun_pid) {
		pr_info("%s: adb enabling on Autorun mode, Ignore it\n",
				__func__);

		dev->adb_enabled = 1;
		/* intended error trigger */
		return -1;
	}
#endif

#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB) || defined(CONFIG_LGE_USB_GADGET_MTP_DRIVER)
    if(product_id == 0x6000 || product_id == 0x6001 || product_id == lg_mtp_pid)
    {
      dev->adb_enabled = 1;
      return 0;
    }
#endif
	mutex_lock(&dev->lock);

	if (dev->adb_enabled)
		goto out;

	dev->adb_enabled = 1;
	pr_debug("enabling adb\n");

#ifdef CONFIG_LGE_USB_GADGET_DRIVER
	if (product_id == lg_ums_pid) {
		pr_info("%s: adb enabling on UMS only mode, enforce to switch ADB\n",
				__func__);
		product_id = lg_android_pid;
	}
#endif
	if (product_id)
		ret = android_switch_composition(product_id);
out:
	mutex_unlock(&dev->lock);

	return ret;
}

static int adb_enable_release(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;
#ifdef CONFIG_LGE_USB_SUPPORT_ANDROID_AUTORUN
	if (product_id == lg_autorun_pid ) {
		pr_info("%s: adb disabling on Autorun mode, Ignore it\n",
				__func__);

		dev->adb_enabled = 0;
		/* intended error trigger */
		return -1;
	}
#endif
#if defined(CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB) || defined(CONFIG_LGE_USB_GADGET_MTP_DRIVER)
    if(product_id == 0x6000 || product_id == 0x6001 || product_id == lg_mtp_pid)
    {
      dev->adb_enabled = 0;
      return 0;
    }
#endif

	mutex_lock(&dev->lock);

	if (!dev->adb_enabled)
		goto out;

	pr_debug("disabling adb\n");
	dev->adb_enabled = 0;

#ifdef CONFIG_LGE_USB_GADGET_DRIVER
	if (product_id == lg_ums_pid) {
		pr_info("%s: adb disabling on UMS only mode, skip disabling ADB\n",
				__func__);
		goto out;
	}
#endif

	if (product_id)
		ret = android_switch_composition(product_id);
out:
	mutex_unlock(&dev->lock);

	return ret;
}

static struct file_operations adb_enable_fops = {
	.owner =   THIS_MODULE,
	.open =    adb_enable_open,
	.release = adb_enable_release,
};

static struct miscdevice adb_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb_enable",
	.fops = &adb_enable_fops,
};

static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;
	int ret;

	pr_debug("android_probe pdata: %p\n", pdata);

	if (!pdata || !pdata->vendor_id || !pdata->product_name ||
		!pdata->manufacturer_name)
		return -ENODEV;

	device_desc.idVendor =	__constant_cpu_to_le16(pdata->vendor_id);
	dev->version = pdata->version;
	strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
	strings_dev[STRING_MANUFACTURER_IDX].s = pdata->manufacturer_name;
	strings_dev[STRING_SERIAL_IDX].s = serial_number;
	dev->nluns = pdata->nluns;
	dev->pdata = pdata;

	ret = sysfs_create_group(&pdev->dev.kobj, &android_attr_grp);
	if (ret < 0) {
		pr_err("%s: Failed to create the sysfs entry \n", __func__);
		return ret;
	}
	ret = sysfs_create_group(&pdev->dev.kobj, &android_func_attr_grp);
	if (ret < 0) {
		pr_err("%s: Failed to create the functions sysfs entry \n",
				__func__);
		sysfs_remove_group(&pdev->dev.kobj, &android_attr_grp);
	}

	return ret;
}

#if defined (CONFIG_LGE_USB_GADGET_SUPPORT_FACTORY_USB)
int LG_USB_GET_ADB_STATE(void)
{
	struct android_dev *dev = _android_dev;
    return dev->adb_enabled;
}

EXPORT_SYMBOL(LG_USB_GET_ADB_STATE);
#endif


#ifdef CONFIG_LGE_USB_GADGET_DRIVER
void lge_usb_composite_unregister(void)
{
	usb_composite_unregister(&android_usb_driver);
}

void lge_usb_composite_register(void)
{
	usb_composite_register(&android_usb_driver);
}
#endif

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", },
	.probe = android_probe,
};

static int __init init(void)
{
	struct android_dev *dev;
	struct usb_composition *func;
	int ret;

	pr_debug("android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto out;
	}

	_android_dev = dev;
	mutex_init(&dev->lock);

	ret = adb_function_init();
	if (ret)
		goto free_dev;

#if defined(CONFIG_LGE_USB_GADGET_MTP_DRIVER)
	ret = mtp_function_init();
    if (ret)
        goto adb_exit;
#endif

	ret = platform_driver_register(&android_platform_driver);
	if (ret)
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
        goto mtp_exit;
#else
		goto adb_exit;
#endif

	ret = misc_register(&adb_enable_device);
	if (ret)
		goto pdrv_unregister;

#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
	ret = misc_register(&mtp_enable_device);
	if (ret)
		goto misc_deregister;
#endif
#ifdef CONFIG_LGE_USB_GADGET_DRIVER
	product_id = lg_android_pid;
#endif

	/* Defer composite driver registration till product id is available */
	mutex_lock(&dev->lock);
	if (!product_id) {
		mutex_unlock(&dev->lock);
		ret = 0; /* not failure */
		goto out;
	}

	func = android_validate_product_id(product_id);
	if (!func) {
		mutex_unlock(&dev->lock);
		pr_err("%s: invalid product id\n", __func__);
		ret = -EINVAL;
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
        goto mtp_misc_deregister;
#else
		goto misc_deregister;
#endif
	}
	dev->functions = func->functions;

#ifdef CONFIG_LGE_USB_GADGET_DRIVER
	usb_composite_unregister(&android_usb_driver);
#endif
	ret = usb_composite_register(&android_usb_driver);
	if (ret) {
		mutex_unlock(&dev->lock);
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
        goto mtp_misc_deregister;
#else
		goto misc_deregister;
#endif
	}
	mutex_unlock(&dev->lock);

	return 0;

#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
mtp_misc_deregister:
    misc_deregister(&mtp_enable_device);
#endif
misc_deregister:
	misc_deregister(&adb_enable_device);
pdrv_unregister:
	platform_driver_unregister(&android_platform_driver);
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
mtp_exit:
    mtp_function_exit();
#endif
adb_exit:
	adb_function_exit();
free_dev:
	kfree(dev);
out:
	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
#if defined(CONFIG_USB_ANDROID_CDC_ECM) || defined(CONFIG_USB_ANDROID_RNDIS)
	gether_cleanup();
#endif

	usb_composite_unregister(&android_usb_driver);
#ifdef CONFIG_LGE_USB_GADGET_MTP_DRIVER
    misc_deregister(&mtp_enable_device);
#endif
	misc_deregister(&adb_enable_device);
	platform_driver_unregister(&android_platform_driver);
	adb_function_exit();
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
