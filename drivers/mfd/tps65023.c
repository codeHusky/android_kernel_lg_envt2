/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/i2c.h>
#include <linux/mfd/tps65023.h>

/* TPS65023_registers */
#define TPS65023_VERSION	0
#define TPS65023_PGOODZ		1
#define TPS65023_MASK		2
#define TPS65023_REG_CTRL	3
#define TPS65023_CON_CTRL	4
#define TPS65023_CON_CTRL2	5
#define TPS65023_DEFCORE	6
#define TPS65023_DEFSLEW	7
#define TPS65023_LDO_CTRL	8
#define TPS65023_MAX		9

static struct i2c_client *tpsclient;

/* LCD Power On/Off */
#if defined(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA) //32bit
static int lcd_power_on = 1;

int is_lcd_panel_power(void)
{
	return lcd_power_on;
}

int tps65023_set_lcd_ldo_on(void)
{
	int ret;

	if (!tpsclient)
		return -ENODEV;

	ret = i2c_smbus_write_byte_data(tpsclient, TPS65023_REG_CTRL, 0xFF);

	if (ret >= 0) {
		lcd_power_on = 1;		
	} else {
		ret = i2c_smbus_write_byte_data(tpsclient,
				TPS65023_CON_CTRL2, 0x80);
		printk(KERN_INFO "TPS65023: lcd power on fail..!!!\n");
	}

	return ret;
}
int tps65023_set_lcd_ldo_off(void)
{
	int ret;

	if (!tpsclient)
		return -ENODEV;

	ret = i2c_smbus_write_byte_data(tpsclient, TPS65023_REG_CTRL, 0xF7);

	if(ret >= 0) {
		lcd_power_on = 0;
	}else {
		ret = i2c_smbus_write_byte_data(tpsclient,
				TPS65023_CON_CTRL2, 0x80);
		printk(KERN_INFO "TPS65023: lcd power off fail..!!!\n");
	}

	return ret;
}
EXPORT_SYMBOL(tps65023_set_lcd_ldo_on);
EXPORT_SYMBOL(tps65023_set_lcd_ldo_off);
#endif

int tps65023_set_dcdc1_level(int mvolts)
{
	int val;
	int ret;

	if (!tpsclient)
		return -ENODEV;

	if (mvolts < 800 || mvolts > 1600)
		return -EINVAL;

	if (mvolts == 1600)
		val = 0x1F;
	else
		val = ((mvolts - 800)/25) & 0x1F;

	ret = i2c_smbus_write_byte_data(tpsclient, TPS65023_DEFCORE, val);

	if (!ret)
		ret = i2c_smbus_write_byte_data(tpsclient,
				TPS65023_CON_CTRL2, 0x80);

	return ret;
}
EXPORT_SYMBOL(tps65023_set_dcdc1_level);

int tps65023_get_dcdc1_level(int *mvolts)
{
	int val;

	if (!tpsclient)
		return -ENODEV;

	val = i2c_smbus_read_byte_data(tpsclient, TPS65023_DEFCORE) & 0x1F;

	if (val == 0x1F)
		*mvolts = 1600;
	else
		*mvolts = (val * 25) + 800;
	return 0;
}
EXPORT_SYMBOL(tps65023_get_dcdc1_level);

static int tps65023_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR "TPS65023 does not support SMBUS_BYTE_DATA.\n");
		return -EINVAL;
	}

	tpsclient = client;
	printk(KERN_INFO "TPS65023: PMIC probed.\n");
	return 0;
}

static int __devexit tps65023_remove(struct i2c_client *client)
{
	tpsclient = NULL;
	return 0;
}

static const struct i2c_device_id tps65023_id[] = {
	{ "tps65023", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps65023_id);

static struct i2c_driver tps65023_driver = {
	.driver = {
		.name   = "tps65023",
		.owner  = THIS_MODULE,
	},
	.probe  = tps65023_probe,
	.remove = __devexit_p(tps65023_remove),
	.id_table = tps65023_id,
};

static int __init tps65023_init(void)
{
	return i2c_add_driver(&tps65023_driver);
}


static void __exit tps65023_exit(void)
{
	i2c_del_driver(&tps65023_driver);
}

module_init(tps65023_init);
module_exit(tps65023_exit);
