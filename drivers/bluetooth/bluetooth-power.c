/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
 */
/*
 * Bluetooth Power Switch Module
 * controls power to external Bluetooth device
 * with interface to power management device
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>

//shiks_test static bool previous;
//shiks_test
#include <mach/gpio.h>

#include <linux/delay.h> // msleep()

#define GPIO_WLAN_RESET     127
#define GPIO_BT_WLAN_REG_ON 144
#define GPIO_BT_RESET       146

//shiks_test


static int bluetooth_toggle_radio(void *data, bool blocked)
{
	int ret = 0;
	int (*power_control)(int enable);

 printk("%s",__func__);;//shiks_test 
	power_control = data;
//shiks_test 	if (previous != blocked)
		ret = (*power_control)(!blocked);
//shiks_test 	previous = blocked;
	return ret;
}

static const struct rfkill_ops bluetooth_power_rfkill_ops = {
	.set_block = bluetooth_toggle_radio,
};

static int bluetooth_power_rfkill_probe(struct platform_device *pdev)
{
	struct rfkill *rfkill;
	int ret;

 printk("%s\n",__func__);;//shiks_test 

	rfkill = rfkill_alloc("bt_power", &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			      &bluetooth_power_rfkill_ops,
			      pdev->dev.platform_data);

	if (!rfkill) {
		dev_err(&pdev->dev, "rfkill allocate failed\n");
		return -ENOMEM;
	}

	/* force Bluetooth off during init to allow for user control */
	rfkill_init_sw_state(rfkill, 1);
//shiks_test 	previous = 1;

	ret = rfkill_register(rfkill);
	if (ret) {
		dev_err(&pdev->dev, "rfkill register failed=%d\n", ret);
		rfkill_destroy(rfkill);
		return ret;
	}

	platform_set_drvdata(pdev, rfkill);

	return 0;
}

static void bluetooth_power_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill *rfkill;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	rfkill = platform_get_drvdata(pdev);
	if (rfkill)
		rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
	platform_set_drvdata(pdev, NULL);
}

static int __devinit bt_power_probe(struct platform_device *pdev)
{
	int ret = 0;

 printk("%s\n",__func__);;//shiks_test 

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "platform data not initialized\n");
		return -ENOSYS;
	}

	if (gpio_request(GPIO_BT_WLAN_REG_ON, "GPIO_BT_WLAN_REG_ON"))printk(KERN_ERR "[BTDRV] %s: gpio request 'GPIO_BT_WLAN_REG_ON' failed\n", __func__);
	if (gpio_request(GPIO_BT_RESET, "GPIO_BT_RESET"))printk(KERN_ERR "[BTDRV] %s: gpio request 'GPIO_BT_RESET' failed\n", __func__);

	gpio_direction_output(GPIO_BT_WLAN_REG_ON, 1);
	msleep(50);
	gpio_direction_output(GPIO_BT_RESET, 1);
	gpio_direction_output(GPIO_WLAN_RESET, 1);
 
	msleep(150);

	gpio_direction_output(GPIO_BT_RESET, 0);
	gpio_direction_output(GPIO_WLAN_RESET, 0);
	msleep(50);
	gpio_direction_output(GPIO_BT_WLAN_REG_ON, 0);

	gpio_free(GPIO_BT_WLAN_REG_ON);
	gpio_free(GPIO_BT_RESET);
	gpio_free(GPIO_WLAN_RESET);

	ret = bluetooth_power_rfkill_probe(pdev);

	return ret;
}

static int __devexit bt_power_remove(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s\n", __func__);

	bluetooth_power_rfkill_remove(pdev);

	return 0;
}

static struct platform_driver bt_power_driver = {
	.probe = bt_power_probe,
	.remove = __devexit_p(bt_power_remove),
	.driver = {
		.name = "bt_power",
		.owner = THIS_MODULE,
	},
};

static int __init bluetooth_power_init(void)
{
	int ret;

 pr_info("bluetooth_power_init \n");

	ret = platform_driver_register(&bt_power_driver);
	return ret;
}

static void __exit bluetooth_power_exit(void)
{
	platform_driver_unregister(&bt_power_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM Bluetooth power control driver");
MODULE_VERSION("1.40");

module_init(bluetooth_power_init);
module_exit(bluetooth_power_exit);
