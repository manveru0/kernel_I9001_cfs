/* drivers/input/touchscreen/Melfas_mcs8000.c
 *
 * Copyright (C) 2007 Google, Inc.
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
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/vreg.h>

#include <linux/slab.h>

#include "mcs8000_download.h"

#define INPUT_INFO_REG 0x10
#define IRQ_TOUCH_INT   MSM_GPIO_TO_INT(GPIO_TOUCH_INT)

#define NEW_FIRMWARE_VERSION 0x09

static int debug_level = 5; 
static struct vreg *vreg_ldo2;

#define debugprintk(level,x...)  if(debug_level>=level) printk(x)

extern int mcsdl_download_binary_data(void);//eunsuk test  [int hw_ver -> void]

extern struct class *sec_class;


struct input_info {
	int max_x;
	int max_y;
	int state;
	int x;
	int y;
	int z;
	int x2; 
	int y2;
	int z2;
	int width;
	int finger_id; 
};

struct mcs8000_ts_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int irq;
	int hw_rev;
	int fw_ver;
	struct input_info info;
	int suspended;
	struct early_suspend	early_suspend;
};

struct mcs8000_ts_driver *melfas_mcs8000_ts = NULL;
struct i2c_driver mcs8000_ts_i2c;
struct workqueue_struct *melfas_mcs8000_ts_wq;

#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_mcs8000_ts_early_suspend(struct early_suspend *h);
void melfas_mcs8000_ts_late_resume(struct early_suspend *h);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#define TOUCH_HOME	KEY_HOME
#define TOUCH_MENU	KEY_MENU
#define TOUCH_BACK	KEY_BACK
#define TOUCH_SEARCH  KEY_SEARCH

int melfas_ts_tk_keycode[] =
{ TOUCH_MENU, TOUCH_HOME, TOUCH_BACK, TOUCH_SEARCH, };

struct device *mcs8000_ts_dev;

void mcsdl_vdd_on(void)
{ 
	gpio_set_value( TSP_LDO_ON, 1 ); 
	mdelay(10);
}

void mcsdl_vdd_off(void)
{
	gpio_set_value( TSP_LDO_ON, 0 ); 
	mdelay(10);
}

static int melfas_mcs8000_i2c_read(struct i2c_client* p_client, u8 reg, u8* data, int len)
{

	struct i2c_msg msg;

	/* set start register for burst read */
	/* send separate i2c msg to give STOP signal after writing. */
	/* Continous start is not allowed for cypress touch sensor. */

	msg.addr = p_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &reg;

		printk("p_client->addr[[ %x  ]\n", __func__, p_client->addr);

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s set data pointer fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	/* begin to read from the starting address */

	msg.addr = p_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	if (1 != i2c_transfer(p_client->adapter, &msg, 1))
	{
		printk("%s fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}
	
	return 0;
}

static void melfas_mcs8000_read_version(void)
{
	u8 buf[2] = {0,};
	
	if (0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_MODULE_VER_REG, buf, 2))
	{

		melfas_mcs8000_ts->hw_rev = buf[0];
		melfas_mcs8000_ts->fw_ver = buf[1];
		
		printk("%s :HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__, buf[0], buf[1]);
	}
	else
	{
		melfas_mcs8000_ts->hw_rev = 0;
		melfas_mcs8000_ts->fw_ver = 0;
		
		printk("%s : Can't find HW Ver, FW ver!\n", __func__);
	}
}

static void melfas_mcs8000_read_resolution(void)
{
	
	uint16_t max_x=0, max_y=0;	

	u8 buf[3] = {0,};
	
	if(0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_RESOL_HIGH_REG , buf, 3)){

		printk("%s :buf[0] : 0x%02x, buf[1] : 0x%02x, buf[2] : 0x%02x\n", __func__,buf[0],buf[1],buf[2]);

		if((buf[0] == 0)||(buf[0] == 0)||(buf[0] == 0)){
			melfas_mcs8000_ts->info.max_x = 320;
			melfas_mcs8000_ts->info.max_y = 480;
			
			printk("%s : Can't find Resolution!\n", __func__);
			}
		
		else{
			max_x = buf[1] | ((uint16_t)(buf[0] & 0x0f) << 8); 
			max_y = buf[2] | (((uint16_t)(buf[0] & 0xf0) >> 4) << 8); 
			melfas_mcs8000_ts->info.max_x = max_x;
		    melfas_mcs8000_ts->info.max_y = max_y;

			printk("%s :max_x: %d, max_y: %d\n", __func__, melfas_mcs8000_ts->info.max_x, melfas_mcs8000_ts->info.max_y);
			}
		}

	else
	{
		melfas_mcs8000_ts->info.max_x = 320;
		melfas_mcs8000_ts->info.max_y = 480;
		
		printk("%s : Can't find Resolution!\n", __func__);
	}
}

static ssize_t registers_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 buf1[2] = {0,};
	u8 buf2[2] = {0,};

	int status, mode_ctl, hw_rev, fw_ver;

	if (0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_STATUS_REG, buf1, 2))
	{
		status = buf1[0];
		mode_ctl = buf1[1];	 
	}
	else
	{
		printk("%s : Can't find status, mode_ctl!\n", __func__); 
	}

	if (0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_MODULE_VER_REG, buf2, 2))
	{
		hw_rev = buf2[0];
		fw_ver = buf2[1];	 
	}
	else
	{
		printk("%s : Can't find HW Ver, FW ver!\n", __func__); 
	}
	
	sprintf(buf, "[TOUCH] Melfas Tsp Register Info.\n");
	sprintf(buf, "%sRegister 0x00 (status)  : 0x%08x\n", buf, status);
	sprintf(buf, "%sRegister 0x01 (mode_ctl): 0x%08x\n", buf, mode_ctl);
	sprintf(buf, "%sRegister 0x30 (hw_rev)  : 0x%08x\n", buf, hw_rev);
	sprintf(buf, "%sRegister 0x31 (fw_ver)  : 0x%08x\n", buf, fw_ver);

	return sprintf(buf, "%s", buf);
}

static ssize_t registers_store_mcs8000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
		
	    ret = i2c_smbus_write_byte_data(melfas_mcs8000_ts->client, 0x01, 0x01);
		if (ret < 0) {
			printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		}
		printk("[TOUCH] software reset.\n");
	}
	return size;
}

static ssize_t gpio_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "[TOUCH] Melfas Tsp Gpio Info.\n");
	sprintf(buf, "%sGPIO TOUCH_INT : %s\n", buf, gpio_get_value(GPIO_TOUCH_INT)? "HIGH":"LOW"); 
	return sprintf(buf, "%s", buf);
}

static ssize_t gpio_store_mcs8000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "ON", 2) == 0 || strncmp(buf, "on", 2) == 0) {
    mcsdl_vdd_on();
		//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
		printk("[TOUCH] enable.\n");
		mdelay(200);
	}

	if(strncmp(buf, "OFF", 3) == 0 || strncmp(buf, "off", 3) == 0) {
    mcsdl_vdd_off();
		printk("[TOUCH] disable.\n");
	}
	
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
    mcsdl_vdd_off();
		mdelay(500);
    mcsdl_vdd_on();
		printk("[TOUCH] reset.\n");
		mdelay(200);
	}
	return size;
}


static ssize_t firmware_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	u8 buf1[2] = {0,};
	int hw_rev, fw_ver;


	if (0 == melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_MODULE_VER_REG, buf1, 2))
	{
		hw_rev = buf1[0];
		fw_ver = buf1[1];	 
		sprintf(buf,"HW Ver : 0x%02x, FW Ver : 0x%02x\n", hw_rev, fw_ver);
	}
	else
	{	 
		printk("%s : Can't find HW Ver, FW ver!\n", __func__);
	}

return sprintf(buf, "%s", buf); 
}


static ssize_t firmware_store_mcs8000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE	
	int ret;
	if(strncmp(buf, "UPDATE", 6) == 0 || strncmp(buf, "update", 6) == 0) {
		printk("[TOUCH] Melfas  H/W version: 0x%02x.\n", melfas_mcs8000_ts->hw_rev);
		printk("[TOUCH] Current F/W version: 0x%02x.\n", melfas_mcs8000_ts->fw_ver);
		if( melfas_mcs8000_ts->fw_ver != NEW_FIRMWARE_VERSION && melfas_mcs8000_ts->hw_rev == 0x12) { 
		disable_irq(melfas_mcs8000_ts->client->irq);

		printk("[F/W D/L] Entry gpio_tlmm_config\n");
		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(TSP_LDO_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		
		printk("[F/W D/L] Entry mcsdl_download_binary_data\n");
		ret = mcsdl_download_binary_data(); //eunsuk test [melfas_mcs8000_ts->hw_rev -> ()]
		
		enable_irq(melfas_mcs8000_ts->client->irq);
		
		melfas_mcs8000_read_version();
			
		if(ret > 0){
				if (melfas_mcs8000_ts->hw_rev < 0) {
					printk(KERN_ERR "i2c_transfer failed\n");;
				}
				
				if (melfas_mcs8000_ts->fw_ver < 0) {
					printk(KERN_ERR "i2c_transfer failed\n");
				}
				
				printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_mcs8000_ts->hw_rev, melfas_mcs8000_ts->fw_ver);

		}
		else {
			printk("[TOUCH] Firmware update failed.. RESET!\n");
      mcsdl_vdd_off();
			mdelay(500);
      mcsdl_vdd_on();
			mdelay(200);
		}
		}
	}
#endif

	return size;
}



static ssize_t debug_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "%d", debug_level);
}

static ssize_t debug_store_mcs8000(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(buf[0]>'0' && buf[0]<='9') {
		debug_level = buf[0] - '0';
	}

	return size;
}

#define N_SENS_COUNT        3
static uint8_t  tkey_sensitivity_read(void)
{
    uint8_t     tkey_sens, sens_min, sens_max ;
    int         n, sens_sum ;

    sens_min = 0xFF ;   /* Document errata!!! */
    sens_max = 0x00 ;
    for (sens_sum = n = 0 ; n < 3 ; n++)
    {
        if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_RT_STRNTH_REG, &tkey_sens, 1) != 0)
        {
            printk(KERN_ERR "tkey_sensitivity_read read fail!!!\n") ;
            break ;
        }
        // tkey_sens &= 0x1F ; Document errata!!!
        printk(KERN_DEBUG "tkey_sens : %d\n", tkey_sens) ;
        sens_sum += tkey_sens ;
        if (sens_min > tkey_sens)
            sens_min = tkey_sens ;
        if (sens_max < tkey_sens)
            sens_max = tkey_sens ;
    }
    tkey_sens = sens_sum - sens_min - sens_max ;    /* median */
    printk(KERN_ERR "tkey_sensitivity_read key(%d)\n", tkey_sens) ;

    return tkey_sens ;
}

/* Touch key Sensitivity ******************************************************/
static ssize_t tkey_sensitivity_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t     tkey_sens, tkey_touched ;

    if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_INFORM_REG, &tkey_touched, 1) != 0 /* Key State */
            || (tkey_touched & 0x08) == 0) /* no touched */
        return sprintf(buf, "-1") ;

    /* else touched */
    msleep(100) ;
    tkey_sens = tkey_sensitivity_read() ;
	return sprintf(buf, "%d", tkey_sens);
}

/* Touch key Sensitivity ******************************************************/
static ssize_t tkey_sens_pline_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t     tkey_sens ;
    uint8_t     tkey_touched ;
    uint8_t     noise_thd = 0 ;
    int         n ;
    char        str[20] ;

    if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_THD_REG, &noise_thd, 1) != 0)
    {
        printk(KERN_ERR "[tkey_sens_pline_show] Fail to read noise_threshold<0x3F>!\n") ;
        return sprintf(buf, "0,0,0,0") ;
    }

    if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_INFORM_REG, &tkey_touched, 1) != 0 /* Key State */
            || (tkey_touched & 0x08) == 0) /* no touched */
    {
        printk(KERN_ERR "[tkey_sens_pline_show] Fail to read key_state<0x1C> or untouched(%d)!\n",
                (tkey_touched & 0x08)) ;
        return sprintf(buf, "%d,%d,%d,%d", noise_thd, noise_thd, noise_thd, noise_thd) ;
    }

    tkey_touched &= 0x07 ;  /* clear touch state bit (0x08) */
    printk(KERN_INFO "[tkey_sens_pline_show] touched =0x%02x\n", tkey_touched) ;
    msleep(100) ;
    tkey_sens = tkey_sensitivity_read() ;

    /* tkey_touched = (1, 2, 3, 4) */
    sprintf(str, "%d", tkey_touched == 1 ? tkey_sens : noise_thd) ;
    printk(KERN_INFO "%s", str) ;
    strcpy(buf, str) ;
    for (n = 2 ; n <= 4 ; n++)
    {
        sprintf(str, ",%d", tkey_touched == n ? tkey_sens : noise_thd) ;
        printk("%s", str) ;
	    strcat(buf, str);
    }
    printk("\n") ;

	return strlen(buf) ;
}

/* Noise Threshold : Touch Key default Sensitivity ****************************/
static ssize_t tkey_noise_thd_show_mcs8000(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char   noise_thd ;

    if (melfas_mcs8000_i2c_read(melfas_mcs8000_ts->client, MCSTS_TKEY_THD_REG, &noise_thd, 1) == 0)
    {
        noise_thd &= 0x1F ;
        printk("TKEY_THRESHOLD : %02d\n", noise_thd) ;
        return sprintf(buf, "%d", noise_thd) ;
    }
    /* else */
    printk(KERN_ERR "TKEY_THRESHOLD : error\n") ;
    return sprintf(buf, "0") ;  // TODO:
}

static ssize_t touch_led_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u8 data = 0x10;
	int rc;
	
	if (sscanf(buf, "%d\n", &data) == 1)
		printk("%d\n", data);

	if(data == 1)
	{
		rc = vreg_enable(vreg_ldo2);

		if (rc) {
			pr_err("%s: LDO2 vreg enable failed (%d)\n",
			       __func__, rc);
			return rc;
		}
	}
	else if(data == 2)
	{
		rc = vreg_disable(vreg_ldo2);

		if (rc) {
			pr_err("%s: LDO2 vreg disable failed (%d)\n",
			       __func__, rc);
			return rc;
		}
	}

	return size;
}

static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show_mcs8000, gpio_store_mcs8000);
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, registers_show_mcs8000, registers_store_mcs8000);
static DEVICE_ATTR(firmware, S_IRUGO | S_IWUGO, firmware_show_mcs8000, firmware_store_mcs8000);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show_mcs8000, debug_store_mcs8000);
static DEVICE_ATTR(tkey_sensitivity, S_IRUGO, tkey_sensitivity_show_mcs8000, NULL) ;
static DEVICE_ATTR(tkey_sens_pline, S_IRUGO, tkey_sens_pline_show_mcs8000, NULL) ;
static DEVICE_ATTR(tkey_noise_thd, S_IRUGO, tkey_noise_thd_show_mcs8000, NULL) ;
static DEVICE_ATTR(brightness, 0666, NULL, touch_led_control);

void melfas_mcs8000_upgrade(void)
{
	#ifdef CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE	
	int ret;
	
	printk("[TOUCH] Melfas	H/W version: 0x%02x.\n", melfas_mcs8000_ts->hw_rev);
	printk("[TOUCH] Current F/W version: 0x%02x.\n", melfas_mcs8000_ts->fw_ver);

	printk("[F/W D/L] Entry gpio_tlmm_config\n");
	
	printk("[F/W D/L] Entry mcsdl_download_binary_data\n");
	ret = mcsdl_download_binary_data(); 
	
	melfas_mcs8000_read_version();
		
	if(ret > 0){
			if (melfas_mcs8000_ts->hw_rev < 0) {
				printk(KERN_ERR "i2c_transfer failed\n");;
			}
			
			if (melfas_mcs8000_ts->fw_ver < 0) {
				printk(KERN_ERR "i2c_transfer failed\n");
			}
			
			printk("[TOUCH] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_mcs8000_ts->hw_rev, melfas_mcs8000_ts->fw_ver);

	}
	else {
		printk("[TOUCH] Firmware update failed.. RESET!\n");
  		mcsdl_vdd_off();
		mdelay(500);
  		mcsdl_vdd_on();
		mdelay(200);
	}
#endif
}



void melfas_mcs8000_ts_work_func(struct work_struct *work)
{
  int ret;
  int ret1; 

  struct i2c_msg msg[2];  
  uint8_t start_reg;
  uint8_t buf1[13];

	



  msg[0].addr = melfas_mcs8000_ts->client->addr;
  msg[0].flags = 0; 
  msg[0].len = 1;
  msg[0].buf = &start_reg;
  start_reg = MCSTS_INPUT_INFO_REG;
  msg[1].addr = melfas_mcs8000_ts->client->addr;
  msg[1].flags = I2C_M_RD; 
  msg[1].len = sizeof(buf1);
  msg[1].buf = buf1;
  
  ret  = i2c_transfer(melfas_mcs8000_ts->client->adapter, &msg[0], 1);
  ret1 = i2c_transfer(melfas_mcs8000_ts->client->adapter, &msg[1], 1);

//  printk("[TSP8000] buf1[0] = %x\n", buf1[0]);

  if((ret < 0) ||  (ret1 < 0)) 
  	{
  		printk(KERN_ERR "==melfas_mcs8000_ts_work_func: i2c_transfer failed!!== ret:%d ,ret1:%d\n",ret,ret1);
	}
  else
  	{    
	    int x = buf1[2] | ((uint16_t)(buf1[1] & 0x0f) << 8); 
	    int y = buf1[3] | (((uint16_t)(buf1[1] & 0xf0) >> 4) << 8); 
	    int z = buf1[5];
		
		int x2 = buf1[8] | ((uint16_t)(buf1[7] & 0x0f) << 8); 
	    int y2 = buf1[9] | (((uint16_t)(buf1[7] & 0xf0) >> 4) << 8); 
	    int z2 = buf1[11];
		
//	    int key_value =(int)( buf1[12] & 0x07); //key value
		int key_value =(int)( buf1[0] & 0x07); //key value	    
		
		
	    int finger1_state = buf1[0] & 0x01;
		int finger2_state = buf1[6] & 0x01;
	    int key_state = (int)((buf1[0] >> 4) & 0x01); //key_status
	    		
		int do_report1 = false; 
		int do_report2 = false;
		int key_event = 0;

//		if(key_value != 0){
		if((buf1[0] & 0x20) == 0){
			if(key_value == 0x1)
			{
			  key_event = TOUCH_MENU;
			}
			else if(key_value == 0x2)
			{
			  key_event = TOUCH_HOME;
			}
			else if(key_value == 0x3)
			{
			  key_event = TOUCH_BACK;
			}
			else if(key_value == 0x4)
			{
			  key_event = TOUCH_SEARCH;
			}
			else
			{
			  enable_irq(melfas_mcs8000_ts->client->irq);
			  return;
			}
			
			input_report_key(melfas_mcs8000_ts->input_dev, key_event, key_state);
			debugprintk(5,"[TOUCH_KEY] key_event = [%d], touchaction = [%d] \n", key_event, key_state);
#if defined(CONFIG_MACH_ESCAPE)
			keypad_backlight_control();
#endif
		}
		else {
			if(finger1_state){
				melfas_mcs8000_ts->info.x = x;
				melfas_mcs8000_ts->info.y = y;
				melfas_mcs8000_ts->info.z = z;
				melfas_mcs8000_ts->info.finger_id = 1; 
				do_report1 = true;
			}
			if(finger2_state){
				melfas_mcs8000_ts->info.x2 = x2;
				melfas_mcs8000_ts->info.y2 = y2;
				melfas_mcs8000_ts->info.z2 = z2;
				melfas_mcs8000_ts->info.finger_id = 2; 	  
				do_report2 = true;
			}

			if(finger1_state == 0 && finger2_state ==0 ){
				melfas_mcs8000_ts->info.x = -1;
				melfas_mcs8000_ts->info.y = -1;
				melfas_mcs8000_ts->info.z = -1;
				melfas_mcs8000_ts->info.x2 = -1;
				melfas_mcs8000_ts->info.y2 = -1;
				melfas_mcs8000_ts->info.z2 = -1; 		  
				melfas_mcs8000_ts->info.finger_id = -1; 
				z2 = 0;
				do_report1 = false;
				do_report2 = false;
			}
				  	
			    //  melfas_mcs8000_ts->info.state = touchaction;
					
			if(do_report1) {
				printk("[TSP8000] buf1[0] = %x\n", buf1[0]);
				debugprintk(5,"[TOUCH_MT] x1: %4d, y1: %4d, z1: %4d, finger: %4d,\n", x, y, z, finger1_state);
				input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, y);
				input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, z);		
				//input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, 1);		
				input_mt_sync(melfas_mcs8000_ts->input_dev);			
			}

			if(do_report2) {
				debugprintk(5,"[TOUCH_MT2] x2: %4d, y2: %4d, z2: %4d, finger: %4d,\n", x, y, z, finger2_state);
				input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, x2);
				input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, y2);
				input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, z2);	  
				//input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, 2);	  
				input_mt_sync(melfas_mcs8000_ts->input_dev);
			}
			
			if(finger1_state ==0 & finger2_state==0)	  	
			input_mt_sync(melfas_mcs8000_ts->input_dev);
		}	
	
    input_sync(melfas_mcs8000_ts->input_dev);
  }
  enable_irq(melfas_mcs8000_ts->client->irq);
}

irqreturn_t melfas_mcs8000_ts_irq_handler(int irq, void *dev_id)
{
//	printk("\n=======         melfas_mcs8000_ts_irq_handler      =========");
 
	disable_irq_nosync(melfas_mcs8000_ts->client->irq);
	//disable_irq(melfas_mcs8000_ts->client->irq);
	queue_work(melfas_mcs8000_ts_wq, &melfas_mcs8000_ts->work);
	return IRQ_HANDLED;
}

int melfas_mcs8000_ts_probe(struct i2c_client *client,
               const struct i2c_device_id *id)
{
	int ret = 0;
	int rc;
	uint16_t max_x=0, max_y=0;
	//int fw_ver = 0;
	//int hw_rev = 0;
	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] PROBE       =========");
	printk("\n====================================================\n");

	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(TSP_LDO_ON, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

	gpio_set_value( GPIO_I2C0_SCL , 1 ); 
	gpio_set_value( GPIO_I2C0_SDA , 1 ); 
	gpio_set_value( GPIO_TOUCH_INT , 1 );
	msleep(10);
	gpio_set_value( TSP_LDO_ON, 1 ); 
	msleep(500);


	// VDD_VIB_3.0V
	vreg_ldo2 = vreg_get(NULL, "xo_out");
	if (IS_ERR(vreg_ldo2)) {
		rc = PTR_ERR(vreg_ldo2);
		pr_err("%s: wlan2 vreg get failed (%d)\n",
		       __func__, rc);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo2, 3300);
	if (rc) {
		pr_err("%s: vreg LDO19 set level failed (%d)\n",
		       __func__, rc);
		return rc;
	}

/*	rc = vreg_enable(vreg_ldo2);

	if (rc) {
		pr_err("%s: LDO19 vreg enable failed (%d)\n",
		       __func__, rc);
		return rc;
	}
*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C/*I2C_FUNC_SMBUS_BYTE_DATA*/)) {
		printk(KERN_ERR "melfas_mcs8000_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	melfas_mcs8000_ts = kzalloc(sizeof(*melfas_mcs8000_ts), GFP_KERNEL);
	if (melfas_mcs8000_ts == NULL) {
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}
	
	INIT_WORK(&melfas_mcs8000_ts->work, melfas_mcs8000_ts_work_func);

	melfas_mcs8000_ts->client = client;
	i2c_set_clientdata(client, melfas_mcs8000_ts);
	
	melfas_mcs8000_read_version(); 
	// firmware updgrade
	//if(melfas_mcs8000_ts->fw_ver != NEW_FIRMWARE_VERSION && melfas_mcs8000_ts->hw_rev == 0x12)
	//	melfas_mcs8000_upgrade();
	
	printk(KERN_INFO "[TOUCH] Melfas  H/W version: 0x%x.\n", melfas_mcs8000_ts->hw_rev);
	printk(KERN_INFO "[TOUCH] Current F/W version: 0x%x.\n", melfas_mcs8000_ts->fw_ver);
	
	melfas_mcs8000_read_resolution(); 
	max_x = 480; //melfas_mcs8000_ts->info.max_x ;
	max_y = 800;//melfas_mcs8000_ts->info.max_y ;
	printk("melfas_ts_probe: max_x: %d, max_y: %d\n", max_x, max_y);

	melfas_mcs8000_ts->input_dev = input_allocate_device();
	if (melfas_mcs8000_ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "melfas_mcs8000_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	melfas_mcs8000_ts->input_dev->name = "mcs8000_ts_input";

	set_bit(EV_SYN, melfas_mcs8000_ts->input_dev->evbit);
	set_bit(EV_KEY, melfas_mcs8000_ts->input_dev->evbit);
	set_bit(TOUCH_HOME, melfas_mcs8000_ts->input_dev->keybit);
	set_bit(TOUCH_MENU, melfas_mcs8000_ts->input_dev->keybit);
	set_bit(TOUCH_BACK, melfas_mcs8000_ts->input_dev->keybit);
	set_bit(TOUCH_SEARCH, melfas_mcs8000_ts->input_dev->keybit);

	melfas_mcs8000_ts->input_dev->keycode = melfas_ts_tk_keycode;	
	set_bit(BTN_TOUCH, melfas_mcs8000_ts->input_dev->keybit);
	set_bit(EV_ABS, melfas_mcs8000_ts->input_dev->evbit);

	input_set_abs_params(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(melfas_mcs8000_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	printk("melfas_mcs8000_ts_probe: max_x: %d, max_y: %d\n", max_x, max_y);

	ret = input_register_device(melfas_mcs8000_ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_mcs8000_ts_probe: Unable to register %s input device\n", melfas_mcs8000_ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	printk("[Touch] irq : %d, irq : %d\n", melfas_mcs8000_ts->client->irq, client->irq);

	set_irq_type(melfas_mcs8000_ts->client->irq, IRQ_TYPE_EDGE_FALLING);
	ret = request_irq(melfas_mcs8000_ts->client->irq, melfas_mcs8000_ts_irq_handler, IRQF_DISABLED  , melfas_mcs8000_ts->client->name, melfas_mcs8000_ts);
	if(ret == 0) {
		printk(KERN_INFO "melfas_mcs8000_ts_probe: Start touchscreen %s \n", melfas_mcs8000_ts->input_dev->name);
	}
	else {
		printk("request_irq failed\n");
	}

	printk("[Melfas] ret : %d, melfas_mcs8000_ts->client name : [%s]  [%d] [0x%x]\n",ret,melfas_mcs8000_ts->client->name, melfas_mcs8000_ts->client->irq, melfas_mcs8000_ts->client->addr);

#ifdef CONFIG_HAS_EARLYSUSPEND
	melfas_mcs8000_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	melfas_mcs8000_ts->early_suspend.suspend = melfas_mcs8000_ts_early_suspend;
	melfas_mcs8000_ts->early_suspend.resume = melfas_mcs8000_ts_late_resume;
	register_early_suspend(&melfas_mcs8000_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	return 0;
err_misc_register_device_failed:
err_input_register_device_failed:
	input_free_device(melfas_mcs8000_ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(melfas_mcs8000_ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;

}

int melfas_mcs8000_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_mcs8000_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_mcs8000_ts->client->irq, 0);
	input_unregister_device(melfas_mcs8000_ts->input_dev);
	return 0;
}

int melfas_mcs8000_ts_gen_touch_up(void)
{
  // report up key if needed
  if(melfas_mcs8000_ts->info.state == 0x1) //down state
  {
    melfas_mcs8000_ts->info.state = 0x0;
	int finger = melfas_mcs8000_ts->info.finger_id;
    int x = melfas_mcs8000_ts->info.x;
    int y = melfas_mcs8000_ts->info.y;
    int z = melfas_mcs8000_ts->info.z;
    printk("[TOUCH] GENERATE UP KEY x: %4d, y: %4d, z: %4d\n", x, y, z);
	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_TRACKING_ID, finger);
    if (x) 	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_X, x);
    if (y)	input_report_abs(melfas_mcs8000_ts->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(melfas_mcs8000_ts->input_dev, ABS_PRESSURE, z);

    input_sync(melfas_mcs8000_ts->input_dev);
  }    
}

int melfas_mcs8000_ts_suspend(pm_message_t mesg)
{
  melfas_mcs8000_ts->suspended = true;
  melfas_mcs8000_ts_gen_touch_up();
  disable_irq(melfas_mcs8000_ts->client->irq);
  
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  
  mcsdl_vdd_off();
  gpio_set_value(GPIO_I2C0_SCL, 0);  // TOUCH SCL DIS
  gpio_set_value(GPIO_I2C0_SDA, 0);  // TOUCH SDA DIS
  
  return 0;
}

int melfas_mcs8000_ts_resume()
{
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
  gpio_tlmm_config(GPIO_CFG(GPIO_I2C0_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

  mcsdl_vdd_on();
  gpio_set_value(GPIO_I2C0_SCL, 1);  // TOUCH SCL EN
  gpio_set_value(GPIO_I2C0_SDA, 1);  // TOUCH SDA EN    
  msleep(300);
  melfas_mcs8000_ts->suspended = false;
  enable_irq(melfas_mcs8000_ts->client->irq);  

  return 0;
}
#if 0 // blocked for now.. we will gen touch when suspend func is called
int tsp_preprocess_suspend(void)
{
  // this function is called before kernel calls suspend functions
  // so we are going suspended if suspended==false
  if(melfas_mcs8000_ts->suspended == false) {  
    // fake as suspended
    melfas_mcs8000_ts->suspended = true;
    
    //generate and report touch event
    melfas_mcs8000_ts_gen_touch_up();
  }
  return 0;
}
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_mcs8000_ts_early_suspend(struct early_suspend *h)
{
	melfas_mcs8000_ts_suspend(PMSG_SUSPEND);
}

void melfas_mcs8000_ts_late_resume(struct early_suspend *h)
{
	melfas_mcs8000_ts_resume();
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */


int melfas_mcs8000_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	melfas_mcs8000_ts->client = client;
	i2c_set_clientdata(client, melfas_mcs8000_ts);
	return 0;
}

#define USE_TS_TA_DETECT_CHANGE_REG 1
#if USE_TS_TA_DETECT_CHANGE_REG 
int set_tsp_for_ta_detect(int state)
{
    return 1;
}
EXPORT_SYMBOL(set_tsp_for_ta_detect);
#endif

static int __devexit melfas_mcs8000_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_mcs8000_ts->early_suspend);
#endif  /* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_mcs8000_ts->client->irq, 0);
	input_unregister_device(melfas_mcs8000_ts->input_dev);
   
	melfas_mcs8000_ts = i2c_get_clientdata(client);
	kfree(melfas_mcs8000_ts);
	return 0;
}

struct i2c_device_id melfas_mcs8000_id[] = {
	{ "mcs8000_i2c", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, melfas_mcs8000_id);

struct i2c_driver mcs8000_ts_i2c = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "mcs8000_i2c",
	},
	.id_table	= melfas_mcs8000_id,
	.probe	= melfas_mcs8000_ts_probe,
	.remove = melfas_mcs8000_ts_remove,
};


int __init melfas_mcs8000_ts_init(void)
{
	int ret;

	printk("\n====================================================");
	printk("\n=======         [TOUCH SCREEN] INIT        =========");
	printk("\n====================================================\n");

	melfas_mcs8000_ts_wq = create_singlethread_workqueue("melfas_mcs8000_ts_wq");
	if (!melfas_mcs8000_ts_wq)
		return -ENOMEM;
	
	melfas_mcs8000_ts = kzalloc(sizeof(struct mcs8000_ts_driver), GFP_KERNEL);
	if(melfas_mcs8000_ts == NULL) {
		return -ENOMEM;
	}

	ret = i2c_add_driver(&mcs8000_ts_i2c);
	if(ret) printk("[%s], i2c_add_driver failed...(%d)\n", __func__, ret);

	printk("[Melfas] ret : %d, melfas_mcs8000_ts->client name : %s\n",ret,melfas_mcs8000_ts->client->name);

	if(!melfas_mcs8000_ts->client) {
		printk("###################################################\n");
		printk("##                                               ##\n");
		printk("##    WARNING! TOUCHSCREEN DRIVER CAN'T WORK.    ##\n");
		printk("##    PLEASE CHECK YOUR TOUCHSCREEN CONNECTOR!   ##\n");
		printk("##                                               ##\n");
		printk("###################################################\n");
		i2c_del_driver(&mcs8000_ts_i2c);
		return 0;
	}

	if (sec_class == NULL)
	sec_class = class_create(THIS_MODULE, "sec");

	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	mcs8000_ts_dev = device_create(sec_class, NULL, 0, NULL, "ts");
	
	if (IS_ERR(mcs8000_ts_dev))
		pr_err("Failed to create device(ts)!\n");
	if (device_create_file(mcs8000_ts_dev, &dev_attr_gpio) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gpio.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_registers) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_registers.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_debug) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);

	if (device_create_file(mcs8000_ts_dev, &dev_attr_tkey_sensitivity) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_sensitivity.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_tkey_sens_pline) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_sens_pline.attr.name);
    if (device_create_file(mcs8000_ts_dev, &dev_attr_tkey_noise_thd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_noise_thd.attr.name);
	if (device_create_file(mcs8000_ts_dev, &dev_attr_brightness) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);



	return 0; //platform_driver_register(&mcs8000_ts_driver);

}

void __exit melfas_mcs8000_ts_exit(void)
{
	i2c_del_driver(&mcs8000_ts_i2c);

	if (melfas_mcs8000_ts_wq)
		destroy_workqueue(melfas_mcs8000_ts_wq);
}
//late_initcall(melfas_mcs8000_ts_init);
module_init(melfas_mcs8000_ts_init);
module_exit(melfas_mcs8000_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");
