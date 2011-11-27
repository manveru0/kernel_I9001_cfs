/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2010, SAMSUNG. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <asm/gpio.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <mach/board.h>
#include <mach/vreg.h>
#include "sr030pc30.h"

#define CAM_FLASH_ENSET 56
#define CAM_FLASH_FLEN 57

#define SR030PC30_WRITE_LIST(A) \
    {\
        sr030pc30_i2c_write_list(A,(sizeof(A) / sizeof(A[0])),#A);\
    }

/*    Read setting file from SDcard
    - There must be no "0x" string in comment. If there is, it cause some problem.
*/
//#define CONFIG_LOAD_FILE

struct sr030pc30_work_t {
    struct work_struct work;
};

static struct  sr030pc30_work_t *sr030pc30_sensorw;
static struct  i2c_client *sr030pc30_client;

struct sr030pc30_ctrl_t {
    int8_t  opened;
    struct  msm_camera_sensor_info     *sensordata;
    int dtp_mode;
    int vtcall_mode;
};

static struct sr030pc30_ctrl_t *sr030pc30_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(sr030pc30_wait_queue);
DECLARE_MUTEX(sr030pc30_sem);

#ifdef CONFIG_LOAD_FILE
static int sr030pc30_regs_table_write(char *name);
#endif
static int sr030pc30_start(void);
extern struct i2c_client *lp8720_i2c_client;
//extern int lp8720_init;

static inline int lp8720_i2c_write(unsigned char addr, unsigned char data)
{
    int rc;
    unsigned char buf[2];

  //  if(!lp8720_init)
  //      return -EIO;

    struct i2c_msg msg = {
        .addr = lp8720_i2c_client->addr,
        .flags = 0,
        .len = 2,
        .buf = buf,
    };

    buf[0] = addr;
    buf[1] = data;

    rc = i2c_transfer(lp8720_i2c_client->adapter, &msg, 1);
    if (rc < 0)
        printk(KERN_ERR "[CAMDRV/SR030PC30] %s: lp8720_i2c_write failed: %d\n",__func__, rc);        
    
    return (rc == 1) ? 0 : -EIO;
}

int sr030pc30_i2c_tx_data(char* txData, int length)
{
    int rc; 

    struct i2c_msg msg[] = {
        {
            .addr = sr030pc30_client->addr,
            .flags = 0,
            .len = length,
            .buf = txData,        
        },
    };
    
    rc = i2c_transfer(sr030pc30_client->adapter, msg, 1);
    if (rc < 0) {
        printk(KERN_ERR "[CAMDRV/SR030PC30] sr030pc30_i2c_tx_data error %d\n", rc);
        return rc;
    }

    return 0;
}

static int sr030pc30_i2c_read(unsigned short page, unsigned short subaddr, unsigned short *data)
{
    int ret;
    unsigned char buf[1] = {0};
    struct i2c_msg msg = { sr030pc30_client->addr, 0, 2, buf };

    /* page select */
    buf[0] = 0xFC;
    buf[1] = page;
    ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;
    
    /* read data */
    msg.buf[0] = subaddr;
    msg.len = 1;
    ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;

    msg.flags = I2C_M_RD;
    
    ret = i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
    if (ret == -EIO) 
        goto error;

    //*data = ((buf[0] << 8) | buf[1]);
    *data = buf[0];

error:
    return ret;
}

static int sr030pc30_i2c_write(unsigned char u_addr, unsigned char u_data)
{
    unsigned char buf[2] = {0};
    struct i2c_msg msg = { sr030pc30_client->addr, 0, 2, buf };

    buf[0] = u_addr;
    buf[1] = u_data;

    printk("addr : 0x%x , value : 0x%x\n",buf[0],buf[1]);
    return i2c_transfer(sr030pc30_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int sr030pc30_i2c_write_read(u8 writedata_num, const u8* writedata, u8 readdata_num, u8* readdata)
{
  int err = 0, i = 0;
  struct i2c_msg msg[1];
  unsigned char writebuf[writedata_num];
  unsigned char readbuf[readdata_num];

  if (!sr030pc30_client->adapter)
  {
    printk("[CAMDRV/SR030PC30] can't search i2c client adapter\n");
    return -ENODEV;
  }

  /* Write */
  msg->addr  = sr030pc30_client->addr;
  msg->len   = writedata_num;
  memcpy(writebuf, writedata, writedata_num);    
  msg->buf   = writebuf;
  
  for(i = 0; i < 10; i++)  
  {
    err = i2c_transfer(sr030pc30_client->adapter, msg, 1) == 1 ? 0 : -EIO;
    if(err == 0) break;
    mdelay(1);
  }

  if(i == 10)
  {
    printk("[CAMDRV/SR030PC30] sr030pc30_i2c_write_read is failed... %d\n", err);
    return err;  
  }

  /* Read */
  msg->addr  = sr030pc30_client->addr;
  msg->flags = I2C_M_RD;
  msg->len   = readdata_num;
  memset(readbuf, 0x0, readdata_num);
  msg->buf   = readbuf;
  
  for(i = 0; i < 10; i++)
  {
    err = i2c_transfer(sr030pc30_client->adapter, msg, 1) == 1 ? 0 : -EIO;
    if (err == 0) 
    {
      memcpy(readdata, readbuf, readdata_num);
      return 0;
    }
    mdelay(1);
  }

  printk("[CAMDRV/SR030PC30] sr030pc30_i2c_write_read is failed... %d\n", err);

  return err;
}

static int sr030pc30_i2c_write_list(const unsigned short *list,int size, char *name)
{
    int ret = 0;
    int i;
    unsigned char addr, value;

#ifdef CONFIG_LOAD_FILE        
    ret = sr030pc30_regs_table_write(name);
#else
    printk(KERN_ERR "[CAMDRV/SR030PC30] list name : %s\n",name);
    for (i = 0; i < size; i++)
    {
        addr = (unsigned char)((list[i] & 0xFF00)>>8);
        value =(unsigned char)( list[i] & 0x00FF);

        //printk("addr = 0x%x, value=0x%x \n",addr,value);
        if(addr == 0xff)
        {
            printk(KERN_ERR "[CAMDRV/SR030PC30] delays for Snapshot - %d ms\n",value*10);
            msleep(value*10);
        }
        else
        {
            if(sr030pc30_i2c_write(addr, value) < 0)
            {
                printk("[CAMDRV/SR030PC30] sensor_write_list failed.\n");
                return -1;
            }
        }
        udelay(10);
    }
#endif
    return ret;
}

#ifdef CONFIG_LOAD_FILE
static char *sr030pc30_regs_table = NULL;

static int sr030pc30_regs_table_size;

void sr030pc30_regs_table_init(void)
{
    struct file *filp;
    char *dp;
    long l;
    loff_t pos;
//    int i;
    int ret;
    mm_segment_t fs = get_fs();

    printk("%s %d\n", __func__, __LINE__);

    set_fs(get_ds());

    filp = filp_open("/sdcard/5AAsensorsetting.h", O_RDONLY, 0);

    if (IS_ERR(filp)) {
        printk("file open error\n");
        return;
    }
    l = filp->f_path.dentry->d_inode->i_size;    
    printk("l = %ld\n", l);
    dp = kmalloc(l, GFP_KERNEL);
    if (dp == NULL) {
        printk("Out of Memory\n");
        filp_close(filp, current->files);
    }
    pos = 0;
    memset(dp, 0, l);
    ret = vfs_read(filp, (char __user *)dp, l, &pos);
    if (ret != l) {
        printk("Failed to read file ret = %d\n", ret);
        kfree(dp);
        filp_close(filp, current->files);
        return;
    }

    filp_close(filp, current->files);
    
    set_fs(fs);

    sr030pc30_regs_table = dp;
    
    sr030pc30_regs_table_size = l;

    *((sr030pc30_regs_table + sr030pc30_regs_table_size) - 1) = '\0';

    //printk("sr030pc30_regs_table 0x%x, %ld\n", dp, l);
}

void sr030pc30_regs_table_exit(void)
{
    printk("%s %d\n", __func__, __LINE__);
    if (sr030pc30_regs_table) {
        kfree(sr030pc30_regs_table);
        sr030pc30_regs_table = NULL;
    }    
}

static int sr030pc30_regs_table_write(char *name)
{
    char *start, *end, *reg;//, *data;    
    unsigned short addr, value;
    char reg_buf[3]={0,}, data_buf[3]={0,};

    addr = value = 0;

/*    *(reg_buf + 4) = '\0';
    *(data_buf + 4) = '\0';
*/
    printk(KERN_ERR "[CAMDRV/SR030PC30] list name : %s\n",name);

    start = strstr(sr030pc30_regs_table, name);
    
    end = strstr(start, "};");

    while (1) {    
        /* Find Address */    
        reg = strstr(start,"0x");        
        if (reg)
            start = (reg + 7);
        if ((reg == NULL) || (reg > end))
            break;
        /* Write Value to Address */    
        if (reg != NULL) {
            memcpy(reg_buf, (reg + 2), 2);    
            memcpy(data_buf, (reg + 4), 2);    

            addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
            value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 

            //printk("addr 0x%x, value 0x%x\n", addr, value);

            if(addr == 0xff)
            {
                printk(KERN_ERR "[CAMDRV/SR030PC30] Delays for Snapshot - %d ms\n",value*10);
                msleep(value*10);
            }    
            else
            {
                if( sr030pc30_i2c_write(addr, value) < 0 )
                {
                    printk(KERN_ERR "[CAMDRV/SR030PC30]  sensor_write_list fail...-_-\n");
                    return -1;
                }
            }
            udelay(10);    
        }
        else
            printk(KERN_ERR "[CAMDRV/SR030PC30]  EXCEPTION! reg value : %c  addr : 0x%x,  value : 0x%x\n", *reg, addr, value);
    }

    return 0;
}
#endif

void sr030pc30_set_power(int onoff)
{
    unsigned int mclk_cfg; 
	 struct vreg *vreg_ldo20, *vreg_ldo11;
    vreg_ldo20 = vreg_get(NULL, "gp13");
        if (!vreg_ldo20) {
		printk("[S5K4ECGX]%s: VREG L20 get failed\n", __func__);
	  }
         if (vreg_set_level(vreg_ldo20, 1800)) {
		printk("[S5K4ECGX]%s: vreg_set_level failed\n", __func__);
         	}
	vreg_ldo11 = vreg_get(NULL, "gp2");
        if (!vreg_ldo11) {
		printk("[S5K4ECGX]%s: VREG L11get failed\n", __func__);
	  }
         if (vreg_set_level(vreg_ldo11, 2800)) {
		printk("[S5K4ECGX]%s: vreg_set_level failed\n", __func__);	
         	}
    if(onoff)
    {
        printk(KERN_ERR "[CAMDRV/SR030PC300] %s: POWER ON.\n", __func__);

		mclk_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        /* initailize power control pin */    
        gpio_set_value(174, 0);
        gpio_set_value(175, 0);
        gpio_set_value(3, 0);
        gpio_set_value(31, 0);
        gpio_set_value(132, 0);

        /* initailize flash IC */
        gpio_set_value(CAM_FLASH_ENSET,0);
        gpio_set_value(CAM_FLASH_FLEN,0);  
       
        if (vreg_enable(vreg_ldo11)) {
		printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR030PC300]%s: reg_enable failed\n", __func__);
	  }
		mdelay(1);
		 if (vreg_enable(vreg_ldo20)) {
		printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![SR030PC300]%s: reg_enable failed\n", __func__);
	  }
		 mdelay(1);
		 gpio_set_value(31, 1);    // VGA_RSTN
		 mdelay(1);
       /* Enable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        mdelay(5);
        gpio_set_value(132, 1);    // VGA_RSTN
        mdelay(10);
        //after power on, below function will be called.
    }
    else
    {
        printk(KERN_ERR "[CAMDRV] %s: POWER OFF.\n", __func__);
        #if 0
        mclk_cfg = GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
        gpio_direction_output(2, 1);
        gpio_direction_output(3, 1);    
        gpio_direction_output(31, 1);    
        gpio_direction_output(132, 1);
        gpio_direction_output(174, 0);
        gpio_direction_output(175, 0);    
        gpio_direction_output(177, 0);

        //before power off, below function will be called.
        /* LP8720 enable */
        gpio_set_value(132,0);
        udelay(50);
        
        /* Disable MCLK */
        gpio_tlmm_config(mclk_cfg, GPIO_CFG_ENABLE);
        udelay(50);
         //vreg_disable(vreg_ldo20);

        /*initailize power control pin*/    
        gpio_set_value(31,0);
        udelay(50);    

        /* initailize flash IC */
        lp8720_i2c_write(0x08, 0x00);
        gpio_set_value(2, 0);        
        gpio_set_value(3, 0);
		#endif
    }
    


    return;
}

void sr030pc30_set_prevew(void)
{
    unsigned short value =0;
    int shade_value = 0;
    unsigned short agc_value = 0;
    
    printk(KERN_ERR "[CAMDRV/SR030PC300] SENSOR_PREVIEW_MODE START\n");

    if(!sr030pc30_ctrl->dtp_mode) {
        if(sr030pc30_ctrl->vtcall_mode) {
            SR030PC30_WRITE_LIST(sr030pc30_init_vt_reg);
        } else {
            SR030PC30_WRITE_LIST(sr030pc30_init_reg);
        }
        msleep(300);
    }
}
void sr030pc30_set_snapshot(void)
{

    unsigned short value =0;
    int shade_value = 0;
    unsigned short agc_value = 0;
    
    printk(KERN_ERR "[CAMDRV/SR030PC300] SENSOR_SNAPSHOT_MODE START\n");        
    printk("<=PCAM=> SENSOR_SNAPSHOT_MODE\n");

    SR030PC30_WRITE_LIST(reg_self_capture_table);
    msleep(200);
  
}

static long sr030pc30_set_sensor_mode(int mode)
{
    switch (mode) 
    {
        case SENSOR_PREVIEW_MODE:
            sr030pc30_set_prevew();
            break;
        case SENSOR_SNAPSHOT_MODE:
            sr030pc30_set_snapshot();
            break;
        case SENSOR_SNAPSHOT_TRANSFER:
            printk(KERN_ERR "[CAMDRV/SR030PC300] SENSOR_SNAPSHOT_TRANSFER START\n");
		sr030pc30_set_snapshot();
            break;
        default:
            return -EFAULT;
    }
    return 0;
}

static long sr030pc30_set_effect(int mode,int8_t effect)
{
    long rc = 0;
    switch(effect)
    {
/*        default:
            printk("[Effect]Invalid Effect !!!\n");
            return -EINVAL;
*/
    }
    return rc;
}

static int sr030pc30_reset(void)
{
    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30_reset");

    sr030pc30_set_power(0);
    mdelay(5);
    sr030pc30_set_power(1);
    mdelay(5);
    return 0;
}

static int sr030pc30_set_ev(int8_t ev)
{
    printk(KERN_ERR "[CAMDRV/SR030PC300] ev : %d \n",ev);

    switch(ev)
    {
        case  SR030PC30_EV_MINUS_4:
            SR030PC30_WRITE_LIST(sr030pc30_ev_m4);
        break;
        
        case  SR030PC30_EV_MINUS_3:
            SR030PC30_WRITE_LIST(sr030pc30_ev_m3);
        break;
        
        case  SR030PC30_EV_MINUS_2:
            SR030PC30_WRITE_LIST(sr030pc30_ev_m2);
        break;
        
        case  SR030PC30_EV_MINUS_1:
            SR030PC30_WRITE_LIST(sr030pc30_ev_m1);
        break;
        
        case  SR030PC30_EV_DEFAULT:
            SR030PC30_WRITE_LIST(sr030pc30_ev_default);
        break;
        
        case  SR030PC30_EV_PLUS_1:
            SR030PC30_WRITE_LIST(sr030pc30_ev_p1);
        break;
        
        case  SR030PC30_EV_PLUS_2:
            SR030PC30_WRITE_LIST(sr030pc30_ev_p2);
        break;    
        
        case  SR030PC30_EV_PLUS_3:
            SR030PC30_WRITE_LIST(sr030pc30_ev_p3);
        break;
        
        case  SR030PC30_EV_PLUS_4:
            SR030PC30_WRITE_LIST(sr030pc30_ev_p4);
        break;
        
        default:
            printk("[EV] Invalid EV !!!\n");
            return -EINVAL;
    }

    return 0;
}

static int sr030pc30_set_dtp(int onoff)
{
    printk(KERN_ERR "[CAMDRV/SR030PC300] dtp onoff : %d ",onoff);

    switch(onoff)
    {
        case  SR030PC30_DTP_OFF:
            if(sr030pc30_ctrl->dtp_mode)sr030pc30_reset();
            sr030pc30_ctrl->dtp_mode = 0;
            sr030pc30_set_sensor_mode(SENSOR_PREVIEW_MODE);
            break;
        
        case SR030PC30_DTP_ON:
            //sr030pc30_reset();
            sr030pc30_ctrl->dtp_mode = 1;
            SR030PC30_WRITE_LIST(sr030pc30_dataline);
            break;
        
        default:
            printk("[DTP]Invalid DTP mode!!!\n");
            return -EINVAL;
    }
    return 0;
}

static int sr030pc30_set_fps_mode(unsigned int mode)
{
    printk(KERN_ERR "[CAMDRV/SR030PC300]  %s -mode : %d \n",__FUNCTION__,mode);

    if(mode) { //fixed
        printk(KERN_ERR "[CAMDRV/SR030PC300] mode change to CAMCORDER_MODE");
        sr030pc30_ctrl->vtcall_mode = 1;
    } else { //auto
        printk(KERN_ERR "[CAMDRV/SR030PC300] mode change to CAMERA_MODE");
        sr030pc30_ctrl->vtcall_mode = 0;
    }
    //we will use a auto mode on i9001 project. 2011-04-05
    sr030pc30_ctrl->vtcall_mode = 0;
    return 0;
}

static int sr030pc30_set_blur(unsigned int vt_mode, unsigned int blurlevel)
{
    int err = -EINVAL;
    printk(KERN_ERR "[CAMDRV/SR030PC300] vt_mode : %d, blur [%d] \n", vt_mode, blurlevel);

    if(vt_mode == 1) {
        switch(blurlevel)
        {
            case BLUR_LEVEL_0:
                SR030PC30_WRITE_LIST(sr030pc30_blur_vt_none);
                break;
            case BLUR_LEVEL_1:
                SR030PC30_WRITE_LIST(sr030pc30_blur_vt_p1);
                break;
            case BLUR_LEVEL_2:
                SR030PC30_WRITE_LIST(sr030pc30_blur_vt_p2);
                break;
            case BLUR_LEVEL_3:
                SR030PC30_WRITE_LIST(sr030pc30_blur_vt_p3);
                break;
            default:
                printk(KERN_ERR "[CAMDRV/SR030PC300] %s: Not Support value \n", __func__);
                err = 0;
                break;

        }
    } else {
        switch(blurlevel)
        {
            case BLUR_LEVEL_0:
                SR030PC30_WRITE_LIST(sr030pc30_blur_none);
                break;
            case BLUR_LEVEL_1:
                SR030PC30_WRITE_LIST(sr030pc30_blur_p1);
                break;
            case BLUR_LEVEL_2:
                SR030PC30_WRITE_LIST(sr030pc30_blur_p2);
                break;
            case BLUR_LEVEL_3:
                SR030PC30_WRITE_LIST(sr030pc30_blur_p3);
                break;

            default:
                printk(KERN_ERR "[CAMDRV/SR030PC300] %s: Not Support value \n", __func__);
                err = 0;
                break;
        }
    }
    return err;
}

static int sr030pc30_start(void)
{
    int rc = 0;
    u8 data[2] = {0xEF, 0x01};
    u8 vender[1] = {0xC5};
    u8 version[1] = {0xC6};
    u8 vendor_id = 0xff, sw_ver = 0xff;    
    
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__func__);
	#if 0
    rc = sr030pc30_i2c_write(0xEF, 0x01);
    rc = sr030pc30_i2c_write_read(1, vender, 1, &vendor_id);
    rc = sr030pc30_i2c_write(0xEF, 0x01);    
    rc = sr030pc30_i2c_write_read(1, version, 1, &sw_ver);

    printk("[CAMDRV/SR030PC300]=================================\n");
    printk("[CAMDRV/SR030PC300]  [VGA CAM] vendor_id ID : 0x%x\n", vendor_id);
    printk("[CAMDRV/SR030PC300]  [VGA CAM] software version : 0x%x\n", sw_ver);
    printk("[CAMDRV/SR030PC300]=================================\n");
#endif
    return rc;
}

static int sr030pc30_sensor_init_probe(struct msm_camera_sensor_info *data)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30_sensor_init_probe start");
    sr030pc30_set_power(1);
    sr030pc30_start();

    return rc;

init_probe_fail:
    return rc;
}

int sr030pc30_sensor_init(struct msm_camera_sensor_info *data)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__func__);
    
    sr030pc30_ctrl = kzalloc(sizeof(struct sr030pc30_ctrl_t), GFP_KERNEL);
    if (!sr030pc30_ctrl) {
        CDBG("sr030pc30_sensor_init failed!\n");
        rc = -ENOMEM;
        goto init_done;
    }    
    
    if (data)
        sr030pc30_ctrl->sensordata = data;

    /* Input MCLK = 24MHz */
    msm_camio_clk_rate_set(24000000);
    mdelay(5);

    msm_camio_camif_pad_reg_reset();

#ifdef CONFIG_LOAD_FILE
    sr030pc30_regs_table_init();
#endif

    rc = sr030pc30_sensor_init_probe(data);
    if (rc < 0) {
        CDBG("sr030pc30_sensor_init failed!\n");
        goto init_fail;
    }

init_done:
    return rc;

init_fail:
    kfree(sr030pc30_ctrl);
    return rc;
}

static int sr030pc30_init_client(struct i2c_client *client)
{
    /* Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&sr030pc30_wait_queue);
    return 0;
}

int sr030pc30_sensor_ext_config(void __user *argp)
{
    sensor_ext_cfg_data        cfg_data;
    int rc=0;

    if(copy_from_user((void *)&cfg_data, (const void *)argp, sizeof(cfg_data))) {
        printk(KERN_ERR "[CAMDRV/SR030PC300] %s fail copy_from_user!\n", __func__);
    }
    
    switch(cfg_data.cmd) {
        case EXT_CFG_SET_BRIGHTNESS:
            printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_BRIGHTNESS (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            rc = sr030pc30_set_ev(cfg_data.value_1);
            break;
        case EXT_CFG_SET_DTP:
            printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_DTP (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            rc = sr030pc30_set_dtp(cfg_data.value_1);
            if(cfg_data.value_1 == 0) {
                cfg_data.value_1 = 2;
            } else if(cfg_data.value_1 == 1) {
                cfg_data.value_1 = 3;
            }        
            break;
        case EXT_CFG_SET_FPS_MODE:
            printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_FPS_MODE (%d %d)\n",cfg_data.cmd,cfg_data.value_1);

            rc = sr030pc30_set_fps_mode(cfg_data.value_1);
            break;
        case EXT_CFG_SET_BLUR:
            printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_BLUR (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            rc = sr030pc30_set_blur(sr030pc30_ctrl->vtcall_mode, cfg_data.value_1);
            break;
        case EXT_CFG_SET_FRONT_CAMERA_MODE:
            printk(KERN_ERR "[CAMDRV/SR030PC300] EXT_CFG_SET_FRONT_CAMERA_MODE (%d %d)\n",cfg_data.cmd,cfg_data.value_1);
            sr030pc30_ctrl->vtcall_mode = cfg_data.value_1;
            break;
        default:
            break;
    }

    if(copy_to_user((void *)argp, (const void *)&cfg_data, sizeof(cfg_data))) {
        printk(" %s : copy_to_user Failed \n", __func__);
    }
    
    return rc;    
}

int sr030pc30_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg_data;
    long   rc = 0;

    if (copy_from_user(
                &cfg_data,
                (void *)argp,
                sizeof(struct sensor_cfg_data)))
        return -EFAULT;

    CDBG("sr030pc30_sensor_config, cfgtype = %d, mode = %d\n",
        cfg_data.cfgtype, cfg_data.mode);

    switch (cfg_data.cfgtype) {
    case CFG_SET_MODE:
        rc = sr030pc30_set_sensor_mode(cfg_data.mode);
        break;

    case CFG_SET_EFFECT:
        rc = sr030pc30_set_effect(cfg_data.mode, cfg_data.cfg.effect);
        break;
        
    default:
        rc = -EFAULT;
        break;
    }
    return rc;
}

static int sr030pc30_sensor_release(void)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__FUNCTION__);

    sr030pc30_set_power(0);
    kfree(sr030pc30_ctrl);
#ifdef CONFIG_LOAD_FILE
    sr030pc30_regs_table_exit();
#endif
    return rc;
}

static int sr030pc30_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__FUNCTION__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        rc = -ENOTSUPP;
        goto probe_failure;
    }

    sr030pc30_sensorw =
        kzalloc(sizeof(struct sr030pc30_work_t), GFP_KERNEL);

    if (!sr030pc30_sensorw) {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, sr030pc30_sensorw);
    sr030pc30_init_client(client);
    sr030pc30_client = client;

    CDBG("sr030pc30_i2c_probe successed!\n");
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s X\n",__FUNCTION__);

    return 0;

probe_failure:
    kfree(sr030pc30_sensorw);
    sr030pc30_sensorw = NULL;
    CDBG("sr030pc30_i2c_probe failed!\n");
    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30_i2c_probe failed!\n");
    return rc;
}

static int __exit sr030pc30_i2c_remove(struct i2c_client *client)
{
    struct sr030pc30_work_t *sensorw = i2c_get_clientdata(client);
    free_irq(client->irq, sensorw);
    //i2c_detach_client(client);
    sr030pc30_client = NULL;
    sr030pc30_sensorw = NULL;
    kfree(sensorw);
    return 0;
}

static const struct i2c_device_id sr030pc30_id[] = {
    { "sr030pc30_i2c", 0 },
    { }
};

static struct i2c_driver sr030pc30_i2c_driver = {
    .id_table    = sr030pc30_id,
    .probe      = sr030pc30_i2c_probe,
    .remove     = __exit_p(sr030pc30_i2c_remove),
    .driver     = {
        .name = "sr030pc30",
    },
};

int32_t sr030pc30_i2c_init(void)
{
    int32_t rc = 0;

    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__FUNCTION__);
    rc = i2c_add_driver(&sr030pc30_i2c_driver);

    if (IS_ERR_VALUE(rc))
        goto init_failure;

    return rc;

init_failure:
    printk(KERN_ERR "[CAMDRV/SR030PC300] failed to sr030pc30_i2c_init, rc = %d\n", rc);
    i2c_del_driver(&sr030pc30_i2c_driver);
    return rc;
}

int sr030pc30_sensor_probe(const struct msm_camera_sensor_info *info,    struct msm_sensor_ctrl *s)
{
    int rc = 0;
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s E\n",__FUNCTION__);

    rc = sr030pc30_i2c_init();
    if (rc < 0)
        goto probe_fail;

    /* Input MCLK = 24MHz */
    msm_camio_clk_rate_set(24000000);
    mdelay(5);

 //   if (!lp8720_init){
    //    rc = -EIO;
   //     goto probe_fail;
 //   }

    s->s_init        = sr030pc30_sensor_init;
    s->s_release    = sr030pc30_sensor_release;
    s->s_config    = sr030pc30_sensor_config;

    printk(KERN_ERR "[CAMDRV/SR030PC300] sr030pc30 sensor probe successful\n");
    return rc;

probe_fail:
    printk(KERN_ERR "[CAMDRV/SR030PC300] %s:probe failed\n", __func__);
    return rc;
}

static int __init __sr030pc30_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, sr030pc30_sensor_probe);
}

static struct platform_driver msm_vga_camera_driver = {
    .probe = __sr030pc30_probe,
    .driver = {
        .name = "msm_camera_sr030pc30",
        .owner = THIS_MODULE,
    },
};

static int __init sr030pc30_init(void)
{
    printk(KERN_INFO "[CAMDRV/SR030PC300] %s: E\n", __func__);
    return platform_driver_register(&msm_vga_camera_driver);
}

module_init(sr030pc30_init);
MODULE_DESCRIPTION("LSI sr030pc30 VGA camera driver");
MODULE_LICENSE("GPL v2");

