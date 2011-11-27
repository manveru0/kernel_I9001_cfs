
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <asm/unistd.h>
#include <asm/errno.h>
#include <asm/uaccess.h>

#include "param.h"


#include <linux/fcntl.h>

#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/syscalls.h>
#include <linux/irq.h>
#include <asm/system.h>
#include <linux/kernel.h>	/* We're doing kernel work */
#include <linux/unistd.h>	/* The list of system calls */
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>



#define FSR_PROC_DIR "LinuStoreIII"
struct proc_dir_entry *fsr_proc_dir;


#define PARAM_SIZE        	84


// must be same as bootable/bootloader/lk/app/aboot/common.h
typedef struct _param {
	int booting_now;    
	int fota_mode;  
	int b;
	int c;
	int d;
	char efs_info[32];
	char str2[32];
} PARAM;

//FSRPartI pstPartI;
int param_n1stVun;
char mBuf[PARAM_SIZE];
//extern struct proc_dir_entry *fsr_proc_dir;


static ssize_t kernel_write(struct file *file, const char *buf, size_t count, loff_t pos)
{
         mm_segment_t old_fs;
         ssize_t res;

         PARAM dfs; //
		 
 
         old_fs = get_fs();
         set_fs(get_ds());
		 
         /* The cast to a user pointer is valid due to the set_fs() */
         res = vfs_write(file, (const char __user *)buf, count, &pos);

         set_fs(old_fs);
 
         return res;
}




static int param_read_proc_debug(char *page, char **start, off_t offset, int count, int *eof, void *data)
{


       struct file *filp;
	PARAM efs;
	char *dp;
	long l;
	int ret;

	printk("%s %d\n", __func__, __LINE__);


       filp = filp_open("/dev/block/mmcblk0p14", O_RDWR, 0666);
	if (IS_ERR(filp)) {
		printk("file open error\n");
		return;
	}

	*eof = 1;
	memset(mBuf, 0xff, PARAM_SIZE);
	
	
	l = sizeof(PARAM);
	printk("l = %ld\n", l);

	ret = kernel_read(filp, 0, mBuf, l);	


       memcpy(&efs, mBuf, sizeof(PARAM));
	printk("PARAM booting_now : %d\n",efs.booting_now);
	printk("PARAM fota_mode   : %d\n",efs.fota_mode);
	printk("PARAM efs_info	  : %s\n",efs.efs_info);


      filp_close(filp, current->files);


	return sprintf(page, "%s", efs.efs_info);
	
}


 
static int param_write_proc_debug(struct file *file, const char *buffer,
		                            unsigned long count, void *data)
{
       struct file *filp;
	char *buf;
     	PARAM efs;
	long l;
	int ret;

	printk("%s %d\n", __func__, __LINE__);

	if (count < 1)
		return -EINVAL;

	if(count > sizeof(efs.efs_info))
		return -EFAULT;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}


	filp = filp_open("/dev/block/mmcblk0p14", O_RDWR, 0666);
	if (IS_ERR(filp)) {
		printk("file open error\n");
		return;
	}

	memset(mBuf, 0xff, PARAM_SIZE);

       l = sizeof(PARAM);
	printk("l = %ld\n", l);   

       ret = kernel_read(filp, 0, mBuf, l);	
	if(ret<0) {
		printk("Partition READ FAIL!\n");
		return ret;
	}
	

	memcpy(&efs, mBuf, sizeof(PARAM));
	// copy user data to efs
	memset(efs.efs_info, 0x0, sizeof(efs.efs_info));
	memcpy(efs.efs_info, buf, (int)count);
	memcpy(mBuf, &efs, sizeof(PARAM));

	// read first block from param block
       ret=kernel_write(filp, mBuf, l,0);
	if(ret<0) {
		printk("Partition write FAIL!\n");
		return ret;
	}

      filp_close(filp, current->files);

	kfree(buf);
	return count;

}




extern int (*set_recovery_mode)(void);
     
int _set_recovery_mode(void)
{
       struct file *filp;
     	PARAM param;
	long l;
	int ret;

	printk("%s %d\n", __func__, __LINE__);


	filp = filp_open("/dev/block/mmcblk0p14", O_RDWR, 0666);
	if (IS_ERR(filp)) {
		printk("file open error\n");
		return;
	}

	memset(mBuf, 0xff, PARAM_SIZE);

       l = sizeof(PARAM);

       ret = kernel_read(filp, 0, mBuf, l);	
	if(ret<0) {
		printk("Partition READ FAIL!\n");
		return ret;
	}

	memcpy(&param, mBuf, sizeof(PARAM));
	// copy user data to efs
	param.booting_now = RECOVERY_ENTER_MODE;
	memcpy(mBuf,&param,sizeof(PARAM));

	// read first block from param block
       ret=kernel_write(filp, mBuf, l,0);
	if(ret<0) {
		printk("Partition write FAIL!\n");
		return ret;
	}


      filp_close(filp, current->files);

	return 0;

}


static int __init param_init(void)
{

	struct proc_dir_entry *ent;

	fsr_proc_dir = proc_mkdir(FSR_PROC_DIR, NULL);
	if (!fsr_proc_dir)
	{
		printk("proc_mkdir FAIL!\n");
	        return NULL;
	}

	ent = create_proc_entry("efs_info", S_IFREG | S_IWUSR | S_IRUGO, fsr_proc_dir);
	ent->read_proc = param_read_proc_debug;
	ent->write_proc = param_write_proc_debug;

	set_recovery_mode = _set_recovery_mode;

	return 0;

}

static void __exit param_exit(void)
{

     
	remove_proc_entry("efs_info", fsr_proc_dir);

}

module_init(param_init);
module_exit(param_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Samsung Param Operation");
