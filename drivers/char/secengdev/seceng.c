#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/ioctl.h>

#define SECENG_SIZE 0x4000
#define SECENG_BASE_ADDR 0xFD204000

#define REG_BASE_SCTRL	(0xFC802000)
#define SCPEREN3		(REG_BASE_SCTRL + 0x50)
#define SCPEREN0		(REG_BASE_SCTRL + 0x20)
#define SCPERRSTEN3	    (REG_BASE_SCTRL + 0xA4)
#define SCPERRSTDIS3	(REG_BASE_SCTRL + 0xA8)
#define SYS_SSDMA_RST_BIT (1<<15)


struct cdev cdev;

int seceng_major = 0;
int seceng_minor = 0;
int number_of_devices = 1;

static int seceng_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int seceng_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int seceng_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long seceng_addr;

	if(SECENG_SIZE != (vma->vm_end - vma->vm_start))
		return -EINVAL;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_READ | VM_WRITE;
	vma->vm_flags |= VM_SHARED;

	seceng_addr = SECENG_BASE_ADDR;
	seceng_addr &= ~(PAGE_SIZE - 1);
	seceng_addr &= 0xffffffffUL;

	if (io_remap_pfn_range(vma, vma->vm_start, seceng_addr >> PAGE_SHIFT, SECENG_SIZE, vma->vm_page_prot)) {
		printk(KERN_ERR "remap_pfn_range failed in seceng_mmap\n");
		return -EAGAIN;
	}

	return 0;
}

#define SECENG_IOC_MAGIC  's' /* Use 's' as magic number */
#define SECENG_IOCXPID    _IOWR(SECENG_IOC_MAGIC, 0, pid_t) /* X means "eXchange": exchange pid */
#define SECENG_IOC_MAXNR  0
static pid_t g_secpid = 0;

static long seceng_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    long retval = 0;
    pid_t tmppid = 0;

    if (_IOC_TYPE(cmd) != SECENG_IOC_MAGIC) {
        printk(KERN_ERR "seceng_ioctl: _IOC_TYPE error\n");
        return -ENOTTY;
    }
    if (_IOC_NR(cmd) > SECENG_IOC_MAXNR) {
        printk(KERN_ERR "seceng_ioctl: _IOC_NR error\n");
        return -ENOTTY;
    }

    if (_IOC_DIR(cmd) & _IOC_READ) {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    } else if (_IOC_DIR(cmd) & _IOC_WRITE) {
        err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }
    if (err) {
        printk(KERN_ERR "seceng_ioctl: access_ok error\n");
        return -EFAULT;
    }
    //printk("seceng_ioctl: _IOC_SIZE(cmd) = %d\n", _IOC_SIZE(cmd));

    switch(cmd)
    {
        case SECENG_IOCXPID: /* exchange pid */
            /* if (! capable (CAP_SYS_ADMIN)) return -EPERM; */
            //printk("seceng_ioctl: previous pid = %d\n", g_secpid);
            tmppid = g_secpid;
            retval = __get_user(g_secpid, (pid_t __user *)arg);
            if (retval != 0) {
                printk(KERN_ERR "seceng_ioctl: __get_user error\n");
                g_secpid = 0;
                break;
            }
            //printk("seceng_ioctl: current pid = %d\n", g_secpid);
            retval = __put_user(tmppid, (pid_t __user *)arg);
            if (retval != 0) {
                printk(KERN_ERR "seceng_ioctl: __put_user error\n");
                g_secpid = 0;
                break;
            }
            break;
        default: /* redundant, as cmd was checked against MAXNR */
            printk(KERN_ERR "seceng_ioctl: default error\n");
            return -ENOTTY;
    }
    return retval;
}


struct file_operations seceng_fops = {
	.owner = THIS_MODULE,
	.open = seceng_open,
	.release = seceng_release,
	.unlocked_ioctl = seceng_ioctl,
	.mmap = seceng_mmap,
};

static void seceng_ssdma_reset(void)
{
	writel(0x1 << 15, IO_ADDRESS(SCPEREN3));
	writel(0x1 << 27, IO_ADDRESS(SCPEREN0));
	writel(SYS_SSDMA_RST_BIT, IO_ADDRESS(SCPERRSTEN3));
	udelay(10);
	writel(SYS_SSDMA_RST_BIT, IO_ADDRESS(SCPERRSTDIS3));
	udelay(10);
}

struct class *my_class;

static int seceng_setup(void)
{
	int result;
	int error;
	dev_t dev = 0;

	printk("k3v2 seceng setup\n");

	dev = MKDEV (seceng_major, seceng_minor);

	if (seceng_major)
	{
		result = register_chrdev_region(dev, 1, "k3v2-seceng");
		if (result<0) {
			printk(KERN_WARNING "seceng: can't get major number %d\n", seceng_major);
			return result;
		}
	}
	else
	{
		result = alloc_chrdev_region(&dev, 0, 1, "k3v2-seceng");
		seceng_major = MAJOR(dev);
	}

	cdev_init (&cdev,&seceng_fops);
	cdev.owner = THIS_MODULE;
	cdev.ops = &seceng_fops;
	error = cdev_add (&cdev, dev ,1);
	if (error)
	{
		printk(KERN_NOTICE "Error %d adding char_reg_setup_cdev", error);
	}

	/* create your own class under/sysfs */
	my_class = class_create(THIS_MODULE,"my_class");
	if(IS_ERR(my_class))
	{
		printk("Err:failed in creating class.\n");
		return -1;
	}

	/* register your own devicein sysfs, and this will cause udev to create corresponding device node */
	device_create( my_class, NULL, MKDEV(seceng_major, 0), NULL, "k3v2-seceng");

	printk (KERN_INFO"Registered character driver\n");
	return 0;
}

static void  seceng_destroy(void)
{
	dev_t devno = MKDEV (seceng_major,seceng_minor);
	cdev_del (&cdev);

	device_destroy(my_class,MKDEV(seceng_major, 0));         //deletedevice node under /dev
	class_destroy(my_class);                            			 //delete class created by us

	unregister_chrdev_region(devno, number_of_devices);

	printk (KERN_INFO"char driver cleaned up\n");
}


static int seceng_suspend(struct platform_device *dev, pm_message_t pm)
{
	printk("[k3v2 seceng suspend] suspend successfully\n");
	return 0;
}


static int seceng_resume(struct platform_device *dev)
{
	printk("[k3v2 seceng resume]+\n");
	seceng_ssdma_reset();
	printk("[k3v2 seceng resume]-\n");

	return 0;
}

static int __devinit seceng_probe(struct platform_device *dev)
{
	printk("k3v2 seceng probe\n");

	seceng_ssdma_reset();

	return seceng_setup();
}

static int __devexit seceng_remove(struct platform_device *dev)
{
	seceng_destroy();
	return 0;
}


static struct platform_driver seceng_device_driver = {
	.probe		= seceng_probe,
	.remove		= __devexit_p(seceng_remove),

#ifdef CONFIG_PM
	.suspend		= seceng_suspend,
	.resume 		= seceng_resume,
#endif

	.driver		= {
		.name	= "k3v2_seceng",
		.owner	= THIS_MODULE,
	},
};

static int __init seceng_init(void)
{
	int rv;

	printk("k3v2 seceng_init++\n");

	rv = platform_driver_register(&seceng_device_driver);
	if (rv) {
		printk("Unable to register driver: %d\n", rv);
		return rv;
	}

	printk("k3v2 seceng_init--\n");
	return 0;
}

static void __exit seceng_exit(void)
{
	printk("k3v2 seceng exit++\n");

	platform_driver_unregister(&seceng_device_driver);

	printk("k3v2 seceng exit--\n");
	return;
}

MODULE_LICENSE ("GPL");

module_init (seceng_init);
module_exit (seceng_exit);
