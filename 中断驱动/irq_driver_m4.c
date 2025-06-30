#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <linux/fcntl.h>

/* irq设备结构体 */
struct irq_dev{
	dev_t irqid;			/* 设备号 	 */
	struct cdev cdev;		/* cdev 	*/
	struct class *class;		/* 类 		*/
	struct device *device;		/* 设备 	 */
	int major;			/* 主设备号	  */
	int minor;			/* 次设备号   */
	struct device_node	*nd; 	/* 设备节点 */
	int irq_gpio;			/* irq所使用的GPIO编号	*/
	atomic_t irqvalue;		/* 按键值 		*/	

	struct fasync_struct *async_queue; /* 异步相关结构体 */
};
struct irq_dev irqdev;			/* irq设备 */

// 定义中断处理函数
static irqreturn_t my_interrupt_handler(int irq, void* dev_id)
{
    struct irq_dev *dev = (struct irq_dev *)dev_id;

    // 发送异步通知
    if (dev->async_queue) {
        kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
    }

    return IRQ_HANDLED;
}

static int irq_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &irqdev;	/* 设置私有数据 */
	return 0;
}

static int irq_fasync(int fd, struct file *filp, int on)
{
	struct irq_dev *dev = (struct irq_dev *)filp->private_data;
	return fasync_helper(fd, filp, on, &dev->async_queue);
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = irq_open,
    .fasync = irq_fasync,
};

static int __init irq_driver_init(void)
{
    int ret;

    /* 1、构建设备号 */
    dev_t dev;
    if (irqdev.major) {
        dev = MKDEV(irqdev.major, 0);
        ret = register_chrdev_region(dev, 1, "loram4_irq_driver");
    } else {
        ret = alloc_chrdev_region(&dev, 0, 1, "loram4_irq_driver");
        irqdev.major = MAJOR(dev);
        irqdev.minor = MINOR(dev);
    }

    if (ret < 0) {
        printk(KERN_ERR "Failed to register char device\n");
        return ret;
    }

    /* 2、注册字符设备 */
    cdev_init(&irqdev.cdev, &fops);
    ret = cdev_add(&irqdev.cdev, dev, 1);
    if (ret < 0) {
        printk(KERN_ERR "Failed to add char device\n");
        unregister_chrdev_region(dev, 1);
        return ret;
    }

    /* 3、创建类 */
    irqdev.class = class_create(THIS_MODULE, "loram4_irq_driver");
    if (IS_ERR(irqdev.class)) {
        printk(KERN_ERR "Failed to create class\n");
        cdev_del(&irqdev.cdev);
        unregister_chrdev_region(dev, 1);
        return PTR_ERR(irqdev.class);
    }

    /* 4、创建设备 */
    irqdev.device = device_create(irqdev.class, NULL, dev, NULL, "loram4_irq_driver");
    if (IS_ERR(irqdev.device)) {
        printk(KERN_ERR "Failed to create device\n");
        class_destroy(irqdev.class);
        cdev_del(&irqdev.cdev);
        unregister_chrdev_region(dev, 1);
        return PTR_ERR(irqdev.device);
    }

    /* 5、初始化 GPIO 和中断 */
    irqdev.nd = of_find_node_by_name(NULL, "loram4"); // 修改为 loram4 或 loram3
    if (!irqdev.nd) {
        printk(KERN_ERR "Failed to find loram4/loram3 node\n");//在设备树中查找名为 "loram4" 或 "loram3" 的节点,之前已在设备书定义好
        return -ENODEV;
    }
    ret = of_get_named_gpio(irqdev.nd, "dio1-gpio", 0);//在loram4节点中查找名为 "dio1-gpio" 的属性，获取DIO1引脚的编号
    if (ret < 0) {
        printk(KERN_ERR "Failed to get GPIO\n");
        device_destroy(irqdev.class, dev);
        class_destroy(irqdev.class);
        cdev_del(&irqdev.cdev);
        unregister_chrdev_region(dev, 1);
        return ret;
    }
    irqdev.irq_gpio = ret;

    ret = gpio_request(irqdev.irq_gpio, "irq_gpio");//gpio_request 函数请求之前获取的GPIO引脚，将其锁定，以确保在使用期间不会被其他部分占用。
    if (ret < 0) {
        printk(KERN_ERR "Failed to request GPIO\n");
        device_destroy(irqdev.class, dev);
        class_destroy(irqdev.class);
        cdev_del(&irqdev.cdev);
        unregister_chrdev_region(dev, 1);
        return ret;
    }

    ret = gpio_direction_input(irqdev.irq_gpio);//DIO1方向为输入，这样它可以被用作中断触发源。
    if (ret < 0) {
        printk(KERN_ERR "Failed to set GPIO direction\n");
        gpio_free(irqdev.irq_gpio);
        device_destroy(irqdev.class, dev);
        class_destroy(irqdev.class);
        cdev_del(&irqdev.cdev);
        unregister_chrdev_region(dev, 1);
        return ret;
    }  

    ret = request_irq(gpio_to_irq(irqdev.irq_gpio), my_interrupt_handler, IRQF_TRIGGER_RISING, "loram4_irq_driver_dio1",&irqdev);
    if (ret < 0) {
        printk(KERN_ERR "Failed to request IRQ\n");//request_irq 函数来注册一个中断处理函数
        gpio_free(irqdev.irq_gpio);
        device_destroy(irqdev.class, dev);
        class_destroy(irqdev.class);
        cdev_del(&irqdev.cdev);
        unregister_chrdev_region(dev, 1);
        return ret;
    }

    printk(KERN_INFO "loram4_irq_driver initialized\n");
    return 0;
}

static void __exit irq_driver_exit(void)
{
     dev_t dev;
    /* 释放中断和 GPIO 资源 */
    int irq_gpio = irqdev.irq_gpio;
    free_irq(gpio_to_irq(irq_gpio), NULL);
    gpio_free(irq_gpio);

    /* 注销字符设备 */
    dev = MKDEV(irqdev.major, 0);
    device_destroy(irqdev.class, dev);
    class_destroy(irqdev.class);
    cdev_del(&irqdev.cdev);
    unregister_chrdev_region(dev, 1);

    printk(KERN_INFO "loram4_irq_driver exited\n");
}

module_init(irq_driver_init);
module_exit(irq_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("IRQ Driver");

