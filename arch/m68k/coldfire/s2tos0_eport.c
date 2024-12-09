// SPDX-License-Identifier: GPL-2.0
/*
 * s2tos0_eport.c -- EPORT4 driver for S2TOS0
 *
 * Copyright (C) 2024, Jean-Michel Hautbois <jeanmichel.hautbois@yoseli.org>
 *
 */

#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/* Configuration */
#define S2TOS0_EPORT_DEVICE_NAME    "s2tos0"
#define CLASS_NAME                  "s2tos0_eport"

struct s2tos0_drv_data {
	struct dentry *debugfs_direntry;
	int s2tos0_eport_major;
	struct class *s2tos0_class;
	struct device *s2tos0_device;

	struct completion s2tos0_complete;

	int s2tos0_reg;
	int s2tos0_irq;

	wait_queue_head_t s2tos0_eport_waitq;

	local_lock_t lock;
};

static struct s2tos0_drv_data *drv_data;

static inline u8 sync_s2tos0_get(void)
{
	/* C4 => 3*8 + 4 */
	gpio_request(drv_data->s2tos0_reg, "sync_s2tos0");
	gpio_direction_input(drv_data->s2tos0_reg);
	return (gpio_get_value(drv_data->s2tos0_reg) ? '1' : '0');
}

static ssize_t sync_s2tos0_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	u8 c = sync_s2tos0_get();

	return simple_read_from_buffer(buf, count, ppos, &c, 1);
}

/* IRQ handler */
static irqreturn_t s2tos0_eport_handler(int irq, void *dev_id)
{
	complete(&drv_data->s2tos0_complete);
	return IRQ_HANDLED;
}

/* File operations */
static int s2tos0_eport_open(struct inode *inode, struct file *filp)
{
	complete(&drv_data->s2tos0_complete);
	return 0;
}

static int s2tos0_eport_close(struct inode *inode, struct file *filp)
{
	complete(&drv_data->s2tos0_complete);
	return 0;
}

static ssize_t s2tos0_eport_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset)
{
	char msg[2];
	int count, ret;

	if (length < 1)
		return -EINVAL;

	ret = wait_for_completion_interruptible(&drv_data->s2tos0_complete);
	if (ret < 0)
		return ret;

	local_lock(&drv_data->lock);
	msg[0] = sync_s2tos0_get();

	if (length >= 2) {
		msg[1] = '\n';
		count = 2;
	} else
		count = 1;

	trace_printk("s2tos0_eport_read: %c\n", msg[0]);
	if (__copy_to_user_inatomic(buffer, msg, count))
		return -EFAULT;

	local_unlock(&drv_data->lock);

	reinit_completion(&drv_data->s2tos0_complete);
	return count;
}

static const struct file_operations s2tos0_fops = {
	.owner = THIS_MODULE,
	.open = s2tos0_eport_open,
	.read = s2tos0_eport_read,
	.release = s2tos0_eport_close,
};

static const struct file_operations debugfs_sync_s2tos0_fops = {
	.read = sync_s2tos0_read,
};

static int s2tos0_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	int retval;

	drv_data = devm_kzalloc(&pdev->dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	drv_data->s2tos0_irq = platform_get_irq(pdev, 0);
	if (drv_data->s2tos0_irq < 0)
		return drv_data->s2tos0_irq;

	res = platform_get_resource_byname(pdev, IORESOURCE_REG, "s2tos0_reg");
	if (!res)
		return -ENODEV;

	drv_data->s2tos0_reg = res->start;
	init_completion(&drv_data->s2tos0_complete);

	/* Register char device */
	drv_data->s2tos0_eport_major = register_chrdev(0, S2TOS0_EPORT_DEVICE_NAME, &s2tos0_fops);
	if (drv_data->s2tos0_eport_major < 0)
		return drv_data->s2tos0_eport_major;

	drv_data->s2tos0_class = class_create(S2TOS0_EPORT_DEVICE_NAME);
	if (IS_ERR(drv_data->s2tos0_class)) {
		unregister_chrdev(drv_data->s2tos0_eport_major, S2TOS0_EPORT_DEVICE_NAME);
		return PTR_ERR(drv_data->s2tos0_class);
	}

	drv_data->s2tos0_device = device_create(drv_data->s2tos0_class, NULL,
						MKDEV(drv_data->s2tos0_eport_major, 0),
						NULL, S2TOS0_EPORT_DEVICE_NAME);
	if (IS_ERR(drv_data->s2tos0_device)) {
		class_destroy(drv_data->s2tos0_class);
		unregister_chrdev(drv_data->s2tos0_eport_major, S2TOS0_EPORT_DEVICE_NAME);
		return PTR_ERR(drv_data->s2tos0_device);
	}

	retval = request_irq(drv_data->s2tos0_irq, s2tos0_eport_handler,
			     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			     S2TOS0_EPORT_DEVICE_NAME, NULL);
	if (retval) {
		device_destroy(drv_data->s2tos0_class, MKDEV(drv_data->s2tos0_eport_major, 0));
		class_destroy(drv_data->s2tos0_class);
		unregister_chrdev(drv_data->s2tos0_eport_major, S2TOS0_EPORT_DEVICE_NAME);
		return retval;
	}

	drv_data->debugfs_direntry = debugfs_create_dir(CLASS_NAME, NULL);
	if (!debugfs_create_file("sync_s2tos0", 0200, drv_data->debugfs_direntry, drv_data, &debugfs_sync_s2tos0_fops))
		dev_warn(dev, "Unable to create %s entry\n", "sync_s2tos0");

	init_waitqueue_head(&drv_data->s2tos0_eport_waitq);

	printk(KERN_INFO "DSPI: Coldfire S2toS0 initialized\n");
	return 0;
}

static void s2tos0_remove(struct platform_device *pdev)
{
	free_irq(drv_data->s2tos0_irq, NULL);
	device_destroy(drv_data->s2tos0_class, MKDEV(drv_data->s2tos0_eport_major, 0));
	class_destroy(drv_data->s2tos0_class);
	unregister_chrdev(drv_data->s2tos0_eport_major, S2TOS0_EPORT_DEVICE_NAME);
}

static struct platform_driver s2tos0_eport_platform_driver = {
	.probe	= s2tos0_probe,
	.remove	= s2tos0_remove,
	.driver		= {
		.name	= "s2tos0_eport",
	},
};
module_platform_driver(s2tos0_eport_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jean-Michel Hautbois");
MODULE_DESCRIPTION("Standalone driver for EPORT4 handling");
