/*
 *  DV700 kernel driver
 *  Copyright (C) 2018  Digital Media Professionals Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define DRM_DEV_NAME "dmp-dv"
#define DRM_NUM_SUBDEV 2

#ifdef DMP_ZC706
#define USE_DEVTREE
// map {0=CNV,1=FC} to irq Offset:
#ifdef USE_DEVTREE
static int sd2i_map[2] = { 0, 2 };
#else
static int sd2i_map[2] = { 61, 63 };
#endif
// map {0=CNV,1=FC} to regBaseAddr:
static unsigned int sd2rb[2] = { 0x43c00000, 0x43c20000 };
#endif
#ifdef DMP_ARRIA10
//#define USE_DEVTREE
// map {0=CNV,1=FC} to irq Offset:
#ifdef USE_DEVTREE
static int sd2i_map[2] = { 2, 3 };
#else
static int sd2i_map[2] = { 53, 54 };
#endif
// map {0=CNV,1=FC} to regBaseAddr:
static unsigned int sd2rb[2] = { 0xff210000, 0xff200000 };
#endif
// map {0=CNV,1=FC} to regSize:
static unsigned int sd2rs[2] = { 0x2000, 0x100 };

static const char *subdev_name[2] = { "conv", "fc" };

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/stddef.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#ifdef USE_DEVTREE
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#define DMP_IOC_MAJOR 0x82

#define REG_IO_ADDR(DV, OF) ((void __iomem *)(DV->bar_logical) + OF)

struct dmp_dev {
	spinlock_t int_exclusive;
	wait_queue_head_t int_status_wait;
	int irqno;
	int init_done;
	int int_status;

	unsigned int bar_physical;
	unsigned int bar_size;
	void *bar_logical;

	dma_addr_t cmb_physical;
	void *cmb_logical;
};

struct drm_dev {
	struct device *dev;
	dev_t devt;
	struct cdev cdev;

	struct dmp_dev subdev[DRM_NUM_SUBDEV];
};

static struct class *dddrm_class = NULL;

static irqreturn_t handle_int_conv(int irq, void *dev_id)
{
	struct dmp_dev *subdev = dev_id;

	spin_lock(&subdev->int_exclusive);
	iowrite32(0, REG_IO_ADDR(subdev, 0x420));

	subdev->int_status = 2;
	wake_up_interruptible(&subdev->int_status_wait);
	spin_unlock(&subdev->int_exclusive);

	return IRQ_HANDLED;
}

static irqreturn_t handle_int_fc(int irq, void *dev_id)
{
	struct dmp_dev *subdev = dev_id;

	spin_lock(&subdev->int_exclusive);
	iowrite32(0, REG_IO_ADDR(subdev, 0x20));

	subdev->int_status = 2;
	wake_up_interruptible(&subdev->int_status_wait);
	spin_unlock(&subdev->int_exclusive);

	return IRQ_HANDLED;
}

static void wait_int(struct dmp_dev *subdev)
{
	long ret = 0;
	int wStat = 0;
	unsigned long irq_save = 0;

	spin_lock(&subdev->int_exclusive);
	wStat = subdev->int_status;
	subdev->int_status = (wStat == 2) ? 0 : 1;
	spin_unlock(&subdev->int_exclusive);

	if (wStat != 2) {
		ret = wait_event_interruptible(subdev->int_status_wait,
					       (subdev->int_status & 2));
		if (!ret) {
			spin_lock_irqsave(&subdev->int_exclusive, irq_save);
			subdev->int_status = 0;
			spin_unlock_irqrestore(&subdev->int_exclusive,
					       irq_save);
		}
	}
}

static int drm_open(struct inode *inode, struct file *file)
{
	struct drm_dev *drm_dev;
	struct dmp_dev *subdev;
	int ret = 0;
	unsigned int minor;

	minor = iminor(inode);
	drm_dev = container_of(inode->i_cdev, struct drm_dev, cdev);
	subdev = kmalloc(sizeof(*subdev), GFP_KERNEL);
	if (!subdev) {
		ret = -ENOMEM;
		goto drm_open_fail;
	}

	*subdev = drm_dev->subdev[minor];
	subdev->cmb_logical = dma_alloc_coherent(
		drm_dev->dev, PAGE_SIZE, &subdev->cmb_physical, GFP_USER);
	if (!subdev->cmb_logical) {
		ret = -ENOMEM;
		kfree(subdev);
	}

	file->private_data = subdev;

drm_open_fail:
	return ret;
}

static int drm_release(struct inode *inode, struct file *file)
{
	struct dmp_dev *subdev = file->private_data;
	kfree(subdev);
	return 0;
}

static long drm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	//struct dmp_dev *subdev = file->private_data;

	switch (cmd) {
	default:
		break;
	}

	return ret;
}

static struct file_operations drm_file_operations = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
};

int drm_register_chrdev(struct drm_dev *drm_dev)
{
	int err = 0, rIRQ = 0, i = 0;
	struct device *dev;
	unsigned int driver_major;

	dev_dbg(drm_dev->dev, "drm_register_chrdev\n");

	err = alloc_chrdev_region(&drm_dev->devt, 0, DRM_NUM_SUBDEV,
				  DRM_DEV_NAME);
	if (err) {
		dev_err(drm_dev->dev, "alloc_chrdev_region fail\n");
		goto fail_alloc_chrdev_region;
	}

	dddrm_class = class_create(THIS_MODULE, DRM_DEV_NAME);
	if (IS_ERR(dddrm_class)) {
		err = PTR_ERR(dddrm_class);
		dev_err(drm_dev->dev, "class_create fail\n");
		goto fail_class_create;
	}

	driver_major = MAJOR(drm_dev->devt);

	// create char device:
	cdev_init(&drm_dev->cdev, &drm_file_operations);
	err = cdev_add(&drm_dev->cdev, drm_dev->devt, DRM_NUM_SUBDEV);
	if (err) {
		dev_err(drm_dev->dev, "cdev_add fail\n");
		goto fail_cdev_add;
	}

	for (i = 0; i < DRM_NUM_SUBDEV; i++) {
		// Create device:
		dev = device_create(dddrm_class, NULL, MKDEV(driver_major, i),
				    drm_dev, DRM_DEV_NAME "%s", subdev_name[i]);
		if (IS_ERR(dev)) {
			err = PTR_ERR(dev);
			dev_err(drm_dev->dev, "device_create fail %d\n", i);
			goto fail_device_init;
		}

		rIRQ = drm_dev->subdev[i].irqno;
		if (i == 0) {
			if (!err)
				err = request_irq(rIRQ, handle_int_conv,
						  IRQF_SHARED, DRM_DEV_NAME,
						  &(drm_dev->subdev[i]));
		} else {
			if (!err)
				err = request_irq(rIRQ, handle_int_fc,
						  IRQF_SHARED, DRM_DEV_NAME,
						  &(drm_dev->subdev[i]));
		}

		if (err) {
			device_destroy(dddrm_class, MKDEV(driver_major, i));
			dev_err(drm_dev->dev,
				"request_irq FAIL: IRQ=%d ERR=%d\n", rIRQ, err);
			goto fail_device_init;
		}

		// initialize:
		init_waitqueue_head(&(drm_dev->subdev[i].int_status_wait));
		spin_lock_init(&(drm_dev->subdev[i].int_exclusive));
		drm_dev->subdev[i].init_done = 1;
	}

	return 0;

fail_device_init:
	for (i = 0; i < DRM_NUM_SUBDEV; i++) {
		if (drm_dev->subdev[i].init_done) {
			free_irq(drm_dev->subdev[i].irqno,
				 &(drm_dev->subdev[i]));
			device_destroy(dddrm_class, MKDEV(driver_major, i));
			drm_dev->subdev[i].init_done = 0;
		}
	}
	cdev_del(&drm_dev->cdev);
fail_cdev_add:
	class_destroy(dddrm_class);
fail_class_create:
	unregister_chrdev_region(drm_dev->devt, DRM_NUM_SUBDEV);
fail_alloc_chrdev_region:

	return err;
}

int drm_unregister_chrdev(struct drm_dev *drm_dev)
{
	int i;
	unsigned int driver_major = MAJOR(drm_dev->devt);
	dev_dbg(drm_dev->dev, "drm_unregister_chrdev\n");

	for (i = 0; i < DRM_NUM_SUBDEV; i++) {
		if (drm_dev->subdev[i].init_done) {
			free_irq(drm_dev->subdev[i].irqno,
				 &(drm_dev->subdev[i]));
			device_destroy(dddrm_class, MKDEV(driver_major, i));
			drm_dev->subdev[i].init_done = 0;
		}
	}

	cdev_del(&drm_dev->cdev);
	class_destroy(dddrm_class);
	unregister_chrdev_region(drm_dev->devt, DRM_NUM_SUBDEV);
	return 0;
}

static int drm_dev_probe(struct platform_device *pdev)
{
	int i = 0, err = 0;
	struct drm_dev *drm_dev;
#ifdef USE_DEVTREE
	struct device_node *devNode;
#endif

	dev_dbg(&pdev->dev, "probe begin\n");
	drm_dev = kzalloc(sizeof(struct drm_dev), GFP_KERNEL);
	if (!drm_dev) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "kzalloc fail\n");
		goto fail_probe_kzalloc;
	}

	platform_set_drvdata(pdev, drm_dev);
	drm_dev->dev = &pdev->dev;

	if (dma_set_mask_and_coherent(drm_dev->dev, DMA_BIT_MASK(32))) {
		printk(KERN_INFO "==>dmp-dv: no suitable DMA available(?!?)\n");
		err = -ENOMEM;
		goto fail_dma_set_mask;
	}

#ifdef USE_DEVTREE
	devNode = of_find_compatible_node(NULL, NULL, "dmp-dv,dmp-dv");
	if (devNode == NULL) {
		err = -ENODEV;
		dev_err(&pdev->dev, "no compatible node!\n");
		goto fail_dma_set_mask;
	} else {
		of_node_put(devNode);
	}
#endif

	for (i = 0; i < DRM_NUM_SUBDEV; i++) {
#ifdef USE_DEVTREE
		drm_dev->subdev[i].irqno = of_irq_get(devNode, sd2i_map[i]);
// NOTE: we could(should) get the reg. address by this method too ...
#else
		drm_dev->subdev[i].irqno = sd2i_map[i];
#endif

		drm_dev->subdev[i].bar_physical = sd2rb[i];
		drm_dev->subdev[i].bar_size = sd2rs[i];
		drm_dev->subdev[i].bar_logical =
			ioremap_nocache(drm_dev->subdev[i].bar_physical,
					drm_dev->subdev[i].bar_size);
		if (!drm_dev->subdev[i].bar_logical) {
			err = -EBUSY;
			dev_err(&pdev->dev, "ioremap_nocache fail %d\n", i);
			goto fail_get_iomap;
		}

		drm_dev->subdev[i].init_done = 0;
		drm_dev->subdev[i].int_status = 0;
		drm_dev->subdev[i].cmb_physical = 0;
		drm_dev->subdev[i].cmb_logical = 0;
	}

	err = drm_register_chrdev(drm_dev);
	if (err) {
		dev_err(&pdev->dev, "register chrdev fail\n");
		goto fail_register_chrdev;
	}
	dev_dbg(&pdev->dev, "probe successful\n");

	return 0;

fail_register_chrdev:

fail_get_iomap:
	for (i = 0; i < DRM_NUM_SUBDEV; i++) {
		if (drm_dev->subdev[i].bar_logical) {
			iounmap(drm_dev->subdev[i].bar_logical);
		}
	}

fail_dma_set_mask:
	platform_set_drvdata(pdev, NULL);
	kfree(drm_dev);

fail_probe_kzalloc:
	return err;
}

static int drm_dev_remove(struct platform_device *pdev)
{
	struct drm_dev *drm_dev;
	int i;

	dev_dbg(&pdev->dev, "remove begin\n");
	drm_dev = platform_get_drvdata(pdev);

	if (drm_dev) {
		drm_unregister_chrdev(drm_dev);
		for (i = 0; i < DRM_NUM_SUBDEV; i++) {
			if (drm_dev->subdev[i].bar_logical) {
				iounmap(drm_dev->subdev[i].bar_logical);
			}
		}

		platform_set_drvdata(pdev, NULL);
		kfree(drm_dev);
	}

	dev_dbg(&pdev->dev, "remove successful\n");
	return 0;
}

static struct platform_driver drm_platform_driver = {
	.probe = drm_dev_probe,
	.remove = drm_dev_remove,
	.driver =
		{
			.name = DRM_DEV_NAME,
			.owner = THIS_MODULE,
		},
};

static void drm_dev_release(struct device *dev)
{
}

static struct platform_device drm_platform_device = {
	.name = DRM_DEV_NAME,
	.id = -1,
	.num_resources = 0,
	.resource = NULL,
	.dev =
		{
			.release = drm_dev_release,
		},
};

static int __init drm_init(void)
{
	int ret;
	ret = platform_driver_register(&drm_platform_driver);
	if (ret)
		return ret;

	ret = platform_device_register(&drm_platform_device);
	if (ret) {
		platform_driver_unregister(&drm_platform_driver);
	}
	return ret;
}

static void __exit drm_exit(void)
{
	platform_device_unregister(&drm_platform_device);
	platform_driver_unregister(&drm_platform_driver);
}

module_init(drm_init);
module_exit(drm_exit);

MODULE_DESCRIPTION("DV core driver");
MODULE_AUTHOR("Digital Media Professionals Inc.");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
