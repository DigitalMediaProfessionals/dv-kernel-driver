/*
 *  DV700 kernel driver
 *
 *  Copyright (C) 2018  Digital Media Professionals Inc.
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

#define USE_DEVTREE

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
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#ifdef USE_DEVTREE
#include <linux/of.h>
#include <linux/of_irq.h>
#endif

#include "dmp-dv.h"
#include "../uapi/dmp-dv.h"

#define REG_IO_ADDR(DV, OF) ((void __iomem *)(DV->bar_logical) + OF)

#ifndef USE_DEVTREE
static int irq_no[2] = { 48, 49 };
static unsigned int reg_base = 0x80000000;
#endif
static unsigned int reg_offset[2] = { 0x0, 0x1000 };
static unsigned int reg_size[2] = { 0x1000, 0x1000 };
static int irq_addr[2] = { 0x420, 0x20 };
static const char *subdev_name[2] = { "conv", "fc" };

#define DEF_UNIFIED_BUFFER_SIZE_KB 640
uint32_t UNIFIED_BUFFER_SIZE = DEF_UNIFIED_BUFFER_SIZE_KB << 10;
uint32_t MAX_CONV_KERNEL_SIZE = 7;
uint32_t MAX_FC_VECTOR_SIZE = 16384;

struct dv_cmd_work_item {
	struct dmp_dev_private *dev_pri;
	void (*run_func)(struct dmp_cmb *, void *);
	uint64_t cmd_id;
	struct work_struct work;
};

struct dmp_dev {
	phys_addr_t bar_physical;
	size_t bar_size;
	void *bar_logical;
	int irq_addr;

	int init_done;
	spinlock_t int_exclusive;
	spinlock_t wq_exclusive;
	wait_queue_head_t wait_queue;
	int irqno;
	uint64_t cmd_id;
	uint64_t hw_id;
	struct workqueue_struct *wq;
};

struct dmp_dev_private {
	struct dmp_dev *dev;
	struct dmp_cmb *cmb;
};

struct drm_dev {
	struct device *dev;
	dev_t devt;
	struct cdev cdev;

	struct dmp_dev subdev[DRM_NUM_SUBDEV];
};

static struct class *dddrm_class = NULL;
static DEFINE_MUTEX(dv_firmware_lock);

static irqreturn_t handle_int(int irq, void *dev_id)
{
	struct dmp_dev *dev = dev_id;

	spin_lock(&dev->int_exclusive);
	iowrite32(0, REG_IO_ADDR(dev, dev->irq_addr));

	++dev->hw_id;
	wake_up_interruptible(&dev->wait_queue);
	spin_unlock(&dev->int_exclusive);

	return IRQ_HANDLED;
}

static int wait_cmd_id(struct dmp_dev *dev, uint64_t cmd_id)
{
	long ret = 0;
	unsigned long irq_save = 0;
	uint64_t hw_id;

	spin_lock_irqsave(&dev->int_exclusive, irq_save);
	hw_id = dev->hw_id;
	spin_unlock_irqrestore(&dev->int_exclusive, irq_save);

	if (hw_id >= cmd_id)
		return 0;

	ret = wait_event_interruptible_timeout(dev->wait_queue,
		(dev->hw_id >= cmd_id), DRM_WAIT_TIMEOUT);

	if (ret > 0)
		return 0;
	else if (ret == 0)
		return -EBUSY;
	return ret;
}

static void cmd_work(struct work_struct *work)
{
	int count = 0;
	struct dv_cmd_work_item	*wo = container_of(work,
		struct dv_cmd_work_item, work);
	struct dmp_dev_private *dev_pri = wo->dev_pri;
	wo->run_func(dev_pri->cmb, dev_pri->dev->bar_logical);
	while (count < DRM_MAX_WAIT_COUNT &&
	       wait_cmd_id(dev_pri->dev, wo->cmd_id) != 0)
		++count;
	kfree(wo);
}

static int drm_open(struct inode *inode, struct file *file)
{
	struct drm_dev *drm_dev;
	struct dmp_dev_private *dev_pri;
	int ret = 0;
	unsigned int minor;

	minor = iminor(inode);
	drm_dev = container_of(inode->i_cdev, struct drm_dev, cdev);
	dev_pri = kmalloc(sizeof(*dev_pri), GFP_KERNEL);
	if (!dev_pri) {
		pr_err(DRM_DEV_NAME ": Failed to allocate sub device data.\n");
		ret = -ENOMEM;
		goto drm_open_fail;
	}

	dev_pri->dev = &drm_dev->subdev[minor];
	dev_pri->cmb = dv_cmb_init(drm_dev->dev);
	if (!dev_pri->cmb) {
		pr_err(DRM_DEV_NAME ": Failed to allocate command buffer.\n");
		ret = -ENOMEM;
		goto drm_allocate_cmb_fail;
	}

	file->private_data = dev_pri;

	return 0;

drm_allocate_cmb_fail:
	kfree(dev_pri);
drm_open_fail:
	return ret;
}

static int drm_release(struct inode *inode, struct file *file)
{
	struct drm_dev *drm_dev;
	struct dmp_dev_private *dev_pri;
	drm_dev = container_of(inode->i_cdev, struct drm_dev, cdev);
	dev_pri = file->private_data;
	flush_workqueue(dev_pri->dev->wq);
	dv_cmb_finalize(drm_dev->dev, dev_pri->cmb);
	kfree(dev_pri);
	return 0;
}

static int (*cmd_func[DRM_NUM_SUBDEV])(struct device *, struct dmp_cmb *,
		struct dmp_dv_kcmd_impl *) = {
	dv_convert_conv_command,
	dv_convert_fc_command,
};

static void (*run_func[DRM_NUM_SUBDEV])(struct dmp_cmb *, void *) = {
	dv_run_conv_command,
	dv_run_fc_command,
};

static long drm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	struct inode *inode = file->f_path.dentry->d_inode;
	struct drm_dev *drm_dev =
		container_of(inode->i_cdev, struct drm_dev, cdev);
	struct dmp_dev_private *dev_pri = file->private_data;
	struct dv_cmd_work_item *wo;
	dmp_dv_kcmd cmd_info;
	uint64_t cmd_id;
	unsigned int minor = iminor(inode);

	switch (cmd) {
	case DMP_DV_IOC_APPEND_CMD:
		if (_IOC_SIZE(cmd) > sizeof(cmd_info))
			return -EINVAL;
		if (copy_from_user(&cmd_info, (void __user *)arg,
				   _IOC_SIZE(cmd)))
			return -EFAULT;
		ret = cmd_func[minor](drm_dev->dev, dev_pri->cmb, &cmd_info);
		break;
	case DMP_DV_IOC_RUN:
		wo = kmalloc(sizeof(*wo), GFP_KERNEL);
		if (!wo)
			return -ENOMEM;
		wo->dev_pri = dev_pri;
		wo->run_func = run_func[minor];
		INIT_WORK(&wo->work, cmd_work);
		spin_lock(&dev_pri->dev->wq_exclusive);
		cmd_id = ++dev_pri->dev->cmd_id;
		if (copy_to_user((void __user *)arg, &cmd_id, _IOC_SIZE(cmd))) {
			spin_unlock(&dev_pri->dev->wq_exclusive);
			return -EFAULT;
		}
		wo->cmd_id = cmd_id;
		queue_work(dev_pri->dev->wq, &wo->work);
		spin_unlock(&dev_pri->dev->wq_exclusive);
		break;
	case DMP_DV_IOC_WAIT:
		if (copy_from_user(&cmd_id, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
		ret = wait_cmd_id(dev_pri->dev, cmd_id);
		break;
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

static ssize_t conv_freq_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct drm_dev *drm_dev = dev_get_drvdata(dev);
	int freq = ioread32(REG_IO_ADDR((&drm_dev->subdev[0]), 0x424));
	return scnprintf(buf, PAGE_SIZE, "%d\n", freq & 0xFF);
}

static ssize_t fc_freq_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct drm_dev *drm_dev = dev_get_drvdata(dev);
	int freq = ioread32(REG_IO_ADDR((&drm_dev->subdev[0]), 0x424));
	return scnprintf(buf, PAGE_SIZE, "%d\n", (freq >> 8) & 0xFF);
}

static ssize_t conv_kick_count_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct drm_dev *drm_dev = dev_get_drvdata(dev);
	int kick_count = ioread32(REG_IO_ADDR((&drm_dev->subdev[0]), 0x0100));
	return scnprintf(buf, PAGE_SIZE, "%d\n", kick_count);
}

static ssize_t ub_size_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", UNIFIED_BUFFER_SIZE);
}

static ssize_t max_kernel_size_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", MAX_CONV_KERNEL_SIZE);
}

static ssize_t max_fc_vector_size_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", MAX_FC_VECTOR_SIZE);
}

static ssize_t drm_firmware_write(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buf, loff_t pos, size_t count)
{
	struct dmp_dev *subdev = bin_attr->private;
	unsigned int len = count / 4;
	uint32_t *f_buf = (uint32_t*)buf;

	if (!subdev)
		return -ENODEV;

	if ((pos + count) > DRM_MAX_FIRMWARE_SIZE)
		return -EINVAL;

	mutex_lock(&dv_firmware_lock);

	if (pos == 0) {
		if (count < 10) {
			// Set firmware to use ROM
			iowrite32(1, REG_IO_ADDR(subdev, 0x44));
			mutex_unlock(&dv_firmware_lock);
			pr_info(DRM_DEV_NAME ": Switch firmware to use ROM.\n");
			return count;
		} else {
			// Set firmware to use ROM
			iowrite32(0, REG_IO_ADDR(subdev, 0x44));
		}
	}

	pr_info(DRM_DEV_NAME ": Updating firmware 0x%04x..0x%04x.\n",
		(unsigned int)pos, (unsigned int)(pos + count));

	iowrite32(pos, REG_IO_ADDR(subdev, 0x80));
	while (len--) {
		iowrite32(*f_buf, REG_IO_ADDR(subdev, 0x84));
		++f_buf;
	}

	mutex_unlock(&dv_firmware_lock);

	return count;
}

static DEVICE_ATTR_RO(conv_freq);
static DEVICE_ATTR_RO(conv_kick_count);
static DEVICE_ATTR_RO(ub_size);
static DEVICE_ATTR_RO(max_kernel_size);

static DEVICE_ATTR_RO(fc_freq);
static DEVICE_ATTR_RO(max_fc_vector_size);

static struct attribute *drm_conv_attrs[] = {
	&dev_attr_conv_freq.attr,
	&dev_attr_conv_kick_count.attr,
	&dev_attr_ub_size.attr,
	&dev_attr_max_kernel_size.attr,
	NULL
};

static struct attribute *drm_fc_attrs[] = {
	&dev_attr_fc_freq.attr,
	&dev_attr_max_fc_vector_size.attr,
	NULL
};

static struct bin_attribute drm_firmware_attr = {
	.attr =
		{
			.name = "firmware",
			.mode = S_IWUSR,
		},
	.size = DRM_MAX_FIRMWARE_SIZE,
	.write = drm_firmware_write,
};

static struct bin_attribute *drm_bin_attrs[] = {
	&drm_firmware_attr,
	NULL
};

static const struct attribute_group drm_conv_attr_group = {
	.attrs = drm_conv_attrs,
	.bin_attrs = drm_bin_attrs,
};

static const struct attribute_group drm_fc_attr_group = {
	.attrs = drm_fc_attrs,
};

static const struct attribute_group *drm_conv_attr_groups[] = {
	&drm_conv_attr_group,
	NULL
};

static const struct attribute_group *drm_fc_attr_groups[] = {
	&drm_fc_attr_group,
	NULL
};

static const struct attribute_group **drm_attr_groups[DRM_NUM_SUBDEV] = {
	drm_conv_attr_groups,
	drm_fc_attr_groups,
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
		dev = device_create_with_groups(dddrm_class, NULL,
			MKDEV(driver_major, i), drm_dev, drm_attr_groups[i],
			"dv_%s", subdev_name[i]);
		if (IS_ERR(dev)) {
			err = PTR_ERR(dev);
			dev_err(drm_dev->dev, "device_create fail %d\n", i);
			goto fail_device_init;
		}

		rIRQ = drm_dev->subdev[i].irqno;
		err = request_irq(rIRQ, handle_int, IRQF_SHARED, DRM_DEV_NAME,
				  &(drm_dev->subdev[i]));
		if (err) {
			device_destroy(dddrm_class, MKDEV(driver_major, i));
			dev_err(drm_dev->dev,
				"request_irq FAIL: IRQ=%d ERR=%d\n", rIRQ, err);
			goto fail_device_init;
		}

		// initialize:
		init_waitqueue_head(&(drm_dev->subdev[i].wait_queue));
		spin_lock_init(&(drm_dev->subdev[i].int_exclusive));
		spin_lock_init(&(drm_dev->subdev[i].wq_exclusive));

		drm_dev->subdev[i].wq = alloc_ordered_workqueue(
			"dv_wq_%s", 0, subdev_name[i]);
		if (!drm_dev->subdev[i].wq) {
			err = -ENOMEM;
			device_destroy(dddrm_class, MKDEV(driver_major, i));
			dev_err(drm_dev->dev,
				"work queue allocation fail %d\n", i);
			goto fail_device_init;
		}

		drm_dev->subdev[i].init_done = 1;
	}

	return 0;

fail_device_init:
	for (i = 0; i < DRM_NUM_SUBDEV; i++) {
		if (drm_dev->subdev[i].init_done) {
			destroy_workqueue(drm_dev->subdev[i].wq);
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
			destroy_workqueue(drm_dev->subdev[i].wq);
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
	struct device_node *dev_node, *parent_node;
	u32 addr_cells;
	unsigned int reg_base, reg_index;
#endif

	dev_dbg(&pdev->dev, "probe begin\n");

	drm_dev = devm_kzalloc(&pdev->dev, sizeof(struct drm_dev), GFP_KERNEL);
	if (!drm_dev) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "kzalloc fail\n");
		goto fail_probe_kzalloc;
	}

	platform_set_drvdata(pdev, drm_dev);
	drm_dev->dev = &pdev->dev;

	if (dma_set_mask_and_coherent(drm_dev->dev, DMA_BIT_MASK(32))) {
		pr_err(DRM_DEV_NAME ": No suitable DMA available.\n");
		err = -ENOMEM;
		goto fail_dma_set_mask;
	}

#ifdef USE_DEVTREE
	dev_node = pdev->dev.of_node;
	parent_node = of_get_parent(dev_node);
	if (parent_node) {
		of_property_read_u32(parent_node, "#address-cells", &addr_cells);
		reg_index = addr_cells - 1;
		of_node_put(parent_node);
	}
	else {
		reg_index = 0;
	}
	of_property_read_u32_index(dev_node, "reg", reg_index, &reg_base);
	// Try to read device dependent properties
	UNIFIED_BUFFER_SIZE = 0;
	of_property_read_u32(dev_node, "ubuf-size", &UNIFIED_BUFFER_SIZE);
	if ((!UNIFIED_BUFFER_SIZE) || (UNIFIED_BUFFER_SIZE > 4194303)) {  // in KBytes
		pr_warning(DRM_DEV_NAME ": Detected suspicious ubuf-size value %u KB in the device tree, using default %u KB instead",
			   UNIFIED_BUFFER_SIZE, DEF_UNIFIED_BUFFER_SIZE_KB);
	}
	UNIFIED_BUFFER_SIZE <<= 10;
	of_property_read_u32(dev_node, "max-conv-size", &MAX_CONV_KERNEL_SIZE);
	of_property_read_u32(dev_node, "max-fc-vector", &MAX_FC_VECTOR_SIZE);
#endif

	for (i = 0; i < DRM_NUM_SUBDEV; i++) {
#ifdef USE_DEVTREE
		drm_dev->subdev[i].irqno = of_irq_get(dev_node, i);
#else
		drm_dev->subdev[i].irqno = irq_no[i];
#endif

		drm_dev->subdev[i].bar_physical = reg_base + reg_offset[i];
		drm_dev->subdev[i].bar_size = reg_size[i];
		drm_dev->subdev[i].bar_logical =
			ioremap_nocache(drm_dev->subdev[i].bar_physical,
					drm_dev->subdev[i].bar_size);
		if (!drm_dev->subdev[i].bar_logical) {
			err = -EBUSY;
			dev_err(&pdev->dev, "ioremap_nocache fail %d\n", i);
			goto fail_get_iomap;
		}
		drm_dev->subdev[i].irq_addr = irq_addr[i];

		drm_dev->subdev[i].init_done = 0;
		drm_dev->subdev[i].cmd_id = 0;
		drm_dev->subdev[i].hw_id = 0;
	}

	// set firmware private attribute to conv subdev
	drm_firmware_attr.private = &drm_dev->subdev[0];

	// Set conv to command list mode
	iowrite32(1, REG_IO_ADDR((&drm_dev->subdev[0]), 0x40C));

	// Set fc to command list mode
	iowrite32(1, REG_IO_ADDR((&drm_dev->subdev[1]), 0x28));

	// Set firmware to use ROM
	iowrite32(1, REG_IO_ADDR((&drm_dev->subdev[0]), 0x44));

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
	}

	dev_dbg(&pdev->dev, "remove successful\n");
	return 0;
}

#ifdef USE_DEVTREE
static const struct of_device_id drm_of_ids[] = {
	{ .compatible = "dmp,dv700" },
	{ }
};
#endif

static struct platform_driver drm_platform_driver = {
	.probe = drm_dev_probe,
	.remove = drm_dev_remove,
	.driver =
		{
			.name = DRM_DEV_NAME,
			.owner = THIS_MODULE,
#ifdef USE_DEVTREE
			.of_match_table = drm_of_ids,
#endif
		},
};

#ifndef USE_DEVTREE
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
			.coherent_dma_mask = DMA_BIT_MASK(32),
			.dma_mask = &drm_platform_device.dev.coherent_dma_mask,
			.release = drm_dev_release,
		},
};
#endif

static int __init drm_init(void)
{
	int ret;
	ret = platform_driver_register(&drm_platform_driver);
	if (ret)
		return ret;

#ifndef USE_DEVTREE
	ret = platform_device_register(&drm_platform_device);
	if (ret) {
		platform_driver_unregister(&drm_platform_driver);
		return ret;
	}
#endif

	return 0;
}

static void __exit drm_exit(void)
{
#ifndef USE_DEVTREE
	platform_device_unregister(&drm_platform_device);
#endif
	platform_driver_unregister(&drm_platform_driver);
}

module_init(drm_init);
module_exit(drm_exit);

MODULE_DESCRIPTION("DV core driver");
MODULE_AUTHOR("Digital Media Professionals Inc.");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
