// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Sony Mobile Communications Inc.
 * Copyright (C) 2026 The LineageOS Project
 *
 * Sony Camera Platform Device Driver for Tama / SDM845
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/compat.h>

#define SONY_CAMERA_CMD_RESERVE         0x10000
#define SONY_CAMERA_CMD_RELEASE         0x10001
#define SONY_CAMERA_CMD_READ_I2C        0x10002
#define SONY_CAMERA_CMD_WRITE_I2C       0x10003
#define SONY_CAMERA_CMD_READ_SPI        0x10004
#define SONY_CAMERA_CMD_WRITE_SPI       0x10005
#define SONY_CAMERA_CMD_GET_MODULE_NAME 0x10006
#define SONY_CAMERA_CMD_GET_EVENT       0x10007
#define SONY_CAMERA_CMD_SET_THERMAL     0x1000b

struct sony_camera_dev {
	struct miscdevice misc;
	struct device *dev;
	u32 cell_index;
	char module_name[32];
	char name[32];
};

static int sony_camera_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct sony_camera_dev *sdev = container_of(misc, struct sony_camera_dev, misc);

	file->private_data = sdev;
	return 0;
}

static int sony_camera_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long sony_camera_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sony_camera_dev *sdev = file->private_data;

	if (!sdev)
		return -ENODEV;

	switch (cmd) {
	case SONY_CAMERA_CMD_RESERVE:
	case SONY_CAMERA_CMD_RELEASE:
		pr_debug("sony_camera%u: cmd 0x%x (reserve/release)\n", sdev->cell_index, cmd);
		return 0;

	case SONY_CAMERA_CMD_GET_MODULE_NAME:
		pr_info("sony_camera%u: GET_MODULE_NAME -> '%s'\n", sdev->cell_index, sdev->module_name);
		if (!arg)
			return -EINVAL;
		if (copy_to_user((void __user *)arg, sdev->module_name, strlen(sdev->module_name) + 1))
			return -EFAULT;
		return 0;

	case SONY_CAMERA_CMD_GET_EVENT:
		if (arg) {
			char zero[32] = {0};
			if (copy_to_user((void __user *)arg, zero, sizeof(zero)))
				return -EFAULT;
		}
		return 0;

	case SONY_CAMERA_CMD_SET_THERMAL:
		pr_debug("sony_camera%u: SET_THERMAL\n", sdev->cell_index);
		return 0;

	case SONY_CAMERA_CMD_READ_I2C:
	case SONY_CAMERA_CMD_WRITE_I2C:
	case SONY_CAMERA_CMD_READ_SPI:
	case SONY_CAMERA_CMD_WRITE_SPI:
		return 0;

	default:
		pr_debug("sony_camera%u: unhandled ioctl cmd=0x%08x\n", sdev->cell_index, cmd);
		return 0;
	}
}

#ifdef CONFIG_COMPAT
static long sony_camera_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return sony_camera_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations sony_camera_fops = {
	.owner = THIS_MODULE,
	.open = sony_camera_open,
	.release = sony_camera_release,
	.unlocked_ioctl = sony_camera_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = sony_camera_compat_ioctl,
#endif
};

static int sony_camera_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sony_camera_dev *sdev;
	const char *mod_name = NULL;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "no device tree node\n");
		return -ENODEV;
	}

	sdev = devm_kzalloc(&pdev->dev, sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;

	sdev->dev = &pdev->dev;

	if (of_property_read_u32(np, "cell-index", &sdev->cell_index)) {
		const struct of_device_id *id = of_match_device(pdev->dev.driver->of_match_table, &pdev->dev);
		if (id && id->data)
			sdev->cell_index = (uintptr_t)id->data;
		else
			sdev->cell_index = 0;
	}

	if (of_property_read_string(np, "default_module_name", &mod_name) < 0) {
		if (of_property_read_string(np, "module_names", &mod_name) < 0) {
			mod_name = (sdev->cell_index == 0) ? "SOI20BS2" : "CHI05BN1";
		}
	}

	strlcpy(sdev->module_name, mod_name, sizeof(sdev->module_name));
	snprintf(sdev->name, sizeof(sdev->name), "sony_camera%u", sdev->cell_index);

	sdev->misc.minor = MISC_DYNAMIC_MINOR;
	sdev->misc.name = sdev->name;
	sdev->misc.fops = &sony_camera_fops;
	sdev->misc.mode = 0660;

	ret = misc_register(&sdev->misc);
	if (ret) {
		dev_err(&pdev->dev, "failed to register misc device %s (ret=%d)\n", sdev->name, ret);
		return ret;
	}

	platform_set_drvdata(pdev, sdev);
	dev_info(&pdev->dev, "registered /dev/%s for module '%s'\n", sdev->name, sdev->module_name);

	return 0;
}

static int sony_camera_remove(struct platform_device *pdev)
{
	struct sony_camera_dev *sdev = platform_get_drvdata(pdev);

	if (sdev)
		misc_deregister(&sdev->misc);

	return 0;
}

static const struct of_device_id sony_camera_of_match[] = {
	{ .compatible = "sony_camera_0", .data = (void *)0 },
	{ .compatible = "sony_camera_1", .data = (void *)1 },
	{ .compatible = "sony_camera_2", .data = (void *)2 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sony_camera_of_match);

static struct platform_driver sony_camera_driver = {
	.probe = sony_camera_probe,
	.remove = sony_camera_remove,
	.driver = {
		.name = "sony_camera",
		.owner = THIS_MODULE,
		.of_match_table = sony_camera_of_match,
	},
};

module_platform_driver(sony_camera_driver);

MODULE_AUTHOR("LineageOS Project");
MODULE_DESCRIPTION("Sony Camera Platform Driver");
MODULE_LICENSE("GPL v2");
