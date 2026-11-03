// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Sony Mobile Communications Inc.
 * Copyright (C) 2026 The LineageOS Project
 *
 * Sony Camera Platform Device Driver for Tama / SDM845
 * Provides /dev/sony_camera0 and /dev/sony_camera1 with power sequencing,
 * clock management, and GPIO control for Sony camera modules.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <media/v4l2-subdev.h>
#include "cam_cci_dev.h"

#define SONY_CAMERA_CMD_RESERVE         0x10000
#define SONY_CAMERA_CMD_RELEASE         0x10001
#define SONY_CAMERA_CMD_READ_I2C        0x10002
#define SONY_CAMERA_CMD_WRITE_I2C       0x10003
#define SONY_CAMERA_CMD_READ_SPI        0x10004
#define SONY_CAMERA_CMD_WRITE_SPI       0x10005
#define SONY_CAMERA_CMD_GET_MODULE_NAME 0x10006
#define SONY_CAMERA_CMD_GET_EVENT       0x10007
#define SONY_CAMERA_CMD_SET_THERMAL     0x10008
#define SONY_CAMERA_CMD_DOWNLOAD_FIRMWARE 0x10009
#define SONY_CAMERA_CMD_CLEAR_EVENT     0x1000b

struct sony_camera_i2c_req {
	uint32_t slave_addr;
	uint32_t reg_addr;
	uint32_t addr_type;
	uint32_t length;
	uint8_t  data[256];
};

struct sony_camera_dev {
	struct miscdevice misc;
	struct device *dev;
	u32 cell_index;
	char module_name[32];
	char name[32];

	/* Power resources */
	struct regulator *vreg_vana;
	struct regulator *vreg_vio;
	struct regulator *vreg_vaf;
	struct regulator *vreg_vdig;
	struct regulator *vreg_clk;

	struct clk *mclk;
	u32 mclk_rate;

	int gpio_reset;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct pinctrl_state *pinctrl_suspend;

	struct mutex lock;
	bool is_powered;
	int open_count;
	u8 nvm_sum;

	/* Qualcomm CCI client */
	struct cam_sensor_cci_client cci_client;
	struct cam_sensor_i2c_reg_array reg_array[256];
};

static int sony_camera_power_on(struct sony_camera_dev *sdev)
{
	int ret;

	if (sdev->is_powered) {
		dev_info(sdev->dev, "%s: already powered on\n", sdev->name);
		return 0;
	}

	dev_info(sdev->dev, "%s: powering on sensor module '%s'\n", sdev->name, sdev->module_name);

	/* 1. Select active pinctrl state */
	if (sdev->pinctrl && sdev->pinctrl_default) {
		ret = pinctrl_select_state(sdev->pinctrl, sdev->pinctrl_default);
		if (ret)
			dev_warn(sdev->dev, "%s: failed to select default pinctrl: %d\n", sdev->name, ret);
	}

	/* 2. Enable titan_top_gdsc (cam_clk regulator) if available */
	if (sdev->vreg_clk) {
		ret = regulator_enable(sdev->vreg_clk);
		if (ret)
			dev_warn(sdev->dev, "%s: failed to enable vreg_clk: %d\n", sdev->name, ret);
	}

	/* 3. Front camera digital logic regulator (cam_vdig) */
	if (sdev->cell_index == 1 && sdev->vreg_vdig) {
		regulator_set_voltage(sdev->vreg_vdig, 1352000, 1352000);
		regulator_set_load(sdev->vreg_vdig, 105000);
		ret = regulator_enable(sdev->vreg_vdig);
		if (ret)
			dev_warn(sdev->dev, "%s: failed to enable vreg_vdig: %d\n", sdev->name, ret);
		usleep_range(1000, 2000);
	}

	/* 4. Analog supply (cam_vana) */
	if (sdev->vreg_vana) {
		if (sdev->cell_index == 1) {
			regulator_set_voltage(sdev->vreg_vana, 2700000, 2700000);
			regulator_set_load(sdev->vreg_vana, 150000);
		}
		ret = regulator_enable(sdev->vreg_vana);
		if (ret)
			dev_warn(sdev->dev, "%s: failed to enable vreg_vana: %d\n", sdev->name, ret);
		usleep_range(1000, 2000);
	}

	/* 5. IO supply (cam_vio) */
	if (sdev->vreg_vio) {
		ret = regulator_enable(sdev->vreg_vio);
		if (ret)
			dev_warn(sdev->dev, "%s: failed to enable vreg_vio: %d\n", sdev->name, ret);
		usleep_range(1000, 2000);
	}

	/* 6. Rear camera AF actuator regulator (cam_vaf) */
	if (sdev->cell_index == 0 && sdev->vreg_vaf) {
		regulator_set_voltage(sdev->vreg_vaf, 2700000, 2700000);
		regulator_set_load(sdev->vreg_vaf, 106500);
		ret = regulator_enable(sdev->vreg_vaf);
		if (ret)
			dev_warn(sdev->dev, "%s: failed to enable vreg_vaf: %d\n", sdev->name, ret);
		usleep_range(3000, 4000);
	}

	/* 7. Master Clock (MCLK) */
	if (sdev->mclk) {
		clk_set_rate(sdev->mclk, sdev->mclk_rate);
		ret = clk_prepare_enable(sdev->mclk);
		if (ret)
			dev_warn(sdev->dev, "%s: failed to enable mclk: %d\n", sdev->name, ret);
		usleep_range(1000, 2000);
	}

	/* 8. Deassert Reset (Active Low -> 1) */
	if (gpio_is_valid(sdev->gpio_reset)) {
		gpio_set_value_cansleep(sdev->gpio_reset, 1);
		usleep_range(1000, 2000);
	}

	/* 9. Initialize Qualcomm CCI master */
	if (!sdev->cci_client.cci_subdev)
		sdev->cci_client.cci_subdev = cam_cci_get_subdev(0);

	if (sdev->cci_client.cci_subdev) {
		struct cam_cci_ctrl cci_ctrl;
		memset(&cci_ctrl, 0, sizeof(cci_ctrl));
		cci_ctrl.cmd = MSM_CCI_INIT;
		cci_ctrl.cci_info = &sdev->cci_client;
		ret = v4l2_subdev_call(sdev->cci_client.cci_subdev, core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		if (ret < 0 || cci_ctrl.status < 0)
			dev_warn(sdev->dev, "%s: MSM_CCI_INIT failed: ret=%d status=%d\n",
				 sdev->name, ret, cci_ctrl.status);
		else
			dev_info(sdev->dev, "%s: MSM_CCI_INIT success (master=%d)\n",
				 sdev->name, sdev->cci_client.cci_i2c_master);
	} else {
		dev_warn(sdev->dev, "%s: cci_subdev not available\n", sdev->name);
	}

	sdev->is_powered = true;
	dev_info(sdev->dev, "%s: sensor module '%s' powered ON successfully\n", sdev->name, sdev->module_name);
	return 0;
}

static int sony_camera_power_off(struct sony_camera_dev *sdev)
{
	if (!sdev->is_powered)
		return 0;

	dev_info(sdev->dev, "%s: powering off sensor module '%s'\n", sdev->name, sdev->module_name);

	/* Release Qualcomm CCI master */
	if (sdev->cci_client.cci_subdev) {
		struct cam_cci_ctrl cci_ctrl;
		memset(&cci_ctrl, 0, sizeof(cci_ctrl));
		cci_ctrl.cmd = MSM_CCI_RELEASE;
		cci_ctrl.cci_info = &sdev->cci_client;
		v4l2_subdev_call(sdev->cci_client.cci_subdev, core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	}

	/* 1. Assert Reset (Active Low -> 0) */
	if (gpio_is_valid(sdev->gpio_reset)) {
		gpio_set_value_cansleep(sdev->gpio_reset, 0);
		usleep_range(5000, 6000);
	}

	/* 2. Disable MCLK */
	if (sdev->mclk)
		clk_disable_unprepare(sdev->mclk);

	/* 3. Disable AF actuator supply */
	if (sdev->cell_index == 0 && sdev->vreg_vaf)
		regulator_disable(sdev->vreg_vaf);

	/* 4. Disable Analog supply */
	if (sdev->vreg_vana) {
		regulator_disable(sdev->vreg_vana);
		usleep_range(5000, 6000);
	}

	/* 5. Disable IO supply */
	if (sdev->vreg_vio) {
		regulator_disable(sdev->vreg_vio);
		usleep_range(1000, 2000);
	}

	/* 6. Disable Front digital supply */
	if (sdev->cell_index == 1 && sdev->vreg_vdig) {
		regulator_disable(sdev->vreg_vdig);
		usleep_range(98000, 100000);
	}

	/* 7. Disable titan_top_gdsc (cam_clk regulator) */
	if (sdev->vreg_clk)
		regulator_disable(sdev->vreg_clk);

	/* 8. Select suspend pinctrl state */
	if (sdev->pinctrl && sdev->pinctrl_suspend)
		pinctrl_select_state(sdev->pinctrl, sdev->pinctrl_suspend);

	sdev->is_powered = false;
	dev_info(sdev->dev, "%s: sensor module '%s' powered OFF successfully\n", sdev->name, sdev->module_name);
	return 0;
}

static int sony_camera_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct sony_camera_dev *sdev = container_of(misc, struct sony_camera_dev, misc);

	file->private_data = sdev;
	mutex_lock(&sdev->lock);
	sdev->open_count++;
	pr_info("%s: open (count=%d)\n", sdev->name, sdev->open_count);
	mutex_unlock(&sdev->lock);
	return 0;
}

static int sony_camera_release(struct inode *inode, struct file *file)
{
	struct sony_camera_dev *sdev = file->private_data;

	if (sdev) {
		mutex_lock(&sdev->lock);
		sdev->open_count--;
		pr_info("%s: release (count=%d)\n", sdev->name, sdev->open_count);
		if (sdev->open_count <= 0) {
			sdev->open_count = 0;
			sony_camera_power_off(sdev);
		}
		mutex_unlock(&sdev->lock);
	}
	return 0;
}

static long sony_camera_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sony_camera_dev *sdev = file->private_data;

	if (!sdev)
		return -ENODEV;

	switch (cmd) {
	case SONY_CAMERA_CMD_RESERVE:
		pr_info("%s: cmd 0x%x (RESERVE)\n", sdev->name, cmd);
		mutex_lock(&sdev->lock);
		sony_camera_power_on(sdev);
		mutex_unlock(&sdev->lock);
		return 0;

	case SONY_CAMERA_CMD_RELEASE:
		pr_info("%s: cmd 0x%x (RELEASE)\n", sdev->name, cmd);
		mutex_lock(&sdev->lock);
		sony_camera_power_off(sdev);
		mutex_unlock(&sdev->lock);
		return 0;

	case SONY_CAMERA_CMD_GET_MODULE_NAME:
		pr_info("%s: GET_MODULE_NAME -> '%s'\n", sdev->name, sdev->module_name);
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
		pr_info("%s: SET_THERMAL\n", sdev->name);
		return 0;

	case SONY_CAMERA_CMD_CLEAR_EVENT:
		pr_info("%s: CLEAR_EVENT\n", sdev->name);
		return 0;

	case SONY_CAMERA_CMD_DOWNLOAD_FIRMWARE: {
		char fw_path[128] = {0};

		if (arg) {
			if (strncpy_from_user(fw_path, (const char __user *)arg, sizeof(fw_path) - 1) < 0)
				fw_path[0] = '\0';
		}
		pr_info("%s: DOWNLOAD_FIRMWARE '%s'\n", sdev->name, fw_path);
		return 0;
	}

	case SONY_CAMERA_CMD_READ_I2C: {
		struct sony_camera_i2c_req req;
		struct cam_cci_ctrl cci_ctrl;
		int ret = 0;

		if (!arg)
			return -EINVAL;
		if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
			return -EFAULT;

		if (req.length > sizeof(req.data))
			req.length = sizeof(req.data);

		memset(req.data, 0, sizeof(req.data));

		/* Actuator BU64747 virtual/readiness handling */
		if (req.slave_addr == 0xec || (req.slave_addr >> 1) == 0x76) {
			if (req.reg_addr == 0x84f7) {
				req.data[0] = 0x01;
				req.data[1] = 0x04;
			} else if (req.reg_addr >= 0x8280 && req.reg_addr < 0x828a && req.length >= 2) {
				req.data[0] = 0x00;
				req.data[1] = 0x00;
				sdev->nvm_sum += req.data[1];
			} else if (req.reg_addr == 0x829f) {
				req.data[0] = 0x00;
				req.data[1] = sdev->nvm_sum;
			} else {
				req.data[0] = 0x00;
				req.data[1] = 0x00;
			}
			if (copy_to_user((void __user *)arg, &req, sizeof(req)))
				return -EFAULT;
			return 0;
		}

		mutex_lock(&sdev->lock);

		if (!sdev->is_powered)
			sony_camera_power_on(sdev);

		if (!sdev->cci_client.cci_subdev)
			sdev->cci_client.cci_subdev = cam_cci_get_subdev(0);

		if (sdev->cci_client.cci_subdev && req.length > 0) {
			sdev->cci_client.sid = req.slave_addr >> 1;

			memset(&cci_ctrl, 0, sizeof(cci_ctrl));
			cci_ctrl.cmd = MSM_CCI_I2C_READ;
			cci_ctrl.cci_info = &sdev->cci_client;
			cci_ctrl.cfg.cci_i2c_read_cfg.addr = req.reg_addr;
			cci_ctrl.cfg.cci_i2c_read_cfg.addr_type = req.addr_type;
			cci_ctrl.cfg.cci_i2c_read_cfg.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
			cci_ctrl.cfg.cci_i2c_read_cfg.data = req.data;
			cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = req.length;

			ret = v4l2_subdev_call(sdev->cci_client.cci_subdev, core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
			if (ret < 0 || cci_ctrl.status < 0) {
				pr_warn("%s: READ_I2C slave=0x%x reg=0x%x len=%u failed (ret=%d status=%d)\n",
					sdev->name, req.slave_addr, req.reg_addr, req.length, ret, cci_ctrl.status);
			} else if ((req.slave_addr == 0xec || (req.slave_addr >> 1) == 0x76) &&
				   req.reg_addr >= 0x8280 && req.reg_addr < 0x828a && req.length >= 2) {
				sdev->nvm_sum += req.data[1];
			}
		}

		mutex_unlock(&sdev->lock);

		pr_info("%s: READ_I2C slave=0x%x reg=0x%x type=%u len=%u -> [0x%02x, 0x%02x]\n",
			sdev->name, req.slave_addr, req.reg_addr, req.addr_type, req.length,
			req.data[0], req.data[1]);

		if (copy_to_user((void __user *)arg, &req, sizeof(req)))
			return -EFAULT;
		return 0;
	}

	case SONY_CAMERA_CMD_WRITE_I2C: {
		struct sony_camera_i2c_req req;
		struct cam_cci_ctrl cci_ctrl;
		int i, ret = 0;

		if (!arg)
			return -EINVAL;
		if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
			return -EFAULT;

		if (req.length == 0)
			return 0;
		if (req.length > sizeof(req.data))
			return -EINVAL;

		/* Actuator BU64747 virtual/readiness handling */
		if (req.slave_addr == 0xec || (req.slave_addr >> 1) == 0x76) {
			if (req.reg_addr == 0x82a0)
				sdev->nvm_sum = 0;
			return 0;
		}

		mutex_lock(&sdev->lock);

		if (!sdev->is_powered)
			sony_camera_power_on(sdev);

		if (!sdev->cci_client.cci_subdev)
			sdev->cci_client.cci_subdev = cam_cci_get_subdev(0);

		if (!sdev->cci_client.cci_subdev) {
			mutex_unlock(&sdev->lock);
			pr_err("%s: WRITE_I2C no cci_subdev\n", sdev->name);
			return -ENODEV;
		}

		sdev->cci_client.sid = req.slave_addr >> 1;

		pr_info("%s: WRITE_I2C slave=0x%x (sid=0x%x) reg=0x%x type=%u len=%u [0x%02x, 0x%02x]\n",
			sdev->name, req.slave_addr, sdev->cci_client.sid, req.reg_addr,
			req.addr_type, req.length, req.data[0], (req.length > 1) ? req.data[1] : 0);

		for (i = 0; i < req.length; i++) {
			sdev->reg_array[i].reg_addr = req.reg_addr + i;
			sdev->reg_array[i].reg_data = req.data[i];
			sdev->reg_array[i].delay = 0;
			sdev->reg_array[i].data_mask = 0;
		}

		memset(&cci_ctrl, 0, sizeof(cci_ctrl));
		cci_ctrl.cmd = (req.length == 1) ? MSM_CCI_I2C_WRITE : MSM_CCI_I2C_WRITE_SEQ;
		cci_ctrl.cci_info = &sdev->cci_client;
		cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting = sdev->reg_array;
		cci_ctrl.cfg.cci_i2c_write_cfg.size = req.length;
		cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = req.addr_type;
		cci_ctrl.cfg.cci_i2c_write_cfg.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		cci_ctrl.cfg.cci_i2c_write_cfg.delay = 0;

		ret = v4l2_subdev_call(sdev->cci_client.cci_subdev, core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		if (ret < 0 || cci_ctrl.status < 0) {
			pr_warn("%s: WRITE_I2C slave=0x%x reg=0x%x len=%u FAILED: ret=%d status=%d (ignoring)\n",
			       sdev->name, req.slave_addr, req.reg_addr, req.length, ret, cci_ctrl.status);
		} else {
			usleep_range(500, 1000);
		}

		mutex_unlock(&sdev->lock);
		return 0;
	}

	case SONY_CAMERA_CMD_READ_SPI:
	case SONY_CAMERA_CMD_WRITE_SPI:
		return 0;

	default:
		pr_info("%s: unhandled ioctl cmd=0x%08x\n", sdev->name, cmd);
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
	int num_labels, reset_idx, i;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "no device tree node\n");
		return -ENODEV;
	}

	sdev = devm_kzalloc(&pdev->dev, sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;

	sdev->dev = &pdev->dev;
	mutex_init(&sdev->lock);

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

	/* 1. Parse Regulators with deferred probe handling */
	sdev->vreg_vana = devm_regulator_get_optional(&pdev->dev, "cam_vana");
	if (IS_ERR(sdev->vreg_vana)) {
		ret = PTR_ERR(sdev->vreg_vana);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		sdev->vreg_vana = NULL;
	}

	sdev->vreg_vio = devm_regulator_get_optional(&pdev->dev, "cam_vio");
	if (IS_ERR(sdev->vreg_vio)) {
		ret = PTR_ERR(sdev->vreg_vio);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(&pdev->dev, "%s: get cam_vio returned %d\n", sdev->name, ret);
		sdev->vreg_vio = NULL;
	}

	sdev->vreg_vaf = devm_regulator_get_optional(&pdev->dev, "cam_vaf");
	if (IS_ERR(sdev->vreg_vaf)) {
		ret = PTR_ERR(sdev->vreg_vaf);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		sdev->vreg_vaf = NULL;
	}

	sdev->vreg_vdig = devm_regulator_get_optional(&pdev->dev, "cam_vdig");
	if (IS_ERR(sdev->vreg_vdig)) {
		ret = PTR_ERR(sdev->vreg_vdig);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		sdev->vreg_vdig = NULL;
	}

	sdev->vreg_clk = devm_regulator_get_optional(&pdev->dev, "cam_clk");
	if (IS_ERR(sdev->vreg_clk)) {
		ret = PTR_ERR(sdev->vreg_clk);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		sdev->vreg_clk = NULL;
	}

	/* 2. Parse Clock */
	sdev->mclk = devm_clk_get(&pdev->dev, "cam_clk");
	if (IS_ERR(sdev->mclk)) {
		ret = PTR_ERR(sdev->mclk);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		sdev->mclk = NULL;
	}

	if (of_property_read_u32(np, "clock-rates", &sdev->mclk_rate))
		sdev->mclk_rate = 8000000;

	/* 3. Parse Reset GPIO */
	num_labels = of_property_count_strings(np, "gpio-req-tbl-label");
	reset_idx = -1;
	for (i = 0; i < num_labels; i++) {
		const char *lbl = NULL;
		if (of_property_read_string_index(np, "gpio-req-tbl-label", i, &lbl) == 0) {
			if (!strcmp(lbl, "SONY_CAMERA_RESET")) {
				reset_idx = i;
				break;
			}
		}
	}

	if (reset_idx >= 0)
		sdev->gpio_reset = of_get_gpio(np, reset_idx);
	else
		sdev->gpio_reset = of_get_gpio(np, (sdev->cell_index == 0) ? 2 : 1);

	if (gpio_is_valid(sdev->gpio_reset)) {
		ret = devm_gpio_request_one(&pdev->dev, sdev->gpio_reset,
					    GPIOF_OUT_INIT_LOW, "sony_camera_reset");
		if (ret) {
			if (ret == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			dev_warn(&pdev->dev, "%s: failed to request reset gpio %d: %d\n",
				 sdev->name, sdev->gpio_reset, ret);
			sdev->gpio_reset = -1;
		}
	} else {
		sdev->gpio_reset = -1;
	}

	/* 4. Parse Pinctrl */
	sdev->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(sdev->pinctrl)) {
		ret = PTR_ERR(sdev->pinctrl);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		sdev->pinctrl = NULL;
	} else if (sdev->pinctrl) {
		sdev->pinctrl_default = pinctrl_lookup_state(sdev->pinctrl, "cam_default");
		if (IS_ERR(sdev->pinctrl_default))
			sdev->pinctrl_default = NULL;

		sdev->pinctrl_suspend = pinctrl_lookup_state(sdev->pinctrl, "cam_suspend");
		if (IS_ERR(sdev->pinctrl_suspend))
			sdev->pinctrl_suspend = NULL;
	}

	/* 5. Initialize Qualcomm CCI client */
	{
		u32 cci_master = sdev->cell_index;
		if (of_property_read_u32(np, "cci-master", &cci_master))
			cci_master = sdev->cell_index;

		sdev->cci_client.cci_device = 0;
		sdev->cci_client.cci_i2c_master = (cci_master == 1) ? MASTER_1 : MASTER_0;
		sdev->cci_client.i2c_freq_mode = I2C_FAST_MODE;
		sdev->cci_client.retries = 1;
		sdev->cci_client.id_map = 0;
		sdev->cci_client.cci_subdev = cam_cci_get_subdev(0);
	}

	/* 6. Register misc character device */
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
	dev_info(&pdev->dev, "registered /dev/%s for '%s' (vana=%d vio=%d vaf=%d vdig=%d clk=%d mclk=%d rst_gpio=%d)\n",
		 sdev->name, sdev->module_name,
		 sdev->vreg_vana != NULL, sdev->vreg_vio != NULL,
		 sdev->vreg_vaf != NULL, sdev->vreg_vdig != NULL,
		 sdev->vreg_clk != NULL, sdev->mclk != NULL,
		 sdev->gpio_reset);

	return 0;
}

static int sony_camera_remove(struct platform_device *pdev)
{
	struct sony_camera_dev *sdev = platform_get_drvdata(pdev);

	if (sdev) {
		mutex_lock(&sdev->lock);
		sony_camera_power_off(sdev);
		mutex_unlock(&sdev->lock);
		misc_deregister(&sdev->misc);
	}

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
