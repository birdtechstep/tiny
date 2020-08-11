// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for 4.0" TFT LCD DISPLAY ST7796S 40 PIN [YT400S006] panels
 *
 * Copyright 2020 BIRD TECHSTEP
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modeset_helper.h>
#include <video/mipi_display.h>

// ST7796 specific commands used in init
#define ST7796_NOP			0x00
#define ST7796_SWRESET		0x01
#define ST7796_RDDID		0x04
#define ST7796_RDDST		0x09

#define ST7796_RDDPM		0x0A      // Read display power mode
#define ST7796_RDD_MADCTL	0x0B      // Read display MADCTL
#define ST7796_RDD_COLMOD	0x0C      // Read display pixel format
#define ST7796_RDDIM		0x0D      // Read display image mode
#define ST7796_RDDSM		0x0E      // Read display signal mode
#define ST7796_RDDSR		0x0F      // Read display self-diagnostic result (ST7796V)

#define ST7796_SLPIN		0x10
#define ST7796_SLPOUT		0x11
#define ST7796_PTLON		0x12
#define ST7796_NORON		0x13

#define ST7796_INVOFF		0x20
#define ST7796_INVON		0x21
#define ST7796_GAMSET		0x26      // Gamma set
#define ST7796_DISPOFF		0x28
#define ST7796_DISPON		0x29
#define ST7796_CASET		0x2A
#define ST7796_RASET		0x2B
#define ST7796_RAMWR		0x2C
#define ST7796_RGBSET		0x2D      // Color setting for 4096, 64K and 262K colors
#define ST7796_RAMRD		0x2E

#define ST7796_PTLAR		0x30
#define ST7796_VSCRDEF		0x33      // Vertical scrolling definition (ST7796V)
#define ST7796_TEOFF		0x34      // Tearing effect line off
#define ST7796_TEON			0x35      // Tearing effect line on
#define ST7796_MADCTL		0x36      // Memory data access control
#define ST7796_IDMOFF		0x38      // Idle mode off
#define ST7796_IDMON		0x39      // Idle mode on
#define ST7796_RAMWRC		0x3C      // Memory write continue (ST7796V)
#define ST7796_RAMRDC		0x3E      // Memory read continue (ST7796V)
#define ST7796_COLMOD		0x3A

#define ST7796_RAMCTRL		0xB0      // RAM control
#define ST7796_RGBCTRL		0xB1      // RGB control
#define ST7796_PORCTRL		0xB2      // Porch control
#define ST7796_FRCTRL1		0xB3      // Frame rate control
#define ST7796_PARCTRL		0xB5      // Partial mode control
#define ST7796_GCTRL		0xB7      // Gate control
#define ST7796_GTADJ		0xB8      // Gate on timing adjustment
#define ST7796_DGMEN		0xBA      // Digital gamma enable
#define ST7796_VCOMS		0xBB      // VCOMS setting
#define ST7796_LCMCTRL		0xC0      // LCM control
#define ST7796_IDSET		0xC1      // ID setting
#define ST7796_VDVVRHEN		0xC2      // VDV and VRH command enable
#define ST7796_VRHS			0xC3      // VRH set
#define ST7796_VDVSET		0xC4      // VDV setting
#define ST7796_VCMOFSET		0xC5      // VCOMS offset set
#define ST7796_FRCTR2		0xC6      // FR Control 2
#define ST7796_CABCCTRL		0xC7      // CABC control
#define ST7796_REGSEL1		0xC8      // Register value section 1
#define ST7796_REGSEL2		0xCA      // Register value section 2
#define ST7796_PWMFRSEL		0xCC      // PWM frequency selection
#define ST7796_PWCTRL1		0xD0      // Power control 1
#define ST7796_VAPVANEN		0xD2      // Enable VAP/VAN signal output
#define ST7796_CMD2EN		0xDF      // Command 2 enable
#define ST7796_PVGAMCTRL	0xE0      // Positive voltage gamma control
#define ST7796_NVGAMCTRL	0xE1      // Negative voltage gamma control
#define ST7796_DGMLUTR		0xE2      // Digital gamma look-up table for red
#define ST7796_DGMLUTB		0xE3      // Digital gamma look-up table for blue
#define ST7796_GATECTRL		0xE4      // Gate control
#define ST7796_SPI2EN		0xE7      // SPI2 enable
#define ST7796_PWCTRL2		0xE8      // Power control 2
#define ST7796_EQCTRL		0xE9      // Equalize time control
#define ST7796_PROMCTRL		0xEC      // Program control
#define ST7796_PROMEN		0xFA      // Program mode enable
#define ST7796_NVMSET		0xFC      // NVM setting
#define ST7796_PROMACT		0xFE      // Program action

#define ST7796_MADCTL_BGR	BIT(3)
#define ST7796_MADCTL_MV	BIT(5)
#define ST7796_MADCTL_MX	BIT(6)
#define ST7796_MADCTL_MY	BIT(7)

static void yt400s006_enable(struct drm_simple_display_pipe *pipe,
			    struct drm_crtc_state *crtc_state,
			    struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;

	mipi_dbi_command(dbi, 0x01); //Software reset
	msleep(120);

	mipi_dbi_command(dbi, 0x11);                                         
	msleep(120);

	mipi_dbi_command(dbi, 0xF0, 0xC3);
	mipi_dbi_command(dbi, 0xF0, 0x96);
	mipi_dbi_command(dbi, 0x36, 0x48);
	mipi_dbi_command(dbi, 0x3A, 0x55);
	mipi_dbi_command(dbi, 0xB4, 0x01);
	mipi_dbi_command(dbi, 0xB6, 0x80, 0x02, 0x3B);
	mipi_dbi_command(dbi, 0xE8, 0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33);
	mipi_dbi_command(dbi, 0xC1, 0x06);
	mipi_dbi_command(dbi, 0xC2, 0xA7);
	mipi_dbi_command(dbi, 0xC5, 0x18);
	msleep(120);

	mipi_dbi_command(dbi, 0xE0, 0xF0, 0x09, 0x0b, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B); 
	mipi_dbi_command(dbi, 0xE1, 0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B);
	msleep(120);

	mipi_dbi_command(dbi, 0xF0, 0x3C);
	mipi_dbi_command(dbi, 0xF0, 0x69);
	msleep(100);
	
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);
	msleep(100);

out_enable:
	/* The PiTFT (ili9340) has a hardware reset circuit that
	 * resets only on power-on and not on each reboot through
	 * a gpio like the rpi-display does.
	 * As a result, we need to always apply the rotation value
	 * regardless of the display "on/off" state.
	 */
	switch (dbidev->rotation) {
	default:
		addr_mode = ST7796_MADCTL_MX;
		break;
	case 90:
		addr_mode = ST7796_MADCTL_MV;
		break;
	case 180:
		addr_mode = ST7796_MADCTL_MY;
		break;
	case 270:
		addr_mode = ST7796_MADCTL_MX | ST7796_MADCTL_MY | ST7796_MADCTL_MV;
		break;
	}
	addr_mode |= ST7796_MADCTL_BGR;
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs yt400s006_pipe_funcs = {
	.enable = yt400s006_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = mipi_dbi_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode yt400s006_mode = {
	DRM_SIMPLE_MODE(320, 480, 55, 83),
};

DEFINE_DRM_GEM_CMA_FOPS(yt400s006_fops);

static struct drm_driver yt400s006_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &yt400s006_fops,
	.release		= mipi_dbi_release,
	DRM_GEM_CMA_VMAP_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "yt400s006",
	.desc			= "YT400S006",
	.date			= "20200808",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id yt400s006_of_match[] = {
	{ .compatible = "makerlab,yt400s006" },
	{},
};
MODULE_DEVICE_TABLE(of, yt400s006_of_match);

static const struct spi_device_id yt400s006_id[] = {
	{ "yt400s006", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, yt400s006_id);

static int yt400s006_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	dbidev = kzalloc(sizeof(*dbidev), GFP_KERNEL);
	if (!dbidev)
		return -ENOMEM;

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;
	ret = devm_drm_dev_init(dev, drm, &yt400s006_driver);
	if (ret) {
		kfree(dbidev);
		return ret;
	}

	drm_mode_config_init(drm);

	dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(dbi->reset);
	}

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dc);
	}

	dbidev->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(dbidev->regulator))
		return PTR_ERR(dbidev->regulator);

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	ret = mipi_dbi_dev_init(dbidev, &yt400s006_pipe_funcs, &yt400s006_mode, rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static int yt400s006_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void yt400s006_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static int __maybe_unused yt400s006_pm_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int __maybe_unused yt400s006_pm_resume(struct device *dev)
{
	drm_mode_config_helper_resume(dev_get_drvdata(dev));

	return 0;
}

static const struct dev_pm_ops yt400s006_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(yt400s006_pm_suspend, yt400s006_pm_resume)
};

static struct spi_driver yt400s006_spi_driver = {
	.driver = {
		.name = "yt400s006",
		.owner = THIS_MODULE,
		.of_match_table = yt400s006_of_match,
		.pm = &yt400s006_pm_ops,
	},
	.id_table = yt400s006_id,
	.probe = yt400s006_probe,
	.remove = yt400s006_remove,
	.shutdown = yt400s006_shutdown,
};
module_spi_driver(yt400s006_spi_driver);

MODULE_DESCRIPTION("MakerLAB YT400S006 DRM driver");
MODULE_AUTHOR("BIRD TECHSTEP");
MODULE_LICENSE("GPL");
