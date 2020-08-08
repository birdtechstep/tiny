// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for 1.54" TFT LCD DISPLAY ST7789VW 12 PIN [LH154T-PG04] panels
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

// ST7789 specific commands used in init
#define ST7789_NOP			0x00
#define ST7789_SWRESET		0x01
#define ST7789_RDDID		0x04
#define ST7789_RDDST		0x09

#define ST7789_RDDPM		0x0A      // Read display power mode
#define ST7789_RDD_MADCTL	0x0B      // Read display MADCTL
#define ST7789_RDD_COLMOD	0x0C      // Read display pixel format
#define ST7789_RDDIM		0x0D      // Read display image mode
#define ST7789_RDDSM		0x0E      // Read display signal mode
#define ST7789_RDDSR		0x0F      // Read display self-diagnostic result (ST7789V)

#define ST7789_SLPIN		0x10
#define ST7789_SLPOUT		0x11
#define ST7789_PTLON		0x12
#define ST7789_NORON		0x13

#define ST7789_INVOFF		0x20
#define ST7789_INVON		0x21
#define ST7789_GAMSET		0x26      // Gamma set
#define ST7789_DISPOFF		0x28
#define ST7789_DISPON		0x29
#define ST7789_CASET		0x2A
#define ST7789_RASET		0x2B
#define ST7789_RAMWR		0x2C
#define ST7789_RGBSET		0x2D      // Color setting for 4096, 64K and 262K colors
#define ST7789_RAMRD		0x2E

#define ST7789_PTLAR		0x30
#define ST7789_VSCRDEF		0x33      // Vertical scrolling definition (ST7789V)
#define ST7789_TEOFF		0x34      // Tearing effect line off
#define ST7789_TEON			0x35      // Tearing effect line on
#define ST7789_MADCTL		0x36      // Memory data access control
#define ST7789_IDMOFF		0x38      // Idle mode off
#define ST7789_IDMON		0x39      // Idle mode on
#define ST7789_RAMWRC		0x3C      // Memory write continue (ST7789V)
#define ST7789_RAMRDC		0x3E      // Memory read continue (ST7789V)
#define ST7789_COLMOD		0x3A

#define ST7789_RAMCTRL		0xB0      // RAM control
#define ST7789_RGBCTRL		0xB1      // RGB control
#define ST7789_PORCTRL		0xB2      // Porch control
#define ST7789_FRCTRL1		0xB3      // Frame rate control
#define ST7789_PARCTRL		0xB5      // Partial mode control
#define ST7789_GCTRL		0xB7      // Gate control
#define ST7789_GTADJ		0xB8      // Gate on timing adjustment
#define ST7789_DGMEN		0xBA      // Digital gamma enable
#define ST7789_VCOMS		0xBB      // VCOMS setting
#define ST7789_LCMCTRL		0xC0      // LCM control
#define ST7789_IDSET		0xC1      // ID setting
#define ST7789_VDVVRHEN		0xC2      // VDV and VRH command enable
#define ST7789_VRHS			0xC3      // VRH set
#define ST7789_VDVSET		0xC4      // VDV setting
#define ST7789_VCMOFSET		0xC5      // VCOMS offset set
#define ST7789_FRCTR2		0xC6      // FR Control 2
#define ST7789_CABCCTRL		0xC7      // CABC control
#define ST7789_REGSEL1		0xC8      // Register value section 1
#define ST7789_REGSEL2		0xCA      // Register value section 2
#define ST7789_PWMFRSEL		0xCC      // PWM frequency selection
#define ST7789_PWCTRL1		0xD0      // Power control 1
#define ST7789_VAPVANEN		0xD2      // Enable VAP/VAN signal output
#define ST7789_CMD2EN		0xDF      // Command 2 enable
#define ST7789_PVGAMCTRL	0xE0      // Positive voltage gamma control
#define ST7789_NVGAMCTRL	0xE1      // Negative voltage gamma control
#define ST7789_DGMLUTR		0xE2      // Digital gamma look-up table for red
#define ST7789_DGMLUTB		0xE3      // Digital gamma look-up table for blue
#define ST7789_GATECTRL		0xE4      // Gate control
#define ST7789_SPI2EN		0xE7      // SPI2 enable
#define ST7789_PWCTRL2		0xE8      // Power control 2
#define ST7789_EQCTRL		0xE9      // Equalize time control
#define ST7789_PROMCTRL		0xEC      // Program control
#define ST7789_PROMEN		0xFA      // Program mode enable
#define ST7789_NVMSET		0xFC      // NVM setting
#define ST7789_PROMACT		0xFE      // Program action

#define ST7789_MADCTL_BGR	BIT(3)
#define ST7789_MADCTL_MV	BIT(5)
#define ST7789_MADCTL_MX	BIT(6)
#define ST7789_MADCTL_MY	BIT(7)

static void lh154t_enable(struct drm_simple_display_pipe *pipe,
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

	mipi_dbi_command(dbi, ST7789_SLPOUT);   // Sleep out
	msleep(120);

	mipi_dbi_command(dbi, ST7789_NORON);    // Normal display mode on

  //------------------------------display and color format setting--------------------------------//
	mipi_dbi_command(dbi, ST7789_MADCTL, 0x00);

  // JLX240 display datasheet
	mipi_dbi_command(dbi, 0xB6, 0x0A, 0x82);

	mipi_dbi_command(dbi, ST7789_COLMOD, 0x55);
	msleep(10);

  //--------------------------------ST7789V Frame rate setting----------------------------------//
	mipi_dbi_command(dbi, ST7789_PORCTRL, 0x0c, 0x0c, 0x00, 0x33, 0x33);

	mipi_dbi_command(dbi, ST7789_GCTRL, 0x35);      // Voltages: VGH / VGL

  //---------------------------------ST7789V Power setting--------------------------------------//
	mipi_dbi_command(dbi, ST7789_VCOMS, 0x28);		// JLX240 display datasheet
	mipi_dbi_command(dbi, ST7789_LCMCTRL, 0x0C);
	mipi_dbi_command(dbi, ST7789_VDVVRHEN, 0x01, 0xFF);
	mipi_dbi_command(dbi, ST7789_VRHS, 0x10);       // voltage VRHS
	mipi_dbi_command(dbi, ST7789_VDVSET, 0x20);
	mipi_dbi_command(dbi, ST7789_FRCTR2, 0x0f);
	mipi_dbi_command(dbi, ST7789_PWCTRL1, 0xa4, 0xa1);

  //--------------------------------ST7789V gamma setting---------------------------------------//
	mipi_dbi_command(dbi, ST7789_PVGAMCTRL, 0xd0, 0x00, 0x02, 0x07, 0x0a, 0x28, 0x32, 0x44, 0x42, 0x06, 0x0e, 0x12, 0x14, 0x17);
	mipi_dbi_command(dbi, ST7789_NVGAMCTRL, 0xd0, 0x00, 0x02, 0x07, 0x0a, 0x28, 0x31, 0x54, 0x47, 0x0e, 0x1c, 0x17, 0x1b, 0x1e);

	mipi_dbi_command(dbi, ST7789_INVON);

	mipi_dbi_command(dbi, ST7789_CASET, 0x00, 0x00, 0x00, 0xE5);    // Column address set
	mipi_dbi_command(dbi, ST7789_RASET, 0x00, 0x00, 0x00, 0xE5);    // Row address set
	msleep(120);

	mipi_dbi_command(dbi, ST7789_DISPON);    //Display on
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
		addr_mode = 0x00;
		break;
	case 90:
		addr_mode = ST7789_MADCTL_MX | ST7789_MADCTL_MV;
		break;
	case 180:
		addr_mode = ST7789_MADCTL_MX | ST7789_MADCTL_MY;
		break;
	case 270:
		addr_mode = ST7789_MADCTL_MV | ST7789_MADCTL_MY;
		break;
	}
	addr_mode |= ST7789_MADCTL_BGR;
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs lh154t_pipe_funcs = {
	.enable = lh154t_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = mipi_dbi_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode lh154t_mode = {
	DRM_SIMPLE_MODE(240, 240, 27, 27),
};

DEFINE_DRM_GEM_CMA_FOPS(lh154t_fops);

static struct drm_driver lh154t_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &lh154t_fops,
	.release		= mipi_dbi_release,
	DRM_GEM_CMA_VMAP_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "lh154t",
	.desc			= "LH154T-PG04",
	.date			= "20200808",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id lh154t_of_match[] = {
	{ .compatible = "makerlab,lh154t" },
	{},
};
MODULE_DEVICE_TABLE(of, lh154t_of_match);

static const struct spi_device_id lh154t_id[] = {
	{ "lh154t", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, lh154t_id);

static int lh154t_probe(struct spi_device *spi)
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
	ret = devm_drm_dev_init(dev, drm, &lh154t_driver);
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

	ret = mipi_dbi_dev_init(dbidev, &lh154t_pipe_funcs, &lh154t_mode, rotation);
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

static int lh154t_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void lh154t_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static int __maybe_unused lh154t_pm_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int __maybe_unused lh154t_pm_resume(struct device *dev)
{
	drm_mode_config_helper_resume(dev_get_drvdata(dev));

	return 0;
}

static const struct dev_pm_ops lh154t_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lh154t_pm_suspend, lh154t_pm_resume)
};

static struct spi_driver lh154t_spi_driver = {
	.driver = {
		.name = "lh154t",
		.owner = THIS_MODULE,
		.of_match_table = lh154t_of_match,
		.pm = &lh154t_pm_ops,
	},
	.id_table = lh154t_id,
	.probe = lh154t_probe,
	.remove = lh154t_remove,
	.shutdown = lh154t_shutdown,
};
module_spi_driver(lh154t_spi_driver);

MODULE_DESCRIPTION("MakerLAB LH154T-PG04 DRM driver");
MODULE_AUTHOR("BIRD TECHSTEP");
MODULE_LICENSE("GPL");
