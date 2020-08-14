// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for SX035HV006 panels
 *
 * Copyright 2016 Noralf Trønnes
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

#define ILI9488_FRMCTR1		0xb1
#define ILI9488_DISCTRL		0xb6
#define ILI9488_ETMOD		0xb7

#define ILI9488_PWCTRL1		0xc0
#define ILI9488_PWCTRL2		0xc1
#define ILI9488_VMCTRL1		0xc5
#define ILI9488_VMCTRL2		0xc7
#define ILI9488_PWCTRLA		0xcb
#define ILI9488_PWCTRLB		0xcf

#define ILI9488_PGAMCTRL	0xe0
#define ILI9488_NGAMCTRL	0xe1
#define ILI9488_DTCTRLA		0xe8
#define ILI9488_DTCTRLB		0xea
#define ILI9488_PWRSEQ		0xed

#define ILI9488_EN3GAM		0xf2
#define ILI9488_PUMPCTRL	0xf7

#define ILI9488_MADCTL_BGR	BIT(3)
#define ILI9488_MADCTL_MV	BIT(5)
#define ILI9488_MADCTL_MX	BIT(6)
#define ILI9488_MADCTL_MY	BIT(7)

static void ili9488_enable(struct drm_simple_display_pipe *pipe,
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

	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_OFF);

	mipi_dbi_command(dbi, ILI9488_PWCTRLB, 0x00, 0x83, 0x30);
	mipi_dbi_command(dbi, ILI9488_PWRSEQ, 0x64, 0x03, 0x12, 0x81);
	mipi_dbi_command(dbi, ILI9488_DTCTRLA, 0x85, 0x01, 0x79);
	mipi_dbi_command(dbi, ILI9488_PWCTRLA, 0x39, 0x2c, 0x00, 0x34, 0x02);
	mipi_dbi_command(dbi, ILI9488_PUMPCTRL, 0x20);
	mipi_dbi_command(dbi, ILI9488_DTCTRLB, 0x00, 0x00);

	/* Power Control */
	mipi_dbi_command(dbi, ILI9488_PWCTRL1, 0x26);
	mipi_dbi_command(dbi, ILI9488_PWCTRL2, 0x11);
	/* VCOM */
	mipi_dbi_command(dbi, ILI9488_VMCTRL1, 0x35, 0x3e);
	mipi_dbi_command(dbi, ILI9488_VMCTRL2, 0xbe);

	/* Memory Access Control */
	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT);

	/* Frame Rate */
	mipi_dbi_command(dbi, ILI9488_FRMCTR1, 0x00, 0x1b);

	/* Gamma */
	mipi_dbi_command(dbi, ILI9488_EN3GAM, 0x08);
	mipi_dbi_command(dbi, MIPI_DCS_SET_GAMMA_CURVE, 0x01);
	mipi_dbi_command(dbi, ILI9488_PGAMCTRL,
		       0x1f, 0x1a, 0x18, 0x0a, 0x0f, 0x06, 0x45, 0x87,
		       0x32, 0x0a, 0x07, 0x02, 0x07, 0x05, 0x00);
	mipi_dbi_command(dbi, ILI9488_NGAMCTRL,
		       0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3a, 0x78,
		       0x4d, 0x05, 0x18, 0x0d, 0x38, 0x3a, 0x1f);

	/* DDRAM */
	mipi_dbi_command(dbi, ILI9488_ETMOD, 0x07);

	/* Display */
	mipi_dbi_command(dbi, ILI9488_DISCTRL, 0x0a, 0x82, 0x27, 0x00);
	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
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
		addr_mode = ILI9488_MADCTL_MV | ILI9488_MADCTL_MY |
			    ILI9488_MADCTL_MX;
		break;
	case 90:
		addr_mode = ILI9488_MADCTL_MY;
		break;
	case 180:
		addr_mode = ILI9488_MADCTL_MV;
		break;
	case 270:
		addr_mode = ILI9488_MADCTL_MX;
		break;
	}
	addr_mode |= ILI9488_MADCTL_BGR;
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	spi_dbi_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs ili9488_pipe_funcs = {
	.enable = ili9488_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = spi_dbi_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

/**
 * drm_fb_xrgb8888_to_rgb666 - Convert XRGB8888 to RGB666 clip buffer
 * @dst: RGB666 destination buffer
 * @vaddr: XRGB8888 source buffer
 * @fb: DRM framebuffer
 * @clip: Clip rectangle area to copy
 * @swab: Swap bytes
 *
 * Drivers can use this function for RGB666 devices that don't natively
 * support XRGB8888.
 *
 * This function does not apply clipping on dst, i.e. the destination
 * is a small buffer containing the clip rect only.
 */
void drm_fb_xrgb8888_to_rgb666(u8 *dst, void *vaddr,
			       struct drm_framebuffer *fb,
			       struct drm_rect *clip,
			       bool dstclip)
{
	size_t len = (clip->x2 - clip->x1) * sizeof(u32);
	unsigned int x, y;
	u32 *src, *buf;
	u8 red, green, blue;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return;

	if (dstclip)
		dst += (clip->y1 * fb->width + clip->x1) * 3;
	for (y = clip->y1; y < clip->y2; y++) {
		src = vaddr + (y * fb->pitches[0]);
		src += clip->x1;
		memcpy(buf, src, len);
		src = buf;
		for (x = clip->x1; x < clip->x2; x++) {
			red   = (*src & 0x00FF0000) >> 16;
			green = (*src & 0x0000FF00) >>  8;
			blue  = (*src & 0x000000FF) >>  0;
			src++;
			*dst++ = blue  >> 2;
			*dst++ = green >> 2;
			*dst++ = red   >> 2;
		}
		if (dstclip)
			dst += (fb->width - (clip->x2 - clip->x1)) * 3;
	}

	kfree(buf);
}

/**
 * mipi_dbi_buf18_copy - Copy a framebuffer, transforming it if necessary
 * @dst: The destination buffer
 * @fb: The source framebuffer
 * @clip: Clipping rectangle of the area to be copied
 * @swap: When true, swap MSB/LSB of 16-bit values
 *
 * Returns:
 * Zero on success, negative error code on failure.
 */
int mipi_dbi_buf18_copy(void *dst, struct drm_framebuffer *fb,
		      struct drm_rect *clip, bool swap)
{
	struct drm_gem_object *gem = drm_gem_fb_get_obj(fb, 0);
	struct drm_gem_cma_object *cma_obj = to_drm_gem_cma_obj(gem);
	struct dma_buf_attachment *import_attach = gem->import_attach;
	struct drm_format_name_buf format_name;
	void *src = cma_obj->vaddr;
	int ret = 0;

	if (import_attach) {
		ret = dma_buf_begin_cpu_access(import_attach->dmabuf,
					       DMA_FROM_DEVICE);
		if (ret)
			return ret;
	}

	switch (fb->format->format) {
	case DRM_FORMAT_RGB565:
		if (swap)
			drm_fb_swab16(dst, src, fb, clip);
		else
			drm_fb_memcpy(dst, src, fb, clip);
		break;
	case DRM_FORMAT_XRGB8888:
		drm_fb_xrgb8888_to_rgb666(dst, src, fb, clip, swap);
		break;
	default:
		dev_err_once(fb->dev->dev, "Format is not supported: %s\n",
			     drm_get_format_name(fb->format->format,
						 &format_name));
		return -EINVAL;
	}

	if (import_attach)
		ret = dma_buf_end_cpu_access(import_attach->dmabuf,
					     DMA_FROM_DEVICE);
	return ret;
}

static void spi_dbi_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
	struct drm_gem_object *gem = drm_gem_fb_get_obj(fb, 0);
	struct drm_gem_cma_object *cma_obj = to_drm_gem_cma_obj(gem);
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	unsigned int height = rect->y2 - rect->y1;
	unsigned int width = rect->x2 - rect->x1;
	struct mipi_dbi *dbi = &dbidev->dbi;
	bool swap = dbi->swap_bytes;
	int idx, ret = 0;
	bool full;
	void *tr;

	if (!dbidev->enabled)
		return;

	if (!drm_dev_enter(fb->dev, &idx))
		return;

	full = width == fb->width && height == fb->height;

	DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id, DRM_RECT_ARG(rect));

	if (!dbi->dc || !full || swap ||
	    fb->format->format == DRM_FORMAT_XRGB8888) {
		tr = dbidev->tx_buf;
		ret = mipi_dbi_buf18_copy(dbidev->tx_buf, fb, rect, swap);
		if (ret)
			goto err_msg;
	} else {
		tr = cma_obj->vaddr;
	}

	mipi_dbi_command(dbi, MIPI_DCS_SET_COLUMN_ADDRESS,
			 (rect->x1 >> 8) & 0xff, rect->x1 & 0xff,
			 ((rect->x2 - 1) >> 8) & 0xff, (rect->x2 - 1) & 0xff);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS,
			 (rect->y1 >> 8) & 0xff, rect->y1 & 0xff,
			 ((rect->y2 - 1) >> 8) & 0xff, (rect->y2 - 1) & 0xff);

	ret = mipi_dbi_command_buf(dbi, MIPI_DCS_WRITE_MEMORY_START, tr,
				   width * height * 2);
err_msg:
	if (ret)
		dev_err_once(fb->dev->dev, "Failed to update display %d\n", ret);

	drm_dev_exit(idx);
}

/**
 * mipi_dbi_pipe_update - Display pipe update helper
 * @pipe: Simple display pipe
 * @old_state: Old plane state
 *
 * This function handles framebuffer flushing and vblank events. Drivers can use
 * this as their &drm_simple_display_pipe_funcs->update callback.
 */
void spi_dbi_pipe_update(struct drm_simple_display_pipe *pipe,
			  struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_rect rect;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect))
		spi_dbi_fb_dirty(state->fb, &rect);

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);
		crtc->state->event = NULL;
	}
}

/**
 * mipi_dbi_enable_flush - MIPI DBI enable helper
 * @dbidev: MIPI DBI device structure
 * @crtc_state: CRTC state
 * @plane_state: Plane state
 *
 * This function sets &mipi_dbi->enabled, flushes the whole framebuffer and
 * enables the backlight. Drivers can use this in their
 * &drm_simple_display_pipe_funcs->enable callback.
 *
 * Note: Drivers which don't use mipi_dbi_pipe_update() because they have custom
 * framebuffer flushing, can't use this function since they both use the same
 * flushing code.
 */
void spi_dbi_enable_flush(struct mipi_dbi_dev *dbidev,
			   struct drm_crtc_state *crtc_state,
			   struct drm_plane_state *plane_state)
{
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_rect rect = {
		.x1 = 0,
		.x2 = fb->width,
		.y1 = 0,
		.y2 = fb->height,
	};
	int idx;

	if (!drm_dev_enter(&dbidev->drm, &idx))
		return;

	dbidev->enabled = true;
	spi_dbi_fb_dirty(fb, &rect);
	backlight_enable(dbidev->backlight);

	drm_dev_exit(idx);
}

static const struct drm_display_mode ili9488_mode = {
	DRM_SIMPLE_MODE(480, 320, 73, 49),
};

DEFINE_DRM_GEM_CMA_FOPS(ili9488_fops);

static struct drm_driver ili9488_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ili9488_fops,
	.release		= mipi_dbi_release,
	DRM_GEM_CMA_VMAP_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "ili9488",
	.desc			= "Ilitek ILI9488",
	.date			= "20200814",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id ili9488_of_match[] = {
	{ .compatible = "ilitek,ili9488" },
	{},
};
MODULE_DEVICE_TABLE(of, ili9488_of_match);

static const struct spi_device_id ili9488_id[] = {
	{ "ili9488", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ili9488_id);

static int ili9488_probe(struct spi_device *spi)
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
	ret = devm_drm_dev_init(dev, drm, &ili9488_driver);
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

	ret = mipi_dbi_dev_init(dbidev, &ili9488_pipe_funcs, &ili9488_mode, rotation);
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

static int ili9488_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void ili9488_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static int __maybe_unused ili9488_pm_suspend(struct device *dev)
{
	return drm_mode_config_helper_suspend(dev_get_drvdata(dev));
}

static int __maybe_unused ili9488_pm_resume(struct device *dev)
{
	drm_mode_config_helper_resume(dev_get_drvdata(dev));

	return 0;
}

static const struct dev_pm_ops ili9488_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ili9488_pm_suspend, ili9488_pm_resume)
};

static struct spi_driver ili9488_spi_driver = {
	.driver = {
		.name = "ili9488",
		.owner = THIS_MODULE,
		.of_match_table = ili9488_of_match,
		.pm = &ili9488_pm_ops,
	},
	.id_table = ili9488_id,
	.probe = ili9488_probe,
	.remove = ili9488_remove,
	.shutdown = ili9488_shutdown,
};
module_spi_driver(ili9488_spi_driver);

MODULE_DESCRIPTION("Multi-Inno MI0283QT DRM driver");
MODULE_AUTHOR("Noralf Trønnes");
MODULE_LICENSE("GPL");
