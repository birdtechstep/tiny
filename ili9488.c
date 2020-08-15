// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for Ilitek ili9488 panels
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

#include <linux/dma-buf.h>
#include <video/mipi_display.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_vblank.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>

#define ILI9488_CMD_NORMAL_DISP_MODE_ON		0x13

#define ILI9488_CMD_MEMORY_ACCESS_CONTROL	0x36
#define ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET	0x3A

#define ILI9488_CMD_INTERFACE_MODE_CONTROL	0xB0
#define ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL	0xB1
#define ILI9488_CMD_DISPLAY_INVERSION_CONTROL	0xB4

#define ILI9488_CMD_POWER_CONTROL_1 		0xC0
#define ILI9488_CMD_POWER_CONTROL_2		0xC1
#define ILI9488_CMD_POWER_CONTROL_NORMAL_3	0xC2
#define ILI9488_CMD_VCOM_CONTROL_1		0xC5
#define ILI9488_CMD_CABC_CONTROL_2		0xC8

#define ILI9488_CMD_POSITIVE_GAMMA_CORRECTION	0xE0
#define ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION	0xE1
#define ILI9488_CMD_SET_IMAGE_FUNCTION		0xE9

#define ILI9488_CMD_ADJUST_CONTROL_3		0xF7

#define ILI9488_MADCTL_MH	BIT(2)		/* Horizontal Refresh Order */
#define ILI9488_MADCTL_BGR	BIT(3)		/* BGR Order, if set */
#define ILI9488_MADCTL_ML	BIT(4)		/* Vertical Refresh Order */
#define ILI9488_MADCTL_MV	BIT(5)		/* Row / Column Exchange */
#define ILI9488_MADCTL_MX	BIT(6)		/* Column Address Order */
#define ILI9488_MADCTL_MY	BIT(7)		/* Row Address Order */
/*
 * ILI9488 pixel format flags
 *
 * DBI is the pixel format of CPU interface
 */
#define ILI9488_DBI_BPP16               0x05    /* 16 bits / pixel */
#define ILI9488_DBI_BPP18               0x06    /* 18 bits / pixel */
#define ILI9488_DBI_BPP24               0x07    /* 24 bits / pixel */

/*
 * DPI is the pixel format select of RGB interface
 */
#define ILI9488_DPI_BPP16               0x50    /* 16 bits / pixel */
#define ILI9488_DPI_BPP18               0x60    /* 18 bits / pixel */
#define ILI9488_DPI_BPP24               0x70    /* 24 bits / pixel */

static const uint32_t ili9488_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888,
};

static void ili9488_rgb565_to_rgb666_line(u8 *dst, u16 *sbuf,
					  unsigned int pixels)
{
	unsigned int x;

	for (x = 0; x < pixels; x++) {
		*dst++ = ((*sbuf & 0xF800) >> 8);
		*dst++ = ((*sbuf & 0x07E0) >> 3);
		*dst++ = ((*sbuf & 0x001F) << 3);
		sbuf++;
	}
}

static void ili9488_rgb565_to_rgb666(u8 *dst, void *vaddr,
				     struct drm_framebuffer *fb,
				     struct drm_rect *rect)
{
	size_t linepixels = rect->x2 - rect->x1;
	size_t src_len = linepixels * sizeof(u16);
	size_t dst_len = linepixels * 3;
	unsigned int y, lines = rect->y2 - rect->y1;
	u16 *sbuf;

	/*
	 * The cma memory is write-combined so reads are uncached.
	 * Speed up by fetching one line at a time.
	 */
	sbuf = kmalloc(src_len, GFP_KERNEL);
	if (!sbuf)
		return;

	vaddr += rect->y1 * fb->pitches[0] + rect->x1 * sizeof(u16);
	for (y = 0; y < lines; y++) {
		memcpy(sbuf, vaddr, src_len);
		ili9488_rgb565_to_rgb666_line(dst, sbuf, linepixels);
		vaddr += fb->pitches[0];
		dst += dst_len;
	}
	kfree(sbuf);
}

static void ili9488_xrgb8888_to_rgb666(u8 *dst, void *vaddr,
					struct drm_framebuffer *fb,
					struct drm_rect *clip)
{
	size_t len = (clip->x2 - clip->x1) * sizeof(u32);
	unsigned int x, y;
	u32 *src, *buf;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return;

	for (y = clip->y1; y < clip->y2; y++) {
		src = vaddr + (y * fb->pitches[0]);
		src += clip->x1;
		memcpy(buf, src, len);
		src = buf;
		for (x = clip->x1; x < clip->x2; x++) {
			*dst++ = ((*src & 0x00FC0000) >> 16);
			*dst++ = ((*src & 0x0000FC00) >> 8);
			*dst++ = ((*src & 0x000000FC));
			src++;
		}
	}

	kfree(buf);
}


static int ili9488_buf_copy(void *dst, struct drm_framebuffer *fb,
			    struct drm_rect *rect)
{
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	struct dma_buf_attachment *import_attach = cma_obj->base.import_attach;
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
		ili9488_rgb565_to_rgb666(dst, src, fb, rect);
		break;
 	case DRM_FORMAT_XRGB8888:
                ili9488_xrgb8888_to_rgb666(dst, src, fb, rect);
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


static void ili9488_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	int idx, ret = 0;
	void *tr;
	bool full;
	unsigned int height = rect->y2 - rect->y1;
	unsigned int width = rect->x2 - rect->x1;

	if (!dbidev->enabled)
		return;

	if (!drm_dev_enter(fb->dev, &idx))
		return;

	full = width == fb->width && height == fb->height;

	DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id,
		      DRM_RECT_ARG(rect));

	/* Always invoke copy buffer routine as the display supports
	 * only RGB666 format which is not implemented in DRM
	 */
	if (!dbi->dc || !full ||
	    fb->format->format == DRM_FORMAT_XRGB8888) {
		tr = dbidev->tx_buf;
		ret = ili9488_buf_copy(dbidev->tx_buf, fb, rect);
		if (ret)
			goto err_msg;
	} else {
		tr = cma_obj->vaddr;
	}

	mipi_dbi_command(dbi,MIPI_DCS_SET_COLUMN_ADDRESS,
			 (rect->x1 >> 8) & 0xFF, rect->x1 & 0xFF,
			 (rect->x2 >> 8) & 0xFF, (rect->x2 - 1) & 0xFF);

	mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS,
			 (rect->y1 >> 8) & 0xFF, rect->y1 & 0xFF,
			 (rect->y2 >> 8) & 0xFF, (rect->y2 - 1) & 0xFF);

	ret = mipi_dbi_command_buf(dbi, MIPI_DCS_WRITE_MEMORY_START, tr,
				   width * height * 3);

 err_msg:
	if (ret)
		dev_err_once(fb->dev->dev, "Failed to update display %d\n", ret);

	drm_dev_exit(idx);
}

static void ili9488_enable_flush(struct mipi_dbi_dev *dbidev,
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
	ili9488_fb_dirty(fb, &rect);
	backlight_enable(dbidev->backlight);

	drm_dev_exit(idx);
}

static void ili9488_pipe_update(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_rect rect;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect))
		ili9488_fb_dirty(state->fb, &rect);

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);
		crtc->state->event = NULL;
	}
}

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

	/* Positive Gamma Control */
	mipi_dbi_command(dbi, ILI9488_CMD_POSITIVE_GAMMA_CORRECTION,
			 0x00, 0x03, 0x09, 0x08, 0x16,
			 0x0a, 0x3f, 0x78, 0x4c, 0x09,
			 0x0a, 0x08, 0x16, 0x1a, 0x0f);

	/* Negative Gamma Control */
	mipi_dbi_command(dbi, ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION,
			 0x00, 0x16, 0x19, 0x03, 0x0f,
			 0x05, 0x32, 0x45, 0x46, 0x04,
			 0x0e, 0x0d, 0x35, 0x37, 0x0f);


	/* Power Control 1 */
	mipi_dbi_command(dbi, ILI9488_CMD_POWER_CONTROL_1, 0x17, 0x15);

	/* Power Control 2 */
	mipi_dbi_command(dbi, ILI9488_CMD_POWER_CONTROL_2, 0x41);

	/* Power Control 3 (Normal mode) */
	mipi_dbi_command(dbi, ILI9488_CMD_POWER_CONTROL_NORMAL_3, 0x44);


	/* VCOM Control 1 */
	mipi_dbi_command(dbi, ILI9488_CMD_VCOM_CONTROL_1, 0x00, 0x12, 0x80);


	/* Memory Access Control */
	//mipi_dbi_command(dbi, ILI9488_CMD_MEMORY_ACCESS_CONTROL, addr_mode);


	/* Pixel Format */
	mipi_dbi_command(dbi, ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET,
			 ILI9488_DBI_BPP18 | ILI9488_DPI_BPP18);

	mipi_dbi_command(dbi, ILI9488_CMD_INTERFACE_MODE_CONTROL, 0x80);
	/* Frame Rate Control */
	/*	Frame rate = 60.76Hz.*/
	mipi_dbi_command(dbi, ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL, 0xa0);
	/* Display Inversion Control */
	/*	2 dot inversion */
	mipi_dbi_command(dbi, ILI9488_CMD_DISPLAY_INVERSION_CONTROL, 0x02);
	/* Set Image Function */
	mipi_dbi_command(dbi, ILI9488_CMD_SET_IMAGE_FUNCTION, 0x00);
	/* Adjust Control 3 */
	mipi_dbi_command(dbi, ILI9488_CMD_ADJUST_CONTROL_3,
			 0xa9, 0x51, 0x2c, 0x82);
	/* CABC control 2 */
	mipi_dbi_command(dbi, ILI9488_CMD_CABC_CONTROL_2, 0xb0);

	/* Display */
	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(100);

	mipi_dbi_command(dbi, ILI9488_CMD_NORMAL_DISP_MODE_ON);

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
		//addr_mode = ILI9488_MADCTL_MV | ILI9488_MADCTL_MY | ILI9488_MADCTL_MX;
		addr_mode = ILI9488_MADCTL_MX;
		break;
	case 90:
		//addr_mode = ILI9488_MADCTL_MY;
		addr_mode = ILI9488_MADCTL_MV;
		break;
	case 180:
		//addr_mode = ILI9488_MADCTL_MV;
		addr_mode = ( ILI9488_MADCTL_MY | ILI9488_MADCTL_ML );
		break;
	case 270:
		//addr_mode = ILI9488_MADCTL_MX;
		addr_mode = ( ILI9488_MADCTL_MX | ILI9488_MADCTL_MY | ILI9488_MADCTL_MV | ILI9488_MADCTL_ML );
		break;
	}
	addr_mode |= ILI9488_MADCTL_BGR;
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	ili9488_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs ili9488_pipe_funcs = {
	.enable = ili9488_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = ili9488_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode ili9488_mode = {
	DRM_SIMPLE_MODE(320, 480, 49, 73),
};

DEFINE_DRM_GEM_CMA_FOPS(ili9488_fops);

static struct drm_driver ili9488_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ili9488_fops,
	.release		= mipi_dbi_release,
	DRM_GEM_CMA_VMAP_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "ili9488",
	.desc			= "Ilitek ili9488",
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
	size_t bufsize;
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

	bufsize = ili9488_mode.vdisplay * ili9488_mode.hdisplay * 3;

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

	//dbidev->drm.mode_config.preferred_depth = 16;

	//ret = mipi_dbi_dev_init(dbidev, &ili9488_pipe_funcs, &ili9488_mode, rotation);
	ret = mipi_dbi_dev_init_with_formats(dbidev, &ili9488_pipe_funcs,
					     ili9488_formats,
					     ARRAY_SIZE(ili9488_formats),
					     &ili9488_mode, rotation, bufsize);
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

MODULE_DESCRIPTION("Ilitek ILI9488 DRM driver");
MODULE_AUTHOR("BIRD TECHSTEP");
MODULE_LICENSE("GPL");

