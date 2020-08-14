// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM driver for SX035HV006 panels
 *
 * Copyright 2016 Noralf Trønnes
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
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

#include <drm/drm_damage_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_rect.h>
#include <drm/drm_vblank.h>
#include <drm/drm_fourcc.h>

#define ILI9488_CMD_NOP					0x00
#define ILI9488_CMD_SOFTWARE_RESET			0x01
#define ILI9488_CMD_READ_DISP_ID			0x04
#define ILI9488_CMD_READ_ERROR_DSI			0x05
#define ILI9488_CMD_READ_DISP_STATUS			0x09
#define ILI9488_CMD_READ_DISP_POWER_MODE		0x0A
#define ILI9488_CMD_READ_DISP_MADCTRL			0x0B
#define ILI9488_CMD_READ_DISP_PIXEL_FORMAT		0x0C
#define ILI9488_CMD_READ_DISP_IMAGE_MODE		0x0D
#define ILI9488_CMD_READ_DISP_SIGNAL_MODE		0x0E
#define ILI9488_CMD_READ_DISP_SELF_DIAGNOSTIC		0x0F
#define ILI9488_CMD_ENTER_SLEEP_MODE			0x10
#define ILI9488_CMD_SLEEP_OUT				0x11
#define ILI9488_CMD_PARTIAL_MODE_ON			0x12
#define ILI9488_CMD_NORMAL_DISP_MODE_ON			0x13
#define ILI9488_CMD_DISP_INVERSION_OFF			0x20
#define ILI9488_CMD_DISP_INVERSION_ON			0x21
#define ILI9488_CMD_PIXEL_OFF				0x22
#define ILI9488_CMD_PIXEL_ON				0x23
#define ILI9488_CMD_DISPLAY_OFF				0x28
#define ILI9488_CMD_DISPLAY_ON				0x29
#define ILI9488_CMD_COLUMN_ADDRESS_SET			0x2A
#define ILI9488_CMD_PAGE_ADDRESS_SET			0x2B
#define ILI9488_CMD_MEMORY_WRITE			0x2C
#define ILI9488_CMD_MEMORY_READ				0x2E
#define ILI9488_CMD_PARTIAL_AREA			0x30
#define ILI9488_CMD_VERT_SCROLL_DEFINITION		0x33
#define ILI9488_CMD_TEARING_EFFECT_LINE_OFF		0x34
#define ILI9488_CMD_TEARING_EFFECT_LINE_ON		0x35
#define ILI9488_CMD_MEMORY_ACCESS_CONTROL		0x36
#define ILI9488_CMD_VERT_SCROLL_START_ADDRESS		0x37
#define ILI9488_CMD_IDLE_MODE_OFF			0x38
#define ILI9488_CMD_IDLE_MODE_ON			0x39
#define ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET		0x3A
#define ILI9488_CMD_WRITE_MEMORY_CONTINUE		0x3C
#define ILI9488_CMD_READ_MEMORY_CONTINUE		0x3E
#define ILI9488_CMD_SET_TEAR_SCANLINE			0x44
#define ILI9488_CMD_GET_SCANLINE			0x45
#define ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS		0x51
#define ILI9488_CMD_READ_DISPLAY_BRIGHTNESS		0x52
#define ILI9488_CMD_WRITE_CTRL_DISPLAY			0x53
#define ILI9488_CMD_READ_CTRL_DISPLAY			0x54
#define ILI9488_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS	0x55
#define ILI9488_CMD_READ_CONTENT_ADAPT_BRIGHTNESS	0x56
#define ILI9488_CMD_WRITE_MIN_CAB_LEVEL			0x5E
#define ILI9488_CMD_READ_MIN_CAB_LEVEL			0x5F
#define ILI9488_CMD_READ_ABC_SELF_DIAG_RES		0x68
#define ILI9488_CMD_READ_ID1				0xDA
#define ILI9488_CMD_READ_ID2				0xDB
#define ILI9488_CMD_READ_ID3				0xDC

/* Level 2 Commands (from the display Datasheet) */
#define ILI9488_CMD_INTERFACE_MODE_CONTROL		0xB0
#define ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL		0xB1
#define ILI9488_CMD_FRAME_RATE_CONTROL_IDLE_8COLOR	0xB2
#define ILI9488_CMD_FRAME_RATE_CONTROL_PARTIAL		0xB3
#define ILI9488_CMD_DISPLAY_INVERSION_CONTROL		0xB4
#define ILI9488_CMD_BLANKING_PORCH_CONTROL		0xB5
#define ILI9488_CMD_DISPLAY_FUNCTION_CONTROL		0xB6
#define ILI9488_CMD_ENTRY_MODE_SET			0xB7
#define ILI9488_CMD_BACKLIGHT_CONTROL_1			0xB9
#define ILI9488_CMD_BACKLIGHT_CONTROL_2			0xBA
#define ILI9488_CMD_HS_LANES_CONTROL			0xBE
#define ILI9488_CMD_POWER_CONTROL_1			0xC0
#define ILI9488_CMD_POWER_CONTROL_2			0xC1
#define ILI9488_CMD_POWER_CONTROL_NORMAL_3		0xC2
#define ILI9488_CMD_POWER_CONTROL_IDEL_4		0xC3
#define ILI9488_CMD_POWER_CONTROL_PARTIAL_5		0xC4
#define ILI9488_CMD_VCOM_CONTROL_1			0xC5
#define ILI9488_CMD_CABC_CONTROL_1			0xC6
#define ILI9488_CMD_CABC_CONTROL_2			0xC8
#define ILI9488_CMD_CABC_CONTROL_3			0xC9
#define ILI9488_CMD_CABC_CONTROL_4			0xCA
#define ILI9488_CMD_CABC_CONTROL_5			0xCB
#define ILI9488_CMD_CABC_CONTROL_6			0xCC
#define ILI9488_CMD_CABC_CONTROL_7			0xCD
#define ILI9488_CMD_CABC_CONTROL_8			0xCE
#define ILI9488_CMD_CABC_CONTROL_9			0xCF
#define ILI9488_CMD_NVMEM_WRITE				0xD0
#define ILI9488_CMD_NVMEM_PROTECTION_KEY		0xD1
#define ILI9488_CMD_NVMEM_STATUS_READ			0xD2
#define ILI9488_CMD_READ_ID4				0xD3
#define ILI9488_CMD_ADJUST_CONTROL_1			0xD7
#define ILI9488_CMD_READ_ID_VERSION			0xD8
#define ILI9488_CMD_POSITIVE_GAMMA_CORRECTION		0xE0
#define ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION		0xE1
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_1		0xE2
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_2		0xE3
#define ILI9488_CMD_SET_IMAGE_FUNCTION			0xE9
#define ILI9488_CMD_ADJUST_CONTROL_2			0xF2
#define ILI9488_CMD_ADJUST_CONTROL_3			0xF7
#define ILI9488_CMD_ADJUST_CONTROL_4			0xF8
#define ILI9488_CMD_ADJUST_CONTROL_5			0xF9
#define ILI9488_CMD_SPI_READ_SETTINGS			0xFB
#define ILI9488_CMD_ADJUST_CONTROL_6			0xFC
#define ILI9488_CMD_ADJUST_CONTROL_7			0xFF


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

/*
 * ILI9488 Memory Access Control flags
 */
#define ILI9488_MY	BIT(7)		/* Row Address Order */
#define ILI9488_MX	BIT(6)		/* Column Address Order */
#define ILI9488_MV	BIT(5)		/* Row / Column Exchange */
#define ILI9488_ML	BIT(4)		/* Vertical Refresh Order */
#define ILI9488_BGR	BIT(3)		/* BGR Order, if set */
#define ILI9488_MH	BIT(2)		/* Horizontal Refresh Order */

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
	mipi_dbi_command(dpi, ILI9488_CMD_POSITIVE_GAMMA_CORRECTION,
			 0x00, 0x03, 0x09, 0x08, 0x16,
			 0x0a, 0x3f, 0x78, 0x4c, 0x09,
			 0x0a, 0x08, 0x16, 0x1a, 0x0f);

	/* Negative Gamma Control */
	mipi_dbi_command(dpi, ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION,
			 0x00, 0x16, 0x19, 0x03, 0x0f,
			 0x05, 0x32, 0x45, 0x46, 0x04,
			 0x0e, 0x0d, 0x35, 0x37, 0x0f);


	/* Power Control 1 */
	mipi_dbi_command(dpi, ILI9488_CMD_POWER_CONTROL_1, 0x17, 0x15);

	/* Power Control 2 */
	mipi_dbi_command(dpi, ILI9488_CMD_POWER_CONTROL_2, 0x41);

	/* Power Control 3 (Normal mode) */
	mipi_dbi_command(dpi, ILI9488_CMD_POWER_CONTROL_NORMAL_3, 0x44);


	/* VCOM Control 1 */
	mipi_dbi_command(dpi, ILI9488_CMD_VCOM_CONTROL_1, 0x00, 0x12, 0x80);


	/* Pixel Format */
	mipi_dbi_command(dpi, ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET,
			 ILI9488_DBI_BPP18 | ILI9488_DPI_BPP18);


	mipi_dbi_command(dpi, ILI9488_CMD_INTERFACE_MODE_CONTROL, 0x80);


	/* Frame Rate Control */
	/*	Frame rate = 60.76Hz.*/
	mipi_dbi_command(dpi, ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL, 0xa0);


	/* Display Inversion Control */
	/*	2 dot inversion */
	mipi_dbi_command(dpi, ILI9488_CMD_DISPLAY_INVERSION_CONTROL, 0x02);


	/* Set Image Function */
	mipi_dbi_command(dpi, ILI9488_CMD_SET_IMAGE_FUNCTION, 0x00);


	/* Adjust Control 3 */
	mipi_dbi_command(dpi, ILI9488_CMD_ADJUST_CONTROL_3,
			 0xa9, 0x51, 0x2c, 0x82);

	/* CABC control 2 */
	mipi_dbi_command(dpi, ILI9488_CMD_CABC_CONTROL_2, 0xb0);


	/* Sleep OUT */
	mipi_dbi_command(dpi, ILI9488_CMD_SLEEP_OUT);

	msleep(120);

	mipi_dbi_command(dpi, ILI9488_CMD_NORMAL_DISP_MODE_ON);

	/* Display ON */
	mipi_dbi_command(dpi, ILI9488_CMD_DISPLAY_ON);
	msleep(100);

out_enable:
	/* The PiTFT (ili9340) has a hardware reset circuit that
	 * resets only on power-on and not on each reboot through
	 * a gpio like the rpi-display does.
	 * As a result, we need to always apply the rotation value
	 * regardless of the display "on/off" state.
	 */
	switch (dbidev->rotation) {
	case 270:
		addr_mode = ( ILI9488_MX | ILI9488_MY | ILI9488_MV | ILI9488_ML );
		break;
	case 180:
		addr_mode = ( ILI9488_MY | ILI9488_ML );
		break;
	case 90:
		addr_mode = ILI9488_MV;
		break;
	case 0:
	default:
		addr_mode = ILI9488_MX;
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
