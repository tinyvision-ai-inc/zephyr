/*
 * Copyright (c) 2024, tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/i2c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tinygen, CONFIG_VIDEO_LOG_LEVEL);

struct tinygen_conf {
	uintptr_t buf_addr;
	size_t buf_size;
};

static int tinygen_init(const struct device *dev)
{
	LOG_DBG("%s", __func__);
	return 0;
}

static int tinygen_stream_start(const struct video_dt_spec *spec)
{
	LOG_DBG("%s", __func__);
	return 0;
}

static int tinygen_stream_stop(const struct video_dt_spec *spec)
{
	LOG_DBG("%s", __func__);
	return 0;
}

static int tinygen_set_ctrl(const struct video_dt_spec *spec, unsigned int cid, void *p)
{
	LOG_DBG("%s", __func__);
	return -ENOTSUP;
}

static int tinygen_get_ctrl(const struct video_dt_spec *spec, unsigned int cid, void *p)
{
	LOG_DBG("%s", __func__);
	return -ENOTSUP;
}

#define FORMAT_CAP(fmt)                                                                            \
	{                                                                                          \
		.pixelformat = fmt, .width_min = 1, .width_max = 3280, .height_min = 1,            \
		.height_max = 2464, .width_step = 1, .height_step = 1,                             \
	}

static const struct video_format_cap fmts[] = {
	FORMAT_CAP(VIDEO_PIX_FMT_BGGR8),  FORMAT_CAP(VIDEO_PIX_FMT_GBRG8),
	FORMAT_CAP(VIDEO_PIX_FMT_GRBG8),  FORMAT_CAP(VIDEO_PIX_FMT_RGGB8),
	FORMAT_CAP(VIDEO_PIX_FMT_RGB565), FORMAT_CAP(VIDEO_PIX_FMT_YUYV),
	FORMAT_CAP(VIDEO_PIX_FMT_JPEG),   {0}};

static int tinygen_get_caps(const struct video_dt_spec *spec, struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static int tinygen_set_format(const struct video_dt_spec *spec, struct video_format *format)
{
	return -ENOTSUP;
}

static int tinygen_get_format(const struct video_dt_spec *spec, struct video_format *format)
{
	return -ENOTSUP;
}

static int tinygen_enqueue(const struct video_dt_spec *spec, struct video_buffer *buf)
{
	const struct tinygen_conf *conf = spec->dev->config;

	if (buf->size > conf->buf_size) {
		return -EINVAL;
	}

	buf->buffer = (uint8_t *)conf->buf_addr;
	return 0;
}

static int tinygen_dequeue(const struct video_dt_spec *spec, struct video_buffer *buf)
{
	return 0;
}

static const struct video_driver_api tinygen_driver_api = {
	.set_format = tinygen_set_format,
	.get_format = tinygen_get_format,
	.set_ctrl = tinygen_set_ctrl,
	.get_ctrl = tinygen_get_ctrl,
	.get_caps = tinygen_get_caps,
	.stream_start = tinygen_stream_start,
	.stream_stop = tinygen_stream_stop,
	.enqueue = tinygen_enqueue,
	.dequeue = tinygen_dequeue,
};

#define DT_DRV_COMPAT tinyvision_tinygen

#define TINYGEN_DEVICE_DEFINE(n)                                                                   \
                                                                                                   \
	struct tinygen_conf tinygen_conf_##n = {                                                   \
		.buf_addr = DT_INST_REG_ADDR(n),                                                   \
		.buf_size = DT_INST_REG_SIZE(n),                                                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, tinygen_init, NULL, NULL, &tinygen_conf_##n, POST_KERNEL,         \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &tinygen_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TINYGEN_DEVICE_DEFINE)
