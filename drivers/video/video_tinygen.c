/*
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tinygen, CONFIG_VIDEO_LOG_LEVEL);

struct tinygen_conf {
	uintptr_t buf_addr;
	size_t buf_size;
};

struct tinygen_data {
	struct k_fifo fifo;
};

static int tinygen_init(const struct device *dev)
{
	struct tinygen_data *data = dev->data;

	LOG_DBG("%s", __func__);
	k_fifo_init(&data->fifo);
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

static int tinygen_enqueue(const struct video_dt_spec *spec, struct video_buffer *vbuf)
{
	const struct tinygen_conf *conf = spec->dev->config;
	struct tinygen_data *data = spec->dev->data;

	LOG_DBG("%s", __func__);

	/* No data transfer: the data is memory mapped by the hardware and immediately ready */
	vbuf->buffer = (uint8_t *)conf->buf_addr;
	vbuf->bytesused = MIN(vbuf->size, conf->buf_size);

	k_fifo_put(&data->fifo, vbuf);
	return 0;
}

static int tinygen_dequeue(const struct video_dt_spec *spec, struct video_buffer **vbufp, k_timeout_t timeout)
{
	struct tinygen_data *data = spec->dev->data;

	LOG_DBG("%s", __func__);

	*vbufp = k_fifo_get(&data->fifo, timeout);
	if (*vbufp == NULL) {
		LOG_ERR("cannot dequeue a buffer");
		return -EIO;
	}

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
	struct tinygen_data tinygen_data_##n;                                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, tinygen_init, NULL, &tinygen_data_##n, &tinygen_conf_##n,         \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &tinygen_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TINYGEN_DEVICE_DEFINE)
