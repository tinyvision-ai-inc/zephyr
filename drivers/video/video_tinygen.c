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
	struct video_format format;
};

static int tinygen_init(const struct device *dev)
{
	struct tinygen_data *data = dev->data;

	k_fifo_init(&data->fifo);
	return 0;
}

static int tinygen_stream_start(const struct device *dev)
{
	LOG_DBG("tinygen: start");
	return 0;
}

static int tinygen_stream_stop(const struct device *dev)
{
	LOG_DBG("tinygen: stop");
	return 0;
}

static int tinygen_set_ctrl(const struct device *dev, unsigned int cid, void *p)
{
	LOG_DBG("tinygen: set cid=%d val=%p", cid, p);
	return -ENOTSUP;
}

static int tinygen_get_ctrl(const struct device *dev, unsigned int cid, void *p)
{
	LOG_DBG("tinygen: get cid=%d", cid);
	return -ENOTSUP;
}

#define FORMAT_CAP(fmt)                                                                            \
	{                                                                                          \
		.pixelformat = fmt, .width_min = 1, .width_max = 3280, .height_min = 1,            \
		.height_max = 2464, .width_step = 1, .height_step = 1,                             \
	}

static const struct video_format_cap fmts[] = {
	FORMAT_CAP(VIDEO_PIX_FMT_BGGR8),
	FORMAT_CAP(VIDEO_PIX_FMT_GBRG8),
	FORMAT_CAP(VIDEO_PIX_FMT_GRBG8),
	FORMAT_CAP(VIDEO_PIX_FMT_RGGB8),
	FORMAT_CAP(VIDEO_PIX_FMT_RGB565),
	FORMAT_CAP(VIDEO_PIX_FMT_YUYV),
	{0}};

static int tinygen_get_caps(const struct device *dev, enum video_endpoint_id ep, struct video_caps *caps)
{
	LOG_DBG("tinygen: caps");
	caps->format_caps = fmts;
	return 0;
}

static int tinygen_set_format(const struct device *dev, enum video_endpoint_id ep, struct video_format *format)
{
	struct tinygen_data *data = dev->data;

	LOG_DBG("tinygen: set ep=%d format=%p", ep, format);
	memcpy(&data->format, format, sizeof(data->format));
	return 0;
}

static int tinygen_get_format(const struct device *dev, enum video_endpoint_id ep, struct video_format *format)
{
	struct tinygen_data *data = dev->data;

	LOG_DBG("tinygen: get ep=%d", ep);
	memcpy(format, &data->format, sizeof(*format));
	return 0;
}

static int tinygen_enqueue(const struct device *dev, enum video_endpoint_id ep, struct video_buffer *vbuf)
{
	const struct tinygen_conf *conf = dev->config;
	struct tinygen_data *data = dev->data;

	/* No data transfer: the data is memory mapped by the hardware and immediately ready */
	vbuf->buffer = (uint8_t *)conf->buf_addr;
	vbuf->bytesused = MIN(vbuf->size, conf->buf_size);

	LOG_DBG("tinygen: enqueuing vbuf=%p data=%p size=%u bytesused=%u max=%u",
		vbuf, vbuf->buffer, vbuf->size, vbuf->bytesused, conf->buf_size);

	k_fifo_put(&data->fifo, vbuf);
	return 0;
}

static int tinygen_dequeue(const struct device *dev, enum video_endpoint_id ep, struct video_buffer **vbufp, k_timeout_t timeout)
{
	struct tinygen_data *data = dev->data;

	*vbufp = k_fifo_get(&data->fifo, timeout);
	if (*vbufp == NULL) {
		LOG_ERR("tinygen: Cannot dequeue a buffer");
		return -EIO;
	}

	LOG_DBG("tinygen: dequeuing vbufp=%p", *vbufp);

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
