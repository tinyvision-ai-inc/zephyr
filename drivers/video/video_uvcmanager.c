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
LOG_MODULE_REGISTER(uvcmanager, CONFIG_VIDEO_LOG_LEVEL);

struct uvcmanager_conf {
	const struct device *source_dev;
	uintptr_t buf_addr;
};

struct uvcmanager_data {
	struct k_fifo fifo;
	struct video_format format;
};

static int uvcmanager_init(const struct device *dev)
{
	struct uvcmanager_data *data = dev->data;

	k_fifo_init(&data->fifo);
	return 0;
}

static int uvcmanager_stream_start(const struct device *dev)
{
	const struct uvcmanager_conf *conf = dev->config;
	int err;

	if (conf->source_dev != NULL) {
		err = video_stream_start(conf->source_dev);
		if (err) {
			LOG_ERR("failed to start the source device");
			return err;
		}
	}

	return 0;
}

static int uvcmanager_stream_stop(const struct device *dev)
{
	const struct uvcmanager_conf *conf = dev->config;
	int err;

	if (conf->source_dev != NULL) {
		err = video_stream_start(conf->source_dev);
		if (err) {
			LOG_ERR("failed to stop the source device");
			return err;
		}
	}

	return 0;
}

#define FORMAT_CAP(fmt, wmax, hmax)                                                                \
	{                                                                                          \
		.pixelformat = fmt, .width_min = 1, .width_max = wmax, .height_min = 1,            \
		.height_max = hmax, .width_step = 1, .height_step = 1,                             \
	}

static const struct video_format_cap fmts[] = {
	FORMAT_CAP(VIDEO_PIX_FMT_BGGR8, UINT32_MAX, UINT32_MAX),
	FORMAT_CAP(VIDEO_PIX_FMT_GBRG8, UINT32_MAX, UINT32_MAX),
	FORMAT_CAP(VIDEO_PIX_FMT_GRBG8, UINT32_MAX, UINT32_MAX),
	FORMAT_CAP(VIDEO_PIX_FMT_RGGB8, UINT32_MAX, UINT32_MAX),
	FORMAT_CAP(VIDEO_PIX_FMT_RGB565, UINT32_MAX, UINT32_MAX),
	FORMAT_CAP(VIDEO_PIX_FMT_YUYV, UINT32_MAX, UINT32_MAX),
	{0}};

static int uvcmanager_get_caps(const struct device *dev, enum video_endpoint_id ep, struct video_caps *caps)
{
	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ANY) {
		return -EINVAL;
	}

	caps->format_caps = fmts;
	return 0;
}

static int uvcmanager_set_format(const struct device *dev, enum video_endpoint_id ep, struct video_format *fmt)
{
	struct uvcmanager_data *data = dev->data;
	const struct uvcmanager_conf *conf = dev->config;
	struct video_format child_fmt = *fmt;
	int err;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ANY) {
		return -EINVAL;
	}

	LOG_DBG("setting format to %ux%u", fmt->width, fmt->height);

	if (conf->source_dev != NULL) {
		/* Requires 2 extra pixels for debayer */
		child_fmt.width += 2;
		child_fmt.height += 2;

		err = video_set_format(conf->source_dev, VIDEO_EP_OUT, &child_fmt);
		if (err) {
			LOG_ERR("failed to set source device's format to %ux%u",
				child_fmt.width, child_fmt.height);
			return err;
		}
	}
	data->format = *fmt;

	return 0;
}

static int uvcmanager_get_format(const struct device *dev, enum video_endpoint_id ep, struct video_format *fmt)
{
	struct uvcmanager_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ANY) {
		return -EINVAL;
	}

	*fmt = data->format;
	return 0;
}

static int uvcmanager_enqueue(const struct device *dev, enum video_endpoint_id ep, struct video_buffer *vbuf)
{
	const struct uvcmanager_conf *conf = dev->config;
	struct uvcmanager_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ANY) {
		return -EINVAL;
	}

	/* No data transfer: the data is memory mapped by the hardware and immediately ready */
	vbuf->buffer = (uint8_t *)conf->buf_addr;
	vbuf->bytesused = vbuf->size;

	LOG_DBG("uvcmanager: enqueuing vbuf=%p data=%p size=%u bytesused=%u",
		vbuf, vbuf->buffer, vbuf->size, vbuf->bytesused);

	k_fifo_put(&data->fifo, vbuf);
	return 0;
}

static int uvcmanager_dequeue(const struct device *dev, enum video_endpoint_id ep, struct video_buffer **vbufp, k_timeout_t timeout)
{
	struct uvcmanager_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ANY) {
		return -EINVAL;
	}

	*vbufp = k_fifo_get(&data->fifo, timeout);
	if (*vbufp == NULL) {
		LOG_ERR("uvcmanager: Cannot dequeue a buffer");
		return -EIO;
	}

	LOG_DBG("uvcmanager: dequeuing vbufp=%p", *vbufp);

	return 0;
}

static const struct video_driver_api uvcmanager_driver_api = {
	.set_format = uvcmanager_set_format,
	.get_format = uvcmanager_get_format,
	.get_caps = uvcmanager_get_caps,
	.stream_start = uvcmanager_stream_start,
	.stream_stop = uvcmanager_stream_stop,
	.enqueue = uvcmanager_enqueue,
	.dequeue = uvcmanager_dequeue,
};

#define DT_DRV_COMPAT tinyvision_uvcmanager

#define UVCMANAGER_DEVICE_DEFINE(n)                                                                \
                                                                                                   \
	const struct uvcmanager_conf uvcmanager_conf_##n = {                                       \
		.buf_addr = DT_INST_REG_ADDR_BY_NAME(n, fifo),                                     \
		.source_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(0, source)),                   \
	};                                                                                         \
                                                                                                   \
	struct uvcmanager_data uvcmanager_data_##n;                                                \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, uvcmanager_init, NULL, &uvcmanager_data_##n, &uvcmanager_conf_##n,\
			      POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &uvcmanager_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UVCMANAGER_DEVICE_DEFINE)
