/*
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_emul_mipi_rx

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "video_common.h"

LOG_MODULE_REGISTER(video_emul_mipi_rx, CONFIG_VIDEO_LOG_LEVEL);

#define EMUL_MIPI_RX_FPS 15

enum emul_mipi_rx_fmt_id {
	BGGR8,
	RGB565,
	YUYV,
};

struct emul_mipi_rx_config {
	const struct device *source_dev;
};

struct emul_mipi_rx_data {
	uint16_t fps;
	enum emul_mipi_rx_fmt_id fmt_id;
	struct video_format fmt;
	const struct device *dev;
	struct k_work work;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
};

/* Video device capabilities where the supported resolutions and pixel formats are listed. */
static const struct video_format_cap fmts[] = {
	[BGGR8] = {.pixelformat = VIDEO_PIX_FMT_BGGR8,
		   .width_min = 2, .width_max = 64, .width_step = 2,
		   .height_min = 2, .height_max = 32, .height_step = 2},
	[RGB565] = {.pixelformat = VIDEO_PIX_FMT_RGB565,
		    .width_min = 2, .width_max = 64, .width_step = 2,
		    .height_min = 1, .height_max = 20, .height_step = 1},
	[YUYV] = {.pixelformat = VIDEO_PIX_FMT_YUYV,
		  .width_min = 2, .width_max = 64, .width_step = 2,
		  .height_min = 1, .height_max = 20, .height_step = 1},
	{0},
};

static int emul_mipi_rx_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct emul_mipi_rx_config *conf = dev->config;

	return video_set_ctrl(conf->source_dev, cid, value);
}

static int emul_mipi_rx_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct emul_mipi_rx_config *conf = dev->config;

	return video_get_ctrl(conf->source_dev, cid, value);
}

static int emul_mipi_rx_apply_mode(const struct device *dev)
{
	return 0;
}

static int emul_mipi_rx_set_frmival(const struct device *dev, enum video_endpoint_id ep,
				    struct video_frmival *frmival)
{
	struct emul_mipi_rx_data *data = dev->data;
	struct video_frmival_enum fie = {.format = &data->fmt, .discrete = *frmival};

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	video_closest_frmival(dev, ep, &fie);
	data->fps = fie.discrete.denominator / fie.discrete.numerator;
	return emul_mipi_rx_apply_mode(dev);
}

static int emul_mipi_rx_get_frmival(const struct device *dev, enum video_endpoint_id ep,
				    struct video_frmival *frmival)
{
	struct emul_mipi_rx_data *data = dev->data;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	frmival->numerator = 1;
	frmival->denominator = data->fps;
	return 0;
}

static int emul_mipi_rx_enum_frmival(const struct device *dev, enum video_endpoint_id ep,
				     struct video_frmival_enum *fie)
{
	size_t fmt_id;
	int ret;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_IN && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	/* Only 1 FPS supported for this emulated driver */
	if (fie->index >= 0) {
		return 1;
	}

	ret = video_format_index(fmts, fie->format, &fmt_id);
	if (ret < 0) {
		return ret;
	}

	fie->type = VIDEO_FRMIVAL_TYPE_DISCRETE;
	fie->discrete.numerator = 1;
	fie->discrete.denominator = EMUL_MIPI_RX_FPS;
	fie->index++;

	return 0;
}

static int emul_mipi_rx_set_fmt(const struct device *const dev, enum video_endpoint_id ep,
				struct video_format *fmt)
{
	const struct emul_mipi_rx_config *conf = dev->config;
	struct emul_mipi_rx_data *data = dev->data;
	size_t fmt_id;
	int ret;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_IN && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	if (memcmp(&data->fmt, fmt, sizeof(data->fmt)) == 0) {
		return 0;
	}

	ret = video_format_index(fmts, fmt, &fmt_id);
	if (ret < 0) {
		LOG_ERR("Format %x %ux%u not found for %s",
			fmt->pixelformat, fmt->width, fmt->height, dev->name);
		return ret;
	}

	/* TODO: if fmt_in != fmt_out, convert between formats */

	ret = video_set_format(conf->source_dev, ep, fmt);
	if (ret < 0) {
		LOG_ERR("Failed to apply format %x %ux%u to source device %s",
			fmt->pixelformat, fmt->width, fmt->height, conf->source_dev->name);
		return ret;
	}

	ret = emul_mipi_rx_apply_mode(dev);
	if (ret < 0) {
		LOG_ERR("Failed to apply %s mode", dev->name);
		return ret;
	}

	data->fmt_id = fmt_id;
	data->fmt = *fmt;
	return 0;
}

static int emul_mipi_rx_get_fmt(const struct device *dev, enum video_endpoint_id ep,
				struct video_format *fmt)
{
	struct emul_mipi_rx_data *data = dev->data;

	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	*fmt = data->fmt;
	return 0;
}

static int emul_mipi_rx_get_caps(const struct device *dev, enum video_endpoint_id ep,
				 struct video_caps *caps)
{
	if (ep != VIDEO_EP_IN && ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	caps->format_caps = fmts;
	return 0;
}

static int emul_mipi_rx_stream_start(const struct device *dev)
{
	const struct emul_mipi_rx_config *conf = dev->config;

	return video_stream_start(conf->source_dev);
}

static int emul_mipi_rx_stream_stop(const struct device *dev)
{
	const struct emul_mipi_rx_config *conf = dev->config;

	return video_stream_start(conf->source_dev);
}

static void emul_mipi_rx_worker(struct k_work *work)
{
	struct emul_mipi_rx_data *data = CONTAINER_OF(work, struct emul_mipi_rx_data, work);
	const struct device *dev = data->dev;
	const struct emul_mipi_rx_config *conf = dev->config;
	struct video_format *fmt = &data->fmt;
	struct video_buffer *vbuf = vbuf;

	LOG_DBG("Queueing a frame of %u bytes in format %x %ux%u",
		fmt->pitch * fmt->height, fmt->pixelformat, fmt->width, fmt->height);

	while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT)) != NULL) {
		vbuf->bytesused = MIN(vbuf->size, fmt->pitch * fmt->height);
		vbuf->flags = VIDEO_BUF_EOF;

		LOG_DBG("Inserting %u bytes into %s buffer %p",
			vbuf->bytesused, dev->name, vbuf->buffer);

		/* Simulate the MIPI hardware transferring image data from the imager to the video
		 * buffer memory using DMA */
		memcpy(vbuf->buffer, conf->source_dev->data, vbuf->bytesused);

		/* Once the buffer is completed, submit it to the video buffer */
		k_fifo_put(&data->fifo_out, vbuf);
	}
}

static int emul_mipi_rx_enqueue(const struct device *dev, enum video_endpoint_id ep,
				struct video_buffer *vbuf)
{
	struct emul_mipi_rx_data *data = dev->data;

	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	if (data->fmt.pixelformat == 0) {
		LOG_DBG("%s format is not configured yet", dev->name);
		return -EINVAL;
	}

	k_fifo_put(&data->fifo_in, vbuf);
	k_work_submit(&data->work);

	return 0;
}

static int emul_mipi_rx_dequeue(const struct device *dev, enum video_endpoint_id ep,
				struct video_buffer **vbufp, k_timeout_t timeout)
{
	struct emul_mipi_rx_data *data = dev->data;

	if (ep != VIDEO_EP_OUT) {
		return -EINVAL;
	}

	*vbufp = k_fifo_get(&data->fifo_out, timeout);
	if (*vbufp == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static const struct video_driver_api emul_mipi_rx_driver_api = {
	.set_ctrl = emul_mipi_rx_set_ctrl,
	.get_ctrl = emul_mipi_rx_get_ctrl,
	.set_frmival = emul_mipi_rx_set_frmival,
	.get_frmival = emul_mipi_rx_get_frmival,
	.enum_frmival = emul_mipi_rx_enum_frmival,
	.set_format = emul_mipi_rx_set_fmt,
	.get_format = emul_mipi_rx_get_fmt,
	.get_caps = emul_mipi_rx_get_caps,
	.stream_start = emul_mipi_rx_stream_start,
	.stream_stop = emul_mipi_rx_stream_stop,
	.enqueue = emul_mipi_rx_enqueue,
	.dequeue = emul_mipi_rx_dequeue,
};

int emul_mipi_rx_init(const struct device *dev)
{
	struct emul_mipi_rx_data *data = dev->data;
	const struct emul_mipi_rx_config *conf = dev->config;
	struct video_format fmt;

	if (!device_is_ready(conf->source_dev)) {
		LOG_ERR("Source device %s is not ready", conf->source_dev->name);
		return -ENODEV;
	}

	data->dev = dev;
	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	k_work_init(&data->work, &emul_mipi_rx_worker);

	fmt.pixelformat = fmts[0].pixelformat;
	fmt.width = fmts[0].width_max;
	fmt.height = fmts[0].height_max;
	fmt.pitch = fmt.width * video_bits_per_pixel(fmt.pixelformat) / 8;

	return emul_mipi_rx_set_fmt(dev, VIDEO_EP_OUT, &fmt);
}

#define VIDEO_DT_REMOTE_ENDPOINT(node, port, ep)                                                   \
	DT_NODELABEL(DT_STRING_TOKEN(DT_CHILD(DT_CHILD(node, port), ep), remote_endpoint_label))

#define VIDEO_DT_REMOTE_DEVICE(node, port, ep)                                                     \
	DEVICE_DT_GET(DT_GPARENT(VIDEO_DT_REMOTE_ENDPOINT(node, port, ep)))

#define VIDEO_DT_INST_REMOTE_DEVICE(inst, port, endpoint)                                          \
	VIDEO_DT_REMOTE_DEVICE(DT_DRV_INST(inst), port, endpoint)

#define EMUL_MIPI_RX_DEFINE(inst)                                                                  \
	static struct emul_mipi_rx_data emul_mipi_rx_data_##inst;                                  \
                                                                                                   \
	static const struct emul_mipi_rx_config emul_mipi_rx_conf_##inst = {                       \
		.source_dev = VIDEO_DT_INST_REMOTE_DEVICE(inst, port, endpoint_1),                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &emul_mipi_rx_init, NULL, &emul_mipi_rx_data_##inst,           \
			      &emul_mipi_rx_conf_##inst, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,  \
			      &emul_mipi_rx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(EMUL_MIPI_RX_DEFINE)
