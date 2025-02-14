/*
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_video_sw_stats

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/video.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(video_sw_stats, CONFIG_VIDEO_LOG_LEVEL);

struct video_sw_stats_config {
	const struct device *source_dev;
};

struct video_sw_stats_data {
	const struct device *dev;
	struct video_format fmt;
	struct k_work work;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
};

struct video_sw_stats_worker()
{
	uint16_t x;

	/* Accumulate RGB channels statistics by sampling one pixel per row. */
	x = sys_rand16_get() % fmt->width;
	vbuf->buffer[x * video_bits_per_pixel(fmt->pixelformat) / BYTES_PER_BITS];
}

struct video_sw_stats_enqueue(const struct device *dev)
{
	;
}

struct video_sw_stats_dequeue(const struct device *dev)
{
	;
}

struct video_sw_stats_get_stats()
{
	;
}
