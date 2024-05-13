/*
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pixelpump, CONFIG_VIDEO_LOG_LEVEL);

struct pixelpump_conf {
	uintptr_t base;
};

struct pixelpump_data {
	struct k_fifo fifo;
};

#define PIXELPUMP_INT_RAW                                0x0000
#define PIXELPUMP_INT_FORCE                              0x0004
#define PIXELPUMP_INT_MASK                               0x0008
#define PIXELPUMP_INT_MASK_IRQMASK_MASK                  0x00000001 // RW, 1 bit
#define PIXELPUMP_INT_STATUS                             0x000c
#define PIXELPUMP_INT_STATUS_IRQSTATUS_MASK              0x00000001 // RO, 1 bit
#define PIXELPUMP_SCRATCH_REG                            0x0010
#define PIXELPUMP_CONTROL_STATUS_REG                     0x0014
#define PIXELPUMP_CONTROL_STATUS_REG_ENABLE_MASK         0x00000001 // RW, 1 bit
#define PIXELPUMP_CONTROL_STATUS_REG_BUSY_SHIFT          1
#define PIXELPUMP_CONTROL_STATUS_REG_BUSY_MASK           0x00000002 // RO, 1 bit
#define PIXELPUMP_NUM_WORDS                              0x0018
#define PIXELPUMP_NUM_WORDS_NUMWORDS_MASK                0x000007ff // RW, 11 bit
#define PIXELPUMP_DOORBELL_ADDRESS                       0x001c
#define PIXELPUMP_DOORBELL_ADDRESS_DOORBELLADDRESS_MASK  0x0000007f // RW, 7 bit
#define PIXELPUMP_START_ADDR                             0x0020
#define PIXELPUMP_START_ADDR_PPSTARTADDR_MASK            0x000007ff // RW, 11 bit
#define PIXELPUMP_TPG                                    0x0024
#define PIXELPUMP_TPG_TPGNUMPIXELS_MASK                  0x0000ffff // RW, 16 bit
#define PIXELPUMP_TPG_TPGNUMPAUSE_SHIFT                  16
#define PIXELPUMP_TPG_TPGNUMPAUSE_MASK                   0xffff0000 // RW, 16 bit

/**
 * @address     0x0000
 * @brief       Interrupt Raw status Register, set when event clear raw when write 1
 */
struct pixelpump_int_raw {
	uint32_t irqpixelpump_raw: 1;   // W1C, reset: 0x0, raw, default 0
	uint32_t reserved_0: 31; // NA, Reserved
};

/**
 * @address     0x0004
 * @brief       Interrupt Force  Register\n for SW debug use \n write 1 set raw
 */
struct pixelpump_int_force {
	uint32_t irqpixelpump_raw: 1;   // W1S, reset: 0x0, force, write 1 set, debug use
	uint32_t reserved_0: 31; // NA, Reserved
};

/**
 * @address     0x0008
 * @brief       Interrupt Mask   Register\n1: int off\n0: int open\n default 1, int off
 */
struct pixelpump_int_mask {
	uint32_t irqpixelpump_mask: 1;  // RW, reset: 0x1, mask, default 1, int off
	uint32_t reserved_0: 31; // NA, Reserved
};

/**
 * @address     0x000c
 * @brief       Interrupt status Register\n status = raw && (!mask)
 */
struct pixelpump_int_status {
	uint32_t irqpixelpump_status: 1; // RO, reset: 0x0, stauts default 0
	uint32_t reserved_0: 31;  // NA, Reserved
};

/**
 * @address     0x0010
 * @brief       Scratch
 */
struct pixelpump_scratch_reg {
	uint32_t val;
};

/**
 * @address     0x0014
 * @brief       Control and Status
 */
struct pixelpump_control_status_reg {
	uint32_t enable: 1;      // RW, reset: 0x0, Block enable
	uint32_t busy: 1;        // RO, reset: 0x0, Block is busy when set
	uint32_t reserved_0: 30; // NA, Reserved
};

/**
 * @address     0x0018
 * @brief       Total number of words to transfer in one transaction. A single transaction can have
 * multiple bursts.
 */
struct pixelpump_num_words {
	uint32_t numwords: 11;   // RW, reset: 0x000, Number of words to transfer
	uint32_t reserved_0: 21; // NA, Reserved
};

/**
 * @address     0x001c
 * @brief       Doorbell address to ring for initiating a transfer.
 */
struct pixelpump_doorbell_address {
	uint32_t doorbelladdress: 7; // RW, reset: 0x00, Address
	uint32_t reserved_0: 25;     // NA, Reserved
};

/**
 * @address     0x0020
 * @brief       Starting address to fill the data from the pixel pump.
 */
struct pixelpump_start_addr {
	uint32_t ppstartaddr: 11; // RW, reset: 0x000, Starting address (64 bit aligned!)
	uint32_t reserved_0: 21;  // NA, Reserved
};

/**
 * @address     0x0024
 * @brief       Test Pattern generator configuration.
 */
struct pixelpump_tpg_reg {
	uint32_t tpgnumpixels: 16; // RW, reset: 0x0100, Number of pixels to generate)
	uint32_t tpgnumpause: 16;  // RW, reset: 0x0200, Number of pause clocks between pixels)
};

struct PIXELPUMP {
	struct pixelpump_int_raw int_raw;
	struct pixelpump_int_force int_force;
	struct pixelpump_int_mask int_mask;
	struct pixelpump_int_status int_status;
	struct pixelpump_scratch_reg scratch;
	struct pixelpump_control_status_reg control_status;
	struct pixelpump_num_words num_words;
	struct pixelpump_doorbell_address doorbell_address;
	struct pixelpump_start_addr start_address;
	struct pixelpump_tpg_reg tpg;
};

static void pixelpump_apply_config(const struct device *dev, uint32_t size, uint32_t pause,
				   uint32_t pixels)
{
	struct pixelpump_data *data = dev->data;
	struct PIXELPUMP *PIXELPUMP = (volatile struct PIXELPUMP *)(conf->base);

	PIXELPUMP->control_status.enable = 0;
	PIXELPUMP->num_words.numwords = size * sizeof(uint32_tx);
	PIXELPUMP->tpg.tpgnumpause = pause;
	PIXELPUMP->tpg.tpgnumpixels = pixels;
	PIXELPUMP->control_status.enable = 1;
	while (PIXELPUMP->control_status.busy)
		k_sleep(K_MSEC(1));
}

static int pixelpump_init(const struct device *dev)
{
	struct pixelpump_data *data = dev->data;
	struct PIXELPUMP *PIXELPUMP = (volatile struct PIXELPUMP *)(conf->base);

	LOG_DBG("%s", __func__);
	k_fifo_init(&data->fifo);

	PIXELPUMP->scratch.val = 0x12345678;
	if (PIXELPUMP->scratch.val != 0x12345678) {
		LOG_DBG("Failed testing scratch access, exp: 0x%x, act: 0x%lx", 0x12345678,
		        PIXELPUMP->scratch.val);
		return -EIO;
	}
	return 0;
}

static int pixelpump_stream_start(const struct video_dt_spec *spec)
{
	LOG_DBG("%s", __func__);
	pixelpump_apply_config(0x200, 0x100, 0x200);
	return 0;
}

static int pixelpump_stream_stop(const struct video_dt_spec *spec)
{
	LOG_DBG("%s", __func__);
	return -ENOTSUP;
}

static int pixelpump_set_ctrl(const struct video_dt_spec *spec, unsigned int cid, void *p)
{
	LOG_DBG("%s", __func__);
	return -ENOTSUP;
}

static int pixelpump_get_ctrl(const struct video_dt_spec *spec, unsigned int cid, void *p)
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
	FORMAT_CAP(VIDEO_PIX_FMT_YUYV)};

static int pixelpump_get_caps(const struct video_dt_spec *spec, struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static int pixelpump_set_format(const struct video_dt_spec *spec, struct video_format *format)
{
	return -ENOTSUP;
}

static int pixelpump_get_format(const struct video_dt_spec *spec, struct video_format *format)
{
	return -ENOTSUP;
}

static int pixelpump_enqueue(const struct video_dt_spec *spec, struct video_buffer *vbuf)
{
	const struct pixelpump_conf *conf = spec->dev->config;
	struct pixelpump_data *data = spec->dev->data;

	LOG_DBG("%s", __func__);

	/* No data transfer: the data is memory mapped by the hardware and immediately ready */
	vbuf->buffer = (uint8_t *)conf->buf_addr;
	vbuf->bytesused = MIN(vbuf->size, conf->buf_size);

	k_fifo_put(&data->fifo, vbuf);
	return 0;
}

static int pixelpump_dequeue(const struct video_dt_spec *spec, struct video_buffer **vbufp, k_timeout_t timeout)
{
	const struct pixelpump_conf *conf = spec->dev->config;
	struct pixelpump_data *data = spec->dev->data;

	LOG_DBG("%s", __func__);

	*vbufp = k_fifo_get(&data->fifo, timeout);
	if (*vbufp == NULL) {
		LOG_ERR("cannot dequeue a buffer");
		return -EIO;
	}

	return 0;
}

static const struct video_driver_api pixelpump_driver_api = {
	.set_format = pixelpump_set_format,
	.get_format = pixelpump_get_format,
	.set_ctrl = pixelpump_set_ctrl,
	.get_ctrl = pixelpump_get_ctrl,
	.get_caps = pixelpump_get_caps,
	.stream_start = pixelpump_stream_start,
	.stream_stop = pixelpump_stream_stop,
	.enqueue = pixelpump_enqueue,
	.dequeue = pixelpump_dequeue,
};

#define DT_DRV_COMPAT tinyvision_pixelpump

#define PIXELPUMP_DEVICE_DEFINE(n)                                                                 \
                                                                                                   \
	struct pixelpump_conf pixelpump_conf_##n = {                                               \
		.base = DT_INST_REG_ADDR(n),                                                   \
	};                                                                                         \
                                                                                                   \
	struct pixelpump_data pixelpump_data_##n;                                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, pixelpump_init, NULL, &pixelpump_data_##n, &pixelpump_conf_##n,   \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                     \
			      &pixelpump_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PIXELPUMP_DEVICE_DEFINE)
