/*
 * Copyright (c) 2023 Gaurav Singh www.CircuitValley.com
 * Copyright 2024 NXP
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sony_imx219

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imx219, CONFIG_VIDEO_LOG_LEVEL);

/* Chip ID */
#define CHIP_ID_REG 0x0000
#define CHIP_ID_VAL 0x0219

#define IMX219_REG_SOFTWARE_RESET 0x0103
#define IMX219_SOFTWARE_RESET     1

/* mode_select */
#define IMX219_REG_MODE_SELECT 0x0100
#define IMX219_MODE_STANDBY    0x00
#define IMX219_MODE_STREAMING  0x01

#define IMX219_REG_CSI_LANE_MODE 0x0114
#define IMX219_CSI_2_LANE_MODE   0x01
#define IMX219_CSI_4_LANE_MODE   0x03

#define IMX219_REG_DPHY_CTRL           0x0128
#define IMX219_DPHY_CTRL_TIMING_AUTO   0
#define IMX219_DPHY_CTRL_TIMING_MANUAL 1

#define IMX219_REG_EXCK_FREQ     0x012a
#define IMX219_REG_EXCK_FREQ_LSB 0x012B

/* Analog gain control */
#define IMX219_REG_ANALOG_GAIN  0x0157
#define IMX219_ANALOG_GAIN_MAX  232

/* Digital gain control */
#define IMX219_REG_DIGITAL_GAIN_MSB 0x0158
#define IMX219_REG_DIGITAL_GAIN_LSB 0x0159
#define IMX219_DIGITAL_GAIN_MAX 0xfff

/* Exposure control */
#define IMX219_REG_INTEGRATION_TIME_MSB 0x015A
#define IMX219_REG_INTEGRATION_TIME_LSB 0x015B
#define IMX219_EXPOSURE_MAX             0x900

/* V_TIMING internal */
#define IMX219_REG_FRAME_LEN_MSB 0x0160
#define IMX219_REG_FRAME_LEN_LSB 0x0161

#define IMX219_REG_LINE_LENGTH_A_MSB 0x0162
#define IMX219_REG_LINE_LENGTH_A_LSB 0x0163
#define IMX219_REG_X_ADD_STA_A_MSB   0x0164
#define IMX219_REG_X_ADD_STA_A_LSB   0x0165
#define IMX219_REG_X_ADD_END_A_MSB   0x0166
#define IMX219_REG_X_ADD_END_A_LSB   0x0167
#define IMX219_REG_Y_ADD_STA_A_MSB   0x0168
#define IMX219_REG_Y_ADD_STA_A_LSB   0x0169
#define IMX219_REG_Y_ADD_END_A_MSB   0x016a
#define IMX219_REG_Y_ADD_END_A_LSB   0x016b
#define IMX219_REG_X_OUTPUT_SIZE_MSB 0x016c
#define IMX219_REG_X_OUTPUT_SIZE_LSB 0x016d
#define IMX219_REG_Y_OUTPUT_SIZE_MSB 0x016e
#define IMX219_REG_Y_OUTPUT_SIZE_LSB 0x016f
#define IMX219_REG_X_ODD_INC_A       0x0170
#define IMX219_REG_Y_ODD_INC_A       0x0171
#define IMX219_REG_ORIENTATION       0x0172

/* Binning  Mode */
#define IMX219_REG_BINNING_MODE_H 0x0174
#define IMX219_REG_BINNING_MODE_V 0x0175
#define IMX219_BINNING_NONE       0x00

#define IMX219_REG_CSI_DATA_FORMAT_A_MSB 0x018c
#define IMX219_REG_CSI_DATA_FORMAT_A_LSB 0x018d

/* PLL Settings */
#define IMX219_REG_VTPXCK_DIV      0x0301
#define IMX219_REG_VTSYCK_DIV      0x0303
#define IMX219_REG_PREPLLCK_VT_DIV 0x0304
#define IMX219_REG_PREPLLCK_OP_DIV 0x0305
#define IMX219_REG_PLL_VT_MPY_MSB  0x0306
#define IMX219_REG_PLL_VT_MPY_LSB  0x0307
#define IMX219_REG_OPPXCK_DIV      0x0309
#define IMX219_REG_OPSYCK_DIV      0x030b
#define IMX219_REG_PLL_OP_MPY_MSB  0x030c
#define IMX219_REG_PLL_OP_MPY_LSB  0x030d

/* Test Pattern Control */
#define IMX219_REG_TEST_PATTERN_MSB 0x0600
#define IMX219_REG_TEST_PATTERN_LSB 0x0601
#define IMX219_TEST_PATTERN_DISABLE 0

#define IMX219_REG_TP_X_OFFSET_MSB 0x0620
#define IMX219_REG_TP_X_OFFSET_LSB 0x0621
#define IMX219_REG_TP_Y_OFFSET_MSB 0x0622
#define IMX219_REG_TP_Y_OFFSET_LSB 0x0623

/* Test pattern colour components */
#define IMX219_REG_TESTP_RED_MSB    0x0602
#define IMX219_REG_TESTP_RED_LSB    0x0603
#define IMX219_REG_TESTP_GREENR_MSB 0x0604
#define IMX219_REG_TESTP_GREENR_LSB 0x0605
#define IMX219_REG_TESTP_BLUE_MSB   0x0606
#define IMX219_REG_TESTP_BLUE_LSB   0x0607

#define IMX219_REG_TP_WINDOW_WIDTH_MSB  0x0624
#define IMX219_REG_TP_WINDOW_WIDTH_LSB  0x0625
#define IMX219_REG_TP_WINDOW_HEIGHT_MSB 0x0626
#define IMX219_REG_TP_WINDOW_HEIGHT_LSB 0x0627

#define IMX219_RESOLUTION_PARAM_NUM 20

struct imx219_reg {
	uint16_t addr;
	uint8_t val;
};

struct imx219_resolution_config {
	uint16_t width;
	uint16_t height;
	const struct imx219_reg *res_params;
};

static const struct imx219_reg imx219_1080p_res_params[] = {
	{IMX219_REG_PLL_VT_MPY_MSB, 0x00},
	{IMX219_REG_PLL_VT_MPY_LSB, 0x20},
	{IMX219_REG_VTPXCK_DIV, 0x4},
	{IMX219_REG_INTEGRATION_TIME_MSB, 0x4},
	{IMX219_REG_INTEGRATION_TIME_LSB, 0xac},
	{IMX219_REG_ANALOG_GAIN, 0x80},
	{IMX219_REG_LINE_LENGTH_A_MSB, 0x0d},
	{IMX219_REG_LINE_LENGTH_A_LSB, 0x78},
	{IMX219_REG_FRAME_LEN_MSB, 0x4},
	{IMX219_REG_FRAME_LEN_LSB, 0xb0},
	{IMX219_REG_X_ADD_STA_A_MSB, 0x02},
	{IMX219_REG_X_ADD_STA_A_LSB, 0xa8},
	{IMX219_REG_Y_ADD_STA_A_MSB, 0x2},
	{IMX219_REG_Y_ADD_STA_A_LSB, 0xb4},
	{IMX219_REG_X_ADD_END_A_MSB, 0x0a},
	{IMX219_REG_X_ADD_END_A_LSB, 0x27},
	{IMX219_REG_Y_ADD_END_A_MSB, 0x6},
	{IMX219_REG_Y_ADD_END_A_LSB, 0xeb},
	{IMX219_REG_X_OUTPUT_SIZE_MSB, 0x7},
	{IMX219_REG_X_OUTPUT_SIZE_LSB, 0x80},
	{IMX219_REG_Y_OUTPUT_SIZE_MSB, 0x4},
	{IMX219_REG_Y_OUTPUT_SIZE_LSB, 0x38},
	{IMX219_REG_TEST_PATTERN_LSB, 0x0},
	{IMX219_REG_BINNING_MODE_H, 0x00},
	{IMX219_REG_BINNING_MODE_V, 0x00},
};

static const struct imx219_resolution_config res_params[] = {
	{.width = 1920, .height = 1080, .res_params = imx219_1080p_res_params},
};

#define IMX219_VIDEO_FORMAT_CAP(width, height, format)                                             \
	{                                                                                          \
		.pixelformat = (format), .width_min = (width), .width_max = (width),               \
		.height_min = (height), .height_max = (height), .width_step = 1, .height_step = 1  \
	}

static const struct video_format_cap fmts[] = {
	IMX219_VIDEO_FORMAT_CAP(1920, 1080, VIDEO_PIX_FMT_BGGR8),
	{0}};

static int imx219_read_reg(const struct i2c_dt_spec *spec, const uint16_t addr, void *val,
			   const uint8_t val_size)
{
	int ret;
	struct i2c_msg msg[2];
	uint8_t addr_buf[2];

	if (val_size > 4) {
		return -ENOTSUP;
	}

	addr_buf[1] = addr & 0xFF;
	addr_buf[0] = addr >> 8;
	msg[0].buf = addr_buf;
	msg[0].len = 2U;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)val;
	msg[1].len = val_size;
	msg[1].flags = I2C_MSG_READ | I2C_MSG_STOP | I2C_MSG_RESTART;

	ret = i2c_transfer_dt(spec, msg, 2);
	if (ret) {
		return ret;
	}

	switch (val_size) {
	case 4:
		*(uint32_t *)val = sys_be32_to_cpu(*(uint32_t *)val);
		break;
	case 2:
		*(uint16_t *)val = sys_be16_to_cpu(*(uint16_t *)val);
		break;
	case 1:
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int imx219_write_reg(const struct i2c_dt_spec *spec, const uint16_t addr, const uint8_t val)
{
	uint8_t addr_buf[2];
	struct i2c_msg msg[2];

	addr_buf[1] = addr & 0xFF;
	addr_buf[0] = addr >> 8;
	msg[0].buf = addr_buf;
	msg[0].len = 2U;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)&val;
	msg[1].len = 1;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer_dt(spec, msg, 2);
}

static int imx219_write_multi_regs(const struct i2c_dt_spec *spec, const struct imx219_reg *regs,
				   const uint32_t num_regs)
{
	int ret;

	for (int i = 0; i < num_regs; i++) {
		ret = imx219_write_reg(spec, regs[i].addr, regs[i].val);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

struct imx219_config {
	struct i2c_dt_spec i2c;
};

struct imx219_data {
	struct video_format fmt;
};

static const struct imx219_reg imx219InitParams[] = {
	{IMX219_REG_MODE_SELECT, IMX219_MODE_STANDBY},
	{0x30EB, 0x05}, // access sequence
	{0x30EB, 0x0C},
	{0x300A, 0xFF},
	{0x300B, 0xFF},
	{0x30EB, 0x05},
	{0x30EB, 0x09},
	{IMX219_REG_CSI_LANE_MODE, IMX219_CSI_2_LANE_MODE},   // 3-> 4Lane 1-> 2Lane
	{IMX219_REG_DPHY_CTRL, IMX219_DPHY_CTRL_TIMING_AUTO}, // DPHY timing 0-> auot 1-> manual
	{IMX219_REG_EXCK_FREQ, 0x18}, // external oscillator frequncy 0x18 -> 24Mhz
	{IMX219_REG_EXCK_FREQ_LSB, 0x00},
	{IMX219_REG_FRAME_LEN_MSB, 0x06}, // frame length , Raspberry pi sends this commands continously when recording video
		// @60fps ,writes come at interval of 32ms , Data 355 for resolution 1280x720
		// command 162 also comes along with data 0DE7 also 15A with data 0200
	{IMX219_REG_FRAME_LEN_LSB, 0xE3},
	{IMX219_REG_LINE_LENGTH_A_MSB, 0x0d}, // does not directly affect how many bits on wire in
					      // one line does affect how many clock between lines
	{IMX219_REG_LINE_LENGTH_A_LSB,
	 0x78}, // appears to be having step in value, not every LSb change will reflect on fps
	{IMX219_REG_X_ADD_STA_A_MSB, 0x02}, // x start
	{IMX219_REG_X_ADD_STA_A_LSB, 0xA8},
	{IMX219_REG_X_ADD_END_A_MSB, 0x0A}, // x end
	{IMX219_REG_X_ADD_END_A_LSB, 0x27},
	{IMX219_REG_Y_ADD_STA_A_MSB, 0x02}, // y start
	{IMX219_REG_Y_ADD_STA_A_LSB, 0xB4},
	{IMX219_REG_Y_ADD_END_A_MSB, 0x06}, // y end
	{IMX219_REG_Y_ADD_END_A_LSB, 0xEB},
	{IMX219_REG_X_OUTPUT_SIZE_MSB,
	 0x07}, // resolution 1280 -> 5 00 , 1920 -> 780 , 2048 -> 0x8 0x00
	{IMX219_REG_X_OUTPUT_SIZE_LSB, 0x80},
	{IMX219_REG_Y_OUTPUT_SIZE_MSB,
	 0x04}, // 720 -> 0x02D0 | 1080 -> 0x438  | this setting changes how many line over wire
		// does not affect frame rate
	{IMX219_REG_Y_OUTPUT_SIZE_LSB, 0x38},
	{IMX219_REG_X_ODD_INC_A, 0x01},           // increment
	{IMX219_REG_Y_ODD_INC_A, 0x01},           // increment
	{IMX219_REG_BINNING_MODE_H, 0x00},        // binning H 0 off 1 x2 2 x4 3 x2 analog
	{IMX219_REG_BINNING_MODE_V, 0x00},        // binning H 0 off 1 x2 2 x4 3 x2 analog
	{IMX219_REG_CSI_DATA_FORMAT_A_MSB, 0x0A}, // CSI Data format A-> 10bit
	{IMX219_REG_CSI_DATA_FORMAT_A_LSB, 0x0A}, // CSI Data format
	{IMX219_REG_VTPXCK_DIV, 0x05},            // vtpxclkd_div	5 301
	{IMX219_REG_VTSYCK_DIV, 0x01},            // vtsclk _div  1	303
	{IMX219_REG_PREPLLCK_VT_DIV, 0x03},       // external oscillator /3
	{IMX219_REG_PREPLLCK_OP_DIV, 0x03},       // external oscillator /3
	{IMX219_REG_PLL_VT_MPY_MSB, 0x00},        // PLL_VT multiplizer
	{IMX219_REG_PLL_VT_MPY_LSB, 0x52}, // Changes Frame rate with , integration register 0x15a
	{IMX219_REG_OPPXCK_DIV, 0x0A},     // oppxck_div
	{IMX219_REG_OPSYCK_DIV, 0x01},     // opsysck_div
	{IMX219_REG_PLL_OP_MPY_MSB, 0x00}, // PLL_OP
	{IMX219_REG_PLL_OP_MPY_LSB,
	 0x32},         // 8Mhz x 0x57 ->696Mhz -> 348Mhz |  0x32 -> 200Mhz | 0x40 -> 256Mhz
	{0x455E, 0x00}, // magic?
	{0x471E, 0x4B},
	{0x4767, 0x0F},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47B4, 0x14},
	{0x4713, 0x30},
	{0x478B, 0x10},
	{0x478F, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0E},
	{0x479B, 0x0E},
	{IMX219_REG_TESTP_RED_MSB, 0x00},
	{IMX219_REG_TESTP_RED_LSB, 0x00},
	{IMX219_REG_TESTP_GREENR_MSB, 0x00},
	{IMX219_REG_TESTP_GREENR_LSB, 0x00},
	{IMX219_REG_TESTP_BLUE_MSB, 0x00},
	{IMX219_REG_TESTP_BLUE_LSB, 0x00},
	{IMX219_REG_TEST_PATTERN_MSB, 0x00}, // test pattern
	{IMX219_REG_TEST_PATTERN_LSB, 0x00},
	{IMX219_REG_TP_X_OFFSET_MSB, 0x00}, // tp offset x 0
	{IMX219_REG_TP_X_OFFSET_LSB, 0x00},
	{IMX219_REG_TP_Y_OFFSET_MSB, 0x00}, // tp offset y 0
	{IMX219_REG_TP_Y_OFFSET_LSB, 0x00},
	{IMX219_REG_TP_WINDOW_WIDTH_MSB, 0x05}, // TP width 1920 ->780 1280->500
	{IMX219_REG_TP_WINDOW_WIDTH_LSB, 0x00},
	{IMX219_REG_TP_WINDOW_HEIGHT_MSB, 0x02}, // TP height 1080 -> 438 720->2D0
	{IMX219_REG_TP_WINDOW_HEIGHT_LSB, 0xD0},
	{IMX219_REG_DIGITAL_GAIN_MSB, 0x01},
	{IMX219_REG_DIGITAL_GAIN_LSB, 0x00},
	{IMX219_REG_ANALOG_GAIN,
	 0x80}, // analog gain , raspberry pi constinouly changes this depending on scense
	{IMX219_REG_INTEGRATION_TIME_MSB,
	 0x03}, // integration time , really important for frame rate
	{IMX219_REG_INTEGRATION_TIME_LSB, 0x51},
	{IMX219_REG_ORIENTATION,
	 0x03}, // image_orientation (for both direction) bit[0]: hor bit[1]: vert
};

static int imx219_get_ctrl_exposure(const struct device *dev, uint32_t op, uint32_t *value)
{
	const struct imx219_config *cfg = dev->config;
	uint16_t u16;
	int ret;

	switch (op) {
	case VIDEO_GET_CUR:
		ret = imx219_read_reg(&cfg->i2c, IMX219_REG_INTEGRATION_TIME_LSB, &u16, 2);
		*value = u16;
		return ret;
	case VIDEO_GET_MIN:
		*(uint32_t *)value = 0;
		return 0;
	case VIDEO_GET_MAX:
		*(uint32_t *)value = IMX219_EXPOSURE_MAX;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx219_set_ctrl_exposure(const struct device *dev, uint32_t value)
{
	const struct imx219_config *cfg = dev->config;
	int err;

	value = MIN(value, IMX219_EXPOSURE_MAX);

	err = imx219_write_reg(&cfg->i2c, IMX219_REG_INTEGRATION_TIME_MSB, value >> 8);
	if (err) {
		return err;
	}
	err = imx219_write_reg(&cfg->i2c, IMX219_REG_INTEGRATION_TIME_LSB, value & 0xff);
	if (err) {
		return err;
	}

	return 0;
}

static int imx219_get_ctrl_gain(const struct device *dev, uint32_t op, uint32_t *value)
{
	const struct imx219_config *cfg = dev->config;
	uint16_t analog_gain;
	uint16_t digital_gain;
	int err;

	switch (op) {
	case VIDEO_GET_CUR:
		err = imx219_read_reg(&cfg->i2c, IMX219_REG_ANALOG_GAIN, &analog_gain, 1);
		if (err) {
			return err;
		}
		err = imx219_read_reg(&cfg->i2c, IMX219_REG_DIGITAL_GAIN_MSB, &digital_gain, 2);
		if (err) {
			return err;
		}
		*value = analog_gain + digital_gain;
		return 0;
	case VIDEO_GET_MIN:
		*value = 0;
		return 0;
	case VIDEO_GET_MAX:
		*value = IMX219_ANALOG_GAIN_MAX + IMX219_DIGITAL_GAIN_MAX;
		return 0;
	default:
		LOG_DBG("Invalid CID operation for gain query");
		return -EINVAL;
	}
}

static int imx219_set_ctrl_gain(const struct device *dev, uint32_t value)
{
	const struct imx219_config *cfg = dev->config;
	uint16_t digital_gain;
	uint8_t analog_gain;
	int err;

	if (value > IMX219_ANALOG_GAIN_MAX + IMX219_DIGITAL_GAIN_MAX) {
		LOG_ERR("Gain %u above the maximum", value);
		return -ERANGE;
	}

	if (value <= IMX219_ANALOG_GAIN_MAX) {
		analog_gain = value;
		digital_gain = 0;
	} else {
		analog_gain = IMX219_ANALOG_GAIN_MAX;
		digital_gain = value - IMX219_ANALOG_GAIN_MAX;
	}

	err = imx219_write_reg(&cfg->i2c, IMX219_REG_ANALOG_GAIN, analog_gain);
	if (err) {
		return err;
	}
	err = imx219_write_reg(&cfg->i2c, IMX219_REG_DIGITAL_GAIN_MSB, digital_gain >> 8);
	if (err) {
		return err;
	}
	err = imx219_write_reg(&cfg->i2c, IMX219_REG_DIGITAL_GAIN_LSB, digital_gain & 0xff);
	if (err) {
		return err;
	}

	return 0;
}

static int imx219_get_fmt(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	struct imx219_data *drv_data = dev->data;

	*fmt = drv_data->fmt;
	return 0;
}

static int imx219_set_fmt(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	struct imx219_data *drv_data = dev->data;
	const struct imx219_config *cfg = dev->config;
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(fmts); ++i) {
		if (fmt->pixelformat == fmts[i].pixelformat && fmt->width >= fmts[i].width_min &&
		    fmt->width <= fmts[i].width_max && fmt->height >= fmts[i].height_min &&
		    fmt->height <= fmts[i].height_max) {
			break;
		}
	}

	if (i == ARRAY_SIZE(fmts)) {
		LOG_ERR("Unsupported pixel format or resolution");
		return -ENOTSUP;
	}

	if (!memcmp(&drv_data->fmt, fmt, sizeof(drv_data->fmt))) {
		return 0;
	}

	drv_data->fmt = *fmt;

	/* Set resolution parameters */
	for (i = 0; i < ARRAY_SIZE(res_params); i++) {
		if (fmt->width == res_params[i].width && fmt->height == res_params[i].height) {
			ret = imx219_write_multi_regs(&cfg->i2c, res_params[i].res_params,
						      IMX219_RESOLUTION_PARAM_NUM);
			if (ret) {
				LOG_ERR("Unable to set resolution parameters");
				return ret;
			}
			break;
		}
	}

	LOG_INF("Setting IMX219 format to %ux%u", fmt->width, fmt->height);
	return 0;
}

static int imx219_stream_start(const struct device *dev)
{
	const struct imx219_config *cfg = dev->config;

	return imx219_write_reg(&cfg->i2c, IMX219_REG_MODE_SELECT, IMX219_MODE_STREAMING);
}

static int imx219_stream_stop(const struct device *dev)
{
	const struct imx219_config *cfg = dev->config;

	return imx219_write_reg(&cfg->i2c, IMX219_REG_MODE_SELECT, IMX219_MODE_STANDBY);
}

static int imx219_get_caps(const struct device *dev, enum video_endpoint_id ep,
			   struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static int imx219_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	switch (cid) {
	case VIDEO_CID_CAMERA_EXPOSURE:
		return imx219_set_ctrl_exposure(dev, (uint32_t)value);
	case VIDEO_CID_CAMERA_GAIN:
		return imx219_set_ctrl_gain(dev, (uint32_t)value);
	default:
		LOG_WRN("Unsupported control 0x%08x", cid);
		return -ENOTSUP;
	}
}

static int imx219_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	switch (cid & ~VIDEO_GET_MASK) {
	case VIDEO_CID_CAMERA_EXPOSURE:
		return imx219_get_ctrl_exposure(dev, cid & VIDEO_GET_MASK, value);
	case VIDEO_CID_CAMERA_GAIN:
		return imx219_get_ctrl_gain(dev, cid & VIDEO_GET_MASK, value);
	default:
		LOG_WRN("Unsupported control 0x%08x", cid);
		return -ENOTSUP;
	}
}

static const struct video_driver_api imx219_driver_api = {
	.set_format = imx219_set_fmt,
	.get_format = imx219_get_fmt,
	.get_caps = imx219_get_caps,
	.stream_start = imx219_stream_start,
	.stream_stop = imx219_stream_stop,
	.set_ctrl = imx219_set_ctrl,
	.get_ctrl = imx219_get_ctrl,
};

static int imx219_init(const struct device *dev)
{
	const struct imx219_config *cfg = dev->config;
	struct video_format fmt;
	uint16_t chip_id;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	k_sleep(K_MSEC(10));

	/* Software reset */
	ret = imx219_write_reg(&cfg->i2c, IMX219_REG_SOFTWARE_RESET, IMX219_SOFTWARE_RESET);
	if (ret) {
		LOG_ERR("Unable to perform software reset");
		return -EIO;
	}

	k_sleep(K_MSEC(5));

	/* Check sensor chip id */
	ret = imx219_read_reg(&cfg->i2c, CHIP_ID_REG, &chip_id, sizeof(chip_id));
	if (ret) {
		LOG_ERR("Unable to read sensor chip ID, ret = %d", ret);
		return -ENODEV;
	}

	if (chip_id != CHIP_ID_VAL) {
		LOG_ERR("Wrong chip ID: %04x (expected %04x)", chip_id, CHIP_ID_VAL);
		return -ENODEV;
	}

	/* Initialize register values */
	ret = imx219_write_multi_regs(&cfg->i2c, imx219InitParams, ARRAY_SIZE(imx219InitParams));
	if (ret) {
		LOG_ERR("Unable to initialize the sensor");
		return -EIO;
	}

	/* Set default format to 1080p BGGR8 */
	fmt.pixelformat = VIDEO_PIX_FMT_BGGR8;
	fmt.width = 1920;
	fmt.height = 1080;
	fmt.pitch = fmt.width * 2;
	ret = imx219_set_fmt(dev, VIDEO_EP_OUT, &fmt);
	if (ret) {
		LOG_ERR("Unable to configure default format");
		return -EIO;
	}

	return 0;
}

#define IMX219_INIT(n)                                                                             \
	static struct imx219_data imx219_data_##n;                                                 \
                                                                                                   \
	static const struct imx219_config imx219_cfg_##n = {                                       \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &imx219_init, NULL, &imx219_data_##n, &imx219_cfg_##n,            \
			      POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &imx219_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IMX219_INIT)
