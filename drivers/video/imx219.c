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
LOG_MODULE_REGISTER(imx219, CONFIG_VIDEO_LOG_LEVEL);

#define IMX219_I2C_ADDR 0x10
#define IMX219_ID       0x0219

#define IMX219_MODEL_ID_MSB  0x0000
#define IMX219_MODEL_ID_LSB  0x0001
#define IMX219_MODE_SEL      0x0100
#define IMX219_SW_RESET      0x0103
#define IMX219_CSI_LANE      0x0114
#define IMX219_DPHY_CTRL     0x0128
#define IMX219_EXCK_FREQ_MSB 0x012A
#define IMX219_EXCK_FREQ_LSB 0x012B
#define IMX219_FRAME_LENGTH_MSB 0x0160
#define IMX219_FRAME_LENGTH_LSB 0x0161
#define IMX219_LINE_LENGTH_MSB  0x0162
#define IMX219_LINE_LENGTH_LSB  0x0163
#define IMX219_X_ADD_START_MSB 0x0164
#define IMX219_X_ADD_START_LSB 0x0165
#define IMX219_X_ADD_END_MSB 0x0166
#define IMX219_X_ADD_END_LSB 0x0167
#define IMX219_Y_ADD_START_MSB 0x0168
#define IMX219_Y_ADD_START_LSB 0x0169
#define IMX219_Y_ADD_END_MSB 0x016A
#define IMX219_Y_ADD_END_LSB 0x016B

#define IMX219_X_OUTPUT_SIZE_MSB 0x016C
#define IMX219_X_OUTPUT_SIZE_LSB 0x016D
#define IMX219_Y_OUTPUT_SIZE_MSB 0x016E
#define IMX219_Y_OUTPUT_SIZE_LSB 0x016F

#define IMX219_X_ODD_INC      0x0170
#define IMX219_Y_ODD_INC      0x0171
#define IMX219_IMG_ORIENT     0x0172
#define IMX219_BINNING_H      0x0174
#define IMX219_BINNING_V      0x0175
#define IMX219_BIN_CALC_MOD_H 0x0176
#define IMX219_BIN_CALC_MOD_V 0x0177

#define IMX219_CSI_FORMAT_C 0x018C
#define IMX219_CSI_FORMAT_D 0x018D

#define IMX219_DIG_GAIN_GLOBAL_MSB  0x0158
#define IMX219_DIG_GAIN_GLOBAL_LSB  0x0159
#define IMX219_ANA_GAIN_GLOBAL      0x0157
#define IMX219_INTEGRATION_TIME_MSB 0x015A
#define IMX219_INTEGRATION_TIME_LSB 0x015B
#define IMX219_ANALOG_GAIN          0x0157

#define IMX219_VTPXCK_DIV      0x0301
#define IMX219_VTSYCK_DIV      0x0303
#define IMX219_PREPLLCK_VT_DIV 0x0304
#define IMX219_PREPLLCK_OP_DIV 0x0305
#define IMX219_PLL_VT_MPY_MSB  0x0306
#define IMX219_PLL_VT_MPY_LSB  0x0307
#define IMX219_OPPXCK_DIV      0x0309
#define IMX219_OPSYCK_DIV      0x030B
#define IMX219_PLL_OP_MPY_MSB  0x030C
#define IMX219_PLL_OP_MPY_LSB  0x030D

#define IMX219_TEST_PATTERN_MSB 0x0600
#define IMX219_TEST_PATTERN_LSB 0x0601
#define IMX219_TP_RED_MSB       0x0602
#define IMX219_TP_RED_LSB       0x0603
#define IMX219_TP_GREEN_MSB     0x0604
#define IMX219_TP_GREEN_LSB     0x0605
#define IMX219_TP_BLUE_MSB      0x0606
#define IMX219_TP_BLUE_LSB      0x0607
#define IMX219_TP_X_OFFSET_MSB  0x0620
#define IMX219_TP_X_OFFSET_LSB  0x0621
#define IMX219_TP_Y_OFFSET_MSB  0x0622
#define IMX219_TP_Y_OFFSET_LSB  0x0623
#define IMX219_TP_WIDTH_MSB     0x0624
#define IMX219_TP_WIDTH_LSB     0x0625
#define IMX219_TP_HEIGHT_MSB    0x0626
#define IMX219_TP_HEIGHT_LSB    0x0627

enum imx219_mode {
	IMX219_MODE_INIT,
	IMX219_MODE_PREVIEW,
	IMX219_MODE_CAPTURE,
	IMX219_MODE_VIDEO,
	IMX219_MODE_HIGH_SPEED_VIDEO,
	IMX219_MODE_SLIM_VIDEO,
};

enum {
	IMAGE_NORMAL = 0,
};

struct imx219_conf {
	struct i2c_dt_spec spec;
};

struct imx219_params {
	uint8_t mirror;

	uint16_t pix_clk_mul;
	uint16_t pix_clk_div;
	uint16_t integration;
	uint16_t gain;
	uint16_t gain_max;

	uint16_t linelength;
	uint16_t framelength;
	uint16_t startx;
	uint16_t starty;
	uint16_t endx;
	uint16_t endy;

	uint16_t width;
	uint16_t height;
	uint16_t framerate;
	uint16_t fps;
	uint8_t test_pattern;
};

/* Only supports 2 lanes */
#define IMX219_MODE0_WIDTH   640
#define IMX219_MODE0_HEIGHT  78
#define IMX219_MODE0_FPS     1000
#define IMX219_MODE1_WIDTH   640
#define IMX219_MODE1_HEIGHT  480
#define IMX219_MODE1_FPS     200
#define IMX219_MODE2_WIDTH   1280
#define IMX219_MODE2_HEIGHT  720
#define IMX219_MODE2_FPS     60
#define IMX219_MODE3_WIDTH   1920
#define IMX219_MODE3_HEIGHT  1080
#define IMX219_MODE3_FPS     30
#define IMX219_MODE4_WIDTH   3280
#define IMX219_MODE4_HEIGHT  2464
#define IMX219_MODE4_FPS_MIN 5
#define IMX219_MODE4_FPS     15

#define HI8(x) (((x) >> 8) & 0xff)
#define LO8(x) (((x) >> 0) & 0xff)

static const struct {
	uint16_t addr;
	uint8_t val;
} imx219_registers[] = {
#define REG32(name, val) { name##_MSB, HI8(val) }, { name##_LSB, LO8(val) }

	/* default register settings, Resolution and FPS specific settings will
	 * be over written */
	{ IMX219_MODE_SEL, 0x00 },

	/* access sequence */
	{ 0x30EB, 0x05 },
	{ 0x30EB, 0x0C },
	{ 0x300A, 0xFF },
	{ 0x300B, 0xFF },
	{ 0x30EB, 0x05 },
	{ 0x30EB, 0x09 },

	/*
	 * 3: 4 Lanes,
	 * 1: 2 Lanes
	 */
	{ IMX219_CSI_LANE, 0x01 },

	/* DPHY timing: 0 for auto, 1 for manual */
	{ IMX219_DPHY_CTRL, 0x00 },

	/* external oscillator frequncy in MHz */
	REG32(IMX219_EXCK_FREQ, 24),

	/*
	 * Frame length. Raspberry Pi sends this commands continously when
	 * recording video @60fps, writes come at interval of 32ms:
	 * - data 0x355 for resolution 1280x720
	 * - command 0x162 also comes along with data 0x0DE7
	 * - also 0x15A with data 0x0200
	 */
	REG32(IMX219_FRAME_LENGTH, 1763),

	/*
	 * does not directly affect how many bits on wire in one line does
	 * affect how many clock between lines
	 * appears to be having step in value, not every LSb change will reflect on FPS
	 */
	REG32(IMX219_LINE_LENGTH, 3448),

	REG32(IMX219_X_ADD_START, 680),
	REG32(IMX219_X_ADD_END, 2599),
	REG32(IMX219_Y_ADD_START, 692),
	REG32(IMX219_Y_ADD_END, 1771),
	REG32(IMX219_X_OUTPUT_SIZE, 1920),

	/* this setting changes how many line over wire, does not affect frame rate */
	REG32(IMX219_Y_OUTPUT_SIZE, 1080),

	{ IMX219_X_ODD_INC, 0x01 },
	{ IMX219_Y_ODD_INC, 0x01 },

	{ IMX219_BINNING_H, 0x00 },
	{ IMX219_BINNING_V, 0x00 },

	{ IMX219_CSI_FORMAT_C, 0x0A },
	{ IMX219_CSI_FORMAT_D, 0x0A },
	{ IMX219_VTPXCK_DIV, 0x05 },
	{ IMX219_VTSYCK_DIV, 0x01 },

	/* external oscillator /3 */
	{ IMX219_PREPLLCK_VT_DIV, 0x03 },
	{ IMX219_PREPLLCK_OP_DIV, 0x03 },

	/* PLL_VT multiplizer. Changes Frame rate with integration register 0x15a */
	REG32(IMX219_PLL_VT_MPY, 82),

	{ IMX219_OPPXCK_DIV, 0x0A },
	{ IMX219_OPSYCK_DIV, 0x01 },

	/* PLL_OP multiplier, 200 MHz */
	REG32(IMX219_PLL_OP_MPY, 50),

	/* Unknown */
	{ 0x455E, 0x00 },
	{ 0x471E, 0x4B },
	{ 0x4767, 0x0F },
	{ 0x4750, 0x14 },
	{ 0x4540, 0x00 },
	{ 0x47B4, 0x14 },
	{ 0x4713, 0x30 },
	{ 0x478B, 0x10 },
	{ 0x478F, 0x10 },
	{ 0x4793, 0x10 },
	{ 0x4797, 0x0E },
	{ 0x479B, 0x0E },

	REG32(IMX219_TP_RED, 0),
	REG32(IMX219_TP_GREEN, 0),
	REG32(IMX219_TP_BLUE, 0),
	REG32(IMX219_TEST_PATTERN, 0),
	REG32(IMX219_TP_X_OFFSET, 0),
	REG32(IMX219_TP_Y_OFFSET, 0),
	REG32(IMX219_TP_Y_OFFSET, 0),
	REG32(IMX219_TP_WIDTH, 1280),
	REG32(IMX219_TP_HEIGHT, 720),
	REG32(IMX219_DIG_GAIN_GLOBAL, 256),

	// Analog gain, raspberry pi constinouly changes this depending on scense
	{ IMX219_ANA_GAIN_GLOBAL, 0x80 },

	// Integration time, really important for frame rate
	REG32(IMX219_INTEGRATION_TIME, 849),

	// Orientation: bit 0 for horizontal, bit 1 for vertical
	{ IMX219_IMG_ORIENT, 0x03 },

	{ IMX219_MODE_SEL, 0x01 },
};

static const struct imx219_params imx219_2_lanes[] = { 
	{
		.pix_clk_mul = 46,
		// only 4 or 5 or 8 or 10
		.pix_clk_div = 4,
		// must be < (linelength - 4) to maintain framerate by framelength
		// or integration time will slow frame rate
		.integration = 258 - 4,
		.gain = 0x70,
		.gain_max = 0xFF,
		// Warning! This value need to be either 0xD78 or 0xDE7 regardless of
		// frame size and FPS, other values will result undefined and
		// ununderstanable issues in image
		.linelength = 3448,
		// decided how long is frame, basically frame rate with pix clock, it has
		// second priority to integration time. absolute minimum is 255 for imx219
		.framelength = 258,
		.startx = 1000,
		.starty = 750,
		.endx = 2280,
		// this has to odd or bayer oder will change
		.endy = 1715,
		.width = 640,
		// each frame will have two extra line to compensate for debayer crop
		.height = 480,
		.fps = 200,
		.test_pattern = 0
	},
	{
		.pix_clk_mul = 0x2E,
		.pix_clk_div = 0x4,
		.integration = 862 - 4,
		.gain = 0x80,
		.gain_max = 0xFF,
		.linelength = 0xD78,
		.framelength = 862,
		.startx = 0x2A8,
		.starty = 0x2B4,
		.endx = 0xA27,
		.endy = 0x6EB,
		.width = 1280,
		.height = 720,
		.fps = 60,
		.test_pattern = 0
	},
	{
		.pix_clk_mul = 0x20,
		.pix_clk_div = 0x4,
		.integration = 1200 - 4,
		.gain = 0x80,
		.gain_max = 0xFF,
		.linelength = 0xD78,
		.framelength = 1200,
		.startx = 0x2A8,
		.starty = 0x2B4,
		.endx = 0xA27,
		.endy = 0x6EB,
		.width = 1920,
		.height = 1080,
		//.binning = 0,
		.fps = 30,
		.test_pattern = 0
	},
	{
		.pix_clk_mul = 0x2D,
		.pix_clk_div = 0x4,
		.integration = 56 - 4,
		.gain = 200,
		.gain_max = 0xFF,
		.linelength = 0xD78,
		.framelength = 56,
		.startx = 1320,
		.starty = 990,
		.endx = 2600,
		.endy = 1561,
		.width = 640,
		.height = 80,
		//.binning = 2,
		.fps = 900,
		.test_pattern = 0
	},
	{
		.pix_clk_mul = 0x12,
		.pix_clk_div = 0x4,
		.integration = 2670 - 4,
		.gain = 200,
		.gain_max = 0xFF,
		.linelength = 3448,
		.framelength = 2670,
		.startx = 0,
		.starty = 0,
		.endx = 3279,
		.endy = 2463,
		.width = 3280,
		.height = 2464,
		//.binning = 0,
		.fps = 5,
		.test_pattern = 0
	},
	{
		.pix_clk_mul = 0x12,
		.pix_clk_div = 0x4,
		.integration = 2670 - 4,
		.gain = 200,
		.gain_max = 0xFF,
		.linelength = 3448,
		.framelength = 2670,
		.startx = 0,
		.starty = 0,
		.endx = 3279,
		.endy = 2463,
		.width = 3280,
		.height = 2464,
		//.binning = 0,
		.fps = 7,
		.test_pattern = 0
	},
};

static const struct imx219_params *params = &imx219_2_lanes[0];

static int imx219_read_u8(const struct device *dev, uint16_t addr, uint8_t *val)
{
	const struct imx219_conf *conf = dev->config;
	uint8_t buf[2] = { HI8(addr), LO8(addr) };

	return i2c_write_read_dt(&conf->spec, buf, sizeof(buf), val, 1);
}

static int imx219_write_u8(const struct device *dev, uint16_t addr, uint8_t val)
{
	const struct imx219_conf *conf = dev->config;
	uint8_t buf[] = { HI8(addr), LO8(addr), val };

	return i2c_write_dt(&conf->spec, buf, sizeof(buf));
}

static int imx219_read_u16(const struct device *dev, uint16_t addr, uint16_t *val)
{
	uint8_t hi8, lo8;
	int err = 0;

	err |= imx219_read_u8(dev, addr + 0, &hi8);
	err |= imx219_read_u8(dev, addr + 1, &lo8);
	*val = (hi8 << 8) | (lo8 << 0);
	return err ? -EIO : 0;
}

static int imx219_write_u16(const struct device *dev, uint16_t addr, uint16_t val)
{
	int err = 0;

	err |= imx219_write_u8(dev, addr + 0, HI8(val));
	err |= imx219_write_u8(dev, addr + 1, HI8(val));
	return err ? -EIO : 0;
}

static int imx219_init(const struct device *dev)
{
	uint16_t u16 = 0;
	int err;

	err = imx219_read_u16(dev, IMX219_MODEL_ID_MSB, &u16);
	if (err != 0) {
		LOG_ERR("cannot read the model ID register");
		return err;
	}
	if (u16 != 0x0219) {
		LOG_ERR("invalid model ID register: 0x%04x", u16);
		return -EIO;
	}

	/* Reset the sensor */
	imx219_write_u8(dev, IMX219_SW_RESET, 1);
	k_sleep(K_USEC(10));
	imx219_write_u8(dev, IMX219_SW_RESET, 0);
	k_sleep(K_USEC(100));

	err |= imx219_write_u16(dev, IMX219_LINE_LENGTH_MSB, 3448);

	for (uint16_t i = 0; i < ARRAY_SIZE(imx219_registers); i++)
		imx219_write_u8(dev, imx219_registers[i].addr, imx219_registers[i].val);

	err |= imx219_write_u8(dev, IMX219_PLL_VT_MPY_MSB, HI8(params->pix_clk_mul));
	err |= imx219_write_u8(dev, IMX219_PLL_VT_MPY_LSB, LO8(params->pix_clk_mul));

	err |= imx219_write_u8(dev, IMX219_VTPXCK_DIV, LO8(params->pix_clk_div));

	err |= imx219_write_u8(dev, IMX219_INTEGRATION_TIME_MSB, HI8(params->integration));
	err |= imx219_write_u8(dev, IMX219_INTEGRATION_TIME_LSB, LO8(params->integration));

	err |= imx219_write_u8(dev, IMX219_FRAME_LENGTH_MSB, HI8(params->framelength));
	err |= imx219_write_u8(dev, IMX219_FRAME_LENGTH_LSB, LO8(params->framelength));

	err |= imx219_write_u8(dev, IMX219_X_ADD_START_MSB, HI8(params->startx));
	err |= imx219_write_u8(dev, IMX219_X_ADD_START_LSB, LO8(params->startx));

	err |= imx219_write_u8(dev, IMX219_Y_ADD_START_MSB, HI8(params->starty));
	err |= imx219_write_u8(dev, IMX219_Y_ADD_START_LSB, LO8(params->starty));

	err |= imx219_write_u8(dev, IMX219_X_ADD_END_MSB, HI8(params->endx));
	err |= imx219_write_u8(dev, IMX219_X_ADD_END_LSB, LO8(params->endx));

	err |= imx219_write_u8(dev, IMX219_Y_ADD_END_MSB, HI8(params->endy));
	err |= imx219_write_u8(dev, IMX219_Y_ADD_END_LSB, LO8(params->endy));

	err |= imx219_write_u8(dev, IMX219_X_OUTPUT_SIZE_MSB, HI8(params->width));
	err |= imx219_write_u8(dev, IMX219_X_OUTPUT_SIZE_LSB, LO8(params->width));

	err |= imx219_write_u8(dev, IMX219_Y_OUTPUT_SIZE_MSB, HI8(params->height));
	err |= imx219_write_u8(dev, IMX219_Y_OUTPUT_SIZE_LSB, LO8(params->height));

	err |= imx219_write_u8(dev, IMX219_TEST_PATTERN_LSB, (params->test_pattern < 8) ? params->test_pattern : 0);

	err |= imx219_write_u8(dev, IMX219_TP_WIDTH_MSB, HI8(params->width));
	err |= imx219_write_u8(dev, IMX219_TP_WIDTH_LSB, LO8(params->width));
	err |= imx219_write_u8(dev, IMX219_TP_HEIGHT_MSB, HI8(params->height));
	err |= imx219_write_u8(dev, IMX219_TP_HEIGHT_LSB, LO8(params->height));

	return err ? -EIO : 0;
}

static int imx219_stream_start(const struct video_dt_spec *spec)
{
	LOG_DBG("%s", __func__);
	return imx219_write_u8(spec->dev, IMX219_MODE_SEL, 1);
}

static int imx219_stream_stop(const struct video_dt_spec *spec)
{
	return imx219_write_u8(spec->dev, IMX219_MODE_SEL, 0);
}

static int imx219_set_ctrl(const struct video_dt_spec *spec, unsigned int cid, void *p)
{
	const struct device *dev = spec->dev;
	int32_t i32 = *(int32_t *)p;
	int err = 0;

	imx219_stream_stop(spec);

	switch (cid) {
	case VIDEO_CID_CAMERA_EXPOSURE:
		err = imx219_write_u16(dev, IMX219_INTEGRATION_TIME_MSB, i32);
		break;
	case VIDEO_CID_CAMERA_GAIN:
		err = imx219_write_u8(dev, IMX219_ANALOG_GAIN, i32);
		break;
	case VIDEO_CID_CAMERA_COLORBAR:
		err = imx219_write_u8(dev, IMX219_TEST_PATTERN_LSB, i32);
		break;
	case VIDEO_CID_CAMERA_BINNING:
		err |= imx219_write_u8(dev, IMX219_BINNING_H, i32 ? 0x03 : 0x00);
		err |= imx219_write_u8(dev, IMX219_BINNING_V, i32 ? 0x03 : 0x00);
		break;
	default:
		return -ENOTSUP;
	}

	imx219_stream_start(spec);

	return err ? -EIO : 0;
}

static int imx219_get_ctrl(const struct video_dt_spec *spec, unsigned int cid, void *p)
{
	const struct device *dev = spec->dev;
	int32_t *i32p = p;
	uint16_t u16;
	uint8_t u8;
	int err = 0;

	switch (cid) {
	case VIDEO_CID_CAMERA_EXPOSURE:
		err = imx219_read_u16(dev, IMX219_INTEGRATION_TIME_MSB, &u16);
		*i32p = u16;
		break;
	case VIDEO_CID_CAMERA_GAIN:
		err = imx219_read_u8(dev, IMX219_ANALOG_GAIN, &u8);
		*i32p = u8;
		break;
	case VIDEO_CID_CAMERA_COLORBAR:
		err = imx219_read_u8(dev, IMX219_TEST_PATTERN_LSB, &u8);
		*i32p = u8;
		break;
	case VIDEO_CID_CAMERA_BINNING:
		err |= imx219_read_u8(dev, IMX219_BINNING_H, &u8);
		*i32p = !!u8;
		break;
	default:
		return -ENOTSUP;
	}

	return err ? -EIO : 0;
}

static const struct video_format_cap fmts[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_GBRG8,
		.width_min = 640,
		.width_max = 3280,
		.height_min = 480,
		.height_max = 2464,
		.width_step = 1,
		.height_step = 1,
	},

	{ 0 }
};

static int imx219_get_caps(const struct video_dt_spec *spec,
			   struct video_caps *caps)
{
	caps->format_caps = fmts;
	return -ENOTSUP;
}

static int imx219_set_format(const struct video_dt_spec *spec,
			     struct video_format *format)
{
	return -ENOTSUP;
}

static int imx219_get_format(const struct video_dt_spec *spec,
			     struct video_format *format)
{
	return -ENOTSUP;
}

static const struct video_driver_api imx219_driver_api = {
	.set_format = imx219_set_format,
	.get_format = imx219_get_format,
	.set_ctrl = imx219_set_ctrl,
	.get_ctrl = imx219_get_ctrl,
	.get_caps = imx219_get_caps,
	.stream_start = imx219_stream_start,
	.stream_stop = imx219_stream_stop,
};

#define DT_DRV_COMPAT sony_imx219

#define IMX219_DEVICE_DEFINE(n)							\
										\
	const struct imx219_conf imx219_conf_##n = {				\
		.spec = I2C_DT_SPEC_INST_GET(n),				\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, imx219_init, NULL, NULL, &imx219_conf_##n,	\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &imx219_driver_api);

DT_INST_FOREACH_STATUS_OKAY(IMX219_DEVICE_DEFINE)
