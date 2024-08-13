/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2023-2024 tinyVision.ai
 *
 * Ad-Hoc driver for the Si5351a external PLL chip until it gets replaced in the Rev3 of the hardware.
 * https://www.latticesemi.com/products/designsoftwareandip/intellectualproperty/ipcore/ipcores05/usb-2_0-3_2-ip-core
 */

#define DT_DRV_COMPAT skyworks_si5351

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control.h>

struct si5351_config {
	struct i2c_dt_spec i2c;
};

static const struct {
	uint8_t addr;
	uint8_t data;
} si5351_conf[] = {
	/* Generated by SkyWorks tool "ClockBuilder Pro", to be replaced by a
	 * handcrafted register map in the future as this seems to have issues
	 * with register order */
	{0x02, 0x53},
	/*{ 0x03, 0xFF },*/
	{0x04, 0x20},
	{0x07, 0x00},
	{0x0F, 0x00},
	{0x10, 0x1F},
	{0x11, 0x0F},
	{0x12, 0x0F},
	{0x13, 0x8C},
	{0x14, 0x8C},
	{0x15, 0x8C},
	{0x16, 0x8C},
	{0x17, 0x8C},
	{0x18, 0x01},
	{0x1A, 0x00},
	{0x1B, 0x01},
	{0x1C, 0x00},
	{0x1D, 0x10},
	{0x1E, 0x00},
	{0x1F, 0x00},
	{0x2A, 0x00},
	{0x2B, 0x01},
	{0x2C, 0x00},
	{0x2D, 0x05},
	{0x2E, 0x80},
	{0x2F, 0x00},
	{0x30, 0x00},
	{0x31, 0x00},
	{0x32, 0x00},
	{0x33, 0x01},
	{0x34, 0x00},
	{0x35, 0x05},
	{0x36, 0x80},
	{0x37, 0x00},
	{0x38, 0x00},
	{0x39, 0x00},
	{0x3A, 0x00},
	{0x3B, 0x02},
	{0x3C, 0x00},
	{0x3D, 0x10},
	{0x3E, 0xC0},
	{0x3F, 0x00},
	{0x40, 0x00},
	{0x41, 0x00},
	{0x5A, 0x00},
	{0x5B, 0x00},
	{0x95, 0x00},
	{0x96, 0x00},
	{0x97, 0x00},
	{0x98, 0x00},
	{0x99, 0x00},
	{0x9A, 0x00},
	{0x9B, 0x00},
	{0xA2, 0x00},
	{0xA3, 0x00},
	{0xA4, 0x00},
	{0xA5, 0x00},
	{0xA6, 0x00},
	{0xA7, 0x00},
	{0xB7, 0x92},
};

static int si5351_init(const struct device *dev)
{
	const struct si5351_config *config = dev->config;
	int ret;

	/* Disable all clock outputs */
	ret = i2c_reg_write_byte_dt(&config->i2c, 3, 0xFF);
	if (ret) {
		return ret;
	}

	/* Power down all output drivers */
	for (int i = 16; i <= 23; i++) {
		ret = i2c_reg_write_byte_dt(&config->i2c, i, 0x80);
		if (ret) {
			return ret;
		}
	}

	for (size_t i = 0; ((i < ARRAY_SIZE(si5351_conf))); i++) {
		ret = i2c_reg_write_byte_dt(&config->i2c, si5351_conf[i].addr, si5351_conf[i].data);
		if (ret) {
			return ret;
		}
	}

	/* Apply PLLA/B reset */
	ret = i2c_reg_write_byte_dt(&config->i2c, 177, 0xAC);
	if (ret) {
		return ret;
	}

	/* Enable all clock outputs */
	ret = i2c_reg_write_byte_dt(&config->i2c, 3, 0x00);
	if (ret) {
		return ret;
	}

	/* Wait for an arbitrary time so the PLL can have time to lock and the clock to switch. */
	for (volatile int i = 0; i < 5000; i++) {
		continue;
	}

	return ret;
}

static int si5351_on(const struct device *dev, clock_control_subsys_t sys)
{
	return 0;
}

static int si5351_off(const struct device *dev, clock_control_subsys_t sys)
{
	return -ENOTSUP;
}

static enum clock_control_status si5351_get_status(const struct device *dev,
						   clock_control_subsys_t sys)
{
	/* Always turned on for this implementation */
	return CLOCK_CONTROL_STATUS_ON;
}

static int si5351_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	/* Always the same rate for this implementation */
	*rate = 0;
	return 0;
}

static struct clock_control_driver_api si5351_api = {
	.on = si5351_on,
	.off = si5351_off,
	.get_rate = si5351_get_rate,
	.get_status = si5351_get_status,
};

#define SI5351_DEVICE_DEFINE(n)                                                                    \
                                                                                                   \
	const struct si5351_config si5351_config_##n = {                                           \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, si5351_init, NULL, NULL, &si5351_config_##n, PRE_KERNEL_1,        \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &si5351_api);

DT_INST_FOREACH_STATUS_OKAY(SI5351_DEVICE_DEFINE)
