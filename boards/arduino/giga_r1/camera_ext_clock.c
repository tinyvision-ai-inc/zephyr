/*
 * Copyright 2024 Felipe Neves
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(camera_ext_clock, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

int camera_ext_clock_enable(void)
{
	const struct device *xclk_dev = DEVICE_DT_GET(DT_NODELABEL(dvp_20pin_xclk));
	uint32_t rate_hz;
	int ret;

	if (!device_is_ready(xclk_dev)) {
		LOG_ERR("Camera external clock source device is not ready!");
		return -ENODEV;
	}

	ret = clock_control_on(xclk_dev, (clock_control_subsys_t)0);
	if (ret < 0) {
		LOG_ERR("Failed to enable camera external clock error: (%d)", ret);
		return ret;
	}

	ret = clock_control_get_rate(xclk_dev, (clock_control_subsys_t)0, &rate_hz);
	if (ret < 0) {
		LOG_ERR("Failed to get camera external clock rate, error: (%d)", ret);
		return ret;
	}

	LOG_INF("Camera external clock rate: %u Hz", rate_hz);

	return 0;
}

SYS_INIT(camera_ext_clock_enable, POST_KERNEL, CONFIG_CLOCK_CONTROL_PWM_INIT_PRIORITY);
