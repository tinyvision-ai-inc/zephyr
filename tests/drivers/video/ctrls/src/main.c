/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>

#include "video_ctrls.h"

ZTEST(video_ctrls, test_video_convert_ctrl_name)
{
	char dest[CONFIG_VIDEO_CTRL_NAME_MAX_SIZE];
	const char *examples[][2] = {
		{"White Balance, Automatic", "white_balance_automatic"},
		{"nothing exceptional", "nothing_exceptional"},
		{"UPPERCASE", "uppercase"},
		{"._!!_###_!!_.", ""},
		{"  _  leading special characters", "leading_special_characters"},
		{"trailing special characters!@#$%^", "trailing_special_characters"},
		{"single trailing space ", "single_trailing_space"},
		{"", ""},
	};

	for (int i = 0; i < ARRAY_SIZE(examples); i++) {
		memset(dest, 0xff, sizeof(dest));

		video_convert_ctrl_name(examples[i][0], dest, sizeof(dest));

		zassert_str_equal(dest, examples[i][1],
				  "'%s' should be converted to '%s' instead of '%s'",
				  examples[i][0], examples[i][1], dest);
	}
}

ZTEST_SUITE(video_ctrls, NULL, NULL, NULL, NULL, NULL);
