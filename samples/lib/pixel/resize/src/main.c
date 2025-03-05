/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zephyr/pixel/resize.h>
#include <zephyr/pixel/print.h>

void gradient(uint8_t *rgb24buf, size_t size, uint8_t beg[3], uint8_t end[3])
{
	for (int i = 0; i + 3 <= size; i += 3) {
		rgb24buf[i + 0] = (beg[0] * (size - i) + end[0] * i) / size;
		rgb24buf[i + 1] = (beg[1] * (size - i) + end[1] * i) / size;
		rgb24buf[i + 2] = (beg[2] * (size - i) + end[2] * i) / size;
	}
}

int main(void)
{
	uint8_t beg[] = {0x00, 0x70, 0xc5};
	uint8_t end[] = {0x79, 0x29, 0xd2};
	uint8_t rgb24frame0[32 * 16 * 3];
	uint8_t rgb24frame1[120 * 50 * 3];
	uint8_t rgb24frame2[10 * 10 * 3];

	printf("input image, 32x16, %u bytes:\n", sizeof(rgb24frame0));
	gradient(rgb24frame0, sizeof(rgb24frame0), beg, end);
	pixel_print_rgb24frame_truecolor(rgb24frame0, sizeof(rgb24frame0), 32, 16);

	printf("output image, bigger, 120x16, %u bytes:\n", sizeof(rgb24frame1));
	pixel_subsample_rgb24frame(rgb24frame0, 32, 16, rgb24frame1, 120, 50);
	pixel_print_rgb24frame_truecolor(rgb24frame1, sizeof(rgb24frame1), 120, 50);

	printf("output image, smaller, 120x16, %u bytes:\n", sizeof(rgb24frame2));
	pixel_subsample_rgb24frame(rgb24frame0, 32, 16, rgb24frame2, 10, 10);
	pixel_print_rgb24frame_truecolor(rgb24frame2, sizeof(rgb24frame2), 10, 10);

	return 0;
}
