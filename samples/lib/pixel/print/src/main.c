/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zephyr/pixel/print.h>

int print_image(void)
{
	uint8_t beg[] = {0x00, 0x70, 0xc5}, end[] = {0x79, 0x29, 0xd2};
	uint8_t rgb24[16 * 32 * 3];

	/* Generate an image with a gradient of the two colors above */
	for (int i = 0; i + 3 <= sizeof(rgb24); i += 3) {
		rgb24[i + 0] = (beg[0] * (sizeof(rgb24) - i) + end[0] * i) / sizeof(rgb24);
		rgb24[i + 1] = (beg[1] * (sizeof(rgb24) - i) + end[1] * i) / sizeof(rgb24);
		rgb24[i + 2] = (beg[2] * (sizeof(rgb24) - i) + end[2] * i) / sizeof(rgb24);
	}

	printf("Printing the gradient #%02x%02x%02x -> #%02x%02x%02x\n",
		beg[0], beg[1], beg[2], end[0], end[1], end[2]);

	printf("truecolor:\n");
	pixel_print_rgb24frame_truecolor(rgb24, sizeof(rgb24), 16, 32);

	printf("256color:\n");
	pixel_print_rgb24frame_256color(rgb24, sizeof(rgb24), 16, 32);

	printf("hexdump:\n");
	pixel_print_rgb24frame_hex(rgb24, sizeof(rgb24), 16, 32);

	return 0;
}

int print_histogram(void)
{
	uint16_t rgb24hist[] = {
		9, 4, 7, 1, 0, 5, 1, 0, 0, 2, 2, 3, 0, 1, 3, 0,
		7, 6, 5, 1, 1, 4, 2, 0, 1, 2, 3, 4, 1, 1, 2, 2,
		8, 4, 7, 4, 2, 3, 1, 2, 2, 2, 2, 2, 0, 0, 1, 1,
	};
	uint16_t y8hist[] = {
		8, 5, 6, 2, 1, 4, 1, 1, 1, 2, 3, 3, 1, 1, 2, 1,
	};

	printf("Printing a histogram of %u RGB buckets\n", ARRAY_SIZE(rgb24hist));
	pixel_print_rgb24hist(rgb24hist, ARRAY_SIZE(rgb24hist), 8);

	printf("Printing a histogram of %u Y (luma) buckets\n", ARRAY_SIZE(y8hist));
	pixel_print_y8hist(y8hist, ARRAY_SIZE(y8hist), 8);

	return 0;
}

int main(void)
{
	print_image();
	print_histogram();
}
