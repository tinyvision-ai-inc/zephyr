/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zephyr/pixel/formats.h>
#include <zephyr/pixel/print.h>

#define WIDTH  20
#define HEIGHT 4

static const uint8_t image_rgb24[WIDTH * HEIGHT * 3] = {
	0x47, 0x84, 0xee, 0x46, 0x84, 0xee, 0x47, 0x84, 0xee, 0x46, 0x83, 0xee, 0x47, 0x84, 0xee,
	0x78, 0xaa, 0xec, 0x74, 0xb2, 0xe0, 0x67, 0xaa, 0xdd, 0x78, 0xb2, 0xef, 0x39, 0x8c, 0xf1,
	0x3a, 0x8c, 0xf2, 0x39, 0x8b, 0xf1, 0x3a, 0x8b, 0xf1, 0x3a, 0x8b, 0xf1, 0x3a, 0x8b, 0xf1,
	0x3a, 0x8b, 0xf1, 0x3b, 0x8b, 0xf1, 0x3b, 0x8b, 0xf1, 0x3b, 0x8b, 0xf1, 0x3b, 0x8a, 0xf1,
	0x47, 0x82, 0xed, 0x47, 0x82, 0xed, 0x47, 0x82, 0xee, 0x47, 0x82, 0xed, 0x47, 0x82, 0xed,
	0x47, 0x82, 0xed, 0x5d, 0x93, 0xed, 0x5f, 0x9d, 0xeb, 0x3a, 0x8a, 0xf1, 0x3a, 0x8a, 0xf0,
	0x3a, 0x8a, 0xf1, 0x3a, 0x8a, 0xf0, 0x3b, 0x8a, 0xf0, 0x3b, 0x8a, 0xf0, 0x3b, 0x8a, 0xf0,
	0x3b, 0x89, 0xf0, 0x3b, 0x8a, 0xf0, 0x3b, 0x89, 0xf0, 0x3c, 0x89, 0xf0, 0x3c, 0x89, 0xf0,
	0x49, 0x82, 0xee, 0x49, 0x82, 0xed, 0x49, 0x82, 0xee, 0x48, 0x82, 0xed, 0x49, 0x82, 0xee,
	0x49, 0x82, 0xed, 0x73, 0x92, 0xe9, 0x50, 0x65, 0xd4, 0x4c, 0x93, 0xf2, 0x3c, 0x8a, 0xf1,
	0x3c, 0x8a, 0xf1, 0x3c, 0x8a, 0xf0, 0x3c, 0x8a, 0xf1, 0x3c, 0x89, 0xf0, 0x3d, 0x8a, 0xf1,
	0x3d, 0x89, 0xf0, 0x3d, 0x89, 0xf0, 0x3d, 0x89, 0xf0, 0x3e, 0x89, 0xf0, 0x3d, 0x89, 0xf0,
	0x4a, 0x81, 0xed, 0x49, 0x81, 0xed, 0x4a, 0x81, 0xed, 0x49, 0x81, 0xed, 0x49, 0x81, 0xed,
	0x71, 0x8c, 0xe5, 0x3e, 0x4c, 0xcc, 0x3d, 0x4c, 0xcb, 0x65, 0x85, 0xe1, 0x3d, 0x89, 0xf0,
	0x3d, 0x89, 0xf0, 0x3d, 0x88, 0xf0, 0x3d, 0x89, 0xf0, 0x3d, 0x88, 0xf0, 0x3e, 0x88, 0xf0,
	0x3e, 0x88, 0xf0, 0x3e, 0x88, 0xf0, 0x3e, 0x88, 0xf0, 0x3f, 0x88, 0xf0, 0x3e, 0x88, 0xef,
};

/* Stream conversion steps are declared at build time */
PIXEL_STREAM_RGB24_TO_RGB565LE(step_rgb24_to_rgb565le, WIDTH, HEIGHT);
PIXEL_STREAM_RGB565LE_TO_RGB24(step_rgb565le_to_rgb24, WIDTH, HEIGHT);
PIXEL_STREAM_RGB24_TO_YUYV_BT709(step_rgb24_to_yuyv, WIDTH, HEIGHT);

int main(void)
{
	uint8_t image_yuyv[WIDTH * HEIGHT * 2] = {0};
	struct pixel_stream root = {0};
	struct pixel_stream *step = &root;
	struct pixel_stream step_export_yuyv = {
		.ring = RING_BUF_INIT(image_yuyv, sizeof(image_yuyv)),
		.pitch = WIDTH * 2,
		.name = "[export_yuyv]",
	};

	printf("Input image preview:\n");
	pixel_print_rgb24frame_truecolor(image_rgb24, sizeof(image_rgb24), WIDTH, HEIGHT);

	/* Interconnection between the stream steps can be done arbitrarily at runtime */
	step = step->next = &step_rgb24_to_rgb565le;
	step = step->next = &step_rgb565le_to_rgb24;
	step = step->next = &step_rgb24_to_yuyv;
	step = step->next = &step_export_yuyv;
	pixel_stream_load(root.next, image_rgb24, sizeof(image_rgb24));

	printf("Output image preview:\n");
	pixel_print_yuyvframe_bt709_truecolor(image_yuyv, sizeof(image_yuyv), WIDTH, HEIGHT);

	return 0;
}
