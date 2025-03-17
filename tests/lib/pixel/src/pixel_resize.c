/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/ztest.h>
#include <zephyr/pixel/resize.h>
#include <zephyr/pixel/formats.h>
#include <zephyr/pixel/print.h>

#define WIDTH_IN 32
#define HEIGHT_IN 16
#define PITCH_IN (WIDTH_IN * 3)

#define WIDTH_OUT 24
#define HEIGHT_OUT 24
#define PITCH_OUT (WIDTH_OUT * 3)

#define ERROR_MARGIN 9

/* Input/output buffers */
static uint8_t rgb24frame_in[WIDTH_IN * HEIGHT_IN * 3];
static uint8_t rgb24frame_out[WIDTH_OUT * HEIGHT_OUT * 3];

/* Input feed from RGB24 */
static PIXEL_STREAM_RGB24_TO_RGB565BE(step_rgb24_to_rgb565be, WIDTH_IN, HEIGHT_IN);
static PIXEL_STREAM_RGB24_TO_RGB565LE(step_rgb24_to_rgb565le, WIDTH_IN, HEIGHT_IN);

/* Resize the stream image */
static PIXEL_STREAM_SUBSAMPLE_RGB24(step_subsample_rgb24, WIDTH_IN, HEIGHT_IN);
static PIXEL_STREAM_SUBSAMPLE_RGB565(step_subsample_rgb565, WIDTH_IN, HEIGHT_IN);

/* resized feed to RGB24 */
static PIXEL_STREAM_RGB565BE_TO_RGB24(step_rgb565be_to_rgb24, WIDTH_OUT, HEIGHT_OUT);
static PIXEL_STREAM_RGB565LE_TO_RGB24(step_rgb565le_to_rgb24, WIDTH_OUT, HEIGHT_OUT);

struct pixel_stream root = {0};

static void test_resize(void)
{
	/* Run the pipeline */
	pixel_stream_load(root.next, rgb24frame_in, sizeof(rgb24frame_in));

	printf("input:\n");
	pixel_print_rgb24frame_truecolor(rgb24frame_in, sizeof(rgb24frame_in), WIDTH_IN, HEIGHT_IN);

	printf("output:\n");
	pixel_print_rgb24frame_truecolor(rgb24frame_out, sizeof(rgb24frame_out), WIDTH_OUT,
					 HEIGHT_OUT);

	size_t w = WIDTH_OUT;
	size_t h = HEIGHT_OUT;
	size_t p = PITCH_OUT;

	/* Test top left quadramt */
	zassert_within(rgb24frame_out[(0) * p + (0) * 3 + 0], 0x00, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(0) * p + (0) * 3 + 1], 0x00, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(0) * p + (0) * 3 + 2], 0x7f, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 - 1) * p + (w / 2 - 1) * 3 + 0], 0x00, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 - 1) * p + (w / 2 - 1) * 3 + 1], 0x00, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 - 1) * p + (w / 2 - 1) * 3 + 2], 0x7f, ERROR_MARGIN);

	/* Test bottom left quadrant */
	zassert_within(rgb24frame_out[(h - 1) * p + (0) * 3 + 0], 0x00, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h - 1) * p + (0) * 3 + 1], 0xff, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h - 1) * p + (0) * 3 + 2], 0x7f, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 + 1) * p + (w / 2 - 1) * 3 + 0], 0x00, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 + 1) * p + (w / 2 - 1) * 3 + 1], 0xff, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 + 1) * p + (w / 2 - 1) * 3 + 2], 0x7f, ERROR_MARGIN);

	/* Test top right quadrant */
	zassert_within(rgb24frame_out[(0) * p + (w - 1) * 3 + 0], 0xff, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(0) * p + (w - 1) * 3 + 1], 0x00, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(0) * p + (w - 1) * 3 + 2], 0x7f, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 - 1) * p + (w / 2 + 1) * 3 + 0], 0xff, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 - 1) * p + (w / 2 + 1) * 3 + 1], 0x00, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 - 1) * p + (w / 2 + 1) * 3 + 2], 0x7f, ERROR_MARGIN);

	/* Test bottom right quadrant */
	zassert_within(rgb24frame_out[(h - 1) * p + (w - 1) * 3 + 0], 0xff, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h - 1) * p + (w - 1) * 3 + 1], 0xff, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h - 1) * p + (w - 1) * 3 + 2], 0x7f, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 + 1) * p + (w / 2 + 1) * 3 + 0], 0xff, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 + 1) * p + (w / 2 + 1) * 3 + 1], 0xff, ERROR_MARGIN);
	zassert_within(rgb24frame_out[(h / 2 + 1) * p + (w / 2 + 1) * 3 + 2], 0x7f, ERROR_MARGIN);
}

ZTEST(lib_pixel_resize, test_pixel_resize_stream)
{
	struct pixel_stream step_export_rgb24 = {
		.ring = RING_BUF_INIT(rgb24frame_out, sizeof(rgb24frame_out)),
		.pitch = WIDTH_OUT * 3,
		.width = WIDTH_OUT,
		.height = HEIGHT_OUT,
		.name = "[export]",
	};
	struct pixel_stream *step;

	/* Generate test input data */
	for (uint16_t h = 0; h <= HEIGHT_IN; h++) {
		for (uint16_t w = 0; w < WIDTH_IN; w++) {
			rgb24frame_in[h * PITCH_IN + w * 3 + 0] = w < WIDTH_IN / 2 ? 0x00 : 0xff;
			rgb24frame_in[h * PITCH_IN + w * 3 + 1] = h < HEIGHT_IN / 2 ? 0x00 : 0xff;
			rgb24frame_in[h * PITCH_IN + w * 3 + 2] = 0x7f;
		}
	}

	/* RGB24 */
	step = &root;
	step = step->next = &step_subsample_rgb24;
	step = step->next = &step_export_rgb24;
	test_resize();
	pixel_stream_get_all_input(&step_export_rgb24);

	/* RGB565LE */
	step = &root;
	step = step->next = &step_rgb24_to_rgb565le;
	step = step->next = &step_subsample_rgb565;
	step = step->next = &step_rgb565le_to_rgb24;
	step = step->next = &step_export_rgb24;
	test_resize();
	pixel_stream_get_all_input(&step_export_rgb24);

	/* RGB565BE */
	step = &root;
	step = step->next = &step_rgb24_to_rgb565be;
	step = step->next = &step_subsample_rgb565;
	step = step->next = &step_rgb565be_to_rgb24;
	step = step->next = &step_export_rgb24;
	test_resize();
	pixel_stream_get_all_input(&step_export_rgb24);
}

ZTEST_SUITE(lib_pixel_resize, NULL, NULL, NULL, NULL, NULL);
