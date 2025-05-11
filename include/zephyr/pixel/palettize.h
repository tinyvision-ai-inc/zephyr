/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_PIXEL_PALETTIZE_H_
#define ZEPHYR_INCLUDE_PIXEL_PALETTIZE_H_

#include <stdlib.h>
#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/pixel/image.h>

struct pixel_palette {
	uint8_t *colors;
	uint16_t size;
	uint32_t format;
};

/**
 * @brief Define a new format conversion operation.
 *
 * @param fn Function converting one input line.
 * @param fmt_in The input format for that operation.
 * @param fmt_out The Output format for that operation.
 */
#define PIXEL_DEFINE_PALETTIZE_OPERATION(fn, fmt_in, fmt_out)                                      \
	static const STRUCT_SECTION_ITERABLE_ALTERNATE(pixel_convert, pixel_operation,             \
						       fn##_op) = {                                \
		.name = #fn,                                                                       \
		.format_in = (PIXEL_FORMAT_##fmt_in),                                              \
		.format_out = (PIXEL_FORMAT_##fmt_out),                                            \
		.window_size = 1,                                                                  \
		.run = pixel_palettize_op,                                                         \
		.arg0 = (fn),                                                                      \
	}

#define PIXEL_DEFINE_DEPALETTIZE_OPERATION PIXEL_DEFINE_PALETTIZE_OPERATION

/**
 * @brief Helper to turn a line palettization function into an operation.
 *
 * The line conversion function is to be provided in @c op->arg.
 * It processes on the input line to convert it to the destination format.
 *
 * The palette is to be provided in @c op->arg1.
 *
 * @param op Current operation in progress.
 */
void pixel_palettize_op(struct pixel_operation *op);

/**
 * @brief Convert a line of pixel data from RGB24 to PALETTE8.
 *
 * You only need to call this function to work directly on raw buffers.
 * See @ref pixel_image_palettize for a more convenient high-level API.
 *
 * @param src Buffer of the input line, with the format @c XXX in @c pixel_line_XXX_to_YYY().
 * @param dst Buffer of the output line, with the format @c YYY in @c pixel_line_XXX_to_YYY().
 * @param width Width of the lines in number of pixels.
 */
void pixel_line_rgb24_to_palette8(const uint8_t *src, uint8_t *dst, uint16_t width,
				  const struct pixel_palette *palette);
/**
 * @brief Convert a line of pixel data from PALETTE8 to RGB24.
 * @copydetails pixel_line_rgb24_to_palette8
 */
void pixel_line_palette8_to_rgb24(const uint8_t *src, uint8_t *dst, uint16_t width,
				  const struct pixel_palette *palette);
/**
 * @brief Convert a line of pixel data from RGB24 to PALETTE4.
 * @copydetails pixel_line_rgb24_to_palette8
 */
void pixel_line_rgb24_to_palette4(const uint8_t *src, uint8_t *dst, uint16_t width,
				  const struct pixel_palette *palette);
/**
 * @brief Convert a line of pixel data from PALETTE4 to RGB24.
 * @copydetails pixel_line_rgb24_to_palette8
 */
void pixel_line_palette4_to_rgb24(const uint8_t *src, uint8_t *dst, uint16_t width,
				  const struct pixel_palette *palette);
/**
 * @brief Convert a line of pixel data from RGB24 to PALETTE2.
 * @copydetails pixel_line_rgb24_to_palette8
 */
void pixel_line_rgb24_to_palette2(const uint8_t *src, uint8_t *dst, uint16_t width,
				  const struct pixel_palette *palette);
/**
 * @brief Convert a line of pixel data from PALETTE2 to RGB24.
 * @copydetails pixel_line_rgb24_to_palette8
 */
void pixel_line_palette2_to_rgb24(const uint8_t *src, uint8_t *dst, uint16_t width,
				  const struct pixel_palette *palette);
/**
 * @brief Convert a line of pixel data from RGB24 to PALETTE1.
 * @copydetails pixel_line_rgb24_to_palette8
 */
void pixel_line_rgb24_to_palette1(const uint8_t *src, uint8_t *dst, uint16_t width,
				  const struct pixel_palette *palette);
/**
 * @brief Convert a line of pixel data from PALETTE1 to RGB24.
 * @copydetails pixel_line_rgb24_to_palette8
 */
void pixel_line_palette1_to_rgb24(const uint8_t *src, uint8_t *dst, uint16_t width,
				  const struct pixel_palette *palette);

#endif /* ZEPHYR_INCLUDE_PIXEL_PALETTIZE_H_ */
