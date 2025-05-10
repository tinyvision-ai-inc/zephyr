/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_PIXEL_FORMAT_H_
#define ZEPHYR_INCLUDE_PIXEL_FORMAT_H_

#include <stdlib.h>
#include <stdint.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

/**
 * @brief Describes a pixel format.
 */
struct pixel_format {
	/** Standard four character code describing the format */
	uint32_t fourcc;
	/** Describe the average number of bits per pixel, 0 if variable */
	uint8_t bits_per_pixel;
	/** Number of pixels to skip horizontally to get next aligned data. 1 for simple formats. */
	uint8_t block_width;
	/** Number of pixels to skip vertically to get next aligned data.1 for simple formats.  */
	uint8_t block_height;
};
typedef const struct pixel_format *pixel_format_t;

#define PIXEL_FORMAT_TO_STR(_format) VIDEO_FOURCC_TO_STR((_format)->fourcc)

/**
 * @brief Define a new pixel format, with defaults for most common values.
 * @param a 1st character of the Four Character Code
 * @param b 2nd character of the Four Character Code
 * @param c 3rd character of the Four Character Code
 * @param d 4rd character of the Four Character Code
 * @param _bits_per_pixel Number of bits per pixel for this format
 * @param _block_width Block width for this format
 * @param _block_height Block height for this format
 */
#define PIXEL_FORMAT(a, b, c, d, _bits_per_pixel, _block_width, _block_height)                     \
	(&(const struct pixel_format){                                                             \
		.fourcc = VIDEO_FOURCC(a, b, c, d),                                                \
		.bits_per_pixel = (_bits_per_pixel),                                               \
		.block_width = (_block_width),                                                     \
		.block_height = (_block_height),                                                   \
	})

#define PIXEL_FORMAT_RGB332	PIXEL_FORMAT('R', 'G', 'B', '1', 8, 1, 1)
#define PIXEL_FORMAT_RGB565	PIXEL_FORMAT('R', 'G', 'B', 'P', 16, 1, 1)
#define PIXEL_FORMAT_RGB565X	PIXEL_FORMAT('R', 'G', 'B', 'R', 16, 1, 1)
#define PIXEL_FORMAT_RGB24	PIXEL_FORMAT('R', 'G', 'B', '3', 24, 1, 1)
#define PIXEL_FORMAT_YUV24	PIXEL_FORMAT('Y', 'U', 'V', '3', 24, 1, 1)
#define PIXEL_FORMAT_YUYV	PIXEL_FORMAT('Y', 'U', 'Y', 'V', 16, 2, 1)
#define PIXEL_FORMAT_GREY	PIXEL_FORMAT('G', 'R', 'E', 'Y', 8, 1, 1)
#define PIXEL_FORMAT_SRGGB8	PIXEL_FORMAT('R', 'G', 'G', 'B', 8, 2, 2)
#define PIXEL_FORMAT_SBGGR8	PIXEL_FORMAT('B', 'G', 'G', 'R', 8, 2, 2)
#define PIXEL_FORMAT_SGBRG8	PIXEL_FORMAT('G', 'B', 'R', 'G', 8, 2, 2)
#define PIXEL_FORMAT_SGRBG8	PIXEL_FORMAT('G', 'R', 'B', 'G', 8, 2, 2)

#endif /* ZEPHYR_INCLUDE_PIXEL_FORMAT_H_ */
