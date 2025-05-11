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

#define PIXEL_FORMAT_TO_STR(_format) VIDEO_FOURCC_TO_STR(_format)

/**
 * @brief Define a new pixel format, with defaults for most common values.
 * @param a 1st character of the Four Character Code
 * @param b 2nd character of the Four Character Code
 * @param c 3rd character of the Four Character Code
 * @param d 4rd character of the Four Character Code
 */
#define PIXEL_FOURCC(a, b, c, d) VIDEO_FOURCC((a), (b), (c), (d))

static inline uint8_t pixel_bits_per_pixel(uint32_t fourcc)
{
	switch (fourcc) {

#define PIXEL_FORMAT_RGB332 PIXEL_FOURCC('R', 'G', 'B', '1')
	case PIXEL_FORMAT_RGB332:
		return 8;

#define PIXEL_FORMAT_RGB565 PIXEL_FOURCC('R', 'G', 'B', 'P')
	case PIXEL_FORMAT_RGB565:
		return 16;

#define PIXEL_FORMAT_RGB565X PIXEL_FOURCC('R', 'G', 'B', 'R')
	case PIXEL_FORMAT_RGB565X:
		return 16;

#define PIXEL_FORMAT_RGB24 PIXEL_FOURCC('R', 'G', 'B', '3')
	case PIXEL_FORMAT_RGB24:
		return 24;

#define PIXEL_FORMAT_YUV12 PIXEL_FOURCC('Y', 'U', 'V', 'C')
	case PIXEL_FORMAT_YUV12:
		return 12;

#define PIXEL_FORMAT_YUV24 PIXEL_FOURCC('Y', 'U', 'V', '3')
	case PIXEL_FORMAT_YUV24:
		return 24;

#define PIXEL_FORMAT_YUYV PIXEL_FOURCC('Y', 'U', 'Y', 'V')
	case PIXEL_FORMAT_YUYV:
		return 16;

#define PIXEL_FORMAT_GREY PIXEL_FOURCC('G', 'R', 'E', 'Y')
	case PIXEL_FORMAT_GREY:
		return 8;

#define PIXEL_FORMAT_SRGGB8 PIXEL_FOURCC('R', 'G', 'G', 'B')
	case PIXEL_FORMAT_SRGGB8:
		return 8;

#define PIXEL_FORMAT_SBGGR8 PIXEL_FOURCC('B', 'G', 'G', 'R')
	case PIXEL_FORMAT_SBGGR8:
		return 8;

#define PIXEL_FORMAT_SGBRG8 PIXEL_FOURCC('G', 'B', 'R', 'G')
	case PIXEL_FORMAT_SGBRG8:
		return 8;

#define PIXEL_FORMAT_SGRBG8 PIXEL_FOURCC('G', 'R', 'B', 'G')
	case PIXEL_FORMAT_SGRBG8:
		return 8;

#define PIXEL_FORMAT_PALETTE1 PIXEL_FOURCC('P', 'L', 'T', '1')
	case PIXEL_FORMAT_PALETTE1:
		return 1;

#define PIXEL_FORMAT_PALETTE2 PIXEL_FOURCC('P', 'L', 'T', '2')
	case PIXEL_FORMAT_PALETTE2:
		return 2;

#define PIXEL_FORMAT_PALETTE4 PIXEL_FOURCC('P', 'L', 'T', '4')
	case PIXEL_FORMAT_PALETTE4:
		return 4;

#define PIXEL_FORMAT_PALETTE8 PIXEL_FOURCC('P', 'L', 'T', '8')
	case PIXEL_FORMAT_PALETTE8:
		return 8;

	default:
		return 0;
	}
}

#endif /* ZEPHYR_INCLUDE_PIXEL_FORMAT_H_ */
