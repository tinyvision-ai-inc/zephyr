/**
 * @file
 *
 * @brief Public APIs for Video.
 */

/*
 * Copyright (c) 2019 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_VIDEO_FORMATS_H_
#define ZEPHYR_INCLUDE_VIDEO_FORMATS_H_

/**
 * @brief Video Formats
 * @defgroup video_formats Video Formats
 * @since 3.7
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/* fourcc - four-character-code */
#define VIDEO_FOURCC(a, b, c, d) ((a) | ((b) << 8) | ((c) << 16) | ((d) << 24))

/**
 * @name Bayer formats
 * @{
 */

/** BGGR8 pixel format */
#define VIDEO_PIX_FMT_BGGR8 VIDEO_FOURCC('B', 'G', 'G', 'R') /*  8  BGBG.. GRGR.. */
#define VIDEO_PIX_FMT_BGGR8_BPP 8
/** GBRG8 pixel format */
#define VIDEO_PIX_FMT_GBRG8 VIDEO_FOURCC('G', 'B', 'R', 'G') /*  8  GBGB.. RGRG.. */
#define VIDEO_PIX_FMT_GBRG8_BPP 8
/** GRBG8 pixel format */
#define VIDEO_PIX_FMT_GRBG8 VIDEO_FOURCC('G', 'R', 'B', 'G') /*  8  GRGR.. BGBG.. */
#define VIDEO_PIX_FMT_GRBG8_BPP 8
/** RGGB8 pixel format */
#define VIDEO_PIX_FMT_RGGB8 VIDEO_FOURCC('R', 'G', 'G', 'B') /*  8  RGRG.. GBGB.. */
#define VIDEO_PIX_FMT_RGGB8_BPP 8

/**
 * @}
 */

/**
 * @name RGB formats
 * @{
 */

/** RGB565 pixel format */
#define VIDEO_PIX_FMT_RGB565 VIDEO_FOURCC('R', 'G', 'B', 'P') /* 16  RGB-5-6-5 */
#define VIDEO_PIX_FMT_RGB565_BPP 16

/** XRGB32 pixel format */
#define VIDEO_PIX_FMT_XRGB32 VIDEO_FOURCC('B', 'X', '2', '4') /* 32  XRGB-8-8-8-8 */
#define VIDEO_PIX_FMT_XRGB32_BPP 32

/**
 * @}
 */

/**
 * @name YUV formats
 * @{
 */

/** YUYV pixel format */
#define VIDEO_PIX_FMT_YUYV VIDEO_FOURCC('Y', 'U', 'Y', '2') /* 16  Y0-Cb0 Y1-Cr0 */
#define VIDEO_PIX_FMT_YUYV_BPP 16

/** XYUV32 pixel format */
#define VIDEO_PIX_FMT_XYUV32 VIDEO_FOURCC('X', 'Y', 'U', 'V') /* 32  XYUV-8-8-8-8 */
#define VIDEO_PIX_FMT_XYUV32_BPP 16

/**
 *
 * @}
 */

/**
 * @name JPEG formats
 * @{
 */

/** JPEG pixel format */
#define VIDEO_PIX_FMT_JPEG VIDEO_FOURCC('J', 'P', 'E', 'G') /*  8  JPEG */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_VIDEO_H_ */
