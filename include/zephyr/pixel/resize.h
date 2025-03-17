/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_PIXEL_RESIZE_H
#define ZEPHYR_INCLUDE_PIXEL_RESIZE_H

#include <stdint.h>

#include <zephyr/pixel/stream.h>

/**
 * @defgroup Resize streams by subsampling the pixels.
 *
 * @note the "width" and "height" macro parameters are for the input like any stream element.
 *       The output size is configured implicitly by connecting this block to another one of a
 *       different size.
 *
 * @{
 */

void pixel_subsample_rgb24stream(struct pixel_stream *strm);

#define PIXEL_STREAM_SUBSAMPLE_RGB24(name, width, height)                                          \
	PIXEL_STREAM_DEFINE(name, pixel_subsample_rgb24stream,                                     \
			    (width), (height), (width) * 3, 1 * (width) * 3)

void pixel_subsample_rgb565stream(struct pixel_stream *strm);

#define PIXEL_STREAM_SUBSAMPLE_RGB565(name, width, height)                                         \
	PIXEL_STREAM_DEFINE(name, pixel_subsample_rgb565stream,                                    \
			    (width), (height), (width) * 3, 1 * (width) * 3)

/**
 * @}
 */

/**
 * @brief Step-by-step increment two indexes preserving their proportion.
 *
 * @param src_index Position to sample on the input image, adjusted to preserve the same ratio
 *                  from @p src_width to @p dst_width and from @p src_index to @p dst_index.
 * @param src_width Width of the input image to sample.
 * @param dst_index Index through the destination image, always incremented by one.
 * @param dst_width Targeted width for the resized output.
 * @param debt Accumulator telling how much overshooting was done in either direction.
 *             It must be initialized to zero on first call.
 */
void pixel_subsample_step(size_t *src_index, size_t src_width, size_t *dst_index, size_t dst_width,
			  int32_t *debt);

/**
 * @brief Resize a line of pixel by compressing/stretching it horizontally.
 *
 * Subsampling is used as algorithm to compute the new pixels: fast but low quality.
 *
 * @param src_buf Input buffer to resize
 * @param src_width Number of pixels to resize to a different format.
 * @param dst_buf Output buffer in which the stretched/compressed is stored.
 * @param dst_width Number of pixels in the output buffer.
 * @{
 */
/** RGB24 variant */
void pixel_subsample_rgb24line(const uint8_t *src_buf, size_t src_width, uint8_t *dst_buf,
			       size_t dst_width);
/** RGB565 variant */
void pixel_subsample_rgb565line(const uint8_t *src_buf, size_t src_width, uint8_t *dst_buf,
				size_t dst_width);
/** @} */

/**
 * @brief Resize a buffer of pixels by compressing/stretching it horizontally and vertically.
 *
 * Subsampling is used as algorithm to compute the new pixels: fast but low quality.
 *
 * The aspect ratio is chosen by the width and height parameters, and preserving the proportions is
 * a choice that the user can optionally make.
 *
 * @param src_buf Input buffer to resize
 * @param src_width Width of the input in number of pixels.
 * @param src_height Height of the input in number of pixels.
 * @param dst_buf Output buffer in which the stretched/compressed is stored.
 * @param dst_width Width of the outputin number of pixels.
 * @param dst_height Height of the outputin number of pixels.
 * @{
 */
/* RGB24 variant */
void pixel_subsample_rgb24frame(const uint8_t *src_buf, size_t src_width, size_t src_height,
				uint8_t *dst_buf, size_t dst_width, size_t dst_height);
/* RGB565 variant */
void pixel_subsample_rgb565frame(const uint8_t *src_buf, size_t src_width, size_t src_height,
				 uint8_t *dst_buf, size_t dst_width, size_t dst_height);
/** @} */

#endif /* ZEPHYR_INCLUDE_PIXEL_RESIZE_H */
