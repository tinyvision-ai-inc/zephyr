/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/slist.h>
#include <zephyr/logging/log.h>
#include <zephyr/pixel/palettize.h>

LOG_MODULE_REGISTER(pixel_palettize, CONFIG_PIXEL_LOG_LEVEL);

static int pixel_image_add_palette(struct pixel_image *img, const struct pixel_operation *op,
				   struct pixel_palette *palette)
{
	struct pixel_operation *tail;
	int ret;

	ret = pixel_image_add_uncompressed(img, op);
	if (ret != 0) {
		return ret;
	}

	tail = SYS_SLIST_PEEK_TAIL_CONTAINER(&img->operations, tail, node);
	tail->arg1 = palette;

	return 0;
}

int pixel_image_depalettize(struct pixel_image *img, struct pixel_palette *palette)
{
	const struct pixel_operation *op = NULL;

	STRUCT_SECTION_FOREACH_ALTERNATE(pixel_convert, pixel_operation, tmp) {
		if (tmp->format_in == img->format &&
		    tmp->format_out == palette->format) {
			op = tmp;
			break;
		}
	}

	if (op == NULL) {
		LOG_ERR("Conversion operation from %s to %s not found",
			PIXEL_FORMAT_TO_STR(img->format),
			PIXEL_FORMAT_TO_STR(palette->format));
		return pixel_image_error(img, -ENOSYS);
	}

	return pixel_image_add_palette(img, op, palette);
}

int pixel_image_palettize(struct pixel_image *img, struct pixel_palette *palette)
{
	const struct pixel_operation *op = NULL;
	uint32_t new_format = 0;
	int ret;

	ret = pixel_image_convert(img, palette->format);
	if (ret != 0) {
		return ret;
	}

	if (palette->size <= 1 << 1) {
		new_format = PIXEL_FORMAT_PALETTE1;
	} else if (palette->size <= 1 << 2) {
		new_format = PIXEL_FORMAT_PALETTE2;
	} else if (palette->size <= 1 << 4) {
		new_format = PIXEL_FORMAT_PALETTE4;
	} else if (palette->size <= 1 << 8) {
		new_format = PIXEL_FORMAT_PALETTE8;
	} else {
		CODE_UNREACHABLE;
	}

	STRUCT_SECTION_FOREACH_ALTERNATE(pixel_convert, pixel_operation, tmp) {
		if (tmp->format_in == img->format && tmp->format_out == new_format) {
			op = tmp;
			break;
		}
	}

	if (op == NULL) {
		LOG_ERR("Conversion operation from %s to %s not found",
			PIXEL_FORMAT_TO_STR(img->format),
			PIXEL_FORMAT_TO_STR(new_format));
		return pixel_image_error(img, -ENOSYS);
	}

	return pixel_image_add_palette(img, op, palette);
}

void pixel_palettize_op(struct pixel_operation *op)
{
	const uint8_t *line_in = pixel_operation_get_input_line(op);
	uint8_t *line_out = pixel_operation_get_output_line(op);
	void (*fn)(const uint8_t *s, uint8_t *d, uint16_t w, struct pixel_palette *p) = op->arg0;
	struct pixel_palette *palette = op->arg1;

	__ASSERT_NO_MSG(fn != NULL);

	fn(line_in, line_out, op->width, palette);
	pixel_operation_done(op);
}

/*
 * The 3D (R, G, B) distance between two points is given by:
 *
 * 	sqrt((r1 - r0)^2 + (g1 - g0)^2 + (b1 - b0)^2)
 *
 * But if A > B then sqrt(A) > sqrt(B), so no need to compute the square root.
 * The square of the distances are computed instead.
 */
static inline uint32_t pixel_rgb_square_distance(const uint8_t rgb0[3], const uint8_t rgb1[3])
{
	int16_t r_dist = (int16_t)rgb1[0] - (int16_t)rgb0[0];
	int16_t g_dist = (int16_t)rgb1[1] - (int16_t)rgb0[1];
	int16_t b_dist = (int16_t)rgb1[2] - (int16_t)rgb0[2];

	return r_dist * r_dist + g_dist * g_dist + b_dist + b_dist;
}

static inline uint8_t pixel_rgb24_to_palette4(const uint8_t rgb[3],
					      const struct pixel_palette *palette)
{
	uint8_t best_color[3];
	uint32_t best_square_distance = UINT32_MAX;
	uint8_t idx = 0;

	for (size_t i = 0; i < palette->size; i++) {
		uint8_t *color = &palette->colors[i * 3];
		uint32_t square_distance = pixel_rgb_square_distance(color, rgb);

		if (square_distance < best_square_distance) {
			best_square_distance = square_distance;
			memcpy(best_color, color, sizeof(best_color));
			idx = i;
		}
	}

	return idx;
}

__weak void pixel_line_rgb24_to_palette4(const uint8_t *src, uint8_t *dst, uint16_t width,
					 const struct pixel_palette *palette)
{
	__ASSERT_NO_MSG(width % 2 == 0);
	__ASSERT_NO_MSG(palette->size == 1 << 4);

	for (uint16_t w = 0; w < width; w += 2, src += 6, dst += 1) {
		dst[0] = 0;
		dst[0] |= pixel_rgb24_to_palette4(&src[0], palette) << 4;
		dst[0] |= pixel_rgb24_to_palette4(&src[3], palette) << 0;
	}
}
PIXEL_DEFINE_PALETTIZE_OPERATION(pixel_line_rgb24_to_palette4, RGB24, PALETTE4);

__weak void pixel_line_palette4_to_rgb24(const uint8_t *src, uint8_t *dst, uint16_t width,
					 const struct pixel_palette *palette)
{
	__ASSERT_NO_MSG(width % 2 == 0);
	__ASSERT_NO_MSG(palette->size == 1 << 4);

	for (uint16_t w = 0; w < width; w += 2, src += 1, dst += 6) {
		uint8_t *color0 = &palette->colors[(src[0] >> 4) * 3];
		uint8_t *color1 = &palette->colors[(src[0] & 0xf) * 3];

		dst[0] = color0[0];
		dst[1] = color0[1];
		dst[2] = color0[2];
		dst[3] = color1[0];
		dst[4] = color1[1];
		dst[5] = color1[2];
	}
}
PIXEL_DEFINE_DEPALETTIZE_OPERATION(pixel_line_palette4_to_rgb24, PALETTE4, RGB24);
