/*
 * Copyright (c) 2024, tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_VIDEO_COMMON_H
#define ZEPHYR_INCLUDE_VIDEO_COMMON_H

#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/drivers/video.h>

#ifndef ABS
#define ABS(a) ((a) < 0 ? -(a) : (a))
#endif

/**
 * @brief Search for a format that matches in a list of capabilities
 *
 * @param fmts The format capability list to search.
 * @param fmts_num The number of capabilities in that list.
 * @param fmt The format to find in the list.
 * @param idx The pointer to a number of the first format that matches.
 *
 * @return 0 when a format is found.
 * @return -ENOENT when no matching format is found.
 */
static inline int video_format_index(const struct video_format_cap *fmts,
				     const struct video_format *fmt, size_t *idx)
{
	for (int i = 0; fmts[i].pixelformat != 0; i++) {
		if (fmts[i].pixelformat == fmt->pixelformat &&
		    IN_RANGE(fmt->width, fmts[i].width_min, fmts[i].width_max) &&
		    IN_RANGE(fmt->height, fmts[i].height_min, fmts[i].height_max)) {
			*idx = i;
			return 0;
		}
	}
	return -ENOENT;
}

/**
 * @brief Compute the difference between two frame intervals
 *
 * @param ap First frame interval.
 * @param bp Second frame interval.
 *
 * @return The signed difference in microsecond between the two frame intervals.
 */
static inline int64_t video_diff_frmival_usec(const struct video_frmival *ap,
					      const struct video_frmival *bp)
{
	struct video_frmival a = *ap;
	struct video_frmival b = *bp;

	if (a.denominator != b.denominator) {
		a.numerator *= b.denominator;
		a.denominator *= b.denominator;
		b.numerator *= a.denominator;
		b.denominator *= a.denominator;
	}

	/* Return the difference in microseconds */
	return DIV_ROUND_CLOSEST((int64_t)USEC_PER_SEC * a.numerator, a.denominator) -
	       DIV_ROUND_CLOSEST((int64_t)USEC_PER_SEC * b.numerator, b.denominator);
}

/**
 * @brief Find the closest match to a frame interval value within a stepwise frame interval.
 *
 * @param stepwise The stepwise frame interval range to search
 * @param desired The frame interval for which find the closest match
 * @param best_match The resulting frame interval closest to @p desired
 */
static inline void video_closest_frmival_stepwise(const struct video_frmival_stepwise *stepwise,
						  const struct video_frmival *desired,
						  struct video_frmival *match)
{
	uint32_t min = stepwise->min.numerator;
	uint32_t max = stepwise->max.numerator;
	uint32_t step = stepwise->step.numerator;
	uint32_t goal = desired->numerator;
	uint32_t z1, z2;

	/* Set a common denominator to all values */
	min *= stepwise->max.denominator * stepwise->step.denominator * desired->denominator;
	max *= stepwise->min.denominator * stepwise->step.denominator * desired->denominator;
	step *= stepwise->min.denominator * stepwise->max.denominator * desired->denominator;
	goal *= stepwise->min.denominator * stepwise->max.denominator * stepwise->step.denominator;

	/* Compute a numerator and denominator */
	match->denominator = stepwise->min.denominator * stepwise->max.denominator *
			     stepwise->step.denominator * desired->denominator;
	match->numerator = min + DIV_ROUND_CLOSEST(goal - min, step) * step;

	/* To reduce risk of overflow: simplify the fraction by powers of 2 */
	z1 = u32_count_trailing_zeros(match->numerator);
	z2 = u32_count_trailing_zeros(match->denominator);
	match->numerator <<= MIN(z1, z2);
	match->denominator <<= MIN(z1, z2);
}

/**
 * @brief Find the closest match to a frame interval value within a video device.
 *
 * @param dev Video device to query.
 * @param ep Video endpoint ID to query.
 * @param match Frame interval enumerator with the query and loaded with the result.
 */
static inline void video_closest_frmival(const struct device *dev, enum video_endpoint_id ep,
					 struct video_frmival_enum *match)
{
	int32_t best_diff_usec = INT32_MAX;
	const struct video_frmival desired = match->discrete;
	struct video_frmival_enum fie = {.format = match->format};

	__ASSERT(match->type == VIDEO_FRMIVAL_TYPE_DISCRETE,
		 "cannot find range matching the range, only a value matching the range");

	while (video_enum_frmival(dev, ep, &fie) == 0) {
		struct video_frmival tmp = {0};
		int32_t diff_usec = 0;

		switch (fie.type) {
		case VIDEO_FRMIVAL_TYPE_DISCRETE:
			tmp = fie.discrete;
			break;
		case VIDEO_FRMIVAL_TYPE_STEPWISE:
			video_closest_frmival_stepwise(&fie.stepwise, &desired, &tmp);
			break;
		default:
			__ASSERT(false, "invalid answer from the queried video device");
		}

		diff_usec = video_diff_frmival_usec(&desired, &tmp);
		if (ABS(diff_usec) < best_diff_usec) {
			best_diff_usec = diff_usec;
			memcpy(&match->discrete, &tmp, sizeof(tmp));
			match->index = fie.index;
		}
	}
}

#endif /* ZEPHYR_INCLUDE_VIDEO_COMMON_H */
