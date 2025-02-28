/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Video Class public header
 *
 * Header exposes API for registering source video device to UVC instances.
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBD_UVC_H
#define ZEPHYR_INCLUDE_USB_CLASS_USBD_UVC_H

#include <zephyr/kernel.h>

/**
 * @brief Configure the video device that is used by UVC
 *
 * The USB Video Class sends queries from the host to a source video device using video API calls.
 * In order to let an UVC instance know which device is used as back-end for the video stream,
 * this function must be called for every UVC instance, and before @ref usbd_enable().
 *
 * @param uvc_dev UVC instance to which assign the video device.
 * @param video_dev Device to be configured as source for this UVC stream.
 */
void uvc_set_video_dev(const struct device *uvc_dev, const struct device *video_dev);

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBD_UVC_H */
