/*
 * Copyright (c) 2024 Emcraft Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB device auto endpoint driver API
 */

#ifndef ZEPHYR_INCLUDE_UDC_AUTO_EP_H
#define ZEPHYR_INCLUDE_UDC_AUTO_EP_H

#include <zephyr/device.h>

struct udc_auto_ep_data {
	const struct device *udc;
	void *trbs;
	unsigned char num_trbs;
	unsigned char epaddr;
	unsigned short events;
};

struct udc_auto_ep_api {
	int (*start)(const struct device *dev);
	void (*stop)(const struct device *dev);
	void *(*ep_base)(const struct device *dev);
};

static inline int udc_auto_ep_start(const struct device *dev)
{
	const struct udc_auto_ep_api *api = dev->api;
	return api->start(dev);
}

static inline void udc_auto_ep_stop(const struct device *dev)
{
	const struct udc_auto_ep_api *api = dev->api;
	api->stop(dev);
}

static inline unsigned char udc_auto_ep_addr(const struct device *dev)
{
	const struct udc_auto_ep_data *data = dev->data;
	return data->epaddr;
}

static inline void *udc_auto_ep_base(const struct device *dev)
{
	const struct udc_auto_ep_api *api = dev->api;
	return api->ep_base(dev);
}

#endif /* ZEPHYR_INCLUDE_UDC_AUTO_EP_H */
