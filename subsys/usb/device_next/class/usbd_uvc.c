/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usb_uvc.h>

#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_uvc, CONFIG_USBD_UVC_LOG_LEVEL);

struct usbd_uvc_desc {
	struct usb_association_descriptor iad_uvc;
	struct usb_if_descriptor if0;
	struct uvc_input_terminal_descriptor if0_input;
	struct uvc_encoding_unit_descriptor if0_encoding;
	struct cdc_acm_descriptor if0_acm;
	struct cdc_union_descriptor if0_union;
	struct usb_ep_descriptor if0_int_ep;

	struct usb_if_descriptor if1;
	struct usb_ep_descriptor if1_in_ep;
	struct usb_ep_descriptor if1_out_ep;

	struct usb_desc_header nil_desc;
} __packed;

#define UVC_DEFINE_DESCRIPTOR(n)						\
static struct usbd_uvc_desc uvc_desc_##n = {					\
										\
	.nil_desc = {								\
		.bLength = 0,							\
		.bDescriptorType = 0,						\
	},									\
}

#define DT_DRV_COMPAT zephyr_uvc

#define USBD_UVC_DT_DEVICE_DEFINE(n)						\
	BUILD_ASSERT(DT_INST_ON_BUS(n, usb),					\
		     "node " DT_NODE_PATH(DT_DRV_INST(n))			\
		     " is not assigned to a USB device controller");		\
										\
	UVC_DEFINE_DESCRIPTOR(n);						\
										\
	static struct usbd_class_data usbd_cdc_acm_data_##n;			\
										\
	USBD_DEFINE_CLASS(cdc_acm_##n,						\
			  &usbd_cdc_acm_api,					\
			  &usbd_cdc_acm_data_##n);				\
										\
	static struct cdc_acm_uart_data uart_data_##n = {			\
		.line_coding = CDC_ACM_DEFAULT_LINECODING,			\
		.c_nd = &cdc_acm_##n,						\
		.rx_fifo.rb = &cdc_acm_rb_rx_##n,				\
		.tx_fifo.rb = &cdc_acm_rb_tx_##n,				\
		.notif_sem = Z_SEM_INITIALIZER(uart_data_##n.notif_sem, 0, 1),	\
	};									\
										\
	static struct usbd_class_data usbd_cdc_acm_data_##n = {			\
		.desc = (struct usb_desc_header *)&cdc_acm_desc_##n,		\
		.priv = (void *)DEVICE_DT_GET(DT_DRV_INST(n)),			\
	};									\


DT_INST_FOREACH_STATUS_OKAY(USBD_UVC_DT_DEVICE_DEFINE);
