/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usb_cdc.h>
#include <zephyr/usb/class/usb_cdc_raw.h>

#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cdc_raw, CONFIG_USBD_CDC_RAW_LOG_LEVEL);

#define CDC_RAW_BULK_EP_MPS		1024
#define CDC_RAW_INT_EP_MPS		64
#define CDC_RAW_INT_INTERVAL		0x0A

struct cdc_raw_desc {
	struct usb_association_descriptor iad_cdc;

	struct usb_if_descriptor if0;
	struct cdc_header_descriptor if0_header;
	struct cdc_cm_descriptor if0_cm;
	struct cdc_acm_descriptor if0_acm;
	struct cdc_union_descriptor if0_union;
	struct usb_ep_descriptor if0_int_ep;
	struct usb_ep_companion_descriptor if0_int_ep_comp;

	struct usb_if_descriptor if1;
	struct usb_ep_descriptor if1_in_ep;
	struct usb_ep_companion_descriptor if1_in_ep_comp;
	struct usb_ep_descriptor if1_out_ep;
	struct usb_ep_companion_descriptor if1_out_ep_comp;

	struct usb_desc_header nil_desc;
} __packed;

struct cdc_raw_data {
	/* Functions called when a read or write completes */
	cdc_raw_callback_t *read_callback;
	cdc_raw_callback_t *write_callback;

	/* Flag telling that the DTR signal was sent from the host */
	bool data_terminal_ready;

	/* Point back to class structure */
	struct usbd_class_node *c_nd;
};

static uint8_t _get_bulk_in(struct usbd_class_node *const c_nd)
{
	struct cdc_raw_desc *desc = c_nd->data->desc;

	return desc->if1_in_ep.bEndpointAddress;
}

static uint8_t _get_bulk_out(struct usbd_class_node *const c_nd)
{
	struct cdc_raw_desc *desc = c_nd->data->desc;

	return desc->if1_out_ep.bEndpointAddress;
}

int cdc_raw_read(const struct device *dev, struct net_buf *buf)
{
	struct cdc_raw_data *data = dev->data;
	struct udc_buf_info *bi = udc_get_buf_info(buf);

	LOG_DBG("%s: buf=%p size=%d data=%p", __func__, buf, buf->size, buf->data);

	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->ep = _get_bulk_out(data->c_nd);
	return usbd_ep_enqueue(data->c_nd, buf);
}

int cdc_raw_write(const struct device *dev, struct net_buf *buf)
{
	struct cdc_raw_data *data = dev->data;
	struct udc_buf_info *bi = udc_get_buf_info(buf);

	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->ep = _get_bulk_in(data->c_nd);
	bi->zlp = true; /* Flush the transfer immediately after this buffer */
	return usbd_ep_enqueue(data->c_nd, buf);
}

void cdc_raw_set_read_callback(const struct device *dev, cdc_raw_callback_t *callback)
{
	struct cdc_raw_data *data = dev->data;

	data->read_callback = callback;
}

void cdc_raw_set_write_callback(const struct device *dev, cdc_raw_callback_t *callback)
{
	struct cdc_raw_data *data = dev->data;

	data->write_callback = callback;
}

bool cdc_raw_is_ready(const struct device *dev)
{
	struct cdc_raw_data *data = dev->data;

	return data->data_terminal_ready;
}

static int _api_request(struct usbd_class_node *const c_nd, struct net_buf *buf, int err)
{
	const struct device *dev = c_nd->data->priv;
	struct cdc_raw_data *data = dev->data;
	struct udc_buf_info *bi = udc_get_buf_info(buf);

	if (bi->ep == _get_bulk_out(c_nd) && data->read_callback) {
		LOG_DBG("%s: read_callback", __func__);
		return data->read_callback(dev, buf, err);
	}
	if (bi->ep == _get_bulk_in(c_nd) && data->write_callback) {
		LOG_DBG("%s: write_callback", __func__);
		return data->write_callback(dev, buf, err);
	}
	return 0;
}

static void _api_update(struct usbd_class_node *const c_nd, uint8_t iface, uint8_t alternate)
{
	LOG_DBG("New configuration, interface %u alternate %u",
		iface, alternate);
}

static int _api_cth(struct usbd_class_node *const c_nd, const struct usb_setup_packet *const setup, struct net_buf *const buf)
{
	return 0;
}

static int _api_ctd(struct usbd_class_node *const c_nd, const struct usb_setup_packet *const setup, const struct net_buf *const buf)
{
	const struct device *dev = c_nd->data->priv;
	struct cdc_raw_data *data = dev->data;

	switch (setup->bRequest) {
	case SET_CONTROL_LINE_STATE:
		if ((setup->wValue & SET_CONTROL_LINE_STATE_DTR)) {
			LOG_DBG("data_terminal_ready");
			data->data_terminal_ready = true;
		}
		break;
	}
	return 0;
}

static int _api_init(struct usbd_class_node *const c_nd)
{
	struct cdc_raw_desc *desc = c_nd->data->desc;

	desc->iad_cdc.bFirstInterface = desc->if0.bInterfaceNumber;
	desc->if0_union.bControlInterface = desc->if0.bInterfaceNumber;
	desc->if0_union.bSubordinateInterface0 = desc->if1.bInterfaceNumber;

	return 0;
}

struct usbd_class_api _api = {
	.request = _api_request,
	.update = _api_update,
	.control_to_host = _api_cth,
	.control_to_dev = _api_ctd,
	.init = _api_init,
};

#define CDC_RAW_DEFINE_DESCRIPTOR(n)						\
static struct cdc_raw_desc _desc_##n = {					\
										\
	.iad_cdc = {								\
		.bLength = sizeof(struct usb_association_descriptor),		\
		.bDescriptorType = USB_DESC_INTERFACE_ASSOC,			\
		.bFirstInterface = 0,						\
		.bInterfaceCount = 0x02,					\
		.bFunctionClass = USB_BCC_CDC_CONTROL,				\
		.bFunctionSubClass = ACM_SUBCLASS,				\
		.bFunctionProtocol = 0,						\
		.iFunction = 0,							\
	},									\
										\
	.if0 = {								\
		.bLength = sizeof(struct usb_if_descriptor),			\
		.bDescriptorType = USB_DESC_INTERFACE,				\
		.bInterfaceNumber = 0,						\
		.bAlternateSetting = 0,						\
		.bNumEndpoints = 1,						\
		.bInterfaceClass = USB_BCC_CDC_CONTROL,				\
		.bInterfaceSubClass = ACM_SUBCLASS,				\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
										\
	.if0_header = {								\
		.bFunctionLength = sizeof(struct cdc_header_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = HEADER_FUNC_DESC,				\
		.bcdCDC = sys_cpu_to_le16(USB_SRN_1_1),				\
	},									\
										\
	.if0_cm = {								\
		.bFunctionLength = sizeof(struct cdc_cm_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = CALL_MANAGEMENT_FUNC_DESC,		\
		.bmCapabilities = 0,						\
		.bDataInterface = 1,						\
	},									\
										\
	.if0_acm = {								\
		.bFunctionLength = sizeof(struct cdc_acm_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = ACM_FUNC_DESC,				\
		/* See CDC PSTN Subclass Chapter 5.3.2 */			\
		.bmCapabilities = BIT(1),					\
	},									\
										\
	.if0_union = {								\
		.bFunctionLength = sizeof(struct cdc_union_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = UNION_FUNC_DESC,				\
		.bControlInterface = 0,						\
		.bSubordinateInterface0 = 1,					\
	},									\
										\
	.if0_int_ep = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x81,					\
		.bmAttributes = USB_EP_TYPE_INTERRUPT,				\
		.wMaxPacketSize = sys_cpu_to_le16(CDC_RAW_INT_EP_MPS),		\
		.bInterval = CDC_RAW_INT_INTERVAL,				\
	},									\
										\
	.if0_int_ep_comp = {							\
		.bLength = sizeof(struct usb_ep_companion_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT_COMPANION,			\
		.bMaxBurst = 0,							\
		.bmAttributes = 0,						\
		.wBytesPerInterval = 0,						\
	},									\
										\
	.if1 = {								\
		.bLength = sizeof(struct usb_if_descriptor),			\
		.bDescriptorType = USB_DESC_INTERFACE,				\
		.bInterfaceNumber = 1,						\
		.bAlternateSetting = 0,						\
		.bNumEndpoints = 2,						\
		.bInterfaceClass = USB_BCC_CDC_DATA,				\
		.bInterfaceSubClass = 0,					\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
										\
	.if1_in_ep = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x82,					\
		.bmAttributes = USB_EP_TYPE_BULK,				\
		.wMaxPacketSize = sys_cpu_to_le16(CDC_RAW_BULK_EP_MPS),		\
		.bInterval = 0,							\
	},									\
										\
	.if1_in_ep_comp = {							\
		.bLength = sizeof(struct usb_ep_companion_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT_COMPANION,			\
		.bMaxBurst = 15,						\
		.bmAttributes = 0,						\
		.wBytesPerInterval = 0,						\
	},									\
										\
	.if1_out_ep = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x01,					\
		.bmAttributes = USB_EP_TYPE_BULK,				\
		.wMaxPacketSize = sys_cpu_to_le16(CDC_RAW_BULK_EP_MPS),		\
		.bInterval = 0,							\
	},									\
										\
	.if1_out_ep_comp = {							\
		.bLength = sizeof(struct usb_ep_companion_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT_COMPANION,			\
		.bMaxBurst = 15,						\
		.bmAttributes = 0,						\
		.wBytesPerInterval = 0,						\
	},									\
										\
	.nil_desc = {								\
		.bLength = 0,							\
		.bDescriptorType = 0,						\
	},									\
}

#define DT_DRV_COMPAT zephyr_cdc_raw

#define cdc_raw_DT_DEVICE_DEFINE(n)						\
										\
	BUILD_ASSERT(DT_INST_ON_BUS(n, usb),					\
		     "node " DT_NODE_PATH(DT_DRV_INST(n))			\
		     " is not assigned to a USB device controller");		\
										\
	CDC_RAW_DEFINE_DESCRIPTOR(n);						\
										\
	static struct usbd_class_data _class_data_##n = {			\
		.desc = (struct usb_desc_header *)&_desc_##n,			\
		.priv = (void *)DEVICE_DT_GET(DT_DRV_INST(n)),			\
	};									\
	USBD_DEFINE_CLASS(cdc_raw_##n, &_api, &_class_data_##n);		\
										\
	static struct cdc_raw_data _dev_data_##n = {				\
		.c_nd = &cdc_raw_##n,						\
	};									\
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, &_dev_data_##n, NULL,		\
		POST_KERNEL, 50, &_api);

DT_INST_FOREACH_STATUS_OKAY(cdc_raw_DT_DEVICE_DEFINE);
