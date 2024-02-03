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

#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_cdc_uvc, CONFIG_USBD_CDC_UVC_LOG_LEVEL);

NET_BUF_POOL_DEFINE(cdc_uvc_ep_pool, 2, 0, sizeof(struct udc_buf_info), NULL);

#define CDC_UVC_DEFAULT_LINECODING	{sys_cpu_to_le32(115200), 0, 0, 8}
#define CDC_UVC_DEFAULT_BULK_EP_MPS	0
#define CDC_UVC_DEFAULT_INT_EP_MPS	16
#define CDC_UVC_DEFAULT_INT_INTERVAL	0x0A

struct usb_ep_companion_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bMaxBurst;
	uint8_t bmAttributes;
	uint16_t wBytesPerInterval;
} __packed;

struct usbd_cdc_uvc_desc {
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

struct usbd_class_node *const cdc_uvc_c_nd;

static uint8_t cdc_uvc_get_bulk_in(struct usbd_class_node *const c_nd)
{
	struct usbd_cdc_uvc_desc *desc = c_nd->data->desc;

	return desc->if1_in_ep.bEndpointAddress;
}

static uint8_t cdc_uvc_get_bulk_out(struct usbd_class_node *const c_nd)
{
	struct usbd_cdc_uvc_desc *desc = c_nd->data->desc;

	return desc->if1_out_ep.bEndpointAddress;
}

void cdc_uvc_enqueue_in(char const *data_buf, size_t data_len)
{
	struct usbd_class_node *c_nd = cdc_uvc_c_nd;
	struct net_buf *buf;
	struct udc_buf_info *bi;
	int ret;

	buf = net_buf_alloc_with_data(&cdc_uvc_ep_pool,
		data_buf, data_len, K_NO_WAIT);
	if (buf == NULL) {
		LOG_DBG("%s buf=NULL err=alloc", __func__);
		return;
	}

	bi = udc_get_buf_info(buf);
	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->ep = cdc_uvc_get_bulk_in(c_nd);

	ret = usbd_ep_enqueue(c_nd, buf);
	if (ret) {
		LOG_DBG("%s buf=%p err=usbd", __func__, buf);
		goto err;
	}

	LOG_DBG("%s buf=%p err=ok", __func__, buf);
	return;
err:
	net_buf_unref(buf);
}

static int usbd_cdc_uvc_request(struct usbd_class_node *const c_nd,
				struct net_buf *buf, int err)
{
	struct usbd_contex *uds_ctx = c_nd->data->uds_ctx;
	struct udc_buf_info *bi;

	LOG_DBG("%s buf=%p err=%d", __func__, buf, err);

	bi = udc_get_buf_info(buf);
	if (err) {
		if (err == -ECONNABORTED) {
			LOG_WRN("request ep 0x%02x, len %u cancelled",
				bi->ep, buf->len);
		} else {
			LOG_ERR("request ep 0x%02x, len %u failed",
				bi->ep, buf->len);
		}
		return usbd_ep_buf_free(uds_ctx, buf);
	}

	if (bi->ep == cdc_uvc_get_bulk_out(c_nd)) {
		/* RX transfer completion */
	}

	if (bi->ep == cdc_uvc_get_bulk_in(c_nd)) {
		/* TX transfer completion */
	}

	return 0;
}

static void usbd_cdc_uvc_update(struct usbd_class_node *const c_nd,
				uint8_t iface, uint8_t alternate)
{
	LOG_DBG("New configuration, interface %u alternate %u",
		iface, alternate);
}

static int usbd_cdc_uvc_cth(struct usbd_class_node *const c_nd,
			    const struct usb_setup_packet *const setup,
			    struct net_buf *const buf)
{
	return 0;
}

static int usbd_cdc_uvc_ctd(struct usbd_class_node *const c_nd,
			    const struct usb_setup_packet *const setup,
			    const struct net_buf *const buf)
{
	return 0;
}

static int usbd_cdc_uvc_init(struct usbd_class_node *const c_nd)
{
	struct usbd_cdc_uvc_desc *desc = c_nd->data->desc;

	desc->iad_cdc.bFirstInterface = desc->if0.bInterfaceNumber;
	desc->if0_union.bControlInterface = desc->if0.bInterfaceNumber;
	desc->if0_union.bSubordinateInterface0 = desc->if1.bInterfaceNumber;

	return 0;
}

struct usbd_class_api usbd_cdc_uvc_api = {
	.request = usbd_cdc_uvc_request,
	.update = usbd_cdc_uvc_update,
	.control_to_host = usbd_cdc_uvc_cth,
	.control_to_dev = usbd_cdc_uvc_ctd,
	.init = usbd_cdc_uvc_init,
};

#define CDC_UVC_DEFINE_DESCRIPTOR(n)						\
static struct usbd_cdc_uvc_desc cdc_uvc_desc_##n = {				\
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
		.wMaxPacketSize = sys_cpu_to_le16(CDC_UVC_DEFAULT_INT_EP_MPS),	\
		.bInterval = CDC_UVC_DEFAULT_INT_INTERVAL,			\
	},									\
										\
	.if0_int_ep_comp = {							\
		.bLength = sizeof(struct usb_ep_companion_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT_COMPANION,			\
		.bMaxBurst = 0,							\
		.bmAttributes = 0,						\
		.wBytesPerInterval = 2,						\
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
		.wMaxPacketSize = sys_cpu_to_le16(CDC_UVC_DEFAULT_BULK_EP_MPS),	\
		.bInterval = 0,							\
	},									\
										\
	.if1_in_ep_comp = {							\
		.bLength = sizeof(struct usb_ep_companion_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT_COMPANION,			\
		.bMaxBurst = 0,							\
		.bmAttributes = 0,						\
		.wBytesPerInterval = 2,						\
	},									\
										\
	.if1_out_ep = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x01,					\
		.bmAttributes = USB_EP_TYPE_BULK,				\
		.wMaxPacketSize = sys_cpu_to_le16(CDC_UVC_DEFAULT_BULK_EP_MPS),	\
		.bInterval = 0,							\
	},									\
										\
	.if1_out_ep_comp = {							\
		.bLength = sizeof(struct usb_ep_companion_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT_COMPANION,			\
		.bMaxBurst = 0,							\
		.bmAttributes = 0,						\
		.wBytesPerInterval = 2,						\
	},									\
										\
	.nil_desc = {								\
		.bLength = 0,							\
		.bDescriptorType = 0,						\
	},									\
}

#define DT_DRV_COMPAT zephyr_cdc_uvc

#define USBD_CDC_UVC_DT_DEVICE_DEFINE(n)					\
	BUILD_ASSERT(DT_INST_ON_BUS(n, usb),					\
		     "node " DT_NODE_PATH(DT_DRV_INST(n))			\
		     " is not assigned to a USB device controller");		\
										\
	CDC_UVC_DEFINE_DESCRIPTOR(n);						\
										\
	static struct usbd_class_data usbd_cdc_uvc_data_##n;			\
										\
	USBD_DEFINE_CLASS(cdc_uvc_##n,						\
			  &usbd_cdc_uvc_api,					\
			  &usbd_cdc_uvc_data_##n);				\
	struct usbd_class_node *const cdc_uvc_c_nd = &cdc_uvc_##n;		\
										\
	static struct usbd_class_data usbd_cdc_uvc_data_##n = {			\
		.desc = (struct usb_desc_header *)&cdc_uvc_desc_##n,		\
		.priv = (void *)DEVICE_DT_GET(DT_DRV_INST(n)),			\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, NULL, NULL,					\
		NULL, NULL,							\
		POST_KERNEL, 50,						\
		&usbd_cdc_uvc_api);

DT_INST_FOREACH_STATUS_OKAY(USBD_CDC_UVC_DT_DEVICE_DEFINE);
