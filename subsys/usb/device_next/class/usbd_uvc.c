/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usb_uvc.h>

#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_uvc, CONFIG_USBD_UVC_LOG_LEVEL);

#define UVC_DEFAULT_BULK_EP_MPS 0

struct usbd_uvc_desc {
	struct usb_association_descriptor iad_uvc;

	struct usb_if_descriptor if0;
	struct uvc_interface_header_descriptor if0_vctl_hdr;
	struct uvc_output_terminal_descriptor if0_vctl_out;
	struct uvc_input_terminal_descriptor if0_vctl_in;

	struct usb_if_descriptor if1;
	struct uvc_stream_input_header_descriptor if1_stream_in;
	struct uvc_uncompressed_format_descriptor if1_format;
	struct uvc_uncompressed_frame_descriptor if1_frame0;

	struct usb_if_descriptor if2;
	struct usb_ep_descriptor if2_in_ep;
	struct usb_ep_companion_descriptor if2_in_ep_comp;

	struct usb_desc_header nil_desc;
} __packed;

static int usbd_uvc_request(struct usbd_class_node *const c_nd,
				struct net_buf *buf, int err)
{
	return 0;
}

static void usbd_uvc_update(struct usbd_class_node *const c_nd,
				uint8_t iface, uint8_t alternate)
{
}

static int usbd_uvc_cth(struct usbd_class_node *const c_nd,
			    const struct usb_setup_packet *const setup,
			    struct net_buf *const buf)
{
	return 0;
}

static int usbd_uvc_ctd(struct usbd_class_node *const c_nd,
			    const struct usb_setup_packet *const setup,
			    const struct net_buf *const buf)
{
	return 0;
}

static int usbd_uvc_init(struct usbd_class_node *const c_nd)
{
	struct usbd_uvc_desc *desc = c_nd->data->desc;

	desc->iad_uvc.bFirstInterface = desc->if0.bInterfaceNumber;
	return 0;
}

struct usbd_class_api usbd_uvc_api = {
	.request = usbd_uvc_request,
	.update = usbd_uvc_update,
	.control_to_host = usbd_uvc_cth,
	.control_to_dev = usbd_uvc_ctd,
	.init = usbd_uvc_init,
};

#define UVC_DEFINE_DESCRIPTOR(n)						\
static struct usbd_uvc_desc uvc_desc_##n = {					\
										\
	.iad_uvc = {								\
		.bLength = sizeof(struct usb_association_descriptor),		\
		.bDescriptorType = USB_DESC_INTERFACE_ASSOC,			\
		.bFirstInterface = 0,						\
		.bInterfaceCount = 2,						\
		.bFunctionClass = USB_BCC_VIDEO,				\
		.bFunctionSubClass = UVC_SC_VIDEO_INTERFACE_COLLECITON,		\
		.bFunctionProtocol = UVC_PC_PROTOCOL_UNDEFINED,			\
		.iFunction = 0,							\
	},									\
										\
	.if0 = {								\
		.bLength = sizeof(struct usb_if_descriptor),			\
		.bDescriptorType = USB_DESC_INTERFACE,				\
		.bInterfaceNumber = 0,						\
		.bAlternateSetting = 0,						\
		.bNumEndpoints = 0,						\
		.bInterfaceClass = USB_BCC_VIDEO,				\
		.bInterfaceSubClass = UVC_SC_VIDEOCONTROL,			\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
										\
	.if0_vctl_hdr = {							\
		.bLength = sizeof(struct uvc_interface_header_descriptor),	\
		.bDescriptorType = UVC_CS_INTERFACE,				\
		.bDescriptorSubtype = UVC_VC_HEADER,				\
		.bcdUVC = 0x0150,						\
		.wTotalLength = sys_cpu_to_le16(				\
			sizeof(struct uvc_interface_header_descriptor)		\
			+ sizeof(struct uvc_input_terminal_descriptor)		\
			+ sizeof(struct uvc_output_terminal_descriptor)		\
		),								\
		.dwClockFrequency = 30000000,					\
		.bInCollection = 1,						\
		.baInterfaceNr = 1,						\
	},									\
										\
	.if0_vctl_out = {							\
		.bLength = sizeof(struct uvc_output_terminal_descriptor),	\
		.bDescriptorType = UVC_CS_INTERFACE,				\
		.bDescriptorSubtype = UVC_VC_OUTPUT_TERMINAL,			\
		.bTerminalID = 2,						\
		.wTerminalType = sys_cpu_to_le16(UVC_TT_STREAMING),		\
		.bAssocTerminal = 0,						\
		.bSourceID = 2,							\
		.iTerminal = 0,							\
	},									\
										\
	.if0_vctl_in = {							\
		.bLength = sizeof(struct uvc_input_terminal_descriptor),	\
		.bDescriptorType = UVC_CS_INTERFACE,				\
		.bDescriptorSubtype = UVC_VC_INPUT_TERMINAL,			\
		.bTerminalID = 1,						\
		.wTerminalType = sys_cpu_to_le16(UVC_ITT_CAMERA),		\
		.bAssocTerminal = 0,						\
		.iTerminal = 0,							\
		.wObjectiveFocalLengthMin = sys_cpu_to_le16(0),			\
		.wObjectiveFocalLengthMax = sys_cpu_to_le16(0),			\
		.wOcularFocalLength = sys_cpu_to_le16(0),			\
		.bControlSize = 3,						\
		.bmControls = { 0x00, 0x00, 0x00 },				\
	},									\
										\
	.if1 = {								\
		.bLength = sizeof(struct usb_if_descriptor),			\
		.bDescriptorType = USB_DESC_INTERFACE,				\
		.bInterfaceNumber = 1,						\
		.bAlternateSetting = 0,						\
		.bNumEndpoints = 0,						\
		.bInterfaceClass = USB_BCC_VIDEO,				\
		.bInterfaceSubClass = UVC_SC_VIDEOSTREAMING,			\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
										\
	.if1_stream_in = {							\
		.bLength = sizeof(struct uvc_stream_input_header_descriptor),	\
		.bDescriptorType = UVC_CS_INTERFACE,				\
		.bDescriptorSubtype = UVC_VS_INPUT_HEADER,			\
		.bNumFormats = 1,						\
		.wTotalLength = sys_cpu_to_le16(				\
			sizeof(struct uvc_stream_input_header_descriptor)	\
			+ sizeof(struct uvc_uncompressed_format_descriptor)	\
			+ sizeof(struct uvc_uncompressed_frame_descriptor)	\
		),								\
		.bEndpointAddress = 0x81,					\
		.bmInfo = 0,							\
		.bTerminalLink = 2,						\
		.bStillCaptureMethod = 0,					\
		.bTriggerSupport = 0,						\
		.bTriggerUsage = 0,						\
		.bControlSize = 1,						\
		.bmaControls = { 0x00 },					\
	},									\
										\
	.if1_format = {								\
		.bLength = sizeof(struct uvc_uncompressed_format_descriptor),	\
		.bDescriptorType = UVC_CS_INTERFACE,				\
		.bDescriptorSubtype = UVC_VS_FORMAT_UNCOMPRESSED,		\
		.bFormatIndex = 1,						\
		.bNumFrameDescriptors = 1,					\
		.guidFormat = { 0x59,0x55,0x59,0x32, 0x00,0x00, 0x10,0x00,	\
			0x80,0x00, 0x00,0xAA,0x00,0x38,0x9B,0x71, },		\
		.bBitsPerPixel = 16,						\
		.bDefaultFrameIndex = 1,					\
		.bAspectRatioX = 0,						\
		.bAspectRatioY = 0,						\
		.bmInterlaceFlags = 0x00,					\
		.bCopyProtect = 0,						\
	},									\
										\
	.if1_frame0 = {								\
		.bLength = sizeof(struct uvc_uncompressed_frame_descriptor),	\
		.bDescriptorType = UVC_CS_INTERFACE,				\
		.bDescriptorSubtype = UVC_VS_FRAME_UNCOMPRESSED,		\
		.bFrameIndex = 1,						\
		.bmCapabilities = 0x00,						\
		.wWidth = sys_cpu_to_le16(640),					\
		.wHeight = sys_cpu_to_le16(480),				\
		.dwMinBitRate = sys_cpu_to_le32(15360000),			\
		.dwMaxBitRate = sys_cpu_to_le32(15360000),			\
		.dwMaxVideoFrameBufferSize = sys_cpu_to_le32(614400),		\
		.dwDefaultFrameInterval = sys_cpu_to_le32(400000),		\
		.bFrameIntervalType = 0,					\
	},									\
										\
	.if2 = {								\
		.bLength = sizeof(struct usb_if_descriptor),			\
		.bDescriptorType = USB_DESC_INTERFACE,				\
		.bInterfaceNumber = 1,						\
		.bAlternateSetting = 1,						\
		.bNumEndpoints = 1,						\
		.bInterfaceClass = USB_BCC_VIDEO,				\
		.bInterfaceSubClass = UVC_SC_VIDEOSTREAMING,			\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
										\
	.if2_in_ep = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x81,					\
		.bmAttributes = USB_EP_TYPE_BULK,				\
		.wMaxPacketSize = sys_cpu_to_le16(UVC_DEFAULT_BULK_EP_MPS),	\
		.bInterval = 0,							\
	},									\
										\
	.if2_in_ep_comp = {							\
		.bLength = sizeof(struct usb_ep_companion_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT_COMPANION,			\
		.bMaxBurst = 0,							\
		.bmAttributes = 0,						\
		.wBytesPerInterval = sys_cpu_to_le16(2),			\
	},									\
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
	static struct usbd_class_data usbd_uvc_data_##n;			\
	USBD_DEFINE_CLASS(uvc_##n, &usbd_uvc_api, &usbd_uvc_data_##n);		\
	struct usbd_class_node *const uvc_c_nd = &uvc_##n;			\
										\
	static struct usbd_class_data usbd_uvc_data_##n = {			\
		.desc = (struct usb_desc_header *)&uvc_desc_##n,		\
		.priv = (void *)DEVICE_DT_GET(DT_DRV_INST(n)),			\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, NULL, NULL,					\
		&usbd_uvc_data_##n, NULL,					\
		POST_KERNEL, 50,						\
		&usbd_uvc_api);

DT_INST_FOREACH_STATUS_OKAY(USBD_UVC_DT_DEVICE_DEFINE);
