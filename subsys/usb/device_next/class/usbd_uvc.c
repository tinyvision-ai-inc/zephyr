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

#define DT_DRV_COMPAT zephyr_uvc

#define CASE(x) case x: LOG_DBG(#x)
#define INC_LE(sz, x, i) x = sys_cpu_to_le##sz(sys_le##sz##_to_cpu(x) + i)
#define UVC_INFO_SUPPORTS_GET_SET	((1 << 0) | (1 << 1))
#define FRAME_WIDTH			320
#define FRAME_HEIGHT			240
#define BLOCK_SIZE			0xffff
#define BITS_PER_PIXEL			16
#define FRAME_SIZE			(FRAME_WIDTH * FRAME_HEIGHT * (BITS_PER_PIXEL / 8))
#define TRANSFER_SIZE			(FRAME_SIZE + sizeof(struct uvc_payload_header))

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0);
NET_BUF_POOL_FIXED_DEFINE(_buf_pool, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 100,
	0, sizeof(struct udc_buf_info), NULL);

struct uvc_data {
	struct uvc_vs_probe_control probe;

	/* Data buffer continuously sent by the UVC class */
	uintptr_t payload_addr;

	/* Pointer to a header structure that can be passed to DMA */
	struct uvc_payload_header *payload_header;

	/* Current state of the transfer to avoid enqueue it twice */
	bool transferring;
};

struct uvc_desc {
	struct usb_association_descriptor iad_uvc;

	/* VideoControl */
	struct usb_if_descriptor			if0;
	struct uvc_interface_header_descriptor		if0_vctl_hdr;
	struct uvc_output_terminal_descriptor		if0_vctl_out;
	struct uvc_input_terminal_descriptor		if0_vctl_in;

	/* VideoStreaming */
	struct usb_if_descriptor			if1;
	struct uvc_stream_input_header_descriptor	if1_stream_in;
	struct uvc_uncompressed_format_descriptor	if1_format;
	struct uvc_uncompressed_frame_descriptor	if1_frame0;
	struct usb_ep_descriptor			if1_in_ep;
	struct usb_ep_companion_descriptor		if1_in_ep_comp;

	struct usb_desc_header nil_desc;
} __packed;

static const struct uvc_vs_probe_control _probe_def = {
	.bFormatIndex = 1,
	.bFrameIndex = 1,
	.dwFrameInterval = sys_cpu_to_le32(300*1000*1000 /*ns*/ / 100),
	.wDelay = sys_cpu_to_le16(100 /*ms*/),
	.dwMaxVideoFrameSize = sys_cpu_to_le32(FRAME_SIZE),
	.dwMaxPayloadTransferSize = sys_cpu_to_le32(TRANSFER_SIZE),
	.dwClockFrequency = sys_cpu_to_le32(1000000 /*Hz*/),
	.bmFramingInfo = UVC_BMFRAMING_INFO_FID,
	.bPreferedVersion = 1,
	.bMinVersion = 1,
	.bMaxVersion = 1,
	.bBitDepthLuma = BITS_PER_PIXEL - 8,
};

static const struct uvc_vs_probe_control _probe_min = {
	.bFormatIndex = 1,
	.bFrameIndex = 1,
	.dwFrameInterval = sys_cpu_to_le32(300*1000*1000 /*ns*/ / 100),
	.wDelay = sys_cpu_to_le16(1 /*ms*/),
	.dwMaxVideoFrameSize = sys_cpu_to_le32(FRAME_SIZE),
	.dwMaxPayloadTransferSize = sys_cpu_to_le32(TRANSFER_SIZE),
	.dwClockFrequency = sys_cpu_to_le32(1000 /*Hz*/),
	.bmFramingInfo = UVC_BMFRAMING_INFO_FID,
	.bPreferedVersion = 1,
	.bMinVersion = 1,
	.bMaxVersion = 1,
	.bBitDepthLuma = BITS_PER_PIXEL - 8,
};

static const struct uvc_vs_probe_control _probe_max = {
	.bFormatIndex = 1,
	.bFrameIndex = 1,
	.dwFrameInterval = sys_cpu_to_le32(300*1000*1000 /*ns*/ / 100),
	.wDelay = sys_cpu_to_le16(10000 /*ms*/),
	.dwMaxVideoFrameSize = sys_cpu_to_le32(FRAME_SIZE),
	.dwMaxPayloadTransferSize = sys_cpu_to_le32(TRANSFER_SIZE),
	.dwClockFrequency = sys_cpu_to_le32(10000000 /*Hz*/),
	.bmFramingInfo = UVC_BMFRAMING_INFO_FID,
	.bPreferedVersion = 255,
	.bMinVersion = 255,
	.bMaxVersion = 255,
	.bBitDepthLuma = BITS_PER_PIXEL - 8,
};

static void _dump_probe(const struct uvc_vs_probe_control *pc)
{
	LOG_DBG("struct uvc_vs_probe_control");
	LOG_DBG(".bmHint = 0x%04x", sys_le16_to_cpu(pc->bmHint));
	LOG_DBG(".bFormatIndex = %d", pc->bFormatIndex);
	LOG_DBG(".bFrameIndex = %d", pc->bFrameIndex);
	LOG_DBG(".dwFrameInterval = %d ns", sys_le32_to_cpu(pc->dwFrameInterval) * 100);
	LOG_DBG(".wKeyFrameRate = %d", sys_le16_to_cpu(pc->wKeyFrameRate));
	LOG_DBG(".wPFrameRate = %d", pc->wPFrameRate);
	LOG_DBG(".wCompQuality = %d", pc->wCompQuality);
	LOG_DBG(".wCompWindowSize = %d", pc->wCompWindowSize);
	LOG_DBG(".wDelay = %d ms", sys_le16_to_cpu(pc->wDelay));
	LOG_DBG(".dwMaxVideoFrameSize = %d", sys_le32_to_cpu(pc->dwMaxVideoFrameSize));
	LOG_DBG(".dwMaxPayloadTransferSize = %d", sys_le32_to_cpu(pc->dwMaxPayloadTransferSize));
	LOG_DBG(".dwClockFrequency = %d Hz", sys_le32_to_cpu(pc->dwClockFrequency));
	LOG_DBG(".bmFramingInfo = 0x%02x", pc->bmFramingInfo);
	LOG_DBG(".bPreferedVersion = %d", pc->bPreferedVersion);
	LOG_DBG(".bMinVersion = %d", pc->bMinVersion);
	LOG_DBG(".bMaxVersion = %d", pc->bMaxVersion);
	LOG_DBG(".bUsage = %d", pc->bUsage);
	LOG_DBG(".bBitDepthLuma = %d", pc->bBitDepthLuma + 8);
	LOG_DBG(".bmSettings = %d", pc->bmSettings);
	LOG_DBG(".bMaxNumberOfRefFramesPlus1 = %d", pc->bMaxNumberOfRefFramesPlus1);
	LOG_DBG(".bmRateControlModes = %d", pc->bmRateControlModes);
	LOG_DBG(".bmLayoutPerStream = { %d, %d, %d, %d }",
		pc->bmLayoutPerStream[0], pc->bmLayoutPerStream[1],
		pc->bmLayoutPerStream[2], pc->bmLayoutPerStream[3]);
}

/*
 * All fields set to 0 are ignored, others are applied
 * TODO: check if within MIN/MAX range
 */
static void _load_probe(struct uvc_vs_probe_control *dst, const struct uvc_vs_probe_control *src)
{
#define APPLY_PARAM(p)								\
	if (src->p != 0 && dst->p != src->p) {					\
		LOG_DBG(".%s = %d -> %d", #p, (int)dst->p, (int)src->p);	\
		dst->p = src->p;						\
	}
	APPLY_PARAM(bFormatIndex);
	APPLY_PARAM(bFrameIndex);
	APPLY_PARAM(dwFrameInterval);
	APPLY_PARAM(wKeyFrameRate);
	APPLY_PARAM(wPFrameRate);
	APPLY_PARAM(wCompQuality);
	APPLY_PARAM(wCompWindowSize);
	APPLY_PARAM(wDelay);
	APPLY_PARAM(dwMaxVideoFrameSize);
	APPLY_PARAM(dwMaxPayloadTransferSize);
	APPLY_PARAM(dwClockFrequency);
	APPLY_PARAM(bmFramingInfo);
	APPLY_PARAM(bPreferedVersion);
	APPLY_PARAM(bMinVersion);
	APPLY_PARAM(bMaxVersion);
	APPLY_PARAM(bUsage);
	APPLY_PARAM(bBitDepthLuma);
	APPLY_PARAM(bmSettings);
	APPLY_PARAM(bMaxNumberOfRefFramesPlus1);
	APPLY_PARAM(bmRateControlModes);
	APPLY_PARAM(bmLayoutPerStream[0]);
	APPLY_PARAM(bmLayoutPerStream[1]);
	APPLY_PARAM(bmLayoutPerStream[2]);
	APPLY_PARAM(bmLayoutPerStream[3]);
#undef APPLY_PARAM
}

static uint8_t _get_bulk_ep_num(struct usbd_class_node *const c_nd)
{
	struct uvc_desc *desc = c_nd->data->desc;

	return desc->if1_in_ep.bEndpointAddress;
}

static struct net_buf *_alloc_net_buf(struct usbd_class_node *const c_nd, void *data, size_t size)
{
	struct net_buf *buf;
	struct udc_buf_info *bi;

	buf = net_buf_alloc_with_data(&_buf_pool, (void *)data, size, K_NO_WAIT);
	__ASSERT_NO_MSG(buf != NULL);

	bi = udc_get_buf_info(buf);
	__ASSERT_NO_MSG(bi != NULL);

	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->ep = _get_bulk_ep_num(c_nd);

	return buf;
}

static int _try_enqueue(struct usbd_class_node *const c_nd, void *data, size_t size, bool last_of_transfer)
{
	struct net_buf *buf;
	struct udc_buf_info *bi;
	int ret;

	LOG_DBG("%s: Enqueueing data=%p size=%d zlp=%d", __func__, data, size, last_of_transfer);

	buf = _alloc_net_buf(c_nd, data, size);
	if (buf == NULL) {
		LOG_WRN("%s: Not enough buffers", __func__);
		return -ENOMEM;
	}

	bi = udc_get_buf_info(buf);
	bi->zlp = last_of_transfer;

	ret = usbd_ep_enqueue(c_nd, buf);
	if (ret) {
		LOG_WRN("%s: Could not enqueue buf=%p", __func__, buf);
		net_buf_unref(buf);
	}
	return ret;
}

static void _start_transfer(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct uvc_data *data = dev->data;
	int ret;

	if (data->transferring) {
		return;
	}

	data->transferring = true;

	/* Toggle the FrameId bit for every new frame. */
	// TODO only do this once per frame, not once per transfer header
	data->payload_header->bmHeaderInfo ^= UVC_BMHEADERINFO_FRAMEID;
	INC_LE(32, data->payload_header->dwPresentationTime, 1);
	INC_LE(32, data->payload_header->scrSourceClockSTC, 1);
	INC_LE(16, data->payload_header->scrSourceClockSOF, 1);

	LOG_INF("Submitting a %dx%d frame with %d bits per pixel (%zd bytes)",
		FRAME_WIDTH, FRAME_HEIGHT, BITS_PER_PIXEL, FRAME_SIZE);

	ret = _try_enqueue(c_nd, data->payload_header, sizeof(struct uvc_payload_header), false);
	__ASSERT_NO_MSG(ret == 0);

	for (size_t i = 0; i < data->probe.dwMaxVideoFrameSize; i += BLOCK_SIZE) {
		const size_t size_left = data->probe.dwMaxVideoFrameSize - i;
		const size_t size_sent = MIN(size_left, BLOCK_SIZE);

		ret = _try_enqueue(c_nd, (void *)data->payload_addr, size_sent,
			size_left <= BLOCK_SIZE);
		__ASSERT_NO_MSG(ret == 0);
	}
}

static void _complete_transfer(struct usbd_class_node *const c_nd)
{
	const struct device *dev = c_nd->data->priv;
	struct uvc_data *data = dev->data;

	data->transferring = false;
}

static int _api_request(struct usbd_class_node *const c_nd, struct net_buf *buf, int err)
{
	struct udc_buf_info bi;

	LOG_DBG("%s: buf=%p len=%d err=%d", __func__, buf, buf->len, err);

	/* Local copy so that we can free buf */
	memcpy(&bi, udc_get_buf_info(buf), sizeof(bi));
	net_buf_unref(buf);

	if (bi.ep == _get_bulk_ep_num(c_nd)) {
		LOG_DBG("%s: transfer completed zlp=%d", __func__, bi.zlp);
		_complete_transfer(c_nd);
		if (bi.zlp) {
			_start_transfer(c_nd);
		}
	} else {
		__ASSERT(false, "transfer completion for unknown enpoint");
	}

	return err;
}

static void _api_update(struct usbd_class_node *const c_nd, uint8_t iface, uint8_t alternate)
{
	LOG_DBG("%s", __func__);
}

static int _api_cth(struct usbd_class_node *const c_nd,
			    const struct usb_setup_packet *const setup,
			    struct net_buf *const buf)
{
	const struct device *dev = c_nd->data->priv;
	struct uvc_data *data = dev->data;
	size_t size = 0;

	switch (setup->wValue >> 8) {
	CASE(UVC_VS_PROBE_CONTROL);
		size = MIN(setup->wLength, sizeof(_probe_def));

		switch (setup->bRequest) {
		CASE(UVC_GET_CUR);
			net_buf_add_mem(buf, &data->probe, size);
			_dump_probe((void *)buf->data);
			return 0;
		CASE(UVC_GET_MIN);
			net_buf_add_mem(buf, &_probe_min, size);
			_dump_probe((void *)buf->data);
			break;
		CASE(UVC_GET_MAX);
			net_buf_add_mem(buf, &_probe_max, size);
			_dump_probe((void *)buf->data);
			break;
		CASE(UVC_GET_DEF);
			net_buf_add_mem(buf, &_probe_def, size);
			_dump_probe((void *)buf->data);
			break;
		CASE(UVC_GET_LEN);
			net_buf_add_le16(buf, sizeof(_probe_def));
			return 0;
		CASE(UVC_GET_INFO);
			net_buf_add_u8(buf, UVC_INFO_SUPPORTS_GET_SET);
			return 0;
		default:
			goto err_unsupported;
		}
		_dump_probe((void *)buf->data);
		return 0;
	}

err_unsupported:
	LOG_WRN("%s: unsupported bRequest=%02x wValue=0x%02x",
		__func__, setup->bRequest, setup->wValue);
	errno = -ENOTSUP;
	return 0;
}

static int _api_ctd(struct usbd_class_node *const c_nd,
			    const struct usb_setup_packet *const setup,
			    const struct net_buf *const buf)
{
	const struct device *dev = c_nd->data->priv;
	struct uvc_data *data = dev->data;

	LOG_DBG("%s: bRequest=%d wValue=%d wIndex=%d wLength=%d", __func__,
		setup->bRequest, setup->wValue, setup->wIndex, setup->wLength);

	switch (setup->wValue >> 8) {
	CASE(UVC_VS_PROBE_CONTROL);
		break;
	CASE(UVC_VS_COMMIT_CONTROL);
		_start_transfer(c_nd);
		break;
	default:
		LOG_WRN("%s: unknown wValue=%d", __func__, setup->wValue);
		errno = -ENOTSUP;
		return 0;
	}

	switch (setup->bRequest) {
	CASE(UVC_SET_CUR);
		if (buf->len != sizeof(data->probe)) {
			LOG_ERR("%s: invalid config buffer size", __func__);
			errno = -ENOTSUP;
			return 0;
		}
		_load_probe(&data->probe, (void *)buf->data);
		_dump_probe(&data->probe);
		break;
        default:
		LOG_ERR("%s: unknown bRequest=0x%02x", __func__, setup->bRequest);
		errno = -ENOTSUP;
		return 0;
        }

	return 0;
}

static int _api_init(struct usbd_class_node *const c_nd)
{
	struct uvc_desc *desc = c_nd->data->desc;
	const struct device *dev = c_nd->data->priv;
	struct uvc_data *data = (void *)dev->data;

	data->payload_header->bHeaderLength = sizeof(struct uvc_payload_header);
	data->payload_header->bmHeaderInfo |= UVC_BMHEADERINFO_HAS_PRESENTATIONTIME;
	data->payload_header->bmHeaderInfo |= UVC_BMHEADERINFO_HAS_SOURCECLOCK;
	data->payload_header->bmHeaderInfo |= UVC_BMHEADERINFO_END_OF_FRAME;
	desc->iad_uvc.bFirstInterface = desc->if0.bInterfaceNumber;
	memcpy(&data->probe, &_probe_def, sizeof(data->probe));
	return 0;
}

struct usbd_class_api _api = {
	.request = _api_request,
	.update = _api_update,
	.control_to_host = _api_cth,
	.control_to_dev = _api_ctd,
	.init = _api_init,
};

#define UVC_DEFINE_DESCRIPTOR(n)						\
static struct uvc_desc _desc_##n = {						\
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
	.if0_vctl_out = {							\
		.bLength = sizeof(struct uvc_output_terminal_descriptor),	\
		.bDescriptorType = UVC_CS_INTERFACE,				\
		.bDescriptorSubtype = UVC_VC_OUTPUT_TERMINAL,			\
		.bTerminalID = 2,						\
		.wTerminalType = sys_cpu_to_le16(UVC_TT_STREAMING),		\
		.bAssocTerminal = 0,						\
		.bSourceID = 1,							\
		.iTerminal = 0,							\
	},									\
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
	.if1 = {								\
		.bLength = sizeof(struct usb_if_descriptor),			\
		.bDescriptorType = USB_DESC_INTERFACE,				\
		.bInterfaceNumber = 1,						\
		.bAlternateSetting = 0,						\
		.bNumEndpoints = 1,						\
		.bInterfaceClass = USB_BCC_VIDEO,				\
		.bInterfaceSubClass = UVC_SC_VIDEOSTREAMING,			\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
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
	.if1_frame0 = {								\
		.bLength = sizeof(struct uvc_uncompressed_frame_descriptor),	\
		.bDescriptorType = UVC_CS_INTERFACE,				\
		.bDescriptorSubtype = UVC_VS_FRAME_UNCOMPRESSED,		\
		.bFrameIndex = 1,						\
		.bmCapabilities = 0x00,						\
		.wWidth = sys_cpu_to_le16(FRAME_WIDTH),				\
		.wHeight = sys_cpu_to_le16(FRAME_HEIGHT),			\
		.dwMinBitRate = sys_cpu_to_le32(15360000),			\
		.dwMaxBitRate = sys_cpu_to_le32(15360000),			\
		.dwMaxVideoFrameBufferSize = sys_cpu_to_le32(FRAME_SIZE),	\
		.dwDefaultFrameInterval = sys_cpu_to_le32(300*1000*1000 / 100), \
		.bFrameIntervalType = 1,					\
		.dwFrameInterval = { sys_cpu_to_le32(400000), },		\
	},									\
	.if1_in_ep = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x81,					\
		.bmAttributes = USB_EP_TYPE_BULK,				\
		.wMaxPacketSize = sys_cpu_to_le16(512),				\
		.bInterval = 0,							\
	},									\
	.if1_in_ep_comp = {							\
		.bLength = sizeof(struct usb_ep_companion_descriptor),		\
		.bDescriptorType = USB_DESC_ENDPOINT_COMPANION,			\
		.bMaxBurst = 0,							\
		.bmAttributes = 0,						\
		.wBytesPerInterval = sys_cpu_to_le16(0),			\
	},									\
	.nil_desc = {								\
		.bLength = 0,							\
		.bDescriptorType = 0,						\
	},									\
}

#define USBD_UVC_DT_DEVICE_DEFINE(n)						\
	BUILD_ASSERT(DT_INST_ON_BUS(n, usb),					\
	     "node " DT_NODE_PATH(DT_DRV_INST(n))				\
	     " is not assigned to a USB device controller");			\
										\
	UVC_DEFINE_DESCRIPTOR(n);						\
										\
	static struct usbd_class_data _class_data_##n = {			\
		.desc = (struct usb_desc_header *)&_desc_##n,			\
		.priv = (void *)DEVICE_DT_GET(DT_DRV_INST(n)),			\
	};									\
										\
	USBD_DEFINE_CLASS(uvc_##n, &_api, &_class_data_##n);			\
										\
	struct uvc_payload_header uvc_payload_header_##n;			\
										\
	static struct uvc_data _data_##n = {					\
		.payload_header = &uvc_payload_header_##n,			\
		.payload_addr = DT_INST_PROP(n, payload_addr),			\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, &_data_##n, NULL,			\
		POST_KERNEL, 50, &_api);

DT_INST_FOREACH_STATUS_OKAY(USBD_UVC_DT_DEVICE_DEFINE);
