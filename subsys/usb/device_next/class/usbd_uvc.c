/*
 * Copyright (c) 2024 tinyVision.ai Inc.
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
#include <zephyr/drivers/video.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbd_uvc, CONFIG_USBD_UVC_LOG_LEVEL);

#define DT_DRV_COMPAT zephyr_uvc

#define FRAME_WIDTH               1920
#define FRAME_HEIGHT              1080
#define BITS_PER_PIXEL            16
#define FRAME_SIZE                (FRAME_WIDTH * FRAME_HEIGHT * BITS_PER_PIXEL / 8)
#define TRANSFER_SIZE             (1024 * 1024)
#define EOF(header)               ((header).bmHeaderInfo & UVC_BMHEADERINFO_END_OF_FRAME)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) > 0);

NET_BUF_POOL_FIXED_DEFINE(uvc_pool_payload, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 100,
			  0, sizeof(struct udc_buf_info), NULL);

NET_BUF_POOL_FIXED_DEFINE(uvc_pool_header, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 100,
			  CONFIG_USBD_VIDEO_HEADER_SIZE, sizeof(struct udc_buf_info), NULL);

struct uvc_desc {
	struct usb_association_descriptor iad;

	struct usb_if_descriptor if0;
	struct uvc_interface_header_descriptor if0_vctl_hdr;
	struct uvc_output_terminal_descriptor if0_vctl_out;
	struct uvc_input_terminal_descriptor if0_vctl_in;

	struct usb_if_descriptor if1;
	struct uvc_stream_input_header_descriptor if1_stream_in;
	struct uvc_uncompressed_format_descriptor if1_format0;
	struct uvc_uncompressed_frame_descriptor if1_frame0;
	struct usb_ep_descriptor if1_fs_in_ep;
	struct usb_ep_descriptor if1_hs_in_ep;
	struct usb_ep_descriptor if1_ss_in_ep;
	struct usb_ep_companion if1_in_comp;

	struct usb_desc_header nil_desc;
} __packed;

struct uvc_data {
	/* Pointer to the the USBD class device */
	const struct usbd_class_data *const c_data;
	/* Pointer to the class interface descriptors */
	struct uvc_desc *const desc;
	const struct usb_desc_header **const fs_desc;
	const struct usb_desc_header **const hs_desc;
	const struct usb_desc_header **const ss_desc;
	/* Current and default values for the probe-commit controls */
	struct uvc_vs_probe_control default_probe __attribute__((aligned(4)));
	/* Current selected format out of the Frame and Format descriptors */
	uint8_t format_index;
	uint8_t frame_index;
	/* Video source, such as an image sensor */
	const struct device *const video_dev;
	/* UVC header passed just before the image data */
	struct uvc_payload_header payload_header;
	/* Work queue to submit more UVC frames */
	struct k_work work;
};

static uint8_t uvc_get_bulk_in(struct uvc_data *data)
{
	return data->desc->if1_stream_in.bEndpointAddress;
}

static int uvc_enqueue_buf(struct uvc_data *data, struct net_buf *buf, bool last_of_transfer)
{
	struct udc_buf_info *bi;
	int ret;

	LOG_DBG("Enqueueing buf=%p data=%p size=%u len=%u zlp=%u",
		buf, buf->data, buf->size, buf->len, last_of_transfer);

	bi = udc_get_buf_info(buf);
	__ASSERT_NO_MSG(bi != NULL);

	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->ep = uvc_get_bulk_in(data);
	bi->zlp = last_of_transfer;

	ret = usbd_ep_enqueue(data->c_data, buf);
	if (ret != 0) {
		LOG_WRN("Could not enqueue buf=%p", buf);
		net_buf_unref(buf);
		return ret;
	}

	return 0;
}

static int uvc_send_transfer(struct uvc_data *const data, size_t size)
{
	const struct device *vdev = data->video_dev;
	struct video_buffer vbuf = {.size = size};
	struct video_buffer *vbufp = NULL;
	struct net_buf *buf;
	int ret;

	LOG_INF("Fragment of %zu bytes bmHeaderInfo=0x%02x", size,
		data->payload_header.bmHeaderInfo);

	video_stream_start(vdev);

	/* Header */

	buf = net_buf_alloc(&uvc_pool_header, K_FOREVER);
	if (buf == NULL) {
		return -EAGAIN;
	}

	net_buf_add_mem(buf, &data->payload_header, 2);
	while (buf->len < buf->size) {
		net_buf_add_u8(buf, 0);
	}

	ret = uvc_enqueue_buf(data, buf, false);
	if (ret != 0) {
		return ret;
	}

	/* Payload */

	ret = video_enqueue(vdev, VIDEO_EP_IN, &vbuf);
	if (ret != 0) {
		LOG_ERR("failed to submit the buffer to the video system");
		return ret;
	}

	ret = video_dequeue(vdev, VIDEO_EP_IN, &vbufp, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("failed to fetch the filled buffer from the video system");
		return ret;
	}

	buf = net_buf_alloc_with_data(&uvc_pool_payload, vbufp->buffer, vbufp->bytesused, K_FOREVER);
	if (buf == NULL) {
		return -EAGAIN;
	}

	ret = uvc_enqueue_buf(data, buf, true);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static int uvc_send_frame(struct uvc_data *const data)
{
	struct uvc_desc *desc = data->desc;
	size_t size = desc->if1_frame0.dwMaxVideoFrameBufferSize;
	int ret;

	LOG_INF("Submitting a %ux%u frame with %u bits per pixel (%zu bytes)", FRAME_WIDTH,
		FRAME_HEIGHT, BITS_PER_PIXEL, FRAME_SIZE);

	/* Toggle the FrameId bit for every new frame. */
	data->payload_header.bmHeaderInfo ^= UVC_BMHEADERINFO_FRAMEID;

	/* Not setting the EOF flag until we reach the last transfer */
	data->payload_header.bmHeaderInfo &= ~UVC_BMHEADERINFO_END_OF_FRAME;
	for (; size > TRANSFER_SIZE; size -= TRANSFER_SIZE) {
		ret = uvc_send_transfer(data, TRANSFER_SIZE);
		if (ret != 0) {
			return ret;
		}
	}

	/* For the last transfer, we set the EOF flag */
	data->payload_header.bmHeaderInfo |= UVC_BMHEADERINFO_END_OF_FRAME;
	ret = uvc_send_transfer(data, size);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static void uvc_worker(struct k_work *work)
{
	uvc_send_frame(CONTAINER_OF(work, struct uvc_data, work));
}

static void uvc_probe_format_index(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	struct uvc_desc *desc = data->desc;

	switch (bRequest) {
	case UVC_GET_MIN:
		probe->bFormatIndex = 1;
		break;
	case UVC_GET_MAX:
		probe->bFormatIndex = desc->if1_stream_in.bNumFormats;
		break;
	case UVC_GET_RES:
		probe->bFormatIndex = 1;
		break;
	case UVC_GET_CUR:
		probe->bFormatIndex = data->format_index + 1;
		break;
	case UVC_SET_CUR:
		if (probe->bFormatIndex > desc->if1_stream_in.bNumFormats) {
			LOG_WRN("invalid format index");
			return;
		}
		if (probe->bFormatIndex > 0) {
			data->format_index = probe->bFormatIndex - 1;
		}
		break;
	}
}

static void uvc_probe_frame_index(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	struct uvc_desc *desc = data->desc;

	/* TODO handle the case where another format than 0 is selected */
	__ASSERT_NO_MSG(data->format_index == 0);

	switch (bRequest) {
	case UVC_GET_MIN:
		probe->bFrameIndex = 1;
		break;
	case UVC_GET_MAX:
		probe->bFrameIndex = desc->if1_format0.bNumFrameDescriptors;
		break;
	case UVC_GET_RES:
		probe->bFrameIndex = 1;
		break;
	case UVC_GET_CUR:
		probe->bFrameIndex = data->frame_index + 1;
		break;
	case UVC_SET_CUR:
		if (probe->bFrameIndex > desc->if1_stream_in.bNumFormats) {
			LOG_WRN("invalid format index");
			return;
		}
		if (probe->bFrameIndex > 0) {
			data->format_index = probe->bFrameIndex - 1;
		}
		break;
	}
}

static void uvc_probe_frame_interval(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	struct uvc_desc *desc = data->desc;

	switch (bRequest) {
	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_CUR:
		probe->dwFrameInterval = desc->if1_frame0.dwFrameInterval[0];
		break;
	case UVC_GET_RES:
		probe->dwFrameInterval = 1;
		break;
	case UVC_SET_CUR:
		/* TODO call the frame interval API on the video source once supported */
		break;
	}
}

static void uvc_probe_key_frame_rate(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->wKeyFrameRate = 0;
}

static void uvc_probe_p_frame_rate(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->wPFrameRate = 0;
}

static void uvc_probe_comp_quality(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->wCompQuality = 0;
}

static void uvc_probe_comp_window_size(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->wCompWindowSize = 0;
}

static void uvc_probe_delay(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	/* TODO devicetree */
	probe->wDelay = 1;
}

static void uvc_probe_max_video_frame_size(struct uvc_data *const data, uint8_t bRequest,
					   struct uvc_vs_probe_control *probe)
{
	struct uvc_desc *desc = data->desc;

	/* TODO walk through all bFormatIndex and bFrameIndex */
	__ASSERT_NO_MSG(desc->if1_stream_in.bNumFormats == 1);
	__ASSERT_NO_MSG(desc->if1_format0.bNumFrameDescriptors == 1);

	switch (bRequest) {
	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_CUR:
		probe->dwMaxVideoFrameSize = desc->if1_frame0.dwMaxVideoFrameBufferSize;
		break;
	case UVC_GET_RES:
		probe->dwMaxVideoFrameSize = 1;
		break;
	case UVC_SET_CUR:
		if (probe->dwMaxVideoFrameSize > 0 &&
		    probe->dwMaxVideoFrameSize != desc->if1_frame0.dwMaxVideoFrameBufferSize) {
			LOG_WRN("dwMaxVideoFrameSize is read-only");
		}
		break;
	}
}

static void uvc_probe_max_payload_size(struct uvc_data *const data, uint8_t bRequest,
				       struct uvc_vs_probe_control *probe)
{
	struct uvc_desc *desc = data->desc;

	/* TODO walk through all bFormatIndex and bFrameIndex */
	__ASSERT_NO_MSG(desc->if1_stream_in.bNumFormats == 1);
	__ASSERT_NO_MSG(desc->if1_format0.bNumFrameDescriptors == 1);

	switch (bRequest) {
	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_CUR:
		probe->dwMaxPayloadTransferSize = desc->if1_frame0.dwMaxVideoFrameBufferSize;
		break;
	case UVC_GET_RES:
		probe->dwMaxPayloadTransferSize = 1;
		break;
	case UVC_SET_CUR:
		if (probe->dwMaxPayloadTransferSize > 0 &&
		    probe->dwMaxPayloadTransferSize != desc->if1_frame0.dwMaxVideoFrameBufferSize) {
			LOG_WRN("dwPayloadTransferSize is read-only");
		}
		break;
	}
}

static void uvc_probe_clock_frequency(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	switch (bRequest) {
	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_CUR:
	case UVC_GET_RES:
		probe->dwClockFrequency = 1;
		break;
	case UVC_SET_CUR:
		if (probe->dwClockFrequency > 0) {
			LOG_WRN("dwClockFrequency is read-only");
		}
		break;
	}
}

static void uvc_probe_framing_info(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	/* Include Frame ID and EOF fields in the payload header */
	probe->bmFramingInfo = BIT(0) | BIT(1);
}

static void uvc_probe_prefered_version(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->bPreferedVersion = 1;
}

static void uvc_probe_min_version(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->bMaxVersion = 1;
}

static void uvc_probe_max_version(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->bMaxVersion = 1;
}

static void uvc_probe_usage(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->bUsage = 0;
}

static void uvc_probe_bit_depth_luma(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	/* TODO support other pixel format */
	switch (bRequest) {
	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_CUR:
		/* YUYV */
		probe->bBitDepthLuma = 16 - 8;
		break;
	case UVC_GET_RES:
		probe->bBitDepthLuma = 8;
		break;
	case UVC_SET_CUR:
		if (probe->bBitDepthLuma > 0) {
			LOG_WRN("does not support setting bBitDepthLuma");
		}
		break;
	}
}

static void uvc_probe_settings(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->bmSettings = 0;
}

static void uvc_probe_max_ref_frames(struct uvc_data *const data, uint8_t bRequest, struct uvc_vs_probe_control *probe)
{
	probe->bMaxNumberOfRefFramesPlus1 = 1;
}

static void uvc_probe_rate_control_modes(struct uvc_data *const data, uint8_t bRequest,
					 struct uvc_vs_probe_control *probe)
{
	probe->bmRateControlModes = 0;
}

static void uvc_probe_layout_per_stream(struct uvc_data *const data, uint8_t bRequest,
					 struct uvc_vs_probe_control *probe)
{
	probe->bmLayoutPerStream = 0;
}

static void uvc_dump_probe(char const *name, const struct uvc_vs_probe_control *probe)
{
	LOG_DBG("UVC_%s", name);
	LOG_DBG("- bmHint = 0x%04x", sys_le16_to_cpu(probe->bmHint));
	LOG_DBG("- bFormatIndex = %u", probe->bFormatIndex);
	LOG_DBG("- bFrameIndex = %u", probe->bFrameIndex);
	LOG_DBG("- dwFrameInterval = %u ns", sys_le32_to_cpu(probe->dwFrameInterval) * 100);
	LOG_DBG("- wKeyFrameRate = %u", sys_le16_to_cpu(probe->wKeyFrameRate));
	LOG_DBG("- wPFrameRate = %u", probe->wPFrameRate);
	LOG_DBG("- wCompQuality = %u", probe->wCompQuality);
	LOG_DBG("- wCompWindowSize = %u", probe->wCompWindowSize);
	LOG_DBG("- wDelay = %u ms", sys_le16_to_cpu(probe->wDelay));
	LOG_DBG("- dwMaxVideoFrameSize = %u", sys_le32_to_cpu(probe->dwMaxVideoFrameSize));
	LOG_DBG("- dwMaxPayloadTransferSize = %u", sys_le32_to_cpu(probe->dwMaxPayloadTransferSize));
	LOG_DBG("- dwClockFrequency = %u Hz", sys_le32_to_cpu(probe->dwClockFrequency));
	LOG_DBG("- bmFramingInfo = 0x%02x", probe->bmFramingInfo);
	LOG_DBG("- bPreferedVersion = %u", probe->bPreferedVersion);
	LOG_DBG("- bMinVersion = %u", probe->bMinVersion);
	LOG_DBG("- bMaxVersion = %u", probe->bMaxVersion);
	LOG_DBG("- bUsage = %u", probe->bUsage);
	LOG_DBG("- bBitDepthLuma = %u", probe->bBitDepthLuma + 8);
	LOG_DBG("- bmSettings = %u", probe->bmSettings);
	LOG_DBG("- bMaxNumberOfRefFramesPlus1 = %u", probe->bMaxNumberOfRefFramesPlus1);
	LOG_DBG("- bmRateControlModes = %u", probe->bmRateControlModes);
	LOG_DBG("- bmLayoutPerStream = 0x%08llx", probe->bmLayoutPerStream);
}

static int uvc_probe(struct uvc_data *const data, uint16_t bRequest,
		     struct uvc_vs_probe_control *probe)
{
	LOG_DBG("UVC control: probe");

	switch (bRequest) {
	case UVC_GET_DEF:
		memcpy(probe, &data->default_probe, sizeof(*probe));
		break;
	case UVC_SET_CUR:
		uvc_dump_probe("SET_CUR", probe);
		goto action;
	case UVC_GET_CUR:
		uvc_dump_probe("GET_CUR", probe);
		goto action;
	case UVC_GET_MIN:
		uvc_dump_probe("GET_MIN", probe);
		goto action;
	case UVC_GET_MAX:
		uvc_dump_probe("GET_MAX", probe);
		goto action;
	case UVC_GET_RES:
		uvc_dump_probe("SET_RES", probe);
		goto action;
	action:
		/* TODO use bmHint to choose in which order configure the fields */
		uvc_probe_format_index(data, bRequest, probe);
		uvc_probe_frame_index(data, bRequest, probe);
		uvc_probe_frame_interval(data, bRequest, probe);
		uvc_probe_key_frame_rate(data, bRequest, probe);
		uvc_probe_p_frame_rate(data, bRequest, probe);
		uvc_probe_comp_quality(data, bRequest, probe);
		uvc_probe_comp_window_size(data, bRequest, probe);
		uvc_probe_delay(data, bRequest, probe);
		uvc_probe_max_video_frame_size(data, bRequest, probe);
		uvc_probe_max_payload_size(data, bRequest, probe);
		uvc_probe_clock_frequency(data, bRequest, probe);
		uvc_probe_framing_info(data, bRequest, probe);
		uvc_probe_prefered_version(data, bRequest, probe);
		uvc_probe_min_version(data, bRequest, probe);
		uvc_probe_max_version(data, bRequest, probe);
		uvc_probe_usage(data, bRequest, probe);
		uvc_probe_bit_depth_luma(data, bRequest, probe);
		uvc_probe_settings(data, bRequest, probe);
		uvc_probe_max_ref_frames(data, bRequest, probe);
		uvc_probe_rate_control_modes(data, bRequest, probe);
		uvc_probe_layout_per_stream(data, bRequest, probe);
		break;
	default:
		__ASSERT_NO_MSG(false);
	}
	return 0;
}

static int uvc_commit(struct uvc_data *const data, uint16_t bRequest,
		      struct uvc_vs_probe_control *probe)
{
	struct uvc_desc *desc = data->desc;
	const struct device *vdev = data->video_dev;
	struct video_format fmt = {0};
	int ret;

	LOG_DBG("UVC control: commit");

	switch (bRequest) {
	case UVC_GET_CUR:
		uvc_probe(data, bRequest, probe);
		break;
	case UVC_SET_CUR:
		uvc_probe(data, bRequest, probe);

		/* TODO support custom/different formats */
		(void)data->format_index;
		(void)data->frame_index;

		fmt.pixelformat = VIDEO_PIX_FMT_YUYV;
		fmt.width = desc->if1_frame0.wWidth;
		fmt.height = desc->if1_frame0.wHeight;
		fmt.pitch = 0;
		ret = video_set_format(vdev, VIDEO_EP_IN, &fmt);
		if (ret < 0) {
			LOG_WRN("unable to set video format (err %u)", ret);
			errno = ret;
			return 0;
		}

		LOG_INF("Starting UVC transfer");
		k_work_submit(&data->work);
		break;
	default:
		LOG_ERR("invalid bRequest (%u)", bRequest);
		errno = -EINVAL;
		return 0;
	}
	return 0;
}

static int uvc_probe_or_commit(struct uvc_data *const data,
			       const struct usb_setup_packet *const setup,
			       struct net_buf *const buf)
{
	struct uvc_vs_probe_control probe = {0};

	switch (setup->bRequest) {
	case UVC_GET_LEN:
		net_buf_add_le16(buf, sizeof(probe));
		return 0;
	case UVC_GET_INFO:
		/* As defined by UVC 1.5 Table 4-76 */
		net_buf_add_u8(buf, BIT(0) | BIT(1));
		return 0;
	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_DEF:
	case UVC_GET_RES:
	case UVC_GET_CUR:
		if (net_buf_add(buf, sizeof(probe)) == NULL) {
			return -ENOMEM;
		}
		break;
	case UVC_SET_CUR:
		if (buf->len != sizeof(probe)) {
			LOG_ERR("Invalid wLength=%u for Probe or Commit", setup->wLength);
			errno = -EINVAL;
			return 0;
		}
		break;
	default:
		LOG_ERR("Invalid bRequest (%u) for Probe or Commit", setup->bRequest);
		errno = -EINVAL;
		return 0;
	}

	/* All remaining request work on a struct uvc_vs_probe_control */
	if (setup->wLength != sizeof(probe)) {
		LOG_ERR("Invalid wLength=%u for Probe or Commit", setup->wLength);
		errno = -EINVAL;
		return 0;
	}

	switch (setup->wValue >> 8) {
	case UVC_VS_PROBE_CONTROL:
		return uvc_probe(data, setup->bRequest, (void *)buf->data);
	case UVC_VS_COMMIT_CONTROL:
		return uvc_commit(data, setup->bRequest, (void *)buf->data);
	default:
		__ASSERT_NO_MSG(false);
	}
}

static int uvc_control(struct usbd_class_data *c_data, const struct usb_setup_packet *const setup,
		       struct net_buf *const buf)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	switch (setup->wValue >> 8) {
	case UVC_VS_PROBE_CONTROL:
	case UVC_VS_COMMIT_CONTROL:
		return uvc_probe_or_commit(data, setup, buf);
	default:
		LOG_WRN("unsupported wValue (%u)", setup->wValue);
		errno = -ENOTSUP;
		return 0;
	}
}

static int uvc_control_to_host(struct usbd_class_data *const c_data,
			       const struct usb_setup_packet *const setup,
			       struct net_buf *const buf)
{
	return uvc_control(c_data, setup, buf);
}

static int uvc_control_to_dev(struct usbd_class_data *const c_data,
			      const struct usb_setup_packet *const setup,
			      const struct net_buf *const buf)
{
	return uvc_control(c_data, setup, (struct net_buf *const)buf);
}

static int uvc_request(struct usbd_class_data *const c_data, struct net_buf *buf, int err)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;
	struct udc_buf_info bi = *udc_get_buf_info(buf);

	LOG_DBG("UVC transfer done for ep=0x%02x buf=%p", bi.ep, buf);
	net_buf_unref(buf);
	__ASSERT_NO_MSG(bi.ep == uvc_get_bulk_in(data));

	if (err) {
		return err;
	}

	/* If this is a completion for the video stream */
	if (bi.ep == uvc_get_bulk_in(data)) {
		/* Only for the last buffer of the transfer and last transfer of the frame */
		if (bi.zlp && EOF(data->payload_header)) {
			k_work_submit(&data->work);
		}
	}
	return 0;
}

static void uvc_update(struct usbd_class_data *const c_data, uint8_t iface, uint8_t alternate)
{
	LOG_DBG("UVC Update");
}

static int uvc_init(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	LOG_INF("Initializing UVC class");

	/* Prepare the payload header fields that are constant over time */
	data->payload_header.bHeaderLength = sizeof(uint64_t); // TODO sizeof(anstruct uvc_payload_header);

	/* The default probe is the current probe at startup */
	uvc_probe(data, UVC_GET_CUR, &data->default_probe);

	/* Worker to start the UVC feed */
	k_work_init(&data->work, uvc_worker);

	return 0;
}

static void *uvc_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;
	struct uvc_desc *desc = data->desc;

	/* The descriptors need some adjustments */
	desc->iad.bFirstInterface = desc->if0.bInterfaceNumber;
	desc->if0_vctl_hdr.baInterfaceNr = desc->if1.bInterfaceNumber;

	switch (speed) {
	case USBD_SPEED_SS:
		desc->if1_stream_in.bEndpointAddress = desc->if1_ss_in_ep.bEndpointAddress;
		return data->ss_desc;
	case USBD_SPEED_HS:
		desc->if1_stream_in.bEndpointAddress = desc->if1_hs_in_ep.bEndpointAddress;
		return data->hs_desc;
	default:
		desc->if1_stream_in.bEndpointAddress = desc->if1_fs_in_ep.bEndpointAddress;
		return data->fs_desc;
	}
}

struct usbd_class_api uvc_api = {
	.request = uvc_request,
	.update = uvc_update,
	.control_to_host = uvc_control_to_host,
	.control_to_dev = uvc_control_to_dev,
	.init = uvc_init,
	.get_desc = uvc_get_desc,
};

#define DEFINE_UVC_DESCRIPTOR(n)                                                                   \
                                                                                                   \
	static struct uvc_desc uvc_desc_##n = {                                                    \
		.iad =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_association_descriptor),              \
				.bDescriptorType = USB_DESC_INTERFACE_ASSOC,                       \
				.bFirstInterface = 0,                                              \
				.bInterfaceCount = 2,                                              \
				.bFunctionClass = USB_BCC_VIDEO,                                   \
				.bFunctionSubClass = UVC_SC_VIDEO_INTERFACE_COLLECITON,            \
				.bFunctionProtocol = UVC_PC_PROTOCOL_UNDEFINED,                    \
				.iFunction = 0,                                                    \
			},                                                                         \
		.if0 =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_if_descriptor),                       \
				.bDescriptorType = USB_DESC_INTERFACE,                             \
				.bInterfaceNumber = 0,                                             \
				.bAlternateSetting = 0,                                            \
				.bNumEndpoints = 0,                                                \
				.bInterfaceClass = USB_BCC_VIDEO,                                  \
				.bInterfaceSubClass = UVC_SC_VIDEOCONTROL,                         \
				.bInterfaceProtocol = 0,                                           \
				.iInterface = 0,                                                   \
			},                                                                         \
		.if0_vctl_hdr =                                                                    \
			{                                                                          \
				.bLength = sizeof(struct uvc_interface_header_descriptor),         \
				.bDescriptorType = UVC_CS_INTERFACE,                               \
				.bDescriptorSubtype = UVC_VC_HEADER,                               \
				.bcdUVC = 0x0150,                                                  \
				.wTotalLength = sys_cpu_to_le16(                                   \
					sizeof(struct uvc_interface_header_descriptor) +           \
					sizeof(struct uvc_input_terminal_descriptor) +             \
					sizeof(struct uvc_output_terminal_descriptor)),            \
				.dwClockFrequency = 30000000,                                      \
				.bInCollection = 1,                                                \
				.baInterfaceNr = 1,                                                \
			},                                                                         \
		.if0_vctl_out =                                                                    \
			{                                                                          \
				.bLength = sizeof(struct uvc_output_terminal_descriptor),          \
				.bDescriptorType = UVC_CS_INTERFACE,                               \
				.bDescriptorSubtype = UVC_VC_OUTPUT_TERMINAL,                      \
				.bTerminalID = 2,                                                  \
				.wTerminalType = sys_cpu_to_le16(UVC_TT_STREAMING),                \
				.bAssocTerminal = 0,                                               \
				.bSourceID = 1,                                                    \
				.iTerminal = 0,                                                    \
			},                                                                         \
		.if0_vctl_in =                                                                     \
			{                                                                          \
				.bLength = sizeof(struct uvc_input_terminal_descriptor),           \
				.bDescriptorType = UVC_CS_INTERFACE,                               \
				.bDescriptorSubtype = UVC_VC_INPUT_TERMINAL,                       \
				.bTerminalID = 1,                                                  \
				.wTerminalType = sys_cpu_to_le16(UVC_ITT_CAMERA),                  \
				.bAssocTerminal = 0,                                               \
				.iTerminal = 0,                                                    \
				.wObjectiveFocalLengthMin = sys_cpu_to_le16(0),                    \
				.wObjectiveFocalLengthMax = sys_cpu_to_le16(0),                    \
				.wOcularFocalLength = sys_cpu_to_le16(0),                          \
				.bControlSize = 3,                                                 \
				.bmControls = {0x00, 0x00, 0x00},                                  \
			},                                                                         \
		.if1 =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_if_descriptor),                       \
				.bDescriptorType = USB_DESC_INTERFACE,                             \
				.bInterfaceNumber = 1,                                             \
				.bAlternateSetting = 0,                                            \
				.bNumEndpoints = 1,                                                \
				.bInterfaceClass = USB_BCC_VIDEO,                                  \
				.bInterfaceSubClass = UVC_SC_VIDEOSTREAMING,                       \
				.bInterfaceProtocol = 0,                                           \
				.iInterface = 0,                                                   \
			},                                                                         \
		.if1_stream_in =                                                                   \
			{                                                                          \
				.bLength = sizeof(struct uvc_stream_input_header_descriptor),      \
				.bDescriptorType = UVC_CS_INTERFACE,                               \
				.bDescriptorSubtype = UVC_VS_INPUT_HEADER,                         \
				.bNumFormats = 1,                                                  \
				.wTotalLength = sys_cpu_to_le16(                                   \
					sizeof(struct uvc_stream_input_header_descriptor) +        \
					sizeof(struct uvc_uncompressed_format_descriptor) +        \
					sizeof(struct uvc_uncompressed_frame_descriptor)),         \
				.bEndpointAddress = 0x81,                                          \
				.bmInfo = 0,                                                       \
				.bTerminalLink = 2,                                                \
				.bStillCaptureMethod = 0,                                          \
				.bTriggerSupport = 0,                                              \
				.bTriggerUsage = 0,                                                \
				.bControlSize = 1,                                                 \
				.bmaControls = {0x00},                                             \
			},                                                                         \
		.if1_format0 =                                                                     \
			{                                                                          \
				.bLength = sizeof(struct uvc_uncompressed_format_descriptor),      \
				.bDescriptorType = UVC_CS_INTERFACE,                               \
				.bDescriptorSubtype = UVC_VS_FORMAT_UNCOMPRESSED,                  \
				.bFormatIndex = 1,                                                 \
				.bNumFrameDescriptors = 1,                                         \
				.guidFormat = UVC_GUID_UNCOMPRESSED_YUY2,                          \
				.bBitsPerPixel = 16,                                               \
				.bDefaultFrameIndex = 1,                                           \
				.bAspectRatioX = 0,                                                \
				.bAspectRatioY = 0,                                                \
				.bmInterlaceFlags = 0x00,                                          \
				.bCopyProtect = 0,                                                 \
			},                                                                         \
		.if1_frame0 =                                                                      \
			{                                                                          \
				.bLength = sizeof(struct uvc_uncompressed_frame_descriptor),       \
				.bDescriptorType = UVC_CS_INTERFACE,                               \
				.bDescriptorSubtype = UVC_VS_FRAME_UNCOMPRESSED,                   \
				.bFrameIndex = 1,                                                  \
				.bmCapabilities = 0x00,                                            \
				.wWidth = sys_cpu_to_le16(FRAME_WIDTH),                            \
				.wHeight = sys_cpu_to_le16(FRAME_HEIGHT),                          \
				.dwMinBitRate = sys_cpu_to_le32(15360000),                         \
				.dwMaxBitRate = sys_cpu_to_le32(15360000),                         \
				.dwMaxVideoFrameBufferSize = sys_cpu_to_le32(FRAME_SIZE),          \
				.dwDefaultFrameInterval = sys_cpu_to_le32(300 * 1000000 / 100),    \
				.bFrameIntervalType = 1,                                           \
				.dwFrameInterval =                                                 \
					{                                                          \
						sys_cpu_to_le32(1000000),                          \
					},                                                         \
			},                                                                         \
		.if1_fs_in_ep =                                                                    \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = 0x81,                                          \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64),                             \
				.bInterval = 0,                                                    \
			},                                                                         \
		.if1_hs_in_ep =                                                                    \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = 0x81,                                          \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(512),                            \
				.bInterval = 0,                                                    \
			},                                                                         \
		.if1_ss_in_ep =                                                                    \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = 0x81,                                          \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(1024),                           \
				.bInterval = 0,                                                    \
			},                                                                         \
		.if1_in_comp =                                                                     \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_companion),                        \
				.bDescriptorType = USB_DESC_ENDPOINT_COMPANION,                    \
				.bMaxBurst = 0,                                                    \
				.bmAttributes = 0,                                                 \
				.wBytesPerInterval = sys_cpu_to_le16(0),                           \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	const static struct usb_desc_header *uvc_fs_desc_##n[] = {                                 \
		(struct usb_desc_header *)&uvc_desc_##n.iad,                                       \
		(struct usb_desc_header *)&uvc_desc_##n.if0,                                       \
		(struct usb_desc_header *)&uvc_desc_##n.if0_vctl_hdr,                              \
		(struct usb_desc_header *)&uvc_desc_##n.if0_vctl_out,                              \
		(struct usb_desc_header *)&uvc_desc_##n.if0_vctl_in,                               \
		(struct usb_desc_header *)&uvc_desc_##n.if1,                                       \
		(struct usb_desc_header *)&uvc_desc_##n.if1_stream_in,                             \
		(struct usb_desc_header *)&uvc_desc_##n.if1_format0,                               \
		(struct usb_desc_header *)&uvc_desc_##n.if1_frame0,                                \
		(struct usb_desc_header *)&uvc_desc_##n.if1_fs_in_ep,                              \
		(struct usb_desc_header *)&uvc_desc_##n.nil_desc,                                  \
	};                                                                                         \
                                                                                                   \
	const static struct usb_desc_header *uvc_hs_desc_##n[] = {                                 \
		(struct usb_desc_header *)&uvc_desc_##n.iad,                                       \
		(struct usb_desc_header *)&uvc_desc_##n.if0,                                       \
		(struct usb_desc_header *)&uvc_desc_##n.if0_vctl_hdr,                              \
		(struct usb_desc_header *)&uvc_desc_##n.if0_vctl_out,                              \
		(struct usb_desc_header *)&uvc_desc_##n.if0_vctl_in,                               \
		(struct usb_desc_header *)&uvc_desc_##n.if1,                                       \
		(struct usb_desc_header *)&uvc_desc_##n.if1_stream_in,                             \
		(struct usb_desc_header *)&uvc_desc_##n.if1_format0,                               \
		(struct usb_desc_header *)&uvc_desc_##n.if1_frame0,                                \
		(struct usb_desc_header *)&uvc_desc_##n.if1_hs_in_ep,                              \
		(struct usb_desc_header *)&uvc_desc_##n.nil_desc,                                  \
	};                                                                                         \
                                                                                                   \
	const static struct usb_desc_header *uvc_ss_desc_##n[] = {                                 \
		(struct usb_desc_header *)&uvc_desc_##n.iad,                                       \
		(struct usb_desc_header *)&uvc_desc_##n.if0,                                       \
		(struct usb_desc_header *)&uvc_desc_##n.if0_vctl_hdr,                              \
		(struct usb_desc_header *)&uvc_desc_##n.if0_vctl_out,                              \
		(struct usb_desc_header *)&uvc_desc_##n.if0_vctl_in,                               \
		(struct usb_desc_header *)&uvc_desc_##n.if1,                                       \
		(struct usb_desc_header *)&uvc_desc_##n.if1_stream_in,                             \
		(struct usb_desc_header *)&uvc_desc_##n.if1_format0,                               \
		(struct usb_desc_header *)&uvc_desc_##n.if1_frame0,                                \
		(struct usb_desc_header *)&uvc_desc_##n.if1_ss_in_ep,                              \
		(struct usb_desc_header *)&uvc_desc_##n.if1_in_comp,                               \
		(struct usb_desc_header *)&uvc_desc_##n.nil_desc,                                  \
	}

#define DEFINE_UVC_DEVICE(n)                                                                       \
	BUILD_ASSERT(DT_INST_ON_BUS(n, usb),                                                       \
		     "node " DT_NODE_PATH(DT_DRV_INST(n)) " is not"                                \
							  " assigned to a USB device controller"); \
                                                                                                   \
	DEFINE_UVC_DESCRIPTOR(n);                                                                  \
                                                                                                   \
	USBD_DEFINE_CLASS(uvc_##n, &uvc_api, (void *)DEVICE_DT_GET(DT_DRV_INST(n)), NULL);         \
                                                                                                   \
	static struct uvc_data uvc_data_##n = {                                                    \
		.c_data = &uvc_##n,                                                                \
		.desc = &uvc_desc_##n,                                                             \
		.fs_desc = uvc_fs_desc_##n,                                                        \
		.hs_desc = uvc_hs_desc_##n,                                                        \
		.ss_desc = uvc_ss_desc_##n,                                                        \
		.video_dev = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(n), source)),                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, &uvc_data_##n, NULL, POST_KERNEL, 50, &uvc_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_UVC_DEVICE)
