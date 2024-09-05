/*
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_uvc

#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/drivers/video.h>
#include <zephyr/logging/log.h>

#include "usbd_uvc_macros.h"

LOG_MODULE_REGISTER(usbd_uvc, CONFIG_USBD_UVC_LOG_LEVEL);

#define UVC_CLASS_ENABLED			0
#define UVC_CLASS_READY				1

/* Video Class-Specific Request Codes */
#define UVC_RC_UNDEFINED			0x00
#define UVC_SET_CUR				0x01
#define UVC_SET_CUR_ALL				0x11
#define UVC_GET_CUR				0x81
#define UVC_GET_MIN				0x82
#define UVC_GET_MAX				0x83
#define UVC_GET_RES				0x84
#define UVC_GET_LEN				0x85
#define UVC_GET_INFO				0x86
#define UVC_GET_DEF				0x87
#define UVC_GET_CUR_ALL				0x91
#define UVC_GET_MIN_ALL				0x92
#define UVC_GET_MAX_ALL				0x93
#define UVC_GET_RES_ALL				0x94
#define UVC_GET_DEF_ALL				0x97

/* Video Probe and Commit Controls */
struct uvc_vs_probe_control {
	uint16_t bmHint;
	uint8_t bFormatIndex;
	uint8_t bFrameIndex;
	uint32_t dwFrameInterval;
	uint16_t wKeyFrameRate;
	uint16_t wPFrameRate;
	uint16_t wCompQuality;
	uint16_t wCompWindowSize;
	uint16_t wDelay;
	uint32_t dwMaxVideoFrameSize;
	uint32_t dwMaxPayloadTransferSize;
	uint32_t dwClockFrequency;
	uint8_t bmFramingInfo;
#define UVC_BMFRAMING_INFO_FID (1 << 0)
#define UVC_BMFRAMING_INFO_EOF (1 << 1)
#define UVC_BMFRAMING_INFO_EOS (1 << 2)
	uint8_t bPreferedVersion;
	uint8_t bMinVersion;
	uint8_t bMaxVersion;
	uint8_t bUsage;
	uint8_t bBitDepthLuma;
	uint8_t bmSettings;
	uint8_t bMaxNumberOfRefFramesPlus1;
	uint16_t bmRateControlModes;
	uint64_t bmLayoutPerStream;
} __packed;

/* Video Control Interface Status Packet Format */
struct uvc_control_status_packet {
	uint8_t bStatusType;
	uint8_t bOriginator;
	uint8_t bEvent;
	uint8_t bSelector;
	uint8_t bAttribute;
	uint8_t bValue;
} __packed;

/* Video Streaming Interface Status Packet Format */
struct uvc_stream_status_packet {
	uint8_t bStatusType;
	uint8_t bOriginator;
	uint8_t bEvent;
	uint8_t bValue;
} __packed;

/* Video and Still Image Payload Headers */
struct uvc_payload_header {
	uint8_t bHeaderLength;
	uint8_t bmHeaderInfo;
	uint32_t dwPresentationTime;	/* optional */
	uint32_t scrSourceClockSTC;	/* optional */
	uint16_t scrSourceClockSOF;	/* optional */
} __packed;

struct uvc_format {
	/* USB-side data from the descriptor */
	uint8_t bFormatIndex;
	uint8_t bFrameIndex;
	uint8_t bDescriptorSubtype;
	uint8_t bits_per_pixel;
	uint32_t frame_interval;	/* see #72254 */
};

struct uvc_data {
	/* USBD class structure */
	const struct usbd_class_data *c_data;
	/* USBD class state */
	atomic_t state;
	/* UVC worker to process the queue */
	struct k_work work;
	/* UVC and Video format and lookup tables */
	int format_id;
	const struct video_format_cap *caps;
	const struct uvc_format *formats;
	/* UVC Descriptors */
	struct usb_desc_header *const *fs_desc;
	struct usb_desc_header *const *hs_desc;
	struct usb_desc_header *const *ss_desc;
	/* UVC Fields that need to be accessed */
	uint8_t *desc_iad_ifnum;
	uint8_t *desc_if_vc_ifnum;
	uint8_t *desc_if_vc_header_ifnum;
	uint8_t *desc_if_vs_ifnum;
	uint8_t *desc_if_vs_header_epaddr;
	uint8_t *fs_desc_ep_epaddr;
	uint8_t *hs_desc_ep_epaddr;
	uint8_t *ss_desc_ep_epaddr;
	/* UVC probe-commit control default values */
	struct uvc_vs_probe_control default_probe;
	/* UVC payload header, passed just before the image data */
	struct uvc_payload_header payload_header;
	/* Video device controlled by the host via UVC */
	const struct device *source_dev;
	/* Video FIFOs for submission (in) and completion (out) queue */
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
};

struct uvc_buf_info {
	struct udc_buf_info udc;
	struct video_buffer *vbuf;
} __packed;

NET_BUF_POOL_VAR_DEFINE(uvc_pool_header, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 10,
			CONFIG_USBD_VIDEO_HEADER_SIZE * 10, sizeof(struct uvc_buf_info), NULL);

NET_BUF_POOL_FIXED_DEFINE(uvc_pool_payload, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 10, 0,
			  sizeof(struct uvc_buf_info), NULL);

static int uvc_get_format(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt);

static uint8_t uvc_get_bulk_in(struct uvc_data *data)
{
	switch (usbd_bus_speed(usbd_class_get_ctx(data->c_data))) {
	case USBD_SPEED_FS:
		return *data->fs_desc_ep_epaddr;
	case USBD_SPEED_HS:
		return *data->hs_desc_ep_epaddr;
	case USBD_SPEED_SS:
		return *data->ss_desc_ep_epaddr;
	default:
		__ASSERT_NO_MSG(false);
		return 0;
	}
}

static uint32_t uvc_get_max_frame_size(struct uvc_data *data, int id)
{
	const struct video_format_cap *cap = &data->caps[id];
	const struct uvc_format *ufmt = &data->formats[id];

	return cap->width_max * cap->width_min * ufmt->bits_per_pixel;
}

static void uvc_probe_format_index(struct uvc_data *const data, uint8_t bRequest,
				   struct uvc_vs_probe_control *probe)
{
	switch (bRequest) {
	case UVC_GET_MIN:
		probe->bFormatIndex = UINT8_MAX;
		for (const struct uvc_format *fmt = data->formats; fmt->bFormatIndex; fmt++) {
			probe->bFormatIndex = MIN(fmt->bFormatIndex, probe->bFormatIndex);
		}
		break;
	case UVC_GET_MAX:
		probe->bFormatIndex = 0;
		for (const struct uvc_format *fmt = data->formats; fmt->bFormatIndex; fmt++) {
			probe->bFormatIndex = MAX(fmt->bFormatIndex, probe->bFormatIndex);
		}
		break;
	case UVC_GET_RES:
		probe->bFormatIndex = 1;
		break;
	case UVC_GET_CUR:
		probe->bFormatIndex = data->formats[data->format_id].bFormatIndex;
		break;
	case UVC_SET_CUR:
		for (size_t i = 0; data->formats[i].bFormatIndex; i++) {
			if (data->formats[i].bFormatIndex == probe->bFormatIndex) {
				data->format_id = i;
				return;
			}
		}
		LOG_WRN("probe: format index not found");
		errno = EINVAL;
	}
}

static void uvc_probe_frame_index(struct uvc_data *const data, uint8_t bRequest,
				  struct uvc_vs_probe_control *probe)
{
	const struct uvc_format *cur = &data->formats[data->format_id];

	switch (bRequest) {
	case UVC_GET_MIN:
		probe->bFrameIndex = UINT8_MAX;
		for (const struct uvc_format *fmt = data->formats; fmt->bFormatIndex; fmt++) {
			if (fmt->bFormatIndex == cur->bFormatIndex) {
				probe->bFrameIndex = MIN(fmt->bFrameIndex, probe->bFrameIndex);
			}
		}
		break;
	case UVC_GET_MAX:
		probe->bFrameIndex = 0;
		for (const struct uvc_format *fmt = data->formats; fmt->bFormatIndex; fmt++) {
			if (fmt->bFormatIndex == cur->bFormatIndex) {
				probe->bFrameIndex = MAX(fmt->bFrameIndex, probe->bFrameIndex);
			}
		}
		break;
	case UVC_GET_RES:
		probe->bFrameIndex = 1;
		break;
	case UVC_GET_CUR:
		probe->bFrameIndex = cur->bFrameIndex;
		break;
	case UVC_SET_CUR:
		for (size_t i = 0; data->formats[i].bFormatIndex; i++) {
			const struct uvc_format *fmt = &data->formats[i];

			if (fmt->bFrameIndex == cur->bFormatIndex &&
			    fmt->bFormatIndex == probe->bFormatIndex) {
				data->format_id = i;
				return;
			}
		}
		LOG_WRN("probe: frame index not found");
		errno = EINVAL;
	}
}

static void uvc_probe_frame_interval(struct uvc_data *const data, uint8_t bRequest,
				     struct uvc_vs_probe_control *probe)
{
	switch (bRequest) {
	case UVC_GET_MIN:
	case UVC_GET_MAX:
		/* TODO call the frame interval API on the video source once supported */
		probe->dwFrameInterval = data->formats[data->format_id].frame_interval;
		break;
	case UVC_GET_RES:
		probe->dwFrameInterval = 1;
		break;
	case UVC_SET_CUR:
		/* TODO call the frame interval API on the video source once supported */
		errno = ENOTSUP;
		break;
	}
}

static void uvc_probe_key_frame_rate(struct uvc_data *const data, uint8_t bRequest,
				     struct uvc_vs_probe_control *probe)
{
	probe->wKeyFrameRate = 0;
}

static void uvc_probe_p_frame_rate(struct uvc_data *const data, uint8_t bRequest,
				   struct uvc_vs_probe_control *probe)
{
	probe->wPFrameRate = 0;
}

static void uvc_probe_comp_quality(struct uvc_data *const data, uint8_t bRequest,
				   struct uvc_vs_probe_control *probe)
{
	probe->wCompQuality = 0;
}

static void uvc_probe_comp_window_size(struct uvc_data *const data, uint8_t bRequest,
				       struct uvc_vs_probe_control *probe)
{
	probe->wCompWindowSize = 0;
}

static void uvc_probe_delay(struct uvc_data *const data, uint8_t bRequest,
			    struct uvc_vs_probe_control *probe)
{
	/* TODO devicetree */
	probe->wDelay = 1;
}

static void uvc_probe_max_video_frame_size(struct uvc_data *const data, uint8_t bRequest,
					   struct uvc_vs_probe_control *probe)
{
	uint32_t max_frame_size = uvc_get_max_frame_size(data, data->format_id);

	switch (bRequest) {
	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_CUR:
		probe->dwMaxVideoFrameSize = max_frame_size;
		break;
	case UVC_GET_RES:
		probe->dwMaxVideoFrameSize = 1;
		break;
	case UVC_SET_CUR:
		if (probe->dwMaxVideoFrameSize > 0 &&
		    probe->dwMaxVideoFrameSize != max_frame_size) {
			LOG_WRN("probe: dwMaxVideoFrameSize is read-only");
		}
		break;
	}
}

static void uvc_probe_max_payload_size(struct uvc_data *const data, uint8_t bRequest,
				       struct uvc_vs_probe_control *probe)
{
	uint32_t max_frame_size = uvc_get_max_frame_size(data, data->format_id);

	switch (bRequest) {
	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_CUR:
		probe->dwMaxPayloadTransferSize = max_frame_size;
		break;
	case UVC_GET_RES:
		probe->dwMaxPayloadTransferSize = 1;
		break;
	case UVC_SET_CUR:
		if (probe->dwMaxPayloadTransferSize > 0 &&
		    probe->dwMaxPayloadTransferSize != max_frame_size) {
			LOG_WRN("probe: dwPayloadTransferSize is read-only");
		}
		break;
	}
}

static void uvc_probe_clock_frequency(struct uvc_data *const data, uint8_t bRequest,
				      struct uvc_vs_probe_control *probe)
{
	switch (bRequest) {
	case UVC_GET_MIN:
	case UVC_GET_MAX:
	case UVC_GET_CUR:
	case UVC_GET_RES:
		probe->dwClockFrequency = 1;
		break;
	case UVC_SET_CUR:
		if (probe->dwClockFrequency > 1) {
			LOG_WRN("probe: dwClockFrequency is read-only");
		}
		break;
	}
}

static void uvc_probe_framing_info(struct uvc_data *const data, uint8_t bRequest,
				   struct uvc_vs_probe_control *probe)
{
	/* Include Frame ID and EOF fields in the payload header */
	probe->bmFramingInfo = BIT(0) | BIT(1);
}

static void uvc_probe_prefered_version(struct uvc_data *const data, uint8_t bRequest,
				       struct uvc_vs_probe_control *probe)
{
	probe->bPreferedVersion = 1;
}

static void uvc_probe_min_version(struct uvc_data *const data, uint8_t bRequest,
				  struct uvc_vs_probe_control *probe)
{
	probe->bMaxVersion = 1;
}

static void uvc_probe_max_version(struct uvc_data *const data, uint8_t bRequest,
				  struct uvc_vs_probe_control *probe)
{
	probe->bMaxVersion = 1;
}

static void uvc_probe_usage(struct uvc_data *const data, uint8_t bRequest,
			    struct uvc_vs_probe_control *probe)
{
	probe->bUsage = 0;
}

static void uvc_probe_bit_depth_luma(struct uvc_data *const data, uint8_t bRequest,
				     struct uvc_vs_probe_control *probe)
{
	/* Bit depth of color/chroma/luma channel, ignored as absent from the Video API. */
	probe->bBitDepthLuma = 0;
}

static void uvc_probe_settings(struct uvc_data *const data, uint8_t bRequest,
			       struct uvc_vs_probe_control *probe)
{
	probe->bmSettings = 0;
}

static void uvc_probe_max_ref_frames(struct uvc_data *const data, uint8_t bRequest,
				     struct uvc_vs_probe_control *probe)
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

static void uvc_probe_dump(char const *name, const struct uvc_vs_probe_control *probe)
{
	LOG_DBG("probe: UVC_%s", name);
	LOG_DBG("probe: - bmHint = 0x%04x", sys_le16_to_cpu(probe->bmHint));
	LOG_DBG("probe: - bFormatIndex = %u", probe->bFormatIndex);
	LOG_DBG("probe: - bFrameIndex = %u", probe->bFrameIndex);
	LOG_DBG("probe: - dwFrameInterval = %u us", sys_le32_to_cpu(probe->dwFrameInterval) / 10);
	LOG_DBG("probe: - wKeyFrameRate = %u", sys_le16_to_cpu(probe->wKeyFrameRate));
	LOG_DBG("probe: - wPFrameRate = %u", probe->wPFrameRate);
	LOG_DBG("probe: - wCompQuality = %u", probe->wCompQuality);
	LOG_DBG("probe: - wCompWindowSize = %u", probe->wCompWindowSize);
	LOG_DBG("probe: - wDelay = %u ms", sys_le16_to_cpu(probe->wDelay));
	LOG_DBG("probe: - dwMaxVideoFrameSize = %u", sys_le32_to_cpu(probe->dwMaxVideoFrameSize));
	LOG_DBG("probe: - dwMaxPayloadTransferSize = %u",
		sys_le32_to_cpu(probe->dwMaxPayloadTransferSize));
	LOG_DBG("probe: - dwClockFrequency = %u Hz", sys_le32_to_cpu(probe->dwClockFrequency));
	LOG_DBG("probe: - bmFramingInfo = 0x%02x", probe->bmFramingInfo);
	LOG_DBG("probe: - bPreferedVersion = %u", probe->bPreferedVersion);
	LOG_DBG("probe: - bMinVersion = %u", probe->bMinVersion);
	LOG_DBG("probe: - bMaxVersion = %u", probe->bMaxVersion);
	LOG_DBG("probe: - bUsage = %u", probe->bUsage);
	LOG_DBG("probe: - bBitDepthLuma = %u", probe->bBitDepthLuma + 8);
	LOG_DBG("probe: - bmSettings = %u", probe->bmSettings);
	LOG_DBG("probe: - bMaxNumberOfRefFramesPlus1 = %u", probe->bMaxNumberOfRefFramesPlus1);
	LOG_DBG("probe: - bmRateControlModes = %u", probe->bmRateControlModes);
	LOG_DBG("probe: - bmLayoutPerStream = 0x%08llx", probe->bmLayoutPerStream);
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
		uvc_probe_dump("SET_CUR", probe);
		goto action;
	case UVC_GET_CUR:
		uvc_probe_dump("GET_CUR", probe);
		goto action;
	case UVC_GET_MIN:
		uvc_probe_dump("GET_MIN", probe);
		goto action;
	case UVC_GET_MAX:
		uvc_probe_dump("GET_MAX", probe);
		goto action;
	case UVC_GET_RES:
		uvc_probe_dump("SET_RES", probe);
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
	switch (bRequest) {
	case UVC_GET_CUR:
		uvc_probe(data, bRequest, probe);
		break;
	case UVC_SET_CUR:
		uvc_probe(data, bRequest, probe);
		LOG_DBG("commit: ready to transfer frames");

		if (data->source_dev != NULL) {
			const struct device *dev = usbd_class_get_private(data->c_data);
			struct video_format fmt = {0};

			LOG_DBG("Setting format of the video source and starting it");

			errno = uvc_get_format(dev, VIDEO_EP_IN, &fmt);
			if (errno) {
				LOG_ERR("Failed to prepare format to the source");
				return 0;
			}

			errno = video_set_format(data->source_dev, VIDEO_EP_OUT, &fmt);
			if (errno) {
				LOG_ERR("Could not set the format of the video source");
				return 0;
			}

			errno = video_stream_start(data->source_dev);
			if (errno) {
				LOG_ERR("Could not start the video source");
				return 0;
			}
		}

		/* Now ready to process  */
		atomic_set_bit(&data->state, UVC_CLASS_READY);
		k_work_submit(&data->work);
		break;
	default:
		LOG_WRN("commit: invalid bRequest (%u)", bRequest);
		errno = EINVAL;
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
		if (buf->size != sizeof(probe)) {
			LOG_WRN("probe: invalid size %u, wanted %u", buf->size, sizeof(probe));
			errno = EINVAL;
			return 0;
		}
		if (net_buf_add(buf, sizeof(probe)) == NULL) {
			return -ENOMEM;
		}
		break;
	case UVC_SET_CUR:
		if (buf->len != sizeof(probe)) {
			LOG_WRN("probe: invalid size %u, wanted %u", buf->len, sizeof(probe));
			errno = EINVAL;
			return 0;
		}
		break;
	default:
		LOG_WRN("probe: invalid bRequest (%u) for Probe or Commit", setup->bRequest);
		errno = EINVAL;
		return 0;
	}

	/* All remaining request work on a struct uvc_vs_probe_control */
	if (setup->wLength != sizeof(probe)) {
		LOG_WRN("probe: invalid wLength %u, wanted %u", setup->wLength, sizeof(probe));
		errno = EINVAL;
		return 0;
	}

	switch (setup->wValue >> 8) {
	case VS_PROBE_CONTROL:
		return uvc_probe(data, setup->bRequest, (void *)buf->data);
	case VS_COMMIT_CONTROL:
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
	case VS_PROBE_CONTROL:
	case VS_COMMIT_CONTROL:
		return uvc_probe_or_commit(data, setup, buf);
	default:
		LOG_WRN("api: control: unsupported wValue (%u)", setup->wValue);
		errno = ENOTSUP;
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
	struct uvc_buf_info bi = *(struct uvc_buf_info *)udc_get_buf_info(buf);

	net_buf_unref(buf);

	if (bi.udc.ep == uvc_get_bulk_in(data)) {
		/* Only for the last buffer of the transfer and last transfer of the frame */
		if (bi.vbuf != NULL) {
			/* Upon completion, move the buffer from submission to completion queue */
			LOG_DBG("request: vbuf=%p transferred", bi.vbuf);
			k_fifo_put(&data->fifo_out, bi.vbuf);
		}
	}

	LOG_DBG("request: transfer done, ep=0x%02x buf=%p", bi.udc.ep, buf);

	return 0;
}

static void uvc_update(struct usbd_class_data *const c_data, uint8_t iface, uint8_t alternate)
{
	LOG_DBG("update");
}

static int uvc_init(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	/* The default probe is the current probe at startup */
	uvc_probe(data, UVC_GET_CUR, &data->default_probe);

	return 0;
}

static void uvc_update_desc(struct uvc_data *const data)
{
	*data->desc_iad_ifnum = *data->desc_if_vc_ifnum;
	*data->desc_if_vc_header_ifnum = *data->desc_if_vs_ifnum;
	*data->desc_if_vs_header_epaddr = uvc_get_bulk_in(data);
}

static void *uvc_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	uvc_update_desc(data);

	switch (speed) {
	case USBD_SPEED_FS:
		return (void *)data->fs_desc;
	case USBD_SPEED_HS:
		return (void *)data->hs_desc;
	case USBD_SPEED_SS:
		return (void *)data->ss_desc;
	default:
		__ASSERT_NO_MSG(false);
	}
}

static int uvc_enqueue_usb(struct uvc_data *data, struct net_buf *buf, bool last_of_transfer)
{
	struct udc_buf_info *bi;
	int err;

	LOG_DBG("queue: usb buf=%p data=%p size=%u len=%u zlp=%u", buf, buf->data, buf->size,
		buf->len, last_of_transfer);

	bi = udc_get_buf_info(buf);
	__ASSERT_NO_MSG(bi != NULL);

	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->ep = uvc_get_bulk_in(data);
	bi->zlp = last_of_transfer;

	err = usbd_ep_enqueue(data->c_data, buf);
	if (err) {
		LOG_ERR("enqueue: error from usbd for buf=%p", buf);
		return err;
	}

	return 0;
}

static void uvc_worker(struct k_work *work)
{
	struct uvc_data *data = CONTAINER_OF(work, struct uvc_data, work);
	struct uvc_buf_info *bi;
	struct video_buffer *vbuf;
	struct net_buf *buf0;
	struct net_buf *buf1;
	int err;

	if (!atomic_test_bit(&data->state, UVC_CLASS_ENABLED) ||
	    !atomic_test_bit(&data->state, UVC_CLASS_READY)) {
		LOG_DBG("queue: UVC not ready to work yet");
		return;
	}

	/* Only remove the buffer from the queue after it is submitted to USB */
	vbuf = k_fifo_peek_head(&data->fifo_in);
	if (vbuf == NULL) {
		return;
	}

	LOG_DBG("queue: video buffer of %u bytes", vbuf->bytesused);

	buf0 = net_buf_alloc_len(&uvc_pool_header, CONFIG_USBD_VIDEO_HEADER_SIZE, K_NO_WAIT);
	if (buf0 == NULL) {
		LOG_DBG("queue: failed to allocate the header");
		return;
	}

	if (vbuf->flags | VIDEO_BUF_EOF) {
		/* new frame: toggle the FRAMEID ans flag as EOF */
		data->payload_header.bmHeaderInfo ^= UVC_BMHEADERINFO_FRAMEID;
		data->payload_header.bmHeaderInfo |= UVC_BMHEADERINFO_END_OF_FRAME;
	} else {
		/* Not setting the EOF flag until we reach the last transfer */
		data->payload_header.bmHeaderInfo &= ~UVC_BMHEADERINFO_END_OF_FRAME;
	}

	/* Only the 2 first (uint8_t) fields are supported for now */
	net_buf_add_mem(buf0, &data->payload_header, 2);

	/* Pad the header up to CONFIG_USBD_VIDEO_HEADER_SIZE */
	while (buf0->len < buf0->size) {
		net_buf_add_u8(buf0, 0);
	}

	bi = (void *)udc_get_buf_info(buf0);
	bi->vbuf = NULL;

	err = uvc_enqueue_usb(data, buf0, false);
	if (err) {
		LOG_ERR("queue: failed to submit the header to USB");
		net_buf_unref(buf0);
		return;
	}

	buf1 = net_buf_alloc_with_data(&uvc_pool_payload, vbuf->buffer, vbuf->bytesused, K_NO_WAIT);
	if (buf1 == NULL) {
		LOG_ERR("queue: failed to allocate the header");
		net_buf_unref(buf1);
		return;
	}

	LOG_DBG("queue: submitting vbuf=%p buffer=%p bytesused=%u",
		vbuf, vbuf->buffer, vbuf->bytesused);

	/* Attach the video buffer to the USB buffer so that we get it back from USB */
	bi = (void *)udc_get_buf_info(buf1);
	bi->vbuf = vbuf;

	err = uvc_enqueue_usb(data, buf1, true);
	if (err) {
		LOG_ERR("queue: failed to submit the payload to USB");
		net_buf_unref(buf0);
		net_buf_unref(buf1);
		return;
	}

	/* Remove the buffer from the queue now that USB driver received it */
	k_fifo_get(&data->fifo_in, K_NO_WAIT);

	/* Work on the next buffer */
	k_work_submit(&data->work);
}

static void uvc_enable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	/* Catch-up with buffers that might have been delayed */
	atomic_set_bit(&data->state, UVC_CLASS_ENABLED);
	k_work_submit(&data->work);
}

static void uvc_disable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	atomic_clear_bit(&data->state, UVC_CLASS_ENABLED);
}

struct usbd_class_api uvc_class_api = {
	.enable = uvc_enable,
	.disable = uvc_disable,
	.request = uvc_request,
	.update = uvc_update,
	.control_to_host = uvc_control_to_host,
	.control_to_dev = uvc_control_to_dev,
	.init = uvc_init,
	.get_desc = uvc_get_desc,
};

static int uvc_enqueue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer *vbuf)
{
	struct uvc_data *data = dev->data;

	if (ep != VIDEO_EP_IN && ep != VIDEO_EP_ANY) {
		return -EINVAL;
	}

	k_fifo_put(&data->fifo_in, vbuf);
	k_work_submit(&data->work);
	return 0;
}

static int uvc_dequeue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer **vbuf, k_timeout_t timeout)
{
	struct uvc_data *data = dev->data;

	if (ep != VIDEO_EP_IN && ep != VIDEO_EP_ANY) {
		return -EINVAL;
	}

	*vbuf = k_fifo_get(&data->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int uvc_get_format(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	struct uvc_data *data = dev->data;
	const struct video_format_cap *cap = &data->caps[data->format_id];
	const struct uvc_format *ufmt = &data->formats[data->format_id];

	if (ep != VIDEO_EP_IN && ep != VIDEO_EP_ANY) {
		return -EINVAL;
	}

	memset(fmt, 0, sizeof(*fmt));

	fmt->pixelformat = cap->pixelformat;
	fmt->width = cap->width_max;
	fmt->height = cap->height_max;
	fmt->pitch = fmt->width * ufmt->bits_per_pixel / 8;

	return 0;
}

static int uvc_stream_start(const struct device *dev)
{
	struct uvc_data *data = dev->data;

	if (data->source_dev != NULL) {
		return video_stream_start(dev);
	}

	return 0;
}

static int uvc_stream_stop(const struct device *dev)
{
	struct uvc_data *data = dev->data;

	if (data->source_dev != NULL) {
		return video_stream_stop(dev);
	}

	return 0;
}

static int uvc_get_caps(const struct device *dev, enum video_endpoint_id ep,
			struct video_caps *caps)
{
	struct uvc_data *data = dev->data;

	caps->format_caps = data->caps;

	return 0;
}

struct video_driver_api uvc_video_api = {
	.get_format = uvc_get_format,
	.stream_start = uvc_stream_start,
	.stream_stop = uvc_stream_stop,
	.get_caps = uvc_get_caps,
	.enqueue = uvc_enqueue,
	.dequeue = uvc_dequeue,
};

static int uvc_preinit(const struct device *dev)
{
	struct uvc_data *data = dev->data;

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	k_work_init(&data->work, &uvc_worker);

	return 0;
}

#define FORMAT_FOURCC(format)							\
	video_fourcc(DT_PROP(format, fourcc)[0], DT_PROP(format, fourcc)[1],	\
		     DT_PROP(format, fourcc)[2], DT_PROP(format, fourcc)[3])

#define VIDEO_FRAME_CAP(frame, format)						\
	{									\
		.pixelformat = FORMAT_FOURCC(format),				\
		.width_min = DT_PROP_BY_IDX(frame, size, 0),			\
		.width_max = DT_PROP_BY_IDX(frame, size, 0),			\
		.width_step = 0,						\
		.height_min = DT_PROP_BY_IDX(frame, size, 1),			\
		.height_max = DT_PROP_BY_IDX(frame, size, 1),			\
		.height_step = 0						\
	},

#define VIDEO_FORMAT_CAP(node)							\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uvc_format), (		\
		DT_FOREACH_CHILD_VARGS(node, VIDEO_FRAME_CAP, node)		\
	))

#define UVC_FORMAT_ENTRY(frame, format)						\
	{									\
		.bFormatIndex = VC_ENTITY_ID(format),				\
		.bFrameIndex = VC_ENTITY_ID(frame),				\
		.bDescriptorSubtype = format##_desc[2],				\
		.frame_interval = 0,						\
		.bits_per_pixel = DT_PROP(format, bits_per_pixel),		\
	},

#define UVC_FORMAT_LOOKUP_TABLE(format)						\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uvc_format), (		\
		DT_FOREACH_CHILD_VARGS(format, UVC_FORMAT_ENTRY, format)	\
	))

#define UVC_DEVICE_DEFINE(node)							\
										\
	UVC_DESCRIPTOR_ARRAYS(node)						\
										\
	static struct usb_desc_header *const node##_fs_desc[] = {		\
		UVC_FULLSPEED_DESCRIPTOR_PTRS(node)				\
	};									\
										\
	static struct usb_desc_header *const node##_hs_desc[] = {		\
		UVC_HIGHSPEED_DESCRIPTOR_PTRS(node)				\
	};									\
										\
	static struct usb_desc_header *const node##_ss_desc[] = {		\
		UVC_SUPERSPEED_DESCRIPTOR_PTRS(node)				\
	};									\
										\
	static const struct video_format_cap node##_caps[] = {			\
		DT_FOREACH_CHILD(node, VIDEO_FORMAT_CAP)			\
		{0}								\
	};									\
										\
	static const struct uvc_format node##_formats[] = {			\
		DT_FOREACH_CHILD(node, UVC_FORMAT_LOOKUP_TABLE)			\
		{0}								\
	};									\
										\
	USBD_DEFINE_CLASS(node##_c_data, &uvc_class_api,			\
			  (void *)DEVICE_DT_GET(node), NULL);			\
										\
	static struct uvc_data node##_data = {					\
		.c_data = &node##_c_data,					\
		.caps = node##_caps,						\
		.formats = node##_formats,					\
		.format_id = 0,							\
		.fs_desc = node##_fs_desc,					\
		.hs_desc = node##_hs_desc,					\
		.ss_desc = node##_ss_desc,					\
		.desc_iad_ifnum = node##_desc_iad + 2,				\
		.desc_if_vc_ifnum = node##_desc_if_vc + 2,			\
		.desc_if_vc_header_ifnum = node##_desc_if_vc_header + 12,	\
		.desc_if_vs_ifnum = node##_desc_if_vs + 2,			\
		.desc_if_vs_header_epaddr = node##_desc_if_vs_header + 6,	\
		.fs_desc_ep_epaddr = node##_fs_desc_ep + 2,			\
		.hs_desc_ep_epaddr = node##_hs_desc_ep + 2,			\
		.ss_desc_ep_epaddr = node##_ss_desc_ep + 2,			\
		.payload_header.bHeaderLength = CONFIG_USBD_VIDEO_HEADER_SIZE,	\
		.source_dev = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(node, source)),	\
	};									\
										\
	BUILD_ASSERT(DT_ON_BUS(node, usb),					\
		     "node " DT_NODE_PATH(node) " is not"			\
		     " assigned to a USB device controller");			\
										\
	DEVICE_DT_DEFINE(node, uvc_preinit, NULL, &node##_data, NULL,		\
			 POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,		\
			 &uvc_video_api);

DT_FOREACH_STATUS_OKAY(zephyr_uvc_format_uncompressed, VS_UNCOMPRESSED_DESCRIPTOR_ARRAYS)
DT_FOREACH_STATUS_OKAY(zephyr_uvc_format_mjpeg, VS_MJPEG_DESCRIPTOR_ARRAYS)
DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, UVC_DEVICE_DEFINE)
