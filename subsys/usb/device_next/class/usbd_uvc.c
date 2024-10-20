/*
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_uvc_device

#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/logging/log.h>

#include "usbd_uvc_macros.h"

LOG_MODULE_REGISTER(usbd_uvc, CONFIG_USBD_UVC_LOG_LEVEL);

#define UVC_CLASS_ENABLED 0
#define UVC_CLASS_READY   1

/* Video Class-Specific Request Codes */
#define RC_UNDEFINED 0x00
#define SET_CUR      0x01
#define GET_CUR      0x81
#define GET_MIN      0x82
#define GET_MAX      0x83
#define GET_RES      0x84
#define GET_LEN      0x85
#define GET_INFO     0x86
#define GET_DEF      0x87

/* 4.2.1.2 Request Error Code Control */
#define ERR_NOT_READY                  0x01
#define ERR_WRONG_STATE                0x02
#define ERR_OUT_OF_RANGE               0x04
#define ERR_INVALID_UNIT               0x05
#define ERR_INVALID_CONTROL            0x06
#define ERR_INVALID_REQUEST            0x07
#define ERR_INVALID_VALUE_WITHIN_RANGE 0x08
#define ERR_UNKNOWN                    0xff

/* Flags announcing which controls are supported */
#define INFO_SUPPORTS_GET BIT(0)
#define INFO_SUPPORTS_SET BIT(1)

/* Video and Still Image Payload Headers */
#define UVC_BMHEADERINFO_FRAMEID              BIT(0)
#define UVC_BMHEADERINFO_END_OF_FRAME         BIT(1)
#define UVC_BMHEADERINFO_HAS_PRESENTATIONTIME BIT(2)
#define UVC_BMHEADERINFO_HAS_SOURCECLOCK      BIT(3)
#define UVC_BMHEADERINFO_PAYLOAD_SPECIFIC_BIT BIT(4)
#define UVC_BMHEADERINFO_STILL_IMAGE          BIT(5)
#define UVC_BMHEADERINFO_ERROR                BIT(6)
#define UVC_BMHEADERINFO_END_OF_HEADER        BIT(7)

/* Entity Descriptor */
struct uvc_desc_entity {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bEntityID;
};

/* Video Probe and Commit Controls */
struct uvc_probe {
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
#define UVC_BMFRAMING_INFO_FID BIT(0)
#define UVC_BMFRAMING_INFO_EOF BIT(1)
#define UVC_BMFRAMING_INFO_EOS BIT(2)
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

/* Video and Still Image Payload Headers */
struct uvc_payload_header {
	uint8_t bHeaderLength;
	uint8_t bmHeaderInfo;
	uint32_t dwPresentationTime; /* optional */
	uint32_t scrSourceClockSTC;  /* optional */
	uint16_t scrSourceClockSOF;  /* optional */
} __packed;

/* Information specific to each VideoControl interface */
struct uvc_control {
	uint8_t entity_id;
	const struct device *dev;
	/* Stream interface affected by this control */
	struct uvc_stream *stream;
	/* Handler function  */
	int (*fn)(const struct usb_setup_packet *setup, struct net_buf *buf,
		  const struct device *dev);
	/* Bitmask of enabled controls for this interface */
	uint64_t mask;
	/* USB descriptors */
	struct usbd_desc_node *desc_str;
	struct usb_desc_header *desc_ctl;
};

/* Information specific to each VideoStreaming interface */
struct uvc_stream {

	/* USB protocol state */

	/* USBD class state */
	atomic_t state;
	/* Descriptor fields that need to be accessed */
	uint8_t *desc_vs_ifnum;
	uint8_t *desc_vs_epaddr;

	/* Video API state */

	/* Device where the data is enqueued/dequeued */
	const struct device *dev;
	/* Video capabilities selected for this device */
	const struct video_format_cap *caps;
	/* UVC worker to process the queue */
	struct k_work work;
	/* Video FIFOs for submission (in) and completion (out) queue */
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	/* Offset in bytes within the fragment including the header */
	uint32_t xfer_offset;

	/* UVC protocol state */

	/* UVC probe-commit control default values */
	struct uvc_probe default_probe;
	/* UVC payload header, passed just before the image data */
	struct uvc_payload_header payload_header;
	/* UVC format currently selected */
	int format_id;
};

/* Global configuration */
struct uvc_data {
	/* UVC error from latest request */
	int err;
};

/* Compile-time constant configuration */
struct uvc_conf {
	/* USBD class structure */
	struct usbd_class_data *c_data;
	/* UVC lookup tables */
	const struct uvc_control *controls;
	struct uvc_stream *streams;
	/* UVC Descriptors */
	struct usb_desc_header *const *fs_desc;
	struct usb_desc_header *const *hs_desc;
	/* UVC Fields that need to be accessed */
	uint8_t *desc_iad_ifnum;
	uint8_t *desc_vc_ifnum;
	uint8_t *desc_vc_header;
	uint8_t *desc_fs_epaddr;
	uint8_t *desc_hs_epaddr;
};

/* Specialized version of UDC net_buf metadata with extra fields */
struct uvc_buf_info {
	/* Regular UDC buf info so that it can be passed to USBD directly */
	struct udc_buf_info udc;
	/* Extra field at the end */
	struct video_buffer *vbuf;
	/* UVC stream this buffer belongs to */
	struct uvc_stream *stream;
} __packed;

NET_BUF_POOL_FIXED_DEFINE(uvc_pool, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 100, 0,
			  sizeof(struct uvc_buf_info), NULL);

static int uvc_get_format(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt);

int uvc_get_stream(const struct device *dev, enum video_endpoint_id ep, struct uvc_stream **result)
{
	const struct uvc_conf *conf = dev->config;

	if (ep == VIDEO_EP_IN) {
		return -EINVAL;
	}

	if (ep == VIDEO_EP_OUT || ep == VIDEO_EP_ALL) {
		*result = &conf->streams[0];
		return 0;
	}

	/* Iterate instead of dereference to prevent overflow */
	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++, ep--) {
		if (ep == 0) {
			*result = strm;
			return 0;
		}
	}
	return -ENODEV;
}

static int uvc_get_errno(int err)
{
	switch (err) {
	case EBUSY:       /* Busy and not ready */
	case EAGAIN:      /* Try again when the device becomes ready */
	case EINPROGRESS: /* Will be ready after this ongoing operation */
	case EALREADY:    /* Already enqueued, will be ready when done */
		return ERR_NOT_READY;
	case EOVERFLOW: /* Values overflowed the range */
	case ERANGE:    /* Value not in range */
	case E2BIG:     /* Value too big for the range */
		return ERR_OUT_OF_RANGE;
	case EDOM:   /* Invalid but still in the range */
	case EINVAL: /* Invalid argument but not ERANGE */
		return ERR_INVALID_VALUE_WITHIN_RANGE;
	case ENODEV:  /* No device supporting this request */
	case ENOTSUP: /* Request not supported */
	case ENOSYS:  /* Request not implemented */
		return ERR_INVALID_REQUEST;
	default:
		return ERR_UNKNOWN;
	}
}

static uint32_t uvc_get_video_cid(const struct usb_setup_packet *setup, uint32_t cid)
{
	switch (setup->bRequest) {
	case GET_DEF:
		return VIDEO_CTRL_GET_DEF | cid;
	case GET_CUR:
		return VIDEO_CTRL_GET_CUR | cid;
	case GET_MIN:
		return VIDEO_CTRL_GET_MIN | cid;
	case GET_MAX:
		return VIDEO_CTRL_GET_MAX | cid;
	default:
		__ASSERT_NO_MSG(false);
		return 0;
	}
}

static int uvc_buf_add(struct net_buf *buf, uint16_t size, uint32_t value)
{
	if (buf->size != size) {
		LOG_ERR("invalid wLength %u, expected %u", buf->size, size);
		return -EINVAL;
	}

	switch (size) {
	case 4:
		net_buf_add_le32(buf, value);
		return 0;
	case 2:
		net_buf_add_le16(buf, value);
		return 0;
	case 1:
		net_buf_add_u8(buf, value);
		return 0;
	default:
		LOG_WRN("control: invalid size %u", size);
		return -ENOTSUP;
	}
}

static int uvc_buf_remove(struct net_buf *buf, uint16_t size, uint32_t *value)
{
	switch (size) {
	case 4:
		*value = net_buf_remove_le32(buf);
		return 0;
	case 2:
		*value = net_buf_remove_le16(buf);
		return 0;
	case 1:
		*value = net_buf_remove_u8(buf);
		return 0;
	default:
		LOG_WRN("control: invalid size %u", size);
		return -ENOTSUP;
	}
}

static uint8_t uvc_get_bulk_in(const struct device *const dev)
{
	const struct uvc_conf *conf = dev->config;

	switch (usbd_bus_speed(usbd_class_get_ctx(conf->c_data))) {
	case USBD_SPEED_FS:
		return *conf->desc_fs_epaddr;
	case USBD_SPEED_HS:
		return *conf->desc_hs_epaddr;
	default:
		__ASSERT_NO_MSG(false);
		return 0;
	}
}

static size_t uvc_get_bulk_mps(struct usbd_class_data *const c_data)
{
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);

	switch (usbd_bus_speed(uds_ctx)) {
	case USBD_SPEED_FS:
		return 64;
	case USBD_SPEED_HS:
		return 512;
	default:
		__ASSERT_NO_MSG(false);
		return 0;
	}
}

static uint32_t uvc_get_max_frame_size(struct uvc_stream *strm)
{
	const struct video_format_cap *cap = &strm->caps[strm->format_id];

	return cap->width_max * cap->height_max * video_bits_per_pixel(cap->pixelformat);
}

static int uvc_control_probe_format_index(const struct device *dev, struct uvc_stream *strm,
					  uint8_t request, struct uvc_probe *probe)
{
	switch (request) {
	case GET_MIN:
		probe->bFormatIndex = 1;
		break;
	case GET_MAX:
		for (size_t i = 0; strm->caps[i].pixelformat != 0; i++) {
			probe->bFormatIndex = i + 1;
		}
		break;
	case GET_RES:
		probe->bFormatIndex = 1;
		break;
	case GET_CUR:
		probe->bFormatIndex = strm->format_id + 1;
		break;
	case SET_CUR:
		if (probe->bFormatIndex == 0) {
			return 0;
		}
		for (size_t i = 0; strm->caps[i].pixelformat != 0; i++) {
			if (probe->bFormatIndex == i + 1) {
				strm->format_id = i;
				return 0;
			}
		}
		LOG_WRN("probe: format index %u not found", probe->bFormatIndex);
		return -ENOTSUP;
	}
	return 0;
}

static int uvc_control_probe_frame_index(const struct device *dev, struct uvc_stream *strm,
					 uint8_t request, struct uvc_probe *probe)
{
	switch (request) {
	case GET_MIN:
	case GET_MAX:
	case GET_RES:
	case GET_CUR:
		probe->bFrameIndex = 1;
		break;
	case SET_CUR:
		if (probe->bFrameIndex == 0 || probe->bFrameIndex == 1) {
			return 0;
		}
		LOG_WRN("probe: frame index %u not found", probe->bFrameIndex);
		return -ENOTSUP;
	}
	return 0;
}

static int uvc_control_probe_frame_interval(const struct device *dev, struct uvc_stream *strm,
					    uint8_t request, struct uvc_probe *probe)
{
	switch (request) {
	case GET_MIN:
	case GET_MAX:
		/* TODO call the frame interval API on the video source once supported */
		probe->dwFrameInterval = sys_cpu_to_le32(10000000);
		break;
	case GET_RES:
		probe->dwFrameInterval = sys_cpu_to_le32(1);
		break;
	case SET_CUR:
		/* TODO call the frame interval API on the video source once supported */
		break;
	}
	return 0;
}

static int uvc_control_probe_max_video_frame_size(const struct device *dev, struct uvc_stream *strm,
						  uint8_t request, struct uvc_probe *probe)
{
	uint32_t max_frame_size = uvc_get_max_frame_size(strm);

	switch (request) {
	case GET_MIN:
	case GET_MAX:
	case GET_CUR:
		probe->dwMaxVideoFrameSize = sys_cpu_to_le32(max_frame_size);
		break;
	case GET_RES:
		probe->dwMaxVideoFrameSize = sys_cpu_to_le32(1);
		break;
	case SET_CUR:
		if (sys_le32_to_cpu(probe->dwMaxVideoFrameSize) > 0 &&
		    sys_le32_to_cpu(probe->dwMaxVideoFrameSize) != max_frame_size) {
			LOG_WRN("probe: dwMaxVideoFrameSize is read-only");
		}
		break;
	}
	return 0;
}

static int uvc_control_probe_max_payload_size(const struct device *dev, struct uvc_stream *strm,
					      uint8_t request, struct uvc_probe *probe)
{
	uint32_t max_payload_size = uvc_get_max_frame_size(strm) + CONFIG_USBD_VIDEO_HEADER_SIZE;

	switch (request) {
	case GET_MIN:
	case GET_MAX:
	case GET_CUR:
		probe->dwMaxPayloadTransferSize = sys_cpu_to_le32(max_payload_size);
		break;
	case GET_RES:
		probe->dwMaxPayloadTransferSize = sys_cpu_to_le32(1);
		break;
	case SET_CUR:
		if (sys_le32_to_cpu(probe->dwMaxPayloadTransferSize) > 0 &&
		    sys_le32_to_cpu(probe->dwMaxPayloadTransferSize) != max_payload_size) {
			LOG_WRN("probe: dwPayloadTransferSize is read-only");
		}
		break;
	}
	return 0;
}

#if CONFIG_USBD_UVC_LOG_LEVEL >= LOG_LEVEL_DBG
static void uvc_log_probe(const char *name, uint32_t probe)
{
	if (probe > 0) {
		LOG_DBG(" %s %u", name, probe);
	}
}
#endif

static char const *uvc_get_request_str(const struct usb_setup_packet *setup)
{
	switch (setup->bRequest) {
	case SET_CUR:
		return "SET_CUR";
	case GET_CUR:
		return "GET_CUR";
	case GET_MIN:
		return "GET_MIN";
	case GET_MAX:
		return "GET_MAX";
	case GET_RES:
		return "GET_RES";
	case GET_LEN:
		return "GET_LEN";
	case GET_DEF:
		return "GET_DEF";
	case GET_INFO:
		return "GET_INFO";
	default:
		return "(unknown)";
	}
}

static int uvc_control_probe(const struct device *dev, struct uvc_stream *strm, uint8_t request,
			     struct uvc_probe *probe)
{
	int ret;

	if (request != GET_MIN && request != GET_MAX && request != GET_RES &&
	    request != GET_CUR && request != SET_CUR) {
		LOG_WRN("control: invalid bRequest %u", request);
		return -EINVAL;
	}

	/* Dynamic fields */
	ret = uvc_control_probe_format_index(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}
	ret = uvc_control_probe_frame_index(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}
	ret = uvc_control_probe_frame_interval(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}
	ret = uvc_control_probe_max_video_frame_size(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}
	ret = uvc_control_probe_max_payload_size(dev, strm, request, probe);
	if (ret < 0) {
		return ret;
	}

	/* Static fields */
	probe->dwClockFrequency = sys_cpu_to_le32(1);
	/* Include Frame ID and EOF fields in the payload header */
	probe->bmFramingInfo = BIT(0) | BIT(1);
	probe->bPreferedVersion = 1;
	probe->bMinVersion = 1;
	probe->bMaxVersion = 1;
	probe->bUsage = 0;
	probe->bBitDepthLuma = 0;
	probe->bmSettings = 0;
	probe->bMaxNumberOfRefFramesPlus1 = 1;
	probe->bmRateControlModes = 0;
	probe->bmLayoutPerStream = 0;
	probe->wKeyFrameRate = sys_cpu_to_le16(0);
	probe->wPFrameRate = sys_cpu_to_le16(0);
	probe->wCompQuality = sys_cpu_to_le16(0);
	probe->wCompWindowSize = sys_cpu_to_le16(0);
	/* TODO devicetree */
	probe->wDelay = sys_cpu_to_le16(1);

#if CONFIG_USBD_UVC_LOG_LEVEL >= LOG_LEVEL_DBG
	uvc_log_probe("bmHint", sys_le16_to_cpu(probe->bmHint));
	uvc_log_probe("bFormatIndex", probe->bFormatIndex);
	uvc_log_probe("bFrameIndex", probe->bFrameIndex);
	uvc_log_probe("dwFrameInterval (us)", sys_le32_to_cpu(probe->dwFrameInterval) / 10);
	uvc_log_probe("wKeyFrameRate", sys_le16_to_cpu(probe->wKeyFrameRate));
	uvc_log_probe("wPFrameRate", sys_le16_to_cpu(probe->wPFrameRate));
	uvc_log_probe("wCompQuality", sys_le16_to_cpu(probe->wCompQuality));
	uvc_log_probe("wCompWindowSize", sys_le16_to_cpu(probe->wCompWindowSize));
	uvc_log_probe("wDelay (ms)", sys_le16_to_cpu(probe->wDelay));
	uvc_log_probe("dwMaxVideoFrameSize", sys_le32_to_cpu(probe->dwMaxVideoFrameSize));
	uvc_log_probe("dwMaxPayloadTransferSize", sys_le32_to_cpu(probe->dwMaxPayloadTransferSize));
	uvc_log_probe("dwClockFrequency (Hz)", sys_le32_to_cpu(probe->dwClockFrequency));
	uvc_log_probe("bmFramingInfo", probe->bmFramingInfo);
	uvc_log_probe("bPreferedVersion", probe->bPreferedVersion);
	uvc_log_probe("bMinVersion", probe->bMinVersion);
	uvc_log_probe("bMaxVersion", probe->bMaxVersion);
	uvc_log_probe("bUsage", probe->bUsage);
	uvc_log_probe("bBitDepthLuma", probe->bBitDepthLuma + 8);
	uvc_log_probe("bmSettings", probe->bmSettings);
	uvc_log_probe("bMaxNumberOfRefFramesPlus1", probe->bMaxNumberOfRefFramesPlus1);
	uvc_log_probe("bmRateControlModes", probe->bmRateControlModes);
	uvc_log_probe("bmLayoutPerStream", probe->bmLayoutPerStream);
#endif

	return 0;
}

static int uvc_control_commit(const struct device *dev, struct uvc_stream *strm, uint8_t request,
			      struct uvc_probe *probe)
{
	struct video_format fmt = {0};
	int ret;

	switch (request) {
	case GET_CUR:
		return uvc_control_probe(dev, strm, request, probe);
	case SET_CUR:
		ret = uvc_control_probe(dev, strm, request, probe);
		if (ret < 0) {
			return ret;
		}

		atomic_set_bit(&strm->state, UVC_CLASS_READY);
		k_work_submit(&strm->work);

		ret = uvc_get_format(dev, VIDEO_EP_IN, &fmt);
		if (ret < 0) {
			LOG_ERR("Failed to inquire the current UVC format");
			return ret;
		}

		LOG_INF("control: ready to transfer %ux%u frames of %u bytes", fmt.width,
			fmt.height, fmt.height * fmt.pitch);

		if (strm->dev != NULL) {
			LOG_DBG("control: setting source format to %ux%u", fmt.width, fmt.height);

			ret = video_set_format(strm->dev, VIDEO_EP_OUT, &fmt);
			if (ret < 0) {
				LOG_ERR("Could not set the format of the video source");
				return ret;
			}

			ret = video_stream_start(strm->dev);
			if (ret < 0) {
				LOG_ERR("Could not start the video source");
				return ret;
			}
		}

		break;
	default:
		LOG_WRN("commit: invalid bRequest %u", request);
		return -EINVAL;
	}
	return 0;
}

static int uvc_control_vs(const struct device *dev, struct uvc_stream *strm,
			  const struct usb_setup_packet *setup, struct net_buf *buf)
{
	uint8_t control_selector = setup->wValue >> 8;
	struct uvc_probe *probe = (void *)buf->data;

	switch (setup->bRequest) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET | INFO_SUPPORTS_SET);
	case GET_LEN:
		return uvc_buf_add(buf, 2, sizeof(probe));
	case GET_DEF:
		if (setup->wLength != sizeof(*probe) || buf->size < sizeof(*probe)) {
			LOG_ERR("control: bad wLength %u or bufsize %u", setup->wLength, buf->size);
			return -EINVAL;
		}
		net_buf_add_mem(buf, &strm->default_probe, sizeof(*probe));
		return 0;
	case GET_MIN:
	case GET_MAX:
	case GET_RES:
	case GET_CUR:
		if (setup->wLength != sizeof(*probe) || buf->size < sizeof(*probe)) {
			LOG_ERR("control: bad wLength %u or bufsize %u", setup->wLength, buf->size);
			return -EINVAL;
		}
		net_buf_add(buf, sizeof(*probe));
		break;
	case SET_CUR:
		if (setup->wLength != sizeof(*probe) || buf->len < sizeof(*probe)) {
			LOG_ERR("control: bad wLength %u or buflen %u", setup->wLength, buf->len);
			return -EINVAL;
		}
		break;
	}

	switch (control_selector) {
	case VS_PROBE_CONTROL:
		LOG_DBG("VS_PROBE_CONTROL");
		return uvc_control_probe(dev, strm, setup->bRequest, probe);
	case VS_COMMIT_CONTROL:
		LOG_DBG("VS_COMMIT_CONTROL");
		return uvc_control_commit(dev, strm, setup->bRequest, probe);
	default:
		LOG_WRN("control: unknown selector %u for streaming interface", control_selector);
		return -ENOTSUP;
	}
}

static int uvc_control_default(const struct usb_setup_packet *setup, struct net_buf *buf,
			       uint8_t size)
{
	switch (setup->bRequest) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET | INFO_SUPPORTS_SET);
	case GET_LEN:
		return uvc_buf_add(buf, setup->wLength, size);
	case GET_RES:
		return uvc_buf_add(buf, size, 1);
	default:
		LOG_WRN("control: unsupported request type %u", setup->bRequest);
		return -ENOTSUP;
	}
}

static int uvc_control_fix(const struct usb_setup_packet *setup, struct net_buf *buf, uint8_t size,
			   uint32_t value)
{
	LOG_DBG("control: fixed type control, size %u", size);

	switch (setup->bRequest) {
	case GET_DEF:
	case GET_CUR:
	case GET_MIN:
	case GET_MAX:
		return uvc_buf_add(buf, size, value);
	case SET_CUR:
		return 0;
	default:
		return uvc_control_default(setup, buf, size);
	}
}

static int uvc_control_uint(const struct usb_setup_packet *setup, struct net_buf *buf, uint8_t size,
			    const struct device *dev, uint32_t cid)
{
	uint32_t value;
	int ret;

	LOG_DBG("control: integer type control, size %u", size);

	switch (setup->bRequest) {
	case GET_DEF:
	case GET_CUR:
	case GET_MIN:
	case GET_MAX:
		ret = video_get_ctrl(dev, uvc_get_video_cid(setup, cid), &value);
		if (ret < 0) {
			LOG_ERR("control: failed to query target video device");
			return ret;
		}
		LOG_DBG("control: value for CID 0x08%x is %u", cid, value);
		return uvc_buf_add(buf, size, value);
	case SET_CUR:
		ret = uvc_buf_remove(buf, size, &value);
		if (ret < 0) {
			return ret;
		}
		LOG_DBG("control: setting CID 0x08%x to %u", cid, value);
		ret = video_set_ctrl(dev, cid, (void *)value);
		if (ret < 0) {
			LOG_ERR("control: failed to configure target video device");
			return ret;
		}
		return 0;
	default:
		return uvc_control_default(setup, buf, size);
	}
}

__unused static int uvc_control_ct(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	uint8_t control_selector = setup->wValue >> 8;

	/* See also zephyr,uvc-control-ct.yaml */
	switch (control_selector) {
	case CT_AE_MODE_CONTROL:
		LOG_DBG("CT_AE_MODE_CONTROL -> (none)");
		return uvc_control_fix(setup, buf, 1, BIT(0));
	case CT_AE_PRIORITY_CONTROL:
		LOG_DBG("CT_AE_PRIORITY_CONTROL -> (none)");
		return uvc_control_fix(setup, buf, 1, 0);
	case CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
		LOG_DBG("CT_EXPOSURE_TIME_ABSOLUTE_CONTROL -> VIDEO_CID_CAMERA_EXPOSURE");
		return uvc_control_uint(setup, buf, 4, dev, VIDEO_CID_CAMERA_EXPOSURE);
	case CT_ZOOM_ABSOLUTE_CONTROL:
		LOG_DBG("CT_ZOOM_ABSOLUTE_CONTROL -> VIDEO_CID_CAMERA_ZOOM");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_ZOOM);
	default:
		LOG_WRN("control: unsupported selector %u for camera terminal ", control_selector);
		return -ENOTSUP;
	}
}

__unused static int uvc_control_pu(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	uint8_t control_selector = setup->wValue >> 8;

	/* See also zephyr,uvc-control-pu.yaml */
	switch (control_selector) {
	case PU_BRIGHTNESS_CONTROL:
		LOG_DBG("PU_BRIGHTNESS_CONTROL -> VIDEO_CID_CAMERA_BRIGHTNESS");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_BRIGHTNESS);
	case PU_CONTRAST_CONTROL:
		LOG_DBG("PU_CONTRAST_CONTROL -> VIDEO_CID_CAMERA_CONTRAST");
		return uvc_control_uint(setup, buf, 1, dev, VIDEO_CID_CAMERA_CONTRAST);
	case PU_GAIN_CONTROL:
		LOG_DBG("PU_GAIN_CONTROL -> VIDEO_CID_CAMERA_GAIN");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_GAIN);
	case PU_SATURATION_CONTROL:
		LOG_DBG("PU_SATURATION_CONTROL -> VIDEO_CID_CAMERA_SATURATION");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_SATURATION);
	case PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
		LOG_DBG("PU_WHITE_BALANCE_TEMPERATURE_CONTROL -> VIDEO_CID_CAMERA_WHITE_BAL");
		return uvc_control_uint(setup, buf, 2, dev, VIDEO_CID_CAMERA_WHITE_BAL);
	default:
		LOG_WRN("control: unsupported selector %u for processing unit", control_selector);
		return -ENOTSUP;
	}
}

__unused static int uvc_control_xu(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	LOG_WRN("control: nothing supported for extension unit");
	return -ENOTSUP;
};

__unused static int uvc_control_it(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	LOG_WRN("control: nothing supported for input terminal");
	return -ENOTSUP;
};

static int uvc_control_ot(const struct usb_setup_packet *setup, struct net_buf *buf,
			  const struct device *dev)
{
	LOG_WRN("control: nothing supported for output terminal");
	return -ENOTSUP;
};

static int uvc_control_vc(const struct device *dev, const struct uvc_control *ctrl,
			  const struct usb_setup_packet *setup, struct net_buf *buf)
{
	struct uvc_data *data = dev->data;
	uint8_t control_selector = setup->wValue >> 8;
	uint8_t entity_id = (setup->wIndex >> 8) & 0xff;
	int ret;

	if ((ctrl->mask & BIT(control_selector)) == 0) {
		LOG_WRN("control selector %u not enabled for bEntityID %u",
		control_selector, entity_id);
		data->err = ERR_INVALID_CONTROL;
		return -ENOTSUP;
	}

	/* Control set as supported by the devicetree, call the handler */
	ret = ctrl->fn(setup, buf, ctrl->dev);
	data->err = uvc_get_errno(-ret);
	return ret;
}

static int uvc_control_errno(const struct usb_setup_packet *setup, struct net_buf *buf, int err)
{
	switch (setup->bRequest) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET);
	case GET_CUR:
		return uvc_buf_add(buf, 1, err);
	default:
		LOG_WRN("control: unsupported request type %u", setup->bRequest);
		return -ENOTSUP;
	}
}

static int uvc_control(const struct device *dev, const struct usb_setup_packet *const setup,
		       struct net_buf *buf)
{
	const struct uvc_conf *conf = dev->config;
	struct uvc_data *data = dev->data;
	uint8_t ifnum = (setup->wIndex >> 0) & 0xff;
	uint8_t entity_id = (setup->wIndex >> 8) & 0xff;

	LOG_DBG("%s", uvc_get_request_str(setup));

	/* VideoStreaming requests */

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		if (*strm->desc_vs_ifnum == ifnum) {
			return uvc_control_vs(dev, strm, setup, buf);
		}
	}

	if (ifnum == *conf->desc_vc_ifnum) {
		LOG_WRN("control: interface %u not found", ifnum);
		data->err = ERR_INVALID_UNIT;
		return -ENOTSUP;
	}

	/* VideoControl requests */

	if (entity_id == 0) {
		return uvc_control_errno(setup, buf, data->err);
	}

	for (const struct uvc_control *ctrl = conf->controls; ctrl->dev != NULL; ctrl++) {
		if (ctrl->entity_id == entity_id) {
			return uvc_control_vc(dev, ctrl, setup, buf);
		}
	}

	LOG_WRN("control: no unit %u found", entity_id);
	data->err = ERR_INVALID_UNIT;
	return -ENOTSUP;
}

static int uvc_control_to_host(struct usbd_class_data *const c_data,
			       const struct usb_setup_packet *const setup,
			       struct net_buf *const buf)
{
	errno = uvc_control(usbd_class_get_private(c_data), setup, buf);
	return 0;
}

static int uvc_control_to_dev(struct usbd_class_data *const c_data,
			      const struct usb_setup_packet *const setup,
			      const struct net_buf *const buf)
{
	errno = uvc_control(usbd_class_get_private(c_data), setup, (struct net_buf *)buf);
	return 0;
}

static int uvc_request(struct usbd_class_data *const c_data, struct net_buf *buf, int err)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_buf_info bi = *(struct uvc_buf_info *)udc_get_buf_info(buf);

	net_buf_unref(buf);

	if (bi.udc.ep == uvc_get_bulk_in(dev)) {
		if (bi.vbuf != NULL) {
			/* Upon completion, move the buffer from submission to completion queue */
			LOG_DBG("Request completed, vbuf %p transferred", bi.vbuf);
			k_fifo_put(&bi.stream->fifo_out, bi.vbuf);
		}
	} else {
		LOG_WRN("Request on unknown endpoint 0x%02x", bi.udc.ep);
	}

	return 0;
}

static void uvc_update(struct usbd_class_data *const c_data, uint8_t iface, uint8_t alternate)
{
	LOG_DBG("update");
}

static int uvc_init(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uvc_conf *conf = dev->config;
	int ret;

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		/* Get the default probe by querying the current probe at startup */
		ret = uvc_control_probe(dev, strm, GET_CUR, &strm->default_probe);
		if (ret < 0) {
			LOG_ERR("init: failed to query the default probe");
			return ret;
		}
	}

	return 0;
}

static void uvc_update_desc(const struct device *dev)
{
	const struct uvc_conf *conf = dev->config;

	*conf->desc_iad_ifnum = *conf->desc_vc_ifnum;
	*conf->desc_vc_header = *conf->desc_vc_ifnum;

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		*strm->desc_vs_epaddr = uvc_get_bulk_in(dev);
	}
}

static void *uvc_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uvc_conf *conf = dev->config;

	uvc_update_desc(dev);

	switch (speed) {
	case USBD_SPEED_FS:
		return (void *)conf->fs_desc;
	case USBD_SPEED_HS:
		return (void *)conf->hs_desc;
	default:
		__ASSERT_NO_MSG(false);
		return NULL;
	}
}

static int uvc_enqueue_usb(const struct device *dev, struct uvc_stream *strm, struct net_buf *buf, struct video_buffer *vbuf)
{
	const struct uvc_conf *conf = dev->config;
	size_t mps = uvc_get_bulk_mps(conf->c_data);
	struct uvc_buf_info *bi = (void *)udc_get_buf_info(buf);
	size_t len = buf->len;
	int ret;

	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->udc.zlp = (vbuf->flags & VIDEO_BUF_EOF) && buf->len == mps;
	bi->udc.ep = uvc_get_bulk_in(dev);
	/* If this is the last buffer, attach vbuf so we can free it from uvc_request() */
	bi->vbuf = (buf->len <= mps) ? vbuf : NULL;
	/* Reference to the stream to be able to find the correct FIFO */
	bi->stream = strm;

	/* Apply USB limit */
	buf->len = len = MIN(buf->len, mps);

	LOG_DBG("Queue USB buffer %p, data %p, size %u, len %u", buf, buf->data, buf->size,
		buf->len);

	ret = usbd_ep_enqueue(conf->c_data, buf);
	if (ret < 0) {
		return ret;
	}

	return len;
}

/* Here, the queue of video frame fragments is processed, each
 * fragment is prepended by the UVC header, and the result is cut into
 * USB packets submitted to the hardware:
 *
 *	frame: [vbuf: [header+payload, payload, payload, payload...],
 *		vbuf: [header+payload, payload, payload, payload...], ...],
 *	frame: [vbuf: [header+payload, payload, payload, payload...],
 *		vbuf: [header+payload, payload, payload, payload...], ...],
 *	frame: [vbuf: [header+payload, payload, payload, payload...],
 *		vbuf: [header+payload, payload, payload, payload...], ...],
 *	...
 */
static int uvc_queue_vbuf(const struct device *dev, struct uvc_stream *strm, struct video_buffer *vbuf)
{
	size_t xfer_len = CONFIG_USBD_VIDEO_HEADER_SIZE + vbuf->bytesused;
	struct net_buf *buf;
	int ret;

	if (strm->xfer_offset >= xfer_len) {
		strm->xfer_offset = 0;
		return 1;
	}

	LOG_DBG("Queue vbuf %p, offset %u/%u, flags 0x%02x", vbuf, strm->xfer_offset, xfer_len,
		vbuf->flags);

	/* Add another video frame an USB packet */
	buf = net_buf_alloc_with_data(&uvc_pool, vbuf->header + strm->xfer_offset,
				      xfer_len - strm->xfer_offset, K_NO_WAIT);
	if (buf == NULL) {
		LOG_ERR("Queue failed: cannot allocate USB buffer");
		return -ENOMEM;
	}

	if (strm->xfer_offset == 0) {
		LOG_INF("Queue start of frame, bmHeaderInfo 0x%02x",
			strm->payload_header.bmHeaderInfo);

		/* Only the 2 first 8-bit fields supported for now, the rest is padded with 0x00 */
		memcpy(vbuf->header, &strm->payload_header, 2);
		memset(vbuf->header + 2, 0, CONFIG_USBD_VIDEO_HEADER_SIZE - 2);

		if (vbuf->flags & VIDEO_BUF_EOF) {
			((struct uvc_payload_header *)vbuf->header)->bmHeaderInfo |=
				UVC_BMHEADERINFO_END_OF_FRAME;
		}

		if (vbuf->flags & VIDEO_BUF_EOF) {
			/* Toggle the Frame ID bit every new frame */
			strm->payload_header.bmHeaderInfo ^= UVC_BMHEADERINFO_FRAMEID;
		}
	}

	ret = uvc_enqueue_usb(dev, strm, buf, vbuf);
	if (ret < 0) {
		LOG_ERR("Queue to USB failed");
		strm->xfer_offset = 0;
		net_buf_unref(buf);
		return ret;
	}

	strm->xfer_offset += ret;
	return 0;
}

static void uvc_worker(struct k_work *work)
{
	struct uvc_stream *strm = CONTAINER_OF(work, struct uvc_stream, work);
	const struct device *dev = "TODO: find the parent device";
	struct video_buffer *vbuf;
	int ret;

	if (!atomic_test_bit(&strm->state, UVC_CLASS_ENABLED) ||
	    !atomic_test_bit(&strm->state, UVC_CLASS_READY)) {
		LOG_DBG("Queue not ready");
		return;
	}

	/* Only remove the buffer from the queue after it is submitted to USB */
	vbuf = k_fifo_peek_head(&strm->fifo_in);
	if (vbuf == NULL) {
		return;
	}

	if (vbuf->buffer - vbuf->header != CONFIG_USBD_VIDEO_HEADER_SIZE) {
		LOG_ERR("Queue expecting header of size %u", CONFIG_USBD_VIDEO_HEADER_SIZE);
		/* TODO: Submit a k_poll event mentioning the error */
		return;
	}

	ret = uvc_queue_vbuf(dev, strm, vbuf);
	if (ret < 0) {
		LOG_ERR("Queue vbuf %p failed", vbuf);
		/* TODO: Submit a k_poll event mentioning the error */
		return;
	}
	if (ret == 1) {
		/* Remove the buffer from the queue now that USB driver received it */
		k_fifo_get(&strm->fifo_in, K_NO_WAIT);
	}

	/* Work on the next buffer */
	k_work_submit(&strm->work);
}

static void uvc_enable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uvc_conf *conf = dev->config;

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		/* Catch-up with buffers that might have been delayed */
		atomic_set_bit(&strm->state, UVC_CLASS_ENABLED);
		k_work_submit(&strm->work);
	}
}

static void uvc_disable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct uvc_conf *conf = dev->config;

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		atomic_clear_bit(&strm->state, UVC_CLASS_ENABLED);
	}
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

USBD_DEFINE_CLASS(uvc_c_data, &uvc_class_api, (void *)DEVICE_DT_GET(DT_DRV_INST(0)), NULL);

static int uvc_enqueue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer *vbuf)
{
	struct uvc_stream *strm;
	int ret;

	ret = uvc_get_stream(dev, ep, &strm);
	if (ret < 0) {
		return ret;
	}

	k_fifo_put(&strm->fifo_in, vbuf);
	k_work_submit(&strm->work);
	return 0;
}

static int uvc_dequeue(const struct device *dev, enum video_endpoint_id ep,
		       struct video_buffer **vbuf, k_timeout_t timeout)
{
	struct uvc_stream *strm;
	int ret;

	ret = uvc_get_stream(dev, ep, &strm);
	if (ret < 0) {
		return ret;
	}

	*vbuf = k_fifo_get(&strm->fifo_out, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int uvc_get_format(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	const struct video_format_cap *cap;
	struct uvc_stream *strm;
	int ret;

	ret = uvc_get_stream(dev, ep, &strm);
	if (ret < 0) {
		return ret;
	}

	if (!atomic_test_bit(&strm->state, UVC_CLASS_ENABLED) ||
	    !atomic_test_bit(&strm->state, UVC_CLASS_READY)) {
		return -EAGAIN;
	}

	cap = &strm->caps[strm->format_id];

	memset(fmt, 0, sizeof(*fmt));
	fmt->pixelformat = cap->pixelformat;
	fmt->width = cap->width_max;
	fmt->height = cap->height_max;
	fmt->pitch = fmt->width * video_bits_per_pixel(cap->pixelformat);
	return 0;
}

static int uvc_stream_start(const struct device *dev)
{
	/* TODO: resume the stream after it was interrupted if needed */
	return 0;
}

static int uvc_stream_stop(const struct device *dev)
{
	/* TODO: cancel the ongoing USB request and stop the stream */
	return 0;
}

static int uvc_get_caps(const struct device *dev, enum video_endpoint_id ep,
			struct video_caps *caps)
{
	struct uvc_stream *strm;
	int ret;

	ret = uvc_get_stream(dev, ep, &strm);
	if (ret < 0) {
		return ret;
	}

	caps->format_caps = strm->caps;
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
	const struct uvc_conf *conf = dev->config;
	struct usbd_class_data *c_data = conf->c_data;
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	int ret;

	/* VideoStreaming initialization */

	for (struct uvc_stream *strm = conf->streams; strm->dev != NULL; strm++) {
		k_fifo_init(&strm->fifo_in);
		k_fifo_init(&strm->fifo_out);
		k_work_init(&strm->work, &uvc_worker);
	}

	/* VideoControl initialization */

	for (const struct uvc_control *ctrl = conf->controls; ctrl->dev != NULL; ctrl++) {
		struct usbd_desc_node *desc_nd = ctrl->desc_str;
		struct uvc_desc_entity *desc = (void *)ctrl->desc_ctl;

		ret = usbd_add_descriptor(uds_ctx, desc_nd);
		if (ret < 0) {
			LOG_ERR("Failed to add string descriptor %s to %s",
				(char *)desc_nd->ptr, dev->name);
			return ret;
		}

		switch (desc->bDescriptorSubtype) {
		case VC_INPUT_TERMINAL:
			((uint8_t *)desc)[VC_CT_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_OUTPUT_TERMINAL:
			((uint8_t *)desc)[VC_OT_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_SELECTOR_UNIT:
			((uint8_t *)desc)[VC_SU_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_PROCESSING_UNIT:
			((uint8_t *)desc)[VC_PU_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_ENCODING_UNIT:
			((uint8_t *)desc)[VC_EU_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		case VC_EXTENSION_UNIT:
			((uint8_t *)desc)[VC_XU_STRING_OFFSET(desc->bLength)] = desc_nd->str.idx;
			break;
		default:
			LOG_WRN("Not adding '%s' to unknown subtype %u",
				(char *)desc_nd->ptr, desc->bEntityID);
			break;
		}
	}

	return 0;
}

#define INTERFACE_ASSOCIATION_DESCRIPTOR_ARRAYS(node)				\
	static uint8_t uvc_desc_iad[] = {					\
		INTERFACE_ASSOCIATION_DESCRIPTOR(node)				\
	};

#define VC_INTERFACE_DESCRIPTOR_ARRAYS(node)					\
	static uint8_t uvc_desc_vc_if[] = {					\
		VC_INTERFACE_DESCRIPTOR(node)					\
	};

#define VC_INTERFACE_HEADER_DESCRIPTOR_ARRAYS(node)				\
	static uint8_t uvc_desc_vc_header[] = {					\
		VC_INTERFACE_HEADER_DESCRIPTOR(node)				\
	};

#define VC_DESCRIPTOR_ARRAYS(node)						\
	static uint8_t DT_CAT(node, _desc)[] = {				\
		VC_DESCRIPTOR(node)						\
	};

#define VS_DESCRIPTOR_ARRAYS(node)						\
	VS_INTERFACE_DESCRIPTOR_ARRAYS(node)					\
	VS_INTERFACE_HEADER_DESCRIPTOR_ARRAYS(node)				\
	VS_FOREACH_FORMAT(node, VS_UNCOMP_DESCRIPTOR_ARRAYS)

#define VS_INTERFACE_DESCRIPTOR_ARRAYS(node)					\
	static uint8_t DT_CAT(node, _desc_if)[] = {				\
		VS_INTERFACE_DESCRIPTOR(node)					\
	};

#define VS_INTERFACE_HEADER_DESCRIPTOR_ARRAYS(node)				\
	static uint8_t DT_CAT(node, _desc_header)[] = {				\
		VS_INPUT_HEADER_DESCRIPTOR(node)				\
	};									\

#define VS_UNCOMP_DESCRIPTOR_ARRAYS(node, prop, id)				\
	static const uint8_t DT_CAT4(node, _desc_uncomp_, id, _format)[] = {	\
		VS_UNCOMPRESSED_FORMAT_DESCRIPTOR(node, prop, id)		\
	};									\
	static const uint8_t DT_CAT4(node, _desc_uncomp_, id, _frame)[] = {	\
		VS_UNCOMPRESSED_FRAME_DESCRIPTOR(node, prop, id)		\
	};

#define VS_MJPEG_DESCRIPTOR_ARRAYS(node, prop, id)				\
	static const uint8_t DT_CAT4(node, _desc_mjpeg_, id, _format)[] = {	\
		VS_MJPEG_FORMAT_DESCRIPTOR(node, prop, id)			\
	};									\
	static const uint8_t DT_CAT4(node, _desc_mjpeg_, id, _frame)[] = {	\
		VS_MJPEG_FRAME_DESCRIPTOR(node, prop, id)			\
	};

#define VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR_ARRAYS(node)			\
	static uint8_t uvc_desc_ep_fs[] = {					\
		VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR(node)			\
	};

#define VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR_ARRAYS(node)			\
	static uint8_t uvc_desc_ep_hs[] = {					\
		VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR(node)			\
	};

#define VS_COLOR_MATCHING_DESCRIPTOR_ARRAYS(node)				\
	static uint8_t uvc_desc_color[] = {					\
		VS_COLOR_MATCHING_DESCRIPTOR(node)				\
	};

#define UVC_DESCRIPTOR_ARRAYS(node)						\
	INTERFACE_ASSOCIATION_DESCRIPTOR_ARRAYS(node)				\
	VC_INTERFACE_DESCRIPTOR_ARRAYS(node)					\
	VC_INTERFACE_HEADER_DESCRIPTOR_ARRAYS(node)				\
	VC_FOREACH_ENTITY(VC_DESCRIPTOR_ARRAYS)					\
	VS_FOREACH_STREAM(VS_DESCRIPTOR_ARRAYS)					\
	VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR_ARRAYS(node)			\
	VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR_ARRAYS(node)			\
	VS_COLOR_MATCHING_DESCRIPTOR_ARRAYS(node)

UVC_DESCRIPTOR_ARRAYS(DT_DRV_INST(0))

#define UVC_DESCRIPTOR_PTRS(node, type)						\
	(struct usb_desc_header *)uvc_desc_iad,					\
	(struct usb_desc_header *)uvc_desc_vc_if,				\
	(struct usb_desc_header *)uvc_desc_vc_header,				\
	VC_FOREACH_ENTITY(VC_DESCRIPTOR_PTRS)					\
	VS_FOREACH_STREAM(VS_DESCRIPTOR_PTRS)					\
	VS_##type##_ENDPOINT_DESCRIPTOR_PTRS(node)

#define VC_DESCRIPTOR_PTRS(node)						\
	(struct usb_desc_header *)DT_CAT(node, _desc),

#define VS_DESCRIPTOR_PTRS(node)						\
	(struct usb_desc_header *)DT_CAT(node, _desc_if),			\
	(struct usb_desc_header *)DT_CAT(node, _desc_header),			\
	VS_FOREACH_FORMAT(node, VS_UNCOMP_DESCRIPTOR_PTRS)

#define VS_UNCOMP_DESCRIPTOR_PTRS(node, prop, id)				\
	(struct usb_desc_header *)DT_CAT4(node, _desc_uncomp_, id, _format),	\
	(struct usb_desc_header *)DT_CAT4(node, _desc_uncomp_, id, _frame),	\
	(struct usb_desc_header *)uvc_desc_color,

#define VS_MJPEG_DESCRIPTOR_PTRS(node, prop, id)				\
	(struct usb_desc_header *)DT_CAT4(node, _desc_mjpeg_, id, _format),	\
	(struct usb_desc_header *)DT_CAT4(node, _desc_mjpeg_, id, _frame),	\
	(struct usb_desc_header *)uvc_desc_color,

#define VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR_PTRS(node)			\
	(struct usb_desc_header *)uvc_desc_ep_fs,

#define VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR_PTRS(node)			\
	(struct usb_desc_header *)uvc_desc_ep_hs,

static struct usb_desc_header *uvc_fs_desc[] = {
	UVC_DESCRIPTOR_PTRS(DT_DRV_INST(0), FULLSPEED_BULK)
	NULL,
};
static struct usb_desc_header *uvc_hs_desc[] = {
	UVC_DESCRIPTOR_PTRS(DT_DRV_INST(0), HIGHSPEED_BULK)
	NULL,
};

#define VC_DESCRIPTOR_STRING(node)						\
	USBD_DESC_STRING_DEFINE(node ## _desc_str, DT_NODE_FULL_NAME(node),	\
				USBD_DUT_STRING_INTERFACE);
VC_FOREACH_ENTITY(VC_DESCRIPTOR_STRING)

#define UVC_CAPABILITY(node, prop, id)						\
	{.pixelformat = DT_PHA_BY_IDX(node, prop, id, fourcc),			\
	 .width_min = DT_PHA_BY_IDX(node, prop, id, width),			\
	 .width_max = DT_PHA_BY_IDX(node, prop, id, width),			\
	 .width_step = 1,							\
	 .height_min = DT_PHA_BY_IDX(node, prop, id, height),			\
	 .height_max = DT_PHA_BY_IDX(node, prop, id, height),			\
	 .height_step = 1},
#define UVC_STREAM_CAPS(node)							\
	static const struct video_format_cap DT_CAT(node, _caps)[] = {		\
		VS_FOREACH_FORMAT(node, UVC_CAPABILITY)				\
		{0},								\
	};
VS_FOREACH_STREAM(UVC_STREAM_CAPS)

#define UVC_STREAM(node)							\
	{.dev = DEVICE_DT_GET(DT_REMOTE_DEVICE(node)),				\
	 .desc_vs_ifnum = DT_CAT(node, _desc_if) + 2,				\
	 .desc_vs_epaddr = DT_CAT(node, _desc_header) + 6,			\
	 .payload_header.bHeaderLength = CONFIG_USBD_VIDEO_HEADER_SIZE,		\
	 .caps = DT_CAT(node, _caps)},
static struct uvc_stream uvc_streams[] = {
	VS_FOREACH_STREAM(UVC_STREAM)
	{0},
};

#define UVC_CONTROL(node)							\
	{.dev = DEVICE_DT_GET(node),						\
	 .entity_id = NODE_ID(node),						\
	 .fn = VC_HANDLER(node),						\
	 .desc_str = &DT_CAT(node, _desc_str),					\
	 .desc_ctl = (struct usb_desc_header *)DT_CAT(node, _desc)},
static const struct uvc_control uvc_controls[] = {
	VC_FOREACH_ENTITY(UVC_CONTROL)
	{0},
};

static const struct uvc_conf uvc_conf = {
	.c_data = &uvc_c_data,
	.controls = uvc_controls,
	.streams = uvc_streams,
	.fs_desc = uvc_fs_desc,
	.hs_desc = uvc_hs_desc,
	.desc_iad_ifnum = uvc_desc_iad + 2,
	.desc_vc_ifnum = uvc_desc_vc_if + 2,
	.desc_vc_header = uvc_desc_vc_header + 12,
	.desc_fs_epaddr = uvc_desc_ep_fs + 2,
	.desc_hs_epaddr = uvc_desc_ep_hs + 2,
};

static struct uvc_data uvc_data = {
	.err = 0,
};

BUILD_ASSERT(DT_INST_ON_BUS(0, usb), "Not assigned to a USB device controller");

DEVICE_DT_INST_DEFINE(0, uvc_preinit, NULL, &uvc_data, &uvc_conf, POST_KERNEL,
		      CONFIG_VIDEO_INIT_PRIORITY, &uvc_video_api);
