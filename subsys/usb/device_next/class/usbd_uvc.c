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
#define RC_UNDEFINED				0x00
#define SET_CUR					0x01
#define GET_CUR					0x81
#define GET_MIN					0x82
#define GET_MAX					0x83
#define GET_RES					0x84
#define GET_LEN					0x85
#define GET_INFO				0x86
#define GET_DEF					0x87

/* Flags announcing which controls are supported */
#define INFO_SUPPORTS_GET			BIT(0)
#define INFO_SUPPORTS_SET			BIT(1)

/* Selector Unit Control Selectors */
#define SU_INPUT_SELECT_CONTROL			0x01

/* Camera Terminal Control Selectors */
#define CT_SCANNING_MODE_CONTROL		0x01
#define CT_AE_MODE_CONTROL			0x02
#define CT_AE_PRIORITY_CONTROL			0x03
#define CT_EXPOSURE_TIME_ABSOLUTE_CONTROL	0x04
#define CT_EXPOSURE_TIME_RELATIVE_CONTROL	0x05
#define CT_FOCUS_ABSOLUTE_CONTROL		0x06
#define CT_FOCUS_RELATIVE_CONTROL		0x07
#define CT_FOCUS_AUTO_CONTROL			0x08
#define CT_IRIS_ABSOLUTE_CONTROL		0x09
#define CT_IRIS_RELATIVE_CONTROL		0x0A
#define CT_ZOOM_ABSOLUTE_CONTROL		0x0B
#define CT_ZOOM_RELATIVE_CONTROL		0x0C
#define CT_PANTILT_ABSOLUTE_CONTROL		0x0D
#define CT_PANTILT_RELATIVE_CONTROL		0x0E
#define CT_ROLL_ABSOLUTE_CONTROL		0x0F
#define CT_ROLL_RELATIVE_CONTROL		0x10
#define CT_PRIVACY_CONTROL			0x11
#define CT_FOCUS_SIMPLE_CONTROL			0x12
#define CT_WINDOW_CONTROL			0x13
#define CT_REGION_OF_INTEREST_CONTROL		0x14

/* Processing Unit Control Selectors */
#define PU_BACKLIGHT_COMPENSATION_CONTROL	0x01
#define PU_BRIGHTNESS_CONTROL			0x02
#define PU_CONTRAST_CONTROL			0x03
#define PU_GAIN_CONTROL				0x04
#define PU_POWER_LINE_FREQUENCY_CONTROL		0x05
#define PU_HUE_CONTROL				0x06
#define PU_SATURATION_CONTROL			0x07
#define PU_SHARPNESS_CONTROL			0x08
#define PU_GAMMA_CONTROL			0x09
#define PU_WHITE_BALANCE_TEMPERATURE_CONTROL	0x0A
#define PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL 0x0B
#define PU_WHITE_BALANCE_COMPONENT_CONTROL	0x0C
#define PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL	0x0D
#define PU_DIGITAL_MULTIPLIER_CONTROL		0x0E
#define PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL	0x0F
#define PU_HUE_AUTO_CONTROL			0x10
#define PU_ANALOG_VIDEO_STANDARD_CONTROL	0x11
#define PU_ANALOG_LOCK_STATUS_CONTROL		0x12
#define PU_CONTRAST_AUTO_CONTROL		0x13

/* Encoding Unit Control Selectors */
#define EU_SELECT_LAYER_CONTROL			0x01
#define EU_PROFILE_TOOLSET_CONTROL		0x02
#define EU_VIDEO_RESOLUTION_CONTROL		0x03
#define EU_MIN_FRAME_INTERVAL_CONTROL		0x04
#define EU_SLICE_MODE_CONTROL			0x05
#define EU_RATE_CONTROL_MODE_CONTROL		0x06
#define EU_AVERAGE_BITRATE_CONTROL		0x07
#define EU_CPB_SIZE_CONTROL			0x08
#define EU_PEAK_BIT_RATE_CONTROL		0x09
#define EU_QUANTIZATION_PARAMS_CONTROL		0x0A
#define EU_SYNC_REF_FRAME_CONTROL		0x0B
#define EU_LTR_BUFFER_CONTROL			0x0C
#define EU_LTR_PICTURE_CONTROL			0x0D
#define EU_LTR_VALIDATION_CONTROL		0x0E
#define EU_LEVEL_IDC_LIMIT_CONTROL		0x0F
#define EU_SEI_PAYLOADTYPE_CONTROL		0x10
#define EU_QP_RANGE_CONTROL			0x11
#define EU_PRIORITY_CONTROL			0x12
#define EU_START_OR_STOP_LAYER_CONTROL		0x13
#define EU_ERROR_RESILIENCY_CONTROL		0x14

/* VideoStreaming Interface Control Selectors */
#define VS_PROBE_CONTROL			0x01
#define VS_COMMIT_CONTROL			0x02
#define VS_STILL_PROBE_CONTROL			0x03
#define VS_STILL_COMMIT_CONTROL			0x04
#define VS_STILL_IMAGE_TRIGGER_CONTROL		0x05
#define VS_STREAM_ERROR_CODE_CONTROL		0x06
#define VS_GENERATE_KEY_FRAME_CONTROL		0x07
#define VS_UPDATE_FRAME_SEGMENT_CONTROL		0x08
#define VS_SYNCH_DELAY_CONTROL			0x09

struct uvc_data;

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
#define UVC_BMFRAMING_INFO_FID			BIT(0)
#define UVC_BMFRAMING_INFO_EOF			BIT(1)
#define UVC_BMFRAMING_INFO_EOS			BIT(2)
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
	uint32_t dwPresentationTime;	/* optional */
	uint32_t scrSourceClockSTC;	/* optional */
	uint16_t scrSourceClockSOF;	/* optional */
} __packed;

/* Lookup table between the format and frame index, and the video caps */
struct uvc_format {
	uint8_t bFormatIndex;
	uint8_t bFrameIndex;
	uint8_t bits_per_pixel;
	uint32_t frame_interval;	/* see #72254 */
};

/* Lookup table between the interface ID and the control function */
struct uvc_control {
	uint8_t entity_id;
	int (*fn)(const struct usb_setup_packet *, struct net_buf *, const struct device *);
	const struct device *target;
};

struct uvc_data {
	/* USBD class structure */
	const struct usbd_class_data *c_data;
	/* USBD class state */
	atomic_t state;
	/* UVC worker to process the queue */
	struct k_work work;
	/* UVC format lookup tables */
	int format_id;
	const struct video_format_cap *caps;
	const struct uvc_format *formats;
	/* UVC control lookup table */
	const struct uvc_control *controls;
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
	struct uvc_probe default_probe;
	/* UVC payload header, passed just before the image data */
	struct uvc_payload_header payload_header;
	/* Video device controlled by the host via UVC */
	const struct device *output_terminal;
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

static int uvc_get_control_max(uint16_t size)
{
	switch (size) {
	case 4:
		return INT32_MAX;
	case 2:
		return INT16_MAX;
	case 1:
		return INT8_MAX;
	default:
		return 0;
	}
}

static int uvc_buf_add(struct net_buf *buf, uint16_t size, uint32_t value)
{
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
	}
	return 0;
}

static uint32_t uvc_get_max_frame_size(struct uvc_data *data, int id)
{
	const struct video_format_cap *cap = &data->caps[id];
	const struct uvc_format *ufmt = &data->formats[id];

	return cap->width_max * cap->height_max * ufmt->bits_per_pixel / 8;
}

static int uvc_control_probe_format_index(struct uvc_data *data, uint8_t request,
					  struct uvc_probe *probe)
{
	switch (request) {
	case GET_MIN:
		probe->bFormatIndex = UINT8_MAX;
		for (const struct uvc_format *fmt = data->formats; fmt->bFormatIndex; fmt++) {
			probe->bFormatIndex = MIN(fmt->bFormatIndex, probe->bFormatIndex);
		}
		break;
	case GET_MAX:
		probe->bFormatIndex = 0;
		for (const struct uvc_format *fmt = data->formats; fmt->bFormatIndex; fmt++) {
			probe->bFormatIndex = MAX(fmt->bFormatIndex, probe->bFormatIndex);
		}
		break;
	case GET_RES:
		probe->bFormatIndex = 1;
		break;
	case GET_CUR:
		probe->bFormatIndex = data->formats[data->format_id].bFormatIndex;
		break;
	case SET_CUR:
		if (probe->bFormatIndex == 0) {
			return 0;
		}
		for (size_t i = 0; data->formats[i].bFormatIndex; i++) {
			if (data->formats[i].bFormatIndex == probe->bFormatIndex) {
				data->format_id = i;
				return 0;
			}
		}
		LOG_WRN("probe: format index %u not found", probe->bFormatIndex);
		return -ENOTSUP;
	}
	return 0;
}

static int uvc_control_probe_frame_index(struct uvc_data *data, uint8_t request,
					 struct uvc_probe *probe)
{
	const struct uvc_format *cur = &data->formats[data->format_id];

	switch (request) {
	case GET_MIN:
		probe->bFrameIndex = UINT8_MAX;
		for (const struct uvc_format *fmt = data->formats; fmt->bFormatIndex; fmt++) {
			if (fmt->bFormatIndex == cur->bFormatIndex) {
				probe->bFrameIndex = MIN(fmt->bFrameIndex, probe->bFrameIndex);
			}
		}
		break;
	case GET_MAX:
		probe->bFrameIndex = 0;
		for (const struct uvc_format *fmt = data->formats; fmt->bFormatIndex; fmt++) {
			if (fmt->bFormatIndex == cur->bFormatIndex) {
				probe->bFrameIndex = MAX(fmt->bFrameIndex, probe->bFrameIndex);
			}
		}
		break;
	case GET_RES:
		probe->bFrameIndex = 1;
		break;
	case GET_CUR:
		probe->bFrameIndex = cur->bFrameIndex;
		break;
	case SET_CUR:
		if (probe->bFrameIndex == 0) {
			return 0;
		}
		for (size_t i = 0; data->formats[i].bFormatIndex; i++) {
			const struct uvc_format *fmt = &data->formats[i];

			LOG_DBG("fmt->bFormatIndex=%u cur->bFormatIndex=%u",
				fmt->bFormatIndex, cur->bFormatIndex);
			LOG_DBG("fmt->bFrameIndex=%u probe->bFrameIndex=%u",
				fmt->bFrameIndex, probe->bFrameIndex);
			if (fmt->bFormatIndex == cur->bFormatIndex &&
			    fmt->bFrameIndex == probe->bFrameIndex) {
				data->format_id = i;
				return 0;
			}
		}
		LOG_WRN("probe: frame index %u not found", probe->bFrameIndex);
		return -ENOTSUP;
	}
	return 0;
}

static int uvc_control_probe_frame_interval(struct uvc_data *data, uint8_t request,
					    struct uvc_probe *probe)
{
	uint32_t frmival = data->formats[data->format_id].frame_interval;

	switch (request) {
	case GET_MIN:
	case GET_MAX:
		/* TODO call the frame interval API on the video source once supported */
		probe->dwFrameInterval = sys_cpu_to_le32(frmival);
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

static int uvc_control_probe_max_video_frame_size(struct uvc_data *data, uint8_t request,
						  struct uvc_probe *probe)
{
	uint32_t max_frame_size = uvc_get_max_frame_size(data, data->format_id);

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

static int uvc_control_probe_max_payload_size(struct uvc_data *data, uint8_t request,
					      struct uvc_probe *probe)
{
	uint32_t max_payload_size =
		uvc_get_max_frame_size(data, data->format_id) + CONFIG_USBD_VIDEO_HEADER_SIZE;

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

static int uvc_control_probe(struct uvc_data *data, uint8_t request, struct uvc_probe *probe)
{
	int err;

	switch (request) {
	case GET_MIN:
	case GET_MAX:
	case GET_RES:
	case GET_CUR:
	case SET_CUR:
		break;
	default:
		LOG_WRN("control: invalid bRequest %u", request);
		return -EINVAL;
	}

	/* Static or unimplemented fields */
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

	/* Dynamic fields */
	if ((err = uvc_control_probe_format_index(data, request, probe)) ||
	    (err = uvc_control_probe_frame_index(data, request, probe)) ||
	    (err = uvc_control_probe_frame_interval(data, request, probe)) ||
	    (err = uvc_control_probe_max_video_frame_size(data, request, probe)) ||
	    (err = uvc_control_probe_max_payload_size(data, request, probe))) {
		return err;
	}

	LOG_DBG("- bmHint: 0x%04x", sys_le16_to_cpu(probe->bmHint));
	LOG_DBG("- bFormatIndex: %u", probe->bFormatIndex);
	LOG_DBG("- bFrameIndex: %u", probe->bFrameIndex);
	LOG_DBG("- dwFrameInterval: %u us", sys_le32_to_cpu(probe->dwFrameInterval) / 10);
	LOG_DBG("- wKeyFrameRate: %u", sys_le16_to_cpu(probe->wKeyFrameRate));
	LOG_DBG("- wPFrameRate: %u", sys_le16_to_cpu(probe->wPFrameRate));
	LOG_DBG("- wCompQuality: %u", sys_le16_to_cpu(probe->wCompQuality));
	LOG_DBG("- wCompWindowSize: %u", sys_le16_to_cpu(probe->wCompWindowSize));
	LOG_DBG("- wDelay: %u ms", sys_le16_to_cpu(probe->wDelay));
	LOG_DBG("- dwMaxVideoFrameSize: %u", sys_le32_to_cpu(probe->dwMaxVideoFrameSize));
	LOG_DBG("- dwMaxPayloadTransferSize: %u", sys_le32_to_cpu(probe->dwMaxPayloadTransferSize));
	LOG_DBG("- dwClockFrequency: %u Hz", sys_le32_to_cpu(probe->dwClockFrequency));
	LOG_DBG("- bmFramingInfo: 0x%02x", probe->bmFramingInfo);
	LOG_DBG("- bPreferedVersion: %u", probe->bPreferedVersion);
	LOG_DBG("- bMinVersion: %u", probe->bMinVersion);
	LOG_DBG("- bMaxVersion: %u", probe->bMaxVersion);
	LOG_DBG("- bUsage: %u", probe->bUsage);
	LOG_DBG("- bBitDepthLuma: %u", probe->bBitDepthLuma + 8);
	LOG_DBG("- bmSettings: %u", probe->bmSettings);
	LOG_DBG("- bMaxNumberOfRefFramesPlus1: %u", probe->bMaxNumberOfRefFramesPlus1);
	LOG_DBG("- bmRateControlModes: %u", probe->bmRateControlModes);
	LOG_DBG("- bmLayoutPerStream: 0x%08llx", probe->bmLayoutPerStream);

	return 0;
}

static int uvc_control_commit(struct uvc_data *data, uint8_t request, struct uvc_probe *probe)
{
	int err;

	switch (request) {
	case GET_CUR:
		return uvc_control_probe(data, request, probe);
	case SET_CUR:
		err = uvc_control_probe(data, request, probe);
		if (err) {
			return err;
		}

		if (data->output_terminal != NULL) {
			const struct device *dev = usbd_class_get_private(data->c_data);
			struct video_format fmt = {0};

			err = uvc_get_format(dev, VIDEO_EP_IN, &fmt);
			if (err) {
				LOG_ERR("Failed to inquire the current UVC format");
				return err;
			}

			LOG_DBG("control: setting source format to %ux%u", fmt.width, fmt.height);

			err = video_set_format(data->output_terminal, VIDEO_EP_OUT, &fmt);
			if (err) {
				LOG_ERR("Could not set the format of the video source");
				return err;
			}

			err = video_stream_start(data->output_terminal);
			if (err) {
				LOG_ERR("Could not start the video source");
				return err;
			}
		}

		LOG_INF("control: ready to transfer frames");
		atomic_set_bit(&data->state, UVC_CLASS_READY);
		k_work_submit(&data->work);
		break;
	default:
		LOG_WRN("commit: invalid bRequest %u", request);
		return -EINVAL;
	}
	return 0;
}

static int uvc_control_streaming(struct uvc_data *data, const struct usb_setup_packet *setup,
			      struct net_buf *buf)
{
	uint8_t control_selector = setup->wValue >> 8;
	struct uvc_probe *probe = (void *)buf->data;

	LOG_DBG("control: streaming interface, selector %u, request %u",
		control_selector, setup->bRequest);

	switch (setup->bRequest) {
	case GET_INFO:
		if (setup->wLength != 1 || buf->size < 1) {
			LOG_ERR("control: bad wLength %u or bufsize %u", setup->wLength, buf->size);
			return -EINVAL;
		}
		return uvc_buf_add(buf, setup->wLength, INFO_SUPPORTS_GET | INFO_SUPPORTS_SET);
	case GET_LEN:
		if (setup->wLength != 2 || buf->size < 2) {
			LOG_ERR("control: bad wLength %u or bufsize %u", setup->wLength, buf->size);
			return -EINVAL;
		}
		net_buf_add_le16(buf, sizeof(probe));
		return 0;
	case GET_DEF:
		LOG_INF("wLength=%u len=%u size=%u probe=%u", setup->wLength, buf->len, buf->size, sizeof(*probe));
		if (setup->wLength != sizeof(*probe) || buf->size < sizeof(*probe)) {
			LOG_ERR("control: bad wLength %u or bufsize %u", setup->wLength, buf->size);
			return -EINVAL;
		}
		net_buf_add_mem(buf, &data->default_probe, sizeof(*probe));
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
		return uvc_control_probe(data, setup->bRequest, probe);
	case VS_COMMIT_CONTROL:
		return uvc_control_commit(data, setup->bRequest, probe);
	default:
		LOG_WRN("control: unknown selector %u for streaming interface", control_selector);
		return -ENOTSUP;
	}
}

static int uvc_control_fix(const struct usb_setup_packet *setup, struct net_buf *buf, uint8_t size,
			   uint32_t value)
{
	size = MIN(setup->wLength, size);
	LOG_DBG("control: fixed type control, size %u", size);

	switch (setup->bRequest) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET | INFO_SUPPORTS_SET);
	case GET_RES:
		return uvc_buf_add(buf, size, 1);
	case GET_DEF:
	case GET_MIN:
	case GET_MAX:
	case GET_CUR:
		return uvc_buf_add(buf, size, value);
	case SET_CUR:
		return 0;
	default:
		LOG_WRN("control: unsupported request type %u", setup->bRequest);
		return -ENOTSUP;
	}
}

static int uvc_control_int(const struct usb_setup_packet *setup, struct net_buf *buf, uint8_t size,
			   const struct device *dev, uint32_t cid)
{
	uint32_t value;
	int err;

	size = MIN(setup->wLength, size);
	LOG_DBG("control: integer type control, size %u", size);

	switch (setup->bRequest) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET | INFO_SUPPORTS_SET);
	case GET_RES:
		return uvc_buf_add(buf, size, 1);
	case GET_DEF:
		LOG_DBG("%s GET_DEF size=%u", __func__, size);
		return uvc_buf_add(buf, size, 1);
	case GET_MIN:
		return uvc_buf_add(buf, size, 0);
	case GET_MAX:
		return uvc_buf_add(buf, size, uvc_get_control_max(size));
	case GET_CUR:
		err = video_get_ctrl(dev, cid, &value);
		if (err) {
			LOG_ERR("control: failed to query target video device");
			return err;
		}
		return uvc_buf_add(buf, size, value);
	case SET_CUR:
		err = uvc_buf_remove(buf, size, &value);
		if (err) {
			return err;
		}
		err = video_set_ctrl(dev, cid, (void *)value);
		if (err) {
			LOG_ERR("control: failed to configure target video device");
			return err;
		}
		return 0;
	default:
		LOG_WRN("control: unsupported request type %u", setup->bRequest);
		return -ENOTSUP;
	}
}

static int zephyr_uvc_control_camera(const struct usb_setup_packet *setup, struct net_buf *buf,
				     const struct device *dev)
{
	uint8_t control_selector = setup->wValue >> 8;

	LOG_DBG("control: camera terminal, selector %u, request %u",
		control_selector, setup->bRequest);

	switch (control_selector) {
	case CT_AE_MODE_CONTROL:
		return uvc_control_fix(setup, buf, 1, BIT(0));
	case CT_AE_PRIORITY_CONTROL:
		return uvc_control_fix(setup, buf, 1, 0);
	case CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
		return uvc_control_int(setup, buf, 4, dev, VIDEO_CID_CAMERA_EXPOSURE);
	case CT_ZOOM_ABSOLUTE_CONTROL:
		return uvc_control_int(setup, buf, 2, dev, VIDEO_CID_CAMERA_ZOOM);
	default:
		LOG_WRN("control: unsupported selector %u for camera terminal ", control_selector);
		return -ENOTSUP;
	}
}

static int zephyr_uvc_control_processing(const struct usb_setup_packet *setup, struct net_buf *buf,
					 const struct device *dev)
{
	uint8_t control_selector = setup->wValue >> 8;

	LOG_DBG("control: processing unit, selector %u, request %u",
		control_selector, setup->bRequest);

	switch (control_selector) {
	case PU_BRIGHTNESS_CONTROL:
		return uvc_control_int(setup, buf, 2, dev, VIDEO_CID_CAMERA_BRIGHTNESS);
	case PU_CONTRAST_CONTROL:
		return uvc_control_int(setup, buf, 1, dev, VIDEO_CID_CAMERA_CONTRAST);
	case PU_GAIN_CONTROL:
		return uvc_control_int(setup, buf, 2, dev, VIDEO_CID_CAMERA_GAIN);
	case PU_SATURATION_CONTROL:
		return uvc_control_int(setup, buf, 2, dev, VIDEO_CID_CAMERA_SATURATION);
	case PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
		return uvc_control_int(setup, buf, 2, dev, VIDEO_CID_CAMERA_WHITE_BAL);
	default:
		LOG_WRN("control: unsupported selector %u for processing unit", control_selector);
		return -ENOTSUP;
	}
}

static int zephyr_uvc_control_output(const struct usb_setup_packet *setup, struct net_buf *buf,
				     const struct device *dev)
{
	LOG_WRN("control: nothing supported for output terminal");
	return -ENOTSUP;
};

static int uvc_control(struct usbd_class_data *c_data, const struct usb_setup_packet *const setup,
		       struct net_buf *const buf)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;
	uint8_t interface = (setup->wIndex >> 0) & 0xff;
	uint8_t entity_id = (setup->wIndex >> 8) & 0xff;

	switch (setup->bRequest) {
	case SET_CUR: LOG_DBG("SET_CUR"); break;
	case GET_CUR: LOG_DBG("GET_CUR"); break;
	case GET_MIN: LOG_DBG("GET_MIN"); break;
	case GET_MAX: LOG_DBG("GET_MAX"); break;
	case GET_RES: LOG_DBG("GET_RES"); break;
	case GET_LEN: LOG_DBG("GET_LEN"); break;
	case GET_INFO: LOG_DBG("GET_INFO"); break;
	case GET_DEF: LOG_DBG("GET_DEF"); break;
	default: LOG_WRN("control: unknown"); break;
	}

	if (interface == *data->desc_if_vs_ifnum) {
		return uvc_control_streaming(data, setup, buf);
	}

	if (interface == *data->desc_if_vc_ifnum) {
		for (const struct uvc_control *p = data->controls; p->entity_id != 0; p++) {
			if (p->entity_id == entity_id) {
				LOG_DBG("control: found video CIDs for bEntityID %u", entity_id);
				return p->fn(setup, buf, p->target);
			}
		}
	}

	LOG_WRN("control: no entity %u found for interface %u", entity_id, interface);
	return -ENOTSUP;
}

static int uvc_control_to_host(struct usbd_class_data *const c_data,
			       const struct usb_setup_packet *const setup,
			       struct net_buf *const buf)
{
	errno = uvc_control(c_data, setup, buf);
	return 0;
}

static int uvc_control_to_dev(struct usbd_class_data *const c_data,
			      const struct usb_setup_packet *const setup,
			      const struct net_buf *const buf)
{
	errno = uvc_control(c_data, setup, (struct net_buf *const)buf);
	return 0;
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
	int err;

	/* Get the default probe by querying the current probe at startup */
	err = uvc_control_probe(data, GET_CUR, &data->default_probe);
	if (err) {
		LOG_ERR("init: failed to query the default probe");
		return err;
	}

	return 0;
}

static void uvc_update_desc(struct uvc_data *data)
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

#define UVC_CAP_ENTRY(frame, format)						\
	{									\
		.pixelformat = video_fourcc(DT_PROP(format, fourcc)[0],		\
					    DT_PROP(format, fourcc)[1],		\
					    DT_PROP(format, fourcc)[2],		\
					    DT_PROP(format, fourcc)[3]),	\
		.width_min = DT_PROP_BY_IDX(frame, size, 0),			\
		.width_max = DT_PROP_BY_IDX(frame, size, 0),			\
		.width_step = 0,						\
		.height_min = DT_PROP_BY_IDX(frame, size, 1),			\
		.height_max = DT_PROP_BY_IDX(frame, size, 1),			\
		.height_step = 0						\
	},

#define UVC_CAPS(format)							\
	IF_DISABLED(IS_EMPTY(VS_DESCRIPTOR(format)), (				\
		DT_FOREACH_CHILD_VARGS(format, UVC_CAP_ENTRY, format)		\
	))

#define UVC_FORMAT_ENTRY(frame, format)						\
	{									\
		.bFormatIndex = NODE_ID(format),				\
		.bFrameIndex = NODE_ID(frame),					\
		.frame_interval = 0,						\
		.bits_per_pixel = DT_PROP(format, bits_per_pixel),		\
	},

#define UVC_FORMATS(format)							\
	IF_DISABLED(IS_EMPTY(VS_DESCRIPTOR(format)), (				\
		DT_FOREACH_CHILD_VARGS(format, UVC_FORMAT_ENTRY, format)	\
	))

#define UVC_CONTROL(node)							\
	IF_DISABLED(IS_EMPTY(VC_DESCRIPTOR(node)), ({				\
		.entity_id = NODE_ID(node),					\
		.fn = &DT_STRING_TOKEN_BY_IDX(node, compatible, 0),		\
		.target = DEVICE_DT_GET(DT_PHANDLE(node, control_target)),	\
	},))

#define OUTPUT_TERMINAL(node)							\
	DT_PHANDLE(LOOKUP_NODE(node, zephyr_uvc_control_output), control_target)

#define UVC_DEVICE_DEFINE(node)							\
										\
	UVC_DESCRIPTOR_ARRAYS(node)						\
										\
	static struct usb_desc_header *node##_fs_desc[] = {			\
		UVC_FULLSPEED_DESCRIPTOR_PTRS(node)				\
	};									\
										\
	static struct usb_desc_header *node##_hs_desc[] = {			\
		UVC_HIGHSPEED_DESCRIPTOR_PTRS(node)				\
	};									\
										\
	static struct usb_desc_header *node##_ss_desc[] = {			\
		UVC_SUPERSPEED_DESCRIPTOR_PTRS(node)				\
	};									\
										\
	static const struct video_format_cap node##_caps[] = {			\
		DT_FOREACH_CHILD(node, UVC_CAPS)				\
		{0}								\
	};									\
										\
	static const struct uvc_control node##_controls[] = {			\
		DT_FOREACH_CHILD(node, UVC_CONTROL)				\
		{0}								\
	};									\
										\
	static const struct uvc_format node##_formats[] = {			\
		DT_FOREACH_CHILD(node, UVC_FORMATS)				\
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
		.controls = node##_controls,					\
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
		.output_terminal = DEVICE_DT_GET_OR_NULL(OUTPUT_TERMINAL(node)),\
	};									\
										\
	BUILD_ASSERT(DT_ON_BUS(node, usb),					\
		     "node " DT_NODE_PATH(node) " is not"			\
		     " assigned to a USB device controller");			\
										\
	DEVICE_DT_DEFINE(node, uvc_preinit, NULL, &node##_data, NULL,		\
			 POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,		\
			 &uvc_video_api);

DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, UVC_DEVICE_DEFINE)
