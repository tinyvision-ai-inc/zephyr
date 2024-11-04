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

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>

LOG_MODULE_REGISTER(usbd_uvc, CONFIG_USBD_UVC_LOG_LEVEL);

/* Video Class-Specific Request Codes */
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

/* 4.2.1.2 Request Error Code Control */
#define ERR_NOT_READY				0x01
#define ERR_WRONG_STATE				0x02
#define ERR_OUT_OF_RANGE			0x04
#define ERR_INVALID_UNIT			0x05
#define ERR_INVALID_CONTROL			0x06
#define ERR_INVALID_REQUEST			0x07
#define ERR_INVALID_VALUE_WITHIN_RANGE		0x08
#define ERR_UNKNOWN				0xff

/* Video and Still Image Payload Headers */
#define UVC_BMHEADERINFO_FRAMEID		BIT(0)
#define UVC_BMHEADERINFO_END_OF_FRAME		BIT(1)
#define UVC_BMHEADERINFO_HAS_PRESENTATIONTIME	BIT(2)
#define UVC_BMHEADERINFO_HAS_SOURCECLOCK	BIT(3)
#define UVC_BMHEADERINFO_PAYLOAD_SPECIFIC_BIT	BIT(4)
#define UVC_BMHEADERINFO_STILL_IMAGE		BIT(5)
#define UVC_BMHEADERINFO_ERROR			BIT(6)
#define UVC_BMHEADERINFO_END_OF_HEADER		BIT(7)

/* Video Interface Subclass Codes */
#define SC_VIDEOCONTROL				0x01
#define SC_VIDEOSTREAMING			0x02
#define SC_VIDEO_INTERFACE_COLLECTION		0x03

/* Video Class-Specific Video Control Interface Descriptor Subtypes */
#define VC_DESCRIPTOR_UNDEFINED			0x00
#define VC_HEADER				0x01
#define VC_INPUT_TERMINAL			0x02
#define VC_OUTPUT_TERMINAL			0x03
#define VC_SELECTOR_UNIT			0x04
#define VC_PROCESSING_UNIT			0x05
#define VC_EXTENSION_UNIT			0x06
#define VC_ENCODING_UNIT			0x07

/* Video Class-Specific Video Stream Interface Descriptor Subtypes */
#define VS_UNDEFINED				0x00
#define VS_INPUT_HEADER				0x01
#define VS_OUTPUT_HEADER			0x02
#define VS_STILL_IMAGE_FRAME			0x03
#define VS_FORMAT_UNCOMPRESSED			0x04
#define VS_FRAME_UNCOMPRESSED			0x05
#define VS_FORMAT_MJPEG				0x06
#define VS_FRAME_MJPEG				0x07
#define VS_FORMAT_MPEG2TS			0x0A
#define VS_FORMAT_DV				0x0C
#define VS_COLORFORMAT				0x0D
#define VS_FORMAT_FRAME_BASED			0x10
#define VS_FRAME_FRAME_BASED			0x11
#define VS_FORMAT_STREAM_BASED			0x12
#define VS_FORMAT_H264				0x13
#define VS_FRAME_H264				0x14
#define VS_FORMAT_H264_SIMULCAST		0x15
#define VS_FORMAT_VP8				0x16
#define VS_FRAME_VP8				0x17
#define VS_FORMAT_VP8_SIMULCAST			0x18

/* Video Class-Specific Endpoint Descriptor Subtypes */
#define EP_UNDEFINED				0x00
#define EP_GENERAL				0x01
#define EP_ENDPOINT				0x02
#define EP_INTERRUPT				0x03

/* USB Terminal Types */
#define TT_VENDOR_SPECIFIC			0x0100
#define TT_STREAMING				0x0101

/* Input Terminal Types */
#define ITT_VENDOR_SPECIFIC			0x0200
#define ITT_CAMERA				0x0201
#define ITT_MEDIA_TRANSPORT_INPUT		0x0202

/* Output Terminal Types */
#define OTT_VENDOR_SPECIFIC			0x0300
#define OTT_DISPLAY				0x0301
#define OTT_MEDIA_TRANSPORT_OUTPUT		0x0302

/* External Terminal Types */
#define EXT_EXTERNAL_VENDOR_SPECIFIC		0x0400
#define EXT_COMPOSITE_CONNECTOR			0x0401
#define EXT_SVIDEO_CONNECTOR			0x0402
#define EXT_COMPONENT_CONNECTOR			0x0403

/* VideoStreaming Interface Controls */
#define VS_PROBE_CONTROL			0x01
#define VS_COMMIT_CONTROL			0x02
#define VS_STILL_PROBE_CONTROL			0x03
#define VS_STILL_COMMIT_CONTROL			0x04
#define VS_STILL_IMAGE_TRIGGER_CONTROL		0x05
#define VS_STREAM_ERROR_CODE_CONTROL		0x06
#define VS_GENERATE_KEY_FRAME_CONTROL		0x07
#define VS_UPDATE_FRAME_SEGMENT_CONTROL		0x08
#define VS_SYNCH_DELAY_CONTROL			0x09

/* VideoControl Interface Controls */
#define VC_CONTROL_UNDEFINED			0x00
#define VC_VIDEO_POWER_MODE_CONTROL		0x01
#define VC_REQUEST_ERROR_CODE_CONTROL		0x02

/* Selector Unit Controls */
#define SU_INPUT_SELECT_CONTROL			0x01
#define SU_INPUT_SELECT_BIT			0
#define SU_CONTROL_MAX				0x01

/* Camera Terminal Controls */
#define CT_SCANNING_MODE_CONTROL		0x01
#define CT_SCANNING_MODE_BIT			0
#define CT_AE_MODE_CONTROL			0x02
#define CT_AE_MODE_BIT				1
#define CT_AE_PRIORITY_CONTROL			0x03
#define CT_AE_PRIORITY_BIT			2
#define CT_EXPOSURE_TIME_ABSOLUTE_CONTROL	0x04
#define CT_EXPOSURE_TIME_ABSOLUTE_BIT		3
#define CT_EXPOSURE_TIME_RELATIVE_CONTROL	0x05
#define CT_EXPOSURE_TIME_RELATIVE_BIT		4
#define CT_FOCUS_ABSOLUTE_CONTROL		0x06
#define CT_FOCUS_ABSOLUTE_BIT			5
#define CT_FOCUS_RELATIVE_CONTROL		0x07
#define CT_FOCUS_RELATIVE_BIT			6
#define CT_FOCUS_AUTO_CONTROL			0x08
#define CT_FOCUS_AUTO_BIT			17
#define CT_IRIS_ABSOLUTE_CONTROL		0x09
#define CT_IRIS_ABSOLUTE_BIT			7
#define CT_IRIS_RELATIVE_CONTROL		0x0A
#define CT_IRIS_RELATIVE_BIT			8
#define CT_ZOOM_ABSOLUTE_CONTROL		0x0B
#define CT_ZOOM_ABSOLUTE_BIT			9
#define CT_ZOOM_RELATIVE_CONTROL		0x0C
#define CT_ZOOM_RELATIVE_BIT			10
#define CT_PANTILT_ABSOLUTE_CONTROL		0x0D
#define CT_PANTILT_ABSOLUTE_BIT			11
#define CT_PANTILT_RELATIVE_CONTROL		0x0E
#define CT_PANTILT_RELATIVE_BIT			12
#define CT_ROLL_ABSOLUTE_CONTROL		0x0F
#define CT_ROLL_ABSOLUTE_BIT			13
#define CT_ROLL_RELATIVE_CONTROL		0x10
#define CT_ROLL_RELATIVE_BIT			14
#define CT_PRIVACY_CONTROL			0x11
#define CT_PRIVACY_BIT				18
#define CT_FOCUS_SIMPLE_CONTROL			0x12
#define CT_FOCUS_SIMPLE_BIT			19
#define CT_WINDOW_CONTROL			0x13
#define CT_WINDOW_BIT				20
#define CT_REGION_OF_INTEREST_CONTROL		0x14
#define CT_REGION_OF_INTEREST_BIT		21
#define CT_CONTROL_MAX				0x14

/* Processing Unit Controls */
#define PU_BACKLIGHT_COMPENSATION_CONTROL	0x01
#define PU_BACKLIGHT_COMPENSATION_BIT		8
#define PU_BRIGHTNESS_CONTROL			0x02
#define PU_BRIGHTNESS_BIT			0
#define PU_CONTRAST_CONTROL			0x03
#define PU_CONTRAST_BIT				1
#define PU_GAIN_CONTROL				0x04
#define PU_GAIN_BIT				9
#define PU_POWER_LINE_FREQUENCY_CONTROL		0x05
#define PU_POWER_LINE_FREQUENCY_BIT		10
#define PU_HUE_CONTROL				0x06
#define PU_HUE_BIT				2
#define PU_SATURATION_CONTROL			0x07
#define PU_SATURATION_BIT			3
#define PU_SHARPNESS_CONTROL			0x08
#define PU_SHARPNESS_BIT			4
#define PU_GAMMA_CONTROL			0x09
#define PU_GAMMA_BIT				5
#define PU_WHITE_BALANCE_TEMPERATURE_CONTROL	0x0A
#define PU_WHITE_BALANCE_TEMPERATURE_BIT	6
#define PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL 0x0B
#define PU_WHITE_BALANCE_TEMPERATURE_AUTO_BIT	12
#define PU_WHITE_BALANCE_COMPONENT_CONTROL	0x0C
#define PU_WHITE_BALANCE_COMPONENT_BIT		7
#define PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL	0x0D
#define PU_WHITE_BALANCE_COMPONENT_AUTO_BIT	13
#define PU_DIGITAL_MULTIPLIER_CONTROL		0x0E
#define PU_DIGITAL_MULTIPLIER_BIT		14
#define PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL	0x0F
#define PU_DIGITAL_MULTIPLIER_LIMIT_BIT		15
#define PU_HUE_AUTO_CONTROL			0x10
#define PU_HUE_AUTO_BIT				11
#define PU_ANALOG_VIDEO_STANDARD_CONTROL	0x11
#define PU_ANALOG_VIDEO_STANDARD_BIT		16
#define PU_ANALOG_LOCK_STATUS_CONTROL		0x12
#define PU_ANALOG_LOCK_STATUS_BIT		17
#define PU_CONTRAST_AUTO_CONTROL		0x13
#define PU_CONTRAST_AUTO_BIT			18
#define PU_CONTROL_MAX				0x13

/* Encoding Unit Controls */
#define EU_SELECT_LAYER_CONTROL			0x01
#define EU_SELECT_LAYER_BIT			0
#define EU_PROFILE_TOOLSET_CONTROL		0x02
#define EU_PROFILE_TOOLSET_BIT			1
#define EU_VIDEO_RESOLUTION_CONTROL		0x03
#define EU_VIDEO_RESOLUTION_BIT			2
#define EU_MIN_FRAME_INTERVAL_CONTROL		0x04
#define EU_MIN_FRAME_INTERVAL_BIT		3
#define EU_SLICE_MODE_CONTROL			0x05
#define EU_SLICE_MODE_BIT			4
#define EU_RATE_CONTROL_MODE_CONTROL		0x06
#define EU_RATE_CONTROL_MODE_BIT		5
#define EU_AVERAGE_BITRATE_CONTROL		0x07
#define EU_AVERAGE_BITRATE_BIT			6
#define EU_CPB_SIZE_CONTROL			0x08
#define EU_CPB_SIZE_BIT				7
#define EU_PEAK_BIT_RATE_CONTROL		0x09
#define EU_PEAK_BIT_RATE_BIT			8
#define EU_QUANTIZATION_PARAMS_CONTROL		0x0A
#define EU_QUANTIZATION_PARAMS_BIT		9
#define EU_SYNC_REF_FRAME_CONTROL		0x0B
#define EU_SYNC_REF_FRAME_BIT			10
#define EU_LTR_BUFFER_CONTROL			0x0C
#define EU_LTR_BUFFER_BIT			11
#define EU_LTR_PICTURE_CONTROL			0x0D
#define EU_LTR_PICTURE_BIT			12
#define EU_LTR_VALIDATION_CONTROL		0x0E
#define EU_LTR_VALIDATION_BIT			13
#define EU_LEVEL_IDC_LIMIT_CONTROL		0x0F
#define EU_LEVEL_IDC_LIMIT_BIT			14
#define EU_SEI_PAYLOADTYPE_CONTROL		0x10
#define EU_SEI_PAYLOADTYPE_BIT			15
#define EU_QP_RANGE_CONTROL			0x11
#define EU_QP_RANGE_BIT				16
#define EU_PRIORITY_CONTROL			0x12
#define EU_PRIORITY_BIT				17
#define EU_START_OR_STOP_LAYER_CONTROL		0x13
#define EU_START_OR_STOP_LAYER_BIT		18
#define EU_ERROR_RESILIENCY_CONTROL		0x14
#define EU_ERROR_RESILIENCY_BIT			19
#define EU_CONTROL_MAX				0x14

/* Extension Unit Controls */
#define XU_BASE_CONTROL				0x00
#define XU_BASE_BIT				0
#define XU_CONTROL_MAX				0x1F

enum uvc_class_status {
	CLASS_ENABLED,
	CLASS_READY,
};

struct uvc_control_header_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint16_t bcdUVC;
	uint16_t wTotalLength;
	uint32_t dwClockFrequency;
	uint8_t bInCollection;
	uint8_t baInterfaceNr[CONFIG_USBD_VIDEO_MAX_STREAMS];
} __packed;

struct uvc_output_terminal_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalID;
	uint16_t wTerminalType;
	uint8_t bAssocTerminal;
	uint8_t bSourceID;
	uint8_t iTerminal;
} __packed;

struct uvc_camera_terminal_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalID;
	uint16_t wTerminalType;
	uint8_t bAssocTerminal;
	uint8_t iTerminal;
	uint16_t wObjectiveFocalLengthMin;
	uint16_t wObjectiveFocalLengthMax;
	uint16_t wOcularFocalLength;
	uint8_t bControlSize;
	uint8_t bmControls[3];
} __packed;

struct uvc_selector_unit_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bUnitID;
	uint8_t bNrInPins;
	uint8_t baSourceID[1];
	uint8_t iSelector;
} __packed;

struct uvc_processing_unit_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bUnitID;
	uint8_t bSourceID;
	uint8_t wMaxMultiplier;
	uint8_t bControlSize;
	uint8_t bmControls[3];
	uint8_t iProcessing;
	uint8_t bmVideoStandards;
} __packed;

struct uvc_encoding_unit_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bUnitID;
	uint8_t bSourceID;
	uint8_t iEncoding;
	uint8_t bControlSize;
	uint8_t bmControls[3];
	uint8_t bmControlsRuntime[3];
} __packed;

struct uvc_extension_unit_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bUnitID;
	uint8_t guidExtensionCode[16];
	uint8_t bNumControls;
	uint8_t bNrInPins;
	uint8_t baSourceID[1];
	uint8_t bControlSize;
	uint8_t bmControls[4];
	uint8_t iExtension;
} __packed;

struct uvc_stream_header_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bNumFormats;
	uint16_t wTotalLength;
	uint8_t bEndpointAddress;
	uint8_t bmInfo;
	uint8_t bTerminalLink;
	uint8_t bStillCaptureMethod;
	uint8_t bTriggerSupport;
	uint8_t bTriggerUsage;
	uint8_t bControlSize;
} __packed;

struct uvc_frame_still_image_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bEndpointAddress;
	uint8_t bNumImageSizePatterns;
	struct {
		uint16_t wWidth;
		uint16_t wHeight;
	} n[1] __packed;
	uint8_t bNumCompressionPattern;
	uint8_t bCompression[1];
} __packed;

struct uvc_format_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFormatIndex;
	uint8_t bNumFrameDescriptors;
} __packed;

struct uvc_format_uncomp_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFormatIndex;
	uint8_t bNumFrameDescriptors;
	uint8_t guidFormat[16];
	uint8_t bBitsPerPixel;
	uint8_t bDefaultFrameIndex;
	uint8_t bAspectRatioX;
	uint8_t bAspectRatioY;
	uint8_t bmInterlaceFlags;
	uint8_t bCopyProtect;
} __packed;

struct uvc_format_mjpeg_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFormatIndex;
	uint8_t bNumFrameDescriptors;
	uint8_t bmFlags;
#define UVC_MJPEG_FLAGS_FIXEDSIZESAMPLES (1 << 0)
	uint8_t bDefaultFrameIndex;
	uint8_t bAspectRatioX;
	uint8_t bAspectRatioY;
	uint8_t bmInterlaceFlags;
	uint8_t bCopyProtect;
} __packed;

struct uvc_frame_continuous_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFrameIndex;
	uint8_t bmCapabilities;
	uint16_t wWidth;
	uint16_t wHeight;
	uint32_t dwMinBitRate;
	uint32_t dwMaxBitRate;
	uint32_t dwMaxVideoFrameBufferSize;
	uint32_t dwDefaultFrameInterval;
	uint8_t bFrameIntervalType;
	uint32_t dwMinFrameInterval;
	uint32_t dwMaxFrameInterval;
	uint32_t dwFrameIntervalStep;
} __packed;

struct uvc_frame_discrete_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFrameIndex;
	uint8_t bmCapabilities;
	uint16_t wWidth;
	uint16_t wHeight;
	uint32_t dwMinBitRate;
	uint32_t dwMaxBitRate;
	uint32_t dwMaxVideoFrameBufferSize;
	uint32_t dwDefaultFrameInterval;
	uint8_t bFrameIntervalType;
	uint32_t dwFrameInterval[CONFIG_USBD_VIDEO_MAX_FRMIVAL];
} __packed;

union uvc_stream_descriptor {
	struct uvc_format_descriptor format;
	struct uvc_format_uncomp_descriptor format_uncomp;
	struct uvc_format_mjpeg_descriptor format_mjpeg;
	struct uvc_frame_discrete_descriptor frame_discrete;
	struct uvc_frame_continuous_descriptor frame_continuous;
} __packed;

struct uvc_color_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bColorPrimaries;
	uint8_t bTransferCharacteristics;
	uint8_t bMatrixCoefficients;
} __packed;

struct usbd_uvc_desc {
	struct usb_association_descriptor iad;
	struct usb_if_descriptor if0;
	struct uvc_control_header_descriptor if0_header;
	struct usb_desc_header nil_desc;
};

struct usbd_uvc_strm_desc {
	struct uvc_camera_terminal_descriptor if0_ct;
	struct uvc_selector_unit_descriptor if0_su;
	struct uvc_processing_unit_descriptor if0_pu;
	struct uvc_extension_unit_descriptor if0_xu;
	struct uvc_output_terminal_descriptor if0_ot;

	struct usb_if_descriptor ifN;
	struct uvc_stream_header_descriptor ifN_header;
	union uvc_stream_descriptor *ifN_formats;
	size_t ifN_format_num;
	struct usb_ep_descriptor ifN_ep_fs;
	struct usb_ep_descriptor ifN_ep_hs;
	struct uvc_color_descriptor ifN_color;
};

struct uvc_probe_msg {
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

struct uvc_payload_header {
	uint8_t bHeaderLength;
	uint8_t bmHeaderInfo;
	uint32_t dwPresentationTime; /* optional */
	uint32_t scrSourceClockSTC;  /* optional */
	uint16_t scrSourceClockSOF;  /* optional */
} __packed;

/* Information specific to each VideoStreaming interface */
struct uvc_stream {
	const struct device *dev;
	atomic_t state;
	struct k_work work;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	struct usbd_uvc_strm_desc *desc;
	struct uvc_probe_msg default_probe;
	struct uvc_payload_header payload_header;
	uint8_t format_id;
	uint8_t frame_id;
	uint32_t ct_mask;
	uint32_t su_mask;
	uint32_t pu_mask;
	uint32_t xu_mask;
	const struct device *video_dev;
	struct video_format video_fmt;
	struct video_frmival video_frmival;
	size_t vbuf_offset;
};

/* Global information */
struct uvc_data {
	struct usbd_class_data *c_data;
	struct uvc_stream *streams;
	struct usbd_uvc_desc *desc;
	struct usb_desc_header **fs_desc;
	struct usb_desc_header **hs_desc;
	/* UVC error from latest request */
	int err;
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

typedef int uvc_control_fn_t(struct uvc_stream *strm, uint8_t selector, uint8_t request,
			     struct net_buf *buf);

NET_BUF_POOL_VAR_DEFINE(uvc_pool, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 16,
			512 * DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 16,
			sizeof(struct uvc_buf_info), NULL);

const uint8_t uvc_base_guid[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
				   0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71};

int uvc_get_stream(const struct device *dev, enum video_endpoint_id ep, struct uvc_stream **result)
{
	const struct uvc_data *data = dev->data;

	if (ep == VIDEO_EP_OUT) {
		return -EINVAL;
	}

	if (ep == VIDEO_EP_IN || ep == VIDEO_EP_ALL) {
		*result = &data->streams[0];
		return 0;
	}

	/* Iterate instead of dereference to prevent overflow */
	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++, ep--) {
		if (ep == 0) {
			*result = strm;
			return 0;
		}
	}
	return -ENODEV;
}

static void uvc_get_format_frame_desc(const struct uvc_stream *strm,
				      union uvc_stream_descriptor **format_desc,
				      union uvc_stream_descriptor **frame_desc)
{
	int i;

	*format_desc = NULL;
	for (i = 0; i < strm->desc->ifN_format_num; i++) {
		union uvc_stream_descriptor *desc = &strm->desc->ifN_formats[i];

		if (desc->format.bDescriptorSubtype != VS_FORMAT_UNCOMPRESSED &&
		    desc->format.bDescriptorSubtype != VS_FORMAT_MJPEG) {
			continue;
		}

		if (desc->format.bFormatIndex == strm->format_id) {
			*format_desc = &strm->desc->ifN_formats[i];
			break;
		}
	}

	*frame_desc = NULL;
	for (i += 1; i < strm->desc->ifN_format_num; i++) {
		union uvc_stream_descriptor *desc = &strm->desc->ifN_formats[i];

		if (desc->frame_discrete.bDescriptorSubtype != VS_FRAME_UNCOMPRESSED &&
		    desc->frame_discrete.bDescriptorSubtype != VS_FRAME_MJPEG) {
			break;
		}
		if (desc->frame_discrete.bFrameIndex == strm->frame_id) {
			*frame_desc = &strm->desc->ifN_formats[i];
			break;
		}
	}
}

static uint32_t uvc_get_video_cid(uint8_t request, uint32_t cid)
{
	switch (request) {
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

static uint8_t uvc_get_bulk_in(struct uvc_stream *strm)
{
	struct uvc_data *data = strm->dev->data;

	switch (usbd_bus_speed(usbd_class_get_ctx(data->c_data))) {
	case USBD_SPEED_FS:
		return strm->desc->ifN_ep_fs.bEndpointAddress;
	case USBD_SPEED_HS:
		return strm->desc->ifN_ep_hs.bEndpointAddress;
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

static int uvc_control_probe_format_index(struct uvc_stream *strm, uint8_t request,
					  struct uvc_probe_msg *probe)
{
	uint8_t max = 0;

	for (int i = 0; i < strm->desc->ifN_format_num; i++) {
		union uvc_stream_descriptor *desc = &strm->desc->ifN_formats[i];

		max += desc->format.bDescriptorSubtype == VS_FORMAT_UNCOMPRESSED ||
		       desc->format.bDescriptorSubtype == VS_FORMAT_MJPEG;
	}

	switch (request) {
	case GET_RES:
	case GET_MIN:
		probe->bFormatIndex = 1;
		break;
	case GET_MAX:
		probe->bFormatIndex = max;
		break;
	case GET_CUR:
		probe->bFormatIndex = strm->format_id;
		break;
	case SET_CUR:
		if (probe->bFormatIndex == 0) {
			return 0;
		}
		if (probe->bFormatIndex > max) {
			LOG_WRN("probe: format index %u not found", probe->bFormatIndex);
			return -EINVAL;
		}
		strm->format_id = probe->bFormatIndex;
		break;
	}
	return 0;
}

static int uvc_control_probe_frame_index(struct uvc_stream *strm, uint8_t request,
					 struct uvc_probe_msg *probe)
{
	uint8_t max = 0;
	int i;

	/* Search the current format */
	for (i = 0; i < strm->desc->ifN_format_num; i++) {
		union uvc_stream_descriptor *desc = &strm->desc->ifN_formats[i];

		if (desc->format.bDescriptorSubtype != VS_FORMAT_UNCOMPRESSED &&
		    desc->format.bDescriptorSubtype != VS_FORMAT_MJPEG) {
			continue;
		}
		if (desc->format.bFormatIndex == strm->format_id) {
			break;
		}
	}

	/* Seek until the next format */
	for (i += 1; i < strm->desc->ifN_format_num; i++) {
		union uvc_stream_descriptor *desc = &strm->desc->ifN_formats[i];

		if (desc->frame_discrete.bDescriptorSubtype != VS_FRAME_UNCOMPRESSED &&
		    desc->frame_discrete.bDescriptorSubtype != VS_FRAME_MJPEG) {
			break;
		}
		max++;
	}

	switch (request) {
	case GET_RES:
	case GET_MIN:
		probe->bFrameIndex = 1;
		break;
	case GET_MAX:
		probe->bFrameIndex = max;
		break;
	case GET_CUR:
		probe->bFrameIndex = strm->frame_id;
		break;
	case SET_CUR:
		if (probe->bFrameIndex == 0) {
			return 0;
		}
		if (probe->bFrameIndex > max) {
			LOG_WRN("probe: frame index %u not found", probe->bFrameIndex);
			return -EINVAL;
		}
		strm->format_id = probe->bFrameIndex;
		break;
	}
	return 0;
}

static int uvc_control_probe_frame_interval(struct uvc_stream *strm, uint8_t request,
					    struct uvc_probe_msg *probe)
{
	union uvc_stream_descriptor *format_desc;
	union uvc_stream_descriptor *frame_desc;
	int max;

	uvc_get_format_frame_desc(strm, &format_desc, &frame_desc);
	if (format_desc == NULL || frame_desc == NULL) {
		LOG_DBG("Selected format ID or frame ID not found");
		return -EINVAL;
	}

	switch (request) {
	case GET_MIN:
		probe->dwFrameInterval =
			sys_cpu_to_le32(frame_desc->frame_discrete.dwFrameInterval[0]);
		break;
	case GET_MAX:
		max = frame_desc->frame_discrete.bFrameIntervalType - 1;
		probe->dwFrameInterval =
			sys_cpu_to_le32(frame_desc->frame_discrete.dwFrameInterval[max]);
		break;
	case GET_RES:
		probe->dwFrameInterval = sys_cpu_to_le32(1);
		break;
	case GET_CUR:
		probe->dwFrameInterval = sys_cpu_to_le32(strm->video_frmival.numerator);
		break;
	case SET_CUR:
		strm->video_frmival.numerator = sys_le32_to_cpu(probe->dwFrameInterval);
		break;
	}
	return 0;
}

static int uvc_control_probe_max_video_frame_size(struct uvc_stream *strm, uint8_t request,
						  struct uvc_probe_msg *probe)
{
	uint32_t max_frame_size = strm->video_fmt.pitch * strm->video_fmt.height;

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

static int uvc_control_probe_max_payload_size(struct uvc_stream *strm, uint8_t request,
					      struct uvc_probe_msg *probe)
{
	uint32_t max_payload_size = strm->video_fmt.pitch * strm->video_fmt.height;

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

static int uvc_control_probe(struct uvc_stream *strm, uint8_t request, struct uvc_probe_msg *probe)
{
	union uvc_stream_descriptor *format_desc = NULL;
	union uvc_stream_descriptor *frame_desc = NULL;
	struct video_format *fmt = &strm->video_fmt;
	int ret;

	if (request != GET_MIN && request != GET_MAX && request != GET_RES && request != GET_CUR &&
	    request != SET_CUR) {
		LOG_WRN("control: invalid bRequest %u", request);
		return -EINVAL;
	}

	/* Video format fields */

	ret = uvc_control_probe_format_index(strm, request, probe);
	if (ret < 0) {
		return ret;
	}

	ret = uvc_control_probe_frame_index(strm, request, probe);
	if (ret < 0) {
		return ret;
	}

	/* Update the format after the what the probe configures */
	uvc_get_format_frame_desc(strm, &format_desc, &frame_desc);
	if (format_desc == NULL || frame_desc == NULL) {
		LOG_ERR("Called probe with invalid format ID (%u) and frame ID (%u)",
			strm->format_id, strm->frame_id);
		return -EINVAL;
	}
	if (format_desc->format.bDescriptorSubtype == VS_FORMAT_MJPEG) {
		fmt->pixelformat = VIDEO_PIX_FMT_JPEG;
	} else {
		fmt->pixelformat = video_fourcc(format_desc->format_uncomp.guidFormat[0],
						format_desc->format_uncomp.guidFormat[1],
						format_desc->format_uncomp.guidFormat[2],
						format_desc->format_uncomp.guidFormat[3]);
	}
	fmt->width = frame_desc->frame_discrete.wWidth;
	fmt->height = frame_desc->frame_discrete.wHeight;
	fmt->pitch = fmt->width * video_pix_fmt_bpp(fmt->pixelformat);

	ret = uvc_control_probe_frame_interval(strm, request, probe);
	if (ret < 0) {
		return ret;
	}

	ret = uvc_control_probe_max_video_frame_size(strm, request, probe);
	if (ret < 0) {
		return ret;
	}

	ret = uvc_control_probe_max_payload_size(strm, request, probe);
	if (ret < 0) {
		return ret;
	}

	/* Static and unimplemnted fields */
	probe->dwClockFrequency = sys_cpu_to_le32(1);
	probe->bmFramingInfo = UVC_BMFRAMING_INFO_FID | UVC_BMFRAMING_INFO_EOF;
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

static int uvc_control_commit(struct uvc_stream *strm, uint8_t request, struct uvc_probe_msg *probe)
{
	struct video_format *fmt = &strm->video_fmt;
	int ret;

	switch (request) {
	case GET_CUR:
		return uvc_control_probe(strm, request, probe);
	case SET_CUR:
		ret = uvc_control_probe(strm, request, probe);
		if (ret < 0) {
			return ret;
		}

		atomic_set_bit(&strm->state, CLASS_READY);
		k_work_submit(&strm->work);

		LOG_INF("control: ready to transfer, setting source format to %ux%u, %u bytes",
			fmt->width, fmt->height, fmt->height * fmt->pitch);

		ret = video_set_format(strm->video_dev, VIDEO_EP_OUT, fmt);
		if (ret < 0) {
			LOG_ERR("Could not set the format of the video source");
			return ret;
		}

		ret = video_set_frmival(strm->video_dev, VIDEO_EP_OUT, &strm->video_frmival);
		if (ret < 0) {
			LOG_ERR("Could not set the format of the video source");
			return ret;
		}

		ret = video_stream_start(strm->video_dev);
		if (ret < 0) {
			LOG_ERR("Could not start the video source");
			return ret;
		}

		break;
	default:
		LOG_WRN("commit: invalid bRequest %u", request);
		return -EINVAL;
	}
	return 0;
}

static int uvc_control_vs(struct uvc_stream *strm, const struct usb_setup_packet *setup,
			  struct net_buf *buf)
{
	uint8_t selector = setup->wValue >> 8;
	struct uvc_probe_msg *probe = (void *)buf->data;

	switch (setup->bRequest) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET | INFO_SUPPORTS_SET);
	case GET_LEN:
		return uvc_buf_add(buf, 2, sizeof(probe));
	default:
		if (buf->size != sizeof(*probe)) {
			LOG_ERR("control: bad wLength %u, expected %u", buf->size, sizeof(*probe));
			return -EINVAL;
		}
	}

	switch (setup->bRequest) {
	case GET_DEF:
		net_buf_add_mem(buf, &strm->default_probe, sizeof(*probe));
		return 0;
	case GET_MIN:
	case GET_MAX:
	case GET_RES:
	case GET_CUR:
		net_buf_add(buf, sizeof(*probe));
		break;
	}

	switch (selector) {
	case VS_PROBE_CONTROL:
		LOG_DBG("VS_PROBE_CONTROL");
		return uvc_control_probe(strm, setup->bRequest, probe);
	case VS_COMMIT_CONTROL:
		LOG_DBG("VS_COMMIT_CONTROL");
		return uvc_control_commit(strm, setup->bRequest, probe);
	default:
		LOG_WRN("control: unknown selector %u for streaming interface", selector);
		return -ENOTSUP;
	}
}

static int uvc_control_default(uint8_t request, struct net_buf *buf, uint8_t size)
{
	switch (request) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET | INFO_SUPPORTS_SET);
	case GET_LEN:
		return uvc_buf_add(buf, buf->size, size);
	case GET_RES:
		return uvc_buf_add(buf, size, 1);
	default:
		LOG_WRN("control: unsupported request type %u", request);
		return -ENOTSUP;
	}
}

static int uvc_control_fix(uint8_t request, struct net_buf *buf, uint8_t size, uint32_t value)
{
	LOG_DBG("control: type 'fixed', size %u", size);

	switch (request) {
	case GET_DEF:
	case GET_CUR:
	case GET_MIN:
	case GET_MAX:
		return uvc_buf_add(buf, size, value);
	case SET_CUR:
		return 0;
	default:
		return uvc_control_default(request, buf, size);
	}
}

static int uvc_control_int(uint8_t request, struct net_buf *buf, uint8_t size,
			   const struct device *targ_dev, unsigned int cid)
{
	int value;
	int ret;

	LOG_DBG("control: type 'integer', size %u", size);

	switch (request) {
	case GET_DEF:
	case GET_CUR:
	case GET_MIN:
	case GET_MAX:
		ret = video_get_ctrl(targ_dev, uvc_get_video_cid(request, cid), &value);
		if (ret < 0) {
			LOG_WRN("control: failed to query '%s'", targ_dev->name);
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
		ret = video_set_ctrl(targ_dev, cid, (void *)value);
		if (ret < 0) {
			LOG_ERR("control: failed to configure target video device");
			return ret;
		}
		return 0;
	default:
		return uvc_control_default(request, buf, size);
	}
}

static int uvc_control_su(struct uvc_stream *strm, uint8_t selector, uint8_t request,
			  struct net_buf *buf)
{
	LOG_DBG("SU_INPUT_SELECT_CONTROL -> VIDEO_CID_CAMERA_TEST_PATTERN");
	return uvc_control_int(request, buf, 1, strm->video_dev, VIDEO_CID_CAMERA_TEST_PATTERN);
};

static int uvc_control_ct(struct uvc_stream *strm, uint8_t selector, uint8_t request,
			  struct net_buf *buf)
{
	switch (selector) {
	case CT_AE_MODE_CONTROL:
		LOG_DBG("CT_AE_MODE_CONTROL -> (none)");
		return uvc_control_fix(request, buf, 1, BIT(0));
	case CT_AE_PRIORITY_CONTROL:
		LOG_DBG("CT_AE_PRIORITY_CONTROL -> (none)");
		return uvc_control_fix(request, buf, 1, 0);
	case CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
		LOG_DBG("CT_EXPOSURE_TIME_ABSOLUTE_CONTROL -> VIDEO_CID_CAMERA_EXPOSURE");
		return uvc_control_int(request, buf, 4, strm->video_dev, VIDEO_CID_CAMERA_EXPOSURE);
	case CT_ZOOM_ABSOLUTE_CONTROL:
		LOG_DBG("CT_ZOOM_ABSOLUTE_CONTROL -> VIDEO_CID_CAMERA_ZOOM");
		return uvc_control_int(request, buf, 2, strm->video_dev, VIDEO_CID_CAMERA_ZOOM);
	default:
		LOG_INF("control: unsupported selector 0x%02x for camera terminal", selector);
		return -ENOTSUP;
	}
}

static int uvc_control_pu(struct uvc_stream *strm, uint8_t selector, uint8_t request,
			  struct net_buf *buf)
{
	const struct device *dev = strm->video_dev;

	switch (selector) {
	case PU_BRIGHTNESS_CONTROL:
		LOG_DBG("PU_BRIGHTNESS_CONTROL -> VIDEO_CID_CAMERA_BRIGHTNESS");
		return uvc_control_int(request, buf, 2, dev, VIDEO_CID_CAMERA_BRIGHTNESS);
	case PU_CONTRAST_CONTROL:
		LOG_DBG("PU_CONTRAST_CONTROL -> VIDEO_CID_CAMERA_CONTRAST");
		return uvc_control_int(request, buf, 1, dev, VIDEO_CID_CAMERA_CONTRAST);
	case PU_GAIN_CONTROL:
		LOG_DBG("PU_GAIN_CONTROL -> VIDEO_CID_CAMERA_GAIN");
		return uvc_control_int(request, buf, 2, dev, VIDEO_CID_CAMERA_GAIN);
	case PU_SATURATION_CONTROL:
		LOG_DBG("PU_SATURATION_CONTROL -> VIDEO_CID_CAMERA_SATURATION");
		return uvc_control_int(request, buf, 2, dev, VIDEO_CID_CAMERA_SATURATION);
	case PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
		LOG_DBG("PU_WHITE_BALANCE_TEMPERATURE_CONTROL -> VIDEO_CID_CAMERA_WHITE_BAL");
		return uvc_control_int(request, buf, 2, dev, VIDEO_CID_CAMERA_WHITE_BAL);
	default:
		LOG_INF("control: unsupported selector 0x%02x for processing unit", selector);
		return -ENOTSUP;
	}
}

static int uvc_control_xu(struct uvc_stream *strm, uint8_t selector, uint8_t request,
			  struct net_buf *buf)
{
	LOG_DBG("XU_BASE_CONTROL -> VIDEO_CTRL_CLASS_VENDOR");
	return uvc_control_int(request, buf, 4, strm->video_dev, VIDEO_CTRL_CLASS_VENDOR + request);
};

static int uvc_control_errno(uint8_t request, struct net_buf *buf, int err)
{
	switch (request) {
	case GET_INFO:
		return uvc_buf_add(buf, 1, INFO_SUPPORTS_GET);
	case GET_CUR:
		return uvc_buf_add(buf, 1, err);
	default:
		LOG_WRN("control: unsupported request type %u", request);
		return -ENOTSUP;
	}
}

static int uvc_control_run(struct uvc_stream *strm, uvc_control_fn_t *fn, uint32_t mask,
			   const struct usb_setup_packet *const setup, struct net_buf *buf)
{
	struct uvc_data *data = strm->dev->data;
	uint8_t selector = setup->wValue >> 8;
	uint8_t request = setup->bRequest;
	int ret;

	if ((mask & 1 << selector) == 0) {
		LOG_ERR("control: no dev for Camera Terminal control %u", selector);
		return -ENODEV;
	}

	ret = (*fn)(strm, selector, request, buf);
	switch (ret) {
	case 0:
		data->err = 0;
		break;
	case -EBUSY:
	case -EAGAIN:
	case -EINPROGRESS:
	case -EALREADY:
		data->err = ERR_NOT_READY;
		break;
	case -EOVERFLOW:
	case -ERANGE:
	case -E2BIG:
		data->err = ERR_OUT_OF_RANGE;
		break;
	case -EDOM:
	case -EINVAL:
		data->err = ERR_INVALID_VALUE_WITHIN_RANGE;
		break;
	case -ENODEV:
	case -ENOTSUP:
	case -ENOSYS:
		data->err = ERR_INVALID_REQUEST;
		break;
	default:
		data->err = ERR_UNKNOWN;
		break;
	}
	return ret;
}

static int uvc_control(const struct device *dev, const struct usb_setup_packet *const setup,
		       struct net_buf *buf)
{
	struct uvc_data *data = dev->data;
	uint8_t ifnum = (setup->wIndex >> 0) & 0xff;
	uint8_t unit_id = (setup->wIndex >> 8) & 0xff;
	uint8_t request = setup->bRequest;

	if (setup->wLength > buf->size) {
		LOG_ERR("control: wLength %u larger than %u bytes", setup->wLength, buf->size);
		return -ENOMEM;
	}
	buf->size = setup->wLength;

	/* VideoStreaming requests */

	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++) {
		if (ifnum == strm->desc->ifN.bInterfaceNumber) {
			return uvc_control_vs(strm, setup, buf);
		}
	}

	/* VideoControl requests */

	LOG_DBG("Host sent a %s VideoControl request",
		request == SET_CUR ? "SET_CUR" : request == GET_MIN ? "GET_MIN" :
		request == GET_MAX ? "GET_MAX" : request == GET_RES ? "GET_RES" :
		request == GET_LEN ? "GET_LEN" : request == GET_DEF ? "GET_DEF" :
		request == GET_INFO ? "GET_INFO" : "bad");

	if (ifnum != data->desc->if0.bInterfaceNumber) {
		LOG_WRN("control: interface %u not found", ifnum);
		data->err = ERR_INVALID_UNIT;
		return -ENOTSUP;
	}

	if (unit_id == 0) {
		return uvc_control_errno(request, buf, data->err);
	}

	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++) {
		if (unit_id == strm->desc->if0_ct.bTerminalID) {
			return uvc_control_run(strm, uvc_control_ct, strm->ct_mask, setup, buf);
		} else if (unit_id == strm->desc->if0_su.bUnitID) {
			return uvc_control_run(strm, uvc_control_su, strm->su_mask, setup, buf);
		} else if (unit_id == strm->desc->if0_pu.bUnitID) {
			return uvc_control_run(strm, uvc_control_pu, strm->pu_mask, setup, buf);
		} else if (unit_id == strm->desc->if0_xu.bUnitID) {
			return uvc_control_run(strm, uvc_control_xu, strm->xu_mask, setup, buf);
		}
	}

	LOG_WRN("control: could not handle command for bUnitID %u", unit_id);

	data->err = ERR_INVALID_UNIT;
	return -ENOTSUP;
}

static int uvc_control_to_host(struct usbd_class_data *const c_data,
			       const struct usb_setup_packet *const setup,
			       struct net_buf *const buf)
{
	errno = -uvc_control(usbd_class_get_private(c_data), setup, buf);
	return 0;
}

static int uvc_control_to_dev(struct usbd_class_data *const c_data,
			      const struct usb_setup_packet *const setup,
			      const struct net_buf *const buf)
{
	errno = -uvc_control(usbd_class_get_private(c_data), setup, (struct net_buf *)buf);
	return 0;
}

static int uvc_request(struct usbd_class_data *const c_data, struct net_buf *buf, int err)
{
	struct uvc_buf_info bi = *(struct uvc_buf_info *)udc_get_buf_info(buf);
	struct uvc_stream *strm = bi.stream;

	net_buf_unref(buf);

	if (bi.udc.ep == uvc_get_bulk_in(bi.stream)) {
		LOG_DBG("Request completed for ubuf %p, vbuf %p", buf, bi.vbuf);
		if (bi.vbuf != NULL) {
			k_fifo_put(&strm->fifo_out, bi.vbuf);
		}

		/* There is now one more net_buff buffer available */
		k_work_submit(&strm->work);
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
	struct uvc_data *data = dev->data;
	int ret;

	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++) {
		/* Now that descriptors are ready, store the default probe */
		ret = uvc_control_probe(strm, GET_CUR, &strm->default_probe);
		if (ret < 0) {
			LOG_ERR("init: failed to query the default probe");
			return ret;
		}
	}

	return 0;
}

static void uvc_update_desc(const struct device *dev)
{
	struct uvc_data *data = dev->data;

	data->desc->iad.bFirstInterface = data->desc->if0.bInterfaceNumber;

	for (int i = 0; data->streams[i].dev != NULL; i++) {
		struct uvc_stream *strm = &data->streams[i];

		strm->desc->ifN_header.bEndpointAddress = uvc_get_bulk_in(strm);
		data->desc->if0_header.baInterfaceNr[i] = strm->desc->ifN.bInterfaceNumber;
	}
}

static void *uvc_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	uvc_update_desc(dev);

	switch (speed) {
	case USBD_SPEED_FS:
		return (void *)data->fs_desc;
	case USBD_SPEED_HS:
		return (void *)data->hs_desc;
	default:
		__ASSERT_NO_MSG(false);
		return NULL;
	}
}

/* The queue of video frame fragments (vbuf) is processed, each fragment (data)
 * is prepended by the UVC header (h). The result is cut into USB packets (pkt)
 * submitted to the USB:
 *
 * [h+data] [data::]...[data::] [data] ... [h+data] [data::]...[data::] [data] ...
 * [pkt:::] [pkt:::]...[pkt:::] [pkt:] ... [pkt:::] [pkt:::]...[pkt:::] [pkt:] ...
 * [vbuf:::::::::::::::::::::::::::::] ... [vbuf:::::::::::::::::::::::::::::] ...
 * [frame::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::] ...
 * +-------------------------------------------------------------------------> time
 *
 * This function uvc_queue_vbuf() is called once per usb packet (pkt).
 *
 * @retval 0 if vbuf was partially transferred.
 * @retval 1 if vbuf was fully transferred and can be released.
 * @return Negative error code on failure.
 */
static int uvc_queue_vbuf(struct uvc_stream *strm, struct video_buffer *vbuf)
{
	struct uvc_data *data = strm->dev->data;
	struct video_format *fmt = &strm->video_fmt;
	struct net_buf *buf;
	struct uvc_buf_info *bi;
	size_t mps = uvc_get_bulk_mps(data->c_data);
	size_t next_vbuf_offset;
	int ret;

	/* Start-of-Transfer condition */
	if (strm->vbuf_offset == 0) {
		buf = net_buf_alloc_len(&uvc_pool, mps, K_NO_WAIT);
		if (buf == NULL) {
			LOG_INF("queue: Cannot allocate first USB buffer for now");
			return -ENOMEM;
		}

		LOG_INF("New USB transfer, bmHeaderInfo 0x%02x, buffer size %u",
			strm->payload_header.bmHeaderInfo, buf->size);

		/* Only the 2 first 8-bit fields supported for now, the rest is padded with 0x00 */
		net_buf_add_mem(buf, &strm->payload_header, sizeof(strm->payload_header));

		/* Pad the header up to the configured size */
		net_buf_add(buf, CONFIG_USBD_VIDEO_HEADER_SIZE - buf->len);

		/* Copy the bytes, needed for the first packet only */
		next_vbuf_offset = MIN(vbuf->bytesused, net_buf_tailroom(buf));
		net_buf_add_mem(buf, vbuf->buffer, next_vbuf_offset);

		/* End-of-Frame condition */
		if (fmt->pitch * vbuf->line_offset + vbuf->bytesused >= fmt->pitch * fmt->height) {
			/* mark the current buffer as EoF */
			((struct uvc_payload_header *)buf->data)->bmHeaderInfo |=
				UVC_BMHEADERINFO_END_OF_FRAME;

			/* Toggle the Frame ID of the next vbuf */
			strm->payload_header.bmHeaderInfo ^= UVC_BMHEADERINFO_FRAMEID;
		}
	} else {
		buf = net_buf_alloc_with_data(&uvc_pool, vbuf->buffer + strm->vbuf_offset,
					      MIN(vbuf->bytesused - strm->vbuf_offset, mps),
					      K_NO_WAIT);
		if (buf == NULL) {
			LOG_INF("queue: Cannot allocate continuation USB buffer for now");
			return -ENOMEM;
		}
		next_vbuf_offset = strm->vbuf_offset + mps;
	}

	bi = (struct uvc_buf_info *)udc_get_buf_info(buf);
	bi->udc.ep = uvc_get_bulk_in(strm);
	bi->stream = strm;

	/* End-of-Transfer condition */
	if (next_vbuf_offset >= vbuf->bytesused) {
		bi->vbuf = vbuf;
		bi->udc.zlp = (buf->len == mps);
	}

	LOG_DBG("queue: vbuf %p, offset %u/%u, size %u",
		vbuf, strm->vbuf_offset, vbuf->bytesused, buf->len);

	ret = usbd_ep_enqueue(data->c_data, buf);
	if (ret < 0) {
		net_buf_unref(buf);
		return ret;
	}

	/* End-of-Transfer condition */
	if (next_vbuf_offset >= vbuf->bytesused) {
		strm->vbuf_offset = 0;
		return 1;
	}

	strm->vbuf_offset = next_vbuf_offset;
	return 0;
}

static void uvc_worker(struct k_work *work)
{
	struct uvc_stream *strm = CONTAINER_OF(work, struct uvc_stream, work);
	struct video_buffer *vbuf;
	int ret;

	if (!atomic_test_bit(&strm->state, CLASS_ENABLED) ||
	    !atomic_test_bit(&strm->state, CLASS_READY)) {
		LOG_DBG("queue: not ready");
		return;
	}

	while ((vbuf = k_fifo_peek_head(&strm->fifo_in)) != NULL) {
		ret = uvc_queue_vbuf(strm, vbuf);
		if (ret < 0) {
			LOG_INF("queue: could not transfer %p for now", vbuf);
			break;
		}
		if (ret == 1) {
			LOG_DBG("queue: vbuf %p transferred, removing from the queue", vbuf);
			k_fifo_get(&strm->fifo_in, K_NO_WAIT);
		}
	}
}

static void uvc_enable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++) {
		/* Catch-up with buffers that might have been delayed */
		atomic_set_bit(&strm->state, CLASS_ENABLED);
		k_work_submit(&strm->work);
	}
}

static void uvc_disable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++) {
		atomic_clear_bit(&strm->state, CLASS_ENABLED);
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
	struct uvc_stream *strm;
	int ret;

	ret = uvc_get_stream(dev, ep, &strm);
	if (ret < 0) {
		return ret;
	}

	if (!atomic_test_bit(&strm->state, CLASS_ENABLED) ||
	    !atomic_test_bit(&strm->state, CLASS_READY)) {
		return -EAGAIN;
	}

	return video_get_format(strm->video_dev, VIDEO_EP_OUT, fmt);
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

	return video_get_caps(strm->video_dev, VIDEO_EP_OUT, caps);
}

struct video_driver_api uvc_video_api = {
	.get_format = uvc_get_format,
	.stream_start = uvc_stream_start,
	.stream_stop = uvc_stream_stop,
	.get_caps = uvc_get_caps,
	.enqueue = uvc_enqueue,
	.dequeue = uvc_dequeue,
};

static void uvc_preinit_control_bits(struct uvc_stream *strm)
{
	uint8_t mask = 0;

	/* For Camera Terminal, Processing Unit and Encoding Unit, the control selector values are
	 * not matching the descriptor bitmask. This requires mapping each bit of the bitmask.
	 */

	mask = 0;
	mask |= !!(strm->ct_mask & BIT(CT_SCANNING_MODE_CONTROL)) << 0;
	mask |= !!(strm->ct_mask & BIT(CT_AE_MODE_CONTROL)) << 1;
	mask |= !!(strm->ct_mask & BIT(CT_AE_PRIORITY_CONTROL)) << 2;
	mask |= !!(strm->ct_mask & BIT(CT_EXPOSURE_TIME_ABSOLUTE_CONTROL)) << 3;
	mask |= !!(strm->ct_mask & BIT(CT_EXPOSURE_TIME_RELATIVE_CONTROL)) << 4;
	mask |= !!(strm->ct_mask & BIT(CT_FOCUS_ABSOLUTE_CONTROL)) << 5;
	mask |= !!(strm->ct_mask & BIT(CT_FOCUS_RELATIVE_CONTROL)) << 6;
	mask |= !!(strm->ct_mask & BIT(CT_FOCUS_AUTO_CONTROL)) << 17;
	mask |= !!(strm->ct_mask & BIT(CT_IRIS_ABSOLUTE_CONTROL)) << 7;
	mask |= !!(strm->ct_mask & BIT(CT_IRIS_RELATIVE_CONTROL)) << 8;
	mask |= !!(strm->ct_mask & BIT(CT_ZOOM_ABSOLUTE_CONTROL)) << 9;
	mask |= !!(strm->ct_mask & BIT(CT_ZOOM_RELATIVE_CONTROL)) << 10;
	mask |= !!(strm->ct_mask & BIT(CT_PANTILT_ABSOLUTE_CONTROL)) << 11;
	mask |= !!(strm->ct_mask & BIT(CT_PANTILT_RELATIVE_CONTROL)) << 12;
	mask |= !!(strm->ct_mask & BIT(CT_ROLL_ABSOLUTE_CONTROL)) << 13;
	mask |= !!(strm->ct_mask & BIT(CT_ROLL_RELATIVE_CONTROL)) << 14;
	mask |= !!(strm->ct_mask & BIT(CT_PRIVACY_CONTROL)) << 18;
	mask |= !!(strm->ct_mask & BIT(CT_FOCUS_SIMPLE_CONTROL)) << 19;
	mask |= !!(strm->ct_mask & BIT(CT_WINDOW_CONTROL)) << 20;
	mask |= !!(strm->ct_mask & BIT(CT_REGION_OF_INTEREST_CONTROL)) << 21;
	strm->desc->if0_ct.bmControls[0] = (mask >> 0) & 0xff;
	strm->desc->if0_ct.bmControls[1] = (mask >> 8) & 0xff;
	strm->desc->if0_ct.bmControls[2] = (mask >> 16) & 0xff;

	mask = 0;
	mask |= !!(strm->pu_mask & BIT(PU_BACKLIGHT_COMPENSATION_CONTROL)) << 8;
	mask |= !!(strm->pu_mask & BIT(PU_BRIGHTNESS_CONTROL)) << 0;
	mask |= !!(strm->pu_mask & BIT(PU_CONTRAST_CONTROL)) << 1;
	mask |= !!(strm->pu_mask & BIT(PU_GAIN_CONTROL)) << 9;
	mask |= !!(strm->pu_mask & BIT(PU_POWER_LINE_FREQUENCY_CONTROL)) << 10;
	mask |= !!(strm->pu_mask & BIT(PU_HUE_CONTROL)) << 2;
	mask |= !!(strm->pu_mask & BIT(PU_SATURATION_CONTROL)) << 3;
	mask |= !!(strm->pu_mask & BIT(PU_SHARPNESS_CONTROL)) << 4;
	mask |= !!(strm->pu_mask & BIT(PU_GAMMA_CONTROL)) << 5;
	mask |= !!(strm->pu_mask & BIT(PU_WHITE_BALANCE_TEMPERATURE_CONTROL)) << 6;
	mask |= !!(strm->pu_mask & BIT(PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL)) << 12;
	mask |= !!(strm->pu_mask & BIT(PU_WHITE_BALANCE_COMPONENT_CONTROL)) << 7;
	mask |= !!(strm->pu_mask & BIT(PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL)) << 13;
	mask |= !!(strm->pu_mask & BIT(PU_DIGITAL_MULTIPLIER_CONTROL)) << 14;
	mask |= !!(strm->pu_mask & BIT(PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL)) << 15;
	mask |= !!(strm->pu_mask & BIT(PU_HUE_AUTO_CONTROL)) << 11;
	mask |= !!(strm->pu_mask & BIT(PU_ANALOG_VIDEO_STANDARD_CONTROL)) << 16;
	mask |= !!(strm->pu_mask & BIT(PU_ANALOG_LOCK_STATUS_CONTROL)) << 17;
	mask |= !!(strm->pu_mask & BIT(PU_CONTRAST_AUTO_CONTROL)) << 18;
	strm->desc->if0_pu.bmControls[0] = (mask >> 0) & 0xff;
	strm->desc->if0_pu.bmControls[1] = (mask >> 8) & 0xff;
	strm->desc->if0_pu.bmControls[2] = (mask >> 16) & 0xff;

	/* For Extension Unit, the standard does not tell the corresponding between the control
	 * selector and the descriptor control bitmask, but we assume that user will want them
	 * in incrementing order and not at random.
	 */

	mask = strm->xu_mask;
	strm->desc->if0_xu.bmControls[0] = (mask >> 0) & 0xff;
	strm->desc->if0_xu.bmControls[1] = (mask >> 8) & 0xff;
	strm->desc->if0_xu.bmControls[2] = (mask >> 16) & 0xff;
}

static void uvc_preinit_control(struct uvc_stream *strm, uvc_control_fn_t *fn, uint32_t *mask,
				uint8_t max)
{
	struct net_buf *buf;
	uint8_t data[sizeof(uint64_t)];
	uint16_t len;
	int ret;

	buf = net_buf_alloc_with_data(&uvc_pool, data, sizeof(data), K_NO_WAIT);

	LOG_DBG("init: Asking '%s' what controls supports", strm->video_dev->name);

	for (uint8_t i = 1; i <= max; i++) {
		buf->size = sizeof(len);
		ret = (*fn)(strm, i, GET_LEN, buf);
		if (ret < 0) {
			continue;
		}

		len = net_buf_remove_le16(buf);
		__ASSERT_NO_MSG(len <= sizeof(data));

		buf->size = len;
		ret = (*fn)(strm, i, GET_DEF, buf);
		*mask |= (ret >= 0) << i;
	}

	net_buf_unref(buf);
}

static int uvc_add_format_desc(struct uvc_stream *strm, int *id,
			       union uvc_stream_descriptor **format_desc,
			       const struct video_format_cap *cap)
{
	union uvc_stream_descriptor *desc;

	if (*id >= strm->desc->ifN_format_num) {
		return -ENOMEM;
	}
	desc = &strm->desc->ifN_formats[*id];
	(*id)++;

	desc->format.bDescriptorType = USB_DESC_CS_INTERFACE;
	desc->format.bFormatIndex = ++strm->desc->ifN_header.bNumFormats;
	desc->format.bNumFrameDescriptors = 0;
	LOG_DBG("bFormatIndex = %u", desc->format.bFormatIndex);

	if (cap->pixelformat == VIDEO_PIX_FMT_JPEG) {
		desc->format_mjpeg.bLength = sizeof(desc->format_mjpeg);
		desc->format_mjpeg.bDescriptorSubtype = VS_FORMAT_MJPEG;
		desc->format_mjpeg.bmFlags = 0;
		desc->format_mjpeg.bDefaultFrameIndex = 1;
		desc->format_mjpeg.bAspectRatioX = 0;
		desc->format_mjpeg.bAspectRatioY = 0;
		desc->format_mjpeg.bmInterlaceFlags = 0;
		desc->format_mjpeg.bCopyProtect = 0;
	} else {
		desc->format_uncomp.bLength = sizeof(desc->format_uncomp);
		desc->format_uncomp.bDescriptorSubtype = VS_FORMAT_UNCOMPRESSED;
		memcpy(&desc->format_uncomp.guidFormat, uvc_base_guid, 16);
		desc->format_uncomp.guidFormat[0] = cap->pixelformat >> 0;
		desc->format_uncomp.guidFormat[1] = cap->pixelformat >> 8;
		desc->format_uncomp.guidFormat[2] = cap->pixelformat >> 16;
		desc->format_uncomp.guidFormat[3] = cap->pixelformat >> 24;
		desc->format_uncomp.bBitsPerPixel = video_pix_fmt_bpp(cap->pixelformat) * 8;
		desc->format_uncomp.bDefaultFrameIndex = 1;
		desc->format_uncomp.bAspectRatioX = 0;
		desc->format_uncomp.bAspectRatioY = 0;
		desc->format_uncomp.bmInterlaceFlags = 0;
		desc->format_uncomp.bCopyProtect = 0;
	}

	*format_desc = desc;
	return 0;
}

static int uvc_add_frame_desc(struct uvc_stream *strm, int *id,
			      union uvc_stream_descriptor *format_desc,
			      const struct video_format_cap *cap, bool min)
{
	uint16_t w = min ? cap->width_min : cap->width_max;
	uint16_t h = min ? cap->height_min : cap->height_max;
	struct video_format fmt = {.pixelformat = cap->pixelformat, .width = w, .height = h};
	struct video_frmival_enum fie = {.format = &fmt};
	union uvc_stream_descriptor *desc;

	if (*id >= strm->desc->ifN_format_num) {
		return -ENOMEM;
	}
	desc = &strm->desc->ifN_formats[*id];
	(*id)++;

	desc->frame_discrete.bLength = sizeof(desc->frame_discrete);
	desc->frame_discrete.bDescriptorType = USB_DESC_CS_INTERFACE;
	desc->frame_discrete.bDescriptorSubtype =
		format_desc->frame_discrete.bDescriptorSubtype == VS_FORMAT_UNCOMPRESSED
			? VS_FRAME_UNCOMPRESSED
			: VS_FRAME_MJPEG;
	desc->frame_discrete.bFrameIndex = ++format_desc->format.bNumFrameDescriptors;
	desc->frame_discrete.bmCapabilities = 0;
	desc->frame_discrete.wWidth = w;
	desc->frame_discrete.wHeight = h;
	desc->frame_discrete.dwMinBitRate = 0;
	desc->frame_discrete.dwMaxBitRate = 0;
	desc->frame_discrete.dwMaxVideoFrameBufferSize =
		w * h * video_pix_fmt_bpp(cap->pixelformat) * 8;
	desc->frame_discrete.bFrameIntervalType = 0;

	LOG_DBG("bFrameIndex = %u", desc->frame_continuous.bFrameIndex);

	for (int i = 0; video_enum_frmival(strm->video_dev, VIDEO_EP_OUT, &fie) == 0; i++) {
		if (i >= CONFIG_USBD_VIDEO_MAX_FRMIVAL) {
			LOG_WRN("out of descriptor storage");
			return -ENOSPC;
		}
		if (fie.type != VIDEO_FRMIVAL_TYPE_DISCRETE) {
			LOG_WRN("Only discrete frame intervals are supported");
			return -EINVAL;
		}
		desc->frame_discrete.dwFrameInterval[i] = video_frmival_nsec(&fie.discrete) / 100;
		desc->frame_discrete.bFrameIntervalType++;
	}
	desc->frame_discrete.dwDefaultFrameInterval = desc->frame_discrete.dwFrameInterval[0];

	return 0;
}

static int uvc_preinit_formats(struct uvc_stream *strm)
{
	union uvc_stream_descriptor *format_desc = NULL;
	struct video_caps caps;
	uint32_t prev_pixfmt = 0;
	int id = 0;
	int ret;

	ret = video_get_caps(strm->video_dev, VIDEO_EP_OUT, &caps);
	if (ret < 0) {
		LOG_DBG("Could not load %s video format list, not configuring descriptors",
			strm->video_dev->name);
		return ret;
	}

	for (int i = 0; caps.format_caps[i].pixelformat != 0; i++) {
		const struct video_format_cap *cap = &caps.format_caps[i];

		if (prev_pixfmt != cap->pixelformat) {
			ret = uvc_add_format_desc(strm, &id, &format_desc, cap);
			if (ret < 0) {
				return ret;
			}
		}

		if (cap->width_min != cap->width_max || cap->height_min != cap->height_max) {
			ret = uvc_add_frame_desc(strm, &id, format_desc, cap, true);
			if (ret < 0) {
				return ret;
			}
		}
		ret = uvc_add_frame_desc(strm, &id, format_desc, cap, false);
		if (ret < 0) {
			return ret;
		}

		prev_pixfmt = cap->pixelformat;
	}

	return 0;
}

static void uvc_preinit_desc_ptrs(const struct device *dev, struct usb_desc_header **desc_ptrs)
{
	struct uvc_data *data = dev->data;
	int src = 0;
	int dst = 0;

	LOG_DBG("Skipping empty gaps in the descriptor pointer list");
	do {
		if (desc_ptrs[src]->bLength == 0 && desc_ptrs[src] != &data->desc->nil_desc) {
			LOG_DBG("Skipping %u, bDescriptorType %u",
				src, desc_ptrs[src]->bDescriptorType);
			src++;
		} else {
			LOG_DBG("Copying %u to %u, bDescriptorType %u",
				src, dst, desc_ptrs[src]->bDescriptorType);
			desc_ptrs[dst] = desc_ptrs[src];
			src++, dst++;
		}
	} while (desc_ptrs[src] != &data->desc->nil_desc);
}

static int uvc_preinit(const struct device *dev)
{
	struct uvc_data *data = dev->data;
	int ret;

	for (int i = 0; data->streams[i].dev != NULL; i++) {
		struct uvc_stream *strm = &data->streams[i];

		k_fifo_init(&strm->fifo_in);
		k_fifo_init(&strm->fifo_out);
		k_work_init(&strm->work, &uvc_worker);

		/* Probe each video device for their controls support within each unit */
		uvc_preinit_control(strm, uvc_control_ct, &strm->ct_mask, CT_CONTROL_MAX);
		uvc_preinit_control(strm, uvc_control_pu, &strm->pu_mask, PU_CONTROL_MAX);
		uvc_preinit_control(strm, uvc_control_su, &strm->su_mask, SU_CONTROL_MAX);
		uvc_preinit_control(strm, uvc_control_xu, &strm->xu_mask, XU_CONTROL_MAX);

		/* Convert between UVC bitmask formats */
		uvc_preinit_control_bits(strm);

		/* Probe each video device for their format support */
		ret = uvc_preinit_formats(strm);
		if (ret < 0) {
			return ret;
		}
	}

	uvc_preinit_desc_ptrs(dev, data->fs_desc);
	uvc_preinit_desc_ptrs(dev, data->hs_desc);

	return 0;
}

/* Helpers for fields that are present inside the USB descriptors. */
#define FRMIVAL(n, prop, i)							\
	sys_cpu_to_le64(DT_PHA_BY_IDX(n, prop, i, max_fps))
#define BUFSIZE(n, prop, i)							\
	sys_cpu_to_le32(DT_PHA_BY_IDX(n, prop, i, bits_per_pixel) *		\
			DT_PHA_BY_IDX(n, prop, i, width) *			\
			DT_PHA_BY_IDX(n, prop, i, height))
#define GUID(fourcc)								\
	((fourcc) >> 0 & 0xff), ((fourcc) >> 8 & 0xff),				\
	((fourcc) >> 16 & 0xff), ((fourcc) >> 24 & 0xff),			\
	0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71

enum uvc_unit_id {
	UVC_UNIT_ID_ZERO = 0,
#define UVC_UNIT_ID(n)								\
	UVC_UNIT_ID_CT_##n,							\
	UVC_UNIT_ID_SU_##n,							\
	UVC_UNIT_ID_PU_##n,							\
	UVC_UNIT_ID_XU_##n,							\
	UVC_UNIT_ID_OT_##n,
#define UVC_UNIT_IDS(n) DT_FOREACH_CHILD(DT_INST_CHILD(n, port), UVC_UNIT_ID)
	DT_INST_FOREACH_STATUS_OKAY(UVC_UNIT_IDS)
};

#define UVC_DEFINE_STREAM_DESCRIPTOR(n)						\
static union uvc_stream_descriptor						\
	uvc_fmt_frm_desc_##n[CONFIG_USBD_VIDEO_MAX_VS_DESC];			\
										\
struct usbd_uvc_strm_desc uvc_strm_desc_##n = {					\
	.if0_ct = {								\
		.bLength = sizeof(struct uvc_camera_terminal_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_INPUT_TERMINAL,			\
		.bTerminalID = UVC_UNIT_ID_CT_##n,				\
		.wTerminalType = sys_cpu_to_le16(ITT_CAMERA),			\
		.bAssocTerminal = 0,						\
		.iTerminal = 0,							\
		.wObjectiveFocalLengthMin = sys_cpu_to_le16(0),			\
		.wObjectiveFocalLengthMax = sys_cpu_to_le16(0),			\
		.wOcularFocalLength = sys_cpu_to_le16(0),			\
		.bControlSize = 3,						\
		.bmControls = {0},						\
	},									\
										\
	.if0_su = {								\
		.bLength = sizeof(struct uvc_selector_unit_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_SELECTOR_UNIT,				\
		.bUnitID = UVC_UNIT_ID_SU_##n,					\
		.bNrInPins = 1,							\
		.baSourceID = {UVC_UNIT_ID_CT_##n},				\
		.iSelector = 0,							\
	},									\
										\
	.if0_pu = {								\
		.bLength = 13,							\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_PROCESSING_UNIT,			\
		.bUnitID = UVC_UNIT_ID_PU_##n,					\
		.bSourceID = UVC_UNIT_ID_SU_##n,				\
		.wMaxMultiplier = sys_cpu_to_le16(0),				\
		.bControlSize = 3,						\
		.bmControls = {0},						\
		.iProcessing = 0,						\
		.bmVideoStandards = 0,						\
	},									\
										\
	.if0_xu = {								\
		.bLength = sizeof(struct uvc_extension_unit_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_EXTENSION_UNIT,			\
		.bUnitID = UVC_UNIT_ID_XU_##n,					\
		.guidExtensionCode = {0},					\
		.bNumControls = 0,						\
		.bNrInPins = 1,							\
		.baSourceID = {UVC_UNIT_ID_PU_##n},				\
		.bControlSize = 3,						\
		.bmControls = {0},						\
		.iExtension = 0,						\
	},									\
										\
	.if0_ot = {								\
		.bLength = sizeof(struct uvc_output_terminal_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_OUTPUT_TERMINAL,			\
		.bTerminalID = UVC_UNIT_ID_OT_##n,				\
		.wTerminalType = sys_cpu_to_le16(TT_STREAMING),			\
		.bAssocTerminal = 0,						\
		.bSourceID = UVC_UNIT_ID_XU_##n,				\
		.iTerminal = 0,							\
	},									\
										\
	.ifN = {								\
		.bLength = sizeof(struct usb_if_descriptor),			\
		.bDescriptorType = USB_DESC_INTERFACE,				\
		.bInterfaceNumber = 0,						\
		.bAlternateSetting = 0,						\
		.bNumEndpoints = 1,						\
		.bInterfaceClass = USB_BCC_VIDEO,				\
		.bInterfaceSubClass = SC_VIDEOSTREAMING,			\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
										\
	.ifN_header = {								\
		.bLength = sizeof(struct uvc_stream_header_descriptor) + 1,	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VS_INPUT_HEADER,				\
		.bNumFormats = 0,						\
		.wTotalLength = 0,						\
		.bEndpointAddress = 0x81,					\
		.bmInfo = 0,							\
		.bTerminalLink = UVC_UNIT_ID_OT_##n,				\
		.bStillCaptureMethod = 0,					\
		.bTriggerSupport = 0,						\
		.bTriggerUsage = 0,						\
		.bControlSize = 0,						\
	},									\
										\
	.ifN_formats = uvc_fmt_frm_desc_##n,					\
	.ifN_format_num = ARRAY_SIZE(uvc_fmt_frm_desc_##n),			\
										\
	.ifN_ep_fs = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x81,					\
		.bmAttributes = USB_EP_TYPE_BULK,				\
		.wMaxPacketSize = sys_cpu_to_le16(64),				\
		.bInterval = 0,							\
	},									\
										\
	.ifN_ep_hs = {								\
		.bLength = sizeof(struct usb_ep_descriptor),			\
		.bDescriptorType = USB_DESC_ENDPOINT,				\
		.bEndpointAddress = 0x81,					\
		.bmAttributes = USB_EP_TYPE_BULK,				\
		.wMaxPacketSize = sys_cpu_to_le16(512),				\
		.bInterval = 0,							\
	},									\
										\
	.ifN_color = {								\
		.bLength = sizeof(struct uvc_color_descriptor),			\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VS_COLORFORMAT,				\
		.bColorPrimaries = 1, /* BT.709, sRGB (default) */		\
		.bTransferCharacteristics = 1, /* BT.709 (default) */		\
		.bMatrixCoefficients = 4, /* SMPTE 170M, BT.601 (default) */	\
	},									\
};

#define UVC_DEFINE_DESCRIPTOR(n)						\
static struct usbd_uvc_desc uvc_desc_##n = {					\
	.iad = {								\
		.bLength = sizeof(struct usb_association_descriptor),		\
		.bDescriptorType = USB_DESC_INTERFACE_ASSOC,			\
		.bFirstInterface = 0,						\
		.bInterfaceCount = 1 + DT_CHILD_NUM(DT_INST_CHILD(n, port)),	\
		.bFunctionClass = USB_BCC_VIDEO,				\
		.bFunctionSubClass = SC_VIDEO_INTERFACE_COLLECTION,		\
		.bFunctionProtocol = 0,						\
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
		.bInterfaceSubClass = SC_VIDEOCONTROL,				\
		.bInterfaceProtocol = 0,					\
		.iInterface = 0,						\
	},									\
										\
	.if0_header = {								\
		.bLength = sizeof(struct uvc_control_header_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_HEADER,				\
		.bcdUVC = sys_cpu_to_le16(0x0150),				\
		.wTotalLength = 0,						\
		.dwClockFrequency = sys_cpu_to_le32(30000000),			\
		.bInCollection = DT_CHILD_NUM(DT_INST_CHILD(n, port)),		\
		.baInterfaceNr = {0},						\
	},									\
										\
	.nil_desc = {								\
		.bLength = 0,							\
		.bDescriptorType = 0,						\
	},									\
};										\
										\
static struct usb_desc_header *uvc_fs_desc_##n[] = {				\
	(struct usb_desc_header *)&uvc_desc_##n.iad,				\
	(struct usb_desc_header *)&uvc_desc_##n.if0,				\
	(struct usb_desc_header *)&uvc_desc_##n.if0_header,			\
	DT_FOREACH_CHILD(DT_INST_CHILD(n, port), UVC_VIDEO_CONTROL_PTRS)	\
	DT_FOREACH_CHILD(DT_INST_CHILD(n, port), UVC_VIDEO_STREAMING_PTRS_FS)	\
	(struct usb_desc_header *)&uvc_desc_##n.nil_desc,			\
};										\
										\
static struct usb_desc_header *uvc_hs_desc_##n[] = {				\
	(struct usb_desc_header *)&uvc_desc_##n.iad,				\
	(struct usb_desc_header *)&uvc_desc_##n.if0,				\
	(struct usb_desc_header *)&uvc_desc_##n.if0_header,			\
	DT_FOREACH_CHILD(DT_INST_CHILD(n, port), UVC_VIDEO_CONTROL_PTRS)	\
	DT_FOREACH_CHILD(DT_INST_CHILD(n, port), UVC_VIDEO_STREAMING_PTRS_HS)	\
	(struct usb_desc_header *)&uvc_desc_##n.nil_desc,			\
};

#define UVC_VIDEO_CONTROL_PTRS(n)						\
	(struct usb_desc_header *)&uvc_strm_desc_##n.if0_ct,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.if0_su,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.if0_pu,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.if0_xu,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.if0_ot,
#define UVC_VIDEO_STREAMING_PTRS(i, n)						\
	(struct usb_desc_header *)&uvc_fmt_frm_desc_##n[i],
#define UVC_VIDEO_STREAMING_PTRS_FS(n)						\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN_header,		\
	LISTIFY(CONFIG_USBD_VIDEO_MAX_VS_DESC, UVC_VIDEO_STREAMING_PTRS, (), n)	\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN_color,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN_ep_fs,
#define UVC_VIDEO_STREAMING_PTRS_HS(n)						\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN_header,		\
	LISTIFY(CONFIG_USBD_VIDEO_MAX_VS_DESC, UVC_VIDEO_STREAMING_PTRS, (), n)	\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN_color,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN_ep_hs,


/* See #80649 */

/* Handle the variability of "ports{port@0{}};" vs "port{};" while going up */
#define DT_ENDPOINT_PARENT_DEVICE(node)						\
	COND_CODE_1(DT_NODE_EXISTS(DT_CHILD(DT_GPARENT(node), port)),		\
		    (DT_GPARENT(node)), (DT_PARENT(DT_GPARENT(node))))

/* Handle the "remote-endpoint-label" */
#define DEVICE_DT_GET_REMOTE_DEVICE(node)					\
	DEVICE_DT_GET(DT_ENDPOINT_PARENT_DEVICE(				\
		DT_NODELABEL(DT_STRING_TOKEN(node, remote_endpoint_label))))

#define UVC_STREAM(n)								\
	{									\
		.dev = DEVICE_DT_GET(DT_GPARENT(n)),				\
		.desc = &uvc_strm_desc_##n,					\
		.payload_header.bHeaderLength = CONFIG_USBD_VIDEO_HEADER_SIZE,	\
		.format_id = 1,							\
		.frame_id = 1,							\
		.video_dev = DEVICE_DT_GET_REMOTE_DEVICE(n),			\
		.video_frmival.denominator = NSEC_PER_SEC / 100,		\
	},

#define USBD_UVC_DT_DEVICE_DEFINE(n)						\
	DT_FOREACH_CHILD(DT_INST_CHILD(n, port), UVC_DEFINE_STREAM_DESCRIPTOR)	\
	UVC_DEFINE_DESCRIPTOR(n)						\
										\
	USBD_DEFINE_CLASS(uvc_c_data_##n, &uvc_class_api,			\
			  (void *)DEVICE_DT_INST_GET(n), NULL);			\
										\
	/* Storage for each VideoStreaming interface with most runtime data */	\
	static struct uvc_stream uvc_streams_##n[] = {				\
		DT_FOREACH_CHILD(DT_INST_CHILD(n, port), UVC_STREAM)		\
		{0},								\
	};									\
										\
	struct uvc_data uvc_data_##n = {					\
		.c_data = &uvc_c_data_##n,					\
		.streams = uvc_streams_##n,					\
		.desc = &uvc_desc_##n,						\
		.fs_desc = uvc_fs_desc_##n,					\
		.hs_desc = uvc_hs_desc_##n,					\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, uvc_preinit, NULL, &uvc_data_##n, NULL,	\
		POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &uvc_video_api);

USBD_UVC_DT_DEVICE_DEFINE(0)
