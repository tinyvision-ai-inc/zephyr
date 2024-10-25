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
#define SC_VIDEO_INTERFACE_COLLECITON		0x03

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

/* Extension Unit Controls */
#define XU_BASE_CONTROL				0x00
#define XU_BASE_BIT				0

enum uvc_control_type {
	/* Camera Terminal control type */
	CTYPE_CT,
	/* Input Terminal control type */
	CTYPE_IT,
	/* Processing Unit control type */
	CTYPE_PU,
	/* Selector Unit control type */
	CTYPE_SU,
	/* Encoding Unit control type */
	CTYPE_EU,
	/* Extension Unit control type */
	CTYPE_XU,
	/* Output Terminal control type */
	CTYPE_OT,
};

enum uvc_status {
	UVC_CLASS_ENABLED,
	UVC_CLASS_READY,
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

struct uvc_unit_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bUnitID;
};

struct uvc_input_terminal_descriptor {
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
	uint8_t baSourceID[CONFIG_USBD_VIDEO_MAX_SOURCES];
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
	uint8_t bmControls;
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
	uint8_t bmaControls[3];
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
	uint8_t bNumCompression ;
	uint8_t Pattern;
	uint8_t bCompression[1];
} __packed;

struct uvc_color_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bColorPrimaries;
	uint8_t bTransferCharacteristics;
	uint8_t bMatrixCoefficients;
} __packed;

struct uvc_uncomp_format_descriptor {
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

struct uvc_mjpeg_format_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFormatIndex;
	uint8_t bNumFrameDescriptors;
	uint8_t bmFlags;
#define UVC_MJPEG_FLAGS_FIXEDSIZESAMPLES	(1 << 0)
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
	uint32_t dwFrameInterval[CONFIG_USBD_VIDEO_MAX_FRMIVALS];
} __packed;

struct uvc_format_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFormatIndex;
	uint8_t bNumFrameDescriptors;
} __packed;

struct usbd_uvc_desc {
	struct usb_association_descriptor iad;
	struct usb_if_descriptor if0;
	struct uvc_control_header_descriptor if0_header;
	struct usb_desc_header nil_desc;
};

struct usbd_uvc_ctrl_it_desc {
	struct uvc_input_terminal_descriptor if0_it;
};

struct usbd_uvc_ctrl_ct_desc {
	struct uvc_input_terminal_descriptor if0_ct;
};

struct usbd_uvc_ctrl_pu_desc {
	struct uvc_processing_unit_descriptor if0_pu;
};

struct usbd_uvc_ctrl_eu_desc {
	struct uvc_encoding_unit_descriptor if0_eu;
};

struct usbd_uvc_ctrl_su_desc {
	struct uvc_selector_unit_descriptor if0_su;
};

struct usbd_uvc_ctrl_xu_desc {
	struct uvc_extension_unit_descriptor if0_xu;
};

struct usbd_uvc_ctrl_ot_desc {
	struct uvc_output_terminal_descriptor if0_ot;
};

struct usbd_uvc_strm_desc {
	struct usb_if_descriptor ifN;
	struct uvc_stream_header_descriptor ifN_header;
	struct usb_ep_descriptor ifN_ep_fs;
	struct usb_ep_descriptor ifN_ep_hs;
	struct uvc_color_descriptor ifN_color;
};

struct usbd_uvc_strm_uncomp_desc {
	struct uvc_uncomp_format_descriptor ifN_format;
	struct uvc_frame_discrete_descriptor ifN_frame;
};

struct usbd_uvc_strm_mjpeg_desc {
	struct uvc_mjpeg_format_descriptor ifN_format;
	struct uvc_frame_discrete_descriptor ifN_frame;
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

struct uvc_payload_header {
	uint8_t bHeaderLength;
	uint8_t bmHeaderInfo;
	uint32_t dwPresentationTime; /* optional */
	uint32_t scrSourceClockSTC;  /* optional */
	uint16_t scrSourceClockSOF;  /* optional */
} __packed;

/* Information specific to each VideoControl interface */
struct uvc_control {

	/* UVC protocol state */

	/* USB descriptors */
	struct uvc_unit_descriptor *desc;
	/* USB string descriptor */
	struct usbd_desc_node *desc_nd;
	/* Stream interface affected by this control */
	struct uvc_stream *stream;
	/* Bitmask of controls enabled for this interface */
	uint64_t mask;
	/* 0-terminated list of source IDs */
	uint16_t *source_ids;

	/* Video API state */

	/* Device that is affected by this control */
	const struct device *dev;
	/* Control handler function */
	int (*fn)(const struct usb_setup_packet *setup, struct net_buf *buf,
		  const struct device *dev);
};

/* Information specific to each VideoStreaming interface */
struct uvc_stream {

	/* UVC protocol state */

	/* USBD class state */
	atomic_t state;
	/* VideoStreaming-specific descriptor collection */
	struct usbd_uvc_strm_desc *desc;
	/* UVC probe-commit control default values */
	struct uvc_probe_msg default_probe;
	/* UVC payload header, passed just before the image data */
	struct uvc_payload_header payload_header;
	/* UVC format currently selected */
	int format_id;

	/* Video API state */

	/* Device where the data is enqueued/dequeued */
	const struct device *dev;
	/* Self reference for use in work queues, loosing the context */
	const struct device *self;
	/* Video capabilities selected for this device */
	const struct video_format_cap *caps;
	/* UVC worker to process the queue */
	struct k_work work;
	/* Video FIFOs for submission (in) and completion (out) queue */
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	/* Offset in bytes within the fragment including the header */
	uint32_t xfer_offset;
};

/* Global information */
struct uvc_data {
	/* UVC error from latest request */
	int err;
	/* USBD class structure */
	struct usbd_class_data *c_data;
	/* UVC lookup tables */
	const struct uvc_control *controls;
	struct uvc_stream *streams;
	/* UVC Descriptors */
	struct usbd_uvc_desc *desc;
	struct usb_desc_header *const *fs_desc;
	struct usb_desc_header *const *hs_desc;
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

static uint8_t uvc_get_bulk_in(const struct device *dev, struct uvc_stream *strm)
{
	struct uvc_data *data = dev->data;

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

static uint32_t uvc_get_max_frame_size(struct uvc_stream *strm)
{
	const struct video_format_cap *cap = &strm->caps[strm->format_id];

	return cap->width_max * cap->height_max * video_bits_per_pixel(cap->pixelformat);
}

static int uvc_control_probe_format_index(const struct device *dev, struct uvc_stream *strm,
					  uint8_t request, struct uvc_probe_msg *probe)
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
					 uint8_t request, struct uvc_probe_msg *probe)
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
					    uint8_t request, struct uvc_probe_msg *probe)
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
						  uint8_t request, struct uvc_probe_msg *probe)
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
					      uint8_t request, struct uvc_probe_msg *probe)
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
			     struct uvc_probe_msg *probe)
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
			      struct uvc_probe_msg *probe)
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
	struct uvc_probe_msg *probe = (void *)buf->data;

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

__unused static int uvc_control_it(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	LOG_WRN("control: nothing supported for Input Terminal");
	return -ENOTSUP;
};

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

#define CTYPE_VIDEO_CID_CAMERA_EXPOSURE CTYPE_CT
#define CTYPE_VIDEO_CID_CAMERA_ZOOM     CTYPE_CT

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

#define CTYPE_VIDEO_CID_CAMERA_BRIGHTNESS CTYPE_PU
#define CTYPE_VIDEO_CID_CAMERA_CONTRAST   CTYPE_PU
#define CTYPE_VIDEO_CID_CAMERA_GAIN       CTYPE_PU
#define CTYPE_VIDEO_CID_CAMERA_SATURATION CTYPE_PU
#define CTYPE_VIDEO_CID_CAMERA_WHITE_BAL  CTYPE_PU

__unused static int uvc_control_eu(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	LOG_WRN("control: nothing supported for Encoding Unit");
	return -ENOTSUP;
};

__unused static int uvc_control_su(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	LOG_WRN("control: nothing supported for Selector Unit");
	return -ENOTSUP;
};

#define CTYPE_VIDEO_CID_TEST_PATTERN CTYPE_SU

__unused static int uvc_control_xu(const struct usb_setup_packet *setup, struct net_buf *buf,
				   const struct device *dev)
{
	LOG_WRN("control: nothing supported for Extension Unit");
	return -ENOTSUP;
};

#define CTYPE_VIDEO_CTRL_CLASS_VENDOR CTYPE_XU

__unused static int uvc_control_ot(const struct usb_setup_packet *setup, struct net_buf *buf,
			  const struct device *dev)
{
	LOG_WRN("control: nothing supported for Output Terminal");
	return -ENOTSUP;
};

static int uvc_control_vc(const struct device *dev, const struct uvc_control *ctrl,
			  const struct usb_setup_packet *setup, struct net_buf *buf)
{
	struct uvc_data *data = dev->data;
	uint8_t control_selector = setup->wValue >> 8;
	uint8_t unit_id = (setup->wIndex >> 8) & 0xff;
	int ret;

	if ((ctrl->mask & BIT(control_selector)) == 0) {
		LOG_WRN("control selector %u not enabled for bUnitID %u",
		control_selector, unit_id);
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
	struct uvc_data *data = dev->data;
	uint8_t ifnum = (setup->wIndex >> 0) & 0xff;
	uint8_t unit_id = (setup->wIndex >> 8) & 0xff;

	LOG_DBG("Host send a %s control command", uvc_get_request_str(setup));

	/* VideoStreaming requests */

	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++) {
		if (strm->desc->ifN.bInterfaceNumber == ifnum) {
			return uvc_control_vs(dev, strm, setup, buf);
		}
	}

	if (ifnum == data->desc->if0.bInterfaceNumber) {
		LOG_WRN("control: interface %u not found", ifnum);
		data->err = ERR_INVALID_UNIT;
		return -ENOTSUP;
	}

	/* VideoControl requests */

	if (unit_id == 0) {
		return uvc_control_errno(setup, buf, data->err);
	}

	for (const struct uvc_control *ctrl = data->controls; ctrl->dev != NULL; ctrl++) {
		if (ctrl->desc->bUnitID == unit_id) {
			return uvc_control_vc(dev, ctrl, setup, buf);
		}
	}

	LOG_WRN("control: no unit %u found", unit_id);
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
	struct uvc_buf_info bi = *(struct uvc_buf_info *)udc_get_buf_info(buf);
	const struct device *dev = usbd_class_get_private(c_data);

	net_buf_unref(buf);

	if (bi.udc.ep == uvc_get_bulk_in(dev, bi.stream)) {
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
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;
	int ret;

	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++) {
		/* Get the default probe by querying the current probe at startup */
		ret = uvc_control_probe(dev, strm, GET_CUR, &strm->default_probe);
		if (ret < 0) {
			LOG_ERR("init: failed to query the default probe");
			return ret;
		}
	}

	for (const struct uvc_control *ctrl = data->controls; ctrl->dev != NULL; ctrl++) {
		struct usbd_desc_node *desc_nd = ctrl->desc_nd;
		struct uvc_unit_descriptor *desc = ctrl->desc;

		LOG_DBG("Adding string descriptor '%s'", (char *)desc_nd->ptr);

		ret = usbd_add_descriptor(uds_ctx, desc_nd);
		if (ret < 0) {
			LOG_WRN("Failed to add string descriptor %s", (char *)desc_nd->ptr);
			continue;
		}

		switch (desc->bDescriptorSubtype) {
		case VC_INPUT_TERMINAL:
			((struct uvc_camera_terminal_descriptor *)desc)->iTerminal =
				desc_nd->str.idx;
			break;
		case VC_OUTPUT_TERMINAL:
			((struct uvc_output_terminal_descriptor *)desc)->iTerminal =
				desc_nd->str.idx;
			break;
		case VC_SELECTOR_UNIT:
			((uint8_t *)desc)[desc->bLength - 1] =
				desc_nd->str.idx;
			break;
		case VC_PROCESSING_UNIT:
			((struct uvc_processing_unit_descriptor *)desc)->iProcessing =
				desc_nd->str.idx;
			break;
		case VC_ENCODING_UNIT:
			((struct uvc_encoding_unit_descriptor *)desc)->iEncoding =
				desc_nd->str.idx;
			break;
		case VC_EXTENSION_UNIT:
			((struct uvc_extension_unit_descriptor *)desc)->iExtension =
				desc_nd->str.idx;
			break;
		default:
			LOG_WRN("Not adding '%s' to unknown subtype %u",
				(char *)desc_nd->ptr, desc->bUnitID);
			break;
		}
	}

	return 0;
}

static void uvc_update_desc(const struct device *dev)
{
	struct uvc_data *data = dev->data;

	data->desc->iad.bFirstInterface = data->desc->if0.bInterfaceNumber;

	for (size_t i = 0; data->streams[i].dev != NULL; i++) {
		struct uvc_stream *strm = &data->streams[i];

		strm->desc->ifN_header.bEndpointAddress = uvc_get_bulk_in(dev, strm);
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

static int uvc_enqueue_usb(const struct device *dev, struct uvc_stream *strm, struct net_buf *buf, struct video_buffer *vbuf)
{
	struct uvc_data *data = dev->data;
	size_t mps = uvc_get_bulk_mps(data->c_data);
	struct uvc_buf_info *bi = (void *)udc_get_buf_info(buf);
	size_t len = buf->len;
	int ret;

	memset(bi, 0, sizeof(struct udc_buf_info));
	bi->udc.zlp = (vbuf->flags & VIDEO_BUF_EOF) && buf->len == mps;
	bi->udc.ep = uvc_get_bulk_in(dev, strm);
	/* If this is the last buffer, attach vbuf so we can free it from uvc_request() */
	bi->vbuf = (buf->len <= mps) ? vbuf : NULL;
	/* Reference to the stream to be able to find the correct FIFO */
	bi->stream = strm;

	/* Apply USB limit */
	buf->len = len = MIN(buf->len, mps);

	LOG_DBG("Queue USB buffer %p, data %p, size %u, len %u", buf, buf->data, buf->size,
		buf->len);

	ret = usbd_ep_enqueue(data->c_data, buf);
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
	const struct device *dev = strm->self;
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
	struct uvc_data *data = dev->data;

	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++) {
		/* Catch-up with buffers that might have been delayed */
		atomic_set_bit(&strm->state, UVC_CLASS_ENABLED);
		k_work_submit(&strm->work);
	}
}

static void uvc_disable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct uvc_data *data = dev->data;

	for (struct uvc_stream *strm = data->streams; strm->dev != NULL; strm++) {
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
	fmt->pitch = fmt->width * video_bits_per_pixel(cap->pixelformat) / 8;
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
	struct uvc_data *data = dev->data;

	for (size_t i = 0; data->streams[i].dev != NULL; i++) {
		struct uvc_stream *strm = &data->streams[i];

		k_fifo_init(&strm->fifo_in);
		k_fifo_init(&strm->fifo_out);
		k_work_init(&strm->work, &uvc_worker);
	}

	return 0;
}

#define FRMIVAL(n, prop, i) sys_cpu_to_le64(DT_PHA_BY_IDX(n, prop, i, max_fps))
#define BUFSIZE(n, prop, i) sys_cpu_to_le32(0)
#define GUID(fourcc)								\
	((fourcc) >> 0 & 0xff), ((fourcc) >> 8 & 0xff),				\
	((fourcc) >> 16 & 0xff), ((fourcc) >> 24 & 0xff),			\
	0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71

/* This will generate a list such as "fn(n, ct, CT) fn(n, pu, PU)..."
 * with one entry per control entity to add. The control entities are selected
 * according to the video-controls properties of every node:
 *
 * - If video-controls has at a CID matching CTYPE_CT, "fn(n, ct, CT)" is added
 * - If video-controls has at a CID matching CTYPE_PU, "fn(n, pu, PU)" is added
 * - If video-controls has at a CID matching CTYPE_EU, "fn(n, eu, EU)" is added
 * - So forth for all possible "CTYPE_??"
 *
 * The matching of CIDs and CTYPEs is done via the macros "CTYPE_VIDEO_CID_*".
 */
#define PROP_IS_CTYPE(n, prop, i, t)						\
	IS_EQ(t, DT_CAT(CTYPE_, DT_STRING_TOKEN_BY_IDX(n, prop, i)))
#define CTYPE_ELEM_IF_EQ(n, prop, i, t)						\
	COND_CODE_1(PROP_IS_CTYPE(n, prop, i, t), (ELEM,), ())
#define CTYPE_LIST(n, t)							\
	DT_FOREACH_PROP_ELEM_VARGS(n, video_controls, CTYPE_ELEM_IF_EQ, t)
#define CTYPE_NUM(n, t)								\
	NUM_VA_ARGS_LESS_1(CTYPE_LIST(n, t) ELEM, ELEM)
#define VIDEO_CONTROL_NODES(n, fn)						\
	COND_CODE_1(DT_NODE_HAS_PROP(n, video_controls), (			\
		COND_CODE_0(CTYPE_NUM(n, CTYPE_CT), (), (fn(n, ct, CT)))	\
		COND_CODE_0(CTYPE_NUM(n, CTYPE_IT), (), (fn(n, it, IT)))	\
		COND_CODE_0(CTYPE_NUM(n, CTYPE_PU), (), (fn(n, pu, PU)))	\
		COND_CODE_0(CTYPE_NUM(n, CTYPE_SU), (), (fn(n, su, SU)))	\
		COND_CODE_0(CTYPE_NUM(n, CTYPE_EU), (), (fn(n, eu, EU)))	\
		COND_CODE_0(CTYPE_NUM(n, CTYPE_XU), (), (fn(n, xu, XU)))	\
		COND_CODE_0(CTYPE_NUM(n, CTYPE_OT), (), (fn(n, ot, OT)))	\
	), ())
#define FOREACH_VIDEO_CONTROL(fn) 						\
	DT_FOREACH_STATUS_OKAY_NODE_VARGS(VIDEO_CONTROL_NODES, fn)

#define FOREACH_VIDEO_STREAM(n, fn, arg)					\
	DT_FOREACH_CHILD_VARGS(DT_INST_CHILD(n, port), fn, arg)

#define UVC_FORMAT_CAP(n, prop, i)						\
	{									\
		.pixelformat = DT_PHA_BY_IDX(n, prop, i, fourcc),		\
		.width_min = DT_PHA_BY_IDX(n, prop, i, width),			\
		.width_max = DT_PHA_BY_IDX(n, prop, i, width),			\
		.width_step = 1,						\
		.height_min = DT_PHA_BY_IDX(n, prop, i, height),		\
		.height_max = DT_PHA_BY_IDX(n, prop, i, height),		\
		.height_step = 1						\
	},

#define UVC_STREAM(n, _)							\
	{									\
		.dev = DEVICE_DT_GET(DT_GPARENT(DT_REMOTE_ENDPOINT(n))),	\
		.self = DEVICE_DT_GET(DT_GPARENT(n)),				\
		.desc = &uvc_strm_desc_##n,					\
		.payload_header.bHeaderLength = CONFIG_USBD_VIDEO_HEADER_SIZE,	\
		.caps = uvc_caps_##n,						\
	},

#define UVC_CONTROL_SOURCE_ID(n) DEP_ORD(DT_GPARENT(DT_REMOTE_ENDPOINT(n)))
#define UVC_CONTROL_SOURCE_IDS(n)						\
	uint16_t uvc_ctrl_source_ids_##n = {					\
		DT_FOREACH_CHILD(DT_CHILD(n, port), UVC_CONTROL_SOURCE_ID), 0 };

#define UVC_CONTROL(n, t, T)							\
	{									\
		.dev = DEVICE_DT_GET(n),					\
		.fn = &uvc_control_##t,						\
		.desc =	(void *) &uvc_ctrl_##t##_desc_##n,			\
		.desc_nd = &uvc_ctrl_##t##_desc_nd_##n,				\
		.source_ids = uvc_ctrl_source_ids_##n,				\
	},

#define UVC_DEFINE_CONTROL_IT_DESCRIPTOR(n)					\
struct usbd_uvc_ctrl_it_desc uvc_ctrl_it_desc_##n = {				\
	.if0_it = {								\
		.bLength = sizeof(struct uvc_input_terminal_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_INPUT_TERMINAL,			\
		.bTerminalID = DT_DEP_ORD(n),					\
		.wTerminalType = sys_cpu_to_le16(ITT_VENDOR_SPECIFIC),		\
		.bAssocTerminal = 0,						\
		.iTerminal = 0,							\
	},									\
};

#define UVC_DEFINE_CONTROL_OT_DESCRIPTOR(n)					\
struct usbd_uvc_ctrl_ot_desc uvc_ctrl_ot_desc_##n = {				\
	.if0_ot = {								\
		.bLength = sizeof(struct uvc_output_terminal_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_OUTPUT_TERMINAL,			\
		.bTerminalID = DT_DEP_ORD(n),					\
		.wTerminalType = sys_cpu_to_le16(TT_STREAMING),			\
		.bAssocTerminal = 0,						\
		.bSourceID = 0,							\
		.iTerminal = 0,							\
	},									\
};

#define UVC_DEFINE_CONTROL_CT_DESCRIPTOR(n)					\
struct usbd_uvc_ctrl_ct_desc uvc_ctrl_ct_desc_##n = {				\
	.if0_ct = {								\
		.bLength = sizeof(struct uvc_camera_terminal_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_INPUT_TERMINAL,			\
		.bTerminalID = DT_DEP_ORD(n),					\
		.wTerminalType = sys_cpu_to_le16(ITT_CAMERA),			\
		.bAssocTerminal = 0,						\
		.iTerminal = 0,							\
		.wObjectiveFocalLengthMin = sys_cpu_to_le16(0),			\
		.wObjectiveFocalLengthMax = sys_cpu_to_le16(0),			\
		.wOcularFocalLength = sys_cpu_to_le16(0),			\
		.bControlSize = 3,						\
		.bmControls = {0},						\
	},									\
};

#define UVC_DEFINE_CONTROL_SU_DESCRIPTOR(n)					\
struct usbd_uvc_ctrl_su_desc uvc_ctrl_su_desc_##n = {				\
	.if0_su = {								\
		.bLength = sizeof(struct uvc_selector_unit_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_SELECTOR_UNIT,				\
		.bUnitID = DT_DEP_ORD(n),					\
		.bNrInPins = 0,							\
		.baSourceID = {0},						\
		.iSelector = 0,							\
	},									\
};

#define UVC_DEFINE_CONTROL_PU_DESCRIPTOR(n)					\
struct usbd_uvc_ctrl_pu_desc uvc_ctrl_pu_desc_##n = {				\
	.if0_pu = {								\
		.bLength = 13,							\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_PROCESSING_UNIT,			\
		.bUnitID = DT_DEP_ORD(n),					\
		.bSourceID = 0,							\
		.wMaxMultiplier = sys_cpu_to_le16(0),				\
		.bControlSize = 3,						\
		.bmControls = {0},						\
		.iProcessing = 0,						\
		.bmVideoStandards = 0,						\
	},									\
};

#define UVC_DEFINE_CONTROL_EU_DESCRIPTOR(n)					\
struct usbd_uvc_ctrl_eu_desc uvc_ctrl_eu_desc_##n = {				\
	.if0_eu = {								\
		.bLength = sizeof(struct uvc_encoding_unit_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_ENCODING_UNIT,				\
		.bUnitID = DT_DEP_ORD(n),					\
		.bSourceID = 0,							\
		.iEncoding = 0,							\
		.bControlSize = 3,						\
		.bmControlsRuntime = {0},					\
	},									\
};

#define UVC_DEFINE_CONTROL_XU_DESCRIPTOR(n)					\
struct usbd_uvc_ctrl_xu_desc uvc_ctrl_xu_desc_##n = {				\
	.if0_xu = {								\
		.bLength = sizeof(struct uvc_extension_unit_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VC_EXTENSION_UNIT,			\
		.bUnitID = DT_DEP_ORD(n),					\
		.guidExtensionCode = {0},					\
		.bNumControls = 0,						\
		.bNrInPins = 0,							\
		.baSourceID = {0},						\
		.bControlSize = 8,						\
		.bmControls = 0,						\
		.iExtension = 0,						\
	},									\
};

#define UVC_DEFINE_STREAM_DESCRIPTOR(n, _)					\
struct usbd_uvc_strm_desc uvc_strm_desc_##n = {					\
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
		.bLength = sizeof(struct uvc_stream_header_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VS_INPUT_HEADER,				\
		.bNumFormats = DT_PROP_LEN(n, formats),				\
		.wTotalLength = 0,						\
		.bEndpointAddress = 0x81,					\
		.bmInfo = 0,							\
		.bTerminalLink = DT_DEP_ORD(n),					\
		.bStillCaptureMethod = 0,					\
		.bTriggerSupport = 0,						\
		.bTriggerUsage = 0,						\
		.bControlSize = 0,						\
	},									\
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
};										\
										\
DT_FOREACH_PROP_ELEM(n, formats, UVC_DEFINE_UNCOMP_DESCRIPTOR)
/* TODO also add support for MJPEG, by comparing the FOURCC integer value */

#define UVC_DEFINE_UNCOMP_DESCRIPTOR(n, prop, i)				\
struct usbd_uvc_strm_uncomp_desc uvc_strm_desc_##n##_##i = {			\
	.ifN_format = {								\
		.bLength = sizeof(struct uvc_uncomp_format_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VS_FORMAT_UNCOMPRESSED,			\
		.bFormatIndex = UTIL_INC(i),					\
		.bNumFrameDescriptors = 1,					\
		.guidFormat = {GUID(DT_PHA_BY_IDX(n, prop, i, fourcc))},	\
		.bBitsPerPixel = DT_PHA_BY_IDX(n, prop, i, bits_per_pixel),	\
		.bDefaultFrameIndex = 1,					\
		.bAspectRatioX = 0,						\
		.bAspectRatioY = 0,						\
		.bmInterlaceFlags = 0,						\
		.bCopyProtect = 0,						\
	},									\
										\
	.ifN_frame = {								\
		.bLength = sizeof(struct uvc_frame_discrete_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VS_FRAME_UNCOMPRESSED,			\
		.bFrameIndex = 1,						\
		.bmCapabilities = 0,						\
		.wWidth = sys_cpu_to_le16(DT_PHA_BY_IDX(n, prop, i, width)),	\
		.wHeight = sys_cpu_to_le16(DT_PHA_BY_IDX(n, prop, i, height)),	\
		.dwMinBitRate = sys_cpu_to_le32(15360000),			\
		.dwMaxBitRate = sys_cpu_to_le32(15360000),			\
		.dwMaxVideoFrameBufferSize = BUFSIZE(n, prop, i),		\
		.dwDefaultFrameInterval = FRMIVAL(n, prop, i),			\
		.bFrameIntervalType = 1,					\
		.dwFrameInterval = {FRMIVAL(n, prop, i)},			\
	},									\
};

#define UVC_DEFINE_MJPEG_DESCRIPTOR(n, prop, i)					\
struct usbd_uvc_strm_mjpeg_desc uvc_strm_desc_##n##_##i = {			\
	.ifN_format = {								\
		.bLength = sizeof(struct uvc_mjpeg_format_descriptor),		\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VS_FORMAT_MJPEG,				\
		.bFormatIndex = id,						\
		.bNumFrameDescriptors = 1,					\
		.bmFlags = BIT(0),						\
		.bDefaultFrameIndex = 1,					\
		.bAspectRatioX = 0,						\
		.bAspectRatioY = 0,						\
		.bmInterlaceFlags = 0,						\
		.bCopyProtect = 0,						\
	},									\
										\
	.ifN_frame = {								\
		.bLength = sizeof(struct uvc_frame_discrete_descriptor),	\
		.bDescriptorType = USB_DESC_CS_INTERFACE,			\
		.bDescriptorSubtype = VS_FRAME_MJPEG,				\
		.bFrameIndex = 1,						\
		.bmCapabilities = 0,						\
		.wWidth = sys_cpu_to_le16(DT_PHA_BY_IDX(n, prop, i, width)),	\
		.wHeight = sys_cpu_to_le16(DT_PHA_BY_IDX(n, prop, i, height)),	\
		.dwMinBitRate = sys_cpu_to_le32(15360000),			\
		.dwMaxBitRate = sys_cpu_to_le32(15360000),			\
		.dwMaxVideoFrameBufferSize = BUFSIZE(n, prop, i),		\
		.dwDefaultFrameInterval = FRMIVAL(n, prop, i),			\
		.bFrameIntervalType = 1,					\
		.dwFrameInterval = FRMIVAL(n, prop, i),				\
	},									\
};

#define UVC_DEFINE_FORMAT_CAPS(n, _)						\
	static const struct video_format_cap uvc_caps_##n[] = {			\
		DT_FOREACH_PROP_ELEM(n, formats, UVC_FORMAT_CAP)		\
		{0},								\
	};

#define UVC_DEFINE_CONTROL_DESCRIPTOR(n, t, T)					\
	UVC_DEFINE_CONTROL_##T##_DESCRIPTOR(n)					\
	USBD_DESC_STRING_DEFINE(uvc_ctrl_##t##_desc_nd_##n,			\
				DT_NODE_FULL_NAME(n),				\
				USBD_DUT_STRING_INTERFACE);

#define UVC_DEFINE_DESCRIPTOR(n)						\
FOREACH_VIDEO_STREAM(n, UVC_DEFINE_STREAM_DESCRIPTOR, _)			\
										\
static struct usbd_uvc_desc uvc_desc_##n = {					\
	.iad = {								\
		.bLength = sizeof(struct usb_association_descriptor),		\
		.bDescriptorType = USB_DESC_INTERFACE_ASSOC,			\
		.bFirstInterface = 0,						\
		.bInterfaceCount = 1 + (n),					\
		.bFunctionClass = USB_BCC_VIDEO,				\
		.bFunctionSubClass = SC_VIDEO_INTERFACE_COLLECITON,		\
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
	},									\
										\
	.nil_desc = {								\
		.bLength = 0,							\
		.bDescriptorType = 0,						\
	},									\
};										\
										\
static struct usb_desc_header *const uvc_fs_desc_##n[] = {			\
	(struct usb_desc_header *)&uvc_desc_##n.iad,				\
	(struct usb_desc_header *)&uvc_desc_##n.if0,				\
	(struct usb_desc_header *)&uvc_desc_##n.if0_header,			\
	FOREACH_VIDEO_CONTROL(VIDEO_CONTROL_PTRS)				\
	FOREACH_VIDEO_STREAM(n, VIDEO_STREAM_PTRS, fs)				\
	(struct usb_desc_header *)&uvc_desc_##n.nil_desc,			\
};										\
										\
static struct usb_desc_header *const uvc_hs_desc_##n[] = {			\
	(struct usb_desc_header *)&uvc_desc_##n.iad,				\
	(struct usb_desc_header *)&uvc_desc_##n.if0,				\
	(struct usb_desc_header *)&uvc_desc_##n.if0_header,			\
	FOREACH_VIDEO_CONTROL(VIDEO_CONTROL_PTRS)				\
	FOREACH_VIDEO_STREAM(n, VIDEO_STREAM_PTRS, hs)				\
	(struct usb_desc_header *)&uvc_desc_##n.nil_desc,			\
};

#define VIDEO_CONTROL_PTRS(n, t, T)						\
	(struct usb_desc_header *)&uvc_ctrl_##t##_desc_##n.if0_##t,

#define VIDEO_FORMAT_PTRS(n, prop, i)						\
	(struct usb_desc_header *)&uvc_strm_desc_##n##_##i.ifN_format,		\
	(struct usb_desc_header *)&uvc_strm_desc_##n##_##i.ifN_frame,
	
#define VIDEO_STREAM_PTRS(n, speed)						\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN_header,		\
	DT_FOREACH_PROP_ELEM(n, formats, VIDEO_FORMAT_PTRS)			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN_color,			\
	(struct usb_desc_header *)&uvc_strm_desc_##n.ifN_ep_##speed,

FOREACH_VIDEO_CONTROL(UVC_DEFINE_CONTROL_DESCRIPTOR)

#define USBD_UVC_DT_DEVICE_DEFINE(n)						\
	UVC_DEFINE_DESCRIPTOR(n)						\
										\
	USBD_DEFINE_CLASS(uvc_c_data_##n, &uvc_class_api,			\
			  (void *)DEVICE_DT_INST_GET(n), NULL);			\
										\
										\
	FOREACH_VIDEO_STREAM(n, UVC_DEFINE_FORMAT_CAPS, _)			\
										\
	static struct uvc_stream uvc_streams_##n[] = {				\
		FOREACH_VIDEO_STREAM(n, UVC_STREAM, _)				\
		{0},								\
	};									\
										\
	static const struct uvc_control uvc_controls_##n[] = {			\
		FOREACH_VIDEO_CONTROL(UVC_CONTROL)				\
		{0},								\
	};									\
										\
	struct uvc_data uvc_data_##n = {					\
		.c_data = &uvc_c_data_##n,					\
		.controls = uvc_controls_##n,					\
		.streams = uvc_streams_##n,					\
		.desc = &uvc_desc_##n,						\
		.fs_desc = uvc_fs_desc_##n,					\
		.hs_desc = uvc_hs_desc_##n,					\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, uvc_preinit, NULL,				\
		&uvc_data_##n, NULL,						\
		POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,			\
		&uvc_video_api);

USBD_UVC_DT_DEVICE_DEFINE(0)
