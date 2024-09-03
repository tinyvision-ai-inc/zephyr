/*
 * Copyright (c) 2024 tinyVision.ai
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The macros in this file are not public, applications should not use them.
 * The macros are used to translate devicetree zephyr,uvc compatible nodes
 * into uint8_t array initializer. The output should be treated as a binary blob
 * for the USB host to use (and parse).
 */

#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>

#ifndef ZEPHYR_INCLUDE_USBD_UVC_MACROS_H_
#define ZEPHYR_INCLUDE_USBD_UVC_MACROS_H_

/* Video Interface Class Code */
#define UVC_CC_VIDEO 0x0E

/* Video Interface Subclass Codes */
#define UVC_SC_UNDEFINED			0x00
#define UVC_SC_VIDEOCONTROL			0x01
#define UVC_SC_VIDEOSTREAMING			0x02
#define UVC_SC_VIDEO_INTERFACE_COLLECITON	0x03

/* Video Interface Protocol Codes */
#define UVC_PC_PROTOCOL_UNDEFINED		0x00
#define UVC_PC_PROTOCOL_15			0x01

/* Video Class-Specific Descriptor Types */
#define UVC_CS_UNDEFINED			0x20
#define UVC_CS_DEVICE				0x21
#define UVC_CS_CONFIGURATION			0x22
#define UVC_CS_STRING				0x23
#define UVC_CS_INTERFACE			0x24
#define UVC_CS_ENDPOINT				0x25

/* Video Class-Specific Video Control Interface Descriptor Subtypes */
#define UVC_VC_DESCRIPTOR_UNDEFINED		0x00
#define UVC_VC_HEADER				0x01
#define UVC_VC_INPUT_TERMINAL			0x02
#define UVC_VC_OUTPUT_TERMINAL			0x03
#define UVC_VC_SELECTOR_UNIT			0x04
#define UVC_VC_PROCESSING_UNIT			0x05
#define UVC_VC_EXTENSION_UNIT			0x06
#define UVC_VC_ENCODING_UNIT			0x07

/* Video Class-Specific Video Stream Interface Descriptor Subtypes */
#define UVC_VS_UNDEFINED			0x00
#define UVC_VS_INPUT_HEADER			0x01
#define UVC_VS_OUTPUT_HEADER			0x02
#define UVC_VS_STILL_IMAGE_FRAME		0x03
#define UVC_VS_FORMAT_UNCOMPRESSED		0x04
#define UVC_VS_FRAME_UNCOMPRESSED		0x05
#define UVC_VS_FORMAT_MJPEG			0x06
#define UVC_VS_FRAME_MJPEG			0x07
#define UVC_VS_FORMAT_MPEG2TS			0x0A
#define UVC_VS_FORMAT_DV			0x0C
#define UVC_VS_COLORFORMAT			0x0D
#define UVC_VS_FORMAT_FRAME_BASED		0x10
#define UVC_VS_FRAME_FRAME_BASED		0x11
#define UVC_VS_FORMAT_STREAM_BASED		0x12
#define UVC_VS_FORMAT_H264			0x13
#define UVC_VS_FRAME_H264			0x14
#define UVC_VS_FORMAT_H264_SIMULCAST		0x15
#define UVC_VS_FORMAT_VP8			0x16
#define UVC_VS_FRAME_VP8			0x17
#define UVC_VS_FORMAT_VP8_SIMULCAST		0x18

/* Video Class-Specific Endpoint Descriptor Subtypes */
#define UVC_EP_UNDEFINED			0x00
#define UVC_EP_GENERAL				0x01
#define UVC_EP_ENDPOINT				0x02
#define UVC_EP_INTERRUPT			0x03

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

/* VideoControl InterfaceSelectors */
#define UVC_VC_CONTROL_UNDEFINED		0x00
#define UVC_VC_VIDEO_POWER_MODE_CONTROL		0x01
#define UVC_VC_REQUEST_ERROR_CODE_CONTROL	0x02

/* Terminal Control Selectors */
#define UVC_TE_CONTROL_UNDEFINED		0x00

/* Selector Unit Control Selectors */
#define UVC_SU_CONTROL_UNDEFINED		0x00
#define UVC_SU_INPUT_SELECT_CONTROL		0x01

/* Camera Terminal Control Selectors */
#define UVC_CT_CONTROL_UNDEFINED		0x00
#define UVC_CT_SCANNING_MODE_CONTROL		0x01
#define UVC_CT_AE_MODE_CONTROL			0x02
#define UVC_CT_AE_PRIORITY_CONTROL		0x03
#define UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL	0x04
#define UVC_CT_EXPOSURE_TIME_RELATIVE_CONTROL	0x05
#define UVC_CT_FOCUS_ABSOLUTE_CONTROL		0x06
#define UVC_CT_FOCUS_RELATIVE_CONTROL		0x07
#define UVC_CT_FOCUS_AUTO_CONTROL		0x08
#define UVC_CT_IRIS_ABSOLUTE_CONTROL		0x09
#define UVC_CT_IRIS_RELATIVE_CONTROL		0x0A
#define UVC_CT_ZOOM_ABSOLUTE_CONTROL		0x0B
#define UVC_CT_ZOOM_RELATIVE_CONTROL		0x0C
#define UVC_CT_PANTILT_ABSOLUTE_CONTROL		0x0D
#define UVC_CT_PANTILT_RELATIVE_CONTROL		0x0E
#define UVC_CT_ROLL_ABSOLUTE_CONTROL		0x0F
#define UVC_CT_ROLL_RELATIVE_CONTROL		0x10
#define UVC_CT_PRIVACY_CONTROL			0x11
#define UVC_CT_FOCUS_SIMPLE_CONTROL		0x12
#define UVC_CT_WINDOW_CONTROL			0x13
#define UVC_CT_REGION_OF_INTEREST_CONTROL	0x14

/* Processing Unit Control Selectors */
#define UVC_PU_CONTROL_UNDEFINED		0x00
#define UVC_PU_BACKLIGHT_COMPENSATION_CONTROL	0x01
#define UVC_PU_BRIGHTNESS_CONTROL		0x02
#define UVC_PU_CONTRAST_CONTROL			0x03
#define UVC_PU_GAIN_CONTROL			0x04
#define UVC_PU_POWER_LINE_FREQUENCY_CONTROL	0x05
#define UVC_PU_HUE_CONTROL			0x06
#define UVC_PU_SATURATION_CONTROL		0x07
#define UVC_PU_SHARPNESS_CONTROL		0x08
#define UVC_PU_GAMMA_CONTROL			0x09
#define UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL 0x0A
#define UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL 0x0B
#define UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL	0x0C
#define UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL 0x0D
#define UVC_PU_DIGITAL_MULTIPLIER_CONTROL	0x0E
#define UVC_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL	0x0F
#define UVC_PU_HUE_AUTO_CONTROL			0x10
#define UVC_PU_ANALOG_VIDEO_STANDARD_CONTROL	0x11
#define UVC_PU_ANALOG_LOCK_STATUS_CONTROL	0x12
#define UVC_PU_CONTRAST_AUTO_CONTROL		0x13

/* Encoding Unit Control Selectors */
#define UVC_EU_CONTROL_UNDEFINED		0x00
#define UVC_EU_SELECT_LAYER_CONTROL		0x01
#define UVC_EU_PROFILE_TOOLSET_CONTROL		0x02
#define UVC_EU_VIDEO_RESOLUTION_CONTROL		0x03
#define UVC_EU_MIN_FRAME_INTERVAL_CONTROL	0x04
#define UVC_EU_SLICE_MODE_CONTROL		0x05
#define UVC_EU_RATE_CONTROL_MODE_CONTROL	0x06
#define UVC_EU_AVERAGE_BITRATE_CONTROL		0x07
#define UVC_EU_CPB_SIZE_CONTROL			0x08
#define UVC_EU_PEAK_BIT_RATE_CONTROL		0x09
#define UVC_EU_QUANTIZATION_PARAMS_CONTROL	0x0A
#define UVC_EU_SYNC_REF_FRAME_CONTROL		0x0B
#define UVC_EU_LTR_BUFFER_			0x0C
#define UVC_EU_LTR_PICTURE_CONTROL		0x0D
#define UVC_EU_LTR_VALIDATION_CONTROL		0x0E
#define UVC_EU_LEVEL_IDC_LIMIT_CONTROL		0x0F
#define UVC_EU_SEI_PAYLOADTYPE_CONTROL		0x10
#define UVC_EU_QP_RANGE_CONTROL			0x11
#define UVC_EU_PRIORITY_CONTROL			0x12
#define UVC_EU_START_OR_STOP_LAYER_CONTROL	0x13
#define UVC_EU_ERROR_RESILIENCY_CONTROL		0x14

/* Extension Unit Control Selectors */
#define UVC_XU_CONTROL_UNDEFINED		0x00

/* VideoStreaming Interface Control Selectors */
#define UVC_VS_CONTROL_UNDEFINED		0x00
#define UVC_VS_PROBE_CONTROL			0x01
#define UVC_VS_COMMIT_CONTROL			0x02
#define UVC_VS_STILL_PROBE_CONTROL		0x03
#define UVC_VS_STILL_COMMIT_CONTROL		0x04
#define UVC_VS_STILL_IMAGE_TRIGGER_CONTROL	0x05
#define UVC_VS_STREAM_ERROR_CODE_CONTROL	0x06
#define UVC_VS_GENERATE_KEY_FRAME_CONTROL	0x07
#define UVC_VS_UPDATE_FRAME_SEGMENT_CONTROL	0x08
#define UVC_VS_SYNCH_DELAY_CONTROL		0x09

/* USB Terminal Types */
#define UVC_TT_VENDOR_SPECIFIC			0x0100
#define UVC_TT_STREAMING			0x0101

/* Input Terminal Types */
#define UVC_ITT_VENDOR_SPECIFIC			0x0200
#define UVC_ITT_CAMERA				0x0201
#define UVC_ITT_MEDIA_TRANSPORT_INPUT		0x0202

/* Output Terminal Types */
#define UVC_OTT_VENDOR_SPECIFIC			0x0300
#define UVC_OTT_DISPLAY				0x0301
#define UVC_OTT_MEDIA_TRANSPORT_OUTPUT		0x0302

/* External Terminal Types */
#define UVC_EXTERNAL_VENDOR_SPECIFIC		0x0400
#define UVC_COMPOSITE_CONNECTOR			0x0401
#define UVC_SVIDEO_CONNECTOR			0x0402
#define UVC_COMPONENT_CONNECTOR			0x0403

#define UVC_MAX_VIDEO_FRAME_BUFFER_SIZE(node)					\
	(DT_PROP_BY_IDX(n, size, 0) * DT_PROP_BY_IDX(n, size, 1) *		\
	 DT_PROP(DT_PARENT(n), bits_per_pixel) / 8 +				\
	 CONFIG_USBD_VIDEO_HEADER_SIZE)

/* 3.6 Interface Association Descriptor */
#define UVC_INTERFACE_ASSOCIATION_DESCRIPTOR(node)				\
	0x08,						/* bLength */		\
	USB_DESC_INTERFACE_ASSOC,			/* bDescriptorType */	\
	0x00,						/* bFirstInterface */	\
	UVC_NUM_INTERFACES(node),			/* bInterfaceCount */	\
	USB_BCC_VIDEO,					/* bFunctionClass */	\
	UVC_SC_VIDEO_INTERFACE_COLLECITON,		/* bFunctionSubClass */	\
	UVC_PC_PROTOCOL_UNDEFINED,			/* bFunctionProtocol */	\
	0x00,						/* iFunction */

/* 3.7 VideoControl Interface Descriptors */
#define UVC_VIDEO_CONTROL_INTERFACE_DESCRIPTOR(node)				\
	0x09,						/* bLength */		\
	USB_DESC_INTERFACE,				/* bDescriptorType */	\
	0x00,						/* bInterfaceNumber */	\
	0x00,						/* bAlternateSetting */	\
	0x00,						/* bNumEndpoints */	\
	USB_BCC_VIDEO,					/* bInterfaceClass */	\
	UVC_SC_VIDEOCONTROL,				/* bInterfaceSubClass */\
	0x00,						/* bInterfaceProtocol */\
	0x00,						/* iInterface */

/* 3.7.2 Interface Header Descriptor */
#define UVC_INTERFACE_HEADER_DESCRIPTOR(node)					\
	0x0c + n,					/* bLength */		\
	UVC_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VC_HEADER,					/* bDescriptorSubtype */\
	0x0150,						/* bcdUVC */		\
	n1 + n2 + n3,					/* wTotalLength */	\
	U32_LE(30000000),				/* dwClockFrequency */	\
	0x01,						/* bInCollection */	\
	0x01,						/* baInterfaceNr */

/* 3.7.2.1 Input Terminal Descriptor */
#define UVC_INPUT_TERMINAL_DESCRIPTOR(node)					\
	0x08 + n,					/* bLength */		\
	UVC_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VC_INPUT_TERMINAL,				/* bDescriptorSubtype */\
	UVC_ENTITY_ID(node),				/* bTerminalID */	\
	U16_LE(UVC_ITT_CAMERA),				/* wTerminalType */	\
	0x00,						/* bAssocTerminal */	\
	0x00,						/* iTerminal */		\
	U16_LE(0),					/* wObjectiveFocalLengthMin */\
	U16_LE(0),					/* wObjectiveFocalLengthMax */\
	U16_LE(0),					/* wOcularFocalLength */\
	3,						/* bControlSize */	\
	0x00, 0x00, 0x00,				/* bmControls */

/* 3.7.2.2 Output Terminal Descriptor */
#define UVC_OUTPUT_TERMINAL_DESCRIPTOR(node)					\
	0x09 + n,					/* bLength */		\
	UVC_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VC_OUTPUT_TERMINAL,				/* bDescriptorSubtype */\
	UVC_ENTITY_ID(node),				/* bTerminalID */	\
	U16_LE(UVC_TT_STREAMING),			/* wTerminalType */	\
	0x00,						/* bAssocTerminal */	\
	0x01,						/* bSourceID */		\
	0x00,						/* iTerminal */

/* 3.9 VideoStreaming Interface Descriptors */
#define UVC_VIDEO_STREAMING_INTERFACE_DESCRIPTOR(node)				\
	0x07,						/* bLength */		\
	USB_DESC_INTERFACE,				/* bDescriptorType */	\
	0x01,						/* bInterfaceNumber */	\
	0x00,						/* bAlternateSetting */	\
	0x01,						/* bNumEndpoints */	\
	USB_BCC_VIDEO,					/* bInterfaceClass */	\
	UVC_SC_VIDEOSTREAMING,				/* bInterfaceSubClass */\
	0x00,						/* bInterfaceProtocol */\
	0x00,						/* iInterface */

/* 3.9.2.1 Input Header Descriptor */
#define UVC_STREAM_INPUT_HEADER_DESCRIPTOR(node)				\
	13 + (p * n),					/* bLength */		\
	UVC_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VS_INPUT_HEADER,				/* bDescriptorSubtype */\
	0,						/* bNumFormats */	\
	a + b + c,					/* wTotalLength */	\
	0x81,						/* bEndpointAddress */	\
	0x00,						/* bmInfo */		\
	0x02,						/* bTerminalLink */	\
	0x00,						/* bStillCaptureMethod */\
	0x00,						/* bTriggerSupport */	\
	0x00,						/* bTriggerUsage */	\
	0x01,						/* bControlSize */	\
	0x00,						/* bmaControls */

/* 3.7.2.3 Camera Terminal Descriptor */
#define UVC_CAMERA_TERMINAL_DESCRIPTOR(node)					\
	0x12,						/* bLength */		\
	USB_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VC_INPUT_TERMINAL,				/* bDescriptorSubtype */\
	UVC_ENTITY_ID(node),				/* bTerminalID */	\
	U16_LE(UVC_ITT_CAMERA),				/* wTerminalType */	\
	n,						/* bAssocTerminal */	\
	0x00,						/* iTerminal */		\
	U16_LE(0),					/* wObjectiveFocalLengthMin */\
	U16_LE(0),					/* wObjectiveFocalLengthMax */\
	U16_LE(0),					/* wOcularFocalLength */\
	0x03,						/* bControlSize */	\
	0x00, 0x00, 0x00,				/* bmControls[3] */	\

/* 3.7.2.4 Selector Unit Descriptor */
#define UVC_SELECTOR_UNIT_DESCRIPTOR(node)					\
	0x06 + p,					/* bLength */		\
	USB_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VC_SELECTOR_UNIT,				/* bDescriptorSubtype */\
	UVC_ENTITY_ID(node),				/* bUnitID */		\
	UVC_NR_IN_PINS(node),				/* bNrInPins */		\
	UVC_SOURCE_ID(node),				/* baSourceID */	\
	0x00,						/* iSelector */

/* 3.7.2.5 Processing Unit Descriptor */
#define UVC_PROCESSING_UNIT_DESCRIPTOR(node)					\
	0x0d,						/* bLength */		\
	USB_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VC_PROCESSING_UNIT,				/* bDescriptorSubtype */\
	UVC_ENTITY_ID(node),				/* bUnitID */		\
	UVC_SOURCE_ID(node),				/* bSourceID */		\
	U16_LE(0),					/* wMaxMultiplier */	\
	0x03,						/* bControlSize */	\
	0x00, 0x00, 0x00,				/* bmControls */	\
	0x00,						/* iProcessing */	\
	0x00,						/* bmVideoStandards */

/* 3.7.2.6 Encoding Unit Descriptor */
#define UVC_ENCODING_UNIT_DESCRIPTOR(node)					\
	0x0d,						/* bLength */		\
	USB_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VC_ENCODING_UNIT,				/* bDescriptorSubtype */\
	UVC_ENTITY_ID(node),				/* bUnitID */		\
	UVC_SOURCE_ID(node),				/* bSourceID */		\
	0x00,						/* iEncoding */		\
	0x03,						/* bControlSize */	\
	0x00, 0x00, 0x00,				/* bmControls */	\
	0x00, 0x00, 0x00,				/* bmControlsRuntime */

/* 3.7.2.7 Extension Unit Descriptor */
#define UVC_EXTENSION_UNIT_DESCRIPTOR(node, guid)				\
	24 + UVC_NR_IN_PINS(node) + UVC_NUM_CTRL(node),	/* bLength */		\
	USB_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VC_EXTENSION_UNIT,				/* bDescriptorSubtype */\
	UVC_UNIT_ID(node),				/* bUnitID */		\
	UVC_EXTENSION_GUID(node),			/* guidExtensionCode */	\
	UVC_NUM_CTRL(node),				/* bNumControls */	\
	UVC_NR_IN_PINS(node),				/* bNrInPins */		\
	UVC_SOURCE_ID(node),				/* baSourceID[1] */	\
	UVC_CTRL_SIZE(node),				/* bControlSize */	\
	UVC_CTRL_LIST(node)				/* bmControls */	\
	0x00,						/* iExtension */

/** 3.9.2.2 Output Header Descriptor */
#define UVC_STREAM_OUTPUT_HEADER_DESCRIPTOR(node)				\
	9 + UVC_NUM_CTRL(node) + UVC_NUM_FORMATS(node),	/* bLength */		\
	USB_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VS_OUTPUT_HEADER,				/* bDescriptorSubtype */\
	UVC_NUM_FORMATS(node),				/* bNumFormats */	\
	U16_LE(),					/* wTotalLength */	\
	0x81,						/* bEndpointAddress */	\
	UVC_SOURCE_ID(node),				/* bTerminalLink */	\
	UVC_CTRL_SIZE(node),				/* bControlSize */	\
	UVC_CTRL_LIST(node),				/* bmaControls */

/* 3.10 VideoStreaming Bulk Endpoint Descriptors (FullSpeed) */
#define UVC_VIDEO_STREAMING_BULK_ENDPOINT_DESCRIPTOR_FS(node)			\
	0x08,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	0x81,						/* bEndpointAddress */	\
	USB_EP_TYPE_BULK,				/* bmAttributes */	\
	U16_LE(64),					/* wMaxPacketSize */	\
	0x00,						/* bInterval */

/* 3.10 VideoStreaming Bulk Endpoint Descriptors (HighSpeed) */
#define UVC_VIDEO_STREAMING_BULK_ENDPOINT_DESCRIPTOR_HS(node)			\
	0x08,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	0x81,						/* bEndpointAddress */	\
	USB_EP_TYPE_BULK,				/* bmAttributes */	\
	U16_LE(512),					/* wMaxPacketSize */	\
	0x00,						/* bInterval */

/* 3.10 VideoStreaming Bulk Endpoint Descriptors (SuperSpeed) */
#define UVC_VIDEO_STREAMING_BULK_ENDPOINT_DESCRIPTOR_SS(node)			\
	0x08,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	0x81,						/* bEndpointAddress */	\
	USB_EP_TYPE_BULK,				/* bmAttributes */	\
	U16_LE(1024),					/* wMaxPacketSize */	\
	0x00,						/* bInterval */

/* 3.10 VideoStreaming Bulk Endpoint Descriptors (SuperSpeed Companion) */
#define UVC_VIDEO_STREAMING_BULK_ENDPOINT_DESCRIPTOR_SS_COMP(node)		\
	0x06,						/* bLength */		\
	USB_DESC_ENDPOINT_COMPANION,			/* bDescriptorType */	\
	0x00,						/* bMaxBurst */		\
	0x00,						/* bmAttributes */	\
	U16_LE(0),					/* wBytesPerInterval */

/* USB_Video_Payload_Uncompressed_1.5.pdf */

/* 3.1.1 Uncompressed Video Format Descriptor */
#define UVC_UNCOMPRESSED_FORMAT_DESCRIPTOR(node)				\
	0x1b,						/* bLength */		\
	USB_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VS_FORMAT_UNCOMPRESSED,			/* bDescriptorSubtype */\
	0x00,						/* bFormatIndex */	\
	DT_CHILD_NUM(node),				/* bNumFrameDescriptors */\
	DT_PROP(n, guid),				/* guidFormat */	\
	DT_PROP(node, bits_per_pixel),			/* bBitsPerPixel */	\
	UVC_FRAME_INDEX(node),				/* bDefaultFrameIndex */\
	0x00,						/* bAspectRatioX */	\
	0x00,						/* bAspectRatioY */	\
	0x00,						/* bmInterlaceFlags */	\
	0x00,						/* bCopyProtect */	\

/* 3.2.1 Uncompressed Video Frame Descriptors (discrete) */
#define UVC_UNCOMPRESSED_FRAME_DESCRIPTOR(node)					\
	0x25,						/* bLength */		\
	USB_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VS_FRAME_UNCOMPRESSED,			/* bDescriptorSubtype */\
	DT_CHILD_NUM(node) + 1,				/* bFrameIndex */	\
	0x00,						/* bmCapabilities */	\
	U16_LE(DT_PROP_BY_IDX(node, size, 0)),		/* wWidth */		\
	U16_LE(DT_PROP_BY_IDX(node, size, 1)),		/* wHeight */		\
	U32_LE(15360000),				/* dwMinBitRate */	\
	U32_LE(15360000),				/* dwMaxBitRate */	\
	U32_LE(UVC_MAX_VIDEO_FRAME_BUFFER_SIZE(node)),	/* dwMaxVideoFrameBufferSize */\
	U32_LE(10000000 / DT_PROP(n, max_fps)),		/* dwDefaultFrameInterval */\
	0x01,						/* bFrameIntervalType */\
	U32_LE(10000000 / DT_PROP(n, max_fps)),		/* dwFrameInterval */

/* USB_Video_Payload_JPEG_1.5.pdf */

/* 3.1.1 Motion-JPEG Video Format Descriptor */
#define UVC_MJPEG_FORMAT_DESCRIPTOR(node)					\
	0x0b,						/* bLength */		\
	USB_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VS_FORMAT_MJPEG,				/* bDescriptorSubtype */\
	0x00,						/* bFormatIndex */	\
	DT_CHILD_NUM(node),				/* bNumFrameDescriptors */\
	BIT(0),						/* bmFlags */		\
	0x01,						/* bDefaultFrameIndex */\
	0x00,						/* bAspectRatioX */	\
	0x00,						/* bAspectRatioY */	\
	0x00,						/* bmInterlaceFlags */	\
	0x00,						/* bCopyProtect */

/* 3.2.1 Motion-JPEG Video Frame Descriptors (discrete) */
#define UVC_MJPEG_FRAME_DESCRIPTOR(node)					\
	0x1d,						/* bLength */		\
	UVC_CS_INTERFACE,				/* bDescriptorType */	\
	UVC_VS_FRAME_MJPEG,				/* bDescriptorSubtype */\
	DT_CHILD_NUM(node) + 1,				/* bFrameIndex */	\
	0x00,						/* bmCapabilities */	\
	U16_LE(DT_PROP_BY_IDX(node, 0)),		/* wWidth */		\
	U16_LE(DT_PROP_BY_IDX(node, 1)),		/* wHeight */		\
	U32_LE(15360000),				/* dwMinBitRate */	\
	U32_LE(15360000),				/* dwMaxBitRate */	\
	U32_LE(UVC_MAX_VIDEO_FRAME_BUFFER_SIZE(node)),	/* dwMaxVideoFrameBufferSize */\
	U32_LE(10000000 / DT_PROP(node, max_fps)),	/* dwDefaultFrameInterval */\
	0x01,						/* bFrameIntervalType */\
	U32_LE(10000000 / DT_PROP(node, max_fps)),	/* dwFrameInterval */

#define UVC_DESCRIPTOR_ARRAYS(node)						\
	static uint8_t uvc_desc_##node##_iad[] = {				\
		UVC_INTERFACE_ASSOCIATION_DESCRIPTOR(node)			\
	};									\
	static uint8_t uvc_desc_##node##_if0[] = {				\
		UVC_VIDEO_CONTROL_INTERFACE_DESCRIPTOR(node)			\
	};									\
	static uint8_t uvc_desc_##node##_if0_header[] = {			\
		UVC_INTERFACE_HEADER_DESCRIPTOR(node)				\
	};									\
	static uint8_t uvc_desc_##node##_if0_input[] = {			\
		UVC_INPUT_TERMINAL_DESCRIPTOR(node)				\
	};									\
	static uint8_t uvc_desc_##node##_if0_output[] = {			\
		UVC_OUTPUT_TERMINAL_DESCRIPTOR(node)				\
	};									\
	static uint8_t uvc_desc_##node##_if1[] = {				\
		UVC_VIDEO_STREAMING_INTERFACE_DESCRIPTOR(node)			\
	};									\
	static uint8_t uvc_desc_##node##_if1_header[] = {			\
		UVC_STREAM_INPUT_HEADER_DESCRIPTOR(node)			\
	};									\
	static uint8_t uvc_desc_##node##_ep_fs[] = {				\
		UVC_VIDEO_STREAMING_BULK_ENDPOINT_DESCRIPTOR_FS(node)		\
	};									\
	static uint8_t uvc_desc_##node##_ep_hs[] = {				\
		UVC_VIDEO_STREAMING_BULK_ENDPOINT_DESCRIPTOR_HS(node)		\
	};									\
	static uint8_t uvc_desc_##node##_ep_ss[] = {				\
		UVC_VIDEO_STREAMING_BULK_ENDPOINT_DESCRIPTOR_SS(node)		\
	};									\
	static uint8_t uvc_desc_##node##_ep_ss_comp[] = {			\
		UVC_VIDEO_STREAMING_BULK_ENDPOINT_DESCRIPTOR_SS_COMP(node)	\
	};

#define UVC_DESCRIPTOR_PTRS_FS(node)						\
	(struct usb_desc_header *)uvc_desc_##node##_iad,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0_header,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0_output,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0_input,			\
	(struct usb_desc_header *)uvc_desc_##node##_if1,			\
	(struct usb_desc_header *)uvc_desc_##node##_if1_header,			\
	DT_INST_FOREACH_CHILD(node, UVC_DESCRIPTOR_PTRS)			\
	(struct usb_desc_header *)uvc_desc_##node##_ep_fs,			\
	(struct usb_desc_header *)uvc_desc_##node##_nil_desc,

#define UVC_DESCRIPTOR_PTRS_HS(node)						\
	(struct usb_desc_header *)uvc_desc_##node##_iad,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0_header,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0_output,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0_input,			\
	(struct usb_desc_header *)uvc_desc_##node##_if1,			\
	(struct usb_desc_header *)uvc_desc_##node##_if1_header,			\
	DT_INST_FOREACH_CHILD(node, UVC_DESCRIPTOR_PTRS)			\
	(struct usb_desc_header *)uvc_desc_##node##_ep_hs,			\
	(struct usb_desc_header *)uvc_desc_##node##_nil_desc,

#define UVC_DESCRIPTOR_PTRS_SS(node)						\
	(struct usb_desc_header *)uvc_desc_##node##_iad,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0_header,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0_output,			\
	(struct usb_desc_header *)uvc_desc_##node##_if0_input,			\
	(struct usb_desc_header *)uvc_desc_##node##_if1,			\
	(struct usb_desc_header *)uvc_desc_##node##_if1_header,			\
	DT_INST_FOREACH_CHILD(node, UVC_DESCRIPTOR_PTRS)			\
	(struct usb_desc_header *)uvc_desc_##node##_ep_ss,			\
	(struct usb_desc_header *)uvc_desc_##node##_ep_comp,			\
	(struct usb_desc_header *)uvc_desc_##node##_nil_desc,

#endif /* ZEPHYR_INCLUDE_USBD_UVC_MACROS_H_ */
