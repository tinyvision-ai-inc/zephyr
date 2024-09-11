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

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>

#ifndef ZEPHYR_INCLUDE_USBD_UVC_MACROS_H_
#define ZEPHYR_INCLUDE_USBD_UVC_MACROS_H_

/* Video and Still Image Payload Headers */
#define UVC_BMHEADERINFO_FRAMEID		BIT(0)
#define UVC_BMHEADERINFO_END_OF_FRAME		BIT(1)
#define UVC_BMHEADERINFO_HAS_PRESENTATIONTIME	BIT(2)
#define UVC_BMHEADERINFO_HAS_SOURCECLOCK	BIT(3)
#define UVC_BMHEADERINFO_PAYLOAD_SPECIFIC_BIT	BIT(4)
#define UVC_BMHEADERINFO_STILL_IMAGE		BIT(5)
#define UVC_BMHEADERINFO_ERROR			BIT(6)
#define UVC_BMHEADERINFO_END_OF_HEADER		BIT(7)

/* Video Interface Class Code */
#define UVC_CC_VIDEO				0x0E

/* Video Interface Subclass Codes */
#define SC_UNDEFINED				0x00
#define SC_VIDEOCONTROL				0x01
#define SC_VIDEOSTREAMING			0x02
#define SC_VIDEO_INTERFACE_COLLECITON		0x03

/* Video Interface Protocol Codes */
#define PC_PROTOCOL_UNDEFINED			0x00
#define PC_PROTOCOL_VERSION_1_5			0x01

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

/* VideoControl Interface Selectors */
#define VC_CONTROL_UNDEFINED			0x00
#define VC_VIDEO_POWER_MODE_CONTROL		0x01
#define VC_REQUEST_ERROR_CODE_CONTROL		0x02

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

/* Descriptors Content */

/* Turn larger types into list of bytes */
#define U(value, shift) (((value) >> shift) & 0xFF)
#define U16_LE(n) U(n, 0), U(n, 8)
#define U24_LE(n) U(n, 0), U(n, 8), U(n, 16)
#define U32_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24)
#define U40_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24), U(n, 32)
#define U48_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24), U(n, 32), U(n, 40)
#define U56_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24), U(n, 32), U(n, 40), U(n, 48)
#define U64_LE(n) U(n, 0), U(n, 8), U(n, 16), U(n, 24), U(n, 32), U(n, 40), U(n, 48), U(n, 56)
#define GUID(node) DT_FOREACH_PROP_ELEM_SEP(node, guid, DT_PROP_BY_IDX, (,))

/* Automatically assign Entity IDs based on entities order in devicetree */
#define NODE_ID(entity) UTIL_INC(DT_NODE_CHILD_IDX(entity))

/* Fetch the ID of a child node with the given compat */
#define NODE_IF_COMPAT(node, compat)						\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, compat), (node))
#define LOOKUP_NODE(node, compat)						\
	DT_FOREACH_CHILD_VARGS(node, NODE_IF_COMPAT, compat)
#define LOOKUP_ID(node, compat)							\
	NODE_ID(LOOKUP_NODE(node, compat))

/* Connect the entities to their source(s) */
#define VC_PROP_N_ID(entity, prop, n) NODE_ID(DT_PHANDLE_BY_IDX(entity, prop, n))
#define VC_SOURCE_ID(entity)							\
	DT_FOREACH_PROP_ELEM_SEP(entity, source_entity, VC_PROP_N_ID, (,))
#define VC_SOURCE_NUM(entity) DT_PROP_LEN(entity, source_entity)

/* Convert a list of integers to an (uint64_t) bitmap */
#define VC_CONTROL_BIT(entity, prop, n) BIT(DT_PROP_BY_IDX(entity, prop, n)) |
#define VC_CONTROLS(entity)							\
	(IF_ENABLED(DT_NODE_HAS_PROP(entity, control_ids), (			\
		DT_FOREACH_PROP_ELEM(entity, control_ids, VC_CONTROL_BIT)	\
	)) 0)

/* Estimate the frame buffer size out of other fields */
#define MAX_VIDEO_FRAME_BUFFER_SIZE(frame)					\
	(DT_PROP_BY_IDX(frame, size, 0) * DT_PROP_BY_IDX(frame, size, 1) *	\
	 DT_PROP(DT_PARENT(frame), bits_per_pixel) / 8 +			\
	 CONFIG_USBD_VIDEO_HEADER_SIZE)

/* 3.6 Interface Association Descriptor */
#define UVC_INTERFACE_ASSOCIATION_DESCRIPTOR(node)				\
	8,						/* bLength */		\
	USB_DESC_INTERFACE_ASSOC,			/* bDescriptorType */	\
	0x00,						/* bFirstInterface */	\
	0x02,						/* bInterfaceCount */	\
	USB_BCC_VIDEO,					/* bFunctionClass */	\
	SC_VIDEO_INTERFACE_COLLECITON,			/* bFunctionSubClass */	\
	PC_PROTOCOL_UNDEFINED,				/* bFunctionProtocol */	\
	0x00,						/* iFunction */

/* Video Control Descriptors */

/* 3.7 VideoControl Interface Descriptors */
#define VC_INTERFACE_DESCRIPTOR(node)						\
	9,						/* bLength */		\
	USB_DESC_INTERFACE,				/* bDescriptorType */	\
	0x00,						/* bInterfaceNumber */	\
	0x00,						/* bAlternateSetting */	\
	0x00,						/* bNumEndpoints */	\
	USB_BCC_VIDEO,					/* bInterfaceClass */	\
	SC_VIDEOCONTROL,				/* bInterfaceSubClass */\
	0x00,						/* bInterfaceProtocol */\
	0x00,						/* iInterface */

/* 3.7.2 Interface Header Descriptor */
#define VC_DESCRIPTORS(node) DT_FOREACH_CHILD(node, VC_DESCRIPTOR)
#define VC_TOTAL_LENGTH(node) sizeof((uint8_t []){VC_DESCRIPTORS(node)})
#define VC_INTERFACE_HEADER_DESCRIPTOR(node)					\
	12 + 1,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_HEADER,					/* bDescriptorSubtype */\
	U16_LE(0x0150),					/* bcdUVC */		\
	U16_LE(VC_TOTAL_LENGTH(node)),			/* wTotalLength */	\
	U32_LE(30000000),				/* dwClockFrequency */	\
	0x01,						/* bInCollection */	\
	0x01,						/* baInterfaceNr */

/* 3.7.2.1 Input Terminal Descriptor */
#define VC_INPUT_TERMINAL_DESCRIPTOR(entity)					\
	8,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_INPUT_TERMINAL,				/* bDescriptorSubtype */\
	NODE_ID(entity),				/* bTerminalID */	\
	U16_LE(ITT_VENDOR_SPECIFIC),			/* wTerminalType */	\
	0x00,						/* bAssocTerminal */	\
	0x00,						/* iTerminal */

/* 3.7.2.2 Output Terminal Descriptor */
#define VC_OUTPUT_TERMINAL_DESCRIPTOR(entity)					\
	9,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_OUTPUT_TERMINAL,				/* bDescriptorSubtype */\
	NODE_ID(entity),				/* bTerminalID */	\
	U16_LE(TT_STREAMING),				/* wTerminalType */	\
	0x00,						/* bAssocTerminal */	\
	VC_SOURCE_ID(entity),				/* bSourceID */		\
	0x00,						/* iTerminal */

/* 3.7.2.3 Camera Terminal Descriptor */
#define VC_CAMERA_TERMINAL_DESCRIPTOR(entity)					\
	18,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_INPUT_TERMINAL,				/* bDescriptorSubtype */\
	NODE_ID(entity),				/* bTerminalID */	\
	U16_LE(ITT_CAMERA),				/* wTerminalType */	\
	0x00,						/* bAssocTerminal */	\
	0x00,						/* iTerminal */		\
	U16_LE(0),					/* wObjectiveFocalLengthMin */\
	U16_LE(0),					/* wObjectiveFocalLengthMax */\
	U16_LE(0),					/* wOcularFocalLength */\
	0x03,						/* bControlSize */	\
	U24_LE(VC_CONTROLS(entity)),			/* bmControls */

/* 3.7.2.4 Selector Unit Descriptor */
#define VC_SELECTOR_UNIT_DESCRIPTOR(entity)					\
	6 + VC_SOURCE_NUM(entity),			/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_SELECTOR_UNIT,				/* bDescriptorSubtype */\
	NODE_ID(entity),				/* bUnitID */		\
	VC_SOURCE_NUM(entity),				/* bNrInPins */		\
	VC_SOURCE_ID(entity),				/* baSourceID */	\
	0x00,						/* iSelector */

/* 3.7.2.5 Processing Unit Descriptor */
#define VC_PROCESSING_UNIT_DESCRIPTOR(entity)					\
	13,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_PROCESSING_UNIT,				/* bDescriptorSubtype */\
	NODE_ID(entity),				/* bUnitID */		\
	VC_SOURCE_ID(entity),				/* bSourceID */		\
	U16_LE(0),					/* wMaxMultiplier */	\
	0x03,						/* bControlSize */	\
	U24_LE(VC_CONTROLS(entity)),			/* bmControls */	\
	0x00,						/* iProcessing */	\
	0x00,						/* bmVideoStandards */

/* 3.7.2.6 Encoding Unit Descriptor */
#define VC_ENCODING_UNIT_DESCRIPTOR(entity)					\
	13,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_ENCODING_UNIT,				/* bDescriptorSubtype */\
	NODE_ID(entity),				/* bUnitID */		\
	VC_SOURCE_ID(entity),				/* bSourceID */		\
	0x00,						/* iEncoding */		\
	0x03,						/* bControlSize */	\
	U48_LE(VC_CONTROLS(entity)),			/* bmControls+Runtime */

/* 3.7.2.7 Extension Unit Descriptor */
#define VC_EXTENSION_UNIT_DESCRIPTOR(entity)					\
	32x + VC_SOURCE_NUM(entity),			/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VC_EXTENSION_UNIT,				/* bDescriptorSubtype */\
	NODE_ID(entity),				/* bUnitID */		\
	GUID(entity),					/* guidExtensionCode */	\
	VC_NUM_CTRL(entity),				/* bNumControls */	\
	VC_SOURCE_NUM(entity),				/* bNrInPins */		\
	VC_SOURCE_ID(entity)				/* baSourceID */	\
	0x08,						/* bControlSize */	\
	U64_LE(VC_CONTROLS(entity)),			/* bmControls */	\
	0x00,						/* iExtension */

/* Video Control descriptor content of a node according to its type */
#define VC_DESCRIPTOR(entity)							\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uvc_control_input), (	\
		VC_INPUT_TERMINAL_DESCRIPTOR(entity)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uvc_control_camera), (	\
		VC_CAMERA_TERMINAL_DESCRIPTOR(entity)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uvc_control_output), (	\
		VC_OUTPUT_TERMINAL_DESCRIPTOR(entity)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uvc_control_selector), (	\
		VC_SELECTOR_UNIT_DESCRIPTOR(entity)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uvc_control_processing), (	\
		VC_PROCESSING_UNIT_DESCRIPTOR(entity)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uvc_control_encoding), (	\
		VC_ENCODING_UNIT_DESCRIPTOR(entity)				\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(entity, zephyr_uvc_control_extension), (	\
		VC_EXTENSION_UNIT_DESCRIPTOR(entity)				\
	))

/* Video Streaming Descriptors */

/* 3.9 VideoStreaming Interface Descriptors */
#define VS_INTERFACE_DESCRIPTOR(node)						\
	9,						/* bLength */		\
	USB_DESC_INTERFACE,				/* bDescriptorType */	\
	0x01,						/* bInterfaceNumber */	\
	0x00,						/* bAlternateSetting */	\
	0x01,						/* bNumEndpoints */	\
	USB_BCC_VIDEO,					/* bInterfaceClass */	\
	SC_VIDEOSTREAMING,				/* bInterfaceSubClass */\
	0x00,						/* bInterfaceProtocol */\
	0x00,						/* iInterface */

/* 3.9.2.1 Input Header Descriptor */
#define VS_CONTROL(format) IF_DISABLED(IS_EMPTY(VS_DESCRIPTOR(format)), (0x00,))
#define VS_CONTROLS(node) DT_FOREACH_CHILD(node, VS_CONTROL)
#define VS_NUM_FORMATS(node) sizeof((uint8_t []){VS_CONTROLS(node)})
#define VS_DESCRIPTORS(node) DT_FOREACH_CHILD_SEP(node, VS_DESCRIPTOR, ())
#define VS_TOTAL_LENGTH(node) sizeof((uint8_t []){VS_DESCRIPTORS(node)})
#define VS_INPUT_HEADER_DESCRIPTOR(node)					\
	14,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_INPUT_HEADER,				/* bDescriptorSubtype */\
	VS_NUM_FORMATS(node),				/* bNumFormats */	\
	U16_LE(VS_TOTAL_LENGTH(node)),			/* wTotalLength */	\
	0x81,						/* bEndpointAddress */	\
	0x00,						/* bmInfo */		\
	LOOKUP_ID(node, zephyr_uvc_control_output),	/* bTerminalLink */	\
	0x00,						/* bStillCaptureMethod */\
	0x00,						/* bTriggerSupport */	\
	0x00,						/* bTriggerUsage */	\
	0x00,						/* bControlSize */	\

/* 3.10 VideoStreaming Bulk FullSpeed Endpoint Descriptors */
#define VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR(node)				\
	7,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	0x81,						/* bEndpointAddress */	\
	USB_EP_TYPE_BULK,				/* bmAttributes */	\
	U16_LE(64),					/* wMaxPacketSize */	\
	0x00,						/* bInterval */

/* 3.10 VideoStreaming Bulk HighSpeed Endpoint Descriptors */
#define VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR(node)				\
	7,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	0x81,						/* bEndpointAddress */	\
	USB_EP_TYPE_BULK,				/* bmAttributes */	\
	U16_LE(512),					/* wMaxPacketSize */	\
	0x00,						/* bInterval */

/* 3.10 VideoStreaming Bulk SuperSpeed Endpoint Descriptors */
#define VS_SUPERSPEED_BULK_ENDPOINT_DESCRIPTOR(node)				\
	7,						/* bLength */		\
	USB_DESC_ENDPOINT,				/* bDescriptorType */	\
	0x81,						/* bEndpointAddress */	\
	USB_EP_TYPE_BULK,				/* bmAttributes */	\
	U16_LE(1024),					/* wMaxPacketSize */	\
	0x00,						/* bInterval */

/* 3.10 VideoStreaming Bulk SuperSpeed Endpoint Companion Descriptors */
#define VS_SUPERSPEED_BULK_ENDPOINT_COMPANION_DESCRIPTOR(node)			\
	6,						/* bLength */		\
	USB_DESC_ENDPOINT_COMPANION,			/* bDescriptorType */	\
	0x00,						/* bMaxBurst */		\
	0x00,						/* bmAttributes */	\
	U16_LE(0),					/* wBytesPerInterval */

/* USB_Video_Payload_Uncompressed_1.5.pdf */

/* 3.1.1 Uncompressed Video Format Descriptor */
#define VS_UNCOMPRESSED_FORMAT_DESCRIPTOR(format)				\
	27,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_FORMAT_UNCOMPRESSED,				/* bDescriptorSubtype */\
	NODE_ID(format),				/* bFormatIndex */	\
	DT_CHILD_NUM(format),				/* bNumFrameDescriptors */\
	GUID(format),					/* guidFormat */	\
	DT_PROP(format, bits_per_pixel),		/* bBitsPerPixel */	\
	0x01,						/* bDefaultFrameIndex */\
	0x00,						/* bAspectRatioX */	\
	0x00,						/* bAspectRatioY */	\
	0x00,						/* bmInterlaceFlags */	\
	0x00,						/* bCopyProtect */

/* 3.2.1 Uncompressed Video Frame Descriptors (discrete) */
#define VS_UNCOMPRESSED_FRAME_DESCRIPTOR(frame)					\
	26 + 4,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_FRAME_UNCOMPRESSED,				/* bDescriptorSubtype */\
	NODE_ID(frame),					/* bFrameIndex */	\
	0x00,						/* bmCapabilities */	\
	U16_LE(DT_PROP_BY_IDX(frame, size, 0)),		/* wWidth */		\
	U16_LE(DT_PROP_BY_IDX(frame, size, 1)),		/* wHeight */		\
	U32_LE(15360000),				/* dwMinBitRate */	\
	U32_LE(15360000),				/* dwMaxBitRate */	\
	U32_LE(MAX_VIDEO_FRAME_BUFFER_SIZE(frame)),	/* dwMaxVideoFrameBufferSize */\
	U32_LE(10000000 / DT_PROP(frame, max_fps)),	/* dwDefaultFrameInterval */\
	0x01,						/* bFrameIntervalType */\
	U32_LE(10000000 / DT_PROP(frame, max_fps)),	/* dwFrameInterval */

/* USB_Video_Payload_JPEG_1.5.pdf */

/* 3.1.1 Motion-JPEG Video Format Descriptor */
#define VS_MJPEG_FORMAT_DESCRIPTOR(format)					\
	11,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_FORMAT_MJPEG,				/* bDescriptorSubtype */\
	NODE_ID(format),				/* bFormatIndex */	\
	DT_CHILD_NUM(format),				/* bNumFrameDescriptors */\
	BIT(0),						/* bmFlags */		\
	0x01,						/* bDefaultFrameIndex */\
	0x00,						/* bAspectRatioX */	\
	0x00,						/* bAspectRatioY */	\
	0x00,						/* bmInterlaceFlags */	\
	0x00,						/* bCopyProtect */

/* 3.2.1 Motion-JPEG Video Frame Descriptors (discrete) */
#define VS_MJPEG_FRAME_DESCRIPTOR(frame)					\
	29,						/* bLength */		\
	USB_DESC_CS_INTERFACE,				/* bDescriptorType */	\
	VS_FRAME_MJPEG,					/* bDescriptorSubtype */\
	NODE_ID(frame),					/* bFrameIndex */	\
	0x00,						/* bmCapabilities */	\
	U16_LE(DT_PROP_BY_IDX(frame, size, 0)),		/* wWidth */		\
	U16_LE(DT_PROP_BY_IDX(frame, size, 1)),		/* wHeight */		\
	U32_LE(15360000),				/* dwMinBitRate */	\
	U32_LE(15360000),				/* dwMaxBitRate */	\
	U32_LE(MAX_VIDEO_FRAME_BUFFER_SIZE(frame)),	/* dwMaxVideoFrameBufferSize */\
	U32_LE(10000000 / DT_PROP(frame, max_fps)),	/* dwDefaultFrameInterval */\
	0x01,						/* bFrameIntervalType */\
	U32_LE(10000000 / DT_PROP(frame, max_fps)),	/* dwFrameInterval */

/* Video Streaming descriptor content of a node according to its type */
#define VS_DESCRIPTOR(format)							\
	IF_ENABLED(DT_NODE_HAS_COMPAT(format, zephyr_uvc_format_mjpeg), (	\
		VS_MJPEG_FORMAT_DESCRIPTOR(format)				\
		DT_FOREACH_CHILD(format, VS_MJPEG_FRAME_DESCRIPTOR)		\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(format, zephyr_uvc_format_uncompressed), (\
		VS_UNCOMPRESSED_FORMAT_DESCRIPTOR(format)			\
		DT_FOREACH_CHILD(format, VS_UNCOMPRESSED_FRAME_DESCRIPTOR)	\
	))

/* Descriptor Arrays */

#define VS_MJPEG_FRAME_DESCRIPTOR_ARRAY(frame)					\
	static uint8_t frame##_desc[] = {					\
		VS_MJPEG_FRAME_DESCRIPTOR(frame)				\
	};

#define VS_UNCOMPRESSED_FRAME_DESCRIPTOR_ARRAY(frame)				\
	static uint8_t frame##_desc[] = {					\
		VS_UNCOMPRESSED_FRAME_DESCRIPTOR(frame)				\
	};

#define INTERFACE_DESCRIPTOR_ARRAYS(node)					\
	static uint8_t node##_desc[] = {					\
		VC_DESCRIPTOR(node)						\
		VS_DESCRIPTOR(node)						\
	};									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uvc_format_mjpeg), (		\
		DT_FOREACH_CHILD(node, VS_MJPEG_FRAME_DESCRIPTOR_ARRAY)		\
	))									\
	IF_ENABLED(DT_NODE_HAS_COMPAT(node, zephyr_uvc_format_uncompressed), (	\
		DT_FOREACH_CHILD(node, VS_UNCOMPRESSED_FRAME_DESCRIPTOR_ARRAY)	\
	))

#define UVC_DESCRIPTOR_ARRAYS(node)						\
	static uint8_t node##_desc_iad[] = {					\
		UVC_INTERFACE_ASSOCIATION_DESCRIPTOR(node)			\
	};									\
	static uint8_t node##_desc_if_vc[] = {					\
		VC_INTERFACE_DESCRIPTOR(node)					\
	};									\
	static uint8_t node##_desc_if_vc_header[] = {				\
		VC_INTERFACE_HEADER_DESCRIPTOR(node)				\
	};									\
	static uint8_t node##_desc_if_vs[] = {					\
		VS_INTERFACE_DESCRIPTOR(node)					\
	};									\
	static uint8_t node##_desc_if_vs_header[] = {				\
		VS_INPUT_HEADER_DESCRIPTOR(node)				\
	};									\
	static uint8_t node##_fs_desc_ep[] = {					\
		VS_FULLSPEED_BULK_ENDPOINT_DESCRIPTOR(node)			\
	};									\
	static uint8_t node##_hs_desc_ep[] = {					\
		VS_HIGHSPEED_BULK_ENDPOINT_DESCRIPTOR(node)			\
	};									\
	static uint8_t node##_ss_desc_ep[] = {					\
		VS_SUPERSPEED_BULK_ENDPOINT_DESCRIPTOR(node)			\
	};									\
	static uint8_t node##_ss_desc_ep_comp[] = {				\
		VS_SUPERSPEED_BULK_ENDPOINT_COMPANION_DESCRIPTOR(node)		\
	};									\
	DT_FOREACH_CHILD_SEP(node, INTERFACE_DESCRIPTOR_ARRAYS, ())

/* Descriptor Pointers */

#define DESCRIPTOR_PTR(node)							\
	((struct usb_desc_header *)&node##_desc),

#define VC_DESCRIPTOR_PTRS(entity)						\
	IF_DISABLED(IS_EMPTY(VC_DESCRIPTOR(entity)), (				\
		(struct usb_desc_header *) &entity##_desc,			\
	))

#define VS_DESCRIPTOR_PTRS(format)						\
	IF_DISABLED(IS_EMPTY(VS_DESCRIPTOR(format)), (				\
		((struct usb_desc_header *)&format##_desc),			\
		DT_FOREACH_CHILD_SEP(format, DESCRIPTOR_PTR, ())		\
	))

#define UVC_DESCRIPTOR_PTRS(node)						\
	(struct usb_desc_header *)node##_desc_iad,				\
	(struct usb_desc_header *)node##_desc_if_vc,				\
	(struct usb_desc_header *)node##_desc_if_vc_header,			\
	DT_FOREACH_CHILD(node, VC_DESCRIPTOR_PTRS)				\
	(struct usb_desc_header *)node##_desc_if_vs,				\
	(struct usb_desc_header *)node##_desc_if_vs_header,			\
	DT_FOREACH_CHILD(node, VS_DESCRIPTOR_PTRS)

#define UVC_FULLSPEED_DESCRIPTOR_PTRS(node)					\
	UVC_DESCRIPTOR_PTRS(node)						\
	(struct usb_desc_header *)node##_fs_desc_ep,				\
	(struct usb_desc_header *)&nil_desc,

#define UVC_HIGHSPEED_DESCRIPTOR_PTRS(node)					\
	UVC_DESCRIPTOR_PTRS(node)						\
	(struct usb_desc_header *)node##_hs_desc_ep,				\
	(struct usb_desc_header *)&nil_desc,

#define UVC_SUPERSPEED_DESCRIPTOR_PTRS(node)					\
	UVC_DESCRIPTOR_PTRS(node)						\
	(struct usb_desc_header *)node##_ss_desc_ep,				\
	(struct usb_desc_header *)node##_ss_desc_ep_comp,			\
	(struct usb_desc_header *)&nil_desc,

static struct usb_desc_header nil_desc;

#endif /* ZEPHYR_INCLUDE_USBD_UVC_MACROS_H_ */
