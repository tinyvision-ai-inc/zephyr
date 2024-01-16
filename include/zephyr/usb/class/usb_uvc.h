/*
 * Copyright (c) 2024 tinyVision.ai
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Video Class (UVC) public header
 *
 * Header follows the Class Definitions for Video Specification
 * (UVC_1.5_Class_specification.pdf).
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USB_UVC_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USB_UVC_H_

/** Video Interface Class Code */
#define UVC_CC_VIDEO				0x0E

/** Video Interface Subclass Codes */
#define UVC_SC_UNDEFINED			0x00
#define UVC_SC_VIDEOCONTROL			0x01
#define UVC_SC_VIDEOSTREAMING			0x02
#define UVC_SC_VIDEO_INTERFACE_COLLECITON	0x03

/** Video Interface Protocol Codes */
#define UVC_PC_PROTOCOL_UNDEFINED		0x00
#define UVC_PC_PROTOCOL_15			0x01

/** Video Class-Specific Descriptor Types */
#define UVC_CS_UNDEFINED			0x20
#define UVC_CS_DEVICE				0x21
#define UVC_CS_CONFIGURATION			0x22
#define UVC_CS_STRING				0x23
#define UVC_CS_INTERFACE			0x24
#define UVC_CS_ENDPOINT				0x25

/** Video Class-Specific VC Interface Descriptor Subtypes */
#define UVC_VC_DESCRIPTOR_UNDEFINED		0x00
#define UVC_VC_HEADER				0x01
#define UVC_VC_INPUT_TERMINAL			0x02
#define UVC_VC_OUTPUT_TERMINAL			0x03
#define UVC_VC_SELECTOR_UNIT			0x04
#define UVC_VC_PROCESSING_UNIT			0x05
#define UVC_VC_EXTENSION_UNIT			0x06
#define UVC_VC_ENCODING_UNIT			0x07

/** Video Class-Specific VS Interface Descriptor Subtypes */
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

/** Video Class-Specific Endpoint Descriptor Subtypes */
#define UVC_EP_UNDEFINED			0x00
#define UVC_EP_GENERAL				0x01
#define UVC_EP_ENDPOINT				0x02
#define UVC_EP_INTERRUPT			0x03

/** Video Class-Specific Request Codes */
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

/** VideoControl Interface Control Selectors */
#define UVC_VC_CONTROL_UNDEFINED		0x00
#define UVC_VC_VIDEO_POWER_MODE_CONTROL		0x01
#define UVC_VC_REQUEST_ERROR_CODE_CONTROL	0x02

/** Terminal Control Selectors */
#define UVC_TE_CONTROL_UNDEFINED		0x00

/** Selector Unit Control Selectors */
#define UVC_SU_CONTROL_UNDEFINED		0x00
#define UVC_SU_INPUT_SELECT_CONTROL		0x01

/** Camera Terminal Control Selectors */
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

/** Processing Unit Control Selectors */
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
#define UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL0x0A
#define UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL 0x0B
#define UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL	0x0C
#define UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL 0x0D
#define UVC_PU_DIGITAL_MULTIPLIER_CONTROL	0x0E
#define UVC_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL	0x0F
#define UVC_PU_HUE_AUTO_CONTROL			0x10
#define UVC_PU_ANALOG_VIDEO_STANDARD_CONTROL	0x11
#define UVC_PU_ANALOG_LOCK_STATUS_CONTROL	0x12
#define UVC_PU_CONTRAST_AUTO_CONTROL		0x13

/** Encoding Unit Control Selectors */
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
#define UVC_EU_LTR_BUFFER_ CONTROL		0x0C
#define UVC_EU_LTR_PICTURE_CONTROL		0x0D
#define UVC_EU_LTR_VALIDATION_CONTROL		0x0E
#define UVC_EU_LEVEL_IDC_LIMIT_CONTROL		0x0F
#define UVC_EU_SEI_PAYLOADTYPE_CONTROL		0x10
#define UVC_EU_QP_RANGE_CONTROL			0x11
#define UVC_EU_PRIORITY_CONTROL			0x12
#define UVC_EU_START_OR_STOP_LAYER_CONTROL	0x13
#define UVC_EU_ERROR_RESILIENCY_CONTROL		0x14

/** Extension Unit Control Selectors */
#define UVC_XU_CONTROL_UNDEFINED		0x00

/** VideoStreaming Interface Control Selectors */
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

/** USB Terminal Types */
#define UVC_TT_VENDOR_SPECIFIC			0x0100
#define UVC_TT_STREAMING			0x0101

/** Input Terminal Types */
#define UVC_ITT_VENDOR_SPECIFIC			0x0200
#define UVC_ITT_CAMERA				0x0201
#define UVC_ITT_MEDIA_TRANSPORT_INPUT		0x0202

/** Output Terminal Types */
#define UVC_OTT_VENDOR_SPECIFIC			0x0300
#define UVC_OTT_DISPLAY				0x0301
#define UVC_OTT_MEDIA_TRANSPORT_OUTPUT		0x0302

/** External Terminal Types */
#define UVC_EXTERNAL_VENDOR_SPECIFIC		0x0400
#define UVC_COMPOSITE_CONNECTOR			0x0401
#define UVC_SVIDEO_CONNECTOR			0x0402
#define UVC_COMPONENT_CONNECTOR			0x0403

/** Video Control Interface Status Packet Format */
struct uvc_vc_status_packet {
	uint8_t bStatusType;
	uint8_t bOriginator;
	uint8_t bEvent;
	uint8_t bSelector;
	uint8_t bAttribute;
	uint8_t bValue;
};

/** Video Streaming Interface Status Packet Format */
struct uvc_vs_status_packet {
	uint8_t bStatusType;
	uint8_t bOriginator;
	uint8_t bEvent;
	uint8_t bValue;
};

/** Video and Still Image Payload Headers */
struct uvc_payload_header {
	uint8_t bHeaderLength;
	uint8_t bmHeaderInfo;
#define UVC_BMHEADERINFO_FRAMEID		(1 << 0)
#define UVC_BMHEADERINFO_END_OF_FRAME		(1 << 1)
#DEFINE UVC_BMHEADERINFO_HAS_PRESENTATIONTIME	(1 << 2)
#DEFINE UVC_BMHEADERINFO_HAS_SOURCECLOCK	(1 << 3)
#DEFINE UVC_BMHEADERINFO_PAYLOAD_SPECIFIC_BIT	(1 << 4)
#DEFINE UVC_BMHEADERINFO_STILL_IMAGE		(1 << 5)
#DEFINE UVC_BMHEADERINFO_ERROR			(1 << 6)
#DEFINE UVC_BMHEADERINFO_END_OF_HEADER		(1 << 7)
	/* uint32_t dwPresentationTime; (optional) */
	/* uint64_t scrSourceClock; (optional) */
} __packed;

/** Interface Association Descriptor */
struct uvc_interface_association_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bFirstInterface;
	uint8_t bInterfaceCount;
	uint8_t bFunctionClass;
	uint8_t bFunctionSubClass;
	uint8_t bFunctionProtocol;
	uint8_t iFunction;
};

/** Standard VC Interface Descriptor */
struct uvc_vc_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
};

/** Class-Specific VC Interface Descriptor */
struct uvc_vc_class_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubType;
	uint16_t bcdUVC;
	uint16_t wTotalLength;
	uint32_t dwClockFrequency;
	uint8_t bInCollection;
	uint8_t baInterfaceNr_baInterfaceNr[];
};

/** Input Terminal Descriptor */
struct uvc_input_terminal_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalID;
	uint16_t wTerminalType;
	uint8_t bAssocTerminal;
	uint8_t iTerminal;
	uint8_t bExtraFields[];
};

/** Output Terminal Descriptor */
struct uvc_output_terminal_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalID;
	uint16_t wTerminalType;
	uint8_t bAssocTerminal;
	uint8_t bSourceID;
	uint8_t iTerminal;
	uint8_t bExtraFields[];
};

/** Camera Terminal Descriptor */
struct uvc_camerma_terminal_descriptor {
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
};

/** Selector Unit Descriptor */
struct uvc_selector_unit_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bUnitID;
	uint8_t bNrInPins;
	uint8_t baSourceID_iSelector[];
};

/** Processing Unit Descriptor */
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
};

/** Encoding Unit Descriptor */
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
};

/** Extension Unit Descriptor */
struct uvc_extension_unit_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bUnitID;
	uint8_t guidExtensionCode[16];
	uint8_t bNumControls;
	uint8_t bNrInPins;
	uint8_t baSourceID[];
	uint8_t bControlSize;
	uint8_t bmControls;
	uint8_t iExtension;
};

/** Standard VC Interrupt Endpoint Descriptor */
struct uvc_vc_interrupt_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};

/** Class-specific VC Interrupt Endpoint Descriptor */
struct uvc_vc_class_interrupt_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubType;
	uint16_t wMaxTransferSize;
};

/** Standard VS Interface Descriptor */
struct uvc_vs_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
};

/** Class-specific VS Interface Input Header Descriptor */
struct uvc_vs_class_interface_input_header_descriptor {
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
	uint8_t bmaControls[];
};

/** Class-specific VS Interface Output Header Descriptor */
struct uvc_vs_class_interface_output_header_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bNumFormats;
	uint16_t wTotalLength;
	uint8_t bEndpointAddress;
	uint8_t bTerminalLink;
	uint8_t bControlSize;
	uint8_t bmaControls[];
};

/** Still Image Frame Descriptor */
struct uvc_still_image_frame_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bEndpointAddress;
	uint8_t bNumImageSizePatterns;
	struct {
		uint16_t wWidth;
		uint16_t wHeight;
	} wWidth_wHeight[] __packed;
	uint8_t bNumCompression ;
	uint8_t Pattern;
	uint8_t bCompression[];
};

/** Color Matching Descriptor */
struct uvc_color_matching_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bColorPrimaries;
	uint8_t bTransferCharacteristics;
	uint8_t bMatrixCoefficients;
};

/** Standard VS Isochronous Video Data Endpoint Descriptor */
struct uvc_vs_isochronous_video_data_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint15_t wMaxPacketSize;
	uint8_t bInterval;
};

/** Standard VS Bulk Video Data Endpoint Descriptor */
struct uvc_bulk_video_data_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint8_t wMaxPacketSize;
	uint8_t bInterval;
};

/**  Standard VS Bulk Still Image Data Endpoint Descriptor */
struct uvc_bulk_still_image_data_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USB_CDC_H_ */
