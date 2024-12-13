/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB Device Firmware Upgrade (DFU) public header
 *
 * Header exposes API for registering DFU images.
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBD_UVC_H
#define ZEPHYR_INCLUDE_USB_CLASS_USBD_UVC_H

#include <zephyr/kernel.h>

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

/* This is a particular variant of this struct that is used by the Zephyr implementation. Other
 * organization of the fields are allowed by the standard.
 */
struct uvc_payload_header {
	uint8_t bHeaderLength;
	uint8_t bmHeaderInfo;
	uint32_t dwPresentationTime; /* optional */
	uint32_t scrSourceClockSTC;  /* optional */
	uint16_t scrSourceClockSOF;  /* optional */
} __packed;

/* Information used for declaring and operating a video stream */
struct uvc_stream_data {
	/* Global state of the UVC video device endpoint */
	atomic_t state;
	/* Input buffers to which enqueued video buffers land */
	struct k_fifo fifo_in;
	/* Output buffers from which dequeued buffers are picked */
	struct k_fifo fifo_out;
	/* VideoControl unit descriptors */
	struct usb_desc_header **desc_vc;
	/* VideoStreaming format and frame descriptors */
	struct usb_desc_header **desc_vs;
	/* Pointer to the endpoint descriptor of this stream */
	struct usb_ep_descriptor *desc_vs_ep_fs;
	struct usb_ep_descriptor *desc_vs_ep_hs;
	/* Default video probe stored at boot time and sent back to the host when requested. */
	struct uvc_probe default_probe;
	/* Video payload header content sent before every frame, updated between every frame */
	struct uvc_payload_header payload_header;
	/* Format currently selected by the host */
	uint8_t format_id;
	/* Frame currently selected by the host */
	uint8_t frame_id;
	/* Video device that is connected to this UVC stream */
	const struct device *video_dev;
	/* Video format cached locally for efficiency */
	struct video_format video_fmt;
	/* Current frame interval selected by the host */
	struct video_frmival video_frmival;
	/* Byte offset within the currently transmitted video buffer */
	size_t vbuf_offset;
	/* Makes sure flushing the stream only happens in one context at a time. */
	struct k_mutex mutex;
	/* Zero Length packet used to reset a stream when restarted */
	struct net_buf zlp;
	/* Signal to alert video devices of buffer-related evenets */
	struct k_poll_signal *signal;
};

/* Instance of a VideoStreaming interface */
struct uvc_stream {
	const struct device *dev;
	/* Name of the stream, matching the string descriptor */
	const char *name;
	/* Pointer to memory for variable stream data */
	struct uvc_stream_data *data;
};

#define USBD_UVC_DEFINE_STREAM(_id, _uvc_dev, _name)				\
	static struct uvc_stream_data uvc_stream_data_##_id;			\
										\
	const static STRUCT_SECTION_ITERABLE(uvc_stream, _id) = {		\
		.dev = _uvc_dev,						\
		.name = _name,							\
		.data = &uvc_stream_data_##_id,					\
	};

static inline void uvc_set_video_dev(const struct uvc_stream *strm, const struct device *dev)
{
	strm->data->video_dev = dev;
}

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBD_UVC_H */
