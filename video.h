/*
 * Copyright (c) 2024 tinyVision.ai Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_USB_VIDEO_H
#define ZEPHYR_INCLUDE_DT_BINDINGS_USB_VIDEO_H

/* UVC GUIDs as defined on Linux source */
#define UVC_GUID_MJPEG          4d 4a 50 47 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_YUY2           59 55 59 32 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_YUY2_ISIGHT    59 55 59 32 00 00 10 00 80 00 00 00 00 38 9b 71
#define UVC_GUID_NV12           4e 56 31 32 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_YV12           59 56 31 32 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_I420           49 34 32 30 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_UYVY           55 59 56 59 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_Y800           59 38 30 30 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_Y8             59 38 20 20 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_Y10            59 31 30 20 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_Y12            59 31 32 20 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_Y16            59 31 36 20 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_BY8            42 59 38 20 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_BA81           42 41 38 31 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_GBRG           47 42 52 47 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_GRBG           47 52 42 47 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_RGGB           52 47 47 42 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_BG16           42 47 31 36 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_GB16           47 42 31 36 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_RG16           52 47 31 36 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_GR16           47 52 31 36 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_RGBP           52 47 42 50 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_BGR3           7d eb 36 e4 4f 52 ce 11 9f 53 00 20 af 0b a7 70
#define UVC_GUID_BGR4           7e eb 36 e4 4f 52 ce 11 9f 53 00 20 af 0b a7 70
#define UVC_GUID_M420           4d 34 32 30 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_H264           48 32 36 34 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_H265           48 32 36 35 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_Y8I            59 38 49 20 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_Y12I           59 31 32 49 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_Z16            5a 31 36 20 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_RW10           52 57 31 30 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_INVZ           49 4e 56 5a 90 2d 58 4a 92 0b 77 3f 1f 2c 55 6b
#define UVC_GUID_INZI           49 4e 5a 49 66 1a 42 a2 90 65 d0 18 14 a8 ef 8a
#define UVC_GUID_INVI           49 4e 56 49 db 57 49 5e 8e 3f f4 79 53 2b 94 6f
#define UVC_GUID_CNF4           43 20 20 20 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_D3DFMT_L8      32 00 00 00 00 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_KSMEDIA_L8_IR  32 00 00 00 02 00 10 00 80 00 00 aa 00 38 9b 71
#define UVC_GUID_HEVC           48 45 56 43 00 00 10 00 80 00 00 aa 00 38 9b 71

/* UVC controls for "zephyr,uvc-control-selector" */
#define UVC_CONTROL_SU_INPUT_SELECT                     UVC_CTRL(0x01, 0)

/* UVC controls for "zephyr,uvc-control-camera" */
#define UVC_CONTROL_CT_SCANNING_MODE                    UVC_CTRL(0x01, 0)
#define UVC_CONTROL_CT_AE_MODE                          UVC_CTRL(0x02, 1)
#define UVC_CONTROL_CT_AE_PRIORITY                      UVC_CTRL(0x03, 2)
#define UVC_CONTROL_CT_EXPOSURE_TIME_ABSOLUTE           UVC_CTRL(0x04, 3)
#define UVC_CONTROL_CT_EXPOSURE_TIME_RELATIVE           UVC_CTRL(0x05, 4)
#define UVC_CONTROL_CT_FOCUS_ABSOLUTE                   UVC_CTRL(0x06, 5)
#define UVC_CONTROL_CT_FOCUS_RELATIVE                   UVC_CTRL(0x07, 6)
#define UVC_CONTROL_CT_FOCUS_AUTO                       UVC_CTRL(0x08, 17)
#define UVC_CONTROL_CT_IRIS_ABSOLUTE                    UVC_CTRL(0x09, 7)
#define UVC_CONTROL_CT_IRIS_RELATIVE                    UVC_CTRL(0x0A, 8)
#define UVC_CONTROL_CT_ZOOM_ABSOLUTE                    UVC_CTRL(0x0B, 9)
#define UVC_CONTROL_CT_ZOOM_RELATIVE                    UVC_CTRL(0x0C, 10)
#define UVC_CONTROL_CT_PANTILT_ABSOLUTE                 UVC_CTRL(0x0D, 11)
#define UVC_CONTROL_CT_PANTILT_RELATIVE                 UVC_CTRL(0x0E, 12)
#define UVC_CONTROL_CT_ROLL_ABSOLUTE                    UVC_CTRL(0x0F, 13)
#define UVC_CONTROL_CT_ROLL_RELATIVE                    UVC_CTRL(0x10, 14)
#define UVC_CONTROL_CT_PRIVACY                          UVC_CTRL(0x11, 18)
#define UVC_CONTROL_CT_FOCUS_SIMPLE                     UVC_CTRL(0x12, 19)
#define UVC_CONTROL_CT_WINDOW                           UVC_CTRL(0x13, 20)
#define UVC_CONTROL_CT_REGION_OF_INTEREST               UVC_CTRL(0x14, 21)

/* UVC controls for "zephyr,uvc-control-processing" */
#define UVC_CONTROL_PU_BACKLIGHT_COMPENSATION           UVC_CTRL(0x01, 8)
#define UVC_CONTROL_PU_BRIGHTNESS                       UVC_CTRL(0x02, 0)
#define UVC_CONTROL_PU_CONTRAST                         UVC_CTRL(0x03, 1)
#define UVC_CONTROL_PU_GAIN                             UVC_CTRL(0x04, 9)
#define UVC_CONTROL_PU_POWER_LINE_FREQUENCY             UVC_CTRL(0x05, 10)
#define UVC_CONTROL_PU_HUE                              UVC_CTRL(0x06, 2)
#define UVC_CONTROL_PU_SATURATION                       UVC_CTRL(0x07, 3)
#define UVC_CONTROL_PU_SHARPNESS                        UVC_CTRL(0x08, 4)
#define UVC_CONTROL_PU_GAMMA                            UVC_CTRL(0x09, 5)
#define UVC_CONTROL_PU_WHITE_BALANCE_TEMPERATURE        UVC_CTRL(0x0A, 6)
#define UVC_CONTROL_PU_WHITE_BALANCE_TEMPERATURE_AUTO   UVC_CTRL(0x0B, 12)
#define UVC_CONTROL_PU_WHITE_BALANCE_COMPONENT          UVC_CTRL(0x0C, 7)
#define UVC_CONTROL_PU_WHITE_BALANCE_COMPONENT_AUTO     UVC_CTRL(0x0D, 13)
#define UVC_CONTROL_PU_DIGITAL_MULTIPLIER               UVC_CTRL(0x0E, 14)
#define UVC_CONTROL_PU_DIGITAL_MULTIPLIER_LIMIT         UVC_CTRL(0x0F, 15)
#define UVC_CONTROL_PU_HUE_AUTO                         UVC_CTRL(0x10, 11)
#define UVC_CONTROL_PU_ANALOG_VIDEO_STANDARD            UVC_CTRL(0x11, 16)
#define UVC_CONTROL_PU_ANALOG_LOCK_STATUS               UVC_CTRL(0x12, 17)
#define UVC_CONTROL_PU_CONTRAST_AUTO                    UVC_CTRL(0x13, 18)

/* UVC controls for "zephyr,uvc-control-encoding" */
#define UVC_CONTROL_EU_SELECT_LAYER                     UVC_CTRL(0x01, 0)
#define UVC_CONTROL_EU_PROFILE_TOOLSET                  UVC_CTRL(0x02, 1)
#define UVC_CONTROL_EU_VIDEO_RESOLUTION                 UVC_CTRL(0x03, 2)
#define UVC_CONTROL_EU_MIN_FRAME_INTERVAL               UVC_CTRL(0x04, 3)
#define UVC_CONTROL_EU_SLICE_MODE                       UVC_CTRL(0x05, 4)
#define UVC_CONTROL_EU_RATE_CONTROL_MODE                UVC_CTRL(0x06, 5)
#define UVC_CONTROL_EU_AVERAGE_BITRATE                  UVC_CTRL(0x07, 6)
#define UVC_CONTROL_EU_CPB_SIZE                         UVC_CTRL(0x08, 7)
#define UVC_CONTROL_EU_PEAK_BIT_RATE                    UVC_CTRL(0x09, 8)
#define UVC_CONTROL_EU_QUANTIZATION_PARAMS              UVC_CTRL(0x0A, 9)
#define UVC_CONTROL_EU_SYNC_REF_FRAME                   UVC_CTRL(0x0B, 10)
#define UVC_CONTROL_EU_LTR_BUFFER                       UVC_CTRL(0x0C, 11)
#define UVC_CONTROL_EU_LTR_PICTURE                      UVC_CTRL(0x0D, 12)
#define UVC_CONTROL_EU_LTR_VALIDATION                   UVC_CTRL(0x0E, 13)
#define UVC_CONTROL_EU_LEVEL_IDC_LIMIT                  UVC_CTRL(0x0F, 14)
#define UVC_CONTROL_EU_SEI_PAYLOADTYPE                  UVC_CTRL(0x10, 15)
#define UVC_CONTROL_EU_QP_RANGE                         UVC_CTRL(0x11, 16)
#define UVC_CONTROL_EU_PRIORITY                         UVC_CTRL(0x12, 17)
#define UVC_CONTROL_EU_START_OR_STOP_LAYER              UVC_CTRL(0x13, 18)
#define UVC_CONTROL_EU_ERROR_RESILIENCY                 UVC_CTRL(0x14, 19)

/* UVC controls for Video Streaming Interface */
#define UVC_CONTROL_VS_PROBE                            UVC_CTRL(0x01, 0)
#define UVC_CONTROL_VS_COMMIT                           UVC_CTRL(0x02, 0)
#define UVC_CONTROL_VS_STILL_PROBE                      UVC_CTRL(0x03, 0)
#define UVC_CONTROL_VS_STILL_COMMIT                     UVC_CTRL(0x04, 0)
#define UVC_CONTROL_VS_STILL_IMAGE_TRIGGER              UVC_CTRL(0x05, 0)
#define UVC_CONTROL_VS_STREAM_ERROR_CODE                UVC_CTRL(0x06, 0)
#define UVC_CONTROL_VS_GENERATE_KEY_FRAME               UVC_CTRL(0x07, 0)
#define UVC_CONTROL_VS_UPDATE_FRAME_SEGMENT             UVC_CTRL(0x08, 0)
#define UVC_CONTROL_VS_SYNCH_DELAY                      UVC_CTRL(0x09, 0)

/* On devicetree, include both values as bits */
#define UVC_CTRL(a, b) BIT(a) BIT(b)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_USB_VIDEO_H */
