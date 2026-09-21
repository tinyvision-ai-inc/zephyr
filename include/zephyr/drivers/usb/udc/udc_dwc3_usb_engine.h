/*
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Track B UsbEngine CSRs at 0xb400B000. See fresh/docs/USB_ENGINE_TRACK_B.md.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC3_USB_ENGINE_H
#define ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC3_USB_ENGINE_H

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_ENGINE_MAGIC		0x5533U
#define USB_ENGINE_VER			0x0001U
#define USB_ENGINE_ID_EXPECT		((USB_ENGINE_VER << 16) | USB_ENGINE_MAGIC)

#define USB_ENGINE_BULK_N		3
#define USB_ENGINE_BULK_WINDOW		0x30U
#define USB_ENGINE_BULK_BASE		0x100U
#define USB_ENGINE_EP_OWN_DEFAULT	0x00020006U
#define USB_ENGINE_VIDEO_ADDR		0x85U
#define USB_ENGINE_TRB_BASE_DEFAULT	0xb1300400U

#define USB_ENGINE_ENG_ID		0x00U
#define USB_ENGINE_ENG_CTRL		0x04U
#define USB_ENGINE_ENG_STATUS		0x08U
#define USB_ENGINE_ENG_IRQ		0x0cU
#define USB_ENGINE_ENG_IRQ_ENA		0x10U
#define USB_ENGINE_VID_CREDIT		0x14U
#define USB_ENGINE_QUIET_OTHER		0x18U
#define USB_ENGINE_EP_OWN		0x1cU
#define USB_ENGINE_EVT_BASE		0x20U
#define USB_ENGINE_EVT_SIZE		0x24U
#define USB_ENGINE_FWD_POP		0x28U
#define USB_ENGINE_PEEK_ADDR		0x2cU
#define USB_ENGINE_PEEK_DATA		0x30U
#define USB_ENGINE_CMD_COUNT		0x34U
#define USB_ENGINE_EVT_COUNT		0x38U
#define USB_ENGINE_VID_DB_COUNT		0x3cU
#define USB_ENGINE_TO_COUNT		0x40U
#define USB_ENGINE_TRACE_W		0x44U
#define USB_ENGINE_TRACE_R		0x48U
#define USB_ENGINE_TRACE_D		0x4cU
#define USB_ENGINE_EP_MAP		0x50U

#define USB_ENGINE_CTRL_ENABLE		BIT(0)
#define USB_ENGINE_CTRL_EVT_OWN		BIT(1)
#define USB_ENGINE_CTRL_VID_EVT		BIT(2)
#define USB_ENGINE_CTRL_CREDIT_EN	BIT(3)
#define USB_ENGINE_CTRL_IRQ_EN		BIT(4)

#define USB_ENGINE_STAT_BUSY		BIT(0)
#define USB_ENGINE_STAT_VID_LIVE	BIT(1)
#define USB_ENGINE_STAT_EVT_OVF		BIT(2)
#define USB_ENGINE_STAT_DESC_OVF	BIT(3)
#define USB_ENGINE_STAT_HALTED		BIT(4)

#define USB_ENGINE_IRQ_FWD		BIT(0)
#define USB_ENGINE_IRQ_CMPL		BIT(1)
#define USB_ENGINE_IRQ_ERR		BIT(2)
#define USB_ENGINE_IRQ_HALTED		BIT(3)

#define USB_ENGINE_DESC_CTRL_IOC	BIT(0)
#define USB_ENGINE_DESC_CTRL_ZLP	BIT(1)
#define USB_ENGINE_DESC_CTRL_SETUP	BIT(2)
#define USB_ENGINE_DESC_CTRL_STATUS	BIT(3)
#define USB_ENGINE_DESC_CTRL_STALL	BIT(4)
#define USB_ENGINE_DESC_CTRL_CLEAR	BIT(5)
#define USB_ENGINE_DESC_CTRL_END	BIT(6)
#define USB_ENGINE_DESC_CTRL_START	BIT(7)

#define USB_ENGINE_EP_DESC_ADDR		0x00U
#define USB_ENGINE_EP_DESC_LEN		0x04U
#define USB_ENGINE_EP_DESC_CTRL		0x08U
#define USB_ENGINE_EP_DESC_FREE		0x0cU
#define USB_ENGINE_EP_CMPL_ADDR		0x10U
#define USB_ENGINE_EP_CMPL_LEN		0x14U
#define USB_ENGINE_EP_CMPL_STAT		0x18U
#define USB_ENGINE_EP_CMPL_LEVEL	0x1cU
#define USB_ENGINE_EP_TRB_BASE		0x20U
#define USB_ENGINE_EP_DEPCMD_ADDR	0x24U
#define USB_ENGINE_EP_XFER_IDX		0x28U
#define USB_ENGINE_EP_STATE		0x2cU

#if DT_HAS_COMPAT_STATUS_OKAY(tinyvision_usb_engine)
#define USB_ENGINE_BASE \
	((mm_reg_t)DT_REG_ADDR(DT_COMPAT_GET_ANY_STATUS_OKAY(tinyvision_usb_engine)))
#else
#define USB_ENGINE_BASE ((mm_reg_t)0xb400b000U)
#endif

static inline int usb_engine_bulk_idx(uint8_t addr)
{
	switch (addr) {
	case 0x81:
		return 0;
	case 0x82:
		return 1;
	case 0x01:
		return 2;
	default:
		return -1;
	}
}

static inline mm_reg_t usb_engine_ep_base(int idx)
{
	return USB_ENGINE_BASE + USB_ENGINE_BULK_BASE +
	       (mm_reg_t)idx * USB_ENGINE_BULK_WINDOW;
}

#if defined(CONFIG_UDC_DWC3_USB_ENGINE)

bool udc_dwc3_engine_present(void);
bool udc_dwc3_engine_enabled(void);
bool udc_dwc3_engine_evt_own(void);
bool udc_dwc3_engine_owns(uint8_t addr);
bool udc_dwc3_engine_posted(uint8_t addr);
void udc_dwc3_engine_dump(void);
bool udc_dwc3_engine_vid_live(void);
uint32_t udc_dwc3_engine_status(void);

void udc_dwc3_engine_program_ep(const struct device *dev, uint8_t addr,
				uint32_t depcmd_addr, uint32_t xfer_idx,
				uint32_t evt_base, uint32_t evt_size);
void udc_dwc3_engine_go(void);
void udc_dwc3_engine_reprime_acm(const struct device *dev);
void udc_dwc3_engine_disable(void);

int udc_dwc3_engine_post(const struct device *dev, uint8_t addr,
			 struct net_buf *buf, uint32_t ctrl);
int udc_dwc3_engine_kick(uint8_t addr, uint32_t trb_addr,
			 uint32_t data_addr, uint32_t len, uint32_t ctrl);
int udc_dwc3_engine_cmd(uint8_t addr, uint32_t ctrl);

void udc_dwc3_engine_poll(const struct device *dev,
			  void (*fwd)(const struct device *dev, uint32_t evt));

#else /* !CONFIG_UDC_DWC3_USB_ENGINE */

static inline bool udc_dwc3_engine_present(void)
{
	return false;
}

static inline bool udc_dwc3_engine_enabled(void)
{
	return false;
}

static inline bool udc_dwc3_engine_evt_own(void)
{
	return false;
}

static inline bool udc_dwc3_engine_owns(uint8_t addr)
{
	ARG_UNUSED(addr);
	return false;
}

static inline bool udc_dwc3_engine_posted(uint8_t addr)
{
	ARG_UNUSED(addr);
	return false;
}

static inline void udc_dwc3_engine_dump(void)
{
}

static inline bool udc_dwc3_engine_vid_live(void)
{
	return false;
}

static inline uint32_t udc_dwc3_engine_status(void)
{
	return 0;
}

static inline void udc_dwc3_engine_program_ep(const struct device *dev, uint8_t addr,
					      uint32_t depcmd_addr, uint32_t xfer_idx,
					      uint32_t evt_base, uint32_t evt_size)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(addr);
	ARG_UNUSED(depcmd_addr);
	ARG_UNUSED(xfer_idx);
	ARG_UNUSED(evt_base);
	ARG_UNUSED(evt_size);
}

static inline void udc_dwc3_engine_go(void)
{
}

static inline void udc_dwc3_engine_reprime_acm(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static inline void udc_dwc3_engine_disable(void)
{
}

static inline int udc_dwc3_engine_post(const struct device *dev, uint8_t addr,
				       struct net_buf *buf, uint32_t ctrl)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(addr);
	ARG_UNUSED(buf);
	ARG_UNUSED(ctrl);
	return -ENOTSUP;
}

static inline int udc_dwc3_engine_kick(uint8_t addr, uint32_t trb_addr,
				       uint32_t data_addr, uint32_t len, uint32_t ctrl)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(trb_addr);
	ARG_UNUSED(data_addr);
	ARG_UNUSED(len);
	ARG_UNUSED(ctrl);
	return -ENOTSUP;
}

static inline int udc_dwc3_engine_cmd(uint8_t addr, uint32_t ctrl)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(ctrl);
	return -ENOTSUP;
}

static inline void udc_dwc3_engine_poll(const struct device *dev,
					void (*fwd)(const struct device *dev, uint32_t evt))
{
	ARG_UNUSED(dev);
	ARG_UNUSED(fwd);
}

#endif /* CONFIG_UDC_DWC3_USB_ENGINE */

/* Always available: keep DWC3 out of U1/U2 during bulk UVC. */
void udc_dwc3_disable_u1u2(const struct device *dev);

/*
 * Windows and Linux ResetPipe the bulk VS endpoint at STREAMON
 * (CLEAR_FEATURE ENDPOINT_HALT). That is not STREAMOFF. Arm a grace
 * window at COMMIT so the first halt keeps RTL ownership.
 */
void udc_dwc3_video_pipe_reset_grace(uint32_t ms);
bool udc_dwc3_video_pipe_reset_pending(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC3_USB_ENGINE_H */
