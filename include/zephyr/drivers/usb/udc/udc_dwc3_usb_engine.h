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
#include <zephyr/sys/printk.h>
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
/* IN bit 2 = 0x82, bit 4 = 0x84. OUT bit 18 = RAW OUT 0x02 (phys 4 >> 1 + 16). */
#define USB_ENGINE_EP_OWN_DEFAULT	0x00040014U
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
/* CPU DEPEVT the snoop missed. Write pulses ownedEvt into BulkRing. */
#define USB_ENGINE_EVT_INJ		0x54U

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
/* Post bit 8. The ring ORs this into TRB dword3 bit 2 (CHN). */
#define USB_ENGINE_DESC_CTRL_CHN	BIT(8)

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

/* After the per-EP windows: EPi_RING_CFG at 0x190 + 8*i, EPi_TRB_CTRL at +4.
 * RING_CFG[7:0] = data slots (0 = fixed TRB at TRB_BASE), [15:8] = slot
 * loaded on write. TRB_CTRL is the dword3 template (reset HWO|CSP|NORMAL|IOC). */
#define USB_ENGINE_RING_CFG_BASE	0x190U
#define USB_ENGINE_RING_CFG(i)		(USB_ENGINE_RING_CFG_BASE + 8U * (uint32_t)(i))
#define USB_ENGINE_RING_TRB_CTRL(i)	(USB_ENGINE_RING_CFG(i) + 4U)

/* EP_STATE bits */
#define USB_ENGINE_EP_STATE_PENDING	BIT(0)	/* descriptor queued or active */
#define USB_ENGINE_EP_STATE_ACTIVE	BIT(1)
#define USB_ENGINE_EP_STATE_STALLED	BIT(2)
#define USB_ENGINE_EP_STATE_RESTARTED	BIT(3)

/* CMPL_STAT low half when the ring aborted the descriptor (ENABLE fell). */
#define USB_ENGINE_CMPL_STAT_ABORT	0xff99U

#if DT_HAS_COMPAT_STATUS_OKAY(tinyvision_usb_engine)
#define USB_ENGINE_BASE \
	((mm_reg_t)DT_REG_ADDR(DT_COMPAT_GET_ANY_STATUS_OKAY(tinyvision_usb_engine)))
#else
#define USB_ENGINE_BASE ((mm_reg_t)0xb400b000U)
#endif

static inline int usb_engine_bulk_idx(uint8_t addr)
{
	switch (addr) {
	case 0x84:
		return 0;
	case 0x82:
		return 1;
	case 0x02:
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

/*
 * Single-master ACM IN: the BulkRing writes the TRB into the CPU's own
 * link-TRB ring and rings UpdateXfer with the CPU's xferrscidx. The CPU
 * keeps ring bookkeeping and the DEPEVT completion path.
 *
 * ring_sync points the ring at trb_base with ring_n data slots and loads
 * the write slot. Refused (-EBUSY) while a descriptor is queued or active.
 * post_slot queues one buffer; -EBUSY when the descriptor FIFO is full.
 * ring_synced reports whether the ring slot still tracks the CPU head;
 * ring_unsync marks a CPU-side TRB write that broke that tracking.
 */
int udc_dwc3_engine_ring_sync(uint8_t addr, uint32_t trb_base, uint8_t ring_n,
			      uint8_t slot);
bool udc_dwc3_engine_ring_synced(uint8_t addr);
void udc_dwc3_engine_ring_unsync(uint8_t addr);
int udc_dwc3_engine_post_slot(uint8_t addr, struct net_buf *buf, uint32_t post_ctrl);
uint32_t udc_dwc3_engine_kicked(uint8_t addr);

/* udc_dwc3.c: a posted slot the ring aborted (ENABLE fell) must be armed
 * again from the CPU TRB path. data_addr is the buffer the ring reported. */
void udc_dwc3_engine_slot_aborted(const struct device *dev, uint8_t addr,
				  uint32_t data_addr);
/* Normal completion of a CPU-ring slot. Retire the net_buf whose data
 * pointer is data_addr when it is still the tail. A DEPEVT that already
 * popped it is a miss. */
void udc_dwc3_engine_slot_done(const struct device *dev, uint8_t addr,
			       uint32_t data_addr, uint32_t len);

void udc_dwc3_engine_poll(const struct device *dev,
			  void (*fwd)(const struct device *dev, uint32_t evt));

/* One-shot IN restart. arm() makes the next completion FIFO pop drop
 * the buffer instead of retiring it (EndXfer looks like a packet).
 * ate() is that pop. disarm() stops the drop. resync() reloads the
 * write slot and clears the ring resource so the next kick is a
 * StartXfer; refused while the ring is still active. */
void udc_dwc3_engine_restart_arm(uint8_t addr);
void udc_dwc3_engine_restart_disarm(uint8_t addr);
bool udc_dwc3_engine_restart_ate(uint8_t addr);
int udc_dwc3_engine_restart_resync(const struct device *dev, uint8_t addr,
				   uint32_t trb_base, uint8_t ring_n,
				   uint8_t slot);

#else /* !CONFIG_UDC_DWC3_USB_ENGINE */

static inline int udc_dwc3_engine_ring_sync(uint8_t addr, uint32_t trb_base,
					    uint8_t ring_n, uint8_t slot)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(trb_base);
	ARG_UNUSED(ring_n);
	ARG_UNUSED(slot);
	return -ENOTSUP;
}

static inline bool udc_dwc3_engine_ring_synced(uint8_t addr)
{
	ARG_UNUSED(addr);
	return false;
}

static inline void udc_dwc3_engine_ring_unsync(uint8_t addr)
{
	ARG_UNUSED(addr);
}

static inline int udc_dwc3_engine_post_slot(uint8_t addr, struct net_buf *buf,
					    uint32_t post_ctrl)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(buf);
	ARG_UNUSED(post_ctrl);
	return -ENOTSUP;
}

static inline uint32_t udc_dwc3_engine_kicked(uint8_t addr)
{
	ARG_UNUSED(addr);
	return 0;
}

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

static inline void udc_dwc3_engine_restart_arm(uint8_t addr)
{
	ARG_UNUSED(addr);
}

static inline void udc_dwc3_engine_restart_disarm(uint8_t addr)
{
	ARG_UNUSED(addr);
}

static inline bool udc_dwc3_engine_restart_ate(uint8_t addr)
{
	ARG_UNUSED(addr);
	return false;
}

static inline int udc_dwc3_engine_restart_resync(const struct device *dev,
						 uint8_t addr, uint32_t trb_base,
						 uint8_t ring_n, uint8_t slot)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(addr);
	ARG_UNUSED(trb_base);
	ARG_UNUSED(ring_n);
	ARG_UNUSED(slot);
	return -ENOTSUP;
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

/*
 * Runtime console-print gate for the UDC. The console is a polled 115200
 * UART, so every printk blocks its caller (~87 us per character). The
 * per-event traces (ACM01/ACM82/SETUP/PARK0/IN-*) run in the DWC3 event
 * thread and the 10 s summaries (RATEPROBE/DWC3HEALTH/ACM ring) in the
 * health timer; both are off unless a bit is set here. Faults (DEPCMD
 * timeouts, event overflow, StartXfer failures, the IN-HOLD drop and
 * its PARK0 snapshot) and the one-shot init / STREAMON lines are not
 * gated: they fire once per fault and are the only in-the-moment
 * evidence in a default-quiet build.
 *
 *   bit0 UDC_DWC3_DBG_TRACE  per-event traces
 *   bit1 UDC_DWC3_DBG_STATS  periodic summaries
 *
 * Reset value: CONFIG_UDC_DWC3_CONSOLE_DEBUG (default 0). Change at
 * runtime from the shell: devmem <&udc_dwc3_dbg> 32 <mask>.
 *
 * The gate test lives in the XIP helpers, not at the call site: UDC code
 * is relocated to RAM (64 KB, ~99.7 % used) and an inline test at ~95
 * sites does not fit. A call to the helper costs the same RAM as the
 * printk call it replaces.
 */
extern uint32_t udc_dwc3_dbg;
#define UDC_DWC3_DBG_TRACE BIT(0)
#define UDC_DWC3_DBG_STATS BIT(1)
void udc_dwc3_trace(const char *fmt, ...);
void udc_dwc3_stats(const char *fmt, ...);
#define DWC3_TRACE(...) udc_dwc3_trace(__VA_ARGS__)
#define DWC3_STATS(...) udc_dwc3_stats(__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_USB_UDC_DWC3_USB_ENGINE_H */
