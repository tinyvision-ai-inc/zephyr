/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dwc3

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/shell/shell.h>
#include <zephyr/net_buf.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include "udc_common.h"
#include "udc_dwc3_int.h"

static atomic_t udc_dwc3_hwirq_count;
static atomic_t udc_dwc3_poll_count;
/* Event counted by hardware whose word had not reached the ring yet. */
static atomic_t udc_dwc3_evt_unlanded_count;

/* Microsecond steps to wait for a counted event's posted write to land. */
#define UDC_DWC3_EVT_LANDING_STEPS 64U
static atomic_t udc_dwc3_evt_count;
static atomic_t udc_dwc3_evt_overflow_count;
static atomic_t udc_dwc3_ctrl_setup_count;
static atomic_t udc_dwc3_ctrl_in_count;
static atomic_t udc_dwc3_ctrl_out_count;
static atomic_t udc_dwc3_norm_done_count;
static atomic_t udc_dwc3_enobufs_count;
static atomic_t udc_dwc3_usbrst_count;

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
/* ACM OUT 0x01 (epn=2) — validate silent HWO clears vs DEPEVT delivery */
#define UDC_DWC3_OUT_ACCT_EPN 2
#define UDC_DWC3_DIAG_LOG_FIRST 3U
#define UDC_DWC3_DIAG_LOG_EVERY 64U
static atomic_t udc_dwc3_out_acct_push;
static atomic_t udc_dwc3_out_acct_evt;
static atomic_t udc_dwc3_out_acct_pop;
static atomic_t udc_dwc3_out_acct_silent;
static atomic_t udc_dwc3_out_acct_defer_hwo;
static atomic_t udc_dwc3_out_silent_log_seq;
static atomic_t udc_dwc3_out_done_empty;
static atomic_t udc_dwc3_out_done_empty_log_seq;
#endif

static atomic_t udc_dwc3_ep0_stage_fixed;
static atomic_t udc_dwc3_ep0_stage_mismatch;
static atomic_t udc_dwc3_ep0_setuppending;
static atomic_t udc_dwc3_ep0_trb_err;
/* EP0 HWO-first: defer until HW releases TRB; absorb duplicate DEPEVT hints. */
static atomic_t udc_dwc3_ep0_defer_hwo_out;
static atomic_t udc_dwc3_ep0_defer_hwo_in;
static atomic_t udc_dwc3_ep0_dup_hint_out;
static atomic_t udc_dwc3_ep0_dup_hint_in;
static atomic_t udc_dwc3_ep0_poll_retired_out;
static atomic_t udc_dwc3_ep0_poll_retired_in;

#define UDC_DWC3_EP0_STORM_DUP_OUT   BIT(0)
#define UDC_DWC3_EP0_STORM_DUP_IN    BIT(1)
#define UDC_DWC3_EP0_STORM_DEFER_OUT BIT(2)
#define UDC_DWC3_EP0_STORM_DEFER_IN  BIT(3)

static uint8_t udc_dwc3_ep0_storm_first_logged;

static bool udc_dwc3_trb_hwo(const volatile struct udc_dwc3_trb *const trb);

/* IN endpoints re-armed with a fresh StartXfer after CLEAR_FEATURE(HALT). */
static atomic_t udc_dwc3_clrhalt_rearm;
/* IN endpoints force-restarted at UVC stream handoff (RTL UpdateXfer wedge). */
static atomic_t udc_dwc3_in_stream_rearm;

typedef void (*lattice_usb23_in_halt_fn)(const struct device *usb_dev, uint8_t ep_addr);
static lattice_usb23_in_halt_fn lattice_in_halt_cb;
static lattice_usb23_in_halt_fn lattice_in_clear_halt_cb;

static struct usb_setup_packet udc_dwc3_dbg_setup;
static bool udc_dwc3_dbg_setup_valid;

static const char *udc_dwc3_linkstate_str(const uint32_t dsts);

#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
static void udc_dwc3_dump_link_cfg(const struct device *const dev, const char *const tag);
#endif

#if defined(CONFIG_UDC_DWC3_DMA_SLOT_DIAG)
#define UDC_DWC3_DMA_SLOT_BYTES 1024U
#define UDC_DWC3_DMA_SCRUB_BYTE 0xA5U
#define UDC_DWC3_DMA_SCRUB_SNAP 16U

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sram1), okay)
#define UDC_DWC3_DMA_SRAM_BASE DT_REG_ADDR(DT_NODELABEL(sram1))
#define UDC_DWC3_DMA_SRAM_SIZE DT_REG_SIZE(DT_NODELABEL(sram1))
#else
#define UDC_DWC3_DMA_SRAM_BASE 0U
#define UDC_DWC3_DMA_SRAM_SIZE 0U
#endif

#define UDC_DWC3_DMA_SLOT_MAX MAX(1U, (UDC_DWC3_DMA_SRAM_SIZE / UDC_DWC3_DMA_SLOT_BYTES))

enum udc_dwc3_dma_op {
	UDC_DWC3_DMA_OUT_ENQ = 0,
	UDC_DWC3_DMA_IN_ENQ,
	UDC_DWC3_DMA_OUT_DONE,
	UDC_DWC3_DMA_IN_DONE,
};

struct udc_dwc3_dma_slot {
	uint8_t last_ep;
	uint8_t last_op;
	uint16_t out_enq;
	uint16_t in_enq;
	uint16_t out_done;
	uint16_t in_done;
	uint8_t scrub_snapshot[UDC_DWC3_DMA_SCRUB_SNAP];
	bool scrub_armed;
};

static struct udc_dwc3_dma_slot udc_dwc3_dma_slots[UDC_DWC3_DMA_SLOT_MAX];

static const char *udc_dwc3_dma_op_str(const enum udc_dwc3_dma_op op)
{
	switch (op) {
	case UDC_DWC3_DMA_OUT_ENQ:
		return "OUT-enq";
	case UDC_DWC3_DMA_IN_ENQ:
		return "IN-enq";
	case UDC_DWC3_DMA_OUT_DONE:
		return "OUT-done";
	case UDC_DWC3_DMA_IN_DONE:
		return "IN-done";
	default:
		return "?";
	}
}

static int udc_dwc3_dma_slot_index(const uintptr_t dma)
{
	uintptr_t off;

	if (UDC_DWC3_DMA_SRAM_SIZE == 0U || dma < UDC_DWC3_DMA_SRAM_BASE) {
		return -1;
	}

	off = dma - UDC_DWC3_DMA_SRAM_BASE;
	if (off >= UDC_DWC3_DMA_SRAM_SIZE) {
		return -1;
	}

	return (int)(off / UDC_DWC3_DMA_SLOT_BYTES);
}

static void udc_dwc3_dma_log_bytes(const char *const tag, const void *const data, const size_t len)
{
	const uint8_t *const bytes = data;
	const size_t n = MIN(len, UDC_DWC3_DMA_SCRUB_SNAP);
	char line[4 * UDC_DWC3_DMA_SCRUB_SNAP + 1];
	size_t pos = 0;

	for (size_t i = 0; i < n && pos + 3 < sizeof(line); i++) {
		pos += snprintk(&line[pos], sizeof(line) - pos, "%02x ", bytes[i]);
	}

	line[pos] = '\0';
	LOG_ERR("%s %s", tag, line);
}

static void udc_dwc3_dma_slot_note(const uint8_t ep, const enum udc_dwc3_dma_op op,
				   void *const data, const size_t len)
{
	const uintptr_t dma = (uintptr_t)data;
	const int idx = udc_dwc3_dma_slot_index(dma);

	LOG_DBG("DMA-SLOT ep=0x%02x %s dma=%p slot=%d", ep, udc_dwc3_dma_op_str(op), data, idx);

	if (idx < 0) {
		return;
	}

	struct udc_dwc3_dma_slot *const slot = &udc_dwc3_dma_slots[idx];

	slot->last_ep = ep;
	slot->last_op = (uint8_t)op;

	switch (op) {
	case UDC_DWC3_DMA_OUT_ENQ:
		slot->out_enq++;
		break;
	case UDC_DWC3_DMA_IN_ENQ:
		slot->in_enq++;
		if (len > 0U) {
			memset(data, UDC_DWC3_DMA_SCRUB_BYTE, len);
			memcpy(slot->scrub_snapshot, data,
			       MIN(len, UDC_DWC3_DMA_SCRUB_SNAP));
			slot->scrub_armed = true;
		}
		break;
	case UDC_DWC3_DMA_OUT_DONE:
		slot->out_done++;
		break;
	case UDC_DWC3_DMA_IN_DONE:
		slot->in_done++;
		slot->scrub_armed = false;
		break;
	default:
		break;
	}
}

static void udc_dwc3_dma_slot_dump_stall(const struct device *const dev);
#endif /* CONFIG_UDC_DWC3_DMA_SLOT_DIAG */

#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
static atomic_t udc_dwc3_ss_inact_count;
static atomic_t udc_dwc3_ss_recov_count;
#endif
static atomic_t udc_dwc3_remwk_count;

/* TRB memory buffer fields */
#define UDC_DWC3_TRB_STATUS_BUFSIZ_MASK				GENMASK(23, 0)
#define UDC_DWC3_TRB_STATUS_PCM1_MASK				GENMASK(25, 24)
#define UDC_DWC3_TRB_STATUS_PCM1_1PKT				(0x0 << 24)
#define UDC_DWC3_TRB_STATUS_PCM1_2PKT				(0x1 << 24)
#define UDC_DWC3_TRB_STATUS_PCM1_3PKT				(0x2 << 24)
#define UDC_DWC3_TRB_STATUS_PCM1_4PKT				(0x3 << 24)
#define UDC_DWC3_TRB_STATUS_TRBSTS_MASK				GENMASK(31, 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_OK				(0x0 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_MISSEDISOC			(0x1 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING			(0x2 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS		(0x4 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING			(0xf << 28)
#define UDC_DWC3_TRB_CTRL_HWO					BIT(0)
#define UDC_DWC3_TRB_CTRL_LST					BIT(1)
#define UDC_DWC3_TRB_CTRL_CHN					BIT(2)
#define UDC_DWC3_TRB_CTRL_CSP					BIT(3)
#define UDC_DWC3_TRB_CTRL_TRBCTL_MASK				GENMASK(9, 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL				(0x1 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP			(0x2 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2		(0x3 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3		(0x4 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA			(0x5 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_ISOCHRONOUS_1			(0x6 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_ISOCHRONOUS_N			(0x7 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB			(0x8 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP			(0x9 << 4)
#define UDC_DWC3_TRB_CTRL_ISP_IMI				BIT(10)
#define UDC_DWC3_TRB_CTRL_IOC					BIT(11)
#define UDC_DWC3_TRB_CTRL_PCM1_MASK				GENMASK(25, 24)
#define UDC_DWC3_TRB_CTRL_SPR					26
#define UDC_DWC3_TRB_CTRL_SIDSOFN_MASK				GENMASK(29, 14)

/* Incomplete coverage of all fields, but suited for what this driver supports */
#define UDC_DWC3_EVT_MASK					GENMASK(11, 0)
#define UDC_DWC3_DEPEVT_EPN_MASK				GENMASK(5, 1)
#define UDC_DWC3_DEPEVT_XFERCOMPLETE(epn)			(((epn) << 1) | (0x01 << 6))
#define UDC_DWC3_DEPEVT_XFERINPROGRESS(epn)			(((epn) << 1) | (0x02 << 6))
#define UDC_DWC3_DEPEVT_XFERNOTREADY(epn)			(((epn) << 1) | (0x03 << 6))

#define UDC_DWC3_DEPEVT_MAX_EPN 16
static atomic_t udc_dwc3_depevt_complete[UDC_DWC3_DEPEVT_MAX_EPN];
static atomic_t udc_dwc3_depevt_inprog[UDC_DWC3_DEPEVT_MAX_EPN];
#if defined(CONFIG_UDC_DWC3_LOST_EVT_DIAG)
static atomic_t udc_dwc3_sw_retire[UDC_DWC3_DEPEVT_MAX_EPN];
#endif

static void udc_dwc3_note_depevt(const uint32_t evt, const bool inprog)
{
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);

	if (epn >= 0 && epn < UDC_DWC3_DEPEVT_MAX_EPN) {
		atomic_inc(inprog ? &udc_dwc3_depevt_inprog[epn]
				  : &udc_dwc3_depevt_complete[epn]);
	}

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (epn == UDC_DWC3_OUT_ACCT_EPN) {
		atomic_inc(&udc_dwc3_out_acct_evt);
	}
#endif
}

static void udc_dwc3_ep0_reset_counts(void)
{
	atomic_set(&udc_dwc3_ep0_defer_hwo_out, 0);
	atomic_set(&udc_dwc3_ep0_defer_hwo_in, 0);
	atomic_set(&udc_dwc3_ep0_dup_hint_out, 0);
	atomic_set(&udc_dwc3_ep0_dup_hint_in, 0);
	atomic_set(&udc_dwc3_ep0_poll_retired_out, 0);
	atomic_set(&udc_dwc3_ep0_poll_retired_in, 0);
	udc_dwc3_ep0_storm_first_logged = 0U;
}

static void udc_dwc3_ep0_storm_bump(atomic_t *const counter, const uint8_t first_bit,
				    const char *const first_msg)
{
	const bool first_count = (atomic_get(counter) == 0);

	atomic_inc(counter);

	if (first_count && (udc_dwc3_ep0_storm_first_logged & first_bit) == 0U) {
		udc_dwc3_ep0_storm_first_logged |= first_bit;
		LOG_ERR("EP0 storm: %s", first_msg);
	}
}

static void udc_dwc3_ep0_storm_maybe_log(void)
{
	static int64_t last_log;
	static atomic_val_t last_dup_out;
	static atomic_val_t last_dup_in;
	static atomic_val_t last_def_out;
	static atomic_val_t last_def_in;
	const atomic_val_t dup_out = atomic_get(&udc_dwc3_ep0_dup_hint_out);
	const atomic_val_t dup_in = atomic_get(&udc_dwc3_ep0_dup_hint_in);
	const atomic_val_t def_out = atomic_get(&udc_dwc3_ep0_defer_hwo_out);
	const atomic_val_t def_in = atomic_get(&udc_dwc3_ep0_defer_hwo_in);
	const int64_t now = k_uptime_get();

	if (dup_out == last_dup_out && dup_in == last_dup_in &&
	    def_out == last_def_out && def_in == last_def_in) {
		return;
	}

	if ((dup_out + dup_in + def_out + def_in) == 0) {
		return;
	}

	if (now - last_log < 1000) {
		return;
	}

	if (dup_out != last_dup_out || dup_in != last_dup_in ||
	    def_out != last_def_out || def_in != last_def_in) {
		LOG_ERR("EP0 storm: dup out=%ld in=%ld defer_hwo out=%ld in=%ld "
			"poll_ret out=%ld in=%ld setup=%u ctrlin=%u ctrlout=%u enobufs=%u",
			(long)dup_out, (long)dup_in, (long)def_out, (long)def_in,
			(long)atomic_get(&udc_dwc3_ep0_poll_retired_out),
			(long)atomic_get(&udc_dwc3_ep0_poll_retired_in),
			(uint32_t)atomic_get(&udc_dwc3_ctrl_setup_count),
			(uint32_t)atomic_get(&udc_dwc3_ctrl_in_count),
			(uint32_t)atomic_get(&udc_dwc3_ctrl_out_count),
			(uint32_t)atomic_get(&udc_dwc3_enobufs_count));
		last_log = now;
		last_dup_out = dup_out;
		last_dup_in = dup_in;
		last_def_out = def_out;
		last_def_in = def_in;
	}
}

enum udc_dwc3_ep0_gate {
	UDC_DWC3_EP0_GATE_PROCESS = 0,
	UDC_DWC3_EP0_GATE_DEFER_HWO,
	UDC_DWC3_EP0_GATE_DUP_HINT,
};

static enum udc_dwc3_ep0_gate udc_dwc3_ep0_gate(struct udc_dwc3_ep_data *const ep_data)
{
	const volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[0];

	if (udc_dwc3_trb_hwo(trb)) {
		return UDC_DWC3_EP0_GATE_DEFER_HWO;
	}

	/*
	 * HW released the TRB but SW has no buffer — completion already
	 * retired (late/duplicate DEPEVT or poll).  Ignore busy: it can stay
	 * set after the real completion path ran.
	 */
	if (udc_buf_peek(&ep_data->cfg) == NULL) {
		return UDC_DWC3_EP0_GATE_DUP_HINT;
	}

	return UDC_DWC3_EP0_GATE_PROCESS;
}

static void udc_dwc3_next_ctrl(const struct device *const dev,
			       struct udc_dwc3_ep_data *const ep_data);
static void udc_dwc3_on_ctrl_in(const struct device *const dev);
static void udc_dwc3_on_ctrl_out(const struct device *const dev);

static bool udc_dwc3_ep0_note_gate(const struct device *const dev,
				   struct udc_dwc3_ep_data *const ep_data,
				   const enum udc_dwc3_ep0_gate gate)
{
	switch (gate) {
	case UDC_DWC3_EP0_GATE_DEFER_HWO:
		if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
			udc_dwc3_ep0_storm_bump(&udc_dwc3_ep0_defer_hwo_in,
						UDC_DWC3_EP0_STORM_DEFER_IN,
						"defer_hwo IN (TRB still owned by HW)");
		} else {
			udc_dwc3_ep0_storm_bump(&udc_dwc3_ep0_defer_hwo_out,
						UDC_DWC3_EP0_STORM_DEFER_OUT,
						"defer_hwo OUT (TRB still owned by HW)");
		}
		udc_dwc3_ep0_storm_maybe_log();
		return false;
	case UDC_DWC3_EP0_GATE_DUP_HINT:
		if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
			udc_dwc3_ep0_storm_bump(&udc_dwc3_ep0_dup_hint_in,
						UDC_DWC3_EP0_STORM_DUP_IN,
						"dup_hint IN (late completion, queue empty)");
		} else {
			udc_dwc3_ep0_storm_bump(&udc_dwc3_ep0_dup_hint_out,
						UDC_DWC3_EP0_STORM_DUP_OUT,
						"dup_hint OUT (late completion, queue empty)");
		}
		if (udc_ep_is_busy(&ep_data->cfg)) {
			udc_ep_set_busy(&ep_data->cfg, false);
		}
		udc_dwc3_ep0_storm_maybe_log();
		return false;
	case UDC_DWC3_EP0_GATE_PROCESS:
		return true;
	}

	return true;
}

static void udc_dwc3_reset_depevt_counts(void)
{
	for (int epn = 0; epn < UDC_DWC3_DEPEVT_MAX_EPN; epn++) {
		atomic_set(&udc_dwc3_depevt_complete[epn], 0);
		atomic_set(&udc_dwc3_depevt_inprog[epn], 0);
	}

	udc_dwc3_ep0_reset_counts();

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	atomic_set(&udc_dwc3_out_acct_push, 0);
	atomic_set(&udc_dwc3_out_acct_evt, 0);
	atomic_set(&udc_dwc3_out_acct_pop, 0);
	atomic_set(&udc_dwc3_out_acct_silent, 0);
	atomic_set(&udc_dwc3_out_acct_defer_hwo, 0);
	atomic_set(&udc_dwc3_out_silent_log_seq, 0);
	atomic_set(&udc_dwc3_out_done_empty, 0);
	atomic_set(&udc_dwc3_out_done_empty_log_seq, 0);
#endif
}
#define UDC_DWC3_DEPEVT_RXTXFIFOEVT(epn)			(((epn) << 1) | (0x04 << 6))
#define UDC_DWC3_DEPEVT_STREAMEVT(epn)				(((epn) << 1) | (0x06 << 6))
#define UDC_DWC3_DEPEVT_EPCMDCMPLT(epn)				(((epn) << 1) | (0x07 << 6))
/* For XferNotReady */
#define UDC_DWC3_DEPEVT_STATUS_B3_MASK				GENMASK(2, 0)
#define UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_SETUP			(0x0 << 0)
#define UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_DATA			(0x1 << 0)
#define UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_STATUS		(0x2 << 0)
/* For XferComplete or XferInProgress */
#define UDC_DWC3_DEPEVT_STATUS_BUSERR				BIT(0)
#define UDC_DWC3_DEPEVT_STATUS_SHORT				BIT(1)
#define UDC_DWC3_DEPEVT_STATUS_IOC				BIT(2)
/* For XferComplete */
#define UDC_DWC3_DEPEVT_STATUS_LST				BIT(3)
/* For XferInProgress */
#define UDC_DWC3_DEPEVT_STATUS_MISSED_ISOC			BIT(3)
/* For StreamEvt */
#define UDC_DWC3_DEPEVT_STATUS_STREAMFOUND			0x1
#define UDC_DWC3_DEPEVT_STATUS_STREAMNOTFOUND			0x2
#define UDC_DWC3_DEVT_DISCONNEVT				(BIT(0) | (0x0 << 8))
#define UDC_DWC3_DEVT_USBRST					(BIT(0) | (0x1 << 8))
#define UDC_DWC3_DEVT_CONNECTDONE				(BIT(0) | (0x2 << 8))
#define UDC_DWC3_DEVT_ULSTCHNG					(BIT(0) | (0x3 << 8))
#define UDC_DWC3_DEVT_WKUPEVT					(BIT(0) | (0x4 << 8))
#define UDC_DWC3_DEVT_SUSPEND					(BIT(0) | (0x6 << 8))
#define UDC_DWC3_DEVT_SOF					(BIT(0) | (0x7 << 8))
#define UDC_DWC3_DEVT_ERRTICERR					(BIT(0) | (0x9 << 8))
#define UDC_DWC3_DEVT_CMDCMPLT					(BIT(0) | (0xa << 8))
#define UDC_DWC3_DEVT_EVNTOVERFLOW				(BIT(0) | (0xb << 8))
#define UDC_DWC3_DEVT_VNDRDEVTSTRCVED				(BIT(0) | (0xc << 8))

/* Device Endpoint Commands and Parameters */
#define UDC_DWC3_DEPCMDPAR2(n)					(0xc800 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR1(n)					(0xc804 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR0(n)					(0xc808 + 16 * (n))
#define UDC_DWC3_DEPCMD(n)					(0xc80c + 16 * (n))
/* Common fields to DEPCMD */
#define UDC_DWC3_DEPCMD_HIPRI_FORCERM				(1 << 11)
#define UDC_DWC3_DEPCMD_STATUS_MASK				GENMASK(15, 12)
#define UDC_DWC3_DEPCMD_STATUS_OK				(0 << 12)
#define UDC_DWC3_DEPCMD_STATUS_CMDERR				(1 << 12)
#define UDC_DWC3_DEPCMD_XFERRSCIDX_MASK				GENMASK(22, 16)
/* DEPCFG Command and Parameters */
#define UDC_DWC3_DEPCMD_DEPCFG					(1 << 0)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_MASK			GENMASK(2, 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL			(0x0 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_ISOC			(0x1 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_BULK			(0x2 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_INT			(0x3 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_MPS_MASK			GENMASK(13, 3)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK			GENMASK(21, 17)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK			GENMASK(25, 22)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MASK			GENMASK(31, 30)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_INIT			(0x0 << 30)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_RESTORE		(0x1 << 30)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MODIFY		(0x2 << 30)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_INTRNUM_MASK			GENMASK(4, 0)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN			BIT(8)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN			BIT(9)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN			BIT(10)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_RXTXFIFOEVTEN		BIT(11)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_STREAMEVTEN			BIT(13)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_LIMITTXDMA			BIT(15)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_BINTERVAL_MASK		GENMASK(23, 16)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_STRMCAP			BIT(24)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK		GENMASK(29, 25)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_BULKBASED			BIT(30)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_FIFOBASED			BIT(31)
#define UDC_DWC3_DEPCMDPAR2_DEPCFG_EPSTATE_MASK			GENMASK(31, 0)
/* DEPXFERCFG Command and Parameters */
#define UDC_DWC3_DEPCMD_DEPXFERCFG				(0x2 << 0)
#define UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK		GENMASK(15, 0)
/* Other Commands */
#define UDC_DWC3_DEPCMD_DEPGETSTATE				(0x3 << 0)
#define UDC_DWC3_DEPCMD_DEPSETSTALL				(0x4 << 0)
#define UDC_DWC3_DEPCMD_DEPCSTALL				(0x5 << 0)
#define UDC_DWC3_DEPCMD_DEPSTRTXFER				(0x6 << 0)
#define UDC_DWC3_DEPCMD_DEPUPDXFER				(0x7 << 0)
#define UDC_DWC3_DEPCMD_DEPENDXFER				(0x8 << 0)
#define UDC_DWC3_DEPCMD_DEPSTARTCFG				(0x9 << 0)
#define UDC_DWC3_DEPCMD_CMDACT					BIT(10)

/* Global USB2 (UTMI/ULPI) PHY configuration */
#define UDC_DWC3_GUSB2PHYCFG					0xC200
#define UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST				BIT(31)
#define UDC_DWC3_GUSB2PHYCFG_ULPIEXTVBUSINDICATOR		BIT(18)
#define UDC_DWC3_GUSB2PHYCFG_ULPIEXTVBUSDRV			BIT(17)
#define UDC_DWC3_GUSB2PHYCFG_ULPICLKSUSM			BIT(16)
#define UDC_DWC3_GUSB2PHYCFG_ULPIAUTORES			BIT(15)
#define UDC_DWC3_GUSB2PHYCFG_USBTRDTIM_MASK			GENMASK(13, 10)
#define UDC_DWC3_GUSB2PHYCFG_USBTRDTIM_16BIT			(5 << 10)
#define UDC_DWC3_GUSB2PHYCFG_USBTRDTIM_8BIT			(9 << 10)
#define UDC_DWC3_GUSB2PHYCFG_ENBLSLPM				BIT(8)
#define UDC_DWC3_GUSB2PHYCFG_PHYSEL				BIT(7)
#define UDC_DWC3_GUSB2PHYCFG_SUSPHY				BIT(6)
#define UDC_DWC3_GUSB2PHYCFG_FSINTF				BIT(5)
#define UDC_DWC3_GUSB2PHYCFG_ULPI_UTMI_SEL			BIT(4)
#define UDC_DWC3_GUSB2PHYCFG_PHYIF				BIT(3)
#define UDC_DWC3_GUSB2PHYCFG_TOUTCAL_MASK			GENMASK(2, 0)

/* Global USB 3.0 PIPE Control Register */
#define UDC_DWC3_GUSB3PIPECTL					0xc2c0
#define UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST			BIT(31)
#define UDC_DWC3_GUSB3PIPECTL_UX_EXIT_IN_PX			BIT(27)
#define UDC_DWC3_GUSB3PIPECTL_PING_ENHANCEMENT_EN		BIT(26)
#define UDC_DWC3_GUSB3PIPECTL_U1U2EXITFAIL_TO_RECOV		BIT(25)
#define UDC_DWC3_GUSB3PIPECTL_REQUEST_P1P2P3			BIT(24)
#define UDC_DWC3_GUSB3PIPECTL_STARTXDETU3RXDET			BIT(23)
#define UDC_DWC3_GUSB3PIPECTL_DISRXDETU3RXDET			BIT(22)
#define UDC_DWC3_GUSB3PIPECTL_P1P2P3DELAY_MASK			GENMASK(21, 19)
#define UDC_DWC3_GUSB3PIPECTL_DELAYP0TOP1P2P3			BIT(18)
#define UDC_DWC3_GUSB3PIPECTL_SUSPENDENABLE			BIT(17)
#define UDC_DWC3_GUSB3PIPECTL_DATWIDTH_MASK			GENMASK(16, 15)
#define UDC_DWC3_GUSB3PIPECTL_ABORTRXDETINU2			BIT(14)
#define UDC_DWC3_GUSB3PIPECTL_SKIPRXDET				BIT(13)
#define UDC_DWC3_GUSB3PIPECTL_LFPSP0ALGN			BIT(12)
#define UDC_DWC3_GUSB3PIPECTL_P3P2TRANOK			BIT(11)
#define UDC_DWC3_GUSB3PIPECTL_P3EXSIGP2				BIT(10)
#define UDC_DWC3_GUSB3PIPECTL_LFPSFILT				BIT(9)
#define UDC_DWC3_GUSB3PIPECTL_TXSWING				BIT(6)
#define UDC_DWC3_GUSB3PIPECTL_TXMARGIN_MASK			GENMASK(5, 3)
#define UDC_DWC3_GUSB3PIPECTL_TXDEEMPHASIS_MASK			GENMASK(2, 1)
#define UDC_DWC3_GUSB3PIPECTL_ELASTICBUFFERMODE			BIT(0)

/* USB Device Configuration Register */
#define UDC_DWC3_DCFG						0xc700
#define UDC_DWC3_DCFG_IGNORESTREAMPP				BIT(23)
#define UDC_DWC3_DCFG_LPMCAP					BIT(22)
#define UDC_DWC3_DCFG_NUMP_MASK					GENMASK(21, 17)
#define UDC_DWC3_DCFG_INTRNUM_MASK				GENMASK(16, 12)
#define UDC_DWC3_DCFG_PERFRINT_MASK				GENMASK(11, 10)
#define UDC_DWC3_DCFG_PERFRINT_80				(0x0 << 10)
#define UDC_DWC3_DCFG_PERFRINT_85				(0x1 << 10)
#define UDC_DWC3_DCFG_PERFRINT_90				(0x2 << 10)
#define UDC_DWC3_DCFG_PERFRINT_95				(0x3 << 10)
#define UDC_DWC3_DCFG_DEVADDR_MASK				GENMASK(9, 3)
#define UDC_DWC3_DCFG_DEVSPD_MASK				GENMASK(2, 0)
#define UDC_DWC3_DCFG_DEVSPD_SUPER_SPEED			(0x4 << 0)
#define UDC_DWC3_DCFG_DEVSPD_HIGH_SPEED				(0x0 << 0)
#define UDC_DWC3_DCFG_DEVSPD_FULL_SPEED				(0x1 << 0)

/* Global SoC Bus Configuration Register */
#define UDC_DWC3_GSBUSCFG0					0xc100
#define UDC_DWC3_GSBUSCFG0_DATRDREQINFO				GENMASK(31, 28)
#define UDC_DWC3_GSBUSCFG0_DESRDREQINFO				GENMASK(27, 24)
#define UDC_DWC3_GSBUSCFG0_DATWRREQINFO				GENMASK(23, 20)
#define UDC_DWC3_GSBUSCFG0_DESWRREQINFO				GENMASK(19, 16)
#define UDC_DWC3_GSBUSCFG0_DATBIGEND				BIT(11)
#define UDC_DWC3_GSBUSCFG0_DESBIGEND				BIT(10)
#define UDC_DWC3_GSBUSCFG0_INCR256BRSTENA			BIT(7)
#define UDC_DWC3_GSBUSCFG0_INCR128BRSTENA			BIT(6)
#define UDC_DWC3_GSBUSCFG0_INCR64BRSTENA			BIT(5)
#define UDC_DWC3_GSBUSCFG0_INCR32BRSTENA			BIT(4)
#define UDC_DWC3_GSBUSCFG0_INCR16BRSTENA			BIT(3)
#define UDC_DWC3_GSBUSCFG0_INCR8BRSTENA				BIT(2)
#define UDC_DWC3_GSBUSCFG0_INCR4BRSTENA				BIT(1)
#define UDC_DWC3_GSBUSCFG0_INCRBRSTENA				BIT(0)

/* Global Tx Threshold Control Register */
#define UDC_DWC3_GTXTHRCFG					0xc108
#define UDC_DWC3_GTXTHRCFG_USBTXPKTCNTSEL			BIT(29)
#define UDC_DWC3_GTXTHRCFG_USBTXPKTCNT_MASK			GENMASK(27, 24)
#define UDC_DWC3_GTXTHRCFG_USBMAXTXBURSTSIZE_MASK		GENMASK(23, 16)

/* Global control register */
#define UDC_DWC3_GCTL						0xc110
#define UDC_DWC3_GCTL_PWRDNSCALE_MASK				GENMASK(31, 19)
#define UDC_DWC3_GCTL_MASTERFILTBYPASS				BIT(18)
#define UDC_DWC3_GCTL_BYPSSETADDR				BIT(17)
#define UDC_DWC3_GCTL_U2RSTECN					BIT(16)
#define UDC_DWC3_GCTL_FRMSCLDWN_MASK				GENMASK(15, 14)
#define UDC_DWC3_GCTL_PRTCAPDIR_MASK				GENMASK(13, 12)
#define UDC_DWC3_GCTL_CORESOFTRESET				BIT(11)
#define UDC_DWC3_GCTL_DEBUGATTACH				BIT(8)
#define UDC_DWC3_GCTL_RAMCLKSEL_MASK				GENMASK(7, 6)
#define UDC_DWC3_GCTL_SCALEDOWN_MASK				GENMASK(5, 4)
#define UDC_DWC3_GCTL_DISSCRAMBLE				BIT(3)
#define UDC_DWC3_GCTL_DSBLCLKGTNG				BIT(0)

/* Global User Control Register */
#define UDC_DWC3_GUCTL						0xc12c
#define UDC_DWC3_GUCTL_NOEXTRDL					BIT(21)
#define UDC_DWC3_GUCTL_PSQEXTRRESSP_MASK			GENMASK(20, 18)
#define UDC_DWC3_GUCTL_PSQEXTRRESSP_EN				BIT(18)
#define UDC_DWC3_GUCTL_SPRSCTRLTRANSEN				BIT(17)
#define UDC_DWC3_GUCTL_RESBWHSEPS				BIT(16)
#define UDC_DWC3_GUCTL_CMDEVADDR				BIT(15)
#define UDC_DWC3_GUCTL_USBHSTINAUTORETRYEN			BIT(14)
#define UDC_DWC3_GUCTL_DTCT_MASK				GENMASK(10, 9)
#define UDC_DWC3_GUCTL_DTFT_MASK				GENMASK(8, 0)

/* USB Device Control register */
#define UDC_DWC3_DCTL						0xc704
#define UDC_DWC3_DCTL_RUNSTOP					BIT(31)
#define UDC_DWC3_DCTL_CSFTRST					BIT(30)
#define UDC_DWC3_DCTL_HIRDTHRES_4				BIT(28)
#define UDC_DWC3_DCTL_HIRDTHRES_TIME_MASK			GENMASK(27, 24)
#define UDC_DWC3_DCTL_APPL1RES					BIT(23)
#define UDC_DWC3_DCTL_LPM_NYET_THRES_MASK			GENMASK(23, 20)
#define UDC_DWC3_DCTL_KEEPCONNECT				BIT(19)
#define UDC_DWC3_DCTL_L1HIBERNATIONEN				BIT(18)
#define UDC_DWC3_DCTL_CRS					BIT(17)
#define UDC_DWC3_DCTL_CSS					BIT(16)
#define UDC_DWC3_DCTL_INITU2ENA					BIT(12)
#define UDC_DWC3_DCTL_ACCEPTU2ENA				BIT(11)
#define UDC_DWC3_DCTL_INITU1ENA					BIT(10)
#define UDC_DWC3_DCTL_ACCEPTU1ENA				BIT(9)
#define UDC_DWC3_DCTL_ULSTCHNGREQ_MASK				GENMASK(8, 5)
#define UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP			(0x8 << 5)
#define UDC_DWC3_DCTL_TSTCTL_MASK				GENMASK(4, 1)

/* USB Device Event Enable Register */
#define UDC_DWC3_DEVTEN						0xc708
#define UDC_DWC3_DEVTEN_INACTTIMEOUTRCVEDEN			BIT(13)
#define UDC_DWC3_DEVTEN_VNDRDEVTSTRCVEDEN			BIT(12)
#define UDC_DWC3_DEVTEN_EVNTOVERFLOWEN				BIT(11)
#define UDC_DWC3_DEVTEN_CMDCMPLTEN				BIT(10)
#define UDC_DWC3_DEVTEN_ERRTICERREN				BIT(9)
#define UDC_DWC3_DEVTEN_SOFEN					BIT(7)
#define UDC_DWC3_DEVTEN_EOPFEN					BIT(6)
#define UDC_DWC3_DEVTEN_HIBERNATIONREQEVTEN			BIT(5)
#define UDC_DWC3_DEVTEN_WKUPEVTEN				BIT(4)
#define UDC_DWC3_DEVTEN_ULSTCNGEN				BIT(3)
#define UDC_DWC3_DEVTEN_CONNECTDONEEN				BIT(2)
#define UDC_DWC3_DEVTEN_USBRSTEN				BIT(1)
#define UDC_DWC3_DEVTEN_DISCONNEVTEN				BIT(0)

/* USB Device Event Register */

/* Endpoint Global Event Buffer Address (64-bit) */
#define UDC_DWC3_GEVNTADR(n)					(0xc400 + 16 * (n))
#define UDC_DWC3_GEVNTADR_LO(n)					(0xc400 + 16 * (n))
#define UDC_DWC3_GEVNTADR_HI(n)					(0xc404 + 16 * (n))

/* Endpoint Global Event Buffer Size */
#define UDC_DWC3_GEVNTSIZ(n)					(0xc408 + 16 * (n))
#define UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK			BIT(31)

/* Endpoint Global Event Buffer Count (of valid event) */
#define UDC_DWC3_GEVNTCOUNT(n)					(0xc40c + 16 * (n))

/* USB Device Active USB Endpoint Enable */
#define UDC_DWC3_DALEPENA					0xC720
#define UDC_DWC3_DALEPENA_USBACTEP(n)				(1 << (n))

/* USB Device Core Identification and Release Number Register */
#define UDC_DWC3_GCOREID					0xC120
#define UDC_DWC3_GCOREID_CORE_MASK				GENMASK(31, 16)
#define UDC_DWC3_GCOREID_REL_MASK				GENMASK(15, 0)

/* USB Globa Status register */
#define UDC_DWC3_GSTS						0xc118
#define UDC_DWC3_GSTS_CBELT_MASK				GENMASK(31, 20)
#define UDC_DWC3_GSTS_SSIC_IP					BIT(11)
#define UDC_DWC3_GSTS_OTG_IP					BIT(10)
#define UDC_DWC3_GSTS_BC_IP					BIT(9)
#define UDC_DWC3_GSTS_ADP_IP					BIT(8)
#define UDC_DWC3_GSTS_HOST_IP					BIT(7)
#define UDC_DWC3_GSTS_DEVICE_IP					BIT(6)
#define UDC_DWC3_GSTS_CSRTIMEOUT				BIT(5)
#define UDC_DWC3_GSTS_BUSERRADDRVLD				BIT(4)
#define UDC_DWC3_GSTS_CURMOD_MASK				GENMASK(1, 0)

/* USB Global TX FIFO Size register */
#define UDC_DWC3_GTXFIFOSIZ(n)					(0xc300 + 4 * (n))
#define UDC_DWC3_GTXFIFOSIZ_TXFSTADDR_MASK			GENMASK(31, 16)
#define UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK				GENMASK(15, 0)

/* USB Global RX FIFO Size register */
#define UDC_DWC3_GRXFIFOSIZ(n)					(0xc380 + 4 * (n))
#define UDC_DWC3_GRXFIFOSIZ_RXFSTADDR_MASK			GENMASK(31, 16)
#define UDC_DWC3_GRXFIFOSIZ_RXFDEP_MASK				GENMASK(15, 0)

/* USB Bus Error Address registers */
#define UDC_DWC3_GBUSERRADDR					0xc130
#define UDC_DWC3_GBUSERRADDR_LO					0xc130
#define UDC_DWC3_GBUSERRADDR_HI					0xc134

/* USB Controller Debug register */
#define UDC_DWC3_CTLDEBUG					0xe000
#define UDC_DWC3_CTLDEBUG_LO					0xe000
#define UDC_DWC3_CTLDEBUG_HI					0xe004

/* USB Analyzer Trace register */
#define UDC_DWC3_ANALYZERTRACE					0xe008

/* USB Global Debug Queue/FIFO Space Available register */
#define UDC_DWC3_GDBGFIFOSPACE					0xc160
#define UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK			GENMASK(31, 16)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_MASK			GENMASK(8, 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TX			(0x0 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RX			(0x1 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXREQ			(0x2 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQ			(0x3 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXINFO			(0x4 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOL		(0x5 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCH		(0x6 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_WREVENT		(0x7 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENT		(0x8 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK			GENMASK(4, 0)

/* USB Global Debug LTSSM register */
#define UDC_DWC3_GDBGLTSSM					0xc164

/* Global Debug LNMCC Register */
#define UDC_DWC3_GDBGLNMCC					0xc168

/* Global Debug BMU Register */
#define UDC_DWC3_GDBGBMU					0xc16c

/* Global Debug LSP MUX Register - Device*/
#define UDC_DWC3_GDBGLSPMUX_DEV					0xc170

/* Global Debug LSP MUX Register - Host */
#define UDC_DWC3_GDBGLSPMUX_HST					0xc170

/* Global Debug LSP Register */
#define UDC_DWC3_GDBGLSP					0xc174

/* Global Debug Endpoint Information Register 0 */
#define UDC_DWC3_GDBGEPINFO0					0xc178

/* Global Debug Endpoint Information Register 1 */
#define UDC_DWC3_GDBGEPINFO1					0xc17c

/* U3 Root Hub Debug Register */
#define UDC_DWC3_BU3RHBDBG0					0xd800

/* USB Device Status register */
#define UDC_DWC3_DSTS						0xC70C
#define UDC_DWC3_DSTS_DCNRD					BIT(29)
#define UDC_DWC3_DSTS_SRE					BIT(28)
#define UDC_DWC3_DSTS_RSS					BIT(25)
#define UDC_DWC3_DSTS_SSS					BIT(24)
#define UDC_DWC3_DSTS_COREIDLE					BIT(23)
#define UDC_DWC3_DSTS_DEVCTRLHLT				BIT(22)
#define UDC_DWC3_DSTS_USBLNKST_MASK				GENMASK(21, 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U0				(0x0 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U1				(0x1 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U2				(0x2 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U3				(0x3 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS			(0x4 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET			(0x5 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT			(0x6 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_POLL			(0x7 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_RECOV			(0x8 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_HRESET			(0x9 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_CMPLY			(0xa << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_LPBK			(0xb << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME		(0xf << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE			(0x0 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE			(0x2 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE		(0x3 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED		(0x4 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND		(0x5 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_RESET			(0xe << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_RESUME			(0xf << 18)
#define UDC_DWC3_DSTS_RXFIFOEMPTY				BIT(17)
#define UDC_DWC3_DSTS_SOFFN_MASK				GENMASK(16, 3)
#define UDC_DWC3_DSTS_CONNECTSPD_MASK				GENMASK(2, 0)
#define UDC_DWC3_DSTS_CONNECTSPD_HS				(0x0 << 0)
#define UDC_DWC3_DSTS_CONNECTSPD_FS				(0x1 << 0)
#define UDC_DWC3_DSTS_CONNECTSPD_SS				(0x4 << 0)

/* Device Generic Command and Parameter */
#define UDC_DWC3_DGCMDPAR					0xc710
#define UDC_DWC3_DGCMD						0xc714
#define UDC_DWC3_DGCMD_STATUS_MASK				GENMASK(15, 12)
#define UDC_DWC3_DGCMD_STATUS_ERR				(1 << 12)
#define UDC_DWC3_DGCMD_STATUS_OK				(0 << 12)
#define UDC_DWC3_DGCMD_ACT					BIT(10)
#define UDC_DWC3_DGCMD_IOC					BIT(8)
#define UDC_DWC3_DGCMD_MASK					GENMASK(7, 0)
/* EXITLATENCY command and parameters */
#define UDC_DWC3_DGCMD_EXITLATENCY				(2 << 0)
/* Other Commands and Parameters */
#define UDC_DWC3_DGCMD_LINKFUNCTION				(1 << 0)
#define UDC_DWC3_DGCMD_WAKENOTIFNUM				(3 << 0)
#define UDC_DWC3_DGCMD_FIFOFLUSHONE				(9 << 0)
#define UDC_DWC3_DGCMD_FIFOFLUSHALL				(10 << 0)
#define UDC_DWC3_DGCMD_ENDPOINTNRDY				(12 << 0)
#define UDC_DWC3_DGCMD_LOOPBACKTEST				(16 << 0)
#define UDC_DWC3_DGCMD_ROLEREQUEST				(6 << 0)

/* Hardware parameters */
#define UDC_DWC3_GHWPARAMS0					0xc140
#define UDC_DWC3_GHWPARAMS1					0xc144
#define UDC_DWC3_GHWPARAMS2					0xc148
#define UDC_DWC3_GHWPARAMS3					0xc14c
#define UDC_DWC3_GHWPARAMS4					0xc150
#define UDC_DWC3_GHWPARAMS5					0xc154
#define UDC_DWC3_GHWPARAMS6					0xc158
#define UDC_DWC3_GHWPARAMS6_USB3_HSPHY_INTERFACE		GENMASK(5, 4)
#define UDC_DWC3_GHWPARAMS7					0xc15c
#define UDC_DWC3_GHWPARAMS8					0xc600

/* Helper macros */
#define LO32(n)			((uint32_t)((uint64_t)(n) & 0xffffffff))
#define HI32(n)			((uint32_t)((uint64_t)(n) >> 32))

/*
 * One DMA transaction request passed from the CPU to the DWC3 core.
 *
 * This structure is described by the datasheet of DWC3 and shared between the hardware and
 * software driver. If the architecture involves cache, it must be flushed before accessing
 * this memory region.
 */
/* struct udc_dwc3_trb — defined in udc_dwc3_int.h */

/*
 * Controller configuration items that can remain in non-volatile memory
 */
struct udc_dwc3_config {
	DEVICE_MMIO_NAMED_ROM(base);
	/* USB endpoints data */
	struct udc_dwc3_ep_data *ep_data_in;
	struct udc_dwc3_ep_data *ep_data_out;
	/* Pointer to the DMA-accessible buffer of TRBs and its size */
	struct udc_dwc3_trb (*trb_buf_in)[CONFIG_UDC_DWC3_TRB_NUM];
	struct udc_dwc3_trb (*trb_buf_out)[CONFIG_UDC_DWC3_TRB_NUM];
	/* USB device configuration */
	int maximum_speed_idx;
	/* Pointers to event buffer fetched by DWC3 with DMA */
	volatile uint32_t *evt_buf;
	/* Data used by vendor-specific functions ("quirks") */
	const void *quirk_config;
	void *quirk_data;
	/* IRQ management functions */
	void (*irq_enable_func)(void);
	void (*irq_disable_func)(void);
	/* Number of hardware endpoint set for input or output */
	uint8_t num_in_eps;
	uint8_t num_out_eps;
};

/*
 * Data of each instance of the driver, that can be read and written to.
 *
 * Accessed via "udc_get_private(dev)".
 */
struct udc_dwc3_data {
	DEVICE_MMIO_NAMED_RAM(base);
	/* Index within trb where to queue new TRBs */
	uint32_t evt_next;
	/* Back-reference to parent */
	const struct device *dev;
	/* Drains the DWC3 event buffer in thread context (mutexes are illegal in ISRs) */
	struct k_work event_work;
	/* Cached SETUP for the in-flight control transfer. udc_data->setup stays
	 * stale because this driver passes NULL to udc_setup_received().
	 */
	struct usb_setup_packet ctrl_setup;
#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
	struct k_work_delayable health_work;
#endif
#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
	/* Periodic SW-retire poll to recover IN transfers whose XferComplete
	 * event was dropped by the controller/RTL (see the worker for details).
	 */
	struct k_work_delayable in_poll_work;
#endif
#if defined(CONFIG_UDC_DWC3_EP_SM)
	/* SM poll/depevt deferred until at least one non-control EP is enabled. */
	atomic_t bulk_eps_live;
	/* Rate-limit for the "poll would have stopped with work pending" report. */
	bool poll_live_zero_reported;
#endif
	/* Non-control transfer resources allocated for the current config. */
	bool startcfg_nonctrl_done;
};

/*
 * Indexes matching the "device-speed" devicetree property values.
 */
enum {
	UDC_DWC3_SPEED_IDX_FULL_SPEED = 1,
	UDC_DWC3_SPEED_IDX_HIGH_SPEED = 2,
	UDC_DWC3_SPEED_IDX_SUPER_SPEED = 3,
};

/*
 * Vendor quirks
 *
 * Definition of vendor-specific functions that can be overwritten on a per-SoC basis.
 */

struct udc_dwc3_vendor_quirks {
	int (*preinit)(const struct device *const dev);
	int (*init)(const struct device *const dev);
	int (*enable)(const struct device *const dev);
	int (*disable)(const struct device *const dev);
	int (*shutdown)(const struct device *const dev);
};

/* Helper for accessing vendor quirks */
#define UDC_DWC3_QUIRK_CFG(dev)  (((const struct udc_dwc3_config *)(dev->config))->quirk_config)
#define UDC_DWC3_QUIRK_DATA(dev) (((const struct udc_dwc3_config *)(dev->config))->quirk_data)

#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
#include "udc_dwc3_lattice_usb23.h"
#endif

/* Wrapper functions that fallback to returning 0 if no quirk is needed */
#define UDC_DWC3_QUIRK_FUNC_DEFINE(fn)						\
	static inline int udc_dwc3_quirk_##fn(const struct device *const dev)	\
	{									\
		if (udc_dwc3_vendor_quirks.fn != NULL) {			\
			return udc_dwc3_vendor_quirks.fn(dev);			\
		}								\
										\
		return 0;							\
	}

UDC_DWC3_QUIRK_FUNC_DEFINE(preinit);
UDC_DWC3_QUIRK_FUNC_DEFINE(init);
UDC_DWC3_QUIRK_FUNC_DEFINE(enable);
UDC_DWC3_QUIRK_FUNC_DEFINE(disable);
UDC_DWC3_QUIRK_FUNC_DEFINE(shutdown);

#define DEV_CFG(dev) ((const struct udc_dwc3_config *)(dev->config))
#define DEV_DATA(dev) ((struct udc_dwc3_data *)udc_get_private(dev))

static int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr);

/* Shut down the controller completely  */
static int udc_dwc3_shutdown(const struct device *const dev)
{
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	return 0;
}

static void udc_dwc3_lock(const struct device *const dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_dwc3_unlock(const struct device *const dev)
{
	udc_unlock_internal(dev);
}

/*
 * Ring buffer
 *
 * Helpers to operate the TRB and event ring buffers, shared with the hardware.
 */

/* Sentinel net_buf pointer for driver-owned terminating ZLP TRBs */
#define UDC_DWC3_ZLP_TRB_MARKER		((struct net_buf *)UINTPTR_MAX)

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
static void *udc_dwc3_net_buf_data(const struct net_buf *const buf)
{
	if (buf == NULL || buf == UDC_DWC3_ZLP_TRB_MARKER) {
		return NULL;
	}

	return buf->data;
}
#endif

static struct net_buf *udc_dwc3_pop_trb(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data);
static void udc_dwc3_on_xfer_done_norm(const struct device *const dev,
				       const uint32_t evt);
static bool udc_dwc3_retire_sw_done(const struct device *const dev,
				    struct udc_dwc3_ep_data *const ep_data,
				    const char *const via);
static void udc_dwc3_ep_ring_reset(struct udc_dwc3_ep_data *const ep_data);
static void udc_dwc3_ep_link_trb_init(struct udc_dwc3_ep_data *const ep_data);
static bool udc_dwc3_trb_hwo(const volatile struct udc_dwc3_trb *const trb);
static void udc_dwc3_trb_commit(volatile struct udc_dwc3_trb *const trb,
				const uint32_t addr_lo, const uint32_t addr_hi,
				const uint32_t status, const uint32_t ctrl);
static bool udc_dwc3_link_trb_valid(const struct udc_dwc3_ep_data *const ep_data);

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
static void udc_dwc3_xfer_trace(const char *const tag,
				struct udc_dwc3_ep_data *const ep_data,
				const char *const detail);
static void udc_dwc3_link_check(struct udc_dwc3_ep_data *const ep_data,
				const char *const where);
static bool udc_dwc3_link_trb_hwo(const struct udc_dwc3_ep_data *const ep_data);
#endif

/* Increment the counter position "nump" of the TRBs/event ring buffer) */
void udc_dwc3_ring_inc(uint32_t *const nump, const uint32_t size)
{
	uint32_t num = *nump + 1;

	*nump = (num >= size) ? 0 : num;
}

static void udc_dwc3_push_trb(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data,
			      struct net_buf *const buf, const uint32_t ctrl)
{
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->head];

	/* If the next TRB in the chain is still owned by the hardware, need
	 * to retry later when more resources become available.
	 */
	__ASSERT_NO_MSG(!ep_data->full);

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (ep_data->cfg.addr == 0x82 || ep_data->cfg.addr == 0x01) {
		const uint32_t head_idx = ep_data->head;
		const uint32_t link_idx = CONFIG_UDC_DWC3_TRB_NUM - 1U;

#if defined(CONFIG_UDC_DWC3_DMA_SLOT_DIAG)
		const int data_slot = udc_dwc3_dma_slot_index((uintptr_t)buf->data);

		for (uint32_t i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
			const int trb_slot = udc_dwc3_dma_slot_index(
				(uintptr_t)&ep_data->trb_buf[i]);

			if (data_slot >= 0 && data_slot == trb_slot) {
				udc_dwc3_xfer_trace("SLOT-COLLIDE", ep_data,
						    "data trb share slot");
				break;
			}
		}
#endif
		if (head_idx == link_idx - 1U) {
			const uint32_t prev = head_idx == 0U ? link_idx - 1U : head_idx - 1U;

			if (udc_dwc3_trb_hwo(&ep_data->trb_buf[prev])) {
				udc_dwc3_xfer_trace("WRAP-HAZ", ep_data,
						    "prev slot still hwo at wrap");
			}
		}
	}
#endif

	/* Associate an active buffer and a TRB together */
	ep_data->net_buf[ep_data->head] = buf;

	/* TRB# with one more chunk of data */
	udc_dwc3_trb_commit(trb, LO32((uintptr_t)buf->data),
			    HI32((uintptr_t)buf->data),
			    USB_EP_DIR_IS_IN(ep_data->cfg.addr) ? buf->len : buf->size,
			    ctrl);

	LOG_DBG("PUSH %u buf %p, data %p, size %u",
		ep_data->head, (void *)buf, (void *)buf->data, buf->size);

	/* Shift the head */
	udc_dwc3_ring_inc(&ep_data->head, CONFIG_UDC_DWC3_TRB_NUM - 1);

	/* If the head touches the tail after we add something, we are full */
	ep_data->full = (ep_data->head == ep_data->tail);

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	udc_dwc3_link_check(ep_data, "post-push");
#endif
}

static uint32_t udc_dwc3_link_trb_idx(void)
{
	return CONFIG_UDC_DWC3_TRB_NUM - 1U;
}

static uint32_t udc_dwc3_ring_data_hwo_mask(const struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t li = udc_dwc3_link_trb_idx();
	uint32_t mask = 0U;

	for (uint32_t i = 0; i < li; i++) {
		if (ep_data->trb_buf[i].ctrl & UDC_DWC3_TRB_CTRL_HWO) {
			mask |= BIT(i);
		}
	}

	return mask;
}

static bool udc_dwc3_link_trb_valid(const struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t li = udc_dwc3_link_trb_idx();
	const volatile struct udc_dwc3_trb *const link = &ep_data->trb_buf[li];

	return (link->ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK) ==
		       UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB &&
	       link->addr_lo == LO32((uintptr_t)ep_data->trb_buf) &&
	       link->addr_hi == HI32((uintptr_t)ep_data->trb_buf);
}

/*
 * After an LST-terminated IN xfer the HW resource ends (DepStart required) but
 * the ring may be quiescent (head==tail, no HWO data TRBs).  Reposition to [0]
 * instead of wiping the whole ring + re-init link every MPS-aligned packet.
 */
static void udc_dwc3_bulk_ring_restart(struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t hwo_mask = udc_dwc3_ring_data_hwo_mask(ep_data);

	if (ep_data->head != ep_data->tail || hwo_mask != 0U ||
	    ep_data->chain_buf != NULL) {
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		udc_dwc3_xfer_trace("RINGRST-FULL", ep_data, "dirty ring");
#endif
		udc_dwc3_ep_ring_reset(ep_data);
		return;
	}

	ep_data->head = ep_data->tail = 0U;
	ep_data->full = false;

	if (!udc_dwc3_link_trb_valid(ep_data)) {
		udc_dwc3_ep_link_trb_init(ep_data);
	}
}

static bool udc_dwc3_link_trb_hwo(const struct udc_dwc3_ep_data *const ep_data)
{
	return !!(ep_data->trb_buf[udc_dwc3_link_trb_idx()].ctrl &
		  UDC_DWC3_TRB_CTRL_HWO);
}

/*
 * Re-init the link TRB only when the slot is fully invalid (cleared/corrupt).
 * Do NOT touch it when valid but HWO=0 — HW may be consuming the link TRB
 * during a ring wrap; rewriting it there wedged OUT at lap boundaries.
 */
static void udc_dwc3_link_trb_ensure(struct udc_dwc3_ep_data *const ep_data)
{
	if (udc_dwc3_link_trb_valid(ep_data)) {
		return;
	}

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	udc_dwc3_xfer_trace("LINK-ENSURE", ep_data, "reinit link trb");
#endif
	udc_dwc3_ep_link_trb_init(ep_data);
}

static void udc_dwc3_trb_clear(volatile struct udc_dwc3_trb *const trb)
{
	trb->addr_lo = 0U;
	trb->addr_hi = 0U;
	trb->status = 0U;
	trb->ctrl = 0U;
	barrier_dmem_fence_full();
}

/*
 * DWC3 databook: addr/status must be visible before HWO is set (Linux wmb()).
 */
static void udc_dwc3_trb_commit(volatile struct udc_dwc3_trb *const trb,
				const uint32_t addr_lo, const uint32_t addr_hi,
				const uint32_t status, const uint32_t ctrl)
{
	trb->addr_lo = addr_lo;
	trb->addr_hi = addr_hi;
	trb->status = status;
	trb->ctrl = ctrl & ~UDC_DWC3_TRB_CTRL_HWO;
	barrier_dmem_fence_full();
	if ((ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
		trb->ctrl = ctrl;
		barrier_dmem_fence_full();
	}
}

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
static bool udc_dwc3_acm_diag_ep(const uint8_t addr)
{
	return addr == 0x82 || addr == 0x01;
}

/* ACM OUT (0x01 / epn=2) completion accounting — validate silent HWO clears. */
static uint32_t udc_dwc3_out_count_orphans(const struct udc_dwc3_ep_data *const ep_data)
{
	uint32_t n = 0U;

	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_TRB_NUM - 1U; i++) {
		if (ep_data->net_buf[i] != NULL &&
		    !(ep_data->trb_buf[i].ctrl & UDC_DWC3_TRB_CTRL_HWO)) {
			n++;
		}
	}

	return n;
}

struct udc_dwc3_trb_snap {
	uint32_t ctrl;
	uint32_t status;
	uint32_t addr_lo;
	uint32_t addr_hi;
};

static void udc_dwc3_trb_snap_read(const volatile struct udc_dwc3_trb *trb,
				   struct udc_dwc3_trb_snap *snap)
{
	barrier_dmem_fence_full();
	snap->ctrl = trb->ctrl;
	snap->status = trb->status;
	snap->addr_lo = trb->addr_lo;
	snap->addr_hi = trb->addr_hi;
	barrier_dmem_fence_full();
}

static bool udc_dwc3_trb_snap_equal(const struct udc_dwc3_trb_snap *a,
				    const struct udc_dwc3_trb_snap *b)
{
	return a->ctrl == b->ctrl && a->status == b->status &&
	       a->addr_lo == b->addr_lo && a->addr_hi == b->addr_hi;
}

static void udc_dwc3_trb_snap_pack(const struct udc_dwc3_trb_snap *const snap,
				   uint8_t out[16])
{
	const uint32_t words[4] = { snap->addr_lo, snap->addr_hi, snap->status, snap->ctrl };

	memcpy(out, words, sizeof(words));
}

static void udc_dwc3_expected_trb_snap(const struct udc_dwc3_ep_data *const ep_data,
				       const uint32_t idx,
				       struct udc_dwc3_trb_snap *const snap)
{
	const uint32_t li = CONFIG_UDC_DWC3_TRB_NUM - 1U;
	struct net_buf *const buf = ep_data->net_buf[idx];

	snap->addr_lo = 0U;
	snap->addr_hi = 0U;
	snap->status = 0U;
	snap->ctrl = 0U;

	if (idx == li) {
		snap->addr_lo = LO32((uintptr_t)ep_data->trb_buf);
		snap->addr_hi = HI32((uintptr_t)ep_data->trb_buf);
		snap->ctrl = UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB;
		if (ep_data->xfer_active) {
			snap->ctrl |= UDC_DWC3_TRB_CTRL_HWO;
		}
		return;
	}

	if (buf == NULL) {
		return;
	}

	snap->addr_lo = LO32((uintptr_t)buf->data);
	snap->addr_hi = HI32((uintptr_t)buf->data);
	snap->status = USB_EP_DIR_IS_OUT(ep_data->cfg.addr) ? buf->size : buf->len;
	snap->ctrl = UDC_DWC3_TRB_CTRL_IOC | UDC_DWC3_TRB_CTRL_CSP |
		     UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL;
	if (idx == ep_data->tail) {
		snap->ctrl |= UDC_DWC3_TRB_CTRL_HWO;
	}
}

static void udc_dwc3_trb_cmp_log_slot(const char *const where, const uint32_t idx,
				      const char *const mark, const uintptr_t meta,
				      struct net_buf *const buf,
				      const struct udc_dwc3_trb_snap *const expect,
				      const struct udc_dwc3_trb_snap *const actual,
				      const bool verbose)
{
	const bool ok = udc_dwc3_trb_snap_equal(expect, actual);
	const char *addr_mark = expect->addr_lo == actual->addr_lo ? "" : " *";
	const char *sts_mark = expect->status == actual->status ? "" : " *";
	const char *ctl_mark = expect->ctrl == actual->ctrl ? "" : " *";

	if (ok && !verbose) {
		LOG_ERR("TRB-CMP %s [%u]%s meta@0x%08x buf=%p data=%p ok",
			where, idx, mark, (uint32_t)meta, (void *)buf,
			udc_dwc3_net_buf_data(buf));
		return;
	}

	LOG_ERR("TRB-CMP %s [%u]%s meta@0x%08x buf=%p data=%p %s",
		where, idx, mark, (uint32_t)meta, (void *)buf,
		udc_dwc3_net_buf_data(buf), ok ? "ok" : "MISMATCH");
	LOG_ERR("         addr   exp=0x%08x act=0x%08x%s",
		expect->addr_lo, actual->addr_lo, addr_mark);
	LOG_ERR("         status exp=0x%08x act=0x%08x%s",
		expect->status, actual->status, sts_mark);
	LOG_ERR("         ctrl   exp=0x%08x act=0x%08x%s",
		expect->ctrl, actual->ctrl, ctl_mark);

	if (verbose) {
		uint8_t exp_bytes[16];
		uint8_t act_bytes[16];
		char exp_line[4 * 16 + 1];
		char act_line[4 * 16 + 1];
		size_t pos;

		udc_dwc3_trb_snap_pack(expect, exp_bytes);
		udc_dwc3_trb_snap_pack(actual, act_bytes);

		pos = 0;
		for (size_t b = 0; b < sizeof(exp_bytes); b++) {
			pos += snprintk(&exp_line[pos], sizeof(exp_line) - pos, "%02x ",
					exp_bytes[b]);
		}

		pos = 0;
		for (size_t b = 0; b < sizeof(act_bytes); b++) {
			pos += snprintk(&act_line[pos], sizeof(act_line) - pos, "%02x ",
					act_bytes[b]);
		}

		exp_line[sizeof(exp_line) - 1] = '\0';
		act_line[sizeof(act_line) - 1] = '\0';
		LOG_ERR("         EXP %s", exp_line);
		LOG_ERR("         ACT %s", act_line);
	}
}

static bool udc_dwc3_diag_verbose(atomic_t *const seq)
{
	const uint32_t n = (uint32_t)atomic_inc(seq);

	return n <= UDC_DWC3_DIAG_LOG_FIRST || (n % UDC_DWC3_DIAG_LOG_EVERY) == 0U;
}

static void udc_dwc3_out_acct_log(const char *const tag,
				  const struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t push = (uint32_t)atomic_get(&udc_dwc3_out_acct_push);
	const uint32_t evt = (uint32_t)atomic_get(&udc_dwc3_out_acct_evt);
	const uint32_t pop = (uint32_t)atomic_get(&udc_dwc3_out_acct_pop);
	const uint32_t silent = (uint32_t)atomic_get(&udc_dwc3_out_acct_silent);
	const uint32_t defer = (uint32_t)atomic_get(&udc_dwc3_out_acct_defer_hwo);
	const uint32_t orphans = ep_data != NULL ? udc_dwc3_out_count_orphans(ep_data) : 0U;

	LOG_WRN("OUT-ACCT %s push=%u evt=%u pop=%u silent=%u defer_hwo=%u "
		"inflight=%u evt_gap=%u orphans=%u hwirq=%u evt_total=%u",
		tag, push, evt, pop, silent, defer,
		push > pop ? push - pop : 0U,
		pop > evt ? pop - evt : 0U,
		orphans,
		(uint32_t)atomic_get(&udc_dwc3_hwirq_count),
		(uint32_t)atomic_get(&udc_dwc3_evt_count));

	if (ep_data != NULL) {
		LOG_WRN("OUT-ACCT %s ring head=%u tail=%u hwo_mask=0x%x",
			tag, ep_data->head, ep_data->tail,
			udc_dwc3_ring_data_hwo_mask(ep_data));
	}
}

static void udc_dwc3_out_acct_note_push(struct udc_dwc3_ep_data *const ep_data)
{
	if (ep_data->cfg.addr != 0x01) {
		return;
	}

	atomic_inc(&udc_dwc3_out_acct_push);

	const uint32_t orphans = udc_dwc3_out_count_orphans(ep_data);

	if (orphans > 0U) {
		const uint32_t tail = ep_data->tail;
		const volatile struct udc_dwc3_trb *const t = &ep_data->trb_buf[tail];

		LOG_WRN("OUT-ORPHAN pre-push n=%u tail=%u tail_hwo=%d tail_buf=%p",
			orphans, tail,
			!!(t->ctrl & UDC_DWC3_TRB_CTRL_HWO),
			(void *)ep_data->net_buf[tail]);
		udc_dwc3_out_acct_log("orphan-pre-push", ep_data);
	}
}

static void udc_dwc3_out_acct_note_silent(const struct device *const dev,
					  struct udc_dwc3_ep_data *const ep_data,
					  const char *const via)
{
	if (ep_data->cfg.addr != 0x01) {
		return;
	}

	atomic_inc(&udc_dwc3_out_acct_silent);

	const uint32_t tail = ep_data->tail;
	const volatile struct udc_dwc3_trb *const t = &ep_data->trb_buf[tail];
	const uint32_t silent_total = (uint32_t)atomic_get(&udc_dwc3_out_acct_silent);

	if (udc_dwc3_diag_verbose(&udc_dwc3_out_silent_log_seq)) {
		LOG_WRN("OUT-SILENT SW-RETIRE tail=%u via=%s hwo=%d sts=0x%08x buf=%p "
			"(silent_total=%u)",
			tail, via,
			!!(t->ctrl & UDC_DWC3_TRB_CTRL_HWO),
			t->status, (void *)ep_data->net_buf[tail], silent_total);
		udc_dwc3_out_acct_log("silent", ep_data);
	}

	ARG_UNUSED(dev);
}

static int udc_dwc3_link_mimics_data_idx(const struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t li = udc_dwc3_link_trb_idx();
	const volatile struct udc_dwc3_trb *const link = &ep_data->trb_buf[li];

	for (uint32_t i = 0; i < li; i++) {
		const volatile struct udc_dwc3_trb *const data = &ep_data->trb_buf[i];

		if (data->ctrl == link->ctrl && data->addr_lo == link->addr_lo &&
		    data->addr_hi == link->addr_hi && data->status == link->status) {
			return (int)i;
		}
	}

	return -1;
}

static void udc_dwc3_log_trb_ring(const struct udc_dwc3_ep_data *const ep_data,
				  const char *const tag)
{
	LOG_WRN("XFER-TRACE RING %s ep=0x%02x head=%u tail=%u full=%d active=%d "
		"ring=0x%08x xferrscidx=0x%x chain=%p",
		tag, ep_data->cfg.addr, ep_data->head, ep_data->tail,
		ep_data->full, ep_data->xfer_active,
		(uint32_t)(uintptr_t)ep_data->trb_buf, ep_data->xferrscidx,
		(void *)ep_data->chain_buf);

	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		const volatile struct udc_dwc3_trb *const t = &ep_data->trb_buf[i];
		const uint32_t trbctl = FIELD_GET(UDC_DWC3_TRB_CTRL_TRBCTL_MASK, t->ctrl);
		const bool hwo = !!(t->ctrl & UDC_DWC3_TRB_CTRL_HWO);
		const char *mark = "";

		if (i == ep_data->head) {
			mark = " HEAD";
		} else if (i == ep_data->tail) {
			mark = ep_data->full ? " TAIL+FULL" : " TAIL";
		}

		LOG_WRN("  [%u]%s ctl=0x%08x sts=0x%08x addr=0x%08x hwo=%d typ=%u "
			"buf=%p data=%p",
			i, mark, t->ctrl, t->status, t->addr_lo, hwo, trbctl,
			(void *)ep_data->net_buf[i],
			udc_dwc3_net_buf_data(ep_data->net_buf[i]));
	}
}

static void udc_dwc3_link_check(struct udc_dwc3_ep_data *const ep_data,
				const char *const where)
{
	const uint32_t li = udc_dwc3_link_trb_idx();
	const volatile struct udc_dwc3_trb *const link = &ep_data->trb_buf[li];
	if (!udc_dwc3_acm_diag_ep(ep_data->cfg.addr) || udc_dwc3_link_trb_valid(ep_data)) {
		return;
	}

	const int mimic = udc_dwc3_link_mimics_data_idx(ep_data);

	udc_dwc3_xfer_trace("LINK-BAD", ep_data, where);
	LOG_WRN("XFER-TRACE LINK-DETAIL @%s ep=0x%02x [%u] ctl=0x%08x sts=0x%08x "
		"addr=0x%08x expect=0x%08x mimic=%d",
		where, ep_data->cfg.addr, li, link->ctrl, link->status, link->addr_lo,
		LO32((uintptr_t)ep_data->trb_buf), mimic);

#if defined(CONFIG_UDC_DWC3_DMA_SLOT_DIAG)
	{
		const int link_slot = udc_dwc3_dma_slot_index((uintptr_t)&ep_data->trb_buf[li]);

		if (link_slot >= 0) {
			const struct udc_dwc3_dma_slot *const slot =
				&udc_dwc3_dma_slots[link_slot];

			LOG_WRN("XFER-TRACE LINK-SLOT [%d] last_ep=0x%02x last=%s "
				"out_enq=%u in_enq=%u out_done=%u in_done=%u",
				link_slot, slot->last_ep, udc_dwc3_dma_op_str(slot->last_op),
				slot->out_enq, slot->in_enq, slot->out_done, slot->in_done);
		}
	}
#endif

	udc_dwc3_log_trb_ring(ep_data, where);
}

static void udc_dwc3_trace_sram_cross(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data,
				      void *const data, const char *const where)
{
#if defined(CONFIG_UDC_DWC3_DMA_SLOT_DIAG)
	const struct udc_dwc3_config *const cfg = dev->config;
	const int data_slot = udc_dwc3_dma_slot_index((uintptr_t)data);
	const uint32_t li = udc_dwc3_link_trb_idx();
	int i;

	if (!udc_dwc3_acm_diag_ep(ep_data->cfg.addr) || data_slot < 0) {
		return;
	}

	for (i = 0; i < cfg->num_in_eps; i++) {
		struct udc_dwc3_ep_data *const other = &cfg->ep_data_in[i];

		if (other->trb_buf == NULL || other == ep_data) {
			continue;
		}

		if (udc_dwc3_dma_slot_index((uintptr_t)other->trb_buf) == data_slot ||
		    udc_dwc3_dma_slot_index((uintptr_t)&other->trb_buf[li]) == data_slot) {
			LOG_WRN("XFER-TRACE SRAM-XFER @%s ep=0x%02x data_slot=%d "
				"hits ep=0x%02x ring/link",
				where, ep_data->cfg.addr, data_slot, other->cfg.addr);
		}
	}

	for (i = 0; i < cfg->num_out_eps; i++) {
		struct udc_dwc3_ep_data *const other = &cfg->ep_data_out[i];

		if (other->trb_buf == NULL || other == ep_data) {
			continue;
		}

		if (udc_dwc3_dma_slot_index((uintptr_t)other->trb_buf) == data_slot ||
		    udc_dwc3_dma_slot_index((uintptr_t)&other->trb_buf[li]) == data_slot) {
			LOG_WRN("XFER-TRACE SRAM-XFER @%s ep=0x%02x data_slot=%d "
				"hits ep=0x%02x ring/link",
				where, ep_data->cfg.addr, data_slot, other->cfg.addr);
		}
	}
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(ep_data);
	ARG_UNUSED(data);
	ARG_UNUSED(where);
#endif
}

static void udc_dwc3_trace_out_push_haz(struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t tail = ep_data->tail;
	const volatile struct udc_dwc3_trb *const tail_trb = &ep_data->trb_buf[tail];
	const uint32_t hwo_mask = udc_dwc3_ring_data_hwo_mask(ep_data);
	const unsigned hwo_n = POPCOUNT(hwo_mask);

	if (tail_trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) {
		udc_dwc3_xfer_trace("OUT-TAIL-HWO", ep_data, "push while tail owned");
		LOG_WRN("XFER-TRACE OUT-TAIL-HWO tail=%u ctl=0x%08x sts=0x%08x "
			"hwo_n=%u mask=0x%x",
			tail, tail_trb->ctrl, tail_trb->status, hwo_n, hwo_mask);
		udc_dwc3_log_trb_ring(ep_data, "out-tail-hwo");
	} else if (hwo_n > 1U) {
		LOG_WRN("XFER-TRACE OUT-RING-HWO hwo_n=%u mask=0x%x head=%u tail=%u",
			hwo_n, hwo_mask, ep_data->head, ep_data->tail);
	}
}

static void udc_dwc3_trace_ring_reset_haz(struct udc_dwc3_ep_data *const ep_data,
					  const char *const where)
{
	const uint32_t hwo_mask = udc_dwc3_ring_data_hwo_mask(ep_data);

	if (!udc_dwc3_acm_diag_ep(ep_data->cfg.addr)) {
		return;
	}

	if (hwo_mask != 0U || ep_data->xfer_active || udc_dwc3_link_trb_hwo(ep_data)) {
		udc_dwc3_xfer_trace("RINGRST-HWO", ep_data, where);
		LOG_WRN("XFER-TRACE RINGRST-HWO @%s ep=0x%02x hwo_mask=0x%x active=%d",
			where, ep_data->cfg.addr, hwo_mask, ep_data->xfer_active);
		udc_dwc3_log_trb_ring(ep_data, where);
	}
}

static void udc_dwc3_xfer_trace(const char *const tag,
				struct udc_dwc3_ep_data *const ep_data,
				const char *const detail)
{
	LOG_WRN("XFER-TRACE %s ep=0x%02x active=%d head=%u tail=%u full=%d "
		"link_hwo=%d skip=%u chain=%p %s",
		tag, ep_data->cfg.addr, ep_data->xfer_active, ep_data->head,
		ep_data->tail, ep_data->full, udc_dwc3_link_trb_hwo(ep_data),
		ep_data->skip_xfer_done_count, (void *)ep_data->chain_buf,
		detail != NULL ? detail : "");
}

static void udc_dwc3_trace_xfer_inactive(struct udc_dwc3_ep_data *const ep_data,
					 const char *const why)
{
	if (!ep_data->xfer_active && udc_dwc3_link_trb_hwo(ep_data) &&
	    udc_dwc3_ring_data_hwo_mask(ep_data) != 0U) {
		udc_dwc3_xfer_trace("XFEREND-LINK", ep_data, why);
	}

	udc_dwc3_link_check(ep_data, why);
}
#endif /* CONFIG_UDC_DWC3_XFER_TRACE */

static bool udc_dwc3_try_retire_chained_zlp(const struct device *const dev,
					    struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *zlp_trb;

	ARG_UNUSED(dev);

	if (ep_data->chain_buf == NULL) {
		return false;
	}

	if (ep_data->net_buf[ep_data->tail] != UDC_DWC3_ZLP_TRB_MARKER) {
		return false;
	}

	zlp_trb = &ep_data->trb_buf[ep_data->tail];
	if (zlp_trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) {
		return false;
	}

	udc_dwc3_pop_trb(dev, ep_data);
	ep_data->absorb_cdc_zlp = true;
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		ep_data->xfer_active = false;
	}

	LOG_DBG("ZLP-SWRET ep=0x%02x", ep_data->cfg.addr);
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	udc_dwc3_xfer_trace("ZLP-SWRET", ep_data, "sw-retired ZLP");
#endif

	return true;
}

static struct net_buf *udc_dwc3_pop_trb(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t retired = ep_data->tail;
	struct net_buf *const buf = ep_data->net_buf[retired];

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (udc_dwc3_acm_diag_ep(ep_data->cfg.addr)) {
		const uint32_t tail = ep_data->tail;
		const volatile struct udc_dwc3_trb *const retiring = &ep_data->trb_buf[tail];

		udc_dwc3_link_check(ep_data, "pre-pop");
		if (!udc_dwc3_link_trb_valid(ep_data)) {
			LOG_WRN("XFER-TRACE POP-RETIRE ep=0x%02x tail=%u ctl=0x%08x "
				"sts=0x%08x addr=0x%08x buf=%p",
				ep_data->cfg.addr, tail, retiring->ctrl, retiring->status,
				retiring->addr_lo, (void *)buf);
		}
	}
#endif

	/* Clear the retired TRB slot */
	ep_data->net_buf[retired] = NULL;
	udc_dwc3_trb_clear(&ep_data->trb_buf[retired]);

	/* Move to the next position in the ring buffer */
	udc_dwc3_ring_inc(&ep_data->tail, CONFIG_UDC_DWC3_TRB_NUM - 1);

	if (buf == NULL) {
		LOG_ERR("pop: the next TRB is emtpy");
		return NULL;
	}

	LOG_DBG("POP %u EP 0x%02x, buf %p, data %p",
		ep_data->tail, ep_data->cfg.addr, (void *)buf,
		buf == UDC_DWC3_ZLP_TRB_MARKER ? NULL : (void *)buf->data);

	/* If we just pulled a TRB, we know we made one hole and we are not full anymore */
	ep_data->full = false;

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	udc_dwc3_link_check(ep_data, "post-pop");
	if (ep_data->cfg.addr == 0x01) {
		atomic_inc(&udc_dwc3_out_acct_pop);
	}
#endif

	return buf;
}

static bool udc_dwc3_trb_hwo(const volatile struct udc_dwc3_trb *const trb)
{
	return !!(trb->ctrl & UDC_DWC3_TRB_CTRL_HWO);
}

/*
 * Refuse to enqueue when the head slot is still owned by HW or already
 * associated with a buffer (stale ctrl after ring wrap).
 */
static int udc_dwc3_ring_push_guard(struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t head = ep_data->head;
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[head];

	if (ep_data->net_buf[head] != NULL) {
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		udc_dwc3_xfer_trace("PUSH-BUSY", ep_data, "head net_buf set");
#endif
		return -EBUSY;
	}

	if (udc_dwc3_trb_hwo(trb)) {
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		udc_dwc3_xfer_trace("PUSH-HWO", ep_data, "head slot HWO");
#endif
		return -EBUSY;
	}

	return 0;
}

/*
 * Commands
 *
 * The DEPCMD register acts as a command interface, where a command number
 * is written along with parameters, an action is performed and a CMDACT bit
 * is reset whenever the command completes.
 */

/*
 * Issue an endpoint command and wait for CMDACT to clear.  Returns the transfer
 * resource index from the completion word; when cmderr is non-NULL it is set to
 * true if the controller rejected the command (CMDERR).  Callers that re-issue a
 * transfer doorbell use this to tell an accepted-but-unfetched command (dropped
 * doorbell, retry) apart from a rejected one (e.g. StartXfer on an already
 * running transfer, back off).
 */
static uint32_t udc_dwc3_depcmd_issue(const struct device *const dev,
				      const uint32_t addr, const uint32_t cmd,
				      bool *const cmderr)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;
	bool err = false;

	sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);
	do {
		reg = sys_read32(base + addr);
	} while ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0);

	switch (reg & UDC_DWC3_DEPCMD_STATUS_MASK) {
	case UDC_DWC3_DEPCMD_STATUS_OK:
		break;
	case UDC_DWC3_DEPCMD_STATUS_CMDERR:
		err = true;
		break;
	default:
		err = true;
		break;
	}

	if (cmderr != NULL) {
		*cmderr = err;
	}

	return FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
}

static uint32_t udc_dwc3_depcmd_status(const struct device *const dev,
				       const uint32_t addr, const uint32_t cmd,
				       bool *const cmderr)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	bool err = false;
	uint32_t reg;
	const uint32_t rscidx = udc_dwc3_depcmd_issue(dev, addr, cmd, &err);

	if (err) {
		reg = sys_read32(base + addr);
		switch (reg & UDC_DWC3_DEPCMD_STATUS_MASK) {
		case UDC_DWC3_DEPCMD_STATUS_CMDERR:
			LOG_ERR("endpoint command failed epn=%u cmd=0x%08x depcmd=0x%08x",
				(addr - UDC_DWC3_DEPCMD(0)) / 16U, cmd, reg);
			break;
		default:
			LOG_ERR("command failed with unknown status: 0x%08x", reg);
			break;
		}
	}

	if (cmderr != NULL) {
		*cmderr = err;
	}

	return rscidx;
}

static uint32_t udc_dwc3_depcmd(const struct device *const dev,
				const uint32_t addr, const uint32_t cmd)
{
	return udc_dwc3_depcmd_status(dev, addr, cmd, NULL);
}

static void udc_dwc3_depcmd_ep_config(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t param0 = 0;
	uint32_t param1 = 0;
	uint32_t burst = 0U;

	LOG_INF("configuring endpoint 0x%02x with wMaxPacketSize=%u",
		ep_data->cfg.addr, ep_data->cfg.mps);

	if (ep_data->cfg.stat.enabled) {
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MODIFY;
	} else {
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_INIT;
	}

	switch (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_CONTROL:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL;
		break;
	case USB_EP_TYPE_BULK:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_BULK;
		burst = 15U;
		break;
	case USB_EP_TYPE_INTERRUPT:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_INT;
		break;
	case USB_EP_TYPE_ISO:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_ISOC;
		burst = 15U;
		break;
	default:
		CODE_UNREACHABLE;
	}

	/* Max Packet Size according to the USB descriptor configuration */
	param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_MPS_MASK, ep_data->cfg.mps);

	/*
	 * BRSTSIZ has to agree with the bMaxBurst the endpoint companion
	 * descriptor advertised, otherwise the controller bursts more packets
	 * than the host agreed to accept.  The classes advertise 15 on bulk and
	 * isochronous endpoints and 0 on control and interrupt ones.
	 */
	param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, burst);

	/* Set the FIFO number, must be 0 for all OUT EPs */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK,
				     ep_data->cfg.addr & 0x7f);
	}

	/* Per-endpoint events */
	param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
	param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN;

#if defined(CONFIG_UDC_DWC3_OUT_NOTREADY_RETAKE)
	/*
	 * XferNotReady on a bulk OUT endpoint means the host tried to send and
	 * HW had no transfer ready to take it.  That is the only direct evidence
	 * the controller offers that a StartXfer doorbell went missing, and
	 * without it a wedged OUT pipe is indistinguishable from an idle one --
	 * leaving a timer as the only detector, which cannot tell the two apart.
	 */
	if (!USB_EP_DIR_IS_IN(ep_data->cfg.addr) &&
	    (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_BULK) {
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
	}
#endif

	/* This is the usb protocol endpoint number, but the data encoding
	 * we chose for physical endpoint number is the same as this
	 * register
	 */
	param1 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK, ep_data->epn);

	sys_write32(param0, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(param1, base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPCFG);
}

static void udc_dwc3_depcmd_ep_xfer_config(const struct device *const dev,
					   struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	LOG_DBG("DepXferConfig: ep=0x%02x", ep_data->cfg.addr);

	reg = FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK, 1);
	sys_write32(reg, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPXFERCFG);
}

static void udc_dwc3_depcmd_set_stall(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	LOG_WRN("DepSetStall: ep=0x%02x", ep_data->cfg.addr);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSETSTALL);
}

static void udc_dwc3_depcmd_clear_stall(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	bool cmderr = false;

	udc_dwc3_depcmd_status(dev, UDC_DWC3_DEPCMD(ep_data->epn),
			       UDC_DWC3_DEPCMD_DEPCSTALL, &cmderr);
	if (cmderr) {
		LOG_DBG("DepClearStall CMDERR ep=0x%02x (endpoint was not stalled)",
			ep_data->cfg.addr);
	} else {
		LOG_INF("DepClearStall ep=0x%02x", ep_data->cfg.addr);
	}
}

static void udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags);

static void udc_dwc3_depcmd_update_xfer(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data);

static void udc_dwc3_depcmd_start_xfer_trb(const struct device *const dev,
					   struct udc_dwc3_ep_data *const ep_data,
					   volatile struct udc_dwc3_trb *const start)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	/* REMOTEWAKEUP from U0 is illegal and can drop the link to SS.Inactive */
	reg = sys_read32(base + UDC_DWC3_DSTS);
	if ((reg & UDC_DWC3_DSTS_CONNECTSPD_MASK) == UDC_DWC3_DSTS_CONNECTSPD_SS) {
		const uint32_t lnkst = reg & UDC_DWC3_DSTS_USBLNKST_MASK;

		if (lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U1 ||
		    lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U2 ||
		    lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U3) {
			atomic_inc(&udc_dwc3_remwk_count);
			reg = sys_read32(base + UDC_DWC3_DCTL);
			reg &= ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;
			reg |= UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP;
			sys_write32(reg, base + UDC_DWC3_DCTL);
		}
	}

	sys_write32(HI32((uintptr_t)start), base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)start), base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	ep_data->xferrscidx =
		udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSTRTXFER);

	LOG_DBG("DepStartXfer done ep=0x%02x xferrscidx=0x%x",
		ep_data->cfg.addr, ep_data->xferrscidx);

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (udc_dwc3_acm_diag_ep(ep_data->cfg.addr)) {
		udc_dwc3_xfer_trace("DEPSTART", ep_data, "DepStartXfer");
		udc_dwc3_link_check(ep_data, "post-DepStartXfer");
	}
#endif
}

static void udc_dwc3_depcmd_start_xfer(const struct device *const dev,
				       struct udc_dwc3_ep_data *const ep_data)
{
	udc_dwc3_depcmd_start_xfer_trb(dev, ep_data, ep_data->trb_buf);
}

/*
 * OUT run-dry re-arm.  When the ring drains and the controller parks on a
 * CPU-owned TRB, the databook resume is DepUpdateXfer — but on this IP an
 * UpdateXfer landing in the park transition window is silently dropped: CMDACT
 * clears (command "succeeds") yet HW never fetches the owned TRB.  With the UVC
 * IN event stream RTL-masked there is no later event to retry on, so the
 * endpoint wedges permanently (confirmed on-core: one HWO=1 TRB at tail,
 * GEVNTCOUNT=0, unmasked).  A re-issued UpdateXfer from a settled state always
 * takes (verified on-core), so issue it and then confirm HW actually consumed
 * the owned tail TRB; if the TRB is still owned after a short settle the
 * doorbell was dropped, so re-issue.  Under the continuous host flood a fetched
 * OUT TRB is filled and released within microseconds, which distinguishes a
 * dropped doorbell from a TRB that is legitimately armed and awaiting data;
 * re-issuing UpdateXfer against a running transfer is harmless (idempotent
 * ring re-scan), so an over-issue in the latter case does no damage.
 *
 * EndXfer+StartXfer is deliberately NOT used: on this IP it leaves the endpoint
 * in a state from which even a manual UpdateXfer cannot recover (observed
 * on-core), i.e. it corrupts rather than restarts the transfer.
 */
#define UDC_DWC3_RUNDRY_REARM_RETRIES  8U
#define UDC_DWC3_RUNDRY_SETTLE_STEPS   16U
#define UDC_DWC3_RUNDRY_SETTLE_US      50U

static void udc_dwc3_out_rundry_restart(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t tail = ep_data->tail;

	for (unsigned int attempt = 0U; attempt < UDC_DWC3_RUNDRY_REARM_RETRIES; attempt++) {
		udc_dwc3_depcmd_update_xfer(dev, ep_data);

		for (unsigned int step = 0U; step < UDC_DWC3_RUNDRY_SETTLE_STEPS; step++) {
			k_busy_wait(UDC_DWC3_RUNDRY_SETTLE_US);
			if (!udc_dwc3_trb_hwo(&ep_data->trb_buf[tail])) {
				/* HW fetched and released the TRB: doorbell took. */
				return;
			}
		}
	}

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	udc_dwc3_xfer_trace("OUT-RUNDRY-REARM", ep_data, "update retries exhausted");
#endif
}

static void udc_dwc3_depcmd_update_xfer(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	uint32_t flags = 0;

	flags |= UDC_DWC3_DEPCMD_DEPUPDXFER;
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	LOG_DBG("DepUpdateXfer done ep=0x%02x addr=0x%08x data=0x%08x",
		ep_data->cfg.addr, UDC_DWC3_DEPCMD(ep_data->epn), flags);

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (udc_dwc3_acm_diag_ep(ep_data->cfg.addr)) {
		udc_dwc3_link_check(ep_data, "post-DepUpdateXfer");
	}
#endif
}

/*
 * IN DepStartXfer doorbell verify-and-retry.
 *
 * Derived from the OUT run-dry recovery (udc_dwc3_out_rundry_restart).  On this
 * IP a transfer-arming doorbell can be accepted (CMDACT clears, status OK) yet
 * HW never fetches the owned TRB when the shared command/scheduler path is busy
 * servicing the two RTL-driven UVC IN streams.  For CDC ACM IN every MPS-aligned
 * packet is LST-terminated, so xfer_active drops and every packet re-arms with
 * DepStartXfer -- one exposure per KB -- and a single dropped start strands the
 * pipe permanently: HWO stays 1, no XferComplete is posted, and the class layer
 * stops enqueuing (observed on-core, con5.out: ep 0x82 head=1 tail=0, tail TRB
 * HWO=1, GEVNTCOUNT=0, host frozen at 24 KiB).
 *
 * The OUT side proved a re-issued doorbell "from a settled state" takes when
 * done promptly and verified.  So immediately after DepStartXfer, confirm HW
 * consumed the just-armed tail TRB (under a continuous host IN drain HWO clears
 * within microseconds).  If it is still owned after a short settle, re-issue
 * DepUpdateXfer -- the transfer resource is now allocated by the StartXfer, so
 * UpdateXfer (a ring re-scan), not another StartXfer, is the correct nudge -- and
 * re-verify.  This is deliberately inline and immediate: the earlier background
 * poll re-kick fired milliseconds late and never recovered.  UpdateXfer against
 * a transfer that is legitimately armed and merely waiting for the host is
 * idempotent, so an over-issue during a host pause is harmless (the retry budget
 * bounds the cost).  EndXfer+StartXfer is intentionally avoided: on this IP it
 * corrupts the endpoint beyond even manual recovery (see out_rundry_restart).
 */
#define UDC_DWC3_INSTART_SETTLE_US     10U
#define UDC_DWC3_INSTART_FAST_STEPS    8U
#define UDC_DWC3_INSTART_REARM_RETRIES 6U
#define UDC_DWC3_INSTART_SETTLE_STEPS  8U

static atomic_t udc_dwc3_in_start_exhausted;
static atomic_t udc_dwc3_in_start_recycled;
#if !defined(CONFIG_UDC_DWC3_EP_SM)
static atomic_t udc_dwc3_in_start_retook;
static atomic_t udc_dwc3_in_start_backoff;
#endif

#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
/*
 * Last-resort recovery for a stranded IN DepStartXfer.
 *
 * When the UpdateXfer verify-and-retry budget is exhausted the transfer is
 * parked in a way UpdateXfer cannot resume (confirmed on-core: DEPCMD shows the
 * UpdateXfer completed with CMDACT clear and no CMDERR, yet the tail TRB stays
 * HWO=1 and unfetched while the two RTL-driven UVC IN streams monopolise the
 * controller scheduler).  The only remaining databook lever is to tear the
 * transfer resource down with EndTransfer(ForceRM) and re-arm the same
 * still-owned TRB with a fresh StartXfer.  Unlike OUT -- where EndXfer+StartXfer
 * corrupts because the restart races host-driven incoming data -- IN is
 * device-paced: the TRB is ours and merely awaiting a token, so a clean
 * end+restart is safe.
 *
 * The ring pointers are deliberately left intact (unlike udc_dwc3_depcmd_end_xfer,
 * which zeroes head/tail): the class layer is still waiting on this TRB's
 * completion, so re-arming the same slot lets the normal XferComplete path retire
 * it and keep bookkeeping in sync.
 */
#define UDC_DWC3_INSTART_RECYCLE_STEPS 16U

static bool udc_dwc3_in_start_tier5_recover(const struct device *const dev,
					    struct udc_dwc3_ep_data *ep_data);

static void udc_dwc3_in_start_endxfer_recycle(const struct device *const dev,
					      struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t tail = ep_data->tail;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
			UDC_DWC3_DEPCMD_DEPENDXFER | UDC_DWC3_DEPCMD_HIPRI_FORCERM |
			FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx));

	/*
	 * Re-arm the same parked (still HWO=1) TRB.  StartXfer allocates a fresh
	 * transfer resource and updates ep_data->xferrscidx.
	 */
	udc_dwc3_depcmd_start_xfer_trb(dev, ep_data, &ep_data->trb_buf[tail]);

	for (unsigned int step = 0U; step < UDC_DWC3_INSTART_RECYCLE_STEPS; step++) {
		k_busy_wait(UDC_DWC3_INSTART_SETTLE_US);
		if (!udc_dwc3_trb_hwo(&ep_data->trb_buf[tail])) {
			atomic_inc(&udc_dwc3_in_start_recycled);
#if defined(CONFIG_UDC_DWC3_EP_SM)
			ep_data->sm.in_start_verify_busy = false;
#endif
			return;
		}
	}

	if (udc_dwc3_in_start_tier5_recover(dev, ep_data)) {
		return;
	}

	atomic_inc(&udc_dwc3_in_start_exhausted);
	LOG_ERR("EP-SM: IN-START-RECYCLE ep=0x%02x EndXfer+StartXfer failed",
		ep_data->cfg.addr);
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	udc_dwc3_xfer_trace("IN-START-RECYCLE", ep_data, "endxfer+start failed");
#endif
}
#endif /* CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE */

#if defined(CONFIG_UDC_DWC3_OUT_RUNDRY_ENDXFER_ESCALATE)
static atomic_t udc_dwc3_out_rundry_recycled;
static atomic_t udc_dwc3_out_rundry_recycle_failed;

/*
 * Last-resort recovery for an OUT endpoint stranded by the run-dry park-window
 * UpdateXfer drop (see udc_dwc3_out_rundry_restart).  Invoked ONLY after the
 * UpdateXfer verify+spin budget in udc_dwc3_sm_out_update_verify is fully
 * exhausted -- i.e. the tail TRB is still HWO=1, HW has parked, and the pipe
 * would otherwise stay permanently wedged.
 *
 * From this confirmed-parked state the ring is fully drained and there is no
 * in-flight host packet, so the historical "EndXfer+StartXfer races host data
 * and corrupts OUT" hazard does not apply: any byte the host sends during the
 * tiny end->start window simply NAKs and the host retries.  And because we only
 * reach here when the alternative is a dead pipe, this is strictly no-worse: it
 * either re-fetches the owned TRB (pipe recovered) or leaves the same state.
 *
 * Mirrors the IN lever (udc_dwc3_in_start_endxfer_recycle) but leaves the ring
 * pointers intact so the normal XferComplete path retires the slot afterwards.
 * Returns true iff HW fetched the re-armed tail TRB.
 */
static bool udc_dwc3_out_rundry_endxfer_recycle(const struct device *const dev,
						struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t tail = ep_data->tail;

	/* Drop the parked transfer resource. */
	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
			UDC_DWC3_DEPCMD_DEPENDXFER | UDC_DWC3_DEPCMD_HIPRI_FORCERM |
			FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx));

	/*
	 * Re-arm the same still-owned tail TRB.  StartXfer allocates a fresh
	 * transfer resource and refreshes ep_data->xferrscidx.
	 */
	udc_dwc3_depcmd_start_xfer_trb(dev, ep_data, &ep_data->trb_buf[tail]);

	for (unsigned int step = 0U; step < UDC_DWC3_RUNDRY_SETTLE_STEPS; step++) {
		k_busy_wait(UDC_DWC3_RUNDRY_SETTLE_US);
		if (!udc_dwc3_trb_hwo(&ep_data->trb_buf[tail])) {
			atomic_inc(&udc_dwc3_out_rundry_recycled);
			LOG_WRN("EP-SM: OUT-RUNDRY-RECYCLE ep=0x%02x recovered via "
				"EndXfer(ForceRM)+StartXfer", ep_data->cfg.addr);
			return true;
		}
	}

	atomic_inc(&udc_dwc3_out_rundry_recycle_failed);
	return false;
}
#endif /* CONFIG_UDC_DWC3_OUT_RUNDRY_ENDXFER_ESCALATE */

#if !defined(CONFIG_UDC_DWC3_EP_SM)
static void udc_dwc3_in_start_verify(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t tail = ep_data->tail;

	/*
	 * Fast path: HW usually fetches the armed TRB within a few microseconds
	 * of the next host IN token, so poll briefly before deciding a doorbell
	 * was dropped.  At real ACM data rates the per-packet cost of this window
	 * is negligible; it only matters when a start is actually stranded.
	 */
	for (unsigned int step = 0U; step < UDC_DWC3_INSTART_FAST_STEPS; step++) {
		if (!udc_dwc3_trb_hwo(&ep_data->trb_buf[tail])) {
			return;
		}
		k_busy_wait(UDC_DWC3_INSTART_SETTLE_US);
	}

	for (unsigned int attempt = 0U; attempt < UDC_DWC3_INSTART_REARM_RETRIES; attempt++) {
		bool cmderr = false;

		udc_dwc3_depcmd_status(dev, UDC_DWC3_DEPCMD(ep_data->epn),
				       UDC_DWC3_DEPCMD_DEPUPDXFER |
				       FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK,
						  ep_data->xferrscidx),
				       &cmderr);

		for (unsigned int step = 0U; step < UDC_DWC3_INSTART_SETTLE_STEPS; step++) {
			k_busy_wait(UDC_DWC3_INSTART_SETTLE_US);
			if (!udc_dwc3_trb_hwo(&ep_data->trb_buf[tail])) {
				atomic_inc(&udc_dwc3_in_start_retook);
				return;
			}
		}

		if (cmderr) {
			atomic_inc(&udc_dwc3_in_start_backoff);
			return;
		}
	}

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	udc_dwc3_xfer_trace("IN-START-REARM", ep_data, "verify retries exhausted");
#endif
#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
	/* UpdateXfer is a dead lever here; escalate to EndXfer(ForceRM)+StartXfer. */
	udc_dwc3_in_start_endxfer_recycle(dev, ep_data);
#else
	atomic_inc(&udc_dwc3_in_start_exhausted);
#endif
}
#endif /* !CONFIG_UDC_DWC3_EP_SM */

static void udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags)
{
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
	flags |= UDC_DWC3_DEPCMD_DEPENDXFER;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	LOG_DBG("DepEndXfer done ep=0x%02x", ep_data->cfg.addr);

	ep_data->head = ep_data->tail = 0;
}

/*
 * DEPSTARTCFG allocates the pool of transfer resources the endpoints draw on.
 *
 * It is issued twice per enumeration, with the resource index saying which
 * pool: 0 after a reset, covering the control endpoint, and 2 when the first
 * non-control endpoint is enabled for the selected configuration, covering
 * everything else.  Miss the second and the bulk endpoints never get resources
 * allocated, so StartXfer on them fails with "no resource available".
 */
static void udc_dwc3_depcmd_start_config(const struct device *const dev,
					 struct udc_dwc3_ep_data *const ep_data,
					 const uint32_t rsc_idx)
{
	uint32_t flags = 0;

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, rsc_idx);
	flags |= UDC_DWC3_DEPCMD_DEPSTARTCFG;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	LOG_DBG("DepStartConfig done ep=0x%02x rsc_idx=%u", ep_data->cfg.addr, rsc_idx);
}

/*
 * Transfer Requests (TRB)
 *
 * DWC3 receives transfer requests from this driver through a shared memory
 * buffer, resubmitted upon every new transfer (through either Start or
 * Update command).
 */

static void udc_dwc3_ep_link_trb_init(struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t i = CONFIG_UDC_DWC3_TRB_NUM - 1U;
	volatile struct udc_dwc3_trb *const link = &ep_data->trb_buf[i];

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (udc_dwc3_acm_diag_ep(ep_data->cfg.addr) &&
	    (ep_data->xfer_active || udc_dwc3_ring_data_hwo_mask(ep_data) != 0U ||
	     udc_dwc3_link_trb_hwo(ep_data))) {
		udc_dwc3_xfer_trace("LINK-REINIT", ep_data, "ep_link_trb_init");
		udc_dwc3_log_trb_ring(ep_data, "pre-link-init");
	}
#endif

	udc_dwc3_trb_commit(link, LO32((uintptr_t)ep_data->trb_buf),
			    HI32((uintptr_t)ep_data->trb_buf), 0U,
			    UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB |
				    UDC_DWC3_TRB_CTRL_HWO);

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	udc_dwc3_link_check(ep_data, "post-link-init");
#endif
}

static void udc_dwc3_trb_norm_init(const struct device *const dev,
				   struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	LOG_DBG("Initializing normal TRB");

	/* TRB0 that prevents the transfer to be started (until it is overwritten) */
	trb[0].ctrl = 0;

	udc_dwc3_ep_link_trb_init(ep_data);

	/* Start the transfer now, update it later */
	udc_dwc3_depcmd_start_xfer(dev, ep_data);
	ep_data->xfer_active = true;
}

static void udc_dwc3_trb_ctrl_out(const struct device *const dev,
				  struct net_buf *const buf,
				  const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	trb[0].addr_lo = LO32((uintptr_t)buf->data);
	trb[0].addr_hi = HI32((uintptr_t)buf->data);
	trb[0].status = buf->size;
	trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;

	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

static void udc_dwc3_trb_ctrl_in(const struct device *const dev,
				 struct net_buf *const buf,
				 const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	if (udc_ep_buf_has_zlp(buf)) {
		trb[0].addr_lo = LO32((uintptr_t)buf->data);
		trb[0].addr_hi = HI32((uintptr_t)buf->data);
		trb[0].status = buf->len;
		trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_CHN | UDC_DWC3_TRB_CTRL_HWO;

		trb[1].addr_lo = 0;
		trb[1].addr_hi = 0;
		trb[1].status = 0;
		trb[1].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;
	} else {
		trb[0].addr_lo = LO32((uintptr_t)buf->data);
		trb[0].addr_hi = HI32((uintptr_t)buf->data);
		trb[0].status = buf->len;
		trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;
	}

	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

static void udc_dwc3_ep_ring_reset(struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t link = CONFIG_UDC_DWC3_TRB_NUM - 1U;

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	udc_dwc3_trace_ring_reset_haz(ep_data, "ep_ring_reset");
#endif

	for (uint32_t i = 0; i < link; i++) {
		ep_data->trb_buf[i].ctrl = 0;
		ep_data->trb_buf[i].status = 0;
		ep_data->trb_buf[i].addr_lo = 0;
		ep_data->trb_buf[i].addr_hi = 0;
		ep_data->net_buf[i] = NULL;
	}

	ep_data->head = ep_data->tail = 0U;
	ep_data->full = false;
	ep_data->chain_buf = NULL;
	udc_dwc3_ep_link_trb_init(ep_data);

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (udc_dwc3_acm_diag_ep(ep_data->cfg.addr)) {
		udc_dwc3_link_check(ep_data, "post-ring-reset");
	}
#endif
}

static int udc_dwc3_trb_bulk(const struct device *const dev,
			     struct udc_dwc3_ep_data *const ep_data,
			     struct net_buf *const buf)
{
	uint32_t ctrl = UDC_DWC3_TRB_CTRL_IOC | UDC_DWC3_TRB_CTRL_HWO | UDC_DWC3_TRB_CTRL_CSP;

	LOG_DBG("TRB_BULK_EP_0x%02x, buf %p, data %p, size %u, len %u",
		ep_data->cfg.addr, (void *)buf, (void *)buf->data, buf->size, buf->len);

	if (ep_data->full) {
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		udc_dwc3_xfer_trace("RING-FULL", ep_data, "trb_bulk");
#endif
		return -EBUSY;
	}

	/* LST ended the HW xfer; DepStart rings from [0] again */
	if (!ep_data->xfer_active) {
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		if (ep_data->head != ep_data->tail || ep_data->chain_buf != NULL) {
			udc_dwc3_xfer_trace("RINGRST-HAZ", ep_data, "pre-reset");
		}
#endif
		udc_dwc3_bulk_ring_restart(ep_data);
	}

	/* CDC ACM flush after MPS-aligned echo — ZLP already chained on wire */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr) && buf->len == 0 &&
	    !udc_ep_buf_has_zlp(buf) && ep_data->absorb_cdc_zlp) {
		ep_data->absorb_cdc_zlp = false;
		ep_data->total = 0;
		LOG_DBG("Absorb CDC ACM ZLP flush (chained on prior TRB)");
		udc_submit_ep_event(dev, buf, 0);
		k_work_submit(&ep_data->work);
		return 0;
	}

	if (udc_ep_buf_has_zlp(buf)) {
		LOG_DBG("Buffer has a ZLP flag, terminating the transfer");
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP;
		ep_data->total = 0;
	} else {
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL;
		ep_data->total += buf->len;

		if (USB_EP_DIR_IS_IN(ep_data->cfg.addr) &&
		    buf->len > 0 &&
		    ep_data->total % ep_data->cfg.mps == 0) {
			LOG_DBG("MPS-aligned IN %u bytes, TRB NORMAL_ZLP",
				ep_data->total);
			udc_ep_buf_set_zlp(buf);
			ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP;
			ctrl |= UDC_DWC3_TRB_CTRL_LST;
			ep_data->total = 0;
		} else {
			LOG_DBG("End of USB transfer, %u bytes transferred", ep_data->total);
			ep_data->total = 0;
		}
	}

#if defined(CONFIG_UDC_DWC3_DMA_SLOT_DIAG)
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		udc_dwc3_dma_slot_note(ep_data->cfg.addr, UDC_DWC3_DMA_IN_ENQ,
				       buf->data, buf->len);
	} else {
		udc_dwc3_dma_slot_note(ep_data->cfg.addr, UDC_DWC3_DMA_OUT_ENQ,
				       buf->data, buf->size);
	}
#endif

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (ep_data->cfg.addr == 0x01) {
		udc_dwc3_trace_out_push_haz(ep_data);
	}
	udc_dwc3_trace_sram_cross(dev, ep_data, buf->data, "bulk-enq");
#endif

	if (udc_dwc3_ring_push_guard(ep_data) != 0) {
		return -EBUSY;
	}

	/*
	 * Detect an OUT re-arm into a parked (fully drained) ring.  If the
	 * transfer is active but no data TRB is currently owned by HW, the
	 * controller has run dry and parked; the plain UpdateXfer used to
	 * resume is dropped in the park transition window (see
	 * udc_dwc3_out_rundry_restart), so this arm needs the EndXfer+StartXfer
	 * recycle instead.  Sampled before the push, while the ring is still
	 * empty.
	 */
	const bool resume_from_park =
		ep_data->xfer_active && udc_dwc3_ring_data_hwo_mask(ep_data) == 0U;
	const bool out_resume_from_park =
		resume_from_park && USB_EP_DIR_IS_OUT(ep_data->cfg.addr);
	/*
	 * IN hits the very same park: the ring drains whenever the class layer
	 * pauses (between shell writes), and the plain UpdateXfer below is then
	 * dropped exactly as it is on OUT, stranding the TRB at HWO=1 with no
	 * mechanism left to notice (see udc_dwc3_sm_watchdog_ep).  Verify the
	 * fetch on this path instead of firing and forgetting.  Under UVC
	 * streaming the ACM IN ring drains far more often -- the video endpoints
	 * monopolise the controller -- which is why this only shows up loaded.
	 *
	 * The verify inspects the tail slot, so it only applies when the slot
	 * just pushed *is* the tail.  A tail left behind on an already-completed
	 * slot is the poll's job; once it retires, the watchdog covers the rest.
	 */
	const uint32_t push_slot = ep_data->head;
	const bool in_resume_from_park =
		resume_from_park && USB_EP_DIR_IS_IN(ep_data->cfg.addr);

	udc_dwc3_push_trb(dev, ep_data, buf, ctrl);
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (ep_data->cfg.addr == 0x01) {
		udc_dwc3_out_acct_note_push(ep_data);
	}
#endif

	if (!ep_data->xfer_active) {
#if defined(CONFIG_UDC_DWC3_EP_SM)
		if (udc_dwc3_ep_sm_is_cpu(ep_data)) {
			(void)udc_dwc3_doorbell_issue(dev, ep_data, UDC_DWC3_DB_START);
		} else {
			udc_dwc3_depcmd_start_xfer(dev, ep_data);
		}
#else
		udc_dwc3_depcmd_start_xfer(dev, ep_data);
#endif
		ep_data->xfer_active = true;
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		udc_dwc3_xfer_trace("DEPSTART", ep_data, "post-DepStartXfer");
#endif
#if !defined(CONFIG_UDC_DWC3_EP_SM)
		/*
		 * Confirm HW actually fetched the freshly-armed TRB; re-nudge if
		 * the StartXfer doorbell was dropped under UVC-stream contention
		 * (see udc_dwc3_in_start_verify).  IN only: OUT re-arm goes
		 * through the run-dry path below.
		 */
		if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
			udc_dwc3_in_start_verify(dev, ep_data);
		}
#endif
	} else {
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		if (ep_data->cfg.addr == 0x01) {
			const uint32_t hwo_mask = udc_dwc3_ring_data_hwo_mask(ep_data);

			if (ep_data->head == ep_data->tail) {
				udc_dwc3_xfer_trace("DEPUPD-EMPTY", ep_data, "head==tail");
			}
			LOG_DBG("XFER-TRACE DEPUPD-OUT head=%u tail=%u hwo_mask=0x%x",
				ep_data->head, ep_data->tail, hwo_mask);
		}
#endif
		if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
			udc_dwc3_link_trb_ensure(ep_data);
		}
		if (out_resume_from_park) {
#if defined(CONFIG_UDC_DWC3_EP_SM)
			if (udc_dwc3_ep_sm_is_cpu(ep_data)) {
				(void)udc_dwc3_doorbell_issue(dev, ep_data,
							      UDC_DWC3_DB_UPDATE_VERIFY);
			} else {
				udc_dwc3_out_rundry_restart(dev, ep_data);
			}
#else
			udc_dwc3_out_rundry_restart(dev, ep_data);
#endif
		} else {
#if defined(CONFIG_UDC_DWC3_EP_SM)
			if (udc_dwc3_ep_sm_is_cpu(ep_data)) {
				const bool verify = in_resume_from_park &&
						    ep_data->tail == push_slot;

				(void)udc_dwc3_doorbell_issue(dev, ep_data,
					verify ? UDC_DWC3_DB_UPDATE_VERIFY_IN
					       : UDC_DWC3_DB_UPDATE);
			} else {
				udc_dwc3_depcmd_update_xfer(dev, ep_data);
			}
#else
			udc_dwc3_depcmd_update_xfer(dev, ep_data);
#endif
		}
	}

	return 0;
}

/*
 * Control buffers
 *
 * There is no worker for control buffers, and instead buffers are udc_dwc3_next_ctrl() is called
 * whenever there is an opportunity to send more, and only when all conditions are met.
 */

static void udc_dwc3_next_ctrl_in(const struct device *const dev,
				  struct net_buf *const buf)
{
	struct udc_data *const data = dev->data;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	/* STATUS_2 vs STATUS_3 must use priv->ctrl_setup, not stale data->setup */
	const struct usb_setup_packet *const setup = &priv->ctrl_setup;
	struct udc_buf_info *const bi = udc_get_buf_info(buf);

	if (bi->status) {
		const bool chose_status2 = (setup->wLength == 0);
		const struct usb_setup_packet *const stale = (void *)data->setup;
		const bool stale_status2 = (stale->wLength == 0);

		if (chose_status2 != stale_status2) {
			atomic_inc(&udc_dwc3_ep0_stage_fixed);
			LOG_INF("EP0 STATUS FIX: bmRT=0x%02x bReq=0x%02x real wLen=%u -> "
				"STATUS_%d (stale data->setup wLen=%u would pick STATUS_%d)",
				setup->bmRequestType, setup->bRequest, setup->wLength,
				chose_status2 ? 2 : 3, stale->wLength, stale_status2 ? 2 : 3);
		}

		const bool want_status2 = udc_dwc3_dbg_setup_valid
			? (udc_dwc3_dbg_setup.wLength == 0) : chose_status2;

		if (chose_status2 != want_status2) {
			atomic_inc(&udc_dwc3_ep0_stage_mismatch);
			LOG_WRN("EP0 STATUS MISMATCH (post-fix!): bmRT=0x%02x bReq=0x%02x "
				"real wLen=%u snapshot wLen=%u chose=STATUS_%d want=STATUS_%d",
				setup->bmRequestType, setup->bRequest, setup->wLength,
				udc_dwc3_dbg_setup.wLength, chose_status2 ? 2 : 3,
				want_status2 ? 2 : 3);
		}
	}

	if (bi->data) {
		LOG_DBG("TRB_CONTROL_IN_DATA len=%d buf=%p", buf->len, buf);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi->status && setup->wLength == 0) {
		buf->size = 0;
		LOG_DBG("TRB_CONTROL_IN_STATUS_2 len=%d buf=%p", buf->len, buf);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
	} else if (bi->status) {
		buf->size = 0;
		LOG_DBG("TRB_CONTROL_IN_STATUS_3 len=%d buf=%p", buf->len, buf);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
	} else {
		LOG_ERR("Unknown buffer IN type");
		udc_submit_ep_event(dev, buf, -EINVAL);
	}
}

static void udc_dwc3_next_ctrl_out(const struct device *const dev,
				   struct net_buf *const buf)
{
	struct udc_buf_info *const bi = udc_get_buf_info(buf);

	if (bi->setup) {
		buf->size = MIN(buf->size, sizeof(struct usb_setup_packet));
		LOG_DBG("TRB_CONTROL_OUT_SETUP size=%d buf=%p", buf->size, buf);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);
	} else if (bi->data) {
		LOG_DBG("TRB_CONTROL_OUT_DATA size=%d buf=%p", buf->size, buf);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi->status) {
		buf->size = 0;
		LOG_DBG("TRB_CONTROL_OUT_STATUS_3 size=%d buf=%p", buf->size, buf);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
	} else {
		LOG_ERR("Unknown buffer OUT, size=%d buf=%p", buf->size, buf);
		udc_submit_ep_event(dev, buf, -EINVAL);
	}
}

static void udc_dwc3_next_ctrl(const struct device *const dev,
			       struct udc_dwc3_ep_data *const ep_data)
{
	struct net_buf *buf;

	if (udc_ep_is_busy(&ep_data->cfg)) {
		return;
	}

	buf = udc_buf_peek(&ep_data->cfg);
	if (buf == NULL) {
		return;
	}

	udc_ep_set_busy(&ep_data->cfg, true);

	/* In DWC3 */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		udc_dwc3_next_ctrl_in(dev, buf);
	} else {
		udc_dwc3_next_ctrl_out(dev, buf);
	}
}

static void udc_dwc3_ep0_check_trb(const struct device *const dev, const uint32_t trb_status)
{
	ARG_UNUSED(dev);

	switch (trb_status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK) {
	case UDC_DWC3_TRB_STATUS_TRBSTS_OK:
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING:
		atomic_inc(&udc_dwc3_ep0_setuppending);
		LOG_WRN("EP0 SETUPPENDING (host started a new control transfer while one "
			"was pending) last setup bmRT=0x%02x bReq=0x%02x wLen=%u",
			udc_dwc3_dbg_setup.bmRequestType, udc_dwc3_dbg_setup.bRequest,
			udc_dwc3_dbg_setup.wLength);
		break;
	default:
		atomic_inc(&udc_dwc3_ep0_trb_err);
		LOG_WRN("EP0 TRB non-OK trbsts=0x%08x (status=0x%08x) last setup "
			"bmRT=0x%02x bReq=0x%02x wLen=%u",
			(uint32_t)(trb_status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK), trb_status,
			udc_dwc3_dbg_setup.bmRequestType, udc_dwc3_dbg_setup.bRequest,
			udc_dwc3_dbg_setup.wLength);
		break;
	}
}

/*
 * Events
 *
 * Process the events from the event ring buffer. Interrupts gives us a
 * hint that an event is available, which we fetch from a ring buffer shared
 * with the hardware.
 */

static void udc_dwc3_on_soft_reset(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	/* Configure and reset the Device Controller */
	/* TODO confirm that DWC_USB3_EN_LPM_ERRATA == 1 */
	reg = UDC_DWC3_DCTL_CSFTRST;
	reg |= FIELD_PREP(UDC_DWC3_DCTL_LPM_NYET_THRES_MASK, 15);
	sys_write32(reg, base + UDC_DWC3_DCTL);
	while (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_CSFTRST) {
		continue;
	}

	/* Enable AXI64 bursts for various sizes expected */
	reg = UDC_DWC3_GSBUSCFG0_INCR256BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR128BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR64BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR32BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR16BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR8BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR4BRSTENA;
	sys_set_bits(base + UDC_DWC3_GSBUSCFG0, reg);

	/*
	 * Program GTXTHRCFG TX threshold (omitted in the 4.4 port). Buffer
	 * USBTXPKTCNT packets before each SS burst to avoid TX FIFO underrun.
	 * count=3 / burst=4 — sized from FIFO depth, see USB_COMPLIANCE_CHANGES.md §6.
	 */
	reg = UDC_DWC3_GTXTHRCFG_USBTXPKTCNTSEL;
	reg |= FIELD_PREP(UDC_DWC3_GTXTHRCFG_USBTXPKTCNT_MASK, 3);
	reg |= FIELD_PREP(UDC_DWC3_GTXTHRCFG_USBMAXTXBURSTSIZE_MASK, 4);
	sys_write32(reg, base + UDC_DWC3_GTXTHRCFG);

	/* Letting GRXTHRCFG unchanged */

	/* Read the chip identification */
	reg = sys_read32(base + UDC_DWC3_GCOREID);
	LOG_INF("event: coreid=0x%04lx rel=0x%04lx",
		FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg),
		FIELD_GET(UDC_DWC3_GCOREID_REL_MASK, reg));
	__ASSERT_NO_MSG(FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg) == 0x5533);

	/* Letting GUID unchanged */
	/* Letting GUSB2PHYCFG and GUSB3PIPECTL unchanged */
	/* Letting GRXFIFOSIZ unchanged */

	/* Setup the event buffer address, size and start event reception */
	memset((void *)cfg->evt_buf, 0, CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t));
	sys_write32(HI32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_HI(0));
	sys_write32(LO32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_LO(0));
	sys_write32(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t), base + UDC_DWC3_GEVNTSIZ(0));
	LOG_INF("Event buffer size is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));
	sys_write32(0, base + UDC_DWC3_GEVNTCOUNT(0));

	/* Letting GCTL unchanged */

	/* Set the USB device configuration, including max supported speed */
	sys_write32(UDC_DWC3_DCFG_PERFRINT_90, base + UDC_DWC3_DCFG);
	switch (cfg->maximum_speed_idx) {
	case UDC_DWC3_SPEED_IDX_SUPER_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_SUPER_SPEED");
		sys_set_bits(base + UDC_DWC3_DCFG, UDC_DWC3_DCFG_DEVSPD_SUPER_SPEED);
		break;
	case UDC_DWC3_SPEED_IDX_HIGH_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_HIGH_SPEED");
		sys_set_bits(base + UDC_DWC3_DCFG, UDC_DWC3_DCFG_DEVSPD_HIGH_SPEED);
		break;
	case UDC_DWC3_SPEED_IDX_FULL_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_FULL_SPEED");
		sys_set_bits(base + UDC_DWC3_DCFG, UDC_DWC3_DCFG_DEVSPD_FULL_SPEED);
		break;
	default:
		CODE_UNREACHABLE;
	}

	/* Set the number of USB3 packets the device can receive at once */
	reg = sys_read32(base + UDC_DWC3_DCFG);
	reg &= ~UDC_DWC3_DCFG_NUMP_MASK;
	reg |= FIELD_PREP(UDC_DWC3_DCFG_NUMP_MASK, 15);
	sys_write32(reg, base + UDC_DWC3_DCFG);

	/* All USB events except ULSTCNGEN (enabled with HEALTH_LOG for link diagnostics) */
	reg = UDC_DWC3_DEVTEN_INACTTIMEOUTRCVEDEN;
	reg |= UDC_DWC3_DEVTEN_VNDRDEVTSTRCVEDEN;
	reg |= UDC_DWC3_DEVTEN_EVNTOVERFLOWEN;
	reg |= UDC_DWC3_DEVTEN_CMDCMPLTEN;
	reg |= UDC_DWC3_DEVTEN_ERRTICERREN;
	reg |= UDC_DWC3_DEVTEN_HIBERNATIONREQEVTEN;
	reg |= UDC_DWC3_DEVTEN_WKUPEVTEN;
#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
	reg |= UDC_DWC3_DEVTEN_ULSTCNGEN;
#endif
	reg |= UDC_DWC3_DEVTEN_CONNECTDONEEN;
	reg |= UDC_DWC3_DEVTEN_USBRSTEN;
	reg |= UDC_DWC3_DEVTEN_DISCONNEVTEN;
	sys_write32(reg, base + UDC_DWC3_DEVTEN);
#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
	udc_dwc3_dump_link_cfg(dev, "after-enable");
#endif

	/*
	 * Control endpoint only; the non-control pool is allocated when the
	 * first of those endpoints is enabled, once a configuration is chosen.
	 */
	udc_dwc3_depcmd_start_config(dev, &cfg->ep_data_out[0], 0U);
	DEV_DATA(dev)->startcfg_nonctrl_done = false;
}

static void udc_dwc3_on_usb_reset(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	atomic_inc(&udc_dwc3_usbrst_count);
	udc_dwc3_reset_depevt_counts();
	LOG_DBG("Going through DWC3 reset logic");

	/* The host will re-enumerate, so the non-control pool is reallocated. */
	DEV_DATA(dev)->startcfg_nonctrl_done = false;

	/* Reset all ongoing transfers on non-control IN endpoints */
	for (int epn = 1; epn < cfg->num_in_eps; epn++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[epn];

		continue; /* TODO */
		udc_dwc3_depcmd_end_xfer(dev, ep_data, 0);
		udc_dwc3_depcmd_clear_stall(dev, ep_data);
	}

	/* Reset all ongoing transfers on non-control OUT endpoints */
	for (int epn = 1; epn < cfg->num_out_eps; epn++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[epn];

		continue; /* TODO */
		udc_dwc3_depcmd_end_xfer(dev, ep_data, 0);
		udc_dwc3_depcmd_clear_stall(dev, ep_data);
	}

	/* Perform the USB reset operations manually to improve latency */
	udc_dwc3_set_address(dev, 0);

	/* Let Zephyr set the device address 0 */
	udc_submit_event(dev, UDC_EVT_RESET, 0);

#if defined(CONFIG_UDC_DWC3_EP_SM)
	udc_dwc3_ep_sm_reset_all(dev);
	atomic_set(&DEV_DATA(dev)->bulk_eps_live, 0);
# if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
	k_work_cancel_delayable(&DEV_DATA(dev)->in_poll_work);
# endif
# if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
	LOG_WRN("EP-SM: USBRST bulk_eps_live=0 poll stopped");
# endif
#endif
}

static void udc_dwc3_on_connect_done(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	int mps = 0;

	/* Adjust parameters against the connection speed */
	switch (sys_read32(base + UDC_DWC3_DSTS) & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
		mps = 64;
		break;
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		mps = 512;
		break;
	}
	__ASSERT_NO_MSG(mps != 0);

	/* Reconfigure control endpoints connection speed */
	udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT)->mps = mps;
	udc_get_ep_cfg(dev, USB_CONTROL_EP_IN)->mps = mps;
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0]);
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_out[0]);

	/* Letting GTXFIFOSIZn unchanged */
}

static void udc_dwc3_on_link_state_event(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	reg = sys_read32(base + UDC_DWC3_DSTS);

#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
	LOG_INF("ULSTCHNG -> %s (dsts=0x%08x)", udc_dwc3_linkstate_str(reg), reg);
#endif

	switch (reg & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		switch (reg & UDC_DWC3_DSTS_USBLNKST_MASK) {
		case UDC_DWC3_DSTS_USBLNKST_USB3_U0:
			LOG_DBG("DSTS_USBLNKST_USB3_U0");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_U1:
			LOG_DBG("DSTS_USBLNKST_USB3_U1");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_U2:
			LOG_DBG("DSTS_USBLNKST_USB3_U2");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_U3:
			LOG_DBG("DSTS_USBLNKST_USB3_U3");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS:
			LOG_DBG("DSTS_USBLNKST_USB3_SS_DIS");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET:
			LOG_DBG("DSTS_USBLNKST_USB3_RX_DET");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT:
#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
			atomic_inc(&udc_dwc3_ss_inact_count);
			LOG_WRN("LINK DROP: SS.Inactive (the re-enumeration trigger) "
				"dsts=0x%08x", reg);
			udc_dwc3_dump_link_cfg(dev, "ss-inact");
#else
			LOG_DBG("DSTS_USBLNKST_USB3_SS_INACT");
#endif
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_POLL:
			LOG_DBG("DSTS_USBLNKST_USB3_POLL");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_RECOV:
#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
			atomic_inc(&udc_dwc3_ss_recov_count);
			LOG_WRN("LINK: entered Recovery dsts=0x%08x", reg);
#else
			LOG_DBG("DSTS_USBLNKST_USB3_RECOV");
#endif
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_HRESET:
			LOG_DBG("DSTS_USBLNKST_USB3_HRESET");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_CMPLY:
			LOG_DBG("DSTS_USBLNKST_USB3_CMPLY");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_LPBK:
			LOG_DBG("DSTS_USBLNKST_USB3_LPBK");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME:
			LOG_DBG("DSTS_USBLNKST_USB3_RESET_RESUME");
			break;
		default:
			LOG_ERR("unknown USB3 link state");
		}
		break;
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
		switch (reg & UDC_DWC3_DSTS_USBLNKST_MASK) {
		case UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE:
			LOG_DBG("DSTS_USBLNKST_USB2_ON_STATE");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE:
			LOG_DBG("DSTS_USBLNKST_USB2_SLEEP_STATE");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE:
			LOG_DBG("DSTS_USBLNKST_USB2_SUSPEND_STATE");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED:
			LOG_DBG("DSTS_USBLNKST_USB2_DISCONNECTED");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND:
			LOG_DBG("DSTS_USBLNKST_USB2_EARLY_SUSPEND");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_RESET:
			LOG_DBG("DSTS_USBLNKST_USB2_RESET");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_RESUME:
			LOG_DBG("DSTS_USBLNKST_USB2_RESUME");
			break;
		default:
			LOG_ERR("unknown USB2 link state");
		}
		break;
	default:
		LOG_ERR("unknown connection speed");
	}
}

/*
 * Handle completion of a CONTROL IN packet (device -> host).
 *
 * Further characterize which type of CONTROL IN packet that is.
 * Handle actions common to all CONTROL IN packets.
 */
static void udc_dwc3_on_ctrl_in(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
	const uint32_t trb_trbctl = ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	struct net_buf *buf;

	if (!udc_dwc3_ep0_note_gate(dev, ep_data, udc_dwc3_ep0_gate(ep_data))) {
		return;
	}

	udc_dwc3_ep0_check_trb(dev, ep_data->trb_buf[0].status);

	buf = udc_buf_get(&ep_data->cfg);
	if (buf == NULL) {
		/*
		 * HWO-first: completion arrived but queue already drained —
		 * late duplicate hint, not ENOBUFS.
		 */
		if (!udc_dwc3_trb_hwo(&ep_data->trb_buf[0])) {
			udc_dwc3_ep0_storm_bump(&udc_dwc3_ep0_dup_hint_in,
						UDC_DWC3_EP0_STORM_DUP_IN,
						"dup_hint IN (no buffer at completion)");
			udc_ep_set_busy(&ep_data->cfg, false);
			udc_dwc3_ep0_storm_maybe_log();
			return;
		}

		atomic_inc(&udc_dwc3_enobufs_count);
		LOG_ERR("Failed to get a buffer for ep 0x%02x", ep_data->cfg.addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return;
	}

	atomic_inc(&udc_dwc3_ctrl_in_count);

	if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
	    trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
		buf->len = 0;
		LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL STATUS packet sent");
	} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
		LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL DATA packet sent");
	} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		LOG_ERR("Unexpected SETUP IN packet");
	} else {
		LOG_ERR("Unexpected IN packet type: 0x%x", trb_trbctl);
	}

	udc_submit_ep_event(dev, buf, 0);

	/* Used when receiving a completed buffer from the hardware: mark as free */
	udc_ep_set_busy(&ep_data->cfg, false);

	udc_dwc3_next_ctrl(dev, ep_data);
}

/*
 * Handle completion of a CONTROL OUT packet (host -> device).
 *
 * Further characterize which type of CONTROL OUT packet that is.
 * Handle actions common to all CONTROL OUT packets.
 */
static void udc_dwc3_on_ctrl_out(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[0];
	const uint32_t trb_trbctl = ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	const uint32_t trb_status = ep_data->trb_buf[0].status;
	struct net_buf *buf;

	if (!udc_dwc3_ep0_note_gate(dev, ep_data, udc_dwc3_ep0_gate(ep_data))) {
		return;
	}

	udc_dwc3_ep0_check_trb(dev, trb_status);

	if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		struct usb_setup_packet *setup;

		buf = udc_buf_peek(&ep_data->cfg);
		if (buf == NULL) {
			if (!udc_dwc3_trb_hwo(&ep_data->trb_buf[0])) {
				udc_dwc3_ep0_storm_bump(&udc_dwc3_ep0_dup_hint_out,
							UDC_DWC3_EP0_STORM_DUP_OUT,
							"dup_hint OUT SETUP (no buffer)");
				udc_ep_set_busy(&ep_data->cfg, false);
				udc_dwc3_ep0_storm_maybe_log();
				return;
			}

			atomic_inc(&udc_dwc3_enobufs_count);
			LOG_ERR("missing buffer for SETUP packet");
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
			return;
		}

		atomic_inc(&udc_dwc3_ctrl_setup_count);

		setup = (struct usb_setup_packet *)buf->data;

		/* Refresh before EP0 IN status stage selects STATUS_2 vs STATUS_3 */
		memcpy(&DEV_DATA(dev)->ctrl_setup, setup, sizeof(DEV_DATA(dev)->ctrl_setup));
		memcpy(&udc_dwc3_dbg_setup, setup, sizeof(udc_dwc3_dbg_setup));
		udc_dwc3_dbg_setup_valid = true;

		/* Latency optimization: set the address immediately to be able to be able
		 * to ACK/NAK the first packets from the host with the new address,
		 * otherwise the host issue a reset.
		 */
		if (setup->bmRequestType == USB_REQTYPE_TYPE_STANDARD &&
		    setup->bRequest == USB_SREQ_SET_ADDRESS) {
			udc_dwc3_set_address(dev, setup->wValue);
		}

		/* Update the size to what the hardware reports */
		buf->len = buf->size - FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb_status);

		LOG_HEXDUMP_DBG(buf->data, buf->len, "SETUP received");

		/* The buffer will directly be taken from the UDC queue */
		udc_setup_received(dev, NULL);
	} else {
		buf = udc_buf_get(&ep_data->cfg);
		if (buf == NULL) {
			if (!udc_dwc3_trb_hwo(&ep_data->trb_buf[0])) {
				udc_dwc3_ep0_storm_bump(&udc_dwc3_ep0_dup_hint_out,
							UDC_DWC3_EP0_STORM_DUP_OUT,
							"dup_hint OUT (no buffer at completion)");
				udc_ep_set_busy(&ep_data->cfg, false);
				udc_dwc3_ep0_storm_maybe_log();
				return;
			}

			atomic_inc(&udc_dwc3_enobufs_count);
			LOG_ERR("Failed to get a buffer for ep 0x%02x", ep_data->cfg.addr);
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
			return;
		}

		atomic_inc(&udc_dwc3_ctrl_out_count);

		/* Update the size to what the hardware reports */
		buf->len = buf->size - FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb_status);

		if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL DATA received");
		} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
			buf->len = 0;
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL STATUS received");
		} else {
			LOG_ERR("Unexpected OUT packet type: 0x%x", trb_trbctl);
		}

		udc_submit_ep_event(dev, buf, 0);
	}

	/* Used when receiving a completed buffer from the hardware: mark as free */
	udc_ep_set_busy(&ep_data->cfg, false);

	udc_dwc3_next_ctrl(dev, ep_data);
}

static void udc_dwc3_ep0_poll_all(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const out0 = &cfg->ep_data_out[0];
	struct udc_dwc3_ep_data *const in0 = &cfg->ep_data_in[0];

	if (!udc_dwc3_trb_hwo(&out0->trb_buf[0]) && udc_ep_is_busy(&out0->cfg) &&
	    udc_buf_peek(&out0->cfg) != NULL) {
		udc_dwc3_on_ctrl_out(dev);
		atomic_inc(&udc_dwc3_ep0_poll_retired_out);
	} else if (!udc_dwc3_trb_hwo(&out0->trb_buf[0]) && udc_ep_is_busy(&out0->cfg)) {
		udc_ep_set_busy(&out0->cfg, false);
	}

	if (!udc_dwc3_trb_hwo(&in0->trb_buf[0]) && udc_ep_is_busy(&in0->cfg) &&
	    udc_buf_peek(&in0->cfg) != NULL) {
		udc_dwc3_on_ctrl_in(dev);
		atomic_inc(&udc_dwc3_ep0_poll_retired_in);
	} else if (!udc_dwc3_trb_hwo(&in0->trb_buf[0]) && udc_ep_is_busy(&in0->cfg)) {
		udc_ep_set_busy(&in0->cfg, false);
	}
}

#if defined(CONFIG_UDC_DWC3_WEDGE_LOG)
static void udc_dwc3_wedge_snap(const struct device *const dev,
				struct udc_dwc3_ep_data *const ep_data,
				const char *const reason);
#endif

static void udc_dwc3_on_xfer_not_ready(const struct device *const dev,
				       const uint32_t evt)
{
	switch (evt & UDC_DWC3_DEPEVT_STATUS_B3_MASK) {
	case UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_SETUP:
		LOG_DBG("UDC_DWC3_DEPEVT_XFERNOTREADY_CONTROL_SETUP");
		break;
	case UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_DATA:
		LOG_DBG("UDC_DWC3_DEPEVT_XFERNOTREADY_CONTROL_DATA");
		break;
	case UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_STATUS:
		LOG_DBG("UDC_DWC3_DEPEVT_XFERNOTREADY_CONTROL_STATUS");
		break;
	}
}

#if defined(CONFIG_UDC_DWC3_OUT_NOTREADY_RETAKE)
static atomic_t udc_dwc3_out_notready_retaken;

/*
 * The host tried to send on a bulk OUT endpoint and HW had nothing ready.
 *
 * If the ring is genuinely empty this is routine -- the class simply has not
 * armed a replacement yet, and the host will retry.  What is not routine is
 * seeing it while the tail TRB is armed and hardware-owned: HW is refusing
 * data it has a buffer for, which only happens when the StartXfer doorbell for
 * that TRB was accepted and then dropped.  The transfer resource is stale, so
 * drop and retake it.
 *
 * Doing this from the event rather than a timer is what makes it safe.  There
 * is real host traffic in flight by definition, so no data is being discarded
 * from an idle pipe, and it costs nothing when the pipe is healthy -- as
 * opposed to an unconditional periodic EndXfer, which churns transfer
 * resources on every quiet endpoint forever.
 */
static void udc_dwc3_on_xfer_not_ready_norm(const struct device *const dev,
					    const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *const ep_data =
		(epn & 1) ? &cfg->ep_data_in[epn >> 1] : &cfg->ep_data_out[epn >> 1];

	if (ep_data->trb_buf == NULL || USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		return;
	}

	if (ep_data->net_buf[ep_data->tail] == NULL ||
	    !udc_dwc3_trb_hwo(&ep_data->trb_buf[ep_data->tail])) {
		LOG_DBG("OUT-NOTREADY ep=0x%02x unarmed, host will retry",
			ep_data->cfg.addr);
		return;
	}

	atomic_inc(&udc_dwc3_out_notready_retaken);
	LOG_WRN("OUT-NOTREADY ep=0x%02x armed but not taken, retaking "
		"(tail=%u hwo_mask=0x%x active=%d)",
		ep_data->cfg.addr, ep_data->tail,
		udc_dwc3_ring_data_hwo_mask(ep_data), ep_data->xfer_active ? 1 : 0);

	udc_dwc3_out_rundry_endxfer_recycle(dev, ep_data);
}
#endif /* CONFIG_UDC_DWC3_OUT_NOTREADY_RETAKE */

static void udc_dwc3_on_xfer_done(const struct device *const dev,
				  struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->tail];

	switch (trb->status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK) {
	case UDC_DWC3_TRB_STATUS_TRBSTS_OK:
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_MISSEDISOC:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_MISSEDISOC");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING");
		break;
	default:
		CODE_UNREACHABLE;
	}
}

static void udc_dwc3_on_xfer_done_norm(const struct device *const dev,
				       const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *const ep_data =
		(epn & 1) ? &cfg->ep_data_in[epn >> 1] : &cfg->ep_data_out[epn >> 1];
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->tail];
	const bool chn = !!(trb->ctrl & UDC_DWC3_TRB_CTRL_CHN);
	const bool lst = !!(trb->ctrl & UDC_DWC3_TRB_CTRL_LST);
	const bool from_inprog = ((evt & GENMASK(7, 6)) == (0x2U << 6));
	uint32_t trb_status_done = 0U;
	struct net_buf *buf;
	int ret;

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (ep_data->cfg.addr == 0x01) {
		const uint32_t tail = ep_data->tail;
		const uint32_t hwo_mask = udc_dwc3_ring_data_hwo_mask(ep_data);

		if (ep_data->net_buf[tail] != NULL) {
			LOG_DBG("XFER-TRACE OUT-DONE tail=%u via=%s hwo_mask=0x%x",
				tail, from_inprog ? "INPROG" : "COMPLETE", hwo_mask);
		}
	}
#endif

	/*
	 * After try_retire_chained_zlp() the tail already passed the ZLP TRB,
	 * but DWC3 may still post XFERCOMPLETE for it.  net_buf[tail] is NULL
	 * while the hardware event is stale — not a real underrun.
	 */
	if (ep_data->net_buf[ep_data->tail] == NULL) {
		if (ep_data->skip_xfer_done_count > 0U) {
			ep_data->skip_xfer_done_count--;
			LOG_DBG("EP 0x%02x: suppress spurious xfer-done (remain %u)",
				ep_data->cfg.addr, ep_data->skip_xfer_done_count);
#if defined(CONFIG_UDC_DWC3_WEDGE_LOG)
			if (ep_data->skip_xfer_done_count == 0U &&
			    udc_dwc3_ep_sm_is_cpu(ep_data)) {
				LOG_WRN("WEDGE spur-done-done ep=0x%02x via=%s link_hwo=%d",
					ep_data->cfg.addr, from_inprog ? "inprog" : "complete",
					udc_dwc3_link_trb_hwo(ep_data));
			}
#endif
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
			if (ep_data->cfg.addr == 0x01) {
				udc_dwc3_xfer_trace("OUT-SPUR-DONE", ep_data,
						    from_inprog ? "inprog" : "complete");
			}
			if (ep_data->skip_xfer_done_count == 0U &&
			    udc_dwc3_link_trb_hwo(ep_data)) {
				udc_dwc3_xfer_trace("SPUR-DONE", ep_data,
						    "last suppress, link still hwo");
			}
#endif
			k_work_submit(&ep_data->work);
			return;
		}

		/*
		 * HWO-first: empty net_buf at tail with HWO clear means the slot
		 * was already retired — late/duplicate DEPEVT (see poll grace note
		 * in udc_dwc3_in_poll_retire_ep).  Not ENOBUFS.
		 */
		if (!udc_dwc3_trb_hwo(trb)) {
			LOG_DBG("EP 0x%02x: duplicate DEPEVT hint at tail %u (inprog=%d)",
				ep_data->cfg.addr, ep_data->tail, from_inprog);
			k_work_submit(&ep_data->work);
			return;
		}

#if defined(CONFIG_UDC_DWC3_WEDGE_LOG)
		if (udc_dwc3_ep_sm_is_cpu(ep_data)) {
			udc_dwc3_wedge_snap(dev, ep_data,
					    from_inprog ? "done-empty-inprog" : "done-empty");
			atomic_inc(&udc_dwc3_enobufs_count);
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
			return;
		}
#endif

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		if (ep_data->cfg.addr == 0x01) {
			const uint32_t empty_total =
				(uint32_t)atomic_inc(&udc_dwc3_out_done_empty) + 1U;
			const bool verbose = udc_dwc3_diag_verbose(&udc_dwc3_out_done_empty_log_seq);

			if (verbose) {
				udc_dwc3_xfer_trace("OUT-DONE-EMPTY", ep_data,
						    from_inprog ? "inprog" : "complete");
				udc_dwc3_log_trb_ring(ep_data, "out-done-empty");
			}

			LOG_ERR("xfer-done on EP 0x%02x with empty TRB at tail %u "
				"(head %u full %d skip %u link_hwo=%d empty_total=%u%s)",
				ep_data->cfg.addr, ep_data->tail, ep_data->head,
				ep_data->full, ep_data->skip_xfer_done_count,
				udc_dwc3_link_trb_hwo(ep_data), empty_total,
				verbose ? "" : " (ring dump suppressed)");
			atomic_inc(&udc_dwc3_enobufs_count);
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
			return;
		}
#endif

		LOG_ERR("xfer-done on EP 0x%02x with empty TRB at tail %u "
			"(head %u full %d skip %u link_hwo=%d)",
			ep_data->cfg.addr, ep_data->tail, ep_data->head,
			ep_data->full, ep_data->skip_xfer_done_count,
			!!(ep_data->trb_buf[CONFIG_UDC_DWC3_TRB_NUM - 1U].ctrl &
			    UDC_DWC3_TRB_CTRL_HWO));
		atomic_inc(&udc_dwc3_enobufs_count);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return;
	}

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (udc_dwc3_acm_diag_ep(ep_data->cfg.addr)) {
		const uint32_t tail = ep_data->tail;
		const volatile struct udc_dwc3_trb *const retiring = &ep_data->trb_buf[tail];

		udc_dwc3_link_check(ep_data, "done-entry");
		if (!udc_dwc3_link_trb_valid(ep_data)) {
			LOG_WRN("XFER-TRACE XFER-DONE ep=0x%02x tail=%u ctl=0x%08x "
				"sts=0x%08x addr=0x%08x chn=%d lst=%d",
				ep_data->cfg.addr, tail, retiring->ctrl, retiring->status,
				retiring->addr_lo, chn, lst);
		}
	}
#endif

	/* Do not retire a TRB until hardware has released ownership. */
	if (udc_dwc3_trb_hwo(trb)) {
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		udc_dwc3_xfer_trace("DONE-HWO", ep_data,
				    from_inprog ? "inprog" : "complete");
		if (ep_data->cfg.addr == 0x01) {
			atomic_inc(&udc_dwc3_out_acct_defer_hwo);
		}
#endif
		/*
		 * OUT run-dry recovery: DWC3 emits an extra XferInProgress when
		 * it drains the ring and finds the next TRB CPU-owned; it then
		 * pauses the (still-started) transfer.  The event arrives with
		 * tail pointing at a freshly-armed, still-HWO TRB — nothing to
		 * reclaim.  The databook resume (DepUpdateXfer) is unreliable
		 * here: issued in the park transition window it is silently
		 * dropped (CMDACT clears but HW never fetches), and with the UVC
		 * IN event stream RTL-masked there is no later event to retry on,
		 * so the endpoint wedges.  Recycle the transfer resource instead
		 * (EndXfer + StartXfer at the still-armed tail), which is not
		 * subject to the resume-window drop.
		 */
		if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr) && ep_data->xfer_active &&
		    udc_dwc3_ring_data_hwo_mask(ep_data) != 0U) {
#if defined(CONFIG_UDC_DWC3_EP_SM)
			if (udc_dwc3_ep_sm_is_cpu(ep_data)) {
				(void)udc_dwc3_doorbell_issue(dev, ep_data,
							      UDC_DWC3_DB_UPDATE_VERIFY);
			} else {
				udc_dwc3_out_rundry_restart(dev, ep_data);
			}
#else
			udc_dwc3_out_rundry_restart(dev, ep_data);
#endif
		}
		k_work_submit(&ep_data->work);
		return;
	}

	/*
	 * Latch the HW writeback BEFORE pop_trb(), which calls udc_dwc3_trb_clear()
	 * and zeroes the slot.  The OUT length below is derived from the residual
	 * BUFSIZ in this field; reading it after the clear yields 0, i.e. "the host
	 * filled the whole buffer", so every OUT transfer over-reported its length
	 * and the class layer was handed the untouched tail of the buffer as if it
	 * were received data.  On CDC ACM that stale tail is previously transmitted
	 * shell output, which the shell then executed as input (its own prompt and
	 * replies coming back as "command not found"), while the oversized length
	 * overran the RX ring ("RX ring buffer full" / "RX buffer to small").
	 * HWO is already clear at this point, so the writeback is final.
	 */
	trb_status_done = trb->status;

	/* Clear the TRB that triggered the event */
	buf = udc_dwc3_pop_trb(dev, ep_data);
	if (buf == NULL) {
		atomic_inc(&udc_dwc3_enobufs_count);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return;
	}

	if (buf == UDC_DWC3_ZLP_TRB_MARKER) {
		buf = ep_data->chain_buf;
		ep_data->chain_buf = NULL;
		ep_data->absorb_cdc_zlp = true;
		if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
			ep_data->xfer_active = false;
		}
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		udc_dwc3_trace_xfer_inactive(ep_data, "zlp-marker done");
#endif
		if (buf == NULL) {
			LOG_ERR("internal ZLP complete but no chained buffer");
			k_work_submit(&ep_data->work);
			return;
		}
	} else if (chn && !lst) {
		ep_data->chain_buf = buf;
		if (!udc_dwc3_try_retire_chained_zlp(dev, ep_data)) {
			LOG_DBG("CHN TRB retired, waiting for chained ZLP");
			k_work_submit(&ep_data->work);
			return;
		}
		buf = ep_data->chain_buf;
		ep_data->chain_buf = NULL;
	}

	/*
	 * Only LST terminates the transfer here.  The historical NORMAL_ZLP term
	 * read trb->ctrl after pop_trb() had already cleared the slot, so it was
	 * always false; the IN re-arm path below was tuned around that.  Enabling
	 * it (via the latched TRBCTL) makes IN absorb the CDC ACM flush ZLP and
	 * drop xfer_active early, which stops shell output reaching the host --
	 * so it stays disabled deliberately.
	 */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr) && lst) {
		ep_data->absorb_cdc_zlp = true;
		ep_data->xfer_active = false;
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		udc_dwc3_trace_xfer_inactive(ep_data, "lst/zlp ctl done");
#endif
	}

	atomic_inc(&udc_dwc3_norm_done_count);

	LOG_DBG("XFER_DONE_NORM: EP 0x%02x, data %p", ep_data->cfg.addr, (void *)buf->data);

#if defined(CONFIG_UDC_DWC3_DMA_SLOT_DIAG)
	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
		udc_dwc3_dma_slot_note(ep_data->cfg.addr, UDC_DWC3_DMA_OUT_DONE,
				       buf->data, buf->len);
	} else {
		udc_dwc3_dma_slot_note(ep_data->cfg.addr, UDC_DWC3_DMA_IN_DONE,
				       buf->data, buf->len);
	}
#endif

	udc_dwc3_on_xfer_done(dev, ep_data);

#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
	/* Forward progress: spend the tier-5 retry budget per wedge, not per boot */
	ep_data->tier5_requeues = 0U;
#endif

	/* For buffers coming from the host, update the size actually received */
	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
		const uint32_t residual =
			FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb_status_done);

		buf->len = (residual <= buf->size) ? (buf->size - residual) : 0U;
	}

	ret = udc_submit_ep_event(dev, buf, 0);
	if (ret != 0) {
		LOG_ERR("Failed to submit buffer %p: %d", buf, ret);
	}

	/* We just made some room for a new buffer, check if something more to enqueue */
	k_work_submit(&ep_data->work);
}

/*
 * HW cleared HWO on the tail TRB but the DEPEVT arrived while HWO was still
 * set (DONE-HWO defer).  Reclaim from ep_worker once HW releases the slot.
 */
static bool udc_dwc3_retire_sw_done(const struct device *const dev,
				    struct udc_dwc3_ep_data *const ep_data,
				    const char *const via)
{
	const uint32_t tail = ep_data->tail;

	if (ep_data->net_buf[tail] == NULL) {
		return false;
	}

#if defined(CONFIG_UDC_DWC3_EP_SM)
	if (ep_data->sm.tier5_recovering) {
		return false;
	}
#endif

	if (udc_dwc3_trb_hwo(&ep_data->trb_buf[tail])) {
		return false;
	}

#if defined(CONFIG_UDC_DWC3_LOST_EVT_DIAG)
	if (ep_data->epn >= 0 && ep_data->epn < UDC_DWC3_DEPEVT_MAX_EPN) {
		atomic_inc(&udc_dwc3_sw_retire[ep_data->epn]);
	}
#endif

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	if (ep_data->cfg.addr == 0x01) {
		udc_dwc3_out_acct_note_silent(dev, ep_data, via);
	} else {
		LOG_WRN("XFER-TRACE SW-RETIRE ep=0x%02x tail=%u via=%s",
			ep_data->cfg.addr, tail, via);
	}
#endif

	udc_dwc3_on_xfer_done_norm(dev,
		USB_EP_DIR_IS_IN(ep_data->cfg.addr)
			? UDC_DWC3_DEPEVT_XFERCOMPLETE(ep_data->epn)
			: UDC_DWC3_DEPEVT_XFERINPROGRESS(ep_data->epn));

	return true;
}

static unsigned udc_dwc3_retire_sw_done_eps(const struct device *const dev,
					    const char *const via)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	unsigned retired = 0U;
	int i;

	for (i = 1; i < cfg->num_in_eps; i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[i];

		if (ep_data->trb_buf == NULL) {
			continue;
		}

		while (udc_dwc3_retire_sw_done(dev, ep_data, via)) {
			retired++;
		}
	}

	for (i = 1; i < cfg->num_out_eps; i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[i];

		if (ep_data->trb_buf == NULL) {
			continue;
		}

		while (udc_dwc3_retire_sw_done(dev, ep_data, via)) {
			retired++;
		}
	}

	return retired;
}

#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
/*
 * Periodic recovery poll for dropped IN completions.
 *
 * On this SoC the bulk IN XferComplete event is not always delivered to the CPU
 * event ring: under multi-stream load (e.g. two UVC IN streams sharing the
 * controller with the CDC IN pipe) an armed IN TRB is filled and released by HW
 * (HWO cleared) but no DEPEVT is posted (GEVNTCOUNT stays 0).  Because the UDC
 * only advances an endpoint from an event, the transfer never retires: the
 * class layer keeps its "transfer busy" flag set, stops enqueuing, and the pipe
 * wedges even though the CPU is otherwise healthy (observed on-core: one IN TRB
 * outstanding forever, watchdog still logging).
 *
 * udc_dwc3_retire_sw_done_eps() already reclaims any endpoint whose tail TRB has
 * had HWO cleared by HW without a matching event — exactly the dropped-event
 * case — so run it on a light periodic cadence.  It executes on the system
 * workqueue, serialised with the event/endpoint workers, so it needs no extra
 * locking.  When events are delivered normally this poll finds nothing (a cheap
 * HWO test per endpoint) and costs only the wakeup.
 */
#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL) && !defined(CONFIG_UDC_DWC3_EP_SM)
static atomic_t udc_dwc3_in_poll_recovered;

static bool udc_dwc3_in_poll_slot_eligible(const struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t tail = ep_data->tail;

	return ep_data->net_buf[tail] != NULL &&
	       !udc_dwc3_trb_hwo(&ep_data->trb_buf[tail]);
}

/*
 * Grace-gated reclaim of a single endpoint's dropped IN completion.
 *
 * The plain retire (udc_dwc3_retire_sw_done) fires as soon as HW has cleared
 * HWO, which races a completion event that is merely late rather than lost: the
 * poll pops the slot, then the real XferComplete arrives and hits an empty tail
 * ("xfer-done ... empty TRB" error, seen in con5.out).  Wait until the same tail
 * slot is still eligible a full poll interval later before reclaiming it -- by
 * then a late event would have retired it -- then drain any further slots HW has
 * already released (their events would have arrived alongside the first, so they
 * are genuinely lost, not late).
 */
static unsigned udc_dwc3_in_poll_retire_ep(const struct device *const dev,
					   struct udc_dwc3_ep_data *const ep_data)
{
	unsigned retired = 0U;

	if (ep_data->trb_buf == NULL) {
		return 0U;
	}

	if (!udc_dwc3_in_poll_slot_eligible(ep_data)) {
		ep_data->poll_grace_armed = false;
		return 0U;
	}

	if (!ep_data->poll_grace_armed || ep_data->poll_grace_tail != ep_data->tail) {
		ep_data->poll_grace_armed = true;
		ep_data->poll_grace_tail = ep_data->tail;
		return 0U;
	}

	ep_data->poll_grace_armed = false;
	while (udc_dwc3_retire_sw_done(dev, ep_data, "in-poll")) {
		retired++;
	}

	return retired;
}

static unsigned udc_dwc3_in_poll_retire_graced(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	unsigned retired = 0U;

	for (int i = 1; i < cfg->num_in_eps; i++) {
		retired += udc_dwc3_in_poll_retire_ep(dev, &cfg->ep_data_in[i]);
	}
	for (int i = 1; i < cfg->num_out_eps; i++) {
		retired += udc_dwc3_in_poll_retire_ep(dev, &cfg->ep_data_out[i]);
	}

	return retired;
}
#endif /* !CONFIG_UDC_DWC3_EP_SM */

#if defined(CONFIG_UDC_DWC3_LOST_EVT_DIAG)
static atomic_t udc_dwc3_lost_evt_samples;
static atomic_t udc_dwc3_lost_evt_count_nz;
static atomic_t udc_dwc3_lost_evt_masked;
static atomic_t udc_dwc3_lost_evt_count_max;

/*
 * Sample the event ring at the moment the poll recovers a completion that
 * never arrived as an event.
 *
 * Every recovery mechanism in this driver exists because a TRB is released by
 * HW without a DEPEVT reaching the CPU, but "never posted" and "posted and
 * never collected" call for completely different fixes and nothing so far has
 * distinguished them.  Three outcomes separate cleanly here:
 *
 *   count>0, mask set    the event is sitting in the ring with interrupts
 *                        masked -- a lost interrupt edge, ours to fix
 *   count>0, mask clear  posted, ring not drained yet -- worker latency
 *   count==0             HW released the TRB and posted nothing at all,
 *                        which no amount of driver work can recover
 *
 * The counters are sampled rather than logged per event: recoveries run into
 * the hundreds per minute under streaming load, and logging each one would
 * perturb the very timing being measured.
 */
static void udc_dwc3_lost_evt_sample(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t count = sys_read32(base + UDC_DWC3_GEVNTCOUNT(0));
	const uint32_t siz = sys_read32(base + UDC_DWC3_GEVNTSIZ(0));

	atomic_inc(&udc_dwc3_lost_evt_samples);

	if (count > 0U) {
		atomic_inc(&udc_dwc3_lost_evt_count_nz);
	}

	if (siz & UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK) {
		atomic_inc(&udc_dwc3_lost_evt_masked);
	}

	if ((atomic_val_t)count > atomic_get(&udc_dwc3_lost_evt_count_max)) {
		atomic_set(&udc_dwc3_lost_evt_count_max, (atomic_val_t)count);
	}
}

static void udc_dwc3_lost_evt_report_eps(const struct device *dev);

static void udc_dwc3_lost_evt_report(const struct device *const dev)
{
	static int64_t last_log;
	static atomic_val_t last_samples;
	const atomic_val_t samples = atomic_get(&udc_dwc3_lost_evt_samples);
	const int64_t now = k_uptime_get();

	if (samples == last_samples || (now - last_log) < 1000) {
		return;
	}

	last_log = now;
	last_samples = samples;

	LOG_WRN("lost-evt: polls=%ld count_nz=%ld masked=%ld count_max=%ld",
		(long)samples, (long)atomic_get(&udc_dwc3_lost_evt_count_nz),
		(long)atomic_get(&udc_dwc3_lost_evt_masked),
		(long)atomic_get(&udc_dwc3_lost_evt_count_max));

	udc_dwc3_lost_evt_report_eps(dev);
}

/*
 * Per-endpoint breakdown that separates a missing event from a late one.
 *
 * An event that is merely late still arrives after the poll has retired the
 * descriptor, and the driver counts that arrival as stale (skip_xfer_done).
 * So a stale count tracking the poll-retire count means the events are being
 * delivered too slowly -- ours to fix -- while poll retires with no matching
 * stale arrivals mean the event was never generated at all.
 */
static void udc_dwc3_lost_evt_report_eps(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	for (int dir = 0; dir < 2; dir++) {
		const int num = dir ? cfg->num_in_eps : cfg->num_out_eps;

		for (int i = 1; i < num; i++) {
			struct udc_dwc3_ep_data *const ep_data =
				dir ? &cfg->ep_data_in[i] : &cfg->ep_data_out[i];
			const int epn = ep_data->epn;
			long sw;

			if (ep_data->trb_buf == NULL || epn < 0 ||
			    epn >= UDC_DWC3_DEPEVT_MAX_EPN) {
				continue;
			}

			sw = (long)atomic_get(&udc_dwc3_sw_retire[epn]);
			if (sw == 0) {
				continue;
			}

			LOG_WRN("lost-evt ep=0x%02x: evt_cmpl=%ld evt_inprog=%ld "
				"sw_retire=%ld stale=%u",
				ep_data->cfg.addr,
				(long)atomic_get(&udc_dwc3_depevt_complete[epn]),
				(long)atomic_get(&udc_dwc3_depevt_inprog[epn]),
				sw, ep_data->skip_xfer_done_count);
		}
	}
}
#endif /* CONFIG_UDC_DWC3_LOST_EVT_DIAG */

static void udc_dwc3_in_poll_worker(struct k_work *const work)
{
	struct k_work_delayable *const dwork = k_work_delayable_from_work(work);
	struct udc_dwc3_data *const priv =
		CONTAINER_OF(dwork, struct udc_dwc3_data, in_poll_work);
	const struct device *const dev = priv->dev;

#if defined(CONFIG_UDC_DWC3_EP_SM)
	const unsigned retired = udc_dwc3_ep_sm_poll_all(dev);

#if defined(CONFIG_UDC_DWC3_LOST_EVT_DIAG)
	if (retired > 0U) {
		udc_dwc3_lost_evt_sample(dev);
	}
	udc_dwc3_lost_evt_report(dev);
#endif

#if defined(CONFIG_UDC_DWC3_EP0_RESCUE)
	{
		/*
		 * EP0 defers a completion whose TRB is still owned and relies on
		 * udc_dwc3_ep0_poll_all() to pick it up -- but that only runs at
		 * the end of an event batch, so it needs another event to arrive
		 * before it can rescue anything.  On a controller that drops
		 * completions, a deferred control transfer can therefore sit
		 * until unrelated traffic happens past, and if the bus goes quiet
		 * (both video streams being torn down and restarted, say) nothing
		 * comes, and the host fails the transfer at its five second
		 * timeout -- the -110 that kills UVC negotiation.
		 *
		 * Tick it independently of events. Deliberately slow: an earlier
		 * attempt to drive this from the 500us bulk poll starved the
		 * endpoint workers, and rescuing within tens of milliseconds is
		 * still orders of magnitude inside the host's timeout.
		 */
		static int64_t last_ep0_rescue;
		const int64_t now = k_uptime_get();

		if ((now - last_ep0_rescue) >= CONFIG_UDC_DWC3_EP0_RESCUE_MS) {
			last_ep0_rescue = now;
			udc_dwc3_ep0_poll_all(dev);
		}
	}
#endif

	if (atomic_get(&priv->bulk_eps_live) > 0) {
		static int64_t last_log;
		static atomic_val_t last_recovered;
		static atomic_val_t last_retook;
		static atomic_val_t last_backoff;
		static atomic_val_t last_recycled;
		static atomic_val_t last_stuck;
		struct udc_dwc3_in_recovery_stats stats;
		const int64_t now = k_uptime_get();

		udc_dwc3_ep_sm_in_recovery_get(&stats);

		if ((stats.poll_recovered != last_recovered ||
		     stats.start_retook != last_retook ||
		     stats.start_backoff != last_backoff ||
		     stats.start_recycled != last_recycled ||
		     stats.start_stuck != last_stuck) &&
		    (now - last_log >= 1000)) {
			LOG_WRN("in-recovery: lost-compl=%ld start-retook=%ld "
				"start-backoff=%ld start-recycled=%ld start-stuck=%ld",
				(long)stats.poll_recovered, (long)stats.start_retook,
				(long)stats.start_backoff, (long)stats.start_recycled,
				(long)stats.start_stuck);
			last_log = now;
			last_recovered = stats.poll_recovered;
			last_retook = stats.start_retook;
			last_backoff = stats.start_backoff;
			last_recycled = stats.start_recycled;
			last_stuck = stats.start_stuck;
		}

		ARG_UNUSED(retired);

		udc_dwc3_ep_sm_watchdog(dev);

		k_work_reschedule(&priv->in_poll_work,
				  K_USEC(CONFIG_UDC_DWC3_IN_COMPLETION_POLL_INTERVAL_US));
	} else if (udc_dwc3_ep_sm_any_pending(dev)) {
		/*
		 * bulk_eps_live gates every recovery lever: poll_all() and
		 * ep_sm_depevt() both bail out when it is zero, and this worker
		 * stops rescheduling itself.  Reaching that state while a CPU
		 * endpoint still holds a queued buffer means nothing will ever
		 * retire it -- the pipe is dead until some unrelated endpoint
		 * gets enabled again.  Keep polling so the buffer can retire,
		 * and say so once.
		 */
		if (!priv->poll_live_zero_reported) {
			priv->poll_live_zero_reported = true;
			LOG_ERR("EP-SM: poll stopped with live=0 while CPU eps still pending");
		}

		udc_dwc3_ep_sm_watchdog(dev);

		k_work_reschedule(&priv->in_poll_work,
				  K_USEC(CONFIG_UDC_DWC3_IN_COMPLETION_POLL_INTERVAL_US));
	}
#else
	const unsigned retired = udc_dwc3_in_poll_retire_graced(dev);

	if (retired > 0U) {
		atomic_add(&udc_dwc3_in_poll_recovered, retired);
	}

	/*
	 * Rate-limited health line covering all three IN recovery signals:
	 *   lost-compl   - completions HW finished but never signalled (this poll)
	 *   start-retook - StartXfer doorbells re-nudged into taking (inline)
	 *   start-recycled - stranded starts recovered via EndXfer(ForceRM)+StartXfer
	 *   start-stuck  - StartXfer arms no lever could recover
	 * Emitted at most once per second and only when a counter advances.
	 */
	{
		static int64_t last_log;
		static atomic_val_t last_recovered;
		static atomic_val_t last_retook;
		static atomic_val_t last_recycled;
		static atomic_val_t last_stuck;
		static atomic_val_t last_backoff;
		const atomic_val_t recovered = atomic_get(&udc_dwc3_in_poll_recovered);
		const atomic_val_t retook = atomic_get(&udc_dwc3_in_start_retook);
		const atomic_val_t backoff = atomic_get(&udc_dwc3_in_start_backoff);
		const atomic_val_t recycled = atomic_get(&udc_dwc3_in_start_recycled);
		const atomic_val_t stuck = atomic_get(&udc_dwc3_in_start_exhausted);
		const int64_t now = k_uptime_get();

		if ((recovered != last_recovered || retook != last_retook ||
		     backoff != last_backoff || recycled != last_recycled ||
		     stuck != last_stuck) &&
		    (now - last_log >= 1000)) {
			LOG_WRN("in-recovery: lost-compl=%ld start-retook=%ld "
				"start-backoff=%ld start-recycled=%ld start-stuck=%ld",
				(long)recovered, (long)retook, (long)backoff,
				(long)recycled, (long)stuck);
			last_log = now;
			last_recovered = recovered;
			last_retook = retook;
			last_backoff = backoff;
			last_recycled = recycled;
			last_stuck = stuck;
		}
	}

	k_work_reschedule(&priv->in_poll_work,
			  K_USEC(CONFIG_UDC_DWC3_IN_COMPLETION_POLL_INTERVAL_US));
#endif
}
#endif /* CONFIG_UDC_DWC3_IN_COMPLETION_POLL */

#define NORMAL_EP(n, fn) fn(n + 2)

static void udc_dwc3_handle_event(const struct device *const dev, const uint32_t evt)
{
	switch (evt) {
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(0):
		udc_dwc3_note_depevt(evt, false);
		LOG_DBG("DEPEVT_XFERCOMPLETE(0)");
		udc_dwc3_on_ctrl_out(dev);
		break;
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(1):
		udc_dwc3_note_depevt(evt, false);
		LOG_DBG("DEPEVT_XFERCOMPLETE(1)");
		udc_dwc3_on_ctrl_in(dev);
		break;
	case LISTIFY(30, NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERCOMPLETE):
		udc_dwc3_note_depevt(evt, false);
		LOG_DBG("DEPEVT_XFERCOMPLETE");
#if defined(CONFIG_UDC_DWC3_EP_SM)
		if (udc_dwc3_ep_sm_depevt(dev, evt)) {
			break;
		}
#endif
		udc_dwc3_on_xfer_done_norm(dev, evt);
		break;
	case LISTIFY(30, NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERINPROGRESS): {
		const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);

		udc_dwc3_note_depevt(evt, true);

		/*
		 * IN endpoints: INPROGRESS is a burst within one TRB — only
		 * XFERCOMPLETE retires it (premature pop wedged ACM IN / UVC).
		 * OUT endpoints: still complete on INPROGRESS (host short-packet
		 * delivery); ignoring it leaves rx_busy stuck (phase0 @ ~3s).
		 */
		if (epn & 1) {
#if defined(CONFIG_UDC_DWC3_IN_INPROG_RETIRE) && defined(CONFIG_UDC_DWC3_EP_SM)
			const struct udc_dwc3_config *const cfg = dev->config;
			struct udc_dwc3_ep_data *const in_ep = &cfg->ep_data_in[epn >> 1];

			/*
			 * On this controller XferComplete never arrives for the
			 * CPU-managed IN endpoints -- measured across a full run,
			 * evt_cmpl was 0 while evt_inprog matched the transfer
			 * count one for one.  Ignoring XferInProgress therefore
			 * discards the only completion signal there is, and every
			 * transfer has to be rediscovered a poll interval later.
			 *
			 * Retire here only once HW has actually released the tail,
			 * which is the same test the poll applies; a burst within a
			 * still-owned TRB fails it and is ignored exactly as before,
			 * so the premature pop this branch was guarding against
			 * still cannot happen.  Hardware-managed endpoints (UVC)
			 * keep the original behaviour untouched.
			 */
			if (udc_dwc3_ep_sm_is_cpu(in_ep) && in_ep->trb_buf != NULL &&
			    in_ep->net_buf[in_ep->tail] != NULL &&
			    !udc_dwc3_trb_hwo(&in_ep->trb_buf[in_ep->tail])) {
				if (udc_dwc3_ep_sm_depevt(dev, evt)) {
					break;
				}
				udc_dwc3_on_xfer_done_norm(dev, evt);
				break;
			}
#endif
			LOG_DBG("DEPEVT_XFERINPROGRESS epn=%u IN (no pop)", epn);
		} else {
			LOG_DBG("DEPEVT_XFERINPROGRESS epn=%u OUT", epn);
#if defined(CONFIG_UDC_DWC3_EP_SM)
			if (udc_dwc3_ep_sm_depevt(dev, evt)) {
				break;
			}
#endif
			udc_dwc3_on_xfer_done_norm(dev, evt);
		}
		break;
	}
	case UDC_DWC3_DEPEVT_XFERNOTREADY(0):
	case UDC_DWC3_DEPEVT_XFERNOTREADY(1):
		udc_dwc3_on_xfer_not_ready(dev, evt);
		break;
#if defined(CONFIG_UDC_DWC3_OUT_NOTREADY_RETAKE)
	case LISTIFY(30, NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERNOTREADY):
		udc_dwc3_on_xfer_not_ready_norm(dev, evt);
		break;
#endif
	case UDC_DWC3_DEVT_DISCONNEVT:
		LOG_INF("DEVT_DISCONNEVT");
		break;
	case UDC_DWC3_DEVT_USBRST:
		LOG_INF("DEVT_USBRST");
		udc_dwc3_on_usb_reset(dev);
		break;
	case UDC_DWC3_DEVT_CONNECTDONE:
		LOG_DBG("DEVT_CONNECTDONE");
		udc_dwc3_on_connect_done(dev);
		break;
	case UDC_DWC3_DEVT_ULSTCHNG:
		LOG_INF("DEVT_ULSTCHNG");
		udc_dwc3_on_link_state_event(dev);
		break;
	case UDC_DWC3_DEVT_WKUPEVT:
		LOG_INF("DEVT_WKUPEVT (resume)");
		break;
	case UDC_DWC3_DEVT_SUSPEND:
		LOG_INF("DEVT_SUSPEND (link entering low power)");
		break;
	case UDC_DWC3_DEVT_SOF:
		LOG_DBG("DEVT_SOF");
		break;
	case UDC_DWC3_DEVT_CMDCMPLT:
		LOG_DBG("DEVT_CMDCMPLT");
		break;
	case UDC_DWC3_DEVT_VNDRDEVTSTRCVED:
		LOG_DBG("DEVT_VNDRDEVTSTRCVED");
		break;
	case UDC_DWC3_DEVT_ERRTICERR:
		LOG_ERR("DEVT_ERRTICERR");
		CODE_UNREACHABLE;
		break;
	case UDC_DWC3_DEVT_EVNTOVERFLOW: {
		struct udc_dwc3_data *const priv = udc_get_private(dev);
		const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

		LOG_ERR("DEVT_EVNTOVERFLOW");
		atomic_inc(&udc_dwc3_evt_overflow_count);
		priv->evt_next = 0U;
		sys_write32(0, base + UDC_DWC3_GEVNTCOUNT(0));
		break;
	}
	default:
		LOG_ERR("unhandled event: 0x%x", evt);
		CODE_UNREACHABLE;
	}
}

static void udc_dwc3_irq_handler(void *const ptr)
{
	const struct device *const dev = ptr;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	if (k_is_in_isr()) {
		atomic_inc(&udc_dwc3_hwirq_count);
	} else {
		atomic_inc(&udc_dwc3_poll_count);
	}

	/*
	 * Mask the event interrupt and defer the actual event processing to a
	 * workqueue. udc_dwc3_handle_event() takes the UDC mutex (and submits
	 * other work), which is illegal from ISR context. The worker re-enables
	 * the interrupt once the event buffer is drained.
	 *
	 * The mask store is a posted MMIO write on this SoC.  If it does not
	 * land before the ISR returns while the event IRQ is still asserted
	 * (more events posting under a sustained IN + UVC load), the ISR
	 * re-enters immediately and storms, pegging the CPU at interrupt level
	 * and starving every thread (console dead, no log output, device
	 * appears hung though ISRs still run).  Confirm the mask is actually set
	 * via read-back before returning so the storm cannot start.
	 */
	do {
		sys_set_bits(base + UDC_DWC3_GEVNTSIZ(0),
			     UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
		barrier_dmem_fence_full();
	} while ((sys_read32(base + UDC_DWC3_GEVNTSIZ(0)) &
		  UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK) == 0U);

	k_work_submit(&priv->event_work);
}

/* Drain the DWC3 event buffer from thread context (mutex-safe). */
static void udc_dwc3_event_worker(struct k_work *const work)
{
	struct udc_dwc3_data *const priv = CONTAINER_OF(work, struct udc_dwc3_data, event_work);
	const struct device *const dev = priv->dev;
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	/*
	 * Drain every event currently posted to the ring.
	 */
	while (sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) > 0U) {
		uint32_t evt = cfg->evt_buf[priv->evt_next];

		/*
		 * GEVNTCOUNT says an event has been counted, which is not the same
		 * as its word having reached the ring: the write is posted, and on
		 * this SoC it can trail the counter.  Zero is never a valid event,
		 * so consuming it would ack an entry that has not arrived and leave
		 * this index permanently ahead of hardware -- from then on every
		 * read returns zero, no real event is ever seen again, and the
		 * device is dead while still logging.  Wait for the write instead,
		 * and if it has not landed leave the entry unacked; the resubmit
		 * below picks it up on the next pass.
		 */
		if (evt == 0U) {
			for (unsigned int i = 0U; i < UDC_DWC3_EVT_LANDING_STEPS; i++) {
				k_busy_wait(1);
				evt = cfg->evt_buf[priv->evt_next];
				if (evt != 0U) {
					break;
				}
			}

			if (evt == 0U) {
				atomic_inc(&udc_dwc3_evt_unlanded_count);
				break;
			}
		}

		atomic_inc(&udc_dwc3_evt_count);
		udc_dwc3_handle_event(dev, evt & UDC_DWC3_EVT_MASK);

		/*
		 * Clear the slot so the next pass around the ring can tell an
		 * arrived event from a stale one.
		 */
		cfg->evt_buf[priv->evt_next] = 0U;

		/* Move to next event entry for both hardware and software */
		sys_write32(sizeof(uint32_t), base + UDC_DWC3_GEVNTCOUNT(0));
		udc_dwc3_ring_inc(&priv->evt_next, CONFIG_UDC_DWC3_EVENTS_NUM);
	}

	/*
	 * EP0 HWO recovery: run once per event batch (~1 ms on LiteX poll), not
	 * from the 500 us bulk poll (that starved endpoint workers and amplified
	 * dup/poll_ret counts).
	 */
	udc_dwc3_ep0_poll_all(dev);

	/*
	 * Ring drained: allow further interrupts.  The unmask MUST be confirmed
	 * landed.  On this SoC MMIO stores are posted and can be delayed/dropped
	 * under load; an unmask that does not take effect leaves GEVNTSIZ's mask
	 * bit stuck set from the IRQ handler.  Because the event IRQ is
	 * edge-on-post, a stuck mask means no future edge is delivered, the
	 * worker is never rescheduled, and the endpoint wedges (observed
	 * on-core: GEVNTSIZ=0x80000400, GEVNTCOUNT climbing, hwirq/evt frozen; a
	 * manual unmask recovered it every time).  Read the register back and
	 * retry until the mask is actually clear.
	 */
	do {
		sys_clear_bits(base + UDC_DWC3_GEVNTSIZ(0),
			       UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
		barrier_dmem_fence_full();
	} while ((sys_read32(base + UDC_DWC3_GEVNTSIZ(0)) &
		  UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK) != 0U);

	/*
	 * If an event posted during the unmask window, the edge-on-post IRQ will
	 * NOT re-fire for it.  Re-mask and resubmit so it is drained on the next
	 * pass without relying on the IRQ.  We deliberately do NOT loop in place:
	 * an unbounded in-place re-drain monopolises the CPU under a sustained
	 * event rate (e.g. high-rate IN + UVC) and starves lower-priority threads
	 * (observed: console "RX ring buffer full", shell unresponsive, log
	 * output stalled — the device appears hung though ISRs still run).
	 * Returning here yields the workqueue between passes so the endpoint
	 * worker and shell make progress; the explicit resubmit (not the IRQ)
	 * guarantees the slipped event is still serviced, so this cannot wedge.
	 */
	if (sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) > 0U) {
		sys_set_bits(base + UDC_DWC3_GEVNTSIZ(0),
			     UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
		k_work_submit(&priv->event_work);
	}
}

static const char *udc_dwc3_linkstate_str(const uint32_t dsts)
{
	if ((dsts & UDC_DWC3_DSTS_CONNECTSPD_MASK) == UDC_DWC3_DSTS_CONNECTSPD_SS) {
		switch (dsts & UDC_DWC3_DSTS_USBLNKST_MASK) {
		case UDC_DWC3_DSTS_USBLNKST_USB3_U0:		return "U0";
		case UDC_DWC3_DSTS_USBLNKST_USB3_U1:		return "U1";
		case UDC_DWC3_DSTS_USBLNKST_USB3_U2:		return "U2";
		case UDC_DWC3_DSTS_USBLNKST_USB3_U3:		return "U3";
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS:	return "SS.Dis";
		case UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET:	return "RxDet";
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT:	return "SS.Inact";
		case UDC_DWC3_DSTS_USBLNKST_USB3_POLL:		return "Poll";
		case UDC_DWC3_DSTS_USBLNKST_USB3_RECOV:		return "Recov";
		case UDC_DWC3_DSTS_USBLNKST_USB3_HRESET:	return "HotReset";
		case UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME:	return "ResetResume";
		default:					return "U?";
		}
	}
	switch (dsts & UDC_DWC3_DSTS_USBLNKST_MASK) {
	case UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE:	return "On";
	case UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE:	return "Sleep(L1)";
	case UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE:	return "Suspend(L2)";
	case UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED:	return "Disc";
	case UDC_DWC3_DSTS_USBLNKST_USB2_RESET:		return "Reset";
	case UDC_DWC3_DSTS_USBLNKST_USB2_RESUME:	return "Resume";
	default:					return "L?";
	}
}

/*
 * DEPCFG points each IN endpoint at TxFIFO number (addr & 0x7f), so a composite
 * device reaches FIFO 4 (CDC-RAW IN).  GTXFIFOSIZn is left at the hard IP
 * defaults, and an endpoint aimed at a FIFO with no depth behind it retires its
 * TRBs while the payload never reaches the wire.  Report the layout of every
 * FIFO an endpoint can select, plus the RAM the IP actually provides.
 */
static bool udc_dwc3_fifo_snapshot_done;

static void udc_dwc3_dump_fifo_cfg(const struct device *const dev, const char *const tag)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	for (unsigned int i = 0U; i < 6U; i++) {
		const uint32_t txf = sys_read32(base + UDC_DWC3_GTXFIFOSIZ(i));

		LOG_WRN("fifosiz(%s): GTXFIFOSIZ[%u]=0x%08x start=%u dep=%u", tag, i, txf,
			(uint32_t)FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFSTADDR_MASK, txf),
			(uint32_t)FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK, txf));
	}

	LOG_WRN("fifosiz(%s): GRXFIFOSIZ[0]=0x%08x dep=%u GHWPARAMS7=0x%08x GHWPARAMS3=0x%08x",
		tag,
		sys_read32(base + UDC_DWC3_GRXFIFOSIZ(0)),
		(uint32_t)FIELD_GET(UDC_DWC3_GRXFIFOSIZ_RXFDEP_MASK,
				    sys_read32(base + UDC_DWC3_GRXFIFOSIZ(0))),
		sys_read32(base + UDC_DWC3_GHWPARAMS7),
		sys_read32(base + UDC_DWC3_GHWPARAMS3));
}

#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
static void udc_dwc3_log_pools(void)
{
	STRUCT_SECTION_FOREACH(net_buf_pool, pool) {
		const size_t total = pool->buf_count;
		const size_t avail = net_buf_get_available(pool);
		const size_t used = total - avail;
		const size_t max_used = net_buf_get_max_used(pool);
		const char *const name = (pool->name != NULL) ? pool->name : "?";

		/* Zero-sized pools are unused, not exhausted */
		if (total != 0 && avail == 0) {
			LOG_WRN("pool %s EXHAUSTED used=%zu/%zu max=%zu", name, used, total,
				max_used);
		} else {
			LOG_INF("pool %s used=%zu/%zu max=%zu", name, used, total, max_used);
		}
	}
}

static void udc_dwc3_dump_link_cfg(const struct device *const dev, const char *const tag)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t dctl = sys_read32(base + UDC_DWC3_DCTL);
	const uint32_t dsts = sys_read32(base + UDC_DWC3_DSTS);

	LOG_INF("linkcfg(%s): GUSB3PIPECTL=0x%08x GCTL=0x%08x GUSB2PHYCFG=0x%08x "
		"DCTL=0x%08x DEVTEN=0x%08x DCFG=0x%08x",
		tag,
		sys_read32(base + UDC_DWC3_GUSB3PIPECTL),
		sys_read32(base + UDC_DWC3_GCTL),
		sys_read32(base + UDC_DWC3_GUSB2PHYCFG),
		dctl,
		sys_read32(base + UDC_DWC3_DEVTEN),
		sys_read32(base + UDC_DWC3_DCFG));
	LOG_INF("linkcfg(%s): U1[init=%d accept=%d] U2[init=%d accept=%d] GTXTHRCFG=0x%08x link=%s dsts=0x%08x",
		tag,
		!!(dctl & UDC_DWC3_DCTL_INITU1ENA), !!(dctl & UDC_DWC3_DCTL_ACCEPTU1ENA),
		!!(dctl & UDC_DWC3_DCTL_INITU2ENA), !!(dctl & UDC_DWC3_DCTL_ACCEPTU2ENA),
		sys_read32(base + UDC_DWC3_GTXTHRCFG),
		udc_dwc3_linkstate_str(dsts), dsts);

	udc_dwc3_dump_fifo_cfg(dev, tag);
}

static void udc_dwc3_log_health(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t dsts = sys_read32(base + UDC_DWC3_DSTS);

	LOG_INF("health %s: hwirq=%u poll=%u evt=%u rst=%u | setup=%u ctrlin=%u ctrlout=%u "
		"norm=%u enobufs=%u | ep0 in_busy=%d out_busy=%d evtcnt=%u | "
		"ep0 fixed=%u mismatch=%u setuppend=%u trberr=%u | "
		"ssinact=%u ssrecov=%u remwk=%u | link=%s dsts=0x%08x",
		dev->name,
		(uint32_t)atomic_get(&udc_dwc3_hwirq_count),
		(uint32_t)atomic_get(&udc_dwc3_poll_count),
		(uint32_t)atomic_get(&udc_dwc3_evt_count),
		(uint32_t)atomic_get(&udc_dwc3_usbrst_count),
		(uint32_t)atomic_get(&udc_dwc3_ctrl_setup_count),
		(uint32_t)atomic_get(&udc_dwc3_ctrl_in_count),
		(uint32_t)atomic_get(&udc_dwc3_ctrl_out_count),
		(uint32_t)atomic_get(&udc_dwc3_norm_done_count),
		(uint32_t)atomic_get(&udc_dwc3_enobufs_count),
		(int)udc_ep_is_busy(&cfg->ep_data_in[0].cfg),
		(int)udc_ep_is_busy(&cfg->ep_data_out[0].cfg),
		sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)),
		(uint32_t)atomic_get(&udc_dwc3_ep0_stage_fixed),
		(uint32_t)atomic_get(&udc_dwc3_ep0_stage_mismatch),
		(uint32_t)atomic_get(&udc_dwc3_ep0_setuppending),
		(uint32_t)atomic_get(&udc_dwc3_ep0_trb_err),
		(uint32_t)atomic_get(&udc_dwc3_ss_inact_count),
		(uint32_t)atomic_get(&udc_dwc3_ss_recov_count),
		(uint32_t)atomic_get(&udc_dwc3_remwk_count),
		udc_dwc3_linkstate_str(dsts), dsts);

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	LOG_INF("health %s: out_acct push=%u evt=%u pop=%u silent=%u "
		"done_empty=%u defer_hwo=%u",
		dev->name,
		(uint32_t)atomic_get(&udc_dwc3_out_acct_push),
		(uint32_t)atomic_get(&udc_dwc3_out_acct_evt),
		(uint32_t)atomic_get(&udc_dwc3_out_acct_pop),
		(uint32_t)atomic_get(&udc_dwc3_out_acct_silent),
		(uint32_t)atomic_get(&udc_dwc3_out_done_empty),
		(uint32_t)atomic_get(&udc_dwc3_out_acct_defer_hwo));
#endif

	udc_dwc3_log_pools();
}

static void udc_dwc3_health_worker(struct k_work *const work)
{
	struct k_work_delayable *const dwork = k_work_delayable_from_work(work);
	struct udc_dwc3_data *const priv = CONTAINER_OF(dwork, struct udc_dwc3_data, health_work);

	udc_dwc3_log_health(priv->dev);

	k_work_reschedule(&priv->health_work, K_MSEC(CONFIG_UDC_DWC3_HEALTH_LOG_INTERVAL_MS));
}
#endif /* CONFIG_UDC_DWC3_HEALTH_LOG */

static void udc_dwc3_log_ep_trb_ring(struct udc_dwc3_ep_data *const ep_data)
{
	const uint8_t addr = ep_data->cfg.addr;

	LOG_ERR("STALL-TRB ep=0x%02x epn=%d busy=%d halted=%d head=%u tail=%u full=%d "
		"xferrscidx=0x%x active=%d",
		addr, ep_data->epn, (int)udc_ep_is_busy(&ep_data->cfg),
		(int)ep_data->cfg.stat.halted, ep_data->head, ep_data->tail,
		(int)ep_data->full, ep_data->xferrscidx, (int)ep_data->xfer_active);

	for (int i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		const volatile struct udc_dwc3_trb *const t = &ep_data->trb_buf[i];
		const char *mark = "";

		if (i == (int)ep_data->head) {
			mark = " HEAD";
		} else if (i == (int)ep_data->tail) {
			mark = ep_data->full ? " TAIL+FULL" : " TAIL";
		}

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		{
			struct udc_dwc3_trb_snap actual;
			struct udc_dwc3_trb_snap expect;
			const uintptr_t meta = (uintptr_t)&ep_data->trb_buf[i];

			udc_dwc3_trb_snap_read(&ep_data->trb_buf[i], &actual);
			udc_dwc3_expected_trb_snap(ep_data, (uint32_t)i, &expect);
			udc_dwc3_trb_cmp_log_slot("stall", (uint32_t)i, mark, meta,
						  ep_data->net_buf[i], &expect, &actual,
						  !udc_dwc3_trb_snap_equal(&expect, &actual));
		}
#else
		LOG_ERR("  [%d]%s ctl=0x%08x sts=0x%08x addr=0x%08x hwo=%d buf=%p",
			i, mark, t->ctrl, t->status, t->addr_lo,
			!!(t->ctrl & UDC_DWC3_TRB_CTRL_HWO),
			(void *)ep_data->net_buf[i]);
#endif
	}
}

#if defined(CONFIG_UDC_DWC3_DMA_SLOT_DIAG)
static void udc_dwc3_dma_slot_dump_stall(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	bool any_slot;

	LOG_ERR("STALL-DMA-SLOTS sram1=0x%08x size=0x%x (%u x %u B)",
		(uint32_t)UDC_DWC3_DMA_SRAM_BASE, (uint32_t)UDC_DWC3_DMA_SRAM_SIZE,
		(uint32_t)UDC_DWC3_DMA_SLOT_MAX, (uint32_t)UDC_DWC3_DMA_SLOT_BYTES);

	any_slot = false;
	for (int i = 0; i < (int)UDC_DWC3_DMA_SLOT_MAX; i++) {
		const struct udc_dwc3_dma_slot *const slot = &udc_dwc3_dma_slots[i];
		const uint32_t slot_base =
			(uint32_t)UDC_DWC3_DMA_SRAM_BASE + (uint32_t)i * UDC_DWC3_DMA_SLOT_BYTES;

		if (slot->out_enq == 0U && slot->in_enq == 0U &&
		    slot->out_done == 0U && slot->in_done == 0U) {
			continue;
		}

		any_slot = true;
		LOG_ERR("  [%d] @0x%08x last_ep=0x%02x last=%s "
			"out_enq=%u in_enq=%u out_done=%u in_done=%u scrub_armed=%d",
			i, slot_base, slot->last_ep, udc_dwc3_dma_op_str(slot->last_op),
			slot->out_enq, slot->in_enq, slot->out_done, slot->in_done,
			(int)slot->scrub_armed);
	}

	if (!any_slot) {
		LOG_ERR("  (no sram1 bulk DMA activity recorded)");
	}

	for (int i = 0; i < cfg->num_in_eps; i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[i];

		if (ep_data->trb_buf == NULL) {
			continue;
		}

		for (int t = 0; t < CONFIG_UDC_DWC3_TRB_NUM; t++) {
			const struct udc_dwc3_trb trb = ep_data->trb_buf[t];
			const uintptr_t trb_dma = (uintptr_t)trb.addr_lo;
			const int idx = udc_dwc3_dma_slot_index(trb_dma);
			const bool hwo = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_HWO);

			if (!hwo ||
			    (trb.ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK) ==
				    UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB ||
			    idx < 0) {
				continue;
			}

			const struct udc_dwc3_dma_slot *const slot =
				&udc_dwc3_dma_slots[idx];
			const uint8_t *const live = (const uint8_t *)trb_dma;
			bool scrub_intact = true;

			for (size_t b = 0; b < UDC_DWC3_DMA_SCRUB_SNAP; b++) {
				if (live[b] != UDC_DWC3_DMA_SCRUB_BYTE) {
					scrub_intact = false;
					break;
				}
			}

			LOG_ERR("STALL-SCRUB ep=0x%02x trb[%d] dma=0x%08x slot=%d "
				"scrub_armed=%d intact=%d",
				ep_data->cfg.addr, t, (uint32_t)trb_dma, idx,
				(int)slot->scrub_armed, (int)scrub_intact);
			udc_dwc3_dma_log_bytes("  scrub-at-enq", slot->scrub_snapshot,
					       UDC_DWC3_DMA_SCRUB_SNAP);
			udc_dwc3_dma_log_bytes("  live-now    ", live, UDC_DWC3_DMA_SCRUB_SNAP);
		}
	}
}
#endif /* CONFIG_UDC_DWC3_DMA_SLOT_DIAG */

#if defined(CONFIG_UDC_DWC3_WEDGE_LOG)
void udc_dwc3_stall_snapshot(const struct device *const dev);

static void udc_dwc3_wedge_snap_ep(struct udc_dwc3_ep_data *const ep_data,
				   const char *const reason)
{
	const uint32_t tail = ep_data->tail;
	const volatile struct udc_dwc3_trb *const t = &ep_data->trb_buf[tail];
	const bool hwo = !!(t->ctrl & UDC_DWC3_TRB_CTRL_HWO);

#if defined(CONFIG_UDC_DWC3_EP_SM)
	LOG_WRN("WEDGE %s ep=0x%02x head=%u tail=%u full=%d active=%d sm=%d "
		"skip=%u chain=%p",
		reason, ep_data->cfg.addr, ep_data->head, tail, ep_data->full,
		ep_data->xfer_active, (int)ep_data->sm.state,
		ep_data->skip_xfer_done_count, (void *)ep_data->chain_buf);
#else
	LOG_WRN("WEDGE %s ep=0x%02x head=%u tail=%u full=%d active=%d skip=%u "
		"chain=%p",
		reason, ep_data->cfg.addr, ep_data->head, tail, ep_data->full,
		ep_data->xfer_active, ep_data->skip_xfer_done_count,
		(void *)ep_data->chain_buf);
#endif
	LOG_WRN("WEDGE %s ep=0x%02x tail[%u] ctl=0x%08x sts=0x%08x hwo=%d "
		"buf=%p",
		reason, ep_data->cfg.addr, tail, t->ctrl, t->status, hwo,
		(void *)ep_data->net_buf[tail]);
}

static void udc_dwc3_wedge_snap(const struct device *const dev,
				struct udc_dwc3_ep_data *const ep_data,
				const char *const reason)
{
	udc_dwc3_wedge_snap_ep(ep_data, reason);
	udc_dwc3_stall_snapshot(dev);
}
#endif /* CONFIG_UDC_DWC3_WEDGE_LOG */

void udc_dwc3_stall_snapshot(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t dsts = sys_read32(base + UDC_DWC3_DSTS);

	LOG_ERR("STALL-SNAPSHOT %s: hwirq=%u poll=%u evt=%u norm=%u enobufs=%u | "
		"link=%s dsts=0x%08x DALEPENA=0x%08x",
		dev->name,
		(uint32_t)atomic_get(&udc_dwc3_hwirq_count),
		(uint32_t)atomic_get(&udc_dwc3_poll_count),
		(uint32_t)atomic_get(&udc_dwc3_evt_count),
		(uint32_t)atomic_get(&udc_dwc3_norm_done_count),
		(uint32_t)atomic_get(&udc_dwc3_enobufs_count),
		udc_dwc3_linkstate_str(dsts), dsts,
		sys_read32(base + UDC_DWC3_DALEPENA));

	LOG_ERR("STALL-EVTRING gevntcount=%u evt_next=%u unlanded=%u",
		sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)), priv->evt_next,
		(uint32_t)atomic_get(&udc_dwc3_evt_unlanded_count));

	/* Report the FIFO layout on the first wedge; see udc_dwc3_dump_fifo_cfg(). */
	if (!udc_dwc3_fifo_snapshot_done) {
		udc_dwc3_fifo_snapshot_done = true;
		udc_dwc3_dump_fifo_cfg(dev, "wedge");
	}

	for (int epn = 0; epn < UDC_DWC3_DEPEVT_MAX_EPN; epn++) {
		const uint32_t complete = (uint32_t)atomic_get(&udc_dwc3_depevt_complete[epn]);
		const uint32_t inprog = (uint32_t)atomic_get(&udc_dwc3_depevt_inprog[epn]);

		if (complete != 0U || inprog != 0U) {
			LOG_ERR("STALL-DEPEVT epn=%d complete=%u inprog=%u", epn, complete, inprog);
		}
	}

	LOG_ERR("STALL-EP0 dup out=%ld in=%ld defer_hwo out=%ld in=%ld "
		"poll_ret out=%ld in=%ld setup=%u ctrlin=%u ctrlout=%u",
		(long)atomic_get(&udc_dwc3_ep0_dup_hint_out),
		(long)atomic_get(&udc_dwc3_ep0_dup_hint_in),
		(long)atomic_get(&udc_dwc3_ep0_defer_hwo_out),
		(long)atomic_get(&udc_dwc3_ep0_defer_hwo_in),
		(long)atomic_get(&udc_dwc3_ep0_poll_retired_out),
		(long)atomic_get(&udc_dwc3_ep0_poll_retired_in),
		(uint32_t)atomic_get(&udc_dwc3_ctrl_setup_count),
		(uint32_t)atomic_get(&udc_dwc3_ctrl_in_count),
		(uint32_t)atomic_get(&udc_dwc3_ctrl_out_count));

	for (int epn = 0; epn < 8; epn++) {
		const uint32_t depcmd = sys_read32(base + UDC_DWC3_DEPCMD(epn));
		const uint32_t par0 = sys_read32(base + UDC_DWC3_DEPCMDPAR0(epn));
		const uint32_t par1 = sys_read32(base + UDC_DWC3_DEPCMDPAR1(epn));
		const uint32_t par2 = sys_read32(base + UDC_DWC3_DEPCMDPAR2(epn));

		if (depcmd != 0 || par0 != 0 || par1 != 0 || par2 != 0) {
			LOG_ERR("STALL-DEPCMD epn=%d cmd=0x%08x par0=0x%08x par1=0x%08x par2=0x%08x",
				epn, depcmd, par0, par1, par2);
		}
	}

	for (int i = 0; i < cfg->num_in_eps; i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[i];

		if (ep_data->trb_buf == NULL) {
			continue;
		}
		udc_dwc3_log_ep_trb_ring(ep_data);
	}

	for (int i = 0; i < cfg->num_out_eps; i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[i];

		if (ep_data->trb_buf == NULL) {
			continue;
		}
		udc_dwc3_log_ep_trb_ring(ep_data);
#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
		if (ep_data->cfg.addr == 0x01) {
			udc_dwc3_out_acct_log("stall", ep_data);
		}
#endif
	}

#if defined(CONFIG_UDC_DWC3_XFER_TRACE)
	for (int i = 0; i < cfg->num_in_eps; i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[i];

		if (ep_data->trb_buf != NULL && udc_dwc3_acm_diag_ep(ep_data->cfg.addr)) {
			udc_dwc3_link_check(ep_data, "stall-snapshot");
		}
	}
	for (int i = 0; i < cfg->num_out_eps; i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[i];

		if (ep_data->trb_buf != NULL && udc_dwc3_acm_diag_ep(ep_data->cfg.addr)) {
			udc_dwc3_link_check(ep_data, "stall-snapshot");
		}
	}
#endif

#if defined(CONFIG_UDC_DWC3_DMA_SLOT_DIAG)
	udc_dwc3_dma_slot_dump_stall(dev);
#endif
}

#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
static bool udc_dwc3_orphan_has(struct net_buf *const *orphaned, unsigned int n,
				struct net_buf *const buf)
{
	for (unsigned int i = 0U; i < n; i++) {
		if (orphaned[i] == buf) {
			return true;
		}
	}

	return false;
}

static bool udc_dwc3_orphan_add(struct net_buf **orphaned, unsigned int *n,
				struct net_buf *const buf, const unsigned int max)
{
	if (buf == NULL || buf == UDC_DWC3_ZLP_TRB_MARKER) {
		return true;
	}

	if (udc_dwc3_orphan_has(orphaned, *n, buf)) {
		return true;
	}

	if (*n >= max) {
		return false;
	}

	orphaned[(*n)++] = buf;
	return true;
}

/* Consecutive tier-5 re-queues allowed before the request is failed to the class */
#define UDC_DWC3_TIER5_REQUEUE_MAX 3U

/*
 * Tier-5 IN recovery when EndXfer+StartXfer recycle cannot clear tail HWO.
 * Detach ring and queued buffers before EndXfer (which can clear HWO and
 * trigger SW-retire), dedupe orphans, then return each net_buf to the class
 * exactly once.
 */
static bool udc_dwc3_in_start_tier5_recover(const struct device *const dev,
					    struct udc_dwc3_ep_data *ep_data)
{
	struct net_buf *orphaned[CONFIG_UDC_DWC3_TRB_NUM + 2U];
	unsigned int n_orphan = 0U;
	const uint32_t link = CONFIG_UDC_DWC3_TRB_NUM - 1U;
	struct net_buf *qbuf;
	bool partially_sent = false;
	bool requeue;

#if defined(CONFIG_UDC_DWC3_EP_SM)
	ep_data->sm.tier5_recovering = true;
#endif

	udc_dwc3_lock(dev);
	udc_dwc3_stall_snapshot(dev);

	for (uint32_t i = 0U; i < link; i++) {
		struct net_buf *const buf = ep_data->net_buf[i];

		/*
		 * A TRB whose remaining count still equals the full request was
		 * never touched by the controller: nothing reached the wire, so
		 * the buffer may be sent again.  Anything else was partially
		 * transmitted and must not be repeated.
		 */
		if (buf != NULL && buf != UDC_DWC3_ZLP_TRB_MARKER &&
		    FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK,
			      ep_data->trb_buf[i].status) != buf->len) {
			partially_sent = true;
		}

		ep_data->net_buf[i] = NULL;
		udc_dwc3_trb_clear(&ep_data->trb_buf[i]);

		if (!udc_dwc3_orphan_add(orphaned, &n_orphan, buf, ARRAY_SIZE(orphaned))) {
			LOG_ERR("EP-SM: IN-START-TIER5 ep=0x%02x orphan ring overflow",
				ep_data->cfg.addr);
		}

		if (ep_data->chain_buf == buf) {
			ep_data->chain_buf = NULL;
		}
	}

	if (!udc_dwc3_orphan_add(orphaned, &n_orphan, ep_data->chain_buf,
				 ARRAY_SIZE(orphaned))) {
		LOG_ERR("EP-SM: IN-START-TIER5 ep=0x%02x orphan chain overflow",
			ep_data->cfg.addr);
	}
	if (ep_data->chain_buf != NULL) {
		/* Half of an MPS-aligned data+ZLP pair: the data may be on the wire. */
		partially_sent = true;
	}
	ep_data->chain_buf = NULL;

	while ((qbuf = udc_buf_get(&ep_data->cfg)) != NULL) {
		if (!udc_dwc3_orphan_add(orphaned, &n_orphan, qbuf, ARRAY_SIZE(orphaned))) {
			LOG_ERR("EP-SM: IN-START-TIER5 ep=0x%02x orphan queue overflow",
				ep_data->cfg.addr);
			break;
		}
	}

	for (unsigned int attempt = 0U; attempt < 3U; attempt++) {
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		k_busy_wait(100);
	}

	udc_dwc3_ep_ring_reset(ep_data);
	ep_data->xfer_active = false;
	ep_data->absorb_cdc_zlp = false;
	ep_data->skip_xfer_done_count = 0U;
	udc_ep_set_busy(&ep_data->cfg, false);

#if defined(CONFIG_UDC_DWC3_EP_SM)
	udc_dwc3_ep_sm_set_state(ep_data, UDC_DWC3_EP_SM_IDLE);
	ep_data->sm.in_start_reported = false;
	ep_data->sm.in_start_verify_busy = false;
	ep_data->sm.poll_grace_armed = false;
#endif

	/*
	 * The wedge this recovers from is a dropped doorbell: the TRB is armed and
	 * the controller never fetches it, so the request never reached the host.
	 * Failing it back to the class loses the payload for good because no layer
	 * above retransmits, which is what turns a transient controller hiccup into
	 * a host-visible timeout.  Re-queue instead whenever the ring proves nothing
	 * was transmitted, and keep a retry budget so a genuinely dead endpoint still
	 * reports the error rather than re-arming forever.
	 */
	requeue = !partially_sent && n_orphan > 0U &&
		  ep_data->tier5_requeues < UDC_DWC3_TIER5_REQUEUE_MAX;
	/*
	 * The queue was drained above either way, so a caller holding a buffer it
	 * peeked before this ran must not treat it as still being the queue head.
	 */
	ep_data->requeue_gen++;

	if (requeue) {
		ep_data->tier5_requeues++;
		for (unsigned int i = 0U; i < n_orphan; i++) {
			udc_buf_put(&ep_data->cfg, orphaned[i]);
		}
	}

	udc_dwc3_unlock(dev);

	if (!requeue) {
		for (unsigned int i = 0U; i < n_orphan; i++) {
			(void)udc_submit_ep_event(dev, orphaned[i], -ECONNRESET);
		}
	}

#if defined(CONFIG_UDC_DWC3_EP_SM)
	ep_data->sm.tier5_recovering = false;
#endif

	LOG_ERR("EP-SM: IN-START-TIER5 ep=0x%02x ring nuked, %s (%u buf, retry %u)",
		ep_data->cfg.addr, requeue ? "re-queued" : "dropped", n_orphan,
		ep_data->tier5_requeues);
	k_work_submit(&ep_data->work);
	return true;
}
#endif /* CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE */

unsigned udc_dwc3_sw_retire_done(const struct device *const dev)
{
	return udc_dwc3_retire_sw_done_eps(dev, "app");
}

/*
 * UDC API
 *
 * Interface called by Zehpyr from the upper levels of abstractions.
 */

static int udc_dwc3_ep_enqueue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg,
			       struct net_buf *const buf)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	udc_buf_put(ep_cfg, buf);

	switch (ep_cfg->addr) {
	case USB_CONTROL_EP_IN:
	case USB_CONTROL_EP_OUT:
		udc_dwc3_next_ctrl(dev, ep_data);
		break;
	default:
		/* Process this buffer along with other waiting */
		if (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) {
			LOG_DBG("submitting to EP 0x%02x", ep_cfg->addr);
			k_work_submit(&ep_data->work);
		}
	}

	return 0;
}

static int udc_dwc3_ep_dequeue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);

	udc_ep_cancel_queued(dev, ep_cfg);
	udc_ep_set_busy(ep_cfg, false);

	return 0;
}

static int udc_dwc3_ep_disable(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	sys_clear_bit(base + UDC_DWC3_DALEPENA, ep_data->epn);

#if defined(CONFIG_UDC_DWC3_EP_SM)
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		/*
		 * Saturating decrement.  atomic_dec() returns the PREVIOUS value, so
		 * the counter reaches zero on the 1 -> 0 transition.  A plain
		 * atomic_dec() here also went negative whenever more disables than
		 * enables were seen -- which USBRST guarantees, since it force-clears
		 * the counter (see udc_dwc3_on_reset) while endpoints are still
		 * enabled.  A negative count never satisfies bulk_eps_live > 0 again,
		 * which permanently silences the EP-SM: poll_all() and depevt() both
		 * bail out early, so CPU-managed endpoints (the CDC ACM shell pipes)
		 * lose every completion and wedge for good.
		 */
		atomic_val_t live = atomic_get(&DEV_DATA(dev)->bulk_eps_live);

		while (live > 0 &&
		       !atomic_cas(&DEV_DATA(dev)->bulk_eps_live, live, live - 1)) {
			live = atomic_get(&DEV_DATA(dev)->bulk_eps_live);
		}

		if (live == 1) {
# if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
			k_work_cancel_delayable(&DEV_DATA(dev)->in_poll_work);
# endif
# if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
			LOG_WRN("EP-SM: last bulk ep=0x%02x live=0 poll stop",
				ep_data->cfg.addr);
# endif
		}
	}
#endif

	return 0;
}

static int udc_dwc3_ep_set_halt(const struct device *const dev,
				struct udc_ep_config *const ep_cfg)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	/* TODO: empty the buffers from the queue */

	switch (ep_data->cfg.addr) {
	case USB_CONTROL_EP_IN:
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		/* The datasheet says to only set stall the OUT direction */
		ep_data = &cfg->ep_data_out[0];
		__fallthrough;
	case USB_CONTROL_EP_OUT:
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		udc_dwc3_depcmd_set_stall(dev, ep_data);
		break;
	default:
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		udc_dwc3_depcmd_set_stall(dev, ep_data);
		ep_data->cfg.stat.halted = true;
		if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
			LOG_WRN("UVC-RESTART: ep_set_halt ep=0x%02x", ep_data->cfg.addr);
#if defined(CONFIG_UDC_DWC3_EP_SM)
			if (udc_dwc3_ep_sm_is_cpu(ep_data)) {
				udc_dwc3_ep_sm_set_state(ep_data, UDC_DWC3_EP_SM_HALTED);
			}
#endif
			if (lattice_in_halt_cb != NULL) {
				lattice_in_halt_cb(dev, ep_data->cfg.addr);
			}
		}
	}

	return 0;
}

static int udc_dwc3_in_ensure_xfer_started(const struct device *const dev,
					   struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t depcmd_addr = UDC_DWC3_DEPCMD(ep_data->epn);
	uint32_t reg = sys_read32(base + depcmd_addr);
	bool cmderr = false;
	uint32_t rscidx;

	if ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0U) {
		LOG_WRN("UVC-RESTART: ensure skip ep=0x%02x depcmd=0x%08x (CMDACT)",
			ep_data->cfg.addr, reg);
		return 0;
	}

	/*
	 * DEPCMD completion word encodes the last accepted command type in
	 * bits [3:0] and the active transfer resource in [22:16].  After a
	 * successful DepStartXfer the type is 0x6 with a non-zero rscidx.
	 * After DepClearStall (0x5) there is no running transfer even though
	 * the TRB ring may still be armed (HWO=1) -- the UVC restart wedge.
	 */
	if ((reg & 0xFU) == UDC_DWC3_DEPCMD_DEPSTRTXFER &&
	    FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg) != 0U) {
		ep_data->xferrscidx = FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
		ep_data->xfer_active = true;
		LOG_WRN("UVC-RESTART: ensure skip ep=0x%02x depcmd=0x%08x (StartXfer active)",
			ep_data->cfg.addr, reg);
		return 0;
	}

	if ((reg & 0xFU) != UDC_DWC3_DEPCMD_DEPCSTALL) {
		LOG_WRN("UVC-RESTART: ensure skip ep=0x%02x depcmd=0x%08x (not ClearStall)",
			ep_data->cfg.addr, reg);
		return 0;
	}

	LOG_WRN("UVC-RESTART: ensure StartXfer ep=0x%02x depcmd_in=0x%08x",
		ep_data->cfg.addr, reg);

	sys_write32(HI32((uintptr_t)&ep_data->trb_buf[ep_data->head]),
		    base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)&ep_data->trb_buf[ep_data->head]),
		    base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	rscidx = udc_dwc3_depcmd_status(dev, depcmd_addr, UDC_DWC3_DEPCMD_DEPSTRTXFER,
					&cmderr);
	if (cmderr) {
		reg = sys_read32(base + depcmd_addr);
		if ((reg & 0xFU) == UDC_DWC3_DEPCMD_DEPSTRTXFER &&
		    FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg) != 0U) {
			ep_data->xferrscidx =
				FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
			ep_data->xfer_active = true;
			atomic_inc(&udc_dwc3_clrhalt_rearm);
			LOG_WRN("UVC-RESTART: ensure recovered ep=0x%02x depcmd=0x%08x",
				ep_data->cfg.addr, reg);
			return 0;
		}

		LOG_WRN("UVC-RESTART: ensure StartXfer CMDERR ep=0x%02x depcmd=0x%08x",
			ep_data->cfg.addr, reg);
		return -EALREADY;
	}

	ep_data->xferrscidx = rscidx;
	ep_data->xfer_active = true;
	atomic_inc(&udc_dwc3_clrhalt_rearm);
#if defined(CONFIG_UDC_DWC3_EP_SM)
	if (udc_dwc3_ep_sm_is_cpu(ep_data)) {
		udc_dwc3_ep_sm_set_state(ep_data, UDC_DWC3_EP_SM_ACTIVE);
	}
#endif
	LOG_WRN("UVC-RESTART: ensure StartXfer ok ep=0x%02x rscidx=%u depcmd=0x%08x",
		ep_data->cfg.addr, rscidx, sys_read32(base + depcmd_addr));

	return 0;
}

/*
 * Force a fresh DepStartXfer at UVC stream handoff.
 *
 * The RTL uvcmanager only rings DepUpdateXfer doorbells.  On stream restart the
 * controller may accept UpdateXfer (DEPCMD shows 0x90007) yet never fetch the
 * armed TRBs -- the manual devmem kick that un-wedged this was always a CPU
 * DepStartXfer, even when a transfer resource was already allocated.  Unlike
 * clear-halt recovery we always issue StartXfer here; CMDERR is benign when the
 * transfer is already running (ACM on a concurrent path).
 */
static int udc_dwc3_in_restart_xfer(const struct device *const dev,
				    struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t depcmd_addr = UDC_DWC3_DEPCMD(ep_data->epn);
	uint32_t reg = sys_read32(base + depcmd_addr);
	bool cmderr = false;
	uint32_t rscidx;
	uint32_t active_rscidx = ep_data->xferrscidx;
	bool endxfer = false;

	LOG_WRN("UVC-RESTART: in_restart ep=0x%02x depcmd_in=0x%08x rscidx_sw=%u",
		ep_data->cfg.addr, reg, active_rscidx);

	if ((reg & UDC_DWC3_DEPCMD_CMDACT) == 0U) {
		const uint32_t depcmd_type = reg & 0xFU;
		const uint32_t depcmd_rscidx =
			FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);

		if ((depcmd_type == UDC_DWC3_DEPCMD_DEPSTRTXFER ||
		     depcmd_type == UDC_DWC3_DEPCMD_DEPUPDXFER) &&
		    depcmd_rscidx != 0U) {
			active_rscidx = depcmd_rscidx;
		}
	}

	/*
	 * Stream handoff must replace any prior transfer resource, including
	 * one left by ep_enable trb_norm_init or a stale RTL UpdateXfer.
	 */
	if (active_rscidx != 0U) {
		endxfer = true;
		(void)udc_dwc3_depcmd_issue(dev, depcmd_addr,
			UDC_DWC3_DEPCMD_DEPENDXFER | UDC_DWC3_DEPCMD_HIPRI_FORCERM |
			FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, active_rscidx),
			&cmderr);
		if (cmderr) {
			LOG_WRN("UVC-RESTART: in_restart EndXfer CMDERR ep=0x%02x rscidx=%u",
				ep_data->cfg.addr, active_rscidx);
		}
	}

	sys_write32(HI32((uintptr_t)&ep_data->trb_buf[ep_data->head]),
		    base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)&ep_data->trb_buf[ep_data->head]),
		    base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	rscidx = udc_dwc3_depcmd_issue(dev, depcmd_addr, UDC_DWC3_DEPCMD_DEPSTRTXFER,
				       &cmderr);
	if (cmderr) {
		LOG_WRN("UVC-RESTART: in_restart StartXfer CMDERR ep=0x%02x endxfer=%d",
			ep_data->cfg.addr, endxfer);
		return -EALREADY;
	}

	ep_data->xferrscidx = rscidx;
	ep_data->xfer_active = true;
	atomic_inc(&udc_dwc3_in_stream_rearm);
	LOG_WRN("UVC-RESTART: in_restart ok ep=0x%02x endxfer=%d rscidx=%u depcmd=0x%08x",
		ep_data->cfg.addr, endxfer, rscidx, sys_read32(base + depcmd_addr));

	return 0;
}

static int udc_dwc3_ep_clear_halt(const struct device *const dev,
				  struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t depcmd_addr = UDC_DWC3_DEPCMD(ep_data->epn);
	uint32_t depcmd_before = sys_read32(base + depcmd_addr);
	int ensure_ret;

	__ASSERT_NO_MSG(ep_data->cfg.addr != USB_CONTROL_EP_OUT);
	__ASSERT_NO_MSG(ep_data->cfg.addr != USB_CONTROL_EP_IN);

	LOG_WRN("UVC-RESTART: clear_halt ep=0x%02x depcmd_before=0x%08x",
		ep_data->cfg.addr, depcmd_before);

	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr) && lattice_in_clear_halt_cb != NULL) {
		lattice_in_clear_halt_cb(dev, ep_data->cfg.addr);
	}

	udc_dwc3_depcmd_clear_stall(dev, ep_data);
	ep_data->cfg.stat.halted = false;
#if defined(CONFIG_UDC_DWC3_EP_SM)
	if (udc_dwc3_ep_sm_is_cpu(ep_data)) {
		udc_dwc3_ep_sm_set_state(ep_data, UDC_DWC3_EP_SM_CLEAR_PENDING);
	}
#endif

	/*
	 * After CLEAR_FEATURE(ENDPOINT_HALT) the DWC3 transfer resource is
	 * freed (DEPCMD completion shows DepClearStall) even though RTL-
	 * offloaded UVC TRBs may still be armed (HWO=1).  Re-issue StartXfer
	 * only when DEPCMD says the last completed command was DepClearStall;
	 * skip when a StartXfer resource is already active (ACM IN during
	 * concurrent restart).  uvcmanager_set_stream() calls the same helper
	 * as a safety net when RTL starts after negotiation.
	 */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		ensure_ret = udc_dwc3_in_ensure_xfer_started(dev, ep_data);
		LOG_WRN("UVC-RESTART: clear_halt ensure ep=0x%02x ret=%d depcmd_after=0x%08x",
			ep_data->cfg.addr, ensure_ret, sys_read32(base + depcmd_addr));
	}

	return 0;
}

static int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	LOG_INF("Setting address to %u", addr);

	/* Configure the new address */
	reg = sys_read32(base + UDC_DWC3_DCFG);
	reg &= ~UDC_DWC3_DCFG_DEVADDR_MASK;
	reg |= FIELD_PREP(UDC_DWC3_DCFG_DEVADDR_MASK, addr);
	sys_write32(reg, base + UDC_DWC3_DCFG);

	/* Re-apply the same endpoint configuration */
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0]);
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_out[0]);

	return 0;
}

static enum udc_bus_speed udc_dwc3_device_speed(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	switch (sys_read32(base + UDC_DWC3_DSTS) & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
		return UDC_BUS_SPEED_FS;
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
		return UDC_BUS_SPEED_HS;
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		return UDC_BUS_SPEED_SS;
	}

	LOG_ERR("Unknown device speed");

	return 0;
}

static int udc_dwc3_enable(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	int ret;

	LOG_DBG("Enabling DWC3 driver");

	ret = udc_dwc3_quirk_enable(dev);
	if (ret != 0) {
		return ret;
	}

	/* Enable the DWC3 events */
	sys_set_bits(base + UDC_DWC3_DCTL, UDC_DWC3_DCTL_RUNSTOP);

	/* Enable the IRQ (for now, just schedule a first work queue job) */
	cfg->irq_enable_func();

#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
	k_work_reschedule(&DEV_DATA(dev)->health_work,
			  K_MSEC(CONFIG_UDC_DWC3_HEALTH_LOG_INTERVAL_MS));
#endif
#if defined(CONFIG_UDC_DWC3_EP_SM)
	atomic_set(&DEV_DATA(dev)->bulk_eps_live, 0);
# if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
	LOG_INF("EP-SM: udc_enable bulk_eps_live=0 poll deferred");
# endif
#endif
#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL) && !defined(CONFIG_UDC_DWC3_EP_SM)
	k_work_reschedule(&DEV_DATA(dev)->in_poll_work,
			  K_USEC(CONFIG_UDC_DWC3_IN_COMPLETION_POLL_INTERVAL_US));
#endif

	return 0;
}

static int udc_dwc3_disable(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	LOG_DBG("Disabling DWC3 driver");

#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
	k_work_cancel_delayable(&DEV_DATA(dev)->health_work);
#endif
#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
	k_work_cancel_delayable(&DEV_DATA(dev)->in_poll_work);
#endif

	sys_clear_bits(base + UDC_DWC3_DCTL, UDC_DWC3_DCTL_RUNSTOP);

	return 0;
}

/*
 * Hardware Init
 *
 * Prepare the driver and the hardware to being used.
 * This goes through register configuration and register commands.
 */

static int udc_dwc3_ep_enable(const struct device *const dev,
			      struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = (struct udc_dwc3_ep_data *)ep_cfg;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	LOG_DBG("%s 0x%02x", __func__, ep_data->cfg.addr);

	ep_data->chain_buf = NULL;
	ep_data->absorb_cdc_zlp = false;
	ep_data->xfer_active = false;
	ep_data->skip_xfer_done_count = 0U;
	ep_data->head = ep_data->tail = 0U;
	ep_data->total = 0U;
	ep_data->full = false;

	memset(ep_data->trb_buf, 0, sizeof(*ep_data->trb_buf) * CONFIG_UDC_DWC3_TRB_NUM);

	/*
	 * Allocate the non-control transfer resource pool before configuring the
	 * first endpoint that needs it.  Once per configuration: DEPSTARTCFG
	 * reassigns the whole pool, so repeating it would pull resources out from
	 * under endpoints already running.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 &&
	    !DEV_DATA(dev)->startcfg_nonctrl_done) {
		const struct udc_dwc3_config *const cfg = dev->config;

		DEV_DATA(dev)->startcfg_nonctrl_done = true;
		udc_dwc3_depcmd_start_config(dev, &cfg->ep_data_out[0], 2U);
	}

	udc_dwc3_depcmd_ep_config(dev, ep_data);
	udc_dwc3_depcmd_ep_xfer_config(dev, ep_data);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		udc_dwc3_trb_norm_init(dev, ep_data);
	}

	/* Starting from here, the endpoint can be used */
	sys_set_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	/* Walk through the list of buffer to enqueue we might have blocked */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
#if defined(CONFIG_UDC_DWC3_EP_SM)
		{
			/*
			 * atomic_inc() returns the PREVIOUS value, so the first
			 * bulk endpoint is the one that sees 0.  Testing for 1
			 * armed the poll only on the second endpoint, and after
			 * a USBRST force-clear it could miss the restart.
			 */
			const atomic_val_t live = atomic_inc(&DEV_DATA(dev)->bulk_eps_live);

			if (live == 0) {
# if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
				k_work_reschedule(&DEV_DATA(dev)->in_poll_work,
					K_USEC(CONFIG_UDC_DWC3_IN_COMPLETION_POLL_INTERVAL_US));
# endif
# if defined(CONFIG_UDC_DWC3_EP_SM_LOG_PHASE)
				LOG_WRN("EP-SM: first bulk ep=0x%02x live=%ld poll start",
					ep_data->cfg.addr, (long)live + 1);
# endif
			}
		}
#endif
		k_work_submit(&ep_data->work);
	}

	return 0;
}

/*
 * Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only udc_dwc3_enable() makes device visible to the host.
 */
static int udc_dwc3_init(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	int ret;

	LOG_DBG("Initializing the DWC3 core");

	ret = udc_dwc3_quirk_init(dev);
	if (ret != 0) {
		return ret;
	}

	/* Issue a soft reset to the core and USB2 and USB3 PHY */
	sys_set_bits(base + UDC_DWC3_GCTL, UDC_DWC3_GCTL_CORESOFTRESET);
	sys_set_bits(base + UDC_DWC3_GUSB3PIPECTL, UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST);
	sys_set_bits(base + UDC_DWC3_GUSB2PHYCFG, UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST);
	k_sleep(K_USEC(100));

	/* Teriminate the reset of the USB2 and USB3 PHY first */
	sys_clear_bits(base + UDC_DWC3_GUSB3PIPECTL, UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST);
	sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG, UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST);

	/* Teriminate the reset of the DWC3 core after it */
	sys_clear_bits(base + UDC_DWC3_GCTL, UDC_DWC3_GCTL_CORESOFTRESET);

	/* The USB core was reset, configure it as documented */
	udc_dwc3_on_soft_reset(dev);

	/* Configure the control OUT endpoint */
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 512, 0);
	if (ret != 0) {
		LOG_ERR("could not enable control OUT ep");
		return ret;
	}

	/* Configure the control IN endpoint */
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 512, 0);
	if (ret != 0) {
		LOG_ERR("could not enable control IN ep");
		return ret;
	}

	LOG_INF("Event buffer is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));

	return 0;
}

static const struct udc_api udc_dwc3_api = {
	.lock = udc_dwc3_lock,
	.unlock = udc_dwc3_unlock,
	.device_speed = udc_dwc3_device_speed,
	.init = udc_dwc3_init,
	.enable = udc_dwc3_enable,
	.disable = udc_dwc3_disable,
	.shutdown = udc_dwc3_shutdown,
	.set_address = udc_dwc3_set_address,
	.ep_enable = udc_dwc3_ep_enable,
	.ep_disable = udc_dwc3_ep_disable,
	.ep_set_halt = udc_dwc3_ep_set_halt,
	.ep_clear_halt = udc_dwc3_ep_clear_halt,
	.ep_enqueue = udc_dwc3_ep_enqueue,
	.ep_dequeue = udc_dwc3_ep_dequeue,
};

static void udc_dwc3_ep_worker(struct k_work *const work)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(work, struct udc_dwc3_ep_data, work);
	const struct device *const dev = ep_data->dev;
	struct net_buf *buf;
	int ret;

	LOG_DBG("checking for pending transfers for EP 0x%02x", ep_data->cfg.addr);

	if (ep_data->cfg.stat.halted) {
		LOG_DBG("endpoint is halted, not processing buffers");
		return;
	}

#if defined(CONFIG_UDC_DWC3_EP_SM)
	udc_dwc3_ep_advance(dev, ep_data, UDC_DWC3_EP_ADV_WORKER);
#else
	while (udc_dwc3_retire_sw_done(dev, ep_data, "worker")) {
		;
	}
#endif

	while ((buf = udc_buf_peek(&ep_data->cfg)) != NULL) {
#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
		const uint8_t requeue_gen = ep_data->requeue_gen;
#endif

		LOG_INF("Processing buffer %p from queue", (void *)buf);

		ret = udc_dwc3_trb_bulk(dev, ep_data, buf);
		if (ret != 0) {
			LOG_DBG("abort: No more room for buffer");
			break;
		}

#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
		/*
		 * A dropped StartXfer doorbell is recovered from inside the arm
		 * above, and that recovery empties the ring and puts the buffers
		 * back on this queue -- including the one being armed here.  It
		 * still reports success, so consuming the queue head now would
		 * throw away the copy recovery just re-queued, leaving the
		 * transfer neither armed nor queued and the class waiting for a
		 * completion that can no longer come.  Recovery submits this
		 * worker again, so leave the queue to that pass.
		 */
		if (ep_data->requeue_gen != requeue_gen) {
			LOG_DBG("re-queued during arm, leaving buffer for the retry");
			break;
		}
#endif

		LOG_DBG("success: Buffer enqueued");

		udc_buf_get(&ep_data->cfg);
	}
}

/*
 * Initialize the controller and endpoints capabilities,
 * register endpoint structures, no hardware I/O yet.
 */
static int udc_dwc3_driver_preinit(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_data *const data = dev->data;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *ep_data;
	uint16_t mps = 0;
	int ret;

	ret = udc_dwc3_quirk_preinit(dev);
	if (ret != 0) {
		return ret;
	}

	DEVICE_MMIO_NAMED_MAP(dev, base, K_MEM_CACHE_NONE);

	k_mutex_init(&data->mutex);

	priv->dev = dev;
	k_work_init(&priv->event_work, udc_dwc3_event_worker);
#if defined(CONFIG_UDC_DWC3_HEALTH_LOG)
	k_work_init_delayable(&priv->health_work, udc_dwc3_health_worker);
#endif
#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
	k_work_init_delayable(&priv->in_poll_work, udc_dwc3_in_poll_worker);
#endif
#if defined(CONFIG_UDC_DWC3_EP_SM)
	atomic_clear(&priv->bulk_eps_live);
#endif

	data->caps.rwup = false;
	data->caps.addr_before_status = true;

	switch (cfg->maximum_speed_idx) {
	case UDC_DWC3_SPEED_IDX_SUPER_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_SUPER_SPEED");
		data->caps.mps0 = UDC_MPS0_512;
		data->caps.hs = true;
		data->caps.ss = true;
		mps = 1024;
		break;
	case UDC_DWC3_SPEED_IDX_HIGH_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_HIGH_SPEED");
		data->caps.mps0 = UDC_MPS0_64;
		data->caps.hs = true;
		mps = 1024;
		break;
	case UDC_DWC3_SPEED_IDX_FULL_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_FULL_SPEED");
		data->caps.mps0 = UDC_MPS0_64;
		mps = 64;
		break;
	default:
		LOG_ERR("Speed %d not supported", cfg->maximum_speed_idx);
		return -ENOTSUP;
	}

	/* Control IN endpoint */

	ep_data = &cfg->ep_data_in[0];
	ep_data->dev = dev;
	ep_data->cfg.addr = USB_CONTROL_EP_IN;
	ep_data->cfg.caps.in = 1;
	ep_data->cfg.caps.control = 1;
	ep_data->cfg.caps.mps = mps;
	ep_data->trb_buf = cfg->trb_buf_in[0];
	ep_data->epn = 1;

	ret = udc_register_ep(dev, &ep_data->cfg);
	if (ret != 0) {
		LOG_ERR("Failed to register endpoint");
		return ret;
	}

	/* Control OUT endpoint */

	ep_data = &cfg->ep_data_out[0];
	ep_data->dev = dev;
	ep_data->cfg.addr = USB_CONTROL_EP_OUT;
	ep_data->cfg.caps.out = 1;
	ep_data->cfg.caps.control = 1;
	ep_data->cfg.caps.mps = mps;
	ep_data->trb_buf = cfg->trb_buf_out[0];
	ep_data->epn = 0;

	ret = udc_register_ep(dev, &ep_data->cfg);
	if (ret != 0) {
		LOG_ERR("Failed to register endpoint");
		return ret;
	}

	/* Normal IN endpoints */
	for (int i = 1; i < cfg->num_in_eps; i++) {
		LOG_DBG("Preinit endpoint 0x%02x", USB_EP_DIR_IN | i);

		ep_data = &cfg->ep_data_in[i];
		k_work_init(&ep_data->work, udc_dwc3_ep_worker);
#if defined(CONFIG_UDC_DWC3_EP_SM)
		udc_dwc3_ep_sm_init(ep_data);
#endif

		ep_data->dev = dev;
		ep_data->cfg.addr = USB_EP_DIR_IN | i;
		ep_data->cfg.caps.in = true;
		ep_data->cfg.caps.bulk = true;
		ep_data->cfg.caps.interrupt = true;
		ep_data->cfg.caps.iso = true;
		ep_data->cfg.caps.mps = mps;
		ep_data->trb_buf = cfg->trb_buf_in[i];
		ep_data->epn = (i << 1) | 1;

		ret = udc_register_ep(dev, &ep_data->cfg);
		if (ret != 0) {
			LOG_ERR("Failed to register endpoint");
			return ret;
		}
	}

	/* Normal OUT endpoints */
	for (int i = 1; i < cfg->num_out_eps; i++) {
		LOG_DBG("Preinit endpoint 0x%02x", USB_EP_DIR_OUT | i);

		ep_data = &cfg->ep_data_out[i];
		k_work_init(&ep_data->work, udc_dwc3_ep_worker);
#if defined(CONFIG_UDC_DWC3_EP_SM)
		udc_dwc3_ep_sm_init(ep_data);
#endif

		ep_data->dev = dev;
		ep_data->cfg.addr = USB_EP_DIR_OUT | i;
		ep_data->cfg.caps.out = true;
		ep_data->cfg.caps.bulk = true;
		ep_data->cfg.caps.interrupt = true;
		ep_data->cfg.caps.iso = true;
		ep_data->cfg.caps.mps = mps;
		ep_data->trb_buf = cfg->trb_buf_out[i];
		ep_data->epn = (i << 1) | 0;

		ret = udc_register_ep(dev, &ep_data->cfg);
		if (ret != 0) {
			LOG_ERR("Failed to register endpoint");
			return ret;
		}
	}

	return 0;
}

#if defined(CONFIG_UDC_DWC3_EP_SM)
bool udc_dwc3_int_trb_hwo(const volatile struct udc_dwc3_trb *trb)
{
	return udc_dwc3_trb_hwo(trb);
}

uint32_t udc_dwc3_int_ring_data_hwo_mask(const struct udc_dwc3_ep_data *ep_data)
{
	return udc_dwc3_ring_data_hwo_mask(ep_data);
}

uint32_t udc_dwc3_int_trb_remaining(const struct udc_dwc3_trb *const trb)
{
	return FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb->status);
}

uint32_t udc_dwc3_int_depcmd_issue(const struct device *dev, uint32_t depcmd_addr,
				   uint32_t cmd, bool *cmderr)
{
	return udc_dwc3_depcmd_issue(dev, depcmd_addr, cmd, cmderr);
}

void udc_dwc3_int_depcmd_start_xfer(const struct device *dev,
				    struct udc_dwc3_ep_data *ep_data)
{
	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

void udc_dwc3_int_depcmd_start_xfer_trb(const struct device *dev,
					struct udc_dwc3_ep_data *ep_data,
					struct udc_dwc3_trb *trb)
{
	udc_dwc3_depcmd_start_xfer_trb(dev, ep_data, trb);
}

void udc_dwc3_int_depcmd_update_xfer(const struct device *dev,
				     struct udc_dwc3_ep_data *ep_data)
{
	udc_dwc3_depcmd_update_xfer(dev, ep_data);
}

uint32_t udc_dwc3_int_depcmd_update_xfer_checked(const struct device *dev,
						 struct udc_dwc3_ep_data *ep_data,
						 bool *cmderr)
{
	uint32_t flags = UDC_DWC3_DEPCMD_DEPUPDXFER;

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

	return udc_dwc3_depcmd_status(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags, cmderr);
}

atomic_val_t udc_dwc3_int_in_start_recycled_get(void)
{
#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
	return atomic_get(&udc_dwc3_in_start_recycled);
#else
	return 0;
#endif
}

atomic_val_t udc_dwc3_int_in_start_exhausted_get(void)
{
	return atomic_get(&udc_dwc3_in_start_exhausted);
}

void udc_dwc3_int_on_xfer_done_norm(const struct device *dev, uint32_t evt)
{
	udc_dwc3_on_xfer_done_norm(dev, evt);
}

bool udc_dwc3_int_retire_sw_done(const struct device *dev,
				 struct udc_dwc3_ep_data *ep_data,
				 const char *via)
{
	return udc_dwc3_retire_sw_done(dev, ep_data, via);
}

void udc_dwc3_int_in_endxfer_recycle(const struct device *dev,
				     struct udc_dwc3_ep_data *ep_data)
{
#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
	udc_dwc3_in_start_endxfer_recycle(dev, ep_data);
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(ep_data);
#endif
}

bool udc_dwc3_int_out_endxfer_recycle(const struct device *dev,
				      struct udc_dwc3_ep_data *ep_data)
{
#if defined(CONFIG_UDC_DWC3_OUT_RUNDRY_ENDXFER_ESCALATE)
	return udc_dwc3_out_rundry_endxfer_recycle(dev, ep_data);
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(ep_data);
	return false;
#endif
}

void udc_dwc3_int_submit_ep_work(struct udc_dwc3_ep_data *ep_data)
{
	k_work_submit(&ep_data->work);
}

struct udc_dwc3_ep_data *udc_dwc3_int_ep_from_evt(const struct device *dev, uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);

	return (epn & 1) ? &cfg->ep_data_in[epn >> 1] : &cfg->ep_data_out[epn >> 1];
}

int udc_dwc3_int_num_in_eps(const struct device *dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	return cfg->num_in_eps;
}

int udc_dwc3_int_num_out_eps(const struct device *dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	return cfg->num_out_eps;
}

struct udc_dwc3_ep_data *udc_dwc3_int_ep_in(const struct device *dev, int idx)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	return &cfg->ep_data_in[idx];
}

struct udc_dwc3_ep_data *udc_dwc3_int_ep_out(const struct device *dev, int idx)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	return &cfg->ep_data_out[idx];
}

bool udc_dwc3_int_bulk_eps_live(const struct device *dev)
{
	return atomic_get(&DEV_DATA(dev)->bulk_eps_live) > 0;
}
#endif /* CONFIG_UDC_DWC3_EP_SM */

void lattice_usb23_set_in_halt_cb(lattice_usb23_in_halt_fn fn)
{
	lattice_in_halt_cb = fn;
}

void lattice_usb23_set_in_clear_halt_cb(lattice_usb23_in_halt_fn fn)
{
	lattice_in_clear_halt_cb = fn;
}

uint32_t lattice_usb23_read_depcmd(const struct device *dev, uint8_t ep_addr)
{
	struct udc_dwc3_ep_data *ep_data = (void *)udc_get_ep_cfg(dev, ep_addr);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	if (ep_data == NULL) {
		return 0U;
	}

	return sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn));
}

int lattice_usb23_in_ensure_xfer_started(const struct device *dev, uint8_t ep_addr)
{
	struct udc_ep_config *const ep_cfg = udc_get_ep_cfg(dev, ep_addr);

	if (ep_cfg == NULL) {
		return -ENODEV;
	}

	if (!USB_EP_DIR_IS_IN(ep_addr)) {
		return -EINVAL;
	}

	return udc_dwc3_in_ensure_xfer_started(dev,
		CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg));
}

int lattice_usb23_in_restart_xfer(const struct device *dev, uint8_t ep_addr)
{
	struct udc_ep_config *const ep_cfg = udc_get_ep_cfg(dev, ep_addr);

	if (ep_cfg == NULL) {
		return -ENODEV;
	}

	if (!USB_EP_DIR_IS_IN(ep_addr)) {
		return -EINVAL;
	}

	return udc_dwc3_in_restart_xfer(dev,
		CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg));
}

#define UDC_DWC3_DEVICE_DEFINE(n)						\
	UDC_DWC3_QUIRK_DEFINE(n);						\
										\
	static void udc_dwc3_irq_enable_func_##n(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    udc_dwc3_irq_handler, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void udc_dwc3_irq_disable_func_##n(void)				\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
										\
	static __nocache uint32_t udc_dwc3_dma_evt_buf_##n			\
		[CONFIG_UDC_DWC3_EVENTS_NUM]					\
		__aligned(16);							\
										\
	static __nocache struct udc_dwc3_trb udc_dwc3_dma_trb_i##n		\
		[DT_INST_PROP(n, num_in_endpoints)][CONFIG_UDC_DWC3_TRB_NUM]	\
		__aligned(16);							\
										\
	static __nocache struct udc_dwc3_trb udc_dwc3_dma_trb_o##n		\
		[DT_INST_PROP(n, num_out_endpoints)][CONFIG_UDC_DWC3_TRB_NUM]	\
		__aligned(16);							\
										\
	static struct udc_dwc3_ep_data udc_dwc3_ep_data_i##n			\
		[DT_INST_PROP(n, num_in_endpoints)];				\
										\
	static struct udc_dwc3_ep_data udc_dwc3_ep_data_o##n			\
		[DT_INST_PROP(n, num_out_endpoints)];				\
										\
	static const struct udc_dwc3_config udc_dwc3_config_##n = {		\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(base, DT_DRV_INST(n)),	\
		.quirk_data = &udc_dwc3_quirk_data_##n,				\
		.quirk_config = &udc_dwc3_quirk_config_##n,			\
		.num_in_eps = DT_INST_PROP(n, num_in_endpoints),		\
		.num_out_eps = DT_INST_PROP(n, num_out_endpoints),		\
		.ep_data_in  = udc_dwc3_ep_data_i##n,				\
		.ep_data_out = udc_dwc3_ep_data_o##n,				\
		.trb_buf_in = udc_dwc3_dma_trb_i##n,				\
		.trb_buf_out = udc_dwc3_dma_trb_o##n,				\
		.evt_buf = udc_dwc3_dma_evt_buf_##n,				\
		.maximum_speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),\
		.irq_enable_func = udc_dwc3_irq_enable_func_##n,		\
		.irq_disable_func = udc_dwc3_irq_disable_func_##n,		\
	};									\
										\
	static struct udc_dwc3_data udc_dwc3_priv_##n = {			\
		.dev = DEVICE_DT_INST_GET(n),					\
	};									\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_dwc3_priv_##n,					\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, udc_dwc3_driver_preinit, NULL, &udc_data_##n,	\
			      &udc_dwc3_config_##n, POST_KERNEL,		\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &udc_dwc3_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_DWC3_DEVICE_DEFINE)
