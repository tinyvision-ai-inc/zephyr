/*
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Track B UsbEngine backend: post DESC_*, drain FWD_POP / CMPL_*.
 * ENABLE=0 (or missing MAGIC) leaves Track A mailbox + GEVNTCOUNT path.
 */

#include <zephyr/drivers/usb/udc.h>
#include <zephyr/drivers/usb/udc/udc_dwc3_usb_engine.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_ch9.h>

#include "udc_common.h"

#include <errno.h>
#include <string.h>

#define USB_ENGINE_POST_DEPTH 2

static bool engine_seen;
static bool engine_on;
static bool engine_evt_own;
static uint8_t programmed;
static uint32_t saved_evt_base;
static uint32_t saved_evt_size;
static void engine_go_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(engine_go_work, engine_go_fn);
static struct net_buf *posted[USB_ENGINE_BULK_N][USB_ENGINE_POST_DEPTH];
static uint8_t post_hd[USB_ENGINE_BULK_N];
static uint8_t post_tl[USB_ENGINE_BULK_N];
static uint8_t post_n[USB_ENGINE_BULK_N];

static const uint8_t bulk_addr[USB_ENGINE_BULK_N] = {
	0x81, 0x82, 0x01,
};

static mm_reg_t eng_base(void)
{
	return USB_ENGINE_BASE;
}

bool udc_dwc3_engine_present(void)
{
	if (engine_seen) {
		return true;
	}
	if ((sys_read32(eng_base() + USB_ENGINE_ENG_ID) & 0xffffU) == USB_ENGINE_MAGIC) {
		engine_seen = true;
		return true;
	}
	return false;
}

bool udc_dwc3_engine_enabled(void)
{
	return engine_on;
}

bool udc_dwc3_engine_evt_own(void)
{
	return engine_on && engine_evt_own;
}

bool udc_dwc3_engine_owns(uint8_t addr)
{
	return engine_on && usb_engine_bulk_idx(addr) >= 0;
}

bool udc_dwc3_engine_posted(uint8_t addr)
{
	const int idx = usb_engine_bulk_idx(addr);

	return engine_on && idx >= 0 && post_n[idx] > 0U;
}

void udc_dwc3_engine_dump(void)
{
	const mm_reg_t base = eng_base();

	if (!engine_on) {
		return;
	}
	printk("engine: st=0x%x evt=%u cmd=%u own=0x%x post=%u/%u/%u\n",
	       sys_read32(base + USB_ENGINE_ENG_STATUS),
	       sys_read32(base + USB_ENGINE_EVT_COUNT),
	       sys_read32(base + USB_ENGINE_CMD_COUNT),
	       sys_read32(base + USB_ENGINE_EP_OWN),
	       post_n[0], post_n[1], post_n[2]);
	for (int i = 0; i < USB_ENGINE_BULK_N; i++) {
		const mm_reg_t epb = usb_engine_ep_base(i);

		printk("engine: ep%u 0x%02x free=%u cmpl=%u st=0x%x\n",
		       i, bulk_addr[i],
		       sys_read32(epb + USB_ENGINE_EP_DESC_FREE),
		       sys_read32(epb + USB_ENGINE_EP_CMPL_LEVEL),
		       sys_read32(epb + USB_ENGINE_EP_STATE));
	}
}

bool udc_dwc3_engine_vid_live(void)
{
	if (!engine_on) {
		return false;
	}
	return (sys_read32(eng_base() + USB_ENGINE_ENG_STATUS) &
		USB_ENGINE_STAT_VID_LIVE) != 0U;
}

uint32_t udc_dwc3_engine_status(void)
{
	if (!udc_dwc3_engine_present()) {
		return 0;
	}
	return sys_read32(eng_base() + USB_ENGINE_ENG_STATUS);
}

static void posted_clear(int idx)
{
	for (int i = 0; i < USB_ENGINE_POST_DEPTH; i++) {
		posted[idx][i] = NULL;
	}
	post_hd[idx] = 0;
	post_tl[idx] = 0;
	post_n[idx] = 0;
}

static int posted_push(int idx, struct net_buf *buf)
{
	if (post_n[idx] >= USB_ENGINE_POST_DEPTH) {
		return -EBUSY;
	}
	posted[idx][post_hd[idx]] = buf;
	post_hd[idx] = (uint8_t)((post_hd[idx] + 1U) % USB_ENGINE_POST_DEPTH);
	post_n[idx]++;
	return 0;
}

static struct net_buf *posted_take(int idx, uint32_t addr)
{
	struct net_buf *buf;

	if (post_n[idx] == 0U) {
		return NULL;
	}
	buf = posted[idx][post_tl[idx]];
	if (buf != NULL && addr != 0U && (uintptr_t)buf->data != addr) {
		/* In-order BulkRing; still accept a pointer mismatch. */
	}
	posted[idx][post_tl[idx]] = NULL;
	post_tl[idx] = (uint8_t)((post_tl[idx] + 1U) % USB_ENGINE_POST_DEPTH);
	post_n[idx]--;
	return buf;
}

static void engine_try_enable(uint32_t evt_base, uint32_t evt_size)
{
	const mm_reg_t base = eng_base();
	uint32_t id;
	uint32_t ctrl;

	if (engine_on || programmed != 0x07U) {
		return;
	}
	if (evt_base == 0U) {
		evt_base = saved_evt_base;
		evt_size = saved_evt_size;
	}
	id = sys_read32(base + USB_ENGINE_ENG_ID);
	if ((id & 0xffffU) != USB_ENGINE_MAGIC) {
		printk("engine: no MAGIC at 0x%x (id=0x%08x), stay Track A\n",
		       (uint32_t)base, id);
		return;
	}
	sys_write32(USB_ENGINE_EP_OWN_DEFAULT, base + USB_ENGINE_EP_OWN);
	sys_write32(evt_base, base + USB_ENGINE_EVT_BASE);
	sys_write32(evt_size, base + USB_ENGINE_EVT_SIZE);
	/* Fair-share: yield to mailbox after this many video grants. */
	sys_write32(32U, base + USB_ENGINE_VID_CREDIT);
	sys_write32(0U, base + USB_ENGINE_QUIET_OTHER);
	sys_write32(USB_ENGINE_IRQ_FWD | USB_ENGINE_IRQ_CMPL | USB_ENGINE_IRQ_ERR,
		    base + USB_ENGINE_ENG_IRQ_ENA);
	/*
	 * ENABLE at STREAMON. EVT_OWN stays 0 so CPU keeps GEVNTCOUNT.
	 * EventDrain still snoops the ring (RTL) and completes BulkRing.
	 */
	ctrl = USB_ENGINE_CTRL_ENABLE | USB_ENGINE_CTRL_CREDIT_EN |
	       USB_ENGINE_CTRL_IRQ_EN;
	sys_write32(ctrl, base + USB_ENGINE_ENG_CTRL);
	engine_on = true;
	engine_evt_own = false;
	engine_seen = true;
	printk("engine: ENABLE=1 EVT_OWN=0 ACM_BR snoop id=0x%08x evt=0x%08x/%u map=0x%08x\n",
	       id, evt_base, evt_size, sys_read32(base + USB_ENGINE_EP_MAP));
}

static void engine_go_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	engine_try_enable(saved_evt_base, saved_evt_size);
}

void udc_dwc3_engine_go(void)
{
	(void)k_work_cancel_delayable(&engine_go_work);
	engine_try_enable(saved_evt_base, saved_evt_size);
}

void udc_dwc3_engine_program_ep(const struct device *dev, uint8_t addr,
				uint32_t depcmd_addr, uint32_t xfer_idx,
				uint32_t evt_base, uint32_t evt_size)
{
	const int idx = usb_engine_bulk_idx(addr);
	mm_reg_t epb;

	ARG_UNUSED(dev);
	if (idx < 0) {
		return;
	}
	if (!udc_dwc3_engine_present() &&
	    (sys_read32(eng_base() + USB_ENGINE_ENG_ID) & 0xffffU) != USB_ENGINE_MAGIC) {
		return;
	}
	epb = usb_engine_ep_base(idx);
	sys_write32(USB_ENGINE_TRB_BASE_DEFAULT + (uint32_t)idx * 128U,
		    epb + USB_ENGINE_EP_TRB_BASE);
	sys_write32(depcmd_addr, epb + USB_ENGINE_EP_DEPCMD_ADDR);
	sys_write32(xfer_idx, epb + USB_ENGINE_EP_XFER_IDX);
	programmed |= (uint8_t)BIT(idx);
	saved_evt_base = evt_base;
	saved_evt_size = evt_size;
	printk("engine: program ep=0x%02x slot=%d depcmd=0x%08x idx=%u (%u/3)\n",
	       addr, idx, depcmd_addr, xfer_idx, (uint32_t)__builtin_popcount(programmed));
	/* ENABLE is STREAMON (`udc_dwc3_engine_go`): video RTL owns 0x85
	 * first so VID_CREDIT does not starve EP0 during probe.
	 */
}

void udc_dwc3_engine_disable(void)
{
	if (!engine_on && programmed == 0U) {
		return;
	}
	(void)k_work_cancel_delayable(&engine_go_work);
	if (udc_dwc3_engine_present()) {
		sys_write32(0, eng_base() + USB_ENGINE_ENG_CTRL);
	}
	engine_on = false;
	engine_evt_own = false;
	programmed = 0;
	for (int i = 0; i < USB_ENGINE_BULK_N; i++) {
		posted_clear(i);
	}
	printk("engine: ENABLE=0\n");
}

int udc_dwc3_engine_post(const struct device *dev, uint8_t addr,
			 struct net_buf *buf, uint32_t ctrl)
{
	const int idx = usb_engine_bulk_idx(addr);
	mm_reg_t epb;
	uint32_t free;
	uint32_t len;

	ARG_UNUSED(dev);
	if (!engine_on || idx < 0 || buf == NULL) {
		return -EINVAL;
	}
	epb = usb_engine_ep_base(idx);
	free = sys_read32(epb + USB_ENGINE_EP_DESC_FREE);
	if (free == 0U) {
		return -EBUSY;
	}
	if (posted_push(idx, buf) != 0) {
		return -EBUSY;
	}
	len = USB_EP_DIR_IS_IN(addr) ? buf->len : buf->size;
	sys_write32((uint32_t)(uintptr_t)buf->data, epb + USB_ENGINE_EP_DESC_ADDR);
	sys_write32(len, epb + USB_ENGINE_EP_DESC_LEN);
	sys_write32(ctrl, epb + USB_ENGINE_EP_DESC_CTRL);
	return 0;
}

int udc_dwc3_engine_kick(uint8_t addr, uint32_t trb_addr,
			 uint32_t data_addr, uint32_t len, uint32_t ctrl)
{
	const int idx = usb_engine_bulk_idx(addr);
	mm_reg_t epb;

	if (!engine_on || idx < 0) {
		return -EINVAL;
	}
	epb = usb_engine_ep_base(idx);
	if (sys_read32(epb + USB_ENGINE_EP_DESC_FREE) == 0U) {
		return -EBUSY;
	}
	if (trb_addr != 0U) {
		sys_write32(trb_addr, epb + USB_ENGINE_EP_TRB_BASE);
	}
	sys_write32(data_addr, epb + USB_ENGINE_EP_DESC_ADDR);
	sys_write32(len, epb + USB_ENGINE_EP_DESC_LEN);
	sys_write32(ctrl, epb + USB_ENGINE_EP_DESC_CTRL);
	return 0;
}

int udc_dwc3_engine_cmd(uint8_t addr, uint32_t ctrl)
{
	const int idx = usb_engine_bulk_idx(addr);
	mm_reg_t epb;

	if (!engine_on || idx < 0) {
		return -EINVAL;
	}
	epb = usb_engine_ep_base(idx);
	if (sys_read32(epb + USB_ENGINE_EP_DESC_FREE) == 0U) {
		return -EBUSY;
	}
	sys_write32(0, epb + USB_ENGINE_EP_DESC_ADDR);
	sys_write32(0, epb + USB_ENGINE_EP_DESC_LEN);
	sys_write32(ctrl, epb + USB_ENGINE_EP_DESC_CTRL);
	return 0;
}

static void drain_cmpl(const struct device *dev, int idx)
{
	const mm_reg_t epb = usb_engine_ep_base(idx);
	const uint8_t addr = bulk_addr[idx];

	while (sys_read32(epb + USB_ENGINE_EP_CMPL_LEVEL) != 0U) {
		struct net_buf *buf;
		uint32_t caddr = sys_read32(epb + USB_ENGINE_EP_CMPL_ADDR);
		uint32_t clen = sys_read32(epb + USB_ENGINE_EP_CMPL_LEN);
		uint32_t cstat = sys_read32(epb + USB_ENGINE_EP_CMPL_STAT);
		int16_t err = (int16_t)(cstat & 0xffffU);

		buf = posted_take(idx, caddr);
		if (buf == NULL) {
			continue;
		}
		if (USB_EP_DIR_IS_OUT(addr)) {
			if (clen > buf->size) {
				clen = buf->size;
			}
			buf->len = clen;
		}
		udc_submit_ep_event(dev, buf, err);
	}
}

void udc_dwc3_engine_poll(const struct device *dev,
			  void (*fwd)(const struct device *dev, uint32_t evt))
{
	const mm_reg_t base = eng_base();
	uint32_t ev;

	if (!engine_on) {
		return;
	}

	/*
	 * EVT_OWN=0: CPU already walks GEVNTCOUNT. Draining FWD_POP here
	 * re-runs the same EP0 events and eats the SETUP buffer (then -71).
	 */
	if (engine_evt_own) {
		for (;;) {
			ev = sys_read32(base + USB_ENGINE_FWD_POP);
			if (ev == 0U) {
				break;
			}
			if (fwd != NULL) {
				fwd(dev, ev);
			}
		}
	}

	for (int i = 0; i < USB_ENGINE_BULK_N; i++) {
		drain_cmpl(dev, i);
	}
}
