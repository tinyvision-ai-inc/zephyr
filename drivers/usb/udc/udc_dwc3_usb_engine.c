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
static bool engine_held_off;
static bool engine_evt_own;


/*
 * Closed-loop video. 0 keeps the historical open-loop arrangement: EVT_OWN=0,
 * no DEPEVTs on 0x85, video doorbells paced only by camera pages. 1 hands the
 * event ring to EventDrain, adds IN 0x85 to EP_OWN, and (in udc_dwc3.c) turns
 * on XferInProgress for the video endpoint, so the arbiter sees real video
 * completions instead of inferring safety from a GDBGFIFOSPACE probe and a
 * timer. Both reference stacks pace the next command from that event; this is
 * the only data path here that does not.
 *
 * Risk if it misbehaves: with EVT_OWN=1 the CPU no longer services
 * GEVNTCOUNT, so EP0/DEVT handling depends on FWD_POP keeping up with a
 * 64-deep FIFO. Falling behind raises EVT_OVF and the engine stops retiring
 * video descriptors - watch for the "engine: EVT_OVF" line.
 *
 * 26 Oct 2026: tried on the board and it does not work yet. Enumeration and
 * one 8 s Y16 run at 60 fps were clean, then EP0 stalled permanently -
 * uvcvideo "Failed to set UVC probe control" -71 then -110 every 5.12 s, the
 * firmware console silent, no "engine: EVT_OVF". A silent stall rather than
 * an overflow points at the handoff rather than FWD throughput: ENG_CTRL=0 at
 * STREAMOFF takes EVT_OWN away mid-flight, and after that GEVNTCOUNT and the
 * driver's ring read pointer no longer agree, so the CPU never sees another
 * event. EP_OWN is not cleared on disable either. Fix that handoff before
 * setting this to 1 again.
 *
 * Note for stage 3 (pacing the video doorbell off a completion event inside
 * the fabric): EventDrain snoops the ring whatever EVT_OWN says, so the
 * arbiter can see video completions with EVT_OWN=0. What EVT_OWN buys is
 * keeping those ~9600 events/s off the CPU, not the events themselves.
 */
extern void udc_dwc3_evt_ring_resync(const char *why);

/*
 * 27 Oct 2026: back to 0 for good. EVT_OWN is not the way in, and the reason
 * is structural, not a handoff bug: TinyClunx io.irq is
 *   Mux(enable && evtOwn, usbEngine.irq, usbCore.io.irq)
 * so EVT_OWN=1 repoints the CPU's only interrupt at ENG_IRQ, whose FWD bit is
 * driven from fwdLevel =/= 0 and is cleared only by FWD_POP - which happens in
 * udc_dwc3_engine_poll() on the event thread, never in the ISR. A forwarded
 * EP0 event therefore raises a level interrupt the ISR cannot dismiss. That is
 * the permanent EP0 stall, and it is why both attempts failed on iteration 1.
 *
 * ACK_OWNED below gets what EVT_OWN was wanted for - video events kept off the
 * CPU - without touching the interrupt source.
 */
uint32_t udc_dwc3_vid_evt_own;
/* See udc_dwc3.c: kept 0 so step 2 tests the EVT_OWN handoff alone. */
/*
 * tinyCLUNX33 dual-UVC: 0. The two video endpoints (0x83/0x84) run open-loop
 * with their DEPEVTs masked, as they do without the engine. This RTL's
 * TRBRamSink has no event-paced doorbell, so a video event would have no
 * consumer in the fabric and would only load the CPU's event ring (~10,800/s
 * at two 1640x1232 streams).
 */
uint32_t udc_dwc3_vid_inprog_en;

/*
 * ACK_OWNED (ENG_CTRL bit 5). EventDrain acks GEVNTCOUNT only for the owned
 * DEPEVTs it consumes; DEVT and EP0 stay pending so the raw DWC3 interrupt
 * still fires and the CPU drains them on its normal path. This also adds IN
 * 0x85 to EP_OWN, which is what makes a video DEPEVT "owned" in the fabric.
 *
 * This is the piece that makes a video event affordable. Measured 26 Oct:
 * with the CPU owning every slot, 2598 video events/s broke the bulk paths
 * 4/4 on iteration 1. Here the CPU never dispatches them - it only steps its
 * ring pointer over the slots the fabric already acked.
 */
/*
 * ACK_OWNED is gone. It was the fabric acking GEVNTCOUNT for the video
 * DEPEVTs it consumed, with the CPU stepping its ring pointer over those
 * slots. Isolated on the board 27 Oct 2026 (config B, gate off, ctrl=0x39):
 * EP0 errored immediately - "CTRL IN completion with no buffer". Splitting
 * GEVNTCOUNT between two writers is the fault, and it needs no fix because it
 * needs no existence: the CPU only had to stop *dispatching* those events,
 * not stop retiring them. See udc_dwc3_evt_is_video() in udc_dwc3.c.
 */

/*
 * VID_EVT (ENG_CTRL bit 2). Pace the video doorbell on the controller's own
 * event in TRBRamSink instead of posting one per committed TRB. Needs a video
 * event to exist, so udc_dwc3_vid_nrdy_en must be on too, and needs
 * udc_dwc3_ack_owned so those events do not land on the CPU.
 */
/*
 * Compile-time, not a variable. This firmware sits at the RAM ceiling: 65492
 * bytes boots, 65508 does not (measured 27 Oct 2026 - four failed loads with
 * no console output at all, against the same bitstream that boots at 65492).
 * A third initialised global was enough to push .data padding over the edge,
 * so the switch that only this file reads became a constant.
 */
/* 0 here: see udc_dwc3_vid_inprog_en. Keeps EP_OWN to ACM IN 0x82 only. */
#define UDC_DWC3_VID_DOORBELL_EVT	0U

/* Defined in udc_dwc3.c; printed in the banner so one line states the
 * whole event-pacing configuration. */
extern uint32_t udc_dwc3_vid_nrdy_en;

/*
 * QUIET_OTHER, written at STREAMON. 60000 cycles = 800 us at 75 MHz.
 *
 * This used to be hardcoded 0, which with the pre-hold02 RTL deleted the whole
 * CPU-command hold ladder for exactly the condition the wedge needs. hold02
 * split the knob: acmIn/rawIn (the 0x82/0x84 packet hold, holdMin 0,
 * event-released) now arm unconditionally, while everyOther and flatRest keep
 * their (quietOther =/= 0) gate.
 *
 * So this value now controls *only* the flat hold on EP0 and the remaining CPU
 * commands - the half with a hard holdMin, and the half that cost frames when
 * the 800 us scheme was tried. hold02 with this at 0 wedged at ~30 min, so the
 * packet hold alone is not sufficient; 60000 restores the full configuration
 * that was measured to remove the wedge completely, with the packet hold no
 * longer depending on it.
 *
 * 28 Oct 2026: 1500 (20 us) was TRIED AND WEDGED. Do not reduce this.
 *
 * Reasoning that failed: pace04 caps the video doorbell at ~4580/s and had
 * ovf=0 across 6+ h, so the event-ring overflow mechanism was fixed and the
 * flat hold looked redundant - keeping only its frame cost. It is not
 * redundant. Measured at 1500 with the licensed pace04 bit:
 *
 *   dwc3: WEDGE ep=N again within holdoff   x89
 *   PARK0 wedge ltssm/lsp/pipectl           x8
 *   ERROR: timeout reading 9 bytes          x38   (0x82 stopped answering)
 *   video dead, cmdrate vid=0, ovf=0
 *
 * It wedged within ~4 minutes. **ovf=0 is the point**: this is a different
 * failure from the one pacing fixes. Pacing removes the event-ring overflow
 * that loses ACM completions; the flat everyOther hold gaps CPU commands away
 * from video DEPCMDs so the hard IP's single command engine is not handed
 * back-to-back work. Two mechanisms, two mitigations, neither replaces the
 * other.
 *
 * Note it failed FASTER than 0 did (~4 min vs ~30 min above), so this is not
 * a monotonic "more is safer" dial. A short everyOther hold arms and then
 * releases, plausibly inside the window it is meant to cover, which may be
 * worse than never arming. Do not assume an untested intermediate value is
 * between the two in safety.
 *
 * The ~1.5% frame loss under SRP (avg_fps ~58.97 vs 60.00 idle) is the price
 * of wedge immunity and is NOT tunable from here. HOLD_SEL (0x54) cannot help
 * either: it only reaches flatRest, gated on !io.engineEn, and the engine is
 * enabled whenever video streams. HOLD_NARROW (0x5c) is read by nothing
 * ("0x5c stays in the map. This build does not branch on it."). Recovering
 * those frames needs the RTL fix in "The fix" - a bounded minimum gap after a
 * no-response write (32-64 cycles) instead of an 800 us endpoint hold - not a
 * firmware constant.
 */
/*
 * 28 Oct 2026, verdict on 7500 (100 us): IT WEDGES. Reverted to 60000.
 *
 * Loaded 04:34:23, logs stop 04:42:34 - 8 min 11 s to the wedge. Same
 * signature as always, so there is no doubt about what it was:
 *   WEDGE ep=0x82 slot=2 st=0x00010273 trb=0x00000811   HWO still set
 *   cmd5=0x00030007  cmd11=0x00080007                   the 0x82 cmd + the doorbell
 *   to=0 tocmd=0x00000000                               CMDACT cleared every time
 *   ovf=0                                               not the event ring
 * Video and SRP were both healthy right up to it: vid=90850/20s (4543/s, the
 * paced rate), 202/202 SRP responses OK, 0 CRC errors, fps mean 59.888.
 *
 * Dose-response, now three points:
 *   20 us  (1500)   wedged in  ~4 min
 *   100 us (7500)   wedged in  8 min 11 s
 *   800 us (60000)  clean over 6+ h (3h52m + 2h18m + others)
 *
 * The important consequence is not that 100 us is too small; it is that the
 * required gap is LARGER than the frame-slack ceiling (104 us = one 16 KiB
 * page at 157.3 MB/s). So the ~1.7% frame loss cannot be fixed by shortening
 * this constant at all - any safe value overruns the video slack by
 * construction. Do not bisect between 100 and 800 us looking for a free
 * value: the previous version of this comment already called that out, and
 * the measurement confirmed it.
 *
 * The frame loss is a ring-depth problem, not a hold-length problem:
 *   7 usable slots x 104 us = 729 us of buffered video  <  800 us hold
 * so the ring drains to empty on every hold. The fix is trbIDBits 3 -> 4
 * (16 entries, 15 usable = 1.56 ms of slack), which covers an 800 us hold
 * with ~2x margin. That is RTL + a new licensed bitstream, not firmware.
 *
 * zephyr_quiet7500.bin must not be used again. Same status as
 * zephyr_quiet1500.bin.
 */
uint32_t udc_dwc3_quiet_other = 60000U;
static bool engine_ovf_seen;
static uint8_t programmed;
static uint32_t saved_evt_base;
static uint32_t saved_evt_size;
static void engine_go_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(engine_go_work, engine_go_fn);
static struct net_buf *posted[USB_ENGINE_BULK_N][USB_ENGINE_POST_DEPTH];
static uint8_t post_hd[USB_ENGINE_BULK_N];
static uint8_t post_tl[USB_ENGINE_BULK_N];
static uint8_t post_n[USB_ENGINE_BULK_N];
/* Slots posted into the CPU's own TRB ring (ring_sync + post_slot).
 * A normal completion retires the matching IN net_buf from drain_cmpl.
 * Not counted in post_n, so stall/clear/dequeue stay CPU. */
static uint32_t kicked[USB_ENGINE_BULK_N];
static uint32_t kicked_total[USB_ENGINE_BULK_N];
static uint32_t kicked_abort[USB_ENGINE_BULK_N];
static bool ring_synced[USB_ENGINE_BULK_N];

static const uint8_t bulk_addr[USB_ENGINE_BULK_N] = {
	0x84, 0x82, 0x02,
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
	const int idx = usb_engine_bulk_idx(addr);

	/* Stall, clear, and dequeue follow the ring only while it holds a
	 * descriptor. With post_n == 0 the CPU TRB ring owns the endpoint. */
	return engine_on && idx >= 0 && post_n[idx] > 0U;
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
	DWC3_STATS("engine: st=0x%x evt=%u cmd=%u own=0x%x post=%u/%u/%u kick82=%u/%u kick84=%u/%u kick02=%u/%u abort=%u sync=%u/%u/%u\n",
	       sys_read32(base + USB_ENGINE_ENG_STATUS),
	       sys_read32(base + USB_ENGINE_EVT_COUNT),
	       sys_read32(base + USB_ENGINE_CMD_COUNT),
	       sys_read32(base + USB_ENGINE_EP_OWN),
	       post_n[0], post_n[1], post_n[2],
	       kicked_total[1], kicked[1], kicked_total[0], kicked[0],
	       kicked_total[2], kicked[2],
	       kicked_abort[1] + kicked_abort[0] + kicked_abort[2],
	       ring_synced[1], ring_synced[0], ring_synced[2]);
	for (int i = 0; i < USB_ENGINE_BULK_N; i++) {
		const mm_reg_t epb = usb_engine_ep_base(i);

		DWC3_STATS("engine: ep%u 0x%02x free=%u cmpl=%u st=0x%x\n",
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

	/* Bit 1 is ACM IN 0x82, programmed before STREAMON. Bits 0 and 2 are
	 * RAW 0x84 / 0x02, which start on the first SRP transfer and must not
	 * hold ENABLE off. */
	if (engine_on || engine_held_off || (programmed & 0x02U) != 0x02U) {
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
	/* bitN = IN 0x8N. BIT(5) adds the video endpoint so EventDrain consumes
	 * its DEPEVTs in the fabric and the CPU never takes the IRQ. */
	/*
	 * BIT(5) adds video IN 0x85 to EP_OWN, which is what makes EventDrain
	 * route its DEPEVTs to ownedEvt - and therefore what makes vidAsk, and
	 * the event-paced doorbell, work at all. Needed by VID_EVT on its own:
	 * it does NOT imply ACK_OWNED.
	 */
	sys_write32(USB_ENGINE_EP_OWN_DEFAULT |
		    (((udc_dwc3_vid_evt_own != 0U) ||
		      (UDC_DWC3_VID_DOORBELL_EVT != 0U)) ? BIT(5) : 0U),
		    base + USB_ENGINE_EP_OWN);

	sys_write32(evt_base, base + USB_ENGINE_EVT_BASE);
	sys_write32(evt_size, base + USB_ENGINE_EVT_SIZE);
	/* Fair-share: yield to mailbox after this many video grants. */
	sys_write32(32U, base + USB_ENGINE_VID_CREDIT);
	sys_write32(udc_dwc3_quiet_other, base + USB_ENGINE_QUIET_OTHER);
	/* A CPU command still occupies the arbiter until CMDACT clears.
	 * HOLD_SEL stays at its reset 0. VID_CMD bit0 resets to 1, which writes
	 * a video UpdateXfer with CMDACT clear and grants the next command
	 * without waiting. */
	sys_write32(udc_dwc3_quiet_other, (mm_reg_t)0xb4008000U + 0x38U);
	sys_write32(0U, (mm_reg_t)0xb4008000U + 0x58U);
	/*
	 * HOLD_NARROW (0x5c) bit 1 = EARLY_FUSE_QUIET (fuse01 bitstream and
	 * later). The 0x82/0x84 hold normally ends on evtRelease, microseconds
	 * after the arm. Bit 1 bounds a *missed* event at QUIET_OTHER (800 us)
	 * instead of the 20 ms cpuQuiet backstop. A miss then costs ~5% of a
	 * 60 fps frame instead of 1.2 frames. Measured evidence the fuse does
	 * fire: a wedge dump read maxus=1507507 against a 1500000-cycle fuse.
	 * Bit 0 (acmNarrow) stays 0.
	 */
	sys_write32(BIT(1), (mm_reg_t)0xb4008000U + 0x5cU);
	sys_write32(USB_ENGINE_IRQ_FWD | USB_ENGINE_IRQ_CMPL | USB_ENGINE_IRQ_ERR,
		    base + USB_ENGINE_ENG_IRQ_ENA);
	/*
	 * ENABLE at STREAMON. EVT_OWN stays 0 so the CPU keeps GEVNTCOUNT.
	 * EventDrain still snoops the ring. Mailbox quiet is 0, so a CPU
	 * command does not arm the 800 µs hold.
	 */
	ctrl = USB_ENGINE_CTRL_ENABLE | USB_ENGINE_CTRL_CREDIT_EN |
	       USB_ENGINE_CTRL_IRQ_EN;
	if (udc_dwc3_vid_evt_own != 0U) {
		ctrl |= USB_ENGINE_CTRL_EVT_OWN;
	}
	if (UDC_DWC3_VID_DOORBELL_EVT != 0U || IS_ENABLED(CONFIG_UDC_DWC3_VID_PACE)) {
		ctrl |= USB_ENGINE_CTRL_VID_EVT;
	}
	sys_write32(ctrl, base + USB_ENGINE_ENG_CTRL);
	engine_on = true;
	engine_held_off = false;
	engine_evt_own = (udc_dwc3_vid_evt_own != 0U);
	engine_seen = true;
	engine_ovf_seen = false;
	printk("engine: ENABLE=1 ackown=%u videvt=%u nrdy=%u EVT_OWN=%u ep_own=0x%08x id=0x%08x evt=0x%08x/%u map=0x%08x ctrl=0x%08x quiet=%u hold=%u vid=%u\n",
	       0U,
	       (UDC_DWC3_VID_DOORBELL_EVT != 0U) ? 1U : 0U,
	       udc_dwc3_vid_nrdy_en,
	       (udc_dwc3_vid_evt_own != 0U) ? 1U : 0U,
	       sys_read32(base + USB_ENGINE_EP_OWN),
	       id, evt_base, evt_size, sys_read32(base + USB_ENGINE_EP_MAP),
	       sys_read32(base + USB_ENGINE_ENG_CTRL),
	       sys_read32((mm_reg_t)0xb4008000U + 0x38U),
	       sys_read32((mm_reg_t)0xb4008000U + 0x54U),
	       sys_read32((mm_reg_t)0xb4008000U + 0x58U) & 1U);
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
	/* New resource: the ring slot must be reloaded before the next kick. */
	ring_synced[idx] = false;
	programmed |= (uint8_t)BIT(idx);
	saved_evt_base = evt_base;
	saved_evt_size = evt_size;
	DWC3_TRACE("engine: program ep=0x%02x slot=%d depcmd=0x%08x idx=%u (%u/3)\n",
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
	const bool had_evt_own = engine_evt_own;

	(void)k_work_cancel_delayable(&engine_go_work);
	if (udc_dwc3_engine_present()) {
		sys_write32(0, eng_base() + USB_ENGINE_ENG_CTRL);
		/*
		 * EP_OWN was never cleared here. EventDrain gates its
		 * GEVNTCOUNT ack on io.enable, so with ENABLE=0 nothing acks
		 * the ring - but a stale EP_OWN still declares those endpoints
		 * fabric-owned to anything that reads it, and leaves the
		 * register disagreeing with engine_on. Clear it with ENABLE.
		 */
		sys_write32(0, eng_base() + USB_ENGINE_EP_OWN);
	}
	engine_on = false;
	engine_held_off = false;
	engine_evt_own = false;
	programmed = 0;
	/*
	 * The CPU is about to resume reading the event ring. Its read index
	 * has been frozen for as long as the fabric owned the ring (the fabric
	 * acked GEVNTCOUNT for the events it forwarded over FWD, which the CPU
	 * consumed without advancing evt_next), so it no longer agrees with
	 * the controller's write pointer. Re-synchronise before the first CPU
	 * read, or EP0 stalls permanently on stale ring entries.
	 *
	 * Only this edge needs it. On 0->1 the CPU owns the ring right up to
	 * the handoff, so its index is correct and the fabric simply starts
	 * acking from wherever the controller is - and a resync there would
	 * discard events in the middle of the control transfer that triggers
	 * STREAMON.
	 */
	if (had_evt_own) {
		udc_dwc3_evt_ring_resync("EVT_OWN 1->0");
	}
	for (int i = 0; i < USB_ENGINE_BULK_N; i++) {
		posted_clear(i);
		/* Queued slots abort on ENABLE=0; drain_cmpl hands them back. */
		ring_synced[i] = false;
	}
	printk("engine: ENABLE=0 kick82=%u pending=%u abort=%u\n",
	       kicked_total[1], kicked[1], kicked_abort[1]);
}

int udc_dwc3_engine_ring_sync(uint8_t addr, uint32_t trb_base, uint8_t ring_n,
			      uint8_t slot)
{
	const int idx = usb_engine_bulk_idx(addr);
	mm_reg_t epb;

	if (!engine_on || idx < 0) {
		return -EINVAL;
	}
	epb = usb_engine_ep_base(idx);
	/* slotLoad resets the ring's write slot and resource ownership. Never
	 * move it under a descriptor that is queued or in flight. */
	if ((sys_read32(epb + USB_ENGINE_EP_STATE) & USB_ENGINE_EP_STATE_PENDING) != 0U ||
	    kicked[idx] != 0U) {
		return -EBUSY;
	}
	sys_write32(trb_base, epb + USB_ENGINE_EP_TRB_BASE);
	sys_write32(((uint32_t)slot << 8) | ring_n,
		    eng_base() + USB_ENGINE_RING_CFG(idx));
	ring_synced[idx] = true;
	DWC3_TRACE("engine: ring sync ep=0x%02x base=0x%08x n=%u slot=%u idx=%u\n",
	       addr, trb_base, ring_n, slot,
	       sys_read32(epb + USB_ENGINE_EP_XFER_IDX));
	return 0;
}

bool udc_dwc3_engine_ring_synced(uint8_t addr)
{
	const int idx = usb_engine_bulk_idx(addr);

	return engine_on && idx >= 0 && ring_synced[idx];
}

void udc_dwc3_engine_ring_unsync(uint8_t addr)
{
	const int idx = usb_engine_bulk_idx(addr);

	if (idx >= 0) {
		ring_synced[idx] = false;
	}
}

int udc_dwc3_engine_post_slot(uint8_t addr, struct net_buf *buf, uint32_t post_ctrl,
			      const void *dma)
{
	const int idx = usb_engine_bulk_idx(addr);
	mm_reg_t epb;

	if (!engine_on || idx < 0 || buf == NULL || dma == NULL || !ring_synced[idx]) {
		return -EINVAL;
	}
	epb = usb_engine_ep_base(idx);
	if (sys_read32(epb + USB_ENGINE_EP_DESC_FREE) == 0U) {
		return -EBUSY;
	}
	kicked[idx]++;
	kicked_total[idx]++;
	sys_write32((uint32_t)(uintptr_t)dma, epb + USB_ENGINE_EP_DESC_ADDR);
	sys_write32(USB_EP_DIR_IS_IN(addr) ? buf->len : buf->size,
		    epb + USB_ENGINE_EP_DESC_LEN);
	/* DESC_CTRL write is the post. TRB dword3 comes from the TRB_CTRL
	 * template (HWO|CSP|NORMAL|IOC); IOC/ZLP bits here force IOC / ZLP. */
	sys_write32(post_ctrl, epb + USB_ENGINE_EP_DESC_CTRL);
	return 0;
}

uint32_t udc_dwc3_engine_kicked(uint8_t addr)
{
	const int idx = usb_engine_bulk_idx(addr);

	return idx >= 0 ? kicked[idx] : 0U;
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
	if (addr == 0x02U || addr == 0x82U) {
		DWC3_TRACE("ENGCMD t=%u ep=0x%02x ctrl=0x%x post=%u st=0x%x\n",
		       (uint32_t)k_uptime_get(), addr, ctrl, post_n[idx],
		       sys_read32(epb + USB_ENGINE_EP_STATE));
	}
	if (sys_read32(epb + USB_ENGINE_EP_DESC_FREE) == 0U) {
		return -EBUSY;
	}
	sys_write32(0, epb + USB_ENGINE_EP_DESC_ADDR);
	sys_write32(0, epb + USB_ENGINE_EP_DESC_LEN);
	sys_write32(ctrl, epb + USB_ENGINE_EP_DESC_CTRL);
	return 0;
}

/* EP_STATE: bit1 active, bit3 restarted after an IN miss. */
static uint32_t acm82_prev;
static bool acm82_saw_cmpl;

/* Bit per bulk index. The EndXfer that restarts a stuck IN also posts
 * a transfer-complete, and the ring would hand that buffer to the class
 * before the host has it. Eat that one completion. */
static uint8_t restart_swallow;
static uint8_t restart_ate;

static int restart_idx(uint8_t addr)
{
	const int idx = usb_engine_bulk_idx(addr);

	if (idx < 0 || idx >= USB_ENGINE_BULK_N) {
		return -1;
	}
	return idx;
}

void udc_dwc3_engine_restart_arm(uint8_t addr)
{
	const int idx = restart_idx(addr);

	if (idx < 0) {
		return;
	}
	restart_ate &= (uint8_t)~BIT(idx);
	restart_swallow |= (uint8_t)BIT(idx);
}

void udc_dwc3_engine_restart_disarm(uint8_t addr)
{
	const int idx = restart_idx(addr);

	if (idx < 0) {
		return;
	}
	restart_swallow &= (uint8_t)~BIT(idx);
}

bool udc_dwc3_engine_restart_ate(uint8_t addr)
{
	const int idx = restart_idx(addr);

	return idx >= 0 && (restart_ate & BIT(idx)) != 0U;
}

static void acm82_note(const char *why, uint32_t st, uint32_t len, uint32_t stat)
{
	DWC3_TRACE("ACM82 t=%u %s st=0x%x len=%u stat=0x%x\n",
	       (uint32_t)k_uptime_get(), why, st, len, stat);
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

		/* Slots posted into the CPU ring. Abort re-arms. A normal
		 * completion retires the matching net_buf here: the DEPEVT
		 * for this TRB is dropped while HWO or SW_PENDING is set
		 * and is never retried, which leaves the class waiting.
		 */
		if (kicked[idx] != 0U && post_n[idx] == 0U) {
			if ((restart_swallow & BIT(idx)) != 0U) {
				restart_swallow &= (uint8_t)~BIT(idx);
				restart_ate |= (uint8_t)BIT(idx);
				kicked[idx]--;
				continue;
			}
			kicked[idx]--;
			if (idx == 1) {
				acm82_saw_cmpl = true;
			}
			if ((cstat & 0xffffU) == USB_ENGINE_CMPL_STAT_ABORT) {
				kicked_abort[idx]++;
				udc_dwc3_engine_slot_aborted(dev, addr, caddr);
			} else if (USB_EP_DIR_IS_IN(addr)) {
				udc_dwc3_engine_slot_done(dev, addr, caddr, clen);
			}
			continue;
		}

		buf = posted_take(idx, caddr);
		if (idx == 1) {
			acm82_saw_cmpl = true;
			acm82_note(buf != NULL ? "cmpl" : "drop",
				   sys_read32(epb + USB_ENGINE_EP_STATE), clen, cstat);
		}
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

int udc_dwc3_engine_restart_resync(const struct device *dev, uint8_t addr,
				   uint32_t trb_base, uint8_t ring_n,
				   uint8_t slot)
{
	const int idx = restart_idx(addr);
	mm_reg_t epb;
	int spins;

	if (idx < 0) {
		return -EINVAL;
	}
	/* slotLoad clears the ring's own resource index. A later kick is
	 * then a StartXfer, because the CSR index is written back to 0
	 * and an End command clears `started`. */
	if (udc_dwc3_engine_ring_sync(addr, trb_base, ring_n, slot) != 0) {
		return -EBUSY;
	}
	epb = usb_engine_ep_base(idx);
	sys_write32(0U, epb + USB_ENGINE_EP_XFER_IDX);
	if (udc_dwc3_engine_cmd(addr, USB_ENGINE_DESC_CTRL_END) != 0) {
		return -EBUSY;
	}
	/* EndXfer of an idle resource. The completion is an abort with
	 * nothing in flight, so drain drops it and `started` is clear. */
	k_busy_wait(500);
	for (spins = 0; spins < 8; spins++) {
		drain_cmpl(dev, idx);
	}
	return 0;
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
	 * The EventDrain snoops the ring whatever EVT_OWN says, so FWD has to
	 * be popped either way. Leaving it unread fills the FIFO, raises
	 * EVT_OVF and the engine then stops retiring video descriptors.
	 * With EVT_OWN=0 the CPU already ran this event off GEVNTCOUNT:
	 * discard it here, replaying it eats the SETUP buffer (-71).
	 */
	for (;;) {
		ev = sys_read32(base + USB_ENGINE_FWD_POP);
		if (ev == 0U) {
			break;
		}
		if (engine_evt_own && fwd != NULL) {
			fwd(dev, ev);
		}
	}

	if ((sys_read32(base + USB_ENGINE_ENG_STATUS) & USB_ENGINE_STAT_EVT_OVF) != 0U) {
		sys_write32(USB_ENGINE_IRQ_ERR, base + USB_ENGINE_ENG_IRQ);
		if (!engine_ovf_seen) {
			engine_ovf_seen = true;
			printk("engine: EVT_OVF (evt=%u) cleared, FWD drained\n",
			       sys_read32(base + USB_ENGINE_EVT_COUNT));
		}
	}

	for (int i = 0; i < USB_ENGINE_BULK_N; i++) {
		drain_cmpl(dev, i);
	}

	/* 0x82 is ep index 1. Print the edges the 5 s dump misses. With every
	 * ACM reply on the ring these edges are per packet: first 8 only,
	 * restarts always. */
	{
		static uint32_t acm82_edges;
		const mm_reg_t epb = usb_engine_ep_base(1);
		const uint32_t st = sys_read32(epb + USB_ENGINE_EP_STATE);
		const uint32_t prev = acm82_prev;

		if ((st & BIT(1)) && !(prev & BIT(1))) {
			acm82_saw_cmpl = false;
			if (acm82_edges < 8U) {
				acm82_edges++;
				acm82_note("active", st, 0U, 0U);
			}
		}
		if ((st & BIT(3)) && !(prev & BIT(3))) {
			acm82_note("restart", st, 0U, 0U);
		}
		if (!(st & BIT(1)) && (prev & BIT(1)) && !acm82_saw_cmpl) {
			acm82_note("idle", st, 0U, 0U);
		}
		acm82_prev = st;
	}
}
