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

#include <zephyr/drivers/usb/udc.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include "udc_common.h"

/*
 * Kconfig knobs, defaulted here so this file builds on its own.
 *
 * The driver is delivered as a single .c and dropped into a tree whose
 * Kconfig.dwc3 belongs to whoever owns that tree - and which may not define
 * every symbol used below. A tree carrying an older DWC3 lineage has no
 * UDC_DWC3_RECOVERY_TIMEOUT at all, and the build then dies on the first
 * K_MSEC() with "undeclared", nowhere near the real cause.
 *
 * KCONFIG STILL WINS wherever it defines one. Zephyr force-includes the
 * generated autoconf.h ahead of this source (-imacros), so a Kconfig-provided
 * value is already defined by the time these tests run and the fallback is
 * skipped. Nothing here overrides a tree that has made its own choice.
 *
 * UDC_DWC3_SHELL is deliberately NOT defaulted. It is a bool, and leaving it
 * undefined is what Kconfig "n" looks like to the preprocessor. Defining it to
 * 0 instead would be worse than useless: this file mixes "#if
 * CONFIG_UDC_DWC3_SHELL" and "#ifdef CONFIG_UDC_DWC3_SHELL", so a value of 0
 * makes the #ifdef true while the #if is false, and an inconsistent hybrid gets
 * compiled.
 *
 * These are a portability shim, not the final home. Fold them into Kconfig.dwc3
 * at release time, where they can carry ranges and help text.
 */
#ifndef CONFIG_UDC_DWC3_EVENTS_NUM
/* 16 entries x 4 bytes = the 64-byte cap this core enforces; see BUILD_ASSERT below. */
#define CONFIG_UDC_DWC3_EVENTS_NUM 16
#endif

#ifndef CONFIG_UDC_DWC3_TRB_NUM
/* Per non-control endpoint. Must be >= 2: the control paths index trb_buf[1]. */
#define CONFIG_UDC_DWC3_TRB_NUM 4
#endif

#ifndef CONFIG_UDC_DWC3_RECOVERY_TIMEOUT
/* ms a control stage may stay outstanding before the watchdog ends the transfer. */
#define CONFIG_UDC_DWC3_RECOVERY_TIMEOUT 1000
#endif

/* TRB memory buffer fields */
#define UDC_DWC3_TRB_STATUS_BUFSIZ_MASK				GENMASK(23, 0)
#define UDC_DWC3_TRB_STATUS_PCM1_MASK				GENMASK(25, 24)
/*
 * Set by the controller when writing a TRB back on an OUT transfer, to mark the
 * last TRB used for that transfer descriptor: "Bit[26] - SPR of the {trbstatus,
 * RSVD, SPR, PCM1, bufsize} dword will be set during an OUT transfer TRB write
 * back if this is the last TRB used for that transfer descriptor."
 */
#define UDC_DWC3_TRB_STATUS_SPR					BIT(26)
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
/*
 * No TRBCTL encoding is 0, so 0 is free to mean "the watchdog is pending but
 * is not guarding a control stage". The deferral in udc_dwc3_ctrl_try() uses
 * it, and it is what watchdog_type is cleared to once a stage completes.
 */
#define UDC_DWC3_WATCHDOG_TYPE_NONE				0U

#define UDC_DWC3_TRB_CTRL_ISP_IMI				BIT(10)
#define UDC_DWC3_TRB_CTRL_IOC					BIT(11)
/*
 * PCM1 and SPR live in the STATUS word, not this one - see Figure 3-1, where the
 * status dword is {TRBSTS[3:0], Rsvd, SPR, PCM1, BUFSIZ[23:0]}. The control word
 * holds only HWO, LST, CHN, CSP, TRBCTL, ISP/IMI, IOC, and Stream ID / SOF
 * Number at 29:14; bits 13:12 and 31:30 are reserved. Definitions for both are
 * with the other TRB status fields above.
 */
#define UDC_DWC3_TRB_CTRL_SIDSOFN_MASK				GENMASK(29, 14)

/* Incomplete coverage of all fields, but suited for what this driver supports */
#define UDC_DWC3_EVT_MASK					GENMASK(11, 0)
#define UDC_DWC3_DEPEVT_EPN_MASK				GENMASK(5, 1)
#define UDC_DWC3_DEPEVT_XFERCOMPLETE(epn)			(((epn) << 1) | (0x01 << 6))
#define UDC_DWC3_DEPEVT_XFERINPROGRESS(epn)			(((epn) << 1) | (0x02 << 6))
#define UDC_DWC3_DEPEVT_XFERNOTREADY(epn)			(((epn) << 1) | (0x03 << 6))
#define UDC_DWC3_DEPEVT_RXTXFIFOEVT(epn)			(((epn) << 1) | (0x04 << 6))
#define UDC_DWC3_DEPEVT_STREAMEVT(epn)				(((epn) << 1) | (0x06 << 6))
#define UDC_DWC3_DEPEVT_EPCMDCMPLT(epn)				(((epn) << 1) | (0x07 << 6))
/*
 * SPEC, Programming Guide 3.30b, DEPEVT field 15:12 "Event Status", within an
 * XferNotReady event (p.326):
 *
 *   "[13:12]: For control endpoints, indicates what stage was requested when
 *    the transfer was not ready:
 *      2'b01: Control Data Request
 *      2'b10: Control Status Request"
 *
 * Only those two are defined. 2'b00 is listed below because the driver reports
 * it as an impossible event rather than accepting it silently, and 2'b11 has no
 * meaning at all - udc_dwc3_ctrl_xnr_check() treats both as suspect rather than
 * as a status request, which is what a bare "not Data" test would do.
 *
 * The mask stops at bit 13 deliberately. Bit 15 of the same field is
 * XferActive/XferNotActive and bit 14 is unused, so widening it would fold the
 * reason for the event into the stage being tested.
 */
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK			GENMASK(13, 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP			(0x0 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA			(0x1 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS			(0x2 << 12)
/*
 * Event Status occupies bits 15:12 of EVERY endpoint-specific event; what it
 * means depends on the event type. All of the per-type decodes below live in
 * that field, including the control-stage mask above.
 *
 * These were previously defined at bits 0..3, which overlap the event's own
 * encoding: bit 0 marks an endpoint-specific event and bits 5:1 carry the
 * endpoint number. A test against them therefore sampled the endpoint number
 * rather than any status.
 */
/* For XferComplete or XferInProgress: short packet received, or the last
 * packet of an isochronous interval.
 */
#define UDC_DWC3_DEPEVT_STATUS_SHORT				BIT(13)
/* IOC bit of the TRB that completed */
#define UDC_DWC3_DEPEVT_STATUS_IOC				BIT(14)
/* For XferComplete: LST bit of the completed TRB */
#define UDC_DWC3_DEPEVT_STATUS_LST				BIT(15)
/* For XferInProgress: the interval did not complete successfully. This shares
 * bit 15 with LST above - the event type is what tells the two apart.
 */
#define UDC_DWC3_DEPEVT_STATUS_MISSED_ISOC			BIT(15)
/*
 * There is no bus-error bit in this field. The name below has no counterpart in
 * the databook at all - "bus error" there refers only to the GBUSERRADDR
 * registers, which report a SoC bus error address and are unrelated to endpoint
 * events. Defined as 0 rather than removed so that anything still referring to
 * it builds, and so a stray test reads false instead of sampling an
 * endpoint-number bit.
 */
#define UDC_DWC3_DEPEVT_STATUS_BUSERR				0u
/* For StreamEvt: 4'h1 StreamFound, 4'h2 StreamNotFound, also in bits 15:12 */
#define UDC_DWC3_DEPEVT_STATUS_STREAMFOUND			(0x1 << 12)
#define UDC_DWC3_DEPEVT_STATUS_STREAMNOTFOUND			(0x2 << 12)
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
/*
 * Device-event payload, databook Table 3-8 field 24:16 (EvtInfo). For a USB/Link
 * State Change event: EvtInfo[4] is set for SuperSpeed, and EvtInfo[3:0] is the
 * link state AT THE TIME OF THE EVENT, in the same encoding as DSTS.
 *
 * This is the only trustworthy source for that state. Re-reading DSTS when the
 * event is handled reports where the link is NOW, and the work queue can be
 * milliseconds behind - so on a link changing hundreds of times a second every
 * transition can be rendered as the same state.
 */
#define UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK			GENMASK(19, 16)
#define UDC_DWC3_DEVT_EVTINFO_SS				BIT(20)
/*
 * Sampling periods for the rate-limited diagnostics, both PRIME on purpose.
 * CONFIG_UDC_DWC3_EVENTS_NUM is 16, so a period that is a multiple of 16 samples
 * the same ring slot every single time whenever the sampled thing advances through
 * the ring by a constant stride. 1024 did exactly that, and made give-ups look
 * confined to two slots when nothing of the sort had been established. Keep these
 * coprime with the ring size.
 */
/* One line per this many repeats of the SAME link state - see udc_dwc3_log_link_event(). */
#define UDC_DWC3_EVT_LINK_LOG_EVERY				257u
/* Liveness check interval, and how long a single dispatch may run. */
/*
 * Heartbeat tick. Short enough to bound how long a drain can sit unscheduled,
 * because that is the failure it exists to break: the controller generates an
 * interrupt when it WRITES an event, so once the ring is full it has nowhere
 * to write, raises nothing further, and a driver that is not already looking
 * will never be told to look again.
 *
 * The tick is cheap by construction - a GEVNTCOUNT read and two comparisons
 * when there is nothing to do - so it does not compete with the event handler
 * it is watching. Everything it can report is rate limited, so a shorter tick
 * does not mean more console traffic.
 */
#define UDC_DWC3_HEARTBEAT_MS					200u


/*
 * Kick the event handler if it has not COMPLETED a pass within this long while
 * the controller still says events are outstanding.
 *
 * Measured from worker EXIT, never entry: a pass that dispatches an endpoint
 * command can sit a long time in udc_dwc3_depcmd(), and an entry stamp would
 * call that "recently scheduled".  Safe to read from the heartbeat because both
 * run on the same work queue, so the stamp is never mid-pass when compared.
 *
 * A kick can never be wrong: it is gated on GEVNTCOUNT > 0 and resubmitting a
 * queued work item is a no-op.
 *
 * This is NOT sized to meet the 50 ms of USB 2.0 9.2.6.4 - detection costs the
 * threshold plus a full tick, so no free-running value can.  Meeting a request
 * deadline is the drain's job (see the mid-pass note in udc_dwc3_evt_drain());
 * this is the backstop for when the drain is not running at all.
 *
 * It must exceed the driver's OWN console cost.  The stats line is emitted from
 * inside a drain pass and LOG_MODE_MINIMAL makes that synchronous: ~250 chars at
 * 115200 is ~22 ms.  At 20 ms the driver detected its own logging as a stalled
 * drain.  100 ms clears any plausible pass and is still 50x inside the host's 5 s.
 */
#define UDC_DWC3_EVT_IDLE_KICK_MS				100u

/*
 * How old a give-up run must be before the heartbeat reports it. Kept at a
 * second so shortening the tick above did not quietly make this five times
 * more talkative.
 */
#define UDC_DWC3_EVT_GAVEUP_AGE_MS				1000u

/*
 * How long control traffic may stop before the driver says so.
 *
 * The wedge this chases is SILENT BY CONSTRUCTION. When a control claim is
 * never released, udc_dwc3_ctrl_try() turns every later request away with a
 * LOG_DBG that is compiled out at INF, and the SETUP lines simply stop. There
 * is no error, no warning, nothing - the log just ends, which is exactly what
 * uart_v6_1, _2 and _6 all do, and it is why three captures of a dying device
 * have told us nothing about why it died.
 *
 * So silence itself has to be the trigger. This will also fire once on a host
 * that has legitimately stopped issuing control requests, and that is an
 * acceptable price: it is one line, it re-arms when traffic resumes, and the
 * alternative is another capture that ends in nothing.
 */
#define UDC_DWC3_CTRL_QUIET_MS					2000u

/*
 * The point at which the ring has no room left to give.
 *
 * SPEC, Programming Guide 3.30b: "The controller always leaves one entry free
 * in each Event Buffer." So the fullest GEVNTCOUNT can legitimately report is
 * one entry short of the ring, and that - not the whole ring - is the
 * threshold worth reacting to.
 */
#define UDC_DWC3_EVT_RING_FULL_BYTES				\
	(((CONFIG_UDC_DWC3_EVENTS_NUM) - 1u) * sizeof(uint32_t))

/*
 * How long a head slot must stay unreadable, with the ring full, before the
 * drain gives up on it and skips it.
 *
 * Comfortably past both UDC_DWC3_EVT_GAVEUP_RETRY_MS and
 * UDC_DWC3_EVT_IDLE_KICK_MS, so every cheaper remedy has been tried and has
 * failed before an event is deliberately discarded. Ordinary late writes never
 * come near this - they are resolved, or at least looked at again, in
 * microseconds to milliseconds.
 */
#define UDC_DWC3_EVT_SKIP_AFTER_MS				250u

/*
 * How long a give-up run must persist before the heartbeat gives up on the slot.
 *
 * Not the same question as UDC_DWC3_EVT_SKIP_AFTER_MS, which times a single
 * drain pass. This times the RUN: how long gc has stayed above zero while that
 * one slot kept reading the free marker, across repeated re-entries into the
 * handler. Only the run distinguishes a write that is late from one that is
 * gone.
 *
 * A second, deliberately. The longest delayed write ever measured inside the
 * poll budget is 1141 us - three orders of magnitude below - and uart_v6_13
 * recorded a slot that filled legitimately after 636 give-ups, roughly 636 ms.
 * A 250 ms threshold would have discarded that event while it was still on its
 * way. Skipping costs an event permanently, so the threshold belongs well past
 * anything that has ever arrived late.
 */
/*
 * TERMINOLOGY, and it matters because the two differ by roughly three orders of
 * magnitude and are handled by different code:
 *
 *   late write - GEVNTCOUNT announced an event and the word had not landed yet.
 *                udc_dwc3_evt_wait_first() polls for it; counter "late".
 *                Microseconds. Nothing is lost - the word arrives.
 *   give-up    - one poll round ended with the slot still empty; counter
 *                "gaveup". Consecutive give-ups on the same slot form a give-up
 *                run. udc_dwc3_evt_force() provokes a write to break it.
 *                Milliseconds. Still nothing lost.
 *   STALL      - a give-up run that has lasted UDC_DWC3_EVT_DEAD_SLOT_MS. ONLY
 *                this is a stall. It is the sole condition under which
 *                udc_dwc3_evt_skip_dead_slot() runs and an event is DISCARDED.
 *
 * The identifiers were renamed to match: everything about the short path says
 * gaveup, never stall. "Stall" elsewhere in this file means the USB endpoint
 * STALL handshake (Set Stall / Clear Stall) and is unrelated to any of this.
 */
#define UDC_DWC3_EVT_DEAD_SLOT_MS				1000u

/*
 * Whether the heartbeat may ACT on a dead slot, or only report it.
 *
 * Undefine for the build handed to the RTL team: the driver then detects and
 * describes the stall in full but never acknowledges the entry, so the ring,
 * GEVNTCOUNT and the controller stay exactly as the fault left them and the
 * failure remains reproducible. Defined for normal use, where liveness matters
 * more than preserving the scene.
 */
#define UDC_DWC3_EVT_DEAD_SLOT_RECOVER

/*
 * Print the eight setup bytes of every control transfer.
 *
 * Invaluable for tracing a fault, and the dominant cost of running one: the
 * console is synchronous at 115200 under LOG_MODE_MINIMAL, so this line is what
 * sets the control-transfer rate, not the host and not the controller.  A 1 h
 * soak spent 8.9 MB and 339k lines on it and reached only 339k SETUPs.
 *
 * Kept ON.  Turning it off was measured and was a bad trade: it bought only 16%
 * more control transfers per second (93.6 -> 108.9), because the real limit is
 * the host re-execing v4l2-ctl per iteration, not the console.  What it costs is
 * the per-transfer trace - the eight setup bytes that identify the request in
 * flight - which is the first thing wanted when a wedge is being diagnosed.
 * Undefine only for an endurance run where nothing needs to be diagnosed.
 */
#define UDC_DWC3_LOG_EVERY_SETUP

/*
 * Escalate a stuck SETUP to a core soft reset when the Set Stall that normally
 * clears it has failed twice running.
 *
 * Undefine for the diagnostic build handed to the RTL team: without it the
 * controller is left in the wedged state for them to probe, which is the whole
 * point of that build.  With it the device recovers on its own.
 */
#define UDC_DWC3_SETUP_STUCK_RESET

/*
 * How long a slot must stay empty before the write is presumed LOST rather
 * than late.
 *
 * evt_late counts slots that were empty on first read and evt_gaveup counts
 * poll budgets that expired, but neither separates a write that eventually
 * lands from one that never does - and only the second kills the device. The
 * longest delayed write yet measured is 1141 us; a slot still empty after a
 * full second is a different phenomenon, not a slower version of the same
 * one.
 */
#define UDC_DWC3_EVT_MISSED_MS					1000u

/*
 * Hardware state dump on a lost event write, for the RTL side.
 *
 * Enabled in the shipping image on purpose: it costs nothing until a write
 * has ALREADY been lost. The block sits inside the once-per-give-up-run
 * UDC_DWC3_EVT_MISSED_MS branch, so in normal operation not one of these
 * registers is read.
 *
 * It exists because a workaround would hide the fault rather than locate it.
 * What the driver can say on its own - "the slot still holds the free marker
 * and GEVNTCOUNT has not moved" - names the symptom; these registers name the
 * cause, and they are the only way to tell four different faults apart that
 * otherwise look identical from software.
 */
#define STALL_DIAG_LOG

/*
 * What a consumed - or never yet written - event slot holds.
 *
 * The drain has to tell "the controller has not written here yet" from "the
 * controller wrote here", and it does so by value, because the delayed write
 * on this silicon means GEVNTCOUNT can announce an event before the word
 * reaches memory. The value chosen must therefore be one the controller can
 * NEVER produce.
 *
 * 0x00000000 fails that test, which is why it is not used. Decoded, it is a
 * well-formed ENDPOINT event: bit 0 = 0 (endpoint-specific), bits 5:1 = 0
 * (physical endpoint 0), bits 9:6 = 4'h0. The databook lists event type 4'h0
 * as Reserved, and reserved is not the same as impossible on a re-implemented
 * core - so a zero word could be a real event for the busiest endpoint we
 * have, and waiting for it to become non-zero would wait for ever.
 *
 * 0xFFFFFFFF cannot be either class. Bit 0 = 1 makes it non-endpoint-specific,
 * and the databook requires bits 7:1 to be 7'h00 for a device event, where
 * this has 0x7f. Neither encoding can produce it, so "not this value" is a
 * sound test rather than a likely-looking one.
 *
 * Kept as a named constant so it can be changed: any candidate must fail to
 * decode as BOTH an endpoint event (bit 0 = 0) and a device event (bit 0 = 1
 * with bits 7:1 = 0).
 */
#define UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE			0xFFFFFFFFu
#define UDC_DWC3_DISPATCH_STUCK_MS				250u
#define UDC_DWC3_DEVT_VNDRDEVTSTRCVED				(BIT(0) | (0xc << 8))

/* Device Endpoint Commands and Parameters */
#define UDC_DWC3_DEPCMDPAR2(n)					(0xc800 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR1(n)					(0xc804 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR0(n)					(0xc808 + 16 * (n))
#define UDC_DWC3_DEPCMD(n)					(0xc80c + 16 * (n))
/* Common fields to DEPCMD */
#define UDC_DWC3_DEPCMD_HIPRI_FORCERM				(1 << 11)
/*
 * Command Interrupt On Completion. Asks the controller to raise an Endpoint
 * Command Complete event (DEPEVT_EPCMDCMPLT) once the command has finished.
 *
 * This matters on End Transfer: CmdAct clearing means only that the command was
 * ACCEPTED, whereas the databook states the controller "will wait until it can
 * complete operations for the endpoint before returning the Command Complete
 * event". Without it there is no way to know that DMA for the ended transfer
 * has actually stopped before a fresh Start Transfer is issued.
 *
 * The field must not be set while DCTL.RunStop is 0.
 */
#define UDC_DWC3_DEPCMD_CMDIOC					BIT(8)
#define UDC_DWC3_DEPCMD_STATUS_MASK				GENMASK(15, 12)
#define UDC_DWC3_DEPCMD_STATUS_OK				(0 << 12)
#define UDC_DWC3_DEPCMD_STATUS_CMDERR				(1 << 12)
#define UDC_DWC3_DEPCMD_XFERRSCIDX_MASK				GENMASK(22, 16)
/*
 * Returned by udc_dwc3_depcmd() when the command did not succeed, so a failed
 * command cannot be mistaken for a transfer resource index. The field is seven
 * bits wide, so no real index can collide with this value.
 */
#define UDC_DWC3_XFERRSCIDX_INVALID				0xffU
/* DEPCFG Command and Parameters */
/* Command type occupies bits 3:0 - DEPCFG(1) through DEPSTARTCFG(9). */
#define UDC_DWC3_DEPCMD_CMDTYP_MASK				GENMASK(3, 0)
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
#define UDC_DWC3_GSBUSCFG1					0xc104
#define UDC_DWC3_GUCTL1						0xc11c
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
/* Global Rx Threshold Control Register (databook 1.2.4) */
#define UDC_DWC3_GRXTHRCFG					0xc10c
#define UDC_DWC3_GRXTHRCFG_USBRXPKTCNTSEL			BIT(29)
#define UDC_DWC3_GRXTHRCFG_USBRXPKTCNT_MASK			GENMASK(27, 24)
/*
 * Apply the databook 1.2.4 erratum workaround - clear GRXTHRCFG.UsbRxPktCntSel
 * so a fixed NUMP is transmitted rather than one derived from the RX threshold.
 * See udc_dwc3_on_soft_reset() for the citations.
 *
 * Set to 0 to leave the register untouched for an A/B comparison. The value the
 * core powered up with is logged either way.
 */
#define UDC_DWC3_RX_THRESHOLD_WORKAROUND 1

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
#define UDC_DWC3_GCTL_RAMCLKSEL_BUS_CLK				(0x00 << 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_PIPE_CLK			(0x01 << 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_PIPE_DIV2_CLK			(0x02 << 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_MAC2_CLK			(0x03 << 6)
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

/* Global User Control Register 2 */
#define UDC_DWC3_GUCTL2						0xc19c
#define UDC_DWC3_GUCTL2_EN_HP_PM_TIMER				GENMASK(25, 19)
#define UDC_DWC3_GUCTL2_NOLOWPWRDUR				GENMASK(18, 15)
#define UDC_DWC3_GUCTL2_RST_ACTBITLATER				BIT(14)
#define UDC_DWC3_GUCTL2_ENABLEEPCACHEEVICT			BIT(12)
#define UDC_DWC3_GUCTL2_DISABLECFC				BIT(11)
#define UDC_DWC3_GUCTL2_RXPINGDURATION				GENMASK(10, 5)
#define UDC_DWC3_GUCTL2_TXPINGDURATION				GENMASK(4, 0)

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
/*
 * DWC_usb3 Programming Guide 3.30b, section 1.2.56 GEVNTCOUNT(#n), Table 1-68
 * "Fields for Register: GEVNTCOUNT(#n)" (p.197): bits 15:0 EVNTCOUNT, bits 30:16
 * reserved, bit 31 EVNT_HANDLER_BUSY.
 *
 * The same section states the rule the drain is built around, verbatim:
 *
 *   "Clock crossing delays may result in the continuous assertion of the
 *    interrupt after software acknowledges the last event. Therefore, when the
 *    interrupt line is asserted, software must read the GEVNTCOUNT register and
 *    only process events if the GEVNTCOUNT is greater than 0."
 *
 * Note what that permits and what it does not. It licenses ONE read per
 * assertion, taken before processing. It does not license re-reading the
 * register after acknowledging and treating the result as a fresh count: by the
 * databook's own statement that value may still reflect events already handed
 * back. See udc_dwc3_evt_drain(), which reads it exactly once.
 *
 * Only bits 15:0 are the count. Bit 31 is EVNT_HANDLER_BUSY
 * and 30:16 are reserved, so the raw register must never be compared against zero -
 * any of those bits set makes an empty buffer look non-empty, and the drain then
 * reads a slot that legitimately holds nothing and waits out the whole poll budget
 * on it. That is indistinguishable from a lost write in the logs.
 *
 * Read the count through this; keep the raw register only where the point is to
 * SEE those upper bits.
 */
#define UDC_DWC3_GEVNTCOUNT_MASK				GENMASK(15, 0)
#define UDC_DWC3_GEVNTCOUNT_EVNT_HANDLER_BUSY			BIT(31)

static inline uint32_t udc_dwc3_gevntcount(const mm_reg_t base)
{
	return sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) & UDC_DWC3_GEVNTCOUNT_MASK;
}

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
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXQ			(0x0 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXQ			(0x1 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXREQQ			(0x2 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQQ			(0x3 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXINFOQ		(0x4 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOLSTATUSQ	(0x5 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCHQ		(0x6 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_WREVENTQ		(0x7 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENTQ		(0x8 << 5)
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
/* DGCMDPAR for 09h: [4:0] FIFO number, [5] 1 = TX FIFO, 0 = RX FIFO. */
#define UDC_DWC3_DGCMD_FIFOFLUSH_NUM_MASK			GENMASK(4, 0)
#define UDC_DWC3_DGCMD_FIFOFLUSH_TX				BIT(5)
/* Generic commands retire in microseconds; this is a wedge guard, not a wait. */
#define UDC_DWC3_DGCMD_POLL_MAX					1000u
#define UDC_DWC3_DGCMD_FIFOFLUSHALL				(10 << 0)
#define UDC_DWC3_DGCMD_ENDPOINTNRDY				(12 << 0)
#define UDC_DWC3_DGCMD_LOOPBACKTEST				(16 << 0)
#define UDC_DWC3_DGCMD_ROLEREQUEST				(6 << 0)

/* Hardware parameters */
#define UDC_DWC3_GHWPARAMS0					0xc140
#define UDC_DWC3_GHWPARAMS1					0xc144
#define UDC_DWC3_GHWPARAMS2					0xc148
#define UDC_DWC3_GHWPARAMS3					0xc14c
#define UDC_DWC3_GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES_MASK	GENMASK(30, 23)
#define UDC_DWC3_GHWPARAMS3_NUM_IN_EPS_MASK			GENMASK(22, 18)
#define UDC_DWC3_GHWPARAMS3_NUM_EPS_MASK			GENMASK(17, 12)
#define UDC_DWC3_GHWPARAMS4					0xc150
#define UDC_DWC3_GHWPARAMS4_BMU_LSP_DEPTH_MASK			GENMASK(31, 28)
#define UDC_DWC3_GHWPARAMS4_BMU_PTL_DEPTH_M1_MASK		GENMASK(27, 24)
#define UDC_DWC3_GHWPARAMS4_CACHE_TRBS_PER_TRANSFER_MASK	GENMASK(5, 0)
#define UDC_DWC3_GHWPARAMS5					0xc154
#define UDC_DWC3_GHWPARAMS5_DFQ_FIFO_DEPTH_MASK			GENMASK(27, 22)
#define UDC_DWC3_GHWPARAMS5_DWQ_FIFO_DEPTH_MASK			GENMASK(21, 16)
#define UDC_DWC3_GHWPARAMS5_TXQ_FIFO_DEPTH_MASK			GENMASK(15, 10)
#define UDC_DWC3_GHWPARAMS5_RXQ_FIFO_DEPTH_MASK			GENMASK(9, 4)
#define UDC_DWC3_GHWPARAMS5_BMU_BUSGM_DEPTH_MASK		GENMASK(3, 0)
#define UDC_DWC3_GHWPARAMS6					0xc158
#define UDC_DWC3_GHWPARAMS6_RAM0_DEPTH_MASK			GENMASK(31, 16)
#define UDC_DWC3_GHWPARAMS6_PSQ_FIFO_DEPTH_MASK			GENMASK(5, 0)
#define UDC_DWC3_GHWPARAMS7					0xc15c
#define UDC_DWC3_GHWPARAMS7_RAM2_DEPTH_MASK			GENMASK(31, 16)
#define UDC_DWC3_GHWPARAMS7_RAM1_DEPTH_MASK			GENMASK(15, 0)
#define UDC_DWC3_GHWPARAMS8					0xc600

/* Helper macros */
#define LO32(n)			((uint32_t)((uint64_t)(n) & 0xffffffff))
#define HI32(n)			((uint32_t)((uint64_t)(n) >> 32))
#define _EP_DATA_FROM_EPN(cfg, epn) \
	(((epn) & 1) ? &(cfg)->ep_data_in[(epn) >> 1] : &(cfg)->ep_data_out[(epn) >> 1])
/*
 * Is an endpoint number carried by an event actually one of ours?
 *
 * DEPEVT gives the physical endpoint in bits 5:1, so the value ranges over
 * 0..31 and _EP_DATA_FROM_EPN() indexes 0..15 - but this controller is built
 * with six IN and two OUT endpoints. Anything above those, from a corrupted
 * event word or an endpoint this driver never configured, indexes past the
 * arrays and corrupts whatever follows them.
 *
 * The exposure grew when the free-slot marker stopped being zero: words that
 * were previously treated as an empty slot are now dispatched as events, so a
 * damaged slot reaches the handlers with an arbitrary endpoint number instead
 * of being ignored.
 */
#define _EPN_IS_VALID(cfg, epn) \
	(((epn) & 1) ? ((uint32_t)((epn) >> 1) < (cfg)->num_in_eps) \
		     : ((uint32_t)((epn) >> 1) < (cfg)->num_out_eps))
#define _NUM_FIFO_SPACE 16
#define _NUM_AUX_EVENT 8
/*
 * Queue types dumped by "dwc3 fifo". Must match the number of entries in
 * udc_dwc3_fifo_regs[] - the array is declared with this bound, so adding a
 * queue type without raising it is a compile error rather than a silent
 * overrun, and max_bytes_avail[][] is sized from it too.
 */
#define _NUM_FIFO_REGS 8

/*
 * One DMA transaction request passed from the CPU to the DWC3 core.
 *
 * This structure is described by the datasheet of DWC3 and shared between the hardware and
 * software driver. If the architecture involves cache, it must be flushed before accessing
 * this memory region.
 */
struct udc_dwc3_trb {
	uint32_t addr_lo;
	uint32_t addr_hi;
	uint32_t status;
	uint32_t ctrl;
} __packed __aligned(16);

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
	/*
	 * DMA-accessible scratch the control OUT data stage points its trailing
	 * alignment TRB at.  Never read: it exists only so the controller has
	 * somewhere to put the bytes it insists on being able to receive.
	 */
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
 * All data specific to one endpoint for use by the driver.
 */
struct udc_dwc3_ep_data {
	/* Allow to cast a pointer between ep_data and ep_cfg */
	struct udc_ep_config cfg;
	/* Endpoint number (physical address): the logical address is on ep_cfg */
	int epn;
	/* A work queue entry to process the buffers to submit on that endpoint */
	struct k_work work;
	/* To re-queue cancelled buffers after an endpoint is disabled */
	struct k_fifo requeue_fifo;
	/* Point back to the device for work queues */
	const struct device *dev;
	/*
	 * Record of the descriptor most recently armed on THIS endpoint.
	 *
	 * Per endpoint, not shared. It used to be two arrays hanging off the device
	 * state - one for IN, one for OUT - which meant every IN endpoint aliased the
	 * same slot and every OUT endpoint the other. The completion paths read it
	 * back to recover which control stage retired, so an alias is not a
	 * cosmetic problem: a completion on one endpoint can be classified from a
	 * descriptor armed on another. Adopted from upstream commit d69311448c5
	 * ("switch to per-ep workqueues, cache"), which made the same move.
	 *
	 * Two entries because a control IN data stage needing a zero-length
	 * terminator is armed as a chained pair.
	 */
	struct udc_dwc3_trb trb_cache[2];
	/* Buffer of pointers to net_buf, with index matching the position in the TRB buffers */
	struct net_buf *net_buf[CONFIG_UDC_DWC3_TRB_NUM];
	/* Buffer of TRB structures, with index matching the position in the net_buf buffers */
	struct udc_dwc3_trb *trb_buf;
	/* Index of the next TRB to receive data in the TRB ring, Link TRB excluded */
	uint32_t head;
	uint32_t tail;
	/* When the TRB ring buffer is full */
	bool full;
	/* Given by the hardware for use in endpoint commands */
	uint32_t xferrscidx;
	/*
	 * Set once a Start Transfer has actually reported an index for this
	 * endpoint, cleared by DEPSTARTCFG, which reassigns transfer resources and
	 * so invalidates every index handed out before it.
	 */
	bool xferrscidx_valid;
	/*
	 * Set when this endpoint's armed transfer has been invalidated but its
	 * completion event is still queued behind us. The event must be discarded
	 * rather than matched against whatever has since been armed - re-reading
	 * the ring cannot tell the two apart, because the re-arm has already
	 * overwritten the status the abort was visible in.
	 */
	bool stale_completion;
	/*
	 * Set when an End Transfer was issued with CmdIOC, cleared by the resulting
	 * Endpoint Command Complete event. While set, the controller may still be
	 * concluding bus traffic for the ended transfer even though CmdAct has
	 * cleared, so a new Start Transfer on this endpoint is premature.
	 *
	 * A Start Transfer is not issued while this is set: udc_dwc3_ep_resume()
	 * defers instead, and udc_dwc3_on_ep_cmd_cmplt() runs it when the event
	 * arrives. udc_dwc3_depcmd_start_xfer() warns if anything gets past that.
	 */
	bool end_xfer_pending;
	/*
	 * Set when udc_dwc3_ep_resume() was called while end_xfer_pending was still
	 * set on this endpoint, and the resume was therefore postponed. Cleared by
	 * udc_dwc3_on_ep_cmd_cmplt(), which then performs the resume.
	 */
	bool resume_pending;
	/*
	 * Arms and retires on this endpoint. The usbmon capture of
	 * uart_02sep_0139_racefix showed EP01 bulk OUT stop accepting host writes
	 * 465 s BEFORE control wedged, with nothing logged device-side at all -
	 * the only trace was the SRP loop slowing down, which had been dismissed
	 * as a flaky test. These two counters make that moment visible: when arms
	 * keep climbing and retires stop, the endpoint has stopped completing.
	 */
	uint32_t n_arm;
	uint32_t n_retire;
	/*
	 * Whether the postponed resume meant Init or Modify. Captured when the
	 * resume defers, because by the time it runs the stack has already set
	 * cfg.stat.enabled and the answer can no longer be read off it.
	 */
	bool resume_modify;
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
	/* Dispatch from IRQ events to workqueue jobs */
	struct k_work event_work;
	/* A work queue entry to test if the previous transaction is stuck */
	struct k_work_delayable watchdog_dwork;
	/* First endpoint to be configured */
	uint8_t first_ep;
#if CONFIG_UDC_DWC3_SHELL
	/* FIFO space initial values */
	uint16_t max_bytes_avail[_NUM_FIFO_SPACE][_NUM_FIFO_REGS];
#endif
	/* Next expected control transfer */
	//atomic_t expected_xfer;
	/* Updated whenever a packet is submitted */
	uint32_t last_xfer_type;
	uint8_t last_xfer_dir;
	/* Cached TRBs to recover from corrputed TRBs */
	/* Cache that is always up to date (before stack could get time to react) */
	struct usb_setup_packet setup_packet;
	/*
	 * Diagnostics for the event-buffer posted-write race. evt_late counts the
	 * events whose write had not landed on the first read; evt_gaveup counts
	 * the ones still not there when the wait ran out. Reported by "dwc3 evt"
	 * rather than logged per occurrence - see the note above the timeout.
	 */
	uint32_t evt_late;
	uint32_t evt_gaveup;
	uint32_t evt_handled;
	/*
	 * Instrumentation for telling apart the two remaining explanations of a
	 * stall: the write really did land later than the deadline, or the deadline
	 * expired while this thread was not running. Nothing here changes behaviour.
	 */
	uint32_t evt_gevntcount_hwm;	/* worst announced-but-unread backlog, bytes */
	uint32_t evt_gaveup_slot;	/* slot the current give-up run is stuck on */
	uint32_t evt_gaveup_run;		/* consecutive give-ups on that same slot */
	uint32_t evt_gaveup_t0;		/* cycle stamp when the run started */
	uint32_t evt_gaveup_us_max;	/* worst UNINSTRUMENTED fill latency, us */
	uint32_t evt_zero;		/* slots the CONTROLLER wrote as 0x00000000 */
	uint32_t evt_missed;		/* give-up runs presumed a LOST write */
	uint32_t evt_missed_frozen;	/* of those, with GEVNTCOUNT not moving */
	uint32_t evt_gaveup_gc0;		/* GEVNTCOUNT when this run opened */
	/*
	 * Control handshake trace. Every control transfer with a data stage runs
	 * SETUP-completes -> reported up -> stack enqueues the data buffer ->
	 * ctrl_try arms it. At a wedge all endpoints look healthy, so the useful
	 * question is which of those three steps did not happen. Stamps rather than
	 * per-transfer logging: control runs at ~64/s and a log line per step would
	 * cost more console time than the interval being measured.
	 */
	uint32_t ctrl_setup_up_t;	/* cycle stamp: SETUP handed to the stack */
	uint32_t ctrl_enq_t;		/* cycle stamp: stack enqueued on EP0 */
	uint32_t ctrl_armed_t;		/* cycle stamp: ctrl_try armed a stage */
	uint32_t ctrl_setup_up_n;
	uint32_t ctrl_enq_n;
	uint32_t ctrl_armed_n;
	uint8_t  ctrl_enq_last;		/* setup|data|status of the last EP0 enqueue */
	uint32_t evt_gaveup_gc0_max;	/* largest gc0 any run has opened with */
	uint32_t evt_gaveup_multi;	/* runs opened owed MORE than one event */
	bool evt_missed_counted;	/* this run already counted as missed */
	uint32_t evt_force_t0;		/* cycle stamp of the last forced command */
	uint32_t evt_worker_exit_t0;	/* cycle stamp when the worker last EXITED */
	uint32_t evt_kick;		/* heartbeat had to restart a stopped drain */
	/*
	 * Interrupts taken versus worker passes entered. These two answer a
	 * question the rest of the instrumentation cannot: when the drain is found
	 * idle with events already in the ring, was the driver never TOLD (no
	 * interrupt), or told and not SCHEDULED (work queue starved)?
	 *
	 * isr climbing while runs stalls = the submit happened and the queue did not
	 * service it. The system work queue is cooperative here
	 * (CONFIG_SYSTEM_WORKQUEUE_PRIORITY = -1) and the USB stack's own thread is
	 * cooperative at a HIGHER priority (K_PRIO_COOP(8)), so a busy usbd thread
	 * that does not yield can starve this queue outright.
	 *
	 * Both stalled together = no interrupt was delivered at all, even though
	 * GEVNTCOUNT was non-zero and the event was already in memory.
	 */
	uint32_t evt_isr;		/* interrupt handler invocations */
	uint32_t evt_worker_runs;	/* event worker passes entered */
	uint32_t evt_skipped;		/* events discarded to free a full ring */
	bool evt_worker_ran;		/* evt_worker_exit_t0 means something */
	bool evt_force_ever;		/* evt_force_t0 means something */
	bool evt_gaveup_quiet;		/* nothing was printed inside this run */
	uint32_t evt_link_total;	/* USB/Link State Change events seen */
	uint32_t evt_link_run;		/* consecutive events reporting the same state */
	uint32_t evt_link_last;		/* that state, EvtInfo[3:0] */
	/* Liveness, watched from the system work queue - see the heartbeat worker. */
	/*
	 * The heartbeat is driven by a PERIODIC KERNEL TIMER, not by a delayed
	 * work item that re-arms itself.
	 *
	 * Self-re-arming was the obvious way to write it and it has one fatal
	 * property: the only thing that schedules the next beat is the end of the
	 * previous one, so any handler that fails to complete stops the heartbeat
	 * permanently, and the last-resort recovery quietly ceases to exist. A
	 * backstop whose own liveness depends on the subsystem it is backstopping
	 * is not a backstop. k_timer is driven by the kernel clock and keeps firing
	 * regardless of what the work queue is doing, so the beat cannot be lost -
	 * and it resumes the instant a blocked queue frees up.
	 *
	 * The timer expiry runs in ISR context, so it does exactly one thing:
	 * submit the work item. Everything the heartbeat actually does - MMIO,
	 * synchronous logging, udc_dwc3_recover() - needs thread context and stays
	 * in the handler. Submitting an already-pending work item is a no-op, so a
	 * slow or blocked handler cannot make beats pile up.
	 */
	struct k_timer heartbeat_timer;
	struct k_work heartbeat_work;
	uint32_t dispatch_evt;		/* event being dispatched now, 0 = none */
	uint32_t dispatch_t0;		/* cycle stamp when that dispatch began */
	uint32_t hb_last_evt_handled;	/* evt_handled at the previous heartbeat */
	/* Worst late-but-arrived wait: polls is the lower bound, us the upper. */
	uint32_t evt_late_polls_max;
	uint32_t evt_late_us_max;
	/* Times the drain re-scheduled itself because an event raced the unmask. */
	uint32_t evt_rearm;
	/*
	 * Set only by udc_dwc3_recover(), only while it holds the UDC mutex, and
	 * cleared before it releases. Read only by udc_dwc3_wait_cmdact_zero().
	 * Single writer under the lock, so no atomicity concern.
	 */
	bool depcmd_no_sleep;
	/*
	 * Where the event ring is copied to before it is acknowledged. In priv and
	 * not on the stack because every work item here shares the system work
	 * queue's 1 KB stack, and this array stays live across udc_dwc3_handle_event()
	 * - the deepest call chain in the driver. Single-threaded and under the UDC
	 * mutex, so there is no reentrancy to worry about.
	 */
	uint32_t evt_copy[CONFIG_UDC_DWC3_EVENTS_NUM];
	/* Set when a drain pass stopped on an empty slot rather than finishing. */
	bool evt_drain_gaveup;
	bool evt_drain_midzero;		/* pass ended on a mid-pass empty slot */
	uint32_t evt_midzero;		/* how many passes ended that way */
	/*
	 * The stuck-control-claim detector.
	 *
	 * ctrl_decline counts every time udc_dwc3_ctrl_try() turned the stack away
	 * because a control endpoint was already claimed; ctrl_arm_t0 stamps the
	 * last time a claim was actually GRANTED. A decline is normal and
	 * transient - enumeration produces hundreds - so neither is a fault on its
	 * own. The fault is the pair: the stack still asking, and nothing granted
	 * for longer than any healthy control transfer could take.
	 */
	uint32_t ctrl_decline;		/* ctrl_try() turned the stack away */
	uint32_t ctrl_recover;		/* stuck claims broken by the heartbeat */
	uint32_t ctrl_arm_t0;		/* cycle stamp of the last granted claim */
	bool ctrl_decline_pending;	/* declined since the last grant */
	uint32_t hb_last_setup_done;	/* ctrl_setup_done at the previous beat */
	/*
	 * Beats in a row with events pending in the ring and nothing consumed.
	 * A healthy device - busy or idle - always reads GEVNTCOUNT 0 between
	 * beats, so a non-zero count that survives while evt_handled stands still
	 * means the drain has stopped making progress. This is what separates a
	 * wedge from an ordinary pause in control traffic, which looks identical
	 * on the claim age and decline count alone.
	 */
	uint32_t hb_last_handled;
	uint32_t hb_drain_stuck_beats;
	uint32_t ctrl_quiet_t0;		/* cycle stamp of the last SETUP retired */
	bool ctrl_quiet_logged;		/* this quiet period already reported */
	uint32_t ctrl_unarmed;		/* EP0-OUT found with no armed TRB */
	uint32_t ctrl_start_fail;	/* Start Transfer commands rejected */
	uint32_t ctrl_recovery_t0;	/* cycle stamp of the pending recovery */
	bool ctrl_unarmed_seen;		/* seen on the previous beat too */
	bool evt_gaveup_logged;		/* this run was reported, so report its recovery */
	/*
	 * NEVER SET - see udc_dwc3_recover().  This was meant to mean "recover()
	 * has issued an End Transfer on a control endpoint and is waiting for its
	 * Endpoint Command Complete event", but recover() issues Set Stall and no
	 * End Transfer, so nothing assigns this true and every path guarded by it
	 * is dead.  Kept because it is inert and removing it cascades; do not read
	 * it as evidence that an End-Transfer recovery exists.
	 * before re-arming. Nothing polls for that command - the event drives
	 * the second half of the recovery, and the watchdog is the fallback if
	 * it never arrives.
	 */
	bool ctrl_recovery_pending;
	/*
	 * The endpoint recovery ended, remembered so the re-arm cannot follow
	 * last_xfer_dir somewhere else. Between issuing the End Transfer and its
	 * completion arriving, the event handler may process an unrelated control
	 * completion and move last_xfer_dir, and udc_dwc3_ep_disable() may complete
	 * an End Transfer on the other control endpoint.
	 */
	struct udc_dwc3_ep_data *ctrl_recovery_ep;
	/*
	 * The control endpoint and stage the watchdog is guarding.
	 *
	 * Recovery used to infer both from last_xfer_dir / last_xfer_type, which
	 * name whatever was armed MOST RECENTLY. That was unambiguous while only
	 * one control TRB could be in flight; now that a SETUP can be armed on the
	 * OUT endpoint while a status stage is still outstanding on the IN one, the
	 * globals describe the SETUP and the watchdog is guarding the status stage.
	 * Recovering from them would End the healthy endpoint and re-arm the wrong
	 * stage.
	 */
	struct udc_dwc3_ep_data *watchdog_ep;
	uint32_t watchdog_type;
	/*
	 * How many control transfers the host abandoned by starting a new SETUP,
	 * and how many completions carried some other non-OK TRBSTS. Reported by
	 * "dwc3 evt". These answer the question the abandon path rests on: whether
	 * this controller reports SetupPending at all. If control stages keep
	 * deadlocking while this counter stays at zero, the detection is not firing
	 * and the trigger needs to come from somewhere else.
	 */
	uint32_t ctrl_setup_pending;
	/* SETUP watchdog: times the RXFIFOEMPTY gate suppressed it / let it fire. */
	uint32_t ctrl_setup_wd_idle;
	uint32_t ctrl_setup_wd_fire;
	/* ctrl_setup_done as of the last fire, to tell progress from a repeat. */
	uint32_t ctrl_setup_wd_mark;
	uint32_t ctrl_setup_wd_reset;
	/* OUT descriptors the caller sized to a non-multiple of MaxPacketSize. */
	uint32_t out_unaligned;
	uint32_t out_unaligned_ctrl;
	/*
	 * Closed-loop control state, DWC_usb3 3.30b sections 4.4.1 and 4.4.2.
	 *
	 * Every XferNotReady carries the stage the HOST is asking for. Without
	 * these two flags the driver can only compare it against what it armed
	 * last, which says nothing about whether the host is still on the same
	 * transfer - and both models list error cases that are defined purely in
	 * terms of the host being somewhere the device is not.
	 */
	bool ctrl_setup_seen;		/* SETUP retired, so setup_packet is current */
	bool ctrl_data_done;		/* the data stage of that request has retired */
	uint32_t ctrl_desync;		/* spec error cases caught and recovered */
	/*
	 * The two ends of a control transfer, counted independently.
	 *
	 * A request that runs to completion retires exactly one SETUP and exactly
	 * one status stage, so on a healthy link these two track each other with a
	 * difference of at most one (the request currently in flight). A gap that
	 * GROWS is the signature of the host starting requests it never finishes -
	 * a new SETUP arriving while the previous transfer is still outstanding.
	 * That is the question the -110 (ETIMEDOUT) windows in the host log raise
	 * and that no existing counter answers: ctrl_desync says a stage looked
	 * out of order, but not whether requests are being abandoned wholesale.
	 */
	uint32_t ctrl_setup_done;	/* SETUP stages retired */
	/*
	 * Retires on every non-control endpoint, used purely as a liveness
	 * proxy by the SETUP watchdog: the RxFIFO is shared by all OUT
	 * endpoints, so DSTS.RXFIFOEMPTY alone cannot say whose data is in it.
	 * If anything retired while the SETUP sat outstanding, the controller
	 * is moving traffic and the occupancy is not evidence of a wedge.
	 */
	uint32_t nonctrl_done;
	/*
	 * XferNotReady seen on a non-control endpoint. Counted as well as logged
	 * because the log line is rate limited, and the question this answers is
	 * whether the event fires at all - a rate limit that hides the only
	 * occurrence would answer it wrongly.
	 */
	uint32_t xnrdy_nonctrl;
	uint32_t ctrl_setup_wd_snap_setup;	/* ctrl_setup_done when armed */
	uint32_t ctrl_setup_wd_snap_nonctrl;	/* nonctrl_done when armed */
	uint32_t ctrl_setup_wd_busy;		/* suppressed: other traffic moving */
	uint32_t ctrl_setup_wd_retired;		/* suppressed: SETUP already retired */
	/*
	 * Update Transfer issued at a stuck SETUP to force the controller to
	 * re-cache the descriptor (databook 3.2.2.6). If the SETUP retires after
	 * one of these, the core was holding a stale HWO=0 for a TRB software
	 * had already armed, and the fault is descriptor visibility - not the
	 * controller refusing to work.
	 */
	uint32_t ctrl_setup_wd_updxfer;
	uint32_t ctrl_resync;		/* control endpoints resynchronised after a stall */
	uint32_t ctrl_recover_mark;	/* ctrl_setup_done at the last recover() */
	uint32_t ctrl_wd_dump;		/* non-SETUP watchdog fires dumped so far */
	uint32_t ctrl_wd_upd_mark;	/* stage count at the last control re-cache */
	uint32_t nonctrl_recache;	/* re-caches issued on non-control endpoints */
	uint32_t ctrl_setup_wd_upd_mark;
	uint32_t ctrl_status_done;	/* status stages retired (IN and OUT) */
	uint32_t ctrl_trbsts_other;
	/*
	 * How often udc_dwc3_ctrl_try() declined to arm because the endpoint's End
	 * Transfer had not reported completion. A handful per SetConfiguration is
	 * the expected shape; a count that climbs while control traffic is stalled
	 * means an Endpoint Command Complete went missing and the deferral has
	 * nothing to release it.
	 */
	uint32_t ctrl_deferred_arm;
	/*
	 * One bit per physical endpoint, set the first time this driver writes that
	 * endpoint's DEPCMD register.
	 *
	 * Until that write, the register must not be read. Databook section 1.3.12,
	 * DEPCMD[0:7]: "Several fields (including Command Type) are write-only, so
	 * their read values are undefined. After power-on, prior to issuing the
	 * first endpoint command, the read value of this register is undefined. In
	 * particular, the CmdAct bit may be set after power-on. In this case, it is
	 * safe to issue an endpoint command."
	 *
	 * Cleared on core soft reset, where the same reasoning applies again and
	 * where no command issued earlier can still be running.
	 */
	uint32_t depcmd_issued;
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
 * Runtime flags
 */
 enum {
	UDC_DWC3_CTRL_SETUP = 1,
	UDC_DWC3_CTRL_IN,
	UDC_DWC3_CTRL_OUT,
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

#if DT_HAS_COMPAT_STATUS_OKAY(snps_dwc3 /* <- replace with your more specific compatible */)
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
static int udc_dwc3_ep_disable(const struct device *const dev, struct udc_ep_config *const ep_cfg);
static int udc_dwc3_ep_resume(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data,
			      const bool modify);
/* Defined next to the control stage checks that are its other caller. */
static void udc_dwc3_fifo_flush_tx(const struct device *const dev, const uint8_t fifo);

/*
 * A control request is finished with: its status stage has been armed, so the
 * only thing still owed on it is a completion, never another XferNotReady.
 *
 * This is deliberately NOT done when a SETUP TRB is armed, which is the
 * tempting place for it. This driver arms the next SETUP speculatively, while
 * the previous request's status stage is still outstanding - that is what lets
 * the controller accept a new request instead of deadlocking the pair. During
 * that window setup_packet still describes a live request, and clearing these
 * flags on the SETUP arm would tell udc_dwc3_ctrl_xnr_check() that a transfer
 * the host is legitimately still finishing belongs to nobody - it would stall
 * a healthy transfer.
 *
 * Nor is it done when the status stage COMPLETES, which is the other tempting
 * place and is worse: it would depend on the SETUP of the next request
 * retiring after the status of this one. That is the order every capture shows,
 * but nothing guarantees it, and the one time it inverted this would clear the
 * flags belonging to the request that had just started and stall it. Arming the
 * status stage is a point this driver chooses, so the ordering is not in
 * question.
 */
static inline void udc_dwc3_ctrl_request_done(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	priv->ctrl_setup_seen = false;
	priv->ctrl_data_done = false;
}
#ifdef CONFIG_UDC_DWC3_SHELL
static void udc_dwc3_init_fifo_space(const struct device *dev);
#endif

#ifdef CONFIG_UDC_DWC3_SHELL
static void udc_dwc3_dump_trb(const struct device *dev, struct udc_dwc3_ep_data *ep_data,
			      const struct shell *sh);
#endif

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
 * Commands
 *
 * The DEPCMD register acts as a command interface, where a command number
 * is written along with parameters, an action is performed and a CMDACT bit
 * is reset whenever the command completes.
 */

/*
 * Wait for any endpoint command on this register to finish, i.e. for CmdAct to
 * read back as 0.
 *
 * Hybrid on purpose. Nearly every command completes within a few reads, so the
 * loop starts by spinning, paced one microsecond apart rather than run flat out
 * - the same reasoning as the event-slot wait, since these reads share a bus
 * with the controller's own traffic. Only once a command is clearly slow does
 * it fall back to sleeping, which frees the CPU instead of burning a whole
 * second of it. End Transfer is the command that can legitimately take a long
 * time: the controller "will wait until it can complete operations for the
 * endpoint before returning the Command Complete event" (section 4.3.11), so
 * the ceiling stays generous and a slow command is not mistaken for a failed
 * one.
 *
 * K_USEC() converts correctly whatever the tick rate is; it cannot give
 * sub-tick resolution, but that only changes how often an already-slow command
 * is re-checked, and the spin above covers everything fast.
 *
 * Returns true when CmdAct is clear, with the register value in *reg_out. The
 * caller can read the status field of that value: it belongs to the last
 * command issued on this endpoint.
 */
/* Ceiling on the CSftRst completion wait - see udc_dwc3_on_soft_reset(). */
#define UDC_DWC3_CSFTRST_POLL_US 10u
#define UDC_DWC3_CSFTRST_MAX_POLLS 10000u
#define UDC_DWC3_CMD_FAST_POLLS 32u
#define UDC_DWC3_CMD_FAST_POLL_US 1u
#define UDC_DWC3_CMD_SLOW_POLL_US 1000u
/*
 * Ceiling on the SLEEPING half of the command wait - and therefore on how long
 * the UDC mutex can be held by a thread that is not running.
 *
 * The fast half busy-waits UDC_DWC3_CMD_FAST_POLLS times and has satisfied
 * every command observed so far; the sleeping half exists only for a command
 * that is genuinely slow. It yields but KEEPS THE MUTEX, so its budget is a
 * direct bound on how long every other thread - including the one draining
 * events - can be blocked. A second was far too generous for that: no command
 * in the databook's control or endpoint set is specified to take anything
 * near it, and a caller that gives up simply reports failure, which every
 * caller already handles.
 */
#define UDC_DWC3_CMD_TIMEOUT_MS 100u

/* Defined with the other event-name decoders; used here for timeout diagnostics. */
static const char *udc_dwc3_get_devt_ulstchng_name(const uint32_t dsts);

/*
 * Physical endpoint number behind a DEPCMD register address, for the
 * priv->depcmd_issued bookkeeping. The addresses all come from
 * UDC_DWC3_DEPCMD(n), so this simply undoes that macro.
 */
static inline uint32_t udc_dwc3_depcmd_epn(const uint32_t addr)
{
	return (addr - UDC_DWC3_DEPCMD(0)) / 16u;
}

static bool udc_dwc3_wait_cmdact_zero(const struct device *const dev,
				      const uint32_t addr, uint32_t *const reg_out)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(UDC_DWC3_CMD_TIMEOUT_MS));
	uint32_t reg = 0;

	for (uint32_t i = 0; i < UDC_DWC3_CMD_FAST_POLLS; i++) {
		reg = sys_read32(base + addr);
		if ((reg & UDC_DWC3_DEPCMD_CMDACT) == 0) {
			*reg_out = reg;
			return true;
		}
		k_busy_wait(UDC_DWC3_CMD_FAST_POLL_US);
	}

	/*
	 * NEVER SLEEP HOLDING THE UDC MUTEX WHEN THE CALLER SAYS NOT TO.
	 *
	 * The loop below yields while keeping the mutex, for up to
	 * UDC_DWC3_CMD_TIMEOUT_MS. Any thread needing that mutex is stopped for the
	 * duration - including udc_dwc3_event_worker(), which is the only thing that
	 * drains the event ring. Blocking the drain for a second is how a recoverable
	 * fault becomes a full event-buffer overflow and a dead device.
	 *
	 * udc_dwc3_recover() therefore runs with depcmd_no_sleep set: it gets the
	 * fast, non-yielding phase above and nothing more. If the previous command is
	 * still active it simply does not issue this one, and the watchdog retries a
	 * second later - outside the lock, which is where waiting belongs.
	 *
	 * The sleeping path has never once been reached in five captures; the fast
	 * phase has always sufficed. It is kept for callers that are not on the
	 * recovery path, but it must never be reachable from one that is.
	 */
	if (priv->depcmd_no_sleep) {
		if (reg_out != NULL) {
			*reg_out = reg;
		}

		return false;
	}

	LOG_WRN_RATELIMIT("cmdact still set on 0x%x after the fast poll; sleeping "
			  "with the UDC mutex held", addr);

	while (!sys_timepoint_expired(end)) {
		k_sleep(K_USEC(UDC_DWC3_CMD_SLOW_POLL_US));

		reg = sys_read32(base + addr);
		if ((reg & UDC_DWC3_DEPCMD_CMDACT) == 0) {
			*reg_out = reg;
			return true;
		}
	}

	*reg_out = reg;
	return false;
}

/*
 * Issue an endpoint command.
 *
 * Only Start Transfer is waited on after posting, because it is the only
 * command whose result this driver uses - it returns the transfer resource
 * index. Every other command's return value is discarded by its caller, so
 * waiting for it would block for no benefit; the wait for those happens
 * instead in the pre-poll above, paid by whoever issues the NEXT command on
 * that endpoint, which is usually much later and therefore usually free.
 *
 * That matters most for End Transfer, which is both the slowest command and one
 * whose result nobody reads. It is issued with CmdIOC set, so its completion
 * arrives as an Endpoint Command Complete event rather than being polled for.
 *
 * The return value is meaningful only for Start Transfer. For every other
 * command it is zero and the caller ignores it.
 */


/*
 * Make a descriptor visible to the controller before the command that fetches it.
 *
 * Call with the word written LAST for that descriptor, immediately before Start
 * Transfer or Update Transfer. Two things happen and both are needed:
 *
 *   read-back - a load cannot be answered until the write ahead of it to the
 *               same location has landed, so this pushes the posted write out.
 *               It must be the last word: the earlier fields were written
 *               before it, so draining it drains them too.
 *
 *   fence     - stops the command-register write from overtaking. It has to be
 *               "iorw,iorw": r/w order MEMORY only, and the command register is
 *               device I/O, so "fence rw,rw" leaves exactly the reordering this
 *               exists to prevent.
 *
 * Without this the controller can fetch a descriptor before the arming write is
 * visible to it, cache HWO=0 for a TRB software has already armed, and then sit
 * on a received packet with nowhere to put it - while the CPU, reading its own
 * writes, sees the descriptor correctly armed.
 */
static inline void udc_dwc3_trb_sync(volatile uint32_t *const last_word)
{
	/*
	 * The read is NOT dead code and the compiler may not remove it: the
	 * pointer is volatile, and an access through a volatile lvalue has to be
	 * performed exactly as written. The (void) cast only silences the
	 * unused-variable warning - it contributes nothing to keeping the load.
	 *
	 * What would silently break this is dropping volatile from trb_buf or
	 * from this parameter. Then the load becomes removable, it disappears at
	 * -Os, and the ordering is gone with no warning anywhere. Verified
	 * present in trb_ctrl_out, trb_ctrl_in and trb_bulk at -Os.
	 */
	uint32_t readback = *last_word;

	(void)readback;

#if defined(CONFIG_RISCV)
	__asm__ volatile ("fence iorw,iorw" ::: "memory");
#else
	barrier_dsync_fence_full();
#endif
}

/*
 * Fill a TRB and make it visible to the controller. The ONLY place a TRB's
 * words are written.
 *
 * Every descriptor goes through here - control SETUP, DATA, STATUS_2 and
 * STATUS_3, bulk, interrupt, and the link descriptor - so no path can be
 * written later that fills a TRB and forgets the ordering. The caller issues
 * its own Start Transfer or Update Transfer immediately afterwards; by then the
 * descriptor is guaranteed visible.
 *
 * The ownership word goes LAST and is what udc_dwc3_trb_sync() reads back,
 * because the controller must not be able to see HWO set while the address or
 * length it refers to are still in flight.
 */
static inline void udc_dwc3_trb_fill(volatile struct udc_dwc3_trb *const trb,
				     const uintptr_t addr, const uint32_t status,
				     const uint32_t ctrl)
{
	trb->addr_lo = LO32(addr);
	trb->addr_hi = HI32(addr);
	trb->status = status;
	trb->ctrl = ctrl;

	udc_dwc3_trb_sync(&trb->ctrl);
}

static uint32_t udc_dwc3_depcmd(const struct device *const dev,
				const uint32_t addr, const uint32_t cmd)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t epn = udc_dwc3_depcmd_epn(addr);
	const bool first_on_ep = (epn >= 32u) ||
				 ((priv->depcmd_issued & BIT(epn)) == 0);
	const bool needs_result =
		(cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK) == UDC_DWC3_DEPCMD_DEPSTRTXFER;
	uint32_t phycfg_saved;
	uint32_t reg = 0;

	/*
	 * A command must not be issued while the previous one on this endpoint is
	 * still active. Waiting here rather than after posting is what keeps the
	 * slow commands off the caller's critical path.
	 *
	 * Not before this driver has written the register at least once, though.
	 * Databook section 1.3.12 says the read value of DEPCMD is undefined until
	 * the first endpoint command is issued on it, that CmdAct in particular may
	 * come up set, and that issuing a command anyway is safe. There is nothing
	 * to wait for either: no command has been issued on this endpoint, so none
	 * can be active.
	 *
	 * Reading it regardless is self-perpetuating rather than merely wrong. The
	 * wait cannot succeed, so the command is never issued, so the register is
	 * never written, so it stays undefined - and the endpoint is locked out for
	 * the lifetime of the boot. That is what happened to ep 0x80 in uart_v4.log:
	 * its very first DEPCFG was refused and all 85 command attempts that
	 * followed, on that one endpoint, were refused for the same reason. EP0-OUT
	 * escaped only because DEPSTARTCFG is issued on DEPCMD(0) first, which makes
	 * that register defined before anything reads it.
	 */
	if (!first_on_ep && !udc_dwc3_wait_cmdact_zero(dev, addr, &reg)) {
		/*
		 * GEVNTCOUNT is reported alongside, because it distinguishes the two
		 * reasons a command can fail to finish. A large count means events
		 * were waiting to be consumed while the controller was trying to
		 * complete this command, which is the deadlock the databook warns
		 * about. A count of zero means the command is stuck for some other
		 * reason and the event path is not implicated.
		 */
		LOG_ERR("previous command still active on addr 0x%x (0x%08x) after %u ms, "
			"not issuing command 0x%x, GEVNTCOUNT=%u bytes, DSTS=0x%08x (%s)",
			addr, reg, UDC_DWC3_CMD_TIMEOUT_MS, cmd,
			udc_dwc3_gevntcount(base),
			sys_read32(base + UDC_DWC3_DSTS),
			udc_dwc3_get_devt_ulstchng_name(sys_read32(base + UDC_DWC3_DSTS)));
		return UDC_DWC3_XFERRSCIDX_INVALID;
	}

	/*
	 * The status field still describes the command that just finished, which is
	 * the only opportunity to notice that one failed now that most commands are
	 * not waited on individually. Reported one command late, which beats not at
	 * all.
	 *
	 * Skipped on the first command of an endpoint, where reg was never read and
	 * the register it would have been read from is undefined anyway.
	 */
	if (!first_on_ep &&
	    (reg & UDC_DWC3_DEPCMD_STATUS_MASK) == UDC_DWC3_DEPCMD_STATUS_CMDERR) {
		LOG_ERR("previous endpoint command on addr 0x%x reported an error "
			"(0x%08x)", addr, reg);
	}

	/*
	 * At USB 2.0 speeds the USB2 PHY must not be suspended and L1 sleep must not
	 * be enabled while an endpoint command executes: "if GUSB2PHYCFG[6] or
	 * GUSB2PHYCFG[8] is set to '1', it must be set to '0' prior to issuing this
	 * command". The databook repeats this on every endpoint command. Leaving
	 * them set makes commands take far longer or fail outright.
	 *
	 * Save and restore rather than clear once at init, because these bits belong
	 * to power management and may be turned on from outside this driver. When
	 * both are already clear - which they appear to be on this core - this costs
	 * one register read and no writes.
	 */
	reg = sys_read32(base + UDC_DWC3_GUSB2PHYCFG);
	phycfg_saved = reg & (UDC_DWC3_GUSB2PHYCFG_SUSPHY |
			      UDC_DWC3_GUSB2PHYCFG_ENBLSLPM);
	if (phycfg_saved != 0) {
		sys_write32(reg & ~phycfg_saved, base + UDC_DWC3_GUSB2PHYCFG);
	}

	/*
	 * Every endpoint command on every endpoint - control, bulk and interrupt
	 * alike - is issued from here, so this is the one place that can promise
	 * the ordering the controller depends on: any descriptor this command
	 * tells it to fetch must be visible before the command is.
	 *
	 * Start Transfer and Update Transfer are the two that matter, but the
	 * fence is unconditional. Placing it per-path is what let the control
	 * SETUP path be armed with only a compiler barrier behind it; a single
	 * fence here cannot be missed by a path added later, and costs one
	 * instruction on a path that already does an MMIO write.
	 */
	sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);

	/*
	 * From here the register has been written, so its read value is defined and
	 * the pre-poll above may use it on the next command for this endpoint.
	 */
	if (epn < 32u) {
		priv->depcmd_issued |= BIT(epn);
	}

	if (!needs_result && phycfg_saved == 0) {
		/*
		 * Nothing to collect and nothing to put back, so the command is left
		 * running and this returns. Whoever issues the next command on this
		 * endpoint waits for it in the pre-poll above.
		 */
		return 0;
	}

	/*
	 * Two reasons to wait here. Start Transfer, because its transfer resource
	 * index is the one result this driver consumes. And any command at all when
	 * the PHY bits had to be cleared: those must stay clear while the command
	 * executes, not merely at the instant it is issued, so putting them back
	 * early could stall the very command that required clearing them. On this
	 * core they are already clear, so that second case never arises.
	 */

	if (!udc_dwc3_wait_cmdact_zero(dev, addr, &reg)) {
		LOG_ERR("command expired: 0x%x on addr 0x%x after %u ms, "
			"GEVNTCOUNT=%u bytes, DSTS=0x%08x (%s)",
			cmd, addr, UDC_DWC3_CMD_TIMEOUT_MS,
			udc_dwc3_gevntcount(base),
			sys_read32(base + UDC_DWC3_DSTS),
			udc_dwc3_get_devt_ulstchng_name(sys_read32(base + UDC_DWC3_DSTS)));
		reg = UDC_DWC3_DEPCMD_CMDACT;
	}

	if (phycfg_saved != 0) {
		/* Re-read: the command may have changed other fields of this register. */
		sys_set_bits(base + UDC_DWC3_GUSB2PHYCFG, phycfg_saved);
	}

	if (!needs_result) {
		return 0;
	}

	/*
	 * Only report a transfer resource index when the command actually
	 * succeeded. On timeout the status field still reads as OK - the command is
	 * simply still active - so it is caught by the CmdAct bit left set above.
	 *
	 * Returning the field regardless would let one failed command overwrite a
	 * good index with a value the controller never assigned, and every later
	 * Update Transfer and End Transfer on that endpoint would address the wrong
	 * transfer resource, silently and permanently.
	 */
	if ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0) {
		return UDC_DWC3_XFERRSCIDX_INVALID;
	}

	switch (reg & UDC_DWC3_DEPCMD_STATUS_MASK) {
	case UDC_DWC3_DEPCMD_STATUS_OK:
		break;
	case UDC_DWC3_DEPCMD_STATUS_CMDERR:
		LOG_ERR("endpoint command 0x%x, addr 0x%x failed (0x%08x)", cmd, addr, reg);
		return UDC_DWC3_XFERRSCIDX_INVALID;
	default:
		LOG_ERR("command failed with unknown status: 0x%08x", reg);
		return UDC_DWC3_XFERRSCIDX_INVALID;
	}

	return FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
}

static void udc_dwc3_depcmd_ep_config(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data,
				      const bool modify)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t param0 = 0;
	uint32_t param1 = 0;

	LOG_INF("Configuring endpoint 0x%02x with wMaxPacketSize=%u",
		ep_data->cfg.addr, ep_data->cfg.mps);

	/*
	 * Init or Modify is passed in rather than inferred from cfg.stat.enabled.
	 * That flag is owned by the stack and set in udc_ep_enable_internal() as
	 * soon as ep_enable() RETURNS, so once udc_dwc3_ep_resume() is allowed to
	 * postpone itself, a first-time configuration that ran late would read the
	 * flag as already true and issue Modify against a configuration that had
	 * never been initialised. The caller knows which it means; this function
	 * should not guess.
	 */
	if (modify) {
		LOG_DBG("UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MODIFY");
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MODIFY;
	} else {
		LOG_DBG("UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_INIT");
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_INIT;
	}

	switch (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_CONTROL:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL;
		break;
	case USB_EP_TYPE_BULK:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_BULK;
		break;
	case USB_EP_TYPE_INTERRUPT:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_INT;
		break;
	case USB_EP_TYPE_ISO:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_ISOC;
		break;
	default:
		CODE_UNREACHABLE;
	}

	/* Max Packet Size according to the USB descriptor configuration */
	param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_MPS_MASK, ep_data->cfg.mps);

	/*
	 * Burst Size is "number of packets per burst minus one".
	 *
	 * Control endpoints do not burst and Table 4-1 programs BrstSiz = 0 for them, so
	 * 0 is forced for endpoint 0.  At SuperSpeed this field drives ACK NumP flow
	 * control on IN bursts, so an over-stated value there is not cosmetic.
	 *
	 * Non-control endpoints keep 15, and that is correct rather than unexamined.  For
	 * IN the field is a CEILING: "If BrstSiz >= the NumP value in the initiating
	 * TP_ACK, then the device controller attempts a burst length of NumP" - NumP comes
	 * from the host, bounded by the descriptor's bMaxBurst, so an over-stated BrstSiz
	 * cannot produce a burst the host did not ask for, while lowering it would cap the
	 * video endpoint.  bMaxBurst is not available here in any case:
	 * udc_ep_enable_internal() takes (addr, attributes, mps, interval) and the UDC API
	 * carries no burst field.
	 *
	 * For a non-control OUT endpoint it IS an assertion - the credit this device
	 * advertises.  15 is left because the host is still bounded by the descriptor and
	 * 0 is explicitly worse (NumP=0 forces a flow-control ERDY per DP).  Revisit only
	 * with evidence.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 0);
	} else {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 15);
	}

	/* Set the FIFO number, must be 0 for all OUT EPs */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK,
				     ep_data->cfg.addr & 0x7f);
	}

	/* Per-endpoint events */
	param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
	param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN;

	/*
	 * XferNotReady is mandatory on the control endpoints: "The XferNotReady
	 * event must not be disabled for control endpoints because the event is an
	 * integral part of control transfer handling" (section 4.2.4).
	 *
	 * It is the controller telling us the host has asked for a stage that has
	 * no TRB armed for it, and it is the only on-demand trigger this driver has
	 * for arming the next one. With it disabled, a single completion that never
	 * arrives leaves the control endpoint with nothing to restart it: the
	 * transfer stays wedged until the recovery watchdog fires, which issues a
	 * Set Stall on EP0-OUT (not an End Transfer - see udc_dwc3_recover()).
	 * That converts a transient miss into a stall
	 * that only a timeout can clear.
	 *
	 * This was previously disabled to work around duplicated XferNotReady
	 * events. A duplicate is now absorbed rather than acted on twice: both
	 * handlers end in udc_dwc3_ctrl_next(), which reaches udc_dwc3_ctrl_try(),
	 * and that refuses to arm a data or status stage while the control pair is
	 * busy. The second event of a pair logs and does nothing. A SETUP is the
	 * one exception, and is checked against its own endpoint only.
	 *
	 * Only the two request stages are reported. The event status field is
	 * "2'b01: Control Data Request" or "2'b10: Control Status Request" in bits
	 * [13:12]; there is no SETUP encoding, which is why the handlers treat that
	 * value as invalid rather than as a stage to arm.
	 */
	/*
	 * XferNotReady on EP0 only.
	 *
	 * It was briefly enabled on every endpoint to answer "does a bulk OUT ever
	 * lack an armed TRB when the host sends?".  Measured answer: no - 107 events
	 * over two runs, every one on EP85 (RTL-owned), none on the CDC bulk OUT
	 * endpoints, which were armed at every stall.  The question is settled, and
	 * the events are not free: peak GEVNTCOUNT went 20 B -> 28 B of a 64 B ring
	 * with it on, on a ring whose write latency is the fault under investigation.
	 * Off again.
	 */
	/*
	 * EP0 only, and a non-control OUT endpoint genuinely does not need it.
	 *
	 * XferNRdyEn (DEPCMDPAR1 bit 10) masks the EVENT, not the protocol: with
	 * it clear the controller still answers NRDY when an OUT endpoint has no
	 * hardware-owned TRB, and still sends ERDY once software arms one. The
	 * handshake closes itself - the class posts a read when it wants data,
	 * arming transmits the ERDY, the host retries. The event would only tell
	 * software something it does not act on, and it is not free: enabled on
	 * every endpoint it put 107 events into a 64-byte ring, all of them from
	 * the video IN endpoint, and took peak GEVNTCOUNT from 20 to 28 bytes.
	 *
	 * Control is the exception the databook names, and the reason is specific:
	 * the driver cannot know WHICH stage the host is asking for without the
	 * event, because that is carried in its status field, bits [13:12]
	 * (Control Data Request / Control Status Request).
	 *
	 * The 4.2.4 on-demand deadlock note does not apply here - it is
	 * conditioned on the RX packet threshold feature, and GRXTHRCFG is 0.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
	}

	/* This is the usb protocol endpoint number, but the data encoding
	 * we chose for physical endpoint number is the same as this register
	 */
	param1 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK, ep_data->epn);

	/*
	 * bInterval_m1 - the endpoint service interval. The field description is
	 * "set to the bInterval value minus 1. The valid values for this field are
	 * 0 through 13. The bInterval value is reported in the endpoint descriptor.
	 * When the controller is operating in Full-Speed mode, this field must be
	 * set to 0." Section 4.3.3 makes it mandatory for isochronous endpoints,
	 * and it carries the same meaning for interrupt ones. Bulk and control
	 * endpoints have no service interval and leave the field at 0.
	 *
	 * The mask has been defined here since the driver was written but was never
	 * used, so every periodic endpoint was configured with an interval of one
	 * microframe regardless of what its descriptor asked for.
	 *
	 * Unlike BrstSiz above, this one can be programmed exactly rather than
	 * guessed: udc_ep_enable_internal() records the descriptor's bInterval in
	 * cfg->interval (udc_common.c), so the value the host was told is the value
	 * written here.
	 */
	switch (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_ISO:
	case USB_EP_TYPE_INTERRUPT: {
		uint32_t binterval_m1 = 0;

		if ((sys_read32(base + UDC_DWC3_DSTS) & UDC_DWC3_DSTS_CONNECTSPD_MASK) !=
		    UDC_DWC3_DSTS_CONNECTSPD_FS) {
			binterval_m1 = (ep_data->cfg.interval > 0U) ?
				       (uint32_t)ep_data->cfg.interval - 1U : 0U;

			if (binterval_m1 > 13U) {
				LOG_WRN("EP%02x bInterval %u is outside the encodable "
					"range, clamping bInterval_m1 to 13",
					ep_data->cfg.addr, ep_data->cfg.interval);
				binterval_m1 = 13U;
			}
		}

		LOG_DBG("EP%02x bInterval %u -> bInterval_m1 %u",
			ep_data->cfg.addr, ep_data->cfg.interval, binterval_m1);

		param1 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR1_DEPCFG_BINTERVAL_MASK,
				     binterval_m1);
		break;
	}
	default:
		break;
	}

	sys_write32(param0, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(param1, base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPCFG);
}

static void udc_dwc3_depcmd_ep_xfer_config(const struct device *const dev,
					   struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	LOG_DBG("DepXferConfig: EP%02x", ep_data->cfg.addr);

	reg = FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK, 1);
	sys_write32(reg, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPXFERCFG);
}

static void udc_dwc3_depcmd_set_stall(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	//struct udc_dwc3_data *const priv = udc_get_private(dev);

	LOG_DBG("DepSetStall: EP%02x", ep_data->cfg.addr);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSETSTALL);

	//atomic_set(&priv->expected_xfer, BIT(UDC_DWC3_CTRL_SETUP));
}

static void udc_dwc3_depcmd_clear_stall(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data,
					uint32_t flags)
{
	LOG_DBG("DepClearStall EP%02x", ep_data->cfg.addr);

	flags |= UDC_DWC3_DEPCMD_DEPCSTALL;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);
}

/* Defined below; needed here to release a transfer resource a rejected Start
 * Transfer could not obtain.
 */
static bool udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags);

/*
 * Returns true when the transfer is running.
 *
 * It used to return void, so a failed Start Transfer was logged and counted but
 * invisible to the caller: udc_dwc3_trb_nonctrl_init() carried on and
 * udc_dwc3_ep_resume() then set DALEPENA, leaving an endpoint enabled with no
 * transfer running and nothing to retry it.  Control callers still ignore the
 * result - the control watchdog is their recovery - but the non-control path
 * now propagates it out through ep_resume().
 */
static bool udc_dwc3_depcmd_start_xfer(const struct device *const dev,
				       struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t idx;
	uint32_t reg;

	/*
	 * Last line of defence for databook 3.2.2.7. Nothing should arrive here
	 * with an End Transfer still concluding on this endpoint: the non-control
	 * resume postpones itself in udc_dwc3_ep_resume(), and the control re-arm
	 * runs from udc_dwc3_on_ep_cmd_cmplt() after the flag has been cleared.
	 *
	 * A warning rather than a refusal. Declining to start would strand the
	 * endpoint if a completion never came, which is the controller-side fault
	 * this driver is chasing; the point is to make a premature Start Transfer
	 * visible rather than to make it impossible.
	 */
	if (ep_data->end_xfer_pending) {
		LOG_WRN("Start Transfer on EP%02x while its End Transfer has not "
			"reported completion", ep_data->cfg.addr);
	}

	/*
	 * The controller fetches the descriptor as soon as it sees this command,
	 * so the descriptor has to be in memory before the command reaches it.
	 */

	/* Make sure the device is in U0 state, assuming TX FIFO is empty */
	/*
	 * Bring the link back to U0 if, and only if, it is not already there.
	 *
	 * This used to be unconditional on every Start Transfer. Three things were
	 * wrong with that. The value written is 8, which the databook calls Resume
	 * - "the software must write Resume (8) into the DCTL.ULStChngReq field" -
	 * not a benign no-op when the link is already up. The field was never
	 * written back to 0, and the databook requires that: "if software wants to
	 * issue the same request back-to-back, it must write a 0 to this field
	 * between the two requests", and "if software is updating other fields of
	 * the DCTL register and not intending to force any link state change, then
	 * it must write a 0 to this field" - which every sys_set_bits() on
	 * DCTL.RunStop does, re-asserting the request each time. And the link state
	 * was never read, though DSTS reports it directly.
	 *
	 * The field also self-clears when the MAC exits suspend, so under load each
	 * subsequent Start Transfer re-issued it as a genuinely new request.
	 *
	 * This matters most for End Transfer on an IN endpoint, which has to
	 * transmit a DPPABORT ordered set on the wire and therefore needs the link
	 * in U0; an OUT endpoint only drains its FIFO to memory and does not.
	 */
	reg = sys_read32(base + UDC_DWC3_DSTS);
	if ((reg & UDC_DWC3_DSTS_CONNECTSPD_MASK) == UDC_DWC3_DSTS_CONNECTSPD_SS &&
	    (reg & UDC_DWC3_DSTS_USBLNKST_MASK) != UDC_DWC3_DSTS_USBLNKST_USB3_U0) {
		LOG_DBG("link not in U0 (%s), requesting resume before Start Transfer",
			udc_dwc3_get_devt_ulstchng_name(reg));

		reg = sys_read32(base + UDC_DWC3_DCTL);
		reg &= ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;
		reg |= UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP;
		sys_write32(reg, base + UDC_DWC3_DCTL);

		/*
		 * Return the field to 0 so the next request is seen as a change, and
		 * so that any later read-modify-write of DCTL does not re-issue this
		 * one.
		 */
		reg &= ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;
		sys_write32(reg, base + UDC_DWC3_DCTL);
	}

	sys_write32(HI32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	idx = udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSTRTXFER);

	/*
	 * Keep the previous index when the command failed. The controller only
	 * assigns a transfer resource on success, so storing what the register held
	 * after a failure would replace a working index with a value that addresses
	 * some other endpoint's resource - or none - for every Update Transfer and
	 * End Transfer that follows.
	 */
	if (idx == UDC_DWC3_XFERRSCIDX_INVALID) {
		struct udc_dwc3_data *const priv = udc_get_private(dev);

		priv->ctrl_start_fail++;
		LOG_ERR("Start Transfer failed on EP%02x, keeping transfer resource "
			"index 0x%x%s (%u so far)", ep_data->cfg.addr,
			ep_data->xferrscidx,
			ep_data->xferrscidx_valid ? "" : " (never established)",
			priv->ctrl_start_fail);

		/*
		 * The arm DID NOT TAKE EFFECT, and returning quietly here is what
		 * turns that into a dead device. The caller has already claimed the
		 * endpoint and goes on believing a stage is armed, so the claim is
		 * never given back and udc_dwc3_ctrl_try() declines every later
		 * request - silently, because those declines are LOG_DBG.
		 * uart_v6_8 caught exactly that: busy o/i 1/0 with 72 declines, a
		 * buffer queued and ready, and the device dead after 22 SETUPs.
		 *
		 * CmdStatus 4'h1 on a Start Transfer means "there is no transfer
		 * resource available on the endpoint", and 3.2.2.2 says how to get
		 * one back: "Start Transfer causes the use of the transfer
		 * resource. End Transfer or an XferComplete event releases the
		 * transfer resource."
		 *
		 * NOT by End Transfer here, though 3.2.2.2 offers it: on a CONTROL
		 * endpoint End Transfer hangs this controller. uart_v6_9 caught it
		 * doing so - DEPCMD 0x00000d08 on EP0, CMDTYP 8 with CMDACT still
		 * set after 1000 ms, sixty times over, while DSTS showed the frame
		 * counter still advancing in U0. The command engine was wedged, not
		 * the link. Releasing the resource is left to the host's next SETUP,
		 * which retires the outstanding descriptor with SetupPending and so
		 * produces the XferComplete that 3.2.2.2 names as the other way a
		 * transfer resource is freed.
		 *
		 * Then drop the claim. Whatever else is true, this endpoint is not
		 * armed, and pretending otherwise is what wedged it.
		 */
		udc_ep_set_busy(&ep_data->cfg, false);
		return false;
	}

	ep_data->xferrscidx = idx;
	ep_data->xferrscidx_valid = true;

	LOG_DBG("start EP%02x idx=0x%x",
		ep_data->cfg.addr, ep_data->xferrscidx);

	return true;
}

static void udc_dwc3_depcmd_update_xfer(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	uint32_t flags = 0;

	/*
	 * Warn, but still issue the command. A non-control endpoint receives exactly
	 * one Start Transfer, from udc_dwc3_trb_nonctrl_init(), and every buffer
	 * after that depends on Update Transfer - so refusing here because that one
	 * command failed would take the whole data path down rather than one
	 * transfer. The command will fail on its own if the resource is genuinely
	 * not there, and that failure is logged by udc_dwc3_depcmd().
	 */
	if (!ep_data->xferrscidx_valid) {
		LOG_WRN("Update Transfer on EP%02x with no established transfer "
			"resource index, proceeding with 0x%x",
			ep_data->cfg.addr, ep_data->xferrscidx);
	}

	/*
	 * Same ordering requirement as Start Transfer, and this is the path that
	 * runs per buffer on every bulk and interrupt endpoint - the hot one.
	 */
	flags |= UDC_DWC3_DEPCMD_DEPUPDXFER;
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	/* DBG: this fires once per buffer from udc_dwc3_trb_bulk(). */
	LOG_DBG("DepUpdateXfer done EP%02x, addr 0x%08x, data 0x%08x, xferrscidx 0x%x",
		ep_data->cfg.addr, UDC_DWC3_DEPCMD(ep_data->epn), flags, ep_data->xferrscidx);
}

/*
 * Issue End Transfer. Returns true when the command was issued AND will report
 * an Endpoint Command Complete event, which is what a caller needs to know
 * before deciding to wait for one.
 */
static bool udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	if (!ep_data->xferrscidx_valid) {
		LOG_WRN("End Transfer on EP%02x with no established transfer "
			"resource index, proceeding with 0x%x",
			ep_data->cfg.addr, ep_data->xferrscidx);
	}

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
	flags |= UDC_DWC3_DEPCMD_DEPENDXFER;

	/*
	 * Ask for a completion event so the conclusion of bus traffic for this
	 * transfer is observable - see UDC_DWC3_DEPCMD_CMDIOC.
	 *
	 * Only while the controller is running: the field must not be set when
	 * DCTL.RunStop is 0, which is reachable because teardown can end a transfer
	 * after the controller has been stopped. With the controller stopped no
	 * completion event would arrive anyway, so the request is dropped and the
	 * endpoint left un-armed. The command itself still runs either way.
	 *
	 * Armed before the command is posted, never after: the event can only be
	 * raised once the command is in flight, but the work queue that consumes it
	 * may run the moment this function returns.
	 */
	if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0) {
		flags |= UDC_DWC3_DEPCMD_CMDIOC;
		ep_data->end_xfer_pending = true;
	}

	/*
	 * A failed return means the command was never issued - the previous one on
	 * this endpoint was still active when the pre-poll gave up. Clearing the
	 * flag matters more than the log line: a caller waiting for a completion
	 * event that no command will ever generate waits for ever.
	 */
	if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags) ==
	    UDC_DWC3_XFERRSCIDX_INVALID) {
		ep_data->end_xfer_pending = false;
		LOG_ERR("End Transfer not issued on EP%02x", ep_data->cfg.addr);
		return false;
	}

	LOG_DBG("DepEndXfer done EP%02x", ep_data->cfg.addr);

	/*
	 * True only when a completion event is genuinely expected. Without CmdIOC -
	 * which is refused while the controller is stopped - the command still runs
	 * but reports nothing, so a caller must not wait for it.
	 */
	return (flags & UDC_DWC3_DEPCMD_CMDIOC) != 0;
}

static void udc_dwc3_depcmd_start_config(const struct device *const dev,
					 bool is_control)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	uint32_t flags = 0;

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, is_control ? 0 : 2);
	flags |= UDC_DWC3_DEPCMD_DEPSTARTCFG;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(0), flags);

	/*
	 * DEPSTARTCFG reassigns the controller's transfer resources, so every index
	 * handed out by an earlier Start Transfer stops being meaningful. Drop them
	 * all, so a stale one cannot be reused before its endpoint has been started
	 * again.
	 */
	for (uint8_t i = 0; i < cfg->num_in_eps; i++) {
		cfg->ep_data_in[i].xferrscidx_valid = false;
	}
	for (uint8_t i = 0; i < cfg->num_out_eps; i++) {
		cfg->ep_data_out[i].xferrscidx_valid = false;
	}

	LOG_DBG("DepStartConfig done ep=%s", is_control ? "control" : "non-control");
}

/*
 * Transfer Requests (TRB)
 *
 * DWC3 receives transfer requests from this driver through a shared memory
 * buffer, resubmitted upon every new transfer (through either Start or
 * Update command).
 */



/*
 * Report an OUT descriptor whose size is not a whole number of packets.
 *
 * Checked at the point the size is decided, BEFORE any padding is applied - the
 * padding would otherwise make every descriptor look compliant and the counter
 * would read zero whether or not callers are actually getting this right.  What
 * is being measured is the size the CALLER supplied, which is the thing we cannot
 * see from here otherwise.
 *
 * The two databook exceptions are not routed here: a Setup TRB must carry exactly
 * 8 (S3.1.2.2) and a status TRB carries 0, which passes the modulo anyway.
 */
static void udc_dwc3_out_size_check(const struct device *const dev,
				    const struct udc_dwc3_ep_data *const ep_data,
				    const uint32_t in_size, const uint32_t trb_size,
				    const char *const what, const bool is_setup)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);

	/*
	 * Silent when the caller already supplied a whole number of packets, which
	 * is the normal case: udc_ctrl_data_alloc() rounds every control OUT buffer
	 * up to bMaxPacketSize0 before the driver ever sees it.  Printing those would
	 * bury the one case worth seeing.
	 *
	 * The test is on trb_size, the size BEFORE correction.  Testing the padded
	 * total instead would be self-defeating - padding always makes it a multiple,
	 * so a misaligned caller would never be reported at all.
	 */
	if (is_setup) {
		if (trb_size == sizeof(struct usb_setup_packet)) {
			return;
		}

		priv->out_unaligned++;
		priv->out_unaligned_ctrl++;
		LOG_ERR_RATELIMIT("EP%02x Setup OUT descriptor is %u B, not the 8 B "
				  "S3.1.2.2 requires (caller buffer %u B, seen %u)",
				  ep_data->cfg.addr, trb_size, in_size,
				  priv->out_unaligned);
		return;
	}

	/* Silent when the caller already supplied whole packets - the normal case. */
	if (mps == 0U || (in_size % mps) == 0U) {
		return;
	}

	priv->out_unaligned++;
	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U) {
		priv->out_unaligned_ctrl++;
	}

	LOG_ERR_RATELIMIT("EP%02x %s OUT descriptor: the stack supplied %u B, not a "
			  "multiple of MPS %u B - rounded up to %u B (databook S4.2.3.3 "
			  "requires whole packets for OUT) (seen %u, control %u)",
			  ep_data->cfg.addr, what, in_size, mps, trb_size,
			  priv->out_unaligned, priv->out_unaligned_ctrl);
}

static void udc_dwc3_push_trb(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data,
			      struct net_buf *const buf, const uint32_t ctrl)
{
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->head];
	const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);
	const uint32_t out_size = !USB_EP_DIR_IS_OUT(ep_data->cfg.addr) ? buf->len
				  : (mps != 0U ? ROUND_UP(buf->size, mps) : buf->size);

	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
		udc_dwc3_out_size_check(dev, ep_data, buf->size, out_size,
					"bulk/intr/isoc", false);
	}

	/*
	 * head, tail, full and net_buf[] are shared with udc_dwc3_pop_trb(), and
	 * neither side takes a lock.
	 *
	 * Against pop_trb specifically that is safe, because both run on the same
	 * work queue: this function is reached from udc_dwc3_ep_worker(), pop_trb
	 * from udc_dwc3_event_worker(), and a work queue runs its items one at a
	 * time. Moving either side off that queue - back into the ISR for latency,
	 * or onto a second queue - would need explicit protection here.
	 *
	 * The work queue alone is NOT enough, because other threads reach this ring
	 * too, off that queue and on the caller's thread:
	 *
	 *   udc_dwc3_ep_enable() -> udc_dwc3_ep_resume(), whose requeue loop calls
	 *     udc_dwc3_trb_bulk() and so reaches this function directly,
	 *   udc_dwc3_ep_disable(), which walks net_buf[] in reverse to drain it,
	 *   udc_dwc3_ep_dequeue().
	 *
	 * Those all hold the UDC mutex. udc_dwc3_ep_worker() now takes the same
	 * mutex for its whole body, so every toucher of head/tail/full/net_buf[]
	 * holds one common lock and the cases above are excluded. Before that it did
	 * not, and holding the mutex elsewhere excluded nothing on this path.
	 *
	 * Both properties are load-bearing: moving either side off the work queue,
	 * or dropping the mutex from the endpoint worker, reopens this.
	 */

	/* If the next TRB in the chain is still owned by the hardware, need
	 * to retry later when more resources become available.
	 */
	__ASSERT_NO_MSG(!ep_data->full);

	/* Associate an active buffer and a TRB together */
	ep_data->net_buf[ep_data->head] = buf;

	/*
	 * An OUT descriptor must be a whole number of packets (S4.2.3.3), so the
	 * programmed size is rounded up to MaxPacketSize.  Normally a no-op: the
	 * allocation behind buf->size is already whole packets - guaranteed by
	 * udc_ctrl_data_alloc() for control, and the class driver's job for its own
	 * OUT endpoints.  udc_dwc3_out_size_check() reports it when it is not.
	 */
	ep_data->n_arm++;

	udc_dwc3_trb_fill(trb, (uintptr_t)buf->data, out_size, ctrl);

	LOG_DBG("PUSH %u, buf %p, data %p, size %u -> %u",
		ep_data->head, (void *)buf, (void *)buf->data, buf->size, out_size);

	ep_data->head = (ep_data->head + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

	ep_data->full = (ep_data->head == ep_data->tail);
}

static int udc_dwc3_pop_trb(const struct device *const dev, struct udc_dwc3_ep_data *const ep_data,
			    struct net_buf **buf, struct udc_dwc3_trb *trb)
{
	*buf = ep_data->net_buf[ep_data->tail];
	*trb = ep_data->trb_buf[ep_data->tail];

	if ((trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0) {
		return -EBUSY;
	}
	if (*buf == NULL) {
		return -ENOBUFS;
	}

	/* Clear the last TRB */
	ep_data->net_buf[ep_data->tail] = NULL;

	LOG_DBG("POP %u EP%02x, buf %p, data %p",
		ep_data->tail, ep_data->cfg.addr, (void *)*buf, (void *)(*buf)->data);

	/* -1 for link trb */
	ep_data->tail = (ep_data->tail + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

	/* If we just pulled a TRB, we know we made one hole and we are not full anymore */
	ep_data->full = false;

	/*
	 * Received length, counted down from the size that was PROGRAMMED, not from
	 * buf->size.
	 *
	 * udc_dwc3_push_trb() rounds an OUT descriptor up to MaxPacketSize, so the
	 * controller decrements BUFSIZ from the rounded value.  Subtracting the
	 * residual from buf->size was therefore wrong whenever the two differ: a
	 * 700-byte buffer on a 512-byte endpoint is programmed as 1024, so a full
	 * 700-byte packet leaves residual 324 and yielded 700 - 324 = 376.  A short
	 * packet was worse - residual could exceed buf->size and the unsigned
	 * subtraction wrapped to a huge length.
	 *
	 * The rounding is deterministic, so the programmed size is recomputed the
	 * same way here.  The result is clamped to buf->size, which bounds it if a
	 * host ever sends more than the caller's buffer can hold.
	 */
	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
		const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);
		const uint32_t programmed =
			(mps != 0U) ? ROUND_UP((*buf)->size, mps) : (*buf)->size;
		const uint32_t residual =
			FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb->status);
		const uint32_t received =
			(programmed > residual) ? (programmed - residual) : 0U;

		(*buf)->len = MIN(received, (*buf)->size);
	}

	return 0;
}

static int udc_dwc3_trb_nonctrl_init(const struct device *const dev,
				   struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *trb = ep_data->trb_buf;
	const uint32_t i = CONFIG_UDC_DWC3_TRB_NUM - 1;

	LOG_DBG("Initializing normal TRB");

	/* HWO=0 on the first TRB will prevent the transfers to start until configured */
	memset((void *)trb, 0x00, sizeof(*trb) * CONFIG_UDC_DWC3_TRB_NUM);

	/* TRB LINK that loops the ring buffer back to the beginning */
	udc_dwc3_trb_fill(&trb[i], (uintptr_t)ep_data->trb_buf, 0U,
			  UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB | UDC_DWC3_TRB_CTRL_HWO);


	/* Start the transfer now, update it later */
	if (!udc_dwc3_depcmd_start_xfer(dev, ep_data)) {
		LOG_ERR("EP%02x ring primed but Start Transfer failed; not enabling",
			ep_data->cfg.addr);
		return -EIO;
	}

	return 0;
}

static void udc_dwc3_trb_ctrl_out(const struct device *const dev, struct net_buf *const buf,
				  const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	uint32_t size;

	priv->last_xfer_type = ctrl;
	priv->last_xfer_dir = USB_EP_DIR_OUT;

	if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
	    ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
		udc_dwc3_ctrl_request_done(dev);
	}

	/*
	 * The TRB carries the size of the TRANSFER, not the size of the buffer
	 * that happens to hold it - and for a SETUP the spec fixes that number.
	 *
	 * SPEC, Programming Guide 3.30b, 3.1.2.2 "Setup and Status TRB Structure":
	 *   "To receive a SETUP packet, the driver queues up a single Setup TRB,
	 *    whose buffer pointer value may be set to any address, including the
	 *    address of the TRB. The buffer size MUST BE SET TO 8. The controller
	 *    writes the 8 bytes of the received SETUP to the address requested."
	 *
	 * This used to program buf->size, and the stack allocates that buffer at
	 * bMaxPacketSize0 - 512 at SuperSpeed - saying so in its own comment:
	 * "Allocate bMaxPacketSize0 despite SETUP being just 8 bytes"
	 * (udc_ctrl_setup_alloc(), udc_common.c). So the Setup TRB was armed with
	 * BUFSIZ = 512 against a spec that says it must be 8, telling the
	 * controller the Setup stage might run to 512 bytes when a SETUP is always
	 * exactly 8. Linux's dwc3 passes a literal 8 (dwc3_ep0_out_start()).
	 *
	 * The status stage is the same mistake in the same line: a status stage is
	 * a zero-length packet, and udc_ctrl_status_alloc() likewise allocates
	 * bMaxPacketSize0 "despite Status being ZLP". The IN direction already
	 * gets this right - udc_dwc3_trb_ctrl_in() uses buf->len, which is 0 - so
	 * only the OUT side was affected.
	 *
	 * The DATA stage keeps buf->size: there the buffer capacity IS the
	 * transfer size the host may fill, which is what that TRB should say.
	 */
	if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		size = sizeof(struct usb_setup_packet);
	} else if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
		   ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
		/*
		 * MaxPacketSize, not 0.  This is the status stage of a control READ:
		 * an OUT transfer, on which the host sends a zero-length packet.
		 *
		 * The databook contradicts itself here.  Figure 3-11 draws the Status
		 * TRB with BUFSIZ 0, but S4.2.3.3 states two prose rules for OUT
		 * endpoints - "The BUFSIZ field must be >= 1 byte" and "The total size
		 * of a Buffer Descriptor must be a multiple of MaxPacketSize" - and
		 * names an exception only for the Setup stage, not the Status stage.
		 * It then adds the rule that decides it: "A received zero-length packet
		 * still requires a MaxPacketSize buffer."  A device cannot know a ZLP is
		 * coming until the transfer completes, so the buffer has to be there
		 * either way.
		 *
		 * Following the prose costs nothing: udc_ctrl_status_alloc() already
		 * allocates bMaxPacketSize0 for this buffer - "despite Status being
		 * ZLP" - so the memory exists and only this driver was discarding it.
		 * The risk is asymmetric: if the figure is right, a ZLP simply lands in
		 * a MaxPacketSize buffer and reports a full residual, which harms
		 * nothing; if the prose is right, BUFSIZ 0 violated both rules on every
		 * control read.
		 *
		 * The IN status stage is unaffected and stays 0 - there the device
		 * SENDS the zero-length packet, and none of these OUT rules apply.
		 */
		size = USB_MPS_EP_SIZE(ep_data->cfg.mps);
	} else {
		size = buf->size;
	}

	/*
	 * An OUT descriptor must be a whole number of packets: "the total size of a
	 * Buffer Descriptor must be a multiple of MaxPacketSize".  Unlike the Setup TRB
	 * size rule there is no interlock - the core accepts a short descriptor and then
	 * has nowhere defined to put a packet that overruns it.  Symptom when it bites:
	 * DSTS.RXFIFOEMPTY clear (a received packet with nowhere to go), GEVNTCOUNT
	 * frozen on an event the core will not place, endpoint dead.
	 *
	 * So round up.  This is a PLAIN round-up of trb[0] - there is no trb[1] and no
	 * scratch buffer.  Linux uses a chained form in __dwc3_ep0_do_control_data();
	 * this driver does not, because chaining costs ring slots and bookkeeping for a
	 * case the stack never produces.
	 *
	 * Tradeoff, accepted knowingly: rounding up permits the controller to write up to
	 * the rounded size into the caller's buffer.  Safe by construction for control -
	 * udc_ctrl_data_alloc() already rounds to bMaxPacketSize0, so this is a no-op
	 * (measured: caller 512 B on every descriptor) - and udc_dwc3_out_size_check()
	 * reports it if that ever stops being true.
	 */
	/*
	 * cfg.mps is the ENCODED Max Packet Size: bits 10:0 are the packet size and
	 * bits 12:11 carry the additional-transactions count for high-bandwidth
	 * periodic endpoints.  The databook rule is a multiple of MaxPacketSize, so
	 * the modulo has to be against the packet-size field alone - using the raw
	 * value would compute against size|(mult<<11) and produce nonsense on any
	 * endpoint that carries mult.  Control endpoints never do, but reading the
	 * field correctly here keeps this right if the padding is ever reused.
	 */
	if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
		const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);

		/* Whole packets for OUT - S4.2.3.3.  Normally already true. */
		if (mps != 0U) {
			size = ROUND_UP(size, mps);
		}
	}

	udc_dwc3_trb_fill(&trb[0], (uintptr_t)buf->data, size,
			  ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO);

	memcpy(&ep_data->trb_cache[0], (void *)&trb[0], sizeof(ep_data->trb_cache[0]));

	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

static void udc_dwc3_trb_ctrl_in(const struct device *const dev,
				 struct net_buf *const buf,
				 const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	priv->last_xfer_type = ctrl;
	priv->last_xfer_dir = USB_EP_DIR_IN;

	if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
	    ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
		udc_dwc3_ctrl_request_done(dev);
	}

	if (udc_ep_buf_has_zlp(buf)) {
		udc_dwc3_trb_fill(&trb[0], (uintptr_t)buf->data, buf->len,
				  ctrl | UDC_DWC3_TRB_CTRL_CHN |
				  UDC_DWC3_TRB_CTRL_HWO);

		/*
		 * The terminating zero-length TRB is not the first TRB of the data
		 * stage, so its type is Normal, not Control-Data: "1: Normal
		 * (Control-Data-2+ / Bulk / Interrupt) - Set TRBCTL to 1 for all TRBs
		 * used in data stage except the first TRB". Repeating Control-Data
		 * here describes a second first-TRB, which is not a shape the
		 * controller is defined to accept.
		 */
		udc_dwc3_trb_fill(&trb[1], 0U, 0U,
				  UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL |
				  UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO);
	} else {
		udc_dwc3_trb_fill(&trb[0], (uintptr_t)buf->data, buf->len,
				  ctrl | UDC_DWC3_TRB_CTRL_LST |
				  UDC_DWC3_TRB_CTRL_HWO);
	}

	memcpy(&ep_data->trb_cache[0], (void *)&trb[0], sizeof(ep_data->trb_cache[0]));

	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

static int udc_dwc3_trb_bulk(const struct device *const dev,
			     struct udc_dwc3_ep_data *const ep_data,
			     struct net_buf *const buf)
{
	uint32_t ctrl = UDC_DWC3_TRB_CTRL_IOC | UDC_DWC3_TRB_CTRL_HWO | UDC_DWC3_TRB_CTRL_CSP;

	/*
	 * DBG for the same reason the control stages are: one line per transfer on
	 * a data endpoint is the pattern that capped control traffic at ~41/s. This
	 * path is idle today because the video endpoint is driven by the RTL block
	 * rather than this driver, but it is the first thing that would fire when
	 * bulk or isochronous support lands.
	 */
	LOG_DBG("TRB_BULK_EP_0x%02x, buf %p, data %p, size %u, len %u",
		ep_data->cfg.addr, (void *)buf, (void *)buf->data, buf->size, buf->len);

	if (ep_data->full) {
		return -EBUSY;
	}

	if (udc_ep_buf_has_zlp(buf)) {
		LOG_DBG("Buffer has a ZLP flag");
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP;
	} else {
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL;
	}

	udc_dwc3_push_trb(dev, ep_data, buf, ctrl);
	udc_ep_set_busy(&ep_data->cfg, true);
	udc_dwc3_depcmd_update_xfer(dev, ep_data);

	/*
	 * last_xfer_type is NOT set here.  It names the current CONTROL stage and
	 * is read as one - udc_dwc3_recover() copies it into watchdog_type when it
	 * has no guarded endpoint.  Writing a Normal/Normal-ZLP TRBCTL into it from
	 * the bulk path made that fallback describe a bulk transfer as a control
	 * stage.
	 */
	return 0;
}

/*
 * Control buffers
 *
 * There is no worker for control buffers, and instead udc_dwc3_ctrl_next()/udc_dwc3_ctrl_try()
 * is called whenever there is an opportunity to send more, and only when all conditions are met.
 *
 * DWC3 internal control buffer state machine does not support them being submitted out of order.
 * This means the driver has to wait the XferNotReady event from the host to make sure the order
 * is respected. This trusts the host for sending the requests in correct order.
 *
 * The USB stack will submit the control buffers out of order, which is supported by most USB
 * controllers (i.e. IN and OUT submitted at the same time rather than one after another).
 *
 *  The TRBs are effectively submitted when the following conditions are met:
 *
 * - There is a buffer ready for ths endopint.
 * - There is an XferNotReady event submitted.
 * - The other endpoint is not busy anymore
 */

/*
 * Arm the stall watchdog against a specific control endpoint and stage, so that
 * recovery acts on what the watchdog was actually guarding rather than on
 * whatever happened to be armed last.
 */
static void udc_dwc3_ctrl_arm_watchdog(const struct device *const dev,
				       const bool is_in, const uint32_t type)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	priv->watchdog_ep = is_in ? &cfg->ep_data_in[0] : &cfg->ep_data_out[0];
	priv->watchdog_type = type;

	/*
	 * Mark where the traffic counters stood, so the SETUP watchdog can tell
	 * "nothing has moved since this SETUP was armed" from "the bus is busy
	 * and the shared RxFIFO simply has someone else's data in it".
	 */
	if (type == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		priv->ctrl_setup_wd_snap_setup = priv->ctrl_setup_done;
		priv->ctrl_setup_wd_snap_nonctrl = priv->nonctrl_done;
	}

	k_work_reschedule(&priv->watchdog_dwork, K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
}

/*
 * Arm the next control stage from this buffer. Returns false when the buffer
 * describes no stage this endpoint can arm, so that udc_dwc3_ctrl_try() can
 * give back the claim it took rather than leaving the control pair blocked.
 */
static bool udc_dwc3_ctrl_next_in(const struct device *const dev,
				  struct net_buf *const buf)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct usb_setup_packet *const setup = &priv->setup_packet;
	const struct udc_buf_info bi = *udc_get_buf_info(buf);

	if (bi.data) {
		LOG_DBG("trb IN_DATA ln=%d d=%p", buf->len, (void *)buf->data);
		//atomic_clear_bit(&priv->expected_xfer, UDC_DWC3_CTRL_IN);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
		udc_dwc3_ctrl_arm_watchdog(dev, true, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi.status && setup->wLength == 0) {
		/*
		 * An IN status stage SENDS the zero-length packet, so its TRB carries
		 * BUFSIZ 0.  (The OUT status stage RECEIVES a ZLP and carries
		 * MaxPacketSize instead - see udc_dwc3_trb_ctrl_out().)
		 * udc_dwc3_trb_ctrl_in() takes BUFSIZ from buf->len for the IN
		 * direction - buf->size is the OUT-side receive capacity, and is what
		 * udc_dwc3_trb_ctrl_out() reads - so len is the field that has to be
		 * cleared here. Clearing only size left the TRB length at whatever the
		 * stack's status buffer happened to carry, which would transmit data
		 * during the status stage. Latent rather than active, because the stack
		 * allocates a zero-length buffer for status - but nothing here enforced
		 * it. Both fields are cleared so the intent reads the same either way.
		 */
		buf->size = 0;
		buf->len = 0;
		LOG_DBG("trb IN_STATUS_2 ln=%d d=%p", buf->len, (void *)buf->data);
		//atomic_clear_bit(&priv->expected_xfer, UDC_DWC3_CTRL_IN);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
		udc_dwc3_ctrl_arm_watchdog(dev, true, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
	} else if (bi.status) {
		/* Same as the two-stage case above: buf->len is what reaches the TRB. */
		buf->size = 0;
		buf->len = 0;
		LOG_DBG("trb IN_STATUS_3 ln=%d d=%p", buf->len, (void *)buf->data);
		//atomic_clear_bit(&priv->expected_xfer, UDC_DWC3_CTRL_IN);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
		udc_dwc3_ctrl_arm_watchdog(dev, true, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
	} else {
		LOG_ERR("Unknown buffer IN type");
		udc_submit_ep_event(dev, buf, -EINVAL);
		return false;
	}

	return true;
}

/*
 * Arm the next control stage from this buffer. Returns false when the buffer
 * describes no stage this endpoint can arm, so that udc_dwc3_ctrl_try() can
 * give back the claim it took rather than leaving the control pair blocked.
 */
static bool udc_dwc3_ctrl_next_out(const struct device *const dev,
				   struct net_buf *const buf)
{
	const struct udc_buf_info bi = *udc_get_buf_info(buf);

	if (bi.setup) {
		LOG_DBG("trb OUT_SETUP sz=%d d=%p", buf->size, (void *)buf->data);
		//atomic_clear_bit(&priv->expected_xfer, UDC_DWC3_CTRL_SETUP);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);

		udc_dwc3_ctrl_arm_watchdog(dev, false,
					   UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);

		/*
		 * The SETUP watchdog is armed, but it cannot act on age alone.
		 *
		 * This TRB is armed speculatively and then waits for the host to decide
		 * to send a request. There is no deadline on that - the bus can sit
		 * idle for minutes with the endpoint perfectly healthy - so a timeout
		 * against it always expires eventually, and the recovery that follows
		 * tears down and re-arms a working endpoint for nothing.
		 *
		 * A rig capture showed the cost of arming it naively: of 490
		 * recoveries, 299 were this. All on the control OUT endpoint, all
		 * re-arming SETUP, all completing successfully and then firing again.
		 * Beyond the noise, each one issued an End Transfer against a live
		 * control endpoint - the command that has been observed to hang.
		 *
		 * So udc_dwc3_watchdog_worker() gates this one on DSTS.RXFIFOEMPTY and
		 * discards the expiry unless a packet is actually stuck. See the gate
		 * for why that bit separates the two cases.
		 *
		 * The stages below keep their unconditional watchdog: once a SETUP has
		 * been received the host owes a prompt data or status stage, so age
		 * alone is meaningful there.
		 */
	} else if (bi.data) {
		LOG_DBG("trb OUT_DATA sz=%d d=%p", buf->size, (void *)buf->data);
		//atomic_clear_bit(&priv->expected_xfer, UDC_DWC3_CTRL_OUT);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
		udc_dwc3_ctrl_arm_watchdog(dev, false, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi.status) {
		/*
		 * buf->size is deliberately NOT zeroed here any more.  The status OUT
		 * buffer is allocated at bMaxPacketSize0 by udc_ctrl_status_alloc(),
		 * and udc_dwc3_trb_ctrl_out() now programs MaxPacketSize for this
		 * stage - see the reasoning there.  Zeroing it would leave the TRB
		 * describing more space than buf->size claims, which reads as a bug
		 * even though the underlying allocation is large enough.
		 */
		LOG_DBG("trb OUT_STATUS_3 sz=%d d=%p", buf->size, (void *)buf->data);
		//atomic_clear_bit(&priv->expected_xfer, UDC_DWC3_CTRL_OUT);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
		udc_dwc3_ctrl_arm_watchdog(dev, false, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
	} else {
		LOG_ERR("Unknown buffer OUT, size %d, data %p", buf->size, (void *)buf->data);
		udc_submit_ep_event(dev, buf, -EINVAL);
		return false;
	}

	return true;
}

/* Defined below; used by the abandon path before their definitions. */
static void udc_dwc3_ctrl_next(const struct device *const dev);
static void udc_dwc3_ctrl_try(const struct device *const dev,
			      struct udc_dwc3_ep_data *ep_data);

/*
 * Did the controller retire this control endpoint's TRB because the host
 * started a new SETUP?
 *
 * TRBSTS 4'h2 is defined as "During the current control transfer data/status
 * phase, another SETUP was received" - the controller's way of saying the host
 * has walked away from the transfer in progress. It is reported in the TRB
 * writeback, so it has to be read from the ring, not from the copy taken when
 * the TRB was armed.
 *
 * TRB 1 is examined only when TRB 0 says it was chained to one. A control IN
 * data stage that needs a zero-length terminator is armed as two TRBs and the
 * status can land on either, but a single-TRB stage leaves TRB 1 holding
 * whatever the last chained arm wrote there - the completion paths clear only
 * TRB 0 - so reading it unconditionally would abandon healthy transfers on
 * stale state. CHN is still readable here because this runs before the
 * completion path clears TRB 0.
 */
static bool udc_dwc3_ctrl_setup_pending(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
	uint32_t sts = trb[0].status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK;

	if (sts == UDC_DWC3_TRB_STATUS_TRBSTS_OK &&
	    (trb[0].ctrl & UDC_DWC3_TRB_CTRL_CHN) != 0) {
		sts = trb[1].status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK;
	}

	if (sts == UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING) {
		priv->ctrl_setup_pending++;
		return true;
	}

	/*
	 * Anything else non-OK is recorded rather than acted on. It is rare, and
	 * seeing it in a capture is what would tell us the controller reports this
	 * condition some other way.
	 */
	if (sts != UDC_DWC3_TRB_STATUS_TRBSTS_OK) {
		priv->ctrl_trbsts_other++;

		/*
		 * First few only. If this turns out to be routine rather than rare,
		 * a line per occurrence is exactly the flood that took the event ring
		 * down before; the counter above carries the real answer and "dwc3
		 * evt" prints it.
		 */
		if (priv->ctrl_trbsts_other <= 8U) {
			LOG_WRN("control completion on EP%02x with TRBSTS 0x%x "
				"(%u so far)", ep_data->cfg.addr,
				(unsigned int)FIELD_GET(UDC_DWC3_TRB_STATUS_TRBSTS_MASK,
							sts),
				priv->ctrl_trbsts_other);
		}
	}

	return false;
}

/*
 * Return every buffer queued on this control endpoint that belongs to the
 * transfer being abandoned, stopping at a SETUP.
 *
 * A SETUP marks the start of the next transfer, so anything queued ahead of it
 * belongs to the one the host walked away from. Leaving those behind would mean
 * arming a stage of a dead transfer: a device-to-host request aborted during
 * its IN data stage still has its status buffer sitting on the OUT endpoint,
 * and that is what the next arm would pick up.
 */
static void udc_dwc3_ctrl_drain_abandoned(const struct device *const dev,
					  struct udc_dwc3_ep_data *const ep_data)
{
	struct net_buf *buf;

	while ((buf = udc_buf_peek(&ep_data->cfg)) != NULL) {
		if (udc_get_buf_info(buf)->setup) {
			break;
		}

		buf = udc_buf_get(&ep_data->cfg);
		if (buf == NULL) {
			break;
		}

		udc_submit_ep_event(dev, buf, -ECONNRESET);
	}
}

/*
 * Abandon the control transfer in progress because the host has started another
 * one, release both control endpoints, and let the queued SETUP be armed.
 */
static void udc_dwc3_ctrl_abandon(const struct device *const dev,
				  struct udc_dwc3_ep_data *const ep_data)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *const peer = (ep_data == &cfg->ep_data_in[0]) ?
					      &cfg->ep_data_out[0] : &cfg->ep_data_in[0];

	/*
	 * Rate limited, because this fires once per abandoned transfer and a host
	 * that floods control requests while streaming abandons them continuously.
	 * A line per occurrence is the same flood the TRBSTS path a few functions
	 * up is already capped against - and under CONFIG_LOG_MODE_MINIMAL every
	 * line is a synchronous busy-wait on the console UART, taken with the UDC
	 * mutex held, so the logging is itself what makes the next transfer late
	 * enough to be abandoned in turn.
	 *
	 * Nothing is lost by suppressing them: the macro reports "Skipped N
	 * messages" when it next emits, and priv->ctrl_setup_pending carried in the
	 * line below is the exact running total, also printed by "dwc3 evt".
	 */
	LOG_WRN_RATELIMIT("host started a new SETUP during a control stage on EP%02x, "
			  "abandoning the transfer in progress (%u so far)",
			  ep_data->cfg.addr, priv->ctrl_setup_pending);

	/*
	 * Drain this endpoint, and the other one only if it is idle.
	 *
	 * A busy peer owns something current - in the ordinary case it is the SETUP
	 * that caused this abort, already armed and about to complete on its own
	 * event. Draining or re-arming it here would either double-arm it or, if
	 * its SETUP has already been processed, throw away the replacement
	 * transfer's freshly queued buffer. An idle peer, on the other hand, can
	 * only be holding buffers belonging to the transfer being abandoned - the
	 * status buffer of a device-to-host request aborted during its data stage,
	 * for instance - and those must go.
	 */
	udc_dwc3_ctrl_drain_abandoned(dev, ep_data);
	udc_ep_set_busy(&ep_data->cfg, false);

	/*
	 * Clear the writeback that brought us here, so a late or duplicate
	 * completion on this endpoint cannot re-enter this path and drain the
	 * transfer that has replaced the abandoned one.
	 */
	memset((void *)&ep_data->trb_buf[0], 0x00, sizeof(ep_data->trb_buf[0]));
	memset((void *)&ep_data->trb_buf[1], 0x00, sizeof(ep_data->trb_buf[1]));
	memset((void *)&ep_data->trb_cache[0], 0x00, sizeof(ep_data->trb_cache));

	/*
	 * SPEC, Programming Guide 3.30b section 4.4.2 step 8, on the controller
	 * skipping a data stage because a new SETUP arrived: "Software has to
	 * reclaim the TRBs with HWO=1 in the skipped TRBs and flush the TxFIFO."
	 * The memsets above are the reclaim; this is the other half, and without it
	 * whatever the controller had already staged for the abandoned IN stage
	 * stays in the FIFO and goes out at the head of the next one - a malformed
	 * response rather than a missing one, which is what a host reports as a
	 * protocol error rather than a stall.
	 *
	 * Flushed whichever endpoint raised the abort: the FIFO belongs to the IN
	 * side either way, and flushing one that is already empty costs a generic
	 * command and nothing else.
	 */
	udc_dwc3_fifo_flush_tx(dev, cfg->ep_data_in[0].cfg.addr & 0x7fU);

	if (udc_ep_is_busy(&peer->cfg)) {
		LOG_DBG("EP%02x is busy with the replacement transfer, "
			"leaving it to its own completion", peer->cfg.addr);

		/*
		 * REVERTED. This used to call udc_dwc3_ctrl_try() on EP0-OUT here,
		 * on the reasoning that abandoning must not leave the control
		 * endpoint unarmed. That reasoning ignored what "peer is busy"
		 * means: a replacement transfer is already in flight, and the SETUP
		 * that caused this abort is being handled on its own completion.
		 * Arming EP0-OUT again from here races that, and a second Start
		 * Transfer against a resource still in use is rejected with
		 * CmdStatus 4'h1 - which is exactly the "Start Transfer failed on
		 * EP00" seen on the rig. Leave the peer to its own completion.
		 */
		return;
	}

	udc_dwc3_ctrl_drain_abandoned(dev, peer);
	udc_ep_set_busy(&peer->cfg, false);

	/*
	 * Release both control endpoints. Only one of them owned a TRB, but the
	 * claim is taken as a pair for data and status stages, so both are cleared
	 * before the replacement SETUP tries to claim its own.
	 */
	/*
	 * Arm the queued SETUP directly rather than going through
	 * udc_dwc3_ctrl_next().
	 *
	 * ctrl_next() asks udc_dwc3_ctrl_get_next_type(), which - when
	 * last_xfer_type is CONTROL_SETUP - derives the next stage from
	 * priv->setup_packet. At this moment that still holds the request the host
	 * has just walked away from, so it would compute a data or status stage for
	 * the dead transfer: an IN stage for a no-data or device-to-host request,
	 * where no buffer is queued, and the new SETUP would never be armed. Only a
	 * host-to-device request with data would happen to route to OUT and work.
	 *
	 * Going straight to the OUT endpoint sidesteps that entirely.
	 * udc_dwc3_ctrl_next_out() re-derives the stage from the buffer's own
	 * setup/data/status flags, and udc_dwc3_trb_ctrl_out() sets last_xfer_type
	 * as a side effect of arming, so the state machine restarts from the SETUP
	 * with no assumption about what came before.
	 */
	udc_dwc3_ctrl_try(dev, &cfg->ep_data_out[0]);
}

/*
 * Is this control endpoint busy only because a SETUP is sitting on it?
 *
 * Read from the ring rather than from remembered state: HWO still set means the
 * controller has not retired it, and TRBCTL says what it is. Nothing to keep in
 * sync, and it stays correct across abandon and re-arm.
 */
static bool udc_dwc3_ctrl_armed_setup(struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	return (trb[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0 &&
	       (trb[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK) ==
		       UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP;
}

static void udc_dwc3_ctrl_try(const struct device *const dev,
			      struct udc_dwc3_ep_data *ep_data)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const peer = USB_EP_DIR_IS_IN(ep_data->cfg.addr) ?
					      &cfg->ep_data_out[0] : &cfg->ep_data_in[0];
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	bool armed;

	/*
	 * The busy test and the claim below are not atomic on their own, and this
	 * function is reached from two places: udc_dwc3_ep_enqueue(), on the caller's
	 * thread, and udc_dwc3_handle_event(), on the event work queue.
	 *
	 * No lock is needed here regardless, because both entries already hold the
	 * UDC mutex: the stack takes it through the driver's .lock op around every
	 * API call including ep_enqueue, and handle_event takes the same mutex with
	 * udc_lock_internal() for the whole dispatch. The two are therefore mutually
	 * exclusive.
	 *
	 * Recorded because it is not obvious from this function alone: anything that
	 * calls it from a path NOT holding the UDC mutex would reintroduce a race in
	 * which both callers pass the test before either sets the flag, and both go
	 * on to program TRB 0 and issue Start Transfer.
	 */
	buf = udc_buf_peek(&ep_data->cfg);
	if (buf == NULL) {
		/*
		 * DBG, not INF: in steady state "nothing queued right now" is the normal
		 * answer, and at INF it was 36% of uart_v5_3.log - 2.4 MB, about 258 s of
		 * console time in a 638 s run. It matters when chasing buffer starvation,
		 * nowhere else.
		 */
		LOG_DBG("EP%02X: no buf", ep_data->cfg.addr);
		return;
	}

	/*
	 * Databook 3.2.2.7 again, on the control side. Arming a stage here ends in
	 * udc_dwc3_depcmd_start_xfer(), which must not run while this endpoint's
	 * End Transfer is still concluding.
	 *
	 * The window is real rather than theoretical:
	 * udc_dwc3_on_set_config_or_interface() ends the control IN endpoint to
	 * force a TX FIFO reconfiguration, and that End Transfer carries CmdIOC
	 * like any other, so ep 0x80 is left with end_xfer_pending set across every
	 * SetConfiguration and SetInterface.
	 *
	 * Nothing is armed and no claim is taken, so the buffer stays queued
	 * exactly as it was; only a peek has happened. udc_dwc3_on_ep_cmd_cmplt()
	 * calls udc_dwc3_ctrl_next() when the completion arrives, which comes back
	 * through here with the flag clear.
	 *
	 * Deliberately not gated on direction or stage. A SETUP is exempt from the
	 * pair check above because it starts a new transfer, but it still needs a
	 * Start Transfer on an endpoint the controller has finished with, so it
	 * waits here like everything else.
	 */
	if (ep_data->end_xfer_pending) {
		priv->ctrl_deferred_arm++;

		/*
		 * Put a deadline on the deferral. Everything else on the control path
		 * is watched from the moment it arms, inside
		 * udc_dwc3_trb_ctrl_in()/_out(), so a stage that never gets that far
		 * would be the one thing here with no timeout behind it - and the
		 * event that releases it is exactly the Endpoint Command Complete this
		 * driver is chasing for going missing.
		 *
		 * k_work_schedule() rather than k_work_reschedule(): if a live stage is
		 * already being watched, its deadline must not be pushed out by a
		 * deferral happening alongside it.
		 */
		/*
		 * Record the endpoint, or udc_dwc3_on_ctrl() cannot recognise this
		 * deadline as its own and will leave it pending for ever. Only when
		 * no stage is already being watched: k_work_schedule() above keeps a
		 * live stage's deadline, so its ownership must be kept too.
		 */
		if (priv->watchdog_ep == NULL) {
			priv->watchdog_ep = ep_data;
			priv->watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
		}

		k_work_schedule(&priv->watchdog_dwork,
				K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));

		LOG_DBG("EP%02X still concluding an End Transfer, not arming yet",
			ep_data->cfg.addr);
		return;
	}

	/*
	 * A SETUP is checked against ITS OWN endpoint only; every other stage has
	 * to wait for the pair.
	 *
	 * A SETUP does not belong to the transfer in progress - it starts a new
	 * one, and the device must be able to receive it at any time. Requiring
	 * both control endpoints to be idle meant that while a status-IN stage was
	 * outstanding on endpoint 1, a SETUP could not be armed on endpoint 0 even
	 * though endpoint 0 was free.
	 *
	 * That is a mutual deadlock, and it is what the captures show: the host
	 * moves on and stops issuing the IN token for the status stage, so that TRB
	 * never retires; the device has no SETUP armed, so the host's new request
	 * is never accepted; neither side progresses, and the watchdog re-arms the
	 * same unwinnable wait for ever.
	 *
	 * With a SETUP armed the controller can accept the new request, and then
	 * retires the outstanding descriptor with TRBSTS = SetupPending, which
	 * udc_dwc3_ctrl_abandon() turns into an orderly restart.
	 */
	if (udc_get_buf_info(buf)->setup) {
		if (udc_ep_is_busy(&ep_data->cfg)) {
			LOG_DBG("EP%02X: busy (SETUP)", ep_data->cfg.addr);
			priv->ctrl_decline++;
			priv->ctrl_decline_pending = true;
			return;
		}
	} else if (udc_ep_is_busy(&ep_data->cfg)) {
		LOG_DBG("EP%02X: busy", ep_data->cfg.addr);
		priv->ctrl_decline++;
		priv->ctrl_decline_pending = true;
		return;
	} else if (udc_ep_is_busy(&peer->cfg) && !udc_dwc3_ctrl_armed_setup(peer)) {
		LOG_DBG("ctrl eps busy");
		priv->ctrl_decline++;
		priv->ctrl_decline_pending = true;
		return;
	}

	/* A claim was granted, so the control path is moving. */
	priv->ctrl_arm_t0 = k_cycle_get_32();
	priv->ctrl_decline_pending = false;

	udc_ep_set_busy(&ep_data->cfg, true);

	/*
	 * Give the claim back if nothing was armed. The reject paths in
	 * udc_dwc3_ctrl_next_in()/_out() report an unusable buffer and return; with
	 * the claim left standing, this endpoint - and through the pair check, the
	 * other one too - would stay blocked until the watchdog intervened.
	 */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		armed = udc_dwc3_ctrl_next_in(dev, buf);
	} else {
		armed = udc_dwc3_ctrl_next_out(dev, buf);
	}

	if (!armed) {
		udc_ep_set_busy(&ep_data->cfg, false);
	} else {
		priv->ctrl_armed_t = k_cycle_get_32();
		priv->ctrl_armed_n++;
	}
}

/*
 * Retired by the change that made udc_dwc3_ctrl_next() offer both control
 * endpoints instead of choosing one. Kept under #if 0 for comparison against
 * the stock driver, which still routes this way. See the comment in
 * udc_dwc3_ctrl_next() for why the routing was unsound once a SETUP could be
 * armed alongside a live transfer.
 */
#if 0
static void udc_dwc3_ctrl_get_next_type(const struct device *const dev,
					uint8_t *dir, uint8_t *type)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	if (priv->last_xfer_type == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		if (priv->setup_packet.wLength == 0) {
			*dir = USB_EP_DIR_IN;
			*type = UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2;

		} else if (priv->setup_packet.RequestType.direction == USB_REQTYPE_DIR_TO_DEVICE) {
			*dir = USB_EP_DIR_OUT;
			*type = UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA;

		} else if (priv->setup_packet.RequestType.direction == USB_REQTYPE_DIR_TO_HOST) {
			*dir = USB_EP_DIR_IN;
			*type = UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA;
		}
	} else if (priv->last_xfer_type == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
		if (priv->setup_packet.RequestType.direction == USB_REQTYPE_DIR_TO_DEVICE) {
			*dir = USB_EP_DIR_IN;
			*type = UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3;
		} else {
			*dir = USB_EP_DIR_OUT;
			*type = UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3;
		}
	} else if (priv->last_xfer_type == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3
		|| priv->last_xfer_type == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2) {
		*dir = USB_EP_DIR_OUT;
		*type = UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP;
	}
}
#endif


static void udc_dwc3_ctrl_next(const struct device *const dev)
{
	//struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;

	LOG_DBG("load");

	/*
	 * Offer BOTH control endpoints rather than computing which is next.
	 *
	 * The computation came from udc_dwc3_ctrl_get_next_type(), which reads
	 * last_xfer_type - set by trb_ctrl_in()/_out() as a side effect of arming ANY
	 * stage, including the speculative SETUP.  Once a SETUP can be armed alongside a
	 * live transfer, last_xfer_type describes the SETUP rather than the transfer the
	 * host is still working through, so the inference picks the wrong endpoint.
	 *
	 * udc_dwc3_ctrl_try() already refuses an endpoint that is busy or has nothing
	 * queued, so offering both is both simpler and correct: whichever genuinely has
	 * work takes it, and the other is a no-op.
	 */
	udc_dwc3_ctrl_try(dev, &cfg->ep_data_in[0]);

	/*
	 * Then give a queued SETUP its own chance, independently of what the stage
	 * above worked out.
	 *
	 * A SETUP starts a new transfer, so it does not depend on which stage the
	 * current one is at, and it is gated only on its own endpoint. Two things
	 * follow. In steady state the status stage and the next SETUP end up armed
	 * together, which is what stops the host and the device deadlocking over an
	 * abandoned transfer. And when last_xfer_type still describes a transfer
	 * that has been abandoned - so the computation above routes to an endpoint
	 * with nothing queued - the SETUP is still armed rather than stranded.
	 *
	 * Harmless when the OUT endpoint already took a stage above:
	 * udc_dwc3_ctrl_try() finds it busy and returns.
	 */
	udc_dwc3_ctrl_try(dev, &cfg->ep_data_out[0]);

#if 0
	if (atomic_test_bit(&priv->expected_xfer, UDC_DWC3_CTRL_SETUP)) {
		udc_dwc3_ctrl_try(dev, &cfg->ep_data_out[0]);

	} else if (atomic_test_bit(&priv->expected_xfer, UDC_DWC3_CTRL_IN)) {
		udc_dwc3_ctrl_try(dev, &cfg->ep_data_in[0]);

	} else if (atomic_test_bit(&priv->expected_xfer, UDC_DWC3_CTRL_OUT)) {
		udc_dwc3_ctrl_try(dev, &cfg->ep_data_out[0]);

	} else {
		LOG_INF("No XferNotReady event yet, waiting");
	}
#endif
}

#include "../../subsys/usb/device_next/usbd_ch9.h"

static int udc_dwc3_ep_dequeue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg);
static int udc_dwc3_disable(const struct device *const dev);
static int udc_dwc3_enable(const struct device *const dev);
static int udc_dwc3_init(const struct device *const dev);
static int udc_dwc3_shutdown(const struct device *const dev);
static int udc_dwc3_ep_enable(const struct device *const dev, struct udc_ep_config *const ep_cfg);
static int udc_dwc3_ep_disable(const struct device *const dev, struct udc_ep_config *const ep_cfg);

/*
 * Second half of a control-endpoint recovery.
 *
 * Called from the Endpoint Command Complete handler once the End Transfer that
 * udc_dwc3_recover() issued has finished. Running here rather than inline in
 * recover() is what keeps a command wait off the UDC mutex: by the time this
 * runs the controller has already reported the End Transfer complete, so the
 * Start Transfer below finds CmdAct clear and its pre-poll costs nothing.
 *
 * Runs with the UDC mutex held, from udc_dwc3_handle_event().
 */
static void udc_dwc3_ctrl_rearm(const struct device *const dev,
				struct udc_dwc3_ep_data *const ep_data)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	/*
	 * The stage the watchdog was guarding, not the one armed most recently.
	 * With a SETUP armable while a status stage is outstanding, last_xfer_type
	 * can already describe the SETUP by the time recovery runs.
	 */
	const uint32_t type = priv->watchdog_type;
	struct net_buf *buf;
	const char *name;

	switch (type) {
	case UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP:
		name = "SETUP";
		break;
	case UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA:
		name = "DATA";
		break;
	case UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2:
		name = "STATUS_2";
		break;
	case UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3:
		name = "STATUS_3";
		break;
	default:
		name = "<?>";
		break;
	}

	/*
	 * The queue can be empty by now. udc_dwc3_on_ctrl() removes the buffer as
	 * soon as a transfer completes, and "dwc3 recover" can be typed at any
	 * moment. Both arming helpers dereference the buffer immediately, so the
	 * peek is checked rather than passed on.
	 *
	 * With nothing queued there is nothing to re-arm and nothing to wait for:
	 * the End Transfer has already cleared the endpoint, and udc_dwc3_ctrl_try()
	 * arms it when the stack queues a buffer. The watchdog is deliberately not
	 * rescheduled on that path - it would only End an idle endpoint again.
	 */
	buf = udc_buf_peek(&ep_data->cfg);
	if (buf == NULL) {
		LOG_WRN("nothing queued on EP%02x, ended the transfer without "
			"re-arming", ep_data->cfg.addr);
		/*
		 * Release the claim too. recover() left it standing because a stage was
		 * about to be re-armed; with nothing to arm, leaving it set would keep
		 * the endpoint marked busy with no TRB behind it, and the pair check
		 * would then block the other one as well.
		 */
		udc_ep_set_busy(&ep_data->cfg, false);
		return;
	}

	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		LOG_INF("trb IN_%s", name);
		udc_dwc3_trb_ctrl_in(dev, buf, type);
	} else {
		LOG_INF("trb OUT_%s", name);
		udc_dwc3_trb_ctrl_out(dev, buf, type);
	}

	/*
	 * Same rule as the arming path, including for SETUP: the expiry is gated
	 * on RXFIFOEMPTY in udc_dwc3_watchdog_worker(), so re-arming it here can no
	 * longer produce the self-sustaining recovery loop that removing it fixed.
	 */
	udc_dwc3_ctrl_arm_watchdog(dev, USB_EP_DIR_IS_IN(ep_data->cfg.addr), type);
}

/*
 * Dump the controller's own view of itself.
 *
 * Everything else this driver logs at a wedge is inference from the outside:
 * TRB ownership, DSTS, ring contents. None of it says what the core is stuck
 * on. These registers do. They are passive reads with no side effects, so this
 * is safe to call from the watchdog with the core in any state.
 *
 * GDBGBMU is the one that matters most: it is the bus master unit, and a core
 * reporting COREIDLE=0 with an armed TRB, data in the RxFIFO and no events
 * posted is either stuck in a DMA or waiting on something upstream of it.
 * GDBGFIFOSPACE walks every queue rather than the single RxFIFO the per-endpoint
 * dump reads, so a queue that has filled and stopped draining is visible here
 * and nowhere else.
 */
static void udc_dwc3_core_state_dump(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	static const struct {
		const char *name;
		uint32_t sel;
	} queues[] = {
		{ "TXQ",      UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXQ },
		{ "RXQ",      UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXQ },
		{ "TXREQQ",   UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXREQQ },
		{ "RXREQQ",   UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQQ },
		{ "RXINFOQ",  UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXINFOQ },
		{ "PSTATUSQ", UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOLSTATUSQ },
		{ "DESCFETQ", UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCHQ },
		{ "WREVENTQ", UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_WREVENTQ },
		{ "AUXEVENTQ", UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENTQ },
	};

	LOG_ERR("  CORE: GDBGLTSSM=0x%08x GDBGBMU=0x%08x GDBGLNMCC=0x%08x "
		"GDBGLSP=0x%08x",
		sys_read32(base + UDC_DWC3_GDBGLTSSM),
		sys_read32(base + UDC_DWC3_GDBGBMU),
		sys_read32(base + UDC_DWC3_GDBGLNMCC),
		sys_read32(base + UDC_DWC3_GDBGLSP));
	LOG_ERR("  CORE: GDBGEPINFO=0x%08x_%08x",
		sys_read32(base + UDC_DWC3_GDBGEPINFO1),
		sys_read32(base + UDC_DWC3_GDBGEPINFO0));

	for (uint32_t i = 0; i < ARRAY_SIZE(queues); i++) {
		uint32_t r = queues[i].sel;

		/* Queue 0: the per-endpoint RxFIFO view is already dumped above. */
		r |= FIELD_PREP(UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK, 0U);
		sys_write32(r, base + UDC_DWC3_GDBGFIFOSPACE);
		r = sys_read32(base + UDC_DWC3_GDBGFIFOSPACE);

		LOG_ERR("  CORE: %-9s space=%u (raw 0x%08x)", queues[i].name,
			(uint32_t)FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK, r),
			r);
	}
}

static int udc_dwc3_recover(const struct device *dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *ep_data;

#if 0
	LOG_WRN("CTRL IN:");
	udc_dwc3_dump_trb(dev, &cfg->ep_data_in[0], NULL);
	LOG_WRN("CTRL OUT:");
	udc_dwc3_dump_trb(dev, &cfg->ep_data_out[0], NULL);
#endif

	LOG_WRN("Recovering USB state");

#if 0
	/* This did not work well */
	priv->evt_next = 0;
	udc_dwc3_disable(dev);
	udc_dwc3_shutdown(dev);
	udc_dwc3_init(dev);
	udc_dwc3_enable(dev);
	return 0;
#endif

#if 0
	udc_dwc3_ep_disable(dev, &cfg->ep_data_in[0].cfg);
	udc_dwc3_ep_enable(dev, &cfg->ep_data_in[0].cfg);

	udc_dwc3_ep_disable(dev, &cfg->ep_data_out[0].cfg);
	udc_dwc3_ep_enable(dev, &cfg->ep_data_out[0].cfg);

	udc_ep_set_busy(&cfg->ep_data_in[0].cfg, false);
	udc_ep_set_busy(&cfg->ep_data_out[0].cfg, false);

	k_sleep(K_MSEC(100));

	/* Ask the stack for a new setup packet */
	//udc_submit_event(dev, UDC_EVT_NEW_SETUP, 0);
	/* TODO: send the correct next packet in the sequence instead: needs refactoring the
	 * driver for this.
	 */

	udc_dwc3_enable(dev);
#endif
	/*
	 * Take the UDC mutex, not the scheduler lock.
	 *
	 * This runs on the SYSTEM work queue - the watchdog is rescheduled with
	 * k_work_reschedule(), which submits to the system queue, while the event
	 * handler runs on udc_get_work_q(). The two are different threads, and
	 * udc_dwc3_handle_event() holds the UDC mutex for its whole dispatch.
	 *
	 * k_sched_lock() is not equivalent to that. It prevents another thread from
	 * STARTING while this one runs; it does nothing about a handle_event() that
	 * had already started and then blocked - inside a log call, or on the mutex
	 * itself. In that window this function would issue End Transfer followed by
	 * Start Transfer on a control endpoint while the event handler was
	 * mid-sequence on the same endpoint: two threads arming control TRBs with no
	 * lock in common, which is the same double-arming this driver has already
	 * been bitten by once.
	 *
	 * Taking the same mutex closes it. There is no deadlock risk: nothing in the
	 * event dispatch calls this function, the only callers are the watchdog
	 * worker and the shell command, and the watchdog cancel in the control
	 * completion path is the non-blocking k_work_cancel_delayable(), which does
	 * not wait for a running handler.
	 */
	udc_lock_internal(dev, K_FOREVER);

	/*
	 * A recovery is already outstanding - the End Transfer issued last time has
	 * not reported completion. Issuing another command on that endpoint now
	 * would have to wait for the first one to finish, and that wait would be
	 * held across this mutex, which is precisely what the event handler needs
	 * in order to deliver the completion being waited for. Doing so would
	 * rebuild the deadlock this function was restructured to avoid.
	 *
	 * So report it and leave the endpoint alone. The watchdog keeps checking,
	 * and udc_dwc3_ctrl_rearm() finishes the recovery if the event does turn
	 * up. An End Transfer that never completes is the controller-side fault
	 * this driver is chasing, and this line is what makes it visible.
	 */
	/*
	 * UNREACHABLE AS IT STANDS.  ctrl_recovery_pending is only ever assigned
	 * false (here, in udc_dwc3_on_ep_cmd_cmplt(), in ctrl_request_done() and at
	 * enable); nothing sets it true, so this block and the re-arm it pairs with
	 * in udc_dwc3_on_ep_cmd_cmplt() never run, and udc_dwc3_ctrl_rearm() is
	 * reached only from there.
	 *
	 * Left in place deliberately rather than deleted: it is inert at runtime, so
	 * removing it cannot fix anything, while ripping it out would cascade into
	 * ctrl_rearm() and the end_xfer_pending bookkeeping during an investigation.
	 * Do not read it as a working End-Transfer recovery - there isn't one.
	 */
	if (priv->ctrl_recovery_pending) {
		const uint32_t age =
			k_cyc_to_ms_near32(k_cycle_get_32() - priv->ctrl_recovery_t0);

		/*
		 * Give the latch an exit.
		 *
		 * ctrl_recovery_pending is cleared by the Endpoint Command Complete
		 * for the End Transfer we issued - and by nothing else short of a
		 * bus reset. So its only way out is an event, on the one path where
		 * events have already stopped arriving: uart_v6_8 shows this branch
		 * repeating to the end of the capture with GEVNTCOUNT=0 every time,
		 * i.e. waiting for a completion that provably cannot come. A latch
		 * whose only exit is the thing that is broken disables recovery for
		 * the rest of the run, which is worse than any recovery it was
		 * guarding against.
		 *
		 * Held for twice the recovery timeout, so an End Transfer that is
		 * merely slow still gets its own completion and the ordinary path is
		 * untouched. Past that the promise is abandoned and the next attempt
		 * is allowed through.
		 */
		if (age >= 2U * CONFIG_UDC_DWC3_RECOVERY_TIMEOUT) {
			LOG_ERR("recovery End Transfer on EP%02x never reported "
				"completion after %u ms (GEVNTCOUNT=%u): abandoning "
				"the wait, recovery is re-enabled",
				priv->ctrl_recovery_ep != NULL ?
					priv->ctrl_recovery_ep->cfg.addr : 0U,
				age,
				sys_read32(DEVICE_MMIO_NAMED_GET(dev, base) +
					   UDC_DWC3_GEVNTCOUNT(0)));

			if (priv->ctrl_recovery_ep != NULL) {
				priv->ctrl_recovery_ep->end_xfer_pending = false;
			}
			priv->ctrl_recovery_pending = false;
			priv->ctrl_recovery_ep = NULL;
		} else {
			LOG_WRN("recovery End Transfer on the control endpoint has "
				"not completed yet (%u ms), not issuing another "
				"command, GEVNTCOUNT=%u bytes", age,
				sys_read32(DEVICE_MMIO_NAMED_GET(dev, base) +
					   UDC_DWC3_GEVNTCOUNT(0)));
			k_work_reschedule(&priv->watchdog_dwork,
					  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
			udc_unlock_internal(dev);

			return 0;
		}

		/* Timed out above: fall through and retry the recovery. */
	}

	/*
	 * A control endpoint still marked as concluding an End Transfer, with no
	 * recovery of our own outstanding, means its Endpoint Command Complete
	 * never arrived. Nothing else will clear that: udc_dwc3_ctrl_try() declines
	 * to arm while it is set, and the only other way out is a USB reset.
	 *
	 * This is the escape for that. The flag is dropped so the control path can
	 * arm again, and the stage is offered immediately rather than waiting for
	 * the next event. Logged loudly and counted, because a lost completion is a
	 * controller-side fault and not something to absorb quietly - if this line
	 * appears at all, priv->ctrl_deferred_arm is the number to read next.
	 */
	{
		struct udc_dwc3_ep_data *const ctrl[2] = {
			&cfg->ep_data_in[0], &cfg->ep_data_out[0],
		};
		bool released = false;

		for (int i = 0; i < 2; i++) {
			if (!ctrl[i]->end_xfer_pending) {
				continue;
			}

			LOG_ERR("no Endpoint Command Complete arrived for the End Transfer "
				"on EP%02x, releasing it (deferred arms so far: %u)",
				ctrl[i]->cfg.addr, priv->ctrl_deferred_arm);

			ctrl[i]->end_xfer_pending = false;
			released = true;
		}

		if (released) {
			udc_dwc3_ctrl_next(dev);
			k_work_reschedule(&priv->watchdog_dwork,
					  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
			udc_unlock_internal(dev);
			return 0;
		}
	}

	/*
	 * End the stuck transfer and stop here. The re-arm happens in
	 * udc_dwc3_ctrl_rearm(), driven by this End Transfer's own completion
	 * event.
	 *
	 * Issuing the Start inline would mean waiting for the End to finish first,
	 * since a command cannot be issued while the previous one on the endpoint
	 * is still active - and that wait would be held across the UDC mutex, which
	 * is exactly what the event handler needs in order to deliver the
	 * completion being waited for. The databook names that deadlock and gives
	 * this as the way out: "waiting for the command complete interrupt (by
	 * setting the bit 8, Command Interrupt on Complete (CmdIOC)) while it is
	 * processing the events".
	 *
	 * End Transfer already carries CmdIOC, so nothing extra is needed to make
	 * the event arrive. The watchdog stays armed as the fallback for when it
	 * does not - which is the failure this driver is chasing.
	 */
	/*
	 * Act on the endpoint the watchdog was guarding. last_xfer_dir names
	 * whichever control TRB was armed most recently, and since a SETUP can be
	 * armed alongside an outstanding status stage that is not necessarily the
	 * one that stalled - recovering from it would End a healthy endpoint.
	 */
	ep_data = priv->watchdog_ep;

	/*
	 * Nothing recorded means this did not come from the watchdog - "dwc3
	 * recover" typed at the shell reaches here too, and a person asking for a
	 * recovery expects one. Fall back to whichever control endpoint is
	 * actually claimed, which is deterministic; only if neither is is there
	 * genuinely nothing to end.
	 */
	if (ep_data == NULL) {
		if (udc_ep_is_busy(&cfg->ep_data_in[0].cfg)) {
			ep_data = &cfg->ep_data_in[0];
		} else if (udc_ep_is_busy(&cfg->ep_data_out[0].cfg)) {
			ep_data = &cfg->ep_data_out[0];
		} else {
			LOG_WRN("no control stage outstanding, nothing to recover");
			udc_unlock_internal(dev);
			return 0;
		}
	}

	/*
	 * The re-arm needs a stage to restore. If the watchdog did not record one,
	 * take the last armed value - imprecise, but this path is manual and the
	 * alternative is refusing to act at all.
	 */
	if (priv->watchdog_ep == NULL) {
		priv->watchdog_type = priv->last_xfer_type;
	}

	/*
	 * Only wait for a completion that is actually coming. If the command could
	 * not be issued, or runs without CmdIOC, no event will arrive and leaving
	 * the flag set would make every later recovery decline to act.
	 */
	/*
	 * Bounded to the fast poll for the whole of this call - see
	 * udc_dwc3_wait_cmdact_zero(). If the endpoint is still busy the command is
	 * not issued and ctrl_recovery_pending stays clear, so the watchdog
	 * rescheduled below tries again with the mutex released in between.
	 */
	priv->depcmd_no_sleep = true;

	/*
	 * Set Stall, NOT End Transfer.
	 *
	 * End Transfer is the natural reflex and it is wrong here. On a control
	 * endpoint this controller does not complete it: uart_v6_9 shows DEPCMD
	 * 0x00000d08 on EP0 - CMDTYP 8, CMDACT still asserted after 1000 ms -
	 * blocking every later command with "previous command still active",
	 * sixty consecutive recovery attempts, none issued. DSTS kept advancing
	 * its frame counter in U0 throughout, so the link and the core were fine
	 * and only the endpoint command engine was stuck. This file already
	 * carried the rule - End Transfer "never on a control endpoint, where it
	 * has been observed to hang the controller" - and this path was the one
	 * place still breaking it.
	 *
	 * Set Stall is what the databook prescribes for control resynchronisation
	 * (4.4.1/4.4.2 error cases: Set Stall on EP0, return to Step 1), and it
	 * is self-completing: "The controller automatically clears the STALL when
	 * it receives a SETUP token", and that SETUP also retires the outstanding
	 * descriptor with SetupPending - an XferComplete, which 3.2.2.2 names as
	 * the other way the transfer resource is released. So the host's own
	 * retry completes the recovery, with no command that can hang.
	 *
	 * Nothing is re-armed here on purpose. The claim is released by
	 * udc_dwc3_ctrl_abandon() when that SetupPending completion arrives,
	 * which is the ordinary path and already re-arms the SETUP.
	 */
	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_out[0]);

	priv->depcmd_no_sleep = false;

	/*
	 * Set Stall on its own is enough when the stage simply needs abandoning -
	 * uart_v6_1355 shows four in a row rescued that way. It is not enough when
	 * the endpoint stays claimed afterwards, and repeating it does not become
	 * enough: uart_01sep_2059_fix2 issued 163 of these in eighteen minutes
	 * without a single SETUP retiring between them, then wedged anyway.
	 *
	 * So escalate on the second recovery that retires nothing. Comparing
	 * ctrl_setup_done rather than counting calls is what keeps this rare - any
	 * SETUP completing in between makes the next recovery a fresh one rather
	 * than a repeat, and the common case never reaches the reclaim at all.
	 */
	/*
	 * NO End Transfer on the control pair here, and no "reclaim".
	 *
	 * A reclaim was tried - End Transfer with HIPRI_FORCERM on both control
	 * endpoints, clear the busy claim, re-arm - and uart_01sep_2328_epdis
	 * shows it CREATING the wedge it was meant to clear. EP80 reads
	 * ctrl=0x00000000 at control watchdog #1, the reclaim runs twice, and by
	 * watchdog #2 it reads ctrl=0x53 with HWO set and busy clear: a descriptor
	 * the controller owns and the driver has forgotten. What follows is
	 * "EpCmdCmplt on EP00 with no End Transfer outstanding" (the flag was
	 * cleared before the completions arrived), "Missing buffer for EP00", and
	 * then Start Transfer on EP80 failing with CmdStatus 1 - no transfer
	 * resource - seven times, permanently.
	 *
	 * This is the hazard udc_dwc3_depcmd_start_xfer() already documents from
	 * uart_v6_9: End Transfer on a CONTROL endpoint hangs this controller's
	 * command engine. Set Stall alone does not clear every stall, but it does
	 * not manufacture this one.
	 */

	k_work_reschedule(&priv->watchdog_dwork, K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));

	udc_unlock_internal(dev);

	return 0;
}

/*
 * Events
 *
 * Process the events from the event ring buffer. Interrupts gives us a
 * hint that an event is available, which we fetch from a ring buffer shared
 * with the hardware.
 */

/*
 * Drop every outstanding End Transfer promise.
 *
 * end_xfer_pending and resume_pending both mean "an Endpoint Command Complete
 * is still on its way". Anything that makes that untrue has to come through
 * here, because three separate paths now refuse to act while those flags are
 * set: udc_dwc3_ep_resume() postpones, udc_dwc3_ep_worker() stops pushing, and
 * udc_dwc3_ctrl_try() declines to arm. If the completion is never going to
 * arrive, leaving them set strands the endpoint for the rest of the session -
 * which is the outcome the deferral was reasoned to be safe from, with the
 * connection-survives assumption removed.
 *
 * Four events qualify, and the transfers those completions belonged to are gone
 * in every one of them: USB reset, disconnect, controller disable, and soft
 * reset. The control-side recovery record is dropped for the same reason.
 */
static void udc_dwc3_drop_xfer_state(const struct device *const dev,
				     const char *const reason)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	LOG_DBG("dropping outstanding End Transfer state (%s)", reason);

	priv->ctrl_recovery_pending = false;
	priv->ctrl_recovery_ep = NULL;

	/*
	 * Cancel, not just forget. Since udc_dwc3_on_ctrl() only cancels a
	 * watchdog whose endpoint matches the one that completed, a deadline left
	 * pending here with its owner cleared is one nothing can ever match - it
	 * fires later against an unrelated stage.
	 */
	k_work_cancel_delayable(&priv->watchdog_dwork);
	priv->watchdog_ep = NULL;
	priv->watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;

	for (int i = 0; i < cfg->num_in_eps; i++) {
		cfg->ep_data_in[i].end_xfer_pending = false;
		cfg->ep_data_in[i].resume_pending = false;
	}

	for (int i = 0; i < cfg->num_out_eps; i++) {
		cfg->ep_data_out[i].end_xfer_pending = false;
		cfg->ep_data_out[i].resume_pending = false;
	}
}

static void udc_dwc3_on_soft_reset(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	/* Configure and reset the Device Controller */
	/* TODO confirm that DWC_USB3_EN_LPM_ERRATA == 1 */
	reg = UDC_DWC3_DCTL_CSFTRST;
	reg |= FIELD_PREP(UDC_DWC3_DCTL_LPM_NYET_THRES_MASK, 15);
	sys_write32(reg, base + UDC_DWC3_DCTL);

	/*
	 * Bounded. This used to be a bare spin with no timeout and no yield, so a
	 * controller that never cleared CSftRst hung the driver here for good -
	 * during init, before any of the recovery machinery exists, with nothing
	 * able to report it. The wait is short by nature (the core clears the bit
	 * when the reset completes), so a generous ceiling costs nothing and turns
	 * an unrecoverable hang into a diagnosable one.
	 */
	for (uint32_t i = 0; i < UDC_DWC3_CSFTRST_MAX_POLLS; i++) {
		if ((sys_read32(base + UDC_DWC3_DCTL) &
		     UDC_DWC3_DCTL_CSFTRST) == 0U) {
			break;
		}
		k_busy_wait(UDC_DWC3_CSFTRST_POLL_US);
	}

	if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_CSFTRST) != 0U) {
		LOG_ERR("CSftRst still set after %u us - the core did not complete "
			"its reset; continuing, but every register written from here "
			"is suspect",
			UDC_DWC3_CSFTRST_MAX_POLLS * UDC_DWC3_CSFTRST_POLL_US);
	}

	/*
	 * The core has just been reset, so no command issued before it can still
	 * report completion. Clearing here also covers the enable path: this runs
	 * from udc_dwc3_init(), so a disable/enable cycle cannot carry stale End
	 * Transfer state across into the new session.
	 */
	udc_dwc3_drop_xfer_state(dev, "soft reset");

	/*
	 * The endpoint command registers go back to being undefined on read, and no
	 * command issued before the reset can still be active, so the next command
	 * on each endpoint must skip the pre-poll again. See priv->depcmd_issued.
	 */
	DEV_DATA(dev)->depcmd_issued = 0;

	/* Enable AXI64 bursts for various sizes expected */
	//reg = UDC_DWC3_GSBUSCFG0_INCR256BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR128BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR64BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR32BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR16BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR8BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR4BRSTENA;
	reg = 0;
	sys_set_bits(base + UDC_DWC3_GSBUSCFG0, reg);

	/*
	 * Global Rx Threshold: disable multi-packet RX thresholding.
	 *
	 * Databook 1.2.4 erratum: an "ACK TP with NumP=0 followed by ACK TP with NumP=1
	 * without ERDY TP ... during a burst bulk OUT transfer" can leave third-party
	 * USB 3.0 hosts waiting for an ERDY.  The documented workaround is
	 * GRXTHRCFG.UsbRxPktCntSel=0 plus fixed DCFG.NUMP - NUMP is programmed below, so
	 * clearing this bit is what selects that mode.
	 *
	 * Databook 4.2.4 reaches the same condition another way: with RX thresholding on,
	 * "do not use the 'on-demand' mode of transfer for SS OUT endpoints", or a final
	 * ACK TP (NumP=0) deadlocks the host waiting for an ERDY that on-demand software
	 * never sends.  This driver arms OUT TRBs only when the stack enqueues a buffer -
	 * exactly that on-demand mode - so either this bit goes or SS OUT endpoints must
	 * keep a TRB permanently armed.  Clearing it is much the smaller change.
	 *
	 * The power-on value is logged before anything is modified.
	 */
	reg = sys_read32(base + UDC_DWC3_GRXTHRCFG);
	/*
	 * The bus/DMA configuration, read once and never before logged.
	 *
	 * GSBUSCFG0's burst-enable bits are all commented out in this function, so
	 * whatever the bitfile leaves there is what the controller uses - and with
	 * no INCR burst enabled the databook says every DMA falls back to the
	 * largest enabled length, i.e. single beats. That is a throughput property
	 * under stress and nobody has ever looked at the value. GSBUSCFG1 carries
	 * the outstanding-request limit; GUCTL1 carries errata workaround bits.
	 * Neither was previously even defined here.
	 */
	LOG_INF("BUSCFG: GSBUSCFG0=0x%08x GSBUSCFG1=0x%08x GUCTL=0x%08x "
		"GUCTL1=0x%08x GCTL=0x%08x",
		sys_read32(base + UDC_DWC3_GSBUSCFG0),
		sys_read32(base + UDC_DWC3_GSBUSCFG1),
		sys_read32(base + UDC_DWC3_GUCTL),
		sys_read32(base + UDC_DWC3_GUCTL1),
		sys_read32(base + UDC_DWC3_GCTL));

	LOG_INF("GRXTHRCFG=0x%08x at reset (UsbRxPktCntSel=%u, UsbRxPktCnt=%u)", reg,
		(reg & UDC_DWC3_GRXTHRCFG_USBRXPKTCNTSEL) ? 1U : 0U,
		(uint32_t)FIELD_GET(UDC_DWC3_GRXTHRCFG_USBRXPKTCNT_MASK, reg));

#if UDC_DWC3_RX_THRESHOLD_WORKAROUND
	if ((reg & UDC_DWC3_GRXTHRCFG_USBRXPKTCNTSEL) != 0) {
		LOG_WRN("clearing GRXTHRCFG.UsbRxPktCntSel (databook 1.2.4 erratum)");
		sys_clear_bits(base + UDC_DWC3_GRXTHRCFG,
			       UDC_DWC3_GRXTHRCFG_USBRXPKTCNTSEL);
	}
#endif

	/* Letting GTXTHRCFG unchanged - the erratum above is RX-side only */

	/* Read the chip identification */
	reg = sys_read32(base + UDC_DWC3_GCOREID);
	LOG_INF("event: coreid=0x%04lx rel=0x%04lx",
		FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg),
		FIELD_GET(UDC_DWC3_GCOREID_REL_MASK, reg));
	__ASSERT_NO_MSG(FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg) == 0x5533);

	/* Letting GUID unchanged */
	/*
	 * "The PHY must not be enabled for auto-resume in device mode. Therefore,
	 * the field GUSB2PHYCFG[15] (ULPIAutoRes) must be written with '0' during
	 * the power-on initialization in case the reset value is '1'" (databook
	 * Table 4-1, Power-On or Soft Reset Register Initialization).
	 *
	 * One bit, and inert if the core already powers up with it clear - but the
	 * spec explicitly says not to rely on that, and the reset value can differ
	 * between configurations. With it set the ULPI PHY auto-resumes without the
	 * driver's involvement, so the link can leave suspend in a state the driver
	 * never learns about.
	 *
	 * Everything else in GUSB2PHYCFG (USBTrdTim, FSIntf, PHYIf, TOUTCal) and all
	 * of GUSB3PIPECTL stay at their coreConsultant power-on values, which the
	 * same table permits.
	 */
	sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG,
		       UDC_DWC3_GUSB2PHYCFG_ULPIAUTORES);
	/*
	 * One-shot FIFO map. GRXFIFOSIZ0 and GTXFIFOSIZn partition a pool whose
	 * total is fixed at synthesis (GHWPARAMS7.RAM1_DEPTH); the split itself is
	 * R/W. Printing it once at init is the only way to know whether the RX
	 * side can be enlarged out of TX slack, or whether the pool is already
	 * fully committed.
	 */
	{
		const uint32_t hp7 = sys_read32(base + UDC_DWC3_GHWPARAMS7);
		const uint32_t ram1 = FIELD_GET(UDC_DWC3_GHWPARAMS7_RAM1_DEPTH_MASK, hp7);
		const uint32_t rx = sys_read32(base + UDC_DWC3_GRXFIFOSIZ(0));
		const uint32_t mdw = (sys_read32(base + UDC_DWC3_GHWPARAMS0) >> 8) & 0xFFU;
		uint32_t used = 0;

		LOG_INF("FIFOMAP: RAM1_DEPTH=%u words RAM2_DEPTH=%u words "
			"(mdwidth=%u; %u / %u bytes)",
			ram1,
			(uint32_t)FIELD_GET(UDC_DWC3_GHWPARAMS7_RAM2_DEPTH_MASK, hp7),
			mdw, ram1 * mdw / BITS_PER_BYTE,
			(uint32_t)FIELD_GET(UDC_DWC3_GHWPARAMS7_RAM2_DEPTH_MASK, hp7) *
				mdw / BITS_PER_BYTE);
		LOG_INF("FIFOMAP: RX0 start=%u depth=%u (%u B)",
			(uint32_t)FIELD_GET(UDC_DWC3_GRXFIFOSIZ_RXFSTADDR_MASK, rx),
			(uint32_t)FIELD_GET(UDC_DWC3_GRXFIFOSIZ_RXFDEP_MASK, rx),
			(uint32_t)FIELD_GET(UDC_DWC3_GRXFIFOSIZ_RXFDEP_MASK, rx) * mdw /
				BITS_PER_BYTE);
		/*
		 * TX only. RX0 lives in RAM2, a separate address space (hence the
		 * separate RAM1_DEPTH/RAM2_DEPTH above), so counting it against the
		 * TX budget understates the spare by exactly the RX depth - which
		 * once turned a healthy map into an apparent 483-word overflow.
		 */
		used = 0U;

		for (uint32_t i = 0; i < 8U; i++) {
			const uint32_t tx = sys_read32(base + UDC_DWC3_GTXFIFOSIZ(i));
			const uint32_t dep =
				FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK, tx);

			if (dep == 0U) {
				continue;
			}
			LOG_INF("FIFOMAP: TX%u start=%u depth=%u (%u B)", i,
				(uint32_t)FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFSTADDR_MASK, tx),
				dep, dep * mdw / BITS_PER_BYTE);
			used += dep;
		}

		LOG_INF("FIFOMAP: TX total=%u of RAM1 %u words -> %d spare (%d B); "
			"RX0 %u of RAM2 %u words",
			used, ram1, (int)ram1 - (int)used,
			((int)ram1 - (int)used) * (int)mdw / 8,
			(uint32_t)FIELD_GET(UDC_DWC3_GRXFIFOSIZ_RXFDEP_MASK, rx),
			(uint32_t)FIELD_GET(UDC_DWC3_GHWPARAMS7_RAM2_DEPTH_MASK, hp7));
	}

	/* Letting GRXFIFOSIZ unchanged */

	/* Setup the event buffer address, size and start event reception */
	memset((void *)cfg->evt_buf, 0, CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t));

	/*
	 * The read pointer belongs with that memset and must never be separated from it.
	 *
	 * CSftRst clears all CSRs except GSTS, GSNPSID, GGPIO, GUID, GUSB2PHYCFGn,
	 * GUSB3PIPECTLn, DCFG, DCTL, DEVTEN and DSTS.  GEVNTADR/SIZ/COUNT are absent from
	 * that list, which is why the lines below reprogram them - and it means the
	 * controller's write pointer restarts at slot 0, so anything the driver believed
	 * about its read position is now wrong.
	 *
	 * Leaving it stale does not lose an event, it HANGS the ring: the drain parks on
	 * evt_next, which the controller will not reach again until it has written every
	 * slot ahead of it, while GEVNTCOUNT reports events piling up at slot 0 that
	 * nothing collects - the empty-slot give-up, arrived at by way of a bug.
	 *
	 * udc_dwc3_init() is a UDC API entry point, not boot-only, so this is reachable on
	 * any re-init.  The give-up bookkeeping goes with it: every field names a slot index
	 * or an age that no longer refers to anything.
	 */
	priv->evt_next = 0;
	priv->evt_gaveup_run = 0;
	priv->evt_gaveup_slot = 0;
	priv->evt_gaveup_logged = false;
	priv->evt_drain_gaveup = false;
	priv->evt_drain_midzero = false;
	/*
	 * Prime every slot before the controller is told where the buffer is.
	 *
	 * This is the whole ring's initial state, and it MUST be done here rather
	 * than relying on priv/cfg starting zeroed: the sentinel is no longer zero,
	 * so an unprimed ring reads as sixteen waiting events and the first drain
	 * would dispatch garbage. It has to happen before GEVNTADR is programmed,
	 * because from that moment the controller may write.
	 */
	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_EVENTS_NUM; i++) {
		cfg->evt_buf[i] = UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
	}

	/*
	 * Commit the priming before the controller is told where the buffer is.
	 *
	 * Those sixteen stores go to the NOCACHE/AXI region and are POSTED: the
	 * fabric accepts them and completes them later. The moment GEVNTADR and
	 * GEVNTSIZ are programmed the controller may start writing events, so a
	 * sentinel store still in flight can land ON TOP of a real event and
	 * erase it - and the result would read as a free slot, indistinguishable
	 * from a write that never happened.
	 *
	 * This is the same hazard udc_dwc3_trb_sync() exists to prevent on the
	 * descriptor path, reintroduced here when the free marker stopped being
	 * whatever the buffer already held and became something we write. The
	 * remedy is the same: read the LAST word written back, which cannot be
	 * answered until the stores ahead of it have drained.
	 */
	udc_dwc3_trb_sync(&cfg->evt_buf[CONFIG_UDC_DWC3_EVENTS_NUM - 1]);

	sys_write32(HI32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_HI(0));
	sys_write32(LO32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_LO(0));
	sys_write32(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t), base + UDC_DWC3_GEVNTSIZ(0));
	LOG_INF("Event buffer size is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));

	/*
	 * Report the address and whether it actually satisfies the size-alignment
	 * rule. The build asserts above cannot check this on their own - alignment
	 * is requested from the linker, and this is the confirmation that it was
	 * honoured for the region the buffer landed in.
	 */
	if (((uintptr_t)cfg->evt_buf &
	     (CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) - 1)) != 0) {
		LOG_ERR("event buffer at %p is NOT aligned to its size (%u bytes): "
			"the controller's wrap will not match this driver's",
			(void *)cfg->evt_buf,
			(unsigned int)(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t)));
	} else {
		LOG_INF("Event buffer at %p, size-aligned", (void *)cfg->evt_buf);
	}

	/* Last step: writing 0 here is what enables the event buffer. */
	sys_write32(0, base + UDC_DWC3_GEVNTCOUNT(0));

	reg = sys_read32(base + UDC_DWC3_GUCTL2);
	reg |= UDC_DWC3_GUCTL2_RST_ACTBITLATER;
	sys_write32(reg, base + UDC_DWC3_GUCTL2);

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
	reg |= FIELD_PREP(UDC_DWC3_DCFG_NUMP_MASK, 1);
	sys_write32(reg, base + UDC_DWC3_DCFG);

	/*
	 * Enable reception of all USB events except ULSTCNGEN, which is a
	 * diagnostic-only class - see below.
	 */
	reg = UDC_DWC3_DEVTEN_INACTTIMEOUTRCVEDEN;
	/*
	 * VNDRDEVTSTRCVED IS DELIBERATELY NOT ENABLED.
	 *
	 * It is the one event that is not four bytes: "The Vendor Device Test LMP
	 * Received Event (VndrDevTstRcved) is a 12-byte event that includes a
	 * header in the first four bytes and the contents of the LMP in the
	 * following eight bytes." This drain consumes one slot per event and uses
	 * a zero word as its "not written yet" sentinel, so the two payload words
	 * would be read as events - and if either is legitimately 0x00000000 the
	 * drain waits for ever on a slot that already holds its final value, while
	 * the controller's write pointer has moved past it. That is an
	 * unrecoverable wedge indistinguishable from a lost write.
	 *
	 * The event is ignored when it arrives, so enabling it bought nothing, and
	 * the databook says not to use the feature anyway: "do not use Vendor
	 * Device Test feature when there is normal traffic on the USB." Leaving it
	 * disabled makes the zero sentinel sound by construction: with no
	 * multi-word event in the ring, no legitimate event word can be zero.
	 */
	reg |= UDC_DWC3_DEVTEN_EVNTOVERFLOWEN;
	reg |= UDC_DWC3_DEVTEN_CMDCMPLTEN;
	reg |= UDC_DWC3_DEVTEN_ERRTICERREN;
	reg |= UDC_DWC3_DEVTEN_HIBERNATIONREQEVTEN;
	reg |= UDC_DWC3_DEVTEN_WKUPEVTEN;
	/*
	 * Link state change events are OFF by default, and this is a diagnostic
	 * switch rather than a functional one.
	 *
	 * Stock does not enable them. They were turned on here to see link
	 * transitions, and nothing in the dispatch acts on them - the value only
	 * ever reached a log line. uart_v6_3 shows the cost of that: 1450 of them
	 * in the last 45 seconds of the run, every one reporting U0 while DSTS read
	 * back U0, SuperSpeed, DEVCTRLHLT=0 and a SOFFN advancing at 93-96%% of
	 * nominal throughout. The link never changed state. Each of those events
	 * still took a slot in a 16-entry ring that was already failing to drain,
	 * and a full ring makes the controller withhold link credits and stop the
	 * bus - so a purely cosmetic event class was feeding the failure.
	 *
	 * Enable it deliberately when link behaviour is the thing under
	 * investigation, and expect the event rate to be part of what you measure.
	 */
#ifdef CONFIG_UDC_DWC3_LINK_STATE_EVENTS
	reg |= UDC_DWC3_DEVTEN_ULSTCNGEN;
#endif
	reg |= UDC_DWC3_DEVTEN_CONNECTDONEEN;
	reg |= UDC_DWC3_DEVTEN_USBRSTEN;
	reg |= UDC_DWC3_DEVTEN_DISCONNEVTEN;
	sys_write32(reg, base + UDC_DWC3_DEVTEN);

	/* Configure control endpoints */
	udc_dwc3_depcmd_start_config(dev, true);
}

static void udc_dwc3_on_usb_reset(const struct device *const dev)
{
	LOG_DBG("Going through DWC3 reset logic");

	udc_dwc3_drop_xfer_state(dev, "USB reset");

	/* TODO: wait that all transfers did complete (if needed) */

	/* Perform the USB reset operations manually to improve latency */
	/* TODO: do after endpoints are configured? */
	udc_dwc3_set_address(dev, 0);
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
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0], true);
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_out[0], true);

	/*
	 * GTXFIFOSIZn is deliberately left alone, and the databook is explicit that
	 * this is the normal case: the defaults are "assigned in coreConsultant
	 * based on the maximum packet size, number of packets to be buffered, speed
	 * of host bus instance, bus latency, and mode of operation", and "upon reset
	 * and mode transitions, hardware automatically programs these registers to
	 * the default values. Consequently, there is typically no need for the
	 * software to modify the pre-defined default values."
	 *
	 * Table 4-1 lists GTXFIFOSIZn conditionally - "unless the packet sizes" of
	 * this configuration differ from what the defaults assume - not as an
	 * unconditional power-on write. Recomputing an allocation here would need
	 * the RAM depth the RTL was built with, which is not something this driver
	 * can discover, and getting it wrong breaks transmit outright.
	 *
	 * GTXFIFOPRIDEV is left alone for the same kind of reason. Its bits select
	 * high (1) or low (0) DMA priority per IN-endpoint TxFIFO, and low-priority
	 * FIFOs are served round-robin only once the high-priority ones have
	 * nothing left to do. The reset value is all zero, which is every endpoint
	 * at equal priority in round-robin - the fair arrangement. Writing it would
	 * create starvation between the video and ACM endpoints rather than remove
	 * it, so it stays unwritten unless a capture argues otherwise.
	 */

	/* After successful speed negotiation, DWC3 sends a CONNECT_DONE event.
	 * Then only the speed-related registers are populated, and we can
	 * report the "reset" event (instead of during USB_RESET).
	 */
	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static void udc_dwc3_on_set_config_or_interface(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	LOG_DBG("SetConfiguration or SetInterface extra init");

	for (int i = 1; i < cfg->num_in_eps; i++) {
		if (udc_ep_is_busy(&cfg->ep_data_in[i].cfg)) {
			udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_in[i], 0);
		}
	}
	for (int i = 1; i < cfg->num_out_eps; i++) {
		if (udc_ep_is_busy(&cfg->ep_data_out[i].cfg)) {
			udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_out[i], 0);
		}
	}

	/* To trigger a reconfiguration of the TX FIFO */
	udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_in[0], UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0], true);

	/* Re-initialize resources IDs for non-control endpoints */
	udc_dwc3_depcmd_start_config(dev, false);
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
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t trb_trbctl = ep_data->trb_cache[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	struct net_buf *buf;

	/*
	 * A completion left over from a transfer that has already been replaced.
	 * Discard it: the buffer it belonged to is gone, and the TRB now in the
	 * ring belongs to the transfer that replaced it.
	 */
	if (ep_data->stale_completion) {
		ep_data->stale_completion = false;
		LOG_DBG("discarding superseded completion on EP%02x",
			ep_data->cfg.addr);
		return;
	}

	if (udc_dwc3_ctrl_setup_pending(dev, ep_data)) {
		udc_dwc3_ctrl_abandon(dev, ep_data);
		return;
	}

	buf = udc_buf_get(&ep_data->cfg);
	if (buf == NULL) {
		LOG_ERR("Missing buffer submitted for EP%02X", ep_data->cfg.addr);
		/*
		 * Release the endpoint before leaving. udc_dwc3_ctrl_try() refuses to
		 * arm while EITHER control endpoint is busy, so returning with the flag
		 * still set wedges both of them until the watchdog intervenes. The
		 * transfer this completion belonged to is gone either way.
		 */
		udc_ep_set_busy(&ep_data->cfg, false);
		udc_dwc3_ctrl_next(dev);
		return;
	}

	LOG_DBG("%u:%u:%u",
		udc_get_buf_info(buf)->setup,
		udc_get_buf_info(buf)->data,
		udc_get_buf_info(buf)->status);


	if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
	    trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
		buf->len = 0;
		priv->ctrl_status_done++;
		LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL STATUS packet sent");
		//atomic_set_bit(&priv->expected_xfer, UDC_DWC3_CTRL_SETUP);
	} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
		LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL DATA packet sent");
		/* 4.4.2 step 5 needs to know the data stage is behind us. */
		priv->ctrl_data_done = true;
	} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		LOG_ERR("Unexpected SETUP IN packet");
	} else {
		LOG_ERR("Unexpected IN packet type: 0x%x", trb_trbctl);
	}

	memset(&ep_data->trb_buf[0], 0x00, sizeof(ep_data->trb_buf[0]));
	memset(&ep_data->trb_cache[0], 0x00, sizeof(ep_data->trb_cache[0]));

	udc_submit_ep_event(dev, buf, 0);

	/* Used when receiving a completed buffer from the hardware: mark as free */
	udc_ep_set_busy(&ep_data->cfg, false);

	udc_dwc3_ctrl_next(dev);
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
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t trb_trbctl = ep_data->trb_cache[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	struct net_buf *buf;

	/*
	 * A completion left over from a transfer that has already been replaced.
	 * Discard it: the buffer it belonged to is gone, and the TRB now in the
	 * ring belongs to the transfer that replaced it.
	 */
	if (ep_data->stale_completion) {
		ep_data->stale_completion = false;
		LOG_DBG("discarding superseded completion on EP%02x",
			ep_data->cfg.addr);
		return;
	}


	/*
	 * A new SETUP can arrive during any stage, not only the status-IN one. If
	 * it lands while an OUT data or status stage is outstanding, the retired
	 * TRB is on this endpoint and the abort has to be caught here too -
	 * otherwise the stale trb_cache classification below would report the
	 * abandoned stage as a normal completion and arm the next stage of a
	 * transfer the host has already left.
	 */
	if (udc_dwc3_ctrl_setup_pending(dev, ep_data)) {
		udc_dwc3_ctrl_abandon(dev, ep_data);
		return;
	}

	if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		buf = udc_buf_peek(&ep_data->cfg);
		if (buf == NULL) {
			LOG_ERR("Missing buffer for EP%02X", ep_data->cfg.addr);
			/*
			 * Release the endpoint before leaving. udc_dwc3_ctrl_try()
			 * refuses to arm while EITHER control endpoint is busy, so
			 * returning with the flag still set wedges both of them until
			 * the watchdog intervenes. The transfer this completion
			 * belonged to is gone either way, so the claim must go with it.
			 */
			udc_ep_set_busy(&ep_data->cfg, false);
			udc_dwc3_ctrl_next(dev);
			return;
		}

		/* Update the size to the setup packet size */
		if (buf->size < sizeof(priv->setup_packet)) {
			LOG_ERR("Invalid size for setup packet buffer: %u", buf->size);
			udc_submit_ep_event(dev, buf, -ENOBUFS);
			/*
			 * Release the endpoint before leaving. udc_dwc3_ctrl_try()
			 * refuses to arm while EITHER control endpoint is busy, so
			 * returning with the flag still set wedges both of them until
			 * the watchdog intervenes. The transfer this completion
			 * belonged to is gone either way, so the claim must go with it.
			 */
			udc_ep_set_busy(&ep_data->cfg, false);
			udc_dwc3_ctrl_next(dev);
			return;
		}

		memcpy(&priv->setup_packet, buf->data, sizeof(priv->setup_packet));

		/*
		 * Step 2 has happened: the SETUP retired and setup_packet describes
		 * the request now in progress. Until this point any XferNotReady for
		 * a data or status stage belongs to a transfer that is already over.
		 */
		priv->ctrl_setup_seen = true;
		priv->ctrl_setup_done++;
		buf->len = 0;

		/*
		 * Stamp here, not in udc_dwc3_on_ctrl_in(): a SETUP retires on EP0-OUT.
		 * This is the moment the request becomes known to the driver, and the
		 * clock against which "the stack never came back with a data buffer"
		 * has to be measured.
		 */
		priv->ctrl_setup_up_t = k_cycle_get_32();
		priv->ctrl_setup_up_n++;

		/* Latency optimization: set the address immediately to be able to be able
		 * to ACK/NAK the first packets from the host with the new address,
		 * otherwise the host issue a reset.
		 */
		if (priv->setup_packet.bmRequestType == USB_REQTYPE_TYPE_STANDARD &&
		    priv->setup_packet.bRequest == USB_SREQ_SET_ADDRESS) {
			udc_dwc3_set_address(dev, sys_le16_to_cpu(priv->setup_packet.wValue));
		}

		/*
		 * The whole SETUP as one 16-digit value instead of a hexdump: one line
		 * rather than two, ~37 bytes less per SETUP, and still fully analysable
		 * - every field is at a fixed digit offset.
		 *
		 * Assembled big-endian on purpose so the digits read left to right in
		 * WIRE order: bmRequestType, bRequest, wValue, wIndex, wLength. Loading
		 * the struct as a uint64_t directly would print it byte-reversed on this
		 * little-endian core, and would type-pun a __packed struct that carries
		 * no alignment guarantee. Byte access avoids both.
		 *
		 * %%llx needs CONFIG_CBPRINTF_FULL_INTEGRAL (set here). Two %%08x halves
		 * print identically if that ever goes away.
		 */
		const uint8_t *const sp = (const uint8_t *)&priv->setup_packet;

		/*
		 * The one line a healthy control transfer prints, and the only context
		 * an error line needs: the eight setup bytes identify the request that
		 * was in flight. Everything the stages used to narrate is derivable
		 * from it, at a tenth of the console cost.
		 */
#ifdef UDC_DWC3_LOG_EVERY_SETUP
		LOG_INF("SETUP %016llx",
			((uint64_t)sp[0] << 56) | ((uint64_t)sp[1] << 48) |
			((uint64_t)sp[2] << 40) | ((uint64_t)sp[3] << 32) |
			((uint64_t)sp[4] << 24) | ((uint64_t)sp[5] << 16) |
			((uint64_t)sp[6] << 8)  |  (uint64_t)sp[7]);
#else
		(void)sp;
#endif
		udc_setup_received(dev, &priv->setup_packet);

		/*
		 * udc_setup_received() has just invalidated whatever the IN endpoint
		 * was doing - it drains that queue and releases the endpoint - but the
		 * controller has already retired that TRB and its completion is still
		 * queued behind this event. Mark it so the handler discards it, and
		 * clear the writeback so nothing else reads the abort out of a ring
		 * that is about to be re-armed for the replacement transfer.
		 *
		 * Without this the late completion is matched against the newly armed
		 * TRB and the replacement transfer's buffer is reported complete before
		 * the host has seen it.
		 */
		if (udc_dwc3_ctrl_setup_pending(dev, &cfg->ep_data_in[0])) {
			cfg->ep_data_in[0].stale_completion = true;
			memset((void *)&cfg->ep_data_in[0].trb_buf[0], 0x00,
			       sizeof(cfg->ep_data_in[0].trb_buf[0]));
			memset((void *)&cfg->ep_data_in[0].trb_buf[1], 0x00,
			       sizeof(cfg->ep_data_in[0].trb_buf[1]));
			/*
			 * ep_data_in[0], not ep_data.  This runs in
			 * udc_dwc3_on_ctrl_out(), so ep_data is the OUT endpoint;
			 * clearing its cache here left the IN cache stale - the one
			 * whose TRBs the three memsets above just discarded - and
			 * wrongly discarded the OUT endpoint's.
			 */
			memset(&cfg->ep_data_in[0].trb_cache[0], 0x00,
			       sizeof(cfg->ep_data_in[0].trb_cache[0]));
		}
	} else {
		buf = udc_buf_get(&ep_data->cfg);
		if (buf == NULL) {
			LOG_ERR("Missing buffer for EP%02X", ep_data->cfg.addr);
			udc_ep_set_busy(&ep_data->cfg, false);
			udc_dwc3_ctrl_next(dev);
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
			return;
		}

		/*
		 * What the hardware actually received, not what the host declared.
		 *
		 * The old code assigned setup_packet.wLength - the DECLARED length -
		 * under a comment claiming it was what the hardware reported.  A host is
		 * free to send a short data stage, and every such transfer was handed
		 * upstream as if the full wLength had arrived, with stale bytes beyond
		 * the real data counted as valid.
		 *
		 * The controller decrements BUFSIZ in the live TRB as it fills the
		 * buffer, so received = programmed - residual, using the same rounding
		 * udc_dwc3_trb_ctrl_out() applied when it armed the descriptor.  The
		 * live TRB is still intact here - it is not cleared until the end of
		 * this function.  Clamped to both the buffer and the declared length, so
		 * a host that overruns either cannot inflate buf->len.
		 */
		if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
			const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);
			const uint32_t programmed =
				(mps != 0U) ? ROUND_UP(buf->size, mps) : buf->size;
			const uint32_t residual = FIELD_GET(
				UDC_DWC3_TRB_STATUS_BUFSIZ_MASK,
				ep_data->trb_buf[0].status);
			const uint32_t received =
				(programmed > residual) ? (programmed - residual) : 0U;

			buf->len = MIN(received,
				       MIN((uint32_t)buf->size,
					   (uint32_t)priv->setup_packet.wLength));
		} else {
			buf->len = priv->setup_packet.wLength;
		}

		if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL DATA received");
			/* 4.4.2 step 5 needs to know the data stage is behind us. */
			priv->ctrl_data_done = true;
		} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 ||
			   trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2) {
			/*
			 * STATUS_2 is accepted here as well as STATUS_3.
			 *
			 * udc_dwc3_ctrl_next_out() only ever arms STATUS_3 - a two-stage
			 * transfer's status is IN - so STATUS_2 should never complete on
			 * this endpoint.  But udc_dwc3_ctrl_rearm() re-arms whatever
			 * watchdog_type held, verbatim, and its switch has a STATUS_2 arm,
			 * so the combination is constructible.  It used to land in the
			 * error branch below, which logs and then falls through having
			 * neither zeroed the length nor counted the stage - a status stage
			 * silently reported as a data one.
			 *
			 * Both are status stages and both want identical handling, so treat
			 * them the same rather than leave the trap armed.
			 */
			buf->len = 0;
			priv->ctrl_status_done++;
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL STATUS received");
		} else {
			LOG_ERR("Unexpected OUT packet type: 0x%x", trb_trbctl);
		}

		udc_submit_ep_event(dev, buf, 0);
	}

	memset(&ep_data->trb_buf[0], 0x00, sizeof(ep_data->trb_buf[0]));

	/*
	 * Defensive: slot 1 is not armed on this endpoint any more.
	 *
	 * It was, when the control OUT data stage chained an alignment descriptor
	 * there; that was replaced by a plain round-up of slot 0, so nothing writes
	 * slot 1 on the OUT control endpoint today.  The clear is kept so a stale
	 * descriptor can never outlive a stage, and costs one memset per control
	 * transfer.
	 */
	memset(&ep_data->trb_buf[1], 0x00, sizeof(ep_data->trb_buf[1]));
	memset(&ep_data->trb_cache[0], 0x00, sizeof(ep_data->trb_cache[0]));

	/* Used when receiving a completed buffer from the hardware: mark as free */
	udc_ep_set_busy(&ep_data->cfg, false);

	udc_dwc3_ctrl_next(dev);
}

/*
 * Dispatch a control completion to the handler for the endpoint it came from.
 *
 * The endpoint number is taken from the event, not from priv->last_xfer_dir.
 * That shortcut was safe only while udc_dwc3_ctrl_try() guaranteed a single
 * control TRB in flight; now that a SETUP can be armed on the OUT endpoint
 * while a status stage is still outstanding on the IN one - which is what
 * breaks the abandoned-transfer deadlock - two control TRBs can be live at
 * once, and last_xfer_dir names whichever was armed most recently rather than
 * whichever just completed.
 *
 * Every other handler in this driver already derives its endpoint this way.
 */
static void udc_dwc3_on_ctrl(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *const completed =
		((epn & 1U) != 0U) ? &cfg->ep_data_in[0] : &cfg->ep_data_out[0];

	/*
	 * Cancel the watchdog only when the endpoint it guards is the one that
	 * just completed. A SETUP can be armed on EP0-OUT while the status stage
	 * is still outstanding on EP0-IN, so both control endpoints can own a
	 * live stage at once, and a completion on one must not disarm the
	 * watchdog that is still guarding the other.
	 *
	 * This was the terminal wedge in uart_01sep_0336_xnrdy_s01: the IN
	 * status completion cancelled the OUT SETUP watchdog, and the following
	 * udc_dwc3_ctrl_next() found EP0-OUT already busy with the speculative
	 * SETUP and declined without re-arming it. The host's next SETUP was
	 * then accepted into the RxFIFO but never retired, with no watchdog
	 * pending to recover it.
	 */
	if (priv->watchdog_ep == completed) {
		k_work_cancel_delayable(&priv->watchdog_dwork);

		/*
		 * Forget what the watchdog was guarding as well as cancelling it:
		 * the cancel does not stop a handler that has already started, and a
		 * late handler must not act on a stage that has just completed.
		 *
		 * Both fields, not just the pointer. udc_dwc3_watchdog_worker()
		 * selects its SETUP path on watchdog_type alone and never reads
		 * watchdog_ep, and udc_dwc3_recover() does not idle on a NULL
		 * pointer either - it falls back to whichever control endpoint is
		 * busy. Clearing the pointer alone would leave a stale CONTROL_SETUP
		 * type to drive both.
		 */
		priv->watchdog_ep = NULL;
		priv->watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
	}

	/* Physical endpoint 0 is control OUT, 1 is control IN. */
	if ((epn & 1U) != 0U) {
		udc_dwc3_on_ctrl_in(dev);
	} else {
		udc_dwc3_on_ctrl_out(dev);
	}
}

/*
 * SPEC, Programming Guide 3.30b, Table 3-2 Device Generic Command Types,
 * command 09h "Selected FIFO Flush":
 *
 *   "Parameter[4:0] = FIFO Number
 *    Paramer[5] = '1' for TX FIFO or '0' for RX FIFO"
 *
 * Required after an aborted control IN transfer. Section 4.4.2 step 8, on a
 * SETUP arriving mid-transfer: "Software has to reclaim the TRBs with HWO=1 in
 * the skipped TRBs and flush the TxFIFO." Reclaiming alone is not enough - the
 * bytes the controller had already staged for the skipped IN stage stay in the
 * FIFO and would be transmitted at the head of the next one.
 *
 * The FIFO number is the one programmed into DEPCFG for this endpoint, which
 * udc_dwc3_depcmd_ep_config() sets to the endpoint number for IN endpoints, so
 * control IN uses TxFIFO 0.
 */
/*
 * Wait for DGCMD.CmdAct to clear, bounded.
 *
 * SPEC, Programming Guide 3.30b, DGCMD bit 10 CMDACT: "The software sets this
 * bit to 1 to enable the device controller to execute the generic command. The
 * device controller sets this bit to 0 after executing the command."
 *
 * Needed on the way IN as well as on the way out, because this is not the only
 * generic command the driver issues: udc_dwc3_evt_force() writes one and
 * deliberately does not wait for it. Both run on the work queue, so a force
 * issued while draining the event ring can still be in flight when a control
 * abort in the same pass reaches the flush below. Refusing to flush in that
 * case would drop a spec obligation precisely when the driver is already
 * recovering from something; waiting costs microseconds.
 */
static bool udc_dwc3_dgcmd_wait_idle(const mm_reg_t base)
{
	uint32_t polls = 0;

	while ((sys_read32(base + UDC_DWC3_DGCMD) & UDC_DWC3_DGCMD_ACT) != 0U) {
		if (++polls >= UDC_DWC3_DGCMD_POLL_MAX) {
			return false;
		}
		k_busy_wait(1);
	}

	return true;
}

static void udc_dwc3_fifo_flush_tx(const struct device *const dev, const uint8_t fifo)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	if (!udc_dwc3_dgcmd_wait_idle(base)) {
		LOG_ERR_RATELIMIT("a generic command stayed active for %u us; TxFIFO "
				  "%u not flushed", UDC_DWC3_DGCMD_POLL_MAX, fifo);
		return;
	}

	sys_write32(UDC_DWC3_DGCMD_FIFOFLUSH_TX |
		    FIELD_PREP(UDC_DWC3_DGCMD_FIFOFLUSH_NUM_MASK, fifo),
		    base + UDC_DWC3_DGCMDPAR);
	sys_write32(UDC_DWC3_DGCMD_FIFOFLUSHONE | UDC_DWC3_DGCMD_ACT,
		    base + UDC_DWC3_DGCMD);

	/*
	 * Wait for it. The next control stage is armed immediately after this
	 * returns, and arming it while the flush is still in flight would race the
	 * very bytes being discarded.
	 */
	if (!udc_dwc3_dgcmd_wait_idle(base)) {
		LOG_ERR_RATELIMIT("TxFIFO %u flush did not complete in %u us", fifo,
				  UDC_DWC3_DGCMD_POLL_MAX);
	}
}

/*
 * Put the control pair back on Step 1 after the host and the device have been
 * found to disagree about which stage is current.
 *
 * SPEC, Programming Guide 3.30b, sections 4.4.1 and 4.4.2. Both models answer
 * every one of their error cases the same way: "issue Set Stall on EP0 and go
 * back to Step 1", Step 1 being "Software sets up a Setup TRB and issues Start
 * Transfer on EP0 pointing to the Setup TRB". Section 4.4 is explicit about
 * which endpoint carries it: "Set STALL is always issued on EP0", the OUT
 * direction, whichever direction the offending event arrived on.
 *
 * end_ep is non-NULL only where the model asks for a transfer to be retired
 * before the stall - section 4.4.2 step 3a, wrong-direction data: "software
 * must issue an End Transfer for the data stage it has already started, then
 * issue Set Stall".
 *
 * A stall here is not a failure being reported upward. It is the recovery: the
 * host sees the stall, abandons the request it was mid-way through, and starts
 * a fresh SETUP - which section 4.4.2 step 12 guarantees will be taken, since
 * "The STALL bit will be cleared by the controller whenever it receives a SETUP
 * packet. SETUPs are always accepted."
 */
/* How many control-stage mismatches to name before going quiet. */
#define UDC_DWC3_CTRL_DESYNC_LOG_FIRST				12u

/* How many non-SETUP watchdog fires to describe before going quiet. */
#define UDC_DWC3_CTRL_WD_DUMP_FIRST				8u

static void udc_dwc3_ctrl_resync(const struct device *const dev,
				 const char *const why)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	/*
	 * REPORT ONLY - deliberately no recovery action.
	 *
	 * The flag-driven tests in udc_dwc3_ctrl_xnr_check() (ctrl_setup_seen, 4.4.x
	 * step 2; ctrl_data_done, 4.4.2 step 5) cannot be trusted here, because this
	 * driver arms AHEAD of the host: the next SETUP speculatively, the status stage
	 * before the host has finished.  That leaves a window where setup_packet still
	 * describes a transfer the host is legitimately working through while the flags
	 * say none is in progress, and every XferNotReady in it trips the check.
	 *
	 * Measured: a cold boot logged ten "SETUP has not retired" plus one "more data
	 * than wLength" during ordinary enumeration.  With recovery enabled each issued
	 * Set Stall on EP0 and an End Transfer against a live control endpoint, which
	 * destroyed enumeration and took the host controller down - 0/3 boots.  Made
	 * report-only, the same build passed 3/3.
	 *
	 * Acting on a signal that fires during healthy traffic is worse than not acting.
	 * Restoring recovery needs a way to know a request is genuinely current, which
	 * the arming events cannot provide.
	 *
	 * Plain LOG_ERR for the first few, NOT LOG_ERR_RATELIMIT: enumeration happens
	 * inside the first CONFIG_LOG_RATELIMIT_INTERVAL_MS (5000 ms), where the limiter
	 * has no previous emission to compare against and swallows the message entirely.
	 */
	priv->ctrl_desync++;

	if (priv->ctrl_desync <= UDC_DWC3_CTRL_DESYNC_LOG_FIRST) {
		LOG_ERR("control stage mismatch #%u (reported, not acted on): %s",
			priv->ctrl_desync, why);
	}
}

/*
 * Check a control XferNotReady against the stage the programming model says is
 * current, and recover if the host is somewhere else.
 *
 * This is the closed loop. The event carries the stage the HOST wants, in
 * DEPEVT status bits 13:12; without comparing it against the device's own
 * position the driver can only ever replay its own prediction, which is right
 * until the first time the host does something the model calls an error - and
 * then wrong for every transfer afterwards, with nothing able to resynchronise
 * it. Each case below is one of those errors, in the order the models list
 * them.
 *
 * Returns true when the event is legitimate and the caller should proceed.
 */
static bool udc_dwc3_ctrl_xnr_check(const struct device *const dev,
				    const uint32_t evt, const bool is_in)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t stage = evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK;
	const struct usb_setup_packet *const setup = &priv->setup_packet;
	const bool wants_in = setup->RequestType.direction == USB_REQTYPE_DIR_TO_HOST;

	/*
	 * 4.4.1 and 4.4.2, step 2: "If a XferNotReady (Data/Status) event is
	 * received before the XferComplete event for the Setup stage, issue Set
	 * Stall. This is an error case where the host is attempting to move data or
	 * start the status stage for a previous control transfer that has already
	 * completed."
	 */
	if (!priv->ctrl_setup_seen) {
		udc_dwc3_ctrl_resync(dev,
			"XferNotReady for a stage of a request whose SETUP has not "
			"retired");
		return false;
	}

	/*
	 * A status request here is legitimate and needs no further checking. Only
	 * the encoding the databook actually defines counts as one: of the four
	 * values bits 13:12 can hold, 2'b01 and 2'b10 are the whole definition,
	 * 2'b00 was rejected by the caller as an impossible SETUP request, and 2'b11
	 * means nothing. Letting an undefined encoding through as "not Data,
	 * therefore Status" would take the one branch that skips every check below.
	 */
	if (stage == UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS) {
		return true;
	}

	if (stage != UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA) {
		udc_dwc3_ctrl_resync(dev,
			"XferNotReady carried a control stage encoding the databook "
			"does not define");
		return false;
	}

	/*
	 * 4.4.1 step 3: "If an XferNotReady event for the Data stage is received
	 * (either direction), issue Set Stall on EP0 and go back to Step 1. This is
	 * an error case where the host is attempting to start the data stage when
	 * the setup bytes did not indicate a data stage was present."
	 */
	if (sys_le16_to_cpu(setup->wLength) == 0U) {
		udc_dwc3_ctrl_resync(dev,
			"host started a data stage on a request that declared "
			"wLength 0");
		return false;
	}

	/*
	 * 4.4.2 step 3a: "If an XferNotReady (Data) event is received for the
	 * incorrect direction, software must issue an End Transfer for the data
	 * stage it has already started, then issue Set Stall. This is an error case
	 * where the host is attempting to move data in the wrong direction."
	 */
	if (is_in != wants_in) {
		udc_dwc3_ctrl_resync(dev,
			"host started the data stage in the direction opposite to "
			"bmRequestType");
		return false;
	}

	/*
	 * 4.4.2 step 5, a data XferNotReady arriving after the data stage already
	 * retired. The model gives it two readings, separated by whether wLength
	 * was an exact multiple of the max packet size:
	 *
	 *   5a - "This host is trying to complete the data stage by moving a
	 *        0-length packet. This can occur if the data stage was an exact
	 *        multiple of max packet size."
	 *   5b - "This host is trying to move more data than specified in the
	 *        wLength field of the setup bytes. In this case, software issues Set
	 *        Stall on EP0 and goes back to Step 1."
	 *
	 * 5b is an error and is recovered. 5a is legitimate, and the IN direction
	 * already carries it: udc_dwc3_trb_ctrl_in() arms the terminating
	 * zero-length TRB itself when the stack marks the buffer with a ZLP, so the
	 * continuation is in place before the host asks. The OUT direction has no
	 * equivalent - completing it needs a receive buffer that the stack has
	 * already taken back - so it is reported and recovered rather than
	 * continued. That is a real limit, not a silent one, and it is unreachable
	 * for this device: it needs a control write whose wLength is an exact
	 * multiple of the endpoint's 512-byte max packet size.
	 */
	if (priv->ctrl_data_done) {
		const uint16_t wlen = sys_le16_to_cpu(setup->wLength);
		const uint16_t mps = USB_MPS_EP_SIZE(cfg->ep_data_out[0].cfg.mps);

		if (mps != 0U && (wlen % mps) == 0U) {
			udc_dwc3_ctrl_resync(dev,
				"host is ending an exact-multiple data stage with a "
				"zero-length OUT packet, which needs a receive buffer "
				"the stack has already reclaimed");
		} else {
			udc_dwc3_ctrl_resync(dev,
				"host is moving more data than the wLength it "
				"declared");
		}
		return false;
	}

	return true;
}

static void udc_dwc3_on_xfer_not_ready_in(const struct device *const dev, const uint32_t evt)
{
	/*
	 * ALWAYS ARM THE NEXT STAGE. Detection must never suppress it.
	 *
	 * udc_dwc3_ctrl_resync() was made report-only after it destroyed cold
	 * boot, but the job was only half done: udc_dwc3_ctrl_xnr_check() still
	 * returns false at five sites and this function still returned on it,
	 * skipping udc_dwc3_ctrl_next() - the one call that arms the stage the
	 * host is asking for. The detector went on silently killing control
	 * transfers while claiming to be passive.
	 *
	 * Silently, because the desync report is rate limited: it prints about a
	 * dozen times and then says nothing, while every later occurrence still
	 * drops a request. That is exactly the shape measured on the rig - 218
	 * of 400 v4l2 control writes failing with NOTHING in the log.
	 *
	 * 10e114e4, which ran 29,054 SETUPs over ~1000 s without a fault, used a
	 * switch whose every case ended in break and fell through to arm. Match
	 * that: report what is odd, then arm regardless.
	 */
	if ((evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK) ==
	    UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP) {
		/*
		 * Rate limited for the same reason as every other report on this
		 * path: it is driven by the controller, so if it ever fires it
		 * fires per event, and an uncapped line under
		 * CONFIG_LOG_MODE_MINIMAL is a synchronous console busy-wait.
		 */
		LOG_ERR_RATELIMIT("Invalid event (SETUP IN not possible)");
	} else {
		(void)udc_dwc3_ctrl_xnr_check(dev, evt, true);
	}

	udc_dwc3_ctrl_next(dev);
}

static void udc_dwc3_on_xfer_not_ready_out(const struct device *const dev, const uint32_t evt)
{
	/*
	 * ALWAYS ARM THE NEXT STAGE. Detection must never suppress it.
	 *
	 * udc_dwc3_ctrl_resync() was made report-only after it destroyed cold
	 * boot, but the job was only half done: udc_dwc3_ctrl_xnr_check() still
	 * returns false at five sites and this function still returned on it,
	 * skipping udc_dwc3_ctrl_next() - the one call that arms the stage the
	 * host is asking for. The detector went on silently killing control
	 * transfers while claiming to be passive.
	 *
	 * Silently, because the desync report is rate limited: it prints about a
	 * dozen times and then says nothing, while every later occurrence still
	 * drops a request. That is exactly the shape measured on the rig - 218
	 * of 400 v4l2 control writes failing with NOTHING in the log.
	 *
	 * 10e114e4, which ran 29,054 SETUPs over ~1000 s without a fault, used a
	 * switch whose every case ended in break and fell through to arm. Match
	 * that: report what is odd, then arm regardless.
	 */
	if ((evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK) ==
	    UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP) {
		LOG_ERR_RATELIMIT(
			"Invalid event (SETUP OUT not expected to have an event)");
	} else {
		(void)udc_dwc3_ctrl_xnr_check(dev, evt, false);
	}

	udc_dwc3_ctrl_next(dev);
}

/*
 * Decode the completion status of a retired TRB.
 *
 * Takes the snapshot that udc_dwc3_pop_trb() already produced, rather than
 * re-reading the ring. Re-reading would be wrong twice over: pop_trb has
 * advanced "tail" by the time this runs, so trb_buf[tail] is the NEXT slot and
 * not the one that just completed; and that slot may still be owned by the
 * controller, so its contents are whatever was last written there.
 */
static void udc_dwc3_on_xfer_done(const struct device *const dev,
				  const struct udc_dwc3_trb *const trb)
{
	switch (trb->status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK) {
	case UDC_DWC3_TRB_STATUS_TRBSTS_OK:
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_MISSEDISOC:
		LOG_ERR_RATELIMIT("TRBSTS MISSEDISOC on a non-control endpoint");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING:
		LOG_ERR_RATELIMIT("TRBSTS SETUPPENDING on a non-control endpoint");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS:
		LOG_ERR_RATELIMIT("TRBSTS XFERINPROGRESS on a non-control endpoint");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING:
		LOG_ERR_RATELIMIT("TRBSTS ZLPPENDING on a non-control endpoint");
		break;
	default:
		LOG_ERR_RATELIMIT("Invalid TRB type: 0x%08lx",
				  (trb->status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK));
		break;
	}
}

/*
 * Retained for comparison against the unmodified driver, and deliberately not
 * compiled.
 *
 * This used to handle XferComplete on a non-control endpoint. That routing was
 * wrong: the event means TRBs were retired, not that anything failed, so it is
 * now handled with XferInProgress by udc_dwc3_on_xfer_done_nonctrl() - see the
 * note on Table 4-8 at that dispatch. Reporting -ECANCELED and tearing the
 * endpoint down on one unexpected event is destructive where draining is not.
 *
 * Kept rather than removed so the difference against the original file stays
 * easy to read. Nothing calls it; if a genuine per-endpoint error path is ever
 * needed, this is the shape it had.
 */
#if 0
static void udc_dwc3_on_xfer_error_nonctrl(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *const ep_data = _EP_DATA_FROM_EPN(cfg, epn);
	struct udc_dwc3_trb trb;
	struct net_buf *buf;
	int ret;

	LOG_ERR("Transfer error on endpoint 0x%02x", ep_data->cfg.addr);

	ret = udc_dwc3_pop_trb(dev, ep_data, &buf, &trb);
	if (ret != 0) {
		LOG_ERR("Failed to pop a TRB");
		udc_submit_event(dev, UDC_EVT_ERROR, ret);
		return;
	}

	ret = udc_submit_ep_event(dev, buf, -ECANCELED);
	if (ret != 0) {
		LOG_ERR("Failed to report error event for buf %p", buf);
		return;
	}

	ret = udc_dwc3_ep_disable(dev, &ep_data->cfg);
	if (ret != 0) {
		LOG_ERR("Failed to resume endpoint 0x%02x", ep_data->cfg.addr);
		return;
	}

	ret = udc_dwc3_ep_resume(dev, ep_data, ep_data->cfg.stat.enabled);
	if (ret != 0) {
		LOG_ERR("Failed to resume endpoint 0x%02x", ep_data->cfg.addr);
		return;
	}
}
#endif

/*
 * The host asked and the core said NRDY. Report whether a TRB was actually armed.
 *
 * This is the measurement that decides whether the wedge is "software never posted
 * a buffer" or something else: if a TRB IS armed when this fires, the on-demand
 * theory is wrong. Rate-limited - under load the host can poll continuously.
 */
static void udc_dwc3_on_xfer_not_ready_nonctrl(const struct device *const dev,
					       const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *ep_data;
	const volatile struct udc_dwc3_trb *trb;
	uint32_t ctrl;

	if (!_EPN_IS_VALID(cfg, epn)) {
		return;
	}

	ep_data = _EP_DATA_FROM_EPN(cfg, epn);
	trb = ep_data->trb_buf;
	ctrl = trb[ep_data->tail].ctrl;

	priv->xnrdy_nonctrl++;

	LOG_WRN_RATELIMIT("XFERNOTREADY EP%02x: armed=%u hwo=%u ctrl=0x%08x "
			  "busy=%u queued=%u head=%u tail=%u full=%u evt=0x%08x",
			  ep_data->cfg.addr,
			  (ctrl & UDC_DWC3_TRB_CTRL_HWO) ? 1U : 0U,
			  (ctrl & UDC_DWC3_TRB_CTRL_HWO) ? 1U : 0U,
			  ctrl,
			  udc_ep_is_busy(&ep_data->cfg) ? 1U : 0U,
			  udc_buf_peek(&ep_data->cfg) != NULL ? 1U : 0U,
			  ep_data->head, ep_data->tail,
			  ep_data->full ? 1U : 0U, evt);
}

static void udc_dwc3_on_xfer_done_nonctrl(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *ep_data;
	struct net_buf *buf;
	int ret;

	if (!_EPN_IS_VALID(cfg, epn)) {
		LOG_ERR_RATELIMIT("event 0x%08x names physical endpoint %d, which "
				  "this controller does not have (%u IN, %u OUT) - "
				  "discarded",
				  evt, epn, cfg->num_in_eps, cfg->num_out_eps);
		return;
	}

	ep_data = _EP_DATA_FROM_EPN(cfg, epn);

	while (true) {
		struct udc_dwc3_trb trb;

		ret = udc_dwc3_pop_trb(dev, ep_data, &buf, &trb);
		if (ret == -ENOBUFS || ret == -EBUSY) {
			break;
		}
		if (ret != 0) {
			LOG_ERR("Failed to pop TRB from non-control endpoint");
			udc_submit_event(dev, UDC_EVT_ERROR, ret);
			break;
		}

		LOG_DBG("XFER_DONE_NORM: EP%02x, data %p",
			ep_data->cfg.addr, (void *)buf->data);

		udc_dwc3_on_xfer_done(dev, &trb);

		/* Liveness proxy for the SETUP watchdog - see nonctrl_done. */
		priv->nonctrl_done++;
		ep_data->n_retire++;

		udc_ep_set_busy(&ep_data->cfg, false);

		ret = udc_submit_ep_event(dev, buf, 0);
		if (ret != 0) {
			LOG_ERR("Failed to submit buffer %p: %d", buf, ret);
		}

		/* We just made some room for a new buffer, check if something more to enqueue */
		k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
	}
}

static const char *udc_dwc3_get_devt_ulstchng_name(const uint32_t dsts)
{
	switch (dsts & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		switch (dsts & UDC_DWC3_DSTS_USBLNKST_MASK) {
		case UDC_DWC3_DSTS_USBLNKST_USB3_U0:
			return "DSTS_USBLNKST_USB3_U0";
		case UDC_DWC3_DSTS_USBLNKST_USB3_U1:
			return "DSTS_USBLNKST_USB3_U1";
		case UDC_DWC3_DSTS_USBLNKST_USB3_U2:
			return "DSTS_USBLNKST_USB3_U2";
		case UDC_DWC3_DSTS_USBLNKST_USB3_U3:
			return "DSTS_USBLNKST_USB3_U3";
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS:
			return "DSTS_USBLNKST_USB3_SS_DIS";
		case UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET:
			return "DSTS_USBLNKST_USB3_RX_DET";
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT:
			return "DSTS_USBLNKST_USB3_SS_INACT";
		case UDC_DWC3_DSTS_USBLNKST_USB3_POLL:
			return "DSTS_USBLNKST_USB3_POLL";
		case UDC_DWC3_DSTS_USBLNKST_USB3_RECOV:
			return "DSTS_USBLNKST_USB3_RECOV";
		case UDC_DWC3_DSTS_USBLNKST_USB3_HRESET:
			return "DSTS_USBLNKST_USB3_HRESET";
		case UDC_DWC3_DSTS_USBLNKST_USB3_CMPLY:
			return "DSTS_USBLNKST_USB3_CMPLY";
		case UDC_DWC3_DSTS_USBLNKST_USB3_LPBK:
			return "DSTS_USBLNKST_USB3_LPBK";
		case UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME:
			return "DSTS_USBLNKST_USB3_RESET_RESUME";
		default:
			return "unknown USB3 link state event";
		}
		break;
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
		switch (dsts & UDC_DWC3_DSTS_USBLNKST_MASK) {
		case UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE:
			return "DSTS_USBLNKST_USB2_ON_STATE";
		case UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE:
			return "DSTS_USBLNKST_USB2_SLEEP_STATE";
		case UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE:
			return "DSTS_USBLNKST_USB2_SUSPEND_STATE";
		case UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED:
			return "DSTS_USBLNKST_USB2_DISCONNECTED";
		case UDC_DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND:
			return "DSTS_USBLNKST_USB2_EARLY_SUSPEND";
		case UDC_DWC3_DSTS_USBLNKST_USB2_RESET:
			return "DSTS_USBLNKST_USB2_RESET";
		case UDC_DWC3_DSTS_USBLNKST_USB2_RESUME:
			return "DSTS_USBLNKST_USB2_RESUME";
		default:
			return "unknown USB2 link state event";
		}
		break;
	default:
		return "DSTS_USBLNKST (unknown)";
	}
}

#define _NORMAL_EP(n, fn) fn(n + 2)

static const char *udc_dwc3_get_event_name(const uint32_t evt, const uint32_t dsts)
{
	switch (evt & UDC_DWC3_EVT_MASK) {
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(0):
		return "DEPEVT_XFERCOMPLETE(0)";
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(1):
		return "DEPEVT_XFERCOMPLETE(1)";
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERCOMPLETE):
		return "DEPEVT_XFERCOMPLETE(n)";
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERINPROGRESS):
		return "DEPEVT_XFERINPROGRESS(n)";
	case UDC_DWC3_DEPEVT_XFERNOTREADY(0):
		return "DEPEVT_XFERNOTREADY(0)";
	case UDC_DWC3_DEPEVT_XFERNOTREADY(1):
		return "DEPEVT_XFERNOTREADY(1)";
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERNOTREADY):
		return "DEPEVT_XFERNOTREADY(n)";
	/*
	 * Without these the command completions land in the default case and print
	 * as "unknown event", which is what every End Transfer completion has been
	 * doing - 302 of them in the last capture, all of them working correctly.
	 */
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(0):
		return "DEPEVT_EPCMDCMPLT(0)";
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(1):
		return "DEPEVT_EPCMDCMPLT(1)";
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_EPCMDCMPLT):
		return "DEPEVT_EPCMDCMPLT(n)";
	case UDC_DWC3_DEVT_DISCONNEVT:
		return "DEVT_DISCONNEVT";
	case UDC_DWC3_DEVT_USBRST:
		return "DEVT_USBRST";
	case UDC_DWC3_DEVT_CONNECTDONE:
		return "DEVT_CONNECTDONE";
	case UDC_DWC3_DEVT_ULSTCHNG:
		/*
		 * Link state from the EVENT, connection speed from DSTS: the speed does
		 * not change within a session so reading it late is harmless, while the
		 * link state is exactly what does change.
		 */
		return udc_dwc3_get_devt_ulstchng_name(
			(dsts & ~UDC_DWC3_DSTS_USBLNKST_MASK) |
			FIELD_PREP(UDC_DWC3_DSTS_USBLNKST_MASK,
				   FIELD_GET(UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK, evt)));
	case UDC_DWC3_DEVT_WKUPEVT:
		return "DEVT_WKUPEVT";
	case UDC_DWC3_DEVT_SUSPEND:
		return "DEVT_SUSPEND";
	case UDC_DWC3_DEVT_SOF:
		return "DEVT_SOF";
	case UDC_DWC3_DEVT_CMDCMPLT:
		return "DEVT_CMDCMPLT";
	case UDC_DWC3_DEVT_VNDRDEVTSTRCVED:
		return "DEVT_VNDRDEVTSTRCVED";
	case UDC_DWC3_DEVT_ERRTICERR:
		return "DEVT_ERRTICERR";
	case UDC_DWC3_DEVT_EVNTOVERFLOW:
		return "DEVT_EVNTOVERFLOW";
	case UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE:
		return "free";
	default:
		return "unknown event";
	}
}

/*
 * Endpoint Command Complete.
 *
 * Only End Transfer asks for this event (see UDC_DWC3_DEPCMD_CMDIOC), and it is
 * the point at which the controller has finished concluding system bus traffic
 * for the transfer it ended - CmdAct clearing earlier meant only that the
 * command had been accepted.
 *
 * The flag is cleared here, but nothing waits on it. Blocking for this event is
 * not possible from the paths that end transfers: they run on the same work
 * queue that dispatches events, so waiting would deadlock against the very
 * worker that would deliver it. Recording it instead makes a premature
 * re-Start visible rather than invisible, and gives the recovery path something
 * to check.
 */
static void udc_dwc3_on_ep_cmd_cmplt(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *ep_data;
	bool rearmed = false;

	if (!_EPN_IS_VALID(cfg, epn)) {
		LOG_ERR_RATELIMIT("event 0x%08x names physical endpoint %d, which "
				  "this controller does not have (%u IN, %u OUT) - "
				  "discarded",
				  evt, epn, cfg->num_in_eps, cfg->num_out_eps);
		return;
	}

	ep_data = _EP_DATA_FROM_EPN(cfg, epn);

	if (!ep_data->end_xfer_pending) {
		LOG_WRN("EpCmdCmplt on EP%02x with no End Transfer outstanding",
			ep_data->cfg.addr);
	}

	ep_data->end_xfer_pending = false;

	LOG_DBG("EpCmdCmplt: DMA stopped for EP%02x", ep_data->cfg.addr);

	/*
	 * If this completion belongs to a recovery End Transfer on a control
	 * endpoint, finish the recovery now. The flag keeps this apart from the
	 * End Transfers issued by udc_dwc3_ep_disable(), which must not re-arm.
	 */
	if (priv->ctrl_recovery_pending && ep_data == priv->ctrl_recovery_ep) {
		priv->ctrl_recovery_pending = false;
		priv->ctrl_recovery_ep = NULL;
		udc_dwc3_ctrl_rearm(dev, ep_data);
		rearmed = true;
	}

	/*
	 * Second half of a non-control resume that udc_dwc3_ep_resume() postponed
	 * because this End Transfer was still concluding. Now that the controller
	 * has reported it complete, the Start Transfer inside is legal.
	 *
	 * Cleared before the call, not after: the resume issues commands of its own
	 * and must be able to defer again on a fresh End Transfer rather than find
	 * its own stale flag still standing.
	 */
	if (ep_data->resume_pending) {
		int ret;

		ep_data->resume_pending = false;

		LOG_DBG("running deferred resume for EP%02x", ep_data->cfg.addr);

		ret = udc_dwc3_ep_resume(dev, ep_data, ep_data->resume_modify);
		if (ret != 0) {
			LOG_ERR("deferred resume failed on EP%02x: %d",
				ep_data->cfg.addr, ret);
			udc_submit_event(dev, UDC_EVT_ERROR, ret);
		}
	} else if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 && ep_data->cfg.stat.enabled) {
		/*
		 * No resume was postponed, but udc_dwc3_ep_worker() may have stopped
		 * against end_xfer_pending while this End Transfer was concluding, and
		 * nothing else would wake it - udc_dwc3_ep_enqueue() only submits the
		 * work when a new buffer arrives. This is reachable whenever an
		 * endpoint is ended without being resumed, which is what
		 * udc_dwc3_on_set_config_or_interface() does to every busy non-control
		 * endpoint other than the one being enabled.
		 *
		 * The resume path needs no equivalent: it ends with the same submit.
		 */
		k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
	} else if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0 && !rearmed) {
		/*
		 * Control counterpart of the same wake-up. udc_dwc3_ctrl_try() declines
		 * to arm while end_xfer_pending is set, and the buffer that was refused
		 * is still queued with nothing scheduled to look at it again - the
		 * control path has no work queue of its own.
		 *
		 * Skipped when the recovery re-arm above already ran: that path arms a
		 * specific stage and leaves the claim standing, so this would find the
		 * endpoint busy and do nothing. Guarding on it says so rather than
		 * relying on it.
		 */
		udc_dwc3_ctrl_next(dev);
	}
}

/*
 * Report a USB/Link State Change event without letting the hardware set the log
 * rate.
 *
 * The dispatch does nothing with these beyond logging them, so under a link that
 * is retraining the driver's entire contribution was two blocking console lines
 * per transition - 2554 events, ~5100 lines and about 7.7 s of console in
 * uart_v5_3.log, on a path the controller drives at whatever rate it likes.
 *
 * Every genuine transition is still reported, because a state that differs from
 * the last one always prints. What is suppressed is repetition: the same state
 * arriving over and over collapses to one line per
 * UDC_DWC3_EVT_LINK_LOG_EVERY, carrying the run length so the rate is still
 * visible. The raw event word goes out with it so the decode can be checked
 * against the databook rather than trusted.
 */
static void udc_dwc3_log_link_event(const struct device *const dev, const uint32_t evt,
				    const uint32_t dsts)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t link = FIELD_GET(UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK, evt);

	priv->evt_link_total++;

	if (priv->evt_link_total == 1U || link != priv->evt_link_last) {
		LOG_INF("link %s evt=0x%08x (previous state x%u, %u total)",
			udc_dwc3_get_event_name(evt, dsts), evt,
			priv->evt_link_run, priv->evt_link_total);
		priv->evt_link_last = link;
		priv->evt_link_run = 1U;
		return;
	}

	priv->evt_link_run++;

	if (priv->evt_link_run % UDC_DWC3_EVT_LINK_LOG_EVERY == 0U) {
		LOG_INF("link %s repeating x%u (%u total)",
			udc_dwc3_get_event_name(evt, dsts),
			priv->evt_link_run, priv->evt_link_total);
	}
}

static void udc_dwc3_handle_event(const struct device *const dev, const uint32_t evt)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t dsts = sys_read32(base + UDC_DWC3_DSTS);

	/*
	 * Both banners are logged OUTSIDE the mutex, deliberately.
	 *
	 * Under CONFIG_LOG_MODE_MINIMAL a log line is a synchronous, per-character
	 * busy-wait on the console UART, so any line written with the lock held adds
	 * its console time directly to the mutex hold time - and through it to every
	 * control transfer waiting on udc_dwc3_ep_enqueue(). These two lines are ~16%
	 * of everything this driver prints, and they are the only two that can move:
	 * the rest sit between state transitions and read state that is only valid
	 * under the lock.
	 *
	 * Safe because neither touches shared state. udc_dwc3_get_event_name() is a
	 * pure switch over evt and dsts returning string literals, and both values are
	 * locals captured above - dsts is read before the lock either way.
	 *
	 * Costs one thing worth knowing: another thread logging concurrently can now
	 * slip a line between this banner and the dispatch it introduces. Threads that
	 * never take this mutex could always do that, so it is a widening of an
	 * existing gap rather than a new one.
	 */
	const uint32_t evt_type = evt & UDC_DWC3_EVT_MASK;
	const bool is_link_evt = evt_type == UDC_DWC3_DEVT_ULSTCHNG;
	/*
	 * Events the CONTROLLER can produce faster than this console can print
	 * them. Both get one rate-limited line of their own and skip the generic
	 * banner and the "end" line entirely - two lines per event, unbounded, is
	 * what turned a link retrain into a dead device.
	 *
	 * Overflow is the worse of the two, because it is raised only when the
	 * driver is ALREADY behind: every line spent reporting it is time not
	 * spent draining, which produces the next overflow. uart_v5_7.log has
	 * 1,960 such lines - 40 kB, about 3.5 s of blocking console - against 6
	 * lines for the 1,546 link events that actually caused it.
	 */
	const bool is_ovfl_evt = evt_type == UDC_DWC3_DEVT_EVNTOVERFLOW;
	/*
	 * Generic command completions are silent for the same reason. The driver
	 * issues exactly one generic command, the one udc_dwc3_evt_force() uses to
	 * shake an event loose, so every completion here is its echo and carries
	 * nothing the forced command's own rate-limited line does not already say.
	 * Naming each one would put a banner on the console at the force rate, and
	 * it would do it while a slot is stuck - precisely when the console must not
	 * be the thing holding up the drain. UDC_DWC3_EVT_FORCE_MIN_GAP_MS now
	 * bounds that rate at two a second; the reasoning stands whatever the bound.
	 */
	const bool is_cmdcmplt_evt = evt_type == UDC_DWC3_DEVT_CMDCMPLT;
	/*
	 * The four events every healthy control transfer generates. Naming each one
	 * costs five console lines per transfer and says nothing that the SETUP line
	 * in udc_dwc3_on_ctrl_out() does not already say - and under
	 * CONFIG_LOG_MODE_MINIMAL those lines are a synchronous busy-wait on the
	 * UART taken inside the event worker. A capture measured ten lines and 24 ms
	 * of blocking console per SET_CUR, which is what caps this device near 41
	 * control transfers a second and what filled the event ring while the drain
	 * was printing rather than draining.
	 *
	 * Only endpoints 0 and 1 are silenced. An event on any other endpoint is
	 * rare here and still worth a line.
	 */
	const bool is_ctrl_xfer_evt = evt_type == UDC_DWC3_DEPEVT_XFERCOMPLETE(0) ||
				      evt_type == UDC_DWC3_DEPEVT_XFERCOMPLETE(1) ||
				      evt_type == UDC_DWC3_DEPEVT_XFERNOTREADY(0) ||
				      evt_type == UDC_DWC3_DEPEVT_XFERNOTREADY(1);
	const bool is_quiet_evt = is_link_evt || is_ovfl_evt || is_cmdcmplt_evt ||
				  is_ctrl_xfer_evt;

	if (is_link_evt) {
		udc_dwc3_log_link_event(dev, evt, dsts);
	} else if (!is_quiet_evt) {
		LOG_INF("%s", udc_dwc3_get_event_name(evt, dsts));
	}

	/*
	 * Published for udc_dwc3_heartbeat_worker(), which runs on another thread.
	 * Stamp first, then the event: a reader must never see a live event against
	 * a stale timestamp. Zero is a safe "no dispatch" value because an all-zero
	 * word is not a valid event encoding.
	 */
	priv->dispatch_t0 = k_cycle_get_32();
	priv->dispatch_evt = evt;

	udc_lock_internal(dev, K_FOREVER);

	switch (evt & UDC_DWC3_EVT_MASK) {
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(0):
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(1):
		udc_dwc3_on_ctrl(dev, evt);
		break;
	/*
	 * Both completion events retire TRBs and both mean success. Which one the
	 * controller raises depends only on the TRB control bits, per Table 4-8:
	 * with IOC=1, CSP=1, LST=0 and CHN=0 - how this driver builds every
	 * non-control TRB - each retired TRB gives XferInProgress, in both
	 * directions. XferComplete needs LST=1, or CSP=0 on an OUT endpoint that
	 * received a short packet, so it should not appear here at all.
	 *
	 * It is still routed to the same handler rather than to a teardown path. If
	 * it ever does arrive, draining is harmless - udc_dwc3_pop_trb() checks HWO
	 * and returns -EBUSY when the controller still owns the TRB, so nothing is
	 * reported and nothing is torn down. Treating it as an error would report
	 * -ECANCELED on a live buffer and disable a working endpoint on the strength
	 * of one unexpected event. It also removes a trap for later: setting LST to
	 * mark transfer boundaries, or clearing CSP on an OUT endpoint, would make
	 * XferComplete the ordinary event for normal traffic.
	 */
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERCOMPLETE):
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERINPROGRESS):
		udc_dwc3_on_xfer_done_nonctrl(dev, evt);
		break;
	case UDC_DWC3_DEPEVT_XFERNOTREADY(0):
		udc_dwc3_on_xfer_not_ready_out(dev, evt);
		break;
	case UDC_DWC3_DEPEVT_XFERNOTREADY(1):
		udc_dwc3_on_xfer_not_ready_in(dev, evt);
		break;
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(0):
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(1):
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_EPCMDCMPLT):
		udc_dwc3_on_ep_cmd_cmplt(dev, evt);
		break;
	case UDC_DWC3_DEVT_USBRST:
		udc_dwc3_on_usb_reset(dev);
		break;
	case UDC_DWC3_DEVT_CONNECTDONE:
		udc_dwc3_on_connect_done(dev);
		break;
	case UDC_DWC3_DEVT_DISCONNEVT:
		/*
		 * The link is gone, so any Endpoint Command Complete still outstanding
		 * is not coming. Without this a disconnect that is not followed by a
		 * USB reset - or one where the flags matter before the reset arrives -
		 * leaves the endpoint deferring for ever.
		 */
		udc_dwc3_drop_xfer_state(dev, "disconnect");
		break;
	/*
	 * XferNotReady on a NON-CONTROL endpoint: ignore it.
	 *
	 * It means only "the host asked and no TRB was available".  The databook: "This
	 * event can happen when software issues a Start Transfer or Update Transfer.  In
	 * this case, software must ignore this event", and "the application must enable
	 * this event if it plans to issue Start Transfer on demand".  This driver arms
	 * bulk and isochronous from ep_enqueue, not on demand, so there is nothing to do.
	 *
	 * It shared the DISCONNECT body until fixed, which was a real defect: _NORMAL_EP
	 * covers endpoints 2..31, so an ordinary poll ran udc_dwc3_drop_xfer_state() and
	 * cleared end_xfer_pending/resume_pending on every endpoint plus the recovery and
	 * watchdog state.  Those flags are what defer arming and what start_xfer() checks,
	 * so clearing them let a Start Transfer go out while the transfer resource was
	 * still held - CmdStatus 4'h1, the "Start Transfer failed on EP00" seen on the
	 * rig - and forgot any recovery in flight.  Invisible: the only trace was a
	 * LOG_DBG labelled "disconnect".
	 */
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERNOTREADY):
		udc_dwc3_on_xfer_not_ready_nonctrl(dev, evt);
		break;
	case UDC_DWC3_DEVT_ULSTCHNG:
	case UDC_DWC3_DEVT_WKUPEVT:
	case UDC_DWC3_DEVT_SUSPEND:
	case UDC_DWC3_DEVT_SOF:
	case UDC_DWC3_DEVT_CMDCMPLT:
	case UDC_DWC3_DEVT_VNDRDEVTSTRCVED:
		break;
	case UDC_DWC3_DEVT_ERRTICERR:
		/*
		 * Erratic error. On UTMI+ this means phy_rxvalid/phy_rxactive stayed
		 * asserted for at least 2 ms; the controller then "goes into Suspended
		 * state and a USB/Link state change event (ULStChng) is generated", and
		 * "the application can only perform a soft disconnect [to] recover". At
		 * SuperSpeed it means a PHY command went unanswered for 100 ms, and
		 * "software must reset the controller". Either way the link does not come
		 * back on its own.
		 *
		 * Dispatched here purely so it is NAMED. Previously it had no case and
		 * fell through to the default arm below, where a genuine hardware fault
		 * would have been reported as "unknown event" and been indistinguishable
		 * from noise - the worst possible state for something being diagnosed
		 * from logs.
		 *
		 * The recovery is deliberately NOT performed here. Both forms mean tearing
		 * the connection down and rebuilding it, which is not this handler's job
		 * and would fight the existing watchdog. Report it and let the layer above
		 * decide.
		 */
		/*
		 * Rate-limited because nothing here clears the fault: the comment above
		 * says the link does not come back on its own, so the controller is free
		 * to raise this again on every pass. Reporting a stuck fault at event
		 * rate is what turned the overflow event into 1,960 lines of blocking
		 * console; the first line is the whole message either way.
		 */
		LOG_ERR_RATELIMIT("DEVT_ERRTICERR: PHY erratic error - the link is "
			"suspended and needs a disconnect/reconnect to recover");
		udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
		break;
	case UDC_DWC3_DEVT_EVNTOVERFLOW:
		/*
		 * The only line for this event now - the generic banner and "end" are
		 * suppressed above. Rate limited, and short: the macro reports how many
		 * it suppressed when it next emits, so the rate is still visible without
		 * a counter or a line per occurrence.
		 */
		LOG_ERR_RATELIMIT("evt ring ovfl");
		break;
	default:
		/*
		 * Skip the event, do not assume it cannot happen.
		 *
		 * This was CODE_UNREACHABLE, which outside ARCH_POSIX is a bare
		 * __builtin_unreachable() - undefined behaviour on an input that comes
		 * from the controller over a posted AXI write, not from this driver. It
		 * would also fall out of the function without reaching the
		 * udc_unlock_internal() below, wedging the UDC mutex for every thread
		 * and taking the whole stack down with it.
		 *
		 * Breaking is also the correct thing for the ring: the caller spends the
		 * GEVNTCOUNT credit for this slot after this function returns, so
		 * returning normally consumes the unknown word and keeps the read index
		 * aligned with the controller's write pointer. Skipping the credit is
		 * what would desync the ring for good.
		 *
		 * Rate limited because breaking makes this repeatable: a desynchronised
		 * ring turns every subsequent word into an unknown event, and an
		 * unbounded line here would replace undefined behaviour with a log flood
		 * - a different way to lose the device, not a fix. The suppressed count
		 * is reported when it next emits.
		 */
		LOG_ERR_RATELIMIT("unknown event: 0x%x (%u out of %u)",
				  evt,
				  udc_dwc3_gevntcount(base),
				  CONFIG_UDC_DWC3_EVENTS_NUM);
		break;
	}

	udc_unlock_internal(dev);

	priv->dispatch_evt = 0U;

	/*
	 * Outside the lock - see the note above the opening banner. DBG rather than
	 * INF now: its only job was to show that a dispatch completed, and proving
	 * that by the ABSENCE of a line means noticing a gap in a six-megabyte log.
	 * udc_dwc3_heartbeat_worker() reports a stuck dispatch positively instead,
	 * from another thread, which is the only place it can be seen from.
	 */
	if (!is_quiet_evt) {
		LOG_DBG("end");
	}
}

/*
 * Liveness, checked from the system work queue.
 *
 * The control watchdog cannot do this job: udc_dwc3_on_ctrl() cancels it on every
 * control event, so once a transfer completes there is no deadline armed anywhere.
 * In uart_v5_3.log the link died moments after a completion and nothing noticed
 * for the twenty seconds until the host gave up - zero warnings in 638 seconds.
 *
 * Deliberately takes NO lock. A dispatch wedged while holding the UDC mutex is
 * precisely what this has to be able to report, and blocking on that mutex to
 * report it would be the one guaranteed way to stay silent.
 */
/*
 * ISR context, so this is limited to what is legal and cheap there: an MMIO
 * read, a subtraction, and k_work_submit_to_queue() - which is explicitly
 * ISR-safe. No mutex, no logging (LOG_MODE_MINIMAL busy-waits on the console
 * UART and would hold interrupts off for milliseconds), no udc_dwc3_recover().
 * Anything that needs those runs in udc_dwc3_heartbeat_worker() instead.
 *
 * The drain kick is decided HERE rather than in the handler, because the
 * handler is itself a work item: if the queue is backed up, waiting for it to
 * run before deciding to drain adds the very delay this exists to remove. The
 * decision needs nothing the ISR cannot do, so it does not have to wait.
 *
 * The drain is submitted BEFORE the housekeeping, so on a shared FIFO queue
 * it also runs first - draining outranks reporting on it.
 */
static void udc_dwc3_heartbeat_expiry(struct k_timer *const timer)
{
	struct udc_dwc3_data *const priv =
		CONTAINER_OF(timer, struct udc_dwc3_data, heartbeat_timer);
	const struct device *const dev = priv->dev;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	if (udc_dwc3_gevntcount(base) > 0U && priv->evt_worker_ran &&
	    k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_worker_exit_t0) >=
					UDC_DWC3_EVT_IDLE_KICK_MS) {
		priv->evt_kick++;
		k_work_submit_to_queue(udc_get_work_q(), &priv->event_work);
	}

	k_work_submit_to_queue(udc_get_work_q(), &priv->heartbeat_work);
}

#ifdef STALL_DIAG_LOG
/*
 * One hardware state dump, for the RTL side, on either shape of failure.
 *
 * Reached only after a fault has already been established - a lost event write,
 * or control traffic that has stopped while the core is not idle - so nothing
 * here runs in normal operation. Kept in the shipping image because a driver
 * side workaround would hide the fault rather than locate it, and these
 * registers are the only ones that can tell the candidate causes apart.
 */
static void udc_dwc3_stall_diag_dump(const struct device *const dev,
				     const char *const why)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	LOG_ERR("=== STALL DIAG (%s) ===", why);
	const uint32_t gsts = sys_read32(base + UDC_DWC3_GSTS);

	/*
	 * The one register that can turn this from an observation
	 * into a hardware fault report. 1.2.13: "When the AHB or AXI
	 * Master Bus returns an 'Error' response, the 'SoC Bus Error'
	 * is generated... In the Device mode, the GSTS.BusErrAddrVld
	 * field is the only indication of the SoC Bus Error." It is
	 * sticky - clearable only by resetting the controller - so it
	 * still stands whenever the stall is noticed, and this driver
	 * never soft-resets on the live path.
	 */
	LOG_ERR("  BUS: GSTS=0x%08x BusErrAddrVld=%u "
		"GBUSERRADDR=0x%08x%08x",
		gsts,
		(gsts & UDC_DWC3_GSTS_BUSERRADDRVLD) ? 1U : 0U,
		sys_read32(base + UDC_DWC3_GBUSERRADDR_HI),
		sys_read32(base + UDC_DWC3_GBUSERRADDR_LO));

	/*
	 * Is the core still pointed at the buffer we are reading? A
	 * corrupted GEVNTADR would have it writing somewhere else
	 * entirely, with symptoms identical to a lost write.
	 */
	LOG_ERR("  RING: GEVNTADR=0x%08x%08x SIZ=0x%08x CNT=0x%08x "
		"| driver buf=%p stalled slot %u at %p",
		sys_read32(base + UDC_DWC3_GEVNTADR_HI(0)),
		sys_read32(base + UDC_DWC3_GEVNTADR_LO(0)),
		sys_read32(base + UDC_DWC3_GEVNTSIZ(0)),
		sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)),
		(void *)cfg->evt_buf, priv->evt_next,
		(void *)&cfg->evt_buf[priv->evt_next]);

	/*
	 * EVERY endpoint, not just EP0.
	 *
	 * All OUT endpoints share one RX FIFO on this core, so a bulk OUT left
	 * without an armed TRB strands its packet in that FIFO and blocks the
	 * control endpoint behind it. EP0's own registers cannot tell that state
	 * apart from a control-only fault: three separate captures showed
	 * rxfifoempty=0 with EP0 correctly armed - once as CONTROL-SETUP, once as
	 * CONTROL-DATA - which says the blockage is not EP0's to begin with.
	 *
	 * One compact line per enabled endpoint. This path only runs once traffic
	 * has already stopped, so the console cost buys the one fact EP0 cannot.
	 */
	/*
	 * The RxFIFO, and why it is worth dumping.
	 *
	 * Device mode has exactly ONE receive FIFO for every OUT endpoint: the
	 * databook states "Since the device mode uses only one RXFIFO, there is no
	 * Device RXFIFO DMA Priority Register", and GRXFIFOSIZ0 "allocate[s] the
	 * receive buffer for all endpoints".  A packet is routed to the addressed
	 * endpoint's TRB only when that endpoint has one armed; until then it
	 * occupies the shared buffer, so a single unserviced OUT endpoint can hold
	 * up every other one - including EP0.
	 *
	 * This driver leaves GRXFIFOSIZ at its reset default, which the databook
	 * only recommends "unless the packet sizes of the endpoints are
	 * application-specific".  Ours are: two 1024-byte bulk OUT endpoints plus a
	 * 512-byte control endpoint.  Dump the depth so the assumption is checkable
	 * against the traffic rather than assumed.
	 */
	{
		const uint32_t rxsz = sys_read32(base + UDC_DWC3_GRXFIFOSIZ(0));
		const uint32_t mdw = (sys_read32(base + UDC_DWC3_GHWPARAMS0) >> 8) & 0xFFU;

		{
		const uint32_t now = k_cycle_get_32();

		LOG_ERR("  CTRLTRACE: setup_up n=%u %ums ago | enq n=%u %ums ago "
			"(s/d/st=%u%u%u) | armed n=%u %ums ago",
			priv->ctrl_setup_up_n,
			k_cyc_to_ms_near32(now - priv->ctrl_setup_up_t),
			priv->ctrl_enq_n,
			k_cyc_to_ms_near32(now - priv->ctrl_enq_t),
			(priv->ctrl_enq_last >> 2) & 1U,
			(priv->ctrl_enq_last >> 1) & 1U,
			priv->ctrl_enq_last & 1U,
			priv->ctrl_armed_n,
			k_cyc_to_ms_near32(now - priv->ctrl_armed_t));
	}

	LOG_ERR("  RXFIFO: GRXFIFOSIZ0=0x%08x depth=%u start=%u mdwidth=%u "
			"(%u bytes) GRXTHRCFG=0x%08x DALEPENA=0x%08x",
			rxsz,
			(uint32_t)FIELD_GET(UDC_DWC3_GRXFIFOSIZ_RXFDEP_MASK, rxsz),
			(uint32_t)FIELD_GET(UDC_DWC3_GRXFIFOSIZ_RXFSTADDR_MASK, rxsz),
			mdw,
			(uint32_t)FIELD_GET(UDC_DWC3_GRXFIFOSIZ_RXFDEP_MASK, rxsz) *
				mdw / BITS_PER_BYTE,
			sys_read32(base + UDC_DWC3_GRXTHRCFG),
			sys_read32(base + UDC_DWC3_DALEPENA));
	}

	for (uint8_t d = 0; d < 2U; d++) {
		const uint8_t n = d ? cfg->num_in_eps : cfg->num_out_eps;

		for (uint8_t i = 0; i < n; i++) {
			struct udc_dwc3_ep_data *const e =
				d ? &cfg->ep_data_in[i] : &cfg->ep_data_out[i];
			const volatile struct udc_dwc3_trb *t;
			uint32_t c;
			uint32_t rxfree;

			if (!e->cfg.stat.enabled) {
				continue;
			}

			t = e->trb_buf;
			c = t[e->tail].ctrl;

			/*
			 * FREE SPACE in the shared RxFIFO, not bytes queued for this
			 * endpoint. GDBGFIFOSPACE[31:16] is SPACE_AVAILABLE, and there
			 * is one RxFIFO for every OUT endpoint in device mode, so this
			 * reads the same on all of them - selecting a queue number does
			 * not make it per-endpoint. An earlier label here said "bytes
			 * queued", which made an identical reading of 8 on EP00/01/02
			 * look like a packet waiting on an endpoint nobody had written
			 * to. There is no per-endpoint RxFIFO occupancy to read.
			 * Read inline: udc_dwc3_read_fifo_space() is defined further
			 * down the file, and this diagnostic must not force a forward
			 * declaration into the middle of the driver.
			 */
			if (USB_EP_DIR_IS_OUT(e->cfg.addr)) {
				uint32_t r = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQQ;

				r |= FIELD_PREP(UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK,
						(uint32_t)e->epn);
				sys_write32(r, base + UDC_DWC3_GDBGFIFOSPACE);
				r = sys_read32(base + UDC_DWC3_GDBGFIFOSPACE);
				rxfree = FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK, r);
			} else {
				rxfree = 0U;
			}

			LOG_ERR("  EP%02x: busy=%u hwo=%u trbctl=%u ctrl=0x%08x "
				"sts=0x%08x head=%u tail=%u full=%u endxfer=%u "
				"depcmd=0x%08x rxfree=%u",
				e->cfg.addr,
				udc_ep_is_busy(&e->cfg) ? 1U : 0U,
				(c & UDC_DWC3_TRB_CTRL_HWO) ? 1U : 0U,
				(uint32_t)((c & UDC_DWC3_TRB_CTRL_TRBCTL_MASK) >> 4),
				c, t[e->tail].status,
				e->head, e->tail, e->full ? 1U : 0U,
				e->end_xfer_pending ? 1U : 0U,
				sys_read32(base + UDC_DWC3_DEPCMD(e->epn)),
				rxfree);
		}
	}

	/*
	 * The whole ring. If the stalled slot holds the free marker
	 * while a LATER one holds data, the write landed out of order
	 * or at the wrong offset - a different fault from a write that
	 * never happened, and not visible from one slot alone.
	 */
	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_EVENTS_NUM; i += 4) {
		LOG_ERR("  RING[%02u]: 0x%08x 0x%08x 0x%08x 0x%08x",
			i, cfg->evt_buf[i], cfg->evt_buf[i + 1],
			cfg->evt_buf[i + 2], cfg->evt_buf[i + 3]);
	}

	udc_dwc3_core_state_dump(dev);

	/* For correlating with an ILA or bus trace. */
	LOG_ERR("  AT: DSTS=0x%08x cycles=%u",
		sys_read32(base + UDC_DWC3_DSTS), k_cycle_get_32());
}
#endif /* STALL_DIAG_LOG */

/* Defined below; the heartbeat owns the decision, this performs it. */
static bool udc_dwc3_evt_skip_dead_slot(const struct device *const dev,
					const uint32_t gc, const bool frozen,
					const uint32_t gaveup_ms);

static void udc_dwc3_heartbeat_worker(struct k_work *work)
{
	struct udc_dwc3_data *const priv =
		CONTAINER_OF(work, struct udc_dwc3_data, heartbeat_work);
	const struct device *const dev = priv->dev;
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t d_evt = priv->dispatch_evt;
	const uint32_t gc = udc_dwc3_gevntcount(base);
	/*
	 * How long the drain has been parked on the same empty slot, or 0 when it
	 * is not parked at all. Only meaningful while a run is active: without one
	 * evt_gaveup_t0 belongs to some older run.
	 */
	const uint32_t gaveup_ms = priv->evt_gaveup_run > 0U
		? k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_gaveup_t0)
		: 0U;
	const uint32_t idle_ms = priv->evt_worker_ran
		? k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_worker_exit_t0)
		: 0U;
	bool drain_stuck;

	if (gc > 0U && priv->evt_handled == priv->hb_last_handled) {
		priv->hb_drain_stuck_beats++;
	} else {
		priv->hb_drain_stuck_beats = 0U;
	}
	priv->hb_last_handled = priv->evt_handled;

	drain_stuck = (priv->hb_drain_stuck_beats * UDC_DWC3_HEARTBEAT_MS) >=
		      CONFIG_UDC_DWC3_RECOVERY_TIMEOUT;



	/*
	 * Kick a handler that has stopped being scheduled.
	 *
	 * This is the one thing here that is a repair rather than a report, and it
	 * is deliberately unconditional on the diagnostic branches below - the
	 * ring needs draining whether or not anything is worth printing about it.
	 *
	 * The condition is simply: the controller says there is something to read,
	 * and no pass has COMPLETED for UDC_DWC3_EVT_IDLE_KICK_MS. That covers the
	 * case the interrupt cannot: a full ring leaves the controller with nowhere
	 * to write, so it generates no further events and therefore no further
	 * interrupts, and nothing else would ever ask the driver to look again.
	 *
	 * Submitting a work item that is already queued or running is a no-op, so
	 * this cannot pile up behind a busy handler.
	 */
	/*
	 * The kick itself has moved into udc_dwc3_heartbeat_expiry(), which sees
	 * the same condition one queue-hop earlier. What is left here is the
	 * reporting below, which needs thread context.
	 */

	/*
	 * Break a control claim that will never be given back.
	 *
	 * This is the hole that leaves the device permanently dead rather than
	 * briefly late, and an occasional control timeout does not excuse it: the
	 * host retries a timed-out request quite happily, but only if the device
	 * can answer the retry. Once ep0 is claimed and the controller never
	 * completes the descriptor there is no event, so no completion, so the
	 * claim is never released - and udc_dwc3_ctrl_try() then turns every later
	 * attempt away at the udc_ep_is_busy() tests. Waiting changes nothing, and
	 * a restarted host loop fails from its very first request, which is
	 * exactly what the rig shows.
	 *
	 * udc_dwc3_ctrl_arm_watchdog() does not cover it. It is armed for DATA and
	 * STATUS stages only, never for a SETUP, and that exclusion is right: this
	 * driver arms SETUPs speculatively, so a SETUP sitting armed with no host
	 * request outstanding is the normal idle state, and timing it out tore
	 * down healthy endpoints - a rig capture once put 299 of 490 recoveries in
	 * that class.
	 *
	 * So the trigger is not "a claim is old" but "a claim is old AND the stack
	 * is still asking". ctrl_decline_pending means udc_dwc3_ctrl_try() was
	 * turned away since the last grant. An idle device never sets it, because
	 * nothing is trying to arm; enumeration sets it constantly but clears it
	 * again within microseconds on the next grant. Only a genuine wedge holds
	 * it set while the clock runs out. That is what makes this safe on the
	 * SETUP path, where a plain timeout was not.
	 *
	 * Reported at ERROR, unconditionally. The declines themselves are LOG_DBG
	 * and so compiled out at INF, which is why a wedging device has been
	 * printing nothing at all while it dies.
	 */
	/*
	 * A SETUP ARMED ON EP0-OUT IS NEVER A WEDGE, however old the claim and
	 * however many declines have piled up behind it.
	 *
	 * Without this clause the test fires on a perfectly healthy idle device.
	 * Declines are NORMAL here - udc_dwc3_ctrl_try() is turned away every
	 * time it is offered a stage while the speculative SETUP legitimately
	 * holds the endpoint - and ctrl_arm_t0 only advances on a GRANT. So any
	 * pause in control traffic longer than the timeout looks identical to a
	 * stuck claim. uart_v6_11 caught it doing exactly that: the detector
	 * fired the moment the host started the video stream and paused control
	 * traffic - "claimed 1100 ms ... (865 declines)" - and every silence
	 * report in that capture shows trb o/i 0x00000023, i.e. HWO set with
	 * TRBCTL 2, a correctly armed Control-Setup with a buffer queued behind
	 * it. Each false firing then issued Set Stall on EP0-OUT and broke the
	 * host's next request; 42 recoveries in that run against 1 in the whole
	 * of the 20-minute uart_v6_6.
	 *
	 * The genuine condition this is for - a claim held with NO descriptor
	 * armed to receive the host's next SETUP - is unaffected, and that is
	 * the only case in which recovery can help.
	 */
	/*
	 * ...OR the drain has stopped, which is the case the clause above misses.
	 *
	 * uart_01sep_2037_fix1 wedged with a Control-Setup correctly armed
	 * (trb o/i 0x00000023) and 83172 declines behind it, so the armed-SETUP
	 * exclusion held recovery off for the entire wedge - resync 0, and the
	 * SETUP watchdog was suppressed too because RXFIFOEMPTY was set. Nothing
	 * could act. What that capture DID show, and a healthy pause never does,
	 * is "gc 12 B" standing unconsumed: the core had placed three events the
	 * drain never took. drain_stuck is that condition held for the full
	 * timeout, and it cannot fire on the healthy pause the exclusion exists
	 * to protect - an idle device reads GEVNTCOUNT 0.
	 */
	if (priv->ctrl_decline_pending &&
	    (!udc_dwc3_ctrl_armed_setup(&cfg->ep_data_out[0]) || drain_stuck) &&
	    k_cyc_to_ms_near32(k_cycle_get_32() - priv->ctrl_arm_t0) >=
					CONFIG_UDC_DWC3_RECOVERY_TIMEOUT) {
		priv->ctrl_recover++;
		LOG_ERR("control endpoint claimed %u ms with the host still asking "
			"(%u declines, %s): recovering through controller recovery "
			"and a fresh SETUP arm",
			k_cyc_to_ms_near32(k_cycle_get_32() - priv->ctrl_arm_t0),
			priv->ctrl_decline,
			drain_stuck ? "drain stuck, events unconsumed"
				    : "no SETUP armed");

		/* Re-arm the detector, or it re-enters on every following beat. */
		priv->hb_drain_stuck_beats = 0U;

		/* Stamp first, so a failed recovery cannot re-enter every beat. */
		priv->ctrl_arm_t0 = k_cycle_get_32();
		priv->ctrl_decline_pending = false;

		/*
		 * udc_dwc3_recover() issues Set Stall on EP0-OUT.  It does NOT issue an
		 * End Transfer - an earlier version of this comment claimed it did, and
		 * that was wrong.  Worth being precise about, because uart_v6_1355 shows
		 * this call rescuing four stalled event writes in a row (the pending
		 * XFERCOMPLETE landed immediately after each "Recovering USB state"), so
		 * Set Stall - not End Transfer - is the action with evidence behind it.
		 */
		(void)udc_dwc3_recover(dev);

		/*
		 * Then STALL EP0-OUT, which is what actually unblocks the HOST.
		 *
		 * Recovering our own side is only half of it. The host is still
		 * waiting on a transfer we have just thrown away, and with nothing
		 * said it waits out its full timeout - USB_CTRL_SET_TIMEOUT, 5 s on
		 * Linux - before it retries. A STALL makes the controller answer the
		 * next IN or OUT of that dead transfer immediately, so the host fails
		 * fast with -EPIPE and reissues instead of sitting out the timeout.
		 * This is the resynchronisation the databook prescribes for every
		 * 4.4.1/4.4.2 error case: Set Stall on EP0, then back to Step 1.
		 *
		 * And it is safe to issue even if this recovery turns out to be
		 * unnecessary, which is what makes it usable on a timeout. Databook
		 * 3.2.2 on Set/Clear Stall: "For control endpoints, the application
		 * issues only the Set Stall command, and only on the OUT direction of
		 * the control endpoint. The controller automatically clears the STALL
		 * when it receives a SETUP token for the endpoint. The application
		 * must not issue the Clear Stall command on a control endpoint.\"
		 * So the stall is self-clearing in HARDWARE on the host's very next
		 * SETUP - it cannot latch, and it cannot leave the endpoint halted
		 * the way a spurious stall on a bulk endpoint would. That is why this
		 * is a safe thing to do on a suspicion, and a Clear Stall must never
		 * be paired with it here.
		 *
		 * Issued after the re-arm on purpose: the replacement SETUP is then
		 * already waiting when the hardware clears the stall.
		 */
		/*
		 * Under the lock. udc_dwc3_recover() takes and drops it internally,
		 * which left this command outside any lock at all - and a DEPCMD
		 * issued bare races the usbd thread, which reaches
		 * udc_dwc3_depcmd_start_xfer() through ep_enqueue on the same
		 * endpoint command registers.
		 */
		udc_lock_internal(dev, K_FOREVER);
		udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_out[0]);

		udc_unlock_internal(dev);
	}

	/*
	 * Report the moment control traffic stops, with the state that decides
	 * WHY. Everything in this line has been guessed at across three captures
	 * and never once measured at the point of failure:
	 *
	 *   busy o/i    - is either control endpoint still claimed? If both are
	 *                 free the wedge is not a stuck claim at all.
	 *   trb o/i     - the ownership words. HWO still set means the controller
	 *                 never retired the descriptor; clear means it did and the
	 *                 completion is what went missing.
	 *   gc          - events pending but undrained.
	 *   DSTS        - link state, so a link-level cause is separable.
	 *   decline     - was the stack still asking? Distinguishes "we refused
	 *                 the host" from "the host stopped asking".
	 *   setuppend   - did the core ever report an abandoned transfer?
	 */
	if (priv->ctrl_setup_done != priv->hb_last_setup_done) {
		priv->hb_last_setup_done = priv->ctrl_setup_done;
		priv->ctrl_quiet_t0 = k_cycle_get_32();
		priv->ctrl_quiet_logged = false;
	} else if (!priv->ctrl_quiet_logged && priv->ctrl_setup_done > 0U &&
		   priv->ctrl_decline_pending &&
		   k_cyc_to_ms_near32(k_cycle_get_32() - priv->ctrl_quiet_t0) >=
						UDC_DWC3_CTRL_QUIET_MS) {
		/*
		 * ctrl_decline_pending is the whole point of this clause: silence on
		 * the control endpoint is only worth reporting if something is waiting
		 * to be armed and cannot be. Without it this fired on a HEALTHY IDLE
		 * bus - uart_02sep_0236_nocspam produced seven of these 32-line dumps
		 * in three minutes purely because the control-spam load had been turned
		 * off and nothing was generating traffic.
		 *
		 * Same error as the non-control sweep that was removed: acting on a
		 * signal that also occurs during normal operation. An idle endpoint and
		 * a stalled one look identical unless something external says the host
		 * is asking, and the decline is that something.
		 */
		priv->ctrl_quiet_logged = true;
		LOG_ERR("no control traffic for %u ms after %u SETUPs: busy o/i %u/%u, "
			"trb o/i 0x%08x/0x%08x, gc %u B, DSTS 0x%08x (rxfifoempty %u), "
			"decline %u, "
			"setuppend %u, ep0out queued %u",
			UDC_DWC3_CTRL_QUIET_MS, priv->ctrl_setup_done,
			udc_ep_is_busy(&cfg->ep_data_out[0].cfg) ? 1U : 0U,
			udc_ep_is_busy(&cfg->ep_data_in[0].cfg) ? 1U : 0U,
			cfg->ep_data_out[0].trb_buf[0].ctrl,
			cfg->ep_data_in[0].trb_buf[0].ctrl,
			gc, sys_read32(base + UDC_DWC3_DSTS),
			(sys_read32(base + UDC_DWC3_DSTS) &
			 UDC_DWC3_DSTS_RXFIFOEMPTY) ? 1U : 0U,
			priv->ctrl_decline, priv->ctrl_setup_pending,
			udc_buf_peek(&cfg->ep_data_out[0].cfg) != NULL ? 1U : 0U);

#ifdef STALL_DIAG_LOG
		udc_dwc3_stall_diag_dump(dev, "control traffic stopped");
#endif

		/*
		 * Non-control endpoints get a re-cache here, and ONLY here.
		 *
		 * They have no stall detection of their own and cannot have one on
		 * their own terms: an OUT endpoint armed and waiting for a host with
		 * nothing to send is byte-for-byte identical to one that is stalled.
		 * HWO stays 1 and BUFSIZ keeps the software-prepared value in both
		 * cases, because the controller only writes the descriptor back when it
		 * retires it - databook 4.2.3, "when the hardware writes back the TRBs,
		 * it updates the BUFSIZ field to represent the remaining unused
		 * buffer". A periodic sweep would therefore fire on healthy endpoints
		 * and its counter would carry no information.
		 *
		 * This branch supplies the evidence the endpoint cannot: control
		 * traffic has stopped, so something is wrong device-wide. Re-caching
		 * every armed OUT descriptor then costs one command each and may
		 * unstick a bulk endpoint that would otherwise fail silently - which
		 * until now it would have, the watchdog guarding only the control pair.
		 *
		 * Re-cache only: no stall, no End Transfer, no claim cleared.
		 */
		for (int i = 1; i < cfg->num_out_eps; i++) {
			struct udc_dwc3_ep_data *const e = &cfg->ep_data_out[i];
			const volatile struct udc_dwc3_trb *t = e->trb_buf;

			if (t == NULL || !udc_ep_is_busy(&e->cfg) ||
			    (t[e->tail].ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U) {
				continue;
			}

			priv->nonctrl_recache++;
			udc_dwc3_depcmd_update_xfer(dev, e);

			LOG_ERR("  EP%02x armed through a control stall (ctrl 0x%08x "
				"sts 0x%08x): re-cached (%u)", e->cfg.addr,
				t[e->tail].ctrl, t[e->tail].status,
				priv->nonctrl_recache);
		}
	}

	/*
	 * THE EP0-OUT ALWAYS-ARMED INVARIANT CHECK WAS REMOVED HERE, and must
	 * not come back in this form.
	 *
	 * It tested `!armed_setup(ep0_out) && (trb_buf[0].ctrl & HWO) == 0` and,
	 * on two consecutive beats, cleared the claim and re-armed. The premise
	 * is wrong: the control completion paths memset trb_buf[0] to zero, so
	 * HWO reads 0 for a perfectly healthy endpoint in the gap between
	 * transfers. Under control traffic that gap is continuous, so the check
	 * fired on healthy endpoints, dropped a live claim and re-armed over the
	 * top - corrupting the control state machine.
	 *
	 * Measured by bisection on the rig, same 400-write control-spam test,
	 * same host, DUT presence verified before and after each run:
	 *
	 *   without this check ... 0 of 400 writes failed
	 *   with this check ..... 400 of 400 writes failed
	 *
	 * That is the spurious-teardown failure this driver already knew about -
	 * an earlier capture put 299 of 490 recoveries in that class - and it is
	 * why the SETUP path deliberately has no plain timeout. A detector for
	 * "EP0-OUT owns no TRB" needs a signal that distinguishes the idle gap
	 * from a real wedge; HWO on a zeroed descriptor is not that signal.
	 */

	if (d_evt != 0U) {
		const uint32_t ms =
			k_cyc_to_ms_near32(k_cycle_get_32() - priv->dispatch_t0);

		if (ms >= UDC_DWC3_DISPATCH_STUCK_MS) {
			LOG_ERR_RATELIMIT("dispatch stuck %u ms in %s (evt 0x%08x)", ms,
				udc_dwc3_get_event_name(d_evt,
					sys_read32(base + UDC_DWC3_DSTS)), d_evt);
		}
	} else if (gc > 0U && (gaveup_ms >= UDC_DWC3_EVT_GAVEUP_AGE_MS ||
				priv->evt_handled == priv->hb_last_evt_handled)) {
		/*
		 * Two different ways the ring stops being drained, and it takes both
		 * tests to see them.
		 *
		 * Nothing consumed in a whole second, with events outstanding, is the
		 * worker not running at all - silence on its own would just be an idle
		 * device. That test alone used to be enough.
		 *
		 * It stopped being enough once udc_dwc3_evt_force() existed. Forcing
		 * lands a generic command completion of its own every few give-ups, so
		 * a drain parked on one dead slot still retires events and still moves
		 * evt_handled - which is precisely the case this was written for, and
		 * precisely the case that test would now miss. A give-up run that has
		 * outlived a heartbeat period catches it directly: an ordinary late
		 * write resolves in microseconds, so a run this old is a lost write,
		 * not a slow one.
		 */
		/*
		 * The slot CONTENTS are what separate the two ways this happens, and the
		 * slot number alone cannot: evt_next is only where the drain would
		 * resume.
		 *
		 *   word == 0 - the controller announced an event whose write never
		 *               landed, and the drain is parked on an empty slot while
		 *               the core fills the ones behind it.
		 *   word != 0 - a perfectly good event that nothing came to collect,
		 *               i.e. the worker was never woken.
		 *
		 * Both end the same way - ring full, controller stops - so only this
		 * tells them apart.
		 */
		/*
		 * How LONG the slot has been empty is what settles it, not how many
		 * times we looked. One give-up spans 64 polls, about 64 microseconds -
		 * far too short to call a write lost rather than merely slow. A slot
		 * still empty seconds later, across repeated give-ups, is not slow.
		 *
		 * Only meaningful while a give-up run is active: without one the drain has
		 * not looked at this slot at all, and evt_gaveup_t0 belongs to some older
		 * run, so it is reported as unchecked instead of as an age.
		 */
		/*
		 * Name the condition that fired. Two can, and they mean opposite
		 * things: "idle" is the worker not running at all, "stalled" is a
		 * drain parked on one slot while events may still be retiring around
		 * it. Reporting one wording for both is how an earlier version of this
		 * line came to claim "none consumed" on runs where events were
		 * flowing, and the claim was then read back as evidence.
		 */
		if (priv->evt_gaveup_run > 0U) {
			LOG_ERR_RATELIMIT("%u B pending, drain NOT ADVANCING on slot %u for "
				"%u ms over %u give-ups (handled %s since last "
				"beat): slot holds 0x%08x, DSTS=0x%08x",
				gc, priv->evt_next, gaveup_ms, priv->evt_gaveup_run,
				priv->evt_handled == priv->hb_last_evt_handled ?
					"nothing" : "events",
				cfg->evt_buf[priv->evt_next],
				sys_read32(base + UDC_DWC3_DSTS));

			/*
			 * Decide here whether the slot is dead, not in the drain.
			 *
			 * The condition is the RUN, not the level of gc: the same
			 * slot has read the free marker across evt_gaveup_run
			 * re-entries while gc stayed above zero, for longer than
			 * any write has ever taken to arrive. Every term is
			 * re-read on this beat, so a transient seen once in the
			 * drain cannot reach the action - the observation and the
			 * decision are separated in time on purpose.
			 */
			if (gc >= sizeof(uint32_t) &&
			    gaveup_ms >= UDC_DWC3_EVT_DEAD_SLOT_MS &&
			    cfg->evt_buf[priv->evt_next] ==
					UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
				const bool frozen = (gc == priv->evt_gaveup_gc0);

#ifdef UDC_DWC3_EVT_DEAD_SLOT_RECOVER
				(void)udc_dwc3_evt_skip_dead_slot(dev, gc, frozen,
								  gaveup_ms);
#else
				/*
				 * Diagnostic build: describe it and leave the
				 * ring, GEVNTCOUNT and the controller exactly as
				 * the fault left them, so it stays reproducible.
				 */
				LOG_ERR("slot %u is dead (%u ms, %u give-ups, "
					"gc %u B, %s) - NOT recovering, this "
					"build preserves the fault",
					priv->evt_next, gaveup_ms,
					priv->evt_gaveup_run, gc,
					frozen ? "GEVNTCOUNT frozen"
					       : "GEVNTCOUNT advancing");
#endif
			}
		} else {
			/*
			 * idle_ms, not the threshold. Printing the constant made every
			 * one of these lines read "200 ms" regardless of how long the
			 * drain had actually been idle, which hid the distribution.
			 */
			LOG_ERR_RATELIMIT("%u B pending, drain IDLE %u ms with no stall "
				"run: slot %u holds 0x%08x, DSTS=0x%08x",
				gc, idle_ms, priv->evt_next,
				cfg->evt_buf[priv->evt_next],
				sys_read32(base + UDC_DWC3_DSTS));
		}

		/* Retry here rather than spinning in the drain loop. */
		k_work_submit_to_queue(udc_get_work_q(), &priv->event_work);
	}

	priv->hb_last_evt_handled = priv->evt_handled;

	/* No re-arm here on purpose: the periodic timer owns the cadence. */
}

#ifdef UDC_DWC3_SETUP_STUCK_RESET
/*
 * Last resort for a SETUP the controller has received and will not retire.
 *
 * SET STALL is what clears the ordinary case - uart_v6_1355 shows the
 * DATA/STATUS watchdog rescuing four stalls in a row through
 * udc_dwc3_recover(), which issues Set Stall on EP0-OUT and nothing else; the
 * pending event landed immediately after each.  (recover() has never issued an
 * End Transfer - the machinery for that is unreachable.  Earlier text here
 * claimed otherwise and misled a reviewer.)
 *
 * When Set Stall does not work, nothing softer does: at the final wedge in that
 * capture the event ring was acknowledged twice, releasing the credits the core
 * was supposedly waiting on, and the core did not move.
 *
 * So take the core down and bring it back.  This costs the video stream, which
 * is why it was held back at first - but the stream is gone by this point
 * anyway, observed directly on the rig, so there is nothing left to protect.
 *
 * GSTS is logged before the reset because the reset clears BusErrAddrVld: if a
 * SoC bus error is ever behind this, that register is the only place it is
 * visible in device mode, and this is the last chance to read it.
 */
static void udc_dwc3_setup_stuck_reset(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t gsts = sys_read32(base + UDC_DWC3_GSTS);
	int ret;

	LOG_ERR("SETUP still stuck after the Set Stall: GSTS=0x%08x "
		"BusErrAddrVld=%u GBUSERRADDR=0x%08x%08x, DSTS=0x%08x - "
		"escalating to a core soft reset (%u so far)",
		gsts, (gsts & UDC_DWC3_GSTS_BUSERRADDRVLD) ? 1U : 0U,
		sys_read32(base + UDC_DWC3_GBUSERRADDR_HI),
		sys_read32(base + UDC_DWC3_GBUSERRADDR_LO),
		sys_read32(base + UDC_DWC3_DSTS), priv->ctrl_setup_wd_reset);

	/*
	 * udc_dwc3_init() sleeps 100 us across the PHY reset.  That is a sleep under
	 * the UDC mutex, which is normally a defect in this driver - but it is
	 * bounded and tiny, and every path that could contend for the lock is
	 * already dead by the time this runs.
	 */
	udc_lock_internal(dev, K_FOREVER);

	priv->evt_next = 0;
	udc_dwc3_disable(dev);

	/*
	 * shutdown() is not optional here.  udc_dwc3_disable() stops the timer,
	 * clears RunStop and masks the IRQ, but it does NOT disable the endpoints -
	 * udc_ep_config.stat.enabled stays set.  udc_dwc3_init() then calls
	 * udc_ep_enable_internal() on EP0, which returns -EALREADY (udc_common.c),
	 * and init() bails out on that error.  The result was a core that had been
	 * soft-reset with its control endpoints never reconfigured - deader than the
	 * wedge this is trying to clear.  shutdown() disables both control
	 * endpoints, which is what lets init() re-enable them.
	 */
	ret = udc_dwc3_shutdown(dev);
	if (ret != 0) {
		LOG_ERR("escalation: shutdown failed (%d), core left reset", ret);
		udc_unlock_internal(dev);
		return;
	}

	ret = udc_dwc3_init(dev);
	if (ret != 0) {
		LOG_ERR("escalation: init failed (%d), core left unconfigured", ret);
		udc_unlock_internal(dev);
		return;
	}

	ret = udc_dwc3_enable(dev);
	if (ret != 0) {
		LOG_ERR("escalation: enable failed (%d)", ret);
	}

	udc_unlock_internal(dev);
}
#endif /* UDC_DWC3_SETUP_STUCK_RESET */

static void udc_dwc3_watchdog_worker(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct udc_dwc3_data *const priv =
		CONTAINER_OF(dwork, struct udc_dwc3_data, watchdog_dwork);
	const struct device *const dev = priv->dev;
	struct udc_dwc3_ep_data *wd_ep;

	/*
	 * A SETUP TRB is armed speculatively and then waits on the host, so its
	 * age says nothing: the bus can sit idle for minutes with the endpoint
	 * perfectly healthy.  DSTS.RXFIFOEMPTY is what separates the two cases.
	 *
	 * uart_v6_1355 shows the split cleanly.  Six event-write stalls: the four
	 * that recovered all read RXFIFOEMPTY set - nothing was stuck, only the
	 * event write was late - and each was rescued by the DATA/STATUS watchdog
	 * firing udc_dwc3_recover(), which issues SET STALL on EP0-OUT.  The
	 * pending XFERCOMPLETE landed immediately after each one.
	 *
	 * Set Stall, not End Transfer.  recover() has never issued an End Transfer
	 * - the machinery for that (ctrl_recovery_pending) is unreachable, as noted
	 * where it is defined.  An earlier version of this comment said otherwise
	 * and led a reviewer to conclude the driver had a broken End-Transfer
	 * recovery; it does not, because that was never the mechanism that worked.
	 *
	 * The two at the final wedge read RXFIFOEMPTY clear, with EP0-OUT holding a
	 * Control-Setup TRB still owned by the controller: a SETUP received and
	 * never retired.  No watchdog was armed for that stage, so nothing issued
	 * the Set Stall that had worked four times already, and the endpoint
	 * stayed dead.  Acknowledging the event ring did not help - two skips
	 * released the credits and the core did not move.
	 *
	 * DSTS.RXFIFOEMPTY alone was a sound proxy only while EP0-OUT was the one
	 * OUT endpoint enabled - UVC declares only 0x81 - so nothing else could put
	 * data in the RxFIFO.  CDC+Video breaks that: device mode has a single
	 * RxFIFO shared by every OUT endpoint, so bulk OUT data keeps the bit clear
	 * while EP0-OUT waits, healthy, for a SETUP that has not arrived.
	 *
	 * The gate below is therefore three tests, not one: the EP0-OUT SETUP TRB
	 * must still be owned by the controller (the only endpoint-specific
	 * evidence there is - GDBGFIFOSPACE reads the shared FIFO and so reports
	 * the same occupancy for every OUT endpoint), the FIFO must be non-empty,
	 * and nothing anywhere must have retired since the SETUP was armed.
	 */
	if (priv->watchdog_type == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		const struct udc_dwc3_config *const cfg = dev->config;
		const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
		const uint32_t dsts = sys_read32(base + UDC_DWC3_DSTS);
		struct udc_dwc3_ep_data *const ep0_out = &cfg->ep_data_out[0];
		const uint32_t trb_ctrl = ep0_out->trb_buf[ep0_out->tail].ctrl;
		const bool moved =
			priv->ctrl_setup_done != priv->ctrl_setup_wd_snap_setup ||
			priv->nonctrl_done != priv->ctrl_setup_wd_snap_nonctrl;

		/*
		 * The SETUP TRB is the only endpoint-specific evidence available.
		 * Still owned by the controller means our SETUP has not been
		 * retired; handed back means it has, and a completion is simply on
		 * its way. Nothing to recover in that case.
		 */
		if ((trb_ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U) {
			priv->ctrl_setup_wd_retired++;
			return;
		}

		if ((dsts & UDC_DWC3_DSTS_RXFIFOEMPTY) != 0U) {
			priv->ctrl_setup_wd_idle++;
			k_work_reschedule(&priv->watchdog_dwork,
					  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
			return;
		}

		/*
		 * Occupancy alone is not evidence. There is one RxFIFO for every OUT
		 * endpoint in device mode, so with a bulk OUT endpoint enabled -
		 * CDC+Video enables several - RXFIFOEMPTY can be clear because of
		 * someone else's data while EP0-OUT waits, perfectly healthy, for a
		 * SETUP that has not arrived. Stalling EP0 then would manufacture
		 * the control failures this watchdog exists to prevent.
		 *
		 * A wedge stops everything, so require that nothing at all retired
		 * while this SETUP was outstanding. Re-mark and wait otherwise.
		 */
		if (moved) {
			priv->ctrl_setup_wd_busy++;
			priv->ctrl_setup_wd_snap_setup = priv->ctrl_setup_done;
			priv->ctrl_setup_wd_snap_nonctrl = priv->nonctrl_done;
			k_work_reschedule(&priv->watchdog_dwork,
					  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
			return;
		}

		priv->ctrl_setup_wd_fire++;
		LOG_ERR("SETUP outstanding for %u ms, TRB still owned by the core and "
			"nothing retired meanwhile (DSTS 0x%08x, TRB ctrl 0x%08x): a "
			"received SETUP has not been retired, stalling EP0-OUT (fired "
			"%u; suppressed idle %u, busy %u, retired %u)",
			CONFIG_UDC_DWC3_RECOVERY_TIMEOUT, dsts, trb_ctrl,
			priv->ctrl_setup_wd_fire, priv->ctrl_setup_wd_idle,
			priv->ctrl_setup_wd_busy, priv->ctrl_setup_wd_retired);

		/*
		 * Read the core's own state before anything is done to it. Set Stall
		 * below has cleared this condition before and would erase the evidence
		 * of what it cleared.
		 */
		udc_dwc3_core_state_dump(dev);

		/*
		 * Last, because it is the only part of this that issues a command.
		 * udc_dwc3_depcmd() bounds its own wait and reports a timeout rather
		 * than hanging, but on a core that is already stuck it can still cost
		 * that timeout - so everything passive is logged before we get here.
		 * EPSTATE is returned in DEPCMDPAR2.
		 */
		{
			const uint32_t epn = ep0_out->epn;

			priv->depcmd_no_sleep = true;
			udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(epn),
					UDC_DWC3_DEPCMD_DEPGETSTATE);
			priv->depcmd_no_sleep = false;

			LOG_ERR("  CORE: EP0-OUT EPSTATE=0x%08x",
				sys_read32(base + UDC_DWC3_DEPCMDPAR2(epn)));
		}

		/*
		 * Cheapest recovery first, and the one that tells us what this is.
		 *
		 * Databook 3.2.2.6: a TRB whose HWO went 0 -> 1 needs an Update
		 * Transfer, because "the hardware uses this information to re-cache
		 * the TRB". The control arm path uses Start Transfer and never
		 * re-caches, so a descriptor the core sampled before the arming store
		 * landed stays stale for ever - which looks exactly like this: data
		 * held in the RxFIFO, an armed TRB by software's reading, and no
		 * event, because XferNotReady has no SETUP encoding to report it with.
		 *
		 * One attempt per stuck episode, marked by ctrl_setup_done so that a
		 * SETUP retiring in between counts as a fresh episode rather than a
		 * repeat. If it works the SETUP retires, the next expiry sees progress
		 * and suppresses itself, and the host notices nothing at all: no
		 * stall, no reset, no re-enumeration. If it does not, the escalation
		 * below runs as before.
		 */
		if (priv->ctrl_setup_done != priv->ctrl_setup_wd_upd_mark ||
		    priv->ctrl_setup_wd_updxfer == 0U) {
			priv->ctrl_setup_wd_updxfer++;
			priv->ctrl_setup_wd_upd_mark = priv->ctrl_setup_done;

			priv->depcmd_no_sleep = true;
			udc_dwc3_depcmd_update_xfer(dev, ep0_out);
			priv->depcmd_no_sleep = false;

			LOG_ERR("  re-cached the EP0-OUT descriptor with Update Transfer "
				"(attempt %u): if the SETUP retires now, the core was "
				"holding a stale HWO=0 and this is a descriptor "
				"visibility fault, not a controller refusal",
				priv->ctrl_setup_wd_updxfer);

			k_work_reschedule(&priv->watchdog_dwork,
					  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
			return;
		}

#ifdef UDC_DWC3_SETUP_STUCK_RESET
		/*
		 * Two fires with no SETUP retired between them means the Set Stall
		 * issued last time did not clear it.  Comparing ctrl_setup_done rather
		 * than counting beats is what makes this specific: any SETUP completing
		 * in between moves the mark, and the second fire is then a fresh fault
		 * rather than the same one persisting.
		 */
		if (priv->ctrl_setup_wd_fire > 1U &&
		    priv->ctrl_setup_done == priv->ctrl_setup_wd_mark) {
			priv->ctrl_setup_wd_mark = priv->ctrl_setup_done;
			priv->ctrl_setup_wd_reset++;
			udc_dwc3_setup_stuck_reset(dev);
			return;
		}
#endif

		priv->ctrl_setup_wd_mark = priv->ctrl_setup_done;
	}

	/*
	 * A DATA or STATUS stage times out and comes straight here, and until now
	 * that path recorded NOTHING - so the one moment control actually broke
	 * was the one moment with no state captured, and the last three wedges had
	 * to be reconstructed backwards from the tail of the log. That guessing is
	 * what this removes: it says which stage was being guarded and what both
	 * control endpoints held when it expired.
	 *
	 * First few only. The fire repeats in bursts of a hundred and more once it
	 * starts, and under LOG_MODE_MINIMAL every line is written out of the UART
	 * synchronously - dumping all of them would itself change what is being
	 * measured. The first one is the one that matters.
	 */
	/*
	 * Snapshot watchdog_ep ONCE. This worker runs on the system work queue and
	 * reads it without the UDC mutex, while udc_dwc3_on_ctrl(), ep_disable()
	 * and drop_xfer_state() all clear it from the UDC work queue. Re-reading
	 * the field between a NULL check and a dereference is a fault, not merely a
	 * stale read. The endpoint objects are static (cfg->ep_data_*), so a
	 * snapshot cannot dangle - at worst it names an endpoint whose stage has
	 * just completed, and Update Transfer against a completed resource is
	 * detected and ignored by the controller (databook 3.2.2.6).
	 */
	wd_ep = priv->watchdog_ep;

	if (priv->ctrl_wd_dump < UDC_DWC3_CTRL_WD_DUMP_FIRST) {
		const struct udc_dwc3_config *const cfg = dev->config;
		const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
		const volatile struct udc_dwc3_trb *const o =
			cfg->ep_data_out[0].trb_buf;
		const volatile struct udc_dwc3_trb *const i =
			cfg->ep_data_in[0].trb_buf;

		priv->ctrl_wd_dump++;

		LOG_ERR("control watchdog #%u: guarding %s, type 0x%02x | "
			"EP00 busy=%u ctrl=0x%08x sts=0x%08x | "
			"EP80 busy=%u ctrl=0x%08x sts=0x%08x | "
			"setup %u status %u decline %u | DSTS 0x%08x",
			priv->ctrl_wd_dump,
			wd_ep == NULL ? "nothing" :
				(USB_EP_DIR_IS_IN(wd_ep->cfg.addr) ? "EP80" : "EP00"),
			priv->watchdog_type,
			udc_ep_is_busy(&cfg->ep_data_out[0].cfg) ? 1U : 0U,
			o[0].ctrl, o[0].status,
			udc_ep_is_busy(&cfg->ep_data_in[0].cfg) ? 1U : 0U,
			i[0].ctrl, i[0].status,
			priv->ctrl_setup_done, priv->ctrl_status_done,
			priv->ctrl_decline,
			sys_read32(base + UDC_DWC3_DSTS));
	}

	/*
	 * Try the cheap remedy on THIS stage before Set Stall, whatever stage it is.
	 *
	 * The same re-cache already existed but was gated inside the SETUP branch
	 * above, so it never ran for a DATA or STATUS stage - updxfer read 0 through
	 * every wedge in uart_02sep_0001_noreclaim, all of which fired with type
	 * 0x50 (CONTROL_DATA). An earlier "the re-cache does not help" result was
	 * measured on a stuck SETUP and does not carry over to this stage.
	 *
	 * Update Transfer only asks the controller to re-read the descriptor;
	 * databook 3.2.2.6 says issuing it against a resource that has already
	 * completed is detected and ignored, so it cannot do the damage that ending
	 * a control transfer did. One attempt per episode, marked by the stage
	 * counters so a stage retiring in between counts as a fresh episode.
	 */
	if (wd_ep != NULL &&
	    (priv->ctrl_setup_done + priv->ctrl_status_done) != priv->ctrl_wd_upd_mark) {
		priv->ctrl_wd_upd_mark = priv->ctrl_setup_done + priv->ctrl_status_done;
		priv->ctrl_setup_wd_updxfer++;

		priv->depcmd_no_sleep = true;
		udc_dwc3_depcmd_update_xfer(dev, wd_ep);
		priv->depcmd_no_sleep = false;

		LOG_ERR("re-cached EP%02x (type 0x%02x) with Update Transfer "
			"(attempt %u): if the stage completes now, the controller was "
			"holding a stale descriptor",
			wd_ep->cfg.addr, priv->watchdog_type,
			priv->ctrl_setup_wd_updxfer);

		k_work_reschedule(&priv->watchdog_dwork,
				  K_MSEC(CONFIG_UDC_DWC3_RECOVERY_TIMEOUT));
		return;
	}

	udc_dwc3_recover(dev);
}

/*
 * Upper bound on the wait for a posted event write to land in the buffer.
 *
 * The core's AXI write is posted: GEVNTCOUNT can become visible while the slot
 * it refers to is still empty.  Reading unconditionally would consume whatever
 * the previous occupant left - a well-formed event from one ring-wrap ago,
 * indistinguishable downstream from a real one.  Hence the sentinel every slot
 * is re-armed with, and the buffer initialised to, at setup.
 *
 * Three properties matter, all learned the hard way:
 *
 *   - It is a TIME budget, not a read count.  A fixed 64-read spin expired on
 *     most events (53558 expiries against 33319 events handled), abandoning the
 *     drain each time until the ring overflowed.  A time also keeps its meaning
 *     across core clocks.
 *   - Reads are PACED.  Unthrottled polling issues back-to-back Wishbone reads
 *     against the same RAM the controller is writing over AXI, holding off the
 *     very event being waited for.
 *   - Expiry is COUNTED, not logged per occurrence.  At 115200 baud tens of
 *     thousands of error lines would themselves starve the ring; "dwc3 evt"
 *     reports the total.
 */
/*
 * Constraints the event buffer has to satisfy, checked here rather than
 * discovered on a rig: the size must be a multiple of four and at least 32
 * bytes, and it has to be a power of two for the size-alignment above to be
 * expressible.
 */
BUILD_ASSERT(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) >= 32,
	     "DWC3 event buffer must be at least 32 bytes");
/*
 * The local drain buffer must be able to hold a whole pass.
 *
 * udc_dwc3_evt_drain() clamps 'want' to CONFIG_UDC_DWC3_EVENTS_NUM and then
 * indexes evt_copy by it, so the two sizes are not merely related - one bounds
 * the other. Sizing evt_copy independently (a fixed 16, say) would turn a
 * future increase of the ring into a silent overrun of the hottest buffer in
 * the driver rather than a build failure, which is the wrong direction to
 * fail in. evt_copy lives in the static per-instance state, not on a stack,
 * so tracking the ring costs bytes of .bss and nothing else.
 */
BUILD_ASSERT(ARRAY_SIZE(((struct udc_dwc3_data *)0)->evt_copy) >=
	     CONFIG_UDC_DWC3_EVENTS_NUM,
	     "evt_copy must hold a full drain: udc_dwc3_evt_drain() clamps 'want' "
	     "to CONFIG_UDC_DWC3_EVENTS_NUM and indexes evt_copy by it");

/*
 * Two separate reasons, recorded because the databook does not give either and
 * reading it alone leads the wrong way: GEVNTSIZ.EVENTSIZ is a 16-bit byte
 * count, so the databook permits up to 64KB and says nothing about 64 bytes.
 *
 * The ring cannot grow because it lives in the AXI block on this part, not in
 * system memory, and the block has a hard limit. GEVNTADR pointing at
 * 0xb1000000 looks like ordinary DRAM and is not.
 *
 * evt_copy is the second reason and is independent of the first: it is the
 * local copy taken from the ring so credit can be returned before the events
 * are processed, and it is to stay at 64 bytes whatever the ring does.
 */
BUILD_ASSERT(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) <= 64,
	     "DWC3 event ring is capped by the AXI block on this part, and "
	     "evt_copy - the local pre-credit copy - stays at 64 bytes");

/*
 * How many times to LOOK for the event word before giving up, rather than how
 * long to wait for it.
 *
 * A wall-clock deadline was the wrong instrument. This runs on a work queue
 * thread that can be descheduled mid-wait, so the deadline expired while nobody
 * was looking and the driver blamed a write that had never been given the chance
 * to be late: uart_v5_3.log caught it at 38 polls across 3384 us against a 250 us
 * deadline, 99% of the window unobserved.
 *
 * A poll count cannot be fooled that way. 64 looks is 64 real observations of the
 * slot however the scheduler behaves, so "still zero" now means it. waited_us is
 * still reported next to it: far above the poll count says we were preempted
 * between looks, which changes how much time the word actually had but no longer
 * changes whether we looked.
 */
#define UDC_DWC3_EVT_ARRIVE_MAX_POLLS 64u
/*
 * Generic command used ONLY to make the controller write an event, when the slot
 * the drain is waiting on will not fill on its own.
 *
 * SPEC, Programming Guide 3.30b section 3.2.1 "Device Generic Command Structure",
 * Table 3-2, command 02h "Set Periodic Parameters" (p.313):
 *   "Note: Currently, the controller does not use the programmed value."
 * It is therefore the one generic command with no effect on the controller. The
 * others all do something: 04h/05h set the scratchpad address, 08h transmits a
 * device notification on the bus, 09h/0Ah flush FIFOs, 0Ch sets an endpoint NRDY,
 * 10h runs a bus loopback test. A parameter of 0 is explicitly a valid setting
 * ("if the value is greater than 125 us, then the software must program a value
 * of zero").
 *
 * With CmdIOC it completes into device event 10, which Table 3-8 calls "Generic
 * Command Complete Event (CmdCmplt)" and which this driver already dispatches to
 * a bare break. Nothing acts on it - the point is the WRITE, not the content.
 *
 * NOT an endpoint command: an EPCmdCmplt carries an endpoint number but no
 * command type, so udc_dwc3_on_ep_cmd_cmplt() would take it for the completion of
 * a real End Transfer - clearing end_xfer_pending, re-arming a control stage
 * mid-recovery, or running a deferred resume early. That is the path changes
 * 43/45/47/49 exist to protect.
 */
#define UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS			0x02u

/*
 * Ceiling on how full the event ring may be for a force to be worth issuing.
 * Half the ring leaves room for the forced event itself and for whatever the
 * link is doing at the time.
 */
#define UDC_DWC3_EVT_FORCE_MAX_GEVNTCOUNT			\
	((CONFIG_UDC_DWC3_EVENTS_NUM / 2u) * sizeof(uint32_t))
/*
 * Force at the first give-up of a give-up run and every this many after, so a slot
 * that stays empty cannot turn into a command storm. At ~400 us per give-up this
 * is roughly one command per 26 ms while stuck.
 */
/*
 * Minimum wall-clock gap between forced generic commands.
 *
 * Replaces a count-based gate that did not do what it looked like it did. It
 * counted consecutive give-ups on the SAME slot, and reset whenever the stall
 * moved - so in uart_v6_3 every stall report carried run=1, the gate was true
 * every time, and the force fired on all ~192 give-ups a second instead of the
 * intended one in eight. Wall-clock bounds it absolutely at two a second no
 * matter how the stall moves between slots, which matters because every forced
 * command lands an event of its own in a ring that is already not draining.
 */
#define UDC_DWC3_EVT_FORCE_MIN_GAP_MS				500u

/*
 * How long a stalled drain keeps re-submitting itself before handing the slot
 * to the heartbeat.
 *
 * A stalled pass used to resubmit nothing at all, on the reasoning that the
 * interrupt is level-sensitive on GEVNTCOUNT and would re-enter the worker by
 * itself. uart_v6_3 shows that failing: two heartbeat reports caught the head
 * slot HOLDING a valid event, with the give-up run at one give-up and 1.7-2.0
 * seconds old, GEVNTCOUNT at 64 B - a completely full ring - and evt_rearm
 * frozen at 283 for the whole run. The event had arrived and nothing looked at
 * it until the 1 Hz heartbeat. A full ring stops the USB, so that second is
 * not free.
 *
 * Bounded rather than unconditional because each retry costs a 400 us poll,
 * and retrying forever would hold the work queue against the USBD thread and
 * the heartbeat itself. 100 ms covers anything that is merely late and leaves
 * the genuinely stuck case to the slower path that is designed for it.
 */
#define UDC_DWC3_EVT_GAVEUP_RETRY_MS				100u
#define UDC_DWC3_EVT_ARRIVE_POLL_US 1u
#define UDC_DWC3_EVT_GAVEUP_LOG_EVERY 1021u
/*
 * How often the running totals are reported without being asked for.
 *
 * "dwc3 evt" prints them on demand, but a soak run is unattended - the capture
 * that produced these numbers had no shell command typed in it at all - so a
 * counter only reachable from the shell is a counter nobody reads.
 *
 * One line per few thousand events is the whole cost: at the rate that capture
 * ran, four lines for the entire session against the 53558 the per-occurrence
 * version emitted. Rare enough that it cannot become the load it is measuring,
 * frequent enough to show a trend while the run is still going.
 */
#define UDC_DWC3_EVT_STATS_EVERY 4096u

/*
 * Make the controller write an event, to release a slot that will not fill on its
 * own. See UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS for why this command and not another.
 *
 * Fire and forget: the completion is device event 10, which the dispatch ignores.
 * Nothing here waits for it - waiting is what got us into trouble elsewhere.
 */
static void udc_dwc3_evt_force(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	/*
	 * Only while the ring still has room. Every forced command lands an event
	 * of its own, and while the head slot is stuck that event queues BEHIND it.
	 * The count-based gate this replaced could fill all
	 * CONFIG_UDC_DWC3_EVENTS_NUM slots in about a tenth of a second and
	 * manufacture the very overflow the block acknowledge exists to avoid;
	 * UDC_DWC3_EVT_FORCE_MIN_GAP_MS bounds the rate, this bounds the depth.
	 * Forcing is a nudge to be taken while it is free, not a retry loop.
	 */
	if (udc_dwc3_gevntcount(base) > UDC_DWC3_EVT_FORCE_MAX_GEVNTCOUNT) {
		return;
	}

	/*
	 * DGCMD bit 10 CMDACT, R/W1S: software sets it to start the generic command and
	 * the controller clears it when done.  Set therefore means the previous command
	 * is still executing.  The databook does not define writing over that, and this
	 * is not the place to find out - forces are milliseconds apart and a generic
	 * command retires in microseconds, so this should never be taken.
	 *
	 * Deliberately does NOT save/clear/restore GUSB2PHYCFG the way udc_dwc3_depcmd()
	 * does.  The requirement it serves is scoped to DCFG.DevSpd 2.0-only mode during
	 * Disconnect handling; this controller enumerates at SuperSpeed and this is not
	 * disconnect.  Doing it anyway would cost more than it bought: it is a
	 * read-modify-write that depcmd() performs on the usbd thread under the UDC mutex,
	 * while this runs unlocked on the work queue, so it would add a real race to
	 * remove a hazard this core does not have.  GUSB2PHYCFGn is also on the CSftRst
	 * exception list, so a reset did not disturb it.
	 */
	if (sys_read32(base + UDC_DWC3_DGCMD) & UDC_DWC3_DGCMD_ACT) {
		return;
	}

	sys_write32(0, base + UDC_DWC3_DGCMDPAR);
	sys_write32(UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS | UDC_DWC3_DGCMD_IOC |
		    UDC_DWC3_DGCMD_ACT, base + UDC_DWC3_DGCMD);

	LOG_WRN_RATELIMIT("forced a generic command to unstick slot %u",
			  priv->evt_next);
}

/*
 * Wait for the FIRST word of a pass, which is the only one worth waiting for:
 * there is nothing copied yet, so returning without it would just spin.
 *
 * Returns the event, or UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE if it never arrived -
 * NOT 0.  Zero is a legal event word here; the sentinel is what marks a slot as
 * unwritten, so a caller testing == 0 would be wrong.  In that case a generic
 * command
 * has been issued to force one.
 */
static uint32_t udc_dwc3_evt_wait_first(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t t0 = k_cycle_get_32();
	uint32_t polls = 0;
	uint32_t evt;
	uint32_t waited_us;

	priv->evt_late++;

	do {
		k_busy_wait(UDC_DWC3_EVT_ARRIVE_POLL_US);
		polls++;
		evt = cfg->evt_buf[priv->evt_next];
	} while (evt == UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE &&
		 polls < UDC_DWC3_EVT_ARRIVE_MAX_POLLS);

	waited_us = k_cyc_to_us_near32(k_cycle_get_32() - t0);

	if (evt != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
		if (polls > priv->evt_late_polls_max) {
			priv->evt_late_polls_max = polls;
		}

		if (waited_us > priv->evt_late_us_max) {
			priv->evt_late_us_max = waited_us;
		}

		return evt;
	}

	priv->evt_gaveup++;

	if (priv->evt_gaveup_run > 0 && priv->evt_gaveup_slot == priv->evt_next) {
		priv->evt_gaveup_run++;
	} else {
		priv->evt_gaveup_slot = priv->evt_next;
		priv->evt_gaveup_run = 1;
		priv->evt_gaveup_t0 = t0;
		priv->evt_gaveup_logged = false;
		priv->evt_gaveup_quiet = true;
		priv->evt_missed_counted = false;
		priv->evt_gaveup_gc0 = udc_dwc3_gevntcount(base);

		/*
		 * Population data for the look-ahead in udc_dwc3_evt_skip_dead_slot():
		 * it can only advance past more than one slot when the controller says
		 * it owes more than one event, so how often gc0 exceeds a single word
		 * is what decides whether that path is worth anything. Counted rather
		 * than logged - a quiet run must stay quiet, because the interval it is
		 * timing is shorter than one console line.
		 */
		if (priv->evt_gaveup_gc0 > priv->evt_gaveup_gc0_max) {
			priv->evt_gaveup_gc0_max = priv->evt_gaveup_gc0;
		}
		if (priv->evt_gaveup_gc0 > sizeof(uint32_t)) {
			priv->evt_gaveup_multi++;
		}
	}

	/*
	 * Say once, per run, that this is a LOST write rather than a late one -
	 * and say which kind, because the two need opposite responses.
	 *
	 * GEVNTCOUNT frozen at the value the run opened with means the controller
	 * has placed nothing at all since: the databook's "events are queued up
	 * internally... when software frees up Event Buffer space, the queued up
	 * events are written out". The core is waiting for US, and only an
	 * acknowledge can release it. GEVNTCOUNT advancing means the core is still
	 * writing and this one slot was skipped - a different fault entirely.
	 *
	 * uart_v6_14 was the frozen kind: raw=0x00000010 on all sixteen samples
	 * across 25 s and 16,113 give-ups, with ~53 forced commands landing
	 * nothing.
	 */
	if (!priv->evt_missed_counted &&
	    k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_gaveup_t0) >=
						UDC_DWC3_EVT_MISSED_MS) {
		const uint32_t gc_now = udc_dwc3_gevntcount(base);
		const bool frozen = (gc_now == priv->evt_gaveup_gc0);

		priv->evt_missed_counted = true;
		priv->evt_missed++;
		if (frozen) {
			priv->evt_missed_frozen++;
		}

		priv->evt_gaveup_quiet = false;
		LOG_ERR("slot %u WRITE LOST, not late: empty %u ms over %u give-ups, "
			"GEVNTCOUNT %s (%u B now, %u B when the run opened) - %s",
			priv->evt_next,
			k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_gaveup_t0),
			priv->evt_gaveup_run,
			frozen ? "FROZEN" : "advancing",
			gc_now, priv->evt_gaveup_gc0,
			frozen ? "core has placed nothing since; cause unknown"
			       : "core still writing, this slot skipped");

		/*
		 * NO REGISTER DUMP HERE. This runs inside udc_dwc3_evt_wait_first(),
		 * which runs inside the drain: nine LOG_ERR lines under
		 * LOG_MODE_MINIMAL is ~600 characters of synchronous UART, about
		 * 52 ms at 115200, with event processing stopped and an ISOC stream
		 * running - some 400 service intervals. A diagnostic that halts the
		 * path it is diagnosing is not observation, and calling it that was
		 * wrong. The one line above is the whole cost on this path.
		 *
		 * The dump still fires from udc_dwc3_heartbeat_worker(), where the
		 * fault has already stopped traffic and there is nothing left to
		 * disturb.
		 */
	}

	/*
	 * Anything emitted from here on sits INSIDE the window that
	 * evt_gaveup_t0 is timing, and at 115200 baud one of these lines is 7.5 ms -
	 * larger than the latency being measured. The run is marked so the fill
	 * path knows not to believe its own clock.
	 */
	if (priv->evt_gaveup % UDC_DWC3_EVT_GAVEUP_LOG_EVERY == 1) {
		priv->evt_gaveup_logged = true;
		priv->evt_gaveup_quiet = false;
		LOG_DBG("slot %u empty: polls=%u waited=%uus gc %u B raw=0x%08x "
			"hwm=%u run=%u (%u so far)",
			priv->evt_next, polls, waited_us,
			udc_dwc3_gevntcount(base),
			sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)),
			priv->evt_gevntcount_hwm, priv->evt_gaveup_run,
			priv->evt_gaveup);
	}

	/*
	 * Force at the first discovery of a stall, then no more often than
	 * UDC_DWC3_EVT_FORCE_MIN_GAP_MS. The first one is free and is the whole
	 * point - if a write is merely sitting unflushed, one command shakes it
	 * loose immediately. Repeating it at give-up rate does not help and does
	 * harm, because each forced command adds an event to a ring whose head is
	 * already stuck.
	 *
	 * The force logs, so it contaminates the timing window the same way the
	 * report above does.
	 */
	if (!priv->evt_force_ever ||
	    k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_force_t0) >=
						UDC_DWC3_EVT_FORCE_MIN_GAP_MS) {
		priv->evt_gaveup_quiet = false;
		udc_dwc3_evt_force(dev);
		priv->evt_force_t0 = k_cycle_get_32();
		priv->evt_force_ever = true;
	}

	/* Nothing landed: report it the same way an unwritten slot reads. */
	return UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
}

/*
 * Drain the ring into priv->evt_copy and hand every slot back in ONE acknowledge,
 * before a single event is dispatched.
 *
 * This is the whole point of the split: dispatching holds slots for as long as
 * handle_event() takes - on a work queue shared with the endpoint worker, the
 * watchdog and the heartbeat - whereas copying is a few memory accesses. The
 * controller gets its ring back in microseconds no matter how slow processing is.
 *
 * A single acknowledge of everything copied is also what the databook requires to
 * escape an overflow: "software must free up space in the Event Buffer by
 * acknowledging more than 1 event".
 *
 * Returns how many events were copied.
 */
/*
 * Free one credit from a ring that is full and whose head will not become
 * readable. Returns true when a slot was skipped.
 *
 * A full event ring is a recoverable condition, and the databook says how it
 * recovers: "During this time, events are queued up internally. When software
 * frees up Event Buffer space, the queued up events are written out and the USB
 * returns to [normal]", and for the overflow event itself, "software must free
 * up space in the Event Buffer by acknowledging more than 1 event (writing a
 * value greater than 4 to the GEVNTCOUNTn register)".
 *
 * Acknowledging IS the recovery - and it is the one thing this driver could not
 * do in that state. The ring is strict FIFO and events are processed in order,
 * so an unreadable head yields nothing processed, and the acknowledge at the
 * end of the drain is conditional on having processed something. Zero bytes are
 * freed, however full the ring is and however long the wait goes on. Polling
 * harder, forcing a generic command, retrying, and the heartbeat kick all try
 * to make the head READABLE; none of them can free space if it never becomes
 * readable. Without this the condition is unrecoverable by construction: the
 * controller has nowhere to write, so it withholds link credits and stops the
 * bus, the host's transfers time out, and its own controller has been observed
 * to declare itself dead trying to recover.
 *
 * GEVNTCOUNT is a byte credit, not a receipt: writing 4 says one event's worth
 * of space is free, and asserts nothing about having understood it. So the
 * escape is to skip the slot deliberately. One event is lost - and if its write
 * never landed, nothing real is lost at all. The slot is zeroed on the way past
 * so a write that arrives afterwards cannot be mistaken for a live event on the
 * next wrap.
 *
 * Gated hard, because discarding events must never become routine: the ring has
 * to be full, a give-up run must be open, and it must have outlived every cheaper
 * remedy. Waiting remains the correct response to a merely late write.
 */
static bool udc_dwc3_evt_skip_dead_slot(const struct device *const dev,
					const uint32_t gc, const bool frozen,
					const uint32_t gaveup_ms)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	/*
	 * The caller decides; this only acts. It used to gate itself on
	 * gc >= UDC_DWC3_EVT_RING_FULL_BYTES, which made it unreachable in the one
	 * failure it exists for: the core stops producing events BECAUSE it is
	 * blocked on the entry software has not consumed, so the ring never fills.
	 * Measured at gc = 16 B and gc = 8 B against a 60 B watermark, with skip 0
	 * both times. The level of gc says nothing about whether a slot is dead.
	 */
	/*
	 * How far to advance.
	 *
	 * GEVNTCOUNT is the controller's own statement of how many event words it
	 * owes, so gc/4 bounds the search: never acknowledge more bytes than were
	 * claimed.  DWC3 writes the event buffer in ring order, so a slot holding a
	 * real event BEYOND the dead one proves every sentinel between the two was
	 * issued earlier and is gone - not merely late, since the later write could
	 * not have landed first.  Those slots can therefore be retired in one step,
	 * instead of paying UDC_DWC3_EVT_DEAD_SLOT_MS over again to rediscover for
	 * each what this one already settles; the ring is stalled for every second
	 * of that, and the previous one-slot-per-second rate is why a burst of
	 * missing writes took as long to clear as it did.
	 *
	 * With no valid word downstream there is no such proof, and the skip stays
	 * at one slot: an empty run with nothing behind it is equally consistent
	 * with writes still in flight, and discarding those would be inventing
	 * evidence rather than reading it.
	 */
	uint32_t owed = gc / sizeof(uint32_t);
	uint32_t skip = 1u;

	if (owed > (CONFIG_UDC_DWC3_EVENTS_NUM - 1u)) {
		owed = CONFIG_UDC_DWC3_EVENTS_NUM - 1u;
	}

	for (uint32_t j = 1u; j < owed; j++) {
		const uint32_t idx =
			(priv->evt_next + j) % CONFIG_UDC_DWC3_EVENTS_NUM;

		if (cfg->evt_buf[idx] != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
			skip = j;
			break;
		}
	}

	priv->evt_skipped += skip;

	LOG_ERR("slot %u unreadable for %u ms over %u give-ups (holds 0x%08x, "
		"gc %u B, %s): acknowledging %u slot%s to release the "
		"controller, %u EVENT%s LOST (%u so far)",
		priv->evt_next, gaveup_ms, priv->evt_gaveup_run,
		cfg->evt_buf[priv->evt_next], gc,
		frozen ? "GEVNTCOUNT frozen - core blocked on us"
		       : "GEVNTCOUNT advancing - core still writing elsewhere",
		skip, skip == 1u ? "" : "s",
		skip, skip == 1u ? "" : "S",
		priv->evt_skipped);

	for (uint32_t j = 0u; j < skip; j++) {
		cfg->evt_buf[(priv->evt_next + j) % CONFIG_UDC_DWC3_EVENTS_NUM] =
			UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
	}
	priv->evt_next = (priv->evt_next + skip) % CONFIG_UDC_DWC3_EVENTS_NUM;
	sys_write32(skip * sizeof(uint32_t), base + UDC_DWC3_GEVNTCOUNT(0));

	priv->evt_gaveup_run = 0;
	priv->evt_gaveup_logged = false;

	return true;
}

static uint32_t udc_dwc3_evt_drain(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	/*
	 * The one and only read of GEVNTCOUNT for this pass, taken before any event
	 * is processed, which is exactly what section 1.2.56 requires - see the
	 * clock-crossing quotation at UDC_DWC3_GEVNTCOUNT_MASK. Everything below
	 * works from this snapshot; the register is not consulted again until the
	 * acknowledge, because after one it may still be reporting what was just
	 * given back.
	 */
	const uint32_t gc = udc_dwc3_gevntcount(base);
	uint32_t want = gc / sizeof(uint32_t);
	uint32_t n = 0;

	priv->evt_drain_gaveup = false;
	priv->evt_drain_midzero = false;

	if (gc > priv->evt_gevntcount_hwm) {
		priv->evt_gevntcount_hwm = gc;
	}

	/*
	 * Hard bound before indexing. The count comes from a register; if it is ever
	 * wrong, an unclamped loop writes past evt_copy. The ring cannot hold more
	 * than CONFIG_UDC_DWC3_EVENTS_NUM events, so anything above that is bogus.
	 */
	if (want > CONFIG_UDC_DWC3_EVENTS_NUM) {
		LOG_ERR_RATELIMIT("GEVNTCOUNT reports %u B, more than the %u B ring",
				  gc, (unsigned int)(CONFIG_UDC_DWC3_EVENTS_NUM *
						     sizeof(uint32_t)));
		want = CONFIG_UDC_DWC3_EVENTS_NUM;
	}

	while (n < want) {
		uint32_t evt = cfg->evt_buf[priv->evt_next];

		if (evt == UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
			/*
			 * Mid-pass: do not wait. There is work in hand, so hand the
			 * slots back and process it; this one gets another look on the
			 * next entry, by which time it has had a full round trip to
			 * land. Only the first word is worth waiting for.
			 *
			 * This must NOT mark the pass stalled, and marking it was a
			 * defect - the one uart_v6_6 was built on. The tail reads
			 * evt_drain_gaveup as "do not come back", and lifts that
			 * refusal only for a pass that opened a stall RUN. This path
			 * opens no run (runs are opened in udc_dwc3_evt_wait_first(),
			 * the head-slot path, which is not reached here), so the
			 * refusal stood unconditionally: the drain announced it would
			 * take another look and then denied itself the entry to do it.
			 * Nothing else re-entered either - the controller raises no
			 * fresh interrupt for events already written and counted - so
			 * the ring sat with GEVNTCOUNT > 0 until the heartbeat kicked
			 * it UDC_DWC3_EVT_IDLE_KICK_MS later.
			 *
			 * That is what uart_v6_6 shows. All five "drain IDLE 200 ms
			 * with no give-up run" reports carry a valid event in the head
			 * slot, and each is followed by the kick counter stepping by
			 * one - 0->1, 1->2, 2->3, 3->4, 4->5, five for five, with no
			 * kick lacking a matching report. The heartbeat was the only
			 * thing restarting the drain for the whole 20-minute soak.
			 *
			 * Consuming events IS progress, so this pass earns its re-entry
			 * on the same grounds udc_dwc3_evt_skip_dead_slot() does. The
			 * re-entry cannot spin: the slot is the HEAD slot next time, so
			 * it goes through udc_dwc3_evt_wait_first() and comes under the
			 * bounded poll budget and the stall-run governor.
			 */
			if (n > 0) {
				priv->evt_midzero++;
				priv->evt_drain_midzero = true;
				break;
			}

			evt = udc_dwc3_evt_wait_first(dev);
			if (evt == UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
				/*
				 * No decision here any more. The drain records the
				 * run - slot, age, give-ups, gc when it opened - and
				 * the 200 ms heartbeat decides whether the slot is
				 * dead. Giving up on an event costs it permanently,
				 * and that judgement does not belong in a path that
				 * re-enters at roughly 1 kHz on a single sample.
				 */
				priv->evt_drain_gaveup = true;
				break;
			}
		}

		if (priv->evt_gaveup_run > 0 &&
		    priv->evt_gaveup_slot == priv->evt_next) {
			/*
			 * How long the slot actually stayed empty - the number the RTL
			 * question turns on, so it is taken only from runs that printed
			 * nothing while it was being timed. A logged or forced run has
			 * milliseconds of synchronous console inside the interval and
			 * measures the UART, not the controller: uart_v6_3 reported
			 * 7.6-11.3 us-scaled figures that matched its own log lines to
			 * within a few per cent, on the 1-in-1021 stalls that were the
			 * only ones it measured at all.
			 *
			 * Every quiet run is sampled, so this is a max over essentially
			 * all of them, and it is reported in the periodic stats line
			 * rather than per occurrence.
			 */
			if (priv->evt_gaveup_quiet) {
				const uint32_t us = k_cyc_to_us_near32(
					k_cycle_get_32() - priv->evt_gaveup_t0);

				if (us > priv->evt_gaveup_us_max) {
					priv->evt_gaveup_us_max = us;
				}
			} else {
				LOG_DBG("slot %u filled after %u give-ups: 0x%08x (%s) "
					"gc0 %u B - interval not timed, this run printed",
					priv->evt_next, priv->evt_gaveup_run, evt,
					udc_dwc3_get_event_name(evt,
						sys_read32(base + UDC_DWC3_DSTS)),
					priv->evt_gaveup_gc0);
			}
			priv->evt_gaveup_run = 0;
			priv->evt_gaveup_logged = false;
		}

		/*
		 * A zero word here is DATA, not absence - and that is new.
		 *
		 * While the free marker was itself zero the two were the same
		 * bit pattern and nothing could tell them apart. Now that a free
		 * slot holds UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE, reaching this
		 * point with 0x00000000 means the controller, or the logic
		 * between it and this RAM, actually placed a zero over the
		 * marker. That is worth knowing precisely because it should not
		 * happen: no valid event encodes as zero except the Reserved
		 * type 4'h0 on physical endpoint 0.
		 *
		 * The likelier explanation is the write path rather than the
		 * core - a 64-bit bus write for a 32-bit event, zero-padding the
		 * other half and clearing the neighbouring slot. If that is what
		 * happens, this counter rises in step with the stalls and names
		 * the cause; if it stays at zero while slots still read as free,
		 * the write is genuinely lost rather than corrupted. Either way
		 * it separates two things that were previously one symptom.
		 *
		 * Logged as well as counted: it is rare enough to be affordable,
		 * and the slot number plus GEVNTCOUNT at the moment it is seen is
		 * what would identify a padding pattern.
		 */
		if (evt == 0U) {
			priv->evt_zero++;
			LOG_DBG("evtword=0 at slot %u (%u so far): the "
				"controller wrote a zero over the free marker - "
				"this is a written word, not a missing one; "
				"gc %u B, next slot 0x%08x",
				priv->evt_next, priv->evt_zero,
				udc_dwc3_gevntcount(base),
				cfg->evt_buf[(priv->evt_next + 1) %
					     CONFIG_UDC_DWC3_EVENTS_NUM]);
		}

		priv->evt_copy[n++] = evt;

		/*
		 * Re-arm BEFORE the acknowledge below. The slot belongs to software
		 * only until then; afterwards the controller may refill it, and a
		 * zero written at that point would destroy a fresh event.
		 */
		cfg->evt_buf[priv->evt_next] = UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
		priv->evt_next = (priv->evt_next + 1) % CONFIG_UDC_DWC3_EVENTS_NUM;
	}

	/*
	 * Acknowledge exactly what was copied - never the entry count.  Crediting
	 * events we did not receive would advance the controller past them for good.
	 *
	 * One write per pass, n events at a time.  The Event Buffer Overflow section
	 * requires software to "free up space ... by acknowledging more than 1 event
	 * (writing a value greater than 4 to the GEVNTCOUNTn register)": crediting a
	 * single slot at a time lets the controller refill it immediately, so an
	 * established overflow can never clear.  uart_v5_9 is that failure - the ring
	 * full for ~15 s at ~4000 overflow events/s while the drain managed ~3300/s.
	 */
	if (n > 0) {
		sys_write32(n * sizeof(uint32_t), base + UDC_DWC3_GEVNTCOUNT(0));
	}

	return n;
}

static void udc_dwc3_event_worker(struct k_work *work)
{
	struct udc_dwc3_data *const priv = CONTAINER_OF(work, struct udc_dwc3_data, event_work);
	const struct device *const dev = priv->dev;
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t n = (priv->evt_worker_runs++, udc_dwc3_evt_drain(dev));

	/* The ring is already back with the controller by this point. */
	for (uint32_t i = 0; i < n; i++) {
		udc_dwc3_handle_event(dev, priv->evt_copy[i]);

		priv->evt_handled++;

		if (priv->evt_handled % UDC_DWC3_EVT_STATS_EVERY == 0) {
			/*
			 * ctrl_desync is here because its log line is capped at the
			 * first UDC_DWC3_CTRL_DESYNC_LOG_FIRST occurrences - without a
			 * counter in this line, a check firing thousands of times looks
			 * exactly like one firing twelve times.
			 */
			LOG_INF("events %u: late %u (worst %u polls/%u us) gaveup %u "
				"(worst empty %u us) isr %u runs %u rearm %u kick %u skip %u "
				"zero %u missed %u/%u desync %u ctrl %u/%u midzero %u "
				"decline %u/%u "
				"setuppending %u unarmed %u startfail %u defer %u trbsts %u "
				"setupwd %u/%u reset %u updxfer %u resync %u recache %u outmisaligned %u/%u xnrdy %u "
				"gc_hwm %u B gc0max %u B multi %u link %u out1 %u/%u out2 %u/%u DSTS 0x%08x",
				priv->evt_handled, priv->evt_late,
				priv->evt_late_polls_max, priv->evt_late_us_max,
				priv->evt_gaveup, priv->evt_gaveup_us_max,
				priv->evt_isr, priv->evt_worker_runs, priv->evt_rearm,
				priv->evt_kick, priv->evt_skipped,
				priv->evt_zero, priv->evt_missed, priv->evt_missed_frozen,
				priv->ctrl_desync,
				priv->ctrl_setup_done, priv->ctrl_status_done,
				priv->evt_midzero, priv->ctrl_decline, priv->ctrl_recover,
				priv->ctrl_setup_pending, priv->ctrl_unarmed,
				priv->ctrl_start_fail,
				priv->ctrl_deferred_arm, priv->ctrl_trbsts_other,
				priv->ctrl_setup_wd_fire, priv->ctrl_setup_wd_idle,
				priv->ctrl_setup_wd_reset,
				priv->ctrl_setup_wd_updxfer,
				priv->ctrl_resync,
				priv->nonctrl_recache,
				priv->out_unaligned, priv->out_unaligned_ctrl,
				priv->xnrdy_nonctrl,
				priv->evt_gevntcount_hwm,
				priv->evt_gaveup_gc0_max, priv->evt_gaveup_multi,
				priv->evt_link_total,
				cfg->num_out_eps > 1 ? cfg->ep_data_out[1].n_arm : 0U,
				cfg->num_out_eps > 1 ? cfg->ep_data_out[1].n_retire : 0U,
				cfg->num_out_eps > 2 ? cfg->ep_data_out[2].n_arm : 0U,
				cfg->num_out_eps > 2 ? cfg->ep_data_out[2].n_retire : 0U,
				/*
				 * DSTS on the periodic line, so COREIDLE and
				 * RXFIFOEMPTY get sampled during HEALTHY STREAMING -
				 * the one case never captured. Both were read only
				 * when control traffic had already stopped, and every
				 * such sample was taken either before the stream
				 * started or at a failure, so "COREIDLE=0" may mean
				 * nothing more than that the video endpoint is busy:
				 * the spec defines the bit across ALL endpoints as
				 * "finished transferring all RxFIFO data to system
				 * memory, writing out all completed descriptors, and
				 * all Event Counts are zero", and it warns the bit
				 * "does not hold a static value".
				 *
				 * One register read folded into a line that already
				 * prints every 4096 events: no new output, so it
				 * cannot perturb the timing it is measuring.
				 */
				sys_read32(base + UDC_DWC3_DSTS));
		}
	}

	sys_clear_bits(base + UDC_DWC3_GEVNTSIZ(0), UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
	cfg->irq_enable_func();

	/*
	 * Re-enter whenever the controller still says there is something to read.
	 *
	 * A clean pass resubmits because more arrived while this one was
	 * dispatching. A STALLED pass resubmits too, for up to
	 * UDC_DWC3_EVT_GAVEUP_RETRY_MS - which is the correction. It used to return
	 * without resubmitting, on the assumption that the level-sensitive
	 * interrupt re-enabled just above would bring the worker back by itself.
	 * uart_v6_3 disproves that: the heartbeat twice found the head slot holding
	 * a valid event, the give-up run one give-up old and nearly two seconds
	 * stale, the ring completely full, and evt_rearm frozen for the entire run.
	 * Nothing had looked. Waiting a whole second for the heartbeat while the
	 * ring is full is what lets the controller run out of room and stop the bus,
	 * so the guard meant to avoid a pointless spin was buying a real outage.
	 */
	if (udc_dwc3_gevntcount(base) > 0U) {
		bool retry = !priv->evt_drain_gaveup;

		/*
		 * A mid-pass zero retries - retry is already true, because that pass
		 * is no longer marked stalled - but it yields first, for the same
		 * reason the stalled-pass retry below does: this queue is cooperative
		 * and so is the usbd thread above it, so a handler that resubmits
		 * itself without ever sleeping starves the thread that decodes SETUP.
		 */
		if (priv->evt_drain_midzero) {
			k_yield();
		}

		/*
		 * A stalled pass is retried only when a stall RUN is open.
		 *
		 * evt_drain_gaveup is set on two different paths and only one of them
		 * establishes a timestamp. The head-slot stall goes through
		 * udc_dwc3_evt_wait_first(), which opens a run and stamps
		 * evt_gaveup_t0. The mid-pass zero - already consumed something, then
		 * found an empty slot - does neither. Testing the age without testing
		 * the run therefore compared against whatever an older run had left
		 * behind, and retried a case that is not a stall at all and was never
		 * meant to be retried.
		 */
		if (!retry && priv->evt_gaveup_run > 0U &&
		    k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt_gaveup_t0) <
						UDC_DWC3_EVT_GAVEUP_RETRY_MS) {
			/*
			 * Yield before going round again.
			 *
			 * This work queue is COOPERATIVE
			 * (CONFIG_SYSTEM_WORKQUEUE_PRIORITY = -1), and so is the USB device
			 * stack's own thread at K_PRIO_COOP(8) - which is the HIGHER
			 * priority of the two, but a cooperative thread is never preempted.
			 * A handler that resubmits itself, with the drain busy-waiting its
			 * whole poll budget and never sleeping, therefore starves that
			 * thread for as long as the retry window lasts. The usbd thread is
			 * what decodes SETUP packets and queues control buffers, so
			 * starving it desynchronises the control state machine - which is
			 * exactly what a retry meant to protect the ring must not do.
			 */
			k_yield();
			retry = true;
		}

		if (retry) {
			priv->evt_rearm++;
			k_work_submit_to_queue(udc_get_work_q(), &priv->event_work);
		}
	}

	/*
	 * Last thing done, so this records when the pass FINISHED - see
	 * UDC_DWC3_EVT_IDLE_KICK_MS for why the exit and not the entry.
	 */
	priv->evt_worker_exit_t0 = k_cycle_get_32();
	priv->evt_worker_ran = true;
}

static void udc_dwc3_irq_handler(void *const ptr)
{
	const struct device *const dev = ptr;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	priv->evt_isr++;

	k_work_submit_to_queue(udc_get_work_q(), &priv->event_work);

	/* Disable further interrupts until all events are processed */
	sys_set_bits(base + UDC_DWC3_GEVNTSIZ(0), UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
	cfg->irq_disable_func();
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
	const struct udc_buf_info bi = *udc_get_buf_info(buf);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	LOG_DBG("enq %p d=%p sz=%u ln=%u EP%02x %u:%u:%u",
		buf, buf->data, buf->size, buf->len, ep_cfg->addr, bi.setup, bi.data, bi.status);

	if (ep_data->cfg.addr == USB_CONTROL_EP_OUT) {
		memset(buf->data, 0x00, buf->size);
	}

	udc_buf_put(ep_cfg, buf);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		struct udc_dwc3_data *const priv = udc_get_private(dev);

		priv->ctrl_enq_t = k_cycle_get_32();
		priv->ctrl_enq_n++;
		priv->ctrl_enq_last = (uint8_t)((bi.setup ? 4U : 0U) |
					       (bi.data ? 2U : 0U) |
					       (bi.status ? 1U : 0U));
		udc_dwc3_ctrl_next(dev);
	} else {
		/* Process this buffer along with other waiting */
		if (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP &&
		    ep_cfg->stat.enabled) {
			LOG_DBG("submitting to EP%02x", ep_cfg->addr);
			k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
		}
	}

	return 0;
}

static int udc_dwc3_ep_dequeue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data =
		CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	/*
	 * SPEC, Programming Guide 3.30b: "it is recommended that software issue an
	 * End Transfer command for the endpoint/transfer resource before
	 * de-allocating the memory."
	 *
	 * Stop the controller before the buffers go back, not after. A TRB the
	 * controller still owns points into the very memory
	 * udc_ep_cancel_queued() is about to hand back to the stack, and nothing
	 * else retires it: for an OUT endpoint the next packet from the host is
	 * then written into a buffer that has been reused, which corrupts whoever
	 * owns it now rather than failing anywhere near here.
	 *
	 * This is a dequeue - the transfer is being abandoned by definition - so
	 * unlike the watchdog path there is no live transfer being torn down
	 * needlessly, and the End Transfer is exactly what is wanted.
	 *
	 * Conditional on the busy claim because the ordinary teardown reaches here
	 * with nothing left to end: usbd_endpoint.c calls udc_ep_disable() and only
	 * then udc_ep_dequeue(), and udc_dwc3_ep_disable() has already cleared
	 * DALEPENA, issued its own End Transfer and dropped the claim. Repeating it
	 * on a disabled endpoint achieves nothing and reports "End Transfer not
	 * issued" on a path that is working correctly. The claim is what says a
	 * descriptor is still armed over the buffers about to be released.
	 */
	/*
	 * NOT on the control endpoints.
	 *
	 * This driver states the hazard itself, in udc_dwc3_ctrl_next_out(): an End
	 * Transfer against a live control endpoint is "the command that has been
	 * observed to hang". It does hang - it cost a cold boot. With CmdAct stuck
	 * on DEPCMD for EP80 no further command can be issued on that endpoint,
	 * including the DEPCFG needed to reconfigure it, so the device can never
	 * enumerate again and the host eventually declares its own controller dead.
	 *
	 * udc_dwc3_ep_clear_halt() already refuses control endpoints for the same
	 * reason; this path was added without the matching guard.
	 *
	 * The buffers are still safe. The reason for ending a transfer here is that
	 * udc_ep_cancel_queued() hands memory back while a TRB may still point into
	 * it - but the control endpoints do not rely on this path for that. Their
	 * descriptors are owned by the control state machine, which clears them in
	 * udc_dwc3_ctrl_abandon(), udc_dwc3_recover() and
	 * udc_dwc3_drop_xfer_state() - the last of which runs on soft reset, USB
	 * reset, disconnect and controller disable.
	 */
	if (USB_EP_GET_IDX(ep_cfg->addr) != 0U && udc_ep_is_busy(ep_cfg) &&
	    udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
		LOG_INF("EP%02x dequeued with a transfer still active; recovering "
			"through End Transfer before the buffers are released",
			ep_cfg->addr);
	}

	udc_ep_cancel_queued(dev, ep_cfg);
	udc_ep_set_busy(ep_cfg, false);

	return 0;
}

static int udc_dwc3_ep_resume(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data,
			      const bool modify)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	struct net_buf *buf;
	int ret;

	/*
	 * Databook 3.2.2.7: a Start Transfer must not be issued on an endpoint
	 * whose End Transfer has not reported Endpoint Command Complete. CmdAct
	 * clearing means only that the command was accepted; the controller may
	 * still be concluding system bus traffic for the transfer it ended.
	 *
	 * Every caller of this function reaches it straight after an End Transfer -
	 * udc_dwc3_ep_disable() issues one directly, and udc_dwc3_ep_enable() goes
	 * through udc_dwc3_on_set_config_or_interface(), which ends every busy
	 * non-control endpoint - so the window is not theoretical.
	 *
	 * Waiting here is not an option: the completion is delivered by
	 * udc_dwc3_handle_event() on the same work queue that runs this code, and
	 * it needs the UDC mutex this path already holds. So the resume is
	 * postponed instead and udc_dwc3_on_ep_cmd_cmplt() performs it, exactly as
	 * udc_dwc3_ctrl_rearm() does for the control endpoints.
	 *
	 * Only non-control endpoints: this function issues no Start Transfer for
	 * endpoint 0, and control recovery already has its own deferred re-arm.
	 *
	 * Postponing cannot strand the endpoint. end_xfer_pending is set only when
	 * CmdIOC was requested - which requires DCTL.RunStop - and is cleared again
	 * if the command could not be issued, so it is set only when a completion
	 * event is genuinely on its way.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 && ep_data->end_xfer_pending) {
		LOG_DBG("End Transfer still concluding on EP%02x, deferring resume",
			ep_data->cfg.addr);
		ep_data->resume_modify = modify;
		ep_data->resume_pending = true;
		return 0;
	}

	/* Reset all ongoing transfers on non-control OUT endpoints */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		udc_dwc3_depcmd_clear_stall(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	}

	udc_dwc3_depcmd_ep_config(dev, ep_data, modify);
	udc_dwc3_depcmd_ep_xfer_config(dev, ep_data);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		ret = udc_dwc3_trb_nonctrl_init(dev, ep_data);
		if (ret != 0) {
			return ret;
		}
	}

	/* Starting from here, the endpoint can be used */
	sys_set_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	/* Re-enqueue all the previously dequeued buffers */
	while (true) {
		buf = k_fifo_get(&ep_data->requeue_fifo, K_NO_WAIT);
		if (buf == NULL) {
			break;
		}

		LOG_DBG("Requeueing buffer %p %d:%d:%d",
			buf,
			udc_get_buf_info(buf)->setup,
			udc_get_buf_info(buf)->data,
			udc_get_buf_info(buf)->status);

		ret = udc_dwc3_trb_bulk(dev, ep_data, buf);
		if (ret != 0) {
			return ret;
		}
	}

	/* We might have blocked transfers earlier */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
	}

	return 0;
}

static int udc_dwc3_ep_enable(const struct device *const dev, struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = (struct udc_dwc3_ep_data *)ep_cfg;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	LOG_DBG("EP%02x, first EP%02x", ep_data->cfg.addr, priv->first_ep);

	if (USB_EP_GET_IDX(ep_cfg->addr) > 0) {
		if (priv->first_ep == 0) {
			priv->first_ep = ep_cfg->addr;
		}
		if (ep_cfg->addr == priv->first_ep) {
			udc_dwc3_on_set_config_or_interface(dev);
		}
	}

	return udc_dwc3_ep_resume(dev, ep_data, ep_data->cfg.stat.enabled);
}

static int udc_dwc3_ep_disable(const struct device *const dev, struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const int slots = CONFIG_UDC_DWC3_TRB_NUM - 1;
	struct net_buf *buf;

	LOG_DBG("Disabling EP%02x", ep_cfg->addr);

	/*
	 * Drop any reference the control machinery holds to this endpoint before
	 * tearing it down. The watchdog and the pending-recovery record both keep a
	 * pointer here, and udc_dwc3_shutdown() disables the control endpoints, so
	 * a watchdog firing afterwards would End an endpoint that no longer exists
	 * as far as the controller is concerned.
	 *
	 * Cancel, don't just clear. A deadline left armed with its owner cleared
	 * still fires: the SETUP branch in udc_dwc3_watchdog_worker() keys off
	 * watchdog_type alone and reads ep0_out directly, and udc_dwc3_recover()
	 * falls back to whichever control endpoint is still busy when the pointer
	 * is NULL. Clearing watchdog_ep without cancelling would therefore leave
	 * the very hazard this guard exists to prevent. Clearing watchdog_type too
	 * matches udc_dwc3_drop_xfer_state() and udc_dwc3_on_ctrl().
	 */
	if (priv->watchdog_ep == ep_data) {
		k_work_cancel_delayable(&priv->watchdog_dwork);
		priv->watchdog_ep = NULL;
		priv->watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
	}
	if (priv->ctrl_recovery_ep == ep_data) {
		priv->ctrl_recovery_ep = NULL;
		priv->ctrl_recovery_pending = false;
	}

	/*
	 * Drop any postponed resume as well. The End Transfer below sets
	 * end_xfer_pending again, and its completion would otherwise run a resume
	 * left over from before this teardown - re-arming an endpoint that has just
	 * been disabled. Callers that disable and then resume set the flag again on
	 * their own way through udc_dwc3_ep_resume().
	 */
	ep_data->resume_pending = false;

	/* Disable the endpoint */
	sys_clear_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	/*
	 * Reset ongoing transfers.
	 *
	 * This does issue End Transfer on a control endpoint when reached via
	 * udc_dwc3_shutdown(), which looks like it contradicts the "End Transfer on
	 * a live control endpoint hangs" rule the recovery paths are built around.
	 * It is not a contradiction, and it was reviewed and left alone deliberately:
	 *
	 * udc_dwc3_on_set_config_or_interface() issues exactly this command on
	 * EP0-IN on every SetConfiguration - it is what forces the TX FIFO
	 * reconfiguration, and the DEPCFG immediately after it depends on it.  That
	 * path runs on every enumeration and has never hung.  The hang was observed
	 * against an ARMED control endpoint during recovery, not on a controlled
	 * teardown, so the rule is about when the endpoint is live, not about the
	 * command being unusable on EP0.
	 */
	udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);

	udc_ep_set_busy(ep_cfg, false);

	/*
	 * Oldest first.  The ring holds the oldest queued buffer at tail, and
	 * requeue_fifo is a FIFO, so walking tail forward preserves submission
	 * order.  The previous walk went backwards from head - newest first - which
	 * combined with FIFO semantics handed the buffers back REVERSED whenever two
	 * or more were outstanding.  (Backwards would have been right for a LIFO,
	 * which is what the old comment assumed.)
	 */
	for (int n = 0; n < slots; n++) {
		const int idx = (ep_data->tail + n) % slots;

		buf = ep_data->net_buf[idx];
		if (buf != NULL) {
			LOG_DBG("Popping buffer %p %d:%d:%d",
				buf,
				udc_get_buf_info(buf)->setup,
				udc_get_buf_info(buf)->data,
				udc_get_buf_info(buf)->status);

			k_fifo_put(&ep_data->requeue_fifo, buf);
		}
	}

	/* Reset the buffers */
	memset(ep_data->trb_buf, 0, sizeof(*ep_data->trb_buf) * (CONFIG_UDC_DWC3_TRB_NUM - 1));
	memset(ep_data->net_buf, 0, sizeof(*ep_data->net_buf) * (CONFIG_UDC_DWC3_TRB_NUM - 1));
	ep_data->head = ep_data->tail = 0;
	ep_data->full = false;

	return 0;
}

static int udc_dwc3_ep_set_halt(const struct device *const dev,
				struct udc_ep_config *const ep_cfg)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	switch (ep_data->cfg.addr) {
	case USB_CONTROL_EP_IN:
		/* The datasheet says to only set stall the OUT direction */
		ep_data = &cfg->ep_data_out[0];
		__fallthrough;
	case USB_CONTROL_EP_OUT:
		udc_dwc3_depcmd_set_stall(dev, ep_data);
		break;
	default:
		udc_dwc3_depcmd_set_stall(dev, ep_data);
		ep_data->cfg.stat.halted = true;
	}

	/* So that the next type is SETUP */
	priv->last_xfer_type = UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2;

	return 0;
}

static int udc_dwc3_ep_clear_halt(const struct device *const dev,
				  struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	LOG_INF("Clearing stall for EP%02x", ep_cfg->addr);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		return 0;
	}

	/*
	 * SPEC, Programming Guide 3.30b section 4.2.7 "Handling ENDPOINT_HALT":
	 *
	 *   "On ClearFeature (ENDPOINT_HALT), software must first remove all pending
	 *    transfers for the endpoint through the End Transfer command. It may then
	 *    issue a Clear Stall command on the endpoint followed by Start Transfer to
	 *    start transfers again."
	 *
	 * The ordering is the requirement. Clearing the stall first leaves whatever
	 * was armed when the endpoint halted still active in the controller, and the
	 * work item below then starts a transfer on top of it. This is the host's
	 * own recovery path - it halts an endpoint it thinks is stuck and clears it
	 * again - so getting it wrong turns a recoverable stall into a wedged
	 * endpoint precisely when the host is trying to put things right.
	 *
	 * "All pending transfers" is the wording, and the busy claim is what says
	 * one is pending: udc_dwc3_ep_set_halt() does not touch it, so a transfer
	 * armed when the endpoint halted still holds it here.
	 */
	if (udc_ep_is_busy(ep_cfg) &&
	    udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
		LOG_INF("EP%02x halted with a transfer still pending; recovering "
			"through End Transfer before Clear Stall", ep_cfg->addr);
	}

	udc_dwc3_depcmd_clear_stall(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	ep_data->cfg.stat.halted = false;

	/* Resume halted previously transfers */
	k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);

	return 0;
}

static int udc_dwc3_set_address_no_op(const struct device *const dev, const uint8_t addr)
{
	return 0;
}

static int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	LOG_INF("Setting address to %u", addr);

	/* Configure the new address */
	reg = sys_read32(base + UDC_DWC3_DCFG);
	reg &= ~UDC_DWC3_DCFG_DEVADDR_MASK;
	reg |= FIELD_PREP(UDC_DWC3_DCFG_DEVADDR_MASK, addr);
	sys_write32(reg, base + UDC_DWC3_DCFG);

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
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	int ret;

	LOG_INF("Enabling DWC3 driver");

	ret = udc_dwc3_quirk_enable(dev);
	if (ret != 0) {
		return ret;
	}

	/* First packet to be expected */
	//atomic_set_bit(&priv->expected_xfer, UDC_DWC3_CTRL_SETUP);

	/* Enable the DWC3 events */
	sys_set_bits(base + UDC_DWC3_DCTL, UDC_DWC3_DCTL_RUNSTOP);

	/* Enable the IRQ (for now, just schedule a first work queue job) */
	cfg->irq_enable_func();

	/*
	 * Stamp the control clocks before the housekeeper can ever read them.
	 *
	 * Both are ages measured as now - stamp, and both start at zero, so a
	 * decline arriving before the first GRANT would be measured against
	 * cycle 0. By enable time the system clock is already seconds old, so
	 * that age blows past RECOVERY_TIMEOUT immediately and the very first
	 * recovery fires on a healthy endpoint - the spurious-teardown class
	 * that made the old SETUP timeout unusable. Two stores rule it out.
	 */
	priv->ctrl_arm_t0 = k_cycle_get_32();
	priv->ctrl_quiet_t0 = priv->ctrl_arm_t0;

	k_timer_start(&priv->heartbeat_timer, K_MSEC(UDC_DWC3_HEARTBEAT_MS),
		      K_MSEC(UDC_DWC3_HEARTBEAT_MS));

	return 0;
}

static int udc_dwc3_disable(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const struct udc_dwc3_config *const cfg = dev->config;

	LOG_DBG("Disabling DWC3 driver");

	k_timer_stop(&priv->heartbeat_timer);

	sys_clear_bits(base + UDC_DWC3_DCTL, UDC_DWC3_DCTL_RUNSTOP);

	/*
	 * With RunStop cleared the controller raises no further Endpoint Command
	 * Complete events, so anything outstanding is stranded. Dropping it here
	 * means a disable/enable cycle starts from a clean state rather than from
	 * flags describing transfers that no longer exist.
	 */
	udc_dwc3_drop_xfer_state(dev, "controller disable");

	cfg->irq_disable_func();

	return 0;
}

static int udc_dwc3_init(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;
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

	//reg = sys_read32(base + UDC_DWC3_GCTL);
	//reg &= ~UDC_DWC3_GCTL_RAMCLKSEL_MASK;
	//reg |= UDC_DWC3_GCTL_RAMCLKSEL_BUS_CLK;
	//reg |= UDC_DWC3_GCTL_RAMCLKSEL_PIPE_CLK;
	//reg |= UDC_DWC3_GCTL_RAMCLKSEL_PIPE_DIV2_CLK;
	//reg |= UDC_DWC3_GCTL_RAMCLKSEL_MAC2_CLK;
	//sys_write32(reg, base + UDC_DWC3_GCTL);

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

#if CONFIG_UDC_DWC3_SHELL
	/* Initialize default queue sizes */
	udc_dwc3_init_fifo_space(dev);
#endif

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS0);
	LOG_DBG("GHWPARAMS0 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS1);
	LOG_DBG("GHWPARAMS1 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS2);
	LOG_DBG("GHWPARAMS2 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS3);
	LOG_DBG("GHWPARAMS3 = 0x%08x", reg);

	LOG_DBG("- GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES_MASK, reg));
	LOG_DBG("- GHWPARAMS3_NUM_IN_EPS %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_IN_EPS_MASK, reg));
	LOG_DBG("- GHWPARAMS3_NUM_EPS %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_EPS_MASK, reg));

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS4);
	LOG_DBG("GHWPARAMS4 = 0x%08x", reg);

	LOG_DBG("- BMU_LSP_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS4_BMU_LSP_DEPTH_MASK, reg));
	LOG_DBG("- BMU_PTL_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS4_BMU_PTL_DEPTH_M1_MASK, reg) + 1);
	LOG_DBG("- CACHE_TRBS_PER_TRANSFER: %lu",
		FIELD_GET(UDC_DWC3_GHWPARAMS4_CACHE_TRBS_PER_TRANSFER_MASK, reg));

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS5);
	LOG_DBG("GHWPARAMS5 = 0x%08x", reg);

	LOG_DBG("- GHWPARAMS5_DFQ_FIFO_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_DFQ_FIFO_DEPTH_MASK, reg));
	LOG_DBG("- GHWPARAMS5_DWQ_FIFO_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_DWQ_FIFO_DEPTH_MASK, reg));
	LOG_DBG("- GHWPARAMS5_TXQ_FIFO_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_TXQ_FIFO_DEPTH_MASK, reg));
	LOG_DBG("- GHWPARAMS5_RXQ_FIFO_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_RXQ_FIFO_DEPTH_MASK, reg));
	LOG_DBG("- GHWPARAMS5_BMU_BUSGM_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_BMU_BUSGM_DEPTH_MASK, reg));

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS6);
	LOG_DBG("GHWPARAMS6 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS7);
	LOG_DBG("GHWPARAMS7 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS8);
	LOG_DBG("GHWPARAMS8 = 0x%08x", reg);

	LOG_DBG("Event buffer is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));

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
	.set_address = udc_dwc3_set_address_no_op,
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

	LOG_DBG("checking for pending transfers for EP%02x", ep_data->cfg.addr);

	/*
	 * This worker is the one producer of TRBs that did not hold the UDC mutex.
	 *
	 * Being on udc_get_work_q() serialises it against udc_dwc3_event_worker(),
	 * and so against pop_trb, because a work queue runs its items one at a time.
	 * It does NOT serialise it against anything running on another thread, and
	 * the other threads all hold the UDC mutex while touching this same ring:
	 * udc_dwc3_ep_enable() through udc_dwc3_ep_resume(), whose requeue loop
	 * pushes inline on the caller's thread, udc_dwc3_ep_disable(), which walks
	 * net_buf[] in reverse to drain it, and udc_dwc3_ep_dequeue(). Without the
	 * mutex here, holding it there excluded nothing.
	 *
	 * Taking it makes every toucher of head/tail/full/net_buf[] hold the same
	 * lock. Safe to block on: the submitters all use k_work_submit_to_queue(),
	 * which does not wait, and nothing cancels or flushes this work
	 * synchronously.
	 */
	udc_lock_internal(dev, K_FOREVER);

	if (ep_data->cfg.stat.halted) {
		LOG_DBG("endpoint is halted, not processing buffers");
		goto unlock;
	}

	/*
	 * The endpoint is between an End Transfer and the Start Transfer that will
	 * replace it. Pushing a TRB now would issue an Update Transfer against a
	 * transfer resource the controller is still concluding - the same 3.2.2.7
	 * violation udc_dwc3_ep_resume() defers to avoid, reached through Update
	 * rather than Start - and the deferred resume would then memset the whole
	 * ring in udc_dwc3_trb_nonctrl_init(), discarding the TRB while head, tail
	 * and net_buf[] stayed advanced.
	 *
	 * The window is reachable because a deferred resume returns 0, so
	 * udc_ep_enable_internal() marks the endpoint enabled and the stack may
	 * enqueue immediately; udc_dwc3_ep_enqueue() gates only on RunStop and
	 * enabled, and this worker runs on the same queue as the event that would
	 * clear the flag, so it can win the race.
	 *
	 * Nothing is lost by stopping here: the buffers stay queued.
	 * udc_dwc3_on_ep_cmd_cmplt() resubmits this work once the End Transfer
	 * completes, either through the tail of the deferred resume or directly.
	 */
	if (ep_data->end_xfer_pending || ep_data->resume_pending) {
		LOG_DBG("EP%02x still concluding an End Transfer, deferring %s",
			ep_data->cfg.addr,
			ep_data->resume_pending ? "until the resume runs" : "buffers");
		goto unlock;
	}

	while ((buf = udc_buf_peek(&ep_data->cfg)) != NULL) {
		LOG_DBG("Processing buffer %p from queue", (void *)buf);

		ret = udc_dwc3_trb_bulk(dev, ep_data, buf);
		if (ret != 0) {
			LOG_DBG("abort: No more room for buffer");
			break;
		}

		LOG_DBG("success: Buffer enqueued");

		udc_buf_get(&ep_data->cfg);
	}

unlock:
	udc_unlock_internal(dev);
}

/*
 * Initialize the controller and endpoints capabilities,
 * register endpoint structures, no hardware I/O yet.
 */
static int udc_dwc3_driver_preinit(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_data *const data = dev->data;
	struct udc_dwc3_ep_data *ep_data;
	uint16_t mps = 0;
	int ret;

	ret = udc_dwc3_quirk_preinit(dev);
	if (ret != 0) {
		return ret;
	}

	DEVICE_MMIO_NAMED_MAP(dev, base, K_MEM_CACHE_NONE);

	k_mutex_init(&data->mutex);
	k_work_init(&priv->event_work, udc_dwc3_event_worker);
	k_work_init_delayable(&priv->watchdog_dwork, udc_dwc3_watchdog_worker);
	k_work_init(&priv->heartbeat_work, udc_dwc3_heartbeat_worker);
	k_timer_init(&priv->heartbeat_timer, udc_dwc3_heartbeat_expiry, NULL);

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

	/*
	 * EP0 needs this too.  The pre-init loops below start at i = 1, so the
	 * control endpoint never got a k_fifo_init() - yet udc_dwc3_ep_resume()
	 * guards only its first three steps with USB_EP_GET_IDX() > 0 and then falls
	 * through to k_fifo_get() on this queue for every endpoint.  It happened to
	 * be harmless because the static zero-init matches what k_fifo_init()
	 * produces, but relying on that is not something to leave in place.
	 */
	k_fifo_init(&ep_data->requeue_fifo);

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
		k_fifo_init(&ep_data->requeue_fifo);

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
		/* The IN loop above does this; the OUT loop did not, leaving every
		 * non-control OUT endpoint with an uninitialised requeue_fifo that
		 * udc_dwc3_ep_resume() reads on every resume. */
		k_fifo_init(&ep_data->requeue_fifo);

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

/*
 * The event buffer must be aligned to its own SIZE, not to a fixed 16
 * bytes: "This address must be aligned to the Event Buffer size", and the
 * GEVNTADR description repeats it - "the lower n bits of the address must
 * be GEVNTSIZn.EVNTSiz-aligned".
 *
 * With a fixed 16-byte alignment the requirement is met only by luck, and
 * the consequence when it is not is that the controller's wrap and
 * software's wrap disagree near the end of the ring: the controller writes
 * outside the declared buffer, GEVNTCOUNT still counts the event, and the
 * slot software reads stays empty. That is exactly the failure the
 * event-slot sentinel reports.
 */
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
		__aligned(64);							\
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

/*
 * Shell
 *
 * Commands to debug DWC3 hardware, DWC3 driver, USB hosts, device-side applications,
 * and cable problems.
 */

#ifdef CONFIG_UDC_DWC3_SHELL

const struct udc_dwc3_reg {
	uint32_t addr;
	char *name;
} udc_dwc3_regs[] = {
	/* main registers */
	{.addr = UDC_DWC3_GCTL, .name = "GCTL"},
	{.addr = UDC_DWC3_DCTL, .name = "DCTL"},
	{.addr = UDC_DWC3_DCFG, .name = "DCFG"},
	{.addr = UDC_DWC3_DEVTEN, .name = "DEVTEN"},
	{.addr = UDC_DWC3_DALEPENA, .name = "DALEPENA"},
	{.addr = UDC_DWC3_GCOREID, .name = "GCOREID"},
	{.addr = UDC_DWC3_GSTS, .name = "GSTS"},
	{.addr = UDC_DWC3_DSTS, .name = "DSTS"},
	{.addr = UDC_DWC3_GEVNTADR_LO(0), .name = "GEVNTADR_LO(0)"},
	{.addr = UDC_DWC3_GEVNTADR_HI(0), .name = "GEVNTADR_HI(0)"},
	{.addr = UDC_DWC3_GEVNTSIZ(0), .name = "GEVNTSIZ(0)"},
	{.addr = UDC_DWC3_GEVNTCOUNT(0), .name = "GEVNTCOUNT(0)"},
	{.addr = UDC_DWC3_GUSB2PHYCFG, .name = "GUSB2PHYCFG"},
	{.addr = UDC_DWC3_GUSB3PIPECTL, .name = "GUSB3PIPECTL"},
	/* debug */
	{.addr = UDC_DWC3_GBUSERRADDR_LO, .name = "GBUSERRADDR_LO"},
	{.addr = UDC_DWC3_GBUSERRADDR_HI, .name = "GBUSERRADDR_HI"},
	{.addr = UDC_DWC3_CTLDEBUG_LO, .name = "CTLDEBUG_LO"},
	{.addr = UDC_DWC3_CTLDEBUG_HI, .name = "CTLDEBUG_HI"},
	{.addr = UDC_DWC3_ANALYZERTRACE, .name = "ANALYZERTRACE"},
	{.addr = UDC_DWC3_GDBGFIFOSPACE, .name = "GDBGFIFOSPACE"},
	{.addr = UDC_DWC3_GDBGLTSSM, .name = "GDBGLTSSM"},
	{.addr = UDC_DWC3_GDBGLNMCC, .name = "GDBGLNMCC"},
	{.addr = UDC_DWC3_GDBGBMU, .name = "GDBGBMU"},
	{.addr = UDC_DWC3_GDBGLSPMUX_DEV, .name = "GDBGLSPMUX_DEV"},
	{.addr = UDC_DWC3_GDBGLSPMUX_HST, .name = "GDBGLSPMUX_HST"},
	{.addr = UDC_DWC3_GDBGLSP, .name = "GDBGLSP"},
	{.addr = UDC_DWC3_GDBGEPINFO0, .name = "GDBGEPINFO0"},
	{.addr = UDC_DWC3_GDBGEPINFO1, .name = "GDBGEPINFO1"},
	{.addr = UDC_DWC3_BU3RHBDBG0, .name = "BU3RHBDBG0"},
	/* physical endpoint numbers */
	{.addr = UDC_DWC3_DEPCMDPAR2(0), .name = "DEPCMDPAR2(0)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(0), .name = "DEPCMDPAR1(0)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(0), .name = "DEPCMDPAR0(0)"},
	{.addr = UDC_DWC3_DEPCMD(0), .name = "DEPCMD(0)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(1), .name = "DEPCMDPAR2(1)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(1), .name = "DEPCMDPAR1(1)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(1), .name = "DEPCMDPAR0(1)"},
	{.addr = UDC_DWC3_DEPCMD(1), .name = "DEPCMD(1)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(2), .name = "DEPCMDPAR2(2)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(2), .name = "DEPCMDPAR1(2)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(2), .name = "DEPCMDPAR0(2)"},
	{.addr = UDC_DWC3_DEPCMD(2), .name = "DEPCMD(2)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(3), .name = "DEPCMDPAR2(3)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(3), .name = "DEPCMDPAR1(3)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(3), .name = "DEPCMDPAR0(3)"},
	{.addr = UDC_DWC3_DEPCMD(3), .name = "DEPCMD(3)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(4), .name = "DEPCMDPAR2(4)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(4), .name = "DEPCMDPAR1(4)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(4), .name = "DEPCMDPAR0(4)"},
	{.addr = UDC_DWC3_DEPCMD(4), .name = "DEPCMD(4)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(5), .name = "DEPCMDPAR2(5)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(5), .name = "DEPCMDPAR1(5)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(5), .name = "DEPCMDPAR0(5)"},
	{.addr = UDC_DWC3_DEPCMD(5), .name = "DEPCMD(5)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(6), .name = "DEPCMDPAR2(6)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(6), .name = "DEPCMDPAR1(6)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(6), .name = "DEPCMDPAR0(6)"},
	{.addr = UDC_DWC3_DEPCMD(6), .name = "DEPCMD(6)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(7), .name = "DEPCMDPAR2(7)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(7), .name = "DEPCMDPAR1(7)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(7), .name = "DEPCMDPAR0(7)"},
	{.addr = UDC_DWC3_DEPCMD(7), .name = "DEPCMD(7)"},
	/* Hardware parameters */
	{.addr = UDC_DWC3_GHWPARAMS0, .name = "GHWPARAMS0"},
	{.addr = UDC_DWC3_GHWPARAMS1, .name = "GHWPARAMS1"},
	{.addr = UDC_DWC3_GHWPARAMS2, .name = "GHWPARAMS2"},
	{.addr = UDC_DWC3_GHWPARAMS3, .name = "GHWPARAMS3"},
	{.addr = UDC_DWC3_GHWPARAMS4, .name = "GHWPARAMS4"},
	{.addr = UDC_DWC3_GHWPARAMS5, .name = "GHWPARAMS5"},
	{.addr = UDC_DWC3_GHWPARAMS6, .name = "GHWPARAMS6"},
	{.addr = UDC_DWC3_GHWPARAMS7, .name = "GHWPARAMS7"},
	{.addr = UDC_DWC3_GHWPARAMS8, .name = "GHWPARAMS8"},
};

static const struct {
	char *name;
	uint32_t type;
} udc_dwc3_fifo_regs[_NUM_FIFO_REGS] = {
	{
		.name = "TxQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXQ
	}, {
		.name = "RxQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXQ
	}, {
		.name = "TxReqQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXREQQ
	}, {
		.name = "RxReqQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQQ
	}, {
		.name = "RxInfoQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXINFOQ
	}, {
		.name = "DescFetchQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCHQ
	}, {
		.name = "WriteBack/EventQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_WREVENTQ
	},
	{
		/*
		 * Defined but never dumped until now. WriteBack/EventQ has read
		 * back as 0/0 in every capture so far, the same as DescFetchQ,
		 * which certainly exists - so those reads look like the debug
		 * register not reporting these queue types in this build rather
		 * than the queues being absent. AuxEventQ is worth a look on the
		 * chance that it reports where the other does not.
		 */
		.name = "AuxEventQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENTQ
	},
};

static uint32_t udc_dwc3_read_fifo_space(const struct device *dev, uint32_t type, uint32_t num)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t mdwidth = (sys_read32(base + UDC_DWC3_GHWPARAMS0) >> 8) & 0xFF;
	uint32_t reg;

	reg = type;
	reg |= FIELD_PREP(UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK, num);
	sys_write32(reg, base + UDC_DWC3_GDBGFIFOSPACE);

	reg = sys_read32(base + UDC_DWC3_GDBGFIFOSPACE);
	reg = FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK, reg);
	return reg * mdwidth / BITS_PER_BYTE;
}

static void udc_dwc3_init_fifo_space(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);

	for (int n = 0; n < _NUM_FIFO_SPACE; n++) {
		for (int i = 0; i < _NUM_FIFO_REGS; i++) {
			priv->max_bytes_avail[n][i] = udc_dwc3_read_fifo_space(
				dev, udc_dwc3_fifo_regs[i].type, n);
		}
	}
}

static void udc_dwc3_dump_registers(const struct device *dev, const struct shell *sh)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	for (size_t i = 0; i < ARRAY_SIZE(udc_dwc3_regs); i++) {
		const struct udc_dwc3_reg *ureg = &udc_dwc3_regs[i];

		reg = sys_read32(base + ureg->addr);
		shell_print(sh, "reg 0x%08x == 0x%08x %s", ureg->addr, reg, ureg->name);
	}
}

static void udc_dwc3_dump_bus_error(const struct device *dev, const struct shell *sh)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	if (sys_read32(base + UDC_DWC3_GSTS) & UDC_DWC3_GSTS_BUSERRADDRVLD) {
		shell_print(sh, "BUS_ERROR addr=0x%08x%08x",
			     sys_read32(base + UDC_DWC3_GBUSERRADDR_HI),
			     sys_read32(base + UDC_DWC3_GBUSERRADDR_LO));
	} else {
		shell_print(sh, "no bus error");
	}
}

static void udc_dwc3_dump_link_state(const struct device *dev, const struct shell *sh)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	reg = sys_read32(base + UDC_DWC3_DSTS);
	switch (reg & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
		shell_print(sh, "DWC3_DSTS_CONNECTSPD_HS");
		goto usb2;
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
		shell_print(sh, "DWC3_DSTS_CONNECTSPD_FS");
		goto usb2;
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		shell_print(sh, "DWC3_DSTS_CONNECTSPD_SS");
		goto usb3;
	default:
		shell_print(sh, "unknown speed");
	}
	return;
usb2:
	switch (reg & UDC_DWC3_DSTS_USBLNKST_MASK) {
	case UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_ON_STATE");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_DISCONNECTED");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_RESET:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_RESET");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_RESUME:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_RESUME");
		break;
	}
	return;
usb3:
	switch (reg & UDC_DWC3_DSTS_USBLNKST_MASK) {
	case UDC_DWC3_DSTS_USBLNKST_USB3_U0:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_U0");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_U1:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_U1");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_U2:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_U2");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_U3:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_U3");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_SS_DIS");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_RX_DET");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_SS_INACT");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_POLL:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_POLL");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_RECOV:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_RECOV");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_HRESET:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_HRESET");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_CMPLY:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_CMPLY");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_LPBK:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_LPBK");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_RESET_RESUME");
		break;
	}
}

static void udc_dwc3_dump_events(const struct device *dev, const struct shell *sh)
{
	const struct udc_dwc3_config *cfg = dev->config;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_EVENTS_NUM; i++) {
		uint32_t evt = cfg->evt_buf[i];
		char *s = (i == priv->evt_next) ? "<-" : "  ";

		shell_print(sh, "evt 0x%02x: 0x%08x %s %s",
			i, evt, s, udc_dwc3_get_event_name(evt, 0));
	}

	/*
	 * How often the controller's posted write had not landed when the event
	 * was read. "late" is normal and cheap - the wait absorbs it. "stalled"
	 * means the wait ran out, which should be rare; if it is not, the ring is
	 * losing ground and the timeout above is the thing to raise.
	 */
	shell_print(sh, "events %u, posted-write waits: late %u, gave up %u",
		    priv->evt_handled, priv->evt_late, priv->evt_gaveup);
	shell_print(sh, "worst late wait: %u polls (lower bound), %u us (upper bound)",
		    priv->evt_late_polls_max, priv->evt_late_us_max);
	shell_print(sh, "drain re-armed after unmask: %u", priv->evt_rearm);
	shell_print(sh, "link state changes %u, last state 0x%x, repeated x%u",
		    priv->evt_link_total, priv->evt_link_last, priv->evt_link_run);
	shell_print(sh, "GEVNTCOUNT high-water %u bytes of %u, give-up run %u on slot %u",
		    priv->evt_gevntcount_hwm,
		    (unsigned int)(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t)),
		    priv->evt_gaveup_run, priv->evt_gaveup_slot);
	shell_print(sh, "control aborts: setup-pending %u, other TRBSTS %u, "
		    "stage desync %u", priv->ctrl_setup_pending,
		    priv->ctrl_trbsts_other, priv->ctrl_desync);
	shell_print(sh, "control arms deferred behind an End Transfer: %u",
		    priv->ctrl_deferred_arm);
}

static void udc_dwc3_dump_trb(const struct device *dev, struct udc_dwc3_ep_data *ep_data,
			      const struct shell *sh)
{
	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		struct udc_dwc3_trb trb = ep_data->trb_buf[i];
		bool hwo = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_HWO);
		bool lst = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_LST);
		bool chn = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_CHN);
		bool csp = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_CSP);
		bool isp = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_ISP_IMI);
		bool ioc = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_IOC);
		bool spr = !!(trb.status & UDC_DWC3_TRB_STATUS_SPR);
		uint32_t trbctl = FIELD_GET(UDC_DWC3_TRB_CTRL_TRBCTL_MASK, trb.ctrl);
		uint32_t trbsts = FIELD_GET(UDC_DWC3_TRB_STATUS_TRBSTS_MASK, trb.status);
		uint32_t pcm1 = FIELD_GET(UDC_DWC3_TRB_STATUS_PCM1_MASK, trb.status);
		uint32_t sidsofn = FIELD_GET(UDC_DWC3_TRB_CTRL_SIDSOFN_MASK, trb.ctrl);
		uint32_t bufsiz = FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb.status);
		char *head = (i == ep_data->head) ? " <HEAD" : "";
		char *tail = (i == ep_data->tail) ? " <TAIL" : "";
		char *full = (i == ep_data->head && ep_data->full) ? " <FULL" : "";

#define FMT	"%p EP%02x addr=0x%08x%08x ctl=%u sts=%u hwo=%u lst=%u chn=%u" \
		" csp=%u isp=%u ioc=%u spr=%u pcm1=%u sof=%u bufsiz=%u%s%s%s"

#define ARGS	&ep_data->trb_buf[i], ep_data->cfg.addr, trb.addr_hi, trb.addr_lo, trbctl, trbsts, \
		hwo, lst, chn, csp, isp, ioc, spr, pcm1, sidsofn, bufsiz, head, tail, full

		if (sh == NULL) {
			LOG_WRN(FMT, ARGS);
		} else {
			shell_print(sh, FMT, ARGS);
		}
	}
}

static void udc_dwc3_dump_each(const struct device *dev,
			     void (*fn)(const struct device *, struct udc_dwc3_ep_data *,
					const struct shell *),
			     char *label, const struct shell *sh)
{
	const struct udc_dwc3_config *cfg = dev->config;

	for (int i = 0; i < cfg->num_in_eps; i++) {
		struct udc_dwc3_ep_data *ep_data = &cfg->ep_data_in[i];
		uint8_t addr = ep_data->cfg.addr;

		if (ep_data->trb_buf == NULL) {
			continue;
		}

		/* xferrscidx is included so a stuck endpoint can be compared against the
		 * value reported by its last "DepStartXfer done ... xferrscidx=" line.
		 * A mismatch means the index was overwritten by a command that did not
		 * succeed, which every later Update and End Transfer would then be using.
		 * Shell output only - no effect on the data path.
		 */
		shell_print(sh, "%s for IN endpoint 0x%02x (%u %s) xferrscidx=0x%x",
			  label, addr, addr & 0x7f, (addr & 0x80) ? "IN" : "OUT",
			  ep_data->xferrscidx);
		(*fn)(dev, ep_data, sh);
	}

	for (int i = 0; i < cfg->num_out_eps; i++) {
		struct udc_dwc3_ep_data *ep_data = &cfg->ep_data_out[i];
		uint8_t addr = ep_data->cfg.addr;

		if (ep_data->trb_buf == NULL) {
			continue;
		}

		shell_print(sh, "%s for OUT endpoint 0x%02x (%u %s) xferrscidx=0x%x",
			  label, addr, addr & 0x7f, (addr & 0x80) ? "IN" : "OUT",
			  ep_data->xferrscidx);
		(*fn)(dev, ep_data, sh);
	}
}

static void udc_dwc3_dump_each_trb(const struct device *dev, const struct shell *sh)
{
	udc_dwc3_dump_each(dev, udc_dwc3_dump_trb, "trb", sh);
}

static void udc_dwc3_dump_fifo_space(const struct device *dev, const struct shell *sh)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t num_in_eps;
	uint32_t num_eps;
	uint32_t total_xfer_resources;
	uint32_t avail;
	uint32_t reg;

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS3);
	num_in_eps = FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_IN_EPS_MASK, reg);
	num_eps = FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_EPS_MASK, reg);
	total_xfer_resources = FIELD_GET(UDC_DWC3_GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES_MASK, reg);

	shell_print(sh, "num_in_eps: %u", num_in_eps);
	shell_print(sh, "num_eps: %u", num_eps);
	shell_print(sh, "total_xfer_resources: %u", total_xfer_resources);

	for (size_t n = 0; n < _NUM_FIFO_SPACE; n++) {
		shell_print(sh, "");
		shell_print(sh, "FIFO %zu", n);

		for (size_t i = 0; i < ARRAY_SIZE(udc_dwc3_fifo_regs); i++) {
			avail = udc_dwc3_read_fifo_space(dev, udc_dwc3_fifo_regs[i].type, n);
			shell_print(sh, "- %-15s = %u / %u bytes available",
				udc_dwc3_fifo_regs[i].name, avail,
				(uint32_t)priv->max_bytes_avail[n][i]);
		}
	}

	shell_print(sh, "");
	shell_print(sh, "Common");

	avail = udc_dwc3_read_fifo_space(
		dev, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOLSTATUSQ, 0);
	shell_print(sh, "- %-15s = %u bytes available", "PROTOCOLSTATUS", avail);
}

static void udc_dwc3_dump_all(const struct device *dev, const struct shell *sh)
{
	shell_print(sh, "");
	shell_print(sh, "Registers:");
	udc_dwc3_dump_registers(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "Bus Errors:");
	udc_dwc3_dump_bus_error(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "Link State:");
	udc_dwc3_dump_link_state(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "Events:");
	udc_dwc3_dump_events(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "FIFO Space:");
	udc_dwc3_dump_fifo_space(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "TRBs:");
	udc_dwc3_dump_each_trb(dev, sh);
	shell_print(sh, "");
}

/* Defined below; compared against to spot the one self-locking callback. */
static void udc_dwc3_cmd_recover(const struct device *dev, const struct shell *sh);

/*
 * Every dwc3 shell command comes through here, and every one of them touches
 * driver state that the work queue and the ISR are also touching: arming control
 * TRBs, issuing endpoint commands, walking the TRB rings and the event buffer.
 * None of it was serialised against the driver, so a command typed while traffic
 * was running raced the event worker - and the shell exists precisely to be used
 * while something is going wrong, which is the worst moment to corrupt state.
 *
 * So take the UDC mutex around the callback, the same lock udc_dwc3_ep_worker()
 * and the recovery paths hold.
 *
 * One exception: udc_dwc3_cmd_recover() reaches udc_dwc3_recover(), which takes
 * that mutex itself.  The lock is not recursive, so taking it here as well would
 * deadlock the shell thread against itself.
 */
static int dump_cmd2_handler(const struct shell *sh, size_t argc, char **argv,
			     void (*fn)(const struct device *, const struct shell *sh))
{
	const struct device *dev;
	bool self_locking;

	__ASSERT_NO_MSG(argc == 2);

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(sh, "Device %s not found", argv[1]);
		return -ENODEV;
	}

	self_locking = (fn == udc_dwc3_cmd_recover);

	if (!self_locking) {
		udc_lock_internal(dev, K_FOREVER);
	}

	(*fn)(dev, sh);

	if (!self_locking) {
		udc_unlock_internal(dev);
	}

	return 0;
}

static void udc_dwc3_cmd_trb_ctrl_status_in(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 IN");

	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_IN, 128);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}
static int cmd_dwc3_trb_ctrl_status_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_status_in);
}

static void udc_dwc3_cmd_trb_ctrl_status_out(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 OUT");

	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_OUT, 128);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}
static int cmd_dwc3_trb_ctrl_status_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_status_out);
}

static void udc_dwc3_cmd_trb_ctrl_data_out(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA OUT");

	/* 512 is wMaxPacketSize0 for USB3 */
	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_OUT, 512);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
}
static int cmd_dwc3_trb_ctrl_data_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_data_out);
}

static void udc_dwc3_cmd_trb_ctrl_data_in(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA IN");

	/* 512 is wMaxPacketSize0 for USB3 */
	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_IN, 512);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
}
static int cmd_dwc3_trb_ctrl_data_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_data_in);
}

static void udc_dwc3_cmd_trb_ctrl_setup(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP (OUT)");

	/* 512 is wMaxPacketSize0 for USB3 */
	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_OUT, 512);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);
}
static int cmd_dwc3_trb_ctrl_setup(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_setup);
}

static void udc_dwc3_cmd_end_ctrl_in(const struct device *dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_in[0], 0);
}
static int cmd_dwc3_end_ctrl_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_end_ctrl_in);
}

static void udc_dwc3_cmd_end_ctrl_out(const struct device *dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_out[0], UDC_DWC3_DEPCMD_HIPRI_FORCERM);
}
static int cmd_dwc3_end_ctrl_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_end_ctrl_out);
}

static void udc_dwc3_cmd_fake_xfercomplete(const struct device *const dev, const struct shell *sh)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	if (priv->last_xfer_dir == USB_EP_DIR_IN) {
		udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(1));
	} else {
		udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(0));
	}
}
static int cmd_fake_xfercomplete(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_fake_xfercomplete);
}

static void udc_dwc3_cmd_fake_xfercomplete0(const struct device *const dev, const struct shell *sh)
{
	udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(0));
}
static int cmd_fake_xfercomplete0(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_fake_xfercomplete0);
}

static void udc_dwc3_cmd_fake_xfercomplete1(const struct device *const dev, const struct shell *sh)
{
	udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(1));
}
static int cmd_fake_xfercomplete1(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_fake_xfercomplete1);
}

static void udc_dwc3_cmd_dwc3_stall_ctrl_out(const struct device *const dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_out[0]);
}
static int cmd_dwc3_stall_ctrl_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_dwc3_stall_ctrl_out);
}

static void udc_dwc3_cmd_dwc3_stall_ctrl_in(const struct device *const dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_in[0]);
}
static int cmd_dwc3_stall_ctrl_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_dwc3_stall_ctrl_in);
}

static void udc_dwc3_cmd_recover(const struct device *dev, const struct shell *sh)
{
	int ret;

	ret = udc_dwc3_recover(dev);
	if (ret != 0) {
		shell_error(sh, "Failed to recover USB state: %d", ret);
	}
}
static int cmd_dwc3_recover(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_recover);
}

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev == NULL) ? NULL : dev->name;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}
SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

static int cmd_dwc3_trb(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_each_trb);
}

static int cmd_dwc3_evt(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_events);
}

static int cmd_dwc3_reg(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_registers);
}

static int cmd_dwc3_buserr(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_bus_error);
}

static int cmd_dwc3_link(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_link_state);
}

static int cmd_dwc3_fifo(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_fifo_space);
}

static int cmd_dwc3_all(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_all);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dwc3,
	SHELL_CMD_ARG(trb, &dsub_device_name,
		      "Dump an endpoint's TRB buffer\nUsage: trb <device>",
		      cmd_dwc3_trb, 2, 0),
	SHELL_CMD_ARG(evt, &dsub_device_name,
		      "Dump the device event buffer\nUsage: evt <device>",
		      cmd_dwc3_evt, 2, 0),
	SHELL_CMD_ARG(reg, &dsub_device_name,
		      "Dump the device status registers\nUsage: reg <device>",
		      cmd_dwc3_reg, 2, 0),
	SHELL_CMD_ARG(buserr, &dsub_device_name,
		      "Dump the AXI64 bus I/O errors\nUsage: buserr <device>",
		      cmd_dwc3_buserr, 2, 0),
	SHELL_CMD_ARG(link, &dsub_device_name,
		      "Dump the USB link state\nUsage: link <device>",
		      cmd_dwc3_link, 2, 0),
	SHELL_CMD_ARG(fifo, &dsub_device_name,
		      "Dump the FIFO available space\nUsage: fifo <device>",
		      cmd_dwc3_fifo, 2, 0),
	SHELL_CMD_ARG(all, &dsub_device_name,
		      "Dump everything\nUsage: all <device>",
		      cmd_dwc3_all, 2, 0),
	SHELL_CMD_ARG(trb_ctrl_setup, &dsub_device_name,
		      "Send a SETUP TRB to the CTRL OUT endpoint\nUsage: trb_ctrl_setup <device>",
		      cmd_dwc3_trb_ctrl_setup, 2, 0),
	SHELL_CMD_ARG(trb_ctrl_data_in, &dsub_device_name,
		      "Send a DATA TRB to the CTRL IN endpoint\nUsage: trb_ctrl_data_in <device>",
		      cmd_dwc3_trb_ctrl_data_in, 2, 0),
	SHELL_CMD_ARG(trb_ctrl_data_out, &dsub_device_name,
		      "Send a DATA TRB to the CTRL OUT endpoint\nUsage: trb_ctrl_data_out <device>",
		      cmd_dwc3_trb_ctrl_data_out, 2, 0),
	SHELL_CMD_ARG(trb_ctrl_status_in, &dsub_device_name,
		      "Send a STATUS TRB to the CTRL IN endpoint\nUsage: trb_ctrl_status_in <device>",
		      cmd_dwc3_trb_ctrl_status_in, 2, 0),
	SHELL_CMD_ARG(trb_ctrl_status_out, &dsub_device_name,
		      "Send a STATUS TRB to the CTRL OUT endpoint\nUsage: trb_ctrl_status_out <device>",
		      cmd_dwc3_trb_ctrl_status_out, 2, 0),
	SHELL_CMD_ARG(end_ctrl_in, &dsub_device_name,
		      "End a transfer for the CTRL IN endpoint\nUsage: end_ctrl_in <device>",
		      cmd_dwc3_end_ctrl_in, 2, 0),
	SHELL_CMD_ARG(end_ctrl_out, &dsub_device_name,
		      "End a transfer for the CTRL OUT endpoint\nUsage: end_ctrl_out <device>",
		      cmd_dwc3_end_ctrl_out, 2, 0),
	SHELL_CMD_ARG(stall_ctrl_in, &dsub_device_name,
		      "Stall the CTRL IN endpoint\nUsage: stall_ctrl_in <device>",
		      cmd_dwc3_stall_ctrl_in, 2, 0),
	SHELL_CMD_ARG(stall_ctrl_out, &dsub_device_name,
		      "Stall the CTRL OUT endpoint\nUsage: stall_ctrl_out <device>",
		      cmd_dwc3_stall_ctrl_out, 2, 0),
	SHELL_CMD_ARG(recover, &dsub_device_name,
		      "Try to recover from a transfer that is stuck\nUsage: recover <device>",
		      cmd_dwc3_recover, 2, 0),
	SHELL_CMD_ARG(fake_xfercomplete, &dsub_device_name,
		      "Fake an XFERCOMMPLETE(0|1) event\nUsage: cmd_fake_xfercomplete <device>",
		      cmd_fake_xfercomplete, 2, 0),
	SHELL_CMD_ARG(fake_xfercomplete0, &dsub_device_name,
		      "Fake an XFERCOMMPLETE(0) event\nUsage: cmd_fake_xfercomplete0 <device>",
		      cmd_fake_xfercomplete0, 2, 0),
	SHELL_CMD_ARG(fake_xfercomplete1, &dsub_device_name,
		      "Fake an XFERCOMMPLETE(1) event\nUsage: cmd_fake_xfercomplete1 <device>",
		      cmd_fake_xfercomplete1, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dwc3, &sub_dwc3, "Synopsys DWC3 controller commands", NULL);

#endif /* CONFIG_UDC_DWC3_SHELL */