/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT   snps_dwc3

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

/*
 * trace_tag() and friends: empty stubs that keep this file a drop-in replacement
 * for the stock udc_dwc3.c. usbd_core.c, usbd_cdc_acm.c and usbd_ch9.c declare and
 * call them, so without these the driver cannot be swapped in alone.
 *
 * No trace buffer: the stock 128-entry array costs 2 KB of a 64 KB RAM and nothing
 * here reads it. Weak, so a tree can supply the real implementation.
 */
__weak void trace_tag(const char *tag)
{
    ARG_UNUSED(tag);
}

__weak void trace_reset(void)
{
}

__weak void trace_dump(void)
{
}

#include "udc_common.h"

/*
 * Kconfig knobs, defaulted here so this file builds on its own.
 */
#ifndef CONFIG_UDC_DWC3_EVENTS_NUM
/* 16 entries x 4 bytes = the 64-byte cap of this part (not the databook's); see BUILD_ASSERT. */
#define CONFIG_UDC_DWC3_EVENTS_NUM  16
#endif

#ifndef CONFIG_UDC_DWC3_TRB_NUM
/* Per non-control endpoint. Must be >= 2: the control paths index trb_buf[1]. */
#define CONFIG_UDC_DWC3_TRB_NUM 4

#endif

/*
 * Wall-clock cap (ms) on the drain's wait for one late head slot, alongside its
 * poll counts: the busy-wait in udc_dwc3_evt_wait_first() and the sleeps between
 * passes in udc_dwc3_event_thread(). Defined here because the SETUP report
 * threshold below is derived from it.
 */
#define UDC_DWC3_EVT_ARRIVE_MAX_MS                          100u

/*
 * Init, recovery, fault and log-only code goes out of line into one section. When
 * the board relocates this driver to RAM, its filter leaves .text.udc_dwc3_recov* in
 * flash, so only the per-event path is copied to RAM. Without relocation it has no
 * effect.
 */
#define UDC_DWC3_COLD      __noinline __attribute__((cold, section(".text.udc_dwc3_recovery_cold")))

/*
 * How long an armed SETUP may stay undelivered before
 * udc_dwc3_ctrl_setup_wd_check() reports it. Must outlast one full event-arrival
 * wait, or it reports a SETUP that was about to arrive; deriving it from that
 * wait keeps the two in step. Not in Kconfig, so the driver builds standalone.
 */
#define UDC_DWC3_SETUP_WD_REPORT_MS                         (2u * UDC_DWC3_EVT_ARRIVE_MAX_MS)


/*
 * Event-drain thread stack. 512 B overflows: the dispatch goes handle_event ->
 * depcmd -> LOG_INF, and a synchronous log backend formats each message on this
 * stack. The heartbeat reports the remaining headroom in its own "evtstack" line.
 */
#define UDC_DWC3_EVT_STACK_SIZE                             1280
/*
 * Event-drain thread priority. Cooperative, so preemptible work cannot split a
 * pass. Not higher: one step up and the drain runs the instant it is signalled,
 * reads slots the controller has not finished writing, and counts them as late.
 */
#define UDC_DWC3_EVT_THREAD_PRIO                            K_PRIO_COOP(7)


/* TRB memory buffer fields */
#define UDC_DWC3_TRB_STATUS_BUFSIZ_MASK                     GENMASK(23, 0)
#define UDC_DWC3_TRB_STATUS_PCM1_MASK                       GENMASK(25, 24)
/*
 * Short Packet Received, bit 26 of the status dword. On OUT write-back the
 * controller sets it on the last TRB used for the transfer descriptor.
 */
#define UDC_DWC3_TRB_STATUS_SPR                             BIT(26)
#define UDC_DWC3_TRB_STATUS_PCM1_1PKT                       (0x0 << 24)
#define UDC_DWC3_TRB_STATUS_PCM1_2PKT                       (0x1 << 24)
#define UDC_DWC3_TRB_STATUS_PCM1_3PKT                       (0x2 << 24)
#define UDC_DWC3_TRB_STATUS_PCM1_4PKT                       (0x3 << 24)
#define UDC_DWC3_TRB_STATUS_TRBSTS_MASK                     GENMASK(31, 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_OK                       (0x0 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_MISSEDISOC               (0x1 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING             (0x2 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS           (0x4 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING               (0xf << 28)
#define UDC_DWC3_TRB_CTRL_HWO                               BIT(0)
#define UDC_DWC3_TRB_CTRL_LST                               BIT(1)
#define UDC_DWC3_TRB_CTRL_CHN                               BIT(2)
#define UDC_DWC3_TRB_CTRL_CSP                               BIT(3)
#define UDC_DWC3_TRB_CTRL_TRBCTL_MASK                       GENMASK(9, 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL                     (0x1 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP              (0x2 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2           (0x3 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3           (0x4 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA               (0x5 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_ISOCHRONOUS_1              (0x6 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_ISOCHRONOUS_N              (0x7 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB                   (0x8 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP                 (0x9 << 4)

/*
 * watchdog_type when no control stage is watched (no TRBCTL encoding is 0):
 * set when a stage completes, its endpoint is disabled, or transfer state is
 * dropped.
 */
#define UDC_DWC3_WATCHDOG_TYPE_NONE                         0U

#define UDC_DWC3_TRB_CTRL_ISP_IMI                           BIT(10)
#define UDC_DWC3_TRB_CTRL_IOC                               BIT(11)
/*
 * Stream ID / SOF Number. The control word holds only HWO, LST, CHN, CSP, TRBCTL,
 * ISP/IMI, IOC and this field; bits 13:12 and 31:30 are reserved. PCM1 and SPR
 * are in the status dword (Figure 3-1).
 */
#define UDC_DWC3_TRB_CTRL_SIDSOFN_MASK                      GENMASK(29, 14)

/* Incomplete coverage of all fields, but suited for what this driver supports */
#define UDC_DWC3_EVT_MASK                                   GENMASK(11, 0)
#define UDC_DWC3_DEPEVT_EPN_MASK                            GENMASK(5, 1)
#define UDC_DWC3_DEPEVT_KIND_MASK                           GENMASK(9, 6)
#define UDC_DWC3_DEPEVT_RSVD_MASK                           GENMASK(11, 10)
/* The kind field of a DEPEVT type, e.g. UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_XFERCOMPLETE(0)). */
#define UDC_DWC3_DEPEVT_KIND(depevt)                        (((depevt) & UDC_DWC3_DEPEVT_KIND_MASK) >> 6)
/*
 * Fields the controller returns in an Endpoint Command Complete event.
 * Programming Guide 3.30b, Table 3-7 "Device Endpoint-n Events: DEPEVT":
 */
#define UDC_DWC3_DEPEVT_CMDTYP_MASK                         GENMASK(27, 24)
#define UDC_DWC3_DEPEVT_XFERRSCIDX_MASK                     GENMASK(22, 16)
#define UDC_DWC3_DEPEVT_CMDSTATUS_MASK                      GENMASK(15, 12)
/* XferNotReady event status bit 3: Transfer Active. */
#define UDC_DWC3_DEPEVT_STATUS_XFER_ACTIVE                  BIT(15)
#define UDC_DWC3_DEPEVT_XFERCOMPLETE(epn)                   (((epn) << 1) | (0x01 << 6))
#define UDC_DWC3_DEPEVT_XFERINPROGRESS(epn)                 (((epn) << 1) | (0x02 << 6))
#define UDC_DWC3_DEPEVT_XFERNOTREADY(epn)                   (((epn) << 1) | (0x03 << 6))
#define UDC_DWC3_DEPEVT_RXTXFIFOEVT(epn)                    (((epn) << 1) | (0x04 << 6))
#define UDC_DWC3_DEPEVT_STREAMEVT(epn)                      (((epn) << 1) | (0x06 << 6))
#define UDC_DWC3_DEPEVT_EPCMDCMPLT(epn)                     (((epn) << 1) | (0x07 << 6))
/* SPEC 3.30b DEPEVT 15:12 "Event Status" in an XferNotReady event (p.326): */
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK                 GENMASK(13, 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP                (0x0 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA                 (0x1 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS               (0x2 << 12)
/*
 * Event Status (bits 15:12) means different things per event type. The decodes
 * below and the control-stage mask above all live in that field.
 */
/* For XferComplete or XferInProgress: short packet received, or the last
 * packet of an isochronous interval.
 */
#define UDC_DWC3_DEPEVT_STATUS_SHORT                        BIT(13)
/* IOC bit of the TRB that completed */
#define UDC_DWC3_DEPEVT_STATUS_IOC                          BIT(14)
/* For XferComplete: LST bit of the completed TRB */
#define UDC_DWC3_DEPEVT_STATUS_LST                          BIT(15)
/* For XferInProgress: the interval did not complete successfully. This shares
 * bit 15 with LST above - the event type is what tells the two apart.
 */
#define UDC_DWC3_DEPEVT_STATUS_MISSED_ISOC                  BIT(15)
/* For StreamEvt: 4'h1 StreamFound, 4'h2 StreamNotFound, also in bits 15:12 */
#define UDC_DWC3_DEPEVT_STATUS_STREAMFOUND                  (0x1 << 12)
#define UDC_DWC3_DEPEVT_STATUS_STREAMNOTFOUND               (0x2 << 12)
#define UDC_DWC3_DEVT_DISCONNEVT                            (BIT(0) | (0x0 << 8))
#define UDC_DWC3_DEVT_USBRST                                (BIT(0) | (0x1 << 8))
#define UDC_DWC3_DEVT_CONNECTDONE                           (BIT(0) | (0x2 << 8))
#define UDC_DWC3_DEVT_ULSTCHNG                              (BIT(0) | (0x3 << 8))
#define UDC_DWC3_DEVT_WKUPEVT                               (BIT(0) | (0x4 << 8))
#define UDC_DWC3_DEVT_SUSPEND                               (BIT(0) | (0x6 << 8))
#define UDC_DWC3_DEVT_SOF                                   (BIT(0) | (0x7 << 8))
#define UDC_DWC3_DEVT_ERRTICERR                             (BIT(0) | (0x9 << 8))
#define UDC_DWC3_DEVT_CMDCMPLT                              (BIT(0) | (0xa << 8))
#define UDC_DWC3_DEVT_EVNTOVERFLOW                          (BIT(0) | (0xb << 8))
/*
 * Device-event payload, databook Table 3-8 field 24:16 (EvtInfo). For a USB/Link
 * State Change event: EvtInfo[4] is set for SuperSpeed, and EvtInfo[3:0] is the
 * link state at the time of the event, in the same encoding as DSTS.
 */
#define UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK                GENMASK(19, 16)
#define UDC_DWC3_DEVT_EVTINFO_SS                            BIT(20)
/*
 * One line per this many repeats of the same link state; see
 * udc_dwc3_log_link_event(). Prime, so coprime with the event ring size (16): a
 * multiple of 16 would sample the same slot every time when the sampled thing
 * advances by a constant stride.
 */
#define UDC_DWC3_EVT_LINK_LOG_EVERY                         257u
/* Heartbeat tick; bounds how long the event drain can sit unscheduled. */
#define UDC_DWC3_HEARTBEAT_MS                               200u
/*
 * Heartbeats between event-ring statistics lines. A wall-clock period, so the
 * report keeps coming while the event ring is stalled.
 */
#define UDC_DWC3_EVT_STATS_BEATS                            25u
/*
 * Beats between forced statistics lines when nothing has changed; proves
 * liveness on a quiet run. The console is synchronous and a full line costs
 * ~35 ms, so an identical report every 5 s is not worth it.
 */
#define UDC_DWC3_EVT_STATS_FORCE_BEATS                      300u

/*
 * Beats between CORE debug-register samples (25 x 200 ms = 5 s). Several of these
 * registers only mean something against a healthy baseline, which a wedge dump
 * is compared to. Costs ~0.2 lines/s.
 */
#define UDC_DWC3_CORE_DBG_BEATS                             25u


/*
 * Kick the drain if it has not completed a pass in this long while the controller
 * still reports events outstanding. A backstop for a drain that is not running,
 * not a request deadline. Must exceed a stats line's console cost (up to ~35 ms,
 * synchronous); 100 ms covers any real pass and is well inside the host's 5 s.
 */
#define UDC_DWC3_EVT_IDLE_KICK_MS                           100u

/*
 * How long the heartbeat must see GEVNTCOUNT > 0 with nothing handled before the
 * drain is treated as dead (not slow) and the controller recovery reconnects.
 * Well past the drain's own dead-slot skip (UDC_DWC3_EVT_DEAD_SLOT_MS), so a
 * single lost event never gets this far.
 */
#define UDC_DWC3_HB_DRAIN_DEAD_MS                           5000u

/*
 * There is no endpoint inactivity timer, and none may be added. A non-control
 * endpoint holding a controller-owned TRB and retiring nothing is normally idle,
 * waiting for the host; elapsed time cannot tell that from a fault.
 */



/*
 * Cumulative give-ups (looks that found the slot empty) that prove a slot dead
 * regardless of wall clock. The 1000 ms route needs an unbroken run, which
 * anything resetting drain.attempts can starve; this is a reset-proof second route.
 */
#define UDC_DWC3_EVT_DEAD_SLOT_GIVEUPS                      64u

/*
 * Minimum age of a give-up run before any abort route may discard a slot.
 */
#define UDC_DWC3_EVT_DEAD_SLOT_MIN_MS                       200u

#define UDC_DWC3_EVT_DEAD_SLOT_MS                           1000u

/*
 * Minimum age before the drain may act on the look-ahead proof rather than on
 * the timeout above.
 */
#define UDC_DWC3_EVT_LOOKAHEAD_MIN_MS                       50u

/*
 * Build in udc_dwc3_controller_recover(): device-initiated disconnect, core soft
 * reset and reconnect, run from the heartbeat for an erratic error or an event
 * ring that has stopped advancing.
 */
#define UDC_DWC3_CONTROLLER_RECOVER

/*
 * 1: udc_dwc3_depcmd() polls every endpoint command until CmdAct clears and
 * reports its outcome synchronously. 0: the two per-transfer commands are
 * posted without that poll.
 *   Start Transfer: its outcome always arrives - its Command Complete event
 *   (CmdIOC), or, if that event is lost, the STARTING deadline resolver, both
 *   reading DEPCMD or the value saved in cmd_record before a later command
 *   overwrote it; a refusal goes to udc_dwc3_ep_start_refused() on every path.
 *   Update Transfer: cannot be refused by a race - "the controller will detect
 *   that the Update Transfer is unnecessary" for a transfer already completed
 *   (3.2.2.6); its only error, an index never started, is a driver bug, still
 *   logged by the next command's pre-poll.
 * The configuration, stall and End Transfer commands are always polled: they
 * are rare and their callers act on the outcome, and DEPSTARTCFG at power-on
 * "must poll the CmdAct bit" (3.2.2.8). The pre-poll before every command is
 * never skipped.
 */
#ifndef UDC_DWC3_DEPCMD_POST_POLL
#define UDC_DWC3_DEPCMD_POST_POLL                           1
#endif

/*
 * How long a slot must stay empty before the event write is presumed lost
 * rather than late.
 */
#define UDC_DWC3_EVT_MISSED_MS                              1000u


/*
 * Marker for a consumed (or not yet written) slot. 0xFFFFFFFF is never a real
 * event: bit 0 = 1 means device event, which requires bits 7:1 = 0, but here they
 * are 0x7f. Zero cannot be used: it decodes as a valid endpoint event.
 */
#define UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE                   0xFFFFFFFFu
#define UDC_DWC3_DISPATCH_STUCK_MS                          250u
#define UDC_DWC3_DEVT_VNDRDEVTSTRCVED                       (BIT(0) | (0xc << 8))

/* Device Endpoint Commands and Parameters */
#define UDC_DWC3_DEPCMDPAR2(n)                              (0xc800 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR1(n)                              (0xc804 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR0(n)                              (0xc808 + 16 * (n))
#define UDC_DWC3_DEPCMD(n)                                  (0xc80c + 16 * (n))
/* Common fields to DEPCMD */
#define UDC_DWC3_DEPCMD_HIPRI_FORCERM                       (1 << 11)
/*
 * Command Interrupt On Completion: raise Endpoint Command Complete when the
 * command finishes. Required on End Transfer: CmdAct clearing only means the
 * command was accepted; the completion event is the only sign that DMA has
 * stopped, which a new Start Transfer must wait for.
 */
#define UDC_DWC3_DEPCMD_CMDIOC                              BIT(8)
#define UDC_DWC3_DEPCMD_STATUS_MASK                         GENMASK(15, 12)
#define UDC_DWC3_DEPCMD_STATUS_OK                           (0 << 12)
#define UDC_DWC3_DEPCMD_STATUS_CMDERR                       (1 << 12)
#define UDC_DWC3_DEPCMD_XFERRSCIDX_MASK                     GENMASK(22, 16)
/*
 * "No transfer resource index": returned by udc_dwc3_depcmd() when a command
 * fails, and held in ep_data->xferrscidx while none is assigned. XferRscIdx is
 * 7 bits (DEPCMD/DEPEVT [22:16]), so no real index can equal it.
 */
#define UDC_DWC3_XFERRSCIDX_INVALID                         0xffffffffU
/*
 * Returned by udc_dwc3_depcmd() instead of UDC_DWC3_XFERRSCIDX_INVALID for a
 * Start Transfer its pre-poll gave up on: nothing was written. Start Transfer
 * only, so every other caller sees the two values above and nothing else.
 */
#define UDC_DWC3_DEPCMD_NOT_POSTED                          0xfffffffeU
/*
 * Returned by udc_dwc3_depcmd() for a Start or Update Transfer posted without
 * the post-poll (UDC_DWC3_DEPCMD_POST_POLL == 0): written, outcome not yet known.
 */
#define UDC_DWC3_DEPCMD_POSTED                              0xfffffffdU
/* DEPCFG Command and Parameters */
/* Command type occupies bits 3:0 - DEPCFG(1) through DEPSTARTCFG(9). */
#define UDC_DWC3_DEPCMD_CMDTYP_MASK                         GENMASK(3, 0)
#define UDC_DWC3_DEPCMD_DEPCFG                              (1 << 0)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_MASK              GENMASK(2, 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL              (0x0 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_ISOC              (0x1 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_BULK              (0x2 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_INT               (0x3 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_MPS_MASK                 GENMASK(13, 3)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK             GENMASK(21, 17)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK             GENMASK(25, 22)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MASK              GENMASK(31, 30)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_INIT              (0x0 << 30)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_RESTORE           (0x1 << 30)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MODIFY            (0x2 << 30)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_INTRNUM_MASK             GENMASK(4, 0)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN               BIT(8)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN             BIT(9)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN               BIT(10)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_RXTXFIFOEVTEN            BIT(11)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_STREAMEVTEN              BIT(13)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_LIMITTXDMA               BIT(15)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_BINTERVAL_MASK           GENMASK(23, 16)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_STRMCAP                  BIT(24)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK            GENMASK(29, 25)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_BULKBASED                BIT(30)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_FIFOBASED                BIT(31)
#define UDC_DWC3_DEPCMDPAR2_DEPCFG_EPSTATE_MASK             GENMASK(31, 0)
/* DEPXFERCFG Command and Parameters */
#define UDC_DWC3_DEPCMD_DEPXFERCFG                          (0x2 << 0)
#define UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK      GENMASK(15, 0)
/* Other Commands */
#define UDC_DWC3_DEPCMD_DEPGETSTATE                         (0x3 << 0)
#define UDC_DWC3_DEPCMD_DEPSETSTALL                         (0x4 << 0)
#define UDC_DWC3_DEPCMD_DEPCSTALL                           (0x5 << 0)
#define UDC_DWC3_DEPCMD_DEPSTRTXFER                         (0x6 << 0)
#define UDC_DWC3_DEPCMD_DEPUPDXFER                          (0x7 << 0)
#define UDC_DWC3_DEPCMD_DEPENDXFER                          (0x8 << 0)
#define UDC_DWC3_DEPCMD_DEPSTARTCFG                         (0x9 << 0)
#define UDC_DWC3_DEPCMD_CMDACT                              BIT(10)
/*
 * ep_data->cmd.depcmd_last before this endpoint's first command since boot or core
 * soft reset, while DEPCMD reads undefined (databook 1.3.12). CmdTyp 0 is
 * reserved, so no command the driver posts is equal to it.
 */
#define UDC_DWC3_DEPCMD_NONE                                0U

/* Global USB2 (UTMI/ULPI) PHY configuration */
#define UDC_DWC3_GUSB2PHYCFG                                0xC200
#define UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST                     BIT(31)
#define UDC_DWC3_GUSB2PHYCFG_ULPIEXTVBUSINDICATOR           BIT(18)
#define UDC_DWC3_GUSB2PHYCFG_ULPIEXTVBUSDRV                 BIT(17)
#define UDC_DWC3_GUSB2PHYCFG_ULPICLKSUSM                    BIT(16)
#define UDC_DWC3_GUSB2PHYCFG_ULPIAUTORES                    BIT(15)
#define UDC_DWC3_GUSB2PHYCFG_USBTRDTIM_MASK                 GENMASK(13, 10)
#define UDC_DWC3_GUSB2PHYCFG_USBTRDTIM_16BIT                (5 << 10)
#define UDC_DWC3_GUSB2PHYCFG_USBTRDTIM_8BIT                 (9 << 10)
#define UDC_DWC3_GUSB2PHYCFG_ENBLSLPM                       BIT(8)
#define UDC_DWC3_GUSB2PHYCFG_PHYSEL                         BIT(7)
#define UDC_DWC3_GUSB2PHYCFG_SUSPHY                         BIT(6)
#define UDC_DWC3_GUSB2PHYCFG_FSINTF                         BIT(5)
#define UDC_DWC3_GUSB2PHYCFG_ULPI_UTMI_SEL                  BIT(4)
#define UDC_DWC3_GUSB2PHYCFG_PHYIF                          BIT(3)
#define UDC_DWC3_GUSB2PHYCFG_TOUTCAL_MASK                   GENMASK(2, 0)

/* Global USB 3.0 PIPE Control Register */
#define UDC_DWC3_GUSB3PIPECTL                               0xc2c0
#define UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST                    BIT(31)
#define UDC_DWC3_GUSB3PIPECTL_UX_EXIT_IN_PX                 BIT(27)
#define UDC_DWC3_GUSB3PIPECTL_PING_ENHANCEMENT_EN           BIT(26)
#define UDC_DWC3_GUSB3PIPECTL_U1U2EXITFAIL_TO_RECOV         BIT(25)
#define UDC_DWC3_GUSB3PIPECTL_REQUEST_P1P2P3                BIT(24)
#define UDC_DWC3_GUSB3PIPECTL_STARTXDETU3RXDET              BIT(23)
#define UDC_DWC3_GUSB3PIPECTL_DISRXDETU3RXDET               BIT(22)
#define UDC_DWC3_GUSB3PIPECTL_P1P2P3DELAY_MASK              GENMASK(21, 19)
#define UDC_DWC3_GUSB3PIPECTL_DELAYP0TOP1P2P3               BIT(18)
#define UDC_DWC3_GUSB3PIPECTL_SUSPENDENABLE                 BIT(17)
#define UDC_DWC3_GUSB3PIPECTL_DATWIDTH_MASK                 GENMASK(16, 15)
#define UDC_DWC3_GUSB3PIPECTL_ABORTRXDETINU2                BIT(14)
#define UDC_DWC3_GUSB3PIPECTL_SKIPRXDET                     BIT(13)
#define UDC_DWC3_GUSB3PIPECTL_LFPSP0ALGN                    BIT(12)
#define UDC_DWC3_GUSB3PIPECTL_P3P2TRANOK                    BIT(11)
#define UDC_DWC3_GUSB3PIPECTL_P3EXSIGP2                     BIT(10)
#define UDC_DWC3_GUSB3PIPECTL_LFPSFILT                      BIT(9)
#define UDC_DWC3_GUSB3PIPECTL_TXSWING                       BIT(6)
#define UDC_DWC3_GUSB3PIPECTL_TXMARGIN_MASK                 GENMASK(5, 3)
#define UDC_DWC3_GUSB3PIPECTL_TXDEEMPHASIS_MASK             GENMASK(2, 1)
#define UDC_DWC3_GUSB3PIPECTL_ELASTICBUFFERMODE             BIT(0)

/* USB Device Configuration Register */
#define UDC_DWC3_DCFG                                       0xc700
#define UDC_DWC3_DCFG_IGNORESTREAMPP                        BIT(23)
#define UDC_DWC3_DCFG_LPMCAP                                BIT(22)
#define UDC_DWC3_DCFG_NUMP_MASK                             GENMASK(21, 17)
#define UDC_DWC3_DCFG_INTRNUM_MASK                          GENMASK(16, 12)
#define UDC_DWC3_DCFG_PERFRINT_MASK                         GENMASK(11, 10)
#define UDC_DWC3_DCFG_PERFRINT_80                           (0x0 << 10)
#define UDC_DWC3_DCFG_PERFRINT_85                           (0x1 << 10)
#define UDC_DWC3_DCFG_PERFRINT_90                           (0x2 << 10)
#define UDC_DWC3_DCFG_PERFRINT_95                           (0x3 << 10)
#define UDC_DWC3_DCFG_DEVADDR_MASK                          GENMASK(9, 3)
#define UDC_DWC3_DCFG_DEVSPD_MASK                           GENMASK(2, 0)
#define UDC_DWC3_DCFG_DEVSPD_SUPER_SPEED                    (0x4 << 0)
#define UDC_DWC3_DCFG_DEVSPD_HIGH_SPEED                     (0x0 << 0)
#define UDC_DWC3_DCFG_DEVSPD_FULL_SPEED                     (0x1 << 0)

/* Global SoC Bus Configuration Register */
#define UDC_DWC3_GSBUSCFG0                                  0xc100
#define UDC_DWC3_GSBUSCFG1                                  0xc104
/*
 * AXI Pipelined Transfers Burst Request Limit, encoded N-1 (0x0 = 1 outstanding
 * request, 0xf = 16). At the limit the AXI master makes no more ARADDR/AWADDR
 * requests until the associated data phases complete.
 */
#define UDC_DWC3_GSBUSCFG1_PIPETRANSLIMIT_MASK              GENMASK(11, 8)
/* Break DMA transfers at the 1k page boundary instead of 4k. */
#define UDC_DWC3_GSBUSCFG1_EN1KPAGE                         BIT(12)
#define UDC_DWC3_GUCTL1                                     0xc11c
#define UDC_DWC3_GSBUSCFG0_DATRDREQINFO                     GENMASK(31, 28)
#define UDC_DWC3_GSBUSCFG0_DESRDREQINFO                     GENMASK(27, 24)
#define UDC_DWC3_GSBUSCFG0_DATWRREQINFO                     GENMASK(23, 20)
#define UDC_DWC3_GSBUSCFG0_DESWRREQINFO                     GENMASK(19, 16)
#define UDC_DWC3_GSBUSCFG0_DATBIGEND                        BIT(11)
#define UDC_DWC3_GSBUSCFG0_DESBIGEND                        BIT(10)
#define UDC_DWC3_GSBUSCFG0_INCR256BRSTENA                   BIT(7)
#define UDC_DWC3_GSBUSCFG0_INCR128BRSTENA                   BIT(6)
#define UDC_DWC3_GSBUSCFG0_INCR64BRSTENA                    BIT(5)
#define UDC_DWC3_GSBUSCFG0_INCR32BRSTENA                    BIT(4)
#define UDC_DWC3_GSBUSCFG0_INCR16BRSTENA                    BIT(3)
#define UDC_DWC3_GSBUSCFG0_INCR8BRSTENA                     BIT(2)
#define UDC_DWC3_GSBUSCFG0_INCR4BRSTENA                     BIT(1)
#define UDC_DWC3_GSBUSCFG0_INCRBRSTENA                      BIT(0)

/* Global Tx Threshold Control Register */
#define UDC_DWC3_GTXTHRCFG                                  0xc108
#define UDC_DWC3_GTXTHRCFG_USBTXPKTCNTSEL                   BIT(29)
#define UDC_DWC3_GTXTHRCFG_USBTXPKTCNT_MASK                 GENMASK(27, 24)
#define UDC_DWC3_GTXTHRCFG_USBMAXTXBURSTSIZE_MASK           GENMASK(23, 16)
/* Global Rx Threshold Control Register (databook 1.2.4) */
#define UDC_DWC3_GRXTHRCFG                                  0xc10c
#define UDC_DWC3_GRXTHRCFG_USBRXPKTCNTSEL                   BIT(29)
#define UDC_DWC3_GRXTHRCFG_USBRXPKTCNT_MASK                 GENMASK(27, 24)
/*
 * Databook 1.2.4 erratum workaround: clear GRXTHRCFG.UsbRxPktCntSel so a fixed
 * NUMP is sent instead of one derived from the RX threshold. Citations are in
 * udc_dwc3_on_soft_reset().
 */
#define UDC_DWC3_RX_THRESHOLD_WORKAROUND                    1

/* Global control register */
#define UDC_DWC3_GCTL                                       0xc110
#define UDC_DWC3_GCTL_PWRDNSCALE_MASK                       GENMASK(31, 19)
#define UDC_DWC3_GCTL_MASTERFILTBYPASS                      BIT(18)
#define UDC_DWC3_GCTL_BYPSSETADDR                           BIT(17)
#define UDC_DWC3_GCTL_U2RSTECN                              BIT(16)
#define UDC_DWC3_GCTL_FRMSCLDWN_MASK                        GENMASK(15, 14)
#define UDC_DWC3_GCTL_PRTCAPDIR_MASK                        GENMASK(13, 12)
#define UDC_DWC3_GCTL_CORESOFTRESET                         BIT(11)
/*
 * Fixed settle time after the core leaves reset, before any read. The release is
 * a posted write, so GHWPARAMS may briefly return its pre-reset value, pass a
 * poll, then read zero. Polling alone is not enough; this wait comes first.
 */
#define UDC_DWC3_CORE_SETTLE_MS                             50u
/*
 * How long PHYSoftRst (GUSB2PHYCFG / GUSB3PIPECTL) is held, and then how long the
 * PHY clocks get to stabilise before GCTL.CoreSoftReset is released.
 */
#define UDC_DWC3_PHY_RESET_MS                               100u
/*
 * Polls of the core's register file after a soft reset, one per ms, after the
 * settle above. The PHY reset delay alone is far too short for the observed failure.
 */
#define UDC_DWC3_CORE_READY_POLLS                           100
/*
 * How many times the reset sequence is retried if the register file does not come
 * back. init() is also re-entered by the reconnect escalation, and a core that
 * never leaves reset must not be configured.
 */
#define UDC_DWC3_CORE_RESET_ATTEMPTS                        3u

/*
 * Wait for DSTS.DEVCTRLHLT after RunStop is cleared, yielding between polls. The
 * controller cannot halt until its written events are acknowledged, so the drain
 * thread must run meanwhile: the wait is done with the UDC mutex released.
 */
#define UDC_DWC3_HALT_POLL_MS                               1u
#define UDC_DWC3_HALT_POLLS                                 500u
/*
 * Longest sleep between two DEPCMD settles in the controller recovery's quiesce
 * wait (udc_dwc3_quiesce_settle()), for when no event reports progress.
 */
#define UDC_DWC3_QUIESCE_SETTLE_MS                          20u
#define UDC_DWC3_GCTL_DEBUGATTACH                           BIT(8)
#define UDC_DWC3_GCTL_RAMCLKSEL_MASK                        GENMASK(7, 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_BUS_CLK                     (0x00 << 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_PIPE_CLK                    (0x01 << 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_PIPE_DIV2_CLK               (0x02 << 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_MAC2_CLK                    (0x03 << 6)
#define UDC_DWC3_GCTL_SCALEDOWN_MASK                        GENMASK(5, 4)
#define UDC_DWC3_GCTL_DISSCRAMBLE                           BIT(3)
#define UDC_DWC3_GCTL_DSBLCLKGTNG                           BIT(0)

/* Global User Control Register */
#define UDC_DWC3_GUCTL                                      0xc12c
#define UDC_DWC3_GUCTL_NOEXTRDL                             BIT(21)
#define UDC_DWC3_GUCTL_PSQEXTRRESSP_MASK                    GENMASK(20, 18)
#define UDC_DWC3_GUCTL_PSQEXTRRESSP_EN                      BIT(18)
#define UDC_DWC3_GUCTL_SPRSCTRLTRANSEN                      BIT(17)
#define UDC_DWC3_GUCTL_RESBWHSEPS                           BIT(16)
#define UDC_DWC3_GUCTL_CMDEVADDR                            BIT(15)
#define UDC_DWC3_GUCTL_USBHSTINAUTORETRYEN                  BIT(14)
#define UDC_DWC3_GUCTL_DTCT_MASK                            GENMASK(10, 9)
#define UDC_DWC3_GUCTL_DTFT_MASK                            GENMASK(8, 0)

/* Global User Control Register 2 */
#define UDC_DWC3_GUCTL2                                     0xc19c
#define UDC_DWC3_GUCTL2_EN_HP_PM_TIMER                      GENMASK(25, 19)
#define UDC_DWC3_GUCTL2_NOLOWPWRDUR                         GENMASK(18, 15)
#define UDC_DWC3_GUCTL2_RST_ACTBITLATER                     BIT(14)
#define UDC_DWC3_GUCTL2_ENABLEEPCACHEEVICT                  BIT(12)
#define UDC_DWC3_GUCTL2_DISABLECFC                          BIT(11)
#define UDC_DWC3_GUCTL2_RXPINGDURATION                      GENMASK(10, 5)
#define UDC_DWC3_GUCTL2_TXPINGDURATION                      GENMASK(4, 0)

/* USB Device Control register */
#define UDC_DWC3_DCTL                                       0xc704
#define UDC_DWC3_DCTL_RUNSTOP                               BIT(31)
#define UDC_DWC3_DCTL_CSFTRST                               BIT(30)
#define UDC_DWC3_DCTL_HIRDTHRES_4                           BIT(28)
#define UDC_DWC3_DCTL_HIRDTHRES_TIME_MASK                   GENMASK(27, 24)
#define UDC_DWC3_DCTL_APPL1RES                              BIT(23)
#define UDC_DWC3_DCTL_LPM_NYET_THRES_MASK                   GENMASK(23, 20)
#define UDC_DWC3_DCTL_KEEPCONNECT                           BIT(19)
#define UDC_DWC3_DCTL_L1HIBERNATIONEN                       BIT(18)
#define UDC_DWC3_DCTL_CRS                                   BIT(17)
#define UDC_DWC3_DCTL_CSS                                   BIT(16)
#define UDC_DWC3_DCTL_INITU2ENA                             BIT(12)
#define UDC_DWC3_DCTL_ACCEPTU2ENA                           BIT(11)
#define UDC_DWC3_DCTL_INITU1ENA                             BIT(10)
#define UDC_DWC3_DCTL_ACCEPTU1ENA                           BIT(9)
#define UDC_DWC3_DCTL_ULSTCHNGREQ_MASK                      GENMASK(8, 5)
#define UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP              (0x8 << 5)
#define UDC_DWC3_DCTL_ULSTCHNGREQ_RXDETECT                  (0x5 << 5)
#define UDC_DWC3_DCTL_TSTCTL_MASK                           GENMASK(4, 1)

/* USB Device Event Enable Register */
#define UDC_DWC3_DEVTEN                                     0xc708
/* Bits 10, 11 and 13 are RESERVED in this controller - do not write them. */
#define UDC_DWC3_DEVTEN_VNDRDEVTSTRCVEDEN                   BIT(12)
#define UDC_DWC3_DEVTEN_ERRTICERREN                         BIT(9)
#define UDC_DWC3_DEVTEN_SOFEN                               BIT(7)
#define UDC_DWC3_DEVTEN_U3L2L1SUSPEN                        BIT(6)
#define UDC_DWC3_DEVTEN_HIBERNATIONREQEVTEN                 BIT(5)
#define UDC_DWC3_DEVTEN_WKUPEVTEN                           BIT(4)
#define UDC_DWC3_DEVTEN_ULSTCNGEN                           BIT(3)
#define UDC_DWC3_DEVTEN_CONNECTDONEEN                       BIT(2)
#define UDC_DWC3_DEVTEN_USBRSTEN                            BIT(1)
#define UDC_DWC3_DEVTEN_DISCONNEVTEN                        BIT(0)

/* USB Device Event Register */

/* Endpoint Global Event Buffer Address (64-bit) */
#define UDC_DWC3_GEVNTADR(n)                                (0xc400 + 16 * (n))
#define UDC_DWC3_GEVNTADR_LO(n)                             (0xc400 + 16 * (n))
#define UDC_DWC3_GEVNTADR_HI(n)                             (0xc404 + 16 * (n))

/* Endpoint Global Event Buffer Size */
#define UDC_DWC3_GEVNTSIZ(n)                                (0xc408 + 16 * (n))
#define UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK                    BIT(31)

/* Endpoint Global Event Buffer Count (of valid event) */
#define UDC_DWC3_GEVNTCOUNT(n)                              (0xc40c + 16 * (n))
/*
 * Programming Guide 3.30b 1.2.56, Table 1-68: bits 15:0 EVNTCOUNT, 30:16
 * reserved, 31 EVNT_HANDLER_BUSY. Read once per assertion, before processing: a
 * re-read after acknowledging may still count events already handed back, so
 * udc_dwc3_evt_drain() reads it exactly once.
 */
#define UDC_DWC3_GEVNTCOUNT_MASK                            GENMASK(15, 0)
#define UDC_DWC3_GEVNTCOUNT_EVNT_HANDLER_BUSY               BIT(31)

/*
 * DWC_usb3 Programming Guide 3.30b, section 1.3.13 DEV_IMOD[0], Table 1-90
 * (p.253): bits 15:0 DEVICE_IMODI (Interrupt Moderation Interval), bits 31:16
 * DEVICE_IMODC (down counter).
 */
#define UDC_DWC3_DEV_IMOD(n)                                (0xca00 + 4 * (n))
#define UDC_DWC3_DEV_IMOD_DEVICE_IMODI_MASK                 GENMASK(15, 0)
#define UDC_DWC3_DEV_IMOD_DEVICE_IMODC_MASK                 GENMASK(31, 16)
/* 250 ns per unit (Table 1-90), so 1 ms = 4000. */
#define UDC_DWC3_DEV_IMOD_INTERVAL_1MS                      4000U

/*
 * Return the event count in bytes from GEVNTCOUNT(0), fencing afterwards so no
 * later access is reordered ahead of the read.
 */
static inline uint32_t udc_dwc3_gevntcount(const mm_reg_t base)
{
    const uint32_t count = sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) &
                   UDC_DWC3_GEVNTCOUNT_MASK;

#if defined(CONFIG_RISCV)
    __asm__ volatile ("fence iorw,iorw" ::: "memory");
#else
    barrier_dsync_fence_full();
#endif
    return count;
}

/*
 * Acknowledge `words` event words, writing EVNT_HANDLER_BUSY (bit 31) with the
 * count (Table 1-68). The only GEVNTCOUNT write once the ring is live; each drain
 * pass calls it exactly once. words == 0 is legal: credits nothing and clears
 * the handler-busy bit.
 */
static inline void udc_dwc3_gevntcount_ack(const mm_reg_t base,
                       const uint32_t words)
{
    sys_write32((words * sizeof(uint32_t)) |
            UDC_DWC3_GEVNTCOUNT_EVNT_HANDLER_BUSY,
            base + UDC_DWC3_GEVNTCOUNT(0));
}

/*
 * Initialise the count at event-buffer setup: write 0. Credits nothing and leaves
 * EVNT_HANDLER_BUSY untouched (bit 31 is write-1-to-clear, so only an
 * acknowledgement clears it).
 */
static inline void udc_dwc3_gevntcount_enable(const mm_reg_t base)
{
    sys_write32(0, base + UDC_DWC3_GEVNTCOUNT(0));
}

/* Global Device TX FIFO DMA Priority: bit[n] = 1 gives TXFIFO[n] high priority. */
#define UDC_DWC3_GTXFIFOPRIDEV                              0xc610

/* USB Device Active USB Endpoint Enable */
#define UDC_DWC3_DALEPENA                                   0xC720
#define UDC_DWC3_DALEPENA_USBACTEP(n)                       (1 << (n))

/* USB Device Core Identification and Release Number Register */
#define UDC_DWC3_GCOREID                                    0xC120
#define UDC_DWC3_GCOREID_CORE_MASK                          GENMASK(31, 16)
#define UDC_DWC3_GCOREID_REL_MASK                           GENMASK(15, 0)

/* USB Globa Status register */
#define UDC_DWC3_GSTS                                       0xc118
#define UDC_DWC3_GSTS_CBELT_MASK                            GENMASK(31, 20)
#define UDC_DWC3_GSTS_SSIC_IP                               BIT(11)
#define UDC_DWC3_GSTS_OTG_IP                                BIT(10)
#define UDC_DWC3_GSTS_BC_IP                                 BIT(9)
#define UDC_DWC3_GSTS_ADP_IP                                BIT(8)
#define UDC_DWC3_GSTS_HOST_IP                               BIT(7)
#define UDC_DWC3_GSTS_DEVICE_IP                             BIT(6)
#define UDC_DWC3_GSTS_CSRTIMEOUT                            BIT(5)
#define UDC_DWC3_GSTS_BUSERRADDRVLD                         BIT(4)
#define UDC_DWC3_GSTS_CURMOD_MASK                           GENMASK(1, 0)

/* USB Global TX FIFO Size register */
#define UDC_DWC3_GTXFIFOSIZ(n)                              (0xc300 + 4 * (n))
#define UDC_DWC3_GTXFIFOSIZ_TXFSTADDR_MASK                  GENMASK(31, 16)
#define UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK                     GENMASK(15, 0)

/* USB Global RX FIFO Size register */
#define UDC_DWC3_GRXFIFOSIZ(n)                              (0xc380 + 4 * (n))
#define UDC_DWC3_GRXFIFOSIZ_RXFSTADDR_MASK                  GENMASK(31, 16)
#define UDC_DWC3_GRXFIFOSIZ_RXFDEP_MASK                     GENMASK(15, 0)

/* USB Bus Error Address registers */
#define UDC_DWC3_GBUSERRADDR                                0xc130
#define UDC_DWC3_GBUSERRADDR_LO                             0xc130
#define UDC_DWC3_GBUSERRADDR_HI                             0xc134

/* USB Controller Debug register */
#define UDC_DWC3_CTLDEBUG                                   0xe000
#define UDC_DWC3_CTLDEBUG_LO                                0xe000
#define UDC_DWC3_CTLDEBUG_HI                                0xe004

/* USB Analyzer Trace register */
#define UDC_DWC3_ANALYZERTRACE                              0xe008

/* Physical endpoints this driver keeps its own per-endpoint state for. */
#define UDC_DWC3_MAX_EPN                                    16U

/* The video streaming endpoint: excluded from the per-arm TRB trace, for volume. */
#define UDC_DWC3_VIDEO_EP                                   0x85U

/* Endpoint excluded from the per-arm TRB trace. */
#define UDC_DWC3_TRBLOG_SKIP_EP                             UDC_DWC3_VIDEO_EP

/* USB Global Debug Queue/FIFO Space Available register */
#define UDC_DWC3_GDBGFIFOSPACE                              0xc160
#define UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK               GENMASK(31, 16)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_MASK               GENMASK(8, 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXFIFO             (0x0 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXQ                (0x0 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXQ                (0x1 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXREQQ             (0x2 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQQ             (0x3 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXINFOQ            (0x4 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOLSTATUSQ    (0x5 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCHQ         (0x6 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_WREVENTQ           (0x7 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENTQ          (0x8 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK                GENMASK(4, 0)

/* USB Global Debug LTSSM register */
#define UDC_DWC3_GDBGLTSSM                                  0xc164

/* Global Debug LNMCC Register */
#define UDC_DWC3_GDBGLNMCC                                  0xc168

/* Global Debug BMU Register */
#define UDC_DWC3_GDBGBMU                                    0xc16c

/* Global Debug LSP MUX Register - Device*/
#define UDC_DWC3_GDBGLSPMUX_DEV                             0xc170

/* Global Debug LSP MUX Register - Host */
#define UDC_DWC3_GDBGLSPMUX_HST                             0xc170

/* Global Debug LSP Register */
#define UDC_DWC3_GDBGLSP                                    0xc174

/* Global Debug Endpoint Information Register 0 */
#define UDC_DWC3_GDBGEPINFO0                                0xc178

/* Global Debug Endpoint Information Register 1 */
#define UDC_DWC3_GDBGEPINFO1                                0xc17c

/* U3 Root Hub Debug Register */
#define UDC_DWC3_BU3RHBDBG0                                 0xd800

/* USB Device Status register */
#define UDC_DWC3_DSTS                                       0xC70C
#define UDC_DWC3_DSTS_DCNRD                                 BIT(29)
#define UDC_DWC3_DSTS_SRE                                   BIT(28)
#define UDC_DWC3_DSTS_RSS                                   BIT(25)
#define UDC_DWC3_DSTS_SSS                                   BIT(24)
#define UDC_DWC3_DSTS_COREIDLE                              BIT(23)
#define UDC_DWC3_DSTS_DEVCTRLHLT                            BIT(22)
#define UDC_DWC3_DSTS_USBLNKST_MASK                         GENMASK(21, 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U0                      (0x0 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U1                      (0x1 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U2                      (0x2 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U3                      (0x3 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS                  (0x4 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET                  (0x5 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT                (0x6 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_POLL                    (0x7 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_RECOV                   (0x8 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_HRESET                  (0x9 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_CMPLY                   (0xa << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_LPBK                    (0xb << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME            (0xf << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE                (0x0 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE             (0x2 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE           (0x3 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED            (0x4 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND           (0x5 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_RESET                   (0xe << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_RESUME                  (0xf << 18)
#define UDC_DWC3_DSTS_RXFIFOEMPTY                           BIT(17)
#define UDC_DWC3_DSTS_SOFFN_MASK                            GENMASK(16, 3)
#define UDC_DWC3_DSTS_CONNECTSPD_MASK                       GENMASK(2, 0)
#define UDC_DWC3_DSTS_CONNECTSPD_HS                         (0x0 << 0)
#define UDC_DWC3_DSTS_CONNECTSPD_FS                         (0x1 << 0)
#define UDC_DWC3_DSTS_CONNECTSPD_SS                         (0x4 << 0)

/* Device Generic Command and Parameter */
#define UDC_DWC3_DGCMDPAR                                   0xc710
#define UDC_DWC3_DGCMD                                      0xc714
#define UDC_DWC3_DGCMD_STATUS_MASK                          GENMASK(15, 12)
#define UDC_DWC3_DGCMD_STATUS_ERR                           (1 << 12)
#define UDC_DWC3_DGCMD_STATUS_OK                            (0 << 12)
#define UDC_DWC3_DGCMD_ACT                                  BIT(10)
#define UDC_DWC3_DGCMD_IOC                                  BIT(8)
#define UDC_DWC3_DGCMD_MASK                                 GENMASK(7, 0)
/* Other Commands and Parameters */
#define UDC_DWC3_DGCMD_LINKFUNCTION                         (1 << 0)
#define UDC_DWC3_DGCMD_WAKENOTIFNUM                         (3 << 0)
#define UDC_DWC3_DGCMD_FIFOFLUSHONE                         (9 << 0)
/* SPEC 3.30b DGCMD 0Ch: Parameter[4:0] = physical endpoint number. 4.1.10 only. */
#define UDC_DWC3_DGCMD_SET_EP_NRDY                          (0xc << 0)
/* DGCMDPAR for 09h: [4:0] FIFO number, [5] 1 = TX FIFO, 0 = RX FIFO. */
#define UDC_DWC3_DGCMD_FIFOFLUSH_NUM_MASK                   GENMASK(4, 0)
#define UDC_DWC3_DGCMD_FIFOFLUSH_TX                         BIT(5)
/*
 * Wedge guard on a generic command, counted in CSR reads rather than
 * microseconds; see udc_dwc3_dgcmd_wait_idle().
 */
#define UDC_DWC3_DGCMD_POLL_MAX                             200u
#define UDC_DWC3_DGCMD_FIFOFLUSHALL                         (10 << 0)
#define UDC_DWC3_DGCMD_LOOPBACKTEST                         (16 << 0)
#define UDC_DWC3_DGCMD_ROLEREQUEST                          (6 << 0)

/* Hardware parameters */
#define UDC_DWC3_GHWPARAMS0                                 0xc140
#define UDC_DWC3_GHWPARAMS1                                 0xc144
#define UDC_DWC3_GHWPARAMS2                                 0xc148
#define UDC_DWC3_GHWPARAMS3                                 0xc14c
#define UDC_DWC3_GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES_MASK GENMASK(30, 23)
#define UDC_DWC3_GHWPARAMS3_NUM_IN_EPS_MASK                 GENMASK(22, 18)
#define UDC_DWC3_GHWPARAMS3_NUM_EPS_MASK                    GENMASK(17, 12)
#define UDC_DWC3_GHWPARAMS4                                 0xc150
#define UDC_DWC3_GHWPARAMS4_BMU_LSP_DEPTH_MASK              GENMASK(31, 28)
#define UDC_DWC3_GHWPARAMS4_BMU_PTL_DEPTH_M1_MASK           GENMASK(27, 24)
#define UDC_DWC3_GHWPARAMS4_CACHE_TRBS_PER_TRANSFER_MASK    GENMASK(5, 0)
#define UDC_DWC3_GHWPARAMS5                                 0xc154
#define UDC_DWC3_GHWPARAMS5_DFQ_FIFO_DEPTH_MASK             GENMASK(27, 22)
#define UDC_DWC3_GHWPARAMS5_DWQ_FIFO_DEPTH_MASK             GENMASK(21, 16)
#define UDC_DWC3_GHWPARAMS5_TXQ_FIFO_DEPTH_MASK             GENMASK(15, 10)
#define UDC_DWC3_GHWPARAMS5_RXQ_FIFO_DEPTH_MASK             GENMASK(9, 4)
#define UDC_DWC3_GHWPARAMS5_BMU_BUSGM_DEPTH_MASK            GENMASK(3, 0)
#define UDC_DWC3_GHWPARAMS6                                 0xc158
#define UDC_DWC3_GHWPARAMS6_RAM0_DEPTH_MASK                 GENMASK(31, 16)
#define UDC_DWC3_GHWPARAMS6_PSQ_FIFO_DEPTH_MASK             GENMASK(5, 0)
#define UDC_DWC3_GHWPARAMS7                                 0xc15c
#define UDC_DWC3_GHWPARAMS7_RAM2_DEPTH_MASK                 GENMASK(31, 16)
#define UDC_DWC3_GHWPARAMS7_RAM1_DEPTH_MASK                 GENMASK(15, 0)
#define UDC_DWC3_GHWPARAMS8                                 0xc600

/* Helper macros */
#define LO32(n) ((uint32_t)((uint64_t)(n) & 0xffffffff))
#define HI32(n) ((uint32_t)((uint64_t)(n) >> 32))
#define _EP_DATA_FROM_EPN(cfg, epn) \
    (((epn) & 1) ? &(cfg)->ep_data_in[(epn) >> 1] : &(cfg)->ep_data_out[(epn) >> 1])
/* True if an endpoint number carried by an event is one this driver owns. */
#define _EPN_IS_VALID(cfg, epn) \
    (((epn) & 1) ? ((uint32_t)((epn) >> 1) < (cfg)->num_in_eps) \
             : ((uint32_t)((epn) >> 1) < (cfg)->num_out_eps))
#define _NUM_FIFO_SPACE 16
/*
 * Queue types dumped by "dwc3 fifo". Bounds udc_dwc3_fifo_regs[] and
 * max_bytes_avail[][], so adding a queue type without raising it fails to compile
 * instead of overrunning.
 */
#define _NUM_FIFO_REGS  8

/*
 * TRB: one DMA request from the CPU to the DWC3 core, layout fixed by the
 * databook. No cache flush needed: this SoC has no data cache.
 */
struct udc_dwc3_trb {
    uint32_t    addr_lo;
    uint32_t    addr_hi;
    uint32_t    status;
    uint32_t    ctrl;
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
    /* USB device configuration */
    int                     maximum_speed_idx;
    /* Pointers to event buffer fetched by DWC3 with DMA */
    volatile uint32_t       *evt_buf;
    /*
     * Driver-owned SETUP buffer (SPEC 3.30b 4.4 step 1: "Setup a Control-Setup
     * TRB"). The driver arms it on every return to the Setup phase; the stack's
     * queue never does. NOCACHE, like every buffer the controller writes (TRBs,
     * event ring, UDC pool heap via CONFIG_UDC_BUF_FORCE_NOCACHE).
     * udc_setup_received() copies the 8 bytes into the stack's SETUP buffer.
     */
    uint8_t                 *setup_buf;
    /* Data used by vendor-specific functions ("quirks") */
    const void              *quirk_config;
    void                    *quirk_data;
    /* IRQ management functions */
    void (*irq_connect_func)(void);
    void (*irq_enable_func)(void);
    void (*irq_disable_func)(void);
    /* Number of hardware endpoint set for input or output */
    uint8_t                 num_in_eps;
    uint8_t                 num_out_eps;
};


/*
 * Transfer state of one endpoint: one variable, one owner per transition. It
 * tells "no transfer running" apart from "index not known yet", which xferrscidx
 * alone cannot. That matters: a second Start on an endpoint that already holds a
 * transfer takes a fresh transfer resource that is never returned.
 *
 * Invariants (checkable from udc_dwc3_ep_state_set() and its callers):
 *   - DEPSTRTXFER only from IDLE.
 *   - DEPUPDXFER and DEPENDXFER only from RUNNING, where xferrscidx is valid.
 *   - DEPSTARTCFG / DEPCFG / DEPXFERCFG belong to endpoint enable, only from IDLE.
 *   - Never DEPXFERCFG on a recovery path: each issue allocates another resource.
 * Each transition is written next to the command that causes it.
 *
 * cfg.stat.enabled is owned by udc_common.c, not folded in here. cfg.stat.halted
 * is written only by the two stall commands, so it always matches the controller.
 */
enum udc_dwc3_ep_state {
    UDC_DWC3_EP_IDLE = 0,     /* no transfer, no controller resource held   */
    UDC_DWC3_EP_STARTING,     /* DEPSTRTXFER posted, still executing        */
    UDC_DWC3_EP_START_UNKNOWN,/* DEPSTRTXFER outcome undetermined past the
                   * deadline: the controller may or may not hold
                   * a transfer resource, so no Start may follow */
    UDC_DWC3_EP_RUNNING,      /* transfer live, xferrscidx valid            */
    UDC_DWC3_EP_ENDING,   /* DEPENDXFER posted, awaiting completion     */
    UDC_DWC3_EP_END_UNKNOWN,  /* DEPENDXFER outcome undetermined past the
                   * deadline: the controller may still own the
                   * ring, so nothing may be reclaimed          */
};

/*
 * Work owed to an endpoint once no command is open on it, carried out by
 * udc_dwc3_ep_recover(). A bitmask, since several can be owed at once (e.g. a
 * Clear Stall deferred for command ordering and a resume deferred for 3.2.2.7).
 * Kept across transfer-state resets. udc_dwc3_ep_disable() withdraws all but
 * the cancel; a core reset or controller disable (udc_dwc3_drop_xfer_state())
 * withdraws everything.
 */
enum udc_dwc3_ep_pending {
    UDC_DWC3_EP_PEND_NONE       = 0,
    UDC_DWC3_EP_PEND_CLEAR_STALL    = BIT(0), /* leave halt once the End reports */
    UDC_DWC3_EP_PEND_RESUME     = BIT(1), /* re-establish the transfer       */
    UDC_DWC3_EP_PEND_DEQUEUE    = BIT(3), /* release the ring, cancel the buffers */
    /*
     * An Update Transfer refused because the Start was still open. Not recovery
     * work: udc_dwc3_ep_update_owed() issues it once the Start has an index.
     */
    UDC_DWC3_EP_PEND_UPDATE         = BIT(5),
};


/*
 * All data specific to one endpoint for use by the driver.
 */
struct udc_dwc3_ep_data {
    /* Allow to cast a pointer between ep_data and ep_cfg */
    struct udc_ep_config            cfg;
    /*
     * epn, trb_buf and xferrscidx are read by name by the vendor quirk API
     * (udc_dwc3_lattice_usb23.h), so they stay at top level.
     *
     * Endpoint number (physical address): the logical address is on ep_cfg.
     */
    int                             epn;
    /*
     * The TRB ring. trb_buf[CONFIG_UDC_DWC3_TRB_NUM - 1] is the LINK TRB.
     * EP0 uses trb_buf[0] and [1] only.
     */
    volatile struct udc_dwc3_trb    *trb_buf;
    /*
     * Transfer resource index for endpoint commands, assigned by the controller.
     * UDC_DWC3_XFERRSCIDX_INVALID until a Start Transfer reports one, and again
     * after DEPSTARTCFG, which reassigns resources and invalidates all earlier
     * indexes.
     */
    uint32_t                        xferrscidx;
    /* A work queue entry to process the buffers to submit on that endpoint */
    struct k_work                   work;
    /* To re-queue cancelled buffers after an endpoint is disabled */
    struct k_fifo                   requeue_fifo;
    /* Point back to the device for work queues */
    const struct device             *dev;
    /*
     * Software side of the TRB ring: the buffer armed in each slot of trb_buf,
     * one slot less, as the LINK TRB never holds one. Slot i is occupied if and
     * only if net_buf[i] != NULL: the ring is full when net_buf[head] != NULL
     * and holds something when net_buf[tail] != NULL.
     */
    struct {
        struct net_buf  *net_buf[CONFIG_UDC_DWC3_TRB_NUM - 1];
        /* Next slot to arm, and oldest slot not yet retired */
        uint32_t        head;
        uint32_t        tail;
    } ring;
    /* The transfer on this endpoint and the work owed to it. */
    struct {
        /*
         * What this endpoint is doing; see enum udc_dwc3_ep_state for the
         * rules. cfg.stat.busy is derived from this and the ring -
         * udc_dwc3_ep_busy_sync().
         */
        enum udc_dwc3_ep_state  state;
        /* Work owed once no command is open - see enum udc_dwc3_ep_pending. */
        uint8_t                 pending;
        /*
         * Index the outstanding End Transfer was posted against. Posting an
         * End invalidates xferrscidx (a successful End frees the resource),
         * but a refused End leaves the transfer holding it. The refusal may be
         * learnt long after the posting call returned, in
         * udc_dwc3_ep_resolve_cmd() from ENDING or END_UNKNOWN, so the index
         * is kept here. Without it the endpoint returns to RUNNING with no
         * index: Update Transfer is refused and no End can reclaim the
         * resource. Valid only while an End is outstanding.
         */
        uint32_t                end_idx;
    } xfer;
    /* This endpoint's DEPCMD bookkeeping. */
    struct {
        /*
         * The last command written (without CmdAct), or UDC_DWC3_DEPCMD_NONE
         * while DEPCMD reads undefined.
         */
        uint32_t    depcmd_last;
        /*
         * DEPCMD as read for this endpoint's open Start or End Transfer when
         * another command was posted over it. If the Command Complete is lost,
         * DEPCMD is the only record of how that command ended, and any later
         * command (e.g. a stack Set Stall) overwrites it. udc_dwc3_depcmd()
         * saves it from its pre-poll read; udc_dwc3_cmd_outcome() uses it once
         * DEPCMD has moved on. 0 (no command type) when none; a new Start or
         * End clears it.
         */
        uint32_t    cmd_record;
        /*
         * Cycle stamp of the last Start or End Transfer posted on this
         * endpoint. STARTING and ENDING last at most UDC_DWC3_CMD_UNKNOWN_MS
         * from here; past that the endpoint moves to the matching UNKNOWN state.
         */
        uint32_t    cmd_t0;
    } cmd;
    /*
     * Pool generation (priv->epcfg.epoch) in which this endpoint took its
     * transfer resource with DEPXFERCFG. Only DEPSTARTCFG frees resources, so
     * DEPXFERCFG runs again only after one: equal values mean "already holds a
     * resource".
     */
    uint32_t                        rsc_epoch;
    /* Diagnostics only: nothing decides on these. */
    struct {
        /*
         * Arms and retires on this endpoint. A bulk endpoint that stops
         * accepting host traffic shows up here and nowhere else device-side.
         */
        uint32_t    n_arm;
        uint32_t    n_retire;
        /* The last command's error has already been logged. */
        bool        cmd_reported;
    } diag;
};

/*
 * What the event drain is doing right now.
 */
enum udc_dwc3_drain_state {
    UDC_DWC3_DRAIN_IDLE = 0,    /* controller owes nothing */
    UDC_DWC3_DRAIN_RUNNING,     /* taking events */
    UDC_DWC3_DRAIN_WAITING,     /* head slot owed but empty: a late-write episode */
};

/*
 * What one arrival wait concluded. The wait reports; the caller decides what
 * state that puts the drain in - see the dispatch in udc_dwc3_evt_drain().
 */
enum udc_dwc3_wait_result {
    UDC_DWC3_WAIT_ARRIVED,  /* the write landed inside the budget */
    UDC_DWC3_WAIT_EXPIRED,  /* budget spent; the slot is still empty */
};

/*
 * Drain state. Written by the drain thread during a pass, without a lock, and
 * reset together with the event ring by udc_dwc3_on_soft_reset() only, while
 * the drain is shut out (udc_dwc3_evt_block()). A pass never sleeps or blocks
 * before its write-back, so no reset lands mid-pass. No field is wider than 32
 * bits, so the heartbeat's lockless reads never see a torn value. Do not widen
 * any field to 64 bits.
 */
struct udc_dwc3_drain {
    uint32_t    state;      /* enum udc_dwc3_drain_state */
    uint32_t    slot;       /* slot the episode is stuck on */
    uint32_t    since;      /* cycle stamp the episode opened */
    uint32_t    attempts;   /* consecutive give-ups on that slot */
    uint32_t    gc0;        /* GEVNTCOUNT when the episode opened */
    uint32_t    watched_us; /* time actually spent looking at the slot */
    uint32_t    quiet;      /* nothing was printed inside this episode */
    uint32_t    counted;    /* this episode was already counted as missed */
    /*
     * The slot held back by the last skip, and whether one is being watched.
     * Armed only when a skip stepped over less than the controller owed, so
     * the held slot lies inside the stuck group and no newly generated event
     * can land in it - see udc_dwc3_evt_skip_dead_slot().
     */
    uint32_t    skip_watch_slot;
    bool        skip_watch;
};

/* Reset the whole drain state as one act - see the enum above. */
static inline void udc_dwc3_drain_reset(struct udc_dwc3_drain *const d)
{
    *d = (struct udc_dwc3_drain){ .state = UDC_DWC3_DRAIN_IDLE };
}

/* One snapshot of the core debug registers (LTSSM, BMU, LNMCC, LSP, EPINFO). */
struct udc_dwc3_core_dbg {
    uint32_t    ltssm;
    uint32_t    bmu;
    uint32_t    lnmcc;
    uint32_t    lsp;
    uint32_t    epinfo0;
    uint32_t    epinfo1;
};

/*
 * Diagnostics: counters, maxima, timestamps and report state for the heartbeat,
 * the dumps and the shell. None of it changes what the driver does to the
 * controller; driver behaviour lives in struct udc_dwc3_data.
 */
struct udc_dwc3_diag {
    uint32_t                    evt_stack_free;     /* smallest observed headroom, bytes */
    /* DEVCTRLHLT never seen after RunStop cleared */
    uint32_t                    halt_timeouts;
#if CONFIG_UDC_DWC3_SHELL
    /* FIFO space initial values */
    uint16_t                    max_bytes_avail[_NUM_FIFO_SPACE][_NUM_FIFO_REGS];
    /* Direction of the last control stage armed (shell fake-XferComplete). */
    uint8_t                     last_xfer_dir;
#endif
    /*
     * Event-buffer posted-write race: events not yet written on first read,
     * give-ups, and events handled.
     */
    uint32_t                    evt_late;
    uint32_t                    evt_gaveup;
    /* worst announced-but-unread backlog, bytes */
    uint32_t                    evt_gevntcount_hwm;
    /* completions the heartbeat sweep returned */
    uint32_t                    evt_sweep_rescued;
    uint32_t                    evt_sweep_runs;     /* sweeps that found something to drain */
    /* stalls ended early by the look-ahead proof */
    uint32_t                    evt_lookahead_short;
    uint32_t                    evt_gaveup_us_max;  /* worst uninstrumented fill latency, us */
    uint32_t                    evt_missed;         /* give-up runs presumed a lost write */
    uint32_t                    evt_missed_frozen;  /* of those, with GEVNTCOUNT not moving */
    uint32_t                    evt_gaveup_multi;   /* runs opened owed MORE than one event */
    /* counter signature at the last stats line */
    uint32_t                    stats_sig_last;
    /* beats since that line, for the forced floor */
    uint32_t                    stats_quiet_beats;
    /* heartbeat had to restart a stopped drain */
    uint32_t                    evt_kick;
    /* Interrupts taken versus worker passes entered. */
    uint32_t                    evt_isr;            /* interrupt handler invocations */
    uint32_t                    evt_worker_runs;    /* event worker passes entered */
    uint32_t                    evt_skipped;        /* events discarded to free a full ring */
    /*
     * Skips later proved wrong: the slot after a skip filled, and the controller
     * writes in order, so the skipped slots were late, not lost. Zero means
     * skipping only ever discarded events that never arrived.
     */
    uint32_t                    evt_skip_refuted;
    uint32_t                    evt_link_total;     /* USB/Link State Change events seen */
    /* consecutive events reporting the same state */
    uint32_t                    evt_link_run;
    uint32_t                    evt_link_last;      /* that state, EvtInfo[3:0] */
    uint32_t                    dispatch_evt;       /* event being dispatched now, 0 = none */
    uint32_t                    dispatch_t0;        /* cycle stamp when that drain pass began */
    /* Worst late-but-arrived wait: polls is the lower bound, us the upper. */
    uint32_t                    evt_late_polls_max;
    uint32_t                    evt_late_us_max;
    uint32_t                    evt_midzero;        /* passes that stopped on an empty slot */
    uint32_t                    post_fail_total;    /* completions posted to a full usbd queue */
    uint32_t                    drain_dead_resets;  /* reconnects issued for a dead ring */
    /* promotions to START_UNKNOWN / END_UNKNOWN */
    uint32_t                    ep_cmd_unknown;
    uint32_t                    core_dbg_beats;     /* beats since the last CORE debug sample */
    /* Last sample reported, so an unchanged core is not re-printed. */
    struct udc_dwc3_core_dbg    core_dbg_last;
    /* beats since that report, for the forced floor */
    uint32_t                    core_dbg_quiet;
    uint32_t                    ctrl_start_fail;    /* Start Transfer commands rejected */
    /* Stage buffers of abandoned transfers returned in the Setup phase. */
    uint32_t                    ctrl_stale_returned;
    /* The control endpoint and stage the watchdog is guarding. */
    struct udc_dwc3_ep_data     *watchdog_ep;
    uint32_t                    watchdog_type;
    /*
     * SETUP watchdog ageing (heartbeat):
     *   wd_gen            armed SETUPs
     *   wd_seen/wd_beats  how long the current one has stood
     *   wd_reported       suppresses a second report for the same SETUP
     */
    uint32_t                    ctrl_setup_wd_gen;
    uint32_t                    ctrl_setup_wd_seen;
    uint32_t                    ctrl_setup_wd_beats;
    bool                        ctrl_setup_wd_reported;
    /* Control transfers the host abandoned by starting a new SETUP. */
    uint32_t                    ctrl_setup_pending;
    /* SETUP watchdog reports issued. */
    uint32_t                    ctrl_setup_wd_fire;
    /* Non-control OUT buffers the class sized to a non-multiple of MaxPacketSize. */
    uint32_t                    out_unaligned;
    /* Set Endpoint NRDY commands issued on U3 exit */
    uint32_t                    u3_exit_nrdy;
    uint32_t                    ctrl_stall_issued;  /* Set Stalls issued for steps 2 and 5b */
    uint32_t                    ep_halts;           /* halts set on non-control endpoints */
    /* The two ends of a control transfer, counted independently. */
    uint32_t                    ctrl_setup_done;    /* SETUP stages retired */
    /* Retires on all non-control endpoints; liveness proxy for the SETUP watchdog. */
    uint32_t                    nonctrl_done;
    /* ctrl_setup_done when armed */
    uint32_t                    ctrl_setup_wd_snap_setup;
    /* nonctrl_done when armed */
    uint32_t                    ctrl_setup_wd_snap_nonctrl;
    /* the one-shot bus/DMA config report has run */
    bool                        buscfg_logged;
    /* Descriptors overwritten while the controller still owned them. */
    uint32_t                    trb_stomp;
    /* Reclaims completed, so the ring was cleared and the TxFIFO flushed. */
    uint32_t                    ctrl_reclaim_done;
    uint32_t                    ctrl_status_done;   /* status stages retired (IN and OUT) */
    /*
     * Device-wide command counts. Per-endpoint bookkeeping (cmd.depcmd_last,
     * diag.cmd_reported) lives on the endpoint.
     */
    uint32_t                    depcmd_n_err;       /* completion seen, CmdStatus != OK     */
    uint32_t                    depcmd_n_timeout;   /* fast poll expired, status unknown    */
    /* Heartbeat liveness, measured. */
    uint32_t                    hb_beats;
    uint32_t                    hb_gap_ms_max;
    uint32_t                    hb_last_t;
    /* Work-queue latency, to tell a late timer from a late work run. */
    uint32_t                    hb_submit_t;
    uint32_t                    hb_q_ms_max;
    /*
     * A periodic k_timer stays on the UDC_DWC3_HEARTBEAT_MS grid under any
     * load; only the work run can be lost. These count expiries and work
     * runs coalesced.
     */
    uint32_t                    hb_expiries;
    uint32_t                    hb_coalesced;
};

/*
 * Per-instance driver state, read and written at run time: udc_get_private(dev).
 * Only what the driver acts on lives here, by concern; counters and report state
 * are in diag.
 */
struct udc_dwc3_data {
    DEVICE_MMIO_NAMED_RAM(base);
    const struct device     *dev;       /* back-reference to the device */

    /* Kernel objects. */
    struct k_sem            evt_sem;    /* ISR -> drain thread */
    k_thread_stack_t        *evt_stack;
    struct k_thread         *evt_thread;
    /*
     * Liveness: a periodic k_timer submits heartbeat_work, so no beat depends on
     * a work item re-arming itself.
     */
    struct k_timer          heartbeat_timer;
    struct k_work           heartbeat_work;
    /* Runs udc_dwc3_evt_force(): one DGCMD to make the controller emit. */
    struct k_work           nudge_work;
    /*
     * Held by udc_dwc3_dgcmd(), the only DGCMDPAR/DGCMD writer; its callers run
     * on the drain thread and the UDC work queue.
     */
    struct k_spinlock       dgcmd_lock;

    /* Event ring and its drain. */
    struct {
        uint32_t                next;           /* ring slot the drain reads next */
        /* one pass, copied out of the ring */
        uint32_t                copy[CONFIG_UDC_DWC3_EVENTS_NUM];
        uint32_t                handled;        /* events dispatched */
        /*
         * Completions this pass posted to a full usbd queue (-ENOMSG). Counted in
         * udc_dwc3_drain_completed() and reported once by the pass, after the
         * UDC mutex is released: a synchronous log line per failure would keep
         * usbd off the CPU and make the next post fail too.
         */
        uint32_t                post_fail;
        /*
         * The drain pass's one GEVNTCOUNT read, shared with every other reader.
         * The controller updates it concurrently, so two reads never describe the
         * same instant. Read only in udc_dwc3_evt_drain() and
         * udc_dwc3_drain_helper(); everything else uses this copy.
         */
        uint32_t                gc_last;
        struct udc_dwc3_drain   drain;          /* drain state - see the enum above */
        uint32_t                force_t0;       /* cycle stamp of the last forced command */
        uint32_t                worker_exit_t0; /* cycle stamp when the drain last exited */
        /* Drain-dead detection: handled at the last beat, and beats with none since. */
        uint32_t                hb_last_handled;
        uint32_t                hb_stuck_beats;
    } evt;

    /* Control transfer (EP0/EP1). */
    struct {
        /*
         * Control transfer progress, as one value rather than flags: a request
         * that ends without a status stage must not leave a stale "data done"
         * flag for the next request's first XferNotReady(Data).
         */
        enum udc_dwc3_ctrl_state {
            UDC_DWC3_CTRL_IDLE = 0,     /* Setup phase: the Setup TRB is (to be) armed */
            UDC_DWC3_CTRL_SETUP_DONE,   /* SETUP taken; data or status not yet armed */
            UDC_DWC3_CTRL_DATA_DONE,    /* data stage retired; waiting XferNotReady(Status) */
            UDC_DWC3_CTRL_STATUS_READY, /* XferNotReady(Status) taken; status not yet armed */
            UDC_DWC3_CTRL_STATUS_ARMED, /* status TRB armed */
            UDC_DWC3_CTRL_DATA_ARMED,   /* data TRB armed */
            /*
             * udc_dwc3_ctrl_ep_recover() in progress: back to Step 1, ending both
             * halves. _STALL ends it with Set Stall. No other state is entered until
             * the recovery finishes (the stage handlers all defer to it).
             */
            UDC_DWC3_CTRL_RECOVERING,
            UDC_DWC3_CTRL_RECOVERING_STALL,
        } state;
        /* Copy of the current SETUP packet, taken before the stack can react. */
        struct usb_setup_packet setup;
        /*
         * The data stage completed with SetupPending: the host abandoned this
         * transfer. Per 4.4.2 steps 4 and 6 / Figure 4-2 the flow still reaches
         * XferNotReady(Status); that is where we Set Stall and return to Step 1.
         */
        bool                    setup_pending;
    } ctrl;

    /* Endpoint configuration (DEPSTARTCFG). */
    struct {
        uint8_t     first_ep;   /* first endpoint configured */
        /*
         * DEPSTARTCFG has been tried for this configuration. Cleared on bus
         * reset. It reassigns the transfer-resource pool and resets every
         * endpoint, so re-running it on a SET_INTERFACE that touches first_ep
         * would tear down streaming endpoints. Set even when it was skipped or
         * refused: the pool keeps its assignment and the next bus reset tries
         * again (udc_dwc3_on_set_config_or_interface()).
         */
        bool        pool_assigned;
        /*
         * Transfer-resource pool generation; bumped by DEPSTARTCFG, the only
         * command that frees resources. Compared with each ep_data->rsc_epoch.
         */
        uint32_t    epoch;
        /*
         * DALEPENA as last written: udc_dwc3_dalepena_set() writes both, and the
         * core reset in udc_dwc3_init() zeroes both. Read instead of the
         * register on the arm paths (endpoint worker, udc_dwc3_trb_bulk()).
         */
        uint32_t    dalepena;
    } epcfg;

    /*
     * SPEC 4.1.10: the link is in U3 (entered, not yet left), and the physical
     * endpoints that had an active transfer on entry.
     */
    struct {
        bool        in_u3;
        uint32_t    u3_active_eps;
    } link;

    /*
     * Controller run/recover state. Written by udc_dwc3_enable(),
     * udc_dwc3_disable(), the ErrticErr event and udc_dwc3_controller_recover().
     * While recovery waits in UDC_DWC3_RUN_STOPPING, quiesce_sem wakes it on each
     * transfer/control state change.
     */
    struct {
        enum udc_dwc3_run_state {
            UDC_DWC3_RUN = 0,       /* normal operation */
            UDC_DWC3_RUN_RESET_OWED,    /* SPEC 3.3.2 ErrticErr: the heartbeat resets */
            UDC_DWC3_RUN_STOPPING,      /* SPEC 4.1.8: transfers ending before RunStop=0;
                             * no new transfer may start */
            UDC_DWC3_RUN_RESETTING,     /* the event ring and drain state are being
                             * re-initialised (udc_dwc3_evt_block()):
                             * no drain pass, no new transfer */
        } state;
        struct k_sem quiesce_sem;
    } run;

    /* Counters and report state only - see struct udc_dwc3_diag. */
    struct udc_dwc3_diag    diag;
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
 * Vendor quirks: vendor-specific hooks, overridable per SoC.
 */

struct udc_dwc3_vendor_quirks {
    int (*preinit)(const struct device *const dev);
    int (*init)(const struct device *const dev);
    int (*enable)(const struct device *const dev);
};

/* Helper for accessing vendor quirks */
#define UDC_DWC3_QUIRK_CFG(dev)     (((const struct udc_dwc3_config *)(dev->config))->quirk_config)
#define UDC_DWC3_QUIRK_DATA(dev)    (((const struct udc_dwc3_config *)(dev->config))->quirk_data)

#if DT_HAS_COMPAT_STATUS_OKAY(snps_dwc3 /* <- replace with your more specific compatible */)
#include "udc_dwc3_lattice_usb23.h"
#endif

/* Wrapper functions that fallback to returning 0 if no quirk is needed */
#define UDC_DWC3_QUIRK_FUNC_DEFINE(fn)                      \
    static inline int udc_dwc3_quirk_##fn(const struct device *const dev)   \
    {                                   \
        if (udc_dwc3_vendor_quirks.fn != NULL) {            \
            return udc_dwc3_vendor_quirks.fn(dev);          \
        }                               \
                                        \
        return 0;                           \
    }

UDC_DWC3_QUIRK_FUNC_DEFINE(preinit);
UDC_DWC3_QUIRK_FUNC_DEFINE(init);
UDC_DWC3_QUIRK_FUNC_DEFINE(enable);

#define DEV_CFG(dev)    ((const struct udc_dwc3_config *)(dev->config))

static int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr);
static int udc_dwc3_ep_disable(const struct device *const dev, struct udc_ep_config *const ep_cfg);
/*
 * Re-establish an endpoint: configure it, enable it in DALEPENA and arm what is
 * queued.
 */
static int udc_dwc3_ep_resume(const struct device *const dev,
                  struct udc_dwc3_ep_data *const ep_data);
/* Flush one TX FIFO; defined next to the control-stage checks that call it. */
static void udc_dwc3_fifo_flush_tx(const struct device *const dev, const uint8_t fifo);

/*
 * Enable or disable an endpoint in DALEPENA, and in priv->epcfg.dalepena, its
 * copy. The only DALEPENA writer; a core reset clears both (udc_dwc3_init()).
 */
static UDC_DWC3_COLD void udc_dwc3_dalepena_set(const struct device *const dev,
                        const int epn, const bool on)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t bit = UDC_DWC3_DALEPENA_USBACTEP(epn);

    if (on) {
        priv->epcfg.dalepena |= bit;
        sys_set_bits(base + UDC_DWC3_DALEPENA, bit);
    } else {
        priv->epcfg.dalepena &= ~bit;
        sys_clear_bits(base + UDC_DWC3_DALEPENA, bit);
    }
}

/*
 * udc_is_enabled(), as one load: atomic_test_bit() is an out-of-line atomic_get()
 * with CONFIG_ATOMIC_OPERATIONS_C. Exact for a caller holding the UDC mutex:
 * udc_enable() and udc_disable() change the bit only under it, and an aligned
 * word load cannot tear.
 */
static inline bool udc_dwc3_stack_enabled(const struct device *const dev)
{
    const struct udc_data *const data = dev->data;

    return (data->status & BIT(UDC_STATUS_ENABLED)) != 0;
}

/* The endpoint is enabled in DALEPENA, read from the driver's copy. */
static inline bool udc_dwc3_ep_in_dalepena(const struct udc_dwc3_data *const priv,
                       const struct udc_dwc3_ep_data *const ep_data)
{
    return (priv->epcfg.dalepena & UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn)) != 0U;
}

/*
 * A transfer or control state changed: wake the controller recovery if it is
 * waiting for the device to quiesce.
 */
static inline void udc_dwc3_quiesce_progress(struct udc_dwc3_data *const priv)
{
    if (priv->run.state == UDC_DWC3_RUN_STOPPING) {
        k_sem_give(&priv->run.quiesce_sem);
    }
}

/*
 * No transfer may start: the transfers are being ended (4.1.8) or the core is
 * being re-initialised.
 */
static inline bool udc_dwc3_run_halting(const struct udc_dwc3_data *const priv)
{
    return priv->run.state == UDC_DWC3_RUN_STOPPING ||
           priv->run.state == UDC_DWC3_RUN_RESETTING;
}

/*
 * The only writer of ctrl.state; wakes a controller recovery waiting for the
 * control endpoint to return to Setup.
 */
static void udc_dwc3_ctrl_state_set(const struct device *const dev,
                    const enum udc_dwc3_ctrl_state next)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    priv->ctrl.state = next;
    udc_dwc3_quiesce_progress(priv);
}

#ifdef CONFIG_UDC_DWC3_SHELL
static void udc_dwc3_init_fifo_space(const struct device *dev);
#endif

/*
 * Shut down the controller completely. A control endpoint already disabled is
 * done, not a failure: a controller recovery that failed after its own shutdown
 * leaves them so, and refusing here would fail every later shutdown.
 */
static int udc_dwc3_shutdown(const struct device *const dev)
{
    static const uint8_t ctrl_eps[] = {USB_CONTROL_EP_OUT, USB_CONTROL_EP_IN};

    for (size_t i = 0; i < ARRAY_SIZE(ctrl_eps); i++) {
        if (udc_get_ep_cfg(dev, ctrl_eps[i])->stat.enabled &&
            udc_ep_disable_internal(dev, ctrl_eps[i]) != 0) {
            LOG_ERR("Failed to disable control endpoint 0x%02x", ctrl_eps[i]);
            return -EIO;
        }
    }

    return 0;
}


/*
 * Write DCTL with ULSTCHNGREQ = 0. Every DCTL write except the link request in
 * udc_dwc3_dctl_link_request() goes through here, because any value in that
 * write-only field is a link state request; the databook requires 0 when not
 * requesting a link change (DCTL, 3.30b).
 */
static UDC_DWC3_COLD void udc_dwc3_dctl_write(const mm_reg_t base, const uint32_t value)
{
    sys_write32(value & ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK, base + UDC_DWC3_DCTL);
}

static UDC_DWC3_COLD void udc_dwc3_dctl_update(const mm_reg_t base, const uint32_t clr,
                         const uint32_t set)
{
    udc_dwc3_dctl_write(base, (sys_read32(base + UDC_DWC3_DCTL) & ~clr) | set);
}

/*
 * Issue a link state request. Writes 0 first, then the request: the databook
 * requires a 0 between back-to-back identical requests.
 */
static UDC_DWC3_COLD void udc_dwc3_dctl_link_request(const mm_reg_t base, const uint32_t req)
{
    const uint32_t v = sys_read32(base + UDC_DWC3_DCTL) & ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;

    sys_write32(v, base + UDC_DWC3_DCTL);
    sys_write32(v | (req & UDC_DWC3_DCTL_ULSTCHNGREQ_MASK), base + UDC_DWC3_DCTL);
}


/*
 * Why a buffer goes back to the stack, and its status (Zephyr's udc_common.c
 * convention):
 *   DONE       0             the transfer completed
 *   ABANDONED  -ECONNRESET   a control data/status stage made obsolete by the host
 *                            starting a new SETUP (as udc_setup_received())
 *   CANCELLED  -ECONNABORTED a queued request taken back by the driver - dequeue,
 *                            disable, reset, recovery (as udc_ep_cancel_queued())
 *   REFUSED    -EINVAL       a buffer this driver cannot use
 * Classes act on the difference: a cancelled request is dropped quietly, any
 * other error is reported as a failed transfer.
 */
enum udc_dwc3_buf_end {
    UDC_DWC3_BUF_DONE,
    UDC_DWC3_BUF_ABANDONED,
    UDC_DWC3_BUF_CANCELLED,
    UDC_DWC3_BUF_REFUSED,
};

static inline int udc_dwc3_buf_return(const struct device *const dev, struct net_buf *const buf,
                      const enum udc_dwc3_buf_end end)
{
    static const int status[] = {
        [UDC_DWC3_BUF_DONE] = 0,
        [UDC_DWC3_BUF_ABANDONED] = -ECONNRESET,
        [UDC_DWC3_BUF_CANCELLED] = -ECONNABORTED,
        [UDC_DWC3_BUF_REFUSED] = -EINVAL,
    };

    return udc_submit_ep_event(dev, buf, status[end]);
}


/*
 * Negotiated speed: the only decode of DSTS.ConnectSpd. Valid from Connect Done
 * on (4.1.3).
 */
static inline enum udc_bus_speed udc_dwc3_connect_speed(const mm_reg_t base)
{
    switch (sys_read32(base + UDC_DWC3_DSTS) & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
    case UDC_DWC3_DSTS_CONNECTSPD_FS:
        return UDC_BUS_SPEED_FS;
    case UDC_DWC3_DSTS_CONNECTSPD_HS:
        return UDC_BUS_SPEED_HS;
    case UDC_DWC3_DSTS_CONNECTSPD_SS:
        return UDC_BUS_SPEED_SS;
    default:
        return UDC_BUS_UNKNOWN;
    }
}


/*
 * Turn the event interrupt on/off: GEVNTSIZ.EvntIntMask plus the CPU line.
 * Masked, the controller still writes events but stops interrupting.
 * The ISR turns it off until the drain has consumed the ring; the drain, enable
 * and disable set it. IRQ-locked because the ISR writes the same register.
 */
static inline void udc_dwc3_evt_irq(const struct device *const dev, const bool on)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const unsigned int key = irq_lock();

    if (on) {
        sys_clear_bits(base + UDC_DWC3_GEVNTSIZ(0), UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
    } else {
        sys_set_bits(base + UDC_DWC3_GEVNTSIZ(0), UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
    }
    irq_unlock(key);

    if (on) {
        cfg->irq_enable_func();
    } else {
        cfg->irq_disable_func();
    }
}

/*
 * Shut the event drain out before the event ring and the drain state are
 * re-initialised (udc_dwc3_on_soft_reset(), one act). RESETTING first: a drain
 * pass checks it on entry, which covers a drain thread already woken but not yet
 * run (in the cooperative builds it runs only when the caller next sleeps). Then
 * every source of a new pass is removed: the event interrupt, the heartbeat's
 * kick, a queued nudge and a pending wake. No pass can be suspended inside the
 * ring code: a pass never sleeps or blocks before its write-back. Returns the
 * state to restore; the event interrupt and the heartbeat come back with
 * udc_dwc3_enable().
 */
static UDC_DWC3_COLD enum udc_dwc3_run_state udc_dwc3_evt_block(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const enum udc_dwc3_run_state prev = priv->run.state;

    priv->run.state = UDC_DWC3_RUN_RESETTING;
    udc_dwc3_evt_irq(dev, false);
    k_timer_stop(&priv->heartbeat_timer);
    (void)k_work_cancel(&priv->nudge_work);
    k_sem_reset(&priv->evt_sem);

    return prev;
}

/* UDC API lock/unlock: thin wrappers over the framework mutex. */
static void udc_dwc3_lock(const struct device *const dev)
{
    udc_lock_internal(dev, K_FOREVER);
}

/* See udc_dwc3_lock(). */
static void udc_dwc3_unlock(const struct device *const dev)
{
    udc_unlock_internal(dev);
}

/*
 * Commands
 *
 * DEPCMD takes a command number plus parameters; the controller clears CMDACT
 * when the command completes.
 */

/*
 * CSftRst completion wait: bounded CSR reads, then sleeping ticks.
 * udc_dwc3_on_soft_reset() runs on the work queue under the mutex, so it must
 * not busy-wait for the whole budget.
 */
#define UDC_DWC3_CSFTRST_FAST_READS 256u
#define UDC_DWC3_CSFTRST_SLOW_TICKS 10u
/*
 * Endpoint-command wait: number of CSR reads watching CmdAct clear. CmdAct still
 * set means the command is still executing, not that it failed.
 */
#define UDC_DWC3_CMD_FAST_POLLS     32u

/*
 * How long a posted command may keep CmdAct set before its outcome is
 * undetermined (STARTING -> START_UNKNOWN, ENDING -> END_UNKNOWN). A slow
 * command retires well inside this. The heartbeat checks it from cmd_t0;
 * nothing waits on it.
 */
#define UDC_DWC3_CMD_UNKNOWN_MS     100u

/* Defined with the other event-name decoders; used here for timeout diagnostics. */
static const char *udc_dwc3_get_devt_ulstchng_name(const uint32_t dsts);

/*
 * Physical endpoint number for a DEPCMD register address (inverse of
 * UDC_DWC3_DEPCMD(n)). Returns UDC_DWC3_MAX_EPN if addr is not a DEPCMD
 * register; the depcmd_last bookkeeping tests for that.
 */
static inline uint32_t udc_dwc3_depcmd_epn(const uint32_t addr)
{
    if (addr < UDC_DWC3_DEPCMD(0) ||
        addr > UDC_DWC3_DEPCMD(UDC_DWC3_MAX_EPN - 1U) ||
        ((addr - UDC_DWC3_DEPCMD(0)) % 16u) != 0u) {
        return UDC_DWC3_MAX_EPN;
    }

    return (addr - UDC_DWC3_DEPCMD(0)) / 16u;
}

/*
 * Wait for the register file to come back after a reset.
 *
 * GHWPARAMS0.MDWIDTH and GHWPARAMS7.RAM1_DEPTH are fixed at synthesis, so zero
 * in either means the reset is not finished. Both feed the FIFO map; building it
 * from zeros gives every FIFO zero depth and silently kills the controller.
 *
 * Needs two consecutive non-zero reads (one can catch a mid-transition value).
 * Returns false if the budget expires with either still zero.
 */
static bool udc_dwc3_wait_regfile_ready(const struct device *const dev)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

    /*
     * Always settle first: a reset that has not started yet still reads the
     * previous (non-zero) values, which would pass every test below.
     */
    k_sleep(K_MSEC(UDC_DWC3_CORE_SETTLE_MS));

    for (int i = 0; i < UDC_DWC3_CORE_READY_POLLS; i++) {
        bool ok = true;

        for (int j = 0; j < 2; j++) {
            if (FIELD_GET(UDC_DWC3_GHWPARAMS7_RAM1_DEPTH_MASK,
                      sys_read32(base + UDC_DWC3_GHWPARAMS7)) == 0U ||
                (((sys_read32(base + UDC_DWC3_GHWPARAMS0) >> 8) & 0xFFU) == 0U)) {
                ok = false;
                break;
            }
        }

        if (ok) {
            return true;
        }

        k_sleep(K_MSEC(1));
    }

    LOG_ERR("register file still reads zero %u ms after reset "
        "(GHWPARAMS0=0x%08x GHWPARAMS7=0x%08x)",
        UDC_DWC3_CORE_SETTLE_MS + UDC_DWC3_CORE_READY_POLLS,
        sys_read32(base + UDC_DWC3_GHWPARAMS0),
        sys_read32(base + UDC_DWC3_GHWPARAMS7));

    return false;
}

/*
 * Poll DEPCMD.CmdAct until it clears. Returns false if still set when the budget
 * expires. *reg_out (required) gets the last value read.
 */
static bool udc_dwc3_wait_cmdact_zero(const struct device *const dev,
                      const uint32_t addr, uint32_t *const reg_out)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    uint32_t reg = 0;

    /*
     * Bounded read loop, no yield. k_yield() does not help: on the drain thread
     * (UDC_DWC3_EVT_THREAD_PRIO) it returns at once and the loop still spins
     * with the UDC mutex held; on the work queue it hands the CPU to the drain
     * thread, which needs this mutex. Every caller handles false, so giving up
     * after a few dozen reads is cheap.
     */
    for (uint32_t i = 0; i < UDC_DWC3_CMD_FAST_POLLS; i++) {
        reg = sys_read32(base + addr);
        if ((reg & UDC_DWC3_DEPCMD_CMDACT) == 0) {
            *reg_out = reg;
            return true;
        }
    }

    /*
     * No sleep fallback. Every DEPCMD is issued with the UDC mutex held, and
     * udc_dwc3_handle_event() takes that mutex for every event, so sleeping here
     * would stall the event drain and recovery on a core that stopped answering.
     */
    *reg_out = reg;

    return false;
}



/*
 * Make a descriptor visible to the controller before the command that fetches it.
 */
static inline void udc_dwc3_trb_sync(volatile uint32_t *const last_word)
{
    /* The fence below is the ordering; no read-back is needed. */
    ARG_UNUSED(last_word);

#if defined(CONFIG_RISCV)
    __asm__ volatile ("fence iorw,iorw" ::: "memory");
#else
    barrier_dsync_fence_full();
#endif
}

/* Log only the first few stomps; this can fire in a tight loop. */
#define UDC_DWC3_TRB_STOMP_LOG_FIRST    8u

/* F6: single controller instance, asserted. */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
         "udc_dwc3 is single-instance: udc_dwc3_stomp_priv is file-scope and "
         "the last initialised controller would own every stomp report");

static struct udc_dwc3_data *udc_dwc3_stomp_priv;

/*
 * Report a TRB that was overwritten while the controller still owned it. Takes
 * the words the caller already read; reads nothing from the TRB itself.
 */
static inline void udc_dwc3_trb_stomp_report(const uint32_t ctrl, const uint32_t status)
{
    struct udc_dwc3_data *const priv = udc_dwc3_stomp_priv;

    if (priv == NULL) {
        return;
    }

    priv->diag.trb_stomp++;

    if (priv->diag.trb_stomp <= UDC_DWC3_TRB_STOMP_LOG_FIRST) {
        LOG_ERR("overwriting a descriptor the controller still owns: "
            "ctrl 0x%08x sts 0x%08x (stomp %u)",
            ctrl, status, priv->diag.trb_stomp);
    }
}

/*
 * The only writer of TRB words. The values are prepared by the caller; the
 * words go out in a fixed order with ctrl (HWO) last, then the fence. A 16-byte
 * TRB cannot be stored in one access on this CPU, and a struct assignment to a
 * volatile TRB leaves the word order to the compiler, so this order is what
 * keeps the controller from seeing HWO before the rest of the TRB.
 */
static inline void udc_dwc3_trb_write(volatile struct udc_dwc3_trb *const trb,
                      const uintptr_t addr, const uint32_t status,
                      const uint32_t ctrl)
{
    trb->addr_lo = LO32(addr);
    trb->addr_hi = HI32(addr);
    trb->status = status;
    trb->ctrl = ctrl;

    udc_dwc3_trb_sync(&trb->ctrl);
}

/*
 * Arm a TRB: udc_dwc3_trb_write(), reporting (not preventing) an overwrite of a
 * TRB the controller still owns. For clearing a TRB the controller has already
 * released, which may still read HWO=1, use udc_dwc3_trb_write() directly.
 */
static inline void udc_dwc3_trb_fill(volatile struct udc_dwc3_trb *const trb,
                     const uintptr_t addr, const uint32_t status,
                     const uint32_t ctrl)
{
    const uint32_t old_ctrl = trb->ctrl;

    if ((old_ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
        udc_dwc3_trb_stomp_report(old_ctrl, trb->status);
    }

    udc_dwc3_trb_write(trb, addr, status, ctrl);
}

/*
 * Snapshot a TRB the controller may have written back: ctrl, then status, each
 * read once. The controller writes status and clears HWO in its write-back, so
 * the status is the written-back one only if ctrl, read before it, has HWO
 * clear. Returns that: true means out->status is the write-back; false means
 * the controller still owns the TRB and out->status may be stale. Not for a
 * poll: each check takes a new snapshot.
 */
static inline bool udc_dwc3_trb_snapshot(const volatile struct udc_dwc3_trb *const t,
                     struct udc_dwc3_trb *const out)
{
    out->ctrl = t->ctrl;
    out->status = t->status;

    return (out->ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U;
}

/* Record a transfer resource index. Defined below. */
static void udc_dwc3_store_xferrscidx(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      uint32_t idx);

/* Name of an enum udc_dwc3_ep_state value, for logging. */
static UDC_DWC3_COLD const char *udc_dwc3_ep_state_name(const enum udc_dwc3_ep_state st)
{
    switch (st) {
    case UDC_DWC3_EP_IDLE:       return "idle";
    case UDC_DWC3_EP_STARTING:   return "starting";
    case UDC_DWC3_EP_START_UNKNOWN:  return "start-unknown";
    case UDC_DWC3_EP_RUNNING:    return "running";
    case UDC_DWC3_EP_ENDING:     return "ending";
    case UDC_DWC3_EP_END_UNKNOWN:    return "end-unknown";
    default:             return "?";
    }
}


/*
 * Recompute cfg.stat.busy (what udc_ep_is_busy() reports):
 *   - EP0 halves: busy while a transfer is in progress (xfer.state != IDLE).
 *   - Other endpoints: busy while the ring holds TRBs not yet retired.
 * Called only where those facts change: the xfer.state writers
 * (udc_dwc3_ep_state_set()/_reset()) and the ring writers (udc_dwc3_push_trb(),
 * udc_dwc3_pop_trb(), udc_dwc3_ep_ring_release()). Written only when it changes:
 * on a streaming ring most arms and retires leave it set.
 */
static inline void udc_dwc3_ep_busy_sync(struct udc_dwc3_ep_data *const ep_data)
{
    const bool busy = (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U)
                  ? (ep_data->xfer.state != UDC_DWC3_EP_IDLE)
                  : (ep_data->ring.net_buf[ep_data->ring.tail] != NULL);

    if ((bool)ep_data->cfg.stat.busy != busy) {
        udc_ep_set_busy(&ep_data->cfg, busy);
    }
}

/*
 * After an xfer.state change (both writers): update cfg.stat.busy and wake a
 * controller recovery waiting for transfers to end.
 */
static inline void udc_dwc3_ep_state_derive(struct udc_dwc3_ep_data *const ep_data)
{
    udc_dwc3_ep_busy_sync(ep_data);
    udc_dwc3_quiesce_progress(udc_get_private(ep_data->dev));
}

/*
 * Change xfer.state along a legal transition. Returns false (and logs) for an
 * illegal one. The only other writer is udc_dwc3_ep_state_reset(), which
 * returns to IDLE from anywhere; every return to IDLE goes through it, so the
 * table has no row ending in IDLE.
 */
static bool udc_dwc3_ep_state_set(struct udc_dwc3_ep_data *const ep_data,
                  const enum udc_dwc3_ep_state next)
{
    static const uint8_t legal[][2] = {
        { UDC_DWC3_EP_IDLE,     UDC_DWC3_EP_STARTING },
        { UDC_DWC3_EP_STARTING,     UDC_DWC3_EP_RUNNING },
        /*
         * Deadline passed with the command still executing: outcome
         * undetermined, neither success nor rejection.
         */
        { UDC_DWC3_EP_STARTING,     UDC_DWC3_EP_START_UNKNOWN },
        { UDC_DWC3_EP_ENDING,       UDC_DWC3_EP_END_UNKNOWN },
        /*
         * UNKNOWN is left only on proof: DEPCMD (or cmd_record, if a later
         * command overwrote it) shows the same command type with CmdAct
         * clear. The Start/End refusals guarantee no second Start or End
         * replaces it. Without proof, and for a Start proven refused or an
         * End proven complete, the exit is udc_dwc3_ep_state_reset().
         */
        { UDC_DWC3_EP_START_UNKNOWN,    UDC_DWC3_EP_RUNNING },
        /*
         * Same proof, showing the End was refused: the transfer still runs
         * and owns its resource. udc_dwc3_ep_end_refused() restores the
         * index first.
         */
        { UDC_DWC3_EP_END_UNKNOWN,  UDC_DWC3_EP_RUNNING },
        { UDC_DWC3_EP_RUNNING,      UDC_DWC3_EP_ENDING },
        /*
         * The End Transfer did not end the transfer: either never issued
         * (pre-poll gave up on a still-active command) or refused.
         */
        { UDC_DWC3_EP_ENDING,       UDC_DWC3_EP_RUNNING },
    };

    if (ep_data->xfer.state == next) {
        return true;
    }

    for (size_t i = 0; i < ARRAY_SIZE(legal); i++) {
        if (legal[i][0] == ep_data->xfer.state && legal[i][1] == next) {
            LOG_DBG("EP%02x xfer %s -> %s", ep_data->cfg.addr,
                udc_dwc3_ep_state_name(ep_data->xfer.state),
                udc_dwc3_ep_state_name(next));
            ep_data->xfer.state = next;
            udc_dwc3_ep_state_derive(ep_data);
            return true;
        }
    }

    LOG_ERR("EP%02x ILLEGAL transfer state change %s -> %s, refused",
        ep_data->cfg.addr, udc_dwc3_ep_state_name(ep_data->xfer.state),
        udc_dwc3_ep_state_name(next));

    return false;
}



/*
 * An End Transfer is outstanding (executing or outcome unknown). Either way the
 * controller may still own the ring.
 */
static inline bool udc_dwc3_ep_is_ending(const struct udc_dwc3_ep_data *const ep_data)
{
    return ep_data->xfer.state == UDC_DWC3_EP_ENDING ||
           ep_data->xfer.state == UDC_DWC3_EP_END_UNKNOWN;
}

/* A command outcome this driver could not determine. */
static inline bool udc_dwc3_ep_is_unknown(const struct udc_dwc3_ep_data *const ep_data)
{
    return ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN ||
           ep_data->xfer.state == UDC_DWC3_EP_END_UNKNOWN;
}

/*
 * A command posted on this endpoint has not resolved, so nothing else may act on
 * it. xfer.state alone decides this.
 */
static inline bool udc_dwc3_ep_cmd_busy(const struct udc_dwc3_ep_data *const ep_data)
{
    return ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
           udc_dwc3_ep_is_ending(ep_data) ||
           udc_dwc3_ep_is_unknown(ep_data);
}

/*
 * Return to IDLE from any state, for paths that know the controller holds no
 * transfer on this endpoint: teardown (bus reset, disconnect, disable,
 * DEPSTARTCFG, controller recovery), a completed stage or ring, a concluded End,
 * and a Start proven refused. So this bypasses the transition table.
 */
static void udc_dwc3_ep_state_reset(struct udc_dwc3_ep_data *const ep_data)
{
    if (ep_data->xfer.state != UDC_DWC3_EP_IDLE) {
        LOG_DBG("EP%02x xfer %s -> idle (reset)", ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state));
    }

    ep_data->xfer.state = UDC_DWC3_EP_IDLE;
    udc_dwc3_ep_state_derive(ep_data);
    ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
    ep_data->xfer.end_idx = UDC_DWC3_XFERRSCIDX_INVALID;
    ep_data->cmd.cmd_record = 0U;

    /*
     * An owed Update belongs to the transfer and ends with it. The other
     * pending bits are owed to the stack or the host, not to a command, and
     * survive: udc_dwc3_ep_recover() carries them out once the endpoint is
     * idle. Teardown withdraws them explicitly (udc_dwc3_ep_disable(),
     * udc_dwc3_drop_xfer_state()).
     */
    ep_data->xfer.pending &= (uint8_t)~UDC_DWC3_EP_PEND_UPDATE;
}

/* Adopt a transfer resource index from DEPCMD. Defined below. */
static void udc_dwc3_adopt_xferrscidx(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const uint32_t reg);

/* Settle an endpoint with a command outstanding. Defined below. */
static void udc_dwc3_ep_resolve_cmd(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data);

/* The End Transfer has concluded; run what it owed. Defined below. */
static void udc_dwc3_ep_end_completed(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data);

/*
 * Result of a posted endpoint command. Three outcomes: CmdAct still set means
 * still executing, which is neither success nor rejection; treating it as either
 * can leak a transfer resource the controller really took.
 * CMD_OTHER is not an outcome: DEPCMD holds a different command.
 */
enum udc_dwc3_cmd_outcome {
    UDC_DWC3_CMD_UNKNOWN = 0,   /* CmdAct set: still executing          */
    UDC_DWC3_CMD_OK,        /* CmdAct clear, CmdStatus OK           */
    UDC_DWC3_CMD_ERROR,     /* CmdAct clear, CmdStatus not OK       */
    UDC_DWC3_CMD_OTHER,     /* DEPCMD holds another command type    */
};

/*
 * Classify DEPCMD as the outcome of this endpoint's command of type cmdtyp.
 * The only place this rule is written. *reg_out (optional) gets the value used.
 *
 * - CmdAct clear means finished, not failed; CmdStatus says which. Testing
 *   CmdAct alone reads a late success as a refusal.
 * - CmdStatus needs the command type too: DEPCMD holds the last command posted,
 *   which may be a later one (e.g. a Set Stall) or an earlier one (our
 *   pre-poll gave up without posting). Reading its status as ours can, e.g.,
 *   release a ring under a live transfer after a refused End Transfer.
 * - If a later command overwrote ours, udc_dwc3_depcmd() saved the old value in
 *   cmd_record, which answers instead.
 * - CMD_OTHER: neither holds it. Right after posting, it means the pre-poll gave
 *   up and nothing was posted.
 */
static enum udc_dwc3_cmd_outcome
udc_dwc3_cmd_outcome(const struct device *const dev,
             const struct udc_dwc3_ep_data *const ep_data,
             const uint32_t cmdtyp,
             uint32_t *const reg_out)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    uint32_t reg = sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn));

    /*
     * Overwritten: answer from cmd_record if it holds the command asked
     * after. The record is only taken with CmdAct clear.
     */
    if ((reg & UDC_DWC3_DEPCMD_CMDTYP_MASK) != cmdtyp) {
        if ((ep_data->cmd.cmd_record & UDC_DWC3_DEPCMD_CMDTYP_MASK) != cmdtyp) {
            if (reg_out != NULL) {
                *reg_out = reg;
            }
            return UDC_DWC3_CMD_OTHER;
        }
        reg = ep_data->cmd.cmd_record;
    }

    if (reg_out != NULL) {
        *reg_out = reg;
    }

    if ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0U) {
        return UDC_DWC3_CMD_UNKNOWN;
    }

    return ((reg & UDC_DWC3_DEPCMD_STATUS_MASK) == UDC_DWC3_DEPCMD_STATUS_OK)
               ? UDC_DWC3_CMD_OK : UDC_DWC3_CMD_ERROR;
}

/*
 * Operands (DEPCMDPAR0/1/2) of one endpoint command.
 *
 * The databook does not say they are latched when CmdAct is written, so they
 * are live while CmdAct is set: never write them over a command still executing
 * on this endpoint. That is why they are passed to udc_dwc3_depcmd() and written
 * after its pre-poll, just before CmdAct; a call-site write would land on the
 * previous, still-running command.
 *
 * A command with operands writes all three, unused ones zero (as the reference
 * driver does). A command without operands passes NULL.
 */
struct udc_dwc3_depcmd_par {
    uint32_t    par0;
    uint32_t    par1;
    uint32_t    par2;
};

/*
 * Issue one endpoint command and collect its result. par holds the operands, or
 * NULL if the command has none.
 *
 * Returns 0 on completion with status OK, else UDC_DWC3_XFERRSCIDX_INVALID
 * (rejected, not issued, or still executing when the poll expired). Callers that
 * must tell these apart use udc_dwc3_cmd_outcome(). A Start Transfer that is not
 * issued returns UDC_DWC3_DEPCMD_NOT_POSTED instead, so its caller knows for
 * certain that nothing was posted.
 */
static uint32_t udc_dwc3_depcmd(const struct device *const dev,
                const uint32_t addr, const uint32_t cmd,
                const struct udc_dwc3_depcmd_par *const par)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const struct udc_dwc3_config *const cfg = DEV_CFG(dev);
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const uint32_t epn = udc_dwc3_depcmd_epn(addr);
    /*
     * Target endpoint, or NULL if addr is not an endpoint command register.
     * Command bookkeeping lives on the endpoint, so an endpoint reset clears it.
     */
    struct udc_dwc3_ep_data *const ep = _EPN_IS_VALID(cfg, epn)
                         ? _EP_DATA_FROM_EPN(cfg, epn) : NULL;
    const bool first_on_ep = (ep == NULL) || ep->cmd.depcmd_last == UDC_DWC3_DEPCMD_NONE;
    /* A Start or End Transfer: the commands whose outcome is tracked. */
    const bool opens = ((cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK) ==
                UDC_DWC3_DEPCMD_DEPSTRTXFER) ||
               ((cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK) ==
                UDC_DWC3_DEPCMD_DEPENDXFER);
    uint32_t reg = 0;

    /*
     * A new Start or End clears the saved record before the pre-poll: if the
     * pre-poll gives up, the caller's outcome read must not find an older
     * command of the same type.
     */
    if (ep != NULL && opens) {
        ep->cmd.cmd_record = 0U;
    }

    /*
     * Never write a command over one still running: CmdAct is R/W1S and the
     * result is undefined (dropped, doubled, or using the previous operands).
     * Exception: the first command on an endpoint, where DEPCMD reads undefined
     * and CmdAct may read set (1.3.12), so issuing is safe.
     */
    if (!first_on_ep && !udc_dwc3_wait_cmdact_zero(dev, addr, &reg)) {
        LOG_ERR("previous command still active on addr 0x%x (0x%08x) after the "
            "bounded poll, not issuing command 0x%x, GEVNTCOUNT=%u bytes, "
            "DSTS=0x%08x (%s)",
            addr, reg, cmd,
            priv->evt.gc_last,
            sys_read32(base + UDC_DWC3_DSTS),
            udc_dwc3_get_devt_ulstchng_name(sys_read32(base + UDC_DWC3_DSTS)));
        return ((cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK) == UDC_DWC3_DEPCMD_DEPSTRTXFER)
               ? UDC_DWC3_DEPCMD_NOT_POSTED : UDC_DWC3_XFERRSCIDX_INVALID;
    }

    /*
     * DEPCMD is about to be overwritten: if it holds the outcome of the
     * endpoint's open Start or End, save it in cmd_record (the resolver would
     * otherwise find this new command). Checked before the adoption below,
     * which may close the Start.
     */
    if (!first_on_ep && !opens && udc_dwc3_ep_cmd_busy(ep)) {
        const uint32_t typ = reg & UDC_DWC3_DEPCMD_CMDTYP_MASK;

        if (typ == UDC_DWC3_DEPCMD_DEPSTRTXFER ||
            typ == UDC_DWC3_DEPCMD_DEPENDXFER) {
            ep->cmd.cmd_record = reg;
        }
    }

    /*
     * The pre-poll value may complete this endpoint's open Start. Not for a
     * Start or End being posted: the caller has already set STARTING or ENDING,
     * and DEPCMD holds an earlier command, so a Start found there belongs to a
     * previous transfer and its index is not this one's.
     */
    if (!first_on_ep && !opens) {
        udc_dwc3_adopt_xferrscidx(dev, ep, reg);
    }

    if (!first_on_ep &&
        (reg & UDC_DWC3_DEPCMD_STATUS_MASK) == UDC_DWC3_DEPCMD_STATUS_CMDERR &&
        !ep->diag.cmd_reported) {
        LOG_ERR("previous endpoint command on addr 0x%x reported an error "
            "(0x%08x): command 0x%08x, type 0x%x", addr, reg,
            ep->cmd.depcmd_last,
            (unsigned int)(ep->cmd.depcmd_last & UDC_DWC3_DEPCMD_CMDTYP_MASK));
    }

    /*
     * SusPHY and EnblSlpM must be clear before an endpoint command (GUSB2PHYCFG
     * notes). This driver never sets them; udc_dwc3_on_soft_reset() clears them
     * once, and the register survives a core soft reset.
     */


    /*
     * The transfer resource index lives and dies with Start/End Transfer
     * (3.2.2.2), so it is invalidated here, where every command is issued,
     * rather than at the call sites.
     */
    {
        const uint32_t cmdtyp = cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK;

        if (ep != NULL &&
            (cmdtyp == UDC_DWC3_DEPCMD_DEPSTRTXFER ||
             cmdtyp == UDC_DWC3_DEPCMD_DEPENDXFER)) {
            ep->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
        }
    }

    /*
     * Operands, then the command, both after the pre-poll, so operands never
     * land on a still-executing command. The give-up path above touches nothing.
     */
    if (par != NULL && epn < UDC_DWC3_MAX_EPN) {
        sys_write32(par->par0, base + UDC_DWC3_DEPCMDPAR0(epn));
        sys_write32(par->par1, base + UDC_DWC3_DEPCMDPAR1(epn));
        sys_write32(par->par2, base + UDC_DWC3_DEPCMDPAR2(epn));
    }

    sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);

    /*
     * DEPCMD is now written, so its read value is defined and the next command
     * on this endpoint may pre-poll it.
     */
    if (ep != NULL) {
        ep->cmd.depcmd_last = cmd;
        ep->diag.cmd_reported = false;
        /*
         * Start of the UDC_DWC3_CMD_UNKNOWN_MS deadline, which only a Start or
         * End has: a later Set Stall or DEPCFG must not extend it.
         */
        if (opens) {
            ep->cmd.cmd_t0 = k_cycle_get_32();
        }
    }

    /*
     * Poll every command, not just Start Transfer: Update Transfer has CmdIOC=0
     * and raises no Command Complete, so its CmdStatus is read only here.
     * Without the post-poll (UDC_DWC3_DEPCMD_POST_POLL), Start and Update
     * return POSTED here; see the #define for where their outcome is learnt.
     */
    if (ep != NULL) {
        const uint32_t cmdtyp = cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK;
        uint32_t done = 0;
        bool finished;

        if (!UDC_DWC3_DEPCMD_POST_POLL &&
            (cmdtyp == UDC_DWC3_DEPCMD_DEPSTRTXFER || cmdtyp == UDC_DWC3_DEPCMD_DEPUPDXFER)) {
            return UDC_DWC3_DEPCMD_POSTED;
        }

        finished = udc_dwc3_wait_cmdact_zero(dev, addr, &done);

        /*
         * Read once more before reporting. The LOG_WRN below takes ~11 ms
         * under CONFIG_LOG_MODE_MINIMAL (~130 chars at ~87 us/char, mutex
         * held). A command retiring in that window would make the caller's
         * CmdAct re-read see clear and treat a success as a rejection,
         * resetting the endpoint and orphaning its transfer resource.
         * CmdStatus is valid as soon as CmdAct clears.
         */
        if (!finished) {
            done = sys_read32(base + addr);
            finished = (done & UDC_DWC3_DEPCMD_CMDACT) == 0U;
        }

        if (!finished) {
            priv->diag.depcmd_n_timeout++;
            /* Not success yet: callers resolve it with udc_dwc3_cmd_outcome(). */
            LOG_WRN("EP%02x command 0x%x on addr 0x%x still executing past the "
                "poll (0x%08x), outcome pending (%u so far)",
                ep->cfg.addr, cmd, addr, done, priv->diag.depcmd_n_timeout);
            return UDC_DWC3_XFERRSCIDX_INVALID;
        }

        /*
         * Only Start Transfer assigns a transfer resource; adopt checks the
         * value before trusting it.
         */
        if (cmdtyp == UDC_DWC3_DEPCMD_DEPSTRTXFER) {
            udc_dwc3_adopt_xferrscidx(dev, ep, done);
        }

        if ((done & UDC_DWC3_DEPCMD_STATUS_MASK) !=
            UDC_DWC3_DEPCMD_STATUS_OK) {
            priv->diag.depcmd_n_err++;

            /* Always logged: the controller refused an issued command. */
            LOG_ERR("EP%02x command type 0x%x REJECTED: DEPCMD=0x%08x "
                "status=0x%x (%u errors so far)",
                ep->cfg.addr,
                (unsigned int)(cmdtyp >> 0),
                done,
                (unsigned int)((done & UDC_DWC3_DEPCMD_STATUS_MASK) >> 12),
                priv->diag.depcmd_n_err);

            ep->diag.cmd_reported = true;

            return UDC_DWC3_XFERRSCIDX_INVALID;
        }

    }

    return 0;
}

/*
 * DEPCFG: program an endpoint's type, packet size, FIFO and interrupt number.
 */
static UDC_DWC3_COLD void udc_dwc3_depcmd_ep_config(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const bool modify)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    uint32_t param0 = 0;
    uint32_t param1 = 0;

    LOG_INF("Configuring endpoint 0x%02x with wMaxPacketSize=%u",
        ep_data->cfg.addr, ep_data->cfg.mps);

    /* Init or Modify is passed in, not inferred from cfg.stat.enabled. */
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
     * BrstSiz = packets per burst - 1. Control endpoints do not burst: 0 for
     * EP0 (Table 4-1).
     */
    if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
        param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 0);
    } else {
        /* Burst of 4, matching bMaxBurst=3 in the class descriptors and
         * DCFG.NUMP=4, as the controller vendor specified. */
        param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 3);
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
     * XferNotReady on EP0 only, where it is mandatory: an integral part of control
     * transfer handling (4.2.4). Non-control endpoints get none.
     */
    if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
        param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
    }

    /* USB endpoint number field; our physical endpoint number uses the same
     * encoding.
     */
    param1 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK, ep_data->epn);

    /*
     * bInterval_m1: bInterval - 1, and 0 at Full-Speed (DEPCFG field
     * description). Mandatory for isochronous endpoints (4.3.3); same meaning
     * for interrupt ones.
     */
    switch (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) {
    case USB_EP_TYPE_ISO:
    case USB_EP_TYPE_INTERRUPT: {
        uint32_t binterval_m1 = 0;

        if (udc_dwc3_connect_speed(base) != UDC_BUS_SPEED_FS) {
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

    {
        const struct udc_dwc3_depcmd_par par = {
            .par0 = param0,
            .par1 = param1,
            .par2 = 0U,
        };

        udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
                UDC_DWC3_DEPCMD_DEPCFG, &par);
    }
}

/*
 * DEPXFERCFG: allocate this endpoint's transfer resource. Endpoint enable only:
 * re-issuing it leaks a resource.
 */
static UDC_DWC3_COLD void udc_dwc3_depcmd_ep_xfer_config(const struct device *const dev,
                       struct udc_dwc3_ep_data *const ep_data)
{
    const struct udc_dwc3_depcmd_par par = {
        .par0 = FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK, 1),
        .par1 = 0U,
        .par2 = 0U,
    };

    LOG_DBG("DepXferConfig: EP%02x", ep_data->cfg.addr);

    udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
            UDC_DWC3_DEPCMD_DEPXFERCFG, &par);
}

/*
 * After udc_dwc3_depcmd() failed: was the command posted and not refused, i.e.
 * still executing (it completes; R2) or completed late with status OK?
 */
static bool udc_dwc3_cmd_posted_ok(const struct device *const dev,
                   const struct udc_dwc3_ep_data *const ep_data,
                   const uint32_t cmdtyp)
{
    const enum udc_dwc3_cmd_outcome out =
        udc_dwc3_cmd_outcome(dev, ep_data, cmdtyp, NULL);

    return out == UDC_DWC3_CMD_UNKNOWN || out == UDC_DWC3_CMD_OK;
}

/*
 * DEPSETSTALL. Returns whether the controller took the command (accepted, or
 * still executing past the poll).
 */
static UDC_DWC3_COLD bool udc_dwc3_depcmd_set_stall(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data)
{
    LOG_DBG("DepSetStall: EP%02x", ep_data->cfg.addr);

    /*
     * udc_dwc3_depcmd() fails both for a refusal and for a command still
     * executing past its poll; the latter was posted and takes effect.
     */
    if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
                UDC_DWC3_DEPCMD_DEPSETSTALL, NULL) != 0U &&
        !udc_dwc3_cmd_posted_ok(dev, ep_data, UDC_DWC3_DEPCMD_DEPSETSTALL)) {
        return false;
    }

    /* Set only here, once the controller has taken the stall. */
    ep_data->cfg.stat.halted = true;

    return true;
}

/*
 * DEPCSTALL. Returns whether the controller took the command (accepted, or
 * still executing past the poll).
 */
static UDC_DWC3_COLD bool udc_dwc3_depcmd_clear_stall(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data,
                    uint32_t flags)
{
    LOG_DBG("DepClearStall EP%02x", ep_data->cfg.addr);

    flags |= UDC_DWC3_DEPCMD_DEPCSTALL;

    /* Same convention as udc_dwc3_depcmd_set_stall(). */
    if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags, NULL) != 0U &&
        !udc_dwc3_cmd_posted_ok(dev, ep_data, UDC_DWC3_DEPCMD_DEPCSTALL)) {
        return false;
    }

    /*
     * Cleared only here, once the controller has taken the command.
     * udc_dwc3_ep_resume() also un-stalls every non-control endpoint it restores.
     * If the flag were cleared anywhere else, a recovery through resume could leave
     * the hardware un-halted with halted still true, and udc_dwc3_ep_worker() would
     * then refuse every buffer.
     */
    ep_data->cfg.stat.halted = false;

    return true;
}

/*
 * Defined below; needed here to release a transfer resource that a rejected
 * Start Transfer could not obtain.
 */
static bool udc_dwc3_depcmd_end_xfer(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data,
                     uint32_t flags);

/* Defined below; udc_dwc3_ep_update_owed() needs it. */
static bool udc_dwc3_ep_ring_outstanding(const struct udc_dwc3_ep_data *const ep_data);

/* Defined below; udc_dwc3_depcmd_start_xfer() needs it for a refusal at post time. */
static void udc_dwc3_ep_start_refused(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const char *const src, const uint32_t val,
                      const bool at_post);

/*
 * Record a transfer resource index the controller returned. Warns if another
 * endpoint already holds the same index.
 */
static void udc_dwc3_store_xferrscidx(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const uint32_t idx)
{
    if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
        const struct udc_dwc3_config *const cfg = DEV_CFG(dev);
        const struct udc_dwc3_ep_data *clash = NULL;

        for (uint8_t i = 0; i < cfg->num_in_eps && clash == NULL; i++) {
            if (&cfg->ep_data_in[i] != ep_data &&
                cfg->ep_data_in[i].xferrscidx == idx) {
                clash = &cfg->ep_data_in[i];
            }
        }
        for (uint8_t i = 0; i < cfg->num_out_eps && clash == NULL; i++) {
            if (&cfg->ep_data_out[i] != ep_data &&
                cfg->ep_data_out[i].xferrscidx == idx) {
                clash = &cfg->ep_data_out[i];
            }
        }

        if (clash != NULL) {
            LOG_WRN("XFERRSCIDX EP%02x = %u, SHARED with EP%02x",
                ep_data->cfg.addr, idx, clash->cfg.addr);
        } else {
            LOG_DBG("XFERRSCIDX EP%02x = %u", ep_data->cfg.addr, idx);
        }
    }

    ep_data->xferrscidx = idx;
}


/*
 * Returns true when the transfer is running.
 */
static bool udc_dwc3_depcmd_start_xfer(const struct device *const dev,
                       struct udc_dwc3_ep_data *const ep_data)
{
    /* Filled in below; issued with the command, not before it. */
    struct udc_dwc3_depcmd_par par;
    uint32_t idx;
    uint32_t cmd;

    /*
     * The controller fetches the TRB as soon as it sees the command, so the TRB
     * must be in memory before the command is posted.
     */

    /*
     * No link-state request here. In U1/U2 the controller serves the Start after
     * its own exit; in U3 (host suspended the device) it waits for the host's
     * resume. Writing ULSTCHNGREQ=Resume here would make every arm in U3 an
     * unsolicited remote wake.
     */

    /*
     * Resolve an open Start first, so the checks below see the controller's real
     * state. Start side only: resolving an End here runs
     * udc_dwc3_ep_end_completed(), which resumes the endpoint and re-enters this
     * function.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
        ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN) {
        udc_dwc3_ep_resolve_cmd(dev, ep_data);
    }

    /*
     * Never post a second Start while the first one's outcome is open. The
     * controller may already hold a transfer resource for it, and a second Start
     * would take another that the driver cannot address or End. Control endpoints
     * are not exempt from this rule.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
        udc_dwc3_ep_is_unknown(ep_data)) {
        LOG_ERR("EP%02x Start Transfer refused: the endpoint is %s, so a "
            "command outcome is still open", ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state));
        return false;
    }

    /*
     * INVARIANT 1: Start Transfer only from IDLE - on every endpoint; EP0 starts
     * each control stage as a new transfer. This also enforces 3.2.2.7: no Start
     * while an End Transfer is still completing on this endpoint.
     */
    if (ep_data->xfer.state != UDC_DWC3_EP_IDLE) {
        LOG_ERR("EP%02x Start Transfer refused: endpoint is %s, not idle",
            ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state));
        return false;
    }

    /*
     * PAR0/PAR1 = address of this transfer's first TRB, trb_buf[tail], not the
     * ring base. After a wrap the base slot has HWO clear, so starting there makes
     * the controller take the transfer resource and never move. EP0 has no ring
     * and always uses slot 0.
     */
    {
        const uint32_t first = (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U)
                           ? 0U : ep_data->ring.tail;
        const uintptr_t trb0 = (uintptr_t)&ep_data->trb_buf[first];

        par.par0 = HI32(trb0);
        par.par1 = LO32(trb0);
        par.par2 = 0U;
    }

    /*
     * The databook returns the transfer resource index "in the DEPCMDn register
     * and in the Command Complete event". CMDIOC (below) requests that event, so
     * the index is not waited for here.
     */
    cmd = UDC_DWC3_DEPCMD_DEPSTRTXFER;

    /*
     * Without CMDIOC the only way to observe the Start is the synchronous poll
     * in udc_dwc3_depcmd(). If that misses, the outcome is unknown and the
     * endpoint stays STARTING until another command runs on it.
     */
    cmd |= UDC_DWC3_DEPCMD_CMDIOC;

    /* Set STARTING before posting, for the same reason End Transfer sets ENDING first. */
    (void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_STARTING);

    idx = udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), cmd, &par);

    /*
     * Not posted: the previous command was still executing when the pre-poll
     * gave up (logged there). Told apart from a posted Start still executing,
     * whose outcome DEPCMD would then report as that older command's. Back to
     * IDLE, as before the call: IDLE holds no index, end_idx, cmd_record or owed
     * Update, so the reset restores exactly that.
     */
    if (idx == UDC_DWC3_DEPCMD_NOT_POSTED) {
        udc_dwc3_ep_state_reset(ep_data);
        return false;
    }

    /*
     * Posted without the post-poll: STARTING until its Command Complete, the
     * next command's pre-poll or the resolver settles it.
     */
    if (idx == UDC_DWC3_DEPCMD_POSTED) {
        return true;
    }

    /*
     * The Start was posted, so failure means still executing past the poll,
     * completed late, or refused.
     */
    if (idx == UDC_DWC3_XFERRSCIDX_INVALID) {
        struct udc_dwc3_data *const priv = udc_get_private(dev);
        uint32_t done = 0U;
        const enum udc_dwc3_cmd_outcome out =
            udc_dwc3_cmd_outcome(dev, ep_data, UDC_DWC3_DEPCMD_DEPSTRTXFER,
                         &done);

        /*
         * Still executing is neither success nor refusal: leave it STARTING for
         * its Command Complete (or the resolver).
         */
        if (out == UDC_DWC3_CMD_UNKNOWN) {
            LOG_WRN("EP%02x Start Transfer still executing past the poll "
                "budget: left STARTING for its Command Complete rather "
                "than resetting the endpoint under a live command",
                ep_data->cfg.addr);
            return true;
        }

        /*
         * The command completed after the poll expired (e.g. during the ~11 ms
         * synchronous log line in udc_dwc3_depcmd()). Adopt the index: resetting
         * to IDLE would abandon a resource the controller just assigned, with no
         * index left to End it.
         */
        if (out == UDC_DWC3_CMD_OK) {
            udc_dwc3_adopt_xferrscidx(dev, ep_data, done);
            return true;
        }

        /*
         * Refused (CMD_OTHER, DEPCMD not holding a Start, cannot follow a posted
         * Start and is handled the same way). No resource was assigned. CmdStatus
         * 4'h1 on Start Transfer means "no transfer resource available on the
         * endpoint"; 3.2.2.2 describes how to get one back. No retry.
         *
         * A non-control endpoint with buffers armed is taken out of service as
         * when the refusal is learnt later (udc_dwc3_ep_start_refused()), so
         * nothing re-issues this Start for an armed ring on an IDLE endpoint.
         * Otherwise (EP0, whose callers recover the control transfer, or an empty
         * ring being built for an enable) the endpoint returns to IDLE and false
         * lets the caller act.
         */
        if (USB_EP_GET_IDX(ep_data->cfg.addr) != 0U &&
            udc_dwc3_ep_ring_outstanding(ep_data)) {
            udc_dwc3_ep_start_refused(dev, ep_data, "DEPCMD", done, true);
            return false;
        }

        priv->diag.ctrl_start_fail++;
        LOG_ERR("EP%02x Start Transfer refused (DEPCMD 0x%08x), endpoint returned "
            "to idle (%u so far)", ep_data->cfg.addr, done,
            priv->diag.ctrl_start_fail);
        udc_dwc3_ep_state_reset(ep_data);

        return false;
    }

    /*
     * udc_dwc3_depcmd() has already invalidated the index. It arrives with the
     * Command Complete event, or the next command's pre-poll reads it back if it
     * is needed before that event is drained.
     */
    LOG_DBG("start EP%02x issued, transfer resource index pending",
        ep_data->cfg.addr);

    return true;
}

/*
 * Adopt the transfer resource index out of a DEPCMD value, if that value is a
 * completed, successful Start Transfer for this endpoint.
 */
static void udc_dwc3_adopt_xferrscidx(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const uint32_t reg)
{
    if (ep_data->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID) {
        return;
    }

    /*
     * Only STARTING and START_UNKNOWN wait for an index. In any other state DEPCMD
     * holds a stale result from a transfer since reset (DEPSTARTCFG, disable or
     * bus reset); adopting it would give a valid xferrscidx to an endpoint with
     * no transfer.
     */
    if (ep_data->xfer.state != UDC_DWC3_EP_STARTING &&
        ep_data->xfer.state != UDC_DWC3_EP_START_UNKNOWN) {
        return;
    }

    if ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0 ||
        (reg & UDC_DWC3_DEPCMD_CMDTYP_MASK) != UDC_DWC3_DEPCMD_DEPSTRTXFER) {
        return;
    }

    if ((reg & UDC_DWC3_DEPCMD_STATUS_MASK) != UDC_DWC3_DEPCMD_STATUS_OK) {
        LOG_ERR("EP%02x Start Transfer reported 0x%08x, keeping transfer "
            "resource index 0x%x", ep_data->cfg.addr, reg,
            ep_data->xferrscidx);
        return;
    }

    udc_dwc3_store_xferrscidx(dev, ep_data,
                  FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg));

    /*
     * The index completes STARTING -> RUNNING. Whichever observer gets here first
     * (post-poll, next command's pre-poll, Command Complete handler) makes the
     * same transition; it is idempotent.
     */
    (void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_RUNNING);
}

/*
 * Adopt a transfer resource index delivered in a Command Complete event.
 */
static void udc_dwc3_adopt_xferrscidx_evt(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const uint32_t idx)
{
    /*
     * The open Start, still executing or past its deadline, when DEPCMD keeps
     * no record of it (udc_dwc3_on_ep_cmd_cmplt() asks DEPCMD first): the
     * event resolves it.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
        ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN) {
        udc_dwc3_store_xferrscidx(dev, ep_data, idx);
        (void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_RUNNING);
        return;
    }

    /*
     * Already RUNNING on the same index is the normal case, not a fault. The
     * post-poll in udc_dwc3_depcmd() usually sees CmdAct clear before this event
     * is drained, and udc_dwc3_adopt_xferrscidx() has already taken the index and
     * moved STARTING -> RUNNING.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_RUNNING) {
        if (ep_data->xferrscidx == idx) {
            return;
        }

        /* RUNNING with no index: adopt it; this event is the only place it exists. */
        if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
            udc_dwc3_store_xferrscidx(dev, ep_data, idx);
            return;
        }

        /*
         * A DIFFERENT valid index on a running endpoint. Do not silently
         * replace a working resource with another one.
         */
        LOG_WRN_RATELIMIT("EP%02x Start Transfer completion carries index %u "
                  "but the endpoint is running on index %u: NOT adopted",
                  ep_data->cfg.addr, idx, ep_data->xferrscidx);
        return;
    }

    /*
     * Anything else is a completion for a transfer the state says does not
     * exist: IDLE after a teardown or a refusal, or ENDING / END_UNKNOWN while
     * an End is outstanding.
     */
    LOG_WRN_RATELIMIT("EP%02x Start Transfer completion for index %u arrived "
              "while the endpoint is %s holding index %u: NOT adopted",
              ep_data->cfg.addr, idx,
              udc_dwc3_ep_state_name(ep_data->xfer.state),
              ep_data->xferrscidx);
}

/*
 * Recover a transfer resource index from DEPCMD if one is already there.
 */
static void udc_dwc3_peek_xferrscidx(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

    if (ep_data->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID) {
        return;
    }

    /* Undefined until this endpoint has been commanded at least once. */
    if (ep_data->cmd.depcmd_last == UDC_DWC3_DEPCMD_NONE) {
        return;
    }

    udc_dwc3_adopt_xferrscidx(dev, ep_data,
                  sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn)));
}

/* Returns true when the command was issued. */
static bool udc_dwc3_depcmd_update_xfer(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data)
{
    uint32_t flags = 0;

    udc_dwc3_peek_xferrscidx(dev, ep_data);

    /* Refuse: without a resource index there is nothing for Update Transfer to address. */
    if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
        if (ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
            ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN) {
            /* Not a fault: the Start is still executing. Owe it. */
            ep_data->xfer.pending |= UDC_DWC3_EP_PEND_UPDATE;
            LOG_DBG("Update Transfer on EP%02x owed: Start still open",
                ep_data->cfg.addr);
            return false;
        }
        LOG_ERR("Update Transfer on EP%02x refused: no transfer resource "
            "index established", ep_data->cfg.addr);
        return false;
    }

    /*
     * INVARIANT 2: Update Transfer only from RUNNING. In STARTING, ENDING or IDLE
     * there is no running transfer to address. Should be unreachable (the peek
     * above promotes STARTING to RUNNING; End Transfer invalidates the index), so
     * a hit is a real state error, worth one compare on the per-buffer path.
     */
    if (ep_data->xfer.state != UDC_DWC3_EP_RUNNING) {
        LOG_ERR("Update Transfer on EP%02x refused: endpoint is %s, not "
            "running (rscidx 0x%x)", ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state),
            ep_data->xferrscidx);
        return false;
    }

    /*
     * Same ordering requirement as Start Transfer. This is the hot path: it runs
     * per buffer on every bulk and interrupt endpoint.
     */
    flags |= UDC_DWC3_DEPCMD_DEPUPDXFER;
    flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

    /*
     * Checked but not re-logged: udc_dwc3_depcmd() has already logged the
     * endpoint, command and reason, and this is the hot path.
     */
    if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags, NULL) ==
        UDC_DWC3_XFERRSCIDX_INVALID) {
        return false;
    }

    /* DBG: this fires once per buffer from udc_dwc3_trb_bulk(). */
    LOG_DBG("DepUpdateXfer done EP%02x, addr 0x%08x, data 0x%08x, xferrscidx 0x%x",
        ep_data->cfg.addr, UDC_DWC3_DEPCMD(ep_data->epn), flags, ep_data->xferrscidx);

    return true;
}

/*
 * Issue the Update Transfer owed by a buffer armed while the Start was still open
 * (UDC_DWC3_EP_PEND_UPDATE), once the Start has its index. Called where a late
 * Start outcome is learnt: its Command Complete, and the sweep (which settles it
 * from DEPCMD when that event is lost). Without it the controller may never fetch
 * that descriptor: it caches the ring at Start time.
 */
static void udc_dwc3_ep_update_owed(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data)
{
    if ((ep_data->xfer.pending & UDC_DWC3_EP_PEND_UPDATE) == 0U ||
        ep_data->xfer.state != UDC_DWC3_EP_RUNNING ||
        ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
        return;
    }

    ep_data->xfer.pending &= (uint8_t)~UDC_DWC3_EP_PEND_UPDATE;
    if (udc_dwc3_ep_ring_outstanding(ep_data) &&
        !udc_dwc3_depcmd_update_xfer(dev, ep_data) &&
        !udc_dwc3_cmd_posted_ok(dev, ep_data, UDC_DWC3_DEPCMD_DEPUPDXFER)) {
        LOG_ERR("EP%02x owed Update Transfer refused with an armed ring",
            ep_data->cfg.addr);
    }
}

/*
 * The End Transfer did not end the transfer (refused, or never posted), so
 * the transfer still runs and owns its resource. Restore the index and return
 * to RUNNING. Shared by both places that learn this: udc_dwc3_depcmd_end_xfer()
 * at post time, and udc_dwc3_ep_resolve_cmd() later, from ENDING or END_UNKNOWN.
 */
static UDC_DWC3_COLD void udc_dwc3_ep_end_refused(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data)
{
    if (ep_data->xfer.end_idx != UDC_DWC3_XFERRSCIDX_INVALID) {
        udc_dwc3_store_xferrscidx(dev, ep_data, ep_data->xfer.end_idx);
    }
    ep_data->xfer.end_idx = UDC_DWC3_XFERRSCIDX_INVALID;
    ep_data->cmd.cmd_record = 0U;

    (void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_RUNNING);
}

/*
 * Issue End Transfer. Returns true only when the command was issued and will
 * raise an Endpoint Command Complete event, i.e. when the caller may wait for it.
 */
static UDC_DWC3_COLD bool udc_dwc3_depcmd_end_xfer(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data,
                     uint32_t flags)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

    if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
        /*
         * Not a fault: no stored index means no running transfer to end - IDLE
         * after a teardown (Linux dwc3 makes the same decision on
         * DWC3_EP_TRANSFER_STARTED), or a command outcome still open (STARTING,
         * ENDING or UNKNOWN, which all hold no index). An End over an open Start
         * could target a transfer that does not exist; the state is left as is
         * for that command's completion or udc_dwc3_ep_resolve_cmd().
         */
        LOG_DBG("End Transfer on EP%02x not issued: no started transfer",
            ep_data->cfg.addr);
        return false;
    }

    flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
    flags |= UDC_DWC3_DEPCMD_DEPENDXFER;

    /*
     * Ask for a completion event so the conclusion of bus traffic for this
     * transfer is observable - see UDC_DWC3_DEPCMD_CMDIOC.
     */
    if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0) {
        flags |= UDC_DWC3_DEPCMD_CMDIOC;

        /* INVARIANT 3: only from RUNNING. */
        if (!udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_ENDING)) {
            return false;
        }
    }

    /*
     * Save the index in end_idx across the command. udc_dwc3_depcmd() invalidates
     * xferrscidx before posting End Transfer, since a successful End returns the
     * resource. A rejected End does not: the transfer still runs and owns the
     * resource, and the endpoint goes back to RUNNING. RUNNING with no index is a
     * dead end (Update is refused, and no second End can reclaim it). Stored on
     * the endpoint because the refusal can be learnt after this returns.
     */
    ep_data->xfer.end_idx = ep_data->xferrscidx;

    /*
     * A failed return means the command was never issued: the previous command
     * on this endpoint was still active when the pre-poll gave up.
     */
    if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags, NULL) ==
        UDC_DWC3_XFERRSCIDX_INVALID) {
        /*
         * Still active is not "not issued", but udc_dwc3_depcmd() returns
         * XFERRSCIDX_INVALID for both. udc_dwc3_cmd_outcome() tells them apart from
         * DEPCMD: first the command type (a pre-poll that gave up leaves the previous
         * command there, active and not ours), then CmdAct. Taking that command's
         * CmdAct as our End's would leave ENDING waiting for a completion that never
         * comes.
         */
        const enum udc_dwc3_cmd_outcome out =
            udc_dwc3_cmd_outcome(dev, ep_data, UDC_DWC3_DEPCMD_DEPENDXFER,
                         NULL);

        if (out == UDC_DWC3_CMD_OTHER) {
            udc_dwc3_ep_end_refused(dev, ep_data);
            LOG_ERR("End Transfer NOT ISSUED on EP%02x: the previous "
                "command on it was still active; transfer left running",
                ep_data->cfg.addr);
            return false;
        }

        /*
         * Still executing. With RunStop set the endpoint stays ENDING for its
         * Command Complete; with RunStop clear no CMDIOC was asked for, so no
         * completion is coming and the caller treats the controller as stopped.
         */
        if (out == UDC_DWC3_CMD_UNKNOWN) {
            LOG_WRN("EP%02x End Transfer still executing past the poll "
                "budget%s", ep_data->cfg.addr,
                ((flags & UDC_DWC3_DEPCMD_CMDIOC) != 0U)
                    ? ": left ENDING for its Command Complete" : "");
            return (flags & UDC_DWC3_DEPCMD_CMDIOC) != 0U;
        }

        /*
         * The End completed after the poll expired, so the resource did come back.
         * Do not restore end_idx: RUNNING with an index the controller has reassigned
         * makes the next Update Transfer address another endpoint's transfer.
         */
        if (out == UDC_DWC3_CMD_OK) {
            return (flags & UDC_DWC3_DEPCMD_CMDIOC) != 0;
        }

        /* Refused, so the resource never came back (see end_idx above). */
        udc_dwc3_ep_end_refused(dev, ep_data);
        LOG_ERR("End Transfer REJECTED on EP%02x (CmdAct clear, so the "
            "controller refused it rather than still running it)",
            ep_data->cfg.addr);
        return false;
    }

    LOG_DBG("DepEndXfer done EP%02x", ep_data->cfg.addr);

    /* True only when a completion event is genuinely expected. */
    return (flags & UDC_DWC3_DEPCMD_CMDIOC) != 0;
}

/*
 * DEPSTARTCFG: (re)allocate the controller's transfer resource pool.
 */
static UDC_DWC3_COLD void udc_dwc3_depcmd_start_config(const struct device *const dev,
                     bool is_control)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    /* XferRscIdx 2 keeps resources 0 and 1, the EP0 halves'. */
    const uint8_t first = is_control ? 0U : 1U;
    uint32_t flags = 0;

    flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, is_control ? 0 : 2);
    flags |= UDC_DWC3_DEPCMD_DEPSTARTCFG;

    /* Not posted, or refused: the pool is unchanged, and so is every index. */
    if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(0), flags, NULL) != 0U &&
        !udc_dwc3_cmd_posted_ok(dev, &cfg->ep_data_out[0],
                    UDC_DWC3_DEPCMD_DEPSTARTCFG)) {
        LOG_ERR("DepStartConfig (%s) not taken by the controller",
            is_control ? "control" : "non-control");
        return;
    }

    /*
     * DEPSTARTCFG reassigns the transfer resources it covers, so every earlier
     * index there, and every transfer running, starting or ending on one, is
     * void. Callers issue it with every covered endpoint idle; work owed to the
     * stack or host (pending) is kept.
     */
    for (uint8_t i = first; i < cfg->num_in_eps; i++) {
        udc_dwc3_ep_state_reset(&cfg->ep_data_in[i]);
    }
    for (uint8_t i = first; i < cfg->num_out_eps; i++) {
        udc_dwc3_ep_state_reset(&cfg->ep_data_out[i]);
    }

    /*
     * New pool generation: every endpoint must issue DEPXFERCFG again, exactly
     * once (see udc_dwc3_ep_resume()).
     */
    ((struct udc_dwc3_data *)udc_get_private(dev))->epcfg.epoch++;

    LOG_DBG("DepStartConfig done ep=%s", is_control ? "control" : "non-control");
}

/*
 * Transfer Requests (TRB)
 *
 * The driver hands transfers to the controller as TRBs in shared memory,
 * submitted with each Start or Update Transfer command.
 */



/*
 * Report a non-control OUT buffer whose size is not a whole number of packets.
 * BUFSIZ must be (databook 4.2.3.3), so the TRB is programmed with the size
 * rounded up (udc_dwc3_trb_programmed_len()), which reaches past the buffer: a
 * full last packet writes up to MPS - 1 bytes beyond it. The class, not the
 * driver, sizes the buffer; the report names the overrun risk.
 */
static void udc_dwc3_out_size_check(const struct device *const dev,
                    const struct udc_dwc3_ep_data *const ep_data,
                    const uint32_t in_size, const uint32_t trb_size)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    /* Silent when the caller already supplied whole packets - the normal case. */
    if (trb_size == in_size) {
        return;
    }

    priv->diag.out_unaligned++;
    LOG_ERR_RATELIMIT("EP%02x OUT buffer is %u B, not a multiple of MPS %u B: "
              "programmed as %u B (databook 4.2.3.3), so a full packet can "
              "write %u B past the buffer (seen %u)",
              ep_data->cfg.addr, in_size, (uint32_t)USB_MPS_EP_SIZE(ep_data->cfg.mps),
              trb_size, trb_size - in_size, priv->diag.out_unaligned);
}

/*
 * Bytes programmed into the TRB for this buffer:
 *   - OUT: buf->size rounded up to whole packets (databook 4.2.3.3).
 *   - IN: buf->len.
 * Derived from the buffer because the controller overwrites BUFSIZ with the
 * bytes not transferred. The single definition for the arm and retire paths,
 * so they always agree.
 */
static uint32_t udc_dwc3_trb_programmed_len(const struct udc_dwc3_ep_data *const ep_data,
                        const struct net_buf *const buf)
{
    const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);

    if (buf == NULL) {
        return 0U;
    }

    if (!USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
        return buf->len;
    }

    return (mps != 0U) ? ROUND_UP(buf->size, mps) : buf->size;
}

/*
 * Arm one buffer in the endpoint's TRB ring and advance head.
 */
static void udc_dwc3_push_trb(const struct device *const dev,
                  struct udc_dwc3_ep_data *const ep_data,
                  struct net_buf *const buf, const uint32_t ctrl)
{
    volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->ring.head];
    const uint32_t out_size = udc_dwc3_trb_programmed_len(ep_data, buf);

    if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
        udc_dwc3_out_size_check(dev, ep_data, buf->size, out_size);
    }

    /*
     * The UDC mutex protects this ring, not the work queue: head, tail and
     * net_buf[] are shared with udc_dwc3_pop_trb(), and the dispatch thread is
     * not udc_get_work_q().
     */

    /*
     * The ring must not be full (next TRB still owned by the hardware); callers
     * retry later when a TRB frees up.
     */
    __ASSERT_NO_MSG(ep_data->ring.net_buf[ep_data->ring.head] == NULL);

    /* Associate an active buffer and a TRB together */
    ep_data->ring.net_buf[ep_data->ring.head] = buf;

    /*
     * OUT size is rounded up to whole packets (S4.2.3.3); see
     * udc_dwc3_trb_programmed_len().
     */
    ep_data->diag.n_arm++;

    udc_dwc3_trb_fill(trb, (uintptr_t)buf->data, out_size, ctrl);

    LOG_DBG("PUSH %u, buf %p, data %p, size %u -> %u",
        ep_data->ring.head, (void *)buf, (void *)buf->data, buf->size, out_size);

    /*
     * Per-arm trace for non-control endpoints. Debug only: it fires once per
     * buffer and the console is synchronous.
     */
    if (ep_data->cfg.addr != UDC_DWC3_TRBLOG_SKIP_EP) {
        LOG_DBG("EP%02x: ARM s%u len=%u n=%u", ep_data->cfg.addr,
            ep_data->ring.head, out_size, ep_data->diag.n_arm);
    }

    ep_data->ring.head = (ep_data->ring.head + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

    udc_dwc3_ep_busy_sync(ep_data);
}

/* True if the controller (HWO set) or software (net_buf held) still owns any slot. */
static bool udc_dwc3_ep_ring_outstanding(const struct udc_dwc3_ep_data *const ep_data)
{
    for (uint32_t i = 0U; i < (CONFIG_UDC_DWC3_TRB_NUM - 1U); i++) {
        if ((ep_data->trb_buf[i].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
            return true;
        }
        if (ep_data->ring.net_buf[i] != NULL) {
            return true;
        }
    }

    return false;
}

/*
 * Retire the oldest completed TRB. Returns -EBUSY while the controller still owns
 * it and -ENOBUFS when the slot holds no buffer.
 *
 * HWO is read first, and the rest of the TRB only once it reads clear (volatile
 * accesses, in program order), so the written-back BUFSIZ is never read ahead of
 * the HWO=0 that says the controller is done with the TRB.
 */
static int udc_dwc3_pop_trb(struct udc_dwc3_ep_data *const ep_data,
                struct net_buf **buf, struct udc_dwc3_trb *trb)
{
    const volatile struct udc_dwc3_trb *const t = &ep_data->trb_buf[ep_data->ring.tail];

    trb->ctrl = t->ctrl;
    if ((trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
        return -EBUSY;
    }

    *buf = ep_data->ring.net_buf[ep_data->ring.tail];
    if (*buf == NULL) {
        return -ENOBUFS;
    }

    trb->status = t->status;
    trb->addr_lo = t->addr_lo;
    trb->addr_hi = t->addr_hi;

    /* Clear the last TRB */
    ep_data->ring.net_buf[ep_data->ring.tail] = NULL;

    LOG_DBG("POP %u EP%02x, buf %p, data %p",
        ep_data->ring.tail, ep_data->cfg.addr, (void *)*buf, (void *)(*buf)->data);

    /* Retire side of the arm trace: together they give arm -> retire time per slot. */
    if (ep_data->cfg.addr != UDC_DWC3_TRBLOG_SKIP_EP) {
        LOG_DBG("EP%02x: RET s%u sts=%x n=%u", ep_data->cfg.addr,
            ep_data->ring.tail, trb->status, ep_data->diag.n_retire);
    }

    /* -1 for link trb */
    ep_data->ring.tail = (ep_data->ring.tail + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

    udc_dwc3_ep_busy_sync(ep_data);

    /* Received length = PROGRAMMED size minus residual BUFSIZ, not buf->size minus it. */
    if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
        const uint32_t programmed =
            udc_dwc3_trb_programmed_len(ep_data, *buf);
        const uint32_t residual =
            FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb->status);
        const uint32_t received =
            (programmed > residual) ? (programmed - residual) : 0U;

        (*buf)->len = MIN(received, (*buf)->size);
    }

    return 0;
}

/*
 * Build a non-control endpoint's ring and start its transfer.
 */
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

/*
 * Fill and start one control OUT stage TRB (data or status) on EP0-OUT. The
 * caller, udc_dwc3_ctrl_try(), has checked that the stage is due and the half
 * holds no transfer, which makes writing the TRB safe.
 */
static bool udc_dwc3_trb_ctrl_out(const struct device *const dev, struct net_buf *const buf,
                  const uint32_t ctrl)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[0];
    volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
    uint32_t size;

#ifdef CONFIG_UDC_DWC3_SHELL
    ((struct udc_dwc3_data *)udc_get_private(dev))->diag.last_xfer_dir = USB_EP_DIR_OUT;
#endif

    /*
     * A Status TRB has BUFSIZ 0 (Programming Guide 3.30b, Status TRB: "a Buffer
     * Size of zero. There is no data buffer associated with a Status TRB";
     * Table 4-14 step 9: "zero-bytes"). The general OUT rule (BUFSIZ a multiple of
     * MPS) does not apply to it.
     */
    if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
        ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
        size = 0U;
    } else {
        size = buf->size;
    }

    /*
     * A control OUT data TRB's BUFSIZ must be a multiple of wMaxPacketSize
     * (4.4.2 step 5a), or a host ending an exact-multiple stage with a ZLP has
     * nowhere to put it.
     */
    if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
        const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);

        if (mps != 0U) {
            size = ROUND_UP(size, mps);
        }
    }

    udc_dwc3_trb_fill(&trb[0], (uintptr_t)buf->data, size,
              ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO);

    return udc_dwc3_depcmd_start_xfer(dev, ep_data);
}


/*
 * Fill and start one control IN stage TRB (data or status) on EP0-IN, with a
 * chained zero-length TRB when the data stage needs a ZLP. Same contract as
 * udc_dwc3_trb_ctrl_out().
 */
static bool udc_dwc3_trb_ctrl_in(const struct device *const dev,
                 struct net_buf *const buf,
                 const uint32_t ctrl)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
    volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

#ifdef CONFIG_UDC_DWC3_SHELL
    ((struct udc_dwc3_data *)udc_get_private(dev))->diag.last_xfer_dir = USB_EP_DIR_IN;
#endif

    if (udc_ep_buf_has_zlp(buf)) {
        udc_dwc3_trb_fill(&trb[0], (uintptr_t)buf->data, buf->len,
                  ctrl | UDC_DWC3_TRB_CTRL_CHN |
                  UDC_DWC3_TRB_CTRL_HWO);
        udc_dwc3_trb_fill(&trb[1], 0U, 0U,
                  UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL |
                  UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO);
    } else {
        udc_dwc3_trb_fill(&trb[0], (uintptr_t)buf->data, buf->len,
                  ctrl | UDC_DWC3_TRB_CTRL_LST |
                  UDC_DWC3_TRB_CTRL_HWO);
    }

    return udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

/*
 * Arm one bulk/interrupt buffer and tell the controller about it. Returns -EBUSY
 * if the ring is full, or -EIO while a refused Start keeps the endpoint out of
 * service; in both cases nothing was armed and the caller keeps the buffer.
 */
static int udc_dwc3_trb_bulk(const struct device *const dev,
                 struct udc_dwc3_ep_data *const ep_data,
                 struct net_buf *const buf)
{
    uint32_t ctrl = UDC_DWC3_TRB_CTRL_IOC | UDC_DWC3_TRB_CTRL_HWO;

    /*
     * CSP (Continue on Short Packet) is set for OUT only. For IN, XferComplete
     * needs LST=1 (Table 4-8), which this driver never sets, and XferComplete and
     * XferInProgress go to the same handler anyway (udc_dwc3_on_xfer_done_nonctrl()).
     */
    if (!USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
        ctrl |= UDC_DWC3_TRB_CTRL_CSP;
    }

    /*
     * DBG, like the control stages: a log line per transfer on a data endpoint
     * capped control traffic at ~41/s.
     */
    LOG_DBG("TRB_BULK_EP_0x%02x, buf %p, data %p, size %u, len %u",
        ep_data->cfg.addr, (void *)buf, (void *)buf->data, buf->size, buf->len);

    if (ep_data->ring.net_buf[ep_data->ring.head] != NULL) {
        return -EBUSY;
    }

    /*
     * Out of service after a refused Start: IDLE and out of DALEPENA
     * (udc_dwc3_ep_start_refused()). Nothing more is armed until an enable
     * puts the endpoint back in DALEPENA. Also stops the caller's loop after a
     * refusal inside this very call. Checked only while IDLE, from the driver's
     * copy of DALEPENA.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_IDLE &&
        !udc_dwc3_ep_in_dalepena(udc_get_private(dev), ep_data)) {
        return -EIO;
    }

    if (udc_ep_buf_has_zlp(buf)) {
        LOG_DBG("Buffer has a ZLP flag");
        ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP;
    } else {
        ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL;
    }

    udc_dwc3_push_trb(dev, ep_data, buf, ctrl);

    /*
     * Start or Update, decided only here, from xfer.state. IDLE means the
     * controller has released the transfer resource (XferComplete and every reset
     * path leave it IDLE), so there is nothing to Update and the arm needs a Start.
     *
     * A failure is not unwound: the TRB is already armed with HWO set, and the
     * buffer stays owned by net_buf[]. Returning an error after
     * udc_dwc3_push_trb() would leave it in both net_buf[] and the stack's queue,
     * and the double free panics the device. An Update refused while the Start is
     * still open is owed and issued once the index is known
     * (udc_dwc3_ep_update_owed()); any other refusal is logged where it happens.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_IDLE) {
        (void)udc_dwc3_depcmd_start_xfer(dev, ep_data);
    } else {
        (void)udc_dwc3_depcmd_update_xfer(dev, ep_data);
    }

    return 0;
}

/*
 * Control buffers
 */




/*
 * Record the control endpoint and stage just armed, for the SETUP watchdog
 * report (udc_dwc3_ctrl_setup_wd_check()), which only reports.
 */
static void udc_dwc3_ctrl_arm_watchdog(const struct device *const dev,
                       const bool is_in, const uint32_t type)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    priv->diag.watchdog_ep = is_in ? &cfg->ep_data_in[0] : &cfg->ep_data_out[0];
    priv->diag.watchdog_type = type;

    /*
     * Snapshot the traffic counters, so the SETUP watchdog can tell "nothing has
     * moved since this SETUP was armed" from "the bus is busy and the shared
     * RxFIFO holds someone else's data".
     */
    if (type == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
        priv->diag.ctrl_setup_wd_snap_setup = priv->diag.ctrl_setup_done;
        priv->diag.ctrl_setup_wd_snap_nonctrl = priv->diag.nonctrl_done;

        /*
         * Telemetry only, SETUP only (see udc_dwc3_ctrl_setup_wd_check()); DATA and
         * STATUS stages are driven by XferNotReady and SetupPending. No timer: the
         * heartbeat ages the armed SETUP, so the hot path pays one increment instead
         * of a timeout insert and removal per stage.
         */
        priv->diag.ctrl_setup_wd_gen++;
    }
}

/*
 * Arm the data or status stage this buffer describes on EP0-IN (the caller,
 * udc_dwc3_ctrl_try(), passes only those). Returns false if the Start Transfer
 * was not taken; the buffer stays queued.
 */
static bool udc_dwc3_ctrl_next_in(const struct device *const dev,
                  struct net_buf *const buf)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const struct usb_setup_packet *const setup = &priv->ctrl.setup;
    const struct udc_buf_info bi = *udc_get_buf_info(buf);

    if (bi.data) {
        LOG_DBG("trb IN_DATA ln=%d d=%p", buf->len, (void *)buf->data);
        if (!udc_dwc3_trb_ctrl_in(dev, buf,
                      UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA)) {
            return false;
        }
        udc_dwc3_ctrl_arm_watchdog(dev, true, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
    } else if (bi.status && setup->wLength == 0) {
        /* An IN status stage sends a ZLP, so its TRB has BUFSIZ 0. */
        buf->size = 0;
        buf->len = 0;
        LOG_DBG("trb IN_STATUS_2 ln=%d d=%p", buf->len, (void *)buf->data);
        if (!udc_dwc3_trb_ctrl_in(dev, buf,
                      UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2)) {
            return false;
        }
        udc_dwc3_ctrl_arm_watchdog(dev, true, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
    } else {
        /* Same as the two-stage case: buf->len is what reaches the TRB. */
        buf->size = 0;
        buf->len = 0;
        LOG_DBG("trb IN_STATUS_3 ln=%d d=%p", buf->len, (void *)buf->data);
        if (!udc_dwc3_trb_ctrl_in(dev, buf,
                      UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3)) {
            return false;
        }
        udc_dwc3_ctrl_arm_watchdog(dev, true, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
    }

    return true;
}

/* Same as udc_dwc3_ctrl_next_in(), on EP0-OUT. */
static bool udc_dwc3_ctrl_next_out(const struct device *const dev,
                   struct net_buf *const buf)
{
    const struct udc_buf_info bi = *udc_get_buf_info(buf);

    if (bi.data) {
        LOG_DBG("trb OUT_DATA sz=%d d=%p", buf->size, (void *)buf->data);
        if (!udc_dwc3_trb_ctrl_out(dev, buf,
                       UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA)) {
            return false;
        }
        udc_dwc3_ctrl_arm_watchdog(dev, false, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
    } else {
        /*
         * buf->size is left as allocated (bMaxPacketSize0, by udc_ctrl_status_alloc());
         * udc_dwc3_trb_ctrl_out() programs BUFSIZ 0 for a status TRB regardless.
         */
        LOG_DBG("trb OUT_STATUS_3 sz=%d d=%p", buf->size, (void *)buf->data);
        if (!udc_dwc3_trb_ctrl_out(dev, buf,
                       UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3)) {
            return false;
        }
        udc_dwc3_ctrl_arm_watchdog(dev, false, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
    }

    return true;
}

/*
 * EP0 follows the control transfer programming model of SPEC 3.30b 4.4 /
 * Figure 4-2, and nothing else. The hardware "automatically recovers from these
 * scenarios as long as software follows this single control transfer
 * programming model".
 *
 * ctrl.state is the current node of Figure 4-2. Each EP0/EP1 event is either an
 * edge from that node (advance), not one (ignore), or one of the databook's
 * error cases (udc_dwc3_ctrl_ep_recover()). No per-scenario handling: the
 * controller absorbs aborts, suspend/resume and resets while the flow is
 * followed.
 *
 * Stage endpoints (4.4):
 *   - control read: SETUP EP0, data EP1, status EP0
 *   - control write / two-stage: SETUP EP0, data EP0, status EP1
 */
static void udc_dwc3_ctrl_next(const struct device *const dev);
static void udc_dwc3_ctrl_ep_recover(const struct device *const dev);

/*
 * Return every buffer queued on this control endpoint that belongs to the
 * transfer being abandoned, stopping at a SETUP. Returns how many.
 */
static UDC_DWC3_COLD uint32_t udc_dwc3_ctrl_drain_abandoned(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data)
{
    struct net_buf *buf;
    uint32_t n = 0U;

    while ((buf = udc_buf_peek(&ep_data->cfg)) != NULL) {
        if (udc_get_buf_info(buf)->setup) {
            break;
        }

        buf = udc_buf_get(&ep_data->cfg);
        if (buf == NULL) {
            break;
        }

        udc_dwc3_buf_return(dev, buf, UDC_DWC3_BUF_ABANDONED);
        n++;
    }

    return n;
}

/*
 * Is this control endpoint busy only because a SETUP TRB is armed on it?
 * Read from the TRB (HWO set, TRBCTL = SETUP), not from saved state, so it
 * stays correct across abandon and re-arm.
 */
static bool udc_dwc3_ctrl_armed_setup(struct udc_dwc3_ep_data *const ep_data)
{
    /* One read, so both tests see the same word. */
    const uint32_t ctrl = ep_data->trb_buf[0].ctrl;

    return (ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0 &&
           (ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK) ==
               UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP;
}

/* From the SETUP of the request in progress (4.4): data stage is device-to-host. */
static inline bool udc_dwc3_ctrl_dir_in(const struct udc_dwc3_data *const priv)
{
    return priv->ctrl.setup.RequestType.direction == USB_REQTYPE_DIR_TO_HOST;
}

/* From the SETUP of the request in progress: it has a data stage (wLength != 0). */
static inline bool udc_dwc3_ctrl_three_stage(const struct udc_dwc3_data *const priv)
{
    return sys_le16_to_cpu(priv->ctrl.setup.wLength) != 0U;
}

/* The status stage's direction for the request in progress (4.4). */
static inline bool udc_dwc3_ctrl_status_is_in(const struct udc_dwc3_data *const priv)
{
    return !(udc_dwc3_ctrl_three_stage(priv) && udc_dwc3_ctrl_dir_in(priv));
}

/* udc_dwc3_ctrl_ep_recover() is in progress. */
static inline bool udc_dwc3_ctrl_recovering(const struct udc_dwc3_data *const priv)
{
    return priv->ctrl.state == UDC_DWC3_CTRL_RECOVERING ||
           priv->ctrl.state == UDC_DWC3_CTRL_RECOVERING_STALL;
}

/*
 * Arm EP0-OUT for the next SETUP (4.4.1/4.4.2 step 1: "Software sets up a Setup
 * TRB and issues Start Transfer on EP0 pointing to the Setup TRB").
 * Uses the driver's own buffer, so the arm never waits behind anything the stack
 * has queued. Runs only in the Setup phase with no transfer on EP0-OUT; otherwise
 * (a stage being ended, or this SETUP already armed) the release re-enters
 * udc_dwc3_ctrl_next() and arms it then.
 */
static void udc_dwc3_ctrl_arm_setup(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct udc_dwc3_ep_data *const out0 = &cfg->ep_data_out[0];

    if (priv->ctrl.state != UDC_DWC3_CTRL_IDLE ||
        out0->xfer.state != UDC_DWC3_EP_IDLE) {
        return;
    }

    /*
     * In the Setup phase no transfer is live, so a stage buffer at the head of
     * either EP0 queue belongs to an abandoned transfer (the state leaves IDLE
     * before a SETUP reaches the stack, and the stack queues a request's stage
     * buffers before its next SETUP buffer). The stack can still queue them after
     * the driver went back to Step 1, as its thread may be preempted between
     * enqueues. Left in place they would block the gate below forever, so return
     * them; Figure 4-2 goes straight back to the Setup TRB after an abandoned
     * transfer.
     */
    {
        struct udc_dwc3_ep_data *const in0 = &cfg->ep_data_in[0];
        const struct net_buf *const oh = udc_buf_peek(&out0->cfg);
        const struct net_buf *const ih = udc_buf_peek(&in0->cfg);

        if ((oh != NULL && !udc_get_buf_info(oh)->setup) || ih != NULL) {
            priv->diag.ctrl_stale_returned += udc_dwc3_ctrl_drain_abandoned(dev, out0) +
                             udc_dwc3_ctrl_drain_abandoned(dev, in0);
        }
    }

    /*
     * Arm the SETUP only while the stack's SETUP buffer is queued on EP0-OUT, so a
     * SETUP never arrives before the stack is ready. Otherwise udc_setup_received()
     * caches it (setup_pending) and udc_ep_enqueue() completes the buffer later; if
     * the stack's message queue is full then, the stack reads -ENOMSG as "not
     * queued" and drops a reference to a buffer that is still queued.
     * udc_dwc3_ep_enqueue() calls back here when the stack queues the buffer.
     */
    {
        struct net_buf *const head = udc_buf_peek(&out0->cfg);

        if (head == NULL || !udc_get_buf_info(head)->setup) {
            return;
        }
    }

    memset(cfg->setup_buf, 0x00, 8U);
    udc_dwc3_trb_fill(&out0->trb_buf[0], (uintptr_t)cfg->setup_buf, 8U,
              UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP |
              UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO);
#ifdef CONFIG_UDC_DWC3_SHELL
    priv->diag.last_xfer_dir = USB_EP_DIR_OUT;
#endif

    /*
     * Not taken (not posted, or refused): the controller does not own the TRB, so
     * clear it. Left with HWO set it would read as an armed SETUP
     * (udc_dwc3_ctrl_armed_setup()) and nothing would arm one again.
     */
    if (!udc_dwc3_depcmd_start_xfer(dev, out0)) {
        udc_dwc3_trb_write(&out0->trb_buf[0], 0U, 0U, 0U);
        return;
    }

    udc_dwc3_ctrl_arm_watchdog(dev, false, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);
}

/*
 * Arm the stage buffer queued on this half only if Figure 4-2 says that stage is
 * due now, on this endpoint, and the half holds no transfer. Otherwise the buffer
 * waits; the event that makes it due (XferNotReady(Status), or the SETUP's
 * XferComplete) calls back here.
 *
 * Never arms the stack's SETUP buffer: the driver owns the SETUP stage
 * (udc_dwc3_ctrl_arm_setup()) and udc_setup_received() fills that buffer.
 */
static void udc_dwc3_ctrl_try(const struct device *const dev,
                  struct udc_dwc3_ep_data *const ep_data)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const bool is_in = USB_EP_DIR_IS_IN(ep_data->cfg.addr);
    struct net_buf *const buf = udc_buf_peek(&ep_data->cfg);
    const struct udc_buf_info *bi;
    bool armed = false;

    if (buf == NULL || udc_dwc3_ctrl_recovering(priv) ||
        ep_data->xfer.state != UDC_DWC3_EP_IDLE) {
        return;
    }

    bi = udc_get_buf_info(buf);
    if (bi->setup) {
        return;
    }

    if (bi->data) {
        /* Step 3: the data stage, on the endpoint bmRequestType names. */
        if (priv->ctrl.state != UDC_DWC3_CTRL_SETUP_DONE ||
            !udc_dwc3_ctrl_three_stage(priv) || is_in != udc_dwc3_ctrl_dir_in(priv)) {
            return;
        }
        armed = is_in ? udc_dwc3_ctrl_next_in(dev, buf)
                  : udc_dwc3_ctrl_next_out(dev, buf);
        if (armed) {
            udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_DATA_ARMED);
        }
    } else if (bi->status) {
        /* 4.4.1 step 4 / 4.4.2 step 7: only after XferNotReady(Status). */
        if (priv->ctrl.state != UDC_DWC3_CTRL_STATUS_READY ||
            is_in != udc_dwc3_ctrl_status_is_in(priv)) {
            return;
        }
        armed = is_in ? udc_dwc3_ctrl_next_in(dev, buf)
                  : udc_dwc3_ctrl_next_out(dev, buf);
        if (armed) {
            udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_STATUS_ARMED);
        }
    } else {
        return;
    }

    /*
     * The stage's Start Transfer was not taken: no event will move this transfer
     * on, so take it back to Step 1 (Set Stall, fresh SETUP).
     */
    if (!armed) {
        udc_dwc3_ctrl_ep_recover(dev);
    }
}

/* Arm whatever Figure 4-2 says is due now: a queued stage, or the SETUP. */
static void udc_dwc3_ctrl_next(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;

    udc_dwc3_ctrl_try(dev, &cfg->ep_data_in[0]);
    udc_dwc3_ctrl_try(dev, &cfg->ep_data_out[0]);
    udc_dwc3_ctrl_arm_setup(dev);
}

/* UDC API: cancel queued buffers on an endpoint. */
static int udc_dwc3_ep_dequeue(const struct device *const dev,
                   struct udc_ep_config *const ep_cfg);
static int udc_dwc3_disable(const struct device *const dev);
static int udc_dwc3_enable(const struct device *const dev);
static int udc_dwc3_init(const struct device *const dev);
static int udc_dwc3_ep_enable(const struct device *const dev, struct udc_ep_config *const ep_cfg);

/*
 * Second half of a control-endpoint recovery.
 */

/* Read the core debug registers - the ones worth comparing healthy vs wedged - into *s. */
static void udc_dwc3_core_dbg_read(const mm_reg_t base,
                   struct udc_dwc3_core_dbg *const s)
{
    /*
     * GDBGLSP is a muxed window: select the source before reading it. Do not
     * decode GDBGLSP until the device-mode selector encoding is confirmed
     * against the databook.
     */
    sys_write32(0U, base + UDC_DWC3_GDBGLSPMUX_DEV);

    s->ltssm   = sys_read32(base + UDC_DWC3_GDBGLTSSM);
    s->bmu     = sys_read32(base + UDC_DWC3_GDBGBMU);
    s->lnmcc   = sys_read32(base + UDC_DWC3_GDBGLNMCC);
    s->lsp     = sys_read32(base + UDC_DWC3_GDBGLSP);
    s->epinfo0 = sys_read32(base + UDC_DWC3_GDBGEPINFO0);
    s->epinfo1 = sys_read32(base + UDC_DWC3_GDBGEPINFO1);
}

/* Log one sample of the core debug registers. */
static void udc_dwc3_core_dbg_log(const char *const tag,
                  const struct udc_dwc3_core_dbg *const s)
{
    LOG_INF("  CORE%s: GDBGLTSSM=0x%08x GDBGBMU=0x%08x GDBGLNMCC=0x%08x "
        "GDBGLSP=0x%08x GDBGEPINFO=0x%08x_%08x",
        tag, s->ltssm, s->bmu, s->lnmcc, s->lsp, s->epinfo1, s->epinfo0);
}

/* Dump the controller's own view of itself (passive register reads only). */
static void udc_dwc3_core_state_dump(const struct device *const dev)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    struct udc_dwc3_core_dbg dbg;
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
    };

    udc_dwc3_core_dbg_read(base, &dbg);
    udc_dwc3_core_dbg_log("", &dbg);

    for (uint32_t i = 0; i < ARRAY_SIZE(queues); i++) {
        uint32_t r = queues[i].sel;

        /* Queue 0 of each type. */
        r |= FIELD_PREP(UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK, 0U);
        sys_write32(r, base + UDC_DWC3_GDBGFIFOSPACE);
        r = sys_read32(base + UDC_DWC3_GDBGFIFOSPACE);

        LOG_INF("  CORE: %-9s space=%u (raw 0x%08x)", queues[i].name,
            (uint32_t)FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK, r),
            r);
    }
}

/*
 * Reclaim one control half's TRBs once the controller holds no transfer on it:
 * clear both TRBs and, on an IN half, flush the TxFIFO (it may still hold data
 * for a stage that never went out).
 *
 * SPEC 3.30b Programming Guide Table 4-14 step 8: "Software has to reclaim the
 * TRBs with HWO=1 in the skipped TRBs and flush the TxFIFO." Safe only when no
 * transfer owns the half, i.e. after its XferComplete (3.2.2.2) or End Transfer
 * completion; every caller guarantees this.
 */
static UDC_DWC3_COLD void udc_dwc3_ctrl_reclaim_half(const struct device *const dev,
                       struct udc_dwc3_ep_data *const h)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    udc_dwc3_trb_write(&h->trb_buf[0], 0U, 0U, 0U);
    udc_dwc3_trb_write(&h->trb_buf[1], 0U, 0U, 0U);

    if (USB_EP_DIR_IS_IN(h->cfg.addr)) {
        udc_dwc3_fifo_flush_tx(dev, h->cfg.addr & 0x7fU);
    }

    priv->diag.ctrl_reclaim_done++;
}

#ifdef CONFIG_UDC_DWC3_SHELL
/* Used only by the "dwc3 recover" shell command. */
static UDC_DWC3_COLD int udc_dwc3_recover(const struct device *dev)
{
    udc_lock_internal(dev, K_FOREVER);
    udc_dwc3_ctrl_ep_recover(dev);
    udc_unlock_internal(dev);

    return 0;
}
#endif /* CONFIG_UDC_DWC3_SHELL */

/*
 * Events
 *
 * Events are fetched from an event ring shared with the hardware; the interrupt
 * only signals that one is available.
 */

/* Defined below; teardown needs it to move armed buffers off the ring. */
static void udc_dwc3_ep_ring_release(struct udc_dwc3_ep_data *const ep_data);

/* Defined below; the endpoint recovery retires written-back TRBs before a restart. */
static uint32_t udc_dwc3_drain_completed(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data);

/*
 * Return every buffer parked on requeue_fifo to the stack; returns the count.
 *
 * udc_dwc3_ep_ring_release() moves armed buffers onto that fifo, and only
 * udc_dwc3_ep_resume() consumes it. On teardown nothing resumes the endpoint, so
 * without this the buffers are never unref'd or reported, are invisible to
 * udc_ep_cancel_queued(), and leak from a fixed pool.
 */
static UDC_DWC3_COLD uint32_t udc_dwc3_ep_return_parked(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const enum udc_dwc3_buf_end end)
{
    uint32_t n = 0U;

    for (;;) {
        struct net_buf *parked =
            k_fifo_get(&ep_data->requeue_fifo, K_NO_WAIT);

        if (parked == NULL) {
            return n;
        }

        udc_dwc3_buf_return(dev, parked, end);
        n++;
    }
}

/*
 * The single non-control endpoint recovery: takes the endpoint back to a known
 * state. Used by USB reset and disconnect, SetConfiguration/SetInterface,
 * ClearFeature(ENDPOINT_HALT), dequeue and disable. The End Transfer completion,
 * the Start Transfer completion and the heartbeat sweep call it again to continue
 * an unfinished recovery. Callers cite no spec; the databook basis is here.
 *
 * The action depends only on the endpoint state:
 *
 *   1. A command outcome is open (STARTING / ENDING / *_UNKNOWN): issue nothing
 *      on the endpoint. Its Command Complete, or the resolver reading DEPCMD,
 *      re-enters here.
 *
 *   2. A transfer holds a resource (xferrscidx valid): End Transfer with ForceRM=1
 *      and CmdIOC set, so a Command Complete reports the end (SPEC 3.2.2.7).
 *      Required on USB reset (4.1.2, 4.2.5), SetConfiguration (4.1.5, all but
 *      EP0), after ClearFeature(STALL) (4.2.7), and before buffers are given back
 *      (dequeue/disable). End Transfer raises no XferComplete and does not update
 *      TRB status (3.2.2.7); the resource is released when it completes (4.2.5).
 *      Until then the ring is the controller's: posted is not completed.
 *
 *   3. No transfer: the ring is the driver's again. In order:
 *      - Buffers: returned as cancelled (-ECONNABORTED, as udc_ep_cancel_queued()
 *        does) after a dequeue, disable or USB reset; parked for re-enable if a
 *        refused Start took the endpoint out of DALEPENA; otherwise left in place.
 *      - Clear Stall, if owed. 4.2.7: End Transfer first, then Clear Stall.
 *        4.1.2: clear any stalled endpoint on USB reset. 3.2.2.4: software owns
 *        STALL on non-control endpoints, so it is owed only on those two
 *        requests, never on a recovery of its own.
 *      - Start Transfer again if enabled (4.2.7): either the resume deferred
 *        behind the End (3.2.2.7: no Start until the End has reported), or a
 *        Start here for buffers still armed on the ring, then the endpoint worker
 *        for whatever the stack has queued since.
 *
 * The only inputs that are not controller state are requests recorded in
 * ep_data->xfer.pending by the path that received them: a Clear Stall owed
 * (ClearFeature(ENDPOINT_HALT), USB reset), a cancel owed (dequeue, disable, USB
 * reset), and a resume deferred by udc_dwc3_ep_resume().
 */
static UDC_DWC3_COLD void udc_dwc3_ep_recover(const struct device *const dev,
                struct udc_dwc3_ep_data *const ep_data)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const struct udc_dwc3_data *const priv = udc_get_private(dev);
    uint32_t returned;
    uint8_t owed;

    if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U) {
        return;
    }

    /* 1. An outcome is open: its completion or the resolver carries on. */
    if (udc_dwc3_ep_cmd_busy(ep_data)) {
        return;
    }

    /* 2. A transfer holds a resource: end it, and wait for the report. */
    if (ep_data->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID) {
        if (udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
            LOG_INF("EP%02x recovery: End Transfer (owed 0x%02x)",
                ep_data->cfg.addr, ep_data->xfer.pending);
            return;
        }
        if (udc_dwc3_ep_cmd_busy(ep_data)) {
            return;
        }
        if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0U) {
            /* Refused on a running controller: it still owns the ring. */
            LOG_ERR_RATELIMIT("EP%02x recovery: End Transfer refused, transfer "
                      "left active (owed 0x%02x)", ep_data->cfg.addr,
                      ep_data->xfer.pending);
            return;
        }
        /* RunStop clear: the controller is stopped, no completion is coming. */
        udc_dwc3_ep_state_reset(ep_data);
    }

    /* 3. No transfer: carry out what is owed, in the databook's order. */
    owed = ep_data->xfer.pending;
    ep_data->xfer.pending = UDC_DWC3_EP_PEND_NONE;
    returned = 0U;

    if ((owed & UDC_DWC3_EP_PEND_DEQUEUE) != 0U) {
        /* Cancelled, as udc_ep_cancel_queued() returns them. */
        udc_dwc3_ep_ring_release(ep_data);
        returned = udc_dwc3_ep_return_parked(dev, ep_data, UDC_DWC3_BUF_CANCELLED);
    } else if ((sys_read32(base + UDC_DWC3_DALEPENA) &
            UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn)) == 0U) {
        /* Not in DALEPENA (a refused Start took it out): park for the next resume. */
        udc_dwc3_ep_ring_release(ep_data);
    }

    if ((owed & UDC_DWC3_EP_PEND_CLEAR_STALL) != 0U &&
        !udc_dwc3_depcmd_clear_stall(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
        LOG_ERR("EP%02x recovery: Clear Stall refused; endpoint remains halted",
            ep_data->cfg.addr);
    }

    /*
     * Only when something was done: a USB reset or disconnect recovers every
     * endpoint, and most have nothing armed and nothing owed but the cancel.
     */
    if (returned != 0U || (owed & (uint8_t)~UDC_DWC3_EP_PEND_DEQUEUE) != 0U) {
        LOG_INF("EP%02x recovery: done (owed 0x%02x, returned %u%s%s)",
            ep_data->cfg.addr, owed, returned,
            ((owed & UDC_DWC3_EP_PEND_CLEAR_STALL) != 0U) ? ", clear-stall" : "",
            ((owed & UDC_DWC3_EP_PEND_RESUME) != 0U) ? ", resume" :
            (ep_data->cfg.stat.enabled ? ", restart" : ""));
    }

    if ((owed & UDC_DWC3_EP_PEND_RESUME) != 0U && udc_dwc3_run_halting(priv)) {
        /*
         * A resume starts a transfer, which 4.1.8 rules out once the transfers
         * are being ended: it stays owed. The controller reset that follows
         * forgets it (udc_dwc3_drop_xfer_state()).
         */
        ep_data->xfer.pending |= UDC_DWC3_EP_PEND_RESUME;
    } else if ((owed & UDC_DWC3_EP_PEND_RESUME) != 0U) {
        const int ret = udc_dwc3_ep_resume(dev, ep_data);

        if (ret != 0) {
            LOG_ERR("EP%02x recovery: resume failed: %d", ep_data->cfg.addr, ret);
            udc_submit_event(dev, UDC_EVT_ERROR, ret);
        }
    } else if (ep_data->cfg.stat.enabled) {
        /*
         * Buffers still armed (enabled in DALEPENA, nothing cancelled) need a
         * Start of their own: the worker arms only what the stack queues, so with
         * nothing new queued the ring would sit idle. Same gates as the worker.
         * Written-back TRBs are retired first, so the Start (at tail) points at a
         * TRB the controller still owns.
         */
        if (!udc_dwc3_run_halting(priv) &&
            (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0U &&
            !ep_data->cfg.stat.halted) {
            (void)udc_dwc3_drain_completed(dev, ep_data);
            if (ep_data->xfer.state == UDC_DWC3_EP_IDLE &&
                udc_dwc3_ep_ring_outstanding(ep_data)) {
                (void)udc_dwc3_depcmd_start_xfer(dev, ep_data);
            }
        }
        k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
    }
}

/*
 * Does this endpoint still owe a recovery? Either a request is recorded, or it
 * is disabled in DALEPENA while a transfer or an armed ring is still there.
 */
static bool udc_dwc3_ep_recovery_owed(const struct device *const dev,
                      const struct udc_dwc3_ep_data *const ep_data)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

    if ((ep_data->xfer.pending & (uint8_t)~UDC_DWC3_EP_PEND_UPDATE) != UDC_DWC3_EP_PEND_NONE) {
        return true;
    }
    if ((sys_read32(base + UDC_DWC3_DALEPENA) &
         UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn)) != 0U) {
        return false;
    }
    return ep_data->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID ||
           udc_dwc3_ep_ring_outstanding(ep_data);
}


/*
 * One pass of the control-endpoint recovery (see udc_dwc3_ctrl_ep_recover()).
 * Each half is handled from the controller's handle on it, never from the stage
 * or the host scenario that led here:
 *
 *   command outcome open    -> wait; its completion (or the resolver) re-enters.
 *   transfer resource valid -> End Transfer (ForceRM); its completion re-enters
 *                              (refused: the heartbeat re-enters and retries).
 *   nothing live            -> return the half's queued stage buffers. The drain
 *                              stops at the stack's SETUP buffer, which stays.
 *
 * Once both halves hold nothing: Set Stall if the case calls for one (after the
 * End, per 4.4.2 step 3a), then enter the Setup phase and arm it.
 *
 * "xferrscidx valid" means "a transfer is live" only because every EP0/EP1
 * completion releases the half (udc_dwc3_ctrl_release()); the controller frees
 * the resource when it generates XferComplete (3.2.2.2).
 */
static UDC_DWC3_COLD void udc_dwc3_ctrl_recover_continue(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    struct udc_dwc3_ep_data *const halves[2] = {
        &cfg->ep_data_out[0], &cfg->ep_data_in[0],
    };
    bool waiting = false;

    if (!udc_dwc3_ctrl_recovering(priv)) {
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(halves); i++) {
        struct udc_dwc3_ep_data *const h = halves[i];

        if (udc_dwc3_ep_cmd_busy(h)) {
            waiting = true;
            continue;
        }

        if (h->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID) {
            if (udc_dwc3_depcmd_end_xfer(dev, h, UDC_DWC3_DEPCMD_HIPRI_FORCERM) ||
                udc_dwc3_ep_cmd_busy(h)) {
                waiting = true;
                continue;
            }
            /*
             * Refused (or not posted) on a running controller: the transfer still
             * runs and owns its TRBs. Wait; the heartbeat's
             * udc_dwc3_recover_all() comes back here and ends it again.
             */
            if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0U) {
                waiting = true;
                continue;
            }
            /* RunStop clear: the controller is stopped, no completion is coming. */
            udc_dwc3_ep_state_reset(h);
        }

        (void)udc_dwc3_ctrl_drain_abandoned(dev, h);
    }

    if (waiting) {
        return;
    }

    /* Neither half holds a transfer: both descriptors and TxFIFO 0 are the driver's. */
    for (size_t i = 0; i < ARRAY_SIZE(halves); i++) {
        udc_dwc3_ctrl_reclaim_half(dev, halves[i]);
    }

    if (priv->ctrl.state == UDC_DWC3_CTRL_RECOVERING_STALL) {
        (void)udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_out[0]);
    }

    priv->ctrl.setup_pending = false;
    udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_IDLE);
    udc_dwc3_ctrl_next(dev);
}

/*
 * The single control-endpoint recovery: back to Step 1.
 *
 * Every databook case ends there:
 *   - 4.4.1/4.4.2 "go back to Step 1": bad setup bytes; a stage XferNotReady
 *     before the SETUP's XferComplete; a data stage on a two-stage request or in
 *     the wrong direction; more data than wLength; a failed data stage at
 *     XferNotReady(Status).
 *   - 4.1.2 USB reset, 4.1.8 device-initiated disconnect: "complete it and get
 *     the controller into the Setup TRB / Start Transfer state".
 * The reference driver does the same (dwc3_ep0_end_control_data +
 * dwc3_ep0_stall_and_restart, from both the XferNotReady handler and the reset
 * interrupt).
 *
 * Whether it ends in Set Stall comes from the state, not from the caller.
 */
static UDC_DWC3_COLD void udc_dwc3_ctrl_ep_recover(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct udc_dwc3_ep_data *const out0 = &cfg->ep_data_out[0];
    struct udc_dwc3_ep_data *const in0 = &cfg->ep_data_in[0];
    /*
     * Set Stall answers a control transfer in progress that cannot complete
     * (4.4.1/4.4.2 steps 2, 3, 3a, 5b, 6); a USB reset's "complete it" (4.1.2)
     * reaches it the same way. In progress = past the Setup phase (a recovery
     * already running froze the state it started from into _STALL or not), or a
     * stage live on either half. Once _STALL, a re-entry keeps it.
     */
    const bool in_progress =
        (priv->ctrl.state != UDC_DWC3_CTRL_IDLE && !udc_dwc3_ctrl_recovering(priv)) ||
        priv->ctrl.state == UDC_DWC3_CTRL_RECOVERING_STALL ||
        out0->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID ||
        in0->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID ||
        udc_dwc3_ep_cmd_busy(out0) || udc_dwc3_ep_cmd_busy(in0);
    const enum udc_dwc3_ctrl_state next = in_progress ? UDC_DWC3_CTRL_RECOVERING_STALL
                              : UDC_DWC3_CTRL_RECOVERING;

    if (priv->ctrl.state != next) {
        udc_dwc3_ctrl_state_set(dev, next);
    }
    udc_dwc3_ctrl_recover_continue(dev);
}

/*
 * An open Start Transfer (STARTING / START_UNKNOWN) proven refused: by its
 * Command Complete, by DEPCMD when that event is lost (udc_dwc3_ep_resolve_cmd()),
 * or at post time (udc_dwc3_depcmd_start_xfer(), non-control with buffers armed
 * only). The one handler for all three. No transfer started and no resource was
 * taken, so the ring is the driver's again. src and val name the evidence for
 * the log.
 *
 *   EP0 half: nothing is armed and no event will move the control transfer on:
 *             back to Step 1, or on with the recovery already doing so.
 *   other:    no new Start: CmdStatus 4'h1 means no transfer resource for the
 *             endpoint, which another Start does not cure. The endpoint leaves
 *             DALEPENA with its buffers parked, as a disable leaves them, and the
 *             stack is told. Work owed to the stack or host (pending) still runs.
 *             at_post: the ring is left armed for the heartbeat sweep's
 *             udc_dwc3_ep_recover() to park (DALEPENA clear with an armed ring is
 *             recovery owed): the poster may still hold a buffer linked in the
 *             stack's queue, and parking would link it into requeue_fifo too.
 */
static UDC_DWC3_COLD void udc_dwc3_ep_start_refused(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data,
                    const char *const src, const uint32_t val,
                    const bool at_post)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const bool is_ctrl = USB_EP_GET_IDX(ep_data->cfg.addr) == 0U;

    priv->diag.ctrl_start_fail++;
    /* Reported here, so udc_dwc3_depcmd()'s late CMDERR line stays quiet. */
    ep_data->diag.cmd_reported = true;

    LOG_ERR("EP%02x Start Transfer refused (%s 0x%08x), %s (%u so far)",
        ep_data->cfg.addr, src, val,
        is_ctrl ? "control transfer back to Setup"
             : "endpoint out of service, buffers parked for re-enable",
        priv->diag.ctrl_start_fail);

    udc_dwc3_ep_state_reset(ep_data);

    if (is_ctrl) {
        if (udc_dwc3_ctrl_recovering(priv)) {
            udc_dwc3_ctrl_recover_continue(dev);
        } else {
            udc_dwc3_ctrl_ep_recover(dev);
        }
        return;
    }

    udc_dwc3_dalepena_set(dev, ep_data->epn, false);

    /* The stack still counts the endpoint enabled: its enable had succeeded. */
    udc_submit_event(dev, UDC_EVT_ERROR, -EIO);

    if (at_post) {
        return;
    }

    udc_dwc3_ep_ring_release(ep_data);

    if (udc_dwc3_ep_recovery_owed(dev, ep_data)) {
        udc_dwc3_ep_recover(dev, ep_data);
    }
}

/*
 * Forget every endpoint's transfer state. Only for a core soft reset or a
 * controller disable, after which the controller holds nothing: no transfer to
 * end, no Setup state to return to. Not for a USB reset or disconnect: there the
 * controller is live and keeps its transfers (use udc_dwc3_end_all_transfers()).
 */
static UDC_DWC3_COLD void udc_dwc3_drop_xfer_state(const struct device *const dev,
                     const char *const reason)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    LOG_DBG("dropping all transfer state (%s)", reason);

    priv->diag.watchdog_ep = NULL;
    priv->diag.watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
    priv->ctrl.setup_pending = false;
    udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_IDLE);
    /* A U3 of the old session has no exit owed in the next (4.1.10). */
    priv->link.in_u3 = false;
    priv->link.u3_active_eps = 0U;

    /*
     * pending is forgotten too: the buffers are cancelled here, and a Clear Stall
     * or resume owed to the old session has no endpoint configuration left to
     * act on.
     */
    for (int i = 0; i < cfg->num_in_eps; i++) {
        udc_dwc3_ep_state_reset(&cfg->ep_data_in[i]);
        cfg->ep_data_in[i].xfer.pending = UDC_DWC3_EP_PEND_NONE;
        if (i > 0) {
            /* Release then return - the buffers were parked by the release. */
            udc_dwc3_ep_ring_release(&cfg->ep_data_in[i]);
            udc_dwc3_ep_return_parked(dev, &cfg->ep_data_in[i], UDC_DWC3_BUF_CANCELLED);
        }
    }
    for (int i = 0; i < cfg->num_out_eps; i++) {
        udc_dwc3_ep_state_reset(&cfg->ep_data_out[i]);
        cfg->ep_data_out[i].xfer.pending = UDC_DWC3_EP_PEND_NONE;
        if (i > 0) {
            udc_dwc3_ep_ring_release(&cfg->ep_data_out[i]);
            udc_dwc3_ep_return_parked(dev, &cfg->ep_data_out[i], UDC_DWC3_BUF_CANCELLED);
        }
    }
}

/*
 * End every transfer and take EP0 back to its Setup stage: the common first step
 * of a USB reset and a device-initiated disconnect (SPEC 3.30b):
 *
 *   4.1.2 Table 4-2 / 4.1.8 Table 4-7:
 *     "If a control transfer is still in progress, complete it and get the
 *     controller into the 'Setup a Control-Setup TRB / Start Transfer' state"
 *     "Issue a DEPENDXFER command for any active transfers (except for the
 *     default control endpoint 0)"
 *   4.1.2 only:
 *     "Issue a DEPCSTALL (ClearStall) command for any endpoint in STALL mode
 *     prior to the USB Reset (excluding control endpoints)"   -> clear_stall
 *
 * Each non-control endpoint goes through udc_dwc3_ep_recover(): End Transfer
 * (ForceRM) if a transfer holds a resource, then buffers returned as cancelled
 * and Clear Stall when owed.
 */
static UDC_DWC3_COLD void udc_dwc3_end_all_transfers(const struct device *const dev,
                             const bool clear_stall)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    for (uint32_t epn = 2U; epn < UDC_DWC3_MAX_EPN; epn++) {
        struct udc_dwc3_ep_data *ep_data;

        if (!_EPN_IS_VALID(cfg, epn)) {
            continue;
        }
        ep_data = _EP_DATA_FROM_EPN(cfg, epn);
        ep_data->xfer.pending |= UDC_DWC3_EP_PEND_DEQUEUE;
        if (clear_stall && ep_data->cfg.stat.halted) {
            ep_data->xfer.pending |= UDC_DWC3_EP_PEND_CLEAR_STALL;
        }
        udc_dwc3_ep_recover(dev, ep_data);
    }

    if (priv->ctrl.state != UDC_DWC3_CTRL_IDLE) {
        udc_dwc3_ctrl_ep_recover(dev);
    } else {
        udc_dwc3_ctrl_next(dev);
    }
}

/*
 * Core soft reset (DCTL.CSFTRST) and full event-ring reinitialisation.
 */
static UDC_DWC3_COLD int udc_dwc3_on_soft_reset(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    uint32_t reg;

    /*
     * Configure and reset the Device Controller.
     * TODO confirm that DWC_USB3_EN_LPM_ERRATA == 1.
     */
    reg = UDC_DWC3_DCTL_CSFTRST;
    reg |= FIELD_PREP(UDC_DWC3_DCTL_LPM_NYET_THRES_MASK, 15);
    udc_dwc3_dctl_write(base, reg);

    /*
     * Wait for CSftRst to clear, bounded so a core that never clears it cannot
     * hang the driver.
     *   Phase 1: reads with no delay; the core normally clears it here.
     *   Phase 2: sleep a tick per read. This also runs at runtime
     *   (udc_dwc3_controller_recover() -> init() -> here) on the work queue with
     *   the UDC mutex held, where a busy spin starves every thread, including
     *   the drain.
     */
    for (uint32_t i = 0; i < UDC_DWC3_CSFTRST_FAST_READS; i++) {
        if ((sys_read32(base + UDC_DWC3_DCTL) &
             UDC_DWC3_DCTL_CSFTRST) == 0U) {
            break;
        }
    }

    for (uint32_t i = 0; i < UDC_DWC3_CSFTRST_SLOW_TICKS; i++) {
        if ((sys_read32(base + UDC_DWC3_DCTL) &
             UDC_DWC3_DCTL_CSFTRST) == 0U) {
            break;
        }
        k_sleep(K_TICKS(1));
    }

    if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_CSFTRST) != 0U) {
        LOG_ERR("CSftRst still set after %u reads and %u ticks - the core "
            "did not complete its reset; continuing, but every register "
            "written from here is suspect",
            UDC_DWC3_CSFTRST_FAST_READS, UDC_DWC3_CSFTRST_SLOW_TICKS);
    }

    /* The core was just reset: no earlier command can still report completion. */
    udc_dwc3_drop_xfer_state(dev, "soft reset");

    /*
     * DEPCMD registers read as undefined again and no earlier command is active,
     * so the next command on each endpoint must skip the pre-poll. (The transfer
     * state, indices included, was dropped above.)
     */
    for (uint8_t i = 0; i < DEV_CFG(dev)->num_in_eps; i++) {
        DEV_CFG(dev)->ep_data_in[i].cmd.depcmd_last = UDC_DWC3_DEPCMD_NONE;
    }
    for (uint8_t i = 0; i < DEV_CFG(dev)->num_out_eps; i++) {
        DEV_CFG(dev)->ep_data_out[i].cmd.depcmd_last = UDC_DWC3_DEPCMD_NONE;
    }

    /*
     * SoC bus configuration. Register hygiene, not a fix: the bitfile's power-on
     * combination is undefined by the databook, and two independent vendor trees
     * ship the same GSBUSCFG0 for this controller.
     */
    LOG_INF("BUSCFG at reset: GSBUSCFG0=0x%08x GSBUSCFG1=0x%08x",
        sys_read32(base + UDC_DWC3_GSBUSCFG0),
        sys_read32(base + UDC_DWC3_GSBUSCFG1));

    /*
     * Log the three registers the driver leaves at their power-on values. The
     * databook's CSftRst exception list names them among the registers a core
     * soft reset does not clear.
     */
    LOG_INF("POR unpinned: GUSB2PHYCFG=0x%08x GUSB3PIPECTL=0x%08x GTXTHRCFG=0x%08x",
        sys_read32(base + UDC_DWC3_GUSB2PHYCFG),
        sys_read32(base + UDC_DWC3_GUSB3PIPECTL),
        sys_read32(base + UDC_DWC3_GTXTHRCFG));

    /*
     * Log the build parameters behind the U3/P3 settings the guide leaves to the
     * integration:
     *   GHWPARAMS0[1:0]    mode: 0 device, 1 host, 2 DRD (for DRD the
     *                      application sets GUSB3PIPECTL.SuspendEnable after init)
     *   GHWPARAMS1[25:24]  power options (2 = hibernation)
     *   GCTL               PwrDnScale (suspend_clk periods per 16 kHz tick)
     */
    {
        const uint32_t hw0 = sys_read32(base + UDC_DWC3_GHWPARAMS0);
        const uint32_t hw1 = sys_read32(base + UDC_DWC3_GHWPARAMS1);

        LOG_INF("HW params: GHWPARAMS0=0x%08x (mode %u) GHWPARAMS1=0x%08x "
            "(pwropt %u) GCTL=0x%08x", hw0, hw0 & 0x3U, hw1, (hw1 >> 24) & 0x3U,
            sys_read32(base + UDC_DWC3_GCTL));
    }

    sys_write32(UDC_DWC3_GSBUSCFG0_INCR16BRSTENA |
            UDC_DWC3_GSBUSCFG0_INCR8BRSTENA |
            UDC_DWC3_GSBUSCFG0_INCR4BRSTENA,
            base + UDC_DWC3_GSBUSCFG0);

    /* GSBUSCFG1 is not written: PipeTransLimit stays at power-on (3 on this part). */

    LOG_INF("BUSCFG programmed: GSBUSCFG0=0x%08x GSBUSCFG1=0x%08x",
        sys_read32(base + UDC_DWC3_GSBUSCFG0),
        sys_read32(base + UDC_DWC3_GSBUSCFG1));

    /* Disable multi-packet RX thresholding (GRXTHRCFG; databook 1.2.4 erratum). */
    reg = sys_read32(base + UDC_DWC3_GRXTHRCFG);
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

    /* GTXTHRCFG left unchanged: the erratum above is RX-only. */

    /* Read the chip identification */
    reg = sys_read32(base + UDC_DWC3_GCOREID);
    LOG_INF("event: coreid=0x%04lx rel=0x%04lx",
        FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg),
        FIELD_GET(UDC_DWC3_GCOREID_REL_MASK, reg));
    __ASSERT_NO_MSG(FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg) == 0x5533);

    /*
     * Clear GUSB2PHYCFG[15] (ULPIAutoRes): the PHY must not auto-resume in device
     * mode, and the reset value may be 1 (databook Table 4-1, Power-On or Soft
     * Reset Register Initialization).
     */
    sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG,
               UDC_DWC3_GUSB2PHYCFG_ULPIAUTORES);

    /*
     * Clear SusPHY and EnblSlpM, and leave them clear: "before issuing any
     * device endpoint command when operating in 2.0 speeds, disable this bit"
     * (GUSB2PHYCFG). The reset value may be 1. Nothing in this driver sets them
     * again, so udc_dwc3_depcmd() does not re-check them per command.
     */
    reg = sys_read32(base + UDC_DWC3_GUSB2PHYCFG);
    if ((reg & (UDC_DWC3_GUSB2PHYCFG_SUSPHY | UDC_DWC3_GUSB2PHYCFG_ENBLSLPM)) != 0U) {
        LOG_WRN("GUSB2PHYCFG had SusPHY/EnblSlpM set (0x%08x): cleared", reg);
        sys_write32(reg & ~(UDC_DWC3_GUSB2PHYCFG_SUSPHY | UDC_DWC3_GUSB2PHYCFG_ENBLSLPM),
                base + UDC_DWC3_GUSB2PHYCFG);
    }
    /*
     * Wait for the register file again here, not only in udc_dwc3_init(): this
     * function issues its own CSftRst after init() settled the core, and the FIFO
     * map below is the first GHWPARAMS read after it. Read too soon after CSftRst
     * clears, it can return RAM1_DEPTH=0 and mdwidth=0 and program every FIFO
     * to zero depth.
     */
    if (!udc_dwc3_wait_regfile_ready(dev)) {
        LOG_ERR("the register file is not out of reset after CSftRst, so every "
            "FIFO would be programmed to zero depth and the controller "
            "would stop moving data for good: abandoning the "
            "configuration rather than completing it");
        return -EIO;
    }

    /*
     * One-shot FIFO map. GRXFIFOSIZ0 and GTXFIFOSIZn partition a pool whose
     * total is fixed at synthesis (GHWPARAMS7.RAM1_DEPTH).
     */
    {
        const uint32_t hp7 = sys_read32(base + UDC_DWC3_GHWPARAMS7);
        const uint32_t ram1 = FIELD_GET(UDC_DWC3_GHWPARAMS7_RAM1_DEPTH_MASK, hp7);
        const uint32_t rx = sys_read32(base + UDC_DWC3_GRXFIFOSIZ(0));
        const uint32_t mdw = (sys_read32(base + UDC_DWC3_GHWPARAMS0) >> 8) & 0xFFU;
        uint32_t used = 0;

        /*
         * Refuse. The wait above should make this unreachable; if reached, the
         * core is still in reset, so abandon the configuration as the wait does.
         */
        if (ram1 == 0U || mdw == 0U) {
            LOG_ERR("FIFOMAP REFUSED: core reports RAM1_DEPTH=%u "
                "mdwidth=%u - the register file is not out of reset "
                "and every FIFO would be programmed to zero depth",
                ram1, mdw);
            return -EIO;
        }

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
         * Count TX only. RX0 lives in RAM2, a separate address space, so counting
         * it against the TX budget would understate the spare by the RX depth.
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

    /* Reset the read pointer together with the memset above; never separate them. */
    priv->evt.next = 0;
    udc_dwc3_drain_reset(&priv->evt.drain);
    /* Both stamps are valid from here, so no "is it meaningful yet" flags. */
    priv->evt.worker_exit_t0 = k_cycle_get_32();
    priv->evt.force_t0 = k_cycle_get_32();
    /* Prime every slot as consumed. */
    for (uint32_t i = 0; i < CONFIG_UDC_DWC3_EVENTS_NUM; i++) {
        cfg->evt_buf[i] = UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
    }

    /* Commit the priming before the controller is told where the buffer is. */
    udc_dwc3_trb_sync(&cfg->evt_buf[CONFIG_UDC_DWC3_EVENTS_NUM - 1]);

    sys_write32(HI32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_HI(0));
    sys_write32(LO32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_LO(0));
    sys_write32(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t), base + UDC_DWC3_GEVNTSIZ(0));
    LOG_INF("Event buffer size is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));

    {
        const uint32_t imod = sys_read32(base + UDC_DWC3_DEV_IMOD(0));
        const uint32_t imodi = FIELD_GET(UDC_DWC3_DEV_IMOD_DEVICE_IMODI_MASK, imod);

        LOG_INF("DEV_IMOD=0x%08x IMODI=%u IMODC=%u at reset - moderation %s",
            imod, imodi,
            (uint32_t)FIELD_GET(UDC_DWC3_DEV_IMOD_DEVICE_IMODC_MASK, imod),
            imodi != 0U ? "ENABLED" : "off");

        /* Program the moderation interval (Table 1-90). */
        sys_write32(FIELD_PREP(UDC_DWC3_DEV_IMOD_DEVICE_IMODI_MASK,
                       UDC_DWC3_DEV_IMOD_INTERVAL_1MS),
                base + UDC_DWC3_DEV_IMOD(0));
        LOG_INF("DEV_IMOD programmed to 0x%08x (IMODI=%u = %u us)",
            sys_read32(base + UDC_DWC3_DEV_IMOD(0)),
            UDC_DWC3_DEV_IMOD_INTERVAL_1MS,
            UDC_DWC3_DEV_IMOD_INTERVAL_1MS / 4U);
    }

    /* Log the address and whether it meets the size-alignment rule. */
    if (((uintptr_t)cfg->evt_buf &
         (CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) - 1)) != 0) {
        LOG_ERR("event buffer at %p is NOT aligned to its size (%u bytes): "
            "the controller's wrap will not match this driver's",
            (void *)cfg->evt_buf,
            (unsigned int)(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t)));
    } else {
        LOG_INF("Event buffer at %p, size-aligned", (void *)cfg->evt_buf);
    }

    /* Last setup step, after GEVNTADR/GEVNTSIZ: the count starts at 0. */
    udc_dwc3_gevntcount_enable(base);

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

    /* Number of USB3 packets the device can receive at once. */
    reg = sys_read32(base + UDC_DWC3_DCFG);
    reg &= ~UDC_DWC3_DCFG_NUMP_MASK;
    reg |= FIELD_PREP(UDC_DWC3_DCFG_NUMP_MASK, 4);
    sys_write32(reg, base + UDC_DWC3_DCFG);

    /* Enable reception of all USB events this controller actually defines. */
    reg = 0;
    /*
     * Not enabled: VNDRDEVTSTRCVED (the driver ignores it and the databook says
     * not to use the feature). EvntOverflowEn, CmdCmpltEn and InactTimeoutRcvedEn
     * do not exist in this controller's DEVTEN.
     */
    reg |= UDC_DWC3_DEVTEN_ERRTICERREN;
    /*
     * HibernationReqEvtEn stays off: this driver implements no hibernation, and
     * the event obliges software "to start the hibernation process" (3.3.2).
     */
    reg |= UDC_DWC3_DEVTEN_WKUPEVTEN;
    /*
     * Link state change, USB Reset and Connection Done: the controller vendor's
     * recommended minimum DEVTEN set.
     */
    reg |= UDC_DWC3_DEVTEN_ULSTCNGEN;
    reg |= UDC_DWC3_DEVTEN_CONNECTDONEEN;
    reg |= UDC_DWC3_DEVTEN_USBRSTEN;
    reg |= UDC_DWC3_DEVTEN_DISCONNEVTEN;
    sys_write32(reg, base + UDC_DWC3_DEVTEN);

    /* Configure control endpoints */
    udc_dwc3_depcmd_start_config(dev, true);

    return 0;
}

/*
 * USBRST handler: return the device to the default state.
 */
static UDC_DWC3_COLD void udc_dwc3_on_usb_reset(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    LOG_DBG("Going through DWC3 reset logic");

    /*
     * A bus reset starts a new configuration: let the next endpoint enable
     * reassign the transfer-resource pool, and reset epcfg.first_ep, since a later
     * session may enable a different endpoint first.
     */
    priv->epcfg.pool_assigned = false;
    priv->epcfg.first_ep = 0U;

    /* SPEC 3.30b 4.1.2, with the same primitives every other path uses. */
    udc_dwc3_end_all_transfers(dev, true);

    /* "Set DevAddr to 0". */
    udc_dwc3_set_address(dev, 0);
}

/*
 * CONNECTDONE handler: adopt the negotiated speed and resize EP0.
 */
static UDC_DWC3_COLD void udc_dwc3_on_connect_done(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    int mps = 0;

    /* Adjust parameters against the connection speed */
    switch (udc_dwc3_connect_speed(base)) {
    case UDC_BUS_SPEED_FS:
    case UDC_BUS_SPEED_HS:
        mps = 64;
        break;
    case UDC_BUS_SPEED_SS:
        mps = 512;
        break;
    default:
        break;
    }
    __ASSERT_NO_MSG(mps != 0);

    /* Reconfigure control endpoints connection speed */
    udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT)->mps = mps;
    udc_get_ep_cfg(dev, USB_CONTROL_EP_IN)->mps = mps;
    udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0], true);
    udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_out[0], true);

    /* GTXFIFOSIZn is left alone; the databook states this is the normal case. */

    /*
     * Report the reset here, not at USB_RESET: the speed-related registers are
     * only valid after CONNECT_DONE.
     */
    udc_submit_event(dev, UDC_EVT_RESET, 0);
}

/*
 * New transfer-resource pool generation (Table 4-5 SetConfiguration), run by
 * udc_dwc3_ep_enable() once per bus reset, on the first non-control endpoint
 * enabled: recover every non-control endpoint, re-initialise physical EP1's TX
 * FIFO allocation (DEPCFG Modify), and reassign the non-control transfer
 * resources (DEPSTARTCFG).
 */
static UDC_DWC3_COLD void udc_dwc3_on_set_config_or_interface(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;

    LOG_DBG("SetConfiguration or SetInterface extra init");

    for (int i = 1; i < cfg->num_in_eps; i++) {
        udc_dwc3_ep_recover(dev, &cfg->ep_data_in[i]);
    }
    for (int i = 1; i < cfg->num_out_eps; i++) {
        udc_dwc3_ep_recover(dev, &cfg->ep_data_out[i]);
    }

    udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0], true);

    /*
     * DEPSTARTCFG reassigns the resources of every transfer the recovery may
     * have left open (an End still executing, a refused End, a Start for an
     * armed ring), and its reset would orphan that command's outcome. Skip it
     * then: the pool keeps its current assignment, which INVARIANT 4 never
     * over-allocates, and the next bus reset tries again.
     */
    for (uint32_t epn = 2U; epn < UDC_DWC3_MAX_EPN; epn++) {
        if (_EPN_IS_VALID(cfg, epn) &&
            _EP_DATA_FROM_EPN(cfg, epn)->xfer.state != UDC_DWC3_EP_IDLE) {
            LOG_WRN("EP%02x still %s: DepStartConfig skipped",
                _EP_DATA_FROM_EPN(cfg, epn)->cfg.addr,
                udc_dwc3_ep_state_name(_EP_DATA_FROM_EPN(cfg, epn)->xfer.state));
            return;
        }
    }

    udc_dwc3_depcmd_start_config(dev, false);
}

/*
 * Release the half whose stage just completed, to match the controller, which
 * freed the transfer resource when it generated XferComplete (3.2.2.2). This
 * keeps the invariant udc_dwc3_ctrl_ep_recover() relies on: on EP0/EP1,
 * "xferrscidx valid" means "a transfer is live". A half with an End Transfer
 * outstanding is left for that End's completion.
 */
static void udc_dwc3_ctrl_release(struct udc_dwc3_ep_data *const h)
{
    if (!udc_dwc3_ep_is_ending(h)) {
        udc_dwc3_ep_state_reset(h);
    }
}

/* 4.4.1/4.4.2 step 2: the SETUP has arrived in the driver's buffer. */
static void udc_dwc3_ctrl_setup_done(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    memcpy(&priv->ctrl.setup, cfg->setup_buf, sizeof(priv->ctrl.setup));
    udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_SETUP_DONE);
    priv->diag.ctrl_setup_done++;

    /*
     * Write SET_ADDRESS to DCFG at the SETUP, not the status stage: the controller
     * applies the new address itself after the status stage.
     */
    if (priv->ctrl.setup.bmRequestType == USB_REQTYPE_TYPE_STANDARD &&
        priv->ctrl.setup.bRequest == USB_SREQ_SET_ADDRESS) {
        udc_dwc3_set_address(dev, sys_le16_to_cpu(priv->ctrl.setup.wValue));
    }

    /*
     * Hand the 8 bytes to the stack. This returns any stage buffers still queued
     * on either half for an earlier transfer, and copies the SETUP into the
     * stack's SETUP buffer, or caches it until the stack queues one.
     */
    udc_setup_received(dev, &priv->ctrl.setup);
    udc_dwc3_ctrl_next(dev);
}

/* 4.4.2 step 4: the data stage has retired on its endpoint. */
static void udc_dwc3_ctrl_data_done(const struct device *const dev,
                    struct udc_dwc3_ep_data *const h,
                    const uint32_t sts, const uint32_t residual)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct net_buf *const buf = udc_buf_get(&h->cfg);


    if (buf == NULL || !udc_get_buf_info(buf)->data) {
        LOG_WRN_RATELIMIT("EP%02x data stage retired with no data buffer queued",
                  h->cfg.addr);
        if (buf != NULL) {
            udc_dwc3_buf_return(dev, buf, UDC_DWC3_BUF_ABANDONED);
        }
        udc_dwc3_ctrl_ep_recover(dev);
        return;
    }

    /*
     * SetupPending: "this control transfer was aborted on the USB bus and the
     * host did not complete the data stage." The flow still continues to
     * XferNotReady(Status) (Figure 4-2), where the failed data stage gets
     * Set Stall (step 6).
     */
    if (sts == UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING) {
        priv->diag.ctrl_setup_pending++;
        priv->ctrl.setup_pending = true;
        /*
         * Table 4-14 step 8: if a new SETUP arrives before the data goes out, the
         * controller skips the data TRBs; software must reclaim them (HWO=1) and
         * flush the TxFIFO. XferComplete released the transfer resource, so both
         * TRBs are the driver's again.
         */
        if (USB_EP_DIR_IS_IN(h->cfg.addr)) {
            udc_dwc3_ctrl_reclaim_half(dev, h);
        }
        udc_dwc3_buf_return(dev, buf, UDC_DWC3_BUF_ABANDONED);
        udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_DATA_DONE);
        return;
    }

    if (!USB_EP_DIR_IS_IN(h->cfg.addr)) {
        /* What the hardware actually received, not what the host declared. */
        const uint32_t mps = USB_MPS_EP_SIZE(h->cfg.mps);
        const uint32_t programmed = (mps != 0U) ? ROUND_UP(buf->size, mps) : buf->size;
        const uint32_t received = (programmed > residual) ? (programmed - residual) : 0U;

        buf->len = MIN(received, MIN((uint32_t)buf->size,
                         (uint32_t)sys_le16_to_cpu(priv->ctrl.setup.wLength)));
    }

    udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_DATA_DONE);
    udc_dwc3_buf_return(dev, buf, UDC_DWC3_BUF_DONE);
    udc_dwc3_ctrl_next(dev);
}

/*
 * Update the DCTL U1/U2 bits from the request whose status stage just completed.
 * Called only once the stack has accepted the request (status stage completed
 * normally); SuperSpeed standard device requests only.
 *   - AcceptU1/U2Ena: set after SetConfiguration.
 *   - InitU1/U2Ena: set on SetFeature(U1/U2_ENABLE), cleared on
 *     ClearFeature(U1/U2_ENABLE).
 * Hardware clears all four on USB reset; a disconnect clears them too.
 */
static UDC_DWC3_COLD void udc_dwc3_ctrl_apply_link_pm(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const struct usb_setup_packet *const setup = &priv->ctrl.setup;
    const uint16_t value = sys_le16_to_cpu(setup->wValue);
    const bool ss = udc_dwc3_connect_speed(base) == UDC_BUS_SPEED_SS;
    uint32_t bit;

    if (!ss || setup->RequestType.type != USB_REQTYPE_TYPE_STANDARD ||
        setup->RequestType.recipient != USB_REQTYPE_RECIPIENT_DEVICE) {
        return;
    }

    switch (setup->bRequest) {
    case USB_SREQ_SET_CONFIGURATION:
        if (value != 0U) {
            udc_dwc3_dctl_update(base, 0U, UDC_DWC3_DCTL_ACCEPTU1ENA |
                                   UDC_DWC3_DCTL_ACCEPTU2ENA);
        } else {
            udc_dwc3_dctl_update(base, UDC_DWC3_DCTL_ACCEPTU1ENA | UDC_DWC3_DCTL_INITU1ENA |
                           UDC_DWC3_DCTL_ACCEPTU2ENA | UDC_DWC3_DCTL_INITU2ENA, 0U);
        }
        break;
    case USB_SREQ_SET_FEATURE:
    case USB_SREQ_CLEAR_FEATURE:
        if (value == USB_SFS_U1_ENABLE) {
            bit = UDC_DWC3_DCTL_INITU1ENA;
        } else if (value == USB_SFS_U2_ENABLE) {
            bit = UDC_DWC3_DCTL_INITU2ENA;
        } else {
            break;
        }
        if (setup->bRequest == USB_SREQ_SET_FEATURE) {
            udc_dwc3_dctl_update(base, 0U, bit);
        } else {
            udc_dwc3_dctl_update(base, bit, 0U);
        }
        break;
    default:
        break;
    }
}

/*
 * 4.4.1 step 5 / 4.4.2 step 8: the status stage has retired (SetupPending or
 * not); go back to Step 1. The status buffer is returned as completed either way,
 * as the reference driver does: the request was served, only the host's ACK is
 * in doubt.
 */
static void udc_dwc3_ctrl_status_done(const struct device *const dev,
                      struct udc_dwc3_ep_data *const h,
                      const uint32_t sts)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct net_buf *const buf = udc_buf_get(&h->cfg);

    if (buf != NULL) {
        buf->len = 0;
        udc_dwc3_buf_return(dev, buf, UDC_DWC3_BUF_DONE);
    }
    priv->diag.ctrl_status_done++;
    if (sts == UDC_DWC3_TRB_STATUS_TRBSTS_OK) {
        udc_dwc3_ctrl_apply_link_pm(dev);
    }
    udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_IDLE);
    udc_dwc3_ctrl_next(dev);
}

/*
 * XferComplete on EP0 or EP1. Attributed by where Figure 4-2 says the transfer
 * is, never by what the driver remembers arming:
 *   Setup node -> the SETUP; data node -> the data stage (on bmRequestType's
 *   endpoint); status node -> the status stage.
 */
static void udc_dwc3_on_ctrl(const struct device *const dev, struct udc_dwc3_ep_data *const h,
                 const bool is_in)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const bool ending = udc_dwc3_ep_is_ending(h);
    struct udc_dwc3_trb t;
    uint32_t status;

    if (priv->diag.watchdog_ep == h) {
        priv->diag.watchdog_ep = NULL;
        priv->diag.watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
    }

    /* Nothing live on this half: a late or repeated event for a transfer already over. */
    if (h->xfer.state == UDC_DWC3_EP_IDLE) {
        LOG_DBG("EP%02x XferComplete with no transfer live: ignored", h->cfg.addr);
        return;
    }

    /*
     * Read the write-back before anything can rearm this half's ring, ctrl
     * before status. HWO still set at XferComplete means the write-back is not
     * visible: the status is read anyway, after ctrl, and used as it reads.
     * A chained ZLP TRB still owned keeps the first TRB's TRBSTS (OK here),
     * which is also what its programmed status of 0 reads as.
     */
    (void)udc_dwc3_trb_snapshot(&h->trb_buf[0], &t);
    status = t.status;
    if ((t.ctrl & UDC_DWC3_TRB_CTRL_CHN) != 0U &&
        (status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK) == UDC_DWC3_TRB_STATUS_TRBSTS_OK &&
        udc_dwc3_trb_snapshot(&h->trb_buf[1], &t)) {
        status = (status & UDC_DWC3_TRB_STATUS_BUFSIZ_MASK) |
             (t.status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK);
    }

    udc_dwc3_ctrl_release(h);
    priv->ctrl.setup_pending = false;

    /* A stage of the transfer being taken back to Step 1: the recovery owns it. */
    if (udc_dwc3_ctrl_recovering(priv) || ending) {
        udc_dwc3_ctrl_recover_continue(dev);
        return;
    }

    switch (priv->ctrl.state) {
    case UDC_DWC3_CTRL_IDLE:
        if (!is_in) {
            udc_dwc3_ctrl_setup_done(dev);
            return;
        }
        break;
    case UDC_DWC3_CTRL_DATA_ARMED:
        if (is_in == udc_dwc3_ctrl_dir_in(priv)) {
            udc_dwc3_ctrl_data_done(dev, h,
                        status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK,
                        FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, status));
            return;
        }
        break;
    case UDC_DWC3_CTRL_STATUS_ARMED:
        if (is_in == udc_dwc3_ctrl_status_is_in(priv)) {
            udc_dwc3_ctrl_status_done(dev, h, status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK);
            return;
        }
        break;
    default:
        break;
    }

    /*
     * A transfer was live on this half but not the one Figure 4-2 says is in
     * progress: driver and controller disagree on where the control transfer is.
     * Back to Step 1.
     */
    LOG_WRN_RATELIMIT("EP%02x XferComplete (TRB status 0x%08x) in control state %u: "
              "back to Setup", h->cfg.addr, status, (unsigned int)priv->ctrl.state);
    udc_dwc3_ctrl_ep_recover(dev);
}

/*
 * Wait, bounded, for DGCMD.CmdAct to clear. Returns false if it is still set.
 * SPEC 3.30b DGCMD bit 10 CMDACT: software sets it to start a generic command;
 * the controller clears it once the command has executed.
 */
static UDC_DWC3_COLD bool udc_dwc3_dgcmd_wait_idle(const mm_reg_t base)
{
    uint32_t polls = 0;

    /*
     * Reads only, no delay: callers run on the drain thread or the UDC work
     * queue, mostly with the UDC mutex held, where a delayed poll would stall
     * every other UDC path. A generic command retires in microseconds; one
     * still active after UDC_DWC3_DGCMD_POLL_MAX reads is reported, not waited
     * on.
     */
    while ((sys_read32(base + UDC_DWC3_DGCMD) & UDC_DWC3_DGCMD_ACT) != 0U) {
        if (++polls >= UDC_DWC3_DGCMD_POLL_MAX) {
            return false;
        }
    }

    return true;
}

/*
 * Issue one device generic command (SPEC 3.30b 3.2.1, Table 3-2). The only place
 * DGCMDPAR and DGCMD are written.
 *
 * Parameter and command are written together under dgcmd_lock, since both the
 * drain thread and the work queue issue generic commands. A new command is never
 * written over one still executing (CmdAct set): bounded wait outside the lock,
 * then re-check under it. With wait, CmdAct is polled again after issuing.
 *
 * Returns 0, -EBUSY (previous command never cleared; nothing issued), or
 * -ETIMEDOUT (issued, still executing after the bounded wait).
 */
static UDC_DWC3_COLD int udc_dwc3_dgcmd(const struct device *const dev, const uint32_t cmd,
                    const uint32_t param, const bool wait)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    k_spinlock_key_t key;

    if (!udc_dwc3_dgcmd_wait_idle(base)) {
        return -EBUSY;
    }

    key = k_spin_lock(&priv->dgcmd_lock);
    if ((sys_read32(base + UDC_DWC3_DGCMD) & UDC_DWC3_DGCMD_ACT) != 0U) {
        k_spin_unlock(&priv->dgcmd_lock, key);
        return -EBUSY;
    }
    sys_write32(param, base + UDC_DWC3_DGCMDPAR);
    sys_write32(cmd | UDC_DWC3_DGCMD_ACT, base + UDC_DWC3_DGCMD);
    k_spin_unlock(&priv->dgcmd_lock, key);

    if (wait && !udc_dwc3_dgcmd_wait_idle(base)) {
        return -ETIMEDOUT;
    }

    return 0;
}

/*
 * Flush one endpoint's TxFIFO (DGCMD 09h "Selected FIFO Flush", Table 3-2).
 *
 * Required after an aborted control IN transfer (4.4.2 step 8: on a SETUP
 * mid-transfer, "reclaim the TRBs with HWO=1 ... and flush the TxFIFO").
 * Reclaiming alone leaves bytes already staged for the skipped IN stage in the
 * FIFO, and they would go out at the head of the next one.
 */
static UDC_DWC3_COLD void udc_dwc3_fifo_flush_tx(const struct device *const dev, const uint8_t fifo)
{
    /*
     * Wait for completion: the next control stage is armed right after this
     * returns, and must not race the bytes being discarded.
     */
    const int ret = udc_dwc3_dgcmd(dev, UDC_DWC3_DGCMD_FIFOFLUSHONE,
                       UDC_DWC3_DGCMD_FIFOFLUSH_TX |
                       FIELD_PREP(UDC_DWC3_DGCMD_FIFOFLUSH_NUM_MASK, fifo),
                       true);

    if (ret == -EBUSY) {
        LOG_ERR_RATELIMIT("a generic command stayed active over %u reads; TxFIFO "
                  "%u not flushed", UDC_DWC3_DGCMD_POLL_MAX, fifo);
    } else if (ret == -ETIMEDOUT) {
        LOG_ERR_RATELIMIT("TxFIFO %u flush did not complete in %u reads", fifo,
                  UDC_DWC3_DGCMD_POLL_MAX);
    }
}

/*
 * True if the endpoint has an "active transfer" in the 4.1.10 sense: it holds a
 * transfer resource (RUNNING, or a Start whose outcome is still open). On EP0
 * only a data or status stage counts: a Setup TRB does not make it active (4.9.1)
 * and a SETUP is never flow-controlled, so no ERDY is owed for it.
 */
static UDC_DWC3_COLD bool udc_dwc3_ep_active_for_u3(const struct udc_dwc3_data *const priv,
                      const uint32_t epn,
                      const struct udc_dwc3_ep_data *const ep_data)
{
    if (ep_data->xfer.state != UDC_DWC3_EP_RUNNING &&
        ep_data->xfer.state != UDC_DWC3_EP_STARTING &&
        ep_data->xfer.state != UDC_DWC3_EP_START_UNKNOWN) {
        return false;
    }

    /* EP0 only in a data/status stage; not while a recovery is ending it. */
    return epn >= 2U ||
           (priv->ctrl.state != UDC_DWC3_CTRL_IDLE && !udc_dwc3_ctrl_recovering(priv));
}

/*
 * Link and bus events - the only place the driver reacts to link state
 * (SPEC 3.30b):
 *
 *   USB Reset        4.1.2  back to the default state, DevAddr 0
 *   Connect Done     4.1.3  adopt the speed, resize EP0
 *   Disconnect       4.1.7  clear the U1/U2 enables, set DCTL[8:5] to 5
 *                           (Rx.Detect)
 *   Link State Chg   3.3.2  SS only; not raised on exit from HOT_RESET/POLL or
 *                           entry to RECOVERY, so a U0 event also ends every
 *                           Recovery
 *   Erratic error    3.3.2  SS: controller reset required; HS/FS: only a soft
 *                           disconnect recovers. Both via
 *                           udc_dwc3_controller_recover()
 *   Buffer overflow  3.3.2  device events after it may be lost; endpoint events
 *                           are not
 *   Wakeup, Suspend         nothing to do (no remote wakeup, no hibernation)
 *
 * SPEC 3.30b 4.1.10, Initialization after U3 Exit: on U3 -> U0, issue
 * "Set Endpoint NRDY" on every endpoint that had an active transfer before U3
 * entry and still has one, so it sends ERDY. Why: an ERDY sent as the host's
 * LGO_U3 arrived is lost on the host, the device keeps no record of it, and the
 * transfer would wait forever. Use only for U3 exit.
 */
static UDC_DWC3_COLD void udc_dwc3_link_event(const struct device *const dev, const uint32_t evt)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t type = evt & UDC_DWC3_EVT_MASK;
    const uint32_t link = FIELD_GET(UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK, evt);
    const bool ss_link = type == UDC_DWC3_DEVT_ULSTCHNG &&
                 (evt & UDC_DWC3_DEVT_EVTINFO_SS) != 0U;
    uint32_t owed = 0U;

    /*
     * 4.1.10 bookkeeping, done only here. The exit is an SS U3 followed by U0
     * (Recovery in between is not reported).
     *   - SS U3: record the endpoints active now; a repeated U3 does not
     *     re-record.
     *   - SS U0 after a recorded U3: those endpoints are owed Set Endpoint NRDY.
     *   - Anything else cancels a pending exit: non-SS events (On/Sleep/Suspend
     *     share the encoding but are never U3), other link states, USB reset,
     *     disconnect, and event overflow (a later U0 may not be this U3's exit).
     * A U0 without a preceding U3 (Recovery, U1/U2 exit) never gets the command.
     */
    if (ss_link && link == FIELD_GET(UDC_DWC3_DSTS_USBLNKST_MASK,
                     UDC_DWC3_DSTS_USBLNKST_USB3_U3)) {
        if (!priv->link.in_u3) {
            priv->link.u3_active_eps = 0U;
            for (uint32_t epn = 0U; epn < UDC_DWC3_MAX_EPN; epn++) {
                if (_EPN_IS_VALID(cfg, epn) &&
                    udc_dwc3_ep_active_for_u3(priv, epn, _EP_DATA_FROM_EPN(cfg, epn))) {
                    priv->link.u3_active_eps |= BIT(epn);
                }
            }
            priv->link.in_u3 = true;
        }
    } else if (type == UDC_DWC3_DEVT_ULSTCHNG || type == UDC_DWC3_DEVT_USBRST ||
           type == UDC_DWC3_DEVT_DISCONNEVT || type == UDC_DWC3_DEVT_EVNTOVERFLOW) {
        if (ss_link && link == FIELD_GET(UDC_DWC3_DSTS_USBLNKST_MASK,
                         UDC_DWC3_DSTS_USBLNKST_USB3_U0) && priv->link.in_u3) {
            owed = priv->link.u3_active_eps;
        }
        priv->link.in_u3 = false;
        priv->link.u3_active_eps = 0U;
    }

    switch (type) {
    case UDC_DWC3_DEVT_USBRST:
        udc_dwc3_on_usb_reset(dev);
        break;
    case UDC_DWC3_DEVT_CONNECTDONE:
        udc_dwc3_on_connect_done(dev);
        break;
    case UDC_DWC3_DEVT_DISCONNEVT:
        /* Any Endpoint Command Complete still outstanding is not coming. */
        udc_dwc3_end_all_transfers(dev, true);
        /*
         * The device is no longer configured: drop the U1/U2 permissions, as
         * a USB reset clears them in hardware (DCTL) and SetConfiguration(0)
         * does in udc_dwc3_ctrl_apply_link_pm(). A new link must not enter or
         * accept U1/U2 before the host enables them again.
         */
        udc_dwc3_dctl_update(base, UDC_DWC3_DCTL_ACCEPTU1ENA | UDC_DWC3_DCTL_INITU1ENA |
                       UDC_DWC3_DCTL_ACCEPTU2ENA | UDC_DWC3_DCTL_INITU2ENA, 0U);
        udc_dwc3_dctl_link_request(base, UDC_DWC3_DCTL_ULSTCHNGREQ_RXDETECT);
        break;
    case UDC_DWC3_DEVT_ULSTCHNG:
        /* Only endpoints recorded at U3 that are still active now. */
        for (uint32_t epn = 0U; epn < UDC_DWC3_MAX_EPN; epn++) {
            if ((owed & BIT(epn)) == 0U || !_EPN_IS_VALID(cfg, epn) ||
                !udc_dwc3_ep_active_for_u3(priv, epn, _EP_DATA_FROM_EPN(cfg, epn))) {
                continue;
            }

            /* "Parameter[4:0]: Physical Endpoint Number" */
            if (udc_dwc3_dgcmd(dev, UDC_DWC3_DGCMD_SET_EP_NRDY, epn & 0x1fU,
                       true) == -EBUSY) {
                LOG_ERR_RATELIMIT("a generic command stayed active; Set Endpoint "
                          "NRDY for physical endpoint %u not issued", epn);
                continue;
            }
            priv->diag.u3_exit_nrdy++;
            LOG_INF("U3 exit: Set Endpoint NRDY on physical endpoint %u (%u total)",
                epn, priv->diag.u3_exit_nrdy);
        }
        break;
    case UDC_DWC3_DEVT_ERRTICERR:
        /*
         * UTMI+: phy_rxvalid/phy_rxactive asserted for >= 2 ms; SS: the PIPE did
         * not answer a PHY command. Rate-limited, since it repeats until the
         * reset. The reset sleeps, so the heartbeat runs it.
         */
        LOG_ERR_RATELIMIT("DEVT_ERRTICERR: PHY erratic error - resetting "
            "the controller");
        udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
        if (priv->run.state == UDC_DWC3_RUN) {
            /* A recovery already under way is the reset this asks for. */
            priv->run.state = UDC_DWC3_RUN_RESET_OWED;
        }
        break;
    case UDC_DWC3_DEVT_EVNTOVERFLOW:
        /* The only line for this event: the generic banner is suppressed. */
        LOG_ERR_RATELIMIT("evt ring ovfl");
        break;
    default:
        /* WKUPEVT, SUSPEND: nothing owed. */
        break;
    }
}

/*
 * XferNotReady on EP0 or EP1: the host asks for a control stage. Implements the
 * transitions of SPEC 3.30b 4.4.1/4.4.2 (Figure 4-2). Each event is either:
 *   - the expected edge: advance;
 *   - not an edge from the current state: ignore (hardware discards unexpected
 *     data/status stages when no SETUP was received);
 *   - a databook error case: back to Step 1 via udc_dwc3_ctrl_ep_recover().
 */
static void udc_dwc3_on_ctrl_xnr(const struct device *const dev, const uint32_t evt,
                 const bool is_in)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const uint32_t stage = evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK;
    struct udc_dwc3_ep_data *const out0 = &cfg->ep_data_out[0];


    if (udc_dwc3_ctrl_recovering(priv) ||
        (stage != UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA &&
         stage != UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS)) {
        return;
    }

    /*
     * Step 2: XferNotReady (Data/Status) before the Setup XferComplete -> Set
     * Stall. The Setup TRB is already armed, so nothing else; state stays at
     * Setup. "Before the XferComplete" is read from the hardware (Setup TRB
     * still HWO), not from drain order: once that TRB has retired, the SETUP's
     * XferComplete is behind this event, and a stall would hit the new request.
     */
    if (priv->ctrl.state == UDC_DWC3_CTRL_IDLE) {
        if (out0->xfer.state == UDC_DWC3_EP_RUNNING &&
            (out0->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
            priv->diag.ctrl_stall_issued++;
            (void)udc_dwc3_depcmd_set_stall(dev, out0);
        }
        return;
    }

    if (stage == UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS) {
        const bool due = priv->ctrl.state == UDC_DWC3_CTRL_DATA_DONE ||
                 (priv->ctrl.state == UDC_DWC3_CTRL_SETUP_DONE &&
                  !udc_dwc3_ctrl_three_stage(priv));

        if (!due) {
            return;
        }

        /*
         * Step 6: if the data stage failed, Set Stall and go back to Step 1.
         * A data stage the host abandoned (SetupPending) counts as failed.
         */
        if (priv->ctrl.setup_pending || is_in != udc_dwc3_ctrl_status_is_in(priv)) {
            udc_dwc3_ctrl_ep_recover(dev);
            return;
        }

        /* Steps 4 / 7: the status stage is due; arm it when the stack's buffer is there. */
        udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_STATUS_READY);
        udc_dwc3_ctrl_next(dev);
        return;
    }

    /* XferNotReady(Data). */
    switch (priv->ctrl.state) {
    case UDC_DWC3_CTRL_SETUP_DONE:
        /*
         * Error: a data stage on a request without one (4.4.1 step 3), or in
         * the wrong direction for bmRequestType (4.4.2 step 3a). Otherwise the
         * host is just early; the stack's buffer arms the stage.
         */
        if (!udc_dwc3_ctrl_three_stage(priv) || is_in != udc_dwc3_ctrl_dir_in(priv)) {
            udc_dwc3_ctrl_ep_recover(dev);
        }
        return;
    case UDC_DWC3_CTRL_DATA_ARMED:
        /* 3a: wrong direction - End the started data stage, Set Stall. 3b: ignore. */
        if (is_in != udc_dwc3_ctrl_dir_in(priv)) {
            udc_dwc3_ctrl_ep_recover(dev);
        }
        return;
    case UDC_DWC3_CTRL_DATA_DONE:
        /*
         * Step 5b: the host sent more data than wLength. (5a, the closing ZLP,
         * never gets here: OUT data TRB BUFSIZ is rounded up to wMaxPacketSize
         * in udc_dwc3_trb_ctrl_out(), so the ZLP lands in it.)
         */
        udc_dwc3_ctrl_ep_recover(dev);
        return;
    default:
        return;
    }
}

/*
 * Decode the completion status of a retired TRB.
 */
static void udc_dwc3_on_xfer_done(const struct udc_dwc3_trb *const trb)
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
        LOG_ERR_RATELIMIT("Invalid TRB status: 0x%08lx",
                  (trb->status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK));
        break;
    }
}


/*
 * Drain every TRB the controller has finished with on one endpoint.
 */
static uint32_t udc_dwc3_drain_completed(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct net_buf *buf;
    uint32_t drained = 0U;
    int ret;

    while (true) {
        struct udc_dwc3_trb trb;

        /* -EBUSY (controller still owns it) or -ENOBUFS (empty slot): done. */
        ret = udc_dwc3_pop_trb(ep_data, &buf, &trb);
        if (ret != 0) {
            break;
        }

        LOG_DBG("XFER_DONE_NORM: EP%02x, data %p",
            ep_data->cfg.addr, (void *)buf->data);

        udc_dwc3_on_xfer_done(&trb);

        /* Liveness proxy for the SETUP watchdog - see nonctrl_done. */
        priv->diag.nonctrl_done++;
        ep_data->diag.n_retire++;


        /*
         * A failed post (usbd queue full) is counted, not logged here; the pass
         * reports it once. The buffer is already on the stack's list, which usbd
         * drains on every message it handles.
         */
        if (udc_dwc3_buf_return(dev, buf, UDC_DWC3_BUF_DONE) != 0) {
            priv->evt.post_fail++;
        }

        drained++;
    }

    /* Ring slots are free: one kick lets the endpoint work queue more buffers. */
    if (drained > 0U) {
        k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
    }

    return drained;
}

/*
 * Transfer completion on a non-control endpoint.
 */
static void udc_dwc3_on_xfer_done_nonctrl(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const bool complete)
{
    (void)udc_dwc3_drain_completed(dev, ep_data);

    /*
     * XferComplete and XferInProgress mean opposite things about the transfer
     * resource (databook 3.2.2.2): only XferComplete means the controller
     * released it.
     */
    if (complete) {
        /*
         * Reset to idle only from RUNNING with nothing still armed. A skipped
         * event slot can be filled by a late write and read one ring wrap later,
         * for a transfer that ended long ago; the event word has no sequence
         * field to tell it from a fresh one. With a Start or End open (R2) its
         * own completion or the resolver settles the endpoint; IDLE here would
         * let a Start follow an End still executing (3.2.2.7).
         */
        if (ep_data->xfer.state != UDC_DWC3_EP_RUNNING) {
            LOG_WRN_RATELIMIT("EP%02x XferComplete ignored: endpoint is %s",
                      ep_data->cfg.addr,
                      udc_dwc3_ep_state_name(ep_data->xfer.state));
        } else if (udc_dwc3_ep_ring_outstanding(ep_data)) {
            LOG_WRN("EP%02x XferComplete ignored: a transfer is still "
                "armed on this endpoint, so this completion cannot "
                "belong to it (late or duplicated event)",
                ep_data->cfg.addr);
        } else {
            LOG_WRN("EP%02x XferComplete: controller released the transfer "
                "resource; endpoint returned to idle so the next arm is a "
                "Start Transfer", ep_data->cfg.addr);
            udc_dwc3_ep_state_reset(ep_data);
        }
    }
}

/*
 * Name of a link state, for logging.
 */
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

#define _NORMAL_EP(n, fn)   fn(n + 2)

/*
 * Name of an event word, for logging.
 */
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
    /* Without these, command completions would log as "unknown event". */
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
         * Link state from the event, speed from DSTS: the speed is fixed within
         * a session, so reading it late is harmless; the link state is not.
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
 * Park every armed buffer on an endpoint back on its requeue FIFO and reset the
 * TRB ring to the empty state.
 */
static void udc_dwc3_ep_ring_release(struct udc_dwc3_ep_data *const ep_data)
{
    /* TRB_NUM - 1: the last TRB is the LINK TRB (udc_dwc3_trb_nonctrl_init),
     * which stays armed to keep the ring intact; only payload slots are cleared.
     */
    const int slots = CONFIG_UDC_DWC3_TRB_NUM - 1;
    struct net_buf *buf;

    for (int n = 0; n < slots; n++) {
        const int idx = (ep_data->ring.tail + n) % slots;

        buf = ep_data->ring.net_buf[idx];
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
    for (uint32_t i = 0U; i < (CONFIG_UDC_DWC3_TRB_NUM - 1U); i++) {
        udc_dwc3_trb_write(&ep_data->trb_buf[i], 0U, 0U, 0U);
    }
    memset(ep_data->ring.net_buf, 0,
           sizeof(*ep_data->ring.net_buf) * (CONFIG_UDC_DWC3_TRB_NUM - 1));
    ep_data->ring.head = ep_data->ring.tail = 0;
    udc_dwc3_ep_busy_sync(ep_data);
}

/*
 * The End Transfer on this endpoint has completed and the resource is free.
 * Continues whatever recovery was waiting for it.
 *
 * Called from both places that detect the completion: the EpCmdCmplt event, and
 * udc_dwc3_ep_resolve_cmd() reading DEPCMD when that event never arrives.
 */
static UDC_DWC3_COLD void udc_dwc3_ep_end_completed(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data)
{
    /* Keeps the work owed to the stack or host; drops an owed Update. */
    udc_dwc3_ep_state_reset(ep_data);

    LOG_DBG("EpCmdCmplt: DMA stopped for EP%02x", ep_data->cfg.addr);

    if (USB_EP_GET_IDX(ep_data->cfg.addr) != 0U) {
        udc_dwc3_ep_recover(dev, ep_data);
        return;
    }

    /*
     * Control half: only the control recovery ends an EP0 transfer, and it stays
     * RECOVERING until both halves are idle; it reclaims the TRBs itself.
     */
    udc_dwc3_ctrl_recover_continue(dev);
}

/*
 * Endpoint Command Complete.
 */
static void udc_dwc3_on_ep_cmd_cmplt(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data, const uint32_t evt)
{
    /*
     * A Start Transfer completion: the index every later Update and End needs,
     * or the Start's refusal.
     */
    if (FIELD_GET(UDC_DWC3_DEPEVT_CMDTYP_MASK, evt) ==
        FIELD_GET(UDC_DWC3_DEPCMD_CMDTYP_MASK, UDC_DWC3_DEPCMD_DEPSTRTXFER)) {
        const bool ok = (evt & UDC_DWC3_DEPEVT_CMDSTATUS_MASK) ==
                UDC_DWC3_DEPCMD_STATUS_OK;
        const uint32_t idx = FIELD_GET(UDC_DWC3_DEPEVT_XFERRSCIDX_MASK, evt);

        if (ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
            ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN) {
            uint32_t reg = 0U;

            /*
             * The event names no command instance: a late-written completion of
             * an earlier Start (event-ring holes) can arrive while this one is
             * open. DEPCMD, or cmd_record once overwritten, holds the open
             * Start's own outcome, so it decides; the event only when neither
             * holds it.
             */
            switch (udc_dwc3_cmd_outcome(dev, ep_data, UDC_DWC3_DEPCMD_DEPSTRTXFER,
                             &reg)) {
            case UDC_DWC3_CMD_UNKNOWN:
                LOG_WRN_RATELIMIT("EP%02x Start Transfer completion 0x%08x while "
                          "the open Start still executes: an earlier "
                          "Start's, discarded", ep_data->cfg.addr, evt);
                return;
            case UDC_DWC3_CMD_OK:
                udc_dwc3_adopt_xferrscidx(dev, ep_data, reg);
                break;
            case UDC_DWC3_CMD_ERROR:
                udc_dwc3_ep_start_refused(dev, ep_data, "DEPCMD", reg, false);
                return;
            case UDC_DWC3_CMD_OTHER:
            default:
                if (!ok) {
                    udc_dwc3_ep_start_refused(dev, ep_data, "Command Complete",
                                  evt, false);
                    return;
                }
                udc_dwc3_adopt_xferrscidx_evt(dev, ep_data, idx);
                break;
            }
        } else if (!ok) {
            /*
             * No Start open: an earlier Start's refusal, already handled when
             * udc_dwc3_depcmd_start_xfer() saw it. Acting on it would tear down
             * whatever runs now.
             */
            LOG_WRN_RATELIMIT("EP%02x Start Transfer refusal 0x%08x while the "
                      "endpoint is %s: an earlier Start's, discarded",
                      ep_data->cfg.addr, evt,
                      udc_dwc3_ep_state_name(ep_data->xfer.state));
            return;
        } else {
            udc_dwc3_adopt_xferrscidx_evt(dev, ep_data, idx);
        }

        /* Buffers armed while the Start was open: their Update is owed now. */
        udc_dwc3_ep_update_owed(dev, ep_data);

        /* A recovery waiting on this Start's outcome goes on. */
        if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U) {
            udc_dwc3_ctrl_recover_continue(dev);
        } else if (udc_dwc3_ep_recovery_owed(dev, ep_data)) {
            udc_dwc3_ep_recover(dev, ep_data);
        }
        return;
    }

    if (!udc_dwc3_ep_is_ending(ep_data)) {
        /*
         * Stale: discard. CMDIOC is set only on Start and End Transfer, so this
         * is an End Transfer completion for an endpoint that is not ending. The
         * endpoint was torn down and re-enabled since (e.g. an alt-setting
         * switch), so it belongs to a previous incarnation.
         *
         * Handling it would reset the new transfer to IDLE and drop its fresh
         * resource index; Update Transfer would then be refused and recovery
         * would take a second transfer resource for the same endpoint.
         */
        LOG_WRN_RATELIMIT("EpCmdCmplt on EP%02x with no End Transfer "
                  "outstanding (%s): discarded",
                  ep_data->cfg.addr,
                  udc_dwc3_ep_state_name(ep_data->xfer.state));
        return;
    }

    udc_dwc3_ep_end_completed(dev, ep_data);
}

/*
 * Log a Link State Change event, rate-limited by repetition.
 *
 * A state different from the last one always prints. Repeats of the same state
 * print once per UDC_DWC3_EVT_LINK_LOG_EVERY, with the run length. The raw event
 * word is logged so the decode can be checked against the databook.
 */
static UDC_DWC3_COLD void udc_dwc3_log_link_event(const struct device *const dev,
                    const uint32_t evt, const uint32_t dsts)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const uint32_t link = FIELD_GET(UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK, evt);

    priv->diag.evt_link_total++;

    if (priv->diag.evt_link_total == 1U || link != priv->diag.evt_link_last) {
        LOG_INF("link %s evt=0x%08x (previous state x%u, %u total)",
            udc_dwc3_get_event_name(evt, dsts), evt,
            priv->diag.evt_link_run, priv->diag.evt_link_total);
        priv->diag.evt_link_last = link;
        priv->diag.evt_link_run = 1U;
        return;
    }

    priv->diag.evt_link_run++;

    if (priv->diag.evt_link_run % UDC_DWC3_EVT_LINK_LOG_EVERY == 0U) {
        LOG_INF("link %s repeating x%u (%u total)",
            udc_dwc3_get_event_name(evt, dsts),
            priv->diag.evt_link_run, priv->diag.evt_link_total);
    }
}

/*
 * An event the dispatch has no handler for: skipped, never assumed impossible.
 */
static UDC_DWC3_COLD void udc_dwc3_evt_unknown(const struct device *const dev,
                           const uint32_t evt)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    LOG_ERR_RATELIMIT("unknown event: 0x%x %s (%u out of %u)", evt,
              udc_dwc3_get_event_name(evt, sys_read32(DEVICE_MMIO_NAMED_GET(dev, base) +
                                  UDC_DWC3_DSTS)),
              priv->evt.gc_last, CONFIG_UDC_DWC3_EVENTS_NUM);
}

/*
 * Endpoint event (DEPEVT, bit 0 = 0): bits 5:1 the physical endpoint, bits 9:6
 * the kind. Both are decoded and the endpoint validated and looked up once here;
 * the handlers take the results. Endpoint events are never logged: every
 * healthy transfer raises them, and a log line costs a synchronous UART write in
 * this thread. Physical endpoints 0 and 1 are the two halves of EP0 (control
 * state machine); the others take the generic ring handler.
 */
static void udc_dwc3_dispatch_ep_event(const struct device *const dev, const uint32_t evt)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    const uint32_t epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
    const uint32_t kind = FIELD_GET(UDC_DWC3_DEPEVT_KIND_MASK, evt);
    struct udc_dwc3_ep_data *ep_data;

    if ((evt & UDC_DWC3_DEPEVT_RSVD_MASK) != 0U) {
        udc_dwc3_evt_unknown(dev, evt);
        return;
    }

    if (!_EPN_IS_VALID(cfg, epn)) {
        LOG_ERR_RATELIMIT("event 0x%08x names physical endpoint %u, which "
                  "this controller does not have (%u IN, %u OUT) - "
                  "discarded",
                  evt, epn, cfg->num_in_eps, cfg->num_out_eps);
        return;
    }

    ep_data = _EP_DATA_FROM_EPN(cfg, epn);

    /* Command completion: one handler for every endpoint, EP0 included. */
    if (kind == UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_EPCMDCMPLT(0))) {
        udc_dwc3_on_ep_cmd_cmplt(dev, ep_data, evt);
        return;
    }

    if (epn < 2U) {
        /* EP0: physical endpoint 1 is its IN half. */
        const bool is_in = epn == 1U;

        switch (kind) {
        case UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_XFERCOMPLETE(0)):
            udc_dwc3_on_ctrl(dev, ep_data, is_in);
            break;
        /* Enabled for EP0 only (DEPCFG). */
        case UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_XFERNOTREADY(0)):
            udc_dwc3_on_ctrl_xnr(dev, evt, is_in);
            break;
        /* EP0 uses no TRB that raises XferInProgress. */
        default:
            udc_dwc3_evt_unknown(dev, evt);
            break;
        }
        return;
    }

    switch (kind) {
    /*
     * Both completion events retire TRBs and mean success; which one the
     * controller raises depends only on the TRB control bits (Table 4-8).
     */
    case UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_XFERCOMPLETE(0)):
        udc_dwc3_on_xfer_done_nonctrl(dev, ep_data, true);
        break;
    case UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_XFERINPROGRESS(0)):
        udc_dwc3_on_xfer_done_nonctrl(dev, ep_data, false);
        break;
    /* XferNotReady is not enabled on non-control endpoints. */
    default:
        udc_dwc3_evt_unknown(dev, evt);
        break;
    }
}

/*
 * Device event (bit 0 = 1): rare, so the logging decision lives here.
 */
static UDC_DWC3_COLD void udc_dwc3_dispatch_dev_event(const struct device *const dev,
                              const uint32_t evt)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t evt_type = evt & UDC_DWC3_EVT_MASK;

    /*
     * Link changes have their own rate-limited log. Overflow and generic
     * command completions can arrive faster than the console prints, and a
     * failing endpoint command is logged by udc_dwc3_on_ep_cmd_cmplt(). DSTS
     * only names the event in a log line, so it is read only for one.
     */
    if (evt_type == UDC_DWC3_DEVT_ULSTCHNG) {
        udc_dwc3_log_link_event(dev, evt, sys_read32(base + UDC_DWC3_DSTS));
    } else if (evt_type != UDC_DWC3_DEVT_EVNTOVERFLOW &&
           evt_type != UDC_DWC3_DEVT_CMDCMPLT) {
        LOG_INF("%s", udc_dwc3_get_event_name(evt, sys_read32(base + UDC_DWC3_DSTS)));
    }

    switch (evt_type) {
    /* Link and bus events: udc_dwc3_link_event() owns all of them. */
    case UDC_DWC3_DEVT_USBRST:
    case UDC_DWC3_DEVT_CONNECTDONE:
    case UDC_DWC3_DEVT_DISCONNEVT:
    case UDC_DWC3_DEVT_ULSTCHNG:
    case UDC_DWC3_DEVT_WKUPEVT:
    case UDC_DWC3_DEVT_SUSPEND:
    case UDC_DWC3_DEVT_ERRTICERR:
    case UDC_DWC3_DEVT_EVNTOVERFLOW:
        udc_dwc3_link_event(dev, evt);
        break;
    case UDC_DWC3_DEVT_SOF:
    case UDC_DWC3_DEVT_CMDCMPLT:
    case UDC_DWC3_DEVT_VNDRDEVTSTRCVED:
        break;
    default:
        udc_dwc3_evt_unknown(dev, evt);
        break;
    }
}

/*
 * Dispatch one event word: bit 0 separates endpoint from device events, and
 * each kind is decoded once below. Caller holds the UDC mutex (the drain takes
 * it once per pass, not per event; the event thread is cooperative anyway) and
 * clears priv->diag.dispatch_evt when its dispatching is over.
 */
static void udc_dwc3_dispatch_event(const struct device *const dev, const uint32_t evt)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    /* Published for udc_dwc3_heartbeat_worker(), which runs on another thread. */
    priv->diag.dispatch_evt = evt;

    if ((evt & BIT(0)) == 0U) {
        udc_dwc3_dispatch_ep_event(dev, evt);
    } else {
        udc_dwc3_dispatch_dev_event(dev, evt);
    }
}

/*
 * Dispatch one event word outside a drain pass, taking the UDC mutex.
 */
static __maybe_unused void udc_dwc3_handle_event(const struct device *const dev,
                           const uint32_t evt)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    priv->diag.dispatch_t0 = k_cycle_get_32();
    udc_lock_internal(dev, K_FOREVER);
    udc_dwc3_dispatch_event(dev, evt);
    priv->diag.dispatch_evt = 0U;
    udc_unlock_internal(dev);
}

#ifdef CONFIG_UDC_DWC3_SHELL
/* Shell only (dwc3 events). */
static const char *udc_dwc3_drain_state_name(const uint32_t state)
{
    switch (state) {
    case UDC_DWC3_DRAIN_IDLE:   return "idle";
    case UDC_DWC3_DRAIN_RUNNING:    return "running";
    case UDC_DWC3_DRAIN_WAITING:    return "waiting";
    default:            return "?";
    }
}
#endif /* CONFIG_UDC_DWC3_SHELL */

/*
 * Check readiness for RunStop=0 (SPEC 4.1.8 Table 4-7): EP0 in its Setup stage
 * (no control transfer, no control recovery, no End Transfer on either half) and
 * no transfer on any other endpoint.
 * Returns -1 when ready, else the first physical endpoint still busy (0 for
 * either EP0 half).
 */
static UDC_DWC3_COLD int udc_dwc3_stop_blocker(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    const struct udc_dwc3_data *const priv = udc_get_private(dev);

    if (priv->ctrl.state != UDC_DWC3_CTRL_IDLE ||
        udc_dwc3_ep_is_ending(&cfg->ep_data_out[0]) ||
        udc_dwc3_ep_is_ending(&cfg->ep_data_in[0])) {
        return 0;
    }
    for (uint32_t epn = 2U; epn < UDC_DWC3_MAX_EPN; epn++) {
        if (_EPN_IS_VALID(cfg, epn) &&
            _EP_DATA_FROM_EPN(cfg, epn)->xfer.state != UDC_DWC3_EP_IDLE) {
            return (int)epn;
        }
    }

    return -1;
}

#ifdef UDC_DWC3_CONTROLLER_RECOVER
static UDC_DWC3_COLD void udc_dwc3_controller_recover(const struct device *const dev,
                       const char *const reason);
#endif

static void udc_dwc3_drain_helper(const struct device *const dev);

/*
 * Heartbeat timer callback: kicks the drain and submits the heartbeat worker.
 *
 * ISR context: only cheap, ISR-safe work (MMIO read, arithmetic,
 * k_work_submit_to_queue()). No mutex, no logging (LOG_MODE_MINIMAL busy-waits
 * on the UART for milliseconds with interrupts off), no recovery; those belong
 * in udc_dwc3_heartbeat_worker().
 */
static UDC_DWC3_COLD void udc_dwc3_heartbeat_expiry(struct k_timer *const timer)
{
    struct udc_dwc3_data *const priv =
        CONTAINER_OF(timer, struct udc_dwc3_data, heartbeat_timer);
    const struct device *const dev = priv->dev;

    udc_dwc3_drain_helper(dev);

    priv->diag.hb_expiries++;

    /* Submit time; the worker reads it on entry to measure the queue wait. */
    if (priv->diag.hb_submit_t == 0U) {
        priv->diag.hb_submit_t = k_cycle_get_32();
    }

    /* 0 = already pending: the previous beat has not run, so drop this one. */
    if (k_work_submit_to_queue(udc_get_work_q(),
                   &priv->heartbeat_work) == 0) {
        priv->diag.hb_coalesced++;
    }
}


/*
 * Is the empty slot at evt.next provably lost rather than merely late?
 */
static UDC_DWC3_COLD bool udc_dwc3_evt_lookahead_lost(const struct device *const dev,
                    const uint32_t gc)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    uint32_t owed = gc / sizeof(uint32_t);

    if (owed > (CONFIG_UDC_DWC3_EVENTS_NUM - 1u)) {
        owed = CONFIG_UDC_DWC3_EVENTS_NUM - 1u;
    }

    for (uint32_t j = 1u; j < owed; j++) {
        const uint32_t idx = (priv->evt.next + j) % CONFIG_UDC_DWC3_EVENTS_NUM;

        if (cfg->evt_buf[idx] != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
            return true;
        }
    }

    return false;
}

/* Defined below; udc_dwc3_drain_slot_is_dead() decides, this performs it. */
static uint32_t udc_dwc3_evt_skip_dead_slot(const struct device *const dev,
                    const uint32_t gc, const bool frozen,
                    const uint32_t gaveup_ms);


/*
 * Resolve an endpoint with a command outstanding by asking the controller
 * directly, and promote it to the matching UNKNOWN state once the command has
 * been executing past UDC_DWC3_CMD_UNKNOWN_MS.
 */
static UDC_DWC3_COLD void udc_dwc3_ep_resolve_cmd(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data)
{
    const bool starting = (ep_data->xfer.state == UDC_DWC3_EP_STARTING) ||
                  (ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN);
    const bool ending = (ep_data->xfer.state == UDC_DWC3_EP_ENDING) ||
                (ep_data->xfer.state == UDC_DWC3_EP_END_UNKNOWN);
    uint32_t reg = 0U;
    enum udc_dwc3_cmd_outcome out;

    if (!starting && !ending) {
        return;
    }

    out = udc_dwc3_cmd_outcome(dev, ep_data,
                   starting ? UDC_DWC3_DEPCMD_DEPSTRTXFER
                        : UDC_DWC3_DEPCMD_DEPENDXFER,
                   &reg);

    switch (out) {
    case UDC_DWC3_CMD_OTHER:
        /*
         * DEPCMD was overwritten with no record kept (see
         * udc_dwc3_cmd_outcome()), so our command's outcome is unknown. Not
         * expected: every overwrite goes through udc_dwc3_depcmd()'s
         * pre-poll, which keeps the record, except the first command on an
         * endpoint. Treat it as still executing: the CMDIOC Command Complete
         * still resolves it, and past the deadline the endpoint goes UNKNOWN
         * for the quiescence door.
         */
        __fallthrough;

    case UDC_DWC3_CMD_UNKNOWN:
        /*
         * Still executing is not UNKNOWN until the deadline passes; after
         * it, mark the endpoint UNKNOWN explicitly. UNKNOWN refuses every
         * further Start and End, so this command's outcome (in DEPCMD, or
         * in cmd_record once overwritten) stays readable.
         */
        if (udc_dwc3_ep_is_unknown(ep_data) ||
            k_cyc_to_ms_near32(k_cycle_get_32() - ep_data->cmd.cmd_t0) <
                UDC_DWC3_CMD_UNKNOWN_MS) {
            return;
        }

        (void)udc_dwc3_ep_state_set(ep_data,
                        starting ? UDC_DWC3_EP_START_UNKNOWN
                             : UDC_DWC3_EP_END_UNKNOWN);
        ((struct udc_dwc3_data *)udc_get_private(dev))->diag.ep_cmd_unknown++;
        LOG_ERR("EP%02x %s Transfer %s for %u ms: outcome "
            "undetermined (DEPCMD 0x%08x), no further command will be "
            "posted on this endpoint until it resolves",
            ep_data->cfg.addr, starting ? "Start" : "End",
            (out == UDC_DWC3_CMD_OTHER)
                ? "unresolved, DEPCMD since overwritten,"
                : "has been executing",
            UDC_DWC3_CMD_UNKNOWN_MS, reg);
        return;

    case UDC_DWC3_CMD_ERROR:
        if (ending) {
            /*
             * End refused: the transfer still runs and owns its
             * resource. Restore its index, or RUNNING is a dead end.
             */
            LOG_WRN("EP%02x End Transfer had failed unobserved "
                "(0x%08x); the transfer is still running",
                ep_data->cfg.addr, reg);
            udc_dwc3_ep_end_refused(dev, ep_data);
            return;
        }
        /* Start refused, its Command Complete lost: as if it had arrived. */
        udc_dwc3_ep_start_refused(dev, ep_data, "DEPCMD", reg, false);
        return;

    case UDC_DWC3_CMD_OK:
    default:
        if (ending) {
            /*
             * Proven, not timed out: DEPCMD (or the record kept when
             * it was overwritten) shows our End Transfer completed
             * successfully. The type is checked, and the Start/End
             * refusals guarantee it is ours, so the resource is free.
             */
            if (ep_data->xfer.state == UDC_DWC3_EP_END_UNKNOWN) {
                LOG_WRN("EP%02x End Transfer resolved late: the "
                    "controller has released the transfer",
                    ep_data->cfg.addr);
            }
            udc_dwc3_ep_end_completed(dev, ep_data);
            return;
        }
        udc_dwc3_adopt_xferrscidx(dev, ep_data, reg);
        return;
    }
}


/*
 * Per-endpoint sweep: one pass per heartbeat, two rules, no policy, no timer.
 *
 * The event ring on this part loses writes, so nothing here relies on events.
 * Each question has an answer that a lost event cannot affect:
 *
 *   what a command did          -> DEPCMD: CmdAct, CmdStatus, XferRscIdx
 *   what a transfer did         -> the TRB: HWO, and BUFSIZ written back
 *
 * Whether the host is starved is not observable here: non-control endpoints get
 * no XferNotReady (it is enabled on EP0 only), and no timer may stand in for it.
 *
 * Control endpoints are not swept: EP0 is a stage machine, not a ring, recovered
 * by Set Stall and a fresh SETUP arm in udc_dwc3_ctrl_ep_recover().
 */
static UDC_DWC3_COLD void udc_dwc3_ep_sweep(const struct device *const dev,
                  struct udc_dwc3_ep_data *const ep_data)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    if (!ep_data->cfg.stat.enabled && !udc_dwc3_ep_recovery_owed(dev, ep_data)) {
        return;
    }

    /*
     * RULE 1: settle any command whose Command Complete never arrived, by
     * reading DEPCMD. Also promotes an unresolvable command to
     * START_UNKNOWN / END_UNKNOWN.
     */
    udc_dwc3_ep_resolve_cmd(dev, ep_data);

    /* A command still owns this endpoint (this includes both UNKNOWN states): leave it. */
    if (udc_dwc3_ep_cmd_busy(ep_data)) {
        return;
    }

    /* A recovery left owing work - e.g. a Start settled by the resolver - goes on. */
    if (udc_dwc3_ep_recovery_owed(dev, ep_data)) {
        udc_dwc3_ep_recover(dev, ep_data);
        return;
    }

    /* A Start settled by the resolver (its Command Complete lost) may owe an Update. */
    udc_dwc3_ep_update_owed(dev, ep_data);

    /*
     * RULE 2: retire every completion already written back. udc_dwc3_pop_trb()
     * checks HWO and BUFSIZ in memory, so this recovers a lost XferComplete.
     */
    {
        const uint32_t got = udc_dwc3_drain_completed(dev, ep_data);

        if (got > 0U) {
            priv->diag.evt_sweep_rescued += got;
            priv->diag.evt_sweep_runs++;
        }
    }

    /*
     * A live ring on an IDLE endpoint means no transfer was started: a reset
     * path cleared the state while buffers stayed armed. The arm path starts
     * from IDLE, so this only catches teardown orphans. Start the transfer.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_IDLE &&
        udc_dwc3_ep_ring_outstanding(ep_data)) {
        LOG_WRN("EP%02x has descriptors armed with no transfer started - "
            "starting it", ep_data->cfg.addr);
        (void)udc_dwc3_depcmd_start_xfer(dev, ep_data);
    }

    /*
     * No rule 3 (inactivity timer), deliberately. A non-control OUT endpoint
     * holding a controller-owned TRB and retiring nothing is the normal idle
     * state: it waits for the host. No timeout can tell that from a fault; a
     * timer here tore down healthy transfers on quiet endpoints.
     */
}

/* Called only from udc_dwc3_heartbeat_worker(), with the UDC mutex held. */
static UDC_DWC3_COLD void udc_dwc3_recover_all(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;

    /*
     * EP0: only settle an open command outcome from DEPCMD; the control stages
     * are event-driven. Without this, a control half whose End Transfer
     * Command Complete is lost stays ENDING forever (there is no EP0
     * watchdog). Passive: reads DEPCMD and acts only on proof.
     */
    udc_dwc3_ep_resolve_cmd(dev, &cfg->ep_data_out[0]);
    udc_dwc3_ep_resolve_cmd(dev, &cfg->ep_data_in[0]);

    /* A control-endpoint recovery waiting on one of those outcomes goes on. */
    udc_dwc3_ctrl_recover_continue(dev);

    for (int i = 1; i < cfg->num_in_eps; i++) {
        udc_dwc3_ep_sweep(dev, &cfg->ep_data_in[i]);
    }
    for (int i = 1; i < cfg->num_out_eps; i++) {
        udc_dwc3_ep_sweep(dev, &cfg->ep_data_out[i]);
    }
}

/*
 * Periodic liveness work: observe the drain, run recovery, report.
 *
 * Driven by a timer, not an event count, so it keeps reporting during event
 * loss. Runs off the drain thread: ~700 characters at ~87 us each under
 * CONFIG_LOG_MODE_MINIMAL is ~60 ms of synchronous console with the ring undrained.
 */
static void udc_dwc3_ctrl_setup_wd_check(const struct device *const dev);

static void udc_dwc3_heartbeat_worker(struct k_work *work)
{

    struct udc_dwc3_data *const priv =
        CONTAINER_OF(work, struct udc_dwc3_data, heartbeat_work);
    const struct device *const dev = priv->dev;
    const struct udc_dwc3_config *const cfg = dev->config;
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t d_evt = priv->diag.dispatch_evt;
    /*
     * No register read here: udc_dwc3_drain_helper() made this beat's single
     * read; use the published value.
     */
    const uint32_t gc = priv->evt.gc_last;
    const uint32_t hb_now = k_cycle_get_32();

    /* Every run counts itself before anything here can block or return. */
    if (priv->diag.hb_beats != 0U) {
        const uint32_t gap = k_cyc_to_ms_near32(hb_now - priv->diag.hb_last_t);

        if (gap > priv->diag.hb_gap_ms_max) {
            priv->diag.hb_gap_ms_max = gap;
        }
    }
    priv->diag.hb_last_t = hb_now;
    priv->diag.hb_beats++;

    /*
     * Sum of the fault counters. If unchanged, the stats line would only
     * repeat, yet still cost the drain thread ~35 ms; skip it (until the force
     * interval).
     */
    const uint32_t stats_sig = priv->diag.evt_late + priv->diag.evt_gaveup +
                   priv->diag.evt_missed +
                   priv->diag.ctrl_start_fail + priv->diag.ctrl_setup_wd_fire;

    if ((priv->diag.hb_beats % UDC_DWC3_EVT_STATS_BEATS) == 0U) {
        priv->diag.stats_quiet_beats += UDC_DWC3_EVT_STATS_BEATS;
    }

    if ((priv->diag.hb_beats % UDC_DWC3_EVT_STATS_BEATS) == 0U &&
        (stats_sig != priv->diag.stats_sig_last ||
         priv->diag.stats_quiet_beats >= UDC_DWC3_EVT_STATS_FORCE_BEATS)) {
        priv->diag.stats_sig_last = stats_sig;
        priv->diag.stats_quiet_beats = 0U;
        /*
         * Drain-thread stack headroom: the smallest never-used span seen so
         * far. A synchronous log backend formats on that stack, so this shows
         * whether UDC_DWC3_EVT_STACK_SIZE is enough. Printed only when measured.
         */
        if (IS_ENABLED(CONFIG_INIT_STACKS) &&
            IS_ENABLED(CONFIG_THREAD_STACK_INFO) &&
            priv->evt_thread != NULL) {
            size_t unused = 0;

            if (k_thread_stack_space_get(priv->evt_thread, &unused) == 0) {
                if ((uint32_t)unused < priv->diag.evt_stack_free) {
                    priv->diag.evt_stack_free = (uint32_t)unused;
                }
                LOG_INF("evtstack %u B free of %u", priv->diag.evt_stack_free,
                    UDC_DWC3_EVT_STACK_SIZE);
            }
        }

        /*
         * Print only non-zero counters: the full ~400-character line cost
         * ~35 ms of synchronous console (ring undrained) even when all were zero.
         * Abbreviations: lt late, gu gaveup, sk skipped/refuted, ms missed/frozen,
         * mz midzero, sf startfail, swd setup-watchdog, kk drain kicks,
         * sp setup-pending, sr stale control buffers returned, st control stalls,
         * eh endpoint halts, sm TRB stomps, rc control reclaims, om unaligned OUT,
         * mu multi-word give-ups, la look-ahead skips, dr drain-dead reconnects,
         * cu commands left UNKNOWN, oN/iN endpoint arms/retires.
         */
        {
            char b[248];
            size_t o = 0;

            b[0] = '\0';

#define _P(cond, fmt, ...)                                                    \
    do {                                                                  \
        if ((cond) && o < sizeof(b)) {                                \
            int _w = snprintk(&b[o], sizeof(b) - o, fmt,          \
                      ##__VA_ARGS__);                     \
            if (_w > 0) {                                         \
                o += (size_t)_w;                              \
            }                                                     \
        }                                                             \
    } while (0)

            _P(priv->diag.evt_late, " lt%u/%up/%uu", priv->diag.evt_late,
               priv->diag.evt_late_polls_max, priv->diag.evt_late_us_max);
            _P(priv->diag.evt_gaveup, " gu%u/%uu", priv->diag.evt_gaveup,
               priv->diag.evt_gaveup_us_max);
            _P(priv->diag.evt_skipped, " sk%u/%u", priv->diag.evt_skipped,
               priv->diag.evt_skip_refuted);
            _P(priv->diag.evt_missed, " ms%u/%u", priv->diag.evt_missed,
               priv->diag.evt_missed_frozen);
            _P(priv->diag.evt_midzero, " mz%u", priv->diag.evt_midzero);
            _P(priv->diag.ctrl_start_fail, " sf%u", priv->diag.ctrl_start_fail);
            _P(priv->diag.ctrl_setup_wd_fire, " swd%u", priv->diag.ctrl_setup_wd_fire);
            _P(priv->diag.evt_kick, " kk%u", priv->diag.evt_kick);
            _P(priv->diag.ctrl_setup_pending, " sp%u", priv->diag.ctrl_setup_pending);
            _P(priv->diag.ctrl_stale_returned, " sr%u", priv->diag.ctrl_stale_returned);
            _P(priv->diag.ctrl_stall_issued, " st%u", priv->diag.ctrl_stall_issued);
            _P(priv->diag.ep_halts, " eh%u", priv->diag.ep_halts);
            _P(priv->diag.trb_stomp, " sm%u", priv->diag.trb_stomp);
            _P(priv->diag.ctrl_reclaim_done, " rc%u", priv->diag.ctrl_reclaim_done);
            _P(priv->diag.out_unaligned, " om%u", priv->diag.out_unaligned);
            _P(priv->diag.evt_gaveup_multi, " mu%u", priv->diag.evt_gaveup_multi);
            /* la: skips proven by the 50 ms look-ahead, not the 1 s budget. */
            _P(priv->diag.evt_lookahead_short, " la%u", priv->diag.evt_lookahead_short);
            /* dr: reconnects issued because the ring stopped advancing. */
            _P(priv->diag.drain_dead_resets, " dr%u", priv->diag.drain_dead_resets);
            /* cu: Start/End outcomes promoted to UNKNOWN by the resolver. */
            _P(priv->diag.ep_cmd_unknown, " cu%u", priv->diag.ep_cmd_unknown);

            /*
             * Per-endpoint arm/retire counts, both directions, for every
             * non-control endpoint ever armed: an endpoint that stops shows as
             * a number that stops. This was once the only evidence of a CDC
             * bulk endpoint frozen while every other counter read clean.
             */
            for (uint8_t _i = 1U; _i < cfg->num_out_eps; _i++) {
                _P(cfg->ep_data_out[_i].diag.n_arm, " o%u:%u/%u", _i,
                   cfg->ep_data_out[_i].diag.n_arm,
                   cfg->ep_data_out[_i].diag.n_retire);
            }
            for (uint8_t _i = 1U; _i < cfg->num_in_eps; _i++) {
                _P(cfg->ep_data_in[_i].diag.n_arm, " i%u:%u/%u", _i,
                   cfg->ep_data_in[_i].diag.n_arm,
                   cfg->ep_data_in[_i].diag.n_retire);
            }
#undef _P

            LOG_INF("ev%u ct%u/%u isr%u rn%u hwm%u lk%u%s D%08x",
                priv->evt.handled,
                priv->diag.ctrl_setup_done, priv->diag.ctrl_status_done,
                priv->diag.evt_isr, priv->diag.evt_worker_runs,
                priv->diag.evt_gevntcount_hwm, priv->diag.evt_link_total,
                b, sys_read32(base + UDC_DWC3_DSTS));
        }
    }

    if (priv->diag.hb_submit_t != 0U) {
        const uint32_t q = k_cyc_to_ms_near32(hb_now - priv->diag.hb_submit_t);

        if (q > priv->diag.hb_q_ms_max) {
            priv->diag.hb_q_ms_max = q;
        }

        priv->diag.hb_submit_t = 0U;
    }

    /*
     * Settle what a lost event left open, from what the controller shows
     * without events (DEPCMD, TRB write-back): udc_dwc3_recover_all().
     */
    udc_lock_internal(dev, K_FOREVER);
    udc_dwc3_recover_all(dev);
    udc_unlock_internal(dev);

    const uint32_t idle_ms =
        k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt.worker_exit_t0);
    /* Nothing dispatched since the previous beat. */
    const bool none_handled = (priv->evt.handled == priv->evt.hb_last_handled);

    if (gc > 0U && none_handled) {
        priv->evt.hb_stuck_beats++;
    } else {
        priv->evt.hb_stuck_beats = 0U;
    }
    priv->evt.hb_last_handled = priv->evt.handled;

#ifdef UDC_DWC3_CONTROLLER_RECOVER
    if (priv->run.state == UDC_DWC3_RUN_RESET_OWED) {
        udc_dwc3_controller_recover(dev, "PHY erratic error");
        return;
    }

    /*
     * Drain dead: events are owed, none handled for UDC_DWC3_HB_DRAIN_DEAD_MS,
     * and kicking the semaphore has not helped. Reconnect through the
     * controller recovery; it is the only path back onto the bus. Clear the
     * beat count first so a failed reconnect does not re-enter on the next beat.
     */
    if ((priv->evt.hb_stuck_beats * UDC_DWC3_HEARTBEAT_MS) >= UDC_DWC3_HB_DRAIN_DEAD_MS) {
        priv->evt.hb_stuck_beats = 0U;
        priv->diag.drain_dead_resets++;

        LOG_ERR("event ring not advancing for %u ms with %u B owed - "
            "reconnecting (%u so far)",
            UDC_DWC3_HB_DRAIN_DEAD_MS, gc, priv->diag.drain_dead_resets);

        udc_dwc3_controller_recover(dev, "event ring not advancing");
        return;
    }
#endif

    /*
     * Report the bus/DMA configuration once, from here rather than from
     * udc_dwc3_init().
     */
    if (!priv->diag.buscfg_logged) {
        priv->diag.buscfg_logged = true;
        LOG_INF("BUSCFG: GSBUSCFG0=0x%08x GSBUSCFG1=0x%08x GUCTL=0x%08x "
            "GUCTL1=0x%08x GCTL=0x%08x",
            sys_read32(base + UDC_DWC3_GSBUSCFG0),
            sys_read32(base + UDC_DWC3_GSBUSCFG1),
            sys_read32(base + UDC_DWC3_GUCTL),
            sys_read32(base + UDC_DWC3_GUCTL1),
            sys_read32(base + UDC_DWC3_GCTL));
    }

    /* CORE debug baseline. */
    if (++priv->diag.core_dbg_beats >= UDC_DWC3_CORE_DBG_BEATS) {
        struct udc_dwc3_core_dbg dbg;
        bool changed;

        priv->diag.core_dbg_beats = 0U;
        priv->diag.core_dbg_quiet += UDC_DWC3_CORE_DBG_BEATS;
        udc_dwc3_core_dbg_read(base, &dbg);

        /*
         * Log only on change (or after the force interval): an unchanged line
         * costs the drain thread ~17 ms of synchronous console. A real
         * transition, e.g. LTSSM dropping to zero when the core dies, is
         * reported at once.
         *
         * Never return from here: the drain report and the SETUP telemetry
         * below still run, and logging must not decide whether they do.
         */
        changed = memcmp(&dbg, &priv->diag.core_dbg_last, sizeof(dbg)) != 0;
        if (changed || priv->diag.core_dbg_quiet >= UDC_DWC3_EVT_STATS_FORCE_BEATS) {
            priv->diag.core_dbg_last = dbg;
            priv->diag.core_dbg_quiet = 0U;
            udc_dwc3_core_dbg_log(" hb", &dbg);

            /*
             * Beats run/timer expiries since boot, beats coalesced, and the
             * worst timer gap and work-queue delay so far.
             */
            LOG_INF("  HB: beats %u/%u drop %u gapmax %u ms qmax %u ms "
                "(nominal %u)",
                priv->diag.hb_beats, priv->diag.hb_expiries,
                priv->diag.hb_coalesced, priv->diag.hb_gap_ms_max,
                priv->diag.hb_q_ms_max, UDC_DWC3_HEARTBEAT_MS);
        }
    }


    if (d_evt != 0U) {
        const uint32_t ms =
            k_cyc_to_ms_near32(k_cycle_get_32() - priv->diag.dispatch_t0);

        if (ms >= UDC_DWC3_DISPATCH_STUCK_MS) {
            LOG_ERR_RATELIMIT("dispatch stuck %u ms in %s (evt 0x%08x)", ms,
                udc_dwc3_get_event_name(d_evt,
                    sys_read32(base + UDC_DWC3_DSTS)), d_evt);
        }
    } else if (gc > 0U && priv->evt.drain.state != UDC_DWC3_DRAIN_WAITING && none_handled) {
        /*
         * Events owed, none handled since the last beat, and the drain is not
         * in a late-write episode (a slot it waits on is the drain's own
         * business: udc_dwc3_drain_slot_is_dead()). The drain has not run.
         * Print idle_ms, not a threshold, so the real idle time shows.
         */
        LOG_ERR_RATELIMIT("%u B pending, drain IDLE %u ms with no stall "
            "run: slot %u holds 0x%08x, DSTS=0x%08x",
            gc, idle_ms, priv->evt.next,
            cfg->evt_buf[priv->evt.next],
            sys_read32(base + UDC_DWC3_DSTS));

        /* Retry here rather than spinning in the drain loop. */
        k_sem_give(&priv->evt_sem);
    }

    udc_dwc3_ctrl_setup_wd_check(dev);

    /* No re-arm here on purpose: the periodic timer owns the cadence. */
}

/*
 * One settle of a quiesce wait (the controller recovery's, udc_dwc3_disable()'s),
 * with the UDC mutex held. The End Transfer completions the wait depends on come
 * as events, which the event ring may have lost or the mutex may hold back, so
 * read the outcomes from DEPCMD instead (udc_dwc3_ep_resolve_cmd(): acts only on
 * proof, and does what the lost event would have done). A Start settled
 * successfully leaves a running transfer, which gets its End Transfer here
 * (4.1.8). EP0 goes on with its return to Setup. No other Start and no Update:
 * STOPPING refuses them (the worker, udc_dwc3_ep_recover()'s restart and
 * udc_dwc3_ep_resume()).
 */
static UDC_DWC3_COLD void udc_dwc3_quiesce_settle(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;

    udc_dwc3_ep_resolve_cmd(dev, &cfg->ep_data_out[0]);
    udc_dwc3_ep_resolve_cmd(dev, &cfg->ep_data_in[0]);
    udc_dwc3_ctrl_recover_continue(dev);

    for (uint32_t epn = 2U; epn < UDC_DWC3_MAX_EPN; epn++) {
        struct udc_dwc3_ep_data *ep_data;

        if (!_EPN_IS_VALID(cfg, epn)) {
            continue;
        }
        ep_data = _EP_DATA_FROM_EPN(cfg, epn);
        udc_dwc3_ep_resolve_cmd(dev, ep_data);
        if (ep_data->xfer.state == UDC_DWC3_EP_RUNNING) {
            udc_dwc3_ep_recover(dev, ep_data);
        }
    }
}

#ifdef UDC_DWC3_CONTROLLER_RECOVER
/*
 * Controller recovery, the only path that resets the controller:
 *   1. device-initiated disconnect (SPEC 3.30b 4.1.8: End Transfer for every
 *      active transfer, then RunStop=0 and wait for DevCtrlHlt);
 *   2. core soft reset;
 *   3. reconnect as after power-on (4.1.9 -> 4.1.1).
 * Runs from the heartbeat (it sleeps), for an erratic error (3.3.2: "Software
 * must reset the controller") or an event ring that stopped advancing.
 */
static UDC_DWC3_COLD void udc_dwc3_controller_recover(const struct device *const dev,
                       const char *const reason)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t gsts = sys_read32(base + UDC_DWC3_GSTS);
    int ret;

    LOG_ERR("%s: GSTS=0x%08x "
        "BusErrAddrVld=%u GBUSERRADDR=0x%08x%08x, DSTS=0x%08x - "
        "device-initiated disconnect, soft reset, reconnect",
        reason, gsts, (gsts & UDC_DWC3_GSTS_BUSERRADDRVLD) ? 1U : 0U,
        sys_read32(base + UDC_DWC3_GBUSERRADDR_HI),
        sys_read32(base + UDC_DWC3_GBUSERRADDR_LO),
        sys_read32(base + UDC_DWC3_DSTS));

    udc_lock_internal(dev, K_FOREVER);

    /*
     * STEP 1. SPEC 4.1.8 Table 4-7, in order, before RunStop is cleared:
     *   - "If a control transfer is still in progress, complete it and get the
     *     controller into the 'Setup a Control-Setup TRB / Start Transfer' state"
     *   - "Issue a DEPENDXFER command for any active transfers (except for the
     *     default control endpoint 0)"
     * UDC_DWC3_RUN_STOPPING keeps every endpoint worker from starting a new
     * transfer. The lock is released so the drain can deliver the End Transfer
     * completions; the transfer-state writers wake this thread on each change
     * until no transfer is left and EP0 is at Setup. The event ring may be what
     * failed, so each wake-up, and at least every UDC_DWC3_QUIESCE_SETTLE_MS,
     * also settles from DEPCMD what those events would have
     * (udc_dwc3_quiesce_settle()). Bounded wait, then stop regardless.
     */
    priv->run.state = UDC_DWC3_RUN_STOPPING;
    k_sem_reset(&priv->run.quiesce_sem);
    udc_dwc3_end_all_transfers(dev, false);
    udc_unlock_internal(dev);

    {
        const int64_t deadline = k_uptime_get() +
                     (int64_t)UDC_DWC3_HALT_POLLS * UDC_DWC3_HALT_POLL_MS;

        int blocker;

        while ((blocker = udc_dwc3_stop_blocker(dev)) >= 0 &&
               k_uptime_get() < deadline) {
            (void)k_sem_take(&priv->run.quiesce_sem,
                     K_MSEC(CLAMP(deadline - k_uptime_get(), (int64_t)1,
                              (int64_t)UDC_DWC3_QUIESCE_SETTLE_MS)));
            udc_lock_internal(dev, K_FOREVER);
            udc_dwc3_quiesce_settle(dev);
            udc_unlock_internal(dev);
        }
        if (blocker >= 0) {
            LOG_WRN("physical EP %d still has a transfer after %u ms; "
                "stopping anyway", blocker,
                UDC_DWC3_HALT_POLLS * UDC_DWC3_HALT_POLL_MS);
        }
    }

    udc_lock_internal(dev, K_FOREVER);

    /*
     * STEP 2. Clear RunStop only. Keep the interrupt unmasked (unlike
     * udc_dwc3_disable()): if nobody acknowledges events, the controller never
     * reaches DEVCTRLHLT.
     */
    udc_dwc3_dctl_update(base, UDC_DWC3_DCTL_RUNSTOP, 0U);

    udc_unlock_internal(dev);

    /*
     * STEP 3. Wait for the halt with the mutex released, so the drain thread
     * can acknowledge events already written. Skipping this wait once reset a
     * live core.
     */
    {
        bool halted = false;

        for (uint32_t i = 0; i < UDC_DWC3_HALT_POLLS; i++) {
            if ((sys_read32(base + UDC_DWC3_DSTS) &
                 UDC_DWC3_DSTS_DEVCTRLHLT) != 0U) {
                halted = true;
                break;
            }
            k_sleep(K_MSEC(UDC_DWC3_HALT_POLL_MS));
        }

        if (!halted) {
            priv->diag.halt_timeouts++;
            LOG_ERR("controller did not report DEVCTRLHLT within %u ms "
                "(DSTS 0x%08x): resetting it anyway (%u so far)",
                UDC_DWC3_HALT_POLLS * UDC_DWC3_HALT_POLL_MS,
                sys_read32(base + UDC_DWC3_DSTS),
                priv->diag.halt_timeouts);
        }
    }

    /*
     * STEP 4. Re-initialise, only now that the controller is halted and writes
     * nothing more. The drain is shut out first, before the mutex is taken, so
     * no pass can copy between here and the ring reset and then dispatch old
     * events into the new session. It stays out (RESETTING) until the
     * re-initialised ring is enabled (udc_dwc3_enable()) or this function ends;
     * the ring and drain state are reset together in udc_dwc3_on_soft_reset().
     * The mutex is held through udc_dwc3_init(), which sleeps 2 x
     * UDC_DWC3_PHY_RESET_MS across the PHY reset: bounded, and the controller is
     * stopped, so callers waiting on the mutex lose nothing.
     */
    (void)udc_dwc3_evt_block(dev);
    udc_lock_internal(dev, K_FOREVER);

    udc_dwc3_disable(dev);

    /*
     * shutdown() is required: udc_dwc3_disable() stops the timer, clears
     * RunStop and masks the IRQ, but leaves the endpoints enabled
     * (udc_ep_config.stat.enabled stays set).
     */
    /*
     * The stack may have disabled or shut the device down while the mutex was
     * released above: bring the controller back only as far as the stack has
     * it (initialised, enabled), never further.
     */
    ret = udc_dwc3_shutdown(dev);
    if (ret != 0) {
        LOG_ERR("escalation: shutdown failed (%d), core left reset", ret);
    } else if (!udc_is_initialized(dev)) {
        LOG_INF("escalation: device shut down by the stack, left so");
    } else {
        ret = udc_dwc3_init(dev);
        if (ret != 0) {
            LOG_ERR("escalation: init failed (%d), core left unconfigured", ret);
        } else if (!udc_is_enabled(dev)) {
            LOG_INF("escalation: device disabled by the stack, not reconnecting");
        } else {
            ret = udc_dwc3_enable(dev);
            if (ret != 0) {
                LOG_ERR("escalation: enable failed (%d)", ret);
            }
        }
    }

    /*
     * RUN in every case. udc_dwc3_enable() leaves STOPPING alone; on any other
     * exit the controller is stopped as udc_dwc3_disable() leaves it, RUN with
     * RunStop clear, so a later enable starts from a known state.
     */
    priv->run.state = UDC_DWC3_RUN;

    udc_unlock_internal(dev);
}
#endif /* UDC_DWC3_CONTROLLER_RECOVER */

/*
 * EP0 SETUP telemetry: reports only, never recovers.
 *
 * The control path is event-driven: XferNotReady says which phase the host wants
 * (4.4.1/4.4.2), SetupPending on a completion reports an abandoned transfer, and
 * the next SETUP resyncs everything. A stage the host stops addressing is idle,
 * not stuck. The one case events miss, a controller that posts nothing, no
 * command has ever cured: Update Transfer, Set Stall and device-initiated
 * disconnect either did nothing or turned a self-clearing stall (or a healthy
 * idle EP0) into a dead device.
 *
 * So, once per armed SETUP, it reports the one signature worth recording: a
 * SETUP in the RxFIFO that the controller has not delivered.
 */
static UDC_DWC3_COLD void udc_dwc3_ctrl_setup_wd_check(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const struct udc_dwc3_config *const cfg = dev->config;
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    struct udc_dwc3_ep_data *const ep0_out = &cfg->ep_data_out[0];
    uint32_t dsts;
    uint32_t trb_ctrl;
    bool machine_owns;

    if (priv->diag.watchdog_type != UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
        priv->diag.ctrl_setup_wd_beats = 0U;
        return;
    }

    /*
     * Age the armed SETUP in heartbeats. A new SETUP since the last beat
     * restarts the count; so does a check that finds nothing to report.
     */
    if (priv->diag.ctrl_setup_wd_gen != priv->diag.ctrl_setup_wd_seen) {
        priv->diag.ctrl_setup_wd_seen = priv->diag.ctrl_setup_wd_gen;
        priv->diag.ctrl_setup_wd_beats = 0U;
        priv->diag.ctrl_setup_wd_reported = false;
        return;
    }
    if (priv->diag.ctrl_setup_wd_reported ||
        ++priv->diag.ctrl_setup_wd_beats * UDC_DWC3_HEARTBEAT_MS <
            UDC_DWC3_SETUP_WD_REPORT_MS) {
        return;
    }
    priv->diag.ctrl_setup_wd_beats = 0U;

    /* A SETUP that retired, or an idle bus, is nothing to report. */
    trb_ctrl = ep0_out->trb_buf[0].ctrl;
    if ((trb_ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U) {
        return;
    }

    dsts = sys_read32(base + UDC_DWC3_DSTS);
    if ((dsts & UDC_DWC3_DSTS_RXFIFOEMPTY) != 0U) {
        return;
    }

    /*
     * Occupancy alone is not evidence: the RxFIFO is shared with every bulk
     * OUT endpoint. Report only if nothing at all has retired since the
     * SETUP was armed.
     */
    if (priv->diag.ctrl_setup_done != priv->diag.ctrl_setup_wd_snap_setup ||
        priv->diag.nonctrl_done != priv->diag.ctrl_setup_wd_snap_nonctrl) {
        priv->diag.ctrl_setup_wd_snap_setup = priv->diag.ctrl_setup_done;
        priv->diag.ctrl_setup_wd_snap_nonctrl = priv->diag.nonctrl_done;
        return;
    }

    /* Read under the mutex: commands are posted from another queue. */
    udc_lock_internal(dev, K_FOREVER);
    machine_owns = udc_dwc3_ep_cmd_busy(ep0_out);
    udc_unlock_internal(dev);
    if (machine_owns) {
        return;
    }

    priv->diag.ctrl_setup_wd_fire++;
    priv->diag.ctrl_setup_wd_reported = true;
    LOG_ERR("SETUP received but not delivered: TRB still owned by the core "
        "%u ms after arming, RxFIFO occupied, nothing retired (DSTS 0x%08x, "
        "TRB ctrl 0x%08x) - reported, not recovered (%u so far)",
        UDC_DWC3_SETUP_WD_REPORT_MS, dsts, trb_ctrl, priv->diag.ctrl_setup_wd_fire);

    /* The core's own state, passively. One report per armed SETUP. */
    udc_dwc3_core_state_dump(dev);
}

/* Event buffer constraint (GEVNTSIZ): at least 32 bytes. */
BUILD_ASSERT(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) >= 32,
         "DWC3 event buffer must be at least 32 bytes");
/* The local drain buffer must hold a whole pass. */
BUILD_ASSERT(ARRAY_SIZE(((struct udc_dwc3_data *)0)->evt.copy) >=
         CONFIG_UDC_DWC3_EVENTS_NUM,
         "evt.copy must hold a full drain: udc_dwc3_evt_drain() clamps 'want' "
         "to CONFIG_UDC_DWC3_EVENTS_NUM and indexes evt.copy by it");

/*
 * The 64-byte cap is not from the databook, which allows up to 64KB
 * (GEVNTSIZ.EVENTSIZ is a 16-bit byte count). Both reasons are in the message.
 */
BUILD_ASSERT(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) <= 64,
         "DWC3 event ring is capped by the AXI block on this part, and "
         "evt.copy - the local pre-credit copy - stays at 64 bytes");

/*
 * Number of fast polls for the event word (a count, not a time). A wall-clock
 * ceiling also applies; it is defined at the top of this file, next to the
 * SETUP report threshold whose floor it sets.
 */
#define UDC_DWC3_EVT_ARRIVE_FAST_POLLS      16u
/*
 * Slow phase, between passes (udc_dwc3_event_thread()): length of each timed
 * sleep, and how many of them per late-write attempt.
 */
#define UDC_DWC3_EVT_SLOW_POLL_MS           10u
#define UDC_DWC3_EVT_ARRIVE_SLOW_POLLS      8u
/*
 * NUDGE command, issued only to make the controller write an event. 3.2.1
 * Table 3-2 says of command 02h "the controller does not use the programmed
 * value", so it is the one generic command with no side effect. Its completion
 * event is ignored.
 */
#define UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS  0x02u

/*
 * Issue a force only while the ring is at most half full, leaving room for the
 * forced event and for ongoing link events.
 */
#define UDC_DWC3_EVT_FORCE_MAX_GEVNTCOUNT           \
    ((CONFIG_UDC_DWC3_EVENTS_NUM / 2u) * sizeof(uint32_t))
/* Minimum gap between forced commands, so a stuck slot cannot cause a command storm. */
#define UDC_DWC3_EVT_FORCE_MIN_GAP_MS   500u

/* Fast phase: busy-wait between two reads of the event word. */
#define UDC_DWC3_EVT_ARRIVE_POLL_US     1u

/*
 * Make the controller write an event, to release a slot that will not fill on
 * its own. See UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS for the choice of command.
 *
 * Fire and forget: the completion (device event 10) is ignored by the dispatch
 * and never waited for.
 */
static UDC_DWC3_COLD void udc_dwc3_evt_force(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    /*
     * Only while the ring has room: each forced command adds an event, which
     * queues behind the stuck head slot.
     */
    if (priv->evt.gc_last > UDC_DWC3_EVT_FORCE_MAX_GEVNTCOUNT) {
        return;
    }

    /* Not issued over a command still executing; not waited for (see above). */
    if (udc_dwc3_dgcmd(dev, UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS | UDC_DWC3_DGCMD_IOC,
               0U, false) != 0) {
        return;
    }

    /*
     * Logged outside the lock: the FLIR build uses CONFIG_LOG_MODE_MINIMAL,
     * which writes to the UART in the caller's context (~45 chars, ~3.9 ms at
     * 115200).
     */
    LOG_WRN_RATELIMIT("evNUDGE s%u",
              priv->evt.next);
}

/*
 * Issue the DGCMD the drain requested. The drain must not write DGCMD itself,
 * so it hands the request here. Rate-limited by UDC_DWC3_EVT_FORCE_MIN_GAP_MS.
 */
static void udc_dwc3_nudge_worker(struct k_work *const work)
{
    struct udc_dwc3_data *const priv =
        CONTAINER_OF(work, struct udc_dwc3_data, nudge_work);
    const struct device *const dev = priv->dev;

    if (k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt.force_t0) <
                    UDC_DWC3_EVT_FORCE_MIN_GAP_MS) {
        return;
    }

    udc_dwc3_evt_force(dev);
    priv->evt.force_t0 = k_cycle_get_32();
}

/*
 * The heartbeat's only job on the event ring: restart the drain if it has
 * stopped while the controller still owes events. Nothing more is safe from
 * this context.
 */
static UDC_DWC3_COLD void udc_dwc3_drain_helper(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

    /*
     * The heartbeat's one GEVNTCOUNT read. The heartbeat worker and
     * udc_dwc3_evt_force() use priv->evt.gc_last instead of the register.
     */
    priv->evt.gc_last = udc_dwc3_gevntcount(base);

    if (priv->evt.gc_last == 0U) {
        return;
    }

    if (k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt.worker_exit_t0) >=
                    UDC_DWC3_EVT_IDLE_KICK_MS) {
        priv->diag.evt_kick++;
        k_sem_give(&priv->evt_sem);
    }
}

/*
 * Wait for the first word of a pass. Only this word is worth waiting for: with
 * nothing copied yet, returning without it would just spin. Busy-wait only: the
 * longer, sleeping wait is the event thread's, between passes, so a pass never
 * sleeps with the UDC mutex or the ring state in hand.
 * Returns UDC_DWC3_WAIT_ARRIVED with *evt_out set, or UDC_DWC3_WAIT_EXPIRED.
 */
static enum udc_dwc3_wait_result udc_dwc3_evt_wait_first(const struct device *const dev,
                             const uint32_t gc,
                             uint32_t evt_idx,
                             uint32_t *const evt_out)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const uint32_t t0 = k_cycle_get_32();
    const uint32_t deadline = t0 + k_ms_to_cyc_ceil32(UDC_DWC3_EVT_ARRIVE_MAX_MS);
    uint32_t polls = 0;
    uint32_t evt;
    int32_t  wait_cycles;
    enum udc_dwc3_wait_result wait_result = UDC_DWC3_WAIT_EXPIRED;

    /* Bounded busy-wait at microsecond resolution, no yield. */
    *evt_out = UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
    evt_idx %= CONFIG_UDC_DWC3_EVENTS_NUM;
    while ((evt = cfg->evt_buf[evt_idx]) == UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE)
    {
        wait_cycles = (int32_t)(deadline - k_cycle_get_32());
        if ((polls++ >= UDC_DWC3_EVT_ARRIVE_FAST_POLLS) || (wait_cycles <= 0))
        {
            break;
        }
        k_busy_wait(UDC_DWC3_EVT_ARRIVE_POLL_US);
    }

    /* Accumulate time spent looking; report a missed event once per episode. */
    uint32_t waited_us = k_cyc_to_us_near32(k_cycle_get_32() - t0);
    priv->evt.drain.watched_us += waited_us;
    if (!priv->evt.drain.counted &&
         k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt.drain.since) >= UDC_DWC3_EVT_MISSED_MS)
    {
         const uint32_t gc_now = gc;    /* this pass's single read   */
         const bool frozen = (gc_now == priv->evt.drain.gc0);  /* vs episode open */
         priv->evt.drain.counted = true;
         priv->diag.evt_missed++;
         if (frozen) {
             priv->diag.evt_missed_frozen++;
         }
         priv->evt.drain.quiet = false;

         LOG_ERR("evLOST s%u age%ums n%u gc%u/%u %s",
                 priv->evt.next,
                 k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt.drain.since),
                 priv->evt.drain.attempts, gc_now, priv->evt.drain.gc0,
                 frozen ? "frz" : "adv");
    }

    if (evt != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE)
    {
        /* The write landed. The caller ends the episode and sets the state. */
        *evt_out = evt;
        wait_result = UDC_DWC3_WAIT_ARRIVED;

        /* Update the sticky maxima. */
        if (polls > priv->diag.evt_late_polls_max) {
            priv->diag.evt_late_polls_max = polls;
        }

        if (waited_us > priv->diag.evt_late_us_max) {
            priv->diag.evt_late_us_max = waited_us;
        }
    }
    return wait_result;

}

/*
 * Walk every endpoint and retire what the TRB rings say is already finished.
 */
static UDC_DWC3_COLD void udc_dwc3_evt_reconcile_endpoints(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    uint32_t rescued = 0U;
    bool ctrl_rearmed = false;

    udc_lock_internal(dev, K_FOREVER);

    for (uint32_t pass = 0U; pass < 2U; pass++) {
        struct udc_dwc3_ep_data *const eps =
            (pass == 0U) ? cfg->ep_data_in : cfg->ep_data_out;
        const uint32_t n = (pass == 0U) ? cfg->num_in_eps : cfg->num_out_eps;

        for (uint32_t i = 0U; i < n; i++) {
            struct udc_dwc3_ep_data *const e = &eps[i];

            if (!e->cfg.stat.enabled) {
                continue;
            }

            /*
             * Non-control: udc_dwc3_pop_trb() checks whether the
             * controller wrote the TRB back. The TRB is not affected
             * by a lost event, so no other test is needed.
             */
            if (USB_EP_GET_IDX(e->cfg.addr) != 0U) {
                rescued += udc_dwc3_drain_completed(dev, e);
                continue;
            }

            /*
             * EP0 (where this device actually fails): re-arm SETUP only
             * if the control machine is CTRL_IDLE and none is armed.
             *
             * "No SETUP armed" alone does not mean EP0 is stuck: DATA
             * and STATUS stages have none, and the lost event may have
             * belonged to any endpoint. Any state other than CTRL_IDLE
             * is a transfer in progress; leave it to finish or to the
             * control recovery.
             */
            if (USB_EP_DIR_IS_OUT(e->cfg.addr) && !ctrl_rearmed &&
                priv->ctrl.state == UDC_DWC3_CTRL_IDLE &&
                !udc_dwc3_ctrl_armed_setup(e)) {
                ctrl_rearmed = true;
                udc_dwc3_ctrl_next(dev);
            }
        }
    }

    udc_unlock_internal(dev);

    if (rescued > 0U || ctrl_rearmed) {
        priv->diag.evt_sweep_rescued += rescued;
        priv->diag.evt_sweep_runs++;
        LOG_WRN("discard reconciled: retired %u completion(s)%s (%u buffers "
            "over %u discards)", rescued,
            ctrl_rearmed ? " and re-armed a stopped control stage" : "",
            priv->diag.evt_sweep_rescued, priv->diag.evt_sweep_runs);
    }
}

/*
 * Decide how many event slots the controller will never fill, and step over
 * them. Returns the number skipped; the caller adds it to the pass's single
 * GEVNTCOUNT credit.
 *
 * Touches no register. GEVNTCOUNT is decrement-on-write and updated by the
 * controller concurrently, so a credit cannot be verified by reading it back
 * (a concurrent event write looks like a refused credit). The pass credits
 * everything it consumed before returning, so nothing needs verifying.
 *
 * A full ring is recoverable: the controller queues events internally and
 * writes them out once software frees space. For the overflow event, "software
 * must free up space in the Event Buffer by acknowledging more than 1 event
 * (writing a value greater than 4 to the GEVNTCOUNTn register)".
 */
static UDC_DWC3_COLD uint32_t udc_dwc3_evt_skip_dead_slot(const struct device *const dev,
                    const uint32_t gc, const bool frozen,
                    const uint32_t gaveup_ms)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    /* The caller decided the slot is dead; this only acts. Words owed: */
    uint32_t owed = gc / sizeof(uint32_t);
    if (owed > (CONFIG_UDC_DWC3_EVENTS_NUM - 1u)) {
        owed = CONFIG_UDC_DWC3_EVENTS_NUM - 1u;
    }

    /*
     * How many slots to skip (the controller fills the ring in order):
     *   - Written slot at P+j: the j slots before it were passed over and are
     *     lost. Skip exactly those; the written slot is read normally.
     *   - No written slot in the owed range: nothing is proved, the words may
     *     still be in flight. Skip all but the last and hold that one back.
     *
     * The held slot is a detector. It lies inside the stuck group, so no new
     * event can land in it; if it fills, the skipped slots filled too and the
     * skip was wrong. udc_dwc3_copy_valid_event() counts that as
     * evt_skip_refuted.
     */
    uint32_t skip = 1;
    bool     held_back = false;

    if (owed > 1)
    {
        uint32_t j = 1;
        bool slot_valid = false;
        for (; j < owed; j++) {
            const uint32_t idx = (priv->evt.next + j) % CONFIG_UDC_DWC3_EVENTS_NUM;
            if (cfg->evt_buf[idx] != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
                slot_valid = true;
                break;
            }
        }
        skip = (true == slot_valid) ? j : (j - 1);
        held_back = !slot_valid;
    }

    /*
     * Act first, log after: the log line costs ~16 ms of synchronous UART
     * output. Skipped slots already hold the sentinel, so nothing is written
     * back; re-writing it could only erase an event that landed meanwhile.
     */
    {
        const uint32_t held         = cfg->evt_buf[priv->evt.next];
        const uint32_t was_slot     = priv->evt.next;
        const uint32_t was_attempts = priv->evt.drain.attempts;
        const uint32_t was_watched  = priv->evt.drain.watched_us;

        priv->evt.next       = (priv->evt.next + skip) % CONFIG_UDC_DWC3_EVENTS_NUM;
        priv->diag.evt_skipped   += skip;
        priv->evt.drain.attempts = 0;

        /*
         * Watch only a held slot. Otherwise (owed was 1, or a written slot
         * proved the loss) the next slot is where the controller writes next,
         * and a new event there refutes nothing.
         */
        priv->evt.drain.skip_watch      = held_back;
        priv->evt.drain.skip_watch_slot = priv->evt.next;

        /*
         * Fields: h = slot value, skip = slots skipped, tot = cumulative,
         * age = wall-clock, w = time actually spent looking (less than age by
         * console output and time not scheduled).
         */
        LOG_ERR("evSKIP s%u age%ums w%ums n%u gc%u h%08x %s skip%u tot%u",
            was_slot, gaveup_ms, was_watched / 1000U, was_attempts,
            gc, held, frozen ? "frz" : "adv", skip, priv->diag.evt_skipped);
    }

    return skip;
}

/*
 * Return true if the event the drain is waiting for is lost for good.
 *
 * Both tests use drain.watched_us, the time actually spent looking at the
 * slot. Wall-clock age is only for the log: it also counts console output and
 * time this thread was not scheduled.
 */
static bool udc_dwc3_drain_slot_is_dead (const struct device *const dev,
                                         const uint32_t gc)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    if (cfg->evt_buf[priv->evt.next] != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
        return false;
    }

    /*
     * Proof, with its own shorter floor: a written slot ahead of this one
     * means the event is lost, since the controller fills the ring in order.
     *
     * GEVNTCOUNT alone proves nothing: gc = 8 with both slots empty may just
     * be two late events. Only a written slot ahead tells late from lost.
     */
    uint32_t wait_tims_ms = priv->evt.drain.watched_us / 1000U;
    if ((wait_tims_ms >= UDC_DWC3_EVT_LOOKAHEAD_MIN_MS) &&
        (true == udc_dwc3_evt_lookahead_lost(dev, gc)))
    {
        priv->diag.evt_lookahead_short++;
        return true;
    }

    /* No proof: fall back on the timeouts, above the minimum floor. */
    if (wait_tims_ms < UDC_DWC3_EVT_DEAD_SLOT_MIN_MS) {
        return false;
    }

    if (wait_tims_ms >= UDC_DWC3_EVT_DEAD_SLOT_MS ||
        priv->evt.drain.attempts >= UDC_DWC3_EVT_DEAD_SLOT_GIVEUPS) {
        return true;
    }

    return false;
}

/*
 * Take one event out of the ring: copy it to priv->evt.copy[copy_idx] and mark
 * the slot empty. The caller owns evt_idx and advances it afterwards.
 *
 * The slot is stamped CONSUMED here, not at the GEVNTCOUNT credit (written once
 * at the end of the pass), so "is this slot written?" is correct on the next lap.
 *
 * Also closes out an arrival wait if one preceded this event. Called from
 * DRAIN_RUNNING (slot already written) and DRAIN_WAITING (late write arrived);
 * doing the bookkeeping here keeps attempts, since and quiet from carrying over
 * to the next event.
 */
static void udc_dwc3_copy_valid_event (struct udc_dwc3_data *const priv,
                                       const struct udc_dwc3_config *const cfg,
                                       uint32_t evt_idx, uint32_t evt, uint32_t copy_idx)
{
    priv->evt.copy[copy_idx]   = evt;
    cfg->evt_buf[evt_idx % CONFIG_UDC_DWC3_EVENTS_NUM] = UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;

    /*
     * If the held-back slot filled, the words were late, not lost, and the
     * skipped slots were live events thrown away. One shot: the watch is
     * dropped on the first event either way, so a later event is not miscounted.
     */
    if (priv->evt.drain.skip_watch) {
        if ((evt_idx % CONFIG_UDC_DWC3_EVENTS_NUM) == priv->evt.drain.skip_watch_slot) {
            priv->diag.evt_skip_refuted++;
        }
        priv->evt.drain.skip_watch = false;
    }

    /* Only when an arrival wait preceded this event. */
    if (priv->evt.drain.since > 0)
    {
        /* How long the slot stayed empty (the key number for the RTL question). */
        if (priv->evt.drain.quiet) {
            const uint32_t us = k_cyc_to_us_near32(
                k_cycle_get_32() - priv->evt.drain.since);

            if (us > priv->diag.evt_gaveup_us_max) {
                priv->diag.evt_gaveup_us_max = us;
            }
        }
        priv->evt.drain.attempts = 0;
        priv->evt.drain.since    = 0;
        priv->evt.drain.quiet    = false;
    }
    return;

}

/*
 * Copy the ring into priv->evt.copy[] and return every slot in one GEVNTCOUNT
 * acknowledge, before any event is dispatched. Returns the number of events
 * copied.
 *
 * One acknowledge of everything is also how the databook escapes an overflow:
 * "software must free up space in the Event Buffer by acknowledging more than
 * 1 event".
 */
static uint32_t udc_dwc3_evt_drain(const struct device *const dev, const bool first,
                   const bool last, bool *const head_pending,
                   bool *const reconcile)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    enum udc_dwc3_drain_state evt_drain_state;

    /*
     * The pass's only GEVNTCOUNT read, taken before any event is processed as
     * section 1.2.56 requires (see UDC_DWC3_GEVNTCOUNT_MASK). Also updates the
     * high-water mark.
     */
    const uint32_t gc = udc_dwc3_gevntcount(base);
    priv->evt.gc_last = gc;
    if (gc > priv->diag.evt_gevntcount_hwm) {
        priv->diag.evt_gevntcount_hwm = gc;
    }

    /* Bound the register-supplied count before indexing. */
    uint32_t want = gc / sizeof(uint32_t);
    if (want > CONFIG_UDC_DWC3_EVENTS_NUM) {
        LOG_ERR_RATELIMIT("GEVNTCOUNT reports %u B, more than the %u B ring",
                  gc, (unsigned int)(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t)));

        want = CONFIG_UDC_DWC3_EVENTS_NUM;
    }

    /* Start at the ring head. */
    uint32_t n       = 0;   /* valid events read in this pass */
    uint32_t skipped = 0;   /* dead slots stepped over in this pass */
    uint32_t evt_idx = priv->evt.next;
    bool exit_loop   = false;
    evt_drain_state  = priv->evt.drain.state;

    /*
     * Read until every owed slot is accounted for, read or skipped, or the pass
     * must stop. A skipped slot is owed too: reading past 'want' would open a
     * wait on a slot the controller has not been asked to fill.
     */
    while (((n + skipped) < want) && (false == exit_loop))
    {
        uint32_t evt = cfg->evt_buf [evt_idx % CONFIG_UDC_DWC3_EVENTS_NUM];
        if (evt != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
            evt_drain_state = UDC_DWC3_DRAIN_RUNNING;
        }

        switch (evt_drain_state)
        {
        case UDC_DWC3_DRAIN_IDLE:
        {
            evt_drain_state = UDC_DWC3_DRAIN_RUNNING;
            __fallthrough;
        }

        case UDC_DWC3_DRAIN_RUNNING:
        {
            if (evt != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE)
            {
                udc_dwc3_copy_valid_event (priv, cfg, evt_idx, evt, n);
                evt_idx++;
                n++;
            }
            else
            {
                if (n > 0)
                {
                    /*
                     * Some events were read, but later ones counted by
                     * GEVNTCOUNT have not landed yet. Stop here; a later
                     * pass picks up the late events.
                     */
                    priv->diag.evt_midzero++;
                }
                /* Open a new late-write episode. */
                evt_drain_state        = UDC_DWC3_DRAIN_WAITING;
                priv->evt.drain.attempts   = 0;
                priv->evt.drain.since      = k_cycle_get_32();
                priv->evt.drain.watched_us = 0U;
                priv->evt.drain.quiet      = true;
                priv->evt.drain.counted    = false;
                priv->evt.drain.slot       = evt_idx % CONFIG_UDC_DWC3_EVENTS_NUM;
                exit_loop              = true;
            }
            break;
        }

        case UDC_DWC3_DRAIN_WAITING:
        {
            enum udc_dwc3_wait_result   drain_result;

            /* Once per episode: its first attempt's first try. */
            if (0 == priv->evt.drain.attempts && first) {
                priv->evt.drain.gc0 = gc;
                if (priv->evt.drain.gc0 > sizeof(uint32_t)) {
                    priv->diag.evt_gaveup_multi++;
                }
            }

            drain_result = udc_dwc3_evt_wait_first (dev, gc, evt_idx, &evt);
            if (drain_result == UDC_DWC3_WAIT_ARRIVED)
            {
                priv->diag.evt_late++;
                udc_dwc3_copy_valid_event (priv, cfg, evt_idx, evt, n);
                evt_idx++;
                n++;
                evt_drain_state = UDC_DWC3_DRAIN_RUNNING;
            }
            else if (drain_result == UDC_DWC3_WAIT_EXPIRED)
            {
                /* Slot still unwritten: end the pass. */
                exit_loop = true;

                /*
                 * Not the attempt's last try: the event thread sleeps
                 * UDC_DWC3_EVT_SLOW_POLL_MS and runs another pass, which
                 * re-reads GEVNTCOUNT and the head.
                 */
                if (!last) {
                    *head_pending = true;
                    break;
                }
                priv->evt.drain.attempts++;

                /*
                 * After the first failed attempt, request a NUDGE: the event
                 * may be stuck in the bus FIFO. Request only;
                 * udc_dwc3_nudge_worker() issues the DGCMD.
                 */
                if (priv->evt.drain.attempts == 1U) {
                    k_work_submit_to_queue (udc_get_work_q(), &priv->nudge_work);
                    priv->diag.evt_gaveup++;
                }
                else {
                    /* How long the slot has been unwritten. */
                    const uint32_t age_ms =
                        k_cyc_to_ms_near32 ( k_cycle_get_32() - priv->evt.drain.since);

                    /*
                     * The NUDGE did not help either. Skip the slot if it is
                     * dead: unwritten too long, or a later slot is written.
                     */
                    priv->evt.next = evt_idx % CONFIG_UDC_DWC3_EVENTS_NUM;
                    if (udc_dwc3_drain_slot_is_dead(dev, gc))
                    {
                        /*
                         * Step over the slot and add its words to this pass's
                         * credit. Cannot fail: it only moves the read pointer.
                         */
                        skipped += udc_dwc3_evt_skip_dead_slot ( dev, gc,
                            (gc == priv->evt.drain.gc0), age_ms );
                        /* After the write-back: it takes the UDC mutex. */
                        *reconcile = true;
                        evt_drain_state = UDC_DWC3_DRAIN_RUNNING;
                    }
                    evt_idx = priv->evt.next;   /* Read updated next-index */
                }
            }
            break;
        }

        default:
        {
            /* The state is only ever one of the three above. */
            CODE_UNREACHABLE;
        }
        }
    }

    /* Save the next ring index and the drain state. */
    priv->evt.next = evt_idx % CONFIG_UDC_DWC3_EVENTS_NUM;
    priv->evt.drain.state = evt_drain_state;

    /*
     * The pass's only GEVNTCOUNT write: events read plus dead slots skipped.
     * Zero is valid (it credits nothing and clears EVNT_HANDLER_BUSY), so the
     * write is unconditional.
     */
    udc_dwc3_gevntcount_ack (base, (n + skipped));

    /*
     * A pass that consumed everything ends IDLE, so dumps do not report a
     * running drain that is doing nothing.
     */
    if (priv->evt.drain.state == UDC_DWC3_DRAIN_RUNNING && (n + skipped) >= want) {
        priv->evt.drain.state = UDC_DWC3_DRAIN_IDLE;
    }

    return n;
}


static bool udc_dwc3_event_drain_once(const struct device *const dev, const bool first,
                      const bool last);

/*
 * Event drain thread, woken by evt_sem.
 *
 * It cannot share udc_get_work_q() with ep_data->work and heartbeat_work. The
 * copy and credit need no UDC mutex, but the dispatch after them takes it; on a
 * shared queue a blocked dispatch would hold up the next drain. Section 3.2.2.5:
 * "Software must always service the event interrupts generated by the controller."
 *
 * A late head slot (the controller counted an event it has not written yet) is
 * waited for here, between passes, not inside one: up to
 * UDC_DWC3_EVT_ARRIVE_SLOW_POLLS sleeps of UDC_DWC3_EVT_SLOW_POLL_MS, capped at
 * UDC_DWC3_EVT_ARRIVE_MAX_MS, with the event interrupt still masked. Each retry
 * is a fresh pass that re-reads GEVNTCOUNT and the head, so a sleep never holds
 * ring state. The heartbeat stays only the backstop.
 */
static void udc_dwc3_event_thread(void *const p1, void *const p2, void *const p3)
{
    struct udc_dwc3_data *const priv = p1;
    const struct device *const dev = priv->dev;

    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    for (;;) {
        k_sem_take(&priv->evt_sem, K_FOREVER);

        /* The normal pass: nothing beyond the pass itself. */
        if (!udc_dwc3_event_drain_once(dev, true, false)) {
            continue;
        }

        /* Head slot late: the retries, and only they, pay for the clock. */
        const uint32_t t0 = k_cycle_get_32();
        uint32_t retries = 0U;
        bool last;

        do {
            retries++;
            k_sleep(K_MSEC(UDC_DWC3_EVT_SLOW_POLL_MS));
            last = retries >= UDC_DWC3_EVT_ARRIVE_SLOW_POLLS ||
                   k_cyc_to_ms_floor32(k_cycle_get_32() - t0) >=
                   UDC_DWC3_EVT_ARRIVE_MAX_MS;
        } while (udc_dwc3_event_drain_once(dev, false, last));
    }
}

/*
 * One pass of the event drain: copy and GEVNTCOUNT credit without the UDC mutex
 * and without sleeping or blocking, then dispatch under the mutex. Skipped
 * entirely while the ring is being re-initialised (udc_dwc3_evt_block()); the
 * interrupt then stays masked until udc_dwc3_enable(). 'first'/'last': the
 * head-slot wait's first and final try; the final one counts the attempt
 * (nudge, dead-slot skip). Returns true, with nothing copied and the interrupt
 * still masked, if the head slot is still unwritten and the caller should sleep
 * and run another pass.
 */
static bool udc_dwc3_event_drain_once(const struct device *const dev, const bool first,
                      const bool last)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    bool head_pending = false;
    bool reconcile = false;
    uint32_t n;

    if (priv->run.state == UDC_DWC3_RUN_RESETTING) {
        return false;
    }

    priv->diag.evt_worker_runs++;
    n = udc_dwc3_evt_drain(dev, first, last, &head_pending, &reconcile);

    /* A dead slot was skipped: retire what its lost event would have. */
    if (reconcile) {
        udc_dwc3_evt_reconcile_endpoints(dev);
    }

    if (n > 0U) {
        priv->diag.dispatch_t0 = k_cycle_get_32();
        udc_lock_internal(dev, K_FOREVER);
        /*
         * RunStop clear: a disable or the controller recovery stopped the
         * controller while this pass waited for the mutex, and has settled or
         * forgotten every transfer itself. The events describe the session it
         * tore down; dispatching them would act on endpoints it reset (e.g. arm
         * a SETUP, or report a reset to the stack). Dropped, still counted as
         * handled: the ring did advance.
         *
         * Read from driver state; DCTL only while STOPPING. RunStop is clear
         * (a) before an enable and after a disable: udc_dwc3_stack_enabled() is
         * false, written with RunStop under the mutex held here; (b) in the recovery
         * from its STEP 2 on: STOPPING, which also covers its STEP 1, where
         * RunStop is still set and the End Transfer completions must be
         * dispatched, hence the read; (c) after a recovery that could not
         * reconnect: the core was reset and never started (no session for
         * events to describe), with the IRQ masked and the heartbeat stopped.
         */
        if (!udc_dwc3_stack_enabled(dev) ||
            (priv->run.state == UDC_DWC3_RUN_STOPPING &&
             (sys_read32(DEVICE_MMIO_NAMED_GET(dev, base) + UDC_DWC3_DCTL) &
              UDC_DWC3_DCTL_RUNSTOP) == 0U)) {
            LOG_DBG("%u events dropped: controller stopped", n);
            priv->evt.handled += n;
        } else {
            for (uint32_t i = 0; i < n; i++) {
                udc_dwc3_dispatch_event(dev, priv->evt.copy[i]);
                priv->evt.handled++;
            }
            priv->diag.dispatch_evt = 0U;
        }
        udc_unlock_internal(dev);

        /* One line per pass, outside the mutex: see evt.post_fail. */
        if (priv->evt.post_fail != 0U) {
            priv->diag.post_fail_total += priv->evt.post_fail;
            LOG_ERR("Failed to submit %u buffers (%u total)",
                priv->evt.post_fail, priv->diag.post_fail_total);
            priv->evt.post_fail = 0U;
        }
    }

    /* Retry: the interrupt stays masked, or the owed count would re-raise it at once. */
    if (head_pending) {
        return true;
    }

    /*
     * Unmask. Events still owed raise the interrupt again, which starts the
     * next pass; the heartbeat (udc_dwc3_drain_helper()) is the backstop.
     */
    udc_dwc3_evt_irq(dev, true);

    /*
     * Done last, so it records when the pass finished; see
     * UDC_DWC3_EVT_IDLE_KICK_MS for why the exit and not the entry.
     */
    priv->evt.worker_exit_t0 = k_cycle_get_32();

    return false;
}

/*
 * Event interrupt: mask, and wake the drain thread.
 */
static void udc_dwc3_irq_handler(void *const ptr)
{
    const struct device *const dev = ptr;
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    priv->diag.evt_isr++;

    k_sem_give(&priv->evt_sem);

    /* Disable further interrupts until all events are processed */
    udc_dwc3_evt_irq(dev, false);
}

/*
 * UDC API: the interface the Zephyr USB stack calls.
 */

static int udc_dwc3_ep_enqueue(const struct device *const dev,
                   struct udc_ep_config *const ep_cfg,
                   struct net_buf *const buf)
{
    struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
    const struct udc_buf_info bi = *udc_get_buf_info(buf);

    LOG_DBG("enq %p d=%p sz=%u ln=%u EP%02x %u:%u:%u",
        buf, buf->data, buf->size, buf->len, ep_cfg->addr, bi.setup, bi.data, bi.status);

    if (ep_data->cfg.addr == USB_CONTROL_EP_OUT) {
        memset(buf->data, 0x00, buf->size);
    }

    udc_buf_put(ep_cfg, buf);

    if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
        udc_dwc3_ctrl_next(dev);
    } else {
        /*
         * Process this buffer along with other waiting. Whether a transfer may
         * start (controller running, endpoint in DALEPENA) is the worker's
         * check, made under the mutex from driver state.
         */
        if (ep_cfg->stat.enabled) {
            LOG_DBG("submitting to EP%02x", ep_cfg->addr);
            k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
        }
    }

    return 0;
}

/*
 * UDC API: cancel queued buffers on an endpoint. The buffers the ring holds go
 * back through the endpoint recovery once the controller has let go of them.
 */
static UDC_DWC3_COLD int udc_dwc3_ep_dequeue(const struct device *const dev,
                   struct udc_ep_config *const ep_cfg)
{
    struct udc_dwc3_ep_data *const ep_data =
        CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

    if (USB_EP_GET_IDX(ep_cfg->addr) != 0U) {
        ep_data->xfer.pending |= UDC_DWC3_EP_PEND_DEQUEUE;
        udc_dwc3_ep_recover(dev, ep_data);
    }

    udc_ep_cancel_queued(dev, ep_cfg);

    return 0;
}

/*
 * Re-establish an endpoint: configure it (DEPCFG Init), enable it in DALEPENA and
 * arm whatever is queued. Reached only from an enable (udc_dwc3_ep_enable(), or
 * deferred, see below, through udc_dwc3_ep_recover()); the stack never
 * enables an endpoint that is already enabled, so DEPCFG Modify is not used here.
 */
static UDC_DWC3_COLD int udc_dwc3_ep_resume(const struct device *const dev,
                  struct udc_dwc3_ep_data *const ep_data)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct net_buf *buf;
    int ret;

    /*
     * Deferred (PEND_RESUME, run by udc_dwc3_ep_recover()) while:
     *   - the endpoint is not IDLE: a transfer or an open command outcome is
     *     still there (e.g. a disable whose End has not reported, or a Start
     *     still open). Clear Stall, DEPCFG, the ring rebuild and Start Transfer
     *     would act on a ring the controller owns (3.2.2.7). Its completion, or
     *     the heartbeat sweep (resume owed), re-enters the recovery, which ends
     *     the transfer first;
     *   - the controller recovery is stopping the device: 4.1.8 allows no new
     *     transfer once they are being ended. The reset that follows forgets it.
     */
    if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 &&
        (ep_data->xfer.state != UDC_DWC3_EP_IDLE || udc_dwc3_run_halting(priv))) {
        LOG_DBG("EP%02x %s%s, deferring resume", ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state),
            udc_dwc3_run_halting(priv) ? ", controller stopping" : "");
        ep_data->xfer.pending |= UDC_DWC3_EP_PEND_RESUME;
        return 0;
    }

    /* Leave any halt on a non-control endpoint, IN or OUT. */
    if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
        udc_dwc3_depcmd_clear_stall(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
    }

    udc_dwc3_depcmd_ep_config(dev, ep_data, false);

    /*
     * INVARIANT 4: DEPXFERCFG only on enable, and only once per pool generation.
     * priv->epcfg.epoch is the device's generation (bumped by each DEPSTARTCFG,
     * the only command that frees resources); ep_data->rsc_epoch is the
     * generation in which this endpoint took its transfer resource. Equal values
     * mean it already holds one.
     *
     * An alt-setting switch is disable/enable, and re-enabling an endpoint other
     * than epcfg.first_ep runs no DEPSTARTCFG. Without the epoch check each switch
     * leaks a transfer resource (End Transfer does not return it) until Start
     * Transfer fails with CmdStatus 4'h1 permanently.
     */
    if (ep_data->rsc_epoch != priv->epcfg.epoch) {
        udc_dwc3_depcmd_ep_xfer_config(dev, ep_data);
        ep_data->rsc_epoch = priv->epcfg.epoch;
    }

    if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
        /*
         * The ring is rebuilt from empty. Anything still on it (the endpoint is
         * IDLE, so the controller owns none of it) is parked first and re-armed
         * below with the rest, so head, tail and net_buf[] match the TRBs.
         */
        udc_dwc3_ep_ring_release(ep_data);
        ret = udc_dwc3_trb_nonctrl_init(dev, ep_data);
        if (ret != 0) {
            return ret;
        }
    }

    /* Starting from here, the endpoint can be used */
    udc_dwc3_dalepena_set(dev, ep_data->epn, true);

    /*
     * Re-arm the parked buffers (left by a refused Start, or released from the
     * ring above). Peek, arm, then remove, as udc_dwc3_ep_worker() does, for the
     * same reason.
     */
    while (true) {
        buf = k_fifo_peek_head(&ep_data->requeue_fifo);
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
            /* Not armed - it stays in the fifo for the next resume. */
            return ret;
        }

        (void)k_fifo_get(&ep_data->requeue_fifo, K_NO_WAIT);
    }

    /* We might have blocked transfers earlier */
    if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
        k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
    }

    return 0;
}

/*
 * UDC API: enable an endpoint.
 */
static UDC_DWC3_COLD int udc_dwc3_ep_enable(const struct device *const dev,
                    struct udc_ep_config *const ep_cfg)
{
    struct udc_dwc3_ep_data *const ep_data = (struct udc_dwc3_ep_data *)ep_cfg;
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    LOG_DBG("EP%02x, first EP%02x", ep_data->cfg.addr, priv->epcfg.first_ep);

    /*
     * Refuse isochronous rather than mis-arm it (see caps.iso). The capability
     * is not advertised; this is the backstop for a class that uses one anyway.
     */
    if ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) ==
        USB_EP_TYPE_ISO) {
        LOG_ERR("EP%02x isochronous is not supported by this driver",
            ep_cfg->addr);
        return -ENOTSUP;
    }

    if (USB_EP_GET_IDX(ep_cfg->addr) > 0) {
        if (priv->epcfg.first_ep == 0) {
            priv->epcfg.first_ep = ep_cfg->addr;
        }
        if (ep_cfg->addr == priv->epcfg.first_ep && !priv->epcfg.pool_assigned) {
            priv->epcfg.pool_assigned = true;
            udc_dwc3_on_set_config_or_interface(dev);
        }
    }

    return udc_dwc3_ep_resume(dev, ep_data);
}

/*
 * UDC API: disable an endpoint. DALEPENA is cleared, then the endpoint recovery
 * ends any transfer and returns the armed buffers as cancelled once the
 * controller has let go of them.
 */
static UDC_DWC3_COLD int udc_dwc3_ep_disable(const struct device *const dev,
                    struct udc_ep_config *const ep_cfg)
{
    struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    LOG_DBG("Disabling EP%02x", ep_cfg->addr);

    /*
     * Drop any reference the control machinery holds to this endpoint before
     * tearing it down.
     */
    if (priv->diag.watchdog_ep == ep_data) {
        priv->diag.watchdog_ep = NULL;
        priv->diag.watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
    }

    /*
     * A resume or Clear Stall owed from before must not run against an
     * endpoint that is going away. The armed buffers are owed back cancelled:
     * usbd_ep_disable() follows with udc_ep_dequeue(), which reaches the driver
     * only for buffers still in the stack's queue, never for those on the ring.
     */
    ep_data->xfer.pending = UDC_DWC3_EP_PEND_DEQUEUE;

    udc_dwc3_dalepena_set(dev, ep_data->epn, false);

    if (USB_EP_GET_IDX(ep_cfg->addr) != 0U) {
        udc_dwc3_ep_recover(dev, ep_data);
    }

    return 0;
}

/*
 * UDC API: STALL an endpoint. cfg.stat.halted follows the hardware.
 */
static UDC_DWC3_COLD int udc_dwc3_ep_set_halt(const struct device *const dev,
                struct udc_ep_config *const ep_cfg)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

    /*
     * Log who halted the endpoint, to tell a stack-requested halt from one the
     * controller raised itself. A data-endpoint halt can be the first sign of
     * a failure that shows up elsewhere (e.g. control dying after the host
     * clears it).
     */
    LOG_INF("Set halt on EP%02x (requested by the stack)", ep_cfg->addr);

    switch (ep_data->cfg.addr) {
    case USB_CONTROL_EP_IN:
    case USB_CONTROL_EP_OUT:
        /* The stack rejected the control request: run control-endpoint recovery. */
        udc_dwc3_ctrl_ep_recover(dev);
        break;
    default:
        /*
         * cfg.stat.halted is written by udc_dwc3_depcmd_set_stall(),
         * after the hardware has actually taken the command.
         */
        if (!udc_dwc3_depcmd_set_stall(dev, ep_data)) {
            LOG_ERR("EP%02x Set Stall was refused by the controller; "
                "endpoint is NOT halted", ep_data->cfg.addr);
            return -EIO;
        }
        priv->diag.ep_halts++;
    }

    return 0;
}

/*
 * UDC API: ClearFeature(ENDPOINT_HALT). The host's request is recorded and the
 * endpoint recovery does the rest. -EIO when the recovery issued the Clear Stall
 * and the controller refused it (still halted), so udc_common keeps
 * cfg.stat.halted; 0 when it is done or deferred behind an End Transfer.
 */
static UDC_DWC3_COLD int udc_dwc3_ep_clear_halt(const struct device *const dev,
                  struct udc_ep_config *const ep_cfg)
{
    struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

    LOG_INF("Clearing stall for EP%02x", ep_cfg->addr);

    if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
        return 0;
    }

    ep_data->xfer.pending |= UDC_DWC3_EP_PEND_CLEAR_STALL;
    udc_dwc3_ep_recover(dev, ep_data);

    if ((ep_data->xfer.pending & UDC_DWC3_EP_PEND_CLEAR_STALL) == 0U &&
        ep_data->cfg.stat.halted) {
        return -EIO;
    }

    return 0;
}

/*
 * UDC API: address is taken from the SETUP packet, so nothing to do here.
 */
static int udc_dwc3_set_address_no_op(const struct device *const dev, const uint8_t addr)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(addr);
    return 0;
}

/*
 * Program DCFG.DevAddr.
 */
static UDC_DWC3_COLD int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr)
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

/*
 * UDC API: report the negotiated speed.
 */
static enum udc_bus_speed udc_dwc3_device_speed(const struct device *const dev)
{
    const enum udc_bus_speed speed = udc_dwc3_connect_speed(DEVICE_MMIO_NAMED_GET(dev, base));

    if (speed == UDC_BUS_UNKNOWN) {
        LOG_ERR("Unknown device speed");
    }

    return speed;
}

/*
 * UDC API: attach to the bus (DCTL.RunStop).
 */
static int udc_dwc3_enable(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    int ret;

    LOG_INF("Enabling DWC3 driver");

    ret = udc_dwc3_quirk_enable(dev);
    if (ret != 0) {
        return ret;
    }

    /*
     * U1/U2 off until the host configures the device: DCTL says software sets
     * AcceptU1/U2Ena "after receiving a SetConfiguration command" and
     * InitU1/U2Ena after SetFeature(U1/U2_ENABLE) - see
     * udc_dwc3_ctrl_apply_link_pm().
     */
    udc_dwc3_dctl_update(base,
                 UDC_DWC3_DCTL_ACCEPTU1ENA | UDC_DWC3_DCTL_INITU1ENA |
                 UDC_DWC3_DCTL_ACCEPTU2ENA | UDC_DWC3_DCTL_INITU2ENA, 0U);

    /*
     * Not over STOPPING: an enable from the stack during the controller
     * recovery's quiesce must not let transfers start; the recovery sets RUN
     * itself once it has reconnected.
     */
    if (priv->run.state != UDC_DWC3_RUN_STOPPING) {
        priv->run.state = UDC_DWC3_RUN;
    }
    udc_dwc3_dctl_update(base, 0U, UDC_DWC3_DCTL_RUNSTOP);

    /* Enable the event interrupt */
    udc_dwc3_evt_irq(dev, true);

    k_timer_start(&priv->heartbeat_timer, K_MSEC(UDC_DWC3_HEARTBEAT_MS),
              K_MSEC(UDC_DWC3_HEARTBEAT_MS));

    return 0;
}

/*
 * UDC API: detach from the bus, as a device-initiated disconnect (SPEC 3.30b 4.1.8
 * Table 4-7): end every active transfer and take EP0 back to Setup, then clear
 * RunStop, then forget the transfer state and return the buffers.
 *
 * udc_common calls this with the UDC mutex held, and the drain dispatches under
 * that mutex, so the End Transfer completions cannot arrive as events here: their
 * outcomes are read from DEPCMD (udc_dwc3_quiesce_settle()), bounded like the
 * controller recovery's quiesce. DEVCTRLHLT is not waited for: the controller
 * halts only once its events are acknowledged, which a drain pass blocked on this
 * mutex may not do. The controller recovery calls this with RunStop already
 * cleared, after its own quiesce and halt wait; only the teardown runs then.
 */
static int udc_dwc3_disable(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

    LOG_DBG("Disabling DWC3 driver");

    k_timer_stop(&priv->heartbeat_timer);

    if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0U) {
        const enum udc_dwc3_run_state prev = priv->run.state;
        uint32_t polls = 0U;

        /*
         * A beat or nudge queued before now would act on the stopped device
         * (a beat can even reconnect it through the controller recovery).
         */
        (void)k_work_cancel(&priv->heartbeat_work);
        (void)k_work_cancel(&priv->nudge_work);

        /* STOPPING: no Start or resume from here on (4.1.8). */
        priv->run.state = UDC_DWC3_RUN_STOPPING;
        udc_dwc3_end_all_transfers(dev, false);

        while (udc_dwc3_stop_blocker(dev) >= 0) {
            if (polls++ >= UDC_DWC3_HALT_POLLS) {
                LOG_WRN("physical EP %d still has a transfer after %u ms; "
                    "stopping anyway", udc_dwc3_stop_blocker(dev),
                    UDC_DWC3_HALT_POLLS * UDC_DWC3_HALT_POLL_MS);
                break;
            }
            k_sleep(K_MSEC(UDC_DWC3_HALT_POLL_MS));
            udc_dwc3_quiesce_settle(dev);
        }

        udc_dwc3_dctl_update(base, UDC_DWC3_DCTL_RUNSTOP, 0U);
        priv->run.state = prev;
    }

    /*
     * With RunStop cleared the controller raises no further Endpoint Command
     * Complete events, so anything still outstanding is stranded.
     */
    udc_dwc3_drop_xfer_state(dev, "controller disable");

    udc_dwc3_evt_irq(dev, false);

    return 0;
}

/*
 * Bring the controller up and enable the control endpoints; see udc_dwc3_init().
 */
static int udc_dwc3_init_core(const struct device *const dev)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    uint32_t reg;
    int ret;

    LOG_DBG("Initializing the DWC3 core");

    ret = udc_dwc3_quirk_init(dev);
    if (ret != 0) {
        return ret;
    }

    /*
     * Soft-reset the core and the USB2 and USB3 PHYs.
     *
     * Two 100 ms holds, both required: the first lets the PHY finish its
     * reset; the second gives the core stable PHY clocks before it leaves
     * reset (released onto an unstable PIPE clock, its registers read zero).
     *
     * Repeated if the register file does not come back; init fails if it never
     * does. A core still in reset ignores register writes, so continuing would
     * report a controller enabled that never moves data. Failing leaves the
     * device off the bus, which the caller can see.
     */
    for (uint32_t attempt = 1U; ; attempt++) {
        sys_set_bits(base + UDC_DWC3_GCTL, UDC_DWC3_GCTL_CORESOFTRESET);
        /*
         * GCTL.CoreSoftReset clears DALEPENA, as does the DCTL.CSftRst that
         * follows (3.30b register reset table): clear the driver's copy too.
         */
        ((struct udc_dwc3_data *)udc_get_private(dev))->epcfg.dalepena = 0U;
        sys_set_bits(base + UDC_DWC3_GUSB3PIPECTL, UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST);
        sys_set_bits(base + UDC_DWC3_GUSB2PHYCFG, UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST);
        k_sleep(K_MSEC(UDC_DWC3_PHY_RESET_MS));

        /* Release the USB2 and USB3 PHY resets first */
        sys_clear_bits(base + UDC_DWC3_GUSB3PIPECTL, UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST);
        sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG, UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST);
        k_sleep(K_MSEC(UDC_DWC3_PHY_RESET_MS));

        /* Then release the core reset */
        sys_clear_bits(base + UDC_DWC3_GCTL, UDC_DWC3_GCTL_CORESOFTRESET);

        if (udc_dwc3_wait_regfile_ready(dev)) {
            break;
        }

        if (attempt >= UDC_DWC3_CORE_RESET_ATTEMPTS) {
            LOG_ERR("core still in reset after %u reset sequences - "
                "not configuring it", UDC_DWC3_CORE_RESET_ATTEMPTS);
            return -EIO;
        }

        LOG_WRN("core did not leave reset, driving the sequence again "
            "(%u of %u)", attempt, UDC_DWC3_CORE_RESET_ATTEMPTS);
    }

    /* The USB core was reset, configure it as documented */
    ret = udc_dwc3_on_soft_reset(dev);
    if (ret != 0) {
        return ret;
    }

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

/*
 * UDC API: bring the controller up and enable the control endpoints. Called by
 * the stack (UDC mutex held) and by udc_dwc3_controller_recover(). The drain is
 * shut out for the whole of it: the core reset and udc_dwc3_on_soft_reset()
 * re-initialise the event ring and the drain state, and the controller is not
 * running yet, so there is nothing to drain. On every return the run state is
 * restored; the recovery's own RESETTING stays until it ends.
 */
static int udc_dwc3_init(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const enum udc_dwc3_run_state prev = udc_dwc3_evt_block(dev);
    const int ret = udc_dwc3_init_core(dev);

    priv->run.state = prev;

    return ret;
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

/*
 * Endpoint work item: arm queued buffers. Takes the UDC mutex for its whole body.
 */
static void udc_dwc3_ep_worker(struct k_work *const work)
{
    struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(work, struct udc_dwc3_ep_data, work);
    const struct device *const dev = ep_data->dev;
    const struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct net_buf *buf;
    int ret;

    LOG_DBG("checking for pending transfers for EP%02x", ep_data->cfg.addr);

    /* Hold the UDC mutex, as every other TRB producer does. */
    udc_lock_internal(dev, K_FOREVER);

    /*
     * No new transfer while the controller is stopping or stopped: 4.1.8 ends
     * every active transfer before RunStop is cleared, and after that the
     * controller is halting, where commands are undefined. Buffers stay queued
     * for the next enable (udc_dwc3_ep_enable() kicks this worker).
     * Driver state, no register read: RunStop is clear only before an enable or
     * after a disable (udc_dwc3_stack_enabled() false; both written under the mutex
     * here), during the controller recovery (STOPPING until its last step), or
     * after a recovery whose re-init failed, where the core reset left every
     * endpoint out of DALEPENA (checked below).
     */
    if (udc_dwc3_run_halting(priv) || !udc_dwc3_stack_enabled(dev)) {
        goto unlock;
    }

    /*
     * Skip a disabled or halted endpoint. This worker can still be queued after
     * udc_dwc3_ep_disable() clears DALEPENA and releases the ring; a TRB armed then
     * goes to an endpoint the controller is not reading, and Update Transfer has
     * no resource index. stat.enabled is written under the mutex held here.
     * DALEPENA is checked too (the driver's copy): udc_dwc3_ep_start_refused() and
     * a core reset take an endpoint out of it while the stack still counts it
     * enabled.
     */
    if (!ep_data->cfg.stat.enabled || ep_data->cfg.stat.halted ||
        !udc_dwc3_ep_in_dalepena(priv, ep_data)) {
        LOG_DBG("endpoint is down or halted, not processing buffers");
        goto unlock;
    }

    /*
     * Defer while an End Transfer is outstanding or a Start/End outcome is
     * unknown: a TRB pushed now needs an Update Transfer on a resource being
     * ended, or not known to exist (3.2.2.7). The outcome's resolution re-enters
     * udc_dwc3_ep_recover(), which submits this work again.
     */
    if (udc_dwc3_ep_is_ending(ep_data) || udc_dwc3_ep_is_unknown(ep_data)) {
        LOG_DBG("EP%02x %s, deferring %s", ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state),
            (ep_data->xfer.pending & UDC_DWC3_EP_PEND_RESUME) != 0U
                ? "until the resume runs" : "buffers");
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

    /* Single controller instance; the stomp reporter needs a way to reach it. */
    udc_dwc3_stomp_priv = priv;
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
    /*
     * The event ring is drained by its own thread (udc_dwc3_event_thread()), not
     * the UDC work queue. The IRQ vector is connected once, here, not on every
     * enable.
     */
    if (cfg->irq_connect_func != NULL) {
        cfg->irq_connect_func();
    }

    k_sem_init(&priv->evt_sem, 0, 1);
    priv->diag.evt_stack_free = UDC_DWC3_EVT_STACK_SIZE;

    k_thread_create(priv->evt_thread, priv->evt_stack,
            UDC_DWC3_EVT_STACK_SIZE,
            udc_dwc3_event_thread, priv, NULL, NULL,
            UDC_DWC3_EVT_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(priv->evt_thread, "udc_dwc3_evt");

    k_work_init(&priv->heartbeat_work, udc_dwc3_heartbeat_worker);
    k_work_init(&priv->nudge_work, udc_dwc3_nudge_worker);
    k_sem_init(&priv->run.quiesce_sem, 0, 1);
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

    /*
     * Same initialisations as EP0-OUT below, for the same reasons; the
     * pre-init loops start at i = 1, so the control pair is set up by hand.
     */
    ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
    ep_data->xfer.end_idx = UDC_DWC3_XFERRSCIDX_INVALID;
    k_fifo_init(&ep_data->requeue_fifo);

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
    /* Static storage starts at 0, which is a legal index. */
    ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
    ep_data->xfer.end_idx = UDC_DWC3_XFERRSCIDX_INVALID;
    ep_data->epn = 0;

    /*
     * EP0 needs this too: the loops below start at i = 1, but
     * udc_dwc3_ep_resume() calls k_fifo_get() on this queue for every endpoint.
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
        /*
         * Isochronous is not advertised. udc_dwc3_trb_bulk() arms every
         * non-control endpoint with TRBCTL_NORMAL and a zero SOF/uframe field,
         * giving the controller no interval to start on, so an ISO endpoint
         * would be mis-armed silently. Refusing it at descriptor-build time
         * fails early instead.
         */
        ep_data->cfg.caps.iso = false;
        ep_data->cfg.caps.mps = mps;
        ep_data->trb_buf = cfg->trb_buf_in[i];
        /* Static storage starts at 0, which is a legal index. */
        ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
        ep_data->xfer.end_idx = UDC_DWC3_XFERRSCIDX_INVALID;
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
        /* udc_dwc3_ep_resume() reads requeue_fifo on every resume. */
        k_fifo_init(&ep_data->requeue_fifo);

        ep_data->dev = dev;
        ep_data->cfg.addr = USB_EP_DIR_OUT | i;
        ep_data->cfg.caps.out = true;
        ep_data->cfg.caps.bulk = true;
        ep_data->cfg.caps.interrupt = true;
        /*
         * Isochronous is not advertised. udc_dwc3_trb_bulk() arms every
         * non-control endpoint with TRBCTL_NORMAL and a zero SOF/uframe field,
         * giving the controller no interval to start on, so an ISO endpoint
         * would be mis-armed silently. Refusing it at descriptor-build time
         * fails early instead.
         */
        ep_data->cfg.caps.iso = false;
        ep_data->cfg.caps.mps = mps;
        ep_data->trb_buf = cfg->trb_buf_out[i];
        /* Static storage starts at 0, which is a legal index. */
        ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
        ep_data->xfer.end_idx = UDC_DWC3_XFERRSCIDX_INVALID;
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
 * The event buffer must be aligned to its own size, not a fixed 16 bytes
 * (GEVNTADR: "the lower n bits of the address must be GEVNTSIZn.EVNTSiz-aligned"),
 * which takes a power-of-two size.
 */
BUILD_ASSERT(IS_POWER_OF_TWO(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t)),
         "the event buffer is aligned to its own size, which must be a power of two");

#define UDC_DWC3_DEVICE_DEFINE(n)                       \
    UDC_DWC3_QUIRK_DEFINE(n);                       \
                                        \
    /*                              \
     * IRQ_CONNECT once, from preinit, not on every enable;     \
     * enable/disable are then only irq_enable()/irq_disable(). \
     */                             \
    static void udc_dwc3_irq_connect_func_##n(void)             \
    {                                   \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),      \
                udc_dwc3_irq_handler, DEVICE_DT_INST_GET(n), 0);    \
    }                                   \
                                        \
    static void udc_dwc3_irq_enable_func_##n(void)              \
    {                                   \
        irq_enable(DT_INST_IRQN(n));                    \
    }                                   \
                                        \
    static void udc_dwc3_irq_disable_func_##n(void)             \
    {                                   \
        irq_disable(DT_INST_IRQN(n));                   \
    }                                   \
                                        \
    static __nocache uint32_t udc_dwc3_dma_evt_buf_##n          \
        [CONFIG_UDC_DWC3_EVENTS_NUM]                    \
        __aligned(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t));  \
                                        \
    static __nocache uint8_t udc_dwc3_dma_setup_##n[8] __aligned(8);   \
                                        \
    static __nocache struct udc_dwc3_trb udc_dwc3_dma_trb_i##n      \
        [DT_INST_PROP(n, num_in_endpoints)][CONFIG_UDC_DWC3_TRB_NUM]    \
        __aligned(16);                          \
                                        \
    static __nocache struct udc_dwc3_trb udc_dwc3_dma_trb_o##n      \
        [DT_INST_PROP(n, num_out_endpoints)][CONFIG_UDC_DWC3_TRB_NUM]   \
        __aligned(16);                          \
                                        \
    static struct udc_dwc3_ep_data udc_dwc3_ep_data_i##n            \
        [DT_INST_PROP(n, num_in_endpoints)];                \
                                        \
    static struct udc_dwc3_ep_data udc_dwc3_ep_data_o##n            \
        [DT_INST_PROP(n, num_out_endpoints)];               \
                                        \
    static const struct udc_dwc3_config udc_dwc3_config_##n = {     \
        DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(base, DT_DRV_INST(n)),   \
        .quirk_data = &udc_dwc3_quirk_data_##n,             \
        .quirk_config = &udc_dwc3_quirk_config_##n,         \
        .num_in_eps = DT_INST_PROP(n, num_in_endpoints),        \
        .num_out_eps = DT_INST_PROP(n, num_out_endpoints),      \
        .ep_data_in  = udc_dwc3_ep_data_i##n,               \
        .ep_data_out = udc_dwc3_ep_data_o##n,               \
        .trb_buf_in = udc_dwc3_dma_trb_i##n,                \
        .trb_buf_out = udc_dwc3_dma_trb_o##n,               \
        .evt_buf = udc_dwc3_dma_evt_buf_##n,                \
        .setup_buf = udc_dwc3_dma_setup_##n,                \
        .maximum_speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),\
        .irq_connect_func = udc_dwc3_irq_connect_func_##n,      \
        .irq_enable_func = udc_dwc3_irq_enable_func_##n,        \
        .irq_disable_func = udc_dwc3_irq_disable_func_##n,      \
    };                                  \
                                        \
    K_THREAD_STACK_DEFINE(udc_dwc3_evt_stack_##n,               \
                  UDC_DWC3_EVT_STACK_SIZE);             \
                                        \
    static struct k_thread udc_dwc3_evt_thread_##n;             \
                                        \
    static struct udc_dwc3_data udc_dwc3_priv_##n = {           \
        .dev = DEVICE_DT_INST_GET(n),                   \
        .evt_stack = udc_dwc3_evt_stack_##n,                \
        .evt_thread = &udc_dwc3_evt_thread_##n,             \
    };                                  \
                                        \
    static struct udc_data udc_data_##n = {                 \
        .mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),       \
        .priv = &udc_dwc3_priv_##n,                 \
    };                                  \
                                        \
    DEVICE_DT_INST_DEFINE(n, udc_dwc3_driver_preinit, NULL, &udc_data_##n,  \
                  &udc_dwc3_config_##n, POST_KERNEL,        \
                  CONFIG_KERNEL_INIT_PRIORITY_DEVICE,       \
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
         * WriteBack/EventQ and DescFetchQ always read 0/0, though DescFetchQ
         * certainly exists: the debug register likely does not report these
         * queue types in this build, rather than the queues being absent.
         */
        .name = "AuxEventQ",
        .type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENTQ
    },
};

/*
 * Read one GDBGFIFOSPACE queue.
 */
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

/*
 * Cache the FIFO space baseline at init.
 */
static void udc_dwc3_init_fifo_space(const struct device *dev)
{
    struct udc_dwc3_data *priv = udc_get_private(dev);

    for (int n = 0; n < _NUM_FIFO_SPACE; n++) {
        for (int i = 0; i < _NUM_FIFO_REGS; i++) {
            priv->diag.max_bytes_avail[n][i] = udc_dwc3_read_fifo_space(
                dev, udc_dwc3_fifo_regs[i].type, n);
        }
    }
}

/*
 * Shell: dump the global and device registers.
 */
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

/*
 * Shell: dump GSTS and GBUSERRADDR.
 */
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

/*
 * Shell: dump the link state.
 */
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

/*
 * Shell: dump the event ring.
 */
static void udc_dwc3_dump_events(const struct device *dev, const struct shell *sh)
{
    const struct udc_dwc3_config *cfg = dev->config;
    struct udc_dwc3_data *priv = udc_get_private(dev);

    for (uint32_t i = 0; i < CONFIG_UDC_DWC3_EVENTS_NUM; i++) {
        uint32_t evt = cfg->evt_buf[i];
        char *s = (i == priv->evt.next) ? "<-" : "  ";

        shell_print(sh, "evt 0x%02x: 0x%08x %s %s",
            i, evt, s, udc_dwc3_get_event_name(evt, 0));
    }

    /*
     * How often the controller's posted write had not landed when the event
     * was read.
     */
    shell_print(sh, "events %u, posted-write waits: late %u, gave up %u",
            priv->evt.handled, priv->diag.evt_late, priv->diag.evt_gaveup);
    shell_print(sh, "worst late wait: %u polls (lower bound), %u us (upper bound)",
            priv->diag.evt_late_polls_max, priv->diag.evt_late_us_max);
    shell_print(sh, "link state changes %u, last state 0x%x, repeated x%u",
            priv->diag.evt_link_total, priv->diag.evt_link_last, priv->diag.evt_link_run);
    shell_print(sh, "GEVNTCOUNT high-water %u bytes of %u, drain %s, "
            "give-up run %u on slot %u",
            priv->diag.evt_gevntcount_hwm,
            (unsigned int)(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t)),
            udc_dwc3_drain_state_name(priv->evt.drain.state),
            priv->evt.drain.attempts, priv->evt.drain.slot);
    shell_print(sh, "control aborts (setup-pending): %u", priv->diag.ctrl_setup_pending);
}

/*
 * Shell: print every TRB in the endpoint's ring.
 */
static void udc_dwc3_dump_trb(const struct device *dev, struct udc_dwc3_ep_data *ep_data,
                  const struct shell *sh)
{
    for (uint32_t i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
        const volatile struct udc_dwc3_trb *const t = &ep_data->trb_buf[i];
        struct udc_dwc3_trb trb;

        /* ctrl first, then the words its write-back covers (see trb_snapshot). */
        trb.ctrl = t->ctrl;
        trb.status = t->status;
        trb.addr_lo = t->addr_lo;
        trb.addr_hi = t->addr_hi;

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
        char *head = (i == ep_data->ring.head) ? " <HEAD" : "";
        char *tail = (i == ep_data->ring.tail) ? " <TAIL" : "";
        char *full = (i == ep_data->ring.head &&
                  ep_data->ring.net_buf[ep_data->ring.head] != NULL) ? " <FULL" : "";

        shell_print(sh, "%p EP%02x addr=0x%08x%08x ctl=%u sts=%u hwo=%u lst=%u chn=%u"
                " csp=%u isp=%u ioc=%u spr=%u pcm1=%u sof=%u bufsiz=%u%s%s%s",
                &ep_data->trb_buf[i], ep_data->cfg.addr, trb.addr_hi, trb.addr_lo,
                trbctl, trbsts, hwo, lst, chn, csp, isp, ioc, spr, pcm1, sidsofn,
                bufsiz, head, tail, full);
    }
}

/*
 * Shell: dump every endpoint.
 */
static void udc_dwc3_dump_each(const struct device *dev,
                 void (*fn)(const struct device *, struct udc_dwc3_ep_data *,
                    const struct shell *),
                 char *label, const struct shell *sh)
{
    const struct udc_dwc3_config *cfg = dev->config;

    for (int i = 0; i < cfg->num_in_eps; i++) {
        struct udc_dwc3_ep_data *ep_data = &cfg->ep_data_in[i];
        uint8_t addr = ep_data->cfg.addr;

        /*
         * xferrscidx: the transfer resource index Update and End Transfer use,
         * UDC_DWC3_XFERRSCIDX_INVALID while no transfer holds one.
         */
        shell_print(sh, "%s for IN endpoint 0x%02x (%u %s) xferrscidx=0x%x",
              label, addr, addr & 0x7f, (addr & 0x80) ? "IN" : "OUT",
              ep_data->xferrscidx);
        (*fn)(dev, ep_data, sh);
    }

    for (int i = 0; i < cfg->num_out_eps; i++) {
        struct udc_dwc3_ep_data *ep_data = &cfg->ep_data_out[i];
        uint8_t addr = ep_data->cfg.addr;

        shell_print(sh, "%s for OUT endpoint 0x%02x (%u %s) xferrscidx=0x%x",
              label, addr, addr & 0x7f, (addr & 0x80) ? "IN" : "OUT",
              ep_data->xferrscidx);
        (*fn)(dev, ep_data, sh);
    }
}

/*
 * Shell: dump every endpoint's TRBs.
 */
static void udc_dwc3_dump_each_trb(const struct device *dev, const struct shell *sh)
{
    udc_dwc3_dump_each(dev, udc_dwc3_dump_trb, "trb", sh);
}

/*
 * Shell: dump the FIFO space counters.
 */
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
                (uint32_t)priv->diag.max_bytes_avail[n][i]);
        }
    }

    shell_print(sh, "");
    shell_print(sh, "Common");

    avail = udc_dwc3_read_fifo_space(
        dev, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOLSTATUSQ, 0);
    shell_print(sh, "- %-15s = %u bytes available", "PROTOCOLSTATUS", avail);
}

/*
 * Shell: dump everything.
 */
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

/*
 * Run a dwc3 shell command under the UDC mutex. Every command touches state the
 * work queue and the drain thread also use (endpoint commands, TRB rings, event
 * buffer), and the shell is used most while traffic is running and something is
 * wrong. The mutex is recursive, so a command that takes it itself is fine.
 */
static int dump_cmd2_handler(const struct shell *sh, size_t argc, char **argv,
                 void (*fn)(const struct device *, const struct shell *sh))
{
    const struct device *dev;

    __ASSERT_NO_MSG(argc == 2);

    dev = device_get_binding(argv[1]);
    if (!dev) {
        shell_error(sh, "Device %s not found", argv[1]);
        return -ENODEV;
    }

    /* Every command casts dev->config and the private data to this driver's. */
    if (dev->api != &udc_dwc3_api) {
        shell_error(sh, "Device %s is not a DWC3 controller", argv[1]);
        return -EINVAL;
    }

    udc_lock_internal(dev, K_FOREVER);
    (*fn)(dev, sh);
    udc_unlock_internal(dev);

    return 0;
}

/*
 * Shell: inject a synthetic XferComplete.
 */
static void udc_dwc3_cmd_fake_xfercomplete(const struct device *const dev, const struct shell *sh)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    if (priv->diag.last_xfer_dir == USB_EP_DIR_IN) {
        udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(1));
    } else {
        udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(0));
    }
}
static int cmd_fake_xfercomplete(const struct shell *sh, size_t argc, char **argv)
{
    return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_fake_xfercomplete);
}

/*
 * Shell: inject a synthetic XferComplete on EP0-OUT.
 */
static void udc_dwc3_cmd_fake_xfercomplete0(const struct device *const dev, const struct shell *sh)
{
    udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(0));
}
static int cmd_fake_xfercomplete0(const struct shell *sh, size_t argc, char **argv)
{
    return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_fake_xfercomplete0);
}

/*
 * Shell: inject a synthetic XferComplete on EP0-IN.
 */
static void udc_dwc3_cmd_fake_xfercomplete1(const struct device *const dev, const struct shell *sh)
{
    udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(1));
}
static int cmd_fake_xfercomplete1(const struct shell *sh, size_t argc, char **argv)
{
    return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_fake_xfercomplete1);
}

/*
 * Shell: run udc_dwc3_recover().
 */
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
