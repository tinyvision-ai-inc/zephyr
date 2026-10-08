/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Synopsys DWC3 USB device controller driver.
 *
 * Sections, in file order:
 * - Configuration: Kconfig defaults and driver limits.
 * - Hardware definitions: TRB, event and register layouts.
 * - Driver data: per-instance configuration and state.
 * - Helpers: run state, DCTL, buffer return, event interrupt and lock.
 * - Commands: endpoint commands and the endpoint state they drive.
 * - Transfer Requests (TRB): the TRB rings.
 * - Control buffers: arming the control transfer stages.
 * - Endpoint and control recovery.
 * - Core setup: soft reset and register set-up.
 * - Events: the event handlers and dispatch.
 * - Heartbeat and controller recovery.
 * - Event drain: the thread that reads the event ring.
 * - UDC API: the functions the Zephyr USB stack calls.
 * - Driver instance: pre-init and the device definition.
 * - Shell: debug commands.
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
 * Empty trace_tag() stubs. usbd_core.c, usbd_cdc_acm.c and usbd_ch9.c call
 * them, so these let this file replace the stock udc_dwc3.c on its own. There
 * is no trace buffer, which saves 2 KB of RAM. They are weak, so a tree can
 * supply the real ones.
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
 * Configuration
 *
 * Kconfig defaults and the driver's time and size limits.
 */

/* Kconfig defaults, so this file builds on its own. */
#ifndef CONFIG_UDC_DWC3_EVENTS_NUM
/* 16 events x 4 bytes = 64 bytes, this part's limit (see BUILD_ASSERT). */
#define CONFIG_UDC_DWC3_EVENTS_NUM  16
#endif

#ifndef CONFIG_UDC_DWC3_TRB_NUM
/* TRBs per endpoint, including the LINK TRB. EP0 uses trb_buf[1], so >= 2. */
#define CONFIG_UDC_DWC3_TRB_NUM 4

#endif

/*
 * Size of the event FIFO between the ring and dispatch. It is a power of two
 * because the indices run free and only the array index wraps.
 */
#define UDC_DWC3_EVQ_NUM                                    64u

/*
 * Longest time (ms) one drain pass busy-waits for a late event write, see
 * udc_dwc3_evt_wait_first(). The SETUP report time below is derived from it.
 */
#define UDC_DWC3_EVT_ARRIVE_MAX_MS                          100u

/*
 * Section for init, recovery, fault and log-only code. When the board copies
 * this driver to RAM, its filter keeps this section in flash. So only the
 * per-event path takes RAM. Without relocation it has no effect.
 */
#define UDC_DWC3_COLD      __noinline __attribute__((cold, section(".text.udc_dwc3_recovery_cold")))

/*
 * How long an armed SETUP may stay undelivered before
 * udc_dwc3_ctrl_setup_wd_check() reports it. It must be longer than one full
 * event-arrival wait, or a SETUP that is about to arrive gets reported. So it
 * is derived from that wait.
 */
#define UDC_DWC3_SETUP_WD_REPORT_MS                         (2u * UDC_DWC3_EVT_ARRIVE_MAX_MS)


/*
 * Event-drain thread stack. Dispatch can reach LOG_INF, and a synchronous log
 * backend formats the message on this stack, so 512 B is too small. The
 * heartbeat reports the free space in its "evtstack" line.
 */
#define UDC_DWC3_EVT_STACK_SIZE                             1280
/*
 * Event-drain thread priority. It is cooperative, so preemptible work cannot
 * split a pass. A higher priority makes the drain run the moment it is
 * signalled. It then reads slots the controller is still writing, and counts
 * them as late.
 */
#define UDC_DWC3_EVT_THREAD_PRIO                            K_PRIO_COOP(7)


/*
 * Hardware definitions
 *
 * TRB, event and register layouts from the databook, with the driver
 * limits that go with them.
 */

/* TRB memory buffer fields */
#define UDC_DWC3_TRB_STATUS_BUFSIZ_MASK                     GENMASK(23, 0)
#define UDC_DWC3_TRB_STATUS_PCM1_MASK                       GENMASK(25, 24)
/*
 * Short Packet Received, bit 26 of the status dword. On OUT write-back the
 * controller sets it on the last TRB of the transfer.
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

/* watchdog_type when no control stage is watched. No TRBCTL encoding is 0. */
#define UDC_DWC3_WATCHDOG_TYPE_NONE                         0U

#define UDC_DWC3_TRB_CTRL_ISP_IMI                           BIT(10)
#define UDC_DWC3_TRB_CTRL_IOC                               BIT(11)
/*
 * Stream ID / SOF Number. Bits 13:12 and 31:30 of the control word are
 * reserved. PCM1 and SPR are in the status dword (Figure 3-1).
 */
#define UDC_DWC3_TRB_CTRL_SIDSOFN_MASK                      GENMASK(29, 14)

/* Not every field is covered, only what this driver uses */
#define UDC_DWC3_EVT_MASK                                   GENMASK(11, 0)
#define UDC_DWC3_DEPEVT_EPN_MASK                            GENMASK(5, 1)
#define UDC_DWC3_DEPEVT_KIND_MASK                           GENMASK(9, 6)
#define UDC_DWC3_DEPEVT_RSVD_MASK                           GENMASK(11, 10)
/* Kind field of a DEPEVT type, e.g. of UDC_DWC3_DEPEVT_XFERCOMPLETE(0). */
#define UDC_DWC3_DEPEVT_KIND(depevt)                        (((depevt) & UDC_DWC3_DEPEVT_KIND_MASK) >> 6)
/*
 * Fields of an Endpoint Command Complete event.
 * Programming Guide 3.30b, Table 3-7 "Device Endpoint-n Events: DEPEVT".
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
/* XferNotReady "Event Status", DEPEVT 15:12 (SPEC 3.30b p.326). */
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK                 GENMASK(13, 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP                (0x0 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA                 (0x1 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS               (0x2 << 12)
/*
 * Event Status (bits 15:12) means different things per event type. The
 * control-stage values above and the bits below are all in that field.
 * SHORT, on XferComplete or XferInProgress: short packet received, or the last
 * packet of an isochronous interval.
 */
#define UDC_DWC3_DEPEVT_STATUS_SHORT                        BIT(13)
/* IOC bit of the TRB that completed */
#define UDC_DWC3_DEPEVT_STATUS_IOC                          BIT(14)
/* For XferComplete: LST bit of the completed TRB */
#define UDC_DWC3_DEPEVT_STATUS_LST                          BIT(15)
/*
 * XferInProgress: the isochronous interval did not complete. This is the same
 * bit as LST above. The event type tells them apart.
 */
#define UDC_DWC3_DEPEVT_STATUS_MISSED_ISOC                  BIT(15)
/* StreamEvt: 4'h1 StreamFound, 4'h2 StreamNotFound, also in bits 15:12 */
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
 * Device-event EvtInfo, bits 24:16 (databook Table 3-8). In a USB/Link State
 * Change event, EvtInfo[4] is set for SuperSpeed. EvtInfo[3:0] is the link
 * state, encoded as in DSTS.
 */
#define UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK                GENMASK(19, 16)
#define UDC_DWC3_DEVT_EVTINFO_SS                            BIT(20)
/*
 * Log one line per this many repeats of the same link state, see
 * udc_dwc3_log_link_event(). It is prime, so it shares no factor with the
 * 16-slot event ring. A multiple of 16 could sample the same slot every time.
 */
#define UDC_DWC3_EVT_LINK_LOG_EVERY                         257u
/* Heartbeat period. It bounds how long the event drain can go unscheduled. */
#define UDC_DWC3_HEARTBEAT_MS                               200u
/*
 * Heartbeats between event-ring statistics lines. The period is wall-clock,
 * so the reports continue while the event ring is not moving.
 */
#define UDC_DWC3_EVT_STATS_BEATS                            25u
/*
 * Heartbeats between statistics lines when nothing has changed (60 s). They
 * show the driver is alive on a quiet run. A line costs ~35 ms on the
 * synchronous console, so it is not printed more often. The core debug report
 * uses the same period.
 */
#define UDC_DWC3_EVT_STATS_FORCE_BEATS                      300u

/* Heartbeats between core debug-register samples (25 x 200 ms = 5 s). */
#define UDC_DWC3_CORE_DBG_BEATS                             25u


/*
 * Kick the drain if it has not finished a pass in this long while the
 * controller still reports events. This is a backstop for a drain that is not
 * running. It must be longer than one statistics line (up to ~35 ms on the
 * synchronous console). 100 ms covers any real pass.
 */
#define UDC_DWC3_EVT_IDLE_KICK_MS                           100u

/*
 * How long the heartbeat must see GEVNTCOUNT > 0 with no event taken before
 * the drain counts as dead, not slow. This is well past the drain's own
 * dead-slot skip (UDC_DWC3_EVT_DEAD_SLOT_MS), so one lost event never gets
 * this far.
 */
#define UDC_DWC3_HB_DRAIN_DEAD_MS                           5000u

/*
 * Give-ups on one slot (looks that found it empty) that also prove it dead.
 * This is a second route besides UDC_DWC3_EVT_DEAD_SLOT_MS.
 */
#define UDC_DWC3_EVT_DEAD_SLOT_GIVEUPS                      64u

/*
 * Minimum watch time before either timeout route may discard a slot. Watch
 * time is the time actually spent looking at the empty slot (drain.watched_us),
 * not wall-clock time.
 */
#define UDC_DWC3_EVT_DEAD_SLOT_MIN_MS                       200u

/* Watch time after which an empty slot is taken as dead. */
#define UDC_DWC3_EVT_DEAD_SLOT_MS                           1000u

/*
 * Minimum watch time before the drain may discard a slot on the look-ahead
 * proof (a written slot ahead of it).
 */
#define UDC_DWC3_EVT_LOOKAHEAD_MIN_MS                       50u

/* Build in the controller recovery, udc_dwc3_controller_recover(). */
#define UDC_DWC3_CONTROLLER_RECOVER

/*
 * 1: udc_dwc3_depcmd() polls every endpoint command until CmdAct clears and
 * returns its outcome.
 * 0: Start Transfer and Update Transfer are posted without that poll. Their
 * outcomes are handled in udc_dwc3_depcmd_start_xfer() and
 * udc_dwc3_depcmd_update_xfer().
 * The configuration, stall and End Transfer commands are always polled. They
 * are rare and their callers act on the outcome. DEPSTARTCFG at power-on "must
 * poll the CmdAct bit" (3.2.2.8).
 */
#ifndef UDC_DWC3_DEPCMD_POST_POLL
#define UDC_DWC3_DEPCMD_POST_POLL                           0
#endif

/*
 * How long (wall clock) a slot may stay empty before the episode is counted
 * as a lost event write rather than a late one.
 */
#define UDC_DWC3_EVT_MISSED_MS                              1000u


/*
 * Marker for a consumed or not yet written slot. 0xFFFFFFFF is never a real
 * event. Bit 0 set means a device event, which needs bits 7:1 clear. Zero is
 * not usable because it decodes as a valid endpoint event.
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
 * Command Interrupt On Completion: the controller raises Endpoint Command
 * Complete when the command finishes.
 */
#define UDC_DWC3_DEPCMD_CMDIOC                              BIT(8)
#define UDC_DWC3_DEPCMD_STATUS_MASK                         GENMASK(15, 12)
#define UDC_DWC3_DEPCMD_STATUS_OK                           (0 << 12)
#define UDC_DWC3_DEPCMD_STATUS_CMDERR                       (1 << 12)
#define UDC_DWC3_DEPCMD_XFERRSCIDX_MASK                     GENMASK(22, 16)
/*
 * "No transfer resource index". udc_dwc3_depcmd() returns it when a command
 * fails, and ep_data->xferrscidx holds it while no index is assigned.
 * XferRscIdx is 7 bits wide (DEPCMD/DEPEVT [22:16]), so no real index equals it.
 */
#define UDC_DWC3_XFERRSCIDX_INVALID                         0xffffffffU
/*
 * Returned by udc_dwc3_depcmd() for a Start Transfer when the pre-poll gave
 * up and nothing was written. Other commands return UDC_DWC3_XFERRSCIDX_INVALID
 * in that case.
 */
#define UDC_DWC3_DEPCMD_NOT_POSTED                          0xfffffffeU
/*
 * Returned by udc_dwc3_depcmd() for a Start or Update Transfer posted without
 * the post-poll (UDC_DWC3_DEPCMD_POST_POLL == 0). The command is written, but
 * its outcome is not known yet.
 */
#define UDC_DWC3_DEPCMD_POSTED                              0xfffffffdU
/* Command type, bits 3:0: DEPCFG (1) to DEPSTARTCFG (9). */
#define UDC_DWC3_DEPCMD_CMDTYP_MASK                         GENMASK(3, 0)

/* DEPCFG Command and Parameters */
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
 * Value of ep_data->cmd.depcmd_last before the endpoint's first command since
 * boot or core soft reset. Until then DEPCMD reads undefined (databook 1.3.12).
 * CmdTyp 0 is reserved, so no real command equals it.
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
 * AXI pipelined transfer limit, encoded N-1 (0x0 = 1 outstanding request,
 * 0xf = 16). At the limit the AXI master issues no new address requests until
 * the earlier data phases complete.
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
 * Databook 1.2.4 erratum workaround. Clear GRXTHRCFG.UsbRxPktCntSel so a fixed
 * NUMP is sent instead of one derived from the RX threshold. The citations are
 * in udc_dwc3_on_soft_reset().
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
 * Fixed wait after the core leaves reset, before any register read. The
 * release is a posted write. So GHWPARAMS may briefly show its old value, pass
 * a poll, and then read zero. This wait therefore comes before the poll.
 */
#define UDC_DWC3_CORE_SETTLE_MS                             50u
/*
 * How long PHYSoftRst (GUSB2PHYCFG / GUSB3PIPECTL) is held. The PHY clocks then
 * get the same time to settle before GCTL.CoreSoftReset is released.
 */
#define UDC_DWC3_PHY_RESET_MS                               100u
/*
 * Polls of the core's register file after a soft reset, one per ms, after the
 * settle time above.
 */
#define UDC_DWC3_CORE_READY_POLLS                           100
/*
 * How many times the reset sequence is tried if the register file does not
 * come back.
 */
#define UDC_DWC3_CORE_RESET_ATTEMPTS                        3u

/* Poll interval and number of polls for DSTS.DevCtrlHlt after RunStop is cleared. */
#define UDC_DWC3_HALT_POLL_MS                               1u
#define UDC_DWC3_HALT_POLLS                                 500u
/*
 * Longest sleep between two DEPCMD settles in the controller recovery's
 * quiesce wait (udc_dwc3_quiesce_settle()), used when no event reports progress.
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
/* Bits 10, 11 and 13 are reserved in this controller. Do not set them. */
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
 * Programming Guide 3.30b 1.2.56, Table 1-68: bits 15:0 EVNTCOUNT, bits 30:16
 * reserved, bit 31 EVNT_HANDLER_BUSY.
 */
#define UDC_DWC3_GEVNTCOUNT_MASK                            GENMASK(15, 0)
#define UDC_DWC3_GEVNTCOUNT_EVNT_HANDLER_BUSY               BIT(31)

/*
 * DEV_IMOD[0], Programming Guide 3.30b 1.3.13, Table 1-90 (p.253): bits 15:0
 * DEVICE_IMODI (moderation interval), bits 31:16 DEVICE_IMODC (down counter).
 */
#define UDC_DWC3_DEV_IMOD(n)                                (0xca00 + 4 * (n))
#define UDC_DWC3_DEV_IMOD_DEVICE_IMODI_MASK                 GENMASK(15, 0)
#define UDC_DWC3_DEV_IMOD_DEVICE_IMODC_MASK                 GENMASK(31, 16)
/* 250 ns per unit (Table 1-90), so 1 ms = 4000. */
#define UDC_DWC3_DEV_IMOD_INTERVAL_1MS                      4000U

/*
 * Return the event count in bytes from GEVNTCOUNT(0). The fence after the read
 * keeps later accesses from moving ahead of it.
 * Each drain pass reads it exactly once, before processing (1.2.56). A re-read
 * after the acknowledgement may still count events already handed back.
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
 * Acknowledge `words` event words. The count is written with EVNT_HANDLER_BUSY
 * (bit 31) set (Table 1-68). Each drain pass calls this exactly once. It is the
 * only GEVNTCOUNT write while the ring is live. words == 0 is legal. It credits
 * nothing and clears the handler-busy bit.
 */
static inline void udc_dwc3_gevntcount_ack(const mm_reg_t base,
                       const uint32_t words)
{
    sys_write32((words * sizeof(uint32_t)) |
            UDC_DWC3_GEVNTCOUNT_EVNT_HANDLER_BUSY,
            base + UDC_DWC3_GEVNTCOUNT(0));
}

/*
 * Initialise the count when the event buffer is set up, by writing 0. This
 * credits nothing. EVNT_HANDLER_BUSY is write-1-to-clear, so it stays as is.
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

/* USB Global Status register */
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

/* The video streaming endpoint. */
#define UDC_DWC3_VIDEO_EP                                   0x85U

/* Endpoint left out of the per-arm TRB trace, because of its volume. */
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
/* SPEC 3.30b DGCMD 0Ch: Parameter[4:0] = physical endpoint. For U3 exit (4.1.10). */
#define UDC_DWC3_DGCMD_SET_EP_NRDY                          (0xc << 0)
/* DGCMDPAR for 09h: [4:0] FIFO number, [5] 1 = TX FIFO, 0 = RX FIFO. */
#define UDC_DWC3_DGCMD_FIFOFLUSH_NUM_MASK                   GENMASK(4, 0)
#define UDC_DWC3_DGCMD_FIFOFLUSH_TX                         BIT(5)
/*
 * Limit on polls of a generic command, counted in register reads rather than
 * time. See udc_dwc3_dgcmd_wait_idle().
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
 * Queue types dumped by "dwc3 fifo". It sizes udc_dwc3_fifo_regs[] and
 * max_bytes_avail[][], so adding a queue type without raising it fails to
 * compile.
 */
#define _NUM_FIFO_REGS  8

/*
 * Driver data
 *
 * The TRB layout and the per-instance configuration and state.
 */

/*
 * TRB: one DMA request from the CPU to the DWC3 core, in the databook layout.
 * No cache flush is needed because this SoC has no data cache.
 */
struct udc_dwc3_trb {
    uint32_t    addr_lo;
    uint32_t    addr_hi;
    uint32_t    status;
    uint32_t    ctrl;
} __packed __aligned(16);

/* Controller configuration that can stay in non-volatile memory */
struct udc_dwc3_config {
    DEVICE_MMIO_NAMED_ROM(base);
    /* USB endpoints data */
    struct udc_dwc3_ep_data *ep_data_in;
    struct udc_dwc3_ep_data *ep_data_out;
    /* DMA-accessible TRB buffers */
    struct udc_dwc3_trb (*trb_buf_in)[CONFIG_UDC_DWC3_TRB_NUM];
    struct udc_dwc3_trb (*trb_buf_out)[CONFIG_UDC_DWC3_TRB_NUM];
    /* USB device configuration */
    int                     maximum_speed_idx;
    /* Event buffer, written by the DWC3 with DMA */
    volatile uint32_t       *evt_buf;
    /*
     * Driver-owned SETUP buffer (SPEC 3.30b 4.4 step 1: "Setup a Control-Setup
     * TRB"). It is NOCACHE, like every buffer the controller writes.
     */
    uint8_t                 *setup_buf;
    /* Data used by vendor-specific functions ("quirks") */
    const void              *quirk_config;
    void                    *quirk_data;
    /* IRQ management functions */
    void (*irq_connect_func)(void);
    void (*irq_enable_func)(void);
    void (*irq_disable_func)(void);
    /* Number of IN and OUT hardware endpoints */
    uint8_t                 num_in_eps;
    uint8_t                 num_out_eps;
};


/*
 * Transfer state of one endpoint. Each transition has one owner. The state
 * tells "no transfer running" apart from "index not known yet", which
 * xferrscidx alone cannot. This matters because a second Start on an endpoint
 * that already holds a transfer takes a new transfer resource that is never
 * returned.
 *
 * Rules (see udc_dwc3_ep_state_set() and its callers):
 *   - DEPSTRTXFER only from IDLE.
 *   - DEPUPDXFER and DEPENDXFER only from RUNNING, where xferrscidx is valid.
 *   - DEPSTARTCFG, DEPCFG and DEPXFERCFG belong to endpoint enable, from IDLE.
 *   - No DEPXFERCFG on a recovery path. Each one allocates another resource.
 * Each transition is written next to the command that causes it.
 *
 * udc_common.c owns cfg.stat.enabled. Only the two stall commands write
 * cfg.stat.halted, so it always matches the controller.
 */
enum udc_dwc3_ep_state {
    UDC_DWC3_EP_IDLE = 0,     /* no transfer, no controller resource held   */
    UDC_DWC3_EP_STARTING,     /* DEPSTRTXFER posted, still executing        */
    UDC_DWC3_EP_START_UNKNOWN,/* DEPSTRTXFER outcome unknown past the
                   * deadline. The controller may hold a
                   * transfer resource, so no Start follows */
    UDC_DWC3_EP_RUNNING,      /* transfer live, xferrscidx valid            */
    UDC_DWC3_EP_ENDING,   /* DEPENDXFER posted, awaiting completion     */
    UDC_DWC3_EP_END_UNKNOWN,  /* DEPENDXFER outcome unknown past the
                   * deadline. The controller may still own
                   * the ring, so nothing is reclaimed          */
};

/*
 * Work owed to an endpoint, done by udc_dwc3_ep_recover() once no command is
 * open on it. It is a bitmask because several can be owed at once, e.g. a
 * deferred Clear Stall and a resume deferred for 3.2.2.7. The recovery bits
 * survive transfer-state resets. udc_dwc3_ep_disable() keeps only the cancel.
 * A core reset or controller disable (udc_dwc3_drop_xfer_state()) clears all.
 */
enum udc_dwc3_ep_pending {
    UDC_DWC3_EP_PEND_NONE       = 0,
    UDC_DWC3_EP_PEND_CLEAR_STALL    = BIT(0), /* leave halt once the End reports */
    UDC_DWC3_EP_PEND_RESUME     = BIT(1), /* re-establish the transfer       */
    UDC_DWC3_EP_PEND_DEQUEUE    = BIT(3), /* release the ring, cancel the buffers */
    /*
     * An Update Transfer refused because the Start was still open. This is not
     * recovery work. udc_dwc3_ep_update_owed() issues it once the Start has an
     * index.
     */
    UDC_DWC3_EP_PEND_UPDATE         = BIT(5),
};


/*
 * Driver data for one endpoint. The vendor quirk API (udc_dwc3_lattice_usb23.h)
 * reads epn, trb_buf and xferrscidx by name, so they stay at top level.
 */
struct udc_dwc3_ep_data {
    /* First member, so ep_data and ep_cfg pointers can be cast */
    struct udc_ep_config            cfg;
    /* Physical endpoint number. The logical address is in cfg. */
    int                             epn;
    /*
     * The TRB ring. trb_buf[CONFIG_UDC_DWC3_TRB_NUM - 1] is the LINK TRB.
     * EP0 uses trb_buf[0] and [1] only.
     */
    volatile struct udc_dwc3_trb    *trb_buf;
    /*
     * Transfer resource index for endpoint commands, assigned by the
     * controller. It is UDC_DWC3_XFERRSCIDX_INVALID while no transfer is
     * active: until a Start Transfer reports one, and again once a Start or End
     * is issued, the endpoint state is reset, or DEPSTARTCFG reassigns all
     * resources.
     */
    uint32_t                        xferrscidx;
    /* Work item that submits the queued buffers of this endpoint */
    struct k_work                   work;
    /* Cancelled buffers to re-queue after the endpoint is disabled */
    struct k_fifo                   requeue_fifo;
    /* Back-pointer to the device, for the work items */
    const struct device             *dev;
    /*
     * Software side of the TRB ring: the buffer armed in each slot of trb_buf.
     * The UDC mutex protects it, because udc_dwc3_push_trb() and
     * udc_dwc3_pop_trb() may run on different threads.
     * It has one slot less, because the LINK TRB never holds a buffer. Slot i
     * is in use when net_buf[i] != NULL. The ring is full when net_buf[head] is
     * set, and not empty when net_buf[tail] is set.
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
         * What this endpoint is doing. See enum udc_dwc3_ep_state for the
         * rules. udc_dwc3_ep_busy_sync() derives cfg.stat.busy from this and
         * the ring.
         */
        enum udc_dwc3_ep_state  state;
        /* Work owed once no command is open. See enum udc_dwc3_ep_pending. */
        uint8_t                 pending;
        /*
         * Index the outstanding End Transfer was posted against. Posting an
         * End clears xferrscidx, because a successful End frees the resource.
         * A refused End leaves the transfer holding it. The refusal may be seen
         * much later, in udc_dwc3_ep_resolve_cmd(). With the index kept here,
         * the endpoint returns to RUNNING with a valid index. Valid only while
         * an End is outstanding.
         */
        uint32_t                end_idx;
    } xfer;
    /*
     * This endpoint's DEPCMD bookkeeping. A core soft reset sets depcmd_last to
     * NONE, and udc_dwc3_ep_state_reset() clears cmd_record.
     */
    struct {
        /*
         * The last command written (without CmdAct). UDC_DWC3_DEPCMD_NONE while
         * DEPCMD reads undefined.
         */
        uint32_t    depcmd_last;
        /*
         * DEPCMD as read for this endpoint's open Start or End Transfer, saved
         * when another command is posted over it. If the Command Complete event
         * is lost, DEPCMD is the only record of how the command ended, and a
         * later command overwrites it. udc_dwc3_depcmd() saves it from its
         * pre-poll read. udc_dwc3_cmd_outcome() uses it once DEPCMD has moved
         * on. It is 0 when empty. A new Start or End clears it.
         */
        uint32_t    cmd_record;
        /*
         * Cycle stamp of the last Start or End Transfer posted on this
         * endpoint. STARTING and ENDING last at most UDC_DWC3_CMD_UNKNOWN_MS
         * from here. Then the endpoint moves to the matching UNKNOWN state.
         */
        uint32_t    cmd_t0;
    } cmd;
    /*
     * Pool generation (priv->epcfg.epoch) in which this endpoint got its
     * transfer resource with DEPXFERCFG. Only DEPSTARTCFG frees resources. So
     * equal values mean the endpoint already holds one, and DEPXFERCFG is not
     * repeated.
     */
    uint32_t                        rsc_epoch;
    /* Diagnostics only. Nothing decides on these. */
    struct {
        /*
         * Arms and retires on this endpoint. This is the only device-side sign
         * of a bulk endpoint that stops accepting host traffic.
         */
        uint32_t    n_arm;
        uint32_t    n_retire;
        /* The last command's error is already logged. */
        bool        cmd_reported;
    } diag;
};

/* What the event drain is doing right now. */
enum udc_dwc3_drain_state {
    UDC_DWC3_DRAIN_IDLE = 0,    /* controller owes nothing */
    UDC_DWC3_DRAIN_RUNNING,     /* taking events */
    UDC_DWC3_DRAIN_WAITING,     /* head slot owed but still empty (late write) */
};

/*
 * Result of one arrival wait. The caller decides the drain state from it, see
 * udc_dwc3_evt_drain().
 */
enum udc_dwc3_wait_result {
    UDC_DWC3_WAIT_ARRIVED,  /* the write landed inside the budget */
    UDC_DWC3_WAIT_EXPIRED,  /* budget spent; the slot is still empty */
};

/*
 * Drain state. The drain thread writes it during a pass, without a lock.
 * udc_dwc3_on_soft_reset() resets it together with the event ring. The
 * heartbeat reads it without a lock, so every field is at most 32 bits wide to
 * avoid torn reads.
 */
struct udc_dwc3_drain {
    uint32_t    state;      /* enum udc_dwc3_drain_state */
    uint32_t    slot;       /* slot the episode is stuck on */
    uint32_t    since;      /* cycle stamp when the episode opened */
    uint32_t    attempts;   /* consecutive give-ups on that slot */
    uint32_t    gc0;        /* GEVNTCOUNT when the episode opened */
    uint32_t    watched_us; /* time actually spent looking at the slot */
    uint32_t    quiet;      /* nothing printed in this episode yet */
    uint32_t    counted;    /* this episode was already counted as missed */
    /*
     * The slot held back by the last skip, and whether it is being watched.
     * Set only when a skip stepped over fewer events than the controller owed.
     * The held slot then lies inside the stuck group, so no new event can land
     * in it. See udc_dwc3_evt_skip_dead_slot().
     */
    uint32_t    skip_watch_slot;
    bool        skip_watch;
};

/* Reset the whole drain state at once. */
static inline void udc_dwc3_drain_reset(struct udc_dwc3_drain *const d)
{
    *d = (struct udc_dwc3_drain){ .state = UDC_DWC3_DRAIN_IDLE };
}

/*
 * One snapshot of the core debug registers (LTSSM, BMU, LNMCC, LSP, EPINFO).
 * Some of them only make sense next to a healthy baseline, so they are sampled
 * periodically and a fault dump is compared with the last sample.
 */
struct udc_dwc3_core_dbg {
    uint32_t    ltssm;
    uint32_t    bmu;
    uint32_t    lnmcc;
    uint32_t    lsp;
    uint32_t    epinfo0;
    uint32_t    epinfo1;
};

/*
 * Diagnostics: counters, maxima, timestamps and report state for the
 * heartbeat, the dumps and the shell. None of it changes what the driver does
 * to the controller.
 */
struct udc_dwc3_diag {
    uint32_t                    evt_stack_free;     /* smallest observed headroom, bytes */
    /* Times DEVCTRLHLT was not seen after RunStop was cleared */
    uint32_t                    halt_timeouts;
#if CONFIG_UDC_DWC3_SHELL
    /* FIFO space initial values */
    uint16_t                    max_bytes_avail[_NUM_FIFO_SPACE][_NUM_FIFO_REGS];
    /* Direction of the last control stage armed (shell fake XferComplete). */
    uint8_t                     last_xfer_dir;
#endif
    /*
     * Late event writes: events not yet written when first read, and give-ups
     * waiting for them.
     */
    uint32_t                    evt_late;
    uint32_t                    evt_gaveup;
    /* largest backlog the controller reported, bytes */
    uint32_t                    evt_gevntcount_hwm;
    /* completions returned by the heartbeat sweep */
    uint32_t                    evt_sweep_rescued;
    uint32_t                    evt_sweep_runs;     /* sweeps that found something to drain */
    /* slots declared dead early by the look-ahead proof */
    uint32_t                    evt_lookahead_short;
    uint32_t                    evt_gaveup_us_max;  /* worst time a slot stayed empty, us */
    uint32_t                    evt_missed;         /* give-up runs presumed a lost write */
    uint32_t                    evt_missed_frozen;  /* of those, with GEVNTCOUNT not moving */
    uint32_t                    evt_gaveup_multi;   /* runs opened owing more than one event */
    /* counter signature at the last stats line */
    uint32_t                    stats_sig_last;
    /* beats since that line, for the forced line */
    uint32_t                    stats_quiet_beats;
    /* times the heartbeat restarted a stopped drain */
    uint32_t                    evt_kick;
    /* Interrupts taken versus worker passes entered. */
    uint32_t                    evt_isr;            /* interrupt handler invocations */
    uint32_t                    evt_worker_runs;    /* event worker passes entered */
    uint32_t                    evt_skipped;        /* event slots skipped as dead */
    /*
     * Skips later proved wrong. The slot after a skip filled, and the
     * controller writes in order, so the skipped slots were late, not lost.
     * Zero means every skip discarded only events that never arrived.
     */
    uint32_t                    evt_skip_refuted;
    uint32_t                    evt_link_total;     /* USB/Link State Change events seen */
    /* consecutive events with the same state */
    uint32_t                    evt_link_run;
    uint32_t                    evt_link_last;      /* that state, EvtInfo[3:0] */
    uint32_t                    dispatch_evt;       /* event being dispatched now, 0 = none */
    uint32_t                    dispatch_t0;        /* cycle stamp when that dispatch began */
    /* Worst wait for a late event that arrived: polls (lower bound), us (upper). */
    uint32_t                    evt_late_polls_max;
    uint32_t                    evt_late_us_max;
    uint32_t                    evt_midzero;        /* passes that stopped on an empty slot */
    /* Completions posted to a full usbd queue. No buffer is lost. Printed as pf. */
    uint32_t                    post_fail_total;
    uint32_t                    drain_dead_resets;  /* reconnects issued for a dead ring */
    /* promotions to START_UNKNOWN / END_UNKNOWN */
    uint32_t                    ep_cmd_unknown;
    uint32_t                    xnrdy_acted;        /* non-control XferNotReady with TRBs armed */
    uint32_t                    core_dbg_beats;     /* beats since the last CORE debug sample */
    /* Last sample reported, so an unchanged core is not printed again. */
    struct udc_dwc3_core_dbg    core_dbg_last;
    /* beats since that report, for the forced report */
    uint32_t                    core_dbg_quiet;
    uint32_t                    ctrl_start_fail;    /* Start Transfer commands rejected */
    /* Stage buffers of abandoned transfers returned in the Setup phase. */
    uint32_t                    ctrl_stale_returned;
    /*
     * The control endpoint and stage the watchdog is guarding. watchdog_type
     * returns to UDC_DWC3_WATCHDOG_TYPE_NONE when the stage completes, its
     * endpoint is disabled, or the transfer state is dropped.
     */
    struct udc_dwc3_ep_data     *watchdog_ep;
    uint32_t                    watchdog_type;
    /*
     * SETUP watchdog state (heartbeat):
     *   wd_gen            SETUPs armed
     *   wd_seen/wd_beats  how long the current one has been waiting
     *   wd_reported       stops a second report for the same SETUP
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
    /* Reclaims completed (ring cleared, TxFIFO flushed). */
    uint32_t                    ctrl_reclaim_done;
    uint32_t                    ctrl_status_done;   /* status stages retired (IN and OUT) */
    /*
     * Device-wide command counts. The per-endpoint ones are on the endpoint.
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
     * The periodic k_timer stays on the UDC_DWC3_HEARTBEAT_MS grid under any
     * load. Only the work run can be late or merged. These count timer
     * expiries and merged work runs.
     */
    uint32_t                    hb_expiries;
    uint32_t                    hb_coalesced;
};

/*
 * Per-instance run-time state, from udc_get_private(dev). It holds what the
 * driver acts on, grouped by concern. Counters and report state are in diag.
 */
struct udc_dwc3_data {
    DEVICE_MMIO_NAMED_RAM(base);
    const struct device     *dev;       /* back-reference to the device */

    /* Kernel objects. */
    struct k_sem            evt_sem;    /* ISR -> drain thread */
    k_thread_stack_t        *evt_stack;
    struct k_thread         *evt_thread;
    /*
     * Liveness. A periodic k_timer submits heartbeat_work, so no beat depends
     * on a work item re-arming itself.
     */
    struct k_timer          heartbeat_timer;
    struct k_work           heartbeat_work;
    /* Runs udc_dwc3_evt_force(), one DGCMD that makes the controller emit an event. */
    struct k_work           nudge_work;
    /*
     * Held by udc_dwc3_dgcmd(), the only DGCMDPAR/DGCMD writer. Its callers run
     * on the drain thread and the UDC work queue.
     */
    struct k_spinlock       dgcmd_lock;

    /* Event ring and its drain. */
    struct {
        uint32_t                next;           /* ring slot the drain reads next */
        /*
         * Events copied out of the ring but not yet dispatched. It is normally
         * emptied in the pass that fills it. Events stay in it only when a reset
         * or error post did not fit in the stack's queue. They are retried on
         * the next pass. When it is full, the drain stops taking events from the
         * ring. Only the event thread uses it, so there is no lock. The indices
         * run free.
         * tail - head is the occupancy, from 0 to UDC_DWC3_EVQ_NUM. It is never
         * reduced modulo the size, or a full FIFO would read as empty. Only the
         * array index is taken modulo the size. Zeroed with the ring in
         * on_soft_reset().
         */
        uint32_t                q[UDC_DWC3_EVQ_NUM];
        uint32_t                q_head;
        uint32_t                q_tail;
        uint32_t                handled;        /* events dispatched */
        /*
         * GEVNTCOUNT from the pass's single read, shared with every other
         * reader. The controller changes the register all the time, so two
         * reads never agree. Only udc_dwc3_evt_drain() and
         * udc_dwc3_drain_helper() read it. The rest of the driver uses this
         * copy.
         */
        uint32_t                gc_last;
        struct udc_dwc3_drain   drain;          /* see struct udc_dwc3_drain */
        uint32_t                force_t0;       /* cycle stamp of the last forced command */
        uint32_t                worker_exit_t0; /* cycle stamp when the drain last exited */
        /*
         * Dead-drain detection: events taken from the ring (handled + FIFO
         * occupancy) at the last beat, and beats with none taken since.
         */
        uint32_t                hb_last_taken;
        uint32_t                hb_stuck_beats;
    } evt;

    /* Control transfer (EP0/EP1). */
    struct {
        /*
         * Control transfer progress, as one value rather than flags. A request
         * that ends without a status stage then leaves no stale "data done"
         * flag for the next request's first XferNotReady(Data).
         */
        enum udc_dwc3_ctrl_state {
            UDC_DWC3_CTRL_IDLE = 0,     /* Setup phase: the Setup TRB is armed or about to be */
            UDC_DWC3_CTRL_SETUP_DONE,   /* SETUP taken; data or status not yet armed */
            UDC_DWC3_CTRL_DATA_DONE,    /* data stage retired; waiting for XferNotReady(Status) */
            UDC_DWC3_CTRL_STATUS_READY, /* XferNotReady(Status) taken; status not yet armed */
            UDC_DWC3_CTRL_STATUS_ARMED, /* status TRB armed */
            UDC_DWC3_CTRL_DATA_ARMED,   /* data TRB armed */
            /*
             * udc_dwc3_ctrl_ep_recover() is running. _STALL means it ends with
             * Set Stall. The stage handlers defer to it, so no other state is
             * entered until it finishes.
             */
            UDC_DWC3_CTRL_RECOVERING,
            UDC_DWC3_CTRL_RECOVERING_STALL,
        } state;
        /* Copy of the current SETUP packet, taken before the stack can react. */
        struct usb_setup_packet setup;
        /*
         * The data stage completed with SetupPending, so the host abandoned
         * this transfer. Handled at XferNotReady(Status), in
         * udc_dwc3_on_ctrl_xnr().
         */
        bool                    setup_pending;
    } ctrl;

    /* Endpoint configuration (DEPSTARTCFG). */
    struct {
        uint8_t     first_ep;   /* first endpoint configured */
        /*
         * DEPSTARTCFG has been tried for this configuration. Cleared on bus
         * reset. Set even when DEPSTARTCFG was skipped or refused.
         */
        bool        pool_assigned;
        /*
         * Transfer-resource pool generation. DEPSTARTCFG, the only command that
         * frees resources, bumps it. Compared with each ep_data->rsc_epoch.
         */
        uint32_t    epoch;
        /*
         * DALEPENA as last written. udc_dwc3_dalepena_set() writes both, and
         * the core reset in udc_dwc3_init() zeroes both. The arm paths
         * (endpoint worker, udc_dwc3_trb_bulk()) read this copy, not the
         * register.
         */
        uint32_t    dalepena;
    } epcfg;

    /*
     * SPEC 4.1.10: whether the link is in U3, and which physical endpoints had
     * an active transfer when it entered U3.
     */
    struct {
        bool        in_u3;
        uint32_t    u3_active_eps;
    } link;

    /*
     * Controller run/recover state. Written by udc_dwc3_enable(),
     * udc_dwc3_disable(), the ErrticErr event and udc_dwc3_controller_recover().
     */
    struct {
        enum udc_dwc3_run_state {
            UDC_DWC3_RUN = 0,       /* normal operation */
            UDC_DWC3_RUN_RESET_OWED,    /* SPEC 3.3.2 ErrticErr: the heartbeat resets */
            UDC_DWC3_RUN_STOPPING,      /* SPEC 4.1.8: transfers end before
                             * RunStop=0. No new transfer starts */
            UDC_DWC3_RUN_RESETTING,     /* the event ring and drain state are
                             * being re-initialised
                             * (udc_dwc3_evt_block()). No drain pass,
                             * no new transfer */
        } state;
        /*
         * Given on each transfer or control state change while the controller
         * recovery waits in UDC_DWC3_RUN_STOPPING.
         */
        struct k_sem quiesce_sem;
    } run;

    /* Counters and report state only - see struct udc_dwc3_diag. */
    struct udc_dwc3_diag    diag;
};


/* Indexes matching the "device-speed" devicetree property values. */
enum {
    UDC_DWC3_SPEED_IDX_FULL_SPEED = 1,
    UDC_DWC3_SPEED_IDX_HIGH_SPEED = 2,
    UDC_DWC3_SPEED_IDX_SUPER_SPEED = 3,
};

/* Vendor quirks: vendor-specific hooks, overridable per SoC. */
struct udc_dwc3_vendor_quirks {
    int (*preinit)(const struct device *const dev);
    int (*init)(const struct device *const dev);
    int (*enable)(const struct device *const dev);
};

/* Helpers for accessing vendor quirks */
#define UDC_DWC3_QUIRK_CFG(dev)     (((const struct udc_dwc3_config *)(dev->config))->quirk_config)
#define UDC_DWC3_QUIRK_DATA(dev)    (((const struct udc_dwc3_config *)(dev->config))->quirk_data)

#if DT_HAS_COMPAT_STATUS_OKAY(snps_dwc3 /* <- replace with your more specific compatible */)
#include "udc_dwc3_lattice_usb23.h"
#endif

/* Wrappers that return 0 when no quirk is set */
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

/*
 * Helpers
 *
 * Small functions used throughout: run state, DCTL writes, buffer return,
 * event interrupt masking and the UDC lock.
 */

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
 * Enable or disable an endpoint in DALEPENA and in its copy,
 * priv->epcfg.dalepena. This is the only DALEPENA writer. A core reset clears
 * both (udc_dwc3_init()).
 */
static UDC_DWC3_COLD void udc_dwc3_dalepena_set(const struct device *const dev,
                        const int epn, const bool on)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t              base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t              bit = UDC_DWC3_DALEPENA_USBACTEP(epn);

    if (on) {
        priv->epcfg.dalepena |= bit;
        sys_set_bits(base + UDC_DWC3_DALEPENA, bit);
    } else {
        priv->epcfg.dalepena &= ~bit;
        sys_clear_bits(base + UDC_DWC3_DALEPENA, bit);
    }
}

/*
 * udc_is_enabled() as a plain load. atomic_test_bit() is an out-of-line call
 * with CONFIG_ATOMIC_OPERATIONS_C. The result is exact when the caller holds the
 * UDC mutex, because udc_enable() and udc_disable() change the bit only under it.
 * An aligned word load cannot tear.
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
 * Called when a transfer or control state changes. Wakes a controller recovery
 * that is waiting for the device to go quiet.
 */
static inline void udc_dwc3_quiesce_progress(struct udc_dwc3_data *const priv)
{
    if (priv->run.state == UDC_DWC3_RUN_STOPPING) {
        k_sem_give(&priv->run.quiesce_sem);
    }
}

/*
 * True while the transfers are being ended (4.1.8) or the core is being
 * re-initialised. No new transfer may start then.
 */
static inline bool udc_dwc3_run_halting(const struct udc_dwc3_data *const priv)
{
    return priv->run.state == UDC_DWC3_RUN_STOPPING ||
           priv->run.state == UDC_DWC3_RUN_RESETTING;
}

/*
 * The only writer of ctrl.state. It also wakes a controller recovery that waits
 * for the control endpoint to return to Setup.
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
 * Shut down the controller. A control endpoint that is already disabled counts
 * as done. A failed controller recovery can leave them disabled, and every later
 * shutdown must still succeed.
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
 * Write DCTL with ULSTCHNGREQ = 0. Any value in that write-only field is a link
 * state request, and the databook requires 0 when no link change is wanted (DCTL,
 * 3.30b). Every DCTL write goes through here, except the link request in
 * udc_dwc3_dctl_link_request().
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
 * Issue a link state request. It writes 0 first, because the databook requires a
 * 0 between two identical requests.
 */
static UDC_DWC3_COLD void udc_dwc3_dctl_link_request(const mm_reg_t base, const uint32_t req)
{
    const uint32_t v = sys_read32(base + UDC_DWC3_DCTL) & ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;

    sys_write32(v, base + UDC_DWC3_DCTL);
    sys_write32(v | (req & UDC_DWC3_DCTL_ULSTCHNGREQ_MASK), base + UDC_DWC3_DCTL);
}


/*
 * Why a buffer goes back to the stack, and the status it carries. The values
 * follow Zephyr's udc_common.c:
 *   DONE       0             the transfer completed
 *   ABANDONED  -ECONNRESET   a control data/status stage dropped because the host
 *                            sent a new SETUP (as udc_setup_received())
 *   CANCELLED  -ECONNABORTED a queued request taken back by the driver on dequeue,
 *                            disable, reset or recovery (as udc_ep_cancel_queued())
 *   REFUSED    -EINVAL       a buffer this driver cannot use
 * Classes drop a cancelled request quietly. They report any other error as a
 * failed transfer.
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
 * Negotiated speed. This is the only place DSTS.ConnectSpd is decoded. It is
 * valid from Connect Done on (4.1.3).
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
 * Turn the event interrupt on or off, both GEVNTSIZ.EvntIntMask and the CPU line.
 * While masked, the controller still writes events but does not interrupt. The
 * ISR turns it off until the drain has emptied the ring. The drain and enable
 * turn it on, and disable turns it off. Interrupts are locked because the ISR
 * writes the same register.
 */
static inline void udc_dwc3_evt_irq(const struct device *const dev, const bool on)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    const mm_reg_t                      base = DEVICE_MMIO_NAMED_GET(dev, base);
    const unsigned int                  key = irq_lock();

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
 * Stop the event drain before the event ring and drain state are re-initialised
 * in udc_dwc3_on_soft_reset(). It sets RESETTING first. Every drain pass checks
 * it on entry, which also stops a drain thread that is woken but has not run yet.
 * Then it removes every source of a new pass: the event interrupt, the heartbeat
 * timer, a queued nudge and a pending wake. A pass never sleeps before its
 * write-back, so none can be caught half-way through the ring. Returns the
 * previous run state. udc_dwc3_enable() turns the event interrupt and the
 * heartbeat back on.
 */
static UDC_DWC3_COLD enum udc_dwc3_run_state udc_dwc3_evt_block(const struct device *const dev)
{
    struct udc_dwc3_data *const   priv = udc_get_private(dev);
    const enum udc_dwc3_run_state prev = priv->run.state;

    priv->run.state = UDC_DWC3_RUN_RESETTING;
    udc_dwc3_evt_irq(dev, false);
    k_timer_stop(&priv->heartbeat_timer);
    (void)k_work_cancel(&priv->nudge_work);
    k_sem_reset(&priv->evt_sem);

    return prev;
}

/* UDC API lock: a thin wrapper over the framework mutex. */
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
 * Endpoint commands (DEPCMD) and the endpoint transfer state they drive.
 * DEPCMD takes a command number and parameters. The controller clears CmdAct
 * when the command completes.
 */

/*
 * CSftRst completion wait: a few fast register reads, then sleeping ticks.
 * udc_dwc3_on_soft_reset() runs on the work queue under the mutex, so it must not
 * busy-wait for the whole time.
 */
#define UDC_DWC3_CSFTRST_FAST_READS 256u
#define UDC_DWC3_CSFTRST_SLOW_TICKS 10u
/* Endpoint-command wait, in register reads, for CmdAct to clear. */
#define UDC_DWC3_CMD_FAST_POLLS     32u

/*
 * How long a posted Start or End may keep CmdAct set before its outcome counts as
 * unknown (STARTING -> START_UNKNOWN, ENDING -> END_UNKNOWN). A slow command
 * finishes well inside this. The heartbeat checks it against cmd_t0.
 */
#define UDC_DWC3_CMD_UNKNOWN_MS     100u

/* Defined with the other event-name decoders. Used here in timeout logs. */
static const char *udc_dwc3_get_devt_ulstchng_name(const uint32_t dsts);

/*
 * Physical endpoint number for a DEPCMD register address (the inverse of
 * UDC_DWC3_DEPCMD(n)). Returns UDC_DWC3_MAX_EPN if addr is not a DEPCMD register.
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
 * GHWPARAMS0.MDWIDTH and GHWPARAMS7.RAM1_DEPTH are fixed at synthesis. Zero in
 * either means the reset has not finished, and the core must not be configured
 * yet.
 *
 * Two non-zero reads in a row are needed, since one read can catch a value in
 * transition. Returns false if either still reads zero when the time runs out.
 */
static bool udc_dwc3_wait_regfile_ready(const struct device *const dev)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

    /*
     * Always settle first. A reset that has not started yet still reads the old,
     * non-zero values, which would pass every test below.
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
 * Poll DEPCMD.CmdAct until it clears. Returns false if it is still set when the
 * poll count runs out. That means the command is still executing, not that it
 * failed. *reg_out (required) gets the last value read.
 */
static bool udc_dwc3_wait_cmdact_zero(const struct device *const dev,
                      const uint32_t addr, uint32_t *const reg_out)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    uint32_t       reg = 0;

    /*
     * A short read loop with no yield, because a yield does not help. On the drain
     * thread (cooperative priority) k_yield() returns at once. On the work queue it
     * hands the CPU to the drain thread, which needs the mutex held here. Every
     * caller handles false, so giving up after a few dozen reads is cheap.
     */
    for (uint32_t i = 0; i < UDC_DWC3_CMD_FAST_POLLS; i++) {
        reg = sys_read32(base + addr);
        if ((reg & UDC_DWC3_DEPCMD_CMDACT) == 0) {
            *reg_out = reg;
            return true;
        }
    }

    /*
     * No sleeping fallback either. Every DEPCMD is issued with the UDC mutex held,
     * and udc_dwc3_handle_event() takes that mutex for every event. Sleeping here
     * would stall event handling and recovery when the core stops answering.
     */
    *reg_out = reg;

    return false;
}



/*
 * Make a descriptor visible to the controller before the command that fetches it.
 */
static inline void udc_dwc3_trb_sync(volatile uint32_t *const last_word)
{
    /* The fence alone orders the writes. No read-back is needed. */
    ARG_UNUSED(last_word);

#if defined(CONFIG_RISCV)
    __asm__ volatile ("fence iorw,iorw" ::: "memory");
#else
    barrier_dsync_fence_full();
#endif
}

/* Log only the first few stomps; this can fire in a tight loop. */
#define UDC_DWC3_TRB_STOMP_LOG_FIRST    8u

/* One controller instance only, because udc_dwc3_stomp_priv is file-scope. */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
         "udc_dwc3 is single-instance: udc_dwc3_stomp_priv is file-scope and "
         "the last initialised controller would own every stomp report");

/* The controller instance that udc_dwc3_trb_stomp_report() reports into. */
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
 * The only writer of TRB words. The words go out in a fixed order, with ctrl
 * (which holds HWO) last, then the fence. This CPU cannot store a 16-byte TRB in
 * one access, and a struct assignment would leave the order to the compiler.
 * Writing ctrl last keeps the controller from seeing HWO before the rest of the
 * TRB.
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
 * Arm a TRB with udc_dwc3_trb_write(). It reports, but does not prevent, an
 * overwrite of a TRB the controller still owns. To clear a TRB the controller has
 * already released, which may still read HWO=1, call udc_dwc3_trb_write()
 * directly.
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
 * Snapshot a TRB the controller may have written back. It reads ctrl, then
 * status, once each. The controller writes status and clears HWO in the same
 * write-back. So status is the written-back value only if ctrl, read first, has
 * HWO clear. Returns true in that case. False means the controller still owns the
 * TRB and out->status may be stale. To poll, take a new snapshot each time.
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
 * Recompute cfg.stat.busy, which udc_ep_is_busy() reports.
 *   - EP0 halves: busy while a transfer is in progress (xfer.state != IDLE).
 *   - Other endpoints: busy while the ring holds TRBs not yet retired.
 * It is called only where those facts change: the xfer.state writers and the ring
 * writers (push, pop and ring release). The flag is written only when it changes,
 * since most arms and retires on a streaming ring leave it set.
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
 * Run after every xfer.state change. Updates cfg.stat.busy and wakes a controller
 * recovery waiting for transfers to end.
 */
static inline void udc_dwc3_ep_state_derive(struct udc_dwc3_ep_data *const ep_data)
{
    udc_dwc3_ep_busy_sync(ep_data);
    udc_dwc3_quiesce_progress(udc_get_private(ep_data->dev));
}

/*
 * Change xfer.state along a legal transition. Returns false and logs for an
 * illegal one. Every return to IDLE goes through udc_dwc3_ep_state_reset(), so
 * the table has no row that ends in IDLE.
 */
static bool udc_dwc3_ep_state_set(struct udc_dwc3_ep_data *const ep_data,
                  const enum udc_dwc3_ep_state next)
{
    static const uint8_t legal[][2] = {
        { UDC_DWC3_EP_IDLE,     UDC_DWC3_EP_STARTING },
        { UDC_DWC3_EP_STARTING,     UDC_DWC3_EP_RUNNING },
        /*
         * The deadline passed with the command still executing. The outcome is
         * unknown.
         */
        { UDC_DWC3_EP_STARTING,     UDC_DWC3_EP_START_UNKNOWN },
        { UDC_DWC3_EP_ENDING,       UDC_DWC3_EP_END_UNKNOWN },
        /*
         * UNKNOWN is left only on proof. DEPCMD (or cmd_record, if a later
         * command overwrote it) must show the same command type with CmdAct
         * clear. The Start and End refusals make sure no second Start or End
         * replaces it. In all other cases the exit is udc_dwc3_ep_state_reset().
         */
        { UDC_DWC3_EP_START_UNKNOWN,    UDC_DWC3_EP_RUNNING },
        /*
         * The same proof shows the End was refused. The transfer still runs and
         * owns its resource. udc_dwc3_ep_end_refused() restores the index first.
         */
        { UDC_DWC3_EP_END_UNKNOWN,  UDC_DWC3_EP_RUNNING },
        { UDC_DWC3_EP_RUNNING,      UDC_DWC3_EP_ENDING },
        /*
         * The End Transfer did not end the transfer. Either it was never issued
         * (the pre-poll gave up on a still-active command) or it was refused.
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
 * An End Transfer is outstanding, either executing or with an unknown outcome.
 * The controller may still own the ring.
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
 * it. Only xfer.state decides this.
 */
static inline bool udc_dwc3_ep_cmd_busy(const struct udc_dwc3_ep_data *const ep_data)
{
    return ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
           udc_dwc3_ep_is_ending(ep_data) ||
           udc_dwc3_ep_is_unknown(ep_data);
}

/*
 * Return to IDLE from any state, bypassing the transition table. It is used where
 * the controller holds no transfer on this endpoint: teardown (bus reset,
 * disconnect, disable, DEPSTARTCFG, controller recovery), a completed stage or
 * ring, a concluded End, and a Start proven refused.
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
     * An owed Update belongs to the transfer and ends with it. The other pending
     * bits are owed to the stack or the host, not to a command, so they stay.
     * udc_dwc3_ep_recover() carries them out once the endpoint is idle. Teardown
     * clears them itself (udc_dwc3_ep_disable(), udc_dwc3_drop_xfer_state()).
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

/* The End Transfer has concluded. Run what it owed. Defined below. */
static void udc_dwc3_ep_end_completed(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data);

/*
 * Result of a posted endpoint command. CmdAct still set means still executing,
 * which is neither success nor rejection. Treating it as either can leak a
 * transfer resource the controller really took. CMD_OTHER is not an outcome. It
 * means DEPCMD holds a different command.
 */
enum udc_dwc3_cmd_outcome {
    UDC_DWC3_CMD_UNKNOWN = 0,   /* CmdAct set: still executing          */
    UDC_DWC3_CMD_OK,        /* CmdAct clear, CmdStatus OK           */
    UDC_DWC3_CMD_ERROR,     /* CmdAct clear, CmdStatus not OK       */
    UDC_DWC3_CMD_OTHER,     /* DEPCMD holds another command type    */
};

/*
 * Classify DEPCMD as the outcome of this endpoint's command of type cmdtyp. This
 * is the only place the rule is written. *reg_out (optional) gets the value used.
 *
 * - CmdAct clear means finished, not failed. CmdStatus says which.
 * - The command type must match too. DEPCMD holds the last command posted, which
 *   may be a later one (such as a Set Stall) or an earlier one (when the pre-poll
 *   gave up without posting). Reading its status as ours could, for example,
 *   release a ring under a live transfer after a refused End.
 * - If a later command overwrote ours, udc_dwc3_depcmd() saved the old value in
 *   cmd_record, and that answers instead.
 * - CMD_OTHER means neither holds it. Right after posting, it means the pre-poll
 *   gave up and nothing was posted.
 */
static enum udc_dwc3_cmd_outcome
udc_dwc3_cmd_outcome(const struct device *const dev,
             const struct udc_dwc3_ep_data *const ep_data,
             const uint32_t cmdtyp,
             uint32_t *const reg_out)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    uint32_t       reg = sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn));

    /*
     * DEPCMD holds another command. Answer from cmd_record if it holds the
     * command asked about. The record is saved only with CmdAct clear.
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
 * The databook does not say they are latched when CmdAct is written. So they must
 * not change while a command on this endpoint is still executing.
 * udc_dwc3_depcmd() writes them after its pre-poll, just before CmdAct. A write at
 * the call site could land on the previous, still-running command.
 *
 * A command with operands writes all three, with unused ones zero, as the
 * reference driver does. A command without operands passes NULL.
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
 * Returns 0 when the command completed with status OK. Otherwise it returns
 * UDC_DWC3_XFERRSCIDX_INVALID: rejected, not issued, or still executing when the
 * poll ran out. Callers that must tell these apart use udc_dwc3_cmd_outcome().
 * Two more values exist. A Start Transfer that is not issued returns
 * UDC_DWC3_DEPCMD_NOT_POSTED, so its caller knows nothing was posted. A Start or
 * Update Transfer posted without the post-poll returns UDC_DWC3_DEPCMD_POSTED.
 */
static uint32_t udc_dwc3_depcmd(const struct device *const dev,
                const uint32_t addr, const uint32_t cmd,
                const struct udc_dwc3_depcmd_par *const par)
{
    const mm_reg_t                      base = DEVICE_MMIO_NAMED_GET(dev, base);
    const struct udc_dwc3_config *const cfg = DEV_CFG(dev);
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    const uint32_t                      epn = udc_dwc3_depcmd_epn(addr);
    /* Target endpoint, or NULL if addr is not an endpoint command register. */
    struct udc_dwc3_ep_data *const ep = _EPN_IS_VALID(cfg, epn)
                         ? _EP_DATA_FROM_EPN(cfg, epn) : NULL;
    const bool first_on_ep = (ep == NULL) || ep->cmd.depcmd_last == UDC_DWC3_DEPCMD_NONE;
    /* A Start or End Transfer: the commands whose outcome is tracked. */
    const bool opens = ((cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK) ==
                UDC_DWC3_DEPCMD_DEPSTRTXFER) ||
               ((cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK) ==
                UDC_DWC3_DEPCMD_DEPENDXFER);
    uint32_t   reg = 0;

    /*
     * A new Start or End clears the saved record before the pre-poll. If the
     * pre-poll gives up, the caller's outcome check then cannot find an older
     * command of the same type.
     */
    if (ep != NULL && opens) {
        ep->cmd.cmd_record = 0U;
    }

    /*
     * Never write a command over one still running. CmdAct is R/W1S, and the result
     * is undefined: dropped, doubled, or run with the old operands. The first
     * command on an endpoint is the exception. DEPCMD reads undefined then and CmdAct
     * may read set (1.3.12), so it is issued without the check.
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
     * DEPCMD is about to be overwritten. If it holds the outcome of this endpoint's
     * open Start or End, save it in cmd_record so the resolver can still find it.
     * This runs before the adoption below, which may close the Start.
     */
    if (!first_on_ep && !opens && udc_dwc3_ep_cmd_busy(ep)) {
        const uint32_t typ = reg & UDC_DWC3_DEPCMD_CMDTYP_MASK;

        if (typ == UDC_DWC3_DEPCMD_DEPSTRTXFER ||
            typ == UDC_DWC3_DEPCMD_DEPENDXFER) {
            ep->cmd.cmd_record = reg;
        }
    }

    /*
     * The pre-poll value may complete this endpoint's open Start. This is skipped
     * when posting a Start or End. The caller has already set STARTING or ENDING,
     * and a Start found in DEPCMD then belongs to an earlier transfer, with a
     * different index.
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
     * The transfer resource index lives and dies with Start and End Transfer
     * (3.2.2.2). So it is invalidated here, where every command is issued.
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
     * Write the operands, then the command, both after the pre-poll. So operands
     * never land on a command still executing. The give-up path above writes
     * nothing.
     */
    if (par != NULL && epn < UDC_DWC3_MAX_EPN) {
        sys_write32(par->par0, base + UDC_DWC3_DEPCMDPAR0(epn));
        sys_write32(par->par1, base + UDC_DWC3_DEPCMDPAR1(epn));
        sys_write32(par->par2, base + UDC_DWC3_DEPCMDPAR2(epn));
    }

    sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);

    /*
     * DEPCMD has now been written, so its value is defined and the next command on
     * this endpoint can pre-poll it.
     */
    if (ep != NULL) {
        ep->cmd.depcmd_last = cmd;
        ep->diag.cmd_reported = false;
        /*
         * Start the UDC_DWC3_CMD_UNKNOWN_MS deadline. Only a Start or End has one,
         * so a later Set Stall or DEPCFG does not extend it.
         */
        if (opens) {
            ep->cmd.cmd_t0 = k_cycle_get_32();
        }
    }

    /*
     * Poll for the result. Update Transfer has CmdIOC=0 and raises no Command
     * Complete, so its CmdStatus can only be read here. Without the post-poll
     * (UDC_DWC3_DEPCMD_POST_POLL), Start and Update return POSTED instead. The
     * #define explains how their outcome is learnt then.
     */
    if (ep != NULL) {
        const uint32_t cmdtyp = cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK;
        uint32_t       done = 0;
        bool           finished;

        if (!UDC_DWC3_DEPCMD_POST_POLL &&
            (cmdtyp == UDC_DWC3_DEPCMD_DEPSTRTXFER || cmdtyp == UDC_DWC3_DEPCMD_DEPUPDXFER)) {
            return UDC_DWC3_DEPCMD_POSTED;
        }

        finished = udc_dwc3_wait_cmdact_zero(dev, addr, &done);

        /*
         * Read once more before reporting. The LOG_WRN below is slow (about 11 ms
         * with CONFIG_LOG_MODE_MINIMAL, with the mutex held). If the command
         * finished in that time, the caller's re-read would see CmdAct clear and
         * take a success for a rejection. It would then reset the endpoint and
         * orphan its transfer resource. CmdStatus is valid as soon as CmdAct clears.
         */
        if (!finished) {
            done = sys_read32(base + addr);
            finished = (done & UDC_DWC3_DEPCMD_CMDACT) == 0U;
        }

        if (!finished) {
            priv->diag.depcmd_n_timeout++;
            /* Not a success yet. Callers resolve it with udc_dwc3_cmd_outcome(). */
            LOG_WRN("EP%02x command 0x%x on addr 0x%x still executing past the "
                "poll (0x%08x), outcome pending (%u so far)",
                ep->cfg.addr, cmd, addr, done, priv->diag.depcmd_n_timeout);
            return UDC_DWC3_XFERRSCIDX_INVALID;
        }

        /*
         * Only Start Transfer assigns a transfer resource. The adopt function
         * checks the value before trusting it.
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
    uint32_t       param0 = 0;
    uint32_t       param1 = 0;

    LOG_INF("Configuring endpoint 0x%02x with wMaxPacketSize=%u",
        ep_data->cfg.addr, ep_data->cfg.mps);

    /* The caller says whether this is Init or Modify. */
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

    /* Max packet size from the USB descriptor. */
    param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_MPS_MASK, ep_data->cfg.mps);

    /*
     * BrstSiz is packets per burst minus 1. Control endpoints do not burst, so it is
     * 0 for EP0 (Table 4-1).
     */
    if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
        param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 0);
    } else {
        /*
         * Burst of 4, matching bMaxBurst=3 in the class descriptors and
         * DCFG.NUMP=4, as the controller vendor specified.
         */
        param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 3);
    }

    /* FIFO number. It must be 0 for every OUT endpoint. */
    if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
        param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK,
                     ep_data->cfg.addr & 0x7f);
    }

    /* Per-endpoint events */
    param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
    param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN;

    /*
     * XferNotReady is enabled on every endpoint except video.
     *   - EP0 needs it for control transfer handling (4.2.4).
     *   - Other endpoints use it to see the host asking while the controller has no
     *     TRB to fetch (udc_dwc3_on_xfer_not_ready_nonctrl()).
     *   - On the video bulk-IN stream that is the normal state between frames, so it
     *     would only add events.
     */
    if (ep_data->cfg.addr != UDC_DWC3_VIDEO_EP) {
        param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
    }

    /* USB endpoint number. The physical endpoint number uses the same encoding. */
    param1 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK, ep_data->epn);

    /*
     * bInterval_m1 is bInterval - 1, and 0 at Full-Speed (DEPCFG field description).
     * It is required for isochronous endpoints (4.3.3) and means the same for
     * interrupt ones.
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
 * DEPXFERCFG: allocate this endpoint's transfer resource. Issue it only on
 * endpoint enable. Issuing it again leaks a resource.
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
 * After udc_dwc3_depcmd() failed, check whether the command was posted and not
 * refused. That is, it is still executing (and will complete), or it completed
 * late with status OK.
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
 * still executing past the poll). When it did, cfg.stat.halted is set.
 */
static UDC_DWC3_COLD bool udc_dwc3_depcmd_set_stall(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data)
{
    LOG_DBG("DepSetStall: EP%02x", ep_data->cfg.addr);

    /*
     * udc_dwc3_depcmd() fails both for a refusal and for a command still executing
     * past its poll. The second one was posted and takes effect.
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
     * udc_dwc3_ep_resume() also un-stalls the endpoints it restores, through this
     * function. Clearing the flag anywhere else could leave the hardware un-halted
     * with halted still true. udc_dwc3_ep_worker() would then refuse every buffer.
     */
    ep_data->cfg.stat.halted = false;

    return true;
}

/* Defined below. */
static bool udc_dwc3_depcmd_end_xfer(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data,
                     uint32_t flags);

/* Defined below. udc_dwc3_depcmd_start_xfer() and udc_dwc3_ep_update_owed() use it. */
static bool udc_dwc3_ep_ring_outstanding(const struct udc_dwc3_ep_data *const ep_data);

/* Defined below. Used for a Start refusal seen at post time. */
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
        const struct udc_dwc3_ep_data      *clash = NULL;

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
 * Post Start Transfer. Returns true when the Start was posted and not refused.
 * The transfer is then running, or STARTING until its outcome arrives.
 *
 * The caller writes the TRB first. The controller fetches it as soon as it
 * sees the command.
 * No link-state request is made. In U1/U2 the controller serves the Start after
 * it leaves the low-power state itself. In U3 (host suspended the device) it
 * waits for the host to resume. Writing ULSTCHNGREQ=Resume here would turn
 * every arm in U3 into an unrequested remote wake.
 */
static bool udc_dwc3_depcmd_start_xfer(const struct device *const dev,
                       struct udc_dwc3_ep_data *const ep_data)
{
    /* Filled in below and written with the command, not before it. */
    struct udc_dwc3_depcmd_par par;
    uint32_t                   idx;
    uint32_t                   cmd;

    /*
     * Resolve an open Start first, so the checks below see the controller's real
     * state. Only the Start side is resolved here. Resolving an End runs
     * udc_dwc3_ep_end_completed(), which resumes the endpoint and would re-enter
     * this function.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
        ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN) {
        udc_dwc3_ep_resolve_cmd(dev, ep_data);
    }

    /*
     * Never post a second Start while the first one's outcome is open. This applies
     * to control endpoints too. The controller may already hold a transfer resource
     * for the first Start. A second Start would take another one that the driver
     * cannot address or end.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
        udc_dwc3_ep_is_unknown(ep_data)) {
        LOG_ERR("EP%02x Start Transfer refused: the endpoint is %s, so a "
            "command outcome is still open", ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state));
        return false;
    }

    /*
     * INVARIANT 1: Start Transfer only from IDLE, on every endpoint. EP0 starts each
     * control stage as a new transfer. This also enforces 3.2.2.7: no Start while an
     * End Transfer is still completing on this endpoint.
     */
    if (ep_data->xfer.state != UDC_DWC3_EP_IDLE) {
        LOG_ERR("EP%02x Start Transfer refused: endpoint is %s, not idle",
            ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state));
        return false;
    }

    /*
     * PAR0/PAR1 hold the address of this transfer's first TRB, trb_buf[tail], not the
     * ring base. After a wrap the base slot has HWO clear. Starting there would make
     * the controller take the transfer resource and never move. EP0 has no ring and
     * always uses slot 0.
     */
    {
        const uint32_t  first = (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U)
                           ? 0U : ep_data->ring.tail;
        const uintptr_t trb0 = (uintptr_t)&ep_data->trb_buf[first];

        par.par0 = HI32(trb0);
        par.par1 = LO32(trb0);
        par.par2 = 0U;
    }

    /*
     * The databook returns the transfer resource index "in the DEPCMDn register and
     * in the Command Complete event".
     */
    cmd = UDC_DWC3_DEPCMD_DEPSTRTXFER;

    /*
     * CMDIOC requests that event. It lets the driver learn the outcome and the index
     * even when the poll in udc_dwc3_depcmd() does not see the command finish.
     */
    cmd |= UDC_DWC3_DEPCMD_CMDIOC;

    /*
     * Set STARTING before posting. Whoever sees the result first (post-poll,
     * pre-poll or Command Complete) then finds the endpoint waiting for it.
     */
    (void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_STARTING);

    idx = udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), cmd, &par);

    /*
     * Not posted: the pre-poll gave up because the previous command was still
     * executing (logged there). Go back to IDLE, as before the call. IDLE holds no
     * index, end_idx, cmd_record or owed Update, so the reset restores exactly that.
     */
    if (idx == UDC_DWC3_DEPCMD_NOT_POSTED) {
        udc_dwc3_ep_state_reset(ep_data);
        return false;
    }

    /*
     * Posted without the post-poll. It stays STARTING until its Command Complete,
     * the next command's pre-poll or the resolver settles it.
     */
    if (idx == UDC_DWC3_DEPCMD_POSTED) {
        return true;
    }

    /*
     * The Start was posted, so the failure means it is still executing past the
     * poll, completed late, or was refused.
     */
    if (idx == UDC_DWC3_XFERRSCIDX_INVALID) {
        struct udc_dwc3_data *const priv = udc_get_private(dev);
        uint32_t                        done = 0U;
        const enum udc_dwc3_cmd_outcome out =
            udc_dwc3_cmd_outcome(dev, ep_data, UDC_DWC3_DEPCMD_DEPSTRTXFER,
                         &done);

        /*
         * Still executing is neither success nor refusal. Leave it STARTING for its
         * Command Complete or the resolver.
         */
        if (out == UDC_DWC3_CMD_UNKNOWN) {
            LOG_WRN("EP%02x Start Transfer still executing past the poll "
                "budget: left STARTING for its Command Complete rather "
                "than resetting the endpoint under a live command",
                ep_data->cfg.addr);
            return true;
        }

        /*
         * The command completed after the poll ran out, for example during the slow
         * log line in udc_dwc3_depcmd(). Adopt the index. Resetting to IDLE would
         * abandon a resource the controller just assigned, with no index left to end
         * it.
         */
        if (out == UDC_DWC3_CMD_OK) {
            udc_dwc3_adopt_xferrscidx(dev, ep_data, done);
            return true;
        }

        /*
         * Refused. CMD_OTHER (DEPCMD not holding a Start) cannot follow a posted
         * Start and is handled the same way. No resource was assigned. CmdStatus
         * 4'h1 on Start Transfer means "no transfer resource available on the
         * endpoint", and 3.2.2.2 describes how to get one back. There is no retry.
         *
         * A non-control endpoint with buffers armed is taken out of service, the
         * same as when the refusal is learnt later (udc_dwc3_ep_start_refused()).
         * This stops anything from re-issuing the Start for an armed ring on an IDLE
         * endpoint. In the other cases (EP0, whose callers recover the control
         * transfer, or an empty ring set up during enable) the endpoint returns to
         * IDLE and the caller acts on false.
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
     * Reached only with the post-poll on and the Start completed with status OK.
     * udc_dwc3_depcmd() has already adopted the index and set RUNNING.
     */
    LOG_DBG("start EP%02x completed, transfer resource index adopted",
        ep_data->cfg.addr);

    return true;
}

/*
 * Take the transfer resource index from a DEPCMD value. This is done only if
 * the value is a finished, successful Start Transfer.
 */
static void udc_dwc3_adopt_xferrscidx(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const uint32_t reg)
{
    if (ep_data->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID) {
        return;
    }

    /*
     * Only a Start still in progress waits for an index. In any other state the
     * value in DEPCMD belongs to an older transfer and is ignored.
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
     * With the index known, the Start is done and the endpoint is RUNNING.
     * Several code paths can get here first. The transition is the same for
     * all of them.
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
     * A Start is still open and DEPCMD no longer holds its result. The event
     * completes it. (udc_dwc3_on_ep_cmd_cmplt() checks DEPCMD first.)
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
        ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN) {
        udc_dwc3_store_xferrscidx(dev, ep_data, idx);
        (void)udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_RUNNING);
        return;
    }

    /*
     * RUNNING on the same index is normal. Another path has already read the
     * index from DEPCMD before this event was handled.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_RUNNING) {
        if (ep_data->xferrscidx == idx) {
            return;
        }

        /* RUNNING with no index. This event is the only source, so take it. */
        if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
            udc_dwc3_store_xferrscidx(dev, ep_data, idx);
            return;
        }

        /*
         * A different index on a running endpoint. Keep the one in use and
         * log the mismatch.
         */
        LOG_WRN_RATELIMIT("EP%02x Start Transfer completion carries index %u "
                  "but the endpoint is running on index %u: NOT adopted",
                  ep_data->cfg.addr, idx, ep_data->xferrscidx);
        return;
    }

    /*
     * In any other state (IDLE, or an End in progress) there is no Start this
     * completion could belong to. It is ignored.
     */
    LOG_WRN_RATELIMIT("EP%02x Start Transfer completion for index %u arrived "
              "while the endpoint is %s holding index %u: NOT adopted",
              ep_data->cfg.addr, idx,
              udc_dwc3_ep_state_name(ep_data->xfer.state),
              ep_data->xferrscidx);
}

/* Take the transfer resource index from DEPCMD if it is already there. */
static void udc_dwc3_peek_xferrscidx(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

    if (ep_data->xferrscidx != UDC_DWC3_XFERRSCIDX_INVALID) {
        return;
    }

    /* DEPCMD is undefined until the first command on this endpoint. */
    if (ep_data->cmd.depcmd_last == UDC_DWC3_DEPCMD_NONE) {
        return;
    }

    udc_dwc3_adopt_xferrscidx(dev, ep_data,
                  sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn)));
}

/*
 * Issue Update Transfer. Returns true when the command was issued.
 * A race cannot refuse it. For a transfer already completed, "the controller
 * will detect that the Update Transfer is unnecessary" (3.2.2.6). Its only
 * error, an index never started, is a driver bug. The next command's pre-poll
 * logs it.
 * The caller writes the TRB first, as for Start Transfer. This runs once per
 * buffer on every bulk and interrupt endpoint.
 */
static bool udc_dwc3_depcmd_update_xfer(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data)
{
    uint32_t flags = 0;

    udc_dwc3_peek_xferrscidx(dev, ep_data);

    /* Update Transfer needs a resource index to address. */
    if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
        if (ep_data->xfer.state == UDC_DWC3_EP_STARTING ||
            ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN) {
            /* The Start is still running. Issue the Update when it ends. */
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
     * INVARIANT 2: Update Transfer only from RUNNING. Other states hold no
     * running transfer. With a valid index this should not happen, so a hit
     * is a real state error.
     */
    if (ep_data->xfer.state != UDC_DWC3_EP_RUNNING) {
        LOG_ERR("Update Transfer on EP%02x refused: endpoint is %s, not "
            "running (rscidx 0x%x)", ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state),
            ep_data->xferrscidx);
        return false;
    }

    flags |= UDC_DWC3_DEPCMD_DEPUPDXFER;
    flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

    /* A failure is already logged by udc_dwc3_depcmd(). */
    if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags, NULL) ==
        UDC_DWC3_XFERRSCIDX_INVALID) {
        return false;
    }

    /* Debug level, because this runs once per buffer. */
    LOG_DBG("DepUpdateXfer done EP%02x, addr 0x%08x, data 0x%08x, xferrscidx 0x%x",
        ep_data->cfg.addr, UDC_DWC3_DEPCMD(ep_data->epn), flags, ep_data->xferrscidx);

    return true;
}

/*
 * Issue the Update Transfer owed by a buffer armed while the Start was still
 * open (UDC_DWC3_EP_PEND_UPDATE). Called once the Start has its index: from its
 * Command Complete, or from the sweep when that event is lost. The Update is
 * needed because the controller may not fetch a TRB added after the Start.
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
 * The End Transfer was refused or never posted. The transfer still runs and
 * owns its resource, so restore the index and go back to RUNNING. Called from
 * udc_dwc3_depcmd_end_xfer() and udc_dwc3_ep_resolve_cmd().
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
 * Issue End Transfer. Returns true only when a Command Complete event will
 * follow, so the caller can wait for it.
 */
static UDC_DWC3_COLD bool udc_dwc3_depcmd_end_xfer(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data,
                     uint32_t flags)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

    if (ep_data->xferrscidx == UDC_DWC3_XFERRSCIDX_INVALID) {
        /*
         * No index means no running transfer to end. The endpoint is IDLE, or
         * a Start or End is still in progress. Linux dwc3 makes the same check.
         * The state is left for that command's completion or for
         * udc_dwc3_ep_resolve_cmd() to settle.
         */
        LOG_DBG("End Transfer on EP%02x not issued: no started transfer",
            ep_data->cfg.addr);
        return false;
    }

    flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
    flags |= UDC_DWC3_DEPCMD_DEPENDXFER;

    /*
     * Ask for a completion event. CmdAct clearing only means the command was
     * accepted. The completion event is the only sign that bus traffic for this
     * transfer has stopped, and a new Start Transfer must wait for it. Only
     * while RunStop is set.
     */
    if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0) {
        flags |= UDC_DWC3_DEPCMD_CMDIOC;

        /* INVARIANT 3: only from RUNNING. */
        if (!udc_dwc3_ep_state_set(ep_data, UDC_DWC3_EP_ENDING)) {
            return false;
        }
    }

    /* Keep the index in end_idx, so a refused End can restore it (see xfer.end_idx). */
    ep_data->xfer.end_idx = ep_data->xferrscidx;

    /*
     * A failure here means the End was not issued, was refused, is still
     * running, or completed after the poll expired.
     */
    if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags, NULL) ==
        UDC_DWC3_XFERRSCIDX_INVALID) {
        /*
         * udc_dwc3_cmd_outcome() tells these cases apart. It checks the command
         * type in DEPCMD first. If the End was not issued, DEPCMD still holds the
         * previous command, and its CmdAct says nothing about our End.
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
         * Still executing. With RunStop set, the endpoint stays ENDING until its
         * Command Complete. With RunStop clear, no event was requested, so the
         * caller treats the controller as stopped.
         */
        if (out == UDC_DWC3_CMD_UNKNOWN) {
            LOG_WRN("EP%02x End Transfer still executing past the poll "
                "budget%s", ep_data->cfg.addr,
                ((flags & UDC_DWC3_DEPCMD_CMDIOC) != 0U)
                    ? ": left ENDING for its Command Complete" : "");
            return (flags & UDC_DWC3_DEPCMD_CMDIOC) != 0U;
        }

        /*
         * The End completed after the poll expired, so the resource is released.
         * end_idx is not restored, because the controller may give that index to
         * another endpoint.
         */
        if (out == UDC_DWC3_CMD_OK) {
            return (flags & UDC_DWC3_DEPCMD_CMDIOC) != 0;
        }

        /* Refused. The transfer still owns its resource (see end_idx above). */
        udc_dwc3_ep_end_refused(dev, ep_data);
        LOG_ERR("End Transfer REJECTED on EP%02x (CmdAct clear, so the "
            "controller refused it rather than still running it)",
            ep_data->cfg.addr);
        return false;
    }

    LOG_DBG("DepEndXfer done EP%02x", ep_data->cfg.addr);

    /* True only when a completion event will follow. */
    return (flags & UDC_DWC3_DEPCMD_CMDIOC) != 0;
}

/* DEPSTARTCFG: set up the controller's pool of transfer resources. */
static UDC_DWC3_COLD void udc_dwc3_depcmd_start_config(const struct device *const dev,
                     bool is_control)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    /* Non-control: the reset loops below skip index 0, the EP0 halves. */
    const uint8_t first = is_control ? 0U : 1U;
    uint32_t      flags = 0;

    /* XferRscIdx 2 keeps resources 0 and 1, which the two EP0 halves use. */
    flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, is_control ? 0 : 2);
    flags |= UDC_DWC3_DEPCMD_DEPSTARTCFG;

    /* Not posted or refused. The pool and every index stay unchanged. */
    if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(0), flags, NULL) != 0U &&
        !udc_dwc3_cmd_posted_ok(dev, &cfg->ep_data_out[0],
                    UDC_DWC3_DEPCMD_DEPSTARTCFG)) {
        LOG_ERR("DepStartConfig (%s) not taken by the controller",
            is_control ? "control" : "non-control");
        return;
    }

    /*
     * DEPSTARTCFG reassigns the resources it covers, so every index and transfer
     * state on those endpoints is now invalid. Reset them. Callers issue it with
     * those endpoints idle. Pending work owed to the stack or host is kept.
     */
    for (uint8_t i = first; i < cfg->num_in_eps; i++) {
        udc_dwc3_ep_state_reset(&cfg->ep_data_in[i]);
    }
    for (uint8_t i = first; i < cfg->num_out_eps; i++) {
        udc_dwc3_ep_state_reset(&cfg->ep_data_out[i]);
    }

    /*
     * A new pool generation. Each endpoint issues DEPXFERCFG again, exactly once
     * (see udc_dwc3_ep_resume()).
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
 * BUFSIZ must be a multiple of MPS (databook 4.2.3.3), so the TRB is programmed
 * with the size rounded up. A full last packet can then write up to MPS - 1
 * bytes past the buffer. The class sizes the buffer, so the driver only reports.
 */
static void udc_dwc3_out_size_check(const struct device *const dev,
                    const struct udc_dwc3_ep_data *const ep_data,
                    const uint32_t in_size, const uint32_t trb_size)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    /* The normal case: the buffer is already whole packets. */
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
 * The controller overwrites BUFSIZ with the bytes left over, so the value is
 * computed from the buffer. Arm and retire both use this function.
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

/* Arm one buffer in the endpoint's TRB ring and advance head. */
static void udc_dwc3_push_trb(const struct device *const dev,
                  struct udc_dwc3_ep_data *const ep_data,
                  struct net_buf *const buf, const uint32_t ctrl)
{
    volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->ring.head];
    const uint32_t                      out_size = udc_dwc3_trb_programmed_len(ep_data, buf);

    if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
        udc_dwc3_out_size_check(dev, ep_data, buf->size, out_size);
    }

    /*
     * The slot must be free. Callers check this and retry later when the ring
     * is full.
     */
    __ASSERT_NO_MSG(ep_data->ring.net_buf[ep_data->ring.head] == NULL);

    /* Link the buffer to its TRB slot. */
    ep_data->ring.net_buf[ep_data->ring.head] = buf;

    ep_data->diag.n_arm++;

    udc_dwc3_trb_fill(trb, (uintptr_t)buf->data, out_size, ctrl);

    LOG_DBG("PUSH %u, buf %p, data %p, size %u -> %u",
        ep_data->ring.head, (void *)buf, (void *)buf->data, buf->size, out_size);

    /*
     * Per-arm trace, skipped on the video endpoint. Debug level, because it runs
     * once per buffer.
     */
    if (ep_data->cfg.addr != UDC_DWC3_TRBLOG_SKIP_EP) {
        LOG_DBG("EP%02x: ARM s%u len=%u n=%u", ep_data->cfg.addr,
            ep_data->ring.head, out_size, ep_data->diag.n_arm);
    }

    ep_data->ring.head = (ep_data->ring.head + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

    udc_dwc3_ep_busy_sync(ep_data);
}

/* True if any slot is still owned by the controller (HWO) or holds a buffer. */
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
 * Retire the oldest completed TRB. Returns -EBUSY while the controller still
 * owns it and -ENOBUFS when the slot holds no buffer.
 *
 * HWO is read first. The rest of the TRB is read only after HWO reads clear,
 * so the written-back BUFSIZ is always final.
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

    /* Free the slot. */
    ep_data->ring.net_buf[ep_data->ring.tail] = NULL;

    LOG_DBG("POP %u EP%02x, buf %p, data %p",
        ep_data->ring.tail, ep_data->cfg.addr, (void *)*buf, (void *)(*buf)->data);

    /* Retire trace. With the arm trace it shows each slot's arm-to-retire time. */
    if (ep_data->cfg.addr != UDC_DWC3_TRBLOG_SKIP_EP) {
        LOG_DBG("EP%02x: RET s%u sts=%x n=%u", ep_data->cfg.addr,
            ep_data->ring.tail, trb->status, ep_data->diag.n_retire);
    }

    /* The last slot is the link TRB. */
    ep_data->ring.tail = (ep_data->ring.tail + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

    udc_dwc3_ep_busy_sync(ep_data);

    /* Received length is the programmed size minus the residual BUFSIZ. */
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

/* Build a non-control endpoint's ring and start its transfer. */
static int udc_dwc3_trb_nonctrl_init(const struct device *const dev,
                   struct udc_dwc3_ep_data *const ep_data)
{
    volatile struct udc_dwc3_trb *trb = ep_data->trb_buf;
    const uint32_t                i = CONFIG_UDC_DWC3_TRB_NUM - 1;

    LOG_DBG("Initializing normal TRB");

    /* All TRBs start with HWO clear, so nothing moves until a buffer is armed. */
    memset((void *)trb, 0x00, sizeof(*trb) * CONFIG_UDC_DWC3_TRB_NUM);

    /* The last TRB is a link back to the start of the ring. */
    udc_dwc3_trb_fill(&trb[i], (uintptr_t)ep_data->trb_buf, 0U,
              UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB | UDC_DWC3_TRB_CTRL_HWO);


    /* Start the transfer now. Buffers are added later with Update Transfer. */
    if (!udc_dwc3_depcmd_start_xfer(dev, ep_data)) {
        LOG_ERR("EP%02x ring primed but Start Transfer failed; not enabling",
            ep_data->cfg.addr);
        return -EIO;
    }

    return 0;
}

/*
 * Fill and start one control OUT stage TRB (data or status) on EP0-OUT. The
 * caller, udc_dwc3_ctrl_try(), has checked that the stage is due and that the
 * half holds no transfer.
 */
static bool udc_dwc3_trb_ctrl_out(const struct device *const dev, struct net_buf *const buf,
                  const uint32_t ctrl)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_ep_data *const      ep_data = &cfg->ep_data_out[0];
    volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
    uint32_t                            size;

#ifdef CONFIG_UDC_DWC3_SHELL
    ((struct udc_dwc3_data *)udc_get_private(dev))->diag.last_xfer_dir = USB_EP_DIR_OUT;
#endif

    /*
     * A Status TRB has BUFSIZ 0 (Programming Guide 3.30b: "There is no data
     * buffer associated with a Status TRB"). The OUT multiple-of-MPS rule does
     * not apply to it.
     */
    if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
        ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
        size = 0U;
    } else {
        size = buf->size;
    }

    /*
     * A control OUT data TRB's BUFSIZ must be a multiple of wMaxPacketSize
     * (4.4.2 step 5a). Otherwise a ZLP from the host has nowhere to go.
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
 * Fill and start one control IN stage TRB (data or status) on EP0-IN. A chained
 * zero-length TRB is added when the data stage needs a ZLP. The caller checks
 * as for udc_dwc3_trb_ctrl_out().
 */
static bool udc_dwc3_trb_ctrl_in(const struct device *const dev,
                 struct net_buf *const buf,
                 const uint32_t ctrl)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_ep_data *const      ep_data = &cfg->ep_data_in[0];
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
 * Arm one bulk or interrupt buffer and tell the controller about it. Returns
 * -EBUSY if the ring is full. Returns -EIO if the endpoint is out of service
 * after a refused Start. In both cases nothing is armed and the caller keeps
 * the buffer.
 */
static int udc_dwc3_trb_bulk(const struct device *const dev,
                 struct udc_dwc3_ep_data *const ep_data,
                 struct net_buf *const buf)
{
    uint32_t ctrl = UDC_DWC3_TRB_CTRL_IOC | UDC_DWC3_TRB_CTRL_HWO;

    /*
     * CSP (Continue on Short Packet) is for OUT only. LST is never set, so IN
     * gets XferInProgress, not XferComplete (Table 4-8). Both events use the
     * same handler (udc_dwc3_on_xfer_done_nonctrl()).
     */
    if (!USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
        ctrl |= UDC_DWC3_TRB_CTRL_CSP;
    }

    /* Debug level, because a log line per transfer slows all traffic. */
    LOG_DBG("TRB_BULK_EP_0x%02x, buf %p, data %p, size %u, len %u",
        ep_data->cfg.addr, (void *)buf, (void *)buf->data, buf->size, buf->len);

    if (ep_data->ring.net_buf[ep_data->ring.head] != NULL) {
        return -EBUSY;
    }

    /*
     * After a refused Start the endpoint is IDLE and removed from DALEPENA
     * (udc_dwc3_ep_start_refused()). Nothing is armed until an enable adds it
     * back. This also stops the caller's loop after a refusal in this call.
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
     * IDLE means no transfer holds a resource, so the buffer needs a Start.
     * Otherwise an Update adds it to the running transfer.
     *
     * A failure is not undone. The TRB is armed and net_buf[] owns the buffer.
     * Returning an error would leave the buffer in both net_buf[] and the
     * stack's queue, and it would be freed twice. An Update that must wait for
     * an open Start is issued later by udc_dwc3_ep_update_owed(). Other failures
     * are logged where they happen.
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
 *
 * Arming the control endpoint stages: SETUP, data and status. Figure 4-2 says
 * which stage is due next.
 */

/*
 * Record the control endpoint and stage just armed, for the SETUP watchdog
 * report (udc_dwc3_ctrl_setup_wd_check()).
 */
static void udc_dwc3_ctrl_arm_watchdog(const struct device *const dev,
                       const bool is_in, const uint32_t type)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);

    priv->diag.watchdog_ep = is_in ? &cfg->ep_data_in[0] : &cfg->ep_data_out[0];
    priv->diag.watchdog_type = type;

    /*
     * Save the traffic counters. The watchdog uses them to tell an idle bus from
     * a busy one where the shared RxFIFO holds other endpoints' data.
     */
    if (type == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
        priv->diag.ctrl_setup_wd_snap_setup = priv->diag.ctrl_setup_done;
        priv->diag.ctrl_setup_wd_snap_nonctrl = priv->diag.nonctrl_done;

        /*
         * Watched for SETUP only. The heartbeat checks how long the SETUP has
         * been armed, so no timer is needed here.
         */
        priv->diag.ctrl_setup_wd_gen++;
    }
}

/*
 * Arm the data or status stage this buffer describes on EP0-IN. Returns false
 * if the Start Transfer was not taken. The buffer then stays queued.
 */
static bool udc_dwc3_ctrl_next_in(const struct device *const dev,
                  struct net_buf *const buf)
{
    struct udc_dwc3_data *const          priv = udc_get_private(dev);
    const struct usb_setup_packet *const setup = &priv->ctrl.setup;
    const struct udc_buf_info            bi = *udc_get_buf_info(buf);

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
        /* Also a ZLP: buf->len is what reaches the TRB. */
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
        /* udc_dwc3_trb_ctrl_out() programs BUFSIZ 0 for a status TRB. */
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
 * EP0 follows the control transfer model of SPEC 3.30b 4.4 / Figure 4-2. The
 * hardware "automatically recovers from these scenarios as long as software
 * follows this single control transfer programming model".
 *
 * ctrl.state is the current node of Figure 4-2. An event on physical endpoint
 * 0 or 1 either moves to the next node, is ignored, or is an error case that
 * udc_dwc3_ctrl_ep_recover() handles. The controller itself handles aborts,
 * suspend/resume and resets.
 *
 * Stage endpoints (4.4):
 *   - control read: SETUP EP0, data EP1, status EP0
 *   - control write / two-stage: SETUP EP0, data EP0, status EP1
 */
static void udc_dwc3_ctrl_next(const struct device *const dev);
static void udc_dwc3_ctrl_ep_recover(const struct device *const dev);

/*
 * Return the queued buffers of an abandoned transfer on this control endpoint.
 * Stops at a SETUP buffer. Returns how many were returned.
 */
static UDC_DWC3_COLD uint32_t udc_dwc3_ctrl_drain_abandoned(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data)
{
    struct net_buf *buf;
    uint32_t        n = 0U;

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
 * True if a SETUP TRB is armed on this control endpoint. Read from the TRB
 * itself, so it stays correct across abandon and re-arm.
 */
static bool udc_dwc3_ctrl_armed_setup(struct udc_dwc3_ep_data *const ep_data)
{
    /* One read, so both tests see the same word. */
    const uint32_t ctrl = ep_data->trb_buf[0].ctrl;

    return (ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0 &&
           (ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK) ==
               UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP;
}

/* The current request's data stage is device-to-host (4.4). */
static inline bool udc_dwc3_ctrl_dir_in(const struct udc_dwc3_data *const priv)
{
    return priv->ctrl.setup.RequestType.direction == USB_REQTYPE_DIR_TO_HOST;
}

/* The current request has a data stage (wLength != 0). */
static inline bool udc_dwc3_ctrl_three_stage(const struct udc_dwc3_data *const priv)
{
    return sys_le16_to_cpu(priv->ctrl.setup.wLength) != 0U;
}

/* True if the current request's status stage is IN (4.4). */
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
 * TRB and issues Start Transfer on EP0 pointing to the Setup TRB"). The TRB
 * uses the driver's own buffer. This runs only in the Setup phase with EP0-OUT
 * idle. Otherwise udc_dwc3_ctrl_next() calls it again when EP0-OUT is free.
 */
static void udc_dwc3_ctrl_arm_setup(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    struct udc_dwc3_ep_data *const      out0 = &cfg->ep_data_out[0];

    if (priv->ctrl.state != UDC_DWC3_CTRL_IDLE ||
        out0->xfer.state != UDC_DWC3_EP_IDLE) {
        return;
    }

    /*
     * In the Setup phase no transfer is live. A stage buffer at the head of
     * either EP0 queue therefore belongs to an abandoned transfer. The stack can
     * queue such buffers late, after the driver has gone back to Step 1. Left in
     * place they would block the check below, so return them.
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
     * Arm the SETUP only while the stack's SETUP buffer is queued on EP0-OUT, so
     * a SETUP never arrives before the stack is ready. An early SETUP takes a
     * path in the stack that can drop a reference to a still-queued buffer.
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
     * Not taken, so the controller does not own the TRB. Clear it. With HWO set
     * it would look like an armed SETUP and no new SETUP would be armed.
     */
    if (!udc_dwc3_depcmd_start_xfer(dev, out0)) {
        udc_dwc3_trb_write(&out0->trb_buf[0], 0U, 0U, 0U);
        return;
    }

    udc_dwc3_ctrl_arm_watchdog(dev, false, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);
}

/*
 * Arm the stage buffer queued on this half if Figure 4-2 says that stage is due
 * now on this endpoint and the half is idle. Otherwise the buffer waits. The
 * event that makes it due (XferNotReady(Status) or the SETUP's XferComplete)
 * calls back here.
 *
 * The stack's SETUP buffer is not armed here. The driver arms the SETUP stage
 * itself (udc_dwc3_ctrl_arm_setup()).
 */
static void udc_dwc3_ctrl_try(const struct device *const dev,
                  struct udc_dwc3_ep_data *const ep_data)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const bool                  is_in = USB_EP_DIR_IS_IN(ep_data->cfg.addr);
    struct net_buf *const       buf = udc_buf_peek(&ep_data->cfg);
    const struct udc_buf_info  *bi;
    bool                        armed = false;

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
     * The stage's Start Transfer was not taken, so no event will move this
     * transfer on. Recover back to Step 1 (Set Stall, then a new SETUP).
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

/* Read the core debug registers into *s, to compare a healthy and a stuck core. */
static void udc_dwc3_core_dbg_read(const mm_reg_t base,
                   struct udc_dwc3_core_dbg *const s)
{
    /*
     * GDBGLSP shows the source selected by its mux, so select it first. GDBGLSP
     * is logged raw, because the device-mode selector encoding is not confirmed.
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
    const mm_reg_t           base = DEVICE_MMIO_NAMED_GET(dev, base);
    struct udc_dwc3_core_dbg dbg;
    static const struct {
        const char *name;
        uint32_t    sel;
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
 * Reclaim one control half's TRBs. Clear both TRBs. On an IN half, also flush
 * the TxFIFO, which may hold data for a stage that never went out.
 *
 * SPEC 3.30b Programming Guide Table 4-14 step 8: "Software has to reclaim the
 * TRBs with HWO=1 in the skipped TRBs and flush the TxFIFO." Callers do this
 * only when no transfer owns the half, after its XferComplete (3.2.2.2) or End
 * Transfer completion.
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
 * Endpoint and control recovery
 *
 * Recovery of one endpoint or of the control endpoint, and the teardown of all
 * transfers on a disconnect.
 */

/* Defined below. Teardown uses it to move armed buffers off the ring. */
static void udc_dwc3_ep_ring_release(struct udc_dwc3_ep_data *const ep_data);

/* Defined below. Endpoint recovery uses it to retire completed TRBs first. */
static uint32_t udc_dwc3_drain_completed(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data);

/*
 * Return every buffer parked on requeue_fifo to the stack. Returns the count.
 *
 * udc_dwc3_ep_ring_release() parks armed buffers there, and normally
 * udc_dwc3_ep_resume() takes them back. On teardown the endpoint is not
 * resumed, so this returns them instead. Otherwise they would leak.
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
 * Recover a non-control endpoint: bring it back to a known state.
 * USB reset, disconnect, SetConfiguration, ClearFeature(ENDPOINT_HALT), dequeue,
 * disable and XferNotReady all use it. The End Transfer and Start Transfer
 * completions and the heartbeat sweep call it again to finish a recovery that
 * had to wait.
 *
 * What it does depends only on the endpoint state:
 *
 *   1. A command outcome is still open (STARTING, ENDING, *_UNKNOWN).
 *      Do nothing. The Command Complete, or the resolver reading DEPCMD,
 *      calls back here.
 *
 *   2. A transfer holds a resource (xferrscidx valid).
 *      Issue End Transfer with ForceRM and CmdIOC, so a Command Complete
 *      reports the end (SPEC 3.2.2.7). The databook requires this on USB reset
 *      (4.1.2, 4.2.5), on SetConfiguration (4.1.5), after ClearFeature(STALL)
 *      (4.2.7) and before buffers are given back. End Transfer raises no
 *      XferComplete and does not update TRB status (3.2.2.7). The controller
 *      owns the ring until the End completes (4.2.5).
 *
 *   3. No transfer. The ring belongs to the driver again. In order:
 *      - Buffers. After a dequeue, disable or USB reset they are returned as
 *        cancelled (-ECONNABORTED, like udc_ep_cancel_queued()). If a refused
 *        Start took the endpoint out of DALEPENA, they are parked for
 *        re-enable. Otherwise they stay on the ring.
 *      - Clear Stall, if owed. 4.2.7: End Transfer first, then Clear Stall.
 *        4.1.2: clear stalled endpoints on USB reset. Software owns STALL on
 *        non-control endpoints (3.2.2.4), so only those two requests owe it.
 *      - Start Transfer again if enabled (4.2.7). This is either a resume that
 *        waited for the End (3.2.2.7: no Start until the End has reported), or
 *        a Start for buffers still armed on the ring. Then the endpoint worker
 *        runs for anything the stack queued since.
 *
 * The only inputs besides controller state are the requests recorded in
 * ep_data->xfer.pending: Clear Stall owed, cancel owed, and a resume deferred
 * by udc_dwc3_ep_resume().
 */
static UDC_DWC3_COLD void udc_dwc3_ep_recover(const struct device *const dev,
                struct udc_dwc3_ep_data *const ep_data)
{
    const mm_reg_t                    base = DEVICE_MMIO_NAMED_GET(dev, base);
    const struct udc_dwc3_data *const priv = udc_get_private(dev);
    uint32_t                          returned;
    uint8_t                           owed;

    if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U) {
        return;
    }

    /* 1. A command outcome is open. Its completion calls back here. */
    if (udc_dwc3_ep_cmd_busy(ep_data)) {
        return;
    }

    /* 2. A transfer holds a resource. End it and wait for the report. */
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
            /* Refused while running. The controller still owns the ring. */
            LOG_ERR_RATELIMIT("EP%02x recovery: End Transfer refused, transfer "
                      "left active (owed 0x%02x)", ep_data->cfg.addr,
                      ep_data->xfer.pending);
            return;
        }
        /* The controller is stopped, so no completion will come. */
        udc_dwc3_ep_state_reset(ep_data);
    }

    /* 3. No transfer. Do what is owed, in the databook's order. */
    owed = ep_data->xfer.pending;
    ep_data->xfer.pending = UDC_DWC3_EP_PEND_NONE;
    returned = 0U;

    if ((owed & UDC_DWC3_EP_PEND_DEQUEUE) != 0U) {
        /* Return the buffers as cancelled. */
        udc_dwc3_ep_ring_release(ep_data);
        returned = udc_dwc3_ep_return_parked(dev, ep_data, UDC_DWC3_BUF_CANCELLED);
    } else if ((sys_read32(base + UDC_DWC3_DALEPENA) &
            UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn)) == 0U) {
        /* A refused Start took it out of DALEPENA. Park for the next resume. */
        udc_dwc3_ep_ring_release(ep_data);
    }

    if ((owed & UDC_DWC3_EP_PEND_CLEAR_STALL) != 0U &&
        !udc_dwc3_depcmd_clear_stall(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
        LOG_ERR("EP%02x recovery: Clear Stall refused; endpoint remains halted",
            ep_data->cfg.addr);
    }

    /*
     * Log only when something was done. A USB reset or disconnect recovers
     * every endpoint, and most of them have nothing to report.
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
         * A resume starts a transfer, and 4.1.8 forbids that while transfers
         * are being ended. Keep it owed. The controller reset that follows
         * clears it (udc_dwc3_drop_xfer_state()).
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
         * Buffers still armed on the ring need their own Start. The worker only
         * arms what the stack queues, so without new buffers the ring would sit
         * idle. The gates match the worker. Completed TRBs are retired first, so
         * the Start points at a TRB the controller still owns.
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
 * Does this endpoint still owe a recovery? Yes if a request other than Update is
 * recorded, or if it is out of DALEPENA but still has a transfer or an armed ring.
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
 * Each half (EP0 OUT and EP0 IN) is handled by what the controller holds for
 * it, not by the control stage:
 *
 *   command outcome open    -> wait. Its completion calls back here.
 *   transfer resource valid -> End Transfer (ForceRM). Its completion calls
 *                              back here. If refused, the heartbeat retries.
 *   nothing live            -> return the half's queued stage buffers. The
 *                              stack's SETUP buffer stays.
 *
 * When both halves are idle, Set Stall if needed (after the End, per 4.4.2
 * step 3a). Then enter the Setup phase and arm it.
 *
 * A valid xferrscidx means a live transfer because every EP0/EP1 completion
 * releases the half (udc_dwc3_ctrl_release()). This matches the controller,
 * which frees the resource at XferComplete (3.2.2.2).
 */
static UDC_DWC3_COLD void udc_dwc3_ctrl_recover_continue(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    const mm_reg_t                      base = DEVICE_MMIO_NAMED_GET(dev, base);
    struct udc_dwc3_ep_data *const      halves[2] = {
        &cfg->ep_data_out[0], &cfg->ep_data_in[0],
    };
    bool                                waiting = false;

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
             * Refused or not posted while running. The transfer still owns its
             * TRBs. Wait. The heartbeat's udc_dwc3_recover_all() calls back here
             * and retries the End.
             */
            if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0U) {
                waiting = true;
                continue;
            }
            /* The controller is stopped, so no completion will come. */
            udc_dwc3_ep_state_reset(h);
        }

        (void)udc_dwc3_ctrl_drain_abandoned(dev, h);
    }

    if (waiting) {
        return;
    }

    /* Neither half holds a transfer. The TRBs and TxFIFO 0 belong to the driver. */
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
 * Recover the control endpoint: go back to Step 1 (wait for a SETUP).
 *
 * Every databook case ends there:
 *   - 4.4.1/4.4.2 "go back to Step 1": bad setup bytes, a stage XferNotReady
 *     before the SETUP's XferComplete, a data stage on a two-stage request or
 *     in the wrong direction, more data than wLength, or a failed data stage
 *     at XferNotReady(Status).
 *   - 4.1.2 USB reset and 4.1.8 device-initiated disconnect: "complete it and
 *     get the controller into the Setup TRB / Start Transfer state".
 * Linux dwc3 does the same (dwc3_ep0_end_control_data and
 * dwc3_ep0_stall_and_restart).
 *
 * The state, not the caller, decides whether it ends with Set Stall.
 */
static UDC_DWC3_COLD void udc_dwc3_ctrl_ep_recover(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    struct udc_dwc3_ep_data *const      out0 = &cfg->ep_data_out[0];
    struct udc_dwc3_ep_data *const      in0 = &cfg->ep_data_in[0];
    /*
     * Set Stall answers a control transfer in progress that cannot complete
     * (4.4.1/4.4.2 steps 2, 3, 3a, 5b, 6). A USB reset (4.1.2) is handled the
     * same way. In progress means past the Setup phase, or a stage live on
     * either half. A running recovery keeps the choice it started with, and
     * once RECOVERING_STALL is chosen it stays.
     */
    const bool                     in_progress =
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
 * Handle a Start Transfer (STARTING / START_UNKNOWN) that is proven refused.
 * The proof comes from its Command Complete, from DEPCMD when that event is
 * lost (udc_dwc3_ep_resolve_cmd()), or at post time
 * (udc_dwc3_depcmd_start_xfer(), non-control endpoints with buffers armed only).
 * No transfer started and no resource was taken, so the ring belongs to the
 * driver again. src and val describe the evidence for the log.
 *
 *   EP0 half: nothing is armed and no event will move the control transfer
 *             on. Go back to Step 1, or let a running recovery continue.
 *   other:    no new Start. CmdStatus 4'h1 means no transfer resource for the
 *             endpoint, and another Start will not fix that. The endpoint
 *             leaves DALEPENA with its buffers parked, as a disable leaves
 *             them, and the stack is told. Owed work (pending) still runs.
 *             With at_post, the ring stays armed, and the heartbeat sweep's
 *             udc_dwc3_ep_recover() parks it later. The caller may still hold
 *             a buffer linked in the stack's queue, and parking it now would
 *             also link it into requeue_fifo.
 */
static UDC_DWC3_COLD void udc_dwc3_ep_start_refused(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data,
                    const char *const src, const uint32_t val,
                    const bool at_post)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const bool                  is_ctrl = USB_EP_GET_IDX(ep_data->cfg.addr) == 0U;

    priv->diag.ctrl_start_fail++;
    /* Logged here, so udc_dwc3_depcmd() skips its own CMDERR line. */
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

    /* Tell the stack. It still counts the endpoint as enabled. */
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
 * Forget every endpoint's transfer state. Use this only after a core soft reset
 * or a controller disable, when the controller holds nothing. A USB reset or
 * disconnect uses udc_dwc3_end_all_transfers() instead, because the controller
 * still holds its transfers then.
 */
static UDC_DWC3_COLD void udc_dwc3_drop_xfer_state(const struct device *const dev,
                     const char *const reason)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);

    LOG_DBG("dropping all transfer state (%s)", reason);

    priv->diag.watchdog_ep = NULL;
    priv->diag.watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
    priv->ctrl.setup_pending = false;
    udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_IDLE);
    /* A U3 from the old session owes no U3-exit work in the new one (4.1.10). */
    priv->link.in_u3 = false;
    priv->link.u3_active_eps = 0U;

    /*
     * Clear pending too. The buffers are cancelled here, and an owed Clear Stall
     * or resume has no endpoint configuration left to act on.
     */
    for (int i = 0; i < cfg->num_in_eps; i++) {
        udc_dwc3_ep_state_reset(&cfg->ep_data_in[i]);
        cfg->ep_data_in[i].xfer.pending = UDC_DWC3_EP_PEND_NONE;
        if (i > 0) {
            /* The release parks the buffers, then they are returned. */
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
 * End every transfer and take EP0 back to its Setup stage. This is the common
 * first step of a USB reset and a disconnect (SPEC 3.30b):
 *
 *   4.1.2 Table 4-2 / 4.1.8 Table 4-7:
 *     "If a control transfer is still in progress, complete it and get the
 *     controller into the 'Setup a Control-Setup TRB / Start Transfer' state"
 *     "Issue a DEPENDXFER command for any active transfers (except for the
 *     default control endpoint 0)"
 *   4.1.2 only (the clear_stall argument):
 *     "Issue a DEPCSTALL (ClearStall) command for any endpoint in STALL mode
 *     prior to the USB Reset (excluding control endpoints)"
 *
 * Each non-control endpoint goes through udc_dwc3_ep_recover(). It ends any
 * transfer, returns the buffers as cancelled, and clears a stall when owed.
 */
static UDC_DWC3_COLD void udc_dwc3_end_all_transfers(const struct device *const dev,
                             const bool clear_stall)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);

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
 * Core setup
 *
 * The core soft reset and the register set-up that follows it.
 */

/*
 * Core soft reset (DCTL.CSFTRST), then set up the controller and event ring.
 */
static UDC_DWC3_COLD int udc_dwc3_on_soft_reset(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    const mm_reg_t                      base = DEVICE_MMIO_NAMED_GET(dev, base);
    uint32_t                            reg;

    /*
     * Reset the device controller.
     * TODO: confirm that DWC_USB3_EN_LPM_ERRATA == 1.
     */
    reg = UDC_DWC3_DCTL_CSFTRST;
    reg |= FIELD_PREP(UDC_DWC3_DCTL_LPM_NYET_THRES_MASK, 15);
    udc_dwc3_dctl_write(base, reg);

    /*
     * Wait for CSftRst to clear, with a bound so a stuck core cannot hang the
     * driver. First poll with no delay. The core normally clears it here.
     * Then sleep one tick per read. This code also runs at runtime, from
     * udc_dwc3_controller_recover() on the work queue with the mutex held.
     * A busy spin there would starve every thread, including the drain.
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

    /* The core was reset, so no earlier command can still complete. */
    udc_dwc3_drop_xfer_state(dev, "soft reset");

    /*
     * The DEPCMD registers read as undefined again and no command is active, so
     * the next command on each endpoint skips the pre-poll.
     */
    for (uint8_t i = 0; i < DEV_CFG(dev)->num_in_eps; i++) {
        DEV_CFG(dev)->ep_data_in[i].cmd.depcmd_last = UDC_DWC3_DEPCMD_NONE;
    }
    for (uint8_t i = 0; i < DEV_CFG(dev)->num_out_eps; i++) {
        DEV_CFG(dev)->ep_data_out[i].cmd.depcmd_last = UDC_DWC3_DEPCMD_NONE;
    }

    /* Log the bus configuration at reset. */
    LOG_INF("BUSCFG at reset: GSBUSCFG0=0x%08x GSBUSCFG1=0x%08x",
        sys_read32(base + UDC_DWC3_GSBUSCFG0),
        sys_read32(base + UDC_DWC3_GSBUSCFG1));

    /*
     * Log three registers that a core soft reset does not clear (the databook's
     * CSftRst exception list). The driver does not fully program them.
     */
    LOG_INF("POR unpinned: GUSB2PHYCFG=0x%08x GUSB3PIPECTL=0x%08x GTXTHRCFG=0x%08x",
        sys_read32(base + UDC_DWC3_GUSB2PHYCFG),
        sys_read32(base + UDC_DWC3_GUSB3PIPECTL),
        sys_read32(base + UDC_DWC3_GTXTHRCFG));

    /*
     * Log the build parameters behind the U3/P3 settings that the guide leaves
     * to the integrator:
     *   GHWPARAMS0[1:0]    mode: 0 device, 1 host, 2 DRD (for DRD, software
     *                      sets GUSB3PIPECTL.SuspendEnable after init)
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

    /*
     * Set the SoC bus configuration to a known value. The databook does not
     * define the power-on value, and two independent vendor trees use the same
     * GSBUSCFG0 for this controller.
     */
    sys_write32(UDC_DWC3_GSBUSCFG0_INCR16BRSTENA |
            UDC_DWC3_GSBUSCFG0_INCR8BRSTENA |
            UDC_DWC3_GSBUSCFG0_INCR4BRSTENA,
            base + UDC_DWC3_GSBUSCFG0);

    /* GSBUSCFG1 keeps its power-on value (PipeTransLimit is 3 on this part). */

    LOG_INF("BUSCFG programmed: GSBUSCFG0=0x%08x GSBUSCFG1=0x%08x",
        sys_read32(base + UDC_DWC3_GSBUSCFG0),
        sys_read32(base + UDC_DWC3_GSBUSCFG1));

    /*
     * Log GRXTHRCFG. When the workaround is built in, clear UsbRxPktCntSel to
     * disable multi-packet RX thresholding (databook 1.2.4 erratum).
     */
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

    /* GTXTHRCFG keeps its value. The erratum above affects RX only. */

    /* Read the chip identification */
    reg = sys_read32(base + UDC_DWC3_GCOREID);
    LOG_INF("event: coreid=0x%04lx rel=0x%04lx",
        FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg),
        FIELD_GET(UDC_DWC3_GCOREID_REL_MASK, reg));
    __ASSERT_NO_MSG(FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg) == 0x5533);

    /*
     * Clear GUSB2PHYCFG[15] (ULPIAutoRes). The PHY must not auto-resume in device
     * mode, and the reset value may be 1 (databook Table 4-1).
     */
    sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG,
               UDC_DWC3_GUSB2PHYCFG_ULPIAUTORES);

    /*
     * Clear SusPHY and EnblSlpM. The databook (GUSB2PHYCFG) says: "before issuing
     * any device endpoint command when operating in 2.0 speeds, disable this
     * bit". The reset value may be 1. The driver never sets them again, so
     * udc_dwc3_depcmd() does not check them per command.
     */
    reg = sys_read32(base + UDC_DWC3_GUSB2PHYCFG);
    if ((reg & (UDC_DWC3_GUSB2PHYCFG_SUSPHY | UDC_DWC3_GUSB2PHYCFG_ENBLSLPM)) != 0U) {
        LOG_WRN("GUSB2PHYCFG had SusPHY/EnblSlpM set (0x%08x): cleared", reg);
        sys_write32(reg & ~(UDC_DWC3_GUSB2PHYCFG_SUSPHY | UDC_DWC3_GUSB2PHYCFG_ENBLSLPM),
                base + UDC_DWC3_GUSB2PHYCFG);
    }
    /*
     * Wait for the register file again, as udc_dwc3_init() does. This function
     * issued its own CSftRst, and the FIFO map below is the first GHWPARAMS read
     * after it. Read too soon after CSftRst clears, GHWPARAMS can return
     * RAM1_DEPTH=0 and mdwidth=0, which means the core is still in reset.
     */
    if (!udc_dwc3_wait_regfile_ready(dev)) {
        LOG_ERR("the register file is not out of reset after CSftRst: "
            "abandoning the configuration");
        return -EIO;
    }

    /*
     * Log the FIFO map. The TX FIFOs (GTXFIFOSIZn) share RAM1, whose size is
     * fixed at synthesis (GHWPARAMS7.RAM1_DEPTH). RX FIFO 0 is in RAM2.
     */
    {
        const uint32_t hp7 = sys_read32(base + UDC_DWC3_GHWPARAMS7);
        const uint32_t ram1 = FIELD_GET(UDC_DWC3_GHWPARAMS7_RAM1_DEPTH_MASK, hp7);
        const uint32_t rx = sys_read32(base + UDC_DWC3_GRXFIFOSIZ(0));
        const uint32_t mdw = (sys_read32(base + UDC_DWC3_GHWPARAMS0) >> 8) & 0xFFU;
        uint32_t       used = 0;

        /*
         * The wait above should make this unreachable. If it happens, the core
         * is still in reset, so stop here as the wait does.
         */
        if (ram1 == 0U || mdw == 0U) {
            LOG_ERR("FIFOMAP REFUSED: core reports RAM1_DEPTH=%u "
                "mdwidth=%u - the register file is not out of reset",
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
         * Count TX only. RX0 is in RAM2, a separate memory, so it does not use
         * the TX budget.
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

    /* GRXFIFOSIZ keeps its value. */

    /* Set up the event buffer and start event reception. */
    memset((void *)cfg->evt_buf, 0, CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t));

    /* Reset the read pointer together with the buffer, so they stay in step. */
    priv->evt.next = 0;
    udc_dwc3_drain_reset(&priv->evt.drain);
    priv->evt.q_head = priv->evt.q_tail = 0U;
    /* Start both timestamps now, so they are always valid. */
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

    /* Enable the event count after GEVNTADR and GEVNTSIZ. It starts at 0. */
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

    /*
     * Choose which device events the controller reports. These stay off:
     *   - VNDRDEVTSTRCVED: the driver ignores it, and the databook advises
     *     against the feature.
     *   - HibernationReqEvtEn: the driver has no hibernation support, and this
     *     event requires software "to start the hibernation process" (3.3.2).
     * EvntOverflowEn, CmdCmpltEn and InactTimeoutRcvedEn do not exist in this
     * controller's DEVTEN.
     */
    reg = 0;
    reg |= UDC_DWC3_DEVTEN_ERRTICERREN;
    reg |= UDC_DWC3_DEVTEN_WKUPEVTEN;
    /*
     * Link state change, USB Reset and Connection Done are the vendor's
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
 * Events
 *
 * Handlers for device and endpoint events, and the dispatch that sends each
 * event to its handler.
 */

/*
 * USBRST handler: return the device to the default state.
 */
static UDC_DWC3_COLD void udc_dwc3_on_usb_reset(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    LOG_DBG("Going through DWC3 reset logic");

    /*
     * A bus reset starts a new configuration. Let the next endpoint enable
     * reassign the transfer-resource pool. Reset first_ep too, because the next
     * session may enable a different endpoint first.
     */
    priv->epcfg.pool_assigned = false;
    priv->epcfg.first_ep = 0U;

    /* End all transfers and clear stalls (SPEC 3.30b 4.1.2). */
    udc_dwc3_end_all_transfers(dev, true);

    /* "Set DevAddr to 0". */
    udc_dwc3_set_address(dev, 0);
}

/*
 * CONNECTDONE handler: adopt the negotiated speed and resize EP0. Returns false
 * if the reset event did not fit in the stack's queue.
 */
static UDC_DWC3_COLD bool udc_dwc3_on_connect_done(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    const mm_reg_t                      base = DEVICE_MMIO_NAMED_GET(dev, base);
    int                                 mps = 0;

    /* EP0 max packet size for the connection speed. */
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

    /* Reconfigure the control endpoints with the new size. */
    udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT)->mps = mps;
    udc_get_ep_cfg(dev, USB_CONTROL_EP_IN)->mps = mps;
    udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0], true);
    udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_out[0], true);

    /* GTXFIFOSIZn keeps its value. The databook says this is the normal case. */

    /*
     * Report the reset here, not at USB_RESET. The speed registers are valid
     * only after CONNECT_DONE.
     */
    return udc_submit_event(dev, UDC_EVT_RESET, 0) == 0;
}

/*
 * Start a new transfer-resource pool (Table 4-5, SetConfiguration).
 * udc_dwc3_ep_enable() runs this once per bus reset, when the first non-control
 * endpoint is enabled. It recovers every non-control endpoint, re-initialises
 * the TX FIFO allocation with DEPCFG Modify on physical EP1, and reassigns the
 * non-control transfer resources (DEPSTARTCFG).
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
     * DEPSTARTCFG would reassign the resources of any transfer the recovery
     * left open (an End still running, a refused End, or a Start for an armed
     * ring). That command's outcome would then be lost. So skip it while any
     * endpoint is busy. The pool keeps its current assignment, which never
     * over-allocates (INVARIANT 4), and the next bus reset tries again.
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
 * Release the half whose stage just completed. This matches the controller,
 * which freed the transfer resource at XferComplete (3.2.2.2). It keeps the
 * rule udc_dwc3_ctrl_ep_recover() relies on: on EP0/EP1, a valid xferrscidx
 * means a transfer is live. A half with an End Transfer outstanding is left
 * for that End's completion.
 */
static void udc_dwc3_ctrl_release(struct udc_dwc3_ep_data *const h)
{
    if (!udc_dwc3_ep_is_ending(h)) {
        udc_dwc3_ep_state_reset(h);
    }
}

/* 4.4.1/4.4.2 step 2. The SETUP has arrived in the driver's buffer. */
static void udc_dwc3_ctrl_setup_done(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);

    memcpy(&priv->ctrl.setup, cfg->setup_buf, sizeof(priv->ctrl.setup));
    udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_SETUP_DONE);
    priv->diag.ctrl_setup_done++;

    /*
     * Write the SET_ADDRESS value to DCFG at the SETUP, not at the status stage.
     * The controller applies the new address itself after the status stage.
     */
    if (priv->ctrl.setup.bmRequestType == USB_REQTYPE_TYPE_STANDARD &&
        priv->ctrl.setup.bRequest == USB_SREQ_SET_ADDRESS) {
        udc_dwc3_set_address(dev, sys_le16_to_cpu(priv->ctrl.setup.wValue));
    }

    /*
     * Hand the 8 bytes to the stack. This returns any stage buffers still queued
     * from an earlier transfer. It copies the SETUP into the stack's SETUP
     * buffer, or keeps it until the stack queues one.
     */
    udc_setup_received(dev, &priv->ctrl.setup);
    udc_dwc3_ctrl_next(dev);
}

/* 4.4.2 step 4. The data stage has completed on its endpoint. */
static void udc_dwc3_ctrl_data_done(const struct device *const dev,
                    struct udc_dwc3_ep_data *const h,
                    const uint32_t sts, const uint32_t residual)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct net_buf *const       buf = udc_buf_get(&h->cfg);


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
     * SetupPending means "this control transfer was aborted on the USB bus and
     * the host did not complete the data stage." The flow still goes on to
     * XferNotReady(Status) (Figure 4-2), where the failed data stage gets
     * Set Stall (step 6).
     */
    if (sts == UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING) {
        priv->diag.ctrl_setup_pending++;
        priv->ctrl.setup_pending = true;
        /*
         * Table 4-14 step 8. If a new SETUP arrives before the IN data goes out,
         * the controller skips the data TRBs. Software reclaims them (HWO=1) and
         * flushes the TxFIFO. XferComplete released the transfer resource, so
         * both TRBs belong to the driver again.
         */
        if (USB_EP_DIR_IS_IN(h->cfg.addr)) {
            udc_dwc3_ctrl_reclaim_half(dev, h);
        }
        udc_dwc3_buf_return(dev, buf, UDC_DWC3_BUF_ABANDONED);
        udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_DATA_DONE);
        return;
    }

    if (!USB_EP_DIR_IS_IN(h->cfg.addr)) {
        /* Use the length actually received, not what the host declared. */
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
 * Update the DCTL U1/U2 bits after a request's status stage completes normally.
 * Only SuperSpeed standard device requests count.
 *   - AcceptU1/U2Ena: set on SetConfiguration(non-zero). SetConfiguration(0)
 *     clears all four bits.
 *   - InitU1/U2Ena: set on SetFeature(U1/U2_ENABLE), cleared on
 *     ClearFeature(U1/U2_ENABLE).
 * Hardware clears all four on USB reset. The disconnect handler clears them too.
 */
static UDC_DWC3_COLD void udc_dwc3_ctrl_apply_link_pm(const struct device *const dev)
{
    struct udc_dwc3_data *const          priv = udc_get_private(dev);
    const mm_reg_t                       base = DEVICE_MMIO_NAMED_GET(dev, base);
    const struct usb_setup_packet *const setup = &priv->ctrl.setup;
    const uint16_t                       value = sys_le16_to_cpu(setup->wValue);
    const bool                           ss = udc_dwc3_connect_speed(base) == UDC_BUS_SPEED_SS;
    uint32_t                             bit;

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
 * 4.4.1 step 5 / 4.4.2 step 8. The status stage has completed, so go back to
 * Step 1. The status buffer is returned as completed even with SetupPending,
 * as Linux dwc3 does. The request was served, and only the host's ACK is in
 * doubt.
 */
static void udc_dwc3_ctrl_status_done(const struct device *const dev,
                      struct udc_dwc3_ep_data *const h,
                      const uint32_t sts)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct net_buf *const       buf = udc_buf_get(&h->cfg);

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
 * XferComplete on EP0 or EP1. The stage is taken from where Figure 4-2 says the
 * transfer is, not from what the driver remembers arming:
 *   Setup node  -> the SETUP.
 *   Data node   -> the data stage, on the endpoint bmRequestType names.
 *   Status node -> the status stage.
 */
static void udc_dwc3_on_ctrl(const struct device *const dev, struct udc_dwc3_ep_data *const h,
                 const bool is_in)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const bool                  ending = udc_dwc3_ep_is_ending(h);
    struct udc_dwc3_trb         t;
    uint32_t                    status;

    if (priv->diag.watchdog_ep == h) {
        priv->diag.watchdog_ep = NULL;
        priv->diag.watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
    }

    /* Nothing live on this half. This is a late or repeated event. */
    if (h->xfer.state == UDC_DWC3_EP_IDLE) {
        LOG_DBG("EP%02x XferComplete with no transfer live: ignored", h->cfg.addr);
        return;
    }

    /*
     * Read the TRB write-back before anything can rearm this half's ring. Read
     * ctrl before status. If HWO is still set at XferComplete, the write-back is
     * not visible yet, and the status is used as read. If a chained ZLP TRB is
     * still owned, the first TRB's TRBSTS (OK here) is kept. Its programmed
     * status of 0 would read as OK too.
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

    /* A recovery owns this transfer. Let it continue. */
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
     * A transfer was live on this half, but not the stage Figure 4-2 expects.
     * The driver and controller disagree on where the control transfer is,
     * so go back to Step 1.
     */
    LOG_WRN_RATELIMIT("EP%02x XferComplete (TRB status 0x%08x) in control state %u: "
              "back to Setup", h->cfg.addr, status, (unsigned int)priv->ctrl.state);
    udc_dwc3_ctrl_ep_recover(dev);
}

/*
 * Wait, bounded, for DGCMD.CmdAct to clear. Returns false if it is still set.
 * SPEC 3.30b DGCMD bit 10 CMDACT: software sets it to start a generic command,
 * and the controller clears it when the command has executed.
 */
static UDC_DWC3_COLD bool udc_dwc3_dgcmd_wait_idle(const mm_reg_t base)
{
    uint32_t polls = 0;

    /*
     * Poll with no delay. Callers run on the drain thread or the UDC work queue,
     * often with the mutex held, so a sleeping poll would stall other UDC paths.
     * A generic command takes microseconds. One still active after
     * UDC_DWC3_DGCMD_POLL_MAX reads is reported, not waited on.
     */
    while ((sys_read32(base + UDC_DWC3_DGCMD) & UDC_DWC3_DGCMD_ACT) != 0U) {
        if (++polls >= UDC_DWC3_DGCMD_POLL_MAX) {
            return false;
        }
    }

    return true;
}

/*
 * Issue one device generic command (SPEC 3.30b 3.2.1, Table 3-2). This is the
 * only place that writes DGCMDPAR and DGCMD.
 *
 * Both the drain thread and the work queue issue generic commands, so the
 * parameter and command are written together under dgcmd_lock. A new command
 * is never written over one still running (CmdAct set). The function waits
 * outside the lock, then checks again under it. With wait set, it also polls
 * CmdAct after issuing.
 *
 * Returns 0, -EBUSY (previous command still running, nothing issued), or
 * -ETIMEDOUT (issued, but still running after the bounded wait).
 */
static UDC_DWC3_COLD int udc_dwc3_dgcmd(const struct device *const dev, const uint32_t cmd,
                    const uint32_t param, const bool wait)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t              base = DEVICE_MMIO_NAMED_GET(dev, base);
    k_spinlock_key_t            key;

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
 * Needed after an aborted control IN transfer (4.4.2 step 8: on a SETUP
 * mid-transfer, "reclaim the TRBs with HWO=1 ... and flush the TxFIFO").
 * Without the flush, bytes already staged for the skipped IN stage would go
 * out at the start of the next one.
 */
static UDC_DWC3_COLD void udc_dwc3_fifo_flush_tx(const struct device *const dev, const uint8_t fifo)
{
    /*
     * Wait for the flush to finish. The next control stage is armed right after
     * this returns.
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
 * True if the endpoint has an "active transfer" in the 4.1.10 sense. That means
 * it holds a transfer resource (RUNNING, or a Start whose outcome is open).
 * On EP0 only a data or status stage counts. A Setup TRB does not make it
 * active (4.9.1), and a SETUP is never flow-controlled, so it needs no ERDY.
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

    /* EP0 counts only in a data or status stage, and not during a recovery. */
    return epn >= 2U ||
           (priv->ctrl.state != UDC_DWC3_CTRL_IDLE && !udc_dwc3_ctrl_recovering(priv));
}

/*
 * Link and bus events. This is the only place the driver reacts to link state
 * (SPEC 3.30b):
 *
 *   USB Reset        4.1.2  back to the default state, DevAddr 0
 *   Connect Done     4.1.3  adopt the speed, resize EP0
 *   Disconnect       4.1.7  clear the U1/U2 enables, set DCTL[8:5] to 5
 *                           (Rx.Detect)
 *   Link State Chg   3.3.2  SS only. Not raised on exit from HOT_RESET or POLL,
 *                           or on entry to RECOVERY, so a U0 event also ends
 *                           every Recovery.
 *   Erratic error    3.3.2  SS needs a controller reset. HS/FS needs a soft
 *                           disconnect. Both go through
 *                           udc_dwc3_controller_recover().
 *   Buffer overflow  3.3.2  device events after it may be lost. Endpoint
 *                           events are not.
 *   Wakeup, Suspend         nothing to do (no remote wakeup, no hibernation)
 *
 * It also handles the exit from U3 (SPEC 3.30b 4.1.10).
 */
static UDC_DWC3_COLD bool udc_dwc3_link_event(const struct device *const dev, const uint32_t evt)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    const mm_reg_t                      base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t                      type = evt & UDC_DWC3_EVT_MASK;
    const uint32_t                      link = FIELD_GET(UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK, evt);
    const bool                          ss_link = type == UDC_DWC3_DEVT_ULSTCHNG &&
                 (evt & UDC_DWC3_DEVT_EVTINFO_SS) != 0U;
    uint32_t                            owed = 0U;

    /*
     * Record U3 entry and exit for SPEC 3.30b 4.1.10. This is done only here.
     * A U3 exit is an SS U3 event followed by U0. Recovery in between is not
     * reported.
     *   - SS U3: record the endpoints active now. A repeated U3 does not
     *     re-record.
     *   - SS U0 after a recorded U3: those endpoints are owed Set Endpoint NRDY.
     *   - Anything else cancels a pending exit. This covers non-SS events
     *     (On/Sleep/Suspend share the encoding but are never U3), other link
     *     states, USB reset, disconnect and event overflow (after an overflow,
     *     a later U0 may not be this U3's exit).
     * A U0 without a recorded U3 (Recovery, U1/U2 exit) never gets the command.
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
        return udc_dwc3_on_connect_done(dev);
    case UDC_DWC3_DEVT_DISCONNEVT:
        /* End every transfer and take EP0 back to its Setup stage (4.1.7). */
        udc_dwc3_end_all_transfers(dev, true);
        /*
         * The device is no longer configured, so clear the U1/U2 enables. A new
         * link must not enter or accept U1/U2 until the host enables them again.
         */
        udc_dwc3_dctl_update(base, UDC_DWC3_DCTL_ACCEPTU1ENA | UDC_DWC3_DCTL_INITU1ENA |
                       UDC_DWC3_DCTL_ACCEPTU2ENA | UDC_DWC3_DCTL_INITU2ENA, 0U);
        udc_dwc3_dctl_link_request(base, UDC_DWC3_DCTL_ULSTCHNGREQ_RXDETECT);
        break;
    case UDC_DWC3_DEVT_ULSTCHNG:
        /*
         * SPEC 3.30b 4.1.10, Initialization after U3 Exit. On U3 -> U0, issue
         * "Set Endpoint NRDY" on every endpoint that was active at U3 entry and
         * is still active now, so that it sends ERDY. An ERDY sent just as the
         * host's LGO_U3 arrived is lost on the host. The device keeps no record
         * of it, so the transfer would wait forever. This command is for U3 exit
         * only.
         */
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
         * UTMI+: phy_rxvalid or phy_rxactive was asserted for 2 ms or more.
         * SS: the PIPE did not answer a PHY command. The log is rate-limited
         * because the event repeats until the reset. The reset sleeps, so the
         * heartbeat runs it.
         */
        LOG_ERR_RATELIMIT("DEVT_ERRTICERR: PHY erratic error - resetting "
            "the controller");
        if (udc_submit_event(dev, UDC_EVT_ERROR, -EIO) != 0) {
            return false;
        }
        if (priv->run.state == UDC_DWC3_RUN) {
            /* If a recovery is already running, it provides the reset. */
            priv->run.state = UDC_DWC3_RUN_RESET_OWED;
        }
        break;
    case UDC_DWC3_DEVT_EVNTOVERFLOW:
        /* The only log line for this event. The generic event log skips it. */
        LOG_ERR_RATELIMIT("evt ring ovfl");
        break;
    default:
        /* WKUPEVT, SUSPEND: nothing to do. */
        break;
    }

    return true;
}

/*
 * XferNotReady on either half of EP0. The host is asking for a control stage.
 * This follows SPEC 3.30b 4.4.1/4.4.2 (Figure 4-2).
 *   - An expected event advances the state.
 *   - An event that does not fit the current state is ignored. The hardware
 *     discards data and status stages that come without a SETUP.
 *   - A databook error case goes back to Step 1 via udc_dwc3_ctrl_ep_recover().
 */
static void udc_dwc3_on_ctrl_xnr(const struct device *const dev, const uint32_t evt,
                 const bool is_in)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    const uint32_t                      stage = evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK;
    struct udc_dwc3_ep_data *const      out0 = &cfg->ep_data_out[0];


    if (udc_dwc3_ctrl_recovering(priv) ||
        (stage != UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA &&
         stage != UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS)) {
        return;
    }

    /*
     * Step 2: a data or status request before the SETUP completed. Set Stall.
     * The Setup TRB is already armed, so the state stays at Setup.
     * "Before the SETUP completed" means the Setup TRB still has HWO set.
     * If that TRB has already retired, its XferComplete is still on the way.
     * A stall then would hit the new request, so nothing is done.
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
         * Step 6: the data stage failed, so Set Stall and go back to Step 1.
         * A data stage the host abandoned (SetupPending) counts as failed.
         * So does a status request in the wrong direction.
         */
        if (priv->ctrl.setup_pending || is_in != udc_dwc3_ctrl_status_is_in(priv)) {
            udc_dwc3_ctrl_ep_recover(dev);
            return;
        }

        /* Steps 4 and 7: the status stage is due. Arm it when the buffer is here. */
        udc_dwc3_ctrl_state_set(dev, UDC_DWC3_CTRL_STATUS_READY);
        udc_dwc3_ctrl_next(dev);
        return;
    }

    /* XferNotReady(Data). */
    switch (priv->ctrl.state) {
    case UDC_DWC3_CTRL_SETUP_DONE:
        /*
         * A data request on a request with no data stage is an error (4.4.1
         * step 3). So is a data request in the wrong direction (4.4.2 step 3a).
         * Otherwise the host is just early. The stack's buffer arms the stage.
         */
        if (!udc_dwc3_ctrl_three_stage(priv) || is_in != udc_dwc3_ctrl_dir_in(priv)) {
            udc_dwc3_ctrl_ep_recover(dev);
        }
        return;
    case UDC_DWC3_CTRL_DATA_ARMED:
        /* 3a: wrong direction, so recover. 3b: right direction, so ignore. */
        if (is_in != udc_dwc3_ctrl_dir_in(priv)) {
            udc_dwc3_ctrl_ep_recover(dev);
        }
        return;
    case UDC_DWC3_CTRL_DATA_DONE:
        /*
         * Step 5b: the host sent more data than wLength.
         * Step 5a (a closing ZLP) does not come here. The OUT data TRB is
         * rounded up to wMaxPacketSize in udc_dwc3_trb_ctrl_out(), so the ZLP
         * lands in it.
         */
        udc_dwc3_ctrl_ep_recover(dev);
        return;
    default:
        return;
    }
}

/* Check the completion status of a retired TRB and log any error. */
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
    struct net_buf             *buf;
    uint32_t                    drained = 0U;
    int                         ret;

    while (true) {
        struct udc_dwc3_trb trb;

        /* Stop at a TRB the controller still owns, or at an empty slot. */
        ret = udc_dwc3_pop_trb(ep_data, &buf, &trb);
        if (ret != 0) {
            break;
        }

        LOG_DBG("XFER_DONE_NORM: EP%02x, data %p",
            ep_data->cfg.addr, (void *)buf->data);

        udc_dwc3_on_xfer_done(&trb);

        priv->diag.nonctrl_done++;
        ep_data->diag.n_retire++;


        /* A failed post (usbd queue full) is only counted. */
        if (udc_dwc3_buf_return(dev, buf, UDC_DWC3_BUF_DONE) != 0) {
            priv->diag.post_fail_total++;
        }

        drained++;
    }

    /* Ring slots are free, so let the endpoint work queue more buffers. */
    if (drained > 0U) {
        k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
    }

    return drained;
}

/* Transfer completion on a non-control endpoint. */
static void udc_dwc3_on_xfer_done_nonctrl(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data,
                      const bool complete)
{
    (void)udc_dwc3_drain_completed(dev, ep_data);

    /*
     * Only XferComplete means the controller released the transfer resource.
     * XferInProgress means the transfer goes on (databook 3.2.2.2).
     */
    if (complete) {
        /*
         * Go to idle only from RUNNING with nothing still armed.
         * A late event can belong to a transfer that ended long ago. The event
         * word has no sequence number to tell it from a fresh one.
         * With a Start or End open, that command's completion settles the
         * endpoint. Going IDLE here would let a Start follow an End that is
         * still executing (3.2.2.7).
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

/* Name of a link state, for logging. */
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

/* Name of an event word, for logging. */
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
         * The link state comes from the event, because DSTS may have moved on.
         * The speed comes from DSTS. It does not change within a session.
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
 * Move every armed buffer on an endpoint to its requeue FIFO and empty the
 * TRB ring.
 */
static void udc_dwc3_ep_ring_release(struct udc_dwc3_ep_data *const ep_data)
{
    /*
     * The last TRB is the LINK TRB (udc_dwc3_trb_nonctrl_init()). It stays
     * armed to keep the ring closed. Only the data slots are cleared.
     */
    const int       slots = CONFIG_UDC_DWC3_TRB_NUM - 1;
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

    /* Clear the data TRBs and the buffer slots. */
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
 * Continue the recovery that was waiting for it.
 *
 * Called on the EpCmdCmplt event. Also called by udc_dwc3_ep_resolve_cmd()
 * when DEPCMD shows the End completed but its event never arrived.
 */
static UDC_DWC3_COLD void udc_dwc3_ep_end_completed(const struct device *const dev,
                      struct udc_dwc3_ep_data *const ep_data)
{
    /* The End completed, so the endpoint holds no transfer. Return it to IDLE. */
    udc_dwc3_ep_state_reset(ep_data);

    LOG_DBG("EpCmdCmplt: DMA stopped for EP%02x", ep_data->cfg.addr);

    if (USB_EP_GET_IDX(ep_data->cfg.addr) != 0U) {
        udc_dwc3_ep_recover(dev, ep_data);
        return;
    }

    /* EP0: only the control recovery ends an EP0 transfer, so continue it. */
    udc_dwc3_ctrl_recover_continue(dev);
}

/* Endpoint Command Complete. */
static void udc_dwc3_on_ep_cmd_cmplt(const struct device *const dev,
                     struct udc_dwc3_ep_data *const ep_data, const uint32_t evt)
{
    /*
     * A Start Transfer completion. It carries the resource index that later
     * Update and End commands need, or it reports the Start was refused.
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
             * The event does not say which Start it belongs to. A late event
             * from an earlier Start can arrive while this one is open.
             * So DEPCMD (or cmd_record, if DEPCMD was overwritten) decides.
             * The event decides only when neither holds this Start.
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
             * No Start is open, so this is an earlier Start's refusal. It was
             * handled when that Start was settled: by the post-poll, the next
             * pre-poll or udc_dwc3_ep_resolve_cmd(). Acting on it again would
             * tear down the transfer that runs now.
             */
            LOG_WRN_RATELIMIT("EP%02x Start Transfer refusal 0x%08x while the "
                      "endpoint is %s: an earlier Start's, discarded",
                      ep_data->cfg.addr, evt,
                      udc_dwc3_ep_state_name(ep_data->xfer.state));
            return;
        } else {
            udc_dwc3_adopt_xferrscidx_evt(dev, ep_data, idx);
        }

        /* Buffers armed while the Start was open need an Update now. */
        udc_dwc3_ep_update_owed(dev, ep_data);

        /* Continue any recovery that was waiting for this Start. */
        if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U) {
            udc_dwc3_ctrl_recover_continue(dev);
        } else if (udc_dwc3_ep_recovery_owed(dev, ep_data)) {
            udc_dwc3_ep_recover(dev, ep_data);
        }
        return;
    }

    if (!udc_dwc3_ep_is_ending(ep_data)) {
        /*
         * A stale event, so discard it. CMDIOC is set only on Start and End
         * Transfer, so this is an End completion on an endpoint that is not
         * ending. It belongs to an earlier use of the endpoint, before it was
         * disabled and enabled again (e.g. an alt-setting switch).
         *
         * Handling it would reset the new transfer to IDLE and lose its
         * resource index. Then Update Transfer would be refused.
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
 * Log a Link State Change event.
 *
 * A new state always prints. A repeated state prints once every
 * UDC_DWC3_EVT_LINK_LOG_EVERY times, with the repeat count. The raw event
 * word is logged so the decode can be checked against the databook.
 */
static UDC_DWC3_COLD void udc_dwc3_log_link_event(const struct device *const dev,
                    const uint32_t evt, const uint32_t dsts)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const uint32_t              link = FIELD_GET(UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK, evt);

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

/* Log and skip an event that has no handler. */
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
 * XferNotReady on a non-control, non-video endpoint. The host asked for data
 * and the controller answered NRDY. DEPCFG enables this event on no other
 * non-control endpoint.
 *
 * With nothing armed, this is normal and is ignored. With TRBs armed, the
 * controller and the driver disagree about the ring. The Transfer Active bit
 * says how.
 *   active = 1  A transfer runs, but the controller found no TRB it owns.
 *               Update Transfer makes it fetch the armed TRBs again. This
 *               needs a RUNNING transfer's resource index.
 *   active = 0  No transfer runs, so recover the endpoint.
 *               udc_dwc3_ep_recover() ends the transfer the driver still
 *               has, or else retires finished TRBs and starts a new transfer.
 * If a command is still open on the endpoint, wait. The host retries, so the
 * event comes again.
 */
static void udc_dwc3_on_xfer_not_ready_nonctrl(const struct device *const dev,
                           struct udc_dwc3_ep_data *const ep_data,
                           const uint32_t evt)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const bool                  active = (evt & UDC_DWC3_DEPEVT_STATUS_XFER_ACTIVE) != 0U;

    if (!udc_dwc3_ep_ring_outstanding(ep_data) || udc_dwc3_ep_cmd_busy(ep_data)) {
        return;
    }

    priv->diag.xnrdy_acted++;
    LOG_WRN_RATELIMIT("EP%02x XferNotReady with TRBs armed: active %u, %s, head %u "
              "tail %u (%u total)", ep_data->cfg.addr, active ? 1U : 0U,
              udc_dwc3_ep_state_name(ep_data->xfer.state), ep_data->ring.head,
              ep_data->ring.tail, priv->diag.xnrdy_acted);

    if (active) {
        if (ep_data->xfer.state == UDC_DWC3_EP_RUNNING) {
            (void)udc_dwc3_depcmd_update_xfer(dev, ep_data);
        }
        return;
    }

    udc_dwc3_ep_recover(dev, ep_data);
}

/*
 * Endpoint event (DEPEVT, bit 0 = 0). Bits 5:1 are the physical endpoint and
 * bits 9:6 the event kind. Both are decoded here once, and the handlers get
 * the results.
 *
 * Endpoint events are not logged here. Every healthy transfer raises them,
 * and a log line costs a synchronous UART write in this thread.
 *
 * Physical endpoints 0 and 1 are the two halves of EP0 and go to the control
 * state machine. The others go to the ring handlers.
 */
static void udc_dwc3_dispatch_ep_event(const struct device *const dev, const uint32_t evt)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    const uint32_t                      epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
    const uint32_t                      kind = FIELD_GET(UDC_DWC3_DEPEVT_KIND_MASK, evt);
    struct udc_dwc3_ep_data            *ep_data;

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

    /* Command completion: one handler for every endpoint, including EP0. */
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
        case UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_XFERNOTREADY(0)):
            udc_dwc3_on_ctrl_xnr(dev, evt, is_in);
            break;
        /* EP0 TRBs never raise XferInProgress. */
        default:
            udc_dwc3_evt_unknown(dev, evt);
            break;
        }
        return;
    }

    switch (kind) {
    /*
     * Both completion events retire TRBs. Which one the controller raises
     * depends on the TRB control bits (Table 4-8).
     */
    case UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_XFERCOMPLETE(0)):
        udc_dwc3_on_xfer_done_nonctrl(dev, ep_data, true);
        break;
    case UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_XFERINPROGRESS(0)):
        udc_dwc3_on_xfer_done_nonctrl(dev, ep_data, false);
        break;
    case UDC_DWC3_DEPEVT_KIND(UDC_DWC3_DEPEVT_XFERNOTREADY(0)):
        udc_dwc3_on_xfer_not_ready_nonctrl(dev, ep_data, evt);
        break;
    default:
        udc_dwc3_evt_unknown(dev, evt);
        break;
    }
}

/*
 * Device event (bit 0 = 1). These are rare, so they are logged here.
 * Returns false if a reset or error event did not fit in the stack's queue.
 */
static UDC_DWC3_COLD bool udc_dwc3_dispatch_dev_event(const struct device *const dev,
                              const uint32_t evt)
{
    const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t evt_type = evt & UDC_DWC3_EVT_MASK;

    /*
     * Link changes have their own rate-limited log. Overflow and generic
     * command completions are not logged, because they can arrive faster than
     * the console prints. DSTS is read only when a line is printed.
     */
    if (evt_type == UDC_DWC3_DEVT_ULSTCHNG) {
        udc_dwc3_log_link_event(dev, evt, sys_read32(base + UDC_DWC3_DSTS));
    } else if (evt_type != UDC_DWC3_DEVT_EVNTOVERFLOW &&
           evt_type != UDC_DWC3_DEVT_CMDCMPLT) {
        LOG_INF("%s", udc_dwc3_get_event_name(evt, sys_read32(base + UDC_DWC3_DSTS)));
    }

    switch (evt_type) {
    /* udc_dwc3_link_event() handles all link and bus events. */
    case UDC_DWC3_DEVT_USBRST:
    case UDC_DWC3_DEVT_CONNECTDONE:
    case UDC_DWC3_DEVT_DISCONNEVT:
    case UDC_DWC3_DEVT_ULSTCHNG:
    case UDC_DWC3_DEVT_WKUPEVT:
    case UDC_DWC3_DEVT_SUSPEND:
    case UDC_DWC3_DEVT_ERRTICERR:
    case UDC_DWC3_DEVT_EVNTOVERFLOW:
        return udc_dwc3_link_event(dev, evt);
    case UDC_DWC3_DEVT_SOF:
    case UDC_DWC3_DEVT_CMDCMPLT:
    case UDC_DWC3_DEVT_VNDRDEVTSTRCVED:
        break;
    default:
        udc_dwc3_evt_unknown(dev, evt);
        break;
    }

    return true;
}

/*
 * Dispatch one event word. Bit 0 tells endpoint events from device events.
 * The caller holds the UDC mutex. It clears priv->diag.dispatch_evt when it
 * has finished dispatching.
 *
 * Returns false only if a reset or error event did not fit in the stack's
 * queue. The event is then dispatched again. An endpoint event always
 * returns true.
 */
static bool udc_dwc3_dispatch_event(const struct device *const dev, const uint32_t evt)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    /* Lets udc_dwc3_heartbeat_worker() see a dispatch that gets stuck. */
    priv->diag.dispatch_evt = evt;

    if ((evt & BIT(0)) == 0U) {
        udc_dwc3_dispatch_ep_event(dev, evt);
        return true;
    }

    return udc_dwc3_dispatch_dev_event(dev, evt);
}

/* Dispatch one event word outside a drain pass, taking the UDC mutex. */
static __maybe_unused void udc_dwc3_handle_event(const struct device *const dev,
                           const uint32_t evt)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    priv->diag.dispatch_t0 = k_cycle_get_32();
    udc_lock_internal(dev, K_FOREVER);
    (void)udc_dwc3_dispatch_event(dev, evt);
    priv->diag.dispatch_evt = 0U;
    udc_unlock_internal(dev);
}

#ifdef CONFIG_UDC_DWC3_SHELL
/* Used by the "dwc3 events" shell command. */
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
 * Heartbeat and controller recovery
 *
 * A periodic heartbeat checks the event ring and the endpoints. It runs the
 * controller recovery when the controller needs a reset.
 */

/*
 * Check that the controller is ready for RunStop=0 (SPEC 4.1.8 Table 4-7).
 * EP0 must be idle with no End Transfer on either half. Every other endpoint
 * must be idle.
 * Returns -1 when ready. Otherwise returns the first busy physical endpoint
 * (0 for either EP0 half).
 */
static UDC_DWC3_COLD int udc_dwc3_stop_blocker(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    const struct udc_dwc3_data *const   priv = udc_get_private(dev);

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
 * Heartbeat timer callback. It kicks the drain and submits the heartbeat
 * worker.
 *
 * This runs in ISR context, so it does only cheap work. Locking, logging and
 * recovery are done in udc_dwc3_heartbeat_worker(). Logging is kept out
 * because LOG_MODE_MINIMAL busy-waits on the UART.
 */
static UDC_DWC3_COLD void udc_dwc3_heartbeat_expiry(struct k_timer *const timer)
{
    struct udc_dwc3_data *const priv =
        CONTAINER_OF(timer, struct udc_dwc3_data, heartbeat_timer);
    const struct device *const  dev = priv->dev;

    udc_dwc3_drain_helper(dev);

    priv->diag.hb_expiries++;

    /* Submit time. The worker uses it to measure the queue delay. */
    if (priv->diag.hb_submit_t == 0U) {
        priv->diag.hb_submit_t = k_cycle_get_32();
    }

    /* 0 means the previous beat has not run yet, so this one is dropped. */
    if (k_work_submit_to_queue(udc_get_work_q(),
                   &priv->heartbeat_work) == 0) {
        priv->diag.hb_coalesced++;
    }
}


/*
 * Returns true if the empty slot at evt.next is surely lost, not just late.
 * It is lost if a later slot is already written, because the controller
 * fills the ring in order.
 */
static UDC_DWC3_COLD bool udc_dwc3_evt_lookahead_lost(const struct device *const dev,
                    const uint32_t gc)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    uint32_t                            owed = gc / sizeof(uint32_t);

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

/* Skips a dead slot. udc_dwc3_drain_slot_is_dead() decides when. */
static uint32_t udc_dwc3_evt_skip_dead_slot(const struct device *const dev,
                    const uint32_t gc, const bool frozen,
                    const uint32_t gaveup_ms);


/*
 * Settle an open Start or End command by reading its outcome from DEPCMD.
 * If it is still executing after UDC_DWC3_CMD_UNKNOWN_MS, mark the endpoint
 * START_UNKNOWN or END_UNKNOWN.
 */
static UDC_DWC3_COLD void udc_dwc3_ep_resolve_cmd(const struct device *const dev,
                    struct udc_dwc3_ep_data *const ep_data)
{
    const bool                starting = (ep_data->xfer.state == UDC_DWC3_EP_STARTING) ||
                  (ep_data->xfer.state == UDC_DWC3_EP_START_UNKNOWN);
    const bool                ending = (ep_data->xfer.state == UDC_DWC3_EP_ENDING) ||
                (ep_data->xfer.state == UDC_DWC3_EP_END_UNKNOWN);
    uint32_t                  reg = 0U;
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
         * DEPCMD was overwritten and no record holds our command, so its
         * outcome is unknown. This is not expected, because udc_dwc3_depcmd()
         * keeps a record before it overwrites DEPCMD.
         * Treat the command as still executing. Its Command Complete event
         * can still settle it. After the deadline the endpoint goes UNKNOWN.
         */
        __fallthrough;

    case UDC_DWC3_CMD_UNKNOWN:
        /*
         * The command is still executing. After the deadline, mark the
         * endpoint UNKNOWN. An UNKNOWN endpoint gets no further Start or End,
         * so this command's outcome stays readable in DEPCMD or cmd_record.
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
             * The End was refused, so the transfer still runs and owns
             * its resource. Restore its resource index.
             */
            LOG_WRN("EP%02x End Transfer had failed unobserved "
                "(0x%08x); the transfer is still running",
                ep_data->cfg.addr, reg);
            udc_dwc3_ep_end_refused(dev, ep_data);
            return;
        }
        /* The Start was refused and its event was lost. Handle it now. */
        udc_dwc3_ep_start_refused(dev, ep_data, "DEPCMD", reg, false);
        return;

    case UDC_DWC3_CMD_OK:
    default:
        if (ending) {
            /*
             * DEPCMD (or cmd_record) shows our End Transfer completed
             * successfully, so the resource is free. The command type
             * was checked, so this result is our End's.
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
 * Per-endpoint sweep, run once per heartbeat.
 *
 * Events can be lost on this part, so the sweep reads state that a lost event
 * cannot hide:
 *
 *   what a command did          -> DEPCMD: CmdAct, CmdStatus, XferRscIdx
 *   what a transfer did         -> the TRB: HWO, and BUFSIZ written back
 *
 * A host waiting for data is not detected here. That is the job of
 * XferNotReady (udc_dwc3_on_xfer_not_ready_nonctrl()). There is no inactivity
 * timeout either. An OUT endpoint with an armed TRB and no completions is
 * waiting for the host, and a timer cannot tell that from a fault.
 *
 * EP0 is not swept. It is a stage machine, not a ring, and
 * udc_dwc3_ctrl_ep_recover() handles it.
 */
static UDC_DWC3_COLD void udc_dwc3_ep_sweep(const struct device *const dev,
                  struct udc_dwc3_ep_data *const ep_data)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    if (!ep_data->cfg.stat.enabled && !udc_dwc3_ep_recovery_owed(dev, ep_data)) {
        return;
    }

    /* Rule 1: settle a command whose Command Complete never arrived. */
    udc_dwc3_ep_resolve_cmd(dev, ep_data);

    /* A command is still open (or UNKNOWN), so leave the endpoint alone. */
    if (udc_dwc3_ep_cmd_busy(ep_data)) {
        return;
    }

    /* Continue a recovery that still has work to do. */
    if (udc_dwc3_ep_recovery_owed(dev, ep_data)) {
        udc_dwc3_ep_recover(dev, ep_data);
        return;
    }

    /* A Start settled above may still owe an Update. */
    udc_dwc3_ep_update_owed(dev, ep_data);

    /*
     * Rule 2: retire every TRB the controller has finished. This reads the
     * TRBs in memory, so it covers a lost XferComplete.
     */
    {
        const uint32_t got = udc_dwc3_drain_completed(dev, ep_data);

        if (got > 0U) {
            priv->diag.evt_sweep_rescued += got;
            priv->diag.evt_sweep_runs++;
        }
    }

    /*
     * Armed TRBs on an IDLE endpoint mean a reset path cleared the state
     * while buffers stayed armed. Start the transfer for them.
     */
    if (ep_data->xfer.state == UDC_DWC3_EP_IDLE &&
        udc_dwc3_ep_ring_outstanding(ep_data)) {
        LOG_WRN("EP%02x has descriptors armed with no transfer started - "
            "starting it", ep_data->cfg.addr);
        (void)udc_dwc3_depcmd_start_xfer(dev, ep_data);
    }
}

/* Called only from udc_dwc3_heartbeat_worker(), with the UDC mutex held. */
static UDC_DWC3_COLD void udc_dwc3_recover_all(const struct device *const dev)
{
    const struct udc_dwc3_config *const cfg = dev->config;

    /*
     * EP0: only settle open commands from DEPCMD. The control stages are
     * driven by events. Without this, an EP0 half whose End Transfer event
     * was lost would stay ENDING.
     */
    udc_dwc3_ep_resolve_cmd(dev, &cfg->ep_data_out[0]);
    udc_dwc3_ep_resolve_cmd(dev, &cfg->ep_data_in[0]);

    /* Continue a control recovery that was waiting for one of them. */
    udc_dwc3_ctrl_recover_continue(dev);

    for (int i = 1; i < cfg->num_in_eps; i++) {
        udc_dwc3_ep_sweep(dev, &cfg->ep_data_in[i]);
    }
    for (int i = 1; i < cfg->num_out_eps; i++) {
        udc_dwc3_ep_sweep(dev, &cfg->ep_data_out[i]);
    }
}

static void udc_dwc3_ctrl_setup_wd_check(const struct device *const dev);

/*
 * Periodic health work: check the drain, run recovery, and report.
 *
 * A timer drives it, so it keeps running when events are lost. It runs on the
 * work queue, not the drain thread, because its log lines are slow to print.
 */
static void udc_dwc3_heartbeat_worker(struct k_work *work)
{

    struct udc_dwc3_data *const         priv =
        CONTAINER_OF(work, struct udc_dwc3_data, heartbeat_work);
    const struct device *const          dev = priv->dev;
    const struct udc_dwc3_config *const cfg = dev->config;
    const mm_reg_t                      base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t                      d_evt = priv->diag.dispatch_evt;
    /* GEVNTCOUNT as read by udc_dwc3_drain_helper() on this beat. */
    const uint32_t gc = priv->evt.gc_last;
    const uint32_t hb_now = k_cycle_get_32();

    /* Count this beat first, before anything can block or return. */
    if (priv->diag.hb_beats != 0U) {
        const uint32_t gap = k_cyc_to_ms_near32(hb_now - priv->diag.hb_last_t);

        if (gap > priv->diag.hb_gap_ms_max) {
            priv->diag.hb_gap_ms_max = gap;
        }
    }
    priv->diag.hb_last_t = hb_now;
    priv->diag.hb_beats++;

    /*
     * Sum of the fault counters. If it has not changed, the stats line is
     * skipped until the force interval. Printing it takes a long time.
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
         * Lowest free stack seen so far on the event thread. It shows whether
         * UDC_DWC3_EVT_STACK_SIZE is enough. Printed only when it can be
         * measured.
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
         * Print only non-zero counters, to keep the line short.
         * Abbreviations: lt late, gu gaveup, sk skipped/refuted, ms missed/frozen,
         * mz midzero, sf startfail, swd setup-watchdog, kk drain kicks,
         * sp setup-pending, sr stale control buffers returned, st control stalls,
         * eh endpoint halts, sm TRB stomps, rc control reclaims, om unaligned OUT,
         * mu multi-word give-ups, la look-ahead skips, dr drain-dead reconnects,
         * cu commands left UNKNOWN, xn XferNotReady acted on,
         * pf failed posts (not in stats_sig), oN/iN endpoint arms/retires.
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
            _P(priv->diag.evt_lookahead_short, " la%u", priv->diag.evt_lookahead_short);
            _P(priv->diag.drain_dead_resets, " dr%u", priv->diag.drain_dead_resets);
            _P(priv->diag.ep_cmd_unknown, " cu%u", priv->diag.ep_cmd_unknown);
            _P(priv->diag.xnrdy_acted, " xn%u", priv->diag.xnrdy_acted);
            _P(priv->diag.post_fail_total, " pf%u", priv->diag.post_fail_total);

            /*
             * Arm and retire counts for every non-control endpoint that was
             * ever armed. An endpoint that stops shows as a count that stops.
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
     * Settle what a lost event left open. This reads DEPCMD and the TRBs,
     * not events.
     */
    udc_lock_internal(dev, K_FOREVER);
    udc_dwc3_recover_all(dev);
    udc_unlock_internal(dev);

    const uint32_t idle_ms =
        k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt.worker_exit_t0);
    /* Nothing taken from the ring since the previous beat. */
    const uint32_t taken = priv->evt.handled + (priv->evt.q_tail - priv->evt.q_head);
    const bool     none_taken = (taken == priv->evt.hb_last_taken);

    if (gc > 0U && none_taken) {
        priv->evt.hb_stuck_beats++;
    } else {
        priv->evt.hb_stuck_beats = 0U;
    }
    priv->evt.hb_last_taken = taken;

#ifdef UDC_DWC3_CONTROLLER_RECOVER
    if (priv->run.state == UDC_DWC3_RUN_RESET_OWED) {
        udc_dwc3_controller_recover(dev, "PHY erratic error");
        return;
    }

    /*
     * The drain is dead. Events are pending but none were taken for
     * UDC_DWC3_HB_DRAIN_DEAD_MS, even after kicks. Reconnect through
     * the controller recovery. The beat count is cleared first, so a failed
     * reconnect does not retry on the very next beat.
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

    /* Report the bus and DMA configuration once. */
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

    /* Periodic core debug registers. */
    if (++priv->diag.core_dbg_beats >= UDC_DWC3_CORE_DBG_BEATS) {
        struct udc_dwc3_core_dbg dbg;
        bool                     changed;

        priv->diag.core_dbg_beats = 0U;
        priv->diag.core_dbg_quiet += UDC_DWC3_CORE_DBG_BEATS;
        udc_dwc3_core_dbg_read(base, &dbg);

        /*
         * Log only on change, or after the force interval. A real change,
         * such as LTSSM dropping to zero when the core dies, shows at once.
         *
         * Do not return from here. The checks below must still run.
         */
        changed = memcmp(&dbg, &priv->diag.core_dbg_last, sizeof(dbg)) != 0;
        if (changed || priv->diag.core_dbg_quiet >= UDC_DWC3_EVT_STATS_FORCE_BEATS) {
            priv->diag.core_dbg_last = dbg;
            priv->diag.core_dbg_quiet = 0U;
            udc_dwc3_core_dbg_log(" hb", &dbg);

            /*
             * Beats run and timer expiries since boot, beats dropped, and the
             * worst beat gap and work-queue delay so far.
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
    } else if (gc > 0U && priv->evt.drain.state != UDC_DWC3_DRAIN_WAITING && none_taken &&
           priv->evt.q_tail - priv->evt.q_head < UDC_DWC3_EVQ_NUM) {
        /*
         * Events are pending and none were taken since the last beat. The
         * drain is not waiting on a late slot, and the FIFO is not full.
         * So the drain has not run. Log the real idle time.
         */
        LOG_ERR_RATELIMIT("%u B pending, drain IDLE %u ms with no stall "
            "run: slot %u holds 0x%08x, DSTS=0x%08x",
            gc, idle_ms, priv->evt.next,
            cfg->evt_buf[priv->evt.next],
            sys_read32(base + UDC_DWC3_DSTS));

        /* Wake the drain thread. */
        k_sem_give(&priv->evt_sem);
    }

    udc_dwc3_ctrl_setup_wd_check(dev);

    /* No re-arm here. The timer is periodic. */
}

/*
 * One step of a quiesce wait, used by the controller recovery and by
 * udc_dwc3_disable(). The caller holds the UDC mutex.
 *
 * The wait needs the End Transfer completions. Their events may be lost, or held
 * back by the mutex, so the outcomes are read from DEPCMD instead
 * (udc_dwc3_ep_resolve_cmd()). A Start that completed leaves a running transfer,
 * and it is ended here (4.1.8). EP0 continues its return to Setup. The STOPPING
 * state blocks any new Start or Update meanwhile.
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
 * Controller recovery. This is the only path that resets the controller.
 *   1. Device-initiated disconnect (SPEC 3.30b 4.1.8). End every active
 *      transfer, clear RunStop and wait for DevCtrlHlt.
 *   2. Core soft reset.
 *   3. Reconnect as after power-on (4.1.9 -> 4.1.1).
 * It runs from the heartbeat work, because it sleeps. It is used for an erratic
 * error (3.3.2: "Software must reset the controller") or an event ring that has
 * stopped advancing.
 */
static UDC_DWC3_COLD void udc_dwc3_controller_recover(const struct device *const dev,
                       const char *const reason)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t              base = DEVICE_MMIO_NAMED_GET(dev, base);
    const uint32_t              gsts = sys_read32(base + UDC_DWC3_GSTS);
    bool                        reconnected = false;
    int                         ret;

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
     * The STOPPING state stops the endpoint workers from starting new transfers.
     * The lock is released so the drain can deliver the End Transfer events.
     * Each transfer state change wakes this thread, until no transfer is left
     * and EP0 is at Setup. The event ring itself may have failed, so each
     * wake-up (and at least every UDC_DWC3_QUIESCE_SETTLE_MS) also reads the
     * outcomes from DEPCMD (udc_dwc3_quiesce_settle()). The wait is bounded.
     * After it, the device is stopped anyway.
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
     * STEP 2. Clear RunStop only. The interrupt stays unmasked, unlike in
     * udc_dwc3_disable(). The controller reaches DEVCTRLHLT only after its
     * events are acknowledged.
     */
    udc_dwc3_dctl_update(base, UDC_DWC3_DCTL_RUNSTOP, 0U);

    udc_unlock_internal(dev);

    /*
     * STEP 3. Wait for the halt with the mutex released, so the drain thread
     * can acknowledge the events already written. The core is reset only after
     * this wait.
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
     * STEP 4. Re-initialise, now that the controller is halted. The drain is
     * blocked first, before the mutex is taken. This keeps old events from
     * being dispatched into the new session. The drain stays blocked
     * (RESETTING) until udc_dwc3_enable() runs or this function ends.
     * udc_dwc3_on_soft_reset() resets the ring and the drain state together.
     * The mutex is held through udc_dwc3_init(), which sleeps across the PHY
     * reset. The controller is stopped, so nothing waiting on the mutex is lost.
     */
    (void)udc_dwc3_evt_block(dev);
    udc_lock_internal(dev, K_FOREVER);

    udc_dwc3_disable(dev);

    /*
     * udc_dwc3_disable() leaves the control endpoints enabled, so shut down
     * as well. The stack may have disabled or shut down the device while the
     * mutex was released. Bring the controller back only as far as the stack
     * has it (initialised, enabled).
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
            } else {
                reconnected = true;
            }
        }
    }

    /*
     * If the device did not reconnect, throw away any events still in the
     * FIFO. They belong to the old connection. After a reconnect, keep them:
     * they may already be from the new one.
     */
    if (!reconnected) {
        priv->evt.handled += priv->evt.q_tail - priv->evt.q_head;
        priv->evt.q_head = priv->evt.q_tail;
    }

    /*
     * Set RUN on every exit. If the device did not reconnect, the controller
     * is left as udc_dwc3_disable() leaves it, with RunStop clear. A later
     * enable then starts from a known state.
     */
    priv->run.state = UDC_DWC3_RUN;

    udc_unlock_internal(dev);
}
#endif /* UDC_DWC3_CONTROLLER_RECOVER */

/*
 * EP0 SETUP check. It only reports, and never recovers.
 *
 * The control path is driven by events. XferNotReady says which phase the host
 * wants (4.4.1/4.4.2). SetupPending on a completion reports an abandoned
 * transfer. The next SETUP resyncs everything. A stage the host stops using is
 * idle, not stuck. No command fixes a controller that posts no events, so this
 * check only logs.
 *
 * Once per armed SETUP, it reports a SETUP that sits in the RxFIFO but has not
 * been delivered.
 */
static UDC_DWC3_COLD void udc_dwc3_ctrl_setup_wd_check(const struct device *const dev)
{
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    const struct udc_dwc3_config *const cfg = dev->config;
    const mm_reg_t                      base = DEVICE_MMIO_NAMED_GET(dev, base);
    struct udc_dwc3_ep_data *const      ep0_out = &cfg->ep_data_out[0];
    uint32_t                            dsts;
    uint32_t                            trb_ctrl;
    bool                                machine_owns;

    if (priv->diag.watchdog_type != UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
        priv->diag.ctrl_setup_wd_beats = 0U;
        return;
    }

    /*
     * Age the armed SETUP in heartbeats. A new SETUP restarts the count. So
     * does each check once the threshold is reached.
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
     * The RxFIFO is shared with every bulk OUT endpoint, so data in it is not
     * enough. Report only if nothing at all has retired since the SETUP was
     * armed.
     */
    if (priv->diag.ctrl_setup_done != priv->diag.ctrl_setup_wd_snap_setup ||
        priv->diag.nonctrl_done != priv->diag.ctrl_setup_wd_snap_nonctrl) {
        priv->diag.ctrl_setup_wd_snap_setup = priv->diag.ctrl_setup_done;
        priv->diag.ctrl_setup_wd_snap_nonctrl = priv->diag.nonctrl_done;
        return;
    }

    /* Read under the mutex, because other threads post commands. */
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

    /* Dump the core's state, read-only. */
    udc_dwc3_core_state_dump(dev);
}

/* Event buffer constraint (GEVNTSIZ): at least 32 bytes. */
BUILD_ASSERT(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) >= 32,
         "DWC3 event buffer must be at least 32 bytes");
/* The event FIFO indices run free, so the size must be a power of two. */
BUILD_ASSERT(IS_POWER_OF_TWO(UDC_DWC3_EVQ_NUM), "UDC_DWC3_EVQ_NUM must be a power of two");
/* The event FIFO must hold a whole pass. */
BUILD_ASSERT(UDC_DWC3_EVQ_NUM >= CONFIG_UDC_DWC3_EVENTS_NUM,
         "evt.q must hold a full drain: udc_dwc3_evt_drain() clamps 'want' "
         "to CONFIG_UDC_DWC3_EVENTS_NUM");

/*
 * The databook allows up to 64KB (GEVNTSIZ.EVENTSIZ is a 16-bit byte count).
 * The 64-byte cap is this design's. The message gives the reasons.
 */
BUILD_ASSERT(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) <= 64,
         "DWC3 event ring is capped by the AXI block on this part, and "
         "a pass's copy in the event FIFO stays within 64 bytes");

/*
 * Number of fast polls for the event word. UDC_DWC3_EVT_ARRIVE_MAX_MS also caps
 * the wait in time.
 */
#define UDC_DWC3_EVT_ARRIVE_FAST_POLLS      16u
/*
 * Generic command 02h, Set Periodic Parameters. 3.2.1 Table 3-2: "the
 * controller does not use the programmed value", so issuing it has no side
 * effect.
 */
#define UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS  0x02u

/*
 * Issue a force only while the ring is at most half full, leaving room for the
 * forced event and for ongoing link events.
 */
#define UDC_DWC3_EVT_FORCE_MAX_GEVNTCOUNT           \
    ((CONFIG_UDC_DWC3_EVENTS_NUM / 2u) * sizeof(uint32_t))
/* Minimum gap between forced commands, so a stuck slot does not flood commands. */
#define UDC_DWC3_EVT_FORCE_MIN_GAP_MS   500u

/* Fast phase: busy-wait between two reads of the event word. */
#define UDC_DWC3_EVT_ARRIVE_POLL_US     1u

/*
 * Make the controller write an event, to release a slot that does not fill on
 * its own. See UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS for the choice of command.
 * The command is not waited for, and its completion event is ignored.
 */
static UDC_DWC3_COLD void udc_dwc3_evt_force(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    /*
     * Only while the ring has room. Each forced command adds an event behind
     * the stuck slot.
     */
    if (priv->evt.gc_last > UDC_DWC3_EVT_FORCE_MAX_GEVNTCOUNT) {
        return;
    }

    /* Skipped if a generic command is still executing. */
    if (udc_dwc3_dgcmd(dev, UDC_DWC3_DGCMD_SET_PERIODIC_PARAMS | UDC_DWC3_DGCMD_IOC,
               0U, false) != 0) {
        return;
    }

    /*
     * Logged after the command is issued. With CONFIG_LOG_MODE_MINIMAL the
     * line goes to the UART in this context, which takes a few ms.
     */
    LOG_WRN_RATELIMIT("evNUDGE s%u",
              priv->evt.next);
}

/*
 * Issue the NUDGE that the drain requested. The drain queues this work instead
 * of writing DGCMD itself. Rate-limited by UDC_DWC3_EVT_FORCE_MIN_GAP_MS.
 */
static void udc_dwc3_nudge_worker(struct k_work *const work)
{
    struct udc_dwc3_data *const priv =
        CONTAINER_OF(work, struct udc_dwc3_data, nudge_work);
    const struct device *const  dev = priv->dev;

    if (k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt.force_t0) <
                    UDC_DWC3_EVT_FORCE_MIN_GAP_MS) {
        return;
    }

    udc_dwc3_evt_force(dev);
    priv->evt.force_t0 = k_cycle_get_32();
}

/*
 * Restart the drain if it has stopped while events are still owed. That is,
 * the controller still counts events, or events wait in the event FIFO (a
 * reset/error post did not fit, and this is its retry). This runs from the
 * heartbeat timer in ISR context, so it does nothing more.
 */
static UDC_DWC3_COLD void udc_dwc3_drain_helper(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t              base = DEVICE_MMIO_NAMED_GET(dev, base);

    /*
     * The heartbeat's only GEVNTCOUNT read. The heartbeat worker and
     * udc_dwc3_evt_force() use this saved value.
     */
    priv->evt.gc_last = udc_dwc3_gevntcount(base);

    if (priv->evt.gc_last == 0U && priv->evt.q_head == priv->evt.q_tail) {
        return;
    }

    if (k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt.worker_exit_t0) >=
                    UDC_DWC3_EVT_IDLE_KICK_MS) {
        priv->diag.evt_kick++;
        k_sem_give(&priv->evt_sem);
    }
}

/*
 * Event drain
 *
 * The controller writes events to a ring in shared memory. The interrupt
 * only signals that events are waiting. The drain thread copies them out
 * and dispatches them.
 */

/*
 * Wait briefly for an empty slot that GEVNTCOUNT says is owed. This is a short
 * busy-wait. Longer waits happen between passes, so a pass never sleeps.
 * Returns UDC_DWC3_WAIT_ARRIVED with *evt_out set, or UDC_DWC3_WAIT_EXPIRED.
 */
static enum udc_dwc3_wait_result udc_dwc3_evt_wait_first(const struct device *const dev,
                             const uint32_t gc,
                             uint32_t evt_idx,
                             uint32_t *const evt_out)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    const uint32_t                      t0 = k_cycle_get_32();
    const uint32_t                      deadline =
        t0 + k_ms_to_cyc_ceil32(UDC_DWC3_EVT_ARRIVE_MAX_MS);
    uint32_t                            polls = 0;
    uint32_t                            evt;
    int32_t                             wait_cycles;
    enum udc_dwc3_wait_result           wait_result = UDC_DWC3_WAIT_EXPIRED;

    /* Bounded busy-wait, polling every microsecond. */
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

    /* Add up the time spent looking. Report a missed event once per episode. */
    uint32_t waited_us = k_cyc_to_us_near32(k_cycle_get_32() - t0);
    priv->evt.drain.watched_us += waited_us;
    if (!priv->evt.drain.counted &&
         k_cyc_to_ms_near32(k_cycle_get_32() - priv->evt.drain.since) >= UDC_DWC3_EVT_MISSED_MS)
    {
         const uint32_t gc_now = gc;    /* this pass's single read   */
         const bool     frozen = (gc_now == priv->evt.drain.gc0);  /* vs episode open */
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
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    uint32_t                            rescued = 0U;
    bool                                ctrl_rearmed = false;

    udc_lock_internal(dev, K_FOREVER);

    for (uint32_t pass = 0U; pass < 2U; pass++) {
        struct udc_dwc3_ep_data *const eps =
            (pass == 0U) ? cfg->ep_data_in : cfg->ep_data_out;
        const uint32_t                 n = (pass == 0U) ? cfg->num_in_eps : cfg->num_out_eps;

        for (uint32_t i = 0U; i < n; i++) {
            struct udc_dwc3_ep_data *const e = &eps[i];

            if (!e->cfg.stat.enabled) {
                continue;
            }

            /*
             * Non-control: retire every TRB the controller has written
             * back. A lost event does not affect the TRB, so this is enough.
             */
            if (USB_EP_GET_IDX(e->cfg.addr) != 0U) {
                rescued += udc_dwc3_drain_completed(dev, e);
                continue;
            }

            /*
             * EP0: re-arm SETUP only if the control machine is idle and no
             * SETUP is armed. In any other state a control transfer is in
             * progress. It is left to finish, or to the control recovery.
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
 * them. Returns the number skipped. The caller adds it to the pass's single
 * GEVNTCOUNT credit. This function touches no register.
 *
 * A full ring is not a dead slot. The controller holds events internally and
 * writes them once software frees space (see udc_dwc3_evt_drain()).
 */
static UDC_DWC3_COLD uint32_t udc_dwc3_evt_skip_dead_slot(const struct device *const dev,
                    const uint32_t gc, const bool frozen,
                    const uint32_t gaveup_ms)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);

    /* The caller has decided the slot is dead. Count the words owed. */
    uint32_t owed = gc / sizeof(uint32_t);
    if (owed > (CONFIG_UDC_DWC3_EVENTS_NUM - 1u)) {
        owed = CONFIG_UDC_DWC3_EVENTS_NUM - 1u;
    }

    /*
     * How many slots to skip. The controller fills the ring in order.
     *   - A written slot at P+j: the j slots before it are lost. Skip exactly
     *     those. The written slot is read normally.
     *   - No written slot in the owed range: the words may still be in flight.
     *     Skip all but the last, and hold that one back.
     *
     * The held slot is a check. If it fills, the skipped events were late, not
     * lost, and the skip was wrong. udc_dwc3_copy_valid_event() counts that in
     * evt_skip_refuted.
     */
    uint32_t skip = 1;
    bool     held_back = false;

    if (owed > 1)
    {
        uint32_t j = 1;
        bool     slot_valid = false;
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
     * Act first and log after, because the log line is slow. The skipped slots
     * are already empty, so they are not written. Writing them could erase an
     * event that has just landed.
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
         * Watch only a held-back slot. Otherwise the next slot is where the
         * controller writes next, and a new event there proves nothing.
         */
        priv->evt.drain.skip_watch      = held_back;
        priv->evt.drain.skip_watch_slot = priv->evt.next;

        /*
         * Fields: h = slot value, skip = slots skipped, tot = total skipped,
         * age = wall-clock time, w = time actually spent looking.
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
    struct udc_dwc3_data *const         priv = udc_get_private(dev);

    if (cfg->evt_buf[priv->evt.next] != UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE) {
        return false;
    }

    /*
     * A written slot ahead of this one proves the event is lost, because the
     * controller fills the ring in order. This test has a shorter floor.
     * GEVNTCOUNT alone cannot tell a late event from a lost one.
     */
    uint32_t wait_tims_ms = priv->evt.drain.watched_us / 1000U;
    if ((wait_tims_ms >= UDC_DWC3_EVT_LOOKAHEAD_MIN_MS) &&
        (true == udc_dwc3_evt_lookahead_lost(dev, gc)))
    {
        priv->diag.evt_lookahead_short++;
        return true;
    }

    /* Without proof, use the timeouts, above the minimum floor. */
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
 * Take one event out of the ring. Copy it to the event FIFO, copy_idx places
 * past the tail, and mark the slot empty. The pass publishes the copies later.
 * The caller advances evt_idx.
 *
 * The slot is marked empty now, not at the GEVNTCOUNT credit at the end of the
 * pass. So the next lap sees correctly whether the slot was written.
 *
 * This also ends an arrival wait, if one came before this event. That keeps
 * the wait's counters from carrying over to the next event.
 */
static void udc_dwc3_copy_valid_event (struct udc_dwc3_data *const priv,
                                       const struct udc_dwc3_config *const cfg,
                                       uint32_t evt_idx, uint32_t evt, uint32_t copy_idx)
{
    priv->evt.q[(priv->evt.q_tail + copy_idx) % UDC_DWC3_EVQ_NUM] = evt;
    cfg->evt_buf[evt_idx % CONFIG_UDC_DWC3_EVENTS_NUM] = UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;

    /*
     * If the held-back slot filled, the skipped events were late, not lost.
     * The watch ends on the first event either way.
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
        /* Record how long the slot stayed empty. */
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
 * Copy the ring into the event FIFO. Return all slots in one GEVNTCOUNT write,
 * then publish the copies (q_tail). No event is dispatched here. If a dead slot
 * was skipped, finish what its lost event would have completed.
 *
 * One write for all events also clears an overflow: "software must free up
 * space in the Event Buffer by acknowledging more than 1 event".
 */
static void udc_dwc3_evt_drain(const struct device *const dev, const uint32_t cap)
{
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_dwc3_data *const         priv = udc_get_private(dev);
    const mm_reg_t                      base = DEVICE_MMIO_NAMED_GET(dev, base);
    enum udc_dwc3_drain_state           evt_drain_state;

    /* The pass's only GEVNTCOUNT read, before any event is processed. */
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

    /* No more than the FIFO has room for; the rest stay in the ring. */
    if (want > cap) {
        want = cap;
    }

    /* Start at the ring head. */
    uint32_t n       = 0;   /* valid events read in this pass */
    uint32_t skipped = 0;   /* dead slots stepped over in this pass */
    uint32_t evt_idx = priv->evt.next;
    bool     exit_loop   = false;
    evt_drain_state  = priv->evt.drain.state;

    /*
     * Read until every owed slot is read or skipped, or the pass stops.
     * Skipped slots count too, so the pass never waits on a slot beyond
     * 'want'.
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
                     * GEVNTCOUNT have not landed yet. A later pass picks
                     * them up.
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

            /* On the first attempt of an episode only. */
            if (0 == priv->evt.drain.attempts) {
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
                /*
                 * The slot is still empty, so end this pass and attempt. The
                 * next interrupt, or the heartbeat's kick, starts the next one.
                 */
                exit_loop = true;
                priv->evt.drain.attempts++;

                /*
                 * After the first failed attempt, request a NUDGE, because
                 * the event may be stuck in the bus FIFO.
                 * udc_dwc3_nudge_worker() issues it.
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
                     * The NUDGE did not help. Skip the slot if it is dead,
                     * that is, empty too long or a later slot is written.
                     */
                    priv->evt.next = evt_idx % CONFIG_UDC_DWC3_EVENTS_NUM;
                    if (udc_dwc3_drain_slot_is_dead(dev, gc))
                    {
                        /*
                         * Step over the slot and add its words to this pass's
                         * credit. This only moves the read pointer.
                         */
                        skipped += udc_dwc3_evt_skip_dead_slot ( dev, gc,
                            (gc == priv->evt.drain.gc0), age_ms );
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
     * It is written even when zero, because that clears EVNT_HANDLER_BUSY.
     */
    udc_dwc3_gevntcount_ack (base, (n + skipped));

    /* A pass that consumed everything ends IDLE. */
    if (priv->evt.drain.state == UDC_DWC3_DRAIN_RUNNING && (n + skipped) >= want) {
        priv->evt.drain.state = UDC_DWC3_DRAIN_IDLE;
    }

    /* Publish the copies. */
    priv->evt.q_tail += n;

    /*
     * An event was lost (a slot was skipped), so finish the transfers it would
     * have completed by checking the TRB rings. This is done last, after all
     * ring updates are saved, because it takes the mutex and may wait for it.
     */
    if (skipped > 0U) {
        udc_dwc3_evt_reconcile_endpoints(dev);
    }
}


static void udc_dwc3_event_drain_once(const struct device *const dev);

/*
 * Event drain thread, woken by evt_sem.
 *
 * It has its own thread, separate from the UDC work queue, so new events are
 * always taken from the ring promptly, even while the endpoint or heartbeat
 * work holds the mutex. Section 3.2.2.5: "Software must always service the
 * event interrupts generated by the controller."
 */
static void udc_dwc3_event_thread(void *const p1, void *const p2, void *const p3)
{
    struct udc_dwc3_data *const priv = p1;
    const struct device *const  dev = priv->dev;

    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    for (;;) {
        k_sem_take(&priv->evt_sem, K_FOREVER);
        udc_dwc3_event_drain_once(dev);
    }
}

/*
 * One pass of the event drain. First copy the new events out of the ring and
 * tell the controller they are taken; this part never takes the mutex or
 * sleeps. Then handle the copied events with the mutex held.
 *
 * Does nothing while the ring is being reset. If an event the controller
 * counted has not been written yet, the pass ends and the next pass (the next
 * interrupt or the heartbeat) picks it up.
 */
static void udc_dwc3_event_drain_once(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    uint32_t                    room;

    if (priv->run.state == UDC_DWC3_RUN_RESETTING) {
        return;
    }

    /* Take events only into free FIFO space; a full FIFO leaves them in the ring. */
    room = UDC_DWC3_EVQ_NUM - (priv->evt.q_tail - priv->evt.q_head);
    if (room > 0U) {
        priv->diag.evt_worker_runs++;
        udc_dwc3_evt_drain(dev, room);
    } else {
        LOG_ERR_RATELIMIT("event FIFO full: dispatch blocked by a reset/error post");
    }

    if (priv->evt.q_head != priv->evt.q_tail) {
        priv->diag.dispatch_t0 = k_cycle_get_32();
        udc_lock_internal(dev, K_FOREVER);
        /*
         * If the controller has been stopped, throw these events away. They
         * belong to the old connection, and handling them would touch
         * endpoints that are already reset.
         *
         * Stopped means the stack has disabled the device, or the recovery has
         * cleared RunStop. Early in the recovery RunStop is still set and its
         * End Transfer events must be handled, so the register is checked.
         */
        const bool stopped = !udc_dwc3_stack_enabled(dev) ||
            (priv->run.state == UDC_DWC3_RUN_STOPPING &&
             (sys_read32(DEVICE_MMIO_NAMED_GET(dev, base) + UDC_DWC3_DCTL) &
              UDC_DWC3_DCTL_RUNSTOP) == 0U);

        /*
         * Keep the indices in locals and write them back once, to save a load
         * and store per event. Nothing else moves them while the mutex is held.
         */
        const uint32_t tail = priv->evt.q_tail;
        uint32_t head = stopped ? tail : priv->evt.q_head;

        /* Stop at a reset/error that did not fit. The next pass retries it. */
        while (head != tail &&
               udc_dwc3_dispatch_event(dev, priv->evt.q[head % UDC_DWC3_EVQ_NUM])) {
            head++;
        }
        priv->evt.handled += head - priv->evt.q_head;
        priv->evt.q_head = head;
        priv->diag.dispatch_evt = 0U;
        udc_unlock_internal(dev);
    }

    /* FIFO full: stay masked, and let the heartbeat retry. */
    if (priv->evt.q_tail - priv->evt.q_head >= UDC_DWC3_EVQ_NUM) {
        return;
    }

    /*
     * Unmask. Events still owed raise the interrupt again and start the next
     * pass. The heartbeat (udc_dwc3_drain_helper()) is the backstop.
     */
    udc_dwc3_evt_irq(dev, true);

    /* Record when the pass finished (see UDC_DWC3_EVT_IDLE_KICK_MS). */
    priv->evt.worker_exit_t0 = k_cycle_get_32();
}

/*
 * Event interrupt: wake the drain thread and mask the interrupt.
 */
static void udc_dwc3_irq_handler(void *const ptr)
{
    const struct device *const  dev = ptr;
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    priv->diag.evt_isr++;

    k_sem_give(&priv->evt_sem);

    /* Disable further interrupts until all events are processed */
    udc_dwc3_evt_irq(dev, false);
}

/*
 * UDC API
 *
 * The functions the Zephyr USB stack calls through udc_dwc3_api, and the
 * endpoint work item that arms queued buffers.
 */

static int udc_dwc3_ep_enqueue(const struct device *const dev,
                   struct udc_ep_config *const ep_cfg,
                   struct net_buf *const buf)
{
    struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
    const struct udc_buf_info      bi = *udc_get_buf_info(buf);

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
         * Let the worker arm this buffer with the others waiting. The worker
         * checks, under the mutex, whether a transfer may start.
         */
        if (ep_cfg->stat.enabled) {
            LOG_DBG("submitting to EP%02x", ep_cfg->addr);
            k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
        }
    }

    return 0;
}

/*
 * UDC API: cancel queued buffers on an endpoint. The endpoint recovery returns
 * the buffers on the ring once the controller has released them.
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
 * Set up an endpoint: configure it (DEPCFG Init), enable it in DALEPENA and arm
 * whatever is queued. Called from udc_dwc3_ep_enable(), or later from
 * udc_dwc3_ep_recover() when deferred. The stack never enables an endpoint that
 * is already enabled, so DEPCFG Modify is not used.
 */
static UDC_DWC3_COLD int udc_dwc3_ep_resume(const struct device *const dev,
                  struct udc_dwc3_ep_data *const ep_data)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct net_buf             *buf;
    int                         ret;

    /*
     * Defer the resume (PEND_RESUME, run later by udc_dwc3_ep_recover()) when:
     *   - The endpoint is not IDLE. A transfer or a command is still open, and
     *     the controller owns the ring (3.2.2.7). Its completion, or the
     *     heartbeat, runs the recovery again, which ends the transfer first.
     *   - The controller is stopping or resetting. 4.1.8 allows no new transfer
     *     once transfers are being ended.
     */
    if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 &&
        (ep_data->xfer.state != UDC_DWC3_EP_IDLE || udc_dwc3_run_halting(priv))) {
        LOG_DBG("EP%02x %s%s, deferring resume", ep_data->cfg.addr,
            udc_dwc3_ep_state_name(ep_data->xfer.state),
            udc_dwc3_run_halting(priv) ? ", controller stopping" : "");
        ep_data->xfer.pending |= UDC_DWC3_EP_PEND_RESUME;
        return 0;
    }

    /* Clear any halt on a non-control endpoint. */
    if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
        udc_dwc3_depcmd_clear_stall(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
    }

    udc_dwc3_depcmd_ep_config(dev, ep_data, false);

    /*
     * INVARIANT 4: DEPXFERCFG only on enable, and only once per pool generation.
     * Equal epochs mean this endpoint already holds a resource.
     *
     * An alt-setting switch disables and re-enables endpoints without a
     * DEPSTARTCFG. End Transfer does not return the resource. Without this
     * check, each switch would take another resource until Start Transfer
     * fails with CmdStatus 4'h1 (no resource available).
     */
    if (ep_data->rsc_epoch != priv->epcfg.epoch) {
        udc_dwc3_depcmd_ep_xfer_config(dev, ep_data);
        ep_data->rsc_epoch = priv->epcfg.epoch;
    }

    if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
        /*
         * Rebuild the ring from empty. Buffers still on it are parked first and
         * re-armed below. The endpoint is IDLE, so the controller owns none of
         * them.
         */
        udc_dwc3_ep_ring_release(ep_data);
        ret = udc_dwc3_trb_nonctrl_init(dev, ep_data);
        if (ret != 0) {
            return ret;
        }
    }

    /* From here on, the endpoint can be used. */
    udc_dwc3_dalepena_set(dev, ep_data->epn, true);

    /*
     * Re-arm the parked buffers. They come from a refused Start or from the
     * ring release above. Peek, arm, then remove, as udc_dwc3_ep_worker() does.
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
            /* Not armed. It stays in the FIFO for the next resume. */
            return ret;
        }

        (void)k_fifo_get(&ep_data->requeue_fifo, K_NO_WAIT);
    }

    /* Let the worker arm buffers that waited while the endpoint was down. */
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
    struct udc_dwc3_data *const    priv = udc_get_private(dev);

    LOG_DBG("EP%02x, first EP%02x", ep_data->cfg.addr, priv->epcfg.first_ep);

    /*
     * Refuse isochronous endpoints, which this driver does not support. The
     * capability is not advertised (see caps.iso); this catches a class that
     * uses one anyway.
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
        /*
         * Reassign the transfer-resource pool once per bus reset. DEPSTARTCFG
         * resets every endpoint, so running it again on a SET_INTERFACE that
         * touches first_ep would tear down streaming endpoints. If it was
         * skipped or refused, the pool keeps its assignment and the next bus
         * reset tries again.
         */
        if (ep_cfg->addr == priv->epcfg.first_ep && !priv->epcfg.pool_assigned) {
            priv->epcfg.pool_assigned = true;
            udc_dwc3_on_set_config_or_interface(dev);
        }
    }

    return udc_dwc3_ep_resume(dev, ep_data);
}

/*
 * UDC API: disable an endpoint. Clear DALEPENA, then the endpoint recovery ends
 * any transfer and returns the armed buffers as cancelled.
 */
static UDC_DWC3_COLD int udc_dwc3_ep_disable(const struct device *const dev,
                    struct udc_ep_config *const ep_cfg)
{
    struct udc_dwc3_ep_data    *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    LOG_DBG("Disabling EP%02x", ep_cfg->addr);

    /* Drop the watchdog's reference to this endpoint. */
    if (priv->diag.watchdog_ep == ep_data) {
        priv->diag.watchdog_ep = NULL;
        priv->diag.watchdog_type = UDC_DWC3_WATCHDOG_TYPE_NONE;
    }

    /*
     * Replace any owed resume or Clear Stall with a dequeue. The recovery then
     * returns the armed buffers as cancelled. udc_ep_dequeue() reaches the
     * driver only for buffers still in the stack's queue, not those on the ring.
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
    struct udc_dwc3_ep_data    *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

    /*
     * Log stack-requested halts, to tell them apart from halts the controller
     * raised itself.
     */
    LOG_INF("Set halt on EP%02x (requested by the stack)", ep_cfg->addr);

    switch (ep_data->cfg.addr) {
    case USB_CONTROL_EP_IN:
    case USB_CONTROL_EP_OUT:
        /* The stack rejected the control request. Run the control recovery. */
        udc_dwc3_ctrl_ep_recover(dev);
        break;
    default:
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
 * UDC API: ClearFeature(ENDPOINT_HALT). The request is recorded and the endpoint
 * recovery carries it out. Returns -EIO if the controller refused the Clear
 * Stall, so udc_common keeps the endpoint halted. Returns 0 when the stall is
 * cleared or waits for an End Transfer to finish.
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
    uint32_t       reg;

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
    const mm_reg_t              base = DEVICE_MMIO_NAMED_GET(dev, base);
    int                         ret;

    LOG_INF("Enabling DWC3 driver");

    ret = udc_dwc3_quirk_enable(dev);
    if (ret != 0) {
        return ret;
    }

    /*
     * Keep U1/U2 off until the host configures the device. The DCTL spec says
     * AcceptU1/U2Ena is set "after receiving a SetConfiguration command".
     * InitU1/U2Ena is set after SetFeature(U1/U2_ENABLE). Both are set in
     * udc_dwc3_ctrl_apply_link_pm().
     */
    udc_dwc3_dctl_update(base,
                 UDC_DWC3_DCTL_ACCEPTU1ENA | UDC_DWC3_DCTL_INITU1ENA |
                 UDC_DWC3_DCTL_ACCEPTU2ENA | UDC_DWC3_DCTL_INITU2ENA, 0U);

    /*
     * Leave STOPPING alone. The controller recovery may be stopping the
     * transfers, and no new transfer may start then. The recovery sets RUN
     * itself when it finishes.
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
 * UDC API: detach from the bus as a device-initiated disconnect (SPEC 3.30b
 * 4.1.8, Table 4-7). The steps are:
 * 1. End every active transfer and take EP0 back to Setup.
 * 2. Clear RunStop.
 * 3. Forget the transfer state and return the buffers.
 *
 * The caller holds the UDC mutex. Event dispatch also needs that mutex, so End
 * Transfer completions cannot arrive as events here. Their outcomes are read
 * from DEPCMD instead (udc_dwc3_quiesce_settle()), with a time limit.
 * DEVCTRLHLT is not waited for. The controller halts only after its events are
 * acknowledged, and the drain may be blocked on this mutex.
 *
 * The controller recovery calls this with RunStop already cleared. Then only
 * step 3 runs.
 */
static int udc_dwc3_disable(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t              base = DEVICE_MMIO_NAMED_GET(dev, base);

    LOG_DBG("Disabling DWC3 driver");

    k_timer_stop(&priv->heartbeat_timer);

    if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0U) {
        const enum udc_dwc3_run_state prev = priv->run.state;
        uint32_t                      polls = 0U;

        /*
         * Cancel any queued heartbeat or nudge. It would act on the stopped
         * device, and a heartbeat could even reconnect it.
         */
        (void)k_work_cancel(&priv->heartbeat_work);
        (void)k_work_cancel(&priv->nudge_work);

        /* STOPPING blocks any new Start or resume (4.1.8). */
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
     * With RunStop clear, the controller sends no more Endpoint Command
     * Complete events. Drop whatever is still outstanding.
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
    uint32_t       reg;
    int            ret;

    LOG_DBG("Initializing the DWC3 core");

    ret = udc_dwc3_quirk_init(dev);
    if (ret != 0) {
        return ret;
    }

    /*
     * Soft-reset the core and the USB2 and USB3 PHYs.
     *
     * There are two waits, and both are needed. The first lets the PHY finish
     * its reset. The second gives the core stable PHY clocks before it leaves
     * reset. A core released onto an unstable PIPE clock reads all registers
     * as zero.
     *
     * The sequence is repeated if the registers do not come back. If they
     * never do, init fails and the device stays off the bus. A core still in
     * reset ignores register writes, so it would never move data.
     */
    for (uint32_t attempt = 1U; ; attempt++) {
        sys_set_bits(base + UDC_DWC3_GCTL, UDC_DWC3_GCTL_CORESOFTRESET);
        /*
         * The core reset clears DALEPENA (3.30b register reset table), so
         * clear the driver's copy too.
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
 * UDC API: bring the controller up and enable the control endpoints. The stack
 * calls it with the UDC mutex held, and so does udc_dwc3_controller_recover().
 * The event drain is blocked throughout, because the reset re-initialises the
 * event ring. The controller is not running yet, so there are no events to
 * miss. The previous run state is restored on return.
 */
static int udc_dwc3_init(const struct device *const dev)
{
    struct udc_dwc3_data *const   priv = udc_get_private(dev);
    const enum udc_dwc3_run_state prev = udc_dwc3_evt_block(dev);
    const int                     ret = udc_dwc3_init_core(dev);

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
 * Endpoint work item: arm the queued buffers, under the UDC mutex.
 */
static void udc_dwc3_ep_worker(struct k_work *const work)
{
    struct udc_dwc3_ep_data *const    ep_data = CONTAINER_OF(work, struct udc_dwc3_ep_data, work);
    const struct device *const        dev = ep_data->dev;
    const struct udc_dwc3_data *const priv = udc_get_private(dev);
    struct net_buf                   *buf;
    int                               ret;

    LOG_DBG("checking for pending transfers for EP%02x", ep_data->cfg.addr);

    /* Hold the UDC mutex, as every other TRB producer does. */
    udc_lock_internal(dev, K_FOREVER);

    /*
     * Start no transfer while the controller is stopping or stopped. 4.1.8 ends
     * every transfer before RunStop is cleared, and commands are undefined
     * while the controller halts. The buffers stay queued, and
     * udc_dwc3_ep_enable() runs this worker again later.
     *
     * The check uses driver state, not RunStop. RunStop is clear only when the
     * stack has not enabled the device, during the controller recovery, or
     * after a failed re-init. In the last case no endpoint is in DALEPENA,
     * which is checked below.
     */
    if (udc_dwc3_run_halting(priv) || !udc_dwc3_stack_enabled(dev)) {
        goto unlock;
    }

    /*
     * Skip a disabled or halted endpoint. This work can still run after
     * udc_dwc3_ep_disable(), and a TRB armed then would never be read.
     * DALEPENA is checked too, because a refused Start or a core reset removes
     * the endpoint from it while the stack still sees it enabled.
     */
    if (!ep_data->cfg.stat.enabled || ep_data->cfg.stat.halted ||
        !udc_dwc3_ep_in_dalepena(priv, ep_data)) {
        LOG_DBG("endpoint is down or halted, not processing buffers");
        goto unlock;
    }

    /*
     * Wait while an End Transfer is pending or a Start/End outcome is unknown.
     * A new TRB would need an Update Transfer on a transfer that is ending or
     * may not exist (3.2.2.7). Once the outcome is known, udc_dwc3_ep_recover()
     * submits this work again.
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
 * Driver instance
 *
 * Pre-init, which runs before the hardware is touched, and the per-instance
 * data and device definition.
 */

/*
 * Set up the controller and endpoint capabilities and register the endpoints.
 * No hardware access yet.
 */
static int udc_dwc3_driver_preinit(const struct device *const dev)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);

    udc_dwc3_stomp_priv = priv;
    const struct udc_dwc3_config *const cfg = dev->config;
    struct udc_data *const              data = dev->data;
    struct udc_dwc3_ep_data            *ep_data;
    uint16_t                            mps = 0;
    int                                 ret;

    ret = udc_dwc3_quirk_preinit(dev);
    if (ret != 0) {
        return ret;
    }

    DEVICE_MMIO_NAMED_MAP(dev, base, K_MEM_CACHE_NONE);

    k_mutex_init(&data->mutex);
    /*
     * Connect the IRQ once, here. The event ring is drained by its own thread,
     * udc_dwc3_event_thread(), created below.
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
     * The loops below skip the control pair, so both halves are set up here.
     * Zero is a valid index, so mark "no index" explicitly. udc_dwc3_ep_resume()
     * reads requeue_fifo on every endpoint, EP0 included.
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
    /* Zero is a valid index, so mark "no index" explicitly. */
    ep_data->xferrscidx = UDC_DWC3_XFERRSCIDX_INVALID;
    ep_data->xfer.end_idx = UDC_DWC3_XFERRSCIDX_INVALID;
    ep_data->epn = 0;

    /* udc_dwc3_ep_resume() reads this queue on every endpoint, EP0 included. */
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
         * non-control endpoint with NORMAL TRBs and no start frame, which is
         * wrong for ISO. Not advertising it makes such a class fail early.
         */
        ep_data->cfg.caps.iso = false;
        ep_data->cfg.caps.mps = mps;
        ep_data->trb_buf = cfg->trb_buf_in[i];
        /* Zero is a valid index, so mark "no index" explicitly. */
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
         * non-control endpoint with NORMAL TRBs and no start frame, which is
         * wrong for ISO. Not advertising it makes such a class fail early.
         */
        ep_data->cfg.caps.iso = false;
        ep_data->cfg.caps.mps = mps;
        ep_data->trb_buf = cfg->trb_buf_out[i];
        /* Zero is a valid index, so mark "no index" explicitly. */
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
 * The event buffer must be aligned to its own size (GEVNTADR: "the lower n bits
 * of the address must be GEVNTSIZn.EVNTSiz-aligned"). That needs a power-of-two
 * size.
 */
BUILD_ASSERT(IS_POWER_OF_TWO(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t)),
         "the event buffer is aligned to its own size, which must be a power of two");

#define UDC_DWC3_DEVICE_DEFINE(n)                       \
    UDC_DWC3_QUIRK_DEFINE(n);                       \
                                        \
    /* Called once from preinit; enable/disable only mask the IRQ. */ \
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

/*
 * Queue types for the "dwc3 fifo" dump. WriteBack/EventQ and DescFetchQ can
 * read 0/0. The queues exist, so the debug register probably does not report
 * these types in this build.
 */
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
    uint32_t       reg;

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
    uint32_t       reg;

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
    uint32_t       reg;

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
    struct udc_dwc3_data         *priv = udc_get_private(dev);

    for (uint32_t i = 0; i < CONFIG_UDC_DWC3_EVENTS_NUM; i++) {
        uint32_t evt = cfg->evt_buf[i];
        char    *s = (i == priv->evt.next) ? "<-" : "  ";

        shell_print(sh, "evt 0x%02x: 0x%08x %s %s",
            i, evt, s, udc_dwc3_get_event_name(evt, 0));
    }

    /* How often an event was read before the controller's write had landed. */
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
        struct udc_dwc3_trb                       trb;

        /* Read ctrl first, as udc_dwc3_trb_snapshot() does. */
        trb.ctrl = t->ctrl;
        trb.status = t->status;
        trb.addr_lo = t->addr_lo;
        trb.addr_hi = t->addr_hi;

        bool     hwo = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_HWO);
        bool     lst = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_LST);
        bool     chn = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_CHN);
        bool     csp = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_CSP);
        bool     isp = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_ISP_IMI);
        bool     ioc = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_IOC);
        bool     spr = !!(trb.status & UDC_DWC3_TRB_STATUS_SPR);
        uint32_t trbctl = FIELD_GET(UDC_DWC3_TRB_CTRL_TRBCTL_MASK, trb.ctrl);
        uint32_t trbsts = FIELD_GET(UDC_DWC3_TRB_STATUS_TRBSTS_MASK, trb.status);
        uint32_t pcm1 = FIELD_GET(UDC_DWC3_TRB_STATUS_PCM1_MASK, trb.status);
        uint32_t sidsofn = FIELD_GET(UDC_DWC3_TRB_CTRL_SIDSOFN_MASK, trb.ctrl);
        uint32_t bufsiz = FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb.status);
        char    *head = (i == ep_data->ring.head) ? " <HEAD" : "";
        char    *tail = (i == ep_data->ring.tail) ? " <TAIL" : "";
        char    *full = (i == ep_data->ring.head &&
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
        uint8_t                  addr = ep_data->cfg.addr;

        shell_print(sh, "%s for IN endpoint 0x%02x (%u %s) xferrscidx=0x%x",
              label, addr, addr & 0x7f, (addr & 0x80) ? "IN" : "OUT",
              ep_data->xferrscidx);
        (*fn)(dev, ep_data, sh);
    }

    for (int i = 0; i < cfg->num_out_eps; i++) {
        struct udc_dwc3_ep_data *ep_data = &cfg->ep_data_out[i];
        uint8_t                  addr = ep_data->cfg.addr;

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

/* Shell: dump the FIFO space counters. */
static void udc_dwc3_dump_fifo_space(const struct device *dev, const struct shell *sh)
{
    struct udc_dwc3_data *const priv = udc_get_private(dev);
    const mm_reg_t              base = DEVICE_MMIO_NAMED_GET(dev, base);
    uint32_t                    num_in_eps;
    uint32_t                    num_eps;
    uint32_t                    total_xfer_resources;
    uint32_t                    avail;
    uint32_t                    reg;

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
 * Run a dwc3 shell command under the UDC mutex. The commands touch the same
 * state as the work queue and the event thread. The mutex is recursive, so a
 * command may also take it itself.
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

    /* The commands assume this driver's config and private data. */
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
 * Shell: inject a synthetic XferComplete on EP0, in the last transfer direction.
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
 * Shell: run the control endpoint recovery (udc_dwc3_recover()).
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
